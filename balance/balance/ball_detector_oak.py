#!/usr/bin/env python3
"""
ball_detector_oak.py  (v2 — ML inference)
------------------------------------------
Ball detector using OAK-D Lite with YOLOv8n inference.
Supports two inference modes via 'inference_mode' parameter.

Inference modes:
  'myriadx'  — runs YOLOv8n blob on MyriadX VPU inside the camera.
               Low host CPU load. OAK-D can be on Pi 5 or Orin Nano.
               Requires: blob_path parameter pointing to .blob file

  'host'     — streams RGB to host, runs ONNX or TensorRT on host GPU.
               Better for Orin Nano with GPU available.
               Requires: onnx_path or engine_path parameter

Publishes (identical interface to USB camera version):
  /ball/position        (geometry_msgs/Point)
                          x,y normalised to cup (-1..+1)
                          z=0 detected, z=-1 not found
  /ball/position_3d     (geometry_msgs/PointStamped)
                          x,y,z metres in camera frame
  /ball/cup_detected    (std_msgs/Bool)
  /ball/image           (sensor_msgs/Image)  annotated debug frame

Parameters:
  inference_mode      'myriadx' or 'host'           default: 'myriadx'
  blob_path           Path to .blob file             default: ''
  onnx_path           Path to .onnx file (host mode) default: ''
  engine_path         Path to TensorRT .engine file  default: ''
  conf_threshold      Detection confidence threshold  default: 0.5
  iou_threshold       NMS IoU threshold               default: 0.45
  input_size          Model input size (px)           default: 416
  rgb_fps             RGB camera framerate            default: 30
  exposure_us         Manual exposure us (0=auto)     default: 8000
  iso                 Manual ISO gain                 default: 400
  camera_frame_id     TF frame for 3D output          default: 'oak_rgb_camera_optical_frame'
  roi_x/y/w/h         Optional ROI crop               default: 0 (disabled)
  depth_min_mm        Min valid depth                 default: 100
  depth_max_mm        Max valid depth                 default: 2000
  show_debug          Annotate published image        default: True
  publish_hz          ROS publish rate                default: 30.0
"""

import os
import cv2
import time
import numpy as np
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, Header
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False

# Class indices must match training order in data.yaml
CLASS_NAMES = ['ball', 'cup']
BALL_IDX    = 0
CUP_IDX     = 1


class BallDetectorOakNode(Node):

    def __init__(self):
        super().__init__('ball_detector_oak')

        if not DEPTHAI_AVAILABLE:
            self.get_logger().error(
                'depthai not installed. '
                'Run: pip install depthai --break-system-packages')
            raise RuntimeError('depthai not available')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('inference_mode',  'myriadx')
        self.declare_parameter('blob_path',       '')
        self.declare_parameter('onnx_path',       '')
        self.declare_parameter('engine_path',     '')
        self.declare_parameter('conf_threshold',   0.5)
        self.declare_parameter('iou_threshold',    0.45)
        self.declare_parameter('input_size',       416)
        self.declare_parameter('rgb_fps',          30)
        self.declare_parameter('exposure_us',      8000)
        self.declare_parameter('iso',              400)
        self.declare_parameter('camera_frame_id',
                               'oak_rgb_camera_optical_frame')
        self.declare_parameter('roi_x', 0)
        self.declare_parameter('roi_y', 0)
        self.declare_parameter('roi_w', 0)
        self.declare_parameter('roi_h', 0)
        self.declare_parameter('depth_min_mm',    100)
        self.declare_parameter('depth_max_mm',   2000)
        self.declare_parameter('show_debug',  True)
        self.declare_parameter('publish_hz',  30.0)

        # ── publishers ────────────────────────────────────────────────────
        self._pub_pos   = self.create_publisher(
            Point,        '/ball/position',     10)
        self._pub_pos3d = self.create_publisher(
            PointStamped, '/ball/position_3d',  10)
        self._pub_cup   = self.create_publisher(
            Bool,         '/ball/cup_detected', 10)
        self._pub_img   = self.create_publisher(
            Image,        '/ball/image',        10)
        self._bridge    = CvBridge()

        # ── shared state ──────────────────────────────────────────────────
        self._latest_rgb   = None
        self._latest_depth = None
        self._latest_dets  = []   # [(label, conf, x1,y1,x2,y2) normalised]
        self._frame_lock   = threading.Lock()
        self._det_lock     = threading.Lock()
        self._running      = True
        self._host_model   = None

        # ── build pipeline (depthai v3 API) ───────────────────────────────
        mode = self.get_parameter('inference_mode').value
        self.get_logger().info(f'Inference mode: {mode}')

        if mode == 'myriadx':
            self._q_rgb, self._q_depth, self._q_det = \
                self._build_myriadx_pipeline_v3()
        else:
            self._q_rgb, self._q_depth = self._build_host_pipeline_v3()
            self._q_det = None
            self._host_model = self._load_host_model()

        # ── grab thread ────────────────────────────────────────────────────
        self._grab_thread = threading.Thread(
            target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        hz = self.get_parameter('publish_hz').value
        self._timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info('ball_detector_oak v2 ready')

    # ── Pipeline builders (depthai v3 API) ────────────────────────────────

    def _build_myriadx_pipeline_v3(self):
        """
        depthai v3 pipeline using Camera node (replaces ColorCamera/MonoCamera).
        DetectionNetwork replaces YoloDetectionNetwork.
        No XLinkOut nodes — createOutputQueue() on node outputs directly.
        """
        blob_path  = self.get_parameter('blob_path').value
        input_size = self.get_parameter('input_size').value

        if not blob_path or not os.path.exists(blob_path):
            raise RuntimeError(
                f'blob_path not found: "{blob_path}"\n'
                f'Set blob_path parameter to your .blob file.')

        pipeline = dai.Pipeline()

        # v3 Camera node replaces ColorCamera
        cam = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A)

        # Full-res RGB output for debug image — STRETCH handles AR mismatch
        rgb_out = cam.requestOutput(
            (1280, 720),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.STRETCH)
        q_rgb = rgb_out.createOutputQueue(maxSize=1, blocking=False)

        # Resized output for NN input
        nn_out = cam.requestOutput(
            (input_size, input_size),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.STRETCH)

        # DetectionNetwork replaces YoloDetectionNetwork in v3
        det = pipeline.create(dai.node.DetectionNetwork)
        det.setBlobPath(blob_path)
        det.setConfidenceThreshold(
            self.get_parameter('conf_threshold').value)
        nn_out.link(det.input)

        q_det   = det.out.createOutputQueue(maxSize=1, blocking=False)
        q_depth = self._add_stereo_v3(pipeline)

        pipeline.start()
        self._pipeline = pipeline
        return q_rgb, q_depth, q_det

    def _build_host_pipeline_v3(self):
        """depthai v3 host pipeline — RGB + depth, inference on host GPU."""
        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_A)
        rgb_out = cam.requestOutput(
            (1280, 720),
            type=dai.ImgFrame.Type.BGR888p,
            resizeMode=dai.ImgResizeMode.STRETCH)
        q_rgb   = rgb_out.createOutputQueue(maxSize=1, blocking=False)
        q_depth = self._add_stereo_v3(pipeline)

        pipeline.start()
        self._pipeline = pipeline
        return q_rgb, q_depth

    def _add_stereo_v3(self, pipeline):
        """Add stereo depth to pipeline using v3 Camera node."""
        fps = self.get_parameter('rgb_fps').value

        # v3: Camera node replaces MonoCamera
        cam_left  = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_B)
        cam_right = pipeline.create(dai.node.Camera).build(
            dai.CameraBoardSocket.CAM_C)

        # OAK-D Lite mono cameras (OV7251) only support 480p or 400p
        left_out  = cam_left.requestOutput(
            (640, 400), type=dai.ImgFrame.Type.GRAY8)
        right_out = cam_right.requestOutput(
            (640, 400), type=dai.ImgFrame.Type.GRAY8)

        stereo = pipeline.create(dai.node.StereoDepth)
        # v3: FAST_DENSITY replaces HIGH_DENSITY
        stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.FAST_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        # Must be multiple of 16 — 640x400 matches mono camera output
        stereo.setOutputSize(640, 400)

        left_out.link(stereo.left)
        right_out.link(stereo.right)

        return stereo.depth.createOutputQueue(maxSize=1, blocking=False)

    def _load_host_model(self):
        engine_path = self.get_parameter('engine_path').value
        onnx_path   = self.get_parameter('onnx_path').value
        from ultralytics import YOLO
        if engine_path and os.path.exists(engine_path):
            self.get_logger().info(f'Loading TensorRT: {engine_path}')
            return YOLO(engine_path)
        elif onnx_path and os.path.exists(onnx_path):
            self.get_logger().info(f'Loading ONNX: {onnx_path}')
            return YOLO(onnx_path)
        else:
            raise RuntimeError(
                'host mode requires engine_path or onnx_path')

    # ── Grab loop ──────────────────────────────────────────────────────────

    def _grab_loop(self):
        while self._running:
            try:
                rgb   = self._q_rgb.tryGet()
                depth = self._q_depth.tryGet()
                with self._frame_lock:
                    if rgb   is not None:
                        self._latest_rgb   = rgb.getCvFrame()
                    if depth is not None:
                        self._latest_depth = depth.getFrame()

                if self._q_det is not None:
                    dets = self._q_det.tryGet()
                    if dets is not None:
                        parsed = [(d.label, d.confidence,
                                   d.xmin, d.ymin, d.xmax, d.ymax)
                                  for d in dets.detections]
                        with self._det_lock:
                            self._latest_dets = parsed
            except Exception as e:
                self.get_logger().warn(
                    str(e), throttle_duration_sec=2.0)
            time.sleep(0.001)

    # ── Main tick ──────────────────────────────────────────────────────────

    def _tick(self):
        with self._frame_lock:
            frame = self._latest_rgb.copy()   if self._latest_rgb   is not None else None
            depth = self._latest_depth.copy() if self._latest_depth is not None else None
        if frame is None:
            return

        mode = self.get_parameter('inference_mode').value
        if mode == 'myriadx':
            with self._det_lock:
                dets = list(self._latest_dets)
            ball_det, cup_det = self._parse_norm(dets, frame.shape)
        else:
            ball_det, cup_det = self._run_host(frame)

        cup_found  = cup_det  is not None
        ball_found = ball_det is not None

        cup_cx = cup_cy = cup_r = None
        if cup_found:
            x1, y1, x2, y2 = cup_det
            cup_cx = (x1 + x2) / 2.0
            cup_cy = (y1 + y2) / 2.0
            cup_r  = max(x2 - x1, y2 - y1) / 2.0

        ball_px = ball_py = None
        if ball_found:
            x1, y1, x2, y2 = ball_det
            ball_px = int((x1 + x2) / 2.0)
            ball_py = int((y1 + y2) / 2.0)

        # Cup detected
        cup_msg      = Bool()
        cup_msg.data = cup_found
        self._pub_cup.publish(cup_msg)

        # Normalised 2D position
        pos_msg = Point()
        if ball_found and cup_found and cup_r and cup_r > 0:
            pos_msg.x = (ball_px - cup_cx) / cup_r
            pos_msg.y = (ball_py - cup_cy) / cup_r
            pos_msg.z = 0.0
        else:
            pos_msg.x = 0.0
            pos_msg.y = 0.0
            pos_msg.z = -1.0
        self._pub_pos.publish(pos_msg)

        # 3D position
        if ball_found and depth is not None and ball_px is not None:
            self._publish_3d(ball_px, ball_py, depth)

        # Debug image
        if self.get_parameter('show_debug').value:
            debug = self._draw_debug(frame, ball_det, cup_det, pos_msg)
        else:
            debug = frame
        img_msg = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        img_msg.header.stamp    = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        self._pub_img.publish(img_msg)

    # ── Detection helpers ──────────────────────────────────────────────────

    def _parse_norm(self, dets, shape):
        h, w = shape[:2]
        best_ball = None; best_ball_c = 0.0
        best_cup  = None; best_cup_c  = 0.0
        for (label, conf, xmin, ymin, xmax, ymax) in dets:
            x1 = int(xmin*w); y1 = int(ymin*h)
            x2 = int(xmax*w); y2 = int(ymax*h)
            if label == BALL_IDX and conf > best_ball_c:
                best_ball_c = conf; best_ball = (x1,y1,x2,y2)
            elif label == CUP_IDX and conf > best_cup_c:
                best_cup_c = conf;  best_cup  = (x1,y1,x2,y2)
        return best_ball, best_cup

    def _run_host(self, frame):
        conf    = self.get_parameter('conf_threshold').value
        imgsz   = self.get_parameter('input_size').value
        results = self._host_model.predict(
            frame, imgsz=imgsz, conf=conf, verbose=False)
        best_ball = None; best_ball_c = 0.0
        best_cup  = None; best_cup_c  = 0.0
        for r in results:
            for box in r.boxes:
                label = int(box.cls); bconf = float(box.conf)
                x1,y1,x2,y2 = [int(v) for v in box.xyxy[0]]
                if label == BALL_IDX and bconf > best_ball_c:
                    best_ball_c = bconf; best_ball = (x1,y1,x2,y2)
                elif label == CUP_IDX and bconf > best_cup_c:
                    best_cup_c = bconf;  best_cup  = (x1,y1,x2,y2)
        return best_ball, best_cup

    def _publish_3d(self, bx, by, depth_frame):
        dh, dw = depth_frame.shape
        if not (0 <= bx < dw and 0 <= by < dh):
            return
        r  = 5
        roi = depth_frame[max(0,by-r):min(dh,by+r),
                          max(0,bx-r):min(dw,bx+r)].astype(float)
        mn = self.get_parameter('depth_min_mm').value
        mx = self.get_parameter('depth_max_mm').value
        valid = roi[(roi > mn) & (roi < mx)]
        if valid.size == 0:
            return
        dm = float(np.median(valid)) / 1000.0
        fx = 1078.0; fy = 1078.0; cx = 640.0; cy = 360.0
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('camera_frame_id').value
        msg.point.x = (bx - cx) * dm / fx
        msg.point.y = (by - cy) * dm / fy
        msg.point.z = dm
        self._pub_pos3d.publish(msg)

    def _draw_debug(self, frame, ball_det, cup_det, pos_msg):
        out = frame.copy()
        if cup_det is not None:
            x1,y1,x2,y2 = cup_det
            cv2.rectangle(out, (x1,y1), (x2,y2), (0,255,0), 2)
            cx = int((x1+x2)/2); cy = int((y1+y2)/2)
            cv2.circle(out, (cx,cy), 3, (0,255,0), -1)
            cv2.line(out,(cx-10,cy),(cx+10,cy),(0,255,0),1)
            cv2.line(out,(cx,cy-10),(cx,cy+10),(0,255,0),1)
            cv2.putText(out,'cup',(x1,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
        if ball_det is not None:
            x1,y1,x2,y2 = ball_det
            cv2.rectangle(out,(x1,y1),(x2,y2),(0,0,255),2)
            bx = int((x1+x2)/2); by = int((y1+y2)/2)
            cv2.circle(out,(bx,by),4,(0,0,255),-1)
            if cup_det is not None:
                cx = int((cup_det[0]+cup_det[2])/2)
                cy = int((cup_det[1]+cup_det[3])/2)
                cv2.line(out,(cx,cy),(bx,by),(0,165,255),1)
            lbl = (f'ball ({pos_msg.x:+.2f},{pos_msg.y:+.2f})'
                   if pos_msg.z >= 0 else 'ball')
            cv2.putText(out,lbl,(x1,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,255),1)
        if ball_det is None and cup_det is None:
            cv2.putText(out,'no detections',(10,30),
                        cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,0,255),2)
        return out

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_pipeline'):
            try:
                self._pipeline.stop()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorOakNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
