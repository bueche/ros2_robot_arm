#!/usr/bin/env python3
"""
ball_detector_oak.py (v7 - depthai v2 API, MyriadX inference)

Detects ball and cup using YOLOv8n blob running on OAK-D Lite MyriadX VPU.
Inference runs on the camera hardware at ~25-30fps, zero CPU/GPU load on host.

Requires: depthai==2.24.0, blob file from blobconverter (OpenVINO 2022.1, 6 shaves)

Publishes:
  /ball/position        geometry_msgs/Point   x,y normalised (-1..+1), z=0 detected
  /ball/position_3d     geometry_msgs/PointStamped  x,y,z metres in camera frame
  /ball/cup_detected    std_msgs/Bool
  /ball/image           sensor_msgs/Image     annotated debug frame

Parameters:
  blob_path         Path to YOLOv8n .blob file         default: ''
  conf_threshold    Detection confidence threshold      default: 0.5
  iou_threshold     NMS IoU threshold                   default: 0.45
  input_size        Model input size (px)               default: 640
  rgb_fps           Camera framerate                    default: 30
  exposure_us       Manual exposure us (0=auto)         default: 25000
  iso               Manual ISO                          default: 800
  enable_depth      Enable stereo depth pipeline        default: False
  camera_frame_id   TF frame for 3D output             default: 'oak_rgb_camera_optical_frame'
  depth_min_mm      Min valid depth                     default: 100
  depth_max_mm      Max valid depth                     default: 2000
  show_debug        Annotate published image            default: True
  publish_image     Publish /ball/image topic           default: True
  debug_width       Debug image width (px)              default: 320
  debug_height      Debug image height (px)             default: 180
  publish_hz        ROS publish rate                    default: 30.0
"""

import os
import cv2
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False

BALL_IDX    = 0
CUP_IDX     = 1
CLASS_NAMES = ['ball', 'cup']


class BallDetectorOakNode(Node):

    def __init__(self):
        super().__init__('ball_detector_oak')

        if not DEPTHAI_AVAILABLE:
            self.get_logger().error(
                'depthai not installed. '
                'Run: pip install depthai==2.24.0')
            raise RuntimeError('depthai not available')

        # Parameters
        self.declare_parameter('blob_path',       '')
        self.declare_parameter('conf_threshold',   0.5)
        self.declare_parameter('iou_threshold',    0.45)
        self.declare_parameter('input_size',       640)
        self.declare_parameter('rgb_fps',          30)
        self.declare_parameter('exposure_us',      25000)
        self.declare_parameter('iso',              800)
        self.declare_parameter('enable_depth',     False)
        self.declare_parameter('camera_frame_id',
                               'oak_rgb_camera_optical_frame')
        self.declare_parameter('depth_min_mm',    100)
        self.declare_parameter('depth_max_mm',   2000)
        self.declare_parameter('show_debug',    True)
        self.declare_parameter('publish_image', True)
        self.declare_parameter('debug_width',   320)
        self.declare_parameter('debug_height',  180)
        self.declare_parameter('publish_hz',    30.0)

        # Publishers
        self._pub_pos   = self.create_publisher(
            Point,        '/ball/position',     10)
        self._pub_pos3d = self.create_publisher(
            PointStamped, '/ball/position_3d',  10)
        self._pub_cup   = self.create_publisher(
            Bool,         '/ball/cup_detected', 10)
        self._pub_img   = self.create_publisher(
            Image,        '/ball/image',        10)
        self._bridge    = CvBridge()

        # Shared state
        self._latest_rgb   = None
        self._latest_depth = None
        self._latest_dets  = []
        self._frame_lock   = threading.Lock()
        self._det_lock     = threading.Lock()
        self._running      = True

        # Drop / resource tracking
        self._last_rgb_seq   = -1
        self._last_det_seq   = -1
        self._rgb_drops      = 0
        self._det_drops      = 0
        self._resource_tick  = 0

        # Temporal size tracking for bbox sanity filter
        # Stores last accepted normalized (w, h) for cup and ball.
        # Detections that jump >max_jump_frac in one frame are rejected.
        self._last_cup_wh    = None   # (w_norm, h_norm)
        self._last_ball_wh   = None
        self._cup_jump_frac  = 0.35   # reject if cup size changes >35% in one frame
        self._ball_jump_frac = 0.50   # ball can move faster, more lenient

        # Build pipeline and connect device
        blob_path = self.get_parameter('blob_path').value
        if not blob_path or not os.path.exists(blob_path):
            raise RuntimeError(
                f'blob_path not found: "{blob_path}"\n'
                f'Set blob_path parameter to your .blob file.')

        self._device = self._build_pipeline(blob_path)

        # Output queues
        self._q_rgb   = self._device.getOutputQueue(
            'rgb',        maxSize=4, blocking=False)
        self._q_det   = self._device.getOutputQueue(
            'detections', maxSize=4, blocking=False)
        self._q_depth = (
            self._device.getOutputQueue('depth', maxSize=4, blocking=False)
            if self.get_parameter('enable_depth').value else None)

        # Grab thread
        self._grab_thread = threading.Thread(
            target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        hz = self.get_parameter('publish_hz').value
        pub_img    = self.get_parameter('publish_image').value
        dbg_w      = self.get_parameter('debug_width').value
        dbg_h      = self.get_parameter('debug_height').value
        ena_depth  = self.get_parameter('enable_depth').value
        # Use ReentrantCallbackGroup so _tick() runs in its own thread,
        # isolated from DDS discovery/deserialization traffic on the spin thread.
        self._tick_cb_group = ReentrantCallbackGroup()
        self._timer = self.create_timer(
            1.0 / hz, self._tick,
            callback_group=self._tick_cb_group)
        self.get_logger().info(
            f'ball_detector_oak ready (depthai {dai.__version__}, '
            f'MyriadX YoloDetectionNetwork, {hz}Hz  '
            f'image={pub_img} {dbg_w}x{dbg_h}  depth={ena_depth})')

    def _build_pipeline(self, blob_path):
        input_size = self.get_parameter('input_size').value
        fps        = self.get_parameter('rgb_fps').value
        exp_us     = self.get_parameter('exposure_us').value
        iso        = self.get_parameter('iso').value

        pipeline = dai.Pipeline()

        # RGB camera
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_720_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setPreviewSize(input_size, input_size)
        cam.setFps(fps)
        if exp_us > 0:
            cam.initialControl.setManualExposure(exp_us, iso)

        # Load JSON config from Luxonis export
        json_path = blob_path.replace('.blob', '.json').replace(
            '_openvino_2022.1_6shave', '')
        # Also try same directory with _v2 naming
        if not os.path.exists(json_path):
            json_path = blob_path.replace(
                '_openvino_2022.1_6shave.blob', '.json')
        nn_config = {}
        import json
        if os.path.exists(json_path):
            try:
                with open(json_path, 'r', encoding='utf-8') as f:
                    nn_config = json.load(f)
                self.get_logger().info(f'Loaded NN config from {json_path}')
            except (UnicodeDecodeError, json.JSONDecodeError) as e:
                self.get_logger().warn(
                    f'Could not parse JSON config at {json_path}: {e}. '
                    f'Using defaults.')
        else:
            self.get_logger().warn(
                f'No JSON config found at {json_path}, using defaults')

        meta = nn_config.get('nn_config', {}).get(
            'NN_specific_metadata', {})
        num_classes  = meta.get('classes',    len(CLASS_NAMES))
        anchors      = meta.get('anchors',    [])
        anchor_masks = meta.get('anchor_masks', {})
        iou_thresh   = meta.get('iou_threshold',
                       self.get_parameter('iou_threshold').value)
        conf_thresh  = meta.get('confidence_threshold',
                       self.get_parameter('conf_threshold').value)

        # YoloDetectionNetwork runs blob on MyriadX
        det = pipeline.create(dai.node.YoloDetectionNetwork)
        det.setBlobPath(blob_path)
        det.setConfidenceThreshold(conf_thresh)
        det.setNumClasses(num_classes)
        det.setCoordinateSize(4)
        det.setAnchors(anchors)
        det.setAnchorMasks(anchor_masks)
        det.setIouThreshold(iou_thresh)
        det.input.setBlocking(False)
        det.input.setQueueSize(1)
        cam.preview.link(det.input)

        # RGB video output for debug image — downscaled to save USB bandwidth
        # Detection runs on input_size preview; debug image uses video downscaled
        dbg_w = self.get_parameter('debug_width').value
        dbg_h = self.get_parameter('debug_height').value
        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setResize(dbg_w, dbg_h)
        manip.initialConfig.setFrameType(
            dai.RawImgFrame.Type.BGR888p)
        manip.inputImage.setBlocking(False)
        manip.inputImage.setQueueSize(1)
        cam.video.link(manip.inputImage)

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        xout_rgb.input.setBlocking(False)
        xout_rgb.input.setQueueSize(1)
        manip.out.link(xout_rgb.input)

        # Detection results output
        xout_det = pipeline.create(dai.node.XLinkOut)
        xout_det.setStreamName('detections')
        xout_det.input.setBlocking(False)
        xout_det.input.setQueueSize(1)
        det.out.link(xout_det.input)

        # Stereo depth — optional, burns significant USB bandwidth
        if self.get_parameter('enable_depth').value:
            mono_l = pipeline.create(dai.node.MonoCamera)
            mono_r = pipeline.create(dai.node.MonoCamera)
            stereo  = pipeline.create(dai.node.StereoDepth)

            mono_l.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            mono_r.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            mono_l.setFps(fps)
            mono_r.setFps(fps)

            stereo.setDefaultProfilePreset(
                dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            stereo.setLeftRightCheck(True)
            stereo.setExtendedDisparity(True)
            stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

            mono_l.out.link(stereo.left)
            mono_r.out.link(stereo.right)

            xout_depth = pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName('depth')
            xout_depth.input.setBlocking(False)
            xout_depth.input.setQueueSize(1)
            stereo.depth.link(xout_depth.input)

        device = dai.Device(pipeline)
        self.get_logger().info(
            f'OAK-D connected: {device.getDeviceName()} '
            f'USB {device.getUsbSpeed().name}')
        return device

    def _grab_loop(self):
        while self._running:
            try:
                rgb   = self._q_rgb.tryGet()
                det   = self._q_det.tryGet()
                depth = self._q_depth.tryGet() if self._q_depth else None

                with self._frame_lock:
                    if rgb is not None:
                        seq = rgb.getSequenceNum()
                        if self._last_rgb_seq >= 0:
                            skip = seq - self._last_rgb_seq - 1
                            if skip > 0:
                                self._rgb_drops += skip
                        self._last_rgb_seq = seq
                        self._latest_rgb = rgb.getCvFrame()

                    if depth is not None:
                        self._latest_depth = depth.getFrame()

                if det is not None:
                    seq = det.getSequenceNum()
                    if self._last_det_seq >= 0:
                        skip = seq - self._last_det_seq - 1
                        if skip > 0:
                            self._det_drops += skip
                    self._last_det_seq = seq
                    with self._det_lock:
                        self._latest_dets = det.detections

                # Resource + drop log every ~2 seconds (2000 * 1ms sleep)
                self._resource_tick += 1
                if self._resource_tick % 2000 == 0:
                    try:
                        temp     = self._device.getChipTemperature()
                        css_heap = self._device.getLeonCssHeapUsage()
                        mss_heap = self._device.getLeonMssHeapUsage()
                        self.get_logger().info(
                            f'[OAK] temp={temp.average:.1f}C  '
                            f'CSS heap: {css_heap.used//1024}/{css_heap.total//1024} KB  '
                            f'MSS heap: {mss_heap.used//1024}/{mss_heap.total//1024} KB  '
                            f'rgb_drops={self._rgb_drops}  '
                            f'det_drops={self._det_drops}  '
                            f'rgb_seq={self._last_rgb_seq}  '
                            f'det_seq={self._last_det_seq}')
                    except Exception as e:
                        self.get_logger().warn(
                            f'Resource query failed: {e}',
                            throttle_duration_sec=5.0)

            except Exception as e:
                self.get_logger().warn(
                    str(e), throttle_duration_sec=2.0)
            time.sleep(0.001)

    def _tick(self):
        with self._frame_lock:
            frame = self._latest_rgb.copy()   if self._latest_rgb   is not None else None
            depth = self._latest_depth.copy() if self._latest_depth is not None else None
        with self._det_lock:
            dets = list(self._latest_dets)

        if frame is None:
            self.get_logger().warn(
                'No RGB frame yet', throttle_duration_sec=2.0)
            return

        h, w = frame.shape[:2]

        # --- Detection selection ---
        # Store normalized coords (0-1) from VPU — these are in square inference
        # space and are aspect-ratio correct regardless of debug frame dimensions.
        # Pixel coords are derived separately for drawing only.
        best_ball_norm = None;  best_ball_c = 0.0   # (xmin,ymin,xmax,ymax) 0-1
        best_cup_norm  = None;  best_cup_c  = 0.0

        for d in dets:
            dw = d.xmax - d.xmin
            dh = d.ymax - d.ymin

            if d.label == BALL_IDX and d.confidence > best_ball_c:
                # Temporal size sanity: reject if bbox jumped too much from last frame
                if self._last_ball_wh is not None:
                    lw, lh = self._last_ball_wh
                    if lw > 0 and lh > 0:
                        jump = max(abs(dw - lw) / lw, abs(dh - lh) / lh)
                        if jump > self._ball_jump_frac:
                            self.get_logger().warn(
                                f'Ball size jump {jump:.0%} — rejected '
                                f'({int(dw*w)}x{int(dh*h)}px)',
                                throttle_duration_sec=1.0)
                            continue
                best_ball_c    = d.confidence
                best_ball_norm = (d.xmin, d.ymin, d.xmax, d.ymax)
                bw = int(dw * w)
                bh = int(dh * h)
                self.get_logger().info(
                    f'Ball bbox: {bw}x{bh}px  conf={d.confidence:.3f}',
                    throttle_duration_sec=1.0)

            elif d.label == CUP_IDX and d.confidence > best_cup_c:
                # Temporal size sanity: reject if bbox jumped too much from last frame
                if self._last_cup_wh is not None:
                    lw, lh = self._last_cup_wh
                    if lw > 0 and lh > 0:
                        jump = max(abs(dw - lw) / lw, abs(dh - lh) / lh)
                        if jump > self._cup_jump_frac:
                            self.get_logger().warn(
                                f'Cup  size jump {jump:.0%} — rejected '
                                f'({int(dw*w)}x{int(dh*h)}px)',
                                throttle_duration_sec=1.0)
                            continue
                best_cup_c    = d.confidence
                best_cup_norm = (d.xmin, d.ymin, d.xmax, d.ymax)
                cw = int(dw * w)
                ch = int(dh * h)
                self.get_logger().info(
                    f'Cup  bbox: {cw}x{ch}px  conf={d.confidence:.3f}',
                    throttle_duration_sec=1.0)

        # Update temporal size trackers with accepted detections
        if best_cup_norm is not None:
            xmin, ymin, xmax, ymax = best_cup_norm
            self._last_cup_wh = (xmax - xmin, ymax - ymin)
        if best_ball_norm is not None:
            xmin, ymin, xmax, ymax = best_ball_norm
            self._last_ball_wh = (xmax - xmin, ymax - ymin)

        cup_found  = best_cup_norm  is not None
        ball_found = best_ball_norm is not None

        # Log when either object is missing — throttled to avoid spam
        if not cup_found and not ball_found:
            self.get_logger().warn(
                'No detections', throttle_duration_sec=2.0)
        elif not cup_found:
            self.get_logger().warn(
                f'Ball found (conf={best_ball_c:.3f}) but NO CUP — '
                f'position invalid (z=-1)',
                throttle_duration_sec=2.0)
        elif not ball_found:
            self.get_logger().warn(
                f'Cup found (conf={best_cup_c:.3f}) but NO BALL',
                throttle_duration_sec=2.0)

        # --- Position calculation in normalized space ---
        # All math done in 0-1 coords from the square inference frame.
        # This is aspect-ratio correct and independent of debug image dimensions.
        cup_cx_n = cup_cy_n = cup_r_n = None
        if cup_found:
            xmin, ymin, xmax, ymax = best_cup_norm
            cup_cx_n = (xmin + xmax) / 2.0
            cup_cy_n = (ymin + ymax) / 2.0
            cup_r_n  = max(xmax - xmin, ymax - ymin) / 2.0

        ball_cx_n = ball_cy_n = None
        if ball_found:
            xmin, ymin, xmax, ymax = best_ball_norm
            ball_cx_n = (xmin + xmax) / 2.0
            ball_cy_n = (ymin + ymax) / 2.0

        # Publish cup detected
        self._pub_cup.publish(Bool(data=cup_found))

        # Publish normalised 2D position
        pos_msg = Point()
        if ball_found and cup_found and cup_r_n and cup_r_n > 0:
            pos_msg.x = (ball_cx_n - cup_cx_n) / cup_r_n
            pos_msg.y = (ball_cy_n - cup_cy_n) / cup_r_n
            pos_msg.z = 0.0
        else:
            pos_msg.x = 0.0
            pos_msg.y = 0.0
            pos_msg.z = -1.0
        self._pub_pos.publish(pos_msg)

        # 3D position from depth — convert normalized to debug frame pixels
        if ball_found and depth is not None:
            ball_px = int(ball_cx_n * w)
            ball_py = int(ball_cy_n * h)
            self._publish_3d(ball_px, ball_py, depth)

        # --- Debug image — pixel coords derived from normalized, scaled to frame ---
        if self.get_parameter('publish_image').value:
            # Convert normalized bboxes to pixel coords in debug frame
            def norm_to_px(n):
                xmin, ymin, xmax, ymax = n
                return (int(xmin*w), int(ymin*h),
                        int(xmax*w), int(ymax*h))

            ball_px_box = norm_to_px(best_ball_norm) if ball_found else None
            cup_px_box  = norm_to_px(best_cup_norm)  if cup_found  else None

            if self.get_parameter('show_debug').value:
                debug = self._draw_debug(frame, ball_px_box, cup_px_box, pos_msg)
            else:
                debug = frame
            img_msg = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
            img_msg.header.stamp    = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'
            self._pub_img.publish(img_msg)

    def _publish_3d(self, bx, by, depth_frame):
        dh, dw = depth_frame.shape
        if not (0 <= bx < dw and 0 <= by < dh):
            return
        r   = 5
        roi = depth_frame[max(0,by-r):min(dh,by+r),
                          max(0,bx-r):min(dw,bx+r)].astype(float)
        mn  = self.get_parameter('depth_min_mm').value
        mx  = self.get_parameter('depth_max_mm').value
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
            x1, y1, x2, y2 = cup_det
            cv2.rectangle(out, (x1,y1), (x2,y2), (0,255,0), 2)
            cx = int((x1+x2)/2); cy = int((y1+y2)/2)
            cv2.circle(out, (cx,cy), 3, (0,255,0), -1)
            cv2.line(out,(cx-10,cy),(cx+10,cy),(0,255,0),1)
            cv2.line(out,(cx,cy-10),(cx,cy+10),(0,255,0),1)
            cv2.putText(out,'cup',(x1,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,255,0),1)
        if ball_det is not None:
            x1, y1, x2, y2 = ball_det
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
        if hasattr(self, '_device'):
            self._device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorOakNode()
    # MultiThreadedExecutor with 3 threads:
    #   - thread 1: DDS spin / discovery traffic
    #   - thread 2: _tick() timer callback (camera publish)
    #   - thread 3: headroom for service/param callbacks
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
