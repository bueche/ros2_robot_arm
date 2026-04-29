#!/usr/bin/env python3
"""
ball_detector_nvidia.py

Detects ball and cup using YOLOv8n TensorRT engine on the Orin Nano GPU.
OAK-D Lite is used as camera only — no NN on device.
Frames are MJPEG-encoded on the OAK ISP and decoded on host to stay within
USB2 bandwidth limits (~35 MB/s real throughput).

Architecture:
  _camera_thread  — depthai MJPEG frame grab → _frame_queue
  _inference_thread — decode + TensorRT YOLOv8n → _det_queue
  _publish_loop   — timer-driven: /ball/position, /ball/cup_detected at inference rate
                    /ball/image at debug_fps (default 5 Hz)

Target: ≥20 FPS detection on Orin Nano (JetPack 6.1, TensorRT 10.x, CUDA 12.x).

Requires:
  depthai==2.24.0
  ultralytics (with TensorRT backend — available in jetson-containers)
  numpy==1.26.4  (numpy 2.x breaks cv_bridge)
  cv_bridge (Jazzy)

Model export (run once on Orin Nano inside container):
  yolo export model=best.pt format=engine device=0 imgsz=640 half=True

Publishes:
  /ball/position        geometry_msgs/Point   x,y normalised (-1..+1), z=0 detected
  /ball/cup_detected    std_msgs/Bool
  /ball/image           sensor_msgs/Image     annotated debug frame (debug_fps Hz)

Subscribes:
  /arm_state            std_msgs/String       MOVING→SETTLED resets cup size filter

Parameters:
  engine_path       Path to TensorRT .engine file          default: ''
  input_size        Model input width/height (px)          default: 640
  conf_threshold    Detection confidence threshold         default: 0.30
  iou_threshold     NMS IoU threshold                      default: 0.45
  rgb_fps           Camera framerate                       default: 20
  mjpeg_quality     OAK onboard MJPEG quality (1-100)      default: 85
  exposure_us       Manual exposure us (0=auto)            default: 25000
  iso               Manual ISO (used when exposure_us > 0) default: 800
  cup_jump_frac     Max fractional cup bbox size change     default: 0.50
  warmup_frames     Size-filter bypass for first N dets     default: 10
  debug_fps         Rate for /ball/image publish            default: 5.0
  show_debug        Annotate debug image                   default: True
  publish_image     Publish /ball/image topic              default: True
  log_spikes        Log inference time spikes              default: True
"""

import os
import cv2
import time
import threading
import queue
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String as StringMsg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

try:
    import depthai as dai
    DEPTHAI_AVAILABLE = True
except ImportError:
    DEPTHAI_AVAILABLE = False

try:
    from ultralytics import YOLO
    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False

BALL_IDX    = 0
CUP_IDX     = 1
CLASS_NAMES = ['ball', 'cup']

# TensorRT warmup: run this many dummy inferences before the live loop
# to JIT-compile CUDA kernels and avoid a latency spike on frame 1.
_TRT_WARMUP_INFERENCES = 3


class BallDetectorNvidiaNode(Node):

    def __init__(self):
        super().__init__('ball_detector_nvidia')

        if not DEPTHAI_AVAILABLE:
            self.get_logger().error(
                'depthai not installed. '
                'Run: pip install depthai==2.24.0')
            raise RuntimeError('depthai not available')

        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error(
                'ultralytics not installed — required for TensorRT engine inference.')
            raise RuntimeError('ultralytics not available')

        # Parameters
        self.declare_parameter('engine_path',      '')
        self.declare_parameter('input_size',        640)
        self.declare_parameter('conf_threshold',    0.30)
        self.declare_parameter('iou_threshold',     0.45)
        self.declare_parameter('rgb_fps',           20)
        self.declare_parameter('mjpeg_quality',     85)
        self.declare_parameter('exposure_us',       25000)
        self.declare_parameter('iso',               800)
        self.declare_parameter('cup_jump_frac',     0.50)
        self.declare_parameter('warmup_frames',     10)
        self.declare_parameter('debug_fps',         5.0)
        self.declare_parameter('show_debug',        True)
        self.declare_parameter('publish_image',     True)
        self.declare_parameter('log_spikes',        True)

        # Cache parameters — avoids ROS2 service round-trips inside hot loops
        self._p_input_size      = self.get_parameter('input_size').value
        self._p_conf_threshold  = self.get_parameter('conf_threshold').value
        self._p_iou_threshold   = self.get_parameter('iou_threshold').value
        self._p_rgb_fps         = self.get_parameter('rgb_fps').value
        self._p_mjpeg_quality   = self.get_parameter('mjpeg_quality').value
        self._p_cup_jump_frac   = self.get_parameter('cup_jump_frac').value
        self._p_warmup_frames   = self.get_parameter('warmup_frames').value
        self._p_debug_fps       = self.get_parameter('debug_fps').value
        self._p_show_debug      = self.get_parameter('show_debug').value
        self._p_publish_image   = self.get_parameter('publish_image').value
        self._p_log_spikes      = self.get_parameter('log_spikes').value

        # Publishers
        self._pub_pos   = self.create_publisher(Point, '/ball/position',     10)
        self._pub_cup   = self.create_publisher(Bool,  '/ball/cup_detected', 10)
        self._pub_img   = self.create_publisher(Image, '/ball/image',        10)
        self._bridge    = CvBridge()

        # Arm state subscriber — resets size filter on MOVING→SETTLED
        self._arm_state = ''
        self._arm_state_sub = self.create_subscription(
            StringMsg, '/arm_state', self._on_arm_state, 10)

        # Shared state
        self._running       = True
        self._frame_lock    = threading.Lock()

        # _frame_queue: raw MJPEG bytes from camera thread → inference thread
        # maxsize=4 — we only want the latest frames; old ones are discarded
        self._frame_queue = queue.Queue(maxsize=4)

        # _det_queue: inference results → publish loop
        # maxsize=60 keeps ~3s of headroom at 20fps
        self._det_queue = queue.Queue(maxsize=60)

        # Temporal size tracking for bbox sanity filter
        self._last_cup_wh    = None
        self._cup_det_count  = 0

        # Drop / diagnostics
        self._frame_drops   = 0
        self._infer_drops   = 0
        self._resource_tick = 0
        self._last_img_pub  = 0.0

        # Load TensorRT engine
        engine_path = self.get_parameter('engine_path').value
        if not engine_path or not os.path.exists(engine_path):
            raise RuntimeError(
                f'engine_path not found: "{engine_path}"\n'
                f'Export with: yolo export model=best.pt format=engine '
                f'device=0 imgsz={self._p_input_size} half=True')

        self.get_logger().info(f'Loading TensorRT engine: {engine_path}')
        self._model = YOLO(engine_path, task='detect')
        self.get_logger().info('TensorRT engine loaded — running warmup inferences')
        self._trt_warmup()

        # Build depthai pipeline — camera only, MJPEG encoded
        self._device = self._build_pipeline()

        self._q_mjpeg = self._device.getOutputQueue(
            'mjpeg', maxSize=4, blocking=False)

        # Threads
        self._camera_thread = threading.Thread(
            target=self._camera_loop, daemon=True, name='cam_grab')
        self._inference_thread = threading.Thread(
            target=self._inference_loop, daemon=True, name='trt_infer')

        self._camera_thread.start()
        self._inference_thread.start()

        self.get_logger().info(
            f'ball_detector_nvidia ready  '
            f'engine={os.path.basename(engine_path)}  '
            f'input={self._p_input_size}px  '
            f'conf={self._p_conf_threshold}  iou={self._p_iou_threshold}  '
            f'rgb_fps={self._p_rgb_fps}  mjpeg_q={self._p_mjpeg_quality}  '
            f'debug_fps={self._p_debug_fps}')

    # ------------------------------------------------------------------
    # Arm state callback
    # ------------------------------------------------------------------

    def _on_arm_state(self, msg):
        """Reset bbox size references when arm transitions MOVING → SETTLED.

        After an arm move the cup appears at a different scale in the frame.
        Keeping the pre-move size reference would cause every post-move detection
        to be rejected as a size jump.  Clearing references re-triggers the
        warmup window so the first _warmup_frames detections are accepted
        unconditionally and seed the new correct reference size.
        """
        clean = msg.data.rstrip('\x00').strip()
        prev = self._arm_state
        self._arm_state = clean
        if prev == 'MOVING' and clean == 'SETTLED':
            self._last_cup_wh   = None
            self._cup_det_count = 0
            self.get_logger().info('Arm SETTLED — cup size reference reset')

    # ------------------------------------------------------------------
    # TensorRT warmup
    # ------------------------------------------------------------------

    def _trt_warmup(self):
        """Run a few dummy inferences to trigger CUDA kernel JIT compilation.

        Without this, the first live inference can take ~100ms, which would
        stall the camera-facing queue and cause frame drops on startup.
        """
        dummy = np.zeros(
            (self._p_input_size, self._p_input_size, 3), dtype=np.uint8)
        for i in range(_TRT_WARMUP_INFERENCES):
            t0 = time.monotonic()
            _ = self._model(
                dummy,
                conf=self._p_conf_threshold,
                iou=self._p_iou_threshold,
                verbose=False)
            ms = (time.monotonic() - t0) * 1000
            self.get_logger().info(f'TRT warmup {i+1}/{_TRT_WARMUP_INFERENCES}: {ms:.1f}ms')

    # ------------------------------------------------------------------
    # depthai pipeline — camera only, MJPEG output
    # ------------------------------------------------------------------

    def _build_pipeline(self):
        """Build depthai pipeline that streams MJPEG over USB to host.

        No NN runs on device.  MJPEG encoding at quality=mjpeg_quality keeps
        USB2 bandwidth well under the ~35 MB/s real-world limit.

        At q=85, 640×640 MJPEG is ~120 KB/frame → 3.5 MB/s @ 30fps — well
        within budget.  encode latency runs on the OAK ISP in parallel with
        USB transfer so it does not add to pipeline latency.
        """
        pipeline = dai.Pipeline()

        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_720_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setPreviewSize(self._p_input_size, self._p_input_size)
        cam.setFps(self._p_rgb_fps)

        exp_us = self.get_parameter('exposure_us').value
        iso    = self.get_parameter('iso').value
        if exp_us > 0:
            cam.initialControl.setManualExposure(exp_us, iso)

        # MJPEG encoder — runs on OAK ISP, parallel with USB transfer
        encoder = pipeline.create(dai.node.VideoEncoder)
        encoder.setDefaultProfilePreset(
            self._p_rgb_fps,
            dai.VideoEncoderProperties.Profile.MJPEG)
        encoder.setQuality(self._p_mjpeg_quality)

        # Encoder expects NV12; ISP preview is BGR — use ImageManip to convert
        manip = pipeline.create(dai.node.ImageManip)
        manip.initialConfig.setFrameType(dai.RawImgFrame.Type.NV12)
        manip.inputImage.setBlocking(False)
        manip.inputImage.setQueueSize(1)

        cam.preview.link(manip.inputImage)
        manip.out.link(encoder.input)

        xout = pipeline.create(dai.node.XLinkOut)
        xout.setStreamName('mjpeg')
        xout.input.setBlocking(False)
        xout.input.setQueueSize(2)
        encoder.bitstream.link(xout.input)

        device = dai.Device(pipeline)
        self.get_logger().info(
            f'OAK-D connected: {device.getDeviceName()}  '
            f'USB {device.getUsbSpeed().name}  '
            f'(camera-only mode, MJPEG q={self._p_mjpeg_quality})')
        return device

    # ------------------------------------------------------------------
    # Camera thread — grabs MJPEG frames from OAK and queues for inference
    # ------------------------------------------------------------------

    def _camera_loop(self):
        """Drain OAK MJPEG queue as fast as possible.

        Pushes raw MJPEG bytes into _frame_queue.  If queue is full, the
        oldest frame is discarded (we always want the freshest frame for
        inference, not a stale one).
        """
        while self._running:
            try:
                pkt = self._q_mjpeg.tryGet()
                if pkt is None:
                    time.sleep(0.0005)
                    continue

                jpeg_bytes = bytes(pkt.getData())

                try:
                    self._frame_queue.put_nowait(jpeg_bytes)
                except queue.Full:
                    # Discard oldest, insert newest
                    self._frame_drops += 1
                    try:
                        self._frame_queue.get_nowait()
                        self._frame_queue.put_nowait(jpeg_bytes)
                    except queue.Empty:
                        pass

                # Periodic diagnostics
                self._resource_tick += 1
                if self._resource_tick % 500 == 0:
                    self.get_logger().info(
                        f'[cam] frame_drops={self._frame_drops}  '
                        f'infer_drops={self._infer_drops}  '
                        f'q_depth={self._frame_queue.qsize()}')

            except Exception as e:
                self.get_logger().warn(str(e), throttle_duration_sec=2.0)

    # ------------------------------------------------------------------
    # Inference thread — decode MJPEG → TensorRT → queue detections
    # ------------------------------------------------------------------

    def _inference_loop(self):
        """Decode MJPEG on CPU, run YOLOv8n TensorRT on GPU, push results.

        Decoding: cv2.imdecode — typically 2-5ms on CPU, acceptable.
        Inference: ~10ms FP16 on Orin Nano @ 640×640.

        The decoded frame is bundled with detections so _process_and_publish
        can annotate the exact frame that was inferred on.
        """
        infer_ms_avg = 0.0
        infer_count  = 0

        while self._running:
            try:
                jpeg_bytes = self._frame_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            # MJPEG decode
            nparr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            frame = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn(
                    'MJPEG decode failed — skipping frame',
                    throttle_duration_sec=2.0)
                continue

            # Resize to model input size if needed (should already be correct
            # from OAK preview size, but guard against encoder rounding)
            sz = self._p_input_size
            if frame.shape[0] != sz or frame.shape[1] != sz:
                frame = cv2.resize(frame, (sz, sz))

            # TensorRT inference via ultralytics
            t0 = time.monotonic()
            results = self._model(
                frame,
                conf=self._p_conf_threshold,
                iou=self._p_iou_threshold,
                verbose=False)
            infer_ms = (time.monotonic() - t0) * 1000

            # Spike logging
            if self._p_log_spikes and infer_ms > 50.0:
                self.get_logger().warn(
                    f'[infer] SPIKE {infer_ms:.1f}ms')

            # Rolling average logged every 100 frames
            infer_count += 1
            infer_ms_avg = 0.9 * infer_ms_avg + 0.1 * infer_ms
            if infer_count % 100 == 0:
                self.get_logger().info(
                    f'[infer] avg={infer_ms_avg:.1f}ms  '
                    f'fps_cap={1000/max(infer_ms_avg,1):.1f}  '
                    f'infer_drops={self._infer_drops}')

            # Parse ultralytics Results object into a flat list of dicts
            dets = self._parse_results(results)

            # Push to publish queue
            try:
                self._det_queue.put_nowait((dets, frame))
            except queue.Full:
                self._infer_drops += 1
                try:
                    self._det_queue.get_nowait()
                    self._det_queue.put_nowait((dets, frame))
                except queue.Empty:
                    pass

    def _parse_results(self, results):
        """Convert ultralytics Results to a list of plain dicts.

        Each dict: {label, confidence, xmin, ymin, xmax, ymax}
        Coordinates are normalised 0-1.
        """
        dets = []
        if not results or results[0].boxes is None:
            return dets

        boxes = results[0].boxes
        sz    = float(self._p_input_size)
        for i in range(len(boxes)):
            xyxy  = boxes.xyxy[i].cpu().numpy()
            conf  = float(boxes.conf[i].cpu().numpy())
            label = int(boxes.cls[i].cpu().numpy())
            dets.append({
                'label':      label,
                'confidence': conf,
                'xmin':       float(xyxy[0]) / sz,
                'ymin':       float(xyxy[1]) / sz,
                'xmax':       float(xyxy[2]) / sz,
                'ymax':       float(xyxy[3]) / sz,
            })
        return dets

    # ------------------------------------------------------------------
    # Publish loop — driven by _det_queue, debug image rate-limited
    # ------------------------------------------------------------------

    def _publish_loop_spin(self):
        """Called from a ROS2 timer — drains _det_queue and publishes.

        /ball/position and /ball/cup_detected: every inference result
        /ball/image: rate-limited to debug_fps
        """
        while not self._det_queue.empty():
            try:
                dets, frame = self._det_queue.get_nowait()
            except queue.Empty:
                break
            self._process_and_publish(dets, frame)

    # ------------------------------------------------------------------
    # Detection processing and publish
    # ------------------------------------------------------------------

    def _process_and_publish(self, dets, frame):
        """Apply size filter, compute normalised position, publish all topics."""
        h, w = frame.shape[:2] if frame is not None else (
            self._p_input_size, self._p_input_size)

        best_ball_norm = None;  best_ball_c = 0.0
        best_cup_norm  = None;  best_cup_c  = 0.0

        for d in dets:
            dw   = d['xmax'] - d['xmin']
            dh_d = d['ymax'] - d['ymin']
            conf = d['confidence']

            if d['label'] == BALL_IDX and conf > best_ball_c:
                # Ball size filter disabled — in controlled scene environment
                # the confidence threshold is sufficient.  Size filtering was
                # causing false negatives when the arm moved (cup scale change
                # caused proportional ball bbox change, exceeding jump threshold).
                best_ball_c    = conf
                best_ball_norm = (d['xmin'], d['ymin'], d['xmax'], d['ymax'])
                self.get_logger().info(
                    f'Ball bbox: {int(dw*w)}x{int(dh_d*h)}px  conf={conf:.3f}',
                    throttle_duration_sec=1.0)

            elif d['label'] == CUP_IDX and conf > best_cup_c:
                # ROI sanity: cup centre must be in lower 75% and middle 80%
                cx_n = (d['xmin'] + d['xmax']) / 2.0
                cy_n = (d['ymin'] + d['ymax']) / 2.0
                if cy_n < 0.25 or cx_n < 0.10 or cx_n > 0.90:
                    self.get_logger().warn(
                        f'Cup rejected by ROI: cx={cx_n:.2f} cy={cy_n:.2f}',
                        throttle_duration_sec=2.0)
                    continue
                # Size jump filter — bypass during warmup
                if (self._last_cup_wh is not None
                        and self._cup_det_count >= self._p_warmup_frames):
                    lw, lh = self._last_cup_wh
                    if lw > 0 and lh > 0:
                        jump = max(abs(dw - lw) / lw, abs(dh_d - lh) / lh)
                        if jump > self._p_cup_jump_frac:
                            self.get_logger().warn(
                                f'Cup size jump {jump:.0%} — rejected',
                                throttle_duration_sec=1.0)
                            continue
                best_cup_c    = conf
                best_cup_norm = (d['xmin'], d['ymin'], d['xmax'], d['ymax'])
                self._cup_det_count += 1
                self.get_logger().info(
                    f'Cup  bbox: {int(dw*w)}x{int(dh_d*h)}px  conf={conf:.3f}',
                    throttle_duration_sec=1.0)

        # Update temporal size trackers
        if best_cup_norm is not None:
            xmin, ymin, xmax, ymax = best_cup_norm
            self._last_cup_wh = (xmax - xmin, ymax - ymin)

        cup_found  = best_cup_norm  is not None
        ball_found = best_ball_norm is not None

        if not cup_found and not ball_found:
            self.get_logger().warn('No detections', throttle_duration_sec=2.0)
        elif not cup_found:
            self.get_logger().warn(
                f'Ball found (conf={best_ball_c:.3f}) but NO CUP',
                throttle_duration_sec=2.0)
        elif not ball_found:
            self.get_logger().warn(
                f'Cup found (conf={best_cup_c:.3f}) but NO BALL',
                throttle_duration_sec=2.0)

        # Normalised cup geometry
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

        # /ball/cup_detected
        self._pub_cup.publish(Bool(data=cup_found))

        # /ball/position — normalised offset from cup centre
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

        # /ball/image — rate-limited to debug_fps, annotated
        if frame is not None and self._p_publish_image:
            now = time.monotonic()
            debug_interval = 1.0 / self._p_debug_fps
            if now - self._last_img_pub >= debug_interval:
                self._last_img_pub = now

                def norm_to_px(n):
                    xmin, ymin, xmax, ymax = n
                    return (int(xmin*w), int(ymin*h),
                            int(xmax*w), int(ymax*h))

                ball_px_box = norm_to_px(best_ball_norm) if ball_found else None
                cup_px_box  = norm_to_px(best_cup_norm)  if cup_found  else None

                if self._p_show_debug:
                    debug = self._draw_debug(frame, ball_px_box, cup_px_box, pos_msg)
                else:
                    debug = frame

                img_msg = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
                img_msg.header.stamp    = self.get_clock().now().to_msg()
                img_msg.header.frame_id = 'camera_frame'
                self._pub_img.publish(img_msg)

    # ------------------------------------------------------------------
    # Debug annotation — identical logic to v37
    # ------------------------------------------------------------------

    def _draw_debug(self, frame, ball_det, cup_det, pos_msg):
        out = frame.copy()
        if cup_det is not None:
            x1, y1, x2, y2 = cup_det
            cv2.rectangle(out, (x1,y1), (x2,y2), (0,255,0), 2)
            cx = int((x1+x2)/2); cy = int((y1+y2)/2)
            cv2.circle(out, (cx,cy), 3, (0,255,0), -1)
            cv2.line(out, (cx-10,cy), (cx+10,cy), (0,255,0), 1)
            cv2.line(out, (cx,cy-10), (cx,cy+10), (0,255,0), 1)
            cv2.putText(out, 'cup', (x1,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        if ball_det is not None:
            x1, y1, x2, y2 = ball_det
            cv2.rectangle(out, (x1,y1), (x2,y2), (0,0,255), 2)
            bx = int((x1+x2)/2); by = int((y1+y2)/2)
            cv2.circle(out, (bx,by), 4, (0,0,255), -1)
            if cup_det is not None:
                cx = int((cup_det[0]+cup_det[2])/2)
                cy = int((cup_det[1]+cup_det[3])/2)
                cv2.line(out, (cx,cy), (bx,by), (0,165,255), 1)
            lbl = (f'ball ({pos_msg.x:+.2f},{pos_msg.y:+.2f})'
                   if pos_msg.z >= 0 else 'ball')
            cv2.putText(out, lbl, (x1,y1-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        if ball_det is None and cup_det is None:
            cv2.putText(out, 'no detections', (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
        return out

    # ------------------------------------------------------------------
    # Shutdown
    # ------------------------------------------------------------------

    def destroy_node(self):
        self._running = False
        if hasattr(self, '_device'):
            try:
                self._device.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNvidiaNode()

    # ROS2 timer drives the publish loop at inference rate.
    # Camera and inference run in daemon threads; the timer drains _det_queue.
    timer_hz = node._p_rgb_fps * 2   # poll at 2x camera fps — never misses a frame
    node.create_timer(1.0 / timer_hz, node._publish_loop_spin)

    executor = MultiThreadedExecutor(num_threads=2)
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
