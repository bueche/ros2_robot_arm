#!/usr/bin/env python3
"""
ball_detector_oak.py (v9 - depthai v2 API, MyriadX inference)

Detects ball and cup using YOLOv8n blob running on OAK-D Lite MyriadX VPU.
Inference runs on the camera hardware at ~12 fps, zero CPU/GPU load on host.

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
  rgb_fps           Camera framerate                    default: 12
  exposure_us       Manual exposure us (0=auto)         default: 25000
  iso               Manual ISO                          default: 800
  enable_depth      Enable stereo depth pipeline        default: False
  camera_frame_id   TF frame for 3D output             default: 'oak_rgb_camera_optical_frame'
  depth_min_mm      Min valid depth                     default: 100
  depth_max_mm      Max valid depth                     default: 2000
  show_debug        Annotate published image            default: True
  publish_image     Publish /ball/image topic           default: True
  debug_width       Debug image width (px)              default: 320
  debug_height      Debug image height (px)             default: 320
  publish_hz        ROS publish rate                    default: 12.0
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
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Bool, String as StringMsg
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
        self.declare_parameter('rgb_fps',          12)
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
        self.declare_parameter('debug_height',  320)
        self.declare_parameter('publish_hz',    12.0)
        self.declare_parameter('log_spikes',   True)   # log publish time spikes >half budget
        self.declare_parameter('containment_margin', 0.30)  # slack around cup bbox
        self.declare_parameter('warmup_frames',      10)    # bypass containment for first N cups
        self.declare_parameter('min_ball_conf',      0.50)  # min conf for ball without cup anchor

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

        # Arm state subscriber — resets size filter on MOVING→SETTLED
        self._arm_state = ''
        self._arm_state_sub = self.create_subscription(
            StringMsg, '/arm_state', self._on_arm_state, 10)

        # Shared state
        self._latest_rgb   = None
        self._latest_depth = None
        self._frame_lock   = threading.Lock()
        self._det_queue    = queue.Queue(maxsize=60)  # buffer ~2s at 30fps
        self._running      = True

        # Drop / resource tracking
        self._last_rgb_seq   = -1
        self._last_det_seq   = -1
        self._rgb_drops      = 0
        self._det_drops_dev  = 0   # device-side: MyriadX produced but XLink dropped
        self._det_drops_host = 0   # host-side: arrived via USB but Python queue full
        self._resource_tick  = 0

        # Temporal size tracking for bbox sanity filter
        # _warmup_count: don't apply size filter until we have N stable detections
        # References are also reset on MOVING→SETTLED arm state transitions so
        # that post-move scale changes don't permanently block detection.
        self._last_cup_wh     = None
        self._last_ball_wh    = None
        self._cup_jump_frac   = 0.35
        self._ball_jump_frac  = 0.50
        self._ball_det_count  = 0   # number of accepted ball detections so far
        self._cup_det_count   = 0

        # Cached parameters
        self._p_warmup_frames      = self.get_parameter('warmup_frames').value
        self._p_containment_margin = self.get_parameter('containment_margin').value
        self._p_min_ball_conf      = self.get_parameter('min_ball_conf').value

        # Cached parameters — read once at startup to avoid get_parameter()
        # calls in the hot publish loop (each call involves a ROS2 service
        # round-trip which can block for 100s of ms under DDS contention)
        self._p_publish_image  = self.get_parameter('publish_image').value
        self._p_show_debug     = self.get_parameter('show_debug').value
        self._p_debug_width    = self.get_parameter('debug_width').value
        self._p_debug_height   = self.get_parameter('debug_height').value
        self._p_rgb_fps        = self.get_parameter('rgb_fps').value
        self._p_log_spikes     = self.get_parameter('log_spikes').value
        self._p_conf_threshold = self.get_parameter('conf_threshold').value
        self._p_depth_min_mm   = self.get_parameter('depth_min_mm').value
        self._p_depth_max_mm   = self.get_parameter('depth_max_mm').value
        self._p_camera_frame   = self.get_parameter('camera_frame_id').value

        # Build pipeline and connect device
        blob_path = self.get_parameter('blob_path').value
        if not blob_path or not os.path.exists(blob_path):
            raise RuntimeError(
                f'blob_path not found: "{blob_path}"\n'
                f'Set blob_path parameter to your .blob file.')

        self._device = self._build_pipeline(blob_path)

        # Detection queue — device-side maxSize=30 to absorb bursts before USB transfer
        # Larger buffer reduces device-side drops when USB is momentarily busy
        self._q_det = self._device.getOutputQueue(
            'detections', maxSize=30, blocking=False)

        # RGB queue — non-blocking, just want latest frame for debug image
        self._q_rgb = self._device.getOutputQueue(
            'rgb', maxSize=2, blocking=False)

        # Depth queue — optional
        self._q_depth = (
            self._device.getOutputQueue('depth', maxSize=2, blocking=False)
            if self.get_parameter('enable_depth').value else None)

        pub_img   = self.get_parameter('publish_image').value
        dbg_w     = self.get_parameter('debug_width').value
        dbg_h     = self.get_parameter('debug_height').value
        ena_depth = self.get_parameter('enable_depth').value

        # Grab thread — drains device queues as fast as possible
        self._grab_thread = threading.Thread(
            target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        # Publish thread — processes and publishes at camera fps rate
        self._publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True)
        self._publish_thread.start()

        self.get_logger().info(
            f'ball_detector_oak ready (depthai {dai.__version__}, '
            f'MyriadX event-driven  '
            f'image={pub_img} {dbg_w}x{dbg_h}  depth={ena_depth}  '
            f'containment_margin={self._p_containment_margin}  '
            f'min_ball_conf={self._p_min_ball_conf})')

    def _on_arm_state(self, msg):
        """Reset bbox size references when arm transitions MOVING → SETTLED.

        After an arm move the cup appears at a different scale in the frame.
        Keeping the pre-move size reference would cause every post-move ball
        detection to be rejected as a 'size jump'.  Clearing the references
        re-triggers the warmup window so the first _warmup_frames detections
        are accepted unconditionally and seed the new correct reference size.
        """
        clean = msg.data.rstrip('\x00').strip()
        prev = self._arm_state
        self._arm_state = clean
        if prev == 'MOVING' and msg.data == 'SETTLED':
            self._last_ball_wh   = None
            self._last_cup_wh    = None
            self._ball_det_count = 0
            self._cup_det_count  = 0
            self.get_logger().info(
                'Arm SETTLED — ball/cup size reference reset')

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

        # ROS parameters always win over JSON config values.
        # JSON may contain thresholds from blob conversion time which are
        # not suitable for runtime tuning. Log both so discrepancies are visible.
        iou_thresh  = self.get_parameter('iou_threshold').value
        conf_thresh = self.get_parameter('conf_threshold').value
        json_iou    = meta.get('iou_threshold',    None)
        json_conf   = meta.get('confidence_threshold', None)
        self.get_logger().info(
            f'Thresholds — using: conf={conf_thresh:.2f}  iou={iou_thresh:.2f}'
            + (f'  (JSON had: conf={json_conf}  iou={json_iou})' 
               if json_conf is not None or json_iou is not None else
               '  (no threshold overrides in JSON)'))

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
        """
        Grab loop — drains device queues as fast as possible.
        Stores latest detection and frame in shared state.
        Never calls publish directly — keeps queue draining fast.
        Resource stats logged every ~500 detections.
        """
        while self._running:
            try:
                # Drain RGB queue — keep only latest frame
                rgb = self._q_rgb.tryGet()
                if rgb is not None:
                    with self._frame_lock:
                        rgb_seq = rgb.getSequenceNum()
                        if self._last_rgb_seq >= 0:
                            self._rgb_drops += max(0, rgb_seq - self._last_rgb_seq - 1)
                        self._last_rgb_seq = rgb_seq
                        self._latest_rgb = rgb.getCvFrame()

                # Drain depth queue if enabled
                if self._q_depth:
                    depth = self._q_depth.tryGet()
                    if depth is not None:
                        with self._frame_lock:
                            self._latest_depth = depth.getFrame()

                # Drain detection queue — store latest for publish loop
                det = self._q_det.tryGet()
                if det is None:
                    time.sleep(0.0005)
                    continue

                seq = det.getSequenceNum()
                if self._last_det_seq >= 0:
                    gap = max(0, seq - self._last_det_seq - 1)
                    self._det_drops_dev += gap   # dropped on device before USB
                self._last_det_seq = seq

                # Put detection in host queue — track host-side drops separately
                try:
                    self._det_queue.put_nowait(det.detections)
                except queue.Full:
                    self._det_drops_host += 1    # arrived via USB but queue full
                    try:
                        self._det_queue.get_nowait()  # discard oldest
                        self._det_queue.put_nowait(det.detections)
                    except queue.Empty:
                        pass

                # Resource log every ~500 detections
                self._resource_tick += 1
                if self._resource_tick % 500 == 0:
                    try:
                        temp     = self._device.getChipTemperature()
                        css_heap = self._device.getLeonCssHeapUsage()
                        mss_heap = self._device.getLeonMssHeapUsage()
                        self.get_logger().info(
                            f'[OAK] temp={temp.average:.1f}C  '
                            f'CSS heap: {css_heap.used//1024}/{css_heap.total//1024} KB  '
                            f'MSS heap: {mss_heap.used//1024}/{mss_heap.total//1024} KB  '
                            f'rgb_drops={self._rgb_drops}  '
                            f'det_drops_dev={self._det_drops_dev}  '
                            f'det_drops_host={self._det_drops_host}  '
                            f'det_seq={self._last_det_seq}')
                    except Exception as e:
                        self.get_logger().warn(
                            f'Resource query failed: {e}',
                            throttle_duration_sec=5.0)

            except Exception as e:
                self.get_logger().warn(
                    str(e), throttle_duration_sec=2.0)

    def _publish_loop(self):
        """
        Publish loop — reads latest detection from shared state and publishes.
        Runs at rgb_fps rate. Decoupled from grab loop so publishing never
        blocks queue draining. Tracks processing time for diagnostics.
        """
        fps      = self._p_rgb_fps
        interval = 1.0 / fps
        proc_ms  = 0.0
        while self._running:
            t0 = time.monotonic()

            # Block until a detection is available (up to 0.1s)
            try:
                dets = self._det_queue.get(timeout=0.1)
            except queue.Empty:
                continue

            with self._frame_lock:
                frame       = self._latest_rgb.copy()   if self._latest_rgb   is not None else None
                depth_frame = self._latest_depth.copy() if self._latest_depth is not None else None

            t_proc = time.monotonic()
            self._process_and_publish(dets, frame, depth_frame)
            proc_ms = (time.monotonic() - t_proc) * 1000

            # Spike detection — log immediately if enabled and over half budget
            budget_ms = interval * 1000
            log_spikes = self._p_log_spikes
            if log_spikes and proc_ms > budget_ms * 0.5:
                self.get_logger().warn(
                    f'[publish] SPIKE proc={proc_ms:.1f}ms  budget={budget_ms:.1f}ms')

            # Log processing time every ~100 publishes
            self._pub_tick = getattr(self, '_pub_tick', 0) + 1
            if self._pub_tick % 100 == 0:
                self.get_logger().info(
                    f'[publish] proc={proc_ms:.1f}ms  '
                    f'budget={budget_ms:.1f}ms  '
                    f'{"OK" if proc_ms < budget_ms else "OVERRUN"}')

            # Sleep remaining time to stay at fps rate
            elapsed   = time.monotonic() - t0
            remaining = interval - elapsed
            if remaining > 0:
                time.sleep(remaining)

    def _ball_inside_cup(self, ball_norm, cup_norm):
        """Return True if ball center falls within cup bbox + containment_margin."""
        bx = (ball_norm[0] + ball_norm[2]) / 2.0
        by = (ball_norm[1] + ball_norm[3]) / 2.0
        m  = self._p_containment_margin
        return (cup_norm[0] - m < bx < cup_norm[2] + m and
                cup_norm[1] - m < by < cup_norm[3] + m)

    def _process_and_publish(self, dets, frame, depth_frame):
        """Containment-based detection, normalised position publish.

        Mirrors ball_detector_nvidia_v7 filter strategy:

        Pass 1: collect highest-confidence ball and cup candidates,
                applying only the size-jump sanity filter.

        Pass 2: containment validation:
          Both found, ball inside cup  -> mutually validating, accept both.
          Both found, ball outside cup -> discard lower-confidence detection.
          Ball only, no cup            -> accept if conf >= min_ball_conf.
          Cup only, no ball            -> accepted (ball may be occluded).

        During warmup (first _p_warmup_frames cup detections after SETTLED
        reset) containment check is bypassed so the system can anchor.
        """
        h, w = (frame.shape[:2] if frame is not None
                else (self._p_debug_height, self._p_debug_width))

        # Pass 1 -- collect best raw candidates with size-jump filter
        best_ball_norm = None;  best_ball_c = 0.0
        best_cup_norm  = None;  best_cup_c  = 0.0

        for d in dets:
            dw   = d.xmax - d.xmin
            dh_d = d.ymax - d.ymin

            if d.label == BALL_IDX and d.confidence > self._p_conf_threshold:
                if self._last_ball_wh is not None and self._ball_det_count >= self._p_warmup_frames:
                    lw, lh = self._last_ball_wh
                    if lw > 0 and lh > 0:
                        jump = max(abs(dw - lw) / lw, abs(dh_d - lh) / lh)
                        if jump > self._ball_jump_frac:
                            self.get_logger().warn(
                                f'Ball size jump {jump:.0%} -- rejected',
                                throttle_duration_sec=1.0)
                            continue
                if d.confidence > best_ball_c:
                    best_ball_c    = d.confidence
                    best_ball_norm = (d.xmin, d.ymin, d.xmax, d.ymax)
                    self.get_logger().info(
                        f'Ball bbox: {int(dw*w)}x{int(dh_d*h)}px  conf={d.confidence:.3f}',
                        throttle_duration_sec=1.0)

            elif d.label == CUP_IDX and d.confidence > self._p_conf_threshold:
                cx_n = (d.xmin + d.xmax) / 2.0
                cy_n = (d.ymin + d.ymax) / 2.0
                if cy_n < 0.25 or cx_n < 0.10 or cx_n > 0.90:
                    self.get_logger().warn(
                        f'Cup rejected by ROI: cx={cx_n:.2f} cy={cy_n:.2f}',
                        throttle_duration_sec=2.0)
                    continue
                if self._last_cup_wh is not None and self._cup_det_count >= self._p_warmup_frames:
                    lw, lh = self._last_cup_wh
                    if lw > 0 and lh > 0:
                        jump = max(abs(dw - lw) / lw, abs(dh_d - lh) / lh)
                        if jump > self._cup_jump_frac:
                            self.get_logger().warn(
                                f'Cup  size jump {jump:.0%} -- rejected',
                                throttle_duration_sec=1.0)
                            continue
                if d.confidence > best_cup_c:
                    best_cup_c    = d.confidence
                    best_cup_norm = (d.xmin, d.ymin, d.xmax, d.ymax)
                    self._cup_det_count += 1
                    self.get_logger().info(
                        f'Cup  bbox: {int(dw*w)}x{int(dh_d*h)}px  conf={d.confidence:.3f}',
                        throttle_duration_sec=1.0)

        # Update temporal size trackers
        if best_cup_norm is not None:
            xmin, ymin, xmax, ymax = best_cup_norm
            self._last_cup_wh = (xmax - xmin, ymax - ymin)
        if best_ball_norm is not None:
            xmin, ymin, xmax, ymax = best_ball_norm
            self._last_ball_wh = (xmax - xmin, ymax - ymin)
            self._ball_det_count += 1

        # Pass 2 -- containment validation
        cup_found  = best_cup_norm  is not None
        ball_found = best_ball_norm is not None
        in_warmup  = self._cup_det_count < self._p_warmup_frames

        if cup_found and ball_found:
            if in_warmup:
                self.get_logger().info(
                    f'Warmup ({self._cup_det_count}/{self._p_warmup_frames}): '                    f'containment check bypassed',
                    throttle_duration_sec=2.0)
            elif self._ball_inside_cup(best_ball_norm, best_cup_norm):
                pass  # mutually validating -- accept both
            else:
                bx = (best_ball_norm[0] + best_ball_norm[2]) / 2.0
                by = (best_ball_norm[1] + best_ball_norm[3]) / 2.0
                if best_ball_c >= best_cup_c:
                    self.get_logger().warn(
                        f'Ball outside cup -- discarding cup '                        f'(ball={best_ball_c:.3f} >= cup={best_cup_c:.3f}) '                        f'ball_center=({bx:.3f},{by:.3f})',
                        throttle_duration_sec=1.0)
                    cup_found = False
                else:
                    self.get_logger().warn(
                        f'Ball outside cup -- discarding ball '                        f'(cup={best_cup_c:.3f} > ball={best_ball_c:.3f}) '                        f'ball_center=({bx:.3f},{by:.3f})',
                        throttle_duration_sec=1.0)
                    ball_found = False

        elif ball_found and not cup_found:
            if best_ball_c < self._p_min_ball_conf:
                self.get_logger().warn(
                    f'Ball (conf={best_ball_c:.3f}) without cup and below '                    f'min_ball_conf={self._p_min_ball_conf} -- discarding',
                    throttle_duration_sec=2.0)
                ball_found = False
            else:
                self.get_logger().warn(
                    f'Ball (conf={best_ball_c:.3f}) found but NO CUP',
                    throttle_duration_sec=2.0)

        elif cup_found and not ball_found:
            self.get_logger().warn(
                f'Cup (conf={best_cup_c:.3f}) found but NO BALL',
                throttle_duration_sec=2.0)

        else:
            self.get_logger().warn('No detections', throttle_duration_sec=2.0)
        # Position calculation in normalised 0-1 space
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

        # 3D position from depth
        if ball_found and depth_frame is not None:
            ball_px = int(ball_cx_n * w)
            ball_py = int(ball_cy_n * h)
            self._publish_3d(ball_px, ball_py, depth_frame)

        # Debug image — rate-limited to max 5Hz to reduce GIL contention
        # Full detection rate still published on /ball/position
        if frame is not None and self._p_publish_image:
            now_img = time.monotonic()
            if not hasattr(self, '_last_img_pub'):
                self._last_img_pub = 0.0
            if now_img - self._last_img_pub >= 0.2:  # 5Hz max
                self._last_img_pub = now_img
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

    def _publish_3d(self, bx, by, depth_frame):
        dh, dw = depth_frame.shape
        if not (0 <= bx < dw and 0 <= by < dh):
            return
        r   = 5
        roi = depth_frame[max(0,by-r):min(dh,by+r),
                          max(0,bx-r):min(dw,bx+r)].astype(float)
        mn  = self._p_depth_min_mm
        mx  = self._p_depth_max_mm
        valid = roi[(roi > mn) & (roi < mx)]
        if valid.size == 0:
            return
        dm = float(np.median(valid)) / 1000.0
        fx = 1078.0; fy = 1078.0; cx = 640.0; cy = 360.0
        msg = PointStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self._p_camera_frame
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
    # MultiThreadedExecutor:
    #   - thread 1: DDS spin / discovery traffic
    #   - thread 2: param/service callbacks
    # Publishing happens in _grab_loop thread, not the ROS executor
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
