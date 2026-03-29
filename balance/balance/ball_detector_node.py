#!/usr/bin/env python3
"""
ball_detector_node.py  (v2 — ellipse fitting)
----------------------------------------------
Detects the cup (large dark ellipse) and ball bearing (brightest blob
inside the cup) from a USB camera that views the cup at an angle.

Key changes from v1:
  - Cup detection uses contour finding + fitEllipse instead of Hough circles.
    This works correctly when the cup appears as an ellipse due to camera angle.
  - Configurable ROI (region of interest) to ignore background clutter.
  - Ball detection uses SimpleBlobDetector on the ROI (more robust than
    Hough circles for a single bright specular highlight).
  - Fallback to brightest-point if blob detector finds nothing.

Publishes:
  /ball/position        (geometry_msgs/Point)   normalised to ellipse semi-axes
                                                  x,y in (-1..+1), z=0 detected
                                                  z=-1 if ball not found
  /ball/cup_detected    (std_msgs/Bool)
  /ball/image           (sensor_msgs/Image)      annotated debug frame

Parameters:
  device          Camera device index            default: 0
  width           Capture width                  default: 640
  height          Capture height                 default: 480
  fps             Capture framerate              default: 30
  publish_hz      Publish rate                   default: 30.0

  # ROI — crop frame before any detection (reduces background clutter)
  # Set to 0,0,0,0 to use full frame
  roi_x           ROI left edge (px)             default: 0
  roi_y           ROI top edge (px)              default: 0
  roi_w           ROI width (px, 0=full)         default: 0
  roi_h           ROI height (px, 0=full)        default: 0

  # Cup detection (contour + ellipse fitting)
  cup_min_area    Min contour area (px²)         default: 3000
  cup_max_area    Max contour area (px²)         default: 200000
  cup_dark_thresh Threshold below which pixels   default: 80
                  are considered "dark" (cup interior)
  cup_min_ellipse_ratio  Min ellipse b/a ratio   default: 0.3
                         (prevents very thin sliver detections)
  cup_smooth_alpha EMA smoothing weight          default: 0.25

  # Ball detection
  ball_min_area   Min blob area (px²)            default: 20
  ball_max_area   Max blob area (px²)            default: 2000
  ball_bright_thresh  Pixels brighter than this  default: 180
                      are candidate ball pixels
  ball_margin     Fraction of ellipse to inset   default: 0.08
                  (avoids detecting the rim itself)

  show_debug      Annotate published image       default: True
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class BallDetectorNode(Node):

    def __init__(self):
        super().__init__('ball_detector_node')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('device',       0)
        self.declare_parameter('width',        640)
        self.declare_parameter('height',       480)
        self.declare_parameter('fps',          30)
        self.declare_parameter('publish_hz',   30.0)

        # ROI
        self.declare_parameter('roi_x', 0)
        self.declare_parameter('roi_y', 0)
        self.declare_parameter('roi_w', 0)
        self.declare_parameter('roi_h', 0)

        # Cup
        self.declare_parameter('cup_min_area',         10000.0)
        self.declare_parameter('cup_max_area',        200000.0)
        self.declare_parameter('cup_dark_thresh',         80)
        self.declare_parameter('cup_min_ellipse_ratio',    0.3)
        self.declare_parameter('cup_smooth_alpha',         0.25)

        # Ball
        self.declare_parameter('ball_min_area',           20.0)
        self.declare_parameter('ball_max_area',         2000.0)
        self.declare_parameter('ball_bright_thresh',      180)
        self.declare_parameter('ball_margin',              0.08)

        self.declare_parameter('show_debug',   True)

        # ── test image mode vs live camera ────────────────────────────────
        self.declare_parameter('test_image_dir', '')
        self.declare_parameter('test_image_hz',  2.0)   # playback rate

        test_dir = self.get_parameter('test_image_dir').value

        self._cap         = None
        self._test_images = []
        self._test_idx    = 0

        if test_dir:
            # Static image mode — load sorted list of jpg files
            import glob
            import os
            pattern = os.path.join(test_dir, '*.jpg')
            self._test_images = sorted(glob.glob(pattern))
            if not self._test_images:
                self.get_logger().error(f'No .jpg files in {test_dir}')
                raise RuntimeError('No test images found')
            self.get_logger().info(
                f'Test image mode: {len(self._test_images)} images'
                f' from {test_dir}')
            hz = self.get_parameter('test_image_hz').value
        else:
            # Live camera mode
            dev = self.get_parameter('device').value
            w   = self.get_parameter('width').value
            h   = self.get_parameter('height').value
            fps = self.get_parameter('fps').value

            self._cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
            self._cap.set(cv2.CAP_PROP_FOURCC,
                          cv2.VideoWriter_fourcc(*'MJPG'))
            self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  w)
            self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            self._cap.set(cv2.CAP_PROP_FPS, fps)

            if not self._cap.isOpened():
                self.get_logger().error(f'Cannot open /dev/video{dev}')
                raise RuntimeError('Camera open failed')

            aw = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            ah = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.get_logger().info(f'Camera opened: {aw}x{ah} @ {fps}fps')
            hz = self.get_parameter('publish_hz').value

        # ── publishers ────────────────────────────────────────────────────
        self._pub_pos = self.create_publisher(Point, '/ball/position',     10)
        self._pub_cup = self.create_publisher(Bool,  '/ball/cup_detected', 10)
        self._pub_img = self.create_publisher(Image, '/ball/image',        10)
        self._bridge  = CvBridge()

        # ── smoothed cup state ────────────────────────────────────────────
        self._ellipse = None

        self._timer = self.create_timer(1.0 / hz, self._tick)
        self.get_logger().info('ball_detector_node v2 ready')

    # ── main loop ─────────────────────────────────────────────────────────
    def _tick(self):
        if self._test_images:
            # Static image mode — cycle through images
            path  = self._test_images[self._test_idx]
            frame = cv2.imread(path)
            if frame is None:
                self.get_logger().warn(f'Cannot read {path}')
                return
            self._test_idx = (self._test_idx + 1) % len(self._test_images)
            # Don't reset ellipse between images — keeps marker stable
            self.get_logger().info(
                f'Test image {self._test_idx}/{len(self._test_images)}: '
                f'{path.split("/")[-1]}',
                throttle_duration_sec=0.4)
        else:
            # Live camera mode
            ret, frame = self._cap.read()
            if not ret:
                self.get_logger().warn('Frame grab failed',
                                       throttle_duration_sec=2.0)
                return

        # Apply ROI crop if configured
        roi_frame, roi_offset = self._apply_roi(frame)

        gray = cv2.cvtColor(roi_frame, cv2.COLOR_BGR2GRAY)

        cup_found = self._detect_cup(gray)
        ball_pos  = None
        if cup_found:
            ball_pos = self._detect_ball(gray)

        # ── publish ───────────────────────────────────────────────────────
        cup_msg      = Bool()
        cup_msg.data = cup_found
        self._pub_cup.publish(cup_msg)

        pos_msg = Point()
        if ball_pos is not None and self._ellipse is not None:
            cx, cy, a, b, angle = self._ellipse
            # Normalise ball position relative to ellipse centre and axes
            # Use semi-major axis (a) as the normalisation radius so that
            # a value of 1.0 = edge of ellipse along major axis
            dx = ball_pos[0] - cx
            dy = ball_pos[1] - cy
            pos_msg.x = dx / a
            pos_msg.y = dy / b
            pos_msg.z = 0.0
        else:
            pos_msg.x = 0.0
            pos_msg.y = 0.0
            pos_msg.z = -1.0

        self._pub_pos.publish(pos_msg)

        # ── debug image ───────────────────────────────────────────────────
        if self.get_parameter('show_debug').value:
            debug = self._draw_debug(roi_frame, ball_pos)
        else:
            debug = roi_frame

        img_msg = self._bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        img_msg.header.stamp    = self.get_clock().now().to_msg()
        img_msg.header.frame_id = 'camera_frame'
        self._pub_img.publish(img_msg)

    # ── ROI ───────────────────────────────────────────────────────────────
    def _apply_roi(self, frame):
        x = self.get_parameter('roi_x').value
        y = self.get_parameter('roi_y').value
        w = self.get_parameter('roi_w').value
        h = self.get_parameter('roi_h').value
        fh, fw = frame.shape[:2]
        if w <= 0 or h <= 0:
            return frame, (0, 0)
        x  = max(0, min(x, fw - 1))
        y  = max(0, min(y, fh - 1))
        w  = min(w, fw - x)
        h  = min(h, fh - y)
        return frame[y:y+h, x:x+w], (x, y)

    # ── cup detection ──────────────────────────────────────────────────────
    def _detect_cup(self, gray) -> bool:
        """
        Find the cup as the largest dark elliptical contour in the frame.
        Works correctly when camera views cup at an angle (ellipse not circle).
        """
        thresh     = self.get_parameter('cup_dark_thresh').value
        min_area   = self.get_parameter('cup_min_area').value
        max_area   = self.get_parameter('cup_max_area').value
        min_ratio  = self.get_parameter('cup_min_ellipse_ratio').value
        alpha      = self.get_parameter('cup_smooth_alpha').value

        # Threshold to isolate dark regions (cup interior is dark)
        _, dark_mask = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY_INV)

        # Morphological closing to fill gaps in cup interior
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
        dark_mask = cv2.morphologyEx(dark_mask, cv2.MORPH_CLOSE, kernel)

        # Find contours
        contours, _ = cv2.findContours(
            dark_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return self._ellipse is not None

        # Filter contours by area and ellipse quality
        best_ellipse = None
        best_area    = 0

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < min_area or area > max_area:
                continue
            # Need at least 5 points to fit ellipse
            if len(cnt) < 5:
                continue

            ellipse = cv2.fitEllipse(cnt)
            (cx, cy), (w_ax, h_ax), angle = ellipse

            # Semi-axes
            a = max(w_ax, h_ax) / 2.0   # semi-major
            b = min(w_ax, h_ax) / 2.0   # semi-minor
            ratio = b / a if a > 0 else 0

            # Reject very thin slivers (not a cup)
            if ratio < min_ratio:
                continue

            if area > best_area:
                best_area    = area
                best_ellipse = (cx, cy, a, b, angle)

        if best_ellipse is None:
            return self._ellipse is not None

        cx, cy, a, b, angle = best_ellipse

        if self._ellipse is None:
            self._ellipse = best_ellipse
        else:
            # Exponential moving average smoothing
            ocx, ocy, oa, ob, oangle = self._ellipse
            self._ellipse = (
                alpha * cx    + (1 - alpha) * ocx,
                alpha * cy    + (1 - alpha) * ocy,
                alpha * a     + (1 - alpha) * oa,
                alpha * b     + (1 - alpha) * ob,
                alpha * angle + (1 - alpha) * oangle,
            )

        return True

    # ── ball detection ─────────────────────────────────────────────────────
    def _detect_ball(self, gray):
        """
        Detect ball bearing using centroid of all bright pixels inside
        the cup ellipse. Handles multiple specular highlights (the
        'two eyes' effect from LED reflections) by finding their
        combined centroid.
        Returns (px, py) in ROI-frame pixel coords, or None.
        """
        if self._ellipse is None:
            return None

        cx, cy, a, b, angle = self._ellipse
        margin        = self.get_parameter('ball_margin').value
        bright_thresh = self.get_parameter('ball_bright_thresh').value
        min_area      = self.get_parameter('ball_min_area').value

        h, w = gray.shape

        # Build ellipse mask inset by margin to avoid detecting rim
        mask = np.zeros((h, w), dtype=np.uint8)
        cv2.ellipse(mask,
                    (int(cx), int(cy)),
                    (max(int(a * (1 - margin)), 1),
                     max(int(b * (1 - margin)), 1)),
                    angle, 0, 360, 255, -1)

        # Threshold bright pixels inside cup
        _, bright = cv2.threshold(gray, bright_thresh, 255, cv2.THRESH_BINARY)
        ball_region = cv2.bitwise_and(bright, bright, mask=mask)

        # Need minimum bright pixel count to be meaningful
        bright_count = cv2.countNonZero(ball_region)
        if bright_count < min_area:
            return None

        # Centroid of ALL bright pixels — handles 1 or 2 specular dots
        M = cv2.moments(ball_region)
        if M['m00'] > 0:
            bx = int(M['m10'] / M['m00'])
            by = int(M['m01'] / M['m00'])
            return (bx, by)

        return None

    # ── debug drawing ──────────────────────────────────────────────────────
    def _draw_debug(self, frame, ball_pos):
        out = frame.copy()

        if self._ellipse is not None:
            cx, cy, a, b, angle = self._ellipse
            # Draw fitted ellipse — green
            cv2.ellipse(out,
                        (int(cx), int(cy)),
                        (max(int(a), 1), max(int(b), 1)),
                        angle, 0, 360, (0, 255, 0), 2)
            # Centre crosshair
            cv2.circle(out, (int(cx), int(cy)), 3, (0, 255, 0), -1)
            cv2.line(out, (int(cx)-12, int(cy)),
                     (int(cx)+12, int(cy)), (0, 255, 0), 1)
            cv2.line(out, (int(cx), int(cy)-12),
                     (int(cx), int(cy)+12), (0, 255, 0), 1)
            # Inset ellipse showing ball search region
            cv2.ellipse(out,
                        (int(cx), int(cy)),
                        (max(int(a * 0.92), 1), max(int(b * 0.92), 1)),
                        angle, 0, 360, (0, 200, 0), 1)

        if ball_pos is not None:
            bx, by = ball_pos
            cv2.circle(out, (bx, by), 8,  (0, 0, 255),  2)
            cv2.circle(out, (bx, by), 2,  (0, 0, 255), -1)

            if self._ellipse is not None:
                cx, cy, a, b, _ = self._ellipse
                cv2.line(out, (int(cx), int(cy)),
                         (bx, by), (0, 165, 255), 1)
                nx = (bx - cx) / a
                ny = (by - cy) / b
                cv2.putText(out,
                            f'ball ({nx:+.2f}, {ny:+.2f})',
                            (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                            (0, 0, 255), 2)
        else:
            cv2.putText(out, 'ball: not detected', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        if self._ellipse is None:
            cv2.putText(out, 'cup: not detected', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cx, cy, a, b, angle = self._ellipse
            cv2.putText(out,
                        f'cup a={a:.0f} b={b:.0f} ang={angle:.0f}',
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1)

        return out

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BallDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
