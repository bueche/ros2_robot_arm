#!/usr/bin/env python3
"""
ball_marker_node.py
-------------------
Subscribes to ball position (from ball_detector_node) and IMU tilt error
(from imu_balance_node) and publishes RViz MarkerArray for visualisation.

Markers published on /ball/markers (visualization_msgs/MarkerArray):
  ID 0 — Ball sphere    : red sphere at ball position within cup frame
  ID 1 — Tilt arrow     : orange arrow showing IMU tilt direction/magnitude
  ID 2 — Cup ring       : green circle showing cup boundary
  ID 3 — Status text    : detection status overlay

The ball marker is positioned relative to the hand_link frame so it moves
with the arm in RViz. The tilt arrow is attached to the same frame.

Subscribes:
  /ball/position        (geometry_msgs/Point)     from ball_detector_node
  /ball/cup_detected    (std_msgs/Bool)            from ball_detector_node
  /imu/balance_error    (geometry_msgs/Vector3)    from imu_balance_node
                                                   x=pitch_err, y=roll_err (rad)

Parameters:
  hand_link_frame   TF frame to attach markers to   default: hand_link
  cup_radius_m      Physical cup radius in metres    default: 0.05  (5cm)
  cup_depth_m       Cup depth offset below hand_link default: 0.02
  ball_radius_m     Ball bearing radius in metres    default: 0.006 (6mm)
  tilt_arrow_scale  Scale tilt arrow length          default: 0.1
  marker_lifetime   Seconds before marker expires    default: 0.1
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class BallMarkerNode(Node):

    def __init__(self):
        super().__init__('ball_marker_node')

        # ── parameters ────────────────────────────────────────────────────
        self.declare_parameter('hand_link_frame',  'cup_plate_link')
        self.declare_parameter('cup_radius_m',     0.02965)  # 29.65mm plate radius
        self.declare_parameter('cup_depth_m',      0.02)
        self.declare_parameter('ball_radius_m',    0.006)
        self.declare_parameter('tilt_arrow_scale', 0.10)
        self.declare_parameter('marker_lifetime',  0.5)

        # ── state ─────────────────────────────────────────────────────────
        self._ball_pos     = None   # Point (normalised, z=-1 if not detected)
        self._cup_detected = False
        self._tilt         = None   # Vector3 (pitch, roll errors in rad)

        # ── callback group — isolates callbacks from DDS discovery traffic ──
        self._cb_group = ReentrantCallbackGroup()

        # ── subscribers ───────────────────────────────────────────────────
        self.create_subscription(Point,   '/ball/position',
                                 self._ball_cb,  10,
                                 callback_group=self._cb_group)
        self.create_subscription(Bool,    '/ball/cup_detected',
                                 self._cup_cb,   10,
                                 callback_group=self._cb_group)
        self.create_subscription(Vector3, '/imu/balance_error',
                                 self._imu_cb,   10,
                                 callback_group=self._cb_group)

        # ── publisher ─────────────────────────────────────────────────────
        self._pub = self.create_publisher(MarkerArray, '/ball/markers', 10)

        # Publish at 30Hz regardless of incoming message rate
        self._timer = self.create_timer(
            1.0 / 30.0, self._publish_markers,
            callback_group=self._cb_group)
        self.get_logger().info('ball_marker_node ready')

    # ── callbacks ─────────────────────────────────────────────────────────
    def _ball_cb(self, msg: Point):
        self._ball_pos = msg

    def _cup_cb(self, msg: Bool):
        self._cup_detected = msg.data

    def _imu_cb(self, msg: Vector3):
        self._tilt = msg

    # ── marker publishing ─────────────────────────────────────────────────
    def _publish_markers(self):
        p         = lambda name: self.get_parameter(name).value
        frame     = p('hand_link_frame')
        cup_r     = p('cup_radius_m')
        cup_d     = p('cup_depth_m')
        ball_r    = p('ball_radius_m')
        arr_scale = p('tilt_arrow_scale')
        lifetime  = p('marker_lifetime')

        # Use zero timestamp so RViz uses latest TF rather than
        # trying to match a specific time (prevents transform errors)
        from builtin_interfaces.msg import Time as RosTime
        now = RosTime()  # zero time = use latest available transform
        markers = MarkerArray()

        # ── ID 2: Cup ring (always shown if we have a frame) ──────────────
        cup_marker = Marker()
        cup_marker.header.frame_id = frame
        cup_marker.header.stamp    = now
        cup_marker.ns              = 'ball_tracking'
        cup_marker.id              = 2
        cup_marker.type            = Marker.CYLINDER
        cup_marker.action          = Marker.ADD
        cup_marker.pose.position.x = 0.0
        cup_marker.pose.position.y = 0.0
        cup_marker.pose.position.z = 0.0   # plate surface in cup_plate_link frame
        # Identity — cup_plate_link Z already points up out of plate
        cup_marker.pose.orientation.x = 0.0
        cup_marker.pose.orientation.y = 0.0
        cup_marker.pose.orientation.z = 0.0
        cup_marker.pose.orientation.w = 1.0
        cup_marker.scale.x         = cup_r * 2.0   # diameter
        cup_marker.scale.y         = cup_r * 2.0
        cup_marker.scale.z         = 0.003          # thin disc (3mm)
        # Green if cup detected, grey if lost
        if self._cup_detected:
            cup_marker.color = ColorRGBA(r=0.0, g=0.9, b=0.2, a=0.4)
        else:
            cup_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.2)
        cup_marker.lifetime = rclpy.duration.Duration(
            seconds=lifetime).to_msg()
        markers.markers.append(cup_marker)

        # ── ID 0: Ball sphere ─────────────────────────────────────────────
        ball_marker = Marker()
        ball_marker.header.frame_id = frame
        ball_marker.header.stamp    = now
        ball_marker.ns              = 'ball_tracking'
        ball_marker.id              = 0
        ball_marker.type            = Marker.SPHERE
        ball_marker.action          = Marker.ADD
        ball_marker.scale.x         = ball_r * 2.0
        ball_marker.scale.y         = ball_r * 2.0
        ball_marker.scale.z         = ball_r * 2.0
        ball_marker.lifetime        = rclpy.duration.Duration(
            seconds=lifetime).to_msg()

        ball_detected = (self._ball_pos is not None and
                         self._ball_pos.z >= 0.0)

        if ball_detected:
            # Map normalised camera coords to cup_plate_link frame
            # cup_plate_link: X=forward, Y=left, Z=up out of plate
            # Camera norm_x and norm_y mapping determined empirically:
            #   norm_x (+right on screen) → plate X
            #   norm_y (+down on screen)  → plate Y (negated)
            # These signs can be flipped with invert_x/invert_y parameters
            # Camera X maps to robot Y, camera Y maps to robot X
            bx = -self._ball_pos.y * cup_r
            by = -self._ball_pos.x * cup_r
            ball_marker.pose.position.x = bx
            ball_marker.pose.position.y = by
            ball_marker.pose.position.z = ball_r   # sits on plate surface
            ball_marker.color = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.9)
        else:
            # Park ball marker at centre, ghost it out
            ball_marker.pose.position.x = 0.0
            ball_marker.pose.position.y = 0.0
            ball_marker.pose.position.z = ball_r
            ball_marker.color = ColorRGBA(r=0.5, g=0.5, b=0.5, a=0.2)

        ball_marker.pose.orientation.w = 1.0
        markers.markers.append(ball_marker)

        # ── ID 1: Tilt arrow (from IMU) — always published, invisible if no IMU
        arrow = Marker()
        arrow.header.frame_id = frame
        arrow.header.stamp    = now
        arrow.ns              = 'ball_tracking'
        arrow.id              = 1
        arrow.type            = Marker.ARROW
        arrow.action          = Marker.ADD
        arrow.lifetime        = rclpy.duration.Duration(
            seconds=lifetime).to_msg()

        from geometry_msgs.msg import Point as GPoint
        start = GPoint()
        start.x = 0.0; start.y = 0.0; start.z = ball_r

        end = GPoint()

        if self._tilt is not None:
            pitch    = self._tilt.x
            roll     = self._tilt.y
            tilt_mag = math.sqrt(pitch**2 + roll**2)
            end.x = -pitch * arr_scale
            end.y = -roll  * arr_scale
            end.z = ball_r
            intensity = min(tilt_mag / 0.2, 1.0)
            arrow.color = ColorRGBA(
                r=intensity, g=1.0 - intensity * 0.5, b=0.0, a=0.9)
        else:
            # No IMU — publish zero-length invisible arrow to keep array size constant
            end.x = 0.0; end.y = 0.0; end.z = ball_r
            arrow.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)

        arrow.points = [start, end]
        arrow.scale.x = 0.004
        arrow.scale.y = 0.008
        arrow.scale.z = 0.010
        markers.markers.append(arrow)

        # ── ID 3: Status text ─────────────────────────────────────────────
        text = Marker()
        text.header.frame_id = frame
        text.header.stamp    = now
        text.ns              = 'ball_tracking'
        text.id              = 3
        text.type            = Marker.TEXT_VIEW_FACING
        text.action          = Marker.ADD
        text.pose.position.x = 0.0
        text.pose.position.y = 0.0
        text.pose.position.z = cup_r + 0.02   # above the cup
        text.pose.orientation.w = 1.0
        text.scale.z         = 0.02           # text height in metres
        text.lifetime        = rclpy.duration.Duration(
            seconds=lifetime).to_msg()

        status_parts = []
        if not self._cup_detected:
            status_parts.append('cup: LOST')
        if not ball_detected:
            status_parts.append('ball: LOST')
        if self._tilt is not None:
            pitch_deg = math.degrees(self._tilt.x)
            roll_deg  = math.degrees(self._tilt.y)
            status_parts.append(f'P:{pitch_deg:+.1f}° R:{roll_deg:+.1f}°')

        text.text  = '  '.join(status_parts) if status_parts else 'OK'
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
        markers.markers.append(text)

        self._pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = BallMarkerNode()
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
