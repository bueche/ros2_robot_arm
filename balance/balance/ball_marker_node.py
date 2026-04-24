#!/usr/bin/env python3
"""
ball_marker_node.py (v13)
-------------------------
Subscribes to ball position, IMU tilt, arm state, PID enable flag, and PID
correction commands and publishes an RViz MarkerArray for visualisation.

Markers published on /ball/markers (visualization_msgs/MarkerArray):
  ID 0  Ball sphere         : red sphere at ball position within cup frame
  ID 2  Cup ring            : green disc showing cup boundary
  ID 3  Status row A (top)  : state + ball position  (color-coded by state)
                               anchored to base_link so it stays upright
  ID 4  PID correction arrow: blue arrow showing flex+roll correction vector
  ID 5  Status row B (below): PID cmd + IMU angles (grey)
                               anchored to base_link so it stays upright

Note: tilt arrow (formerly ID 1) removed — IMU pitch/roll values are raw
and frequently misleading due to uncalibrated orientation offset.

The ball marker is positioned relative to cup_plate_link so it moves with
the arm in RViz. All markers share the same frame.

Subscribes:
  /ball/position        (geometry_msgs/Point)     from ball_detector_oak
  /ball/cup_detected    (std_msgs/Bool)            from ball_detector_oak
  /imu/balance_error    (geometry_msgs/Vector3)    from imu_balance_node
                                                   x=pitch_err, y=roll_err (rad)
  /arm_state            (std_msgs/String)          MOVING | SETTLED
  /balance_enabled      (std_msgs/Bool)            PID active flag
  /imu/balance_cmd      (geometry_msgs/Vector3)    PID correction output
                                                   x=flex_cmd, y=roll_cmd

Parameters:
  hand_link_frame   TF frame to attach markers to   default: cup_plate_link
  cup_radius_m      Physical cup radius in metres    default: 0.02965
  cup_depth_m       Cup depth offset below hand_link default: 0.02
  ball_radius_m     Ball bearing radius in metres    default: 0.006
  tilt_arrow_scale  Scale tilt arrow length          default: 0.10
  cmd_arrow_scale   Scale PID cmd arrow length       default: 0.15
  marker_lifetime   Seconds before marker expires    default: 0.5
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Bool, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


class BallMarkerNode(Node):

    def __init__(self):
        super().__init__('ball_marker_node')

        # parameters 
        self.declare_parameter('hand_link_frame',  'cup_plate_link')
        self.declare_parameter('cup_radius_m',     0.02965)  # 29.65mm plate radius
        self.declare_parameter('cup_depth_m',      0.02)
        self.declare_parameter('ball_radius_m',    0.006)
        self.declare_parameter('tilt_arrow_scale', 0.10)
        self.declare_parameter('cmd_arrow_scale',  0.15)
        self.declare_parameter('marker_lifetime',  0.5)
        # Text overlay position in base_link frame (tune without rebuild)
        self.declare_parameter('text_x',   0.0)
        self.declare_parameter('text_y',   -0.25)
        self.declare_parameter('text_za',  0.35)   # row A (state+ball)
        self.declare_parameter('text_zb',  0.30)  # row B (cmd+imu)

        # state 
        self._ball_pos        = None   # Point (normalised, z=-1 if not detected)
        self._cup_detected    = False
        self._tilt            = None   # Vector3 (pitch, roll errors in rad)
        self._arm_state       = 'UNKNOWN'   # MOVING | SETTLED
        self._balance_enabled = False       # PID active
        self._balance_cmd     = None        # Vector3 (flex_cmd, roll_cmd)

        # callback group isolates callbacks from DDS discovery traffic 
        self._cb_group = ReentrantCallbackGroup()

        # subscribers 
        self.create_subscription(Point,   '/ball/position',
                                 self._ball_cb,  10,
                                 callback_group=self._cb_group)
        self.create_subscription(Bool,    '/ball/cup_detected',
                                 self._cup_cb,   10,
                                 callback_group=self._cb_group)
        self.create_subscription(Vector3, '/imu/balance_error',
                                 self._imu_cb,   10,
                                 callback_group=self._cb_group)
        self.create_subscription(String,  '/arm_state',
                                 self._arm_state_cb, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Bool,    '/balance_enabled',
                                 self._balance_enabled_cb, 10,
                                 callback_group=self._cb_group)
        self.create_subscription(Vector3, '/imu/balance_cmd',
                                 self._balance_cmd_cb, 10,
                                 callback_group=self._cb_group)

        # publisher 
        self._pub = self.create_publisher(MarkerArray, '/ball/markers', 10)

        # Publish at 30Hz regardless of incoming message rate
        self._timer = self.create_timer(
            1.0 / 30.0, self._publish_markers,
            callback_group=self._cb_group)
        self.get_logger().info('ball_marker_node ready')

    # callbacks 
    def _ball_cb(self, msg: Point):
        self._ball_pos = msg

    def _cup_cb(self, msg: Bool):
        self._cup_detected = msg.data

    def _imu_cb(self, msg: Vector3):
        self._tilt = msg

    def _arm_state_cb(self, msg: String):
        self._arm_state = msg.data

    def _balance_enabled_cb(self, msg: Bool):
        self._balance_enabled = msg.data

    def _balance_cmd_cb(self, msg: Vector3):
        self._balance_cmd = msg

    # marker publishing 
    def _publish_markers(self):
        p         = lambda name: self.get_parameter(name).value
        frame     = p('hand_link_frame')
        cup_r     = p('cup_radius_m')
        cup_d     = p('cup_depth_m')
        ball_r    = p('ball_radius_m')
        cmd_scale = p('cmd_arrow_scale')
        lifetime  = p('marker_lifetime')

        # Use zero timestamp so RViz uses latest TF rather than
        # trying to match a specific time (prevents transform errors)
        from builtin_interfaces.msg import Time as RosTime
        now = RosTime()  # zero time = use latest available transform
        markers = MarkerArray()

        # Delete stale tilt arrow (ID 1) from any previous version of this node
        # so RViz doesn't keep showing the old orange line.
        delete_old = Marker()
        delete_old.ns     = 'ball_tracking'
        delete_old.id     = 1
        delete_old.action = Marker.DELETE
        markers.markers.append(delete_old)

        # ID 2: Cup ring (always shown if we have a frame) 
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
        # Identity cup_plate_link Z already points up out of plate
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

        # ID 0: Ball sphere 
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
            #   norm_x (+right on screen) plate X
            #   norm_y (+down on screen)  plate Y (negated)
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

        # ID 3 & 5: Status text anchored to base_link 
        # Text is attached to base_link at a fixed world position so it stays
        # upright and visible regardless of wrist orientation.
        # Tune position with text_x/text_y/text_za/text_zb params at runtime.
        TEXT_FRAME = 'base_link'
        TEXT_X     = p('text_x')
        TEXT_Y     = p('text_y')
        TEXT_ZA    = p('text_za')
        TEXT_ZB    = p('text_zb')

        def _make_text_marker(mid, z_pos, content, color):
            t = Marker()
            t.header.frame_id = TEXT_FRAME
            t.header.stamp    = now
            t.ns              = 'ball_tracking'
            t.id              = mid
            t.type            = Marker.TEXT_VIEW_FACING
            t.action          = Marker.ADD
            t.pose.position.x = TEXT_X
            t.pose.position.y = TEXT_Y
            t.pose.position.z = z_pos
            t.pose.orientation.w = 1.0
            t.scale.z            = 0.015
            t.lifetime           = rclpy.duration.Duration(
                seconds=lifetime).to_msg()
            t.text  = content
            t.color = color
            return t

        # State color
        if self._arm_state == 'MOVING':
            state_str   = 'MOVING'
            state_color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.9)
        elif self._balance_enabled:
            state_str   = 'PID_ON'
            state_color = ColorRGBA(r=0.2, g=1.0, b=0.2, a=0.9)
        else:
            state_str   = 'PID_OFF'
            state_color = ColorRGBA(r=0.8, g=0.8, b=0.8, a=0.9)

        # Ball position — keep short, no spaces
        if ball_detected:
            ball_str = f'b:({self._ball_pos.x:+.2f},{self._ball_pos.y:+.2f})'
        else:
            ball_str = 'ball:LOST'

        # Row A: state + ball
        row_a = f'{state_str}__{ball_str}'
        # row_a = 'foo_bar'
        markers.markers.append(
            _make_text_marker(3, TEXT_ZA, row_a, state_color))

        # PID cmd — no spaces
        if self._balance_cmd is not None and self._balance_enabled:
            cmd_str = (f'f:{self._balance_cmd.x:+.3f}_'
                       f'r:{self._balance_cmd.y:+.3f}')
        else:
            cmd_str = 'cmd:--'

        # IMU angles — no spaces
        if self._tilt is not None:
            pitch_deg = math.degrees(self._tilt.x)
            roll_deg  = math.degrees(self._tilt.y)
            imu_str = f'imu:P:{pitch_deg:+.1f}_R:{roll_deg:+.1f}'
        else:
            imu_str = 'imu:--'

        # Row B: cmd + IMU
        row_b = f'{cmd_str}__{imu_str}'
        dim_color = ColorRGBA(r=0.9, g=0.9, b=0.9, a=0.7)
        markers.markers.append(
            _make_text_marker(5, TEXT_ZB, row_b, dim_color))

        # ID 4: PID correction arrow 
        # Shows the flex+roll correction vector being sent to wrist controller.
        # Blue when PID active, invisible when off.
        # flex_cmd maps to plate X, roll_cmd maps to plate Y
        # (same axis mapping as ball position above)
        cmd_arrow = Marker()
        cmd_arrow.header.frame_id = frame
        cmd_arrow.header.stamp    = now
        cmd_arrow.ns              = 'ball_tracking'
        cmd_arrow.id              = 4
        cmd_arrow.type            = Marker.ARROW
        cmd_arrow.action          = Marker.ADD
        cmd_arrow.lifetime        = rclpy.duration.Duration(
            seconds=lifetime).to_msg()

        from geometry_msgs.msg import Point as GPoint
        cmd_start = GPoint()
        cmd_start.x = 0.0
        cmd_start.y = 0.0
        cmd_start.z = ball_r * 2.0   # slightly above tilt arrow base

        cmd_end = GPoint()

        if self._balance_cmd is not None and self._balance_enabled:
            # flex_cmd (x in balance_cmd) tilts cup forward/back → plate X
            # roll_cmd (y in balance_cmd) tilts cup left/right  → plate Y
            cmd_end.x = -self._balance_cmd.x * cmd_scale
            cmd_end.y = -self._balance_cmd.y * cmd_scale
            cmd_end.z = ball_r * 2.0
            cmd_mag   = math.sqrt(self._balance_cmd.x**2 +
                                  self._balance_cmd.y**2)
            intensity = min(cmd_mag / 0.3, 1.0)
            cmd_arrow.color = ColorRGBA(
                r=0.0, g=0.3 * (1.0 - intensity),
                b=0.8 + 0.2 * intensity, a=0.9)
        else:
            # PID off — invisible zero-length arrow
            cmd_end.x = 0.0
            cmd_end.y = 0.0
            cmd_end.z = ball_r * 2.0
            cmd_arrow.color = ColorRGBA(r=0.0, g=0.0, b=0.0, a=0.0)

        cmd_arrow.points = [cmd_start, cmd_end]
        cmd_arrow.scale.x = 0.004   # shaft diameter
        cmd_arrow.scale.y = 0.010   # head diameter
        cmd_arrow.scale.z = 0.012   # head length
        markers.markers.append(cmd_arrow)

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
