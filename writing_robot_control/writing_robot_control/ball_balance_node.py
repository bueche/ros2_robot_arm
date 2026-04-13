#!/usr/bin/env python3
"""
ball_balance_node.py — Combined camera + IMU ball balancing controller
======================================================================

Replaces imu_balance_node for ball balancing. Uses camera ball position
as the primary error signal and IMU tilt as a derivative/feedforward signal.

Architecture:
  /ball/position      (geometry_msgs/Point)    — camera: where ball IS
  /imu/balance_error  (geometry_msgs/Vector3)  — IMU: cup tilt pitch/roll
  /joint_states       (sensor_msgs/JointState) — current joint positions
  /arm_state          (std_msgs/String)         — MOVING / SETTLED

Publishes:
  /imu/balance_cmd    (geometry_msgs/Vector3)  — corrections for wrist_balance_controller
  /balance_enabled    (std_msgs/Bool)           — gate for wrist_balance_controller
  /imu/is_stable      (std_msgs/Bool)           — ball near centre
  /ball/balance_error (geometry_msgs/Vector3)   — (ball_x, ball_y, magnitude) for debug

Control law:
  error_x = ball_pos.x   (positive = ball right  → roll  cup left  → +wrist_roll)
  error_y = ball_pos.y   (positive = ball toward robot → flex forward → -wrist_flex)

  flex_cmd = -Kp_flex * error_y  - Kd_flex * imu_pitch
  roll_cmd = +Kp_roll * error_x  + Kd_roll * imu_roll

  The IMU term acts as derivative — it damps oscillation by sensing the
  direction the cup is already tilting before the ball reaches the edge.

Coordinate conventions (validated against balance_v1.yaml):
  wrist_roll  decreasing → tilt right,  increasing → tilt left
  wrist_flex  decreasing → tilt forward (away from robot)

  ball_pos.x  positive → ball right  → need roll left  → roll_cmd positive (+wrist_roll)
  ball_pos.y  positive → ball near   → need flex fwd   → flex_cmd negative (-wrist_flex)

Parameters:
  kp_flex         Camera P gain, flex axis         default: 0.3
  kp_roll         Camera P gain, roll axis         default: 0.3
  kd_flex         IMU D gain, flex axis            default: 0.1
  kd_roll         IMU D gain, roll axis            default: 0.1
  ki_flex         Integral gain, flex axis         default: 0.01
  ki_roll         Integral gain, roll axis         default: 0.01
  max_cmd         Max correction command (rad/s)   default: 0.3
  deadband        Ball position deadband (0..1)    default: 0.05
  stable_thresh   Ball within this = stable        default: 0.15
  publish_hz      Control loop rate (Hz)           default: 10.0
                  (limited by camera rate ~1.4Hz ONNX CPU)
  settle_delay    Seconds after SETTLED before PID default: 0.5
  camera_timeout  Seconds before camera stale      default: 2.0
  imu_timeout     Seconds before IMU stale         default: 0.5
  ball_lost_timeout  Seconds ball undetected before suspend  default: 1.0
  attempt_timeout    Seconds before giving up balancing      default: 5.0
  dry_run         Log only, don't publish cmd      default: False
  use_imu         Use IMU derivative term          default: True

Usage:
  ros2 run writing_robot_control ball_balance_node

  # Dry run first to verify corrections:
  ros2 run writing_robot_control ball_balance_node --ros-args -p dry_run:=true

  # Tune gains:
  ros2 param set /ball_balance_node kp_flex 0.5
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class BallBalanceNode(Node):

    def __init__(self):
        super().__init__('ball_balance_node')

        # Parameters
        self.declare_parameter('kp_flex',        0.3)
        self.declare_parameter('kp_roll',        0.3)
        self.declare_parameter('kd_flex',        0.1)
        self.declare_parameter('kd_roll',        0.1)
        self.declare_parameter('ki_flex',        0.01)
        self.declare_parameter('ki_roll',        0.01)
        self.declare_parameter('max_cmd',        0.3)
        self.declare_parameter('deadband',       0.05)
        self.declare_parameter('stable_thresh',  0.15)
        self.declare_parameter('publish_hz',     10.0)
        self.declare_parameter('settle_delay',   0.5)
        self.declare_parameter('camera_timeout',    2.0)
        self.declare_parameter('imu_timeout',       0.5)
        self.declare_parameter('ball_lost_timeout', 1.0)
        self.declare_parameter('attempt_timeout',   5.0)
        self.declare_parameter('dry_run',           False)
        self.declare_parameter('use_imu',           True)

        # State
        self._lock              = threading.Lock()

        # Camera
        self._ball_pos              = None   # Point (x, y, z)
        self._ball_time             = None   # monotonic timestamp of last fresh frame
        self._last_valid_ball_time  = None   # monotonic timestamp of last z>=0 detection

        # Attempt tracking
        self._pid_started_at        = None   # when PID became active this session

        # IMU
        self._imu_pitch         = 0.0   # rad
        self._imu_roll          = 0.0   # rad
        self._imu_time          = None

        # Arm state
        self._arm_state         = 'MOVING'
        self._settled_at        = None
        self._pid_active        = False

        # Integrals
        self._integral_flex     = 0.0
        self._integral_roll     = 0.0
        self._last_control_time = None

        # Publishers
        self._pub_cmd     = self.create_publisher(
            Vector3, '/imu/balance_cmd',    10)
        self._pub_enabled = self.create_publisher(
            Bool,    '/balance_enabled',    10)
        self._pub_stable  = self.create_publisher(
            Bool,    '/imu/is_stable',      10)
        self._pub_err     = self.create_publisher(
            Vector3, '/ball/balance_error', 10)

        # Subscribers
        self.create_subscription(
            Point,      '/ball/position',
            self._ball_cb,       10)
        self.create_subscription(
            Vector3,    '/imu/balance_error',
            self._imu_cb,        10)
        self.create_subscription(
            String,     '/arm_state',
            self._arm_state_cb,  10)
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10)

        # Control timer
        hz = self.get_parameter('publish_hz').value
        self._timer = self.create_timer(1.0 / hz, self._control_loop)

        self.get_logger().info('ball_balance_node started.')
        self.get_logger().info(
            'Subscriptions: /ball/position  /imu/balance_error  /arm_state')
        self.get_logger().info(
            'Publishes: /imu/balance_cmd  /balance_enabled  /imu/is_stable')
        if self.get_parameter('dry_run').value:
            self.get_logger().info(
                'DRY RUN — corrections logged but NOT published to /imu/balance_cmd')

    # Callbacks

    def _ball_cb(self, msg: Point):
        """Camera ball position callback."""
        with self._lock:
            if msg.z < 0:
                # z = -1 means ball not detected — don't update position
                # but DO update ball_time so camera_timeout doesn't fire
                self._ball_time = time.monotonic()
                return
            self._ball_pos             = msg
            self._ball_time            = time.monotonic()
            self._last_valid_ball_time = time.monotonic()

    def _imu_cb(self, msg: Vector3):
        """IMU balance error callback (pitch, roll from imu_balance_node)."""
        with self._lock:
            self._imu_pitch = msg.x
            self._imu_roll  = msg.y
            self._imu_time  = time.monotonic()

    def _arm_state_cb(self, msg: String):
        """Arm state machine: MOVING or SETTLED."""
        state = msg.data
        if state != self._arm_state:
            self._arm_state = state
            self.get_logger().info(f'Arm state → {state}')
            if state == 'MOVING':
                self._pid_active           = False
                self._settled_at           = None
                self._pid_started_at       = None
                self._last_valid_ball_time = None
                self._integral_flex        = 0.0
                self._integral_roll        = 0.0
                self._pub_enabled.publish(Bool(data=False))
            elif state == 'SETTLED':
                self._settled_at = time.monotonic()

    def _joint_state_cb(self, msg: JointState):
        """Joint states — kept for future FK setpoint computation."""
        pass

    # Control loop

    def _control_loop(self):
        now = time.monotonic()

        # Check settle delay
        if (not self._pid_active
                and self._arm_state == 'SETTLED'
                and self._settled_at is not None):
            delay = self.get_parameter('settle_delay').value
            if now - self._settled_at >= delay:
                self._pid_active     = True
                self._pid_started_at = now
                self.get_logger().info('PID active — ball balancing started.')
                self._pub_enabled.publish(Bool(data=True))

        if not self._pid_active:
            return

        with self._lock:
            ball_pos  = self._ball_pos
            ball_time = self._ball_time
            imu_pitch = self._imu_pitch
            imu_roll  = self._imu_roll
            imu_time  = self._imu_time

        # Camera freshness check
        cam_timeout = self.get_parameter('camera_timeout').value
        if ball_pos is None or ball_time is None:
            self.get_logger().warn(
                'No camera data yet — waiting.',
                throttle_duration_sec=5.0)
            return
        if now - ball_time > cam_timeout:
            self.get_logger().warn(
                f'Camera data stale ({now - ball_time:.1f}s) — skipping.',
                throttle_duration_sec=2.0)
            return

        # Ball-lost check — ball undetected for too long
        ball_lost_timeout = self.get_parameter('ball_lost_timeout').value
        if (self._last_valid_ball_time is not None and
                now - self._last_valid_ball_time > ball_lost_timeout):
            self.get_logger().warn(
                f'Ball lost for {now - self._last_valid_ball_time:.1f}s '
                f'(>{ball_lost_timeout:.1f}s) — suspending PID.',
                throttle_duration_sec=1.0)
            return

        # Attempt timeout — give up if not achieved within limit.
        # IMPORTANT: clear _settled_at so the settle_delay check cannot
        # immediately restart PID while the arm is still in SETTLED state.
        attempt_timeout = self.get_parameter('attempt_timeout').value
        if (self._pid_started_at is not None and
                now - self._pid_started_at > attempt_timeout):
            self._pid_active     = False
            self._pid_started_at = None
            self._settled_at     = None   # prevents immediate PID restart
            self._integral_flex  = 0.0
            self._integral_roll  = 0.0
            self._pub_enabled.publish(Bool(data=False))
            self.get_logger().warn(
                f'Attempt timeout ({attempt_timeout:.0f}s) — '
                f'PID suspended. Waiting for next SETTLED transition.')
            return

        # Compute errors
        error_x = float(ball_pos.x)   # positive = ball right
        error_y = float(ball_pos.y)   # positive = ball toward robot

        # Apply deadband
        deadband = self.get_parameter('deadband').value
        if abs(error_x) < deadband:
            error_x = 0.0
        if abs(error_y) < deadband:
            error_y = 0.0

        # Magnitude for stability check
        magnitude = math.sqrt(error_x**2 + error_y**2)

        # Publish balance error for monitoring
        err_msg = Vector3()
        err_msg.x = error_x
        err_msg.y = error_y
        err_msg.z = magnitude
        self._pub_err.publish(err_msg)

        # Stability
        stable_thresh = self.get_parameter('stable_thresh').value
        is_stable = magnitude < stable_thresh
        self._pub_stable.publish(Bool(data=is_stable))

        if error_x == 0.0 and error_y == 0.0:
            # Within deadband — reset integrals slowly
            self._integral_flex *= 0.9
            self._integral_roll *= 0.9
            return

        # dt for integral
        if self._last_control_time is None:
            dt = 0.0
        else:
            dt = now - self._last_control_time
            dt = max(dt, 1e-3)
        self._last_control_time = now

        # Integral
        ki_flex = self.get_parameter('ki_flex').value
        ki_roll = self.get_parameter('ki_roll').value
        max_cmd = self.get_parameter('max_cmd').value

        self._integral_flex += error_y * dt
        self._integral_roll += error_x * dt

        # Anti-windup clamp
        max_integral = max_cmd / max(ki_flex, 1e-6)
        self._integral_flex = max(-max_integral,
                                   min(max_integral, self._integral_flex))
        self._integral_roll = max(-max_integral,
                                   min(max_integral, self._integral_roll))

        # PID computation
        kp_flex = self.get_parameter('kp_flex').value
        kp_roll = self.get_parameter('kp_roll').value
        kd_flex = self.get_parameter('kd_flex').value
        kd_roll = self.get_parameter('kd_roll').value
        use_imu = self.get_parameter('use_imu').value

        # IMU derivative term (damp oscillation)
        imu_fresh = (use_imu and imu_time is not None and
                     now - imu_time < self.get_parameter('imu_timeout').value)
        d_flex = imu_pitch if imu_fresh else 0.0
        d_roll = imu_roll  if imu_fresh else 0.0

        # Control law (validated signs from balance_v1.yaml):
        #   ball right (+x) → roll left → +wrist_roll → roll_cmd positive
        #   ball near  (+y) → flex fwd  → -wrist_flex → flex_cmd negative
        flex_cmd = (-kp_flex * error_y
                    - ki_flex * self._integral_flex
                    - kd_flex * d_flex)
        roll_cmd = (+kp_roll * error_x
                    + ki_roll * self._integral_roll
                    + kd_roll * d_roll)

        # Clamp
        flex_cmd = max(-max_cmd, min(max_cmd, flex_cmd))
        roll_cmd = max(-max_cmd, min(max_cmd, roll_cmd))

        # Publish or log
        if self.get_parameter('dry_run').value:
            self.get_logger().info(
                f'DRY RUN | ball=({error_x:+.3f},{error_y:+.3f}) '
                f'mag={magnitude:.3f} '
                f'flex_cmd={flex_cmd:+.4f} roll_cmd={roll_cmd:+.4f} '
                f'stable={is_stable}',
                throttle_duration_sec=0.5)
            return

        cmd = Vector3()
        cmd.x = flex_cmd   # → wrist_flex in wrist_balance_controller
        cmd.y = roll_cmd   # → wrist_roll in wrist_balance_controller
        cmd.z = 0.0
        self._pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = BallBalanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
