#!/usr/bin/env python3
"""
ball_balance_node.py — Combined camera + IMU ball balancing controller
======================================================================

Replaces imu_balance_node for ball balancing. Uses camera ball position
as the primary error signal and IMU tilt as a derivative/feedforward signal.

Architecture:
  /ball/position      (geometry_msgs/Point)    — camera: where ball IS
  /imu/balance_error      (geometry_msgs/Vector3) — IMU: cup tilt angles (pitch, roll) rad
  /balance/cmd_delta      (geometry_msgs/Vector3) — wrist: commanded flex/roll delta per CORR
  /balance/achieved_delta (geometry_msgs/Vector3) — wrist: actually achieved flex/roll delta
  /imu/raw            (sensor_msgs/Imu)         — IMU: angular_velocity.x=roll_rate,
                                                   angular_velocity.y=pitch_rate (rad/s)
                                                   NOTE: x=roll, y=pitch — opposite to
                                                   ROS REP-103. See _imu_raw_cb.
  /joint_states       (sensor_msgs/JointState) — current joint positions
  /arm_state          (std_msgs/String)         — MOVING / SETTLED

Publishes:
  /imu/balance_cmd    (geometry_msgs/Vector3)  — corrections for wrist_balance_controller
  /balance_enabled    (std_msgs/Bool)           — gate for wrist_balance_controller
  /ball/is_centered       (std_msgs/Bool)          — True only when ball has been
                                                   continuously within stable_thresh
                                                   for centered_hold_time seconds.
                                                   Causes pose_test to advance immediately.
                                                   Distinct from /imu/is_stable (IMU-based).
  /ball/balance_error (geometry_msgs/Vector3)   — (ball_x, ball_y, magnitude) for debug
  /ball/pid_detail_flex (geometry_msgs/Vector3) — (P_flex, I_flex, D_flex) per tick
  /ball/pid_detail_roll (geometry_msgs/Vector3) — (P_roll, I_roll, D_roll) per tick
                                                   x+y+z = pre-clamp flex/roll cmd

Control law:
  error_x = ball_pos.x   (positive = ball right  → roll  cup left  → +wrist_roll)
  error_y = ball_pos.y   (positive = ball toward robot → flex forward → -wrist_flex)

  flex_cmd = +Kp_flex * error_y  - Kd_flex * imu_pitch_rate
  roll_cmd = +Kp_roll * error_x  - Kd_roll * imu_roll_rate

  imu_pitch_rate = angular_velocity.y from /imu/raw (rad/s) — true gyroscope rate.
  SIGN: D-term is SUBTRACTED so it opposes motion, providing true damping.
  When the cup is already tilting in the corrective direction (pitch_rate same
  sign as P command), D-term reduces total command magnitude — preventing overshoot.
  Adding D-term (wrong sign) would reinforce motion → positive feedback → oscillation.
  The IMU angle (from /imu/balance_error) is NOT used for D-term — that would
  act as a second P term biased by cup tilt, not a damping signal.

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
  jiggle_amplitude   Cmd amplitude per axis during jiggle (rad/s) default: 0.02
  jiggle_start_delay Seconds lost before jiggle starts            default: 2.0
  jiggle_hz          Jiggle rate Hz — should match correction_hz  default: 1.0
                     4-phase circular: flex+, roll+, flex-, roll- over 4 steps.
                     Wrist controller hard limits naturally clamp any axis near its wall.
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
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String


class BallBalanceNode(Node):

    def __init__(self):
        super().__init__('ball_balance_node')

        # Parameters
        self.declare_parameter('kp_flex',        0.3)
        self.declare_parameter('kp_roll',        0.3)
        self.declare_parameter('kd_flex',        0.02)  # conservative default — tune up carefully
        self.declare_parameter('kd_roll',        0.02)
        # d_term_source: where to read angular velocity for the D-term.
        #   'imu'    — /imu/raw angular_velocity (MPU-6050 gyro, 50Hz)
        #              KNOWN ISSUE: quantizes at 0.000133 rad/s/LSB — frozen
        #              at slow angular velocities typical of 1Hz corrections.
        #   'joints' — /balance/wrist_velocity (finite difference of joint_states, ~100Hz)
        #              More reliable for slow servo motion; directly measures
        #              what the servo did rather than cup vibration.
        #              Requires wrist_balance_controller_v25+.
        self.declare_parameter('d_term_source',  'joints')
        # Anti-windup decay factor applied when ball error changes sign (ball
        # crosses center axis). Decays the I-term to prevent windup from
        # pushing the ball further past center on overshoot.
        # 0.0 = full reset on sign change, 1.0 = no decay (disabled).
        # 0.2 = decay to 20% — recommended starting value.
        # Applied independently per axis: flex when error_y changes sign,
        # roll when error_x changes sign.
        self.declare_parameter('antiwindup_decay', 0.2)
        self.declare_parameter('ki_flex',        0.01)
        self.declare_parameter('ki_roll',        0.01)
        self.declare_parameter('max_cmd',        0.3)
        self.declare_parameter('deadband',       0.05)
        self.declare_parameter('stable_thresh',  0.15)
        self.declare_parameter('publish_hz',     15.0)
        self.declare_parameter('settle_delay',   0.5)
        self.declare_parameter('camera_timeout',    2.0)
        self.declare_parameter('imu_timeout',       0.5)
        self.declare_parameter('ball_lost_timeout', 1.0)
        self.declare_parameter('jiggle_amplitude',   0.02)  # rad/s per axis during jiggle
        self.declare_parameter('jiggle_start_delay', 2.0)   # seconds lost before jiggle starts
        self.declare_parameter('jiggle_hz',          1.0)   # jiggle rate — match correction_hz
        # Jiggle uses a 4-phase circular pattern: (+x,0), (0,+y), (-x,0), (0,-y).
        # This naturally avoids limit-slamming — any clamped axis just doesn't move
        # while the other axis still rocks the cup. No need to duplicate URDF limits.
        self.declare_parameter('attempt_timeout',     30.0)
        self.declare_parameter('centered_hold_time',  2.0)  # seconds ball must stay within stable_thresh
        self.declare_parameter('near_thresh',          0.30)  # near-center zone for summary stats
        self.declare_parameter('use_achievement',      True)  # scale commands by servo achievement ratio
        self.declare_parameter('achievement_alpha',    0.3)   # EMA weight for ratio update (0=frozen, 1=instant)
        self.declare_parameter('achievement_max_scale',3.0)   # max multiplier to prevent runaway
        self.declare_parameter('achievement_min_cmd',  0.003) # min |cmd| to avoid updating on HOLD steps

        # ki mode — three options, select via 'ki_mode' parameter:
        #   'ball'     — classic: accumulate ball_error × dt (standard PID integral)
        #   'servo'    — accumulate (commanded - achieved) servo delta per CORR step
        #                grows when servo underperforms, not just when ball is off-center
        #   'combined' — accumulate ball_error × (1 - achievement) × dt
        #                grows only when ball is wrong AND servo is underperforming
        #   'none'     — integral disabled (ki_flex=0 and ki_roll=0 still work as before)
        self.declare_parameter('ki_mode',  'ball')  # 'ball' | 'servo' | 'combined' | 'none'
        self.declare_parameter('ki_windup_limit', 0.5)  # clamp integral contribution to ±this
        self.declare_parameter('summary_hz',           1.0)   # rate of PID_SUMMARY log lines
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

        # Initialize ball time to now so ball-lost timer doesn't fire immediately
        # on first detection after a long startup period
        self._last_valid_ball_time  = time.monotonic()

        # IMU
        self._imu_pitch         = 0.0   # rad — tilt angle from /imu/balance_error
        self._imu_roll          = 0.0   # rad
        self._imu_time          = None
        self._imu_pitch_rate    = 0.0   # rad/s — low-pass filtered angular velocity
        self._imu_roll_rate     = 0.0   # rad/s
        self._imu_raw_time      = None

        # Joint-state-derived angular velocity from /balance/wrist_velocity.
        # Published by wrist_balance_controller_v25+ at ~100Hz.
        # Used as D-term source when d_term_source='joints'.
        self._wrist_flex_vel    = 0.0   # d(wrist_flex_pos)/dt  rad/s
        self._wrist_roll_vel    = 0.0   # d(wrist_roll_pos)/dt  rad/s
        self._wrist_vel_time    = None  # monotonic time of last message
        # wrist_roll_pos removed — 4-phase jiggle doesn't need limit tracking
        # Low-pass filter coefficient for gyro smoothing.
        # alpha=0.3 means 30% new reading + 70% history — smooths noise
        # without adding significant lag at 50Hz IMU rate.
        self._gyro_alpha        = 0.3

        # Arm state
        self._arm_state         = 'MOVING'
        self._settled_at        = None
        self._pid_active        = False

        # Integrals
        self._integral_flex     = 0.0
        self._integral_roll     = 0.0
        self._integral_flex_servo  = 0.0   # ki_mode='servo'/'combined': undelivered flex
        self._integral_roll_servo  = 0.0   # ki_mode='servo'/'combined': undelivered roll

        # Previous error signs — used for anti-windup sign-change detection.
        # None until first valid ball reading.
        self._prev_error_x_sign = None   # sign of error_x on previous tick
        self._prev_error_y_sign = None   # sign of error_y on previous tick
        self._last_control_time = None

        # Servo achievement ratio — exponential moving average of (achieved/commanded).
        # When the servo only achieves 50% of commanded, next command is scaled up 2×.
        # Separate per axis since flex and roll have different load characteristics.
        self._flex_achievement  = 1.0   # starts at 1.0 (assume perfect)
        self._roll_achievement  = 1.0
        self._last_cmd_flex     = None  # most recent commanded delta
        self._last_cmd_roll     = None

        # Centered hold tracking — ball must stay within stable_thresh
        # continuously for centered_hold_time seconds before /ball/is_centered
        # is published True and the PID session ends.
        self._centered_since    = None

        # Jiggle state — alternating roll direction when ball is lost
        self._jiggle_phase      = 0  # 0-3 cycles through 4-phase circular pattern
        self._last_jiggle_time  = None  # monotonic time of last jiggle publish
        self._ball_lost_neutralized = False  # True once neutral cmd sent on ball-lost

        # 1Hz PID_SUMMARY accumulator — collects every /ball/position reading
        # at full 15Hz rate during active PID for accurate proximity stats.
        self._acc_magnitudes    = []
        self._acc_valid         = 0
        self._acc_invalid       = 0
        self._acc_near          = 0
        self._acc_tight         = 0

        # Publishers
        self._pub_cmd     = self.create_publisher(
            Vector3, '/imu/balance_cmd',    10)
        self._pub_enabled = self.create_publisher(
            Bool,    '/balance_enabled',    10)
        # Publishes True only when ball has been held at center for
        # centered_hold_time seconds — distinct from /imu/is_stable (IMU tilt).
        self._pub_centered = self.create_publisher(
            Bool,    '/ball/is_centered',   10)
        self._pub_err     = self.create_publisher(
            Vector3, '/ball/balance_error', 10)
        # Publishes full PID term breakdown for pid_logger and offline analysis.
        # Two messages per control tick — one for flex axis, one for roll axis.
        # Format: x=P_term  y=I_term  z=D_term
        # Total cmd = x + y + z (before max_cmd clamp).
        self._pub_pid_flex = self.create_publisher(
            Vector3, '/ball/pid_detail_flex', 10)
        self._pub_pid_roll = self.create_publisher(
            Vector3, '/ball/pid_detail_roll', 10)

        # Subscribers
        self.create_subscription(
            Point,      '/ball/position',
            self._ball_cb,       10)
        self.create_subscription(
            Vector3,    '/imu/balance_error',
            self._imu_cb,        10)
        # Subscribe to servo achievement feedback from wrist_balance_controller.
        # cmd_delta = what was commanded, achieved_delta = what the servo did.
        # Used to scale future commands so persistent servo underperformance
        # is compensated — true closed-loop PID spirit.
        self.create_subscription(
            Vector3, '/balance/cmd_delta',
            self._cmd_delta_cb, 10)
        self.create_subscription(
            Vector3, '/balance/achieved_delta',
            self._achieved_delta_cb, 10)

        # Subscribe to /imu/raw for angular velocity (true D-term signal).
        # imu_balance_node publishes: angular_velocity.x = dr (roll rate),
        #                             angular_velocity.y = dp (pitch rate).
        # This is opposite to ROS REP-103 — see _imu_raw_cb for full explanation.
        self.create_subscription(
            Imu, '/imu/raw',
            self._imu_raw_cb, 10)
        # Joint-state-derived wrist velocity from wrist_balance_controller_v25+.
        # x = d(wrist_flex_pos)/dt, y = d(wrist_roll_pos)/dt  (rad/s, ~100Hz).
        # Used as D-term source when d_term_source='joints'.
        self.create_subscription(
            Vector3, '/balance/wrist_velocity',
            self._wrist_vel_cb, 10)
        self.create_subscription(
            String,     '/arm_state',
            self._arm_state_cb,  10)
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10)

        # Control timer
        hz = self.get_parameter('publish_hz').value
        self._timer = self.create_timer(1.0 / hz, self._control_loop)

        # 1Hz summary timer — logs PID_SUMMARY during active sessions
        summary_hz = self.get_parameter('summary_hz').value
        self.create_timer(1.0 / summary_hz, self._summary_loop)

        self.get_logger().info('ball_balance_node started.')
        self.get_logger().info(
            'Subscriptions: /ball/position  /imu/balance_error  /imu/raw  /arm_state')
        self.get_logger().info(
            f'D-term: kd_flex={self.get_parameter("kd_flex").value}  '
            f'kd_roll={self.get_parameter("kd_roll").value}  '
            f'use_imu={self.get_parameter("use_imu").value}  '
            f'(angular velocity from /imu/raw, NOT tilt angle)')
        self.get_logger().info(
            'Publishes: /imu/balance_cmd  /balance_enabled  /ball/is_centered')
        if self.get_parameter('dry_run').value:
            self.get_logger().info(
                'DRY RUN — corrections logged but NOT published to /imu/balance_cmd')

    # Callbacks

    def _ball_cb(self, msg: Point):
        """Camera ball position callback.

        Accumulates every reading into the 1Hz PID_SUMMARY stats.
        Tracks centered_hold_time to determine when to publish /ball/is_centered.
        """
        with self._lock:
            if msg.z < 0:
                # z=-1: ball not detected. Update ball_time so timeout logic
                # still fires correctly, but reset centered_since — the ball
                # cannot be considered at center if we cannot see it.
                self._ball_time     = time.monotonic()
                self._centered_since = None
                if self._pid_active:
                    self._acc_invalid += 1
                return
            self._ball_pos             = msg
            self._ball_time            = time.monotonic()
            self._last_valid_ball_time = time.monotonic()
            # Ball redetected — reset jiggle and neutral-cmd state so next
            # ball-lost event starts fresh.
            self._ball_lost_neutralized = False
            self._last_jiggle_time      = None
            self._jiggle_phase          = 0

            if self._pid_active:
                import math as _m
                mag = _m.sqrt(msg.x**2 + msg.y**2)
                st  = self.get_parameter('stable_thresh').value
                nt  = self.get_parameter('near_thresh').value
                self._acc_magnitudes.append(mag)
                self._acc_valid += 1
                if mag < nt: self._acc_near  += 1
                if mag < st: self._acc_tight += 1

    def _imu_cb(self, msg: Vector3):
        """IMU balance error callback (pitch, roll from imu_balance_node)."""
        with self._lock:
            self._imu_pitch = msg.x
            self._imu_roll  = msg.y
            self._imu_time  = time.monotonic()

    def _cmd_delta_cb(self, msg: Vector3):
        """Receive the commanded flex/roll delta from the previous CORR step."""
        with self._lock:
            self._last_cmd_flex = msg.x
            self._last_cmd_roll = msg.y

    def _achieved_delta_cb(self, msg: Vector3):
        """Receive achieved flex/roll delta and update achievement ratios.

        Updates an exponential moving average of achieved/commanded per axis.
        Only updates when the commanded delta is above a minimum threshold
        to avoid dividing by near-zero and to ignore HOLD steps.
        """
        with self._lock:
            if not self.get_parameter('use_achievement').value:
                return
            alpha   = self.get_parameter('achievement_alpha').value
            min_cmd = self.get_parameter('achievement_min_cmd').value

            ki_mode     = self.get_parameter('ki_mode').value
            windup_lim  = self.get_parameter('ki_windup_limit').value
            ki_flex     = self.get_parameter('ki_flex').value
            ki_roll     = self.get_parameter('ki_roll').value

            if self._last_cmd_flex is not None:
                cmd_f = abs(self._last_cmd_flex)
                act_f = abs(msg.x)
                if cmd_f >= min_cmd:
                    ratio_f = min(act_f / cmd_f, 2.0)
                    self._flex_achievement = ((1 - alpha) * self._flex_achievement +
                                              alpha * ratio_f)
                    self._flex_achievement = max(0.05, self._flex_achievement)
                    # servo ki — accumulate undelivered flex displacement
                    if ki_mode in ('servo', 'combined') and ki_flex > 0:
                        undelivered_f = self._last_cmd_flex - msg.x  # signed gap
                        self._integral_flex_servo += undelivered_f
                        self._integral_flex_servo = max(
                            -windup_lim / max(ki_flex, 1e-6),
                            min(windup_lim / max(ki_flex, 1e-6),
                                self._integral_flex_servo))

            if self._last_cmd_roll is not None:
                cmd_r = abs(self._last_cmd_roll)
                act_r = abs(msg.y)
                if cmd_r >= min_cmd:
                    ratio_r = min(act_r / cmd_r, 2.0)
                    self._roll_achievement = ((1 - alpha) * self._roll_achievement +
                                              alpha * ratio_r)
                    self._roll_achievement = max(0.05, self._roll_achievement)
                    # servo ki — accumulate undelivered roll displacement
                    if ki_mode in ('servo', 'combined') and ki_roll > 0:
                        undelivered_r = self._last_cmd_roll - msg.y
                        self._integral_roll_servo += undelivered_r
                        self._integral_roll_servo = max(
                            -windup_lim / max(ki_roll, 1e-6),
                            min(windup_lim / max(ki_roll, 1e-6),
                                self._integral_roll_servo))

    def _imu_raw_cb(self, msg: Imu):
        """Receive angular velocity from /imu/raw for true D-term.

        AXIS CONVENTION — follow this chain carefully, it has been a source of bugs:

        ESP32 sketch (imu_balance.ino):
          MPU-6050 physical axes on the wrist mount:
            gyro X (gx) → roll rate  → named 'dr' in JSON
            gyro Y (gy) → pitch rate → named 'dp' in JSON
          Published in JSON: {"dp": gy, "dr": gx, ...}

        imu_balance_node_v19.py parses JSON and fills sensor_msgs/Imu:
          angular_velocity.x = dr  (roll rate,  rad/s)   ← NOTE: x=ROLL
          angular_velocity.y = dp  (pitch rate, rad/s)   ← NOTE: y=PITCH
          This is OPPOSITE to the ROS REP-103 convention where x=forward(pitch).
          It was set this way to match the physical wrist mounting orientation
          where the IMU is mounted upside-down with axes rotated ~56°.
          Do not "fix" this to REP-103 without re-validating on hardware.

        ball_balance_node reads:
          imu_pitch_rate ← angular_velocity.y  (dp, pitch rate)
          imu_roll_rate  ← angular_velocity.x  (dr, roll rate)

        D-term application (control law):
          flex_cmd -= kd_flex * imu_pitch_rate   (MINUS = damping, opposes motion)
          roll_cmd -= kd_roll * imu_roll_rate

        Sign check: if the cup is already tilting toward center (pitch_rate same
        sign as the P-term flex_cmd), the D-term reduces the command — preventing
        overshoot.  If signs are wrong the D-term reinforces motion → oscillation.
        Validate sign with kd=0.02 and use_imu=True before increasing.

        NOTE: kd_flex and kd_roll default to 0.0 — D-term is OFF until explicitly
        enabled.  The axis swap bug (reading .x as pitch, .y as roll) was present
        in v17 and earlier.  Fixed here.
        """
        with self._lock:
            # Low-pass filter to reduce gyro noise before D-term use.
            # Raw gyro can spike ±0.5+ rad/s from vibration.
            a = self._gyro_alpha
            # .y = dp = pitch rate (wrist_flex axis) — see convention note above
            # .x = dr = roll rate  (wrist_roll axis)
            self._imu_pitch_rate = (a * msg.angular_velocity.y +
                                    (1 - a) * self._imu_pitch_rate)
            self._imu_roll_rate  = (a * msg.angular_velocity.x +
                                    (1 - a) * self._imu_roll_rate)
            self._imu_raw_time   = time.monotonic()

    def _wrist_vel_cb(self, msg: Vector3):
        """Joint-state finite difference velocity from wrist_balance_controller_v25+.
        x = d(wrist_flex_pos)/dt  (rad/s) — pitch axis, replaces imu_pitch_rate
        y = d(wrist_roll_pos)/dt  (rad/s) — roll axis,  replaces imu_roll_rate
        Published at ~100Hz. No quantization artifacts — resolution is the servo
        encoder resolution (~0.088°) divided by the 10ms sample interval.
        Sign convention: positive = wrist_flex increasing (cup tilting forward).
        Same sign as imu_pitch_rate when IMU is working correctly."""
        with self._lock:
            self._wrist_flex_vel = msg.x
            self._wrist_roll_vel = msg.y
            self._wrist_vel_time = time.monotonic()

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
                self._integral_flex_servo  = 0.0
                self._integral_roll_servo  = 0.0
                self._prev_error_x_sign    = None
                self._prev_error_y_sign    = None
                self._centered_since       = None
                self._flex_achievement     = 1.0   # reset — new pose may have different load
                self._roll_achievement     = 1.0
                self._last_cmd_flex        = None
                self._last_cmd_roll        = None
                self._acc_magnitudes       = []
                self._acc_valid            = 0
                self._acc_invalid          = 0
                self._acc_near             = 0
                self._ball_lost_neutralized = False
                self._last_jiggle_time      = None
                self._jiggle_phase          = 0
                self._acc_tight            = 0
                self._pub_enabled.publish(Bool(data=False))
            elif state == 'SETTLED':
                self._settled_at = time.monotonic()

    def _joint_state_cb(self, msg: JointState):
        """Joint states — kept for future FK setpoint computation."""
        pass

    # Summary loop

    def _summary_loop(self):
        """1Hz timer — logs PID_SUMMARY during active PID sessions.

        Accumulates every /ball/position reading (15Hz) into a one-line
        summary each second showing detection rate, near-center count,
        tight-center count, and min/mean distance.

        This gives accurate proximity stats that the ~1Hz throttled CMD
        log cannot capture.
        """
        with self._lock:
            if not self._pid_active:
                self._acc_magnitudes = []; self._acc_valid = 0
                self._acc_invalid = 0; self._acc_near = 0; self._acc_tight = 0
                return
            mags=list(self._acc_magnitudes); valid=self._acc_valid
            invalid=self._acc_invalid; near=self._acc_near
            tight=self._acc_tight; ball=self._ball_pos
            self._acc_magnitudes=[]; self._acc_valid=0
            self._acc_invalid=0; self._acc_near=0; self._acc_tight=0
        total=valid+invalid
        if total==0:
            self.get_logger().info('PID_SUMMARY | no ball data this second')
            return
        import math as _m
        st=self.get_parameter('stable_thresh').value
        nt=self.get_parameter('near_thresh').value
        min_mag =min(mags)          if mags else float('nan')
        mean_mag=sum(mags)/len(mags) if mags else float('nan')
        bx=ball.x if ball else 0.0; by=ball.y if ball else 0.0
        self.get_logger().info(
            f'PID_SUMMARY | '
            f'frames={valid}/{total}  '
            f'near({nt:.0%})={near}({100*near/valid if valid else 0:.0f}%)  '
            f'tight({st:.0%})={tight}({100*tight/valid if valid else 0:.0f}%)  '
            f'min={min_mag:.3f}  mean={mean_mag:.3f}  '
            f'ball=({bx:+.3f},{by:+.3f})')

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
            imu_pitch      = self._imu_pitch
            imu_roll       = self._imu_roll
            imu_time       = self._imu_time
            imu_pitch_rate = self._imu_pitch_rate
            imu_roll_rate  = self._imu_roll_rate
            imu_raw_time   = self._imu_raw_time
            wrist_flex_vel = self._wrist_flex_vel
            wrist_roll_vel = self._wrist_roll_vel
            wrist_vel_time = self._wrist_vel_time

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

        # Ball-lost check — ball undetected for too long.
        # After ball_lost_timeout: suspend normal PID and neutralize the last
        # PID command so the wrist controller stops applying it. Without this,
        # the wrist controller keeps executing the stale PID command for up to
        # cmd_timeout=2.0s, potentially moving the cup further from the ball.
        # After jiggle_start_delay: issue 4-phase circular jiggle commands to
        # rock the cup and make the ball visible to the detector.
        # Jiggle stops immediately when ball is redetected (z>=0 in _ball_cb).
        ball_lost_timeout  = self.get_parameter('ball_lost_timeout').value
        jiggle_amplitude   = self.get_parameter('jiggle_amplitude').value
        jiggle_start_delay = self.get_parameter('jiggle_start_delay').value
        if (self._last_valid_ball_time is not None and
                now - self._last_valid_ball_time > ball_lost_timeout):
            lost_duration = now - self._last_valid_ball_time
            self.get_logger().warn(
                f'Ball lost for {lost_duration:.1f}s '
                f'(>{ball_lost_timeout:.1f}s) — suspending PID.',
                throttle_duration_sec=1.0)

            # Publish neutral command once on ball-lost entry to cancel any
            # stale PID command the wrist controller is still executing.
            if not self._ball_lost_neutralized:
                neutral = Vector3()
                neutral.x = 0.0
                neutral.y = 0.0
                neutral.z = 0.0
                if not self.get_parameter('dry_run').value:
                    self._pub_cmd.publish(neutral)
                self._ball_lost_neutralized = True
                self.get_logger().info('Ball lost: published neutral cmd to cancel stale PID command')

            if lost_duration > jiggle_start_delay and jiggle_amplitude > 0.0:
                # Rate-limit jiggle to correction rate (default 1Hz).
                # Uses a 4-phase circular pattern so both axes are exercised:
                #   phase 0: flex+,  roll=0   (cup tips forward)
                #   phase 1: flex=0, roll+    (cup tilts right)
                #   phase 2: flex-,  roll=0   (cup tips back)
                #   phase 3: flex=0, roll-    (cup tilts left)
                # Any axis that is clamped by the wrist controller's hard limits
                # simply doesn't move — no need to duplicate URDF limits here.
                # All four directions are tried across 4 seconds, so the ball
                # is rocked in every direction and should become visible.
                jiggle_period = 1.0 / max(self.get_parameter('jiggle_hz').value, 0.1)
                if (self._last_jiggle_time is None or
                        now - self._last_jiggle_time >= jiggle_period):
                    self._jiggle_phase = (self._jiggle_phase + 1) % 4
                    self._last_jiggle_time = now

                    # 4-phase: (flex+, roll=0), (flex=0, roll+),
                    #          (flex-, roll=0), (flex=0, roll-)
                    _phases = [
                        ( jiggle_amplitude,  0.0),   # phase 0
                        ( 0.0,  jiggle_amplitude),   # phase 1
                        (-jiggle_amplitude,  0.0),   # phase 2
                        ( 0.0, -jiggle_amplitude),   # phase 3
                    ]
                    fx, ry = _phases[self._jiggle_phase]

                    jiggle_cmd = Vector3()
                    jiggle_cmd.x = fx
                    jiggle_cmd.y = ry
                    jiggle_cmd.z = 0.0
                    if not self.get_parameter('dry_run').value:
                        self._pub_cmd.publish(jiggle_cmd)
                    self.get_logger().info(
                        f'JIGGLE | lost={lost_duration:.1f}s '
                        f'flex={fx:+.3f} roll={ry:+.3f} '
                        f'(phase={self._jiggle_phase}/4)')
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

        # Latency monitoring — time since each message arrived
        cam_age_ms = (now - ball_time) * 1000
        imu_age_ms = (now - imu_time) * 1000 if imu_time else 0.0
        if cam_age_ms > 150:   # >2 frames at 15Hz
            self.get_logger().warn(
                f'CAM LAG: {cam_age_ms:.0f}ms since last ball position',
                throttle_duration_sec=1.0)
        if imu_age_ms > 60:    # >3 frames at 50Hz
            self.get_logger().warn(
                f'IMU LAG: {imu_age_ms:.0f}ms since last IMU reading',
                throttle_duration_sec=1.0)
        imu_raw_age_ms = (now - imu_raw_time) * 1000 if imu_raw_time else 9999.0
        if imu_raw_age_ms > 60:
            self.get_logger().warn(
                f'IMU RAW LAG: {imu_raw_age_ms:.0f}ms since last /imu/raw',
                throttle_duration_sec=1.0)

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

        # Centered hold — ball must stay within stable_thresh continuously
        # for centered_hold_time seconds before we declare success.
        # A single frame at center is not enough — the ball oscillates through.
        # /ball/is_centered is published True only on the hold expiry;
        # pose_test subscribes to this topic and advances immediately.
        stable_thresh     = self.get_parameter('stable_thresh').value
        centered_hold     = self.get_parameter('centered_hold_time').value
        is_stable         = magnitude < stable_thresh

        if is_stable:
            if self._centered_since is None:
                self._centered_since = now
            elif now - self._centered_since >= centered_hold:
                # Ball held at center for the full hold duration — declare success
                self.get_logger().info(
                    f'Ball centered — held within stable_thresh={stable_thresh:.2f} '
                    f'for {centered_hold:.1f}s  '
                    f'magnitude={magnitude:.3f}  '
                    f'ball=({error_x:+.3f},{error_y:+.3f})')
                self._pub_centered.publish(Bool(data=True))
                self._pid_active     = False
                self._pid_started_at = None
                self._settled_at     = None
                self._centered_since = None
                self._pub_enabled.publish(Bool(data=False))
                return
        else:
            # Ball left stable zone — reset hold timer
            self._centered_since = None

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

        # Anti-windup clamp — hard magnitude limit
        max_integral = max_cmd / max(ki_flex, 1e-6)
        self._integral_flex = max(-max_integral,
                                   min(max_integral, self._integral_flex))
        self._integral_roll = max(-max_integral,
                                   min(max_integral, self._integral_roll))

        # Anti-windup sign-change decay — when the ball crosses center on
        # either axis the accumulated I-term is now pushing the wrong way.
        # Decay it toward zero so the P-term can respond cleanly.
        # Applied to both ball-mode and servo-mode integrals independently.
        antiwindup_decay = self.get_parameter('antiwindup_decay').value
        curr_y_sign = 1 if error_y > 0 else (-1 if error_y < 0 else 0)
        curr_x_sign = 1 if error_x > 0 else (-1 if error_x < 0 else 0)

        if (antiwindup_decay < 1.0 and
                self._prev_error_y_sign is not None and
                curr_y_sign != 0 and self._prev_error_y_sign != 0 and
                curr_y_sign != self._prev_error_y_sign):
            old_i = self._integral_flex_servo
            self._integral_flex       *= antiwindup_decay
            self._integral_flex_servo *= antiwindup_decay
            self.get_logger().info(
                f'ANTI-WINDUP flex: error_y {self._prev_error_y_sign:+d}→{curr_y_sign:+d} '                f'i_flex {old_i:+.4f}→{self._integral_flex_servo:+.4f} '                f'(decay={antiwindup_decay})')

        if (antiwindup_decay < 1.0 and
                self._prev_error_x_sign is not None and
                curr_x_sign != 0 and self._prev_error_x_sign != 0 and
                curr_x_sign != self._prev_error_x_sign):
            old_i = self._integral_roll_servo
            self._integral_roll       *= antiwindup_decay
            self._integral_roll_servo *= antiwindup_decay
            self.get_logger().info(
                f'ANTI-WINDUP roll: error_x {self._prev_error_x_sign:+d}→{curr_x_sign:+d} '                f'i_roll {old_i:+.4f}→{self._integral_roll_servo:+.4f} '                f'(decay={antiwindup_decay})')

        self._prev_error_y_sign = curr_y_sign
        self._prev_error_x_sign = curr_x_sign

        # PID computation
        kp_flex = self.get_parameter('kp_flex').value
        kp_roll = self.get_parameter('kp_roll').value
        kd_flex = self.get_parameter('kd_flex').value
        kd_roll = self.get_parameter('kd_roll').value
        use_imu = self.get_parameter('use_imu').value

        # D-term source selection — controlled by d_term_source parameter.
        # 'imu'    — MPU-6050 gyro via /imu/raw. Known issue: quantizes at
        #            0.000133 rad/s/LSB, effectively frozen at slow angular
        #            velocities typical of 1Hz wrist corrections.
        # 'joints' — finite difference of joint_states via /balance/wrist_velocity
        #            published by wrist_balance_controller_v25+. No quantization;
        #            resolution ~0.0015 rad/s at 100Hz. Default.
        imu_timeout   = self.get_parameter('imu_timeout').value
        d_term_source = self.get_parameter('d_term_source').value

        if d_term_source == 'joints':
            joints_fresh = (wrist_vel_time is not None and
                            now - wrist_vel_time < imu_timeout)
            d_flex = wrist_flex_vel if joints_fresh else 0.0
            d_roll = wrist_roll_vel if joints_fresh else 0.0
            if not joints_fresh and use_imu:
                self.get_logger().warn(
                    'wrist_velocity stale — D-term zeroed. '
                    'Is wrist_balance_controller_v25+ running?',
                    throttle_duration_sec=5.0)
        else:  # 'imu'
            imu_raw_fresh = (use_imu and
                             imu_raw_time is not None and
                             now - imu_raw_time < imu_timeout)
            d_flex = imu_pitch_rate if imu_raw_fresh else 0.0
            d_roll = imu_roll_rate  if imu_raw_fresh else 0.0

        # Control law (validated signs from balance_v1.yaml):
        #   ball right (+x) → roll left  → +wrist_roll → roll_cmd positive
        #   ball near  (+y) → flex back → +wrist_flex → flex_cmd positive
        # Select integral source based on ki_mode
        ki_mode = self.get_parameter('ki_mode').value
        if ki_mode == 'servo':
            i_flex = self._integral_flex_servo
            i_roll = self._integral_roll_servo
        else:  # 'ball', 'combined', 'none'
            i_flex = self._integral_flex
            i_roll = self._integral_roll

        flex_cmd = (+kp_flex * error_y
                    + ki_flex * i_flex
                    - kd_flex * d_flex)   # MINUS: D-term opposes motion (damping)
        roll_cmd = (+kp_roll * error_x
                    + ki_roll * i_roll
                    - kd_roll * d_roll)   # MINUS: D-term opposes motion (damping)

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

        # Publish individual PID terms for pid_logger.
        # x=P_term, y=I_term, z=D_term — sum equals pre-clamp flex/roll cmd.
        pid_flex = Vector3()
        pid_flex.x = +kp_flex * error_y          # P
        pid_flex.y = +ki_flex * i_flex            # I
        pid_flex.z = -kd_flex * d_flex            # D (negative = damping)
        self._pub_pid_flex.publish(pid_flex)

        pid_roll = Vector3()
        pid_roll.x = +kp_roll * error_x          # P
        pid_roll.y = +ki_roll * i_roll            # I
        pid_roll.z = -kd_roll * d_roll            # D
        self._pub_pid_roll.publish(pid_roll)
        self.get_logger().info(
            f'CMD | ball=({error_x:+.3f},{error_y:+.3f}) '
            f'Pflex={+kp_flex*error_y:+.4f} Proll={+kp_roll*error_x:+.4f} '
            f'Dflex={-kd_flex*d_flex:+.4f} Droll={-kd_roll*d_roll:+.4f} '
            f'→ flex={flex_cmd:+.4f} roll={roll_cmd:+.4f} '
            f'd_src={d_term_source} flex_rate={d_flex:+.4f}rad/s roll_rate={d_roll:+.4f}rad/s '
            f'ach_flex={self._flex_achievement:.2f} ach_roll={self._roll_achievement:.2f} '
            f'i_flex={i_flex:+.4f} i_roll={i_roll:+.4f} ki_mode={self.get_parameter("ki_mode").value} '
            f'stable={is_stable} '
            f'hold={f"{now-self._centered_since:.1f}s" if self._centered_since else "--"}',
            throttle_duration_sec=1.0)


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
