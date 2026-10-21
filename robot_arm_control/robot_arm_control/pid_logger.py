#!/usr/bin/env python3
"""
pid_logger_v1.py — PID state logger for ball balancing system

Subscribes to all relevant topics and logs one CSV row per
/joint_states message (100Hz), merging the latest values from
all other topics at that moment. This gives full resolution on
joint position and IMU data. Ball position updates at its native
~12Hz (OAK) and is held between camera frames.

This lets you study:
  - How IMU pitch/roll rate changes as servo arms move
  - How ball position tracks servo corrections
  - PID state evolution over time
  - Correlation between imu_pitch_rate and subsequent ball.y changes

CSV columns (one row per /ball/position message):
  timestamp              float   wall-clock seconds (time.time())
  ros_time               float   ROS time of /ball/position message (s)

  -- Ball position --
  ball_x                 float   normalized x offset from cup center (-1..+1)
  ball_y                 float   normalized y offset from cup center (-1..+1)
  ball_mag               float   sqrt(x²+y²) — distance from center
  ball_detected          int     1 if valid detection (z>=0), 0 if lost

  -- PID state (from ball_balance_node CMD log topic) --
  arm_state              str     MOVING / SETTLED / unknown
  pose_active            int     1 between PoseEvent start and end
  pose_number            int     current pose number
  Pflex                  float   P-term flex command
  Proll                  float   P-term roll command
  Iflex                  float   I-term flex accumulator
  Iroll                  float   I-term roll accumulator
  Dflex                  float   D-term flex contribution
  Droll                  float   D-term roll contribution
  flex_cmd               float   total flex command sent to wrist controller
  roll_cmd               float   total roll command sent to wrist controller
  ach_flex               float   flex achievement EMA (1.0=perfect delivery)
  ach_roll               float   roll achievement EMA

  -- Balance command (what wrist controller received) --
  balance_cmd_flex       float   /imu/balance_cmd.x (rad/s)
  balance_cmd_roll       float   /imu/balance_cmd.y (rad/s)

  -- Wrist controller delivery --
  cmd_delta_flex         float   /balance/cmd_delta.x (rad commanded this step)
  cmd_delta_roll         float   /balance/cmd_delta.y
  achieved_delta_flex    float   /balance/achieved_delta.x (rad actually moved)
  achieved_delta_roll    float   /balance/achieved_delta.y

  -- IMU tilt angles (from /imu/balance_error) --
  imu_pitch_err          float   pitch error angle (rad) — cup tilt forward/back
  imu_roll_err           float   roll error angle (rad)  — cup tilt left/right

  -- IMU angular velocity (from /imu/raw) --
  imu_pitch_rate         float   angular_velocity.y (rad/s) — dp, pitch rate
  imu_roll_rate          float   angular_velocity.x (rad/s) — dr, roll rate
  imu_ax                 float   linear_acceleration.x (m/s²)
  imu_ay                 float   linear_acceleration.y (m/s²)
  imu_az                 float   linear_acceleration.z (m/s²)

  -- Joint positions (from /joint_states) --
  wrist_flex_pos         float   wrist_flex joint position (rad)
  wrist_roll_pos         float   wrist_roll joint position (rad)
  shoulder_pan_pos       float   shoulder_pan joint position (rad)
  shoulder_lift_pos      float   shoulder_lift joint position (rad)
  elbow_flex_pos         float   elbow_flex joint position (rad)

  -- Finite difference angular velocity (computed at 100Hz) --
  dwrist_flex_dt         float   d(wrist_flex_pos)/dt (rad/s) — gyro-independent
  dwrist_roll_dt         float   d(wrist_roll_pos)/dt (rad/s) — gyro-independent
                                 Compare to imu_pitch_rate / imu_roll_rate.
                                 Should match if IMU is accurate.
                                 0.0 between 1Hz corrections (servo holding).

  -- Correction step tracking --
  corr_step              int     monotonically increasing correction counter.
                                 Increments each time cmd_delta changes (1Hz).
                                 Use to group rows by correction step for analysis.
  corr_new               int     1 on the first row of each new correction, 0 otherwise.
                                 Use to mark correction boundaries on time-series plots.

  -- Current pose (from pose_events topic) --
  pose_name              str     current pose name
  pose_number            int     current pose number

Parameters:
  output_dir    Directory for CSV output    default: '.'
  flush_hz      How often to flush to disk  default: 1.0

Usage:
  ros2 run balance pid_logger
  ros2 run balance pid_logger --ros-args -p output_dir:=/home/ubuntu/logs
"""

import csv
import math
import os
import time
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, String
from koch_v1_1_msgs.msg import PoseEvent


# Regex to parse CMD log lines published on /ball/pid_state
# (we subscribe to raw topics and reconstruct state ourselves)

class PIDLoggerNode(Node):

    def __init__(self):
        super().__init__('pid_logger')

        self.declare_parameter('output_dir', '.')
        self.declare_parameter('flush_hz',   1.0)

        out_dir  = self.get_parameter('output_dir').value
        flush_hz = self.get_parameter('flush_hz').value

        ts_str   = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(out_dir, f'pid_log_{ts_str}.csv')

        self._csvfile = open(filename, 'w', newline='')
        self._writer  = csv.writer(self._csvfile)
        self._lock    = threading.Lock()

        # Write header
        self._writer.writerow([
            'timestamp', 'ros_time',
            # Ball
            'ball_x', 'ball_y', 'ball_mag', 'ball_detected',
            # Arm / PID state
            'arm_state', 'pose_active', 'pose_name', 'pose_number',
            'Pflex', 'Proll', 'Iflex', 'Iroll', 'Dflex', 'Droll',
            'flex_cmd', 'roll_cmd', 'ach_flex', 'ach_roll',
            # Balance cmd
            'balance_cmd_flex', 'balance_cmd_roll',
            # Wrist controller delivery
            'cmd_delta_flex', 'cmd_delta_roll',
            'achieved_delta_flex', 'achieved_delta_roll',
            # IMU tilt angles
            'imu_pitch_err', 'imu_roll_err',
            # IMU angular velocity
            'imu_pitch_rate', 'imu_roll_rate',
            'imu_ax', 'imu_ay', 'imu_az',
            # Joint positions
            'wrist_flex_pos', 'wrist_roll_pos',
            'shoulder_pan_pos', 'shoulder_lift_pos', 'elbow_flex_pos',
            # Finite difference angular velocity — compare to imu_pitch/roll_rate
            'dwrist_flex_dt', 'dwrist_roll_dt',
            # Correction step tracking
            'corr_step', 'corr_new',
        ])

        # ── Latest values from each topic ────────────────────────────────
        self._arm_state         = 'unknown'
        self._pid_active        = 0
        self._pose_name         = ''
        self._pose_number       = 0
        self._pose_active       = False   # True between PoseEvent start and end

        # Ball position — held state, updated by _ball_cb at ~12Hz
        self._ball_x            = 0.0
        self._ball_y            = 0.0
        self._ball_mag          = float('nan')
        self._ball_detected     = 0

        # PID terms — from /ball/pid_detail_flex and /ball/pid_detail_roll
        # x=P, y=I, z=D per message
        self._Pflex = self._Iflex = self._Dflex = 0.0
        self._Proll = self._Iroll = self._Droll = 0.0
        self._flex_cmd = self._roll_cmd = 0.0
        self._ach_flex = self._ach_roll = 1.0

        self._balance_cmd_flex = self._balance_cmd_roll = 0.0
        self._cmd_delta_flex   = self._cmd_delta_roll   = 0.0
        self._ach_delta_flex   = self._ach_delta_roll   = 0.0

        self._imu_pitch_err  = self._imu_roll_err  = 0.0
        self._imu_pitch_rate = self._imu_roll_rate  = 0.0
        self._imu_ax = self._imu_ay = self._imu_az  = 0.0

        self._joint_positions = {}   # name → position (rad)

        # Finite difference derivatives — computed in _joint_state_cb at 100Hz.
        # Used as a gyro-independent measure of wrist angular velocity.
        # More reliable than imu_pitch_rate for slow servo motion due to
        # MPU-6050 quantization at ±250°/s range (0.000133 rad/s per LSB).
        self._prev_wrist_flex_pos  = None   # previous wrist_flex_pos (rad)
        self._prev_wrist_roll_pos  = None   # previous wrist_roll_pos (rad)
        self._prev_joint_time      = None   # wall-clock time of previous sample
        self._dwrist_flex_dt       = 0.0    # d(wrist_flex_pos)/dt  (rad/s)
        self._dwrist_roll_dt       = 0.0    # d(wrist_roll_pos)/dt  (rad/s)

        # Correction step counter — increments each time cmd_delta changes.
        # Identifies the start of each 1Hz wrist controller correction.
        # corr_new = 1 on the first row of a new correction, 0 otherwise.
        self._corr_step            = 0
        self._corr_new             = 0      # 1 for one row at correction start
        self._prev_cmd_delta_flex  = None
        self._prev_cmd_delta_roll  = None

        # ── Subscriptions ────────────────────────────────────────────────

        # /ball/position triggers each CSV row (12Hz)
        self.create_subscription(
            Point, '/ball/position',
            self._ball_cb, 10)

        # /arm_state — MOVING / SETTLED
        self.create_subscription(
            String, '/arm_state',
            self._arm_state_cb, 10)

        # /imu/balance_cmd — what ball_balance_node sent to wrist controller
        # x=flex_cmd (rad/s), y=roll_cmd (rad/s)
        self.create_subscription(
            Vector3, '/imu/balance_cmd',
            self._balance_cmd_cb, 10)

        # /balance/cmd_delta — what wrist controller commanded to servo (rad)
        self.create_subscription(
            Vector3, '/balance/cmd_delta',
            self._cmd_delta_cb, 10)

        # /balance/achieved_delta — what servo actually moved (rad)
        self.create_subscription(
            Vector3, '/balance/achieved_delta',
            self._ach_delta_cb, 10)

        # /imu/balance_error — cup tilt angles (pitch_err, roll_err) rad
        self.create_subscription(
            Vector3, '/imu/balance_error',
            self._imu_err_cb, 10)

        # /imu/raw — angular velocity + linear acceleration at 50Hz
        # angular_velocity.x = dr (roll rate), .y = dp (pitch rate)
        # See ball_balance_node_v18._imu_raw_cb for full convention note
        self.create_subscription(
            Imu, '/imu/raw',
            self._imu_raw_cb, 10)

        # /joint_states — all joint positions at 100Hz
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_cb, 10)

        # /pose_events — PoseEvent start/end from pose_test
        # Provides pose name, number, and active state — same as power_logger.
        self.create_subscription(
            PoseEvent, 'pose_events',
            self._pose_event_cb, 10)

        # /balance_enabled — True when wrist controller should act
        # Kept for ach_flex/ach_roll context but not used for pose_active
        self.create_subscription(
            Bool, '/balance_enabled',
            self._balance_enabled_cb, 10)

        # /ball/pid_detail_flex — (P_flex, I_flex, D_flex) per PID tick
        # /ball/pid_detail_roll — (P_roll, I_roll, D_roll) per PID tick
        # Published by ball_balance_node_v19+. x=P, y=I, z=D.
        self.create_subscription(
            Vector3, '/ball/pid_detail_flex',
            self._pid_detail_flex_cb, 10)
        self.create_subscription(
            Vector3, '/ball/pid_detail_roll',
            self._pid_detail_roll_cb, 10)

        # Periodic flush timer
        self.create_timer(1.0 / flush_hz, self._flush)

        self.get_logger().info(f'pid_logger started — writing to {filename}')
        self.get_logger().info(
            'Logging at /joint_states rate (100Hz) — ball/IMU held at their native rates')

    # ── Callbacks ────────────────────────────────────────────────────────

    def _ball_cb(self, msg: Point):
        """Store latest ball position — CSV row is written by _joint_state_cb."""
        with self._lock:
            self._ball_x        = msg.x
            self._ball_y        = msg.y
            self._ball_detected = 1 if msg.z >= 0 else 0
            self._ball_mag      = (math.sqrt(msg.x**2 + msg.y**2)
                                   if self._ball_detected else float('nan'))

    def _pose_event_cb(self, msg: PoseEvent):
        """PoseEvent start/end from pose_test — drives pose_name and pose_active."""
        with self._lock:
            if msg.event_type == 'start':
                self._pose_name   = msg.pose_name
                self._pose_number = msg.pose_number
                self._pose_active = True
                self.get_logger().info(
                    f'Pose started: {msg.pose_name} (#{msg.pose_number})')
            elif msg.event_type == 'end':
                self._pose_active = False
                self.get_logger().info(
                    f'Pose ended: {msg.pose_name}')

    def _arm_state_cb(self, msg: String):
        with self._lock:
            self._arm_state = msg.data

    def _balance_enabled_cb(self, msg: Bool):
        """Track whether wrist controller is active — for context only."""
        with self._lock:
            self._pid_active = 1 if msg.data else 0

    def _balance_cmd_cb(self, msg: Vector3):
        """x=flex_cmd (rad/s), y=roll_cmd (rad/s) — the full PID sum."""
        with self._lock:
            self._balance_cmd_flex = msg.x
            self._balance_cmd_roll = msg.y
            # /imu/balance_cmd IS the total flex_cmd/roll_cmd after P+I+D
            # Store as flex_cmd/roll_cmd for the CSV
            self._flex_cmd = msg.x
            self._roll_cmd = msg.y

    def _cmd_delta_cb(self, msg: Vector3):
        """x=commanded flex delta (rad), y=commanded roll delta (rad).
        Increments corr_step and sets corr_new=1 when either axis changes —
        marks the start of a new 1Hz wrist controller correction."""
        with self._lock:
            changed = (self._prev_cmd_delta_flex is None or
                       abs(msg.x - self._prev_cmd_delta_flex) > 1e-6 or
                       abs(msg.y - (self._prev_cmd_delta_roll or 0.0)) > 1e-6)
            if changed:
                self._corr_step += 1
                self._corr_new   = 1
            self._cmd_delta_flex      = msg.x
            self._cmd_delta_roll      = msg.y
            self._prev_cmd_delta_flex = msg.x
            self._prev_cmd_delta_roll = msg.y

    def _ach_delta_cb(self, msg: Vector3):
        """x=achieved flex delta (rad), y=achieved roll delta (rad)."""
        with self._lock:
            self._ach_delta_flex = msg.x
            self._ach_delta_roll = msg.y

    def _imu_err_cb(self, msg: Vector3):
        """x=pitch_err (rad), y=roll_err (rad) — cup tilt angles."""
        with self._lock:
            self._imu_pitch_err = msg.x
            self._imu_roll_err  = msg.y

    def _imu_raw_cb(self, msg: Imu):
        """angular_velocity: x=dr (roll rate), y=dp (pitch rate) — rad/s.
        NOTE: x=roll, y=pitch — opposite to ROS REP-103. See ball_balance_node_v18."""
        with self._lock:
            self._imu_pitch_rate = msg.angular_velocity.y   # dp
            self._imu_roll_rate  = msg.angular_velocity.x   # dr
            self._imu_ax         = msg.linear_acceleration.x
            self._imu_ay         = msg.linear_acceleration.y
            self._imu_az         = msg.linear_acceleration.z

    def _joint_state_cb(self, msg: JointState):
        """Triggered at 100Hz — write one CSV row with all latest state."""
        now     = time.time()
        ros_now = self.get_clock().now().nanoseconds / 1e9

        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

            # Finite difference angular velocity — d(pos)/dt at 100Hz.
            # Provides gyro-independent wrist angular velocity for D-term
            # analysis and IMU quality comparison.
            # Guard: skip first sample (no previous), clamp dt to avoid
            # divide-by-zero or stale-timestamp artifacts.
            wf = self._joint_positions.get('wrist_flex', None)
            wr = self._joint_positions.get('wrist_roll', None)
            if (self._prev_joint_time is not None and
                    self._prev_wrist_flex_pos is not None and wf is not None):
                dt_j = now - self._prev_joint_time
                if 0.001 < dt_j < 0.5:   # 1ms–500ms guard
                    self._dwrist_flex_dt = (wf - self._prev_wrist_flex_pos) / dt_j
                    self._dwrist_roll_dt = ((wr - self._prev_wrist_roll_pos) / dt_j
                                            if wr is not None
                                            and self._prev_wrist_roll_pos is not None
                                            else 0.0)
            self._prev_wrist_flex_pos = wf
            self._prev_wrist_roll_pos = wr
            self._prev_joint_time     = now

            self._writer.writerow([
                f'{now:.6f}',
                f'{ros_now:.6f}',
                # Ball (held from last /ball/position — updates at ~12Hz)
                f'{self._ball_x:.6f}',
                f'{self._ball_y:.6f}',
                f'{self._ball_mag:.6f}' if self._ball_detected else '',
                self._ball_detected,
                # PID / pose state
                self._arm_state,
                1 if self._pose_active else 0,
                self._pose_name,
                self._pose_number,
                f'{self._Pflex:.6f}',
                f'{self._Proll:.6f}',
                f'{self._Iflex:.6f}',
                f'{self._Iroll:.6f}',
                f'{self._Dflex:.6f}',
                f'{self._Droll:.6f}',
                f'{self._flex_cmd:.6f}',
                f'{self._roll_cmd:.6f}',
                f'{self._ach_flex:.6f}',
                f'{self._ach_roll:.6f}',
                # Balance cmd
                f'{self._balance_cmd_flex:.6f}',
                f'{self._balance_cmd_roll:.6f}',
                # Wrist delivery
                f'{self._cmd_delta_flex:.6f}',
                f'{self._cmd_delta_roll:.6f}',
                f'{self._ach_delta_flex:.6f}',
                f'{self._ach_delta_roll:.6f}',
                # IMU tilt angles (held from /imu/balance_error — 50Hz)
                f'{self._imu_pitch_err:.6f}',
                f'{self._imu_roll_err:.6f}',
                # IMU angular velocity (held from /imu/raw — 50Hz)
                f'{self._imu_pitch_rate:.6f}',
                f'{self._imu_roll_rate:.6f}',
                f'{self._imu_ax:.6f}',
                f'{self._imu_ay:.6f}',
                f'{self._imu_az:.6f}',
                # Joint positions (fresh from this message — 100Hz)
                f'{self._joint_positions.get("wrist_flex",  float("nan")):.6f}',
                f'{self._joint_positions.get("wrist_roll",  float("nan")):.6f}',
                f'{self._joint_positions.get("shoulder_pan",  float("nan")):.6f}',
                f'{self._joint_positions.get("shoulder_lift", float("nan")):.6f}',
                f'{self._joint_positions.get("elbow_flex",  float("nan")):.6f}',
                # Finite difference derivatives
                f'{self._dwrist_flex_dt:.6f}',
                f'{self._dwrist_roll_dt:.6f}',
                # Correction step counter
                self._corr_step,
                self._corr_new,
            ])
            # corr_new is a one-row flag — clear after writing
            self._corr_new = 0

    def _pid_detail_flex_cb(self, msg: Vector3):
        """x=P_flex, y=I_flex, z=D_flex — individual PID terms for flex axis."""
        with self._lock:
            self._Pflex = msg.x
            self._Iflex = msg.y
            self._Dflex = msg.z

    def _pid_detail_roll_cb(self, msg: Vector3):
        """x=P_roll, y=I_roll, z=D_roll — individual PID terms for roll axis."""
        with self._lock:
            self._Proll = msg.x
            self._Iroll = msg.y
            self._Droll = msg.z

    def _flush(self):
        with self._lock:
            self._csvfile.flush()

    def destroy_node(self):
        with self._lock:
            self._csvfile.flush()
            self._csvfile.close()
        self.get_logger().info('pid_logger: CSV file closed.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PIDLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
