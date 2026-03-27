#!/usr/bin/env python3
"""
imu_balance_node.py — ROS2 node for ESP32/MPU-6050 cup-balancing IMU

Reads the JSON stream from the ESP32 over serial, publishes:
  /imu/raw           sensor_msgs/Imu          — fused pitch/roll + rates
  /imu/balance_error geometry_msgs/Vector3    — (pitch_err, roll_err, 0)
  /imu/balance_cmd   geometry_msgs/Vector3    — (wrist_flex_delta, wrist_roll_delta, 0)
                                                scaled PID output, ready to feed
                                                into wrist_balance_controller

Architecture (v2):
  - Subscribes to /arm_state (std_msgs/String) published by pose_test.
    Values: "MOVING" or "SETTLED".
  - In MOVING state: PID is suspended, integrals are reset.  The wrist_balance_controller
    is simultaneously disabled via /balance_enabled (std_msgs/Bool).
  - In SETTLED state: FK-derived wrist setpoint is computed from /joint_states and used
    as the PID reference instead of a fixed zero.  This handles the chicken-and-egg
    problem: the arm may settle at a non-zero wrist angle that is already level — the
    IMU then only corrects *deviations* from that kinematically-level pose.
  - A unified 2D error vector (tilt magnitude + direction) drives a single correction
    vector that is then projected onto the wrist_flex and wrist_roll joint axes,
    avoiding independent-axis fight.

FK setpoint:
  Given shoulder_lift (θ1), elbow_flex (θ2), and wrist_flex (θ3) joint angles the
  wrist_flex setpoint that keeps the cup mouth horizontal is:
      wrist_flex_setpoint = π - (θ1 + θ2)   [simplified planar model]
  The wrist_roll setpoint is always 0 (level about the roll axis).
  This is a planar approximation — sufficient for cup balancing.

Usage:
  ros2 run <your_package> imu_balance_node \
    --ros-args -p port:=/dev/ttyIMU -p baud:=115200

Parameters:
  port          Serial port of the ESP32           default: /dev/ttyIMU
  baud          Baud rate                          default: 115200
  frame_id      IMU frame name in TF               default: imu_link
  pitch_kp      Proportional gain, pitch axis       default: 1.5
  pitch_ki      Integral gain, pitch axis           default: 0.05
  pitch_kd      Derivative gain, pitch axis         default: 0.3
  roll_kp       Proportional gain, roll axis        default: 1.5
  roll_ki       Integral gain, roll axis            default: 0.05
  roll_kd       Derivative gain, roll axis          default: 0.3
  deadband_rad  Error below this is treated as 0   default: 0.005 (~0.3°)
  max_cmd       Maximum correction command (rad/s)  default: 0.5
  publish_hz    Rate to publish (Hz)                default: 50
  invert_pitch  Flip pitch correction sign          default: False
  invert_roll   Flip roll correction sign           default: False
  settle_delay  Seconds after SETTLED before PID activates  default: 0.3

Axis convention:
  pitch → wrist_flex joint correction
  roll  → wrist_roll joint correction

If the correction direction is backwards, use invert_pitch/invert_roll params
rather than editing code — makes it easy to flip at launch time.

Dependencies:
  pip install pyserial --break-system-packages
  ros2 packages: rclpy, sensor_msgs, geometry_msgs, std_msgs
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool, String

import serial
import json
import math
import time
import threading
from collections import deque


class PIDController:
    """Simple PID with anti-windup and deadband."""

    def __init__(self, kp: float, ki: float, kd: float,
                 deadband: float = 0.0, max_output: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.deadband = deadband
        self.max_output = max_output

        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    def reset(self):
        self._integral = 0.0
        self._last_error = 0.0
        self._last_time = None

    def update(self, error: float, now: float) -> float:
        """
        Compute PID correction for given error at timestamp `now` (seconds).
        Returns correction in the same units as error (radians → rad/s command).
        """
        if abs(error) < self.deadband:
            error = 0.0

        if self._last_time is None:
            dt = 0.0
        else:
            dt = now - self._last_time
            dt = max(dt, 1e-4)  # guard divide-by-zero

        self._last_time = now

        # Proportional
        p = self.kp * error

        # Integral with anti-windup clamping
        self._integral += error * dt
        i = self.ki * self._integral

        # Derivative (on error, not measurement — avoids derivative kick on setpoint change)
        d = 0.0
        if dt > 0:
            d = self.kd * (error - self._last_error) / dt

        self._last_error = error

        output = p + i + d

        # Clamp and anti-windup: if clamped, unwind integral
        if abs(output) > self.max_output:
            output = math.copysign(self.max_output, output)
            self._integral -= error * dt   # undo this step's integral contribution

        return output

    def update_gains(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.reset()


class ImuBalanceNode(Node):

    def __init__(self):
        super().__init__('imu_balance_node')

        # Parameters 
        self.declare_parameter('port',          '/dev/ttyIMU')
        self.declare_parameter('baud',          115200)
        self.declare_parameter('frame_id',      'imu_link')
        self.declare_parameter('pitch_kp',      1.5)
        self.declare_parameter('pitch_ki',      0.05)
        self.declare_parameter('pitch_kd',      0.3)
        self.declare_parameter('roll_kp',       1.5)
        self.declare_parameter('roll_ki',       0.05)
        self.declare_parameter('roll_kd',       0.3)
        self.declare_parameter('deadband_rad',  0.005)
        self.declare_parameter('max_cmd',       0.5)
        self.declare_parameter('publish_hz',    50)
        self.declare_parameter('invert_pitch',  False)
        self.declare_parameter('invert_roll',   False)
        self.declare_parameter('settle_delay',  0.3)   # seconds after SETTLED before PID activates
        self.declare_parameter('debug_mode',    False)  # publish /imu/balance_diagnosis

        self._port       = self.get_parameter('port').value
        self._baud       = self.get_parameter('baud').value
        self._frame_id   = self.get_parameter('frame_id').value
        self._pub_hz     = self.get_parameter('publish_hz').value
        self._inv_pitch  = self.get_parameter('invert_pitch').value
        self._inv_roll   = self.get_parameter('invert_roll').value
        deadband         = self.get_parameter('deadband_rad').value
        max_cmd          = self.get_parameter('max_cmd').value

        # PID controllers 
        self._pid_pitch = PIDController(
            kp=self.get_parameter('pitch_kp').value,
            ki=self.get_parameter('pitch_ki').value,
            kd=self.get_parameter('pitch_kd').value,
            deadband=deadband,
            max_output=max_cmd
        )
        self._pid_roll = PIDController(
            kp=self.get_parameter('roll_kp').value,
            ki=self.get_parameter('roll_ki').value,
            kd=self.get_parameter('roll_kd').value,
            deadband=deadband,
            max_output=max_cmd
        )

        # Publishers 
        self._pub_imu = self.create_publisher(Imu,     '/imu/raw',           10)
        self._pub_err = self.create_publisher(Vector3, '/imu/balance_error', 10)
        self._pub_cmd = self.create_publisher(Vector3, '/imu/balance_cmd',   10)
        # Publish True while balance error is within deadband
        self._pub_stable = self.create_publisher(Bool, '/imu/is_stable',     10)
        # Tell wrist_balance_controller whether to apply corrections
        self._pub_balance_enabled = self.create_publisher(Bool, '/balance_enabled', 10)
        # Human-readable debug diagnosis — always created, only published when debug_mode=True
        self._pub_diagnosis = self.create_publisher(String, '/imu/balance_diagnosis', 10)

        # State 
        self._latest = None          # most recent parsed JSON dict
        self._lock   = threading.Lock()
        self._serial = None
        self._running = True

        # Arm state machine: "MOVING" or "SETTLED"
        self._arm_state = 'MOVING'
        self._settled_at = None      # monotonic timestamp when SETTLED arrived
        self._pid_active = False     # True only after settle_delay has elapsed

        # FK joint angles (updated from /joint_states)
        self._joint_positions = {}   # name → float (rad)
        self._joints_lock = threading.Lock()

        # Rolling window for stability detection (last N samples)
        self._err_window_pitch = deque(maxlen=20)
        self._err_window_roll  = deque(maxlen=20)

        # Subscribers 
        self.create_subscription(
            String, '/arm_state',
            self._arm_state_callback, 10)

        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10)

        # Serial reader thread 
        self._serial_thread = threading.Thread(
            target=self._serial_reader, daemon=True
        )
        self._serial_thread.start()

        # Publish timer 
        timer_period = 1.0 / self._pub_hz
        self._timer = self.create_timer(timer_period, self._publish_callback)

        self.get_logger().info(
            f'IMU balance node started — {self._port} @ {self._baud} baud, '
            f'{self._pub_hz}Hz publish rate'
        )
        self.get_logger().info(
            'Topics: /imu/raw  /imu/balance_error  /imu/balance_cmd  /imu/is_stable'
        )
        self.get_logger().info(
            'Tip: recalibrate IMU by sending "c" to ESP32 serial when arm is level.'
        )
        if self.get_parameter('debug_mode').value:
            self.get_logger().info(
                'DEBUG MODE ON — publishing diagnosis to /imu/balance_diagnosis\n'
                '  Monitor with: ros2 topic echo /imu/balance_diagnosis'
            )

    # 
    def _arm_state_callback(self, msg: String):
        """
        Receive arm state from pose_test ('MOVING' or 'SETTLED').
        On MOVING: suspend PID and reset integrals.
        On SETTLED: start settle_delay timer; PID activates after it expires.
        """
        new_state = msg.data.upper()
        if new_state == self._arm_state:
            return

        self._arm_state = new_state
        if new_state == 'MOVING':
            self._pid_active = False
            self._settled_at = None
            self._pid_pitch.reset()
            self._pid_roll.reset()
            self.get_logger().info('Arm MOVING — PID suspended, integrals reset.')
            # Tell wrist_balance_controller to stand down
            self._pub_balance_enabled.publish(Bool(data=False))
        elif new_state == 'SETTLED':
            self._settled_at = time.monotonic()
            self.get_logger().info(
                f'Arm SETTLED — PID activates in '
                f'{self.get_parameter("settle_delay").value:.2f}s.'
            )

    def _joint_state_callback(self, msg: JointState):
        """Cache latest joint positions for FK setpoint computation."""
        with self._joints_lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def _compute_fk_setpoint(self):
        """
        Compute the wrist_flex angle that keeps the cup mouth horizontal,
        given the current shoulder_lift and elbow_flex angles.

        Planar model (gravity in the plane of shoulder_lift + elbow_flex):
            wrist_flex_setpoint = π - (shoulder_lift + elbow_flex)

        Returns (flex_setpoint, roll_setpoint) in radians.
        Returns (None, None) if joint data is unavailable.
        """
        with self._joints_lock:
            sl = self._joint_positions.get('shoulder_lift')
            ef = self._joint_positions.get('elbow_flex')

        if sl is None or ef is None:
            return None, None

        flex_setpoint = math.pi - (sl + ef)
        roll_setpoint = 0.0
        return flex_setpoint, roll_setpoint

    # 
    def _serial_reader(self):
        """Background thread: open serial port and read JSON lines."""
        while self._running:
            try:
                self.get_logger().info(f'Opening serial port {self._port}...')
                self._serial = serial.Serial(self._port, self._baud, timeout=1.0)
                self.get_logger().info('Serial port open.')

                hash_cnt = 0
                hash_cnt_limit = 1000

                while self._running:
                    line = self._serial.readline().decode('utf-8', errors='ignore').strip()

                    if not line or line.startswith('#'):
                       if line.startswith('#'):
                             if line.count("fail") > 0 or line.count("Fail") > 0 or line.count("ERROR") > 0:
                                                         self.get_logger().info("error: " + line)
                             if hash_cnt == 0:
                                self.get_logger().info('hash lines: ' + line)
                             hash_cnt += 1
 
                             if hash_cnt == hash_cnt_limit:
                                self.get_logger().info('reset hash cnt at 1000 : ' + line)
                                hash_cnt = 0

                       continue  # skip comments/empty lines from ESP32

                    try:
                        data = json.loads(line)
                        with self._lock:
                            self._latest = data
                    except json.JSONDecodeError:
                        self.get_logger().debug(f'Unparseable line: {line[:60]}')

            except serial.SerialException as e:
                self.get_logger().warn(f'Serial error: {e} — retrying in 2s...')
                time.sleep(2.0)
            except Exception as e:
                self.get_logger().error(f'Reader thread error: {e}')
                time.sleep(1.0)

    # 
    def _publish_callback(self):
        """Timer callback: publish IMU, balance error, and PID command."""
        with self._lock:
            data = self._latest

        if data is None:
            return

        now_ros = self.get_clock().now()
        now_sec = now_ros.nanoseconds * 1e-9

        pitch = float(data.get('pitch', 0.0))
        roll  = float(data.get('roll',  0.0))
        dp    = float(data.get('dp',    0.0))   # pitch rate (rad/s)
        dr    = float(data.get('dr',    0.0))   # roll rate  (rad/s)
        ax    = float(data.get('ax',    0.0))
        ay    = float(data.get('ay',    0.0))
        az    = float(data.get('az',    9.81))

        # sensor_msgs/Imu 
        imu_msg = Imu()
        imu_msg.header.stamp    = now_ros.to_msg()
        imu_msg.header.frame_id = self._frame_id

        # orientation: convert pitch/roll to quaternion (yaw=0 assumed)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cr = math.cos(roll  / 2.0)
        sr = math.sin(roll  / 2.0)
        imu_msg.orientation.w = cp * cr
        imu_msg.orientation.x = sp * cr
        imu_msg.orientation.y = cp * sr
        imu_msg.orientation.z = sp * sr   # yaw cross-term (small)

        # orientation covariance: diagonal, rough estimate
        oc = 0.01 ** 2   # ~0.6° std dev
        imu_msg.orientation_covariance = [
            oc,  0,  0,
             0, oc,  0,
             0,  0, oc
        ]

        # angular velocity
        imu_msg.angular_velocity.x = dr
        imu_msg.angular_velocity.y = dp
        imu_msg.angular_velocity.z = 0.0
        gc = 0.003 ** 2
        imu_msg.angular_velocity_covariance = [
            gc,  0,  0,
             0, gc,  0,
             0,  0, gc
        ]

        # linear acceleration
        imu_msg.linear_acceleration.x = ax
        imu_msg.linear_acceleration.y = ay
        imu_msg.linear_acceleration.z = az
        ac = 0.05 ** 2
        imu_msg.linear_acceleration_covariance = [
            ac,  0,  0,
             0, ac,  0,
             0,  0, ac
        ]

        self._pub_imu.publish(imu_msg)

        # FK-derived setpoint 
        # The IMU measures absolute tilt. We want to correct deviations from
        # the kinematically-level pose, not from IMU zero.
        # fk_flex is what wrist_flex should be to keep the cup horizontal;
        # the IMU pitch at that pose is our reference, not zero.
        #
        # Since the IMU is calibrated to the wrist, when the wrist is at the
        # FK setpoint the cup IS level, so the IMU reading at that moment is
        # our target pitch. We track the difference between current IMU pitch
        # and the pitch that corresponds to the FK-level pose.
        #
        # Simplified: error = IMU_pitch - FK_pitch_offset
        # FK_pitch_offset ≈ (actual wrist_flex - fk_setpoint) scaled by sensor mount.
        # For a wrist-mounted IMU, pitch offset ≈ (wrist_flex_actual - flex_setpoint).
        fk_flex_setpoint, fk_roll_setpoint = self._compute_fk_setpoint()

        if fk_flex_setpoint is not None:
            with self._joints_lock:
                wrist_flex_actual = self._joint_positions.get('wrist_flex', 0.0)
                wrist_roll_actual = self._joint_positions.get('wrist_roll', 0.0)
            # Residual joint error (how far the wrist is from kinematically level)
            flex_joint_error = wrist_flex_actual - fk_flex_setpoint
            roll_joint_error = wrist_roll_actual - fk_roll_setpoint
            # IMU error is the tilt reading minus the expected tilt from joint offset
            pitch_err = pitch - flex_joint_error
            roll_err  = roll  - roll_joint_error
        else:
            # No FK data yet — fall back to raw IMU error
            pitch_err = pitch
            roll_err  = roll

        self._err_window_pitch.append(abs(pitch_err))
        self._err_window_roll.append(abs(roll_err))

        err_msg = Vector3()
        err_msg.x = pitch_err
        err_msg.y = roll_err
        err_msg.z = 0.0
        self._pub_err.publish(err_msg)

        # State machine: check settle delay 
        if (not self._pid_active
                and self._arm_state == 'SETTLED'
                and self._settled_at is not None):
            settle_delay = self.get_parameter('settle_delay').value
            if time.monotonic() - self._settled_at >= settle_delay:
                self._pid_active = True
                self.get_logger().info('PID now active — balancing cup.')
                self._pub_balance_enabled.publish(Bool(data=True))

        # PID correction commands 
        # Unified 2D error: compute tilt magnitude and direction, then project
        # onto joint axes. This prevents independent-axis corrections from
        # fighting each other when the tilt is diagonal.
        if self._pid_active:
            tilt_magnitude = math.sqrt(pitch_err ** 2 + roll_err ** 2)
            tilt_direction = math.atan2(roll_err, pitch_err)  # angle in error plane

            # Single PID on the magnitude (use pitch gains as the unified gains)
            magnitude_cmd = self._pid_pitch.update(tilt_magnitude, now_sec)

            # Project correction back onto pitch and roll axes
            pitch_cmd = magnitude_cmd * math.cos(tilt_direction)
            roll_cmd  = magnitude_cmd * math.sin(tilt_direction)

            # Keep roll PID integral in sync (reset it since we're not using it directly)
            self._pid_roll.reset()
        else:
            pitch_cmd = 0.0
            roll_cmd  = 0.0

        if self._inv_pitch:
            pitch_cmd = -pitch_cmd
        if self._inv_roll:
            roll_cmd  = -roll_cmd

        # pitch_cmd → wrist_flex joint delta (rad/s)
        # roll_cmd  → wrist_roll  joint delta (rad/s)
        cmd_msg = Vector3()
        cmd_msg.x = pitch_cmd   # wrist_flex
        cmd_msg.y = roll_cmd    # wrist_roll
        cmd_msg.z = 0.0
        self._pub_cmd.publish(cmd_msg)

        # Stability flag 
        # True when average error over rolling window is within deadband
        deadband = self.get_parameter('deadband_rad').value
        avg_pitch_err = sum(self._err_window_pitch) / max(len(self._err_window_pitch), 1)
        avg_roll_err  = sum(self._err_window_roll)  / max(len(self._err_window_roll),  1)
        is_stable = (avg_pitch_err < deadband * 3) and (avg_roll_err < deadband * 3)

        stable_msg = Bool()
        stable_msg.data = is_stable
        self._pub_stable.publish(stable_msg)

        # Debug diagnosis (only when debug_mode=True) 
        if self.get_parameter('debug_mode').value:
            fk_flex_str = f'{fk_flex_setpoint:.4f} rad' if fk_flex_setpoint is not None else 'unavailable'
            tilt_magnitude = math.sqrt(pitch_err ** 2 + roll_err ** 2)
            tilt_direction_deg = math.degrees(math.atan2(roll_err, pitch_err)) % 360.0

            # Map tilt direction to cup quadrant description.
            # 0° = robot-side edge of cup, 90° = right, 180° = far side, 270° = left.
            # The tilt_direction_deg tells us which edge of the cup is currently HIGH
            # (the ball rolls toward the LOW side, which is opposite).
            high_edge = tilt_direction_deg
            low_edge  = (tilt_direction_deg + 180.0) % 360.0

            def _quadrant_label(deg):
                deg = deg % 360.0
                if deg < 22.5 or deg >= 337.5:
                    return f'0° (robot-side)'
                elif deg < 67.5:
                    return f'45°'
                elif deg < 112.5:
                    return f'90° (your right)'
                elif deg < 157.5:
                    return f'135°'
                elif deg < 202.5:
                    return f'180° (far side)'
                elif deg < 247.5:
                    return f'225°'
                elif deg < 292.5:
                    return f'270° (your left)'
                else:
                    return f'315°'

            tilt_deg = math.degrees(tilt_magnitude)
            pid_state = ('ACTIVE' if self._pid_active
                         else ('SETTLING' if self._arm_state == 'SETTLED' else 'SUSPENDED'))

            diag_lines = [
                f'=== IMU Balance Diagnosis ===',
                f'ARM STATE : {self._arm_state}  PID: {pid_state}',
                f'FK setpoint  wrist_flex: {fk_flex_str}  wrist_roll: 0.0000 rad',
                f'IMU reading  pitch: {pitch:+.4f} rad  roll: {roll:+.4f} rad',
                f'Error vector magnitude: {tilt_magnitude:.4f} rad ({tilt_deg:.2f}°)  direction: {tilt_direction_deg:.1f}°',
                f'Cup tilt     {_quadrant_label(high_edge)} edge HIGH (+{tilt_deg/2:.2f}°)',
                f'             {_quadrant_label(low_edge)}  edge LOW  (-{tilt_deg/2:.2f}°)',
                f'Ball rolls → {_quadrant_label(low_edge)}',
                f'PID output   flex_cmd: {pitch_cmd:+.4f} rad/s  roll_cmd: {roll_cmd:+.4f} rad/s',
                f'Stable: {is_stable}',
            ]
            diag_msg = String()
            diag_msg.data = '\n'.join(diag_lines)
            self._pub_diagnosis.publish(diag_msg)

    # 
    def send_calibrate(self):
        """Send 'c' to ESP32 to trigger onboard recalibration."""
        if self._serial and self._serial.is_open:
            self._serial.write(b'c')
            self.get_logger().info('Sent calibration command to ESP32.')
        else:
            self.get_logger().warn('Serial port not open, cannot send calibrate.')

    def destroy_node(self):
        self._running = False
        if self._serial:
            self._serial.close()
        super().destroy_node()


# 
def main(args=None):
    rclpy.init(args=args)
    node = ImuBalanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
