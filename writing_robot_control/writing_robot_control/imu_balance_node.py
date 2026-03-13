#!/usr/bin/env python3
"""
imu_balance_node.py — ROS2 node for ESP32/MPU-6050 cup-balancing IMU

Reads the JSON stream from the ESP32 over serial, publishes:
  /imu/raw           sensor_msgs/Imu          — fused pitch/roll + rates
  /imu/balance_error geometry_msgs/Vector3    — (pitch_err, roll_err, 0)
  /imu/balance_cmd   geometry_msgs/Vector3    — (wrist_flex_delta, wrist_roll_delta, 0)
                                                scaled PID output, ready to feed
                                                into a joint velocity commander

The balance_cmd topic gives you the joint correction signals directly.
Wire these into a joint velocity publisher (or MoveIt Servo) in a separate node.

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

Axis convention:
  pitch → wrist_flex joint correction
  roll  → wrist_roll joint correction

If the correction direction is backwards, use invert_pitch/invert_roll params
rather than editing code — makes it easy to flip at launch time.

Dependencies:
  pip install pyserial --break-system-packages
  ros2 packages: rclpy, sensor_msgs, geometry_msgs, std_msgs

Notes: this draws heavily from: https://web2.qatar.cmu.edu/~gdicaro/16311-Fall17/slides/PID-without-PhD.pdf
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3
from std_msgs.msg import Bool

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

        # State 
        self._latest = None          # most recent parsed JSON dict
        self._lock   = threading.Lock()
        self._serial = None
        self._running = True

        # Rolling window for stability detection (last N samples)
        self._err_window_pitch = deque(maxlen=20)
        self._err_window_roll  = deque(maxlen=20)

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

    def _serial_reader(self):
        """Background thread: open serial port and read JSON lines."""
        hash_cnt = 0
        hash_cnt_limit = 1000
        while self._running:
            try:
                self.get_logger().info(f'Opening serial port {self._port}...')
                self._serial = serial.Serial(self._port, self._baud, timeout=1.0)
                self.get_logger().info('Serial port open.')

                while self._running:
                    line = self._serial.readline().decode('utf-8', errors='ignore').strip()

                    if not line or line.startswith('#'):
                        if line.startswith('#'):
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

        # Balance error 
        # Setpoint is 0 (the calibrated "level" position on the ESP32).
        # Error = current tilt angle (already zeroed by calibration on ESP32).
        pitch_err = pitch
        roll_err  = roll

        self._err_window_pitch.append(abs(pitch_err))
        self._err_window_roll.append(abs(roll_err))

        err_msg = Vector3()
        err_msg.x = pitch_err
        err_msg.y = roll_err
        err_msg.z = 0.0
        self._pub_err.publish(err_msg)

        # PID correction commands 
        pitch_cmd = self._pid_pitch.update(pitch_err, now_sec)
        roll_cmd  = self._pid_roll.update(roll_err,  now_sec)

        if self._inv_pitch:
            pitch_cmd = -pitch_cmd
        if self._inv_roll:
            roll_cmd  = -roll_cmd

        # pitch_cmd : wrist_flex joint delta (rad/s)
        # roll_cmd  : wrist_roll  joint delta (rad/s)
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
