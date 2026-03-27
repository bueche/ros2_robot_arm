#!/usr/bin/env python3
"""
balance_test_injector.py — Synthetic test signal injector for cup-balancing pipeline

Publishes fake IMU data and arm state signals to exercise the
imu_balance_node + wrist_balance_controller pipeline without any hardware.
Use this to verify that:

  1. IMU orientation is interpreted correctly (pitch/roll sign and axis mapping)
  2. FK setpoint computation produces sensible wrist angles across arm poses
  3. Correction direction is correct — tilt toward 90° → 90° edge commanded DOWN
  4. wrist_balance_controller dry_run output matches expectation

The injector does NOT touch the actual servos.  Run wrist_balance_controller
with dry_run:=true and watch /balance/intended_action to see what would happen.

Usage:
  # List available scenarios:
  ros2 run writing_robot_control balance_test_injector --ros-args -p scenario:=list

  # Run a specific scenario:
  ros2 run writing_robot_control balance_test_injector \\
    --ros-args -p scenario:=tilt_toward_90 -p magnitude:=0.1

  # Override arm pose (radians):
  ros2 run writing_robot_control balance_test_injector \\
    --ros-args -p scenario:=tilt_toward_0 \\
               -p shoulder_lift:=2.5 -p elbow_flex:=1.2 -p wrist_flex:=1.5

  # Inject a sweep of tilt directions (360° in steps):
  ros2 run writing_robot_control balance_test_injector \\
    --ros-args -p scenario:=sweep_360 -p magnitude:=0.08

Monitor outputs:
  ros2 topic echo /imu/balance_diagnosis        # from imu_balance_node (debug_mode:=true)
  ros2 topic echo /balance/intended_action      # from wrist_balance_controller (dry_run:=true)
  ros2 topic echo /imu/balance_error            # raw error vector

Scenarios:
  level           — cup perfectly level (should produce zero correction)
  tilt_toward_0   — 0° (robot-side) edge high, ball rolls toward 180°
  tilt_toward_90  — 90° (your right) edge high, ball rolls toward 270°
  tilt_toward_180 — 180° (far side) edge high, ball rolls toward 0°
  tilt_toward_270 — 270° (your left) edge high, ball rolls toward 90°
  tilt_diagonal   — 45° tilt (between 0° and 90°)
  sweep_360       — continuously rotate tilt direction through all quadrants
  custom          — use pitch:= and roll:= params directly
  list            — print this list and exit

Parameters:
  scenario        Scenario name (see above)           default: level
  magnitude       Tilt angle in radians               default: 0.05
  pitch           Manual pitch override (rad)          default: 0.0
  roll            Manual roll override (rad)           default: 0.0
  publish_hz      Injection rate (Hz)                  default: 10.0
  arm_state       'MOVING' or 'SETTLED'               default: SETTLED
  shoulder_lift   Simulated joint angle (rad)          default: 2.6
  elbow_flex      Simulated joint angle (rad)          default: 1.2
  wrist_flex      Simulated joint angle (rad)          default: 1.5
  wrist_roll      Simulated joint angle (rad)          default: 0.0
  sweep_period    Seconds for one full 360° sweep      default: 10.0
  duration        How long to run in seconds (0=forever) default: 0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, JointState
from geometry_msgs.msg import Vector3
from std_msgs.msg import String, Bool
from builtin_interfaces.msg import Duration

import math
import time


SCENARIOS = [
    'level',
    'tilt_toward_0',
    'tilt_toward_90',
    'tilt_toward_180',
    'tilt_toward_270',
    'tilt_diagonal',
    'sweep_360',
    'custom',
    'list',
]


class BalanceTestInjector(Node):

    def __init__(self):
        super().__init__('balance_test_injector')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('scenario',       'level')
        self.declare_parameter('magnitude',      0.05)
        self.declare_parameter('pitch',          0.0)
        self.declare_parameter('roll',           0.0)
        self.declare_parameter('publish_hz',     10.0)
        self.declare_parameter('arm_state',      'SETTLED')
        self.declare_parameter('shoulder_lift',  2.6)
        self.declare_parameter('elbow_flex',     1.2)
        self.declare_parameter('wrist_flex',     1.5)
        self.declare_parameter('wrist_roll',     0.0)
        self.declare_parameter('sweep_period',   10.0)
        self.declare_parameter('duration',       0.0)

        self._scenario     = self.get_parameter('scenario').value
        self._magnitude    = self.get_parameter('magnitude').value
        self._custom_pitch = self.get_parameter('pitch').value
        self._custom_roll  = self.get_parameter('roll').value
        self._hz           = self.get_parameter('publish_hz').value
        self._arm_state    = self.get_parameter('arm_state').value
        self._shoulder_lift= self.get_parameter('shoulder_lift').value
        self._elbow_flex   = self.get_parameter('elbow_flex').value
        self._wrist_flex   = self.get_parameter('wrist_flex').value
        self._wrist_roll   = self.get_parameter('wrist_roll').value
        self._sweep_period = self.get_parameter('sweep_period').value
        self._duration     = self.get_parameter('duration').value

        # Handle 'list' scenario
        if self._scenario == 'list':
            self.get_logger().info('Available scenarios:')
            for s in SCENARIOS:
                self.get_logger().info(f'  {s}')
            raise SystemExit(0)

        if self._scenario not in SCENARIOS:
            self.get_logger().error(
                f'Unknown scenario: "{self._scenario}". '
                f'Use -p scenario:=list to see options.'
            )
            raise SystemExit(1)

        # ── Publishers ──────────────────────────────────────────────────
        self._pub_imu        = self.create_publisher(Imu,       '/imu/raw',      10)
        self._pub_arm_state  = self.create_publisher(String,    '/arm_state',    10)
        self._pub_joint      = self.create_publisher(JointState,'/joint_states', 10)

        # ── State ────────────────────────────────────────────────────────
        self._start_time = time.monotonic()
        self._step = 0

        # ── Timer ────────────────────────────────────────────────────────
        self._timer = self.create_timer(1.0 / self._hz, self._inject)

        self.get_logger().info('=' * 55)
        self.get_logger().info('BALANCE TEST INJECTOR')
        self.get_logger().info('=' * 55)
        self.get_logger().info(f'Scenario   : {self._scenario}')
        self.get_logger().info(f'Magnitude  : {self._magnitude:.4f} rad  ({math.degrees(self._magnitude):.2f}°)')
        self.get_logger().info(f'Arm state  : {self._arm_state}')
        self.get_logger().info(f'Arm pose   : shoulder_lift={self._shoulder_lift:.3f}  '
                               f'elbow_flex={self._elbow_flex:.3f}  '
                               f'wrist_flex={self._wrist_flex:.3f}')
        fk = math.pi - (self._shoulder_lift + self._elbow_flex)
        self.get_logger().info(f'FK setpoint: wrist_flex={fk:.4f} rad  '
                               f'(actual={self._wrist_flex:.4f}, '
                               f'residual={self._wrist_flex - fk:+.4f})')
        self.get_logger().info('-' * 55)
        self.get_logger().info('Monitor with:')
        self.get_logger().info('  ros2 topic echo /imu/balance_diagnosis')
        self.get_logger().info('  ros2 topic echo /balance/intended_action')
        self.get_logger().info('=' * 55)

    # ────────────────────────────────────────────────────────────────────
    def _compute_pitch_roll(self):
        """Return (pitch, roll) in radians for the current scenario/step."""
        m = self._magnitude
        elapsed = time.monotonic() - self._start_time

        if self._scenario == 'level':
            return 0.0, 0.0

        elif self._scenario == 'tilt_toward_0':
            # 0° edge HIGH → pitch positive (cup tilts toward robot side up)
            return m, 0.0

        elif self._scenario == 'tilt_toward_90':
            # 90° (right) edge HIGH → roll positive
            return 0.0, m

        elif self._scenario == 'tilt_toward_180':
            # 180° (far) edge HIGH → pitch negative
            return -m, 0.0

        elif self._scenario == 'tilt_toward_270':
            # 270° (left) edge HIGH → roll negative
            return 0.0, -m

        elif self._scenario == 'tilt_diagonal':
            # 45° tilt — equal pitch and roll components
            c = m / math.sqrt(2.0)
            return c, c

        elif self._scenario == 'sweep_360':
            # Rotate tilt direction continuously through 360°
            angle = (2.0 * math.pi * elapsed) / self._sweep_period
            return m * math.cos(angle), m * math.sin(angle)

        elif self._scenario == 'custom':
            return self._custom_pitch, self._custom_roll

        return 0.0, 0.0

    # ────────────────────────────────────────────────────────────────────
    def _inject(self):
        """Publish fake IMU, joint state, and arm state."""

        # Check duration limit
        if self._duration > 0:
            elapsed = time.monotonic() - self._start_time
            if elapsed > self._duration:
                self.get_logger().info(f'Duration {self._duration:.1f}s elapsed — stopping.')
                self._timer.cancel()
                return

        now = self.get_clock().now().to_msg()
        pitch, roll = self._compute_pitch_roll()

        # ── Fake IMU message ─────────────────────────────────────────────
        imu_msg = Imu()
        imu_msg.header.stamp    = now
        imu_msg.header.frame_id = 'imu_link'

        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cr = math.cos(roll  / 2.0)
        sr = math.sin(roll  / 2.0)
        imu_msg.orientation.w = cp * cr
        imu_msg.orientation.x = sp * cr
        imu_msg.orientation.y = cp * sr
        imu_msg.orientation.z = sp * sr

        oc = 0.01 ** 2
        imu_msg.orientation_covariance = [oc, 0, 0, 0, oc, 0, 0, 0, oc]

        # Fake gyro — zero rates (static tilt)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = 0.0
        gc = 0.003 ** 2
        imu_msg.angular_velocity_covariance = [gc, 0, 0, 0, gc, 0, 0, 0, gc]

        # Fake accelerometer consistent with tilt
        imu_msg.linear_acceleration.x = 9.81 * math.sin(pitch)
        imu_msg.linear_acceleration.y = 9.81 * math.sin(roll)
        imu_msg.linear_acceleration.z = 9.81 * math.cos(pitch) * math.cos(roll)
        ac = 0.05 ** 2
        imu_msg.linear_acceleration_covariance = [ac, 0, 0, 0, ac, 0, 0, 0, ac]

        self._pub_imu.publish(imu_msg)

        # ── Fake joint states ────────────────────────────────────────────
        js = JointState()
        js.header.stamp = now
        js.name = [
            'shoulder_pan', 'shoulder_lift', 'elbow_flex',
            'wrist_flex', 'wrist_roll', 'pen_holder'
        ]
        js.position = [
            1.55,
            self._shoulder_lift,
            self._elbow_flex,
            self._wrist_flex,
            self._wrist_roll,
            0.9
        ]
        js.velocity = [0.0] * 6
        js.effort   = [0.0] * 6
        self._pub_joint.publish(js)

        # ── Arm state ────────────────────────────────────────────────────
        state_msg = String()
        state_msg.data = self._arm_state
        self._pub_arm_state.publish(state_msg)

        # ── Console summary (every 10 steps) ────────────────────────────
        self._step += 1
        if self._step % (max(1, int(self._hz / 2))) == 0:
            tilt_mag = math.sqrt(pitch**2 + roll**2)
            tilt_dir = math.degrees(math.atan2(roll, pitch)) % 360.0
            self.get_logger().info(
                f'[{self._scenario}]  pitch={pitch:+.4f}  roll={roll:+.4f}  '
                f'|tilt|={tilt_mag:.4f} rad ({math.degrees(tilt_mag):.2f}°)  '
                f'dir={tilt_dir:.1f}°  arm={self._arm_state}'
            )


# ═══════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    try:
        node = BalanceTestInjector()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
