#!/usr/bin/env python3
"""
wrist_balance_controller.py — Cup balancing controller for Koch v1.1

Subscribes to /imu/balance_cmd (geometry_msgs/Vector3) from imu_balance_node
and translates the PID corrections into wrist joint position adjustments,
sent via the JointTrajectory action to ros2_control.

Architecture:
  /imu/balance_cmd  →  this node  →  /joint_trajectory_controller/follow_joint_trajectory
  /imu/is_stable    →  logged / available for state machine use
  /joint_states     →  read current wrist positions as the base for corrections

Control loop:
  1. Receive balance_cmd (x=wrist_flex correction, y=wrist_roll correction) in rad/s
  2. Multiply by dt to get a position delta (rad)
  3. Clamp delta to max_delta_per_step to prevent jerky motion
  4. Add delta to current joint position from /joint_states
  5. Clamp result to joint limits
  6. Send as a short-duration JointTrajectory goal

Parameters (all tunable via ros2 param set at runtime):
  enabled            bool    Enable/disable corrections         default: True
  update_hz          float   Control loop rate (Hz)            default: 50.0
  max_delta_rad      float   Max position change per step (rad) default: 0.02  (~1.1°)
  move_duration      float   Trajectory point duration (s)     default: 0.05
  wrist_flex_joint   str     Joint name for pitch correction    default: wrist_flex
  wrist_roll_joint   str     Joint name for roll correction     default: wrist_roll
  flex_scale         float   Scale factor for flex correction   default: 1.0
  roll_scale         float   Scale factor for roll correction   default: 1.0
  flex_min_rad       float   wrist_flex lower limit (rad)       default: -1.57
  flex_max_rad       float   wrist_flex upper limit (rad)       default:  1.57
  roll_min_rad       float   wrist_roll lower limit (rad)       default: -1.57
  roll_max_rad       float   wrist_roll upper limit (rad)       default:  1.57

Topics subscribed:
  /imu/balance_cmd         geometry_msgs/Vector3   PID corrections (rad/s)
  /imu/is_stable           std_msgs/Bool           Stability flag
  /joint_states            sensor_msgs/JointState  Current joint positions

Topics/actions used:
  /joint_trajectory_controller/follow_joint_trajectory   (action, SendGoal)

Usage:
  ros2 run writing_robot_control wrist_balance_controller

  # Disable corrections while arm is moving to a new pose:
  ros2 param set /wrist_balance_controller enabled false

  # Re-enable:
  ros2 param set /wrist_balance_controller enabled true

  # Reduce aggressiveness if oscillating:
  ros2 param set /wrist_balance_controller max_delta_rad 0.01

Tuning guide:
  - If the arm oscillates: reduce max_delta_rad or reduce PID gains in imu_balance_node
  - If the arm is slow to respond: increase max_delta_rad or increase Kp in imu_balance_node
  - If one axis is inverted: set flex_scale or roll_scale to -1.0
  - The PID gains live in imu_balance_node, not here — tune there first
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import math
import threading
import time


class WristBalanceController(Node):

    def __init__(self):
        super().__init__('wrist_balance_controller')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('enabled',           True)
        self.declare_parameter('update_hz',         50.0)
        self.declare_parameter('max_delta_rad',     0.02)
        self.declare_parameter('move_duration',     0.05)
        self.declare_parameter('wrist_flex_joint',  'wrist_flex')
        self.declare_parameter('wrist_roll_joint',  'wrist_roll')
        self.declare_parameter('flex_scale',        1.0)
        self.declare_parameter('roll_scale',        1.0)
        self.declare_parameter('flex_min_rad',      0.297)   # wrist_flex lower bound
        self.declare_parameter('flex_max_rad',      2.700)   # wrist_flex upper bound
        self.declare_parameter('roll_min_rad',     -1.448)   # wrist_roll lower bound
        self.declare_parameter('roll_max_rad',      1.900)   # wrist_roll upper bound
        self.declare_parameter('dry_run',           False)  # log intended actions, don't actuate

        # ── State ────────────────────────────────────────────────────────
        self._joint_positions = {}   # name → current position (rad)
        self._latest_cmd      = None # most recent Vector3 from /imu/balance_cmd
        self._is_stable       = False
        self._balance_active  = False  # set by /balance_enabled from imu_balance_node
        self._lock            = threading.Lock()
        self._last_cmd_time   = None

        # ── Subscribers ──────────────────────────────────────────────────
        self.create_subscription(
            Vector3, '/imu/balance_cmd',
            self._cmd_callback, 10)

        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10)

        self.create_subscription(
            Bool, '/imu/is_stable',
            self._stable_callback, 10)

        # imu_balance_node publishes True when arm is SETTLED and PID is active,
        # False when arm is MOVING.  This takes priority over the 'enabled' param
        # so pose_test drives the gate automatically without manual param-sets.
        self.create_subscription(
            Bool, '/balance_enabled',
            self._balance_enabled_callback, 10)

        # ── Action client ────────────────────────────────────────────────
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Human-readable intended action report (published in dry_run mode)
        self._pub_intended = self.create_publisher(String, '/balance/intended_action', 10)

        # ── Control timer ────────────────────────────────────────────────
        hz = self.get_parameter('update_hz').value
        self._timer = self.create_timer(1.0 / hz, self._control_loop)

        # Track in-flight action goal to avoid flooding
        self._goal_in_flight = False
        self._goal_lock = threading.Lock()

        self.get_logger().info('Wrist balance controller started.')
        self.get_logger().info(
            f'Joints: {self.get_parameter("wrist_flex_joint").value} (pitch), '
            f'{self.get_parameter("wrist_roll_joint").value} (roll)'
        )
        self.get_logger().info(
            "Disable with: ros2 param set /wrist_balance_controller enabled false"
        )
        if self.get_parameter('dry_run').value:
            self.get_logger().info(
                'DRY RUN MODE — no trajectories will be sent.\n'
                '  Monitor intended actions: ros2 topic echo /balance/intended_action'
            )

    # ────────────────────────────────────────────────────────────────────
    def _cmd_callback(self, msg: Vector3):
        with self._lock:
            self._latest_cmd   = msg
            self._last_cmd_time = time.monotonic()

    def _joint_state_callback(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def _stable_callback(self, msg: Bool):
        self._is_stable = msg.data

    def _balance_enabled_callback(self, msg: Bool):
        """
        Driven by imu_balance_node when arm transitions MOVING↔SETTLED.
        Overrides the 'enabled' parameter so pose_test controls the gate
        automatically without requiring manual ros2 param set calls.
        """
        if msg.data != self._balance_active:
            self._balance_active = msg.data
            self.get_logger().info(
                f'Balance {"ENABLED" if msg.data else "DISABLED"} '
                f'(from /balance_enabled topic)'
            )

    # ────────────────────────────────────────────────────────────────────
    def _control_loop(self):
        """Main control loop — runs at update_hz."""

        # 'enabled' param is a manual override (e.g. for debugging).
        # _balance_active is driven automatically by imu_balance_node.
        if not self.get_parameter('enabled').value:
            return
        if not self._balance_active:
            return

        with self._lock:
            cmd            = self._latest_cmd
            joint_pos      = dict(self._joint_positions)
            last_cmd_time  = self._last_cmd_time

        # Safety: ignore stale commands (> 0.5s old)
        if cmd is None or last_cmd_time is None:
            return
        if time.monotonic() - last_cmd_time > 0.5:
            self.get_logger().warn('IMU command stale (>0.5s), skipping correction.',
                                   throttle_duration_sec=5.0)
            return

        flex_joint = self.get_parameter('wrist_flex_joint').value
        roll_joint = self.get_parameter('wrist_roll_joint').value

        if flex_joint not in joint_pos or roll_joint not in joint_pos:
            self.get_logger().warn(
                f'Joint positions not yet available for {flex_joint} / {roll_joint}',
                throttle_duration_sec=5.0)
            return

        # Don't send a new goal if one is already in flight
        with self._goal_lock:
            if self._goal_in_flight:
                return

        dt           = 1.0 / self.get_parameter('update_hz').value
        max_delta    = self.get_parameter('max_delta_rad').value
        flex_scale   = self.get_parameter('flex_scale').value
        roll_scale   = self.get_parameter('roll_scale').value
        move_dur     = self.get_parameter('move_duration').value

        # cmd.x = pitch correction (rad/s) → wrist_flex
        # cmd.y = roll  correction (rad/s) → wrist_roll
        flex_delta = float(cmd.x) * flex_scale * dt
        roll_delta = float(cmd.y) * roll_scale * dt

        # Clamp per-step delta
        flex_delta = max(-max_delta, min(max_delta, flex_delta))
        roll_delta = max(-max_delta, min(max_delta, roll_delta))

        # Skip if both deltas are negligible (within 10% of max_delta)
        if abs(flex_delta) < max_delta * 0.1 and abs(roll_delta) < max_delta * 0.1:
            return

        # Compute new target positions
        new_flex = joint_pos[flex_joint] + flex_delta
        new_roll = joint_pos[roll_joint] + roll_delta

        # Clamp to joint limits — warn if clamping fires (sign of over-aggressive gains)
        flex_min = self.get_parameter('flex_min_rad').value
        flex_max = self.get_parameter('flex_max_rad').value
        roll_min = self.get_parameter('roll_min_rad').value
        roll_max = self.get_parameter('roll_max_rad').value

        clamped_flex = max(flex_min, min(flex_max, new_flex))
        clamped_roll = max(roll_min, min(roll_max, new_roll))

        if clamped_flex != new_flex:
            self.get_logger().warn(
                f'wrist_flex clamped: requested {new_flex:.4f} rad, '
                f'limited to {clamped_flex:.4f} rad  '
                f'(limit: [{flex_min:.2f}, {flex_max:.2f}]) — consider reducing gains.',
                throttle_duration_sec=2.0)
        if clamped_roll != new_roll:
            self.get_logger().warn(
                f'wrist_roll clamped: requested {new_roll:.4f} rad, '
                f'limited to {clamped_roll:.4f} rad  '
                f'(limit: [{roll_min:.2f}, {roll_max:.2f}]) — consider reducing gains.',
                throttle_duration_sec=2.0)

        new_flex = clamped_flex
        new_roll = clamped_roll

        # ── Cup quadrant effect report ───────────────────────────────────
        # Coordinate convention (facing the robot head-on):
        #   0°  = robot-side edge of cup
        #   90° = right edge
        #   180° = far edge
        #   270° = left edge
        #
        # wrist_flex increasing → 0° edge moves UP,  180° edge moves DOWN
        # wrist_roll increasing → 90° edge moves UP,  270° edge moves DOWN
        # (Signs may need inversion after physical validation.)
        def _cup_effect(flex_d, roll_d):
            lines = []
            if abs(flex_d) > 1e-5:
                if flex_d > 0:
                    lines.append(f'  0° (robot-side) UP   by {math.degrees(abs(flex_d)):.3f}°')
                    lines.append(f'  180° (far side) DOWN by {math.degrees(abs(flex_d)):.3f}°')
                else:
                    lines.append(f'  0° (robot-side) DOWN by {math.degrees(abs(flex_d)):.3f}°')
                    lines.append(f'  180° (far side) UP   by {math.degrees(abs(flex_d)):.3f}°')
            if abs(roll_d) > 1e-5:
                if roll_d > 0:
                    lines.append(f'  90° (your right) UP   by {math.degrees(abs(roll_d)):.3f}°')
                    lines.append(f'  270° (your left) DOWN by {math.degrees(abs(roll_d)):.3f}°')
                else:
                    lines.append(f'  90° (your right) DOWN by {math.degrees(abs(roll_d)):.3f}°')
                    lines.append(f'  270° (your left) UP   by {math.degrees(abs(roll_d)):.3f}°')
            return lines if lines else ['  (no significant correction)']

        cup_lines = _cup_effect(flex_delta, roll_delta)

        # ── Dry run: log and publish intended action, do not actuate ────
        if self.get_parameter('dry_run').value:
            report_lines = [
                '=== Wrist Balance Controller — DRY RUN ===',
                f'  {flex_joint}: {joint_pos[flex_joint]:.4f} → {new_flex:.4f} rad  '
                f'(Δ {flex_delta:+.4f} rad)',
                f'  {roll_joint}: {joint_pos[roll_joint]:.4f} → {new_roll:.4f} rad  '
                f'(Δ {roll_delta:+.4f} rad)',
                'Cup edge motion (positive = edge rises):',
            ] + cup_lines
            report = '\n'.join(report_lines)
            self.get_logger().info(report, throttle_duration_sec=0.5)
            msg = String()
            msg.data = report
            self._pub_intended.publish(msg)
            return   # ← do not actuate

        self._send_trajectory(flex_joint, roll_joint, new_flex, new_roll, move_dur)

    # ────────────────────────────────────────────────────────────────────
    def _send_trajectory(self, flex_joint, roll_joint,
                         flex_pos, roll_pos, duration_sec):
        """Send a 2-joint trajectory goal for the wrist joints only."""

        if not self._action_client.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('Action server not available', 
                                   throttle_duration_sec=5.0)
            return

        # Build trajectory message
        traj = JointTrajectory()
        traj.joint_names = [flex_joint, roll_joint]

        point = JointTrajectoryPoint()
        point.positions = [flex_pos, roll_pos]
        point.velocities = [0.0, 0.0]   # let controller interpolate

        # Duration as builtin_interfaces/Duration
        secs     = int(duration_sec)
        nanosecs = int((duration_sec - secs) * 1e9)
        point.time_from_start = Duration(sec=secs, nanosec=nanosecs)

        traj.points = [point]

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        with self._goal_lock:
            self._goal_in_flight = True

        future = self._action_client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    # ────────────────────────────────────────────────────────────────────
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Trajectory goal rejected')
            with self._goal_lock:
                self._goal_in_flight = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)

    def _goal_result_callback(self, future):
        with self._goal_lock:
            self._goal_in_flight = False


# ═══════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = WristBalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
