#!/usr/bin/env python3
"""
wrist_balance_controller.py — Wrist correction controller for ball balancing

Uses JointTrajectory topic (not action) for safe, non-flooding corrections.

Key safety design:
  - correction_hz=2Hz: one correction every 500ms, giving joints time to move
  - max_step_rad=0.05: max 2.9 degrees per correction
  - max_total_rad=0.3: max 17 degrees total displacement from start pose
  - move_duration=0.3s: 300ms to reach each target

Parameters:
  enabled            Master enable                           default: True
  dry_run            Log only, don't publish                 default: False
  correction_hz      Correction rate (Hz)                    default: 2.0
  move_duration      Trajectory duration per step (s)        default: 0.3
  max_step_rad           Max per-correction displacement (rad)       default: 0.05
  stall_scale_enable     Scale max_step_rad up when servo stalls      default: True
  stall_scale_max        Max multiplier for stall scaling             default: 3.0
  stall_entry_steps      Consecutive bad steps to enter boost         default: 1
  stall_exit_steps       Consecutive good steps to release boost      default: 3
  stall_threshold        Achievement ratio below which stall fires    default: 0.50
  stall_exit_threshold   Achievement ratio above which recovery counts default: 0.70
  max_total_rad      Max total displacement from start (rad) default: 0.3
  cmd_timeout        Stale command threshold (s)             default: 2.0
  wrist_flex_joint   Flex joint name                         default: wrist_flex
  wrist_roll_joint   Roll joint name                         default: wrist_roll
  flex_scale         Flex correction scale                   default: 1.0
  roll_scale         Roll correction scale                   default: 1.0
  flex_min_rad       Flex hard minimum                       default: 0.297
  flex_max_rad       Flex hard maximum                       default: 2.700
  roll_min_rad       Roll hard minimum                       default: -1.448
  roll_max_rad       Roll hard maximum                       default: 1.900
"""

import math
import threading
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class WristBalanceController(Node):

    def __init__(self):
        super().__init__('wrist_balance_controller')

        # Parameters
        self.declare_parameter('enabled',          True)
        self.declare_parameter('dry_run',          False)
        self.declare_parameter('correction_hz',    2.0)
        self.declare_parameter('move_duration',    0.3)
        self.declare_parameter('max_step_rad',         0.05)
        self.declare_parameter('stall_scale_enable',   True)
        self.declare_parameter('stall_scale_max',      3.0)
        self.declare_parameter('stall_entry_steps',    1)
        self.declare_parameter('stall_exit_steps',     3)
        self.declare_parameter('stall_threshold',      0.50)
        self.declare_parameter('stall_exit_threshold', 0.70)
        self.declare_parameter('max_total_rad',      0.3)   # fallback for both axes
        self.declare_parameter('max_total_flex_rad', -1.0)  # -1 = use max_total_rad
        self.declare_parameter('max_total_roll_rad', -1.0)  # -1 = use max_total_rad
        self.declare_parameter('cmd_timeout',      2.0)
        self.declare_parameter('wrist_flex_joint', 'wrist_flex')
        self.declare_parameter('wrist_roll_joint', 'wrist_roll')
        self.declare_parameter('flex_scale',       1.0)
        self.declare_parameter('roll_scale',       1.0)
        self.declare_parameter('flex_min_rad',     0.297)
        self.declare_parameter('flex_max_rad',     2.700)
        self.declare_parameter('roll_min_rad',    -1.448)
        self.declare_parameter('roll_max_rad',     1.900)

        # Cache hot-loop parameters
        self._p_enabled     = self.get_parameter('enabled').value
        self._p_dry_run     = self.get_parameter('dry_run').value
        self._p_corr_hz     = self.get_parameter('correction_hz').value
        self._p_move_dur    = self.get_parameter('move_duration').value
        self._p_max_step    = self.get_parameter('max_step_rad').value
        _max_total           = self.get_parameter('max_total_rad').value
        _flex_override       = self.get_parameter('max_total_flex_rad').value
        _roll_override       = self.get_parameter('max_total_roll_rad').value
        self._p_max_total_flex = _flex_override if _flex_override > 0 else _max_total
        self._p_max_total_roll = _roll_override if _roll_override > 0 else _max_total
        self._p_cmd_timeout = self.get_parameter('cmd_timeout').value
        self._p_flex_joint  = self.get_parameter('wrist_flex_joint').value
        self._p_roll_joint  = self.get_parameter('wrist_roll_joint').value
        self._p_flex_scale  = self.get_parameter('flex_scale').value
        self._p_roll_scale  = self.get_parameter('roll_scale').value
        self._p_flex_min    = self.get_parameter('flex_min_rad').value
        self._p_flex_max    = self.get_parameter('flex_max_rad').value
        self._p_roll_min    = self.get_parameter('roll_min_rad').value
        self._p_roll_max    = self.get_parameter('roll_max_rad').value

        # State
        self._lock            = threading.Lock()
        self._joint_positions = {}
        self._latest_cmd      = None
        self._last_cmd_time   = None
        self._balance_active  = False

        # Displacement tracking — reset each time balance activates
        self._start_flex  = None
        self._start_roll  = None
        self._total_flex  = 0.0
        self._total_roll  = 0.0

        # Per-axis stall detection.
        # Entry: 1 step below stall_threshold immediately activates boost.
        # Exit:  stall_exit_steps consecutive steps above stall_exit_threshold
        #        required before boost is released. Asymmetric by design —
        #        stalls are urgent, recovery must be sustained.
        # Separate per axis — roll may be fine while flex stalls (pose 5).
        self._flex_ratios      = []    # recent true achievement ratios (for logging)
        self._roll_ratios      = []
        self._flex_stalled     = False  # boost currently active
        self._roll_stalled     = False
        self._flex_good_streak = 0      # consecutive steps above exit threshold
        self._roll_good_streak = 0
        self._prev_target_flex = None  # target set by previous CORR step
        self._prev_target_roll = None
        self._prev_cmd_flex    = None  # commanded delta for previous step
        self._prev_cmd_roll    = None

        # Publishers
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/koch_v11_controller/joint_trajectory',
            10)
        self._pub_intended = self.create_publisher(
            String, '/balance/intended_action', 10)
        self._pub_cmd_delta = self.create_publisher(
            Vector3, '/balance/cmd_delta', 10)
        self._pub_achieved_delta = self.create_publisher(
            Vector3, '/balance/achieved_delta', 10)

        # Subscribers
        self.create_subscription(
            Vector3, '/imu/balance_cmd',
            self._cmd_callback, 10)
        self.create_subscription(
            JointState, '/joint_states',
            self._joint_state_callback, 10)
        self.create_subscription(
            Bool, '/imu/is_stable',
            lambda msg: None, 10)
        self.create_subscription(
            Bool, '/balance_enabled',
            self._balance_enabled_callback, 10)

        # Control timer
        self._timer = self.create_timer(
            1.0 / self._p_corr_hz, self._control_loop)

        self.get_logger().info('Wrist balance controller started.')
        self.get_logger().info(
            f'Joints: {self._p_flex_joint} (pitch), {self._p_roll_joint} (roll)')
        self.get_logger().info(
            f'Rate={self._p_corr_hz}Hz  '
            f'step={math.degrees(self._p_max_step):.1f}deg  '
            f'total_limit=flex:{math.degrees(self._p_max_total_flex):.1f}deg '
            f'roll:{math.degrees(self._p_max_total_roll):.1f}deg  '
            f'move_dur={self._p_move_dur}s')
        self.get_logger().info(
            'Disable: ros2 param set /wrist_balance_controller enabled false')
        if self._p_dry_run:
            self.get_logger().info('DRY RUN — no trajectories sent.')

    def _cmd_callback(self, msg: Vector3):
        with self._lock:
            self._latest_cmd    = msg
            self._last_cmd_time = time.monotonic()

    def _joint_state_callback(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                self._joint_positions[name] = pos

    def _balance_enabled_callback(self, msg: Bool):
        if msg.data == self._balance_active:
            return
        self._balance_active = msg.data
        self.get_logger().info(
            f'Balance {"ENABLED" if msg.data else "DISABLED"}')

        if msg.data:
            # Record starting pose
            with self._lock:
                self._start_flex = self._joint_positions.get(
                    self._p_flex_joint)
                self._start_roll = self._joint_positions.get(
                    self._p_roll_joint)
            self._total_flex = 0.0
            self._total_roll = 0.0
            self.get_logger().info(
                f'Start pose: flex={self._start_flex:.4f}  '
                f'roll={self._start_roll:.4f}')
        else:
            self.get_logger().info(
                f'Total displacement: '
                f'flex={math.degrees(self._total_flex):+.1f}deg  '
                f'roll={math.degrees(self._total_roll):+.1f}deg')

    def _control_loop(self):
        if not self._p_enabled or not self._balance_active:
            return

        with self._lock:
            cmd           = self._latest_cmd
            joint_pos     = dict(self._joint_positions)
            last_cmd_time = self._last_cmd_time

        if cmd is None or last_cmd_time is None:
            self.get_logger().warn(
                'No command yet', throttle_duration_sec=5.0)
            return
        if time.monotonic() - last_cmd_time > self._p_cmd_timeout:
            self.get_logger().warn(
                f'Command stale', throttle_duration_sec=5.0)
            return

        fj = self._p_flex_joint
        rj = self._p_roll_joint
        if fj not in joint_pos or rj not in joint_pos:
            self.get_logger().warn(
                'Joint positions unavailable', throttle_duration_sec=5.0)
            return

        # Desired step
        dt         = 1.0 / self._p_corr_hz
        flex_delta = float(cmd.x) * self._p_flex_scale * dt
        roll_delta = float(cmd.y) * self._p_roll_scale * dt

        # Update stall detection ratios BEFORE computing stall scale,
        # so this step's achievement informs this step's boost decision.
        # True achievement = how far servo moved since prev target was set.
        # Also publish /balance/achieved_delta so ball_balance_node can update
        # its ki_mode=servo integral and achievement EMA.
        if self._prev_target_flex is not None and abs(self._prev_cmd_flex) > 1e-4:
            true_achieved_f = joint_pos[fj] - (self._prev_target_flex - self._prev_cmd_flex)
            ratio_f = min(abs(true_achieved_f) / abs(self._prev_cmd_flex), 2.0)
            self._flex_ratios.append(ratio_f)
            if len(self._flex_ratios) > 20:
                self._flex_ratios.pop(0)
        else:
            true_achieved_f = 0.0
        if self._prev_target_roll is not None and abs(self._prev_cmd_roll) > 1e-4:
            true_achieved_r = joint_pos[rj] - (self._prev_target_roll - self._prev_cmd_roll)
            ratio_r = min(abs(true_achieved_r) / abs(self._prev_cmd_roll), 2.0)
            self._roll_ratios.append(ratio_r)
            if len(self._roll_ratios) > 20:
                self._roll_ratios.pop(0)
        else:
            true_achieved_r = 0.0

        # Publish achieved_delta — uses prev step's commanded delta and this
        # step's measured position to report what the servo actually delivered.
        if self._prev_cmd_flex is not None or self._prev_cmd_roll is not None:
            ach_msg = Vector3()
            ach_msg.x = float(true_achieved_f)
            ach_msg.y = float(true_achieved_r)
            self._pub_achieved_delta.publish(ach_msg)

        # Stall-adaptive step size clamping.
        # When the rolling average of achieved/commanded drops below
        # stall_threshold, scale up max_step_rad to give the servo a
        # larger positional error — generating more PWM/torque to
        # overcome gravity or mechanical resistance.  The scale factor
        # is capped at stall_scale_max to prevent violent lurching.
        # When the servo recovers (achievement improves), the scale
        # returns toward 1.0 automatically as the window fills.
        base_step    = self._p_max_step
        threshold    = self.get_parameter('stall_threshold').value
        exit_thresh  = self.get_parameter('stall_exit_threshold').value
        exit_steps   = self.get_parameter('stall_exit_steps').value
        scale_max    = self.get_parameter('stall_scale_max').value
        scale_en     = self.get_parameter('stall_scale_enable').value

        def stall_scale(axis):
            # axis: 'flex' or 'roll'
            stalled     = self._flex_stalled     if axis == 'flex' else self._roll_stalled
            good_streak = self._flex_good_streak if axis == 'flex' else self._roll_good_streak
            ratios      = self._flex_ratios      if axis == 'flex' else self._roll_ratios

            if not scale_en or not ratios:
                return 1.0
            last = ratios[-1]

            if not stalled:
                # Entry: one bad step is enough
                if last < threshold:
                    if axis == 'flex':
                        self._flex_stalled     = True
                        self._flex_good_streak = 0
                    else:
                        self._roll_stalled     = True
                        self._roll_good_streak = 0
                    stalled = True
            else:
                # Exit: require sustained recovery above exit_thresh
                if last >= exit_thresh:
                    if axis == 'flex':
                        self._flex_good_streak += 1
                        good_streak = self._flex_good_streak
                    else:
                        self._roll_good_streak += 1
                        good_streak = self._roll_good_streak
                    if good_streak >= exit_steps:
                        if axis == 'flex':
                            self._flex_stalled     = False
                            self._flex_good_streak = 0
                        else:
                            self._roll_stalled     = False
                            self._roll_good_streak = 0
                        stalled = False
                else:
                    # Bad step resets the good streak
                    if axis == 'flex':
                        self._flex_good_streak = 0
                    else:
                        self._roll_good_streak = 0

            if not stalled:
                return 1.0
            # Scale inversely with recent achievement, capped at scale_max
            avg = sum(ratios[-3:]) / len(ratios[-3:])
            return min(scale_max, threshold / max(avg, 0.01))

        flex_scale = stall_scale('flex')
        roll_scale = stall_scale('roll')
        eff_flex_step = base_step * flex_scale
        eff_roll_step = base_step * roll_scale

        if flex_scale > 1.0 or roll_scale > 1.0:
            self.get_logger().info(
                f'Stall scale active: '
                f'flex={flex_scale:.2f}× (stalled={self._flex_stalled} good={self._flex_good_streak})  '
                f'roll={roll_scale:.2f}× (stalled={self._roll_stalled} good={self._roll_good_streak})',
                throttle_duration_sec=2.0)

        flex_delta = max(-eff_flex_step, min(eff_flex_step, flex_delta))
        roll_delta = max(-eff_roll_step, min(eff_roll_step, roll_delta))

        if abs(flex_delta) < 1e-4 and abs(roll_delta) < 1e-4:
            return

        # Total displacement limit
        projected_flex = self._total_flex + flex_delta
        projected_roll = self._total_roll + roll_delta

        if abs(projected_flex) > self._p_max_total_flex:
            remaining = self._p_max_total_flex - abs(self._total_flex)
            flex_delta = math.copysign(max(0.0, remaining), flex_delta)
            self.get_logger().warn(
                f'Flex total limit reached '
                f'({math.degrees(self._total_flex):+.1f}deg '
                f'limit={math.degrees(self._p_max_total_flex):.1f}deg) — clamping',
                throttle_duration_sec=2.0)

        if abs(projected_roll) > self._p_max_total_roll:
            remaining = self._p_max_total_roll - abs(self._total_roll)
            roll_delta = math.copysign(max(0.0, remaining), roll_delta)
            self.get_logger().warn(
                f'Roll total limit reached '
                f'({math.degrees(self._total_roll):+.1f}deg '
                f'limit={math.degrees(self._p_max_total_roll):.1f}deg) — clamping',
                throttle_duration_sec=2.0)

        if abs(flex_delta) < 1e-4 and abs(roll_delta) < 1e-4:
            self.get_logger().warn(
                'Both axes at total limit', throttle_duration_sec=2.0)
            return

        # Target positions
        new_flex = max(self._p_flex_min,
                       min(self._p_flex_max,
                           joint_pos[fj] + flex_delta))
        new_roll = max(self._p_roll_min,
                       min(self._p_roll_max,
                           joint_pos[rj] + roll_delta))

        actual_flex_delta = new_flex - joint_pos[fj]
        actual_roll_delta = new_roll - joint_pos[rj]

        self._total_flex += actual_flex_delta
        self._total_roll += actual_roll_delta

        # Store current targets for next step's true achievement calculation.
        # Also publish /balance/cmd_delta so ball_balance_node knows what was
        # commanded this step (needed to compute undelivered delta next step).
        self._prev_target_flex = new_flex
        self._prev_target_roll = new_roll
        self._prev_cmd_flex    = actual_flex_delta
        self._prev_cmd_roll    = actual_roll_delta

        cmd_msg = Vector3()
        cmd_msg.x = float(actual_flex_delta)
        cmd_msg.y = float(actual_roll_delta)
        self._pub_cmd_delta.publish(cmd_msg)

        self.get_logger().info(
            f'CORR | '
            f'flex {joint_pos[fj]:.4f}→{new_flex:.4f} '
            f'(Δ{math.degrees(actual_flex_delta):+.2f}deg)  '
            f'roll {joint_pos[rj]:.4f}→{new_roll:.4f} '
            f'(Δ{math.degrees(actual_roll_delta):+.2f}deg)  '
            f'cumul: flex={math.degrees(self._total_flex):+.1f}deg '
            f'roll={math.degrees(self._total_roll):+.1f}deg')

        if self._p_dry_run:
            msg_out      = String()
            msg_out.data = (
                f'DRY flex {joint_pos[fj]:.4f}→{new_flex:.4f}  '
                f'roll {joint_pos[rj]:.4f}→{new_roll:.4f}')
            self._pub_intended.publish(msg_out)
            return

        # Publish trajectory (topic, non-blocking, replaces previous)
        traj              = JointTrajectory()
        traj.joint_names  = [fj, rj]
        traj.header.stamp = self.get_clock().now().to_msg()

        pt             = JointTrajectoryPoint()
        pt.positions   = [new_flex, new_roll]
        pt.velocities  = [0.0, 0.0]

        dur      = self._p_move_dur
        secs     = int(dur)
        nanosecs = int((dur - secs) * 1e9)
        pt.time_from_start = Duration(sec=secs, nanosec=nanosecs)

        traj.points = [pt]
        self._traj_pub.publish(traj)


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
