#!/usr/bin/env python3
"""
joint_nudge.py — Move one or more joints while holding all others at current position.

Reads current joint positions from /joint_states, then sends a two-point
trajectory to /koch_v11_controller/joint_trajectory:
  - Point 0: all joints at last commanded positions (anchor)
  - Point 1: target joints at new positions, rest held at last commanded

Using last COMMANDED (not measured) positions for the hold joints prevents
drooping — gravity-loaded joints are typically slightly below their commanded
position, so re-commanding the measured position would gradually lower them.

Usage — single joint by delta:
  ros2 run writing_robot_control joint_nudge \
    --ros-args -p joint:=wrist_flex -p delta:=0.05

Usage — single joint to absolute position:
  ros2 run writing_robot_control joint_nudge \
    --ros-args -p joint:=wrist_flex -p position:=2.5

Usage — multiple joints (comma-separated, matched lists):
  ros2 run writing_robot_control joint_nudge \
    --ros-args -p joints:=wrist_flex,wrist_roll -p deltas:=0.05,-0.02

Usage — interactive mode (nudge repeatedly without restarting):
  ros2 run writing_robot_control joint_nudge \
    --ros-args -p interactive:=true -p joint:=wrist_flex -p step:=0.01

Parameters:
  joint         Single joint name                               default: ''
  joints        Comma-separated joint names                     default: ''
  position      Absolute target position (rad) — single joint   default: nan
  positions     Comma-separated absolute targets                default: ''
  delta         Relative change (rad) — single joint            default: 0.0
  deltas        Comma-separated relative changes                default: ''
  duration      Trajectory duration (sec)                       default: 1.5
  step          Per-keypress step size for interactive mode     default: 0.01
  interactive   Interactive nudge mode                          default: False
  read_timeout  Seconds to wait for /joint_states               default: 5.0
  topic         Trajectory topic
                  default: /koch_v11_controller/joint_trajectory

Joint names (Koch v1.1):
  shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, pen_holder

Examples:
  # Nudge wrist_flex up by 0.05 rad
  ros2 run writing_robot_control joint_nudge --ros-args -p joint:=wrist_flex -p delta:=0.05

  # Move wrist_flex to absolute 2.5 rad
  ros2 run writing_robot_control joint_nudge --ros-args -p joint:=wrist_flex -p position:=2.5

  # Move two joints simultaneously
  ros2 run writing_robot_control joint_nudge \
    --ros-args -p joints:=wrist_flex,wrist_roll -p deltas:=0.05,-0.02

  # Interactive mode — press +/- to nudge, q to quit
  ros2 run writing_robot_control joint_nudge \
    --ros-args -p interactive:=true -p joint:=wrist_flex -p step:=0.01
"""

import rclpy
import rclpy.executors
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

import math
import sys
import time
import threading


# Koch v1.1 joint names in controller order
JOINT_NAMES = [
    'shoulder_pan',
    'shoulder_lift',
    'elbow_flex',
    'wrist_flex',
    'wrist_roll',
    'pen_holder',
]

# Joint limits (rad) — derived from URDF <limit lower= upper= > tags.
# All Koch v1.1 joints have inverted lower/upper conventions (lower > upper
# in several cases) so we store true (min, max) as min(lower,upper) / max(lower,upper).
#
# Source: koch_v11_arm_real.urdf
#   shoulder_pan:  lower=2.044  upper=0.540   → range [0.540, 2.044]
#   shoulder_lift: lower=2.8857 upper=2.2000  → range [2.200, 2.886]
#   elbow_flex:    lower=0.726  upper=2.250   → range [0.726, 2.250]
#   wrist_flex:    lower=2.70   upper=0.297   → range [0.297, 2.700]
#   wrist_roll:    lower=1.900  upper=-1.448  → range [-1.448, 1.900]
#   pen_holder:    lower=1.60   upper=0.19    → range [0.190, 1.600]
#
# Physical notes (from URDF comments):
#   shoulder_pan:  2.044=far left,   0.540=far right
#   shoulder_lift: 2.886=highest,    2.200=lowest   (INVERTED: higher rad = lower physical)
#   elbow_flex:    0.726=highest,    2.250=lowest   (INVERTED: higher rad = lower physical)
#   wrist_flex:    2.70=highest,     0.297=lowest   (INVERTED: higher rad = lower physical)
#   wrist_roll:    1.900=jacks same side, -1.448=jacks opposite (~180 deg)
#   pen_holder:    1.60=closed,      0.19=open      (INVERTED: higher rad = more closed)
JOINT_LIMITS = {
    'shoulder_pan':  (0.540,  2.044),
    'shoulder_lift': (2.200,  2.886),
    'elbow_flex':    (0.726,  2.250),
    'wrist_flex':    (0.297,  2.700),
    'wrist_roll':    (-1.448, 1.900),
    'pen_holder':    (0.190,  1.600),
}


class JointNudge(Node):

    def __init__(self):
        super().__init__('joint_nudge')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('joint',        '')
        self.declare_parameter('joints',       '')
        self.declare_parameter('position',     float('nan'))
        self.declare_parameter('positions',    '')
        self.declare_parameter('delta',        0.0)
        self.declare_parameter('deltas',       '')
        self.declare_parameter('duration',     1.5)
        self.declare_parameter('step',         0.01)
        self.declare_parameter('interactive',  False)
        self.declare_parameter('read_timeout', 5.0)
        self.declare_parameter('topic',
            '/koch_v11_controller/joint_trajectory')

        self._duration     = self.get_parameter('duration').value
        self._step         = self.get_parameter('step').value
        self._interactive  = self.get_parameter('interactive').value
        self._read_timeout = self.get_parameter('read_timeout').value
        self._topic        = self.get_parameter('topic').value

        # ── Publisher & subscriber ───────────────────────────────────────
        self._pub = self.create_publisher(
            JointTrajectory, self._topic, 10)

        self._current_positions = None   # latest measured from /joint_states
        self._last_commanded    = {}     # last position WE commanded per joint
        self._js_lock = threading.Lock()
        self._js_sub = self.create_subscription(
            JointState, '/joint_states',
            self._js_callback, 10)

    # ────────────────────────────────────────────────────────────────────
    def _js_callback(self, msg: JointState):
        pos = {}
        for name, p in zip(msg.name, msg.position):
            pos[name] = p
        with self._js_lock:
            self._current_positions = pos

    def _wait_for_joint_states(self):
        """Block until /joint_states arrives or timeout."""
        self.get_logger().info('Reading current joint positions...')
        start = time.time()
        while time.time() - start < self._read_timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            with self._js_lock:
                if self._current_positions is not None:
                    return True
        self.get_logger().error(
            f'No /joint_states received within {self._read_timeout:.1f}s. '
            f'Is the robot running?')
        return False

    def _get_current(self):
        with self._js_lock:
            return dict(self._current_positions) if self._current_positions else {}

    # ────────────────────────────────────────────────────────────────────
    def _validate(self, joint, target):
        """Check target is within joint limits. Returns (ok, clamped_value)."""
        if joint not in JOINT_LIMITS:
            self.get_logger().warn(f'No limits known for {joint} — sending anyway.')
            return True, target
        lo, hi = JOINT_LIMITS[joint]
        if target < lo or target > hi:
            clamped = max(lo, min(hi, target))
            self.get_logger().warn(
                f'{joint}: target {target:.4f} rad outside limits '
                f'[{lo:.3f}, {hi:.3f}] — clamped to {clamped:.4f} rad'
            )
            return False, clamped
        return True, target

    # ────────────────────────────────────────────────────────────────────
    def _send(self, targets: dict, duration: float = None):
        """
        Send a two-point trajectory.
        targets: {joint_name: target_position_rad}

        Hold strategy for unspecified joints:
          - Use last commanded position for anchor AND hold target.
            This prevents drooping from re-commanding measured positions
            that are already slightly below commanded due to gravity load.
          - Falls back to measured position only on first call before any
            command has been issued.
        """
        if duration is None:
            duration = self._duration

        measured = self._get_current()
        if not measured:
            self.get_logger().error('No current joint positions available.')
            return False

        # Validate and clamp targets
        validated = {}
        for joint, tgt in targets.items():
            if joint not in JOINT_NAMES:
                self.get_logger().error(f'Unknown joint: {joint}')
                return False
            _, clamped = self._validate(joint, tgt)
            validated[joint] = clamped

        # Build position lists.
        # For hold joints: use last commanded if available, else measured.
        # For target joints: use validated value.
        with self._js_lock:
            last_cmd = dict(self._last_commanded)

        anchor_positions = []
        target_positions = []
        for name in JOINT_NAMES:
            hold = last_cmd.get(name, measured.get(name, 0.0))
            anchor_positions.append(hold)
            target_positions.append(validated.get(name, hold))

        # Point 0: anchor at last commanded (t=0)
        anchor = JointTrajectoryPoint()
        anchor.positions  = anchor_positions
        anchor.velocities = [0.0] * len(JOINT_NAMES)
        anchor.time_from_start = Duration(sec=0, nanosec=0)

        # Point 1: target
        secs     = int(duration)
        nanosecs = int((duration - secs) * 1e9)
        target = JointTrajectoryPoint()
        target.positions  = target_positions
        target.velocities = [0.0] * len(JOINT_NAMES)
        target.time_from_start = Duration(sec=secs, nanosec=nanosecs)

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names  = JOINT_NAMES
        traj.points       = [anchor, target]

        self._pub.publish(traj)

        # Track what we commanded
        with self._js_lock:
            for name, pos in zip(JOINT_NAMES, target_positions):
                self._last_commanded[name] = pos

        # Log what changed
        for joint, tgt in validated.items():
            old = measured.get(joint, 0.0)
            delta = tgt - old
            self.get_logger().info(
                f'  {joint}: {old:.4f} → {tgt:.4f} rad  '
                f'(delta {delta:+.4f} rad = {math.degrees(delta):+.2f} deg)'
            )
        return True

    # ────────────────────────────────────────────────────────────────────
    def _send_and_report(self, targets: dict, duration: float = None,
                         interactive: bool = False):
        """
        Send a nudge, wait for the trajectory to complete, then print a
        settled-position report showing commanded vs actual for the
        nudged joints.

        In interactive mode prints directly to stdout (no ROS logger noise).
        In one-shot mode uses the ROS logger.

        Report format:
          wrist_flex   cmd: 2.3139  meas: 2.3092  error: -0.0047 rad (-0.27 deg)  [droop]
        """
        if duration is None:
            duration = self._duration

        ok = self._send(targets, duration=duration)
        if not ok:
            return False

        # Wait for the trajectory to finish, spinning so callbacks keep firing
        settle_time = duration + 0.15   # small margin after trajectory end
        deadline = time.time() + settle_time
        while time.time() < deadline:
            time.sleep(0.05)            # executor thread handles spinning

        # Read settled positions
        settled = self._get_current()
        with self._js_lock:
            cmd = dict(self._last_commanded)

        lines = []
        for joint in targets:
            c = cmd.get(joint, float('nan'))
            m = settled.get(joint, float('nan'))
            err = m - c
            err_deg = math.degrees(err)
            droop_label = ''
            if err < -0.003:
                droop_label = '  [droop]'
            elif err > 0.003:
                droop_label = '  [overshoot]'
            lines.append(
                f'  {joint:<16} '
                f'cmd: {c:.4f}  '
                f'meas: {m:.4f}  '
                f'error: {err:+.4f} rad ({err_deg:+.2f} deg)'
                f'{droop_label}'
            )

        report = '\n'.join(lines)
        if interactive:
            sys.stdout.write('\r' + report + '\n')
            sys.stdout.flush()
        else:
            for line in lines:
                self.get_logger().info(line)

        return True

    # ────────────────────────────────────────────────────────────────────
    def run_once(self):
        """Parse parameters, send one nudge, exit."""
        if not self._wait_for_joint_states():
            return False

        # Seed _last_commanded from measured before first command
        measured = self._get_current()
        with self._js_lock:
            for name in JOINT_NAMES:
                if name in measured:
                    self._last_commanded[name] = measured[name]

        current = self._get_current()
        targets = {}

        joint_param    = self.get_parameter('joint').value
        position_param = self.get_parameter('position').value
        delta_param    = self.get_parameter('delta').value
        joints_param   = self.get_parameter('joints').value
        positions_param= self.get_parameter('positions').value
        deltas_param   = self.get_parameter('deltas').value

        if joints_param:
            names = [j.strip() for j in joints_param.split(',') if j.strip()]
            if deltas_param:
                deltas = [float(d.strip()) for d in deltas_param.split(',')]
                if len(deltas) != len(names):
                    self.get_logger().error(
                        f'joints ({len(names)}) and deltas ({len(deltas)}) '
                        f'must have the same count.')
                    return False
                for name, d in zip(names, deltas):
                    targets[name] = current.get(name, 0.0) + d
            elif positions_param:
                positions = [float(p.strip()) for p in positions_param.split(',')]
                if len(positions) != len(names):
                    self.get_logger().error(
                        f'joints ({len(names)}) and positions ({len(positions)}) '
                        f'must have the same count.')
                    return False
                for name, p in zip(names, positions):
                    targets[name] = p
            else:
                self.get_logger().error(
                    'With joints:= you must also provide deltas:= or positions:=')
                return False

        elif joint_param:
            if not math.isnan(position_param):
                targets[joint_param] = position_param
            else:
                targets[joint_param] = current.get(joint_param, 0.0) + delta_param
        else:
            self.get_logger().error(
                'Must specify joint:= or joints:=\n'
                'Examples:\n'
                '  -p joint:=wrist_flex -p delta:=0.05\n'
                '  -p joint:=wrist_flex -p position:=2.5\n'
                '  -p joints:=wrist_flex,wrist_roll -p deltas:=0.05,-0.02'
            )
            return False

        self.get_logger().info(f'Sending nudge (duration={self._duration:.2f}s):')
        return self._send_and_report(targets, interactive=False)

    # ────────────────────────────────────────────────────────────────────
    def run_interactive(self):
        """
        Interactive mode: repeatedly nudge a single joint with keyboard input.

        Seeds _last_commanded from measured positions at startup so the first
        command anchors correctly. Uses a dedicated executor thread for safe
        spinning alongside blocking keyboard input on the main thread.

        Keys:
          + or = or ]   nudge positive by step
          - or [        nudge negative by step
          1-9           set step multiplier (e.g. 5 -> step x5)
          0             reset multiplier to 1
          p             print positions (measured + last commanded)
          q or Ctrl-C   quit
        """
        joint = self.get_parameter('joint').value
        if not joint:
            self.get_logger().error('Interactive mode requires -p joint:=<name>')
            return

        if not self._wait_for_joint_states():
            return

        # Seed _last_commanded from current measured positions
        measured = self._get_current()
        with self._js_lock:
            for name in JOINT_NAMES:
                if name in measured:
                    self._last_commanded[name] = measured[name]
        self.get_logger().info('Seeded commanded positions from current measurements.')

        # Spin in a dedicated executor thread
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(self)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        import tty
        import termios

        step = self._step
        multiplier = 1

        print(f'\n{"="*55}')
        print(f'INTERACTIVE NUDGE — joint: {joint}  step: {step:.4f} rad')
        print(f'{"="*55}')
        print(f'  +/=/]    nudge +{step:.4f} rad')
        print(f'  -/[      nudge -{step:.4f} rad')
        print(f'  1-9      set step multiplier (e.g. 5 -> step x5)')
        print(f'  0        reset multiplier to 1')
        print(f'  p        print positions (measured | commanded)')
        print(f'  q        quit')
        print(f'{"="*55}\n')

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)

        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)

                if ch in ('q', 'Q', '\x03'):
                    break

                elif ch in ('+', '=', ']'):
                    d = step * multiplier
                    with self._js_lock:
                        cur = self._last_commanded.get(
                            joint, self._get_current().get(joint, 0.0))
                    self._send_and_report(
                        {joint: cur + d},
                        duration=max(0.5, self._duration),
                        interactive=True)

                elif ch in ('-', '['):
                    d = step * multiplier
                    with self._js_lock:
                        cur = self._last_commanded.get(
                            joint, self._get_current().get(joint, 0.0))
                    self._send_and_report(
                        {joint: cur - d},
                        duration=max(0.5, self._duration),
                        interactive=True)

                elif ch.isdigit():
                    multiplier = int(ch) if int(ch) > 0 else 1
                    sys.stdout.write(
                        f'\r  Step multiplier: {multiplier}  '
                        f'(effective step: {step * multiplier:.4f} rad)\n'
                    )
                    sys.stdout.flush()

                elif ch in ('p', 'P'):
                    cur = self._get_current()
                    with self._js_lock:
                        cmd = dict(self._last_commanded)
                    sys.stdout.write('\r  Joint positions (measured | last commanded):\n')
                    for name in JOINT_NAMES:
                        m = cur.get(name, float('nan'))
                        c = cmd.get(name, float('nan'))
                        marker = ' <--' if name == joint else ''
                        sys.stdout.write(
                            f'    {name:<16} '
                            f'meas: {m:.4f} rad  '
                            f'cmd:  {c:.4f} rad'
                            f'{marker}\n'
                        )
                    sys.stdout.flush()

        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            executor.shutdown(wait=False)
            print('\nInteractive nudge exited.')


# =========================================================================
def main(args=None):
    rclpy.init(args=args)
    node = JointNudge()

    try:
        if node._interactive:
            node.run_interactive()
        else:
            node.run_once()
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
