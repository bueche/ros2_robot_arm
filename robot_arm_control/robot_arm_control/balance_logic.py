"""ROS-free pieces of the ball-balancing nodes, kept here so they can be unit tested.

Nothing in this module imports rclpy or message types; the nodes
(wrist_balance_controller, ball_balance_node) call into it.
"""

from typing import Optional


class WristAxis:
    """Per-axis bookkeeping for wrist_balance_controller between CORR steps.

    'start' is the joint position when balancing was enabled. 'prev_target' and
    'prev_delta' describe the previous CORR step (joint units, rad): its target
    and how far that target was from the joint when it was sent.

    achieved() and commanded() report in command units (joint delta / scale),
    the units of ball_balance_node's commands, so that its servo integral of
    commanded - achieved has the sign of the command whatever the scale.
    """

    def __init__(self, scale: float = 1.0):
        self.scale = scale              # flex_scale / roll_scale
        self.start: Optional[float] = None
        self.prev_target: Optional[float] = None
        self.prev_delta: Optional[float] = None

    def begin(self, position: Optional[float]):
        """Start a balance session at joint position 'position'.
        The pose_test  executes a step pose sequence. Between each pose the 
        pid controller kicks in to balance the ball. Each step is assumed to be 
        independent of the last step and balance attempt. It starts fresh at every
        new step. 

        The begin method forgets the previous session's last step: the arm has usually moved
        to a new pose since, so measuring against that step would report the
        whole pose change as 'achieved'.
        """
        self.start = position
        self.prev_target = None
        self.prev_delta = None

    @property
    def has_step(self) -> bool:
        """True once a CORR step has been recorded."""
        return self.prev_delta is not None

    def _to_command(self, joint_delta: float) -> float:
        return joint_delta / self.scale if self.scale != 0.0 else 0.0

    def achieved(self, position: float) -> float:
        """How far the joint has moved since the previous step was sent (command units)."""
        if self.prev_target is None or abs(self.prev_delta) <= 1e-4:
            return 0.0
        return self._to_command(position - (self.prev_target - self.prev_delta))

    def commanded(self) -> float:
        """The previous step's delta (command units); 0.0 before the first step."""
        return self._to_command(self.prev_delta) if self.prev_delta is not None else 0.0

    def record(self, target: float, position: float) -> float:
        """Record a step sent from 'position' to 'target'; returns the joint delta."""
        self.prev_target = target
        self.prev_delta = target - position
        return self.prev_delta


def ball_lost_duration(now: float, last_valid_ball_time: Optional[float],
                       pid_started_at: Optional[float]) -> Optional[float]:
    """Seconds the ball has been missing in the current attempt, or None if unknown.

    The PID controller depends on the vision model. Sometimes the ball might not
    be detected (due to lighting, occlusion, etc.). There is logic to try to jiggle
    the ball so that it reappears but it could be that the ball is lost all the way 
    to the end of the attempt period. 
    
    This function returns the duration of time since 
    the ball was last detected. Measured from the last valid detection, or from 
    the start of the attempt if the ball has not been detected since (last_valid_ball_time 
    is cleared on every arm move, so a ball never found after a move still times out).
    """
    ref = last_valid_ball_time if last_valid_ball_time is not None else pid_started_at
    if ref is None:
        return None
    return now - ref
