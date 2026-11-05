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
    """

    def __init__(self, scale: float = 1.0):
        self.scale = scale              # flex_scale / roll_scale
        self.start: Optional[float] = None
        self.prev_target: Optional[float] = None
        self.prev_delta: Optional[float] = None

    def begin(self, position: Optional[float]):
        """Start a balance session at joint position 'position'."""
        self.start = position

    @property
    def has_step(self) -> bool:
        """True once a CORR step has been recorded."""
        return self.prev_delta is not None

    def achieved(self, position: float) -> float:
        """How far the joint has moved since the previous step was sent (rad)."""
        if self.prev_target is None or abs(self.prev_delta) <= 1e-4:
            return 0.0
        return position - (self.prev_target - self.prev_delta)

    def commanded(self) -> float:
        """The previous step's commanded delta (rad); 0.0 before the first step."""
        return self.prev_delta if self.prev_delta is not None else 0.0

    def record(self, target: float, position: float) -> float:
        """Record a step sent from 'position' to 'target'; returns the joint delta."""
        self.prev_target = target
        self.prev_delta = target - position
        return self.prev_delta


def ball_lost_duration(now: float, last_valid_ball_time: Optional[float],
                       pid_started_at: Optional[float]) -> Optional[float]:
    """Seconds the ball has been missing in the current attempt, or None if unknown."""
    if last_valid_ball_time is None:
        return None
    return now - last_valid_ball_time
