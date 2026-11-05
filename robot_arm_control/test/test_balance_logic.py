"""Unit tests for robot_arm_control.balance_logic (no ROS needed).

Run from the robot_arm_control package directory:
    python3 -m pytest test/test_balance_logic.py
or with the rest of the package tests:
    colcon test --packages-select robot_arm_control

Positions in the scenario tests are taken from the SO-ARM101 run of
2026-11-04 (test.results.110426.soarm101.t1.txt).
"""

import pytest

from robot_arm_control.balance_logic import WristAxis, ball_lost_duration


# WristAxis: basic step bookkeeping

def test_achieved_is_zero_before_first_step():
    axis = WristAxis()
    axis.begin(1.0)
    assert not axis.has_step
    assert axis.achieved(1.2) == 0.0
    assert axis.commanded() == 0.0


def test_achieved_measures_motion_since_step_was_sent():
    axis = WristAxis()
    axis.begin(1.0)
    axis.record(target=1.08, position=1.0)
    assert axis.commanded() == pytest.approx(0.08)
    assert axis.achieved(1.05) == pytest.approx(0.05)


def test_hold_step_reports_no_achievement():
    axis = WristAxis()
    axis.begin(1.0)
    axis.record(target=1.00005, position=1.0)  # below 1e-4: treated as a hold
    assert axis.achieved(1.01) == 0.0


# Bug: a new balance session inherited the previous session's last step.
# Pose 10 ended with a step -1.6414 -> -1.6581; the arm then moved to pose 11
# and balancing restarted at -1.0753. The first "achieved" value included the
# whole pose change (about +0.57 rad), which wound ball_balance_node's servo
# integral to -0.646 in one step and reversed the flex correction.

def test_new_session_does_not_inherit_previous_step():
    axis = WristAxis(scale=-1.0)
    axis.begin(-1.1996)
    axis.record(target=-1.6581, position=-1.6414)

    axis.begin(-1.0753)  # pose 11, balance re-enabled
    assert axis.start == pytest.approx(-1.0753)
    assert not axis.has_step
    assert axis.achieved(-1.0753) == 0.0
    assert axis.commanded() == 0.0


# Bug: with flex_scale = -1 the wrist controller reported cmd_delta and
# achieved_delta in joint units, while ball_balance_node integrates their
# difference in command units. An undershooting servo then drove the servo
# integral against the command instead of with it.

def test_deltas_are_in_command_units_with_negative_scale():
    axis = WristAxis(scale=-1.0)
    axis.begin(-1.0)
    # Command +0.08 rad -> joint target -1.08; the servo only reaches -1.05.
    axis.record(target=-1.08, position=-1.0)
    assert axis.commanded() == pytest.approx(+0.08)
    assert axis.achieved(-1.05) == pytest.approx(+0.05)
    undelivered = axis.commanded() - axis.achieved(-1.05)
    assert undelivered == pytest.approx(+0.03)  # same sign as the command


def test_deltas_unchanged_with_positive_scale():
    axis = WristAxis(scale=1.0)
    axis.begin(1.0)
    axis.record(target=1.08, position=1.0)
    assert axis.commanded() == pytest.approx(0.08)
    assert axis.achieved(1.05) == pytest.approx(0.05)


def test_zero_scale_reports_no_motion():
    axis = WristAxis(scale=0.0)
    axis.begin(1.0)
    axis.record(target=1.0, position=1.0)
    assert axis.commanded() == 0.0
    assert axis.achieved(1.0) == 0.0


# ball_lost_duration

def test_ball_lost_duration_from_last_detection():
    assert ball_lost_duration(now=10.0, last_valid_ball_time=8.5,
                              pid_started_at=5.0) == pytest.approx(1.5)


def test_no_attempt_and_no_detection_is_unknown():
    assert ball_lost_duration(now=10.0, last_valid_ball_time=None,
                              pid_started_at=None) is None


# Bug: on pose 10 the detector never found the ball. ball_balance_node clears
# last_valid_ball_time on MOVING, so with no detection after the move the
# ball-lost check never fired and the PID ran for 19 s on the previous pose's
# ball position. The ball has been missing since the attempt started.

def test_ball_never_seen_in_attempt_counts_from_attempt_start():
    assert ball_lost_duration(now=10.0, last_valid_ball_time=None,
                              pid_started_at=7.0) == pytest.approx(3.0)
