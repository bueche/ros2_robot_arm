"""Unit tests for ball_balance_node's control law (balance_logic, no ROS needed).

    python3 -m pytest robot_arm_control/test/test_pid_law.py

Gains are the SO-ARM101 balance tuning (config/balance/so_arm101.yaml).
The I term is the servo integral: undelivered wrist delta per CORR step.
Axis convention: flex is driven by the ball's y error, roll by its x error.
"""

import pytest

from robot_arm_control.balance_logic import (
    apply_deadband, crossed_center, error_sign, pid_axis, servo_integral_step)

KP = 0.15
KI_FLEX = 0.20
WINDUP_LIMIT = 0.5   # ki_windup_limit default
KD = 0.05
MAX_CMD = 0.3


# Control law: signs, clamp

def test_p_term_follows_the_error():
    # ball toward the robot (+y) -> +flex; ball right (+x) -> +roll
    assert pid_axis(+0.5, 0.0, 0.0, KP, KI_FLEX, KD, MAX_CMD).cmd == pytest.approx(+0.075)
    assert pid_axis(-0.5, 0.0, 0.0, KP, KI_FLEX, KD, MAX_CMD).cmd == pytest.approx(-0.075)


def test_d_term_opposes_the_wrist_rate():
    terms = pid_axis(0.0, 0.0, +0.2, KP, KI_FLEX, KD, MAX_CMD)
    assert terms.d == pytest.approx(-0.01)
    assert terms.cmd == pytest.approx(-0.01)


def test_i_term_adds_ki_times_integral():
    terms = pid_axis(0.0, 0.5, 0.0, KP, KI_FLEX, KD, MAX_CMD)
    assert terms.i == pytest.approx(0.1)
    assert terms.cmd == pytest.approx(0.1)


def test_command_is_clamped_to_max_cmd():
    high = pid_axis(+1.0, +2.0, -1.0, KP, KI_FLEX, KD, MAX_CMD)
    low = pid_axis(-1.0, -2.0, +1.0, KP, KI_FLEX, KD, MAX_CMD)
    assert high.cmd == MAX_CMD
    assert low.cmd == -MAX_CMD
    # the reported terms are the unclamped contributions
    assert high.p + high.i + high.d == pytest.approx(0.15 + 0.4 + 0.05)


def test_deadband_zeroes_small_errors_only():
    assert apply_deadband(0.02, 0.03) == 0.0
    assert apply_deadband(-0.02, 0.03) == 0.0
    assert apply_deadband(0.03, 0.03) == 0.03
    assert apply_deadband(-0.4, 0.03) == -0.4


# Servo integral: undelivered wrist delta per CORR step (command units)

def test_servo_integral_grows_with_the_command_when_the_servo_undershoots():
    # commanded +0.08, achieved +0.05 -> 0.03 undelivered, same sign as the command
    assert servo_integral_step(0.0, +0.08, +0.05, KI_FLEX, WINDUP_LIMIT) == pytest.approx(+0.03)
    assert servo_integral_step(0.0, -0.08, -0.05, KI_FLEX, WINDUP_LIMIT) == pytest.approx(-0.03)


def test_servo_integral_shrinks_when_the_servo_overshoots():
    assert servo_integral_step(0.10, +0.08, +0.10, KI_FLEX, WINDUP_LIMIT) == pytest.approx(0.08)


def test_servo_integral_accumulates_across_steps():
    i = 0.0
    for _ in range(5):  # a stalled servo: nothing delivered
        i = servo_integral_step(i, +0.08, 0.0, KI_FLEX, WINDUP_LIMIT)
    assert i == pytest.approx(0.40)
    assert KI_FLEX * i == pytest.approx(0.08)  # I term now doubles the next step


def test_servo_integral_clamped_so_i_term_stays_within_windup_limit():
    i = servo_integral_step(2.45, +0.08, 0.0, KI_FLEX, WINDUP_LIMIT)  # 2.53 before clamp
    assert i == pytest.approx(WINDUP_LIMIT / KI_FLEX)  # 2.5
    assert servo_integral_step(-2.45, -0.08, 0.0, KI_FLEX, WINDUP_LIMIT) == pytest.approx(-2.5)


def test_zero_ki_leaves_servo_integral_unchanged():
    assert servo_integral_step(0.3, +0.08, 0.0, 0.0, WINDUP_LIMIT) == 0.3


# Sign-change decay trigger

@pytest.mark.parametrize('prev, curr, expected', [
    (+1, -1, True), (-1, +1, True),
    (+1, +1, False), (+1, 0, False), (0, -1, False), (None, -1, False),
])
def test_crossed_center(prev, curr, expected):
    assert crossed_center(prev, curr) is expected


def test_error_sign():
    assert (error_sign(0.3), error_sign(-0.3), error_sign(0.0)) == (1, -1, 0)
