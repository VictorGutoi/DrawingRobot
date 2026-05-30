from math import isclose, pi, radians

from drawingrobot_failsafe.correction import plan_correction
from drawingrobot_failsafe.kinematics import Pose, step
from drawingrobot_failsafe.limits import NO_LIMITS


WHEELBASE = 20.0
LIN_SPEED = 12.0
ANG_SPEED = radians(180.0)
POS_THR = 1.0
HEAD_THR = radians(5.0)


def _plan(actual, expected):
    return plan_correction(
        actual, expected,
        linear_speed_cm_s=LIN_SPEED, angular_speed_rad_s=ANG_SPEED,
        wheelbase_cm=WHEELBASE, limits=NO_LIMITS,
        pos_threshold_cm=POS_THR, heading_threshold_rad=HEAD_THR,
    )


def _integrate(actual, cmds, dt=0.001):
    pose = Pose(actual.x, actual.y, actual.theta)
    for c in cmds:
        remaining = c.duration
        while remaining > 0:
            sub = min(dt, remaining)
            pose = step(pose, c.v_left, c.v_right, WHEELBASE, sub)
            remaining -= sub
    return pose


def test_zero_error_returns_empty():
    p = Pose(5.0, 3.0, 0.4)
    assert _plan(p, p) == []


def test_below_threshold_returns_empty():
    assert _plan(Pose(0.0, 0.0, 0.0), Pose(0.5, 0.0, radians(2.0))) == []


def test_pure_heading_error_emits_single_rotate():
    actual = Pose(0.0, 0.0, 0.0)
    expected = Pose(0.0, 0.0, radians(30.0))
    cmds = _plan(actual, expected)
    assert len(cmds) == 1
    # Rotation about the midpoint: v_left = -v_right.
    assert isclose(cmds[0].v_left, -cmds[0].v_right, abs_tol=1e-9)
    final = _integrate(actual, cmds)
    assert isclose(final.theta, expected.theta, abs_tol=1e-6)


def test_mixed_error_round_trip():
    actual = Pose(0.0, 0.0, 0.0)
    expected = Pose(5.0, 3.0, radians(45.0))
    cmds = _plan(actual, expected)
    assert cmds
    final = _integrate(actual, cmds)
    assert isclose(final.x, expected.x, abs_tol=1e-3)
    assert isclose(final.y, expected.y, abs_tol=1e-3)
    assert isclose(final.theta, expected.theta, abs_tol=1e-3)


def test_backward_translation_used_when_shorter():
    # Facing -x (theta=pi); target is "behind" along that heading.
    actual = Pose(0.0, 0.0, pi)
    expected = Pose(5.0, 0.0, pi)
    cmds = _plan(actual, expected)
    assert len(cmds) == 1
    assert cmds[0].v_left < 0 and cmds[0].v_right < 0   # backward straight
    final = _integrate(actual, cmds)
    assert isclose(final.x, expected.x, abs_tol=1e-3)
    assert isclose(final.y, expected.y, abs_tol=1e-3)
    assert isclose(final.theta, expected.theta, abs_tol=1e-3)
