from math import isclose, pi, radians

from drawingrobot.correction import plan_correction
from drawingrobot.kinematics import Pose, step
from drawingrobot.limits import NO_LIMITS


WHEELBASE = 20.0
LIN_SPEED = 12.0
ANG_SPEED = radians(180.0)


def integrate(actual: Pose, cmds, wheelbase: float = WHEELBASE,
              dt: float = 0.001) -> Pose:
    pose = Pose(actual.x, actual.y, actual.theta)
    for c in cmds:
        remaining = c.duration
        while remaining > 0:
            sub = min(dt, remaining)
            pose = step(pose, c.v_left, c.v_right, wheelbase, sub)
            remaining -= sub
    return pose


def test_zero_error_returns_empty():
    p = Pose(5.0, 3.0, 0.4)
    assert plan_correction(
        p, p,
        linear_speed_cm_s=LIN_SPEED, angular_speed_rad_s=ANG_SPEED,
        wheelbase_cm=WHEELBASE, limits=NO_LIMITS,
        pos_threshold_cm=1.0, heading_threshold_rad=radians(5.0),
    ) == []


def test_below_threshold_returns_empty():
    actual = Pose(0.0, 0.0, 0.0)
    expected = Pose(0.5, 0.0, radians(2.0))
    assert plan_correction(
        actual, expected,
        linear_speed_cm_s=LIN_SPEED, angular_speed_rad_s=ANG_SPEED,
        wheelbase_cm=WHEELBASE, limits=NO_LIMITS,
        pos_threshold_cm=1.0, heading_threshold_rad=radians(5.0),
    ) == []


def test_pure_heading_error_emits_single_rotate():
    actual = Pose(0.0, 0.0, 0.0)
    expected = Pose(0.0, 0.0, radians(30.0))
    cmds = plan_correction(
        actual, expected,
        linear_speed_cm_s=LIN_SPEED, angular_speed_rad_s=ANG_SPEED,
        wheelbase_cm=WHEELBASE, limits=NO_LIMITS,
        pos_threshold_cm=1.0, heading_threshold_rad=radians(5.0),
    )
    assert len(cmds) == 1
    # In-place rotation has v_left = -v_right.
    assert isclose(cmds[0].v_left, -cmds[0].v_right, abs_tol=1e-9)
    final = integrate(actual, cmds)
    assert isclose(final.theta, expected.theta, abs_tol=1e-6)


def test_mixed_error_round_trip():
    actual = Pose(0.0, 0.0, 0.0)
    expected = Pose(5.0, 3.0, radians(45.0))
    cmds = plan_correction(
        actual, expected,
        linear_speed_cm_s=LIN_SPEED, angular_speed_rad_s=ANG_SPEED,
        wheelbase_cm=WHEELBASE, limits=NO_LIMITS,
        pos_threshold_cm=1.0, heading_threshold_rad=radians(5.0),
    )
    assert cmds
    final = integrate(actual, cmds)
    assert isclose(final.x, expected.x, abs_tol=1e-3)
    assert isclose(final.y, expected.y, abs_tol=1e-3)
    assert isclose(final.theta, expected.theta, abs_tol=1e-3)


def test_backward_correction_is_used_when_shorter():
    # actual heading π faces -x. expected is "behind" along that heading
    # (toward +x). Backward translation should beat 180° rotate + forward.
    actual = Pose(0.0, 0.0, pi)
    expected = Pose(5.0, 0.0, pi)
    cmds = plan_correction(
        actual, expected,
        linear_speed_cm_s=LIN_SPEED, angular_speed_rad_s=ANG_SPEED,
        wheelbase_cm=WHEELBASE, limits=NO_LIMITS,
        pos_threshold_cm=1.0, heading_threshold_rad=radians(5.0),
    )
    # Should be a single backward move_straight (no rotation needed).
    assert len(cmds) == 1
    assert cmds[0].v_left < 0 and cmds[0].v_right < 0
    final = integrate(actual, cmds)
    assert isclose(final.x, expected.x, abs_tol=1e-3)
    assert isclose(final.y, expected.y, abs_tol=1e-3)
    assert isclose(final.theta, expected.theta, abs_tol=1e-3)
