from math import isclose, pi

import pytest

from drawingrobot.kinematics import Pose, step, transform_point


def test_straight_line_advances_along_heading():
    pose = Pose(0.0, 0.0, 0.0)
    new = step(pose, v_left=2.0, v_right=2.0, wheelbase=1.0, dt=1.5)
    assert isclose(new.x, 3.0, abs_tol=1e-9)
    assert isclose(new.y, 0.0, abs_tol=1e-9)
    assert isclose(new.theta, 0.0, abs_tol=1e-9)


def test_straight_line_respects_heading():
    pose = Pose(1.0, 2.0, pi / 2)
    new = step(pose, v_left=1.0, v_right=1.0, wheelbase=1.0, dt=1.0)
    assert isclose(new.x, 1.0, abs_tol=1e-9)
    assert isclose(new.y, 3.0, abs_tol=1e-9)
    assert isclose(new.theta, pi / 2, abs_tol=1e-9)


def test_in_place_rotation_keeps_position():
    pose = Pose(5.0, -3.0, 0.0)
    new = step(pose, v_left=-1.0, v_right=1.0, wheelbase=1.0, dt=pi)
    assert isclose(new.x, 5.0, abs_tol=1e-9)
    assert isclose(new.y, -3.0, abs_tol=1e-9)
    assert isclose(new.theta, 2 * pi, abs_tol=1e-9)


def test_arc_returns_to_start_after_full_circle():
    pose = Pose(0.0, 0.0, 0.0)
    v_left, v_right, wheelbase = 1.0, 2.0, 1.0
    omega = (v_right - v_left) / wheelbase
    dt = 2 * pi / omega
    new = step(pose, v_left, v_right, wheelbase, dt)
    assert isclose(new.x, 0.0, abs_tol=1e-9)
    assert isclose(new.y, 0.0, abs_tol=1e-9)
    assert isclose(new.theta % (2 * pi), 0.0, abs_tol=1e-9)


def test_left_turn_is_ccw():
    pose = Pose(0.0, 0.0, 0.0)
    new = step(pose, v_left=0.5, v_right=1.5, wheelbase=1.0, dt=0.1)
    assert new.theta > 0


def test_invalid_wheelbase_raises():
    with pytest.raises(ValueError):
        step(Pose(0.0, 0.0, 0.0), 1.0, 1.0, 0.0, 1.0)


def test_transform_point_identity_at_origin():
    x, y = transform_point(Pose(0.0, 0.0, 0.0), 3.0, 4.0)
    assert isclose(x, 3.0)
    assert isclose(y, 4.0)


def test_transform_point_rotates_then_translates():
    x, y = transform_point(Pose(1.0, 2.0, pi / 2), 1.0, 0.0)
    assert isclose(x, 1.0, abs_tol=1e-9)
    assert isclose(y, 3.0, abs_tol=1e-9)
