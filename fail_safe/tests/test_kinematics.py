"""Sanity checks on the fail-safe kinematics layer.

The math is the same as the parent's (integration is exact for arcs of
constant (v, ω)), but verifying here means the package is self-contained.
"""

from math import isclose, pi

from drawingrobot_failsafe.kinematics import Pose, step, transform_point


def test_straight_motion():
    pose = step(Pose(0.0, 0.0, 0.0), v_left=10.0, v_right=10.0, wheelbase=20.0, dt=1.0)
    assert isclose(pose.x, 10.0, abs_tol=1e-9)
    assert isclose(pose.y, 0.0, abs_tol=1e-9)
    assert isclose(pose.theta, 0.0, abs_tol=1e-9)


def test_in_place_rotation_keeps_midpoint_still():
    pose = step(Pose(0.0, 0.0, 0.0), v_left=-5.0, v_right=5.0, wheelbase=10.0, dt=1.0)
    # ω = (5 - -5) / 10 = 1 rad/s, midpoint stays at origin.
    assert isclose(pose.x, 0.0, abs_tol=1e-9)
    assert isclose(pose.y, 0.0, abs_tol=1e-9)
    assert isclose(pose.theta, 1.0, abs_tol=1e-9)


def test_full_circle_returns_to_start():
    # v_l + v_r != 0, omega != 0 — arc integration. One full revolution.
    wheelbase = 10.0
    omega = 1.0
    v_mid = 5.0
    v_l = v_mid - omega * wheelbase / 2  # 0
    v_r = v_mid + omega * wheelbase / 2  # 10
    pose = step(Pose(0.0, 0.0, 0.0), v_l, v_r, wheelbase, dt=2 * pi)
    assert isclose(pose.x, 0.0, abs_tol=1e-9)
    assert isclose(pose.y, 0.0, abs_tol=1e-9)
    # theta wraps to 2π, the integrator doesn't normalize — that's fine.


def test_transform_point_identity_at_origin():
    pose = Pose(0.0, 0.0, 0.0)
    x, y = transform_point(pose, 3.0, 4.0)
    assert isclose(x, 3.0) and isclose(y, 4.0)


def test_transform_point_rotated():
    pose = Pose(0.0, 0.0, pi / 2)
    # Body (0, +1) at heading +π/2 maps to world (-1, 0): y_body is "left",
    # rotated to point along world -x.
    x, y = transform_point(pose, 0.0, 1.0)
    assert isclose(x, -1.0, abs_tol=1e-12)
    assert isclose(y, 0.0, abs_tol=1e-12)
