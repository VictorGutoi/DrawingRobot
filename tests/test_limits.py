from math import inf, isclose, pi

from drawingrobot.commands import CommandRunner, WheelCommand, arc, rotate_in_place
from drawingrobot.kinematics import Pose, step
from drawingrobot.limits import Limits, NO_LIMITS


def test_no_clamp_within_bounds():
    lim = Limits(max_linear_cm_s=50.0, max_angular_rad_s=0.5)
    v, w = lim.clamp_vw(10.0, 0.2)
    assert (v, w) == (10.0, 0.2)


def test_clamp_only_linear_excess_scales_both():
    lim = Limits(max_linear_cm_s=50.0, max_angular_rad_s=10.0)
    v, w = lim.clamp_vw(100.0, 0.4)
    # ratio = 50/100 = 0.5, scales omega by the same factor
    assert isclose(v, 50.0)
    assert isclose(w, 0.2)


def test_clamp_only_angular_excess_scales_both():
    lim = Limits(max_linear_cm_s=200.0, max_angular_rad_s=0.5)
    v, w = lim.clamp_vw(20.0, 2.0)
    # ratio = 0.5/2.0 = 0.25
    assert isclose(v, 5.0)
    assert isclose(w, 0.5)


def test_clamp_both_excess_uses_smaller_ratio():
    # v ratio = 50/200 = 0.25; omega ratio = 0.5/1.0 = 0.5; smaller wins
    lim = Limits(max_linear_cm_s=50.0, max_angular_rad_s=0.5)
    v, w = lim.clamp_vw(200.0, 1.0)
    assert isclose(v, 50.0)        # saturated
    assert isclose(w, 0.25)         # scaled by the same ratio (0.25)


def test_curvature_preserved_when_clamped():
    lim = Limits(max_linear_cm_s=10.0, max_angular_rad_s=10.0)
    v_in, w_in = 30.0, 0.3
    v_out, w_out = lim.clamp_vw(v_in, w_in)
    # both scaled by the same ratio, so v/omega is unchanged
    assert isclose(v_in / w_in, v_out / w_out)


def test_signs_preserved():
    lim = Limits(max_linear_cm_s=10.0, max_angular_rad_s=0.5)
    v, w = lim.clamp_vw(-50.0, 1.0)
    assert v < 0 and w > 0
    assert isclose(v, -10.0)
    assert isclose(w, 0.2)


def test_no_limits_is_noop():
    assert NO_LIMITS.clamp_vw(1e6, 1e6) == (1e6, 1e6)
    assert NO_LIMITS.max_linear_cm_s == inf
    assert NO_LIMITS.max_angular_rad_s == inf


def test_zero_or_negative_ceiling_returns_zero():
    # Defensive: a ceiling of 0 means "stop", so we send (0, 0) instead of div-by-zero.
    lim = Limits(max_linear_cm_s=0.0, max_angular_rad_s=0.5)
    assert lim.clamp_vw(5.0, 0.1) == (0.0, 0.0)


def _integrate_pose(cmds, wheelbase, dt=0.005):
    runner = CommandRunner(cmds)
    pose = Pose(0.0, 0.0, 0.0)
    while not runner.done:
        for v_l, v_r, sub_dt in runner.consume(dt):
            pose = step(pose, v_l, v_r, wheelbase, sub_dt)
    return pose


def test_apply_to_command_noop_within_limits():
    lim = Limits(max_linear_cm_s=50.0, max_angular_rad_s=0.5)
    cmd = WheelCommand(v_left=10.0, v_right=10.0, duration=2.0)
    out = lim.apply_to_command(cmd, wheelbase=20.0)
    assert out == cmd


def test_apply_to_command_preserves_arc_geometry_under_clamp():
    # Aggressive arc: speed 30 cm/s, radius 5 cm → ω = 6 rad/s, way above 0.5.
    wheelbase = 20.0
    cmd = arc(radius=5.0, angle=pi / 2, wheelbase=wheelbase, speed=30.0)
    lim = Limits(max_linear_cm_s=100.0, max_angular_rad_s=0.5)

    clamped = lim.apply_to_command(cmd, wheelbase)
    # ω after clamp must be at the ceiling (within ε)
    v_after = 0.5 * (clamped.v_left + clamped.v_right)
    w_after = (clamped.v_right - clamped.v_left) / wheelbase
    assert abs(w_after) <= lim.max_angular_rad_s + 1e-12
    # Curvature preserved → v/ω same as before clamp
    v_before = 0.5 * (cmd.v_left + cmd.v_right)
    w_before = (cmd.v_right - cmd.v_left) / wheelbase
    assert isclose(v_after / w_after, v_before / w_before)

    # Geometry (final pose) preserved between clamped and unclamped integration.
    pose_orig = _integrate_pose([cmd], wheelbase)
    pose_clamped = _integrate_pose([clamped], wheelbase)
    assert isclose(pose_orig.x, pose_clamped.x, abs_tol=1e-6)
    assert isclose(pose_orig.y, pose_clamped.y, abs_tol=1e-6)
    assert isclose(pose_orig.theta, pose_clamped.theta, abs_tol=1e-6)
    # And duration stretched by 1/r
    assert clamped.duration > cmd.duration


def test_apply_to_command_preserves_rotate_geometry_under_clamp():
    wheelbase = 20.0
    cmd = rotate_in_place(angle=pi / 2, wheelbase=wheelbase, angular_speed=4.0)
    lim = Limits(max_linear_cm_s=50.0, max_angular_rad_s=0.5)
    clamped = lim.apply_to_command(cmd, wheelbase)
    w_after = (clamped.v_right - clamped.v_left) / wheelbase
    assert abs(w_after) <= lim.max_angular_rad_s + 1e-12

    pose_orig = _integrate_pose([cmd], wheelbase)
    pose_clamped = _integrate_pose([clamped], wheelbase)
    assert isclose(pose_orig.theta, pose_clamped.theta, abs_tol=1e-6)
    assert isclose(pose_orig.x, pose_clamped.x, abs_tol=1e-6)
    assert isclose(pose_orig.y, pose_clamped.y, abs_tol=1e-6)


def test_apply_to_command_no_limits_returns_same():
    cmd = WheelCommand(v_left=1e6, v_right=-1e6, duration=1.0)
    assert NO_LIMITS.apply_to_command(cmd, wheelbase=20.0) == cmd
