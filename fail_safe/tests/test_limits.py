"""Tests for the velocity-limiting layer."""

from math import isclose, pi

from drawingrobot_failsafe.limits import Limits, NO_LIMITS
from drawingrobot_failsafe.script import WheelCommand


WHEELBASE = 20.0


def test_no_limits_is_identity():
    cmd = WheelCommand(50.0, 100.0, 1.0)
    result = NO_LIMITS.apply_to_command(cmd, WHEELBASE)
    assert result == cmd


def test_clamp_vw_scales_when_linear_exceeds():
    lim = Limits(max_linear_cm_s=10.0, max_angular_rad_s=100.0)
    v, w = lim.clamp_vw(v=20.0, omega=2.0)
    # ratio = 10/20 = 0.5, both halved.
    assert isclose(v, 10.0)
    assert isclose(w, 1.0)


def test_clamp_vw_scales_when_angular_exceeds():
    lim = Limits(max_linear_cm_s=100.0, max_angular_rad_s=1.0)
    v, w = lim.clamp_vw(v=5.0, omega=4.0)
    # ratio = 1/4 = 0.25, both quartered.
    assert isclose(v, 1.25)
    assert isclose(w, 1.0)


def test_apply_to_command_preserves_curvature():
    """ratio scales (v, ω) equally, so R = v/ω is unchanged."""
    lim = Limits(max_linear_cm_s=100.0, max_angular_rad_s=1.0)
    # Build a command with omega=2 (over limit) and v=20.
    # v_l = v - ωL/2 = 20 - 1·10 = 10; v_r = 30. duration = 1.
    cmd = WheelCommand(v_left=10.0, v_right=30.0, duration=1.0)
    result = lim.apply_to_command(cmd, WHEELBASE)
    v_pre = 0.5 * (cmd.v_left + cmd.v_right)         # 20
    w_pre = (cmd.v_right - cmd.v_left) / WHEELBASE   # 1
    v_post = 0.5 * (result.v_left + result.v_right)
    w_post = (result.v_right - result.v_left) / WHEELBASE
    # Curvature radius R = v/ω preserved.
    assert isclose(v_post / w_post, v_pre / w_pre)


def test_apply_to_command_preserves_distance_via_stretched_duration():
    """v · duration is preserved when limits scale."""
    lim = Limits(max_linear_cm_s=10.0, max_angular_rad_s=100.0)
    cmd = WheelCommand(v_left=20.0, v_right=20.0, duration=2.0)
    result = lim.apply_to_command(cmd, WHEELBASE)
    # v halved, duration doubled, v · duration unchanged.
    pre_distance = 0.5 * (cmd.v_left + cmd.v_right) * cmd.duration
    post_distance = 0.5 * (result.v_left + result.v_right) * result.duration
    assert isclose(pre_distance, post_distance)
