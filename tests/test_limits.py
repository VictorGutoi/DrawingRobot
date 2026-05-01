from math import inf, isclose

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
