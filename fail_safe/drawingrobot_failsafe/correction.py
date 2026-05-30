"""Closed-loop drift correction at WheelCommand boundaries (fail-safe edition).

Given the encoder-derived `actual` pose and the script-intended `expected`
pose at a command boundary, returns corrective WheelCommands that drive the
wheel-axis midpoint from `actual` to `expected`. Empty list when both
position and heading errors are within their thresholds.

NOTE on the turn primitive: the fail-safe DSL's `turn` pivots around the
LEFT wheel (the pen), which is the wrong primitive for nudging the *midpoint*
pose back onto plan. Corrections instead use a true rotate-in-place about the
midpoint (`v_left = -v_right`), built directly here. The forward leg reuses
`move_straight`.

CAVEAT (rigid pen): on the physical fail-safe robot the pen is always down,
so a correction maneuver leaves a mark on the surface. Corrections are small
(gated by thresholds) and localized at command boundaries, but if a clean
drawing matters more than positional accuracy, run with `--no-correction`
(encoder ghost still shows the drift, nothing is injected).
"""

from __future__ import annotations

from math import atan2, hypot, pi

from .kinematics import Pose
from .limits import Limits
from .script import WheelCommand, move_straight


def _wrap_pi(angle: float) -> float:
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


def _rotate_in_place(angle_rad: float, wheelbase: float,
                     angular_speed: float) -> WheelCommand:
    """Rotate about the wheel-axis midpoint: v_left = -v_right.

    This is the parent package's `rotate_in_place`, *not* the fail-safe
    `pivot_turn` (which pivots about the pen). Correction targets the
    midpoint pose, so midpoint rotation is the correct primitive.
    """
    sign = 1.0 if angle_rad >= 0 else -1.0
    v = sign * angular_speed * wheelbase / 2
    return WheelCommand(-v, v, abs(angle_rad) / angular_speed)


def plan_correction(
    actual: Pose,
    expected: Pose,
    *,
    linear_speed_cm_s: float,
    angular_speed_rad_s: float,
    wheelbase_cm: float,
    limits: Limits,
    pos_threshold_cm: float,
    heading_threshold_rad: float,
) -> list[WheelCommand]:
    pos_err = hypot(expected.x - actual.x, expected.y - actual.y)
    head_err = abs(_wrap_pi(expected.theta - actual.theta))

    if pos_err <= pos_threshold_cm and head_err <= heading_threshold_rad:
        return []

    out: list[WheelCommand] = []
    dx = expected.x - actual.x
    dy = expected.y - actual.y
    dist = hypot(dx, dy)

    if dist > 1e-9:
        # Drive to the target point, then rotate to the target heading.
        # Choose forward or backward translation to minimize total rotation.
        alpha_fwd = atan2(dy, dx)
        alpha_bwd = _wrap_pi(alpha_fwd + pi)

        def total_rot(a: float) -> float:
            return abs(_wrap_pi(a - actual.theta)) + abs(_wrap_pi(expected.theta - a))

        if total_rot(alpha_fwd) <= total_rot(alpha_bwd):
            alpha = alpha_fwd
            d_signed = dist
        else:
            alpha = alpha_bwd
            d_signed = -dist

        rot1 = _wrap_pi(alpha - actual.theta)
        if abs(rot1) > 1e-12:
            out.append(_rotate_in_place(rot1, wheelbase_cm, angular_speed_rad_s))
        out.append(move_straight(d_signed, linear_speed_cm_s))
        rot2 = _wrap_pi(expected.theta - alpha)
        if abs(rot2) > 1e-12:
            out.append(_rotate_in_place(rot2, wheelbase_cm, angular_speed_rad_s))
    else:
        rot = _wrap_pi(expected.theta - actual.theta)
        if abs(rot) > 1e-12:
            out.append(_rotate_in_place(rot, wheelbase_cm, angular_speed_rad_s))

    return [limits.apply_to_command(c, wheelbase_cm) for c in out]
