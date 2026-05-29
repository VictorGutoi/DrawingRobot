"""Closed-loop drift correction at WheelCommand boundaries.

Given the encoder-derived `actual` pose and the script-intended `expected`
pose at a command boundary, returns a list of corrective WheelCommands
that drives wheel-midpoint from `actual` to `expected`. Empty list when
both position and heading errors are within their thresholds.

Reuses `_rotate_translate_rotate` from `script.py` — same forward/backward-
minimizing helper that `_plan_line_to` uses for pen-aware corner setup.
"""

from math import hypot

from .commands import WheelCommand
from .kinematics import Pose
from .limits import Limits
from .script import _rotate_translate_rotate, _wrap_pi


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

    cmds = _rotate_translate_rotate(
        actual, expected.x, expected.y, expected.theta,
        wheelbase_cm, linear_speed_cm_s, angular_speed_rad_s,
    )
    return [limits.apply_to_command(c, wheelbase_cm) for c in cmds]
