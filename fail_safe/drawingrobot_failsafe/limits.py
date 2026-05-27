from __future__ import annotations

from dataclasses import dataclass
from math import inf
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .script import WheelCommand


@dataclass(frozen=True)
class Limits:
    """Velocity ceilings for the body. v in cm/s, omega in rad/s.

    Both `clamp_vw` and `apply_to_command` scale (v, omega) by the same ratio
    when either exceeds its ceiling, so the instantaneous curvature R = v/omega
    is preserved and the body follows the same arc — just slower. `clamp_vw`
    is a stateless safety net used at publish time; `apply_to_command` also
    stretches the WheelCommand's duration by 1/r so the total geometry of the
    motion (distance, angle) is preserved.

    Fail-safe defaults are looser on angular than the parent (2.0 vs 0.5 rad/s)
    because the `turn` and `circle` primitives both want ω > 0.5 rad/s under
    the default speed/angular_speed settings — clamping is graceful (duration
    is stretched, not the motion distorted) but slow.
    """

    max_linear_cm_s: float = 50.0          # 0.5 m/s
    max_angular_rad_s: float = 2.0

    def _ratio(self, v: float, omega: float) -> float:
        ratio = 1.0
        if self.max_linear_cm_s > 0 and abs(v) > self.max_linear_cm_s:
            ratio = min(ratio, self.max_linear_cm_s / abs(v))
        if self.max_angular_rad_s > 0 and abs(omega) > self.max_angular_rad_s:
            ratio = min(ratio, self.max_angular_rad_s / abs(omega))
        return ratio

    def clamp_vw(self, v: float, omega: float) -> tuple[float, float]:
        if self.max_linear_cm_s <= 0 or self.max_angular_rad_s <= 0:
            return 0.0, 0.0
        ratio = self._ratio(v, omega)
        return v * ratio, omega * ratio

    def apply_to_command(self, cmd: "WheelCommand", wheelbase: float) -> "WheelCommand":
        if cmd.duration <= 0 or wheelbase <= 0:
            return cmd
        v = 0.5 * (cmd.v_left + cmd.v_right)
        omega = (cmd.v_right - cmd.v_left) / wheelbase
        ratio = self._ratio(v, omega)
        if ratio >= 1.0:
            return cmd
        # `type(cmd)` lets us construct a same-type instance without importing
        # WheelCommand here (which would create a circular dep with script.py).
        return type(cmd)(
            v_left=cmd.v_left * ratio,
            v_right=cmd.v_right * ratio,
            duration=cmd.duration / ratio,
        )


NO_LIMITS = Limits(max_linear_cm_s=inf, max_angular_rad_s=inf)
