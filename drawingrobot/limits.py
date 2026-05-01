from dataclasses import dataclass
from math import inf


@dataclass(frozen=True)
class Limits:
    """Velocity ceilings for the body. v in cm/s, omega in rad/s.

    `clamp_vw` scales (v, omega) by the same ratio when either exceeds its
    ceiling, so the instantaneous curvature R = v / omega is preserved and
    the body follows the same arc — just slower. This is the conservative
    safety semantics for a real robot: paths keep their shape, durations
    stretch.
    """

    max_linear_cm_s: float = 50.0          # 0.5 m/s
    max_angular_rad_s: float = 0.5

    def clamp_vw(self, v: float, omega: float) -> tuple[float, float]:
        if self.max_linear_cm_s <= 0 or self.max_angular_rad_s <= 0:
            return 0.0, 0.0
        ratio = 1.0
        if abs(v) > self.max_linear_cm_s:
            ratio = min(ratio, self.max_linear_cm_s / abs(v))
        if abs(omega) > self.max_angular_rad_s:
            ratio = min(ratio, self.max_angular_rad_s / abs(omega))
        return v * ratio, omega * ratio


NO_LIMITS = Limits(max_linear_cm_s=inf, max_angular_rad_s=inf)
