from dataclasses import dataclass


@dataclass(frozen=True)
class RobotGeometry:
    """Rectangular chassis with two opposed wheels and a pen on the outline.

    Body frame: origin at wheel-axis midpoint, x forward, y left.
    `wheel_offset` is measured from the back of the chassis to the wheel axis,
    along the length axis. So back edge is at x=-wheel_offset, front at x=length-wheel_offset.
    Wheels sit at (0, +width/2) and (0, -width/2).

    `wheel_diameter` is for visualization only — the kinematics layer doesn't
    depend on it. Pass 0 to fall back to a default size proportional to length.
    """
    width: float
    length: float
    wheel_offset: float
    wheel_diameter: float = 0.0

    def __post_init__(self):
        if self.width <= 0 or self.length <= 0:
            raise ValueError("width and length must be positive")
        if not 0 <= self.wheel_offset <= self.length:
            raise ValueError("wheel_offset must lie within [0, length]")
        if self.wheel_diameter < 0:
            raise ValueError("wheel_diameter must be non-negative")

    @property
    def perimeter(self) -> float:
        return 2 * (self.width + self.length)

    @property
    def back_x(self) -> float:
        return -self.wheel_offset

    @property
    def front_x(self) -> float:
        return self.length - self.wheel_offset

    def chassis_corners(self) -> list[tuple[float, float]]:
        h = self.width / 2
        return [
            (self.back_x, -h),
            (self.front_x, -h),
            (self.front_x, h),
            (self.back_x, h),
        ]

    def wheel_endpoints(self) -> tuple[
        tuple[tuple[float, float], tuple[float, float]],
        tuple[tuple[float, float], tuple[float, float]],
    ]:
        """Return ((left_back, left_front), (right_back, right_front)) for drawing wheels as line segments.

        Wheels are drawn as a segment of length = wheel_diameter along the rolling axis.
        """
        h = self.width / 2
        diameter = self.wheel_diameter if self.wheel_diameter > 0 else self.length * 0.25
        t = diameter / 2
        return (
            ((-t, h), (t, h)),
            ((-t, -h), (t, -h)),
        )

    def pen_offset(self, s: float) -> tuple[float, float]:
        """Map perimeter parameter s to a point on the chassis outline.

        Walks counterclockwise (when viewed from above with x-forward, y-left)
        starting at the back-right corner.
        """
        s = s % self.perimeter
        L, W = self.length, self.width
        h = W / 2

        if s < L:
            return (self.back_x + s, -h)
        s -= L
        if s < W:
            return (self.front_x, -h + s)
        s -= W
        if s < L:
            return (self.front_x - s, h)
        s -= L
        return (self.back_x, h - s)
