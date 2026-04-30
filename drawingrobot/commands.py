from dataclasses import dataclass
from math import pi
from typing import Iterable


@dataclass(frozen=True)
class WheelCommand:
    """Constant left/right wheel velocities held for `duration` seconds."""
    v_left: float
    v_right: float
    duration: float


def move_straight(distance: float, speed: float) -> WheelCommand:
    if speed <= 0:
        raise ValueError("speed must be positive")
    sign = 1 if distance >= 0 else -1
    return WheelCommand(sign * speed, sign * speed, abs(distance) / speed)


def rotate_in_place(angle: float, wheelbase: float, angular_speed: float) -> WheelCommand:
    if angular_speed <= 0:
        raise ValueError("angular_speed must be positive")
    if wheelbase <= 0:
        raise ValueError("wheelbase must be positive")
    sign = 1 if angle >= 0 else -1
    v = sign * angular_speed * wheelbase / 2
    return WheelCommand(-v, v, abs(angle) / angular_speed)


def arc(radius: float, angle: float, wheelbase: float, speed: float) -> WheelCommand:
    """Drive an arc of given radius and signed angle (positive = left/CCW turn).

    `speed` is the linear speed of the wheel-axis midpoint along the arc.
    """
    if speed <= 0:
        raise ValueError("speed must be positive")
    if radius <= 0:
        raise ValueError("radius must be positive")
    if wheelbase <= 0:
        raise ValueError("wheelbase must be positive")
    sign = 1 if angle >= 0 else -1
    omega = sign * speed / radius
    half_L = wheelbase / 2
    return WheelCommand(
        v_left=speed - omega * half_L,
        v_right=speed + omega * half_L,
        duration=abs(angle) / abs(omega),
    )


def circle(radius: float, wheelbase: float, speed: float) -> WheelCommand:
    return arc(radius, 2 * pi, wheelbase, speed)


class CommandRunner:
    """Steps through a sequence of WheelCommands in real time."""

    def __init__(self, commands: Iterable[WheelCommand]):
        self._commands = list(commands)
        self._idx = 0
        self._elapsed = 0.0

    @property
    def done(self) -> bool:
        return self._idx >= len(self._commands)

    def current_velocities(self) -> tuple[float, float]:
        if self.done:
            return 0.0, 0.0
        cmd = self._commands[self._idx]
        return cmd.v_left, cmd.v_right

    def advance(self, dt: float) -> None:
        if self.done:
            return
        self._elapsed += dt
        while not self.done and self._elapsed >= self._commands[self._idx].duration:
            self._elapsed -= self._commands[self._idx].duration
            self._idx += 1

    def reset(self) -> None:
        self._idx = 0
        self._elapsed = 0.0

    def append(self, commands: Iterable[WheelCommand]) -> None:
        self._commands.extend(commands)
