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

    def consume(self, dt: float) -> list[tuple[float, float, float]]:
        """Consume `dt` from the command stream as (v_left, v_right, sub_dt) segments.

        Splits dt at command boundaries so each segment runs at the right velocities.
        Without this, time spilling past a command boundary integrates against the
        previous command's velocities — small at low dt, but with off-axis pens or
        strong rotations the residue compounds into visibly wrong angles.
        """
        segments: list[tuple[float, float, float]] = []
        remaining = dt
        while remaining > 0 and not self.done:
            cmd = self._commands[self._idx]
            cmd_remaining = cmd.duration - self._elapsed
            if remaining < cmd_remaining:
                segments.append((cmd.v_left, cmd.v_right, remaining))
                self._elapsed += remaining
                remaining = 0.0
            else:
                segments.append((cmd.v_left, cmd.v_right, cmd_remaining))
                remaining -= cmd_remaining
                self._idx += 1
                self._elapsed = 0.0
        return segments

    def reset(self) -> None:
        self._idx = 0
        self._elapsed = 0.0

    def append(self, commands: Iterable[WheelCommand]) -> None:
        self._commands.extend(commands)


def rescale_runner(runner: CommandRunner, target_time: float) -> CommandRunner:
    """Uniformly scale all commands so the runner's total duration = target_time.

    Path geometry is preserved (same wheel-velocity ratios, same per-command
    distance) — only the timing changes. Velocities × scale, duration / scale.
    No-op if the runner is empty or target_time is non-positive.
    """
    cmds = runner._commands
    if not cmds or target_time <= 0:
        return runner
    total = sum(c.duration for c in cmds)
    if total <= 0:
        return runner
    scale = total / target_time
    return CommandRunner([
        WheelCommand(c.v_left * scale, c.v_right * scale, c.duration / scale)
        for c in cmds
    ])
