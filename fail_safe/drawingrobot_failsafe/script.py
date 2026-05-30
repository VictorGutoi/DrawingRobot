"""Fail-safe script DSL: only three movement primitives.

  forward <cm>           drive straight (negative ok). Both wheels at `speed`.
                         Pen draws a line of length `cm` along the heading.
  turn <deg>             rotate around the pen (left wheel locked at v=0).
                         v_right = sign(θ) · angular_speed · wheelbase.
                         Positive deg = CCW.  Pen does NOT move.
  circle <radius>        full CCW circle of radius `radius` traced by the pen,
                         at tangential pen-speed = `speed`. ICC at body
                         (0, +wheelbase/2 + radius); v_left = ω·R,
                         v_right = ω·(W + R), where ω = speed/R.

Plus the two settings (`speed`, `angular_speed`) and `#` comments.

Frame convention matches the parent: x forward, y left, θ CCW from +x.
The pen is pinned to the LEFT wheel (body offset (0, +W/2)). With v_left=0,
the left wheel's world velocity is identically zero, so the pen sits
still during `turn` — that's what enables sharp polygon corners.

In `circle`, `speed` is the **pen** tangential speed (pen sits at the inner
edge of the circle, sweeping the orbit at exactly the configured speed).
This differs from the parent's `arc`/`circle`, where `speed` is the
midpoint speed; documented here so users porting scripts know.
"""

from __future__ import annotations

from dataclasses import dataclass
from math import pi, radians
from pathlib import Path
from typing import Callable, Iterable

from .kinematics import Pose, step
from .limits import Limits, NO_LIMITS


DEFAULT_SPEED = 12.0                       # cm/s
DEFAULT_ANGULAR_SPEED_DEG = 180.0          # deg/s
DURATION_WARNING_S = 60.0

SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "scripts"


@dataclass(frozen=True)
class WheelCommand:
    """Constant left/right wheel velocities held for `duration` seconds."""
    v_left: float
    v_right: float
    duration: float


def move_straight(distance_cm: float, speed: float) -> WheelCommand:
    if speed <= 0:
        raise ValueError("speed must be positive")
    sign = 1 if distance_cm >= 0 else -1
    return WheelCommand(sign * speed, sign * speed, abs(distance_cm) / speed)


def pivot_turn(angle_rad: float, wheelbase: float, angular_speed: float) -> WheelCommand:
    """Pivot around the left wheel: v_left=0, v_right=±angular_speed·W.

    Positive angle = CCW. Robot's wheel-axis midpoint sweeps an arc of radius
    W/2 around the left wheel; left wheel (where the pen sits) stays put.
    """
    if angular_speed <= 0:
        raise ValueError("angular_speed must be positive")
    if wheelbase <= 0:
        raise ValueError("wheelbase must be positive")
    sign = 1 if angle_rad >= 0 else -1
    v_right = sign * angular_speed * wheelbase
    return WheelCommand(0.0, v_right, abs(angle_rad) / angular_speed)


def pen_circle(radius_cm: float, wheelbase: float, pen_speed: float) -> WheelCommand:
    """Full CCW circle traced by the pen at the left wheel.

    ω = pen_speed / radius; v_left = ω·R, v_right = ω·(W + R).
    Duration = 2π / ω = 2π·R / pen_speed.
    """
    if pen_speed <= 0:
        raise ValueError("speed must be positive")
    if radius_cm <= 0:
        raise ValueError("circle radius must be positive")
    if wheelbase <= 0:
        raise ValueError("wheelbase must be positive")
    omega = pen_speed / radius_cm
    v_left = omega * radius_cm
    v_right = omega * (wheelbase + radius_cm)
    duration = 2 * pi / omega
    return WheelCommand(v_left, v_right, duration)


class CommandRunner:
    """Steps through a sequence of WheelCommands in real time."""

    def __init__(self, commands: Iterable[WheelCommand]):
        self._commands: list[WheelCommand] = list(commands)
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

        Splits dt at command boundaries so each segment runs at the right
        velocities. Without this, time spilling past a `turn` boundary
        integrates against the previous command's velocities and the next
        `forward` starts at the wrong heading.
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

    def inject(self, commands: Iterable[WheelCommand]) -> None:
        """Splice commands after the currently-executing one, before the next pending.

        If `done`, inserts at the end and re-arms the runner so it picks up the
        new commands on the next `consume()`. Does not touch `_elapsed` so the
        in-flight command finishes cleanly before the injected sequence runs.
        Used for encoder-feedback corrections fired at command boundaries.
        """
        cmds = list(commands)
        if not cmds:
            return
        insert_at = self._idx if self.done else self._idx + 1
        self._commands[insert_at:insert_at] = cmds


def rescale_runner(runner: CommandRunner, target_time: float) -> CommandRunner:
    """Uniformly scale all commands so the runner's total duration = target_time."""
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


class ScriptError(Exception):
    def __init__(self, line_no: int, message: str):
        super().__init__(f"line {line_no}: {message}")
        self.line_no = line_no
        self.detail = message


def parse_script(
    text: str,
    wheelbase: float,
    limits: Limits = NO_LIMITS,
    on_warning: Callable[[str], None] = print,
) -> list[WheelCommand]:
    """Parse a fail-safe script into a flat list of WheelCommands.

    Grammar (one statement per line, `#` for comments):
        forward <cm>            drive straight
        turn <deg>              pivot around the pen (left wheel locked)
        circle <radius>         CCW pen-circle of given radius
        speed <cm/s>            set linear / pen speed for following commands
        angular_speed <deg/s>   set in-place rotation speed
    """
    if wheelbase <= 0:
        raise ValueError("wheelbase must be positive")

    speed = DEFAULT_SPEED
    angular_speed = radians(DEFAULT_ANGULAR_SPEED_DEG)
    cmds: list[WheelCommand] = []
    pose = Pose(0.0, 0.0, 0.0)

    def _exec(cmd: WheelCommand) -> None:
        nonlocal pose
        cmd = limits.apply_to_command(cmd, wheelbase)
        cmds.append(cmd)
        pose = step(pose, cmd.v_left, cmd.v_right, wheelbase, cmd.duration)

    for line_no, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        op = parts[0].lower()
        args = parts[1:]
        try:
            if op == "forward":
                _exec(move_straight(_arg(args, 0, "distance"), speed))
            elif op == "turn":
                _exec(pivot_turn(
                    radians(_arg(args, 0, "angle")), wheelbase, angular_speed))
            elif op == "circle":
                _exec(pen_circle(_arg(args, 0, "radius"), wheelbase, speed))
            elif op == "speed":
                v = _arg(args, 0, "speed")
                if v <= 0:
                    raise ValueError("speed must be positive")
                speed = v
            elif op == "angular_speed":
                w = radians(_arg(args, 0, "angular_speed"))
                if w <= 0:
                    raise ValueError("angular_speed must be positive")
                angular_speed = w
            else:
                raise ValueError(
                    f"unknown command: {op!r} "
                    "(fail-safe supports only forward / turn / circle / speed / angular_speed)"
                )
        except (IndexError, ValueError) as e:
            raise ScriptError(line_no, str(e)) from e

    total_duration = sum(c.duration for c in cmds)
    if total_duration > DURATION_WARNING_S:
        on_warning(
            f"[parse warning] planned duration {total_duration:.1f}s exceeds "
            f"{DURATION_WARNING_S:.0f}s — limits "
            f"(v≤{limits.max_linear_cm_s*0.01:.2f} m/s, "
            f"ω≤{limits.max_angular_rad_s:.2f} rad/s) may be stretching the "
            f"motion; consider raising limits or lowering script speed."
        )
    return cmds


def _arg(args: list[str], idx: int, name: str) -> float:
    if idx >= len(args):
        raise ValueError(f"missing argument: {name}")
    try:
        return float(args[idx])
    except ValueError:
        raise ValueError(f"invalid {name}: {args[idx]!r}")


def list_scripts() -> list[str]:
    if not SCRIPTS_DIR.exists():
        return []
    return sorted(p.stem for p in SCRIPTS_DIR.glob("*.script"))


def load_script(name: str) -> str:
    return (SCRIPTS_DIR / f"{name}.script").read_text()
