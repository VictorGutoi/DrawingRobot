from math import radians
from pathlib import Path

from .commands import WheelCommand, arc, circle, move_straight, rotate_in_place
from .robot import RobotGeometry


DEFAULT_SPEED = 12.0
DEFAULT_ANGULAR_SPEED_DEG = 180.0

SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "scripts"


class ScriptError(Exception):
    def __init__(self, line_no: int, message: str):
        super().__init__(f"line {line_no}: {message}")
        self.line_no = line_no
        self.detail = message


def parse_script(text: str, geometry: RobotGeometry) -> list[WheelCommand]:
    """Parse a script into a flat list of WheelCommands.

    Grammar (one statement per line, `#` for comments):
        forward <cm>            - drive straight (negative ok)
        back <cm>               - sugar for `forward -<cm>`
        turn <deg>              - rotate in place, + = CCW (left)
        left <deg>              - sugar for `turn <deg>`
        right <deg>             - sugar for `turn -<deg>`
        arc <radius> <deg>      - arc, + angle = left turn
        circle <radius>         - full CCW circle
        speed <cm/s>            - set linear speed for following commands
        angular_speed <deg/s>   - set in-place rotation speed
    """
    speed = DEFAULT_SPEED
    angular_speed = radians(DEFAULT_ANGULAR_SPEED_DEG)
    cmds: list[WheelCommand] = []

    for line_no, raw in enumerate(text.splitlines(), start=1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        parts = line.split()
        op = parts[0].lower()
        args = parts[1:]
        try:
            if op == "forward":
                cmds.append(move_straight(_arg(args, 0, "distance"), speed))
            elif op == "back":
                cmds.append(move_straight(-_arg(args, 0, "distance"), speed))
            elif op in ("turn", "left"):
                cmds.append(rotate_in_place(
                    radians(_arg(args, 0, "angle")), geometry.width, angular_speed))
            elif op == "right":
                cmds.append(rotate_in_place(
                    -radians(_arg(args, 0, "angle")), geometry.width, angular_speed))
            elif op == "arc":
                r = _arg(args, 0, "radius")
                a = _arg(args, 1, "angle")
                cmds.append(arc(r, radians(a), geometry.width, speed))
            elif op == "circle":
                cmds.append(circle(_arg(args, 0, "radius"), geometry.width, speed))
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
                raise ValueError(f"unknown command: {op!r}")
        except (IndexError, ValueError) as e:
            raise ScriptError(line_no, str(e)) from e
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
