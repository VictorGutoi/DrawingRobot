from math import asin, atan2, cos, hypot, pi, radians, sin, sqrt
from pathlib import Path
from typing import Callable

from .commands import WheelCommand, arc, circle, move_straight, rotate_in_place
from .kinematics import Pose, step
from .limits import Limits, NO_LIMITS
from .robot import RobotGeometry


DEFAULT_SPEED = 12.0
DEFAULT_ANGULAR_SPEED_DEG = 180.0
TRACE_DT = 1.0 / 120.0
TRACE_KP = 8.0
DURATION_WARNING_S = 60.0
TRACE_MAX_STEPS_PER_EDGE = 100_000

SCRIPTS_DIR = Path(__file__).resolve().parent.parent / "scripts"


class ScriptError(Exception):
    def __init__(self, line_no: int, message: str):
        super().__init__(f"line {line_no}: {message}")
        self.line_no = line_no
        self.detail = message


def parse_script(
    text: str,
    geometry: RobotGeometry,
    pen_body: tuple[float, float] = (0.0, 0.0),
    limits: Limits = NO_LIMITS,
    on_warning: Callable[[str], None] = print,
) -> list[WheelCommand]:
    """Parse a script into a flat list of WheelCommands.

    Grammar (one statement per line, `#` for comments):
        forward <cm>            - drive straight (negative ok)
        back <cm>               - sugar for `forward -<cm>`
        turn <deg>              - rotate in place, + = CCW (left)
        left <deg>              - sugar for `turn <deg>`
        right <deg>             - sugar for `turn -<deg>`
        arc <radius> <deg>      - arc, + angle = left turn
        circle <radius>         - full CCW circle
        goto <x> <y>            - drive pen to world point (x, y) along a straight
                                  line. With an off-axis pen, the line itself is
                                  straight; preceding rotation sweeps an arc of
                                  radius |pen_body| (the unavoidable corner fillet).
        line_to <x> <y>         - draw a straight pen line from the current pen
                                  position to (x, y). Edge-aligned planner: emits
                                  a setup (rotate-translate-rotate) that puts the
                                  pen back at its current position with body
                                  pointing at the target, then a forward leg that
                                  traces the polyline edge. The setup pen path is
                                  the corner curve and stays localized at the
                                  corner instead of bulging across the polyline.
        trace <x1> <y1> ...     - track a polyline pen path via feedback
                                  linearisation. Inverts the pen-position
                                  Jacobian (det = px) every timestep and emits
                                  many small WheelCommands; pen follows the
                                  polyline exactly within discretisation.
                                  Requires px ≠ 0 (off-axis pen).
        speed <cm/s>            - set linear speed for following commands
        angular_speed <deg/s>   - set in-place rotation speed

    `pen_body` is the pen's body-frame offset (px, py). It only affects `goto`'s
    inverse-kinematic plan; other commands ignore it.
    """
    speed = DEFAULT_SPEED
    angular_speed = radians(DEFAULT_ANGULAR_SPEED_DEG)
    cmds: list[WheelCommand] = []
    pose = Pose(0.0, 0.0, 0.0)

    def _exec(cmd: WheelCommand) -> None:
        nonlocal pose
        cmd = limits.apply_to_command(cmd, geometry.width)
        cmds.append(cmd)
        pose = step(pose, cmd.v_left, cmd.v_right, geometry.width, cmd.duration)

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
            elif op == "back":
                _exec(move_straight(-_arg(args, 0, "distance"), speed))
            elif op in ("turn", "left"):
                _exec(rotate_in_place(
                    radians(_arg(args, 0, "angle")), geometry.width, angular_speed))
            elif op == "right":
                _exec(rotate_in_place(
                    -radians(_arg(args, 0, "angle")), geometry.width, angular_speed))
            elif op == "arc":
                r = _arg(args, 0, "radius")
                a = _arg(args, 1, "angle")
                _exec(arc(r, radians(a), geometry.width, speed))
            elif op == "circle":
                _exec(circle(_arg(args, 0, "radius"), geometry.width, speed))
            elif op == "goto":
                x_t = _arg(args, 0, "x")
                y_t = _arg(args, 1, "y")
                for plan_cmd in _plan_goto(
                    (x_t, y_t), pose, pen_body, geometry, speed, angular_speed):
                    _exec(plan_cmd)
            elif op == "line_to":
                x_t = _arg(args, 0, "x")
                y_t = _arg(args, 1, "y")
                for plan_cmd in _plan_line_to(
                    (x_t, y_t), pose, pen_body, geometry, speed, angular_speed):
                    _exec(plan_cmd)
            elif op == "trace":
                if len(args) < 2 or len(args) % 2 != 0:
                    raise ValueError("trace needs at least one (x y) pair, "
                                     "given as space-separated numbers")
                vertices: list[tuple[float, float]] = []
                for j in range(0, len(args), 2):
                    vx = _arg(args, j, f"x{j // 2}")
                    vy = _arg(args, j + 1, f"y{j // 2}")
                    vertices.append((vx, vy))
                for plan_cmd in _plan_trace(
                    vertices, pose, pen_body, geometry, speed, limits):
                    _exec(plan_cmd)
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

    total_duration = sum(c.duration for c in cmds)
    if total_duration > DURATION_WARNING_S:
        on_warning(
            f"[parse warning] planned duration {total_duration:.1f}s exceeds "
            f"{DURATION_WARNING_S:.0f}s — limits "
            f"(v≤{limits.max_linear_cm_s*0.01:.2f} m/s, "
            f"ω≤{limits.max_angular_rad_s:.2f} rad/s) may be stretching trace "
            f"corners; consider raising limits or lowering script speed."
        )
    return cmds


def _plan_goto(
    target: tuple[float, float],
    pose: Pose,
    pen_body: tuple[float, float],
    geometry: RobotGeometry,
    speed: float,
    angular_speed: float,
) -> list[WheelCommand]:
    """Return rotate+forward that lands the pen exactly on `target`.

    Math: with wheel midpoint at M = (pose.x, pose.y) and pen body offset (px, py),
    after rotating to heading θ and driving forward d, the pen is at
        M + R_θ·(px, py) + d·(cosθ, sinθ).
    Setting this equal to `target` and projecting perpendicular to (cosθ, sinθ)
    gives the heading equation
        dx·sinθ − dy·cosθ = −py            (dx, dy = target − M)
    with closed-form solution θ = atan2(dy, dx) + asin(−py / r), r = hypot(dx, dy).
    The forward distance is then d = sqrt(r² − py²) − px.

    Reachability: r must be ≥ hypot(px, py) (target outside the pen's swept circle
    around M, otherwise no straight-line approach exists).
    """
    px, py = pen_body
    dx = target[0] - pose.x
    dy = target[1] - pose.y
    r2 = dx * dx + dy * dy
    pen_r2 = px * px + py * py
    if r2 < pen_r2 + 1e-12:
        raise ValueError(
            f"goto target {target} is inside the pen's swept circle around the "
            f"wheel midpoint (r={sqrt(r2):.2f} cm < |pen|={sqrt(pen_r2):.2f} cm) "
            f"— pen can't reach it via rotate+forward"
        )
    beta = atan2(dy, dx)
    theta_t = beta + asin(-py / sqrt(r2))
    d = sqrt(r2 - py * py) - px
    if d <= 1e-12:
        raise ValueError(
            f"goto target {target} requires non-positive forward distance "
            f"(d={d:.3f}) — reorder commands or use forward/back"
        )
    delta = theta_t - pose.theta
    while delta > pi:
        delta -= 2 * pi
    while delta < -pi:
        delta += 2 * pi

    out: list[WheelCommand] = []
    if abs(delta) > 1e-12:
        out.append(rotate_in_place(delta, geometry.width, angular_speed))
    out.append(move_straight(d, speed))
    return out


def _wrap_pi(angle: float) -> float:
    while angle > pi:
        angle -= 2 * pi
    while angle < -pi:
        angle += 2 * pi
    return angle


def _plan_line_to(
    target: tuple[float, float],
    pose: Pose,
    pen_body: tuple[float, float],
    geometry: RobotGeometry,
    speed: float,
    angular_speed: float,
) -> list[WheelCommand]:
    """Edge-aligned: pen draws a straight line from current pen position to target.

    Compiles to setup (rotate-translate-rotate) + forward. The setup repositions
    the wheel midpoint such that, with body heading aligned to the target
    direction, the pen sits at its current world position; the forward leg then
    traces the polyline edge directly.

    Setup translation Δ = (R_θ_curr − R_θ_new) · pen_body — depends only on the
    heading change, not on the polyline geometry. For px ≠ 0, |Δ| is generally
    nonzero, so the corner is a 3-segment curve (arc, straight, arc) at radius
    |pen_body|. Localized at the corner, edges stay clean.
    """
    px, py = pen_body
    c, s = cos(pose.theta), sin(pose.theta)
    pen_x = pose.x + px * c - py * s
    pen_y = pose.y + px * s + py * c

    dx = target[0] - pen_x
    dy = target[1] - pen_y
    edge_len = hypot(dx, dy)
    if edge_len < 1e-12:
        return []

    theta_new = atan2(dy, dx)
    cn, sn = cos(theta_new), sin(theta_new)
    M_setup_x = pen_x - (px * cn - py * sn)
    M_setup_y = pen_y - (px * sn + py * cn)

    delta_x = M_setup_x - pose.x
    delta_y = M_setup_y - pose.y
    delta_mag = hypot(delta_x, delta_y)

    out: list[WheelCommand] = []

    if delta_mag > 1e-12:
        alpha_fwd = atan2(delta_y, delta_x)
        alpha_bwd = _wrap_pi(alpha_fwd + pi)

        def total_rot(a: float) -> float:
            return abs(_wrap_pi(a - pose.theta)) + abs(_wrap_pi(theta_new - a))

        if total_rot(alpha_fwd) <= total_rot(alpha_bwd):
            alpha = alpha_fwd
            d_setup = delta_mag
        else:
            alpha = alpha_bwd
            d_setup = -delta_mag

        rot1 = _wrap_pi(alpha - pose.theta)
        if abs(rot1) > 1e-12:
            out.append(rotate_in_place(rot1, geometry.width, angular_speed))
        out.append(move_straight(d_setup, speed))
        rot2 = _wrap_pi(theta_new - alpha)
        if abs(rot2) > 1e-12:
            out.append(rotate_in_place(rot2, geometry.width, angular_speed))
    else:
        rot = _wrap_pi(theta_new - pose.theta)
        if abs(rot) > 1e-12:
            out.append(rotate_in_place(rot, geometry.width, angular_speed))

    out.append(move_straight(edge_len, speed))
    return out


def _plan_trace(
    vertices: list[tuple[float, float]],
    pose: Pose,
    pen_body: tuple[float, float],
    geometry: RobotGeometry,
    speed: float,
    limits: Limits = NO_LIMITS,
) -> list[WheelCommand]:
    """Track a polyline with the pen via feedback linearisation at the offset point.

    Pen world velocity = J(θ) · (v, ω) where J has det = px (the body-x component
    of the pen offset). Whenever px ≠ 0, J is invertible, so we can synthesise
    (v, ω) at every timestep that produces *any* commanded pen velocity. The pen
    tracks an arbitrary polyline (sharp corners and all) within timestep
    discretisation; the body weaves underneath to make it work.

    With finite `limits`, each step computes the un-clamped (v, ω) demand,
    scales both by r = min(1, v_max/|v|, ω_max/|ω|), and advances the path
    parameter by r·speed·TRACE_DT. The pen still tracks the polyline exactly;
    body dwells longer near sharp corners so peak |ω| stays legal. With
    `limits = NO_LIMITS` this reduces to the unclamped controller running at
    a fixed pen-speed of `speed`.

    Reference: De Luca / Oriolo / Vendittelli, *Control of Wheeled Mobile Robots*
    (RAMSETE 2001) — input-output linearisation at an offset output point.
    """
    px, py = pen_body
    if abs(px) < 1e-9:
        raise ValueError(
            "trace requires off-axis pen (body-x ≠ 0); the Jacobian is singular "
            "for pen on the wheel-axis line. Use line_to/goto for that case."
        )
    if not vertices:
        return []

    L = geometry.width
    cmds: list[WheelCommand] = []

    c0, s0 = cos(pose.theta), sin(pose.theta)
    pen_x = pose.x + px * c0 - py * s0
    pen_y = pose.y + px * s0 + py * c0
    waypoints = [(pen_x, pen_y), *vertices]

    cur = Pose(pose.x, pose.y, pose.theta)
    nominal_advance = speed * TRACE_DT

    for i in range(len(waypoints) - 1):
        v_start = waypoints[i]
        v_end = waypoints[i + 1]
        ex = v_end[0] - v_start[0]
        ey = v_end[1] - v_start[1]
        edge_len = hypot(ex, ey)
        if edge_len < 1e-12:
            continue
        tx = ex / edge_len
        ty = ey / edge_len

        s_along = 0.0
        step_count = 0
        while s_along < edge_len - 1e-12 and step_count < TRACE_MAX_STEPS_PER_EDGE:
            step_count += 1
            target_s = min(s_along + nominal_advance, edge_len)
            ratio_along = target_s / edge_len
            pen_des_x = v_start[0] + ex * ratio_along
            pen_des_y = v_start[1] + ey * ratio_along

            cth, sth = cos(cur.theta), sin(cur.theta)
            pen_cur_x = cur.x + px * cth - py * sth
            pen_cur_y = cur.y + px * sth + py * cth

            v_pen_x = speed * tx + TRACE_KP * (pen_des_x - pen_cur_x)
            v_pen_y = speed * ty + TRACE_KP * (pen_des_y - pen_cur_y)

            a = px * sth + py * cth
            b = px * cth - py * sth
            v = (b * v_pen_x + a * v_pen_y) / px
            omega = (-sth * v_pen_x + cth * v_pen_y) / px

            r = limits._ratio(v, omega)
            v *= r
            omega *= r

            v_l = v - omega * L / 2
            v_r = v + omega * L / 2

            cmds.append(WheelCommand(v_l, v_r, TRACE_DT))
            cur = step(cur, v_l, v_r, L, TRACE_DT)
            s_along = min(s_along + r * nominal_advance, edge_len)

        if step_count >= TRACE_MAX_STEPS_PER_EDGE:
            raise ValueError(
                f"trace edge {i} hit max-step guard "
                f"({TRACE_MAX_STEPS_PER_EDGE}) before completing; limits may be "
                f"too tight or the path too aggressive."
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
