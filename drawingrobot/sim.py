from dataclasses import dataclass, field

import pygame

from .commands import CommandRunner, WheelCommand
from .kinematics import Pose, step, transform_point
from .robot import RobotGeometry
from .script import ScriptError, list_scripts, load_script, parse_script
from . import ui


WINDOW_W, WINDOW_H = 1200, 800
CANVAS_W = 800
PANEL_X = CANVAS_W
PANEL_W = WINDOW_W - CANVAS_W
PADDING = 28
INNER_W = PANEL_W - 2 * PADDING
PIXELS_PER_CM = 4.0

# Real robot specs (cm). Used as initial slider values.
REAL_WIDTH_CM = 20.4
REAL_LENGTH_CM = 23.2
REAL_WHEELS_FROM_FRONT_CM = 14.4
REAL_WHEEL_DIAMETER_CM = 6.6

COLOR_CANVAS = (245, 245, 240)
COLOR_GRID = (220, 220, 215)
COLOR_AXIS = (180, 180, 175)
COLOR_TRACE = (40, 80, 200)
COLOR_CHASSIS = (60, 60, 70)
COLOR_WHEEL = (20, 20, 25)
COLOR_PEN = (220, 50, 50)
COLOR_HEADING = (100, 160, 100)
COLOR_ERROR = (220, 110, 110)
COLOR_OK = (140, 200, 140)
COLOR_WHEEL_AXIS = (180, 100, 200)
COLOR_CORNER_ARC = (220, 160, 60)


@dataclass
class SimState:
    geometry: RobotGeometry
    pen_s_normalized: float
    pose: Pose
    runner: CommandRunner
    program_source: str = ""
    running: bool = False
    trace_segments: list[list[tuple[float, float]]] = field(default_factory=lambda: [[]])
    last_console_msg: str = ""
    last_console_msg_is_error: bool = False

    def current_segment(self) -> list[tuple[float, float]]:
        return self.trace_segments[-1]

    def break_trace(self) -> None:
        if self.current_segment():
            self.trace_segments.append([])


def world_to_screen(x: float, y: float) -> tuple[int, int]:
    sx = CANVAS_W / 2 + x * PIXELS_PER_CM
    sy = WINDOW_H / 2 - y * PIXELS_PER_CM
    return int(sx), int(sy)


def draw_canvas_background(surface: pygame.Surface) -> None:
    pygame.draw.rect(surface, COLOR_CANVAS, pygame.Rect(0, 0, CANVAS_W, WINDOW_H))
    grid_step_cm = 10
    grid_px = int(grid_step_cm * PIXELS_PER_CM)
    cx, cy = CANVAS_W // 2, WINDOW_H // 2
    for x in range(cx % grid_px, CANVAS_W, grid_px):
        pygame.draw.line(surface, COLOR_GRID, (x, 0), (x, WINDOW_H))
    for y in range(cy % grid_px, WINDOW_H, grid_px):
        pygame.draw.line(surface, COLOR_GRID, (0, y), (CANVAS_W, y))
    pygame.draw.line(surface, COLOR_AXIS, (cx, 0), (cx, WINDOW_H))
    pygame.draw.line(surface, COLOR_AXIS, (0, cy), (CANVAS_W, cy))


def draw_trace(surface: pygame.Surface, segments: list[list[tuple[float, float]]]) -> None:
    for seg in segments:
        if len(seg) < 2:
            continue
        pts = [world_to_screen(x, y) for x, y in seg]
        pygame.draw.lines(surface, COLOR_TRACE, False, pts, 2)


def draw_robot(surface: pygame.Surface, geometry: RobotGeometry, pose: Pose,
               pen_body: tuple[float, float], pen_world: tuple[float, float]) -> None:
    corners = [transform_point(pose, bx, by) for bx, by in geometry.chassis_corners()]
    screen_corners = [world_to_screen(x, y) for x, y in corners]
    pygame.draw.polygon(surface, COLOR_CHASSIS, screen_corners, 2)

    h = geometry.width / 2
    axis_a = transform_point(pose, 0.0, -h)
    axis_b = transform_point(pose, 0.0, h)
    pygame.draw.line(surface, COLOR_WHEEL_AXIS,
                     world_to_screen(*axis_a), world_to_screen(*axis_b), 1)

    (lb, lf), (rb, rf) = geometry.wheel_endpoints()
    for back_pt, front_pt in ((lb, lf), (rb, rf)):
        wx0, wy0 = transform_point(pose, *back_pt)
        wx1, wy1 = transform_point(pose, *front_pt)
        pygame.draw.line(surface, COLOR_WHEEL, world_to_screen(wx0, wy0),
                         world_to_screen(wx1, wy1), 6)

    heading_tip = transform_point(pose, geometry.front_x, 0.0)
    pygame.draw.line(surface, COLOR_HEADING,
                     world_to_screen(pose.x, pose.y),
                     world_to_screen(*heading_tip), 2)

    px, py = pen_body
    corner_radius_cm = (px * px + py * py) ** 0.5
    if corner_radius_cm > 1e-6:
        radius_px = max(2, int(corner_radius_cm * PIXELS_PER_CM))
        pygame.draw.circle(surface, COLOR_CORNER_ARC,
                           world_to_screen(pose.x, pose.y), radius_px, 1)

    pygame.draw.circle(surface, COLOR_PEN, world_to_screen(*pen_world), 5)


def make_sliders() -> dict[str, ui.Slider]:
    x = PANEL_X + PADDING
    return {
        "width": ui.Slider(pygame.Rect(x, 80, INNER_W, 6), "Width (cm)",
                           8.0, 40.0, REAL_WIDTH_CM, "{:.1f}"),
        "length": ui.Slider(pygame.Rect(x, 140, INNER_W, 6), "Length (cm)",
                            8.0, 50.0, REAL_LENGTH_CM, "{:.1f}"),
        "wheels_from_front": ui.Slider(pygame.Rect(x, 200, INNER_W, 6),
                                       "Wheels from front (cm)",
                                       0.0, 50.0, REAL_WHEELS_FROM_FRONT_CM, "{:.1f}"),
        "pen_s": ui.Slider(pygame.Rect(x, 260, INNER_W, 6),
                           "Pen position (along outline)", 0.0, 1.0, 0.0, "{:.3f}"),
    }


def build_geometry(width: float, length: float, wheels_from_front: float) -> RobotGeometry:
    wheels_from_front = max(0.0, min(length, wheels_from_front))
    wheel_offset_from_back = length - wheels_from_front
    return RobotGeometry(
        width=width,
        length=length,
        wheel_offset=wheel_offset_from_back,
        wheel_diameter=REAL_WHEEL_DIAMETER_CM,
    )


def build_runner(source: str, geometry: RobotGeometry,
                 pen_body: tuple[float, float] = (0.0, 0.0)) -> tuple[CommandRunner, str]:
    """Returns (runner, error_message). On parse error, runner is empty."""
    try:
        cmds = parse_script(source, geometry, pen_body=pen_body)
        return CommandRunner(cmds), ""
    except ScriptError as e:
        return CommandRunner([]), str(e)


def run() -> None:
    pygame.init()
    pygame.display.set_caption("Drawing Robot Simulator")
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Helvetica", 14)
    title_font = pygame.font.SysFont("Helvetica", 18, bold=True)
    mono = pygame.font.SysFont("Menlo,Courier", 13)

    geometry = build_geometry(REAL_WIDTH_CM, REAL_LENGTH_CM, REAL_WHEELS_FROM_FRONT_CM)
    available_scripts = list_scripts()
    initial_source = load_script("square") if "square" in available_scripts else ""
    initial_runner, _ = build_runner(initial_source, geometry, geometry.pen_offset(0.0))

    state = SimState(
        geometry=geometry,
        pen_s_normalized=0.0,
        pose=Pose(0.0, 0.0, 0.0),
        runner=initial_runner,
        program_source=initial_source,
    )

    sliders = make_sliders()

    def current_pen_body() -> tuple[float, float]:
        return state.geometry.pen_offset(state.pen_s_normalized * state.geometry.perimeter)

    def reset_pose_and_trace():
        state.pose = Pose(0.0, 0.0, 0.0)
        state.trace_segments = [[]]

    def rebuild_runner():
        state.runner, err = build_runner(
            state.program_source, state.geometry, current_pen_body())
        if err:
            state.last_console_msg = err
            state.last_console_msg_is_error = True

    def on_start():
        if state.runner.done:
            rebuild_runner()
            reset_pose_and_trace()
        state.running = True
        state.break_trace()

    def on_stop():
        state.running = False
        state.break_trace()

    def on_reset():
        state.running = False
        rebuild_runner()
        reset_pose_and_trace()
        state.last_console_msg = ""
        state.last_console_msg_is_error = False

    def on_select_script(name: str):
        try:
            state.program_source = load_script(name)
        except OSError as e:
            state.last_console_msg = f"failed to load {name}: {e}"
            state.last_console_msg_is_error = True
            return
        rebuild_runner()
        reset_pose_and_trace()
        state.running = False
        state.last_console_msg = f"loaded {name}"
        state.last_console_msg_is_error = False

    def on_console_submit(line: str):
        try:
            new_cmds: list[WheelCommand] = parse_script(
                line, state.geometry, pen_body=current_pen_body())
        except ScriptError as e:
            state.last_console_msg = str(e)
            state.last_console_msg_is_error = True
            return
        state.runner.append(new_cmds)
        if state.program_source and not state.program_source.endswith("\n"):
            state.program_source += "\n"
        state.program_source += line + "\n"
        if not state.running:
            state.running = True
            state.break_trace()
        state.last_console_msg = f"+ {line}"
        state.last_console_msg_is_error = False

    btn_y = 320
    btn_w = (INNER_W - 16) // 3
    buttons = [
        ui.Button(pygame.Rect(PANEL_X + PADDING, btn_y, btn_w, 36), "Start", on_start),
        ui.Button(pygame.Rect(PANEL_X + PADDING + btn_w + 8, btn_y, btn_w, 36), "Stop", on_stop),
        ui.Button(pygame.Rect(PANEL_X + PADDING + 2 * (btn_w + 8), btn_y, btn_w, 36), "Reset", on_reset),
    ]

    cycler = ui.Cycler(
        rect=pygame.Rect(PANEL_X + PADDING, 478, INNER_W, 32),
        options=available_scripts,
        on_change=on_select_script,
        index=available_scripts.index("square") if "square" in available_scripts else 0,
    )

    console = ui.TextInput(
        rect=pygame.Rect(PANEL_X + PADDING, 600, INNER_W, 32),
        placeholder="forward 30 / turn 90 / arc 15 90 / circle 15",
        on_submit=on_console_submit,
    )

    while True:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return
            console.handle_event(event)
            for s in sliders.values():
                s.handle_event(event)
            for b in buttons:
                b.handle_event(event)
            cycler.handle_event(event)

        new_geometry = build_geometry(
            sliders["width"].value,
            sliders["length"].value,
            sliders["wheels_from_front"].value,
        )
        sliders["wheels_from_front"].max_value = new_geometry.length
        if sliders["wheels_from_front"].value > new_geometry.length:
            sliders["wheels_from_front"].value = new_geometry.length

        prev_pen_geom = (state.geometry.width, state.geometry.length,
                         state.geometry.wheel_offset, state.pen_s_normalized)
        state.geometry = new_geometry
        state.pen_s_normalized = sliders["pen_s"].value
        new_pen_geom = (new_geometry.width, new_geometry.length,
                        new_geometry.wheel_offset, state.pen_s_normalized)
        if new_pen_geom != prev_pen_geom:
            state.break_trace()

        pen_body = state.geometry.pen_offset(state.pen_s_normalized * state.geometry.perimeter)

        if state.running and not state.runner.done:
            for v_left, v_right, sub_dt in state.runner.consume(dt):
                state.pose = step(state.pose, v_left, v_right, state.geometry.width, sub_dt)
                state.current_segment().append(transform_point(state.pose, *pen_body))
        elif state.runner.done and state.running:
            state.running = False

        pen_world = transform_point(state.pose, *pen_body)

        screen.fill((20, 22, 26))
        draw_canvas_background(screen)
        draw_trace(screen, state.trace_segments)
        draw_robot(screen, state.geometry, state.pose, pen_body, pen_world)

        ui.draw_panel_background(screen, pygame.Rect(PANEL_X, 0, PANEL_W, WINDOW_H))
        ui.draw_text(screen, title_font, "Drawing Robot Simulator",
                     (PANEL_X + PADDING, 28))

        for s in sliders.values():
            s.draw(screen, font)
        for b in buttons:
            b.draw(screen, font)

        status_y = 380
        status = "running" if state.running else ("done" if state.runner.done else "stopped")
        status_color = COLOR_OK if state.running else ui.COLOR_TEXT
        ui.draw_text(screen, font, f"Status: {status}", (PANEL_X + PADDING, status_y),
                     status_color)
        ui.draw_text(screen, font,
                     f"Pose: x={state.pose.x:6.1f}  y={state.pose.y:6.1f}  th={state.pose.theta:5.2f}",
                     (PANEL_X + PADDING, status_y + 22), ui.COLOR_TEXT_DIM)
        px, py = pen_body
        corner_r = (px * px + py * py) ** 0.5
        on_axis = abs(px) < 1e-6
        axis_msg = " (on axis — sharp corners possible)" if on_axis \
            else f"  off-axis dx={abs(px):.1f} cm"
        ui.draw_text(screen, font,
                     f"Pen swept-arc radius: {corner_r:.1f} cm{axis_msg}",
                     (PANEL_X + PADDING, status_y + 44), ui.COLOR_TEXT_DIM)

        ui.draw_divider(screen, PANEL_X + PADDING, 450, INNER_W)
        ui.draw_text(screen, font, "Script:", (PANEL_X + PADDING, 458))
        cycler.draw(screen, font)

        ui.draw_divider(screen, PANEL_X + PADDING, 555, INNER_W)
        ui.draw_text(screen, font, "Console — type a command, hit Enter",
                     (PANEL_X + PADDING, 575))
        console.draw(screen, mono)

        if state.last_console_msg:
            color = COLOR_ERROR if state.last_console_msg_is_error else ui.COLOR_TEXT_DIM
            msg = state.last_console_msg
            if len(msg) > 48:
                msg = msg[:45] + "..."
            ui.draw_text(screen, mono, msg, (PANEL_X + PADDING, 644), color)

        program_y = 680
        ui.draw_text(screen, font, "Program preview:", (PANEL_X + PADDING, program_y))
        preview_lines = [ln for ln in state.program_source.splitlines() if ln.strip()][-5:]
        for i, line in enumerate(preview_lines):
            display = line if len(line) <= 50 else line[:47] + "..."
            ui.draw_text(screen, mono, display,
                         (PANEL_X + PADDING, program_y + 22 + i * 18),
                         ui.COLOR_TEXT_DIM)

        pygame.display.flip()


if __name__ == "__main__":
    run()
