from dataclasses import dataclass, field

import pygame

from .commands import CommandRunner, WheelCommand
from .kinematics import Pose, step, transform_point
from .limits import Limits, NO_LIMITS
from .robot import RobotGeometry
from .script import ScriptError, list_scripts, load_script, parse_script
from .slots_config import SlotsConfigError, default_slots_path, load_slots
from . import ui


WINDOW_W, WINDOW_H = 1200, 960
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
COLOR_WHEEL_VECTOR = (230, 130, 50)

# Visual arrow length per (cm/s) of wheel velocity, in world cm. A 12 cm/s
# wheel velocity gives a 6 cm arrow on the simulated canvas.
WHEEL_VECTOR_CM_PER_CM_S = 0.5

# "Ghost" robot — what the *clamped* (v, ω) actually sent on /cmd_vel will
# trace on the real robot. Diverges from the main render whenever the script
# asks for more linear or angular velocity than the limits allow.
COLOR_GHOST_TRACE = (170, 190, 220)
COLOR_GHOST_CHASSIS = (160, 160, 170)
COLOR_GHOST_WHEEL = (130, 130, 140)
COLOR_GHOST_PEN = (235, 170, 170)
COLOR_GHOST_HEADING = (170, 200, 170)
COLOR_GHOST_WHEEL_AXIS = (200, 170, 215)
COLOR_GHOST_CORNER_ARC = (225, 195, 150)
COLOR_GHOST_WHEEL_VECTOR = (220, 180, 140)


@dataclass
class SimState:
    geometry: RobotGeometry
    pen_s_normalized: float
    pose: Pose
    runner: CommandRunner
    ghost_pose: Pose = field(default_factory=lambda: Pose(0.0, 0.0, 0.0))
    program_source: str = ""
    running: bool = False
    trace_segments: list[list[tuple[float, float]]] = field(default_factory=lambda: [[]])
    ghost_trace_segments: list[list[tuple[float, float]]] = field(default_factory=lambda: [[]])
    last_console_msg: str = ""
    last_console_msg_is_error: bool = False
    show_robot: bool = True

    def current_segment(self) -> list[tuple[float, float]]:
        return self.trace_segments[-1]

    def current_ghost_segment(self) -> list[tuple[float, float]]:
        return self.ghost_trace_segments[-1]

    def break_trace(self) -> None:
        if self.current_segment():
            self.trace_segments.append([])
        if self.current_ghost_segment():
            self.ghost_trace_segments.append([])


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


@dataclass(frozen=True)
class RobotPalette:
    chassis: tuple[int, int, int]
    wheel: tuple[int, int, int]
    wheel_axis: tuple[int, int, int]
    heading: tuple[int, int, int]
    pen: tuple[int, int, int]
    corner_arc: tuple[int, int, int]
    wheel_vector: tuple[int, int, int]


PALETTE_DEFAULT = RobotPalette(
    chassis=COLOR_CHASSIS, wheel=COLOR_WHEEL, wheel_axis=COLOR_WHEEL_AXIS,
    heading=COLOR_HEADING, pen=COLOR_PEN, corner_arc=COLOR_CORNER_ARC,
    wheel_vector=COLOR_WHEEL_VECTOR,
)
PALETTE_GHOST = RobotPalette(
    chassis=COLOR_GHOST_CHASSIS, wheel=COLOR_GHOST_WHEEL,
    wheel_axis=COLOR_GHOST_WHEEL_AXIS, heading=COLOR_GHOST_HEADING,
    pen=COLOR_GHOST_PEN, corner_arc=COLOR_GHOST_CORNER_ARC,
    wheel_vector=COLOR_GHOST_WHEEL_VECTOR,
)


def draw_trace(surface: pygame.Surface,
               segments: list[list[tuple[float, float]]],
               color: tuple[int, int, int] = COLOR_TRACE,
               width: int = 2) -> None:
    for seg in segments:
        if len(seg) < 2:
            continue
        pts = [world_to_screen(x, y) for x, y in seg]
        pygame.draw.lines(surface, color, False, pts, width)


def _draw_arrow(surface: pygame.Surface, tail: tuple[float, float],
                tip: tuple[float, float], color: tuple[int, int, int],
                width: int = 2, head_px: float = 7.0) -> None:
    """Line segment from tail to tip plus a small chevron arrowhead at tip.
    Coordinates are in screen pixels.
    """
    from math import atan2, cos, sin
    tx, ty = tail
    hx, hy = tip
    pygame.draw.line(surface, color, (tx, ty), (hx, hy), width)
    angle = atan2(hy - ty, hx - tx)
    spread = 0.5  # ~30° on each side of the shaft
    for sign in (1, -1):
        ax = hx - head_px * cos(angle + sign * spread)
        ay = hy - head_px * sin(angle + sign * spread)
        pygame.draw.line(surface, color, (hx, hy), (ax, ay), width)


def draw_robot(surface: pygame.Surface, geometry: RobotGeometry, pose: Pose,
               pen_body: tuple[float, float], pen_world: tuple[float, float],
               palette: RobotPalette = PALETTE_DEFAULT,
               wheel_velocities: tuple[float, float] = (0.0, 0.0)) -> None:
    corners = [transform_point(pose, bx, by) for bx, by in geometry.chassis_corners()]
    screen_corners = [world_to_screen(x, y) for x, y in corners]
    pygame.draw.polygon(surface, palette.chassis, screen_corners, 2)

    h = geometry.width / 2
    axis_a = transform_point(pose, 0.0, -h)
    axis_b = transform_point(pose, 0.0, h)
    pygame.draw.line(surface, palette.wheel_axis,
                     world_to_screen(*axis_a), world_to_screen(*axis_b), 1)

    (lb, lf), (rb, rf) = geometry.wheel_endpoints()
    for back_pt, front_pt in ((lb, lf), (rb, rf)):
        wx0, wy0 = transform_point(pose, *back_pt)
        wx1, wy1 = transform_point(pose, *front_pt)
        pygame.draw.line(surface, palette.wheel, world_to_screen(wx0, wy0),
                         world_to_screen(wx1, wy1), 6)

    # Per-wheel velocity arrows: anchored at each wheel's center on the axis,
    # pointing along the body forward axis. Length proportional to |v_wheel|;
    # sign flips the direction so negative (reverse-spinning) wheels show a
    # backward arrow.
    v_left, v_right = wheel_velocities
    h = geometry.width / 2
    for body_y, v_wheel in ((h, v_left), (-h, v_right)):
        if abs(v_wheel) < 1e-6:
            continue
        length_cm = v_wheel * WHEEL_VECTOR_CM_PER_CM_S
        tail_world = transform_point(pose, 0.0, body_y)
        tip_world = transform_point(pose, length_cm, body_y)
        _draw_arrow(surface,
                    world_to_screen(*tail_world),
                    world_to_screen(*tip_world),
                    palette.wheel_vector)

    heading_tip = transform_point(pose, geometry.front_x, 0.0)
    pygame.draw.line(surface, palette.heading,
                     world_to_screen(pose.x, pose.y),
                     world_to_screen(*heading_tip), 2)

    px, py = pen_body
    corner_radius_cm = (px * px + py * py) ** 0.5
    if corner_radius_cm > 1e-6:
        radius_px = max(2, int(corner_radius_cm * PIXELS_PER_CM))
        pygame.draw.circle(surface, palette.corner_arc,
                           world_to_screen(pose.x, pose.y), radius_px, 1)

    pygame.draw.circle(surface, palette.pen, world_to_screen(*pen_world), 5)


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
        "total_time_s": ui.Slider(pygame.Rect(x, 320, INNER_W, 6),
                                  "Total run time (s)", 5.0, 120.0, 30.0, "{:.0f}"),
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
                 pen_body: tuple[float, float] = (0.0, 0.0),
                 limits: Limits = NO_LIMITS,
                 on_warning=print) -> tuple[CommandRunner, str]:
    """Returns (runner, error_message). On parse error, runner is empty."""
    try:
        cmds = parse_script(source, geometry, pen_body=pen_body,
                            limits=limits, on_warning=on_warning)
        return CommandRunner(cmds), ""
    except ScriptError as e:
        return CommandRunner([]), str(e)


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


def run(ros_enabled: bool = False,
        ros_topic: str = "/cmd_vel",
        limits: Limits | None = None,
        mode_topic: str = "/robot/mode_cmd",
        slots_path: str | None = None) -> None:
    if limits is None:
        limits = NO_LIMITS

    pending_warnings: list[str] = []

    def record_warning(msg: str) -> None:
        print(msg, flush=True)
        pending_warnings.append(msg)

    publisher = None
    mode_publisher = None
    twist_listener = None
    if ros_enabled:
        from .ros_publisher import RosPublisher, TwistListener
        from .mode_publisher import ModePublisher, MODE_REMOTE_DRIVE, MODE_STOP
        publisher = RosPublisher(topic=ros_topic)
        mode_publisher = ModePublisher(topic=mode_topic)
        twist_listener = TwistListener(topic=ros_topic)
    else:
        # Import the mode constants unconditionally so handlers below can
        # reference them without a NameError when ROS is off.
        from .mode_publisher import MODE_REMOTE_DRIVE, MODE_STOP

    slots_load_error: str | None = None
    if slots_path is None:
        slots_path = str(default_slots_path())
    try:
        slots = load_slots(slots_path)
    except SlotsConfigError as e:
        slots = {}
        slots_load_error = f"slots: {e}"
        print(f"[sim] {slots_load_error}", flush=True)

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
    initial_runner, _ = build_runner(
        initial_source, geometry, geometry.pen_offset(0.0),
        limits=limits, on_warning=record_warning)
    initial_runner = rescale_runner(initial_runner, 30.0)  # match slider default

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
        state.ghost_pose = Pose(0.0, 0.0, 0.0)
        state.trace_segments = [[]]
        state.ghost_trace_segments = [[]]

    def rebuild_runner():
        pending_warnings.clear()
        state.runner, err = build_runner(
            state.program_source, state.geometry, current_pen_body(),
            limits=limits, on_warning=record_warning)
        state.runner = rescale_runner(state.runner, sliders["total_time_s"].value)
        if err:
            state.last_console_msg = err
            state.last_console_msg_is_error = True
        elif pending_warnings:
            state.last_console_msg = pending_warnings[-1]
            state.last_console_msg_is_error = False

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

    def on_set_drive_mode():
        if mode_publisher is None:
            state.last_console_msg = "needs --ros to publish /robot/mode_cmd"
            state.last_console_msg_is_error = True
            return
        try:
            mode_publisher.publish_mode(MODE_REMOTE_DRIVE)
        except Exception as e:
            state.last_console_msg = f"mode publish failed: {e}"
            state.last_console_msg_is_error = True
            return
        state.last_console_msg = f"published Int8({MODE_REMOTE_DRIVE}) → {mode_topic}"
        state.last_console_msg_is_error = False

    def make_on_slot(slot_idx: int):
        # Captured-by-value closure for the slot button click.
        def handler():
            if mode_publisher is None:
                state.last_console_msg = "needs --ros to publish /robot/mode_cmd"
                state.last_console_msg_is_error = True
                return
            try:
                mode_publisher.publish_script_slot(slot_idx)
            except Exception as e:
                state.last_console_msg = f"slot publish failed: {e}"
                state.last_console_msg_is_error = True
                return
            entry = slots.get(slot_idx)
            label = entry.script if entry else "(empty slot)"
            state.last_console_msg = (
                f"published Int8({80 + slot_idx}) → {mode_topic}  [{label}]"
            )
            state.last_console_msg_is_error = False
        return handler

    def on_stop_remote():
        if mode_publisher is None:
            state.last_console_msg = "needs --ros to publish /robot/mode_cmd"
            state.last_console_msg_is_error = True
            return
        try:
            mode_publisher.publish_mode(MODE_STOP)
        except Exception as e:
            state.last_console_msg = f"stop publish failed: {e}"
            state.last_console_msg_is_error = True
            return
        state.last_console_msg = f"published Int8({MODE_STOP}) → {mode_topic}  [STOP]"
        state.last_console_msg_is_error = False

    def on_console_submit(line: str):
        pending_warnings.clear()
        try:
            new_cmds: list[WheelCommand] = parse_script(
                line, state.geometry, pen_body=current_pen_body(),
                limits=limits, on_warning=record_warning)
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

    btn_y = 380
    btn_w = (INNER_W - 24) // 4
    toggle_robot_btn = ui.Button(
        pygame.Rect(PANEL_X + PADDING + 3 * (btn_w + 8), btn_y, btn_w, 36),
        "Hide Robot", lambda: None)  # on_click set below once we can reference btn

    def on_toggle_robot():
        state.show_robot = not state.show_robot
        toggle_robot_btn.label = "Show Robot" if not state.show_robot else "Hide Robot"

    toggle_robot_btn.on_click = on_toggle_robot

    buttons = [
        ui.Button(pygame.Rect(PANEL_X + PADDING, btn_y, btn_w, 36), "Start", on_start),
        ui.Button(pygame.Rect(PANEL_X + PADDING + btn_w + 8, btn_y, btn_w, 36), "Stop", on_stop),
        ui.Button(pygame.Rect(PANEL_X + PADDING + 2 * (btn_w + 8), btn_y, btn_w, 36), "Reset", on_reset),
        toggle_robot_btn,
        # LULOC2: publish Int8(3) to /robot/mode_cmd. Sits between the script
        # cycler (bottom y≈510) and the console divider (y=555).
        ui.Button(pygame.Rect(PANEL_X + PADDING, 518, INNER_W, 32),
                  "Set LULOC2 → drive mode (3)", on_set_drive_mode,
                  enabled=mode_publisher is not None),
    ]

    # Slots panel: 2 rows × 5 columns of slot buttons + a wide STOP row.
    # Disabled when --ros is off (mode_publisher absent) or when the slot
    # is unconfigured. The handler still gets a chance to surface a hint
    # in the console message line — Button.handle_event fires regardless.
    SLOTS_HEADER_Y = 810
    SLOT_ROW1_Y = 836
    SLOT_ROW2_Y = 876
    SLOT_STOP_Y = 916
    SLOT_BTN_H = 34
    SLOT_GAP = 6
    slot_btn_w = (INNER_W - 4 * SLOT_GAP) // 5

    slot_buttons: list[ui.Button] = []
    for i in range(10):
        row = i // 5
        col = i % 5
        x = PANEL_X + PADDING + col * (slot_btn_w + SLOT_GAP)
        y = SLOT_ROW1_Y if row == 0 else SLOT_ROW2_Y
        entry = slots.get(i)
        label = f"{i}: {entry.label}" if entry else f"{i}: —"
        slot_buttons.append(ui.Button(
            pygame.Rect(x, y, slot_btn_w, SLOT_BTN_H),
            label,
            make_on_slot(i),
            enabled=(mode_publisher is not None and entry is not None),
        ))
    stop_button = ui.Button(
        pygame.Rect(PANEL_X + PADDING, SLOT_STOP_Y, INNER_W, 30),
        "STOP (Int8 0)",
        on_stop_remote,
        enabled=mode_publisher is not None,
    )
    buttons.extend(slot_buttons)
    buttons.append(stop_button)

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

    last_v = 0.0
    last_omega = 0.0
    last_wheel_v = (0.0, 0.0)              # unclamped — main robot's arrows
    last_wheel_v_clamped = (0.0, 0.0)      # clamped — ghost's arrows
    prev_total_time = sliders["total_time_s"].value

    while True:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # Tear down listener and mode publisher first — neither owns
                # rclpy, so RosPublisher.close() shuts down the context last.
                if twist_listener is not None:
                    twist_listener.close()
                if mode_publisher is not None:
                    mode_publisher.close()
                if publisher is not None:
                    publisher.publish(0.0, 0.0, force=True)
                    publisher.close()
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

        # Total-time slider: any change rebuilds the runner with new scaling
        # and resets pose/trace. Inline (rather than calling on_reset) to keep
        # any console message visible while the user drags.
        new_total_time = sliders["total_time_s"].value
        if abs(new_total_time - prev_total_time) > 0.01:
            state.running = False
            rebuild_runner()
            reset_pose_and_trace()
            prev_total_time = new_total_time

        pen_body = state.geometry.pen_offset(state.pen_s_normalized * state.geometry.perimeter)

        # Drain the wire every frame so the listener buffer doesn't grow stale.
        # When the local script is running we discard the sample (avoids
        # feeding our own publish back through the subscriber); when it isn't,
        # we integrate the wire's (v, ω) into the main pose/trace — the sim
        # becomes a passive visualiser of whoever else is on /cmd_vel.
        listen_active = False
        if twist_listener is not None:
            twist_listener.spin_once(timeout_s=0.0)

        if state.running and not state.runner.done:
            if twist_listener is not None:
                twist_listener.pop_latest()  # discard self-publish echo
            for v_left, v_right, sub_dt in state.runner.consume(dt):
                # Integrate the script's intended velocities — main render shows
                # what was asked for. Limits apply only to the ROS publish, as
                # a hardware safety net (clamping the integration would mangle
                # `trace` corners, where peak |ω| spikes far above the ceiling).
                state.pose = step(state.pose, v_left, v_right, state.geometry.width, sub_dt)
                state.current_segment().append(transform_point(state.pose, *pen_body))
                v = 0.5 * (v_left + v_right)
                omega = (v_right - v_left) / state.geometry.width
                last_v, last_omega = limits.clamp_vw(v, omega)

                # Ghost integrates the *clamped* (v, ω) — the values that
                # actually go on /cmd_vel — so the user can see how the limits
                # will distort the trace on the real robot.
                half_w = 0.5 * state.geometry.width
                v_left_clamped = last_v - last_omega * half_w
                v_right_clamped = last_v + last_omega * half_w
                state.ghost_pose = step(state.ghost_pose,
                                        v_left_clamped, v_right_clamped,
                                        state.geometry.width, sub_dt)
                state.current_ghost_segment().append(
                    transform_point(state.ghost_pose, *pen_body))

                last_wheel_v = (v_left, v_right)
                last_wheel_v_clamped = (v_left_clamped, v_right_clamped)
        elif state.runner.done and state.running:
            state.running = False
            last_v, last_omega = 0.0, 0.0
            last_wheel_v = (0.0, 0.0)
            last_wheel_v_clamped = (0.0, 0.0)

        if not state.running:
            wire = twist_listener.pop_latest() if twist_listener is not None else None
            if wire is not None:
                listen_active = True
                v_wire, omega_wire = wire
                half_w = 0.5 * state.geometry.width
                v_left = v_wire - omega_wire * half_w
                v_right = v_wire + omega_wire * half_w
                state.pose = step(state.pose, v_left, v_right,
                                  state.geometry.width, dt)
                state.current_segment().append(transform_point(state.pose, *pen_body))
                last_v, last_omega = v_wire, omega_wire
                last_wheel_v = (v_left, v_right)
                last_wheel_v_clamped = (v_left, v_right)
            else:
                last_v, last_omega = 0.0, 0.0
                last_wheel_v = (0.0, 0.0)
                last_wheel_v_clamped = (0.0, 0.0)

        # Suppress the local publish when we're listening to someone else —
        # two publishers fighting over /cmd_vel would confuse downstream
        # consumers and (for our own subscriber) cause an integration loop.
        if publisher is not None and not listen_active:
            publisher.publish(last_v, last_omega)

        pen_world = transform_point(state.pose, *pen_body)
        ghost_pen_world = transform_point(state.ghost_pose, *pen_body)
        ghost_visible = limits is not NO_LIMITS

        screen.fill((20, 22, 26))
        draw_canvas_background(screen)
        if ghost_visible:
            draw_trace(screen, state.ghost_trace_segments,
                       color=COLOR_GHOST_TRACE, width=2)
        draw_trace(screen, state.trace_segments)
        if state.show_robot:
            if ghost_visible:
                draw_robot(screen, state.geometry, state.ghost_pose,
                           pen_body, ghost_pen_world, palette=PALETTE_GHOST,
                           wheel_velocities=last_wheel_v_clamped)
            draw_robot(screen, state.geometry, state.pose, pen_body, pen_world,
                       wheel_velocities=last_wheel_v)

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
        ui.draw_text(screen, font,
                     f"Cmd: v={last_v * 0.01:+.3f} m/s  ω={last_omega:+.3f} rad/s",
                     (PANEL_X + PADDING, status_y + 66), ui.COLOR_TEXT_DIM)
        if publisher is not None:
            rx = twist_listener.received_count if twist_listener else 0
            source = "wire" if listen_active else ("local" if state.running else "idle")
            ros_msg = (f"ROS {source}: {ros_topic}  pubs={publisher.published_count}  "
                       f"rx={rx}  "
                       f"lim v≤{limits.max_linear_cm_s * 0.01:.2f} m/s, "
                       f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
        elif limits is not NO_LIMITS:
            ros_msg = (f"ROS: off · limits v≤{limits.max_linear_cm_s * 0.01:.2f} m/s, "
                       f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
        else:
            ros_msg = "ROS: off · no velocity limits"
        ros_color = COLOR_OK if publisher is not None else ui.COLOR_TEXT_DIM
        ui.draw_text(screen, font, ros_msg,
                     (PANEL_X + PADDING, status_y + 88), ros_color)

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

        ui.draw_divider(screen, PANEL_X + PADDING, SLOTS_HEADER_Y - 10, INNER_W)
        slots_header = f"Remote slots → {mode_topic}"
        if slots_load_error:
            slots_header += "  (config error — see console)"
        elif mode_publisher is None:
            slots_header += "  (needs --ros)"
        ui.draw_text(screen, font, slots_header,
                     (PANEL_X + PADDING, SLOTS_HEADER_Y))

        pygame.display.flip()


if __name__ == "__main__":
    run()
