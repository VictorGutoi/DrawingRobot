import time
from dataclasses import dataclass, field
from math import radians

import pygame

from .commands import CommandRunner, WheelCommand, rescale_runner
from .correction import plan_correction
from .kinematics import Pose, step, transform_point
from .limits import Limits, NO_LIMITS
from .odometry import update_from_encoders
from .robot import RobotGeometry
from .script import (
    DEFAULT_ANGULAR_SPEED_DEG, DEFAULT_SPEED, ScriptError,
    list_scripts, load_script, parse_script,
)
from .slots_config import SlotsConfigError, default_slots_path, load_slots
from . import ui


WINDOW_W, WINDOW_H = 1200, 800
# The right panel's content extends below WINDOW_H (slots/time/stop row sits
# around y≈994). We render the panel onto a tall offscreen surface of this
# virtual height and blit a vertical slice; a scroll wheel / scrollbar pages
# through it. Bumping this is the "make room for more panel widgets" knob.
PANEL_VIRTUAL_H = 1020
CANVAS_W = 800
PANEL_X = CANVAS_W
PANEL_W = WINDOW_W - CANVAS_W
PADDING = 28
INNER_W = PANEL_W - 2 * PADDING
PIXELS_PER_CM = 4.0
PANEL_SCROLL_STEP = 40       # px per mouse-wheel notch
SCROLLBAR_W = 6

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

# Encoder trace — pen path reconstructed from /sensors wheel-distance
# deltas. Ground truth of where the robot actually drew (modulo encoder
# noise and slip-during-the-tick). Diverges from both COLOR_TRACE and
# COLOR_GHOST_TRACE in the presence of wheel slip / motor lag / etc.
COLOR_ENCODER_TRACE = (220, 120, 220)

# Drop encoder corrections if /sensors goes silent for this long while
# the local runner is active — pose is stale, blind correction does more
# harm than good.
SENSOR_TIMEOUT_S = 1.0

# Suppress correction when the just-completed command's duration is below
# this. Trace mode emits ~120 Hz WheelCommands (~8 ms each); correcting
# every tick would dominate the stream and trace self-corrects
# geometrically anyway via feedback linearisation.
MIN_CORRECTION_DURATION_S = 0.25


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
    # True when the active run was triggered by a remote-slot click. The Pi
    # service will publish /cmd_vel for it; suppress local publish so we
    # don't double-source the topic.
    remote_initiated: bool = False
    # Last (v, ω) sample received on /cmd_vel; held across frames so the
    # ghost integrates every tick even when no new sample arrives that
    # frame. None = never received any sample yet (don't integrate).
    last_wire_vw: tuple[float, float] | None = None
    # Encoder-derived pose, integrated from /sensors per-wheel distance
    # deltas. Tracks where the robot actually is, not where the script
    # thinks it is.
    encoder_pose: Pose = field(default_factory=lambda: Pose(0.0, 0.0, 0.0))
    encoder_trace_segments: list[list[tuple[float, float]]] = field(
        default_factory=lambda: [[]])
    # Latched per-wheel cumulative distances (cm) from the previous
    # /sensors sample. None until first sample arrives — that sample is
    # used only as the baseline, not integrated.
    last_encoder_distances_cm: tuple[float, float] | None = None
    last_sensor_sample_t: float | None = None
    # Set to True once /sensors has been silent for SENSOR_TIMEOUT_S
    # while running. Latched: don't fight back, the run owns it now.
    corrections_disabled_stale: bool = False
    # Runner-index seen at the start of the previous tick. When the
    # current tick's index differs, at least one WheelCommand boundary
    # was crossed — the trigger to compare encoder pose vs intended pose.
    prev_runner_idx: int = 0
    # Boundaries to wait before firing the next correction. Set to 1 +
    # len(injected_correction) after an injection so the corrective
    # sequence fully executes before we re-measure drift.
    correction_cooldown_boundaries: int = 0

    def current_segment(self) -> list[tuple[float, float]]:
        return self.trace_segments[-1]

    def current_ghost_segment(self) -> list[tuple[float, float]]:
        return self.ghost_trace_segments[-1]

    def current_encoder_segment(self) -> list[tuple[float, float]]:
        return self.encoder_trace_segments[-1]

    def break_trace(self) -> None:
        if self.current_segment():
            self.trace_segments.append([])
        if self.current_ghost_segment():
            self.ghost_trace_segments.append([])
        if self.current_encoder_segment():
            self.encoder_trace_segments.append([])


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
                                  "Total run time (s)", 5.0, 120.0, 50.0, "{:.0f}"),
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


def run(ros_enabled: bool = False,
        ros_topic: str = "/cmd_vel",
        limits: Limits | None = None,
        mode_topic: str = "/robot/mode_cmd",
        slots_path: str | None = None,
        sensors_topic: str = "/sensors",
        correction_enabled: bool = True,
        correction_threshold_cm: float = 1.0,
        correction_threshold_rad: float = radians(5.0)) -> None:
    if limits is None:
        limits = NO_LIMITS

    pending_warnings: list[str] = []

    def record_warning(msg: str) -> None:
        print(msg, flush=True)
        pending_warnings.append(msg)

    publisher = None
    mode_publisher = None
    twist_listener = None
    sensor_listener = None
    if ros_enabled:
        from .ros_publisher import RosPublisher, SensorListener, TwistListener
        from .mode_publisher import (
            ModePublisher, MODE_REMOTE_DRIVE, MODE_STOP, TIME_PRESETS,
        )
        publisher = RosPublisher(topic=ros_topic)
        mode_publisher = ModePublisher(topic=mode_topic)
        twist_listener = TwistListener(topic=ros_topic)
        sensor_listener = SensorListener(topic=sensors_topic)
    else:
        # Import the mode constants unconditionally so handlers below can
        # reference them without a NameError when ROS is off.
        from .mode_publisher import MODE_REMOTE_DRIVE, MODE_STOP, TIME_PRESETS

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
    initial_runner = rescale_runner(initial_runner, 50.0)  # match slider default

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
        state.encoder_pose = Pose(0.0, 0.0, 0.0)
        state.trace_segments = [[]]
        state.ghost_trace_segments = [[]]
        state.encoder_trace_segments = [[]]
        state.last_encoder_distances_cm = None
        state.last_sensor_sample_t = None
        state.corrections_disabled_stale = False
        state.prev_runner_idx = 0
        state.correction_cooldown_boundaries = 0

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
        # Local Start = locally-driven run; restore /cmd_vel publishing.
        state.remote_initiated = False
        state.running = True
        state.break_trace()

    def on_stop():
        state.running = False
        state.break_trace()

    def on_reset():
        state.running = False
        state.remote_initiated = False
        state.last_wire_vw = None
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
        # Publishes the mode code AND starts the same script locally so the
        # sim's main car traces what the Pi service will be doing. The
        # remote_initiated flag suppresses local /cmd_vel publishing — the
        # Pi service owns that topic for this run; the ghost mirrors it.
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
            if entry is None:
                state.last_console_msg = (
                    f"published Int8({80 + slot_idx}) → {mode_topic}  [empty slot]"
                )
                state.last_console_msg_is_error = False
                return
            try:
                state.program_source = load_script(entry.script)
            except OSError as e:
                state.last_console_msg = (
                    f"published Int8({80 + slot_idx}) but local script load "
                    f"failed: {e}"
                )
                state.last_console_msg_is_error = True
                return
            rebuild_runner()
            reset_pose_and_trace()
            state.remote_initiated = True
            state.last_wire_vw = None
            state.running = True
            state.break_trace()
            state.last_console_msg = (
                f"published Int8({80 + slot_idx}) → {mode_topic}  "
                f"[{entry.script}] + local run"
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

    def make_on_time_preset(idx: int):
        # Sets the Pi service's target_duration_s for the next slot launch.
        # Also nudges the local slider so the sim preview matches.
        def handler():
            if mode_publisher is None:
                state.last_console_msg = "needs --ros to publish /robot/mode_cmd"
                state.last_console_msg_is_error = True
                return
            duration_s = TIME_PRESETS[idx]
            try:
                mode_publisher.publish_time_preset(idx)
            except Exception as e:
                state.last_console_msg = f"time publish failed: {e}"
                state.last_console_msg_is_error = True
                return
            sliders["total_time_s"].value = duration_s
            state.last_console_msg = (
                f"published Int8({75 + idx}) → {mode_topic}  [{duration_s:.0f}s]"
            )
            state.last_console_msg_is_error = False
        return handler

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
    SLOT_TIME_Y = 924
    SLOT_STOP_Y = 964
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

    # Drawing-time preset row: 5 buttons publishing Int8 75..79. The Pi
    # service applies the new target_duration on the *next* slot launch.
    time_buttons: list[ui.Button] = []
    for i, duration_s in enumerate(TIME_PRESETS):
        x = PANEL_X + PADDING + i * (slot_btn_w + SLOT_GAP)
        time_buttons.append(ui.Button(
            pygame.Rect(x, SLOT_TIME_Y, slot_btn_w, 30),
            f"{int(duration_s)}s",
            make_on_time_preset(i),
            enabled=mode_publisher is not None,
        ))

    stop_button = ui.Button(
        pygame.Rect(PANEL_X + PADDING, SLOT_STOP_Y, INNER_W, 30),
        "STOP (Int8 0)",
        on_stop_remote,
        enabled=mode_publisher is not None,
    )
    buttons.extend(slot_buttons)
    buttons.extend(time_buttons)
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
    panel_scroll_y = 0
    max_scroll = max(0, PANEL_VIRTUAL_H - WINDOW_H)

    while True:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                # Tear down listeners and mode publisher first — none of them
                # owns rclpy, so RosPublisher.close() shuts down the context last.
                if twist_listener is not None:
                    twist_listener.close()
                if sensor_listener is not None:
                    sensor_listener.close()
                if mode_publisher is not None:
                    mode_publisher.close()
                if publisher is not None:
                    publisher.publish(0.0, 0.0, force=True)
                    publisher.close()
                pygame.quit()
                return

            # Mouse wheel over the panel area pages the scroll offset; don't
            # propagate the event to widgets (no slider should be wheel-
            # scrubbable today, and capturing it avoids accidental jumps).
            if event.type == pygame.MOUSEWHEEL and max_scroll > 0:
                mx, _ = pygame.mouse.get_pos()
                if mx >= PANEL_X:
                    panel_scroll_y = max(0, min(max_scroll,
                                                panel_scroll_y - event.y * PANEL_SCROLL_STEP))
                    continue

            # Widget rects live in panel-virtual coordinates (y unshifted by
            # scroll). Translate mouse-position events to virtual coords for
            # any event aimed at the panel half of the window.
            panel_event = event
            if panel_scroll_y > 0 and hasattr(event, "pos"):
                mx, my = event.pos
                if mx >= PANEL_X:
                    attrs = {k: v for k, v in event.__dict__.items() if k != "pos"}
                    attrs["pos"] = (mx, my + panel_scroll_y)
                    panel_event = pygame.event.Event(event.type, attrs)

            console.handle_event(panel_event)
            for s in sliders.values():
                s.handle_event(panel_event)
            for b in buttons:
                b.handle_event(panel_event)
            cycler.handle_event(panel_event)

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

        # Ghost is the wire mirror: every frame, drain the listener and
        # integrate the most recently known (v, ω) sample into the ghost
        # pose/trace. Zero-order hold — without it, frames with no new sample
        # skip integration entirely while the real wheels are still spinning,
        # so the ghost under-traces by a large factor under any wire jitter.
        if twist_listener is not None:
            twist_listener.drain()
            wire = twist_listener.pop_latest()
            if wire is not None:
                state.last_wire_vw = wire
            if state.last_wire_vw is not None:
                v_wire, omega_wire = state.last_wire_vw
                half_w = 0.5 * state.geometry.width
                v_left_w = v_wire - omega_wire * half_w
                v_right_w = v_wire + omega_wire * half_w
                state.ghost_pose = step(state.ghost_pose, v_left_w, v_right_w,
                                        state.geometry.width, dt)
                state.current_ghost_segment().append(
                    transform_point(state.ghost_pose, *pen_body))
                last_wheel_v_clamped = (v_left_w, v_right_w)

        # Encoder pose: integrate per-wheel cumulative distance deltas from
        # /sensors. First sample latches the baseline only (no integration),
        # so a non-zero cumulative reading at startup doesn't slam the
        # pose to garbage. Subsequent samples integrate Δd_L, Δd_R deltas.
        if sensor_listener is not None:
            sensor_listener.drain()
            sample = sensor_listener.pop_latest()
            if sample is not None:
                _vL_mps, _vR_mps, dL_m, dR_m = sample
                dL_cm = dL_m * 100.0
                dR_cm = dR_m * 100.0
                state.last_sensor_sample_t = time.monotonic()
                if state.last_encoder_distances_cm is None:
                    state.last_encoder_distances_cm = (dL_cm, dR_cm)
                else:
                    prev_L, prev_R = state.last_encoder_distances_cm
                    state.encoder_pose = update_from_encoders(
                        state.encoder_pose, dL_cm - prev_L, dR_cm - prev_R,
                        state.geometry.width)
                    state.last_encoder_distances_cm = (dL_cm, dR_cm)
                    state.current_encoder_segment().append(
                        transform_point(state.encoder_pose, *pen_body))

        # Stale watchdog: if /sensors has gone silent for too long while the
        # local runner is active, disable corrections for the rest of the
        # run. Encoder pose freezes (no false motion). Latched until reset.
        if (sensor_listener is not None
                and state.running
                and state.last_sensor_sample_t is not None
                and not state.corrections_disabled_stale
                and (time.monotonic() - state.last_sensor_sample_t)
                    > SENSOR_TIMEOUT_S):
            state.corrections_disabled_stale = True
            state.last_console_msg = (
                f"WARN: /sensors stale > {SENSOR_TIMEOUT_S:.1f}s — "
                "corrections disabled")
            state.last_console_msg_is_error = True

        # Main car is the script mirror: only moves while the local runner
        # is active. Pose integrates the script's intended velocities;
        # limits affect only the publish path.
        cur_idx_before_consume = state.runner._idx
        last_completed_cmd_duration = (
            state.runner._commands[cur_idx_before_consume].duration
            if not state.runner.done else 0.0
        )
        if state.running and not state.runner.done:
            for v_left, v_right, sub_dt in state.runner.consume(dt):
                state.pose = step(state.pose, v_left, v_right, state.geometry.width, sub_dt)
                state.current_segment().append(transform_point(state.pose, *pen_body))
                v = 0.5 * (v_left + v_right)
                omega = (v_right - v_left) / state.geometry.width
                last_v, last_omega = limits.clamp_vw(v, omega)
                last_wheel_v = (v_left, v_right)
        elif state.runner.done and state.running:
            state.running = False
            state.remote_initiated = False
            last_v, last_omega = 0.0, 0.0
            last_wheel_v = (0.0, 0.0)

        if not state.running:
            last_v, last_omega = 0.0, 0.0
            last_wheel_v = (0.0, 0.0)

        # Boundary detection + correction injection. When the runner's
        # command index advanced this tick, a WheelCommand finished — the
        # right moment to compare encoder pose (ground truth) against
        # intended pose (state.pose) and inject a corrective sequence if
        # drift exceeds threshold.
        cur_idx_after = state.runner._idx
        if cur_idx_after != state.prev_runner_idx:
            if state.correction_cooldown_boundaries > 0:
                state.correction_cooldown_boundaries -= 1
            elif (correction_enabled
                  and sensor_listener is not None
                  and not state.corrections_disabled_stale
                  and state.last_encoder_distances_cm is not None
                  and not state.runner.done
                  and not state.remote_initiated
                  and last_completed_cmd_duration >= MIN_CORRECTION_DURATION_S):
                corr = plan_correction(
                    state.encoder_pose, state.pose,
                    linear_speed_cm_s=DEFAULT_SPEED,
                    angular_speed_rad_s=radians(DEFAULT_ANGULAR_SPEED_DEG),
                    wheelbase_cm=state.geometry.width,
                    limits=limits,
                    pos_threshold_cm=correction_threshold_cm,
                    heading_threshold_rad=correction_threshold_rad,
                )
                if corr:
                    # Teleport intended pose to encoder pose: the correction
                    # commands are designed to drive *from encoder pose to
                    # intended pose*, so when state.runner.consume yields
                    # them, integrating from encoder pose returns state.pose
                    # to the value it had a moment ago. Without the teleport
                    # we'd double-apply the correction delta.
                    state.break_trace()
                    state.pose = Pose(
                        state.encoder_pose.x,
                        state.encoder_pose.y,
                        state.encoder_pose.theta,
                    )
                    state.runner.inject(corr)
                    state.correction_cooldown_boundaries = 1 + len(corr)
            state.prev_runner_idx = cur_idx_after

        # Publish /cmd_vel only when locally driving AND this run wasn't
        # triggered by a slot click. Slot-click runs are mirrors of the Pi
        # service's run — the Pi owns /cmd_vel; we'd otherwise double-source.
        if (publisher is not None and state.running
                and not state.remote_initiated):
            publisher.publish(last_v, last_omega)

        pen_world = transform_point(state.pose, *pen_body)
        ghost_pen_world = transform_point(state.ghost_pose, *pen_body)
        # Ghost is the wire mirror; show it whenever we have a listener
        # (i.e. --ros was passed). Without --ros it would just sit at origin.
        ghost_visible = twist_listener is not None
        encoder_visible = sensor_listener is not None

        screen.fill((20, 22, 26))
        draw_canvas_background(screen)
        if ghost_visible:
            draw_trace(screen, state.ghost_trace_segments,
                       color=COLOR_GHOST_TRACE, width=2)
        if encoder_visible:
            draw_trace(screen, state.encoder_trace_segments,
                       color=COLOR_ENCODER_TRACE, width=2)
        draw_trace(screen, state.trace_segments)
        if state.show_robot:
            if ghost_visible:
                draw_robot(screen, state.geometry, state.ghost_pose,
                           pen_body, ghost_pen_world, palette=PALETTE_GHOST,
                           wheel_velocities=last_wheel_v_clamped)
            draw_robot(screen, state.geometry, state.pose, pen_body, pen_world,
                       wheel_velocities=last_wheel_v)

        # Draw the panel onto its own tall surface — same X coordinates as the
        # screen, just unconstrained in Y. Widget rects/positions are already
        # in this virtual coordinate space; we just blit a vertical slice.
        panel_target = pygame.Surface((WINDOW_W, PANEL_VIRTUAL_H))
        ui.draw_panel_background(panel_target,
                                 pygame.Rect(PANEL_X, 0, PANEL_W, PANEL_VIRTUAL_H))
        ui.draw_text(panel_target, title_font, "Drawing Robot Simulator",
                     (PANEL_X + PADDING, 28))

        for s in sliders.values():
            s.draw(panel_target, font)
        for b in buttons:
            b.draw(panel_target, font)

        status_y = 380
        status = "running" if state.running else ("done" if state.runner.done else "stopped")
        status_color = COLOR_OK if state.running else ui.COLOR_TEXT
        ui.draw_text(panel_target, font, f"Status: {status}", (PANEL_X + PADDING, status_y),
                     status_color)
        ui.draw_text(panel_target, font,
                     f"Pose: x={state.pose.x:6.1f}  y={state.pose.y:6.1f}  th={state.pose.theta:5.2f}",
                     (PANEL_X + PADDING, status_y + 22), ui.COLOR_TEXT_DIM)
        px, py = pen_body
        corner_r = (px * px + py * py) ** 0.5
        on_axis = abs(px) < 1e-6
        axis_msg = " (on axis — sharp corners possible)" if on_axis \
            else f"  off-axis dx={abs(px):.1f} cm"
        ui.draw_text(panel_target, font,
                     f"Pen swept-arc radius: {corner_r:.1f} cm{axis_msg}",
                     (PANEL_X + PADDING, status_y + 44), ui.COLOR_TEXT_DIM)
        ui.draw_text(panel_target, font,
                     f"Cmd: v={last_v * 0.01:+.3f} m/s  ω={last_omega:+.3f} rad/s",
                     (PANEL_X + PADDING, status_y + 66), ui.COLOR_TEXT_DIM)
        if publisher is not None:
            rx = twist_listener.received_count if twist_listener else 0
            if state.running and state.remote_initiated:
                drive = "remote+local"     # Pi publishes, sim mirrors locally
            elif state.running:
                drive = "local"            # sim publishes /cmd_vel
            else:
                drive = "idle"
            ros_msg = (f"ROS {drive}: {ros_topic}  pubs={publisher.published_count}  "
                       f"rx={rx}  "
                       f"lim v≤{limits.max_linear_cm_s * 0.01:.2f} m/s, "
                       f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
        elif limits is not NO_LIMITS:
            ros_msg = (f"ROS: off · limits v≤{limits.max_linear_cm_s * 0.01:.2f} m/s, "
                       f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
        else:
            ros_msg = "ROS: off · no velocity limits"
        ros_color = COLOR_OK if publisher is not None else ui.COLOR_TEXT_DIM
        ui.draw_text(panel_target, font, ros_msg,
                     (PANEL_X + PADDING, status_y + 88), ros_color)

        ui.draw_divider(panel_target, PANEL_X + PADDING, 450, INNER_W)
        ui.draw_text(panel_target, font, "Script:", (PANEL_X + PADDING, 458))
        cycler.draw(panel_target, font)

        ui.draw_divider(panel_target, PANEL_X + PADDING, 555, INNER_W)
        ui.draw_text(panel_target, font, "Console — type a command, hit Enter",
                     (PANEL_X + PADDING, 575))
        console.draw(panel_target, mono)

        if state.last_console_msg:
            color = COLOR_ERROR if state.last_console_msg_is_error else ui.COLOR_TEXT_DIM
            msg = state.last_console_msg
            if len(msg) > 48:
                msg = msg[:45] + "..."
            ui.draw_text(panel_target, mono, msg, (PANEL_X + PADDING, 644), color)

        program_y = 680
        ui.draw_text(panel_target, font, "Program preview:", (PANEL_X + PADDING, program_y))
        preview_lines = [ln for ln in state.program_source.splitlines() if ln.strip()][-5:]
        for i, line in enumerate(preview_lines):
            display = line if len(line) <= 50 else line[:47] + "..."
            ui.draw_text(panel_target, mono, display,
                         (PANEL_X + PADDING, program_y + 22 + i * 18),
                         ui.COLOR_TEXT_DIM)

        ui.draw_divider(panel_target, PANEL_X + PADDING, SLOTS_HEADER_Y - 10, INNER_W)
        slots_header = f"Remote slots → {mode_topic}"
        if slots_load_error:
            slots_header += "  (config error — see console)"
        elif mode_publisher is None:
            slots_header += "  (needs --ros)"
        ui.draw_text(panel_target, font, slots_header,
                     (PANEL_X + PADDING, SLOTS_HEADER_Y))

        # Blit the visible slice of the panel onto the screen. Only the
        # panel-x range is copied so the canvas (left half) stays untouched.
        screen.blit(panel_target, (PANEL_X, 0),
                    area=pygame.Rect(PANEL_X, panel_scroll_y, PANEL_W, WINDOW_H))

        # Scrollbar overlay on the right edge — only when there's actually
        # something to scroll to.
        if max_scroll > 0:
            track_x = WINDOW_W - SCROLLBAR_W - 2
            pygame.draw.rect(screen, (45, 45, 50),
                             (track_x, 0, SCROLLBAR_W, WINDOW_H))
            thumb_h = max(24, int(WINDOW_H * WINDOW_H / PANEL_VIRTUAL_H))
            thumb_travel = WINDOW_H - thumb_h
            thumb_y = int((panel_scroll_y / max_scroll) * thumb_travel)
            pygame.draw.rect(screen, (130, 130, 140),
                             (track_x, thumb_y, SCROLLBAR_W, thumb_h))

        pygame.display.flip()


if __name__ == "__main__":
    run()
