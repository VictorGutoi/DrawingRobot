"""Fail-safe pygame simulator.

Mirrors the parent sim's layout (canvas left, panel right) but trims:
  * no pen-perimeter slider — pen is pinned to the left wheel
  * no ghost / wire-mirror trace — the publisher is one-way here
  * no wheels-from-front slider — chassis is rendered with the wheels at
    the middle of the long edges; the wheel position relative to the
    chassis outline is cosmetic in the fail-safe (pen sits at the wheel)
  * no `arc` / `goto` / `line_to` / `trace` — three commands only

Keeps: width/length sliders, Start/Stop/Reset, script cycler, slot
buttons (so the sim can drive the Pi service over /robot/mode_cmd
identically to the parent), drawing-time presets, console.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from math import radians

import pygame

from .correction import plan_correction
from .kinematics import Pose, step, transform_point
from .limits import Limits, NO_LIMITS
from .odometry import update_from_encoders
from .script import (
    DEFAULT_ANGULAR_SPEED_DEG,
    DEFAULT_SPEED,
    CommandRunner,
    ScriptError,
    WheelCommand,
    list_scripts,
    load_script,
    parse_script,
    rescale_runner,
)
from .slots_config import SlotsConfigError, default_slots_path, load_slots
from . import ui


WINDOW_W, WINDOW_H = 1200, 990
CANVAS_W = 800
PANEL_X = CANVAS_W
PANEL_W = WINDOW_W - CANVAS_W
PADDING = 28
INNER_W = PANEL_W - 2 * PADDING
PIXELS_PER_CM = 4.0

# Real-robot specs (cm). Initial slider values.
REAL_WIDTH_CM = 20.4
REAL_LENGTH_CM = 23.2
REAL_WHEEL_DIAMETER_CM = 6.6

COLOR_CANVAS = (245, 245, 240)
COLOR_GRID = (220, 220, 215)
COLOR_AXIS = (180, 180, 175)
COLOR_TRACE = (40, 80, 200)
COLOR_CHASSIS = (60, 60, 70)
COLOR_WHEEL = (20, 20, 25)
COLOR_PEN = (220, 50, 50)
COLOR_LEFT_WHEEL = (220, 50, 50)
COLOR_HEADING = (100, 160, 100)
COLOR_ERROR = (220, 110, 110)
COLOR_OK = (140, 200, 140)
COLOR_WHEEL_AXIS = (180, 100, 200)

# Encoder trace — pen path reconstructed from /sensors wheel-distance
# deltas. Ground truth of where the robot actually drew (modulo encoder
# noise / slip during the tick). Diverges from COLOR_TRACE under wheel
# slip, motor lag, or a wrong wheelbase.
COLOR_ENCODER_TRACE = (220, 120, 220)

# Drop encoder corrections if /sensors goes silent for this long while the
# local runner is active — pose is stale, blind correction does more harm
# than good.
SENSOR_TIMEOUT_S = 1.0

# Suppress correction when the just-completed command's duration is below
# this. Keeps corrections at real corners, not every sub-second segment.
MIN_CORRECTION_DURATION_S = 0.25


@dataclass(frozen=True)
class FailSafeGeometry:
    """Pure-rendering geometry. The kinematics layer only needs width."""
    width: float       # wheelbase, cm
    length: float      # chassis length, cm
    wheel_diameter: float = REAL_WHEEL_DIAMETER_CM

    @property
    def half_width(self) -> float:
        return self.width / 2

    @property
    def front_x(self) -> float:
        # Render chassis with the wheel axis at the midline of the chassis;
        # not load-bearing for kinematics, only for the drawn outline.
        return self.length / 2

    @property
    def back_x(self) -> float:
        return -self.length / 2

    def chassis_corners(self) -> list[tuple[float, float]]:
        h = self.half_width
        return [
            (self.back_x, -h),
            (self.front_x, -h),
            (self.front_x, h),
            (self.back_x, h),
        ]


@dataclass
class SimState:
    geometry: FailSafeGeometry
    pose: Pose
    runner: CommandRunner
    program_source: str = ""
    running: bool = False
    trace_segments: list[list[tuple[float, float]]] = field(
        default_factory=lambda: [[]])
    last_console_msg: str = ""
    last_console_msg_is_error: bool = False
    show_robot: bool = True
    remote_initiated: bool = False
    # Encoder-derived pose, integrated from /sensors per-wheel distance
    # deltas. Tracks where the robot actually is, not where the script
    # thinks it is.
    encoder_pose: Pose = field(default_factory=lambda: Pose(0.0, 0.0, 0.0))
    encoder_trace_segments: list[list[tuple[float, float]]] = field(
        default_factory=lambda: [[]])
    # Latched per-wheel cumulative distances (cm) from the previous
    # /sensors sample. None until the first sample, which sets the baseline
    # only (not integrated).
    last_encoder_distances_cm: tuple[float, float] | None = None
    last_sensor_sample_t: float | None = None
    # Latched once /sensors has been silent for SENSOR_TIMEOUT_S while
    # running — corrections disabled for the rest of the run.
    corrections_disabled_stale: bool = False
    # Runner-index seen last tick; a change means a WheelCommand boundary
    # was crossed (the trigger to measure drift and maybe correct).
    prev_runner_idx: int = 0
    # Boundaries to skip before the next correction, so an injected
    # corrective sequence fully executes before we re-measure.
    correction_cooldown_boundaries: int = 0

    def current_segment(self) -> list[tuple[float, float]]:
        return self.trace_segments[-1]

    def current_encoder_segment(self) -> list[tuple[float, float]]:
        return self.encoder_trace_segments[-1]

    def break_trace(self) -> None:
        if self.current_segment():
            self.trace_segments.append([])
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


def draw_trace(surface: pygame.Surface,
               segments: list[list[tuple[float, float]]],
               color: tuple[int, int, int] = COLOR_TRACE,
               width: int = 2) -> None:
    for seg in segments:
        if len(seg) < 2:
            continue
        pts = [world_to_screen(x, y) for x, y in seg]
        pygame.draw.lines(surface, color, False, pts, width)


def draw_robot(surface: pygame.Surface, geometry: FailSafeGeometry, pose: Pose,
               pen_world: tuple[float, float]) -> None:
    corners = [transform_point(pose, bx, by) for bx, by in geometry.chassis_corners()]
    screen_corners = [world_to_screen(x, y) for x, y in corners]
    pygame.draw.polygon(surface, COLOR_CHASSIS, screen_corners, 2)

    h = geometry.half_width
    axis_a = transform_point(pose, 0.0, -h)
    axis_b = transform_point(pose, 0.0, h)
    pygame.draw.line(surface, COLOR_WHEEL_AXIS,
                     world_to_screen(*axis_a), world_to_screen(*axis_b), 1)

    # Wheels rendered as short line segments along the rolling axis (length =
    # wheel_diameter). Left wheel is highlighted as the pen carrier.
    diameter = geometry.wheel_diameter if geometry.wheel_diameter > 0 \
        else geometry.length * 0.25
    t = diameter / 2
    for body_y, color in ((h, COLOR_LEFT_WHEEL), (-h, COLOR_WHEEL)):
        wx0, wy0 = transform_point(pose, -t, body_y)
        wx1, wy1 = transform_point(pose, t, body_y)
        pygame.draw.line(surface, color, world_to_screen(wx0, wy0),
                         world_to_screen(wx1, wy1), 6)

    heading_tip = transform_point(pose, geometry.front_x, 0.0)
    pygame.draw.line(surface, COLOR_HEADING,
                     world_to_screen(pose.x, pose.y),
                     world_to_screen(*heading_tip), 2)

    pygame.draw.circle(surface, COLOR_PEN, world_to_screen(*pen_world), 5)


def make_sliders() -> dict[str, ui.Slider]:
    x = PANEL_X + PADDING
    return {
        "width": ui.Slider(pygame.Rect(x, 95, INNER_W, 6), "Width (cm)",
                           8.0, 40.0, REAL_WIDTH_CM, "{:.1f}"),
        "length": ui.Slider(pygame.Rect(x, 150, INNER_W, 6), "Length (cm)",
                            8.0, 50.0, REAL_LENGTH_CM, "{:.1f}"),
        "total_time_s": ui.Slider(pygame.Rect(x, 205, INNER_W, 6),
                                  "Total run time (s)", 5.0, 120.0, 50.0, "{:.0f}"),
    }


def build_geometry(width: float, length: float) -> FailSafeGeometry:
    return FailSafeGeometry(width=width, length=length,
                            wheel_diameter=REAL_WHEEL_DIAMETER_CM)


def build_runner(source: str, wheelbase: float, limits: Limits = NO_LIMITS,
                 on_warning=print) -> tuple[CommandRunner, str]:
    """Returns (runner, error_message). On parse error, runner is empty."""
    try:
        cmds = parse_script(source, wheelbase=wheelbase, limits=limits,
                            on_warning=on_warning)
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
    sensor_listener = None
    if ros_enabled:
        from .ros_publisher import RosPublisher, SensorListener
        from .mode_publisher import ModePublisher, MODE_STOP, TIME_PRESETS
        publisher = RosPublisher(topic=ros_topic)
        mode_publisher = ModePublisher(topic=mode_topic)
        sensor_listener = SensorListener(topic=sensors_topic)
    else:
        from .mode_publisher import MODE_STOP, TIME_PRESETS  # for handler use

    slots_load_error: str | None = None
    if slots_path is None:
        slots_path = str(default_slots_path())
    try:
        slots = load_slots(slots_path)
    except SlotsConfigError as e:
        slots = {}
        slots_load_error = f"slots: {e}"
        print(f"[failsafe-sim] {slots_load_error}", flush=True)

    pygame.init()
    pygame.display.set_caption("Drawing Robot Simulator — Fail-Safe")
    screen = pygame.display.set_mode((WINDOW_W, WINDOW_H))
    clock = pygame.time.Clock()
    font = pygame.font.SysFont("Helvetica", 14)
    title_font = pygame.font.SysFont("Helvetica", 18, bold=True)
    mono = pygame.font.SysFont("Menlo,Courier", 13)

    geometry = build_geometry(REAL_WIDTH_CM, REAL_LENGTH_CM)
    available_scripts = list_scripts()
    initial_name = "square" if "square" in available_scripts else (
        available_scripts[0] if available_scripts else "")
    initial_source = load_script(initial_name) if initial_name else ""
    initial_runner, _ = build_runner(initial_source, geometry.width,
                                     limits=limits, on_warning=record_warning)
    initial_runner = rescale_runner(initial_runner, 50.0)

    state = SimState(
        geometry=geometry,
        pose=Pose(0.0, 0.0, 0.0),
        runner=initial_runner,
        program_source=initial_source,
    )

    sliders = make_sliders()

    def pen_body() -> tuple[float, float]:
        return (0.0, state.geometry.half_width)

    def reset_pose_and_trace():
        state.pose = Pose(0.0, 0.0, 0.0)
        state.trace_segments = [[]]
        state.encoder_pose = Pose(0.0, 0.0, 0.0)
        state.encoder_trace_segments = [[]]
        state.last_encoder_distances_cm = None
        state.last_sensor_sample_t = None
        state.corrections_disabled_stale = False
        state.prev_runner_idx = 0
        state.correction_cooldown_boundaries = 0

    def rebuild_runner():
        pending_warnings.clear()
        state.runner, err = build_runner(
            state.program_source, state.geometry.width,
            limits=limits, on_warning=record_warning)
        state.runner = rescale_runner(state.runner, sliders["total_time_s"].value)
        if err:
            state.last_console_msg = err
            state.last_console_msg_is_error = True
        elif pending_warnings:
            state.last_console_msg = pending_warnings[-1]
            state.last_console_msg_is_error = False
        elif state.last_console_msg_is_error:
            # Previous parse failed; this one succeeded — clear the stale
            # error so the user knows the buffer is valid again.
            state.last_console_msg = ""
            state.last_console_msg_is_error = False

    def on_start():
        if state.runner.done:
            rebuild_runner()
            reset_pose_and_trace()
        state.remote_initiated = False
        state.running = True
        state.break_trace()

    def on_stop():
        state.running = False
        state.break_trace()

    def on_reset():
        state.running = False
        state.remote_initiated = False
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
        # Push the loaded source into the editor without firing on_change
        # (we already rebuild below).
        editor.set_text(state.program_source, fire_change=False)
        rebuild_runner()
        reset_pose_and_trace()
        state.running = False
        state.last_console_msg = f"loaded {name}"
        state.last_console_msg_is_error = False

    def make_on_slot(slot_idx: int):
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
                line, wheelbase=state.geometry.width,
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

    # ---------- panel widgets ----------
    btn_y = 260
    btn_w = (INNER_W - 24) // 4

    def make_toggle_btn():
        return ui.Button(
            pygame.Rect(PANEL_X + PADDING + 3 * (btn_w + 8), btn_y, btn_w, 36),
            "Hide Robot", lambda: None)

    toggle_robot_btn = make_toggle_btn()

    def on_toggle_robot():
        state.show_robot = not state.show_robot
        toggle_robot_btn.label = "Show Robot" if not state.show_robot else "Hide Robot"

    toggle_robot_btn.on_click = on_toggle_robot

    buttons = [
        ui.Button(pygame.Rect(PANEL_X + PADDING, btn_y, btn_w, 36), "Start", on_start),
        ui.Button(pygame.Rect(PANEL_X + PADDING + btn_w + 8, btn_y, btn_w, 36),
                  "Stop", on_stop),
        ui.Button(pygame.Rect(PANEL_X + PADDING + 2 * (btn_w + 8), btn_y, btn_w, 36),
                  "Reset", on_reset),
        toggle_robot_btn,
    ]

    cycler = ui.Cycler(
        rect=pygame.Rect(PANEL_X + PADDING, 385, INNER_W, 32),
        options=available_scripts,
        on_change=on_select_script,
        index=available_scripts.index(initial_name) if initial_name in available_scripts else 0,
    )

    console = ui.TextInput(
        rect=pygame.Rect(PANEL_X + PADDING, 458, INNER_W, 32),
        placeholder="forward 20 / turn 90 / circle 15",
        on_submit=on_console_submit,
    )

    def on_editor_change(new_text: str) -> None:
        # Live rebuild: every keystroke re-parses. Parse errors are surfaced
        # in the console message line; the previous runner is replaced with
        # an empty one so the user doesn't accidentally run stale commands.
        state.program_source = new_text
        rebuild_runner()
        # Don't auto-reset the pose mid-edit; user can hit Reset when ready.

    editor = ui.TextArea(
        rect=pygame.Rect(PANEL_X + PADDING, 532, INNER_W, 140),
        text=initial_source,
        on_change=on_editor_change,
    )

    # Slot grid mirrors parent layout: 2x5 slot buttons + time preset row + STOP.
    SLOTS_HEADER_Y = 690
    SLOT_ROW1_Y = 716
    SLOT_ROW2_Y = 756
    SLOT_TIME_Y = 804
    SLOT_STOP_Y = 844
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

    last_v = 0.0
    last_omega = 0.0
    prev_total_time = sliders["total_time_s"].value

    # ---------- main loop ----------
    while True:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                if sensor_listener is not None:
                    sensor_listener.close()
                if mode_publisher is not None:
                    mode_publisher.close()
                if publisher is not None:
                    publisher.publish(0.0, 0.0, force=True)
                    publisher.close()
                pygame.quit()
                return
            console.handle_event(event)
            editor.handle_event(event, mono)
            for s in sliders.values():
                s.handle_event(event)
            for b in buttons:
                b.handle_event(event)
            cycler.handle_event(event)

        new_geometry = build_geometry(
            sliders["width"].value,
            sliders["length"].value,
        )
        prev_geom = (state.geometry.width, state.geometry.length)
        state.geometry = new_geometry
        new_geom = (new_geometry.width, new_geometry.length)
        if new_geom != prev_geom:
            state.break_trace()

        new_total_time = sliders["total_time_s"].value
        if abs(new_total_time - prev_total_time) > 0.01:
            state.running = False
            rebuild_runner()
            reset_pose_and_trace()
            prev_total_time = new_total_time

        # Encoder pose: integrate per-wheel cumulative distance deltas from
        # /sensors. The first sample latches the baseline only (no
        # integration) so a non-zero startup reading doesn't slam the pose;
        # later samples integrate Δd_L, Δd_R.
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
                        transform_point(state.encoder_pose, *pen_body()))

        # Stale watchdog: if /sensors goes silent while running, disable
        # corrections for the rest of the run (encoder pose just freezes).
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

        # Duration of the command about to finish (for the sub-second skip).
        idx_before = state.runner._idx
        last_completed_cmd_duration = (
            state.runner._commands[idx_before].duration
            if not state.runner.done else 0.0
        )

        if state.running and not state.runner.done:
            for v_left, v_right, sub_dt in state.runner.consume(dt):
                state.pose = step(state.pose, v_left, v_right,
                                  state.geometry.width, sub_dt)
                state.current_segment().append(
                    transform_point(state.pose, *pen_body()))
                v = 0.5 * (v_left + v_right)
                omega = (v_right - v_left) / state.geometry.width
                last_v, last_omega = limits.clamp_vw(v, omega)
        elif state.runner.done and state.running:
            state.running = False
            state.remote_initiated = False
            last_v, last_omega = 0.0, 0.0

        if not state.running:
            last_v, last_omega = 0.0, 0.0

        # Boundary detection + correction injection. When the runner's
        # command index advanced this tick, a WheelCommand finished — the
        # moment to compare encoder pose (ground truth) against the
        # intended pose (state.pose) and inject a corrective sequence if
        # drift exceeds threshold.
        idx_after = state.runner._idx
        if idx_after != state.prev_runner_idx:
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
                    # commands drive *from encoder pose to intended pose*, so
                    # when consume() yields them, integrating from the encoder
                    # pose returns state.pose to where it just was. Without the
                    # teleport we'd double-apply the correction delta. Break
                    # the trace so the teleport doesn't draw a phantom line.
                    state.break_trace()
                    state.pose = Pose(state.encoder_pose.x,
                                      state.encoder_pose.y,
                                      state.encoder_pose.theta)
                    state.runner.inject(corr)
                    state.correction_cooldown_boundaries = 1 + len(corr)
            state.prev_runner_idx = idx_after

        # Publish /cmd_vel only when locally driving AND not initiated via a
        # slot click (the Pi service owns /cmd_vel in that case; the sim is
        # just mirroring locally so the user sees what the Pi will draw).
        if (publisher is not None and state.running
                and not state.remote_initiated):
            publisher.publish(last_v, last_omega)

        pen_world = transform_point(state.pose, *pen_body())

        screen.fill((20, 22, 26))
        draw_canvas_background(screen)
        if sensor_listener is not None:
            draw_trace(screen, state.encoder_trace_segments,
                       color=COLOR_ENCODER_TRACE, width=2)
        draw_trace(screen, state.trace_segments)
        if state.show_robot:
            draw_robot(screen, state.geometry, state.pose, pen_world)

        ui.draw_panel_background(screen, pygame.Rect(PANEL_X, 0, PANEL_W, WINDOW_H))
        ui.draw_text(screen, title_font, "Drawing Robot — Fail-Safe",
                     (PANEL_X + PADDING, 28))
        ui.draw_text(screen, font, "Pen at LEFT wheel · 3 commands: forward / turn / circle",
                     (PANEL_X + PADDING, 52), ui.COLOR_TEXT_DIM)

        for s in sliders.values():
            s.draw(screen, font)
        for b in buttons:
            b.draw(screen, font)

        status_y = 308
        status = "running" if state.running else ("done" if state.runner.done else "stopped")
        status_color = COLOR_OK if state.running else ui.COLOR_TEXT
        ui.draw_text(screen, font, f"Status: {status}",
                     (PANEL_X + PADDING, status_y), status_color)
        ui.draw_text(screen, font,
                     f"Pose: x={state.pose.x:6.1f}  y={state.pose.y:6.1f}  th={state.pose.theta:5.2f}",
                     (PANEL_X + PADDING, status_y + 18), ui.COLOR_TEXT_DIM)
        if publisher is not None:
            drive = "remote+local" if state.remote_initiated else (
                "local" if state.running else "idle")
            rx = sensor_listener.received_count if sensor_listener else 0
            ros_msg = (f"ROS {drive}: {ros_topic}  pubs={publisher.published_count}  "
                       f"/sensors rx={rx}  "
                       f"lim v≤{limits.max_linear_cm_s * 0.01:.2f} m/s, "
                       f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
        elif limits is not NO_LIMITS:
            ros_msg = (f"ROS: off · limits v≤{limits.max_linear_cm_s * 0.01:.2f} m/s, "
                       f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
        else:
            ros_msg = "ROS: off · no velocity limits"
        ros_color = COLOR_OK if publisher is not None else ui.COLOR_TEXT_DIM
        ui.draw_text(screen, font, ros_msg, (PANEL_X + PADDING, status_y + 36), ros_color)
        if sensor_listener is not None:
            if state.corrections_disabled_stale:
                corr_txt = "correction: DISABLED (/sensors stale)"
                corr_color = COLOR_ERROR
            elif not correction_enabled:
                corr_txt = "correction: off (--no-correction) · encoder ghost only"
                corr_color = ui.COLOR_TEXT_DIM
            else:
                ex = state.encoder_pose.x - state.pose.x
                ey = state.encoder_pose.y - state.pose.y
                drift = (ex * ex + ey * ey) ** 0.5
                corr_txt = (f"correction: on · drift {drift:4.1f} cm "
                            f"(thr {correction_threshold_cm:.1f} cm)")
                corr_color = ui.COLOR_TEXT_DIM
            ui.draw_text(screen, font, corr_txt,
                         (PANEL_X + PADDING, status_y + 54), corr_color)

        ui.draw_divider(screen, PANEL_X + PADDING, 365, INNER_W)
        ui.draw_text(screen, font, "Script:", (PANEL_X + PADDING, 373))
        cycler.draw(screen, font)

        ui.draw_divider(screen, PANEL_X + PADDING, 415, INNER_W)
        ui.draw_text(screen, font, "Console — type a command, hit Enter",
                     (PANEL_X + PADDING, 433))
        console.draw(screen, mono)

        if state.last_console_msg:
            color = COLOR_ERROR if state.last_console_msg_is_error else ui.COLOR_TEXT_DIM
            msg = state.last_console_msg
            if len(msg) > 48:
                msg = msg[:45] + "..."
            ui.draw_text(screen, mono, msg, (PANEL_X + PADDING, 496), color)

        ui.draw_text(screen, font, "Script editor — edit and watch the trace update:",
                     (PANEL_X + PADDING, 514))
        editor.draw(screen, mono)

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
