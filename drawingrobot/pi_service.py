"""Pi-side daemon: listens on /robot/mode_cmd, runs stored scripts, publishes
the resulting (v, ω) to /cmd_vel.

Mode codes (see `mode_publisher`):
  * 80..89 → run script slot N (cancel-and-restart if one is already in flight)
  * 0      → stop (publish (0, 0) forced, idle the runner)
  * other  → ignored (firmware may consume some of these on its own)

Single-thread loop. The same `CommandRunner.consume(dt)` segmentation that
`headless.run_headless` uses, plus a ROS spin every tick to drain mode_cmd
callbacks. Mid-run cancellation is just "drop the runner, publish a forced
zero Twist, start a new runner from the new slot".
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from .commands import CommandRunner, rescale_runner
from .headless import _build_geometry
from .kinematics import Pose, step
from .limits import Limits, NO_LIMITS
from .mode_publisher import (
    DEFAULT_DRAWING_TIME_S,
    MODE_STOP,
    duration_for_code,
    is_script_slot_code,
    is_time_code,
    slot_for_code,
)
from .script import load_script, parse_script
from .slots_config import SlotEntry, SlotsConfigError, load_slots


def _build_runner(entry: SlotEntry, geometry, limits: Limits,
                  target_duration_s: float) -> CommandRunner:
    pen_body = geometry.pen_offset(entry.pen_s * geometry.perimeter)
    source = load_script(entry.script)
    cmds = parse_script(source, geometry, pen_body=pen_body, limits=limits)
    runner = CommandRunner(cmds)
    return rescale_runner(runner, target_duration_s)


def run_pi_service(slots_path: Path | str,
                   ros_topic: str = "/cmd_vel",
                   mode_topic: str = "/robot/mode_cmd",
                   limits: Optional[Limits] = None,
                   rate_hz: float = 60.0) -> None:
    if limits is None:
        limits = NO_LIMITS

    try:
        slots = load_slots(slots_path)
    except SlotsConfigError as e:
        raise SystemExit(f"[pi-service] {e}")

    geometry = _build_geometry()

    # Lazy imports so this module is importable without ROS2.
    from .mode_publisher import ModeListener
    from .ros_publisher import RosPublisher

    publisher = RosPublisher(topic=ros_topic)
    listener = ModeListener(topic=mode_topic)

    print(f"[pi-service] rate={rate_hz:g} Hz  slots_loaded={len(slots)}  "
          f"limits: v≤{limits.max_linear_cm_s*0.01:.2f} m/s, "
          f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
    for idx in sorted(slots):
        e = slots[idx]
        print(f"[pi-service]   slot {idx}: {e.script}  (pen_s={e.pen_s:.3f})")

    runner: Optional[CommandRunner] = None
    pose = Pose(0.0, 0.0, 0.0)
    dt = 1.0 / rate_hz
    last_v = 0.0
    last_omega = 0.0
    current_slot: Optional[int] = None
    target_duration_s = DEFAULT_DRAWING_TIME_S
    print(f"[pi-service] target_duration={target_duration_s:.0f}s "
          f"(change via /robot/mode_cmd Int8 75..79)")

    try:
        next_tick = time.monotonic()
        while True:
            listener.spin_once(timeout_s=0.0)
            code = listener.pop_request()
            if code is not None:
                if code == MODE_STOP:
                    if runner is not None or current_slot is not None:
                        print("[pi-service] STOP — halting current run")
                    publisher.publish(0.0, 0.0, force=True)
                    runner = None
                    current_slot = None
                    pose = Pose(0.0, 0.0, 0.0)
                    last_v = last_omega = 0.0
                elif is_time_code(code):
                    target_duration_s = duration_for_code(code)
                    print(f"[pi-service] target_duration={target_duration_s:.0f}s "
                          f"(applies to next slot launch)")
                elif is_script_slot_code(code):
                    slot = slot_for_code(code)
                    entry = slots.get(slot)
                    if entry is None:
                        print(f"[pi-service] slot {slot} not configured; ignoring")
                    else:
                        if runner is not None:
                            print(f"[pi-service] cancelling slot {current_slot} "
                                  f"→ starting slot {slot} ({entry.script}, "
                                  f"target={target_duration_s:.0f}s)")
                        else:
                            print(f"[pi-service] starting slot {slot} ({entry.script}, "
                                  f"target={target_duration_s:.0f}s)")
                        publisher.publish(0.0, 0.0, force=True)
                        try:
                            runner = _build_runner(entry, geometry, limits,
                                                   target_duration_s)
                        except Exception as e:
                            print(f"[pi-service] failed to plan slot {slot}: {e}")
                            runner = None
                            current_slot = None
                        else:
                            current_slot = slot
                            pose = Pose(0.0, 0.0, 0.0)
                            last_v = last_omega = 0.0
                # else: not for us, firmware handles it.

            if runner is not None and not runner.done:
                for v_left, v_right, sub_dt in runner.consume(dt):
                    pose = step(pose, v_left, v_right, geometry.width, sub_dt)
                    v = 0.5 * (v_left + v_right)
                    omega = (v_right - v_left) / geometry.width
                    last_v, last_omega = limits.clamp_vw(v, omega)
                publisher.publish(last_v, last_omega)

                if runner.done:
                    print(f"[pi-service] slot {current_slot} complete · final pose "
                          f"x={pose.x:.2f} y={pose.y:.2f} θ={pose.theta:.3f}")
                    publisher.publish(0.0, 0.0, force=True)
                    runner = None
                    current_slot = None
                    last_v = last_omega = 0.0
            else:
                publisher.publish(0.0, 0.0)  # heartbeat throttled internally

            next_tick += dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()
    except KeyboardInterrupt:
        print("\n[pi-service] interrupted")
    finally:
        publisher.publish(0.0, 0.0, force=True)
        listener.close()
        publisher.close()
