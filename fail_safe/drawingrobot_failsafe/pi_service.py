"""Pi-side daemon (fail-safe edition): listens on /robot/mode_cmd, runs
stored scripts, publishes the resulting (v, ω) to /cmd_vel.

Mode codes (kept identical to the parent so the ESP32 firmware and any
upstream Int8 source — buttons, voice, remote — keep working unchanged):
  * 80..89 → run script slot N (cancel-and-restart if one is already in flight)
  * 75..79 → set target drawing duration for the next slot launch
  * 0      → stop (publish (0, 0) forced, idle the runner)
  * other  → ignored (firmware may consume some of these on its own)

Single-thread loop. Same `CommandRunner.consume(dt)` segmentation that
`run_headless` uses, plus a ROS spin every tick to drain mode_cmd
callbacks. Mid-run cancellation: drop the runner, publish a forced
zero Twist, start a new runner from the new slot.
"""

from __future__ import annotations

import time
from pathlib import Path
from typing import Optional

from .headless import REAL_WIDTH_CM
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
from .script import CommandRunner, load_script, parse_script, rescale_runner
from .slots_config import SlotEntry, SlotsConfigError, load_slots


def _build_runner(entry: SlotEntry, wheelbase: float, limits: Limits,
                  target_duration_s: float) -> CommandRunner:
    source = load_script(entry.script)
    cmds = parse_script(source, wheelbase=wheelbase, limits=limits)
    runner = CommandRunner(cmds)
    return rescale_runner(runner, target_duration_s)


def run_pi_service(slots_path: Path | str,
                   ros_topic: str = "/cmd_vel",
                   mode_topic: str = "/robot/mode_cmd",
                   limits: Optional[Limits] = None,
                   rate_hz: float = 60.0,
                   wheelbase_cm: float = REAL_WIDTH_CM) -> None:
    if limits is None:
        limits = NO_LIMITS

    try:
        slots = load_slots(slots_path)
    except SlotsConfigError as e:
        raise SystemExit(f"[failsafe-pi-service] {e}")

    from .mode_publisher import ModeListener
    from .ros_publisher import RosPublisher

    publisher = RosPublisher(topic=ros_topic)
    listener = ModeListener(topic=mode_topic)

    print(f"[failsafe-pi-service] rate={rate_hz:g} Hz  slots_loaded={len(slots)}  "
          f"wheelbase={wheelbase_cm:.1f} cm  "
          f"limits: v≤{limits.max_linear_cm_s*0.01:.2f} m/s, "
          f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
    for idx in sorted(slots):
        e = slots[idx]
        print(f"[failsafe-pi-service]   slot {idx}: {e.script}  ({e.label})")

    runner: Optional[CommandRunner] = None
    pose = Pose(0.0, 0.0, 0.0)
    dt = 1.0 / rate_hz
    last_v = 0.0
    last_omega = 0.0
    current_slot: Optional[int] = None
    target_duration_s = DEFAULT_DRAWING_TIME_S
    print(f"[failsafe-pi-service] target_duration={target_duration_s:.0f}s "
          f"(change via /robot/mode_cmd Int8 75..79)")

    try:
        next_tick = time.monotonic()
        while True:
            listener.spin_once(timeout_s=0.0)
            code = listener.pop_request()
            if code is not None:
                if code == MODE_STOP:
                    if runner is not None or current_slot is not None:
                        print("[failsafe-pi-service] STOP — halting current run")
                    publisher.publish(0.0, 0.0, force=True)
                    runner = None
                    current_slot = None
                    pose = Pose(0.0, 0.0, 0.0)
                    last_v = last_omega = 0.0
                elif is_time_code(code):
                    target_duration_s = duration_for_code(code)
                    print(f"[failsafe-pi-service] target_duration={target_duration_s:.0f}s "
                          f"(applies to next slot launch)")
                elif is_script_slot_code(code):
                    slot = slot_for_code(code)
                    entry = slots.get(slot)
                    if entry is None:
                        print(f"[failsafe-pi-service] slot {slot} not configured; ignoring")
                    else:
                        if runner is not None:
                            print(f"[failsafe-pi-service] cancelling slot {current_slot} "
                                  f"→ starting slot {slot} ({entry.script}, "
                                  f"target={target_duration_s:.0f}s)")
                        else:
                            print(f"[failsafe-pi-service] starting slot {slot} "
                                  f"({entry.script}, target={target_duration_s:.0f}s)")
                        publisher.publish(0.0, 0.0, force=True)
                        try:
                            runner = _build_runner(entry, wheelbase_cm, limits,
                                                   target_duration_s)
                        except Exception as e:
                            print(f"[failsafe-pi-service] failed to plan slot {slot}: {e}")
                            runner = None
                            current_slot = None
                        else:
                            current_slot = slot
                            pose = Pose(0.0, 0.0, 0.0)
                            last_v = last_omega = 0.0

            if runner is not None and not runner.done:
                for v_left, v_right, sub_dt in runner.consume(dt):
                    pose = step(pose, v_left, v_right, wheelbase_cm, sub_dt)
                    v = 0.5 * (v_left + v_right)
                    omega = (v_right - v_left) / wheelbase_cm
                    last_v, last_omega = limits.clamp_vw(v, omega)
                publisher.publish(last_v, last_omega)

                if runner.done:
                    print(f"[failsafe-pi-service] slot {current_slot} complete · "
                          f"final pose x={pose.x:.2f} y={pose.y:.2f} θ={pose.theta:.3f}")
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
        print("\n[failsafe-pi-service] interrupted")
    finally:
        publisher.publish(0.0, 0.0, force=True)
        listener.close()
        publisher.close()
