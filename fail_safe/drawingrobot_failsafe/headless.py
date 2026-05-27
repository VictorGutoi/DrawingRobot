"""Headless runner: parse a script, integrate kinematics, publish (v, ω) to
ROS2. No pygame. Useful on a Pi or any host without a display.

Loop at a fixed rate; for each tick, consume any wheel-command segments
that fall in this dt, clamp to limits, integrate the pose for state-
tracking, and publish the latest (v, ω) once per tick. On script
completion or KeyboardInterrupt, emit a final (0, 0) Twist before
shutting down.
"""

from __future__ import annotations

import time

from .kinematics import Pose, step
from .limits import Limits, NO_LIMITS
from .script import CommandRunner, load_script, parse_script, rescale_runner


# Real-robot specs (cm). Fail-safe pins width because nothing in the script
# layer depends on length; the simulator uses these as render defaults but
# headless only needs wheelbase = width.
REAL_WIDTH_CM = 20.4


def run_headless(script_name: str,
                 ros_enabled: bool = True,
                 ros_topic: str = "/cmd_vel",
                 limits: Limits | None = None,
                 rate_hz: float = 60.0,
                 wheelbase_cm: float = REAL_WIDTH_CM,
                 target_duration_s: float | None = None) -> None:
    if limits is None:
        limits = NO_LIMITS

    source = load_script(script_name)
    cmds = parse_script(source, wheelbase=wheelbase_cm, limits=limits)
    runner = CommandRunner(cmds)
    if target_duration_s is not None and target_duration_s > 0:
        runner = rescale_runner(runner, target_duration_s)

    publisher = None
    if ros_enabled:
        from .ros_publisher import RosPublisher
        publisher = RosPublisher(topic=ros_topic)

    pose = Pose(0.0, 0.0, 0.0)
    dt = 1.0 / rate_hz
    last_v = 0.0
    last_omega = 0.0

    print(f"[failsafe-headless] script='{script_name}'  cmds_queued={len(cmds)}  "
          f"wheelbase={wheelbase_cm:.1f} cm  rate={rate_hz:g} Hz")
    if publisher is not None:
        print(f"[failsafe-headless] ROS publish to {publisher.topic}  "
              f"limits: v≤{limits.max_linear_cm_s*0.01:.2f} m/s, "
              f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
    else:
        print("[failsafe-headless] ROS publish disabled (--ros not set)")

    try:
        next_tick = time.monotonic()
        while not runner.done:
            for v_left, v_right, sub_dt in runner.consume(dt):
                pose = step(pose, v_left, v_right, wheelbase_cm, sub_dt)
                v = 0.5 * (v_left + v_right)
                omega = (v_right - v_left) / wheelbase_cm
                last_v, last_omega = limits.clamp_vw(v, omega)

            if publisher is not None:
                publisher.publish(last_v, last_omega)

            next_tick += dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.monotonic()

        print(f"[failsafe-headless] script complete · final pose "
              f"x={pose.x:.2f} y={pose.y:.2f} θ={pose.theta:.3f}")
    except KeyboardInterrupt:
        print("\n[failsafe-headless] interrupted")
    finally:
        if publisher is not None:
            publisher.publish(0.0, 0.0, force=True)
            publisher.close()
