"""Headless runner: execute a script through the kinematics layer and publish
(v, ω) to ROS2, without the pygame UI. Designed for the Pi (or any host
without a display).

Loop at a fixed rate; for each tick, consume any wheel-command segments that
fall in this dt, clamp to limits, integrate the pose for state-tracking, and
publish the latest (v, ω) once per tick. On script completion or KeyboardInterrupt,
emit a final (0, 0) Twist before shutting down.
"""

from __future__ import annotations

import time

from .commands import CommandRunner
from .kinematics import Pose, step
from .limits import Limits, NO_LIMITS
from .robot import RobotGeometry
from .script import load_script, parse_script


REAL_WIDTH_CM = 20.4
REAL_LENGTH_CM = 23.2
REAL_WHEELS_FROM_FRONT_CM = 14.4
REAL_WHEEL_DIAMETER_CM = 6.6


def _build_geometry() -> RobotGeometry:
    wheel_offset = REAL_LENGTH_CM - REAL_WHEELS_FROM_FRONT_CM
    return RobotGeometry(
        width=REAL_WIDTH_CM,
        length=REAL_LENGTH_CM,
        wheel_offset=wheel_offset,
        wheel_diameter=REAL_WHEEL_DIAMETER_CM,
    )


def run_headless(script_name: str,
                 pen_s_normalized: float = 0.0,
                 ros_enabled: bool = True,
                 ros_topic: str = "/cmd_vel",
                 limits: Limits | None = None,
                 rate_hz: float = 60.0) -> None:
    if limits is None:
        limits = NO_LIMITS

    geometry = _build_geometry()
    pen_body = geometry.pen_offset(pen_s_normalized * geometry.perimeter)

    source = load_script(script_name)
    cmds = parse_script(source, geometry, pen_body=pen_body)
    runner = CommandRunner(cmds)

    publisher = None
    if ros_enabled:
        from .ros_publisher import RosPublisher
        publisher = RosPublisher(topic=ros_topic)

    pose = Pose(0.0, 0.0, 0.0)
    dt = 1.0 / rate_hz
    last_v = 0.0
    last_omega = 0.0

    print(f"[headless] script='{script_name}'  cmds_queued={len(cmds)}  "
          f"rate={rate_hz:g} Hz  pen_body=({pen_body[0]:.2f},{pen_body[1]:.2f}) cm")
    if publisher is not None:
        print(f"[headless] ROS publish to {publisher.topic}  "
              f"limits: v≤{limits.max_linear_cm_s*0.01:.2f} m/s, "
              f"ω≤{limits.max_angular_rad_s:.2f} rad/s")
    else:
        print("[headless] ROS publish disabled (--ros not set)")

    try:
        next_tick = time.monotonic()
        while not runner.done:
            for v_left, v_right, sub_dt in runner.consume(dt):
                # Integrate the script's intended velocities (state-tracking
                # only; clamping the integration would mangle `trace` corners).
                pose = step(pose, v_left, v_right, geometry.width, sub_dt)
                v = 0.5 * (v_left + v_right)
                omega = (v_right - v_left) / geometry.width
                last_v, last_omega = limits.clamp_vw(v, omega)

            if publisher is not None:
                publisher.publish(last_v, last_omega)

            next_tick += dt
            sleep_for = next_tick - time.monotonic()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                # Falling behind real time; resync rather than spiral.
                next_tick = time.monotonic()

        print(f"[headless] script complete · final pose "
              f"x={pose.x:.2f} y={pose.y:.2f} θ={pose.theta:.3f}")
    except KeyboardInterrupt:
        print("\n[headless] interrupted")
    finally:
        if publisher is not None:
            publisher.publish(0.0, 0.0, force=True)
            publisher.close()
