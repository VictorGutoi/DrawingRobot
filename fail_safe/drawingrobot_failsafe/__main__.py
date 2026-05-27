"""Fail-safe drawing-robot CLI.

By default opens the pygame simulator. `--ros` also publishes (v, ω) to a
ROS2 Twist topic. `--headless` runs a script through kinematics + ROS
publisher with no UI (useful on a Pi). `--pi-service` runs the Pi-side
daemon (subscribe /robot/mode_cmd, play stored scripts by slot).

Mode codes (80..89 for slots, 75..79 for time presets, 0 for stop) match
the parent package, so the same Int8 source (button board, voice control,
remote) drives the fail-safe service identically.
"""

import argparse

from .limits import Limits


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="drawingrobot_failsafe",
        description=__doc__,
    )
    parser.add_argument("--ros", action="store_true",
                        help="Publish the commanded body velocity to a ROS2 topic.")
    parser.add_argument("--ros-topic", default="/cmd_vel",
                        help="ROS2 topic name (default: /cmd_vel).")
    parser.add_argument("--mode-topic", default="/robot/mode_cmd",
                        help="ROS2 topic for the LULOC2 mode-set button "
                             "(std_msgs/Int8, default: /robot/mode_cmd). "
                             "Requires --ros for the sim's slot buttons.")
    parser.add_argument("--max-linear", type=float, default=0.5,
                        metavar="M_S",
                        help="Linear-speed ceiling in m/s (default: 0.5). "
                             "Applied to both the simulator and the ROS publish.")
    parser.add_argument("--max-angular", type=float, default=2.0,
                        metavar="RAD_S",
                        help="Angular-speed ceiling in rad/s (default: 2.0). "
                             "Higher than the parent's 0.5 to accommodate the "
                             "default `turn` (~π rad/s) and small `circle` radii.")
    parser.add_argument("--headless", action="store_true",
                        help="Run a script with no pygame UI; integrates "
                             "kinematics + publishes ROS2. Useful on a Pi.")
    parser.add_argument("--pi-service", action="store_true",
                        help="Run the Pi-side daemon: subscribe to "
                             "/robot/mode_cmd, play stored scripts by slot "
                             "(codes 80..89), publish /cmd_vel. Mutually "
                             "exclusive with --headless.")
    parser.add_argument("--slots-config", default=None,
                        metavar="PATH",
                        help="Path to the pi_slots.json mapping "
                             "(default: fail_safe/pi_slots.json).")
    parser.add_argument("--script", default="square",
                        help="Script name (in fail_safe/scripts/) for --headless "
                             "mode (default: square).")
    parser.add_argument("--rate", type=float, default=60.0,
                        metavar="HZ",
                        help="Headless / Pi-service loop rate in Hz (default: 60).")
    parser.add_argument("--target-duration", type=float, default=None,
                        metavar="S",
                        help="Uniformly rescale the script to take this many "
                             "seconds (default: no rescale). Headless mode only.")
    args = parser.parse_args()

    if args.headless and args.pi_service:
        parser.error("--headless and --pi-service are mutually exclusive")

    limits = Limits(
        max_linear_cm_s=args.max_linear * 100.0,
        max_angular_rad_s=args.max_angular,
    )

    from .slots_config import default_slots_path
    slots_path = args.slots_config or str(default_slots_path())

    if args.pi_service:
        from .pi_service import run_pi_service
        run_pi_service(
            slots_path=slots_path,
            ros_topic=args.ros_topic,
            mode_topic=args.mode_topic,
            limits=limits,
            rate_hz=args.rate,
        )
    elif args.headless:
        from .headless import run_headless
        run_headless(
            script_name=args.script,
            ros_enabled=args.ros,
            ros_topic=args.ros_topic,
            limits=limits,
            rate_hz=args.rate,
            target_duration_s=args.target_duration,
        )
    else:
        from .sim import run
        run(ros_enabled=args.ros, ros_topic=args.ros_topic, limits=limits,
            mode_topic=args.mode_topic, slots_path=slots_path)


if __name__ == "__main__":
    main()
