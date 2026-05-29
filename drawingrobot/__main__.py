import argparse
from math import radians

from .limits import Limits


def main() -> None:
    parser = argparse.ArgumentParser(
        prog="drawingrobot",
        description="Drawing-robot simulator. By default runs the pygame "
                    "simulator stand-alone; --ros also publishes (v, ω) to a "
                    "ROS2 Twist topic; --headless runs a script through the "
                    "kinematics + ROS publisher with no UI (useful on a Pi).",
    )
    parser.add_argument("--ros", action="store_true",
                        help="Publish the commanded body velocity to a ROS2 topic.")
    parser.add_argument("--ros-topic", default="/cmd_vel",
                        help="ROS2 topic name (default: /cmd_vel).")
    parser.add_argument("--mode-topic", default="/robot/mode_cmd",
                        help="ROS2 topic for the LULOC2 mode-set button "
                             "(std_msgs/Int8, default: /robot/mode_cmd). "
                             "Requires --ros.")
    parser.add_argument("--max-linear", type=float, default=0.5,
                        metavar="M_S",
                        help="Linear-speed ceiling in m/s (default: 0.5). "
                             "Applied to both the simulator and the ROS publish.")
    parser.add_argument("--max-angular", type=float, default=0.5,
                        metavar="RAD_S",
                        help="Angular-speed ceiling in rad/s (default: 0.5). "
                             "Applied to both the simulator and the ROS publish.")
    parser.add_argument("--headless", action="store_true",
                        help="Run a script with no pygame UI; integrates kinematics "
                             "+ publishes ROS2. Useful on a Pi or other display-less host.")
    parser.add_argument("--pi-service", action="store_true",
                        help="Run the Pi-side daemon: subscribe to /robot/mode_cmd, "
                             "play stored scripts by slot (codes 80..89), publish "
                             "/cmd_vel. Mutually exclusive with --headless. "
                             "Implies --ros.")
    parser.add_argument("--slots-config", default=None,
                        metavar="PATH",
                        help="Path to the pi_slots.json mapping (default: "
                             "<repo>/pi_slots.json). Used by --pi-service and the "
                             "simulator's slots panel.")
    parser.add_argument("--script", default="square_trace",
                        help="Script name (in scripts/) for --headless mode "
                             "(default: square_trace).")
    parser.add_argument("--pen-s", type=float, default=0.0,
                        metavar="S",
                        help="Pen position in [0, 1) along the chassis perimeter, "
                             "for --headless mode (default: 0.0).")
    parser.add_argument("--rate", type=float, default=60.0,
                        metavar="HZ",
                        help="Headless / Pi-service loop rate in Hz (default: 60).")
    parser.add_argument("--sensors-topic", default="/sensors",
                        help="ROS2 topic for encoder feedback (default: "
                             "/sensors). Subscribed when --ros is on; "
                             "drives the encoder ghost and the closed-loop "
                             "correction.")
    parser.add_argument("--correction-threshold-cm", type=float, default=1.0,
                        metavar="CM",
                        help="Position drift threshold (cm) below which no "
                             "correction is injected at a command boundary "
                             "(default: 1.0).")
    parser.add_argument("--correction-threshold-deg", type=float, default=5.0,
                        metavar="DEG",
                        help="Heading drift threshold (degrees) below which "
                             "no correction is injected (default: 5.0).")
    parser.add_argument("--no-correction", action="store_true",
                        help="Display the encoder ghost but do not inject "
                             "corrective WheelCommands. Useful for observing "
                             "drift without altering the /cmd_vel stream.")
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
            pen_s_normalized=args.pen_s,
            ros_enabled=args.ros,
            ros_topic=args.ros_topic,
            limits=limits,
            rate_hz=args.rate,
        )
    else:
        from .sim import run
        run(ros_enabled=args.ros, ros_topic=args.ros_topic, limits=limits,
            mode_topic=args.mode_topic, slots_path=slots_path,
            sensors_topic=args.sensors_topic,
            correction_enabled=not args.no_correction,
            correction_threshold_cm=args.correction_threshold_cm,
            correction_threshold_rad=radians(args.correction_threshold_deg))


if __name__ == "__main__":
    main()
