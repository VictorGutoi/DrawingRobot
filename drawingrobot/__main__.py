import argparse

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
    parser.add_argument("--script", default="square_trace",
                        help="Script name (in scripts/) for --headless mode "
                             "(default: square_trace).")
    parser.add_argument("--pen-s", type=float, default=0.0,
                        metavar="S",
                        help="Pen position in [0, 1) along the chassis perimeter, "
                             "for --headless mode (default: 0.0).")
    parser.add_argument("--rate", type=float, default=60.0,
                        metavar="HZ",
                        help="Headless loop rate in Hz (default: 60).")
    args = parser.parse_args()

    limits = Limits(
        max_linear_cm_s=args.max_linear * 100.0,
        max_angular_rad_s=args.max_angular,
    )

    if args.headless:
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
        run(ros_enabled=args.ros, ros_topic=args.ros_topic, limits=limits)


if __name__ == "__main__":
    main()
