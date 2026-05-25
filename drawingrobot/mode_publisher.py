"""ROS2 mode-command publisher for the LULOC2 robot.

Publishes std_msgs/Int8 on /robot/mode_cmd (verified live on the Pi:
subscriber is `esp32_p4_robot`, the firmware's micro-ROS node). The integer
maps to robot_mode_t in the firmware's state_machine_config.h:
  0=NONE, 1=AUTO_PATH, 2=AUTO_OBSTACLE, 3=REMOTE_DRIVE,
  4=TELEMETRY_STREAM, 5=CALIB_MOTORS, 6=CALIB_LINE.

Lazy-imports rclpy inside __init__ so the rest of the package stays usable
on machines without ROS2 installed. Does NOT own the rclpy lifecycle —
assumes RosPublisher (or another caller) has already called rclpy.init()
and will be the one to call rclpy.shutdown(). Close just destroys our node.
"""


MODE_NONE = 0
MODE_AUTONOMOUS_PATH = 1
MODE_AUTONOMOUS_OBSTACLE = 2
MODE_REMOTE_DRIVE = 3
MODE_TELEMETRY_STREAM = 4
MODE_CALIBRATE_MOTORS = 5
MODE_CALIBRATE_LINE = 6


class ModePublisher:
    DEFAULT_TOPIC = "/robot/mode_cmd"

    def __init__(self, topic: str = DEFAULT_TOPIC,
                 node_name: str = "drawingrobot_mode_publisher",
                 verbose: bool = True):
        try:
            import rclpy
            from std_msgs.msg import Int8
        except ImportError as e:
            raise RuntimeError(
                "ROS2 packages not available (rclpy / std_msgs). "
                "Source a ROS2 environment, or install via the standard ROS2 "
                "distribution, before running with --ros."
            ) from e

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._Int8 = Int8
        self._node = rclpy.create_node(node_name)
        self._publisher = self._node.create_publisher(Int8, topic, 10)
        self.topic = topic
        self.node_name = node_name
        self.verbose = verbose
        self.published_count = 0

        if self.verbose:
            print(f"[ROS mode] node='{node_name}'  topic='{topic}'", flush=True)

    def publish_mode(self, mode_id: int) -> None:
        msg = self._Int8()
        msg.data = int(mode_id)
        self._publisher.publish(msg)
        self.published_count += 1
        if self.verbose:
            print(f"[ROS mode] published Int8({mode_id})", flush=True)

    def close(self) -> None:
        # rclpy lifecycle is owned by RosPublisher — just drop our node.
        try:
            self._node.destroy_node()
        except Exception:
            pass
