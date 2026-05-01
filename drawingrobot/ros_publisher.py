"""ROS2 cmd_vel publisher.

The simulator can optionally mirror its commanded body velocities (v, ω) to a
ROS2 topic so a connected robot follows the same plan. Importing this module
does NOT pull in rclpy — the `rclpy` and `geometry_msgs` imports live inside
`RosPublisher.__init__` so the rest of the codebase stays usable on machines
without a ROS2 install.

Units: the simulator works in cm/s, but ROS `geometry_msgs/Twist` expects m/s,
so v is converted by × 0.01 before publishing. ω is already in rad/s in both.
"""

import os


class RosPublisher:
    DEFAULT_TOPIC = "/cmd_vel"

    def __init__(self, topic: str = DEFAULT_TOPIC,
                 node_name: str = "drawingrobot_publisher",
                 verbose: bool = True):
        try:
            import rclpy
            from geometry_msgs.msg import Twist
        except ImportError as e:
            raise RuntimeError(
                "ROS2 packages not available (rclpy / geometry_msgs). "
                "Source a ROS2 environment, or install via the standard ROS2 "
                "distribution, before running with --ros."
            ) from e

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._Twist = Twist
        self._node = rclpy.create_node(node_name)
        self._publisher = self._node.create_publisher(Twist, topic, 10)
        self.topic = topic
        self.node_name = node_name
        self.verbose = verbose
        self.published_count = 0
        self.skipped_count = 0
        self._last_sent_v: float | None = None
        self._last_sent_omega: float | None = None

        if self.verbose:
            domain = os.environ.get("ROS_DOMAIN_ID", "0")
            rmw = os.environ.get("RMW_IMPLEMENTATION", "<default>")
            print(f"[ROS publish] node='{node_name}'  topic='{topic}'  "
                  f"domain={domain}  rmw={rmw}", flush=True)

    def publish(self, v_cm_s: float, omega_rad_s: float, *, force: bool = False) -> None:
        if (not force
                and self._last_sent_v is not None
                and abs(v_cm_s - self._last_sent_v) < 1e-6
                and abs(omega_rad_s - self._last_sent_omega) < 1e-6):
            self.skipped_count += 1
            return

        msg = self._Twist()
        msg.linear.x = float(v_cm_s) * 0.01   # cm/s -> m/s
        msg.angular.z = float(omega_rad_s)
        self._publisher.publish(msg)
        self.published_count += 1
        self._last_sent_v = v_cm_s
        self._last_sent_omega = omega_rad_s

        if self.verbose:
            print(f"[ROS publish #{self.published_count:>5}] "
                  f"v={msg.linear.x:+.3f} m/s  ω={msg.angular.z:+.3f} rad/s",
                  flush=True)

    def close(self) -> None:
        if self.verbose:
            print(f"[ROS publish] closing — {self.published_count} messages sent",
                  flush=True)
        try:
            self._node.destroy_node()
        finally:
            if self._rclpy.ok():
                self._rclpy.shutdown()
