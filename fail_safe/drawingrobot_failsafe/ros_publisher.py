"""ROS2 cmd_vel publisher (fail-safe edition).

Publishes a fresh Twist per call; non-zero (v, ω) goes out every tick,
idle (0, 0) is throttled to a heartbeat rate (default 1 Hz) so the
consumer still knows we're alive without flooding the topic when
nothing is moving.

Units: simulator works in cm/s, geometry_msgs/Twist expects m/s, so v is
converted by × 0.01 before publishing. ω stays as rad/s.

This is the verbatim parent publisher minus the TwistListener — the
fail-safe sim does not render a ghost trace driven by the wire.
"""

import os
import time


class RosPublisher:
    DEFAULT_TOPIC = "/cmd_vel"
    DEFAULT_HEARTBEAT_S = 1.0

    def __init__(self, topic: str = DEFAULT_TOPIC,
                 node_name: str = "drawingrobot_failsafe_publisher",
                 verbose: bool = True,
                 heartbeat_s: float = DEFAULT_HEARTBEAT_S):
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
        self.heartbeat_s = heartbeat_s
        self.published_count = 0
        self.skipped_count = 0
        self._last_pub_t = float("-inf")

        if self.verbose:
            domain = os.environ.get("ROS_DOMAIN_ID", "0")
            rmw = os.environ.get("RMW_IMPLEMENTATION", "<default>")
            print(f"[ROS publish] node='{node_name}'  topic='{topic}'  "
                  f"domain={domain}  rmw={rmw}  "
                  f"heartbeat={heartbeat_s:g}s", flush=True)

    @staticmethod
    def _should_publish(is_zero: bool, force: bool, now: float,
                        last_pub_t: float, heartbeat_s: float) -> bool:
        """Throttle decision: non-zero always goes out, zero is rate-limited.

        force=True bypasses (used for the final stop Twist at shutdown).
        """
        if force or not is_zero:
            return True
        return (now - last_pub_t) >= heartbeat_s

    def publish(self, v_cm_s: float, omega_rad_s: float, *, force: bool = False) -> None:
        is_zero = (v_cm_s == 0.0 and omega_rad_s == 0.0)
        now = time.monotonic()
        if not self._should_publish(is_zero, force, now, self._last_pub_t, self.heartbeat_s):
            self.skipped_count += 1
            return

        msg = self._Twist()
        msg.linear.x = float(v_cm_s) * 0.01   # cm/s -> m/s
        msg.angular.z = float(omega_rad_s)
        self._publisher.publish(msg)
        self.published_count += 1
        self._last_pub_t = now

        if self.verbose and self.published_count % 60 == 0:
            print(f"[ROS publish #{self.published_count:>5}] "
                  f"v={msg.linear.x:+.3f} m/s  ω={msg.angular.z:+.3f} rad/s  "
                  f"(skipped {self.skipped_count} idle ticks)",
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
