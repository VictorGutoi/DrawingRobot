"""ROS2 cmd_vel publisher.

Mirrors scripts/_ros_smoke_publisher.py exactly — that publisher is known
to drive the ESP32 correctly end-to-end. Publishes a fresh Twist per call;
non-zero (v, ω) goes out every tick, idle (0, 0) is throttled to a
heartbeat rate (default 1 Hz) so the consumer still knows we're alive
without flooding the topic when nothing is moving.

Units: simulator works in cm/s, geometry_msgs/Twist expects m/s, so v is
converted by × 0.01 before publishing. ω stays as rad/s.
"""

import os
import time


class RosPublisher:
    DEFAULT_TOPIC = "/cmd_vel"
    DEFAULT_HEARTBEAT_S = 1.0

    def __init__(self, topic: str = DEFAULT_TOPIC,
                 node_name: str = "drawingrobot_publisher",
                 verbose: bool = True,
                 heartbeat_s: float = DEFAULT_HEARTBEAT_S):
        try:
            import rclpy
                
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
        # -inf so the first publish always fires (even a leading zero gives the
        # consumer something to lock onto at startup).
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


class TwistListener:
    """Subscribes to a Twist topic; exposes the latest unseen sample.

    Mirrors `mode_publisher.ModeListener`: latched single-element buffer
    (only the most recent sample is kept), `spin_once` drained from the
    main loop. Does not own the rclpy lifecycle — `RosPublisher.close`
    is the one that calls `rclpy.shutdown()`.

    Units: returns (v_cm_s, omega_rad_s) to match `RosPublisher.publish`'s
    inputs, so the sim can integrate without unit conversions.
    """

    def __init__(self, topic: str = "/cmd_vel",
                 node_name: str = "drawingrobot_listener",
                 verbose: bool = True):
        try:
            import rclpy
            from geometry_msgs.msg import Twist
        except ImportError as e:
            raise RuntimeError(
                "ROS2 packages not available (rclpy / geometry_msgs)."
            ) from e

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node(node_name)
        self._subscription = self._node.create_subscription(
            Twist, topic, self._on_msg, 10)
        self.topic = topic
        self.node_name = node_name
        self.verbose = verbose
        self.received_count = 0
        self._latest: tuple[float, float] | None = None

        if self.verbose:
            print(f"[ROS listen] node='{node_name}'  topic='{topic}'", flush=True)

    def _on_msg(self, msg) -> None:
        # m/s on the wire (geometry_msgs/Twist convention) → cm/s for the sim.
        self._latest = (float(msg.linear.x) * 100.0, float(msg.angular.z))
        self.received_count += 1

    def spin_once(self, timeout_s: float = 0.0) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=timeout_s)

    def drain(self, max_iters: int = 16) -> None:
        # rclpy's spin_once(timeout_sec=0) processes at most one callback. At
        # 60 Hz publish → 60 Hz consume, any jitter leaves samples queued; the
        # next single spin only consumes one and the backlog grows. Loop until
        # received_count stops changing (queue drained) or hit max_iters.
        for _ in range(max_iters):
            before = self.received_count
            self._rclpy.spin_once(self._node, timeout_sec=0.0)
            if self.received_count == before:
                return

    def pop_latest(self) -> tuple[float, float] | None:
        sample, self._latest = self._latest, None
        return sample

    def close(self) -> None:
        try:
            self._node.destroy_node()
        except Exception:
            pass
