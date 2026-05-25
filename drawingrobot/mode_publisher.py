"""ROS2 mode-command publisher + listener for the LULOC2 robot.

Topic: `/robot/mode_cmd` (std_msgs/Int8). Two concurrent consumers exist on
the robot side:
  * the firmware (`esp32_p4_robot`, micro-ROS) handles codes 0..6
    from `robot_mode_t` in `state_machine_config.h`
    (0=NONE, 1=AUTO_PATH, 2=AUTO_OBSTACLE, 3=REMOTE_DRIVE,
     4=TELEMETRY_STREAM, 5=CALIB_MOTORS, 6=CALIB_LINE);
  * the Pi-side `pi_service` (this repo) handles codes 80..89 as
    "run stored script slot 0..9" — see `drawingrobot.pi_service`.

The firmware ignores unknown codes, so the two ranges coexist.

Lazy-imports rclpy inside __init__ so the rest of the package stays usable
on machines without ROS2 installed. Does NOT own the rclpy lifecycle —
assumes RosPublisher (or another caller) has already called rclpy.init()
and will be the one to call rclpy.shutdown(). Close just destroys our node.
"""

from __future__ import annotations

import threading
from typing import Optional


MODE_NONE = 0
MODE_AUTONOMOUS_PATH = 1
MODE_AUTONOMOUS_OBSTACLE = 2
MODE_REMOTE_DRIVE = 3
MODE_TELEMETRY_STREAM = 4
MODE_CALIBRATE_MOTORS = 5
MODE_CALIBRATE_LINE = 6

MODE_STOP = MODE_NONE
MODE_SCRIPT_BASE = 80
MODE_SCRIPT_MAX = 89

# Drawing-time presets: codes 75..79 set the pi_service's target run duration
# (uniform rescale applied at slot launch). Indices line up with TIME_PRESETS
# (75 → 20 s, 76 → 30 s, ..., 79 → 60 s).
MODE_TIME_BASE = 75
MODE_TIME_MAX = 79
TIME_PRESETS = (20.0, 30.0, 40.0, 50.0, 60.0)
DEFAULT_DRAWING_TIME_S = 50.0


def is_script_slot_code(code: int) -> bool:
    return MODE_SCRIPT_BASE <= code <= MODE_SCRIPT_MAX


def slot_for_code(code: int) -> int:
    return code - MODE_SCRIPT_BASE


def code_for_slot(slot: int) -> int:
    return MODE_SCRIPT_BASE + slot


def is_time_code(code: int) -> bool:
    return MODE_TIME_BASE <= code <= MODE_TIME_MAX


def duration_for_code(code: int) -> float:
    return TIME_PRESETS[code - MODE_TIME_BASE]


def code_for_time_index(idx: int) -> int:
    return MODE_TIME_BASE + idx


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

    def publish_script_slot(self, slot: int) -> None:
        if slot < 0 or slot > (MODE_SCRIPT_MAX - MODE_SCRIPT_BASE):
            raise ValueError(
                f"script slot must be in [0, {MODE_SCRIPT_MAX - MODE_SCRIPT_BASE}], "
                f"got {slot}"
            )
        self.publish_mode(code_for_slot(slot))

    def publish_time_preset(self, idx: int) -> None:
        if idx < 0 or idx >= len(TIME_PRESETS):
            raise ValueError(
                f"time preset index must be in [0, {len(TIME_PRESETS) - 1}], got {idx}"
            )
        self.publish_mode(code_for_time_index(idx))

    def publish_stop(self) -> None:
        self.publish_mode(MODE_STOP)

    def close(self) -> None:
        # rclpy lifecycle is owned by RosPublisher — just drop our node.
        try:
            self._node.destroy_node()
        except Exception:
            pass


class ModeListener:
    """Subscribes to /robot/mode_cmd; exposes the latest unseen code.

    Latched single-element buffer: callbacks just overwrite `_pending`, so if
    several codes arrive between drains only the latest one is seen. That's
    exactly what pi_service wants ("cancel and restart"): if the user mashes
    three buttons quickly we should run the last one, not queue all three.

    Does not own the rclpy lifecycle (mirrors `ModePublisher`). Caller must
    drive the executor; use `spin_once(timeout_s)` from the main loop.
    """

    DEFAULT_TOPIC = ModePublisher.DEFAULT_TOPIC

    def __init__(self, topic: str = DEFAULT_TOPIC,
                 node_name: str = "drawingrobot_mode_listener",
                 verbose: bool = True):
        try:
            import rclpy
            from std_msgs.msg import Int8
        except ImportError as e:
            raise RuntimeError(
                "ROS2 packages not available (rclpy / std_msgs). "
                "Source a ROS2 environment, or install via the standard ROS2 "
                "distribution, before running with --pi-service."
            ) from e

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self._node = rclpy.create_node(node_name)
        self._subscription = self._node.create_subscription(
            Int8, topic, self._on_msg, 10)
        self.topic = topic
        self.node_name = node_name
        self.verbose = verbose
        self._lock = threading.Lock()
        self._pending: Optional[int] = None
        self.received_count = 0

        if self.verbose:
            print(f"[ROS mode-listen] node='{node_name}'  topic='{topic}'",
                  flush=True)

    def _on_msg(self, msg) -> None:
        with self._lock:
            self._pending = int(msg.data)
            self.received_count += 1
        if self.verbose:
            print(f"[ROS mode-listen] rx Int8({int(msg.data)})", flush=True)

    def spin_once(self, timeout_s: float = 0.0) -> None:
        self._rclpy.spin_once(self._node, timeout_sec=timeout_s)

    def pop_request(self) -> Optional[int]:
        with self._lock:
            code, self._pending = self._pending, None
        return code

    def close(self) -> None:
        try:
            self._node.destroy_node()
        except Exception:
            pass
