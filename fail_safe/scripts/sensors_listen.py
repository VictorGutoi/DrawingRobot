#!/usr/bin/env python3
"""Standalone /sensors reader — a Python replacement for `ros2 topic echo /sensors`.

Why this exists:
  1. You don't have to keep typing the `ros2 topic echo` incantation.
  2. The /sensors message type is project-specific and not yet wired into
     `SensorListener`. This script *discovers* the type at runtime, subscribes,
     and prints every field name + value of the first message — exactly what
     you need to fill in the `_extract_*` adapter in
     `drawingrobot_failsafe/ros_publisher.py`.

Usage (with a ROS2 env sourced — e.g. inside the `ros2` conda env that
bringup_mac.sh activates, so the FastDDS profile + domain match the robot):

    python scripts/sensors_listen.py
    python scripts/sensors_listen.py --topic /sensors --count 5
    python scripts/sensors_listen.py --raw          # stream every message

It prints the RMW / domain / profile it's using up front, so if you see the
robot's other topics in `ros2 topic list` but nothing here, the env is the
suspect (wrong RMW, wrong domain, or the FastDDS profile not loaded).
"""

from __future__ import annotations

import argparse
import os
import sys
import time


def _print_env() -> None:
    rmw = os.environ.get("RMW_IMPLEMENTATION", "<default>")
    domain = os.environ.get("ROS_DOMAIN_ID", "0 (default)")
    profile = os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", "<none>")
    cyclone = os.environ.get("CYCLONEDDS_URI", "<none>")
    print("─" * 60)
    print(f"  RMW_IMPLEMENTATION         = {rmw}")
    print(f"  ROS_DOMAIN_ID              = {domain}")
    print(f"  FASTRTPS_DEFAULT_PROFILES  = {profile}")
    print(f"  CYCLONEDDS_URI             = {cyclone}")
    print("─" * 60)
    if rmw not in ("rmw_fastrtps_cpp", "<default>"):
        print("  ⚠  RMW is not FastDDS. The Pi agent is FastDDS — set\n"
              "     RMW_IMPLEMENTATION=rmw_fastrtps_cpp to match.")
    if cyclone != "<none>" and "fastrtps" in rmw:
        print("  ⚠  CYCLONEDDS_URI is set while using FastDDS — unset it.")
    print()


def _resolve_topic_type(node, topic: str, timeout_s: float):
    """Poll the graph until `topic` appears; return its type string or None."""
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for name, types in node.get_topic_names_and_types():
            if name == topic and types:
                return types[0]
        time.sleep(0.2)
    return None


def _dump_fields(msg) -> None:
    """Print every field of a ROS message with its declared type and value."""
    fields = getattr(msg, "get_fields_and_field_types", lambda: {})()
    if not fields:
        print(f"    (no introspectable fields) repr={msg!r}")
        return
    for fname, ftype in fields.items():
        value = getattr(msg, fname)
        # Arrays print their length + first few entries so long encoder
        # arrays stay readable.
        if isinstance(value, (list, tuple)) or hasattr(value, "__len__") and not isinstance(value, str):
            try:
                seq = list(value)
                preview = seq[:6]
                more = "" if len(seq) <= 6 else f" … (+{len(seq) - 6} more)"
                print(f"    {fname:<20} {ftype:<24} len={len(seq)}  {preview}{more}")
                continue
            except TypeError:
                pass
        print(f"    {fname:<20} {ftype:<24} {value}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/sensors", help="Topic to read (default: /sensors)")
    ap.add_argument("--count", type=int, default=3,
                    help="Print this many full-field dumps then exit (default: 3). "
                         "Use --raw to stream forever instead.")
    ap.add_argument("--raw", action="store_true",
                    help="Stream a one-line summary of every message, no exit.")
    ap.add_argument("--discover-timeout", type=float, default=10.0,
                    help="Seconds to wait for the topic to appear (default: 10).")
    args = ap.parse_args()

    _print_env()

    try:
        import rclpy
        from rosidl_runtime_py.utilities import get_message
    except ImportError as e:
        print(f"ERROR: ROS2 Python not available: {e}\n"
              "Source your ROS2 env first (e.g. `conda activate ros2`).",
              file=sys.stderr)
        return 2

    rclpy.init()
    node = rclpy.create_node("sensors_listen_probe")
    print(f"Looking for {args.topic} (up to {args.discover_timeout:.0f}s)…")

    type_str = _resolve_topic_type(node, args.topic, args.discover_timeout)
    if type_str is None:
        print(f"\n✗ {args.topic} not found on the graph.\n"
              f"  `ros2 topic list` from this same shell should show it; if it\n"
              f"  doesn't, the env above doesn't match the robot's DDS config.")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    print(f"✓ {args.topic} is '{type_str}'\n")
    msg_class = get_message(type_str)

    received = {"n": 0}

    def on_msg(msg):
        received["n"] += 1
        n = received["n"]
        if args.raw:
            fields = getattr(msg, "get_fields_and_field_types", lambda: {})()
            summ = ", ".join(f"{f}={getattr(msg, f)}" for f in list(fields)[:4])
            print(f"[{n:>4}] {summ}")
        else:
            print(f"── message #{n} ─────────────────────────────")
            _dump_fields(msg)
            print()

    node.create_subscription(msg_class, args.topic, on_msg, 10)

    print("Listening… (Ctrl-C to stop)\n")
    try:
        if args.raw:
            rclpy.spin(node)
        else:
            while received["n"] < args.count and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.5)
            if received["n"] == 0:
                print("✗ topic exists but no messages arrived — the publisher\n"
                      "  may be idle, or one-way DDS visibility is blocking data.")
            else:
                print("─" * 60)
                print("To wire this into SensorListener, edit\n"
                      "  drawingrobot_failsafe/ros_publisher.py\n"
                      f"set the import to '{type_str}' and write an _extract\n"
                      "that returns (v_left_mps, v_right_mps, d_left_m, d_right_m)\n"
                      "from the fields printed above.")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
