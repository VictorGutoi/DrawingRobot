#!/usr/bin/env bash
# Pi side: spin up a humble container with FastDDS and echo /cmd_vel.
#
# The Pi has no native ROS2 — `ros2` on the host is a wrapper that runs
# ros:humble-ros-base in Docker. This script does the same thing, with an
# extra unicast initial-peers profile so the listener participant can
# discover the Mac's publisher (multicast over WiFi is broken on this
# network).
#
# This is a *diagnostic* container, separate from the compose stack's
# micro-ros-agent. The agent itself works fine without an XML because the
# Mac side declares it as an initial peer; this listener is its own
# participant and needs its own peer config.
#
# Usage (from the Pi, or from the Mac via SSH):
#   ./scripts/run_pi_listener.sh                 # echoes /cmd_vel
#   ./scripts/run_pi_listener.sh topic info      # `ros2 topic info /cmd_vel -v`
#   ./scripts/run_pi_listener.sh node info esp32_p4_robot
#
# Requires sudo (passwordless) for `docker run`.
set -euo pipefail

CMD=(ros2 topic echo /cmd_vel geometry_msgs/msg/Twist)
if [ $# -gt 0 ]; then
    CMD=(ros2 "$@")
fi

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DDS_XML="$REPO_DIR/scripts/dds/fastdds-pi.xml"
[ -f "$DDS_XML" ] || { echo "missing $DDS_XML" >&2; exit 1; }

sudo docker run --rm -it --net=host --privileged \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
    -v "$DDS_XML:/fastdds.xml:ro" \
    ros:humble-ros-base bash -c "
        source /opt/ros/humble/setup.bash
        ${CMD[*]}
    "
