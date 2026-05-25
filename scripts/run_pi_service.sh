#!/usr/bin/env bash
# Pi side: run `python -m drawingrobot --pi-service --ros` inside a humble
# container so we don't have to install rclpy natively.
#
# Default image is microros/micro-ros-agent:humble — already on the Pi (it's
# what the compose stack uses for the agent itself), so no pull. The image
# ships rclpy + std_msgs + geometry_msgs + the ros2 CLI, which is everything
# pi_service needs. ros:humble-ros-base also works if you have it; override
# with PI_SERVICE_IMAGE.
#
# DDS: same setup as run_pi_listener.sh — FastDDS pinned to wlan0 with the Mac
# listed as a unicast initial peer (the network's AP drops multicast, so without
# the peer list the Pi participant can't discover the Mac's publisher).
#
# Usage (from the Pi):
#   ./scripts/run_pi_service.sh
#   ./scripts/run_pi_service.sh --max-linear 0.3 --max-angular 1.0
#   PI_SERVICE_IMAGE=ros:humble-ros-base ./scripts/run_pi_service.sh
#
# Requires (passwordless) sudo for `docker run`.
set -euo pipefail

REPO_DIR="$(cd "$(dirname "$0")/.." && pwd)"
DDS_XML="$REPO_DIR/scripts/dds/fastdds-pi.xml"
[ -f "$DDS_XML" ] || { echo "missing $DDS_XML" >&2; exit 1; }

IMAGE="${PI_SERVICE_IMAGE:-microros/micro-ros-agent:humble}"

sudo docker run --rm -it --net=host --privileged \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e FASTRTPS_DEFAULT_PROFILES_FILE=/fastdds.xml \
    -e PYTHONUNBUFFERED=1 \
    -v "$DDS_XML:/fastdds.xml:ro" \
    -v "$REPO_DIR:/work" \
    -w /work \
    --entrypoint bash \
    "$IMAGE" -lc "
        source /opt/ros/humble/setup.bash
        python3 -m drawingrobot --pi-service --ros $*
    "
