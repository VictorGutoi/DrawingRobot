#!/usr/bin/env bash
# Mac side: launch the simulator wired to ROS2 over FastDDS, no Pi orchestration.
#
# Why FastDDS: the Pi's docker-compose stack runs micro-ros-agent on the
# default RMW (FastDDS). For interop the Mac speaks FastDDS too. Default
# multicast discovery does not work on this network (the home AP drops
# multicast between WiFi clients), so the Mac uses an XML profile that
# adds the Pi as a unicast initial peer.
#
# Usage:
#   conda activate ros2     # one-time per shell
#   ./scripts/run_sim_ros.sh
#
# Args after the script name are forwarded to `python -m drawingrobot`,
# e.g. ./scripts/run_sim_ros.sh --max-linear 0.3
set -euo pipefail

cd "$(dirname "$0")/.."

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$(pwd)/scripts/dds/fastdds-mac.xml"
unset CYCLONEDDS_URI

exec python -m drawingrobot --ros "$@"
