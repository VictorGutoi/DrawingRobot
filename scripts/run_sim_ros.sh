#!/usr/bin/env bash
# Mac side: launch the simulator wired to ROS2 over Cyclone DDS.
#
# Why Cyclone: in this project's setup, FastDDS (RoboStack default on macOS)
# does not announce publishers in a way the Pi container's FastDDS can see.
# Cyclone↔Cyclone interops; the Pi side script (run_pi_listener.sh) installs
# Cyclone in the ROS humble container.
#
# Usage:
#   conda activate ros2     # one-time per shell
#   ./scripts/run_sim_ros.sh
#
# Args after the script name are forwarded to `python -m drawingrobot`,
# e.g. ./scripts/run_sim_ros.sh --max-linear 0.3
set -euo pipefail

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
unset CYCLONEDDS_URI    # interface pin breaks same-host loopback on macOS

cd "$(dirname "$0")/.."
exec python -m drawingrobot --ros "$@"
