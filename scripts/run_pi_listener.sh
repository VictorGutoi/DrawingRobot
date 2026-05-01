#!/usr/bin/env bash
# Pi side: spin up a humble container with Cyclone DDS and echo /cmd_vel.
#
# The Pi has no native ROS2 — `ros2` on the host is a wrapper that runs
# ros:humble-ros-base in Docker. This script does the same thing but with
# the env tweaks our publisher needs:
#   - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  (must match the Mac)
#   - ros-humble-rmw-cyclonedds-cpp installed in the container
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

sudo docker run --rm -it --net=host --privileged \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    ros:humble-ros-base bash -c "
        apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp >/dev/null 2>&1 || \
            (apt-get update -qq && apt-get install -y -qq ros-humble-rmw-cyclonedds-cpp >/dev/null 2>&1)
        source /opt/ros/humble/setup.bash
        ${CMD[*]}
    "
