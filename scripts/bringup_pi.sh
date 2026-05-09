#!/usr/bin/env bash
# Pi side: start the micro-ROS agent that bridges /cmd_vel into the ESP32.
#
# Designed to be invoked by ./scripts/bringup_mac.sh over ssh, but also
# runs standalone for debugging.
#
# Usage:
#   ./scripts/bringup_pi.sh           # safe — aborts if another agent is already running
#   ./scripts/bringup_pi.sh --force   # stop our prior container if any, then start
#
# Env overrides:
#   AGENT_IMAGE  — default microros/micro-ros-agent:humble
#   AGENT_BAUD   — default 115200
#   AGENT_DEVICE — default auto (first of /dev/ttyUSB0 /dev/ttyACM0)
#
# IMPORTANT: requires passwordless sudo for `docker run`. The Mac side's
# preflight checks this; if you see a sudo prompt here, follow the
# bringup_mac.sh instructions to add a NOPASSWD line.
set -euo pipefail

FORCE=0
for arg in "$@"; do
    case "$arg" in
        --force) FORCE=1 ;;
        *) echo "unknown arg: $arg" >&2; exit 2 ;;
    esac
done

CONTAINER_NAME="microros_agent"
AGENT_IMAGE="${AGENT_IMAGE:-microros/micro-ros-agent:humble}"
AGENT_BAUD="${AGENT_BAUD:-115200}"

cd "$(dirname "$0")/.."
REPO_DIR="$(pwd)"
DDS_XML="$REPO_DIR/scripts/dds/cyclone-pi.xml"

# --- Preflight -----------------------------------------------------------

if ! command -v docker >/dev/null 2>&1; then
    echo "ERROR: docker not installed on this Pi" >&2
    exit 1
fi

if [ ! -f "$DDS_XML" ]; then
    echo "ERROR: missing $DDS_XML — run bringup_mac.sh from your laptop to sync the repo" >&2
    exit 1
fi

# Pick the first attached USB serial device unless the user pinned one.
if [ -z "${AGENT_DEVICE:-}" ]; then
    for dev in /dev/ttyUSB0 /dev/ttyACM0 /dev/ttyUSB1 /dev/ttyACM1; do
        if [ -e "$dev" ]; then
            AGENT_DEVICE="$dev"
            break
        fi
    done
fi

if [ -z "${AGENT_DEVICE:-}" ] || [ ! -e "$AGENT_DEVICE" ]; then
    echo "ERROR: no ESP32 serial device found at /dev/ttyUSB* or /dev/ttyACM*" >&2
    echo "       plug the ESP32 in, or set AGENT_DEVICE=/dev/ttyXXX" >&2
    exit 1
fi

echo "agent device: $AGENT_DEVICE"
echo "agent image:  $AGENT_IMAGE"
echo "baud:         $AGENT_BAUD"

# --- Conflict detection --------------------------------------------------
# Don't yank someone else's agent. If --force, only stop *our* prior container.

OTHER_CONTAINER="$(sudo docker ps --format '{{.Names}}\t{{.Image}}' \
    | awk -v me="$CONTAINER_NAME" '$1 != me && (tolower($0) ~ /micro.?ros|agent/) {print $1; exit}')"

if [ -n "$OTHER_CONTAINER" ]; then
    echo "ERROR: another agent-like container is already running: $OTHER_CONTAINER" >&2
    echo "       stop it first (sudo docker stop $OTHER_CONTAINER), or rename ours via CONTAINER_NAME" >&2
    exit 1
fi

if command -v lsof >/dev/null 2>&1; then
    HOLDER="$(sudo lsof "$AGENT_DEVICE" 2>/dev/null | awk 'NR>1 {print $1"("$2")"}' | head -1 || true)"
    if [ -n "$HOLDER" ]; then
        # Our container also shows up here when it's running, so allow if it's ours.
        OUR_RUNNING="$(sudo docker ps --format '{{.Names}}' | grep -Fx "$CONTAINER_NAME" || true)"
        if [ -z "$OUR_RUNNING" ]; then
            echo "ERROR: $AGENT_DEVICE is held by: $HOLDER" >&2
            echo "       another process is talking to the ESP32 — stop it first" >&2
            exit 1
        fi
    fi
fi

if command -v systemctl >/dev/null 2>&1; then
    OTHER_UNIT="$(systemctl list-units --type=service --state=running --no-legend 2>/dev/null \
        | awk '{print $1}' | grep -iE 'micro.?ros|agent' || true)"
    if [ -n "$OTHER_UNIT" ]; then
        echo "ERROR: a systemd unit looks like an agent: $OTHER_UNIT" >&2
        echo "       stop it (sudo systemctl stop $OTHER_UNIT) before bringing this one up" >&2
        exit 1
    fi
fi

# Stop our own prior container so re-runs are idempotent.
if sudo docker ps -a --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
    if [ "$FORCE" -eq 1 ] || [ -z "$(sudo docker ps --format '{{.Names}}' | grep -Fx "$CONTAINER_NAME")" ]; then
        sudo docker rm -f "$CONTAINER_NAME" >/dev/null
    else
        echo "our agent container is already running; reusing it"
        sudo docker logs --tail 5 "$CONTAINER_NAME" || true
        echo "agent ready"
        exec sudo docker logs -f "$CONTAINER_NAME"
    fi
fi

# --- Start agent ---------------------------------------------------------

sudo docker run -d --rm --name "$CONTAINER_NAME" \
    --net=host --privileged \
    --device="$AGENT_DEVICE" \
    -v "$DDS_XML:/cyclone.xml:ro" \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e CYCLONEDDS_URI=file:///cyclone.xml \
    "$AGENT_IMAGE" \
    serial --dev "$AGENT_DEVICE" -b "$AGENT_BAUD" -v6 >/dev/null

# Wait for the container to come up healthy.
for _ in $(seq 1 20); do
    if sudo docker ps --format '{{.Names}}' | grep -Fxq "$CONTAINER_NAME"; then
        echo "agent ready"
        exec sudo docker logs -f "$CONTAINER_NAME"
    fi
    sleep 0.5
done

echo "ERROR: agent container did not come up; recent logs:" >&2
sudo docker logs "$CONTAINER_NAME" 2>&1 | tail -20 >&2 || true
exit 1
