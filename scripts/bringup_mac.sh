#!/usr/bin/env bash
# Mac side: launch the simulator wired to the Pi's existing micro-ROS agent
# over FastDDS.
#
# This script assumes the Pi already runs the micro-ros-agent (and any
# other ROS2 services) as part of its docker-compose stack at
# ~/Documentos/Luloc-ASTI-25-26/02-software/. The Mac just joins that DDS
# bus over WiFi using a FastDDS unicast initial-peers profile. Nothing on
# the Pi is started, stopped, or reconfigured.
#
# If the Pi compose stack is not running, ssh into the Pi and start it
# with `docker compose up -d` from that directory. (Or, for an alternative
# clean-slate flow that brings up its own agent, see scripts/bringup_pi.sh.)
#
# Usage:
#   ./scripts/bringup_mac.sh                           # default: open the GUI sim
#   ./scripts/bringup_mac.sh --script square_trace     # forwards args to drawingrobot
#   ./scripts/bringup_mac.sh --headless --script line_to
#
# Env overrides:
#   PI_HOST       — default pi5@192.168.50.53 (used only for an SSH preflight)
#   CONDA_ENV     — default ros2
#   CONDA_ROOT    — default ~/radioconda (auto-detect fallback)
#   SKIP_PI_CHECK — set to 1 to skip the SSH preflight (e.g. when offline)
set -euo pipefail

PI_HOST="${PI_HOST:-pi5@192.168.50.53}"
CONDA_ENV="${CONDA_ENV:-ros2}"

cd "$(dirname "$0")/.."
REPO_DIR="$(pwd)"
DDS_XML="$REPO_DIR/scripts/dds/fastdds-mac.xml"

say() { printf '\n=== %s ===\n' "$*"; }
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# --- Preflight -----------------------------------------------------------

say "preflight"

[ -f "$DDS_XML" ] || fail "missing $DDS_XML"

# Locate conda. radioconda is the user's setup; allow override.
CONDA_ROOT="${CONDA_ROOT:-$HOME/radioconda}"
if [ ! -f "$CONDA_ROOT/etc/profile.d/conda.sh" ]; then
    for candidate in "$HOME/miniforge3" "$HOME/miniconda3" "$HOME/anaconda3" "/opt/homebrew/Caskroom/miniconda/base"; do
        if [ -f "$candidate/etc/profile.d/conda.sh" ]; then
            CONDA_ROOT="$candidate"
            break
        fi
    done
fi
[ -f "$CONDA_ROOT/etc/profile.d/conda.sh" ] || \
    fail "conda not found (tried $CONDA_ROOT). Set CONDA_ROOT=/path/to/your/conda"
# shellcheck disable=SC1091
source "$CONDA_ROOT/etc/profile.d/conda.sh"

if ! conda env list | awk '{print $1}' | grep -Fxq "$CONDA_ENV"; then
    fail "conda env '$CONDA_ENV' not found. See CLAUDE.md for RoboStack install."
fi
echo "OK conda env: $CONDA_ENV"

# Optional: sanity-check that the Pi is reachable and the agent is running.
# Non-fatal — the user might be offline, or the Pi might use a different host.
if [ "${SKIP_PI_CHECK:-0}" != "1" ]; then
    if ssh -o BatchMode=yes -o ConnectTimeout=4 "$PI_HOST" true 2>/dev/null; then
        if ssh "$PI_HOST" 'sudo docker ps --format "{{.Names}}" 2>/dev/null | grep -Fxq micro_ros_agent' 2>/dev/null; then
            echo "OK Pi reachable at $PI_HOST, micro_ros_agent container is up"
        else
            echo "WARN Pi reachable but micro_ros_agent container is not running."
            echo "     ssh $PI_HOST then 'cd ~/Documentos/Luloc-ASTI-25-26/02-software && docker compose up -d'"
        fi
    else
        echo "WARN cannot ssh to $PI_HOST (set SKIP_PI_CHECK=1 to silence) — continuing anyway"
    fi
fi

# --- Mac DDS env + run sim ----------------------------------------------

say "starting simulator (Ctrl-C to stop)"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_XML"
unset CYCLONEDDS_URI

# conda's activate scripts reference unset vars (CONDA_BUILD etc.); relax -u
# around the activate, then restore.
set +u
conda activate "$CONDA_ENV"
set -u

exec python -m drawingrobot --ros "$@"
