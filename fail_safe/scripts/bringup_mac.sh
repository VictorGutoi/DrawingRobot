#!/usr/bin/env bash
# Mac side: launch the FAIL-SAFE simulator wired to the Pi's micro-ROS agent
# over FastDDS. Mirrors ../../scripts/bringup_mac.sh (the parent project's
# bringup) — same conda env, same DDS XML, same Pi preflight — only the
# final exec line runs `drawingrobot_failsafe` instead of `drawingrobot`.
#
# The fail-safe shares /robot/mode_cmd and /cmd_vel topics with the parent,
# so no Pi-side reconfiguration is needed.
#
# Usage:
#   ./scripts/bringup_mac.sh                            # default: open GUI sim
#   ./scripts/bringup_mac.sh --headless --script square
#   ./scripts/bringup_mac.sh --pi-service               # run the fail-safe Pi daemon locally
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
FAILSAFE_DIR="$(pwd)"
# Reuse the parent's FastDDS XML — it's pinned to the Mac's en0 interface
# and lists the Pi as a unicast initial peer. No reason to duplicate it.
PARENT_REPO="$(cd .. && pwd)"
DDS_XML="$PARENT_REPO/scripts/dds/fastdds-mac.xml"

say() { printf '\n=== %s ===\n' "$*"; }
fail() { printf 'ERROR: %s\n' "$*" >&2; exit 1; }

# --- Preflight -----------------------------------------------------------

say "preflight"

[ -f "$DDS_XML" ] || fail "missing $DDS_XML (expected the parent project's DDS profile)"

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
    fail "conda env '$CONDA_ENV' not found. See parent project's CLAUDE.md for RoboStack install."
fi
echo "OK conda env: $CONDA_ENV"

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

# --- Mac DDS env + run fail-safe ----------------------------------------

say "starting fail-safe simulator (Ctrl-C to stop)"

set +u
conda activate "$CONDA_ENV"
set -u

# Export AFTER conda activate — RoboStack's activate.d overrides RMW_IMPLEMENTATION
# with the env default; exporting before activation gets clobbered.
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$DDS_XML"
unset CYCLONEDDS_URI

# Run from the fail_safe dir so the relative pi_slots.json / scripts/ paths
# resolve correctly.
cd "$FAILSAFE_DIR"
exec python -m drawingrobot_failsafe --ros "$@"
