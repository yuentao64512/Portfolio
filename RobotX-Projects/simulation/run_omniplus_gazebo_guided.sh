#!/usr/bin/env bash
set -euo pipefail

# Start Gazebo separately first:
#   bash run_mavlink_gazebo.sh
#
# Usage: bash run_omniplus_gazebo_guided.sh [duration] [forward_mps] [lateral_mps] [rear_trim] [options]
cd "$(dirname "${BASH_SOURCE[0]}")"
duration="${1:-5}"
forward_speed="${2:-0.0}"
lateral_speed="${3:--0.30}"
rear_trim="${4:-1.0}"
shift $(( $# < 4 ? $# : 4 ))
exec python3 scripts/mavlink_omniplus_gazebo_bridge.py \
  --guided \
  --send \
  --arm \
  --duration "${duration}" \
  --forward-speed "${forward_speed}" \
  --lateral-speed "${lateral_speed}" \
  --rear-gazebo-trim "${rear_trim}" \
  "$@"
