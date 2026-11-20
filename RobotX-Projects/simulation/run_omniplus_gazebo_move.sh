#!/usr/bin/env bash
set -euo pipefail

# Start Gazebo separately first:
#   bash run_mavlink_gazebo.sh
#
# Usage: bash run_omniplus_gazebo_move.sh [duration] [lateral] [rear_trim] [options]
cd "$(dirname "${BASH_SOURCE[0]}")"
duration="${1:-5}"
lateral="${2:--0.20}"
rear_trim="${3:-1.0}"
shift $(( $# < 3 ? $# : 3 ))
exec python3 scripts/mavlink_omniplus_gazebo_bridge.py \
  --send \
  --arm \
  --duration "${duration}" \
  --lateral "${lateral}" \
  --rear-gazebo-trim "${rear_trim}" \
  "$@"
