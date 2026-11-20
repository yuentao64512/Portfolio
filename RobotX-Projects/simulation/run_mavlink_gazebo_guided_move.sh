#!/usr/bin/env bash
set -euo pipefail

# Guided-mode movement-only launcher. Start run_mavlink_gazebo.sh first.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/world_models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/worlds:${SCRIPT_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
export LD_LIBRARY_PATH="${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

if [[ "${1:-}" == "--dry-run" ]]; then
  shift
  python3 "${SCRIPT_DIR}/scripts/set_fcu_mode.py" --mode GUIDED
  python3 "${SCRIPT_DIR}/scripts/mavlink_gazebo_crab_bridge.py" "$@"
else
  python3 "${SCRIPT_DIR}/scripts/set_fcu_mode.py" --mode GUIDED
  python3 "${SCRIPT_DIR}/scripts/mavlink_gazebo_crab_bridge.py" --send "$@"
fi
