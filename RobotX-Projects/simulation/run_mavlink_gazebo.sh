#!/usr/bin/env bash
set -euo pipefail

# Start the four-thruster Gazebo world only. Movement is controlled separately
# by run_mavlink_gazebo_crab.sh.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_ONLY=0
if [[ "${1:-}" == "--server-only" ]]; then
  SERVER_ONLY=1
elif [[ -n "${1:-}" ]]; then
  echo "Usage: $0 [--server-only]" >&2
  exit 2
fi

export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/world_models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/worlds:${SCRIPT_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
export LD_LIBRARY_PATH="${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/lib:${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

"${SCRIPT_DIR}/scripts/build_navigation_beacons.sh"

echo "Starting four-thruster Gazebo world"
echo "Leave this terminal running; use another terminal for movement."
if [[ "${SERVER_ONLY}" -eq 1 ]]; then
  exec gz sim -s -r -v 3 "${SCRIPT_DIR}/worlds/usv_four_thruster.world"
else
  exec gz sim -r -v 3 "${SCRIPT_DIR}/worlds/usv_four_thruster.world"
fi
