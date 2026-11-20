#!/usr/bin/env bash
set -euo pipefail

# Passive float-only launch.
# This starts Gazebo physics and the selected USV, but never starts the
# controller or publishes thruster commands.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-four}"
SERVER_ONLY=0

case "${MODE}" in
  four|four_thruster)
    WORLD="${SCRIPT_DIR}/worlds/usv_four_thruster.world"
    MODEL="usv_four_thruster"
    ;;
  two|two_thruster)
    WORLD="${SCRIPT_DIR}/worlds/usv_two_thruster.world"
    MODEL="usv_two_thruster"
    ;;
  *)
    echo "Usage: $0 [four|two] [--server-only]" >&2
    exit 2
    ;;
esac

if [[ "${2:-}" == "--server-only" ]]; then
  SERVER_ONLY=1
elif [[ -n "${2:-}" ]]; then
  echo "Usage: $0 [four|two] [--server-only]" >&2
  exit 2
fi

export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/world_models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/worlds:${SCRIPT_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
export LD_LIBRARY_PATH="${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/lib:${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

"${SCRIPT_DIR}/scripts/build_navigation_beacons.sh"

echo "Starting passive float test for ${MODEL}"
echo "No controller or thruster commands will be sent."

if [[ "${SERVER_ONLY}" -eq 1 ]]; then
  exec gz sim -s -r -v 3 "${WORLD}"
else
  exec gz sim -r -v 3 "${WORLD}"
fi
