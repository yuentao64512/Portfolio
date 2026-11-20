#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-four}"
CONTROL="${2:-interactive}"

case "${MODE}" in
  four|four_thruster)
    WORLD="${SCRIPT_DIR}/worlds/usv_four_thruster.world"
    MODEL="usv_four_thruster"
    CONTROLLER_ARGS=()
    ;;
  two|two_thruster)
    WORLD="${SCRIPT_DIR}/worlds/usv_two_thruster.world"
    MODEL="usv_two_thruster"
    CONTROLLER_ARGS=(--two-thruster)
    ;;
  *)
    echo "Usage: $0 [four|two] [interactive|demo] [--server-only]" >&2
    exit 2
    ;;
esac

SERVER_ONLY=0
if [[ "${3:-}" == "--server-only" ]]; then
  SERVER_ONLY=1
fi

export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/world_models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/worlds:${SCRIPT_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
export LD_LIBRARY_PATH="${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/lib:${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

"${SCRIPT_DIR}/scripts/build_navigation_beacons.sh"

cleanup() {
  if [[ -n "${GZ_PID:-}" ]]; then
    kill "${GZ_PID}" 2>/dev/null || true
    wait "${GZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Launching ${MODEL} on the shared MBS waterfront map"
if [[ "${SERVER_ONLY}" -eq 1 ]]; then
  gz sim -s -r -v 3 "${WORLD}" &
else
  gz sim -r -v 3 "${WORLD}" &
fi
GZ_PID=$!
sleep 5

if [[ "${SERVER_ONLY}" -eq 1 ]]; then
  wait "${GZ_PID}"
elif [[ "${CONTROL}" == "demo" ]]; then
  python3 "${SCRIPT_DIR}/scripts/usv_controller.py" --model "${MODEL}" "${CONTROLLER_ARGS[@]}" --demo
else
  python3 "${SCRIPT_DIR}/scripts/usv_controller.py" --model "${MODEL}" "${CONTROLLER_ARGS[@]}"
fi
