#!/usr/bin/env bash
set -euo pipefail

# Hardware-in-the-loop path:
# Pixhawk MAVLink command -> SERVO_OUTPUT_RAW -> Gazebo thruster topics.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SERVER_ONLY=1
if [[ "${1:-}" == "--gui" ]]; then
  SERVER_ONLY=0
  shift
fi

export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/world_models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/worlds:${SCRIPT_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
export LD_LIBRARY_PATH="${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/lib:${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

"${SCRIPT_DIR}/scripts/build_navigation_beacons.sh"

GZ_ARGS=(-r -v 3 "${SCRIPT_DIR}/worlds/usv_four_thruster.world")
if [[ "${SERVER_ONLY}" -eq 1 ]]; then
  GZ_ARGS=(-s "${GZ_ARGS[@]}")
fi

cleanup() {
  if [[ -n "${GZ_PID:-}" ]]; then
    kill "${GZ_PID}" 2>/dev/null || true
    wait "${GZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting four-thruster Gazebo with Pixhawk MAVLink bridge"
gz sim "${GZ_ARGS[@]}" &
GZ_PID=$!
sleep 4

python3 "${SCRIPT_DIR}/scripts/mavlink_gazebo_crab_bridge.py" "$@"
