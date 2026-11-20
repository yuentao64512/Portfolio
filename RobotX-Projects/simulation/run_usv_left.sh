#!/usr/bin/env bash
set -euo pipefail

# Start the four-thruster USV and perform a passive-heading left crab.
# Port/starboard propulsion is held at zero; bow and stern lateral thrusters
# receive equal commands, so there is no intentional yaw input.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DURATION="${1:-8}"
MAGNITUDE="${2:-7}"
REAR_GAIN="1.15"
SERVER_ONLY=0

if [[ "${3:-}" == "--server-only" ]]; then
  SERVER_ONLY=1
elif [[ -n "${3:-}" ]]; then
  REAR_GAIN="${3}"
  if [[ "${4:-}" == "--server-only" ]]; then
    SERVER_ONLY=1
  elif [[ -n "${4:-}" ]]; then
    echo "Usage: $0 [duration_seconds] [lateral_thrust_magnitude] [rear_gain] [--server-only]" >&2
    exit 2
  fi
fi

if [[ -n "${5:-}" ]]; then
  echo "Usage: $0 [duration_seconds] [lateral_thrust_magnitude] [rear_gain] [--server-only]" >&2
  exit 2
fi

export GZ_SIM_RESOURCE_PATH="${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/world_models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/worlds:${SCRIPT_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
export LD_LIBRARY_PATH="${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/lib:${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

"${SCRIPT_DIR}/scripts/build_navigation_beacons.sh"

WORLD="${SCRIPT_DIR}/worlds/usv_four_thruster.world"
GZ_PID=""

cleanup() {
  if [[ -n "${GZ_PID}" ]]; then
    kill -INT "${GZ_PID}" 2>/dev/null || true
    wait "${GZ_PID}" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "Starting four-thruster USV left-crab test"
echo "Duration: ${DURATION}s, lateral magnitude: ${MAGNITUDE}, rear gain: ${REAR_GAIN}"
echo "Port/starboard thrust: 0; front/rear lateral thrust: same direction with rear trim"

if [[ "${SERVER_ONLY}" -eq 1 ]]; then
  gz sim -s -r -v 3 "${WORLD}" &
else
  gz sim -r -v 3 "${WORLD}" &
fi
GZ_PID=$!
sleep 5

python3 "${SCRIPT_DIR}/scripts/usv_left_controller.py" \
  --model usv_four_thruster \
  --duration "${DURATION}" \
  --magnitude "${MAGNITUDE}" \
  --rear-gain "${REAR_GAIN}"

echo "Boat stopped. Gazebo is still running; press Ctrl+C to close it."
wait "${GZ_PID}"
