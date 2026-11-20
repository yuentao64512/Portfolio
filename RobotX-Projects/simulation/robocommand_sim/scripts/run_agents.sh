#!/usr/bin/env bash
set -euo pipefail
PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
exec 9>"${PKG}/.agents.lock"
if ! flock -n 9; then
  echo "RoboCommand agents are already running. Do not start a second copy." >&2
  exit 2
fi
export PYTHONPATH="${PKG}:${PYTHONPATH:-}"
LOG_DIR="${PKG}/logs"
mkdir -p "${LOG_DIR}"

cleanup() {
  set +e
  [[ -n "${UAV_PID:-}" ]] && kill "${UAV_PID}" 2>/dev/null
  [[ -n "${BOAT_PID:-}" ]] && kill "${BOAT_PID}" 2>/dev/null
  [[ -n "${LIGHT_PID:-}" ]] && kill "${LIGHT_PID}" 2>/dev/null
  [[ -n "${FLIGHT_PID:-}" ]] && kill "${FLIGHT_PID}" 2>/dev/null
  [[ -n "${DRIVER_PID:-}" ]] && kill "${DRIVER_PID}" 2>/dev/null
  wait 2>/dev/null
}
trap cleanup EXIT INT TERM

python3 -m robocommand.uav_perception --config "${PKG}/config/task1_world.yaml" "$@" 2>&1 | tee "${LOG_DIR}/uav_perception.log" &
UAV_PID=$!
python3 -m robocommand.boat_agent --config "${PKG}/config/task1_world.yaml" 2>&1 | tee "${LOG_DIR}/boat_agent.log" &
BOAT_PID=$!
if [[ "${SIM_BOAT_DRIVE:-1}" == "1" ]]; then
  if pgrep -f 'mavlink.*gazebo.*bridge.py|rc_mavlink_gazebo_bridge.py|usv_controller.py' >/dev/null; then
    echo "An existing Gazebo actuator bridge is active; refusing a second boat controller." >&2
    exit 3
  fi
  python3 -m robocommand.sim_boat_driver >"${LOG_DIR}/sim_boat_driver.log" 2>&1 &
  DRIVER_PID=$!
fi
python3 -m robocommand.light_ctl 2>&1 | tee "${LOG_DIR}/light_ctl.log" &
LIGHT_PID=$!
"/home/robotx/venv-ardupilot/bin/python" -m robocommand.uav_survey_flight --config "${PKG}/config/task1_world.yaml" --hover-s 4.0 2>&1 | tee "${LOG_DIR}/uav_flight.log" &
FLIGHT_PID=$!
echo "agents started perception=${UAV_PID} flight=${FLIGHT_PID} boat=${BOAT_PID} driver=${DRIVER_PID:-disabled} light_ctl=${LIGHT_PID}"
wait -n "${UAV_PID}" "${FLIGHT_PID}" "${BOAT_PID}" "${LIGHT_PID}" ${DRIVER_PID:+"${DRIVER_PID}"}

