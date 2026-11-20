#!/usr/bin/env bash
set -euo pipefail

# Launch one Gazebo waterfront scene containing the four-thruster boat and a
# separate Iris drone.  The drone uses ArduCopter SITL on TCP 5760 and UDP
# 9002/9003; the boat remains controlled through its existing Pixhawk bridge.

PACKAGE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SCRIPT_DIR="$(cd "${PACKAGE_DIR}/.." && pwd)"
exec 9>"${PACKAGE_DIR}/.simulation.lock"
if ! flock -n 9; then
  echo "RoboCommand simulation is already running. Stop it before starting another copy." >&2
  exit 2
fi
TIER="${1:-advanced}"
if [[ "${TIER}" != "core" && "${TIER}" != "advanced" && "${TIER}" != "disruptive" ]]; then
  echo "Usage: $0 core|advanced|disruptive" >&2
  exit 2
fi
WORLD_VARIANT="advanced"
[[ "${TIER}" == "core" ]] && WORLD_VARIANT="core"
python3 "${PACKAGE_DIR}/scripts/build_world_variants.py" \
  --source "${SCRIPT_DIR}/worlds/usv_four_thruster.world" \
  --output-dir "${PACKAGE_DIR}/worlds" >/dev/null
WORLD_FILE="${PACKAGE_DIR}/worlds/task1_${WORLD_VARIANT}.world"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-${SCRIPT_DIR}/../ardupilot}"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${SCRIPT_DIR}/../ardupilot_gazebo}"
ARDUPILOT_PYTHON="${ARDUPILOT_PYTHON:-/home/robotx/venv-ardupilot/bin/python}"
SITL_DIR="$(mktemp -d /tmp/usv-drone-sitl.XXXXXX)"
GZ_PID=""
SITL_PID=""

cleanup() {
  set +e
  if [[ -n "${SITL_PID}" ]]; then
    kill "${SITL_PID}" 2>/dev/null
    wait "${SITL_PID}" 2>/dev/null
  fi
  if [[ -n "${GZ_PID}" ]]; then
    kill "${GZ_PID}" 2>/dev/null
    wait "${GZ_PID}" 2>/dev/null
  fi
  echo "Logs retained in ${SITL_DIR}"
}
trap cleanup EXIT INT TERM

create_beacon() {
  local name="$1"
  local x="$2"
  local y="$3"
  local r="$4"
  local g="$5"
  local b="$6"
  local light_sdf="<sdf version='1.9'><light name='${name}' type='point'><pose>${x} ${y} 1.8 0 0 0</pose><diffuse>${r} ${g} ${b} 1</diffuse><specular>${r} ${g} ${b} 1</specular><attenuation><range>18</range><constant>0.4</constant><linear>0.05</linear><quadratic>0.01</quadratic></attenuation></light></sdf>"
  gz service -s /world/usv_four_thruster/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf: \"${light_sdf}\""
}

if gz service -l 2>/dev/null | grep -q '/world/usv_four_thruster/create$'; then
  echo "The usv_four_thruster Gazebo world is already running." >&2
  echo "Stop the existing Gazebo instance before using this combined launcher." >&2
  exit 2
fi

export GZ_SIM_RESOURCE_PATH="${PACKAGE_DIR}/models:${ARDUPILOT_GAZEBO_DIR}/models:${SCRIPT_DIR}/models:${SCRIPT_DIR}/worlds:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/world_models:${SCRIPT_DIR}/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/worlds:${SCRIPT_DIR}:${GZ_SIM_RESOURCE_PATH:-}"
export LD_LIBRARY_PATH="${ARDUPILOT_GAZEBO_DIR}/build:${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:${LD_LIBRARY_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO_DIR}/build:${SCRIPT_DIR}/lib:${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${SCRIPT_DIR}/asv_wave_sim_ws/build/lib:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"

"${SCRIPT_DIR}/scripts/build_navigation_beacons.sh" >"${SITL_DIR}/beacon_build.log" 2>&1

echo "Starting combined boat + drone Gazebo world"
GZ_OPTIONS=(-r -v 3)
if [[ "${SIM_HEADLESS:-0}" == "1" || ( -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ) ]]; then
  echo "No graphical display requested/available; using Gazebo server mode"
  GZ_OPTIONS=(-s "${GZ_OPTIONS[@]}")
fi
gz sim "${GZ_OPTIONS[@]}" "${WORLD_FILE}" >"${SITL_DIR}/gazebo.log" 2>&1 &
GZ_PID=$!

for _ in {1..150}; do
  if ! kill -0 "${GZ_PID}" 2>/dev/null; then
    echo "Gazebo exited before the world was ready; see ${SITL_DIR}/gazebo.log" >&2
    exit 3
  fi
  if gz service -l 2>/dev/null | grep -q '/world/usv_four_thruster/create$'; then
    break
  fi
  sleep 0.2
done

if ! gz service -l 2>/dev/null | grep -q '/world/usv_four_thruster/create$'; then
  echo "Gazebo world did not provide its create service." >&2
  exit 3
fi

echo "Creating drone landing pad on the north land area"
gz service -s /world/usv_four_thruster/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"${SCRIPT_DIR}/models/drone_landing_pad/model.sdf\", name: \"drone_landing_pad\", pose: { position: { x: 0, y: 76, z: 0.3 }, orientation: { w: 1 } }, allow_renaming: false"

echo "Creating RED, GREEN, and BLUE boat docking stations near the drone landing pad"
for dock in red green blue; do
  case "${dock}" in
    red) dock_x=-14 ;;
    green) dock_x=0 ;;
    blue) dock_x=14 ;;
  esac
  gz service -s /world/usv_four_thruster/create \
    --reqtype gz.msgs.EntityFactory \
    --reptype gz.msgs.Boolean \
    --timeout 5000 \
    --req "sdf_filename: \"${SCRIPT_DIR}/models/boat_dock_${dock}/model.sdf\", name: \"boat_dock_${dock}\", pose: { position: { x: ${dock_x}, y: 45, z: 0.25 }, orientation: { w: 1 } }, allow_renaming: false"
  case "${dock}" in
    red) create_beacon boat_dock_red_beacon_left $((dock_x - 3)) 41.5 1.0 0.0 0.0; create_beacon boat_dock_red_beacon_right $((dock_x + 3)) 41.5 1.0 0.0 0.0 ;;
    green) create_beacon boat_dock_green_beacon_left $((dock_x - 3)) 41.5 0.0 1.0 0.0; create_beacon boat_dock_green_beacon_right $((dock_x + 3)) 41.5 0.0 1.0 0.0 ;;
    blue) create_beacon boat_dock_blue_beacon_left $((dock_x - 3)) 41.5 0.0 0.2 1.0; create_beacon boat_dock_blue_beacon_right $((dock_x + 3)) 41.5 0.0 0.2 1.0 ;;
  esac
done

echo "Creating 10 m high back wall behind the red-to-blue docking area"
gz service -s /world/usv_four_thruster/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"${SCRIPT_DIR}/models/dock_back_wall/model.sdf\", name: \"dock_back_wall\", pose: { position: { x: 0, y: 50, z: 5.0 }, orientation: { w: 1 } }, allow_renaming: false"

echo "Inserting Iris drone on land at x=0, y=76, z=1.0"
gz service -s /world/usv_four_thruster/create \
  --reqtype gz.msgs.EntityFactory \
  --reptype gz.msgs.Boolean \
  --timeout 5000 \
  --req "sdf_filename: \"${ARDUPILOT_GAZEBO_DIR}/models/iris_with_gimbal/model.sdf\", name: \"iris_drone\", pose: { position: { x: 0, y: 76, z: 1.0 }, orientation: { w: 1 } }, allow_renaming: false"

echo "Starting ArduCopter SITL for the drone"
cd "${ARDUPILOT_DIR}"
"${ARDUPILOT_PYTHON}" Tools/autotest/sim_vehicle.py \
  -N -v ArduCopter -f gazebo-iris --model JSON \
  --no-mavproxy --no-extra-ports --use-dir="${SITL_DIR}" \
  --add-param-file="${PACKAGE_DIR}/config/task1_copter.parm" \
  >"${SITL_DIR}/arducopter.log" 2>&1 &
SITL_PID=$!

# Keep a real telemetry connection until the first heartbeat. A connect/close
# port probe can leave SITL waiting for a client and freeze Gazebo lock-step.
"${ARDUPILOT_PYTHON}" - <<'PY'
import time
from pymavlink import mavutil

deadline = time.monotonic() + 30
while time.monotonic() < deadline:
    connection = None
    try:
        connection = mavutil.mavlink_connection("tcp:127.0.0.1:5760", source_system=254)
        if connection.wait_heartbeat(timeout=3) is not None:
            print("Drone SITL heartbeat received; simulation connection ready.")
            break
    except OSError:
        time.sleep(0.2)
    finally:
        if connection is not None:
            connection.close()
else:
    raise SystemExit("Drone SITL did not provide a heartbeat within 30 seconds.")
PY

echo "Combined simulation is ready."
echo "Boat: use the existing Pixhawk/OmniPlus bridge."
echo "Drone: use another terminal with:"
echo "  bash ${SCRIPT_DIR}/run_drone_guided.sh takeoff 5"
echo "  bash ${SCRIPT_DIR}/run_drone_guided.sh move 1.0 0.0 0.0 5"
echo "  bash ${SCRIPT_DIR}/run_drone_guided.sh land"
echo "Press Ctrl+C here to stop Gazebo and drone SITL."

wait "${SITL_PID}"
