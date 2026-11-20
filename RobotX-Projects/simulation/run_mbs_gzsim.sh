#!/usr/bin/env bash

set -e


SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_GAZEBO_DIR="${ARDUPILOT_GAZEBO_DIR:-${SCRIPT_DIR}/../ardupilot_gazebo}"


# Fix the typo: usv_sim_workplace -> usv_sim_workspace

export GZ_SIM_RESOURCE_PATH="$SCRIPT_DIR/models:$SCRIPT_DIR/worlds:$SCRIPT_DIR/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/models:$SCRIPT_DIR/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/world_models:$SCRIPT_DIR/asv_wave_sim_ws/src/asv_wave_sim/gz-waves-models/worlds:$SCRIPT_DIR:$GZ_SIM_RESOURCE_PATH"

export LD_LIBRARY_PATH="$SCRIPT_DIR/asv_wave_sim_ws/install/lib:$SCRIPT_DIR/asv_wave_sim_ws/build/lib:$LD_LIBRARY_PATH"

export GZ_SIM_SYSTEM_PLUGIN_PATH="${SCRIPT_DIR}/asv_wave_sim_ws/build:${SCRIPT_DIR}/asv_wave_sim_ws/install/lib:${ARDUPILOT_GAZEBO_DIR}/build:/usr/local/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"


cd "$SCRIPT_DIR"


# ------------------------------------------------------------

# Launch Gazebo in the background

# ------------------------------------------------------------

echo "[1/2] Launching Gazebo..."

gz sim -r -v 4 worlds/mbs_waterfront_sitl.world &

GZ_PID=$!


# Give Gazebo time to initialize fully

sleep 5


# ------------------------------------------------------------

# Run the Python controller in the foreground

# ------------------------------------------------------------

echo "[2/2] Launching Python Controller..."

echo "Press Ctrl+C to stop everything."

echo "======================================================"


python3 "$SCRIPT_DIR/src/usv_controller.py"


# When the Python script exits (e.g., Ctrl+C), kill Gazebo

echo "Stopping Gazebo..."

kill $GZ_PID 2>/dev/null

wait $GZ_PID 2>/dev/null

echo "Simulation stopped."
