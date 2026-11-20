#!/usr/bin/env bash
set -euo pipefail

PROJECT=/home/jetson/pipe_cones
PYTHON=/home/jetson/fall_action_project/.venv/bin/python
BAUD=115200
MAVLINK=
CAMERA_YAW_DEG=0
CAMERA_FORWARD_M=0
CAMERA_RIGHT_M=0
CONF=0.15
RESOLUTION=hd2k
FPS=15
FRAMES=0
HEADLESS=0

usage() {
    cat <<'EOF'
Usage: run_pipe_cone_mapping.sh [options]
  --mavlink ENDPOINT          Serial path or udp:0.0.0.0:14550 (default: auto)
  --baud RATE                 Serial baud rate (default: 115200)
  --camera-yaw-deg DEGREES    Camera clockwise yaw from vehicle forward
  --camera-forward-m METERS   Camera forward offset from Pixhawk/EKF origin
  --camera-right-m METERS     Camera right offset from Pixhawk/EKF origin
  --conf VALUE                Detector threshold (default: 0.15)
  --resolution MODE           hd2k, hd1080, hd720, or vga (default: hd2k)
  --fps RATE                  Camera FPS (default: 15; HD2K only supports 15)
  --frames COUNT              Stop detector after COUNT frames; 0 runs continuously
  --headless                  No detector or map windows
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --mavlink) MAVLINK=$2; shift 2 ;;
        --baud) BAUD=$2; shift 2 ;;
        --camera-yaw-deg) CAMERA_YAW_DEG=$2; shift 2 ;;
        --camera-forward-m) CAMERA_FORWARD_M=$2; shift 2 ;;
        --camera-right-m) CAMERA_RIGHT_M=$2; shift 2 ;;
        --conf) CONF=$2; shift 2 ;;
        --resolution) RESOLUTION=$2; shift 2 ;;
        --fps) FPS=$2; shift 2 ;;
        --frames) FRAMES=$2; shift 2 ;;
        --headless) HEADLESS=1; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "Unknown argument: $1" >&2; usage >&2; exit 2 ;;
    esac
done

if [[ -z "$MAVLINK" ]]; then
    for candidate in \
        /dev/serial/by-id/*ArduPilot* \
        /dev/serial/by-id/*Pixhawk* \
        /dev/serial/by-id/*Cube* \
        /dev/ttyACM0 \
        /dev/ttyUSB0; do
        if [[ -e "$candidate" ]]; then
            MAVLINK=$candidate
            break
        fi
    done
fi

if [[ -z "$MAVLINK" ]]; then
    echo "No Pixhawk serial device found. Connect it or pass --mavlink ENDPOINT." >&2
    exit 1
fi

SESSION="$PROJECT/logs/launcher/$(date -u +%Y%m%d_%H%M%S)"
mkdir -p "$SESSION"
PIDS=()

cleanup() {
    trap - EXIT INT TERM
    for pid in "${PIDS[@]}"; do
        kill "$pid" 2>/dev/null || true
    done
    wait "${PIDS[@]}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Pixhawk: $MAVLINK @ $BAUD"
echo "Session logs: $SESSION"

"$PYTHON" "$PROJECT/pipe_cone_mavlink_observer.py" \
    --mavlink "$MAVLINK" --baud "$BAUD" \
    --camera-yaw-deg "$CAMERA_YAW_DEG" \
    --camera-forward-m "$CAMERA_FORWARD_M" \
    --camera-right-m "$CAMERA_RIGHT_M" \
    >"$SESSION/observer.log" 2>&1 &
PIDS+=("$!")

sleep 2
if ! kill -0 "${PIDS[0]}" 2>/dev/null; then
    cat "$SESSION/observer.log" >&2
    exit 1
fi

if [[ "$HEADLESS" -eq 0 ]]; then
    "$PYTHON" "$PROJECT/live_pipe_cone_map_window.py" \
        >"$SESSION/map_window.log" 2>&1 &
    PIDS+=("$!")
fi

DETECTOR_ARGS=(
    --conf "$CONF"
    --resolution "$RESOLUTION"
    --fps "$FPS"
    --frames "$FRAMES"
    --observer-udp 127.0.0.1:15250
)
if [[ "$HEADLESS" -eq 1 ]]; then
    DETECTOR_ARGS+=(--headless)
fi

"$PYTHON" "$PROJECT/live_pipe_cone_map_yolo26_zed.py" \
    "${DETECTOR_ARGS[@]}" >"$SESSION/detector.log" 2>&1 &
PIDS+=("$!")

echo "Mapping is running. Close a window or press Ctrl+C to stop."
wait -n "${PIDS[@]}"
