#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual
RUN_FILE="$ROOT/boat_yolo/current_model_a_img3922_run.txt"

if [[ ! -f "$RUN_FILE" ]]; then
  echo "No IMG_3922 Model A run has been recorded." >&2
  exit 1
fi

run="$(cat "$RUN_FILE")"
pid_file="$ROOT/boat_yolo/${run}.pid"
log="$ROOT/boat_yolo/logs/${run}.log"
weights="$ROOT/boat_yolo/runs/${run}/weights"
pid="$(cat "$pid_file" 2>/dev/null || true)"

echo "run=$run"
echo "pid=${pid:-unknown}"
if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
  echo "status=RUNNING"
  ps -p "$pid" -o pid,etime,%cpu,%mem,cmd
else
  echo "status=FINISHED_OR_STOPPED"
fi

if [[ -f "$weights/best.pt" ]]; then
  echo "best=$weights/best.pt"
fi
if [[ -f "$weights/last.pt" ]]; then
  echo "last=$weights/last.pt"
fi

echo "--- recent log ---"
tail -n 30 "$log"
