#!/usr/bin/env bash
set -u
ROOT="/home/robotx/buoy_actual"
PID_FILE="$ROOT/boat_yolo/background.pid"
LOG_FILE="$ROOT/boat_yolo/logs/setup_extract.log"

if [[ -f "$PID_FILE" ]]; then
  PID="$(cat "$PID_FILE")"
  if kill -0 "$PID" 2>/dev/null; then
    echo "status=running pid=$PID"
  else
    echo "status=stopped pid=$PID"
  fi
else
  echo "status=not_started"
fi

if [[ -f "$ROOT/boat_yolo/SETUP_AND_EXTRACTION_COMPLETE" ]]; then
  echo "setup_and_extraction=complete"
else
  echo "setup_and_extraction=incomplete"
fi

if [[ -d "$ROOT/boat_yolo/frames/images" ]]; then
  printf 'frame_count='
  find "$ROOT/boat_yolo/frames/images" -maxdepth 1 -type f | wc -l
fi

echo "log=$LOG_FILE"
[[ -f "$LOG_FILE" ]] && tail -n 25 "$LOG_FILE"
