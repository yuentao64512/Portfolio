#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/robotx/buoy_actual"
RUN_FILE="$ROOT/boat_yolo/current_model_b_run.txt"
LOG_FILE="$ROOT/boat_yolo/current_model_b_log.txt"
PID_FILE="$ROOT/boat_yolo/train_model_b.pid"

if [[ ! -s "$RUN_FILE" || ! -s "$LOG_FILE" || ! -s "$PID_FILE" ]]; then
  echo "Model B training has not been launched."
  exit 1
fi

RUN="$(cat "$RUN_FILE")"
LOG="$(cat "$LOG_FILE")"
PID="$(cat "$PID_FILE")"
RUN_DIR="$ROOT/boat_yolo/runs/$RUN"

if kill -0 "$PID" 2>/dev/null; then
  echo "STATUS: RUNNING"
else
  echo "STATUS: FINISHED OR STOPPED"
fi
echo "RUN: $RUN"
echo "PID: $PID"
echo "LOG: $LOG"
echo "BEST: $RUN_DIR/weights/best.pt"
echo
echo "LATEST PROGRESS:"
tr '\r' '\n' < "$LOG" | grep -E '^[[:space:]]*[0-9]+/100|^[[:space:]]+all[[:space:]]|EarlyStopping|epochs completed|Results saved|Traceback|Error' | tail -n 20 || true
