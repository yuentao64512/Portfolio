#!/usr/bin/env bash
set -euo pipefail

WORK=/home/robotx/buoy_actual/v16_8class_depth_20261112
PID_FILE="$WORK/train_v16.pid"
LOG="$WORK/logs/train_v16.log"
RUN=$(cat "$WORK/current_train_run.txt" 2>/dev/null || true)

if [[ -s "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "RUNNING PID $(cat "$PID_FILE")"
else
    echo "NOT RUNNING"
fi
echo "Run: ${RUN:-unknown}"
echo "Best: $WORK/runs/${RUN:-unknown}/weights/best.pt"
echo "Latest progress:"
grep -E '^[[:space:]]*[0-9]+/[0-9]+' "$LOG" 2>/dev/null | tail -n 3 || true
tail -n 12 "$LOG" 2>/dev/null || true

