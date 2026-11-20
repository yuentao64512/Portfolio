#!/usr/bin/env bash
set -u

ROOT=/home/robotx/buoy_actual/boundary_synthetic_v14_20261111
PIDFILE="$ROOT/train_baseline.pid"
LOG="$ROOT/train_baseline.log"
RUN="$ROOT/runs/boundary_v14_baseline_yolo11m_1280_100ep"

if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  echo "RUNNING PID $(cat "$PIDFILE")"
else
  echo "NOT RUNNING"
fi

if [[ -f "$RUN/weights/best.pt" ]]; then
  ls -lh "$RUN/weights/best.pt"
else
  echo "best.pt not created yet"
fi

echo "--- latest log ---"
tail -n 25 "$LOG" 2>/dev/null || true
