#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual/boundary_synthetic_v14_20261111
RUN=boundary_v14_baseline_yolo11m_1280_100ep
LOG="$ROOT/train_baseline.log"
PIDFILE="$ROOT/train_baseline.pid"

if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  echo "Training is already running as PID $(cat "$PIDFILE")"
  exit 0
fi

nohup /home/robotx/buoy_actual/.venv/bin/yolo detect train \
  model=/home/robotx/buoy_actual/six_class_training_20261110/runs/robotx_six_class_yolo11m_1280_100ep/weights/best.pt \
  data="$ROOT/dataset/data.yaml" \
  epochs=100 patience=20 imgsz=1280 batch=2 amp=True device=0 workers=8 \
  hsv_h=0.0 hsv_s=0.35 hsv_v=0.4 degrees=6.0 flipud=0.0 fliplr=0.5 \
  project="$ROOT/runs" name="$RUN" exist_ok=False \
  >"$LOG" 2>&1 </dev/null &

echo $! >"$PIDFILE"
echo "Started PID $(cat "$PIDFILE")"
echo "Log: $LOG"
echo "Run: $ROOT/runs/$RUN"
