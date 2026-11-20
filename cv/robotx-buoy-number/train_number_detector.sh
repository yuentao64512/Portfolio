#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/Robotx_train
VENV=/home/robotx/buoy_actual/.venv
DATA="$ROOT/number_detector_dataset_v1/data.yaml"
START_MODEL=/home/robotx/buoy_actual/arena_dual_perception/weights/digit_pseudo_v2_best.pt
PROJECT="$ROOT/number_detector_training_v1"
STAMP=$(date +%Y%m%d_%H%M%S)
RUN="number_detector_yolo11n_320_${STAMP}"
LOG="$PROJECT/${RUN}.log"
PID_FILE="$PROJECT/${RUN}.pid"

mkdir -p "$PROJECT"

nohup "$VENV/bin/yolo" detect train \
  model="$START_MODEL" \
  data="$DATA" \
  epochs=100 \
  patience=20 \
  imgsz=320 \
  batch=32 \
  amp=True \
  device=0 \
  workers=8 \
  hsv_h=0.0 \
  flipud=0.0 \
  degrees=20.0 \
  translate=0.10 \
  scale=0.40 \
  project="$PROJECT" \
  name="$RUN" \
  >"$LOG" 2>&1 </dev/null &

PID=$!
printf '%s\n' "$PID" >"$PID_FILE"
cat >"$PROJECT/current_run.env" <<EOF
RUN=$RUN
RUN_DIR=$PROJECT/$RUN
LOG=$LOG
PID=$PID
PID_FILE=$PID_FILE
EOF

echo "Started number-detector training"
echo "PID=$PID"
echo "RUN_DIR=$PROJECT/$RUN"
echo "LOG=$LOG"
