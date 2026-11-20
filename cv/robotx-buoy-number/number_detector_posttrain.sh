#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/Robotx_train
TRAIN_ROOT="$ROOT/number_detector_training_v1"
source "$TRAIN_ROOT/current_run.env"
POST_LOG="$TRAIN_ROOT/${RUN}_posttrain.log"

while kill -0 "$PID" 2>/dev/null; do
  sleep 15
done

BEST="$RUN_DIR/weights/best.pt"
if [[ ! -f "$BEST" ]]; then
  echo "Training ended without best.pt; see $LOG" >"$POST_LOG"
  exit 1
fi

BUOY_BEST="$ROOT/two_weight_training_v1/buoy_detector_yolo11m_1280_20261112_204219/weights/best.pt"
SOURCE="$ROOT/two_weight_training_v1/IMG_3969.MOV"
OUT_DIR="$TRAIN_ROOT/predictions"
mkdir -p "$OUT_DIR"

/home/robotx/buoy_actual/.venv/bin/python "$ROOT/run_number_detector_video.py" \
  --source "$SOURCE" \
  --buoy-detector "$BUOY_BEST" \
  --number-detector "$BEST" \
  --output "$OUT_DIR/IMG_3969_number_detector_annotated.mp4" \
  --csv "$OUT_DIR/IMG_3969_number_detections.csv" \
  >"$POST_LOG" 2>&1

touch "$TRAIN_ROOT/NUMBER_DETECTOR_PIPELINE_COMPLETE"
