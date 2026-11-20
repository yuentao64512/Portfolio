#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/Robotx_train
JOB="$ROOT/photo_214805_training_v1"
VENV=/home/robotx/buoy_actual/.venv
LOG="$JOB/training.log"
STATUS="$JOB/status.env"

exec > >(tee -a "$LOG") 2>&1
echo "STARTED_AT=$(date -Iseconds)"

cat >"$STATUS" <<EOF
STATE=TRAINING_BUOY_DETECTOR
EOF

"$VENV/bin/yolo" detect train \
  model="$ROOT/two_weight_training_v1/buoy_detector_yolo11m_1280_20261112_204219/weights/best.pt" \
  data="$JOB/buoy_detector_dataset/data.yaml" \
  epochs=50 patience=15 imgsz=1280 batch=2 amp=True device=0 workers=8 \
  hsv_h=0.0 flipud=0.0 \
  project="$JOB/runs" name=buoy_detector_photo_214805 exist_ok=True

cat >"$STATUS" <<EOF
STATE=TRAINING_MARKER_CLASSIFIER
EOF

"$VENV/bin/yolo" classify train \
  model="$ROOT/two_weight_training_v1/marker_classifier_yolo11m_320_20261112_204219/weights/best.pt" \
  data="$JOB/marker_classifier_dataset" \
  epochs=30 patience=10 imgsz=320 batch=32 amp=True device=0 workers=8 \
  hsv_h=0.0 flipud=0.0 \
  project="$JOB/runs" name=marker_classifier_photo_214805 exist_ok=True

cat >"$STATUS" <<EOF
STATE=TRAINING_NUMBER_DETECTOR
EOF

"$VENV/bin/yolo" detect train \
  model="$ROOT/number_detector_training_v1/number_detector_yolo11n_320_20261112_213348/weights/best.pt" \
  data="$JOB/number_detector_dataset/data.yaml" \
  epochs=50 patience=15 imgsz=320 batch=32 amp=True device=0 workers=8 \
  hsv_h=0.0 flipud=0.0 \
  project="$JOB/runs" name=number_detector_photo_214805 exist_ok=True

cat >"$STATUS" <<EOF
STATE=COMPLETE
BUOY_WEIGHT=$JOB/runs/buoy_detector_photo_214805/weights/best.pt
MARKER_WEIGHT=$JOB/runs/marker_classifier_photo_214805/weights/best.pt
NUMBER_WEIGHT=$JOB/runs/number_detector_photo_214805/weights/best.pt
EOF
touch "$JOB/ALL_PHOTO_TRAINING_COMPLETE"
echo "ALL PHOTO TRAINING COMPLETE"
