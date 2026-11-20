#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/Robotx_train
DATASET="$ROOT/two_weight_dataset_v2"
VENV=/home/robotx/buoy_actual/.venv
PROJECT="$ROOT/two_weight_training_v1"
STAMP="$(date +%Y%m%d_%H%M%S)"
DETECT_NAME="buoy_detector_yolo11m_1280_${STAMP}"
CLASS_NAME="marker_classifier_yolo11m_320_${STAMP}"
PIPELINE_LOG="$PROJECT/pipeline_${STAMP}.log"

mkdir -p "$PROJECT"
cat > "$PROJECT/current_runs.env" <<EOF
STAMP=$STAMP
DETECT_NAME=$DETECT_NAME
CLASS_NAME=$CLASS_NAME
PIPELINE_LOG=$PIPELINE_LOG
EOF

exec > >(tee -a "$PIPELINE_LOG") 2>&1
echo "[$(date --iso-8601=seconds)] two-weight training started"
echo "detector=$PROJECT/$DETECT_NAME"
echo "classifier=$PROJECT/$CLASS_NAME"

"$VENV/bin/yolo" detect train \
  model=/home/robotx/buoy_actual/v16_8class_depth_20261112/runs/v16b_balanced_from_v16best_1280_100ep_20261112/weights/best.pt \
  data="$DATASET/buoy_detector/data.yaml" \
  epochs=100 patience=20 imgsz=1280 batch=2 amp=True device=0 workers=8 \
  hsv_h=0.0 flipud=0.0 fliplr=0.5 \
  project="$PROJECT" name="$DETECT_NAME" exist_ok=False

touch "$PROJECT/DETECTOR_COMPLETE_${STAMP}"
echo "[$(date --iso-8601=seconds)] detector complete"

"$VENV/bin/yolo" classify train \
  model=yolo11m-cls.pt \
  data="$DATASET/marker_classifier" \
  epochs=100 patience=20 imgsz=320 batch=32 amp=True device=0 workers=8 \
  hsv_h=0.0 flipud=0.0 fliplr=0.5 \
  project="$PROJECT" name="$CLASS_NAME" exist_ok=False

touch "$PROJECT/CLASSIFIER_COMPLETE_${STAMP}"
touch "$PROJECT/ALL_TRAINING_COMPLETE_${STAMP}"
echo "[$(date --iso-8601=seconds)] all training complete"
echo "detector_best=$PROJECT/$DETECT_NAME/weights/best.pt"
echo "classifier_best=$PROJECT/$CLASS_NAME/weights/best.pt"
