#!/usr/bin/env bash
set -Eeuo pipefail

ROOT=/home/robotx/buoy_actual/v16_8class_depth_20261112
RECORDINGS=/home/robotx/training_12
V14=/home/robotx/buoy_actual/boundary_synthetic_v14_20261111/runs/boundary_v14_baseline_yolo11m_1280_100ep/weights/best.pt
V15=/home/robotx/buoy_actual/v15_6class_20261111/runs/v15_r3_from_v14best_1280_150ep_20261111/weights/best.pt
V14_DATA=/home/robotx/buoy_actual/boundary_synthetic_v14_20261111/dataset
NEW_LABELS="$ROOT/autolabel_training12_v1"
DATASET="$ROOT/dataset_v1"
LOG="$ROOT/logs/prepare.log"

mkdir -p "$ROOT/logs"
exec >>"$LOG" 2>&1

timestamp() { date --iso-8601=seconds; }

declare -A EXPECTED_FILES=(
  [20261112_111929_obs]=204
  [20261112_113136_obs]=129
  [20261112_113936_obs]=147
)
declare -A EXPECTED_BYTES=(
  [20261112_111929_obs]=24008301
  [20261112_113136_obs]=17258024
  [20261112_113936_obs]=18269204
)

ready=0
while (( ready == 0 )); do
  ready=1
  echo "[$(timestamp)] checking training-ready transfer"
  for session in "${!EXPECTED_FILES[@]}"; do
    folder="$RECORDINGS/$session"
    files=0
    bytes=0
    if [[ -d "$folder" ]]; then
      files=$(find "$folder" -type f ! -name 'cam.svo2' | wc -l)
      bytes=$(find "$folder" -type f ! -name 'cam.svo2' -printf '%s\n' | awk '{s+=$1} END {print s+0}')
    fi
    echo "  $session files=$files/${EXPECTED_FILES[$session]} bytes=$bytes/${EXPECTED_BYTES[$session]}"
    if [[ "$files" -ne "${EXPECTED_FILES[$session]}" || "$bytes" -ne "${EXPECTED_BYTES[$session]}" ]]; then
      ready=0
    fi
  done
  (( ready == 1 )) || sleep 900
done

echo "[$(timestamp)] training-ready transfer complete"
touch "$ROOT/TRAINING_READY_TRANSFER_COMPLETE"

if [[ ! -f "$NEW_LABELS/summary.json" ]]; then
  echo "[$(timestamp)] generating conservative ensemble labels"
  /home/robotx/buoy_actual/.venv/bin/python "$ROOT/scripts/autolabel_training12_v16.py" \
    --recordings "$RECORDINGS" \
    --v14-model "$V14" \
    --v15-model "$V15" \
    --output "$NEW_LABELS" \
    --imgsz 1280 --conf 0.18
fi

if [[ ! -f "$DATASET/summary.json" ]]; then
  echo "[$(timestamp)] building non-destructive incremental dataset"
  PYTHONPATH="$ROOT/scripts" /home/robotx/buoy_actual/.venv/bin/python \
    "$ROOT/scripts/build_v16_incremental_dataset.py" \
    --v14 "$V14_DATA" \
    --new "$NEW_LABELS" \
    --output "$DATASET"
fi

echo "[$(timestamp)] labels and dataset ready for visual QA"
touch "$ROOT/LABELS_READY_FOR_QA"
