#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual
PROJECT="$ROOT/buoy_project"
SOURCE=/home/robotx/robotx_do_not_touch
JOB="$PROJECT/retrain_robotx_do_not_touch_20261110"
PYTHON="$ROOT/.venv/bin/python"

test -d "$SOURCE"
mkdir -p "$JOB"
find "$SOURCE" -type f -print0 | sort -z | xargs -0 sha256sum > "$JOB/source_before.sha256"

"$PYTHON" "$PROJECT/pseudolabel_robotx_folder.py" \
  --source "$SOURCE" \
  --output "$JOB/pseudo_full" \
  --buoy-weights "$PROJECT/runs/buoys_v1/weights/best.pt" \
  --shape-weights "$ROOT/boat_yolo/runs/model_b_1280_100ep_20261109_082016/weights/best.pt" \
  --digit-weights "$PROJECT/runs/digits_v1_legacy_provisional/weights/best.pt" \
  --fps 2 \
  --buoy-conf 0.25 \
  --diamond-conf 0.15 \
  --digit-conf 0.05 \
  --ocr-conf 20

find "$SOURCE" -type f -print0 | sort -z | xargs -0 sha256sum > "$JOB/source_after.sha256"
cmp "$JOB/source_before.sha256" "$JOB/source_after.sha256"

"$PYTHON" "$PROJECT/prepare_robotx_retrain.py" \
  --project "$PROJECT" \
  --pseudo-full "$JOB/pseudo_full" \
  --output "$JOB/merged"

"$PYTHON" "$PROJECT/retrain_robotx_folder.py" \
  --project "$PROJECT" \
  --dataset "$JOB/merged" \
  --run-name robotx_do_not_touch_v2

mkdir -p "$JOB/deploy/models"
cp "$PROJECT/runs/buoys_robotx_do_not_touch_v2/weights/best.pt" "$JOB/deploy/models/buoys_best.pt"
cp "$PROJECT/runs/digits_robotx_do_not_touch_v2_pseudo/weights/best.pt" "$JOB/deploy/models/digits_best.pt"
cp "$PROJECT/export_orin.py" "$PROJECT/predict_orin.py" "$JOB/deploy/"
cd "$JOB/deploy"
sha256sum models/*.pt export_orin.py predict_orin.py > SHA256SUMS
sha256sum -c SHA256SUMS
touch "$JOB/RETRAIN_COMPLETE"
