#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual
PROJECT="$ROOT/arena_dual_perception"
JOB="$ROOT/six_class_training_20261110"
DATA="$JOB/dataset"
RUN_NAME=robotx_six_class_yolo11m_1280_100ep

mkdir -p "$JOB"

"$ROOT/.venv/bin/python" "$PROJECT/prepare_six_class_dataset.py" \
  --reviewed "$ROOT/boat_yolo/datasets/model_a_plus_img3922_v1" \
  --media /home/robotx/robotx_do_not_touch \
  --output "$DATA" \
  --buoy-model "$PROJECT/weights/buoy_oneclass_v2_best.pt" \
  --nav-model "$PROJECT/weights/legacy_nav_6class_best.pt" \
  --fps 2 \
  --motion-threshold 1.8

"$ROOT/.venv/bin/python" - "$DATA/dataset_summary.json" <<'PY'
import json, sys
summary=json.load(open(sys.argv[1]))
required=('buoy_red','buoy_green','buoy_blue','buoy','dock','platform')
train=summary['counts']['train']
missing=[name for name in required if train.get(name,0)<20]
if missing:
    raise SystemExit('TRAINING BLOCKED: fewer than 20 training boxes for '+', '.join(missing))
print('Dataset gate passed:', train, flush=True)
PY

"$ROOT/.venv/bin/python" "$PROJECT/train_six_class.py" \
  --data "$DATA/data.yaml" \
  --project "$JOB/runs" \
  --name "$RUN_NAME" \
  --model /home/robotx/yolo11m.pt \
  --epochs 100 \
  --imgsz 1280 \
  --batch 2

touch "$JOB/TRAINING_COMPLETE"
