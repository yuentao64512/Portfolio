#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual
WORK="$ROOT/v16_8class_depth_20261112"
PID_FILE="$WORK/train_v16b_balanced.pid"
WATCH_LOG="$WORK/logs/watch_v16b_balanced.log"
VALIDATE_LOG="$WORK/logs/validate_v16b_balanced.log"
RUN=$(cat "$WORK/current_balanced_run.txt")
BEST="$WORK/runs/$RUN/weights/best.pt"

while [[ ! -s "$PID_FILE" ]]; do sleep 5; done
PID=$(cat "$PID_FILE")
while kill -0 "$PID" 2>/dev/null; do
    {
        date --iso-8601=seconds
        grep -E '^[[:space:]]*[0-9]+/[0-9]+' "$WORK/logs/train_v16b_balanced.log" 2>/dev/null | tail -n 1 || true
    } >>"$WATCH_LOG"
    sleep 900
done

{
    date --iso-8601=seconds
    echo "Balanced training ended; starting validation"
} >>"$WATCH_LOG"

if [[ ! -f "$BEST" ]]; then
    echo "No balanced best.pt was produced" >>"$WATCH_LOG"
    exit 1
fi

"$ROOT/.venv/bin/yolo" detect val \
    model="$BEST" data="$WORK/dataset_balanced_final/data.yaml" \
    split=val imgsz=1280 batch=2 device=0 workers=8 \
    project="$WORK/validation" name=balanced_combined_val exist_ok=True \
    >"$VALIDATE_LOG" 2>&1

"$ROOT/.venv/bin/yolo" detect val \
    model="$BEST" data="$WORK/dataset_balanced_final/data.yaml" \
    split=test imgsz=1280 batch=2 device=0 workers=8 \
    project="$WORK/validation" name=balanced_new_depth_test exist_ok=True \
    >>"$VALIDATE_LOG" 2>&1

"$ROOT/.venv/bin/yolo" detect predict \
    model="$BEST" source="$WORK/dataset_balanced_final/images/test_new" \
    imgsz=1280 conf=0.15 device=0 save=True save_txt=True save_conf=True \
    project="$WORK/predictions" name=balanced_new_depth_test exist_ok=True \
    >>"$VALIDATE_LOG" 2>&1

date --iso-8601=seconds >"$WORK/BALANCED_VALIDATION_COMPLETE"
echo "Balanced validation complete" >>"$WATCH_LOG"

