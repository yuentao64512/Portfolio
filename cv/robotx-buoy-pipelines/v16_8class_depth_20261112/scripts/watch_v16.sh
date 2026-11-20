#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual
WORK="$ROOT/v16_8class_depth_20261112"
PID_FILE="$WORK/train_v16.pid"
WATCH_LOG="$WORK/logs/watch_v16.log"
VALIDATE_LOG="$WORK/logs/validate_v16.log"
RUN=$(cat "$WORK/current_train_run.txt")
BEST="$WORK/runs/$RUN/weights/best.pt"

while [[ ! -s "$PID_FILE" ]]; do sleep 5; done
PID=$(cat "$PID_FILE")
while kill -0 "$PID" 2>/dev/null; do
    {
        date --iso-8601=seconds
        grep -E '^[[:space:]]*[0-9]+/[0-9]+' "$WORK/logs/train_v16.log" 2>/dev/null | tail -n 1 || true
    } >>"$WATCH_LOG"
    sleep 900
done

{
    date --iso-8601=seconds
    echo "Training process ended; starting held-out validation"
} >>"$WATCH_LOG"

if [[ ! -f "$BEST" ]]; then
    echo "No best.pt was produced" >>"$WATCH_LOG"
    exit 1
fi

"$ROOT/.venv/bin/yolo" detect val \
    model="$BEST" \
    data="$WORK/dataset_v12/data.yaml" \
    split=test imgsz=1280 batch=2 device=0 workers=8 \
    project="$WORK/validation" name=new_depth_session exist_ok=True \
    >"$VALIDATE_LOG" 2>&1

"$ROOT/.venv/bin/yolo" detect val \
    model="$BEST" \
    data="$WORK/dataset_v12/data_old_test.yaml" \
    split=test imgsz=1280 batch=2 device=0 workers=8 \
    project="$WORK/validation" name=legacy_test exist_ok=True \
    >>"$VALIDATE_LOG" 2>&1

date --iso-8601=seconds >"$WORK/VALIDATION_COMPLETE"
echo "Validation complete" >>"$WATCH_LOG"

