#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual
WORK="$ROOT/v16_8class_depth_20261112"
DATA="$WORK/dataset_v12/data.yaml"
MODEL="$ROOT/v15_6class_20261111/runs/v15_r3_from_v14best_1280_150ep_20261111/weights/best.pt"
RUN=v16_from_v15best_8class_1280_100ep_20261112
LOG="$WORK/logs/train_v16.log"
PID_FILE="$WORK/train_v16.pid"

mkdir -p "$WORK/logs" "$WORK/runs"
if [[ -s "$PID_FILE" ]] && kill -0 "$(cat "$PID_FILE")" 2>/dev/null; then
    echo "Training is already running as PID $(cat "$PID_FILE")"
    exit 0
fi

test -f "$MODEL"
test -f "$DATA"

nohup "$ROOT/.venv/bin/yolo" detect train \
    model="$MODEL" \
    data="$DATA" \
    epochs=100 \
    patience=25 \
    imgsz=1280 \
    batch=2 \
    amp=True \
    device=0 \
    workers=8 \
    hsv_h=0.0 \
    hsv_s=0.30 \
    hsv_v=0.40 \
    flipud=0.0 \
    fliplr=0.5 \
    cls_remap=True \
    project="$WORK/runs" \
    name="$RUN" \
    exist_ok=True \
    >"$LOG" 2>&1 </dev/null &

echo $! >"$PID_FILE"
echo "$RUN" >"$WORK/current_train_run.txt"
echo "Started PID $(cat "$PID_FILE")"
echo "Log: $LOG"

