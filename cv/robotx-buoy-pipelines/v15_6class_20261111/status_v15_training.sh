#!/usr/bin/env bash
set -u

ROOT=/home/robotx/buoy_actual/v15_6class_20261111
PID_FILE="$ROOT/train_v15_r3.pid"
LOG_FILE="$ROOT/logs/v15_r3_from_v14best_1280_150ep_20261111.log"
RUN_DIR="$ROOT/runs/v15_r3_from_v14best_1280_150ep_20261111"

if [[ -f "$PID_FILE" ]]; then
    PID=$(cat "$PID_FILE")
    if kill -0 "$PID" 2>/dev/null; then
        echo "RUNNING pid=$PID"
        ps -p "$PID" -o pid=,etime=,stat=,cmd=
    else
        echo "NOT RUNNING (last pid=$PID)"
    fi
else
    echo "PID file not found"
fi

echo
nvidia-smi --query-gpu=name,memory.used,memory.total,utilization.gpu \
    --format=csv,noheader 2>/dev/null || true

echo
echo "Latest epoch rows:"
if [[ -f "$RUN_DIR/results.csv" ]]; then
    tail -n 4 "$RUN_DIR/results.csv"
else
    echo "results.csv not created yet"
fi

echo
echo "Latest log:"
tail -n 25 "$LOG_FILE" 2>/dev/null || echo "Log not created yet"

echo
if [[ -f "$RUN_DIR/weights/best.pt" ]]; then
    echo "Best weight: $RUN_DIR/weights/best.pt"
else
    echo "best.pt will appear after the first completed validation epoch"
fi
