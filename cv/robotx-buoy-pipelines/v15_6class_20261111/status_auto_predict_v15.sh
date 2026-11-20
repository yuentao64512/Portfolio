#!/usr/bin/env bash
set -u

ROOT=/home/robotx/buoy_actual/v15_6class_20261111
PIPE_PID_FILE="$ROOT/auto_predict.pid"
STATUS="$ROOT/auto_predict_status.txt"
PIPE_LOG="$ROOT/logs/combine_and_predict_v15.log"

if [[ -f "$PIPE_PID_FILE" ]]; then
    PID=$(cat "$PIPE_PID_FILE")
    if kill -0 "$PID" 2>/dev/null; then
        echo "PIPELINE RUNNING pid=$PID"
    else
        echo "PIPELINE NOT RUNNING (last pid=$PID)"
    fi
else
    echo "Pipeline PID file not found"
fi

[[ -f "$STATUS" ]] && cat "$STATUS" || echo "No status yet"
echo
tail -n 20 "$PIPE_LOG" 2>/dev/null || true
