#!/usr/bin/env bash
set -euo pipefail

JOB=/home/robotx/Robotx_train/photo_214805_training_v1
if [[ -f "$JOB/status.env" ]]; then
  cat "$JOB/status.env"
else
  echo "STATE=NOT_STARTED"
fi
if [[ -f "$JOB/pipeline.pid" ]]; then
  PID=$(cat "$JOB/pipeline.pid")
  if kill -0 "$PID" 2>/dev/null; then echo "PROCESS=RUNNING PID=$PID"; else echo "PROCESS=STOPPED PID=$PID"; fi
fi
for weight in "$JOB"/runs/*/weights/best.pt; do
  [[ -f "$weight" ]] && ls -lh "$weight"
done
echo "--- latest log ---"
tail -n 20 "$JOB/training.log" 2>/dev/null || true
