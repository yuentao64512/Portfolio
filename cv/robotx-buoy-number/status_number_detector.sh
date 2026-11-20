#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/Robotx_train/number_detector_training_v1
ENV_FILE="$ROOT/current_run.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "No number-detector run has been launched."
  exit 1
fi

# shellcheck disable=SC1090
source "$ENV_FILE"
echo "Run: $RUN"
echo "PID: $PID"
if kill -0 "$PID" 2>/dev/null; then
  echo "State: RUNNING"
else
  echo "State: STOPPED/FINISHED"
fi
if [[ -f "$RUN_DIR/weights/best.pt" ]]; then
  ls -lh "$RUN_DIR/weights/best.pt"
else
  echo "best.pt: not created yet"
fi
echo "--- latest log ---"
tail -n 25 "$LOG"
