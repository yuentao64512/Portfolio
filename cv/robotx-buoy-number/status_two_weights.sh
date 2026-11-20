#!/usr/bin/env bash
set -u

ROOT=/home/robotx/Robotx_train/two_weight_training_v1
ENV_FILE="$ROOT/current_runs.env"
PID_FILE="$ROOT/pipeline.pid"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "No two-weight run has been launched."
  exit 1
fi

# shellcheck disable=SC1090
source "$ENV_FILE"
PID="$(cat "$PID_FILE" 2>/dev/null || true)"
if [[ -n "$PID" ]] && kill -0 "$PID" 2>/dev/null; then
  echo "Status: RUNNING (PID $PID)"
else
  echo "Status: STOPPED/FINISHED"
fi

echo "Detector: $ROOT/$DETECT_NAME"
if [[ -f "$ROOT/DETECTOR_COMPLETE_${STAMP}" ]]; then
  echo "Detector stage: COMPLETE"
elif [[ -f "$ROOT/$DETECT_NAME/results.csv" ]]; then
  echo "Detector stage: TRAINING"
else
  echo "Detector stage: STARTING"
fi

echo "Classifier: $ROOT/$CLASS_NAME"
if [[ -f "$ROOT/CLASSIFIER_COMPLETE_${STAMP}" ]]; then
  echo "Classifier stage: COMPLETE"
elif [[ -f "$ROOT/$CLASS_NAME/results.csv" ]]; then
  echo "Classifier stage: TRAINING"
else
  echo "Classifier stage: WAITING"
fi

echo
echo "Latest progress:"
grep -E "^[[:space:]]*[0-9]+/[0-9]+|EarlyStopping|Results saved to|all training complete" "$PIPELINE_LOG" 2>/dev/null | tail -8 || true

echo
echo "Weights found:"
find "$ROOT" -path "*/weights/best.pt" -printf "%TY-%Tm-%Td %TH:%TM:%TS %p\n" 2>/dev/null | sort

echo
echo "GPU:"
nvidia-smi --query-gpu=name,memory.used,memory.total,utilization.gpu --format=csv,noheader 2>/dev/null || true

echo
echo "Log tail:"
tail -25 "$PIPELINE_LOG" 2>/dev/null || true
