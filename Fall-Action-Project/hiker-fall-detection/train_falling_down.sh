#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-/home/robotx/fall_action_project/.venv/bin/python}"
MODEL_OUT="${MODEL_OUT:-$ROOT/models/actions.pkl}"
REPORT_OUT="${REPORT_OUT:-$ROOT/reports/training_report.json}"
AUGMENTATION="${AUGMENTATION:-$ROOT/data/falling_down_photo.npz}"

if [[ ! -x "$PYTHON" ]]; then
  echo "Python environment not found: $PYTHON" >&2
  echo "Set PYTHON=/path/to/python and run again." >&2
  exit 2
fi
if [[ ! -f "$AUGMENTATION" ]]; then
  echo "Falling-down augmentation not found: $AUGMENTATION" >&2
  exit 2
fi

echo "Training the five-class action model with falling_down-only augmentation"
echo "Other new action photos are not included."
exec "$PYTHON" "$ROOT/train_rse_actions.py" \
  --falling-aug-npz "$AUGMENTATION" \
  --model-out "$MODEL_OUT" \
  --report-out "$REPORT_OUT" \
  "$@"
