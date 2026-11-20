#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PYTHON_BIN=${PYTHON_BIN:-}

if [[ -z "$PYTHON_BIN" ]]; then
  for candidate in \
    "$HOME/buoy_robotx/.venv/bin/python" \
    "$HOME/buoy_robotx/runtime/.venv/bin/python" \
    "$HOME/buoy_actual/.venv/bin/python" \
    "$(command -v python3 2>/dev/null || true)"; do
    if [[ -n "$candidate" && -x "$candidate" ]]; then
      PYTHON_BIN=$candidate
      break
    fi
  done
fi

if [[ -z "$PYTHON_BIN" ]]; then
  echo "ERROR: Python runtime not found."
  exit 1
fi

for required in \
  buoy_detector_yolo11m_1280_best.engine \
  marker_classifier_yolo11m_320_best.engine \
  number_detector_yolo11n_320_best.engine; do
  [[ -f "$ROOT/$required" ]] || { echo "Missing: $ROOT/$required"; exit 1; }
done

"$PYTHON_BIN" "$ROOT/predict_combined_photo.py" \
  --source "$ROOT/photo_2026-11-12_21-48-05.jpg" \
  --buoy-detector "$ROOT/buoy_detector_yolo11m_1280_best.engine" \
  --number-detector "$ROOT/number_detector_yolo11n_320_best.engine" \
  --marker-classifier "$ROOT/marker_classifier_yolo11m_320_best.engine" \
  --output "$ROOT/agx_engine_test_prediction.jpg" \
  --csv "$ROOT/agx_engine_test_prediction.csv"

touch "$ROOT/AGX_ENGINE_TEST_COMPLETE"
echo "Test output: $ROOT/agx_engine_test_prediction.jpg"
