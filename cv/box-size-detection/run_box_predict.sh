#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/robotx/box_size_training"
PYTHON="/home/robotx/buoy_yolo26/.venv/bin/python"
WEIGHTS="$ROOT/runs/yolo11n_dataset_v2/weights/best.pt"

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 /path/to/video.mp4 [output_name]" >&2
    exit 2
fi

VIDEO="$1"
if [[ ! -f "$VIDEO" ]]; then
    echo "Video not found: $VIDEO" >&2
    exit 1
fi

if [[ ! -x "$PYTHON" ]]; then
    echo "Python environment not found: $PYTHON" >&2
    exit 1
fi

if [[ ! -f "$WEIGHTS" ]]; then
    echo "Model weights not found: $WEIGHTS" >&2
    exit 1
fi

VIDEO_BASENAME="$(basename "$VIDEO")"
DEFAULT_NAME="${VIDEO_BASENAME%.*}"
OUTPUT_NAME="${2:-$DEFAULT_NAME}"
OUTPUT_NAME="${OUTPUT_NAME// /_}"
OUTPUT_DIR="$ROOT/outputs/$OUTPUT_NAME"

mkdir -p "$OUTPUT_DIR"

echo "Input:   $VIDEO"
echo "Model:   $WEIGHTS"
echo "Output:  $OUTPUT_DIR"

"$PYTHON" "$ROOT/predict_video.py" "$VIDEO" \
    --weights "$WEIGHTS" \
    --output "$OUTPUT_DIR/${OUTPUT_NAME}_annotated.mp4" \
    --csv "$OUTPUT_DIR/${OUTPUT_NAME}_predictions.csv" \
    --device 0

echo "Done."
echo "Annotated video: $OUTPUT_DIR/${OUTPUT_NAME}_annotated.mp4"
echo "Prediction CSV:  $OUTPUT_DIR/${OUTPUT_NAME}_predictions.csv"
