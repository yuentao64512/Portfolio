#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 INPUT_FOLDER [OUTPUT_FOLDER]"
  exit 1
fi

INPUT_DIR=$(readlink -f "$1")
OUTPUT_DIR=${2:-"$INPUT_DIR/predictions"}
mkdir -p "$OUTPUT_DIR"

if [[ ! -d "$INPUT_DIR" ]]; then
  echo "Input folder does not exist: $INPUT_DIR"
  exit 1
fi

PYTHON=/home/robotx/buoy_actual/.venv/bin/python
PREDICTOR=/home/robotx/Robotx_train/predict_combined_photo.py

shopt -s nullglob nocaseglob
images=(
  "$INPUT_DIR"/*.jpg
  "$INPUT_DIR"/*.jpeg
  "$INPUT_DIR"/*.png
  "$INPUT_DIR"/*.bmp
  "$INPUT_DIR"/*.webp
)

if (( ${#images[@]} == 0 )); then
  echo "No supported images found in: $INPUT_DIR"
  exit 1
fi

echo "Found ${#images[@]} images"
completed=0
failed=0

for image in "${images[@]}"; do
  filename=$(basename "$image")
  stem=${filename%.*}
  echo "[$((completed + failed + 1))/${#images[@]}] Predicting: $filename"
  if "$PYTHON" "$PREDICTOR" \
      --source "$image" \
      --output "$OUTPUT_DIR/${stem}_prediction.jpg" \
      --csv "$OUTPUT_DIR/${stem}_prediction.csv"; then
    completed=$((completed + 1))
  else
    echo "FAILED: $image" >&2
    failed=$((failed + 1))
  fi
done

echo "COMPLETE: succeeded=$completed failed=$failed"
echo "Results: $OUTPUT_DIR"
