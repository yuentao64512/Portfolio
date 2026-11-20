#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/robotx/buoy_actual"
VENV="$ROOT/.venv"
MARKER="$ROOT/boat_yolo/SETUP_AND_EXTRACTION_COMPLETE"
OUTPUT="$ROOT/boat_yolo/AUTO_REVIEW_REQUIRED"

echo "[$(date -Is)] waiting for setup and frame extraction"
while [[ ! -f "$MARKER" ]]; do
  sleep 20
done

echo "[$(date -Is)] installing YOLOE text encoder dependency"
"$VENV/bin/python" -m pip install -U \
  "https://github.com/ultralytics/CLIP/archive/7ffa84b3bfa40c42ecc1c77147a855e69cb2dd40.zip"

echo "[$(date -Is)] starting candidate identification and cropping"
"$VENV/bin/python" "$ROOT/pipeline/autocrop_candidates.py" \
  --images "$ROOT/boat_yolo/frames/images" \
  --output "$OUTPUT" \
  --conf 0.08

touch "$ROOT/boat_yolo/AUTO_CROPPING_COMPLETE"
echo "[$(date -Is)] candidate identification and cropping complete"
