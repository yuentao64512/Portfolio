#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
YOLO="/home/optiprime-agxorin/actual_buoy/.venv/bin/yolo"

if [[ ! -x "$YOLO" ]]; then
  echo "AGX Ultralytics runtime not found: $YOLO" >&2
  exit 1
fi

echo "[$(date -Is)] exporting Model A at 1280"
"$YOLO" export \
  model="$ROOT/models/model_a_best.pt" \
  format=engine imgsz=1280 half=True batch=1 device=0 workspace=4

echo "[$(date -Is)] exporting Model B at 640"
"$YOLO" export \
  model="$ROOT/models/model_b_best.pt" \
  format=engine imgsz=640 half=True batch=1 device=0 workspace=4

echo "[$(date -Is)] exports complete"
sha256sum "$ROOT/models/model_a_best.engine" "$ROOT/models/model_b_best.engine"
touch "$ROOT/TENSORRT_EXPORT_COMPLETE"
