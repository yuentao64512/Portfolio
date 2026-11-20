#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/robotx/box_size_training"
PYTHON="/home/robotx/buoy_yolo26/.venv/bin/python"
cd "$ROOT"
exec "$PYTHON" "$ROOT/zed_box_cuboid_scene.py" \
  --serial 37406048 \
  --camera-height-m 0.49 \
  --overhead \
  "$@"
