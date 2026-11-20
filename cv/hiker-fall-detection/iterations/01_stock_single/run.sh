#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONNOUSERSITE=1
exec "$ROOT/.venv/bin/python" "$ROOT/run_detection.py" \
  --pose-model "$ROOT/models/yolo11n-pose.pt" --imgsz 640 \
  --max-people 1 --skeleton full --title "Stage 1: stock single-person" "$@"

