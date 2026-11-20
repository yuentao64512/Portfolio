#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PYTHONNOUSERSITE=1
exec "$ROOT/.venv/bin/python" "$ROOT/run_detection.py" \
  --pose-model "$ROOT/models/yolo11n_pose_lite.engine" --imgsz 384 \
  --max-people 4 --skeleton body --title "Stage 3: custom four-person" "$@"

