#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="${PYTHON:-/home/robotx/fall_action_project/.venv/bin/python}"
POSE_MODEL="${POSE_MODEL:-/home/robotx/fall_action_project/yolo11n-pose.pt}"
exec "$PYTHON" "$ROOT/run_detection.py" \
  --pose-model "$POSE_MODEL" \
  --classifier "$ROOT/models/actions.pkl" \
  --imgsz 640 --max-people 1 --skeleton full \
  --title "RSE4207 trained five-class action detector" "$@"
