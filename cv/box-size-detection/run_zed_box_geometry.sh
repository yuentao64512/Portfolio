#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/robotx/box_size_training"
PYTHON="/home/robotx/buoy_yolo26/.venv/bin/python"

if [[ ! -x "$PYTHON" ]]; then
    echo "Python environment not found: $PYTHON" >&2
    exit 1
fi

cd "$ROOT"
exec "$PYTHON" "$ROOT/zed_box_geometry.py" "$@"
