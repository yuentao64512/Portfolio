#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/robotx/box_size_training"
PYTHON="/home/robotx/buoy_yolo26/.venv/bin/python"
SDK_HELPER="/usr/local/zed/get_python_api.py"
WHEEL_DIR="$ROOT/zed_wheels"

if [[ ! -x "$PYTHON" ]]; then
    echo "Python environment not found: $PYTHON" >&2
    exit 1
fi
if [[ ! -f "$SDK_HELPER" ]]; then
    echo "ZED SDK helper not found: $SDK_HELPER" >&2
    exit 1
fi

mkdir -p "$WHEEL_DIR"
"$PYTHON" "$SDK_HELPER" --path "$WHEEL_DIR"
"$PYTHON" -c 'import pyzed.sl as sl; print("PyZED import OK")'
"$PYTHON" -c 'import cv2, numpy; print("Geometry environment OK")'
