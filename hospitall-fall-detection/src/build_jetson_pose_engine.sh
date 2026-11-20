#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="${PYTHON:-$ROOT/.venv/bin/python}"
WEIGHTS="${1:-$ROOT/yolo11n-pose.pt}"
IMGSZ="${2:-640}"
WORKSPACE="${3:-2}"

if [[ ! -x "$PYTHON" ]]; then
    echo "Python environment not found: $PYTHON" >&2
    exit 1
fi
if [[ ! -f "$WEIGHTS" ]]; then
    echo "Pose weights not found: $WEIGHTS" >&2
    exit 1
fi

"$PYTHON" - "$WEIGHTS" "$IMGSZ" "$WORKSPACE" <<'PY'
import sys
from ultralytics import YOLO

weights, imgsz, workspace = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
output = YOLO(weights).export(
    format="engine",
    imgsz=imgsz,
    half=True,
    device=0,
    dynamic=False,
    workspace=workspace,
)
print(f"Jetson TensorRT FP16 engine: {output}")
PY

echo "Smoke test command:"
echo "$PYTHON -c 'from ultralytics import YOLO; YOLO(\"${WEIGHTS%.*}.engine\").predict(source=0, imgsz=$IMGSZ, device=0)'"
