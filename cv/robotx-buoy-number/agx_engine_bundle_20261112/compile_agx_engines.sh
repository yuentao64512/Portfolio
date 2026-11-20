#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
LOG="$ROOT/compile_agx_engines.log"
exec > >(tee -a "$LOG") 2>&1

find_yolo() {
  local candidates=(
    "$HOME/buoy_robotx/.venv/bin/yolo"
    "$HOME/buoy_robotx/runtime/.venv/bin/yolo"
    "$HOME/buoy_actual/.venv/bin/yolo"
    "$HOME/.local/bin/yolo"
  )
  for candidate in "${candidates[@]}"; do
    [[ -x "$candidate" ]] && { echo "$candidate"; return 0; }
  done
  command -v yolo 2>/dev/null || return 1
}

YOLO_BIN=${YOLO_BIN:-$(find_yolo)}
if [[ -z "${YOLO_BIN:-}" || ! -x "$YOLO_BIN" ]]; then
  echo "ERROR: Ultralytics yolo executable not found."
  echo "Set it explicitly: YOLO_BIN=/path/to/yolo $0"
  exit 1
fi

echo "Started: $(date -Iseconds)"
echo "Host: $(hostname)"
echo "Architecture: $(uname -m)"
echo "YOLO: $YOLO_BIN"
nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader || true

cd "$ROOT"
sha256sum -c SHA256SUMS

export_one() {
  local model=$1
  local size=$2
  echo "============================================================"
  echo "Exporting $model at imgsz=$size, FP16, static batch=1"
  "$YOLO_BIN" export \
    model="$ROOT/$model" \
    format=engine \
    imgsz="$size" \
    batch=1 \
    half=True \
    dynamic=False \
    device=0 \
    workspace=4
}

export_one buoy_detector_yolo11m_1280_best.pt 1280
export_one marker_classifier_yolo11m_320_best.pt 320
export_one number_detector_yolo11n_320_best.pt 320

echo "============================================================"
echo "Generated engines:"
ls -lh "$ROOT"/*.engine
touch "$ROOT/AGX_ENGINE_EXPORT_COMPLETE"
echo "Completed: $(date -Iseconds)"
