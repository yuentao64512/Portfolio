#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="/home/robotx/yolo11env/bin/python"

"${PYTHON}" "${ROOT_DIR}/collect_images.py" "$@"
"${PYTHON}" "${ROOT_DIR}/auto_annotate.py"
"${PYTHON}" "${ROOT_DIR}/collect_images.py" \
  --queries "${ROOT_DIR}/negative_queries.txt" \
  --output "${ROOT_DIR}/raw_negative" \
  --per-query 12 --max-images 140
"${PYTHON}" "${ROOT_DIR}/add_negatives.py"
"${PYTHON}" "${ROOT_DIR}/train_yolo11_seg.py"
