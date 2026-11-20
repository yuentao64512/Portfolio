#!/usr/bin/env bash
set -Eeuo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
"$ROOT/.venv/bin/python" -m py_compile "$ROOT/run_detection.py" "$ROOT/action_core.py"
for stage in 01_stock_single 02_custom_single 03_custom_four_people 04_custom_eight_full_pose; do
  "$ROOT/iterations/$stage/run.sh" --help >/dev/null
done
for model in actions.pkl yolo11n-pose.pt yolo11n_pose_lite.pt yolo11n_pose_lite.engine; do
  test -e "$ROOT/models/$model" || { echo "Missing model: $model" >&2; exit 1; }
done
echo "PASS: all four normal-camera stages and model links are ready."

