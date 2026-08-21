#!/usr/bin/env bash
set -euo pipefail

cd /home/jetson/fall_action_project
source .venv/bin/activate

export PYTHONNOUSERSITE=1

python src/run_live_camera.py \
  --camera "${1:-0}" \
  --pose-model yolo11n-pose.engine \
  --classifier models/current_pose_rf.pkl \
  --max-people 8 \
  --sample-fps 5 \
  --seq-len 16 \
  --lying-alert-sec 10
