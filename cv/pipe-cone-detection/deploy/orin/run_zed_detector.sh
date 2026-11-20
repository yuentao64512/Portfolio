#!/usr/bin/env bash
set -euo pipefail

PROJECT=/home/jetson/pipe_cones
PYTHON=/home/jetson/fall_action_project/.venv/bin/python
ENGINE="$PROJECT/models/pipe_cone_yolo26l_fp16.engine"
PT="$PROJECT/models/pipe_cone_yolo26l_best.pt"

if [[ -f "$ENGINE" ]]; then
    WEIGHTS="$ENGINE"
else
    WEIGHTS="$PT"
fi

exec "$PYTHON" "$PROJECT/zed_pipe_cone_detector.py" --weights "$WEIGHTS" "$@"
