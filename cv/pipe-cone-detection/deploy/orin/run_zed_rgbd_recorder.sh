#!/usr/bin/env bash
set -euo pipefail

PROJECT=/home/jetson/pipe_cones
PYTHON=/home/jetson/fall_action_project/.venv/bin/python

exec "$PYTHON" "$PROJECT/record_zed_rgbd.py" "$@"
