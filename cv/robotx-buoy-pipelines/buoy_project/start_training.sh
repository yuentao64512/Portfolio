#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual
PROJECT="$ROOT/buoy_project"
PYTHON="$ROOT/.venv/bin/python"
LOG="$PROJECT/logs/train_both.log"
PIDFILE="$PROJECT/train_both.pid"

mkdir -p "$PROJECT/logs"
if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  echo "Training is already running as PID $(cat "$PIDFILE")" >&2
  exit 1
fi

nohup "$PYTHON" "$PROJECT/train.py" --model both >"$LOG" 2>&1 </dev/null &
pid=$!
echo "$pid" > "$PIDFILE"
echo "pid=$pid"
echo "log=$LOG"
