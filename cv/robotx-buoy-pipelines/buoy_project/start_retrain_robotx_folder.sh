#!/usr/bin/env bash
set -euo pipefail

PROJECT=/home/robotx/buoy_actual/buoy_project
PIDFILE="$PROJECT/retrain_robotx_folder.pid"
LOG="$PROJECT/logs/retrain_robotx_folder.log"
JOB="$PROJECT/retrain_robotx_do_not_touch_20261110"

mkdir -p "$PROJECT/logs"
if [[ -f "$PIDFILE" ]] && kill -0 "$(cat "$PIDFILE")" 2>/dev/null; then
  echo "Retraining is already running as PID $(cat "$PIDFILE")" >&2
  exit 1
fi
if [[ -e "$JOB" ]]; then
  echo "Refusing to overwrite existing job: $JOB" >&2
  exit 1
fi

nohup "$PROJECT/run_retrain_robotx_folder.sh" > "$LOG" 2>&1 </dev/null &
pid=$!
echo "$pid" > "$PIDFILE"
echo "pid=$pid"
echo "log=$LOG"
echo "job=$JOB"
