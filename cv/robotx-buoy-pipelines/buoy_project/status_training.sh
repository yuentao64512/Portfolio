#!/usr/bin/env bash
set -euo pipefail

PROJECT=/home/robotx/buoy_actual/buoy_project
PIDFILE="$PROJECT/train_both.pid"
LOG="$PROJECT/logs/train_both.log"
pid="$(cat "$PIDFILE" 2>/dev/null || true)"
if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
  echo "status=RUNNING pid=$pid"
  ps -p "$pid" -o pid,etime,%cpu,%mem,cmd
else
  echo "status=FINISHED_OR_STOPPED pid=${pid:-unknown}"
fi
find "$PROJECT/runs" -path '*/weights/best.pt' -printf 'best=%p\n' 2>/dev/null || true
printf 'frames_to_label='
find "$PROJECT/frames" -maxdepth 1 -type f -name '*.jpg' -printf '.' 2>/dev/null | wc -c

if [[ -s "$PROJECT/runs/digits_v1_legacy_provisional/results.csv" ]]; then
  echo "stage=digit"
  tail -n 1 "$PROJECT/runs/digits_v1_legacy_provisional/results.csv"
elif [[ -s "$PROJECT/runs/buoys_v1/results.csv" ]]; then
  echo "stage=buoy"
  tail -n 1 "$PROJECT/runs/buoys_v1/results.csv"
fi

echo "--- milestones ---"
tr '\r' '\n' < "$LOG" 2>/dev/null \
  | sed -E 's/\x1B\[[0-9;]*[mK]//g' \
  | grep -aE 'Starting training|EarlyStopping|epochs completed|Results saved|^ +all +' \
  | tail -n 12 || true
