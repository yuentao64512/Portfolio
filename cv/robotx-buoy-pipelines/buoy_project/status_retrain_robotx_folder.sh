#!/usr/bin/env bash
set -euo pipefail

PROJECT=/home/robotx/buoy_actual/buoy_project
PIDFILE="$PROJECT/retrain_robotx_folder.pid"
LOG="$PROJECT/logs/retrain_robotx_folder.log"
JOB="$PROJECT/retrain_robotx_do_not_touch_20261110"
pid="$(cat "$PIDFILE" 2>/dev/null || true)"

if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
  echo "status=RUNNING pid=$pid"
  ps -p "$pid" -o pid,etime,%cpu,%mem,cmd
elif [[ -f "$JOB/RETRAIN_COMPLETE" ]]; then
  echo "status=COMPLETE pid=${pid:-unknown}"
else
  echo "status=FINISHED_OR_FAILED pid=${pid:-unknown}"
fi

if [[ -f "$JOB/pseudo_full/pseudolabel_report.json" ]]; then
  echo "stage=pseudo_labels_ready"
fi
if [[ -f "$JOB/merged/dataset_summary.json" ]]; then
  echo "stage=training_or_later"
fi
find "$PROJECT/runs" -maxdepth 3 -path '*robotx_do_not_touch_v2*/weights/best.pt' -printf 'best=%p\n' 2>/dev/null || true
echo "--- milestones ---"
tr '\r' '\n' < "$LOG" 2>/dev/null \
  | sed -E 's/\x1B\[[0-9;]*[mK]//g' \
  | grep -aE '^\{|^  "counts"|^    "(images|buoys|digit_[0-9]|uncertain_diamonds)"|Starting training|EarlyStopping|epochs completed|Results saved|SOURCE_DATA_UNCHANGED|models/.*OK' \
  | tail -n 30 || true
