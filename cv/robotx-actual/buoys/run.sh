#!/usr/bin/env bash
# cv/buoys: the buoy observer (students' actual_buoy_observer.py, verbatim snapshot) from OUR tree:
#   weights cv/buoys/models/best.pt (cv/sync_models.sh), runs under cv/buoys/runs/<stamp>/ (latest_status.json each frame)
#   usage: buoys/run.sh [--headless] [--frames N] [--self-test] [--mavlink ENDPOINT] [observer args...]
#          buoys/run.sh --sender [sender args]     the students' buoy_camera_sender.py (multi-buoy HMAC UDP link) instead
#          buoys/run.sh --verify                   verify_agx_setup.py (CUDA / ZED / TensorRT import check)
#          buoys/run.sh -h | --help               this text (the observer's own options: run it with --help after the flags)
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/../_env.sh"
PY="$(cv_python)"
mkdir -p "$HERE/runs"
case "${1:-}" in
  --sender) shift; exec "$PY" "$HERE/buoy_camera_sender.py" --weights "$HERE/models/best.pt" --key-file "${AGX_KEY_FILE:-/home/optiprime-agxorin/actual_buoy/buoy_link.key}" "$@" ;;
  --verify) exec "$PY" "$HERE/verify_agx_setup.py" ;;
esac
[ -f "$HERE/models/best.pt" ] || { echo "no $HERE/models/best.pt - run cv/sync_models.sh first" >&2; exit 1; }
echo "[cv/buoys] $PY  weights models/best.pt  output runs/" >&2
exec "$PY" "$HERE/actual_buoy_observer.py" --weights "$HERE/models/best.pt" --output "$HERE/runs" "$@"
