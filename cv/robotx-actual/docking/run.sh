#!/usr/bin/env bash
# cv/docking: the red-dock observer (perception only: detector + ZED range + geolocation -> runs/latest/latest_status.json)
#   usage: docking/run.sh [--headless] [--frames N] [--self-test] [--mavlink ENDPOINT] [--weights ...] [observer args...]
#          docking/run.sh --smoke [--source 0|image|video]   run_docking.py: the TensorRT detector alone, no ZED/FCU
#   models: cv/docking/models/dock_partial_best.{engine,onnx,pt} (cv/sync_models.sh; the .engine is built for this AGX)
#   The cannon link is separate: cv/cannon/run.sh reads the status file this writes.
#          docking/run.sh -h | --help             this text (the observer's own options: run it with --help after the flags)
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/../_env.sh"
PY="$(cv_python)"
mkdir -p "$HERE/runs"
if [ "${1:-}" = "--smoke" ]; then shift; exec "$PY" "$HERE/run_docking.py" --model "$HERE/models/dock_partial_best.engine" "$@"; fi
case " $* " in *" --self-test "*) ;; *)
  [ -f "$HERE/models/dock_partial_best.engine" ] || { echo "no $HERE/models/dock_partial_best.engine - run cv/sync_models.sh first" >&2; exit 1; } ;;
esac
echo "[cv/docking] $PY  config dock_config.json  output runs/" >&2
exec "$PY" "$HERE/dock_observer.py" --config "$HERE/dock_config.json" --output "$HERE/runs" "$@"
