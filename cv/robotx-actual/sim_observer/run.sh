#!/usr/bin/env bash
# OUR buoy observer for the simulator (v0.1.36, ROADMAP WP10e): the boat's measuring chain on simview's rendered frames ->
# logs/…/sim_observer/<stamp>/latest_status.json (cv_publish follows it). scripts/run_sim.sh starts it; by hand:
#   cv/sim_observer/run.sh [--detector synthetic | model --model DIR] [--depth-noise zed|none] [--sim http://127.0.0.1:8097] ...
#   cv/sim_observer/run.sh --help        the observer's options
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//' ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIT="$(cd "$HERE/../.." && pwd)"
PY="${PY:-$KIT/.venv/bin/python}"
[ -x "$PY" ] || PY=python3
cd "$KIT" && exec env PYTHONPATH="$KIT" "$PY" "$HERE/sim_observer.py" "$@"
