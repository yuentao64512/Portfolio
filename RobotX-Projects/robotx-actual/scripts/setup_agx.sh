#!/usr/bin/env bash
# One-time setup ON THE AGX ORIN for the perception leg of the kit (~/Desktop/dm3/MASTER_SCRIPTS). Only cv_publish.py
# runs here; it needs paho-mqtt and nothing of the student's environments (ZED SDK, Torch, Ultralytics stay theirs).
# Makes a small venv at .venv (same place as on the NX, so the launchers find it). Re-runnable.
# usage: bash scripts/setup_agx.sh           -h | --help prints this
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BENCH="$KIT"                                                # the package + .venv live at the root since v0.1.19
PYTHON="${PYTHON:-python3}"
echo "python: $($PYTHON --version 2>&1)  ($(command -v $PYTHON))   kit: $KIT"
if [ ! -x "$KIT/.venv/bin/python" ]; then
  $PYTHON -m venv "$KIT/.venv" || { echo "python3-venv missing: sudo apt install python3-venv"; exit 1; }
fi
"$KIT/.venv/bin/pip" install --upgrade pip wheel >/dev/null
"$KIT/.venv/bin/pip" install paho-mqtt pyyaml jsonschema      # pyyaml + jsonschema: the sim CV (run_agx.sh --sim) imports optiprimex
"$KIT/.venv/bin/python" -c "import paho.mqtt, yaml, jsonschema; print('paho-mqtt + pyyaml + jsonschema ok')"
# the student programs this script reads from (paths from ../network.env; the observer and the dock mapper write them)
NETENV="$KIT/network.env"
OBS="$(sed -n 's/^AGX_OBSERVER_LOGS=\([^#]*\).*/\1/p' "$NETENV" 2>/dev/null | tr -d '[:space:]')"
DOCK="$(sed -n 's/^AGX_DOCK_LATEST=\([^#]*\).*/\1/p' "$NETENV" 2>/dev/null | tr -d '[:space:]')"
BOAT="$(sed -n 's/^BOAT_HOST=\([^#]*\).*/\1/p' "$NETENV" 2>/dev/null | tr -d '[:space:]')"
mkdir -p "$KIT/cv/buoys/runs" "$KIT/cv/docking/runs"
[ -d "${OBS:-/nonexistent}" ] && echo "buoy observer runs: $OBS (ok)" || echo "NOTE: buoy observer runs dir not found: ${OBS:-unset} - network.env AGX_OBSERVER_LOGS should be <kit>/cv/buoys/runs"
[ -d "$(dirname "${DOCK:-/nonexistent}")" ] && echo "dock observer runs: $DOCK (latest/ appears after docking/run.sh ran once)" || echo "NOTE: dock observer runs dir not found: ${DOCK:-unset} - network.env AGX_DOCK_LATEST should be <kit>/cv/docking/runs/latest"
BUOY_VENV="$(sed -n 's/^AGX_BUOY_VENV=\([^#]*\).*/\1/p' "$NETENV" 2>/dev/null | tr -d '[:space:]')"
[ -x "${BUOY_VENV:-/nonexistent}/bin/python" ] && echo "students' venv for cv/*/run.sh: $BUOY_VENV (ok, used read-only)" || echo "NOTE: AGX_BUOY_VENV ${BUOY_VENV:-unset} has no bin/python - cv/buoys and cv/docking cannot run until network.env points at a venv with pyzed/torch/ultralytics"
[ -f "$KIT/cv/buoys/models/best.pt" ] && echo "models synced" || echo "NEXT: cd cv && ./sync_models.sh   (models are not in the archive)"
if command -v nc >/dev/null && [ -n "$BOAT" ]; then
  nc -z -w 2 "$BOAT" 1884 2>/dev/null && echo "boat broker $BOAT:1884 reachable" || echo "NOTE: boat broker $BOAT:1884 not reachable now (run_usv.sh on the NX first, or wrong BOAT_HOST)"
fi
echo "clock: $(date -u '+%Y-%m-%d %H:%M:%S') UTC - must match the NX within 1 s (chrony, or: sudo date -u -s '...')"
echo "ok - next: scripts/run_agx.sh --demo (link test); cv/buoys/run.sh --verify; then cv/buoys/run.sh --headless + scripts/run_agx.sh"
