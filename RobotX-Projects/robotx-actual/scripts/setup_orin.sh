#!/usr/bin/env bash
# One-time setup ON THE ORIN NX for MASTER_SCRIPTS (optiprimex/ + usv_ui/ + scripts/): a venv with the Python deps, the serial
# group, and the two apt packages the stack needs. Python >= 3.8 is enough for running (pip install -e needs 3.10;
# the launchers use PYTHONPATH instead). Re-runnable.
# usage: bash scripts/setup_orin.sh          -h | --help prints this
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BENCH="$KIT"                                                # the package + .venv live at the root since v0.1.19
PYTHON="${PYTHON:-python3}"
echo "python: $($PYTHON --version 2>&1)  ($(command -v $PYTHON))"
if [ ! -x "$KIT/.venv/bin/python" ]; then
  $PYTHON -m venv "$KIT/.venv" || { echo "python3-venv missing: sudo apt install python3-venv"; exit 1; }
fi
"$KIT/.venv/bin/pip" install --upgrade pip wheel >/dev/null
"$KIT/.venv/bin/pip" install pymavlink pyserial paho-mqtt pyyaml jsonschema pytest pytest-timeout
if "$KIT/.venv/bin/python" -c "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)"; then
  "$KIT/.venv/bin/pip" install -e "$KIT"        # console scripts op-*, ocs_cli
else
  echo "python < 3.10: optiprimex is used through PYTHONPATH=$KIT (the launchers set it); no console scripts"
fi
PYTHONPATH="$BENCH" "$KIT/.venv/bin/python" -c "import optiprimex, optiprimex.usv.mission_manager; print('optiprimex', optiprimex.__version__, 'imports ok')"
for pkg in mosquitto tmux; do
  command -v $pkg >/dev/null || echo "MISSING: sudo apt install $pkg   ($pkg not on PATH)"
done
if command -v mosquitto >/dev/null && systemctl is-enabled mosquitto >/dev/null 2>&1; then
  echo "NOTE: the mosquitto service is enabled - the launcher runs its own broker on 1884: sudo systemctl disable --now mosquitto"
fi
if ! id -nG | grep -qw dialout; then
  echo "NOTE: $(id -un) is not in 'dialout' (the UART): sudo usermod -aG dialout $(id -un) ; then log out and in"
fi
if [ -e /dev/ttyTHS1 ]; then
  echo "serial: /dev/ttyTHS1 present $(stat -c '%U:%G %a' /dev/ttyTHS1)"
else
  echo "serial: /dev/ttyTHS1 not present (USB jumper: /dev/ttyACM0 ; Ethernet: udpin:0.0.0.0:14550)"
fi
echo "ok - next: scripts/run_usv.sh --fcu fake (GUIDE stage A) or scripts/run_usv.sh   (the tool alone: $KIT/.venv/bin/python $KIT/usv_ui/usv_ui.py --help)"
