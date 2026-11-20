#!/usr/bin/env bash
# The one script (v0.1.38): opens the launcher page, http://127.0.0.1:8094/ - pick a preset (simulation, lab, marina, custom; the file
# profiles/presets.yaml), change any field (sim or real, the judge: ours / rc-test / the course, the network: ts / lan / typed by hand,
# the FCU fake or real, the CV real / fake / off, the boat's profile, the heartbeat / status / map rates, logging), read the plan - every
# command it will run, per host - then LAUNCH. It starts today's launchers in order (run_sim.sh here; or run_ocs.sh here, run_usv.sh on
# the NX and run_agx.sh on the AGX over ssh) and streams their output; STOP ALL (type stop) = scripts/stop_all.sh.
#   scripts/launch.sh               the page opens in the browser (LAUNCH_PORT=8094); Ctrl-C closes the page - the stacks keep running
#   scripts/launch.sh --dry-run     LAUNCH / STOP ALL only show what they would run
#   scripts/launch.sh --no-open     do not open the browser
# The companion never arms: arm / disarm stay the operator's typed commands in usv_ui. A real Pixhawk on the bench = props off.
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "$0")/.." && pwd)"; cd "$KIT"
PY="$KIT/.venv/bin/python"; [ -x "$PY" ] || PY=python3
OPEN=(--open); ARGS=()
for a in "$@"; do case "$a" in --no-open) OPEN=() ;; --dry-run) ARGS+=(--dry-run) ;; *) echo "unknown: $a (--dry-run, --no-open)" >&2; exit 2 ;; esac; done
exec "$PY" -m optiprimex.tools.launcher ${OPEN[@]+"${OPEN[@]}"} ${ARGS[@]+"${ARGS[@]}"}      # bash 3.2 (macOS) + set -u: empty arrays need the + form
