#!/usr/bin/env bash
# RESET SIM (v0.1.33, ROADMAP WP10b): one clean start of the simulator without restarting anything - the boat to the world's start
# (pose + heading), the mockups back to the world file, every fault off (kill, RC5, RC6, GPS, FCU pause), the mode switch HOLD, then
# abort, confirm_safe, reset (the OCS -> R2, the boat forgets the run), clear_map and the fake judge's counters.
#   scripts/sim_reset.sh            (SIM_PORT=8097; the simulator must be up: scripts/run_sim.sh)
# The same as the SIM page's RESET SIM… and the OCS page's ⚙ SETTINGS -> Reset the simulator (both ask you to type reset).
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
PORT="${SIM_PORT:-8097}"
curl -sS --max-time 60 -X POST -H 'Content-Type: application/json' -H 'X-Sim-Web: 1' -d '{"confirm":"reset"}' "http://127.0.0.1:$PORT/reset" \
  || { echo "the simulator does not answer on $PORT (scripts/run_sim.sh)" >&2; exit 1; }
echo
