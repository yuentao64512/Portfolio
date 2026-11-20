#!/usr/bin/env bash
# RESET EVERYTHING (v0.1.37, ROADMAP WP10d) - ON THE MAC: a clean start without stopping anything:
#   a simulator attached (SIM_PORT answers) -> RESET SIM (scripts/sim_reset.sh: the boat to the start, faults off, HOLD, the mockups,
#   abort / confirm_safe / reset on both ends, clear_map, the fake judge's counters); else `ocs_cli reset` (the OCS -> R2, the boat
#   forgets the run) + the fake judge's counters (rc/sim/inject sim_reset on the hub). The stacks stay up. The page asks for `reset`.
#   scripts/reset_all.sh [--dry-run]
set -uo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; cd "$KIT"
DRY=0; [ "${1:-}" = "--dry-run" ] && DRY=1
PY="$KIT/.venv/bin/python"; HUB="${HUB_PORT:-1882}"
if curl -s --max-time 2 "http://127.0.0.1:${SIM_PORT:-8097}/status" >/dev/null 2>&1; then
  echo "[reset_all] a simulator answers: RESET SIM"
  [ $DRY -eq 1 ] || scripts/sim_reset.sh
else
  echo "[reset_all] no simulator: ocs_cli abort, reset; the fake judge's counters"
  if [ $DRY -eq 0 ]; then
    "$PY" -m optiprimex.tools.ocs_cli --port "$HUB" abort 2>&1 | tail -1
    "$PY" -m optiprimex.tools.ocs_cli --port "$HUB" reset 2>&1 | tail -1
    mosquitto_pub -p "$HUB" -t rc/sim/inject -m '{"kind": "sim_reset"}' 2>/dev/null || true
  fi
fi
echo "[reset_all] done$( [ $DRY -eq 1 ] && echo ' (dry run)')"
