#!/usr/bin/env bash
# STOP EVERYTHING (v0.1.37, ROADMAP WP10d) - ON THE MAC: every stack down, in the agreed order, the OCS last:
#   1. the boat (NX): scripts/stop_usv.sh - usv_ui gets `quit` first (mode-20 outputs zeroed; the FCU stays as the transmitter left it:
#      the companion never arms or disarms), then the mission manager and the boat broker
#   2. perception (AGX): scripts/stop_agx.sh - cv_publish / the observers / the camera stream (and the recorder when it exists)
#   3. the judge console (tmux judge): scripts/stop_judge.sh      4. rc-test (if up): scripts/stop_rctest.sh
#   5. the laptop bench / simulator (if up): scripts/stop_bench.sh      6. the OCS stack last: scripts/stop_ocs.sh
# A host that does not answer is reported and skipped. The OCS page refuses it while a task runs (abort first) and asks for `stop`.
#   scripts/stop_all.sh [--dry-run]          ORIN_SSH / AGX_SSH from network*.env (scripts/_netenv.sh)
set -uo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"; cd "$KIT"
[ -f scripts/_netenv.sh ] && . scripts/_netenv.sh >/dev/null 2>&1 || true
DRY=0; [ "${1:-}" = "--dry-run" ] && DRY=1
NX="${ORIN_SSH:-usv-pixhawk-ts}"; AGX="${AGX_SSH:-agxorin}"; REMOTE="cd ~/Desktop/dm3/MASTER_SCRIPTS &&"
step() {   # name, command
  echo "[stop_all] $1: $2"
  [ $DRY -eq 1 ] && return 0
  bash -c "$2" 2>&1 | tail -3 | sed 's/^/    /' || echo "    (did not answer - skipped)"
}
step "1 boat (NX)" "ssh -o ConnectTimeout=5 -o BatchMode=yes $NX '$REMOTE scripts/stop_usv.sh'"
step "2 perception (AGX)" "ssh -o ConnectTimeout=5 -o BatchMode=yes $AGX '$REMOTE scripts/stop_agx.sh'"
step "3 judge console" "scripts/stop_judge.sh"
step "4 rc-test" "scripts/stop_rctest.sh"
step "5 bench / simulator" "tmux has-session -t ${OPTIPRIMEX_SESSION:-optiprimex} 2>/dev/null && scripts/stop_bench.sh || echo 'no bench running'"
step "6 OCS (last)" "scripts/stop_ocs.sh"
echo "[stop_all] done$( [ $DRY -eq 1 ] && echo ' (dry run: nothing was stopped)')"
