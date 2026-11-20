#!/usr/bin/env bash
# The UAV window (v0.1.65, D16 of docs/OVERNIGHT_2026-11-07_UAV.md): http://<this Mac>:8099/ - a monitoring page for the drone, read-only
# but for the whitelisted OCS lines (pilot ready, declare uav, uav task, resend course); the window_control lock (one commander, a PIN for
# other computers). On the Mac only (tmux session `uavwin`), reading the hub. Started by the launcher (UAV: on), run_uav.sh (--uav);
# stopped by stop_ocs.sh / stop_bench.sh.
#   scripts/run_uav_window.sh [start] [--ts | --lan] [--bind ADDR] [--port 8099] [--pin NNNN]
#   scripts/run_uav_window.sh stop | status
# Bound to OCS_HOST (the team-LAN / tailnet address of this Mac) when set, else 127.0.0.1 - never 0.0.0.0 (the RoboCommand NIC).
set -uo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${UAV_WINDOW_SESSION:-uavwin}"
OP=start; [ $# -gt 0 ] && case "$1" in start|stop|status) OP="$1"; shift ;; esac
for _a in "$@"; do case "$_a" in --ts) NET_PROFILE=ts ;; --lan) NET_PROFILE=lan ;; esac; done
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT" 2>/dev/null || true
BIND=""; PORT="${UAV_WEB_PORT:-8099}"; PIN="${UAV_WINDOW_PIN:-}"
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;;
    --bind) BIND="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --pin) PIN="$2"; shift 2 ;;
    --ts|--lan) shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
case "$OP" in
  stop) if tmux has-session -t "$SESSION" 2>/dev/null; then tmux send-keys -t "$SESSION:web" C-c 2>/dev/null; sleep 0.5; tmux kill-session -t "$SESSION"; echo "UAV window stopped"; fi; exit 0 ;;
  status) tmux has-session -t "$SESSION" 2>/dev/null && echo "UAV window running (tmux $SESSION)" || echo "no UAV window"; exit 0 ;;
esac
tmux has-session -t "$SESSION" 2>/dev/null && { echo "a UAV window runs already here (tmux $SESSION): scripts/run_uav_window.sh stop first" >&2; exit 1; }
BIND="${BIND:-${OCS_HOST:-127.0.0.1}}"
PY="$KIT/.venv/bin/python"
RUN_DIR="$KIT/logs/ocs/uavwin_$(date +%Y%m%d_%H%M%S)"; mkdir -p "$RUN_DIR"
printf '%s\n' "cd '$KIT' && UAV_WINDOW_PIN='$PIN' '$PY' -m optiprimex.tools.uav_web --hub 127.0.0.1:${HUB_PORT:-1882} --bind '$BIND' --port $PORT 2>&1 | tee -i '$RUN_DIR/uav_web.log'; echo '[exited]'" > "$RUN_DIR/uav_web.sh"
tmux new-session -d -s "$SESSION" -n web -c "$RUN_DIR"
tmux send-keys -t "$SESSION:web" ". ./uav_web.sh" C-m   # v0.1.78.11: sourced, not `bash x.sh` - the window's own shell runs the program, so the launcher sees it (not a shell = died)
sleep 1.0
echo "UAV window: http://$BIND:$PORT/ (tmux $SESSION; log $RUN_DIR/uav_web.log - the PIN is printed there when bound beyond 127.0.0.1)"
