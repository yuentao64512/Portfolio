#!/usr/bin/env bash
# The USV window (boat_web, v0.1.63) on the team LAN - on the NX (the event's default) or on the OCS Mac - for a delegate's laptop.
# Dmitrii 2026-11-07: one commander (take over at once); v0.1.78.9 (2026-11-09): no PIN by default - USV_WINDOW_PIN / --pin set one.
#
# usage: scripts/run_boat_window.sh [--lan|--ts] [--bind ADDR] [--pin PIN] [--port 8098] [--classic (v0.1.74: the v0.1.68 window at /)]     start (tmux session 'bwin')
#        scripts/run_boat_window.sh stop | status
#   on the NX  (hostname usv-pixhawk*): the boat broker here, the OCS lines through the hub (OCS_HOST:HUB_PORT, the OCS app decides),
#              bound to BOAT_HOST (this NX's address in the network profile)
#   on the Mac: the boat broker at BOAT_HOST:BOAT_PORT, the OCS lines through the OCS page (127.0.0.1:8096), bound to OCS_HOST
#   never 0.0.0.0 by default: the Mac's RoboCommand-facing NIC must not serve it. The PIN: --pin / USV_WINDOW_PIN, else a 4-digit one
#   generated and printed below (and in logs/bwin/boat_web.log). Open http://ADDR:8098/ on the other laptop.
set -uo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${BWIN_SESSION:-bwin}"
case "${1:-}" in
  stop)   tmux has-session -t "$SESSION" 2>/dev/null && { tmux send-keys -t "$SESSION:web" C-c; sleep 1; tmux kill-session -t "$SESSION" 2>/dev/null; echo "USV window stopped"; } || echo "no USV window running here"; exit 0 ;;
  status) if tmux has-session -t "$SESSION" 2>/dev/null; then echo "USV window UP (tmux $SESSION)"; grep -E "boat window of|PIN" "$KIT/logs/bwin/boat_web.log" 2>/dev/null | tail -2; else echo "USV window down"; fi; exit 0 ;;
esac
BIND=""; PIN="${USV_WINDOW_PIN:-}"; PORT="${BOAT_WEB_PORT:-8098}"
while [ $# -gt 0 ]; do
  case "$1" in
    --lan) export NET_PROFILE=lan; shift ;;
    --ts) export NET_PROFILE=ts; shift ;;
    --bind) BIND="${2:-}"; shift 2 ;;
    --pin) PIN="${2:-}"; shift 2 ;;
    --port) PORT="${2:-}"; shift 2 ;;
    --classic) export BOAT_WEB_UI=classic; shift ;;           # v0.1.74 (WP30): / serves the v0.1.68 window (the launcher's mac-classic / nx-classic)
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT"
if tmux has-session -t "$SESSION" 2>/dev/null; then echo "a USV window runs already here (tmux $SESSION): scripts/run_boat_window.sh stop first" >&2; exit 1; fi
PY="$KIT/.venv/bin/python"; [ -x "$PY" ] || PY=python3
case "$(hostname)" in
  usv-pixhawk*) WHERE=nx;  BIND="${BIND:-${BOAT_HOST:-127.0.0.1}}"; BH=127.0.0.1; OCSURL=""; OCSMQ="${OCS_HOST:-127.0.0.1}:${HUB_PORT:-1882}" ;;
  *)            WHERE=mac; BIND="${BIND:-${OCS_HOST:-127.0.0.1}}"; BH="${BOAT_HOST:-127.0.0.1}"; OCSURL="http://127.0.0.1:${OCS_WEB_PORT:-8096}"; OCSMQ="" ;;
esac
CAM="${CAM_STATUS_URL:-${AGX_HOST:+http://$AGX_HOST:${CAM_PORT:-8090}/status}}"
mkdir -p "$KIT/logs/bwin"; LOG="$KIT/logs/bwin/boat_web.log"; : > "$LOG"
ARGS=(--boat-host "$BH" --boat-port "${BOAT_PORT:-1884}" --port "$PORT" --bind "$BIND" --ocs-url "$OCSURL" --ocs-mqtt "$OCSMQ" --cam-status "${CAM:-}" --where "$WHERE")
tmux new-session -d -s "$SESSION" -n web
tmux send-keys -t "$SESSION:web" "cd $KIT && BOAT_WEB_UI=${BOAT_WEB_UI:-new} USV_WINDOW_PIN=$(printf %q "$PIN") PYTHONPATH=$KIT $PY -m optiprimex.tools.boat_web $(printf '%q ' "${ARGS[@]}") 2>&1 | tee $LOG; echo '[exited]'" C-m
for _ in $(seq 1 20); do grep -q "boat window of" "$LOG" && break; sleep 0.3; done
echo "USV window on the $WHERE: open http://$BIND:$PORT/ (tmux $SESSION; stop: scripts/run_boat_window.sh stop)"
grep -E "PIN" "$LOG" | tail -1
[ -n "$PIN" ] && echo "PIN: from USV_WINDOW_PIN / --pin (not printed)"
exit 0
