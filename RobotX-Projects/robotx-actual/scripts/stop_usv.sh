#!/usr/bin/env bash
# Stop the 'usv' tmux stack started by run_usv.sh: usv_ui gets 'quit' (it zeros mode-20 outputs when it has the
# FCU, leaves it alone when handed over), then SIGINT to the rest, then the session is killed. --wipe clears the broker
# persistence (/tmp/optiprimex_boat).
# usage: scripts/stop_usv.sh [--wipe]        -h | --help prints this
set -u
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${USV_SESSION:-usv}"
# v0.1.63: the USV window (tmux bwin, scripts/run_boat_window.sh) goes first - it serves the team LAN
tmux has-session -t "${BWIN_SESSION:-bwin}" 2>/dev/null && "$KIT/scripts/run_boat_window.sh" stop
# ---- network addresses: ../network.env + the profile overlay network.ts.env | network.lan.env (--ts / --lan, NET_PROFILE); the command line wins
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT"
BOAT_PORT="${BOAT_PORT:-1884}"; BOAT_WS="${BOAT_WS:-9002}"           # v0.1.90.12: always 9002 (9001 = the hub's websocket; one number per job)
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux send-keys -t "$SESSION:ui" "quit" C-m 2>/dev/null || true
  sleep 1.5
  for w in mm host log fcu boat; do tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null || true; done
  sleep 1.5
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  echo "usv stack stopped"
else
  echo "no tmux session '$SESSION'"
fi
rm -f "$KIT/usv_ui/.share"
if [ "${1:-}" = "--wipe" ] || [ "${WIPE:-0}" = "1" ]; then rm -rf /tmp/optiprimex_boat; echo "broker persistence wiped (retained presence / old beats gone)"; fi
# ---- is it really down? the boat broker's ports and the programs themselves ---------------------------------------------
sleep 0.5
left="$( (ss -ltn 2>/dev/null || lsof -nP -iTCP -sTCP:LISTEN 2>/dev/null) | grep -E ":($BOAT_PORT|$BOAT_WS) " || true)"
procs="$(pgrep -fl 'usv_ui\.py|mission_manager|fake_fcu|companion\.logger|host_health' | grep -vE "^($$|$PPID) " || true)"       # not this script nor the shell that typed it
if [ -z "$left" ] && [ -z "$procs" ]; then
  echo "down: nothing listens on $BOAT_PORT/$BOAT_WS; no usv_ui, mission_manager, fake_fcu, logger or host_health process left"
else
  echo "STILL UP:"; [ -n "$left" ] && echo "$left"; [ -n "$procs" ] && echo "$procs"
  echo "  -> pkill -f mission_manager; pkill -f 'usv_ui.p[y]'; pkill -f 'fake_fc[u]'; pkill -f 'host_healt[h]'; pkill -x mosquitto   (then run this script again)"
fi
