#!/usr/bin/env bash
# Stop the 'ocs' tmux stack started by run_ocs.sh (SIGINT to every window, then kill the session). --wipe clears
# the hub persistence (/tmp/optiprimex_hub).
# usage: scripts/stop_ocs.sh [--wipe]        -h | --help prints this
set -u
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${OCS_SESSION:-ocs}"
# v0.1.63: the USV window (tmux bwin, scripts/run_boat_window.sh) goes first - it serves the team LAN
tmux has-session -t "${BWIN_SESSION:-bwin}" 2>/dev/null && "$KIT/scripts/run_boat_window.sh" stop
tmux has-session -t "${UAV_WINDOW_SESSION:-uavwin}" 2>/dev/null && "$KIT/scripts/run_uav_window.sh" stop     # v0.1.65: the UAV window
# ---- network addresses: ../network.env + the profile overlay network.ts.env | network.lan.env (--ts / --lan, NET_PROFILE); the command line wins
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT"
HUB_PORT="${HUB_PORT:-1882}"; HUB_WS="${HUB_WS:-9001}"
if tmux has-session -t "$SESSION" 2>/dev/null; then
  for w in tui ocs rc log hub; do tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null || true; done
  sleep 1.5
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  echo "ocs stack stopped"
else
  echo "no tmux session '$SESSION'"
fi
if [ "${1:-}" = "--wipe" ] || [ "${WIPE:-0}" = "1" ]; then rm -rf /tmp/optiprimex_hub; echo "hub persistence wiped (retained presence / old beats gone)"; fi
# ---- is it really down? the hub's ports and the programs themselves -------------------------------------------------------
sleep 0.5
"$(dirname "$0")/stop_judge.sh" --owner ocs
left="$( (lsof -nP -iTCP -sTCP:LISTEN 2>/dev/null || ss -ltn 2>/dev/null) | grep -E ":($HUB_PORT|$HUB_WS) " || true)"
procs="$(pgrep -fl 'ocs_app|fake_robocommand|companion\.logger|bench_tui|ocs_web' | grep -vE "^($$|$PPID) " || true)"       # not this script nor the shell that typed it
if [ -z "$left" ] && [ -z "$procs" ]; then
  echo "down: nothing listens on $HUB_PORT/$HUB_WS; no ocs_app, fake_robocommand, logger, op-tui or ocs_web process left (the judge console: tmux judge)"
else
  echo "STILL UP:"; [ -n "$left" ] && echo "$left"; [ -n "$procs" ] && echo "$procs"
  echo "  -> pkill -f ocs_app; pkill -f fake_robocommand; pkill -x mosquitto   (a brew service? brew services stop mosquitto)"
fi
