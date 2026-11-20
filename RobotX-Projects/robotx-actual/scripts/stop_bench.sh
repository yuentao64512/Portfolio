#!/usr/bin/env bash
# Stop everything run_bench.sh started: the tmux session, or the pid-file processes of the --no-tmux mode.
# Broker persistence (/tmp/optiprimex_hub, /tmp/optiprimex_boat) is kept unless --wipe is given, so a queued
# bridge backlog survives a restart exactly as it would on the boat.
# usage: scripts/stop_bench.sh [--wipe]      -h | --help prints this
set -uo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # the MASTER_SCRIPTS root
SESSION="${OPTIPRIMEX_SESSION:-optiprimex}"
LOGDIR="$BENCH_DIR/logs/bench"
WIPE=0
[[ "${1:-}" == "--wipe" ]] && WIPE=1

tmux has-session -t "${UAV_WINDOW_SESSION:-uavwin}" 2>/dev/null && "$BENCH_DIR/scripts/run_uav_window.sh" stop     # v0.1.65: the UAV window
if command -v tmux >/dev/null && tmux has-session -t "$SESSION" 2>/dev/null; then
  # SIGINT first so the loggers flush and the mission manager publishes a clean OFFLINE, then kill the session
  for w in $(tmux list-windows -t "$SESSION" -F '#W'); do
    tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null || true
  done
  sleep 1.5
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  echo "tmux session '$SESSION' stopped"
fi

RUN_DIR="$(cat "$LOGDIR/current_run" 2>/dev/null || true)"
if [[ -n "$RUN_DIR" && -d "$RUN_DIR" ]]; then
  for f in "$RUN_DIR"/*.pid; do
    [[ -f "$f" ]] || continue
    pid="$(cat "$f")"
    if kill -0 "$pid" 2>/dev/null; then
      kill -INT "$pid" 2>/dev/null || true
    fi
  done
  sleep 1.5
  for f in "$RUN_DIR"/*.pid; do
    [[ -f "$f" ]] || continue
    pid="$(cat "$f")"
    kill -0 "$pid" 2>/dev/null && kill -KILL "$pid" 2>/dev/null || true
    rm -f "$f"
  done
fi

# anything still holding the bench ports (a crashed pane, a stray mosquitto started by hand)
for port in 1882 1884 9001; do
  for pid in $(lsof -nP -tiTCP:$port -sTCP:LISTEN 2>/dev/null); do
    kill -TERM "$pid" 2>/dev/null && echo "killed pid $pid on port $port" || true
  done
done

if [[ $WIPE -eq 1 ]]; then
  rm -rf /tmp/optiprimex_hub /tmp/optiprimex_boat
  echo "broker persistence wiped"
fi
"$BENCH_DIR/scripts/stop_judge.sh" --owner bench >/dev/null 2>&1 || true     # v0.1.78.2: the venue judge console run_sim.sh opened
echo "bench stopped"
