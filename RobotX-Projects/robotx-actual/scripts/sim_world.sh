#!/usr/bin/env bash
# Switch the laptop bench's fake CV to another world (v0.1.28.1): windows fp (fake_perception) and dock (fake_dock) of the
# bench's tmux session are restarted on that world; the map's truth layer follows (fake_perception clears sim/truth on exit).
#
# usage: scripts/sim_world.sh empty|bench|pool|por|boxes|marina|extra|FILE    a world (a name = profiles/bench/world_<name>.yaml;
#        FILE: also a world the SIM page moved to another course, logs/sim/worlds/*.yaml - v0.1.53)
#        scripts/sim_world.sh off                               stop both: no detections, no truth layer
#        -h | --help prints this
# The boat's own map ages the old objects out (stale, then dropped). On the Jetsons the sim CV is run_agx.sh --sim --world FILE.
# Bench only (run_bench.sh's tmux session); plain bash 3.2.
set -uo pipefail
case "${1:-}" in ""|-h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$BENCH_DIR"
SESSION="${OPTIPRIMEX_SESSION:-optiprimex}"
PY="${PY:-$BENCH_DIR/.venv/bin/python}"
case "$1" in
  off) WORLD="" ;;
  empty|bench|pool|por|boxes|marina|extra|extra_t1adv|scout) WORLD="profiles/bench/world_$1.yaml" ;;
  *) WORLD="$1" ;;
esac
[ -z "$WORLD" ] || [ -f "$WORLD" ] || { echo "world file not found: $WORLD" >&2; exit 2; }
tmux has-session -t "$SESSION" 2>/dev/null || { echo "no bench running (tmux session '$SESSION'): scripts/run_bench.sh" >&2; exit 1; }
RUN_DIR="$(cat logs/bench/current_run 2>/dev/null || echo logs/bench)"

for w in fp dock; do                       # Ctrl-C first: fake_perception publishes presence down and clears sim/truth
  tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$w" && tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null
done
sleep 1.5
if [ -z "$WORLD" ]; then
  echo "fake CV off (windows fp, dock idle); restart: scripts/sim_world.sh empty|bench|pool|por|boxes"
  exit 0
fi
for w in fp dock; do
  case "$w" in
    fp)   cmd="$PY -m optiprimex.sim.fake_perception --world $WORLD --port 1884 --hub-port 1882" ;;
    dock) cmd="$PY -m optiprimex.sim.fake_dock --world $WORLD --port 1884 --auto on" ;;
  esac
  tmux list-windows -t "$SESSION" -F '#W' | grep -qx "$w" || tmux new-window -d -t "$SESSION:" -n "$w" -c "$BENCH_DIR"
  tmux send-keys -t "$SESSION:$w" "$cmd 2>&1 | tee -a $RUN_DIR/$w.log; echo '[exited]'" C-m
done
echo "fake CV on $WORLD (windows fp, dock of tmux '$SESSION')"
