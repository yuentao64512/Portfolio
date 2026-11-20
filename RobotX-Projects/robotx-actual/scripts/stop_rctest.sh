#!/usr/bin/env bash
# Stop the RoboCommand test environment (tmux 'rctest': broker 1885 + RoboNation's test_server.py). Its logs stay in logs/rctest/.
# usage: scripts/stop_rctest.sh            -h | --help prints this
set -u
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
SESSION="${RCTEST_SESSION:-rctest}"; PORT="${RCTEST_PORT:-1885}"
"$(dirname "$0")/stop_judge.sh" --owner rctest
if tmux has-session -t "$SESSION" 2>/dev/null; then
  for w in server broker; do tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null || true; done
  sleep 1; tmux kill-session -t "$SESSION" 2>/dev/null || true
fi
sleep 0.5
if lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1 || pgrep -f 'test_server[.]py' >/dev/null; then
  echo "STILL UP: $(lsof -nP -iTCP:$PORT -sTCP:LISTEN 2>/dev/null | tail -n +2 | awk '{print $1, $2}') $(pgrep -fl 'test_server[.]py')"
else
  echo "down: nothing listens on $PORT; no test_server.py"
fi
