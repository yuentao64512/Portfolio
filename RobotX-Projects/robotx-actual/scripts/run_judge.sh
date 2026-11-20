#!/usr/bin/env bash
# The judge's console in ONE tmux session named `judge` (v0.1.26, ROADMAP WP7): window tui = rc-tui (both wires decoded, typed
# commands), window web = judge_web (the same view in the browser, http://127.0.0.1:8095/, read-only).
# The launchers open it for you: scripts/run_ocs.sh (fake judge), scripts/run_rctest.sh (rc-test, --as-judge), run_ocs.sh
# --judge course (--monitor). By hand:
#   scripts/run_judge.sh [--host H] [--port P] [--as-judge [--wire protobuf|json] | --monitor] [--open] [--no-web] [--owner NAME]
#     default --port 1882 (our hub; the fake judge publishes for you); --port 1885 = rc-test; --open = the page in the browser
# Then:   tmux attach -t judge        (Ctrl-b n/p = tui / web, Ctrl-b d leaves it running)       stop: scripts/stop_judge.sh
# usage:  scripts/run_judge.sh -h | --help          JUDGE_WEB_PORT=8095 (default)
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$KIT"
HOST=127.0.0.1; PORT=1882; MODE=""; WIRE=""; OPEN=0; WEB=1; OWNER=hand; TEAM="${TEAM_ID:-SGIT}"
while [ $# -gt 0 ]; do
  case "$1" in
    --host) HOST="$2"; shift 2 ;;
    --port) PORT="$2"; shift 2 ;;
    --as-judge) MODE="--as-judge"; shift ;;
    --monitor) MODE="--monitor"; shift ;;
    --wire) WIRE="--wire $2"; shift 2 ;;
    --open) OPEN=1; shift ;;
    --no-web) WEB=0; shift ;;
    --owner) OWNER="$2"; shift 2 ;;
    --team) TEAM="$2"; shift 2 ;;
    *) echo "unknown option $1 (scripts/run_judge.sh --help)" >&2; exit 2 ;;
  esac
done
PY="$KIT/.venv/bin/python"; [ -x "$PY" ] || PY="$(command -v python3)"
WEB_PORT="${JUDGE_WEB_PORT:-8095}"
tmux kill-session -t judge 2>/dev/null || true
tmux new-session -d -s judge -n tui -c "$KIT"
tmux set-environment -t judge JUDGE_OWNER "$OWNER"                     # stop_ocs / stop_rctest close only their own console
tmux send-keys -t judge:tui "$PY -m optiprimex.tools.rc_tui --host $HOST --port $PORT --team $TEAM $MODE $WIRE" C-m
if [ $WEB -eq 1 ]; then
  tmux new-window -t judge: -n web -c "$KIT"
  tmux send-keys -t judge:web "$PY -m optiprimex.tools.judge_web --host $HOST --port $PORT --team $TEAM $MODE $WIRE --web-port $WEB_PORT $( [ $OPEN -eq 1 ] && echo --open ); echo '[exited]'" C-m
  tmux select-window -t judge:tui
fi
echo "judge console: tmux attach -t judge   (rc-tui on $HOST:$PORT${MODE:+ $MODE}; both wires shown)$( [ $WEB -eq 1 ] && echo "   browser: http://127.0.0.1:$WEB_PORT/" )"
