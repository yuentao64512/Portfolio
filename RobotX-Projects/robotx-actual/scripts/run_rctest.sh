#!/usr/bin/env bash
# Runs ON THE MAC: RoboNation's RoboCommand test environment without docker (ROADMAP WP4, docs/ROBOCOMMAND.md), tmux 'rctest':
#   broker   mosquitto on 1885 (profiles/course/mosquitto/rctest.conf) - never 1882, that is our hub
#   server   RoboNation's rc_test/test_server.py, unmodified, from the robocommand checkout (RC_REPO, default ../robocommand
#            beside this tree, commit 8fc09f2): publishes the retained RxCourse (v0.1.53: Course Extra by default, below), checks declarations, sends
#            RunStart when every declared vehicle reports STATE_AUTO (only with RCTEST_RUNSTART=auto, below), logs every decoded message to
#            logs/rctest/run_<stamp>/robocommand_<YYYYMMDD>_<HHMMSS>.log (the Proof-of-Readiness artifact)
# Also:   tmux `judge` = the judge's console (rc-tui --as-judge on 1885) + the judge page http://127.0.0.1:8095/ wired to 1885 - always
#         started; JUDGE_OPEN=0 (the old name JUDGE_WEB=0 still works) = do not open it in the browser (the launcher links it)
# Then:   tmux attach -t rctest                (see the server's log; Ctrl-b d leaves it running)
#         tmux attach -t judge                 (type start / assist / confirm ... as RoboCommand would send them)
#         scripts/run_ocs.sh --judge rctest   (the OCS on the real wire against it)    stop: scripts/stop_rctest.sh
# RunStart (v0.1.52): RCTEST_RUNSTART=manual (default) = the server runs through optiprimex.tools.rctest_manual - the same file,
#         its auto-RunStart replaced by a log line `[manual-start] ...`: press RunStart on the judge page (8095) or rc-tui `start`;
#         RCTEST_RUNSTART=auto = test_server.py as it is (RunStart the moment every declared vehicle reports AUTO, as before v0.1.52)
# Course (v0.1.53): RCTEST_COURSE=extra (default) | alpha - the RxCourse rc-test publishes and checks the UAV geofence against
#         (optiprimex/sim/courses.py: EXTRA = our 120 x 80 m course containing Alpha; alpha = RoboNation's own Course Alpha, the file's
#         values). Set through rctest_manual (the server file stays unmodified); RCTEST_RUNSTART=auto + RCTEST_COURSE=alpha = the bare
#         test_server.py as before. The judge page's "publish course" re-publishes another course on 1885 and rctest_manual follows it
#         (RCTEST_FOLLOW=0: it does not); the bare test_server.py does not - restart: scripts/stop_rctest.sh; RCTEST_COURSE=alpha scripts/run_rctest.sh
# usage:  scripts/run_rctest.sh [-h | --help]       RC_REPO=/path/to/robocommand RCTEST_PORT=1885 RCTEST_RUNSTART=manual RCTEST_COURSE=extra (defaults)
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$KIT"
RC_REPO="${RC_REPO:-$KIT/../robocommand}"
RCX="$RC_REPO/RobotX_2026"
PORT="${RCTEST_PORT:-1885}"
SESSION="${RCTEST_SESSION:-rctest}"
RUN_DIR="$KIT/logs/rctest/run_$(date +%Y%m%d_%H%M%S)"
[ -f "$RCX/rc_test/test_server.py" ] || { echo "RoboNation's repo not found at $RC_REPO: gh repo clone robonation/robocommand $RC_REPO && git -C $RC_REPO checkout 8fc09f2" >&2; exit 1; }
PY="$KIT/.venv/bin/python"
RUNSTART="${RCTEST_RUNSTART:-manual}"
case "$RUNSTART" in
  manual) SERVER_CMD="PYTHONPATH=$RCX/gen/python:$KIT $PY -m optiprimex.tools.rctest_manual test_server.py" ;;
  auto)   SERVER_CMD="PYTHONPATH=$RCX/gen/python $PY test_server.py" ;;
  *) echo "RCTEST_RUNSTART=manual | auto (got $RUNSTART)" >&2; exit 2 ;;
esac
COURSE="$(echo "${RCTEST_COURSE:-extra}" | tr '[:upper:]' '[:lower:]')"     # v0.1.53: the course (sim/courses.py)
case "$COURSE" in
  extra|alpha) ;;
  *) echo "RCTEST_COURSE=extra | alpha (got $COURSE)" >&2; exit 2 ;;
esac
if [ "$RUNSTART" = auto ] && [ "$COURSE" != alpha ]; then         # the auto-RunStart kept, our course through the runner
  SERVER_CMD="RCTEST_RUNSTART=auto PYTHONPATH=$RCX/gen/python:$KIT $PY -m optiprimex.tools.rctest_manual test_server.py"
fi
"$PY" -c "import google.protobuf, paho.mqtt" 2>/dev/null || { echo "$PY lacks protobuf / paho: make venv (requirements.txt has protobuf==7.35.0)" >&2; exit 1; }
MOSQ="$(command -v mosquitto || true)"; [ -z "$MOSQ" ] && for c in /opt/homebrew/sbin/mosquitto /usr/local/sbin/mosquitto; do [ -x "$c" ] && MOSQ="$c"; done
[ -n "$MOSQ" ] || { echo "mosquitto not found: brew install mosquitto" >&2; exit 1; }
if lsof -nP -iTCP:"$PORT" -sTCP:LISTEN >/dev/null 2>&1; then echo "port $PORT already in use (scripts/stop_rctest.sh?)" >&2; exit 1; fi
mkdir -p "$RUN_DIR"; ln -sfn "$RUN_DIR" "$KIT/logs/rctest/latest"
sed "s/^listener 1885/listener $PORT/" profiles/course/mosquitto/rctest.conf > "$RUN_DIR/rctest.conf"
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n broker -c "$KIT"
tmux send-keys -t "$SESSION:broker" "$MOSQ -c $RUN_DIR/rctest.conf 2>&1 | tee $RUN_DIR/broker.log; echo '[exited]'" C-m
sleep 0.8
tmux new-window -t "$SESSION:" -n server -c "$RCX/rc_test"
tmux send-keys -t "$SESSION:server" "BROKER_HOST=127.0.0.1 BROKER_PORT=$PORT RCTEST_COURSE=$COURSE LOG_DIR=$RUN_DIR $SERVER_CMD 2>&1 | tee $RUN_DIR/server.out; echo '[exited]'" C-m
echo "rc-test up in tmux session '$SESSION': broker 127.0.0.1:$PORT, RoboNation test_server.py ($(git -C "$RC_REPO" log -1 --format=%h 2>/dev/null || echo '?')), RunStart $RUNSTART$( [ "$RUNSTART" = manual ] && echo " (the judge page's RunStart)" ), course $(echo "$COURSE" | tr '[:lower:]' '[:upper:]'); log: $RUN_DIR/robocommand_*.log"
# v0.1.26: the judge's console (tmux `judge`: rc-tui --as-judge + the judge page, always; opened in the browser unless JUDGE_OPEN=0)
"$KIT/scripts/run_judge.sh" --port "$PORT" --as-judge --owner rctest $( [ "${JUDGE_OPEN:-${JUDGE_WEB:-1}}" = "1" ] && echo --open ) >/dev/null
echo "see it:  tmux attach -t $SESSION        (window server = RoboNation's log, broker; Ctrl-b n/p switch, Ctrl-b d leave it running)"
echo "judge console: tmux attach -t judge     (rc-tui --as-judge on $PORT: start / assist / confirm / keepout ...; browser: http://127.0.0.1:${JUDGE_WEB_PORT:-8095}/)"
echo "next:    scripts/run_ocs.sh --judge rctest"
echo "stop:    scripts/stop_rctest.sh"
