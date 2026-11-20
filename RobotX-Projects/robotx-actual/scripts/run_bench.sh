#!/usr/bin/env bash
# Start the whole laptop bench (ARCHITECTURE 7, B1 + B2) in one tmux session:
#
#   hub      mosquitto 1882 (+ws 9001)        the OCS hub broker
#   boat     mosquitto 1884 -> bridge -> hub  stands in for the Orin's broker
#   fcu      fake Pixhawk on udp 14550        kinematic plant, JSON control on udp 14560, mirror on 14551 (TUI)
#   mm       mission manager (usv_bench.yaml) the real companion code, talking to the fake FCU
#   fp       sim/fake_perception.py           detections of the sim world -> the map. Default (v0.1.28.1): world_empty.yaml, nothing
#                                             in view; WORLD=bench|pool|por|boxes (or a file) at start, or later scripts/sim_world.sh NAME /
#                                             the OCS page's SETTINGS -> Sim scenario (bench = gates + dock, pool = the 50 x 25 m pool,
#                                             por = the PoR course, boxes = the m0t4 boxes)
#   dock     sim/fake_dock.py                 Task 3 dock lights of the world file (GREEN bay, target window; sim/dock/cmd)
#   cannon   sim/fake_cannon_pi.py            v0.1.57 (WP12): the cannon's Pi on udp 127.0.0.1:${CANNON_UDP_PORT:-5005} + its child
#                                             cv/cannon/cannon_sender.py (--source aim, signed with a TEST key generated into
#                                             logs/bench/run_<stamp>/cannon_test.key); local/usv/cannon/status at 2 Hz
#                                             v0.1.61: the Pi = the students' color_cannon_receiver.py in DRY_RUN (run_<stamp>/cannon_pi/),
#                                             dock_color_aim.v1, the target (the lit RED window) from the world's truth into
#                                             run_<stamp>/cannon_target.json (CANNON_TARGET=camera: run_sim.sh's sim observer writes it)
#   rc       fake RoboCommand on the hub      RunStart 2 s after a valid declaration; its course: COURSE=extra (default) | alpha | world
#                                             (v0.1.53, sim/courses.py; world = the world file's `course:`); the judge page switches it
#   ocs      ocs_app (ocs_bench.yaml)         judge client + cmd authority + heartbeats
#   loghub   SQLite logger on the hub         logs/bench/run_<stamp>/hub_<stamp>.sqlite
#   tui      op-tui bench console            ladders R0-R6 / V0-V6, BOAT tab (text map, detector, who has the FCU), typed intents
#   logboat  SQLite logger on the boat broker logs/bench/run_<stamp>/boat_<stamp>.sqlite
#
# Usage:  scripts/run_bench.sh [--no-tmux] [--rc-args "..."] [--fcu-args "..."] [--judge fake|rctest|venue]   (venue, v0.1.78.2: the venue rc-test, VENUE_RC_* in network.env)
#   --judge     fake (default): the fake RoboCommand (window rc) and profiles/bench/ocs_bench.yaml
#               rctest (v0.1.33): RoboNation's rc-test on 127.0.0.1:1885 (scripts/run_rctest.sh is started unless 1885 already
#               listens), the real protobuf wire, profiles/course/ocs_course.yaml (the course rules: no goto, no inject; T4 from
#               the judge page); no window rc. The judge console is rc-test's (tmux judge, page 8095)
#   --no-tmux   run everything as background processes with pid files in logs/bench/run_*/ (stop with stop_bench.sh)
#   --rc-args   extra args for fake_robocommand, e.g. "--refuse" (stall path) or "--auto-incident-after 5"
#   --fcu-args  extra args for fake_fcu, e.g. "--mode 0" or "--lat 1.31 --lon 103.81"
#   --fp-args   extra args for fake_perception, e.g. "--hz 2 --seed 7"
#   --uav [--uav-scenario S]   v0.1.64 (WP14): + the UAV windows (scripts/run_uav.sh --sim): his fake copter + listener from uav/, our
#               adapter; `ocs_cli declare uav on` to declare it (off by default)
#   MAP=0             do not open the browser map (optiprimex/tools/map.html on ws 9001) at start
#   OCS_CONFIG=path   ocs_app profile (default by --judge: ocs_bench.yaml | ocs_course.yaml)
#   USV_CONFIG=path   mission manager profile (default profiles/bench/usv_bench.yaml; the usv_ui rehearsal uses
#                     usv_ui/config/usv_bench_share.yaml, where the FCU endpoint is usv_ui's share 14551)
# Then:   ocs_cli state | ocs_cli declare | ocs_cli enable_auto | ocs_cli start_task T4 | ...   (see README)
#
# Plain POSIX-ish bash: works with macOS /bin/bash 3.2 (no associative arrays).
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"   # for --help after cd

BENCH_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # the MASTER_SCRIPTS root
cd "$BENCH_DIR"
SESSION="${OPTIPRIMEX_SESSION:-optiprimex}"
LOGDIR="$BENCH_DIR/logs/bench"
RUN_DIR="$LOGDIR/run_$(date +%Y%m%d_%H%M%S)"
USE_TMUX=1
RC_ARGS="--start-delay 2 --confirm-delay 1"
FCU_ARGS="--armed"
FP_ARGS=""
JUDGE=fake
UAV=0; UAV_SCEN=quiet
WORLD="${WORLD:-empty}"                                          # v0.1.28.1: no fake buoys until a scenario is picked
case "$WORLD" in empty|bench|pool|por|boxes|marina|extra|extra_t1adv|scout) WORLD="profiles/bench/world_$WORLD.yaml" ;; esac
RC_COURSE="${COURSE:-extra}"                                     # v0.1.53: the fake judge's course (extra | alpha | world)

while [ $# -gt 0 ]; do
  case "$1" in
    --no-tmux) USE_TMUX=0; shift ;;
    --rc-args) RC_ARGS="$RC_ARGS $2"; shift 2 ;;
    --fcu-args) FCU_ARGS="$FCU_ARGS $2"; shift 2 ;;
    --fp-args) FP_ARGS="$FP_ARGS $2"; shift 2 ;;
    --judge) JUDGE="$2"; shift 2 ;;
    --uav) UAV=1; shift ;;
    --uav-scenario) UAV_SCEN="$2"; shift 2 ;;
    -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done

RCENV=""
case "$JUDGE" in
  fake)   OCS_CONFIG="${OCS_CONFIG:-profiles/bench/ocs_bench.yaml}" ;;
  rctest) OCS_CONFIG="${OCS_CONFIG:-profiles/course/ocs_course.yaml}"; RCENV="RC_HOST=127.0.0.1 RC_PORT=${RCTEST_PORT:-1885} " ;;
  venue)  . "$BENCH_DIR/scripts/_netenv.sh"                               # v0.1.78.2: the venue rc-test (our team broker on it), protobuf
          load_venue_rc "$BENCH_DIR" || { echo "judge venue: VENUE_RC_HOST / VENUE_RC_PORT not set - add the venue rc-test's team broker to network.env" >&2; exit 2; }
          OCS_CONFIG="${OCS_CONFIG:-profiles/course/ocs_course.yaml}"; RCENV="RC_HOST=$VENUE_RC_HOST RC_PORT=$VENUE_RC_PORT " ;;
  *) echo "--judge fake | rctest | venue" >&2; exit 2 ;;
esac
[ -f "$WORLD" ] || { echo "world file not found: $WORLD (WORLD=empty|bench|pool|por|boxes|marina|extra or a file)" >&2; exit 2; }

# ---- toolchain -------------------------------------------------------------------------------------------
PY="${PY:-$BENCH_DIR/.venv/bin/python}"
[ -x "$PY" ] || { echo "no venv at $PY - run 'make venv' first" >&2; exit 1; }
START_ARGS="$("$PY" -m optiprimex.sim.courses --start-args "$WORLD" 2>/dev/null || true)"   # v0.1.53: '' unless the world is course-anchored
MOSQ="${MOSQUITTO:-$(command -v mosquitto || true)}"
for cand in /opt/homebrew/sbin/mosquitto /usr/local/sbin/mosquitto /usr/sbin/mosquitto; do
  if [ -z "$MOSQ" ] && [ -x "$cand" ]; then MOSQ="$cand"; fi
done
[ -n "$MOSQ" ] || { echo "mosquitto not found - 'brew install mosquitto' (do NOT 'brew services start' it; the bench runs its own two)" >&2; exit 1; }
if [ $USE_TMUX -eq 1 ] && ! command -v tmux >/dev/null; then
  echo "tmux not found ('brew install tmux'); falling back to --no-tmux" >&2; USE_TMUX=0
fi

# a leftover bench (or a brew-started mosquitto on 1882) would make the brokers fail to bind
for port in 1882 1884 9001; do
  if lsof -nP -iTCP:$port -sTCP:LISTEN >/dev/null 2>&1; then
    echo "port $port is already in use (old bench or a system mosquitto?) - run scripts/stop_bench.sh or stop that service" >&2
    exit 1
  fi
done

mkdir -p "$RUN_DIR" /tmp/optiprimex_hub /tmp/optiprimex_boat
# mosquitto 2.x started as root drops to user 'mosquitto' and must still be able to write its persistence dir
if [ "$(id -u)" -eq 0 ]; then chmod 777 /tmp/optiprimex_hub /tmp/optiprimex_boat; fi
ln -sfn "$RUN_DIR" "$LOGDIR/latest"
echo "$RUN_DIR" > "$LOGDIR/current_run"
echo "$JUDGE" > "$RUN_DIR/judge"

# ---- the nodes (start order matters: brokers, plant, companion, judge, OCS, loggers) -----------------------
NODES="hub boat fcu mm fp dock cannon rc ocs loghub logboat host"
if [ "$JUDGE" = venue ]; then
  NODES="hub boat fcu mm fp dock cannon ocs loghub logboat host"     # v0.1.78.2: the judge is the venue rc-test, not ours
fi
if [ "$JUDGE" = rctest ]; then
  NODES="hub boat fcu mm fp dock cannon ocs loghub logboat host"     # the judge is rc-test, not ours
  if lsof -nP -iTCP:"${RCTEST_PORT:-1885}" -sTCP:LISTEN >/dev/null 2>&1; then
    echo "rc-test already listens on ${RCTEST_PORT:-1885}: using it (its judge console and page stay as they are)"
  else
    JUDGE_WEB="${JUDGE_WEB:-0}" scripts/run_rctest.sh >/dev/null || { echo "rc-test did not start (scripts/run_rctest.sh)" >&2; exit 1; }
    sleep 1.5
  fi
fi

cmd_for() {
  case "$1" in
    hub)     echo "$MOSQ -c profiles/bench/mosquitto/hub.conf" ;;
    boat)    echo "$MOSQ -c profiles/bench/mosquitto/boat.conf" ;;
    fcu)     echo "$PY -m optiprimex.sim.fake_fcu --conn udpout:127.0.0.1:14550 --control-port 14560 --mirror 14551 --hull ${FCU_HULL:-ideal} --str-ramp-s ${FCU_STR_RAMP_S:-0} $START_ARGS $FCU_ARGS" ;;   # v0.1.53: a course world's start
    mm)      echo "$PY -m optiprimex.usv.mission_manager --config ${USV_CONFIG:-profiles/bench/usv_bench.yaml} --trace $RUN_DIR/mm_trace.jsonl --map-rec $RUN_DIR/maps${MM_GUIDANCE:+ --guidance $MM_GUIDANCE}" ;;
    fp)      echo "$PY -m optiprimex.sim.fake_perception --world $WORLD --port 1884 --hub-port 1882 $FP_ARGS" ;;
    dock)    echo "$PY -m optiprimex.sim.fake_dock --world $WORLD --port 1884 --auto ${DOCK_AUTO:-on}" ;;   # v0.1.48: the bench lights the window by itself (usv_ui dockto sends no DockingReport); run_sim.sh: DOCK_AUTO=world
    cannon)  echo "$PY -m optiprimex.sim.fake_cannon_pi --port 1884 --listen 127.0.0.1:${CANNON_UDP_PORT:-5005} --key-out $RUN_DIR/cannon_test.key --target-out $RUN_DIR/cannon_target.json$( [ "${CANNON_TARGET:-truth}" = truth ] && echo " --truth-world $WORLD" ) --with-sender '--mqtt 127.0.0.1:1884${CANNON_SENDER_ARGS:+ $CANNON_SENDER_ARGS}'" ;;   # v0.1.57: the Pi + the sender, a test key (never the real one); v0.1.61: the students' receiver (DRY_RUN), the target from the truth (CANNON_TARGET=camera: the sim observer writes it)
    rc)      echo "$PY -m optiprimex.sim.fake_robocommand --port 1882 --world $WORLD --course $RC_COURSE --log $RUN_DIR/robocommand.jsonl $RC_ARGS" ;;   # v0.1.53: --course (world = v0.1.46.1's course from the world)
    ocs)     echo "${RCENV}$PY -m optiprimex.ocs.ocs_app --config $OCS_CONFIG" ;;
    loghub)  echo "$PY -m optiprimex.companion.logger --port 1882 --name hub --client-id logger-hub --run-dir $RUN_DIR" ;;
    logboat) echo "$PY -m optiprimex.companion.logger --port 1884 --name boat --client-id logger-boat --run-dir $RUN_DIR" ;;
    host)    echo "$PY -m optiprimex.tools.host_health --host nx,agx --simulated --mqtt 127.0.0.1:1884" ;;   # v0.1.55 (D3): the Jetsons' beats from this Mac, tagged simulated
  esac
}
delay_for() {
  case "$1" in
    hub|boat|fcu) echo 0.8 ;;
    mm) echo 1.5 ;;
    rc|ocs|fp|dock|cannon) echo 0.5 ;;
    *) echo 0 ;;
  esac
}

if [ $USE_TMUX -eq 1 ]; then
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  tmux new-session -d -s "$SESSION" -n hub -c "$BENCH_DIR"
  first=1
  for n in $NODES; do
    if [ $first -eq 1 ]; then first=0; else tmux new-window -t "$SESSION" -n "$n" -c "$BENCH_DIR"; fi
    # keep the pane open after a crash so the traceback stays readable
    tmux send-keys -t "$SESSION:$n" "$(cmd_for "$n") 2>&1 | tee $RUN_DIR/$n.log; echo '[exited]'" C-m
    sleep "$(delay_for "$n")"
  done
  tmux new-window -t "$SESSION" -n cli -c "$BENCH_DIR"
  tmux send-keys -t "$SESSION:cli" "source .venv/bin/activate; echo 'try: ocs_cli next   |   op-scenario list   |   op-tui'" C-m
  # the bench console: ladders, log, one-key intents; starts after the stack is up so its first screen is live
  tmux new-window -t "$SESSION" -n tui -c "$BENCH_DIR"
  tmux send-keys -t "$SESSION:tui" "sleep 1.5; $PY -m optiprimex.tools.bench_tui" C-m
  tmux select-window -t "$SESSION:tui"
  if [ $UAV -eq 1 ]; then "$BENCH_DIR/scripts/run_uav.sh" --session "$SESSION" --ocs-config "$OCS_CONFIG" --run-dir "$RUN_DIR" --sim --scenario "$UAV_SCEN"; fi
  echo "bench up in tmux session '$SESSION'  (tmux attach -t $SESSION opens on the TUI; Ctrl-b p = cli window, Ctrl-b n/p switch, Ctrl-b d detach)"
  # the browser map (optiprimex/tools/map.html over the hub's websocket 9001); MAP=0 skips it, 'map' in the TUI reopens it
  if [ "${MAP:-1}" = "1" ]; then "$PY" -c "from optiprimex.tools.bench_tui import open_map; print(open_map('127.0.0.1', 9001))" || true; fi
else
  for n in $NODES; do
    nohup bash -c "$(cmd_for "$n")" > "$RUN_DIR/$n.log" 2>&1 &
    echo $! > "$RUN_DIR/$n.pid"
    sleep "$(delay_for "$n")"
  done
  echo "bench up (background processes; pid files in $RUN_DIR)"
fi
echo "sim world: $WORLD  (another: scripts/sim_world.sh bench|pool|por|boxes|marina|extra|empty|off) · the fake judge's course: $RC_COURSE"
echo "logs: $RUN_DIR  (symlink logs/bench/latest)"
echo "stop: scripts/stop_bench.sh"
