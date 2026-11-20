#!/usr/bin/env bash
# Our simulator (ROADMAP WP10, v0.1.32): the laptop bench + the rendered world in its own window.
#
#   scripts/run_sim.sh [pool|marina|extra|por|boxes|bench|empty|scout|FILE]   (scout = v0.1.79: S15/S16's colour-blind field, ZED 4 mm at 1.0 m) [--judge fake|rctest|venue] [--course extra|alpha|world]
#   --judge venue (v0.1.78.2): our simulated boat against the venue rc-test (VENUE_RC_HOST:VENUE_RC_PORT in network.env, protobuf,
#     the course profile; the course is theirs; the judge console read-only on that broker). Nothing else is read from the network files.
#                      [--perception camera|geometric] [--no-ui] [--detector synthetic | --detector model --model DIR] [--depth-noise zed|none] [--no-open]
#   --course (v0.1.53, sim/courses.py): the judge's course - the fake judge's --course (COURSE for run_bench.sh) or rc-test's RCTEST_COURSE;
#     default extra (Course Extra, 120 x 80 m, every task; world_extra is laid out on it), alpha = RoboNation's Course Alpha (world_marina),
#     world = the world file's own course (v0.1.46.1, the fake judge only). The judge page's course select re-publishes another one at run time.
#     When the judge's course differs from the world's, the SIM page shows "course X received - move the world there / ignore" (move =
#     the world re-anchored to the course, fp + dock restarted on it through scripts/sim_world.sh, the boat to the course's start).
#     1. scripts/run_bench.sh on that world (default pool): brokers, the fake FCU (the kinematic plant), the mission manager, the
#        fake perception + fake dock (the geometric "camera" the map uses today), the fake judge, ocs_app, op-tui (tmux optiprimex),
#        the cannon link (window cannon, v0.1.57: fake_cannon_pi + cannon_sender --source aim on a generated test key)
#     2. + windows in the same tmux session:
#          sim   optiprimex.sim.simview  - the SIM page http://127.0.0.1:8097/ (chase view, the boat's rendered camera, the top-down
#                world with mockups, arrow keys = the sticks in MANUAL, the transmitter's switches, faults)
#          web   the OCS page http://127.0.0.1:8096/ with CAM_URL = the simulator (its CAMERA tab shows the rendered camera)
#          jweb  the judge page http://127.0.0.1:8095/ (buttons ask the fake judge)
#          bweb  the boat window http://127.0.0.1:8098/ (v0.1.45, WP10f: boat-web on the boat broker 1884 - usv_ui's FCU block, the
#                wireframe, OVERVIEW; handover / takeover = usv_ui's own words; BOAT_WEB_PORT overrides the port)
#     3. opens the SIM page and the OCS page (--no-open: neither)
#   --judge rctest (v0.1.33): RoboNation's rc-test on 1885 instead of the fake judge (run_bench.sh --judge rctest: the course
#     profile, the real protobuf wire; run_rctest.sh is started unless 1885 listens already). The judge page is rc-test's (8095,
#     tmux judge), so no jweb window here; the OCS page's RoboCommand peek listens on 1885. stop rc-test: scripts/stop_rctest.sh
#     v0.1.52: RunStart is the judge page's button (RCTEST_RUNSTART=manual, run_rctest.sh's default; =auto = rc-test sends it when
#     the boat reports AUTO). m1t1: window ui `handover` · ocs_cli mission m1t1 --auto · ocs_cli ready (R4, waits) · the judge page
#     RunStart -> R5, 3-2-1, m1t1 runs (without --auto: ocs_cli start_task m1t1 after RunStart)
#   RESET SIM (the SIM page, or the OCS page's SETTINGS): the boat back to the world's start, the mockups back to the file, every
#     fault off, the mode switch HOLD, clear_map, reset on both ends, the fake judge's counters. Teleports: the world's list.
#   THE CAMERA PATH (v0.1.36, WP10e; the default --perception camera): the buoys reach the map as on the boat -
#          obs    cv/sim_observer (the rendered frames + ZED-like depth -> the students' measure_mask -> our radius / near-limit /
#                 colour rules -> runs/<stamp>/latest_status.json; --detector model --model DIR = a CV_TRAIN model on the picture)
#          cvpub  cv/publish/cv_publish.py following it -> the boat broker -> the mission manager's map (and usv_ui's after `cv on`)
#          fp     the geometric fake perception keeps only the dock faces, windows and boxes (no detector for them in the loop yet)
#   v0.1.48 (WP10j-b): with --detector model the dock comes through the camera too (sim_observer --dock: dock_face / window /
#     indicator -> cv_publish -> perception/dock + /window) and fp keeps only the boxes (--skip-kinds buoy,dock,window); the synthetic
#     masks keep the split above. e.g. scripts/run_sim.sh pool --judge rctest --detector model --model ../CV_TRAIN/models/cvt-v2-l
#   --perception geometric: the v0.1.32 path (fake perception reports the buoys too; no obs / cvpub) - the fast runs.
#   USV_UI ON THE FAKE PLANT (v0.1.36; --no-ui = without): window ui = usv_ui holding the fake FCU (udp 14550) and sharing it (14552)
#     to the mission manager, as on the boat: type `handover` there before `ready` / clear for run, `cv on` for its map; the …sim
#     mission codes work. The SIM page's switches and keys still reach the plant directly (its control port 14560).
#   --uav [--uav-scenario quiet|mpfence_alt20|mpalt_alt20|mpfence|mpsilent|mpsilentfix] (v0.1.64, WP14): the UAV in the loop - the UAV
#     team's fake copter + their geofence listener, unmodified from uav/, and our adapter (windows copter, uavlst, uav; scripts/run_uav.sh).
#     The UAV is declared only after `ocs_cli declare uav on` (then `ocs_cli uav ready on`); without --uav nothing UAV runs.
#   stop: scripts/stop_bench.sh (the whole tmux session, the simulator included)
# Needs requirements-sim.txt in the venv (numpy, Pillow): .venv/bin/pip install -r requirements-sim.txt
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$KIT"
UAV=0; UAV_SCEN=quiet; WORLD_ARG=pool; OPEN=1; JUDGE=fake; PERCEPTION=camera; UI=1; DETECTOR=synthetic; MODEL=""; DNOISE=zed; COURSE_ARG="${COURSE:-extra}"
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;;
    --no-open) OPEN=0; shift ;;
    --judge) JUDGE="$2"; shift 2 ;;
    --perception) PERCEPTION="$2"; shift 2 ;;
    --no-ui) UI=0; shift ;;
    --detector) DETECTOR="$2"; shift 2 ;;
    --model) MODEL="$2"; shift 2 ;;
    --depth-noise) DNOISE="$2"; shift 2 ;;
    --course) COURSE_ARG="$2"; shift 2 ;;
    --uav) UAV=1; shift ;;
    --uav-scenario) UAV_SCEN="$2"; shift 2 ;;
    *) WORLD_ARG="$1"; shift ;;
  esac
done
PY="$KIT/.venv/bin/python"
"$PY" -c "import numpy, PIL" 2>/dev/null || { echo "the simulator needs numpy + Pillow: .venv/bin/pip install -r requirements-sim.txt" >&2; exit 1; }
SESSION="${OPTIPRIMEX_SESSION:-optiprimex}"
case "$PERCEPTION" in camera) FP="--skip-kinds buoy" ;; geometric) FP="" ;; *) echo "--perception camera | geometric" >&2; exit 2 ;; esac
case "$DETECTOR" in synthetic) ;; model) [ -n "$MODEL" ] || { echo "--detector model needs --model DIR (a CV_TRAIN model folder)" >&2; exit 2; } ;; *) echo "--detector synthetic | model" >&2; exit 2 ;; esac
if [ "$PERCEPTION" = camera ] && [ "$DETECTOR" = model ]; then FP="--skip-kinds buoy,dock,window"; fi     # v0.1.48: the dock from the model
case "$COURSE_ARG" in extra|alpha|world|EXTRA|ALPHA) ;; *) echo "--course extra | alpha | world" >&2; exit 2 ;; esac
if [ "$JUDGE" = rctest ] && [ "$COURSE_ARG" = world ]; then echo "--course world is the fake judge's (rc-test: extra | alpha)" >&2; exit 2; fi
case "$JUDGE" in fake|rctest) ;; venue)                                 # v0.1.78.2: our simulated boat against the venue rc-test
  . "$KIT/scripts/_netenv.sh"
  load_venue_rc "$KIT" || { echo "judge venue: VENUE_RC_HOST / VENUE_RC_PORT not set - add the venue rc-test's team broker to network.env" >&2; exit 2; }
  [ "$COURSE_ARG" = world ] && { echo "--course world is the fake judge's (the venue rc-test sends its own course)" >&2; exit 2; } ;;
  *) echo "--judge fake | rctest | venue" >&2; exit 2 ;; esac
export COURSE="$COURSE_ARG" RCTEST_COURSE="${RCTEST_COURSE:-$COURSE_ARG}"   # run_bench.sh's fake judge / run_rctest.sh's rc-test
# v0.1.85 (WP41 T0.4): the simulator runs the water day's hull (omni_usv: ~2 deg/s in place, ~4 at 0.5 m/s) with the under-way guidance;
# HULL=ideal = the old 45 deg/s plant with the FCU's own GUIDED (as before); FCU_STR_RAMP_S=25 reproduces the 11 Nov steering ramp
export FCU_HULL="${HULL:-omni_usv}"
if [ "$FCU_HULL" = ideal ]; then export MM_GUIDANCE="${MM_GUIDANCE:-fcu}"; else export MM_GUIDANCE="${MM_GUIDANCE:-underway}"; fi
echo "sim plant: $FCU_HULL, guidance $MM_GUIDANCE (HULL=ideal for the old plant)"
if [ $UI -eq 1 ]; then export USV_CONFIG="${USV_CONFIG:-usv_ui/config/usv_bench_share.yaml}"; fi      # the mission manager on usv_ui's share, as on the boat
[ "$PERCEPTION" = camera ] && export CANNON_TARGET=camera CANNON_SENDER_ARGS="--target-fresh 1.5"   # v0.1.61: the sim observer measures the window
if [ -n "$FP" ]; then DOCK_AUTO=world MAP=0 WORLD="$WORLD_ARG" scripts/run_bench.sh --judge "$JUDGE" --fp-args "$FP" >/dev/null
else DOCK_AUTO=world MAP=0 WORLD="$WORLD_ARG" scripts/run_bench.sh --judge "$JUDGE" >/dev/null; fi
case "$WORLD_ARG" in empty|bench|pool|por|boxes|marina|extra|extra_t1adv|scout) WORLD_FILE="profiles/bench/world_$WORLD_ARG.yaml" ;; *) WORLD_FILE="$WORLD_ARG" ;; esac
SIM_PORT="${SIM_PORT:-8097}"; WEB_PORT="${OCS_WEB_PORT:-8096}"; JWEB_PORT="${JUDGE_WEB_PORT:-8095}"; BWEB_PORT="${BOAT_WEB_PORT:-8098}"
RUN_DIR="$(cat logs/bench/current_run)"
if [ "$JUDGE" = rctest ]; then OCS_CFG=profiles/course/ocs_course.yaml; PEEK="--judge-port ${RCTEST_PORT:-1885}"; else OCS_CFG=profiles/bench/ocs_bench.yaml; PEEK=""; fi
SIM_RC=""; [ "$JUDGE" = rctest ] && SIM_RC="--rc-port ${RCTEST_PORT:-1885}"    # v0.1.53: the SIM page hears rc-test's RxCourse too
if [ "$JUDGE" = venue ]; then                                       # v0.1.78.2: the course profile; the peek + the SIM page on the venue broker
  OCS_CFG=profiles/course/ocs_course.yaml; PEEK="--judge-host $VENUE_RC_HOST --judge-port $VENUE_RC_PORT"; SIM_RC="--rc-host $VENUE_RC_HOST --rc-port $VENUE_RC_PORT"
fi
OCS_CFG="${OCS_CONFIG:-$OCS_CFG}"
tmux new-window -d -t "$SESSION:" -n sim -c "$KIT"
tmux send-keys -t "$SESSION:sim" "$PY -m optiprimex.sim.simview --world $WORLD_FILE --fcu-control 14560 --hub-port 1882 --boat-port 1884 --port $SIM_PORT --world-hook scripts/sim_world.sh --world-dir $RUN_DIR/worlds $SIM_RC 2>&1 | tee $RUN_DIR/sim.log; echo '[exited]'" C-m
tmux new-window -d -t "$SESSION:" -n web -c "$KIT"
tmux send-keys -t "$SESSION:web" "OCS_SIM=1 CAM_URL=http://127.0.0.1:$SIM_PORT/ JUDGE_WEB_PORT=$JWEB_PORT $PY -m optiprimex.tools.ocs_web --web-port $WEB_PORT --ocs-config $OCS_CFG $PEEK 2>&1 | tee $RUN_DIR/ocs_web.log; echo '[exited]'" C-m
if [ "$JUDGE" = fake ]; then
  tmux new-window -d -t "$SESSION:" -n jweb -c "$KIT"
  tmux send-keys -t "$SESSION:jweb" "$PY -m optiprimex.tools.judge_web --port 1882 --web-port $JWEB_PORT 2>&1 | tee $RUN_DIR/judge_web.log; echo '[exited]'" C-m
fi
tmux new-window -d -t "$SESSION:" -n bweb -c "$KIT"
tmux send-keys -t "$SESSION:bweb" "OCS_WEB_PORT=$WEB_PORT $PY -m optiprimex.tools.boat_web --boat-host 127.0.0.1 --boat-port 1884 --port $BWEB_PORT 2>&1 | tee $RUN_DIR/boat_web.log; echo '[exited]'" C-m
if [ $UI -eq 1 ]; then
  tmux new-window -d -t "$SESSION:" -n ui -c "$KIT/usv_ui"
  tmux send-keys -t "$SESSION:ui" "LAUNCH_PRESET=${LAUNCH_PRESET:-} $PY usv_ui.py udpin:127.0.0.1:14550 --share udpout:127.0.0.1:14552" C-m
fi
if [ "$PERCEPTION" = camera ]; then
  OBS_OUT="$RUN_DIR/sim_observer"; mkdir -p "$OBS_OUT"                # in the bench run folder (logs/, never archived)
  MODEL_ARGS=""; [ "$DETECTOR" = model ] && MODEL_ARGS="--detector model --model $MODEL"
  tmux new-window -d -t "$SESSION:" -n obs -c "$KIT"
  tmux send-keys -t "$SESSION:obs" "PYTHONPATH=$KIT $PY cv/sim_observer/sim_observer.py --sim http://127.0.0.1:$SIM_PORT --output $OBS_OUT --depth-noise $DNOISE $MODEL_ARGS --cannon red --cannon-out $RUN_DIR/cannon_target.json 2>&1 | tee $RUN_DIR/sim_observer.log; echo '[exited]'" C-m
  tmux new-window -d -t "$SESSION:" -n cvpub -c "$KIT"
  tmux send-keys -t "$SESSION:cvpub" "$PY cv/publish/cv_publish.py --mqtt 127.0.0.1:1884 --output $OBS_OUT --detector sim_observer 2>&1 | tee $RUN_DIR/cv_publish.log; echo '[exited]'" C-m
fi
if [ "$JUDGE" = venue ]; then                                       # the judge console + page 8095, read-only on the venue broker
  JUDGE_WEB_PORT=$JWEB_PORT scripts/run_judge.sh --host "$VENUE_RC_HOST" --port "$VENUE_RC_PORT" --monitor --owner bench >/dev/null || echo "WARNING: the judge console did not start" >&2
fi
if [ $UAV -eq 1 ]; then scripts/run_uav.sh --session "$SESSION" --ocs-config "$OCS_CFG" --run-dir "$RUN_DIR" --sim --scenario "$UAV_SCEN"; fi
sleep 2.5
if [ $OPEN -eq 1 ] && command -v open >/dev/null; then open "http://127.0.0.1:$SIM_PORT/"; open "http://127.0.0.1:$WEB_PORT/"; fi
echo "judge: $JUDGE$( [ "$JUDGE" = rctest ] && echo " (rc-test on ${RCTEST_PORT:-1885}, the course profile; RunStart ${RCTEST_RUNSTART:-manual}: the judge page's RunStart button; tmux attach -t judge)" )"
[ "$JUDGE" = venue ] && echo "judge venue: our simulated boat against the venue rc-test $VENUE_RC_HOST:$VENUE_RC_PORT (the course profile, protobuf; the course is theirs)${VENUE_RC_WEB:+ - its page $VENUE_RC_WEB}; the console read-only: tmux attach -t judge"
echo "the judge's course: $COURSE_ARG (the judge page's course select re-publishes another; the SIM page offers moving the world there)"
echo "simulator up on $WORLD_FILE: SIM http://127.0.0.1:$SIM_PORT/ · OCS http://127.0.0.1:$WEB_PORT/ (CAMERA = the rendered camera) · judge http://127.0.0.1:$JWEB_PORT/"
echo "perception: $PERCEPTION$( [ "$PERCEPTION" = camera ] && echo " (obs: $DETECTOR, depth $DNOISE -> cvpub -> the map)" ) · usv_ui: $( [ $UI -eq 1 ] && echo "on the fake plant (tmux window ui: handover, cv on)" || echo off )"
echo "boat window: http://127.0.0.1:$BWEB_PORT/ (boat-web on the boat broker 1884: usv_ui's FCU block, the wireframe; handover / takeover$( [ $UI -eq 1 ] || echo " need usv_ui - --no-ui leaves them unanswered" ))"
echo "cannon link (v0.1.61): tmux window cannon = the students' receiver (DRY_RUN) on udp 127.0.0.1:${CANNON_UDP_PORT:-5005} + cannon_sender dock_color_aim.v1 (test key $RUN_DIR/cannon_test.key; the RED window from $( [ "$PERCEPTION" = camera ] && echo "the sim observer's camera" || echo "the world's truth" )); the boat window's WATER CANNON callout shows its status"
echo "drive: the SIM page's mode switch -> MANUAL, then the arrow keys; the missions from the OCS page as on the bench. stop: scripts/stop_bench.sh"
