#!/usr/bin/env bash
# Runs ON THE AGX ORIN: cv/publish/cv_publish.py in tmux session 'agx' - the frames written by cv/buoys/run.sh and
# cv/docking/run.sh (our snapshots of the student programs) -> the boat broker on the NX.
#
#   scripts/run_agx.sh [--ts | --lan]  follow the buoy observer (AGX_OBSERVER_LOGS) and, when its dir exists, the dock mapper (AGX_DOCK_LATEST);
#                                      --ts / --lan = tailnet or pool-switch addresses (default: NET_PROFILE in ../network.env)
#   scripts/run_agx.sh --demo          no camera: a synthetic buoy 8 m ahead at 5 Hz (the link test, L3)
#   scripts/run_agx.sh --replay CSV    replay a logged buoy_observations.csv
#   scripts/run_agx.sh --no-dock       buoys only, even if the dock mapper dir exists
#   scripts/run_agx.sh --no-cam        no camera stream (window cam: cv/stream, MJPEG on CAM_PORT, default on since v0.1.24)
#   scripts/run_agx.sh --cam-source S  what the camera page at http://AGX_HOST:8090/ shows:
#        dir  (default) the buoy / dock observers' preview JPEGs - the page stays BLANK until an observer runs
#             (window cli: buoys/run.sh --headless); then ~1-2 frames/s with the detections drawn on them
#        zed  the ZED itself, 5 frames/s - holds the camera, so no observer can run at the same time
#        test a moving test pattern (no camera; the default with --sim)
#   Just the camera, nothing else:  cv/stream/run.sh --source zed     (Ctrl-C stops it)
#   scripts/run_agx.sh --sim [--world FILE]   SIM CV instead of cv_publish (ROADMAP WP1): optiprimex.sim.fake_perception sees the
#                                      world file (default profiles/bench/world_pool.yaml; world_por.yaml = the PoR course) from the
#                                      boat's pose (local/usv/mm/state on the NX broker) and publishes the same perception/* topics
#                                      (health source: sim); optiprimex.sim.fake_dock runs the Task 3 dock lights; sim/truth goes to
#                                      the hub (OCS_HOST:HUB_PORT) for map.html. Use with the fake FCU on the NX: run_usv.sh --fcu fake
# Addresses and paths from ../network.env (BOAT_HOST, BOAT_PORT, AGX_OBSERVER_LOGS, AGX_DOCK_LATEST); command line wins.
# The observers themselves are started from cv/: buoys/run.sh --headless, docking/run.sh --headless (their own tmux/shell).
# Window host (v0.1.43): the AGX's host-health beat (CPU / GPU, thermal zones, throttling, disk, 1 Hz) -> local/usv/host/agx on the boat broker.
# Windows: cv = cv_publish (its log, nothing to type there) - in --sim: cv = fake_perception, dock = fake_dock; cli = a shell in cv/ with the venv, for the observers and the
# self-tests (cv/buoys/run.sh --headless, cv/docking/run.sh --headless, cv/cannon/run.sh --dry-run ...).
# Then: tmux attach -t agx   (Ctrl-b n/p switch windows, Ctrl-b d detach)   stop: scripts/stop_agx.sh
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"   # for --help after cd
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BENCH="$KIT"                                                # the package + .venv live at the root since v0.1.19
cd "$KIT"
SESSION="${AGX_SESSION:-agx}"
RUN_DIR="$KIT/logs/agx/run_$(date +%Y%m%d_%H%M%S)"
# ---- network addresses: ../network.env + the profile overlay network.ts.env | network.lan.env (--ts / --lan, NET_PROFILE); the command line wins
for _a in "$@"; do case "$_a" in --ts) NET_PROFILE=ts ;; --lan) NET_PROFILE=lan ;; esac; done   # the profile before the addresses load
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT"
BOAT_HOST="${BOAT_HOST:-192.168.1.218}"; BOAT_PORT="${BOAT_PORT:-1884}"
CV="$(cd "$KIT/cv" && pwd)"
OBS="${AGX_OBSERVER_LOGS:-$CV/buoys/runs}"
DOCK="${AGX_DOCK_LATEST:-$CV/docking/runs/latest}"
WITH_DOCK=1; MODE=(); SIM=0; WORLD="profiles/bench/world_pool.yaml"; WITH_CAM=1; CAM_SOURCE=""
while [ $# -gt 0 ]; do
  case "$1" in
    --demo) MODE=(--demo); shift ;;
    --replay) MODE=(--replay "$2"); shift 2 ;;
    --no-dock) WITH_DOCK=0; shift ;;
    --sim) SIM=1; shift ;;
    --no-cam) WITH_CAM=0; shift ;;
    --cam-source) CAM_SOURCE="$2"; shift 2 ;;
    --world) WORLD="$2"; shift 2 ;;
    --ts|--lan) shift ;;                                       # handled by the pre-scan above
    -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
if [ -z "${PY:-}" ]; then
  if [ -x "$KIT/.venv/bin/python" ]; then PY="$KIT/.venv/bin/python"; else PY="$(command -v python3)"; fi
fi
"$PY" -c "import paho.mqtt" 2>/dev/null || { echo "$PY lacks paho-mqtt - run scripts/setup_agx.sh first" >&2; exit 1; }
if [ $SIM -eq 1 ]; then
  "$PY" -c "import yaml, jsonschema" 2>/dev/null || { echo "$PY lacks pyyaml / jsonschema (the sim imports optiprimex) - run scripts/setup_agx.sh again" >&2; exit 1; }
  [ -f "$WORLD" ] || { echo "world file not found: $WORLD" >&2; exit 1; }
fi
command -v tmux >/dev/null || { echo "tmux not found: sudo apt install tmux" >&2; exit 1; }
mkdir -p "$RUN_DIR"; ln -sfn "$RUN_DIR" "$KIT/logs/agx/latest"
ARGS=(--mqtt "$BOAT_HOST:$BOAT_PORT" --output "$OBS")
if [ $WITH_DOCK -eq 1 ] && [ -d "$DOCK" ]; then ARGS+=(--dock-output "$DOCK"); fi
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n cv -c "$KIT"
if [ $SIM -eq 1 ]; then
  HUB_HOST="${OCS_HOST:-127.0.0.1}"; HUB_PORT="${HUB_PORT:-1882}"
  tmux send-keys -t "$SESSION:cv" "PYTHONPATH=$KIT $PY -m optiprimex.sim.fake_perception --world $WORLD --host $BOAT_HOST --port $BOAT_PORT --hub-host $HUB_HOST --hub-port $HUB_PORT --source sim 2>&1 | tee $RUN_DIR/fake_perception.log; echo '[exited]'" C-m
  tmux new-window -t "$SESSION:" -n dock -c "$KIT"
  tmux send-keys -t "$SESSION:dock" "PYTHONPATH=$KIT $PY -m optiprimex.sim.fake_dock --world $WORLD --host $BOAT_HOST --port $BOAT_PORT --auto on 2>&1 | tee $RUN_DIR/fake_dock.log; echo '[exited]'" C-m
else
  tmux send-keys -t "$SESSION:cv" "$PY $CV/publish/cv_publish.py ${ARGS[*]} ${MODE[*]:-} 2>&1 | tee $RUN_DIR/cv_publish.log; echo '[exited]'" C-m
fi
# the camera stream (v0.1.24, ROADMAP WP5): its own window, independent of the detectors; the sim has no camera -> test pattern
if [ $WITH_CAM -eq 1 ]; then
  [ -z "$CAM_SOURCE" ] && { [ $SIM -eq 1 ] && CAM_SOURCE=test || CAM_SOURCE=dir; }
  tmux new-window -t "$SESSION:" -n cam -c "$CV"
  tmux send-keys -t "$SESSION:cam" "CAM_PORT=${CAM_PORT:-8090} $CV/stream/run.sh --source $CAM_SOURCE 2>&1 | tee $RUN_DIR/cam_stream.log; echo '[exited]'" C-m
fi
CAMHINT=""
[ "$CAM_SOURCE" = "dir" ] && CAMHINT=" - blank until an observer runs (window cli: buoys/run.sh --headless); the camera itself: scripts/stop_agx.sh; scripts/run_agx.sh --cam-source zed"
[ "$CAM_SOURCE" = "zed" ] && CAMHINT=" - the ZED itself, 5 Hz; it holds the camera: no observer can run beside it"
CAMTXT="$( [ $WITH_CAM -eq 1 ] && echo "camera http://${AGX_HOST:-<AGX>}:${CAM_PORT:-8090}/ (source $CAM_SOURCE)$CAMHINT" || echo "camera off")"
# v0.1.43 (OCS_V2_SPEC §11): the host-health beat, its own window (the NX bridges it out as telemetry/usv/host/agx)
# v0.1.86 (WP42, §6 item 1 - 11 Nov: the recording stopped on disk_low at 17:36): the free disk said at start, amber < 15 GB, red < 8 GB;
# the beat carries rec_gb_per_10min from the recordings folder's growth (the OCS HOSTS tab: minutes of recording left)
REC_DIR="${REC_DIR:-$HOME/Desktop/dm3/recordings}"
FREE_GB="$(df -Pk "$HOME" 2>/dev/null | awk 'NR==2 {printf "%.1f", $4/1048576}')"
if [ -n "$FREE_GB" ]; then
  if awk "BEGIN{exit !($FREE_GB < 8)}"; then echo "*** AGX DISK RED: $FREE_GB GB free (< 8 GB) - the ZED recording will stop on disk_low; clear old recordings under $REC_DIR first ***"
  elif awk "BEGIN{exit !($FREE_GB < 15)}"; then echo "AGX disk amber: $FREE_GB GB free (< 15 GB) - about an hour of ZED recording; clear old recordings under $REC_DIR"
  else echo "AGX disk: $FREE_GB GB free"; fi
fi
tmux new-window -t "$SESSION:" -n host -c "$KIT"
tmux send-keys -t "$SESSION:host" "PYTHONPATH=$KIT $PY -m optiprimex.tools.host_health --host agx --mqtt $BOAT_HOST:$BOAT_PORT --rec-dir $REC_DIR 2>&1 | tee $RUN_DIR/host.log; echo '[exited]'" C-m
# a shell to type in: the cv/ folder, the paho venv, and the commands worth trying (the observers run from here, each its own window)
tmux new-window -t "$SESSION:" -n cli -c "$CV"
ACT=""; [ -f "$KIT/.venv/bin/activate" ] && ACT="source $KIT/.venv/bin/activate; "
tmux send-keys -t "$SESSION:cli" "${ACT}clear; echo 'cv/ shell (window cli). cv_publish runs in window cv (Ctrl-b p) - its log, nothing to type there.'; echo 'try: buoys/run.sh --self-test | docking/run.sh --self-test | cannon/run.sh --self-test     (no camera, no key)'; echo '     buoys/run.sh --headless (ZED buoy observer -> buoys/runs/<stamp>/)   docking/run.sh --headless (dock observer -> docking/runs/latest/)'; echo '     cannon/run.sh --dry-run (aiming link: prints, sends nothing)   tail -f $RUN_DIR/cv_publish.log   nc -z -w2 $BOAT_HOST $BOAT_PORT && echo boat broker reachable'; echo 'an observer started here keeps this window: Ctrl-b c for another shell, Ctrl-C here stops it'" C-m
tmux select-window -t "$SESSION:cli"
if [ $SIM -eq 1 ]; then
  echo "SIM CV up in tmux session '$SESSION': fake_perception + fake_dock on $WORLD -> boat $BOAT_HOST:$BOAT_PORT, sim/truth -> hub $HUB_HOST:$HUB_PORT (network: $NET_PROFILE); $CAMTXT"
  echo "tmux attach -t $SESSION  -> window cv (fake_perception), dock (fake_dock), cli ; dock by hand: mosquitto_pub -h $BOAT_HOST -p $BOAT_PORT -t sim/dock/cmd -m '{\"cmd\": \"reset\"}' ; logs: $RUN_DIR ; stop: scripts/stop_agx.sh"
  exit 0
fi
echo "cv_publish up in tmux session '$SESSION' -> $BOAT_HOST:$BOAT_PORT (network: $NET_PROFILE)  (observer: $OBS${MODE[*]:+; mode: ${MODE[*]}})$( [ $WITH_DOCK -eq 1 ] && [ -d "$DOCK" ] && echo "; dock mapper: $DOCK" || echo "; dock mapper: off (dir absent or --no-dock)")"
echo "$CAMTXT"
echo "tmux attach -t $SESSION  -> window cli (a shell in cv/ with hints), window cv (cv_publish's log), cam ; logs: $RUN_DIR ; stop: scripts/stop_agx.sh"
