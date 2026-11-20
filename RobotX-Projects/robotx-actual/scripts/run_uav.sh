#!/usr/bin/env bash
# The UAV windows (v0.1.64, WP14 D4/D14 of docs/OVERNIGHT_2026-11-07_UAV.md) - added to a running tmux session by run_sim.sh / run_bench.sh /
# run_ocs.sh with --uav; nothing else in the stack knows whether they run (ocs_app keys on `declare uav`, off by default).
#
#   scripts/run_uav.sh --session S --ocs-config FILE --run-dir DIR [--sim [--scenario NAME]]
#     uav     our adapter (op-uav-adapter): the hub <-> the UAV team's §5 wire (UDP: hello 2 s, robocommand, telemetry, reports);
#             nothing it sends commands the drone
#   --sim (the simulator / bench): the UAV team's own programs, UNMODIFIED, from the read-only snapshot uav/ (pymavlink from the venv):
#     copter  uav/sim-tests/fake_copter.py on udp 127.0.0.1:UAV_COPTER_PORT (default 14581); --scenario: quiet (default; no scripted
#             Mission Planner write, FENCE_ALT_MAX 20) | mpfence_alt20 | mpalt_alt20 | mpfence | mpsilent | mpsilentfix (his five)
#     uavlst  uav/geofence-tools/geofence_udp_listener.py --listen-port UAV_LISTEN_PORT (14600) --vehicle-id UAV1; its state file
#             in DIR/uav/ (never in the snapshot); stopped with SIGINT (C-c) so it restores the FCU's fence params (his invariant 12)
#   real (no --sim): the adapter only, to UAV_HOST:UAV_LISTEN_PORT (network*.env names; the values are never printed into a record)
#   UAV_BRIDGE_PORT (14605): the adapter's UDP port - the listener sends its telemetry to the address the hello came from
#   v0.1.65: then the UAV window (scripts/run_uav_window.sh, tmux uavwin, 8099; --sim: on 127.0.0.1); --no-window = without it
#   v0.1.66 (--sim): fakeuav = sim/fake_uav.py - the UAV's buoy survey after our RunStart (uav/perception/buoys on the hub, the relay path);
#     FAKE_UAV_ARGS="--noise-m 1.5 --no-complete ..." passes options
set -uo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION=""; OCS_CFG=""; RUN_DIR=""; SIM=0; SCEN="quiet"; WIN=1
while [ $# -gt 0 ]; do
  case "$1" in
    -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;;
    --session) SESSION="$2"; shift 2 ;;
    --ocs-config) OCS_CFG="$2"; shift 2 ;;
    --run-dir) RUN_DIR="$2"; shift 2 ;;
    --sim) SIM=1; shift ;;
    --scenario) SCEN="$2"; shift 2 ;;
    --no-window) WIN=0; shift ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
[ -n "$SESSION" ] && [ -n "$OCS_CFG" ] && [ -n "$RUN_DIR" ] || { echo "run_uav.sh --session S --ocs-config FILE --run-dir DIR [--sim]" >&2; exit 2; }
tmux has-session -t "$SESSION" 2>/dev/null || { echo "no tmux session $SESSION" >&2; exit 2; }
PY="$KIT/.venv/bin/python"
LPORT="${UAV_LISTEN_PORT:-14600}"; BPORT="${UAV_BRIDGE_PORT:-14605}"; CPORT="${UAV_COPTER_PORT:-14581}"
UD="$RUN_DIR/uav"; mkdir -p "$UD"
if [ $SIM -eq 1 ]; then
  case "$SCEN" in quiet) FC_SCEN=quiet_alt20 ;; mpfence_alt20|mpalt_alt20|mpfence|mpsilent|mpsilentfix) FC_SCEN="$SCEN" ;;
    *) echo "--scenario quiet | mpfence_alt20 | mpalt_alt20 | mpfence | mpsilent | mpsilentfix" >&2; exit 2 ;; esac
  UHOST=127.0.0.1; LINK=fake
  printf '%s\n' "trap '' INT; cd '$KIT/uav/sim-tests' && PYTHONDONTWRITEBYTECODE=1 '$PY' -u fake_copter.py $CPORT $FC_SCEN 2>&1 | tee -i '$UD/fake_copter.log'; echo '[exited]'" > "$UD/copter.sh"   # the FCU outlives the listener's C-c (its restore needs it)
  tmux new-window -d -t "$SESSION:" -n copter -c "$UD"
  tmux send-keys -t "$SESSION:copter" "bash copter.sh" C-m        # a file: a long line overflows the tty's 1 KB input buffer
  sleep 0.8
  printf '%s\n' "cd '$UD' && PYTHONDONTWRITEBYTECODE=1 '$PY' -u '$KIT/uav/geofence-tools/geofence_udp_listener.py' --connection udp:127.0.0.1:$CPORT --listen-port $LPORT --vehicle-id UAV1 --preset-fence-file '$KIT/uav/geofence-tools/preset_fence_example.csv' --notify '' --robocommand-state '$UD/robocommand_state.json' 2>&1 | tee -i '$UD/listener.log'; echo '[exited]'" > "$UD/listener.sh"
  tmux new-window -d -t "$SESSION:" -n uavlst -c "$UD"
  tmux send-keys -t "$SESSION:uavlst" "bash listener.sh" C-m
  sleep 1.0
  printf '%s\n' "cd '$KIT' && '$PY' -m optiprimex.sim.fake_uav --hub 127.0.0.1:${HUB_PORT:-1882} ${FAKE_UAV_ARGS:-} 2>&1 | tee -i '$UD/fake_uav.log'; echo '[exited]'" > "$UD/fake_uav.sh"
  tmux new-window -d -t "$SESSION:" -n fakeuav -c "$UD"                 # v0.1.66 (D23): the buoy survey
  tmux send-keys -t "$SESSION:fakeuav" ". ./fake_uav.sh" C-m
else
  UHOST="${UAV_HOST:-}"; LINK=real
  [ -n "$UHOST" ] || { echo "UAV_HOST is not set (network.lan.env): the adapter needs the Orin's address" >&2; exit 2; }
fi
printf '%s\n' "cd '$KIT' && UAV_HOST='$UHOST' '$PY' -m optiprimex.uav.adapter --config '$OCS_CFG' --uav-port $LPORT --bridge-port $BPORT --fcu-link $LINK 2>&1 | tee -i '$UD/adapter.log'; echo '[exited]'" > "$UD/adapter.sh"
tmux new-window -d -t "$SESSION:" -n uav -c "$UD"
tmux send-keys -t "$SESSION:uav" ". ./adapter.sh" C-m   # v0.1.78.11: sourced, not `bash x.sh` - the window's own shell runs the program, so the launcher sees it (not a shell = died)
if [ $WIN -eq 1 ]; then                              # v0.1.65 (D16): the UAV window on this Mac
  if [ $SIM -eq 1 ]; then "$KIT/scripts/run_uav_window.sh" start --bind 127.0.0.1; else "$KIT/scripts/run_uav_window.sh" start; fi
fi
echo "uav: adapter (window uav, udp $BPORT)$( [ $SIM -eq 1 ] && echo " + his fake copter ($FC_SCEN, udp $CPORT) + his listener (udp $LPORT) from uav/" || echo " -> the Orin's listener (UAV_HOST, udp $LPORT)" ); logs $UD"
