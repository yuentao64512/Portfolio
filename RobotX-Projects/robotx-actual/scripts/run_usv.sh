#!/usr/bin/env bash
# Runs ON THE ORIN NX: the boat side of the stack in one tmux session 'usv' (scripts/ = the launchers, profiles/pool/ = the host profiles):
#
#   boat   mosquitto 1884 (+ws 9002, + the UAV map listener 1883) from profiles/pool/mosquitto/boat_pool.conf.in; bridge to the Mac when OCS_HOST is set
#   ui    usv_ui.py: holds the Pixhawk link (UART default) and shares it on udp 14551; the operator's window
#   mm     mission manager (profiles/pool/usv_pool.yaml; --lab: profiles/pool/usv_lab.yaml) through the share - MAVLink 200/191, usv_ui is 200/190;
#          its command trace (modes, frames, stages) -> logs/usv/run_<stamp>/mm_trace.jsonl: `mode-log` reads it (GUIDE §6.2)
#   host   the host-health beat (v0.1.43): CPU, thermal zones, throttling, disk at 1 Hz -> local/usv/host/nx (bridged out as
#          telemetry/usv/host/nx; optiprimex/tools/host_health.py)
#   log    (only with LOG=1) SQLite logger on the boat broker -> logs/usv/run_<stamp>/boat_<stamp>.sqlite
#
# Usage:  scripts/run_usv.sh [--fcu fake|uart|usb|eth|<endpoint>] [--pool | --lab | --course] [--ts | --lan] [--no-ui] [--no-handover (v0.1.86: handed over by default)] [--ui-args "--log"] [--no-mm]
#   --fcu fake             no Pixhawk: the bench's kinematic fake FCU runs in a window (udp 14550) and usv_ui holds it -
#                          the whole MQTT / handshake architecture on the real hosts without touching the boat (GUIDE.md stage A)
#   --fcu uart             the real Pixhawk on /dev/ttyTHS1 @ 921600 (the default); usb = the FMU USB jumper; eth = udpin:0.0.0.0:14550
#                          (--fake-fcu = --fcu fake; without --fcu the FCU_LINK line of ../network.env decides, 'uart' when unset)
#   --pool                 the pool profile profiles/pool/usv_pool.yaml, said out loud (the default without a profile flag; v0.1.27.1)
#   --course               the competition profile profiles/course/usv_course.yaml (no GOTO task; v0.1.23)
#   --lab                  REAL Pixhawk on the bench: mission manager profile profiles/pool/usv_lab.yaml - AUTO accepted disarmed and
#                          without EKF, EKF_BAD / RC_LOST reported but never a FAULT, so the state machine can be walked with
#                          mode changes only (props off / ESC power off). Never at the pool: the default stays usv_pool.yaml
#   --free-run [--world F] v0.1.68 (WP17): with --lab and a REAL FCU only - the mission manager runs every stage against an in-process
#                          ghost plant (its start: --world F's start, else the lab profile's free_run.start); the FCU gets MODE REQUESTS
#                          ONLY (no frames / targets / velocities). Refused without --lab and with --fcu fake (exit 2). `skip` works.
#   LOG=0 scripts/run_usv.sh   no SQLite log of every MQTT message on the boat broker (v0.1.86: ON by default - LOG=1);
#                          the small per-window text logs (boat.log, mm.log, fcu.log) are always written to logs/usv/run_<stamp>/
#   --ts | --lan           which address set: ../network.ts.env (tailnet) or ../network.lan.env (pool switch); default = the
#                          NET_PROFILE line of ../network.env. The launcher prints 'network: ts (...)'. Same flag on run_ocs / run_agx.
#   ../network.env         ports, FCU_LINK, paths and NET_PROFILE; a variable on the command line wins:
#   OCS_HOST=192.168.1.x   bridge the boat broker to the hub on the Mac;  OCS_HOST= (empty) = standalone: usv_ui + mm + AGX
#   BOAT_WS=9002           websocket port for map.html (v0.1.90.12: the default everywhere; 9001 is the hub's)
#   MM_CONFIG_OVERRIDE=f   the mission manager's profile file instead (scripts/launch.sh: a copy with other rates; v0.1.38)
#   PY=...                 python with pymavlink + paho-mqtt + pyyaml + jsonschema (default: .venv, else python3)
#   FCU_LINK=fake|uart|usb|eth|<endpoint>   the same switch from ../network.env (LINK=... on the command line also works)
# Then: tmux attach -t usv  (window ui = the pool tool; Ctrl-b n/p switch windows, Ctrl-b d detach)
#       stop: scripts/stop_usv.sh
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"   # for --help after cd
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"          # the MASTER_SCRIPTS root (scripts/ is under it)
UI="$(cd "$KIT/usv_ui" && pwd)"                          # the pool tool lives there (its .share, points.json, co.json too)
BENCH="$KIT"                                                # the package + .venv live at the root since v0.1.19
cd "$KIT"
SESSION="${USV_SESSION:-usv}"
RUN_DIR="$KIT/logs/usv/run_$(date +%Y%m%d_%H%M%S)"
LOG="${LOG:-1}"                                                  # LOG=1: the SQLite logger window - v0.1.86: ON by default (§6 item 4: on 11 Nov LOG=1 never reached the NX); LOG=0 = off

# ---- network addresses: ../network.env + the profile overlay network.ts.env | network.lan.env (--ts / --lan, NET_PROFILE); the command line wins
for _a in "$@"; do case "$_a" in --ts) NET_PROFILE=ts ;; --lan) NET_PROFILE=lan ;; esac; done   # the profile before the addresses load
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT"
HANDOVER="${USV_HANDOVER:-1}"                               # v0.1.86 (WP42 §2.3b, Dmitrii: "handover by default upon firing the stack")
WITH_UI=1; WITH_MM=1; FAKE_FCU=0; UI_ARGS=""; MM_CONFIG="profiles/pool/usv_pool.yaml"; LAB=0; NPROF=0; FREE_RUN=0; FR_WORLD=""
FCU="${LINK:-${FCU_LINK:-uart}}"                             # fake | uart | usb | eth | <pymavlink endpoint>; the command line wins
while [ $# -gt 0 ]; do
  case "$1" in
    --fcu) FCU="$2"; shift 2 ;;
    --fake-fcu) FCU=fake; shift ;;
    --pool) NPROF=$((NPROF+1)); MM_CONFIG="profiles/pool/usv_pool.yaml"; shift ;;
    --lab) NPROF=$((NPROF+1)); LAB=1; MM_CONFIG="profiles/pool/usv_lab.yaml"; shift ;;
    --course) NPROF=$((NPROF+1)); MM_CONFIG="profiles/course/usv_course.yaml"; shift ;;
    --free-run) FREE_RUN=1; shift ;;                          # v0.1.68 (WP17)
    --world) FR_WORLD="$2"; shift 2 ;;
    --no-ui) WITH_UI=0; shift ;;
    --no-handover) HANDOVER=0; shift ;;                       # v0.1.86: usv_ui starts holding the FCU (as before)
    --handover) HANDOVER=1; shift ;;
    --no-mm) WITH_MM=0; shift ;;
    --ui-args) UI_ARGS="$UI_ARGS $2"; shift 2 ;;
    --ts|--lan) shift ;;                                       # handled by the pre-scan above
    -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
[ $NPROF -le 1 ] || { echo "--pool, --lab and --course pick one profile each: give one" >&2; exit 2; }
case "$FCU" in
  fake) FAKE_FCU=1; UI_ARGS="udpin:127.0.0.1:14550 $UI_ARGS" ;;
  uart) ;;                                                   # usv_ui's default link: /dev/ttyTHS1 @ 921600
  *) UI_ARGS="$FCU $UI_ARGS" ;;                              # usb | eth | an explicit endpoint, as usv_ui takes it
esac
if [ $FAKE_FCU -eq 1 ]; then
  [ $WITH_UI -eq 1 ] || { echo "--fcu fake needs usv_ui (it holds the fake link and shares it to the mission manager)" >&2; exit 1; }
fi
if [ $FREE_RUN -eq 1 ]; then                                # v0.1.68 (WP17 F1): the lab profile with a real FCU only
  [ $LAB -eq 1 ] || { echo "--free-run needs --lab (the lab profile's free_run block; never at the pool)" >&2; exit 2; }
  [ $FAKE_FCU -eq 0 ] || { echo "--free-run refuses --fcu fake: the fake plant already has a position (run_sim.sh is the simulator)" >&2; exit 2; }
  if [ -n "$FR_WORLD" ] && [ ! -f "$FR_WORLD" ] && [ ! -f "$KIT/$FR_WORLD" ]; then echo "--world $FR_WORLD: no such file" >&2; exit 2; fi
elif [ -n "$FR_WORLD" ]; then echo "--world belongs to --free-run" >&2; exit 2
fi
FR_ARGS=""; [ $FREE_RUN -eq 1 ] && FR_ARGS="--free-run${FR_WORLD:+ --world $FR_WORLD}"
if [ $LAB -eq 1 ] && [ $FAKE_FCU -eq 1 ]; then echo "note: --lab with the fake FCU changes nothing useful (the fake plant has GPS, RC and arms itself)"; fi

# ---- toolchain -------------------------------------------------------------------------------------------
if [ -z "${PY:-}" ]; then
  if [ -x "$KIT/.venv/bin/python" ]; then PY="$KIT/.venv/bin/python"; else PY="$(command -v python3)"; fi
fi
export PYTHONPATH="$KIT${PYTHONPATH:+:$PYTHONPATH}"            # optiprimex importable without pip install -e
"$PY" -c "import pymavlink, paho.mqtt, yaml, jsonschema" 2>/dev/null || {
  echo "$PY lacks pymavlink/paho-mqtt/pyyaml/jsonschema - run scripts/setup_orin.sh first" >&2; exit 1; }
MOSQ="${MOSQUITTO:-$(command -v mosquitto || true)}"
if [ -z "$MOSQ" ] && [ -x /usr/sbin/mosquitto ]; then MOSQ=/usr/sbin/mosquitto; fi
[ -n "$MOSQ" ] || { echo "mosquitto not found: sudo apt install mosquitto  (and: sudo systemctl disable --now mosquitto; this script runs its own)" >&2; exit 1; }
command -v tmux >/dev/null || { echo "tmux not found: sudo apt install tmux" >&2; exit 1; }
BOAT_WS="${BOAT_WS:-9002}"                                        # map.html websocket port (v0.1.90.12: 9002 everywhere)
BOAT_PORT="${BOAT_PORT:-1884}"
for port in $BOAT_PORT $BOAT_WS; do
  if (ss -ltn 2>/dev/null || lsof -nP -iTCP -sTCP:LISTEN 2>/dev/null) | grep -q ":$port "; then
    echo "port $port already in use (a system mosquitto? 'sudo systemctl stop mosquitto', or an old run: scripts/stop_usv.sh)" >&2; exit 1
  fi
done
if [ -f "$UI/.share" ]; then
  read -r _ SH_PID < "$UI/.share"
  if kill -0 "$SH_PID" 2>/dev/null; then echo "usv_ui / rc.sh already holds the FCU link (pid $SH_PID) - stop it first" >&2; exit 1; fi
  rm -f "$UI/.share"
fi

# ---- render the broker config ------------------------------------------------------------------------------
mkdir -p "$RUN_DIR" /tmp/optiprimex_boat
ln -sfn "$RUN_DIR" "$KIT/logs/usv/latest"
echo "$RUN_DIR" > "$KIT/logs/usv/current_run"
CONF="$RUN_DIR/boat.conf"
# v0.1.66 (WP16, D21): the direct UAV bridge block only with UAV_BROKER_HOST set and UAV_BUOY_PATH direct (the default when the host is set)
UAV_BUOY_PATH="${UAV_BUOY_PATH:-$( [ -n "${UAV_BROKER_HOST:-}" ] && echo direct || echo off )}"
if [ -n "${UAV_BROKER_HOST:-}" ] && [ "$UAV_BUOY_PATH" = direct ]; then
  UAVSED="-e s|@UAV_BROKER@|${UAV_BROKER_HOST}:${UAV_BROKER_PORT:-1883}| -e s|^#UAVBRIDGE.||"     # no space in a pattern: $UAVSED is split
else
  UAVSED="-e /^#UAVBRIDGE/d"
fi
# v0.1.90.12 (Dmitrii: "reassign Mac port, and keep drone on 1883"): the UAV team's map publisher keeps ITS DEFAULT - the boat broker
# listens for it on 1883 (sgit/uav/cv/#); our hub moved to 1882 on the Mac (HUB_PORT), 1884 = our boat broker. Any host where 1883 is
# free; BOAT_UAV_PORT=0 = off. (v0.1.90.11's 1886 reverted.)
BOAT_UAV_PORT="${BOAT_UAV_PORT:-1883}"
if [ "$BOAT_UAV_PORT" != 0 ] && [ "$BOAT_UAV_PORT" != "$BOAT_PORT" ] && ! (ss -ltn 2>/dev/null | grep -q ":$BOAT_UAV_PORT ") ; then
  UAVSED="$UAVSED -e s|@UAV_PORT@|$BOAT_UAV_PORT| -e s|^#UAVLISTEN.||"
  echo "boat broker: + listener $BOAT_UAV_PORT for the UAV team's map publisher (sgit/uav/cv/#: their --port $BOAT_UAV_PORT)"
else
  [ "$BOAT_UAV_PORT" != 0 ] && echo "boat broker: NO listener $BOAT_UAV_PORT for the UAV map publisher (the port is busy)"
  UAVSED="$UAVSED -e /^#UAVLISTEN/d"
fi
if [ -n "${OCS_HOST:-}" ]; then
  sed -e "s|@PERSIST@|/tmp/optiprimex_boat|" -e "s|@WS@|$BOAT_WS|" -e "s|@PORT@|$BOAT_PORT|" -e "s|@OCS_HOST@|$OCS_HOST|" -e "s|@HUB_PORT@|${HUB_PORT:-1882}|" -e "s|^#BRIDGE ||" $UAVSED profiles/pool/mosquitto/boat_pool.conf.in > "$CONF"
  echo "boat broker: $BOAT_PORT + ws $BOAT_WS, bridged to the hub at $OCS_HOST:${HUB_PORT:-1882}  (network.env: $NETENV)"
else
  sed -e "s|@PERSIST@|/tmp/optiprimex_boat|" -e "s|@WS@|$BOAT_WS|" -e "s|@PORT@|$BOAT_PORT|" -e "/^#BRIDGE /d" $UAVSED profiles/pool/mosquitto/boat_pool.conf.in > "$CONF"
  echo "boat broker: $BOAT_PORT + ws $BOAT_WS, standalone - OCS_HOST is empty in $NETENV (fill it with the Mac's address, 'tailscale ip -4' there) for the bridge to the hub"
fi

# ---- tmux --------------------------------------------------------------------------------------------------
tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n boat -c "$KIT"
tmux send-keys -t "$SESSION:boat" "$MOSQ -c $CONF 2>&1 | tee $RUN_DIR/boat.log; echo '[exited]'" C-m
sleep 0.8
if [ $FAKE_FCU -eq 1 ]; then
  tmux new-window -t "$SESSION:" -n fcu -c "$KIT"
  tmux send-keys -t "$SESSION:fcu" "$PY -m optiprimex.sim.fake_fcu --conn udpout:127.0.0.1:14550 --control-port 14560 --armed 2>&1 | tee $RUN_DIR/fcu.log; echo '[exited]'" C-m
  sleep 1.0
  echo "FAKE FCU (bench plant) on udp 14550 - no Pixhawk involved; usv_ui holds it on udpin:127.0.0.1:14550"
fi
if [ $WITH_UI -eq 1 ]; then
  tmux new-window -t "$SESSION:" -n ui -c "$UI"
  case " $UI_ARGS " in *" --log"*) ;; *) [ "${FCU_LOG:-1}" = 1 ] && UI_ARGS="$UI_ARGS --log run" ;; esac   # v0.1.83: the FCU log ON by default (tlog = position, IMU, attitude, heading; csv; events) - FCU_LOG=0 = off
  # v0.1.86: HANDED OVER at start - the mission manager owns the FCU after enable_auto; `take` in usv_ui gives it back (the transmitter's
  # mode channel and kill are the pilot's regardless). --no-handover / USV_HANDOVER=0 = usv_ui holds it, as before
  case " $UI_ARGS " in *" --handover"*|*" --released"*) ;; *) [ "$HANDOVER" = 1 ] && UI_ARGS="$UI_ARGS --handover" ;; esac
  tmux send-keys -t "$SESSION:ui" "NET_PROFILE=$NET_PROFILE LAUNCH_PRESET=${LAUNCH_PRESET:-} $PY usv_ui.py $UI_ARGS --log-dir $RUN_DIR; echo '[exited]'" C-m   # NET_PROFILE: the WHERE WE ARE box
  sleep 2.0
fi
if [ $WITH_MM -eq 1 ]; then
  tmux new-window -t "$SESSION:" -n mm -c "$KIT"
  MM_CONFIG="${MM_CONFIG_OVERRIDE:-$MM_CONFIG}"            # v0.1.38: scripts/launch.sh's generated copy (changed rates)
  tmux send-keys -t "$SESSION:mm" "$PY -m optiprimex.usv.mission_manager --config $MM_CONFIG $FR_ARGS --trace $RUN_DIR/mm_trace.jsonl --map-rec $RUN_DIR/maps 2>&1 | tee $RUN_DIR/mm.log; echo '[exited]'" C-m
  sleep 1.0
fi
tmux new-window -t "$SESSION:" -n host -c "$KIT"           # v0.1.43 (OCS_V2_SPEC §11): the NX's host-health beat
tmux send-keys -t "$SESSION:host" "$PY -m optiprimex.tools.host_health --host nx --mqtt 127.0.0.1:$BOAT_PORT 2>&1 | tee $RUN_DIR/host.log; echo '[exited]'" C-m
if [ "$LOG" = "1" ]; then
  tmux new-window -t "$SESSION:" -n log -c "$KIT"
  tmux send-keys -t "$SESSION:log" "$PY -m optiprimex.companion.logger --port $BOAT_PORT --name boat --client-id logger-boat --run-dir $RUN_DIR 2>&1 | tee $RUN_DIR/logboat.log; echo '[exited]'" C-m
  LOGNOTE="SQLite log ON -> $RUN_DIR/boat_*.sqlite"
else
  LOGNOTE="SQLite log off (LOG=1 scripts/run_usv.sh to record every message)"
fi
[ $WITH_UI -eq 1 ] && tmux select-window -t "$SESSION:ui"
if [ $FAKE_FCU -eq 1 ]; then FCUNOTE="FCU: FAKE (bench plant on udp 14550, no Pixhawk)"; else FCUNOTE="FCU: REAL ($FCU)"; fi
if [ $LAB -eq 1 ]; then
  echo "*** LAB PROFILE ($MM_CONFIG): AUTO accepted disarmed / without EKF, EKF_BAD + RC_LOST never a FAULT - bench only, props off, NEVER at the pool ***"
fi
if [ $FREE_RUN -eq 1 ]; then
  echo "*** FREE RUN: ghost position, no motion to the FCU (mode requests only) - bench only, props off, ESC power off, NEVER at the pool ***${FR_WORLD:+ (world $FR_WORLD)}"
fi
echo "usv stack up in tmux session '$SESSION' (tmux attach -t $SESSION); network: $NET_PROFILE (OCS_HOST ${OCS_HOST:-EMPTY}); $FCUNOTE; mission manager $MM_CONFIG; text logs: $RUN_DIR (mode-log $RUN_DIR/mm_trace.jsonl = the mode sequence per stage); $LOGNOTE; stop: scripts/stop_usv.sh"
