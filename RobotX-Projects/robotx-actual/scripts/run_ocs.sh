#!/usr/bin/env bash
# Runs ON THE MAC (the OCS laptop): the shore side of the stack in one tmux session 'ocs' (scripts/ = the launchers, profiles/pool/ = the host profiles):
#
#   hub    mosquitto 1882 (+ws 9001; v0.1.90.12: off 1883, which is the drone's map port on the NX) on all interfaces, from profiles/pool/mosquitto/hub_pool.conf.in; the boat bridges in
#   rc     fake RoboCommand (no judge at the pool): RunStart 2 s after a valid declaration
#   ocs    ocs_app (profiles/pool/ocs_pool.yaml): command authority, heartbeats, run lifecycle R0-R6
#   log    (only with LOG=1) SQLite logger on the hub -> logs/ocs/run_<stamp>/hub_<stamp>.sqlite
#   tui    op-tui, boat feeds from the Orin's broker (BOAT_HOST): NETWORK tab shows which nodes are up - the main console
#   web    ocs_web (v0.1.28): the same in the browser, http://127.0.0.1:8096/ (OCS_WEB_PORT) - op-tui's view and commands in tabs,
#          tooltips, the mission picker, the MAP tab (map.html), and SYSTEM: restart / reboot the stacks, the FCU, the Jetsons
#   cli    a shell with the venv: ocs_cli next | ocs_cli ready | ocs_cli goto --x 10 --y 0 | op-scenario list
#
# Usage:  scripts/run_ocs.sh [--ts | --lan] [--rc-args "..."] [--judge fake|rctest|course]
#         --judge fake (default)  the fake RoboCommand (JSON bench wire) in window rc (its log), profiles/pool/ocs_pool.yaml
#         --judge rctest          RoboNation's rc-test on this Mac (scripts/run_rctest.sh first: 127.0.0.1:1885), the real protobuf wire,
#                                 profiles/course/ocs_course.yaml
#         --judge alpha|bravo|charlie|delta   v0.1.82 (WP39): the course's RoboCommand, RC_<COURSE>_HOST / _PORT from network.lan.env
#                                 (handbook 3.4.2: 192.168.65-68.2:1883); the OCS checks RxCourse's course_id and its own address against it
#         --judge demo            v0.1.82: = venue (Team Village demo, VLAN 70)
#         --judge auto            v0.1.94: the course whose network an adapter is on now (scripts/net_roles.sh status); never the venue demo (v0.1.94.2)
#         --judge venue           v0.1.78.1: the venue's rc-test - RoboNation's practice server on our team broker
#                                 VENUE_RC_HOST:VENUE_RC_PORT (network.env; its web page VENUE_RC_WEB), protobuf, profiles/course/ocs_course.yaml,
#                                 the console read-only (--monitor) - the home rc-test is --judge rctest
#         --judge course          the course RoboCommand at RC_HOST:RC_PORT (network.lan.env; e.g. 192.168.65.2:1883), protobuf,
#                                 profiles/course/ocs_course.yaml; the console read-only (--monitor)
#         The judge's console is tmux `judge` whatever the judge (v0.1.26): tmux attach -t judge  (rc-tui + the browser page
#         http://127.0.0.1:8095/); opened here for fake / course, by run_rctest.sh for rctest   (addresses from ../network.<profile>.env; BOAT_HOST=... on the command line wins)
#         --ui v1 | v2                          the page it opens: v2 (default) or the classic v1 page frozen at /v1/ (v0.1.43; LAUNCH_UI)
#         --ts | --lan                          tailnet or pool-switch addresses (default: NET_PROFILE in ../network.env)
#         MAP=0 scripts/run_ocs.sh                     do not open the OCS page in the browser at start (it still runs)
#         OCS_CONFIG_OVERRIDE=f scripts/run_ocs.sh      the OCS profile file instead (scripts/launch.sh: a copy with another heartbeat rate)
#         LOG=1 scripts/run_ocs.sh                     also record every MQTT message on the hub to SQLite (off by default: disk space)
#         --uav                                 v0.1.64 (WP14): + window uav, our adapter to the UAV team's Orin (UAV_HOST, UAV_LISTEN_PORT 14600;
#                                               our UDP UAV_BRIDGE_PORT 14605) - scripts/run_uav.sh; `ocs_cli declare uav on` declares it
# Then:   tmux attach -t ocs ; the browser: http://127.0.0.1:8096/ (the map is its MAP tab)
#         stop: scripts/stop_ocs.sh
set -euo pipefail
SELF="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"   # for --help after cd
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BENCH="$KIT"                                                # the package + .venv live at the root since v0.1.19
cd "$KIT"
SESSION="${OCS_SESSION:-ocs}"
RUN_DIR="$KIT/logs/ocs/run_$(date +%Y%m%d_%H%M%S)"
RC_ARGS="--start-delay 2 --confirm-delay 1"
JUDGE=fake
UAV=0

# ---- network addresses: ../network.env + the profile overlay network.ts.env | network.lan.env (--ts / --lan, NET_PROFILE); the command line wins
for _a in "$@"; do case "$_a" in --ts) NET_PROFILE=ts ;; --lan) NET_PROFILE=lan ;; esac; done   # the profile before the addresses load
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT"
BOAT_HOST="${BOAT_HOST:-192.168.1.218}"
BOAT_PORT="${BOAT_PORT:-1884}"
HUB_PORT="${HUB_PORT:-1882}"
HUB_WS="${HUB_WS:-9001}"
MAP="${MAP:-1}"                                                    # MAP=0: do not open the browser map at start
LOG="${LOG:-0}"                                                    # LOG=1: SQLite logger window (off by default)
while [ $# -gt 0 ]; do
  case "$1" in
    --rc-args) RC_ARGS="$RC_ARGS $2"; shift 2 ;;
    --judge) JUDGE="$2"; shift 2 ;;
    --ui) UI="$2"; shift 2 ;;                                 # v0.1.43: v1 = open the classic page (/v1/) instead of v2
    --ts|--lan) shift ;;                                       # handled by the pre-scan above
    --uav) UAV=1; shift ;;                                     # v0.1.64: the UAV adapter window
    -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$SELF" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
RC_COURSE="${RC_COURSE:-}"
if [ "$JUDGE" = auto ]; then                                   # v0.1.94 (Dmitrii 14 Nov): the course this Mac is plugged into (scripts/net_roles.py)
  JUDGE="$(/usr/bin/python3 "$KIT/scripts/net_roles.py" course | tr '[:upper:]' '[:lower:]')"
  [ -n "$JUDGE" ] || { echo "judge auto: no adapter has a course address yet - plug the course cable into either adapter (both on DHCP), wait ~10 s; scripts/net_roles.sh status" >&2; exit 2; }
  [ "$JUDGE" = venue ] && { echo "judge auto: never the venue demo (v0.1.94.2) - --judge demo for it" >&2; exit 2; }   # belt and braces
  echo "judge auto: on the $JUDGE course network"
fi
case "$JUDGE" in       # v0.1.82 (WP39): the course dropdown - RC_<COURSE>_HOST / _PORT from network.lan.env (handbook 3.4.2: 192.168.6x.2:1883)
  alpha|bravo|charlie|delta)
    RC_COURSE="$(echo "$JUDGE" | tr '[:lower:]' '[:upper:]')"; hv="RC_${RC_COURSE}_HOST"; pv="RC_${RC_COURSE}_PORT"
    RC_HOST="${!hv:-}"; RC_PORT="${!pv:-1883}"
    [ -n "$RC_HOST" ] || { echo "judge $JUDGE: $hv not set - add the course RoboCommand address to network.lan.env" >&2; exit 2; }
    JUDGE=course ;;
  demo) RC_COURSE=DEMO; JUDGE=venue ;;                         # v0.1.82: DEMO (Team Village, VLAN 70) = the venue rc-test
esac
if [ "$JUDGE" = venue ] && { [ -z "${VENUE_RC_HOST:-}" ] || [ -z "${VENUE_RC_PORT:-}" ]; }; then   # v0.1.78.1: before anything starts
  echo "judge venue: VENUE_RC_HOST / VENUE_RC_PORT not set - add the venue rc-test's team broker to network.env (VENUE_RC_WEB = its page)" >&2; exit 2
fi
PY="${PY:-$KIT/.venv/bin/python}"
[ -x "$PY" ] || { echo "no venv at $PY - make venv at the tree root" >&2; exit 1; }
MOSQ="${MOSQUITTO:-$(command -v mosquitto || true)}"
for cand in /opt/homebrew/sbin/mosquitto /usr/local/sbin/mosquitto /usr/sbin/mosquitto; do
  if [ -z "$MOSQ" ] && [ -x "$cand" ]; then MOSQ="$cand"; fi
done
[ -n "$MOSQ" ] || { echo "mosquitto not found (brew install mosquitto; do not 'brew services start' it)" >&2; exit 1; }
command -v tmux >/dev/null || { echo "tmux not found (brew install tmux)" >&2; exit 1; }
for port in $HUB_PORT $HUB_WS; do
  if lsof -nP -iTCP:$port -sTCP:LISTEN >/dev/null 2>&1; then
    echo "port $port already in use (the bench? scripts/stop_bench.sh there; a brew mosquitto? brew services stop mosquitto)" >&2; exit 1
  fi
done
mkdir -p "$RUN_DIR" /tmp/optiprimex_hub
ln -sfn "$RUN_DIR" "$KIT/logs/ocs/latest"
CONF="$RUN_DIR/hub.conf"
sed -e "s|@PERSIST@|/tmp/optiprimex_hub|" -e "s|@PORT@|$HUB_PORT|" -e "s|@WS@|$HUB_WS|" profiles/pool/mosquitto/hub_pool.conf.in > "$CONF"
MYIP="$(ipconfig getifaddr en0 2>/dev/null || hostname -I 2>/dev/null | awk '{print $1}' || echo '?')"
TSIP="$( (command -v tailscale >/dev/null && tailscale ip -4) 2>/dev/null || /Applications/Tailscale.app/Contents/MacOS/Tailscale ip -4 2>/dev/null || true)"
echo "hub broker: $HUB_PORT + ws $HUB_WS on all interfaces (this Mac: LAN $MYIP, tailnet ${TSIP:-?}; network: $NET_PROFILE, OCS_HOST=${OCS_HOST:-EMPTY}; boat at $BOAT_HOST:$BOAT_PORT)"
if [ -z "${OCS_HOST:-}" ]; then
  echo "WARNING: OCS_HOST is empty in network.env - put this Mac's address there (${TSIP:-tailscale ip -4} on the tailnet, $MYIP on the pool switch) or the Orin's bridge has nowhere to go" >&2
elif [ "$OCS_HOST" != "$MYIP" ] && [ "$OCS_HOST" != "${TSIP:-none}" ] \
     && ! { ifconfig 2>/dev/null || ip -4 addr 2>/dev/null; } | grep -qE "inet ${OCS_HOST//./\\.}[ /]"; then   # v0.1.78.5: any interface (the switch on a USB LAN, en16)
  echo "WARNING: network.env OCS_HOST=$OCS_HOST but this Mac is LAN $MYIP / tailnet ${TSIP:-?} (on no interface here) - the Orin's bridge will not find the hub" >&2
fi

tmux kill-session -t "$SESSION" 2>/dev/null || true
tmux new-session -d -s "$SESSION" -n hub -c "$BENCH"
tmux send-keys -t "$SESSION:hub" "$MOSQ -c $CONF 2>&1 | tee $RUN_DIR/hub.log; echo '[exited]'" C-m
sleep 0.8
OCS_CONFIG="$KIT/profiles/pool/ocs_pool.yaml"; RCENV=""; JUDGE_ARGS=""
case "$JUDGE" in       # v0.1.26: window rc = only the fake judge's log; the judge's console is tmux `judge` (scripts/run_judge.sh)
  fake)   tmux new-window -t "$SESSION:" -n rc -c "$BENCH"
          tmux send-keys -t "$SESSION:rc" "$PY -m optiprimex.sim.fake_robocommand --port $HUB_PORT --log $RUN_DIR/robocommand.jsonl $RC_ARGS 2>&1 | tee $RUN_DIR/rc.log; echo '[exited]'" C-m
          JUDGE_ARGS="--port $HUB_PORT" ;;
  rctest) OCS_CONFIG="$KIT/profiles/course/ocs_course.yaml"; RC_HOST=127.0.0.1; RC_PORT="${RCTEST_PORT:-1885}"; RCENV="RC_HOST=$RC_HOST RC_PORT=$RC_PORT " ;;
          # the console is opened by scripts/run_rctest.sh (--as-judge on 1885)
  course) OCS_CONFIG="$KIT/profiles/course/ocs_course.yaml"; RC_HOST="${RC_HOST:?RC_HOST not set: put the course RoboCommand address in network.lan.env}"; RC_PORT="${RC_PORT:-1883}"
          RCENV="RC_HOST=$RC_HOST RC_PORT=$RC_PORT RC_COURSE=${RC_COURSE:-AUTO} "; JUDGE_ARGS="--host $RC_HOST --port $RC_PORT --monitor"
          echo "judge course ${RC_COURSE:-AUTO}: RoboCommand at $RC_HOST:$RC_PORT" ;;
  venue)  OCS_CONFIG="$KIT/profiles/course/ocs_course.yaml"
          RC_HOST="${VENUE_RC_HOST:?VENUE_RC_HOST not set - the team broker of the venue rc-test, in network.env}"   # v0.1.78.5: no apostrophe in a :? word
          RC_PORT="${VENUE_RC_PORT:?VENUE_RC_PORT not set - the team broker port of the venue rc-test, in network.env}"   # (bash opened a quote there)
          RCENV="RC_HOST=$RC_HOST RC_PORT=$RC_PORT RC_COURSE=${RC_COURSE:-DEMO} "; JUDGE_ARGS="--host $RC_HOST --port $RC_PORT --monitor"
          echo "judge venue: rc-test at $RC_HOST:$RC_PORT${VENUE_RC_WEB:+ (its page $VENUE_RC_WEB)}" ;;
  *) echo "--judge fake | rctest | demo (= venue) | alpha | bravo | charlie | delta | course (auto: the NIC's gateway)" >&2; exit 2 ;;
esac
OCS_CONFIG="${OCS_CONFIG_OVERRIDE:-$OCS_CONFIG}"       # v0.1.38: scripts/launch.sh's generated copy (another heartbeat rate)
# v0.1.29: the page's RoboCommand peek listens where the judge talks (read-only): the hub for the fake judge, else RC_HOST:RC_PORT
if [ "$JUDGE" = fake ]; then PEEK_HOST=127.0.0.1; PEEK_PORT=$HUB_PORT; else PEEK_HOST=$RC_HOST; PEEK_PORT=$RC_PORT; fi
sleep 0.5
tmux new-window -t "$SESSION:" -n ocs -c "$BENCH"
tmux send-keys -t "$SESSION:ocs" "${RCENV}$PY -m optiprimex.ocs.ocs_app --config $OCS_CONFIG 2>&1 | tee $RUN_DIR/ocs.log; echo '[exited]'" C-m
sleep 0.5
if [ "$LOG" = "1" ]; then
  tmux new-window -t "$SESSION:" -n log -c "$BENCH"
  tmux send-keys -t "$SESSION:log" "$PY -m optiprimex.companion.logger --port $HUB_PORT --name hub --client-id logger-hub --run-dir $RUN_DIR 2>&1 | tee $RUN_DIR/loghub.log; echo '[exited]'" C-m
  LOGNOTE="SQLite log ON -> $RUN_DIR/hub_*.sqlite"
else
  LOGNOTE="SQLite log off (LOG=1 scripts/run_ocs.sh to record every message)"
fi
tmux new-window -t "$SESSION:" -n cli -c "$BENCH"
tmux send-keys -t "$SESSION:cli" "source .venv/bin/activate; echo 'try: ocs_cli next | ocs_cli ready | ocs_cli goto --x 10 --y 0 | ocs_cli manual'" C-m
tmux new-window -t "$SESSION:" -n tui -c "$BENCH"
tmux send-keys -t "$SESSION:tui" "sleep 1.5; NET_PROFILE=$NET_PROFILE $PY -m optiprimex.tools.bench_tui --port $HUB_PORT --ws-port $HUB_WS --boat-host $BOAT_HOST --boat-port $BOAT_PORT" C-m   # NET_PROFILE: the WHERE WE ARE box
OCS_WEB_PORT="${OCS_WEB_PORT:-8096}"
tmux new-window -t "$SESSION:" -n web -c "$BENCH"            # the environment is passed explicitly: the tmux server may predate it
tmux send-keys -t "$SESSION:web" "NET_PROFILE=$NET_PROFILE ORIN_SSH=${ORIN_SSH:-usv-pixhawk-ts} AGX_SSH=${AGX_SSH:-agxorin} AGX_HOST=${AGX_HOST:-} CAM_PORT=${CAM_PORT:-8090} JUDGE_WEB_PORT=${JUDGE_WEB_PORT:-8095} $PY -m optiprimex.tools.ocs_web --port $HUB_PORT --ws-port $HUB_WS --boat-host $BOAT_HOST --boat-port $BOAT_PORT --web-port $OCS_WEB_PORT --judge-host $PEEK_HOST --judge-port $PEEK_PORT --ocs-config $OCS_CONFIG 2>&1 | tee $RUN_DIR/ocs_web.log; echo '[exited]'" C-m
tmux select-window -t "$SESSION:tui"
if [ "$UAV" = "1" ]; then "$KIT/scripts/run_uav.sh" --session "$SESSION" --ocs-config "$OCS_CONFIG" --run-dir "$RUN_DIR" || echo "WARNING: the UAV adapter did not start (UAV_HOST?)" >&2; fi
if [ "$MAP" = "1" ] && [ "$(uname)" = "Darwin" ]; then
  sleep 1.5; open "http://127.0.0.1:$OCS_WEB_PORT/$([ "${UI:-${LAUNCH_UI:-v2}}" = v1 ] && echo v1/)" || true
fi
if [ -n "$JUDGE_ARGS" ]; then
  JUDGE_WEB_PORT="${JUDGE_WEB_PORT:-8095}" "$KIT/scripts/run_judge.sh" $JUDGE_ARGS --owner ocs >/dev/null
elif ! tmux has-session -t judge 2>/dev/null; then
  echo "NOTE: no judge console yet - scripts/run_rctest.sh opens it (or scripts/run_judge.sh --port ${RC_PORT:-1885} --as-judge)"
fi
echo "ocs stack up in tmux session '$SESSION' (tmux attach -t $SESSION); boat feeds from $BOAT_HOST:$BOAT_PORT; text logs: $RUN_DIR; $LOGNOTE; stop: scripts/stop_ocs.sh"
echo "OCS page: http://127.0.0.1:$OCS_WEB_PORT/ (window web; the map is its MAP tab; op-tui stays the main console)"
echo "judge console: tmux attach -t judge   (rc-tui: both wires; browser: http://127.0.0.1:${JUDGE_WEB_PORT:-8095}/)"
