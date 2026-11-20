#!/usr/bin/env bash
# What of OUR stack is running on THIS host (Mac, NX or AGX)? One screen: tmux sessions, the broker/websocket ports,
# our processes. Use it after a run_*.sh (everything up?) and after a stop_*.sh (everything down?). Exit 0 = nothing of
# ours is running (clean), 1 = something is.
#   scripts/status.sh            this host
#   scripts/status.sh --watch    refresh every 2 s (Ctrl-C to leave)
#   scripts/status.sh -h | --help   this text
# v0.1.59 (A6): on the Mac it also prints each Jetson's clock offset (ssh <host> date +%s; ORIN_SSH / AGX_SSH from network*.env;
# above 5 s every command would be REJECTED stale - the fix is GUIDE §1's date line). STATUS_CLOCK=0 skips it.
set -u
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
PORTS="1882 1883 1884 9001 9002 14550 14551"
PROCS="mosquitto -c|usv_ui.py|mission_manager|fake_fcu|fake_robocommand|ocs_app|companion.logger|cv_publish|bench_tui|dock_observer|actual_buoy_observer|cannon_sender|fake_cannon_pi|host_health|uav.adapter|geofence_udp_listener|fake_copter|uav_web|fake_uav"
show() {
  echo "== $(hostname) $(date '+%H:%M:%S')"
  echo "-- tmux sessions"; tmux ls 2>/dev/null || echo "   none"
  echo "-- listening TCP ports (1882 hub, 1884 boat broker, 1883 the boat broker's UAV map listener on the NX, 9001/9002 websockets)"
  if command -v ss >/dev/null; then L="$(ss -ltn 2>/dev/null)"; else L="$(lsof -nP -iTCP -sTCP:LISTEN 2>/dev/null)"; fi
  found=0
  for p in 1882 1883 1884 9001 9002; do
    if echo "$L" | grep -qE "[:.]$p( |$)"; then echo "   $p LISTENING"; found=1; fi
  done
  [ $found -eq 0 ] && echo "   none of ours"
  echo "-- udp 14550/14551 (fake FCU / usv_ui share) and our processes"
  P="$(pgrep -fa "$PROCS" 2>/dev/null | grep -v "status.sh" | sed 's/^/   /')"
  if [ -n "$P" ]; then echo "$P"; else echo "   no optiprimex / usv_ui / cv processes"; fi
  [ -f "$(dirname "$0")/../usv_ui/.share" ] && echo "   .share file present: $(cat "$(dirname "$0")/../usv_ui/.share")"
  [ -z "$P" ] && [ $found -eq 0 ] && ! tmux ls >/dev/null 2>&1
}
clocks() {
  case "$(hostname)" in usv-pixhawk*|optiprime*agx*) return 0 ;; esac
  [ "${STATUS_CLOCK:-1}" = 0 ] && return 0
  KIT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"; . "$KIT_ROOT/scripts/_netenv.sh"; load_network_env "$KIT_ROOT" 2>/dev/null
  echo "-- clocks (Jetson minus this Mac; above 5 s: fix with GUIDE §1's date line)"
  for h in "${ORIN_SSH:-usv-pixhawk-ts}" "${AGX_SSH:-agxorin}"; do
    t="$(ssh -o ConnectTimeout=3 -o BatchMode=yes "$h" date +%s 2>/dev/null)"
    if [ -z "$t" ]; then echo "   $h: no answer"; continue; fi
    o=$(( t - $(date +%s) )); a=${o#-}
    if [ "$a" -gt 5 ]; then echo "   $h: ${o} s  OFF (> 5 s)"; else echo "   $h: ${o} s  ok"; fi
  done
}
if [ "${1:-}" = "--watch" ]; then while true; do clear; show; sleep 2; done; fi
clocks
show && { echo "CLEAN: nothing of ours is running here"; exit 0; } || { echo "RUNNING (see above)"; exit 1; }
