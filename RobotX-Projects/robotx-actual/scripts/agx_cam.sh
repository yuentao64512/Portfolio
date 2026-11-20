#!/usr/bin/env bash
# Runs ON THE AGX ORIN: start / stop only the camera stream (cv/stream, MJPEG on CAM_PORT) as window cam of tmux session 'agx'
# (v0.1.28.1; the OCS page's CAMERA toggle calls it over ssh). The rest of run_agx.sh's session is left alone.
#
# usage: scripts/agx_cam.sh start [--source auto|dir|zed|test]   (re)start the stream. auto (default, v0.1.28.2) = zed when no
#                                                           observer runs, else dir; dir = the observers' preview frames (blank
#                                                           until one runs); zed = the ZED itself (it holds the camera: stop the
#                                                           stream before starting an observer); test = a pattern
#        scripts/agx_cam.sh stop                            stop the stream (Ctrl-C, then the window goes)
#        scripts/agx_cam.sh status                          is it up (the python running cv/stream/cam_stream.py)
#        -h | --help prints this
set -uo pipefail
case "${1:-}" in ""|-h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CV="$KIT/cv"
SESSION="${AGX_SESSION:-agx}"
DO="$1"; shift
SOURCE=auto
while [ $# -gt 0 ]; do
  case "$1" in
    --source) SOURCE="$2"; shift 2 ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
case "$SOURCE" in auto|dir|zed|test) ;; *) echo "--source auto|dir|zed|test" >&2; exit 2 ;; esac
# the stream = a python running cv/stream/cam_stream.py (v0.1.28.2: not any command line that mentions it, e.g. a caller's ssh)
up() { pgrep -if 'pytho[n][0-9.]* .*stream/cam_stream\.py' >/dev/null; }
observers() { pgrep -if 'pytho[n][0-9.]* .*(actual_buoy_observer|multi_buoy_observer|dock_observer|buoy_camera_sender|run_docking)' >/dev/null; }   # v0.1.82: + our multi_buoy_observer (it was missing: auto picked zed while ours held the camera = no stream)   # what may hold the ZED (our cv/ or the students' own copies)
has_win() { tmux has-session -t "$SESSION" 2>/dev/null && tmux list-windows -t "$SESSION" -F '#W' | grep -qx cam; }

case "$DO" in
  status) up && echo "camera stream up (port ${CAM_PORT:-8090})" || echo "camera stream down"; exit 0 ;;
  stop|start)
    if has_win; then tmux send-keys -t "$SESSION:cam" C-c 2>/dev/null; sleep 1.2; tmux kill-window -t "$SESSION:cam" 2>/dev/null; fi
    if [ "$DO" = stop ]; then
      sleep 0.3; up && { echo "STILL UP: $(pgrep -ifa 'pytho[n][0-9.]* .*stream/cam_stream\.py')  -> pkill -f 'stream/cam_strea[m]'"; exit 1; }
      echo "camera stream stopped"; exit 0
    fi ;;
  *) echo "start | stop | status" >&2; exit 2 ;;
esac
if [ "$SOURCE" = auto ]; then
  if observers; then SOURCE=dir; echo "auto: an observer runs -> dir (its preview frames; the ZED is theirs)"
  else SOURCE=zed; echo "auto: no observer runs -> zed (the camera itself; stop the stream before starting an observer)"; fi
fi
command -v tmux >/dev/null || { echo "tmux not found: sudo apt install tmux" >&2; exit 1; }
RUN_DIR="$KIT/logs/agx/latest"; [ -d "$RUN_DIR" ] || { RUN_DIR="$KIT/logs/agx"; mkdir -p "$RUN_DIR"; }
if tmux has-session -t "$SESSION" 2>/dev/null; then tmux new-window -d -t "$SESSION:" -n cam -c "$CV"
else tmux new-session -d -s "$SESSION" -n cam -c "$CV"; fi
tmux send-keys -t "$SESSION:cam" "CAM_PORT=${CAM_PORT:-8090} $CV/stream/run.sh --source $SOURCE 2>&1 | tee -a $RUN_DIR/cam_stream.log; echo '[exited]'" C-m
sleep 2
up && echo "camera stream started (source $SOURCE, port ${CAM_PORT:-8090}; tmux $SESSION:cam)" || { echo "camera stream did not start: tail $RUN_DIR/cam_stream.log"; exit 1; }
