#!/usr/bin/env bash
# Stop cv_publish or the sim CV (fake_perception + fake_dock) (tmux session 'agx'): SIGINT so it publishes presence down; Ctrl-C into the cli window too (an observer
# started there gets its clean stop), then kill the session.
# usage: scripts/stop_agx.sh                 -h | --help prints this
set -u
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
SESSION="${AGX_SESSION:-agx}"
# v0.1.40: the ZED recorder (its own tmux session agxrec) first - a clean Ctrl-C closes its SVO
if tmux has-session -t "${AGX_REC_SESSION:-agxrec}" 2>/dev/null; then "$(dirname "$0")/agx_record.sh" stop | head -1; fi
# v0.1.60: the CAMERA tab's frame recorder (tmux agxframes) - its session.json closed cleanly
if tmux has-session -t "${AGX_FRAMES_SESSION:-agxframes}" 2>/dev/null; then "$(dirname "$0")/agx_frames.sh" stop >/dev/null; fi
if tmux has-session -t "$SESSION" 2>/dev/null; then
  tmux send-keys -t "$SESSION:cv" C-c 2>/dev/null || true
  for w in $(tmux list-windows -t "$SESSION" -F '#{window_name}' 2>/dev/null | grep -v '^cv$'); do tmux send-keys -t "$SESSION:$w" C-c 2>/dev/null || true; done
  sleep 1.5
  tmux kill-session -t "$SESSION" 2>/dev/null || true
  echo "cv_publish stopped"
else
  echo "no tmux session '$SESSION'"
fi
sleep 0.5
procs="$(pgrep -fl 'cv_publish|actual_buoy_observer|dock_observer|cannon_sender|fake_perception|fake_dock|cam_stream|host_health' | grep -vE "^($$|$PPID) " || true)"   # not this script nor the shell that typed it
if [ -z "$procs" ]; then echo "down: no cv_publish, buoy/dock observer, cannon_sender, sim (fake_perception, fake_dock), cam_stream or host_health process left"
else echo "STILL UP:"; echo "$procs"; echo "  -> pkill -f 'cv_publis[h]'; pkill -f 'actual_buoy_observe[r]'; pkill -f 'dock_observe[r]'; pkill -f 'cannon_sende[r]'; pkill -f 'fake_perceptio[n]'; pkill -f 'fake_doc[k]'; pkill -f 'cam_strea[m]'; pkill -f 'host_healt[h]'"; fi
