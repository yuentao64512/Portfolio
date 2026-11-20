#!/usr/bin/env bash
# scripts/agx_observer.sh - OUR buoy observer on the AGX (v0.1.82, Dmitrii: "add start directly observer to the OCS page"):
# cv/buoys_multi/run.sh --headless (buoys + the ZED guard + the 3-8 m obstacle clusters; output cv/buoys/runs where cv_publish looks)
# as window `obs` of tmux `agx` (stop_agx.sh stops it with the rest). The camera stream then shows the observer's preview frames
# (source dir) - a stream holding the ZED itself (source zed) is restarted on dir first; any other ZED holder = refused.
#   usage: scripts/agx_observer.sh start [--detector default|model-a|boundary-v14|six-class] [--no-record] [--pairs-s N] [--led-burst N]
#          [--depth-mode neural|neural_light|ultra] [--resolution hd2k|hd1080|hd720] | stop | status [--json]
# v0.1.84.4 (Dmitrii: "record also at least 1 depth image along its RGB counterpart if I check the box"): --pairs-s N = while
# recording also <session>/pairs/rgb_N.jpg + depth_N.png (uint16 mm) + depth_N_vis.jpg, the first at once, then every N s; status
# --json says "pairs" (how many) and "pairs_dir".
# v0.1.83.2 (Dmitrii: "a clear button to start recording video, if already recording - confirm"): status says whether the ZED is
# really being recorded - `observer: up (buoys_multi) · RECORDING <dir> · <MB> MB · written <s> s ago` (or `· not recording`);
# --json = one line {"observer": up|down, "recording": bool, "dir", "mb", "age_s", "free_gb"} for the OCS backend (/recording).
# v0.1.83 (Dmitrii: logging on by default): start RECORDS the ZED (--record ~/Desktop/dm3/recordings --label obs: cam.svo2 = video + the
# ZED IMU, frames.jsonl, session.json; stops by itself below 5 GB free, detecting goes on); --no-record = detect only.
# Called by the launcher's observer ▶ / ■ and the OCS page's OBSERVER button (SYSTEM action `observer`), over ssh.
set -uo pipefail
case "${1:-}" in ""|-h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SESSION="${AGX_SESSION:-agx}"
DO="$1"; shift
# v0.1.93.2 (Dmitrii 13 Nov: "bake them in"): the measured fast camera by default - neural_light + hd1080 + the engine: 6.7 fps vs 4.2
# (hd2k / neural / .pt, nothing in view); 1.75 vs 1.25 fps with buoys. AGX_DEPTH_MODE / AGX_RESOLUTION or the flags override.
# v0.1.94 (Dmitrii 14 Nov: "lets go back to hd2k + neural!"): on the course hd1080 + neural_light ranged 6-17 % of the near boxes (13 Nov pm)
# vs hd2k + neural 92-98 % (12 / 13 Nov mornings) at 2.5-3.8 fps in a run; the sim: 49 % -> 99 % success on the camera alone.
DET=default; REC=1; JSON=0; PAIRS=0; BURST=0; DMODE="${AGX_DEPTH_MODE:-neural}"; RES="${AGX_RESOLUTION:-hd2k}"
while [ $# -gt 0 ]; do
  case "$1" in
    --json) JSON=1; shift ;;
    --pairs-s) PAIRS="$2"; shift 2 ;;
    --detector) DET="$2"; shift 2 ;;
    --no-record) REC=0; shift ;;
    --led-burst) BURST="$2"; shift 2 ;;            # v0.1.92: light-only grabs between full frames (the blink needs >= 5 fps)
    --depth-mode) DMODE="$2"; shift 2 ;;           # v0.1.92: neural | neural_light | ultra (the speed trials)
    --resolution) RES="$2"; shift 2 ;;             # v0.1.92: hd2k | hd1080 | hd720
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
case "$DET" in default|model-a|boundary-v14|six-class) ;; *) echo "--detector default|model-a|boundary-v14|six-class" >&2; exit 2 ;; esac
case "$PAIRS" in ''|*[!0-9.]*) echo "--pairs-s N (seconds)" >&2; exit 2 ;; esac
PAT='multi_buoy_observe[r]|actual_buoy_observe[r]'
up() { pgrep -f "$PAT" >/dev/null; }
which_obs() { pgrep -fa "$PAT" | grep -q multi_buoy && echo "buoys_multi" || echo "the students' actual_buoy_observer"; }
stream_up() { pgrep -if 'pytho[n][0-9.]* .*stream/cam_stream\.py' >/dev/null; }
RECDIR="$HOME/Desktop/dm3/recordings"
# rec_state -> R_ON (1 = the observer runs with --record and its svo2 was written < 10 s ago), R_DIR, R_MB, R_AGE, R_FREE
rec_state() {
  R_ON=0; R_DIR=""; R_MB=0; R_AGE=-1; R_PAIRS=0
  R_FREE=$(df -Pk "$HOME" 2>/dev/null | awk 'NR==2{printf "%.1f", $4/1048576}')
  pgrep -fa "$PAT" | grep -q -- '--record' || return 0
  R_DIR=$(ls -td "$RECDIR"/*_obs 2>/dev/null | head -1)
  [ -n "$R_DIR" ] || return 0
  R_MB=$(du -sm "$R_DIR" 2>/dev/null | awk '{print $1}')
  R_PAIRS=$(ls "$R_DIR"/pairs/rgb_*.jpg 2>/dev/null | wc -l | tr -d ' ')
  local f="$R_DIR/cam.svo2"; [ -f "$f" ] || f=$(ls -t "$R_DIR"/* 2>/dev/null | head -1)
  [ -n "$f" ] && R_AGE=$(( $(date +%s) - $(stat -c %Y "$f" 2>/dev/null || stat -f %m "$f") ))
  [ "$R_AGE" -ge 0 ] && [ "$R_AGE" -lt 10 ] && R_ON=1
  return 0
}
case "$DO" in
  status)
    rec_state
    if [ "$JSON" = 1 ]; then
      printf '{"observer": "%s", "recording": %s, "dir": "%s", "mb": %s, "age_s": %s, "free_gb": %s, "pairs": %s, "pairs_dir": "%s"}\n' \
        "$(up && echo up || echo down)" "$([ "$R_ON" = 1 ] && echo true || echo false)" "$R_DIR" "${R_MB:-0}" "$R_AGE" "${R_FREE:-0}" \
        "${R_PAIRS:-0}" "$([ -n "$R_DIR" ] && echo "$R_DIR/pairs")"
      exit 0
    fi
    if up; then
      if [ "$R_ON" = 1 ]; then echo "observer: up ($(which_obs)) · RECORDING $(basename "$R_DIR") · ${R_MB} MB · written ${R_AGE} s ago · ${R_FREE} GB free"
      elif [ -n "$R_DIR" ]; then echo "observer: up ($(which_obs)) · recording STALLED $(basename "$R_DIR") · ${R_MB} MB · last write ${R_AGE} s ago"
      else echo "observer: up ($(which_obs)) · not recording"; fi
    else echo "observer: down"; fi
    exit 0 ;;
  stop)
    tmux send-keys -t "$SESSION:obs" C-c 2>/dev/null; sleep 2; tmux kill-window -t "$SESSION:obs" 2>/dev/null
    if up; then echo "observer: still up ($(which_obs), not started from here - scripts/stop_agx.sh stops every observer)"; else echo "observer: down"; fi
    exit 0 ;;
  start) ;;
  *) echo "start | stop | status" >&2; exit 2 ;;
esac
# v0.1.87: start with another detector than the running one = switch (stop ours, start the chosen one; ~10 s without detections)
running_det() { local d; d=$(pgrep -fa 'multi_buoy_observe[r]' | grep -o -- '--detector [a-z0-9-]*' | head -1 | awk '{print $2}'); [ -z "$d" ] || [ "$d" = mask ] && d=default; echo "$d"; }
if up && pgrep -fa "$PAT" | grep -q multi_buoy && [ "$(running_det)" != "$DET" ]; then
  echo "observer: switching $(running_det) -> $DET"
  tmux send-keys -t "$SESSION:obs" C-c 2>/dev/null; sleep 2; tmux kill-window -t "$SESSION:obs" 2>/dev/null
  for i in 1 2 3 4 5; do up || break; sleep 1; done
  up && { echo "refused: the running observer was not started from here (scripts/stop_agx.sh stops every observer)"; exit 7; }
fi
if up; then echo "observer: up (already running: $(which_obs), detector $(running_det))"; exit 0; fi
tmux has-session -t "$SESSION" 2>/dev/null || { echo "refused: no $SESSION session - launch perception on the AGX first (run_agx.sh: cv_publish)"; exit 4; }
h=$(fuser /dev/video0 2>/dev/null | tr -s ' ')
if [ -n "$h" ]; then
  if pgrep -if 'pytho[n][0-9.]* .*stream/cam_stream\.py' | tr '\n' ' ' | grep -qw "$(echo $h | awk '{print $1}')"; then
    echo "the camera stream holds the ZED (source zed): stopping it, the stream comes back on the observer's frames"
    bash "$KIT/scripts/agx_cam.sh" stop >/dev/null 2>&1; sleep 1
  else
    echo "refused: the ZED is held by pid$h (a recorder or another program) - stop it first"; exit 5
  fi
fi
mkdir -p "$KIT/logs/agx"
DARG=""; [ "$DET" != default ] && DARG=" --detector $DET"
[ "$REC" = 1 ] && { mkdir -p "$HOME/Desktop/dm3/recordings"; DARG="$DARG --record $HOME/Desktop/dm3/recordings --label obs"; }
[ "$REC" = 1 ] && [ "$PAIRS" != 0 ] && DARG="$DARG --pairs-s $PAIRS"
case "$BURST" in ''|*[!0-9]*) echo "--led-burst N (frames)" >&2; exit 2 ;; esac
[ "$BURST" != 0 ] && DARG="$DARG --led-burst $BURST"
case "$DMODE" in ''|neural|neural_light|ultra) ;; *) echo "--depth-mode neural|neural_light|ultra" >&2; exit 2 ;; esac
case "$RES" in ''|hd2k|hd1080|hd720) ;; *) echo "--resolution hd2k|hd1080|hd720" >&2; exit 2 ;; esac
[ -n "$DMODE" ] && DARG="$DARG --depth-mode $DMODE"
[ -n "$RES" ] && DARG="$DARG --resolution $RES"
tmux new-window -d -t "$SESSION:" -n obs -c "$KIT/cv" "buoys_multi/run.sh --headless$DARG 2>&1 | tee $KIT/logs/agx/observer_$(date +%Y%m%d_%H%M%S).log; echo '[exited]'; sleep 30"
for i in 1 2 3 4 5 6 7 8 9 10; do sleep 1; up && break; done
up || { echo "observer: not up after 10 s - tmux attach -t $SESSION (window obs)"; exit 6; }
stream_up || bash "$KIT/scripts/agx_cam.sh" start --source dir >/dev/null 2>&1
echo "observer: up (buoys_multi, detector $DET$([ "$REC" = 1 ] && echo ', RECORDING ~/Desktop/dm3/recordings')$([ "$REC" = 1 ] && [ "$PAIRS" != 0 ] && echo " + RGB/depth pairs every ${PAIRS} s")); tmux $SESSION:obs$(stream_up && echo '; camera stream on its frames'))"
