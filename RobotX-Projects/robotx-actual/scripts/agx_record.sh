#!/usr/bin/env bash
# Runs ON THE AGX ORIN: the ZED recorder (v0.1.40, ROADMAP WP9b) = OUR buoy observer (cv/buoys_multi) with --record, in tmux
# session 'agxrec' window rec (its own session: run_agx.sh's kill-session does not cut a clip; stop_agx.sh - and so the page's
# AGX restart / STOP EVERYTHING - stops it cleanly first). It detects
# as usual (cv_publish follows it) and records DIR/<stamp>_<LABEL>/ cam.svo2 + frames.jsonl + session.json. The OCS page's
# RECORDER card (SETTINGS) calls it over ssh.
#
# usage: scripts/agx_record.sh start LABEL [--max-min N] [--dir DIR] [--no-pixhawk] [--replace] [-- observer options]
#            LABEL       letters, digits, _ and - (e.g. day1_t1_gate, practice_dock_am); the folder is <stamp>_<LABEL>
#            --max-min N stop recording after N minutes (default 10; 0 = until stop); detecting goes on afterwards
#            --dir DIR   where the sessions go (default AGX_REC_DIR, else ~/Desktop/dm3/recordings - outside the kit tree)
#            --no-pixhawk  a camera-only bench clip (no telemetry in frames.jsonl)
#            --replace   one program owns the ZED: Ctrl-C the observer / ZED stream holding it first (else: refused)
#            -- ...      passed to the observer (e.g. -- --max-depth 35 --model ../cv/models/NAME); also REC_ARGS
#        scripts/agx_record.sh stop       Ctrl-C the recorder: the SVO is closed, session.json says why (perception from it ends too:
#                                         restart cv/buoys_multi/run.sh --headless for detection without recording)
#        scripts/agx_record.sh status     recorder up / down, the newest session (label, frames, MB, why it stopped), free disk
#        -h | --help prints this
# The recording stops by itself at --max-min or below 5 GB free (the AGX has ~26 GB); status says so. Bring clips home from the
# Mac: scripts/agx_pull_clips.sh. Replay one: cv/buoys_multi/run.sh --headless --svo DIR/<session>/cam.svo2
set -uo pipefail
case "${1:-}" in ""|-h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CV="$KIT/cv"
SESSION="${AGX_REC_SESSION:-agxrec}"
REC_DIR="${AGX_REC_DIR:-$HOME/Desktop/dm3/recordings}"
MIN_FREE_GB="${REC_MIN_FREE_GB:-5}"
PY3="$(command -v python3 || echo python3)"
DO="$1"; shift
LABEL=""; MAXMIN=10; NOPIX=0; REPLACE=0; EXTRA=()
if [ "$DO" = start ]; then
  [ $# -gt 0 ] && case "$1" in -*) ;; *) LABEL="$1"; shift ;; esac
fi
while [ $# -gt 0 ]; do
  case "$1" in
    --max-min) MAXMIN="${2:-}"; shift 2 || { echo "--max-min N" >&2; exit 2; } ;;
    --dir) REC_DIR="${2:-}"; shift 2 || { echo "--dir DIR" >&2; exit 2; } ;;
    --no-pixhawk) NOPIX=1; shift ;;
    --replace) REPLACE=1; shift ;;
    --) shift; EXTRA=("$@"); break ;;
    *) echo "unknown option $1" >&2; exit 2 ;;
  esac
done
# the recorder = a python running multi_buoy_observer.py with --record (not a caller's ssh line that mentions it)
REC_PAT='pytho[n][0-9.]* .*multi_buoy_observer\.py.* --record'
HOLD_PAT='pytho[n][0-9.]* .*(actual_buoy_observer|multi_buoy_observer|dock_observer|buoy_camera_sender|run_docking)|pytho[n][0-9.]* .*stream/cam_stream\.py.*--source zed'
up() { pgrep -f "$REC_PAT" >/dev/null; }
has_win() { tmux has-session -t "$SESSION" 2>/dev/null && tmux list-windows -t "$SESSION" -F '#W' | grep -qx rec; }
status() {
  if up; then echo "recorder UP (pid $(pgrep -f "$REC_PAT" | tr '\n' ' ')- tmux $SESSION:rec)"; else echo "recorder down"; fi
  "$PY3" "$CV/buoys_multi/rec_core.py" status "$REC_DIR"
}
case "$DO" in
  status) status; exit 0 ;;
  stop)
    if has_win; then tmux send-keys -t "$SESSION:rec" C-c 2>/dev/null; fi
    if up && ! has_win; then pkill -INT -f "$REC_PAT"; fi            # started by hand elsewhere
    for _ in $(seq 1 40); do up || break; sleep 0.25; done          # the SDK closes the SVO (up to ~10 s)
    if up; then echo "STILL UP: $(pgrep -fa "$REC_PAT")  -> pkill -INT -f 'multi_buoy_observe[r]'"; exit 1; fi
    has_win && tmux kill-window -t "$SESSION:rec" 2>/dev/null
    echo "recorder stopped"; status; exit 0 ;;
  start) ;;
  *) echo "start LABEL | stop | status" >&2; exit 2 ;;
esac
[[ "$LABEL" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$ ]] || { echo "start LABEL: letters, digits, _ and - (1-40, no spaces), e.g. day1_t1_gate (got '${LABEL}')" >&2; exit 2; }
[[ "$MAXMIN" =~ ^[0-9]+([.][0-9]+)?$ ]] || { echo "--max-min N: minutes, a number >= 0 (got '$MAXMIN')" >&2; exit 2; }
if up; then echo "already recording: $(pgrep -fa "$REC_PAT" | head -1) - scripts/agx_record.sh stop first"; status; exit 1; fi
mkdir -p "$REC_DIR" || { echo "cannot create $REC_DIR" >&2; exit 1; }
FREE_KB="$(df -Pk "$REC_DIR" | awk 'NR==2 {print $4}')"
if [ -n "$FREE_KB" ] && [ "$FREE_KB" -lt $((MIN_FREE_GB * 1000000)) ]; then
  echo "NOT STARTED: disk low - $((FREE_KB / 1000000)) GB free < $MIN_FREE_GB GB on $REC_DIR (bring clips home: agx_pull_clips.sh on the Mac, then free space)"; exit 1
fi
if pgrep -f "$HOLD_PAT" >/dev/null; then
  if [ $REPLACE -eq 0 ]; then
    echo "REFUSED: the ZED is held by: $(pgrep -fa "$HOLD_PAT" | cut -c1-160 | tr '\n' ';') - stop it, or start --replace (Ctrl-C to it first)"; exit 1
  fi
  echo "--replace: Ctrl-C to what holds the ZED"; pkill -INT -f "$HOLD_PAT"
  for _ in $(seq 1 40); do pgrep -f "$HOLD_PAT" >/dev/null || break; sleep 0.25; done
  pgrep -f "$HOLD_PAT" >/dev/null && { echo "the ZED is still held: $(pgrep -fa "$HOLD_PAT" | head -1)"; exit 1; }
fi
command -v tmux >/dev/null || { echo "tmux not found: sudo apt install tmux" >&2; exit 1; }
LOGD="$KIT/logs/agx"; mkdir -p "$LOGD"
ARGS=(--headless --record "$REC_DIR" --label "$LABEL" --max-min "$MAXMIN")
[ $NOPIX -eq 1 ] && ARGS+=(--allow-no-pixhawk)
# shellcheck disable=SC2206
[ -n "${REC_ARGS:-}" ] && ARGS+=(${REC_ARGS})
ARGS+=(${EXTRA[@]+"${EXTRA[@]}"})
LINE="$CV/buoys_multi/run.sh$(printf ' %q' "${ARGS[@]}") 2>&1 | tee -a $LOGD/recorder.log; echo '[exited]'"
if tmux has-session -t "$SESSION" 2>/dev/null; then has_win && tmux kill-window -t "$SESSION:rec"; tmux new-window -d -t "$SESSION:" -n rec -c "$CV"
else tmux new-session -d -s "$SESSION" -n rec -c "$CV"; fi
tmux send-keys -t "$SESSION:rec" "$LINE" C-m
for _ in $(seq 1 20); do up && break; sleep 0.25; done
if up; then
  echo "recorder started: label $LABEL, max $MAXMIN min, into $REC_DIR (tmux $SESSION:rec; log $LOGD/recorder.log) - the model and the ZED take ~10-20 s: status shows RECORDING once the clip runs"
else
  echo "recorder did not start: tail $LOGD/recorder.log"; tail -5 "$LOGD/recorder.log" 2>/dev/null; exit 1
fi
