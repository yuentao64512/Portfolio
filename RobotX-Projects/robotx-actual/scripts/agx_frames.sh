#!/usr/bin/env bash
# Runs ON THE AGX ORIN: the CAMERA tab's frame recorder (v0.1.60, WP10m) = optiprimex/tools/frame_rec.py in tmux session 'agxframes'
# window rec: JPEG frames + frames.jsonl from cam_stream's /snapshot.jpg (any source: the detectors' previews, the ZED, the test
# pattern) into AGX_REC_DIR (default ~/Desktop/dm3/recordings, outside the kit) / <date> / <HHMMSS>_<LABEL>/, with the detections
# per frame from the boat broker (local/usv/perception/frame_dets). The OCS page's CAMERA tab calls it over ssh.
#
# usage: scripts/agx_frames.sh start LABEL [--hz N] [--max-min N] [--depth]    (letters, digits, _ and - ; max-min 0 = until stop)
#        scripts/agx_frames.sh stop | status | sessions
# stop: scripts/agx_frames.sh stop (scripts/stop_agx.sh does it too)
set -uo pipefail
case "${1:-}" in ""|-h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
KIT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
. "$KIT/scripts/_netenv.sh"; load_network_env "$KIT"
BOAT_HOST="${BOAT_HOST:-127.0.0.1}"; BOAT_PORT="${BOAT_PORT:-1884}"
SESSION="${AGX_FRAMES_SESSION:-agxframes}"
PY="$KIT/.venv/bin/python"; [ -x "$PY" ] || PY=python3
FR=(env PYTHONPATH="$KIT" "$PY" -m optiprimex.tools.frame_rec)
DO="$1"; shift
case "$DO" in
  status)   "${FR[@]}" status --root agx_rec ;;
  sessions) "${FR[@]}" sessions --root agx_rec ;;
  stop)
    if tmux has-session -t "$SESSION" 2>/dev/null; then "${FR[@]}" stop --root agx_rec; tmux kill-session -t "$SESSION" 2>/dev/null; else "${FR[@]}" status --root agx_rec; fi ;;
  start)
    LABEL="${1:-}"; shift || true
    if tmux has-session -t "$SESSION" 2>/dev/null; then echo '{"state": "recording", "why": "refused: already recording - stop it first"}'; exit 1; fi
    CMD="$(printf '%q ' "${FR[@]}" start "$LABEL" --root agx_rec --dets-mqtt "$BOAT_HOST:$BOAT_PORT" "$@")"
    tmux new-session -d -s "$SESSION" -n rec
    tmux send-keys -t "$SESSION:rec" "$CMD; echo '[exited]'" C-m
    sleep 2; "${FR[@]}" status --root agx_rec ;;
  *) echo "start LABEL | stop | status | sessions" >&2; exit 2 ;;
esac
