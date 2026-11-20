#!/usr/bin/env bash
# cv/stream: the camera as MJPEG over HTTP for the Mac's browser (ROADMAP WP5) - http://AGX_HOST:CAM_PORT/ (CAM_PORT in
# ../network.env, default 8090). Read-only: never commands anything, never needs MQTT.
#   usage: stream/run.sh [--source dir|zed|test] [--hz 5] [--width 640] [--port P]
#     dir  (default) the newest observer preview JPEG (buoys / docking runs/) - runs beside the detectors that hold the ZED;
#          the page stays BLANK until an observer writes previews (buoys/run.sh --headless), then ~1-2 frames/s
#     zed  the ZED left image itself, 5 frames/s - only with no observer running (it holds the camera)
#     test a moving test pattern (no camera)
#   e.g.  cv/stream/run.sh --source zed        then open http://AGX_HOST:8090/ on the Mac (Ctrl-C stops it)
#   dir needs nothing but python3; zed / test need OpenCV (+ pyzed): the cv interpreter (_env.sh: CV_VENV, cv/.venv, the
#   students' venv read-only) is used for them.
#          stream/run.sh -h | --help              this text (cam_stream.py --help for the full list)
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/../_env.sh"
SRC=dir
for ((i = 1; i <= $#; i++)); do [ "${!i}" = "--source" ] && { j=$((i + 1)); SRC="${!j}"; }; done
if [ "$SRC" = "dir" ]; then PY="$(command -v python3)"; export CAM_NO_OPENCV=1; else PY="$(cv_python)"; fi
export CAM_PORT="${CAM_PORT:-8090}"
exec "$PY" "$HERE/cam_stream.py" "$@"
