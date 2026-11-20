#!/usr/bin/env bash
# cv/buoys_multi: OUR multi-buoy observer (v0.1.30) - every buoy of a frame with a colour, the students' measuring functions
# imported read-only from cv/buoys/; output into cv/buoys/runs/<stamp>/ where cv_publish already looks.
#   usage: buoys_multi/run.sh [--headless] [--max-buoys 6] [--min-share 0.45] [--sat-min 0.35] [--frames N] [--mavlink EP] [...]
#          buoys_multi/run.sh --headless --record ~/Desktop/dm3/recordings --label pool_t1 [--max-min 10]   + the ZED recording
#                                             (v0.1.40; scripts/agx_record.sh start LABEL does this in tmux, the OCS page's card)
#          buoys_multi/run.sh --headless --svo DIR/<session>/cam.svo2   replay a recording (output: beside it, replay/<stamp>/)
#          buoys_multi/run.sh --headless --detector model-a [--colour auto|model|hsv]   the colleague's Model A (v0.1.78.18,
#                                             cv/buoys/two_stage/; ~/buoy_robotx/runtime/models/ read-only, or CV_MODEL_A=<file>);
#                                             --colour hsv = our colour vote for every buoy (the backup)
#          buoys_multi/run.sh --headless --detector boundary-v14 [--obstacle-classes obstacle[,dock]|none]   v0.1.87: the
#                                             colleague's boundary-v14 (red / green / blue / unlit boundary buoys + obstacles -> the
#                                             boat's map; ~/buoy_robotx/runtime/boundary_v14_20261111/models/ read-only, or CV_MODEL_V14=<file>)
#          buoys_multi/run.sh --headless --detector six-class   v0.1.87: their 11 Nov six-class model (red / green / blue / buoy)
#          buoys_multi/run.sh --self-test     the colour classifier and the payload, no camera
#          buoys_multi/run.sh -h | --help     this text (the observer's options: run it with --help after the flags)
# One program owns the ZED: stop buoys/run.sh (or any other camera user) first.
set -euo pipefail
case "${1:-}" in -h|--help) sed -n '2,/^set -/{/^set -/!p;}' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/../_env.sh"
PY="$(cv_python)"
if [ "${1:-}" = "--self-test" ]; then exec "$PY" "$HERE/self_test.py"; fi
WB="$HERE/../buoys/models/best"; W="$WB.pt"                    # v0.1.93.2 (Dmitrii 13 Nov: "switch to engine"): the AGX's TensorRT engine of
[ "$(uname -m)" = aarch64 ] && [ -f "$WB.engine" ] && [ "${CV_DEFAULT_PT:-0}" != 1 ] && W="$WB.engine"   # best.pt (FP16, static 768) when built; CV_DEFAULT_PT=1 = the .pt
for x in "$@"; do if [ "$x" = "model-a" ]; then                  # v0.1.78.18: --detector model-a = the colleague's Model A, read-only
  MA="$HOME/buoy_robotx/runtime/models/model_a_best"; [ "$(uname -m)" = aarch64 ] && [ -f "$MA.engine" ] && D="$MA.engine" || D="$MA.pt"
  W="${CV_MODEL_A:-$D}"; fi; done
for x in "$@"; do if [ "$x" = "boundary-v14" ]; then             # v0.1.87: the colleague's boundary-v14, read-only (the .engine on the AGX)
  MV="$HOME/buoy_robotx/runtime/boundary_v14_20261111/models/boundary_v14_best"; [ "$(uname -m)" = aarch64 ] && [ -f "$MV.engine" ] && D="$MV.engine" || D="$MV.pt"
  W="${CV_MODEL_V14:-$D}"; fi; done
for x in "$@"; do if [ "$x" = "six-class" ]; then                # v0.1.87: the colleague's 11 Nov six-class model, read-only (no engine yet: the .pt)
  MS="$HOME/buoy_robotx/runtime/six_class_20261111/models/robotx_six_class_yolo11m_1280_best"; [ "$(uname -m)" = aarch64 ] && [ -f "$MS.engine" ] && D="$MS.engine" || D="$MS.pt"
  W="${CV_MODEL_SIX:-$D}"; fi; done
[ -f "$W" ] || { echo "no $W - run cv/sync_models.sh first (Model A: CV_MODEL_A=<file>, boundary-v14: CV_MODEL_V14=<file>, six-class: CV_MODEL_SIX=<file>)" >&2; exit 1; }
mkdir -p "$HERE/../buoys/runs"
echo "[cv/buoys_multi] $PY  weights $W  output buoys/runs/" >&2
OUT=(--output "$HERE/../buoys/runs")
for x in "$@"; do case "$x" in --svo|--output) OUT=() ;; esac; done      # a replay writes beside its file, never where cv_publish looks
exec "$PY" "$HERE/multi_buoy_observer.py" --weights "$W" ${OUT[@]+"${OUT[@]}"} "$@"
