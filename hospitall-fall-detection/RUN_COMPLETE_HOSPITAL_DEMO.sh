#!/usr/bin/env bash
set -euo pipefail

cd /home/robotx/fall_action_project

mode="${1:-existing}"
if [[ "$mode" == "rerun" ]]; then
  ./RUN_STITCHED_HOSPITAL_TEST.sh headless
  ./RUN_MULTI_PERSON_HOSPITAL_TEST.sh
elif [[ "$mode" != "existing" ]]; then
  echo "Usage: $0 [existing|rerun]" >&2
  exit 2
fi

first="outputs/hospital_all_scenarios_prediction_h264.mp4"
second="outputs/hospital_three_person_prediction_h264.mp4"
output="outputs/hospital_complete_demo_reel_h264.mp4"

for input in "$first" "$second"; do
  if [[ ! -s "$input" ]]; then
    echo "Missing input: $input" >&2
    echo "Run $0 rerun to rebuild all predictions." >&2
    exit 1
  fi
done

ffmpeg -nostdin -hide_banner -loglevel error -y \
  -i "$first" -i "$second" \
  -filter_complex \
  "[0:v]fps=15,scale=1280:720,setsar=1,setpts=PTS-STARTPTS[first];\
   [1:v]fps=15,scale=1280:720,setsar=1,setpts=PTS-STARTPTS[second];\
   [first][second]concat=n=2:v=1:a=0[video]" \
  -map "[video]" \
  -c:v libx264 -preset fast -crf 20 -pix_fmt yuv420p -movflags +faststart \
  "$output"

.venv/bin/python - "$output" <<'PY'
import cv2
import sys

path = sys.argv[1]
cap = cv2.VideoCapture(path)
if not cap.isOpened():
    raise SystemExit(f"Could not verify {path}")
fps = cap.get(cv2.CAP_PROP_FPS)
frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
cap.release()
print(
    f"Verified: {path} | {width}x{height} | {fps:.2f} FPS | "
    f"{frames} frames | {frames / fps:.2f} seconds"
)
PY

sha256sum "$output"
ls -lh "$output"
echo "Complete annotated demo: /home/robotx/fall_action_project/$output"
