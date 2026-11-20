#!/usr/bin/env bash
set -euo pipefail

cd /home/robotx/fall_action_project

input="test_videos/generated_hospital/hospital_three_person_bed_fall_occlusion.mp4"
output="outputs/hospital_three_person_prediction.mp4"
csv="outputs/hospital_three_person_prediction.csv"

.venv/bin/python src/run_hospital_multi_video_demo.py \
  --video "$input" \
  --roi-config configs/synthetic_ward_context.json \
  --output "$output" \
  --csv "$csv" \
  --lying-alert-sec 1.0 \
  --descent-threshold 0.06 \
  --headless

ffmpeg -nostdin -hide_banner -loglevel error -y -i "$output" \
  -c:v libx264 -preset fast -crf 20 -pix_fmt yuv420p \
  outputs/hospital_three_person_prediction_h264.mp4

ffmpeg -nostdin -hide_banner -loglevel error -y -i "$input" \
  -c:v libx264 -preset fast -crf 20 -pix_fmt yuv420p \
  test_videos/generated_hospital/hospital_three_person_bed_fall_occlusion_h264.mp4

mkdir -p outputs/hospital_three_person_frames
while read -r timestamp name; do
  ffmpeg -nostdin -hide_banner -loglevel error -y -ss "$timestamp" \
    -i outputs/hospital_three_person_prediction_h264.mp4 -frames:v 1 \
    "outputs/hospital_three_person_frames/${name}.jpg"
done <<'EOF'
2.0 01_walk
3.2 02_buckle
4.0 03_descend
5.4 04_alert
10.0 05_nurse_enter
11.5 06_overlap
14.5 07_after
EOF

.venv/bin/python - "$csv" <<'PY'
import csv
import sys

rows = list(csv.DictReader(open(sys.argv[1], encoding="utf-8")))
ids = sorted({row["track_id"] for row in rows}, key=int)
print(f"rows={len(rows)} track_ids={ids}")
for track_id in ids:
    selected = [row for row in rows if row["track_id"] == track_id]
    alerts = [row for row in selected if row["alert"] == "True"]
    descents = [row for row in selected if row["full_body_descent"] == "True"]
    print(
        f"ID {track_id}: rows={len(selected)} alerts={len(alerts)} "
        f"full_body_descent={len(descents)} "
        f"first_alert={alerts[0]['time_sec'] if alerts else 'none'} "
        f"locations={sorted({row['location'] for row in selected})}"
    )
PY

sha256sum \
  outputs/hospital_three_person_prediction_h264.mp4 \
  test_videos/generated_hospital/hospital_three_person_bed_fall_occlusion_h264.mp4

ls -lh \
  outputs/hospital_three_person_prediction_h264.mp4 \
  outputs/hospital_three_person_prediction.csv \
  test_videos/generated_hospital/hospital_three_person_bed_fall_occlusion_h264.mp4 \
  outputs/hospital_three_person_frames/*.jpg
