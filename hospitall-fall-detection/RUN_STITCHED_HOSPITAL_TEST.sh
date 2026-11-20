#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/robotx/fall_action_project"
cd "$PROJECT_DIR"

MODE="${1:-headless}"
INPUT="test_videos/generated_hospital/hospital_all_scenarios_stitched_h264.mp4"
OUTPUT="outputs/hospital_all_scenarios_prediction.mp4"
CSV_OUTPUT="outputs/hospital_all_scenarios_prediction.csv"

DISPLAY_ARGS=(--headless)
if [[ "$MODE" == "show" ]]; then
    DISPLAY_ARGS=()
elif [[ "$MODE" != "headless" ]]; then
    echo "Usage: $0 [headless|show]" >&2
    exit 2
fi

.venv/bin/python src/run_hospital_video_demo.py \
    --video "$INPUT" \
    --roi-config configs/synthetic_ward_context.json \
    --classifier models/current_pose_rf.pkl \
    --pose-model yolo11n-pose.pt \
    --output "$OUTPUT" \
    --csv "$CSV_OUTPUT" \
    --device 0 \
    --reset-after-no-person-sec 2.0 \
    "${DISPLAY_ARGS[@]}"

ffmpeg -y -v error -i "$OUTPUT" \
    -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p -movflags +faststart \
    outputs/hospital_all_scenarios_prediction_h264.mp4

echo "Annotated video: $PROJECT_DIR/outputs/hospital_all_scenarios_prediction_h264.mp4"
echo "Frame CSV:       $PROJECT_DIR/$CSV_OUTPUT"
