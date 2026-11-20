#!/usr/bin/env bash
set -Eeuo pipefail

ROOT=/home/robotx/buoy_actual/v15_6class_20261111
RAW=/home/robotx/robotx_do_not_touch/agx_buoy_recordings_2hz
MEDIA="$ROOT/media"
VIDEO="$MEDIA/all_sessions_2hz_2208x1242.mp4"
MANIFEST="$MEDIA/all_sessions_frames.ffconcat"
TRAIN_PID_FILE="$ROOT/train_v15_r3.pid"
TRAIN_LOG="$ROOT/logs/v15_r3_from_v14best_1280_150ep_20261111.log"
RUN="$ROOT/runs/v15_r3_from_v14best_1280_150ep_20261111"
WEIGHTS="$RUN/weights/best.pt"
PREDICT_PROJECT="$ROOT/predictions"
PREDICT_NAME=v15_r3_all_sessions_2hz
STATUS="$ROOT/auto_predict_status.txt"

mkdir -p "$MEDIA" "$PREDICT_PROJECT"
echo "BUILDING_VIDEO" > "$STATUS"

if [[ ! -s "$VIDEO" ]]; then
    /usr/bin/python3 - "$RAW" "$MANIFEST" <<'PY'
from pathlib import Path
import sys

raw = Path(sys.argv[1])
manifest = Path(sys.argv[2])
sessions = ("20261111_112939", "20261111_115135", "20261111_164851")
frames = []
for session in sessions:
    images = raw / session / "images"
    frames.extend(sorted(p for p in images.iterdir()
                         if p.suffix.lower() in {".jpg", ".jpeg", ".png"}))
if not frames:
    raise SystemExit("No recording frames found")

def quote(path: Path) -> str:
    return str(path.resolve()).replace("'", "'\\''")

with manifest.open("w") as handle:
    handle.write("ffconcat version 1.0\n")
    for frame in frames:
        handle.write(f"file '{quote(frame)}'\n")
        handle.write("duration 0.5\n")
    # Repeat the final frame because concat applies a duration only when a
    # following entry exists.
    handle.write(f"file '{quote(frames[-1])}'\n")
print(f"frames={len(frames)} first={frames[0].name} last={frames[-1].name}")
PY

    TEMP="$MEDIA/.all_sessions_2hz_2208x1242.tmp.mp4"
    rm -f "$TEMP"
    /snap/bin/ffmpeg -hide_banner -loglevel warning -y \
        -f concat -safe 0 -i "$MANIFEST" \
        -vf "fps=2,format=yuv420p" -an -c:v libx264 -preset veryfast \
        -crf 20 -movflags +faststart -threads 2 "$TEMP"
    mv "$TEMP" "$VIDEO"
fi

MANIFEST_FILES=$(grep -c '^file ' "$MANIFEST")
FRAME_COUNT=$((MANIFEST_FILES - 1))
echo "VIDEO_READY path=$VIDEO frames=${FRAME_COUNT:-unknown}" > "$STATUS"

while [[ -f "$TRAIN_PID_FILE" ]] && kill -0 "$(cat "$TRAIN_PID_FILE")" 2>/dev/null; do
    echo "WAITING_FOR_TRAINING video=$VIDEO frames=${FRAME_COUNT:-unknown}" > "$STATUS"
    sleep 30
done

if [[ ! -f "$WEIGHTS" ]]; then
    echo "FAILED_NO_BEST_PT expected=$WEIGHTS" > "$STATUS"
    exit 2
fi
if ! grep -q "Results saved to" "$TRAIN_LOG"; then
    echo "FAILED_TRAINING_NOT_CONFIRMED best_exists=$WEIGHTS log=$TRAIN_LOG" > "$STATUS"
    exit 3
fi

echo "PREDICTING model=$WEIGHTS source=$VIDEO" > "$STATUS"
/home/robotx/buoy_actual/.venv/bin/yolo detect predict \
    model="$WEIGHTS" source="$VIDEO" imgsz=1280 conf=0.25 iou=0.55 \
    device=0 save=True save_txt=True save_conf=True \
    project="$PREDICT_PROJECT" name="$PREDICT_NAME" exist_ok=False

OUTPUT_DIR="$PREDICT_PROJECT/$PREDICT_NAME"
echo "COMPLETE output=$OUTPUT_DIR model=$WEIGHTS source=$VIDEO" > "$STATUS"

