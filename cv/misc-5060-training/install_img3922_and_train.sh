#!/usr/bin/env bash
set -euo pipefail

ROOT=/home/robotx/buoy_actual
BASE_DATA="$ROOT/boat_yolo/datasets/model_a"
NEW_DATA="$ROOT/boat_yolo/datasets/model_a_plus_img3922_v1"
IMPORT_TAR=/home/robotx/IMG_3922_model_a_labels_v4.tar.gz
SOURCE_VIDEO=/home/robotx/IMG_3922.MOV
DATA_YAML="$ROOT/boat_yolo/model_a_img3922_v1.yaml"
START_MODEL="$ROOT/boat_yolo/runs/model_a_1280_100ep_20261109_072904/weights/best.pt"

for required in "$BASE_DATA" "$IMPORT_TAR" "$SOURCE_VIDEO" "$DATA_YAML" "$START_MODEL"; do
  if [[ ! -e "$required" ]]; then
    echo "Missing required input: $required" >&2
    exit 1
  fi
done

if [[ -e "$NEW_DATA" ]]; then
  echo "Refusing to overwrite existing dataset: $NEW_DATA" >&2
  exit 1
fi

if ps -eo cmd | grep -E '[y]olo .*train' >/dev/null; then
  echo "A YOLO training process is already running; not starting another." >&2
  ps -eo pid,etime,cmd | grep -E '[y]olo .*train' >&2
  exit 1
fi

work_dir="$(mktemp -d /tmp/img3922_import.XXXXXX)"
cleanup() {
  rm -rf -- "$work_dir"
}
trap cleanup EXIT

tar -xzf "$IMPORT_TAR" -C "$work_dir"
import_dir="$work_dir/img3922_model_a_slim_v1"

image_count="$(find "$import_dir/images" -maxdepth 1 -type f -name '*.jpg' | wc -l)"
label_count="$(find "$import_dir/labels" -maxdepth 1 -type f -name '*.txt' | wc -l)"
if [[ "$image_count" -ne 30 || "$label_count" -ne 30 ]]; then
  echo "Expected 30 images and labels, got images=$image_count labels=$label_count" >&2
  exit 1
fi

python3 - "$import_dir" <<'PY'
from pathlib import Path
import sys

root = Path(sys.argv[1])
images = {p.stem for p in (root / "images").glob("*.jpg")}
labels = {p.stem for p in (root / "labels").glob("*.txt")}
assert images == labels, "image/label stems differ"
for label in (root / "labels").glob("*.txt"):
    rows = [row.split() for row in label.read_text().splitlines() if row.strip()]
    assert len(rows) == 1, f"{label}: expected exactly one buoy"
    assert rows[0][0] == "2", f"{label}: expected class 2 buoy_no_led"
    values = [float(value) for value in rows[0][1:]]
    assert len(values) == 4 and all(0.0 < value <= 1.0 for value in values), label
print(f"validated {len(images)} paired labels")
PY

cp -a -- "$BASE_DATA" "$NEW_DATA"
cp -a -- "$import_dir/images/." "$NEW_DATA/images/train/"
cp -a -- "$import_dir/labels/." "$NEW_DATA/labels/train/"

source_dir="$ROOT/boat_yolo/incremental_sources/IMG_3922"
mkdir -p "$source_dir"
cp -a -- "$SOURCE_VIDEO" "$source_dir/"
cp -a -- "$import_dir/manifest.csv" "$source_dir/"

final_images="$(find "$NEW_DATA/images/train" -maxdepth 1 -type f | wc -l)"
final_labels="$(find "$NEW_DATA/labels/train" -maxdepth 1 -type f | wc -l)"
if [[ "$final_images" -ne 577 || "$final_labels" -ne 577 ]]; then
  echo "Unexpected final train counts: images=$final_images labels=$final_labels" >&2
  exit 1
fi

run="model_a_img3922_ft_1280_60ep_$(date +%Y%m%d_%H%M%S)"
log="$ROOT/boat_yolo/logs/${run}.log"
pid_file="$ROOT/boat_yolo/${run}.pid"

nohup "$ROOT/.venv/bin/yolo" detect train \
  model="$START_MODEL" \
  data="$DATA_YAML" \
  epochs=60 patience=15 \
  imgsz=1280 batch=2 amp=True device=0 workers=8 \
  hsv_h=0.0 hsv_s=0.5 hsv_v=0.4 \
  degrees=20.0 translate=0.10 scale=0.40 shear=2.0 perspective=0.0005 \
  fliplr=0.5 flipud=0.0 \
  mosaic=0.5 mixup=0.0 copy_paste=0.0 close_mosaic=10 \
  project="$ROOT/boat_yolo/runs" name="$run" \
  >"$log" 2>&1 </dev/null &

pid=$!
printf '%s\n' "$pid" > "$pid_file"
printf '%s\n' "$run" > "$ROOT/boat_yolo/current_model_a_img3922_run.txt"

echo "dataset=$NEW_DATA"
echo "train_images=$final_images"
echo "train_labels=$final_labels"
echo "run=$run"
echo "pid=$pid"
echo "log=$log"
