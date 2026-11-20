#!/usr/bin/env bash
set -euo pipefail

ROOT="/home/robotx/buoy_actual"
PIPELINE="$ROOT/pipeline"
VENV="$ROOT/.venv"
FRAMES="$ROOT/boat_yolo/frames/images"

mkdir -p "$ROOT/boat_yolo"/{frames,labels_A,crops,labels_B,datasets,runs,logs}
mkdir -p "$FRAMES"

echo "[$(date -Is)] creating Python environment"
if [[ ! -x "$VENV/bin/python" ]]; then
  python3 -m venv "$VENV"
fi

"$VENV/bin/python" -m pip install --upgrade pip setuptools wheel
echo "[$(date -Is)] installing PyTorch CUDA 12.8"
"$VENV/bin/python" -m pip install \
  torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 \
  --index-url https://download.pytorch.org/whl/cu128
echo "[$(date -Is)] installing vision pipeline packages"
"$VENV/bin/python" -m pip install -U ultralytics opencv-python-headless pyyaml pandas

echo "[$(date -Is)] validating GPU"
"$VENV/bin/python" - <<'PY'
import json
import torch
import ultralytics

assert torch.cuda.is_available(), "PyTorch cannot see the NVIDIA GPU"
x = torch.randn((1024, 1024), device="cuda", dtype=torch.float16)
y = x @ x
print(json.dumps({
    "torch": torch.__version__,
    "cuda_build": torch.version.cuda,
    "gpu": torch.cuda.get_device_name(0),
    "capability": torch.cuda.get_device_capability(0),
    "ultralytics": ultralytics.__version__,
    "fp16_test": float(y[0, 0]),
}, indent=2))
PY

echo "[$(date -Is)] extracting 2 fps with motion filtering"
"$VENV/bin/python" "$PIPELINE/extract_frames.py" \
  --media "$ROOT" \
  --output "$FRAMES" \
  --fps 2.0 \
  --motion-threshold 1.8

echo "[$(date -Is)] setup and extraction complete"
touch "$ROOT/boat_yolo/SETUP_AND_EXTRACTION_COMPLETE"
