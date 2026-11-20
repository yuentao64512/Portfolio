#!/usr/bin/env python3
"""Software-only dependency and CUDA verification for actual_buoy."""

from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl
import tensorrt as trt
import torch
import torchvision
import ultralytics
from pymavlink import mavutil
from ultralytics import YOLO


def main() -> None:
    _ = mavutil  # Import is the MAVLink dependency check.
    print(f"numpy={np.__version__}")
    print(f"opencv={cv2.__version__}")
    print(f"torch={torch.__version__}")
    print(f"torchvision={torchvision.__version__}")
    print(f"ultralytics={ultralytics.__version__}")
    print(f"zed_sdk={sl.Camera().get_sdk_version()}")
    print(f"tensorrt={trt.__version__}")
    print(f"cuda_available={torch.cuda.is_available()}")
    if not torch.cuda.is_available():
        raise RuntimeError("PyTorch cannot access CUDA")
    print(f"gpu={torch.cuda.get_device_name(0)}")

    root = Path(__file__).resolve().parent
    model = YOLO(root / "models" / "best.pt")                      # snapshot: models/ (cv/sync_models.sh), not the students' folder
    print(f"model_task={model.task}")
    print(f"inference_device={next(model.model.parameters()).device}")
    image = root / "runs" / "latest_sender_preview.jpg"
    if image.exists():
        results = model.predict(image, device=0, verbose=False)
        print(f"detections={len(results[0].boxes) if results[0].boxes is not None else 0}")
    else:
        print("detections=skipped (no runs/latest_sender_preview.jpg yet)")
    print("software_check=PASS")


if __name__ == "__main__":
    main()
