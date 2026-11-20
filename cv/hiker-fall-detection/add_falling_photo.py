#!/usr/bin/env python3
"""Convert the requested falling-down photo into one pose-sequence sample."""
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from extract_user_sequences import pose


ROOT = Path(__file__).resolve().parent
IMAGE = ROOT / "reports" / "figures" / "actions" / "falling.jpg"
MODEL = Path("/home/robotx/fall_action_project/yolo11n-pose.pt")
OUT = ROOT / "data" / "falling_down_photo.npz"

image = cv2.imread(str(IMAGE))
if image is None:
    raise FileNotFoundError(IMAGE)
height, width = image.shape[:2]
result = YOLO(str(MODEL), task="pose").predict(image, imgsz=640, device="cpu", verbose=False)[0]
keypoints = pose(result, width, height)
if keypoints[:, 2].max() <= 0:
    raise RuntimeError("No person pose detected in the falling photo")

# The classifier consumes 12-frame pose windows. Repeating the still frame adds
# the requested falling appearance without adding any other action photos.
sequence = np.repeat(keypoints[None, ...], 12, axis=0)[None, ...].astype(np.float32)
OUT.parent.mkdir(parents=True, exist_ok=True)
np.savez_compressed(
    OUT,
    X=sequence,
    y=np.array(["falling_down"]),
    segment_id=np.array(["PHOTO_falling_down_000"]),
    group=np.array(["PHOTO_falling_down"]),
    source=np.array(["falling_photo"]),
)
print(f"Wrote {OUT} with pose confidence {keypoints[:, 2].max():.3f}")
