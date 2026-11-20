#!/usr/bin/env python3
"""Run the trained air-purifier segmenter on an image, video, folder, or camera."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


def main() -> int:
    root = Path(__file__).parent
    parser = argparse.ArgumentParser()
    parser.add_argument("source", help="Image/video/folder path, URL, or camera number such as 0")
    parser.add_argument(
        "--model",
        default=str(root / "air_purifier_yolo11n_seg.pt"),
    )
    parser.add_argument(
        "--confidence", type=float, default=0.80,
        help="Detection threshold; 0.80 is conservative against lookalike appliances",
    )
    parser.add_argument("--show", action="store_true", help="Open a live display window")
    parser.add_argument(
        "--cutouts",
        action="store_true",
        help="Also save each purifier as a transparent-background PNG (still images/folders)",
    )
    args = parser.parse_args()
    source = int(args.source) if args.source.isdigit() else args.source
    results = YOLO(args.model).predict(
        source, conf=args.confidence, device=0, save=True, show=args.show
    )
    if args.cutouts:
        output = root / "runs" / "cutouts"
        output.mkdir(parents=True, exist_ok=True)
        for result in results:
            if result.masks is None:
                continue
            image = result.orig_img
            height, width = image.shape[:2]
            stem = Path(result.path).stem
            for index, mask_tensor in enumerate(result.masks.data):
                mask = mask_tensor.detach().cpu().numpy()
                mask = cv2.resize(mask, (width, height), interpolation=cv2.INTER_LINEAR)
                alpha = np.where(mask >= 0.5, 255, 0).astype(np.uint8)
                rgba = cv2.cvtColor(image, cv2.COLOR_BGR2BGRA)
                rgba[:, :, 3] = alpha
                x1, y1, x2, y2 = result.boxes.xyxy[index].round().int().cpu().tolist()
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(width, x2), min(height, y2)
                cv2.imwrite(str(output / f"{stem}_{index + 1}.png"), rgba[y1:y2, x1:x2])
        print(f"Transparent cutouts saved to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
