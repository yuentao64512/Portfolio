#!/usr/bin/env python3
"""Create numbered contact sheets for visual annotation QA."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--per-sheet", type=int, default=20)
    args = parser.parse_args()
    files = sorted(p for p in args.images.iterdir()
                   if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    args.output.mkdir(parents=True, exist_ok=True)
    tile_w, tile_h, label_h, cols = 320, 180, 28, 4
    for start in range(0, len(files), args.per_sheet):
        subset = files[start:start + args.per_sheet]
        rows = math.ceil(len(subset) / cols)
        sheet = np.full((rows * (tile_h + label_h), cols * tile_w, 3), 245, np.uint8)
        for offset, path in enumerate(subset):
            image = cv2.imread(str(path))
            if image is None:
                continue
            h, w = image.shape[:2]
            scale = min(tile_w / w, tile_h / h)
            resized = cv2.resize(image, (max(1, int(w * scale)), max(1, int(h * scale))))
            row, col = divmod(offset, cols)
            y0, x0 = row * (tile_h + label_h), col * tile_w
            y = y0 + (tile_h - resized.shape[0]) // 2
            x = x0 + (tile_w - resized.shape[1]) // 2
            sheet[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
            cv2.putText(sheet, f"{start + offset + 1:03d}  {path.name}",
                        (x0 + 5, y0 + tile_h + 20), cv2.FONT_HERSHEY_SIMPLEX,
                        .55, (20, 20, 20), 1, cv2.LINE_AA)
        cv2.imwrite(str(args.output / f"contact_{start + 1:03d}_{start + len(subset):03d}.jpg"), sheet)
    print(f"{len(files)} images -> {math.ceil(len(files) / args.per_sheet)} sheets")


if __name__ == "__main__":
    main()
