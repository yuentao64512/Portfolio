#!/usr/bin/env python3
"""Render all YOLO boxes into a compact contact sheet for human review."""

from pathlib import Path

import cv2
import numpy as np


PROJECT = Path("/home/robotx/pipe_cones")
DATASET = PROJECT / "dataset"
OUTPUT = PROJECT / "review" / "all_120_labels.jpg"
THUMB_W = 268
COLS = 10


tiles = []
for split in ("train", "val", "test"):
    for image_path in sorted((DATASET / "images" / split).glob("*.jpg")):
        image = cv2.imread(str(image_path))
        h, w = image.shape[:2]
        values = (DATASET / "labels" / split / f"{image_path.stem}.txt").read_text().split()
        _, xc, yc, bw, bh = map(float, values)
        x1 = int((xc - bw / 2) * w)
        y1 = int((yc - bh / 2) * h)
        x2 = int((xc + bw / 2) * w)
        y2 = int((yc + bh / 2) * h)
        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 5)
        scale = THUMB_W / w
        tile = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        cv2.rectangle(tile, (0, 0), (THUMB_W, 30), (0, 0, 0), -1)
        cv2.putText(
            tile,
            f"{split} {image_path.stem}",
            (4, 21),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        tiles.append(tile)

while len(tiles) % COLS:
    tiles.append(np.zeros_like(tiles[0]))
rows = [np.hstack(tiles[i : i + COLS]) for i in range(0, len(tiles), COLS)]
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(OUTPUT), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 92])
print(OUTPUT)
