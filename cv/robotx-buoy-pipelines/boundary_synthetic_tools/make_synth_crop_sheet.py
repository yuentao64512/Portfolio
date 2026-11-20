#!/usr/bin/env python3
"""Create a close-up QA sheet from synthetic YOLO images."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=150)
    args = parser.parse_args()

    manifest = args.root / "synthetic_manifest.csv"
    dataset = args.root / "dataset"
    records = list(csv.DictReader(manifest.open(newline="")))[:args.limit]
    tiles: list[np.ndarray] = []
    for record in records:
        image_path = dataset / "images" / "train" / record["file"]
        label_path = dataset / "labels" / "train" / Path(record["file"]).with_suffix(".txt")
        image = cv2.imread(str(image_path))
        rows = [line.split() for line in label_path.read_text().splitlines() if line.strip()]
        if image is None or not rows:
            continue
        cls, x, y, w, h = rows[-1][:5]
        x, y, w, h = map(float, (x, y, w, h))
        ih, iw = image.shape[:2]
        x1, y1 = (x - w / 2) * iw, (y - h / 2) * ih
        x2, y2 = (x + w / 2) * iw, (y + h / 2) * ih
        pad = 0.18 * max(x2 - x1, y2 - y1)
        a, b = max(0, int(x1 - pad)), max(0, int(y1 - pad))
        c, d = min(iw, int(x2 + pad)), min(ih, int(y2 + pad))
        crop = image[b:d, a:c]
        if crop.size == 0:
            continue
        scale = min(220 / crop.shape[1], 190 / crop.shape[0])
        resized = cv2.resize(crop, (max(1, int(crop.shape[1] * scale)),
                                    max(1, int(crop.shape[0] * scale))),
                             interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_CUBIC)
        tile = np.full((230, 240, 3), 32, dtype=np.uint8)
        top = 30 + (190 - resized.shape[0]) // 2
        left = (240 - resized.shape[1]) // 2
        tile[top:top + resized.shape[0], left:left + resized.shape[1]] = resized
        title = f"{record['target_class']} {record['file'][-16:-4]}"
        cv2.putText(tile, title, (5, 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.43, (0, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)

    if not tiles:
        raise SystemExit("No synthetic crops found")
    columns = 10
    blank = np.full_like(tiles[0], 32)
    rows = []
    for start in range(0, len(tiles), columns):
        row = tiles[start:start + columns]
        row += [blank] * (columns - len(row))
        rows.append(np.hstack(row))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 94])
    print(f"wrote={args.output} crops={len(tiles)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
