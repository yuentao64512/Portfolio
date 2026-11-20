#!/usr/bin/env python3
"""Create an evenly sampled contact sheet from a raw image directory."""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, default=60)
    args = parser.parse_args()
    paths = sorted(path for path in args.images.iterdir()
                   if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not paths:
        raise SystemExit("No images found")
    indices = np.linspace(0, len(paths) - 1, min(args.count, len(paths)), dtype=int)
    tiles = []
    for index in indices:
        path = paths[int(index)]
        image = cv2.imread(str(path))
        if image is None:
            continue
        tile = cv2.resize(image, (384, 216), interpolation=cv2.INTER_AREA)
        cv2.rectangle(tile, (0, 0), (384, 27), (0, 0, 0), -1)
        cv2.putText(tile, f"{index:04d} {path.stem[-24:]}", (5, 19),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)
        tiles.append(tile)
    columns = 5
    blank = np.zeros_like(tiles[0])
    rows = []
    for start in range(0, len(tiles), columns):
        row = tiles[start:start + columns]
        row.extend([blank] * (columns - len(row)))
        rows.append(np.hstack(row))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 94])
    print(f"images={len(paths)} sampled={len(tiles)} output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
