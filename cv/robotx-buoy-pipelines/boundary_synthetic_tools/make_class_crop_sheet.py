#!/usr/bin/env python3
"""Render close-up YOLO boxes for one class from one or more splits."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import cv2
import numpy as np


def image_for(dataset: Path, label: Path) -> Path | None:
    rel = label.relative_to(dataset)
    parts = list(rel.parts)
    parts[parts.index("labels")] = "images"
    stem = dataset.joinpath(*parts).with_suffix("")
    for suffix in (".jpg", ".jpeg", ".png"):
        candidate = stem.with_suffix(suffix)
        if candidate.exists():
            return candidate
    return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--class-id", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--limit", type=int, default=200)
    parser.add_argument("--start", type=int, default=0)
    parser.add_argument("--manifest", type=Path)
    args = parser.parse_args()

    tiles = []
    manifest_rows = []
    global_index = 0
    for label in sorted((args.dataset / "labels").glob("*/*.txt")):
        image_path = image_for(args.dataset, label)
        if image_path is None:
            continue
        image = cv2.imread(str(image_path.resolve()))
        if image is None:
            continue
        ih, iw = image.shape[:2]
        for row_index, raw in enumerate(label.read_text().splitlines()):
            fields = raw.split()
            if len(fields) < 5 or int(fields[0]) != args.class_id:
                continue
            review_id = global_index
            global_index += 1
            if review_id < args.start:
                continue
            x, y, w, h = map(float, fields[1:5])
            x1, y1 = (x - w / 2) * iw, (y - h / 2) * ih
            x2, y2 = (x + w / 2) * iw, (y + h / 2) * ih
            pad = 0.15 * max(x2 - x1, y2 - y1)
            a, b = max(0, int(x1 - pad)), max(0, int(y1 - pad))
            c, d = min(iw, int(x2 + pad)), min(ih, int(y2 + pad))
            crop = image[b:d, a:c]
            if crop.size == 0:
                continue
            scale = min(210 / crop.shape[1], 175 / crop.shape[0])
            resized = cv2.resize(crop, (max(1, int(crop.shape[1] * scale)),
                                        max(1, int(crop.shape[0] * scale))),
                                 interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_CUBIC)
            tile = np.full((215, 230, 3), 30, dtype=np.uint8)
            top = 28 + (175 - resized.shape[0]) // 2
            left = (230 - resized.shape[1]) // 2
            tile[top:top+resized.shape[0], left:left+resized.shape[1]] = resized
            cv2.putText(tile, f"{review_id:03d} {label.stem[:24]}", (5, 18), cv2.FONT_HERSHEY_SIMPLEX,
                        .38, (0, 255, 255), 1, cv2.LINE_AA)
            tiles.append(tile)
            manifest_rows.append({"review_id": review_id, "label": str(label),
                                  "row_index": row_index, "image": str(image_path)})
            if len(tiles) >= args.limit:
                break
        if len(tiles) >= args.limit:
            break
    if not tiles:
        raise SystemExit("No boxes found")
    columns = 10
    blank = np.full_like(tiles[0], 30)
    rows = []
    for start in range(0, len(tiles), columns):
        row = tiles[start:start+columns]
        row += [blank] * (columns-len(row))
        rows.append(np.hstack(row))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 94])
    if args.manifest:
        args.manifest.parent.mkdir(parents=True, exist_ok=True)
        with args.manifest.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=["review_id", "label", "row_index", "image"])
            writer.writeheader()
            writer.writerows(manifest_rows)
    print(f"wrote={args.output} crops={len(tiles)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
