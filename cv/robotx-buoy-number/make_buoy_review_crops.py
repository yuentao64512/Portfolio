#!/usr/bin/env python3
"""Build stable buoy-crop IDs and review contact sheets from YOLO proposals."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import cv2
import numpy as np


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def read_yolo(path: Path):
    if not path.exists():
        return []
    rows = []
    for line in path.read_text(encoding="utf-8").splitlines():
        parts = line.split()
        if len(parts) < 5:
            continue
        values = [float(value) for value in parts]
        rows.append(values)
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--buoy-labels", type=Path, required=True)
    parser.add_argument("--digit-labels", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--min-conf", type=float, default=0.05)
    parser.add_argument("--marker-min-width", type=int, default=50)
    parser.add_argument("--marker-min-height", type=int, default=35)
    parser.add_argument("--sheet-mode", choices=("eligible", "ineligible", "all"),
                        default="eligible")
    args = parser.parse_args()

    crop_dir = args.output / "crops"
    sheet_dir = args.output / "sheets"
    crop_dir.mkdir(parents=True, exist_ok=True)
    sheet_dir.mkdir(parents=True, exist_ok=True)

    records = []
    candidate_id = 0
    images = sorted(p for p in args.images.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)
    for image_path in images:
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        height, width = image.shape[:2]
        digits = read_yolo(args.digit_labels / f"{image_path.stem}.txt")
        buoys = read_yolo(args.buoy_labels / f"{image_path.stem}.txt")
        for buoy in buoys:
            class_id = int(buoy[0])
            if class_id > 3:
                continue
            xc, yc, bw, bh = buoy[1:5]
            conf = buoy[5] if len(buoy) > 5 else 1.0
            if conf < args.min_conf:
                continue
            x1 = max(0, int(round((xc - bw / 2) * width)))
            y1 = max(0, int(round((yc - bh / 2) * height)))
            x2 = min(width, int(round((xc + bw / 2) * width)))
            y2 = min(height, int(round((yc + bh / 2) * height)))
            if x2 <= x1 or y2 <= y1:
                continue

            candidate_id += 1
            crop_width, crop_height = x2 - x1, y2 - y1
            pad_x = max(2, int(crop_width * 0.08))
            pad_y = max(2, int(crop_height * 0.08))
            px1, py1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
            px2, py2 = min(width, x2 + pad_x), min(height, y2 + pad_y)
            crop = image[py1:py2, px1:px2]
            crop_name = f"B{candidate_id:04d}__{image_path.stem}.jpg"
            cv2.imwrite(str(crop_dir / crop_name), crop)

            digit_hits = []
            for digit in digits:
                dclass = int(digit[0])
                dxc, dyc, dw, dh = digit[1:5]
                dconf = digit[5] if len(digit) > 5 else 1.0
                dx = dxc * width
                dy = dyc * height
                if x1 <= dx <= x2 and y1 <= dy <= y2:
                    digit_hits.append((dconf, dclass))
            digit_hits.sort(reverse=True)
            legacy_digit = str(digit_hits[0][1]) if digit_hits else ""
            legacy_conf = f"{digit_hits[0][0]:.6f}" if digit_hits else ""
            marker_eligible = crop_width >= args.marker_min_width and crop_height >= args.marker_min_height
            records.append({
                "id": f"B{candidate_id:04d}",
                "image": image_path.name,
                "crop": crop_name,
                "x1": x1,
                "y1": y1,
                "x2": x2,
                "y2": y2,
                "proposal_conf": f"{conf:.6f}",
                "pixel_width": crop_width,
                "pixel_height": crop_height,
                "marker_eligible": int(marker_eligible),
                "legacy_digit": legacy_digit,
                "legacy_conf": legacy_conf,
                "manual_label": "",
                "review_status": "UNREVIEWED",
            })

    fieldnames = list(records[0]) if records else []
    with (args.output / "candidates.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(records)

    if args.sheet_mode == "eligible":
        eligible = [record for record in records if record["marker_eligible"]]
    elif args.sheet_mode == "ineligible":
        eligible = [record for record in records if not record["marker_eligible"]]
    else:
        eligible = records
    tile_w, tile_h, label_h, columns, per_sheet = 240, 190, 46, 5, 25
    for start in range(0, len(eligible), per_sheet):
        subset = eligible[start:start + per_sheet]
        rows = math.ceil(len(subset) / columns)
        sheet = np.full((rows * (tile_h + label_h), columns * tile_w, 3), 245, np.uint8)
        for offset, record in enumerate(subset):
            crop = cv2.imread(str(crop_dir / record["crop"]))
            if crop is None:
                continue
            h, w = crop.shape[:2]
            scale = min((tile_w - 8) / w, (tile_h - 8) / h)
            resized = cv2.resize(crop, (max(1, int(w * scale)), max(1, int(h * scale))))
            row, column = divmod(offset, columns)
            x0, y0 = column * tile_w, row * (tile_h + label_h)
            x = x0 + (tile_w - resized.shape[1]) // 2
            y = y0 + (tile_h - resized.shape[0]) // 2
            sheet[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
            cv2.putText(sheet, f"{record['id']}  {record['image']}", (x0 + 4, y0 + tile_h + 17),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (20, 20, 20), 1, cv2.LINE_AA)
            prediction = record["legacy_digit"] or "?"
            confidence = record["legacy_conf"][:4] if record["legacy_conf"] else ""
            cv2.putText(sheet, f"old={prediction} {confidence}  {record['pixel_width']}x{record['pixel_height']}",
                        (x0 + 4, y0 + tile_h + 36), cv2.FONT_HERSHEY_SIMPLEX,
                        .42, (40, 40, 180), 1, cv2.LINE_AA)
        end = start + len(subset)
        cv2.imwrite(str(sheet_dir / f"markers_{start + 1:03d}_{end:03d}.jpg"), sheet)

    print(f"whole-buoy candidates={len(records)}")
    print(f"marker-review crops={len(eligible)}")
    print(f"sheets={math.ceil(len(eligible) / per_sheet) if eligible else 0}")


if __name__ == "__main__":
    main()
