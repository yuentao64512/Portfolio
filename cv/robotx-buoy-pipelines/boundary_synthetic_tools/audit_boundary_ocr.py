#!/usr/bin/env python3
"""Audit Tesseract and sign-model evidence for class-3 crops."""

from __future__ import annotations

import argparse
import csv
import subprocess
from pathlib import Path

import cv2
from ultralytics import YOLO


def image_for_label(dataset: Path, label: Path) -> Path | None:
    rel = label.relative_to(dataset)
    parts = list(rel.parts)
    parts[parts.index("labels")] = "images"
    stem = dataset.joinpath(*parts).with_suffix("")
    for suffix in (".jpg", ".jpeg", ".png"):
        candidate = stem.with_suffix(suffix)
        if candidate.exists():
            return candidate
    return None


def crop_for_row(image, fields):
    h, w = image.shape[:2]
    x, y, bw, bh = map(float, fields[1:5])
    x1 = max(0, int((x - bw / 2) * w))
    y1 = max(0, int((y - bh / 2) * h))
    x2 = min(w, int((x + bw / 2) * w + 0.999))
    y2 = min(h, int((y + bh / 2) * h + 0.999))
    return image[y1:y2, x1:x2]


def ocr(crop, whitelist="ABCD", psm=10):
    gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
    scale = max(2.0, 220.0 / max(gray.shape))
    enlarged = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    enlarged = cv2.GaussianBlur(enlarged, (3, 3), 0)
    _, binary = cv2.threshold(enlarged, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    ok, encoded = cv2.imencode(".png", binary)
    if not ok:
        return []
    command = ["tesseract", "stdin", "stdout", "--psm", str(psm), "-l", "eng"]
    if whitelist:
        command += ["-c", f"tessedit_char_whitelist={whitelist}"]
    command += ["tsv"]
    proc = subprocess.run(
        command,
        input=encoded.tobytes(), stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
        check=False, timeout=3,
    )
    lines = proc.stdout.decode(errors="ignore").splitlines()
    out = []
    for line in lines[1:]:
        cols = line.split("\t")
        if len(cols) >= 12 and cols[11].strip():
            out.append((cols[11].strip(), float(cols[10])))
    return out


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--shape-model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    model = YOLO(str(args.shape_model))
    rows = []
    for split in ("train", "val", "test"):
        for label in sorted((args.dataset / "labels" / split).glob("*.txt")):
            image_path = image_for_label(args.dataset, label)
            if image_path is None:
                continue
            image = cv2.imread(str(image_path))
            if image is None:
                continue
            for row_index, raw in enumerate(label.read_text().splitlines()):
                fields = raw.split()
                if len(fields) < 5 or int(fields[0]) != 3:
                    continue
                crop = crop_for_row(image, fields)
                if crop.size == 0:
                    continue
                result = model.predict(crop, imgsz=640, conf=0.01, iou=0.55, verbose=False)[0]
                detections = []
                if result.boxes is not None:
                    names = result.names
                    for item in result.boxes:
                        cls = int(item.cls.item())
                        name = str(names.get(cls, cls)) if isinstance(names, dict) else str(names[cls])
                        detections.append(f"{name}:{float(item.conf.item()):.3f}")
                rows.append({
                    "split": split,
                    "image": image_path.name,
                    "row": row_index,
                    "width": crop.shape[1],
                    "height": crop.shape[0],
                    "ocr": ";".join(f"{t}:{c:.1f}" for t, c in ocr(crop)),
                    "ocr_alnum_10": ";".join(f"{t}:{c:.1f}" for t, c in ocr(crop, "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789", 10)),
                    "ocr_alnum_6": ";".join(f"{t}:{c:.1f}" for t, c in ocr(crop, "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789", 6)),
                    "ocr_raw_11": ";".join(f"{t}:{c:.1f}" for t, c in ocr(crop, "", 11)),
                    "shape": ";".join(detections),
                })
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=rows[0].keys() if rows else ["split", "image"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"wrote={args.output} rows={len(rows)}")


if __name__ == "__main__":
    main()
