#!/usr/bin/env python3
"""Export a compact, auditable summary of the completed YOLO training run."""
from __future__ import annotations

import csv
import json
from collections import Counter
from pathlib import Path

from PIL import Image


ROOT = Path("/home/robotx/fall_action_project/data/AERIAL_UPRIGHT_DOWN/yolo_autolabel_v1")
OUT = Path("reports/yolo_training")
OUT.mkdir(parents=True, exist_ok=True)
NAMES = {"0": "upright", "1": "down"}

manifest_path = OUT / "dataset_manifest.csv"
summary = {"dataset_root": str(ROOT), "classes": NAMES, "splits": {}, "total_images": 0, "total_boxes": 0}

with manifest_path.open("w", newline="", encoding="utf-8") as handle:
    writer = csv.DictWriter(handle, fieldnames=["split", "image", "label_file", "width", "height", "box_count", "classes"])
    writer.writeheader()
    for split in ("train", "val"):
        counts = Counter()
        image_count = 0
        box_count = 0
        for image_path in sorted((ROOT / "images" / split).glob("*")):
            label_path = ROOT / "labels" / split / f"{image_path.stem}.txt"
            with Image.open(image_path) as image:
                width, height = image.size
            labels = [line.split()[0] for line in label_path.read_text(encoding="utf-8").splitlines() if line.strip()]
            counts.update(NAMES.get(label, f"class_{label}") for label in labels)
            image_count += 1
            box_count += len(labels)
            writer.writerow({
                "split": split,
                "image": str(image_path),
                "label_file": str(label_path),
                "width": width,
                "height": height,
                "box_count": len(labels),
                "classes": ";".join(sorted(set(NAMES.get(label, f"class_{label}") for label in labels))),
            })
        summary["splits"][split] = {"images": image_count, "boxes": box_count, "class_boxes": dict(counts)}
        summary["total_images"] += image_count
        summary["total_boxes"] += box_count

(OUT / "dataset_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
print(json.dumps(summary, indent=2))
