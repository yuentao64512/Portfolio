#!/usr/bin/env python3
from collections import Counter
from pathlib import Path

import cv2


PROJECT = Path("/home/robotx/pipe_cones")
DATASET = PROJECT / "dataset"
counts = Counter()
box_heights = Counter()

for split in ("train", "val", "test"):
    images = sorted((DATASET / "images" / split).glob("*.jpg"))
    labels = sorted((DATASET / "labels" / split).glob("*.txt"))
    assert len(images) == len(labels), (split, len(images), len(labels))
    for image_path in images:
        label_path = DATASET / "labels" / split / f"{image_path.stem}.txt"
        rows = [line for line in label_path.read_text().splitlines() if line.strip()]
        assert len(rows) == 1, (label_path, len(rows))
        values = rows[0].split()
        assert len(values) == 5 and values[0] == "0", (label_path, values)
        xc, yc, bw, bh = map(float, values[1:])
        assert 0 < xc < 1 and 0 < yc < 1 and 0 < bw <= 1 and 0 < bh <= 1
        assert 0 <= xc - bw / 2 and xc + bw / 2 <= 1
        assert 0 <= yc - bh / 2 and yc + bh / 2 <= 1
        image = cv2.imread(str(image_path))
        assert image is not None, image_path
        box_heights[split] += round(bh * image.shape[0])
    counts[split] = len(images)

assert sum(counts.values()) == 120, counts
for split in ("train", "val", "test"):
    mean_height = box_heights[split] / counts[split]
    print(f"{split}: {counts[split]} images, mean box height {mean_height:.1f}px")
print(f"total: {sum(counts.values())} images; labels valid")
