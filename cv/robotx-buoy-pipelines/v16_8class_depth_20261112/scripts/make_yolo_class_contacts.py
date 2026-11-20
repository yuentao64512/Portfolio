#!/usr/bin/env python3
import argparse
import random
from pathlib import Path

import cv2
import numpy as np
import yaml


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--split", default="train")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--per-class", type=int, default=50)
    args = parser.parse_args()
    config = yaml.safe_load(args.data.read_text())
    root = Path(config["path"])
    names = {int(k): v for k, v in config["names"].items()}
    labels = sorted((root / "labels" / args.split).glob("*.txt"))
    rows = {class_id: [] for class_id in names}
    for label in labels:
        image = None
        for suffix in (".jpg", ".jpeg", ".png"):
            candidate = root / "images" / args.split / label.with_suffix(suffix).name
            if candidate.exists():
                image = candidate
                break
        if image is None:
            continue
        for line in label.read_text().splitlines():
            parts = line.split()
            if len(parts) >= 5 and int(parts[0]) in rows:
                rows[int(parts[0])].append((image, tuple(map(float, parts[1:5]))))
    args.output.mkdir(parents=True, exist_ok=True)
    rng = random.Random(20261112)
    for class_id, items in rows.items():
        rng.shuffle(items)
        tiles = []
        for image_path, (xc, yc, bw, bh) in items[:args.per_class]:
            image = cv2.imread(str(image_path))
            if image is None:
                continue
            h, w = image.shape[:2]
            x1, x2 = int((xc-bw/2)*w), int((xc+bw/2)*w)
            y1, y2 = int((yc-bh/2)*h), int((yc+bh/2)*h)
            px, py = max(8, (x2-x1)//4), max(8, (y2-y1)//4)
            patch = image[max(0,y1-py):min(h,y2+py), max(0,x1-px):min(w,x2+px)]
            if patch.size == 0:
                continue
            patch = cv2.resize(patch, (240,160), interpolation=cv2.INTER_AREA)
            cv2.putText(patch, names[class_id], (5,18), cv2.FONT_HERSHEY_SIMPLEX,
                        .5, (0,255,255), 1, cv2.LINE_AA)
            tiles.append(patch)
        if not tiles:
            continue
        while len(tiles) % 5:
            tiles.append(np.zeros_like(tiles[0]))
        sheet = np.vstack([np.hstack(tiles[i:i+5]) for i in range(0,len(tiles),5)])
        cv2.imwrite(str(args.output / f"class_{class_id}_{names[class_id]}.jpg"), sheet)


if __name__ == "__main__":
    main()
