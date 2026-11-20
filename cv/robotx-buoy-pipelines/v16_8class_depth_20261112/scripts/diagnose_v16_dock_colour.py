#!/usr/bin/env python3
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

from autolabel_training12_v16 import crop

rows = [r for r in csv.DictReader(open(sys.argv[1])) if int(r["class_id"]) == 4]
for idx, row in enumerate(rows[:80]):
    image = cv2.imread(row["output_image"])
    box = tuple(float(row[k]) for k in ("x1", "y1", "x2", "y2"))
    patch = crop(image, box)
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    vivid = (sat >= 55) & (val >= 60)
    masks = {
        "red": vivid & ((hue <= 25) | (hue >= 168)),
        "green": vivid & (hue >= 35) & (hue <= 90),
    }
    values = []
    for name, mask in masks.items():
        count, _, stats, centers = cv2.connectedComponentsWithStats(mask.astype(np.uint8), 8)
        comps = []
        for i in range(1, count):
            x, y, w, h, area = stats[i]
            cx, cy = centers[i]
            if cy >= .35 * patch.shape[0]:
                comps.append((int(area), round(cx/patch.shape[1], 2), round(cy/patch.shape[0], 2)))
        values.append((name, max(comps, default=(0, 0, 0))))
    print(idx, Path(row["source"]).name, patch.shape[:2], values)
