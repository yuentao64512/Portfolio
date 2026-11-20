#!/usr/bin/env python3
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

from autolabel_training12_v16 import crop, looks_like_buoy, looks_like_dock

audit = Path(sys.argv[1])
class_id = int(sys.argv[2])
limit = int(sys.argv[3]) if len(sys.argv) > 3 else 50
rows = [r for r in csv.DictReader(audit.open()) if int(r["class_id"]) == class_id][:limit]
for index, row in enumerate(rows):
    image = cv2.imread(row["output_image"])
    box = tuple(float(row[k]) for k in ("x1", "y1", "x2", "y2"))
    patch = crop(image, box)
    h, w = patch.shape[:2]
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV)
    hue, sat, val = cv2.split(hsv)
    white = (sat <= 65) & (val >= 105)
    cyan = ((hue >= 72) & (hue <= 112) & (sat >= 50) & (val >= 45))
    dark = val <= 105
    upper = white[int(.12*h):max(int(.36*h),1)]
    middle = white[int(.38*h):max(int(.72*h),1)]
    uw = int(np.any(upper, axis=0).sum()) if upper.size else 0
    mw = int(np.any(middle, axis=0).sum()) if middle.size else 0
    lower = slice(int(.58*h), h)
    print(index, Path(row["source"]).name, f"aspect={w/max(h,1):.2f}",
          f"white={white.mean():.2f}", f"uw/mw={uw/max(mw,1):.2f}",
          f"mw/w={mw/max(w,1):.2f}", f"dark={dark[lower].mean():.2f}",
          f"cyan={cyan[lower].mean():.2f}", f"buoy={looks_like_buoy(patch)}",
          f"dock={looks_like_dock(patch)}")
