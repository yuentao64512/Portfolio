"""Build a conservative training set from reviewed automatic labels.

It filters the visible false detections, adds deterministic blur variants and
background-only crops, and keeps contiguous time splits from the source clips.
"""

from __future__ import annotations

import csv
import json
import random
import shutil
from pathlib import Path

import cv2
import numpy as np


ROOT = Path('/home/robotx/box_size_training')
SOURCE = ROOT / 'dataset_v1'
TARGET = ROOT / 'dataset_v2'
RNG = random.Random(4207)


def blur_image(frame: np.ndarray, kind: str) -> np.ndarray:
    if kind == 'motion':
        size = RNG.randrange(5, 26, 2)
        kernel = np.zeros((size, size), dtype=np.float32)
        if RNG.random() < 0.5:
            kernel[size // 2, :] = 1.0 / size
        else:
            kernel[:, size // 2] = 1.0 / size
        return cv2.filter2D(frame, -1, kernel)
    if kind == 'gaussian':
        sigma = RNG.uniform(1, 3)
        return cv2.GaussianBlur(frame, (0, 0), sigma)
    if kind == 'defocus':
        radius = RNG.randint(3, 7)
        return cv2.GaussianBlur(frame, (radius * 2 + 1, radius * 2 + 1), 0)
    if kind == 'resize':
        fraction = RNG.uniform(0.5, 0.75)
        height, width = frame.shape[:2]
        small = cv2.resize(frame, (round(width * fraction), round(height * fraction)),
                           interpolation=cv2.INTER_AREA)
        return cv2.resize(small, (width, height), interpolation=cv2.INTER_LINEAR)
    quality = RNG.randint(40, 80)
    _, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
    return cv2.imdecode(encoded, cv2.IMREAD_COLOR)


def negative_crop(frame: np.ndarray, row: dict[str, str]) -> np.ndarray | None:
    height, width = frame.shape[:2]
    x1, y1, x2, y2 = (int(float(row[key])) for key in ('x1', 'y1', 'x2', 'y2'))
    margin = 8
    regions = [
        (0, 0, max(0, x1 - margin), height),
        (min(width, x2 + margin), 0, width, height),
        (0, 0, width, max(0, y1 - margin)),
        (0, min(height, y2 + margin), width, height),
    ]
    usable = [(xa, ya, xb, yb) for xa, ya, xb, yb in regions
              if xb - xa >= 170 and yb - ya >= 170]
    if not usable:
        return None
    xa, ya, xb, yb = max(usable, key=lambda r: (r[2] - r[0]) * (r[3] - r[1]))
    return frame[ya:yb, xa:xb]


def main() -> None:
    rows = list(csv.DictReader((ROOT / 'manifest_v1.csv').open()))
    for split in ('train', 'val', 'test'):
        (TARGET / 'images' / split).mkdir(parents=True, exist_ok=True)
        (TARGET / 'labels' / split).mkdir(parents=True, exist_ok=True)
    kept = []
    removed = []
    for row in rows:
        if row['accepted'] != 'True':
            removed.append({**row, 'reason': 'no confident proposal'})
            continue
        area = float(row['area_fraction'])
        class_name = row['class_name']
        if class_name == 'medium' and area < 0.10:
            removed.append({**row, 'reason': 'partial/false medium proposal'})
            continue
        if class_name == 'large' and area < 0.10:
            removed.append({**row, 'reason': 'small background false proposal'})
            continue
        source_image = ROOT / row['image']
        source_label = SOURCE / 'labels' / row['split'] / (source_image.stem + '.txt')
        target_image = TARGET / 'images' / row['split'] / source_image.name
        target_label = TARGET / 'labels' / row['split'] / source_label.name
        shutil.copy2(source_image, target_image)
        shutil.copy2(source_label, target_label)
        kept.append(row)

    training = [row for row in kept if row['split'] == 'train']
    blur_count = round(len(training) * 0.54)  # ~35% of positive train images after addition
    blur_choices = RNG.sample(training, blur_count)
    blur_kinds = ('motion', 'gaussian', 'defocus', 'resize', 'jpeg')
    for index, row in enumerate(blur_choices):
        source_image = TARGET / 'images' / 'train' / Path(row['image']).name
        frame = cv2.imread(str(source_image))
        kind = blur_kinds[index % len(blur_kinds)]
        blurred = blur_image(frame, kind)
        stem = source_image.stem + '__blur_' + kind
        cv2.imwrite(str(TARGET / 'images' / 'train' / (stem + '.jpg')),
                    blurred, [cv2.IMWRITE_JPEG_QUALITY, 90])
        label = TARGET / 'labels' / 'train' / (source_image.stem + '.txt')
        shutil.copy2(label, TARGET / 'labels' / 'train' / (stem + '.txt'))

    negatives = 0
    for row in RNG.sample(training, min(len(training), 130)):
        source_image = TARGET / 'images' / 'train' / Path(row['image']).name
        frame = cv2.imread(str(source_image))
        crop = negative_crop(frame, row)
        if crop is None:
            continue
        stem = 'negative_' + source_image.stem
        cv2.imwrite(str(TARGET / 'images' / 'train' / (stem + '.jpg')),
                    crop, [cv2.IMWRITE_JPEG_QUALITY, 90])
        (TARGET / 'labels' / 'train' / (stem + '.txt')).write_text('')
        negatives += 1

    (TARGET / 'data.yaml').write_text(
        f'path: {TARGET}\ntrain: images/train\nval: images/val\n'
        'test: images/test\nnames:\n  0: small\n  1: medium\n  2: large\n'
    )
    summary = {'positive_train': len(training),
               'positive_val': sum(row['split'] == 'val' for row in kept),
               'positive_test': sum(row['split'] == 'test' for row in kept),
               'blurred_train_copies': blur_count,
               'negative_train_crops': negatives,
               'excluded_candidates': len(removed),
               'class_train': {name: sum(row['class_name'] == name for row in training)
                               for name in ('small', 'medium', 'large')},
               'auto_labels': True,
               'independent_test_video': False}
    (ROOT / 'dataset_v2_summary.json').write_text(json.dumps(summary, indent=2))
    with (ROOT / 'excluded_v2.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=sorted({key for row in removed for key in row}))
        writer.writeheader()
        writer.writerows(removed)
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
