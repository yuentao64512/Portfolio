"""Create SAM cutouts and 1,000 labelled multi-box training composites."""

from __future__ import annotations

import csv
import json
import random
import shutil
from pathlib import Path

import cv2
import numpy as np
from ultralytics import SAM

from prepare_v2 import blur_image


ROOT = Path('/home/robotx/box_size_training')
SOURCE = ROOT / 'dataset_v2'
TARGET = ROOT / 'dataset_v3'
CUTOUTS = ROOT / 'cutouts_v3'
RNG = random.Random(5117)
N_COMPOSITES = 1000
CLASSES = ('small', 'medium', 'large')


def make_cutouts() -> dict[int, list[Path]]:
    rows = [r for r in csv.DictReader((ROOT / 'manifest_v1.csv').open())
            if r['accepted'] == 'True' and r['split'] == 'train'
            and (SOURCE / 'images' / 'train' / Path(r['image']).name).is_file()]
    sam = SAM('/home/robotx/dock_yolo_20261025/sam2.1_t.pt')
    paths: dict[int, list[Path]] = {i: [] for i in range(3)}
    for class_id, name in enumerate(CLASSES):
        group = sorted((r for r in rows if r['class_name'] == name),
                       key=lambda r: float(r['fraction']))
        for bucket in range(24):
            start = int(bucket * len(group) / 24)
            end = max(start + 1, int((bucket + 1) * len(group) / 24))
            chosen = max(group[start:end], key=lambda r: float(r['confidence']))
            frame = cv2.imread(str(ROOT / chosen['image']))
            box = [float(chosen[k]) for k in ('x1', 'y1', 'x2', 'y2')]
            result = sam.predict(frame, bboxes=[box], device=0, verbose=False)[0]
            if result.masks is None or not len(result.masks.data):
                continue
            mask = (result.masks.data[0].cpu().numpy() > 0.5).astype(np.uint8)
            ys, xs = np.nonzero(mask)
            if len(xs) < 100:
                continue
            x1, x2 = max(0, int(xs.min()) - 2), min(frame.shape[1], int(xs.max()) + 3)
            y1, y2 = max(0, int(ys.min()) - 2), min(frame.shape[0], int(ys.max()) + 3)
            crop = frame[y1:y2, x1:x2]
            alpha = mask[y1:y2, x1:x2] * 255
            fill = float((alpha > 0).mean())
            if not 0.35 <= fill <= 0.97:
                continue
            bgra = cv2.cvtColor(crop, cv2.COLOR_BGR2BGRA)
            bgra[:, :, 3] = alpha
            folder = CUTOUTS / name
            folder.mkdir(parents=True, exist_ok=True)
            path = folder / f'{name}_{bucket:02d}.png'
            cv2.imwrite(str(path), bgra)
            paths[class_id].append(path)
    return paths


def fit_background(path: Path) -> np.ndarray:
    image = cv2.imread(str(path))
    height, width = image.shape[:2]
    scale = max(640 / width, 640 / height)
    image = cv2.resize(image, (round(width * scale), round(height * scale)))
    height, width = image.shape[:2]
    x = RNG.randrange(max(1, width - 639))
    y = RNG.randrange(max(1, height - 639))
    return image[y:y + 640, x:x + 640].copy()


def transform_cutout(path: Path, class_id: int) -> np.ndarray:
    cutout = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if RNG.random() < 0.5:
        cutout = cv2.flip(cutout, 1)
    target_long = (155, 215, 275)[class_id] * RNG.uniform(0.80, 1.25)
    height, width = cutout.shape[:2]
    scale = target_long / max(width, height)
    cutout = cv2.resize(cutout, (max(2, round(width * scale)),
                                 max(2, round(height * scale))),
                         interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_LINEAR)
    angle = RNG.choice((0, 90, 180, 270)) + RNG.uniform(-12, 12)
    height, width = cutout.shape[:2]
    center = (width / 2, height / 2)
    matrix = cv2.getRotationMatrix2D(center, angle, 1.0)
    cosine, sine = abs(matrix[0, 0]), abs(matrix[0, 1])
    new_width = int(height * sine + width * cosine)
    new_height = int(height * cosine + width * sine)
    matrix[0, 2] += new_width / 2 - center[0]
    matrix[1, 2] += new_height / 2 - center[1]
    return cv2.warpAffine(cutout, matrix, (new_width, new_height),
                          flags=cv2.INTER_LINEAR,
                          borderMode=cv2.BORDER_CONSTANT, borderValue=(0, 0, 0, 0))


def paste(background: np.ndarray, cutout: np.ndarray, x: int, y: int) -> tuple[int, int, int, int]:
    height, width = cutout.shape[:2]
    alpha = cutout[:, :, 3].astype(np.float32) / 255.0
    shadow = cv2.GaussianBlur(alpha, (0, 0), 9)
    shadow_x, shadow_y = min(x + 8, 640 - width), min(y + 8, 640 - height)
    shadow_roi = background[shadow_y:shadow_y + height, shadow_x:shadow_x + width]
    shadow_roi[:] = (shadow_roi * (1 - 0.22 * shadow[:, :, None])).astype(np.uint8)
    roi = background[y:y + height, x:x + width]
    roi[:] = (roi * (1 - alpha[:, :, None]) +
              cutout[:, :, :3] * alpha[:, :, None]).astype(np.uint8)
    ys, xs = np.nonzero(alpha > 0.1)
    return x + int(xs.min()), y + int(ys.min()), x + int(xs.max()) + 1, y + int(ys.max()) + 1


def overlap(a: tuple[int, int, int, int], b: tuple[int, int, int, int]) -> float:
    x1, y1 = max(a[0], b[0]), max(a[1], b[1])
    x2, y2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(x2 - x1, 0) * max(y2 - y1, 0)
    area_a = (a[2] - a[0]) * (a[3] - a[1])
    area_b = (b[2] - b[0]) * (b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1)


def main() -> None:
    if TARGET.exists():
        raise SystemExit(f'Target already exists: {TARGET}')
    shutil.copytree(SOURCE, TARGET)
    cutouts = make_cutouts()
    if any(len(paths) < 12 for paths in cutouts.values()):
        raise RuntimeError(f'Too few usable cutouts: {[(k, len(v)) for k, v in cutouts.items()]}')
    backgrounds = sorted((SOURCE / 'images' / 'train').glob('negative_*.jpg'))
    if len(backgrounds) < 50:
        raise RuntimeError('Not enough empty background crops')
    blur_kinds = ('motion', 'gaussian', 'defocus', 'resize', 'jpeg')
    preview = []
    object_counts = [0, 0, 0]
    for index in range(N_COMPOSITES):
        canvas = fit_background(RNG.choice(backgrounds))
        count = RNG.choices((1, 2, 3), weights=(0.50, 0.35, 0.15))[0]
        classes = RNG.sample(range(3), count)
        placed: list[tuple[int, int, int, int]] = []
        labels = []
        for class_id in classes:
            cutout = transform_cutout(RNG.choice(cutouts[class_id]), class_id)
            height, width = cutout.shape[:2]
            if height >= 600 or width >= 600:
                continue
            for _ in range(30):
                x = RNG.randrange(10, 640 - width - 9)
                y = RNG.randrange(10, 640 - height - 9)
                candidate = (x, y, x + width, y + height)
                if all(overlap(candidate, other) < 0.12 for other in placed):
                    break
            else:
                continue
            box = paste(canvas, cutout, x, y)
            placed.append(box)
            x1, y1, x2, y2 = box
            labels.append((class_id, (x1 + x2) / 1280, (y1 + y2) / 1280,
                           (x2 - x1) / 640, (y2 - y1) / 640))
            object_counts[class_id] += 1
        if not labels:
            continue
        if index % 100 < 35:
            canvas = blur_image(canvas, blur_kinds[index % len(blur_kinds)])
        stem = f'composite_{index:04d}'
        cv2.imwrite(str(TARGET / 'images' / 'train' / f'{stem}.jpg'), canvas,
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        (TARGET / 'labels' / 'train' / f'{stem}.txt').write_text(
            ''.join(f'{cls} {xc:.6f} {yc:.6f} {w:.6f} {h:.6f}\n'
                    for cls, xc, yc, w, h in labels)
        )
        if index % 50 == 0:
            tile = canvas.copy()
            for cls, xc, yc, w, h in labels:
                x1, y1 = int((xc - w / 2) * 640), int((yc - h / 2) * 640)
                x2, y2 = int((xc + w / 2) * 640), int((yc + h / 2) * 640)
                cv2.rectangle(tile, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(tile, CLASSES[cls], (x1, max(20, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            preview.append(cv2.resize(tile, (320, 320)))
    sheet = np.vstack([np.hstack(preview[i:i + 5]) for i in range(0, len(preview), 5)])
    cv2.imwrite(str(ROOT / 'composites_v3_preview.jpg'), sheet)
    (TARGET / 'data.yaml').write_text(
        f'path: {TARGET}\ntrain: images/train\nval: images/val\n'
        'test: images/test\nnames:\n  0: small\n  1: medium\n  2: large\n'
    )
    summary = {'composites': N_COMPOSITES, 'blurred_composites': 350,
               'cutouts': {CLASSES[k]: len(v) for k, v in cutouts.items()},
               'source_background_crops': len(backgrounds),
               'object_counts': {CLASSES[i]: count for i, count in enumerate(object_counts)},
               'train_images': len(list((TARGET / 'images' / 'train').glob('*.jpg'))),
               'val_images': len(list((TARGET / 'images' / 'val').glob('*.jpg'))),
               'test_images': len(list((TARGET / 'images' / 'test').glob('*.jpg'))),
               'backgrounds_are_from_the_three_source_clips': True}
    (ROOT / 'dataset_v3_summary.json').write_text(json.dumps(summary, indent=2))
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()
