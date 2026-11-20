"""Draft one YOLO box label per sampled video frame using YOLO World.

Automatic labels are candidates, not verified ground truth. Review the saved
label previews and manifest before treating validation metrics as reliable.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLOWorld


ROOT = Path('/home/robotx/box_size_training')
SOURCE = ROOT / 'source_videos'
DATASET = ROOT / 'dataset_v1'
CLASSES = ('small', 'medium', 'large')
VIDEO_NAMES = (
    'WhatsApp Video 2026-11-05 at 11.19.23 PM.mp4',
    'WhatsApp Video 2026-11-05 at 11.31.38 PM.mp4',
    'WhatsApp Video 2026-11-05 at 11.33.52 PM.mp4',
)
SAMPLES_PER_VIDEO = 160


def colour_fraction(frame: np.ndarray, box: np.ndarray, class_id: int) -> float:
    x1, y1, x2, y2 = map(int, box)
    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return 0.0
    hsv = cv2.cvtColor(cv2.resize(roi, (64, 64)), cv2.COLOR_BGR2HSV)
    hue, saturation, value = cv2.split(hsv)
    if class_id == 0:
        mask = ((hue >= 3) & (hue <= 30) & (saturation >= 30)
                & (value >= 35) & (value <= 230))
    elif class_id == 1:
        mask = ((hue >= 28) & (hue <= 95) & (saturation >= 25))
    else:
        mask = (value <= 100)
    return float(mask.mean())


def choose_box(frame: np.ndarray, result, class_id: int):
    height, width = frame.shape[:2]
    candidates = []
    for box, confidence, cls in zip(result.boxes.xyxy.cpu().numpy(),
                                    result.boxes.conf.cpu().numpy(),
                                    result.boxes.cls.cpu().numpy()):
        x1, y1, x2, y2 = map(float, box)
        area = (x2 - x1) * (y2 - y1) / (width * height)
        if area < 0.012 or area > 0.72:
            continue
        aspect = (x2 - x1) / max(y2 - y1, 1)
        if not 0.28 <= aspect <= 5.0:
            continue
        colour = colour_fraction(frame, box, class_id)
        if class_id == 1 and colour < 0.12:
            continue
        if class_id == 0 and colour < 0.08:
            continue
        name = result.names[int(cls)]
        name_bonus = 1.0
        if class_id == 2 and name == 'white cardboard box':
            name_bonus = 1.2
        if class_id == 0 and name == 'green cardboard box':
            name_bonus = 0.6
        if class_id == 1 and name == 'green cardboard box':
            name_bonus = 1.25
        score = float(confidence) * np.sqrt(area) * name_bonus
        score *= 0.65 + min(colour, 0.5) if class_id in (0, 1) else 1.0
        candidates.append((score, box, float(confidence), name, colour, area))
    if not candidates:
        return None
    return max(candidates, key=lambda item: item[0])


def main() -> None:
    model = YOLOWorld(str(ROOT / 'yolov8s-worldv2.pt'))
    model.set_classes(['cardboard box', 'product packaging box',
                       'white cardboard box', 'green cardboard box'])
    for split in ('train', 'val', 'test'):
        (DATASET / 'images' / split).mkdir(parents=True, exist_ok=True)
        (DATASET / 'labels' / split).mkdir(parents=True, exist_ok=True)
    review_dir = ROOT / 'review_v1'
    review_dir.mkdir(exist_ok=True)
    manifest = []
    tile_groups: dict[int, list[np.ndarray]] = {i: [] for i in range(3)}

    for class_id, video_name in enumerate(VIDEO_NAMES):
        video = SOURCE / video_name
        cap = cv2.VideoCapture(str(video))
        if not cap.isOpened():
            raise RuntimeError(f'Could not open {video}')
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        for sample_id in range(SAMPLES_PER_VIDEO):
            fraction = (sample_id + 0.5) / SAMPLES_PER_VIDEO
            frame_id = round(fraction * (total - 1))
            cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
            ok, frame = cap.read()
            if not ok:
                continue
            split = 'train' if fraction < 0.7 else 'val' if fraction < 0.85 else 'test'
            result = model.predict(frame, conf=0.08, iou=0.4, device=0,
                                   verbose=False)[0]
            chosen = choose_box(frame, result, class_id)
            row = {'class_id': class_id, 'class_name': CLASSES[class_id],
                   'video': video_name, 'frame': frame_id, 'fraction': fraction,
                   'split': split, 'accepted': bool(chosen)}
            if chosen is not None:
                _, box, confidence, model_class, colour, area = chosen
                height, width = frame.shape[:2]
                x1, y1, x2, y2 = map(float, box)
                xc = (x1 + x2) / (2 * width)
                yc = (y1 + y2) / (2 * height)
                bw = (x2 - x1) / width
                bh = (y2 - y1) / height
                stem = f'{CLASSES[class_id]}_{sample_id:03d}'
                image_path = DATASET / 'images' / split / f'{stem}.jpg'
                label_path = DATASET / 'labels' / split / f'{stem}.txt'
                cv2.imwrite(str(image_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 93])
                label_path.write_text(f'{class_id} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}\n')
                row.update({'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
                            'confidence': confidence, 'model_class': model_class,
                            'colour_fraction': colour, 'area_fraction': area,
                            'image': str(image_path.relative_to(ROOT))})
                if sample_id % 8 == 0:
                    tile = frame.copy()
                    cv2.rectangle(tile, (int(x1), int(y1)), (int(x2), int(y2)),
                                  (0, 255, 0), 3)
                    cv2.putText(tile, f'{stem} {confidence:.2f}', (5, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    h, w = tile.shape[:2]
                    scale = min(360 / w, 270 / h)
                    thumb = cv2.resize(tile, (round(w * scale), round(h * scale)))
                    canvas = np.zeros((300, 380, 3), dtype=np.uint8)
                    left = (380 - thumb.shape[1]) // 2
                    top = (300 - thumb.shape[0]) // 2
                    canvas[top:top + thumb.shape[0], left:left + thumb.shape[1]] = thumb
                    tile_groups[class_id].append(canvas)
            manifest.append(row)
        cap.release()

    for class_id, tiles in tile_groups.items():
        if tiles:
            rows = []
            for start in range(0, len(tiles), 5):
                group = tiles[start:start + 5]
                rows.append(np.hstack(group + [np.zeros_like(group[0])] * (5 - len(group))))
            cv2.imwrite(str(review_dir / f'{CLASSES[class_id]}_labels.jpg'),
                        np.vstack(rows))
    with (ROOT / 'manifest_v1.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=sorted({key for row in manifest for key in row}))
        writer.writeheader()
        writer.writerows(manifest)
    (DATASET / 'data.yaml').write_text(
        f'path: {DATASET}\ntrain: images/train\nval: images/val\n'
        'test: images/test\nnames:\n  0: small\n  1: medium\n  2: large\n'
    )
    counts = {}
    for row in manifest:
        key = f'{row["class_name"]}:{row["split"]}'
        counts.setdefault(key, {'accepted': 0, 'rejected': 0})
        counts[key]['accepted' if row['accepted'] else 'rejected'] += 1
    print(json.dumps({'counts': counts, 'dataset': str(DATASET)}, indent=2))


if __name__ == '__main__':
    main()
