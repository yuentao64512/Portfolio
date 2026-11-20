"""Evaluate a trained box detector on time-held-out source frames.

The reference boxes were drafted by YOLO World, so metrics are provisional.
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


ROOT = Path('/home/robotx/box_size_training')
THRESHOLD = 0.95


def iou(box_a: np.ndarray, box_b: np.ndarray) -> float:
    x1 = max(box_a[0], box_b[0])
    y1 = max(box_a[1], box_b[1])
    x2 = min(box_a[2], box_b[2])
    y2 = min(box_a[3], box_b[3])
    intersection = max(x2 - x1, 0) * max(y2 - y1, 0)
    area_a = max(box_a[2] - box_a[0], 0) * max(box_a[3] - box_a[1], 0)
    area_b = max(box_b[2] - box_b[0], 0) * max(box_b[3] - box_b[1], 0)
    return float(intersection / max(area_a + area_b - intersection, 1e-9))


def reference_box(label_path: Path, width: int, height: int) -> tuple[int, np.ndarray]:
    cls, xc, yc, bw, bh = map(float, label_path.read_text().split())
    return int(cls), np.asarray([(xc - bw / 2) * width, (yc - bh / 2) * height,
                                 (xc + bw / 2) * width, (yc + bh / 2) * height])


def score_frame(model: YOLO, image: np.ndarray, gt_cls: int, gt_box: np.ndarray):
    result = model.predict(image, conf=0.01, imgsz=640, device=0,
                           verbose=False)[0]
    boxes = result.boxes.xyxy.cpu().numpy()
    confidences = result.boxes.conf.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    matched = [(idx, iou(box, gt_box)) for idx, box in enumerate(boxes)]
    matched = [(idx, overlap) for idx, overlap in matched if overlap >= 0.5]
    best = max(matched, key=lambda item: float(confidences[item[0]]), default=None)
    correct = bool(best and classes[best[0]] == gt_cls and confidences[best[0]] >= THRESHOLD)
    wrong = bool(best and classes[best[0]] != gt_cls and confidences[best[0]] >= THRESHOLD)
    high = [(int(cls), float(conf), float(iou(box, gt_box)))
            for box, conf, cls in zip(boxes, confidences, classes)
            if conf >= THRESHOLD]
    best_info = None if best is None else {
        'class_id': int(classes[best[0]]),
        'confidence': float(confidences[best[0]]),
        'iou': float(best[1]),
    }
    return correct, wrong, high, best_info, boxes, confidences, classes


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--version', choices=('v2', 'v3_clean'), default='v2')
    parser.add_argument('--model', choices=('n', 's'), default='n')
    args = parser.parse_args()
    dataset = ROOT / f'dataset_{args.version}'
    weights = ROOT / 'runs' / f'yolo11{args.model}_dataset_{args.version}' / 'weights' / 'best.pt'
    report_name = f'{args.version}_yolo11{args.model}'
    if not weights.is_file():
        raise SystemExit(f'Missing trained weights: {weights}')
    model = YOLO(str(weights))
    report = defaultdict(lambda: {'frames': 0, 'correct_at_095': 0,
                                   'wrong_label_at_095': 0,
                                   'predictions_at_095': 0,
                                   'true_predictions_at_095': 0,
                                   'blur_correct_at_095': 0})
    rows = []
    preview = []
    preview_counts = defaultdict(int)
    for image_path in sorted((dataset / 'images' / 'test').glob('*.jpg')):
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        height, width = image.shape[:2]
        gt_cls, gt_box = reference_box(dataset / 'labels' / 'test' /
                                       (image_path.stem + '.txt'), width, height)
        name = model.names[gt_cls]
        correct, wrong, high, best, boxes, confs, classes = score_frame(
            model, image, gt_cls, gt_box)
        blurred = cv2.GaussianBlur(image, (0, 0), 2.0)
        blur_correct = score_frame(model, blurred, gt_cls, gt_box)[0]
        counters = report[name]
        counters['frames'] += 1
        counters['correct_at_095'] += int(correct)
        counters['wrong_label_at_095'] += int(wrong)
        counters['blur_correct_at_095'] += int(blur_correct)
        for cls, confidence, overlap in high:
            class_counters = report[model.names[cls]]
            class_counters['predictions_at_095'] += 1
            if cls == gt_cls and overlap >= 0.5:
                class_counters['true_predictions_at_095'] += 1
        rows.append({'image': image_path.name, 'true_class': name,
                     'correct_at_095': correct, 'wrong_label_at_095': wrong,
                     'blur_correct_at_095': blur_correct,
                     'best_class': None if best is None else model.names[best['class_id']],
                     'best_confidence': None if best is None else best['confidence'],
                     'best_iou': None if best is None else best['iou']})
        if preview_counts[name] < 10:
            annotated = image.copy()
            cv2.rectangle(annotated, tuple(gt_box[:2].astype(int)),
                          tuple(gt_box[2:].astype(int)), (255, 0, 0), 2)
            for box, confidence, cls in zip(boxes, confs, classes):
                if confidence < 0.5:
                    continue
                x1, y1, x2, y2 = box.astype(int)
                cv2.rectangle(annotated, (x1, y1), (x2, y2),
                              (0, 255, 0) if cls == gt_cls else (0, 0, 255), 2)
            scale = min(360 / width, 270 / height)
            thumb = cv2.resize(annotated, (round(width * scale), round(height * scale)))
            tile = np.zeros((300, 380, 3), dtype=np.uint8)
            left = (380 - thumb.shape[1]) // 2
            top = 28 + (270 - thumb.shape[0]) // 2
            tile[top:top + thumb.shape[0], left:left + thumb.shape[1]] = thumb
            cv2.putText(tile, f'{image_path.stem} {best["confidence"]:.2f}' if best else image_path.stem,
                        (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.56, (255, 255, 255), 1)
            preview.append(tile)
            preview_counts[name] += 1
    totals = {key: sum(value[key] for value in report.values())
              for key in ('frames', 'correct_at_095', 'wrong_label_at_095',
                          'predictions_at_095', 'true_predictions_at_095',
                          'blur_correct_at_095')}
    output = {'weights': str(weights), 'threshold': THRESHOLD,
              'reference_labels': 'automatically drafted; not fully hand-verified',
              'test_split': 'last 15% of each source video, contiguous by time',
              'per_class': dict(report), 'total': totals}
    (ROOT / f'test_report_{report_name}.json').write_text(json.dumps(output, indent=2))
    with (ROOT / f'test_predictions_{report_name}.csv').open('w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    if preview:
        sheet = np.vstack([np.hstack(preview[i:i + 5]) for i in range(0, len(preview), 5)])
        cv2.imwrite(str(ROOT / f'test_preview_{report_name}.jpg'), sheet)
    print(json.dumps(output, indent=2))


if __name__ == '__main__':
    main()
