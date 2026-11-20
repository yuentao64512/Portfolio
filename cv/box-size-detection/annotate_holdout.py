"""Annotate the three-panel held-out composite with model predictions."""

import argparse
from collections import Counter, deque
import csv
from pathlib import Path

import cv2
from ultralytics import YOLO


ROOT = Path('/home/robotx/box_size_training')
SOURCE = ROOT / 'three_boxes_holdout_composite.mp4'
OUTPUT = ROOT / 'three_boxes_holdout_annotated.mp4'
CSV_PATH = ROOT / 'holdout_composite_predictions.csv'


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--weights', type=Path, default=ROOT / 'runs' / 'yolo11n_dataset_v2' / 'weights' / 'best.pt')
    parser.add_argument('--output', type=Path, default=OUTPUT)
    parser.add_argument('--csv', type=Path, default=CSV_PATH)
    parser.add_argument('--device', default='cpu')
    args = parser.parse_args()
    model = YOLO(str(args.weights))
    cap = cv2.VideoCapture(str(args.source))
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if width != 1920 or height != 720:
        raise RuntimeError(f'Unexpected composite size: {width}x{height}')
    writer = cv2.VideoWriter(str(args.output), cv2.VideoWriter_fourcc(*'mp4v'),
                             fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f'Cannot create {args.output}')
    histories = [deque(maxlen=12) for _ in range(3)]
    rows = []
    frame_id = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        for panel in range(3):
            left = panel * 640
            roi = frame[:, left:left + 640]
            result = model.predict(roi, imgsz=640, conf=0.5,
                                   device=args.device, verbose=False)[0]
            if len(result.boxes):
                best_index = int(result.boxes.conf.argmax().item())
                box = result.boxes.xyxy[best_index].cpu().numpy().astype(int)
                class_id = int(result.boxes.cls[best_index].item())
                confidence = float(result.boxes.conf[best_index].item())
                histories[panel].append((class_id, confidence))
                x1, y1, x2, y2 = box
                cv2.rectangle(frame, (left + x1, y1), (left + x2, y2),
                              (0, 255, 0), 3)
                raw_label = model.names[class_id]
                cv2.putText(frame, f'{raw_label} {confidence:.2f}',
                            (left + max(x1, 5), max(y1 - 8, 75)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            else:
                class_id, confidence, raw_label = None, 0.0, ''
                histories[panel].append(None)
            valid = [item for item in histories[panel] if item is not None]
            if valid:
                counts = Counter(item[0] for item in valid)
                winner, votes = counts.most_common(1)[0]
                average = sum(item[1] for item in valid if item[0] == winner) / votes
                confirmed = votes >= 8 and average >= 0.95
                smooth_label = model.names[winner] if confirmed else 'uncertain'
            else:
                average, confirmed, smooth_label = 0.0, False, 'uncertain'
            cv2.putText(frame, f'Model: {smooth_label}',
                        (left + 20, 82), cv2.FONT_HERSHEY_SIMPLEX,
                        0.75, (255, 255, 255), 2)
            rows.append({'frame': frame_id, 'time_s': frame_id / fps,
                         'panel': panel, 'raw_label': raw_label,
                         'raw_confidence': confidence,
                         'smoothed_label': smooth_label,
                         'smoothed_confidence': average,
                         'confirmed': confirmed})
        writer.write(frame)
        frame_id += 1
    writer.release()
    cap.release()
    with args.csv.open('w', newline='') as handle:
        csv_writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        csv_writer.writeheader()
        csv_writer.writerows(rows)
    print(f'frames={frame_id} video={args.output} csv={args.csv}')


if __name__ == '__main__':
    main()
