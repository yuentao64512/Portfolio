from pathlib import Path
import csv
import cv2
import numpy as np
from ultralytics import SAM

root = Path('/home/robotx/box_size_training')
rows = list(csv.DictReader((root / 'manifest_v1.csv').open()))
sam = SAM('/home/robotx/dock_yolo_20261025/sam2.1_t.pt')
samples = {'small': 40, 'medium': 40, 'large': 40}
tiles = []
for name, sample_id in samples.items():
    row = next(r for r in rows if r['class_name'] == name and
               Path(r.get('image', '')).stem == f'{name}_{sample_id:03d}')
    frame = cv2.imread(str(root / row['image']))
    box = [float(row[k]) for k in ('x1', 'y1', 'x2', 'y2')]
    result = sam.predict(frame, bboxes=[box], device=0, verbose=False)[0]
    if result.masks is None or not len(result.masks.data):
        raise RuntimeError(f'No mask for {name}')
    mask = result.masks.data[0].cpu().numpy() > 0.5
    overlay = frame.copy()
    overlay[mask] = (0.5 * overlay[mask] + 0.5 * np.array([0, 0, 255])).astype(np.uint8)
    cv2.rectangle(overlay, tuple(map(int, box[:2])), tuple(map(int, box[2:])),
                  (0, 255, 0), 3)
    h, w = overlay.shape[:2]
    scale = min(600 / w, 500 / h)
    thumb = cv2.resize(overlay, (round(w * scale), round(h * scale)))
    tile = np.zeros((540, 620, 3), dtype=np.uint8)
    x = (620 - thumb.shape[1]) // 2
    y = 35 + (500 - thumb.shape[0]) // 2
    tile[y:y + thumb.shape[0], x:x + thumb.shape[1]] = thumb
    cv2.putText(tile, name, (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.85,
                (255, 255, 255), 2)
    tiles.append(tile)
cv2.imwrite(str(root / 'sam_probe.jpg'), np.hstack(tiles))
