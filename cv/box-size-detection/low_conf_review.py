from pathlib import Path
import csv
import cv2
import numpy as np

root = Path('/home/robotx/box_size_training')
rows = [r for r in csv.DictReader((root / 'manifest_v1.csv').open()) if r['accepted'] == 'True']
for class_name in ('small', 'medium', 'large'):
    selected = sorted((r for r in rows if r['class_name'] == class_name),
                      key=lambda r: float(r['confidence']))[:20]
    tiles = []
    for r in selected:
        frame = cv2.imread(str(root / r['image']))
        if frame is None:
            continue
        p1 = int(float(r['x1'])), int(float(r['y1']))
        p2 = int(float(r['x2'])), int(float(r['y2']))
        cv2.rectangle(frame, p1, p2, (0, 255, 0), 3)
        h, w = frame.shape[:2]
        scale = min(360 / w, 270 / h)
        thumb = cv2.resize(frame, (round(w * scale), round(h * scale)))
        tile = np.zeros((300, 380, 3), dtype=np.uint8)
        left = (380 - thumb.shape[1]) // 2
        top = 28 + (270 - thumb.shape[0]) // 2
        tile[top:top + thumb.shape[0], left:left + thumb.shape[1]] = thumb
        text = f'{class_name} f{r["frame"]} p{float(r["confidence"]):.2f}'
        cv2.putText(tile, text, (8, 19), cv2.FONT_HERSHEY_SIMPLEX,
                    0.58, (255, 255, 255), 1)
        tiles.append(tile)
    sheet = np.vstack([np.hstack(tiles[i:i + 5]) for i in range(0, len(tiles), 5)])
    cv2.imwrite(str(root / 'review_v1' / f'{class_name}_low_conf.jpg'), sheet)
