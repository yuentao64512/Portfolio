from pathlib import Path
import cv2
import numpy as np
from ultralytics import YOLOWorld

root = Path('/home/robotx/box_size_training')
model = YOLOWorld('yolov8s-worldv2.pt')
model.set_classes(['cardboard box', 'product packaging box', 'white cardboard box',
                   'green cardboard box'])
files = sorted((root / 'source_videos').glob('*.mp4'))
for class_id, path in enumerate(files):
    cap = cv2.VideoCapture(str(path))
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    tiles = []
    for index in range(9):
        frame_id = int((index + 0.5) * count / 9)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f'Cannot read {path.name} frame {frame_id}')
        result = model.predict(frame, conf=0.08, iou=0.4, device=0, verbose=False)[0]
        candidates = sorted(zip(result.boxes.xyxy.cpu().numpy(),
                                result.boxes.conf.cpu().numpy(),
                                result.boxes.cls.cpu().numpy()),
                            key=lambda item: float(item[1]), reverse=True)
        annotated = frame.copy()
        for box, conf, cls in candidates[:3]:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(annotated, f'{model.names[int(cls)]} {conf:.2f}',
                        (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 2)
        h, w = annotated.shape[:2]
        scale = min(480 / w, 320 / h)
        thumb = cv2.resize(annotated, (round(w * scale), round(h * scale)))
        tile = np.zeros((350, 500, 3), dtype=np.uint8)
        x = (500 - thumb.shape[1]) // 2
        y = 26 + (320 - thumb.shape[0]) // 2
        tile[y:y + thumb.shape[0], x:x + thumb.shape[1]] = thumb
        cv2.putText(tile, f'{class_id} frame {frame_id}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        tiles.append(tile)
    cap.release()
    sheet = np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, 9, 3)])
    cv2.imwrite(str(root / f'world_class_{class_id}.jpg'), sheet)
