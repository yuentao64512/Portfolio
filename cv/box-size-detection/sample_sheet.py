from pathlib import Path
import cv2
import numpy as np

root = Path('/home/robotx/box_size_training')
files = sorted((root / 'source_videos').glob('*.mp4'))
for class_id, path in enumerate(files):
    cap = cv2.VideoCapture(str(path))
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    tiles = []
    for index in range(18):
        frame_id = int((index + 0.5) * count / 18)
        cap.set(cv2.CAP_PROP_POS_FRAMES, frame_id)
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f'Cannot read {path.name} frame {frame_id}')
        h, w = frame.shape[:2]
        scale = min(350 / w, 250 / h)
        resized = cv2.resize(frame, (round(w * scale), round(h * scale)))
        tile = np.zeros((280, 370, 3), dtype=np.uint8)
        x = (370 - resized.shape[1]) // 2
        y = 26 + (250 - resized.shape[0]) // 2
        tile[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
        cv2.putText(tile, f'{class_id}: frame {frame_id}', (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        tiles.append(tile)
    cap.release()
    sheet = np.vstack([np.hstack(tiles[i:i + 6]) for i in range(0, 18, 6)])
    cv2.imwrite(str(root / f'class_{class_id}_samples.jpg'), sheet,
                [cv2.IMWRITE_JPEG_QUALITY, 90])
