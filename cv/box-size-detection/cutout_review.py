from pathlib import Path
import cv2
import numpy as np

root = Path('/home/robotx/box_size_training')
tiles = []
for name in ('small', 'medium', 'large'):
    paths = sorted((root / 'cutouts_v3' / name).glob('*.png'))
    for path in paths:
        image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
        height, width = image.shape[:2]
        scale = min(180 / width, 180 / height)
        image = cv2.resize(image, (max(1, round(width * scale)),
                                   max(1, round(height * scale))))
        tile = np.full((210, 200, 3), 160, dtype=np.uint8)
        x = (200 - image.shape[1]) // 2
        y = 24 + (180 - image.shape[0]) // 2
        alpha = image[:, :, 3:4].astype(np.float32) / 255
        roi = tile[y:y + image.shape[0], x:x + image.shape[1]]
        roi[:] = (roi * (1 - alpha) + image[:, :, :3] * alpha).astype(np.uint8)
        cv2.putText(tile, path.stem, (4, 17), cv2.FONT_HERSHEY_SIMPLEX,
                    0.46, (255, 255, 255), 1)
        tiles.append(tile)
sheet = np.vstack([np.hstack(tiles[i:i + 8]) for i in range(0, len(tiles), 8)])
cv2.imwrite(str(root / 'cutouts_v3_review.jpg'), sheet)
