from pathlib import Path
import cv2
import json
import numpy as np

root = Path('/home/robotx/box_size_training')
files = sorted((root / 'source_videos').glob('*.mp4'))
tiles = []
metadata = []
for path in files:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f'Cannot open {path}')
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = float(cap.get(cv2.CAP_PROP_FPS))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    metadata.append({'name': path.name, 'frames': count, 'fps': fps,
                     'duration_s': count / fps if fps else None,
                     'width': width, 'height': height,
                     'bytes': path.stat().st_size})
    for fraction in (0.05, 0.25, 0.5, 0.75, 0.95):
        cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, min(count - 1, int(fraction * count))))
        ok, frame = cap.read()
        if not ok:
            continue
        thumb = cv2.resize(frame, (384, 216))
        cv2.putText(thumb, f'{path.name[25:]} @ {fraction:.0%}', (8, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
        tiles.append(thumb)
    cap.release()
if not tiles:
    raise RuntimeError('No frames decoded')
rows = []
for start in range(0, len(tiles), 5):
    group = tiles[start:start + 5]
    rows.append(np.hstack(group + [np.zeros_like(group[0])] * (5 - len(group))))
sheet = np.vstack(rows)
cv2.imwrite(str(root / 'contact_sheet.jpg'), sheet)
print(json.dumps(metadata, indent=2))
