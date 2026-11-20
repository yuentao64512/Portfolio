from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLOE

ROOT = Path("/home/robotx/buoy_yolo26")
images = sorted((ROOT / "source_frames").glob("*.jpg"))[::8]
model = YOLOE("yoloe-26l-seg.pt")
model.set_classes([
    "white floating buoy",
    "floating pool buoy marker",
    "white trapezoid-shaped buoy",
])

results = model.predict([str(p) for p in images], imgsz=960, conf=0.08, iou=0.45,
                        device=0, half=True, verbose=False)
thumbs = []
for path, result in zip(images, results):
    plotted = result.plot(labels=True, conf=True)
    thumb = cv2.resize(plotted, (442, 249))
    cv2.putText(thumb, path.stem.split("_")[1], (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(thumb, path.stem.split("_")[1], (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (255, 255, 255), 1, cv2.LINE_AA)
    thumbs.append(thumb)

cols = 5
rows = int(np.ceil(len(thumbs) / cols))
sheet = np.full((rows * 249, cols * 442, 3), 245, dtype=np.uint8)
for i, thumb in enumerate(thumbs):
    row, col = divmod(i, cols)
    sheet[row * 249:(row + 1) * 249, col * 442:(col + 1) * 442] = thumb
cv2.imwrite(str(ROOT / "yoloe_proposals.jpg"), sheet, [cv2.IMWRITE_JPEG_QUALITY, 92])
for path, result in zip(images, results):
    n = 0 if result.boxes is None else len(result.boxes)
    print(path.name, n)
