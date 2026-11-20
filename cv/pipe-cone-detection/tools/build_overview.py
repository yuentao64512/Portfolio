#!/usr/bin/env python3
from pathlib import Path

import cv2
import numpy as np


ROOT = Path("/home/robotx")
OUT = ROOT / "pipe_cones" / "review" / "video_overview.jpg"
VIDEOS = [ROOT / f"IMG_{n}.MOV" for n in range(7816, 7821)]
SAMPLES_PER_VIDEO = 12
THUMB_W, THUMB_H = 268, 480


def letterbox(frame: np.ndarray) -> np.ndarray:
    scale = min(THUMB_W / frame.shape[1], THUMB_H / frame.shape[0])
    resized = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    canvas = np.zeros((THUMB_H, THUMB_W, 3), dtype=np.uint8)
    y = (THUMB_H - resized.shape[0]) // 2
    x = (THUMB_W - resized.shape[1]) // 2
    canvas[y : y + resized.shape[0], x : x + resized.shape[1]] = resized
    return canvas


rows = []
for path in VIDEOS:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {path}")
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    indices = np.linspace(0, frame_count - 1, SAMPLES_PER_VIDEO + 2, dtype=int)[1:-1]
    thumbs = []
    for index in indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Could not read frame {index} from {path}")
        thumb = letterbox(frame)
        seconds = index / fps
        cv2.rectangle(thumb, (0, 0), (THUMB_W, 38), (0, 0, 0), -1)
        cv2.putText(
            thumb,
            f"{path.stem}  {seconds:.1f}s",
            (6, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        thumbs.append(thumb)
    cap.release()
    rows.append(np.hstack(thumbs))

OUT.parent.mkdir(parents=True, exist_ok=True)
cv2.imwrite(str(OUT), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 88])
print(OUT)
