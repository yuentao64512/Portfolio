#!/usr/bin/env python3
import argparse
from pathlib import Path

import cv2
import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("video", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--columns", type=int, default=6)
    args = parser.parse_args()

    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {args.video}")
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    indices = np.linspace(0, count - 1, args.samples, dtype=int)
    thumbs = []
    for index in indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(index))
        ok, frame = cap.read()
        if not ok:
            raise RuntimeError(f"Could not read frame {index}")
        scale = 320 / frame.shape[1]
        thumb = cv2.resize(frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        cv2.rectangle(thumb, (0, 0), (thumb.shape[1], 35), (0, 0, 0), -1)
        cv2.putText(
            thumb,
            f"f{index}  {index / fps:.1f}s",
            (6, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        thumbs.append(thumb)
    cap.release()

    while len(thumbs) % args.columns:
        thumbs.append(np.zeros_like(thumbs[0]))
    rows = [
        np.hstack(thumbs[i : i + args.columns])
        for i in range(0, len(thumbs), args.columns)
    ]
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), np.vstack(rows), [cv2.IMWRITE_JPEG_QUALITY, 92])
    print(args.output)


if __name__ == "__main__":
    main()
