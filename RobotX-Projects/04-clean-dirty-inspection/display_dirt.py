"""Display live YOLO dirt detections from a camera or video file."""

from __future__ import annotations

import argparse
from pathlib import Path
import time

import cv2
from ultralytics import YOLO


WINDOW_NAME = "YOLO Dirt Detector"


def parse_source(value: str) -> int | str:
    """Treat a plain integer as a camera index; otherwise use a file/stream URL."""
    try:
        return int(value)
    except ValueError:
        return value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--source",
        default="0",
        help="camera index, video path, or stream URL (default: camera 0)",
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=Path(__file__).with_name("best.pt"),
        help="trained YOLO weights (default: best.pt beside this script)",
    )
    parser.add_argument("--confidence", type=float, default=0.90)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fullscreen", action="store_true")
    args = parser.parse_args()

    if not 0.90 <= args.confidence <= 1.0:
        parser.error("--confidence must be between 0.90 and 1.0")
    if not args.model.is_file():
        parser.error(f"model not found: {args.model}")

    source = parse_source(args.source)
    capture = cv2.VideoCapture(source)
    if isinstance(source, int):
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not capture.isOpened():
        print(f"Could not open source: {args.source}")
        return 2

    model = YOLO(str(args.model))
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    if args.fullscreen:
        cv2.setWindowProperty(WINDOW_NAME, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    previous_time = time.perf_counter()
    try:
        while True:
            ok, frame = capture.read()
            if not ok:
                break

            result = model.predict(frame, conf=args.confidence, verbose=False)[0]
            displayed = result.plot(labels=True, conf=True)
            dirt_detected = len(result.boxes) > 0

            now = time.perf_counter()
            fps = 1.0 / max(now - previous_time, 1e-6)
            previous_time = now

            status = "DIRT DETECTED" if dirt_detected else "CLEAN / NO DIRT"
            color = (0, 0, 255) if dirt_detected else (0, 200, 0)
            cv2.rectangle(displayed, (0, 0), (displayed.shape[1], 70), (0, 0, 0), -1)
            cv2.putText(
                displayed,
                status,
                (18, 45),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.1,
                color,
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                displayed,
                f"threshold >= {args.confidence:.2f}   FPS {fps:.1f}   Q/Esc to exit",
                (max(18, displayed.shape[1] - 550), 42),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

            cv2.imshow(WINDOW_NAME, displayed)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    finally:
        capture.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
