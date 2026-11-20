#!/usr/bin/env python3
"""Two-stage tracked buoy-number recognition for ZED or recorded video."""

from __future__ import annotations

import argparse
from collections import defaultdict, deque
from pathlib import Path

import cv2
from ultralytics import YOLO


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="zed", help="zed, camera index, image, or video")
    parser.add_argument("--buoy", type=Path, default=Path("models/buoys_best.engine"))
    parser.add_argument("--digit", type=Path, default=Path("models/digits_best.engine"))
    parser.add_argument("--buoy-conf", type=float, default=0.50)
    parser.add_argument("--digit-conf", type=float, default=0.35)
    parser.add_argument("--history", type=int, default=15)
    parser.add_argument("--min-score", type=float, default=1.0)
    parser.add_argument("--no-show", action="store_true")
    return parser.parse_args()


class ZedFrames:
    def __init__(self):
        import pyzed.sl as sl
        self.sl = sl
        self.camera = sl.Camera()
        init = sl.InitParameters()
        init.camera_resolution = sl.RESOLUTION.HD2K
        init.camera_fps = 15
        init.depth_mode = sl.DEPTH_MODE.NONE
        status = self.camera.open(init)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Cannot open ZED: {status}")
        self.runtime = sl.RuntimeParameters()
        self.image = sl.Mat()

    def read(self):
        if self.camera.grab(self.runtime) != self.sl.ERROR_CODE.SUCCESS:
            return False, None
        self.camera.retrieve_image(self.image, self.sl.VIEW.LEFT)
        return True, cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)

    def release(self):
        self.camera.close()


def classify_colour(_crop):
    """Placeholder for future red/blue top-marker classification."""
    return None


def main() -> int:
    args = arguments()
    buoy_model = YOLO(str(args.buoy), task="detect")
    digit_model = YOLO(str(args.digit), task="detect")
    votes = defaultdict(lambda: deque(maxlen=args.history))
    source = ZedFrames() if args.source.lower() == "zed" else cv2.VideoCapture(
        int(args.source) if args.source.isdigit() else args.source
    )
    if not isinstance(source, ZedFrames) and not source.isOpened():
        raise SystemExit(f"Cannot open {args.source}")

    try:
        while True:
            ok, frame = source.read()
            if not ok:
                if isinstance(source, ZedFrames):
                    continue
                break
            height, width = frame.shape[:2]
            tracked = buoy_model.track(
                frame, persist=True, conf=args.buoy_conf, imgsz=960,
                tracker="bytetrack.yaml", verbose=False
            )[0]
            items, crops = [], []
            if tracked.boxes is not None:
                for box in tracked.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    pad_x, pad_y = int((x2-x1)*0.10), int((y2-y1)*0.10)
                    ox, oy = max(0, x1-pad_x), max(0, y1-pad_y)
                    ex, ey = min(width, x2+pad_x), min(height, y2+pad_y)
                    crop = frame[oy:ey, ox:ex]
                    if crop.size == 0:
                        continue
                    track_id = int(box.id.item()) if box.id is not None else None
                    items.append((track_id, (x1, y1, x2, y2), (ox, oy), crop))
                    crops.append(crop)

            digit_results = []
            for start in range(0, len(crops), 8):
                digit_results.extend(digit_model.predict(
                    crops[start:start+8], imgsz=320, conf=args.digit_conf,
                    batch=8, verbose=False
                ))

            outputs = []
            for item, digit_result in zip(items, digit_results):
                track_id, (x1, y1, x2, y2), (ox, oy), crop = item
                frame_votes = []
                if digit_result.boxes is not None:
                    for digit_box in digit_result.boxes:
                        a, b, c, d = digit_box.xyxy[0].tolist()
                        center_x, center_y = ox+(a+c)/2, oy+(b+d)/2
                        if x1 <= center_x <= x2 and y1 <= center_y <= y2:
                            frame_votes.append((int(digit_box.cls.item()), float(digit_box.conf.item())))
                if track_id is not None:
                    votes[track_id].append(frame_votes)
                    all_votes = [vote for group in votes[track_id] for vote in group]
                else:
                    all_votes = frame_votes
                scores = defaultdict(float)
                for digit, confidence in all_votes:
                    scores[digit] += confidence
                number, score = max(scores.items(), key=lambda pair: pair[1]) if scores else (None, 0.0)
                if score < args.min_score:
                    number = None
                outputs.append({
                    "id": track_id, "number": number, "score": round(score, 2),
                    "colour": classify_colour(crop), "box": (x1, y1, x2, y2)
                })

            if not args.no_show:
                for result in outputs:
                    x1, y1, x2, y2 = result["box"]
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame, f'id{result["id"]} #{result["number"]} ({result["score"]})',
                        (x1, max(20, y1-6)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                        (0, 255, 0), 2
                    )
                cv2.imshow("buoys", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        source.release()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
