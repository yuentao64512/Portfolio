#!/usr/bin/env python3
"""Two-stage buoy and digit prediction with tracking, voting, video and CSV output."""

from __future__ import annotations

import argparse
import csv
from collections import defaultdict, deque
from datetime import datetime
from pathlib import Path

import cv2
from ultralytics import YOLO


PROJECT = Path(__file__).resolve().parent
NEW_MODELS = PROJECT / "retrain_robotx_do_not_touch_20261110" / "deploy" / "models"
OLD_MODELS = PROJECT / "deploy" / "models"
BUNDLED_MODELS = PROJECT / "models"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="/home/robotx/robotx_do_not_touch/IMG_3924.MOV",
                        help="video/image/folder, camera index, or zed")
    parser.add_argument("--buoy", type=Path, help="buoy .pt or .engine; newest retrained weight is automatic")
    parser.add_argument("--digit", type=Path, help="digit .pt or .engine; newest retrained weight is automatic")
    parser.add_argument("--output", type=Path, help="output directory")
    parser.add_argument("--buoy-conf", type=float, default=0.35)
    parser.add_argument("--digit-conf", type=float, default=0.20)
    parser.add_argument("--history", type=int, default=15)
    parser.add_argument("--min-score", type=float, default=0.60)
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--no-save", action="store_true")
    return parser.parse_args()


class ZedSource:
    fps = 15.0
    is_images = False

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
            return False, None, "zed"
        self.camera.retrieve_image(self.image, self.sl.VIEW.LEFT)
        frame = cv2.cvtColor(self.image.get_data(), cv2.COLOR_BGRA2BGR)
        return True, frame, "zed"

    def release(self):
        self.camera.close()


class CaptureSource:
    is_images = False

    def __init__(self, source: str):
        target = int(source) if source.isdigit() else source
        self.capture = cv2.VideoCapture(target)
        if not self.capture.isOpened():
            raise RuntimeError(f"Cannot open source: {source}")
        self.fps = self.capture.get(cv2.CAP_PROP_FPS)
        if not 1 <= self.fps <= 240:
            self.fps = 30.0
        self.name = Path(source).stem if not source.isdigit() else f"camera{source}"

    def read(self):
        ok, frame = self.capture.read()
        return ok, frame, self.name

    def release(self):
        self.capture.release()


class ImageSource:
    fps = 1.0
    is_images = True

    def __init__(self, source: Path):
        if source.is_dir():
            self.paths = sorted(path for path in source.iterdir() if path.suffix.lower() in IMAGE_SUFFIXES)
        elif source.suffix.lower() in IMAGE_SUFFIXES:
            self.paths = [source]
        else:
            self.paths = []
        if not self.paths:
            raise RuntimeError(f"No supported images: {source}")
        self.index = 0

    def read(self):
        if self.index >= len(self.paths):
            return False, None, ""
        path = self.paths[self.index]
        self.index += 1
        frame = cv2.imread(str(path))
        if frame is None:
            raise RuntimeError(f"Cannot read image: {path}")
        return True, frame, path.stem

    def release(self):
        pass


def open_source(value: str):
    if value.lower() == "zed":
        return ZedSource()
    path = Path(value)
    if path.is_dir() or path.suffix.lower() in IMAGE_SUFFIXES:
        return ImageSource(path)
    return CaptureSource(value)


def choose_weight(requested: Path | None, filename: str) -> Path:
    if requested is not None:
        weight = requested
    elif (BUNDLED_MODELS / filename).is_file():
        weight = BUNDLED_MODELS / filename
    elif (NEW_MODELS / filename).is_file():
        weight = NEW_MODELS / filename
    else:
        weight = OLD_MODELS / filename
    if not weight.is_file():
        raise FileNotFoundError(weight)
    return weight


def main() -> int:
    args = arguments()
    buoy_weight = choose_weight(args.buoy, "buoys_best.pt")
    digit_weight = choose_weight(args.digit, "digits_best.pt")
    output = args.output or PROJECT / "predictions" / datetime.now().strftime("%Y%m%d_%H%M%S")
    if not args.no_save:
        output.mkdir(parents=True, exist_ok=False)
        (output / "images").mkdir()

    print(f"buoy_model={buoy_weight}")
    print(f"digit_model={digit_weight}")
    print(f"output={output if not args.no_save else 'disabled'}")
    buoy_model = YOLO(str(buoy_weight), task="detect")
    digit_model = YOLO(str(digit_weight), task="detect")
    source = open_source(args.source)
    history = defaultdict(lambda: deque(maxlen=args.history))
    writer = None
    csv_file = None
    csv_writer = None
    frame_index = 0

    if not args.no_save:
        csv_file = (output / "detections.csv").open("w", newline="")
        csv_writer = csv.DictWriter(csv_file, fieldnames=(
            "frame", "source", "track_id", "buoy_confidence", "number",
            "number_score", "x1", "y1", "x2", "y2"
        ))
        csv_writer.writeheader()

    try:
        while True:
            ok, frame, frame_name = source.read()
            if not ok:
                if isinstance(source, ZedSource):
                    continue
                break
            height, width = frame.shape[:2]
            tracked = buoy_model.track(
                frame, persist=not source.is_images, conf=args.buoy_conf,
                imgsz=960, tracker="bytetrack.yaml", verbose=False
            )[0]
            items, crops = [], []
            if tracked.boxes is not None:
                for index, box in enumerate(tracked.boxes):
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(width, x2), min(height, y2)
                    pad_x, pad_y = int((x2-x1)*0.10), int((y2-y1)*0.10)
                    ox, oy = max(0, x1-pad_x), max(0, y1-pad_y)
                    ex, ey = min(width, x2+pad_x), min(height, y2+pad_y)
                    crop = frame[oy:ey, ox:ex]
                    if crop.size == 0:
                        continue
                    track_id = int(box.id.item()) if box.id is not None else frame_index*10000 + index
                    items.append((track_id, float(box.conf.item()), (x1, y1, x2, y2), (ox, oy), crop))
                    crops.append(crop)

            digit_results = []
            for start in range(0, len(crops), 8):
                digit_results.extend(digit_model.predict(
                    crops[start:start+8], imgsz=320, conf=args.digit_conf,
                    batch=8, verbose=False
                ))

            outputs = []
            for item, digit_result in zip(items, digit_results):
                track_id, buoy_confidence, (x1, y1, x2, y2), (ox, oy), _ = item
                frame_votes = []
                if digit_result.boxes is not None:
                    for digit_box in digit_result.boxes:
                        a, b, c, d = digit_box.xyxy[0].tolist()
                        center_x, center_y = ox+(a+c)/2, oy+(b+d)/2
                        if x1 <= center_x <= x2 and y1 <= center_y <= y2:
                            frame_votes.append((int(digit_box.cls.item()), float(digit_box.conf.item())))
                history[track_id].append(frame_votes)
                all_votes = [vote for group in history[track_id] for vote in group]
                scores = defaultdict(float)
                for digit, confidence in all_votes:
                    scores[digit] += confidence
                number, score = max(scores.items(), key=lambda pair: pair[1]) if scores else (None, 0.0)
                if score < args.min_score:
                    number = None
                result = {
                    "frame": frame_index, "source": frame_name, "track_id": track_id,
                    "buoy_confidence": round(buoy_confidence, 4), "number": number,
                    "number_score": round(score, 4), "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                }
                outputs.append(result)
                if csv_writer:
                    csv_writer.writerow(result)

            for result in outputs:
                x1, y1, x2, y2 = result["x1"], result["y1"], result["x2"], result["y2"]
                number_text = "?" if result["number"] is None else str(result["number"])
                label = f'id{result["track_id"]} #{number_text} {result["number_score"]:.2f}'
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 3)
                cv2.putText(frame, label, (x1, max(25, y1-8)), cv2.FONT_HERSHEY_SIMPLEX,
                            0.75, (0, 255, 0), 2, cv2.LINE_AA)

            if not args.no_save:
                if source.is_images:
                    cv2.imwrite(str(output / "images" / f"{frame_name}.jpg"), frame)
                else:
                    if writer is None:
                        writer = cv2.VideoWriter(
                            str(output / "annotated.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                            source.fps, (width, height)
                        )
                        if not writer.isOpened():
                            raise RuntimeError("Cannot create output video")
                    writer.write(frame)
            if args.show:
                cv2.imshow("Buoy number prediction", frame)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            frame_index += 1
    finally:
        source.release()
        if writer is not None:
            writer.release()
        if csv_file is not None:
            csv_file.close()
        cv2.destroyAllWindows()
    print(f"frames={frame_index}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
