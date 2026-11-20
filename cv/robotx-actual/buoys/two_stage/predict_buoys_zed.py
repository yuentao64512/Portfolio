#!/usr/bin/env python3
"""Live ZED preview using the trained Model A buoy detector only."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2
import pyzed.sl as sl
from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
DEFAULT_ENGINE = ROOT / "models" / "model_a_best.engine"
DEFAULT_PT = ROOT / "models" / "model_a_best.pt"


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Model A on a live ZED camera")
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_ENGINE if DEFAULT_ENGINE.is_file() else DEFAULT_PT,
    )
    parser.add_argument("--resolution", choices=("hd2k", "hd1080", "hd720"), default="hd2k")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--iou", type=float, default=0.50)
    parser.add_argument("--save", type=Path, help="optional annotated MP4 output")
    return parser.parse_args()


def main() -> int:
    args = arguments()
    if not args.model.is_file():
        raise SystemExit(f"Model not found: {args.model}")

    resolutions = {
        "hd2k": sl.RESOLUTION.HD2K,
        "hd1080": sl.RESOLUTION.HD1080,
        "hd720": sl.RESOLUTION.HD720,
    }

    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = resolutions[args.resolution]
    init.camera_fps = args.fps
    init.depth_mode = sl.DEPTH_MODE.NONE
    init.coordinate_units = sl.UNIT.METER

    error = camera.open(init)
    if error != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"Cannot open ZED camera: {error}")

    model = YOLO(str(args.model), task="detect")
    image = sl.Mat()
    runtime = sl.RuntimeParameters()
    writer = None
    previous = time.perf_counter()

    print(f"Model: {args.model}")
    print(f"Camera: ZED {args.resolution.upper()} @ {args.fps} FPS")
    print("Press Q or Esc in the preview window to stop.")

    try:
        while True:
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue

            camera.retrieve_image(image, sl.VIEW.LEFT)
            frame = cv2.cvtColor(image.get_data(), cv2.COLOR_BGRA2BGR)
            result = model.predict(
                source=frame,
                imgsz=args.imgsz,
                conf=args.conf,
                iou=args.iou,
                device=0,
                verbose=False,
            )[0]
            annotated = result.plot()

            now = time.perf_counter()
            fps = 1.0 / max(now - previous, 1e-6)
            previous = now
            count = 0 if result.boxes is None else len(result.boxes)
            cv2.putText(
                annotated,
                f"detections={count}  FPS={fps:.1f}",
                (20, 42),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

            if args.save is not None:
                if writer is None:
                    args.save.parent.mkdir(parents=True, exist_ok=True)
                    height, width = annotated.shape[:2]
                    writer = cv2.VideoWriter(
                        str(args.save),
                        cv2.VideoWriter_fourcc(*"mp4v"),
                        float(args.fps),
                        (width, height),
                    )
                    if not writer.isOpened():
                        raise RuntimeError(f"Cannot create video: {args.save}")
                writer.write(annotated)

            cv2.imshow("Live buoy prediction - Model A", annotated)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if writer is not None:
            writer.release()
        camera.close()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
