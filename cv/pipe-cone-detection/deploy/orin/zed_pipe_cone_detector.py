#!/usr/bin/env python3
"""Run the trained pipe-cone detector on a Stereolabs ZED 2i."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO


DEFAULT_PT = Path("/home/jetson/pipe_cones/models/pipe_cone_yolo26l_best.pt")
DEFAULT_ENGINE = Path("/home/jetson/pipe_cones/models/pipe_cone_yolo26l_fp16.engine")
RESOLUTIONS = {
    "hd2k": sl.RESOLUTION.HD2K,
    "hd1080": sl.RESOLUTION.HD1080,
    "hd720": sl.RESOLUTION.HD720,
    "vga": sl.RESOLUTION.VGA,
}
VALID_FPS = {
    "hd2k": {15},
    "hd1080": {15, 30},
    "hd720": {15, 30, 60},
    "vga": {15, 30, 60, 100},
}


def parse_args() -> argparse.Namespace:
    default_weights = DEFAULT_ENGINE if DEFAULT_ENGINE.exists() else DEFAULT_PT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--weights", type=Path, default=default_weights)
    parser.add_argument("--resolution", choices=RESOLUTIONS, default="hd1080")
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument("--conf", type=float, default=0.20)
    parser.add_argument("--iou", type=float, default=0.60)
    parser.add_argument("--depth", action="store_true", help="Show median ZED range in each box")
    parser.add_argument("--headless", action="store_true", help="Do not open a display window")
    parser.add_argument("--frames", type=int, default=0, help="Stop after N frames; 0 runs until Q/Esc")
    parser.add_argument("--output", type=Path, help="Optional annotated MP4 output path")
    args = parser.parse_args()
    if not args.weights.is_file():
        parser.error(f"weights do not exist: {args.weights}")
    if args.fps not in VALID_FPS[args.resolution]:
        parser.error(f"{args.resolution} supports FPS values {sorted(VALID_FPS[args.resolution])}")
    if not 0.0 < args.conf <= 1.0 or not 0.0 < args.iou <= 1.0:
        parser.error("--conf and --iou must be in (0, 1]")
    if args.frames < 0:
        parser.error("--frames must be non-negative")
    return args


def open_zed(args: argparse.Namespace) -> sl.Camera:
    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = RESOLUTIONS[args.resolution]
    init.camera_fps = args.fps
    init.coordinate_units = sl.UNIT.METER
    init.depth_mode = sl.DEPTH_MODE.NEURAL if args.depth else sl.DEPTH_MODE.NONE
    if args.depth:
        init.depth_minimum_distance = 0.4
        init.depth_maximum_distance = 40.0
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Could not open ZED 2i: {status}")
    camera.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 1)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)
    return camera


def median_range(depth: np.ndarray, xyxy: np.ndarray) -> float | None:
    """Estimate target range from the central body region, rejecting invalid depth."""
    height, width = depth.shape[:2]
    x1, y1, x2, y2 = map(float, xyxy)
    x1, x2 = sorted((max(0.0, x1), min(float(width), x2)))
    y1, y2 = sorted((max(0.0, y1), min(float(height), y2)))
    left = int(x1 + 0.25 * (x2 - x1))
    right = int(x1 + 0.75 * (x2 - x1))
    top = int(y1 + 0.25 * (y2 - y1))
    bottom = int(y1 + 0.80 * (y2 - y1))
    values = depth[top:bottom, left:right]
    values = values[np.isfinite(values) & (values > 0.3) & (values < 40.0)]
    if values.size < 12:
        return None
    median = float(np.median(values))
    mad = float(np.median(np.abs(values - median)))
    filtered = values[np.abs(values - median) <= max(0.10, 3.0 * 1.4826 * mad)]
    return float(np.median(filtered)) if filtered.size >= 12 else None


def main() -> None:
    args = parse_args()
    cv2.setNumThreads(2)
    model = YOLO(str(args.weights), task="detect")
    camera = open_zed(args)
    runtime = sl.RuntimeParameters()
    image_mat = sl.Mat()
    depth_mat = sl.Mat()
    writer = None
    frame_count = 0
    fps_ema = 0.0
    window = "ZED 2i pipe-cone detector - Q/Esc to quit"

    try:
        while args.frames == 0 or frame_count < args.frames:
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue
            started = time.perf_counter()
            camera.retrieve_image(image_mat, sl.VIEW.LEFT)
            frame = cv2.cvtColor(image_mat.get_data(), cv2.COLOR_BGRA2BGR)
            depth = None
            if args.depth:
                camera.retrieve_measure(depth_mat, sl.MEASURE.DEPTH)
                depth = depth_mat.get_data()

            result = model.predict(
                frame,
                imgsz=args.imgsz,
                conf=args.conf,
                iou=args.iou,
                device=0,
                verbose=False,
            )[0]
            detections = 0
            for box in result.boxes:
                xyxy = box.xyxy[0].detach().cpu().numpy()
                x1, y1, x2, y2 = map(int, xyxy)
                confidence = float(box.conf[0])
                label = f"pipe_cone {confidence:.2f}"
                if depth is not None:
                    distance = median_range(depth, xyxy)
                    if distance is not None and math.isfinite(distance):
                        label += f"  {distance:.1f} m"
                color = (0, 255, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                text_y = max(24, y1 - 8)
                cv2.putText(
                    frame, label, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, color, 2, cv2.LINE_AA,
                )
                detections += 1

            elapsed = max(time.perf_counter() - started, 1e-6)
            instantaneous_fps = 1.0 / elapsed
            fps_ema = instantaneous_fps if fps_ema == 0.0 else 0.9 * fps_ema + 0.1 * instantaneous_fps
            status = f"FPS {fps_ema:.1f}  detections {detections}  conf {args.conf:.2f}"
            cv2.rectangle(frame, (0, 0), (min(frame.shape[1], 620), 42), (0, 0, 0), -1)
            cv2.putText(frame, status, (10, 29), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                        (255, 255, 255), 2, cv2.LINE_AA)

            if args.output is not None:
                if writer is None:
                    args.output.parent.mkdir(parents=True, exist_ok=True)
                    writer = cv2.VideoWriter(
                        str(args.output), cv2.VideoWriter_fourcc(*"mp4v"),
                        float(args.fps), (frame.shape[1], frame.shape[0]),
                    )
                    if not writer.isOpened():
                        raise RuntimeError(f"Could not open video output: {args.output}")
                writer.write(frame)

            if not args.headless:
                cv2.imshow(window, frame)
                if cv2.waitKey(1) & 0xFF in (27, ord("q"), ord("Q")):
                    break
            frame_count += 1
            if args.headless and frame_count % 30 == 0:
                print(status, flush=True)
    finally:
        if writer is not None:
            writer.release()
        camera.close()
        cv2.destroyAllWindows()
    print(f"Processed {frame_count} frames; average live FPS {fps_ema:.1f}")


if __name__ == "__main__":
    main()
