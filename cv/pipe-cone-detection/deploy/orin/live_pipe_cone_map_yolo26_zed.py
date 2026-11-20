import argparse
import csv
import json
import math
import socket
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl
from ultralytics import YOLO


MIN_CONFIDENCE = 0.15
CLASS_NAMES = {0: "pipe_cone"}

CAMERA_RESOLUTIONS = {
    "hd2k": sl.RESOLUTION.HD2K,
    "hd1080": sl.RESOLUTION.HD1080,
    "hd720": sl.RESOLUTION.HD720,
    "vga": sl.RESOLUTION.VGA,
}

SUPPORTED_FPS = {
    "hd2k": {15},
    "hd1080": {15, 30},
    "hd720": {15, 30, 60},
    "vga": {15, 30, 60, 100},
}


@dataclass(frozen=True)
class ConePosition:
    camera_x_m: float
    camera_y_m: float
    camera_z_m: float
    range_m: float
    valid_pixels: int
    valid_fraction: float


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--weights",
        default="/home/jetson/pipe_cones/models/pipe_cone_yolo26l_best.pt",
    )
    parser.add_argument(
        "--output",
        default="/home/jetson/pipe_cones/logs/detector",
    )
    parser.add_argument("--conf", type=float, default=MIN_CONFIDENCE)
    parser.add_argument("--imgsz", type=int, default=960)
    parser.add_argument(
        "--resolution",
        choices=CAMERA_RESOLUTIONS,
        default="hd2k",
        help="HD2K maximizes distant-target detail and supports 15 FPS only",
    )
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--frames", type=int, default=0, help="0 runs until Q/Esc")
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument(
        "--manual-exposure",
        action="store_false",
        dest="auto_exposure",
        help="Disable auto exposure/gain and use --exposure/--gain",
    )
    parser.set_defaults(auto_exposure=True)
    parser.add_argument("--exposure", type=int, default=40, help="Manual ZED exposure 0..100")
    parser.add_argument("--gain", type=int, default=30, help="Manual ZED gain 0..100")
    parser.add_argument(
        "--clahe",
        type=float,
        default=1.5,
        help="Local contrast strength; 0 disables enhancement (recommended: 1.0-2.0)",
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument(
        "--observer-udp",
        default="127.0.0.1:15250",
        help="Observer UDP host:port; use an empty string to disable publishing",
    )
    args = parser.parse_args()
    if args.frames < 0 or args.warmup_frames < 0:
        parser.error("frame counts must be non-negative")
    if args.conf < MIN_CONFIDENCE:
        parser.error(f"--conf must be at least {MIN_CONFIDENCE:.2f}")
    if not 0 <= args.exposure <= 100 or not 0 <= args.gain <= 100:
        parser.error("--exposure and --gain must be between 0 and 100")
    if not 0.0 <= args.clahe <= 5.0:
        parser.error("--clahe must be between 0 and 5")
    if args.fps not in SUPPORTED_FPS[args.resolution]:
        allowed = ", ".join(map(str, sorted(SUPPORTED_FPS[args.resolution])))
        parser.error(f"--resolution {args.resolution} supports --fps values: {allowed}")
    return args


def pipe_cone_mask(bgr):
    """Select the low-saturation, bright body inside the detector box."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    saturation, value = hsv[:, :, 1], hsv[:, :, 2]
    return (saturation < 115) & (value > 65)


def improve_contrast(bgr, clip_limit):
    """Improve local luminance contrast without distorting cone colours."""
    if clip_limit <= 0.0:
        return bgr
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    lightness, channel_a, channel_b = cv2.split(lab)
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(8, 8))
    lightness = clahe.apply(lightness)
    return cv2.cvtColor(cv2.merge((lightness, channel_a, channel_b)), cv2.COLOR_LAB2BGR)


def measure_cone(xyz, image, xyxy):
    """Return a robust median 3D point on the visible cone surface."""
    height, width = image.shape[:2]
    x1, y1, x2, y2 = map(float, xyxy)
    x1, x2 = sorted((max(0.0, x1), min(float(width), x2)))
    y1, y2 = sorted((max(0.0, y1), min(float(height), y2)))

    left = int(x1 + 0.22 * (x2 - x1))
    right = int(x1 + 0.78 * (x2 - x1))
    top = int(y1 + 0.22 * (y2 - y1))
    bottom = int(y1 + 0.82 * (y2 - y1))
    roi = (left, top, right, bottom)
    if right - left < 3 or bottom - top < 3:
        return None, roi

    points_patch = xyz[top:bottom, left:right, :3]
    white = pipe_cone_mask(image[top:bottom, left:right])
    depth_valid = np.isfinite(points_patch).all(axis=2) & (points_patch[:, :, 2] > 0.0)
    valid = depth_valid & white
    if int(valid.sum()) < 12:
        valid = depth_valid
    fraction = float(valid.sum() / max(1, valid.size))
    points = points_patch[valid]
    if len(points) < 12 or fraction < 0.10:
        return None, roi

    z = points[:, 2]
    median_z = float(np.median(z))
    mad = float(np.median(np.abs(z - median_z)))
    points = points[np.abs(z - median_z) <= max(0.05, 3.0 * 1.4826 * mad)]
    if len(points) < 12:
        return None, roi

    x_m, y_m, z_m = np.median(points, axis=0)
    return ConePosition(
        camera_x_m=float(x_m),
        camera_y_m=float(y_m),
        camera_z_m=float(z_m),
        range_m=float(np.linalg.norm([x_m, y_m, z_m])),
        valid_pixels=int(len(points)),
        valid_fraction=fraction,
    ), roi


def open_camera(args):
    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = CAMERA_RESOLUTIONS[args.resolution]
    init.camera_fps = args.fps
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.depth_minimum_distance = 0.40
    init.depth_maximum_distance = 40.0
    init.camera_disable_self_calib = False
    error = camera.open(init)
    if error != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Could not open ZED camera: {error}")
    # Automatic exposure handles sun/shade transitions far better than one fixed
    # exposure/gain pair. Manual mode remains available for controlled lighting.
    camera.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 1 if args.auto_exposure else 0)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)
    if not args.auto_exposure:
        camera.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, args.exposure)
        camera.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, args.gain)
    return camera


def main():
    args = parse_args()
    cv2.setNumThreads(2)
    run_dir = Path(args.output) / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(args.weights, task="detect")
    camera = open_camera(args)
    runtime = sl.RuntimeParameters()
    runtime.confidence_threshold = 50
    runtime.texture_confidence_threshold = 100
    runtime.enable_fill_mode = False
    image, cloud = sl.Mat(), sl.Mat()
    info = camera.get_camera_information()
    window = "ZED pipe-cone observations - Q/Esc to close"
    analysed = 0
    started = time.monotonic()
    observer_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM) if args.observer_udp else None
    observer_address = None
    if observer_socket:
        host, port = args.observer_udp.rsplit(":", 1)
        observer_address = (host, int(port))

    settings = {
        "camera": str(info.camera_model),
        "serial": info.serial_number,
        "camera_resolution": args.resolution,
        "camera_fps": args.fps,
        "model_imgsz": args.imgsz,
        "detector_confidence": args.conf,
        "auto_exposure": args.auto_exposure,
        "manual_exposure": args.exposure if not args.auto_exposure else None,
        "manual_gain": args.gain if not args.auto_exposure else None,
        "clahe_clip_limit": args.clahe,
        "coordinate_assumptions": "ZED IMAGE coordinates: camera x=right, y=down, z=forward",
        "note": "Pixhawk fusion and geographic coordinates are produced by pipe_cone_mavlink_observer.py",
    }
    (run_dir / "settings.json").write_text(json.dumps(settings, indent=2))

    try:
        for _ in range(args.warmup_frames):
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.02)
        if not args.headless:
            cv2.namedWindow(window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(window, 1200, 720)

        with (run_dir / "pipe_cone_observations.csv").open("w", newline="") as output_file:
            log = csv.writer(output_file)
            log.writerow([
                "timestamp_ns", "frame", "class", "detection_confidence",
                "range_m", "camera_x_right_m", "camera_y_down_m", "camera_z_forward_m",
                "valid_fraction", "valid_pixels", "x1", "y1", "x2", "y2",
            ])

            while not args.frames or analysed < args.frames:
                if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                    time.sleep(0.02)
                    continue
                if camera.retrieve_image(image, sl.VIEW.LEFT) != sl.ERROR_CODE.SUCCESS:
                    continue
                if camera.retrieve_measure(cloud, sl.MEASURE.XYZRGBA) != sl.ERROR_CODE.SUCCESS:
                    continue

                raw_frame = image.get_data()[:, :, :3].copy()
                frame = improve_contrast(raw_frame, args.clahe)
                xyz = cloud.get_data()
                result = model.predict(frame, conf=max(args.conf, MIN_CONFIDENCE), imgsz=args.imgsz, device=0, verbose=False)[0]
                display = frame.copy()
                messages = []
                timestamp = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()

                for box in result.boxes:
                    class_id = int(box.cls.item())
                    if class_id != 0:
                        continue
                    xyxy = box.xyxy[0].cpu().numpy()
                    score = float(box.conf.item())
                    position, roi = measure_cone(xyz, frame, xyxy)
                    name = CLASS_NAMES.get(class_id, f"class{class_id}")
                    colour = (255, 80, 220)
                    x1, y1, x2, y2 = map(int, xyxy)
                    cv2.rectangle(display, (x1, y1), (x2, y2), colour, 2)
                    left, top, right, bottom = roi
                    cv2.rectangle(display, (left, top), (right, bottom), (0, 255, 255), 1)

                    if not position:
                        label = f"{name}: unreliable depth"
                    else:
                        label = f"{name}: {score:.2f}  {position.range_m:.2f} m"
                        log.writerow([
                            timestamp, analysed, name, score, position.range_m,
                            position.camera_x_m, position.camera_y_m, position.camera_z_m,
                            position.valid_fraction, position.valid_pixels, *xyxy.tolist(),
                        ])
                        if observer_socket:
                            observer_socket.sendto(json.dumps({
                                "timestamp_ns": timestamp,
                                "frame": analysed,
                                "class": name,
                                "confidence": score,
                                "range_m": position.range_m,
                                "camera_x_right_m": position.camera_x_m,
                                "camera_y_down_m": position.camera_y_m,
                                "camera_z_forward_m": position.camera_z_m,
                                "valid_fraction": position.valid_fraction,
                                "valid_pixels": position.valid_pixels,
                            }, separators=(",", ":")).encode("utf-8"), observer_address)
                        messages.append(
                            f"{name} conf={score:.2f} range={position.range_m:.2f}m "
                            f"x={position.camera_x_m:.2f}m z={position.camera_z_m:.2f}m"
                        )
                    cv2.putText(display, label, (max(0, x1), max(85, y1 - 10)), 0, 0.60, colour, 2)

                analysed += 1
                output_file.flush()
                fps = analysed / max(time.monotonic() - started, 1e-6)
                cv2.rectangle(display, (0, 0), (display.shape[1], 65), (25, 25, 25), -1)
                cv2.putText(display, f"Pipe-cone detector | {fps:.1f} FPS", (12, 25), 0, 0.65, (255, 255, 255), 2)
                cv2.putText(display, "Pixhawk fusion -> UDP 127.0.0.1:15250", (12, 51), 0, 0.55, (230, 230, 230), 1)
                if not messages:
                    cv2.putText(display, "No reliable cone position", (20, 100), 0, 0.70, (0, 255, 255), 2)
                if analysed == 1 or analysed % 30 == 0:
                    cv2.imwrite(str(run_dir / "latest_preview.jpg"), display)
                    print(f"frame={analysed} " + ("; ".join(messages) or "No reliable cone position"), flush=True)
                if not args.headless:
                    cv2.imshow(window, display)
                    if cv2.waitKey(1) & 255 in (27, ord("q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                        break
    finally:
        camera.close()
        if observer_socket:
            observer_socket.close()
        if not args.headless:
            cv2.destroyAllWindows()
    print(f"Logs: {run_dir}", flush=True)


if __name__ == "__main__":
    main()
