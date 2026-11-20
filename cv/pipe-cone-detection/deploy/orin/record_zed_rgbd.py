#!/usr/bin/env python3
"""Record synchronized ZED left video and metric depth frames."""

import argparse
import csv
import json
import queue
import shutil
import signal
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl


RESOLUTIONS = {
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


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="/home/jetson/pipe_cones/recordings")
    parser.add_argument("--name", default="", help="Session name (default: UTC timestamp)")
    parser.add_argument("--resolution", choices=RESOLUTIONS, default="hd2k")
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument("--duration", type=float, default=0, help="Seconds; 0 records until Ctrl+C/Q")
    parser.add_argument("--frames", type=int, default=0, help="Frame limit; 0 is unlimited")
    parser.add_argument("--depth-max-m", type=float, default=40.0)
    parser.add_argument("--png-compression", type=int, choices=range(10), default=1)
    parser.add_argument("--min-free-gb", type=float, default=10.0)
    parser.add_argument("--no-depth-preview-video", action="store_true")
    parser.add_argument("--preview", action="store_true", help="Show RGB/depth while recording")
    args = parser.parse_args()
    if args.fps not in SUPPORTED_FPS[args.resolution]:
        allowed = ", ".join(map(str, sorted(SUPPORTED_FPS[args.resolution])))
        parser.error(f"{args.resolution} supports FPS values: {allowed}")
    if args.duration < 0 or args.frames < 0 or args.depth_max_m <= 0 or args.min_free_gb < 0:
        parser.error("duration, frames, and min-free-gb must be non-negative; depth-max-m must be positive")
    return args


def depth_preview(depth_m, max_depth_m):
    """Make a display-only colour map. Saved PNG depth remains metric uint16."""
    valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m <= max_depth_m)
    scaled = np.zeros(depth_m.shape, dtype=np.uint8)
    scaled[valid] = np.clip(255.0 * depth_m[valid] / max_depth_m, 1, 255).astype(np.uint8)
    coloured = cv2.applyColorMap(255 - scaled, cv2.COLORMAP_TURBO)
    coloured[~valid] = 0
    return coloured


class RecordingWriter(threading.Thread):
    def __init__(self, items, run_dir, fps, size, max_depth_m, png_compression, preview_video):
        super().__init__(name="rgbd-writer", daemon=False)
        self.items = items
        self.run_dir = run_dir
        self.max_depth_m = max_depth_m
        self.png_compression = png_compression
        self.error = None
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.rgb_video = cv2.VideoWriter(str(run_dir / "rgb_left.mp4"), fourcc, fps, size)
        self.preview_video = (
            cv2.VideoWriter(str(run_dir / "depth_preview.mp4"), fourcc, fps, size)
            if preview_video else None
        )
        if not self.rgb_video.isOpened() or (self.preview_video and not self.preview_video.isOpened()):
            raise RuntimeError("OpenCV could not create an MP4 writer")

    def run(self):
        try:
            with (self.run_dir / "frames.csv").open("w", newline="") as output:
                log = csv.writer(output)
                log.writerow([
                    "frame", "zed_timestamp_ns", "host_utc", "rgb_video",
                    "depth_png", "valid_depth_pixels", "nearest_m", "median_m", "farthest_m",
                ])
                while True:
                    item = self.items.get()
                    if item is None:
                        self.items.task_done()
                        break
                    index, timestamp_ns, host_utc, bgr, depth_m = item
                    valid = np.isfinite(depth_m) & (depth_m > 0.0) & (depth_m <= self.max_depth_m)
                    depth_mm = np.zeros(depth_m.shape, dtype=np.uint16)
                    depth_mm[valid] = np.clip(depth_m[valid] * 1000.0, 1, 65535).astype(np.uint16)
                    filename = f"depth_{index:08d}.png"
                    if not cv2.imwrite(
                        str(self.run_dir / "depth_mm" / filename),
                        depth_mm,
                        [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression],
                    ):
                        raise RuntimeError(f"Failed to write {filename}")
                    self.rgb_video.write(bgr)
                    if self.preview_video:
                        self.preview_video.write(depth_preview(depth_m, self.max_depth_m))
                    values = depth_m[valid]
                    stats = (
                        (float(values.min()), float(np.median(values)), float(values.max()))
                        if values.size else ("", "", "")
                    )
                    log.writerow([
                        index, timestamp_ns, host_utc, "rgb_left.mp4", f"depth_mm/{filename}",
                        int(values.size), *stats,
                    ])
                    if index % 30 == 0:
                        output.flush()
                    self.items.task_done()
        except Exception as exc:  # relayed to capture thread after clean shutdown
            self.error = exc
        finally:
            self.rgb_video.release()
            if self.preview_video:
                self.preview_video.release()


def main():
    args = parse_args()
    session_name = args.name or datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S")
    run_dir = Path(args.output).expanduser() / session_name
    if run_dir.exists():
        raise SystemExit(f"Output already exists: {run_dir}")
    (run_dir / "depth_mm").mkdir(parents=True)

    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = RESOLUTIONS[args.resolution]
    init.camera_fps = args.fps
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.depth_mode = sl.DEPTH_MODE.NEURAL
    init.depth_minimum_distance = 0.4
    init.depth_maximum_distance = args.depth_max_m
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(
            f"Could not open ZED ({status}). Stop pipe-cone mapping first; only one process can own the camera."
        )

    camera.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 1)
    camera.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_AUTO, 1)
    info = camera.get_camera_information()
    config = info.camera_configuration.resolution
    width, height = int(config.width), int(config.height)
    settings = {
        "camera_model": str(info.camera_model),
        "serial_number": info.serial_number,
        "resolution": args.resolution,
        "width": width,
        "height": height,
        "fps": args.fps,
        "depth_mode": "NEURAL",
        "depth_unit_png": "millimetres; uint16; 0 means invalid/unavailable",
        "depth_max_m": args.depth_max_m,
        "rgb_video": "rgb_left.mp4 (rectified left camera)",
        "frame_pairing": "frames.csv associates video frame N with depth PNG N and ZED timestamp",
    }
    (run_dir / "metadata.json").write_text(json.dumps(settings, indent=2) + "\n")

    image, depth = sl.Mat(), sl.Mat()
    runtime = sl.RuntimeParameters()
    runtime.confidence_threshold = 50
    runtime.texture_confidence_threshold = 100
    items = queue.Queue(maxsize=max(2 * args.fps, 10))
    writer = RecordingWriter(
        items, run_dir, args.fps, (width, height), args.depth_max_m,
        args.png_compression, not args.no_depth_preview_video,
    )
    writer.start()
    stopping = False

    def request_stop(_signum=None, _frame=None):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    started = time.monotonic()
    count = 0
    print(f"Recording synchronized RGB + depth to {run_dir}", flush=True)
    print("Press Ctrl+C" + (" or Q in the preview" if args.preview else "") + " to stop cleanly.", flush=True)

    try:
        while not stopping:
            if args.frames and count >= args.frames:
                break
            if args.duration and time.monotonic() - started >= args.duration:
                break
            if writer.error:
                raise writer.error
            if count % max(args.fps * 10, 1) == 0:
                free_gb = shutil.disk_usage(run_dir).free / (1024 ** 3)
                if free_gb < args.min_free_gb:
                    print(f"Stopping: only {free_gb:.1f} GiB free (limit {args.min_free_gb:.1f} GiB).", flush=True)
                    break
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.005)
                continue
            if camera.retrieve_image(image, sl.VIEW.LEFT) != sl.ERROR_CODE.SUCCESS:
                continue
            if camera.retrieve_measure(depth, sl.MEASURE.DEPTH) != sl.ERROR_CODE.SUCCESS:
                continue
            bgr = image.get_data()[:, :, :3].copy()
            depth_m = depth.get_data().copy()
            timestamp_ns = camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds()
            host_utc = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
            while True:
                if writer.error:
                    raise writer.error
                try:
                    items.put((count, timestamp_ns, host_utc, bgr, depth_m), timeout=0.5)
                    break
                except queue.Full:
                    continue
            count += 1
            if count == 1 or count % args.fps == 0:
                elapsed = max(time.monotonic() - started, 1e-6)
                print(f"frames={count} elapsed={elapsed:.1f}s capture={count / elapsed:.1f} FPS", flush=True)
            if args.preview:
                preview_depth = depth_preview(depth_m, args.depth_max_m)
                display = np.hstack((bgr, preview_depth))
                scale = min(1.0, 1600 / display.shape[1], 800 / display.shape[0])
                if scale < 1.0:
                    display = cv2.resize(display, None, fx=scale, fy=scale)
                cv2.imshow("ZED RGB + depth recording - Q to stop", display)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
    finally:
        camera.close()
        if writer.is_alive():
            items.put(None)
            writer.join()
        if args.preview:
            cv2.destroyAllWindows()
    if writer.error:
        raise writer.error
    elapsed = time.monotonic() - started
    print(f"Saved {count} paired frames ({elapsed:.1f}s) to {run_dir}", flush=True)


if __name__ == "__main__":
    main()
