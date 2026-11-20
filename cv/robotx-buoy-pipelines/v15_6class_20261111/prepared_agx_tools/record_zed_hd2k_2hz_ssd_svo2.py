#!/home/optiprime-agxorin/actual_buoy/.venv/bin/python
"""Record ZED2i HD2K JPGs at 2 Hz plus a full-session H.265 SVO2.

The RobotX SSD is verified by block device and UUID. The program never falls
back to the AGX internal drive. With --save-depth-png, each saved JPG also gets
an aligned 16-bit depth PNG in millimetres with the same filename stem.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl


EXPECTED_DEVICE = "/dev/sda2"
EXPECTED_UUID = "76A8-A5A6"
WIDTH = 2208
HEIGHT = 1242
CAMERA_FPS = 15
SAVE_RATE_HZ = 2.0


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-root",
        type=Path,
        help="optional directory on the mounted SSD; default: <SSD>/buoy_recordings_2hz",
    )
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="recording duration; 0 records until Q/Esc/Ctrl+C")
    parser.add_argument("--jpeg-quality", type=int, default=95)
    parser.add_argument("--delay-seconds", type=int, default=0,
                        help="countdown after Enter; default 0")
    parser.add_argument("--no-show", action="store_true")
    parser.add_argument("--preview-width", type=int, default=960)
    parser.add_argument("--save-depth-png", action="store_true",
                        help="save aligned uint16 depth PNG in millimetres per JPG")
    parser.add_argument("--depth-mode", choices=("performance", "quality", "neural"),
                        default="neural", help="used only with --save-depth-png")
    parser.add_argument("--min-depth-mm", type=float, default=200.0)
    parser.add_argument("--max-depth-mm", type=float, default=65000.0)
    return parser.parse_args()


def verify_ssd(output_root: Path | None) -> tuple[Path, Path, float]:
    if not Path(EXPECTED_DEVICE).is_block_device():
        raise SystemExit(f"RobotX SSD is not connected as {EXPECTED_DEVICE}")
    uuid_result = subprocess.run(
        ["lsblk", "-dn", "-o", "UUID", EXPECTED_DEVICE],
        check=True, text=True, capture_output=True,
    )
    actual_uuid = uuid_result.stdout.strip()
    if actual_uuid != EXPECTED_UUID:
        raise SystemExit(
            f"Wrong SSD: expected UUID {EXPECTED_UUID}, found {actual_uuid or 'nothing'}"
        )
    mount_result = subprocess.run(
        ["findmnt", "-rn", "-S", EXPECTED_DEVICE, "-o", "TARGET"],
        check=False, text=True, capture_output=True,
    )
    mount_text = mount_result.stdout.strip()
    if mount_result.returncode != 0 or not mount_text:
        raise SystemExit(f"{EXPECTED_DEVICE} is connected but not mounted")
    mount = Path(mount_text).resolve()
    root = output_root.expanduser().resolve() if output_root else mount / "buoy_recordings_2hz"
    if not root.is_relative_to(mount):
        raise SystemExit(f"Output must remain on the RobotX SSD under {mount}")
    if not os.access(mount, os.W_OK):
        raise SystemExit(f"SSD is not writable: {mount}")
    free_gib = shutil.disk_usage(mount).free / (1024 ** 3)
    if free_gib < 10.0:
        raise SystemExit(f"SSD has only {free_gib:.1f} GiB free; 10 GiB is required")
    return root, mount, free_gib


def depth_mode(name: str):
    return {
        "performance": sl.DEPTH_MODE.PERFORMANCE,
        "quality": sl.DEPTH_MODE.QUALITY,
        "neural": sl.DEPTH_MODE.NEURAL,
    }[name]


def write_depth_png(path: Path, values_mm: np.ndarray,
                    minimum_mm: float, maximum_mm: float) -> tuple[int, float]:
    values = np.asarray(values_mm)
    valid = np.isfinite(values) & (values >= minimum_mm) & (values <= maximum_mm)
    encoded = np.zeros(values.shape, dtype=np.uint16)
    encoded[valid] = np.rint(values[valid]).astype(np.uint16)
    temporary = path.with_name(f".{path.name}")
    if not cv2.imwrite(str(temporary), encoded, [cv2.IMWRITE_PNG_COMPRESSION, 3]):
        raise RuntimeError(f"Failed to write depth image: {temporary}")
    temporary.replace(path)
    return int(valid.sum()), float(valid.mean())


def main() -> int:
    args = arguments()
    if not 1 <= args.jpeg_quality <= 100:
        raise SystemExit("--jpeg-quality must be between 1 and 100")
    if args.delay_seconds < 0 or args.seconds < 0:
        raise SystemExit("delay and duration cannot be negative")
    if not 0 <= args.min_depth_mm < args.max_depth_mm <= 65535:
        raise SystemExit("depth range must satisfy 0 <= min < max <= 65535 mm")

    output_root, ssd_mount, free_gib = verify_ssd(args.output_root)
    print(f"ssd_device={EXPECTED_DEVICE}")
    print(f"ssd_uuid={EXPECTED_UUID}")
    print(f"ssd_mount={ssd_mount}")
    print(f"ssd_free={free_gib:.1f} GiB")
    input("Press Enter to arm recording...")
    if args.delay_seconds:
        deadline = time.monotonic() + args.delay_seconds
        last = None
        while True:
            remaining = max(0, int(deadline - time.monotonic() + 0.999))
            if remaining == 0:
                break
            if remaining <= 5 or remaining % 10 == 0:
                if remaining != last:
                    print(f"countdown={remaining}", flush=True)
                    last = remaining
            time.sleep(0.1)

    # Re-check after the interactive delay so a disconnected SSD can never
    # cause data to land on internal storage.
    output_root, ssd_mount, free_gib = verify_ssd(args.output_root)
    local_start = datetime.now().astimezone()
    session_name = local_start.strftime("%Y%m%d_%H%M%S")
    session = output_root / session_name
    images_dir = session / "images"
    depth_dir = session / "depth" if args.save_depth_png else None
    images_dir.mkdir(parents=True, exist_ok=False)
    if depth_dir:
        depth_dir.mkdir(parents=True, exist_ok=False)
    svo_path = session / f"zed2i_{session_name}.svo2"

    camera = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD2K
    init.camera_fps = CAMERA_FPS
    init.coordinate_units = sl.UNIT.MILLIMETER
    init.depth_mode = depth_mode(args.depth_mode) if args.save_depth_png else sl.DEPTH_MODE.NONE
    status = camera.open(init)
    if status != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"Cannot open ZED2i: {status}")

    recording_enabled = False
    manifest_handle = None
    sequence = 0
    started = time.monotonic()
    try:
        recording = sl.RecordingParameters(str(svo_path), sl.SVO_COMPRESSION_MODE.H265)
        record_status = camera.enable_recording(recording)
        if record_status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Cannot start H.265 SVO2 recording: {record_status}")
        recording_enabled = True

        info = camera.get_camera_information()
        metadata = {
            "status": "recording",
            "camera": "ZED 2i",
            "camera_serial": int(info.serial_number),
            "camera_mode": "HD2K@15",
            "saved_resolution": [WIDTH, HEIGHT],
            "saved_rate_hz": SAVE_RATE_HZ,
            "jpeg_quality": args.jpeg_quality,
            "svo2": str(svo_path),
            "svo_compression": "H265",
            "depth_png": bool(args.save_depth_png),
            "depth_units": "millimetres_uint16_zero_invalid" if args.save_depth_png else None,
            "depth_mode": args.depth_mode if args.save_depth_png else None,
            "ssd_device": EXPECTED_DEVICE,
            "ssd_uuid": EXPECTED_UUID,
            "ssd_mount": str(ssd_mount),
            "started_local": local_start.isoformat(),
            "started_utc": local_start.astimezone(timezone.utc).isoformat(),
        }
        metadata_path = session / "session.json"
        metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")

        manifest_path = session / "manifest.csv"
        manifest_handle = manifest_path.open("w", newline="", buffering=1)
        manifest = csv.writer(manifest_handle)
        manifest.writerow((
            "sequence", "timestamp_utc", "elapsed_seconds", "jpg", "depth_png",
            "width", "height", "jpg_bytes", "valid_depth_pixels", "valid_depth_fraction",
        ))

        runtime = sl.RuntimeParameters()
        left = sl.Mat()
        depth = sl.Mat() if args.save_depth_png else None
        next_save = started
        interval = 1.0 / SAVE_RATE_HZ

        print(f"session={session}")
        print(f"svo2={svo_path}")
        print(f"depth_png={args.save_depth_png}")
        print("Press Q/Esc in the preview or Ctrl+C in the terminal to stop.")

        while True:
            if camera.grab(runtime) != sl.ERROR_CODE.SUCCESS:
                continue
            camera.retrieve_image(left, sl.VIEW.LEFT)
            frame = cv2.cvtColor(left.get_data(), cv2.COLOR_BGRA2BGR)
            now = time.monotonic()
            elapsed = now - started

            if now >= next_save:
                height, width = frame.shape[:2]
                if (width, height) != (WIDTH, HEIGHT):
                    raise RuntimeError(f"Unexpected ZED frame size: {width}x{height}")
                utc = datetime.now(timezone.utc)
                stamp = utc.strftime("%Y%m%dT%H%M%S.%fZ")
                stem = f"zed2i_{stamp}_{sequence:07d}"
                jpg_path = images_dir / f"{stem}.jpg"
                jpg_temp = images_dir / f".{stem}.jpg"
                if not cv2.imwrite(str(jpg_temp), frame,
                                   [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]):
                    raise RuntimeError(f"Failed to write {jpg_temp}")
                jpg_temp.replace(jpg_path)

                depth_name = ""
                valid_pixels = ""
                valid_fraction = ""
                if args.save_depth_png and depth is not None and depth_dir is not None:
                    camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
                    depth_path = depth_dir / f"{stem}.png"
                    valid_pixels, valid_fraction_value = write_depth_png(
                        depth_path, depth.get_data(), args.min_depth_mm, args.max_depth_mm,
                    )
                    depth_name = depth_path.name
                    valid_fraction = f"{valid_fraction_value:.6f}"

                manifest.writerow((
                    sequence, utc.isoformat(), f"{elapsed:.6f}", jpg_path.name,
                    depth_name, width, height, jpg_path.stat().st_size,
                    valid_pixels, valid_fraction,
                ))
                sequence += 1
                next_save += interval
                if next_save <= now:
                    next_save = now + interval
                if sequence == 1 or sequence % 10 == 0:
                    print(f"saved_frames={sequence} elapsed={elapsed:.1f}s", flush=True)

            if not args.no_show:
                preview_height = round(frame.shape[0] * args.preview_width / frame.shape[1])
                preview = cv2.resize(frame, (args.preview_width, preview_height))
                cv2.putText(preview, f"REC SVO2 + JPG 2Hz  saved={sequence}", (20, 42),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
                cv2.imshow("RobotX ZED2i SVO2 recorder", preview)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            if args.seconds and elapsed >= args.seconds:
                break
    except KeyboardInterrupt:
        print("Stopping cleanly after Ctrl+C")
    finally:
        if manifest_handle is not None:
            manifest_handle.flush()
            os.fsync(manifest_handle.fileno())
            manifest_handle.close()
        if recording_enabled:
            camera.disable_recording()
        camera.close()
        cv2.destroyAllWindows()

    elapsed = max(time.monotonic() - started, 1e-6)
    metadata.update({
        "status": "complete",
        "ended_local": datetime.now().astimezone().isoformat(),
        "frames_saved": sequence,
        "elapsed_seconds": round(elapsed, 3),
        "effective_jpg_rate_hz": round(sequence / elapsed, 4),
        "svo2_bytes": svo_path.stat().st_size if svo_path.exists() else 0,
    })
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"frames_saved={sequence}")
    print(f"saved_to={session}")
    print(f"svo2={svo_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
