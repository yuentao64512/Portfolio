#!/usr/bin/env python3
"""Match ZED JPG UTC timestamps to ArduPilot local NED and attitude.

Supports MAVLink telemetry logs (.tlog) containing LOCAL_POSITION_NED and
ATTITUDE, and ArduPilot DataFlash logs (.bin) containing POS and ATT. Pymavlink
must be able to attach UTC timestamps to the selected messages.
"""

from __future__ import annotations

import argparse
import csv
import math
import re
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
from pymavlink import mavutil


FRAME_PATTERN = re.compile(r"^zed2i_(.+)_(\d+)\.(?:jpg|jpeg)$", re.IGNORECASE)


@dataclass
class Sample:
    timestamp: float
    values: tuple[float, ...]


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--images", type=Path, required=True,
                        help="session images directory containing zed2i_*.jpg")
    parser.add_argument("--fcu-log", type=Path, required=True,
                        help="ArduPilot .tlog or DataFlash .bin")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--max-gap", type=float, default=2.0,
                        help="largest permitted time from a frame to each telemetry stream")
    parser.add_argument("--time-offset", type=float, default=0.0,
                        help="seconds added to frame UTC before matching")
    return parser.parse_args()


def parse_frame_time(path: Path) -> tuple[float, int]:
    match = FRAME_PATTERN.match(path.name)
    if not match:
        raise ValueError(f"Unsupported ZED filename: {path.name}")
    stamp, frame_text = match.groups()
    parsed = None
    for fmt in ("%Y%m%dT%H%M%S.%fZ", "%Y%m%dT%H%M%SZ"):
        try:
            parsed = datetime.strptime(stamp, fmt).replace(tzinfo=timezone.utc)
            break
        except ValueError:
            pass
    if parsed is None:
        try:
            parsed = datetime.fromisoformat(stamp.replace("Z", "+00:00"))
            if parsed.tzinfo is None:
                parsed = parsed.replace(tzinfo=timezone.utc)
            parsed = parsed.astimezone(timezone.utc)
        except ValueError as error:
            raise ValueError(f"Cannot parse UTC timestamp in {path.name}") from error
    return parsed.timestamp(), int(frame_text)


def message_time(message) -> float | None:
    value = getattr(message, "_timestamp", None)
    if value is None:
        return None
    value = float(value)
    # Epoch 2000. Boot-relative timestamps cannot be matched to UTC JPG names.
    return value if value >= 946684800.0 else None


def finite_tuple(*values) -> tuple[float, ...] | None:
    converted = tuple(float(value) for value in values)
    return converted if all(math.isfinite(value) for value in converted) else None


def load_telemetry(path: Path) -> tuple[list[Sample], list[Sample], dict[str, int]]:
    connection = mavutil.mavlink_connection(str(path), dialect="ardupilotmega")
    positions: list[Sample] = []
    attitudes: list[Sample] = []
    counts: dict[str, int] = {}
    while True:
        message = connection.recv_match(blocking=False)
        if message is None:
            break
        kind = message.get_type()
        counts[kind] = counts.get(kind, 0) + 1
        timestamp = message_time(message)
        if timestamp is None:
            continue
        if kind == "LOCAL_POSITION_NED":
            values = finite_tuple(message.x, message.y, message.z)
            if values is not None:
                positions.append(Sample(timestamp, values))
        elif kind == "POS" and all(hasattr(message, field) for field in ("PN", "PE", "PD")):
            values = finite_tuple(message.PN, message.PE, message.PD)
            if values is not None:
                positions.append(Sample(timestamp, values))
        elif kind == "ATTITUDE":
            # MAVLink ATTITUDE fields are radians.
            values = finite_tuple(message.yaw, message.pitch, message.roll)
            if values is not None:
                attitudes.append(Sample(timestamp, tuple(math.degrees(v) for v in values)))
        elif kind == "ATT" and all(hasattr(message, field) for field in ("Yaw", "Pitch", "Roll")):
            # DataFlash ATT fields are degrees.
            values = finite_tuple(message.Yaw, message.Pitch, message.Roll)
            if values is not None:
                attitudes.append(Sample(timestamp, values))
    connection.close()
    positions.sort(key=lambda item: item.timestamp)
    attitudes.sort(key=lambda item: item.timestamp)
    return positions, attitudes, counts


def interpolate(samples: list[Sample], timestamp: float, unwrap_first: bool = False
                ) -> tuple[tuple[float, ...] | None, float]:
    if not samples:
        return None, math.inf
    times = np.asarray([sample.timestamp for sample in samples], dtype=np.float64)
    index = int(np.searchsorted(times, timestamp))
    nearest = min(abs(timestamp - times[max(0, index - 1)]),
                  abs(timestamp - times[min(len(times) - 1, index)]))
    if timestamp < times[0] or timestamp > times[-1]:
        return None, float(nearest)
    columns = np.asarray([sample.values for sample in samples], dtype=np.float64)
    output = []
    for column in range(columns.shape[1]):
        values = columns[:, column]
        if unwrap_first and column == 0:
            values = np.rad2deg(np.unwrap(np.deg2rad(values)))
        output.append(float(np.interp(timestamp, times, values)))
    if unwrap_first:
        output[0] = output[0] % 360.0
    return tuple(output), float(nearest)


def main() -> int:
    args = arguments()
    if not args.images.is_dir():
        raise SystemExit(f"Image folder not found: {args.images}")
    if not args.fcu_log.is_file():
        raise SystemExit(f"FCU log not found: {args.fcu_log}")
    images = sorted(
        path for path in args.images.iterdir()
        if path.suffix.lower() in {".jpg", ".jpeg"} and path.name.startswith("zed2i_")
    )
    if not images:
        raise SystemExit(f"No zed2i JPGs found in {args.images}")

    positions, attitudes, counts = load_telemetry(args.fcu_log)
    if not positions:
        raise SystemExit("No UTC-timestamped LOCAL_POSITION_NED or POS samples found")
    if not attitudes:
        raise SystemExit("No UTC-timestamped ATTITUDE or ATT samples found")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    matched = 0
    with args.output.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow((
            "frame", "sequence", "frame_utc", "frame_epoch", "north_m", "east_m", "down_m",
            "yaw_deg", "pitch_deg", "roll_deg", "position_nearest_s", "attitude_nearest_s", "matched",
        ))
        for image in images:
            epoch, sequence = parse_frame_time(image)
            epoch += args.time_offset
            position, position_gap = interpolate(positions, epoch)
            attitude, attitude_gap = interpolate(attitudes, epoch, unwrap_first=True)
            valid = (position is not None and attitude is not None
                     and position_gap <= args.max_gap and attitude_gap <= args.max_gap)
            if valid:
                matched += 1
                north, east, down = position
                yaw, pitch, roll = attitude
                values = (f"{north:.4f}", f"{east:.4f}", f"{down:.4f}",
                          f"{yaw:.4f}", f"{pitch:.4f}", f"{roll:.4f}")
            else:
                values = ("", "", "", "", "", "")
            frame_utc = datetime.fromtimestamp(epoch, timezone.utc).isoformat()
            writer.writerow((image.name, sequence, frame_utc, f"{epoch:.6f}", *values,
                             f"{position_gap:.4f}", f"{attitude_gap:.4f}", int(valid)))

    print(f"images={len(images)}")
    print(f"matched={matched}")
    print(f"unmatched={len(images) - matched}")
    print(f"position_samples={len(positions)}")
    print(f"attitude_samples={len(attitudes)}")
    print(f"message_types={counts}")
    print(f"output={args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
