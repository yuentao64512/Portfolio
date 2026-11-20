#!/usr/bin/env python3
"""Live upright/down detector for the aerial YOLO model."""

from __future__ import annotations

import argparse
import time
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO


@dataclass
class Track:
    track_id: int
    center: tuple[float, float]
    box: np.ndarray
    label: str
    conf: float
    last_seen: float
    down_since: float | None = None
    alert: bool = False


def parse_source(value: str) -> int | str:
    return int(value) if value.isdigit() else value


def open_capture(source: int | str, backend: str, width: int, height: int, fps: int) -> cv2.VideoCapture:
    cv_backend = cv2.CAP_V4L2 if backend == "v4l2" else cv2.CAP_ANY
    cap = cv2.VideoCapture(source, cv_backend)
    if isinstance(source, int):
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap


def result_detections(result, names: dict[int, str], min_area: float) -> list[dict]:
    if result.boxes is None or len(result.boxes) == 0:
        return []

    boxes = result.boxes.xyxy.detach().cpu().numpy()
    confs = result.boxes.conf.detach().cpu().numpy()
    classes = result.boxes.cls.detach().cpu().numpy().astype(int)
    detections = []

    for box, conf, cls_id in zip(boxes, confs, classes):
        x1, y1, x2, y2 = box.astype(float)
        area = max(x2 - x1, 0.0) * max(y2 - y1, 0.0)
        if area < min_area:
            continue
        detections.append(
            {
                "box": box.astype(np.float32),
                "center": ((x1 + x2) / 2.0, (y1 + y2) / 2.0),
                "label": names.get(int(cls_id), str(cls_id)),
                "conf": float(conf),
                "area": float(area),
            }
        )

    detections.sort(key=lambda item: (item["label"] != "down", -item["area"]))
    return detections


def match_tracks(
    tracks: dict[int, Track],
    detections: list[dict],
    next_track_id: int,
    now: float,
    max_distance_px: float,
    max_people: int,
) -> int:
    detections = detections[:max_people]
    unmatched_detections = set(range(len(detections)))
    unmatched_tracks = set(tracks)
    pairs: list[tuple[float, int, int]] = []

    for tid, track in tracks.items():
        tx, ty = track.center
        for det_idx, det in enumerate(detections):
            dx, dy = det["center"]
            distance = ((tx - dx) ** 2 + (ty - dy) ** 2) ** 0.5
            pairs.append((distance, tid, det_idx))

    for distance, tid, det_idx in sorted(pairs, key=lambda item: item[0]):
        if distance > max_distance_px or tid not in unmatched_tracks or det_idx not in unmatched_detections:
            continue
        update_track(tracks[tid], detections[det_idx], now)
        unmatched_tracks.remove(tid)
        unmatched_detections.remove(det_idx)

    for det_idx in sorted(unmatched_detections):
        det = detections[det_idx]
        track = Track(
            track_id=next_track_id,
            center=det["center"],
            box=det["box"],
            label=det["label"],
            conf=det["conf"],
            last_seen=now,
        )
        if track.label == "down":
            track.down_since = now
        tracks[track.track_id] = track
        next_track_id += 1

    return next_track_id


def update_track(track: Track, detection: dict, now: float) -> None:
    track.center = detection["center"]
    track.box = detection["box"]
    track.label = detection["label"]
    track.conf = detection["conf"]
    track.last_seen = now
    if track.label == "down":
        if track.down_since is None:
            track.down_since = now
    else:
        track.down_since = None
        track.alert = False


def draw_text(frame, text: str, org: tuple[int, int], color: tuple[int, int, int], scale: float = 0.7) -> None:
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def draw_track(frame, track: Track, now: float, lying_alert_sec: float) -> None:
    x1, y1, x2, y2 = np.round(track.box).astype(int)
    down_for = 0.0 if track.down_since is None else now - track.down_since
    track.alert = track.label == "down" and down_for >= lying_alert_sec

    if track.alert:
        color = (0, 0, 255)
    elif track.label == "down":
        color = (0, 165, 255)
    else:
        color = (0, 220, 80)

    cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
    label = f"ID {track.track_id} {track.label} {track.conf:.2f}"
    if track.label == "down":
        label += f" {down_for:.1f}s"
    draw_text(frame, label, (max(0, x1), max(20, y1 - 8)), color, 0.55)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera", default="0", help="Camera index, video path, or stream URL.")
    parser.add_argument("--backend", default="v4l2", choices=["auto", "v4l2"])
    parser.add_argument("--model", type=Path, default=Path("models/aerial_upright_down_y11n_v1.pt"))
    parser.add_argument("--device", default="0")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--sample-fps", type=float, default=12.0)
    parser.add_argument("--lying-alert-sec", type=float, default=10.0)
    parser.add_argument("--max-people", type=int, default=8)
    parser.add_argument("--track-timeout-sec", type=float, default=1.5)
    parser.add_argument("--match-distance-px", type=float, default=120.0)
    parser.add_argument("--min-area", type=float, default=80.0)
    parser.add_argument("--headless", action="store_true")
    args = parser.parse_args()

    source = parse_source(args.camera)
    cap = open_capture(source, args.backend, args.width, args.height, args.fps)
    if not cap.isOpened():
        raise SystemExit(f"Could not open camera/video source: {args.camera}")

    model = YOLO(str(args.model))
    tracks: dict[int, Track] = {}
    next_track_id = 1
    frame_count = 0
    last_sample = 0.0
    last_print = 0.0
    start = time.monotonic()

    print("Upright/down live detector started. Press q in the preview window to stop.")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("Frame read failed; stopping.")
            break

        frame_count += 1
        now = time.monotonic()
        if now - last_sample >= 1.0 / max(args.sample_fps, 0.1):
            last_sample = now
            result = model.predict(frame, imgsz=args.imgsz, conf=args.conf, device=args.device, verbose=False)[0]
            detections = result_detections(result, model.names, args.min_area)
            next_track_id = match_tracks(
                tracks,
                detections,
                next_track_id,
                now,
                args.match_distance_px,
                args.max_people,
            )

        for tid in list(tracks):
            if now - tracks[tid].last_seen > args.track_timeout_sec:
                del tracks[tid]

        active_tracks = sorted(
            tracks.values(),
            key=lambda track: (track.label != "down", not track.alert, track.track_id),
        )

        for track in active_tracks:
            if not args.headless:
                draw_track(frame, track, now, args.lying_alert_sec)
            else:
                down_for = 0.0 if track.down_since is None else now - track.down_since
                track.alert = track.label == "down" and down_for >= args.lying_alert_sec

        elapsed = max(now - start, 1e-6)
        fps = frame_count / elapsed
        alerts = [track for track in active_tracks if track.alert]

        if not args.headless:
            if alerts:
                cv2.rectangle(frame, (0, 0), (frame.shape[1], 78), (0, 0, 180), -1)
                ids = ",".join(str(track.track_id) for track in alerts)
                draw_text(frame, f"ALERT: person down IDs={ids}", (16, 34), (255, 255, 255))
                draw_text(frame, f"people={len(active_tracks)} fps={fps:.1f}", (16, 66), (255, 255, 255))
            else:
                draw_text(frame, f"people={len(active_tracks)} fps={fps:.1f}", (16, 34), (255, 255, 255))
            cv2.imshow("Aerial Upright Down Detector", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

        if now - last_print >= 1.0:
            last_print = now
            if active_tracks:
                status = "; ".join(
                    f"ID {track.track_id} {track.label} conf={track.conf:.2f} alert={track.alert}"
                    for track in active_tracks[: args.max_people]
                )
            else:
                status = "no_person"
            print(f"people={len(active_tracks)} fps={fps:.1f} {status}")

    cap.release()
    if not args.headless:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
