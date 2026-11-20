#!/usr/bin/env python3
"""Draw per-camera hospital bed/floor/furniture polygons."""

from __future__ import annotations

import argparse
from collections import Counter
from pathlib import Path

import cv2

from hospital_context import (
    ContextConfig,
    ContextGeometry,
    PolygonRegion,
    REGION_COLORS,
    normalized_points,
    save_context_config,
)


KEY_TYPES = {
    ord("1"): "bed",
    ord("2"): "floor",
    ord("3"): "couch",
    ord("4"): "chair",
    ord("5"): "stretcher",
    ord("6"): "ignore",
}


def parse_source(value: str) -> int | str:
    return int(value) if value.isdigit() else value


def read_reference(source: int | str, width: int, height: int) -> tuple[cv2.VideoCapture | None, object]:
    if isinstance(source, str) and Path(source).suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"}:
        frame = cv2.imread(source)
        if frame is None:
            raise SystemExit(f"Could not read reference image: {source}")
        return None, frame
    capture = cv2.VideoCapture(source)
    if isinstance(source, int):
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    if not capture.isOpened():
        raise SystemExit(f"Could not open source: {source}")
    frame = None
    for _ in range(10):
        ok, candidate = capture.read()
        if ok:
            frame = candidate
    if frame is None:
        capture.release()
        raise SystemExit(f"Could not read a frame from: {source}")
    return capture, frame


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", default="0", help="Camera index, video, or reference image")
    parser.add_argument("--camera-id", required=True)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    args = parser.parse_args()

    output = args.output or Path("configs") / f"{args.camera_id}_context_rois.json"
    capture, reference = read_reference(parse_source(args.source), args.width, args.height)
    height, width = reference.shape[:2]
    window = "Hospital Context ROI Editor"
    regions: list[PolygonRegion] = []
    current_points: list[tuple[int, int]] = []
    current_type = "bed"
    counts: Counter[str] = Counter()

    def on_mouse(event, x, y, _flags, _parameter):
        if event == cv2.EVENT_LBUTTONDOWN:
            current_points.append((x, y))
        elif event == cv2.EVENT_RBUTTONDOWN and current_points:
            current_points.pop()

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    print("Left click: add point | right click/u: undo | Enter: finish polygon")
    print("1 bed | 2 floor | 3 couch | 4 chair | 5 stretcher | 6 ignore | s save | q quit")

    try:
        while True:
            config = ContextConfig(1, args.camera_id, width, height, list(regions))
            canvas = ContextGeometry(config).draw(reference) if regions else reference.copy()
            color = REGION_COLORS[current_type]
            if current_points:
                for first, second in zip(current_points, current_points[1:]):
                    cv2.line(canvas, first, second, color, 2, cv2.LINE_AA)
                for point in current_points:
                    cv2.circle(canvas, point, 4, color, -1, cv2.LINE_AA)
            instructions = f"type={current_type} points={len(current_points)} regions={len(regions)}"
            cv2.putText(canvas, instructions, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4)
            cv2.putText(canvas, instructions, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.imshow(window, canvas)
            key = cv2.waitKey(20) & 0xFF
            if key in KEY_TYPES:
                current_type = KEY_TYPES[key]
            elif key in (13, 10) and len(current_points) >= 3:
                counts[current_type] += 1
                regions.append(
                    PolygonRegion(
                        name=f"{current_type}_{counts[current_type]}",
                        region_type=current_type,
                        points=normalized_points(current_points, width, height),
                    )
                )
                current_points.clear()
            elif key in (ord("u"), 8, 127) and current_points:
                current_points.pop()
            elif key == ord("x"):
                current_points.clear()
            elif key == ord("d") and regions:
                regions.pop()
            elif key == ord("s"):
                save_context_config(ContextConfig(1, args.camera_id, width, height, regions), output)
                print(f"Saved {len(regions)} regions to {output}")
            elif key in (ord("q"), 27):
                break
    finally:
        if capture is not None:
            capture.release()
        cv2.destroyAllWindows()

    if regions:
        save_context_config(ContextConfig(1, args.camera_id, width, height, regions), output)
        print(f"Saved {len(regions)} regions to {output}")


if __name__ == "__main__":
    main()
