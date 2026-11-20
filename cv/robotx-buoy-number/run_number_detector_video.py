#!/usr/bin/env python3
"""Run buoy detection, then detect digit boxes inside number-buoy crops."""

from __future__ import annotations

import argparse
import csv
import time
from pathlib import Path

import cv2
from ultralytics import YOLO


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--buoy-detector", type=Path, required=True)
    parser.add_argument("--number-detector", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--buoy-conf", type=float, default=0.15)
    parser.add_argument("--number-conf", type=float, default=0.20)
    parser.add_argument("--device", default="0")
    args = parser.parse_args()

    buoy_model = YOLO(str(args.buoy_detector))
    number_model = YOLO(str(args.number_detector))
    capture = cv2.VideoCapture(str(args.source))
    if not capture.isOpened():
        raise SystemExit(f"Cannot open source: {args.source}")

    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = capture.get(cv2.CAP_PROP_FPS) or 30.0
    total = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(
        str(args.output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
    )
    if not writer.isOpened():
        raise SystemExit(f"Cannot create output: {args.output}")

    fields = [
        "frame", "time_s", "buoy_conf", "digit", "digit_conf",
        "buoy_x1", "buoy_y1", "buoy_x2", "buoy_y2",
        "digit_x1", "digit_y1", "digit_x2", "digit_y2",
    ]
    digit_count = 0
    start = time.time()
    with args.csv.open("w", newline="", encoding="utf-8") as handle:
        csv_writer = csv.DictWriter(handle, fieldnames=fields)
        csv_writer.writeheader()
        frame_index = 0
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            result = buoy_model.predict(
                frame, imgsz=1280, conf=args.buoy_conf, iou=0.45,
                device=args.device, verbose=False,
            )[0]
            for buoy_box in result.boxes:
                buoy_class = str(result.names[int(buoy_box.cls.item())])
                if buoy_class != "number_buoy":
                    continue
                buoy_conf = float(buoy_box.conf.item())
                x1, y1, x2, y2 = [int(round(v)) for v in buoy_box.xyxy[0].tolist()]
                x1, y1 = max(0, x1), max(0, y1)
                x2, y2 = min(width, x2), min(height, y2)
                bw, bh = x2 - x1, y2 - y1
                if bw < 12 or bh < 12:
                    continue
                pad_x, pad_y = max(2, int(bw * 0.08)), max(2, int(bh * 0.08))
                cx1, cy1 = max(0, x1 - pad_x), max(0, y1 - pad_y)
                cx2, cy2 = min(width, x2 + pad_x), min(height, y2 + pad_y)
                crop = frame[cy1:cy2, cx1:cx2]
                if not crop.size:
                    continue

                digits = number_model.predict(
                    crop, imgsz=320, conf=args.number_conf, iou=0.45,
                    device=args.device, verbose=False,
                )[0]
                labels = []
                for digit_box in digits.boxes:
                    digit = str(digits.names[int(digit_box.cls.item())])
                    digit_conf = float(digit_box.conf.item())
                    dx1, dy1, dx2, dy2 = [
                        int(round(v)) for v in digit_box.xyxy[0].tolist()
                    ]
                    gx1, gy1 = cx1 + dx1, cy1 + dy1
                    gx2, gy2 = cx1 + dx2, cy1 + dy2
                    cv2.rectangle(frame, (gx1, gy1), (gx2, gy2), (60, 255, 60), 2)
                    digit_text_y = max(18, gy1 - 5)
                    digit_text = f"{digit} {digit_conf:.2f}"
                    cv2.putText(frame, digit_text, (gx1, digit_text_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 4, cv2.LINE_AA)
                    cv2.putText(frame, digit_text, (gx1, digit_text_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (60, 255, 60), 2, cv2.LINE_AA)
                    labels.append((digit_conf, digit))
                    csv_writer.writerow({
                        "frame": frame_index,
                        "time_s": f"{frame_index / fps:.3f}",
                        "buoy_conf": f"{buoy_conf:.6f}",
                        "digit": digit,
                        "digit_conf": f"{digit_conf:.6f}",
                        "buoy_x1": x1, "buoy_y1": y1, "buoy_x2": x2, "buoy_y2": y2,
                        "digit_x1": gx1, "digit_y1": gy1, "digit_x2": gx2, "digit_y2": gy2,
                    })
                    digit_count += 1

                labels.sort(reverse=True)
                summary = labels[0][1] if labels else "?"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 210, 255), 2)
                buoy_text = f"number_buoy {buoy_conf:.2f} | digit {summary}"
                text_y = max(22, y1 - 7)
                cv2.putText(frame, buoy_text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 0, 0), 4, cv2.LINE_AA)
                cv2.putText(frame, buoy_text, (x1, text_y), cv2.FONT_HERSHEY_SIMPLEX,
                            0.55, (0, 210, 255), 2, cv2.LINE_AA)

            writer.write(frame)
            frame_index += 1
            if frame_index % 100 == 0 or frame_index == total:
                elapsed = max(0.001, time.time() - start)
                print(
                    f"frames={frame_index}/{total} speed={frame_index / elapsed:.1f}fps "
                    f"digit_detections={digit_count}", flush=True,
                )

    capture.release()
    writer.release()
    elapsed = time.time() - start
    print(f"COMPLETE frames={frame_index} digit_detections={digit_count} elapsed={elapsed:.1f}s")
    print(f"video={args.output}")
    print(f"csv={args.csv}")


if __name__ == "__main__":
    main()
