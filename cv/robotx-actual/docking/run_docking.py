#!/usr/bin/env python3
import argparse
import statistics
from pathlib import Path
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DEFAULT_MODEL = ROOT / "models" / "dock_partial_best.engine"

def main():
    p = argparse.ArgumentParser(description="Run the AGX-native TensorRT dock detector.")
    p.add_argument("--source", default="0", help="Camera index, image, video, or stream URL")
    p.add_argument("--model", default=str(DEFAULT_MODEL))
    p.add_argument("--conf", type=float, default=0.50)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--save", action="store_true", help="Save annotated output under reports/")
    p.add_argument("--show", action="store_true", help="Display a window")
    p.add_argument("--name", default="predict")
    args = p.parse_args()

    source = int(args.source) if args.source.isdigit() else args.source
    model = YOLO(args.model, task="detect")
    frames = detections = 0
    inference_ms = []
    for result in model.predict(
        source=source,
        stream=True,
        imgsz=args.imgsz,
        conf=args.conf,
        device=0,
        save=args.save,
        show=args.show,
        project=str(ROOT / "reports"),
        name=args.name,
        exist_ok=True,
        verbose=True,
    ):
        frames += 1
        detections += 0 if result.boxes is None else len(result.boxes)
        inference_ms.append(float(result.speed.get("inference", 0.0)))

    median_ms = statistics.median(inference_ms) if inference_ms else 0.0
    fps = 1000.0 / median_ms if median_ms > 0 else 0.0
    print(f"frames={frames} detections={detections} median_inference_ms={median_ms:.2f} model_fps={fps:.1f}")

if __name__ == "__main__":
    main()
