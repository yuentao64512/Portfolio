#!/usr/bin/env python3
"""Train YOLO26l for 100 epochs on the new ZED buoy recordings."""

from pathlib import Path

from ultralytics import YOLO


PROJECT = Path(__file__).resolve().parent


def main():
    # Start from the general pretrained model; the previous checkpoint assigned
    # only 0.01-0.12 confidence to this new camera/domain.
    model = YOLO(str(PROJECT / "yolo26l.pt"))
    model.train(
        data=str(PROJECT / "dataset_recorded_400.yaml"),
        epochs=100,
        patience=0,
        imgsz=960,
        batch=2,
        device=0,
        workers=4,
        cache="ram",
        pretrained=True,
        single_cls=True,
        seed=42,
        deterministic=True,
        amp=True,
        multi_scale=0.10,
        close_mosaic=10,
        project=str(PROJECT / "runs"),
        name="yolo26l_recorded_400_100e",
        exist_ok=True,
        save=True,
        save_period=10,
        plots=True,
        verbose=True,
    )


if __name__ == "__main__":
    main()
