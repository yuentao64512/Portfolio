#!/usr/bin/env python3
"""Fine-tune YOLO26l for pipe-cone detection for exactly 100 epochs."""

from pathlib import Path

from ultralytics import YOLO


PROJECT = Path(__file__).resolve().parent


def main() -> None:
    model = YOLO("yolo26l.pt")
    model.train(
        data=str(PROJECT / "dataset.yaml"),
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
        name="yolo26l_100e",
        exist_ok=True,
        save=True,
        save_period=10,
        plots=True,
        verbose=True,
    )


if __name__ == "__main__":
    main()
