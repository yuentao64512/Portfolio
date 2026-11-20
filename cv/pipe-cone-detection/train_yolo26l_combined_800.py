#!/usr/bin/env python3
"""Train YOLO26l for 100 epochs on the mixed 800-image dataset."""

from pathlib import Path

from ultralytics import YOLO


PROJECT = Path(__file__).resolve().parent


def main():
    # Preserve the small, portrait-video cone features learned by the original
    # ground model, then adapt them to the waterfront/ZED domain. Starting from
    # generic weights caused the larger waterfront targets to dominate.
    model = YOLO(str(PROJECT / "runs/yolo26l_100e/weights/best.pt"))
    model.train(
        data=str(PROJECT / "dataset_combined_800.yaml"),
        epochs=100,
        patience=0,
        imgsz=960,
        batch=2,
        device=0,
        workers=4,
        cache="ram",
        pretrained=True,
        optimizer="AdamW",
        lr0=0.0002,
        lrf=0.10,
        warmup_epochs=1.0,
        single_cls=True,
        seed=42,
        deterministic=True,
        amp=True,
        multi_scale=0.10,
        close_mosaic=10,
        project=str(PROJECT / "runs"),
        name="yolo26l_combined_800_finetune_restart_100e",
        exist_ok=True,
        save=True,
        save_period=10,
        plots=True,
        verbose=True,
    )


if __name__ == "__main__":
    main()
