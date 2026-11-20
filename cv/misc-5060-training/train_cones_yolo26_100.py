#!/usr/bin/env python3
"""Train the orange/green cone YOLO26L detector for a full 100 epochs."""

from ultralytics import YOLO


DATA = "/home/robotx/cone_yolo26_v5/dataset.yaml"
BASE_WEIGHTS = "/home/robotx/yolo26l.pt"


def main() -> None:
    model = YOLO(BASE_WEIGHTS)
    model.train(
        data=DATA,
        imgsz=640,
        epochs=100,
        batch=4,
        device=0,
        workers=4,
        pretrained=True,
        project="/home/robotx/cone_yolo26/runs",
        name="yolo26l_cones_100ep",
        exist_ok=True,
        patience=100,       # do not stop before the requested 100 epochs
        cos_lr=True,
        close_mosaic=10,
        hsv_h=0.005,
        hsv_s=0.20,
        hsv_v=0.20,
        cache=False,
        plots=True,
        save=True,
    )


if __name__ == "__main__":
    main()
