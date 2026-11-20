#!/usr/bin/env python3
"""Export the deployed YOLO26l checkpoint to a fixed-shape FP16 TensorRT engine."""

from pathlib import Path

from ultralytics import YOLO


SOURCE = Path("/home/jetson/pipe_cones/models/pipe_cone_yolo26l_best.pt")
TARGET = Path("/home/jetson/pipe_cones/models/pipe_cone_yolo26l_fp16.engine")


def main() -> None:
    exported = Path(
        YOLO(str(SOURCE), task="detect").export(
            format="engine",
            imgsz=960,
            half=True,
            batch=1,
            dynamic=False,
            workspace=2,
            device=0,
            simplify=True,
        )
    )
    exported.replace(TARGET)
    print(TARGET)


if __name__ == "__main__":
    main()
