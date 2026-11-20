#!/usr/bin/env python3
"""Evaluate available test splits without reusing validation as test data."""

from pathlib import Path

from ultralytics import YOLO


ROOT = Path(__file__).resolve().parent
CASES = (
    (ROOT / "runs" / "buoys_v1" / "weights" / "best.pt", ROOT / "buoys.yaml", ROOT / "buoy_dataset", 960),
    (ROOT / "runs" / "digits_v1_legacy_provisional" / "weights" / "best.pt", ROOT / "digits.yaml", ROOT / "digit_dataset", 320),
)


def main() -> int:
    for weight, yaml_path, dataset, image_size in CASES:
        test_images = list((dataset / "images" / "test").glob("*"))
        if not weight.is_file():
            print(f"SKIP missing weight: {weight}")
            continue
        if not test_images:
            print(f"SKIP no independent test images: {dataset}")
            continue
        result = YOLO(str(weight)).val(data=str(yaml_path), split="test", imgsz=image_size)
        print(weight)
        print("mAP50", round(float(result.box.map50), 4))
        print("mAP50-95", round(float(result.box.map), 4))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
