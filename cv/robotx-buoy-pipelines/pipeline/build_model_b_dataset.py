from __future__ import annotations

import argparse
import csv
import json
import shutil
from collections import Counter
from pathlib import Path

import cv2
import yaml


CLASS_NAMES = ["circle", "diamond"] + [f"digit_{digit}" for digit in range(10)]
CLASS_IDS = {name: index for index, name in enumerate(CLASS_NAMES)}

SPLIT_BY_GROUP = {
    "IMG_3867_ecbdf083": "train",
    "IMG_7695_7633ac3e": "train",
    "WhatsApp_Video_2026-11-08_at_4_51_46_PM_3e4231fa": "train",
    "WhatsApp_Video_2026-11-08_at_4_51_48_PM_3e56d734": "train",
    "still__photo_2_2026-11-08_22-59-05_d10a8e2d": "train",
    "still__photo_3_2026-11-08_22-59-05_2a940ad0": "train",
    "IMG_3870_88c543a9": "val",
    "IMG_3866_f356c2ac": "test",
}


def source_group(stem: str) -> str:
    if "__f" in stem:
        return stem.split("__f", 1)[0]
    if "__d" in stem:
        return stem.split("__d", 1)[0]
    return stem


def yolo_line(class_id: int, points: list[list[float]], width: int, height: int) -> str:
    if len(points) < 2:
        raise ValueError(f"rectangle must contain at least two points, received {points!r}")
    x_values = [float(point[0]) for point in points]
    y_values = [float(point[1]) for point in points]
    x1, x2 = min(x_values), max(x_values)
    y1, y2 = min(y_values), max(y_values)
    x1, x2 = sorted((max(0.0, min(float(width), x1)), max(0.0, min(float(width), x2))))
    y1, y2 = sorted((max(0.0, min(float(height), y1)), max(0.0, min(float(height), y2))))
    if x2 - x1 < 1.0 or y2 - y1 < 1.0:
        raise ValueError(f"degenerate rectangle: {points!r}")
    return (
        f"{class_id} {((x1 + x2) / 2) / width:.6f} {((y1 + y2) / 2) / height:.6f} "
        f"{(x2 - x1) / width:.6f} {(y2 - y1) / height:.6f}"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--review-json", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {args.output}")

    image_files = sorted(path for path in args.images.iterdir() if path.suffix.lower() in {".jpg", ".jpeg", ".png"})
    observed_groups = {source_group(path.stem) for path in image_files}
    missing_assignments = sorted(observed_groups - set(SPLIT_BY_GROUP))
    stale_assignments = sorted(set(SPLIT_BY_GROUP) - observed_groups)
    if missing_assignments or stale_assignments:
        raise SystemExit(
            f"split mapping mismatch: missing={missing_assignments}, stale={stale_assignments}"
        )

    for split in ("train", "val", "test"):
        (args.output / "images" / split).mkdir(parents=True, exist_ok=False)
        (args.output / "labels" / split).mkdir(parents=True, exist_ok=False)

    split_counts: Counter[str] = Counter()
    class_counts: dict[str, Counter[str]] = {
        split: Counter() for split in ("train", "val", "test")
    }
    rows: list[dict[str, object]] = []

    for image_path in image_files:
        group = source_group(image_path.stem)
        split = SPLIT_BY_GROUP[group]
        image = cv2.imread(str(image_path))
        if image is None:
            raise SystemExit(f"cannot read image: {image_path}")
        height, width = image.shape[:2]
        label_lines: list[str] = []
        json_path = args.review_json / f"{image_path.stem}.json"

        if json_path.exists():
            payload = json.loads(json_path.read_text(encoding="utf-8"))
            if int(payload.get("imageWidth", width)) != width or int(payload.get("imageHeight", height)) != height:
                raise SystemExit(f"image size mismatch in {json_path}")
            for shape in payload.get("shapes", []):
                label = shape.get("label")
                if label not in CLASS_IDS:
                    raise SystemExit(f"unknown label {label!r} in {json_path}")
                if shape.get("shape_type") != "rectangle":
                    raise SystemExit(f"non-rectangle shape in {json_path}: {shape.get('shape_type')!r}")
                label_lines.append(yolo_line(CLASS_IDS[label], shape["points"], width, height))
                class_counts[split][label] += 1

        shutil.copy2(image_path, args.output / "images" / split / image_path.name)
        (args.output / "labels" / split / f"{image_path.stem}.txt").write_text(
            "\n".join(label_lines) + ("\n" if label_lines else ""), encoding="utf-8"
        )
        split_counts[split] += 1
        rows.append(
            {
                "image": image_path.name,
                "source_group": group,
                "split": split,
                "objects": len(label_lines),
                "review_json": json_path.name if json_path.exists() else "",
            }
        )

    data = {
        "path": str(args.output),
        "train": "images/train",
        "val": "images/val",
        "test": "images/test",
        "names": {index: name for index, name in enumerate(CLASS_NAMES)},
    }
    (args.output / "data.yaml").write_text(yaml.safe_dump(data, sort_keys=False), encoding="utf-8")
    with (args.output / "split_manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=["image", "source_group", "split", "objects", "review_json"])
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "images": dict(split_counts),
        "class_instances": {split: dict(class_counts[split]) for split in ("train", "val", "test")},
        "groups": {
            split: sorted(group for group, assigned in SPLIT_BY_GROUP.items() if assigned == split)
            for split in ("train", "val", "test")
        },
        "class_names": CLASS_NAMES,
        "review_source": str(args.review_json),
    }
    (args.output / "dataset_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
