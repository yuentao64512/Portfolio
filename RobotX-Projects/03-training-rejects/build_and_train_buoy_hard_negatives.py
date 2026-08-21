from __future__ import annotations

import argparse
import csv
import shutil
from collections import Counter
from pathlib import Path

from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")
MARKER_NAMES = ("blue_marker", "red_marker", "green_marker")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Combine the original 3-class buoy dataset with human_* crops as empty-label "
            "hard negatives, then fine-tune the good marker model."
        )
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument(
        "--source-dataset",
        type=Path,
        default=ROOT / "water_test",
        help="Original marker dataset used by train-19.",
    )
    parser.add_argument(
        "--hard-negative-source",
        type=Path,
        default=ROOT / "yolo_train",
        help="Dataset containing the human_* crop images.",
    )
    parser.add_argument("--output-dataset", type=Path, default=ROOT / "yolo_train_hard_negative")
    parser.add_argument(
        "--model",
        type=Path,
        default=ROOT / "runs" / "detect" / "train-19" / "weights" / "best.pt",
    )
    parser.add_argument("--epochs", type=int, default=40)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--batch", type=int, default=1)
    parser.add_argument("--device", default="0")
    parser.add_argument("--name", default="buoy_hard_negative_v1")
    parser.add_argument("--dataset-only", action="store_true")
    return parser.parse_args()


def backup_existing(path: Path) -> Path | None:
    if not path.exists():
        return None
    counter = 0
    while True:
        suffix = "_old" if counter == 0 else f"_old_{counter}"
        backup = path.with_name(path.name + suffix)
        if not backup.exists():
            path.rename(backup)
            return backup
        counter += 1


def label_for_image(image: Path, source_dataset: Path, split: str) -> Path:
    return source_dataset / "labels" / split / f"{image.stem}.txt"


def parse_marker_label(label_path: Path) -> tuple[list[str], Counter[int]]:
    if not label_path.exists():
        return [], Counter()

    clean_lines: list[str] = []
    counts: Counter[int] = Counter()
    for line_number, raw_line in enumerate(
        label_path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line:
            continue
        fields = line.split()
        if len(fields) != 5:
            raise RuntimeError(f"Invalid detection label at {label_path}:{line_number}: {line}")
        try:
            class_id = int(float(fields[0]))
            coordinates = [float(value) for value in fields[1:]]
        except ValueError as error:
            raise RuntimeError(
                f"Invalid numeric value at {label_path}:{line_number}: {line}"
            ) from error
        if class_id not in (0, 1, 2):
            raise RuntimeError(
                f"Non-marker class {class_id} remains in {label_path}:{line_number}. "
                "Only files beginning with human_ are converted to hard negatives."
            )
        if any(value < 0.0 or value > 1.0 for value in coordinates):
            raise RuntimeError(f"Out-of-range box at {label_path}:{line_number}: {line}")
        clean_lines.append(" ".join(fields))
        counts[class_id] += 1
    return clean_lines, counts


def unique_image_path(folder: Path, stem: str, suffix: str) -> Path:
    candidate = folder / f"{stem}{suffix.lower()}"
    counter = 1
    while candidate.exists():
        candidate = folder / f"{stem}_{counter}{suffix.lower()}"
        counter += 1
    return candidate


def write_dataset_metadata(dataset: Path) -> None:
    for relative in ("images/train", "images/val", "labels/train", "labels/val"):
        (dataset / relative).mkdir(parents=True, exist_ok=True)
    (dataset / "classes.txt").write_text("\n".join(MARKER_NAMES) + "\n", encoding="utf-8")
    (dataset / "data.yaml").write_text(
        "\n".join(
            (
                f"path: {dataset.as_posix()}",
                "train: images/train",
                "val: images/val",
                "nc: 3",
                "names:",
                "  0: blue_marker",
                "  1: red_marker",
                "  2: green_marker",
                "",
            )
        ),
        encoding="utf-8",
    )


def collect_images(source_dataset: Path) -> list[tuple[str, Path]]:
    records: list[tuple[str, Path]] = []
    for split in ("train", "val"):
        folder = source_dataset / "images" / split
        if not folder.is_dir():
            raise FileNotFoundError(f"Missing source image folder: {folder}")
        records.extend(
            (split, path)
            for path in sorted(folder.iterdir())
            if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS
        )
    return records


def build_dataset(
    source_dataset: Path, hard_negative_source: Path, output_dataset: Path
) -> dict[str, object]:
    marker_records = [
        record
        for record in collect_images(source_dataset)
        if not record[1].stem.lower().startswith("human_")
    ]
    human_records = [
        record
        for record in collect_images(hard_negative_source)
        if record[1].stem.lower().startswith("human_")
    ]
    if not human_records:
        raise RuntimeError(
            f"No human_* crop images were found under {hard_negative_source}."
        )

    previous = backup_existing(output_dataset)
    if previous is not None:
        print(f"Previous hard-negative dataset moved to: {previous}")
    write_dataset_metadata(output_dataset)

    class_counts: Counter[int] = Counter()
    marker_images = 0
    existing_negative_images = 0
    manifest_rows: list[tuple[str, str, str, str, str]] = []

    for source_split, image in marker_records:
        source_label = label_for_image(image, source_dataset, source_split)
        lines, counts = parse_marker_label(source_label)
        target_image = output_dataset / "images" / source_split / image.name
        target_label = output_dataset / "labels" / source_split / f"{image.stem}.txt"
        shutil.copy2(image, target_image)
        target_label.write_text(("\n".join(lines) + "\n") if lines else "", encoding="utf-8")
        class_counts.update(counts)
        marker_images += int(bool(lines))
        existing_negative_images += int(not lines)
        manifest_rows.append(("marker", source_split, str(image), str(target_image), str(target_label)))

    hard_negative_counts: Counter[str] = Counter()
    for index, (source_split, image) in enumerate(sorted(human_records, key=lambda item: str(item[1]).lower())):
        # Guarantee validation coverage without mixing the original marker splits.
        target_split = "val" if index % 5 == 0 else "train"
        target_image_dir = output_dataset / "images" / target_split
        target_label_dir = output_dataset / "labels" / target_split
        target_image = unique_image_path(target_image_dir, f"hardneg_{image.stem}", image.suffix)
        target_label = target_label_dir / f"{target_image.stem}.txt"
        shutil.copy2(image, target_image)
        target_label.write_text("", encoding="utf-8")
        hard_negative_counts[target_split] += 1
        manifest_rows.append(("human_hard_negative", source_split, str(image), str(target_image), str(target_label)))

    with (output_dataset / "manifest.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(("kind", "source_split", "source_image", "output_image", "output_label"))
        writer.writerows(manifest_rows)

    for class_id in range(3):
        if class_counts[class_id] == 0:
            raise RuntimeError(f"No training labels were found for {MARKER_NAMES[class_id]}.")

    summary = {
        "marker_positive_images": marker_images,
        "existing_negative_images": existing_negative_images,
        "human_hard_negatives_train": hard_negative_counts["train"],
        "human_hard_negatives_val": hard_negative_counts["val"],
        "class_boxes": {MARKER_NAMES[key]: class_counts[key] for key in range(3)},
    }
    print("Dataset built successfully:")
    for key, value in summary.items():
        print(f"  {key}: {value}")
    print(f"  yaml: {output_dataset / 'data.yaml'}")
    return summary


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    source_dataset = args.source_dataset if args.source_dataset.is_absolute() else root / args.source_dataset
    hard_negative_source = (
        args.hard_negative_source
        if args.hard_negative_source.is_absolute()
        else root / args.hard_negative_source
    )
    output_dataset = args.output_dataset if args.output_dataset.is_absolute() else root / args.output_dataset
    model_path = args.model if args.model.is_absolute() else root / args.model

    if not model_path.is_file():
        raise FileNotFoundError(f"Good marker weights not found: {model_path}")

    build_dataset(
        source_dataset.resolve(),
        hard_negative_source.resolve(),
        output_dataset.resolve(),
    )
    if args.dataset_only:
        return 0

    run_path = root / "runs" / args.name
    previous_run = backup_existing(run_path)
    if previous_run is not None:
        print(f"Previous training run moved to: {previous_run}")

    print(f"Fine-tuning from: {model_path}")
    model = YOLO(str(model_path))
    model.train(
        data=str(output_dataset / "data.yaml"),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=0,
        patience=15,
        cache=False,
        amp=False,
        project=str(root / "runs"),
        name=args.name,
        exist_ok=True,
        plots=True,
    )
    best = root / "runs" / args.name / "weights" / "best.pt"
    if not best.is_file():
        raise RuntimeError(f"Training finished without expected best weights: {best}")
    print(f"Hard-negative buoy model ready: {best}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
