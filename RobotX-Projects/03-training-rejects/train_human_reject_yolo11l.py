from __future__ import annotations

import argparse
from pathlib import Path

from ultralytics import YOLO


ROOT = Path(r"C:\Users\OptiPrime-UAV\yolov11_train")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Fine-tune YOLO11L as a dedicated single-class human detector."
    )
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument(
        "--data",
        type=Path,
        default=ROOT / "human_reject_dataset_fullframe" / "data.yaml",
    )
    parser.add_argument("--model", type=Path, default=ROOT / "yolo11l.pt")
    parser.add_argument("--epochs", type=int, default=80)
    parser.add_argument("--imgsz", type=int, default=1280)
    parser.add_argument("--batch", type=int, default=1)
    parser.add_argument("--patience", type=int, default=20)
    parser.add_argument("--name", default="human_reject_yolo11l_fullframe")
    parser.add_argument("--device", default="0")
    return parser.parse_args()


def count_dataset(data_yaml: Path) -> tuple[int, int, int, int]:
    dataset = data_yaml.parent
    train_images = len(list((dataset / "images" / "train").glob("*.jpg")))
    val_images = len(list((dataset / "images" / "val").glob("*.jpg")))
    train_labels = sum(
        bool(path.read_text(encoding="utf-8").strip())
        for path in (dataset / "labels" / "train").glob("*.txt")
    )
    val_labels = sum(
        bool(path.read_text(encoding="utf-8").strip())
        for path in (dataset / "labels" / "val").glob("*.txt")
    )
    return train_images, val_images, train_labels, val_labels


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    data = args.data if args.data.is_absolute() else root / args.data
    model_path = args.model if args.model.is_absolute() else root / args.model

    if not data.is_file():
        raise FileNotFoundError(f"Dataset YAML not found: {data}")
    if not model_path.is_file():
        raise FileNotFoundError(f"YOLO11L weights not found: {model_path}")

    train_images, val_images, train_positive, val_positive = count_dataset(data)
    print(
        f"Dataset check: {train_images} train frames ({train_positive} positive), "
        f"{val_images} val frames ({val_positive} positive)."
    )
    if min(train_images, val_images, train_positive, val_positive) == 0:
        raise RuntimeError(
            "Both train and val need images containing humans. "
            "Review the extraction manifest and labels before training."
        )

    model = YOLO(str(model_path))
    model.train(
        data=str(data),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device,
        workers=0,
        patience=args.patience,
        project=str(root / "runs"),
        name=args.name,
        pretrained=True,
        single_cls=True,
        close_mosaic=10,
        cache=False,
        plots=True,
    )

    best = root / "runs" / args.name / "weights" / "best.pt"
    print(f"Training complete. Best human reject weights: {best}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
