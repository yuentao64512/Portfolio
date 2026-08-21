from pathlib import Path
import json
import os
import sys


ROOT = Path(__file__).resolve().parent
os.environ["YOLO_CONFIG_DIR"] = str(ROOT / ".config")
os.environ["TORCH_HOME"] = str(ROOT / ".torch")

from ultralytics import YOLO


def main() -> None:
    weights = ROOT / "runs" / "clean_dirty_yolo11n_cls" / "weights" / "best.pt"
    dataset = ROOT / "dataset"
    if not weights.is_file():
        raise FileNotFoundError(f"Missing trained model: {weights}")
    if not (dataset / "val" / "clean").is_dir() or not (dataset / "val" / "dirty").is_dir():
        raise FileNotFoundError(f"Validation dataset is incomplete: {dataset / 'val'}")

    model = YOLO(str(weights))
    expected_names = {0: "clean", 1: "dirty"}
    if model.names != expected_names:
        raise RuntimeError(f"Unexpected class mapping: {model.names}; expected {expected_names}")

    metrics = model.val(
        data=str(dataset),
        split="val",
        imgsz=640,
        batch=24,
        device=0,
        workers=8,
        project=str(ROOT / "validation_runs"),
        name="best_model_validation",
        exist_ok=True,
        plots=True,
    )
    report = {
        "status": "PASS",
        "weights": str(weights),
        "classes": model.names,
        "validation_images": sum(1 for p in (dataset / "val").rglob("*.jpg")),
        "top1_accuracy": float(metrics.top1),
        "top5_accuracy": float(metrics.top5),
        "python": sys.executable,
    }
    (ROOT / "validation_report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("\nVALIDATION PASS")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
