from pathlib import Path
import os


ROOT = Path(__file__).resolve().parent
os.environ["YOLO_CONFIG_DIR"] = str(ROOT / ".config")
os.environ["TORCH_HOME"] = str(ROOT / ".torch")

from ultralytics import YOLO


def main() -> None:
    # YOLO classification uses train/<class> and val/<class> directories.
    # RandAugment, color jitter, geometric transforms, and random erasing reduce
    # reliance on the exact recording background, though diverse real-world
    # backgrounds are still the best route to robust deployment.
    model = YOLO(str(ROOT / "yolo11n-cls.pt"))
    model.train(
        data=str(ROOT / "dataset"),
        project=str(ROOT / "runs"),
        name="clean_dirty_yolo11n_cls",
        exist_ok=True,
        epochs=100,
        imgsz=640,
        batch=24,  # Stable high-throughput setting for the 8 GB Blackwell GPU.
        device=0,
        workers=8,
        seed=42,
        deterministic=True,
        patience=0,
        auto_augment="randaugment",
        erasing=0.4,
        hsv_h=0.03,
        hsv_s=0.7,
        hsv_v=0.5,
        degrees=12.0,
        translate=0.12,
        scale=0.35,
        shear=5.0,
        perspective=0.0005,
        fliplr=0.5,
        flipud=0.1,
        cache=False,
        plots=True,
    )


if __name__ == "__main__":
    main()
