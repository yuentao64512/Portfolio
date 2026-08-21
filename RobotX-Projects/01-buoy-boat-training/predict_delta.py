from pathlib import Path

from ultralytics import YOLO


# ================= CHANGE THESE SETTINGS =================

# Image, video, folder, RTSP URL, or 0 for webcam.
SOURCE = r"C:\Users\OptiPrime-UAV\yolov11_train\usv_12082026_yolo11l\dataset\images\train"
#SOURCE = 0

# Leave as None to automatically find the newest best.pt.
WEIGHTS = None

CONFIDENCE = 0.35
IMAGE_SIZE = 640
DEVICE = 0                 # NVIDIA GPU; use "cpu" for CPU
SHOW_LIVE_WINDOW = True
VIDEO_STRIDE = 1

# =========================================================


ROOT = Path(__file__).resolve().parent
OUTPUT_FOLDER = ROOT / "predictions"
OUTPUT_NAME = "buoy_boat_prediction"


def find_latest_best_pt():
    candidates = list(ROOT.rglob("best.pt"))

    if not candidates:
        raise FileNotFoundError(
            f"No best.pt found under {ROOT}. "
            "Set WEIGHTS to the exact checkpoint path."
        )

    return max(candidates, key=lambda path: path.stat().st_mtime)


def main():
    weights_path = Path(WEIGHTS) if WEIGHTS else find_latest_best_pt()

    print(f"Using weights: {weights_path}")
    model = YOLO(str(weights_path))
    print(f"Model classes: {model.names}")

    results = model.predict(
        source=SOURCE,
        conf=CONFIDENCE,
        imgsz=IMAGE_SIZE,
        device=DEVICE,
        show=SHOW_LIVE_WINDOW,
        save=True,
        save_txt=True,
        save_conf=True,
        vid_stride=VIDEO_STRIDE,
        project=str(OUTPUT_FOLDER),
        name=OUTPUT_NAME,
        exist_ok=True,
        stream=True,
    )

    for result in results:
        if result.boxes is None:
            continue

        for box in result.boxes:
            class_id = int(box.cls[0])
            confidence = float(box.conf[0])
            class_name = model.names[class_id]

            print(
                f"Detected: {class_name}, "
                f"confidence: {confidence:.2f}"
            )

    print(f"Results saved in: {OUTPUT_FOLDER / OUTPUT_NAME}")


if __name__ == "__main__":
    main()