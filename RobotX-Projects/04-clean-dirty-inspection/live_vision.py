from pathlib import Path
import argparse
import os
import time

import cv2


ROOT = Path(__file__).resolve().parent
os.environ["YOLO_CONFIG_DIR"] = str(ROOT / ".config")
os.environ["TORCH_HOME"] = str(ROOT / ".torch")

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Live clean/dirty YOLO11 classifier")
    parser.add_argument("--source", default="1", help="Camera index, video path, or stream URL")
    parser.add_argument("--width", type=int, default=1280, help="Requested camera width")
    parser.add_argument("--height", type=int, default=720, help="Requested camera height")
    parser.add_argument("--confidence", type=float, default=0.60, help="Minimum confidence to show a class")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    weights = ROOT / "runs" / "clean_dirty_yolo11n_cls" / "weights" / "best.pt"
    if not weights.is_file():
        raise FileNotFoundError(f"Model not found: {weights}")

    model = YOLO(str(weights))
    try:
        source: int | str = int(args.source)
    except ValueError:
        source = args.source

    cap = cv2.VideoCapture(source, cv2.CAP_DSHOW) if isinstance(source, int) else cv2.VideoCapture(source)
    if isinstance(source, int):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(
            f"Could not open source {args.source}. Close other camera apps or choose another source."
        )

    title = "YOLO11 Clean vs Dirty - Q or Esc to quit"
    previous = time.perf_counter()
    smoothed_fps = 0.0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError("Camera stopped returning frames")

            result = model.predict(frame, imgsz=640, device=0, verbose=False)[0]
            class_id = int(result.probs.top1)
            confidence = float(result.probs.top1conf)
            label = model.names[class_id]

            now = time.perf_counter()
            instant_fps = 1.0 / max(now - previous, 1e-6)
            previous = now
            smoothed_fps = instant_fps if smoothed_fps == 0 else 0.9 * smoothed_fps + 0.1 * instant_fps

            if confidence < args.confidence:
                display_label = "UNCERTAIN"
                color = (0, 215, 255)
            elif label.lower() == "clean":
                display_label = "CLEAN"
                color = (40, 210, 40)
            else:
                display_label = "DIRTY"
                color = (30, 30, 230)

            cv2.rectangle(frame, (15, 15), (570, 115), (20, 20, 20), -1)
            cv2.putText(frame, f"{display_label}  {confidence:.1%}", (30, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.25, color, 3, cv2.LINE_AA)
            cv2.putText(frame, f"FPS: {smoothed_fps:.1f} | GPU: RTX PRO 2000 Blackwell", (30, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (235, 235, 235), 2, cv2.LINE_AA)
            cv2.imshow(title, frame)

            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
