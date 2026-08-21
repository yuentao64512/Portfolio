from pathlib import Path

import cv2
from ultralytics import YOLO


# ============================================================
# SETTINGS
# ============================================================

# Path to your trained YOLO model
MODEL_PATH = Path(
    r"C:\Users\OptiPrime-UAV\yolov11_train\runs\detect\train-19\weights\best.pt"
)

# Path to the input video
VIDEO_PATH = Path(
    r"C:\Users\OptiPrime-UAV\yolov11_train\water_test.mp4"
)

# Path for the processed output video
OUTPUT_PATH = Path(
    r"C:\Users\OptiPrime-UAV\yolov11_train\outputs\predicted_video_small.mp4"
)

# Minimum confidence required for a detection
CONF_THRESHOLD = 0.25

# YOLO processing resolution
# Lower values are faster but may reduce small-object detection accuracy
IMAGE_SIZE = 640

# Output video width
# The height will be calculated automatically to avoid stretching
OUTPUT_WIDTH = 640

# Set to False if cv2.imshow causes an error
SHOW_VIDEO = False

# GPU settings:
# Use 0 for your NVIDIA GPU
# Use "cpu" if no GPU is available
DEVICE = 0


def calculate_output_size(
    original_width: int,
    original_height: int,
    target_width: int,
) -> tuple[int, int]:
    """
    Calculate a resized output resolution while preserving aspect ratio.
    """

    if original_width <= 0 or original_height <= 0:
        raise ValueError("Invalid input video dimensions.")

    scale = target_width / original_width
    target_height = int(original_height * scale)

    # Some video codecs work better with even dimensions
    if target_height % 2 != 0:
        target_height -= 1

    return target_width, target_height


def main() -> None:
    # --------------------------------------------------------
    # Validate files
    # --------------------------------------------------------

    if not MODEL_PATH.exists():
        raise FileNotFoundError(
            f"YOLO model not found:\n{MODEL_PATH}"
        )

    if not VIDEO_PATH.exists():
        raise FileNotFoundError(
            f"Input video not found:\n{VIDEO_PATH}"
        )

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)

    # --------------------------------------------------------
    # Load YOLO model
    # --------------------------------------------------------

    print("Loading YOLO model...")

    model = YOLO(str(MODEL_PATH))

    print("Model loaded successfully.")

    # --------------------------------------------------------
    # Open input video
    # --------------------------------------------------------

    capture = cv2.VideoCapture(str(VIDEO_PATH))

    if not capture.isOpened():
        raise RuntimeError(
            f"Unable to open input video:\n{VIDEO_PATH}"
        )

    original_width = int(
        capture.get(cv2.CAP_PROP_FRAME_WIDTH)
    )

    original_height = int(
        capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
    )

    fps = capture.get(cv2.CAP_PROP_FPS)

    total_frames = int(
        capture.get(cv2.CAP_PROP_FRAME_COUNT)
    )

    if fps <= 0:
        print("Warning: Could not detect video FPS. Using 30 FPS.")
        fps = 30.0

    output_width, output_height = calculate_output_size(
        original_width=original_width,
        original_height=original_height,
        target_width=OUTPUT_WIDTH,
    )

    print(f"Original resolution: {original_width} x {original_height}")
    print(f"Output resolution:   {output_width} x {output_height}")
    print(f"Video FPS:           {fps:.2f}")
    print(f"Total frames:        {total_frames}")

    # --------------------------------------------------------
    # Create output video writer
    # --------------------------------------------------------

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    writer = cv2.VideoWriter(
        str(OUTPUT_PATH),
        fourcc,
        fps,
        (output_width, output_height),
    )

    if not writer.isOpened():
        capture.release()

        raise RuntimeError(
            f"Unable to create output video:\n{OUTPUT_PATH}"
        )

    frame_number = 0
    total_detections = 0

    print("\nStarting video prediction...")
    print("Press Q to stop early when the display window is enabled.\n")

    try:
        while True:
            success, frame = capture.read()

            if not success:
                break

            frame_number += 1

            # ------------------------------------------------
            # Run YOLO prediction
            # ------------------------------------------------

            results = model.predict(
                source=frame,
                conf=CONF_THRESHOLD,
                imgsz=IMAGE_SIZE,
                device=DEVICE,
                verbose=False,
            )

            result = results[0]

            # YOLO draws bounding boxes, labels and confidence
            annotated_frame = result.plot()

            if result.boxes is not None:
                detection_count = len(result.boxes)
            else:
                detection_count = 0

            total_detections += detection_count

            # ------------------------------------------------
            # Add information text
            # ------------------------------------------------

            cv2.putText(
                annotated_frame,
                f"Frame: {frame_number}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.putText(
                annotated_frame,
                f"Detections: {detection_count}",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            # ------------------------------------------------
            # Resize the video while preserving aspect ratio
            # ------------------------------------------------

            resized_frame = cv2.resize(
                annotated_frame,
                (output_width, output_height),
                interpolation=cv2.INTER_AREA,
            )

            # Save resized frame
            writer.write(resized_frame)

            # ------------------------------------------------
            # Show progress
            # ------------------------------------------------

            if total_frames > 0:
                progress = frame_number / total_frames * 100

                print(
                    f"\rFrame {frame_number}/{total_frames} "
                    f"({progress:.1f}%) | "
                    f"Detections: {detection_count}",
                    end="",
                    flush=True,
                )
            else:
                print(
                    f"\rFrame {frame_number} | "
                    f"Detections: {detection_count}",
                    end="",
                    flush=True,
                )

            # ------------------------------------------------
            # Optional display window
            # ------------------------------------------------

            if SHOW_VIDEO:
                cv2.imshow(
                    "YOLO Video Prediction",
                    resized_frame,
                )

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("\nStopped by user.")
                    break

    except KeyboardInterrupt:
        print("\nPrediction stopped using Ctrl+C.")

    finally:
        capture.release()
        writer.release()
        cv2.destroyAllWindows()

    print("\n\nPrediction completed.")
    print(f"Frames processed: {frame_number}")
    print(f"Total detections: {total_detections}")
    print(f"Output saved to:\n{OUTPUT_PATH}")


if __name__ == "__main__":
    main()