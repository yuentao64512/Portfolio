from pathlib import Path
import json
import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
DATASET = ROOT / "dataset"
SOURCES = {
    "clean": Path(r"C:\Users\OptiPrime-UAV\Desktop\Clean Paper.MOV"),
    "dirty": Path(r"C:\Users\OptiPrime-UAV\Desktop\Dirty Paper.MOV"),
}
COUNTS = {"train": 240, "val": 60}


def sample_indices(total: int) -> dict[str, np.ndarray]:
    # Hold out the final 20% as a contiguous validation interval to reduce
    # near-duplicate leakage between train and validation video frames.
    boundary = int(total * 0.8)
    return {
        "train": np.linspace(0, boundary - 1, COUNTS["train"], dtype=int),
        "val": np.linspace(boundary, total - 1, COUNTS["val"], dtype=int),
    }


def main() -> None:
    manifest = {"sources": {}, "split_strategy": "first 80% train, final 20% val"}
    for split in COUNTS:
        for label in SOURCES:
            (DATASET / split / label).mkdir(parents=True, exist_ok=True)

    for label, video_path in SOURCES.items():
        cap = cv2.VideoCapture(str(video_path))
        if not cap.isOpened():
            raise RuntimeError(f"Could not open {video_path}")
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        fps = float(cap.get(cv2.CAP_PROP_FPS))
        indices = sample_indices(total)
        wanted = {int(i): split for split, values in indices.items() for i in values}
        written = {"train": 0, "val": 0}
        frame_index = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            split = wanted.get(frame_index)
            if split:
                number = written[split] + 1
                output = DATASET / split / label / f"{label}_{number:04d}_f{frame_index:05d}.jpg"
                if not cv2.imwrite(str(output), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                    raise RuntimeError(f"Could not write {output}")
                written[split] = number
            frame_index += 1
        cap.release()
        if written != COUNTS:
            raise RuntimeError(f"Unexpected counts for {label}: {written}")
        manifest["sources"][label] = {
            "video": str(video_path),
            "total_video_frames": total,
            "fps": fps,
            "duration_seconds": total / fps,
            "written": written,
        }

    (ROOT / "dataset_manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
