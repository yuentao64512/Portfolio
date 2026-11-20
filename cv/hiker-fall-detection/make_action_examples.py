from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parent
OUT = ROOT / "reports" / "figures" / "actions"
OUT.mkdir(parents=True, exist_ok=True)

examples = [
    ("standing", Path("/home/robotx/DJI_20261017214456_0016_D.MP4"), 25.00),
    ("walking", Path("/home/robotx/DJI_20261017214456_0016_D.MP4"), 35.00),
    ("falling", Path("/home/robotx/DJI_20261017214456_0016_D.MP4"), 37.00),
    ("sitting", Path("/home/robotx/DJI_20261017214456_0016_D.MP4"), 95.00),
    ("lying_down", Path("/home/robotx/DJI_20261017214456_0016_D.MP4"), 60.00),
]

for label, video, seconds in examples:
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise RuntimeError(f"Could not open {video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    cap.set(cv2.CAP_PROP_POS_FRAMES, round(seconds * fps))
    ok, frame = cap.read()
    cap.release()
    if not ok:
        raise RuntimeError(f"Could not read {video} at {seconds}s")
    # Save the complete source frame. No crop, resize, or aspect-ratio change.
    output = OUT / f"{label}.jpg"
    if not cv2.imwrite(str(output), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
        raise RuntimeError(f"Could not write {output}")
