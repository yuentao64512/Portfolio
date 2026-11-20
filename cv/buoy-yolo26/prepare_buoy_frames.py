from pathlib import Path
import json

import cv2
import numpy as np

VIDEO = Path("/home/robotx/rgb_left.mp4")
ROOT = Path("/home/robotx/buoy_yolo26")
FRAMES = ROOT / "source_frames"
FRAMES.mkdir(parents=True, exist_ok=True)

capture = cv2.VideoCapture(str(VIDEO))
frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
fps = float(capture.get(cv2.CAP_PROP_FPS))
indices = np.linspace(0, frame_count - 1, 200).round().astype(int)

manifest = []
thumbs = []
for output_index, frame_index in enumerate(indices):
    capture.set(cv2.CAP_PROP_POS_FRAMES, int(frame_index))
    ok, frame = capture.read()
    if not ok:
        raise RuntimeError(f"Could not read frame {frame_index}")
    name = f"frame_{output_index:03d}_src_{frame_index:04d}.jpg"
    cv2.imwrite(str(FRAMES / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
    manifest.append({
        "file": name,
        "source_frame": int(frame_index),
        "time_s": round(float(frame_index / fps), 3),
    })
    if output_index % 8 == 0:
        thumb = cv2.resize(frame, (442, 249))
        cv2.putText(thumb, f"{output_index:03d}  {frame_index / fps:5.1f}s", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(thumb, f"{output_index:03d}  {frame_index / fps:5.1f}s", (10, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1, cv2.LINE_AA)
        thumbs.append(thumb)

capture.release()
(ROOT / "frames_manifest.json").write_text(json.dumps(manifest, indent=2))

cols = 5
rows = int(np.ceil(len(thumbs) / cols))
sheet = np.full((rows * 249, cols * 442, 3), 245, dtype=np.uint8)
for i, thumb in enumerate(thumbs):
    row, col = divmod(i, cols)
    sheet[row * 249:(row + 1) * 249, col * 442:(col + 1) * 442] = thumb
cv2.imwrite(str(ROOT / "contact_sheet.jpg"), sheet, [cv2.IMWRITE_JPEG_QUALITY, 92])
print(f"Extracted {len(manifest)} frames to {FRAMES}")
print(f"Contact sheet: {ROOT / 'contact_sheet.jpg'}")
