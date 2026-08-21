import cv2
from pathlib import Path

DATASET = Path("dataset")
OUT_DATASET = Path("tiled_dataset")

TILE = 640
OVERLAP = 0.2
MIN_KEEP = 0.4

splits = ["train", "val"]

def yolo_to_xyxy(label, w, h):
    cls, x, y, bw, bh = label
    x = float(x) * w
    y = float(y) * h
    bw = float(bw) * w
    bh = float(bh) * h
    return int(cls), x - bw / 2, y - bh / 2, x + bw / 2, y + bh / 2

def xyxy_to_yolo(cls, x1, y1, x2, y2, tw, th):
    xc = ((x1 + x2) / 2) / tw
    yc = ((y1 + y2) / 2) / th
    bw = (x2 - x1) / tw
    bh = (y2 - y1) / th
    return f"{cls} {xc:.6f} {yc:.6f} {bw:.6f} {bh:.6f}"

def tile_split(split):
    img_dir = DATASET / "images" / split
    lbl_dir = DATASET / "labels" / split

    out_img = OUT_DATASET / "images" / split
    out_lbl = OUT_DATASET / "labels" / split

    out_img.mkdir(parents=True, exist_ok=True)
    out_lbl.mkdir(parents=True, exist_ok=True)

    stride = int(TILE * (1 - OVERLAP))

    for img_path in img_dir.glob("*.*"):
        img = cv2.imread(str(img_path))
        if img is None:
            continue

        h, w = img.shape[:2]
        label_path = lbl_dir / f"{img_path.stem}.txt"

        labels = []
        if label_path.exists():
            with open(label_path, "r") as f:
                labels = [line.strip().split() for line in f if line.strip()]

        boxes = [yolo_to_xyxy(l, w, h) for l in labels]

        for y in range(0, h, stride):
            for x in range(0, w, stride):
                x_end = min(x + TILE, w)
                y_end = min(y + TILE, h)

                tile = img[y:y_end, x:x_end]
                tile_labels = []

                for cls, bx1, by1, bx2, by2 in boxes:
                    ix1 = max(bx1, x)
                    iy1 = max(by1, y)
                    ix2 = min(bx2, x_end)
                    iy2 = min(by2, y_end)

                    if ix2 <= ix1 or iy2 <= iy1:
                        continue

                    original_area = (bx2 - bx1) * (by2 - by1)
                    new_area = (ix2 - ix1) * (iy2 - iy1)

                    if new_area / original_area < MIN_KEEP:
                        continue

                    tile_labels.append(
                        xyxy_to_yolo(
                            cls,
                            ix1 - x,
                            iy1 - y,
                            ix2 - x,
                            iy2 - y,
                            x_end - x,
                            y_end - y
                        )
                    )

                if tile_labels:
                    out_name = f"{img_path.stem}_{x}_{y}.jpg"
                    cv2.imwrite(str(out_img / out_name), tile)

                    with open(out_lbl / out_name.replace(".jpg", ".txt"), "w") as f:
                        f.write("\n".join(tile_labels))

    print(f"{split} tiling complete.")

for split in splits:
    tile_split(split)

print("All tiling complete.")