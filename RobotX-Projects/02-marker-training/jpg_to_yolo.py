import argparse
import random
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

IMG_EXTS = [".jpg", ".jpeg", ".png", ".bmp", ".webp", ".tif", ".tiff"]


def find_image(images_dir: Path, stem: str):
    for ext in IMG_EXTS:
        p = images_dir / f"{stem}{ext}"
        if p.exists():
            return p
    return None


def convert_box(size_w, size_h, xmin, ymin, xmax, ymax):
    x_center = ((xmin + xmax) / 2.0) / size_w
    y_center = ((ymin + ymax) / 2.0) / size_h
    width = (xmax - xmin) / size_w
    height = (ymax - ymin) / size_h
    return x_center, y_center, width, height


def parse_voc_xml(xml_file: Path, class_to_id: dict, skip_difficult: bool = True):
    tree = ET.parse(xml_file)
    root = tree.getroot()

    size = root.find("size")
    if size is None:
        raise ValueError(f"{xml_file}: missing <size>")

    w = int(float(size.findtext("width", "0")))
    h = int(float(size.findtext("height", "0")))

    if w <= 0 or h <= 0:
        raise ValueError(f"{xml_file}: invalid image size {w}x{h}")

    labels = []

    for obj in root.findall("object"):
        name = obj.findtext("name", "").strip()
        difficult = int(obj.findtext("difficult", "0"))

        if skip_difficult and difficult == 1:
            continue

        if name not in class_to_id:
            print(f"[WARN] {xml_file.name}: class '{name}' not in class list, skipping")
            continue

        bnd = obj.find("bndbox")
        if bnd is None:
            print(f"[WARN] {xml_file.name}: object '{name}' missing bndbox, skipping")
            continue

        xmin = float(bnd.findtext("xmin", "0"))
        ymin = float(bnd.findtext("ymin", "0"))
        xmax = float(bnd.findtext("xmax", "0"))
        ymax = float(bnd.findtext("ymax", "0"))

        # Clamp to image bounds
        xmin = max(0.0, min(xmin, w - 1))
        ymin = max(0.0, min(ymin, h - 1))
        xmax = max(0.0, min(xmax, w - 1))
        ymax = max(0.0, min(ymax, h - 1))

        if xmax <= xmin or ymax <= ymin:
            print(f"[WARN] {xml_file.name}: invalid bbox for '{name}', skipping")
            continue

        x, y, bw, bh = convert_box(w, h, xmin, ymin, xmax, ymax)
        cls_id = class_to_id[name]
        labels.append(f"{cls_id} {x:.6f} {y:.6f} {bw:.6f} {bh:.6f}")

    return labels


def write_yaml(out_dir: Path, class_names):
    yaml_path = out_dir / "data.yaml"
    with open(yaml_path, "w", encoding="utf-8") as f:
        f.write(f"path: {out_dir.resolve()}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write("names:\n")
        for i, name in enumerate(class_names):
            f.write(f"  {i}: {name}\n")
    return yaml_path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--images", required=True, help="Folder with source images")
    parser.add_argument("--xml", required=True, help="Folder with Pascal VOC XML files")
    parser.add_argument("--out", required=True, help="Output YOLO dataset folder")
    parser.add_argument("--classes", required=True,
                        help='Comma-separated class names in exact order, e.g. "person,helmet,vest"')
    parser.add_argument("--val-ratio", type=float, default=0.2, help="Validation split ratio")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--keep-empty", action="store_true",
                        help="Keep images with no valid labels")
    args = parser.parse_args()

    images_dir = Path(args.images)
    xml_dir = Path(args.xml)
    out_dir = Path(args.out)

    class_names = [x.strip() for x in args.classes.split(",") if x.strip()]
    class_to_id = {name: i for i, name in enumerate(class_names)}

    if not images_dir.exists():
        raise FileNotFoundError(f"Images folder not found: {images_dir}")
    if not xml_dir.exists():
        raise FileNotFoundError(f"XML folder not found: {xml_dir}")
    if not class_names:
        raise ValueError("No classes provided")

    (out_dir / "images" / "train").mkdir(parents=True, exist_ok=True)
    (out_dir / "images" / "val").mkdir(parents=True, exist_ok=True)
    (out_dir / "labels" / "train").mkdir(parents=True, exist_ok=True)
    (out_dir / "labels" / "val").mkdir(parents=True, exist_ok=True)

    xml_files = sorted(xml_dir.glob("*.xml"))
    if not xml_files:
        raise FileNotFoundError(f"No XML files found in {xml_dir}")

    pairs = []
    missing_images = 0

    for xml_file in xml_files:
        stem = xml_file.stem
        img_file = find_image(images_dir, stem)

        if img_file is None:
            try:
                root = ET.parse(xml_file).getroot()
                filename = root.findtext("filename", "").strip()
                if filename:
                    candidate = images_dir / filename
                    if candidate.exists():
                        img_file = candidate
            except Exception:
                pass

        if img_file is None:
            print(f"[WARN] no matching image for {xml_file.name}")
            missing_images += 1
            continue

        pairs.append((img_file, xml_file))

    if not pairs:
        raise RuntimeError("No image/XML pairs found")

    random.seed(args.seed)
    random.shuffle(pairs)

    val_count = max(1, int(len(pairs) * args.val_ratio)) if len(pairs) > 1 else 0
    val_set = set(x[0].stem for x in pairs[:val_count])

    written = 0
    skipped_empty = 0

    for img_file, xml_file in pairs:
        split = "val" if img_file.stem in val_set else "train"

        labels = parse_voc_xml(xml_file, class_to_id)

        if not labels and not args.keep_empty:
            skipped_empty += 1
            continue

        out_img = out_dir / "images" / split / img_file.name
        out_lbl = out_dir / "labels" / split / f"{img_file.stem}.txt"

        shutil.copy2(img_file, out_img)

        with open(out_lbl, "w", encoding="utf-8") as f:
            f.write("\n".join(labels))

        written += 1

    yaml_path = write_yaml(out_dir, class_names)

    print("\nDone.")
    print(f"Classes: {class_names}")
    print(f"Paired image/XML files: {len(pairs)}")
    print(f"Missing images: {missing_images}")
    print(f"Written samples: {written}")
    print(f"Skipped empty-label samples: {skipped_empty}")
    print(f"Dataset YAML: {yaml_path}")


if __name__ == "__main__":
    main()
