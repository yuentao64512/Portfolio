#!/usr/bin/env python3
"""Prepare a non-destructive, de-duplicated YOLO review workspace."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
from pathlib import Path


IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def digest(path: Path) -> str:
    value = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            value.update(chunk)
    return value.hexdigest()


def link_or_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source, target)
    except OSError:
        shutil.copy2(source, target)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("root", type=Path)
    args = parser.parse_args()
    root = args.root.resolve()
    images = root / "images"
    labels = root / "labels"
    images.mkdir(exist_ok=True)
    labels.mkdir(exist_ok=True)

    originals = sorted(p for p in root.iterdir()
                       if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)
    by_hash = {}
    duplicates = {}
    for image in originals:
        sha = digest(image)
        if sha in by_hash:
            duplicates[str(image.name)] = str(by_hash[sha].name)
            continue
        by_hash[sha] = image
        target = images / image.name
        if not target.exists():
            link_or_copy(image, target)
        # An empty YOLO label is correct until boxes are added; the viewer
        # marks it EMPTY for review.
        (labels / f"{image.stem}.txt").touch(exist_ok=True)

    data_yaml = """path: /home/robotx/Robotx_train
train: images
val: images
names:
  0: number_buoy
  1: boundary
"""
    (root / "data.yaml").write_text(data_yaml)
    manifest = {
        "original_images": len(originals),
        "unique_images": len(by_hash),
        "exact_duplicates_skipped": len(duplicates),
        "duplicates": duplicates,
        "classes": {"0": "number_buoy", "1": "boundary"},
        "source_images_modified": False,
    }
    (root / "prepare_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(json.dumps(manifest, indent=2))


if __name__ == "__main__":
    main()
