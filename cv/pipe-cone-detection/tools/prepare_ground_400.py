#!/usr/bin/env python3
"""Expand the five ground MOV recordings into 400 tracked YOLO images."""

from pathlib import Path

import prepare_dataset as base


base.DATASET = Path("/home/robotx/pipe_cones/dataset_ground_400")
base.SOURCES = (
    base.Source("IMG_7816", 47, "train", (286, 825, 528, 438)),
    base.Source("IMG_7817", 110, "train", (388, 788, 250, 225)),
    base.Source("IMG_7818", 27, "test", (460, 805, 95, 108)),
    base.Source("IMG_7819", 90, "val", (422, 750, 318, 258)),
    base.Source("IMG_7820", 126, "train", (414, 925, 263, 280)),
)


if __name__ == "__main__":
    base.main()
