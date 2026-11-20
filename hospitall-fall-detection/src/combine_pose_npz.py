#!/usr/bin/env python3
"""Combine multiple pose-sequence NPZ files."""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("inputs", nargs="+", type=Path)
    args = parser.parse_args()

    arrays = [np.load(path, allow_pickle=True) for path in args.inputs]
    X = np.concatenate([arr["X"] for arr in arrays], axis=0)
    y = np.concatenate([arr["y"].astype(str) for arr in arrays], axis=0)
    segment_id = np.concatenate([arr["segment_id"].astype(str) for arr in arrays], axis=0)
    detection_rate = np.concatenate([arr["detection_rate"].astype(np.float32) for arr in arrays], axis=0)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, X=X, y=y, segment_id=segment_id, detection_rate=detection_rate)
    print(f"Wrote {len(y)} combined pose sequences to {args.output}")
    for label in sorted(set(y)):
        print(f"{label}: {int((y == label).sum())}")


if __name__ == "__main__":
    main()
