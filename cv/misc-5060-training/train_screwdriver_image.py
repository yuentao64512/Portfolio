#!/usr/bin/env python3
"""Train a one-shot screwdriver image model."""

from __future__ import annotations

import argparse

from screwdriver_vision import train_reference_image


def main() -> None:
    parser = argparse.ArgumentParser(description="Train ORB features from a screwdriver reference image")
    parser.add_argument("image", help="Tightly cropped reference image of the screwdriver")
    parser.add_argument(
        "--output",
        default="models/screwdriver_orb.npz",
        help="Output model path (default: models/screwdriver_orb.npz)",
    )
    args = parser.parse_args()
    result = train_reference_image(args.image, args.output)
    print(f"trained {result['features']} features -> {result['model_path']}")


if __name__ == "__main__":
    main()
