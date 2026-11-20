#!/usr/bin/env python3
"""Export a portable pose ONNX and optionally a machine-specific TensorRT engine."""

from __future__ import annotations

import argparse
import hashlib
import json
import platform
import shutil
from datetime import datetime, timezone
from pathlib import Path

import torch
import ultralytics
from ultralytics import YOLO


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", type=Path, default=Path("yolo11n-pose.pt"))
    parser.add_argument("--output-dir", type=Path, default=Path("models/pose_exports"))
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--opset", type=int, default=17)
    parser.add_argument("--dynamic", action="store_true")
    parser.add_argument("--build-engine", action="store_true",
                        help="Build a TensorRT FP16 engine for THIS machine only")
    parser.add_argument("--workspace", type=float, default=2.0)
    args = parser.parse_args()

    if not args.weights.is_file():
        raise SystemExit(f"Pose weights not found: {args.weights}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    model = YOLO(str(args.weights))

    exported_onnx = Path(
        model.export(
            format="onnx", imgsz=args.imgsz, opset=args.opset,
            dynamic=args.dynamic, simplify=True, half=False, device="cpu",
        )
    )
    onnx_target = args.output_dir / f"{args.weights.stem}_{args.imgsz}.onnx"
    if exported_onnx.resolve() != onnx_target.resolve():
        shutil.copy2(exported_onnx, onnx_target)

    engine_target = None
    if args.build_engine:
        exported_engine = Path(
            model.export(
                format="engine", imgsz=args.imgsz, half=True, device=0,
                dynamic=False, workspace=args.workspace,
            )
        )
        engine_target = args.output_dir / f"{args.weights.stem}_{args.imgsz}_fp16.engine"
        if exported_engine.resolve() != engine_target.resolve():
            shutil.copy2(exported_engine, engine_target)

    metadata = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "source_weights": str(args.weights.resolve()),
        "source_sha256": sha256(args.weights),
        "imgsz": args.imgsz,
        "opset": args.opset,
        "dynamic": args.dynamic,
        "onnx": str(onnx_target.resolve()),
        "onnx_sha256": sha256(onnx_target),
        "engine": str(engine_target.resolve()) if engine_target else None,
        "engine_note": "TensorRT engines are hardware/JetPack-specific; build the Jetson engine on the Orin Nano.",
        "ultralytics": ultralytics.__version__,
        "torch": torch.__version__,
        "cuda": torch.version.cuda,
        "host": platform.node(),
        "classes": model.names,
    }
    metadata_path = args.output_dir / f"{args.weights.stem}_{args.imgsz}_metadata.json"
    metadata_path.write_text(json.dumps(metadata, indent=2) + "\n", encoding="utf-8")
    print(f"onnx={onnx_target}")
    print(f"metadata={metadata_path}")
    if engine_target:
        print(f"engine={engine_target}")


if __name__ == "__main__":
    main()
