#!/usr/bin/env python3
"""Small mapping test; no training or external data required."""

from __future__ import annotations

import subprocess
import sys
import tempfile
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    with tempfile.TemporaryDirectory() as temporary:
        base = Path(temporary)
        master = base / "master"
        (master / "images" / "train").mkdir(parents=True)
        (master / "labels" / "train").mkdir(parents=True)
        (master / "images" / "train" / "sample.jpg").write_bytes(b"test")
        rows = [f"{index} 0.5 0.5 0.1 0.1" for index in range(10)]
        (master / "labels" / "train" / "sample.txt").write_text("\n".join(rows) + "\n")
        output = base / "derived"
        subprocess.run([
            sys.executable, str(ROOT / "build_dual_datasets.py"),
            "--master", str(master), "--output", str(output), "--min-boxes", "0",
        ], check=True, stdout=subprocess.DEVNULL)
        nav = (output / "nav_dataset" / "labels" / "train" / "sample.txt").read_text().splitlines()
        dock = (output / "dock_dataset" / "labels" / "train" / "sample.txt").read_text().splitlines()
        assert [int(row.split()[0]) for row in nav] == list(range(7))
        assert [int(row.split()[0]) for row in dock] == [3, 3, 3, 3, 3, 3, 0, 1, 2]
    print("dual dataset mapping test: PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

