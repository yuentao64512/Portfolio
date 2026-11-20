#!/usr/bin/env python3
"""Create a hard-linked dataset copy with class-3 boxes filtered by review IDs."""

from __future__ import annotations

import argparse
import csv
import json
import os
import shutil
from collections import Counter
from pathlib import Path


def link_or_copy(source: Path, target: Path) -> None:
    target.parent.mkdir(parents=True, exist_ok=True)
    try:
        os.link(source.resolve(), target)
    except OSError:
        shutil.copy2(source.resolve(), target)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifests", type=Path, nargs="+", required=True)
    parser.add_argument("--accepted", required=True,
                        help="Comma-separated reviewed boundary IDs")
    args = parser.parse_args()

    if args.output.exists():
        raise SystemExit(f"Refusing to overwrite existing output: {args.output}")

    accepted = {int(value) for value in args.accepted.split(",") if value.strip()}
    reviewed: dict[tuple[str, int], tuple[int, bool]] = {}
    for manifest in args.manifests:
        with manifest.open(newline="") as handle:
            for row in csv.DictReader(handle):
                source_label = Path(row["label"])
                try:
                    rel = source_label.relative_to(args.source)
                except ValueError:
                    rel = Path("labels") / source_label.parent.name / source_label.name
                review_id = int(row["review_id"])
                reviewed[(rel.as_posix(), int(row["row_index"]))] = (review_id, review_id in accepted)

    report = Counter()
    for image in sorted((args.source / "images").glob("*/*")):
        if image.is_file():
            link_or_copy(image, args.output / image.relative_to(args.source))
            report["images"] += 1

    for label in sorted((args.source / "labels").glob("*/*.txt")):
        rel = label.relative_to(args.source)
        output_label = args.output / rel
        output_label.parent.mkdir(parents=True, exist_ok=True)
        kept = []
        for row_index, raw in enumerate(label.read_text().splitlines()):
            fields = raw.split()
            if not fields:
                continue
            cls = int(fields[0])
            if cls != 3:
                kept.append(raw)
                report[f"class_{cls}_kept"] += 1
                continue
            decision = reviewed.get((rel.as_posix(), row_index))
            if decision is None:
                report["class_3_unreviewed_removed"] += 1
                continue
            review_id, keep = decision
            if keep:
                kept.append(raw)
                report["class_3_kept"] += 1
            else:
                report["class_3_removed"] += 1
        output_label.write_text("\n".join(kept) + ("\n" if kept else ""))
        report["labels"] += 1

    yaml_source = args.source / "data.yaml"
    if yaml_source.exists():
        shutil.copy2(yaml_source, args.output / "data.yaml")

    report_data = {
        "source": str(args.source),
        "output": str(args.output),
        "accepted_review_ids": sorted(accepted),
        "reviewed_entries": len(reviewed),
        "counts": dict(report),
    }
    (args.output.parent / "boundary_manual_curation.json").write_text(
        json.dumps(report_data, indent=2) + "\n")
    print(json.dumps(report_data, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
