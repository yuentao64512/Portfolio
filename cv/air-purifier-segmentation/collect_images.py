#!/usr/bin/env python3
"""Collect diverse air-purifier images and retain source metadata."""

from __future__ import annotations

import argparse
import hashlib
import io
import json
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from threading import Lock
from urllib.request import Request, urlopen

from ddgs import DDGS
from PIL import Image, ImageOps


NEGATIVE_WORDS = {
    "filter replacement", "replacement filter", "filter only", "manual", "parts",
    "diagram", "logo", "icon", "clipart", "blueprint", "infographic", "comparison chart",
}


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--queries", type=Path, default=Path(__file__).with_name("queries.txt"))
    parser.add_argument("--output", type=Path, default=Path(__file__).with_name("raw"))
    parser.add_argument("--per-query", type=int, default=25)
    parser.add_argument("--workers", type=int, default=12)
    parser.add_argument("--max-images", type=int, default=800)
    return parser.parse_args()


def dhash(image: Image.Image) -> int:
    gray = image.convert("L").resize((9, 8), Image.Resampling.LANCZOS)
    pixels = list(gray.getdata())
    bits = 0
    for y in range(8):
        for x in range(8):
            bits = (bits << 1) | (pixels[y * 9 + x] > pixels[y * 9 + x + 1])
    return bits


def download(candidate: dict) -> tuple[bytes, int, int, int] | None:
    request = Request(candidate["image"], headers={"User-Agent": "Mozilla/5.0 dataset-research/1.0"})
    try:
        with urlopen(request, timeout=15) as response:
            content_type = response.headers.get("Content-Type", "")
            if "image" not in content_type.lower():
                return None
            payload = response.read(20_000_001)
        if len(payload) > 20_000_000:
            return None
        with Image.open(io.BytesIO(payload)) as opened:
            image = ImageOps.exif_transpose(opened).convert("RGB")
        width, height = image.size
        if min(width, height) < 320 or max(width, height) > 12_000:
            return None
        image.thumbnail((1600, 1600), Image.Resampling.LANCZOS)
        normalized = io.BytesIO()
        image.save(normalized, "JPEG", quality=92, optimize=True)
        return normalized.getvalue(), image.width, image.height, dhash(image)
    except Exception:
        return None


def main() -> int:
    args = arguments()
    args.output.mkdir(parents=True, exist_ok=True)
    queries = [line.strip() for line in args.queries.read_text().splitlines() if line.strip()]
    candidates: list[dict] = []
    seen_urls: set[str] = set()

    for number, query in enumerate(queries, 1):
        print(f"Search {number}/{len(queries)}: {query}")
        try:
            results = DDGS().images(query, max_results=args.per_query, safesearch="moderate")
            for result in results:
                image_url = result.get("image")
                title = result.get("title", "")
                if not image_url or image_url in seen_urls:
                    continue
                if any(word in title.lower() for word in NEGATIVE_WORDS):
                    continue
                seen_urls.add(image_url)
                candidates.append({"query": query, **result})
        except Exception as exc:
            print(f"  search warning: {exc}")

    print(f"Downloading {len(candidates)} unique candidate URLs")
    metadata_path = args.output / "sources.jsonl"
    existing_hashes: list[int] = []
    accepted = 0
    lock = Lock()

    with metadata_path.open("w", encoding="utf-8") as metadata:
        with ThreadPoolExecutor(max_workers=args.workers) as pool:
            future_map = {pool.submit(download, item): item for item in candidates}
            for future in as_completed(future_map):
                if accepted >= args.max_images:
                    continue
                item = future_map[future]
                result = future.result()
                if result is None:
                    continue
                payload, width, height, fingerprint = result
                with lock:
                    if any((fingerprint ^ old).bit_count() <= 4 for old in existing_hashes):
                        continue
                    digest = hashlib.sha256(payload).hexdigest()
                    filename = f"{digest[:16]}.jpg"
                    (args.output / filename).write_bytes(payload)
                    existing_hashes.append(fingerprint)
                    record = {
                        "file": filename,
                        "sha256": digest,
                        "dhash": f"{fingerprint:016x}",
                        "width": width,
                        "height": height,
                        "query": item.get("query"),
                        "title": item.get("title"),
                        "image_url": item.get("image"),
                        "page_url": item.get("url"),
                        "source": item.get("source"),
                        "license": "unknown; review before redistribution or commercial use",
                    }
                    metadata.write(json.dumps(record, ensure_ascii=False) + "\n")
                    accepted += 1
                    if accepted % 25 == 0:
                        print(f"  accepted {accepted}")

    print(f"Accepted {accepted} distinct images in {args.output}")
    return 0 if accepted else 1


if __name__ == "__main__":
    raise SystemExit(main())
