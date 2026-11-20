#!/usr/bin/env python3
"""Range-download a useful UP-Fall video subset without fetching the full ZIP."""
from __future__ import annotations

import argparse
import io
import re
import struct
import zipfile
from pathlib import Path

import requests

URL = "https://zenodo.org/api/records/17170592/files/UP_Fall.zip/content"
ACTIVITIES = {1: "falling_down", 2: "falling_down", 3: "falling_down", 4: "falling_down", 5: "falling_down",
              6: "walking", 7: "standing", 9: "sitting", 11: "lying_down"}


class RemoteFile(io.RawIOBase):
    def __init__(self, url: str, block: int = 2_000_000):
        self.url, self.block = url, block
        h = requests.head(url, allow_redirects=True, timeout=60)
        h.raise_for_status()
        self.size = int(h.headers["Content-Length"])
        self.pos = 0
        self.cache: dict[tuple[int, int], bytes] = {}

    def seekable(self): return True
    def readable(self): return True
    def tell(self): return self.pos
    def seek(self, offset, whence=io.SEEK_SET):
        if whence == io.SEEK_SET: self.pos = offset
        elif whence == io.SEEK_CUR: self.pos += offset
        elif whence == io.SEEK_END: self.pos = self.size + offset
        else: raise ValueError(whence)
        return self.pos

    def read(self, n=-1):
        if n is None or n < 0: n = self.size - self.pos
        if n == 0 or self.pos >= self.size: return b""
        end = min(self.size, self.pos + n)
        out = bytearray()
        cursor = self.pos
        while cursor < end:
            block_start = (cursor // self.block) * self.block
            block_end = min(self.size, block_start + self.block)
            key = (block_start, block_end)
            if key not in self.cache:
                r = requests.get(self.url, headers={"Range": f"bytes={block_start}-{block_end-1}"}, timeout=120)
                r.raise_for_status()
                self.cache[key] = r.content
            a, b = cursor - block_start, min(end, block_end) - block_start
            out.extend(self.cache[key][a:b])
            cursor = block_start + b
        self.pos = end
        return bytes(out)


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, default=Path("data/UP_Fall/videos"))
    p.add_argument("--subjects", type=int, nargs="+", default=[1, 2, 3, 4, 5, 6])
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    remote = RemoteFile(URL)
    with zipfile.ZipFile(remote) as archive:
        selected = []
        for info in archive.infolist():
            name = info.filename
            m = re.search(r"Subject(\d+)Activity(\d+)Trial1Camera1_", name)
            if not m or int(m.group(1)) not in args.subjects or int(m.group(2)) not in ACTIVITIES:
                continue
            if not name.lower().endswith((".avi", ".mp4")):
                continue
            # Fall recordings are split into several temporal pieces. Keep the
            # first post-event piece to limit the public subset size; the ADL
            # recordings are already one continuous clip.
            activity = int(m.group(2))
            if activity <= 5 and not re.search(r"Camera1_1_", name):
                continue
            selected.append((info, int(m.group(1)), int(m.group(2))))
        print(f"Selected {len(selected)} UP-Fall video clips")
        manifest = []
        for info, subject, activity in selected:
            out = args.output / Path(info.filename).name
            if not out.exists() or out.stat().st_size != info.file_size:
                print(f"extract {out.name} ({info.file_size / 1e6:.1f} MB)", flush=True)
                with archive.open(info) as src, out.open("wb") as dst:
                    while True:
                        chunk = src.read(1024 * 1024)
                        if not chunk: break
                        dst.write(chunk)
            manifest.append((out, subject, activity, ACTIVITIES[activity]))
    manifest_path = args.output.parent / "segments.csv"
    with manifest_path.open("w", encoding="utf-8") as handle:
        handle.write("segment_id,group,source,video_path,label,start_sec,end_sec\n")
        for i, (path, subject, activity, label) in enumerate(manifest):
            # The archive clips are already temporally trimmed; use their full duration.
            import cv2
            cap = cv2.VideoCapture(str(path))
            fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
            frames = cap.get(cv2.CAP_PROP_FRAME_COUNT)
            cap.release()
            duration = max(0.5, frames / fps)
            handle.write(f"UPFALL_{subject}_{activity}_{i:03d},UPFALL_SUBJECT_{subject},UP_FALL,{path.resolve()},{label},0,{duration:.3f}\n")
    print(f"Wrote {len(manifest)} segments to {manifest_path}")


if __name__ == "__main__":
    main()
