#!/usr/bin/env python3
"""svo2_mp4: a ZED .svo2 recording -> an ordinary .mp4 you can watch (QuickTime / VLC), on the Mac, WITHOUT the ZED SDK
(v0.1.84.7, Dmitrii 2026-11-11: "where do logs and recordings land? ... it seems I can't watch them").

A .svo2 is an MCAP container (magic \\x89MCAP0). Its channel `Camera_SN<serial>/side_by_side` carries the camera's own H.264 stream
(both eyes stacked - the left on top - an 8-byte ZED prefix before the Annex-B start code); this copies that stream out unchanged and lets ffmpeg
wrap it (no re-encode for --eye both; a crop + encode for left / right). The depth is NOT in the file as pictures - the SDK recomputes
it from the two eyes (replay on the AGX: cv/buoys_multi/run.sh --svo FILE). Standard library + ffmpeg only (brew install ffmpeg).

  python3 scripts/svo2_mp4.py FILE.svo2 [--eye left|right|both] [--out FILE.mp4]      one file
  python3 scripts/svo2_mp4.py DIR [--eye left]                                          every */cam.svo2 under DIR (skips a done .mp4)
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import struct
import subprocess
import sys
import tempfile
from typing import Iterator, Optional, Tuple

MAGIC = b"\x89MCAP0\r\n"
OP_SCHEMA, OP_CHANNEL, OP_MESSAGE, OP_CHUNK = 0x03, 0x04, 0x05, 0x06


def _str(b: bytes, o: int) -> Tuple[str, int]:
    n = struct.unpack_from("<I", b, o)[0]
    return b[o + 4:o + 4 + n].decode(errors="replace"), o + 4 + n


def _records(buf: bytes) -> Iterator[Tuple[int, bytes]]:
    o = 0
    while o + 9 <= len(buf):
        op = buf[o]
        ln = struct.unpack_from("<Q", buf, o + 1)[0]
        yield op, buf[o + 9:o + 9 + ln]
        o += 9 + ln


def walk(path: str) -> Iterator[Tuple[str, bytes]]:
    """(topic, payload) of every message, in file order (chunks opened; only uncompressed chunks - the ZED writes those)."""
    topics = {}
    with open(path, "rb") as fh:
        if fh.read(8) != MAGIC:
            raise ValueError(f"{path}: not an MCAP / svo2 file")
        while True:
            h = fh.read(9)
            if len(h) < 9:
                return
            op, ln = h[0], struct.unpack("<Q", h[1:])[0]
            if op not in (OP_CHANNEL, OP_MESSAGE, OP_CHUNK):
                fh.seek(ln, 1)
                continue
            b = fh.read(ln)
            if len(b) < ln:
                return                                              # a file cut short (the recorder was killed): keep what came
            inner = [(op, b)]
            if op == OP_CHUNK:
                comp, o = _str(b, 28)
                if comp:
                    raise ValueError(f"{path}: compressed chunks ({comp}) are not supported here - use the ZED SDK")
                rlen = struct.unpack_from("<Q", b, o)[0]
                inner = list(_records(b[o + 8:o + 8 + rlen]))
            for iop, ib in inner:
                if iop == OP_CHANNEL:
                    cid = struct.unpack_from("<H", ib, 0)[0]
                    topics[cid] = _str(ib, 4)[0]
                elif iop == OP_MESSAGE:
                    cid = struct.unpack_from("<H", ib, 0)[0]
                    yield topics.get(cid, ""), ib[22:]


def nal_codec(b: int) -> str:
    """v0.1.86.3 (WP47: the observer records H.265 now): the first NAL header byte after the start code -> h264 | hevc
    (H.264: forbidden bit 0, nal_unit_type = b & 31 in 1..23; HEVC: type = (b >> 1) & 63, the VPS / SPS / PPS / IDR are 32-34, 19-21)."""
    t265 = (b >> 1) & 63
    if b & 0x81 == 0 and t265 in (19, 21, 32, 33, 34, 35, 39):         # not 20: 0x28 is also an H.264 PPS (nri 1)
        return "hevc"
    return "h264"


def extract(path: str, h264_out: str) -> dict:
    """Write the side_by_side H.264 stream to h264_out; -> {frames, fps, width, height, imu}."""
    n, imu, header, codec = 0, 0, {}, None
    with open(h264_out, "wb") as out:
        for topic, data in walk(path):
            if topic.endswith("/side_by_side"):
                k = data.find(b"\x00\x00\x00\x01", 0, 64)
                if k < 0:
                    continue
                if codec is None and len(data) > k + 4:
                    codec = nal_codec(data[k + 4])
                out.write(data[k:])
                n += 1
            elif topic.endswith("/sensors"):
                imu += 1
            elif topic == "svo_header" and not header:
                try:
                    header = json.loads(data.decode(errors="ignore"))
                except ValueError:
                    header = {}
    fps = None
    for k in ("fps", "FPS", "frame_rate", "camera_fps"):
        v = header.get(k) if isinstance(header, dict) else None
        if isinstance(v, (int, float)) and v > 0:
            fps = float(v)
    return {"frames": n, "fps": fps, "imu": imu, "codec": codec or "h264", "header_keys": sorted(header)[:20] if isinstance(header, dict) else []}


def frames_rate(path: str) -> Optional[float]:
    """The recorded rate from frames.jsonl beside the svo2 (the observer's own lines: utc per frame)."""
    fj = os.path.join(os.path.dirname(path), "frames.jsonl")
    try:
        from datetime import datetime
        ts = []
        with open(fj) as fh:
            for line in fh:
                j = json.loads(line)
                u = j.get("t") or j.get("utc")                     # the observer writes "t" (UTC ISO)
                if u:
                    ts.append(datetime.fromisoformat(u.replace("Z", "+00:00")).timestamp())
        if len(ts) >= 2 and ts[-1] > ts[0]:
            return round((len(ts) - 1) / (ts[-1] - ts[0]), 2)
    except (OSError, ValueError):
        pass
    return None


def convert(path: str, eye: str = "left", out: Optional[str] = None) -> str:
    ff = shutil.which("ffmpeg")
    if not ff:
        raise SystemExit("ffmpeg not found (brew install ffmpeg)")
    out = out or os.path.splitext(path)[0] + ("" if eye == "both" else f"_{eye}") + ".mp4"
    with tempfile.TemporaryDirectory() as td:
        raw = os.path.join(td, "cam.h264")
        info = extract(path, raw)
        if not info["frames"]:
            raise SystemExit(f"{path}: no video frames found")
        fps = frames_rate(path) or info["fps"] or 15.0     # the observer writes one frame per loop (~1 Hz), not the camera's 15
        # the ZED's "side_by_side" is stacked: the left eye on top, the right below (2208 x 2484 at hd2k)
        vf = {"left": ["-vf", "crop=iw:ih/2:0:0"], "right": ["-vf", "crop=iw:ih/2:0:ih/2"], "both": []}[eye]
        codec = (["-c:v", "copy"] + (["-tag:v", "hvc1"] if info["codec"] == "hevc" else [])) if eye == "both" else \
            ["-c:v", "libx264", "-preset", "veryfast", "-crf", "23", "-pix_fmt", "yuv420p"]
        cmd = [ff, "-y", "-loglevel", "error", "-r", f"{fps:g}", "-f", info["codec"], "-i", raw] + vf + codec + ["-movflags", "+faststart", out]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode != 0:
            raise SystemExit(f"ffmpeg failed: {r.stderr[-400:]}")
    print(f"{out}  ({info['frames']} frames at {fps:g} fps = {info['frames'] / fps:.0f} s real time, {eye}; IMU samples {info['imu']})",
          flush=True)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="a .svo2 file, or a folder: every */cam.svo2 under it")
    ap.add_argument("--eye", choices=("left", "right", "both"), default="left")
    ap.add_argument("--out", default=None, help="the .mp4 (one file only)")
    a = ap.parse_args(argv)
    if os.path.isdir(a.path):
        files = sorted(os.path.join(d, "cam.svo2") for d, _s, fs in os.walk(a.path) if "cam.svo2" in fs)
        for f in files:
            target = os.path.splitext(f)[0] + ("" if a.eye == "both" else f"_{a.eye}") + ".mp4"
            if os.path.exists(target) and os.path.getmtime(target) >= os.path.getmtime(f):
                print(f"{target}  (done before)")
                continue
            try:
                convert(f, a.eye)
            except (SystemExit, ValueError) as exc:
                print(f"{f}: {exc}", file=sys.stderr)
        return 0
    convert(a.path, a.eye, a.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
