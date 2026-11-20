"""v0.1.84.7: scripts/svo2_mp4.py reads a ZED .svo2 (MCAP, uncompressed chunks) without the SDK: the side_by_side H.264 stream after
the ZED prefix, the IMU count, the real rate from frames.jsonl ("t")."""
from __future__ import annotations

import importlib.util
import json
import struct

from .conftest import ROOT


def _mod():
    spec = importlib.util.spec_from_file_location("svo2_mp4", ROOT / "scripts" / "svo2_mp4.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _rec(op, body):
    return bytes([op]) + struct.pack("<Q", len(body)) + body


def _s(x):
    b = x.encode()
    return struct.pack("<I", len(b)) + b


def _chan(cid, topic):
    return _rec(0x04, struct.pack("<HH", cid, 0) + _s(topic) + _s("zed_sdk_encoded") + struct.pack("<I", 0))


def _msg(cid, data):
    return _rec(0x05, struct.pack("<HIQQ", cid, 0, 0, 0) + data)


def test_extract_the_h264_from_a_synthetic_svo2(tmp_path):
    m = _mod()
    nal = b"\x00\x00\x00\x01\x67\x4d\x0c\x32" + b"\x11" * 20
    inner = _chan(2, "Camera_SN1/side_by_side") + _chan(3, "Camera_SN1/sensors") + _msg(2, b"\x8c\xe9\x09\x00\x50\xe9\x09\x00" + nal) + \
        _msg(3, b'{"imu":1}') + _msg(2, b"\x01\x02\x03\x04\x05\x06\x07\x08" + nal)
    chunk = struct.pack("<QQQI", 0, 0, len(inner), 0) + _s("") + struct.pack("<Q", len(inner)) + inner
    d = tmp_path / "20261111_000000_obs"
    d.mkdir()
    f = d / "cam.svo2"
    f.write_bytes(m.MAGIC + _rec(0x06, chunk))
    (d / "frames.jsonl").write_text("\n".join(json.dumps({"t": f"2026-11-11T00:00:{i:02d}+00:00"}) for i in range(0, 11)) + "\n")
    out = tmp_path / "x.h264"
    info = m.extract(str(f), str(out))
    assert info["frames"] == 2 and info["imu"] == 1 and out.read_bytes() == nal + nal          # the ZED prefix stripped
    assert m.frames_rate(str(f)) == 1.0
    (tmp_path / "bad.svo2").write_bytes(b"nope")
    try:
        list(m.walk(str(tmp_path / "bad.svo2")))
        raise AssertionError("not refused")
    except ValueError as exc:
        assert "not an MCAP" in str(exc)


def test_the_first_nal_tells_h264_from_hevc():
    """v0.1.86.3 (WP47): the observer records H.265 now - the converter reads the codec off the stream."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("svo2_mp4_nal", str(__import__("pathlib").Path(__file__).resolve().parents[1] / "scripts" / "svo2_mp4.py"))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    assert [m.nal_codec(b) for b in (0x67, 0x09, 0x65, 0x06, 0x28)] == ["h264"] * 5           # SPS, AUD, IDR, SEI, PPS
    assert [m.nal_codec(b) for b in (0x40, 0x42, 0x44, 0x26, 0x46)] == ["hevc"] * 5           # VPS, SPS, PPS, IDR_W_RADL, AUD
