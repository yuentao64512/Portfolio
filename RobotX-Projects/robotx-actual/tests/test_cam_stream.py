"""The AGX camera stream (ROADMAP WP5): cv/stream/cam_stream.py serves the newest observer JPEG as MJPEG over HTTP, stdlib only
(the dir source; our AGX venv has no OpenCV) - page, /status, /snapshot.jpg, /stream parts that follow the file, stale files
ignored; the launcher wiring (run_agx.sh window cam, network.env CAM_PORT, map.html's camera link)."""
from __future__ import annotations

import json
import os
import time
import urllib.request

from .conftest import PY, ROOT, Proc, free_port, wait_until

CAM = ROOT / "cv" / "stream" / "cam_stream.py"


def _jpeg(tag: bytes) -> bytes:
    return b"\xff\xd8\xff\xe0" + tag * 50 + b"\xff\xd9"         # the server checks the SOI marker only


def _get(url: str, timeout: float = 3.0) -> bytes:
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def test_dir_source_serves_page_status_snapshot_and_stream(tmp_path):
    port = free_port()
    f = tmp_path / "latest_preview.jpg"
    f.write_bytes(_jpeg(b"A"))
    p = Proc([PY, str(CAM), "--source", "dir", "--dir", str(tmp_path / "*.jpg"), "--port", str(port), "--bind", "127.0.0.1", "--hz", "10"],
             tmp_path / "cam.log")
    base = f"http://127.0.0.1:{port}"
    try:
        assert wait_until(lambda: _ok(base + "/status"), 8), p.log()
        st = json.loads(_get(base + "/status"))
        assert st["source"] == "dir" and st["frames"] >= 1 and st["from"].endswith("latest_preview.jpg") or "latest_preview.jpg" in st["from"]
        assert _get(base + "/snapshot.jpg") == _jpeg(b"A")
        assert b"USV camera" in _get(base + "/") and b'src="/stream"' in _get(base + "/")
        r = urllib.request.urlopen(base + "/stream", timeout=5)
        assert r.headers["Content-Type"].startswith("multipart/x-mixed-replace; boundary=")
        first = _read_part(r)
        assert first == _jpeg(b"A")
        time.sleep(0.05)
        f.write_bytes(_jpeg(b"B"))
        os.utime(f, None)
        assert _read_part(r) == _jpeg(b"B")                                  # the stream follows the newest frame
        assert json.loads(_get(base + "/status"))["clients"] == 1
        r.close()
        old = tmp_path / "old.jpg"                                          # a stale file is never served
        old.write_bytes(_jpeg(b"C"))
        os.utime(old, (time.time() - 120, time.time() - 120))
        os.utime(f, (time.time() - 120, time.time() - 120))
        time.sleep(0.5)
        assert _get(base + "/snapshot.jpg") == _jpeg(b"B")
    finally:
        p.kill()


def _ok(url: str) -> bool:
    try:
        return json.loads(_get(url, 1.0)).get("frames", 0) >= 1
    except Exception:
        return False


def _read_part(r) -> bytes:
    """One multipart part: boundary line, headers, the JPEG of Content-Length bytes."""
    line = r.readline()
    while not line.startswith(b"--"):
        line = r.readline()
    n = 0
    while True:
        h = r.readline().strip()
        if not h:
            break
        if h.lower().startswith(b"content-length:"):
            n = int(h.split(b":")[1])
    return r.read(n)


def test_camera_wiring_in_the_launchers():
    agx = (ROOT / "scripts" / "run_agx.sh").read_text()
    assert "-n cam" in agx and "--no-cam" in agx and "stream/run.sh --source" in agx
    assert "cam_stream" in (ROOT / "scripts" / "stop_agx.sh").read_text()
    html = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert "Q.get('cam')" in html and 'id="camlink"' in html
    os.environ["CAM_URL"] = "http://100.114.195.19:8090/"
    try:
        from optiprimex.tools.bench_tui import map_url
        assert map_url("127.0.0.1", 9001).endswith("&cam=http://100.114.195.19:8090/")
    finally:
        del os.environ["CAM_URL"]
    import subprocess
    out = subprocess.run(["bash", str(ROOT / "cv" / "stream" / "run.sh"), "--help"], capture_output=True, text=True, timeout=20).stdout
    assert "MJPEG" in out and "--source dir|zed|test" in out
