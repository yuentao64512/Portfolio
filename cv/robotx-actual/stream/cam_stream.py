#!/usr/bin/env python3
"""cam_stream: the AGX camera as an MJPEG stream over HTTP, for the Mac's browser (ROADMAP WP5; decided 2026-11-02: MJPEG
over HTTP - any browser, no GStreamer on the Mac). Independent of the detectors and of MQTT; read-only on everything.

  http://AGX_HOST:CAM_PORT/            a page with the stream, the source and the frame age
  http://AGX_HOST:CAM_PORT/stream      multipart/x-mixed-replace MJPEG at --hz
  http://AGX_HOST:CAM_PORT/snapshot.jpg the latest frame        /status  JSON (source, frames, age, clients, bytes)

Sources (--source):
  dir   (default) the newest of the observers' preview JPEGs - cv/buoys/runs/*/latest_preview.jpg (every frame, 960 px, since v0.1.84.8; was every 15th),
        cv/docking/runs/latest/latest_combined.jpg (every 5th), cv/buoys/runs/latest_sender_preview.jpg - so it runs beside the
        detectors, which hold the ZED. Stdlib only (our paho venv is enough); frames are sent as written (re-scaled to --width
        when OpenCV happens to be importable).
  zed   the ZED left image through pyzed (only when no observer holds the camera); needs the students' venv (pyzed + cv2,
        used read-only through AGX_BUOY_VENV, see cv/stream/run.sh).
  test  a moving test pattern with the time (needs cv2) - proves the link and measures the cost without a camera.

v0.1.60 (WP10m, the OCS CAMERA tab; Dmitrii 2026-11-07: depth only when no detector holds the camera): modes rgb | depth | both.
  POST /mode {"mode": ...}   switch (409 + why when depth is not available: the dir source = a detector holds the ZED)
  GET  /depth.jpg            the depth coloured (near red -> far blue, nothing black; the same stops as optiprimex/tools/depthviz)
  GET  /depth_at?x=&y=       the range (m) at the normalised position (0..1 from the top left), a 5 x 5 median
  /status gains mode, modes, depth {available, why}. zed: the camera is reopened with DEPTH_MODE.PERFORMANCE the first time a depth
  mode is asked (images-only until then); test: a synthetic depth ramp (so the link and the page are testable without a camera).
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import signal
import socketserver
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import List, Optional

HERE = os.path.dirname(os.path.abspath(__file__))
CV = os.path.dirname(HERE)
DIR_PATTERNS = [os.path.join(CV, "buoys", "runs", "*", "latest_preview.jpg"), os.path.join(CV, "docking", "runs", "latest", "latest_combined.jpg"),
                os.path.join(CV, "buoys", "runs", "latest_sender_preview.jpg")]
BOUNDARY = "optiprimeframe"

cv2 = np = None
if os.environ.get("CAM_NO_OPENCV") != "1":    # run.sh sets it for dir: the AGX's system cv2 is built against another numpy
    try:                                # optional: scaling and the zed / test sources
        import cv2                      # type: ignore
        import numpy as np              # type: ignore
    except Exception:                   # noqa: BLE001 - any import problem means "no OpenCV here"
        cv2 = np = None


class Frames:
    """The latest JPEG and its metadata, shared by the producer thread and the HTTP handlers."""

    def __init__(self):
        self.cond = threading.Condition()
        self.jpeg: Optional[bytes] = None
        self.n = 0
        self.t = 0.0
        self.origin = ""
        self.clients = 0
        self.sent_bytes = 0
        self.mode = "rgb"                       # v0.1.60: rgb | depth | both
        self.depth = None                       # the latest depth map (np float32, m) when a depth mode runs
        self.depth_jpeg: Optional[bytes] = None
        self.depth_ok, self.depth_why = False, "not started"

    def put(self, jpeg: bytes, origin: str) -> None:
        with self.cond:
            self.jpeg, self.t, self.origin = jpeg, time.time(), origin
            self.n += 1
            self.cond.notify_all()

    def status(self, source: str) -> dict:
        with self.cond:
            return {"source": source, "frames": self.n, "age_s": round(time.time() - self.t, 2) if self.n else None, "from": self.origin,
                    "clients": self.clients, "bytes_sent": self.sent_bytes, "last_frame_bytes": len(self.jpeg or b""), "opencv": cv2 is not None,
                    "mode": self.mode, "modes": ["rgb", "depth", "both"] if self.depth_ok else ["rgb"],
                    "depth": {"available": self.depth_ok, "why": self.depth_why}}

    def put_depth(self, depth) -> None:
        img = colourise(depth)
        ok, buf = cv2.imencode(".jpg", img[:, :, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 70])
        with self.cond:
            self.depth = depth
            if ok:
                self.depth_jpeg = buf.tobytes()


# v0.1.60: the depth colours - the same stops as optiprimex/tools/depthviz (near red -> far blue; nothing black)
NEAR_M, FAR_M = 0.5, 25.0
STOPS = ((0.0, (220, 40, 30)), (0.25, (250, 200, 40)), (0.5, (70, 200, 90)), (0.75, (40, 180, 220)), (1.0, (40, 60, 200)))


def colourise(depth):
    d = np.asarray(depth, dtype=np.float32)
    valid = np.isfinite(d) & (d > 0)
    f = np.clip((np.where(valid, d, FAR_M) - NEAR_M) / (FAR_M - NEAR_M), 0.0, 1.0)
    xs = np.array([s[0] for s in STOPS], np.float32)
    out = np.zeros(d.shape + (3,), np.uint8)
    for c in range(3):
        out[..., c] = np.interp(f, xs, np.array([s[1][c] for s in STOPS], np.float32)).astype(np.uint8)
    out[~valid] = 0
    return out                                  # RGB


def depth_at(depth, x: float, y: float):
    if depth is None or not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0):
        return None
    h, w = depth.shape[:2]
    u, v = min(w - 1, int(x * w)), min(h - 1, int(y * h))
    patch = np.asarray(depth[max(0, v - 2):v + 3, max(0, u - 2):u + 3], np.float32)
    ok = patch[np.isfinite(patch) & (patch > 0)]
    return None if ok.size == 0 else round(float(np.median(ok)), 2)


def scale(jpeg: bytes, width: int) -> bytes:
    if cv2 is None or width <= 0:
        return jpeg
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None or img.shape[1] <= width:
        return jpeg
    h = int(img.shape[0] * width / img.shape[1])
    ok, buf = cv2.imencode(".jpg", cv2.resize(img, (width, h), interpolation=cv2.INTER_AREA), [cv2.IMWRITE_JPEG_QUALITY, 70])
    return buf.tobytes() if ok else jpeg


# ---- producers -------------------------------------------------------------------------------------------------------------
def run_dir(frames: Frames, a, stop: threading.Event) -> None:
    frames.depth_ok, frames.depth_why = False, ("a detector holds the camera (source dir): depth only when no detector runs - stop the "
                                                "observer, then scripts/agx_cam.sh start --source zed")
    patterns: List[str] = a.dir or DIR_PATTERNS
    last = (None, None)
    while not stop.is_set():
        files = [f for p in patterns for f in glob.glob(p)]
        if files:
            newest = max(files, key=lambda f: os.stat(f).st_mtime)
            st = os.stat(newest)
            if (newest, st.st_mtime) != last and time.time() - st.st_mtime < a.max_age_s:
                try:
                    with open(newest, "rb") as fh:
                        data = fh.read()
                    if data[:2] == b"\xff\xd8":       # a complete JPEG (the observers write in place; skip a half-written one)
                        frames.put(scale(data, a.width), os.path.relpath(newest, CV))
                        last = (newest, st.st_mtime)
                except OSError:
                    pass
        stop.wait(1.0 / a.hz)


def run_test(frames: Frames, a, stop: threading.Event) -> None:
    if cv2 is None:
        raise SystemExit("--source test needs OpenCV (run with the students' venv: cv/stream/run.sh --source test)")
    w, h = a.width or 640, int((a.width or 640) * 9 / 16)
    k = 0
    frames.depth_ok, frames.depth_why = True, ""
    ramp = np.tile(np.linspace(30.0, 1.0, h, dtype=np.float32)[:, None], (1, w))      # far at the top, near at the bottom
    ramp[: h // 6] = np.inf                                                           # the sky: no return
    while not stop.is_set():
        if frames.mode != "rgb":
            frames.put_depth(ramp)
        img = np.zeros((h, w, 3), np.uint8)
        x = int((k * 8) % w)
        cv2.rectangle(img, (x, h // 3), (x + 60, 2 * h // 3), (0, 160, 255), -1)
        cv2.putText(img, time.strftime("%H:%M:%S") + f"  test pattern  frame {k}", (16, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            frames.put(buf.tobytes(), "test pattern")
        k += 1
        stop.wait(1.0 / a.hz)


def run_zed(frames: Frames, a, stop: threading.Event) -> None:
    if cv2 is None:
        raise SystemExit("--source zed needs OpenCV + pyzed (the students' venv: cv/stream/run.sh --source zed)")
    import pyzed.sl as sl  # type: ignore
    cam = sl.Camera()
    init = sl.InitParameters()
    init.camera_resolution = sl.RESOLUTION.HD720
    init.camera_fps = 15
    init.depth_mode = sl.DEPTH_MODE.NONE                      # images only: no depth work for the stream (until a depth mode)
    init.coordinate_units = sl.UNIT.METER
    err = cam.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        raise SystemExit(f"ZED open failed: {err} (an observer holding the camera? use --source dir)")
    frames.depth_ok, frames.depth_why = True, ""
    mat, dmat, with_depth = sl.Mat(), sl.Mat(), False
    try:
        while not stop.is_set():
            t0 = time.time()
            if frames.mode != "rgb" and not with_depth:        # v0.1.60: the first depth request reopens the camera with depth
                cam.close()
                init.depth_mode = sl.DEPTH_MODE.PERFORMANCE
                err = cam.open(init)
                if err != sl.ERROR_CODE.SUCCESS:
                    frames.depth_ok, frames.depth_why, frames.mode = False, f"ZED reopen with depth failed: {err}", "rgb"
                    init.depth_mode = sl.DEPTH_MODE.NONE
                    if cam.open(init) != sl.ERROR_CODE.SUCCESS:
                        raise SystemExit(f"ZED reopen failed: {err}")
                else:
                    with_depth = True
            if cam.grab() == sl.ERROR_CODE.SUCCESS:
                if with_depth and frames.mode != "rgb":
                    cam.retrieve_measure(dmat, sl.MEASURE.DEPTH)
                    d = dmat.get_data()
                    if a.width and d.shape[1] > a.width:
                        d = cv2.resize(d, (a.width, int(d.shape[0] * a.width / d.shape[1])), interpolation=cv2.INTER_NEAREST)
                    frames.put_depth(d)
                cam.retrieve_image(mat, sl.VIEW.LEFT)
                img = mat.get_data()[:, :, :3]
                if a.width and img.shape[1] > a.width:
                    img = cv2.resize(img, (a.width, int(img.shape[0] * a.width / img.shape[1])), interpolation=cv2.INTER_AREA)
                ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    frames.put(buf.tobytes(), "zed left")
            stop.wait(max(0.0, 1.0 / a.hz - (time.time() - t0)))
    finally:
        cam.close()


# ---- HTTP ----------------------------------------------------------------------------------------------------------------
PAGE = """<!doctype html><html><head><meta charset="utf-8"><title>USV camera</title>
<style>body{margin:0;background:#0d1117;color:#c9d1d9;font:13px ui-monospace,Menlo,monospace}header{padding:6px 10px;background:#161b22}
img{display:block;max-width:100%;margin:0 auto}#s{color:#8b949e}</style></head><body>
<header><b>USV camera</b> (AGX, source {SOURCE}, {HZ} Hz) <span id="s"></span></header><img src="/stream" alt="waiting for frames">
<script>setInterval(()=>fetch('/status').then(r=>r.json()).then(j=>{document.getElementById('s').textContent=
` frames ${j.frames} · age ${j.age_s==null?'-':j.age_s+' s'} · ${j.from||''} · clients ${j.clients}`}).catch(()=>{}),1000)</script></body></html>"""


def make_handler(frames: Frames, a):
    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):            # quiet: one line per client in the stream loop instead
            pass

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                body = PAGE.replace("{SOURCE}", a.source).replace("{HZ}", f"{a.hz:g}").encode()
                self._send(200, "text/html; charset=utf-8", body)
            elif path == "/status":
                self._send(200, "application/json", json.dumps(frames.status(a.source)).encode())
            elif path == "/snapshot.jpg":
                with frames.cond:
                    j = frames.jpeg
                self._send(200, "image/jpeg", j) if j else self._send(503, "text/plain", b"no frame yet")
            elif path == "/stream":
                self.stream()
            elif path == "/depth.jpg":                          # v0.1.60 (WP10m)
                with frames.cond:
                    j = frames.depth_jpeg if frames.mode != "rgb" else None
                self._send(200, "image/jpeg", j) if j else self._send(503, "text/plain", (frames.depth_why or "no depth mode").encode())
            elif path == "/depth_at":
                from urllib.parse import parse_qs, urlparse
                q = parse_qs(urlparse(self.path).query)
                try:
                    x, y = float((q.get("x") or ["-1"])[0]), float((q.get("y") or ["-1"])[0])
                except ValueError:
                    return self._send(400, "application/json", b'{"ok": false, "why": "x, y: numbers 0..1"}')
                with frames.cond:
                    d = frames.depth
                r = depth_at(d, x, y) if (d is not None and np is not None) else None
                self._send(200, "application/json", json.dumps({"ok": d is not None, "x": x, "y": y, "range_m": r,
                                                                "near_m": NEAR_M, "far_m": FAR_M}).encode())
            else:
                self._send(404, "text/plain", b"not found")

        def do_POST(self):
            """v0.1.60: POST /mode {"mode": "rgb|depth|both"}; depth only when the source can give it (never with a detector on the ZED)."""
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._send(400, "application/json", b'{"ok": false, "why": "bad json"}')
            if self.path != "/mode":
                return self._send(404, "application/json", b'{"ok": false, "why": "unknown"}')
            mode = str(body.get("mode", ""))
            if mode not in ("rgb", "depth", "both"):
                return self._send(400, "application/json", b'{"ok": false, "why": "mode: rgb, depth or both"}')
            if mode != "rgb" and not frames.depth_ok:
                return self._send(409, "application/json", json.dumps({"ok": False, "why": frames.depth_why}).encode())
            frames.mode = mode
            self._send(200, "application/json", json.dumps({"ok": True, "mode": mode}).encode())

        def _send(self, code, ctype, body):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def stream(self):
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            with frames.cond:
                frames.clients += 1
            print(f"[cam_stream] client {self.client_address[0]} connected", flush=True)
            last_n = -1
            try:
                while True:
                    with frames.cond:
                        frames.cond.wait_for(lambda: frames.n != last_n, timeout=5.0)
                        j, last_n = frames.jpeg, frames.n
                    if j is None:
                        continue
                    part = (f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(j)}\r\n\r\n").encode() + j + b"\r\n"
                    self.wfile.write(part)
                    self.wfile.flush()
                    with frames.cond:
                        frames.sent_bytes += len(part)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with frames.cond:
                    frames.clients -= 1
                print(f"[cam_stream] client {self.client_address[0]} left", flush=True)
    return H


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):
        """HTTPServer.server_bind does a reverse-DNS getfqdn() that can hang for many seconds (macOS, a tailnet without DNS)."""
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = str(self.server_address[0]), int(self.server_address[1])


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", choices=["dir", "zed", "test"], default="dir")
    ap.add_argument("--port", type=int, default=int(os.environ.get("CAM_PORT", "8090")))
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument("--hz", type=float, default=5.0)
    ap.add_argument("--width", type=int, default=640, help="scale frames down to this width (OpenCV only; 0 = as is)")
    ap.add_argument("--dir", action="append", help="dir source: glob pattern(s) of JPEGs to follow (default: the observers' previews)")
    ap.add_argument("--max-age-s", type=float, default=30.0, help="dir source: ignore files older than this")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    frames, stop = Frames(), threading.Event()
    producer = {"dir": run_dir, "zed": run_zed, "test": run_test}[a.source]
    th = threading.Thread(target=producer, args=(frames, a, stop), daemon=True)
    th.start()
    srv = Server((a.bind, a.port), make_handler(frames, a))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    print(f"[cam_stream] source {a.source} at {a.hz} Hz on http://{a.bind}:{a.port}/  (stream /stream, snapshot /snapshot.jpg, /status; opencv {'yes' if cv2 is not None else 'no'})", flush=True)
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
