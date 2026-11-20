"""frame_rec: JPEG frames + frames.jsonl from a camera's HTTP snapshot, for training (v0.1.60, WP10m - the OCS CAMERA tab).

Dmitrii 2026-11-07: the format is JPEG frames + jsonl; the folders are an allow-list of roots, never a typed path:
  mac_logs    <kit>/logs/recordings                                   (the Mac: the simulator's frames, any camera the tab shows)
  cv_staging  ~/Documents/BH1/RobotX/recordings  (CV_STAGING_DIR)    (the Mac: CV_TRAIN's import staging - OUTSIDE CV_TRAIN; CV_TRAIN
                                                                      itself is never written: it imports from here)
  agx_rec     ~/Desktop/dm3/recordings           (AGX_REC_DIR)       (the AGX: beside the ZED recorder's sessions, outside the kit)
A session is <root>/<YYYY-MM-DD>/<HHMMSS>_<label>/ with frame_000001.jpg ... (+ depth_000001.jpg when the depth mode runs),
frames.jsonl (one line per frame: i, t, file, depth_file, the camera's /status age, the detections of that moment, the pose) and
session.json (label, camera, rate, start / stop, frames, bytes, why it stopped). Only new pictures are written (a stalled camera
gives the same bytes: skipped and counted). It stops at max_min, on stop(), or when the disk has < MIN_FREE_GB left.

  python -m optiprimex.tools.frame_rec start LABEL --cam http://127.0.0.1:8090/ --root agx_rec [--hz 2] [--max-min 10] [--depth]
         [--dets-mqtt HOST:PORT]    (the AGX: in tmux via scripts/agx_frames.sh; Ctrl-C or the stop file ends it)
  python -m optiprimex.tools.frame_rec status --root agx_rec      (the JSON the page shows)
  python -m optiprimex.tools.frame_rec stop --root agx_rec
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys
import threading
import time
import urllib.request
from typing import Callable, Dict, List, Optional

KIT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
LABEL_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")
MIN_FREE_GB = 2.0
ROOT_INFO = {"mac_logs": ("Mac · logs/recordings", "mac"), "cv_staging": ("Mac · CV_TRAIN staging (~/Documents/BH1/RobotX/recordings)", "mac"),
             "agx_rec": ("AGX · recordings (AGX_REC_DIR)", "agx")}


def root_path(root: str, env: Optional[Dict[str, str]] = None) -> str:
    """The allow-list: a root NAME -> its folder. Anything else raises (a typed path is never accepted)."""
    e = os.environ if env is None else env
    if root == "mac_logs":
        return os.path.join(KIT, "logs", "recordings")
    if root == "cv_staging":
        return os.path.expanduser(e.get("CV_STAGING_DIR") or "~/Documents/BH1/RobotX/recordings")
    if root == "agx_rec":
        return os.path.expanduser(e.get("AGX_REC_DIR") or "~/Desktop/dm3/recordings")
    raise ValueError(f"root: one of {', '.join(ROOT_INFO)} (a typed path is refused)")


def check_label(label: str) -> str:
    if not LABEL_RE.match(label or ""):
        raise ValueError("label: 1-40 letters, digits, _ or - (e.g. day1_t1_gate)")
    return label


def free_gb(path: str) -> Optional[float]:
    p = path
    while p and not os.path.exists(p):
        p = os.path.dirname(p)
    try:
        return round(shutil.disk_usage(p or "/").free / 1e9, 1)
    except OSError:
        return None


def _get(url: str, timeout: float = 2.0) -> Optional[bytes]:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read() if r.status == 200 else None
    except Exception:
        return None



FRAME_WIDTH = 640                     # v0.1.86.3 (WP47): the Mac's training frames at 640 px wide (0 = as served)


def shrink(jpeg: bytes, width: int) -> bytes:
    """A JPEG narrowed to `width` px (Pillow when installed; else unchanged - never fails a frame)."""
    if not width:
        return jpeg
    try:
        import io
        from PIL import Image
        im = Image.open(io.BytesIO(jpeg))
        if im.width <= width:
            return jpeg
        im = im.resize((width, int(round(im.height * width / im.width))), Image.LANCZOS)
        out = io.BytesIO()
        im.save(out, "JPEG", quality=85)
        return out.getvalue()
    except Exception:
        return jpeg

class Recorder:
    """One recording at a time. dets_fn() -> the detections of the moment (the page's /dets.json view); pose_fn() -> the boat's
    position / heading (status). Both optional; clock / fetch injectable for the tests."""

    def __init__(self, cam_url: Callable[[], str], dets_fn: Optional[Callable[[], dict]] = None, pose_fn: Optional[Callable[[], dict]] = None,
                 env: Optional[Dict[str, str]] = None, fetch: Callable[[str], Optional[bytes]] = _get, clock=time.time):
        self.cam_url, self.dets_fn, self.pose_fn, self.env, self.fetch, self.clock = cam_url, dets_fn, pose_fn, env, fetch, clock
        self.lock = threading.Lock()
        self.stop_ev = threading.Event()
        self.th: Optional[threading.Thread] = None
        self.st: dict = {"state": "idle"}

    # ---- control ------------------------------------------------------------------------------------------------------
    def start(self, label: str, root: str, hz: float = 2.0, max_min: float = 10.0, depth: bool = False) -> dict:
        label = check_label(label)
        base = root_path(root, self.env)
        if not 0.2 <= float(hz) <= 15.0:
            raise ValueError("hz: 0.2 .. 15")
        if not 0 <= float(max_min) <= 120:
            raise ValueError("max_min: 0 .. 120 (0 = until stop)")
        with self.lock:
            if self.th and self.th.is_alive():
                raise RuntimeError(f"already recording {self.st.get('label')} - stop it first")
            fg = free_gb(base)
            if fg is not None and fg < MIN_FREE_GB:
                raise RuntimeError(f"disk_low: {fg} GB free under {base} (< {MIN_FREE_GB} GB)")
            t = self.clock()
            d = os.path.join(base, time.strftime("%Y-%m-%d", time.localtime(t)), time.strftime("%H%M%S", time.localtime(t)) + "_" + label)
            os.makedirs(d, exist_ok=False)
            self.stop_ev.clear()
            self.st = {"state": "recording", "label": label, "root": root, "dir": d, "hz": float(hz), "max_min": float(max_min),
                       "depth": bool(depth), "cam": self.cam_url(), "t0": t, "frames": 0, "repeats": 0, "misses": 0, "bytes": 0,
                       "why": "", "depth_frames": 0, "depth_misses": 0}          # v0.1.84.4: the depth counters
            self._session(d)
            self.th = threading.Thread(target=self._loop, daemon=True)
            self.th.start()
            return self.status()

    def stop(self, why: str = "stopped") -> dict:
        self.stop_ev.set()
        th = self.th
        if th:
            th.join(timeout=5.0)
        with self.lock:
            if self.st.get("state") == "recording":
                self.st.update(state="idle", why=self.st.get("why") or why, t1=self.clock())
                self._session(self.st["dir"])
        return self.status()

    def snapshot(self, label: str, root: str) -> dict:
        """One picture now into <root>/<date>/snapshots/<HHMMSS>_<label>.jpg (+ its line in snapshots.jsonl)."""
        label = check_label(label)
        base = root_path(root, self.env)
        j = self.fetch(self.cam_url().rstrip("/") + "/snapshot.jpg")
        if not j:
            raise RuntimeError("no picture from the camera")
        t = self.clock()
        d = os.path.join(base, time.strftime("%Y-%m-%d", time.localtime(t)), "snapshots")
        os.makedirs(d, exist_ok=True)
        f = os.path.join(d, time.strftime("%H%M%S", time.localtime(t)) + f"_{label}.jpg")
        with open(f, "wb") as fh:
            fh.write(j)
        with open(os.path.join(d, "snapshots.jsonl"), "a") as fh:
            fh.write(json.dumps({"t": t, "file": os.path.basename(f), "label": label, "cam": self.cam_url(),
                                 "dets": self._dets(), "pose": self._pose()}) + "\n")
        return {"ok": True, "file": f, "bytes": len(j)}

    # ---- the loop -------------------------------------------------------------------------------------------------------
    def _dets(self):
        try:
            return self.dets_fn() if self.dets_fn else None
        except Exception:
            return None

    def _pose(self):
        try:
            return self.pose_fn() if self.pose_fn else None
        except Exception:
            return None

    def _loop(self) -> None:
        st = self.st
        period = 1.0 / st["hz"]
        last = None
        base = st["cam"].rstrip("/")
        with open(os.path.join(st["dir"], "frames.jsonl"), "a") as jl:
            while not self.stop_ev.is_set():
                t0 = self.clock()
                if st["max_min"] > 0 and t0 - st["t0"] >= st["max_min"] * 60:
                    st["why"] = f"max {st['max_min']:g} min"
                    break
                j = self.fetch(base + "/snapshot.jpg")
                if j is None:
                    st["misses"] += 1
                else:
                    h = hashlib.sha1(j).digest()
                    if h == last:
                        st["repeats"] += 1                  # the camera did not move on: nothing new to learn from
                    else:
                        last = h
                        i = st["frames"] + 1
                        name = f"frame_{i:06d}.jpg"
                        j = shrink(j, st.get("width", FRAME_WIDTH))       # v0.1.86.3 (WP47): the Mac frames at 640 px
                        with open(os.path.join(st["dir"], name), "wb") as fh:
                            fh.write(j)
                        dname = None
                        if st["depth"]:
                            dj = self.fetch(base + "/depth.jpg")
                            if dj:
                                dname = f"depth_{i:06d}.jpg"
                                with open(os.path.join(st["dir"], dname), "wb") as fh:
                                    fh.write(dj)
                                st["bytes"] += len(dj)
                                st["depth_frames"] = st.get("depth_frames", 0) + 1
                            else:
                                st["depth_misses"] = st.get("depth_misses", 0) + 1
                        jl.write(json.dumps({"i": i, "t": round(t0, 3), "file": name, "depth_file": dname, "dets": self._dets(),
                                             "pose": self._pose()}) + "\n")
                        jl.flush()
                        st["frames"], st["bytes"] = i, st["bytes"] + len(j)
                        if i % 20 == 0:
                            fg = free_gb(st["dir"])
                            if fg is not None and fg < MIN_FREE_GB:
                                st["why"] = f"disk_low: {fg} GB free"
                                break
                self.stop_ev.wait(max(0.0, period - (self.clock() - t0)))
        with self.lock:
            if st.get("state") == "recording":
                st.update(state="idle", why=st.get("why") or "stopped", t1=self.clock())
                self._session(st["dir"])

    def _session(self, d: str) -> None:
        from optiprimex import __version__
        s = {k: v for k, v in self.st.items() if k != "state"}
        s.update(version=__version__, format="jpeg frames + frames.jsonl (v0.1.60)")
        with open(os.path.join(d, "session.json"), "w") as fh:
            json.dump(s, fh, indent=1)

    def status(self) -> dict:
        st = dict(self.st)
        if st.get("t0"):
            st["elapsed_s"] = round((st.get("t1") or self.clock()) - st["t0"], 1)
            st["mb"] = round(st.get("bytes", 0) / 1e6, 1)
            st["free_gb"] = free_gb(st["dir"])
        return st


def sessions(root: str, n: int = 8, env: Optional[Dict[str, str]] = None) -> List[dict]:
    """The newest sessions under a root (newest first): folder, label, frames, MB."""
    base = root_path(root, env)
    out = []
    for day in sorted((d for d in os.listdir(base) if re.match(r"^\d{4}-\d{2}-\d{2}$", d)), reverse=True) if os.path.isdir(base) else []:
        for s in sorted(os.listdir(os.path.join(base, day)), reverse=True):
            p = os.path.join(base, day, s)
            if s == "snapshots" or not os.path.isfile(os.path.join(p, "session.json")):
                continue
            try:
                with open(os.path.join(p, "session.json")) as fh:
                    meta = json.load(fh)
            except (OSError, ValueError):
                meta = {}
            out.append({"dir": os.path.join(day, s), "label": meta.get("label"), "frames": meta.get("frames"),
                        "mb": round((meta.get("bytes") or 0) / 1e6, 1), "why": meta.get("why")})
            if len(out) >= n:
                return out
    return out


# ---- the AGX's CLI (scripts/agx_frames.sh) --------------------------------------------------------------------------------------
def _status_file(root: str) -> str:
    return os.path.join(root_path(root), ".frame_rec_status.json")


def _mqtt_dets(spec: str, vehicle: str = "usv"):
    """--dets-mqtt HOST:PORT: the latest local/<v>/perception/frame_dets body (as the page's DetsTap)."""
    import paho.mqtt.client as mqtt
    host, port = spec.rsplit(":", 1)
    last: Dict[str, dict] = {}
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"frame-rec-{os.getpid()}", protocol=mqtt.MQTTv311)
    c.on_connect = lambda cl, u, f, rc, p=None: cl.subscribe(f"local/{vehicle}/perception/frame_dets", 0)

    def on_msg(cl, u, msg):
        try:
            last["b"] = (json.loads(msg.payload).get("body") or {})
        except ValueError:
            pass
    c.on_message = on_msg
    c.connect_async(host, int(port), keepalive=15)
    c.loop_start()
    return lambda: last.get("b")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("do", choices=["start", "status", "stop", "sessions"])
    ap.add_argument("label", nargs="?", default="")
    ap.add_argument("--root", default="agx_rec", choices=list(ROOT_INFO))
    ap.add_argument("--cam", default=os.environ.get("CAM_LOCAL_URL", "http://127.0.0.1:8090/"))
    ap.add_argument("--hz", type=float, default=2.0)
    ap.add_argument("--max-min", type=float, default=10.0)
    ap.add_argument("--depth", action="store_true")
    ap.add_argument("--dets-mqtt", default="", help="HOST:PORT of the boat broker: record the detections per frame")
    a = ap.parse_args(argv)
    sf = _status_file(a.root)
    if a.do == "status":
        try:
            with open(sf) as fh:
                print(fh.read())
        except OSError:
            print(json.dumps({"state": "idle", "why": "never recorded here"}))
        return 0
    if a.do == "sessions":
        print(json.dumps(sessions(a.root)))
        return 0
    if a.do == "stop":
        open(sf + ".stop", "w").close()
        for _ in range(40):
            time.sleep(0.25)
            if not os.path.exists(sf + ".stop"):
                break
        print(open(sf).read() if os.path.exists(sf) else "{}")
        return 0
    rec = Recorder(lambda: a.cam, dets_fn=_mqtt_dets(a.dets_mqtt) if a.dets_mqtt else None)
    try:
        os.makedirs(root_path(a.root), exist_ok=True)
        rec.start(a.label, a.root, a.hz, a.max_min, a.depth)
    except (ValueError, RuntimeError, OSError) as e:
        print(json.dumps({"state": "idle", "why": f"refused: {e}"}))
        return 2
    if os.path.exists(sf + ".stop"):
        os.remove(sf + ".stop")
    print(f"[frame_rec] recording {rec.st['dir']} from {a.cam} at {a.hz:g} Hz", flush=True)
    try:
        while rec.th and rec.th.is_alive():
            with open(sf, "w") as fh:
                json.dump(rec.status(), fh)
            if os.path.exists(sf + ".stop"):
                rec.stop("stopped (page)")
                os.remove(sf + ".stop")
                break
            time.sleep(1.0)
    except KeyboardInterrupt:
        rec.stop("stopped (Ctrl-C)")
    s = rec.stop()
    with open(sf, "w") as fh:
        json.dump(s, fh)
    print(f"[frame_rec] {s.get('frames')} frames, {s.get('mb')} MB - {s.get('why')}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
