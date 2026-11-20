"""rec_core: the pure part of OUR observer's ZED recorder (v0.1.40, ROADMAP WP9b) - standard library only, no ZED, no OpenCV.

multi_buoy_observer.py --record DIR [--label L] [--max-min N] records what the camera sees while it detects:
  DIR/<YYYYmmdd_HHMMSS>_<label>/cam.svo2       the ZED SDK's own recording (H.264): both images + the IMU, re-opened by the SDK like
                                               a camera (`--svo FILE` replays it through the same loop, depth recomputed)
  DIR/<session>/frames.jsonl                   one line per recorded frame: t (UTC), the observer's frame id and the SVO's own
                                               frame number, the ZED image timestamp, the boat's lat / lon / heading from the
                                               Pixhawk telemetry the observer already reads, and the buoys it found (colour, conf,
                                               range / bearing, bbox, geolocation)
  DIR/<session>/session.json                   label, start / end, versions (kit, ZED SDK, camera, model), the options, why it stopped
The recording stops (the observer keeps detecting, cv_publish keeps following it) at --max-min or when the disk holding DIR has
less than --min-free-gb (5 GB; the AGX has ~26 GB) - the reason goes into session.json and the observer's latest_status.json
(`recording`), and `scripts/agx_record.sh status` prints it (the OCS page's RECORDER card runs that).

CLI (stdlib python3, also on the Mac):  rec_core.py status DIR              the newest session: label, frames, size, running / why stopped
                                        rec_core.py compare REC.jsonl REPLAY.jsonl   the recorded detections vs a --svo replay's
"""
from __future__ import annotations

import json
import os
import re
import shutil
import sys
import time
from datetime import datetime, timezone
from typing import Callable, Dict, List, Optional

LABEL_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]{0,39}$")
MIN_FREE_GB = 5.0
SVO_NAME, FRAMES_NAME, SESSION_NAME = "cam.svo2", "frames.jsonl", "session.json"
TEL_KEYS = ("armed", "mode", "gps_fix", "satellites", "relative_alt_m", "position_age_s", "attitude_age_s", "heartbeat_age_s")


def check_label(label: str) -> str:
    """A label is a folder-name part: letters, digits, _ and -, at most 40, starting with a letter or digit."""
    label = (label or "").strip()
    if not LABEL_RE.match(label):
        raise ValueError(f"bad label {label!r}: letters, digits, _ and - only (1-40, no spaces), e.g. pool_t1_gate or day2_course_run1")
    return label


def session_name(label: str, now: Optional[datetime] = None) -> str:
    return f"{(now or datetime.now()).strftime('%Y%m%d_%H%M%S')}_{check_label(label)}"


def free_gb(path: str, usage: Callable = shutil.disk_usage) -> float:
    """Free GB on the disk holding path (its nearest existing parent)."""
    p = os.path.abspath(path)
    while not os.path.exists(p) and os.path.dirname(p) != p:
        p = os.path.dirname(p)
    return usage(p).free / 1e9


class Guard:
    """When to stop recording: after max_min minutes (0 = no limit), or when the disk has < min_free_gb (checked every every_s)."""

    def __init__(self, path: str, max_min: float = 0.0, min_free_gb: float = MIN_FREE_GB, every_s: float = 5.0,
                 clock: Optional[Callable[[], float]] = None, free_fn: Optional[Callable[[str], float]] = None):
        self.path, self.max_min, self.min_free_gb, self.every_s = path, float(max_min or 0.0), float(min_free_gb), float(every_s)
        self.clock = clock or time.monotonic
        self.free_fn = free_fn or (lambda p: free_gb(p))                          # looked up at call time (a test's stub)
        self.t0 = self.clock()
        self.t_disk = -1e18
        self.free = None

    def start_refusal(self) -> str:
        """'' when there is room to start, else why not."""
        self.free = self.free_fn(self.path)
        if self.free < self.min_free_gb:
            return f"disk low: {self.free:.1f} GB free < {self.min_free_gb:g} GB on {self.path} - nothing recorded (bring clips home, then free space)"
        return ""

    def check(self) -> str:
        now = self.clock()
        if self.max_min > 0 and now - self.t0 >= self.max_min * 60.0:
            return f"max_min: {self.max_min:g} min recorded"
        if now - self.t_disk >= self.every_s:
            self.t_disk = now
            self.free = self.free_fn(self.path)
            if self.free < self.min_free_gb:
                return f"disk_low: {self.free:.1f} GB free < {self.min_free_gb:g} GB"
        return ""


def heading_deg(tel: dict) -> Optional[float]:
    import math
    y = tel.get("yaw_rad")
    return None if y is None else round(math.degrees(float(y)) % 360.0, 2)


def tel_summary(tel: dict) -> dict:
    """What of PixhawkTelemetry.snapshot() is kept per frame (the monotonic clock stamps are dropped: meaningless off the AGX)."""
    out = {"lat": tel.get("lat"), "lon": tel.get("lon"), "heading_deg": heading_deg(tel), "yaw_rad": tel.get("yaw_rad")}
    out.update({k: (round(tel[k], 3) if isinstance(tel.get(k), float) else tel.get(k)) for k in TEL_KEYS if k in tel})
    return out


def buoy_brief(b: dict, bbox: Optional[List[float]] = None) -> dict:
    keep = ("index", "colour", "confidence", "range_m", "surface_range_m", "bearing_deg", "elev_deg", "forward_m", "right_m", "down_m",
            "bearing_only", "reason", "buoy_lat", "buoy_lon", "pixel", "led", "model_class")   # v0.1.91: + the LED, the model's class
    out = {k: b[k] for k in keep if k in b}
    if bbox is not None:
        out["bbox"] = [round(float(v), 1) for v in bbox]
    return out


def frame_record(utc: str, frame: int, svo_frame: int, ts_ns: Optional[int], tel: dict, fresh: bool, status: str,
                 buoys: List[dict], boxes: Optional[List[Optional[List[float]]]] = None) -> dict:
    """One line of frames.jsonl."""
    boxes = boxes or [None] * len(buoys)
    s = tel_summary(tel or {})
    return {"t": utc, "frame": int(frame), "svo_frame": int(svo_frame), "ts_ns": ts_ns, "lat": s["lat"], "lon": s["lon"],
            "heading_deg": s["heading_deg"], "tel_fresh": bool(fresh), "status": status, "n_buoys": len(buoys),
            "buoys": [buoy_brief(b, bb) for b, bb in zip(buoys, boxes)], "tel": s}


def kit_version(root: str) -> str:
    try:
        with open(os.path.join(root, "optiprimex", "__init__.py"), encoding="utf-8") as fh:
            m = re.search(r'__version__\s*=\s*"([^"]+)"', fh.read())
        return m.group(1) if m else "?"
    except OSError:
        return "?"


def write_json_atomic(path: str, obj: dict) -> None:
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(obj, fh, indent=1)
    os.replace(tmp, path)


class Session:
    """The bookkeeping of one recording (the ZED calls stay in the observer): the folder, session.json, frames.jsonl, the stop."""

    def __init__(self, root: str, label: str, versions: dict, options: dict, now: Optional[datetime] = None):
        self.label = check_label(label)
        self.name = session_name(self.label, now)
        self.dir = os.path.join(root, self.name)
        os.makedirs(self.dir, exist_ok=False)
        self.svo = os.path.join(self.dir, SVO_NAME)
        self.frames = 0
        self.active = True
        self.stop_reason = ""
        self.doc = {"session": self.name, "label": self.label, "start_utc": datetime.now(timezone.utc).isoformat(), "end_utc": None,
                    "state": "recording", "stop_reason": "", "frames": 0, "svo": SVO_NAME, "frames_file": FRAMES_NAME,
                    "versions": versions, "options": options, "pid": os.getpid()}
        write_json_atomic(os.path.join(self.dir, SESSION_NAME), self.doc)
        self._fh = open(os.path.join(self.dir, FRAMES_NAME), "a", encoding="utf-8")

    def add(self, rec: dict) -> None:
        if not self.active:
            return
        self._fh.write(json.dumps(rec, separators=(",", ":")) + "\n")
        self.frames += 1
        if self.frames % 15 == 0:
            self._fh.flush()

    def stop(self, reason: str) -> None:
        if not self.active:
            return
        self.active = False
        self.stop_reason = reason
        self._fh.close()
        size = os.path.getsize(self.svo) if os.path.exists(self.svo) else 0
        self.doc.update(state="stopped", stop_reason=reason, end_utc=datetime.now(timezone.utc).isoformat(), frames=self.frames,
                        svo_bytes=size)
        write_json_atomic(os.path.join(self.dir, SESSION_NAME), self.doc)

    def public(self) -> dict:
        return {"session": self.name, "label": self.label, "active": self.active, "frames": self.frames, "stop_reason": self.stop_reason}


# ---- v0.1.84.4: RGB + depth pairs ------------------------------------------------------------------------------------------------
PAIRS_DIR, PAIRS_NAME = "pairs", "pairs.jsonl"


class Pairs:
    """Dmitrii 2026-11-11: "record also at least 1 depth image along its RGB counterpart if I check the box". While recording, the
    observer saves <session>/pairs/rgb_NNNNNN.jpg + depth_NNNNNN.png (uint16, millimetres, 0 = no depth) + depth_NNNNNN_vis.jpg (coloured,
    near red -> far blue) from the SAME grab, the first one at once, then every every_s; one pairs.jsonl line each. The pixel writing is
    the observer's (OpenCV); this class decides when and names the files. every_s 0 = off."""

    def __init__(self, session_dir: str, every_s: float, clock: Optional[Callable[[], float]] = None):
        self.dir = os.path.join(session_dir, PAIRS_DIR)
        self.every_s = float(every_s or 0.0)
        self.clock = clock or time.monotonic
        self.n = 0
        self.last_t: Optional[float] = None
        self.last: Optional[dict] = None
        self._fh = None
        if self.every_s > 0:
            os.makedirs(self.dir, exist_ok=True)
            self._fh = open(os.path.join(self.dir, PAIRS_NAME), "a", encoding="utf-8")

    def due(self) -> bool:
        return self._fh is not None and (self.last_t is None or self.clock() - self.last_t >= self.every_s)

    def names(self) -> Dict[str, str]:
        i = self.n + 1
        return {"rgb": os.path.join(self.dir, f"rgb_{i:06d}.jpg"), "depth": os.path.join(self.dir, f"depth_{i:06d}.png"),
                "vis": os.path.join(self.dir, f"depth_{i:06d}_vis.jpg")}

    def add(self, rec: dict) -> None:
        """After the observer wrote the three files of names(): count it and write its line."""
        self.n += 1
        self.last_t = self.clock()
        self.last = dict(rec, i=self.n)
        self._fh.write(json.dumps(self.last, separators=(",", ":")) + "\n")
        self._fh.flush()

    def public(self) -> dict:
        return {"on": self._fh is not None, "every_s": self.every_s, "n": self.n, "dir": self.dir if self._fh else None}

    def close(self) -> None:
        if self._fh is not None:
            self._fh.close()
            self._fh = None


# ---- replay ---------------------------------------------------------------------------------------------------------------------
class ReplayTelemetry:
    """The telemetry recorded beside an SVO (frames.jsonl), served by the SVO's frame number, so a replay geolocates as live."""

    def __init__(self, frames_path: str):
        self.by_frame: Dict[int, dict] = {}
        if frames_path and os.path.isfile(frames_path):
            for line in open(frames_path, encoding="utf-8"):
                try:
                    r = json.loads(line)
                except ValueError:
                    continue
                self.by_frame[int(r.get("svo_frame", -1))] = r.get("tel") or {}

    def __len__(self) -> int:
        return len(self.by_frame)

    def at(self, svo_frame: int) -> dict:
        return dict(self.by_frame.get(int(svo_frame), {}))


def replay_output(svo_path: str) -> str:
    """A replay never writes where cv_publish looks: <the SVO's folder>/replay/ (the observer adds its <stamp>/ run folder)."""
    return os.path.join(os.path.dirname(os.path.abspath(svo_path)), "replay")


def _load(path: str) -> Dict[int, dict]:
    out = {}
    for line in open(path, encoding="utf-8"):
        try:
            r = json.loads(line)
        except ValueError:
            continue
        out[int(r.get("svo_frame", r.get("frame", -1)))] = r
    return out


def compare(rec_path: str, replay_path: str, range_tol_m: float = 0.5, bearing_tol_deg: float = 2.0) -> dict:
    """The live detections vs the replay's, frame by frame (by the SVO frame number): same count, same colours (as multisets),
    the ranged buoys' range / bearing within tolerance (nearest first). The replay decodes the H.264 images, so a few frames
    differ - read the share, not a single frame."""
    a, b = _load(rec_path), _load(replay_path)
    common = sorted(set(a) & set(b))
    same_n = same_col = within = 0
    worst_r = worst_b = 0.0
    for k in common:
        ba, bb = a[k].get("buoys") or [], b[k].get("buoys") or []
        if len(ba) == len(bb):
            same_n += 1
        if sorted(x.get("colour", "") for x in ba) == sorted(x.get("colour", "") for x in bb):
            same_col += 1
        ra = sorted((x for x in ba if x.get("range_m") is not None), key=lambda x: x["range_m"])
        rb = sorted((x for x in bb if x.get("range_m") is not None), key=lambda x: x["range_m"])
        ok = len(ra) == len(rb)
        for x, y in zip(ra, rb):
            dr, db = abs(x["range_m"] - y["range_m"]), abs(x.get("bearing_deg", 0.0) - y.get("bearing_deg", 0.0))
            worst_r, worst_b = max(worst_r, dr), max(worst_b, db)
            ok = ok and dr <= range_tol_m and db <= bearing_tol_deg
        within += ok
    n = len(common)
    share = lambda c: round(c / n, 3) if n else 0.0  # noqa: E731
    return {"frames_recorded": len(a), "frames_replayed": len(b), "frames_compared": n, "same_count": share(same_n),
            "same_colours": share(same_col), "ranges_within_tol": share(within), "worst_range_diff_m": round(worst_r, 3),
            "worst_bearing_diff_deg": round(worst_b, 3), "range_tol_m": range_tol_m, "bearing_tol_deg": bearing_tol_deg}


# ---- status (agx_record.sh status) ----------------------------------------------------------------------------------------------
def pid_alive(pid) -> bool:
    try:
        os.kill(int(pid), 0)
        return True
    except (OSError, TypeError, ValueError):
        return False


def sessions(root: str) -> List[str]:
    if not os.path.isdir(root):
        return []
    return sorted(d for d in os.listdir(root) if os.path.isfile(os.path.join(root, d, SESSION_NAME)))


def status_text(root: str, min_free_gb: float = MIN_FREE_GB, free_fn: Optional[Callable[[str], float]] = None) -> str:
    free = (free_fn or free_gb)(root)
    head = f"recordings {root}: {len(sessions(root))} session(s), {free:.1f} GB free" + (
        f"  WARNING: < {min_free_gb:g} GB - a recording will not start; bring clips home (agx_pull_clips.sh) and free space" if free < min_free_gb else "")
    ss = sessions(root)
    if not ss:
        return head + "\nno session yet"
    d = os.path.join(root, ss[-1])
    try:
        doc = json.load(open(os.path.join(d, SESSION_NAME), encoding="utf-8"))
    except (OSError, ValueError) as exc:
        return head + f"\nlatest {ss[-1]}: unreadable session.json ({exc})"
    size = os.path.getsize(os.path.join(d, SVO_NAME)) / 1e6 if os.path.exists(os.path.join(d, SVO_NAME)) else 0.0
    n = sum(1 for _ in open(os.path.join(d, FRAMES_NAME), encoding="utf-8")) if os.path.exists(os.path.join(d, FRAMES_NAME)) else 0
    if doc.get("state") == "recording" and pid_alive(doc.get("pid")):
        state = "RECORDING"
    elif doc.get("state") == "recording":
        state = "stopped (the observer ended without closing the session: Ctrl-C / killed - the SVO may be cut short)"
    else:
        state = f"stopped: {doc.get('stop_reason') or '?'}"
    return head + f"\nlatest {ss[-1]} (label {doc.get('label')}, since {doc.get('start_utc')}): {state} · {n} frames · {size:.0f} MB"


def main(argv: Optional[List[str]] = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if len(argv) >= 2 and argv[0] == "status":
        print(status_text(argv[1]))
        return 0
    if len(argv) >= 3 and argv[0] == "compare":
        r = compare(argv[1], argv[2])
        print(json.dumps(r, indent=1))
        return 0 if r["frames_compared"] else 1
    if len(argv) >= 2 and argv[0] == "label":
        try:
            print(check_label(argv[1]))
            return 0
        except ValueError as exc:
            print(exc, file=sys.stderr)
            return 2
    print(__doc__)
    return 2


if __name__ == "__main__":
    sys.exit(main())
