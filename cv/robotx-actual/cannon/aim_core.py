"""aim_core - the cannon link's pure part (stdlib only): the target file, the aim gate, the dock_color_aim.v1 body.

v0.1.61 (WP12-rest, decided with Dmitrii 2026-11-07): the students' receiver (pi/color_cannon_receiver.py, a verbatim snapshot) is
the contract. It accepts only `protocol: "dock_color_aim.v1"` and a target with class_name "dock", an allowed colour, confidence,
color_fraction, x_right_m, y_forward_m, z_up_m and range_m (all finite). It aims the servos at every OK packet and, in pump mode
only, fires once after a stable lock. So the target must be the real window, measured, and only while the mission aims:

  the observer that holds the ZED (cv/buoys_multi, cv/docking, cv/sim_observer with --cannon COLOUR)
      -> aim_cv.find_target (their colour segmentation + depth, ported) -> write_target(<target file>)     [one camera owner]
  the mission manager's local/<v>/cannon/aim {on, bay, window, window_id, t}                                [the gate]
      -> AimGate: target only when the aim is on and < aim_fresh_s old AND the target file is < target_fresh_s old and of the
         aimed colour; else target null with the reason as `status` (the receiver then centres the servos, pump off)
      -> cannon_sender.py signs it (HMAC-SHA256, the sorted-keys JSON body) and sends it to the Pi by UDP.

Colour: RED by default (handbook 3.3.4: the target window lights RED and turns GREEN when hit); `--color green` for the students'
bench target. Coordinates as theirs: x_right_m + starboard, y_forward_m + forward, z_up_m + up, from the cannon pivot.
"""
from __future__ import annotations

import json
import math
import os
import socket
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional, Tuple

PROTOCOL = "dock_color_aim.v1"          # the students' receiver checks exactly this
LEGACY_PROTOCOL = "dock_cannon.v1"      # our v0.1.57 packet (kept behind --protocol for the fake Pi's old checks)
TARGET_FILE_PROTOCOL = "cannon_target.v1"
AIM_FRESH_S = 1.0
TARGET_FRESH_S = 0.5
COLOURS = ("red", "green")

# HSV bands (OpenCV: H 0..180). Two bands per colour as in their green_hsv (red wraps round 0). Overridable: cannon_config.json colour_hsv.
HSV = {
    "red": {"hue_low_1": 0, "hue_high_1": 10, "hue_low_2": 170, "hue_high_2": 180, "saturation_min": 120, "value_min": 90},
    "green": {"hue_low_1": 40, "hue_high_1": 85, "hue_low_2": 40, "hue_high_2": 85, "saturation_min": 100, "value_min": 80},
}
# The mount, their camera_to_cannon section (color_dock_config.json is not in the tree yet: zeros until the students send it).
MOUNT0 = {"camera_yaw_deg": 0.0, "camera_forward_from_cannon_m": 0.0, "camera_right_from_cannon_m": 0.0, "camera_up_from_cannon_m": 0.0}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def hsv_for(colour: str, cfg: Optional[dict] = None) -> dict:
    if colour not in COLOURS:
        raise ValueError(f"colour {colour!r}: one of {COLOURS}")
    return dict(HSV[colour], **((cfg or {}).get("colour_hsv", {}).get(colour, {})))


def mount_of(cfg: Optional[dict]) -> dict:
    return dict(MOUNT0, **((cfg or {}).get("camera_to_cannon") or {}))


def cannon_coordinates(forward_m: float, right_m: float, down_m: float, mount: dict) -> dict:
    """Camera frame (forward, right, down) -> the cannon frame. Verbatim the students' color_dock_sender.cannon_coordinates."""
    yaw = math.radians(float(mount["camera_yaw_deg"]))
    c, s = math.cos(yaw), math.sin(yaw)
    forward = forward_m * c - right_m * s
    right = forward_m * s + right_m * c
    y_forward = forward + float(mount["camera_forward_from_cannon_m"])
    x_right = right + float(mount["camera_right_from_cannon_m"])
    z_up = -down_m + float(mount["camera_up_from_cannon_m"])
    horizontal = math.hypot(x_right, y_forward)
    return {
        "x_right_m": x_right, "y_forward_m": y_forward, "z_up_m": z_up, "horizontal_m": horizontal,
        "range_m": math.hypot(horizontal, z_up),
        "bearing_deg": math.degrees(math.atan2(x_right, y_forward)),
        "elevation_deg": math.degrees(math.atan2(z_up, horizontal)),
    }


def target_record(colour: str, fraction: float, pixels: int, bbox, coords: dict, **extra) -> dict:
    """One measured component -> the receiver's target (their candidate dict; `green_pixels` generalised to `color_pixels`)."""
    return {"class_name": "dock", "color": colour, "confidence": min(1.0, 0.5 + 0.5 * float(fraction)),
            "color_fraction": float(fraction), "color_pixels": int(pixels), "bbox_xyxy": list(bbox), **extra, **coords}


# ---- the target file (observer -> sender), atomic like the observers' latest_status.json --------------------------------
def write_target(path, status: str, target: Optional[dict], colour: str, source: str) -> dict:
    rec = {"protocol": TARGET_FILE_PROTOCOL, "sent_unix_ns": time.time_ns(), "sent_utc": utc_now(), "source": source,
           "colour": colour, "status": status, "target": target}
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_name(p.name + ".tmp")
    tmp.write_text(json.dumps(rec, indent=1, default=float))
    os.replace(tmp, p)
    return rec


def read_target(path) -> Optional[dict]:
    try:
        rec = json.loads(Path(path).read_text())
    except (OSError, ValueError):
        return None
    return rec if isinstance(rec, dict) and rec.get("protocol") == TARGET_FILE_PROTOCOL else None


# ---- the gate: the aim x the target -> one dock_color_aim.v1 body per packet -------------------------------------------------
class AimGate:
    """Pure: the caller gives the times (monotonic for the aim, wall ns for the target file's stamp), so the fast SIL drives it on
    its own clock. status: NO_AIM | AIM_STALE | AIM_OFF | NO_TARGET (aim on, no fresh measured target of the colour) | OK."""

    def __init__(self, session: str, colour: str = "red", aim_fresh_s: float = AIM_FRESH_S, target_fresh_s: float = TARGET_FRESH_S,
                 host: Optional[str] = None, protocol: str = PROTOCOL):
        if colour not in COLOURS:
            raise ValueError(f"colour {colour!r}")
        self.session, self.colour, self.protocol = session, colour, protocol
        self.aim_fresh_s, self.target_fresh_s = float(aim_fresh_s), float(target_fresh_s)
        self.host = host or socket.gethostname()
        self.aim: Optional[dict] = None
        self.aim_t = -1e18
        self.rec: Optional[dict] = None
        self.sequence = 0
        self.why_target = "no target file"

    def on_aim(self, body: dict, now: float) -> None:
        if isinstance(body, dict):
            self.aim, self.aim_t = body, now

    def on_target(self, rec: Optional[dict]) -> None:
        self.rec = rec if isinstance(rec, dict) else None

    def target(self, now_ns: int) -> Tuple[Optional[dict], str]:
        r = self.rec
        if r is None:
            return None, "no target file"
        age = (now_ns - int(r.get("sent_unix_ns") or 0)) / 1e9
        if age > self.target_fresh_s or age < -0.5:
            return None, f"target file {age:.1f} s old"
        t = r.get("target")
        if r.get("status") != "OK" or not isinstance(t, dict):
            return None, f"observer: {r.get('status')}"
        if t.get("color") != self.colour:
            return None, f"observer colour {t.get('color')} (aiming {self.colour})"
        return t, "ok"

    def state(self, now: float, now_ns: Optional[int] = None) -> str:
        if self.aim is None:
            return "NO_AIM"
        if now - self.aim_t > self.aim_fresh_s:
            return "AIM_STALE"
        if not self.aim.get("on"):
            return "AIM_OFF"
        t, self.why_target = self.target(time.time_ns() if now_ns is None else now_ns)
        return "OK" if t is not None else "NO_TARGET"

    def body(self, now: float, now_ns: Optional[int] = None) -> dict:
        now_ns = time.time_ns() if now_ns is None else now_ns
        st = self.state(now, now_ns)
        target = None
        if st == "OK":
            a = self.aim
            target = dict(self.target(now_ns)[0], bay=a.get("bay"), window=a.get("window"), window_id=a.get("window_id"))
        self.sequence += 1
        return {"protocol": self.protocol, "session": self.session, "sequence": self.sequence - 1, "sent_unix_ns": now_ns,
                "sent_utc": utc_now(), "source_host": self.host, "status": st, "target": target, "color": self.colour,
                "window_id": target.get("window_id") if target else None, "aim_utc": (self.aim or {}).get("t"),
                "why": None if st == "OK" else (self.why_target if st == "NO_TARGET" else st.lower())}
