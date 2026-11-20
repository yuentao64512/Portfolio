"""boat_web: the BOAT window (v0.1.45, ROADMAP WP10f phases 1-2, docs/BOAT_WINDOW_SPEC.md §5) - the USV control workspace in the
browser, beside the boat broker. Reads, never invents: usv_ui's FCU block (local/<v>/ui/fcu, 2 Hz), the mission manager's status
(team/optiprime/v1/status/usv) and local state, presence/#, the bridge ($SYS/broker/connection/+/state), ocs/state (bridged in),
the host-health beats (local/<v>/host/+), perception health and the cannon aim. Draws the persistent header, the generic five-system
wireframe (ids thr-mp thr-ms thr-sf thr-sa cam lamp ant box cannon; the page's setState(json) changes only fills, strokes,
opacities and text) and the OVERVIEW tab.

v0.1.49 (WP10k-b, docs/UI_QOL_2026-11-07.md §E): the five other tabs with live content (CONTROL + the TUNING card, ACTUATION, TELEMETRY
with a 60 s canvas plotter, NAV & SENSORS, DIAGNOSTICS), the radar (heading-up PPI of the mission manager's map digest, rings 5 / 10 /
25 / 40 m, sqrt radial scale), the compass and the speed dial, the wireframe at BlueBoat proportions (1.19 x 0.76 m) with boxed
callouts placed by `place_callouts` (no overlaps, stable slots), the camera callout naming the stream source (--cam-status: cv/stream or
the sim's /status) and every perception source with its age, the CV-model hint (tools/cv_hint).

v0.1.51 (WP10k-c item 28, ref_04): the compass is a rose - hull glyph, HDG solid, COG open, the commanded yaw dashed (NAV_CONTROLLER_OUTPUT
.nav_bearing while the FCU's own controller steers; else hatched "not published"), set / act as two numbers, the ROT scale (±30 °/s) from
usv_ui's yawspeed_dps (ATTITUDE.yawspeed); the speed dial's commanded surge tick stays hatched until a source exists.

v0.1.52 (Dmitrii 2026-11-06, the BOAT window stream): the top plate - big colour-coded VEHICLE / FCU MODE / ARM blocks (`plate()`:
AUTO family green, MANUAL yellow, KILLED / FAULT red, ARMED red, DISARMED grey, unknown hatched) and the small toggle boxes (holder,
log, cv: usv_ui's FCU block `log` / `cv_on`, its presence beat, its map's `cv.on`); the SIMULATION banner (`sim_flag()`: usv_ui's
fcu_link fake or the sim camera); EVENTS + usv_ui's CONSOLE + the page's replies on OVERVIEW with levels (`ev_level()`); the light map
(`light_map()`: north-up plan of the map digests - boat, track, buoys, CO, saved points, course edge - auto-fit, scale bar); the
wireframe's callouts reduced to a word + one value, the detail in the tooltip and the click panel (`callouts()` -> detail, tab).

The whitelist buttons send usv_ui's own words over local/<v>/ui/cmd: handover takeover stop log on/off note save cv on/off points.
usv_ui answers every word on local/<v>/ui/cmd/result; decision 25 holds on both ends: only handover takeover stop log note save co cv
points ever pass - no arm / disarm, mode, bursts, E-stop, thruster or cannon command. POST /cmd {"word": "handover"} needs the header
X-Boat-Web: 1. The TUNING card: POST /tune {"key", "value"} | {"save": true} (X-Boat-Web: 1) is passed to the OCS page's /cmd as
`tune KEY VALUE` / `tune save` - the authority stays on the OCS (cmd_authority TUNE -> cmd/usv/run SET_TUNING).

  python -m optiprimex.tools.boat_web [--boat-host 127.0.0.1] [--boat-port 1884] [--port 8098] [--bind 127.0.0.1] [--vehicle usv]
                                      [--thr-map mp=1,ms=3,sf=4,sa=2] [--ocs-url http://127.0.0.1:8096] [--cam-status URL] [--open]
  on the Mac against run_sim.sh's boat broker: window `bweb` of tmux optiprimex (run_sim.sh); the OCS page links it (USV CONTROL ↗).
"""
from __future__ import annotations

import argparse
import collections
import json
import math
import os
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import List, Optional

from ..companion import tuning as TUN
from .cv_hint import hint as cv_hint
from . import playbook

NS = "team/optiprime/v1"
HERE = os.path.dirname(os.path.abspath(__file__))
WHITELIST = ("handover", "takeover", "stop", "log", "note", "save", "co", "cv", "points",    # = usv_ui App.PAGE_WORDS (decision 25)
             "mode", "arm", "disarm")                       # v0.1.50 (Dmitrii 2026-11-06): the FCU's mode and ARM / DISARM, any holder
# v0.1.50: the FCU card's modes (ArduPilot Rover) - (label, usv_ui's `mode` argument)
FCU_MODES = (("MANUAL", "manual"), ("HOLD", "hold"), ("ACRO", "1"), ("STEERING", "3"), ("LOITER", "loiter"), ("AUTO", "auto"),
             ("RTL", "11"), ("SMART_RTL", "12"), ("GUIDED", "guided"), ("MODE 20", "20"))
DEFAULT_PIN = "0000"            # v0.1.78.9 (Dmitrii 2026-11-09): the USV window's PIN, fixed and shown in the launcher - not a secret,
                               # a guard against a stray click from another laptop (one commander at a time still)
OCS_LINES = ("rc6_policy latched", "rc6_policy hold")                     # v0.1.50: the only OCS lines the FCU card may forward (the RC6 rule)
OCS_LINES = OCS_LINES + ("stop_reset",)                                     # v0.1.54: the ■ STOP & RESET button (the OCS's macro)
OCS_LINES = OCS_LINES + ("enable_auto", "disable_auto", "confirm_safe")    # v0.1.62: the top plate's VEHICLE menu (Dmitrii 2026-11-07)
OCS_LINES = OCS_LINES + ("station_keep hold", "station_keep loiter")      # v0.1.81 (WP37): the FCU card's HOLD / LOITER switch
OCS_LINE_TIMEOUT_S = {"stop_reset": 45.0}                                    # seven steps, each waits for its acks
DETS_STALE_S = 2.0                                                           # v0.1.54: perception/frame_dets (the observer's rate, 4 Hz)
CAM_FRESH_S = 5.0                                                            # the camera's /status poll (every 2 s)
PAGE_BUTTONS = ("handover", "takeover")                                                      # v0.1.45's two (kept for the console line)
# v0.1.49 (item 20): the whitelist words as buttons - (button id, word, args, tooltip); note / save take the text field
BUTTONS = (("handover", "handover", None, "handover: usv_ui stops its tui/active beat - the mission manager may take the FCU"),
           ("takeover", "takeover", None, "takeover: usv_ui commands the FCU again - the mission manager drops to MANUAL"),
           ("stop", "stop", None, "stop: usv_ui's stop - neutral, any running console action ends (takes the FCU back if handed over)"),
           ("log-on", "log", ["on"], "log on: usv_ui starts its tlog / CSV log (logs/usv_ui)"),
           ("log-off", "log", ["off"], "log off: usv_ui closes its log"),
           ("note", "note", "text", "note TEXT: a line in usv_ui's console and its log (the text field)"),
           ("save", "save", "text", "save NAME: the boat's own fix as a point in points.json (the text field = the name; a 3D fix needed)"),
           ("cv-on", "cv", ["on"], "cv on: usv_ui folds the AGX's perception into its own map"),
           ("cv-off", "cv", ["off"], "cv off: usv_ui stops folding perception"),
           ("points", "points", None, "points: usv_ui lists its saved points (the reply below)"))
MAP_STALE_S = 2.0           # the map digest (2 Hz)
RINGS_M = (5.0, 10.0, 25.0, 40.0)     # decided 2026-11-07 (Dmitrii): 40 = the bearing-only band's outer edge
RADAR_R_M = 40.0
ROT_MAX_DPS = 30.0          # v0.1.51 (item 28): the ROT scale's ends, °/s
CMD_YAW_MODES = ("GUIDED", "AUTO", "RTL", "SMART_RTL", "LOITER")     # the FCU's own controller steers: NAV_CONTROLLER_OUTPUT is its set point
OBJ = {"red": "#d64541", "green": "#2e9e5b", "blue": "#3d7bd9", "black": "#555b63", "white": "#e8e8e8", "yellow": "#e0b400",
       "orange": "#e07b00", "unknown": "#8a8a8a"}     # object colours (exempt from the colour rule)
EKF_BITS = ((1, "attitude"), (2, "vel horiz"), (4, "vel vert"), (8, "pos horiz rel"), (16, "pos horiz abs"), (32, "pos vert abs"),
            (64, "pos vert agl"), (128, "const pos"), (256, "pred pos rel"), (512, "pred pos abs"), (1024, "UNINITIALIZED"), (32768, "GPS GLITCH"))
# the output channel of each thruster: usv_ui's MOTOR_NAMES (M1 PORT, M2 STERN, M3 STBD, M4 BOW) - to verify against the CAD and
# the firmware's output map (BOAT_WINDOW_SPEC §1); --thr-map overrides, a thruster left out is drawn dashed (unmapped)
THR_MAP = {"thr-mp": 1, "thr-ms": 3, "thr-sf": 4, "thr-sa": 2}
THR_NAME = {"thr-mp": "M-P", "thr-ms": "M-S", "thr-sf": "SW-F", "thr-sa": "SW-A"}
HB_STALE_S = 2.0            # the FCU heartbeat (OCS_V2_SPEC §2 rule 12)
BLOCK_STALE_S = 2.0         # usv_ui's FCU block (2 Hz)
STATUS_STALE_S = 1.0        # status/usv (2 Hz): weak past 1 s, lost past 5 s
STATUS_LOST_S = 5.0
OCS_STALE_S = 3.0           # ocs/state (1 Hz, bridged in)
PERC_STALE_S = 2.0
HOST_STALE_S = 3.0
AIM_STALE_S = 1.0           # the cannon aim: stale after 1 s = off (DATAFLOW C)
CAN_STALE_S = 3.0           # v0.1.57: the cannon Pi's status (2 Hz): older than this = stale
LAMP = {"RED": "#ff2d2d", "YELLOW": "#ffd400", "GREEN": "#2ecc71"}     # the light's own colours (object colours are exempt)
CMD_STALE_S = 2.0                                                         # v0.1.55: mm/state cmd older than this greys
LAMP_CODE = {0: None, 1: "RED", 2: "YELLOW", 3: "GREEN"}                 # v0.1.55 (D4): NAMED_VALUE_INT LAMP (0 off)


def U(text):                # a value that does not exist: drawn hatched, "unknown / not published"
    return {"u": text}


def W(text):
    return {"w": text}


def A(text):
    return {"a": text}


def _f(x, d=1):
    return "?" if x is None else ("%.*f" % (d, x))


def parse_thr_map(s: Optional[str]) -> dict:
    if not s:
        return dict(THR_MAP)
    out = {}
    for part in s.split(","):
        k, _, v = part.strip().partition("=")
        k = k.strip().lower()
        if k and v.strip():
            out["thr-" + k.replace("thr-", "")] = int(v)
    bad = set(out) - set(THR_NAME)
    if bad:
        raise ValueError("unknown thruster(s) %s: mp ms sf sa" % sorted(bad))
    return out


class Model:
    """The latest of every stream with its arrival time (monotonic), the boat events this page saw change, the command replies."""

    def __init__(self, vehicle: str = "usv", thr_map: Optional[dict] = None, warn_c: float = 85.0, alarm_c: float = 95.0):
        self.v = vehicle
        self.thr_map = dict(THR_MAP if thr_map is None else thr_map)
        self.warn_c, self.alarm_c = warn_c, alarm_c
        self.thr_src = "flags"              # v0.1.55 (D3): "OCS profile" once /meta.json answered
        self.last = {}                      # fcu | status | mm | ocs | perc | aim | can -> (t_recv, body)
        self.presence = {}                  # node -> (t_recv, body)
        self.hosts = {}                     # nx | agx -> (t_recv, body)
        self.bridge = None                  # (up, t_recv)
        self.connected = False
        self.broker = "?"
        self.events = collections.deque(maxlen=60)     # (t_unix, text)
        self.cmds = collections.deque(maxlen=20)       # the page's words and their replies
        self.results = {}
        self.cond = threading.Condition()
        self._prev = {}
        self.perc_by = {}                   # v0.1.49: perception source (health.detector) -> (t_recv, body)
        self.course = None                  # the last COURSE command's boundary [[lat, lon], ...] (cmd/<v>/run, bridged in)
        self.cam = None                     # (t_recv, /status body | None, url, error) from the --cam-status poller
        self.hist = collections.deque(maxlen=130)      # (t_mono, sog, hdg, pwm1..4, hb_age) per FCU block (2 Hz): 60 s + margin
        self.fixes = collections.deque(maxlen=12)      # (t_mono, lat, lon) for COG
        self._stage = None                  # (task, stage) and its heading errors for MAE / RMSE / max
        self._stage_err: List[float] = []
        self.maps = {}                      # v0.1.52: "mm" | "ui" -> (t_recv, telemetry/<v>/map body); usv_ui's carries points + cv
        self.dets = None                    # v0.1.54: (t_recv, local/<v>/perception/frame_dets body) - the observer's pixel boxes

    # ---- inbound ----
    def topics(self):
        v = self.v
        return [f"local/{v}/ui/fcu", f"local/{v}/ui/cmd/result", f"{NS}/status/{v}", f"local/{v}/mm/state", "presence/#",
                "$SYS/broker/connection/+/state", "ocs/state", f"local/{v}/host/+", f"local/{v}/perception/health", f"local/{v}/cannon/aim",
                f"local/{v}/cannon/status", f"{NS}/telemetry/{v}/map", f"{NS}/cmd/{v}/run", f"local/{v}/perception/frame_dets"]

    def on_message(self, topic: str, payload: bytes, now: Optional[float] = None) -> None:
        now = time.monotonic() if now is None else now
        if topic.startswith("$SYS/broker/connection/"):
            up = payload.decode(errors="ignore").strip() == "1"
            if self.bridge is None or self.bridge[0] != up:
                self.event("bridge to the OCS %s (%s)" % ("UP" if up else "DOWN", topic.split("/")[3]))
            self.bridge = (up, now)
            return
        try:
            env = json.loads(payload)
        except Exception:
            return
        if not isinstance(env, dict):
            return
        if topic.startswith("presence/"):
            node = topic[len("presence/"):]
            tu = env.get("t_unix")
            t = now - (time.time() - tu) if isinstance(tu, (int, float)) and time.time() - tu > 60.0 else now   # a retained old beat
            self.presence[node] = (t, env)
            return
        body = env.get("body") if isinstance(env.get("body"), dict) else None
        if body is None:
            return
        v = self.v
        if topic == f"local/{v}/ui/cmd/result":
            with self.cond:
                self.results[str(body.get("id"))] = body
                self.cond.notify_all()
            self.event("usv_ui: %s -> %s%s" % (body.get("word"), "" if body.get("ok") else "REFUSED: ", body.get("text")))
            return
        key = {f"local/{v}/ui/fcu": "fcu", f"{NS}/status/{v}": "status", f"local/{v}/mm/state": "mm", "ocs/state": "ocs",
               f"local/{v}/perception/health": "perc", f"local/{v}/cannon/aim": "aim",
               f"local/{v}/cannon/status": "can", f"{NS}/telemetry/{v}/map": "map"}.get(topic)
        if topic == f"local/{v}/perception/frame_dets":            # v0.1.54: the camera card's boxes (not an event, not in `last`)
            self.dets = (now, body)
            return
        if topic == f"{NS}/cmd/{v}/run":
            if body.get("action") == "COURSE" and isinstance((body.get("args") or {}).get("boundary"), list):
                self.course = body["args"]["boundary"]
            return
        if key == "map":
            self.maps["ui" if ("points" in body or "cv" in body) else "mm"] = (now, body)
        if key == "perc":
            self.perc_by[str(body.get("detector") or body.get("source") or "?")] = (now, body)
        if key == "fcu":
            self._sample(now, body)
        if topic.startswith(f"local/{v}/host/"):
            self.hosts[topic.rsplit("/", 1)[-1]] = (now, body)
            return
        if key is None:
            return
        self.last[key] = (now, body)
        self._changes(key, body)

    def event(self, text: str) -> None:
        self.events.appendleft((time.time(), text))

    def _changes(self, key, b):
        watch = {"fcu": (("active", lambda x: "usv_ui ACTIVE (commands the FCU)" if x else "usv_ui HANDED OVER: the mission manager may hold the FCU"),
                         ("mode", lambda x: "FCU mode %s" % x), ("armed", lambda x: "FCU %s" % ("ARMED" if x else "disarmed")),
                         ("fcu_link", lambda x: "FCU link %s" % x)),
                 "status": (("state", lambda x: "vehicle %s" % x), ("task_state", lambda x: "task stage %s" % x),
                            ("kill", lambda x: "KILL asserted" if x else "kill clear"))}.get(key, ())
        for field, fmt in watch:
            k = key + "." + field
            if field in b and self._prev.get(k, "\0") != b[field]:
                if k in self._prev:
                    self.event(fmt(b[field]))
                self._prev[k] = b[field]

    # ---- helpers ----
    def get(self, key, now):
        r = self.last.get(key)
        return (None, None) if r is None else (r[1], now - r[0])

    def hb_age(self, now):
        """The FCU heartbeat age: usv_ui's block when it is current, else the mission manager's health.fcu_age_s; None = unknown."""
        fcu, fa = self.get("fcu", now)
        if fcu is not None and fa <= BLOCK_STALE_S and fcu.get("hb_age_s") is not None:
            return fcu["hb_age_s"] + fa
        st, sa = self.get("status", now)
        if st is not None and sa <= 2.0 and (st.get("health") or {}).get("fcu_age_s") is not None:
            return st["health"]["fcu_age_s"] + sa
        return None

    # ---- the view (all the logic the page needs; the page only draws it) ----
    def thruster(self, eid, now):
        ch = self.thr_map.get(eid)
        base = {"name": THR_NAME[eid], "ch": ch, "pwm": None, "frac": 0.0, "dir": 0}
        if ch is None:
            return dict(base, state="unavailable", label="UNAVAILABLE · not mapped")
        fcu, fa = self.get("fcu", now)
        if fcu is None:
            return dict(base, state="unknown", label="unknown · no FCU block")
        hb = self.hb_age(now)
        if fa > BLOCK_STALE_S:
            return dict(base, state="unknown", label="unknown · FCU block %.0f s old" % fa)
        if hb is None or hb > HB_STALE_S:
            return dict(base, state="unknown", label="unknown · no FCU heartbeat" + ("" if hb is None else " %.0f s" % hb))
        servo = fcu.get("servo")
        if not servo:
            return dict(base, state="unknown", label="unknown · no SERVO_OUTPUT_RAW")
        pwm = servo[ch - 1] if len(servo) >= ch else None
        if not pwm:
            return dict(base, state="unavailable", label="UNAVAILABLE · ch %d no output" % ch)
        d = pwm - 1500
        frac = min(1.0, abs(d) / 400.0)
        if abs(d) <= 2:
            return dict(base, pwm=pwm, state="available", label="idle %d" % pwm)
        sat = frac > 0.9
        return dict(base, pwm=pwm, frac=round(frac, 3), dir=1 if d > 0 else -1, state="warning" if sat else "active",
                    label="%s%d%% · %d%s" % ("+" if d > 0 else "−", round(frac * 100), pwm, " SAT" if sat else ""))

    def lamp_read(self, now):
        """v0.1.55 (D4): the lamp's read-back - NAMED_VALUE_INT LAMP from usv_ui's FCU block (lamp, lamp_age_s) or the mission
        manager's fcu (lamp); None while nobody reports one (today's Lua does not send it: the window shows *expected*)."""
        f, fa = self.get("fcu", now)
        if f is not None and fa <= 2.0 and f.get("lamp") is not None and (f.get("lamp_age_s") is None or f["lamp_age_s"] <= 3.0):
            return int(f["lamp"])
        mm, ma = self.get("mm", now)
        lv = ((mm or {}).get("fcu") or {}).get("lamp")
        if mm is not None and ma <= 2.0 and lv is not None:
            return int(lv)
        return None

    def lamp(self, now):
        st, sa = self.get("status", now)
        hb = self.hb_age(now)
        out = {"colour": None, "l2": "feedback: none (expected only, no readback)"}
        if hb is None or hb > HB_STALE_S:
            return dict(out, state="unknown", l1="UNKNOWN · no FCU heartbeat" + ("" if hb is None else " %.0f s" % hb))
        if st is None or sa > 2.0:
            return dict(out, state="unknown", l1="UNKNOWN · no vehicle state" + ("" if st is None else " (status %.0f s)" % sa))
        v = st.get("state") or "?"
        rd = self.lamp_read(now)                                       # v0.1.55 (D4): a read-back wins over the expectation
        if rd is not None:
            c = LAMP_CODE.get(rd)
            return dict(out, state="read", colour=c, hex=LAMP.get(c) if c else None, l2="feedback: NAMED_VALUE_INT LAMP = %d" % rd,
                        l1="read %s · state %s" % (c or "OFF", v))
        c = "RED" if v == "KILLED" else "YELLOW" if v == "MANUAL" else "GREEN" if v.startswith("AUTO") else None
        if c is None:
            return dict(out, state="unknown", l1="UNKNOWN · no lamp rule for %s" % v)
        return dict(out, state="expected", colour=c, hex=LAMP[c], l1="expected %s · state %s" % (c, v))

    def antenna(self, now):
        st, sa = self.get("status", now)
        ocs, oa = self.get("ocs", now)
        mm, _ = self.get("mm", now)
        loss = (mm or {}).get("link_loss_action") or (st or {}).get("link_loss_action")
        l2 = "RSSI n/a (no poller) · on loss: %s" % (loss or "link_loss_action")
        br = None if self.bridge is None else self.bridge[0]
        brs = "bridge ?" if br is None else ("bridge up" if br else "bridge DOWN")
        if not self.connected:
            return {"state": "fault", "q": "lost", "l1": "LOST · this page has no boat broker (%s)" % self.broker, "l2": l2}
        if st is None:
            return {"state": "unknown", "q": "unknown", "l1": "UNKNOWN · no status/usv yet · " + brs, "l2": l2}
        if br is False or sa > STATUS_LOST_S:
            return {"state": "fault", "q": "lost", "l1": "LOST · %s · status age %.1f s" % (brs, sa), "l2": l2}
        if sa > STATUS_STALE_S or (ocs is not None and oa > OCS_STALE_S):
            return {"state": "warning", "q": "weak", "age_s": round(sa, 1), "l1": "WEAK · status age %.1f s · ocs/state %s" % (sa, "?" if ocs is None else "%.1f s" % oa), "l2": l2}
        return {"state": "active", "q": "good", "age_s": round(sa, 1), "l1": "good · %s · status age %.1f s" % (brs, sa), "l2": l2}

    def electronics(self, now):
        hb = self.hb_age(now)
        fcu, _ = self.get("fcu", now)
        st, _ = self.get("status", now)
        txt, worst, throttled, free, seen, sim = {}, 0.0, [], [], [], []
        for h in ("nx", "agx"):
            r = self.hosts.get(h)
            if r is None or now - r[0] > HOST_STALE_S:
                txt[h] = None
                continue
            b = r[1]
            seen.append(h)
            if b.get("simulated"):
                sim.append(h)
            temps = [x for x in (b.get("temp_c") or {}).values() if isinstance(x, (int, float))]
            t = max(temps) if temps else None
            txt[h] = t
            worst = max(worst, t or 0.0)
            if b.get("throttling"):
                throttled.append(h.upper())
            if b.get("disk_free_gb") is not None:
                free.append("%s %.0f GB free" % (h.upper(), b["disk_free_gb"]))
        known = fcu is not None or st is not None or bool(seen)

        def tt(h):
            return "no beat" if h not in seen else ("n/a °C" if txt[h] is None else "%.0f °C" % txt[h])
        o = {"nx": tt("nx"), "agx": tt("agx"),
             "fcu": "?" if hb is None else ("HB %.1f s" % hb if hb <= HB_STALE_S else "NO HB %.0f s" % hb),
             "l3": ("host beat 1 Hz: %s" % " ".join(x.upper() for x in seen)) if seen else "host beat: none (window host of run_usv.sh)",
             "simulated": bool(sim)}
        if sim:                                                         # v0.1.55 (D3): the bench / sim beats from the Mac
            o["l3"] = "this Mac (simulated host): %s · thresholds from %s" % (" ".join(x.upper() for x in sim), self.thr_src)
        o["l2"] = "NX %s · AGX %s" % (o["nx"], o["agx"]) + ("".join(" · " + x for x in free[:1]))
        if not known:
            return dict(o, state="unknown", l1="UNKNOWN · nothing heard from the boat")
        if (fcu is not None or st is not None) and (hb is None or hb > HB_STALE_S):
            return dict(o, state="fault", l1="ALARM · FCU link lost", hot="fcu")
        if worst >= self.alarm_c:
            return dict(o, state="fault", l1="ALARM · %.0f °C (alarm %.0f)" % (worst, self.alarm_c), hot="temp")
        if worst >= self.warn_c or throttled:
            return dict(o, state="warning", l1="WARNING · %s" % (("%s throttling" % "/".join(throttled)) if throttled else "%.0f °C (warn %.0f)" % (worst, self.warn_c)), hot="temp")
        return dict(o, state="available", worst_c=(worst or None) if seen else None, n_hosts=len(seen),
                    l1=("ok · no throttling" + (" · this Mac (simulated host)" if sim else "")) if seen else "FCU link ok · hosts unknown")

    def perception_sources(self, now):
        """Every perception source with its age (sim_observer 0.9 s · fake_perception 0.7 s), never one flipping with the other."""
        return [(k, b, now - t) for k, (t, b) in sorted(self.perc_by.items())]

    def stream(self, now):
        """The camera stream source: cv/stream's or the sim's /status (--cam-status), polled every 2 s."""
        if self.cam is None:
            return "stream: not polled (--cam-status URL)"
        t, body, url, err = self.cam
        host = url.split("//", 1)[-1].split("/", 1)[0]
        if body is None:
            return "stream: none at %s (%s)" % (host, err or "no answer")
        age = body.get("age_s")
        return "stream: %s @ %s · age %s · %s client%s%s" % (
            body.get("source") or "?", host, "?" if age is None else "%.1f s" % age, body.get("clients"),
            "" if body.get("clients") == 1 else "s", "" if now - t <= 5.0 else " (status %.0f s old)" % (now - t))

    def camview(self, now):
        """v0.1.54 (item 10): the camera card - the snapshot base derived from --cam-status (http://H:P/status -> http://H:P/), `ok`
        only while its /status answered within CAM_FRESH_S (the page loops snapshot.jpg only then: a loop against a dead address hangs
        a browser's connections), frames / age from that /status."""
        if self.cam is None:
            return {"ok": False, "base": None, "why": "no camera address (--cam-status)"}
        t, body, url, err = self.cam
        base = url.rsplit("/status", 1)[0].rstrip("/") + "/" if "/status" in url else url.rstrip("/") + "/"
        if body is None:
            return {"ok": False, "base": base, "why": "camera not answering at %s (%s)" % (base, err or "no answer")}
        if now - t > CAM_FRESH_S:
            return {"ok": False, "base": base, "why": "camera status %.0f s old" % (now - t)}
        return {"ok": True, "base": base, "source": body.get("source"), "frames": body.get("frames"), "age_s": body.get("age_s"),
                "clients": body.get("clients"), "why": ""}

    def dets_view(self, now):
        """v0.1.54 (item 11): the latest perception/frame_dets for the overlay, with its age (the snapshot and the boxes may be a frame
        apart: the age is shown). Never invented: no message -> 'boxes not published' (the AGX's own observers give no pixel boxes;
        cv_publish's health says `boxes: false`), stale -> nothing drawn."""
        p = self.get("perc", now)[0] or {}
        if self.dets is None:
            who = p.get("observer_detector") or p.get("detector")
            return {"state": "none", "dets": [], "why": "boxes not published" + (" (%s gives no pixel boxes)" % who if who else "")}
        t, b = self.dets
        age = now - t
        out = {"age_s": round(age, 1), "frame_w": b.get("frame_w"), "frame_h": b.get("frame_h"), "frame_id": b.get("frame_id"),
               "source": b.get("source"), "n": len(b.get("dets") or [])}
        if age > DETS_STALE_S:
            return dict(out, state="stale", dets=[], why="boxes stale %.0f s (limit %.0f)" % (age, DETS_STALE_S))
        return dict(out, state="live", dets=list(b.get("dets") or []), why="")

    def camera(self, now):
        srcs = self.perception_sources(now)
        sl = " · ".join("%s %.1f s" % (k, a) for k, _b, a in srcs)
        hint = cv_hint(srcs)
        out = {"l3": self.stream(now), "l4": (hint or {}).get("text") or "CV: no perception health", "sources": [
            {"name": k, "age_s": round(a, 1), "stale": a > PERC_STALE_S} for k, _b, a in srcs]}
        p, pa = self.get("perc", now)
        if p is None:
            return dict(out, state="unavailable", l1="OFF · no perception health", l2="sources: none (cv_publish / fake_perception)")
        fresh = [x for x in srcs if x[2] <= PERC_STALE_S]
        if not fresh:
            return dict(out, state="warning", l1="STALE · newest %.1f s (limit %.0f)" % (min(a for _k, _b, a in srcs), PERC_STALE_S), l2="sources: " + sl)
        rate = fresh[0][1].get("rate_hz")
        return dict(out, state="active", rate=rate, n_fresh=len(fresh), l1="perception up · %d source%s%s · 51° HFOV" % (len(fresh), "" if len(fresh) == 1 else "s",
                                                                                     "" if rate is None else " · %.1f Hz" % rate),
                    l2="sources: " + sl)

    def cannon_link(self, now):
        """v0.1.57 (WP12): the cannon Pi's local/<v>/cannon/status (sim: fake_cannon_pi) -> (state, text): sender health, last packet age, hmac."""
        c, ca = self.get("can", now)
        if c is None:
            return "none", "Pi status: none (no local/%s/cannon/status)" % self.v
        if ca > CAN_STALE_S:
            return "stale", "Pi status STALE %.0f s" % ca
        age = c.get("age_s")
        pk = "no packet yet" if age is None else "last packet %.1f s" % age
        h = "hmac ?" if c.get("hmac_ok") is None else ("hmac ok" if c.get("hmac_ok") else "HMAC BAD")
        tgt = "target w%s" % c.get("window_id") if c.get("target") else "target none"
        return ("ok" if c.get("ok") else "bad"), "Pi %s · %s · %s · %s · n %s%s%s" % ("ok" if c.get("ok") else "NOT OK", pk, h, tgt, c.get("n_rx"),
                                                                                      " (%s bad)" % c["n_bad"] if c.get("n_bad") else "", self._rx_text(c))

    @staticmethod
    def _rx_text(c):
        """v0.1.61: the students' receiver's own view (cv/cannon/pi/cannon_status_pub.py): its last event, the servo angles, the lock, the mode."""
        ev = c.get("event")
        if not ev:
            return ""
        t = " · rx " + ev
        if c.get("pan_deg") is not None and c.get("tilt_deg") is not None:
            t += " pan %+.1f° tilt %+.1f°" % (c["pan_deg"], c["tilt_deg"])
        if c.get("reason"):
            t += " (%s)" % c["reason"]
        if c.get("ready"):
            t += " · LOCK"
        if c.get("mode") and c.get("mode") != "DRY_RUN":
            t += " · %s" % c["mode"]
        if c.get("pump") == "ON":
            t += " · PUMP ON"
        return t

    def _can_row(self, now):
        ls, lt = self.cannon_link(now)
        return lt if ls == "ok" else (U(lt) if ls == "none" else W(lt))

    def cannon(self, now):
        a, aa = self.get("aim", now)
        ls, lt = self.cannon_link(now)
        l2 = lt + " · no fire button"
        if a is None:
            return {"state": "available", "l1": "link idle · no aim seen", "l2": l2}
        if aa > AIM_STALE_S or not a.get("on"):
            return {"state": "available", "l1": "aim off%s" % ("" if aa <= AIM_STALE_S else " (last %.0f s ago)" % aa), "l2": l2}
        if ls in ("bad", "stale"):
            return {"state": "warning", "l1": "AIM on · bay %s · w%s · Pi link %s" % (a.get("bay"), a.get("window_id"), "NOT OK" if ls == "bad" else "STALE"), "l2": l2}
        c, _ca = self.get("can", now)
        if c and str(c.get("event") or "").startswith("DROP"):               # v0.1.61: the receiver refuses what we send
            return {"state": "warning", "l1": "AIM on · bay %s · w%s · Pi REFUSES: %s" % (a.get("bay"), a.get("window_id"), c.get("reason") or c["event"]), "l2": l2}
        return {"state": "active", "l1": "AIM on · bay %s%s · %.1f s" % (a.get("bay"), "" if a.get("window_id") is None else " · w%s" % a.get("window_id"), aa), "l2": l2}

    def alarms(self, now):
        out = []
        fcu, fa = self.get("fcu", now)
        st, sa = self.get("status", now)
        hb = self.hb_age(now)
        if not self.connected:
            out.append(("alarm", "this page is not connected to the boat broker (%s)" % self.broker))
        if (fcu is not None or st is not None) and (hb is None or hb > HB_STALE_S):
            out.append(("alarm", "FCU LINK LOST: no heartbeat" + ("" if hb is None else " %.0f s" % hb) + " - the transmitter still rules the boat"))
        if fcu is not None and fa > BLOCK_STALE_S:
            out.append(("warning", "usv_ui silent: FCU block %.0f s old" % fa))
        if st is not None and sa > 2.0:
            out.append(("alarm", "mission manager silent: status %.0f s old" % sa))
        b = st or {}
        if b.get("kill") or b.get("estop") or (fcu or {}).get("kill") or (fcu or {}).get("estop"):
            out.append(("alarm", "KILL / E-stop asserted"))
        if (fcu or {}).get("rc5_estop"):
            out.append(("alarm", "RC5 (SF) motor E-stop on the transmitter"))
        if b.get("state") in ("KILLED", "FAULT"):
            out.append(("alarm", "vehicle %s%s" % (b["state"], (" " + str(b["fault"])) if b.get("fault") else "")))
        if (fcu or {}).get("rc6_override"):
            out.append(("warning", "RC6 MANUAL override high"))
        if self.bridge is not None and not self.bridge[0]:
            out.append(("alarm", "OCS LINK DOWN: the boat broker lost the hub (bridge)"))
        p, pa = self.get("perc", now)
        if p is not None and pa > PERC_STALE_S:
            out.append(("warning", "perception stale %.1f s" % pa))
        el = self.electronics(now)
        if el.get("hot") == "temp":
            out.append(("alarm" if el["state"] == "fault" else "warning", el["l1"]))
        return [{"level": lv, "text": t} for lv, t in out]

    # ---- v0.1.49: history (TELEMETRY, COG, stage errors) ----
    def _sample(self, now, f):
        servo = f.get("servo") or []
        pwm = [servo[ch - 1] if (ch and len(servo) >= ch and servo[ch - 1]) else None
               for ch in (self.thr_map.get(k) for k in ("thr-mp", "thr-ms", "thr-sf", "thr-sa"))]
        hdg = f.get("heading_deg") if f.get("heading_deg") is not None else f.get("hdg_deg")
        self.hist.append((now, f.get("groundspeed_mps"), hdg, pwm, f.get("hb_age_s")))
        if f.get("lat") is not None and f.get("lon") is not None:
            self.fixes.append((now, float(f["lat"]), float(f["lon"])))
        nv = f.get("nav") or {}
        st, _sa = self.get("status", now)
        key = ((st or {}).get("task_name"), (st or {}).get("task_state"))
        if key != self._stage:
            self._stage, self._stage_err = key, []
        if hdg is not None and nv.get("target_bearing_deg") is not None:
            self._stage_err.append((nv["target_bearing_deg"] - hdg + 180.0) % 360.0 - 180.0)
            del self._stage_err[:-1200]

    def cog(self, now):
        """COG from the fixes of the last ~4 s (None below 0.3 m of travel: the boat is not moving enough to say)."""
        fx = [x for x in self.fixes if now - x[0] <= 4.5]
        if len(fx) < 2:
            return None
        (_t0, la0, lo0), (_t1, la1, lo1) = fx[0], fx[-1]
        dn = (la1 - la0) * 111320.0
        de = (lo1 - lo0) * 111320.0 * math.cos(math.radians(la0))
        if math.hypot(dn, de) < 0.3:
            return None
        return math.degrees(math.atan2(de, dn)) % 360.0

    def telemetry(self, now):
        """TELEMETRY: 60 s of the 2 Hz block, t = seconds before now (the page's plotter draws it; pause freezes it there)."""
        h = [x for x in self.hist if now - x[0] <= 60.0]
        t = [round(x[0] - now, 2) for x in h]
        ser = {"sog": [x[1] for x in h], "hdg": [x[2] for x in h], "hb": [x[4] for x in h]}
        for i, k in enumerate(("thr-mp", "thr-ms", "thr-sf", "thr-sa")):
            ser["pwm_" + THR_NAME[k]] = [x[3][i] for x in h]
        return {"t": t, "series": ser, "window_s": 60, "n": len(t)}

    # ---- v0.1.49: the dials ----
    def radar(self, now):
        """Heading-up PPI of the mission manager's map digest: rings 5 / 10 / 25 / 40 m on a sqrt radial scale (unit = the 40 m edge)."""
        d, da = self.map_pref(now)
        out = {"rings": [{"m": r, "u": round(radar_r(r), 4)} for r in RINGS_M], "items": [], "rays": [], "dock": None, "edge": [],
               "state": "unknown", "text": []}
        if d is None or not d.get("boat"):
            out["text"] = [U("no map digest (the mission manager's telemetry/usv/map)")]
            return out
        b = d["boat"]
        bx, by, hdg = float(b["x_n"]), float(b["y_e"]), float(b.get("hdg_deg") or 0.0)
        out["state"] = "stale" if da > MAP_STALE_S else "live"
        out["hdg"] = round(hdg, 1)
        n = {"buoy": 0, "dock": 0, "other": 0}
        for e in d.get("entities") or []:
            try:
                dn, de = float(e["x_n"]) - bx, float(e["y_e"]) - by
            except (KeyError, TypeError, ValueError):
                continue
            r = math.hypot(dn, de)
            x, y = radar_xy(dn, de, hdg)
            kind = e.get("kind") or "buoy"
            n[kind if kind in n else "other"] += 1
            col = e.get("colour_now") or e.get("colour") or "unknown"
            out["items"].append({"x": x, "y": y, "kind": kind, "colour": col, "hex": OBJ.get(col, OBJ["unknown"]), "r_m": round(r, 1),
                                 "out": r > RADAR_R_M, "id": e.get("id"), "label": "%s %s %.0f m" % (col, kind, r)})
        for ray in d.get("rays") or []:
            try:
                ox, oy, az = float(ray["x_n"]) - bx, float(ray["y_e"]) - by, math.radians(float(ray["az_deg"]))
            except (KeyError, TypeError, ValueError):
                continue
            pts = []
            for k in range(0, 81):                     # the ray from where it was seen out to the 40 m band edge
                px, py = ox + 0.5 * k * math.cos(az), oy + 0.5 * k * math.sin(az)
                if math.hypot(px, py) > RADAR_R_M:
                    break
                pts.append(radar_xy(px, py, hdg))
            if len(pts) >= 2:
                out["rays"].append({"pts": pts, "hex": OBJ.get(ray.get("colour"), OBJ["unknown"]), "colour": ray.get("colour")})
        dk = d.get("dock")
        if dk and dk.get("line_hdg_deg") is not None:
            th, L = math.radians(float(dk["line_hdg_deg"])), float(dk.get("length_m") or 4.0)
            cx, cy = float(dk["x_n"]) - bx, float(dk["y_e"]) - by
            pts = [radar_xy(cx + s * math.cos(th), cy + s * math.sin(th), hdg) for s in frange(-L / 2, L / 2, 0.5)]
            out["dock"] = {"pts": pts, "r_m": round(math.hypot(cx, cy), 1)}
        f, _fa = self.get("fcu", now)
        lat, lon = (f or {}).get("lat"), (f or {}).get("lon")
        if self.course and lat is not None:
            ring = [p for p in self.course if isinstance(p, (list, tuple)) and len(p) >= 2]
            k = 111320.0
            loc = [((p[0] - lat) * k, (p[1] - lon) * k * math.cos(math.radians(lat))) for p in ring]
            for (a0, a1), (b0, b1) in zip(loc, loc[1:] + loc[:1]):
                L = math.hypot(b0 - a0, b1 - a1)
                run = []
                for s_ in frange(0.0, L, 1.0):
                    px, py = a0 + (b0 - a0) * s_ / max(L, 1e-6), a1 + (b1 - a1) * s_ / max(L, 1e-6)
                    if math.hypot(px, py) <= RADAR_R_M:
                        run.append(radar_xy(px, py, hdg))
                    elif run:
                        out["edge"].append(run)
                        run = []
                if len(run) >= 2:
                    out["edge"].append(run)
        out["text"] = ["%d buoy%s · %d dock face%s%s · %d ray%s" % (n["buoy"], "" if n["buoy"] == 1 else "s", n["dock"], "" if n["dock"] == 1 else "s",
                                                                    (" · %d other" % n["other"]) if n["other"] else "", len(out["rays"]),
                                                                    "" if len(out["rays"]) == 1 else "s"),
                       "dock line %.0f m" % out["dock"]["r_m"] if out["dock"] else U("no dock line"),
                       ("course edge in range" if out["edge"] else "course edge beyond 40 m") if self.course and lat is not None
                       else U("no course edge (no COURSE seen by this page)"),
                       ("map %.1f s" % da) if da <= MAP_STALE_S else W("map STALE %.1f s" % da)]
        return out

    def cmd_block(self, now):
        """v0.1.55 (D1 / D2): the mission manager's commanded side (local/<v>/mm/state `cmd`) -> (cmd | None, why). Stale (> 2 s)
        greys like everything else: (None, 'STALE n s'); no mm/state -> (None, 'no mm/state')."""
        mm, ma = self.get("mm", now)
        if mm is None:
            return None, "no mm/state"
        if ma > CMD_STALE_S:
            return None, "mm/state STALE %.0f s" % ma
        c = mm.get("cmd")
        return (c, "ok") if isinstance(c, dict) else (None, "no cmd block (mission manager < v0.1.55)")

    def _speed_des(self, now, gs):
        """v0.1.55 (D2): desired (the speed dial's commanded tick), actual SOG, the error - never a made-up desired value."""
        sp = self.speed(now, gs)
        des = sp.get("cmd_surge")
        return ["%.2f m/s" % des if des is not None else U("nothing commanded"), "%.2f m/s" % gs if gs is not None else U("?"),
                ("%+.2f m/s" % (gs - des)) if (des is not None and gs is not None) else U("—")]

    def _tau_text(self, now):
        c, why = self.cmd_block(now)
        f_ = (c or {}).get("frame")
        if f_:
            return "surge %+.2f · sway %+.2f · yaw %+.2f" % (f_["surge"], f_["sway"], f_["yaw"])
        return U("no mode-20 frame commanded" if why == "ok" else why)

    def set_heading(self, now, nav_cmd):
        """v0.1.55 (D2): the compass SET = the behaviour's heading goal (cmd.hdg_goal_deg), else NAV_CONTROLLER_OUTPUT's nav
        bearing while the FCU steers, else None (hatched). -> (deg | None, source)."""
        c, _why = self.cmd_block(now)
        if c and c.get("hdg_goal_deg") is not None:
            return float(c["hdg_goal_deg"]) % 360.0, "goal (%s)" % (c.get("behaviour") or "behaviour")
        if nav_cmd is not None:
            return nav_cmd, "FCU nav bearing"
        return None, None

    def compass(self, now, hdg, nv, mode=None, rot=None, cmd=None):
        """v0.1.51 (WP10k-c item 28): the rose's three vectors - HDG (solid), COG (open), the commanded yaw (dashed; `cmd` from
        cmd_yaw(), None = not published: drawn hatched, never as a 0) - set vs actual as two numbers, the ROT scale (°/s, ±ROT_MAX_DPS)
        from ATTITUDE.yawspeed (`rot` from rot_dps(), None = not published)."""
        cog = self.cog(now)
        des = (nv or {}).get("target_bearing_deg")
        cmd, src = self.set_heading(now, cmd)                          # v0.1.55 (D2)
        return {"set_src": src, "hdg": hdg, "cog": None if cog is None else round(cog, 1), "des": des, "cmd": cmd, "rot": rot, "rot_max": ROT_MAX_DPS,
                "set": "%03.0f°" % (cmd % 360.0) if cmd is not None else U("not published"),
                "act": "%03.0f°" % (hdg % 360.0) if hdg is not None else U("no heading"),
                "text": ["heading %s°" % _f(hdg, 0) if hdg is not None else U("no heading"),
                         "COG %.0f°" % cog if cog is not None else U("COG: not moving / no fix"),
                         "target brg %s° (GUIDED)" % _f(des, 0) if des is not None else U("no target bearing"),
                         ("cmd yaw %.0f° (FCU %s)" % (cmd, mode) if src == "FCU nav bearing" else "SET %.0f° · %s (FCU %s)" % (cmd, src, mode))
                         if cmd is not None
                         else U("SET: nothing commands a heading (hold / manual / handed over)"),
                         "ROT %+.1f°/s" % rot if rot is not None else U("ROT: not published (no ATTITUDE.yawspeed)")]}

    @staticmethod
    def cmd_yaw(f, fresh_f, mode):
        """The commanded yaw, only where the FCU publishes one: NAV_CONTROLLER_OUTPUT.nav_bearing while its own controller steers
        (GUIDED / AUTO / RTL / SMART_RTL / LOITER) and the message is current; mode 20 (our frame) does not publish it yet -> None."""
        nv = (f.get("nav") or {}) if fresh_f else {}
        age = (f.get("ages_s") or {}).get("NAV_CONTROLLER_OUTPUT")
        if mode not in CMD_YAW_MODES or nv.get("nav_bearing_deg") is None or (age is not None and age > BLOCK_STALE_S):
            return None
        return float(nv["nav_bearing_deg"]) % 360.0

    @staticmethod
    def rot_dps(f, fresh_f):
        """ATTITUDE.yawspeed in °/s (usv_ui's yawspeed_dps, v0.1.51); None when the block is stale, the field absent or ATTITUDE old."""
        age = (f.get("ages_s") or {}).get("ATTITUDE")
        if not fresh_f or f.get("yawspeed_dps") is None or (age is not None and age > BLOCK_STALE_S):
            return None
        return round(float(f["yawspeed_dps"]), 1)

    def speed(self, now, gs):
        """v0.1.55 (D2): the commanded tick = cmd.target.speed_mps in GUIDED, or frame.surge × max_speed_mps (*τ surge*) in mode 20,
        None (hatched) otherwise; the sway of the frame as text only (the dial has no sway axis)."""
        c, why = self.cmd_block(now)
        tick, label, sway = None, None, None
        if c:
            t_, f_ = c.get("target"), c.get("frame")
            if t_ and t_.get("speed_mps") is not None:
                tick, label = float(t_["speed_mps"]), "cmd %.2f m/s (GUIDED target)" % float(t_["speed_mps"])
            elif f_ and f_.get("surge") is not None:
                tick = float(f_["surge"]) * float(c.get("max_speed_mps") or 1.0)
                label = "τ surge %+.2f × %.1f m/s = %+.2f m/s (mode 20)" % (float(f_["surge"]), float(c.get("max_speed_mps") or 1.0), tick)
                sway = f_.get("sway")
        if tick is None:
            label = U("cmd surge: nothing commanded" if why == "ok" else "cmd surge: " + why)
        return {"sog": gs, "max": 2.0, "cmd_surge": None if tick is None else round(max(0.0, tick), 2), "cmd_sway": sway,
                "cmd_label": label if tick is not None else None,
                "text": ["SOG %.2f m/s" % gs if gs is not None else U("no SOG (VFR_HUD)"), label]
                + (["τ sway %+.2f" % sway] if sway is not None else [])}

    # ---- v0.1.49: the wireframe's callouts ----
    def callouts(self, wf):
        c = wf["cam"]
        items = []
        for eid in ("thr-mp", "thr-ms", "thr-sf", "thr-sa"):
            o = wf[eid]
            items.append((eid, "%s · ch %s" % (o["name"], o["ch"] if o["ch"] is not None else "—"), [o["label"]], o["state"]))
        items += [("cannon", "WATER CANNON", [wf["cannon"]["l1"]], wf["cannon"]["state"]),
                  ("cam", "CAMERA · CV", [short_word("cam", c)], c["state"]),
                  ("lamp", "LAMP", [short_word("lamp", wf["lamp"])], wf["lamp"]["state"]),
                  ("ant", "LINK · QoS", [short_word("ant", wf["ant"])], wf["ant"]["state"]),
                  ("box", "ELECTRONICS", [short_word("box", wf["box"])], wf["box"]["state"])]
        out = place_callouts(items)
        # v0.1.52: the long texts moved to the tooltip and the click panel (detail), with the tab that holds the rest
        det = {k: [wf[k]["label"]] + (["output channel %s (usv_ui MOTOR_NAMES; --thr-map overrides)" % wf[k]["ch"]] if wf[k]["ch"] else ["not mapped"])
               + ["applied PWM = SERVO_OUTPUT_RAW · no RPM / current feedback (T200 / Basic ESC)"] for k in THR_NAME}
        det.update(cannon=[wf["cannon"]["l1"], wf["cannon"]["l2"]], cam=[c["l1"], c["l2"], c["l3"], c["l4"]],
                   lamp=[wf["lamp"]["l1"], wf["lamp"]["l2"]], ant=[wf["ant"]["l1"], wf["ant"]["l2"]],
                   box=[wf["box"]["l1"], wf["box"]["l2"], wf["box"]["l3"], "FCU " + str(wf["box"].get("fcu"))])
        names = {"thr-mp": "THRUSTER M-P (main, port)", "thr-ms": "THRUSTER M-S (main, starboard)", "thr-sf": "THRUSTER SW-F (sway, forward)",
                 "thr-sa": "THRUSTER SW-A (sway, aft)", "cannon": "WATER CANNON", "cam": "CAMERA (ZED 2i, mast) · CV",
                 "lamp": "LAMP (RGY, expected only)", "ant": "LINK · QoS (antenna, bridge)", "box": "ELECTRONICS (NX, AGX, FCU)"}
        for b in out:
            b["detail"] = [str(x) for x in det.get(b["id"], []) if x not in (None, "")]
            b["name"] = names.get(b["id"], b["id"])
            b["tab"] = WF_TAB.get(b["id"])
        return out

    # ---- v0.1.49: the tabs ----
    def tabs(self, now, f, fa, s, nav_rows, ctl_rows, hdg, gs, nv, wf):
        fresh_f = bool(f) and fa is not None and fa <= BLOCK_STALE_S
        servo = (f.get("servo") or []) if fresh_f else []
        norm = {}
        for k in ("thr-mp", "thr-ms", "thr-sf", "thr-sa"):
            o = wf[k]
            norm[k] = None if o["pwm"] is None else (o["pwm"] - 1500) / 400.0
        alloc = []
        for k in ("thr-mp", "thr-ms", "thr-sf", "thr-sa"):
            o = wf[k]
            alloc.append([o["name"] + " · ch %s" % (o["ch"] if o["ch"] is not None else "—"),
                          ["%+.2f" % norm[k] if norm[k] is not None else U(o["state"]), o["label"]]])
        if all(norm[k] is not None for k in norm):
            u, r = (norm["thr-mp"] + norm["thr-ms"]) / 2.0, (norm["thr-mp"] - norm["thr-ms"]) / 2.0
            alloc.append(["implied u · v · r (from the FCU output)", "%+.2f · %+.2f · %+.2f" % (u, (norm["thr-sf"] + norm["thr-sa"]) / 2.0, r)])
        else:
            alloc.append(["implied u · v · r (from the FCU output)", U("needs all four outputs")])
        err = self._stage_err
        stats = (["%.1f°" % (sum(abs(e) for e in err) / len(err)), "%.1f°" % math.sqrt(sum(e * e for e in err) / len(err)),
                  "%.1f°" % max(abs(e) for e in err)] if err else [U("no heading error samples in this stage")])
        stage = "%s / %s" % self._stage if self._stage and self._stage[0] else "no task"
        mode = f.get("mode") if fresh_f else s.get("mode")
        cmdb, cwhy = self.cmd_block(now)                                   # v0.1.55 (D2): commanded and applied, two numbers
        cfr = (cmdb or {}).get("frame")
        ctg = (cmdb or {}).get("target")
        if cfr:
            cmd_text = "τ surge %+.2f · sway %+.2f · yaw %+.2f (%s, %s)" % (cfr["surge"], cfr["sway"], cfr["yaw"], cmdb.get("behaviour") or "?",
                                                                        cmdb.get("stage") or "?")
        elif ctg:
            cmd_text = "GUIDED target %.6f, %.6f · brg %s° · %s m/s%s" % (ctg["lat"], ctg["lon"], _f(ctg.get("hdg_deg"), 0),
                                                                          _f(ctg.get("speed_mps"), 2),
                                                                          (" · carrot %.1f / %.1f m" % (ctg["carrot"]["x_n"], ctg["carrot"]["y_e"]))
                                                                          if ctg.get("carrot") else "")
        else:
            cmd_text = U("nothing commanded (hold / manual / handed over)" if cwhy == "ok" else cwhy)
        share = None
        if cfr:                                                            # the frame's per-thruster share, before the stern factor / clip
            u_, v_, r_ = float(cfr["surge"]), float(cfr["sway"]), float(cfr["yaw"])
            share = {"thr-mp": u_ + r_, "thr-ms": u_ - r_, "thr-sa": v_, "thr-sf": v_}
        control = {
            "chain": [["1 reference", ctl_rows[1][1]], ["2 controller", ctl_rows[0][1]],
                      ["3 commanded (mm/state cmd)", cmd_text],
                      ["4 allocation", "the FCU's (mode 20: our frame → its four outputs)" if mode == "CUSTOM_MANUAL" else
                       ("ArduRover's own (GUIDED / HOLD)" if mode else U("unknown"))],
                      ["5 response", ctl_rows[2][1]]],
            "alloc": alloc,
            "response": [ctl_rows[2], ctl_rows[3], ctl_rows[4], ["heading err MAE · RMSE · max (%s, %d samples)" % (stage, len(err)), stats],
                         ctl_rows[7]],
        }
        mm, _ma = self.get("mm", now)
        tv = (mm or {}).get("tuning")
        if tv is None:
            tv = TUN.view({})
            for r_ in tv["knobs"]:
                r_["value"] = None
            tv["live"] = False
        else:
            tv = dict(tv, live=True)
        control["tuning"] = tv
        control["tuning_state"] = s.get("state") if s else None
        control["tuning_ok"] = (s.get("state") in TUN.TUNE_STATES) if s else False
        act = []
        for k in ("thr-mp", "thr-ms", "thr-sf", "thr-sa"):
            o = wf[k]
            role = {"thr-mp": "main, port hull aft", "thr-ms": "main, starboard hull aft", "thr-sf": "sway, forward cross-beam",
                    "thr-sa": "sway, aft cross-beam"}[k]
            act.append({"id": k, "name": o["name"], "role": role, "ch": o["ch"], "state": o["state"], "pwm": o["pwm"], "frac": o["frac"], "dir": o["dir"],
                        "label": o["label"], "rows": [["output channel", ("ch %d" % o["ch"]) if o["ch"] else U("not mapped")],
                                                      ["applied PWM (SERVO_OUTPUT_RAW)", o["pwm"] if o["pwm"] is not None else U(o["state"])],
                                                      ["share of range", "%+.0f %%" % (100 * o["frac"] * o["dir"]) if o["pwm"] is not None else U("—")],
                                                      ["saturation (> 90 %)", W("SATURATED") if o["state"] == "warning" else ("no" if o["pwm"] is not None else U("—"))],
                                                      ["commanded (frame share)", ("%+.0f %% (τ, before stern factor / clip)" % (100 * share[k]))
                                                       if share else U("no mode-20 frame commanded")],
                                                      ["feedback (RPM, current)", U("none by design (T200 / Basic ESC)")]]})
        chmap = [["%s ch %s" % (THR_NAME[k], self.thr_map.get(k, "—")), {"thr-mp": "M1 PORT", "thr-sa": "M2 STERN", "thr-ms": "M3 STBD",
                                                                      "thr-sf": "M4 BOW"}[k] + " (usv_ui MOTOR_NAMES)"]
                 for k in ("thr-mp", "thr-sa", "thr-ms", "thr-sf")]
        gps = f.get("gps") if fresh_f else None
        flags = f.get("ekf_flags") if fresh_f else None
        rates = (f.get("rates_hz") or {}) if fresh_f else {}
        ages = (f.get("ages_s") or {}) if fresh_f else {}

        def rate_row(k):
            r_, a_ = rates.get(k), ages.get(k)
            return [k, ["%.1f Hz" % r_ if r_ is not None else U("no rate"), ("%.1f s" % a_) if a_ is not None else U("never")]]
        cog = self.cog(now)
        navt = {
            "gnss": [nav_rows[0], ["fix · sats · hdop", nav_rows[5][1][1]],
                     ["position age", ("%.1f s" % ages["GLOBAL_POSITION_INT"]) if ages.get("GLOBAL_POSITION_INT") is not None else U("no GLOBAL_POSITION_INT")]],
            "ekf": [["flags", ("0x%04x" % flags) if flags is not None else U("no EKF_STATUS_REPORT")],
                    ["set", (", ".join(n_ for b_, n_ in EKF_BITS if flags & b_) or "none") if flags is not None else U("—")],
                    ["position ok", nav_rows[5][1][0]]],
            "att": [nav_rows[3], ["heading sources", ["VFR_HUD %s°" % _f(f.get("heading_deg"), 0) if fresh_f and f.get("heading_deg") is not None else U("no VFR_HUD"),
                                                      "GLOBAL_POSITION %s°" % _f(f.get("hdg_deg"), 0) if fresh_f and f.get("hdg_deg") is not None else U("no hdg"),
                                                      "COG %.0f°" % cog if cog is not None else U("COG: not moving")]],
                    nav_rows[2], nav_rows[4]],
            "rates": [rate_row(k) for k in ("GLOBAL_POSITION_INT", "GPS_RAW_INT", "ATTITUDE", "EKF_STATUS_REPORT", "VFR_HUD", "NAV_CONTROLLER_OUTPUT")],
        }
        topics = [[k, "%.1f s" % (now - t)] for k, (t, _b) in sorted(self.last.items())]
        topics += [["presence/" + n_, ("up" if b_.get("up") else "DOWN") + " · %.1f s" % (now - t)] for n_, (t, b_) in sorted(self.presence.items())]
        hosts = []
        for h_ in ("nx", "agx"):
            r_ = self.hosts.get(h_)
            if r_ is None:
                hosts.append([h_.upper(), U("no host beat")])
                continue
            b_ = r_[1]
            temps = [x for x in (b_.get("temp_c") or {}).values() if isinstance(x, (int, float))]
            hosts.append([h_.upper() + (" · this Mac (simulated host)" if b_.get("simulated") else ""), ["%.1f s" % (now - r_[0]), "cpu %s %%" % _f(b_.get("cpu_pct"), 0) if b_.get("cpu_pct") is not None else U("cpu ?"),
                                       "%.0f °C" % max(temps) if temps else U("n/a °C"), W("THROTTLING") if b_.get("throttling") else "no throttling",
                                       "%.0f GB free" % b_["disk_free_gb"] if b_.get("disk_free_gb") is not None else U("disk ?")]])
        diag = {"mavlink": [rate_row(k) for k in ("HEARTBEAT", "SYS_STATUS", "RC_CHANNELS", "SERVO_OUTPUT_RAW", "NAMED_VALUE_INT")],
                "mqtt": [["boat broker", ("connected " + self.broker) if self.connected else A("NOT connected " + self.broker)],
                         ["bridge", U("unknown") if self.bridge is None else ("up" if self.bridge[0] else A("DOWN"))]] + topics,
                "cmds": [{"t": c_.get("t"), "word": c_.get("word"), "ok": c_.get("ok"), "text": c_.get("text")} for c_ in list(self.cmds)],
                "tail": (f.get("tail") or []) if f else [], "tail_age": None if not f else round(fa, 1), "hosts": hosts}
        return {"control": control, "actuation": {"thr": act, "map": chmap}, "nav": navt, "diag": diag}

    # ---- v0.1.52: the top plate, the SIM flag, the light map, the console ----
    def map_pref(self, now):
        """The map digest to draw: the mission manager's while it is current, else the newest (usv_ui's carries its own) - so the
        radar and the light map never flip between the two publishers of telemetry/<v>/map."""
        mm = self.maps.get("mm")
        if mm is not None and now - mm[0] <= MAP_STALE_S:
            return mm[1], now - mm[0]
        return self.get("map", now)

    def _ui_presence(self, now):
        r = self.presence.get("usv_ui")
        return r[1] if r and r[1].get("up") and now - r[0] <= PRES_STALE_S else None

    def plate(self, now, V, mode, armed, kill, estop, holder, hb, s):
        """The top plate: VEHICLE / FCU MODE / ARM as big colour-coded blocks + the toggle boxes (holder, log, cv). cls: auto (green)
        manual (yellow) alarm (red) armed (red) disarmed (grey) neutral (outline) on / off (toggles) unk (hatched: not published)."""
        hb_ok = hb is not None and hb <= HB_STALE_S
        hbs = "no FCU heartbeat" if hb is None else ("HB %.1f s" % hb if hb_ok else "NO HB %.0f s · last value" % hb)
        task = (s.get("task_name") or s.get("current_task")) if s else None
        veh = {"label": "VEHICLE", "text": V or "unknown", "cls": vehicle_class(V),
               "sub": ("task %s · %s" % (task, s.get("task_state") or "-")) if task and task != "TASK_NONE" else
                      ((playbook.vehicle_words(V, mode, bool((s.get("rc6") or {}).get("holds")), holder == "usv_ui", s.get("station_keep"))
                        or "no task") if V else "no status/usv (mission manager)"), "stale": False}    # v0.1.81 (WP37): the words
        md = {"label": "FCU MODE", "text": ("MODE 20" if mode == "CUSTOM_MANUAL" else mode) if mode else "unknown",
              "cls": fcu_mode_class(mode, holder), "sub": ("CUSTOM_MANUAL · " if mode == "CUSTOM_MANUAL" else "") + hbs,
              "stale": mode is not None and not hb_ok}
        ks = "kill ?" if kill is None else ("KILL" if kill else "no kill")
        es = "E-stop ?" if estop is None else ("E-STOP" if estop else "no E-stop")
        arm = {"label": "ARM", "text": "unknown" if armed is None else ("ARMED" if armed else "DISARMED"),
               "cls": "unk" if armed is None else ("armed" if armed else "disarmed"), "sub": "%s · %s" % (ks, es),
               "stale": armed is not None and not hb_ok, "alarm": bool(kill or estop)}
        f, fa = self.get("fcu", now)
        fresh = f is not None and fa <= BLOCK_STALE_S
        pres = self._ui_presence(now)
        tg = [{"id": "holder", "label": "FCU HOLDER", "text": holder, "cls": "unk" if holder == "unknown" else "on",
               "sub": {"usv_ui": "after takeover", "mission manager": "after handover"}.get(holder, "no usv_ui block / beat")}]
        log, src = None, None
        if fresh and "log" in f:
            log, src = f.get("log") or False, "usv_ui FCU block"
        elif pres is not None and "log" in pres:
            log, src = pres.get("log") or False, "usv_ui presence"
        tg.append({"id": "log", "label": "LOG", "text": "unknown" if log is None else ("ON" if log else "off"),
                   "cls": "unk" if log is None else ("on" if log else "off"), "src": src,
                   "sub": "not published" if log is None else (str(log) if log else "usv_ui not logging")})
        cv, src = None, None
        um = self.maps.get("ui")
        if fresh and "cv_on" in f:
            cv, src = bool(f["cv_on"]), "usv_ui FCU block"
        elif pres is not None and "cv_on" in pres:
            cv, src = bool(pres["cv_on"]), "usv_ui presence"
        elif um is not None and now - um[0] <= 3.0 and isinstance(um[1].get("cv"), dict) and "on" in um[1]["cv"]:
            cv, src = bool(um[1]["cv"]["on"]), "usv_ui map"
        tg.append({"id": "cv", "label": "CV FOLD", "text": "unknown" if cv is None else ("ON" if cv else "off"),
                   "cls": "unk" if cv is None else ("on" if cv else "off"), "src": src,
                   "sub": "not published" if cv is None else ("→ usv_ui map" if cv else "not folded")})
        return {"vehicle": veh, "mode": md, "arm": arm, "toggles": tg}

    def sim_flag(self, now):
        """SIMULATION when usv_ui holds the fake FCU (its block, its presence beat or the mission manager's status) or the camera
        stream is the simulator's; the reason is named. Unknown link = no banner (never a guess)."""
        f, fa = self.get("fcu", now)
        st, sa = self.get("status", now)
        pres = self._ui_presence(now)
        link, src = None, None
        if f is not None and fa <= BLOCK_STALE_S and f.get("fcu_link"):
            link, src = f["fcu_link"], "usv_ui FCU block"
        elif pres is not None and pres.get("fcu_link"):
            link, src = pres["fcu_link"], "usv_ui presence"
        elif st is not None and sa <= 2.0 and st.get("fcu_link"):
            link, src = st["fcu_link"], "status/usv"
        why = []
        if link == "fake":
            why.append("FCU link fake: usv_ui holds the kinematic plant, no Pixhawk")
        if self.cam is not None and self.cam[1] and self.cam[1].get("source") == "sim" and now - self.cam[0] <= 5.0:
            why.append("camera = the simulator's stream")
        if st is not None and sa <= 5.0 and st.get("nav") == "ghost":       # v0.1.68 (WP17 F6): free run - where SIMULATION goes
            fr = st.get("free_run") or {}
            fw = ["ghost position, mode requests only to the FCU - NEVER at the pool"] + ([fr["banner"]] if fr.get("banner") else [])
            return {"on": True, "free_run": True, "link": link, "src": "status/usv nav ghost", "why": fw,
                    "text": "FREE RUN · " + " · ".join(fw)}
        return {"on": bool(why), "link": link, "src": src, "why": why,
                "text": ("SIMULATION · " + " · ".join(why)) if why else ""}

    def light_map(self, now):
        """The OVERVIEW light map: north-up plan of the map digests in one frame (the mission manager's CO; usv_ui's map shifted by
        the CO difference): boat + heading, its track, buoys / docks by colour, CO (home), usv_ui's saved points, the course edge.
        Coordinates are (north, east) metres; `bounds` = the auto-fit box (a 20 m minimum span)."""
        out = {"state": "unknown", "boat": None, "trail": [], "items": [], "points": [], "home": None, "edge": [], "bounds": None,
               "text": [U("no map digest (telemetry/usv/map)")]}
        if not self.maps:
            return out
        ref_src = "mm" if "mm" in self.maps and isinstance(self.maps["mm"][1].get("co"), dict) else next(iter(sorted(self.maps)))
        ref = self.maps[ref_src][1].get("co") or {}
        try:
            rlat, rlon = float(ref["lat"]), float(ref["lon"])
        except (KeyError, TypeError, ValueError):
            rlat = rlon = None

        def shift(body):
            co = body.get("co") or {}
            try:
                return ((float(co["lat"]) - rlat) * 111320.0, (float(co["lon"]) - rlon) * 111320.0 * math.cos(math.radians(rlat)))
            except (KeyError, TypeError, ValueError):
                return (0.0, 0.0)

        def pt(x, y, sh):
            return [round(float(x) + sh[0], 2), round(float(y) + sh[1], 2)]
        mm = self.maps.get("mm")
        newest = mm if mm is not None and now - mm[0] <= MAP_STALE_S else max(self.maps.values(), key=lambda r: r[0])   # no flip-flop
        withboat = [r for r in ([newest] + list(self.maps.values())) if r[1].get("boat")]
        age = now - newest[0]
        out["state"] = "stale" if age > MAP_STALE_S else "live"
        if withboat:
            t_b, b = withboat[0]
            sh = shift(b)
            bb = b["boat"]
            try:
                out["boat"] = {"n": pt(bb["x_n"], 0, sh)[0], "e": pt(0, bb["y_e"], sh)[1],
                               "hdg": None if bb.get("hdg_deg") is None else round(float(bb["hdg_deg"]), 1), "age_s": round(now - t_b, 1)}
                out["trail"] = [pt(p[0], p[1], sh) for p in (bb.get("trail") or []) if isinstance(p, (list, tuple)) and len(p) >= 2]
            except (KeyError, TypeError, ValueError):
                out["boat"] = None
        body = newest[1]
        sh = shift(body)
        for e in body.get("entities") or []:
            try:
                n_, e_ = pt(e["x_n"], e["y_e"], sh)
            except (KeyError, TypeError, ValueError):
                continue
            col = e.get("colour_now") or e.get("colour") or "unknown"
            kind = e.get("kind") or "buoy"
            out["items"].append({"n": n_, "e": e_, "kind": kind, "colour": col, "hex": OBJ.get(col, OBJ["unknown"]),
                                 "label": "%s %s #%s" % (col, kind, e.get("id"))})
        um = self.maps.get("ui")
        if um is not None and isinstance(um[1].get("points"), dict):
            shu = shift(um[1])
            for name, p in sorted(um[1]["points"].items()):
                if isinstance(p, (list, tuple)) and len(p) >= 2:
                    out["points"].append({"name": str(name), "n": pt(p[0], p[1], shu)[0], "e": pt(p[0], p[1], shu)[1]})
        if rlat is not None:
            out["home"] = {"n": 0.0, "e": 0.0, "label": "CO (home)"}
        cbl = [p for p in (body.get("course_boundary_local") or []) if isinstance(p, (list, tuple)) and len(p) >= 2]
        if cbl:
            out["edge"] = [pt(p[0], p[1], sh) for p in cbl]
        elif self.course and rlat is not None:
            out["edge"] = [[round((p[0] - rlat) * 111320.0, 2), round((p[1] - rlon) * 111320.0 * math.cos(math.radians(rlat)), 2)]
                           for p in self.course if isinstance(p, (list, tuple)) and len(p) >= 2]
        allp = ([[out["boat"]["n"], out["boat"]["e"]]] if out["boat"] else []) + out["trail"] + [[i["n"], i["e"]] for i in out["items"]] + \
            [[p["n"], p["e"]] for p in out["points"]] + ([[0.0, 0.0]] if out["home"] else []) + out["edge"]
        out["bounds"] = fit_bounds(allp)
        nb = sum(1 for i in out["items"] if i["kind"] == "buoy")
        out["text"] = ["%d buoy%s · %d other · %d point%s" % (nb, "" if nb == 1 else "s", len(out["items"]) - nb, len(out["points"]),
                                                              "" if len(out["points"]) == 1 else "s"),
                       ("map %.1f s" % age) if age <= MAP_STALE_S else W("map STALE %.1f s" % age)]
        if not out["edge"]:
            out["text"].append(U("no course edge (no COURSE seen)"))
        return out

    def console(self, now):
        """OVERVIEW's console: usv_ui's tail (its FCU block, newest first, with levels) and this page's last replies."""
        f, fa = self.get("fcu", now)
        rows = []
        if f is not None:
            for x in reversed(f.get("tail") or []):
                if isinstance(x, (list, tuple)) and len(x) >= 2:
                    rows.append({"age_s": None if x[0] is None else round(float(x[0]) + fa, 1), "text": str(x[1]),
                                 "kind": str(x[2]) if len(x) > 2 else "event", "level": ev_level(str(x[1]))})
        return {"state": "none" if f is None else ("stale" if fa > BLOCK_STALE_S else "live"), "age_s": None if f is None else round(fa, 1),
                "rows": rows, "replies": [{"t": c.get("t"), "word": c.get("word"), "ok": bool(c.get("ok")), "text": c.get("text")}
                                          for c in list(self.cmds)[:4]]}

    def snapshot(self, now: Optional[float] = None) -> dict:
        now = time.monotonic() if now is None else now
        fcu, fa = self.get("fcu", now)
        st, sa = self.get("status", now)
        ocs, oa = self.get("ocs", now)
        p, pa = self.get("perc", now)
        f = fcu or {}
        s = st or {}
        hb = self.hb_age(now)
        al = self.alarms(now)
        n_al = sum(1 for a in al if a["level"] == "alarm")
        wf = {k: self.thruster(k, now) for k in THR_NAME}
        wf.update(lamp=self.lamp(now), ant=self.antenna(now), box=self.electronics(now), cam=self.camera(now), cannon=self.cannon(now))
        fresh_f = fcu is not None and fa <= BLOCK_STALE_S
        fresh_s = st is not None and sa <= 2.0
        # who holds the FCU (displayed, never requested - OCS_V2_SPEC §7)
        pres_ui = self.presence.get("usv_ui")
        if fresh_f:
            holder = "usv_ui" if f.get("active") else "mission manager"
        elif pres_ui and pres_ui[1].get("up") and now - pres_ui[0] < 5.0 and "active" in pres_ui[1]:
            holder = "usv_ui" if pres_ui[1]["active"] else "mission manager"
        else:
            holder = "unknown"
        kill = s.get("kill") if fresh_s else f.get("kill") if fresh_f else None
        estop = s.get("estop") if fresh_s else f.get("estop") if fresh_f else None
        armed = f.get("armed") if fresh_f else s.get("armed") if fresh_s else None
        mode = f.get("mode") if fresh_f else s.get("mode") if fresh_s else None

        def chip(text, cls="", title=""):
            return {"text": text, "cls": cls, "title": title}
        V = s.get("state") if fresh_s else None
        hdr = [
            chip("USV1 · sysid %s" % ((f.get("own") or [None])[0] or 200), "", "MAVLink: usv_ui 200/190 · the mission manager 200/191"),
            chip("FCU link %s · %s · HB %s" % (f.get("fcu_link") or s.get("fcu_link") or "?", f.get("fcu_conn") or "?", "?" if hb is None else "%.1f s" % hb),
                 "unk" if hb is None else ("alarm" if hb > HB_STALE_S else ""), "fake = the kinematic plant, real = the Pixhawk; HB = the FCU heartbeat age"),
            chip("%s · held by %s" % (V or "V ?", holder), "alarm" if V in ("KILLED", "FAULT") else ("unk" if V is None else "cur"),
                 "the vehicle state and who commands the FCU (the transmitter always beats both)"),
            chip("mode %s · %s · %s · %s" % (mode or "?", "armed ?" if armed is None else ("armed" if armed else "disarmed"),
                                             "kill ?" if kill is None else ("KILL" if kill else "kill clear"),
                                             "estop ?" if estop is None else ("ESTOP" if estop else "estop clear")),
                 "alarm" if (kill or estop) else ("unk" if mode is None or hb is None or hb > HB_STALE_S else ""),
                 "FCU mode · armed · kill · E-stop (greyed: the FCU heartbeat is older than 2 s - the last values, not current)"),
            chip("task %s · stage %s" % (s.get("task_name") or s.get("current_task") or "none", s.get("task_state") or "-") if fresh_s else "task ?",
                 "" if fresh_s else "unk", "the task and its stage (status/usv)"),
            chip("OCS link %s · bridge %s" % ("?" if ocs is None else ("up" if oa <= OCS_STALE_S else "SILENT %.0f s" % oa),
                                              "?" if self.bridge is None else ("up" if self.bridge[0] else "DOWN")),
                 "alarm" if (self.bridge is not None and not self.bridge[0]) else ("warn" if ocs is not None and oa > OCS_STALE_S else ("unk" if ocs is None and self.bridge is None else "")),
                 "OCS link = ocs/state bridged in · bridge = $SYS/broker/connection/<name>/state"),
            chip("AGX perception %s" % ("?" if p is None else "%.1f s" % pa), "unk" if p is None else ("warn" if pa > PERC_STALE_S else ""),
                 "the age of local/usv/perception/health"),
            chip("%d alarm%s · %d warning%s" % (n_al, "" if n_al == 1 else "s", len(al) - n_al, "" if len(al) - n_al == 1 else "s"),
                 "alarm" if n_al else ("warn" if al else ""), "the alarms this page derives from the boat's streams (listed under Health)"),
        ]
        # OVERVIEW panels
        lat, lon = (f.get("lat"), f.get("lon")) if fresh_f and f.get("lat") is not None else (s.get("lat"), s.get("lon"))
        hdg = f.get("heading_deg") if fresh_f and f.get("heading_deg") is not None else (f.get("hdg_deg") if fresh_f else s.get("hdg_deg"))
        gs = f.get("groundspeed_mps") if fresh_f and f.get("groundspeed_mps") is not None else s.get("sog_mps")
        gps = f.get("gps") if fresh_f else None
        nav = [["position", "%.7f N %.7f E" % (lat, lon) if lat is not None else U("no position")],
               ["heading · COG", ["%s°" % _f(hdg, 0) if hdg is not None else U("no heading"),
                                  "COG %.0f°" % self.cog(now) if self.cog(now) is not None else U("COG: not moving / no fix")]],
               ["SOG", "%.2f m/s" % gs if gs is not None else U("no VFR_HUD")],
               ["roll · pitch", "%s° · %s°" % (_f(f.get("roll_deg")), _f(f.get("pitch_deg"))) if fresh_f and f.get("roll_deg") is not None else U("no ATTITUDE")],
               ["surge u · sway v · yaw rate r", [U("u not published"), U("v not published"),
                                                  "r %+.1f°/s" % self.rot_dps(f, fresh_f) if self.rot_dps(f, fresh_f) is not None else U("r not published")]],
               ["EKF · GPS", ["ok" if f.get("ekf_pos_ok") else (A("NO POSITION") if f.get("ekf_pos_ok") is False else U("no EKF")),
                              ("%s · %s sats · hdop %s" % ({0: "no GPS", 1: "no fix", 2: "2D", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}.get(gps["fix"], gps["fix"]),
                                                          gps["sats"], _f(gps["hdop"]))) if gps else U("no GPS_RAW_INT")]],
               ["source · age", "usv_ui FCU block · %.1f s" % fa if fcu is not None else U("no FCU block (usv_ui not on this broker)")]]
        nv = f.get("nav") if fresh_f else None
        err = None
        if nv and hdg is not None and nv.get("target_bearing_deg") is not None:
            err = (nv["target_bearing_deg"] - hdg + 180.0) % 360.0 - 180.0
        ctl = [["controller", {"GUIDED": "ArduRover GUIDED (the FCU's own)", "CUSTOM_MANUAL": "mode 20: our frame → the FCU's allocation",
                               "HOLD": "HOLD (the FCU)", "MANUAL": "MANUAL: the transmitter"}.get(mode, mode) if mode else U("unknown")],
               ["reference", ("%s / %s" % (s.get("task_name") or s.get("current_task"), s.get("task_state") or "-")) if fresh_s and (s.get("task_name") or s.get("current_task")) else "none"],
               ["heading des · act · err", ["%s°" % _f(nv.get("target_bearing_deg"), 0) if nv else U("no NAV_CONTROLLER_OUTPUT"),
                                            "%s°" % _f(hdg, 0) if hdg is not None else U("?"), "%+.0f°" % err if err is not None else U("—")]],
               ["speed des · act · err", self._speed_des(now, gs)],
               ["xtrack · wp dist", "%s m · %s m" % (_f(nv.get("xtrack_m")), nv.get("wp_dist_m")) if nv else U("no NAV_CONTROLLER_OUTPUT")],
               ["τ frame (surge, sway, yaw)", self._tau_text(now)],
               ["frame cadence", U("not published")],
               ["realized force", U("not measured")]]
        act = [dict(wf[k], id=k) for k in ("thr-mp", "thr-ms", "thr-sf", "thr-sa")]
        lamp, cam, can = wf["lamp"], wf["cam"], wf["cannon"]
        aux = [["mast light", [lamp["l1"] if lamp["state"] in ("expected", "read") else U(lamp["l1"]),
                               lamp["l2"] if lamp["state"] == "read" else U("no feedback")]],
               ["camera", cam["l1"] if cam["state"] == "active" else (W(cam["l1"]) if cam["state"] == "warning" else U(cam["l1"]))],
               ["recorder", U("not published")],
               ["water cannon", can["l1"]],
               ["cannon status back", self._can_row(now)]]
        h = s.get("health") or {}
        rc = f.get("rc") if fresh_f else None
        health = [["FCU link", U("unknown") if hb is None else (A("NO HB %.0f s" % hb) if hb > HB_STALE_S else "%.1f s" % hb)],
                  ["RC", ("%d ch%s%s" % (len(rc), " · RC5 E-STOP" if f.get("rc5_estop") else "", " · RC6 high" if f.get("rc6_override") else "")) if rc
                   else (("ok" if h.get("rc_ok") else A("RC LOST")) if fresh_s and "rc_ok" in h else U("no RC_CHANNELS"))],
                  ["EKF", "ok" if f.get("ekf_pos_ok") else (A("no position") if f.get("ekf_pos_ok") is False else U("unknown"))],
                  ["perception", U("unknown") if p is None else (W("%.1f s" % pa) if pa > PERC_STALE_S else "%.1f s" % pa)],
                  ["bridge", U("unknown") if self.bridge is None else ("up" if self.bridge[0] else A("DOWN"))],
                  ["clock", ("ok" if h.get("clock_ok") else W("skew")) if fresh_s and "clock_ok" in h else U("unknown")],
                  ["loop", (W("slow") if h.get("loop_slow") else "ok") if fresh_s and "loop_slow" in h else U("unknown")],
                  ["power", ["%.1f V" % f["voltage_v"] if fresh_f and f.get("voltage_v") is not None else U("V not reported"),
                             "%.1f A" % f["current_a"] if fresh_f and f.get("current_a") is not None else U("I not reported")]],
                  ["main props", " · ".join(wf[k]["state"] for k in ("thr-mp", "thr-ms"))],
                  ["sway props", " · ".join(wf[k]["state"] for k in ("thr-sf", "thr-sa"))]]
        srcs = self.perception_sources(now)
        return {"t_unix": time.time(), "connected": self.connected, "broker": self.broker, "vehicle": self.v,
                "hdr": hdr, "holder": holder, "wf": wf, "alarms": al, "thr_map": self.thr_map,
                "panels": {"nav": nav, "ctl": ctl, "aux": aux, "health": health, "act": act},
                "dials": {"radar": self.radar(now), "compass": self.compass(now, hdg, nv, mode, self.rot_dps(f, fresh_f), self.cmd_yaw(f, fresh_f, mode)), "speed": self.speed(now, gs)},
                "callouts": self.callouts(wf), "wf_box": {"w": WF_W, "h": WF_H, "hull": HULL},
                "tabs": self.tabs(now, f, fa, s, nav, ctl, hdg, gs, nv, wf), "tele": self.telemetry(now),
                "fcu_ctl": {"mode": f.get("mode") if fcu is not None else None, "armed": f.get("armed") if fcu is not None else None,
                            "rc6": s.get("rc6") if st is not None else None, "modes": [list(m) for m in FCU_MODES],   # v0.1.50
                            "fence": s.get("fence") if st is not None else None,                                     # v0.1.53
                            "guard": s.get("guard") if st is not None else None,                                     # v0.1.80 (WP36)
                            "station_keep": s.get("station_keep") if st is not None else None},                      # v0.1.81 (WP37)
                "cv_hint": cv_hint(srcs), "buttons_v49": [{"id": b[0], "word": b[1], "args": b[2], "title": b[3]} for b in BUTTONS],
                "events": [{"t": t, "text": x, "level": ev_level(x)} for t, x in list(self.events)[:20]],
                "plate": self.plate(now, V, mode, armed, kill, estop, holder, hb, s if fresh_s else {}),           # v0.1.52
                "sim": self.sim_flag(now), "lmap": self.light_map(now), "console": self.console(now),
                "camv": self.camview(now), "dets": self.dets_view(now),                                               # v0.1.54
                "cmds": list(self.cmds), "whitelist": list(WHITELIST), "buttons": list(PAGE_BUTTONS),
                "ages": {"fcu": None if fcu is None else round(fa, 2), "status": None if st is None else round(sa, 2),
                         "ocs": None if ocs is None else round(oa, 2), "hb": None if hb is None else round(hb, 2)},
                # v0.1.74 (WP30): what the two-row header and the FCU / RC line show - read from the same blocks, nothing new published
                "n6": {"sog": f.get("groundspeed_mps") if fresh_f and f.get("groundspeed_mps") is not None else (s.get("sog_mps") if fresh_s else None),
                       "hdg": f.get("heading_deg") if fresh_f and f.get("heading_deg") is not None else (s.get("hdg_deg") if fresh_s else None),
                       "hb": None if hb is None else round(hb, 2),
                       "ekf_pos_ok": f.get("ekf_pos_ok") if fresh_f else None, "fix": f.get("fix") if fresh_f else None,
                       "rc_ok": h.get("rc_ok") if fresh_s else None, "sf": bool(f.get("rc5_estop") or f.get("kill") or s.get("kill")),
                       "rc6": f.get("rc6_override") if fresh_f else None, "pwm": (f.get("servo") or [])[:4] if fresh_f else [],
                       "fcu_link": f.get("fcu_link") if fcu is not None else None, "log": f.get("log") if fresh_f else None,
                       "rates": f.get("rates_hz") or {} if fresh_f else {}, "faults": list(h.get("faults") or []) if fresh_s else [],
                       "perc_age": None if p is None else round(pa, 2), "status_age": None if st is None else round(sa, 2),
                       "vstate": s.get("state") if fresh_s else None, "estop": bool(s.get("estop")) if fresh_s else False}}   # v0.1.78.9: CONFIRM SAFE


# ---- v0.1.52: the top plate's classes, the event levels, the callouts' short words, the light map's fit ----
PRES_STALE_S = 6.0          # usv_ui's presence beat (every 2 s)
AUTO_FCU_MODES = ("AUTO", "GUIDED", "RTL", "SMART_RTL", "LOITER")       # the FCU's own controller steers
MANUAL_FCU_MODES = ("MANUAL", "ACRO", "STEERING")
WF_TAB = {"thr-mp": "ACTUATION", "thr-ms": "ACTUATION", "thr-sf": "ACTUATION", "thr-sa": "ACTUATION", "ant": "DIAG", "box": "DIAG",
          "cam": "NAV", "lamp": None, "cannon": None}


def vehicle_class(v: Optional[str]) -> str:
    """The VEHICLE block: AUTO_* green, MANUAL yellow, KILLED / FAULT red, BOOT neutral, None hatched."""
    if not v:
        return "unk"
    if v in ("KILLED", "FAULT"):
        return "alarm"
    if v == "MANUAL":
        return "manual"
    return "auto" if v.startswith("AUTO") else "neutral"


def fcu_mode_class(mode: Optional[str], holder: str = "unknown") -> str:
    """The FCU MODE block: AUTO / GUIDED / RTL / SMART_RTL / LOITER green; mode 20 (CUSTOM_MANUAL) green under the mission manager,
    yellow under usv_ui (the console's own frames); MANUAL / ACRO / STEERING yellow; HOLD green (v0.1.78.10, Dmitrii: "we start from this
    mode"); the rest neutral; None hatched. v0.1.81 (WP37, Dmitrii 2026-11-11): HOLD keeps its class `hold` but its colour is YELLOW now
    (standby, not autonomous); LOITER green (the position hold)."""
    if not mode:
        return "unk"
    if mode in AUTO_FCU_MODES:
        return "auto"
    if mode == "CUSTOM_MANUAL":
        return "auto" if holder == "mission manager" else "manual"
    if mode == "HOLD":
        return "hold"
    return "manual" if mode in MANUAL_FCU_MODES else "neutral"


_EV_ALARM = ("KILL asserted", "LOST", "DOWN", "REFUSED", "KILLED", "FAULT", "ESTOP", "E-STOP", "not sent", "no answer", "FAILED", "ERROR", "ARMED")
_EV_WARN = ("HANDED OVER", "silent", "SILENT", "STALE", "WEAK", "MANUAL", "RC6", "throttl", "WARN")
_EV_OK = (" UP", "kill clear", "ACTIVE", "disarmed", "AUTO_", " ok", "OK", "GUIDED")


def ev_level(text: str) -> str:
    """An event / console line's level for the colour: alarm | warn | ok | info (the first match wins, alarm first)."""
    t = str(text or "").replace("DISARMED", "disarmed")
    if any(k in t for k in _EV_ALARM):
        return "alarm"
    if any(k in t for k in _EV_WARN):
        return "warn"
    if any(k in t for k in _EV_OK) or "-> " in t:
        return "ok"
    return "info"


def short_word(eid: str, o: dict) -> str:
    """One status word + one value for a wireframe callout (the long text lives in the detail)."""
    st = o.get("state")
    if eid == "lamp":
        if st == "read":                                                   # v0.1.55 (D4)
            return "%s · read" % (o.get("colour") or "OFF")
        return ("%s · expected" % o["colour"]) if st == "expected" and o.get("colour") else "unknown · no state / HB"
    if eid == "ant":
        age = o.get("age_s")
        return {"good": "ok", "weak": "WEAK", "lost": "LOST"}.get(o.get("q"), "unknown") + ("" if age is None else " · status %.1f s" % age)
    if eid == "cam":
        if st == "active":
            return "up%s · %d src" % ("" if o.get("rate") is None else " %.1f Hz" % o["rate"], o.get("n_fresh") or 0)
        return ("STALE · %d src" % len(o.get("sources") or [])) if st == "warning" else "OFF · no perception health"
    if eid == "box":
        if st == "available":
            return "ok" + ((" · %.0f °C" % o["worst_c"]) if o.get("worst_c") is not None else "" if o.get("n_hosts") else " · hosts unknown")
        l1 = str(o.get("l1") or "")
        return {"fault": "ALARM", "warning": "WARNING"}.get(st, "unknown") + ((" · " + l1.split(" · ", 1)[1]) if " · " in l1 and st in ("fault", "warning") else "")
    return str(o.get("l1") or st or "?")


def fit_bounds(pts, min_span: float = 20.0, pad: float = 0.12):
    """The auto-fit box [n_min, n_max, e_min, e_max] around the points, padded, at least min_span metres each way; None if empty."""
    pts = [p for p in pts if p is not None]
    if not pts:
        return None
    ns, es = [p[0] for p in pts], [p[1] for p in pts]
    out = []
    for lo, hi in ((min(ns), max(ns)), (min(es), max(es))):
        span = max(min_span, (hi - lo) * (1 + 2 * pad))
        c = (lo + hi) / 2.0
        out += [round(c - span / 2.0, 2), round(c + span / 2.0, 2)]
    return out


# ---- v0.1.49 (item 19): the radar's projection - heading-up, sqrt radial scale, unit = the 40 m band edge ----
def radar_r(r_m: float) -> float:
    return math.sqrt(max(0.0, min(r_m, RADAR_R_M)) / RADAR_R_M)


def radar_xy(dn: float, de: float, hdg_deg: float):
    """(north, east) metres from the boat -> (x right, y down) in units of the radar radius, bow up."""
    r = math.hypot(dn, de)
    rel = math.atan2(de, dn) - math.radians(hdg_deg)
    s = radar_r(r)
    return [round(s * math.sin(rel), 4), round(-s * math.cos(rel), 4)]


def frange(a: float, b: float, step: float):
    n = max(1, int(abs(b - a) / step))
    return [a + (b - a) * i / n for i in range(n + 1)]


# ---- v0.1.49 (item 21): the wireframe at BlueBoat proportions and the callout placer ----
# The drawing: 1 m = 400 units (the top projection 1.19 x 0.76 m -> 476 x 304, hull x 328..632, y 112..588, bow up), shown at 0.75
# (WF_T) in a 900 x 600 viewBox so the callout columns either side of the hull keep a readable 14-unit type at 1440 px.
WF_W, WF_H = 900, 600
WF_T = (336.0, 70.0, 0.75)                                      # translate x, y, scale (the page's <g transform>)


def wf_xy(x: float, y: float):
    return (round(WF_T[0] + WF_T[2] * (x - 328.0), 1), round(WF_T[1] + WF_T[2] * (y - 112.0), 1))


HULL = dict(zip(("x0", "y0"), wf_xy(328.0, 112.0)), **dict(zip(("x1", "y1"), wf_xy(632.0, 588.0))))
ANCHOR = {k: wf_xy(*v) for k, v in {"thr-mp": (364.0, 592.0), "thr-ms": (596.0, 592.0), "thr-sf": (480.0, 215.0), "thr-sa": (480.0, 501.0),
                                    "cannon": (480.0, 151.0), "cam": (480.0, 268.0), "lamp": (480.0, 306.0), "ant": (480.0, 344.0),
                                    "box": (480.0, 401.0)}.items()}
SIDE = {"thr-mp": "L", "thr-sa": "L", "cam": "L", "box": "L", "thr-ms": "R", "thr-sf": "R", "cannon": "R", "lamp": "R", "ant": "R"}
LINES = {"thr-mp": 1, "thr-ms": 1, "thr-sf": 1, "thr-sa": 1, "cannon": 1, "cam": 1, "lamp": 1, "ant": 1, "box": 1}   # fixed heights: no jumps
# (v0.1.52: one value row per element - a word + one value; the detail is the tooltip / the click panel)
COL = {"L": (8.0, 318.0), "R": (582.0, 892.0)}
CO_FONT, CO_CHAR, CO_LINE, CO_PAD = 14.0, 8.7, 17.0, 6.0       # monospace 14 units: 0.62 em per character


def _cut(t: str, n: int) -> str:
    t = str(t if t is not None else "")
    return t if len(t) <= n else t[: n - 1] + "…"


def _wrap(lines, n: int, budget: int):
    """The lines folded at n characters (at ' · ' or a space when it can), at most `budget` rows, the last cut with '…'."""
    rows = []
    for t in lines:
        t = str(t if t is not None else "")
        while t:
            if len(t) <= n:
                rows.append(t)
                break
            k = t.rfind(" · ", 0, n + 1)
            k = k if k > n // 3 else t.rfind(" ", 0, n + 1)
            k = k if k > n // 3 else n
            rows.append(t[:k].rstrip())
            t = t[k:].lstrip(" ·")
    if len(rows) > budget:
        rows = rows[:budget]
        rows[-1] = _cut(rows[-1] + " …", n) if len(rows[-1]) < n - 1 else rows[-1][: n - 1] + "…"
    return rows


def place_callouts(items, top: float = 8.0, bottom: float = WF_H - 22.0, gap: float = 8.0):
    """items = [(id, title, [lines], state)] -> boxes {id, x, y, w, h, title, lines, state, ax, ay, lx, ly} in two columns beside the hull,
    each box as near its element's height as the column allows, none overlapping (a 1-D sweep down, then up from the bottom) - like
    map.html's lbl(), but every box has a fixed number of rows (LINES), so a changing value never moves a box."""
    out = []
    for side in ("L", "R"):
        x0, x1 = COL[side]
        w = x1 - x0
        nch = int((w - 2 * CO_PAD) / CO_CHAR)
        col = sorted([it for it in items if SIDE.get(it[0], "L") == side], key=lambda it: ANCHOR[it[0]][1])
        boxes = []
        for eid, title, lines, state in col:
            n = LINES.get(eid, 3)
            rows = _wrap([x for x in lines if x not in (None, "")], nch, n)
            h = CO_PAD * 2 + CO_LINE * (1 + n)
            boxes.append({"id": eid, "x": x0, "w": w, "h": h, "y": ANCHOR[eid][1] - h / 2.0, "title": _cut(title, nch), "lines": rows,
                          "state": state, "ax": ANCHOR[eid][0], "ay": ANCHOR[eid][1]})
        prev = top - gap
        for b in boxes:
            b["y"] = max(b["y"], prev + gap)
            prev = b["y"] + b["h"]
        nxt = bottom + gap
        for b in reversed(boxes):
            b["y"] = min(b["y"], nxt - gap - b["h"])
            nxt = b["y"]
        for b in boxes:
            b["y"] = round(b["y"], 1)
            b["lx"] = x1 if side == "L" else x0
            b["ly"] = round(min(max(b["ay"], b["y"] + 8.0), b["y"] + b["h"] - 8.0), 1)
        out += boxes
    return out


def overlaps(boxes) -> List[tuple]:
    """The pairs of boxes (and box / hull) that intersect - [] when the placer did its job."""
    bad = []
    rects = [(b["id"], b["x"], b["y"], b["w"], b["h"]) for b in boxes] + [("hull", HULL["x0"], HULL["y0"], HULL["x1"] - HULL["x0"], HULL["y1"] - HULL["y0"])]
    for i, a in enumerate(rects):
        for c in rects[i + 1:]:
            if a[1] < c[1] + c[3] and c[1] < a[1] + a[3] and a[2] < c[2] + c[4] and c[2] < a[2] + a[4]:
                bad.append((a[0], c[0]))
    return bad


class Link:
    """The boat broker: subscribe the model's topics, publish the page's words, wait for usv_ui's answer."""

    def __init__(self, model: Model, host: str, port: int):
        import paho.mqtt.client as mqtt
        self.m = model
        self.m.broker = "%s:%d" % (host, port)
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="boat-web-%s" % uuid.uuid4().hex[:8], protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = lambda *a, **k: setattr(self.m, "connected", False)
        self.c.on_message = lambda c, u, msg: self.m.on_message(msg.topic, msg.payload)
        self.c.reconnect_delay_set(1, 4)
        self.c.connect_async(host, port, keepalive=15)
        self.c.loop_start()
        self._t_last = 0.0

    def _on_connect(self, c, u, f, rc, p=None):
        self.m.connected = True
        for t in self.m.topics():
            c.subscribe(t, 0)

    def send(self, word: str, args=None, timeout_s: float = 3.0) -> dict:
        word = str(word or "").strip().lower()
        if word not in WHITELIST:
            out = {"ok": False, "word": word, "text": "refused by the page: '%s' is not a page word (only %s; decision 25)" % (word, " ".join(WHITELIST))}
            self.m.cmds.appendleft(dict(out, t=time.time()))
            return out
        if not self.m.connected:
            return {"ok": False, "word": word, "text": "not sent: this page is not connected to the boat broker (%s)" % self.m.broker}
        cid = uuid.uuid4().hex[:12]
        self.c.publish("local/%s/ui/cmd" % self.m.v, json.dumps({"word": word, "args": args or [], "id": cid, "t": time.time(), "src": "boat-web"}), qos=0)
        t_end = time.monotonic() + timeout_s
        with self.m.cond:
            while cid not in self.m.results and time.monotonic() < t_end:
                self.m.cond.wait(max(0.01, t_end - time.monotonic()))
            r = self.m.results.pop(cid, None)
        out = ({"ok": bool(r.get("ok")), "word": word, "text": r.get("text"), "active": r.get("active")} if r else
               {"ok": False, "word": word, "text": "no answer from usv_ui in %.0f s (is it running on %s?)" % (timeout_s, self.m.broker)})
        self.m.cmds.appendleft(dict(out, t=time.time()))
        return out

    def close(self):
        self.c.loop_stop()
        self.c.disconnect()


def _ocs_bar() -> Optional[dict]:
    try:
        from .judge_web import _ocs_bar as ob          # the OCS page's global bar, as the judge page reads it
        return ob()
    except Exception:
        return None


def fetch_host_thresholds(model: "Model", ocs_url: str, timeout_s: float = 2.0) -> bool:
    """v0.1.55 (D3): host.warn_c / alarm_c from the OCS profile through the OCS page's /meta.json; the --warn-c / --alarm-c flags
    stay the fallback (model.thr_src says which one is in force)."""
    import urllib.request
    try:
        with urllib.request.urlopen(ocs_url.rstrip("/") + "/meta.json", timeout=timeout_s) as r:
            th = (json.loads(r.read().decode()) or {}).get("host_thresholds") or {}
        if th.get("warn_c") is not None and th.get("alarm_c") is not None:
            model.warn_c, model.alarm_c, model.thr_src = float(th["warn_c"]), float(th["alarm_c"]), "OCS profile"
            return True
    except Exception:
        pass
    return False


def ocs_tune(ocs_url: str, req: dict, timeout_s: float = 8.0) -> dict:
    """v0.1.49 (item 22): the TUNING card's apply / save -> the OCS page's /cmd as `tune KEY VALUE` / `tune save` (op-tui's grammar);
    the OCS's authority decides (TUNE: whitelist, range, MANUAL / AUTO_HOLD) and the mission manager acks. The reply = the OCS's lines."""
    import urllib.request
    if req.get("save"):
        line = "tune save"
    else:
        key, value = str(req.get("key") or ""), req.get("value")
        why, v = TUN.check(key, value)                 # the page's own check first (the same table), so a typo never travels
        if why:
            return {"ok": False, "text": "refused by the page: " + why, "lines": [why]}
        line = "tune %s %g" % (key, v)
    try:
        r = urllib.request.Request(ocs_url.rstrip("/") + "/cmd", data=json.dumps({"line": line}).encode(), method="POST",
                                   headers={"Content-Type": "application/json", "X-OCS-Web": "1"})
        with urllib.request.urlopen(r, timeout=timeout_s) as fh:
            rec = json.loads(fh.read())
    except Exception as exc:
        return {"ok": False, "text": "not sent: the OCS page (%s) is not reachable - %s (the authority is the OCS: no OCS, no tuning)" % (ocs_url, exc),
                "lines": [], "line": line}
    lines = [str(x) for x in rec.get("lines") or []]
    return {"ok": bool(rec.get("ok")), "text": " | ".join(x.strip() for x in lines[-3:]) or ("ok" if rec.get("ok") else "no answer"),
            "lines": lines, "line": line}


def ocs_line(ocs_url: str, line: str, timeout_s: Optional[float] = None) -> dict:
    """v0.1.50: the FCU card's RC6 rule -> the OCS page's /cmd (`rc6_policy latched|hold`); only OCS_LINES travel.
    v0.1.54: + `stop_reset` (the ■ STOP & RESET button; the OCS runs the macro, its step lines come back)."""
    import urllib.request
    line = " ".join(str(line or "").split()).lower()
    if timeout_s is None:
        timeout_s = OCS_LINE_TIMEOUT_S.get(line, 8.0)
    if line not in OCS_LINES:
        return {"ok": False, "text": "refused by the page: only %s" % " | ".join(OCS_LINES), "line": line}
    try:
        r = urllib.request.Request(ocs_url.rstrip("/") + "/cmd", data=json.dumps({"line": line}).encode(), method="POST",
                                   headers={"Content-Type": "application/json", "X-OCS-Web": "1"})
        with urllib.request.urlopen(r, timeout=timeout_s) as fh:
            rec = json.loads(fh.read())
    except Exception as exc:
        return {"ok": False, "text": "not sent: the OCS page (%s) is not reachable - %s" % (ocs_url, exc), "line": line}
    lines = [str(x) for x in rec.get("lines") or []]
    return {"ok": bool(rec.get("ok")), "text": " | ".join(x.strip() for x in lines[-3:]) or "no answer", "line": line, "lines": lines}


def ocs_mqtt_line(hub: str, line: str, vehicle: str = "usv", vehicle_id: str = "USV1", timeout_s: float = 6.0) -> dict:
    """v0.1.63: the same OCS lines (OCS_LINES, `tune ...`) when this window runs on the NX - the OCS page is private to the Mac, so the
    line goes the way op-tui sends it: ocs_cli on the hub broker (HOST:PORT), the OCS app's authority decides, the intent answers."""
    from . import ocs_cli
    host, _, port = str(hub).rpartition(":")
    con = ocs_cli.OcsConsole(host or "127.0.0.1", int(port or 1882), vehicle_id=vehicle_id, vehicle=vehicle, operator="usv-window")
    try:
        try:
            con.start()
        except OSError as exc:
            return {"ok": False, "text": "not sent: the hub broker (%s) is not reachable from here (%s)" % (hub, exc), "line": line}
        if not con.connected(min(4.0, timeout_s)):
            return {"ok": False, "text": "not sent: the hub broker (%s) is not reachable from here" % hub, "line": line}
        code, lines = ocs_cli.execute(con, ocs_cli.parse(line.split()))
    except SystemExit:
        return {"ok": False, "text": "refused: not an OCS line (%s)" % line, "line": line}
    except Exception as exc:
        return {"ok": False, "text": "not sent: %s" % exc, "line": line}
    finally:
        try:
            con.stop()
        except Exception:
            pass
    lines = [str(x) for x in lines or []]
    return {"ok": code == 0, "text": " | ".join(x.strip() for x in lines[-3:]) or "no answer", "line": line, "lines": lines}


def ocs_send(ocs_url: Optional[str], ocs_mqtt: Optional[str], line: str, tune: Optional[dict] = None) -> dict:
    """v0.1.63: an OCS line through the OCS page (the Mac) or, without one, through the hub (the NX). `tune` = the TUNING card's request."""
    if tune is not None:
        if ocs_url:
            return ocs_tune(ocs_url, tune)
        if tune.get("save"):
            tl = "tune save"
        else:
            why, v = TUN.check(str(tune.get("key") or ""), tune.get("value"))
            if why:
                return {"ok": False, "text": "refused by the page: " + why, "lines": [why]}
            tl = "tune %s %g" % (tune.get("key"), v)
        return ocs_mqtt_line(ocs_mqtt, tl) if ocs_mqtt else {"ok": False, "text": "no OCS configured (--ocs-url / --ocs-mqtt)"}
    if ocs_url:
        return ocs_line(ocs_url, line)
    line = " ".join(str(line or "").split()).lower()
    if line not in OCS_LINES:
        return {"ok": False, "text": "refused by the page: only %s" % " | ".join(OCS_LINES), "line": line}
    return ocs_mqtt_line(ocs_mqtt, line) if ocs_mqtt else {"ok": False, "text": "no OCS configured (--ocs-url / --ocs-mqtt)", "line": line}


def cam_poller(model: Model, url: str, period_s: float = 2.0) -> threading.Thread:
    """The camera stream's /status (cv/stream or the sim's) into model.cam every 2 s - the camera callout names the source."""
    import urllib.request

    def run():
        while True:
            try:
                with urllib.request.urlopen(url, timeout=0.8) as fh:
                    model.cam = (time.monotonic(), json.loads(fh.read()), url, None)
            except Exception as exc:
                model.cam = (time.monotonic(), None, url, type(exc).__name__)
            time.sleep(period_s)
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


def make_handler(model: Model, link: Optional[Link] = None, bar=_ocs_bar, ocs_url: Optional[str] = None, control=None,
                 ocs_mqtt: Optional[str] = None, where: Optional[dict] = None):
    from .window_control import Control
    control = control or Control()                      # v0.1.63: one commander, take over at once; a PIN for other computers

    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _who(self):
            return self.headers.get("X-Window-Session"), self.headers.get("X-Window-Name"), self.client_address[0]

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/v1", "/v1/", "/v1/index.html"):        # v0.1.74 (WP30): the v0.1.68 window, frozen (the GUI switch's classic)
                with open(os.path.join(HERE, "boat_web_v1.html"), "rb") as fh:
                    return self._send(200, fh.read(), "text/html; charset=utf-8")
            if path.startswith("/v1/"):
                path = path[3:]                                  # the classic page's own requests reach the same endpoints
            if path in ("/", "/index.html"):
                name = "boat_web_v1.html" if os.environ.get("BOAT_WEB_UI") == "classic" else "boat_web.html"   # v0.1.74: --classic
                with open(os.path.join(HERE, name), "rb") as fh:
                    return self._send(200, fh.read(), "text/html; charset=utf-8")
            if path == "/themes.css":
                with open(os.path.join(HERE, "themes.css"), "rb") as fh:
                    return self._send(200, fh.read(), "text/css; charset=utf-8")
            if path == "/prefs":                        # v0.1.49: the shared theme (the OCS page's /prefs, the launcher's when it is down)
                from . import prefs as P
                t = P.get_theme(ocs_url) if ocs_url else None
                r = {"theme": t, "from": "ocs"} if t is not None else P.read_shared()
                return self._send(200, json.dumps({"ok": True, "prefs": {"theme": r["theme"]} if r["theme"] else {}, "from": r["from"]}).encode(),
                                  "application/json")
            if path == "/state.json":
                sid, name, addr = self._who()
                control.seen(sid, name, addr)
                snap = model.snapshot()
                snap["bar"] = bar() if bar else None
                snap["control"] = dict(control.view(sid), where=where or {})      # v0.1.63
                return self._send(200, json.dumps(snap, default=str).encode(), "application/json")
            self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                req = {}
            if self.headers.get("X-Boat-Web") == "1" and self.path == "/prefs":   # v0.1.49: a pick here sets every page's theme
                from . import prefs as P
                t = P.norm(req.get("theme"))
                ok = bool(t) and bool(ocs_url) and P.post_theme(ocs_url, t, "X-OCS-Web")
                return self._send(200, json.dumps({"ok": True, "prefs": {"theme": t}, "to": "ocs"} if ok else P.write_shared(req.get("theme"))).encode(),
                                  "application/json")
            sid, name, addr = self._who()
            if self.headers.get("X-Boat-Web") == "1" and self.path == "/control":  # v0.1.63: take / release control
                op = str(req.get("op") or "")
                ok, why = control.take(sid, name, addr, req.get("pin")) if op == "take" else control.release(sid) if op == "release" else (False, "op: take | release")
                if ok:
                    model.event("USV window: %s" % why)
                return self._send(200, json.dumps({"ok": ok, "text": why, "control": control.view(sid)}).encode(), "application/json")
            if self.headers.get("X-Boat-Web") == "1" and self.path in ("/ocs", "/tune", "/cmd"):   # v0.1.63: only the commander sends
                ok, why = control.may_command(sid, name, addr)
                if not ok:
                    return self._send(200, json.dumps({"ok": False, "text": why, "control": control.view(sid)}).encode(), "application/json")
            if self.headers.get("X-Boat-Web") == "1" and self.path == "/ocs":      # v0.1.50: the RC6 rule through the OCS's authority
                code, out = 200, (ocs_send(ocs_url, ocs_mqtt, req.get("line")) if (ocs_url or ocs_mqtt) else {"ok": False, "text": "no OCS page configured (--ocs-url)"})
                w0 = out.get("line") if out.get("line") in ("stop_reset", "enable_auto", "disable_auto", "confirm_safe") else "rc6"  # v0.1.54 / v0.1.62
                model.cmds.appendleft(dict(word=w0, ok=out["ok"], text=out["text"], t=time.time()))
                if w0 == "stop_reset":
                    model.event("STOP & RESET -> the OCS: %s" % out["text"])
                return self._send(code, json.dumps(out).encode(), "application/json")
            if self.headers.get("X-Boat-Web") == "1" and self.path == "/tune":
                code, out = 200, (ocs_send(ocs_url, ocs_mqtt, "", tune=req) if (ocs_url or ocs_mqtt) else {"ok": False, "text": "no OCS page configured (--ocs-url)"})
                model.cmds.appendleft(dict(word="tune", ok=out["ok"], text=out["text"], t=time.time()))
                model.event("tune: %s -> %s" % (out.get("line") or req, out["text"]))
            elif self.headers.get("X-Boat-Web") != "1" or self.path != "/cmd":
                code, out = 403, {"ok": False, "text": "refused (needs POST /cmd with X-Boat-Web: 1)"}
            elif link is None:
                code, out = 200, {"ok": False, "text": "this page is read-only (no boat broker link)"}
            else:
                code, out = 200, link.send(req.get("word"), req.get("args"))
            self._send(code, json.dumps(out).encode(), "application/json")
    return H


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):                       # no reverse-DNS getfqdn() (slow on macOS)
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = str(self.server_address[0]), int(self.server_address[1])


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--boat-host", default=os.environ.get("BOAT_HOST", "127.0.0.1"), help="the boat broker (run_sim.sh: 127.0.0.1)")
    ap.add_argument("--boat-port", type=int, default=1884)
    ap.add_argument("--port", type=int, default=8098, help="this page's port (the OCS page links http://127.0.0.1:8098/)")
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--thr-map", default=None, help="thruster -> output channel, e.g. mp=1,ms=3,sf=4,sa=2 (default: usv_ui's motor names)")
    ap.add_argument("--warn-c", type=float, default=85.0, help="host temperature warning (profile host.warn_c)")
    ap.add_argument("--alarm-c", type=float, default=95.0, help="host temperature alarm (profile host.alarm_c)")
    ap.add_argument("--ocs-url", default=os.environ.get("OCS_WEB_URL", "http://127.0.0.1:%s" % os.environ.get("OCS_WEB_PORT", "8096")),
                    help="v0.1.49: the OCS page the TUNING card sends `tune ...` to (its /cmd; the authority is the OCS)")
    ap.add_argument("--cam-status", default=os.environ.get("CAM_STATUS_URL", "http://127.0.0.1:8097/status"),
                    help="v0.1.49: the camera stream's /status (the sim's 8097 by default; on the boat cv/stream http://AGX:PORT/status); '' = off")
    ap.add_argument("--ocs-mqtt", default=os.environ.get("OCS_MQTT", ""),
                    help="v0.1.63: HOST:PORT of the hub broker - the OCS lines go through it when this window runs away from the OCS "
                         "page (the NX); with --ocs-url '' it is the only path")
    ap.add_argument("--pin", default=os.environ.get("USV_WINDOW_PIN", ""),
                    help="v0.1.63: the PIN a browser on another computer gives once before it may command (watching needs none); "
                         "v0.1.78.9: default 0000 (DEFAULT_PIN), shown in the launcher")
    ap.add_argument("--where", default=os.environ.get("USV_WINDOW_WHERE", ""), help="v0.1.63: nx | mac - shown on the pages")
    ap.add_argument("--open", action="store_true", help="open the page in the browser (macOS `open`)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    model = Model(a.vehicle, parse_thr_map(a.thr_map), a.warn_c, a.alarm_c)
    link = Link(model, a.boat_host, a.boat_port)
    def _thr_loop():                                                     # v0.1.55 (D3): until the OCS answers, then every 60 s
        while True:
            ok = fetch_host_thresholds(model, a.ocs_url)
            time.sleep(60.0 if ok else 5.0)
    threading.Thread(target=_thr_loop, daemon=True).start()
    from .window_control import Control
    pin = a.pin.strip() or DEFAULT_PIN                  # v0.1.78.9 (Dmitrii 2026-11-09): a fixed, known PIN "0000" (USV_WINDOW_PIN / --pin override)
    print("[boat_web] commands from other computers need the PIN %s (watching needs none; this computer needs none) - USV_WINDOW_PIN changes it"
          % pin, flush=True)
    control = Control(pin)
    url_pub = "http://%s:%d/" % ("127.0.0.1" if a.bind in ("0.0.0.0", "") else a.bind, a.port)
    where = {"where": a.where or ("this host"), "url": url_pub, "host": socket.gethostname(), "ocs": "page" if a.ocs_url else ("hub " + a.ocs_mqtt if a.ocs_mqtt else "none")}

    def _pub_loop():                                                     # v0.1.63: who is connected / commands -> the OCS SYSTEM card
        while True:
            try:
                v = control.view()
                link.c.publish("local/%s/ui/window" % a.vehicle, json.dumps({"t": time.time(), **where, "commander": v["commander"],
                                                                              "sessions": v["sessions"], "pin": v["pin"]}), qos=0)
            except Exception:
                pass
            time.sleep(1.0)
    threading.Thread(target=_pub_loop, daemon=True).start()
    srv = Server((a.bind, a.port), make_handler(model, link, ocs_url=a.ocs_url or None, control=control, ocs_mqtt=a.ocs_mqtt or None, where=where))
    if a.cam_status:
        cam_poller(model, a.cam_status)
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    url = "http://%s:%d/" % ("127.0.0.1" if a.bind in ("0.0.0.0", "") else a.bind, a.port)
    print("[boat_web] the boat window of %s:%d (vehicle %s) on %s - buttons: %s (usv_ui's own words)" % (
        a.boat_host, a.boat_port, a.vehicle, url, " ".join(PAGE_BUTTONS)), flush=True)
    if a.open and sys.platform == "darwin":
        subprocess.Popen(["open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        link.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
