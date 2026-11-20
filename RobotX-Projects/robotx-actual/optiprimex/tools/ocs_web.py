"""ocs_web: the OCS in the browser (v0.1.28, ROADMAP WP8a) - op-tui's view and commands, grouped, with tooltips and drop-downs,
plus a SYSTEM tab (restart / reboot the Mac's OCS stack, the boat stack with its FCU and profile, the AGX, the FCU, the Jetsons).

op-tui stays the main console: this page runs on op-tui's own Model and Feeds (tools/bench_tui) and sends through the same
`ocs_cli.execute` with op-tui's grammar (intents + the fake plant's words), so both agree and both work at once. The playbook
(tools/playbook) decides what is allowed now (the buttons grey out and say why) and gives every tooltip. Plain HTML/JS
(tools/ocs_web.html), no build step, no external resources; the map is today's map.html in a tab.

  ocs-web [--host 127.0.0.1 --port 1882] [--boat-host H --boat-port 1884] [--web-port 8096] [--bind 127.0.0.1] [--open]
  normally window `web` of tmux `ocs` (scripts/run_ocs.sh), which opens http://127.0.0.1:8096/ (MAP=0 = not).

GET /  the page · /meta.json  the catalogue (commands, playbook entries, missions + their options, SYSTEM actions) ·
/state.json  the live state (1 Hz) · /map.html  the map · /cam.json  the AGX camera stream's /status (asked from here: no CORS) ·
POST /cmd {"line": "..."} · POST /system {"action", "options", "confirm"}.
v0.1.29 (WP8b): MAIN gets the run checklist (each condition, the next step as one button, `clear for run` = the whole chain),
the mission picker with the selected mission's own controls (m0t2's points, m0t4's colour, step / resume), a RoboCommand peek
(our declaration and the judge's answer, the last reports, a live judge feed - read-only, on the judge's broker --judge-host /
--judge-port), and problems highlighted in every log. POST /clear runs `ready` after the hard checks pass.
v0.1.28.1: CAMERA tab (the stream, a toggle that also starts / stops it on the AGX) and SETTINGS (the gear: sim scenario, camera,
quick stops that need `stop` typed, the SYSTEM cards again, the connections).
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import signal
import socketserver
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Callable, Dict, List, Optional

from .. import __version__
from . import bench_tui as T
from . import ocs_cli, playbook
from .rc_tui import RcModel, monitor_subscriptions
from .system_actions import Runner

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "ocs_web.html")
PAGE_V1 = os.path.join(HERE, "ocs_web_v1.html")
THEMES_CSS = os.path.join(HERE, "themes.css")        # v0.1.43: the classic page, frozen, served at /v1/ (OCS_V2_SPEC §3)

# the options each mission takes, as the page's drop-downs / fields (ocs_cli flags); "from" lists the task's entry stages
MISSION_OPTIONS: Dict[str, List[dict]] = {
    "m1t1": [{"flag": "--planner", "label": "planner", "choices": ["", "simple", "s6"], "help": "the gate planner (empty = t1.planner of the profile)"}],
    "m0t2": [{"flag": "--names", "label": "points", "text": "EN,G1,G2,EX", "help": "saved point names (usv_ui points.json); empty = the gates in the map"}],
    "m0t3": [{"flag": "--names", "label": "points", "text": "A,B", "help": "two saved points; empty = the dock line from the map"}],
    "m3t1": [{"flag": "--bay", "label": "bay", "choices": ["", "1", "2", "3"], "help": "a bay by hand (1..3 left to right facing the dock); empty = the GREEN one"}],
    "m0t4": [{"flag": "--target-colour", "label": "target", "choices": ["", "red", "green", "blue"], "help": "the face to approach; empty = the vehicle's GREEN rule"},
             {"flag": "--scan-x", "label": "scan N", "number": True, "help": "scan point north, m (local)"},
             {"flag": "--scan-y", "label": "scan E", "number": True, "help": "scan point east, m (local)"},
             {"flag": "--scan-hdg", "label": "scan heading", "number": True, "help": "the search looks this way, deg (empty = the heading on arrival)"},
             {"flag": "--strafe-side", "label": "strafe", "choices": ["", "starboard", "port"], "help": "the search sways this way (empty = box_inspect.strafe_side)"},
             {"flag": "--expected-boxes", "label": "faces", "number": True, "help": "faces to survey (default 3)"}],
    "m0t6": [{"flag": "--centre", "label": "centre", "text": "A", "help": "the saved point to orbit (tick one below, or drop it on the map)"},
             {"flag": "--radius", "label": "radius m", "number": True, "help": "orbit radius (default 3 m)"},
             {"flag": "--dir", "label": "direction", "choices": ["", "cw", "ccw"], "help": "cw = clockwise seen from above (default cw)"},
             {"flag": "--turns", "label": "turns", "number": True, "help": "full circles (default 1)"},
             {"flag": "--speed", "label": "speed m/s", "number": True, "help": "speed over ground (default 0.5)"}],
}
STEP_OPTIONS = [{"flag": "--step", "label": "stage by stage", "check": True,
                 "help": "park in AUTO_SAFE_HOLD at every stage boundary; resume enters the next stage"}]


# v0.1.29: op-tui's groups + POINTS - the page only (op-tui's panel has no row left at 150x50)
PAGE_GROUPS = ([(g, ws + ["home"]) if g == "VEHICLE" else (g, ws) for g, ws in T.COMMAND_GROUPS[:3]]      # v0.1.30: + home
               + [("POINTS", ["save_point", "del_point", "clear_map"])] + T.COMMAND_GROUPS[3:])


def entry_points() -> Dict[str, List[str]]:
    try:
        from ..usv.tasks import REGISTRY
        return {name: list(getattr(cls, "ENTRY_POINTS", {}) or {}) for name, cls in REGISTRY.items()}
    except Exception:
        return {}


def lan_agx_host(root: Optional[str] = None) -> str:
    """AGX_HOST of network.lan.env (read-only; the address files are never written here) - v0.1.29.2: the camera is tried on the
    pool switch first, whatever the profile, then over the profile's address (the tailnet)."""
    p = os.path.join(root or os.path.dirname(os.path.dirname(HERE)), "network.lan.env")
    try:
        for line in open(p):
            line = line.split("#", 1)[0].strip()
            if line.startswith("AGX_HOST="):
                return line.split("=", 1)[1].strip()
    except OSError:
        pass
    return ""


def cam_urls(env: Dict[str, str], root: Optional[str] = None) -> List[str]:
    """The camera addresses to try, in order: CAM_URL (explicit, alone), else the LAN AGX, then the profile's AGX_HOST."""
    if env.get("CAM_URL"):
        return [env["CAM_URL"]]
    port = env.get("CAM_PORT", "8090")
    out = []
    for h in (lan_agx_host(root), env.get("AGX_HOST", "")):
        if h and f"http://{h}:{port}/" not in out:
            out.append(f"http://{h}:{port}/")
    return out


def meta(env: Dict[str, str], runner: Runner) -> dict:
    eps = entry_points()
    missions = []
    for code in playbook.MISSION_ORDER:
        m = playbook.MISSIONS.get(code)
        if m is None:
            continue
        opts = list(MISSION_OPTIONS.get(code, []))
        if eps.get(m.task):
            opts = opts + STEP_OPTIONS + [{"flag": "--from", "label": "start at", "choices": [""] + eps[m.task],
                                           "help": "an entry stage (needs its precondition, else REJECTED from: ...)"}]
        missions.append({"code": code, "task": m.task, "title": m.title, "tier": m.tier, "status": m.status, "what": list(m.what),
                         "starts": m.starts, "ends": m.ends, "judge_in": m.judge_in, "judge_out": m.judge_out, "config": m.config,
                         "note": m.note, "stages": [s.__dict__ for s in m.stages], "options": opts,
                         "sim": code != "goto", "hidden": bool(getattr(m, "hidden", False))})   # v0.1.86: Mission.hidden (pickers leave it out)
    return {"version": __version__, "r_order": playbook.R_ORDER, "v_order": playbook.V_ORDER, "r_meaning": playbook.R_MEANING,
            "v_meaning": playbook.V_MEANING, "usage": [{"word": w, "usage": u, "does": d} for w, u, d in T.USAGE],
            "groups": PAGE_GROUPS, "word_to_action": T.WORD_TO_ACTION, "examples": T.EXAMPLES,
            "entries": {a: {k: (list(v) if isinstance(v, tuple) else v) for k, v in e.__dict__.items()} for a, e in playbook.PLAYBOOK.items()},
            "missions": missions, "flow": playbook.FLOW, "flow_caption": playbook.FLOW_CAPTION,
            "provenance": __import__("optiprimex.ocs.provenance", fromlist=["table"]).table(),     # v0.1.73 (WP29): the tooltips' sources
            "system": runner.catalogue({}), "system_dry": runner.dry, "judge_url": f"http://127.0.0.1:{env.get('JUDGE_WEB_PORT', '8095')}/",
            "cam_url": (cam_urls(env) or [""])[0], "cam_urls": cam_urls(env),
            "net_profile": env.get("NET_PROFILE", ""),
            "stage_typical": _stage_typical(),                # v0.1.43: the campaign's median seconds per stage (the stage list)
            "sim_url": env.get("SIM_URL") or f"http://127.0.0.1:{env.get('SIM_PORT', '8097')}/"}      # v0.1.32: the simulator (its own window)


BAD_WORDS = ("refused", "rejected", "failed", "error", "fault", "killed", "lost", "timeout", "not ok", "stall", "down", "denied", "unknown")
WARN_WORDS = ("hold", "stale", "warn", "waiting", "no answer", "retry", "safe_hold")


def severity(text: str) -> str:
    """bad | warn | ok | '' for a log line - the page colours every log by it (v0.1.29)."""
    t = (text or "").lower()
    if any(w in t for w in BAD_WORDS):
        return "bad"
    if any(w in t for w in WARN_WORDS):
        return "warn"
    if any(w in t for w in ("accepted", "done", " ok", "started", "saved", "running")):
        return "ok"
    return ""


class JudgePeek:
    """A read-only RoboCommand listener for MAIN (v0.1.29): the judge's broker (our hub with the fake judge, rc-test 1885, the
    course RoboCommand), both wires, decoded by tools/rc_tui.RcModel. It never publishes."""

    def __init__(self, host: str, port: int, team: str, start: bool = True):
        self.host, self.port = host, port
        self.m = RcModel(team)
        self.c = None
        if start:
            import paho.mqtt.client as mqtt
            self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"ocs-web-peek-{int(time.time() * 1000) % 1000000}", protocol=mqtt.MQTTv311)
            self.c.on_connect = lambda cl, u, f, rc, p=None: (setattr(self.m, "connected", True), cl.subscribe(monitor_subscriptions(team)))   # v0.1.59 (A3)
            self.c.on_disconnect = lambda *x: setattr(self.m, "connected", False)
            self.c.on_message = lambda cl, u, msg: self.m.on_wire(msg.topic, msg.payload)
            self.c.reconnect_delay_set(1, 5)
            self.c.connect_async(host, port, keepalive=15)
            self.c.loop_start()

    def stop(self) -> None:
        if self.c is not None:
            self.c.loop_stop()

    def summary(self, now: Optional[float] = None) -> dict:
        now = time.time() if now is None else now
        m = self.m
        with m.lock:
            decl = m.declarations[-1] if m.declarations else None
            rs = m.run_start
            course = m.course
            reports = [{"vehicle": v, "kind": k, "ago_s": round(now - r.get("_t", now), 0)} for v, reps in m.last_report.items() for k, r in reps.items()]
            feed = sorted([(t, "to", x) for t, x in m.received] + [(t, "from", x) for t, x in m.sent], key=lambda e: e[0])[-25:]
            hb = {v: round(now - h.get("_t", now), 1) for v, h in m.heartbeats.items()}
            course_t = m.course_t
        answer, style = "no declaration seen by the judge yet", ""
        if decl is not None:
            seq, dt = decl.get("request_seq"), decl.get("_t", now)
            if rs and rs.get("declaration_seq") == seq:
                answer, style = f"RunStart run {rs.get('run_id')} for declaration {seq} after {max(0.0, rs.get('_t', now) - dt):.1f} s", "ok"
            else:
                waited = now - dt
                answer, style = f"declaration {seq}: no RunStart for {waited:.0f} s", ("bad" if waited > 30 else "warn")
        return {"broker": f"{self.host}:{self.port}", "connected": m.connected, "wires": dict(m.wires_seen),
                "course": None if not course else {"id": course.get("course_id"), "ago_s": round(now - course_t, 0)},
                "declaration": None if not decl else {"seq": decl.get("request_seq"), "ago_s": round(now - decl.get("_t", now), 0),
                                                      "vehicles": decl.get("vehicle_ids")},
                "run_start": None if not rs else {"seq": rs.get("declaration_seq"), "run_id": rs.get("run_id"), "ago_s": round(now - rs.get("_t", now), 0)},
                "answer": answer, "answer_style": style, "heartbeat_age": hb, "reports": sorted(reports, key=lambda r: r["ago_s"])[:6],
                "feed": [{"t": t, "dir": d, "text": x, "sev": severity(x)} for t, d, x in feed]}


# ---- v0.1.33 (WP10b): the course drawn before the judge's, the editable geofence, the simulator behind the page --------------
GEOFENCE_LINE = re.compile(r"^geofence:.*$", re.M)


def profile_view(path: Optional[str]) -> dict:
    """The OCS profile's `course.default` and `geofence` (lat, lon lists); {} without a profile."""
    if not path or not os.path.isfile(path):
        return {"default": None, "geofence": [], "profile": path or ""}
    import yaml
    with open(path) as fh:
        d = yaml.safe_load(fh) or {}
    crs = (d.get("course") or {}).get("default") or None
    return {"default": crs, "geofence": [list(map(float, p)) for p in d.get("geofence") or []], "profile": os.path.relpath(path, ROOT)}


def save_geofence(path: Optional[str], pts) -> dict:
    """Rewrite the profile's one `geofence:` line (flow style, 7 decimals); refuses fewer than 3 corners (0 = clear) or bad numbers."""
    if not path or not os.path.isfile(path):
        return {"ok": False, "why": "no OCS profile (ocs_web --ocs-config)"}
    try:
        pts = [[round(float(p[0]), 7), round(float(p[1]), 7)] for p in pts or []]
    except (TypeError, ValueError, IndexError):
        return {"ok": False, "why": "geofence: a list of [lat, lon]"}
    if 0 < len(pts) < 3 or any(not (-90 <= a <= 90 and -180 <= b <= 180) for a, b in pts):
        return {"ok": False, "why": "geofence: 3 corners or more (or none to clear), lat / lon in range"}
    with open(path) as fh:
        text = fh.read()
    line = "geofence: [" + ", ".join(f"[{a}, {b}]" for a, b in pts) + "]"
    text = GEOFENCE_LINE.sub(line, text, count=1) if GEOFENCE_LINE.search(text) else text.rstrip("\n") + "\n" + line + "\n"
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        fh.write(text)
    os.replace(tmp, path)
    return {"ok": True, "corners": len(pts), "profile": os.path.relpath(path, ROOT)}


def sim_post(sim_url: str, path: str, body: dict, timeout_s: float = 60.0) -> dict:
    """POST to the simulator (simview) with its header - the page's MAP teleports and RESET go through here."""
    import urllib.error
    import urllib.request
    if path not in ("/teleport", "/reset", "/objects", "/waypoint", "/world"):       # v0.1.53: /world = move to the received course / ignore
        return {"ok": False, "why": "sim: /teleport | /reset | /objects | /waypoint | /world"}
    req = urllib.request.Request(sim_url.rstrip("/") + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json", "X-Sim-Web": "1"})
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        try:
            return json.loads(e.read())
        except ValueError:
            return {"ok": False, "why": f"simulator: HTTP {e.code}"}
    except (OSError, ValueError) as exc:
        return {"ok": False, "why": f"no simulator at {sim_url} ({exc.__class__.__name__}: scripts/run_sim.sh)"}


MARKERS = os.path.join(ROOT, "logs", "ocs", "markers.json")
STATE_TTL_S = 0.25                                        # v0.1.86.9 (B5): /state.json shared between clients within this age
MARKER_NAME = re.compile(r"^[A-Za-z0-9 _.:-]{1,24}$")


class Markers:
    """v0.1.35 (WP10c): named notes on the map - OCS-side only, never sent to the boat. One JSON file (atomic writes):
    {name: {lat, lon, note, t}}. The map draws them; drag moves, Delete removes."""

    def __init__(self, path: str = MARKERS):
        self.path = path
        self.lock = threading.Lock()

    def load(self) -> Dict[str, dict]:
        try:
            with open(self.path) as fh:
                d = json.load(fh)
            return d if isinstance(d, dict) else {}
        except (OSError, ValueError):
            return {}

    def _save(self, d: Dict[str, dict]) -> None:
        os.makedirs(os.path.dirname(self.path), exist_ok=True)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(d, fh, indent=1)
        os.replace(tmp, self.path)

    def apply(self, body: dict) -> dict:
        op, name = body.get("op"), str(body.get("name") or "").strip()
        if not MARKER_NAME.match(name):
            return {"ok": False, "why": "marker name: 1-24 of letters, digits, space _ . : -"}
        with self.lock:
            d = self.load()
            if op in ("add", "move"):
                try:
                    lat, lon = float(body["lat"]), float(body["lon"])
                except (KeyError, TypeError, ValueError):
                    return {"ok": False, "why": "marker: lat and lon"}
                if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                    return {"ok": False, "why": "marker: lat / lon out of range"}
                if op == "move" and name not in d:
                    return {"ok": False, "why": f"no marker {name!r}"}
                old = d.get(name) or {}
                d[name] = {"lat": round(lat, 8), "lon": round(lon, 8), "note": str(body.get("note", old.get("note", "")))[:120],
                           "t": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())}
                self._save(d)
                return {"ok": True, "name": name, "replaced": op == "add" and bool(old)}
            if op == "del":
                if name not in d:
                    return {"ok": False, "why": f"no marker {name!r}"}
                del d[name]
                self._save(d)
                return {"ok": True, "name": name}
        return {"ok": False, "why": "marker op: add | move | del"}


# ---- v0.1.37 (WP10d): MISSION CONTROL's state word, the judges' view (the run timeline as RoboCommand's own display shows it) ---------
_LAUNCH_CACHE: Dict[str, Any] = {"mtime": None, "body": None}


def launch_defaults() -> Optional[dict]:
    """v0.1.47 (UI_QOL item 11): the launcher's record (logs/launch/last.json) as the SYSTEM ACTIONS cards' defaults - {when, preset,
    stamp, options: {action: {option: value}}}; None when nothing was launched from the launcher. Cached on the file's mtime."""
    try:
        from .launcher import RECORD, load_record
        mt = os.path.getmtime(RECORD)
    except (OSError, ImportError):
        return None
    if _LAUNCH_CACHE["mtime"] == mt:
        return _LAUNCH_CACHE["body"]
    rec = load_record() or {}
    c = rec.get("cfg") or {}
    if not c:
        return None
    net = c.get("net") if c.get("net") in ("ts", "lan") else "profile"
    log = "on" if c.get("log") else "off"
    opts = {"ocs": {"judge": c.get("judge"), "net": net, "log": log},
            "nx": {"fcu": c.get("fcu"), "profile": c.get("usv_profile"), "net": net, "log": log},
            "agx": {"mode": {"real": "real", "fake": "sim"}.get(c.get("cv"), None), "world": c.get("world"), "net": net}}
    body = {"when": rec.get("when"), "preset": c.get("preset") or rec.get("title"), "stamp": (rec.get("plan") or {}).get("stamp"),
            "mode": c.get("mode"), "options": {a: {k: v for k, v in o.items() if v} for a, o in opts.items()}}
    _LAUNCH_CACHE.update(mtime=mt, body=body)
    return body


def _scene() -> Optional[dict]:
    """v0.1.44: the map's scene layer (profiles/course/scene_marina.json - illustrative until the OSM export)."""
    try:
        with open(os.path.join(ROOT, "profiles", "course", "scene_marina.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def _stage_typical() -> dict:
    try:
        with open(os.path.join(HERE, "stage_typical.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {}


LINK_LOST_S = 3.0          # OCS_V2_SPEC §8: bridge down = the boat's status older than 3 s
PERCEPTION_STALE_S = 2.0        # the mock and §2 rule 12: perception limit 2 s


def safety_rows(snap: dict) -> None:
    """v0.1.43 (OCS_V2_SPEC §8, the fault-injection test): rows the alarm table needs that the monitor did not raise - RC lost (ALARM
    in AUTO), the boat link lost (status older than 3 s: ALARM in AUTO), perception stale (WARNING). Each says the first action."""
    st, h = snap.get("status") or {}, (snap.get("status") or {}).get("health") or {}
    auto = str(st.get("state") or "").startswith("AUTO")
    rows = snap.setdefault("warnings", [])
    sa = snap.get("status_age")
    if st and sa is not None and sa > LINK_LOST_S:
        rows.insert(0, {"age": sa, "style": "bad" if auto else "warn",
                        "text": f"BOAT LINK LOST: status {sa:.0f} s old (bridge / radio down) - check the radio; the boat does its "
                                f"link_loss_action (hold) by itself"})
    elif h.get("fcu_age_s") is not None and h["fcu_age_s"] > 2.0:          # §8: FCU link lost (fcu_age_s > 2 s) - before the mm's FAULT
        rows.insert(0, {"age": h["fcu_age_s"], "style": "bad",
                        "text": f"FCU LINK LOST: no heartbeat for {h['fcu_age_s']:.0f} s - takeover on the boat (usv_ui); check the UART / the power"})
    elif h.get("rc_ok") is False:
        rows.insert(0, {"age": 0, "style": "bad" if auto else "warn",
                        "text": "RC LOST (the receiver)" + (" while AUTO" if auto else "") + ": the transmitter is the safety - "
                                "HOME or takeover; check the receiver / the transmitter's battery"})
    by = snap.get("perception_by") or {}
    stale = {k: a for k, a in by.items() if a > PERCEPTION_STALE_S and a < 600}      # a source silent for > 10 min = gone, not stale
    if not by and snap.get("perception_age") is not None and snap["perception_age"] > PERCEPTION_STALE_S:
        stale = {"camera": snap["perception_age"]}
    if st and stale:
        k, pa = max(stale.items(), key=lambda kv: kv[1])
        rows.append({"age": pa, "style": "warn", "text": f"PERCEPTION STALE: {k} silent for {pa:.0f} s - CAMERA tab -> the observer "
                                                          f"(cv_publish / buoys_multi); a recorder holding the ZED?"})


CAUTION_WORDS = ("slow loop", "overwritten", "recorder off", "stall approaching", "loop_slow", "clock", "no runstart after", "telemetry slow")   # v0.1.65: + the UAV's   # v0.1.59 (B3)   # §8: self-clearing, silent
ESCALATE_S = 300.0         # §8: a WARNING unacknowledged for 5 min becomes an ALARM


class AlarmBook:
    """v0.1.44 (OCS_V2_SPEC §8): the alarm model over the page's warnings - ALARM (red: flashes until acknowledged, stays on every page),
    WARNING (amber: acknowledged; escalates to ALARM after 5 min unacknowledged), CAUTION (self-clearing, silent, history only).
    Acknowledging never clears a condition; a condition that clears goes to the history (an in-memory 24 h ring - the hub SQLite has
    the messages when LOG=1). Keyed by the text before its first ':' (the numbers after it change)."""

    def __init__(self, keep_s: float = 86400.0, clock=time.time):
        self.clock, self.keep_s = clock, keep_s
        self.active: Dict[str, dict] = {}
        self.history: List[dict] = []
        self.RECENT_S = 900.0                          # v0.1.78.7: the ALARMS card shows what cleared in the last 15 min
        self.lock = threading.Lock()

    @staticmethod
    def key(text: str) -> str:
        return re.sub(r"[0-9.]+", "#", (text or "").split(":")[0].strip())[:80]

    @staticmethod
    def level_of(row: dict) -> str:
        t = (row.get("text") or "").lower()
        if any(w in t for w in CAUTION_WORDS):
            return "CAUTION"
        return "ALARM" if row.get("style") == "bad" else "WARNING"

    def update(self, rows: List[dict]) -> dict:
        now = self.clock()
        with self.lock:
            seen = set()
            for r in rows:
                k = self.key(r.get("text", ""))
                if not k or k in seen:
                    continue
                seen.add(k)
                lvl = self.level_of(r)
                a = self.active.get(k)
                if a is None:
                    a = self.active[k] = {"key": k, "level": lvl, "first_t": now, "acked": False, "acked_t": None, "escalated": False}
                if a["level"] != "ALARM" and lvl == "ALARM":
                    a["level"], a["acked"] = "ALARM", False                     # got worse: needs a new acknowledgement
                if a["level"] == "WARNING" and not a["acked"] and now - a["first_t"] > ESCALATE_S:
                    a["level"], a["escalated"] = "ALARM", True
                a.update(text=r.get("text", ""), last_t=now)
            for k in [k for k in self.active if k not in seen]:
                a = self.active.pop(k)
                self.history.append(dict(a, cleared_t=now))
            self.history = [h for h in self.history if now - h["cleared_t"] < self.keep_s][-500:]
            return self.body(now)

    def ack(self, key: Optional[str] = None) -> int:
        """One alarm (its key) or all of them (key None / 'all' - also the 'silence all' action). Returns how many."""
        now, n = self.clock(), 0
        with self.lock:
            for k, a in self.active.items():
                if key in (None, "", "all") or k == key:
                    if not a["acked"]:
                        a["acked"], a["acked_t"], n = True, now, n + 1
        return n

    def body(self, now: Optional[float] = None) -> dict:
        now = self.clock() if now is None else now
        order = {"ALARM": 0, "WARNING": 1, "CAUTION": 2}
        act = sorted(self.active.values(), key=lambda a: (order[a["level"]], a["acked"], -a["first_t"]))
        rows = [dict(a, age_s=round(now - a["first_t"], 1)) for a in act]
        return {"active": rows, "n_alarm": sum(1 for a in act if a["level"] == "ALARM"),
                "n_warning": sum(1 for a in act if a["level"] == "WARNING"), "n_caution": sum(1 for a in act if a["level"] == "CAUTION"),
                "unacked": sum(1 for a in act if not a["acked"] and a["level"] != "CAUTION"), "history_n": len(self.history),
                # v0.1.78.7 (Dmitrii: "after a warning is resolved, I need to see in the warnings that it is resolved"): the last 15 min
                "recent": [dict(h, dur_s=round(h["cleared_t"] - h["first_t"], 1)) for h in reversed(self.history)
                           if now - h["cleared_t"] <= self.RECENT_S][:8]}

    def history_rows(self, n: int = 200) -> List[dict]:
        with self.lock:
            return list(reversed(self.history[-n:]))


def top_bar(snap: dict) -> dict:
    """v0.1.43 (OCS_V2_SPEC §3, §7): the global bar every page shows - the state word + why (MISSION CONTROL's), who commands the
    FCU (the transmitter always beats it), V · mode · armed · kill, the R chip, the alarm / warning counts. One source: the OCS page
    draws it, judge_web fetches it from /state.json."""
    st, ocs, w = snap.get("status") or {}, snap.get("ocs") or {}, snap.get("control") or {}
    ui = bool(snap.get("ui_active"))
    holder = "usv_ui" if ui else ("mission manager" if st else "unknown")
    warns = snap.get("warnings") or []
    n_alarm = sum(1 for x in warns if x.get("style") == "bad")
    stale = snap.get("status_age") is None or snap.get("status_age") > 2.0
    sa = snap.get("status_age")
    if st and sa is not None and sa > LINK_LOST_S:                 # the bar says it first: nothing the boat says is current
        w = {"word": "LINK LOST", "why": f"the boat's status is {sa:.0f} s old (bridge / radio)",
             "hint": "check the radio; the boat holds by itself (link_loss_action)", "override": True}
    return {"word": w.get("word") or "NOT READY", "why": w.get("why") or "", "hint": w.get("hint") or "", "override": bool(w.get("override")),
            "holder": holder, "v": st.get("state") or "?", "mode": st.get("mode") or "?",
            "armed": st.get("armed"), "kill": bool(st.get("kill")), "estop": bool(st.get("estop")), "stale": stale,
            "r": ocs.get("r") or "?", "run_id": ocs.get("run_id"), "declaration_seq": ocs.get("declaration_seq"),
            "alarms": (snap.get("alarms") or {}).get("n_alarm", n_alarm), "warnings": (snap.get("alarms") or {}).get("n_warning", len(warns) - n_alarm),
            "unacked": (snap.get("alarms") or {}).get("unacked", 0), "flavour": (snap.get("flavour") or {}).get("text"),
            "theme": (snap.get("prefs") or {}).get("theme")}


def mss(s) -> str:
    s = int(round(float(s or 0)))
    return f"{s // 60}:{s % 60:02d}"


def r4_words(o: dict, st: Optional[dict] = None) -> str:
    """v0.1.59 (B3): R4 in the handbook's words - "declared seq N · heartbeat AUTO · waiting for the course manager — m:ss" (the
    elapsed counts from R4)."""
    hb = str(((st or {}).get("robot_state")) or "STATE_AUTO").replace("STATE_", "")
    return f"declared seq {o.get('declaration_seq')} · heartbeat {hb} · waiting for the course manager — {mss(o.get('waiting_s'))}"


def control_word(st: Optional[dict], ocs: Optional[dict], ui_active: bool, t_task: Optional[float] = None, now: Optional[float] = None) -> dict:
    """NOT READY (+ the one thing missing and what to do) · ARMED · RUNNING (task, stage, elapsed) · HELD (why). `judged_ready` = R5
    (a judged mission also needs RunStart: the page adds that for the picked mission); `rearm` = enable_auto would bring it back."""
    now = time.time() if now is None else now
    o = ocs or {}
    base = {"judged_ready": o.get("r") == "R5", "r": o.get("r"), "test_mode": bool(o.get("test_mode")), "rearm": False}

    def nr(why, hint, **kw):
        return dict(base, word="NOT READY", why=why, hint=hint, **kw)
    if not st:
        return nr("no boat: no vehicle status", "start the boat stack (run_usv.sh / run_sim.sh)")
    v = st.get("state")
    faults = (st.get("health") or {}).get("faults") or []
    if v == "KILLED" or st.get("kill") or st.get("estop"):
        if st.get("kill") or st.get("estop"):
            return nr("kill / E-stop asserted", "release it on the transmitter (pool / course: MANUAL by itself since v0.1.87.2; bench / lab: then confirm_safe)")
        return nr("the kill latched (KILLED)", "confirm_safe, then RE-ARM")
    if v == "FAULT":
        f = st.get("fault") or ", ".join(faults) or "?"
        if st.get("fault") and not any(x in faults for x in str(st["fault"]).split(",")):   # v0.1.52: latched, the cause gone
            return nr(f"fault {f} latched - the cause is gone", "confirm_safe, then enable_auto")
        return nr(f"fault {f}", "clear the cause, then confirm_safe")
    if v in ("AUTO_TASK",):
        el = None if t_task is None else round(now - t_task)
        return dict(base, word="RUNNING", why=f"{st.get('task_name') or '?'} · {st.get('task_state') or '?'}" + (f" · {el} s" if el is not None else ""),
                    hint="hold pauses, abort ends it, ⌂ HOME goes home", task=st.get("task_name"), stage=st.get("task_state"), elapsed_s=el)
    if v == "AUTO_SAFE_HOLD":
        return dict(base, word="HELD", why=st.get("hold_reason") or "safe hold", hint="resume goes on (or the next stage with --step); abort ends it",
                    task=st.get("task_name"), stage=st.get("task_state"))
    if v == "AUTO_HOLD" and o.get("r") == "R4":          # v0.1.59 (B3): the handbook's words - declared, AUTO, the course manager's turn
        return dict(base, word="READY", why=r4_words(o, st), hint="the course manager sends RunStart; nothing to press" if not o.get("stalled")
                    else "no RunStart after " + mss(o.get("stall_s")) + " — ask the course manager, or redeclare")
    if v == "AUTO_HOLD":
        return dict(base, word="ARMED", why="AUTO_HOLD: a mission can start", hint="pick one and START")
    if ui_active or st.get("ui_active"):
        return nr("usv_ui holds the FCU", "type handover in usv_ui")
    if not st.get("armed"):
        return nr("the FCU is not armed", "ARM in the bar (asks first) or on the transmitter")   # v0.1.50: the operator's button
    if str(st.get("mode")) == "MANUAL":
        return nr("the FCU is in MANUAL (RC6 / the mode switch)", "put it in HOLD with the transmitter")
    if "EKF_BAD" in faults:
        return nr("EKF position not ok", "wait for the GPS / EKF (lab: --lab)")
    if v == "MANUAL":
        return nr("the boat is in MANUAL", "RE-ARM (enable_auto)" + ("" if base["test_mode"] or o.get("declaration_seq") else " - declare first"),
                  rearm=True)
    return nr(f"vehicle {v or '?'}", "wait")


class RunTimeline:
    """What RoboCommand's own display shows (2026-11-05): Declared -> Held -> (Released: UAV only, Garuda) -> Started -> Ended, with
    times and reasons. A run ENDS on their side when the last declared vehicle leaves AUTO (a kill, a takeover, RC6) - our OCS stays R5,
    so this view says so in red: declare again."""

    def __init__(self):
        self.events: List[dict] = []
        self.decl = None
        self.run_id = None
        self.ended = False
        self.last_auto: Dict[str, bool] = {}

    def _add(self, kind: str, text: str, style: str = "", t: Optional[float] = None) -> None:
        self.events.append({"t": t or time.time(), "kind": kind, "text": text, "style": style})
        self.events = self.events[-40:]

    def update(self, ocs: Optional[dict], vehicles: Dict[str, dict], now: Optional[float] = None) -> None:
        o = ocs or {}
        r, seq, run = o.get("r"), o.get("declaration_seq") or 0, o.get("run_id")
        if seq and seq != self.decl and r in ("R3", "R4", "R5"):
            self.decl, self.run_id, self.ended = seq, None, False
            self._add("declared", f"Run Declaration · declaration {seq} · vehicles {', '.join(sorted(vehicles)) or '-'}", "", now)
        if r == "R4" and not any(e["kind"] == "held" and e.get("seq") == seq for e in self.events):
            self._add("held", "Run Held · every declared vehicle in AUTO, waiting for RunStart (a UAV run: Garuda clears the flight)", "warn", now)
            self.events[-1]["seq"] = seq
        if r == "R5" and run is not None and run != self.run_id:
            self.run_id, self.ended = run, False
            self._add("started", f"Run Started · run_id={run}", "ok", now)
        if r == "R5" and self.run_id is not None and not self.ended:
            auto = {v: (d or {}).get("robot_state") == "STATE_AUTO" for v, d in vehicles.items()}
            if auto and not any(auto.values()) and any(self.last_auto.values()):
                who = ", ".join(f"{v} left AUTO ({(vehicles[v] or {}).get('state') or '?'})" for v in auto)
                self.ended = True
                self._add("ended", f"Run Ended · {who} - the last vehicle out: RoboCommand ends the run here, declare again", "bad", now)
            self.last_auto = auto
        if r == "R6" and self.run_id is not None and not self.ended:
            self.ended = True
            self._add("ended", "Run Ended · end_run by the operator", "", now)

    def body(self) -> dict:
        return {"events": self.events, "ended": self.ended, "run_id": self.run_id, "declaration": self.decl}


HARD = ("hub", "boat", "judge", "course", "vehicle", "armed", "rcmode", "handover")


def _rc6(snap: dict) -> Optional[int]:
    rc = (((snap.get("local") or {}).get("fcu") or {}).get("rc") or {})
    v = rc.get("6", rc.get(6))
    return int(v) if v is not None else None


def _rc_free(snap: dict) -> bool:
    """The transmitter does not hold the boat: the FCU not in MANUAL and RC6 (when the boat broker is visible) below 1700."""
    st = snap.get("status") or {}
    rc6 = _rc6(snap)
    return st.get("mode") != "MANUAL" and not (rc6 is not None and rc6 >= 1700)     # no status at all: the boat row says it


def _rc_free_detail(snap: dict) -> str:
    rc6 = _rc6(snap)
    return f"mode {(snap.get('status') or {}).get('mode') or '-'}" + (f" · RC6 {rc6}" if rc6 is not None else "")


def clearance(snap: dict, vid: str = "USV1") -> dict:
    """The run checklist (v0.1.29): every condition the hand-over needs, green / red with how to fix it, and the next step."""
    o, st = snap.get("ocs") or {}, snap.get("status") or {}
    r, v = o.get("r"), st.get("state")
    age = snap.get("status_age")
    pa = snap.get("perception_age")
    mi = o.get("mission") or {}
    checks = [
        ("hub", "hub broker (this Mac)", bool(snap.get("hub_up")), "", "scripts/run_ocs.sh"),
        ("boat", "boat answers (status)", age is not None and age < 3, f"status {age} s" if age is not None else "never", "NX: scripts/run_usv.sh; the bridge (NETWORK tab)"),
        ("judge", "judge connected", bool(o.get("judge_connected")), str(o.get("judge") or "-"), "run_ocs.sh --judge …; rc-test: scripts/run_rctest.sh"),
        ("course", "course received + valid", bool(o.get("course_id")) or r in ("R2", "R3", "R4", "R5", "R6"), str(o.get("course_id") or "-"),
         "the judge publishes it (judge page: Course, as judge)"),
        ("vehicle", "vehicle ready (no kill / fault)", v in ("MANUAL", "AUTO_HOLD", "AUTO_TASK", "AUTO_SAFE_HOLD"), str(v or "-"),
         "KILLED / FAULT: clear the input, then confirm_safe; OFFLINE: the boat stack"),
        ("armed", "FCU armed (by you)", bool(st.get("armed")), "armed" if st.get("armed") else "disarmed", "arm from the transmitter / `arm` in usv_ui - never software"),
        ("rcmode", "FCU not in MANUAL, RC6 low", _rc_free(snap), _rc_free_detail(snap),
         "the FCU is in MANUAL (RC6 override or the transmitter's mode switch): put it in HOLD, then enable_auto (v0.1.29.3, pool F6)"),
        ("handover", "usv_ui handed over", snap.get("ui_active") is False, {True: "usv_ui ACTIVE", False: "mission manager", None: "unknown"}[snap.get("ui_active")],
         "type `handover` in usv_ui on the NX"),
        ("fcu", "FCU link", None, str(st.get("fcu_link") or "-"), "real = the Pixhawk; fake = the bench plant (sim missions only)"),
        ("perception", "perception alive", pa is not None and pa < 3, f"{pa} s" if pa is not None else "no health beat", "AGX: scripts/run_agx.sh (or --sim)"),
        ("mission", "mission preselected", bool(mi.get("code") or mi.get("task")), str(mi.get("code") or mi.get("task") or "none"), "pick it in MISSION (MAIN)"),
    ]
    out = [{"id": i, "label": l, "ok": ok, "detail": d, "fix": f, "hard": i in HARD} for i, l, ok, d, f in checks]
    out += declaration_checklist(o)
    stalled = bool(o.get("stalled"))
    nxt, why = None, ""
    if r in (None, "R0", "R1"):
        why = "waiting for the hub link and a valid course"
    elif r in ("R2", "R6") or (r == "R4" and stalled):
        nxt = "declare"
    elif v == "MANUAL" and r in ("R3", "R4", "R5"):
        nxt = "enable_auto"
    elif r == "R3":
        why = "declared: waiting for every vehicle's PREPARE ack"
    elif r == "R4":
        why = "waiting for the judge's RunStart (the RoboCommand peek shows it)"
    elif r == "R5" and v == "AUTO_HOLD":
        nxt = "start_task" if (mi.get("code") or mi.get("task")) else None
        why = "" if nxt else "R5 and AUTO_HOLD: preselect a mission, then start it"
    elif r == "R5" and v in ("AUTO_TASK", "AUTO_SAFE_HOLD"):
        why = f"running ({v})"
        if st.get("task_name") == "T4" and st.get("task_state") == "LISTEN":      # v0.1.30 (the pool: it looked like nothing ran)
            why = "m4t0 (T4) waits in LISTEN for the judge's assistance request: judge page -> Assistance request (bench: inject assistance)"
    blocked = [c["label"] for c in out if c["hard"] and c["ok"] is False]
    return {"checks": out, "next": nxt, "why": why, "blocked": blocked, "clear": not blocked}


def declaration_checklist(o: dict) -> List[dict]:
    """v0.1.41 (WP9c): the RunDeclaration rows as checklist lines - one green line when clean, else one line per row; a hard row
    under `declaration.policy: refuse` is bold (blocks `clear for run`), under warn it is a soft (amber) line."""
    d = o.get("declaration_checks")
    if not isinstance(d, dict):
        return []                                                              # an older ocs_app: no line at all
    pol = d.get("policy") or "warn"
    rows = playbook.declaration_rows(o, step="declare" if o.get("r") in ("R2", "R6", None) else "state")
    if not rows:
        tiers = d.get("tiers") or {}
        return [{"id": "declaration", "label": "declaration (tiers, mission)", "ok": True, "hard": pol == "refuse",
                 "detail": " ".join(f"{k}:{str(v).replace('TIER_', '').lower()}" for k, v in sorted(tiers.items())) + f" · policy {pol}",
                 "fix": "the RunDeclaration checks (handbook 3.4.10 / 3.4.16): every task a tier, the mission's task declared with its tier"}]
    return [{"id": "decl:" + x["code"], "label": "declaration: " + x["code"].replace("_", " "), "ok": False,
             "hard": x["severity"] == "hard" and pol == "refuse", "detail": f"{x['severity']} · policy {pol}", "fix": x["text"]} for x in rows]


def mission_declaration(o: Optional[dict]) -> Dict[str, dict]:
    """v0.1.41 (WP9c): per mission code, what the declaration checks say if it were started now (MISSION CONTROL's word)."""
    o = o or {}
    if not isinstance(o.get("declaration_checks"), dict):
        return {}
    out = {}
    for c in playbook.MISSION_ORDER:
        rows = playbook.declaration_rows(o, c, "start_task")
        if rows:
            out[c] = {"block": bool(playbook.declaration_block(o, c, "start_task")), "text": rows[0]["text"]}
    return out


# ==== v0.1.43 (WP10i-lite) backend block: the host-health rows (OCS_V2_SPEC §11) + the TIMED OUT warnings (§6) ===============
# Self-contained; Web touches it in three lines (marked "v0.1.43 hosts"): HostHealth() + attach() in __init__, v0143_state() in state().
from typing import Tuple  # noqa: E402  (kept inside the block: one place to merge)
HOST_WARN_C, HOST_ALARM_C, HOST_DISK_MIN_GB = 85.0, 95.0, 15.0   # profile `host: {warn_c, alarm_c}` overrides the first two
HOST_DISK_RED_GB = 8.0       # v0.1.86 (§6 item 1, the 11 Nov AGX disk_low): amber < 15 GB free, red < 8 GB (was one warning < 5 GB)
HOST_HIST_N = 600            # v0.1.86 (WP42 §2.5): /hosts.json keeps 10 min @ 1 Hz per host
HOST_STALE_S = 10.0          # a beat older than this raises no temperature / disk warning (its numbers are old; age_s shows it)
TIMED_OUT_WARN_S = 60.0      # a TIMED OUT command stays in the warnings list this long (ocs/state.timed_out keeps the last 10)
HOST_TOPIC_MARK = "/telemetry/"


def host_thresholds(path: Optional[str]) -> Tuple[float, float]:
    """(warn_c, alarm_c) from the OCS profile's `host:` section; the defaults without one."""
    try:
        if path and os.path.isfile(path):
            import yaml
            with open(path) as fh:
                h = (yaml.safe_load(fh) or {}).get("host") or {}
            return float(h.get("warn_c", HOST_WARN_C)), float(h.get("alarm_c", HOST_ALARM_C))
    except Exception:
        pass
    return HOST_WARN_C, HOST_ALARM_C


def load_launch(path: Optional[str]) -> dict:
    """v0.1.86: the `launch:` block the launcher writes into the OCS profile copy ({} without one)."""
    if not path:
        return {}
    try:
        import yaml
        with open(path) as fh:
            return dict((yaml.safe_load(fh) or {}).get("launch") or {})
    except Exception:
        return {}


class HostHealth:
    """The last beat of each Jetson from team/optiprime/v1/telemetry/<v>/host/<host> on the hub (the hub feed already subscribes
    to the whole namespace; attach() chains its on_message)."""

    def __init__(self, vehicle: str = "usv", warn_c: float = HOST_WARN_C, alarm_c: float = HOST_ALARM_C):
        self.vehicle, self.warn_c, self.alarm_c = vehicle, warn_c, alarm_c
        self.lock = threading.Lock()
        self.beats: Dict[str, Tuple[dict, float]] = {}
        self.since: Dict[str, float] = {}               # warning key -> first seen (the age column, as op-tui's conditions)
        self.hist: Dict[str, Dict[str, "deque"]] = {}   # v0.1.86: host -> {cpu_pct, gpu_pct, ram_pct, temp_max_c, power_w} rings (600)
        self.mac_sampler = None

    def attach(self, client) -> None:
        prev = client.on_message

        def on_message(c, u, msg):
            self.on_msg(msg.topic, msg.payload)
            if prev:
                prev(c, u, msg)
        client.on_message = on_message

    def on_msg(self, topic: str, payload: bytes, now: Optional[float] = None) -> bool:
        if HOST_TOPIC_MARK + self.vehicle + "/host/" not in topic:
            return False
        try:
            env = json.loads(payload)
        except Exception:
            return True
        body = env.get("body", env) if isinstance(env, dict) else None
        if isinstance(body, dict):
            self.put(topic.rsplit("/", 1)[-1], body, now)
        return True

    # ---- v0.1.86 (WP42 §2.5): the 10-min history + GET /hosts.json (session A's HOSTS tab; tests/fixtures/hosts_sample.json) -----
    HIST_KEYS = ("cpu_pct", "gpu_pct", "ram_pct", "temp_max_c", "power_w")

    @staticmethod
    def _point(b: dict) -> Dict[str, Optional[float]]:
        temps = [v for v in (b.get("temp_c") or {}).values() if isinstance(v, (int, float))]
        tmax = b.get("temp_max_c") if isinstance(b.get("temp_max_c"), (int, float)) else (max(temps) if temps else None)
        ru, rt = b.get("ram_used_mb"), b.get("ram_total_mb")
        pw = b.get("power_w")
        return {"cpu_pct": b.get("cpu_pct"), "gpu_pct": b.get("gpu_pct"),
                "ram_pct": round(100.0 * ru / rt, 1) if isinstance(ru, (int, float)) and isinstance(rt, (int, float)) and rt else None,
                "temp_max_c": tmax, "power_w": pw.get("total") if isinstance(pw, dict) else (pw if isinstance(pw, (int, float)) else None)}

    def put(self, host: str, body: dict, now: Optional[float] = None) -> None:
        from collections import deque
        now = time.time() if now is None else now
        pt = self._point(body)
        with self.lock:
            self.beats[host] = (body, now)
            h = self.hist.setdefault(host, {k: deque(maxlen=HOST_HIST_N) for k in self.HIST_KEYS})
            for k in self.HIST_KEYS:
                h[k].append(pt[k])

    def start_mac_sampler(self, period_s: float = 1.0) -> None:
        """The Mac's own beat in process (host `mac`): host_health.Sampler (disk, load, uptime) + psutil's cpu / ram when importable."""
        if self.mac_sampler is not None:
            return
        from . import host_health as HH
        smp = HH.Sampler(disk_path=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
        smp.tegra, smp.rec = None, None

        def run():
            while True:
                try:
                    b = smp.body("mac")
                    if b.get("cpu_pct") is None:
                        try:
                            import psutil
                            b["cpu_pct"] = psutil.cpu_percent(None)
                        except Exception:
                            pass
                    self.put("mac", b)
                except Exception:
                    pass
                time.sleep(period_s)
        self.mac_sampler = threading.Thread(target=run, daemon=True, name="mac-host-sampler")
        self.mac_sampler.start()

    # ---- v0.1.86.2 (session A: "signal quality ... as a status bar with text"): the link from the OCS to each Jetson -----------------
    LINK_N = 10

    def start_link_probe(self, targets: Dict[str, Tuple[str, int]], period_s: float = 1.0, connect=None) -> None:
        """A TCP connect per Jetson each period_s (ssh :22, 0.5 s timeout): rtt and loss over the last LINK_N samples -> link {rtt_ms,
        loss_pct} in /hosts.json (rssi_dbm / iface come from the host's own beat: /proc/net/wireless)."""
        from collections import deque
        import socket as _socket
        self.link_samples = {h: deque(maxlen=self.LINK_N) for h in targets}

        def one(addr, port):
            t0 = time.monotonic()
            try:
                with _socket.create_connection((addr, port), timeout=0.5):
                    return (time.monotonic() - t0) * 1000.0
            except OSError:
                return None
        connect = connect or one

        def run():
            while True:
                for h, (addr, port) in targets.items():
                    self.link_samples[h].append(connect(addr, port))
                time.sleep(period_s)
        if targets:
            threading.Thread(target=run, daemon=True, name="link-probe").start()

    def link_view(self, host: str, body: dict) -> Optional[dict]:
        sm = list((getattr(self, "link_samples", {}) or {}).get(host) or [])
        wf = body.get("wifi") if isinstance(body.get("wifi"), dict) else {}
        if not sm and not wf:
            return None
        ok = [x for x in sm if x is not None]
        return {"rtt_ms": round(sum(ok) / len(ok), 1) if ok else None, "loss_pct": round(100.0 * (len(sm) - len(ok)) / len(sm), 0) if sm else None,
                "samples": len(sm), "rssi_dbm": wf.get("rssi_dbm"), "iface": wf.get("iface"), "how": "tcp :22 from the OCS"}

    def hosts_body(self, now: Optional[float] = None) -> dict:
        """GET /hosts.json: {t, hosts: {nx|agx|mac: {host, age_s, power_mode, throttling, throttle_src, last {...}, hist {dt_s, ...}}}};
        nulls where unknown, hist newest-last (shorter than 600 while filling). From memory only."""
        now = time.time() if now is None else now
        out = {}
        with self.lock:
            items = [(h, b, t, {k: list(v) for k, v in self.hist.get(h, {}).items()}) for h, (b, t) in sorted(self.beats.items())]
        for h, b, t, hist in items:
            pt = self._point(b)
            pw = b.get("power_w")
            last = {"cpu_pct": b.get("cpu_pct"), "gpu_pct": b.get("gpu_pct"), "ram_used_mb": b.get("ram_used_mb"), "ram_total_mb": b.get("ram_total_mb"),
                    "temp_max_c": pt["temp_max_c"], "temp_c": b.get("temp_c") or {}, "power_w": pw if isinstance(pw, dict) else None,
                    "fan_pct": b.get("fan_pct"), "disk_free_gb": b.get("disk_free_gb"), "rec_gb_per_10min": b.get("rec_gb_per_10min")}
            for k in ("swap_used_mb", "cpu_core_pct", "emc_pct"):
                if k in b:
                    last[k] = b.get(k)
            out[h] = {"host": b.get("hostname") or h, "age_s": round(now - t, 1), "power_mode": b.get("power_mode"), "link": self.link_view(h, b),
                      "throttling": b.get("throttling"), "throttle_src": b.get("throttle_src"), "simulated": bool(b.get("simulated")),
                      "last": last, "hist": {"dt_s": 1.0, **{k: [v for v in hist.get(k, []) if v is not None] for k in self.HIST_KEYS}}}
        return {"t": round(now, 2), "hosts": out, "disk_amber_gb": HOST_DISK_MIN_GB, "disk_red_gb": HOST_DISK_RED_GB}

    def view(self, now: Optional[float] = None) -> Dict[str, dict]:
        """{host: {the beat's body..., age_s}}"""
        now = time.time() if now is None else now
        with self.lock:
            items = list(self.beats.items())
        return {h: {**b, "age_s": round(now - t, 1)} for h, (b, t) in sorted(items)}

    def warnings(self, now: Optional[float] = None) -> List[dict]:
        """[{age, text, style}] like snap['warnings']: a zone >= alarm_c (bad) / >= warn_c (warn), throttling, disk < 5 GB."""
        now = time.time() if now is None else now
        out: List[dict] = []
        for h, b in self.view(now).items():
            if b["age_s"] > HOST_STALE_S:
                continue
            temps = b.get("temp_c") or {}
            hot = [(z, c) for z, c in temps.items() if isinstance(c, (int, float))]
            if hot:
                z, c = max(hot, key=lambda zc: zc[1])
                if c >= self.alarm_c:
                    out.append({"key": f"{h}:temp", "text": f"{h.upper()} HOT {z} {c:.0f} °C (alarm {self.alarm_c:g}): shade it / stop the recorder", "style": "bad"})
                elif c >= self.warn_c:
                    out.append({"key": f"{h}:temp", "text": f"{h.upper()} hot {z} {c:.0f} °C (warn {self.warn_c:g}): shade it", "style": "warn"})
            if b.get("throttling") is True:
                out.append({"key": f"{h}:throttle", "text": f"{h.upper()} throttling ({b.get('throttle_src') or 'thermal'}): shade it", "style": "warn"})
            d = b.get("disk_free_gb")
            if isinstance(d, (int, float)) and d < HOST_DISK_MIN_GB:       # v0.1.86: amber < 15, red < 8 (+ minutes left at the measured rate)
                rr = b.get("rec_gb_per_10min")
                left = f", ~{d / rr * 10.0:.0f} min of recording at {rr:g} GB/10 min" if isinstance(rr, (int, float)) and rr > 0 else ""
                red = d < HOST_DISK_RED_GB
                out.append({"key": f"{h}:disk", "text": f"{h.upper()} disk {d:.1f} GB free (< {HOST_DISK_RED_GB if red else HOST_DISK_MIN_GB:g} GB{left}): "
                            "clear old recordings / logs", "style": "bad" if red else "warn"})
        with self.lock:
            keys = {w["key"] for w in out}
            for k in [k for k in self.since if k not in keys]:
                del self.since[k]
            for w in out:
                w["age"] = round(max(0.0, now - self.since.setdefault(w.pop("key"), now)), 1)
        return out


def timed_out_warnings(ocs: Optional[dict], now: Optional[float] = None) -> List[dict]:
    """ocs/state.timed_out -> [{age, text, style warn}] for the commands TIMED OUT in the last TIMED_OUT_WARN_S."""
    now = time.time() if now is None else now
    out: List[dict] = []
    for r in (ocs or {}).get("timed_out") or []:
        t = r.get("t_unix")
        if isinstance(t, (int, float)) and now - t < TIMED_OUT_WARN_S:
            out.append({"age": round(max(0.0, now - t), 1), "text": r.get("text") or f"command {r.get('action')} TIMED OUT", "style": "warn"})
    return out


def v0143_state(snap: dict, hosts: Optional[HostHealth], now: Optional[float] = None) -> None:
    """Adds snap['hosts'] {nx: {..., age_s}, agx: {...}} and snap['host_thresholds'], and appends the host + TIMED OUT warnings."""
    now = time.time() if now is None else now
    snap["hosts"] = hosts.view(now) if hosts else {}
    snap["host_thresholds"] = {"warn_c": hosts.warn_c if hosts else HOST_WARN_C, "alarm_c": hosts.alarm_c if hosts else HOST_ALARM_C,
                               "disk_min_gb": HOST_DISK_MIN_GB, "disk_red_gb": HOST_DISK_RED_GB, "stale_s": HOST_STALE_S}
    extra = (hosts.warnings(now) if hosts else []) + timed_out_warnings(snap.get("ocs"), now)
    snap.setdefault("warnings", []).extend(extra)
# ==== end of the v0.1.43 backend block ========================================================================================


# ==== v0.1.54 (item 11): GET /dets.json - the observer's pixel boxes for the OCS camera's overlay ===============================
# perception topics stay on the boat broker (the bridge does not carry local/#): the page's own boat-broker client (op-tui's Feeds)
# also takes local/<v>/perception/frame_dets; /dets.json serves the latest as it came, with its age. Shape:
#   {"state": "live" | "stale" | "none", "age_s", "frame_w", "frame_h", "frame_id", "source", "t_cam",
#    "dets": [{"cls", "colour", "light", "x0", "y0", "x1", "y1", "range_m", "conf"}], "why"}   (none / stale -> dets [])
DETS_STALE_S = 2.0


class DetsTap:
    def __init__(self, vehicle: str):
        self.topic = f"local/{vehicle}/perception/frame_dets"
        self.last: Optional[Tuple[float, dict]] = None

    def attach(self, client) -> None:
        """Before Feeds.start: the topic's callback + its subscription on every (re)connect (Feeds' own on_connect runs first)."""
        prev = client.on_connect

        def on_connect(c, u, f, rc, p=None):
            if prev:
                prev(c, u, f, rc, p)
            c.subscribe(self.topic, 0)
        client.on_connect = on_connect
        client.message_callback_add(self.topic, lambda c, u, msg: self.on_message(msg.payload))

    def on_message(self, payload: bytes, now: Optional[float] = None) -> None:
        try:
            env = json.loads(payload)
        except Exception:
            return
        b = env.get("body") if isinstance(env, dict) else None
        if isinstance(b, dict):
            self.last = (time.monotonic() if now is None else now, b)

    def view(self, perc: Optional[dict] = None, now: Optional[float] = None) -> dict:
        now = time.monotonic() if now is None else now
        if self.last is None:
            who = (perc or {}).get("observer_detector") or (perc or {}).get("detector")
            return {"state": "none", "dets": [], "why": "boxes not published" + (f" ({who} gives no pixel boxes)" if who else "")}
        t, b = self.last
        age = now - t
        out = {k: b.get(k) for k in ("frame_w", "frame_h", "frame_id", "source", "t_cam")}
        out["age_s"] = round(age, 2)
        if age > DETS_STALE_S:
            return dict(out, state="stale", dets=[], why=f"boxes stale {age:.0f} s (limit {DETS_STALE_S:.0f})")
        return dict(out, state="live", dets=list(b.get("dets") or []), why="")
# ==== end v0.1.54 /dets.json ======================================================================================================


# ==== v0.1.57 (WP12, D12): the cannon link on the SYSTEM card ===================================================================
# local/<v>/cannon/status (the cannon Pi; in the sim optiprimex/sim/fake_cannon_pi.py, 2 Hz) stays on the boat broker: the page's
# boat-broker client takes it. state.json "cannon":
#   {"state": "ok" | "bad" | "stale" | "none", "age_s" (of the status message), "ok", "hmac_ok", "last_t", "packet_age_s",
#    "window_id", "target_on", "n_rx", "n_bad", "status" (the sender's: OK | AIM_OFF | AIM_STALE | NO_AIM | SENDER_STOPPED), "key", "why"}
CANNON_STALE_S = 3.0


class WindowTap(DetsTap):
    """v0.1.63: the USV window's 1 Hz `local/<v>/ui/window` (boat_web: where it runs, its address, who is connected, who commands)."""
    def __init__(self, vehicle: str):
        super().__init__(vehicle)
        self.topic = f"local/{vehicle}/ui/window"

    def on_message(self, payload: bytes, now: Optional[float] = None) -> None:
        try:
            b = json.loads(payload)
        except Exception:
            return
        if isinstance(b, dict):
            self.last = (time.monotonic() if now is None else now, b)

    def view(self, perc: Optional[dict] = None, now: Optional[float] = None) -> dict:
        now = time.monotonic() if now is None else now
        if self.last is None:
            return {"state": "none", "why": "no USV window on the team LAN (launcher: USV window nx | mac)"}
        t, b = self.last
        age = now - t
        out = {k: b.get(k) for k in ("where", "url", "host", "ocs", "commander", "sessions", "pin")}
        out.update(age_s=round(age, 1), state="stale" if age > 5.0 else "up")
        return out


# ==== v0.1.65 (D17-D18 of docs/OVERNIGHT_2026-11-07_UAV.md): the UAV on the OCS page =================================================
# status/uav (the adapter, 2 Hz) and ocs/uav_window (the UAV window, 1 Hz) on the hub; state.json "uav" / "uav_window"; the alarm rows.
UAV_TEL_CAUTION_S, UAV_TEL_ALARM_S = 2.0, 5.0


class UavTap(DetsTap):
    def __init__(self, ns: str = "team/optiprime/v1", tok: str = "uav"):
        super().__init__(tok)
        self.topic = f"{ns}/status/{tok}"

    def view(self, perc: Optional[dict] = None, now: Optional[float] = None) -> dict:
        now = time.monotonic() if now is None else now
        if self.last is None:
            return {"state": "none", "why": "no UAV adapter on the hub (the launcher's UAV field, or --uav)"}
        t, b = self.last
        age = now - t
        keep = ("state", "robot_state", "current_task", "mode", "armed", "lat", "lon", "altitude_hae_m", "flight_phase", "sog_mps", "hdg_deg", "roll_deg", "pitch_deg",
                "health", "fence", "uav", "robocommand", "fcu_link", "listener", "events")
        out = {k: b.get(k) for k in keep}
        out.update(age_s=round(age, 1), link="down" if age > 3.0 or b.get("state") == "OFFLINE" else "up")
        f = dict(out.get("fence") or {})
        f.pop("points", None)                                   # the page needs the count, not the polygon
        out["fence"] = f
        return out


class UavBuoysTap(DetsTap):
    """v0.1.69 (WP25): the UAV's buoys as they reach the hub (uav/perception/buoys: the adapter's relay or the fake UAV) - raw JSON, no
    envelope - for the `dry uavmap` session's METRICS and the maps' "fit to UAV items"."""

    def __init__(self):
        super().__init__("uav")
        self.topic = "uav/perception/buoys"

    def on_message(self, payload: bytes, now: Optional[float] = None) -> None:
        try:
            b = json.loads(payload)
        except Exception:
            return
        if isinstance(b, dict) and isinstance(b.get("items"), list):
            self.last = (time.monotonic() if now is None else now, b)

    def get(self, now: Optional[float] = None) -> Tuple[Optional[dict], Optional[float]]:
        if self.last is None:
            return None, None
        t, b = self.last
        return b, round((time.monotonic() if now is None else now) - t, 1)


class SessionCmdTap(DetsTap):
    """v0.1.69: ocs/session/cmd (the OCS app hands an accepted `session` word over) -> the SessionManager, off the network thread."""

    def __init__(self, handler):
        super().__init__("x")
        self.topic, self.handler = "ocs/session/cmd", handler

    def on_message(self, payload: bytes, now: Optional[float] = None) -> None:
        try:
            b = json.loads(payload)
        except Exception:
            return
        if isinstance(b, dict):
            threading.Thread(target=self.handler, args=(b,), daemon=True).start()


class UavWindowTap(WindowTap):
    def __init__(self):
        super().__init__("uav")
        self.topic = "ocs/uav_window"

    def view(self, perc: Optional[dict] = None, now: Optional[float] = None) -> dict:
        v = super().view(perc, now)
        if v.get("state") == "none":
            v["why"] = "no UAV window (launcher UAV: on, or scripts/run_uav_window.sh)"
        return v


AUTO_CAUTION_S = 20.0


def auto_warnings(snap: dict) -> List[dict]:
    """v0.1.67 (B3, from the UAV team's checklist §9 step 6): declared (R3) for AUTO_CAUTION_S and a declared vehicle still not STATE_AUTO -
    RoboCommand starts a run only when EVERY declared vehicle reports AUTO: a CAUTION naming who (silent, self-clearing)."""
    o = snap.get("ocs") or {}
    if o.get("r") != "R3" or (o.get("declared_s") or 0.0) < AUTO_CAUTION_S:
        return []
    late = [v for v, ok in (o.get("ready") or {}).items() if not ok]
    if not late:
        return []
    return [{"age": 0.0, "text": "no runstart after %.0f s: %s not reporting STATE_AUTO - the judges start the run when every declared "
                                 "vehicle is AUTO (enable_auto; the UAV: GUIDED / AUTO)" % (float(o["declared_s"]), ", ".join(late)), "style": "warn"}]


def uav_warnings(snap: dict) -> List[dict]:
    """D18 - the UAV alarm rows (only while the UAV is declared or `declare uav` is on): UAV TELEMETRY > 2 s CAUTION (the words 'telemetry
    slow'), > 5 s ALARM 'UAV heartbeat stopped - Remote ID'; DECLARATION REFUSED (UAV); FENCE CHANGED (re-declared / after RunStart);
    FOREIGN DECLARATION (an RxRequest we did not send - their bridge is on) ALARM; RATE (rate_dropped) WARNING."""
    o = snap.get("ocs") or {}
    u = o.get("uav") or {}
    if not u.get("configured"):
        return []
    rows: List[dict] = []
    on = bool(u.get("declare") or u.get("declared"))
    st = snap.get("uav") or {}
    ta = (st.get("health") or {}).get("telemetry_age_s") if st.get("state") not in (None, "none") else None
    if on:
        if st.get("state") in (None, "none") or st.get("link") == "down":
            rows.append({"age": 0.0, "text": "UAV heartbeat stopped - Remote ID: no UAV adapter on the hub (UAV declared) - start it / check the Orin", "style": "bad"})
        elif ta is None or ta > UAV_TEL_ALARM_S:
            rows.append({"age": 0.0, "text": "UAV heartbeat stopped - Remote ID: telemetry %s - the Orin's listener / 14600-14605" % ("none" if ta is None else "%.0f s old" % ta), "style": "bad"})
        elif ta > UAV_TEL_CAUTION_S:
            rows.append({"age": 0.0, "text": "UAV telemetry slow: %.1f s old - the Orin's listener / the link (14600-14605)" % ta, "style": "warn"})
    lr = u.get("last_refusal")
    if lr and o.get("r") in ("R1", "R2", "R6"):
        rows.append({"age": 0.0, "text": "DECLARATION REFUSED (UAV): " + str(lr.get("reason") or "")[:160], "style": "bad"})
    fc = u.get("last_fence_change")
    if fc and fc.get("t"):
        try:
            import calendar
            age = time.time() - calendar.timegm(time.strptime(fc["t"][:19], "%Y-%m-%dT%H:%M:%S"))      # utc_iso: UTC
        except Exception:
            age = 0.0
        if 0 <= age < 30.0 or (fc.get("ok") is False and o.get("r") in ("R5", "R6")):
            rows.append({"age": 0.0, "text": "FENCE CHANGED (UAV): " + str(fc.get("text") or "")[:160], "style": "warn" if fc.get("ok") else "bad"})
    fd = o.get("foreign_declaration")
    if fd:
        rows.append({"age": 0.0, "text": "FOREIGN DECLARATION: an RxRequest we did not send (seq %s) - the UAV team's robocommand_bridge.py is on? stop it (one declaration owner)" % fd.get("seq"), "style": "bad"})
    if int(o.get("rate_dropped") or 0) > 0:
        rows.append({"age": 0.0, "text": "RATE: %d message(s) dropped by the 5 msg/s team cap" % int(o.get("rate_dropped") or 0), "style": "warn"})
    return rows
# ==== end v0.1.65 =================================================================================================================


class CannonTap(DetsTap):
    def __init__(self, vehicle: str):
        super().__init__(vehicle)
        self.topic = f"local/{vehicle}/cannon/status"

    def view(self, perc: Optional[dict] = None, now: Optional[float] = None) -> dict:
        now = time.monotonic() if now is None else now
        if self.last is None:
            return {"state": "none", "why": "no cannon status (local/<v>/cannon/status: the cannon Pi / fake_cannon_pi)"}
        t, b = self.last
        age = now - t
        out = {"age_s": round(age, 2), "ok": b.get("ok"), "hmac_ok": b.get("hmac_ok"), "last_t": b.get("last_t"),
               "packet_age_s": b.get("age_s"), "window_id": b.get("window_id"), "target_on": b.get("target") is not None,
               "n_rx": b.get("n_rx"), "n_bad": b.get("n_bad"), "status": b.get("status"), "key": b.get("key"),
               "event": b.get("event"), "reason": b.get("reason"), "mode": b.get("mode"), "pump": b.get("pump"), "ready": b.get("ready")}   # v0.1.61: the receiver's view
        if isinstance(b.get("target"), dict):                   # v0.1.69: the target's geometry for `session start cannon`
            out["target"] = {k: b["target"].get(k) for k in ("range_m", "bearing_deg", "elevation_deg", "color", "pan_deg", "tilt_deg")
                             if k in b["target"]}
        if age > CANNON_STALE_S:
            return dict(out, state="stale", why=f"cannon status {age:.0f} s old (limit {CANNON_STALE_S:.0f})")
        if not b.get("ok"):
            return dict(out, state="bad", why="HMAC failed on the last packet" if b.get("hmac_ok") is False else "no packet from the sender for 2 s")
        return dict(out, state="ok", why="")
# ==== end v0.1.57 cannon ==========================================================================================================


class CamTab:
    """v0.1.60 (WP10m, Dmitrii 2026-11-07): the CAMERA tab's backend - per vehicle (USV now; the UAV when WP14/15 give it a camera on
    the same contract: /snapshot.jpg, /status with modes + depth, /depth.jpg, /depth_at, POST /mode), the mode switch and the range
    under the cursor proxied (the camera is another host: the page talks to its own origin), and the frame recorder: the Mac roots
    (mac_logs, cv_staging) in this process, the AGX root over ssh (scripts/agx_frames.sh). Depth only when no detector holds the ZED
    (the camera says so; the tab shows its reason)."""

    def __init__(self, web: "Web", ssh_run=None):
        from . import frame_rec
        self.web, self.fr = web, frame_rec
        self.rec = frame_rec.Recorder(self.url, dets_fn=self._dets, pose_fn=self._pose)
        self.ssh_run = ssh_run or self._ssh_run
        self.agx: dict = {"state": "unknown"}

    def url(self, vehicle: str = "usv") -> str:
        if vehicle == "uav":
            return self.web.env.get("UAV_CAM_URL", "")
        return (self.web.cam_status() or {}).get("url") or ""

    def _dets(self):
        v = self.web.dets.view(self.web.m.perc_health)
        return {k: v.get(k) for k in ("state", "dets", "frame", "age_s", "source") if k in v}

    def _pose(self):
        st = self.web.m.status or {}
        return {k: st.get(k) for k in ("lat", "lon", "hdg_deg", "sog_mps", "state", "task_name", "task_state")}

    def view(self) -> dict:
        from . import depthviz
        cs = self.web.cam_status()
        status = cs.get("status") or {}
        sim = status.get("source") == "sim"
        uav = self.url("uav")
        roots = [{"id": r, "label": lab, "where": where,
                  "ok": where == "mac" or (not sim and bool(cs.get("ok"))),
                  "why": "" if where == "mac" or (not sim and cs.get("ok")) else ("the simulator's camera is on this Mac: record to a Mac root"
                                                                                     if sim else "the AGX camera does not answer")}
                 for r, (lab, where) in self.fr.ROOT_INFO.items()]
        sess = {}
        for r in ("mac_logs", "cv_staging"):
            try:
                sess[r] = self.fr.sessions(r, 6)
            except Exception:
                sess[r] = []
        return {"vehicles": {"usv": {"url": cs.get("url", ""), "ok": bool(cs.get("ok")), "why": cs.get("why", ""), "status": status,
                                     "sim": sim},
                             "uav": {"url": uav, "ok": False, "why": "no UAV camera yet (WP14 / WP15: the colleague's Orin, the same contract)"
                                     if not uav else "UAV camera not wired yet (WP15)"}},
                "roots": roots, "local": self.rec.status(), "agx": self.agx, "sessions": sess, "scale": depthviz.scale_bar()}

    # ---- proxied to the camera -------------------------------------------------------------------------------------------
    def set_mode(self, mode: str, vehicle: str = "usv") -> dict:
        u = self.url(vehicle)
        if not u:
            return {"ok": False, "why": "no camera address"}
        req = urllib.request.Request(u.rstrip("/") + "/mode", data=json.dumps({"mode": mode}).encode(), method="POST",
                                     headers={"Content-Type": "application/json", "X-Sim-Web": "1"})   # the simulator's post guard
        try:
            with urllib.request.urlopen(req, timeout=3.0) as r:
                return json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            try:
                return json.loads(e.read() or b"{}")
            except ValueError:
                return {"ok": False, "why": f"HTTP {e.code}"}
        except Exception as exc:
            return {"ok": False, "why": f"the camera did not answer ({exc.__class__.__name__})"}

    def depth_at(self, x: str, y: str, vehicle: str = "usv") -> dict:
        u = self.url(vehicle)
        try:
            with urllib.request.urlopen(f"{u.rstrip('/')}/depth_at?x={float(x):.4f}&y={float(y):.4f}", timeout=2.0) as r:
                return json.loads(r.read() or b"{}")
        except Exception as exc:
            return {"ok": False, "why": exc.__class__.__name__}

    # ---- the recorder ---------------------------------------------------------------------------------------------------
    def agx_refusal(self) -> str:
        """v0.1.84.6 (session A's finding): a simulator run never touches the real AGX - OCS_SIM=1 (run_sim.sh) or every camera address
        on this Mac (the simulator's / a bench camera) -> refused; AGX_SSH set to "" / none -> refused too."""
        e = self.web.env
        if str(e.get("OCS_SIM", "")).lower() in ("1", "true", "yes"):
            return "refused: a simulator run (OCS_SIM) - the real AGX is not touched"
        if str(e.get("AGX_SSH", "agxorin")).strip().lower() in ("", "none", "off"):
            return "refused: no AGX_SSH - the AGX is not touched"
        urls = self.web._meta.get("cam_urls") or ([self.web._meta["cam_url"]] if self.web._meta.get("cam_url") else [])
        if urls and all(re.search(r"//(127\.0\.0\.1|localhost)(:|/|$)", u) for u in urls):
            return "refused: the camera is on this Mac (the simulator) - the real AGX is not touched"
        return ""

    def _ssh_run(self, cmd: str, timeout_s: Optional[float] = None) -> str:
        from .system_actions import _kit, _ssh
        why = self.agx_refusal()
        if why:
            return why                                     # not JSON: _obs_json -> observer unknown + this why; _ssh_text -> this line
        argv = _ssh(self.web.env.get("AGX_SSH", "agxorin"), f"cd {_kit(self.web.env)} && {cmd}")
        try:
            r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout_s or 20)
        except subprocess.TimeoutExpired:
            return f"ssh: no answer within {timeout_s or 20:g} s"
        return (r.stdout or r.stderr or "").strip()

    def _agx(self, cmd: str) -> dict:
        try:
            out = self.ssh_run(cmd)
            line = [ln for ln in out.splitlines() if ln.strip().startswith("{")]
            self.agx = json.loads(line[-1]) if line else {"state": "unknown", "why": out[-200:] or "no answer"}
        except Exception as exc:
            self.agx = {"state": "unknown", "why": f"ssh: {exc.__class__.__name__}"}
        return self.agx

    def record(self, body: dict) -> dict:
        op, root = str(body.get("op", "")), str(body.get("root", "mac_logs"))
        if root not in self.fr.ROOT_INFO:
            return {"ok": False, "why": f"root: one of {', '.join(self.fr.ROOT_INFO)} (a typed path is refused)"}
        label = str(body.get("label") or "")
        try:
            if root == "agx_rec":
                if op == "status":
                    return {"ok": True, "agx": self._agx("scripts/agx_frames.sh status")}
                if op == "stop":
                    return {"ok": True, "agx": self._agx("scripts/agx_frames.sh stop")}
                if op == "start":
                    self.fr.check_label(label)
                    args = f" --hz {float(body.get('hz', 2)):g} --max-min {float(body.get('max_min', 10)):g}" + (" --depth" if body.get("depth") else "")
                    st = self._agx(f"scripts/agx_frames.sh start {shlex.quote(label)}{args}")
                    return {"ok": st.get("state") == "recording", "agx": st, "why": st.get("why", "")}
                return {"ok": False, "why": "snapshot: to a Mac root (the AGX records sessions)"}
            if op == "start":
                return {"ok": True, "local": self.rec.start(label, root, float(body.get("hz", 2)), float(body.get("max_min", 10)),
                                                            bool(body.get("depth")))}
            if op == "stop":
                return {"ok": True, "local": self.rec.stop("stopped (page)")}
            if op == "snapshot":
                return self.rec.snapshot(label or "snap", root)
            if op == "status":
                return {"ok": True, "local": self.rec.status()}
        except (ValueError, RuntimeError, OSError) as e:
            return {"ok": False, "why": str(e)}
        return {"ok": False, "why": "op: start | stop | snapshot | status"}

    # ---- v0.1.83.2: one RECORD for the tk console's CAMERA page (Dmitrii: "a clear button to start recording video, if it is already
    # recording - confirm"; both = the ZED video on the AGX + the 2 Hz frames on the Mac). Additive: the page's /camrec is untouched.
    def _obs_json(self, cmd: str, timeout_s: Optional[float] = None) -> dict:
        def run(c):
            if timeout_s is None:
                return self.ssh_run(c)
            try:
                return self.ssh_run(c, timeout_s=timeout_s)           # v0.1.86.5: the refresher bounds its ssh
            except TypeError:                                       # an injected runner (tests) without the argument
                return self.ssh_run(c)
        try:
            out = run(f"bash scripts/agx_observer.sh {cmd}")
        except Exception as exc:
            return {"observer": "unknown", "recording": False, "why": f"ssh: {exc.__class__.__name__}"}
        line = [ln for ln in out.splitlines() if ln.strip().startswith("{")]
        if line:
            try:
                return json.loads(line[-1])
            except ValueError:
                pass
        if "unknown option --json" in out:                  # v0.1.83.6: an AGX tree before 0.1.83.2 - ask the plain status
            try:
                txt = run("bash scripts/agx_observer.sh status")
            except Exception as exc:
                txt = f"ssh: {exc.__class__.__name__}"
            up = "observer: up" in txt
            return {"observer": "up" if up else "down" if "observer: down" in txt else "unknown", "recording": False,
                    "why": "the AGX's agx_observer.sh predates 0.1.83.2: recording not known (deploy the AGX)", "text": txt[-300:]}
        return {"observer": "unknown", "recording": False, "why": out[-200:] or "no answer", "text": out[-300:]}

    def rec_targets(self) -> dict:
        """v0.1.84.4: where a RECORD stores, known before it starts - the AGX's ZED sessions (agx_observer.sh: ~/Desktop/dm3/recordings
        /<stamp>_obs/ cam.svo2 = RGB + depth + IMU) and the Mac's frames (frame_rec mac_logs/<date>/<HHMMSS>_<label>/)."""
        return {"agx": (self.web.env.get("AGX_REC_DIR") or "~/Desktop/dm3/recordings") + "/<stamp>_obs/ (cam.svo2: RGB + depth + IMU; "
                       "+ depth: pairs/rgb_N.jpg + depth_N.png)",
                "mac": self.fr.root_path("mac_logs", self.web.env) + "/<date>/<HHMMSS>_<label>/ (frame_*.jpg + depth_*.jpg)"}

    # ---- v0.1.86.5 (WP43 Tier 1.3): GET /recording.json never runs ssh in the request thread ------------------------------------------
    REC_REFRESH_S, REC_SSH_TIMEOUT_S = 2.0, 5.0

    def recording_cached(self, now: Optional[float] = None) -> dict:
        """The last recording_status() a background refresher took (every REC_REFRESH_S, its ssh bounded to REC_SSH_TIMEOUT_S), with
        `age_s`; the Mac's own part is read now (no ssh). The first call starts the refresher; until its first answer: 'checking'."""
        import threading as _th
        now = time.time() if now is None else now
        if getattr(self, "_rec_thread", None) is None:
            self._rec_cache, self._rec_cache_t = None, None

            def run():
                while True:
                    try:
                        st = self.recording_status(ssh_timeout_s=self.REC_SSH_TIMEOUT_S)
                    except Exception as e:                          # never kill the refresher
                        st = {"ok": False, "recording": False, "agx": {"observer": "unknown", "recording": False, "why": str(e)}}
                    self._rec_cache, self._rec_cache_t = st, time.time()
                    time.sleep(self.REC_REFRESH_S)
            self._rec_thread = _th.Thread(target=run, daemon=True, name="recording-refresher")
            self._rec_thread.start()
        c, t = getattr(self, "_rec_cache", None), getattr(self, "_rec_cache_t", None)
        mac = self.rec.status()
        if c is None:
            agx = {"observer": "unknown", "recording": False, "why": "checking the AGX (first answer within a few s)"}
            return {"ok": True, "recording": mac.get("state") == "recording", "agx": agx, "mac": mac, "targets": self.rec_targets(),
                    "text": "AGX video: checking · " + ("Mac frames ●" if mac.get("state") == "recording" else "Mac frames ○"), "age_s": None}
        out = dict(c, mac=mac, age_s=round(now - t, 1))
        out["recording"] = bool((c.get("agx") or {}).get("recording")) or mac.get("state") == "recording"
        return out

    def recording_status(self, ssh_timeout_s: Optional[float] = None) -> dict:
        agx, mac = self._obs_json("status --json", timeout_s=ssh_timeout_s), self.rec.status()
        mac_on = mac.get("state") == "recording"
        return {"ok": True, "recording": bool(agx.get("recording")) or mac_on, "agx": agx, "mac": mac, "targets": self.rec_targets(),
                "text": " · ".join(x for x in (
                    (f"AGX video ● {os.path.basename(agx.get('dir') or '')} {agx.get('mb')} MB" if agx.get("recording") else
                     f"AGX video ○ ({'observer ' + str(agx.get('observer'))}{' - ' + agx['why'] if agx.get('why') else ''})"),
                    (f"Mac frames ● {mac.get('frames', 0)} · {mac.get('mb', 0)} MB" if mac_on else "Mac frames ○")) if x)}

    def recording(self, body: dict) -> dict:
        """POST /recording {op: status|start|stop, confirm?}. start while anything records -> refused with already=True unless
        confirm == "new" (stop both, start a new clip); start with the observer up but not recording -> confirm == "restart" (the observer
        restarts with --record: the ZED guard pauses ~10 s); stop needs confirm == "stop" (the observer restarts without --record)."""
        op, confirm = str(body.get("op", "status")), str(body.get("confirm") or "")
        if op == "status":
            return self.recording_status()
        st = self.recording_status()
        agx, lines = st["agx"], []
        if op == "start":
            if st["recording"] and confirm != "new":
                return {**st, "ok": False, "already": True, "why": f"already recording: {st['text']} - confirm to start a new clip"}
            if agx.get("observer") == "up" and not agx.get("recording") and confirm not in ("restart", "new"):
                return {**st, "ok": False, "needs": "restart", "why": "the observer runs without recording: restarting it with --record "
                                                                       "pauses the detections and the ZED guard ~10 s - confirm restart"}
            if confirm == "new" or agx.get("observer") == "up":
                lines.append(self._ssh_text("bash scripts/agx_observer.sh stop"))
            if self.rec.status().get("state") == "recording":
                self.rec.stop("new clip (console)")
            want_depth = bool(body.get("depth"))                  # v0.1.84.4: the console's "+ depth" (was hard-wired off)
            lines.append(self._ssh_text("bash scripts/agx_observer.sh start" + (f" --pairs-s {self.PAIRS_S:g}" if want_depth else "")))
            depth = None
            if want_depth:
                m = self.set_mode("both")                         # the stream refuses (409) while a detector holds the ZED
                if not m.get("ok", True) or m.get("error"):
                    depth = {"ok": False, "why": "no depth on the stream: " + str(m.get("why") or m.get("error") or "refused")
                             + " - the AGX's cam.svo2 holds the depth"}
            try:
                self.rec.start(self.fr.check_label(str(body.get("label") or "obs")), "mac_logs", 2.0, 0.0, want_depth)
                lines.append("Mac frames: recording (2 Hz" + (", + depth" if want_depth else "") + ", logs/recordings)")
            except (ValueError, RuntimeError, OSError) as e:
                lines.append(f"Mac frames: refused - {e}")
            if want_depth:                                        # at least one RGB + depth pair, or say why not
                depth = self._depth_check(depth)
                lines.append("depth: " + (depth["text"] if depth["ok"] else depth["why"]))
        elif op == "stop":
            if confirm != "stop":
                return {**st, "ok": False, "needs": "stop", "why": "stop recording? the observer restarts without --record (detections "
                                                                  "and the ZED guard pause ~10 s) - confirm stop"}
            if agx.get("recording") or agx.get("observer") == "up":
                lines.append(self._ssh_text("bash scripts/agx_observer.sh stop"))
                lines.append(self._ssh_text("bash scripts/agx_observer.sh start --no-record"))
            if self.rec.status().get("state") == "recording":
                self.rec.stop("stopped (console)")
                lines.append("Mac frames: stopped")
        else:
            return {"ok": False, "why": "op: status | start | stop"}
        after = self.recording_status()
        self.web.m.add("ui", f"web RECORDING {op}: {after['text']}")
        ok = after["recording"] if op == "start" else not after["recording"]
        out = {**after, "ok": ok, "lines": [x for x in lines if x], "why": "" if ok else after["text"]}
        if op == "start" and body.get("depth"):
            out["depth"] = depth                                  # {ok, why, frames}: the console's toast says it (recording goes on)
        return out

    PAIRS_S = 10.0                                      # v0.1.84.4: an RGB + depth pair every 10 s (+ one at once): ~1 GB/h beside the svo2
    DEPTH_WAIT_S = 20.0                                 # the observer loads its model (~10 s) before its first grab

    def _depth_check(self, stream_refusal: Optional[dict]) -> dict:
        """The first depth picture: the AGX observer's pairs (<session>/pairs/, the first at once) or the Mac's stream depth frames;
        waits up to DEPTH_WAIT_S. {ok, text | why, agx_pairs, mac_depth, where}."""
        t_end = time.time() + self.DEPTH_WAIT_S
        agx: dict = {}
        while True:
            agx = self._obs_json("status --json")
            n_agx = int(agx.get("pairs") or 0)
            n_mac = int(self.rec.status().get("depth_frames") or 0)
            if n_agx or n_mac or time.time() >= t_end:
                break
            time.sleep(1.0)
        where = agx.get("pairs_dir") or None
        if n_agx or n_mac:
            parts = ([f"{n_agx} RGB + depth pair(s) on the AGX → {where}"] if n_agx else []) + ([f"{n_mac} depth frame(s) on the Mac"] if n_mac else [])
            return {"ok": True, "text": " · ".join(parts), "why": "", "agx_pairs": n_agx, "mac_depth": n_mac, "where": where}
        if "pairs" not in agx:
            why = "the AGX's agx_observer.sh predates 0.1.84.4 (no pairs) - deploy the AGX; cam.svo2 holds the depth meanwhile"
        elif agx.get("observer") != "up":
            why = f"the observer is not up ({agx.get('why') or agx.get('observer')}) - no depth"
        else:
            why = f"no RGB + depth pair after {self.DEPTH_WAIT_S:g} s" + (f"; the stream: {stream_refusal['why']}" if stream_refusal else "")
        return {"ok": False, "why": why, "agx_pairs": 0, "mac_depth": 0, "where": where}

    def _ssh_text(self, cmd: str) -> str:
        try:
            return (self.ssh_run(cmd).strip().splitlines() or [""])[-1]
        except Exception as exc:
            return f"ssh: {exc.__class__.__name__}"


class Web:
    """The page's backend: op-tui's model + feeds, the command path, the SYSTEM runner."""

    def __init__(self, a: argparse.Namespace, env: Optional[Dict[str, str]] = None, feeds: bool = True):
        self.a = a
        self.env = dict(os.environ if env is None else env)
        self.m = T.Model(a.vehicle_id, a.vehicle)
        self.feeds = T.Feeds(self.m, a.host, a.port, a.boat_host, a.boat_port) if feeds else None
        self.peek = JudgePeek(a.judge_host or a.host, a.judge_port or a.port, a.team, start=feeds)
        self.clearing: Optional[threading.Thread] = None
        self.con = ocs_cli.OcsConsole(a.host, a.port, vehicle_id=a.vehicle_id, vehicle=a.vehicle, operator="web") if feeds else None
        # v0.1.86.9 (the venue analysis B1/B2): /cmd answers at the vehicle's ACCEPTED / REJECTED, the final DONE / FAILED is followed
        # in a thread and served at GET /cmd_result?id=; OCS_CMD_UNTIL=done = the old way (the reply waits for DONE).
        self.cmd_until = "done" if (self.env.get("OCS_CMD_UNTIL") or "").strip().lower() == "done" else "accepted"
        if self.con is not None:
            self.con.default_until = self.cmd_until
        self.cmd_final: Dict[str, dict] = {}            # rec id -> the final outcome (bounded)
        self.cmd_timing_path = self.env.get("OCS_CMD_TIMING") or os.path.join(ROOT, "logs", "ocs", "cmd_timing.jsonl")
        self._state_cache: Optional[tuple] = None        # v0.1.86.9 (B5): (monotonic, json bytes) - one build per STATE_TTL_S for every client
        self._state_lock = threading.Lock()
        self.view = T.View(self.m, a.vehicle_id)
        self.runner = Runner(self.env)
        self.busy = threading.Lock()
        self.results: List[dict] = []           # the page's console: what each typed / clicked line returned
        self._meta = meta(self.env, self.runner)
        self.markers = Markers(self.env.get("OCS_MARKERS") or MARKERS)
        self.timeline = RunTimeline()
        self.prefs: Dict[str, str] = {}               # v0.1.43: {theme} from the page (POST /prefs)
        self.alarms = AlarmBook()                      # v0.1.44: ALARM / WARNING / CAUTION, acknowledgement, history
        self._task_key, self._task_t0 = None, None
        self._track_lock = threading.Lock()
        self._stop_track = threading.Event()
        rec_dir = getattr(a, "map_rec", None)                    # v0.1.81.1: the USV map as it reached the Mac (usv/map_rec.py)
        self.map_rec = None
        if feeds and rec_dir and rec_dir != "off":
            from ..usv.map_rec import MapRecorder
            self.map_rec = MapRecorder(rec_dir, "ocs", a.vehicle, version=__version__)
        self._map_rec_t = None
        self.hosts = HostHealth(a.vehicle, *host_thresholds(getattr(a, "ocs_config", None)))   # v0.1.43 hosts
        self.launch = load_launch(getattr(a, "ocs_config", None))                          # v0.1.86: the preset's launch block
        if self.feeds:
            self.hosts.attach(self.feeds.hub)                                                 # v0.1.43 hosts
            self.hosts.start_mac_sampler()                                                    # v0.1.86: the Mac's row of /hosts.json
            lt = {h: (addr, 22) for h, addr in (("nx", a.boat_host), ("agx", self.env.get("AGX_HOST") or lan_agx_host()))
                  if addr and addr not in ("127.0.0.1", "localhost") and not self.env.get("OCS_SIM")}
            self.hosts.start_link_probe(lt)                                                    # v0.1.86.2: link {rtt_ms, loss_pct}
            threading.Thread(target=self._auto_log_loop, daemon=True, name="auto-log").start()   # v0.1.86: logging at DECLARE
        self.dets = DetsTap(a.vehicle)                                                        # v0.1.54 /dets.json
        self.camtab = CamTab(self)                                                            # v0.1.60 (WP10m) the CAMERA tab
        self.log_session: dict = {"running": False, "auto": False}   # v0.1.84.5: START LOGGING (POST /logging); v0.1.86: + auto
        self.boat_word: Optional[Callable[[str, List[str], float], dict]] = None   # tests swap it; None = the boat feed
        self.log_lock = threading.Lock()
        if self.feeds:
            self.dets.attach(self.feeds.boat)
        self.cannon = CannonTap(a.vehicle)                                                    # v0.1.57 the cannon link
        self.window = WindowTap(a.vehicle)                                                    # v0.1.63 the USV window on the team LAN
        self.uav = UavTap()                                                                   # v0.1.65 the UAV (status/uav on the hub)
        self.uav_window = UavWindowTap()                                                      # v0.1.65 the UAV window (ocs/uav_window)
        self.uav_buoys = UavBuoysTap()                                                        # v0.1.69 (WP25) the UAV's buoys on the hub
        from ..ocs.sessions import SessionManager                                             # v0.1.69 (WP25) the dry / wet sessions
        self.sessions = SessionManager(self._session_snapshot, self._session_publish, self.env, sim=self._session_sim,
                                       aim=self._session_aim, rec=self._session_rec, hub=f"{a.host}:{a.port}")
        self.session_cmd = SessionCmdTap(self._session_cmd)
        if self.feeds:
            self.cannon.attach(self.feeds.boat)
            self.window.attach(self.feeds.boat)
            self.uav.attach(self.feeds.hub)
            self.uav_window.attach(self.feeds.hub)
            self.uav_buoys.attach(self.feeds.hub)
            self.session_cmd.attach(self.feeds.hub)

    def _track(self, ocs: Optional[dict], st: Optional[dict]) -> None:
        """The run timeline and MISSION CONTROL's task clock, fed at 2 Hz by a thread (and by every /state.json): RoboCommand's end
        of a run happens whether or not a page is open."""
        with self._track_lock:
            self.timeline.update(ocs, (ocs or {}).get("vehicles") or {})
            key = ((st or {}).get("task_name"), (st or {}).get("state") == "AUTO_TASK")
            if key != self._task_key:
                self._task_key, self._task_t0 = key, time.time()

    def _track_loop(self) -> None:
        n = 0
        while not self._stop_track.wait(0.5):
            with self.m.lock:
                ocs, st = self.m.ocs, self.m.status
            self._track(ocs, st)
            n += 1
            if self.map_rec is not None:                       # v0.1.81.1: every new map body (the recorder keeps 1 a second)
                with self.m.lock:
                    mp, mt = self.m.map, getattr(self.m, "map_t", None)
                if mp and mt != self._map_rec_t:
                    self._map_rec_t = mt
                    self.map_rec.feed(mp)
            if n % 2 == 0:
                try:
                    self.sessions.tick()                       # v0.1.69: the session's METRICS + ocs/session at 1 Hz
                except Exception:
                    pass

    def _aim_loop(self) -> None:
        while not self._stop_track.wait(1.0 / 5.0):            # v0.1.69: `session start cannon --aim10` (5 Hz while on)
            try:
                self.sessions.aim_tick()
            except Exception:
                pass

    # ---- v0.1.69 (WP25): the sessions' plumbing ---------------------------------------------------------------------------------
    def _session_cmd(self, b: dict) -> None:
        ok, text = self.sessions.command(str(b.get("op") or ""), b.get("name"), b.get("opts") or {})
        self.m.add("ui", f"session {b.get('op')} {b.get('name') or ''}: {text}".replace("  ", " "))

    def _session_publish(self, body: dict) -> None:
        if self.feeds:
            self.feeds.hub.publish("ocs/session", json.dumps(body, default=str), qos=0, retain=True)

    def _session_sim(self) -> bool:
        with self.m.lock:
            st = self.m.status or {}
        if os.environ.get("OCS_SESSION_SIM") in ("0", "1"):
            return os.environ["OCS_SESSION_SIM"] == "1"
        try:
            from .launcher import flavour
            if (flavour(None, None, None) or {}).get("preset") == "sim":
                return True
        except Exception:
            pass
        return st.get("fcu_link") == "fake"

    def _session_aim(self, on: bool) -> None:
        if self.feeds:
            self.feeds.boat.publish(f"local/{self.a.vehicle}/cannon/aim", json.dumps(
                {"on": bool(on), "bay": 2, "window": "upper", "window_id": 1, "t": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
                 "source": "session cannon --aim10"}), qos=0)

    def _session_rec(self, op: str, label: str) -> dict:
        r = self.camtab.rec
        if op == "start":
            return r.start(label, "mac_logs", 2.0, 60.0, False)
        return r.stop("session stopped")

    def _session_snapshot(self) -> dict:
        now = time.time()
        with self.m.lock:
            d, dt = self.m.map, getattr(self.m, "map_t", None)
            st, ph, pt = self.m.status, self.m.perc_health, self.m.perc_t
        msg, age = self.uav_buoys.get()
        try:
            rec = self.camtab.rec.status()
        except Exception:
            rec = None
        return {"digest": d, "digest_age": None if not dt else round(now - dt, 1), "status": st, "cannon": self.cannon.view(),
                "uav_msg": msg, "uav_msg_age": age, "points": (d or {}).get("points") or {}, "perception": ph,
                "perception_age": None if ph is None else round(now - pt, 1), "recording": rec}

    def start(self) -> None:
        if self.feeds:
            self.feeds.start()
            self.con.start()
            threading.Thread(target=self._track_loop, name="ocs-web-track", daemon=True).start()
            threading.Thread(target=self._aim_loop, name="ocs-web-aim", daemon=True).start()

    def stop(self) -> None:
        self._stop_track.set()
        if self.map_rec is not None:
            self.map_rec.close()
        if self.feeds:
            self.feeds.stop()
            self.con.stop()
        self.peek.stop()

    # ---- the course (v0.1.33) -----------------------------------------------------------------------------------------
    def course_view(self) -> dict:
        """What the MAP draws: the profile's default course (dashed) until a course is received (solid), and the geofence."""
        v = profile_view(getattr(self.a, "ocs_config", None))
        with self.peek.m.lock:
            rc = self.peek.m.course
            rc = None if not rc else {"course_id": rc.get("course_id"), "boundary": rc.get("boundary") or [],
                                      "uav_transit_area": rc.get("uav_transit_area") or []}
        return {"default": v["default"], "received": rc, "geofence": v["geofence"], "profile": v["profile"], "scene": _scene()}

    # ---- state ------------------------------------------------------------------------------------------------------
    def sys_state(self) -> dict:
        with self.m.lock:
            return {"ocs": self.m.ocs, "status": self.m.status, "local": self.m.local}

    FAST_KEYS = ("state", "mode", "armed", "kill", "estop", "hdg_deg", "sog_mps", "roll_deg", "pitch_deg", "yaw_rate_dps", "task_name",
                 "task_state", "rc6", "lat", "lon")                  # v0.1.83.5: lat / lon (session A's tk console map, 5 Hz)

    def fast(self) -> dict:
        """v0.1.51.1: only what must move fast (the HUD, the big numerals, the bar's blocks, the map's card) - < 1 KB, polled at 5 Hz.
        From the boat's 5 Hz local/<v>/mm/state when it is fresher than the bridged 2 Hz status/<v>; everything else stays on
        /state.json (1 Hz). Never fabricated: a missing field stays missing."""
        now = time.time()
        m = self.m
        with m.lock:
            st, st_t, loc, loc_t = m.status, m.status_t, m.local, m.local_t
            bridge = m.bridge
        src, body, t = ("local", loc, loc_t) if loc is not None and (st is None or loc_t >= st_t) else ("status", st, st_t)
        v = {k: body.get(k) for k in self.FAST_KEYS if k in body} if body else {}
        if body:
            v["fcu_age_s"] = (body.get("health") or {}).get("fcu_age_s")
        return {"t": now, "src": src if body else None, "age": None if not body else round(now - t, 2),
                "status_age": None if st is None else round(now - st_t, 2), "bridge": bridge, "v": v}

    def state(self) -> dict:
        now = time.time()
        m = self.m
        ui = m.ui_state(now)
        with m.lock:
            ocs, st, loc = m.ocs, m.status, m.local
            snap = {"t": now, "hub_up": m.hub_up, "boat_up": m.boat_up, "bridge": m.bridge,
                    "ocs": ocs, "ocs_age": None if ocs is None else round(now - m.ocs_t, 1),
                    "status": st, "status_age": None if st is None else round(now - m.status_t, 1),
                    "local": loc, "local_age": None if loc is None else round(now - m.local_t, 1),
                    "stats": m.stats, "perception": m.perc_health, "perception_age": None if m.perc_health is None else round(now - m.perc_t, 1),
                    "perception_by": {k: round(now - t, 1) for k, (_b, t) in getattr(m, "perc_by", {}).items()},
                    "map": self._map_summary(now), "sys": {f"{b}:{k}": v for (b, k), v in m.sys.items()},
                    "log": [{"t": t, "kind": k, "text": x, "sev": severity(x)} for t, k, x in list(m.log)[-300:]],
                    "judge_log": [{"t": t, "text": x, "sev": severity(x)} for t, x in list(m.judge_log)[-100:]],
                    "points": self._points(),
                    "uav_gate": (m.map or {}).get("uav_gate") if isinstance(m.map, dict) else None,   # v0.1.89: the drone's touchdown sequence
                    "micro": (m.map or {}).get("micro") if isinstance(m.map, dict) else None,         # v0.1.92: the task's micro-stage {stage, sub, why, next, mods}
                    "uav_map": {k: ((m.map or {}).get("uav") or {}).get(k) for k in ("age_s", "change_age_s", "n_uav_only", "n_linked", "items",
                                                                                     "feeds", "scan", "ignored", "ingest_off")}
                    if isinstance(m.map, dict) and isinstance(m.map.get("uav"), dict) else None}
        snap["ui_active"] = ui
        try:                                                   # v0.1.49 (WP10k-b item 23): which CV model for which task (the parent draws it)
            from .cv_hint import hint as _cv_hint
            with m.lock:
                srcs = [(k, b, now - t) for k, (b, t) in getattr(m, "perc_by", {}).items()]
            snap["cv_hint"] = _cv_hint(srcs)
        except Exception:
            snap["cv_hint"] = None
        try:                                                   # v0.1.42 (WP9d): preset · judge · net · FCU · profile · CV
            from .launcher import flavour
            ui_node = next((b for n, up, age, b in m.nodes("boat", now) if n == "usv_ui" and up), None) or {}
            snap["flavour"] = flavour((ocs or {}).get("judge"), os.environ.get("NET_PROFILE") or None, ui_node.get("fcu_link"))
        except Exception:
            snap["flavour"] = None
        snap["warnings"] = [{"age": round(a, 1), "text": t, "style": s} for a, t, s in m.warnings(now)]
        snap["nodes"] = {b: [{"node": n, "up": up, "age": round(age, 1), "body": body} for n, up, age, body in m.nodes(b, now)]
                         for b in ("hub", "boat")}
        vid = self.a.vehicle_id
        snap["verdicts"] = {a: playbook.check(a, ocs, vid, status=st, ui_active=ui).__dict__ for a in playbook.PLAYBOOK}
        snap["next"] = playbook.suggest(ocs, vid, ui)
        try:
            snap["flow_stage"] = playbook.flow_stage(ocs, vid, ui)
        except Exception:
            snap["flow_stage"] = None
        veh = ((ocs or {}).get("vehicles") or {}).get(vid) or {}
        snap["stage_meaning"] = playbook.task_stage_meaning(veh.get("task_name") or (st or {}).get("task_name"),
                                                           veh.get("task_state") or (st or {}).get("task_state"))
        sstate = {"ocs": ocs, "status": st, "local": loc}
        snap["system"] = {"refused": {x["id"]: x["refused"] for x in self.runner.catalogue(sstate)}, "jobs": self.runner.job_list()}
        snap["logging"] = self.logging_state(now)             # v0.1.83.5: the tk console's REC (session A's ask)
        snap["logging"]["session"] = self.log_session          # v0.1.84.5: START LOGGING's session {running, label, t0, parts}
        snap["logging"]["session"].setdefault("auto", False) if isinstance(snap["logging"]["session"], dict) else None   # v0.1.86
        snap["recording"] = {"auto": bool((self.log_session or {}).get("auto") and (self.log_session or {}).get("running")),
                                  "record": getattr(self, "launch", {}).get("record") or "auto"}   # v0.1.86: recording.auto (A's REC)
        snap["clocks"] = self.clocks_state(now)                # v0.1.84.5: the Jetsons' clocks vs this Mac (presence beats)
        snap["results"] = self.results[-30:]
        snap["judge"] = j = self.peek.summary(now)
        if j["declaration"] is None and (ocs or {}).get("r") in ("R3", "R4", "R5"):   # the peek joined after the declaration: the OCS's view
            o = ocs or {}
            j["answer"], j["answer_style"] = ((f"run {o.get('run_id')} running (declared before this page listened: the OCS's view)", "ok") if o.get("r") == "R5"
                                              else (f"declared ({o.get('r')}, declaration {o.get('declaration_seq')}): waiting for RunStart - this page joined later", "warn"))
        self._track(ocs, st)
        key = self._task_key or (None, False)
        snap["control"] = control_word(st, ocs, bool(snap.get("ui_active")), self._task_t0 if key[1] else None)
        snap["timeline"] = self.timeline.body()
        snap["clearance"] = clearance(snap, vid)
        snap["mission_declaration"] = mission_declaration(ocs)
        snap["clearing"] = bool(self.clearing and self.clearing.is_alive())
        v0143_state(snap, getattr(self, "hosts", None), now)                                  # v0.1.43 hosts
        safety_rows(snap)                                                                     # v0.1.43: RC / link / perception
        snap["uav"] = self.uav.view() if getattr(self, "uav", None) else {"state": "none", "why": "no hub feed"}       # v0.1.65
        snap["uav_window"] = self.uav_window.view() if getattr(self, "uav_window", None) else {"state": "none", "why": "no hub feed"}
        snap["warnings"] = (snap.get("warnings") or []) + uav_warnings(snap) + auto_warnings(snap)   # v0.1.65 (D18) / v0.1.67 (B3)
        snap["alarms"] = self.alarms.update(snap.get("warnings") or [])                      # v0.1.44: the alarm model (§8)
        snap["prefs"] = dict(self.prefs)
        snap["launch"] = launch_defaults()                                                     # v0.1.47: the cards' defaults
        snap["cannon"] = self.cannon.view() if getattr(self, "cannon", None) else {"state": "none", "why": "no boat feed"}   # v0.1.57
        snap["usv_window"] = self.window.view() if getattr(self, "window", None) else {"state": "none", "why": "no boat feed"}   # v0.1.63
        snap["bar"] = top_bar(snap)
        try:                                                   # v0.1.69 (WP25): the session (its METRICS) + MISSION METRICS
            from ..ocs.sessions import mission_metrics
            snap["session"] = self.sessions.body()
            mi = ((ocs or {}).get("mission") or {})
            with m.lock:
                snap["mission_metrics"] = mission_metrics((st or {}).get("task_name") or mi.get("task"), mi.get("tier"), m.map)
        except Exception:
            snap["session"] = snap.get("session")
        return snap

    def _points(self) -> List[dict]:
        """The saved points the boat's map carries (usv_ui's or the mission manager's digest: name -> [x_n, y_e], + lat/lon)."""
        d = self.m.map or {}
        ll = d.get("points_ll") or {}
        return [{"name": n, "x_n": xy[0], "y_e": xy[1], **{k: (ll.get(n) or {}).get(k) for k in ("lat", "lon", "utc", "from")}}
                for n, xy in sorted((d.get("points") or {}).items()) if isinstance(xy, (list, tuple)) and len(xy) >= 2]

    def clear_for_run(self) -> dict:
        """v0.1.29: the hard checks green -> `ready` (declare, enable_auto, wait for RunStart) in the background."""
        c = self.state()["clearance"]
        if c["blocked"]:
            return {"ok": False, "why": "not clear: " + "; ".join(c["blocked"])}
        if self.clearing and self.clearing.is_alive():
            return {"ok": False, "why": "already clearing"}
        self.clearing = threading.Thread(target=self.command, args=("ready",), daemon=True)
        self.clearing.start()
        return {"ok": True, "why": "declare -> enable_auto -> wait for RunStart (CONSOLE shows each step)"}

    LOG_FRESH_S = 6.0                                   # 3 presence beats

    def logging_state(self, now: Optional[float] = None) -> dict:
        """v0.1.83.5 (session A's ask, docs/TK_CONSOLE.md §3): what records now, from the presence beats this backend already has -
        usv: usv_ui's run log (.tlog/.csv/.events; `log`, and since 0.1.83.5 `log_bytes` / `log_t0` - an older usv_ui leaves them None),
        usv_sqlite: the boat's SQLite logger (LOG=1 on the NX), ocs: the hub's SQLite logger (LOG=1 here; this Mac stats its file).
        on = the beat is fresh (< 6 s), up, and names a file. Never fabricated: what a beat does not carry stays None."""
        now = time.time() if now is None else now
        with self.m.lock:
            pres = dict(getattr(self.m, "presence", {}) or {})

        def one(broker: str, node: str, file_key: str, t0_key: str, bytes_key: Optional[str], local: bool) -> dict:
            b, t = pres.get((broker, node), (None, None))
            if b is None:
                return {"on": False, "file": None, "bytes": None, "t0": None, "age_s": None, "why": f"no {node} beat"}
            age = round(now - t, 1)
            f = b.get(file_key)
            n = b.get(bytes_key) if bytes_key else None
            if local and f and n is None:
                try:
                    n = os.path.getsize(f)
                except OSError:
                    n = None
            on = bool(b.get("up")) and age < self.LOG_FRESH_S and bool(f)
            why = "" if on else ("stopped" if b.get("up") is False else f"beat {age} s old" if age >= self.LOG_FRESH_S else "logging off")
            return {"on": on, "file": os.path.basename(f) if f else None, "bytes": n, "t0": b.get(t0_key), "age_s": age, "why": why}
        d = {"usv": one("boat", "usv_ui", "log", "log_t0", "log_bytes", False),
             "usv_sqlite": one("boat", "logger-boat", "file", "t0", None, False),
             "ocs": one("hub", "logger-hub", "file", "t0", None, True)}
        d["any"] = any(v["on"] for v in d.values() if isinstance(v, dict))
        return d

    # ---- v0.1.84.5: the clocks + START LOGGING (Dmitrii via session A: "is there an explicit button to start logging? ... so we engage
    # with synced clock" - ONE button: the clocks checked first, then usv_ui's FCU log + the AGX's ZED recording + the Mac's frames,
    # one label, one t0, a session file) ----------------------------------------------------------------------------------------
    CLOCK_OK_S = 0.2
    CLOCK_STALE_S = 10.0

    @staticmethod
    def host_role(host: str) -> str:
        h = host.lower()
        return "nx" if h.startswith("usv-pixhawk") else "agx" if "agx" in h else "uav" if h.startswith("uav-pixhawk") else host

    def clocks_state(self, now: Optional[float] = None) -> dict:
        """{nx|agx|<host>: {offset_s, n, ok, age_s, host}} - offset = the host's clock minus this Mac's: -(min over the last 10 presence
        beats of receipt - t_unix); the minimum strips the network latency. ok = |offset| <= 0.2 s and a beat < 10 s old."""
        import socket
        now = time.time() if now is None else now
        me = socket.gethostname().lower()
        with self.m.lock:
            samples = {h: list(d) for h, d in getattr(self.m, "clock_samples", {}).items()}
            last = dict(getattr(self.m, "clock_last", {}))
        out: Dict[str, dict] = {}
        for host, xs in samples.items():
            if not xs or host.lower() == me or host.lower().split(".")[0] == me.split(".")[0]:
                continue
            off = round(-min(xs), 3)
            age = round(now - last.get(host, 0.0), 1)
            ok = abs(off) <= self.CLOCK_OK_S and age < self.CLOCK_STALE_S
            role = self.host_role(host)
            if role in out and out[role]["age_s"] <= age:
                continue
            out[role] = {"offset_s": off, "n": len(xs), "ok": ok, "age_s": age, "host": host}
        return out

    def clock_refusal(self, clocks: dict, need=("nx", "agx")) -> str:
        bad = []
        for r in need:
            c = clocks.get(r)
            if c is None or c["age_s"] >= self.CLOCK_STALE_S:
                bad.append(f"{r.upper()} unknown (no recent beat)")
            elif not c["ok"]:
                bad.append(f"{r.upper()} {c['offset_s']:+.2f} s vs the Mac")
        return "; ".join(bad)

    def _boat_word(self, word: str, args: List[str], wait_s: float = 5.0) -> dict:
        """usv_ui's own word over the boat broker (local/<v>/ui/cmd, as the boat window) and its answer (…/ui/cmd/result by id)."""
        if self.boat_word is not None:
            return self.boat_word(word, args, wait_s)
        fd = self.feeds
        if fd is None or not getattr(self.m, "boat_up", False):
            return {"ok": False, "text": "no boat broker link from the OCS"}
        cid = f"ocs-log-{time.time():.3f}"
        fd.boat.publish(f"local/{self.m.vehicle}/ui/cmd", json.dumps({"word": word, "args": args, "id": cid, "t": time.time(), "src": "ocs-logging"}), qos=1)
        end = time.time() + wait_s
        while time.time() < end:
            with self.m.lock:
                r = self.m.ui_results.pop(cid, None)
            if r is not None:
                return {"ok": bool(r.get("ok")), "text": str(r.get("text") or "")}
            time.sleep(0.1)
        return {"ok": False, "text": f"no answer from usv_ui in {wait_s:g} s (is it running on the boat?)"}

    def logging_cmd(self, body: dict) -> dict:
        """POST /logging {op: start|stop|status, label?, depth?, confirm?} - see the block comment above. A part that already runs
        (usv_ui logs by default since 0.1.83; the observer records by default) counts as started and is left as found at stop."""
        if not self.log_lock.acquire(timeout=0.5):
            return {"ok": False, "why": "busy: a logging start / stop is still running"}
        try:
            return self._logging_cmd(body)
        finally:
            self.log_lock.release()

    def _logging_cmd(self, body: dict) -> dict:
        op, confirm = str(body.get("op", "status")), str(body.get("confirm") or "")
        clocks = self.clocks_state()
        ses = self.log_session
        if op == "status":
            return {"ok": True, "session": ses, "clocks": clocks}
        if op == "stop":
            if not ses.get("running"):
                return {"ok": False, "why": "no logging session runs", "session": ses}
            if confirm != "stop":
                return {"ok": False, "needs": "stop", "why": f"stop logging {ses.get('label')}? the FCU log / recordings this session "
                                                            "started end (what ran before it stays on)", "session": ses}
            parts, lines = dict(ses.get("parts") or {}), []
            if (parts.get("fcu") or {}).get("started"):
                r = self._boat_word("log", ["off"])
                lines.append("FCU log: " + r["text"])
                parts["fcu"] = dict(parts["fcu"], stopped=r["ok"])
            else:
                lines.append("FCU log: left on (it ran before this session)")
            if (parts.get("agx") or {}).get("started"):            # this session started the AGX recording: stop it (and the Mac's)
                r = self.camtab.recording({"op": "stop", "confirm": "stop"})
                lines += r.get("lines") or [r.get("text", "")]
            elif (parts.get("mac") or {}).get("started"):           # only the Mac frames were ours: the AGX recording stays as found
                self.camtab.rec.stop("logging session stopped")
                lines.append("Mac frames: stopped · AGX recording: left on (it ran before this session)")
            ses = dict(ses, running=False, t1=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), parts=parts, stop_lines=lines)
            self.log_session = ses
            self._write_session(ses)
            self.m.add("ui", f"web LOGGING stop {ses.get('label')}")
            return {"ok": True, "label": ses.get("label"), "session": ses, "lines": lines}
        if op != "start":
            return {"ok": False, "why": "op: start | stop | status"}
        if ses.get("running") and confirm != "new":
            return {"ok": False, "already": True, "why": f"logging {ses.get('label')} runs since {ses.get('t0')} - confirm to start a new one",
                    "session": ses}
        why = self.clock_refusal(clocks)
        if why and confirm not in ("clock", "new_clock"):
            return {"ok": False, "needs": "clock", "why": f"clocks not synced: {why} (limit {self.CLOCK_OK_S:g} s) - confirm to log anyway",
                    "clocks": clocks}
        label = "".join(ch if ch.isalnum() or ch in "-_" else "-" for ch in str(body.get("label") or "log"))[:24] or "log"
        label = f"{label}_{time.strftime('%Y%m%d_%H%M%S', time.gmtime())}"
        t0 = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        lines: List[str] = []
        parts: Dict[str, dict] = {}
        lg = self.logging_state()
        if lg["usv"]["on"]:                                        # usv_ui logs since 0.1.83 by default: keep that file
            parts["fcu"] = {"ok": True, "file": lg["usv"]["file"], "started": False, "note": "already logging (left on at stop)"}
            r = self._boat_word("note", [f"LOGGING session {label} t0 {t0}"])   # a mark in its .events: the session's start
            lines.append(f"FCU log: already on → {lg['usv']['file']} (note: {r['text'][:60]})")
        else:
            r = self._boat_word("log", ["on", label])
            parts["fcu"] = {"ok": r["ok"], "started": r["ok"], "file": None, "why": "" if r["ok"] else r["text"], "text": r["text"]}
            lines.append("FCU log: " + r["text"])
        rs = self.camtab.recording_status() if str(body.get("record") or "") != "off" else {"recording": False}   # v0.1.86: no ssh when off
        if str(body.get("record") or "") == "off":                  # v0.1.86: the preset's record: off - the FCU log only, no ZED / frames
            parts["agx"] = {"ok": True, "started": False, "note": "record off (the preset)"}
            lines.append("recording: off (the preset's record: off)")
        elif rs["recording"] and not body.get("depth"):
            parts["agx"] = {"ok": bool(rs["agx"].get("recording")), "started": False, "dir": rs["agx"].get("dir"), "note": "already recording"}
            if rs["mac"].get("state") == "recording":
                parts["mac"] = {"ok": True, "started": False, "dir": rs["mac"].get("dir"), "note": "already recording"}
            else:                                                  # the AGX records by default, the Mac frames do not: start them
                try:
                    st = self.camtab.rec.start(label, "mac_logs", 2.0, 0.0, False)
                    parts["mac"] = {"ok": st.get("state") == "recording", "started": True, "dir": st.get("dir")}
                except (ValueError, RuntimeError, OSError) as e:
                    parts["mac"] = {"ok": False, "started": False, "dir": None, "why": str(e)}
            lines.append("recording: AGX already on - " + rs["text"] + (" · Mac frames started" if parts["mac"].get("started") else ""))
        else:
            rconf = "new" if rs["recording"] else ("restart" if confirm in ("restart",) else "")
            r = self.camtab.recording({"op": "start", "depth": bool(body.get("depth")), "label": label, "confirm": rconf})   # the session's label
            if r.get("needs") == "restart" and body.get("auto"):      # v0.1.86 (§6 item 2): the automatic session never restarts the observer
                parts["agx"] = {"ok": False, "started": False, "why": "the observer runs without --record - not restarted at DECLARE (§6 item 2)"}
                try:
                    st = self.camtab.rec.start(label, "mac_logs", 2.0, 0.0, False)
                    parts["mac"] = {"ok": st.get("state") == "recording", "started": True, "dir": st.get("dir")}
                except (ValueError, RuntimeError, OSError) as e:
                    parts["mac"] = {"ok": False, "started": False, "dir": None, "why": str(e)}
                r = {"lines": ["AGX: the observer is not recording - left as it is (no restart at DECLARE); Mac frames started"]}
            elif r.get("needs") == "restart":
                return {"ok": False, "needs": "restart", "why": r["why"], "clocks": clocks, "parts": parts, "lines": lines}
            lines += r.get("lines") or []
        if "agx" not in parts:
            parts["agx"] = {"ok": bool((r.get("agx") or {}).get("recording")), "started": True, "dir": (r.get("agx") or {}).get("dir"),
                            "why": "" if (r.get("agx") or {}).get("recording") else str((r.get("agx") or {}).get("why") or "not recording")}
            parts["mac"] = {"ok": (r.get("mac") or {}).get("state") == "recording", "started": True, "dir": (r.get("mac") or {}).get("dir")}
            if r.get("depth") is not None:
                parts["depth"] = r["depth"]
        ok = all(p.get("ok") for p in parts.values())
        ses = {"running": True, "label": label, "t0": t0, "clocks": clocks, "parts": parts, "clock_override": bool(why),
               "auto": bool(body.get("auto")), "declaration_seq": body.get("declaration_seq")}   # v0.1.86: the DECLARE session (no press)
        self.log_session = ses
        self._write_session(ses)
        self.m.add("ui", f"web LOGGING start {label}: " + ", ".join(f"{k} {'ok' if v.get('ok') else 'NO'}" for k, v in parts.items()))
        return {"ok": ok, "label": label, "t0": t0, "parts": parts, "lines": lines, "clocks": clocks, "session": ses,
                "why": "" if ok else "; ".join(f"{k}: {v.get('why') or 'not started'}" for k, v in parts.items() if not v.get("ok"))}

    # ---- v0.1.86 (WP42 §2.3c, decision 5): logging + recording by themselves at DECLARE, stopped at the run's end ------------------
    AUTO_LOG_R = ("R3", "R4", "R5")

    def auto_log_tick(self, ocs: Optional[dict] = None) -> Optional[dict]:
        """One look at ocs/state: a new declaration (R3..R5 with a declaration_seq we have not logged) -> START LOGGING by itself
        (label <preset>_<mission>_<seq>; the clocks never block it; record: off = the FCU log only; never an observer restart); the run
        over (R6 / back to R0-R2) -> stop the session it started. A session the operator started is left alone. Returns the action's answer."""
        if str(self.launch.get("auto_log", "on")).lower() in ("off", "false", "0"):
            return None
        ocs = self.m.ocs if ocs is None else ocs
        if not isinstance(ocs, dict):
            return None
        r, seq = ocs.get("r"), ocs.get("declaration_seq")
        ses = self.log_session or {}
        if r in self.AUTO_LOG_R and seq is not None and seq != getattr(self, "_auto_seq", None):
            self._auto_seq = seq
            if ses.get("running"):
                self.m.add("ui", f"web LOGGING at DECLARE: {ses.get('label')} already runs since {ses.get('t0')} (kept)")
                return {"ok": True, "already": True, "session": ses}
            mission = (ocs.get("mission") or {}).get("code") or self.launch.get("mission") or "run"
            label = f"{self.launch.get('preset') or 'run'}_{mission}_{seq}"
            res = self._logging_cmd({"op": "start", "label": label, "confirm": "clock", "auto": True, "declaration_seq": seq,
                                     "record": self.launch.get("record") or "auto"})
            self.m.add("ui", f"web LOGGING at DECLARE {label}: " + ("ok" if res.get("ok") else str(res.get("why") or "partly")))
            return res
        if r not in self.AUTO_LOG_R and ses.get("running") and ses.get("auto") and getattr(self, "_auto_seq", None) is not None:
            self._auto_seq = None
            res = self._logging_cmd({"op": "stop", "confirm": "stop"})
            self.m.add("ui", f"web LOGGING stopped at the run's end ({r}): {ses.get('label')}")
            return res
        return None

    def _auto_log_loop(self) -> None:
        while True:
            try:
                self.auto_log_tick()
            except Exception as e:                                  # never kill the page server
                self.m.add("ui", f"web LOGGING auto: {e}")
            time.sleep(1.0)

    def _write_session(self, ses: dict) -> None:
        d = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "logs", "ocs", "sessions")
        if self.env.get("OCS_LOG_SESSIONS"):
            d = self.env["OCS_LOG_SESSIONS"]
        try:
            os.makedirs(d, exist_ok=True)
            with open(os.path.join(d, f"{ses['label']}.json"), "w") as fh:
                json.dump(ses, fh, indent=1, default=str)
        except OSError:
            pass

    def map_body(self) -> dict:
        """v0.1.83.5: GET /map.json - the map digest map.html draws (telemetry/<v>/map: entities, rays, gates, markers, boat, co,
        course_boundary_local, obstacles ...), as this backend last received it (usv_ui's while it publishes, else the mission
        manager's). None while nothing came."""
        now = time.time()
        with self.m.lock:
            mp, mt, src = self.m.map, self.m.map_t, getattr(self.m, "map_src", None)
        return {"t": now, "age": None if not mp else round(now - mt, 2), "source": src if mp else None, "map": mp}

    def _map_summary(self, now: float) -> dict:
        d = self.m.map or {}
        ents = d.get("entities") or d.get("items") or []
        kinds: Dict[str, int] = {}
        for e in ents if isinstance(ents, list) else []:
            k = str(e.get("kind", "?"))
            kinds[k] = kinds.get(k, 0) + 1
        return {"source": self.m.map_src, "age": None if not self.m.map else round(now - self.m.map_t, 1), "kinds": kinds,
                "gates": len(d.get("gates") or []),
                "n": sum(kinds.values()), "fcu_holder": d.get("task_state")}

    # ---- commands -----------------------------------------------------------------------------------------------------
    def command(self, line: str) -> dict:
        """One line of op-tui's grammar: an intent (through ocs_cli, waits for the acks) or a fake-plant word."""
        line = (line or "").strip()
        rec = {"t": time.time(), "line": line, "lines": [], "ok": False}
        self.results.append(rec)
        if not line:
            rec["lines"] = ["empty line"]
            return rec
        self.m.add("ui", "web> " + line)
        try:
            argv = shlex.split(line)
        except ValueError as exc:
            rec["lines"] = [f"bad line: {exc}"]
            return rec
        word = T.ALIAS.get(argv[0].lower(), argv[0].lower())
        argv[0] = word
        if word in T.PLANT_WORDS:
            rec["lines"] = [self._plant(word, argv[1:])]
            rec["ok"] = not rec["lines"][0].startswith(("usage", "bad", "plant control failed"))
            return rec
        if word not in T.INTENT_WORDS:
            rec["lines"] = [f"unknown command {word} (the page sends op-tui's words: {' '.join(sorted(T.INTENT_WORDS | T.PLANT_WORDS))})"]
            return rec
        t_recv = time.monotonic()
        rec["id"] = f"{int(rec['t'] * 1000) % 100000000:08d}{len(self.results) % 1000:03d}"
        # v0.1.86.9 (B2): a press waits its turn (the lock is held ~0.1-0.3 s per word now), never "busy" within 10 s
        if not self.busy.acquire(timeout=25.0 if word == "stop_reset" else 10.0):   # v0.1.54: STOP & RESET waits its turn
            rec["lines"] = ["busy: the previous command is still waiting for its acks (10 s)"]
            return rec
        t_lock = time.monotonic()
        if self.con is not None:
            self.con.last_out = None
        try:
            try:
                ns = ocs_cli.parse(argv)
            except SystemExit:
                u = next((x for w, x, _ in T.USAGE if w == word), None)             # v0.1.30: say how (save_point with no name)
                rec["lines"] = [f"cannot parse: {line}"] + ([f"usage: {u}"] if u else [])
                for ln in rec["lines"]:
                    self.m.add("ui", "web  " + ln)
                return rec
            why = self._names_unknown(ns)
            if why:
                rec["lines"] = [f"not sent: {why}"]
                self.m.add("ui", "web  not sent: " + why)
                return rec
            with self.m.lock:
                self.con.state, self.con.status = self.m.ocs, self.m.status
            code, lines = ocs_cli.execute(self.con, ns)
            rec["lines"] = [ln.rstrip() for ln in lines]
            rec["ok"] = code == 0
        except Exception as exc:
            rec["lines"] = [f"error: {exc}"]
        finally:
            self.busy.release()
        self._state_cache = None                                  # the next /state.json shows the press
        for ln in rec["lines"][-3:]:
            self.m.add("ui", "web  " + ln.strip())
        out = getattr(self.con, "last_out", None) if self.con is not None else None
        compound = word in ("ready", "manual", "stop_reset")
        rec["final"] = True
        timing = {"queue_ms": round((t_lock - t_recv) * 1000)}
        if out and not compound:
            timing.update(self._press_ms(out))
            if rec["ok"] and not out.get("final") and (out.get("result") or {}).get("cmd_msg_id"):
                rec["final"] = False                              # accepted: the DONE / FAILED follows (GET /cmd_result?id=)
                left = max(1.0, float(out.get("wait_s") or 3.0) - (time.monotonic() - out["t_send"]))
                threading.Thread(target=self._follow, args=(rec, out, left, t_recv, timing, word), daemon=True,
                                 name="ocs_web-follow").start()
        timing["reply_ms"] = round((time.monotonic() - t_recv) * 1000)
        rec["ms"] = timing
        if rec["final"]:
            self._final(rec, word, timing, "ok" if rec["ok"] else "refused")
        return rec

    @staticmethod
    def _press_ms(out: dict, acks: Optional[list] = None) -> dict:
        """v0.1.86.9: when each answer arrived after the publish (ms): the OCS's verdict, the vehicle's ACCEPTED, its final ack."""
        t0, r = out["t_send"], out.get("result") or {}
        d: Dict[str, Any] = {}
        if r.get("_rx"):
            d["verdict_ms"] = round((r["_rx"] - t0) * 1000)
        for a in (acks if acks is not None else out.get("acks") or []):
            res, rx = (a.get("body") or {}).get("result"), a.get("_rx")
            if not rx:
                continue
            if res in ("ACCEPTED", "REJECTED") and "accepted_ms" not in d:
                d["accepted_ms"] = round((rx - t0) * 1000)
            if res in ("DONE", "FAILED", "REJECTED"):
                d["final_ms"] = round((rx - t0) * 1000)
        return d

    def _follow(self, rec: dict, out: dict, left_s: float, t_recv: float, timing: dict, word: str) -> None:
        """v0.1.86.9: the accepted press's final ack (DONE / FAILED, or none within the old wait) -> /cmd_result + the LOG."""
        try:
            acks = self.con.follow(out["intent_id"], left_s)
        except Exception:                                         # the console gone (shutdown)
            acks = []
        fin = next((a["body"] for a in acks if a["body"]["result"] in ocs_cli.OcsConsole.FINAL), None)
        t = dict(timing, **self._press_ms(out, acks))
        if fin is None:
            line, verdict = f"  ack (no DONE / FAILED within {left_s:.0f} s - see the LOG)", "timeout"
        else:
            line = f"  ack {fin['result']:9s} state={fin.get('state')} {fin.get('reason', '')}".rstrip()
            verdict = "ok" if fin["result"] == "DONE" else "failed"
        self.m.add("ui", f"web  {word}: " + line.strip())
        self._final(dict(rec, lines=rec.get("lines", []) + [line], ok=verdict == "ok", final=True), word, t, verdict)

    def _final(self, rec: dict, word: str, timing: dict, verdict: str) -> None:
        body = {"id": rec.get("id"), "line": rec.get("line"), "ok": bool(rec.get("ok")), "final": True, "verdict": verdict,
                "lines": list(rec.get("lines") or []), "ms": timing}
        self.cmd_final[str(rec.get("id"))] = body
        if len(self.cmd_final) > 300:
            for k in list(self.cmd_final)[:-200]:
                self.cmd_final.pop(k, None)
        try:
            os.makedirs(os.path.dirname(self.cmd_timing_path), exist_ok=True)
            with open(self.cmd_timing_path, "a") as fh:
                fh.write(json.dumps({"t": round(rec.get("t") or time.time(), 3), "word": word, "line": rec.get("line"),
                                     "verdict": verdict, "until": self.cmd_until, **timing}) + "\n")
        except OSError:
            pass

    def cmd_result(self, rid: str) -> dict:
        """GET /cmd_result?id= : the final outcome of a press, or {final: false} while it is still followed."""
        return self.cmd_final.get(rid) or {"id": rid, "final": False}

    def state_json(self) -> bytes:
        """v0.1.86.9 (B5): /state.json built once per STATE_TTL_S for every client (alarms advance once per build, not per client)."""
        with self._state_lock:
            c = self._state_cache
            if c is not None and time.monotonic() - c[0] < STATE_TTL_S:
                return c[1]
            b = json.dumps(self.state(), default=str).encode()
            self._state_cache = (time.monotonic(), b)
            return b

    def _names_unknown(self, ns) -> Optional[str]:
        """v0.1.30 (pool report F2): the points a mission names, checked against the saved points the boat's map digest carries."""
        if getattr(ns, "cmd", None) not in ("mission", "start_task"):
            return None
        names = ocs_cli.split_names(getattr(ns, "names", None)) + ([ns.centre] if getattr(ns, "centre", None) else [])
        d = self.m.map or {}
        known = set((d.get("points") or {})) if "points" in d else None
        return ocs_cli.check_names(names, known)

    def _plant(self, word: str, args: List[str]) -> str:
        msg: Optional[dict] = None
        try:
            if word in ("kill", "rc6", "estop", "ekf", "armed"):
                v = T._onoff(args[0]) if args else None
                if v is None:
                    return f"usage: {word} on|off"
                key = {"kill": "kill", "rc6": "rc6", "estop": "rc5", "ekf": "ekf_ok", "armed": "armed"}[word]
                msg = {"set": {key: (1900 if v else 1000) if word in ("rc6", "estop") else v}}
            elif word == "fcu":
                if not args or args[0] != "pause":
                    return "usage: fcu pause [S]"
                msg = {"pause_stream": float(args[1]) if len(args) > 1 else 4.0}
            elif word == "teleport":
                if len(args) < 2:
                    return "usage: teleport LAT LON [HDG]"
                msg = {"teleport": {"lat": float(args[0]), "lon": float(args[1]), "hdg": float(args[2]) if len(args) > 2 else 0.0}}
        except ValueError as exc:
            return f"bad number: {exc}"
        try:
            T.fcu_control(self.a.fcu_control, msg)
            self.m.add("ui", f"web plant <- {json.dumps(msg)}")
            return f"plant <- {json.dumps(msg)} (the fake FCU on this machine, udp {self.a.fcu_control})"
        except Exception as exc:
            return f"plant control failed: {exc}"

    def cam_status(self) -> dict:
        """The first camera address that answers (the LAN one first); `url` is what the page then streams from."""
        urls = self._meta.get("cam_urls") or ([self._meta["cam_url"]] if self._meta.get("cam_url") else [])
        if not urls:
            return {"url": "", "ok": False, "why": "no camera address: AGX_HOST (network*.env) or CAM_URL"}
        tried = []
        for url in urls:
            try:
                with urllib.request.urlopen(url.rstrip("/") + "/status", timeout=1.0) as r:
                    return {"url": url, "ok": True, "status": json.loads(r.read() or b"{}"), "tried": tried}
            except Exception as exc:
                tried.append(f"{url} ({exc.__class__.__name__})")
        return {"url": urls[0], "ok": False, "why": "no answer from " + " or ".join(tried) + ": stream down or AGX unreachable"}

    def system(self, body: dict) -> dict:
        r = self.runner.run(str(body.get("action")), dict(body.get("options") or {}), str(body.get("confirm") or ""), self.sys_state())
        self.m.add("ui", f"web SYSTEM {body.get('action')} {body.get('options') or ''}: {'started' if r.get('ok') else r.get('why')}")
        return r


def make_handler(web: Web):
    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj, default=str).encode(), "application/json")

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/v1", "/v1/", "/v1/index.html"):            # v0.1.43: the classic page (same state, same commands)
                with open(PAGE_V1, "rb") as fh:
                    return self._send(200, fh.read(), "text/html; charset=utf-8")
            if path.startswith("/v1/"):                                # its relative links (map.html, ...) = the same files
                path = path[3:]
            if path in ("/", "/index.html"):
                with open(PAGE, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            elif path == "/themes.css":                               # v0.1.43: the five palettes (OCS_V2_SPEC §3)
                with open(THEMES_CSS, "rb") as fh:
                    self._send(200, fh.read(), "text/css; charset=utf-8")
            elif path == "/map.html":
                with open(T.MAP_HTML, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            elif path == "/meta.json":
                hh = getattr(web, "hosts", None)                       # v0.1.55 (D3): the boat window reads the OCS profile's host thresholds here
                self._json(dict(web._meta, ws={"host": "127.0.0.1", "port": web.a.ws_port},
                                host_thresholds={"warn_c": hh.warn_c, "alarm_c": hh.alarm_c} if hh else None))
            elif path == "/state.json":
                self._send(200, web.state_json(), "application/json")      # v0.1.86.9: cached 0.25 s, shared by every client
            elif path == "/cmd_result":                                    # v0.1.86.9: a press's final DONE / FAILED
                from urllib.parse import parse_qs, urlparse
                self._json(web.cmd_result(((parse_qs(urlparse(self.path).query).get("id") or [""])[0])))
            elif path == "/fast.json":                                 # v0.1.51.1: the 5 Hz subset
                self._json(web.fast())
            elif path == "/cam.json":
                self._json(web.cam_status())
            elif path == "/dets.json":                                 # v0.1.54: the camera overlay's boxes (DetsTap)
                self._json(web.dets.view(web.m.perc_health))
            elif path == "/camtab.json":                               # v0.1.60 (WP10m): the CAMERA tab
                self._json(web.camtab.view())
            elif path == "/hosts.json":                                # v0.1.86 (WP42 §2.5): the HOSTS tab (session A)
                self._json(web.hosts.hosts_body())
            elif path == "/map.json":                                  # v0.1.83.5: the map digest (session A's Canvas map)
                self._json(web.map_body())
            elif path == "/recording.json":                            # v0.1.83.2: the tk console's RECORD (AGX video + Mac frames)
                self._json(web.camtab.recording_cached())             # v0.1.86.5: from memory (a refresher thread does the ssh)
            elif path == "/cam_depth_at":
                from urllib.parse import parse_qs, urlparse
                q = parse_qs(urlparse(self.path).query)
                self._json(web.camtab.depth_at((q.get("x") or ["-1"])[0], (q.get("y") or ["-1"])[0], (q.get("v") or ["usv"])[0]))
            elif path == "/course.json":
                self._json(web.course_view())
            elif path == "/alarms.json":                               # v0.1.44: the active list + the history (24 h)
                self._json({"active": web.alarms.body()["active"], "history": web.alarms.history_rows()})
            elif path == "/markers.json":
                self._json({"markers": web.markers.load()})
            # ---- v0.1.47 (WP10k-a A6): GET /prefs - the shared theme every page and the launcher read (the POST below since v0.1.43) ----
            elif path == "/prefs":
                self._json({"ok": True, "prefs": dict(web.prefs)})
            # ---- end v0.1.47 A6 ----------------------------------------------------------------------------------------------------------
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            try:
                self._do_post()
            finally:
                web._state_cache = None                       # v0.1.86.9: any POST may change the state - the next GET rebuilds it

        def _do_post(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._json({"ok": False, "why": "bad json"}, 400)
            if self.path.startswith("/v1/"):                  # v0.1.43: the classic page posts to the same endpoints
                self.path = self.path[3:]
            if self.headers.get("X-OCS-Web") != "1":           # a cross-site form cannot set this header
                return self._json({"ok": False, "why": "missing X-OCS-Web header"}, 403)
            if self.path == "/cmd":
                self._json(web.command(str(body.get("line", ""))))
            elif self.path == "/system":
                self._json(web.system(body))
            elif self.path == "/ack":                              # v0.1.44: acknowledge one alarm (its key) or all ("silence all")
                return self._json({"ok": True, "acked": web.alarms.ack(body.get("key"))})
            elif self.path == "/prefs":                            # v0.1.43: the page's theme, so the judge page can follow it
                t = str(body.get("theme", ""))
                if t in ("dark", "light", "hc-dark", "hc-light", "fun"):
                    web.prefs["theme"] = t
                return self._json({"ok": True, "prefs": web.prefs})
            elif self.path == "/clear":
                self._json(web.clear_for_run())
            elif self.path == "/cam_mode":                             # v0.1.60 (WP10m)
                self._json(web.camtab.set_mode(str(body.get("mode", "")), str(body.get("vehicle", "usv"))))
            elif self.path == "/camrec":
                self._json(web.camtab.record(body))
            elif self.path == "/logging":                                # v0.1.84.5: START / STOP LOGGING
                self._json(web.logging_cmd(body))
            elif self.path == "/recording":                              # v0.1.83.2
                self._json(web.camtab.recording(body))
            elif self.path == "/markers":
                self._json(web.markers.apply(body))
            elif self.path == "/geofence":
                self._json(save_geofence(web.a.ocs_config, body.get("points")))
            elif self.path == "/sim":
                self._json(sim_post(web._meta["sim_url"], str(body.get("path", "")), body.get("body") or {}))
            else:
                self._json({"ok": False, "why": "unknown"}, 404)
    return H


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):                       # no reverse-DNS getfqdn() (slow on macOS)
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = str(self.server_address[0]), int(self.server_address[1])


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1882)
    ap.add_argument("--boat-host", default="127.0.0.1")
    ap.add_argument("--boat-port", type=int, default=1884)
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--team", default="SGIT", help="the team id on the judge wire (the RoboCommand peek)")
    ap.add_argument("--judge-host", default=None, help="the judge's broker for the RoboCommand peek (default --host: the fake judge on the hub)")
    ap.add_argument("--judge-port", type=int, default=None, help="rc-test: 1885; the course: RC_PORT (default --port)")
    ap.add_argument("--fcu-control", type=int, default=14560, help="fake FCU UDP control port (bench only)")
    ap.add_argument("--ocs-config", default=os.environ.get("OCS_CONFIG") or None,
                    help="the OCS profile: course.default (drawn before the judge's course) and the MAP tab's geofence (v0.1.33)")
    ap.add_argument("--ws-port", type=int, default=9001, help="the hub broker's websocket listener (the MAP tab)")
    ap.add_argument("--web-port", type=int, default=int(os.environ.get("OCS_WEB_PORT", "8096")))
    ap.add_argument("--bind", default="127.0.0.1", help="127.0.0.1 = this Mac only (the page can restart and reboot things)")
    ap.add_argument("--open", action="store_true", help="open the page in the browser (macOS `open`)")
    ap.add_argument("--map-rec", default=os.environ.get("OCS_MAP_REC") or os.path.join(ROOT, "logs", "ocs", "maps"),
                    help="v0.1.81.1: record the USV map as it reaches the Mac (timeline + snapshots, usv/map_rec.py); off = none")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    web = Web(a)
    web.start()
    srv = Server((a.bind, a.web_port), make_handler(web))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    url = f"http://127.0.0.1:{a.web_port}/"
    print(f"[ocs_web] v{__version__}: the OCS page on {url} (hub {a.host}:{a.port}, boat {a.boat_host}:{a.boat_port}; "
          f"SYSTEM dry run: {web.runner.dry})", flush=True)
    if a.open and sys.platform == "darwin":
        subprocess.Popen(["open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        srv.server_close()
        web.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
