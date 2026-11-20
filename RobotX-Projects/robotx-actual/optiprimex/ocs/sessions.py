"""sessions: the dry / wet SESSIONS (v0.1.69, WP25 = docs/OVERNIGHT_2026-11-09_GUI.md N1).

A session is OCS-side: it starts the right observers and feeds, shows one stage's result live (METRICS), records if asked and writes a
result file. It is NOT a mission-manager task: nothing moves, no V-state is needed, the FCU is not touched. The words:

  session start NAME [--model students|cvt-v2-l] [--tape 3,5,8] [--path broker|udp] [--aim10 --confirm aim] [--label NAME]
  session stop          session result

go through the command authority (cmd_authority SESSION: bench / lab only, refused on the course network except `collect`); the OCS app
hands an accepted one to the page server (ocs_web, which holds the hub + boat feeds and the ssh path) on `ocs/session/cmd`; the page
server runs it (SessionManager below) and publishes `ocs/session` (1 Hz, retained), which the OCS app copies into ocs/state `session`.

| name     | group | starts (real: the AGX over ssh, tmux agx window `sess`)              | simulator (the fakes stand in)                    |
|----------|-------|---------------------------------------------------------------------|---------------------------------------------------|
| buoys    | dry   | cv/buoys_multi/run.sh --headless [--model cv/models/cvt-v2-l] + cam  | the fake CV's buoys in the boat's map             |
| dockface | dry   | cv/docking/run.sh --headless + cam                                   | the fake CV's dock faces                          |
| windows  | dry   | cv/buoys_multi/run.sh --headless --model cvt-v2-l --cannon red       | the fake dock's windows                           |
| uavmap   | dry   | nothing (their detector -> broker | udp -> the adapter)              | fake_uav --start-now --offset-km 16 (far away)    |
| cannon   | dry   | the observer --cannon red + cv/cannon/run.sh                         | the fake Pi (run_sim's window cannon)             |
| collect  | wet   | scripts/agx_record.sh start LABEL                                   | (no AGX)                                          |
`collect` also records the camera the page shows (frame_rec, mac_logs) and the boat's track (1 Hz) on the Mac; its DROP POINT is the
existing word `save_point <label>_NN` (the boat's own fix). `--aim10` (cannon): the H4.3 aim loop - local/<v>/cannon/aim on for 10 s at
5 Hz on the boat broker; it needs `--confirm aim` and is refused on the course network. Nothing here arms, sets a mode or moves the boat.

When the AGX's tree (v0.1.58 until the deploy) lacks a script, the step's output says so and the session runs what it can.
Results: logs/ocs/sessions/<stamp>_<name>.md (+ .json), one line in logs/ocs/sessions/sessions.jsonl.
"""
from __future__ import annotations

import json
import math
import os
import re
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional, Tuple

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
BANDS = ((0.0, 5.0), (5.0, 10.0), (10.0, 15.0), (15.0, 25.0))       # G5's distance bands (m)
COLOURS = ("red", "green", "blue", "black", "yellow", "white")
LABEL_RE = r"[A-Za-z0-9][A-Za-z0-9_-]{0,39}"
AIM_S, AIM_HZ = 10.0, 5.0
UAV_FAR_M = 2000.0                                                    # the far-UAV caution: the UAV items' centre this far from the course


@dataclass
class Spec:
    name: str
    group: str                           # dry | wet
    title: str
    what: str
    shows: str
    opts: Dict[str, Tuple[str, ...]] = field(default_factory=dict)    # option -> allowed values ("" first = free text)
    course_ok: bool = False              # allowed on the course network (collect only)


CATALOGUE: Dict[str, Spec] = {s.name: s for s in [
    Spec("buoys", "dry", "dry buoys", "the buoy observer on the real buoys (students' model or cvt-v2-l) + the camera stream",
         "the feed with boxes, the map filling: n buoys · colours · ranges · found per band · age", {"model": ("students", "cvt-v2-l")}),
    Spec("dockface", "dry", "dry dockface", "the docking observer on the dock box face; the range against the tape distances",
         "faces · range vs the tape · normal", {"tape": ("",)}),
    Spec("windows", "dry", "dry windows", "cvt-v2-l + the cannon's aim on the face: the target windows and their light",
         "upper / lower found · lit colour · stable lock (the cannon gate's rule)", {}),
    Spec("uavmap", "dry", "dry uavmap", "the UAV's buoy path (their broker, or JSON over UDP to the adapter) into the boat's map",
         "items · scan state · age · merged (0 expected when the footage is elsewhere); fit to UAV items", {"path": ("broker", "udp")}),
    Spec("cannon", "dry", "dry cannon", "the observer --cannon red on the lit window + the cannon sender + the Pi's status",
         "target range / bearing / elevation · packets · the Pi's event · LOCK", {"aim10": ("no", "yes"), "confirm": ("",)}),
    Spec("collect", "wet", "wet collect (m0t7)", "record on the water: the ZED (agx_record), the camera frames, the track; DROP POINT "
         "= save_point <label>_NN at the boat; the boat stays in MANUAL under the transmitter; no declaration",
         "the track, the points dropped, the recording", {"label": ("",)}, course_ok=True),
]}


def parse_opts(name: str, raw: Optional[dict]) -> Tuple[Dict[str, str], str]:
    """(the options, '' or why refused)."""
    spec = CATALOGUE.get(name)
    if spec is None:
        return {}, f"unknown session {name!r} (one of {', '.join(CATALOGUE)})"
    out: Dict[str, str] = {}
    for k, v in (raw or {}).items():
        if v in (None, "", False):
            continue
        if k not in spec.opts:
            return {}, f"session {name}: no option --{k} (options: {', '.join('--' + o for o in spec.opts) or 'none'})"
        allowed = spec.opts[k]
        v = "yes" if v is True else str(v)
        if allowed and allowed[0] != "" and v not in allowed:
            return {}, f"session {name} --{k}: one of {', '.join(allowed)}"
        out[k] = v
    if name == "dockface":
        tape = out.get("tape", "3,5,8")
        try:
            vals = [float(x) for x in tape.split(",") if x.strip()]
        except ValueError:
            return {}, "session dockface --tape: metres, comma-separated (3,5,8)"
        if not vals or any(not 0.5 <= x <= 40 for x in vals):
            return {}, "session dockface --tape: 0.5 .. 40 m each"
        out["tape"] = ",".join(f"{x:g}" for x in vals)
    if name == "collect":
        lab = out.get("label", "collect")
        if not re.fullmatch(LABEL_RE, lab):
            return {}, "session collect --label: letters, digits, _ and - (e.g. day1_buoys)"
        out["label"] = lab
    if name == "cannon" and out.get("aim10") == "yes" and out.get("confirm") != "aim":
        return {}, "session cannon --aim10 turns the cannon's aim on for 10 s: add --confirm aim (bench only; the Pi in DRY_RUN unless you mean it)"
    return out, ""


# ---- the plans --------------------------------------------------------------------------------------------------------------
def _kit(env: Dict[str, str]) -> str:
    return env.get("REMOTE_KIT", "~/Desktop/dm3/MASTER_SCRIPTS")


def _ssh(host: str, cmd: str) -> List[str]:
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", host, "bash -lc " + shlex.quote(cmd)]


def _tmux_window(kit: str, win: str, cmd: str) -> str:
    """A long-running AGX program in tmux `agx`, window `win` (created / replaced): the same place the operator uses by hand."""
    run = f"cd {kit} && if [ -x {shlex.quote(cmd.split()[0])} ]; then {cmd}; else echo 'this AGX tree lacks {cmd.split()[0]} (deploy first)'; sleep 30; fi"
    return (f"cd {kit} && (tmux has-session -t agx 2>/dev/null || tmux new-session -d -s agx -n cli) && "
            f"(tmux kill-window -t agx:{win} 2>/dev/null; true) && tmux new-window -d -t agx -n {win} {shlex.quote(run)} && echo started agx:{win}")


def _tmux_stop(kit: str, win: str) -> str:
    return f"tmux send-keys -t agx:{win} C-c 2>/dev/null; sleep 2; tmux kill-window -t agx:{win} 2>/dev/null; echo stopped agx:{win}"


@dataclass
class Step:
    label: str
    argv: Optional[List[str]] = None     # None = nothing to run (a stand-in / a note)
    note: str = ""
    stop_argv: Optional[List[str]] = None
    long: bool = False                   # keeps running until the session stops (a local helper)


def plan(name: str, opts: Dict[str, str], env: Dict[str, str], sim: bool, py: str, hub: str = "127.0.0.1:1882") -> List[Step]:
    """What a session starts - real (the AGX over ssh) or in the simulator (the fakes stand in). Pure: nothing runs here."""
    agx, kit = env.get("AGX_SSH", "agxorin"), _kit(env)
    steps: List[Step] = []
    obs = {"buoys": "cv/buoys_multi/run.sh --headless" + (" --model cv/models/cvt-v2-l" if opts.get("model") == "cvt-v2-l" else ""),
           "dockface": "cv/docking/run.sh --headless",
           "windows": "cv/buoys_multi/run.sh --headless --model cv/models/cvt-v2-l --cannon red",
           "cannon": "cv/buoys_multi/run.sh --headless --model cv/models/cvt-v2-l --cannon red"}.get(name)
    if obs:
        if sim:
            steps.append(Step(f"the observer ({obs.split()[0]})", None, "simulator: the fake CV stands in (nothing started on an AGX)"))
        else:
            steps.append(Step(f"the observer on the AGX: {obs}", _ssh(agx, _tmux_window(kit, "sess", obs)),
                              stop_argv=_ssh(agx, _tmux_stop(kit, "sess"))))
            if name in ("buoys", "dockface"):
                steps.append(Step("the camera stream (the observer's previews)", _ssh(agx, f"cd {kit} && scripts/agx_cam.sh start --source dir")))
    if name == "cannon":
        if sim:
            steps.append(Step("the cannon sender + the Pi", None, "simulator: run_sim's window cannon (the students' receiver in DRY_RUN + "
                                                                 "the sender) stands in"))
        else:
            steps.append(Step("the cannon sender (cv/cannon/run.sh)", _ssh(agx, _tmux_window(kit, "sesscan", "cv/cannon/run.sh")),
                              stop_argv=_ssh(agx, _tmux_stop(kit, "sesscan"))))
        if opts.get("aim10") == "yes":
            steps.append(Step(f"the aim loop: local/<v>/cannon/aim on for {AIM_S:.0f} s at {AIM_HZ:.0f} Hz (H4.3)", None, "aim10"))
    if name == "uavmap":
        path = opts.get("path", "broker")
        if sim:
            out = ["--broker", hub] if path == "broker" else ["--udp", env.get("UAV_ADAPTER_UDP", "127.0.0.1:14605")]
            steps.append(Step(f"fake_uav far from the course ({path} path)", [py, "-m", "optiprimex.sim.fake_uav", "--hub", hub, "--start-now",
                                                                            "--offset-km", "16"] + out, long=True))
        else:
            steps.append(Step(f"the UAV's buoys ({path}): their detector publishes", None,
                              "nothing to start here: broker = their mosquitto -> the boat's bridge (UAV_BUOY_PATH direct); udp = JSON "
                              "{type: uav_buoys} to the adapter's 14605 (the relay path)"))
    if name == "collect":
        lab = opts.get("label", "collect")
        if sim:
            steps.append(Step("the ZED recorder", None, "simulator: no AGX - the page's frame recorder takes the camera"))
        else:
            steps.append(Step(f"the ZED recorder: agx_record.sh start {lab}", _ssh(agx, f"cd {kit} && scripts/agx_record.sh start {shlex.quote(lab)}"),
                              stop_argv=_ssh(agx, f"cd {kit} && scripts/agx_record.sh stop")))
        steps.append(Step("the camera frames (frame_rec, mac_logs) + the track (1 Hz, logs/ocs/sessions/)", None, "collect"))
    return steps


# ---- the metrics -------------------------------------------------------------------------------------------------------------
def _boat(digest: dict) -> Optional[Tuple[float, float]]:
    b = (digest or {}).get("boat") or {}
    return (float(b["x_n"]), float(b["y_e"])) if "x_n" in b and "y_e" in b else None


def _ents(digest: dict, kind: str) -> List[dict]:
    return [e for e in (digest or {}).get("entities") or [] if e.get("kind") == kind]


def _band(r: float) -> Optional[str]:
    for lo, hi in BANDS:
        if lo <= r < hi:
            return f"{lo:g}-{hi:g}"
    return None


def metrics(name: str, s: dict, hist: dict) -> dict:
    """The live METRICS of a session from the page server's snapshot s = {digest, digest_age, cannon, uav_msg, uav_msg_age, points,
    perception, perception_age, status}; hist keeps what must persist across ticks (first seen, locks, counts). Never invented: a value
    the feeds do not carry is None."""
    d = s.get("digest") or {}
    boat = _boat(d)
    m: Dict[str, Any] = {"map_age_s": s.get("digest_age"), "perception_age_s": s.get("perception_age")}
    if name in ("buoys", "uavmap"):
        bs = _ents(d, "buoy")
        cam = [e for e in bs if e.get("source") != "uav"]
        rng = [(e, math.hypot(e["x_n"] - boat[0], e["y_e"] - boat[1])) for e in cam] if boat else []
        bands = {f"{lo:g}-{hi:g}": 0 for lo, hi in BANDS}
        for _e, r in rng:
            b = _band(r)
            if b:
                bands[b] += 1
        cols: Dict[str, int] = {}
        for e in cam:
            cols[e.get("colour") or "unknown"] = cols.get(e.get("colour") or "unknown", 0) + 1
        if name == "buoys":
            m.update(n_buoys=len(cam), colours=cols, per_band=bands,
                     ranges_m=sorted(round(r, 1) for _e, r in rng)[:20], max_n_obs=max((e.get("n_obs") or 0 for e in cam), default=0))
            hist["max_n"] = max(hist.get("max_n", 0), len(cam))
            m["max_buoys_seen"] = hist["max_n"]
    if name == "uavmap":
        u = (d.get("uav") or {})
        msg, age = s.get("uav_msg"), s.get("uav_msg_age")
        items = (msg or {}).get("items") or []
        if msg is not None:
            hist.setdefault("first_t", time.time())
            hist["last_t"] = time.time()
            hist["n_msgs"] = hist.get("n_msgs", 0) + (1 if msg.get("frame_id") != hist.get("last_frame") else 0)
            hist["last_frame"] = msg.get("frame_id")
        lat = [it["lat"] for it in items if isinstance(it.get("lat"), (int, float))]
        lon = [it["lon"] for it in items if isinstance(it.get("lon"), (int, float))]
        centre = (sum(lat) / len(lat), sum(lon) / len(lon)) if lat else None
        m.update(items=len(items), scan=((msg or {}).get("scan") or {}).get("state"), msg_age_s=age, n_msgs=hist.get("n_msgs", 0),
                 first_age_s=None if "first_t" not in hist else round(time.time() - hist["first_t"], 1),
                 items_centre=None if centre is None else [round(centre[0], 7), round(centre[1], 7)],
                 merged=u.get("n_linked"), uav_only=u.get("n_uav_only"), assoc_rate=u.get("assoc_rate"), map_uav_age_s=u.get("age_s"))
        co = (d.get("co") or {})
        if centre and co.get("lat") is not None:
            dn = (centre[0] - co["lat"]) * 111320.0
            de = (centre[1] - co["lon"]) * 111320.0 * math.cos(math.radians(co["lat"]))
            m["uav_items_from_course_m"] = round(math.hypot(dn, de))
            if m["uav_items_from_course_m"] > UAV_FAR_M:
                m["caution"] = f"UAV layer {m['uav_items_from_course_m'] / 1000:.1f} km from the course (the footage is elsewhere: merged 0 expected)"
    if name == "dockface":
        faces = _ents(d, "dock")
        tape = [float(x) for x in (s.get("opts") or {}).get("tape", "3,5,8").split(",")]
        rows = []
        for e in faces:
            r = math.hypot(e["x_n"] - boat[0], e["y_e"] - boat[1]) if boat else None
            near = min(tape, key=lambda t: abs(t - r)) if r is not None else None
            rows.append({"id": e["id"], "colour": e.get("colour"), "range_m": None if r is None else round(r, 2),
                         "tape_m": near, "err_m": None if r is None else round(r - near, 2), "n_obs": e.get("n_obs")})
        dk = d.get("dock") or {}
        m.update(n_faces=len(faces), faces=rows[:8], normal_deg=dk.get("normal_deg") if isinstance(dk, dict) else None)
    if name in ("windows", "cannon"):
        wins = _ents(d, "window")
        lit = [w for w in wins if (w.get("light") or w.get("colour_now")) not in (None, "off", "unknown")]
        m.update(n_windows=len(wins), windows=[{"id": w["id"], "colour": w.get("colour_now") or w.get("colour"), "light": w.get("light"),
                                                "n_obs": w.get("n_obs")} for w in wins][:6])
        red = [w for w in lit if (w.get("colour_now") or w.get("colour")) == "red"]
        now = time.time()
        if red:
            hist.setdefault("red_since", now)
        else:
            hist.pop("red_since", None)
        m["red_lit"] = bool(red)
        m["red_stable_s"] = None if "red_since" not in hist else round(now - hist["red_since"], 1)
    if name == "cannon":
        c = s.get("cannon") or {}
        tgt = c.get("target") or {}
        m.update(cannon_state=c.get("state"), cannon_age_s=c.get("age_s"), pi_event=c.get("event") or c.get("status"),
                 packets=c.get("n_rx"), bad_packets=c.get("n_bad"), target_on=c.get("target_on"),
                 lock=bool(c.get("ready")) if c.get("ready") is not None else None, pump=c.get("pump"), hmac_ok=c.get("hmac_ok"),
                 target_range_m=tgt.get("range_m"), target_bearing_deg=tgt.get("bearing_deg"), target_elevation_deg=tgt.get("elevation_deg"))
        ev = m.get("pi_event")
        if ev and ev != hist.get("last_ev"):
            hist.setdefault("trace", []).append(f"{time.strftime('%H:%M:%S')} {ev}")
            hist["trace"] = hist["trace"][-40:]
            hist["last_ev"] = ev
        m["pi_trace"] = list(hist.get("trace", []))[-8:]
    if name == "collect":
        pts = s.get("points") or {}
        lab = (s.get("opts") or {}).get("label", "collect")
        mine = sorted(k for k in pts if re.fullmatch(re.escape(lab) + r"_\d{2}", k))
        m.update(points=mine, n_points=len(mine), next_point=f"{lab}_{len(mine) + 1:02d}", track_n=hist.get("track_n", 0),
                 recording=s.get("recording"))
    return m


def result_md(sess: dict) -> str:
    """The result file's markdown - G5's row format for buoys (the CV sheet reads it), one table per session otherwise."""
    m, name = sess.get("metrics") or {}, sess.get("name")
    head = (f"# session {name} · {sess.get('started')} → {sess.get('stopped')} ({sess.get('duration_s')} s) · "
            f"{'simulator' if sess.get('sim') else 'real'}\n\noptions: {json.dumps(sess.get('opts') or {})}\n\n")
    if name == "buoys":
        b = m.get("per_band") or {}
        cols = " · ".join(f"{k} {v}" for k, v in sorted((m.get("colours") or {}).items())) or "-"
        return head + ("| model | n buoys (max seen) | 0-5 m | 5-10 m | 10-15 m | 15-25 m | colours | ranges (m) |\n|---|---|---|---|---|---|---|---|\n"
                       f"| {(sess.get('opts') or {}).get('model', 'students')} | {m.get('n_buoys')} ({m.get('max_buoys_seen')}) | {b.get('0-5')} | "
                       f"{b.get('5-10')} | {b.get('10-15')} | {b.get('15-25')} | {cols} | {', '.join(str(r) for r in m.get('ranges_m') or [])} |\n")
    rows = "\n".join(f"| {k} | {json.dumps(v) if isinstance(v, (dict, list)) else v} |" for k, v in m.items())
    return head + "| metric | value |\n|---|---|\n" + rows + "\n"


def result_line(sess: dict) -> str:
    m, n = sess.get("metrics") or {}, sess.get("name")
    if n == "buoys":
        return f"buoys: {m.get('n_buoys')} now (max {m.get('max_buoys_seen')}), per band {m.get('per_band')}, colours {m.get('colours')}"
    if n == "dockface":
        return f"dockface: {m.get('n_faces')} faces, errors {[f.get('err_m') for f in m.get('faces') or []]} m vs the tape"
    if n == "windows":
        return f"windows: {m.get('n_windows')} found, red lit {m.get('red_lit')}, stable {m.get('red_stable_s')} s"
    if n == "uavmap":
        return (f"uavmap: {m.get('items')} items, {m.get('n_msgs')} messages, scan {m.get('scan')}, merged {m.get('merged')}, "
                f"{m.get('uav_items_from_course_m')} m from the course")
    if n == "cannon":
        return f"cannon: {m.get('cannon_state')} · target {m.get('target_range_m')} m · Pi {m.get('pi_event')} · lock {m.get('lock')}"
    if n == "collect":
        return f"collect: {m.get('n_points')} points ({', '.join(m.get('points') or []) or '-'}), track {m.get('track_n')} fixes"
    return json.dumps(m)[:200]


# ---- the manager (the page server runs it) ------------------------------------------------------------------------------------------
class SessionManager:
    """One session at a time. snapshot() -> the metrics' inputs (the page server's feeds); publish(body) -> ocs/session;
    aim(on) -> local/<v>/cannon/aim on the boat broker; rec(op, label) -> the page's frame recorder."""

    def __init__(self, snapshot: Callable[[], dict], publish: Callable[[dict], None], env: Optional[Dict[str, str]] = None,
                 sim: Callable[[], bool] = lambda: False, aim: Optional[Callable[[bool], None]] = None,
                 rec: Optional[Callable[[str, str], dict]] = None, dry_run: Optional[bool] = None, root: Optional[str] = None,
                 py: Optional[str] = None, hub: str = "127.0.0.1:1882"):
        self.snapshot, self.publish, self.sim, self.aim, self.rec = snapshot, publish, sim, aim, rec
        self.env = dict(os.environ if env is None else env)
        self.dry = (os.environ.get("OCS_WEB_DRYRUN") == "1") if dry_run is None else dry_run
        self.root = root or os.path.join(ROOT, "logs", "ocs", "sessions")
        vpy = os.path.join(ROOT, ".venv", "bin", "python")
        self.py = py or (vpy if os.path.exists(vpy) else "python3")
        self.hub = hub
        self.cur: Optional[dict] = None
        self.last: Optional[dict] = None
        self.lock = threading.Lock()
        self._procs: List[subprocess.Popen] = []
        self._hist: dict = {}
        self._aim_until = 0.0
        self._track_fh = None

    # ---- the words ---------------------------------------------------------------------------------------------------------
    def command(self, op: str, name: Optional[str] = None, opts: Optional[dict] = None) -> Tuple[bool, str]:
        if op == "start":
            return self.start(str(name or ""), opts or {})
        if op == "stop":
            return self.stop()
        if op == "result":
            s = self.cur or self.last
            return (True, result_line(s)) if s else (False, "no session yet")
        return False, f"session {op}: start NAME | stop | result"

    def start(self, name: str, raw: dict) -> Tuple[bool, str]:
        opts, why = parse_opts(name, raw)
        if why:
            return False, why
        with self.lock:
            if self.cur is not None:
                return False, f"session {self.cur['name']} is running: session stop first"
            sim = bool(self.sim())
            steps = plan(name, opts, self.env, sim, self.py, self.hub)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.cur = {"name": name, "group": CATALOGUE[name].group, "title": CATALOGUE[name].title, "opts": opts, "sim": sim,
                        "stamp": stamp, "t0": time.time(), "started": time.strftime("%H:%M:%S"), "state": "running",
                        "steps": [{"label": st.label, "note": st.note, "cmd": " ".join(shlex.quote(a) for a in st.argv) if st.argv else None,
                                   "rc": None, "lines": []} for st in steps], "metrics": {}, "dry_run": self.dry}
            self._steps, self._hist = steps, {}
        for i, st in enumerate(steps):
            self._run_step(i, st)
        if name == "collect":
            self._collect_start(opts.get("label", "collect"), stamp)
        if opts.get("aim10") == "yes":
            self._aim_until = time.time() + AIM_S
        self.tick()
        return True, f"session {name} started ({'simulator' if sim else 'real'}): " + "; ".join(st.label for st in steps)

    def stop(self) -> Tuple[bool, str]:
        with self.lock:
            s = self.cur
            if s is None:
                return False, "no session running"
        self._aim_until = 0.0
        if self.aim and s["name"] == "cannon":                 # the aim off once the cannon session ends (an --aim10 may still run)
            try:
                self.aim(False)
            except Exception:
                pass
        for i, st in enumerate(self._steps):
            if st.stop_argv:
                self._run(st.stop_argv, s["steps"][i])
        for p in self._procs:
            try:
                p.terminate()
                p.wait(timeout=5)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        self._procs = []
        if s["name"] == "collect":
            self._collect_stop()
        self.tick()
        with self.lock:
            s["state"], s["stopped"] = "done", time.strftime("%H:%M:%S")
            s["duration_s"] = round(time.time() - s["t0"], 1)
            s["result"] = result_line(s)
            s["files"] = self._write(s)
            self.last, self.cur = s, None
        self._publish()
        return True, f"session {s['name']} stopped: {s['result']} -> {s['files'].get('md')}"

    # ---- per tick (the page server calls it at 1 Hz) ------------------------------------------------------------------------------
    def tick(self) -> None:
        s = self.cur
        if s is not None:
            try:
                snap = dict(self.snapshot() or {}, opts=s["opts"])
            except Exception as exc:
                snap = {"error": str(exc)}
            s["metrics"] = metrics(s["name"], snap, self._hist)
            s["age_s"] = round(time.time() - s["t0"], 1)
            if s["name"] == "collect":
                self._track(snap)
                s["metrics"]["track_n"] = self._hist.get("track_n", 0)
            if self.aim and self._aim_until:
                on = time.time() < self._aim_until
                s["aim"] = {"on": on, "left_s": max(0.0, round(self._aim_until - time.time(), 1))}
                if not on:
                    self._aim_until = 0.0
        self._publish()

    def aim_tick(self) -> None:
        """5 Hz while the aim loop runs (the page server's own thread)."""
        if self.aim and self._aim_until and time.time() < self._aim_until:
            self.aim(True)

    def body(self) -> dict:
        with self.lock:
            return {"t": time.time(), "current": self.cur, "last": self.last,
                    "catalogue": [{"name": c.name, "group": c.group, "title": c.title, "what": c.what, "shows": c.shows,
                                   "opts": {k: list(v) for k, v in c.opts.items()}, "course_ok": c.course_ok} for c in CATALOGUE.values()]}

    def _publish(self) -> None:
        try:
            self.publish(self.body())
        except Exception:
            pass

    # ---- helpers ---------------------------------------------------------------------------------------------------------------
    def _run_step(self, i: int, st: Step) -> None:
        rec = self.cur["steps"][i]
        if st.argv is None:
            rec["rc"] = 0
            rec["lines"].append(st.note)
            return
        if self.dry:
            rec["rc"], rec["lines"] = 0, ["(dry run) " + rec["cmd"]]
            return
        if st.long:
            try:
                p = subprocess.Popen(st.argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True, cwd=ROOT,
                                     env=self.env)
            except OSError as exc:
                rec["rc"], rec["lines"] = -1, [f"failed: {exc}"]
                return
            self._procs.append(p)
            rec["lines"].append(f"running (pid {p.pid})")
            threading.Thread(target=self._pump, args=(p, rec), daemon=True).start()
            return
        threading.Thread(target=self._run, args=(st.argv, rec), daemon=True).start()

    def _pump(self, p: subprocess.Popen, rec: dict) -> None:
        for line in p.stdout:
            rec["lines"] = (rec["lines"] + [line.rstrip()])[-20:]
        rec["rc"] = p.wait()

    def _run(self, argv: List[str], rec: dict) -> None:
        if self.dry:
            rec["lines"] = (rec["lines"] + ["(dry run) " + " ".join(shlex.quote(a) for a in argv)])[-20:]
            return
        try:
            r = subprocess.run(argv, capture_output=True, text=True, timeout=60, env=self.env)
            rec["lines"] = (rec["lines"] + (r.stdout + r.stderr).strip().splitlines()[-10:])[-20:]
            rec["rc"] = r.returncode
        except Exception as exc:
            rec["lines"] = (rec["lines"] + [f"failed: {exc}"])[-20:]
            rec["rc"] = -1

    def _collect_start(self, label: str, stamp: str) -> None:
        os.makedirs(self.root, exist_ok=True)
        path = os.path.join(self.root, f"{stamp}_collect_{label}_track.csv")
        self._track_fh = open(path, "w", buffering=1)
        self._track_fh.write("utc,lat,lon,hdg_deg,sog_mps,state,mode,armed\n")
        self.cur["track_file"] = path
        if self.rec:
            try:
                self.cur["frames"] = self.rec("start", label)
            except Exception as exc:
                self.cur["frames"] = {"ok": False, "why": str(exc)}

    def _collect_stop(self) -> None:
        if self._track_fh:
            self._track_fh.close()
            self._track_fh = None
        if self.rec:
            try:
                self.cur["frames_stop"] = self.rec("stop", "")
            except Exception as exc:
                self.cur["frames_stop"] = {"ok": False, "why": str(exc)}

    def _track(self, snap: dict) -> None:
        st = snap.get("status") or {}
        if not self._track_fh or st.get("lat") is None:
            return
        self._track_fh.write(f"{time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())},{st.get('lat')},{st.get('lon')},{st.get('hdg_deg')},"
                             f"{st.get('sog_mps')},{st.get('state')},{st.get('mode')},{st.get('armed')}\n")
        self._hist["track_n"] = self._hist.get("track_n", 0) + 1

    def _write(self, s: dict) -> dict:
        os.makedirs(self.root, exist_ok=True)
        base = os.path.join(self.root, f"{s['stamp']}_{s['name']}")
        files = {"md": base + ".md", "json": base + ".json"}
        with open(files["md"], "w") as fh:
            fh.write(result_md(s))
        with open(files["json"], "w") as fh:
            json.dump(s, fh, indent=1, default=str)
        with open(os.path.join(self.root, "sessions.jsonl"), "a") as fh:
            fh.write(json.dumps({"stamp": s["stamp"], "name": s["name"], "sim": s["sim"], "opts": s["opts"], "result": s.get("result"),
                                 "duration_s": s.get("duration_s"), "md": files["md"]}) + "\n")
        if s.get("track_file"):
            files["track"] = s["track_file"]
        return files


def mission_metrics(task: Optional[str], tier: Optional[str], digest: Optional[dict]) -> dict:
    """v0.1.69 (N1): MISSION METRICS for the running / preselected mission from the boat's map digest - m1t1: entry · gates detected /
    passed · exit; m3t1: faces · windows lit; m1t2: + the UAV's scan · items · merged. Never invented: what the digest lacks is None."""
    d = digest or {}
    mk = d.get("markers") or {}
    gates = d.get("gates") or []
    out: Dict[str, Any] = {"task": task}
    if task in (None, "T1"):
        out.update(entry_found=mk.get("entry") is not None if "markers" in d else None, gates_detected=len(gates),
                   gates_passed=sum(1 for g in gates if g.get("passed")), exit_found=mk.get("exit") is not None if "markers" in d else None,
                   buoys=len(_ents(d, "buoy")))
        if tier == "advanced" or (d.get("uav") or {}).get("colour_source") == "uav":
            u = d.get("uav") or {}
            out.update(uav_scan=(u.get("scan") or {}).get("state"), uav_items=(u.get("stats") or {}).get("published"),
                       uav_merged_pct=None if u.get("assoc_rate") is None else round(100.0 * u["assoc_rate"]))
    if task in (None, "T3"):
        wins = _ents(d, "window")
        out.update(faces=len(_ents(d, "dock")), windows=len(wins),
                   windows_lit=sum(1 for w in wins if (w.get("light") or w.get("colour_now")) not in (None, "off", "unknown")))
    return out
