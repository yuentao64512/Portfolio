"""rc-tui: the judge's console (ROADMAP WP4, WP7). What RoboCommand receives from our OCS (left) and what it sends (right),
decoded from BOTH wires at once (v0.1.26): the bench JSON on `rc/#` and RoboNation's protobuf on `robocommand/robotx/#`.

  rc-tui [--host 127.0.0.1 --port 1882] [--team SGIT] [--as-judge [--wire protobuf|json] | --monitor]
  normally from `scripts/run_judge.sh` (tmux `judge`, opened by run_ocs.sh / run_rctest.sh): tmux attach -t judge

Left  JUDGE RECEIVES: the RunDeclaration (vehicles, tiers, geofence), per vehicle the whole Heartbeat the schema carries (state,
      position, speed, heading, roll, pitch, altitude, depth, current task, type, flight phase, UAV registration; seq, rate, age),
      the vehicle's task timeline (each change of current_task = an attempt begins or stands down), the latest report of each
      kind with its content (SafePassage entry / exit / buoys by beacon, Docking bay, Firefighting window, IncidentAck,
      Readiness) and the report log.
Right JUDGE SENDS: the retained course (id, corners, transit area, pinger), the RunStart, every command.
Typed: course | start [DECL_SEQ] | assist [LAT LON] [air] | confirm | keepout LAT LON R | allclear | moving LAT LON HDG SPD | clear | quit
      Default: a command goes to our fake judge (rc/sim/inject; it sends with its own sequence numbers, either wire);
      --as-judge publishes the RxCommand / RxCourse itself on --wire (rc-test, a bare broker); --monitor never publishes.
RcModel is shared with the browser page (tools/judge_web.py); render(model, w, h) -> Canvas is testable without curses.
"""
from __future__ import annotations

import argparse
import curses
import json
import threading
import time
from collections import deque
from typing import Any, Deque, Dict, List, Optional

import paho.mqtt.client as mqtt

from .bench_tui import BOX_CHARS, Canvas

HELP = "course | start [SEQ] | assist [LAT LON] [air] | confirm | keepout LAT LON R | allclear | moving LAT LON HDG SPD | clear | quit"
REPORT_ORDER = ("safe_passage", "docking", "firefighting", "incident_ack", "readiness", "resource_request")


def monitor_subscriptions(team: str) -> list:
    """v0.1.59 (A3, handbook 3.4.6: never another team's namespace - the Team Village demo broker is shared): the bench JSON wire
    plus RoboNation's course topic and OUR team's request / command / reports only (was robocommand/robotx/#)."""
    root = "robocommand/robotx"
    return [("rc/#", 0), (f"{root}/course", 1), (f"{root}/{team}/request", 1), (f"{root}/{team}/command", 1),
            (f"{root}/{team}/+/report", 1)]


class RcModel:
    """Everything the judge sees, decoded from either wire. Thread-safe; snapshot() for the web page."""

    def __init__(self, team: str):
        self.team = team
        self.lock = threading.Lock()
        self.course: Optional[dict] = None
        self.course_t = 0.0
        self.course_wire = ""
        self.declarations: List[dict] = []
        self.run_start: Optional[dict] = None
        self.heartbeats: Dict[str, dict] = {}
        self.hb_times: Dict[str, Deque[float]] = {}
        self.timeline: Dict[str, List[tuple]] = {}           # vehicle -> [(t, current_task)] on every change
        self.last_report: Dict[str, Dict[str, dict]] = {}    # vehicle -> kind -> decoded report (+ _t)
        self.received: Deque[tuple] = deque(maxlen=300)      # (t, text) judge <- OCS
        self.sent: Deque[tuple] = deque(maxlen=300)          # (t, text) judge -> OCS
        self.notes: Deque[tuple] = deque(maxlen=50)
        self.last_readiness: Optional[dict] = None
        self.connected = False
        self.n_bad = 0
        self.wires_seen: Dict[str, int] = {"json": 0, "protobuf": 0}

    def note(self, text: str) -> None:
        with self.lock:
            self.notes.append((time.time(), text))

    # ---- traffic in: the topic root says the wire ------------------------------------------------------------------
    def on_wire(self, topic: str, payload: bytes) -> None:
        now = time.time()
        try:
            if topic.startswith("robocommand/"):
                from ..ocs import rc_wire
                d = rc_wire.decode(topic, payload)
                if d is None:
                    return
                where, wire = d.pop("_topic_kind"), "protobuf"
            elif topic.startswith("rc/"):
                parts = topic.split("/")
                where = parts[1] if len(parts) > 1 else ""
                if where not in ("course", "request", "report", "command"):
                    return                                     # rc/sim/* = the fake judge's control and stats
                d = json.loads(payload)
                if where == "report":
                    d.setdefault("vehicle_id", parts[-1])
                wire = "json"
            else:
                return
        except Exception:
            with self.lock:
                self.n_bad += 1
            return
        if where != "course" and d.get("team_id") not in (None, "", self.team):
            return                                             # another team's traffic (3.4.6)
        with self.lock:
            self.wires_seen[wire] += 1
            if where == "course":
                self.course, self.course_t, self.course_wire = d, now, wire
                self.sent.append((now, f"RxCourse {d.get('course_id')}: {len(d.get('boundary') or [])} corners, transit {len(d.get('uav_transit_area') or [])}, pinger {d.get('pinger_hz')} Hz  [{wire}]"))
            elif where == "request":
                d["_t"] = now
                self.declarations.append(d)
                tiers = " ".join(f"{k}:{str(v).replace('TIER_', '').lower()}" for k, v in (d.get("tiers") or {}).items())
                self.received.append((now, f"RunDeclaration seq {d.get('request_seq')}: {','.join(d.get('vehicle_ids') or [])}  {tiers}"))
            elif where == "report":
                v = d.get("vehicle_id")
                d["_t"] = now
                if d.get("kind") == "heartbeat":
                    prev = self.heartbeats.get(v)
                    task = d.get("current_task")
                    if prev is None or prev.get("current_task") != task:
                        self.timeline.setdefault(v, []).append((now, task))
                        self.timeline[v] = self.timeline[v][-30:]
                        if prev is not None:
                            self.received.append((now, f"{v} current_task {prev.get('current_task')} -> {task}"))
                    self.heartbeats[v] = d
                    self.hb_times.setdefault(v, deque(maxlen=20)).append(now)
                    return
                if d.get("kind") == "readiness":
                    self.last_readiness = d
                self.last_report.setdefault(v, {})[str(d.get("kind"))] = d
                self.received.append((now, f"{v} #{d.get('report_seq')} {report_text(d)}"))
            elif where == "command":
                if d.get("kind") == "run_start":
                    self.run_start = dict(d, _t=now)
                self.sent.append((now, f"#{d.get('command_seq')} {command_text(d)}"))

    def hb_rate(self, v: str) -> float:
        q = self.hb_times.get(v) or []
        return (len(q) - 1) / (q[-1] - q[0]) if len(q) > 1 and q[-1] > q[0] else 0.0

    def snapshot(self) -> Dict[str, Any]:
        """A JSON-ready copy for the browser page (tools/judge_web.py)."""
        now = time.time()
        with self.lock:
            vehicles = {}
            for v, hb in self.heartbeats.items():
                vehicles[v] = {"heartbeat": {k: x for k, x in hb.items() if not k.startswith("_")}, "age_s": round(now - hb["_t"], 1),
                               "rate_hz": round(self.hb_rate(v), 2),
                               "timeline": [{"t": t, "ago_s": round(now - t, 0), "task": task} for t, task in self.timeline.get(v, [])],
                               "reports": {k: dict({kk: x for kk, x in r.items() if not kk.startswith("_")}, ago_s=round(now - r["_t"], 0), text=report_text(r))
                                           for k, r in self.last_report.get(v, {}).items()}}
            for v, reps in self.last_report.items():
                vehicles.setdefault(v, {"heartbeat": None, "age_s": None, "rate_hz": 0, "timeline": [],
                                        "reports": {k: dict({kk: x for kk, x in r.items() if not kk.startswith("_")}, ago_s=round(now - r["_t"], 0), text=report_text(r))
                                                    for k, r in reps.items()}})
            decl = self.declarations[-1] if self.declarations else None
            return {"team": self.team, "connected": self.connected, "undecodable": self.n_bad, "wires": dict(self.wires_seen), "now": now,
                    "course": None if not self.course else dict(self.course, ago_s=round(now - self.course_t, 0), wire=self.course_wire),
                    "declaration": None if not decl else {k: x for k, x in decl.items() if not k.startswith("_")},
                    "declarations": len(self.declarations),
                    "run_start": None if not self.run_start else {k: x for k, x in self.run_start.items() if not k.startswith("_")},
                    "vehicles": vehicles,
                    "received": [{"t": t, "text": x} for t, x in list(self.received)[-60:]],
                    "sent": [{"t": t, "text": x} for t, x in list(self.sent)[-60:]]}


def _beacons(d: dict) -> str:
    states: Dict[str, int] = {}
    for b in d.get("buoys") or []:
        s = str(b.get("beacon") or (f"{b.get('state')}_{b.get('colour')}" if b.get("state") else "?")).replace("BEACON_STATE_", "").lower()
        states[s] = states.get(s, 0) + 1
    return " ".join(f"{k}={n}" for k, n in sorted(states.items()))


def report_text(d: dict) -> str:
    k = d.get("kind")
    if k == "safe_passage":
        bs = d.get("buoys") or []
        return f"SafePassageReport: entry {'yes' if d.get('entry_position') else '-'} exit {'yes' if d.get('exit_position') else '-'}, {len(bs)} buoys {_beacons(d)}"
    if k == "docking":
        return f"DockingReport bay {d.get('bay')}"
    if k == "firefighting":
        w = d.get("window_id", d.get("window"))
        return f"FirefightingReport window {w}" + (" (1 upper / 2 lower; 0 = unset)" if isinstance(w, int) else "")
    if k == "incident_ack":
        return f"IncidentAck of command #{d.get('command_seq')}"
    if k == "readiness":
        return f"ReadinessReport for command #{d.get('command_seq')}"
    if k == "resource_request":
        return f"ResourceDeliveryRequest {d.get('task')} {d.get('resource_color')} -> {d.get('delivery_color')}"
    return str(k)


def command_text(d: dict) -> str:
    k = d.get("kind")
    p = d.get("position") or d.get("center") or {}
    at = f" at {p.get('lat'):.6f} {p.get('lon'):.6f}" if p.get("lat") is not None else ""
    if k == "run_start":
        return f"RunStart declaration {d.get('declaration_seq')} run {d.get('run_id')}"
    if k == "assistance_request":
        return f"AssistanceRequest {d.get('domain')}{at}"
    if k == "keep_out_zone":
        return f"KeepOutZone {d.get('domain')}{at} r {d.get('radius_m')} m"
    if k == "all_clear":
        return f"AllClear {d.get('domain')}"
    if k == "moving_object":
        return f"MovingObjectAlert{at} hdg {d.get('heading_deg')} spd {d.get('speed_mps')}"
    if k == "readiness_confirm":
        return f"ReadinessConfirm report #{d.get('report_seq')} {d.get('vehicle_id') or ''}"
    return str(k)


def _f(x, fmt="{:.1f}", none="-") -> str:
    return none if x is None else fmt.format(x)


def render(m: RcModel, w: int, h: int, cmdline: str = "", last: str = "", mode: str = "fake") -> Canvas:
    c = Canvas(w, h)
    now = time.time()
    c.put(0, 0, f" rc-tui  team {m.team}  {'connected' if m.connected else 'CONNECTING'}  mode {mode}  wires seen: json {m.wires_seen['json']} protobuf {m.wires_seen['protobuf']}  undecodable {m.n_bad} ".ljust(w - 1), "bar")
    c.put(1, 1, "THE JUDGE'S VIEW  ·  left: what RoboCommand receives from our OCS  ·  right: what it sends  ·  type below  ·  the same in the browser: judge_web", "title")
    half = (w * 3) // 5
    top = 2
    c.box(top, 0, h - top - 4, half, "JUDGE RECEIVES  (OCS -> RoboCommand)")
    c.box(top, half, h - top - 4, w - half, "JUDGE SENDS  (RoboCommand -> OCS)")
    lw = half - 4
    with m.lock:
        y = top + 1
        decl = m.declarations[-1] if m.declarations else None
        if decl:
            tiers = " ".join(f"{k}:{str(v).replace('TIER_', '').lower()}" for k, v in (decl.get("tiers") or {}).items())
            c.put(y, 2, f"declaration seq {decl.get('request_seq')}: {','.join(decl.get('vehicle_ids') or [])}  {tiers}  geofence {len(decl.get('uav_geofence') or [])} pts"[:lw], "ok")
        else:
            c.put(y, 2, "declaration: none yet", "dim")
        y += 1
        for v, hb in sorted(m.heartbeats.items()):
            age = now - hb["_t"]
            c.put(y, 2, (f"{v:5s} {str(hb.get('state', '?')).replace('STATE_', ''):7s} {str(hb.get('current_task', '?')).replace('TASK_', ''):24s} "
                         f"{str(hb.get('vehicle_type') or '?'):4s} {str(hb.get('flight_phase') or '').replace('FLIGHT_PHASE_', ''):9s} "
                         f"seq {hb.get('report_seq')}  {m.hb_rate(v):.1f} Hz  age {age:.1f} s")[:lw], "ok" if age < 2 else "bad")
            y += 1
            c.put(y, 8, (f"pos {_f(hb.get('lat'), '{:.6f}')} {_f(hb.get('lon'), '{:.6f}')}  spd {_f(hb.get('spd_mps'), '{:.2f}')} m/s  hdg {_f(hb.get('heading_deg'), '{:.0f}')}  "
                         f"roll {_f(hb.get('roll_deg'))} pitch {_f(hb.get('pitch_deg'))}  alt {_f(hb.get('altitude_hae_m'))}  depth {_f(hb.get('depth_m'))}"
                         + (f"  reg {hb.get('uav_registration')}" if hb.get("uav_registration") else ""))[: lw - 6], "")
            y += 1
            tl = m.timeline.get(v, [])
            c.put(y, 8, ("tasks: " + "  >  ".join(f"{str(t).replace('TASK_', '')} ({now - tt:.0f}s ago)" for tt, t in tl[-4:]))[: lw - 6], "dim")
            y += 1
            for k in REPORT_ORDER:
                r = m.last_report.get(v, {}).get(k)
                if r:
                    c.put(y, 8, f"last {report_text(r)}  ({now - r['_t']:.0f} s ago)"[: lw - 6], "")
                    y += 1
        if not m.heartbeats:
            c.put(y, 2, "no heartbeat yet", "dim")
            y += 1
        c.put(y, 2, "log:", "title")
        y += 1
        rows = h - 4 - y - 1
        for t, txt in list(m.received)[-rows:] if rows > 0 else []:
            c.put(y, 2, f"{time.strftime('%H:%M:%S', time.localtime(t))} {txt}"[:lw], "")
            y += 1
        y = top + 1
        crs, rw = m.course, w - half - 4
        c.put(y, half + 2, ("course: " + (f"{crs.get('course_id')}  {len(crs.get('boundary') or [])} corners  transit {len(crs.get('uav_transit_area') or [])}  "
                                          f"pinger {crs.get('pinger_hz')} Hz  ({now - m.course_t:.0f} s ago, {m.course_wire})" if crs else "none (the retained RxCourse not seen)"))[:rw],
              "ok" if crs else "warn")
        y += 1
        rs = m.run_start
        c.put(y, half + 2, ("run: " + (f"RunStart run {rs.get('run_id')} for declaration {rs.get('declaration_seq')} ({now - rs['_t']:.0f} s ago)" if rs else "no RunStart yet"))[:rw],
              "ok" if rs else "dim")
        y += 2
        rows = h - 4 - y - 1
        for t, txt in list(m.sent)[-rows:] if rows > 0 else []:
            c.put(y, half + 2, f"{time.strftime('%H:%M:%S', time.localtime(t))} {txt}"[:rw], "judge")
            y += 1
        note = m.notes[-1][1] if m.notes else ""
    c.put(h - 4, 1, ("last: " + (last or note))[: w - 2], "dim")
    c.put(h - 3, 1, ("commands: " + HELP)[: w - 2], "dim")
    c.put(h - 2, 0, ("> " + cmdline)[: w - 1], "bold")
    return c


class JudgeSender:
    """What a typed judge command publishes (v0.1.29: shared by rc-tui and the judge page). mode: fake = ask our fake judge
    (rc/sim/inject), judge = publish the RxCommand / RxCourse ourselves on `wire` (rc-test, a bare broker), monitor = never."""

    def __init__(self, client, model: RcModel, mode: str, team: str, wire: str = "protobuf"):
        self.c, self.m, self.mode, self.team, self.wire = client, model, mode, team, wire
        self.cmd_seq = 9000                     # --as-judge: our own command numbering (far from the fake judge's)

    def send(self, body: dict) -> str:
        if self.mode == "monitor":
            return "refused: --monitor never publishes"
        if self.mode == "fake":
            self.c.publish("rc/sim/inject", json.dumps(body), qos=0)
            return f"asked the fake judge: {body.get('kind')}"
        if self.wire == "json":
            if body.get("kind") == "course":
                self.c.publish("rc/course", json.dumps(body["course"]), qos=0, retain=True)
                return "course published (json, retained)"
            self.cmd_seq += 1
            self.c.publish(f"rc/command/{self.team}", json.dumps(dict(body, command_seq=self.cmd_seq, team_id=self.team, sent_at=time.time())), qos=0)
            return f"command #{self.cmd_seq} {body.get('kind')} published (json)"
        from ..ocs import rc_wire
        if body.get("kind") == "course":
            self.c.publish(rc_wire.t_course(), rc_wire.encode_course(body["course"]), qos=1, retain=True)
            return "RxCourse published (retained)"
        self.cmd_seq += 1
        self.c.publish(rc_wire.t_command(self.team), rc_wire.encode_command(self.team, self.cmd_seq, body), qos=1)
        return f"RxCommand #{self.cmd_seq} {body.get('kind')} published"

    def do(self, line: str) -> str:
        """One typed judge command (course | start | assist | confirm | keepout | allclear | moving); '' for an empty line."""
        p = line.split()
        if not p:
            return ""
        w, args = p[0].lower(), p[1:]
        f = [float(x) for x in args if _num(x)]
        dom = "air" if "air" in args else "surface"
        if w == "course":
            from ..sim.marina import COURSE_ALPHA
            crs = self.m.course or {"course_id": "RCTUI-1", "boundary": [list(p) for p in COURSE_ALPHA],
                                   "uav_transit_area": [], "pinger_hz": 30000}
            return self.send({"kind": "course", "course": crs}) if self.mode == "judge" else "course: only --as-judge publishes a course (the fake judge keeps its own)"
        if w == "start":
            decl = self.m.declarations[-1] if self.m.declarations else {}
            seq = int(f[0]) if f else decl.get("request_seq")
            if seq is None:
                return "no declaration seen: start SEQ"
            return self.send({"kind": "run_start", "declaration_seq": int(seq), "run_id": 7000 + int(seq)})
        if w == "assist":
            lat, lon = (f[0], f[1]) if len(f) >= 2 else (1.28113, 103.85503)
            return self.send({"kind": "assistance_request", "position": {"lat": lat, "lon": lon}, "domain": dom})
        if w == "confirm":
            r = self.m.last_readiness
            if not r:
                return "no ReadinessReport seen"
            return self.send({"kind": "readiness_confirm", "report_seq": r.get("report_seq"), "vehicle_id": r.get("vehicle_id")})
        if w == "keepout":
            if len(f) < 3:
                return "keepout LAT LON R"
            return self.send({"kind": "keep_out_zone", "position": {"lat": f[0], "lon": f[1]}, "radius_m": f[2], "domain": dom})
        if w == "allclear":
            return self.send({"kind": "all_clear", "domain": dom})
        if w == "moving":
            if len(f) < 4:
                return "moving LAT LON HDG SPD"
            return self.send({"kind": "moving_object", "position": {"lat": f[0], "lon": f[1]}, "heading_deg": f[2], "speed_mps": f[3], "domain": dom})
        return "unknown: " + HELP


class RcTui:
    def __init__(self, a):
        self.a = a
        self.m = RcModel(a.team)
        self.mode = "monitor" if a.monitor else ("judge" if a.as_judge else "fake")
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"rc-tui-{int(time.time() * 1000) % 1000000}", protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = lambda *x: setattr(self.m, "connected", False)
        self.c.on_message = lambda c, u, msg: self.m.on_wire(msg.topic, msg.payload)
        self.sender = JudgeSender(self.c, self.m, self.mode, a.team, a.wire)
        self.cmdline, self.last, self.quit = "", "", False

    def _on_connect(self, c, u, f, rc, p=None):
        self.m.connected = True
        c.subscribe(monitor_subscriptions(self.a.team))        # v0.1.26: both wires, decoded by topic root; v0.1.59: our team only

    # ---- typed commands ----------------------------------------------------------------------------------------------
    def _sender(self) -> JudgeSender:
        s = self.sender                                    # the console's current client / model / mode / wire (tests swap them)
        s.c, s.m, s.mode, s.wire = self.c, self.m, self.mode, self.a.wire
        return s

    def send(self, body: dict) -> str:
        return self._sender().send(body)

    def do(self, line: str) -> str:
        w = (line.split() or [""])[0].lower()
        if w in ("quit", "q", "exit"):
            self.quit = True
            return "bye"
        if w == "clear":
            with self.m.lock:
                self.m.received.clear(); self.m.sent.clear()
            return "cleared"
        return self._sender().do(line)

    # ---- curses ------------------------------------------------------------------------------------------------------
    def paint(self, scr, C) -> None:
        h, w = scr.getmaxyx()
        cv = render(self.m, w, h, self.cmdline, self.last, self.mode)
        scr.erase()
        for y in range(h):
            for x, ch in enumerate(cv.rows[y][: w - 1]):
                a = cv.attrs[y][x]
                try:
                    acs = BOX_CHARS.get(ch)
                    if acs is not None:
                        scr.addch(y, x, getattr(curses, acs), C.get("dim", 0))
                    elif ch != " " or a:
                        scr.addstr(y, x, ch, C.get(a, 0))
                except curses.error:
                    pass
        scr.refresh()

    def main(self, scr) -> None:
        curses.curs_set(1)
        scr.nodelay(True)
        scr.keypad(True)
        C = {"bar": curses.A_REVERSE, "dim": curses.A_DIM, "bold": curses.A_BOLD, "title": curses.A_BOLD}
        if curses.has_colors():
            curses.start_color()
            try:
                curses.use_default_colors()
                bg = -1
            except curses.error:
                bg = curses.COLOR_BLACK
            for n, fg in ((1, curses.COLOR_GREEN), (2, curses.COLOR_RED), (3, curses.COLOR_YELLOW), (4, curses.COLOR_CYAN), (7, curses.COLOR_MAGENTA)):
                curses.init_pair(n, fg, bg)
            cp = curses.color_pair
            C.update(ok=cp(1), bad=cp(2) | curses.A_BOLD, warn=cp(3), title=cp(4) | curses.A_BOLD, judge=cp(7))
        self.c.connect_async(self.a.host, self.a.port, keepalive=15)
        self.c.loop_start()
        try:
            last = 0.0
            while not self.quit:
                ch = scr.getch()
                while ch != -1:
                    if ch in (10, 13):
                        self.last = self.do(self.cmdline)
                        self.cmdline = ""
                    elif ch in (curses.KEY_BACKSPACE, 127, 8):
                        self.cmdline = self.cmdline[:-1]
                    elif ch == 27:
                        self.cmdline = ""
                    elif 32 <= ch < 127:
                        self.cmdline += chr(ch)
                    ch = scr.getch()
                if time.monotonic() - last > 0.2:
                    self.paint(scr, C)
                    last = time.monotonic()
                time.sleep(0.03)
        finally:
            self.c.loop_stop()
            self.c.disconnect()


def _num(s: str) -> bool:
    try:
        float(s)
        return True
    except ValueError:
        return False


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1882)
    ap.add_argument("--team", default="SGIT")
    ap.add_argument("--wire", choices=["protobuf", "json"], default="protobuf", help="what --as-judge publishes (both wires are always shown)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--monitor", action="store_true", help="read-only (the course; never publishes)")
    g.add_argument("--as-judge", action="store_true", help="publish the RxCommand / RxCourse yourself (rc-test, a bare broker)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    curses.wrapper(RcTui(parse_args(argv)).main)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
