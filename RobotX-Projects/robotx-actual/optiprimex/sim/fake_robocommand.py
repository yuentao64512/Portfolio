#!/usr/bin/env python3
"""fake_robocommand: a scripted stand-in for the RoboCommand judge system, JSON over the hub broker.

Speaks the rc/* topics of ocs.judge_client.FakeJudgeClient:
  rc/course (retained)            <- RxCourse
  rc/command/<team>               <- RxCommand: run_start, assistance_request, keep_out_zone, all_clear,
                                     moving_object, readiness_confirm
  rc/request/<team>               -> RxRequest: run_declaration
  rc/report/<team>/<vehicle>      -> RxReport: heartbeat, incident_ack, readiness, safe_passage, docking, ...
  rc/sim/inject                   test/CLI trigger: {"kind": "assistance_request", "lat":.., "lon":..}; {"kind": "sim_config", "refuse": true}
                                  (v0.1.43: refuse the next declarations, silently - the fault-timing test);
                                  {"kind": "sim_config", "auto_incident_after": null | s} switches the auto incident (tests);
                                  {"kind": "sim_reset"} (v0.1.33, RESET SIM): the counters and the current declaration / run forgotten
                                  (the sequence numbers keep counting up, as a judge's do)
                                  {"kind": "course", "course": "alpha" | "extra" | ...} (v0.1.53, the judge page's "publish course"): select a
                                  catalogue course (sim/courses.py) and re-publish the retained RxCourse on the current wire
  rc/sim/stats (1 Hz, retained)   counters for tests: heartbeats per vehicle, states seen, reports, rate violations

The course (v0.1.53): --course ID (a catalogue course: extra | alpha, sim/courses.py) | world (the world file's `course:`, v0.1.46.1);
without --course: the world's course when --world names one, else Course Extra's boundary under --course-id. run_bench.sh passes
--course ${COURSE:-extra}, so the bench's judge publishes EXTRA whatever the world (the SIM page offers "move the world there").

--wire protobuf (v0.1.23, ROADMAP WP4): the same judge on the real wire instead - RoboNation's RobotX 2026 protobuf messages
on robocommand/robotx/course, /<team>/request, /<team>/<vehicle>/report, /<team>/command (ocs/rc_wire), RunStart.declaration_seq
= the RxRequest seq, ReadinessConfirm.report_seq = the RxReport seq, as RoboCommand does. rc/sim/inject and rc/sim/stats stay JSON.

Behaviour: validates a declaration (vehicle_ids non-empty, tiers present, geofence >= 4 vertices when a UAV is
declared), then publishes RunStart after --start-delay unless --refuse; answers every readiness report with a
ReadinessConfirm after --confirm-delay; --auto-incident-after fires an AssistanceRequest that many seconds
after RunStart. Rate check: > 5 msg/s from one team is counted as a violation (handbook 3.4).
"""
from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from collections import deque
from typing import Deque, Dict, List, Optional

import paho.mqtt.client as mqtt

from ..companion.mqtt import start_presence


def world_course(path):
    """v0.1.46.1: (boundary [[lat, lon]...], uav_transit_area, course_id) from a world file's `course:` key, or (None, [], None).
    `course: bounds` = the world's bounds (the pool's walls), `course: course_alpha` = RoboNation's Course Alpha (the marina), a list of
    [x_n, y_e] = that polygon in the world frame; `course_id:` names it."""
    if not path:
        return None, [], None
    try:
        from . import world as W
        from .marina import COURSE_ALPHA, TRANSIT_A
        wd = W.load(path)
    except Exception:
        return None, [], None
    c = wd.get("course")
    cid = wd.get("course_id")
    if c == "course_alpha":
        return [list(p) for p in COURSE_ALPHA], [list(p) for p in TRANSIT_A], cid or "COURSE-ALPHA"
    if isinstance(c, str) and c != "bounds":                 # v0.1.53: a catalogue course (world_extra: `course: EXTRA`)
        from . import courses as C
        crs = C.get(c)
        if crs is not None:
            return [list(p) for p in crs.corners], [list(p) for p in crs.transit], cid or crs.id
    pts = wd.get("bounds") if c == "bounds" else (c if isinstance(c, list) else None)
    if not pts or len(pts) < 3:
        return None, [], None
    fr = W.WorldFrame(wd.get("origin"))
    return [list(fr.to_latlon(float(p[0]), float(p[1]))) for p in pts], [], cid or f"SIM-{str(wd.get('name') or 'WORLD').upper()}"


class FakeRoboCommand:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self._refuse0 = bool(a.refuse)                    # v0.1.43: what RESET SIM restores
        self.team = a.team
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake-robocommand", protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_message = self._on_message
        self.cmd_seq = 0
        self.run_id = 0
        self.declaration: Optional[dict] = None
        self.run_started_at: Optional[float] = None
        self.incident_fired = False
        self.lock = threading.Lock()
        self.stats: Dict = {"declarations": 0, "declarations_refused": 0, "run_starts": 0, "heartbeats": {}, "last_state": {},
                            "last_task": {}, "states_seen": {}, "tasks_seen": {}, "reports": [], "incident_acks": 0, "readiness": 0, "confirms": 0,
                            "rate_violations": 0, "max_rate": 0, "commands": 0}
        self._times: Deque[float] = deque()
        self.log_fh = open(a.log, "a") if a.log else None
        self.pb = None
        if getattr(a, "wire", "json") == "protobuf":
            from ..ocs import rc_wire
            self.pb = rc_wire
        self._stop = threading.Event()

    # ---- helpers ---------------------------------------------------------------------------
    def log(self, kind: str, body: dict) -> None:
        rec = {"t": time.time(), "kind": kind, **body}
        print(f"[fake_rc] {kind} {json.dumps(body)[:200]}", flush=True)
        if self.log_fh:
            self.log_fh.write(json.dumps(rec) + "\n")
            self.log_fh.flush()

    def command(self, kind: str, **fields) -> dict:
        with self.lock:
            self.cmd_seq += 1
            msg = {"kind": kind, "command_seq": self.cmd_seq, "team_id": self.team, "sent_at": time.time(), **fields}
        if self.pb is not None:
            self.c.publish(self.pb.t_command(self.team), self.pb.encode_command(self.team, msg["command_seq"], msg), qos=1)
        else:
            self.c.publish(f"rc/command/{self.team}", json.dumps(msg), qos=0)
        self.stats["commands"] += 1
        self.log("command_out", msg)
        return msg

    def course(self) -> dict:
        """The RxCourse body. v0.1.53: an explicit course (--course ID, or the judge page's {"kind": "course"} inject) = that catalogue
        course; --course world / none = the world's course (v0.1.46.1); else the default course (EXTRA from the command line; a bare
        object without the v0.1.53 arguments keeps Course Alpha, as before) under --course-id."""
        from . import courses as C
        from .marina import COURSE_ALPHA, TRANSIT_A               # v0.1.33: RoboNation's Course Alpha, as rc-test publishes it
        sel = getattr(self.a, "course", None)
        crs = C.get(sel) if sel and str(sel).lower() != "world" else None
        if crs is not None:
            return dict(crs.body(), published_at=time.time())
        boundary, transit, cid = world_course(getattr(self.a, "world", None))
        if boundary:
            return {"course_id": cid or self.a.course_id, "boundary": boundary, "uav_transit_area": transit, "pinger_hz": 30000,
                    "published_at": time.time()}
        dflt = C.get(getattr(self.a, "default_course", None))
        if dflt is not None:
            return dict(dflt.body(), course_id=self.a.course_id or dflt.id, published_at=time.time())
        return {"course_id": self.a.course_id, "boundary": [list(p) for p in COURSE_ALPHA], "uav_transit_area": [list(p) for p in TRANSIT_A],
                "pinger_hz": 30000, "published_at": time.time()}

    def publish_course(self) -> dict:
        """The retained RxCourse on the current wire (at connect, and again when the judge page selects another course)."""
        crs = self.course()
        if self.pb is not None:
            self.c.publish(self.pb.t_course(), self.pb.encode_course(crs), qos=1, retain=True)
        else:
            self.c.publish("rc/course", json.dumps(crs), qos=0, retain=True)
        self.log("course_published", {"course_id": crs.get("course_id"), "wire": "protobuf" if self.pb is not None else "json"})
        return crs

    def _rate_check(self) -> None:
        now = time.monotonic()
        self._times.append(now)
        while self._times and now - self._times[0] > 1.0:
            self._times.popleft()
        n = len(self._times)
        self.stats["max_rate"] = max(self.stats["max_rate"], n)
        if n > self.a.rate_cap:
            self.stats["rate_violations"] += 1

    # ---- MQTT -------------------------------------------------------------------------------
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        client.subscribe("rc/sim/inject", 0)
        if self.pb is not None:
            client.subscribe(self.pb.t_request(self.team), 1)
            client.subscribe(self.pb.t_report(self.team, "+"), 1)
            self.publish_course()
            return
        client.subscribe(f"rc/request/{self.team}", 0)
        client.subscribe(f"rc/report/{self.team}/#", 0)
        self.publish_course()

    def _on_message(self, client, userdata, msg):
        if self.pb is not None and msg.topic.startswith("robocommand/"):
            try:
                body = self.pb.decode(msg.topic, msg.payload)
            except Exception as e:
                self.log("malformed", {"topic": msg.topic, "error": str(e)})
                return
            if body is None:
                return
            if body.get("team_id") != self.team:
                self.log("wrong_team", {"topic": msg.topic, "team_id": body.get("team_id")})
                return
            self._rate_check()
            if body["_topic_kind"] == "request":
                body["declaration_seq"] = body.get("request_seq")      # RunStart answers with the RxRequest seq (rx_commands.proto)
                self.on_request(body)
            elif body["_topic_kind"] == "report":
                self.on_report(body.get("vehicle_id"), body)
            return
        try:
            body = json.loads(msg.payload)
        except Exception:
            self.log("malformed", {"topic": msg.topic})
            return
        if msg.topic == "rc/sim/inject":
            self.inject(body)
            return
        if body.get("team_id") != self.team:
            self.log("wrong_team", {"topic": msg.topic, "team_id": body.get("team_id")})
            return
        self._rate_check()
        if msg.topic.startswith("rc/request/"):
            self.on_request(body)
        else:
            self.on_report(msg.topic.split("/")[-1], body)

    def on_request(self, req: dict) -> None:
        if req.get("kind") != "run_declaration":
            return
        self.stats["declarations"] += 1
        ok, why = self.validate_declaration(req)
        self.log("declaration", {"ok": ok, "why": why, "declaration_seq": req.get("declaration_seq"), "vehicles": req.get("vehicle_ids")})
        if not ok or self.a.refuse:
            self.stats["declarations_refused"] += 1
            return   # a refused declaration is silent (ADR-8 stall path)
        self.declaration = req
        threading.Timer(self.a.start_delay, self.send_run_start, args=(req.get("declaration_seq"),)).start()

    def validate_declaration(self, req: dict):
        if not req.get("vehicle_ids"):
            return False, "no vehicles"
        if not req.get("tiers"):
            return False, "no tiers"
        has_uav = any(v.upper().startswith("UAV") for v in req["vehicle_ids"])
        if has_uav and len(req.get("uav_geofence") or []) < 4:
            return False, "UAV declared without a geofence of >= 4 vertices"
        if has_uav:
            alt = req.get("uav_geofence_max_altitude_hae_m")
            if alt is None or not (0 < float(alt) <= 50):
                return False, "uav_geofence_max_altitude_hae_m must be in (0, 50]"
        return True, ""

    def send_run_start(self, declaration_seq) -> None:
        if self._stop.is_set():
            return
        self.run_id += 1
        self.run_started_at = time.monotonic()
        self.incident_fired = False
        self.stats["run_starts"] += 1
        self.command("run_start", declaration_seq=declaration_seq, run_id=1000 + self.run_id)
        if self.a.auto_incident_after is not None:
            threading.Timer(self.a.auto_incident_after, self.fire_incident).start()

    def fire_incident(self) -> None:
        if self._stop.is_set() or self.incident_fired:
            return
        self.incident_fired = True
        self.command("assistance_request", position={"lat": self.a.incident_lat, "lon": self.a.incident_lon}, domain="surface")

    def on_report(self, vehicle_id: str, rep: dict) -> None:
        kind = rep.get("kind")
        if kind == "heartbeat":
            self.stats["heartbeats"][vehicle_id] = self.stats["heartbeats"].get(vehicle_id, 0) + 1
            self.stats["last_state"][vehicle_id] = rep.get("state")
            self.stats["last_task"][vehicle_id] = rep.get("current_task")
            self.stats["states_seen"].setdefault(vehicle_id, [])
            self.stats["tasks_seen"].setdefault(vehicle_id, [])
            if rep.get("state") not in self.stats["states_seen"][vehicle_id]:
                self.stats["states_seen"][vehicle_id].append(rep.get("state"))
            if rep.get("current_task") not in self.stats["tasks_seen"][vehicle_id]:
                self.stats["tasks_seen"][vehicle_id].append(rep.get("current_task"))
            return
        self.stats["reports"].append({"vehicle": vehicle_id, "kind": kind, "report_seq": rep.get("report_seq"), "t": time.time()})
        self.log("report", {"vehicle": vehicle_id, "kind": kind, "report_seq": rep.get("report_seq"), "command_seq": rep.get("command_seq")})
        if kind == "incident_ack":
            self.stats["incident_acks"] += 1
        elif kind == "readiness":
            self.stats["readiness"] += 1
            threading.Timer(self.a.confirm_delay, self.command, args=("readiness_confirm",),
                            kwargs={"vehicle_id": vehicle_id, "report_seq": rep.get("report_seq"), "ref_command_seq": rep.get("command_seq")}).start()
            self.stats["confirms"] += 1

    def inject(self, body: dict) -> None:
        kind = body.get("kind")
        if kind == "sim_config":                     # v0.1.26 (tests): {"kind": "sim_config", "auto_incident_after": null | seconds}
            if "auto_incident_after" in body:
                self.a.auto_incident_after = body.get("auto_incident_after")
            if "refuse" in body:                         # v0.1.43: the fault-timing test - refuse declarations from now on (silent)
                self.a.refuse = bool(body["refuse"])
            self.log("sim_config", {"auto_incident_after": self.a.auto_incident_after, "refuse": self.a.refuse})
            return
        if kind == "course":                         # v0.1.53: the judge page's course select - the RxCourse goes out as the judge's
            from . import courses as C
            crs = C.get(body.get("course"))
            if crs is None:
                self.log("course_refused", {"course": body.get("course"), "known": list(C.COURSES)})
                return
            self.a.course = crs.id
            self.publish_course()
            return
        if kind == "sim_reset":
            if hasattr(self, "a"):
                self.a.refuse = getattr(self, "_refuse0", False)   # v0.1.43: RESET SIM restores the launch's --refuse
            with self.lock:
                self.declaration, self.run_started_at, self.incident_fired = None, None, False
                for k, v in list(self.stats.items()):
                    self.stats[k] = type(v)() if isinstance(v, (dict, list)) else 0
            self._times.clear()
            self.log("sim_reset", {"run_id": self.run_id, "cmd_seq": self.cmd_seq})
            return
        fields = {k: v for k, v in body.items() if k != "kind"}
        if kind == "assistance_request" and "position" not in fields:
            fields["position"] = {"lat": fields.pop("lat", self.a.incident_lat), "lon": fields.pop("lon", self.a.incident_lon)}
            fields.setdefault("domain", "surface")
        if kind == "run_start":
            self.send_run_start(fields.get("declaration_seq", (self.declaration or {}).get("declaration_seq")))
            return
        self.command(kind, **fields)

    # ---- main ---------------------------------------------------------------------------------
    def run(self) -> None:
        self.c.connect(self.a.host, self.a.port, keepalive=15)
        self.c.loop_start()
        stop_presence = start_presence(self.c, "fake_robocommand", f"{self.a.host}:{self.a.port}")
        print(f"[fake_rc] judge ({'protobuf' if self.pb else 'json'} wire) for team {self.team} on {self.a.host}:{self.a.port} start_delay={self.a.start_delay}s "
              f"refuse={self.a.refuse} auto_incident_after={self.a.auto_incident_after}", flush=True)
        try:
            while not self._stop.is_set():
                self.c.publish("rc/sim/stats", json.dumps(self.stats), qos=0, retain=True)
                time.sleep(1.0)
        except KeyboardInterrupt:
            pass
        finally:
            self._stop.set()
            stop_presence()
            self.c.loop_stop()
            self.c.disconnect()


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1882)
    ap.add_argument("--team", default="SGIT")
    ap.add_argument("--course-id", default="BENCH-COURSE-1", help="the course_id when the course is the default one (no --course, no world course)")
    ap.add_argument("--course", default=None, help="v0.1.53: extra | alpha (sim/courses.py; the id goes out as course_id) | world (the world's "
                    "`course:`); default: the world's course, else Course Extra's boundary. The judge page's course select changes it at run time")
    ap.add_argument("--world", default=None, help="v0.1.46.1: the course comes from the world file's `course:` (bounds | course_alpha | [[x_n, y_e], ...]); "
                    "absent = the default course (v0.1.53: Course Extra; --course overrides both). A course received from rc-test / RoboCommand still overrides it on the maps")
    ap.add_argument("--start-delay", type=float, default=2.0, help="seconds from a valid declaration to RunStart")
    ap.add_argument("--refuse", action="store_true", help="never send RunStart (stall-path test)")
    ap.add_argument("--confirm-delay", type=float, default=1.0)
    ap.add_argument("--auto-incident-after", type=float, default=None)
    ap.add_argument("--incident-lat", type=float, default=1.28113)
    ap.add_argument("--incident-lon", type=float, default=103.85503)
    ap.add_argument("--rate-cap", type=float, default=5.0)
    ap.add_argument("--log", default=None, help="JSONL log of everything exchanged")
    ap.add_argument("--wire", choices=["json", "protobuf"], default="json",
                    help="json = the rc/* bench topics; protobuf = RoboNation's RobotX 2026 messages on robocommand/robotx/*")
    a = ap.parse_args(argv)
    a.default_course = "EXTRA"                       # v0.1.53 (sim/courses.DEFAULT)
    return a


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None) -> int:
    signal.signal(signal.SIGTERM, _sigterm)
    FakeRoboCommand(parse_args(argv)).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
