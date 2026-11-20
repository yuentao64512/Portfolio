"""Run lifecycle R0-R6 with the stall path, command authority guards, report mapping and heartbeats."""
from __future__ import annotations

from typing import List

from optiprimex.companion.config import Config
from optiprimex.companion.mqtt import Envelope
from optiprimex.ocs.cmd_authority import CmdAuthority
from optiprimex.ocs.heartbeat_publisher import HeartbeatPublisher
from optiprimex.ocs.report_mapper import ReportMapper, event_to_report
from optiprimex.ocs.run_lifecycle import R, RunLifecycle


class Clock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


COURSE = {"course_id": "C1", "boundary": [[1.299, 103.799], [1.299, 103.801], [1.301, 103.801]]}


def test_lifecycle_happy_path_and_republished_course():
    clk = Clock()
    alerts = []
    lc = RunLifecycle(stall_s=5.0, on_alert=lambda k, d: alerts.append(k), clock=clk)
    assert lc.state == R["R0"]
    lc.on_link(True)
    assert lc.state == R["R1"]
    assert lc.on_course({"course_id": "C1", "boundary": [[1, 2]]}) is False       # too few points
    assert lc.on_course(COURSE) and lc.state == R["R2"]
    lc.declare(["USV1"], {"T4": "TIER_CORE"}, [])
    assert lc.state == R["R3"] and lc.declaration_seq == 1
    lc.vehicle_status("USV1", "STATE_MANUAL")
    assert lc.state == R["R3"]
    lc.vehicle_status("USV1", "STATE_AUTO")
    assert lc.state == R["R4"]
    assert lc.on_run_start({"declaration_seq": 2, "run_id": 1}) is False and alerts == ["run_start_ignored"]
    assert lc.on_run_start({"declaration_seq": 1, "run_id": 1001}) and lc.state == R["R5"] and lc.run_id == 1001
    assert lc.on_course(COURSE) is False                                            # same course: no change
    assert lc.on_course({"course_id": "C2", "boundary": COURSE["boundary"]}) and lc.state == R["R2"]   # changed course


def test_lifecycle_stall_path_and_redeclare():
    clk = Clock()
    alerts: List[tuple] = []
    lc = RunLifecycle(stall_s=5.0, on_alert=lambda k, d: alerts.append((k, d)), clock=clk)
    lc.on_link(True)
    lc.on_course(COURSE)
    lc.declare(["USV1"], {}, [])
    lc.vehicle_status("USV1", "STATE_AUTO")
    assert lc.state == R["R4"]
    clk.t = 4.0
    lc.tick()
    assert not lc.stalled
    clk.t = 6.0
    lc.tick()
    assert lc.stalled and alerts[-1][0] == "run_start_stall"
    lc.declare(["USV1"], {}, [])                                                     # the only re-arm
    assert lc.state == R["R3"] and lc.declaration_seq == 2 and not lc.stalled
    lc.vehicle_status("USV1", "STATE_AUTO")
    assert lc.on_run_start({"declaration_seq": 2, "run_id": 7}) and lc.state == R["R5"]
    lc.end()
    assert lc.state == R["R6"]
    lc.on_link(False)
    assert lc.state == R["R0"]


class FakeBus:
    def __init__(self):
        self.cfg = Config()
        self.cfg.vehicle_id = "OCS"
        self.run_id = None
        self.declaration_seq = None
        self.published = []
        self._n = 0

    def publish(self, topic, kind, body, qos=1, intent_id=None, vehicle_id=None, origin=None, **kw):
        self._n += 1
        self.published.append({"topic": topic, "kind": kind, "body": body, "intent_id": intent_id, "vehicle_id": vehicle_id, "origin": origin})
        return f"msg-{self._n}"


def intent(action, task=None, args=None, vehicle_id="USV1"):
    env = Envelope.wrap("intent", {"action": action, "task": task, "args": args or {}, "operator": "t"}, vehicle_id, 1)
    return env


def make_authority(bench: bool, course_net: bool = False, lc_state: str = "DISCONNECTED"):
    bus = FakeBus()
    lc = RunLifecycle()
    if lc_state != "DISCONNECTED":
        lc.on_link(True)
        if lc_state != "CONNECTED":
            lc.on_course(COURSE)
        if lc_state in ("DECLARED", "READY", "RUNNING"):
            lc.declare(["USV1"], {}, [])
        if lc_state in ("READY", "RUNNING"):
            lc.vehicle_status("USV1", "STATE_AUTO")
        if lc_state == "RUNNING":
            lc.on_run_start({"declaration_seq": 1, "run_id": 1001})
    results = []
    inj = {"run_start": lambda a: "injected"}
    auth = CmdAuthority(bus, lc, {"USV1": "usv"}, bench, lambda: course_net, results.append, injectors=inj)
    return bus, lc, auth, results


def test_authority_normal_guards():
    bus, lc, auth, results = make_authority(bench=False, lc_state="COURSE_VALID")
    r = auth.on_intent("t", intent("START_TASK", "T4"))
    assert r.result == "refused" and "R5+" in r.reason
    r = auth.on_intent("t", intent("GOTO", args={"x_n": 1, "y_e": 2}))
    assert r.result == "refused" and "bench only" in r.reason
    r = auth.on_intent("t", intent("ARM"))
    assert r.result == "refused" and "never" in r.reason
    r = auth.on_intent("t", intent("ENABLE_AUTO"))
    assert r.result == "accepted" and bus.published[-1]["body"]["action"] == "ENABLE_AUTO"
    assert bus.published[-1]["intent_id"] == r.intent_id and bus.published[-1]["origin"] == "real"
    assert bus.published[-1]["topic"].endswith("cmd/usv/run")


def test_authority_bench_mode_relaxes_only_ocs_guards():
    bus, lc, auth, results = make_authority(bench=True, lc_state="CONNECTED")
    r = auth.on_intent("t", intent("START_TASK", "T4"))
    assert r.result == "accepted" and bus.published[-1]["origin"] == "bench"          # R-guard relaxed...
    r = auth.on_intent("t", intent("GOTO", args={"x_n": 1, "y_e": 2}))
    assert r.result == "accepted" and bus.published[-1]["body"] == {"action": "START_TASK", "task": "GOTO", "args": {"x_n": 1, "y_e": 2}}
    r = auth.on_intent("t", intent("INJECT", args={"what": "run_start"}))
    assert r.result == "accepted" and r.reason == "injected"
    r = auth.on_intent("t", intent("DISARM"))
    assert r.result == "refused"                                                       # ...but never the never-list


def test_authority_bench_refused_on_course_network():
    bus, lc, auth, results = make_authority(bench=True, course_net=True, lc_state="RUNNING")
    assert not auth.bench_mode
    assert auth.on_intent("t", intent("INJECT", args={"what": "run_start"})).result == "refused"
    assert auth.on_intent("t", intent("START_TASK", "T4")).result == "accepted"     # normal guard satisfied in R5


def test_report_mapping_and_coalescing():
    clk = Clock()
    sent = []
    rm = ReportMapper(lambda v, b: (sent.append((v, b)) or len(sent)), min_period_s=1.0, clock=clk)
    assert rm.on_event("USV1", {"event": "task_state", "to": "X"}) is False
    rm.on_event("USV1", {"event": "incident", "kind": "ack", "command_seq": 4})
    rm.on_event("USV1", {"event": "safe_passage", "entry": {"lat": 1, "lon": 2}, "buoys": [1]})
    rm.on_event("USV1", {"event": "safe_passage", "entry": {"lat": 1, "lon": 2}, "buoys": [1, 2]})   # coalesced
    assert [b["kind"] for _, b in sent] == ["incident_ack", "safe_passage"] and sent[-1][1]["buoys"] == [1]
    clk.t = 1.1
    rm.tick()
    assert len(sent) == 3 and sent[-1][1]["buoys"] == [1, 2]
    k, body = event_to_report("USV1", {"event": "incident", "kind": "readiness", "command_seq": 4, "position": {"lat": 1, "lon": 2}})
    assert body["kind"] == "readiness" and body["command_seq"] == 4


class FakeJudge:
    def __init__(self):
        self.reports = []

    def publish_report(self, v, body):
        self.reports.append((v, body))
        return len(self.reports)


def test_heartbeat_mapping_and_stale_rule():
    clk = Clock()
    j = FakeJudge()
    hb = HeartbeatPublisher(j, ["USV1"], hz=2.0, stale_stop_s=10.0, clock=clk)
    hb.enabled = True
    hb.tick()
    assert j.reports == []                                                 # no status yet, no fabricated heartbeat
    hb.update("USV1", {"state": "AUTO_TASK", "robot_state": "STATE_AUTO", "current_task": "TASK_DYNAMIC_INCIDENT", "lat": 1.3, "lon": 103.8,
                       "sog_mps": 0.5, "hdg_deg": 90.0})
    hb.tick()
    assert j.reports[-1][1]["state"] == "STATE_AUTO" and j.reports[-1][1]["current_task"] == "TASK_DYNAMIC_INCIDENT"
    clk.t = 0.4
    hb.tick()
    assert len(j.reports) == 1                                             # 2 Hz
    clk.t = 0.5
    hb.tick()
    assert len(j.reports) == 2
    clk.t = 11.0
    hb.tick()
    assert len(j.reports) == 2 and hb.stopped_stale["USV1"]              # decision 4: stop after 10 s
    assert HeartbeatPublisher.to_heartbeat({"state": "FAULT", "robot_state": "STATE_UNKNOWN", "current_task": "TASK_NONE"})["state"] == "STATE_MANUAL"
    assert HeartbeatPublisher.to_heartbeat({"state": "KILLED", "robot_state": "STATE_KILLED", "current_task": "TASK_NONE"})["state"] == "STATE_KILLED"


def test_lifecycle_prepare_acks_and_reset():
    """declare with require_prepare_ack: R3 only when every vehicle acks PREPARE DONE; a REJECTED or a timeout drops it;
    reset takes R3-R6 back to R2 (R1 without a course)."""
    clk = Clock()
    alerts: List[tuple] = []
    lc = RunLifecycle(stall_s=5.0, on_alert=lambda k, d: alerts.append((k, d)), clock=clk, prepare_ack_s=3.0)
    lc.on_link(True)
    lc.on_course(COURSE)
    lc.declare(["USV1", "UAV1"], {}, [], require_prepare_ack=True)
    assert lc.state == R["R2"] and lc.body()["prepare_pending"] == ["USV1", "UAV1"] and lc.declaration_seq == 1
    assert lc.on_prepare_ack("USV1", "DONE") is False and lc.state == R["R2"]
    assert lc.on_prepare_ack("UAV1", "DONE") is True and lc.state == R["R3"] and lc.body()["prepare_pending"] == []
    # a rejection drops the next declaration
    lc.end()
    assert lc.state == R["R6"]
    lc.declare(["USV1"], {}, [], require_prepare_ack=True)
    assert lc.on_prepare_ack("USV1", "REJECTED", "not armed") is False and lc.state == R["R6"] and alerts[-1][0] == "prepare_rejected"
    # a timeout too
    lc.declare(["USV1"], {}, [], require_prepare_ack=True)
    clk.t = 4.0
    lc.tick()
    assert alerts[-1][0] == "prepare_timeout" and lc.body()["prepare_pending"] == [] and lc.state == R["R6"]
    # reset: R6 -> R2 with a valid course, the run context gone
    assert lc.reset() and lc.state == R["R2"] and lc.declaration is None and lc.run_id is None and lc.ready_vehicles == {}
    assert lc.reset() and lc.state == R["R2"]                                       # idempotent
    lc2 = RunLifecycle(clock=clk)
    lc2.on_link(True)
    assert lc2.reset() is False and lc2.state == R["R1"]                            # nothing to reset
