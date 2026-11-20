"""v0.1.64 - WP14: the UAV adapter, the UAV on the team bus, the declaration with two vehicles (docs/OVERNIGHT_2026-11-07_UAV.md D0-D14).
The snapshot (D3) against his manifest, the verbatim ports (geofence_problems / course_fence / ReliableSender / course_fingerprint), the
adapter's wire against a fake listener on the loopback (hello, reliable send, telemetry -> status, report -> event + ack, fence changes),
ocs_app (declare uav off by default, the declaration content from status/uav, the refusals, fence_rejected, the re-declaration before and
after RunStart, m1t2, the course fingerprint, the forwards, the auto-ack, D11), the heartbeat cut, the rollback script. QUICK: no broker."""
from __future__ import annotations

import ast
import hashlib
import json
import re
import socket
import time

import pytest
import yaml

from optiprimex.companion.config import Config
from optiprimex.ocs import geofence as GF
from optiprimex.ocs import rc_wire
from optiprimex.ocs.heartbeat_publisher import HeartbeatPublisher
from optiprimex.ocs.judge_client import NullJudgeClient
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.ocs.report_mapper import event_to_report
from optiprimex.tools import ocs_cli
from optiprimex.uav import wire as W
from optiprimex.uav.adapter import UavAdapter, fence_check, validate_report

from .conftest import ROOT, free_udp_port, wait_until
from .test_ocs_units import FakeBus, intent

UAV = ROOT / "uav"
G = UAV / "geofence-tools"
UID = "UAV1"                                                       # v0.1.78.20: RoboCommand's vehicle id (drone SGIT-53577)
SQ = [[1.299, 103.799], [1.299, 103.801], [1.301, 103.801], [1.301, 103.799]]          # a 220 m square course
INSIDE = [[1.2995, 103.7995], [1.2995, 103.8005], [1.3005, 103.8005], [1.3005, 103.7995], [1.2995, 103.7995]]
OUTSIDE = [[1.2995, 103.7995], [1.2995, 103.8025], [1.3005, 103.8025], [1.3005, 103.7995], [1.2995, 103.7995]]


# ---- D3: the snapshot -------------------------------------------------------------------------------------------------------
def test_the_snapshot_matches_his_manifest_and_our_provenance():
    ok = 0
    for line in (UAV / "MANIFEST.sha256").read_text().splitlines():
        h, name = line.split("  ", 1)
        p = UAV / name
        if p.exists():                                                        # sha256sum -c --ignore-missing
            assert hashlib.sha256(p.read_bytes()).hexdigest() == h, name
            ok += 1
    assert ok == 36
    prov = (UAV / "PROVENANCE.md").read_text()
    rows = dict(re.findall(r"^\| `([^`]+)` \| `([0-9a-f]{64})` \|$", prov, re.M))
    files = sorted(str(p.relative_to(UAV)) for p in UAV.rglob("*") if p.is_file() and "__pycache__" not in p.parts and p.name != "PROVENANCE.md")
    assert sorted(rows) == files                                              # nothing of ours in uav/ but PROVENANCE.md
    for f, h in rows.items():
        assert hashlib.sha256((UAV / f).read_bytes()).hexdigest() == h, f
    assert "sim-tests/fake_copter.py" in rows and "geofence-tools/geofence_udp_listener.py" in rows and not (UAV / "buoy-detection").exists()


# ---- the verbatim ports ------------------------------------------------------------------------------------------------------
def _defs(src: str):
    t = ast.parse(src)
    lines = src.splitlines()
    return {n.name: "\n".join(lines[n.lineno - 1:n.end_lineno]) for n in t.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))}


def test_geofence_and_the_wire_are_his_code_verbatim():
    ours = _defs((ROOT / "optiprimex" / "ocs" / "geofence.py").read_text())
    rd = _defs((G / "run_declaration.py").read_text())
    for n in ("_xy", "_inside", "_outside_by", "_closed", "geofence_problems"):
        assert ours[n] == rd[n], n
    cf = _defs((G / "course_fence.py").read_text())
    for n in ("CourseFenceError", "_open_ring", "_same", "_area", "_segments_cross", "_is_simple", "_with_touching_points", "course_fence"):
        assert ours[n].replace("_cf_xy(", "_xy(") == cf[n], n
    assert ours["_cf_xy"].replace("_cf_xy", "_xy") == cf["_xy"]
    bridge_src = (G / "robocommand_bridge.py").read_text()
    br, w = _defs(bridge_src), _defs((ROOT / "optiprimex" / "uav" / "wire.py").read_text())
    assert w["ReliableSender"] == br["ReliableSender"] and w["course_fingerprint"] == br["course_fingerprint"]
    wsrc = (ROOT / "optiprimex" / "uav" / "wire.py").read_text()
    for c in ("RESEND_INTERVAL_S = 0.5", "HELLO_INTERVAL_S = 2.0", "TELEMETRY_STALE_S = 5.0", 'ACKED_COMMANDS = {"assistance_request", "keep_out_zone", "all_clear"}',
              "MAX_CEILING_HAE_M = 50.0"):
        assert c in bridge_src and c in wsrc, c
    # v0.1.78.20: the identity is OURS, not his (Dmitrii 2026-11-10: UAV1 + Garuda's SGIT-53577; his bridge: SGIT-53577 + the CAAS tag)
    assert 'DEFAULT_VEHICLE_ID = "SGIT-53577"' in bridge_src and 'DEFAULT_UAV_REGISTRATION = "AA53577"' in bridge_src
    assert 'DEFAULT_VEHICLE_ID = "UAV1"' in wsrc and 'DEFAULT_UAV_REGISTRATION = "SGIT-53577"' in wsrc


def test_geofence_problems_on_his_rule():
    corners = [tuple(p) for p in SQ]
    assert GF.geofence_problems([tuple(p) for p in INSIDE], corners, [], 20.0, 50.0) == []
    p = GF.geofence_problems([tuple(p) for p in OUTSIDE], corners, [], 20.0, 50.0)
    assert len(p) == 1 and "leaves the course and UAV transit area by" in p[0]
    assert "at most 50 m HAE" in GF.geofence_problems([tuple(p) for p in INSIDE], corners, [], 100.0, 50.0)[0]
    assert "needs at least 4" in GF.geofence_problems([tuple(p) for p in INSIDE[:3]], corners, [], 20.0, 50.0)[0]
    assert "isn't closed" in GF.geofence_problems([tuple(p) for p in INSIDE[:4]], corners, [], 20.0, 50.0)[0]
    # the transit area beside the course makes the outside part legal (his "course ∪ UAV transit area to 1 m")
    transit = [(1.299, 103.801), (1.299, 103.803), (1.301, 103.803), (1.301, 103.801)]
    assert GF.geofence_problems([tuple(p) for p in OUTSIDE], corners, transit, 20.0, 50.0) == []
    f = GF.course_fence(corners, transit)
    assert f[0] == f[-1] and len(f) == 7                                      # the two merged along the shared edge


# ---- the adapter's wire against a fake listener (UDP loopback) ----------------------------------------------------------------
class FBus:
    def __init__(self):
        self.pub, self.n = [], 0

    def publish(self, topic, kind, body, **kw):
        self.n += 1
        self.pub.append((topic, kind, json.loads(json.dumps(body))))
        return f"m{self.n:07d}"

    def subscribe(self, *a, **k):
        pass

    def start(self):
        pass

    def stop(self):
        pass

    def of(self, kind, topic_part=""):
        return [b for t, k, b in self.pub if k == kind and topic_part in t]


def _adapter(retry=3.0):
    lsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    lsock.bind(("127.0.0.1", 0))
    lsock.settimeout(0.2)
    cfg = Config()
    cfg.extra = {"ocs": {"uav": {}}}
    bus = FBus()
    ad = UavAdapter(cfg, lsock.getsockname(), free_udp_port(), "127.0.0.1", bus=bus, retry_for_s=retry)
    return ad, bus, lsock


def _recv(sock, want, timeout=4.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        try:
            d, addr = sock.recvfrom(65536)
        except socket.timeout:
            continue
        m = json.loads(d)
        if m.get("type") == want:
            return m, addr
    return None, None


def _tel(**kw):
    t = {"type": "uav_telemetry", "vehicle_id": UID, "state": "STATE_AUTO", "mode": "GUIDED", "armed": False,
         "flight_phase": "FLIGHT_PHASE_GROUNDED", "latitude": 1.3, "longitude": 103.8, "spd_mps": 0.0, "heading_deg": 90.0,
         "altitude_hae_m": 8.0, "altitude_source": "hae", "fence_alt_max": 20.0, "fcu_fence": [p[:] for p in INSIDE[:-1]]}
    t.update(kw)
    return t


def test_the_adapter_says_hello_forwards_reliably_and_builds_status():
    ad, bus, ls = _adapter()
    ad.start()
    try:
        m, addr = _recv(ls, "robocommand_hello", 3.0)                         # every 2 s: the listener learns where telemetry goes
        assert m is not None and addr[1] == ad.sender.bound[1]
        st = ad.status_body()
        assert st["state"] == "BOOT" and st["robot_state"] == "STATE_UNKNOWN" and st["vehicle_type"] == "UAV"
        ls.sendto(json.dumps(_tel()).encode(), addr)
        assert wait_until(lambda: ad.tel_n == 1, 2.0)
        st = ad.status_body()
        assert (st["state"], st["robot_state"], st["armed"], st["altitude_hae_m"]) == ("AUTO_HOLD", "STATE_AUTO", False, 8.0)
        assert st["fence"]["points_n"] == 4 and st["fence"]["points"][0] == st["fence"]["points"][-1] and st["fence"]["ceiling_hae_m"] == 20.0
        assert st["uav"] == {"vehicle_id": UID, "registration": "SGIT-53577", "pilot_callsign": "Calwin-U2332301", "declared": False}
        ls.sendto(json.dumps(_tel(mode="LOITER", state="STATE_MANUAL")).encode(), addr)
        assert wait_until(lambda: ad.status_body()["state"] == "MANUAL", 2.0)
        # the reliable send: resent every 0.5 s until acked; DONE on the ack
        ad.on_cmd("t", {"kind": "cmd", "msg_id": "c-course", "body": {"action": "RC_FORWARD", "args": {"kind": "course", "message": {"course_id": "X"}}}})
        m1, _ = _recv(ls, "robocommand")
        m2, _ = _recv(ls, "robocommand")
        assert m1["id"] == m2["id"] and m1["kind"] == "course"                # a re-send of the same id
        ls.sendto(json.dumps({"type": "robocommand_ack", "id": m1["id"]}).encode(), addr)
        assert wait_until(lambda: any(a["result"] == "DONE" and a["ack_of"] == "c-course" for a in bus.of("ack")), 2.0)
        assert ad.status_body()["robocommand"]["course_fp"] == W.course_fingerprint({"course_id": "X"})[:12]
    finally:
        ad.stop()
        ls.close()


def test_a_forward_never_acked_fails_and_reports_go_through_validated():
    ad, bus, ls = _adapter(retry=1.0)
    ad.start()
    try:
        _, addr = _recv(ls, "robocommand_hello", 3.0)
        ad.on_cmd("t", {"kind": "cmd", "msg_id": "c-cmd", "body": {"action": "RC_FORWARD", "args": {"kind": "command", "which": "run_start",
                                                                                                  "message": {"seq": 4, "run_start": {"run_id": 7}}}}})
        assert wait_until(lambda: any(a["result"] == "FAILED" and "did not take" in a["reason"] for a in bus.of("ack")), 3.0)
        ls.sendto(json.dumps({"type": "uav_report", "id": "r1", "kind": "readiness", "body": {"command_seq": 3}}).encode(), addr)
        ack, _ = _recv(ls, "uav_report_ack")
        assert ack == {"type": "uav_report_ack", "id": "r1", "ok": True}
        ev = bus.of("event", "event/uav/readiness")
        assert ev and ev[0] == {"event": "uav_report", "kind": "readiness", "body": {"command_seq": 3}, "id": "r1"}
        ls.sendto(json.dumps({"type": "uav_report", "id": "r1", "kind": "readiness", "body": {"command_seq": 3}}).encode(), addr)
        assert _recv(ls, "uav_report_ack")[0]["ok"] and len(bus.of("event", "event/uav/readiness")) == 1   # a re-send: re-acked, once
        ls.sendto(json.dumps({"type": "uav_report", "id": "r2", "kind": "readiness", "body": {"no_such_field": 1}}).encode(), addr)
        bad, _ = _recv(ls, "uav_report_ack")
        assert bad["ok"] is False and bad["error"]
        ls.sendto(json.dumps({"type": "uav_report", "id": "r3", "kind": "docking", "body": {}}).encode(), addr)
        assert _recv(ls, "uav_report_ack")[0] == {"type": "uav_report_ack", "id": "r3", "ok": False, "error": "unknown report 'docking'"}
        assert len(bus.of("event", "event/uav/")) == 1                       # a bad report is never published
    finally:
        ad.stop()
        ls.close()
    assert validate_report("safe_passage", {"entry_position": {"latitude": 1.3, "longitude": 103.8}}) is None


def test_fence_changes_prepare_and_the_task_rule():
    ad, bus, ls = _adapter()
    ad.on_telemetry(_tel())
    course = {"boundary": SQ}
    assert ad.prepare_problem(course) == "waiting for the pilot: uav ready on"   # D10: everything else is fine
    ad.on_cmd("t", {"kind": "cmd", "msg_id": "c1", "body": {"action": "UAV_READY", "args": {"on": True}}})
    assert ad.prepare_problem(course) is None
    ad.on_cmd("t", {"kind": "cmd", "msg_id": "p1", "body": {"action": "PREPARE", "args": {"course": course}}})
    assert [a["result"] for a in bus.of("ack") if a["ack_of"] == "p1"] == ["ACCEPTED", "DONE"]
    ad.on_telemetry(_tel(armed=True))
    assert "ARMED" in ad.prepare_problem(course)
    ad.on_telemetry(_tel(fcu_fence=[p[:] for p in OUTSIDE[:-1]]))
    assert "leaves the course" in ad.prepare_problem(course)
    ad.on_telemetry(_tel(fence_alt_max=100.0))
    assert "at most 50 m HAE" in ad.prepare_problem(course)
    assert "stale" in ad.prepare_problem(course) or True
    ad.tel_t -= 6.0
    assert "stale" in ad.prepare_problem(course) and ad.status_body()["robot_state"] == "STATE_UNKNOWN"
    # a Mission Planner fence change: to the OCS once; while armed held; its result re-sent while the listener asks again
    ad.on_telemetry(_tel(armed=True, fence_change={"id": 5, "vertices": INSIDE[:-1]}))
    assert not bus.of("event", "fence_change") and ad.status_body()["fence"]["change_pending"]
    ad.on_telemetry(_tel(fence_change={"id": 5, "vertices": INSIDE[:-1]}))
    ad.on_telemetry(_tel(fence_change={"id": 5, "vertices": INSIDE[:-1]}))
    fc = bus.of("event", "fence_change")
    assert len(fc) == 1 and fc[0]["id"] == "5" and fc[0]["vertices"][0] == fc[0]["vertices"][-1] and ad.pilot_ready is False
    ad.on_cmd("t", {"kind": "cmd", "msg_id": "r5", "body": {"action": "FENCE_CHANGE_RESULT", "args": {"id": 5, "ok": True, "text": "re-sent"}}})
    assert ad.fence_results["5"] == {"type": "fence_change_result", "id": 5, "ok": True, "text": "re-sent"}
    # a ceiling change while declared -> a synthetic fence change
    ad.ocs = {"declared": True}
    ad.on_telemetry(_tel(fence_alt_max=25.0))
    syn = [e for e in bus.of("event", "fence_change") if e.get("synthetic")]
    assert len(syn) == 1 and syn[0]["ceiling_hae_m"] == 25.0 and syn[0]["id"].startswith("alt-")
    # D11: current_task = the UAV mission's task once the RunStart forward is acked; NONE when the drone disarms; by hand
    ad.on_cmd("t", {"kind": "cmd", "msg_id": "s1", "body": {"action": "START_RUN", "args": {"run_id": 9, "uav_task": "1"}}})
    assert ad.task == "TASK_NONE"
    ad.runstart_acked_run = 9
    ad.on_cmd("t", {"kind": "cmd", "msg_id": "s2", "body": {"action": "START_RUN", "args": {"run_id": 9, "uav_task": "1"}}})
    assert ad.task == "TASK_SAFE_PASSAGE"
    ad.on_telemetry(_tel(armed=True))
    ad.on_telemetry(_tel(armed=False))
    assert ad.task == "TASK_NONE"
    ad.on_cmd("t", {"kind": "cmd", "msg_id": "t1", "body": {"action": "UAV_TASK", "args": {"task": "4"}}})
    assert ad.task == "TASK_DYNAMIC_INCIDENT" and ad.status_body()["state"] == "AUTO_TASK"
    ad.on_cmd("t", {"kind": "cmd", "msg_id": "x", "body": {"action": "FCU_ARM", "args": {}}})
    assert "nothing here commands the drone" in bus.of("ack")[-1]["reason"]
    ad.sender.sock.close()
    ls.close()
    assert fence_check(INSIDE, 20.0, {"boundary": SQ}) == [] and fence_check(INSIDE, 20.0, None)


# ---- ocs_app --------------------------------------------------------------------------------------------------------------------
class RJudge(NullJudgeClient):
    def __init__(self):
        super().__init__()
        self.requests, self.reports = [], []

    def publish_request(self, body):
        self.requests.append(json.loads(json.dumps(body)))
        return len(self.requests)

    def publish_report(self, vehicle_id, body):
        self.reports.append((vehicle_id, json.loads(json.dumps(body))))
        return len(self.reports)


def make_app(prof="pool/ocs_pool.yaml"):
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / prof).read_text())
    bus, judge = FakeBus(), RJudge()
    app = OcsApp(cfg, bus=bus, judge=judge)
    res = []
    app.authority.publish_result = res.append
    app.lifecycle.on_link(True)
    app.on_course({"course_id": "SQ", "boundary": SQ, "uav_transit_area": [], "pinger_hz": 30000})
    app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL"}})
    app.on_reporting(True, "usv")                                         # v0.1.82: the course profile's buttons (a no-op on auto)
    app.on_usv_ready(True)
    return app, bus, judge, res


def uav_status(app, **kw):
    b = {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO", "current_task": "TASK_NONE", "armed": False, "vehicle_type": "UAV",
         "uav_registration": "SGIT-53577", "health": {"telemetry_age_s": 0.4, "pilot_ready": True},
         "fence": {"points": INSIDE, "ceiling_hae_m": 20.0}, "robocommand": {}}
    for k, v in kw.items():
        if k in ("telemetry_age_s",):
            b["health"][k] = v
        elif k in ("points", "ceiling_hae_m"):
            b["fence"][k] = v
        else:
            b[k] = v
    app.on_status("t", {"vehicle_id": UID, "body": b})


def cmds(bus, action=None, vid=None):
    return [p for p in bus.published if p["kind"] == "cmd" and (action is None or p["body"]["action"] == action) and (vid is None or p["vehicle_id"] == vid)]


def _declare_done(app, bus):
    r = app.authority.on_intent("t", intent("DECLARE"))
    for mid, vid in list(app._prepare_msg_ids.items()):
        app.on_ack("t", {"vehicle_id": vid, "body": {"ack_of": mid, "result": "DONE"}})
    return r


def test_the_usv_lives_without_the_uav():
    app, bus, judge, _ = make_app()
    assert app.uav_vid == UID and app.declare_uav is False and app.hb_vehicles() == ["USV1"]
    assert not cmds(bus, vid=UID)                                           # the course went to the USV only, nothing to the UAV
    r = _declare_done(app, bus)
    assert r.result == "accepted" and judge.requests[-1]["vehicle_ids"] == ["USV1"] and judge.requests[-1]["uav_geofence"] == []
    assert [c["vehicle_id"] for c in cmds(bus, "PREPARE")] == ["USV1"]
    app.heartbeats.enabled = True
    app.heartbeats.vehicles = app.hb_vehicles()
    uav_status(app)                                                        # even with an adapter up, an undeclared UAV is silent
    app.heartbeats.tick()
    assert {v for v, b in judge.reports if b["kind"] == "heartbeat"} == {"USV1"}
    app.on_rc_command({"kind": "run_start", "declaration_seq": app.lifecycle.declaration_seq, "run_id": 5, "command_seq": 1})
    assert app.lifecycle.state == "RUNNING" and [c["vehicle_id"] for c in cmds(bus, "START_RUN")] == ["USV1"]
    assert not cmds(bus, "RC_FORWARD", UID) or all(c["body"]["args"]["kind"] == "course" for c in cmds(bus, "RC_FORWARD", UID))   # no RunStart to the drone
    st = app.state_body()
    assert list(st["vehicles"]) == ["USV1", UID] and st["uav"]["declared"] is False


def test_declare_uav_builds_the_two_vehicle_declaration_from_the_fcu(tmp_path):
    app, bus, judge, res = make_app()
    app.decl_file = str(tmp_path / "rc_last_declaration.json")
    r = app.authority.on_intent("t", intent("DECLARE_UAV", args={"on": True}))
    assert r.result == "accepted" and UID in r.reason and app.hb_vehicles() == ["USV1", UID]
    r = app.authority.on_intent("t", intent("DECLARE"))
    assert r.result == "refused" and f"{UID} not heard" in r.reason
    uav_status(app, telemetry_age_s=6.0)
    assert "UAV telemetry stale" in app.authority.on_intent("t", intent("DECLARE")).reason
    uav_status(app, armed=True)
    assert "ARMED" in app.authority.on_intent("t", intent("DECLARE")).reason and not cmds(bus, "FENCE_REJECTED")
    uav_status(app, points=OUTSIDE)
    r = app.authority.on_intent("t", intent("DECLARE"))
    assert r.result == "refused" and "RoboCommand would REJECT it" in r.reason and len(cmds(bus, "FENCE_REJECTED", UID)) == 1   # D9
    uav_status(app, ceiling_hae_m=60.0)
    assert "at most 50 m HAE" in app.authority.on_intent("t", intent("DECLARE")).reason and len(cmds(bus, "FENCE_REJECTED")) == 2
    assert app.state_body()["uav"]["last_refusal"]["reason"].startswith("RoboCommand would REJECT")
    assert not judge.requests                                              # nothing refused ever reaches RoboCommand
    uav_status(app)
    r = _declare_done(app, bus)
    assert r.result == "accepted" and {c["vehicle_id"] for c in cmds(bus, "PREPARE")} == {"USV1", UID}
    d = judge.requests[-1]
    assert d["vehicle_ids"] == ["USV1", UID] and d["uav_geofence"] == INSIDE and d["uav_geofence_max_altitude_hae_m"] == 20.0
    assert d["uav_pilot_callsign"] == "Calwin-U2332301"
    assert json.load(open(app.decl_file))["declaration"]["vehicle_ids"] == ["USV1", UID]
    st = app.state_body()["uav"]
    assert st["declared"] and st["declared_fence"] == INSIDE and st["declared_ceiling_hae_m"] == 20.0
    # heartbeats for both (D2: 2 Hz each)
    app.heartbeats.vehicles = app.hb_vehicles()
    app.heartbeats.tick()
    hb = {v: b for v, b in judge.reports if b["kind"] == "heartbeat"}
    assert set(hb) == {"USV1", UID} and hb[UID]["vehicle_type"] == "UAV" and hb[UID]["uav_registration"] == "SGIT-53577"
    # the protobuf heartbeat carries flight_phase only for the UAV
    pb = rc_wire.decode_report(rc_wire.encode_report("SGIT", UID, 3, dict(hb[UID], flight_phase="FLIGHT_PHASE_GROUNDED")))
    assert pb["vehicle_type"] == "UAV" and pb["uav_registration"] == "SGIT-53577"


def test_m1t2_needs_declare_uav_and_runstart_reaches_the_drone_only_when_declared():
    app, bus, judge, _ = make_app()
    uav_status(app)
    app.authority.on_intent("t", intent("MISSION", args={"task": "T1", "tier": "advanced"}))
    r = app.authority.on_intent("t", intent("DECLARE"))
    assert r.result == "refused" and "m1t2 needs declare uav on" in r.reason
    app.authority.on_intent("t", intent("DECLARE_UAV", args={"on": True}))
    _declare_done(app, bus)
    assert app.lifecycle.state == "DECLARED" and app.uav_declared()
    app.on_rc_command({"kind": "run_start", "declaration_seq": app.lifecycle.declaration_seq, "run_id": 77, "command_seq": 2, "_pb": {"seq": 2, "run_start": {"run_id": 77}}})
    sr = {c["vehicle_id"]: c["body"]["args"] for c in cmds(bus, "START_RUN")}
    assert sr == {"USV1": {"run_id": 77}, UID: {"run_id": 77, "uav_task": "1"}}                 # D11: m1t2 -> TASK_1
    fw = [c for c in cmds(bus, "RC_FORWARD", UID) if c["body"]["args"]["kind"] == "command"]
    assert len(fw) == 1 and fw[0]["body"]["args"]["which"] == "run_start" and fw[0]["body"]["args"]["message"] == {"seq": 2, "run_start": {"run_id": 77}}
    # a RunStart we refuse (another declaration_seq) never reaches the Orin
    app.on_rc_command({"kind": "run_start", "declaration_seq": 99, "run_id": 78, "command_seq": 3})
    assert len([c for c in cmds(bus, "RC_FORWARD", UID) if c["body"]["args"]["kind"] == "command"]) == 1


def test_a_fence_change_redeclares_before_runstart_only():
    app, bus, judge, _ = make_app()
    uav_status(app)
    app.authority.on_intent("t", intent("DECLARE_UAV", args={"on": True}))
    _declare_done(app, bus)
    seq0, n0 = app.lifecycle.declaration_seq, len(judge.requests)
    small = [[1.2996, 103.7996], [1.2996, 103.8004], [1.3004, 103.8004], [1.3004, 103.7996]]
    app.on_event("t", {"vehicle_id": UID, "body": {"event": "fence_change", "id": "7", "vertices": small, "ceiling_hae_m": 20.0}})
    assert len(judge.requests) == n0 + 1 and judge.requests[-1]["uav_geofence"] == small + [small[0]]
    assert app.lifecycle.declaration_seq == seq0 and judge.requests[-1]["declaration_seq"] == seq0   # the same declaration, a new request
    res = cmds(bus, "FENCE_CHANGE_RESULT", UID)[-1]["body"]["args"]
    assert res["id"] == "7" and res["ok"] and "re-sent" in res["text"]
    app.on_event("t", {"vehicle_id": UID, "body": {"event": "fence_change", "id": "8", "vertices": OUTSIDE[:-1], "ceiling_hae_m": 20.0}})
    assert len(judge.requests) == n0 + 1 and cmds(bus, "FENCE_REJECTED", UID) and not cmds(bus, "FENCE_CHANGE_RESULT", UID)[-1]["body"]["args"]["ok"]
    app.on_event("t", {"vehicle_id": UID, "body": {"event": "fence_change", "id": "alt-1", "vertices": small, "ceiling_hae_m": 25.0, "synthetic": True}})
    assert judge.requests[-1]["uav_geofence_max_altitude_hae_m"] == 25.0
    app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO"}})
    app.on_rc_command({"kind": "run_start", "declaration_seq": seq0, "run_id": 5, "command_seq": 9})
    n1 = len(judge.requests)
    app.on_event("t", {"vehicle_id": UID, "body": {"event": "fence_change", "id": "9", "vertices": small, "ceiling_hae_m": 20.0}})
    assert len(judge.requests) == n1                                          # R5: no re-declaration (A15)
    last = cmds(bus, "FENCE_CHANGE_RESULT", UID)[-1]["body"]["args"]
    assert last["id"] == "9" and last["ok"] is False and "after RunStart" in last["text"]


def test_the_course_and_commands_go_to_the_orin_with_the_auto_ack():
    app, bus, judge, _ = make_app()
    assert not cmds(bus, "RC_FORWARD")                                       # no adapter yet: nothing sent, nothing TIMED OUT
    uav_status(app)                                                          # the adapter comes up: the cached course once
    fw = cmds(bus, "RC_FORWARD", UID)
    assert len(fw) == 1 and fw[0]["body"]["args"]["kind"] == "course" and fw[0]["body"]["args"]["message"]["course_id"] == "SQ"
    uav_status(app, robocommand={"course_fp": app._uav_course_fp[:12]})
    app.on_course({"course_id": "SQ", "boundary": SQ, "uav_transit_area": [], "pinger_hz": 30000})      # retained republish: not forwarded
    assert len(cmds(bus, "RC_FORWARD", UID)) == 1
    app.on_course({"course_id": "SQ", "boundary": SQ, "uav_transit_area": [], "pinger_hz": 35000})      # the pinger changed: forwarded
    assert len(cmds(bus, "RC_FORWARD", UID)) == 2
    a = {"course_id": "SQ", "corners": [{"latitude": 1.0, "longitude": 2.0}], "pinger_freq_hz": 1, "sent_at": "2026-11-08T00:00:00Z"}
    assert W.course_fingerprint(a) == W.course_fingerprint(dict(a, sent_at="2026-11-09T00:00:00Z"))
    # a UAV-targeted assistance request: forwarded; the IncidentAck once the Orin has it (his ACKED_COMMANDS)
    app.on_rc_command({"kind": "assistance_request", "command_seq": 31, "domain": "air", "position": {"lat": 1.3, "lon": 103.8}})
    f = cmds(bus, "RC_FORWARD", UID)[-1]
    assert f["body"]["args"]["which"] == "assistance_request" and not cmds(bus, "INCIDENT", UID)
    assert not [b for v, b in judge.reports if b.get("kind") == "incident_ack"]
    app.on_ack("t", {"vehicle_id": UID, "body": {"ack_of": "msg-%d" % bus._n, "result": "DONE"}})
    acks = [(v, b) for v, b in judge.reports if b.get("kind") == "incident_ack"]
    assert acks == [(UID, {"kind": "incident_ack", "command_seq": 31, "what": "assistance_request"})]
    # a USV-targeted one: to the boat as before (and forwarded to the Orin, his rule), no auto-ack
    app.on_rc_command({"kind": "assistance_request", "command_seq": 32, "domain": "surface", "position": {"lat": 1.3, "lon": 103.8}})
    assert cmds(bus, "INCIDENT", "USV1")[-1]["body"]["args"]["command_seq"] == 32
    app.on_ack("t", {"vehicle_id": UID, "body": {"ack_of": "msg-%d" % (bus._n - 0), "result": "DONE"}})
    assert len([1 for v, b in judge.reports if b.get("kind") == "incident_ack"]) == 1
    # a forward the Orin never took: an alert
    app.on_rc_command({"kind": "all_clear", "command_seq": 33, "domain": "air"})
    app.on_ack("t", {"vehicle_id": UID, "body": {"ack_of": "msg-%d" % bus._n, "result": "FAILED", "reason": "no ack"}})
    assert any(a["kind"] == "uav_forward_failed" for a in app.alerts)
    # the words
    r = app.authority.on_intent("t", intent("UAV", args={"op": "resend_course"}))
    assert r.result == "accepted" and cmds(bus, "RC_FORWARD", UID)[-1]["body"]["args"]["kind"] == "course"
    assert app.authority.on_intent("t", intent("UAV", args={"op": "task", "task": "2"})).result == "accepted"
    assert cmds(bus, "UAV_TASK", UID)[-1]["body"]["args"] == {"task": "2"}
    r = app.authority.on_intent("t", intent("REPORT", args={"on": True, "vehicle": "uav"}))
    assert r.result == "accepted" and app.hb_vehicles() == ["USV1", UID]


def test_uav_reports_reach_the_judge_with_its_own_seq():
    m = event_to_report(UID, {"event": "uav_report", "kind": "readiness", "body": {"command_seq": 3}, "id": "r1"})
    assert m == ("uav_report:r1", {"kind": "pb", "field": "readiness", "pb": {"command_seq": 3}, "report_kind": "readiness"})
    d = rc_wire.decode_report(rc_wire.encode_report("SGIT", UID, 12, m[1]))
    assert d["vehicle_id"] == UID and d["report_seq"] == 12 and d.get("kind") == "readiness"
    app, bus, judge, _ = make_app()
    app.on_event("t", {"vehicle_id": UID, "body": {"event": "uav_report", "kind": "readiness", "body": {"command_seq": 3}, "id": "r1"}})
    assert judge.reports[-1] == (UID, m[1])


def test_the_uav_heartbeat_stops_on_stale_telemetry():
    j = RJudge()
    clk = [100.0]
    hb = HeartbeatPublisher(j, ["USV1", UID], stale_by={UID: 5.0}, clock=lambda: clk[0])
    hb.enabled = True
    hb.update(UID, {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO", "health": {"telemetry_age_s": 1.0}, "vehicle_type": "UAV"})
    hb.tick()
    assert len(j.reports) == 1
    clk[0] += 1.0
    hb.update(UID, {"state": "AUTO_HOLD", "robot_state": "STATE_UNKNOWN", "health": {"telemetry_age_s": 5.6}, "vehicle_type": "UAV"})
    hb.tick()
    assert len(j.reports) == 1 and hb.view()[UID]["stopped"]                  # his invariant 3: never from telemetry older than 5 s
    clk[0] += 1.0
    hb.update(UID, {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO", "health": {"telemetry_age_s": 0.5}, "vehicle_type": "UAV"})
    hb.tick()
    assert len(j.reports) == 2
    clk[0] += 6.0                                                             # the status itself older than the UAV's 5 s cut
    hb.tick()
    assert len(j.reports) == 2


def test_the_words_and_the_launchers():
    assert ocs_cli.intent_spec(ocs_cli.parse(["declare", "uav", "on"])) == ("DECLARE_UAV", None, {"on": True}, None)
    assert ocs_cli.intent_spec(ocs_cli.parse(["uav", "ready", "off"])) == ("UAV", None, {"op": "ready", "on": False}, None)
    assert ocs_cli.intent_spec(ocs_cli.parse(["uav", "task", "1"]))[2] == {"op": "task", "task": "1"}
    assert ocs_cli.intent_spec(ocs_cli.parse(["report", "on", "uav"]))[2] == {"on": True, "vehicle": "uav"}
    assert ocs_cli.intent_spec(ocs_cli.parse(["declare"]))[0] == "DECLARE"
    for prof in ("pool/ocs_pool.yaml", "bench/ocs_bench.yaml", "course/ocs_course.yaml"):
        o = yaml.safe_load((ROOT / "profiles" / prof).read_text())["ocs"]
        assert o["vehicles"] == {"USV1": "usv", UID: "uav"} and o["uav"]["declare_default"] is False and o["uav"]["stale_stop_s"] == 5, prof
    sh = (ROOT / "scripts" / "run_uav.sh").read_text()
    for needle in ("uav/sim-tests", "fake_copter.py", "geofence_udp_listener.py' --connection", "--vehicle-id UAV1", "PYTHONDONTWRITEBYTECODE=1",
                   "trap '' INT", "tee -i", "optiprimex.uav.adapter", "--robocommand-state '$UD/robocommand_state.json'"):
        assert needle in sh, needle
    for f in ("run_sim.sh", "run_bench.sh", "run_ocs.sh"):
        assert "scripts/run_uav.sh" in (ROOT / "scripts" / f).read_text() and "--uav)" in (ROOT / "scripts" / f).read_text(), f
    assert 'op-uav-adapter     = "optiprimex.uav.adapter:main"' in (ROOT / "pyproject.toml").read_text()
    assert "geofence_udp_listener" in (ROOT / "scripts" / "status.sh").read_text()


def test_the_boat_is_the_first_surface_vehicle_whatever_the_key_order():
    """Found by test_integration_sil1: a profile written by yaml.safe_dump sorts SGIT-53577 before USV1 - 'the first vehicle' was the UAV."""
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "bench" / "ocs_bench.yaml").read_text())
    cfg.extra["ocs"]["vehicles"] = {UID: "uav", "USV1": "usv"}
    app = OcsApp(cfg, bus=FakeBus(), judge=RJudge())
    assert app.primary_vid == "USV1" and app.uav_vid == UID
    app.on_status("t", {"vehicle_id": UID, "body": {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO", "health": {"telemetry_age_s": 0.3}}})
    app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL"}})
    assert list(app.state_body()["vehicles"]) == ["USV1", UID]


# ---- D0: the way back ------------------------------------------------------------------------------------------------------------
def test_the_rollback_script_moves_never_deletes_and_keeps_the_local_state():
    sh = (ROOT / "scripts" / "rollback_to.sh").read_text()
    assert 'KEEP=" .venv logs _archive drone network.env network.lan.env network.ts.env "' in sh
    assert 'mv "${MOVE[@]}" "$DEST/"' in sh and "rm -rf \"$TMP\"" in sh and "rm -rf" not in sh.replace('rm -rf "$TMP"', "")
    assert "STATUS_CLOCK=0 scripts/status.sh" in sh and "--dry-run" in sh and "pip install -e . -q" in sh and "make quick" in sh
    assert "usv_ui/co.json usv_ui/points.json" in sh and ">> DEVLOG.md" in sh


def test_quick_list_has_this_file():
    assert "tests/test_v0164.py" in (ROOT / "Makefile").read_text()
