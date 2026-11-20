"""The real RoboCommand wire (ROADMAP WP4): RoboNation's RobotX 2026 protobuf messages (robonation/robocommand @ 8fc09f2,
vendored in optiprimex/ocs/rc_pb) through ocs/rc_wire, ProtobufJudgeClient and the fake judge's `--wire protobuf`.

Unit: every body round-trips; topics and team filtering. Integration: the client against the protobuf fake (course, the
declaration's RunStart mapped back to our declaration_seq, heartbeats counted, the ReadinessConfirm mapped back to its
command); SIL: ocs_app with `judge: protobuf` + the mission manager + the fake FCU run the whole T4 chain on the real wire,
and a raw subscriber sees only valid protobuf on robocommand/robotx/#.
"""
from __future__ import annotations

import json
import threading
import time

import paho.mqtt.client as mqtt
import pytest

from optiprimex.ocs import rc_wire as W
from optiprimex.ocs.judge_client import ProtobufJudgeClient

from .conftest import PY, Probe, Proc, make_ocs_config, make_usv_config, wait_until

TEAM = "SGIT"
NS = "team/optiprime/v1"


def test_wire_round_trips_and_topics():
    hb = {"kind": "heartbeat", "state": "STATE_AUTO", "current_task": "TASK_SAFE_PASSAGE", "lat": 1.3, "lon": 103.8, "spd_mps": 0.4,
          "heading_deg": 45.0, "vehicle_type": "USV"}
    d = W.decode_report(W.encode_report(TEAM, "USV1", 3, hb))
    assert (d["kind"], d["state"], d["current_task"], d["report_seq"], round(d["heading_deg"])) == ("heartbeat", "STATE_AUTO", "TASK_SAFE_PASSAGE", 3, 45)
    sp = {"kind": "safe_passage", "entry_position": {"lat": 1.3, "lon": 103.8}, "exit_position": None,
          "buoys": [{"colour": "red", "state": "flashing", "lat": 1.3, "lon": 103.8}, {"colour": "blue", "state": "solid", "lat": 1, "lon": 2},
                    {"colour": "black", "state": "off", "lat": 1, "lon": 2}, {"colour": "red", "state": "solid", "lat": 1, "lon": 2}]}
    d = W.decode_report(W.encode_report(TEAM, "USV1", 4, sp))
    assert [b["beacon"] for b in d["buoys"]] == ["BEACON_STATE_FLASHING_RED", "BEACON_STATE_STEADY_BLUE", "BEACON_STATE_OFF", "BEACON_STATE_UNKNOWN"]
    assert d["exit_position"] is None and d["entry_position"] == {"lat": 1.3, "lon": 103.8}
    assert W.decode_report(W.encode_report(TEAM, "USV1", 5, {"kind": "docking", "bay": 2}))["bay"] == 2
    assert W.decode_report(W.encode_report(TEAM, "USV1", 6, {"kind": "firefighting", "window_id": 1}))["window_id"] == 1
    decl = {"kind": "run_declaration", "vehicle_ids": ["USV1"], "tiers": {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_CORE"}}
    r = W.decode_request(W.encode_request(TEAM, 9, decl))
    assert r["request_seq"] == 9 and r["tiers"]["T2"] == "TIER_NONE" and r["vehicle_ids"] == ["USV1"]
    c = W.decode_command(W.encode_command(TEAM, 11, {"kind": "assistance_request", "position": {"lat": 1.3, "lon": 103.8}, "domain": "air"}))
    assert c["kind"] == "assistance_request" and c["domain"] == "air" and c["command_seq"] == 11
    c = W.decode_command(W.encode_command(TEAM, 12, {"kind": "readiness_confirm", "report_seq": 7, "vehicle_id": "USV1"}))
    assert (c["report_seq"], c["vehicle_id"]) == (7, "USV1")
    crs = W.decode_course(W.encode_course({"course_id": "A", "boundary": [[1.0, 2.0], [1.0, 3.0], [2.0, 3.0]], "pinger_hz": 30000}))
    assert crs["boundary"][0] == crs["boundary"][-1] and crs["pinger_hz"] == 30000            # closed ring (3.4.9)
    assert W.t_report(TEAM, "USV1") == "robocommand/robotx/SGIT/USV1/report" and W.t_command(TEAM) == "robocommand/robotx/SGIT/command"
    assert W.classify("robocommand/robotx/OTHER/command", TEAM) is None and W.classify("robocommand/robotx/course", TEAM) == ("course", None)
    with pytest.raises(ValueError):
        W.encode_report(TEAM, "USV1", 1, {"kind": "bogus"})


@pytest.fixture(scope="module")
def pb_judge(brokers, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("pb")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--wire", "protobuf", "--start-delay", "0.5",
               "--confirm-delay", "0.3", "--auto-incident-after", "2", "--log", str(tmp / "rc.jsonl")], tmp / "rc.log")
    yield {"rc": rc, "tmp": tmp, "brokers": brokers}
    rc.kill()


def test_client_against_the_protobuf_fake(pb_judge):
    port = pb_judge["brokers"].hub_port
    got = {"course": [], "cmd": []}
    j = ProtobufJudgeClient("127.0.0.1", port, TEAM, network="bench", client_id="t-pb-client")
    j.on_course(got["course"].append)
    j.on_command(got["cmd"].append)
    raw = []
    sniff = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-pb-sniff", protocol=mqtt.MQTTv311)
    sniff.on_message = lambda c, u, m: raw.append((m.topic, m.payload))
    sniff.connect("127.0.0.1", port)
    sniff.subscribe("robocommand/robotx/#", 1)
    sniff.loop_start()
    j.start()
    try:
        assert j.wait_connected(5)
        assert wait_until(lambda: got["course"], 5) and got["course"][0]["course_id"] == "BENCH-COURSE-1"
        assert got["course"][0]["boundary"][0] != got["course"][0]["boundary"][-1]                 # handed to the OCS as an open ring
        # our declaration_seq 41 travels as RxRequest.seq 1; RunStart comes back with 1 and the client maps it to 41
        seq = j.publish_request({"kind": "run_declaration", "declaration_seq": 41, "vehicle_ids": ["USV1"],
                                 "tiers": {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_CORE"}, "uav_geofence": []})
        assert seq == 1
        assert wait_until(lambda: any(c["kind"] == "run_start" for c in got["cmd"]), 5), got["cmd"]
        rs = [c for c in got["cmd"] if c["kind"] == "run_start"][0]
        assert rs["declaration_seq"] == 41 and rs["request_seq"] == 1 and rs["run_id"] == 1001
        for _ in range(3):
            assert j.publish_report("USV1", {"kind": "heartbeat", "state": "STATE_AUTO", "current_task": "TASK_DYNAMIC_INCIDENT", "lat": 1.3,
                                             "lon": 103.8, "spd_mps": 0.0, "heading_deg": 0.0, "vehicle_type": "USV"}) is not None
        assert wait_until(lambda: any(c["kind"] == "assistance_request" for c in got["cmd"]), 6)
        ar = [c for c in got["cmd"] if c["kind"] == "assistance_request"][0]
        assert ar["domain"] == "surface" and abs(ar["position"]["lat"] - 1.28113) < 1e-9
        j.publish_report("USV1", {"kind": "incident_ack", "command_seq": ar["command_seq"]})
        rseq = j.publish_report("USV1", {"kind": "readiness", "command_seq": ar["command_seq"], "position": {"lat": 1.3, "lon": 103.8}})
        assert wait_until(lambda: any(c["kind"] == "readiness_confirm" for c in got["cmd"]), 5)
        rc_ = [c for c in got["cmd"] if c["kind"] == "readiness_confirm"][0]
        assert rc_["report_seq"] == rseq and rc_["vehicle_id"] == "USV1" and rc_["ref_command_seq"] == ar["command_seq"]
        stats = {}
        stats_c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-pb-stats", protocol=mqtt.MQTTv311)
        stats_c.on_message = lambda c, u, m: stats.update(json.loads(m.payload))
        stats_c.connect("127.0.0.1", port)
        stats_c.subscribe("rc/sim/stats", 0)
        stats_c.loop_start()
        assert wait_until(lambda: stats.get("heartbeats", {}).get("USV1", 0) >= 3 and stats.get("incident_acks", 0) >= 1, 5), stats
        stats_c.loop_stop(); stats_c.disconnect()
        assert stats["declarations"] == 1 and stats["rate_violations"] == 0 and "TASK_DYNAMIC_INCIDENT" in stats["tasks_seen"]["USV1"]
        # everything on robocommand/robotx/# decodes as RoboNation protobuf (no JSON leaks onto the real topics)
        for topic, payload in raw:
            assert W.decode(topic, payload) is not None, topic
        assert {W.classify(t)[0] for t, _ in raw} >= {"course", "request", "report", "command"}
        # another team's command is ignored
        n = len(got["cmd"])
        sniff.publish(W.t_command("OTHER"), W.encode_command("OTHER", 99, {"kind": "all_clear"}), qos=1).wait_for_publish(2)
        time.sleep(0.5)
        assert len(got["cmd"]) == n
    finally:
        j.stop()
        sniff.loop_stop(); sniff.disconnect()


def test_t4_chain_on_the_real_wire(pb_judge, brokers, fake_fcu, tmp_path_factory):
    """ocs_app with judge: protobuf -> the whole Task 4 core chain, as SIL-1 does on the JSON bench wire."""
    from optiprimex.tools.ocs_cli import OcsConsole
    tmp = tmp_path_factory.mktemp("pbsil")
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(make_usv_config(tmp, brokers, fake_fcu))], tmp / "mm.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#", "rc/sim/stats"]).start()
    rc = pb_judge["rc"]                                    # the module's protobuf judge (one judge per broker: the course is retained)
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(make_ocs_config(tmp, brokers, judge="protobuf", judge_network="bench"))], tmp / "ocs.log")
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    try:
        assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2"
                          and (m["env"]["body"].get("course") or {}).get("course_id", "BENCH-COURSE-1") == "BENCH-COURSE-1", 20, include_past=True), ocs.log()[-1500:]
        assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15, include_past=True)
        assert con.intent("DECLARE")["result"]["result"] == "accepted"
        assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
        assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 20), ocs.log()[-2000:]
        assert probe.wait(lambda m: m["topic"].endswith("event/usv/incident") and m["env"]["body"]["kind"] == "readiness", 60), mm.log()[-1500:]
        assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                          and m["env"]["body"]["current_task"] == "TASK_NONE", 30), mm.log()[-1500:]

        def stats():
            m = probe.last("rc/sim/stats")
            return json.loads(m["payload"]) if m else {}
        # v0.1.33: wait for this test's own readiness in the 1 Hz stats (confirms >= 1 was already true from the test before)
        assert wait_until(lambda: stats().get("readiness", 0) >= 2 and stats().get("incident_acks", 0) >= 1, 10), stats()
        s = stats()
        assert s["run_starts"] >= 2 and s["readiness"] >= 2 and s["rate_violations"] == 0 and s["heartbeats"]["USV1"] >= 5
        assert "TASK_DYNAMIC_INCIDENT" in s["tasks_seen"]["USV1"] and "STATE_AUTO" in s["states_seen"]["USV1"]
        # found live against rc-test: two judges' tools number from 1 - a command with a seq already seen but another kind / send
        # time is NOT a duplicate (an AllClear #777 then an AssistanceRequest #777 -> T4 still starts)
        pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-pb-dup", protocol=mqtt.MQTTv311)
        pub.connect("127.0.0.1", brokers.hub_port)
        pub.loop_start()
        pub.publish(W.t_command(TEAM), W.encode_command(TEAM, 777, {"kind": "all_clear"}), qos=1).wait_for_publish(2)
        time.sleep(0.5)
        pub.publish(W.t_command(TEAM), W.encode_command(TEAM, 777, {"kind": "assistance_request", "position": {"lat": 1.3001, "lon": 103.8001}}),
                    qos=1).wait_for_publish(2)
        pub.loop_stop(); pub.disconnect()
        assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("task_name") == "T4", 10), ocs.log()[-1500:]
    finally:
        con.stop(); probe.stop()
        for p in (ocs, mm):
            p.kill()


def test_rc_tui_renders_both_wires_200x60_and_commands():
    """v0.1.26: one model decodes both wires by topic root (rc/# JSON + robocommand/robotx/# protobuf); the expanded heartbeat, the
    task timeline and the latest report per kind are on screen; commands through the fake judge / as judge / refused in monitor."""
    from optiprimex.tools import rc_tui
    m = rc_tui.RcModel(TEAM)
    m.on_wire(W.t_course(), W.encode_course({"course_id": "ALPHA", "boundary": [[1.28, 103.85], [1.28, 103.86], [1.29, 103.86]], "pinger_hz": 25000}))
    m.on_wire(W.t_request(TEAM), W.encode_request(TEAM, 1, {"kind": "run_declaration", "vehicle_ids": ["USV1"],
                                                            "tiers": {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_CORE"}}))
    hb = {"kind": "heartbeat", "state": "STATE_AUTO", "current_task": "TASK_NONE", "lat": 1.3, "lon": 103.8, "spd_mps": 0.4,
          "heading_deg": 91.0, "roll_deg": 2.5, "pitch_deg": -1.5, "vehicle_type": "USV"}
    m.on_wire(W.t_report(TEAM, "USV1"), W.encode_report(TEAM, "USV1", 1, hb))
    m.on_wire(W.t_report(TEAM, "USV1"), W.encode_report(TEAM, "USV1", 2, dict(hb, current_task="TASK_SAFE_PASSAGE")))
    m.on_wire(W.t_report(TEAM, "USV1"), W.encode_report(TEAM, "USV1", 3, {"kind": "safe_passage", "entry_position": {"lat": 1.3, "lon": 103.8},
                                                                          "buoys": [{"colour": "red", "state": "flashing", "lat": 1.3, "lon": 103.8},
                                                                                    {"colour": "black", "state": "off", "lat": 1.3, "lon": 103.8}]}))
    m.on_wire(W.t_report(TEAM, "USV1"), W.encode_report(TEAM, "USV1", 4, {"kind": "readiness", "command_seq": 3}))
    m.on_wire(W.t_command(TEAM), W.encode_command(TEAM, 2, {"kind": "run_start", "declaration_seq": 1, "run_id": 1001}))
    m.on_wire(W.t_command("OTHER"), W.encode_command("OTHER", 2, {"kind": "all_clear"}))            # another team: not shown
    m.on_wire(W.t_report(TEAM, "USV1"), b"not protobuf")
    # the bench JSON wire on the same model: the fake judge's topics
    m.on_wire(f"rc/report/{TEAM}/UAV1", json.dumps({"kind": "heartbeat", "team_id": TEAM, "report_seq": 7, "state": "STATE_MANUAL",
                                                     "current_task": "TASK_NONE", "lat": 1.31, "lon": 103.81, "vehicle_type": "UAV",
                                                     "flight_phase": "FLIGHT_PHASE_GROUNDED"}).encode())
    m.on_wire(f"rc/report/{TEAM}/USV1", json.dumps({"kind": "docking", "team_id": TEAM, "report_seq": 9, "bay": 2}).encode())
    m.on_wire(f"rc/command/{TEAM}", json.dumps({"kind": "assistance_request", "team_id": TEAM, "command_seq": 5,
                                                 "position": {"lat": 1.3, "lon": 103.8}, "domain": "surface"}).encode())
    m.on_wire("rc/sim/stats", b"{}")                                                                # the fake judge's control: ignored
    assert m.wires_seen["protobuf"] >= 6 and m.wires_seen["json"] == 3
    text = "\n".join(rc_tui.render(m, 200, 60, "assist", "").lines())
    assert "JUDGE RECEIVES" in text and "JUDGE SENDS" in text and "course: ALPHA" in text and "run: RunStart run 1001 for declaration 1" in text
    assert "declaration seq 1: USV1" in text and "USV1  AUTO    SAFE_PASSAGE" in text and "UAV1  MANUAL  NONE" in text and "GROUNDED" in text
    assert "roll 2.5 pitch -1.5" in text and "hdg 91" in text                                       # the expanded heartbeat
    assert "tasks: NONE" in text and "SAFE_PASSAGE" in text                                         # the timeline
    assert "last SafePassageReport: entry yes exit -, 2 buoys flashing_red=1 off=1" in text and "last DockingReport bay 2" in text
    assert "ReadinessReport for command #3" in text and "AssistanceRequest surface" in text
    assert "AllClear" not in text and "undecodable 1" in text and "> assist" in text and "json 3 protobuf" in text
    snap = m.snapshot()
    assert snap["vehicles"]["USV1"]["heartbeat"]["roll_deg"] == 2.5 and [x["task"] for x in snap["vehicles"]["USV1"]["timeline"]] == ["TASK_NONE", "TASK_SAFE_PASSAGE"]
    assert set(snap["vehicles"]["USV1"]["reports"]) == {"safe_passage", "readiness", "docking"} and snap["run_start"]["run_id"] == 1001
    json.dumps(snap)                                                                                # JSON-ready for the web page

    class Cap:
        def __init__(self):
            self.pubs = []

        def publish(self, topic, payload, qos=0, retain=False):
            self.pubs.append((topic, payload, retain))
    t = rc_tui.RcTui(rc_tui.parse_args(["--team", TEAM]))
    t.m, t.c = m, Cap()
    assert t.do("assist 1.31 103.81 air").startswith("asked the fake judge")                          # default: via rc/sim/inject
    topic, payload, _ = t.c.pubs[-1]
    assert topic == "rc/sim/inject" and json.loads(payload) == {"kind": "assistance_request", "position": {"lat": 1.31, "lon": 103.81}, "domain": "air"}
    assert t.do("confirm").startswith("asked") and json.loads(t.c.pubs[-1][1])["report_seq"] == 4
    t.mode = "judge"
    assert "RxCommand #9001 run_start" in t.do("start")
    d = W.decode_command(t.c.pubs[-1][1])
    assert t.c.pubs[-1][0] == W.t_command(TEAM) and d["declaration_seq"] == 1
    assert "retained" in t.do("course") and t.c.pubs[-1][2] is True
    t.a.wire = "json"
    assert "(json)" in t.do("allclear") and t.c.pubs[-1][0] == f"rc/command/{TEAM}"                  # --as-judge --wire json
    t.mode = "monitor"
    n = len(t.c.pubs)
    assert t.do("allclear").startswith("refused") and len(t.c.pubs) == n


def test_judge_web_serves_the_same_view(pb_judge, tmp_path):
    """v0.1.26: judge_web subscribes to both wires and serves the page and /state.json (Dmitrii's request: the TUI in the browser)."""
    import urllib.request
    from .conftest import free_port
    port, web = pb_judge["brokers"].hub_port, free_port()
    p = Proc([PY, "-m", "optiprimex.tools.judge_web", "--port", str(port), "--team", TEAM, "--web-port", str(web)], tmp_path / "web.log")
    pub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-web-pub", protocol=mqtt.MQTTv311)
    pub.connect("127.0.0.1", port)
    pub.loop_start()
    try:
        def state():
            try:
                return json.loads(urllib.request.urlopen(f"http://127.0.0.1:{web}/state.json", timeout=2).read())
            except Exception:
                return None
        assert wait_until(lambda: (state() or {}).get("connected"), 8), p.log()
        assert wait_until(lambda: ((state() or {}).get("course") or {}).get("course_id") == "BENCH-COURSE-1", 5)       # the retained RxCourse
        pub.publish(W.t_report(TEAM, "USV1"), W.encode_report(TEAM, "USV1", 1, {"kind": "heartbeat", "state": "STATE_AUTO", "current_task": "TASK_NONE",
                                                                                 "lat": 1.3, "lon": 103.8, "roll_deg": 3.0, "vehicle_type": "USV"}), qos=1).wait_for_publish(2)
        pub.publish(f"rc/report/{TEAM}/USV1", json.dumps({"kind": "docking", "team_id": TEAM, "report_seq": 2, "bay": 3}), qos=1).wait_for_publish(2)
        assert wait_until(lambda: "docking" in (((state() or {}).get("vehicles") or {}).get("USV1") or {}).get("reports", {}), 5), state()
        s = state()
        assert s["vehicles"]["USV1"]["heartbeat"]["roll_deg"] == 3.0 and s["wires"]["json"] >= 1 and s["wires"]["protobuf"] >= 2
        page = urllib.request.urlopen(f"http://127.0.0.1:{web}/", timeout=2).read().decode()
        assert "Judge view" in page and "/state.json" in page and "roll / pitch" in page
    finally:
        pub.loop_stop(); pub.disconnect()
        p.kill()
