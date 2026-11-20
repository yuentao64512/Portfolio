"""SIL-1 (ARCHITECTURE 7): fake RoboCommand + ocs_app + mission manager + fake FCU, all as processes.

Proves the P2 gate on the laptop: RunDeclaration accepted -> heartbeats -> RunStart -> START_RUN acked ->
STATE_AUTO + TASK_DYNAMIC_INCIDENT in the judge's heartbeats -> IncidentAck -> ReadinessReport ->
ReadinessConfirm -> task done; the 5 msg/s cap is respected; bench injectors work; the stall path alerts.
"""
from __future__ import annotations

import json
import time

import pytest

from optiprimex.tools.ocs_cli import OcsConsole

from .conftest import PY, Probe, Proc, make_ocs_config, make_usv_config, wait_until

NS = "team/optiprime/v1"


def _stats(probe: Probe):
    m = probe.last("rc/sim/stats")
    return json.loads(m["payload"]) if m else None


def _ocs_state(probe: Probe):
    m = probe.last("ocs/state")
    return m["env"]["body"] if m else None


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("sil1")
    usv_cfg = make_usv_config(tmp, brokers, fake_fcu)
    ocs_cfg = make_ocs_config(tmp, brokers, stall_s=4.0)
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(usv_cfg)], tmp / "mm.log")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--start-delay", "1.5",
               "--auto-incident-after", "3", "--confirm-delay", "0.5", "--log", str(tmp / "rc.jsonl")], tmp / "rc.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#", "rc/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15, include_past=True), mm.log()
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(ocs_cfg)], tmp / "ocs.log")
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 15, include_past=True), ocs.log()
    yield {"mm": mm, "rc": rc, "ocs": ocs, "probe": probe, "con": con, "tmp": tmp, "fcu": fake_fcu, "brokers": brokers,
           "ocs_cfg": ocs_cfg}
    con.stop()
    probe.stop()
    for p in (ocs, rc, mm):
        p.kill()


def test_full_t4_chain_through_robocommand(stack):
    con, p, fcu = stack["con"], stack["probe"], stack["fcu"]
    # R2 -> declare -> R3; PREPARE reaches the vehicle; course went down with it
    r = con.intent("DECLARE")
    assert r["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 5)
    assert p.wait(lambda m: m["topic"].endswith("cmd/usv/run") and m["env"]["body"]["action"] == "PREPARE", 5, include_past=True)
    # ENABLE_AUTO -> vehicle AUTO_HOLD -> R4 -> RunStart from the judge -> R5 and START_RUN to the vehicle
    r = con.intent("ENABLE_AUTO", wait_s=5)
    assert r["result"]["result"] == "accepted" and [a["body"]["result"] for a in r["acks"]] == ["ACCEPTED", "DONE"]
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15), _ocs_state(p)
    st = _ocs_state(p)
    assert st["run_id"] is not None and st["declaration_seq"] == 1
    assert p.wait(lambda m: m["topic"].endswith("ack/usv") and m["env"]["body"]["result"] == "DONE", 5, include_past=True)
    # the judge fires an AssistanceRequest 3 s after RunStart: ack, transit, readiness, confirm, done
    assert p.wait(lambda m: m["topic"].endswith("event/usv/incident") and m["env"]["body"]["kind"] == "ack", 15), "no IncidentAck event"
    assert p.wait(lambda m: m["topic"].endswith("event/usv/incident") and m["env"]["body"]["kind"] == "readiness", 60), "no readiness event"
    assert p.wait(lambda m: m["topic"].endswith("cmd/usv/incident") and m["env"]["body"]["args"].get("kind") == "readiness_confirm", 10), "no ReadinessConfirm relayed"
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                  and m["env"]["body"]["current_task"] == "TASK_NONE", 10), "task did not finish"
    # what the judge saw
    assert wait_until(lambda: (_stats(p) or {}).get("confirms", 0) >= 1, 5)
    s = _stats(p)
    assert s["declarations"] == 1 and s["run_starts"] == 1 and s["incident_acks"] == 1 and s["readiness"] == 1
    assert s["heartbeats"]["USV1"] >= 10 and s["rate_violations"] == 0 and s["max_rate"] <= 5
    assert "STATE_AUTO" in s["states_seen"]["USV1"] and "TASK_DYNAMIC_INCIDENT" in s["tasks_seen"]["USV1"]
    assert "STATE_MANUAL" in s["states_seen"]["USV1"]      # heartbeats started at the declaration, before ENABLE_AUTO
    pos = fcu.state()
    assert abs(pos["lat"] - 1.28113) < 2e-5 and abs(pos["lon"] - 103.85503) < 2e-5   # at the request position (within ~2 m)


def test_bench_injectors_and_guards(stack):
    con, p = stack["con"], stack["probe"]
    # a keep-out zone injected from the console is acked by the vehicle like a real one
    r = con.intent("INJECT", args={"what": "keep_out_zone", "polygon": [[1.2811, 103.8551], [1.2812, 103.8551], [1.2812, 103.8552]]})
    assert r["result"]["result"] == "accepted"
    m = p.wait(lambda m: m["topic"].endswith("event/usv/incident") and m["env"]["body"].get("what") == "keep_out_zone", 10)
    assert m and m["env"]["body"]["kind"] == "ack"
    assert p.wait(lambda m: m["topic"].startswith("rc/report/") and json.loads(m["payload"]).get("kind") == "incident_ack", 10, include_past=True)
    # after the keep-out a T4 task is listening; abort it, then the never-list and a bad task name are refused
    r = con.intent("ABORT_TASK", wait_s=5)
    assert r["result"]["result"] == "accepted"
    assert con.intent("ARM")["result"]["result"] == "refused"
    assert con.intent("START_TASK", "T9")["result"]["result"] == "refused"
    # GOTO is a bench-only intent and runs as a vehicle task
    r = con.intent("GOTO", args={"x_n": 5.0, "y_e": -5.0}, wait_s=5)
    assert r["result"]["result"] == "accepted" and [a["body"]["result"] for a in r["acks"]] == ["ACCEPTED", "DONE"]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["task_name"] == "GOTO", 5)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                  and m["env"]["body"]["current_task"] == "TASK_NONE", 40)
    # end the run: STOP_RUN to the vehicle, R6
    assert con.intent("END_RUN", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R6", 5)


def test_redeclare_after_end_run(stack):
    """A new run needs a new declaration (R6 -> R3); the judge accepts it and starts run 2."""
    con, p = stack["con"], stack["probe"]
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15)
    st = _ocs_state(p)
    assert st["declaration_seq"] == 2 and not st["stalled"]
    assert wait_until(lambda: (_stats(p) or {}).get("run_starts", 0) >= 2, 5)
    s = _stats(p)
    assert s["declarations"] == 2 and s["run_starts"] == 2


def test_reset_both_ends_mission_preselect_and_test_missions(stack):
    """reset: OCS R2 + vehicle MANUAL with the run forgotten; mission TEST_AUTO --auto starts by itself on RunStart;
    STEP walks TEST_MANUAL; the judge view and the PREPARE gate are visible in ocs/state."""
    con, p = stack["con"], stack["probe"]
    # v0.1.26 (WP7): this stack's judge fires an AssistanceRequest 3 s after every RunStart; here it would start a T4 detour in the
    # middle of TEST_AUTO (the old flake). Switch it off for this test and back on at the end.
    _judge_sim_config(stack, None)
    # 1. reset from R5 (previous test): both ends back
    r = con.intent("RESET_RUN", wait_s=5)
    assert r["result"]["result"] == "accepted", r
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2" and m["env"]["body"]["run_id"] is None, 10)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 10)
    assert con.intent("RESET_RUN", wait_s=3)["result"]["result"] == "accepted"          # idempotent in R2
    # 2. preselect TEST_AUTO with auto-start, declare (PREPARE gate), auto, RunStart -> the task starts by itself
    r = con.intent("MISSION", args={"task": "TEST_AUTO", "auto": True})
    assert r["result"]["result"] == "accepted" and "auto-start" in r["result"]["reason"]
    assert p.wait(lambda m: m["topic"] == "ocs/state" and (m["env"]["body"].get("mission") or {}).get("task") == "TEST_AUTO", 5)
    r = con.intent("DECLARE")
    assert r["result"]["result"] == "accepted" and "PREPARE" in r["result"]["reason"]
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10)    # after the PREPARE ack
    assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/alert" and m["env"]["body"]["kind"] == "mission_started", 15)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_TASK"
                  and m["env"]["body"]["task_name"] == "TEST_AUTO", 10)
    m = p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["run_id"] is not None
               and m["env"]["body"]["vehicles"]["USV1"].get("task_name") == "TEST_AUTO", 5, include_past=True)
    st = m["env"]["body"]
    jv = st["judge_view"]
    assert jv["course"]["valid"] and jv["run_start"]["accepted"] and jv["run_start"]["run_id"] == st["run_id"], jv
    # STEP skips the timers: S1 -> S2 -> S3 -> DONE -> AUTO_HOLD
    for _ in range(3):
        r = con.intent("STEP", wait_s=5)
        assert r["result"]["result"] == "accepted", r
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10)
    # 3. TEST_MANUAL by hand, then a step with nothing running is rejected by the vehicle
    r = con.intent("START_TASK", "TEST_MANUAL", wait_s=5)
    assert r["result"]["result"] == "accepted" and [a["body"]["result"] for a in r["acks"]][-1] == "DONE"
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["task_state"] == "S1", 10)
    r = con.intent("STEP", wait_s=5)
    assert [a["body"]["result"] for a in r["acks"]][-1] == "DONE" and any("S1 -> S2" in a["body"]["reason"] for a in r["acks"])
    r = con.intent("ABORT_TASK", wait_s=5)
    assert r["result"]["result"] == "accepted", r
    last = {}
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and (last.update(m["env"]["body"]) or m["env"]["body"]["state"] == "AUTO_HOLD"), 10), \
        (r, {k: last.get(k) for k in ("state", "hold_reason", "fault", "task_name", "task_state")}, stack["mm"].log()[-1500:])
    r = con.intent("STEP", wait_s=5)
    assert [a["body"]["result"] for a in r["acks"]][-1] == "REJECTED"
    assert con.intent("MISSION", args={"task": "off"})["result"]["result"] == "accepted"
    assert con.intent("RESET_RUN", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 10)
    _judge_sim_config(stack, 3)


def _judge_sim_config(stack, auto_incident_after) -> None:
    import paho.mqtt.client as mqtt
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-sim-config", protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", stack["brokers"].hub_port)
    c.loop_start()
    c.publish("rc/sim/inject", json.dumps({"kind": "sim_config", "auto_incident_after": auto_incident_after}), qos=1).wait_for_publish(2)
    c.loop_stop()
    c.disconnect()
    assert wait_until(lambda: "sim_config" in stack["rc"].log(), 3), stack["rc"].log()[-500:]


def test_mission_codes_and_sim_guard(stack):
    """v0.1.19: the operator's mission code is resolved on the OCS side only (the wire keeps task T4 / TEST_AUTO + args
    tier / sim); ocs/state.mission carries the code; a `sim` mission is REJECTED by the vehicle until usv_ui's presence
    beat says it holds the fake FCU, and the status then carries fcu_link."""
    import paho.mqtt.client as mqtt
    from optiprimex.tools import ocs_cli, playbook
    con, p, brokers = stack["con"], stack["probe"], stack["brokers"]
    # the cli resolves codes, suffix and old names into the wire task + args
    a = ocs_cli.parse(["start_task", "m0t1sim"])
    assert ocs_cli.intent_spec(a)[:3] == ("START_TASK", "TEST_AUTO", {"sim": True})
    a = ocs_cli.parse(["mission", "M1T2", "--auto"])
    assert ocs_cli.intent_spec(a)[2] == {"task": "T1", "auto": True, "tier": "advanced"}
    a = ocs_cli.parse(["start_task", "TEST_MANUAL"])
    assert ocs_cli.intent_spec(a)[1] == "TEST_MANUAL"
    assert playbook.resolve("m4t0")[1] == "T4"
    # start clean whatever the previous test left (it may have failed under load): both ends to R2 / MANUAL
    con.intent("RESET_RUN", wait_s=5)
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2" and m["env"]["body"]["run_id"] is None, 10)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 10)
    # preselect m0t1sim (TEST_AUTO + sim), no auto-start; the OCS state shows the code
    r = con.intent("MISSION", args={"task": "TEST_AUTO", "sim": True})
    assert r["result"]["result"] == "accepted" and "sim=True" in r["result"]["reason"], r
    assert p.wait(lambda m: m["topic"] == "ocs/state" and (m["env"]["body"].get("mission") or {}).get("code") == "m0t1sim", 5)
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10)
    assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10)
    # the fake judge of this stack sends one AssistanceRequest 3 s after RunStart, which auto-starts T4 in AUTO_HOLD: let it, drop it
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("task_name") == "T4", 10)
    assert con.intent("ABORT_TASK", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10)
    # 1. no usv_ui beat on the boat broker -> the vehicle rejects the sim mission (the preselection carried sim into START_TASK)
    r = con.intent("START_TASK", wait_s=5)
    assert r["result"]["result"] == "accepted", r
    acks = [a["body"] for a in r["acks"]]
    assert acks and acks[-1]["result"] == "REJECTED" and "sim mission needs the fake FCU" in acks[-1]["reason"], acks
    mid = r["result"]["cmd_msg_id"]
    cmd = p.wait(lambda m: m["topic"].endswith("cmd/usv/run") and m["env"]["msg_id"] == mid, 5, include_past=True)
    assert cmd["env"]["body"]["task"] == "TEST_AUTO" and cmd["env"]["body"]["args"].get("sim") is True        # the wire: task + args, no code
    # 2. a usv_ui beat saying fcu_link fake (retained, as usv_ui publishes it) -> accepted; the status carries fcu_link
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-beat", protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", brokers.boat_port)
    c.loop_start()
    beat = {"node": "usv_ui", "up": True, "pid": 1, "fcu_link": "fake", "fcu_conn": "udpin:127.0.0.1:14550", "t_unix": time.time()}
    c.publish("presence/usv_ui", json.dumps(beat), qos=0, retain=True).wait_for_publish(2)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("fcu_link") == "fake", 10)
    r = con.intent("START_TASK", "TEST_AUTO", args={"sim": True}, wait_s=5)
    assert r["result"]["result"] == "accepted" and [a["body"]["result"] for a in r["acks"]][-1] == "DONE", r
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_TASK"
                  and m["env"]["body"]["task_name"] == "TEST_AUTO", 10)
    # 3. a real link -> rejected again; then the beat goes down -> unknown -> rejected
    assert con.intent("ABORT_TASK", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10)
    c.publish("presence/usv_ui", json.dumps(dict(beat, fcu_link="real")), qos=0, retain=True).wait_for_publish(2)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("fcu_link") == "real", 10)
    r = con.intent("START_TASK", "TEST_AUTO", args={"sim": True}, wait_s=5)
    assert [a["body"]["result"] for a in r["acks"]][-1] == "REJECTED"
    r = con.intent("START_TASK", "TEST_AUTO", wait_s=5)                       # the same mission without sim runs on a real link
    assert [a["body"]["result"] for a in r["acks"]][-1] == "DONE", [a["body"] for a in r["acks"]]
    assert con.intent("ABORT_TASK", wait_s=5)["result"]["result"] == "accepted"
    c.publish("presence/usv_ui", json.dumps(dict(beat, up=False)), qos=0, retain=True).wait_for_publish(2)
    c.publish("presence/usv_ui", None, qos=0, retain=True).wait_for_publish(2)            # clear the retained beat for the other tests
    c.loop_stop(); c.disconnect()
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("fcu_link") is None, 10)
    assert con.intent("MISSION", args={"task": "off"})["result"]["result"] == "accepted"
    assert con.intent("RESET_RUN", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 10)


def test_step_and_from_through_the_mission_manager(stack):
    """v0.1.26 (WP7) on the wire: START_TASK args.from with an unmet precondition is REJECTED before the vehicle moves; args.step
    parks the vehicle in AUTO_SAFE_HOLD at the stage boundary with hold_reason "step: next S2" (seen in ocs/state, and the
    playbook's next line says resume -> S2); RESUME enters S2. This stack has no camera: the dock is never in the map."""
    from optiprimex.tools import playbook
    con, p = stack["con"], stack["probe"]
    _judge_sim_config(stack, None)
    try:
        con.intent("RESET_RUN", wait_s=5)
        assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2" and m["env"]["body"]["run_id"] is None, 10)
        assert con.intent("DECLARE")["result"]["result"] == "accepted"
        assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10)
        assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
        assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15)
        assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10)
        r = con.intent("START_TASK", "T3", args={"from": "ENTER"}, wait_s=5)
        ack = r["acks"][-1]["body"]
        assert ack["result"] == "REJECTED" and ack["reason"].startswith("from: ENTER needs the dock line"), ack
        assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 5, include_past=True)
        r = con.intent("START_TASK", "TEST_AUTO", args={"step": True}, wait_s=5)
        assert [a["body"]["result"] for a in r["acks"]][-1] == "DONE", [a["body"] for a in r["acks"]]
        assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_SAFE_HOLD"
                      and m["env"]["body"].get("hold_reason") == "step: next S2", 15), stack["mm"].log()[-1500:]
        m = p.wait(lambda m: m["topic"] == "ocs/state" and (m["env"]["body"]["vehicles"].get("USV1") or {}).get("hold_reason") == "step: next S2", 5,
                   include_past=True)
        assert playbook.suggest(m["env"]["body"], "USV1").startswith("resume -> S2")
        assert con.intent("RESUME", wait_s=5)["result"]["result"] == "accepted"
        assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_TASK"
                      and m["env"]["body"]["task_state"] == "S2", 10)
        assert con.intent("ABORT_TASK", wait_s=5)["result"]["result"] == "accepted"
        assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10)
    finally:
        con.intent("RESET_RUN", wait_s=5)
        _judge_sim_config(stack, 3)
