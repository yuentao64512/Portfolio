"""B1 integration: two brokers with the real bridge, the fake FCU and the mission manager as processes.

Proves on the laptop what the P1 bench gate asks for: status at 2 Hz on the hub, commands/acks with dedup
and run-id rules, kill / RC6 / FCU-loss driving V1 / V2 / V6, LWT, and the bridge queue flush after a hub outage.
"""
from __future__ import annotations

import json
import signal
import time

import pytest

from optiprimex.tools.vcmd import VehicleCommander

from .conftest import PY, Probe, Proc, make_usv_config, wait_until

NS = "team/optiprime/v1"


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("b1")
    cfg = make_usv_config(tmp, brokers, fake_fcu, **{"mqtt.keepalive_s": 5,   # LWT after 1.5 x keepalive
                                                     "guards.test_mode": False})   # v0.1.37: the run lifecycle's rules (the course's), not TEST MODE
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(cfg)], tmp / "mm.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#"]).start()
    vc = VehicleCommander(port=brokers.hub_port)
    vc.start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15), mm.log()
    st = {"mm": mm, "probe": probe, "vc": vc, "cfg": cfg, "tmp": tmp, "brokers": brokers, "fcu": fake_fcu}
    yield st
    vc.stop()
    probe.stop()
    st["mm"].kill()          # the LWT test replaces the manager; kill whichever one is current


def status(stack):
    m = stack["probe"].last(f"{NS}/status/usv")
    return m["env"]["body"] if m else None


def wait_state(stack, state, timeout=10):
    m = stack["probe"].wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == state, timeout)
    assert m, f"never saw state {state}; last {status(stack)}"
    return m["env"]["body"]


def results(acks):
    return [a["body"]["result"] for a in acks]


def test_status_rate_on_hub_through_bridge(stack):
    p = stack["probe"]
    p.clear()
    time.sleep(3.0)
    st = p.all(f"{NS}/status/usv")
    assert len(st) >= 5, f"only {len(st)} status messages in 3 s"
    body = st[-1]["env"]["body"]
    assert body["state"] == "MANUAL" and body["robot_state"] == "STATE_MANUAL" and body["mode"] == "HOLD" and body["armed"] is True
    assert st[-1]["env"]["vehicle_id"] == "USV1" and st[-1]["env"]["kind"] == "status"
    assert p.all(f"{NS}/telemetry/usv/map"), "map digest missing on the hub"


def test_command_chain_prepare_enable_run_goto(stack):
    vc, fcu = stack["vc"], stack["fcu"]
    assert results(vc.send("run", "ENABLE_AUTO"))[0] == "ACCEPTED"                     # v0.1.90.8: before PREPARE too (the rule everywhere)
    vc.send("run", "DISABLE_AUTO")                                                     # back to MANUAL: the chain below as before
    vc.declaration_seq = 1
    assert results(vc.send("run", "PREPARE", args={"declaration_seq": 1, "tiers": {"T4": "TIER_CORE"}})) == ["ACCEPTED", "DONE"]
    acks = vc.send("run", "ENABLE_AUTO")
    assert results(acks) == ["ACCEPTED", "DONE"] and acks[-1]["body"]["state"] == "AUTO_HOLD"
    wait_state(stack, "AUTO_HOLD")
    assert results(vc.send("run", "START_TASK", task="GOTO", args={"x_n": 12.0, "y_e": 0.0})) == ["REJECTED"]   # no START_RUN
    assert results(vc.send("run", "START_TASK", task="GOTO", args={"x_n": 12.0, "y_e": 0.0}, run_id=4242)) == ["REJECTED"]   # unknown run
    assert results(vc.send("run", "START_RUN", args={"run_id": 1001})) == ["ACCEPTED", "DONE"]
    vc.run_id = 1001
    assert results(vc.send("run", "START_TASK", task="GOTO", args={"x_n": 12.0, "y_e": 0.0})) == ["ACCEPTED", "DONE"]
    wait_state(stack, "AUTO_TASK", 5)
    assert wait_until(lambda: fcu.state()["x_n"] > 10.5, 25), f"boat did not move: {fcu.state()}"
    wait_state(stack, "AUTO_HOLD", 20)
    s = fcu.state()
    assert s["counters"]["arm_requests"] == 0 and s["counters"]["targets"] >= 1 and s["mode_name"] == "HOLD"


def test_duplicate_msg_id_is_acked_not_reexecuted(stack):
    vc, p = stack["vc"], stack["probe"]
    msg_id = "dup-" + str(time.time())
    a1 = vc.send("run", "HOLD", msg_id=msg_id)
    assert results(a1) == ["ACCEPTED", "DONE"]
    wait_state(stack, "AUTO_SAFE_HOLD", 5)
    a2 = vc.send("run", "HOLD", msg_id=msg_id)
    assert results(a2) == ["ACCEPTED"] and a2[0]["body"]["reason"].startswith("duplicate")
    before = len(p.all(f"{NS}/event/usv/task_state"))
    time.sleep(0.5)
    assert len(p.all(f"{NS}/event/usv/task_state")) == before                          # nothing re-executed
    assert results(vc.send("run", "RESUME")) == ["ACCEPTED", "DONE"]
    wait_state(stack, "AUTO_HOLD", 5)


def test_kill_rc6_and_fcu_loss_drive_v1_v2_v6(stack):
    vc, fcu, p = stack["vc"], stack["fcu"], stack["probe"]
    # kill -> KILLED, STATE_KILLED; clearing the kill is not enough; CONFIRM_SAFE completes on the next tick
    fcu.set(kill=True)
    body = wait_state(stack, "KILLED", 5)
    assert body["robot_state"] == "STATE_KILLED" and body["kill"] is True
    assert results(vc.send("run", "ENABLE_AUTO")) == ["REJECTED"]
    fcu.set(kill=False)
    time.sleep(1.0)
    assert status(stack)["state"] == "KILLED"
    acks = vc.send("run", "CONFIRM_SAFE", wait_s=4.0)
    assert results(acks) == ["ACCEPTED", "DONE"] and acks[-1]["body"]["state"] == "MANUAL"
    wait_state(stack, "MANUAL", 5)
    # back to AUTO, then RC6 override -> MANUAL and stays there after RC6 drops
    assert results(vc.send("run", "ENABLE_AUTO")) == ["ACCEPTED", "DONE"]
    wait_state(stack, "AUTO_HOLD", 5)
    fcu.set(rc6=1900)
    wait_state(stack, "MANUAL", 5)
    fcu.set(rc6=1000)
    time.sleep(1.0)
    assert status(stack)["state"] == "MANUAL"
    assert results(vc.send("run", "ENABLE_AUTO")) == ["ACCEPTED", "DONE"]
    wait_state(stack, "AUTO_HOLD", 5)
    # FCU stream loss -> FAULT; stream back + CONFIRM_SAFE -> MANUAL (never straight back to AUTO)
    fcu.ctrl({"pause_stream": 4.0})
    body = wait_state(stack, "FAULT", 8)
    assert body["fault"] == "FCU_LOST"
    assert wait_until(lambda: (status(stack) or {}).get("health", {}).get("fcu_age_s") is not None and status(stack)["health"]["fcu_age_s"] < 1.0, 8)
    time.sleep(0.5)
    assert status(stack)["state"] == "FAULT"
    acks = vc.send("run", "CONFIRM_SAFE", wait_s=4.0)
    assert results(acks) == ["ACCEPTED", "DONE"] and acks[-1]["body"]["state"] == "MANUAL"


def test_bridge_queue_flushes_after_hub_outage(stack):
    """Events raised while the hub is down are queued by the boat bridge (QoS 1, cleansession false) and arrive
    once the hub is back; a persistent-session subscriber on the hub receives them even if it reconnects later."""
    brokers, fcu, p = stack["brokers"], stack["fcu"], stack["probe"]
    assert status(stack)["health"]["bridge_up"] is True
    p2 = Probe(brokers.hub_port, [f"{NS}/event/usv/#"], client_id="probe-persistent", clean_session=False).start()
    time.sleep(0.5)
    brokers.stop_hub()
    time.sleep(1.0)
    fcu.set(kill=True)                 # KILLED transition -> task_state event on the boat broker, hub unreachable
    time.sleep(1.5)
    fcu.set(kill=False)
    assert wait_until(lambda: not status(stack) or True, 1)
    time.sleep(1.0)
    brokers.start_hub()
    m = p2.wait(lambda m: m["env"]["body"].get("to") == "KILLED", 20, include_past=True)
    p2.stop()
    assert m, "queued task_state event did not arrive after the hub came back"
    wait_state(stack, "KILLED", 10)
    assert wait_until(lambda: (status(stack) or {}).get("health", {}).get("bridge_up") is True, 10)
    vc = stack["vc"]
    assert results(vc.send("run", "CONFIRM_SAFE", wait_s=4.0)) == ["ACCEPTED", "DONE"]
    wait_state(stack, "MANUAL", 5)


def test_last_will_offline_on_crash(stack):
    mm, p, tmp, brokers, fcu = stack["mm"], stack["probe"], stack["tmp"], stack["brokers"], stack["fcu"]
    # A second probe on the boat broker localises a failure: will published locally (broker) vs relayed (bridge).
    pb = Probe(brokers.boat_port, [f"{NS}/status/usv"]).start()
    p.clear()
    t0 = time.monotonic()
    mm.kill(sig=signal.SIGKILL)
    # include_past: on a real OS the kernel closes the socket at once and the will can reach the probes before
    # mm.kill() returns (macOS); in the sandbox the broker only notices at 1.5 x keepalive (7.5 s here).
    mb = pb.wait(lambda m: m["env"]["body"]["state"] == "OFFLINE", 30, include_past=True)
    assert mb, "LWT OFFLINE never published on the boat broker (will not registered, or the client was not closed)"
    m = p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "OFFLINE", 30, include_past=True)
    pb.stop()
    assert m, "LWT OFFLINE reached the boat broker but not the hub (bridge out rule for status/usv)"
    print(f"LWT after {time.monotonic() - t0:.1f} s (keepalive 5 s)")
    # restart for any test that follows
    stack["mm"] = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(stack["cfg"])], tmp / "mm2.log")
    wait_state(stack, "MANUAL", 15)
