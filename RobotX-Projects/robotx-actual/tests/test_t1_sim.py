"""m1t1 (Task 1 Safe Passage, core) and m0t2 (path test) in simulation (ROADMAP WP2).

Fast SIL (optiprimex.sim.fast_sil: the fake FCU's plant + the sim camera + the PerceptionMap + the task, in simulated
time) proves the task logic against the world file's truth: both planners pass the three gates of world_pool with RED to
starboard, circle the entry clockwise and the exit counter-clockwise, strike no buoy and report; a gate that could only
be crossed with RED to port is refused (safe hold); a missing gate scans, then safe-holds; m0t2 runs the PoR course.
The live test runs m1t1sim through the real processes (mission manager, OCS, fake judge, sim CV) and the judge sees the
SafePassageReport.
"""
from __future__ import annotations

import json
import math
import random
import time

import pytest
import yaml

from optiprimex.sim.marina import CENTRE  # v0.1.33: the worlds' origin (Course Alpha, Marina Bay)
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil
from optiprimex.usv.gatepath import dubins

from .conftest import PY, ROOT, Probe, Proc, make_ocs_config, make_usv_config, wait_until

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
POR = str(ROOT / "profiles" / "bench" / "world_por.yaml")
FAST = {"circle_v": 1.0, "transit_v": 1.2}
T1_STATES = ["SEARCH_ENTRY", "CIRCLE_ENTRY_CW", "TRANSIT", "EXIT", "CIRCLE_EXIT_CCW", "DONE"]
STRIKE_M = 0.25 + 0.91 / 2                                       # buoy radius + half the beam


def _crossings(world, track):
    """For each truth gate: None (never crossed), True (RED to starboard) or False (RED to port)."""
    def o(p, q, r):
        return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])
    gates = {}
    for b in W.load(world)["objects"]:
        if b.get("gate") is not None:
            gates.setdefault(b["gate"], {})[b["colour"]] = (b["x_n"], b["y_e"])
    tr = [(x, y) for _, x, y, _, _ in track]
    out = {}
    for g, d in sorted(gates.items()):
        out[g] = None
        a, b = d["red"], d["green"]
        for p1, p2 in zip(tr, tr[1:]):
            s1, s2 = o(a, b, p1), o(a, b, p2)                 # half-open: a sample exactly on the line counts once (v0.1.39)
            if o(p1, p2, a) * o(p1, p2, b) < 0 and ((s1 < 0 <= s2) or (s2 < 0 <= s1)):
                mv = (p2[0] - p1[0], p2[1] - p1[1])
                out[g] = ((a[0] - p1[0]) * -mv[1] + (a[1] - p1[1]) * mv[0]) > 0
                break
    return out


def _clearance(world, track):
    buoys = [o for o in W.load(world)["objects"] if o["kind"] == "buoy"]
    return min(math.hypot(x - b["x_n"], y - b["y_e"]) for _, x, y, _, _ in track for b in buoys)


def _winding(track, c, state):
    """Signed turning of the boat about c (deg, + = clockwise) over the track samples taken in `state`."""
    pts = [(x, y) for _, x, y, _, s in track if s == state]
    tot = 0.0
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        a0, a1 = math.atan2(y0 - c[1], x0 - c[0]), math.atan2(y1 - c[1], x1 - c[0])
        tot += math.atan2(math.sin(a1 - a0), math.cos(a1 - a0))
    return math.degrees(tot)


def _world_variant(tmp_path, name, edit):
    w = yaml.safe_load(open(POOL))
    edit(w)
    p = tmp_path / f"{name}.yaml"
    p.write_text(yaml.safe_dump(w))
    return str(p)


def test_dubins_reference_endpoints():
    """pathing_sim spec 11: 2000 random pose pairs end within 1e-6 m of the goal."""
    rng = random.Random(7)
    worst = 0.0
    for _ in range(2000):
        s = (rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(-math.pi, math.pi))
        g = (rng.uniform(-30, 30), rng.uniform(-30, 30), rng.uniform(-math.pi, math.pi))
        pts = dubins.shortest(s, g, rng.uniform(1.0, 6.0)).sample(0.25)
        worst = max(worst, math.hypot(pts[-1][0] - g[0], pts[-1][1] - g[1]))
    assert worst < 1e-6, worst
    p = dubins.shortest((0, 0, 0), (0, 10, math.pi / 2), 3.0)      # north-facing to a point east facing east: a right turn first
    assert p.kind.startswith("R") and p.sample(1.0)[2][1] > 0


@pytest.mark.parametrize("planner", ["simple", "s6"])
def test_m1t1_fast_sil_passes_the_pool(planner):
    r = FastSil(POOL, "T1", dict(FAST, planner=planner)).run(600)
    assert r["state"] == "DONE", (r["state"], r["reason"], r["logs"][-5:])
    assert r["states"] == T1_STATES, r["states"]
    assert _crossings(POOL, r["track"]) == {1: True, 2: True, 3: True}
    assert _clearance(POOL, r["track"]) > STRIKE_M + 0.05
    assert _winding(r["track"], (5.0, -7.0), "CIRCLE_ENTRY_CW") >= 330           # clockwise round the entry
    assert _winding(r["track"], (33.0, -5.5), "CIRCLE_EXIT_CCW") <= -330          # counter-clockwise round the exit
    rep = r["reports"][-1]
    assert rep["complete"] and rep["gates_passed"] == 3
    from optiprimex.companion.frames import ControlOrigin
    o = ControlOrigin(*CENTRE)
    en, ex = o.to_local(rep["entry"]["lat"], rep["entry"]["lon"]), o.to_local(rep["exit"]["lat"], rep["exit"]["lon"])
    assert math.hypot(en[0] - 5.0, en[1] + 7.0) < 1.0 and math.hypot(ex[0] - 33.0, ex[1] + 5.5) < 1.0
    states = {b["state"] for b in rep["buoys"]}
    assert {"flashing", "solid", "off"} <= states and len(rep["buoys"]) >= 9
    assert len(r["reports"]) >= 2                                                 # updated as the map grew
    if planner == "s6":
        assert any("dubins" in l for l in r["logs"]), r["logs"]


def test_m1t1_after_a_long_wait_at_the_start():
    """Found live 2026-11-02: after minutes at the start pose, far wrong-colour frames made the unlit obstacle at 22 m look like
    a flashing blue light and m1t1 circled it as the entry. The lit colour must now dominate the lit sightings (LIT_DOMINANT)."""
    s = FastSil(POOL, "T1", FAST, seed=3)
    s.warmup(150.0)
    mk = s.map.markers()
    assert mk["entry"] is None and mk["exit"] is None, {k: v and (v.x_n, v.y_e, v.colour_votes) for k, v in mk.items()}
    obstacle = min(s.map.entities("buoy"), key=lambda e: abs(e.x_n - 22.5) + abs(e.y_e + 1.5))
    assert obstacle.colour == "black" and obstacle.light == "off", obstacle.colour_votes
    r = s.run(600)
    assert r["state"] == "DONE" and _crossings(POOL, r["track"]) == {1: True, 2: True, 3: True}, r["logs"][-6:]
    assert any("entry locked: " in l and "N +5." in l for l in r["logs"]), [l for l in r["logs"] if "entry" in l]


def test_m1t1_refuses_a_wrong_side_gate(tmp_path):
    def swap(gate):
        def edit(w):
            for o in w["objects"]:
                if o.get("gate") == gate:
                    o["colour"] = "green" if o["colour"] == "red" else "red"   # RED west / GREEN east: legal only southbound
        return edit
    # the last gate swapped: no legal way through it from the channel -> refused, scan, safe hold
    world = _world_variant(tmp_path, "wrong_side_3", swap(3))
    r = FastSil(world, "T1", FAST).run(600)
    assert r["state"] == "SAFE_HOLD", (r["state"], r["logs"][-6:])
    assert r["sub_state"] == "SCAN" and "RED to port" in r["safe_holds"][0], r["safe_holds"]
    cr = _crossings(world, r["track"])
    assert cr[1] is True and cr[2] is True and cr[3] is None                     # never crossed the refused gate
    # a middle gate swapped: the chain takes gate 3 first and crosses gate 2 southbound, RED still to starboard (legal)
    world = _world_variant(tmp_path, "wrong_side_2", swap(2))
    r = FastSil(world, "T1", FAST).run(600)
    assert r["state"] == "DONE" and _crossings(world, r["track"]) == {1: True, 2: True, 3: True}, r["logs"][-6:]
    assert _clearance(world, r["track"]) > STRIKE_M


def test_m1t1_missing_gate_scans_then_holds_or_goes_to_the_exit(tmp_path):
    world = _world_variant(tmp_path, "no_gate3_no_exit",
                           lambda w: w.update(objects=[o for o in w["objects"] if o.get("gate") != 3 and o.get("role") != "exit"]))
    r = FastSil(world, "T1", FAST).run(600)
    assert r["state"] == "SAFE_HOLD" and r["sub_state"] == "SCAN", (r["state"], r["logs"][-5:])
    assert "no gate ahead and no exit buoy found" in r["safe_holds"][0]
    world = _world_variant(tmp_path, "no_gate3", lambda w: w.update(objects=[o for o in w["objects"] if o.get("gate") != 3]))
    r = FastSil(world, "T1", FAST).run(600)
    assert r["state"] == "DONE" and "EXIT" in r["states"], (r["state"], r["logs"][-5:])


def test_m0t2_runs_the_por_course_and_given_points():
    r = FastSil(POR, "TEST_PATH", {"speed_mps": 1.0}).run(200)
    assert r["state"] == "DONE" and _crossings(POR, r["track"]) == {1: True, 2: True}, (r["logs"], r["states"])
    assert _clearance(POR, r["track"]) > STRIKE_M + 0.05
    o = W.WorldFrame({"lat": CENTRE[0], "lon": CENTRE[1]})
    pts = [list(o.to_latlon(6.0, 2.0)), {"lat": o.to_latlon(10.0, -2.0)[0], "lon": o.to_latlon(10.0, -2.0)[1]}]
    r = FastSil(POR, "TEST_PATH", {"speed_mps": 1.0, "points": pts}).run(120)
    assert r["state"] == "DONE" and "2 points from START_TASK" in r["logs"][0]
    end = r["track"][-1]
    assert math.hypot(end[1] - 10.0, end[2] + 2.0) < 1.5


def test_t1_wiring_playbook_cli_and_vehicle_guards():
    from optiprimex.tools import ocs_cli, playbook
    from optiprimex.usv.tasks import REGISTRY
    assert "T1" in REGISTRY and "TEST_PATH" in REGISTRY
    assert playbook.MISSIONS["m1t1"].status == "built" and playbook.MISSIONS["m0t2"].status == "test"
    assert "SCAN" in playbook.mission_stage_names("m1t1")
    a = ocs_cli.parse(["start_task", "m1t1sim", "--planner", "s6"])
    assert ocs_cli.intent_spec(a)[:3] == ("START_TASK", "T1", {"tier": "core", "sim": True, "planner": "s6"})
    a = ocs_cli.parse(["mission", "m0t2", "--names", "EN,G1,EX"])
    assert ocs_cli.intent_spec(a)[2]["names"] == ["EN", "G1", "EX"]
    # found live 2026-11-02: the vehicle FSM kept its own task list and refused TEST_PATH ("unknown task"); the OCS its MISSION list
    from optiprimex.companion.config import Config
    from optiprimex.companion.state import Command
    from optiprimex.ocs.cmd_authority import MISSION_TASKS, START_TASKS
    from optiprimex.usv.vehicle_fsm import VehicleFsm
    fsm = VehicleFsm(Config())
    for t in REGISTRY:
        ok, why = fsm.can_start_task(Command(env={}, action="START_TASK", task=t))
        assert "unknown task" not in why, (t, why)
        if t == "RETURN_HOME":                    # v0.1.30: started by HOME only (the go-home override), never a mission
            assert t not in MISSION_TASKS and t not in START_TASKS
            continue
        assert t in MISSION_TASKS and (t in START_TASKS or t == "GOTO"), t


# ---- live: the mission manager, the OCS and the fake judge, sim CV on world_pool ----------------------------------------
NS = "team/optiprime/v1"


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    import paho.mqtt.client as mqtt
    tmp = tmp_path_factory.mktemp("t1")
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(make_usv_config(tmp, brokers, fake_fcu))], tmp / "mm.log")
    fp = Proc([PY, "-m", "optiprimex.sim.fake_perception", "--world", POOL, "--port", str(brokers.boat_port), "--hub-port", str(brokers.hub_port),
               "--seed", "3"], tmp / "fp.log")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--start-delay", "1.0", "--log", str(tmp / "rc.jsonl")], tmp / "rc.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#", "rc/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15, include_past=True), mm.log()
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(make_ocs_config(tmp, brokers))], tmp / "ocs.log")
    from optiprimex.tools.ocs_cli import OcsConsole
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 15, include_past=True), ocs.log()
    beat = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t1-ui-beat", protocol=mqtt.MQTTv311)
    beat.connect("127.0.0.1", brokers.boat_port)
    beat.loop_start()
    body = {"node": "usv_ui", "up": True, "pid": 1, "fcu_link": "fake", "t_unix": time.time()}
    beat.publish("presence/usv_ui", json.dumps(body), qos=0, retain=True).wait_for_publish(2)     # the sim guard: usv_ui holds the fake FCU
    yield {"mm": mm, "probe": probe, "con": con, "fcu": fake_fcu, "ocs": ocs, "rc": rc, "fp": fp}
    beat.publish("presence/usv_ui", None, qos=0, retain=True).wait_for_publish(2)
    beat.loop_stop(); beat.disconnect()
    con.stop(); probe.stop()
    for p in (ocs, rc, fp, mm):
        p.kill()


def test_m1t1sim_live_through_the_mission_manager_and_the_judge(stack):
    con, p = stack["con"], stack["probe"]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("fcu_link") == "fake", 10)
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10)
    assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15)
    r = con.intent("START_TASK", "T1", args={"tier": "advanced", "sim": True}, wait_s=5)      # advanced needs the UAV: refused (v0.1.66: at the OCS)
    assert (r["result"]["result"] == "refused" and "needs the UAV" in r["result"]["reason"]) or (
        [a["body"]["result"] for a in r["acks"]][-1] == "REJECTED" and "needs the UAV" in r["acks"][-1]["body"]["reason"])
    r = con.intent("START_TASK", "T1", args=dict(FAST, tier="core", sim=True, planner="simple"), wait_s=5)
    assert [a["body"]["result"] for a in r["acks"]][-1] == "DONE", [a["body"] for a in r["acks"]]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("task_state") == "CIRCLE_ENTRY_CW", 40), stack["mm"].log()[-2000:]
    assert p.wait(lambda m: m["topic"].endswith("telemetry/usv/map") and len(m["env"]["body"].get("path") or []) > 10, 10)   # the orbit drawn
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("task_state") == "TRANSIT", 60)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("task_state") == "CIRCLE_EXIT_CCW", 120), stack["mm"].log()[-2000:]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                  and m["env"]["body"]["current_task"] == "TASK_NONE", 60), stack["mm"].log()[-2000:]
    seen = [m["env"]["body"].get("task_state") for m in p.all(f"{NS}/status/usv")]
    assert "TASK_SAFE_PASSAGE" in {m["env"]["body"].get("current_task") for m in p.all(f"{NS}/status/usv")}
    for s in T1_STATES[:-1]:
        assert s in seen, (s, sorted(set(x for x in seen if x)))

    def stats():
        m = p.last("rc/sim/stats")
        return json.loads(m["payload"]) if m else {}
    assert wait_until(lambda: any(x["kind"] == "safe_passage" for x in stats().get("reports", [])), 10), stats().get("reports")
    assert stats()["rate_violations"] == 0
    st = stack["fcu"].state()
    assert 27.0 < st["x_n"] < 39.0 and -11.0 < st["y_e"] < 0.0, st                              # ended at the exit circle
