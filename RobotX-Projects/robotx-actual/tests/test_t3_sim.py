"""m3t1 (Task 3 Coordinated Logistics, core) and m0t3 (sway test) in simulation (ROADMAP WP3).

Fast SIL on world_pool with the dock simulator (fake_dock's DockSim reacting to the task's DockingReport and cannon aim):
the boat finds the GREEN bay, sways abeam at the 5 m standoff, bows in to 1.5 m off the face (v0.1.39) without touching a pontoon,
reports the docking, fires until the window turns GREEN, reports, backs out to the standoff. A RED bay chosen by hand gets
no GREEN window: the firing time runs out and the report says so. No dock in view: safe hold. m0t3 sways along the dock
line and between two points with the bow held. Live: m3t1sim through the mission manager, the OCS and the fake judge, with
fake_dock as a process lit by the DockingReport event.
"""
from __future__ import annotations

import json
import math
import time

import pytest

from optiprimex.sim.marina import CENTRE  # v0.1.33: the worlds' origin (Course Alpha, Marina Bay)
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil

from .conftest import PY, ROOT, Probe, Proc, make_ocs_config, make_usv_config, wait_until

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
NEAR_DOCK = {"x_n": 32.0, "y_e": 3.0, "hdg_deg": 0.0}
T3_STATES = ["FIND_DOCK", "TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM",   # v0.1.48: + SWAY_SEARCH
              "TURN_IN", "ENTER", "DOCKED", "REPORT_DOCK", "AIM_FIRE", "REPORT_FIRE", "UNDOCK", "DONE"]
HALF_BEAM = 0.91 / 2
SD, SD2 = __import__("optiprimex.usv.dockpath", fromlist=["x"]).standoff_lines()   # v0.1.56 (D6): 5.70 / 5.00 m from the dock geometry


def _pontoon_clearance(track):
    """Smallest lateral distance from the hull centreline to a pontoon finger while level with the fingers (x 41.5 .. 45)."""
    fingers = [o for o in W.load(POOL)["objects"] if o["kind"] == "pontoon"]
    d = 1e9
    for _, x, y, _, _ in track:
        if 41.5 <= x <= 45.0:
            d = min(d, min(abs(y - f["y_e"]) for f in fingers))
    return d


def test_m3t1_fast_sil_docks_fires_and_undocks():
    r = FastSil(POOL, "T3", {}, start=NEAR_DOCK).run(400)
    assert r["state"] == "DONE", (r["state"], r["reason"], r["logs"][-5:])
    assert r["states"] == T3_STATES, r["states"]
    xs = [x for _, x, _, _, s in r["track"] if s in ("DOCKED", "REPORT_DOCK", "AIM_FIRE")]
    ys = [y for _, _, y, _, s in r["track"] if s in ("DOCKED", "REPORT_DOCK", "AIM_FIRE")]
    assert xs and 43.1 <= min(xs) and max(xs) <= 43.8 and all(abs(y - 5.5) < 0.6 for y in ys)      # 1.5 m off the face of bay 2
    assert _pontoon_clearance(r["track"]) > HALF_BEAM + 0.2
    sway = [(x, y, h) for _, x, y, h, s in r["track"] if s == "SWAY_TO_ABEAM"]
    assert sway and all(abs(x - (45.0 - SD)) < 0.8 for x, _, _ in sway[3:])                  # along the standoff line (v0.1.56: derived)
    ev = {e["event"]: e for e in r["events"]}
    assert ev["docking"]["bay"] == 2
    assert ev["firefighting"]["result"] == "green" and ev["firefighting"]["complete"] and ev["firefighting"]["seen_red"]   # v0.1.48: window = upper | lower
    assert r["dock"]["phase"] in ("green", "done") and r["aimed"] >= 1
    end = r["track"][-1]
    assert 44.9 - end[1] >= SD - 0.3                                                          # backed out to the standoff
    assert any("bay 2 (green)" in l for l in r["logs"])


def test_m3t1_red_bay_by_hand_fires_out_the_time():
    r = FastSil(POOL, "T3", {"bay": 3, "fire_s": 8.0, "bay_recheck": False}, start=NEAR_DOCK).run(400)   # v0.1.48: no BACK_OUT (test_v0148)
    assert r["state"] == "DONE", (r["state"], r["logs"][-5:])
    ff = [e for e in r["events"] if e["event"] == "firefighting"][0]
    assert ff["result"] == "not seen green" and not ff["complete"] and 7.5 <= ff["fired_s"] <= 9.0
    ys = [y for _, _, y, _, s in r["track"] if s == "DOCKED"]
    assert ys and all(abs(y - 9.0) < 0.6 for y in ys)                                         # in bay 3
    assert r["dock"]["phase"] == "idle"                                                       # the RED bay's window never lights


def test_m3t1_without_a_dock_in_view_safe_holds():
    r = FastSil(POOL, "T3", {}, start={"x_n": 2.0, "y_e": -8.0, "hdg_deg": 180.0}).run(200)
    assert r["state"] == "SAFE_HOLD" and r["sub_state"] == "FIND_DOCK" and "GREEN" in r["safe_holds"][0], (r["state"], r["safe_holds"])


def test_m0t3_sways_along_the_dock_line_and_between_points():
    r = FastSil(POOL, "TEST_SWAY", {}, start={"x_n": 34.0, "y_e": 4.0, "hdg_deg": 0.0}).run(300)   # all three bays in view
    assert r["state"] == "DONE", (r["state"], r["logs"][-4:])
    assert "the dock line (3 bays)" in r["logs"][0] and "to bay 3" in r["logs"][0], r["logs"][0]
    sway = [(x, y, h) for _, x, y, h, s in r["track"] if s == "SWAY"]
    moving = sway[len(sway) // 3:]
    assert all(abs(x - (45.0 - SD)) < 0.8 for x, _, _ in moving) and all(min(h, 360 - h) < 15 for _, _, h in moving)
    assert abs(r["track"][-1][2] - 9.0) < 0.8
    o = W.WorldFrame({"lat": CENTRE[0], "lon": CENTRE[1]})
    pts = [list(o.to_latlon(10.0, 0.0)), list(o.to_latlon(10.0, -6.0))]                        # A -> B westwards, starboard to B: bow south
    r = FastSil(POOL, "TEST_SWAY", {"points": pts}, start={"x_n": 6.0, "y_e": 0.0, "hdg_deg": 0.0}).run(300)
    assert r["state"] == "DONE" and "two points" in r["logs"][0], r["logs"]
    sway = [(x, y, h) for _, x, y, h, s in r["track"] if s == "SWAY"][-10:]
    assert all(abs(h - 180) < 15 for _, _, h in sway) and abs(r["track"][-1][2] + 6.0) < 0.8


def test_t3_wiring():
    from optiprimex.ocs.cmd_authority import MISSION_TASKS
    from optiprimex.tools import ocs_cli, playbook
    from optiprimex.usv.tasks import REGISTRY
    assert {"T3", "TEST_SWAY"} <= set(REGISTRY) and {"T3", "TEST_SWAY"} <= set(MISSION_TASKS)
    assert playbook.MISSIONS["m3t1"].status == "built" and playbook.MISSIONS["m0t3"].status == "test"
    assert playbook.mission_stage_names("m3t1")[:3] == ["WAIT_START", "FIND_DOCK", "TO_LINE"]
    assert ocs_cli.intent_spec(ocs_cli.parse(["start_task", "m3t1sim", "--bay", "3"]))[:3] == ("START_TASK", "T3", {"tier": "core", "sim": True, "bay": 3})


# ---- live --------------------------------------------------------------------------------------------------------------
NS = "team/optiprime/v1"


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    import paho.mqtt.client as mqtt
    tmp = tmp_path_factory.mktemp("t3")
    lat, lon = W.WorldFrame({"lat": CENTRE[0], "lon": CENTRE[1]}).to_latlon(NEAR_DOCK["x_n"], NEAR_DOCK["y_e"])
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(make_usv_config(tmp, brokers, fake_fcu))], tmp / "mm.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#", "rc/#"]).start()
    boat = Probe(brokers.boat_port, ["sim/#", "local/usv/cannon/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15, include_past=True), mm.log()
    fake_fcu.ctrl({"teleport": {"lat": lat, "lon": lon, "hdg": 0.0}})     # after the mission manager took its origin at the start
    fp = Proc([PY, "-m", "optiprimex.sim.fake_perception", "--world", POOL, "--port", str(brokers.boat_port), "--hub-port", str(brokers.hub_port),
               "--seed", "3"], tmp / "fp.log")
    fd = Proc([PY, "-m", "optiprimex.sim.fake_dock", "--world", POOL, "--port", str(brokers.boat_port)], tmp / "fd.log")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--start-delay", "1.0", "--log", str(tmp / "rc.jsonl")], tmp / "rc.log")
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(make_ocs_config(tmp, brokers))], tmp / "ocs.log")
    from optiprimex.tools.ocs_cli import OcsConsole
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 15, include_past=True), ocs.log()
    beat = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t3-ui-beat", protocol=mqtt.MQTTv311)
    beat.connect("127.0.0.1", brokers.boat_port)
    beat.loop_start()
    beat.publish("presence/usv_ui", json.dumps({"node": "usv_ui", "up": True, "pid": 1, "fcu_link": "fake", "t_unix": time.time()}),
                 qos=0, retain=True).wait_for_publish(2)
    yield {"mm": mm, "probe": probe, "boat": boat, "con": con, "fcu": fake_fcu, "fd": fd}
    beat.publish("presence/usv_ui", None, qos=0, retain=True).wait_for_publish(2)
    beat.loop_stop(); beat.disconnect()
    con.stop(); probe.stop(); boat.stop()
    for p in (ocs, rc, fd, fp, mm):
        p.kill()


def test_m3t1sim_live_dock_fire_report(stack):
    con, p, boat = stack["con"], stack["probe"], stack["boat"]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("fcu_link") == "fake", 10)
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10)
    assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15)
    r = con.intent("START_TASK", "T3", args={"tier": "core", "sim": True}, wait_s=5)
    assert [a["body"]["result"] for a in r["acks"]][-1] == "DONE", [a["body"] for a in r["acks"]]
    assert p.wait(lambda m: m["topic"].endswith("event/usv/docking") and m["env"]["body"]["bay"] == 2, 200), stack["mm"].log()[-2500:]   # v0.1.48: + the sway search
    assert boat.wait(lambda m: m["topic"] == "sim/dock/state" and m["env"]["body"]["phase"] == "lit", 10, include_past=True), stack["fd"].log()
    assert boat.wait(lambda m: m["topic"] == "local/usv/cannon/aim" and m["env"]["body"].get("on") is True
                     and m["env"]["body"].get("window_id") in (1, 2), 15, include_past=True)           # v0.1.48: the window id on the aim
    assert p.wait(lambda m: m["topic"].endswith("event/usv/firefighting") and m["env"]["body"]["result"] == "green"
                  and m["env"]["body"]["window_id"] in (1, 2), 40, include_past=True), stack["mm"].log()[-2000:]
    assert boat.wait(lambda m: m["topic"] == "local/usv/cannon/aim" and m["env"]["body"].get("on") is False, 10, include_past=True)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                  and m["env"]["body"]["current_task"] == "TASK_NONE", 60), stack["mm"].log()[-2000:]

    def stats():
        m = p.last("rc/sim/stats")
        return json.loads(m["payload"]) if m else {}
    assert wait_until(lambda: {"docking", "firefighting"} <= {x["kind"] for x in stats().get("reports", [])}, 10), stats().get("reports")
    assert "TASK_COORDINATED_LOGISTICS" in stats()["tasks_seen"]["USV1"]
