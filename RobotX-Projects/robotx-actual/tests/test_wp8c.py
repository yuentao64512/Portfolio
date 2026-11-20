"""ROADMAP WP8c (v0.1.30): the go-home override (HOME -> RETURN_HOME, any AUTO state, with or without a run; MANUAL refuses), the
m0t6 orbit test (TEST_ORBIT round a saved point / a map click), and the pool's guards (m0t2 refused at once without points or with
unknown names; `--names "1 2 3"` split; names checked at the OCS against the saved points; save_point usage; the m4t0 note)."""
from __future__ import annotations

import json
import math

import pytest

from optiprimex.companion.config import Config
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import ocs_cli, ocs_web, playbook
from optiprimex.usv import points_store

from .conftest import PY, ROOT, Probe, Proc, make_ocs_config, make_usv_config, wait_until

NS = "team/optiprime/v1"
EMPTY = str(ROOT / "profiles" / "bench" / "world_empty.yaml")


# ---- the tasks in simulated time --------------------------------------------------------------------------------------------
def test_m0t6_orbits_the_centre_and_ends_in_hold():
    s = FastSil(EMPTY, "TEST_ORBIT", {"x_n": 15.0, "y_e": 0.0, "r_m": 4.0, "direction": "cw", "turns": 1, "speed_mps": 0.8})
    r = s.run(max_s=300)
    assert r["state"] == "DONE", r["reason"]
    orbit = [(x, y) for _, x, y, _, st in r["track"] if st == "ORBIT"]
    assert len(orbit) > 20
    radii = [math.hypot(x - 15.0, y) for x, y in orbit[5:]]
    assert max(abs(rr - 4.0) for rr in radii) < 1.0, (min(radii), max(radii))       # on the circle
    ang = [math.atan2(y, x - 15.0) for x, y in orbit]
    turn = sum(math.atan2(math.sin(b - a), math.cos(b - a)) for a, b in zip(ang, ang[1:]))
    assert turn > 0 and abs(turn) >= 1.9 * math.pi                                     # clockwise from above (bearing increasing), a full turn
    s2 = FastSil(EMPTY, "TEST_ORBIT", {"x_n": 15.0, "y_e": 0.0, "r_m": 4.0, "direction": "ccw", "turns": 0.5, "speed_mps": 0.8})
    r2 = s2.run(max_s=300)
    ang2 = [math.atan2(y, x - 15.0) for _, x, y, _, st in r2["track"] if st == "ORBIT"]
    assert r2["state"] == "DONE" and sum(math.atan2(math.sin(b - a), math.cos(b - a)) for a, b in zip(ang2, ang2[1:])) < 0


def test_m0t6_prechecks_and_entry():
    s = FastSil(EMPTY, "TEST_ORBIT", {})
    assert "no centre" in s.task.precheck(s.ctx())
    s = FastSil(EMPTY, "TEST_ORBIT", {"names": ["NOPE"]})
    assert "not in" in s.task.precheck(s.ctx())
    s = FastSil(EMPTY, "TEST_ORBIT", {"x_n": 15.0, "y_e": 0.0, "direction": "up"})
    assert "cw | ccw" in s.task.precheck(s.ctx())
    s = FastSil(EMPTY, "TEST_ORBIT", {"x_n": 15.0, "y_e": 0.0, "from": "ORBIT"})
    assert "off the circle" in s.from_refused                                           # the boat is 15 m away: APPROACH first
    lat, lon = s.origin.from_local(15.0, 0.0)
    s = FastSil(EMPTY, "TEST_ORBIT", {"lat": lat, "lon": lon})
    assert s.task.precheck(s.ctx()) == "" and abs(s.task.centre(s.ctx())[0][0] - 15.0) < 0.05


def test_return_home_slow_then_done():
    s = FastSil(EMPTY, "RETURN_HOME", {})
    lat, lon = s.origin.from_local(-8.0, 3.0)
    s.task.args.update(lat=lat, lon=lon)
    r = s.run(max_s=200)
    assert r["state"] == "DONE" and r["states"][-1] == "DONE"
    x, y = r["track"][-1][1], r["track"][-1][2]
    assert math.hypot(x + 8.0, y - 3.0) < 2.0
    assert abs(s.task.p["speed_mps"] - 0.3) < 1e-9 and s.task.behaviour_args("GO", s.ctx())["speed_mps"] == 0.3


# ---- the OCS side -----------------------------------------------------------------------------------------------------------
def test_names_split_checked_and_the_page_parts():
    assert ocs_cli.split_names("1 2, 3,,4") == ["1", "2", "3", "4"]
    assert ocs_cli.intent_spec(ocs_cli.parse(["start_task", "m0t2", "--names", "1 2 3"]))[2]["names"] == ["1", "2", "3"]
    spec = ocs_cli.intent_spec(ocs_cli.parse(["mission", "m0t6", "--centre", "A", "--radius", "4", "--dir", "ccw", "--turns", "2", "--speed", "0.4"]))
    assert spec[0] == "MISSION" and spec[2] == {"task": "TEST_ORBIT", "auto": False, "names": ["A"], "r_m": 4.0, "direction": "ccw", "turns": 2.0, "speed_mps": 0.4}
    assert ocs_cli.intent_spec(ocs_cli.parse(["home"]))[0] == "HOME"
    assert "X not saved" in ocs_cli.check_names(["1", "X"], {"1", "2"}) and ocs_cli.check_names(["1"], None) is None
    web = ocs_web.Web(ocs_web.parse_args([]), env={}, feeds=False)
    web.m.map = {"points": {"1": [0, 0], "2": [5, 0]}}
    r = web.command("start_task m0t2 --names 1,2,3")
    assert not r["ok"] and r["lines"][0].startswith("not sent: point 3 not saved")
    r = web.command("save_point")
    assert r["lines"][0].startswith("cannot parse") and r["lines"][1].startswith("usage: save_point NAME")
    meta = web._meta
    m6 = next(m for m in meta["missions"] if m["code"] == "m0t6")
    assert m6["task"] == "TEST_ORBIT" and {o["flag"] for o in m6["options"]} >= {"--centre", "--radius", "--dir", "--turns", "--speed", "--from"}
    assert "home" in dict(meta["groups"])["VEHICLE"] and meta["word_to_action"]["home"] == "HOME"
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'id="homebtn"' in html and "function qNeed" in html
    st = {"r": "R2", "vehicles": {"USV1": {"state": "MANUAL"}}}
    v = playbook.check("HOME", st, "USV1")
    assert not v.ok and "AUTO only" in v.why and "transmitter" in v.hint
    st["vehicles"]["USV1"]["state"] = "AUTO_TASK"
    assert playbook.check("HOME", st, "USV1").ok
    snap = {"hub_up": True, "status_age": 0.2, "ui_active": False, "perception_age": 0.5, "ocs": {"r": "R5", "judge_connected": True, "course_id": "C"},
            "status": {"state": "AUTO_TASK", "armed": True, "mode": "HOLD", "task_name": "T4", "task_state": "LISTEN"}}
    assert "assistance request" in ocs_web.clearance(snap)["why"]


# ---- live: the override and the prechecks through the mission manager -----------------------------------------------------------
@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("wp8c")
    pts = tmp / "points.json"
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config",
               str(make_usv_config(tmp, brokers, fake_fcu, **{"test_path.points_file": str(pts)}))], tmp / "mm.log")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--start-delay", "6.0", "--log", str(tmp / "rc.jsonl")], tmp / "rc.log")
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(make_ocs_config(tmp, brokers))], tmp / "ocs.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#"]).start()
    try:                                     # a failed set-up must not leave the processes behind (they attach to later tests' ports)
        assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] in ("MANUAL", "AUTO_HOLD"), 15, include_past=True), mm.log()
        assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 15, include_past=True), ocs.log()
    except BaseException:
        probe.stop()
        for p in (ocs, rc, mm):
            p.kill()
        raise
    con = ocs_cli.OcsConsole(port=brokers.hub_port)
    con.start()
    yield {"mm": mm, "probe": probe, "con": con, "pts": pts}
    con.stop(); probe.stop()
    for p in (ocs, rc, mm):
        p.kill()


def run(con, *argv):
    code, lines = ocs_cli.execute(con, ocs_cli.parse(list(argv) + ["--wait", "5"]))
    return code, " | ".join(lines)


def status(p):
    m = p.last(f"{NS}/status/usv")
    return m["env"]["body"] if m else {}


def test_home_override_live(stack):
    con, p, pts = stack["con"], stack["probe"], stack["pts"]
    code, out = run(con, "home")
    assert code != 0 and "AUTO only" in out, out                                      # MANUAL: the boat is the operator's
    assert run(con, "declare")[0] == 0
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] in ("R3", "R4", "R5"), 10)
    assert run(con, "enable_auto")[0] == 0
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10)
    # no run needed: the fake judge gives a RunStart only later (ocs_bench start delay) - home now, with the origin as HOME
    code, out = run(con, "home")
    assert code == 0 and "going home to the control origin" in out, out
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                  and m["env"]["body"].get("task_name") in (None, "RETURN_HOME"), 60), status(p)    # the boat starts at the origin: DONE soon
    # a saved HOME wins; a running task is aborted for it
    here = status(p)
    o = points_store.load(str(pts))
    assert "HOME" not in o
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 20)     # the fake judge's RunStart
    assert run(con, "start_task", "m0t1")[0] == 0
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_TASK", 10)
    lat0, lon0 = 1.2810602, 103.8550197                             # v0.1.33: Marina Bay (4.4 m N, 1.1 m E of the origin)
    assert run(con, "save_point", "HOME", "--lat", str(lat0), "--lon", str(lon0))[0] == 0
    code, out = run(con, "home", "--speed", "0.6")
    assert code == 0 and "TEST_AUTO aborted" in out and "the saved point HOME" in out and "0.6 m/s" in out, out
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("task_name") == "RETURN_HOME"
                  and m["env"]["body"]["state"] == "AUTO_TASK", 10, include_past=True)
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                  and m["env"]["body"].get("task_name") in (None, "RETURN_HOME"), 90), status(p)
    assert here is not None


def test_prechecks_refuse_at_once_live(stack):
    con, p, pts = stack["con"], stack["probe"], stack["pts"]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 30)
    code, out = run(con, "start_task", "m0t2", "--names", "1 2 3")                  # the pool's line: three names, none saved
    assert code != 0 and "points 1, 2, 3 not in" in out, out
    code, out = run(con, "start_task", "m0t2")                                       # no names, an empty map
    assert code != 0 and "no --names and no gate in the map" in out, out
    code, out = run(con, "start_task", "m0t6")
    assert code != 0 and "no centre" in out, out
    assert status(p)["state"] == "AUTO_HOLD"                                          # nothing moved
