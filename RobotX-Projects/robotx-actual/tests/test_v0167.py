"""v0.1.67 - borrowed from the UAV team's code (docs/TODO_UAV_BORROWINGS.md, Dmitrii 2026-11-08: "work on the todo borrowings during this
overnight session"): B1 the full course fingerprint for the boat's R2 rule, B3 the declared-but-not-AUTO caution, B4 T4's readiness after
holding on station, B13 the field-lessons table, B14 the merge rule. QUICK: no broker."""
from __future__ import annotations

import math

from optiprimex.ocs.run_lifecycle import RunLifecycle
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import ocs_web as OW

from .conftest import ROOT

C = {"course_id": "A", "boundary": [[1.0, 2.0], [1.0, 2.1], [1.1, 2.1]], "uav_transit_area": [], "pinger_hz": 30000}


def test_b1_the_course_fingerprint_counts_the_pinger_and_the_transit_area():
    lc = RunLifecycle()
    lc.on_link(True)
    assert lc.on_course(dict(C)) and lc.state == "COURSE_VALID"
    assert not lc._course_changed(dict(C, published_at=123.0, sent_at=9))                  # a retained republish: no change
    assert not lc._course_changed(dict(C, boundary=[tuple(p) for p in C["boundary"]]))   # tuples or lists: the same corners
    assert lc._course_changed(dict(C, pinger_hz=35000))                                  # was missed before v0.1.67
    assert lc._course_changed(dict(C, uav_transit_area=[[1.1, 2.1], [1.1, 2.2], [1.2, 2.2]]))
    assert lc._course_changed(dict(C, course_id="B"))
    lc.declare(["USV1"], {"T1": "TIER_CORE"}, [])
    assert lc.state == "DECLARED" and lc.on_course(dict(C, pinger_hz=35000)) and lc.state == "COURSE_VALID"   # a changed course = R2


def test_b3_declared_but_not_auto_is_a_caution_after_20_s():
    clk = [0.0]
    lc = RunLifecycle(clock=lambda: clk[0])
    lc.on_link(True)
    lc.on_course(dict(C))
    lc.declare(["USV1", "UAV1"], {"T1": "TIER_CORE"}, [])
    lc.vehicle_status("USV1", "STATE_AUTO")
    clk[0] = 25.0
    b = lc.body()
    assert b["r"] == "R3" and b["declared_s"] == 25.0
    w = OW.auto_warnings({"ocs": b})
    assert len(w) == 1 and "UAV1 not reporting STATE_AUTO" in w[0]["text"] and OW.AlarmBook.level_of(w[0]) == "CAUTION"
    clk[0] = 10.0
    assert OW.auto_warnings({"ocs": lc.body()}) == []                                    # not yet 20 s
    lc.vehicle_status("UAV1", "STATE_AUTO")
    assert lc.state == "READY" and OW.auto_warnings({"ocs": lc.body()}) == []            # every one AUTO: R4, nothing to say


def _m4t0():
    w = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
    f = FastSil(w, "T4", {}, seed=3)
    pz = f.pose()
    lat, lon = f.origin.from_local(pz.x_n + 12.0, pz.y_e)
    f.task.on_incident({"kind": "assistance_request", "command_seq": 41, "position": {"lat": lat, "lon": lon}, "domain": "surface"}, f.ctx())
    return f


def test_b4_readiness_only_after_holding_on_station():
    f = _m4t0()
    r = f.run(max_s=200, until=lambda s: s.task.m.state == "WAIT_CONFIRM")
    assert r["state"] == "WAIT_CONFIRM", r["logs"][-5:]
    log = [l for l in r["logs"] if "on station" in l or "not settled" in l]
    assert log and "on station" in log[0], r["logs"][-6:]
    ready_t = next(t for k, st, t in r["event_log"] if k == "incident" and st == "READY")
    arrived = [p for p in r["track"] if p[4] == "READY"]
    assert arrived and ready_t - arrived[0][0] >= 2.5                                     # held >= 3 s (track sampled every 0.5 s)
    g = _m4t0()
    g.task.cfg.extra = dict(g.task.cfg.extra, t4={"hold_s": 0})
    r2 = g.run(max_s=200, until=lambda s: s.task.m.state == "WAIT_CONFIRM")
    assert r2["state"] == "WAIT_CONFIRM" and not any("on station" in l for l in r2["logs"])   # hold_s 0 = the old rule


def test_b13_b14_the_lessons_and_the_rule():
    h = (ROOT / "HANDOFF.md").read_text()
    assert "### 7.1 Field lessons" in h and h.count("\n| ") >= 14 and "tee -i" in h and "geofence_problems()" in h
    c = (ROOT / "CLAUDE.md").read_text()
    assert "integration, not improvement" in c and "HANDOFF §7.1" in c
    todo = (ROOT / "docs" / "TODO_UAV_BORROWINGS.md").read_text()
    for b in ("B1", "B3", "B4", "B13", "B14"):
        assert f"| {b} (v0.1.67 ✅) |" in todo


def test_quick_list_has_this_file():
    assert "tests/test_v0167.py" in (ROOT / "Makefile").read_text()
