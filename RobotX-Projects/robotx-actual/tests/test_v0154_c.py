"""v0.1.54 part C (Dmitrii's items 12-14): the SafePassageReport's buoys with their coordinates and BeaconStates end to end (T1 ->
report_mapper -> rc_wire -> RoboNation's rc-test decoder -> the judge page's list); the S6 planner bug ("couldn't find the start ...
then it found a dark buoy and started to orbit it": a far dark buoy read flashing blue and locked as the entry - both planners; and
the start ~50 m from Task 1 on Course Extra); the 0.60 m clearance rule (hull edge, gatepath.planner.Clearance) kept in fast-SIL
runs of m1t1 simple + s6, m3t1, m0t4 (and m4t0) with the documented exemptions."""
from __future__ import annotations

import json
import math
import os
import subprocess
import sys
import types
from pathlib import Path

import pytest

from optiprimex.companion.config import load
from optiprimex.ocs import rc_wire as WIRE
from optiprimex.ocs.report_mapper import ReportMapper, event_to_report
from optiprimex.sim import campaign as C
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import judge_web as JW
from optiprimex.usv.gatepath import planner as gp
from optiprimex.usv.tasks.t1_safe_passage import DEFAULTS, T1SafePassage

from .conftest import ROOT

EXTRA = C.world_file("extra")
TRUTH_BEACON = JW.TRUTH_BEACON


def _rc_repo():
    for p in (os.environ.get("RC_REPO"), ROOT.parent / "robocommand", ROOT / "robotx-uav-robocommand" / "robocommand"):
        if p and (Path(p) / "RobotX_2026" / "rc_test" / "test_server.py").exists():
            return Path(p)
    return None


def _truth(wd):
    return [o for o in wd["objects"] if o.get("kind") == "buoy" and o.get("sensed", True) is not False]


def _check_report(wd, rep):
    """Every truth buoy reported once within 1 m with its state; entry / exit within 1 m."""
    fr = W.WorldFrame(wd["origin"])
    truth = _truth(wd)
    hit = {}
    for b in rep["buoys"]:
        x, y = fr.to_world(b["lat"], b["lon"])
        o = min(truth, key=lambda o: math.hypot(o["x_n"] - x, o["y_e"] - y))
        d = math.hypot(o["x_n"] - x, o["y_e"] - y)
        assert d < 1.0, (o["id"], d)
        assert b["beacon"] == TRUTH_BEACON.get((o["colour"], o.get("light")), "BEACON_STATE_OFF"), (o, b)
        hit.setdefault(o["id"], []).append(b)
    assert sorted(hit) == sorted(o["id"] for o in truth) and all(len(v) == 1 for v in hit.values()), hit
    for k in ("entry", "exit"):
        o = next(o for o in truth if o.get("role") == k)
        x, y = fr.to_world(rep[k]["lat"], rep[k]["lon"])
        assert math.hypot(o["x_n"] - x, o["y_e"] - y) < 1.0, k


# ---- item 13: the S6 bug, both planners ------------------------------------------------------------------------------------------
@pytest.mark.parametrize("planner", ["s6", "simple"])
def test_extra_far_start_rejects_the_dark_buoy_and_circles_the_flashing_blue(planner):
    """The far start of campaign seed 4 on Course Extra: a dark obstacle collects far wrong-colour frames and reads blue, flashing -
    it is locked first (Dmitrii's 'found a dark buoy and started to orbit it'), the near sightings (never blue) reject it, the search
    goes on and the real FLASHING BLUE is circled; the run completes, no other buoy circled."""
    wd = W.load(EXTRA)
    new = FastSil(EXTRA, "T1", dict(C.T1_FAST, planner=planner), start=C.far_start(wd, 4), seed=4).run(600)
    assert new["state"] == "DONE" and C.grade_m1t1(wd["objects"], new)["ok"]            # v0.1.92: the dark buoy is CHECKED, never circled
    assert new["states"].count("CIRCLE_ENTRY_CW") == 1                                 # the dark buoy never circled (v0.1.92 FINAL ladder)
    r = FastSil(EXTRA, "T1", dict(C.T1_FAST, planner=planner, scout_entry_s=0, start_mode="straight", gate_steps=False),
                start=C.far_start(wd, 4), seed=4).run(600)   # v0.1.54's ladder
    g = C.grade_m1t1(wd["objects"], r)
    assert r["state"] == "DONE" and g["ok"], (g, r["logs"][-6:])
    # v0.1.92: the geofence keeps an outside dark buoy from ever being locked - the "not blue after all" line is no longer guaranteed
    assert r["states"][:4] == ["SEARCH_ENTRY", "APPROACH_LEAD", "SEARCH_ENTRY", "APPROACH_LEAD"] and "CIRCLE_ENTRY_CW" in r["states"]
    i = r["states"].index("CIRCLE_ENTRY_CW")
    assert r["states"][i + 1] in ("SEARCH_ENTRY", "TRANSIT")                     # NOT_ENTRY (the dark one locked first) or, fenced out, the real one
    entry = next(o for o in wd["objects"] if o.get("role") == "entry")
    assert C.winding(r["track"], (entry["x_n"], entry["y_e"]), "CIRCLE_ENTRY_CW") >= 330


@pytest.mark.parametrize("planner", ["s6", "simple"])
def test_extra_from_the_start_area_goes_to_the_search_point(planner):
    """Course Extra's start area is ~50 m from Task 1 (beyond the camera's 40 m bearings): without a search point the scan finds
    nothing (Dmitrii's 'couldn't find the start from where it was'); with t1.search_lat / search_lon the task drives there,
    finds the entry and completes."""
    wd = W.load(EXTRA)
    r0 = FastSil(EXTRA, "T1", dict(C.T1_FAST, planner=planner), start=wd["start"], seed=1).run(120)
    assert r0["state"] == "SAFE_HOLD" and "no flashing blue" in r0["safe_holds"][0]
    r = FastSil(EXTRA, "T1", dict(C.T1_FAST, planner=planner, **C.t1_hint(wd)), start=wd["start"], seed=1).run(600)
    assert r["state"] == "DONE" and C.grade_m1t1(wd["objects"], r)["ok"], r["logs"][-6:]
    assert any("to the search point" in l for l in r["logs"])
    _check_report(wd, r["reports"][-1])


def test_the_near_sightings_decide_not_the_map_colour():
    """A real flashing blue whose map vote wavers (green miscolours) is kept: rejection needs verify_n near sightings, none blue."""
    t = T1SafePassage(load(str(ROOT / "profiles" / "bench" / "usv_bench.yaml")))
    t.p, t.near, t.rejected = dict(DEFAULTS), {}, set()
    e = types.SimpleNamespace(id=7, colour="black", light="off", t_last=0.0, colour_now="black", x_n=1.0, y_e=0.0, colour_votes={})
    t.entry_id, t.exit_id = 7, None
    t.m = types.SimpleNamespace(state="CIRCLE_ENTRY_CW", handle=lambda ev, **k: setattr(t, "ev", ev))
    t._beh, t.circle_r, t.entry_xy = None, {}, (1.0, 0.0)
    logs = []
    m = types.SimpleNamespace(entities=lambda kind, **k: [e])
    ctx = types.SimpleNamespace(b=types.SimpleNamespace(map=m, pose=types.SimpleNamespace(x_n=0.0, y_e=0.0)), log=logs.append)
    for i in range(int(DEFAULTS["verify_n"]) - 1):
        e.t_last, e.colour_now = float(i), ("blue" if i == 3 else "black")
        t.watch_locked(ctx)
    assert 7 not in t.rejected                                                    # one blue frame among them: still the entry
    t.near[7] = [(float(i), "black") for i in range(int(DEFAULTS["verify_n"]) - 1)]
    e.t_last = 99.0
    t.watch_locked(ctx)
    assert 7 in t.rejected and t.ev == "NOT_ENTRY" and t.entry_id is None and "none blue" in logs[-1]


def test_the_campaign_has_the_check_that_would_have_caught_it():
    wd = W.load(EXTRA)
    dark = next(o for o in wd["objects"] if o.get("role") == "obstacle")
    track = [(k * 0.5, dark["x_n"] + 3.0 * math.cos(k * 0.1), dark["y_e"] + 3.0 * math.sin(k * 0.1), 0, "CIRCLE_ENTRY_CW") for k in range(70)]
    r = {"state": "DONE", "sub_state": "DONE", "reason": "", "safe_holds": [], "track": track}
    assert f"circled buoy {dark['id']} (black " in C.grade_m1t1(wd["objects"], r)["why"]       # (YAML reads `light: off` as False)
    cs = C.cases(["m1t1"], 1, ["pool", "extra"], ["s6"], ["geometric"], ("near", "far", "ring"))
    assert {(c["world"], c["start"]) for c in cs} == {("pool", "near"), ("pool", "ring"), ("extra", "near"), ("extra", "far"), ("extra", "ring")}
    src = (ROOT / "optiprimex" / "sim" / "campaign.py").read_text()
    assert 'default="pool,marina,extra"' in src and 'default="near,far,ring"' in src
    st = C.ring_start(wd, 3)
    assert all(math.hypot(st["x_n"] - o["x_n"], st["y_e"] - o["y_e"]) > 3.0 for o in _truth(wd))


# ---- item 12: the report, end to end ---------------------------------------------------------------------------------------------
def test_the_beacon_vocabulary():
    t = T1SafePassage(load(str(ROOT / "profiles" / "bench" / "usv_bench.yaml")))
    t.rejected, t.near, t.entry_id, t.exit_id = set(), {}, None, None

    def b(colour, light, i=1):
        return t.beacon(types.SimpleNamespace(id=i, colour=colour, light=light))
    assert b("black", "off") == b("black", "?") == "BEACON_STATE_OFF"
    assert b("red", "flash") == b("red", "steady") == b("red", "?") == "BEACON_STATE_FLASHING_RED"
    assert b("green", "?") == "BEACON_STATE_FLASHING_GREEN"
    assert b("blue", "flash") == "BEACON_STATE_FLASHING_BLUE" and b("blue", "steady") == "BEACON_STATE_STEADY_BLUE"
    assert b("blue", "?") == b("unknown", "?") == b("white", "steady") == "BEACON_STATE_UNKNOWN"
    t.entry_id, t.exit_id = 1, 2
    assert b("blue", "?", 1) == "BEACON_STATE_FLASHING_BLUE" and b("blue", "?", 2) == "BEACON_STATE_STEADY_BLUE"
    t.rejected, t.near = {3}, {3: [(0, "black")] * 8}
    assert b("blue", "flash", 3) == "BEACON_STATE_OFF"                            # the map still says blue: the near frames win
    assert WIRE.beacon_of("black", "flashing") == "BEACON_STATE_OFF" and WIRE.beacon_of("blue", "solid") == "BEACON_STATE_STEADY_BLUE"


@pytest.mark.parametrize("world", ["pool", "marina"])
def test_every_buoy_is_reported_with_its_state_and_position(world):
    wf = C.world_file(world)
    wd = W.load(wf)
    r = FastSil(wf, "T1", dict(C.T1_FAST, planner="s6"), start=C.jitter(wd["start"], 2), seed=2).run(600)
    assert r["state"] == "DONE"
    last = r["reports"][-1]
    assert last["complete"] is True
    _check_report(wd, last)                                                       # the stale ones too: the entry, out of view since
    times = [t for k, _st, t in r["event_log"] if k == "safe_passage"]
    assert len(times) >= 3 and all(b - a >= DEFAULTS["report_period_s"] - 0.11 for a, b in zip(times, times[1:-1]))   # no flood


def test_the_mapper_and_the_wire_keep_the_states_and_coordinates():
    body = {"event": "safe_passage", "entry": {"lat": 1.2812, "lon": 103.8546}, "exit": {"lat": 1.2813, "lon": 103.8547}, "complete": False,
            "buoys": [{"id": 1, "colour": "black", "state": "unknown", "beacon": "BEACON_STATE_OFF", "lat": 1.28121234, "lon": 103.85461234},
                      {"id": 2, "colour": "red", "state": "solid", "beacon": "BEACON_STATE_FLASHING_RED", "lat": 1.2813, "lon": 103.8548}]}
    key, rep = event_to_report("USV", body)
    d = WIRE.decode_report(WIRE.encode_report("SGIT", "USV", 9, rep))
    assert d["kind"] == "safe_passage" and [b["beacon"] for b in d["buoys"]] == ["BEACON_STATE_OFF", "BEACON_STATE_FLASHING_RED"]
    assert abs(d["buoys"][0]["lat"] - 1.28121234) < 1e-9 and abs(d["entry_position"]["lon"] - 103.8546) < 1e-9
    t = [0.0]
    sent = []
    m = ReportMapper(lambda v, b: sent.append(b) or len(sent), min_period_s=1.0, clock=lambda: t[0])
    for k in range(5):                                                            # five updates within a second: one now, the latest after
        t[0] = k * 0.2
        m.on_event("USV", dict(body, buoys=body["buoys"][:1 + k % 2]))
    t[0] = 1.05
    m.tick()
    assert len(sent) == 2 and len(sent[-1]["buoys"]) == 1


@pytest.mark.skipif(_rc_repo() is None, reason="RoboNation's robocommand checkout not found (RC_REPO)")
def test_roboNations_rc_test_decodes_our_report(tmp_path):
    """The SIL's last SafePassageReport -> report_mapper -> rc_wire -> rc-test's own parse_message (test_server.py, unmodified, in a
    process of its own): the same BeaconStates, lat / lon to 1e-6 deg, entry / exit set."""
    wd = W.load(EXTRA)
    r = FastSil(EXTRA, "T1", dict(C.T1_FAST, planner="s6", **C.t1_hint(wd)), start=wd["start"], seed=1).run(600)
    _key, rep = event_to_report("USV", r["reports"][-1])
    (tmp_path / "rep.bin").write_bytes(WIRE.encode_report("SGIT", "USV", 3, rep))
    rc = _rc_repo() / "RobotX_2026"
    code = r'''
import json, sys
from optiprimex.tools import rctest_manual as M
ts = M.load(sys.argv[1])
from robotx import rx_common_pb2
kind, msg = ts.parse_message("robocommand/robotx/SGIT/USV/report", open(sys.argv[2], "rb").read())
sp = msg.safe_passage
print(json.dumps({"kind": kind, "body": msg.WhichOneof("body"), "entry": [sp.entry_position.latitude, sp.entry_position.longitude],
                  "exit": [sp.exit_position.latitude, sp.exit_position.longitude],
                  "buoys": [[b.position.latitude, b.position.longitude, rx_common_pb2.BeaconState.Name(b.state)] for b in sp.buoys]}))
'''
    env = dict(os.environ, LOG_DIR=str(tmp_path / "logs"), PYTHONDONTWRITEBYTECODE="1",
               PYTHONPATH=os.pathsep.join([str(rc / "gen" / "python"), str(ROOT)]))
    p = subprocess.run([sys.executable, "-c", code, str(rc / "rc_test" / "test_server.py"), str(tmp_path / "rep.bin")],
                       capture_output=True, text=True, timeout=60, env=env, cwd=str(tmp_path))
    assert p.returncode == 0, p.stderr[-800:]
    got = json.loads(p.stdout.strip().splitlines()[-1])
    assert got["kind"] == "RxReport" and got["body"] == "safe_passage"
    assert [b[2] for b in got["buoys"]] == [b["beacon"] for b in rep["buoys"]]
    for g, b in zip(got["buoys"], rep["buoys"]):
        assert abs(g[0] - b["lat"]) < 1e-6 and abs(g[1] - b["lon"]) < 1e-6
    assert abs(got["entry"][0] - rep["entry_position"]["lat"]) < 1e-6 and abs(got["exit"][1] - rep["exit_position"]["lon"]) < 1e-6
    _check_report(wd, {"buoys": [{"lat": a, "lon": o, "beacon": s} for a, o, s in got["buoys"]],
                       "entry": {"lat": got["entry"][0], "lon": got["entry"][1]}, "exit": {"lat": got["exit"][0], "lon": got["exit"][1]}})


def test_the_judge_page_lists_the_last_reports_buoys():
    wd = W.load(C.world_file("pool"))
    fr = W.WorldFrame(wd["origin"])
    b1 = next(o for o in wd["objects"] if o["id"] == 3)
    lat, lon = fr.to_latlon(b1["x_n"] + 0.3, b1["y_e"])
    snap = {"vehicles": {"USV": {"reports": {"safe_passage": {"kind": "safe_passage", "ago_s": 2.0, "entry_position": {"lat": lat, "lon": lon},
                                                               "exit_position": None,
                                                               "buoys": [{"lat": lat, "lon": lon, "beacon": "BEACON_STATE_FLASHING_GREEN"}]}}}}}
    world = {"name": "world_pool", "origin": wd["origin"], "objects": wd["objects"]}
    p = JW.passage_buoys(snap, world)[0]
    row = p["buoys"][0]
    assert row["truth_id"] == 3 and row["truth_beacon"] == "BEACON_STATE_FLASHING_RED" and abs(row["d_m"] - 0.3) < 0.02
    assert p["exit"] is None and p["entry"]["d_m"] > 5 and len(p["missing"]) == 9 and p["truth"] == "world_pool"
    assert JW.passage_buoys(snap, None)[0]["buoys"][0].get("d_m") is None                     # no simulator: the list without truth
    assert "SAFE PASSAGE · THE LAST REPORT'S BUOYS" in JW.PAGE and "drawPassage(s.passage)" in JW.PAGE


# ---- item 14: the 0.60 m clearance rule ------------------------------------------------------------------------------------------
def test_the_planner_keeps_the_rule_round_objects():
    clr = gp.Clearance()
    assert abs(clr.keep(0.25) - (0.25 + 0.38 + 0.60 + clr.track_m)) < 1e-9
    obs = [(10.0, 0.0), (10.0, 2.0), (10.0, -2.0), (5.0, 0.5)]
    path = gp.avoid([(0.0, 0.0), (20.0, 0.0)], obs, clr=clr)
    assert len(path) > 2 and gp.path_clearance(gp.resample(path, 0.1), obs, clr) >= clr.clear_m + clr.track_m - 0.01
    assert gp.avoid([(0.0, 5.0), (20.0, 5.0)], obs, clr=clr) == [(0.0, 5.0), (20.0, 5.0)]          # clear already: unchanged
    r, edge = gp.circle_radius((0.0, 0.0), [(3.0, 0.0)], 3.0, clr)
    assert abs(r - 3.0) > 1.5 and edge >= 0.6                                    # a buoy on the 3 m ring: another radius
    q = gp.free_point((10.0, 0.1), obs[:2], clr)
    assert all(math.hypot(q[0] - o[0], q[1] - o[1]) >= clr.keep() for o in obs[:2])
    for f in ("pool/usv_pool.yaml", "pool/usv_lab.yaml", "course/usv_course.yaml", "bench/usv_bench.yaml"):
        assert load(str(ROOT / "profiles" / f)).extra["t1"]["clear_m"] == 0.60, f
    assert DEFAULTS["clear_m"] == 0.60 and DEFAULTS["hull_beam_m"] == 0.76 and DEFAULTS["hull_len_m"] == 1.19


def test_the_hull_measure_and_its_exemptions():
    wd = W.load(C.world_file("pool"))
    b = next(o for o in wd["objects"] if o["id"] == 9)                            # a dark obstacle
    side = [(0.0, b["x_n"] - 2.0, b["y_e"] + 0.25 + 0.38 + 0.5, 0, "TRANSIT"), (1.0, b["x_n"] + 2.0, b["y_e"] + 0.25 + 0.38 + 0.5, 0, "TRANSIT")]
    h = C.hull_clearance(wd, side, "m1t1")
    assert abs(h["min_m"] - 0.5) < 0.02 and not h["ok"] and "buoy 9" in h["at"]
    g = [o for o in wd["objects"] if o.get("gate") == 1]
    mid = ((g[0]["x_n"] + g[1]["x_n"]) / 2, (g[0]["y_e"] + g[1]["y_e"]) / 2)
    cross = [(0.0, mid[0] - 2.0, mid[1], 0, "TRANSIT"), (1.0, mid[0] + 2.0, mid[1], 0, "TRANSIT")]
    h = C.hull_clearance(wd, cross, "m1t1")                                       # the 2.4 m gate on its centre line: exempt, reported
    assert h["ok"] and 0.5 < h["exempt_min_m"] < 0.6 and "gate 2.4 m" in h["exempt_at"]
    assert not C.hull_clearance(wd, [(t, x, y, h_, "EXIT") for t, x, y, h_, _ in cross], "m1t1")["ok"]      # not crossing it: counts
    dock = [(0.0, 44.3, 5.5, 0, "DOCKED"), (1.0, 44.3, 5.5, 0, "DOCKED")]                 # the bow 0.1 m off bay 2's face
    assert C.hull_clearance(wd, dock, "m3t1")["ok"]                               # inside the bay: the dock is exempt
    assert not C.hull_clearance(wd, [(t, x, y, h_, "SWAY_SEARCH") for t, x, y, h_, _ in dock], "m3t1")["ok"]
    assert "min clr (exempt)" in C.table([dict(mission="m1t1", world="pool", planner="s6", perception="geometric", seed=1, ok=True,
                                               t_s=90.0, clr_m=0.7, clr_exempt_m=0.55)])


@pytest.mark.parametrize("case", [
    {"mission": "m1t1", "world": "pool", "planner": "simple", "perception": "geometric", "seed": 1, "start": "near"},
    {"mission": "m1t1", "world": "pool", "planner": "s6", "perception": "geometric", "seed": 2, "start": "near"},
    {"mission": "m1t1", "world": "extra", "planner": "s6", "perception": "geometric", "seed": 3, "start": "ring"},
    {"mission": "m1t1", "world": "marina", "planner": "simple", "perception": "geometric", "seed": 1, "start": "far"},
    {"mission": "m3t1", "world": "pool", "planner": "-", "perception": "geometric", "seed": 1},
    {"mission": "m0t4", "world": "boxes", "planner": "-", "perception": "geometric", "seed": 2},
    {"mission": "m4t0", "world": "pool", "planner": "-", "perception": "geometric", "seed": 8},
], ids=lambda c: f"{c['mission']}-{c['world']}-{c['planner']}-{c.get('start', '')}-s{c['seed']}")
def test_the_clearance_is_kept_in_the_fast_sil(case):
    r = C.run_case(case)
    assert r["ok"], (r["why"], r.get("logs"))
    assert r["clr_m"] >= 0.60, (r["clr_m"], r["clr_at"])
