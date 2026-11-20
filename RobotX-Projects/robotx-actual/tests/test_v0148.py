"""v0.1.48 (ROADMAP WP10j-b, docs/M3T1_SIM_PLAN.md): m3t1 through the camera. The dock in the perception map from the camera
(dock_face / window / indicator: cv/sim_observer.observe_dock -> cv_publish.dock_to_items -> PerceptionMap: the bay colour from the
indicator votes, windows numbered 1 upper / 2 lower with their light), T3's SWAY_SEARCH (the whole line, votes per bay, then commit;
a second pass at 3.5 m; hold) and BACK_OUT (a RED indicator on the way in; at most 2 re-engagements), the reports strictly in order,
the fake dock `auto: false` (the RED window lights on our DockingReport) and the window-matched aim (`window_id` on the aim topic),
the campaign's m3t1 (random starts, the strict grader), the sim-day sheet's S9, run_sim.sh's dock through the model."""
from __future__ import annotations

import importlib.util
import json
import math
import os

import pytest

from optiprimex.companion.config import Config
from optiprimex.companion.state import LocalPose, Observation
from optiprimex.sim import campaign as C
from optiprimex.sim import world as W
from optiprimex.sim.fake_dock import DockSim
from optiprimex.sim.fast_sil import FastSil
from optiprimex.usv.dockpath import standoff_lines
from optiprimex.usv.perception_map import PerceptionMap

from .conftest import ROOT

np = pytest.importorskip("numpy")
pytest.importorskip("PIL")

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
NEAR_DOCK = {"x_n": 32.0, "y_e": 3.0, "hdg_deg": 0.0}
SD, SD2 = standoff_lines()                       # v0.1.56 (D6): the standoff lines from the dock geometry (5.70 / 5.00 m)
MODEL = os.environ.get("CVT_MODEL", str(ROOT.parent / "CV_TRAIN" / "models" / "cvt-v2-l"))
T3_BOUNDARIES = ["FIND_DOCK", "TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "DOCKED", "REPORT_DOCK", "AIM_FIRE",
                 "REPORT_FIRE", "UNDOCK"]


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _reports(r):
    return [(k, st) for k, st, _ in r["event_log"] if k in ("docking", "firefighting")]


# ---- the camera: the dock parts ------------------------------------------------------------------------------------------------
def test_the_observer_measures_faces_windows_and_indicators():
    from optiprimex.sim.fast_sil import CameraPath
    wd = W.load(POOL)
    cp = CameraPath(wd, 3, (736, 414), "none", dock=True)
    cp.dock_state = {"green_bay": 2, "phase": "lit", "window": {"bay": 2, "window": 2, "colour": "red"}}
    pz = LocalPose(36.0, 5.5, 0.0, 0, 0, 0)
    cp.observe(pz, 0.0, 1.28, 103.85)
    parts = cp.last["dock"]
    faces = [p for p in parts if p["cls"] == "dock_face"]
    inds = sorted((p for p in parts if p["cls"] == "indicator"), key=lambda p: p["bearing_deg"])
    wins = [p for p in parts if p["cls"] == "window"]
    assert len(faces) == 3 and all(p["colour"] == "white" for p in faces)
    assert [p["colour"] for p in inds] == ["red", "green", "red"]                 # bay 1 (left) .. bay 3
    assert len(wins) == 6 and sorted(p["window"] for p in wins) == [1, 1, 1, 2, 2, 2]
    lit = [p for p in wins if p["light"] != "off"]
    assert len(lit) == 1 and lit[0]["light"] == "red" and lit[0]["window"] == 2
    for p in faces:                                                               # the face point ~9 m off, on the line x_n 45
        x = 36.0 + p["forward_m"]
        assert abs(x - 45.0) < 0.3 and 8.5 < p["range_m"] < 10.5
    pub = _load("cvpub_t148", "cv/publish/cv_publish.py")
    items = pub.dock_to_items(parts, 7)
    assert {i["class"] for i in items["dock"]} == {"dock", "indicator"} and len(items["window"]) == 6
    assert [i["class"] for i in items["dock"]][:3] == ["dock"] * 3                # faces first
    w2 = [i for i in items["window"] if i["colour"] == "red"]
    assert len(w2) == 1 and w2[0]["lit"] and w2[0]["quality"]["window"] == 2


def test_the_map_colours_a_face_from_its_indicator_and_keeps_windows_apart():
    m = PerceptionMap(Config())
    pz = LocalPose(40.0, 5.5, 0.0, 0, 0, 0)

    def ob(kind, items, t):
        return Observation.from_body({"t_cam": t, "kind": kind, "items": items})
    for k in range(6):
        t = 100.0 + k
        m.ingest(ob("dock", [{"class": "indicator", "colour": "red" if k < 2 else "green", "confidence": 0.9, "range_m": 5.0, "bearing_deg": 0.0},
                             {"class": "dock", "colour": "white", "confidence": 0.9, "range_m": 5.0, "bearing_deg": 0.0}], t), pz, now=t)
        m.ingest(ob("window", [{"class": "window", "colour": "black", "confidence": 0.9, "range_m": 5.0, "bearing_deg": -2.0, "quality": {"window": 1}},
                               {"class": "window", "colour": "red", "confidence": 0.9, "range_m": 5.0, "bearing_deg": 2.0, "quality": {"window": 2}}], t), pz, now=t)
    docks = m.entities("dock", include_young=True)
    assert len(docks) == 1 and docks[0].colour == "green" and docks[0].ind_votes == {"red": 2, "green": 4}   # the face sorted first: 6 votes
    assert [c for _, c in docks[0].ind_hist] == ["red", "red", "green", "green", "green", "green"]
    wins = sorted(m.entities("window", include_young=True), key=lambda e: e.y_e)
    assert len(wins) == 2 and [w.window_no for w in wins] == [1, 2] and [w.light for w in wins] == ["off", "red"]   # 0.35 m apart, not merged
    d = m.digest(pz, now=106.0)
    assert any(e.get("votes") == {"red": 2, "green": 4} for e in d["entities"]) and sorted(e.get("window") for e in d["entities"] if e["kind"] == "window") == [1, 2]
    m.ingest(ob("dock", [{"class": "indicator", "colour": "red", "confidence": 0.9, "range_m": 5.0, "bearing_deg": 40.0}], 107.0), pz, now=107.0)
    assert docks[0].ind_votes["red"] == 2                                          # an indicator with no face near it: no vote


# ---- the fake dock -----------------------------------------------------------------------------------------------------------
def test_the_fake_dock_lights_on_the_report_and_wants_the_lit_window():
    wd = W.load(POOL)
    assert wd["dock"]["auto"] is False
    sim = DockSim(wd, seed=1)
    assert sim.auto is False and DockSim(wd, 1, auto=True).auto is True
    t = 0.0

    def run(secs, pose, aim=None):
        nonlocal t
        for _ in range(int(secs * 10)):
            t += 0.1
            sim.tick(t, pose, aim, 0.05)
    run(5, {"x_n": 43.5, "y_e": 5.5, "hdg_deg": 0})                                 # in the GREEN bay, slow: the dwell is off
    assert sim.phase == "idle"
    assert "ignored" in sim.command({"cmd": "docked", "bay": 1}, t) and sim.phase == "idle"   # a report for a RED bay lights nothing
    sim.command({"cmd": "docked", "bay": 2}, t)
    assert sim.phase == "lit" and sim.window in (1, 2)
    wrong = 3 - sim.window
    run(6, None, {"on": True, "bay": 2, "window": "upper", "window_id": wrong})     # the wrong window never turns GREEN
    assert sim.phase == "lit"
    run(6, None, {"on": True, "bay": 2, "window_id": None})                         # at the bay centre: neither
    assert sim.phase == "lit"
    run(4.2, None, {"on": True, "bay": 2, "window_id": sim.window})
    assert sim.phase == "green"
    sim.command({"cmd": "reset"}, t)
    sim.command({"cmd": "docked"}, t)
    run(4.2, None, {"on": True, "bay": 2})                                          # a hand-made aim without the key: the bay alone
    assert sim.phase == "green"
    from optiprimex.sim import fake_dock as FD
    assert FD.parse_args([]).auto == "world" and FD.parse_args(["--auto", "on"]).auto == "on"


# ---- T3: the sway search, the reports in order, the aim ------------------------------------------------------------------------
def test_m3t1_sways_the_whole_line_then_reports_in_order():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    r = s.run(500)
    assert r["state"] == "DONE", (r["state"], r["logs"][-5:])
    assert r["states"] == T3_BOUNDARIES + ["DONE"]
    (p,) = r["search"]
    assert p["chosen"] == 2 and p["standoff_m"] == SD and all(v["red"] + v["green"] >= 3 for v in p["votes"].values())
    assert p["votes"][2]["green"] > 0 and p["votes"][2]["red"] == 0 and p["votes"][1]["green"] == 0
    sweep = [y for _, x, y, _, st in r["track"] if st == "SWAY_SEARCH"]
    assert min(sweep) < 1.0 and max(sweep) > 9.8                                   # past all three bays (faces at E 2 / 5.5 / 9)
    assert all(abs(x - (45.0 - SD)) < 0.8 for _, x, _, _, st in r["track"][5:] if st == "SWAY_SEARCH")
    assert _reports(r) == [("docking", "REPORT_DOCK"), ("firefighting", "REPORT_FIRE")]
    ev = {e["event"]: e for e in r["events"]}
    lit = r["dock"]["window"]["window"]
    assert ev["docking"]["bay"] == 2 and ev["firefighting"]["window_id"] == lit and ev["firefighting"]["window"] == {1: "upper", 2: "lower"}[lit]
    assert ev["firefighting"]["complete"] and ev["firefighting"]["result"] == "green"
    on = [a for a in s.aims if a.get("on")]
    assert on and all(a["window_id"] == lit and a["bay"] == 2 for a in on)


def test_m3t1_the_camera_dock_and_a_green_bay_three():
    s = FastSil(POOL, "T3", {}, start={"x_n": 30.0, "y_e": 0.0, "hdg_deg": 200.0}, perception="camera", cam_dock=True, cam_size=(552, 310), seed=5)
    s.dock.command({"cmd": "green_bay", "bay": 3}, s.t)
    assert {o["kind"] for o in s.fp.objects}.isdisjoint({"buoy", "dock", "window"})   # the dock only from the camera
    r = s.run(500)
    g = C.grade_m3t1(W.load(POOL)["objects"], r, green_bay=3, lit_window=s.dock.window, strict=True)
    assert g["ok"], (g["why"], r["logs"][-6:])
    assert any(e["event"] == "docking" and e["bay"] == 3 for e in r["events"])


def test_a_wrong_call_backs_out_and_searches_again():
    r = FastSil(POOL, "T3", {"bay": 1}, start=NEAR_DOCK).run(500)                 # forced RED bay 1 on a GREEN-2 world
    assert r["state"] == "DONE", (r["state"], r["logs"][-5:])
    st = r["states"]
    assert st[:7] == ["FIND_DOCK", "TO_LINE", "SWAY_TO_ABEAM", "TURN_IN", "BACK_OUT", "SWAY_SEARCH", "SWAY_TO_ABEAM"] or \
        st[:8] == ["FIND_DOCK", "TO_LINE", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "BACK_OUT", "SWAY_SEARCH", "SWAY_TO_ABEAM"], st
    assert r["reengaged"] == 1 and r["search"][-1]["excluded"] == [1] and r["search"][-1]["chosen"] == 2
    assert [e["bay"] for e in r["events"] if e["event"] == "docking"] == [2]
    assert any("wrong call: bay 1 reads RED" in x for x in r["logs"])
    r = FastSil(POOL, "T3", {"bay": 1, "reengage_max": 0}, start=NEAR_DOCK).run(300)   # no re-engagement left: hold, say why
    assert r["state"] == "SAFE_HOLD" and r["sub_state"] in ("TURN_IN", "ENTER") and "0 re-engagement(s) used" in r["safe_holds"][0]
    assert not any(e["event"] == "docking" for e in r["events"])


def test_the_choice_follows_the_green_bay_moved_mid_search():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    s.run(300, until=lambda x: x.task.m.state == "SWAY_SEARCH")
    s.dock.command({"cmd": "green_bay", "bay": 3}, s.t)
    r = s.run(500)
    assert r["state"] == "DONE" and [e["bay"] for e in r["events"] if e["event"] == "docking"] == [3], r["logs"][-6:]
    assert r["search"][-1]["chosen"] == 3


def test_no_green_bay_two_passes_then_hold():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    s.dock.green_bay = 99                                                          # every indicator RED
    r = s.run(500)
    assert r["state"] == "SAFE_HOLD" and r["sub_state"] == "SWAY_SEARCH", (r["state"], r["logs"][-4:])
    assert [p["standoff_m"] for p in r["search"]] == [SD, SD2] and all(p["chosen"] is None for p in r["search"])
    assert "no single GREEN bay after two passes" in r["safe_holds"][0] and not any(e["event"] == "docking" for e in r["events"])


def test_no_window_lit_fires_at_the_bay_centre_then_reports_incomplete():
    s = FastSil(POOL, "T3", {"fire_s": 16.0}, start=NEAR_DOCK)
    s.dock = None                                                                  # nobody lights a window
    r = s.run(500)
    assert r["state"] == "DONE", r["logs"][-4:]
    ff = [e for e in r["events"] if e["event"] == "firefighting"][0]
    assert not ff["complete"] and ff["window_id"] is None and ff["result"] == "not seen green" and 15.5 <= ff["fired_s"] <= 17.0
    assert any("at the bay centre" in x for x in r["logs"])
    on = [a for a in s.aims if a.get("on")]
    assert on and all(a["window_id"] is None for a in on) and any(not a.get("on") for a in s.aims)   # off for the first 10 s


def test_the_lit_window_unseen_is_aimed_at_by_elimination():
    """The model misses a window at the 1.5 m stop from some poses (CV_TRAIN: 14 % at 1.2-2.5 m; campaign seed 5): one window seen dark
    -> after window_wait_s the aim goes to the other one, and the report carries its window_id (complete false: GREEN never seen)."""
    s = FastSil(POOL, "T3", {"fire_s": 20.0}, start=NEAR_DOCK)
    s.run(400, until=lambda x: x.task.m.state == "AIM_FIRE")
    lit = s.dock.window
    s.fp.objects = [o for o in s.fp.objects if not (o.get("kind") == "window" and o.get("bay") == 2 and o.get("window") == lit)]
    for e in list(s.map.ents.values()):
        if e.kind == "window" and getattr(e, "window_no", None) == lit:
            del s.map.ents[e.id]
    r = s.run(300)
    assert r["state"] == "DONE" and any("seen dark - at window %d" % lit in x for x in r["logs"]), r["logs"][-4:]
    ff = [e for e in r["events"] if e["event"] == "firefighting"][0]
    assert ff["window_id"] == lit and not ff["complete"] and r["dock"]["phase"] in ("green", "done")   # the hit happened, unseen
    assert {a["window_id"] for a in s.aims if a.get("on")} == {lit}


def test_the_step_walk_parks_at_every_stage_and_the_entry_points():
    r = FastSil(POOL, "T3", {"step": True}, start=NEAR_DOCK).run(600)
    assert r["state"] == "DONE", (r["state"], r["logs"][-4:])
    assert r["steps"] == [f"step: next {s}" for s in T3_BOUNDARIES]
    assert _reports(r) == [("docking", "REPORT_DOCK"), ("firefighting", "REPORT_FIRE")]
    r = FastSil(POOL, "T3", {"step": True, "bay": 1}, start=NEAR_DOCK).run(600)    # the BACK_OUT detour is a stage too
    assert "step: next BACK_OUT" in r["steps"] and r["state"] == "DONE"
    from optiprimex.usv.tasks.t3_logistics import T3Logistics
    assert "SWAY_SEARCH" in T3Logistics.ENTRY_POINTS and "BACK_OUT" not in T3Logistics.ENTRY_POINTS
    s = FastSil(POOL, "T3", {"from": "SWAY_SEARCH"}, start={"x_n": 36.0, "y_e": 5.5, "hdg_deg": 0.0}, warmup_s=4.0)   # 9 m off: three bays
    assert s.from_refused == ""
    r = s.run(400)
    assert r["state"] == "DONE" and r["states"][0] == "SWAY_SEARCH" and r["search"][0]["chosen"] == 2


@pytest.mark.skipif(not os.path.isdir(MODEL), reason="no CV_TRAIN model (../CV_TRAIN/models/cvt-v2-l)")
def test_m3t1_step_walk_with_the_model_cvt_v2_l():
    """The --step walk on the model's detections (CPU: ~3 min): a park before every stage, the strict grader."""
    pytest.importorskip("ultralytics")
    s = FastSil(POOL, "T3", {"step": True}, start={"x_n": 30.0, "y_e": 4.0, "hdg_deg": 0.0}, perception="camera", detector="model", model=MODEL,
                cam_size=(736, 414), seed=4)
    assert s.cam.dock and "dock" not in {o["kind"] for o in s.fp.objects}
    r = s.run(500)
    assert r["steps"] == [f"step: next {x}" for x in T3_BOUNDARIES], r["steps"]
    g = C.grade_m3t1(W.load(POOL)["objects"], r, green_bay=2, lit_window=s.dock.window, strict=True)
    assert g["ok"], (g["why"], r["logs"][-6:])


# ---- the campaign, the sheet, the launcher -------------------------------------------------------------------------------------
def test_the_campaign_m3t1_cases_starts_and_the_strict_grader():
    cs = C.cases(["m3t1"], 2, ["pool"], ["-"], ["camera"], detectors=("synthetic", "model"), model=str(ROOT / "nope"))
    assert [(c["detector"], c["start"], c["seed"]) for c in cs] == [("synthetic", "random", 1), ("synthetic", "random", 2)]   # no model: skipped
    cs = C.cases(["m3t1"], 1, ["pool"], ["-"], ["camera", "geometric"], detectors=("synthetic", "model"), model=str(ROOT))
    assert [(c["perception"], c.get("detector")) for c in cs] == [("camera", "synthetic"), ("camera", "model"), ("geometric", None)]
    wd = W.load(POOL)
    greens = set()
    for seed in range(1, 25):
        st = C.m3t1_start(wd, seed)
        d = 45.0 - st["x_n"]
        assert 8.0 <= d <= 20.0 and abs(math.degrees(math.atan2(st["y_e"] - 5.5, d))) <= 40.01 and -11.0 <= st["y_e"] <= 11.0
        assert all(math.hypot(st["x_n"] - o["x_n"], st["y_e"] - o["y_e"]) >= 2.5 for o in wd["objects"] if o["kind"] == "buoy")
        greens.add(st["green_bay"])
    assert greens == {1, 2, 3} and C.m3t1_start(wd, 3) == C.m3t1_start(wd, 3)
    objs = wd["objects"]
    good = {"state": "DONE", "sub_state": "DONE", "reason": "", "safe_holds": [], "reengaged": 0,
            "events": [{"event": "docking", "bay": 2}, {"event": "firefighting", "window_id": 1, "complete": True}],
            "event_log": [("docking", "REPORT_DOCK", 50.0), ("firefighting", "REPORT_FIRE", 60.0)],
            "states": ["FIND_DOCK", "TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "DOCKED", "REPORT_DOCK", "AIM_FIRE"],
            "search": [{"pass": 1, "votes": {1: {"red": 5, "green": 0}, 2: {"red": 0, "green": 5}, 3: {"red": 4, "green": 0}}, "chosen": 2}],
            "track": [(40.0, 43.5, 5.5, 0, "DOCKED")]}
    assert C.grade_m3t1(objs, good, 2, 1, strict=True)["ok"]
    bad = dict(good, event_log=[("docking", "ENTER", 40.0), ("firefighting", "REPORT_FIRE", 60.0)])
    assert "reports" in C.grade_m3t1(objs, bad, 2, 1, strict=True)["why"]
    assert "window_id 1 (lit = 2)" in C.grade_m3t1(objs, good, 2, 2, strict=True)["why"]
    few = dict(good, search=[{"pass": 1, "votes": {1: {"red": 1, "green": 0}, 2: {"red": 0, "green": 5}, 3: {"red": 4, "green": 0}}, "chosen": 2}])
    assert "not every bay seen" in C.grade_m3t1(objs, few, 2, 1, strict=True)["why"]
    assert "but docked in bay 3" in C.grade_m3t1(objs, dict(good, track=[(40.0, 43.5, 9.0, 0, "DOCKED")]), 2, 1, strict=True)["why"]
    assert C.grade_m3t1(objs, good)["ok"]                                          # the old call: docked in the world's GREEN + fire


def test_the_campaign_doc_section(tmp_path):
    doc = tmp_path / "c.md"
    doc.write_text("# x\n<!-- CAMPAIGN BEGIN -->\nmain\n<!-- CAMPAIGN END -->\n")
    C.write_doc(str(doc), "| m3t1 |", "12/12", section="m3t1")
    txt = doc.read_text()
    assert "main" in txt and "<!-- CAMPAIGN M3T1 BEGIN -->\n12/12\n\n| m3t1 |\n<!-- CAMPAIGN M3T1 END -->" in txt
    assert "CAMPAIGN M3T1 BEGIN" in (ROOT / "docs" / "CAMPAIGN.md").read_text()


def test_the_sheet_s9_and_run_sim_with_the_model():
    from optiprimex.tools import sim_day
    assert "SWAY_SEARCH" in sim_day.Walk.T3_STAGES and "REPORT_FIRE" in sim_day.Walk.T3_STAGES
    src = (ROOT / "optiprimex" / "tools" / "sim_day.py").read_text()
    assert '"green_bay", "bay": 3' in src and '"--bay", "1"' in src and "--boat-port" in src
    sheet = (ROOT / "docs" / "SIM_DAY.md").read_text()
    assert "SWAY_SEARCH" in sheet and "BACK_OUT" in sheet and "window_id" in sheet
    sh = (ROOT / "scripts" / "run_sim.sh").read_text()
    assert 'FP="--skip-kinds buoy,dock,window"' in sh
    SO = _load("so_t148", "cv/sim_observer/sim_observer.py")
    assert SO.parse_args([]).dock_on is False and SO.parse_args(["--detector", "model", "--model", "x"]).dock_on is True
    assert SO.parse_args(["--dock", "on"]).dock_on is True
    assert "ultralytics==" in (ROOT / "requirements-sim.txt").read_text()
    from optiprimex.tools import playbook
    names = [s.name for s in playbook.MISSIONS["m3t1"].stages]
    assert names.index("TO_LINE") < names.index("SWAY_SEARCH") < names.index("SWAY_TO_ABEAM") and "BACK_OUT" in names


# ---- live: the mission manager, the OCS and the fake judge (the rc-test proof is the parent's, see the hand-back) -------------
NS = "team/optiprime/v1"


@pytest.fixture(scope="module")
def t3stack(brokers, fake_fcu, tmp_path_factory):
    import time
    import paho.mqtt.client as mqtt
    from optiprimex.sim.marina import CENTRE
    from .conftest import PY, Probe, Proc, make_ocs_config, make_usv_config
    tmp = tmp_path_factory.mktemp("t3b")
    lat, lon = W.WorldFrame({"lat": CENTRE[0], "lon": CENTRE[1]}).to_latlon(NEAR_DOCK["x_n"], NEAR_DOCK["y_e"])
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(make_usv_config(tmp, brokers, fake_fcu))], tmp / "mm.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#", "rc/#"]).start()
    boat = Probe(brokers.boat_port, ["sim/#", "local/usv/cannon/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15, include_past=True), mm.log()
    fake_fcu.ctrl({"teleport": {"lat": lat, "lon": lon, "hdg": 0.0}})
    fp = Proc([PY, "-m", "optiprimex.sim.fake_perception", "--world", POOL, "--port", str(brokers.boat_port), "--hub-port", str(brokers.hub_port),
               "--seed", "3"], tmp / "fp.log")
    fd = Proc([PY, "-m", "optiprimex.sim.fake_dock", "--world", POOL, "--port", str(brokers.boat_port)], tmp / "fd.log")     # auto: the world's (off)
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--start-delay", "1.0", "--log", str(tmp / "rc.jsonl")], tmp / "rc.log")
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(make_ocs_config(tmp, brokers))], tmp / "ocs.log")
    from optiprimex.tools.ocs_cli import OcsConsole
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 15, include_past=True), ocs.log()
    beat = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t3b-ui-beat", protocol=mqtt.MQTTv311)
    beat.connect("127.0.0.1", brokers.boat_port)
    beat.loop_start()
    beat.publish("presence/usv_ui", json.dumps({"node": "usv_ui", "up": True, "pid": 1, "fcu_link": "fake", "t_unix": time.time()}),
                 qos=0, retain=True).wait_for_publish(2)
    yield {"mm": mm, "probe": probe, "boat": boat, "con": con, "fd": fd, "rc_log": tmp / "rc.jsonl"}
    beat.publish("presence/usv_ui", None, qos=0, retain=True).wait_for_publish(2)
    beat.loop_stop(); beat.disconnect()
    con.stop(); probe.stop(); boat.stop()
    for p in (ocs, rc, fd, fp, mm):
        p.kill()


def test_m3t1sim_live_wrong_call_back_out_reports_in_order(t3stack):
    """--bay 1 on the GREEN-2 pool: BACK_OUT -> SWAY_SEARCH -> bay 2; the window lights on the DockingReport alone (fake_dock auto off);
    the aim carries the lit window's id; DockingReport then FirefightingReport (window_id) reach the judge once each, in order;
    the heartbeat's current_task is T3 from START_TASK to DONE; a task_state event for every stage incl. BACK_OUT."""
    from .conftest import wait_until
    con, p, boat = t3stack["con"], t3stack["probe"], t3stack["boat"]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("fcu_link") == "fake", 10)
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10)
    assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15)
    n0 = len(p.all())
    r = con.intent("START_TASK", "T3", args={"tier": "core", "sim": True, "bay": 1}, wait_s=5)
    assert [a["body"]["result"] for a in r["acks"]][-1] == "DONE", [a["body"] for a in r["acks"]]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                  and m["env"]["body"]["current_task"] == "TASK_NONE", 360), t3stack["mm"].log()[-3000:]
    ms = p.all()[n0:]
    stages = [m["env"]["body"]["to"] for m in ms if m["topic"].endswith("event/usv/task_state") and m["env"]["body"].get("machine") == "T3"]
    assert stages[:6] == ["FIND_DOCK", "TO_LINE", "SWAY_TO_ABEAM", "TURN_IN", "BACK_OUT", "SWAY_SEARCH"] or \
        stages[:7] == ["FIND_DOCK", "TO_LINE", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "BACK_OUT", "SWAY_SEARCH"], stages
    assert stages[-6:] == ["DOCKED", "REPORT_DOCK", "AIM_FIRE", "REPORT_FIRE", "UNDOCK", "DONE"], stages
    reps = [(m["topic"].rsplit("/", 1)[-1], m["env"]["body"]) for m in ms if m["topic"].endswith(("event/usv/docking", "event/usv/firefighting"))]
    assert [k for k, _ in reps] == ["docking", "firefighting"] and reps[0][1]["bay"] == 2, reps
    lit = boat.last("sim/dock/state")["env"]["body"]["window"]["window"]
    assert reps[1][1]["window_id"] == lit and reps[1][1]["complete"] is True
    assert boat.wait(lambda m: m["topic"] == "local/usv/cannon/aim" and m["env"]["body"].get("on") is True
                     and m["env"]["body"].get("window_id") == lit, 1, include_past=True)
    i_start = next(i for i, m in enumerate(ms) if m["topic"].endswith("event/usv/task_state") and m["env"]["body"].get("to") == "FIND_DOCK")
    i_done = next(i for i, m in enumerate(ms) if m["topic"].endswith("event/usv/task_state") and m["env"]["body"].get("to") == "DONE")
    tasks = {m["env"]["body"]["current_task"] for m in ms[i_start:i_done] if m["topic"].endswith("status/usv")}
    assert tasks == {"TASK_COORDINATED_LOGISTICS"}, tasks

    def stats():
        m = p.last("rc/sim/stats")
        return json.loads(m["payload"]) if m else {}
    assert wait_until(lambda: [x["kind"] for x in stats().get("reports", []) if x["kind"] in ("docking", "firefighting")] == ["docking", "firefighting"], 10), \
        stats().get("reports")


def test_the_bench_lights_the_window_itself_the_simulator_waits_for_the_report():
    """v0.1.48 (merge): usv_ui `dockto` sends no DockingReport, so the bench launchers keep fake_dock's own lighting (--auto on);
    run_sim.sh passes DOCK_AUTO=world (the world file's dock.auto: false = lit only on our DockingReport)."""
    from .conftest import ROOT
    s = ROOT / "scripts"
    assert "--auto ${DOCK_AUTO:-on}" in (s / "run_bench.sh").read_text()
    assert (s / "run_sim.sh").read_text().count("DOCK_AUTO=world MAP=0") == 2
    assert "--auto on" in (s / "sim_world.sh").read_text() and "--auto on" in (s / "run_agx.sh").read_text()
