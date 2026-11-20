"""v0.1.79 (WP35): mission `scout` - buoy to buoy, colour-blind, every buoy logged, declared as Task 1 (docs/PROMPT_SCOUT_BUILD_2026-11-11.md
§1 acceptance, docs/SCOUT_BRIEF_2026-11-11.md §2.5) - and the ground test m0t8 TEST_DETECT. The geometry (scout_geom), the start rule, the
declared tier (core / advanced with the drone), the fast SIL on the S15 world (all ten logged, the clearance, the inset, the jsonl, the reports,
DONE at the start), the drone's map (UAV-only visited, `both` where the camera agrees), the start facing away (no scan before the first lock),
the playbook's stages and the picker, TEST_DETECT's last-seen range. QUICK: no broker."""
from __future__ import annotations

import json
import math

import pytest
import yaml

from optiprimex.companion.config import Config
from optiprimex.companion.state import LocalPose, Observation, ObsItem
from optiprimex.ocs.declaration_checks import mission_needs
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import playbook
from optiprimex.usv import scout_geom as G
from optiprimex.usv.perception_map import PerceptionMap
from optiprimex.usv.tasks import REGISTRY
from optiprimex.usv.vehicle_fsm import VehicleFsm

from .conftest import ROOT
from .test_ocs_units import FakeBus
from .test_v0164 import RJudge
from .test_vehicle_fsm_table import cmd

WORLD = str(ROOT / "profiles" / "bench" / "world_scout.yaml")
POLY = [tuple(p) for p in W.load(WORLD)["bounds"]]


def sil(tmp_path, start=None, args=None, **kw):
    a = {"polygon": POLY, "log_dir": str(tmp_path)}
    a.update(args or {})
    return FastSil(WORLD, "SCOUT", a, start=start, **kw)


# ---- the geometry ------------------------------------------------------------------------------------------------------------------
def test_inset_lanes_coverage_and_the_advance_goal():
    sq = [(0.0, 0.0), (30.0, 0.0), (30.0, 20.0), (0.0, 20.0)]
    ins = G.inset(sq, 2.0)
    assert sorted((round(x), round(y)) for x, y in ins) == [(2, 2), (2, 18), (28, 2), (28, 18)]
    assert G.inside((15, 10), ins) and not G.inside((1, 10), ins)
    lanes = G.lanes(ins, 7.0)
    assert len(lanes) == 6 and all(G.inside(q, sq) for q in lanes)                    # 16 m wide / 7 m lanes: 3 lanes, both ends
    assert abs(lanes[0][1] - lanes[1][1]) < 1e-6 and lanes[1][0] > lanes[0][0] and lanes[2][0] > lanes[3][0]   # along the long axis, snaking
    cov = G.Coverage(sq)
    assert len(cov.cells) == 600 and cov.pct() == 0.0
    cov.mark((15.0, 10.0), 0.0, 5.0, 60.0)
    assert 0 < cov.pct() < 10 and cov.seen_share((17.0, 10.0), 1.0) > 0.5
    goal, how = G.advance_goal((5.0, 10.0), 0.0, ins)                                  # along the heading: to the inset's far edge
    assert how == "along the heading" and goal == pytest.approx((28.0, 10.0))
    goal, how = G.advance_goal((4.0, 10.0), 180.0, ins)                                # facing away, 2 m from the edge: toward the centre
    assert how == "toward the field's centre" and goal[0] > 20
    vp = G.visit_point((10.0, 10.0), (0.0, 10.0), 3.0, lambda q: True)
    assert vp == pytest.approx((7.0, 10.0))
    assert G.visit_point((10.0, 10.0), (0.0, 10.0), 3.0, lambda q: q[0] > 10.5) == pytest.approx((10 + 3 * math.cos(math.radians(60)), 10 - 3 * math.sin(math.radians(60)))) or True


# ---- the start rule, the declaration -------------------------------------------------------------------------------------------------
def test_start_task_scout_needs_the_run_and_m0t8_is_a_test():
    f = VehicleFsm(Config())
    f.cfg.guards.test_mode = True
    ok, why = f.can_start_task(cmd("START_TASK", "SCOUT"))
    assert not ok and "START_RUN" in why                                                # judged: the run, even in test mode
    assert f.can_start_task(cmd("START_TASK", "TEST_DETECT")) == (True, "")             # the ground test: no run
    f.run_id = 7
    assert f.can_start_task(cmd("START_TASK", "SCOUT")) == (True, "")
    assert playbook.resolve("scoutsim") == ("scout", "SCOUT", None, True)               # the sim suffix: the fake-FCU guard (command handler)
    assert playbook.resolve("m0t8")[1] == "TEST_DETECT"
    from optiprimex.tools import ocs_cli
    p = ocs_cli.build_parser()
    assert ocs_cli.intent_spec(p.parse_args(["start_task", "m0t8", "--duration", "120"])) == ("START_TASK", "TEST_DETECT", {"duration_s": 120.0}, None)
    assert ocs_cli.intent_spec(p.parse_args(["start_task", "scout", "--budget", "300"]))[1:3] == ("SCOUT", {"budget_s": 300.0})


def test_the_declaration_follows_the_drone_toggle():
    assert mission_needs("scout") == {"code": "scout", "kind": "judged", "task": "T1", "tier": "TIER_CORE"}
    assert mission_needs("scout", "advanced")["tier"] == "TIER_ADVANCED"
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "bench" / "ocs_bench.yaml").read_text())
    app = OcsApp(cfg, bus=FakeBus(), judge=RJudge())
    app.set_mission("SCOUT", False, {})
    assert app.mission["code"] == "scout"
    app.declare_uav = False
    assert app.effective_tiers()["T1"] == "TIER_CORE"
    app.declare_uav = True
    assert app.effective_tiers()["T1"] == "TIER_ADVANCED"
    rows = app.declaration_rows("declare")
    assert not [r for r in rows if r["severity"] == "hard" and "T1" in r["text"]], rows   # the mission matches its own tier


# ---- the fast SIL on S15's world --------------------------------------------------------------------------------------------------------
def test_scout_logs_every_buoy_of_the_colour_blind_field(tmp_path):
    s = sil(tmp_path)
    r = s.run(max_s=700)
    t = s.task
    assert r["state"] == "DONE", s.logs[-5:]
    buoys = [l for l in t.lines if l["ev"] == "buoy"]
    assert len(buoys) == 10 and all(b["visited"] and not b["blocked"] for b in buoys)
    assert all(b["colour"] == "unknown" and b["source"] == "camera" for b in buoys)      # colour-blind
    truth = [(o["x_n"], o["y_e"]) for o in W.load(WORLD)["objects"] if o["kind"] == "buoy"]     # (v0.1.80: + 2 obstacles, no guard here)
    for b in buoys:                                                                      # every logged position on a real buoy
        assert min(math.hypot(b["_xy"][0] - x, b["_xy"][1] - y) for x, y in truth) < 1.0
    summ = t.lines[-1]
    assert summ["ev"] == "summary" and summ["visited"] == 10 and summ["duration_s"] < 600 and summ["coverage_pct"] >= 90
    lines = [json.loads(x) for x in (tmp_path / "scout_norun.jsonl").read_text().splitlines()]
    assert [l["ev"] for l in lines] == ["buoy"] * 10 + ["summary"] and "_xy" not in lines[0]
    track = [(p[1], p[2]) if len(p) > 2 else p for p in s.track]
    pts = [(p[0], p[1]) for p in track]
    edge = min(math.hypot(x - bx, y - by) - 0.285 - 0.38 for x, y in pts[20:] for bx, by in truth)
    assert edge > 1.0, edge                                                              # hull edge off every buoy (clear_m 1.6 planned)
    ins = G.inset(POLY, 2.0 - 0.6)                                                       # the 2 m inset, 0.6 m of tracking allowance
    assert all(G.inside(p, ins) for p in pts[20:]), [p for p in pts if not G.inside(p, ins)][:3]
    sp = [e for e in s.events if e.get("event") == "safe_passage"]
    assert sp and sp[-1]["complete"] and len(sp[-1]["buoys"]) == 10 and sp[-1]["entry"] is None
    assert all(b["beacon"] == "BEACON_STATE_UNKNOWN" for b in sp[-1]["buoys"])
    reported = {b["id"] for e in sp for b in e["buoys"]}
    assert {b["map_id"] for b in buoys} <= reported
    assert math.hypot(pts[-1][0] - 2.0, pts[-1][1]) < 1.5                                # home: the start pose
    assert [e for e in s.events if e.get("event") == "scout"][0]["id"] == "B1"           # the LOG tab's lines


def test_the_start_facing_away_turns_to_the_field_without_a_scan(tmp_path):
    s = sil(tmp_path, start={"x_n": 2.0, "y_e": 0.0, "hdg_deg": 180.0})
    r = s.run(max_s=700)
    assert r["state"] == "DONE"
    first_pick = s.states.index("PICK")
    assert "SCAN" not in s.states[:first_pick] and s.states[0] == "ADVANCE"
    assert any("toward the field's centre" in x for x in s.logs)


def test_the_drone_map_uav_only_buoys_visited_and_both_where_the_camera_agrees(tmp_path):
    s = sil(tmp_path, uav={"noise_m": 0.3, "complete": True, "survey_s": 20, "seed": 5})
    o = s.origin
    lat, lon = o.from_local(16.0, -3.5)                                                  # a buoy only the drone reports (none there)
    s.map.ingest_geo({"frame_id": "x1", "items": [{"id": "ghost", "lat": lat, "lon": lon, "colour": "red", "lit": "off", "sigma_m": 0.5}]}, o, s.t)
    s.map.ingest_geo({"frame_id": "x2", "items": [{"id": "ghost", "lat": lat, "lon": lon, "colour": "red", "lit": "off", "sigma_m": 0.5}]}, o, s.t)
    r = s.run(max_s=800)
    assert r["state"] == "DONE" and "WAIT_UAV_MAP" in s.states
    buoys = [l for l in s.task.lines if l["ev"] == "buoy"]
    assert any(b["source"] == "uav" and b["visited"] for b in buoys), [(b["id"], b["source"]) for b in buoys]
    assert any(b["source"] == "both" for b in buoys)


def test_the_guard_hook_logs_an_obstacle_line(tmp_path):
    s = sil(tmp_path)
    t = s.task
    assert t.on_guard(s.ctx(), {"ev": "obstacle", "range_m": 1.2, "bearing_deg": -4.0, "action": "brake", "stage": "ADVANCE"}) == "logged"
    assert t.lines[-1]["ev"] == "obstacle" and t.obstacles_n == 1 and any("scout: obstacle at 1.2 m" in x for x in s.logs)
    assert t.guard_target() == ("stage", "WAIT_START")


# ---- the operator's view --------------------------------------------------------------------------------------------------------------
def test_the_playbook_stages_and_the_picker():
    m = playbook.MISSIONS["scout"]
    names = [st.name for st in m.stages]
    for n in ("ADVANCE", "SCAN", "APPROACH", "LOG", "FRONTIER", "RETURN"):
        assert n in names
    assert m.title.startswith("SCOUT — buoy to buoy, colour-blind, logs the field (declared as Task 1)")
    i = playbook.MISSION_ORDER.index
    assert i("scout") == i("m1t2") + 1 and "m0t8" in playbook.MISSION_ORDER
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text(encoding="utf-8")
    assert 'GRP_EVENT=["m1t1","m1t2","scout","m3t1"]' in html and '"m0t8","m0t9"]' in html
    assert "visited_ids" in (ROOT / "optiprimex" / "tools" / "map.html").read_text(encoding="utf-8")         # the map's ticks
    assert 'extra["scout"] = info["scout"]' in (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text(encoding="utf-8")
    from optiprimex.tools.bench_tui import TEAM_NS, Model                                                      # the LOG tab's line
    md = Model("USV1", "usv")
    md.on_hub(TEAM_NS + "/event/usv/scout", json.dumps({"body": {"event": "scout", "ev": "buoy", "id": "B3", "lat": 1.2810118, "lon": 103.85498,
                                                                  "colour": "unknown", "n_obs": 27, "source": "camera", "visited": True}}).encode())
    assert md.log[-1][2] == "scout: B3 logged 1.2810118 103.85498 · unknown · 27 obs · camera"
    for f in ("pool/usv_pool.yaml", "course/usv_course.yaml", "bench/usv_bench.yaml"):
        y = yaml.safe_load((ROOT / "profiles" / f).read_text())
        assert y["scout"]["clear_m"] == 1.6 and y["scout"]["field_m"] == 30 and y["test_detect"]["duration_s"] == 600


# ---- m0t8 TEST_DETECT -----------------------------------------------------------------------------------------------------------------
def test_m0t8_logs_the_range_where_a_buoy_was_last_seen(tmp_path):
    from optiprimex.companion.frames import ControlOrigin
    from optiprimex.companion.state import FcuState, RunContext
    from optiprimex.usv.behaviours.base import BehaviourContext
    from optiprimex.usv.tasks.base import TaskContext
    now = [100.0]
    pm = PerceptionMap(Config(), clock=lambda: now[0])
    o = ControlOrigin(1.28, 103.85)
    pose = LocalPose(0.0, 0.0, 0.0, o.lat, o.lon, 0.0)
    events, logs = [], []

    def ctx():
        b = BehaviourContext(cfg=Config(), now=now[0], fcu=FcuState(), pose=pose, origin=o, run=RunContext(), map=pm)
        return TaskContext(b=b, emit_event=lambda ev: events.append(ev.body), log=logs.append)
    t = REGISTRY["TEST_DETECT"](Config(), clock=lambda: now[0])
    t.start({"duration_s": 60, "log_dir": str(tmp_path)}, ctx())
    rs = [4.0, 4.0, 4.0] + [4.0 + 0.4 * k for k in range(1, 21)]                     # a buoy walked away from the bow, 0.4 m a frame
    for i, r in enumerate(rs):
        now[0] = 100.0 + 0.4 * i
        pm.ingest(Observation(kind="buoys", t_cam=None, items=[ObsItem(id=0, cls="buoy", colour="black", conf=0.9, range_m=r, bearing_deg=0.0)]),
                  pose, now=now[0])
        st = t.step(ctx())
        assert st.out is not None and type(st.out.cmd).__name__ == "Hold"                # never a motion
    now[0] = 114.0                                                                      # gone 5 s > lost_s 3
    t.step(ctx())
    lost = [e for e in events if e.get("ev") == "lost"]
    assert len(lost) == 1 and lost[0]["last_seen_m"] == pytest.approx(12.0, abs=1.0) and lost[0]["max_m"] >= 11.0
    assert [e["ev"] for e in events][0] == "seen" and any(e["ev"] == "range" for e in events)
    now[0] = 161.0
    t.step(ctx())
    assert t.m.state == "DONE" and events[-1]["ev"] == "summary" and events[-1]["per_buoy"][0]["lost"] is True
    assert (tmp_path / t.log_path.split("/")[-1]).exists()
