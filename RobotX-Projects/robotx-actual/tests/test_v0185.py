"""v0.1.85 (WP41 + WP46): turn under way on the water day's hull, the circle scan, move-first, the yaw couple test, our course only.

docs/PROMPT_COMPUTE_NIGHT_2026-11-11.md P0 (T0.2-T0.5) + its addendum (T0.2a-e), docs/PROMPT_PRESETS_NIGHT_2026-11-12.md §6 items 7, 9, 10.
QUICK: no broker (the fast SIL in process)."""
from __future__ import annotations

import math

import pytest
import yaml

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin
from optiprimex.companion.state import (Course, FcuState, GuidedTarget, LocalPose, Observation, ObsItem, RunContext,
                                        ThrusterFrame, VelocityYawTarget)
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil
from optiprimex.sim.plant import Plant
from optiprimex.usv import guidance as GD
from optiprimex.usv import scout_geom as G
from optiprimex.usv.behaviours import ScanOrbit
from optiprimex.usv.behaviours.base import BehaviourContext
from optiprimex.usv.behaviours.mode20 import frame as m20_frame
from optiprimex.usv.perception_map import PerceptionMap
from optiprimex.usv.tasks.base import TaskContext
from optiprimex.usv.tasks.test_yaw import TestYaw, mean_rate, yaw_frame

from .conftest import ROOT

SCOUT_W = str(ROOT / "profiles" / "bench" / "world_scout.yaml")
POOL_W = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
POLY = [tuple(p) for p in W.load(SCOUT_W)["bounds"]]
PROF = yaml.safe_load((ROOT / "profiles" / "course" / "usv_course.yaml").read_text())
YAW = float(PROF["guidance"]["yaw_rate_max_dps"])                 # v0.1.93.2: the course 6.5 (Dmitrii: slightly more yaw); the pool 5
STARTS = [None, {"x_n": 2.0, "y_e": 0.0, "hdg_deg": 180.0}, {"x_n": 3.0, "y_e": -10.0, "hdg_deg": 90.0}, {"x_n": 2.0, "y_e": 8.0, "hdg_deg": 30.0}]


def uw_cfg(**guid) -> Config:
    cfg = Config()
    cfg.extra = {"guidance": dict(PROF["guidance"], **guid), "scout": dict(PROF["scout"]), "plant": {"hull": "omni_usv"}}
    return cfg


def traced(s: FastSil):
    """Record (t, cmd, x, y) of every task output."""
    cmds = []
    orig = s.task.step

    def step(ctx):
        r = orig(ctx)
        if r.out is not None and r.out.cmd is not None:
            cmds.append((s.t - 1000.0, r.out.cmd, ctx.b.pose.x_n, ctx.b.pose.y_e))
        return r
    s.task.step = step
    return cmds


def pose(x=0.0, y=0.0, h=0.0, t=0.0):
    return LocalPose(x, y, h, 1.28, 103.85, t, sog_mps=0.0)


# ---- the profiles -----------------------------------------------------------------------------------------------------------------------
def test_the_profiles_carry_the_night_numbers():
    for name in ("pool/usv_pool.yaml", "course/usv_course.yaml"):
        p = yaml.safe_load((ROOT / "profiles" / name).read_text())
        g = p["guidance"]
        assert g["mode"] == "underway" and g["v_min_mps"] == 0.4 and g["yaw_rate_max_dps"] == (8.0 if "course" in name else 5) and g["scan_v_mps"] == 0.3
        assert g["start_fwd_m"] == 1.0 and g["course_tol_m"] == 1.5 and g["bow_clear_m"] == 3.5 and g["los_m"] == 3
        assert p["scout"]["budget_s"] == 900 and p["scout"]["margin_m"] == 4.0
    bench = yaml.safe_load((ROOT / "profiles" / "bench" / "usv_bench.yaml").read_text())
    assert "guidance" not in bench and not GD.underway(Config())          # the bench / the ideal campaigns: unchanged (fcu)


# ---- T0.4 the plant ---------------------------------------------------------------------------------------------------------------------
def test_the_omni_hull_turns_2_in_place_and_4_under_way_the_ideal_one_unchanged():
    def rate(hull, u, ramp=0.0, secs=20.0):
        p = Plant(1.28, 103.85, 0.0, hull=hull, str_ramp_s=ramp)
        for _ in range(int(secs / 0.1)):
            p.step(0.1, u, 0.0, Plant.R_MAX)                     # full differential
        return math.degrees(p.r)
    assert rate("omni_usv", 0.0) <= 2.0 + 1e-6
    assert 3.8 <= rate("omni_usv", 0.5) <= 4.0 + 1e-6
    assert rate("ideal", 0.0) == pytest.approx(45.0, abs=0.1)
    assert rate("omni_usv", 0.0, ramp=25.0, secs=10.0) < 2.0     # the 11 Nov ramp: still short of the authority after 10 s
    with pytest.raises(ValueError):
        Plant(1.28, 103.85, 0.0, hull="banana")


# ---- T0.2 the law -----------------------------------------------------------------------------------------------------------------------
def test_the_law_drives_while_turning_caps_the_yaw_and_arcs_beyond_the_cone():
    st = GD.Steer(uw_cfg())
    c = st.law(0.0, (0.0, 0.0), 0.0, (10.0, 0.0), 0.6)
    assert c.vx_mps == pytest.approx(0.6) and c.yaw_rate_rad_s == pytest.approx(0.0)
    c = st.law(0.0, (0.0, 0.0), 0.0, (10.0, 10.0), 0.6)            # 45 deg off: still forward (>= v_min), yaw at the cap
    assert c.vx_mps >= 0.4 and math.degrees(c.yaw_rate_rad_s) == pytest.approx(YAW)
    c = st.law(0.0, (0.0, 0.0), 0.0, (-30.0, 1.0), 0.6)            # behind: a slow arc, not a pirouette
    assert c.vx_mps == pytest.approx(0.3) and abs(math.degrees(c.yaw_rate_rad_s)) == pytest.approx(YAW)


def test_bow_clear_the_boundary_and_the_heading_time_out():
    st = GD.Steer(uw_cfg(), log=(logs := []).append)
    c = st.law(0.0, (0.0, 0.0), 0.0, (0.0, 20.0), 0.6, obstacles=[(2.5, 0.0, 0.25)])     # an object 2.5 m dead ahead, the goal 90 deg off
    assert c.vx_mps == 0.0 and "bow-clear" in st.last["why"]
    c = st.law(0.0, (0.0, 0.0), 0.0, (0.0, 20.0), 0.6, obstacles=[(2.5, 4.0, 0.25)])     # out of the bow cone: drives
    assert c.vx_mps > 0.0
    sq = [(-10.0, -20.0), (5.0, -20.0), (5.0, 20.0), (-10.0, 20.0)]                       # the boundary 5 m ahead
    c = st.law(0.0, (0.0, 0.0), 0.0, (3.0, 1.0), 0.6, poly=sq)
    assert c.vx_mps <= 0.15 + 1e-9
    c = st.law(0.0, (3.5, 0.0), 0.0, (0.0, 15.0), 0.6, poly=sq, leg="L")                 # 1.5 m from it, the goal far off the bow: turn in place
    assert c.vx_mps == 0.0
    for t in range(1, 17):                                                                # the heading never comes round: 15 s -> drive anyway
        c = st.law(float(t), (3.5, 0.0), 0.0, (0.0, 15.0), 0.6, poly=sq, leg="L")
    assert c.vx_mps == pytest.approx(0.4) and any("heading time-out" in x for x in logs)


def test_closest_approach_arrival():
    a = GD.Arrival(1.0, 1.5)
    assert not a(5.0) and not a(3.0) and not a(2.0) and a(2.4)      # closest 2.0 < 2.5, then growing: reached
    b = GD.Arrival(1.0, 1.5)
    assert not b(6.0) and not b(4.0) and not b(4.5)                 # closest 4.0: too far to count
    assert b(0.9)


# ---- §6 item 10 move-first + the trace rules, scout on omni_usv --------------------------------------------------------------------------
def test_scout_on_the_water_day_hull_moves_first_never_spins_fast_and_finishes(tmp_path):
    s = FastSil(SCOUT_W, "SCOUT", {"polygon": POLY, "log_dir": str(tmp_path)}, cfg=uw_cfg())
    cmds = traced(s)
    r = s.run(max_s=1100)
    assert r["state"] == "DONE", s.logs[-6:]
    vel = [c for c in cmds if isinstance(c[1], VelocityYawTarget)]
    assert not [c for c in cmds if isinstance(c[1], GuidedTarget)]                       # under way: no GUIDED carrot reaches the FCU
    assert max(abs(math.degrees(c[1].yaw_rate_rad_s)) for c in vel) <= YAW + 1e-6       # no |yaw cmd| > the cap
    assert vel[0][0] - cmds[0][0] < 1.0 and vel[0][1].vx_mps >= 0.4                       # the first command: forward >= 0.4 within 1 s
    first_yaw = next(c for c in vel if abs(c[1].yaw_rate_rad_s) > 0)
    assert math.hypot(first_yaw[2] - vel[0][2], first_yaw[3] - vel[0][3]) >= 1.0 - 0.05  # >= 1 m before the first heading change
    summ = s.task.lines[-1]
    assert summ["ev"] == "summary" and summ["visited"] >= 9
    assert all(G.inside((p[1], p[2]), POLY) for p in s.track)                             # never out of the course polygon


def test_move_first_is_skipped_with_an_object_close_ahead():
    mf = GD.MoveFirst(uw_cfg(), log=(logs := []).append)
    assert mf.cmd(0.0, pose(), obstacles=[(2.0, 0.2, 0.25)]) is None and "bow-clear" in logs[-1]
    mf = GD.MoveFirst(uw_cfg())
    c = mf.cmd(0.0, pose())
    assert c.vx_mps >= 0.4 and c.yaw_rate_rad_s == 0.0
    assert mf.cmd(5.0, pose(x=1.1, t=5.0)) is None and mf.done


def test_twenty_scout_runs_on_omni_usv_no_boundary_exit(tmp_path):
    """The addendum's acceptance: no boundary exit in 20 scout runs on omni_usv (the sketch: 95 % buoys, 0 exits)."""
    visited = exits = done = 0
    for k in range(20):
        s = FastSil(SCOUT_W, "SCOUT", {"polygon": POLY, "log_dir": str(tmp_path)}, cfg=uw_cfg(), seed=k + 1, start=STARTS[k % 4])
        r = s.run(max_s=1100)
        done += r["state"] == "DONE"
        exits += any(not G.inside((p[1], p[2]), POLY) for p in s.track)
        visited += sum(1 for line in s.task.lines if line["ev"] == "buoy" and line["visited"])
    assert exits == 0 and done == 20 and visited >= 190, (exits, done, visited)


# ---- T0.3 the circle scan, m1t1 on omni_usv ---------------------------------------------------------------------------------------------
def test_the_circle_scan_sweeps_360_forward_on_the_omni_hull():
    cfg = uw_cfg()
    p = Plant(1.28, 103.85, 0.0, hull="omni_usv")
    o = ControlOrigin(1.28, 103.85)
    b = ScanOrbit()
    t = 0.0

    def ctx():
        x, y = o.to_local(p.lat, p.lon)
        return BehaviourContext(cfg=cfg, now=t, fcu=FcuState(), pose=LocalPose(x, y, p.hdg_deg, o.lat, o.lon, t), origin=o, run=RunContext())
    b.start({"until": lambda c: False, "reason": "nothing"}, ctx())
    out = None
    while t < 200:
        out = b.step(ctx())
        if out.abort or out.done:
            break
        assert out.cmd.vx_mps == pytest.approx(0.3) and math.degrees(abs(out.cmd.yaw_rate_rad_s)) <= YAW + 1e-9
        p.step(0.1, out.cmd.vx_mps, 0.0, out.cmd.yaw_rate_rad_s)
        t += 0.1
    assert out.abort and out.info["swept_deg"] >= 359 and t <= 130.0, (t, out.info)     # 360 deg at ~3.2 deg/s (0.3 m/s) ~ 113 s
    assert 4.5 <= out.info["r_m"] <= 6.5


@pytest.mark.parametrize("planner", ["simple", "s6"])
def test_m1t1_completes_on_the_omni_hull(planner):
    s = FastSil(POOL_W, "T1", {"planner": planner}, cfg=uw_cfg(), seed=2)
    cmds = traced(s)
    r = s.run(max_s=1500)
    assert r["state"] == "DONE", (r["reason"], r["logs"][-6:])
    assert "CIRCLE_EXIT_CCW" in r["states"]
    assert max(abs(math.degrees(c[1].yaw_rate_rad_s)) for c in cmds if isinstance(c[1], VelocityYawTarget)) <= YAW + 1e-6


# ---- T0.5 the yaw couple test -----------------------------------------------------------------------------------------------------------
def test_test_yaw_frames_the_rate_and_the_course_refusal():
    assert yaw_frame("side", 0.6).as_list() == [0.0, 0.6, 0.0, -0.6]
    assert yaw_frame("aft", 0.6).as_list() == [0.6, 0.0, -0.6, 0.0]
    assert yaw_frame("both", 0.5, -1).as_list() == [-0.5, -0.5, 0.5, 0.5]
    assert mean_rate([(0.0, 350.0), (1.0, 355.0), (2.0, 0.0), (3.0, 5.0)]) == pytest.approx(5.0)
    course = Config()
    course.guards.test_mode = False
    t = TestYaw(course)
    tc = TaskContext(b=BehaviourContext(cfg=course, now=0.0, fcu=FcuState(), pose=pose(), origin=None, run=RunContext()), emit_event=lambda e: None)
    t.start({"pair": "side"}, tc)
    assert "refused on the course" in t.precheck(tc)
    pool = Config()
    pool.guards.test_mode = True
    t = TestYaw(pool)
    t.start({"pair": "side", "amp": 0.6}, tc)
    assert t.precheck(tc) == ""


def test_test_yaw_in_the_fast_sil_logs_the_hulls_rate(tmp_path):
    cfg = uw_cfg()
    cfg.guards.test_mode = True
    s = FastSil(SCOUT_W, "TEST_YAW", {"pair": "aft", "amp": 1.0, "duration_s": 12, "log_dir": str(tmp_path)}, cfg=cfg)
    r = s.run(max_s=30)
    assert r["state"] == "DONE"
    summ = s.task.lines[-1]
    assert summ["ev"] == "summary" and 0.5 < summ["rate_dps"] <= 2.0 + 0.05           # full differential, in place: <= 2 deg/s
    assert (tmp_path / s.task.log_path.split("/")[-1]).exists()


def test_yaw_side_gain_default_changes_nothing():
    g = Config().gains
    assert m20_frame(g, 1.0, 0.3, 0.0, 0.2).as_list() == pytest.approx([0.5, 0.0, 0.1, 0.0])
    g.yaw_side_gain = 0.5
    f = m20_frame(g, 1.0, 0.3, 0.0, 0.2).as_list()
    assert f[1] == pytest.approx(0.1) and f[3] == pytest.approx(-0.1)


# ---- WP46 our course only ---------------------------------------------------------------------------------------------------------------
def _buoy(m: PerceptionMap, x: float, y: float, colour: str, n: int = 6, t0: float = 0.0):
    for k in range(n):
        p = pose(t=t0 + k)
        rng, brg = math.hypot(x, y), math.degrees(math.atan2(y, x))
        m.ingest(Observation(t_cam=t0 + k, items=[ObsItem(id=int(x * 100 + y), cls="buoy", colour=colour, range_m=rng, bearing_deg=brg, conf=0.9)]),
                 p, now=t0 + k)


def test_a_neighbours_gate_outside_the_boundary_is_no_gate_but_stays_an_obstacle():
    from optiprimex.usv.gatepath import planner as gp
    m = PerceptionMap(Config())
    _buoy(m, 10.0, -2.0, "red")
    _buoy(m, 10.0, 2.0, "green")                                    # ours
    _buoy(m, 10.0, 18.0, "red")
    _buoy(m, 10.0, 22.0, "green")                                   # the neighbour's: 5 m outside the boundary at y_e 13
    assert len(m.gates()) == 2                                      # no boundary (bench): every pair
    m.set_course([(0.0, -13.0), (30.0, -13.0), (30.0, 13.0), (0.0, 13.0)], 1.5)
    gs = m.gates()
    assert len(gs) == 1 and abs(gs[0].mid_e) < 1.0
    assert len(gp.map_obstacles(m, 3)) == 4                         # all four kept clear of
    d = m.digest(pose(), now=10.0)
    out = [e for e in d["entities"] if e["outside_course"]]
    assert len(out) == 2 and d["n_outside_course"] == 2 and len(d["course_boundary_local"]) == 4
    assert m.in_course(0.0, 14.0) and not m.in_course(0.0, 15.0)    # + course_tol_m 1.5


def test_a_scout_target_outside_is_skipped_and_a_real_judge_needs_the_rxcourse():
    o = ControlOrigin(1.2810202, 103.8550097)
    cfg = uw_cfg()
    m = PerceptionMap(cfg)
    run = RunContext()
    tc = TaskContext(b=BehaviourContext(cfg=cfg, now=0.0, fcu=FcuState(), pose=pose(), origin=o, run=run, map=m), emit_event=lambda e: None)
    assert GD.course_refusal(cfg, tc) == ""                         # the fake judge (bench / sim): scout's square-ahead fallback stays
    m.ocs_judge = "protobuf"
    assert "no RxCourse on the boat" in GD.course_refusal(cfg, tc)
    run.course = Course(course_id="ALPHA", boundary=[o.from_local(0, -12.5), o.from_local(30, -12.5), o.from_local(30, 12.5), o.from_local(0, 12.5)])
    assert GD.course_refusal(cfg, tc) == ""
    from optiprimex.usv.tasks.scout import Scout
    sc = Scout(cfg)
    sc.start({"log_dir": "/tmp"}, tc)
    _buoy(m, 10.0, 0.0, "black")
    _buoy(m, 10.0, 16.0, "black")                                   # 3.5 m outside the RxCourse boundary
    ids = {(round(e.x_n), round(e.y_e)) for e in sc.candidates(tc)}
    assert ids == {(10, 0)}


def test_the_command_handler_refuses_scout_on_a_real_judge_without_rxcourse():
    from optiprimex.usv import command_handler as CH
    src = (ROOT / "optiprimex" / "usv" / "command_handler.py").read_text()
    assert "GD.course_refusal(self.runner.cfg, ctx)" in src and 'getattr(cls, "NEEDS_FIELD", False)' in src
    from optiprimex.usv.tasks import REGISTRY
    assert REGISTRY["SCOUT"].UNDERWAY and REGISTRY["T1"].UNDERWAY and not REGISTRY["T3"].UNDERWAY and REGISTRY["TEST_ORBIT"].UNDERWAY
    assert REGISTRY["SCOUT"].NEEDS_FIELD and REGISTRY["T1"].NEEDS_FIELD and not REGISTRY["TEST_ORBIT"].NEEDS_FIELD
    assert CH is not None


# ---- T0.2e the guard's blame radius --------------------------------------------------------------------------------------------------------
def test_a_guard_stop_far_from_the_target_does_not_block_it():
    from optiprimex.usv.guard import GuardSupervisor  # noqa: F401  (import check)
    src = (ROOT / "optiprimex" / "usv" / "guard.py").read_text()
    assert '("intermediate",) + tuple(key)' in src
    from optiprimex.usv.tasks.scout import Scout
    sc = Scout(uw_cfg())
    sc.p = {"standoff_m": 3.0}
    sc.target = {"id": 4, "xy": (10.0, 0.0)}
    sc.m.state = "APPROACH"
    assert sc.guard_target_xy() == (10.0, 0.0, 7.0)
