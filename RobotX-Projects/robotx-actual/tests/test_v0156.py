"""v0.1.56 (docs/OVERNIGHT_2026-11-07.md section 3, D6 / D7): m3t1 hardening in the fast SIL - no broker, `make quick`.

D6: the standoff lines derive from the dock geometry (usv/dockpath.standoff_lines: finger_m + hull_len_m / 2 + clear_m + a margin;
a non-null t3.standoff_m / standoff2_m is an explicit override, the TUNE knob sets it). D7: stage by stage - every ENTRY_POINT of T3
through `--from`, a park and RESUME at every stage boundary through `--step`, and the failure cases, each ending in the documented
state with its reason, the reports graded by the strict grader of v0.1.48 extended (campaign.report_order: each once, in order, each
from its own stage, never before DOCKED).
"""
from __future__ import annotations

import math

import pytest

from optiprimex.companion import tuning as TUN
from optiprimex.companion.config import Config, load as load_cfg
from optiprimex.sim import campaign as C
from optiprimex.sim import world as W
from optiprimex.sim.fake_fcu import Plant
from optiprimex.sim.fast_sil import FastSil
from optiprimex.usv.dockpath import FINGER_M, standoff_lines
from optiprimex.usv.tasks.t3_logistics import DEFAULTS, T3Logistics

from .conftest import ROOT

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
NEAR_DOCK = {"x_n": 32.0, "y_e": 3.0, "hdg_deg": 0.0}
LOOK = (34.0, 5.5, 0.0)                         # 11 m off the dock: all three faces in the camera
SD, SD2 = standoff_lines()
LINE_X = 45.0 - SD                              # the standoff line in world_pool (faces on x_n 45, the water to the south)
T3_BOUNDARIES = ["FIND_DOCK", "TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "DOCKED", "REPORT_DOCK", "AIM_FIRE",
                 "REPORT_FIRE", "UNDOCK"]
OBJS = W.load(POOL)["objects"]
FINGERS = [o for o in C.course_objects(W.load(POOL)) if o["kind"] == "pontoon"]


def _pontoon_clr(track, stages=None) -> float:
    """The hull rectangle's (1.19 x 0.76 m) smallest distance to a pontoon finger over the track samples in `stages`."""
    d = 99.0
    for _t, x, y, h, st in track:
        if stages is not None and st not in stages:
            continue
        hull = C._rect(x, y, h, C.HULL_LEN_M / 2, C.HULL_BEAM_M / 2)
        d = min(d, min(C._poly_dist(hull, f["pts"]) for f in FINGERS))
    return d


def _entry(stage: str, pose, args=None, look=LOOK) -> tuple:
    """The map filled from `look` (as the operator's boat had seen the dock), then the boat at `pose` and START_TASK --from stage."""
    s = FastSil(POOL, "T3", dict(args or {}), start=dict(zip(("x_n", "y_e", "hdg_deg"), look)), warmup_s=4.0)
    lat, lon = W.WorldFrame(W.load(POOL).get("origin")).to_latlon(pose[0], pose[1])
    s.plant = Plant(lat, lon, pose[2])
    return s, s.task.enter_from(stage, s.ctx())


def _strict(r, s, green=2, entry=None):
    if entry is None:
        return C.grade_m3t1(OBJS, r, green_bay=green, lit_window=s.dock.window if s.dock else None, strict=True)
    why = C.report_order(r, entry)
    return {"ok": not why, "why": "; ".join(why)}


# ---- D6: the standoff lines ---------------------------------------------------------------------------------------------------
def test_the_standoff_lines_derive_from_the_dock_geometry():
    assert (SD, SD2) == (round(3.5 + 1.19 / 2 + 0.60 + 1.0, 3), round(3.5 + 1.19 / 2 + 0.60 + 0.3, 3)) == (5.695, 4.995)
    assert DEFAULTS["standoff_m"] is None and DEFAULTS["standoff2_m"] is None and DEFAULTS["finger_m"] == FINGER_M
    for w in ("pool", "extra"):                                          # the default finger = the worlds' pontoons
        fs = [o for o in W.load(C.world_file(w))["objects"] if o.get("kind") == "pontoon"]
        ln = [math.hypot(f.get("x2_n", f.get("u2", 0)) - f.get("x_n", f.get("u", 0)), f.get("y2_e", f.get("v2", 0)) - f.get("y_e", f.get("v", 0))) for f in fs]
        assert ln and all(abs(x - FINGER_M) < 0.01 for x in ln), (w, ln)
    assert standoff_lines({"finger_m": 4.0}, {"clear_m": 0.5, "hull_len_m": 1.0}) == (6.0, 5.3)
    assert standoff_lines({"standoff_m": 6.5, "standoff2_m": None}) == (6.5, SD2)                  # non-null wins
    for prof in ("pool/usv_pool.yaml", "pool/usv_lab.yaml", "bench/usv_bench.yaml", "course/usv_course.yaml"):
        cfg = load_cfg(ROOT / "profiles" / prof)
        t3 = cfg.extra["t3"]
        assert t3["standoff_m"] is None and t3["standoff2_m"] is None and t3["finger_m"] == FINGER_M, prof
        assert standoff_lines(t3, cfg.extra.get("t1")) == (SD, SD2), prof
        assert TUN.current(cfg.extra, "standoff_m") == SD                                         # the TUNING card shows the derived line
    # the TUNE knob writes the override; the task then uses it (and the second pass keeps the formula)
    cfg = Config()
    cfg.extra = {"t3": {"standoff_m": None}}
    assert TUN.KNOBS["standoff_m"][1] == SD and TUN.current(cfg.extra, "standoff_m") == SD
    assert TUN.apply(cfg.extra, "standoff_m", 6.5) == (SD, 6.5) and TUN.diff({"t3": {"standoff_m": None}}, cfg.extra) == ["t3.standoff_m 5.695 -> 6.5"]
    s = FastSil(POOL, "T3", {}, cfg=cfg, start=NEAR_DOCK)
    assert (s.task.p["standoff_m"], s.task.p["standoff2_m"]) == (6.5, SD2)
    r = s.run(500)
    assert r["state"] == "DONE" and r["search"][0]["standoff_m"] == 6.5
    assert all(abs(x - (45.0 - 6.5)) < 0.8 for _, x, _, _, st in r["track"][5:] if st == "SWAY_SEARCH")


def test_the_default_run_sways_on_the_derived_line_and_keeps_the_fingers():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    r = s.run(500)
    g = _strict(r, s)
    assert g["ok"], (g["why"], r["logs"][-5:])
    assert r["search"][0]["standoff_m"] == SD and "standoff 5.70 / 5.00 m" in r["logs"][0]
    sway = [x for _, x, _, _, st in r["track"][5:] if st in ("SWAY_SEARCH", "SWAY_TO_ABEAM")]
    assert sway and all(abs(x - LINE_X) < 0.8 for x in sway)
    assert _pontoon_clr(r["track"], ("SWAY_SEARCH", "SWAY_TO_ABEAM")) >= 1.0          # the line keeps the hull well off the finger tips


# ---- D7: every entry point, every park --------------------------------------------------------------------------------------------
ENTRY_POSES = {"TO_LINE": ((34.0, 5.5, 0.0), {}), "SWAY_SEARCH": ((36.0, 5.5, 0.0), {}), "SWAY_TO_ABEAM": ((36.0, 3.0, 0.0), {}),
               "TURN_IN": ((LINE_X, 5.5, 20.0), {}), "ENTER": ((LINE_X, 5.5, 0.0), {}), "AIM_FIRE": ((43.5, 5.5, 0.0), {"fire_s": 12.0}),
               "UNDOCK": ((43.5, 5.5, 0.0), {})}


def test_every_entry_point_through_from():
    assert set(ENTRY_POSES) == set(T3Logistics.ENTRY_POINTS)
    for stage, (pose, args) in ENTRY_POSES.items():
        s, why = _entry(stage, pose, args)
        assert why == "", (stage, why)
        r = s.run(500)
        assert r["state"] == "DONE" and r["states"][0] == stage, (stage, r["state"], r["logs"][-4:])
        g = _strict(r, s, entry=stage)
        assert g["ok"], (stage, g["why"])
        bays = {e.get("bay") for e in r["events"] if e.get("event") in ("docking", "firefighting")}
        assert bays <= {2}, (stage, bays)                                # the GREEN bay, numbered on the whole line
        if T3_BOUNDARIES.index(stage) <= T3_BOUNDARIES.index("ENTER"):
            assert [k for k, _ in [(e[0], e[1]) for e in r["event_log"]] if k in ("docking", "firefighting")] == ["docking", "firefighting"]
        if stage == "UNDOCK":
            assert not r["events"] and 45.0 - r["track"][-1][1] >= SD - 0.3
    # a partial view (two faces of three) numbers the bays wrong: the entry points then refuse (v0.1.56)
    s = FastSil(POOL, "T3", {}, start={"x_n": LINE_X, "y_e": 5.5, "hdg_deg": 20.0}, warmup_s=4.0)
    why = s.task.enter_from("TURN_IN", s.ctx())
    assert "the dock line (every face" in why and s.task.m.state == "WAIT_START"
    assert s.task.enter_from("BACK_OUT", s.ctx()).startswith("BACK_OUT is not an entry point")


def test_a_park_and_resume_at_every_stage_boundary():
    s = FastSil(POOL, "T3", {"step": True}, start=NEAR_DOCK)
    parks = []
    for _ in range(20):
        r = s.run(400, resume_steps=False)
        if r["state"] != "SAFE_HOLD":
            break
        assert r["safe_holds"][-1].startswith("step: next "), r["safe_holds"]
        parks.append(r["safe_holds"][-1])
        p0 = s.pose()
        held = s.run(3.0, resume_steps=False)                            # parked: the task holds (nothing moves it on)
        assert held["state"] == "TIMEOUT" and s.task.m.state == r["sub_state"]
        assert math.hypot(s.pose().x_n - p0.x_n, s.pose().y_e - p0.y_e) < 1.0
        assert not [e for e in held["event_log"][len(r["event_log"]):]]  # and reports nothing while parked
        s.task.resume()                                                  # the operator's RESUME
    assert r["state"] == "DONE", (r["state"], r["logs"][-4:])
    assert parks == [f"step: next {x}" for x in T3_BOUNDARIES]
    g = _strict(r, s)
    assert g["ok"], g["why"]


# ---- D7: the failure cases ----------------------------------------------------------------------------------------------------
def test_the_wrong_call_backs_out_to_the_right_bay():
    s = FastSil(POOL, "T3", {"bay": 1}, start=NEAR_DOCK)                 # --bay 1 on a GREEN-2 world
    r = s.run(600)
    assert r["state"] == "DONE", r["logs"][-4:]
    assert "BACK_OUT" in r["states"] and r["reengaged"] == 1 and r["search"][-1]["excluded"] == [1] and r["search"][-1]["chosen"] == 2
    assert any("wrong call: bay 1 reads RED" in x for x in r["logs"])
    i = r["states"].index("BACK_OUT")
    assert r["states"][i + 1] == "SWAY_SEARCH" and "DOCKED" not in r["states"][:i]
    g = C.report_order(r)
    assert not g and [e["bay"] for e in r["events"] if e["event"] == "docking"] == [2]


def test_red_on_the_way_in_backs_out_at_most_reengage_max_then_holds():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    nxt, moved = {2: 3, 3: 1, 1: 2}, []

    def flip(x):                                                         # the chosen bay turns RED as the boat bows in, three times
        if x.task.m.state == "TURN_IN" and x.task.bay.number not in moved and len(moved) < 3:
            moved.append(x.task.bay.number)
            x.dock.command({"cmd": "green_bay", "bay": nxt[x.task.bay.number]}, x.t)
        return False
    r = s.run(900, until=flip)
    assert moved == [2, 3, 1]
    assert r["state"] == "SAFE_HOLD" and r["sub_state"] in ("TURN_IN", "ENTER"), (r["state"], r["logs"][-4:])
    assert r["reengaged"] == int(DEFAULTS["reengage_max"]) == 2 and r["states"].count("BACK_OUT") == 2
    assert "reads RED on the way in" in r["safe_holds"][0] and "2 re-engagement(s) used - holding" in r["safe_holds"][0]
    wrong = [x for x in r["logs"] if "wrong call" in x]
    assert len(wrong) == 2 and all("sightings in" in x for x in wrong)
    n = [int(x.split("(")[1].split(" sightings")[0]) for x in wrong]
    assert all(k >= 3 for k in n)                                        # >= 3 sightings over >= bay_confirm_s
    assert not [e for e in r["events"] if e["event"] in ("docking", "firefighting")] and not C.report_order(r)


def test_the_dock_line_lost_mid_search_holds_with_the_reason():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    s.run(300, until=lambda x: x.task.m.state == "SWAY_SEARCH")
    s.run(5)
    s.map.clear()                                                        # CLEAR_MAP's effect (or the line aged out)
    r = s.run(30)
    assert r["state"] == "SAFE_HOLD" and r["sub_state"] == "SWAY_SEARCH", (r["state"], r["logs"][-3:])
    assert r["safe_holds"] == ["T3 SWAY_SEARCH: the dock line is lost (bay(s) [1, 2, 3] no longer in the map)"]
    assert not [e for e in r["events"] if e["event"] in ("docking", "firefighting")] and r["search"] == []
    from optiprimex.companion.state import Hold
    p0 = s.pose()
    s.run(5)
    assert isinstance(s.cmd, Hold) and math.hypot(s.pose().x_n - p0.x_n, s.pose().y_e - p0.y_e) < 0.5    # held there


def test_no_green_after_two_passes_holds_and_sends_no_docking_report():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    s.dock.green_bay = 99                                                # every indicator RED
    r = s.run(600)
    assert r["state"] == "SAFE_HOLD" and r["sub_state"] == "SWAY_SEARCH", (r["state"], r["logs"][-3:])
    assert [p["standoff_m"] for p in r["search"]] == [SD, SD2] and all(p["chosen"] is None for p in r["search"])
    assert "no single GREEN bay after two passes" in r["safe_holds"][0]
    assert not [e for e in r["events"] if e["event"] in ("docking", "firefighting")]
    assert _pontoon_clr(r["track"], ("SWAY_SEARCH",)) >= C.CLEAR_M                  # the second pass too keeps the 0.60 m rule


def test_the_window_never_lit_reports_incomplete_after_fire_s():
    s = FastSil(POOL, "T3", {"fire_s": 16.0}, start=NEAR_DOCK)
    s.dock = None                                                        # nobody lights a window
    r = s.run(600)
    assert r["state"] == "DONE", r["logs"][-4:]
    (ff,) = [e for e in r["events"] if e["event"] == "firefighting"]
    assert ff["complete"] is False and 15.5 <= ff["fired_s"] <= 17.0 and ff["result"] == "not seen green"
    assert not C.report_order(r)


def test_green_moved_mid_search_the_choice_follows():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    s.run(300, until=lambda x: x.task.m.state == "SWAY_SEARCH")
    s.dock.command({"cmd": "green_bay", "bay": 3}, s.t)
    r = s.run(600)
    g = _strict(r, s, green=3)
    assert g["ok"], (g["why"], r["logs"][-4:])
    assert r["search"][-1]["chosen"] == 3


@pytest.mark.parametrize("start", [(42.5, 5.5, 180.0), (42.5, 5.5, 0.0), (42.0, 2.0, 90.0)])
def test_a_start_inside_the_standoff_line_backs_out_to_it_first(start):
    s = FastSil(POOL, "T3", {}, start=dict(zip(("x_n", "y_e", "hdg_deg"), start)))   # 2.5 - 3 m off the dock, between the fingers
    r = s.run(600)
    g = _strict(r, s)
    assert g["ok"], (g["why"], r["logs"][-5:])
    assert any("inside the standoff line" in x and "out along the normal" in x for x in r["logs"]), r["logs"][:4]
    to_line = [(x, y) for _, x, y, _, st in r["track"] if st == "TO_LINE"]
    k = next(i for i, (x, _) in enumerate(to_line) if x <= LINE_X + 0.3)  # out to the line first ...
    assert all(abs(y - start[1]) < 0.6 for _, y in to_line[:k])           # ... straight out along the normal (no sideways sweep)
    assert _pontoon_clr(r["track"]) >= C.CLEAR_M                                    # the scan in the bay and the way out keep the rule


def test_undock_astern_clears_the_fingers_by_the_rule():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    r = s.run(500)
    assert r["state"] == "DONE"
    assert _pontoon_clr(r["track"], ("UNDOCK",)) >= C.CLEAR_M
    assert _pontoon_clr(r["track"], ("TURN_IN", "ENTER", "DOCKED", "AIM_FIRE")) >= C.CLEAR_M
    hc = C.hull_clearance(W.load(POOL), r["track"], "m3t1")
    assert hc["ok"], hc


def test_the_strict_grader_catches_every_report_fault():
    ok = {"state": "DONE", "states": ["DOCKED", "REPORT_DOCK", "AIM_FIRE", "REPORT_FIRE", "UNDOCK", "DONE"],
          "event_log": [("docking", "REPORT_DOCK", 1.0), ("firefighting", "REPORT_FIRE", 2.0)]}
    assert C.report_order(ok) == []
    twice = dict(ok, event_log=ok["event_log"] + [("docking", "REPORT_DOCK", 3.0)])
    assert "2 docking reports" in C.report_order(twice)
    early = dict(ok, states=["ENTER", "REPORT_DOCK", "DOCKED"], event_log=[("docking", "REPORT_DOCK", 1.0)])
    assert "DockingReport before DOCKED" in C.report_order(early)
    stage = dict(ok, event_log=[("docking", "ENTER", 1.0), ("firefighting", "REPORT_FIRE", 2.0)])
    assert any("a docking report from ['ENTER']" in x for x in C.report_order(stage))
    order = dict(ok, event_log=[("firefighting", "REPORT_FIRE", 1.0), ("docking", "REPORT_DOCK", 2.0)])
    assert "FirefightingReport before the DockingReport" in C.report_order(order)
    held = {"state": "SAFE_HOLD", "states": ["SWAY_SEARCH"], "event_log": [("docking", "REPORT_DOCK", 1.0)]}
    assert "a DockingReport without DOCKED" in C.report_order(held)
    assert C.report_order(dict(ok, event_log=[("firefighting", "REPORT_FIRE", 1.0)]), entry="AIM_FIRE") == []
    assert C.report_order(dict(ok, event_log=[("firefighting", "REPORT_FIRE", 1.0)])) != []


# ---- D8: what the campaign found ----------------------------------------------------------------------------------------------
def test_the_line_is_refitted_from_the_map_before_lining_up():
    """The line locked from 20 m (camera campaign pool seed 2: the normal 17 deg off) is refitted from the same face entities'
    current positions, whatever their age (abeam one bay the camera sees no other)."""
    import types
    from optiprimex.usv.dockpath import Bay, DockGeom, refit
    bad = DockGeom((44.2, 4.7), (-0.956, -0.294), (-0.294, 0.956),
                   [Bay(1, (44.93, 1.95), "red", 2), Bay(2, (44.45, 5.03), "green", 5), Bay(3, (43.28, 7.22), "red", 8)])
    ents = {i: types.SimpleNamespace(x_n=45.0, y_e=y) for i, y in ((2, 2.0), (5, 5.5), (8, 9.0))}
    g = refit(bad, types.SimpleNamespace(ents=ents))
    assert g.normal[0] == pytest.approx(-1.0) and abs(g.normal[1]) < 1e-9 and [b.face for b in g.bays] == [(45.0, 2.0), (45.0, 5.5), (45.0, 9.0)]
    assert [(b.number, b.colour, b.entity_id) for b in g.bays] == [(1, "red", 2), (2, "green", 5), (3, "red", 8)]
    assert refit(bad, types.SimpleNamespace(ents={2: ents[2]})) is bad                 # one face: nothing to fit
    wf = C.world_file("pool")
    st = C.m3t1_start(W.load(wf), 2)
    s = FastSil(wf, "T3", {}, start={k: st[k] for k in ("x_n", "y_e", "hdg_deg")}, seed=2, perception="camera", cam_dock=True)
    s.dock.command({"cmd": "green_bay", "bay": st["green_bay"]}, s.t)
    r = s.run(600)
    g = C.grade_m3t1(OBJS, r, green_bay=st["green_bay"], lit_window=s.dock.window, strict=True)
    assert g["ok"] and r["reengaged"] == 0 and C.finger_clearance(W.load(wf), r["track"]) >= 1.0, (g["why"], r["logs"][-4:])


def test_the_campaign_m3t1_on_every_dock_world_and_marina_gets_windows():
    cs = C.cases(["m3t1"], 5, ["pool", "marina", "extra", "boxes"], ["-"], ["geometric", "camera"], detectors=("synthetic", "model"),
                 model=str(ROOT), model_seeds=4)
    assert {c["world"] for c in cs} == {"pool", "marina", "extra"} and all(c["start"] == "random" for c in cs)
    per = {(c["world"], c["perception"], c.get("detector")): 0 for c in cs}
    for c in cs:
        per[(c["world"], c["perception"], c.get("detector"))] += 1
    assert set(per.values()) <= {5, 4} and all(v == 4 for k, v in per.items() if k[2] == "model") and all(c.get("device", "cpu") == "cpu" for c in cs)
    m = W.load(C.world_file("marina"))
    assert not [o for o in m["objects"] if o["kind"] == "window"]
    ww = [o for o in C.with_windows(m)["objects"] if o["kind"] == "window"]
    assert len(ww) == 6 and {(o["bay"], o["window"]) for o in ww} == {(b, k) for b in (1, 2, 3) for k in (1, 2)}
    assert C.with_windows(W.load(POOL)) is None and C.world_finger(W.load(POOL)) == FINGER_M and C.world_finger(m) is None
