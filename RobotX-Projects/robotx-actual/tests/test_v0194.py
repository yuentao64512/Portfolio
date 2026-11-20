"""v0.1.94 (Dmitrii 2026-11-14, after the 13 Nov afternoon run): the lamp blink decision on the boat by world bearing, the lamp's
colour on the bearing-only sightings too, the buoy merge pass (range + number), far depth as a bearing, report ids from 0, CHECK_BLUE /
LOOK_LOCK holding still."""
from __future__ import annotations

import importlib.util
import math
import random
import sys
import types

import pytest

from optiprimex.companion.config import Config, load as load_cfg
from optiprimex.companion.state import LocalPose, MapEntity, Observation, ObsItem
from optiprimex.usv.lamp_tracks import LampTracker, pattern_runs, DEFAULTS as LT
from optiprimex.usv.perception_map import PerceptionMap

from .conftest import ROOT

COURSE = ROOT / "profiles" / "course" / "usv_course.yaml"


def _led_core():
    spec = importlib.util.spec_from_file_location("led_core_v0194", ROOT / "cv" / "buoys_multi" / "led_core.py")
    m = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = m
    spec.loader.exec_module(m)
    return m


def _flash(t0, n, dt=0.43, period=2.0, colour="blue", phase=0.0):
    return [(t0 + i * dt, ((t0 + i * dt + phase) % period) < period / 2, colour) for i in range(n)]


def test_the_boats_rule_is_led_cores_run_rule():
    LC = _led_core()
    rnd = random.Random(4)
    for k in range(60):
        dt = rnd.uniform(0.2, 0.6)
        kind = k % 3
        s = []
        for i in range(rnd.randint(4, 30)):
            t = i * dt + rnd.uniform(0, 0.05)
            on = True if kind == 0 else ((t % 2.0) < 1.0 if kind == 1 else rnd.random() < 0.5)
            s.append((t, on and rnd.random() > 0.05, rnd.choice(["blue", "blue", "blue", "red"]) if on else None))
        p = dict(LC.DEFAULTS, min_samples=8)
        tr = LC.Track(1, 0, 0, 10, 0, samples=__import__("collections").deque(s))
        a = LC.LedCache(p).pattern_runs(tr)
        b = pattern_runs(s, {**LT, **{k2: p[k2] for k2 in LT if k2 in p}})
        a.pop("track")
        assert a == b, (k, a, b)


def test_a_lamp_keeps_one_track_while_the_boat_yaws():
    lt = LampTracker()
    hdg = 0.0
    for i in range(20):                       # the boat turns 4 deg a frame; the lamp stays at 30 deg true (its bearing runs 30 .. -46)
        t = i * 0.43
        on = (t % 2.0) < 1.0
        (tr,) = lt.frame(t, [(0.0, 0.0, 30.0 + random.Random(i).uniform(-1.5, 1.5), on, "blue" if on else None)])
        hdg += 4.0
    assert lt.stats["tracks"] == 1 and tr.result["pattern"] == "flashing" and tr.result["colour"] == "blue"


def test_two_lamps_apart_in_azimuth_are_two_tracks_and_a_lost_one_goes():
    lt = LampTracker()
    a = b = None
    for i in range(12):
        a, b = lt.frame(i * 0.5, [(0, 0, 10.0, True, "red"), (0, 0, 40.0, True, "green")])
    assert a.tid != b.tid and a.result["colour"] == "red" and b.result["colour"] == "green" and a.result["pattern"] == "steady"
    lt.frame(20.0, [])
    assert lt.tracks == {}


def _pm(cfg=None, extra=None):
    cfg = cfg or Config()
    if extra:
        cfg.extra = dict(cfg.extra or {}, perception=dict((cfg.extra or {}).get("perception") or {}, **extra))
    now = [0.0]
    return PerceptionMap(cfg, clock=lambda: now[0]), now


def _item(r, b, on=None, col=None, colour="white", number=None):
    q = {}
    if on is not None:
        q["led"] = {"on": on, "now": col if on else None, "pattern": "unknown", "colour": None}
    if number is not None:
        q["number"] = {"digit": number, "p": 0.95}
    return ObsItem(id=0, cls="buoy", colour=colour, conf=0.9, range_m=r, bearing_deg=b, quality=q)


POSE = LocalPose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def test_bearing_only_lamp_readings_colour_the_ray_and_then_the_buoy():
    pm, now = _pm()
    for i in range(20):                       # far: bearing only, a flashing red lamp at 20 deg
        t = i * 0.43
        now[0] = t
        on = (t % 2.0) < 1.0
        pm.ingest(Observation(t_cam=t, items=[_item(0.0, 20.0, on, "red")]), POSE, now=t)
    assert any(r["colour"] == "red" for r in pm.rays)
    for i in range(20, 26):                   # nearer: depth arrives (the body reads white) - the lamp already decided the buoy
        t = i * 0.43
        now[0] = t
        on = (t % 2.0) < 1.0
        pm.ingest(Observation(t_cam=t, items=[_item(9.0, 20.0, on, "red")]), POSE, now=t)
    (e,) = pm.entities("buoy", include_young=True)
    assert e.colour == "red" and e.light == "flash" and e.led["pattern"] == "flashing"


def test_a_lamps_colour_colours_the_buoy_before_its_pattern_is_decided():
    pm, now = _pm()
    for i in range(4):
        now[0] = i * 0.43
        pm.ingest(Observation(t_cam=now[0], items=[_item(6.0, 0.0, True, "green")]), POSE, now=now[0])
    (e,) = pm.entities("buoy", include_young=True)
    assert e.colour == "green" and e.light == "?" and e.lamp["colour"] == "green"


def _e(i, x, y, n=10, rmin=20.0, number=None, lamp=None):
    e = MapEntity(i, "buoy", "white", x, y, 0.3, n, 0.0, 10.0, colour_votes={"white": n})
    e.min_range_m, e.number, e.ranged = rmin, number, True
    if lamp:
        e.lamp = {"colour": lamp, "pattern": "unknown"}
    return e


def test_the_merge_radius_grows_with_range_and_with_a_number():
    pm, _ = _pm(load_cfg(COURSE))
    near, far = _e(1, 0, 0, rmin=5.0), _e(2, 0, 1.5, rmin=5.0)
    assert pm.merge_radius(near, far) == pytest.approx(0.8)                       # seen near: tight (markers 1.4-3 m apart)
    assert pm.merge_radius(_e(1, 0, 0, rmin=24), _e(2, 0, 2, rmin=24)) == pytest.approx(2.27)
    assert pm.merge_radius(_e(1, 0, 0, rmin=5, number=8), _e(2, 0, 4, rmin=5, number=8)) == 5.0


def test_merge_pass_merges_ghosts_never_two_numbers_or_two_lamps():
    pm, _ = _pm(load_cfg(COURSE))
    pm.ents = {1: _e(1, 0, 0, n=12), 2: _e(2, 0, 1.8, n=4),                       # far-only ghosts 1.8 m apart: one
               3: _e(3, 10, 0, rmin=5), 4: _e(4, 10, 1.5, rmin=5),                # near markers 1.5 m apart: two
               5: _e(5, 20, 0, rmin=5, number=3), 6: _e(6, 20, 4.0, rmin=5, number=3),   # one number: one
               7: _e(7, 30, 0, rmin=5, number=1), 8: _e(8, 30, 0.3, rmin=5, number=7),   # two numbers: two
               9: _e(9, 40, 0, lamp="red"), 10: _e(10, 40, 1.0, lamp="green")}   # two lamps: two
    assert pm.merge_buoys(1.0) == 2
    assert set(pm.ents) == {1, 3, 4, 5, 7, 8, 9, 10} and pm.ents[1].n_obs == 16 and pm.ents[1].y_e == pytest.approx(0.45)
    assert pm.resolve(2) == 1 and pm.resolve(6) == 5 and pm.resolve(3) == 3 and pm.resolve(None) is None


def test_far_depth_counts_as_a_bearing():
    pm, now = _pm(load_cfg(COURSE))
    assert pm.ranged_max_m == 15.0
    pm.ingest(Observation(t_cam=0.0, items=[_item(22.0, 5.0)]), POSE, now=0.0)
    assert pm.ents == {} and len(pm.rays) == 1
    pm.ingest(Observation(t_cam=0.5, items=[_item(12.0, 5.0)]), POSE, now=0.5)
    assert len(pm.ents) == 1


def test_numbers_vote_and_need_two_reads():
    e = _e(1, 0, 0)
    e.number = None
    PerceptionMap._number_vote(e, {"digit": 8, "p": 0.9})
    assert e.number is None
    PerceptionMap._number_vote(e, {"digit": 8, "p": 0.9})
    assert e.number == 8
    PerceptionMap._number_vote(e, {"digit": 3, "p": 0.9})
    PerceptionMap._number_vote(e, {"digit": 3, "p": 0.9})
    assert e.number is None                                                       # 50/50: unknown


def test_report_ids_run_from_0_and_follow_a_merge():
    from optiprimex.sim.fast_sil import FastSil
    s = FastSil(str(ROOT / "profiles" / "bench" / "world_pool.yaml"), "T1", {}, cfg=load_cfg(COURSE))
    s.sense = lambda: None
    t = s.task
    t.m.state = "SEARCH_ENTRY"
    s.map.ents = {41: _e(41, 5, 0), 57: _e(57, 5, 6), 63: _e(63, 9, 0, rmin=24)}
    for e in s.map.ents.values():
        e.colour, e.light = "red", "flash"
    t.maybe_report(s.ctx())
    rep = [e for e in s.events if e.get("event") == "safe_passage"][-1]
    assert [(b["id"], b["map_id"]) for b in rep["buoys"]] == [(0, 41), (1, 57), (2, 63)]
    s.map._absorb(s.map.ents[41], s.map.ents[63])                                 # 63 goes into 41
    t.entry_id = 63
    t.maybe_report(s.ctx())                                                       # at once (not after report_period_s)
    rep = [e for e in s.events if e.get("event") == "safe_passage"][-1]
    assert [(b["id"], b["map_id"]) for b in rep["buoys"]] == [(0, 41), (1, 57)] and t.entry_id == 41


def test_check_blue_holds_still_once_facing():
    from optiprimex.usv.behaviours.scan_yaw import ScanYaw
    from optiprimex.usv.behaviours.base import BehaviourContext
    from optiprimex.companion.state import VelocityYawTarget

    def ctx(h, now):
        return BehaviourContext(cfg=Config(), now=now, fcu=None, pose=LocalPose(0, 0, h, 0.0, 0.0, now), origin=None, run=None)
    b = ScanYaw()
    b.start({"until": lambda c: False, "face": (10.0, 0.0), "max_s": 30.0, "rate_dps": 8.0, "still_deg": 4.0}, ctx(20.0, 0.0))
    assert b.step(ctx(20.0, 1.0)).cmd.yaw_rate_rad_s != 0.0
    out = b.step(ctx(3.0, 2.0))
    assert isinstance(out.cmd, VelocityYawTarget) and out.cmd.yaw_rate_rad_s == 0.0 and out.info["still"]
    assert b.step(ctx(7.0, 3.0)).cmd.yaw_rate_rad_s == 0.0                       # hysteresis: still up to 2x
    assert b.step(ctx(9.0, 4.0)).cmd.yaw_rate_rad_s != 0.0
    cfg = load_cfg(COURSE)
    t1 = (cfg.extra or {}).get("t1") or {}
    assert t1.get("check_still_deg") == 4.0 and t1.get("check_s") == 10.0


def test_cv_publish_passes_the_lamp_and_number_on_bearing_only_items():
    spec = importlib.util.spec_from_file_location("cvp_v0194", ROOT / "cv" / "publish" / "cv_publish.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    pub = types.SimpleNamespace(a=types.SimpleNamespace(colour="unknown"))
    meas = {"bearing_deg": 3.0, "range_m": None, "bearing_only": True, "reason": "beyond_depth", "bbox": [0, 10, 20, 52],
            "led": {"pattern": "unknown", "colour": None, "on": True, "now": "red", "track": 4}, "number": {"digit": 8, "p": 0.9, "cls": "8"}}
    (it,) = m.Publisher.frame_to_items(pub, meas, 3, "white", 0)
    assert it["quality"]["led"] == {"pattern": "unknown", "colour": None, "on": True, "now": "red"}
    assert it["quality"]["number"]["digit"] == 8 and it["quality"]["bbox_h"] == 42.0


def test_a_depthless_box_takes_the_guards_range_on_its_bearing():
    from optiprimex.usv.guard import GuardSupervisor
    cfg = load_cfg(COURSE)
    pm, now = _pm(cfg)
    gs = GuardSupervisor((cfg.extra or {}).get("zed_guard"))
    box = ObsItem(id=1, cls="buoy", colour="white", conf=0.9, range_m=0.0, bearing_deg=10.0,
                  quality={"bearing_only": True, "reason": "depth_invalid", "bbox_h": 170.0})          # ~6 m by its size
    far_box = ObsItem(id=2, cls="buoy", colour="white", conf=0.9, range_m=0.0, bearing_deg=-20.0,
                      quality={"bearing_only": True, "reason": "depth_invalid", "bbox_h": 40.0})       # ~25 m: not the 6 m thing
    pm.ingest(Observation(t_cam=5.0, items=[box, far_box]), POSE, now=5.0)
    msg = {"t_cam": 5.0, "far": [{"range_m": 5.8, "bearing_deg": 9.0, "bearing_min_deg": 8.0, "bearing_max_deg": 12.0, "px": 4000},
                                 {"range_m": 6.0, "bearing_deg": -20.0, "bearing_min_deg": -21.0, "bearing_max_deg": -19.0, "px": 4000}]}
    gs.map_far(msg, POSE, pm, 5.0)
    buoys = [e for e in pm.ents.values() if e.kind == "buoy"]
    obst = [e for e in pm.ents.values() if e.kind == "obstacle"]
    assert len(buoys) == 1 and math.hypot(buoys[0].x_n, buoys[0].y_e) == pytest.approx(6.0, abs=0.05)
    assert len(obst) >= 1 and all(e.y_e < 0 for e in obst)            # the -20 deg cluster is still an obstacle (the far box isn't it)
    pm2, _ = _pm(cfg)
    pm2.ingest(Observation(t_cam=5.0, items=[box]), POSE, now=5.0)
    assert pm2.fuse_depth(dict(msg, t_cam=5.5), POSE, 5.5) == set()     # another frame: never


def test_a_far_steady_lamp_behind_a_near_flashing_one_is_another_track():
    lt = LampTracker()
    near = far = None
    for i in range(24):                       # the entry flashing at ~10 m, the exit steady at ~28 m, on one bearing (13 Nov pm)
        t = i * 0.43
        on = (t % 2.0) < 1.0
        near, far = lt.frame(t, [(0, 0, 200.0, on, "blue" if on else None, 10.0), (0, 0, 201.0, True, "blue", 28.0)])
    assert near.tid != far.tid and near.result["pattern"] == "flashing" and far.result["pattern"] == "steady"


def test_a_range_its_box_denies_is_dropped_and_one_lamp_seen_twice_merges():
    pm, now = _pm(load_cfg(COURSE))
    wrong = ObsItem(id=1, cls="buoy", colour="white", conf=0.9, range_m=21.9, bearing_deg=0.0, quality={"bbox_h": 148.0})  # ~6.8 m by size
    pm.ingest(Observation(t_cam=0.0, items=[wrong]), POSE, now=0.0)
    assert pm.ents == {} and pm.n_size_mismatch == 1
    a, b = _e(1, 0, 0, rmin=5), _e(2, 0, 2.3, rmin=5)
    a.led = b.led = {"pattern": "flashing", "colour": "blue", "n": 5}
    assert pm.merge_radius(a, b) == 3.0 and not pm.merge_veto(a, b)
    b.led = {"pattern": "steady", "colour": "blue", "n": 5}
    assert pm.merge_radius(a, b) == pytest.approx(0.8) and not pm.merge_veto(a, b)     # a pattern alone never vetoes


def test_lucky_orbits_a_blue_whose_light_stays_undecided():
    from optiprimex.sim.fast_sil import FastSil
    for lucky, want in ((True, "CIRCLE_ENTRY_CW"), (False, None)):
        s = FastSil(str(ROOT / "profiles" / "bench" / "world_pool.yaml"), "T1", {"lucky": lucky}, cfg=load_cfg(COURSE))
        s.sense = lambda: None
        t = s.task
        e = _e(7, 5, 0, rmin=5)
        e.colour, e.light = "blue", "?"
        s.map.ents = {7: e}
        t.m.state, t.check_id, t.check_xy = "CHECK_BLUE", 7, (5.0, 0.0)
        t.checks[7] = int(t.p["check_tries"]) - 1
        t._check_result(s.ctx(), decided=False)
        if want:
            assert t.m.state == want and t.entry_id == 7 and any("LUCKY" in l for l in s.logs)
        else:
            assert t.entry_id is None and t.m.state != "CIRCLE_ENTRY_CW"


def test_blue_agnostic_lucky_orbits_any_blue_not_read_steady_after_one_check():
    from optiprimex.sim.fast_sil import FastSil
    for light, mode, want in (("off", "any", True), ("?", "any", True), ("steady", "any", False), ("?", True, False)):
        s = FastSil(str(ROOT / "profiles" / "bench" / "world_pool.yaml"), "T1", {"lucky": True, "lucky_blue": mode}, cfg=load_cfg(COURSE))
        s.sense = lambda: None
        t = s.task
        e = _e(7, 5, 0, rmin=5)
        e.colour, e.light = "blue", light
        s.map.ents = {7: e}
        t.m.state, t.check_id, t.check_xy = "CHECK_BLUE", 7, (5.0, 0.0)
        t._check_result(s.ctx(), decided=light == "steady")          # the first check
        assert (t.entry_id == 7 and t.m.state == "CIRCLE_ENTRY_CW") == want, (light, mode, t.m.state)
        if light == "steady":
            assert 7 in t.exit_seen


def test_a_sightings_colour_is_its_lamp_so_t1s_near_votes_see_blue():
    pm, now = _pm()
    for i, on in enumerate((True, False, True)):
        now[0] = i * 0.43
        pm.ingest(Observation(t_cam=now[0], items=[_item(5.0, 0.0, on, "blue")]), POSE, now=now[0])
        (e,) = pm.ents.values()
        assert e.colour_now == ("blue" if on else "black")            # the body says white; T1's watch_locked reads colour_now
    pm.ingest(Observation(t_cam=9.0, items=[_item(5.0, 0.0)]), POSE, now=9.0)
    assert e.colour_now == "white"                                    # no lamp reading (an older observer): the body as before


def test_a_well_mapped_gate_goes_faster_a_doubtful_one_stays_careful():
    from optiprimex.sim.fast_sil import FastSil
    from optiprimex.usv.gatepath import planner as gp
    s = FastSil(str(ROOT / "profiles" / "bench" / "world_pool.yaml"), "T1", {"through_v_conf": 0.3, "transit_v_conf": 0.4}, cfg=load_cfg(COURSE))
    s.sense = lambda: None
    t = s.task
    r, g = _e(1, 10, 1.25, n=14, rmin=6.0), _e(2, 10, -1.25, n=14, rmin=6.0)
    r.colour, g.colour = "red", "green"
    r.led, g.led = {"pattern": "flashing", "colour": "red", "n": 4}, {"pattern": "flashing", "colour": "green", "n": 4}
    r.sigma_m = g.sigma_m = 0.2
    s.map.ents = {1: r, 2: g}
    t.current = gp.GateItem((1, 2), (10.0, 1.25), (10.0, -1.25), (10.0, 0.0), (1.0, 0.0), 2.5)
    ctx = s.ctx()
    assert t.gate_confident(ctx) and t._gate_v(ctx, "through") == 0.3 and t._gate_v(ctx, "transit") == 0.4
    g.min_range_m = 14.0                                              # never seen near: doubtful
    assert not t.gate_confident(ctx) and t._gate_v(ctx, "through") == t.p["through_v"]
    g.min_range_m, g.led = 6.0, None                                  # its colour from the body votes only
    g.lamp = {"colour": None}
    assert not t.gate_confident(ctx)
    t.p["through_v_conf"] = 0.0                                       # off: the plain speed even when confident
    g.led = {"pattern": "flashing", "colour": "green", "n": 4}
    assert t.gate_confident(ctx) and t._gate_v(ctx, "through") == t.p["through_v"]


def test_transit_with_no_gate_picked_never_crashes():
    """v0.1.94.1: LUCKY_GO entered TRANSIT from a behaviour's end with no gate picked -> AttributeError on None.mid (the sim)."""
    from optiprimex.sim.fast_sil import FastSil
    s = FastSil(str(ROOT / "profiles" / "bench" / "world_pool.yaml"), "T1", {"lucky": True}, cfg=load_cfg(COURSE))
    s.sense = lambda: None
    t = s.task
    s.map.ents = {}
    t.m.state, t.current = "TRANSIT", None
    a = t.behaviour_args("TRANSIT", s.ctx())
    assert a["speed_mps"] == 0.0 and t.current is None
    t.on_behaviour_done("TRANSIT", s.ctx())
    assert t.current is None and not t.passed


def _lucky_sil(**args):
    from optiprimex.sim.fast_sil import FastSil
    s = FastSil(str(ROOT / "profiles" / "bench" / "world_pool.yaml"), "T1", dict({"lucky": True}, **args), cfg=load_cfg(COURSE))
    s.sense = lambda: None
    return s


def test_lucky_moves_faster_than_normal_but_circles_and_gate_lines_stay():
    from optiprimex.sim.fast_sil import FastSil
    n = FastSil(str(ROOT / "profiles" / "bench" / "world_pool.yaml"), "T1", {}, cfg=load_cfg(COURSE)).task.p
    l = _lucky_sil().task.p
    for k in ("transit_v", "approach_v", "start_v"):
        assert l[k] == pytest.approx(n[k] * 1.25, abs=1e-3)
    assert l["circle_v"] == n["circle_v"] and l["through_v"] == n["through_v"]


def test_lucky_with_no_entry_orbits_the_known_exit_then_runs_the_gates():
    s = _lucky_sil()
    t = s.task
    x = _e(4, 30, 0, rmin=5)
    x.colour, x.light = "blue", "steady"
    s.map.ents = {4: x}
    t.exit_seen.add(4)
    t.m.state = "SEARCH_ENTRY"
    assert t._lucky_go(s.ctx(), "test")
    assert t.m.state == "CIRCLE_ENTRY_CW" and t.entry_id == 4 and t.lucky_unsure
    t.watch_locked(s.ctx())                                  # a steady stand-in is never "not the entry"
    assert t.m.state == "CIRCLE_ENTRY_CW"
    t.on_behaviour_done("CIRCLE_ENTRY_CW", s.ctx())
    assert t.m.state == "TRANSIT" and t.entry_id is None and t.lucky_from_exit
    assert t.marker(s.ctx(), "exit").id == 4                  # still the exit, circled CCW at the end


def test_lucky_reruns_from_a_late_decided_entry_once():
    s = _lucky_sil()
    t = s.task
    e = _e(9, 5, 3, rmin=5)
    e.colour, e.light, e.led = "blue", "flash", {"pattern": "flash"}
    s.map.ents = {9: e}
    t.m.state, t.lucky_unsure = "TRANSIT", True
    t.passed = [{"mid": (10.0, 0.0), "dir": (1.0, 0.0), "key": ("a", "b"), "correct": True}]
    assert t._lucky_retry(s.ctx())
    assert t.m.state == "CIRCLE_ENTRY_CW" and t.entry_id == 9 and t.passed == [] and t.lucky_retried
    t.m.state, t.lucky_unsure = "TRANSIT", True
    assert not t._lucky_retry(s.ctx())                        # once
    t2 = _lucky_sil(lucky=False).task
    t2.m.state, t2.lucky_unsure = "TRANSIT", True
    assert not t2._lucky_retry(s.ctx())                       # lucky only


def test_orbit_speeds_up_only_while_pushed_back_by_a_current():
    """v0.1.94.2: the drift boost - circle_v 0.12, up to circle_v_max when the pose moves slower than 70 % of the ask."""
    from types import SimpleNamespace as NS
    from optiprimex.usv.behaviours.orbit import Orbit
    from optiprimex.companion.config import Config
    org = NS(from_local=lambda x, y: (x, y))
    def run(gnd):
        o = Orbit()
        ctx = NS(pose=NS(x_n=3.0, y_e=0.0, hdg_deg=90.0), origin=org, now=0.0, cfg=Config())
        o.start({"x_n": 0.0, "y_e": 0.0, "r_m": 3.0, "speed_mps": 0.12, "speed_max": 0.15}, ctx)
        a, v = 0.0, None
        for k in range(200):                       # 20 s at 10 Hz, moving gnd m/s round the ring
            ctx.now = k * 0.1
            a += gnd * 0.1 / 3.0
            ctx.pose = NS(x_n=3.0 * math.cos(a), y_e=3.0 * math.sin(a), hdg_deg=0.0)
            v = o.step(ctx).cmd.speed_mps
        return v
    assert run(0.12) == pytest.approx(0.12, abs=1e-3)       # keeps up: no boost
    assert run(0.04) == pytest.approx(0.15, abs=1e-3)       # pushed back: up to the top, never past it
