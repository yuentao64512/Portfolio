"""ROADMAP WP10h (v0.1.39), the mission profile campaign and what it changed: the pending buoy (the map) and m1t1's LOOK at it, the
search driving toward rays (APPROACH_LEAD), the ScanYaw face mode, the dock stop 1.5 m (the windows in the 51 deg view), the fast SIL's
camera path, the campaign's grading (the on-the-line crossing) and its runner."""
from __future__ import annotations

import math

import pytest

from optiprimex.companion.config import Config, load
from optiprimex.companion.state import LocalPose, MapEntity, VelocityYawTarget
from optiprimex.sim import campaign as C
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil
from optiprimex.usv.behaviours.base import BehaviourContext
from optiprimex.usv.behaviours.scan_yaw import ScanYaw
from optiprimex.usv.perception_map import classify_light

from .conftest import ROOT

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
MARINA = str(ROOT / "profiles" / "bench" / "world_marina.yaml")


def test_a_rarely_lit_buoy_is_pending_until_the_evidence_is_in():
    assert classify_light({"red": 2, "black": 7}) == ("unknown", "?")           # 9 % of a flashing light's 9-sample windows
    assert classify_light({"red": 4, "black": 14}) == ("black", "off")          # 18 sightings: decided, an unlit obstacle
    assert classify_light({"red": 1, "black": 8}) == ("black", "off")           # one stray frame is noise
    assert classify_light({"red": 5, "black": 4}) == ("red", "flash")
    assert classify_light({"blue": 6, "green": 4, "red": 1, "black": 16})[0] == "black"   # v0.1.34's far unlit obstacle stays black


def _ctx(x, y, hdg, now=10.0):
    return BehaviourContext(cfg=Config(), now=now, fcu=None, pose=LocalPose(x, y, hdg, 0.0, 0.0, now), origin=None, run=None)


def test_scan_yaw_faces_a_point_and_gives_up_in_time():
    b = ScanYaw()
    b.start({"until": lambda c: False, "face": (0.0, 10.0), "max_s": 5.0, "rate_dps": 20.0, "reason": "undecided"}, _ctx(0, 0, 0.0))
    out = b.step(_ctx(0, 0, 0.0, 11.0))
    assert isinstance(out.cmd, VelocityYawTarget) and out.cmd.vx_mps == 0.0 and out.cmd.yaw_rate_rad_s == pytest.approx(math.radians(20.0))
    out = b.step(_ctx(0, 0, 85.0, 12.0))
    assert 0 < out.cmd.yaw_rate_rad_s < math.radians(20.0) and out.info["face_err_deg"] == 5     # slows as it lines up
    assert b.step(_ctx(0, 0, 90.0, 16.0)).abort == "undecided"


def _ent(i, x, y, colour, n=12, light="flash"):
    return MapEntity(i, "buoy", colour, x, y, 0.2, n, 0.0, 100.0, light=light, colour_votes={colour: n})


def test_m1t1_looks_at_a_pending_buoy_before_a_gate_further_on():
    s = FastSil(POOL, "T1", dict(C.T1_FAST), start={"x_n": 8.0, "y_e": -6.0, "hdg_deg": 0.0})
    s.sense = lambda: None                                            # the map is set by hand below
    t = s.task
    t.m.state, t.entry_xy, t.last_dir = "TRANSIT", (5.0, -7.0), (1.0, 0.0)
    s.map.ents = {1: _ent(1, 12.0, -7.2, "green"), 2: _ent(2, 12.0, -4.8, "unknown", light="?"),     # gate 1: its red still pending
                  3: _ent(3, 19.0, -5.7, "green"), 4: _ent(4, 19.0, -3.3, "red")}                    # gate 2: usable
    t._next_from_transit(s.ctx())
    assert t.m.state == "LOOK" and t.look_id == 2 and any("T1 look at #2" in l for l in s.logs)
    s.map.ents[2] = _ent(2, 12.0, -4.8, "red")                        # the light seen: decided
    assert t._look_done(s.ctx())
    t._looked(s.ctx(), "decided")
    assert t.m.state == "TRANSIT" and 2 in t.skipped
    t._next_from_transit(s.ctx())
    assert t.current is not None and t.current.key == (2, 1)          # gate 1 now, not gate 2
    t2 = FastSil(POOL, "T1", dict(C.T1_FAST)).task                    # behind the travel direction: not looked at
    t2.start(dict(C.T1_FAST), s.ctx())
    t2.last_dir = (1.0, 0.0)
    s.map.ents[2] = _ent(2, 4.0, -4.8, "unknown", light="?")
    assert t2.pending_lead(s.ctx(), None) is None


def test_m1t1_drives_toward_a_ray_from_far():
    wd = W.load(MARINA)
    r = FastSil(MARINA, "T1", dict(C.T1_FAST), start=C.far_start(wd, 1), seed=1).run(600)
    g = C.grade_m1t1(wd["objects"], r)
    assert r["state"] == "DONE" and g["ok"], (g, r["logs"][-5:])
    assert r["states"][:3] == ["SEARCH_ENTRY", "APPROACH_LEAD", "SEARCH_ENTRY"]
    assert any("toward a far sighting" in l for l in r["logs"])


def test_the_dock_stop_keeps_the_windows_in_view():
    for name in ("pool/usv_pool.yaml", "pool/usv_lab.yaml", "course/usv_course.yaml", "bench/usv_bench.yaml"):
        assert load(str(ROOT / "profiles" / name)).extra["t3"]["stop_m"] == 1.5, name
    from optiprimex.usv.tasks.t3_logistics import DEFAULTS
    from optiprimex.sim import zed
    assert DEFAULTS["stop_m"] == 1.5 and math.degrees(math.atan2(0.6, 1.5)) < zed.hfov_deg("hd2k") / 2 - 3.0
    objs = W.load(POOL)["objects"]
    for seed in (1, 2, 4):                                            # the seeds that fired 30 s at nothing at 1.0 m
        r = FastSil(POOL, "T3", {}, start=C.jitter(C.NEAR_DOCK["pool"], seed, 1.5, 15.0), seed=seed).run(400)
        assert C.grade_m3t1(objs, r)["ok"], (seed, r["events"][-2:])


def test_the_grader_counts_a_sample_on_the_gate_line():
    objs = [{"kind": "buoy", "gate": 1, "colour": "red", "x_n": 12.0, "y_e": -4.8}, {"kind": "buoy", "gate": 1, "colour": "green", "x_n": 12.0, "y_e": -7.2}]
    track = [(0, 11.4, -6.0, 0, "T"), (1, 12.0, -6.0, 0, "T"), (2, 12.6, -6.0, 0, "T")]
    assert C.crossings(objs, track) == {1: True}
    assert C.crossings(objs, [(0, 12.6, -6.0, 0, "T"), (1, 12.0, -6.0, 0, "T"), (2, 11.4, -6.0, 0, "T")]) == {1: False}
    assert C.crossings(objs, [(0, 11.0, -3.0, 0, "T"), (1, 13.0, -3.0, 0, "T")]) == {1: None}


def test_the_fast_sil_camera_path_maps_the_buoys():
    s = FastSil(POOL, "T1", dict(C.T1_FAST), perception="camera", cam_size=(552, 310), start={"x_n": 8.0, "y_e": -6.0, "hdg_deg": 0.0})
    s.warmup(4.0)
    assert s.cam.n >= 8 and s.cam.last["observer"] == "sim_observer"
    truth = {(o["x_n"], o["y_e"]): o for o in W.load(POOL)["objects"] if o["kind"] == "buoy"}
    near = [e for e in s.map.entities("buoy", include_young=True) if math.hypot(e.x_n - 8.0, e.y_e + 6.0) < 20.0]
    assert len(near) >= 5
    for e in near:
        d, o = min((math.hypot(e.x_n - x, e.y_e - y), o) for (x, y), o in truth.items())
        assert d < 0.4, (e, o)
    assert {o.get("kind") for o in s.fp.objects} >= {"dock"} and "buoy" not in {o.get("kind") for o in s.fp.objects}
    with pytest.raises(ValueError):
        FastSil(POOL, "T1", {}, perception="lidar")


def test_the_campaign_runs_and_writes_its_table(tmp_path):
    cs = C.cases(["m1t1", "m4t0"], 1, ["pool", "marina"], ["simple"], ["geometric"], ["near", "far"])
    assert [(c["mission"], c["world"], c.get("start")) for c in cs] == [("m1t1", "pool", "near"), ("m1t1", "marina", "near"),
                                                                          ("m1t1", "marina", "far"), ("m4t0", "pool", None)]
    res = [C.run_case(c) for c in cs if c["mission"] == "m4t0" or c["world"] == "pool"]
    assert all(r["ok"] for r in res), [r["why"] for r in res]
    doc = tmp_path / "c.md"
    doc.write_text("# x\n\n<!-- CAMPAIGN BEGIN -->\nold\n<!-- CAMPAIGN END -->\ntail\n")
    C.write_doc(str(doc), C.table(res), "2/2 passed")
    txt = doc.read_text()
    assert "old" not in txt and "| m4t0 | pool | - | geometric | near | 1/1 |" in txt and txt.endswith("<!-- CAMPAIGN END -->\ntail\n")
    assert "240/240 passed" in (ROOT / "docs" / "CAMPAIGN.md").read_text()
