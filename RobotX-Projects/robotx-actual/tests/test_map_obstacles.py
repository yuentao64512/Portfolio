"""v0.1.81.1: obstacles on the USV map + the recorded map (Dmitrii 2026-11-11: "if we see something, we put a 2-3 m circle behind that
area" -> the ZED depth 3-8 m + the guard's STOPs, a 2 m ring centred 1 m behind the seen face; "after the run, can we save the map we
recorded? later we might want to replay that" -> on the boat and on the Mac, a timeline + snapshots). QUICK: no broker."""
from __future__ import annotations

import json
import math
import sys

import numpy as np
import yaml

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin
from optiprimex.companion.state import GuidedTarget, LocalPose, MapEntity
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil
from optiprimex.usv.gatepath.planner import map_obstacles
from optiprimex.usv.guard import GuardSupervisor
from optiprimex.usv.map_rec import MapRecorder, snapshot
from optiprimex.usv.perception_map import PerceptionMap

from .conftest import ROOT
from .test_guard import STOP, _ctx, _Task

sys.path.insert(0, str(ROOT / "cv" / "buoys_multi"))
import guard_core as GC  # noqa: E402

WORLD = str(ROOT / "profiles" / "bench" / "world_scout.yaml")
POLY = [tuple(p) for p in W.load(WORLD)["bounds"]]
O = ControlOrigin(1.28, 103.85)


def face(dist, x0=0.0, half=0.2, h=(0.2, 0.6), h_cam=1.0, n=30):
    xs, hs = np.meshgrid(np.linspace(x0 - half, x0 + half, n), np.linspace(h[0], h[1], n))
    return np.stack([xs, h_cam - hs, np.full_like(xs, dist)], axis=2).astype(np.float32)


def pose(x=0.0, y=0.0, hdg=0.0):
    return LocalPose(x, y, hdg, O.lat, O.lon, 0.0)


# ---- the AGX side: guard_core's `far` -------------------------------------------------------------------------------------------------
def test_far_clusters_between_slow_and_map_range():
    g = GC.GuardFilter({"step": 1})
    m = g.frame(face(6.0))
    assert m["clear"] is True and len(m["far"]) == 1 and abs(m["far"][0]["range_m"] - 6.0) < 0.1      # 3-8 m: mapped, never a STOP
    two = np.concatenate([face(5.0, x0=-2.0), face(7.0, x0=2.5)], axis=0)
    far = GC.GuardFilter({"step": 1}).frame(two)["far"]
    assert [round(c["range_m"]) for c in far] == [5, 7] and far[0]["bearing_deg"] < 0 < far[1]["bearing_deg"]   # two things, nearest first
    assert "far" not in GC.GuardFilter({"step": 1}).frame(face(10.0))                                  # beyond map_m 8
    assert "far" not in GC.GuardFilter({"step": 1}).frame(face(2.0))                                   # inside slow_m: the STOP path's
    assert "far" not in GC.GuardFilter({"step": 1}).frame(face(6.0, half=0.02, n=6))                   # 36 px < map_min_px: a speck
    assert "far" not in GC.GuardFilter({"step": 1}).frame(face(6.0, h=(0.0, 0.1)))                     # the water's glitter (< 0.15 m)
    assert "far" not in GC.GuardFilter({"step": 1, "map_m": 0}).frame(face(6.0))                       # --guard-map 0 = off
    obs = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert '"--guard-map", type=float, default=8.0' in obs and '"map_m": a.guard_map' in obs


# ---- the boat: map_far -> depth obstacles --------------------------------------------------------------------------------------------
FAR = {"clear": True, "far": [{"range_m": 6.0, "bearing_deg": 0.0, "bearing_min_deg": -2.0, "bearing_max_deg": 2.0, "px": 900}]}


def test_map_far_puts_a_2m_ring_1m_behind_the_face_after_three_sightings():
    pm, g = PerceptionMap(Config()), GuardSupervisor()
    assert g.p["ring_m"] == 2.0 and g.p["ring_behind_m"] == 1.0
    assert g.map_far(FAR, pose(), pm, 10.0) == 1
    assert g.map_far(FAR, pose(), pm, 10.2) == 0                                  # map_hz 2: one pass a 0.5 s
    e = pm.entities("obstacle", include_young=True)[0]
    assert e.source == "depth" and abs(e.x_n - 7.0) < 1e-6 and abs(e.y_e) < 1e-6 and e.ring_m == 2.0
    assert not pm.obstacle_counts(e) and not map_obstacles(pm) and not [b for b in pm.digest(pose(), 10.2)["entities"] if b["kind"] == "obstacle"]
    g.map_far(FAR, pose(), pm, 10.6)
    g.map_far(FAR, pose(0.2), pm, 11.1)                                          # 0.2 m nearer: the same obstacle (merge 1 m)
    assert len(pm.entities("obstacle", include_young=True)) == 1 and e.n_obs == 3 and pm.obstacle_counts(e)
    assert map_obstacles(pm) == [(e.x_n, e.y_e, 2.0)]
    b = [b for b in pm.digest(pose(), 11.1)["entities"] if b["kind"] == "obstacle"][0]
    assert b["ring_m"] == 2.0 and b["source"] == "depth"


def test_a_mapped_buoy_explains_the_depth_cluster_and_a_late_one_removes_it():
    pm, g = PerceptionMap(Config()), GuardSupervisor()
    pm.ents[1] = MapEntity(id=1, kind="buoy", colour="red", x_n=6.2, y_e=0.0, sigma_m=0.3, n_obs=5, t_first=0.0, t_last=10.0)
    pm._next_id = 2
    for t in (10.0, 10.6, 11.2):
        g.map_far(FAR, pose(), pm, t)
    assert not pm.entities("obstacle", include_young=True)                       # the buoy is the thing seen: no ring round it
    pm2, g2 = PerceptionMap(Config()), GuardSupervisor()
    for t in (10.0, 10.6, 11.2):
        g2.map_far(FAR, pose(), pm2, t)
    assert pm2.entities("obstacle", include_young=True)
    pm2.ents[99] = MapEntity(id=99, kind="buoy", colour="green", x_n=6.25, y_e=0.1, sigma_m=0.3, n_obs=0, t_first=11.3, t_last=11.3)
    pm2.age_out(11.4)
    assert not pm2.entities("obstacle", include_young=True)                      # the detector found the buoy later: the ring goes


def test_a_wide_cluster_gets_rings_along_its_arc_and_off_maps_nothing():
    pm, g = PerceptionMap(Config()), GuardSupervisor()
    wide = {"far": [{"range_m": 5.0, "bearing_deg": 0.0, "bearing_min_deg": -30.0, "bearing_max_deg": 30.0, "px": 4000}]}   # a pontoon
    n = g.map_far(wide, pose(), pm, 10.0)
    assert 3 <= n <= 7 and len(pm.entities("obstacle", include_young=True)) == n
    pm3, g3 = PerceptionMap(Config()), GuardSupervisor({"map": False})
    assert g3.map_far(FAR, pose(), pm3, 10.0) == 0 and not pm3.ents


def test_a_guard_stop_ring_is_2m_centred_1m_behind_and_claims_a_depth_one():
    pm = PerceptionMap(Config())
    g = GuardSupervisor()
    g.on_message(STOP, 10.0, pm)
    g.filter(_Task(), GuidedTarget(1.29, 103.85, 0.6), _ctx(10.0, pm=pm))
    rings = pm.entities("obstacle", include_young=True)
    near = min(rings, key=lambda e: math.hypot(e.x_n, e.y_e))
    d = math.hypot(near.x_n, near.y_e)
    assert abs(d - (1.3 + 1.0)) < 0.05 and near.ring_m == 2.0 and near.source == "guard"
    pm2 = PerceptionMap(Config())
    pm2.add_obstacle(5.0, 0.0, 2.0, 1.0, source="depth", face=(4.0, 0.0))
    e = pm2.add_obstacle(5.2, 0.0, 2.0, 2.0)                                     # the guard stopped for it: kept as a guard obstacle
    assert e.source == "guard" and pm2.obstacle_counts(e) and len(pm2.ents) == 1


def test_the_profiles_carry_the_ring_and_the_switch():
    for f in ("pool/usv_pool.yaml", "course/usv_course.yaml", "bench/usv_bench.yaml"):
        z = yaml.safe_load((ROOT / "profiles" / f).read_text())["zed_guard"]
        assert z["ring_m"] == 2.0 and z["ring_behind_m"] == 1.0 and z["map"] is True and z["depth_min_obs"] == 3, f


# ---- the fast SIL: S15 with the depth map ---------------------------------------------------------------------------------------------
def test_scout_s15_maps_the_hidden_obstacles_early_and_never_stops(tmp_path):
    s = FastSil(WORLD, "SCOUT", {"polygon": POLY, "log_dir": str(tmp_path)}, guard={})
    r = s.run(max_s=900)
    assert r["state"] == "DONE" and not r["safe_holds"]
    assert not [l for l in s.task.lines if l["ev"] == "obstacle"]                # mapped from 3-8 m: the planner kept off, no STOP
    obst = [e for e in s.map.ents.values() if e.kind == "obstacle" and s.map.obstacle_counts(e)]
    assert obst and all(e.source == "depth" for e in obst)
    assert any(math.hypot(e.x_n - 11.0, e.y_e - 2.6) < 1.5 for e in obst)        # the pole
    assert any(abs(e.x_n - 23.5) < 1.5 and -2.5 < e.y_e < 1.0 for e in obst)     # the boom
    world_buoys = [o for o in W.load(WORLD)["objects"] if o["kind"] == "buoy"]
    assert all(min(math.hypot(e.x_n - b["x_n"], e.y_e - b["y_e"]) for b in world_buoys) > 1.0 for e in obst)   # no ring round a buoy
    buoys = [l for l in s.task.lines if l["ev"] == "buoy"]
    assert len(buoys) == 10 and sum(b["visited"] for b in buoys) >= 9
    assert min(math.hypot(p[1] - 11.0, p[2] - 2.6) for p in s.track) > 1.0


# ---- the recorded map ------------------------------------------------------------------------------------------------------------------
def _body(task, x, ents=None):
    return {"co": {"lat": O.lat, "lon": O.lon}, "boat": {"x_n": x, "y_e": 0.0, "hdg_deg": 0.0, "trail": []}, "task": task,
            "entities": ents or [], "rays": [{"id": 1, "colour": "red", "x_n": 20.0, "y_e": 0.0, "az_deg": 0.0, "n": 1, "age_s": 0.0}],
            "gates": []}


def test_the_recorder_writes_the_timeline_latest_and_a_snapshot_per_task(tmp_path):
    clock = [1000.0]
    rec = MapRecorder(str(tmp_path), "boat", "usv", clock=lambda: clock[0], version="0.1.81.1")
    ob = {"id": 7, "kind": "obstacle", "colour": "grey", "x_n": 10.0, "y_e": 5.0, "sigma_m": 2.0, "ring_m": 2.0, "source": "depth", "n_obs": 3}
    for i in range(6):                                                           # 3 s at 2 Hz: 3 frames (1 Hz)
        clock[0] = 1000.0 + 0.5 * i
        rec.feed(_body("SCOUT", 0.5 * i, [ob]))
    clock[0] = 1003.2
    rec.feed(_body(None, 3.0, [ob]))                                             # the task ended -> its snapshot
    rec.feed(_body("idle", 3.0, [ob]), 1003.3)
    rec.feed(_body(None, 3.0, [ob]), 1003.4)                                   # "idle" ending: no snapshot
    rec.feed(_body("idle", 3.0, [ob]), 1003.3)
    rec.feed(_body(None, 3.0, [ob]), 1003.4)                                   # "idle" ending: no snapshot
    rec.close()
    lines = [json.loads(x) for x in open(rec.path("map.jsonl"))]
    assert lines[0]["k"] == "header" and lines[0]["format"] == "usv_map_rec/1" and lines[0]["co"]["lat"] == O.lat
    assert [x["k"] for x in lines].count("frame") == 4 and any(x["k"] == "saved" and x["task"] == "SCOUT" for x in lines)
    names = [p.rsplit("/", 1)[-1] for p in rec.written]
    assert names[0].endswith(".json") and "_boat_SCOUT_" in names[0] and "_boat_end_" in names[-1] and len(names) == 2 and len(names) == 2
    snap = json.load(open(rec.written[0]))
    assert snap["format"] == "usv_map/1" and snap["task"] == "SCOUT" and snap["counts"] == {"obstacle": 1}
    lat, lon = O.from_local(10.0, 5.0)
    e = snap["entities"][0]
    assert abs(e["lat"] - lat) < 1e-7 and abs(e["lon"] - lon) < 1e-7 and e["ring_m"] == 2.0          # survives another run's origin
    assert snap["rays"][0]["lat"] and len(snap["track"]) >= 3 and snap["boat"]["lat"]
    latest = json.load(open(rec.path("latest.json")))
    assert latest["format"] == "usv_map/1"
    assert snapshot(_body("X", 0.0), "ocs", "usv", "X", [])["entities"] == []


def test_the_wiring_boat_and_mac(monkeypatch):
    mm = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text()
    assert '"--map-rec"' in mm and 'MapRecorder(a.map_rec, "boat"' in mm and "self.map_pub.recorder.close()" in mm
    assert 'self._obs_q.put(("guard/far", body, self.clock()))' in mm and "self.guard.map_far(" in mm
    assert "self.recorder.feed(body)" in (ROOT / "optiprimex" / "usv" / "publishers.py").read_text()
    for sh in ("run_usv.sh", "run_bench.sh"):
        assert "--map-rec $RUN_DIR/maps" in (ROOT / "scripts" / sh).read_text(), sh
    from optiprimex.tools import ocs_web
    monkeypatch.delenv("OCS_MAP_REC", raising=False)                             # the suite sets it off (conftest)
    a = ocs_web.parse_args([])
    assert a.map_rec.endswith("logs/ocs/maps") and ocs_web.parse_args(["--map-rec", "off"]).map_rec == "off"
    assert ocs_web.Web(a, feeds=False).map_rec is None                           # no feeds (the tests): nothing recorded
    page = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert "if (e.kind === 'obstacle')" in page and "e.ring_m" in page and "obstacle (depth)" in page
