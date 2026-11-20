"""PerceptionMap: geolocation at the capture pose, association, EMA, colour vote, gates, dock line, age-out."""
from __future__ import annotations

import math

from optiprimex.companion.config import Config
from optiprimex.companion.state import LocalPose, Observation, ObsItem
from optiprimex.usv.perception_map import PerceptionMap


def pose(x=0.0, y=0.0, hdg=0.0, t=0.0):
    return LocalPose(x_n=x, y_e=y, hdg_deg=hdg, co_lat=1.3, co_lon=103.8, t=t)


def obs_of(objs, p, kind="buoys", t_cam=0.0, conf=0.9):
    items = []
    for i, (x, y, colour) in enumerate(objs):
        dx, dy = x - p.x_n, y - p.y_e
        rng = math.hypot(dx, dy)
        brg = (math.degrees(math.atan2(dy, dx)) - p.hdg_deg + 180) % 360 - 180
        items.append(ObsItem(id=i, cls="buoy" if kind == "buoys" else "dock", colour=colour, conf=conf, range_m=rng, bearing_deg=brg))
    return Observation(t_cam=t_cam, kind=kind, items=items)


def test_geolocation_association_and_ema():
    cfg = Config()
    m = PerceptionMap(cfg, clock=lambda: 0.0)
    world = [(12.0, -2.0, "green"), (12.0, 2.0, "red")]
    # seen from three poses, including a rotated one: still two entities, at the world positions
    for k, p in enumerate((pose(0, 0, 0), pose(2, 0, 10), pose(4, 1, -15))):
        used = m.ingest(obs_of(world, p), p, now=float(k))
        assert used == 2
    ents = m.entities("buoy")
    assert len(ents) == 2 and all(e.n_obs == 3 for e in ents)
    for e in ents:
        truth = world[0] if e.colour == "green" else world[1]
        assert math.hypot(e.x_n - truth[0], e.y_e - truth[1]) < 0.05 and e.sigma_m < 0.5
    # a low-confidence item is ignored; a noisy repeat inside the gate updates, outside it creates
    assert m.ingest(obs_of([(12.0, 2.3, "red")], pose(), conf=0.1), pose(), now=3.0) == 0
    assert m.ingest(obs_of([(12.0, 2.3, "red")], pose()), pose(), now=3.0) == 1 and len(m.entities("buoy")) == 2
    assert m.ingest(obs_of([(30.0, 0.0, "red")], pose()), pose(), now=3.0) == 1 and len(m.entities("buoy")) == 3
    # colour by majority vote
    e = next(e for e in m.entities("buoy") if abs(e.x_n - 12) < 0.5 and e.y_e > 0)
    for n in range(5):
        m.ingest(obs_of([(12.0, 2.0, "green")], pose()), pose(), now=4.0 + n)
    assert e.colour_votes == {"red": 4, "green": 5} and e.colour == "green"      # majority vote, 4 red vs 5 green
    m.ingest(obs_of([(12.0, 2.0, "red")], pose()), pose(), now=9.0)
    m.ingest(obs_of([(12.0, 2.0, "red")], pose()), pose(), now=9.5)
    assert e.colour == "red"


def test_gates_and_through_heading():
    cfg = Config()
    m = PerceptionMap(cfg, clock=lambda: 0.0)
    world = [(12.0, -2.0, "green"), (12.0, 2.0, "red"), (24.0, -2.0, "green"), (24.0, 2.0, "red"), (18.0, 9.0, "white"),
             (40.0, -20.0, "red")]
    p = pose(0, 0, 0)
    m.ingest(obs_of(world, p), p, now=0.0)
    gates = m.gates(p)
    assert len(gates) == 2
    assert abs(gates[0].mid_n - 12.0) < 0.1 and abs(gates[1].mid_n - 24.0) < 0.1       # ordered by distance
    assert abs(gates[0].width_m - 4.0) < 0.05
    assert abs(gates[0].through_hdg_deg - 0.0) < 1.0                                   # north, red to starboard (east)
    # from beyond the gate the through heading flips to point away from the boat
    assert abs(m.gates(pose(30, 0, 180))[1].through_hdg_deg - 180.0) < 1.0


def test_dock_line_and_age_out():
    cfg = Config()
    m = PerceptionMap(cfg, clock=lambda: 0.0)
    faces = [(50.0, 6.0, "white"), (50.0, 10.0, "white"), (50.0, 14.0, "white")]
    p = pose(40, 10, 0)
    m.ingest(obs_of(faces, p, kind="dock"), p, now=0.0)
    d = m.dock(p, now=0.0)
    assert d and abs(d["x_n"] - 50.0) < 0.05 and abs(d["y_e"] - 10.0) < 0.05 and abs(d["length_m"] - 8.0) < 0.1
    assert abs(((d["psi_n"] - 180.0) + 180) % 360 - 180) < 1.0          # normal toward the boat (south)
    assert abs(d["line_hdg_deg"] - 90.0) < 1.0 or abs(d["line_hdg_deg"] - 270.0) < 1.0
    assert d["rms_m"] < 0.05
    # ageing: stale after max_age_s, dropped after max_age_s + drop_age_s; stale faces leave the dock fit
    m.age_out(now=cfg.map.max_age_s + 1)
    assert all(e.state == "stale" for e in m.entities("dock", include_stale=True)) and m.entities("dock") == []
    assert m.dock(p, now=cfg.map.max_age_s + 1) is None
    dg = m.digest(p, now=cfg.map.max_age_s + 1)
    assert len(dg["entities"]) == 3 and dg["entities"][0]["state"] == "stale" and dg["gates"] == []
    m.age_out(now=cfg.map.max_age_s + cfg.map.drop_age_s + 1)
    assert m.entities("dock", include_stale=True) == []
