"""ROADMAP WP9a (v0.1.31), the software half: the map's ghost filter - `perception.min_obs` (entities seen fewer times stay internal:
not in the digest, the map, gates, markers, the dock line or a task's queries) and `perception.forget_s` (such a ghost is dropped when
not seen again). The pool 2026-11-04: ghosts with 1-3 sightings beside two tracks of 280 / 270. The ZED colour tuning is the lab day."""
from __future__ import annotations

import os

import pytest

from optiprimex.companion.config import Config, from_dict, load
from optiprimex.usv.perception_map import PerceptionMap

from .test_perception_map import obs_of, pose

HERE = os.path.dirname(__file__)


def cfg(min_obs=3, forget_s=5.0):
    c = Config()
    c.map.min_obs, c.map.forget_s = min_obs, forget_s
    return c


def test_young_entities_stay_out_of_everything():
    t = [0.0]
    m = PerceptionMap(cfg(), clock=lambda: t[0])
    real = [(12.0, -2.0, "green"), (12.0, 2.0, "red")]
    for k in range(3):                                             # the gate: three sightings each
        m.ingest(obs_of(real, pose(t=k)), pose(t=k), now=float(k))
    m.ingest(obs_of([(20.0, 9.0, "red")], pose()), pose(), now=2.5)  # a ghost: one sighting
    t[0] = 3.0
    assert len(m.ents) == 3 and len(m.entities("buoy")) == 2 and len(m.entities("buoy", include_young=True)) == 3
    d = m.digest(pose(), now=3.0)
    assert len(d["entities"]) == 2 and d["n_young"] == 1 and d["min_obs"] == 3
    assert len(m.gates(pose())) == 1 and len(d["gates"]) == 1
    # the ghost seen twice more is a real entity from then on (association still finds young ones: no new ghosts)
    m.ingest(obs_of([(20.1, 9.0, "red")], pose()), pose(), now=3.1)
    m.ingest(obs_of([(20.0, 9.1, "red")], pose()), pose(), now=3.2)
    assert len(m.ents) == 3 and len(m.digest(pose(), now=3.3)["entities"]) == 3


def test_ghosts_are_forgotten_tracks_are_not():
    t = [0.0]
    m = PerceptionMap(cfg(), clock=lambda: t[0])
    for k in range(4):
        m.ingest(obs_of([(12.0, 2.0, "red")], pose()), pose(), now=float(k))
    m.ingest(obs_of([(25.0, -6.0, "green")], pose()), pose(), now=3.0)
    t[0] = 8.5                                                     # 5.5 s since either was seen
    m.age_out()
    assert [e.colour for e in m.ents.values()] == ["red"]          # the ghost dropped, the track kept (stale only after 30 s)
    m2 = PerceptionMap(cfg(forget_s=0.0), clock=lambda: 8.5)       # forget_s 0: only the normal ageing
    m2.ingest(obs_of([(25.0, -6.0, "green")], pose()), pose(), now=3.0)
    m2.age_out()
    assert len(m2.ents) == 1
    m3 = PerceptionMap(Config(), clock=lambda: 0.0)                # the code default (Config()): min_obs 1, as before
    m3.ingest(obs_of([(25.0, -6.0, "green")], pose()), pose(), now=0.0)
    assert len(m3.digest(pose(), now=0.0)["entities"]) == 1


def test_profiles_and_validation():
    for name in ("pool/usv_pool.yaml", "pool/usv_lab.yaml", "bench/usv_bench.yaml", "course/usv_course.yaml"):
        c = load(os.path.join(HERE, "..", "profiles", name))
        assert (c.map.min_obs, c.map.forget_s) == (3, 5.0), name
    assert from_dict({"perception": {"min_obs": 5}}).map.min_obs == 5
    with pytest.raises(ValueError):
        from_dict({"perception": {"min_obs": 0}})
    html = open(os.path.join(HERE, "..", "optiprimex", "tools", "map.html")).read()
    assert "#f2c94c" in html and "e.colour === 'blue'" in html                  # blue / yellow glyphs and table rows
    ui = open(os.path.join(HERE, "..", "usv_ui", "usv_ui.py")).read()
    assert "young (< %d obs, not in the map)" in ui
