"""v0.1.46.1 map fixes: the fake judge's course from the world file (pool = its bounds, marina = Course Alpha, else Course Alpha as
before); the boat in the theme's text colour as a hull pentagon + heading line + track, a hollow accent carrot; the label placer
(bay vs face id, A / B, CO / HOME); entry / exit rings dashed = flashing, full = solid; the canvas follows the theme."""
from __future__ import annotations

import argparse

from optiprimex.sim import marina
from optiprimex.sim.fake_robocommand import FakeRoboCommand, world_course

from .conftest import ROOT

MAP = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
BENCH = ROOT / "profiles" / "bench"


def test_the_fake_judges_course_comes_from_the_world():
    b, t, cid = world_course(str(BENCH / "world_pool.yaml"))
    assert cid == "SIM-POOL" and len(b) == 4 and t == []
    lat = sorted(p[0] for p in b)
    assert lat[-1] - lat[0] > 0.0004                                             # the pool's 50 m north-south
    b, t, cid = world_course(str(BENCH / "world_marina.yaml"))
    assert cid == "COURSE-ALPHA" and [tuple(p) for p in b] == list(marina.COURSE_ALPHA) and t
    assert world_course(str(BENCH / "world_boxes.yaml")) == (None, [], None) and world_course(None) == (None, [], None)
    fr = FakeRoboCommand.__new__(FakeRoboCommand)
    fr.a = argparse.Namespace(course_id="BENCH-COURSE-1", world=str(BENCH / "world_pool.yaml"))
    assert fr.course()["course_id"] == "SIM-POOL"
    fr.a.world = None
    c = fr.course()
    assert c["course_id"] == "BENCH-COURSE-1" and [tuple(p) for p in c["boundary"]] == list(marina.COURSE_ALPHA)   # as before
    assert "fake_robocommand --port 1882 --world $WORLD" in (ROOT / "scripts" / "run_bench.sh").read_text()


def test_the_boat_glyph_and_the_carrot():
    i = MAP.index("  // boat + track (v0.1.46.1")
    boat = MAP[i:MAP.index("  // scale bar", i)]
    assert "themeVar('--text'" in boat and "catamaran()" in boat                   # v0.1.52: the pentagon became the BlueBoat (test_v0152_wb)
    assert "const hd = P(BOAT.L / 2 * k" in boat and "b.trail" in boat           # heading line, track
    for col in ("#3fd16b", "#ff5a5a", "#5aa0ff", "#2d7a44"):
        assert col not in boat, col                                                # no green / red / blue on the boat
    assert "ctx.strokeStyle = themeVar('--acc', '#4cc2ff'); ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(px, py, 5, 0, 6.283); ctx.stroke(); }" in MAP


def test_labels_are_placed_and_entry_exit_say_flashing_or_solid():
    assert "function lbl(s, x, y, col)" in MAP and "LBL = []; mapColours();" in MAP
    for call in ("lbl(`bay ${o.bay}`", "lbl('CO'", "lbl(name, px, py", "lbl(`${r.colour} ${r.az_deg.toFixed(0)}°`"):
        assert call in MAP, call
    assert "fl ? 'flashing' : o.light === 'steady' ? 'solid'" in MAP and "ctx.setLineDash(fl ? [3, 3] : [])" in MAP
    assert "e.light === 'flash' ? ' flashing' : e.light === 'steady' ? ' solid'" in MAP
    assert "background:var(--mapBg, #0b0f14)" in MAP and "MC.grid" in MAP and "'#1c232c'; ctx.lineWidth = 1" not in MAP
