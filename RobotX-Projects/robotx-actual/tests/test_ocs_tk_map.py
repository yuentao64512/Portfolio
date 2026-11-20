"""K6: the Canvas map - the projection, the 2 m inset, the layer from the RECORDED /map.json (pool sim 2026-11-11, SCOUT running: 9 camera
buoys, 2 visited, a target) and /course.json, and the Canvas drawing the polygon, the inset, the boat and the buoys."""
from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from optiprimex.tools.tkconsole import canvasmap as C

FIX = Path(__file__).parent / "fixtures"


def load(name):
    return json.loads((FIX / f"state_{name}.json").read_text())


def test_the_frame_is_centred_on_the_polygon_and_round_trips():
    poly = C.ring(load("course")["received"]["boundary"])
    fr = C.frame_for(poly)
    pts = [fr.en(*p) for p in poly]
    cx, cy = C.centroid(pts)
    assert abs(cx) < 0.01 and abs(cy) < 0.01                                   # the centroid is (0, 0)
    lat, lon = fr.ll(*fr.en(1.2812, 103.8551))
    assert abs(lat - 1.2812) < 1e-9 and abs(lon - 103.8551) < 1e-9
    e, n = fr.en(fr.lat0 + 10 / C.M_PER_DEG, fr.lon0)
    assert abs(n - 10) < 1e-6 and abs(e) < 1e-6


def test_the_inset_is_2_m_inside_every_edge():
    sq = [(0, 0), (20, 0), (20, 10), (0, 10)]
    assert [tuple(round(v, 6) for v in p) for p in C.inset(sq)] == [(2, 2), (18, 2), (18, 8), (2, 8)]
    assert [tuple(round(v, 6) for v in p) for p in C.inset(list(reversed(sq)))] == [(2, 8), (18, 8), (18, 2), (2, 2)]   # either winding
    poly = [C.frame_for(C.ring(load("course")["received"]["boundary"])).en(*p) for p in C.ring(load("course")["received"]["boundary"])]
    ins = C.inset(poly)
    for i in range(len(poly)):                                                  # every inset vertex is 2 m from its two edges
        for (a, b) in ((poly[i - 1], poly[i]), (poly[i], poly[(i + 1) % len(poly)])):
            (x0, y0), (x1, y1), (px, py) = a, b, ins[i]
            d = abs((x1 - x0) * (y0 - py) - (x0 - px) * (y1 - y0)) / math.hypot(x1 - x0, y1 - y0)
            assert abs(d - 2.0) < 1e-6


def test_the_layer_from_the_recorded_map():
    mp = load("map")
    m = mp["map"]
    L = C.build_layer(load("course"), mp, (1.28115, 103.85498), 300.0)
    assert L["polygon_src"] == "received" and len(L["polygon"]) == 4 and len(L["inset"]) == 4
    assert len(L["items"]) == 9 and all(i["kind"] == "buoy" for i in L["items"])
    vis = {i["id"] for i in L["items"] if i["visited"]}
    assert vis == set(m["scout"]["visited_ids"]) and L["target"] is not None
    assert {p["name"] for p in L["points"]} >= {"HOME", "A"} and L["boat"][2] == 300.0
    fr = L["frame"]
    co = m["co"]
    home = next(p for p in L["points"] if p["name"] == "HOME")
    assert math.hypot(home["e"] - fr.en(co["lat"], co["lon"])[0], home["n"] - fr.en(co["lat"], co["lon"])[1]) < 0.05
    b1 = next(i for i in L["items"] if i["id"] == 1)                         # local N/E around co -> the frame
    e0, n0 = fr.en(co["lat"], co["lon"])
    src = next(e for e in m["entities"] if e["id"] == 1)
    assert abs((b1["n"] - n0) - src["x_n"]) < 0.01 and abs((b1["e"] - e0) - src["y_e"]) < 0.01


def test_uav_buoys_obstacles_and_no_pose():
    mp = json.loads(json.dumps(load("map")))
    ents = mp["map"]["entities"]
    ents.append({"id": 40, "kind": "buoy", "colour": "unknown", "x_n": 5, "y_e": 5, "sigma_m": 1.2, "source": "uav"})
    ents.append({"id": 41, "kind": "obstacle", "x_n": 8, "y_e": 1, "sigma_m": 0.3, "source": "usv"})
    L = C.build_layer(load("course"), mp, None, None)
    assert any(i["source"] == "uav" for i in L["items"]) and len(L["obstacles"]) == 1 and L["obstacles"][0]["ring"] == 1.5
    assert L["boat"] is not None                                               # no /fast pose: the digest's boat
    assert C.build_layer({}, None, None, None)["frame"] is None                 # nothing at all: nothing drawn, no crash


tk = pytest.importorskip("tkinter")


def test_the_canvas_draws_the_polygon_the_boat_and_the_buoys():
    from .test_ocs_tk_smoke import shared_root
    root = tk.Toplevel(shared_root())
    try:
        root.geometry("800x600")
        from optiprimex.tools.tkconsole.theme import palette
        cm = C.CanvasMap(root, palette("light"))
        cm.c.pack(fill="both", expand=True)
        root.update_idletasks()
        cm.set_layer(C.build_layer(load("course"), load("map"), (1.28115, 103.85498), 300.0))
        cm.fit()
        c = cm.c
        assert c.find_withtag("polygon") and c.find_withtag("inset") and c.find_withtag("boat")
        assert len(c.find_withtag("item")) == 9 and len(c.find_withtag("visited")) == 2 and c.find_withtag("target")
        assert "dashed" not in str(c.itemcget(c.find_withtag("polygon")[0], "dash")) and c.itemcget(c.find_withtag("inset")[0], "dash")
        x0, y0, x1, y1 = c.bbox("polygon")
        assert 0 <= x0 < x1 <= c.winfo_width() and 0 <= y0 < y1 <= c.winfo_height()     # fit: the course is in view
        s = cm.scale
        cm._zoom(1.2, 400, 300)
        assert abs(cm.scale - s * 1.2) < 1e-9
        cm.toggle_follow()
        assert cm.follow and abs(cm.cx - cm.layer["boat"][0]) < 1e-9
        assert c.postscript(colormode="color")[:4] == "%!PS"
    finally:
        root.destroy()
