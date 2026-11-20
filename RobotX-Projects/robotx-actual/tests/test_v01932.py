"""v0.1.93.2 (13 Nov run, Dmitrii: "we are getting multiple blue targets ... a blue foam on the bottom of the buoy"): the observer's colour
vote over the mask's top part only (--colour-top 0.5), and the LED band over the lantern above the detector's box
(logs/cv_review/run_2026-11-13/)."""
from __future__ import annotations

import ast
import importlib.util
import sys
from types import SimpleNamespace

import numpy as np

from .conftest import ROOT

OBS = ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py"


def _load(name, rel):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    m = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = m
    spec.loader.exec_module(m)
    return m


MC = _load("multi_core_v01932", "cv/buoys_multi/multi_core.py")
LC = _load("led_core_v01932", "cv/buoys_multi/led_core.py")


def _colour_of():
    """The observer's colour_of, taken from its source (the observer imports the ZED SDK)."""
    tree = ast.parse(OBS.read_text())
    fn = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "colour_of")
    ns = {"np": np, "M": MC}
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(OBS), "exec"), ns)
    return ns["colour_of"]


def _marker():
    """A white marker (a mask 100 rows high) with a pale-blue foam float in its bottom 30 rows."""
    im = np.full((300, 300, 3), (60, 90, 70), np.uint8)
    im[100:170, 100:200] = (240, 240, 240)
    im[170:200, 100:200] = (230, 170, 90)                       # BGR: a light blue (S ~0.6, hue ~210)
    mask = np.zeros((300, 300), np.float32)
    mask[100:200, 100:200] = 1.0
    return im, mask


def test_the_foam_float_no_longer_makes_a_white_marker_blue():
    colour_of = _colour_of()
    im, mask = _marker()
    a = SimpleNamespace(sat_min=0.35, min_share=0.45, colour_top=1.0)
    assert colour_of(im, mask, a)[0] == "blue"                  # v0.1.93.1: the whole mask - the foam wins (30 % saturated, all blue)
    a.colour_top = 0.5
    assert colour_of(im, mask, a)[0] == "white"                 # the top half: the white body


def test_a_buoy_coloured_all_over_keeps_its_colour():
    colour_of = _colour_of()
    im = np.full((300, 300, 3), (60, 90, 70), np.uint8)
    im[100:200, 100:200] = (30, 30, 220)                        # red all over
    mask = np.zeros((300, 300), np.float32)
    mask[100:200, 100:200] = 1.0
    assert colour_of(im, mask, SimpleNamespace(sat_min=0.35, min_share=0.45, colour_top=0.5))[0] == "red"


def test_the_default_is_the_top_half_and_a_tiny_mask_keeps_every_pixel():
    src = OBS.read_text()
    assert 'p.add_argument("--colour-top", type=float, default=0.5' in src
    colour_of = _colour_of()
    im = np.full((50, 50, 3), (30, 30, 220), np.uint8)
    mask = np.zeros((50, 50), np.float32)
    mask[10:13, 10:14] = 1.0                                    # 12 pixels: the top half would be too few to vote
    assert colour_of(im, mask, SimpleNamespace(sat_min=0.35, min_share=0.45, colour_top=0.5))[0] == "red"


def test_the_led_band_covers_the_lantern_above_the_box():
    box = [100, 100, 200, 260]
    x1, y1, x2, y2 = LC.led_band(box)
    assert y1 < 100 < y2 <= 100 + 0.2 * 160 and x2 - x1 <= 0.35 * 100     # over the box's top edge, the middle 30 %
    im = np.full((400, 400, 3), (60, 90, 70), np.uint8)
    im[100:260, 100:200] = (235, 235, 235)
    im[50:80, 140:160] = (255, 40, 20)                          # a lit blue lantern on top of the box
    col, on, _ = LC.led_sample(im, box)
    assert on and col == "blue"
    im[50:80, 140:160] = (60, 90, 70)
    im[150:180, 135:165] = (255, 40, 20)                        # blue paint on the body (the box's 31-50 %) is no lamp any more
    assert not LC.led_sample(im, box)[1]


def test_a_few_bluish_pixels_on_a_white_marker_are_not_a_colour():
    """The 13 Nov replay (805 masks): white markers in shade showed 3-21 % bluish pixels and won "blue" on 12 of 400 voters; a colour now
    needs --colour-min-sat (0.20) of the pixels saturated. A buoy coloured all over keeps its colour."""
    colour_of = _colour_of()
    im = np.full((300, 300, 3), (60, 90, 70), np.uint8)
    im[100:200, 100:200] = (235, 235, 235)
    im[100:200, 140:160] = (20, 20, 20)                         # the black diamond: white alone stays under its 60 %
    im[100:108, 100:200] = (230, 170, 90)                       # 8 % bluish rows
    mask = np.zeros((300, 300), np.float32)
    mask[100:200, 100:200] = 1.0
    a = SimpleNamespace(sat_min=0.35, min_share=0.45, colour_top=0.5, colour_min_sat=0.0)
    assert colour_of(im, mask, a)[0] == "blue"                  # the old rule: 8 % is enough
    a.colour_min_sat = 0.20
    assert colour_of(im, mask, a)[0] in ("white", "unknown")
    red = np.full((300, 300, 3), (60, 90, 70), np.uint8)
    red[100:200, 100:200] = (30, 30, 220)
    assert colour_of(red, mask, a)[0] == "red"
    assert 'p.add_argument("--colour-min-sat", type=float, default=0.20' in OBS.read_text()


def test_the_start_turn_stops_at_a_near_flashing_blue_and_the_run_finishes(tmp_path):
    """Dmitrii 13 Nov: "after we get a blue fix during the initial look-around, we stop yawing and proceed to approach and look-lock" -
    only a FLASHING blue within start_scan_stop_r_m seen 2 x min_obs (the sim: a steady decoy or a far misread stopped it wrongly)."""
    import math as _m
    from optiprimex.companion.config import load as load_cfg
    from optiprimex.sim.fast_sil import FastSil
    from .test_v0192 import _world
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    assert cfg.extra["t1"]["start_scan_stop_blue"] is True and cfg.extra["guidance"]["yaw_rate_max_dps"] == 8.0 and cfg.extra["t1"]["scan_rate_dps"] == 6.5
    s = FastSil(_world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0)), "T1", {"planner": "simple"}, seed=1, cfg=cfg, guard={"fake": {}, "sup": {}})
    r = s.run(max_s=1500)
    assert r["state"] == "DONE" and len(s.task.passed) == 4
    assert s.task.blue_fix_id is not None and any("blue fix" in x for x in r["logs"])
    e = s.task.entry_xy
    assert e is not None and _m.hypot(e[0] - 3.0, e[1] + 3.0) < 1.5


def test_the_observer_prefers_the_agx_engine_and_the_pt_can_be_forced():
    """Dmitrii 13 Nov: "lets switch to engine anyway" - run.sh takes cv/buoys/models/best.engine on the AGX when it exists (FP16, 768);
    CV_DEFAULT_PT=1 = best.pt; anywhere else (the Mac) the .pt."""
    src = (ROOT / "cv" / "buoys_multi" / "run.sh").read_text()
    assert 'W="$WB.pt"' in src and '[ -f "$WB.engine" ] && [ "${CV_DEFAULT_PT:-0}" != 1 ] && W="$WB.engine"' in src
    assert src.index('W="$WB.engine"') < src.index('exec "$PY" "$HERE/multi_buoy_observer.py" --weights "$W"')


def test_the_lamp_model_ships_and_never_turns_a_pure_blue_lamp_green():
    """v0.1.93.2 hybrid (13 Nov grass test: the rule read green lanterns blue and red not at all; the model red 68/69, green 66/88): the
    model's weights ship beside led_core; a saturated pure blue lamp (which the model alone called 0.96 green) stays blue - the hue guard."""
    m = LC.lamp_model()
    assert m is not None and m["classes"] == ["none", "red", "green", "blue"] and len(m["W"]) == 18 * 4 + 3
    box = [100, 100, 200, 260]
    im = np.full((400, 400, 3), (60, 90, 70), np.uint8)
    im[100:260, 100:200] = (235, 235, 235)
    im[50:80, 140:160] = (255, 40, 20)
    col, on, sh = LC.led_sample(im, box)
    assert on and col == "blue" and "model" in sh
    im[50:80, 140:160] = (20, 20, 255)                          # a pure red lamp stays red
    assert LC.led_sample(im, box)[:2] == ("red", True)
    assert LC.led_sample(im, box, {"model": False})[:2] == ("red", True)   # the rule alone, as before


def test_launch_starts_the_observer_with_the_fast_camera():
    """Dmitrii 13 Nov: "bake them in" - agx_observer.sh (what LAUNCH runs) defaults to neural_light + hd1080; the env / flags override."""
    sh = (ROOT / "scripts" / "agx_observer.sh").read_text()
    assert 'DMODE="${AGX_DEPTH_MODE:-neural}"; RES="${AGX_RESOLUTION:-hd2k}"' in sh      # v0.1.94: back to hd2k + neural
    assert '[ -n "$DMODE" ] && DARG="$DARG --depth-mode $DMODE"' in sh and '[ -n "$RES" ] && DARG="$DARG --resolution $RES"' in sh


def test_m1t1_ignores_the_uav_map_and_m1t2_keeps_it():
    """Dmitrii 13 Nov: "we have UAV on our network" - while T1 runs in camera mode (m1t1) the UAV team's map adds and recolours nothing,
    and what an earlier flight put in the map is dropped; m1t2 (colour_source uav), scout and the GUI keep the feed."""
    from optiprimex.companion.state import LocalPose
    from optiprimex.usv.perception_map import PerceptionMap
    from optiprimex.companion.config import load as load_cfg
    from optiprimex.companion.frames import ControlOrigin
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    m = PerceptionMap(cfg, clock=lambda: 1.0)
    o = ControlOrigin(1.2810202, 103.8550097)
    msg = {"frame_id": "a", "items": [{"lat": 1.28103, "lon": 103.85502, "colour": "blue", "lit": "flash", "class": "buoy", "id": 7}]}
    uav = lambda: [e for e in m.ents.values() if e.source == "uav"]
    assert m.ingest_geo(msg, o, 1.0) == 1 and len(uav()) == 1                                         # camera mode, outside T1: as before
    assert m.forget_uav() == 1 and not uav()
    m.uav_ingest_off = True
    assert m.ingest_geo(dict(msg, frame_id="b"), o, 2.0) == 0 and not uav()
    assert m.uav_stats["ignored"] == 1 and m.uav_feeds["contract"]["messages"] == 2                  # the feed still seen (GUI / touchdown)
    src = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text()
    assert 'off = self.runner.task_name == "T1" and self.map.colour_source == "camera"' in src and "self.map.forget_uav()" in src


def test_no_blue_buoy_ends_with_3m_ahead_a_quick_turn_and_done(tmp_path):
    """Dmitrii 13 Nov: "what if we don't see a blue buoy - advance 3 meters, make a quick 360 turn, and end mission"."""
    from optiprimex.companion.config import load as load_cfg
    from optiprimex.sim.fast_sil import FastSil
    from .test_v0192 import _world
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    assert cfg.extra["t1"]["no_entry_finale"] is True
    s = FastSil(_world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0), lights=False), "T1", {"planner": "simple"}, seed=1, cfg=cfg,
                guard={"fake": {}, "sup": {}})
    r = s.run(max_s=2500)
    st = [x for i, x in enumerate(r["states"]) if i == 0 or x != r["states"][i - 1]]
    assert r["state"] == "DONE" and st[-3:] == ["FINAL_ADVANCE", "FINAL_SCAN", "DONE"] and not s.task.passed, st[-6:]
    assert any("no blue buoy" in x for x in r["logs"])


def test_the_drone_map_shows_which_buoys_arrived_linked_or_uav_only():
    """Dmitrii 13 Nov: "in the gui I need status updates from drone stages ... and if and which buoys we receive": the map digest lists
    each drone buoy (linked to a camera track or UAV-only, the UAV's colour / light); the tk banner's drone line and the page show it."""
    from optiprimex.usv.perception_map import PerceptionMap
    from optiprimex.companion.config import load as load_cfg
    from optiprimex.companion.frames import ControlOrigin
    from optiprimex.tools.tkconsole import model as TM
    m = PerceptionMap(load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml"), clock=lambda: 1.0)
    o = ControlOrigin(1.2810202, 103.8550097)
    m.ingest_geo({"frame_id": "a", "items": [{"lat": 1.28103, "lon": 103.85502, "colour": "blue", "lit": "flash", "class": "buoy", "id": 7},
                                             {"lat": 1.28106, "lon": 103.85505, "colour": "red", "lit": "steady", "class": "buoy", "id": 8}]}, o, 1.0)
    d = m.uav_digest(2.0)
    assert len(d["items"]) == 2 and {i["uav_colour"] for i in d["items"]} == {"blue", "red"} and all(i["src"] == "uav" for i in d["items"])
    line = TM.drone_map_line(d)
    assert line.startswith("drone map: ") and "2 buoy(s)" in line and "blue flash UAV-only" in line and "red steady UAV-only" in line
    assert TM.drone_map_line({}) == "" and "IGNORED in m1t1" in TM.drone_map_line(dict(d, ingest_off=True, ignored=3))
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "ignored in m1t1" in page and "UAV-only #${esc(it.id)}" in page
