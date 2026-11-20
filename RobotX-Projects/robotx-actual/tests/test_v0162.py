"""v0.1.62 - Dmitrii's round 2026-11-07: MISSION's preselect / start with the answer in place + a copy icon on the command line; the boat
window's top-plate boxes clickable (a menu, two clicks within 4 s; VEHICLE = enable_auto / disable_auto / confirm_safe through the OCS);
m3t1's dock on the OCS - the CV boxes' key points on the camera (frame_dets `kp`), the faces' key points + the sway path on the map
(`dock_view` in the map digest). No broker."""
from __future__ import annotations

import math
import os

import pytest

from optiprimex.companion.state import LocalPose
from optiprimex.usv import dockpath as D
from optiprimex.usv.box_inspection import FaceGeometry, PointNE

from .conftest import ROOT

KIT = str(ROOT)
POOL = os.path.join(KIT, "profiles", "bench", "world_pool.yaml")


def _face(cn, ce, hdg_deg, width=1.0, colour="red", i=1):
    h = math.radians(hdg_deg - 90.0)
    r = (math.cos(h), math.sin(h))
    g = FaceGeometry(PointNE(cn - width / 2 * r[0], ce - width / 2 * r[1]), PointNE(cn, ce), PointNE(cn + width / 2 * r[0], ce + width / 2 * r[1]))
    return {"box_id": i, "entity_id": i, "face": g, "colour": colour}


# ---- the map: dock_view ------------------------------------------------------------------------------------------------------------
def test_dock_view_gives_the_key_points_the_line_and_the_sway_path(monkeypatch):
    faces = [_face(45.0, 2.0, 180.0, i=1), _face(45.0, 5.5, 180.0, colour="green", i=2)]
    monkeypatch.setattr(D, "box_faces", lambda pmap, kw=None: faces)
    monkeypatch.setattr(D, "dock_geometry", lambda pmap, pose, now=None, source="map", stable_kw=None: D.boxes_geometry(faces, pose))
    pose = LocalPose(30.0, 4.0, 0.0, 0, 0, 0)
    v = D.dock_view(object(), pose, 0.0, {"dock_source": "boxes"}, {})
    assert v["source"] == "boxes" and len(v["faces"]) == 2
    assert v["faces"][0]["mid"] == [45.0, 2.0] and v["faces"][1]["colour"] == "green"
    f0 = v["faces"][0]
    assert f0["left"][0] == f0["right"][0] == 45.0 and abs(f0["left"][1] - f0["right"][1]) == pytest.approx(1.0)   # 1 m wide, on the line
    sd, sd2 = D.standoff_lines({"dock_source": "boxes"}, {})
    assert v["standoff_m"] == sd and v["standoff2_m"] == sd2 and sd == pytest.approx(5.70, abs=0.01)
    a, b = v["sway"]
    assert a[0] == pytest.approx(45.0 - sd, abs=0.01) and b[0] == pytest.approx(45.0 - sd, abs=0.01)          # the face line, offset toward the boat
    assert v["sway2"][0][0] == pytest.approx(45.0 - sd2, abs=0.01) and [x["n"] for x in v["bays"]] == [1, 2]
    monkeypatch.setattr(D, "box_faces", lambda pmap, kw=None: [])
    monkeypatch.setattr(D, "dock_geometry", lambda *a, **k: None)
    assert D.dock_view(object(), pose) is None                                          # no dock: nothing drawn


def test_the_map_digest_carries_dock_view_and_the_map_draws_it():
    src = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text()
    assert 'extra["dock_view"] = dock_view(self.map, vs.local, now, self.cfg.extra.get("t3"), self.cfg.extra.get("t1"))' in src
    page = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    for needle in ("m.dock_view", "v.sway", "v.sway2", "sway ${(+v.standoff_m).toFixed(2)} m", "f.left", "f.mid", "f.right"):
        assert needle in page, needle
    assert "esc(" not in page                                                          # map.html has no esc(): never call it


# ---- the camera: the key points in pixels ------------------------------------------------------------------------------------------
def test_kp_pixels_from_the_bearings():
    import sys
    sys.path.insert(0, str(ROOT / "cv" / "sim_observer"))
    import sim_observer as SO
    cam = {"fx": 500.0, "cx": 320.0}
    fp = {"left": {"forward_m": 10.0, "right_m": -1.0}, "midpoint": {"forward_m": 10.0, "right_m": 0.0}, "right": {"forward_m": 10.0, "right_m": 1.0}}
    kp = SO.kp_pixels(fp, [260, 100, 380, 140], cam)
    assert kp == {"left": [270.0, 120.0], "midpoint": [320.0, 120.0], "right": [370.0, 120.0]}
    assert SO.kp_pixels(fp, None, cam) is None and SO.kp_pixels(fp, [1, 2, 3, 4], None) is None and SO.kp_pixels(None, [1, 2, 3, 4], cam) is None


def test_the_rendered_pool_dock_publishes_kp_in_frame_dets():
    from optiprimex.sim import world as W
    from optiprimex.sim.fast_sil import CameraPath
    cp = CameraPath(W.load(POOL), 7, (736, 414), "zed", "synthetic", None, True)
    fr = cp.frame(LocalPose(36.0, 5.5, 0.0, 0, 0, 0), 0.0, 0.0, 0.0)               # 9 m off the middle bay: the three in view
    pay = cp.SO.observe(fr, [], cp.args, cp.SO.synthetic_dock_detections(fr))
    faces = [d for d in pay["frame_dets"]["dets"] if d["cls"] == "dock_face"]
    with_kp = [d for d in faces if d.get("kp")]
    assert len(with_kp) >= 2, faces
    for d in with_kp:
        k = d["kp"]
        assert {"left", "midpoint", "right"} <= set(k)
        assert d["x0"] - 3 <= k["left"][0] < k["midpoint"][0] < k["right"][0] <= d["x1"] + 3      # inside the box, in order
        assert d["y0"] <= k["midpoint"][1] <= d["y1"]


def test_the_pages_draw_the_key_points():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "function kpSvg(d,c,lw)" in page and page.count("kpSvg(d,c,lw)+") == 2                # OVERVIEW / MISSION and the CAMERA tab


# ---- MISSION: preselect with the answer, copy -----------------------------------------------------------------------------------
def test_mission_preselect_answers_in_place_and_copies():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('id="qack"', 'id="mack"', 'data-copy="qprev"', 'data-copy="mprev"', "async function ackSend(line,id)",
                   '$("qpre").onclick=()=>ackSend(qLine("mission"),"qack")', '$("mpre").onclick=()=>ackSend(mline("mission"),"mack")',
                   'ackSend(qLine("start_task"),"qack")', 'ackSend(mline("start_task"),"mack")', 'closest("button.cpb")', "✓ preselect"):
        assert needle in page, needle
    assert '$("qprev").parentElement' in page                                           # cPlace moves the row with its copy button


# ---- the boat window's plate --------------------------------------------------------------------------------------------------
def test_the_plate_boxes_are_buttons_with_two_clicks():
    from optiprimex.tools import boat_web as BW
    for w in ("enable_auto", "disable_auto", "confirm_safe", "stop_reset"):
        assert w in BW.OCS_LINES
    assert BW.ocs_line("http://127.0.0.1:1/", "start_task m3t1")["ok"] is False          # only the listed lines travel
    assert "refused by the page" in BW.ocs_line("http://127.0.0.1:1/", "reset")["text"]
    page = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text()
    for needle in ('id="plmenu"', "const PLM={vehicle:", "mode:{h:'FCU MODE", "arm:{h:'ARM", "holder:{h:'FCU HOLDER", "log:{h:'LOG", "cv:{h:'CV FOLD",
                   "['pl-vehicle','vehicle']", "Date.now()-PLM_ARM.t>=4000", "'send '+b.dataset.line+'?'", "fetch('/ocs'", "#plate .pb{cursor:pointer}"):
        assert needle in page, needle
    assert "reporting OFF" not in page.split('id="b-sr"')[1].split(">")[0]               # v0.1.59: STOP & RESET keeps reporting
    assert "report off, reset" not in page


def test_quick_list_has_this_file():
    assert "tests/test_v0162.py" in (ROOT / "Makefile").read_text()
