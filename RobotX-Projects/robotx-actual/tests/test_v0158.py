"""v0.1.58 (WP13 + Dmitrii 2026-11-07): the dock from the faces' key points - the line through >= 2 faces, its offset = the sway-search
path, `t3.dock_source: map | boxes`, the camera's face key points (cv/sim_observer.face_keypoints). No broker."""
import math
import os

import pytest
import yaml

from optiprimex.companion.state import LocalPose
from optiprimex.sim.fast_sil import FastSil
from optiprimex.usv.box_inspection import FaceGeometry, PointNE
from optiprimex.usv.dockpath import boxes_geometry, dock_geometry, fit_line

KIT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
POOL = os.path.join(KIT, "profiles", "bench", "world_pool.yaml")
BOXES = os.path.join(KIT, "profiles", "bench", "world_boxes.yaml")
NEAR_DOCK = {"x_n": 32.0, "y_e": 3.0, "hdg_deg": 0.0}
BOX_START = {"x_n": 38.0, "y_e": 0.0, "hdg_deg": 0.0}


def face(cn, ce, hdg_deg, width=1.0, colour="red", i=1):
    """A face centred at (cn, ce) looking along hdg_deg (toward the water), `width` wide."""
    h = math.radians(hdg_deg - 90.0)
    r = (math.cos(h), math.sin(h))
    g = FaceGeometry(PointNE(cn - width / 2 * r[0], ce - width / 2 * r[1]), PointNE(cn, ce), PointNE(cn + width / 2 * r[0], ce + width / 2 * r[1]))
    return {"box_id": i, "entity_id": i, "face": g, "colour": colour}


def test_line_from_two_boxes_and_its_offset_is_the_sway_path():
    faces = [face(45.0, 2.0, 180.0, i=1), face(45.0, 5.5, 180.0, colour="green", i=2)]
    g = boxes_geometry(faces, LocalPose(30.0, 4.0, 0.0, 0, 0, 0))
    assert g is not None and g.source == "boxes" and g.rms_m < 1e-6
    assert g.normal[0] == pytest.approx(-1.0) and abs(g.normal[1]) < 1e-9          # toward the boat (south)
    assert [b.number for b in g.bays] == [1, 2] and g.bays[0].face == (45.0, 2.0)  # left -> right facing the dock (west -> east)
    assert g.green().number == 2
    a, b = g.sway_path(5.0, 1.5)
    assert a == pytest.approx((40.0, 0.5)) and b == pytest.approx((40.0, 7.0))
    assert min(g.bow_in_hdg, 360.0 - g.bow_in_hdg) < 1e-6


def test_line_from_three_boxes_on_an_oblique_dock():
    hdg = 212.0                                                    # world_marina's dock looks 212 deg
    along = math.radians(hdg - 90.0)
    faces = [face(8.9 + k * 3.5 * math.cos(along), 10.28 + k * 3.5 * math.sin(along), hdg, i=k + 1) for k in range(3)]
    g = boxes_geometry(faces, LocalPose(0.0, 5.0, 32.0, 0, 0, 0))
    assert math.degrees(math.atan2(g.normal[1], g.normal[0])) % 360 == pytest.approx(hdg, abs=1e-6)
    assert [b.entity_id for b in g.bays] == [1, 2, 3]
    a, b = g.sway_path(5.0, 0.0)
    for p, f in ((a, faces[0]), (b, faces[2])):                    # 5 m off the outer faces along the normal
        m = f["face"].midpoint
        assert math.hypot(p[0] - m.n, p[1] - m.e) == pytest.approx(5.0)


def test_fit_line_least_squares_and_one_box_refusal():
    c, n, al, rms = fit_line([(0.0, 0.0), (0.0, 2.0), (0.1, 4.0), (-0.1, 6.0)], (-5.0, 3.0))
    assert abs(n[0] + 1.0) < 0.01 and rms < 0.1
    assert fit_line([(1.0, 1.0)]) is None
    assert boxes_geometry([face(45.0, 2.0, 180.0)], None) is None                 # one face: no line


def test_dock_source_boxes_on_world_boxes_reaches_docked():
    s = FastSil(BOXES, "T3", {"dock_source": "boxes"}, start=BOX_START)
    r = s.run(500)
    assert "DOCKED" in r["states"] and r["state"] == "DONE", (r["states"], r["safe_holds"])
    assert s.task.geom.source == "boxes" and len(s.task.geom.bays) == 3
    assert s.task.bay.number == 2 and s.task.bay.colour == "green"                 # the green face, no indicator in this world
    assert r["search"][0]["chosen"] == 2
    f = s.task.bay.face
    assert math.hypot(f[0] - 50.0, f[1] - 0.0) < 0.3


def test_dock_source_boxes_on_the_pool_dock_reaches_docked_in_the_green_bay():
    s = FastSil(POOL, "T3", {"dock_source": "boxes"}, start=NEAR_DOCK)
    r = s.run(400)
    assert "DOCKED" in r["states"] and r["state"] == "DONE", (r["states"], r["safe_holds"])
    assert s.task.geom.source == "boxes" and s.map.dock_tracks                     # the dock faces' key points
    assert s.task.bay.number == 2 and [e["bay"] for e in r["events"] if e.get("event") == "docking"] == [2]


def test_map_source_still_passes():
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    r = s.run(400)
    assert r["state"] == "DONE" and "DOCKED" in r["states"] and s.task.geom.source == "map"


def test_one_box_world_holds_with_the_reason(tmp_path):
    w = yaml.safe_load(open(BOXES))
    w["objects"] = [o for o in w["objects"] if o["id"] == 102]
    p = tmp_path / "world_one_box.yaml"
    p.write_text(yaml.safe_dump(w))
    r = FastSil(str(p), "T3", {"dock_source": "boxes"}, start=BOX_START).run(200)
    assert r["state"] == "SAFE_HOLD" and "DOCKED" not in r["states"]
    assert "1 stable face(s) with key points, a line needs 2" in r["safe_holds"][0]


def test_bad_dock_source_is_refused():
    with pytest.raises(ValueError):
        FastSil(POOL, "T3", {"dock_source": "lidar"}, start=NEAR_DOCK)
    with pytest.raises(ValueError):
        dock_geometry(None, None, source="lidar")


def test_camera_face_keypoints_on_a_rendered_pool_dock():
    from optiprimex.sim import world as W
    from optiprimex.sim.fast_sil import CameraPath
    from optiprimex.tools import dock_keypoints_eval as E
    wd = W.load(POOL)
    cp = CameraPath(wd, 7, (736, 414), "zed", "synthetic", None, True)
    pz = LocalPose(36.0, 5.5, 0.0, 0, 0, 0)                     # 9 m off the middle bay, all three in view
    fr = cp.frame(pz, 0.0, 0.0, 0.0)
    pay = cp.SO.observe(fr, [], cp.args, cp.SO.synthetic_dock_detections(fr))
    faces = [r for r in pay["dock"] if r["cls"] == "dock_face" and r.get("face_points")]
    assert len(faces) == 3
    truth = E.truth_faces(wd)["dock"]
    for r in faces:
        kp = {k: E.to_local(pz, v) for k, v in r["face_points"].items()}
        t = min(truth, key=lambda f: E.dist(f["mid"], kp["midpoint"]))
        for k, tk in (("left", "left"), ("midpoint", "mid"), ("right", "right")):
            assert E.dist(kp[k], t[tk]) < 0.12, (k, kp[k], t[tk])
    # cv_publish carries them on the dock item (body-polar range / bearing, as fake_perception's)
    from importlib.util import module_from_spec, spec_from_file_location
    spec = spec_from_file_location("cvp_v0158", os.path.join(KIT, "cv", "publish", "cv_publish.py"))
    m = module_from_spec(spec)
    spec.loader.exec_module(m)
    items = m.dock_to_items(pay["dock"], 1)["dock"]
    fp = [it["quality"]["face_points"] for it in items if it["quality"].get("face_points")]
    assert len(fp) == 3 and set(fp[0]) == {"left", "midpoint", "right"} and set(fp[0]["left"]) == {"range_m", "bearing_deg"}


def test_eval_line_metrics_on_truth():
    from optiprimex.tools import dock_keypoints_eval as E
    from optiprimex.sim import world as W
    truth = E.truth_faces(W.load(POOL))["dock"]
    tc, tn, ta = E.group_geom(truth)
    lm = E.line_metrics(truth, tc, tn, ta, LocalPose(35.0, 5.5, 0.0, 0, 0, 0))
    assert lm["ok"] and lm["angle_deg"] < 1e-6 and abs(lm["offset_m"]) < 1e-6 and lm["path_m"] < 1e-6
    assert E.line_metrics(truth[:1], tc, tn, ta, LocalPose(35.0, 5.5, 0.0, 0, 0, 0)) is None


def test_quick_list_has_this_file():
    assert "tests/test_v0158.py" in open(os.path.join(KIT, "Makefile")).read()


def test_dock_source_boxes_through_the_camera_reaches_docked():
    """The whole chain: the renderer -> cv/sim_observer's face key points -> cv_publish -> the map's dock tracks -> the fitted line."""
    s = FastSil(POOL, "T3", {"dock_source": "boxes"}, start=NEAR_DOCK, perception="camera", cam_dock=True, cam_size=(552, 310), seed=5)
    r = s.run(250, until=lambda x: "DOCKED" in x.states)
    assert "DOCKED" in r["states"], (r["states"], r["safe_holds"])
    g = s.task.geom
    assert g.source == "boxes" and len(g.bays) == 3 and g.rms_m < 0.1 and s.task.bay.number == 2
    for b, y in zip(g.bays, (2.0, 5.5, 9.0)):
        assert math.hypot(b.face[0] - 45.0, b.face[1] - y) < 0.2
