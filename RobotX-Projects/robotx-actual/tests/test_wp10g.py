"""ROADMAP WP10g (v0.1.34), buoy localisation (CV_TRAIN's REQUEST_localisation.md): one camera model (the ZED's own calibration)
everywhere, range = surface + radius, bearing-only detections beyond the depth -> rays -> triangulated points, a ZED-like depth model."""
from __future__ import annotations

import importlib.util
import json
import math
import random
import shutil

import pytest

from .conftest import ROOT

CV = ROOT / "cv"


def core():
    spec = importlib.util.spec_from_file_location("multi_core_wp10g", CV / "buoys_multi" / "multi_core.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_one_camera_model():
    from optiprimex.sim import world as W
    from optiprimex.sim import zed
    assert zed.hfov_deg("hd2k") == pytest.approx(50.95, abs=0.05) and zed.hfov_deg("hd720") == pytest.approx(57.83, abs=0.05)
    for name in ("pool", "por", "boxes", "empty", "marina"):                         # fake perception's field of view (bench: a wide test camera)
        assert W.load(str(ROOT / "profiles" / "bench" / f"world_{name}.yaml"))["camera"]["fov_deg"] == pytest.approx(zed.hfov_deg(), abs=0.01)
    bad = ROOT / "profiles" / "bench" / "world_pool.yaml"
    w = W.load(str(bad))
    assert w["camera"]["intrinsics"] == "zed2i_hd2k"
    pytest.importorskip("numpy")
    from optiprimex.sim import render as R
    cam = R.Camera(w=1104, h=621)                                                       # hd2k at half size
    fx, fy, cx, cy = zed.scaled(1104, 621)
    assert (cam.fx, cam.cx, cam.cy) == pytest.approx((fx, cx, cy)) and cam.fx == pytest.approx(2317.22 / 2, abs=0.01)
    for brg in (-20.0, 0.0, 12.0):                                                       # project() through the calibration
        p = (10.0 * math.cos(math.radians(brg)), 10.0 * math.sin(math.radians(brg)), cam.height_m)
        u, v, z = R.project(cam, 0.0, 0.0, 0.0, p)
        assert u == pytest.approx(cx + fx * math.tan(math.radians(brg)), abs=1e-3) and v == pytest.approx(cy, abs=1e-3)
        assert zed.bearing_deg(u, 1104, 621) == pytest.approx(brg, abs=1e-4)
    sym = R.Camera(w=640, h=360, hfov_deg=70.0)                                         # an explicit field of view: a centred pinhole
    assert sym.cx == 320.0 and sym.hfov_deg == 70.0


def test_range_is_surface_plus_radius_in_the_rendered_world():
    """CV_TRAIN measured the students' measure_mask 0.22 m short in the sim (the visible surface); with the radius the bias is
    within +-0.05 m. The same method here: the median 3-D point of the buoy's visible pixels from the rendered depth."""
    np = pytest.importorskip("numpy")
    from optiprimex.sim import render as R
    M = core()
    cam = R.Camera(w=1104, h=621)
    d = 0.5                                                                              # the sim's buoy: 0.5 m, centre 0.2 d up
    for rng in (5.0, 9.0, 14.0, 19.0):
        for brg in (-15.0, 8.0):
            cxw, cyw = rng * math.cos(math.radians(brg)), rng * math.sin(math.radians(brg))
            sc = R.Scene(spheres=[R.Sphere((cxw, cyw, 0.2 * d), d / 2.0, R.COLOURS["red"], tag="buoy:1")])
            _, depth, pid = R.render_ids(sc, 0.0, 0.0, 0.0, cam)
            vv, uu = np.nonzero(pid == 0)
            z = depth[vv, uu]
            x = (uu + 0.5 - cam.cx) / cam.fx * z
            y = (vv + 0.5 - cam.cy) / cam.fy * z
            right, down, fwd = float(np.median(x)), float(np.median(y)), float(np.median(z))
            meas = {"range_m": math.sqrt(right ** 2 + down ** 2 + fwd ** 2), "forward_m": fwd, "right_m": right, "down_m": down,
                    "bearing_deg": math.degrees(math.atan2(right, fwd))}
            truth = math.sqrt(cxw ** 2 + cyw ** 2 + (cam.height_m - 0.2 * d) ** 2)
            assert meas["range_m"] - truth < -0.12                                       # the surface: short
            c = M.centre_correction(meas, M.buoy_radius())
            assert abs(c["range_m"] - truth) <= 0.05, (rng, brg, c["range_m"], truth)
            assert c["surface_range_m"] == pytest.approx(meas["range_m"]) and c["bearing_deg"] == meas["bearing_deg"]
            assert math.degrees(math.atan2(c["right_m"], c["forward_m"])) == pytest.approx(meas["bearing_deg"])


def test_bearing_only_record_payload_and_cv_publish(tmp_path):
    M = core()
    b = M.bearing_record(1500.0, 597.9, 2317.22, 2318.07, 1134.1, 597.9, 0.8, "green", {}, "beyond_depth", 0)
    assert b["range_m"] is None and b["bearing_only"] and b["bearing_deg"] == pytest.approx(math.degrees(math.atan2(365.9, 2317.22)), abs=1e-3)
    assert abs(b["elev_deg"]) < 1e-6 and M.mask_centre([1, 2, 9], [4, 5, 6]) == (2.0, 5.0)
    ranged = M.buoy_record({"range_m": 8.0, "forward_m": 8.0, "right_m": 0.0, "down_m": 0.5, "bearing_deg": 0.0}, 0.9, "red", {}, None, 1)
    p = M.frame_payload("2026-11-05T10:00:00.000+00:00", 3, [b, ranged], {})
    assert [x.get("range_m") for x in p["buoys"]] == [8.0, None] and p["measurement"]["colour"] == "red" and p["n_bearing_only"] == 1
    assert M.frame_payload("u", 4, [b], {})["status"] == "BEARING_ONLY" and M.frame_payload("u", 4, [b], {})["measurement"].get("range_m") is None
    kit = tmp_path / "MASTER_SCRIPTS"
    (kit / "cv" / "publish").mkdir(parents=True)
    shutil.copy(CV / "publish" / "cv_publish.py", kit / "cv" / "publish" / "cv_publish.py")
    spec = importlib.util.spec_from_file_location("cv_publish_wp10g", kit / "cv" / "publish" / "cv_publish.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    class Sink:
        def __init__(self):
            self.msgs = []

        def publish(self, topic, payload, qos=0, retain=False):
            self.msgs.append((topic, json.loads(payload)))
    ns = mod.main.__globals__["argparse"].Namespace(mqtt="127.0.0.1:1884", vehicle="usv", vehicle_id="USV1", camera="zed2i_left",
                                                     detector="multi_buoy_observer", colour="unknown")
    pub = mod.Publisher(ns)
    pub.c = Sink()
    mod.publish_payload(pub, p)
    items = pub.c.msgs[-1][1]["body"]["items"]
    assert len(items) == 2 and items[1]["range_m"] is None and items[1]["quality"]["bearing_only"] and items[1]["colour"] == "green"
    from optiprimex.companion.state import ObsItem
    it = ObsItem.from_body(items[1])
    assert it.range_m == 0.0 and it.body()["range_m"] is None and it.bearing_deg == pytest.approx(b["bearing_deg"], abs=0.01)


def _drive_past(seed: int, bx: float = 32.0, by: float = 0.0, triangulate: bool = False):
    from optiprimex.companion import config as C
    from optiprimex.companion.state import LocalPose, ObsItem, Observation
    from optiprimex.usv.perception_map import PerceptionMap
    cfg = C.load(str(ROOT / "profiles" / "bench" / "usv_bench.yaml"))
    cfg.map.triangulate = triangulate
    m = PerceptionMap(cfg, clock=lambda: 0.0)
    rnd = random.Random(seed)
    first = None
    for k in range(150):                                       # the boat crabs east 30 m, 32 m south of the buoy, heading north
        t, y = k * 0.2, -15.0 + k * 0.2
        pose = LocalPose(0.0, y, 0.0, 1.28, 103.85, t)
        brg = math.degrees(math.atan2(by - y, bx)) + rnd.gauss(0.0, 0.25)
        if abs(brg) > 25.47:
            continue
        m.ingest(Observation(t_cam=t, kind="buoys", items=[ObsItem(id=1, cls="buoy", colour="red", conf=0.8, range_m=0.0, bearing_deg=brg)]),
                 pose, now=t)
        m.age_out(t)
        if first is None:
            first = m.digest(pose, t)
    return m, first


def test_rays_are_leads_by_default():
    """Decided 2026-11-05 (Dmitrii): the pathing_sim way - a bearing stays a ray (a lead for the map and the search), never a
    position by itself; a position only from depth (the depth limit raised to 25 m)."""
    from optiprimex.companion import config as C
    assert C.load(str(ROOT / "profiles" / "bench" / "usv_bench.yaml")).map.triangulate is False
    m, first = _drive_past(1)
    assert m.entities(include_young=True) == [] and m.n_triangulated == 0 and len(m.rays) >= 1
    assert all(r["colour"] == "red" for r in m.digest(None, 30.0)["rays"])
    from optiprimex.sim import zed
    assert zed.DEPTH_MAX_M == 25.0
    src = (CV / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert "max_depth_m=a.max_depth" in src and "depth_max_m=a.max_depth" in src and '"--max-depth", type=float, default=25.0' in src


def test_rays_become_a_point_as_the_boat_moves_when_triangulating():
    for seed in (1, 2, 3, 4, 5):
        m, first = _drive_past(seed, triangulate=True)
        assert len(first["rays"]) == 1 and first["entities"] == [] and first["rays"][0]["colour"] == "red"   # first: only a bearing
        ents = m.entities()
        assert len(ents) == 1 and math.hypot(ents[0].x_n - 32.0, ents[0].y_e) < 1.5 and ents[0].colour == "red", (seed, ents)
        assert m.n_triangulated > 10
    # a ranged buoy along a ray explains it: no new ray
    from optiprimex.companion import config as C
    from optiprimex.companion.state import LocalPose, ObsItem, Observation
    from optiprimex.usv.perception_map import PerceptionMap
    pm = PerceptionMap(C.load(str(ROOT / "profiles" / "bench" / "usv_bench.yaml")), clock=lambda: 0.0)
    pose = LocalPose(0.0, 0.0, 0.0, 1.28, 103.85, 0.0)
    for i in range(4):
        pm.ingest(Observation(t_cam=i, kind="buoys", items=[ObsItem(id=1, cls="buoy", colour="red", conf=0.8, range_m=15.0, bearing_deg=5.0)]), pose, now=i)
    pm.ingest(Observation(t_cam=5, kind="buoys", items=[ObsItem(id=2, cls="buoy", colour="red", conf=0.8, range_m=0.0, bearing_deg=5.5)]), pose, now=5)
    assert pm.rays == []
    pm.ingest(Observation(t_cam=6, kind="buoys", items=[ObsItem(id=3, cls="buoy", colour="red", conf=0.8, range_m=0.0, bearing_deg=-30.0)]), pose, now=6)
    assert len(pm.rays) == 1
    pm.age_out(40.0)                                                                      # ray_ttl_s 30
    assert pm.rays == [] and pm.digest(pose, 40.0)["rays"] == []


def test_fake_perception_reports_bearings_beyond_its_depth():
    from optiprimex.sim import fake_perception as FP
    a = FP.parse_args(["--world", str(ROOT / "profiles" / "bench" / "world_por.yaml")])
    fp = FP.FakePerception(a)
    fp.cam.update(dropout=0.0, p_drop_max=0.0, noise_bearing_deg=0.0)
    fp.objects = [dict(id=1, kind="buoy", colour="red", light="steady", x_n=10.0, y_e=0.0), dict(id=2, kind="buoy", colour="green", light="steady", x_n=32.0, y_e=2.0),
                  dict(id=3, kind="buoy", colour="red", light="steady", x_n=60.0, y_e=0.0)]
    import time as _t
    fp.pose, fp.pose_t = {"x_n": 0.0, "y_e": 0.0, "hdg_deg": 0.0, "lat": None, "lon": None}, _t.monotonic()
    items = {it.id: it for it in fp.observe()["buoys"].items}
    assert items[1].range_m > 0 and items[2].range_m == 0.0 and items[2].quality["bearing_only"] and 3 not in items
    assert items[2].bearing_deg == pytest.approx(math.degrees(math.atan2(2.0, 32.0)), abs=1e-6)


def test_zed_like_depth():
    np = pytest.importorskip("numpy")
    from optiprimex.sim import zed
    z = np.full((200, 200), 10.0, np.float32)
    z[:, 100:] = 30.0
    ids = np.zeros((200, 200), np.int32)                                                  # all "object" pixels: no texture holes
    n = zed.stereo_noise(z, ids, 1158.6, np.random.default_rng(1), {"flying_px": 0})
    s10, s30 = float(np.nanstd(n[:, :100])), float(np.nanstd(n[:, 100:]))
    assert s30 / s10 == pytest.approx(9.0, rel=0.15)                                     # sigma ~ z^2
    assert s10 == pytest.approx(100 * 0.25 / (1158.6 * zed.BASELINE_M), rel=0.1)
    flat = zed.stereo_noise(np.full((100, 100), 20.0, np.float32), np.full((100, 100), -1, np.int32), 1158.6, np.random.default_rng(2))
    assert 0.1 < float(np.isinf(flat).mean()) < 0.6                                       # holes on the water
    far = zed.stereo_noise(np.full((10, 10), 60.0, np.float32), np.zeros((10, 10), np.int32), 1158.6, np.random.default_rng(3), {"sigma_d_px": 0.05})
    assert np.isinf(far).all()                                                            # beyond max_m (40): nothing
    edge = zed.stereo_noise(z, np.where(np.arange(200)[None, :] < 100, 1, 2).repeat(200, 0).astype(np.int32), 1158.6, np.random.default_rng(4),
                            {"sigma_d_px": 0.0})
    col = edge[:, 100]
    assert ((col > 10.5) & (col < 29.5)).mean() > 0.8                                     # flying pixels between the two surfaces
    assert (edge[:, :99] == 10.0).all()


def test_cv_train_follow_up_and_its_first_model(tmp_path):
    """v0.1.34.2: CV_TRAIN's follow-up (a range within 2 sigma_z of the depth limit is a bearing; the bearing threshold = the
    detector's conf) and its first model (cvt-v1-l: a model folder -> weights, size, conf, iou, the buoy class)."""
    import subprocess
    M = core()
    assert M.sigma_z(25.0, 2317.22) == pytest.approx(625 * 0.25 / (2317.22 * 0.11991847621653))
    assert M.near_limit(24.0, 25.0, 2317.22) and not M.near_limit(23.0, 25.0, 2317.22) and not M.near_limit(24.9, 25.0, 2317.22, k_sigma=0)
    from optiprimex.sim import zed
    assert M.ZED_BASELINE_M == zed.BASELINE_M
    d = tmp_path / "cvt-x"
    d.mkdir()
    (d / "input.yaml").write_text("imgsz: 768\nconf_default: 0.5\niou_default: 0.7\n")
    (d / "classes.yaml").write_text("names:\n  0: buoy\n  1: dock_face\n  2: window\n  3: box\n")
    (d / "weights.pt").write_text("pt")
    sp = M.model_spec(str(d))
    assert sp["weights"].endswith("weights.pt") and (sp["imgsz"], sp["conf"], sp["iou"], sp["buoy_ids"], sp["name"]) == (768, 0.5, 0.7, [0], "cvt-x")
    (d / "weights.engine").write_text("engine")
    assert M.model_spec(str(d))["weights"].endswith("weights.engine") and M.model_spec(str(d), prefer_engine=False)["weights"].endswith(".pt")
    src = (CV / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert "M.model_spec(a.model" in src and "a.bearing_conf = a.bearing_conf if a.bearing_conf is not None else a.conf" in src
    assert "M.near_limit(meas.range_m, a.max_depth, cal.fx, a.limit_sigma)" in src and "not in buoy_ids" in src
    h = subprocess.run(["bash", str(CV / "sync_cvtrain.sh"), "--help"], capture_output=True, text=True, timeout=10)
    assert h.returncode == 0 and "sha256.txt" in h.stdout and "never writes" in h.stdout
