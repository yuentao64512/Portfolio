"""ROADMAP WP10e (v0.1.36), the CV loop closed in the simulator: simview's /frame.npz, cv/sim_observer (the students' measuring code on
rendered frames + our rules -> latest_status.json), --detector model with a stand-in, fake perception without buoys, run_sim.sh's
camera path and usv_ui, the dark-bodied flashing buoys."""
from __future__ import annotations

import importlib.util
import io
import json
import math
import subprocess
import sys
import threading
import types
import urllib.request

import pytest

from .conftest import ROOT, free_port

np = pytest.importorskip("numpy")
pytest.importorskip("PIL")


def observer():
    spec = importlib.util.spec_from_file_location("sim_observer_t", ROOT / "cv" / "sim_observer" / "sim_observer.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def _frame(world_name="pool", x=8.0, y=-6.0, hdg=0.0, lit=None, w=1104, h=621, override=None):
    from optiprimex.sim import render as R
    from optiprimex.sim import world as W
    wd = W.load(str(ROOT / "profiles" / "bench" / f"world_{world_name}.yaml"))
    if override:
        override(wd)
    cam = R.Camera(w=w, h=h)
    sc = R.scene_from_world(wd, lit=lit)
    rgb, depth, pid = R.render_ids(sc, x, y, hdg, cam)
    codes, objs, kinds = [], [], []
    for i, sp in enumerate(sc.spheres):
        k, o = sp.tag.split(":")
        codes.append(i), objs.append(int(o)), kinds.append("buoy" if k in ("buoy", "light") else k)
    for j, bx in enumerate(sc.boxes):
        k, o = bx.tag.split(":")
        codes.append(R.BOX_ID0 + j), objs.append(int(o)), kinds.append(k)
    fr0 = W.WorldFrame(wd["origin"])
    lat, lon = fr0.to_latlon(x, y)
    meta = {"t": 0.0, "frame": 1, "pose": {"x_n": x, "y_e": y, "hdg_deg": hdg, "lat": lat, "lon": lon, "gps_fix": 3},
            "camera": {"w": w, "h": h, "fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy}}
    return {"rgb": rgb, "depth": depth, "ids": pid, "prim_code": np.array(codes), "prim_obj": np.array(objs), "prim_kind": np.array(kinds),
            "meta": np.array(json.dumps(meta))}, wd, fr0


def test_the_students_code_loads_without_pyzed():
    SO = observer()
    S = SO.students()
    assert {"measure_mask", "buoy_geolocation", "BuoyMeasurement", "offset_to_gps"} <= set(vars(S)) and S.EARTH_M == 6_378_137.0
    img = np.zeros((9, 9), np.uint8)
    img[2:7, 2:8] = 1
    er = SO._erode(img, np.ones((5, 5), np.uint8))
    assert er.sum() == 2 and er[4, 4] == 1 and er[4, 5] == 1                     # a 5x6 block eroded by a 5x5 kernel: its 1x2 core
    src = (ROOT / "cv" / "buoys" / "actual_buoy_observer.py").read_text()
    assert "def measure_mask(xyz, mask, min_valid_fraction=0.25, max_depth_m=20.0):" in src     # the snapshot unchanged


def test_the_observer_on_a_rendered_pool_frame():
    SO = observer()
    fr, wd, fr0 = _frame()
    objs = {o["id"]: o for o in wd["objects"]}
    for noise in ("none", "zed"):
        p = SO.observe(fr, SO.synthetic_detections(fr), SO.parse_args(["--depth-noise", noise]))
        assert p["status"] == "OK" and p["observer"] == "sim_observer"
        ranged = [b for b in p["buoys"] if b.get("range_m") is not None]
        assert len(ranged) >= 7
        for b in ranged:
            o = objs[b["truth_obj"]]
            x, y = fr0.to_world(b["buoy_lat"], b["buoy_lon"])
            assert math.hypot(x - o["x_n"], y - o["y_e"]) < 0.15, (noise, b)                  # the centre, not the surface
            want = "black" if o.get("light") == "off" else o["colour"]
            assert b["colour"] == want or (o.get("light") == "flash" and b["colour"] == "black"), (b["colour"], o)
        far = [b for b in p["buoys"] if b.get("range_m") is None]
        assert all(b["reason"] in ("near_limit", "beyond_depth", "depth_invalid") for b in far)


def test_a_flashing_buoy_is_its_light():
    """v0.1.36: the flashing buoys have a dark body - the camera sees the light's colour when it is on, black when it is off."""
    SO = observer()
    a = SO.parse_args(["--depth-noise", "none"])
    seen = []
    for lit in (True, False):
        fr, _, _ = _frame(x=0.0, y=0.0, hdg=-54.0, lit={1: lit})
        p = SO.observe(fr, SO.synthetic_detections(fr), a)
        seen.append(next(b["colour"] for b in p["buoys"] if b["truth_obj"] == 1))
    assert seen == ["blue", "black"]


def test_detector_model_with_a_stand_in(tmp_path, monkeypatch):
    """--detector model: a CV_TRAIN folder through model_spec, Ultralytics' result shape (a stand-in module here); only `buoy` kept."""
    SO = observer()
    fr, wd, fr0 = _frame()
    synth = SO.synthetic_detections(fr)
    one = next(d for d in synth if d["obj"] == 4)
    ys, xs = np.nonzero(one["mask"] > 0.5)

    class T:
        def __init__(self, v):
            self.v = v

        def item(self):
            return self.v

        def cpu(self):
            return self

        def numpy(self):
            return self.v

    class Box:
        def __init__(self, cls, conf):
            self.cls, self.conf = T(cls), T(conf)

    small = one["mask"][::2, ::2]                                                        # the model's mask at its own size: resized back
    res = types.SimpleNamespace(boxes=[Box(0, 0.8), Box(1, 0.9)], masks=types.SimpleNamespace(data=T(np.stack([small, small]))))

    class YOLO:
        def __init__(self, weights, task=None):
            assert weights.endswith("weights.pt") and task == "segment"

        def predict(self, img, conf, imgsz, iou, verbose):
            assert img.shape == fr["rgb"].shape and (conf, imgsz, iou) == (0.5, 768, 0.7)
            return [res]
    monkeypatch.setitem(sys.modules, "ultralytics", types.SimpleNamespace(YOLO=YOLO))
    d = tmp_path / "cvt-stand-in"
    d.mkdir()
    (d / "input.yaml").write_text("imgsz: 768\nconf_default: 0.5\niou_default: 0.7\n")
    (d / "classes.yaml").write_text("names:\n  0: buoy\n  1: dock_face\n")
    (d / "weights.pt").write_text("stand-in")
    det = SO.ModelDetector(str(d))
    dets = det(fr)
    assert len(dets) == 1 and dets[0]["conf"] == 0.8 and dets[0]["mask"].shape == fr["depth"].shape
    p = SO.observe(fr, dets, SO.parse_args(["--detector", "model", "--model", str(d), "--depth-noise", "none"]))
    b = p["buoys"][0]
    x, y = fr0.to_world(b["buoy_lat"], b["buoy_lon"])
    assert p["detector"] == "model" and b["colour"] == "green" and math.hypot(x - 19.0, y + 5.7) < 0.3


def test_simview_frame_npz(fake_fcu):
    from optiprimex.sim import simview as SV
    a = SV.parse_args(["--world", "pool", "--fcu-control", str(fake_fcu.ctrl_port), "--port", str(free_port()), "--hub-port", "0",
                       "--width", "160", "--height", "90"])
    sim = SV.Sim(a)
    srv = SV.Server(("127.0.0.1", a.port), SV.make_handler(sim))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        fake_fcu.ctrl({"teleport": {"lat": sim.frame.to_latlon(8.0, -6.0)[0], "lon": sim.frame.to_latlon(8.0, -6.0)[1], "hdg": 0.0}})
        raw = urllib.request.urlopen(f"http://127.0.0.1:{a.port}/frame.npz?w=320&h=180", timeout=20).read()
        z = np.load(io.BytesIO(raw))
        meta = json.loads(str(z["meta"]))
        assert z["rgb"].shape == (180, 320, 3) and z["depth"].shape == (180, 320) and z["ids"].shape == (180, 320)
        assert len(z["prim_code"]) == len(z["prim_obj"]) == len(z["prim_kind"]) and "buoy" in set(z["prim_kind"].tolist())
        assert abs(meta["pose"]["x_n"] - 8.0) < 0.2 and meta["camera"]["fx"] == pytest.approx(2317.22 * 320 / 2208, rel=1e-6)
        with pytest.raises(urllib.error.HTTPError):
            urllib.request.urlopen(f"http://127.0.0.1:{a.port}/frame.npz?w=5000&h=180", timeout=5)
    finally:
        srv.shutdown()
        fake_fcu.ctrl({"teleport": {"lat": 1.2810202, "lon": 103.8550097, "hdg": 0.0}})


def test_fake_perception_without_buoys_and_the_launcher():
    from optiprimex.sim import fake_perception as FP
    fp = FP.FakePerception(FP.parse_args(["--world", str(ROOT / "profiles" / "bench" / "world_pool.yaml"), "--skip-kinds", "buoy"]))
    kinds = {o["kind"] for o in fp.objects}
    assert "buoy" not in kinds and {"dock", "window"} <= kinds
    h = subprocess.run(["bash", str(ROOT / "scripts" / "run_sim.sh"), "--help"], capture_output=True, text=True, timeout=10).stdout
    assert "--perception camera|geometric" in h and "--no-ui" in h and "cv/sim_observer" in h and "handover" in h
    r = subprocess.run(["bash", str(ROOT / "scripts" / "run_sim.sh"), "pool", "--perception", "nope"], capture_output=True, text=True, timeout=20)
    assert r.returncode == 2 and "camera | geometric" in r.stderr
    import yaml
    a = yaml.safe_load((ROOT / "profiles" / "bench" / "usv_bench.yaml").read_text())
    b = yaml.safe_load((ROOT / "usv_ui" / "config" / "usv_bench_share.yaml").read_text())
    assert b["mavlink"]["endpoint"] == "udpin:127.0.0.1:14552"
    b["mavlink"]["endpoint"] = a["mavlink"]["endpoint"]
    assert a == b                                                                         # generated: no drift
    for n in ("pool", "marina"):
        wd = yaml.safe_load((ROOT / "profiles" / "bench" / f"world_{n}.yaml").read_text())
        assert all(o.get("body") == "black" for o in wd["objects"] if o.get("light") == "flash")
