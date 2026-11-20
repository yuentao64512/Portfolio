"""ROADMAP WP10b (v0.1.33), sim plumbing: the dataset export for CV_TRAIN (YOLO polygons, exact masks, depth, the truth), RESET SIM,
the teleports (the world's places, saved points, the 2 m snap), the Marina Bay defaults (Course Alpha everywhere), the OCS page's
course view and editable geofence, run_bench / run_sim --judge rctest. Needs requirements-sim.txt for the renderer parts."""
from __future__ import annotations

import json
import subprocess
import threading
import urllib.request

import pytest
import yaml

from .conftest import ROOT, free_port

np = pytest.importorskip("numpy")
pytest.importorskip("PIL")

from optiprimex.sim import dataset as D  # noqa: E402
from optiprimex.sim import marina  # noqa: E402
from optiprimex.sim import simview as SV  # noqa: E402


def test_the_dataset_export(tmp_path):
    """CV_TRAIN's request (docs/REQUEST_dataset_export.md there): flat Ultralytics layout, option-A names, YOLO-seg polygons, stems
    grouped by pose, truth/ per frame, depth as float16 npz, ~10 % negatives, both the pool and the Marina Bay world."""
    from PIL import Image
    out = tmp_path / "ds"
    man = D.export(str(out), 12, ["marina", "pool"], seed=3, w=320, h=180, frames_per_pose=3, negatives=0.25, log=lambda *_: None)
    data = yaml.safe_load((out / "data.yaml").read_text())
    assert data["names"] == {0: "buoy", 1: "dock_face", 2: "window", 3: "box"} and data["train"] == data["val"] == "images"
    assert "12 px" in (out / "data.yaml").read_text() and "20 % visible" in (out / "data.yaml").read_text()
    assert man["frames"]["all"] == 12 and man["poses"] == 4 and len(man["worlds"]) == 2 and len(man["files_sha256"]) == 64
    stems = sorted(p.stem for p in (out / "images").glob("*.jpg"))
    assert len(stems) == 12 and stems[0].startswith("world_marina_s3_p00000_f00") and any(s.startswith("world_pool_s3_p00001_f02") for s in stems)
    n_lab = 0
    for stem in stems:
        t = json.loads((out / "truth" / f"{stem}.json").read_text())
        lines = (out / "labels" / f"{stem}.txt").read_text().splitlines()
        mask = np.array(Image.open(out / "masks" / f"{stem}.png"))
        depth = np.load(out / "depth" / f"{stem}.npz")["depth"]
        assert Image.open(out / "images" / f"{stem}.jpg").size == (320, 180) and depth.dtype == np.float16 and depth.shape == (180, 320)
        assert t["camera"]["w"] == 320 and t["camera"]["hfov_deg"] == pytest.approx(50.95, abs=0.01) and {"x_n", "y_e", "hdg_deg", "pitch_deg", "z_m"} <= set(t["pose"])
        assert len(lines) == len(t["objects"]) and int(mask.max()) == len(t["objects"]) and t["negative"] == (not lines)
        for ln, o in zip(lines, t["objects"]):
            f = ln.split()
            v = [float(x) for x in f[1:]]
            assert int(f[0]) == o["class_id"] and len(v) >= 6 and len(v) % 2 == 0 and all(0.0 <= x <= 1.0 for x in v)
            xs, ys = v[0::2], v[1::2]
            x0, y0, x1, y1 = o["bbox"]                                              # the polygon lies inside the mask's box
            assert min(xs) * 320 >= x0 - 1 and max(xs) * 320 <= x1 + 1 and min(ys) * 180 >= y0 - 1 and max(ys) * 180 <= y1 + 1
            assert int((mask == o["k"]).sum()) == o["area_px"] >= D.MIN_PX and o["visible_frac"] >= D.MIN_VIS
            tr = o["truth"]
            if o["kind"] == "buoy" and tr["depth_median_m"] and tr["range_m"] < 20:   # the depth on the buoy's pixels ~ its forward range
                assert abs(tr["depth_median_m"] - tr["forward_m"]) < 0.6, o
            assert tr["range_m"] >= D.CLEAR_M - 0.6
            n_lab += 1
    assert n_lab > 5 and sum(man["instances"]["all"].values()) == n_lab
    with pytest.raises(SystemExit):                                                 # never overwritten
        D.export(str(out), 1, ["pool"], log=lambda *_: None)
    # the split layout: by pose, never one pose in both
    out2 = tmp_path / "ds2"
    D.export(str(out2), 9, ["pool"], seed=5, w=160, h=90, layout="split", val=0.5, depth_fmt="png", log=lambda *_: None)
    tr = {p.stem.rsplit("_f", 1)[0] for p in (out2 / "images" / "train").glob("*.jpg")}
    va = {p.stem.rsplit("_f", 1)[0] for p in (out2 / "images" / "val").glob("*.jpg")}
    assert tr and not (tr & va) and next((out2 / "depth" / "train").glob("*.png"), None) is not None


def test_negatives_and_the_open_water_world():
    import random
    w = __import__("optiprimex.sim.world", fromlist=["load"]).load(str(ROOT / "profiles" / "bench" / "world_marina.yaml"))
    assert w["walls"] is False and len(w["bounds"]) == 4 and not R_scene(w).walls and R_scene(w).spheres
    sx, sy = w["start"]["x_n"], w["start"]["y_e"]                                  # the course's start: open water behind it
    h = D.negative_heading(w, sx, sy, 100.0, random.Random(1))
    assert h is not None and D.negative_heading(w, 0.0, 0.0, 100.0, random.Random(1)) is None   # mid-course: something everywhere
    from optiprimex.sim import render as R
    cam = R.Camera(w=160, h=90)
    _, _, pid = R.render_ids(R.scene_from_world(w), sx, sy, h, cam)
    assert (pid >= 0).sum() == 0                                                    # a negative frame: nothing of the world in view
    st = D.random_style(random.Random(2))
    rgb, _, _ = R.render_ids(R.scene_from_world(w), 0.0, 0.0, 32.0, cam, style=st)
    rgb0, _, _ = R.render_ids(R.scene_from_world(w), 0.0, 0.0, 32.0, cam)
    assert np.abs(rgb.astype(int) - rgb0.astype(int)).mean() > 2                     # the domain jitter changes the picture


def R_scene(w):
    from optiprimex.sim import render as R
    return R.scene_from_world(w)


def test_outline_and_the_colour_classes():
    m = np.zeros((10, 12), bool)
    m[2:7, 3:9] = True
    m[4, 10] = True                                                                 # a stray pixel: the largest part is traced
    poly = D.polygon(m, 0, 0, 12, 10)
    xs, ys = poly[0::2], poly[1::2]
    assert abs(min(xs) * 12 - 3.5) < 1e-6 and abs(max(xs) * 12 - 8.5) < 1e-6 and abs(min(ys) * 10 - 2.5) < 1e-6 and abs(max(ys) * 10 - 6.5) < 1e-6
    assert D.class_id("buoy", "red", "colour") == D.CLASSES_COLOUR.index("buoy_red") and D.class_id("box", "blue", "colour") == D.CLASSES_COLOUR.index("box")
    assert D.class_id("dock_face", "red", "kind") == 1 and D.class_id("buoy", "orange", "colour") is None


def test_marina_bay_defaults():
    for w in ("pool", "por", "boxes", "bench", "empty", "marina"):
        d = yaml.safe_load((ROOT / "profiles" / "bench" / f"world_{w}.yaml").read_text())
        assert (d["origin"]["lat"], d["origin"]["lon"]) == marina.CENTRE and "start" in d
    for p in ("bench/ocs_bench.yaml", "pool/ocs_pool.yaml", "course/ocs_course.yaml"):
        d = yaml.safe_load((ROOT / "profiles" / p).read_text())
        assert [tuple(c) for c in d["course"]["default"]["boundary"]] == marina.COURSE_ALPHA and d["geofence"] == []
    b = yaml.safe_load((ROOT / "profiles" / "bench" / "usv_bench.yaml").read_text())["frames"]
    assert (b["origin_lat"], b["origin_lon"]) == marina.CENTRE
    from optiprimex.sim import fake_fcu, fake_robocommand
    a = fake_fcu.parse_args([])
    assert (a.lat, a.lon) == marina.CENTRE
    fr = fake_robocommand.FakeRoboCommand.__new__(fake_robocommand.FakeRoboCommand)
    fr.a = type("A", (), {"course_id": "X"})()
    assert [tuple(p) for p in fr.course()["boundary"]] == marina.COURSE_ALPHA
    # Course Alpha: a 30.1 x 25.0 m rectangle, long side on 032 deg, centred on CENTRE
    from optiprimex.sim.world import WorldFrame
    f = WorldFrame({"lat": marina.CENTRE[0], "lon": marina.CENTRE[1]})
    c = [f.to_world(*p) for p in marina.COURSE_ALPHA]
    sides = sorted(round(((c[i][0] - c[i - 1][0]) ** 2 + (c[i][1] - c[i - 1][1]) ** 2) ** 0.5, 1) for i in range(4))
    assert sides[0] == pytest.approx(25.0, abs=0.2) and sides[-1] == pytest.approx(30.1, abs=0.2)
    assert abs(sum(p[0] for p in c) / 4) < 0.5 and abs(sum(p[1] for p in c) / 4) < 0.5


def test_fake_judge_sim_reset():
    from optiprimex.sim import fake_robocommand as F
    fr = F.FakeRoboCommand.__new__(F.FakeRoboCommand)
    fr.lock, fr._times, fr.log = threading.Lock(), __import__("collections").deque([1.0]), lambda *a: None
    fr.declaration, fr.run_started_at, fr.incident_fired, fr.run_id, fr.cmd_seq = {"declaration_seq": 4}, 5.0, True, 3, 9
    fr.stats = {"declarations": 2, "heartbeats": {"USV1": 7}, "reports": [1], "commands": 4}
    fr.inject({"kind": "sim_reset"})
    assert fr.declaration is None and not fr.incident_fired and fr.stats == {"declarations": 0, "heartbeats": {}, "reports": [], "commands": 0}
    assert fr.run_id == 3 and fr.cmd_seq == 9                                         # the sequence numbers keep counting


def test_teleports_and_reset_sim(fake_fcu, tmp_path):
    pts = tmp_path / "points.json"
    from optiprimex.sim.world import WorldFrame
    lat, lon = WorldFrame({"lat": marina.CENTRE[0], "lon": marina.CENTRE[1]}).to_latlon(20.0, 4.0)
    pts.write_text(json.dumps({"HOME": {"lat": lat, "lon": lon, "hdg": 90.0}}))
    a = SV.parse_args(["--world", "pool", "--fcu-control", str(fake_fcu.ctrl_port), "--port", str(free_port()), "--hub-port", "0",
                       "--points", str(pts), "--width", "160", "--height", "90"])
    sim = SV.Sim(a)
    srv = SV.Server(("127.0.0.1", a.port), SV.make_handler(sim))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{a.port}"

    def post(path, body):
        req = urllib.request.Request(url + path, data=json.dumps(body).encode(), method="POST", headers={"X-Sim-Web": "1"})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return dict(json.loads(e.read()), code=e.code)
    try:
        w = json.loads(urllib.request.urlopen(url + "/world.json", timeout=5).read())
        assert [t["name"] for t in w["teleports"]] == ["start", "before gate 1", "abeam bay 2", "N32 E3 dock"]
        assert w["points"][0]["name"] == "HOME" and abs(w["points"][0]["x_n"] - 20.0) < 0.05
        r = post("/teleport", {"name": "abeam bay 2"})
        st = fake_fcu.state()
        assert r["ok"] and abs(st["lat"] - r["lat"]) < 1e-7 and abs(r["x_n"] - 36.0) < 1e-6
        r = post("/teleport", {"x_n": 21.2, "y_e": 4.9, "snap": True})                  # 1.5 m from HOME: onto it, its heading
        assert r["ok"] and r["snapped"] == "HOME" and r["x_n"] == 20.0 and r["hdg"] == 90.0
        r = post("/teleport", {"x_n": 23.0, "y_e": 4.0, "snap": True})                  # 3 m: stays where clicked
        assert r["ok"] and r["snapped"] is None and r["x_n"] == 23.0
        assert post("/teleport", {"name": "nowhere"})["ok"] is False
        # a messy state: killed, E-stop, GPS lost, RC6, a mockup, MANUAL -> RESET SIM
        fake_fcu.set(kill=True, rc5=1900, gps_fix=0, rc6=1900, mode=0)
        assert post("/objects", {"op": "add", "kind": "buoy", "colour": "red", "x_n": 5.0, "y_e": 1.0})["ok"]
        n0 = len(sim.world["objects"])
        assert post("/reset", {})["code"] == 400                                       # typed confirmation
        r = post("/reset", {"confirm": "reset"})
        st = fake_fcu.state()
        assert r["ok"] and not st["kill"] and st["rc5"] < 1700 and st["rc6"] < 1700 and st["gps_fix"] == 3 and st["mode"] == 4
        sim.poll_pose()
        assert abs(sim.pose[0]) < 0.05 and abs(sim.pose[1]) < 0.05 and abs(sim.pose[2]) < 0.5
        assert len(sim.world["objects"]) == n0 - 1 and "skipped" in r["steps"][-1]
    finally:
        srv.shutdown()
        fake_fcu.set(mode=4, kill=False, rc5=1000, rc6=1000, gps_fix=3)


def test_ocs_page_course_view_and_the_geofence(tmp_path):
    from optiprimex.tools import ocs_web as OW
    prof = tmp_path / "ocs.yaml"
    prof.write_text((ROOT / "profiles" / "bench" / "ocs_bench.yaml").read_text())
    v = OW.profile_view(str(prof))
    assert v["default"]["course_id"] == "ALPHA (default)" and v["geofence"] == []
    assert not OW.save_geofence(str(prof), [[1.281, 103.855], [1.2811, 103.855]])["ok"]          # two corners: refused
    assert not OW.save_geofence(str(prof), [[91, 0], [1, 1], [2, 2]])["ok"]
    fence = [list(p) for p in marina.COURSE_ALPHA[:3]]
    r = OW.save_geofence(str(prof), fence)
    assert r["ok"] and r["corners"] == 3 and OW.profile_view(str(prof))["geofence"] == fence
    from optiprimex.companion import config as C
    assert C.load(str(prof)).extra["geofence"] == fence                                # the profile still loads
    assert OW.save_geofence(str(prof), [])["ok"] and OW.profile_view(str(prof))["geofence"] == []
    assert prof.read_text().count("\ngeofence:") == 1                                # the one line, rewritten in place
    assert not OW.sim_post("http://127.0.0.1:9/", "/objects", {})["ok"] and "no simulator" in OW.sim_post("http://127.0.0.1:9/", "/reset", {})["why"]
    a = OW.parse_args(["--ocs-config", str(prof)])
    web = OW.Web(a, env={}, feeds=False)
    web.peek.m.course = {"course_id": "ALPHA", "boundary": [list(p) for p in marina.COURSE_ALPHA]}
    cv = web.course_view()
    assert cv["received"]["course_id"] == "ALPHA" and cv["default"]["course_id"] == "ALPHA (default)"
    html = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert "/course.json" in html and "/geofence" in html and "'/sim'" in html and "snap: true" in html
    from optiprimex.tools.system_actions import ACTIONS
    act = next(x for x in ACTIONS if x.id == "simreset")
    assert act.confirm == "reset" and act.group == "settings" and "sim_reset.sh" in " ".join(act.build({}, {}))


def test_launchers_know_rctest():
    for s in ("run_bench.sh", "run_sim.sh"):
        h = subprocess.run(["bash", str(ROOT / "scripts" / s), "--help"], capture_output=True, text=True, timeout=10).stdout
        assert "--judge" in h and "rctest" in h and "1885" in h, s
    h = subprocess.run(["bash", str(ROOT / "scripts" / "sim_reset.sh"), "--help"], capture_output=True, text=True, timeout=10).stdout
    assert "RESET SIM" in h
    r = subprocess.run(["bash", str(ROOT / "scripts" / "run_bench.sh"), "--judge", "nope"], capture_output=True, text=True, timeout=10)
    assert r.returncode == 2 and "fake | rctest" in r.stderr


def test_sim_day_driver_needs_a_simulator():
    r = subprocess.run([".venv/bin/python", "-m", "optiprimex.tools.sim_day", "--sim", "http://127.0.0.1:9", "--only", "S1"], cwd=ROOT,
                       capture_output=True, text=True, timeout=30)
    assert r.returncode == 2 and "no simulator" in r.stderr
    from optiprimex.tools import sim_day
    assert sim_day.ORDER[0] == "S0" and all(hasattr(sim_day.Walk, s) for s in sim_day.ORDER)
