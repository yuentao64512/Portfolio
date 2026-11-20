"""ROADMAP WP10 (v0.1.32), the first version of our simulator: the Python renderer (a buoy at a known range / bearing lands at the right
pixel and depth; water, sky, walls), the fake FCU's MANUAL sticks (the arrow keys; MANUAL only; released after 0.5 s), and the
simview server (the camera with cv/stream's interface, the world, the guarded posts, the mockups). Needs requirements-sim.txt."""
from __future__ import annotations

import json
import math
import subprocess
import threading
import time
import urllib.request

import pytest

np = pytest.importorskip("numpy")
pytest.importorskip("PIL")

from optiprimex.sim import render as R  # noqa: E402
from optiprimex.sim import simview as SV  # noqa: E402

from .conftest import ROOT, free_port, wait_until  # noqa: E402


def test_projection_and_the_rendered_buoy():
    cam = R.Camera()
    sc = R.Scene(spheres=[R.Sphere((10.0, 0.0, 0.1), 0.25, R.COLOURS["red"])])
    u, v, z = R.project(cam, 0.0, 0.0, 0.0, (10.0, 0.0, 0.1))
    assert abs(u - cam.cx) < 0.5 and v > cam.cy and abs(z - 10.0) < 1e-4                # ahead (the principal point, v0.1.34: the ZED's cx), below the horizon
    rgb, depth = R.render(sc, 0.0, 0.0, 0.0, cam)
    px = rgb[int(v), int(u)]
    assert px[0] > 80 and px[0] > 3 * px[1] and px[0] > 3 * px[2], px                     # red (shaded) where the buoy projects
    assert abs(float(depth[int(v), int(u)]) - 9.75) < 0.15                               # the near surface: 10 m - r, along the axis
    # turned 90 deg: a buoy east of the boat is dead ahead; one north of it is out of view
    u2, _, z2 = R.project(cam, 0.0, 0.0, 90.0, (0.0, 10.0, 0.1))
    assert abs(u2 - cam.cx) < 0.5 and abs(z2 - 10.0) < 1e-4 and R.project(cam, 0.0, 0.0, 90.0, (-10.0, 0.0, 0.1)) is None
    # a buoy to starboard lands right of centre at atan(right / forward)
    u3, _, _ = R.project(cam, 0.0, 0.0, 0.0, (10.0, 3.0, 0.1))
    assert abs(u3 - (cam.cx + cam.fx * 0.3)) < 0.5
    # water below the horizon (blue), sky above (light grey), a wall where the bounds are
    assert rgb[-5, 5][2] > rgb[-5, 5][0] and rgb[3, 5].min() > 180 and np.isinf(depth[3, 5])
    sc.walls.append(R.Wall((20.0, -30.0), (20.0, 30.0), 1.5))
    rgb2, depth2 = R.render(sc, 0.0, 0.0, 0.0, cam)
    row = int(R.project(cam, 0.0, 0.0, 0.0, (20.0, -5.0, 1.0))[1])
    assert abs(float(depth2[row, 20]) - 20.0) < 0.2 and abs(int(rgb2[row, 20][0]) - int(rgb2[row, 20][2])) < 25   # grey, 20 m


def test_scene_from_the_world_files():
    from optiprimex.sim import world as W
    sc = R.scene_from_world(W.load(str(ROOT / "profiles" / "bench" / "world_pool.yaml")))
    assert len(sc.walls) == 4 and len([s for s in sc.spheres if s.tag.startswith("buoy:")]) >= 10 and sc.boxes
    assert any(s.emissive for s in sc.spheres)                                            # the lit beacons
    dark = R.scene_from_world(W.load(str(ROOT / "profiles" / "bench" / "world_pool.yaml")), lit={1: False})
    assert len([s for s in dark.spheres if s.emissive]) == len([s for s in sc.spheres if s.emissive]) - 1
    boxes = R.scene_from_world(W.load(str(ROOT / "profiles" / "bench" / "world_boxes.yaml")))
    assert len(boxes.boxes) == 3 and {b.colour for b in boxes.boxes} == {R.COLOURS["red"], R.COLOURS["green"], R.COLOURS["blue"]}


def test_sticks_move_the_plant_in_manual_only(fake_fcu):
    fake_fcu.set(mode=4, kill=False, armed=True)
    x0 = fake_fcu.state()["x_n"]
    for _ in range(10):
        fake_fcu.ctrl({"sticks": {"surge": 1.0}})
        time.sleep(0.1)
    assert abs(fake_fcu.state()["x_n"] - x0) < 0.05                                     # HOLD: the sticks do nothing
    fake_fcu.set(mode=0)
    for _ in range(15):
        r = fake_fcu.ctrl({"sticks": {"surge": 1.0, "yaw": 0.0, "sway": 0.0}})
        time.sleep(0.1)
    st = fake_fcu.state()
    assert r["mode"] == 0 and st["x_n"] - x0 > 0.3 and st["sticks"][0] == 1.0
    time.sleep(0.7)                                                                       # released: zero after 0.5 s
    assert fake_fcu.state()["sticks"] == [0.0, 0.0, 0.0]
    fake_fcu.ctrl({"teleport": {"lat": 1.3, "lon": 103.8, "hdg": 0.0}})
    fake_fcu.set(mode=4)


def test_simview_server(fake_fcu):
    a = SV.parse_args(["--world", "pool", "--fcu-control", str(fake_fcu.ctrl_port), "--port", str(free_port()), "--width", "320", "--height", "180"])
    sim = SV.Sim(a)
    sim.poll_pose()
    sim.render_once()
    srv = SV.Server(("127.0.0.1", a.port), SV.make_handler(sim))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{a.port}"

    def post(path, body, header=True):
        req = urllib.request.Request(url + path, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json", **({"X-Sim-Web": "1"} if header else {})})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())
    try:
        assert b"<title>SIM" in urllib.request.urlopen(url + "/", timeout=5).read()
        st = json.loads(urllib.request.urlopen(url + "/status", timeout=5).read())
        assert st["source"] == "sim" and st["frames"] == 1 and st["w"] == 320                  # cv/stream's /status
        assert urllib.request.urlopen(url + "/snapshot.jpg", timeout=5).read()[:2] == b"\xff\xd8"
        w = json.loads(urllib.request.urlopen(url + "/world.json", timeout=5).read())
        assert w["name"] == "world_pool" and w["fcu"]["mode"] is not None and len(w["bounds"]) == 4
        assert post("/fcu", {"set": {"mode": 0}}, header=False)[0] == 403
        assert not post("/fcu", {"set": {"mode": 15}})[1]["ok"]                              # the mode switch: MANUAL / HOLD only
        assert post("/fcu", {"set": {"mode": 0, "armed": False}})[1]["ok"] and fake_fcu.state()["mode"] == 0 and fake_fcu.state()["armed"]   # no arming from here
        code, r = post("/objects", {"op": "add", "kind": "buoy", "colour": "white", "x_n": 6.0, "y_e": 1.0, "size_m": 0.6})
        assert r["ok"]
        new = r["id"]
        assert post("/objects", {"op": "move", "id": new, "x_n": 7.0, "y_e": 1.5})[1]["ok"]
        sim.render_once()
        w = json.loads(urllib.request.urlopen(url + "/world.json", timeout=5).read())
        assert any(o["id"] == new and o["x_n"] == 7.0 for o in w["objects"])
        assert post("/objects", {"op": "del", "id": new})[1]["ok"] and not post("/objects", {"op": "del", "id": new})[1]["ok"]
        assert post("/sticks", {"surge": 0.5})[1]["ok"]
    finally:
        srv.shutdown()
        fake_fcu.set(mode=4)
    h = subprocess.run(["bash", str(ROOT / "scripts" / "run_sim.sh"), "--help"], capture_output=True, text=True, timeout=10).stdout
    assert "simview" in h and "8097" in h and "arrow keys" in h
