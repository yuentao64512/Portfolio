"""v0.1.60 - WP10m, the OCS CAMERA tab (Dmitrii 2026-11-07: depth only when no detector holds the camera; JPEG frames + jsonl;
the roots AGX recordings / Mac logs / CV_TRAIN staging; its own tab). depthviz, cam_stream's modes and depth (test / dir sources),
the simulator's /depth.jpg + /depth_at, the frame recorder (allow-list, new frames only, jsonl, session.json, max_min, snapshot),
ocs_web's CamTab (the camera list per vehicle, the proxies, the Mac and AGX recorders), the page's tab."""
from __future__ import annotations

import json
import os
import re
import threading
import time
import urllib.error
import urllib.request

import numpy as np
import pytest

from optiprimex.tools import depthviz, frame_rec

from .conftest import PY, ROOT, Proc, free_port, wait_until

CAM = ROOT / "cv" / "stream" / "cam_stream.py"


def _get(url, timeout=3.0):
    with urllib.request.urlopen(url, timeout=timeout) as r:
        return r.read()


def _post(url, body):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST", headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _ok(url):
    try:
        _get(url, 1.0)
        return True
    except Exception:
        return False


# ---- depthviz ---------------------------------------------------------------------------------------------------------------------
def test_depth_colours_and_the_range_under_the_cursor():
    d = np.array([[0.5, 25.0, np.inf], [12.75, np.nan, 0.0]], np.float32)
    c = depthviz.colourise(d)
    assert c.shape == (2, 3, 3) and c.dtype == np.uint8
    assert tuple(c[0, 0]) == (220, 40, 30) and tuple(c[0, 1]) == (40, 60, 200)        # near red, far blue
    assert tuple(c[0, 2]) == (0, 0, 0) and tuple(c[1, 1]) == (0, 0, 0) and tuple(c[1, 2]) == (0, 0, 0)   # no return = black
    big = np.full((100, 200), 7.0, np.float32)
    big[50, 100] = 99.0                                                               # one wild pixel: the 5x5 median ignores it
    assert depthviz.depth_at(big, 0.5, 0.5) == 7.0 and depthviz.depth_at(big, 1.5, 0.5) is None
    assert depthviz.depth_at(np.full((10, 10), np.inf), 0.5, 0.5) is None and depthviz.depth_at(None, 0.5, 0.5) is None
    sb = depthviz.scale_bar()
    assert sb["near_m"] == 0.5 and sb["far_m"] == 25.0 and sb["stops"][0] == [0.0, "#dc281e"]


# ---- cam_stream: modes ------------------------------------------------------------------------------------------------------------
def test_cam_stream_depth_on_the_test_source(tmp_path):
    port = free_port()
    p = Proc([PY, str(CAM), "--source", "test", "--port", str(port), "--bind", "127.0.0.1", "--hz", "10", "--width", "320"], tmp_path / "cam.log")
    base = f"http://127.0.0.1:{port}"
    try:
        assert wait_until(lambda: _ok(base + "/status"), 10), p.log()
        st = json.loads(_get(base + "/status"))
        assert st["mode"] == "rgb" and st["depth"]["available"] and st["modes"] == ["rgb", "depth", "both"]
        assert _post(base + "/mode", {"mode": "sideways"})[0] == 400
        code, r = _post(base + "/mode", {"mode": "both"})
        assert code == 200 and r == {"ok": True, "mode": "both"}
        assert wait_until(lambda: _ok(base + "/depth.jpg"), 5)
        assert _get(base + "/depth.jpg")[:2] == b"\xff\xd8" and _get(base + "/snapshot.jpg")[:2] == b"\xff\xd8"
        top = json.loads(_get(base + "/depth_at?x=0.5&y=0.05"))
        low = json.loads(_get(base + "/depth_at?x=0.5&y=0.95"))
        assert top["range_m"] is None and 1.0 <= low["range_m"] < 3.0                     # the sky: no return; the bottom: near
        assert json.loads(_get(base + "/status"))["mode"] == "both"
        assert _post(base + "/mode", {"mode": "rgb"})[0] == 200
    finally:
        p.kill()


def test_cam_stream_no_depth_while_a_detector_holds_the_camera(tmp_path):
    port = free_port()
    (tmp_path / "latest_preview.jpg").write_bytes(b"\xff\xd8" + b"x" * 100 + b"\xff\xd9")
    p = Proc([PY, str(CAM), "--source", "dir", "--dir", str(tmp_path / "*.jpg"), "--port", str(port), "--bind", "127.0.0.1"], tmp_path / "cam.log",
             env={"CAM_NO_OPENCV": "1"})
    base = f"http://127.0.0.1:{port}"
    try:
        assert wait_until(lambda: _ok(base + "/status"), 10), p.log()
        st = json.loads(_get(base + "/status"))
        assert st["modes"] == ["rgb"] and not st["depth"]["available"] and "a detector holds the camera" in st["depth"]["why"]
        code, r = _post(base + "/mode", {"mode": "depth"})
        assert code == 409 and "depth only when no detector runs" in r["why"]
        with pytest.raises(urllib.error.HTTPError):
            _get(base + "/depth.jpg")
    finally:
        p.kill()


# ---- the simulator --------------------------------------------------------------------------------------------------------------
def test_the_simulator_gives_depth(fake_fcu):
    from optiprimex.sim import simview as SV
    a = SV.parse_args(["--world", "pool", "--fcu-control", str(fake_fcu.ctrl_port), "--port", str(free_port()), "--width", "320", "--height", "180"])
    sim = SV.Sim(a)
    sim.poll_pose()
    sim.render_once()
    srv = SV.Server(("127.0.0.1", a.port), SV.make_handler(sim))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{a.port}"
    try:
        st = json.loads(_get(url + "/status"))
        assert st["modes"] == ["rgb", "depth", "both"] and st["depth"]["available"] and st["mode"] == "rgb"
        req = urllib.request.Request(url + "/mode", data=b'{"mode": "both"}', method="POST",
                                     headers={"Content-Type": "application/json", "X-Sim-Web": "1"})
        assert json.loads(urllib.request.urlopen(req, timeout=5).read()) == {"ok": True, "mode": "both"}
        assert json.loads(_get(url + "/status"))["mode"] == "both"
        assert _get(url + "/depth.jpg")[:2] == b"\xff\xd8"
        water = json.loads(_get(url + "/depth_at?x=0.5&y=0.95"))                       # the water just ahead of the bow
        sky = json.loads(_get(url + "/depth_at?x=0.5&y=0.02"))
        assert water["ok"] and water["range_m"] is not None and water["range_m"] < 30 and sky["range_m"] is None
        with pytest.raises(urllib.error.HTTPError):
            _get(url + "/depth_at?x=abc&y=1")                                       # 400: not numbers
    finally:
        srv.shutdown()
        srv.server_close()


# ---- the frame recorder -----------------------------------------------------------------------------------------------------------
class FakeCam:
    def __init__(self):
        self.k = 0
        self.same = False

    def __call__(self, url):
        if url.endswith("/depth.jpg"):
            return b"\xff\xd8depth"
        if not self.same:
            self.k += 1
        return b"\xff\xd8frame%d" % self.k


def test_the_recorder_writes_new_frames_jsonl_and_the_session(tmp_path):
    env = {"CV_STAGING_DIR": str(tmp_path / "staging")}
    cam = FakeCam()
    rec = frame_rec.Recorder(lambda: "http://cam/", dets_fn=lambda: {"state": "live", "dets": [{"cls": "buoy"}]},
                             pose_fn=lambda: {"lat": 1.0, "lon": 2.0}, env=env, fetch=cam)
    with pytest.raises(ValueError):
        rec.start("bad label!", "cv_staging")
    with pytest.raises(ValueError):
        rec.start("ok", "/tmp")                                                    # a typed path is refused
    with pytest.raises(ValueError):
        frame_rec.root_path("../../etc")
    st = rec.start("day1_gate", "cv_staging", hz=15, max_min=0, depth=True)
    assert st["state"] == "recording" and st["dir"].startswith(str(tmp_path / "staging"))
    assert re.search(r"/\d{4}-\d{2}-\d{2}/\d{6}_day1_gate$", st["dir"])
    with pytest.raises(RuntimeError):
        rec.start("again", "cv_staging")                                           # one at a time
    assert wait_until(lambda: rec.status()["frames"] >= 4, 5)
    cam.same = True                                                                # a stalled camera: nothing new is written
    time.sleep(0.4)
    n = rec.status()["frames"]
    time.sleep(0.4)
    assert rec.status()["frames"] == n and rec.status()["repeats"] > 0
    s = rec.stop()
    assert s["state"] == "idle" and s["why"] == "stopped" and s["frames"] == n
    d = s["dir"]
    lines = [json.loads(x) for x in open(os.path.join(d, "frames.jsonl"))]
    assert len(lines) == n and lines[0]["file"] == "frame_000001.jpg" and lines[0]["depth_file"] == "depth_000001.jpg"
    assert lines[0]["dets"]["dets"][0]["cls"] == "buoy" and lines[0]["pose"] == {"lat": 1.0, "lon": 2.0}
    assert os.path.isfile(os.path.join(d, "frame_000001.jpg")) and os.path.isfile(os.path.join(d, "depth_000001.jpg"))
    meta = json.load(open(os.path.join(d, "session.json")))
    assert meta["frames"] == n and meta["label"] == "day1_gate" and "jpeg frames" in meta["format"]
    ss = frame_rec.sessions("cv_staging", env=env)
    assert ss[0]["label"] == "day1_gate" and ss[0]["frames"] == n
    snap = rec.snapshot("buoy_close", "cv_staging")
    assert snap["ok"] and "/snapshots/" in snap["file"] and snap["file"].endswith("_buoy_close.jpg")


def test_the_recorder_stops_at_max_min(tmp_path):
    t = {"now": 1000.0}
    rec = frame_rec.Recorder(lambda: "http://cam/", env={"CV_STAGING_DIR": str(tmp_path)}, fetch=FakeCam(), clock=lambda: t["now"])
    rec.start("short", "cv_staging", hz=15, max_min=1)
    t["now"] += 61
    assert wait_until(lambda: rec.status()["state"] == "idle", 3)
    assert rec.status()["why"] == "max 1 min"


def test_the_roots_are_the_allow_list():
    assert set(frame_rec.ROOT_INFO) == {"mac_logs", "cv_staging", "agx_rec"}
    assert frame_rec.root_path("mac_logs").endswith("logs/recordings")
    assert frame_rec.root_path("cv_staging", {}).endswith("Documents/BH1/RobotX/recordings")      # outside CV_TRAIN
    assert "CV_TRAIN" not in frame_rec.root_path("cv_staging", {})
    assert frame_rec.root_path("agx_rec", {"AGX_REC_DIR": "/x/rec"}) == "/x/rec"
    sh = (ROOT / "scripts" / "agx_frames.sh").read_text()
    assert "frame_rec" in sh and "--root agx_rec" in sh and "agxframes" in sh
    assert "agx_frames.sh" in (ROOT / "scripts" / "stop_agx.sh").read_text()


# ---- ocs_web: CamTab ----------------------------------------------------------------------------------------------------------------
def _web(tmp_path, cam_url):
    import argparse
    from optiprimex.tools import ocs_web as OW
    a = argparse.Namespace(vehicle_id="USV1", vehicle="usv", host="127.0.0.1", port=1, boat_host="127.0.0.1", boat_port=1, judge_host=None,
                           judge_port=None, team="SGIT", ocs_config=None, fcu_control=0)
    try:
        web = OW.Web(a, env={"CAM_URL": cam_url, "CV_STAGING_DIR": str(tmp_path / "st"), "AGX_SSH": "agx-test"}, feeds=False)
    except Exception as exc:
        pytest.skip(f"ocs_web.Web without feeds: {exc}")
    return web, OW


def test_camtab_view_mode_proxy_and_recorders(tmp_path, monkeypatch):
    port = free_port()
    p = Proc([PY, str(CAM), "--source", "test", "--port", str(port), "--bind", "127.0.0.1", "--hz", "10", "--width", "320"], tmp_path / "cam.log")
    try:
        assert wait_until(lambda: _ok(f"http://127.0.0.1:{port}/status"), 10), p.log()
        web, OW = _web(tmp_path, f"http://127.0.0.1:{port}/")
        v = web.camtab.view()
        assert v["vehicles"]["usv"]["ok"] and v["vehicles"]["usv"]["status"]["source"] == "test"
        assert not v["vehicles"]["uav"]["ok"] and "WP14" in v["vehicles"]["uav"]["why"]
        roots = {r["id"]: r for r in v["roots"]}
        assert roots["mac_logs"]["ok"] and roots["cv_staging"]["ok"] and roots["agx_rec"]["ok"]       # a real (non-sim) camera
        assert web.camtab.set_mode("depth") == {"ok": True, "mode": "depth"}
        assert wait_until(lambda: web.camtab.depth_at("0.5", "0.95").get("range_m") is not None, 5)
        r = web.camtab.record({"op": "start", "root": "/etc", "label": "x"})
        assert not r["ok"] and "typed path is refused" in r["why"]
        calls = []
        web.camtab.ssh_run = lambda cmd: calls.append(cmd) or '{"state": "recording", "label": "agx1", "frames": 0}'
        r = web.camtab.record({"op": "start", "root": "agx_rec", "label": "agx1", "hz": 3, "max_min": 5, "depth": True})
        assert r["ok"] and calls[-1] == "scripts/agx_frames.sh start agx1 --hz 3 --max-min 5 --depth"
        assert web.camtab.record({"op": "stop", "root": "agx_rec"})["ok"] and calls[-1] == "scripts/agx_frames.sh stop"
        assert web.camtab.record({"op": "start", "root": "agx_rec", "label": "x; rm -rf /"})["ok"] is False   # the label is checked first
        r = web.camtab.record({"op": "start", "root": "cv_staging", "label": "mac1", "hz": 10, "max_min": 0})
        assert r["ok"] and r["local"]["state"] == "recording"
        assert wait_until(lambda: web.camtab.rec.status()["frames"] >= 2, 5)
        r = web.camtab.record({"op": "stop", "root": "cv_staging"})
        assert r["ok"] and r["local"]["frames"] >= 2
        assert web.camtab.view()["sessions"]["cv_staging"][0]["label"] == "mac1"
        assert web.camtab.record({"op": "snapshot", "root": "cv_staging", "label": "s1"})["ok"]
    finally:
        p.kill()


def test_the_camera_tab_on_the_page():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert '"OVERVIEW","MISSION","CAMERA","USV"' in page and 'CAMERA:"MISSION"' not in page
    sec = page.split('<section class="tab" id="t-CAMERA">')[1].split("</section>")[0]
    for needle in ('id="cx-veh"', 'data-v="uav"', 'data-m="depth"', 'data-m="both"', 'id="cx-img"', 'id="cx-dimg"', 'id="cx-ov"',
                   'id="cx-preset"', 'id="cx-lab"', 'id="cx-root"', 'id="cx-start"', 'id="cx-stop"', 'id="cx-snap"', 'id="cx-scale"'):
        assert needle in sec, needle
    assert "aspect-ratio:16/9" in page                                                 # the picture keeps its place: no jump
    for needle in ('post("/cam_mode"', 'fetch("/camtab.json"', "/cam_depth_at?x=", 'post("/camrec"', 'e.code==="Space"',
                   'ask("<b>stop recording?</b>"'):
        assert needle in page, needle
    src = (ROOT / "optiprimex" / "tools" / "ocs_web.py").read_text()
    for route in ('"/camtab.json"', '"/cam_depth_at"', '"/cam_mode"', '"/camrec"'):
        assert route in src, route
