"""The OCS page's SETTINGS and CAMERA (v0.1.28.1): the bench starts on an empty sim world and a scenario is picked
(scripts/sim_world.sh, SETTINGS -> Sim scenario); fake_perception takes its world off the map when it stops (an empty retained
sim/truth); the AGX camera stream on / off (scripts/agx_cam.sh, the CAMERA toggle; /cam.json asks the stream's /status); the
quick stops need `stop` typed and the boat / AGX ones keep their guards."""
from __future__ import annotations

import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import paho.mqtt.client as mqtt
import yaml

from optiprimex.tools import ocs_web, system_actions
from optiprimex.tools.system_actions import Runner

from .conftest import PY, ROOT, Proc, free_port, wait_until

AUTO_TASK = {"status": {"state": "AUTO_TASK", "armed": True, "fcu_link": "real"}}


def test_the_bench_starts_on_an_empty_world():
    run = (ROOT / "scripts" / "run_bench.sh").read_text()
    assert 'WORLD="${WORLD:-empty}"' in run and "world_$WORLD.yaml" in run and "--world $WORLD" in run
    assert yaml.safe_load((ROOT / "profiles" / "bench" / "world_empty.yaml").read_text())["objects"] == []
    for name in ("empty", "bench", "pool", "por", "boxes"):
        assert (ROOT / system_actions.WORLDS[name]).exists()
    out = subprocess.run(["bash", str(ROOT / "scripts" / "sim_world.sh"), "nowhere.yaml"], capture_output=True, text=True, timeout=10)
    assert out.returncode == 2 and "world file not found" in out.stderr
    out = subprocess.run(["bash", str(ROOT / "scripts" / "sim_world.sh"), "pool"], capture_output=True, text=True, timeout=10,
                         env={"PATH": "/usr/bin:/bin:/opt/homebrew/bin:/usr/local/bin", "OPTIPRIMEX_SESSION": "no-such-bench-session"})
    assert out.returncode == 1 and "no bench running" in out.stderr
    for sh in ("sim_world.sh", "agx_cam.sh"):
        assert "usage" in subprocess.run(["bash", str(ROOT / "scripts" / sh), "--help"], capture_output=True, text=True, timeout=10).stdout


def test_fake_perception_takes_its_world_off_the_map(brokers, tmp_path):
    seen = []
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-truth", protocol=mqtt.MQTTv311)
    c.on_connect = lambda cl, u, f, rc, p=None: cl.subscribe("sim/truth", 0)
    c.on_message = lambda cl, u, m: seen.append(m.payload)
    c.connect("127.0.0.1", brokers.hub_port)
    c.loop_start()
    fp = Proc([PY, "-m", "optiprimex.sim.fake_perception", "--world", str(ROOT / "profiles" / "bench" / "world_empty.yaml"),
               "--port", str(brokers.boat_port), "--hub-port", str(brokers.hub_port)], tmp_path / "fp.log")
    try:
        assert wait_until(lambda: any(seen), 10), fp.log()
        assert json.loads(seen[-1])["body"]["objects"] == [] and json.loads(seen[-1])["body"]["name"] == "empty"
        fp.kill()
        assert wait_until(lambda: seen[-1] == b"", 5), "no empty sim/truth on exit"
        later = []                                          # the hub keeps nothing retained: a new viewer sees no world
        c2 = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-truth2", protocol=mqtt.MQTTv311)
        c2.on_connect = lambda cl, u, f, rc, p=None: cl.subscribe("sim/truth", 0)
        c2.on_message = lambda cl, u, m: later.append(m.payload)
        c2.connect("127.0.0.1", brokers.hub_port)
        c2.loop_start()
        time.sleep(1.0)
        c2.loop_stop()
        assert later == []
    finally:
        fp.kill()
        c.loop_stop()
    html = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert "env === null && topic === T_TRUTH" in html


def test_settings_actions_stops_and_camera_guards():
    r = Runner(env={"ORIN_SSH": "nx-host", "AGX_SSH": "agx-host"}, dry_run=True, log_path="/dev/null")
    groups = {a.id: a.group for a in system_actions.ACTIONS}
    assert [i for i, g in groups.items() if g == "stop"] == ["stop_bench", "stop_simcv", "stop_nx", "stop_agx", "stop_ocs"]
    for sid in ("stop_bench", "stop_simcv", "stop_nx", "stop_agx", "stop_ocs"):
        assert r.run(sid, {}, "yes", {})["why"] == "type 'stop' to confirm"
        assert r.run(sid, {}, "STOP", {})["ok"], sid
    cmds = [j.public()["cmd"] for j in r.jobs]
    assert "scripts/stop_bench.sh" in cmds[0] and "sim_world.sh off" in cmds[1] and "nx-host" in cmds[2] and "stop_usv.sh" in cmds[2]
    assert "agx-host" in cmds[3] and "stop_agx.sh" in cmds[3] and "stop_ocs.sh" in cmds[4]
    assert system_actions.BY_ID["stop_ocs"].detach and system_actions.BY_ID["ocs"].detach and not system_actions.BY_ID["stop_nx"].detach
    # the boat and the AGX keep their guards; the Mac's stops do not need any
    assert r.run("stop_nx", {}, "stop", AUTO_TASK)["why"].startswith("refused: the vehicle is AUTO_TASK")
    assert r.run("stop_agx", {}, "stop", AUTO_TASK)["why"].startswith("refused: a task is running")
    assert r.run("stop_bench", {}, "stop", AUTO_TASK)["ok"] and r.run("stop_ocs", {}, "stop", AUTO_TASK)["ok"]
    # the sim scenario and the camera
    assert r.run("simworld", {"world": "boxes"}, "", {})["ok"] and r.jobs[-1].public()["cmd"].endswith("scripts/sim_world.sh boxes'")
    assert r.run("simworld", {"world": "x; rm -rf ~"}, "", {})["ok"] and r.jobs[-1].public()["cmd"].endswith("scripts/sim_world.sh empty'")
    assert r.run("cam", {"do": "start", "source": "test"}, "", {})["ok"] and "agx_cam.sh start --source test" in r.jobs[-1].public()["cmd"]
    assert r.run("cam", {"do": "start"}, "", {})["ok"] and "agx_cam.sh start --source auto" in r.jobs[-1].public()["cmd"]    # v0.1.28.2
    assert r.run("cam", {"do": "stop"}, "", AUTO_TASK)["ok"] and "agx_cam.sh stop" in r.jobs[-1].public()["cmd"]
    assert "takes the ZED" in r.run("cam", {"do": "start", "source": "zed"}, "", AUTO_TASK)["why"]
    assert r.run("cam", {"do": "start", "source": "dir"}, "", AUTO_TASK)["ok"]


def test_cam_json_asks_the_stream(tmp_path):
    class Cam(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            body = json.dumps({"source": "test", "frames": 42, "age_s": 0.2, "clients": 1}).encode()
            self.send_response(200 if self.path == "/status" else 404)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

    srv = HTTPServer(("127.0.0.1", free_port()), Cam)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        web = ocs_web.Web(ocs_web.parse_args([]), env={"CAM_URL": f"http://127.0.0.1:{srv.server_address[1]}/"}, feeds=False)
        st = web.cam_status()
        assert st["ok"] and st["status"]["frames"] == 42
        srv.shutdown()
        srv.server_close()
        st = web.cam_status()
        assert not st["ok"] and "stream down or AGX unreachable" in st["why"]
        real = ocs_web.lan_agx_host
        ocs_web.lan_agx_host = lambda root=None: ""                  # not the tree's own network.lan.env
        try:
            assert "no camera address" in ocs_web.Web(ocs_web.parse_args([]), env={}, feeds=False).cam_status()["why"]
        finally:
            ocs_web.lan_agx_host = real
    finally:
        srv.server_close()
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'id="c-on"' in html and "/cam.json" in html and "⚙ SETTINGS" in html and 'id="stops"' in html


def test_agx_cam_status_sees_only_the_stream_itself():
    """v0.1.28.2: `status` used to say up whenever any command line mentioned cam_stream (a caller's ssh / pgrep); now only a python
    running .../stream/cam_stream.py counts. `auto` is a source; an unknown one is refused."""
    sh = str(ROOT / "scripts" / "agx_cam.sh")
    out = subprocess.run(["bash", "-c", f"true cam_stream cam_stream.py; bash {sh} status"], capture_output=True, text=True, timeout=10)
    assert out.stdout.strip() == "camera stream down", out.stdout
    fake = subprocess.Popen([PY, "-c", "import time; time.sleep(20)", "/x/cv/stream/cam_stream.py"])
    try:
        assert wait_until(lambda: "camera stream up" in subprocess.run(["bash", sh, "status"], capture_output=True, text=True, timeout=10).stdout, 5)
    finally:
        fake.kill()
        fake.wait()
    bad = subprocess.run(["bash", sh, "start", "--source", "webcam"], capture_output=True, text=True, timeout=10)
    assert bad.returncode == 2 and "auto|dir|zed|test" in bad.stderr
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert '<option value="auto">' in html and "blank without one" in html


def test_the_camera_is_tried_on_the_lan_first(tmp_path):
    """v0.1.29.2: the camera address from network.lan.env first (the pool switch), the profile's (tailnet) second; CAM_URL alone."""
    (tmp_path / "network.lan.env").write_text("OCS_HOST=\nAGX_HOST=192.168.1.99   # the AGX on the switch\n")
    assert ocs_web.lan_agx_host(str(tmp_path)) == "192.168.1.99"
    assert ocs_web.cam_urls({"AGX_HOST": "100.114.195.19"}, str(tmp_path)) == ["http://192.168.1.99:8090/", "http://100.114.195.19:8090/"]
    assert ocs_web.cam_urls({"AGX_HOST": "192.168.1.99", "CAM_PORT": "8091"}, str(tmp_path)) == ["http://192.168.1.99:8091/"]
    assert ocs_web.cam_urls({"CAM_URL": "http://x:1/", "AGX_HOST": "y"}, str(tmp_path)) == ["http://x:1/"]
    assert ocs_web.cam_urls({}, str(tmp_path / "none")) == []
    web = ocs_web.Web(ocs_web.parse_args([]), env={"CAM_URL": "http://127.0.0.1:9/"}, feeds=False)
    web._meta["cam_urls"] = ["http://127.0.0.1:9/", "http://127.0.0.1:10/"]
    st = web.cam_status()
    assert not st["ok"] and "127.0.0.1:9" in st["why"] and "127.0.0.1:10" in st["why"]
