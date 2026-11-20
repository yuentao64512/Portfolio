"""cv/ kit (MASTER_SCRIPTS/cv): the cannon link and cv_publish's dock reader, with no camera, GPU or key. Skipped when
the cv/ folder is missing from the tree. The ZED / YOLO programs themselves can only run on the AGX (their --self-test)."""
from __future__ import annotations

import hashlib
import hmac
import json
import math
import os
import socket
import subprocess
import sys
import time

import pytest

from .conftest import ROOT, free_udp_port

CV = ROOT / "cv"
pytestmark = pytest.mark.skipif(not CV.exists(), reason="cv/ kit missing from the tree")


def _verify_like_the_pi(packet: bytes, key: bytes) -> bool:
    """The students' receiver-side check (dock_mapper_sender.verify_packet), verbatim."""
    envelope = json.loads(packet)
    raw = json.dumps(envelope["body"], separators=(",", ":"), sort_keys=True).encode()
    return hmac.compare_digest(hmac.new(key, raw, hashlib.sha256).hexdigest(), envelope["hmac_sha256"])


def test_cannon_sender_self_test_and_packets(tmp_path):
    out = subprocess.run([sys.executable, str(CV / "cannon" / "cannon_sender.py"), "--self-test"], capture_output=True, text=True, timeout=30)
    assert out.returncode == 0 and '"self_test": "PASS"' in out.stdout, out.stdout + out.stderr
    key = "k" * 40
    key_file = tmp_path / "key"
    key_file.write_text(key + "\n")
    status = tmp_path / "latest_status.json"
    port = free_udp_port()
    cfg = {"water_cannon": {"forward_from_camera_m": 1.0, "right_from_camera_m": 0.5},
           "network": {"pi_ip": "127.0.0.1", "pi_port": port, "key_file": str(key_file)},
           "status_file": str(status), "max_rate_hz": 20, "stale_after_s": 0.5}
    cfg_file = tmp_path / "cannon_config.json"
    cfg_file.write_text(json.dumps(cfg))
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", port))
    rx.settimeout(5)
    p = subprocess.Popen([sys.executable, str(CV / "cannon" / "cannon_sender.py"), "--config", str(cfg_file), "--source", "file"], stdout=subprocess.PIPE, text=True)
    try:
        time.sleep(0.3)
        for i in range(3):
            body = {"protocol": "dock_observer.v1", "sequence": i, "sent_utc": "2026-10-29T00:00:00+00:00", "status": "OK",
                    "target": {"confidence": 0.9, "range_m": 8.0, "forward_m": 7.9, "right_m": 1.0, "down_m": 0.5, "body_forward_m": 7.9,
                               "body_right_m": 1.0, "fcu_forward_m": 7.9, "fcu_right_m": 1.0, "latitude_deg": 1.3, "longitude_deg": 103.8,
                               "north_offset_m": 7.9, "east_offset_m": 1.0, "vehicle_latitude_deg": 1.3, "vehicle_longitude_deg": 103.8,
                               "vehicle_heading_deg": 0.0, "red_fraction": 0.3}}
            (tmp_path / "s.tmp").write_text(json.dumps(body))
            os.replace(tmp_path / "s.tmp", status)
            time.sleep(0.15)
        got = []
        for _ in range(3):
            pkt, _ = rx.recvfrom(65535)
            assert _verify_like_the_pi(pkt, key.encode())
            got.append(json.loads(pkt)["body"])
        t = got[-1]["target"]
        assert got[-1]["protocol"] == "dock_cannon.v1" and t["class_name"] == "dock" and t["color"] == "red"
        assert t["y_forward_m"] == pytest.approx(6.9) and t["x_right_m"] == pytest.approx(0.5)
        assert t["cannon_bearing_deg"] == pytest.approx(math.degrees(math.atan2(0.5, 6.9)))
        stale, _ = rx.recvfrom(65535)
        assert json.loads(stale)["body"]["status"] == "OBSERVER_STALE"
    finally:
        p.terminate()
        p.wait(5)
    last, _ = rx.recvfrom(65535)
    assert json.loads(last)["body"]["status"] == "SENDER_STOPPED"
    rx.close()


def test_cv_publish_dock_reader_and_network_env(tmp_path, monkeypatch):
    """v0.1.26: on a copy of cv_publish.py beside a synthetic network.env + network.ts.env - the archive never carries the real
    address files, so a fresh extract has none (WP7 test hygiene)."""
    import importlib.util
    import shutil
    kit = tmp_path / "MASTER_SCRIPTS"
    (kit / "cv" / "publish").mkdir(parents=True)
    shutil.copy(CV / "publish" / "cv_publish.py", kit / "cv" / "publish" / "cv_publish.py")
    (kit / "network.env").write_text("NET_PROFILE=ts\nBOAT_PORT=1884\nAGX_OBSERVER_LOGS=/x/MASTER_SCRIPTS/cv/buoys/runs   # comment\n"
                                     "AGX_DOCK_LATEST=/x/MASTER_SCRIPTS/cv/docking/runs/latest\n")
    (kit / "network.ts.env").write_text("BOAT_HOST=100.64.0.2   # the NX on a test tailnet\n")
    monkeypatch.delenv("NET_PROFILE", raising=False)
    spec = importlib.util.spec_from_file_location("cv_publish", kit / "cv" / "publish" / "cv_publish.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    env = mod.read_network_env()
    assert env.get("_path", "").endswith("network.env") and env.get("BOAT_HOST") == "100.64.0.2" and env.get("_overlay", "").endswith("network.ts.env")
    assert env.get("AGX_OBSERVER_LOGS", "").endswith("cv/buoys/runs") and env.get("AGX_DOCK_LATEST", "").endswith("cv/docking/runs/latest")

    class Sink:                       # stands in for the MQTT client: records what would be published
        def __init__(self):
            self.msgs = []

        def publish(self, topic, payload, qos=0, retain=False):
            self.msgs.append((topic, json.loads(payload)))

    pub = mod.Publisher(mod.main.__globals__["argparse"].Namespace(mqtt="127.0.0.1:1884", vehicle="usv", vehicle_id="USV1", camera="zed2i_left",
                                                                     detector="dock_observer", colour="unknown"))
    pub.c = Sink()
    status = {"protocol": "dock_observer.v1", "sequence": 4, "sent_utc": "2026-10-29T01:02:03.000+00:00", "status": "OK",
              "target": {"confidence": 0.93, "range_m": 6.1, "forward_m": 6.0, "right_m": 1.0, "down_m": 0.7, "bearing_deg": 9.46,
                         "body_forward_m": 6.0, "body_right_m": 1.0, "red_fraction": 0.4, "color": "red",
                         "vehicle_latitude_deg": 1.3, "vehicle_longitude_deg": 103.8, "vehicle_heading_deg": 2.0}}
    mod.publish_dock_payload(pub, status)
    topic, env = pub.c.msgs[-1]
    assert topic == "local/usv/perception/dock" and env["kind"] == "local"
    item = env["body"]["items"][0]
    assert item["class"] == "dock" and item["colour"] == "red" and item["range_m"] == pytest.approx(math.hypot(6.0, 1.0), abs=1e-3)
    assert item["bearing_deg"] == pytest.approx(9.46) and env["body"]["pose_hint"]["hdg_deg"] == 2.0
    mod.publish_dock_payload(pub, {"status": "NO_DOCK", "sequence": 5, "sent_utc": "2026-10-29T01:02:04+00:00", "target": None})
    assert pub.c.msgs[-1][1]["body"]["items"] == []
