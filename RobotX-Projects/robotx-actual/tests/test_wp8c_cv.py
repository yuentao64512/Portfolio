"""ROADMAP WP8c (v0.1.30), CV part: OUR multi-buoy observer's pure core (every buoy of a frame, a colour each from its own pixels)
and cv_publish reading its `buoys` list - while the students' single-buoy files (`measurement` only) still publish as before.
The observer itself needs the ZED and the model (the AGX): `cv/buoys_multi/run.sh --self-test` there, the pool for the rest."""
from __future__ import annotations

import importlib.util
import json
import shutil
import subprocess
import sys

import pytest

from .conftest import ROOT

CV = ROOT / "cv"


def core():
    spec = importlib.util.spec_from_file_location("multi_core", CV / "buoys_multi" / "multi_core.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_colour_from_the_buoys_own_pixels():
    M = core()
    assert M.classify_bgr([(30, 30, 200)] * 150 + [(30, 30, 30)] * 60)[0] == "red"          # red + the dark shadow side
    assert M.classify_bgr([(200, 60, 200)] * 3 + [(40, 30, 190)] * 120)[0] == "red"          # magenta-ish highlights do not win
    assert M.classify_bgr([(70, 170, 40)] * 100)[0] == "green"
    assert M.classify_bgr([(20, 200, 230)] * 100)[0] == "yellow" and M.classify_bgr([(200, 90, 20)] * 100)[0] == "blue"
    assert M.classify_bgr([(15, 15, 15)] * 100)[0] == "black" and M.classify_bgr([(235, 235, 235)] * 100)[0] == "white"
    c, shares = M.classify_bgr([(30, 30, 200)] * 50 + [(70, 170, 40)] * 50)                  # half and half: no colour wins
    assert c == "unknown" and shares["red"] == 0.5
    assert M.classify_bgr([(30, 30, 200)] * 5)[0] == "unknown"                                # too few pixels to vote
    assert M.sample_indices(10) == list(range(10)) and len(M.sample_indices(100000)) == 400


def test_payload_and_cv_publish_read_every_buoy(tmp_path):
    M = core()
    geo = {"latitude_deg": 1.30005, "longitude_deg": 103.80002, "north_offset_m": 5.5, "east_offset_m": 2.2}
    p = M.frame_payload("2026-11-04T08:10:00.000+00:00", 7, [
        M.buoy_record({"range_m": 7.2, "forward_m": 7.0, "right_m": 1.7, "down_m": 0.4, "bearing_deg": 13.6}, 0.81, "green", {}, geo, 1),
        M.buoy_record({"range_m": 5.9, "forward_m": 5.8, "right_m": -1.1, "down_m": 0.4, "bearing_deg": -10.7}, 0.9, "red", {}, geo, 0)], {"lat": 1.3})
    assert p["status"] == "OK" and p["n_buoys"] == 2 and [b["colour"] for b in p["buoys"]] == ["red", "green"]
    assert p["measurement"]["colour"] == "red"                                                   # the old key = the nearest
    assert M.frame_payload("u", 8, [], {})["status"] == "NO_RELIABLE_BUOY"
    kit = tmp_path / "MASTER_SCRIPTS"
    (kit / "cv" / "publish").mkdir(parents=True)
    shutil.copy(CV / "publish" / "cv_publish.py", kit / "cv" / "publish" / "cv_publish.py")
    spec = importlib.util.spec_from_file_location("cv_publish_wp8c", kit / "cv" / "publish" / "cv_publish.py")
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
    topic, env = pub.c.msgs[-1]
    items = env["body"]["items"]
    assert topic == "local/usv/perception/buoys" and [i["colour"] for i in items] == ["red", "green"] and len({i["id"] for i in items}) == 2
    assert items[0]["bearing_deg"] == pytest.approx(-10.7) and items[1]["range_m"] == pytest.approx((7.0 ** 2 + 1.7 ** 2) ** 0.5, abs=1e-3)
    old = {"updated_utc": "2026-11-04T08:10:01.000+00:00", "frame": 9, "status": "OK", "telemetry": {},
           "measurement": {"range_m": 6.0, "forward_m": 6.0, "right_m": 0.0, "down_m": 0.3, "bearing_deg": 0.0, "confidence": 0.7}}
    mod.publish_payload(pub, old)                                                                 # the students' single observer
    assert [i["colour"] for i in pub.c.msgs[-1][1]["body"]["items"]] == ["unknown"]


def test_self_test_and_the_launcher():
    out = subprocess.run([sys.executable, str(CV / "buoys_multi" / "self_test.py")], capture_output=True, text=True, timeout=20)
    assert out.returncode == 0 and '"self_test": "PASS"' in out.stdout, out.stderr
    run = (CV / "buoys_multi" / "run.sh").read_text()
    assert "../buoys/runs" in run and "multi_buoy_observer.py" in run and "--self-test" in run
    obs = (CV / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert "import actual_buoy_observer as S" in obs and "max_buoys" in obs                     # their functions, imported read-only
    for f in (CV / "buoys").glob("*.py"):
        assert not (f.stat().st_mode & 0o222), f"{f.name} must stay read-only (the students' snapshot)"
