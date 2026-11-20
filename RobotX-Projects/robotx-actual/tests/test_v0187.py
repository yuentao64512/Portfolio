"""v0.1.87 (Dmitrii 2026-11-12: "our CV model is ready ... buoy detection with a colour and obstacle detection with boundaries"):
`--detector boundary-v14` in our observer (cv/buoys_multi) - the colleague's boundary-v14 YOLO11m, read-only from ~/buoy_robotx: its
red / green / blue / unlit boundary buoys through the Model A adapter, its obstacle boxes measured with the ZED depth and handed to the
boat as the guard's `far` clusters (the boat already rings those); the detector choice on the page, the launcher and the AGX script."""
from __future__ import annotations

import importlib.util
import math
import subprocess

import pytest

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin
from optiprimex.companion.state import LocalPose
from optiprimex.tools import launcher as L
from optiprimex.tools import system_actions as SA
from optiprimex.usv.guard import GuardSupervisor
from optiprimex.usv.perception_map import PerceptionMap

from .conftest import ROOT

CV = ROOT / "cv"
V14 = {0: "buoy_red", 1: "buoy_green", 2: "buoy_blue", 3: "buoy_boundary_no_light", 4: "dock", 5: "obstacle"}


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


TS = load("two_stage_core_v0187", CV / "buoys_multi" / "two_stage_core.py")


def test_v14_classes_buoys_and_the_name_check():
    assert TS.buoy_class_ids(V14) == [0, 1, 2, 3]                      # dock, obstacle are not buoys
    assert TS.class_ids(V14, ("obstacle",)) == [5] and TS.class_ids(V14, ("obstacle", "dock")) == [4, 5]
    TS.check_names("boundary-v14", V14)
    TS.check_names("boundary-v14", list(V14.values()))
    with pytest.raises(SystemExit):
        TS.check_names("boundary-v14", {0: "buoy_led_red", 1: "dock"})  # another file under the same name stops the observer
    TS.check_names("model-a", {0: "buoy_led_red"})                     # Model A is not checked here


@pytest.mark.parametrize("mode,cls,conf,hsv,want", [
    ("auto", "buoy_blue", 0.6, "white", ("blue", "model")),            # the model's colour when confident (blue: Task 1's entry / exit)
    ("auto", "buoy_red", 0.3, "green", ("green", "hsv")),              # low score -> our vote
    ("auto", "buoy_boundary_no_light", 0.9, "white", ("white", "hsv")),
    ("model", "buoy_boundary_no_light", 0.9, None, ("black", "model")),  # unlit -> cv_publish lit = False
    ("model", "buoy_green", 0.1, None, ("green", "model")),
    ("hsv", "buoy_red", 0.99, "green", ("green", "hsv")),               # the backup
])
def test_v14_colour_precedence(mode, cls, conf, hsv, want):
    assert TS.pick_colour(mode, cls, conf, hsv, led_conf=0.5) == want


def test_obstacle_classes_option():
    assert TS.parse_classes("obstacle", "boundary-v14") == ("obstacle",)
    assert TS.parse_classes("obstacle, dock", "boundary-v14") == ("obstacle", "dock")
    assert TS.parse_classes("none", "boundary-v14") == () and TS.parse_classes("", "boundary-v14") == ()
    with pytest.raises(ValueError):
        TS.parse_classes("buoy_red", "boundary-v14")                   # a buoy is never an obstacle ring


def test_obstacle_far_geometry():
    o = TS.obstacle_far(6.0, 0.0, [900, 500, 1308, 700], fx=1000.0, cx=1104.0, conf=0.71, cls="obstacle")
    assert o["range_m"] == 6.0 and o["bearing_deg"] == 0.0
    assert o["bearing_min_deg"] == pytest.approx(math.degrees(math.atan2(-204, 1000)), abs=0.05)
    assert o["bearing_max_deg"] == pytest.approx(math.degrees(math.atan2(204, 1000)), abs=0.05)
    assert o["src"] == "model" and o["cls"] == "obstacle" and o["conf"] == 0.71 and o["px"] == 408
    o2 = TS.obstacle_far(4.0, 3.0, [0, 0, 10, 10], 1000.0, 0.0, 0.5, "dock")
    assert o2["range_m"] == 5.0 and o2["bearing_deg"] == pytest.approx(36.9, abs=0.1)   # + starboard, horizontal range


def test_the_boat_rings_a_model_obstacle_like_a_depth_cluster():
    """No boat change: the model's cluster rides the guard's `far` list, map_far rings it after 3 sightings (2 m ring, 1 m behind)."""
    O = ControlOrigin(1.28, 103.85)
    pose = LocalPose(0.0, 0.0, 0.0, O.lat, O.lon, 0.0)
    pm, g = PerceptionMap(Config()), GuardSupervisor()
    msg = {"clear": True, "far": [TS.obstacle_far(12.0, 0.0, [1090, 0, 1118, 10], 1000.0, 1104.0, 0.8, "obstacle")]}
    for t in (10.0, 10.6, 11.2):
        g.map_far(msg, pose, pm, t)
    e = pm.entities("obstacle", include_young=True)
    assert len(e) == 1 and abs(e[0].x_n - 13.0) < 1e-6 and pm.obstacle_counts(e[0])     # 12 m: beyond the depth guard's 8 m


def test_observer_wiring():
    src = (CV / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert "TS.check_names(a.detector, model.names)" in src and "obst_ids = set(TS.class_ids(model.names, a.obstacle_classes))" in src
    assert 'payload["guard"]["far"] = sorted(' in src and 'payload["obstacles"]' in src and 'frec["obstacles"]' in src
    assert "boxes_only = a.detector in TS.BOX_DETECTORS" in src
    assert "meas = S.measure_mask(xyz, mask, max_depth_m=a.max_depth)" in src      # the buoy path's call unchanged
    assert TS.DETECTORS == ("mask", "model-a", "boundary-v14", "six-class") and TS.V14_NAMES == tuple(V14.values())


def test_run_sh_v14_path(tmp_path):
    r = subprocess.run(["bash", str(CV / "buoys_multi" / "run.sh"), "--headless", "--detector", "boundary-v14"], capture_output=True,
                       text=True, env={"PATH": "/usr/bin:/bin", "HOME": str(tmp_path), "CV_MODEL_V14": str(tmp_path / "v14.engine")})
    assert r.returncode == 1 and "v14.engine" in r.stderr and "CV_MODEL_V14" in r.stderr
    r = subprocess.run(["bash", str(CV / "buoys_multi" / "run.sh"), "--headless", "--detector", "boundary-v14"], capture_output=True,
                       text=True, env={"PATH": "/usr/bin:/bin", "HOME": str(tmp_path)})
    assert r.returncode == 1 and "buoy_robotx/runtime/boundary_v14_20261111/models/boundary_v14_best" in r.stderr


def test_the_detector_choice_everywhere():
    a = next(x for x in SA.ACTIONS if x.id == "observer")
    assert "agx_observer.sh start --detector boundary-v14" in " ".join(a.build({"do": "start", "detector": "boundary-v14"}, {}))
    assert "agx_observer.sh start --detector default" in " ".join(a.build({"do": "start", "detector": "rm -rf"}, {}))
    assert L.observer_cmd("start", detector="boundary-v14").endswith("--detector boundary-v14")
    sh = (ROOT / "scripts" / "agx_observer.sh").read_text()
    assert "default|model-a|boundary-v14|six-class) ;;" in sh and 'DARG=" --detector $DET"' in sh
    assert subprocess.run(["bash", "-n", str(ROOT / "scripts" / "agx_observer.sh")]).returncode == 0
    r = subprocess.run(["bash", str(ROOT / "scripts" / "agx_observer.sh"), "start", "--detector", "v15"], capture_output=True, text=True)
    assert r.returncode == 2
    assert '<option value="boundary-v14">boundary-v14</option>' in (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()


# ---- the model switch from the OCS (Dmitrii: "keep previous models, so we could select from drop-down") ----------------------------
def test_switching_the_model_from_the_ocs():
    a = next(x for x in SA.ACTIONS if x.id == "observer")
    task = {"status": {"state": "AUTO_TASK"}, "perception": {"observer_detector": "default"}}
    assert "not while a task runs" in a.opt_guard({"do": "start", "detector": "boundary-v14"}, task)
    assert a.opt_guard({"do": "start", "detector": "default"}, task) == ""                       # the same model: nothing restarts
    assert a.opt_guard({"do": "start", "detector": "boundary-v14"}, dict(task, status={"state": "AUTO_HOLD"})) == ""   # before the task: yes
    sh = (ROOT / "scripts" / "agx_observer.sh").read_text()
    assert 'observer: switching $(running_det) -> $DET' in sh and sh.index("running_det()") < sh.index('if up; then echo "observer: up (already')
    src = (CV / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert 'payload["detector"] = "default" if a.detector == "mask" else a.detector' in src
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "⇄ switch the model ${esc(cur)} → ${esc(det)}?" in page


def test_cv_publish_reports_the_running_model():
    CP = load("cv_publish_v0187", CV / "publish" / "cv_publish.py")

    class _A:
        model = None
    pub = type("P", (), {"a": _A(), "obs_detector": None, "model": "?", "classes": []})()
    CP.observer_model(pub, {"detector": "boundary-v14", "model": "boundary_v14_best.engine"})
    assert pub.obs_detector == "boundary-v14" and pub.model == "boundary_v14_best.engine"


def test_six_class_is_a_choice_too():
    six = ["buoy_red", "buoy_green", "buoy_blue", "buoy", "dock", "platform"]
    assert TS.buoy_class_ids(six) == [0, 1, 2, 3]
    TS.check_names("six-class", six)
    with pytest.raises(SystemExit):
        TS.check_names("six-class", list(V14.values()))
    assert TS.pick_colour("auto", "buoy", 0.9, "white") == ("white", "hsv")      # the generic buoy: our colour vote
    a = next(x for x in SA.ACTIONS if x.id == "observer")
    assert "--detector six-class" in " ".join(a.build({"do": "start", "detector": "six-class"}, {}))
    assert L.observer_cmd("start", detector="six-class").endswith("--detector six-class")
    assert '<option value="six-class">six-class</option>' in (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    r = subprocess.run(["bash", str(CV / "buoys_multi" / "run.sh"), "--headless", "--detector", "six-class"], capture_output=True,
                       text=True, env={"PATH": "/usr/bin:/bin", "HOME": "/nonexistent"})
    assert r.returncode == 1 and "six_class_20261111/models/robotx_six_class_yolo11m_1280_best.pt" in r.stderr


# ---- v0.1.87.1: the UAV map publisher's port (their default: the boat on 1883; ours 1884) ------------------------------------------
def test_the_boat_broker_also_listens_where_the_uav_publishes(tmp_path):
    conf = (ROOT / "profiles" / "pool" / "mosquitto" / "boat_pool.conf.in").read_text()
    assert "#UAVLISTEN listener @UAV_PORT@ 0.0.0.0\n#UAVLISTEN protocol mqtt\n" in conf
    sh = (ROOT / "scripts" / "run_usv.sh").read_text()
    assert 'BOAT_UAV_PORT="${BOAT_UAV_PORT:-1883}"' in sh and 'grep -q ":$BOAT_UAV_PORT "' in sh   # v0.1.90.12: the drone keeps 1883 (our hub moved to 1882)
    assert sh.index("BOAT_UAV_PORT=") < sh.index('if [ -n "${OCS_HOST:-}" ]; then\n  sed')
    assert subprocess.run(["bash", "-n", str(ROOT / "scripts" / "run_usv.sh")]).returncode == 0
    on = subprocess.run(["bash", "-c", 'U="-e s|@UAV_PORT@|1883| -e s|^#UAVLISTEN.||"; sed -e "/^#BRIDGE /d" -e "/^#UAVBRIDGE/d" $U "$0"',
                         str(ROOT / "profiles" / "pool" / "mosquitto" / "boat_pool.conf.in")], capture_output=True, text=True).stdout
    assert "listener 1883 0.0.0.0\nprotocol mqtt" in on and "@UAV_PORT@" not in on
    off = subprocess.run(["bash", "-c", 'sed -e "/^#BRIDGE /d" -e "/^#UAVBRIDGE/d" -e "/^#UAVLISTEN/d" "$0"',
                          str(ROOT / "profiles" / "pool" / "mosquitto" / "boat_pool.conf.in")], capture_output=True, text=True).stdout
    assert "1886" not in off.split("# ----")[0] and "1883" not in off.split("# ----")[0]
