"""ROADMAP WP9b (v0.1.40): the ZED recorder in OUR buoy observer (--record DIR [--label] [--max-min], cam.svo2 + frames.jsonl +
session.json, the free-space guard), the replay (--svo FILE: the same loop from the file), scripts/agx_record.sh, the bring-home
script (scripts/agx_pull_clips.sh) and the page's RECORDER card. On the Mac: no pyzed, no camera - the ZED SDK, OpenCV, YOLO and
the students' module are stubs (the real recording is rehearsed on the AGX)."""
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import types
from dataclasses import dataclass
from datetime import datetime

import pytest

from .conftest import ROOT

np = pytest.importorskip("numpy")
BM = ROOT / "cv" / "buoys_multi"
H, W = 40, 60


def rec_core():
    spec = importlib.util.spec_from_file_location("rec_core", BM / "rec_core.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


# ---- the stubs: pyzed.sl, cv2, ultralytics, the students' module ----------------------------------------------------------------
class Code:
    def __init__(self, name):
        self.name = name

    def __repr__(self):
        return self.name


def make_sl(log):
    sl = types.ModuleType("pyzed.sl")
    sl.ERROR_CODE = types.SimpleNamespace(SUCCESS=Code("SUCCESS"), END_OF_SVOFILE_REACHED=Code("END_OF_SVOFILE_REACHED"), FAILURE=Code("FAILURE"))
    ns = types.SimpleNamespace
    sl.VIEW, sl.MEASURE, sl.TIME_REFERENCE = ns(LEFT=1), ns(XYZRGBA=2), ns(IMAGE=3)
    sl.UNIT, sl.COORDINATE_SYSTEM = ns(METER="m"), ns(IMAGE="img")
    sl.DEPTH_MODE = ns(NEURAL="neural", NEURAL_LIGHT="nl", ULTRA="ultra")
    sl.SVO_COMPRESSION_MODE = ns(H264="H264", H265="H265")

    class RuntimeParameters:
        pass

    class Mat:
        def __init__(self):
            self.data = None

        def get_data(self):
            return self.data

    class InitParameters:
        def __init__(self):
            self.svo = None

        def set_from_svo_file(self, path):
            self.svo = path

    class RecordingParameters:
        def __init__(self, path, mode):
            self.path, self.mode = path, mode

    class Camera:
        svo_frames = 4

        def __init__(self):
            self.n, self.svo, self.recording = 0, None, None
            log.setdefault("cameras", []).append(self)

        @staticmethod
        def get_sdk_version():
            return "5.0.0-stub"

        def open(self, init):
            self.svo = init.svo
            log["init"] = init
            return sl.ERROR_CODE.SUCCESS

        def get_camera_information(self):
            cal = ns(left_cam=ns(fx=500.0, fy=500.0, cx=W / 2, cy=H / 2))
            return ns(camera_model="ZED2i", serial_number=33545355, camera_configuration=ns(calibration_parameters=cal, firmware_version=1523))

        def grab(self, runtime):
            if self.svo and self.n >= self.svo_frames:
                return sl.ERROR_CODE.END_OF_SVOFILE_REACHED
            self.n += 1
            if self.recording:
                with open(self.recording, "ab") as fh:
                    fh.write(b"\0" * 1000)                       # the SDK writes the frame
            return sl.ERROR_CODE.SUCCESS

        def get_svo_position(self):
            return self.n - 1

        def get_timestamp(self, ref):
            return ns(get_nanoseconds=lambda: 1_700_000_000_000_000_000 + self.n * 66_000_000)

        def retrieve_image(self, mat, view):
            img = np.zeros((H, W, 4), np.uint8)
            img[10:20, 20:30, :3] = (0, 0, 200)                   # a red buoy (BGR)
            mat.data = img

        def retrieve_measure(self, mat, what):
            xyz = np.full((H, W, 4), np.nan, np.float32)
            xyz[10:20, 20:30, 2] = 6.0
            mat.data = xyz

        def enable_recording(self, params):
            log.setdefault("recordings", []).append((params.path, params.mode))
            self.recording = params.path
            open(params.path, "wb").close()
            return sl.ERROR_CODE.SUCCESS

        def disable_recording(self):
            log["disabled"] = log.get("disabled", 0) + 1
            self.recording = None

        def close(self):
            log["closed"] = True

    sl.RuntimeParameters, sl.Mat, sl.InitParameters, sl.RecordingParameters, sl.Camera = RuntimeParameters, Mat, InitParameters, RecordingParameters, Camera
    return sl


@dataclass
class Meas:
    range_m: float
    forward_m: float
    right_m: float
    down_m: float
    bearing_deg: float


def make_students(log, sl):
    S = types.ModuleType("actual_buoy_observer")
    S.WEIGHTS, S.DEFAULT_MAVLINK = "/stub/best.pt", "udpin:0.0.0.0:14550"
    S.discover_mavlink = lambda: None
    S.utc_now = lambda: datetime.utcnow().isoformat() + "+00:00"

    class Tel:
        def __init__(self, ep, baud):
            log["mavlink"] = ep

        def start(self):
            pass

        def snapshot(self):
            return {"lat": 26.1, "lon": -80.1, "yaw_rad": 1.5707963, "gps_fix": 3, "satellites": 14, "position_age_s": 0.1,
                    "attitude_age_s": 0.1, "heartbeat_age_s": 0.2, "position_time": 123.0, "armed": False, "mode": "MANUAL"}

        def close(self):
            pass

    S.PixhawkTelemetry = Tel

    def open_zed(res, fps, mode, depth_max_m=20.0):
        log["open_zed"] = (res, fps, mode, depth_max_m)
        return sl.Camera()

    S.open_zed = open_zed
    S.measure_mask = lambda xyz, mask, max_depth_m=20.0: Meas(6.0, 5.9, 0.5, 0.2, 4.8)
    S.buoy_geolocation = lambda meas, tel, f, r, y: {"latitude_deg": tel["lat"] + 1e-5, "longitude_deg": tel["lon"] + 5e-5,
                                                     "north_offset_m": 1.0, "east_offset_m": 5.9}
    return S


class Arr:
    def __init__(self, a):
        self.a = a

    def cpu(self):
        return self

    def numpy(self):
        return self.a


class Scalar:
    def __init__(self, v):
        self.v = v

    def item(self):
        return self.v


def make_yolo():
    class Box:
        cls, conf = Scalar(0), Scalar(0.9)
        xyxy = [Arr(np.array([20.0, 10.0, 30.0, 20.0]))]

    class Result:
        boxes = [Box()]
        masks = types.SimpleNamespace(data=Arr(np.pad(np.ones((10, 10), np.float32), ((10, H - 20), (20, W - 30)))[None]))

    class YOLO:
        def __init__(self, w, task=None):
            pass

        def predict(self, frame, **kw):
            return [Result()]

    return types.SimpleNamespace(YOLO=YOLO)


def make_cv2():
    c = types.ModuleType("cv2")
    c.INTER_LINEAR = 1
    c.INTER_AREA, c.IMWRITE_JPEG_QUALITY = 3, 1                       # v0.1.84.8: the observer's live preview (write_preview)
    c.resize = lambda m, size, interpolation=None: m
    c.imencode = lambda ext, img, params=None: (True, np.frombuffer(b"\xff\xd8fake", dtype=np.uint8))
    for f in ("rectangle", "putText", "imwrite", "imshow", "destroyAllWindows"):
        setattr(c, f, lambda *a, **k: None)
    c.waitKey = lambda *a: 0
    return c


@pytest.fixture
def obs(monkeypatch):
    log = {}
    sl = make_sl(log)
    pyzed = types.ModuleType("pyzed")
    pyzed.sl = sl
    for name, mod in (("pyzed", pyzed), ("pyzed.sl", sl), ("cv2", make_cv2()), ("ultralytics", make_yolo()),
                      ("actual_buoy_observer", make_students(log, sl))):
        monkeypatch.setitem(sys.modules, name, mod)
    monkeypatch.setitem(sys.modules, "rec_core", rec_core())
    spec = importlib.util.spec_from_file_location("multi_buoy_observer_wp9b", BM / "multi_buoy_observer.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    m._log = log
    return m


def only_session(root):
    ss = [d for d in os.listdir(root) if os.path.isdir(os.path.join(root, d))]
    assert len(ss) == 1, ss
    return os.path.join(root, ss[0])


def lines(path):
    return [json.loads(x) for x in open(path)]


# ---- the arguments ------------------------------------------------------------------------------------------------------------
def test_record_and_svo_arguments(obs, tmp_path):
    a = obs.parse_args(["--record", str(tmp_path), "--label", "day1_t1_gate", "--max-min", "10"])
    assert a.record == str(tmp_path) and a.label == "day1_t1_gate" and a.max_min == 10 and a.svo is None
    assert a.output.endswith(os.path.join("buoys", "runs")) and a.min_free_gb == 5.0
    for bad in (["--record", str(tmp_path), "--label", "bad label"], ["--record", str(tmp_path), "--label", "../x"],
                ["--max-min", "5"], ["--label", "x"], ["--record", str(tmp_path), "--max-min", "-1"],
                ["--svo", str(tmp_path / "missing.svo2")]):
        with pytest.raises(SystemExit) as e:
            obs.parse_args(bad)
        assert e.value.code == 2, bad
    svo = tmp_path / "s" / "cam.svo2"
    svo.parent.mkdir()
    svo.write_bytes(b"x")
    with pytest.raises(SystemExit):
        obs.parse_args(["--svo", str(svo), "--record", str(tmp_path)])            # one or the other
    a = obs.parse_args(["--svo", str(svo)])
    assert a.output == str(svo.parent / "replay")                                 # never where cv_publish looks
    assert obs.parse_args(["--svo", str(svo), "--output", "/x"]).output == "/x"
    run_sh = (BM / "run.sh").read_text()
    assert "--svo|--output) OUT=()" in run_sh


# ---- recording --------------------------------------------------------------------------------------------------------------
def test_record_writes_svo_frames_and_session(obs, tmp_path):
    rec, out = tmp_path / "rec", tmp_path / "runs"
    obs.main(["--record", str(rec), "--label", "bench_test", "--frames", "5", "--output", str(out), "--headless", "--mavlink", "udpin:stub"])
    log = obs._log
    d = only_session(str(rec))
    assert os.path.basename(d).endswith("_bench_test") and len(os.path.basename(d)) == len("20261106_120000_bench_test")
    assert log["recordings"] == [(os.path.join(d, "cam.svo2"), "H265")] and log["disabled"] == 1 and log["closed"]   # v0.1.86.3 (WP47): H.265
    assert os.path.getsize(os.path.join(d, "cam.svo2")) == 5000
    doc = json.load(open(os.path.join(d, "session.json")))
    kit = (ROOT / "optiprimex" / "__init__.py").read_text().split('__version__ = "')[1].split('"')[0]
    assert doc["label"] == "bench_test" and doc["state"] == "stopped" and doc["stop_reason"] == "frames done" and doc["frames"] == 5
    assert doc["versions"]["kit"] == kit and doc["versions"]["zed_sdk"] == "5.0.0-stub" and doc["versions"]["camera_serial"] == 33545355
    assert doc["start_utc"] and doc["end_utc"] and doc["svo_bytes"] == 5000 and doc["options"]["max_depth"] == 25.0
    fr = lines(os.path.join(d, "frames.jsonl"))
    assert [f["svo_frame"] for f in fr] == [0, 1, 2, 3, 4] and [f["frame"] for f in fr] == [0, 1, 2, 3, 4]
    f0 = fr[0]
    assert f0["lat"] == 26.1 and f0["lon"] == -80.1 and f0["heading_deg"] == pytest.approx(90.0, abs=0.01) and f0["tel_fresh"]
    assert f0["ts_ns"] and f0["status"] == "OK" and f0["n_buoys"] == 1
    b = f0["buoys"][0]
    assert b["colour"] == "red" and b["bbox"] == [20.0, 10.0, 30.0, 20.0] and b["range_m"] > 6.0 and b["buoy_lat"] == pytest.approx(26.10001)
    assert "position_time" not in f0["tel"] and f0["tel"]["gps_fix"] == 3                 # the monotonic stamps are dropped
    st = json.load(open(os.path.join(only_session(str(out)), "latest_status.json")))
    assert st["recording"]["label"] == "bench_test" and st["recording"]["frames"] == 5     # what the page / status can see


def test_free_space_guard_refuses_to_start(obs, tmp_path, monkeypatch):
    R = sys.modules["rec_core"]
    monkeypatch.setattr(R, "free_gb", lambda p, usage=None: 3.2)
    with pytest.raises(SystemExit) as e:
        obs.main(["--record", str(tmp_path / "rec"), "--frames", "3", "--output", str(tmp_path / "runs"), "--headless", "--allow-no-pixhawk"])
    assert e.value.code == 3 and "cameras" not in obs._log and os.listdir(tmp_path / "rec") == []    # the camera never opened


def test_disk_low_stops_the_recording_detection_goes_on(obs, tmp_path, monkeypatch):
    R = sys.modules["rec_core"]
    free = iter([30.0, 30.0, 30.0] + [4.0] * 50)                                        # the start check, frames 0 and 1, then full at frame 2
    monkeypatch.setattr(R, "free_gb", lambda p, usage=None: next(free))
    clock = iter(range(0, 1000, 10))                                                     # 10 s per check: the disk is read each time
    real = R.Guard
    monkeypatch.setattr(R, "Guard", lambda *a, **k: real(*a, clock=lambda: next(clock), **k))
    out = tmp_path / "runs"
    obs.main(["--record", str(tmp_path / "rec"), "--frames", "6", "--output", str(out), "--headless", "--allow-no-pixhawk"])
    d = only_session(str(tmp_path / "rec"))
    doc = json.load(open(os.path.join(d, "session.json")))
    assert doc["stop_reason"].startswith("disk_low: 4.0 GB free < 5 GB") and doc["frames"] == 3 and obs._log["disabled"] == 1
    assert len(lines(os.path.join(d, "frames.jsonl"))) == 3
    csv_rows = open(os.path.join(only_session(str(out)), "buoy_observations.csv")).read().strip().splitlines()
    assert len(csv_rows) == 1 + 6                                                        # all six frames still detected
    st = json.load(open(os.path.join(only_session(str(out)), "latest_status.json")))
    assert st["recording"]["active"] is False and st["recording"]["stop_reason"].startswith("disk_low")
    assert "stopped: disk_low" in R.status_text(str(tmp_path / "rec"), free_fn=lambda p: 4.0)
    assert "WARNING: < 5 GB" in R.status_text(str(tmp_path / "rec"), free_fn=lambda p: 4.0)


def test_max_min_stops_the_recording(obs, tmp_path, monkeypatch):
    R = sys.modules["rec_core"]
    monkeypatch.setattr(R, "free_gb", lambda p, usage=None: 50.0)
    clock = iter(range(0, 10000, 20))                                                    # 20 s per frame
    real = R.Guard
    monkeypatch.setattr(R, "Guard", lambda *a, **k: real(*a, clock=lambda: next(clock), **k))
    obs.main(["--record", str(tmp_path / "rec"), "--max-min", "1", "--frames", "8", "--output", str(tmp_path / "runs"), "--headless",
              "--allow-no-pixhawk"])
    doc = json.load(open(os.path.join(only_session(str(tmp_path / "rec")), "session.json")))
    assert doc["stop_reason"] == "max_min: 1 min recorded" and doc["frames"] == 3 and doc["options"]["max_min"] == 1.0


def test_guard_and_label_units():
    R = rec_core()
    t = [0.0]
    free = [20.0]
    g = R.Guard("/x", max_min=2, min_free_gb=5, every_s=5, clock=lambda: t[0], free_fn=lambda p: free[0])
    assert g.start_refusal() == "" and g.check() == ""
    free[0] = 4.5
    t[0] = 3.0
    assert g.check() == ""                                                               # the disk is read every 5 s
    t[0] = 6.0
    assert g.check().startswith("disk_low: 4.5 GB")
    free[0] = 20.0
    t[0] = 120.0
    assert g.check() == "max_min: 2 min recorded"
    assert R.Guard("/x", free_fn=lambda p: 1.0).start_refusal().startswith("disk low: 1.0 GB free < 5 GB")
    assert R.Guard("/x", max_min=0, clock=lambda: 1e9, free_fn=lambda p: 9.0).check() == ""   # 0 = no time limit
    assert R.session_name("pool_t1", datetime(2026, 11, 6, 9, 5, 1)) == "20261106_090501_pool_t1"
    for bad in ("", "a b", "x/y", "-x", "é", "a" * 41, "x;rm"):
        with pytest.raises(ValueError):
            R.check_label(bad)
    assert R.check_label(" day2_dock-am ") == "day2_dock-am"
    assert R.free_gb("/definitely/not/here/x", usage=lambda p: types.SimpleNamespace(free=7e9)) == 7.0
    assert R.main(["label", "ok_1"]) == 0 and R.main(["label", "no way"]) == 2


# ---- the replay -------------------------------------------------------------------------------------------------------------
def test_svo_replays_the_same_loop_from_the_file(obs, tmp_path):
    rec = tmp_path / "rec"
    obs.main(["--record", str(rec), "--label", "clip", "--frames", "4", "--output", str(tmp_path / "runs"), "--headless", "--mavlink", "udpin:stub"])
    d = only_session(str(rec))
    obs._log.clear()
    obs.main(["--svo", os.path.join(d, "cam.svo2"), "--headless"])                 # no --mavlink: the telemetry comes from frames.jsonl
    log = obs._log
    assert log["init"].svo == os.path.join(d, "cam.svo2") and "open_zed" not in log and "recordings" not in log and "mavlink" not in log
    assert log["init"].depth_maximum_distance == 25.0 and log["init"].svo_real_time_mode is False
    rp = os.path.join(d, "replay")
    run = only_session(rp)
    got = lines(os.path.join(run, "frames.jsonl"))
    assert [g["svo_frame"] for g in got] == [0, 1, 2, 3]                                # stopped at the end of the file
    assert got[0]["buoys"][0]["colour"] == "red" and got[0]["buoys"][0]["buoy_lat"] == pytest.approx(26.10001)   # geolocated from the recording
    assert os.listdir(tmp_path / "runs") == [only_session(str(tmp_path / "runs")).split(os.sep)[-1]]           # nothing new where cv_publish looks
    R = sys.modules["rec_core"]
    c = R.compare(os.path.join(d, "frames.jsonl"), os.path.join(run, "frames.jsonl"))
    assert c["frames_compared"] == 4 and c["same_count"] == 1.0 and c["same_colours"] == 1.0 and c["ranges_within_tol"] == 1.0
    # a replay frame that differs
    bad = [dict(g) for g in got]
    bad[1] = dict(bad[1], buoys=[dict(bad[1]["buoys"][0], colour="green", range_m=bad[1]["buoys"][0]["range_m"] + 1.0)])
    p = tmp_path / "bad.jsonl"
    p.write_text("".join(json.dumps(x) + "\n" for x in bad))
    c = R.compare(os.path.join(d, "frames.jsonl"), str(p))
    assert c["same_count"] == 1.0 and c["same_colours"] == 0.75 and c["ranges_within_tol"] == 0.75 and c["worst_range_diff_m"] == pytest.approx(1.0)
    out = subprocess.run([sys.executable, str(BM / "rec_core.py"), "compare", os.path.join(d, "frames.jsonl"), str(p)], capture_output=True, text=True, timeout=20)
    assert out.returncode == 0 and json.loads(out.stdout)["same_colours"] == 0.75


def test_status_text(tmp_path):
    R = rec_core()
    root = str(tmp_path / "recs")
    assert "0 session(s)" in R.status_text(root, free_fn=lambda p: 20.0) and "no session yet" in R.status_text(root, free_fn=lambda p: 20.0)
    s = R.Session(root, "live_one", {"kit": "x"}, {}, now=datetime(2026, 11, 6, 10, 0, 0))
    s.add({"svo_frame": 0})
    assert "RECORDING" in R.status_text(root, free_fn=lambda p: 20.0)                   # this process is alive
    doc = json.load(open(os.path.join(s.dir, "session.json")))
    doc["pid"] = 999999                                                                  # an observer that died without closing
    R.write_json_atomic(os.path.join(s.dir, "session.json"), doc)
    assert "ended without closing" in R.status_text(root, free_fn=lambda p: 20.0)
    s.stop("max_min: 10 min recorded")
    t = R.status_text(root, free_fn=lambda p: 20.0)
    assert "stopped: max_min: 10 min recorded" in t and "1 frames" in t and "label live_one" in t and "20.0 GB free" in t


# ---- the scripts ------------------------------------------------------------------------------------------------------------
def sh(*args, env=None, timeout=20):
    e = dict(os.environ, **(env or {}))
    return subprocess.run(["bash", *args], capture_output=True, text=True, timeout=timeout, env=e)


def test_agx_record_help_bad_args_and_status_without_an_agx(tmp_path):
    s = str(ROOT / "scripts" / "agx_record.sh")
    h = sh(s, "--help")
    assert h.returncode == 0 and "start LABEL" in h.stdout and "status" in h.stdout and "agx_pull_clips.sh" in h.stdout
    assert sh(s).returncode == 0                                                         # no verb = the help
    for bad, want in ((["bogus"], "start LABEL | stop | status"), (["start"], "start LABEL: letters"), (["start", "a b"], "start LABEL"),
                      (["start", "ok", "--max-min", "x"], "--max-min N"), (["start", "ok", "--wat"], "unknown option --wat")):
        r = sh(s, *bad, env={"AGX_REC_DIR": str(tmp_path / "r")})
        assert r.returncode == 2 and want in r.stderr, (bad, r.stderr)
    env = {"AGX_REC_DIR": str(tmp_path / "recs"), "AGX_REC_SESSION": "wp9b_test_none"}
    r = sh(s, "status", env=env)
    assert r.returncode == 0 and "recorder down" in r.stdout and "0 session(s)" in r.stdout
    R = rec_core()
    sess = R.Session(str(tmp_path / "recs"), "bench_clip", {}, {})
    sess.stop("max_min: 1 min recorded")
    r = sh(s, "status", env=env)
    assert "recorder down" in r.stdout and "label bench_clip" in r.stdout and "stopped: max_min: 1 min recorded" in r.stdout
    r = sh(s, "stop", env=env)                                                           # nothing to stop: says so, no error
    assert r.returncode == 0 and "recorder stopped" in r.stdout
    stop_agx = (ROOT / "scripts" / "stop_agx.sh").read_text()
    assert "agx_record.sh\" stop" in stop_agx


def test_pull_clips_dry_run(tmp_path):
    s = str(ROOT / "scripts" / "agx_pull_clips.sh")
    env = {"AGX_SSH": "agx-test-host", "REC_HOME": str(tmp_path / "home_recs"), "NET_PROFILE": "lan"}
    h = sh(s, "--help")
    assert h.returncode == 0 and "OUTSIDE CV_TRAIN" in h.stdout
    r = sh(s, "--dry-run", env=env)
    today = datetime.now().strftime("%Y-%m-%d")
    assert r.returncode == 0 and "nothing contacted" in r.stdout and "agx-test-host:Desktop/dm3/recordings/" in r.stdout
    assert f"{tmp_path}/home_recs/{today}/" in r.stdout and "rsync -a --partial" in r.stdout and "replay/" in r.stdout
    assert "rec_core.py status Desktop/dm3/recordings" in r.stdout and not (tmp_path / "home_recs").exists()   # nothing created
    r = sh(s, "--dry-run", "--session", "20261106_100000_day1_t1", "--session", "20261106_110000_dock", "--with-replays", env=env)
    assert "agx-test-host:Desktop/dm3/recordings/20261106_100000_day1_t1 agx-test-host:Desktop/dm3/recordings/20261106_110000_dock" in r.stdout
    assert "replay/" not in r.stdout.split("would run:")[1].split("\n")[0]
    r = sh(s, "--dry-run", "--list", env=env)
    assert "du -sh" in r.stdout and "agx-test-host" in r.stdout
    r = sh(s, "--dry-run", "--dest", str(tmp_path / "CV_TRAIN" / "data"), env=env)
    assert r.returncode == 2 and "inside CV_TRAIN" in r.stderr
    for bad in (["--session", "../etc"], ["--session", "a b"], ["--bogus"]):
        assert sh(s, "--dry-run", *bad, env=env).returncode == 2, bad


# ---- the page and the SYSTEM actions ----------------------------------------------------------------------------------------
def test_the_page_card_and_the_actions():
    from optiprimex.tools import system_actions as SA
    from optiprimex.tools.system_actions import Runner
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'id="x-rec"' in html and "RECORDER" in html and 'a.id==="rec"' in html and "o.free?" in html and 'status:"status"' in html
    a = SA.BY_ID["rec"]
    assert a.host == "agx" and a.group == "settings" and not a.confirm
    pub = a.public({})
    assert [o["name"] for o in pub["options"]] == ["do", "label", "max_min", "replace", "pixhawk"] and pub["options"][1]["free"]
    r = Runner(env={"AGX_SSH": "agx-host", "ORIN_SSH": "nx-host"}, dry_run=True, log_path="/dev/null")
    AUTO_TASK = {"status": {"state": "AUTO_TASK", "task_name": "T1"}}
    cmd = lambda: r.jobs[-1].public()["cmd"]  # noqa: E731
    assert r.run("rec", {}, "", {})["ok"] and "agx-host" in cmd() and "scripts/agx_record.sh status" in cmd()
    assert r.run("rec", {"do": "start", "label": "day1_t1_gate", "max_min": "20", "replace": "yes"}, "", {})["ok"]
    assert "scripts/agx_record.sh start day1_t1_gate --max-min 20 --replace" in cmd() and "--no-pixhawk" not in cmd()
    assert r.run("rec", {"do": "start", "label": "x; rm -rf ~", "max_min": "999", "pixhawk": "no"}, "", {})["ok"]
    assert "agx_record.sh start event --max-min 10 --no-pixhawk" in cmd() and "rm -rf" not in cmd()
    assert r.run("rec", {"do": "stop"}, "", {})["ok"] and "scripts/agx_record.sh stop" in cmd()
    assert "the recorder IS the buoy observer" in r.run("rec", {"do": "stop"}, "", AUTO_TASK)["why"]
    assert "the recorder IS the buoy observer" in r.run("rec", {"do": "start", "replace": "yes"}, "", AUTO_TASK)["why"]
    assert r.run("rec", {"do": "start", "label": "course_run1"}, "", AUTO_TASK)["ok"] and r.run("rec", {"do": "status"}, "", AUTO_TASK)["ok"]
    # the old drop-downs keep their whitelist
    assert SA.Option("x", "x", ["a", "b"], "a").accept("c") == "a" and SA.Option("x", "x", ["a"], "a", free=r"[a-z]+").accept("zz") == "zz"
