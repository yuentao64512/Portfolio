"""v0.1.80 (WP36): the ZED proximity guard - the AGX filter (cv/buoys_multi/guard_core: the levelled cloud, the band, persistence, the
IMU tilt), the companion's supervisor (usv/guard: SLOW cap, BRAKE <= 0.3 s after the message, BACKOFF 2 m, the ring, re-plan, the 2nd trigger
blocks the target, the 3rd within 3 m -> AUTO_SAFE_HOLD, silence -> cap + CAUTION, a low thing remembered under the frame's edge), the fast SIL
on S15 with its two obstacles, the sim observer's guard on the rendered depth, the ground test m0t9, the wiring (mission manager, cv_publish,
the OCS chip, the boat window). QUICK: no broker."""
from __future__ import annotations

import importlib.util
import json
import math
import sys

import numpy as np
import pytest
import yaml

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin
from optiprimex.companion.state import FcuState, GuidedTarget, Hold, LocalPose, RunContext, ThrusterFrame
from optiprimex.sim import world as W
from optiprimex.sim.fake_guard import FakeGuard
from optiprimex.sim.fast_sil import FastSil
from optiprimex.usv.behaviours.base import BehaviourContext
from optiprimex.usv.guard import GuardSupervisor
from optiprimex.usv.perception_map import PerceptionMap
from optiprimex.usv.tasks import REGISTRY
from optiprimex.usv.tasks.base import TaskContext

from .conftest import ROOT

sys.path.insert(0, str(ROOT / "cv" / "buoys_multi"))
import guard_core as GC  # noqa: E402

WORLD = str(ROOT / "profiles" / "bench" / "world_scout.yaml")
POLY = [tuple(p) for p in W.load(WORLD)["bounds"]]


def board(dist=1.2, h_cam=1.0):
    xs, hs = np.meshgrid(np.linspace(-0.3, 0.3, 40), np.linspace(0.2, 1.0, 40))
    water = np.stack([xs, np.full_like(xs, h_cam - 0.02), np.full_like(xs, 2.0)], axis=2)     # glitter 2 cm above the water: never
    obj = np.stack([xs, h_cam - hs, np.full_like(xs, dist)], axis=2)
    return np.concatenate([obj, water], axis=0).astype(np.float32)


def quat_from_R(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)


# ---- the AGX filter ---------------------------------------------------------------------------------------------------------------
def test_the_filter_persists_three_frames_and_ignores_the_water():
    g = GC.GuardFilter({"step": 1, "min_px": 100})
    out = [g.frame(board(1.2)) for _ in range(3)]
    assert [m["zone"] for m in out] == ["slow", "slow", "stop"] and out[-1]["frames"] == 3
    assert out[-1]["range_m"] == pytest.approx(1.2, abs=0.05) and out[-1]["bearing_min_deg"] < -10 < 10 < out[-1]["bearing_max_deg"]
    assert g.frame(board(2.5))["zone"] == "slow" and g.frame(board(2.5))["frames"] == 0                # out of STOP: the count restarts
    assert GC.GuardFilter({"step": 1, "min_px": 100}).frame(board(4.0))["clear"] is True             # beyond slow_m
    water_only = board(1.2)[40:]
    assert GC.GuardFilter({"step": 1, "min_px": 10}).frame(water_only)["clear"] is True             # 2 cm glitter: below the band


def test_the_imu_tilt_does_not_move_the_trigger():
    """The brief's bench check: tilt the camera +-10 deg - the trigger distance does not move (the cloud levelled by gravity)."""
    def Rz(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])   # noqa: E702
    def Rx(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])   # noqa: E702
    pw = board(1.3).reshape(-1, 3).astype(np.float64)
    for R in (Rz(math.radians(10)), Rz(math.radians(-10)), Rx(math.radians(10)), Rx(math.radians(-10)), Rz(math.radians(7)) @ Rx(math.radians(-6))):
        pc = (R.T @ pw.T).T.reshape(80, 40, 3).astype(np.float32)
        g = GC.GuardFilter({"step": 1, "min_px": 100})
        m = [g.frame(pc, quat_from_R(R)) for _ in range(3)][-1]
        assert m["zone"] == "stop" and m["range_m"] == pytest.approx(1.3, abs=0.05), (R, m)
    r, p = GC.quat_to_roll_pitch(quat_from_R(Rx(math.radians(10))))
    assert abs(math.degrees(p)) == pytest.approx(10.0, abs=0.01) and abs(r) < 1e-9


# ---- the companion's supervisor ---------------------------------------------------------------------------------------------------
class _Task:
    def __init__(self):
        self.m = type("M", (), {"state": "APPROACH"})()
        self.name, self.blocked, self.events, self._beh, self._beh_state = "SCOUT", [], [], None, None

    def guard_target(self):
        return ("buoy", 7)

    def on_guard(self, ctx, ev):
        self.events.append(ev)

    def on_guard_block(self, ctx, key):
        self.blocked.append(key)


def _ctx(now, x=0.0, y=0.0, hdg=0.0, sog=0.5, pm=None):
    o = ControlOrigin(1.28, 103.85)
    pose = LocalPose(x, y, hdg, o.lat, o.lon, now, sog_mps=sog)
    return TaskContext(b=BehaviourContext(cfg=Config(), now=now, fcu=FcuState(), pose=pose, origin=o, run=RunContext(), map=pm), emit_event=lambda e: None,
                       log=lambda s: None)


STOP = {"clear": False, "zone": "stop", "range_m": 1.3, "bearing_deg": 2.0, "bearing_min_deg": -12.0, "bearing_max_deg": 14.0, "frames": 3, "px": 900}


def test_slow_caps_stop_brakes_at_once_backs_off_rings_and_replans():
    pm = PerceptionMap(Config(), clock=lambda: 0.0)
    g, t = GuardSupervisor({}), _Task()
    go = GuidedTarget(1.29, 103.85, 0.6)
    g.on_message({"clear": True}, 10.0)
    assert g.filter(t, go, _ctx(10.0, pm=pm))[0] is go                                         # clear: untouched
    g.on_message({"clear": False, "zone": "slow", "range_m": 2.6, "bearing_deg": 0.0, "frames": 0}, 10.1)
    assert g.filter(t, go, _ctx(10.1, pm=pm))[0].speed_mps == 0.3                             # SLOW: the leg capped
    g.on_message(STOP, 10.2)
    cmd, view = g.filter(t, go, _ctx(10.2, pm=pm))                                            # the same tick as the message (< 0.3 s)
    assert isinstance(cmd, ThrusterFrame) and cmd.m1 < 0 and cmd.m3 < 0 and view["chip"] == "BRAKE"
    assert t.events[-1]["action"] == "brake" and t.events[-1]["rings"] == 4                   # nearest + both ends (26 deg > 8) + the SLOW sighting
    assert len(pm.entities("obstacle", include_stale=True, include_young=True)) >= 2       # rings within 0.5 m merge
    assert isinstance(g.filter(t, None, _ctx(11.0, pm=pm, sog=0.2))[0], ThrusterFrame) and g.mode == "BRAKE"
    assert isinstance(g.filter(t, None, _ctx(11.8, pm=pm, sog=0.2))[0], ThrusterFrame) and g.mode == "BACKOFF"   # brake_s 1.5
    t._beh = type("B", (), {"cancel": lambda self: None})()
    assert isinstance(g.filter(t, None, _ctx(13.0, x=-1.0, pm=pm))[0], ThrusterFrame)                            # 1 m astern: on
    cmd, view = g.filter(t, None, _ctx(14.0, x=-2.05, pm=pm))                                                    # 2 m: done
    assert isinstance(cmd, Hold) and g.mode == "CLEAR" and t._beh is None                    # the behaviour restarts = the re-plan
    assert not t.blocked


def test_the_second_trigger_blocks_the_target_and_the_third_holds():
    g, t = GuardSupervisor({}), _Task()
    go = GuidedTarget(1.29, 103.85, 0.6)
    for k, now in enumerate((10.0, 30.0)):
        g.on_message(STOP, now)
        g.filter(t, go, _ctx(now))
        g.mode = "CLEAR"
    assert t.blocked == [("buoy", 7)] and t.events[-1]["blocked"] is True
    g.on_message(STOP, 50.0)
    cmd, _ = g.filter(t, go, _ctx(50.0))
    assert isinstance(cmd, Hold) and g.safe_hold and "obstacle persists" in g.safe_hold and t.events[-1]["action"] == "safe_hold"


def test_silence_caps_with_a_caution_and_the_quiet_stages_are_muted():
    g, t = GuardSupervisor({"require": True}), _Task()
    go = GuidedTarget(1.29, 103.85, 0.6)
    cmd, view = g.filter(t, go, _ctx(5.0))
    assert cmd.speed_mps == 0.3 and view["chip"] == "lost" and "guard lost" in view["caution"]
    g.on_message({"clear": True}, 6.0)
    assert g.filter(t, go, _ctx(6.5))[0] is go
    assert g.filter(t, go, _ctx(8.5))[0].speed_mps == 0.3                                     # silent 2.5 s > lost_s 1.5
    t.m.state = "LOG"
    g.on_message(STOP, 9.0)
    assert g.filter(t, go, _ctx(9.0))[0] is go                                                # LOG / WAIT_*: muted
    t.m.state = "APPROACH"
    assert g.filter(t, Hold(), _ctx(9.1))[0].__class__ is Hold                                # a hold is not motion
    assert GuardSupervisor({"require": False}).filter(t, go, _ctx(1.0))[0] is go              # never heard, not required: no cap


def test_a_low_thing_under_the_frame_is_remembered():
    """ZED 2i hd2k: +-15 deg vertical; 1.0 m up, a 0.5 m object leaves the frame inside 1.87 m - the SLOW sighting stops the boat."""
    g, t = GuardSupervisor({}), _Task()
    go = GuidedTarget(1.29, 103.85, 0.6)
    g.on_message({"clear": False, "zone": "slow", "range_m": 2.4, "bearing_deg": 0.0, "bearing_min_deg": -5.0, "bearing_max_deg": 5.0, "frames": 0}, 10.0)
    g.filter(t, go, _ctx(10.0, x=0.0))
    g.on_message({"clear": True}, 11.0)                                                       # gone under the frame's edge
    assert isinstance(g.filter(t, go, _ctx(11.0, x=0.5))[0], GuidedTarget)                    # 1.9 m: capped, not yet
    cmd, _ = g.filter(t, go, _ctx(11.5, x=1.0))                                               # 1.4 m: STOP from memory
    assert isinstance(cmd, ThrusterFrame) and t.events[-1].get("remembered") is True
    fg = FakeGuard({"camera": {"height_m": 1.0}, "objects": [{"kind": "obstacle", "x_n": 0.0, "y_e": 0.0, "x2_n": 0.0, "y2_e": 1.0, "height_m": 0.5}]},
                   {"buoys": False, "frames": 1})
    assert fg.frame(-2.0, 0.5, 0.0)["clear"] is False and fg.frame(-1.3, 0.5, 0.0)["clear"] is True     # the frame's lower edge


# ---- the fast SIL and the sim observer --------------------------------------------------------------------------------------------
def test_scout_with_the_guard_on_s15_brakes_rings_and_finishes(tmp_path):
    # v0.1.81.1: zed_guard.map false = the v0.1.80 behaviour (the obstacles only from the STOPs); with the depth map on: test_map_obstacles
    s = FastSil(WORLD, "SCOUT", {"polygon": POLY, "log_dir": str(tmp_path)}, guard={"sup": {"map": False}})
    r = s.run(max_s=900)
    assert r["state"] == "DONE" and not r["safe_holds"]
    obst = [l for l in s.task.lines if l["ev"] == "obstacle"]
    assert obst and all(o["action"] == "brake" for o in obst)
    assert s.map.entities("obstacle", include_stale=True, include_young=True)
    buoys = [l for l in s.task.lines if l["ev"] == "buoy"]
    assert len(buoys) == 10 and sum(b["visited"] for b in buoys) >= 9
    assert any("GUARD_BACKOFF" == p[4] for p in s.track)

    def segd(p, a, b):
        t = max(0.0, min(1.0, ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1])) / ((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)))
        return math.hypot(p[0] - a[0] - t * (b[0] - a[0]), p[1] - a[1] - t * (b[1] - a[1]))
    assert min(segd((p[1], p[2]), (23.5, -2.0), (23.5, 0.5)) for p in s.track) > 1.0         # the centre never within 1 m of the boom
    assert min(math.hypot(p[1] - 11.0, p[2] - 2.6) for p in s.track) > 1.0


def test_the_sim_observer_runs_the_real_filter_on_the_rendered_depth():
    spec = importlib.util.spec_from_file_location("so_guard", ROOT / "cv" / "sim_observer" / "sim_observer.py")
    so = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(so)
    from optiprimex.sim.fast_sil import CameraPath
    cp = CameraPath(W.load(WORLD), 3, (552, 310), "zed")
    fr = cp.frame(LocalPose(21.5, -0.75, 0.0, 1.28, 103.85, 0.0), 1.0, 1.28, 103.85)
    g = so.GC.GuardFilter({"h_cam_m": 1.0})
    m = g.frame(so.guard_cloud(fr))
    assert m["clear"] is False and m["range_m"] == pytest.approx(1.9, abs=0.2) and m["bearing_max_deg"] - m["bearing_min_deg"] > 20   # the boom
    assert "guard" in so.parse_args([]).__dict__ and so.parse_args([]).guard == "on"


# ---- m0t9 --------------------------------------------------------------------------------------------------------------------------
def test_m0t9_logs_the_verdicts_and_pulses_on_a_stop(tmp_path):
    now = [100.0]
    pm = PerceptionMap(Config(), clock=lambda: now[0])
    t = REGISTRY["TEST_GUARD"](Config(), clock=lambda: now[0])
    events = []

    def ctx():
        c = _ctx(now[0], pm=pm, sog=0.0)
        c.emit_event = lambda e: events.append(e.body)
        return c
    t.start({"duration_s": 30, "log_dir": str(tmp_path)}, ctx())
    sup = GuardSupervisor({})
    assert isinstance(t.step(ctx()).out.cmd, Hold) and events[-1]["verdict"] == "lost"
    sup.on_message({"clear": True}, now[0], pm)
    now[0] += 0.2
    assert isinstance(t.step(ctx()).out.cmd, Hold) and events[-1]["verdict"] == "clear"
    sup.on_message(dict(STOP, roll_deg=1.0, pitch_deg=-2.0, imu=True), now[0], pm)
    now[0] += 0.2
    out = t.step(ctx()).out
    assert isinstance(out.cmd, ThrusterFrame) and out.cmd.m1 == -0.4                           # the brake pulse (props in the air)
    assert [e["ev"] for e in events[-2:]] == ["verdict", "would"] and events[-2]["range_m"] == 1.3 and events[-2]["roll_deg"] == 1.0
    assert sup.filter(t, out.cmd, ctx())[0] is out.cmd                                         # the supervisor leaves m0t9 alone
    now[0] += 2.0
    sup.on_message({"clear": True}, now[0], pm)
    assert isinstance(t.step(ctx()).out.cmd, Hold)                                             # pulse_s 1.5 over: HOLD again
    now[0] = 131.0
    t.step(ctx())
    assert t.m.state == "DONE" and events[-1]["ev"] == "summary" and events[-1]["stops"] == 1
    t2 = REGISTRY["TEST_GUARD"](Config(), clock=lambda: now[0])
    t2.start({"pulse": 0, "log_dir": str(tmp_path)}, ctx())
    sup.on_message(STOP, now[0], pm)
    assert isinstance(t2.step(ctx()).out.cmd, Hold)                                            # pulse 0: never a motor


# ---- the wiring ---------------------------------------------------------------------------------------------------------------------
def test_the_wiring():
    mm = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text(encoding="utf-8")
    assert 'if topic.endswith("/guard"):' in mm and "self.guard.filter(self.runner.task, st.out.cmd, ctx)" in mm and "self.guard.safe_hold" in mm
    assert 'pub.c.publish("%s/perception/guard" % pub.ns' in (ROOT / "cv" / "publish" / "cv_publish.py").read_text(encoding="utf-8")
    obs = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text(encoding="utf-8")
    assert 'payload["guard"] = gfilt.frame(xyz, quat, time.time())' in obs and "sl.TIME_REFERENCE.IMAGE" in obs
    assert 'id="guardchip"' in (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text(encoding="utf-8")
    assert 'id="guard-now"' in (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text(encoding="utf-8")
    for f, req in (("pool/usv_pool.yaml", True), ("course/usv_course.yaml", True), ("bench/usv_bench.yaml", False)):
        y = yaml.safe_load((ROOT / "profiles" / f).read_text())
        assert y["zed_guard"]["stop_m"] == 1.5 and y["zed_guard"]["require"] is req and y["test_guard"]["pulse"] == 0.4
    from optiprimex.tools import playbook
    assert playbook.resolve("m0t9")[1] == "TEST_GUARD" and "m0t9" in playbook.MISSION_ORDER
    from optiprimex.companion.state import VehicleState
    vs = VehicleState()
    vs.guard = {"chip": "SLOW"}
    assert vs.status_body()["guard"] == {"chip": "SLOW"}
    assert json.dumps(GC.GuardFilter().frame(board()), allow_nan=False)
