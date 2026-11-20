"""v0.1.55 (WP10f-rest, docs/OVERNIGHT_2026-11-07.md §2 D1-D5): the commanded side in local/<v>/mm/state `cmd`, the boat window's
SET / speed tick / CONTROL / ACTUATION from it, the simulated host beat, the lamp read-back hook."""
from __future__ import annotations

import json
import types

from optiprimex.companion import link as L
from optiprimex.companion.config import Config
from optiprimex.companion.state import GuidedTarget, Hold, ThrusterFrame, VState, VehicleState
from optiprimex.tools import boat_web as BW
from optiprimex.tools import host_health as HH
from optiprimex.usv.mission_manager import MissionManager

from .conftest import ROOT


def _mm(state=VState.V4):
    cfg = Config()
    origin = types.SimpleNamespace(to_local=lambda lat, lon: ((lat - 1.0) * 111000.0, (lon - 103.0) * 111000.0))
    beh = types.SimpleNamespace(name="sway_transit")
    self = types.SimpleNamespace(cfg=cfg, origin=origin, runner=types.SimpleNamespace(task=types.SimpleNamespace(_beh=beh)),
                                 adapter=types.SimpleNamespace(mode_wanted="CUSTOM_MANUAL"))
    vs = VehicleState()
    vs.state, vs.task_state = state, "SWAY_SEARCH"
    vs.local = types.SimpleNamespace(x_n=0.0, y_e=0.0, hdg_deg=0.0)
    fcu = types.SimpleNamespace(mode_name="CUSTOM_MANUAL")
    return self, vs, fcu


def _st(cmd, info):
    return types.SimpleNamespace(out=types.SimpleNamespace(cmd=cmd, info=info), done=False, aborted=False)


def test_cmd_block_frame_target_hold_and_nulls():
    self, vs, fcu = _mm()
    d = MissionManager._cmd_view(self, vs, _st(ThrusterFrame(0.2, 0.1, 0.0, 0.1), {"tau": {"surge": 0.1, "sway": 0.2, "yaw": -0.05},
                                                                                       "hold_hdg": 270.0}), fcu, False)
    assert d["frame"] == {"surge": 0.1, "sway": 0.2, "yaw": -0.05} and d["hdg_goal_deg"] == 270.0 and d["target"] is None
    assert d["behaviour"] == "sway_transit" and d["stage"] == "SWAY_SEARCH" and d["mode"] == "CUSTOM_MANUAL"
    # nothing this tick: the last command stands
    d2 = MissionManager._cmd_view(self, vs, _st(None, {}), fcu, False)
    assert d2["frame"] == d["frame"]
    # GUIDED target with a carrot: bearing from the boat, the speed
    d = MissionManager._cmd_view(self, vs, _st(GuidedTarget(1.0001, 103.0, 0.8), {"carrot": [3.0, 4.0]}), fcu, False)
    assert d["frame"] is None and d["target"]["speed_mps"] == 0.8 and d["target"]["hdg_deg"] == 0.0
    assert d["target"]["carrot"] == {"x_n": 3.0, "y_e": 4.0}
    # Hold: all null
    d = MissionManager._cmd_view(self, vs, _st(Hold(), {"hold_hdg": 10}), fcu, False)
    assert d["frame"] is None and d["target"] is None and d["hdg_goal_deg"] is None
    # not AUTO_TASK, or usv_ui holding the FCU: all null
    for state, tui in ((VState.V2, False), (VState.V4, True)):
        self, vs, fcu = _mm(state)
        d = MissionManager._cmd_view(self, vs, _st(ThrusterFrame(0.2), {"tau": {"surge": 0.2, "sway": 0, "yaw": 0}}), fcu, tui)
        assert d["frame"] is None and d["target"] is None and d["hdg_goal_deg"] is None


def test_local_body_carries_cmd():
    vs = VehicleState()
    vs.cmd = {"frame": None}
    assert vs.local_body()["cmd"] == {"frame": None}
    assert "cmd" in VehicleState().local_body()


def _bw(now, cmd, mm_age=0.2):
    m = BW.Model("usv")
    m.on_message("local/usv/mm/state", json.dumps({"body": {"state": "AUTO_TASK", "cmd": cmd, "fcu": {}}}).encode(), now=now - mm_age)
    return m


def test_boat_window_set_and_speed_tick_sources():
    now = 500.0
    # mode 20: SET = the behaviour's goal, the tick = τ surge × max_speed
    m = _bw(now, {"frame": {"surge": 0.5, "sway": 0.1, "yaw": 0.0}, "target": None, "hdg_goal_deg": 90.0, "behaviour": "dock_los",
                  "stage": "ENTER", "max_speed_mps": 1.0})
    s = m.snapshot(now=now)
    c, sp = s["dials"]["compass"], s["dials"]["speed"]
    assert c["cmd"] == 90.0 and c["set"] == "090°" and c["set_src"] == "goal (dock_los)"
    assert sp["cmd_surge"] == 0.5 and "τ surge" in sp["text"][1]
    # GUIDED: the tick = target.speed_mps
    m = _bw(now, {"frame": None, "target": {"lat": 1, "lon": 2, "hdg_deg": 10, "speed_mps": 0.7, "carrot": None}, "hdg_goal_deg": None})
    sp = m.snapshot(now=now)["dials"]["speed"]
    assert sp["cmd_surge"] == 0.7 and "GUIDED" in sp["text"][1]
    # stale cmd (> 2 s): hatched
    m = _bw(now, {"frame": {"surge": 0.5, "sway": 0, "yaw": 0}, "hdg_goal_deg": 90.0}, mm_age=3.0)
    s = m.snapshot(now=now)
    assert s["dials"]["speed"]["cmd_surge"] is None and s["dials"]["compass"]["cmd"] is None
    assert isinstance(s["dials"]["speed"]["text"][1], dict) and "STALE" in s["dials"]["speed"]["text"][1]["u"]


def test_boat_window_control_chain_and_actuation_share():
    now = 500.0
    m = _bw(now, {"frame": {"surge": 0.4, "sway": 0.2, "yaw": 0.1}, "target": None, "hdg_goal_deg": 0.0, "behaviour": "sway_transit",
                  "stage": "SWAY_SEARCH"})
    blob = json.dumps(m.snapshot(now=now), ensure_ascii=False)
    assert "τ surge +0.40 · sway +0.20 · yaw +0.10" in blob
    assert "+50 % (τ, before stern factor / clip)" in blob and "+30 % (τ, before stern factor / clip)" in blob


def test_simulated_host_beat_and_label():
    b = HH.Sampler().body("agx", simulated=True)
    assert b["simulated"] is True and b["host"] == "agx"
    assert "simulated" not in HH.Sampler().body("nx")
    m = BW.Model("usv")
    env = HH.envelope(b, 1)
    m.on_message("local/usv/host/agx", json.dumps(env).encode(), now=10.0)
    e = m.electronics(10.5)
    assert e["simulated"] is True and "this Mac (simulated host)" in e["l3"]
    run_bench = (ROOT / "scripts" / "run_bench.sh").read_text()
    assert "host_health --host nx,agx --simulated" in run_bench
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "this Mac (simulated host)" in html


def test_lamp_readback_hook():
    m = L.MavLink(Config(), endpoint="udpin:127.0.0.1:0")
    m._apply("NAMED_VALUE_INT", types.SimpleNamespace(name="LAMP", value=3))
    assert m._state.lamp == 3
    bw = BW.Model("usv")
    now = 100.0
    bw.on_message("local/usv/ui/fcu", json.dumps({"body": {"hb_age_s": 0.2, "mode": "HOLD", "lamp": 2, "lamp_age_s": 0.3}}).encode(), now=now)
    bw.on_message(f"{BW.NS}/status/usv", json.dumps({"body": {"state": "MANUAL"}}).encode(), now=now)
    lp = bw.lamp(now + 0.1)
    assert lp["state"] == "read" and lp["colour"] == "YELLOW" and "NAMED_VALUE_INT LAMP = 2" in lp["l2"]
    bw2 = BW.Model("usv")
    bw2.on_message("local/usv/ui/fcu", json.dumps({"body": {"hb_age_s": 0.2, "mode": "HOLD"}}).encode(), now=now)
    bw2.on_message(f"{BW.NS}/status/usv", json.dumps({"body": {"state": "MANUAL"}}).encode(), now=now)
    assert bw2.lamp(now + 0.1)["state"] == "expected"


def test_meta_json_host_thresholds_in_ocs_web():
    src = (ROOT / "optiprimex" / "tools" / "ocs_web.py").read_text()
    assert "host_thresholds={" in src
    m = BW.Model("usv")
    assert m.thr_src == "flags"
