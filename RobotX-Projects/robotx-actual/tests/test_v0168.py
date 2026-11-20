"""v0.1.68 (WP17) - FREE RUN: the ghost plant, the muted adapter (the invariant F3), skip, the armed guard, the flag's guards.

docs/PROMPT_FREE_RUN_2026-11-08.md F1-F9; the bench acceptance is docs/TEST_GUIDE_VENUE_2026-11-08.md H2b.
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time

import pytest

from optiprimex.companion.config import load
from optiprimex.companion.state import Command, FcuState, VState

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
LAB = os.path.join(ROOT, "profiles", "pool", "usv_lab.yaml")
POOL = os.path.join(ROOT, "profiles", "pool", "usv_pool.yaml")
WORLD = os.path.join(ROOT, "profiles", "bench", "world_pool.yaml")
PY = sys.executable

MOTION = ("send_actuator_frame", "send_position_target_global", "send_velocity_yaw", "do_change_speed")


class StubLink:
    """The real FCU as the bench has it indoors: heartbeat, disarmed, HOLD, no fix; GUIDED / CUSTOM_MANUAL refused. Any motion
    sender called = the F3 invariant broken (recorded, and the test fails)."""

    def __init__(self, accept=("HOLD", "MANUAL")):
        self.accept = set(accept)
        self.mode_name = "HOLD"
        self.armed = False
        self.gps_fix = 0
        self.modes = []
        self.motion = []
        self.t = 0.0

    def start(self): pass
    def stop(self): pass
    def param_request(self, name): pass

    def state(self) -> FcuState:
        f = FcuState(seen_heartbeat=True, mode_name=self.mode_name, armed=self.armed, ekf_pos_ok=False, rc_chancount=16, rc_ok=True,
                     gps_fix=self.gps_fix, t_mono=self.t)
        f.rc = {5: 1000, 6: 1000, 8: 1500}
        f.ages = {"HEARTBEAT": 0.1, "RC_CHANNELS": 0.1, "SYS_STATUS": 0.5, "GLOBAL_POSITION_INT": 99.0}
        return f

    def set_mode(self, name, timeout_s=None):
        ok = name in self.accept
        self.modes.append((name, ok))
        if ok:
            self.mode_name = name
        return ok

    def __getattr__(self, name):
        if name in MOTION:
            def _motion(*a, **k):
                self.motion.append(name)
                return True
            return _motion
        raise AttributeError(name)


class StubBus:
    host, port = "127.0.0.1", 0

    def __init__(self):
        self.pub = []

    def subscribe(self, *a, **k): pass
    def watch_bridge(self): pass
    def start(self): pass
    def stop(self): pass
    def bridge_state(self, name=None): return True
    def ack(self, *a, **k): pass
    def publish_env(self, *a, **k): pass

    def publish(self, topic, kind, body, **k):
        self.pub.append((topic, body))
        return "id"


def cmd(action, task=None, **args) -> Command:
    env = {"msg_id": f"m-{action}-{time.monotonic_ns()}", "run_id": None, "declaration_seq": None, "vehicle_id": "USV1", "kind": "cmd", "body": {}}
    return Command(env=env, action=action, task=task, args=args)


class Rig:
    """A real MissionManager (lab profile, free run) on the stub link, in simulated time, the geometric fake camera on world_pool."""

    def __init__(self, armed_policy="refuse", accept=("HOLD", "MANUAL"), ghost=True, start=None):
        from optiprimex.sim import world as W
        from optiprimex.sim.fake_dock import DockSim
        from optiprimex.usv.ghost import GhostNav, world_start
        from optiprimex.usv.mission_manager import MissionManager
        from optiprimex.sim.fake_perception import FakePerception, parse_args as fp_args
        self.cfg = load(LAB)
        self.cfg.guards.require_run_for_task = False
        self.cfg.guards.require_prepare_for_auto = False
        self.cfg.extra.setdefault("free_run", {})["armed"] = armed_policy
        self.t = 1000.0
        self.link, self.bus = StubLink(accept), StubBus()
        wd = W.load(WORLD)
        st = world_start(WORLD)
        if start:                                         # a teleport of the world (x_n, y_e, hdg) - as --free-start would give
            lat, lon = W.WorldFrame(wd["origin"]).to_latlon(start[0], start[1])
            st = (lat, lon, start[2])
        self.mm = MissionManager(self.cfg, link=self.link, bus=self.bus, clock=lambda: self.t,
                                 ghost=GhostNav(st) if ghost else None)
        self.fp = FakePerception(fp_args(["--world", WORLD, "--hub-port", "0", "--seed", "3"]))
        self.dock = DockSim(wd, 3)
        self.origin_w = W.WorldFrame(wd["origin"])
        self.n = 0

    def ctx(self):
        from optiprimex.companion.state import RunContext
        from optiprimex.usv.behaviours.base import BehaviourContext
        from optiprimex.usv.tasks.base import TaskContext
        mm = self.mm
        fcu = mm._ghost_fcu(self.link.state()) if mm.ghost else self.link.state()
        return TaskContext(b=BehaviourContext(cfg=self.cfg, now=self.t, fcu=fcu, pose=mm.vs.local, origin=mm.origin, run=mm.vs.run,
                                              map=mm.map), emit_event=mm.events.emit, log=lambda s: None)

    def on_docking(self):
        for (t, b) in self.bus.pub:
            if t.endswith("/event/usv/docking") and not getattr(self, "_docked", False):
                self._docked = True
                self.dock.command({"cmd": "docked", "bay": b.get("bay")}, self.t)

    def do(self, action, task=None, **args):
        return self.mm.cmd._dispatch(cmd(action, task, **args), self.ctx())

    def tick(self, n=1):
        for _ in range(n):
            self.link.t = self.t
            self.mm.tick()
            pz = self.mm.vs.local
            if pz is not None and self.n % 2 == 0:
                g = self.mm.ghost
                if g is not None:                          # the world frame from the ghost's lat / lon (what fake_perception does)
                    x, y = self.origin_w.to_world(g.lat, g.lon)
                else:
                    x, y = pz.x_n, pz.y_e
                self.fp.pose, self.fp.pose_t = {"x_n": x, "y_e": y, "hdg_deg": pz.hdg_deg, "lat": None, "lon": None}, time.monotonic()
                aim = None
                if self.mm.runner.task is not None and self.mm.vs.state == VState.V4:
                    aim = (getattr(self.mm.runner.last_status, "out", None) and self.mm.runner.last_status.out.info.get("cannon"))
                self.dock.tick(self.t, {"x_n": x, "y_e": y, "hdg_deg": pz.hdg_deg}, aim, 0.0 if aim else 1e9)
                self.fp.dock_state = self.dock.state(self.t)
                for obs in self.fp.observe(self.t).values():
                    self.mm.map.ingest(obs, pz, now=self.t)
            self.on_docking()
            self.t += 0.1
            self.n += 1

    def auto(self):
        self.tick(3)
        assert self.mm.fsm.state == VState.V2, self.mm.fsm.state
        assert self.do("ENABLE_AUTO")[0] == "ACCEPTED"
        self.tick(2)
        assert self.mm.fsm.state == VState.V3

    def run_until(self, pred, max_s):
        for _ in range(int(max_s * 10)):
            self.tick()
            if pred():
                return True
        return False


def _events(rig, kind):
    return [b for (t, b) in rig.bus.pub if t.endswith("/event/usv/" + kind)]


# ---- F3: the invariant ----------------------------------------------------------------------------------------------------
def test_f3_whole_m1t1_on_the_ghost_no_motion_to_the_fcu():
    """A stub link + a full m1t1 on the ghost: the four motion senders never called, set_mode called (GUIDED refused and ignored),
    DONE with a SafePassageReport; the pose is the ghost's, the origin set without a fix."""
    rig = Rig()
    assert rig.mm.origin is not None and rig.mm.vs.nav == "ghost" and rig.mm.vs.motion == "muted"
    rig.auto()
    assert rig.do("START_TASK", "T1", tier="core")[0] == "ACCEPTED"
    done = rig.run_until(lambda: rig.mm.fsm.state == VState.V3 and not rig.mm.runner.active, 400)
    st = rig.mm.vs
    assert done, (st.state, st.task_name, st.task_state)
    assert rig.link.motion == [], rig.link.motion[:5]
    asked = {m for m, ok in rig.link.modes}
    assert "GUIDED" in asked and ("GUIDED", False) in rig.link.modes          # requested, refused, the ghost went on
    assert rig.mm.adapter.sent["frames"] == rig.mm.adapter.sent["targets"] == rig.mm.adapter.sent["velocity"] == 0
    assert rig.mm.ghost.counts["target"] > 10                                 # the behaviours drove the ghost
    rep = _events(rig, "safe_passage")
    assert rep, "no SafePassageReport"
    b = rig.mm.vs.status_body()
    assert b["nav"] == "ghost" and b["motion"] == "muted" and b["free_run"]["sent"]["frames"] == 0
    assert abs(b["lat"] - rig.mm.ghost.lat) < 1e-5                            # status carries the ghost's position


def test_f3_refusal_logged_once_per_stage_and_mode(caplog):
    rig = Rig()
    rig.auto()
    rig.do("START_TASK", "T1", tier="core")
    with caplog.at_level("WARNING", logger="optiprimex.rover"):
        rig.run_until(lambda: rig.mm.runner.sub_state == "CIRCLE_ENTRY_CW", 120)
        rig.tick(80)                                     # 8 s in the stage: GUIDED re-asked every hold_reassert_s, logged once
    lines = [r.getMessage() for r in caplog.records if "FREE RUN: the FCU refused" in r.getMessage()]
    keys = [(l.split(" refused ")[1].split(" in ")[0], l.split(" in ")[1].split(" - ")[0]) for l in lines]
    assert len(keys) == len(set(keys)) and keys, lines


def test_f3_mode20_entry_and_cancel_frames_muted():
    """The zero frame on mode-20 entry and the neutral frame on cancel never reach the link in free run; they reach the ghost."""
    from optiprimex.companion.state import ThrusterFrame
    from optiprimex.usv.adapters.rover import RoverAdapter
    from optiprimex.usv.ghost import GhostNav
    cfg = load(LAB)
    link = StubLink()
    ad = RoverAdapter(link, cfg)
    ad.ghost = GhostNav((1.28, 103.85, 0.0))
    assert ad.direct(ThrusterFrame(m1=0.3, m2=0.0, m3=0.3, m4=0.0))
    ad.cancel()
    assert link.motion == [] and ("CUSTOM_MANUAL", False) in link.modes and link.modes[-1] == ("HOLD", True)
    assert ad.ghost.counts["frame"] >= 3 and ad.ghost.kind == "hold"           # entry, cmd, neutral; then hold
    ad2 = RoverAdapter(StubLink(accept=("HOLD", "CUSTOM_MANUAL")), cfg)       # the flag off: as before (entry + cmd + neutral frames)
    ad2.direct(ThrusterFrame(m1=0.3, m2=0.0, m3=0.3, m4=0.0))
    ad2.cancel()
    assert ad2.link.motion == ["send_actuator_frame"] * 3


# ---- F2: the ghost plant ---------------------------------------------------------------------------------------------------
def test_f2_plant_moved_names_and_numbers_unchanged():
    from optiprimex.sim import fake_fcu, plant
    assert fake_fcu.Plant is plant.Plant and fake_fcu.R_EARTH == plant.R_EARTH
    p = plant.Plant(1.28, 103.85, 30.0)
    for _ in range(50):
        u, r = plant.guided_ur(p, (1.2801, 103.8502), 2.0)
        p.step(0.1, u, 0.0, r)
    assert p.x > 1.0 and p.u > 0.1
    assert plant.frame_uvr([0.5, 0.0, 0.5, 0.0]) == (0.75, 0.0, 0.0)


def test_f2_start_order_and_dt_clamp():
    from optiprimex.usv.ghost import GhostNav, parse_start, resolve_start, world_start
    assert parse_start("1.5,103.5,90") == (1.5, 103.5, 90.0)
    with pytest.raises(ValueError):
        parse_start("1.5")
    w = world_start(WORLD)
    assert abs(w[0] - 1.2810202) < 1e-7 and abs(w[1] - 103.8550097) < 1e-7
    assert resolve_start("1,2,3", WORLD, {"start": {"lat": 5, "lon": 6}})[1] == "--free-start"
    assert resolve_start(None, WORLD, {"start": {"lat": 5, "lon": 6}})[1].startswith("--world")
    assert resolve_start(None, None, {"start": {"lat": 5, "lon": 6, "hdg_deg": 7}}) == ((5.0, 6.0, 7.0), "free_run.start")
    g = GhostNav((1.28, 103.85, 0.0))
    assert g.step(10.0) == 0.0 and g.step(15.0) == 0.5                          # the first tick 0, then clamped to 0.5 s
    g.set_vel(1.0, 0.0, 15.0)
    g.step(15.5)
    assert g.sog_mps > 0.1
    g.set_hold()
    assert g.targets(15.6) == (0.0, 0.0, 0.0)


def test_f2_indoor_fix_shown_not_used():
    rig = Rig()
    rig.link.gps_fix = 3                                   # a fix appears indoors: the pose stays the ghost's
    rig.tick(5)
    fr = rig.mm.vs.status_body()["free_run"]
    assert fr["gps_note"] == "gps fix 3 (ignored: free run)"
    assert abs(rig.mm.vs.local.x_n) < 0.01 and abs(rig.mm.vs.local.y_e) < 0.01


# ---- F4: skip ------------------------------------------------------------------------------------------------------------
NEAR_DOCK = (32.0, 3.0, 0.0)


def _m3t1_to(rig, stage, max_s=200):
    rig.auto()
    assert rig.do("START_TASK", "T3")[0] == "ACCEPTED"
    assert rig.run_until(lambda: rig.mm.runner.sub_state == stage, max_s), rig.mm.runner.sub_state


def test_f4_skip_on_a_motion_stage_goes_to_the_next():
    rig = Rig(start=NEAR_DOCK)
    _m3t1_to(rig, "SWAY_SEARCH")
    rig.tick(10)
    assert rig.mm.vs.status_body()["free_run"]["skip"] == {"ok": True, "why": ""}
    task = rig.mm.runner.task
    res, why, _ = rig.do("SKIP")
    assert res == "ACCEPTED" and "SWAY_SEARCH ended by skip" in why, why
    # the task's own next event fired: the pass's decision (T3._decide) - the chosen bay's stage, or (no GREEN yet) the second pass
    assert len(task.search_log) == 1 and (rig.mm.runner.sub_state != "SWAY_SEARCH" or task.pass_no == 2), task.search_log


def test_f4_skip_refused_on_a_map_stage_and_outside_free_run():
    rig = Rig()                                            # from the world's start: FIND_DOCK scans a while
    _m3t1_to(rig, "FIND_DOCK", 30)
    res, why, _ = rig.do("SKIP")
    assert (res, why) == ("REJECTED", "skip: FIND_DOCK ends on the map, not on motion")
    sk = rig.mm.vs.status_body()["free_run"]["skip"]
    assert sk["ok"] is False and "ends on the map" in sk["why"]
    real = Rig(ghost=False)                               # the flag off: free run only
    real.tick(3)
    real.do("ENABLE_AUTO")
    real.do("START_TASK", "TEST_MANUAL")
    assert real.do("SKIP")[:2] == ("REJECTED", "skip: free run only")
    assert "nav" not in real.mm.vs.status_body()
    none = Rig()
    assert none.do("SKIP")[:2] == ("REJECTED", "SKIP needs a running task (now none)")


def test_f4_skip_with_step_parks_before_the_next_stage():
    rig = Rig(start=NEAR_DOCK)
    rig.auto()
    assert rig.do("START_TASK", "T3", step=True)[0] == "ACCEPTED"
    seen = []
    for _ in range(4000):
        rig.tick()
        mm = rig.mm
        if mm.fsm.state == VState.V5 and (mm.fsm.hold_reason or "").startswith("step: next"):
            seen.append(mm.fsm.hold_reason)
            if mm.fsm.hold_reason == "step: next TO_LINE" or len(seen) > 6:
                break
            assert rig.do("RESUME")[0] == "ACCEPTED"
    assert seen[-1] == "step: next TO_LINE", seen
    assert rig.do("SKIP")[:2] == ("REJECTED", "skip: TO_LINE - skip needs AUTO_TASK (now AUTO_SAFE_HOLD; resume first)")
    assert rig.do("RESUME")[0] == "ACCEPTED"
    rig.tick(5)
    assert rig.mm.runner.sub_state == "TO_LINE"
    assert rig.do("SKIP")[0] == "ACCEPTED"                 # TO_LINE ended now -> SWAY_SEARCH, parked before it (step mode)
    rig.tick(3)
    assert rig.mm.fsm.state == VState.V5 and rig.mm.fsm.hold_reason == "step: next SWAY_SEARCH", rig.mm.fsm.hold_reason


# ---- F5: the armed guard ------------------------------------------------------------------------------------------------------
def test_f5_refuse_at_start_and_abort_mid_task():
    rig = Rig()
    rig.auto()
    rig.link.armed = True
    rig.tick(2)
    assert rig.do("START_TASK", "T1", tier="core")[:2] == ("REJECTED", "FREE RUN: the FCU is armed (free_run.armed refuse; disarm, or warn in the profile for the kill test)")
    rig = Rig()                                            # (disarming in AUTO is the health monitor's UNEXPECTED_DISARM: a fresh rig)
    rig.auto()
    assert rig.do("START_TASK", "T1", tier="core")[0] == "ACCEPTED"
    rig.tick(30)
    rig.link.armed = True                                  # the transmitter arms mid-task
    rig.tick(2)
    assert rig.mm.fsm.state == VState.V5 and rig.mm.fsm.hold_reason == "FREE RUN: FCU armed" and not rig.mm.runner.active
    al = _events(rig, "alert")
    assert al and al[-1]["policy"] == "refuse" and "aborted" in al[-1]["text"]
    assert rig.link.motion == []


def test_f5_warn_event_banner_task_goes_on():
    rig = Rig(armed_policy="warn")
    rig.auto()
    assert rig.do("START_TASK", "T1", tier="core")[0] == "ACCEPTED"
    rig.tick(30)
    rig.link.armed = True
    rig.tick(20)
    assert rig.mm.fsm.state == VState.V4 and rig.mm.runner.active
    assert rig.mm.vs.status_body()["free_run"]["banner"] == "FREE RUN · FCU ARMED"
    al = _events(rig, "alert")
    assert len(al) == 1 and al[0]["policy"] == "warn"
    assert rig.link.motion == []


# ---- F1: the flag's guards -------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("profile", ["usv_pool.yaml", "usv_course.yaml"])
def test_f1_free_run_refused_without_the_block(profile):
    path = os.path.join(ROOT, "profiles", "pool", profile)
    if not os.path.exists(path):
        path = os.path.join(ROOT, "profiles", "course", profile)
    if not os.path.exists(path):
        pytest.skip(f"{profile} not in this tree")
    r = subprocess.run([PY, "-m", "optiprimex.usv.mission_manager", "--config", path, "--free-run"], capture_output=True, text=True,
                       timeout=60, cwd=ROOT)
    assert r.returncode == 2 and "--free-run needs a profile with free_run.allowed (the lab profile); never at the pool" in r.stderr


def test_f1_lab_profile_has_the_block():
    cfg = load(LAB)
    fr = cfg.extra["free_run"]
    assert fr["allowed"] is True and fr["armed"] == "refuse"
    assert abs(fr["start"]["lat"] - 1.2810202) < 1e-7 and abs(fr["start"]["lon"] - 103.8550097) < 1e-7
    assert "free_run" not in load(POOL).extra


def _sh(args, env=None):
    e = dict(os.environ)
    e.update(env or {})
    return subprocess.run(["bash", os.path.join(ROOT, "scripts", "run_usv.sh")] + args, capture_output=True, text=True, timeout=60,
                          cwd=ROOT, env=e)


def test_f1_run_usv_refusals():
    r = _sh(["--free-run", "--fcu", "uart"])
    assert r.returncode == 2 and "--free-run needs --lab" in (r.stdout + r.stderr)
    r = _sh(["--free-run", "--lab", "--fcu", "fake"])
    assert r.returncode == 2 and "the fake plant already has a position" in (r.stdout + r.stderr)


# ---- F9: the fake FCU without a fix --------------------------------------------------------------------------------------------
def test_f9_fake_fcu_refuses_guided_without_a_fix():
    import argparse
    from optiprimex.sim.fake_fcu import FakeFcu, parse_args
    a = parse_args(["--conn", "udpout:127.0.0.1:1", "--gps-fix", "0"])
    f = FakeFcu(a)
    assert f.gps_fix == 0
    assert f.set_mode(15) is False and f.mode == 4               # GUIDED refused (ArduRover: no position estimate)
    assert f.set_mode(20) is True                               # mode 20 accepted (no position needed)
    f.apply_control({"set": {"gps_fix": 3}})
    assert f.set_mode(15) is True
    sent = []
    f.send = lambda fn, *args: sent.append(fn)
    f.gps_fix = 0
    f.emit(33)                                                  # GLOBAL_POSITION_INT: not sent without a fix
    assert sent == []


# ---- F8: mode-log on a free-run trace --------------------------------------------------------------------------------------------
def test_f8_mode_log_reads_a_free_run_trace(tmp_path):
    from optiprimex.tools import mode_log
    from optiprimex.usv.cmd_trace import CmdTrace
    rig = Rig()
    tr = str(tmp_path / "mm_trace.jsonl")
    rig.mm.adapter.trace = CmdTrace(tr, profile="usv_lab.yaml", fcu="udpin:127.0.0.1:14551")
    rig.auto()
    rig.do("START_TASK", "T1", tier="core")
    rig.run_until(lambda: rig.mm.runner.sub_state == "TRANSIT", 120)
    rig.tick(30)
    rig.mm.adapter.trace.close()
    head, evs = mode_log.load_trace(tr)
    r = mode_log.analyse(head, evs, "lab")
    assert r["free_run"] and r["ok"], [f.text for f in r["findings"] if f.level == "FAIL"]
    txt = mode_log.render_text(r)
    assert txt.startswith("FREE RUN") and "GUIDED (refused)" in txt and "target " in txt
    st = [s for s in r["stages"] if s.sub == "TRANSIT"]
    assert st and st[0].ghost.get("target", 0) > 0 and st[0].frames == 0
    refused = [f for f in r["findings"] if f.check == "refused"]
    assert len(refused) == len({f.text for f in refused})                  # once per stage and mode
    assert "**FREE RUN**" in mode_log.render_md(r)
    with open(tr, "a") as fh:                                              # a motion record in a free-run trace = FAIL
        fh.write(json.dumps({"k": "target", "t": 999.0, "lat": 1.0, "lon": 2.0}) + "\n")
    r2 = mode_log.analyse(*mode_log.load_trace(tr), "lab")
    assert not r2["ok"] and any(f.check == "motion" for f in r2["findings"])


# ---- F6: what everyone sees ----------------------------------------------------------------------------------------------------
def test_f6_op_tui_free_run_banner_200x60():
    from optiprimex.tools import bench_tui
    from .test_playbook_tui import ocs
    m = bench_tui.Model("USV1", "usv")
    view = bench_tui.View(m, "USV1")
    m.on_hub("ocs/state", json.dumps({"body": ocs("R2", "IDLE", None, None)}).encode())
    m.hub_up = True
    rig = Rig()
    rig.tick(3)
    m.on_hub("team/SGIT/status/usv", json.dumps({"body": rig.mm.vs.status_body()}).encode())
    c = bench_tui.Canvas(200, 60)
    view.draw(c, 0, "", "")
    text = "\n".join(c.lines())
    assert "FREE RUN  ·  ghost position, no motion to the FCU" in text, text[:400]
    assert "skip" in bench_tui.INTENT_WORDS                                  # typed (the panel row stays as it was: 150-wide)


def test_f6_status_only_in_free_run():
    rig = Rig(ghost=False)
    rig.tick(2)
    b = rig.mm.vs.status_body()
    assert "nav" not in b and "motion" not in b and "free_run" not in b
    rig = Rig()
    rig.tick(2)
    b = rig.mm.vs.status_body()
    assert (b["nav"], b["motion"]) == ("ghost", "muted")
    loc = rig.mm.vs.local_body()                                          # mm/state carries the ghost pose (the fake CV follows it)
    assert abs(loc["lat"] - 1.2810202) < 1e-6 and abs(loc["lon"] - 103.8550097) < 1e-6


def test_f6_boat_window_free_run_banner():
    from .test_v0152_wa import BW, _m
    s = _m(None, {"state": "AUTO_HOLD", "fcu_link": "real", "nav": "ghost", "motion": "muted", "free_run": {}}).snapshot(now=100.2)["sim"]
    assert s["on"] and s["free_run"] and s["text"].startswith("FREE RUN · ghost position")
    s = _m(None, {"state": "AUTO_HOLD", "nav": "ghost", "free_run": {"banner": "FREE RUN · FCU ARMED"}}).snapshot(now=100.2)["sim"]
    assert s["why"][-1] == "FREE RUN · FCU ARMED"
    page = open(os.path.join(ROOT, "optiprimex", "tools", "boat_web.html")).read()
    assert "fr?'FREE RUN':'SIMULATION'" in page


def test_f4_f6_ocs_page_skip_button_and_banner():
    page = open(os.path.join(ROOT, "optiprimex", "tools", "ocs_web.html")).read()
    assert 'id="c-skip" hidden' in page and 'send("skip")' in page
    assert '"SKIP refused now: "+(k.why' in page and 'b.classList.toggle("no",!k.ok)' in page          # greyed with the reason
    assert 'st.nav==="ghost"' in page and '"FREE RUN"+' in page
    mp = open(os.path.join(ROOT, "optiprimex", "tools", "map.html")).read()
    assert "b.nav === 'ghost'" in mp and "setLineDash(ghost ? [4, 3] : [])" in mp
    rig = Rig()
    rig.tick(2)
    assert rig.bus.pub and any(t.endswith("/telemetry/usv/map") and b["boat"].get("nav") == "ghost" for t, b in rig.bus.pub)


def test_f4_skip_word_everywhere():
    from optiprimex.ocs.cmd_authority import GUARDS
    from optiprimex.tools import bench_tui, ocs_cli, playbook
    assert GUARDS["SKIP"] == GUARDS["RESUME"][:2] + ("SKIP", "run", False)
    assert "SKIP" in playbook.PLAYBOOK and "skip" in bench_tui.INTENT_WORDS and bench_tui.WORD_TO_ACTION["skip"] == "SKIP"
    schema = json.load(open(os.path.join(ROOT, "optiprimex", "schemas", "bodies.schema.json")))
    assert "SKIP" in json.dumps(schema)
    ui = open(os.path.join(ROOT, "usv_ui", "usv_ui.py")).read()
    assert 'elif cmd in ("reset", "step", "skip"):' in ui


def test_f4_skip_refused_on_the_course_network():
    from optiprimex.ocs.cmd_authority import CmdAuthority
    from optiprimex.ocs.run_lifecycle import RunLifecycle
    res = []
    for course in (True, False):
        a = CmdAuthority(StubBus(), RunLifecycle(), {"USV1": "usv"}, True, lambda c=course: c, res.append)
        a.emit = lambda *x, **k: "id"
        r = a.on_intent("ui/cmd/usv", {"kind": "intent", "msg_id": f"i{course}", "vehicle_id": "USV1", "body": {"action": "SKIP"}})
        if course:
            assert r.result == "refused" and "never used on the course network" in r.reason
        else:
            assert r.result == "accepted" or "course network" not in r.reason


def test_f6_declare_refused_for_a_ghost_on_the_course_judge():
    from .test_v0164 import make_app
    app, bus, judge, res = make_app()
    app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL", "nav": "ghost"}})
    judge.network = "course"
    assert app.declare() == "free run boat on the course judge (USV1: nav ghost) - restart it without --free-run"
    judge.network = "bench"
    assert app.declare() is None                                            # the bench judge: free run is fine (rc-test, the fake judge)


# ---- F7: the launcher ---------------------------------------------------------------------------------------------------------------
def test_f7_launcher_field_plan_and_refusal():
    from optiprimex.tools import launcher as L
    P = L.load_presets()
    assert P["lab"]["free_run"] == "off" and P["custom"]["free_run"] == "off"
    assert "free_run" not in P["sim"] and "free_run" not in P["event"]
    cfg = dict(P["lab"], free_run="on", preset="lab")
    assert L.validate(cfg) == []
    p = L.build_plan(cfg, stamp="x", net_env={}, pre={"clocks": {}, "nic": None, "refuse": []})
    nx = [s for s in p["steps"] if s["host"] == "nx"][0]
    assert "--lab --free-run --world profiles/bench/world_pool.yaml" in nx["cmd"] and "FREE RUN (ghost position, no motion to the FCU)" in nx["label"]
    agx = [s for s in p["steps"] if s["host"] == "agx" and "perception" in s["label"]][0]
    assert "--sim --world profiles/bench/world_pool.yaml" in agx["cmd"]
    assert p["notes"][0].startswith("FREE RUN (ghost position, no motion to the FCU)") and not p["refuse"]
    pc = L.build_plan(dict(cfg, judge="course", rc_host="192.168.65.2"), stamp="x", net_env={}, pre={"clocks": {}, "nic": None, "refuse": []})
    assert any(r.startswith("FREE RUN with judge course: refused") for r in pc["refuse"])
    assert "free run: a real FCU (uart / usb / eth) - the fake plant already has a position" in L.validate(dict(P["custom"], free_run="on"))
    off = L.build_plan(dict(P["lab"], preset="lab"), stamp="x", net_env={}, pre={"clocks": {}, "nic": None, "refuse": []})
    assert "--free-run" not in json.dumps(off["steps"])
