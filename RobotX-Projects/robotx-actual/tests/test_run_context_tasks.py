"""Run context rules, the T4 task chain and the breakpoint detour, without any broker."""
from __future__ import annotations

from typing import List

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin
from optiprimex.companion.state import Command, Event, FcuState, LocalPose, RunContext
from optiprimex.usv import run_context as rc
from optiprimex.usv.behaviours.base import BehaviourContext
from optiprimex.usv.task_runner import TaskRunner
from optiprimex.usv.tasks.base import TaskContext


def cmd(action, run_id=None, seq=None, **args):
    env = {"msg_id": "m", "run_id": run_id, "declaration_seq": seq, "vehicle_id": "USV1", "kind": "cmd", "body": {}}
    return Command(env=env, action=action, args=args)


def test_run_context_rules():
    run = RunContext()
    assert rc.accepts(run, cmd("START_RUN", run_id=5))[0] is False          # before PREPARE
    assert rc.apply(run, cmd("PREPARE", declaration_seq=1, tiers={"T4": "TIER_CORE"})) == (True, "")
    assert run.declaration_seq == 1 and run.tiers == {"T4": "TIER_CORE"}
    assert rc.accepts(run, cmd("START_TASK", seq=2))[0] is False             # wrong declaration_seq
    assert rc.accepts(run, cmd("START_TASK", seq=1))[0] is True
    assert rc.accepts(run, cmd("HOLD", seq=2))[0] is True                    # reducing autonomy is never run-scoped
    assert rc.accepts(run, cmd("START_TASK", run_id=1001))[0] is False      # run unknown
    assert rc.apply(run, cmd("START_RUN", run_id=1001)) == (True, "")
    assert rc.accepts(run, cmd("START_TASK", run_id=1001, seq=1))[0] is True
    assert rc.accepts(run, cmd("START_TASK", run_id=1000, seq=1))[0] is False
    assert rc.accepts(run, cmd("PREPARE", run_id=999, seq=9))[0] is True     # run-free action
    rc.apply(run, cmd("STOP_RUN"))
    assert run.run_id is None and rc.accepts(run, cmd("START_TASK", run_id=1001))[0] is False   # nothing from the old run
    assert rc.accepts(run, cmd("ABORT_TASK", run_id=1001))[0] is True and rc.accepts(run, cmd("DISABLE_AUTO", run_id=1001))[0] is True


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class Rig:
    """Minimal in-process rig: a clock, a pose we move by hand, a list of emitted events."""

    def __init__(self, allow_goto=True):
        self.cfg = Config()
        self.cfg.guards.allow_goto_task = allow_goto
        self.clock = Clock()
        self.origin = ControlOrigin(1.3, 103.8)
        self.fcu = FcuState(seen_heartbeat=True, lat=1.3, lon=103.8, hdg_deg=0.0, sog_mps=0.0, gps_fix=3, armed=True)
        self.events: List[Event] = []
        self.transitions = []
        self.runner = TaskRunner(self.cfg, on_transition=self.transitions.append, clock=self.clock)

    def move_to(self, x_n, y_e, sog=0.0):
        self.fcu.lat, self.fcu.lon = self.origin.from_local(x_n, y_e)
        self.fcu.sog_mps = sog

    def ctx(self) -> TaskContext:
        pose = self.origin.pose_from_fcu(self.fcu)
        return TaskContext(b=BehaviourContext(cfg=self.cfg, now=self.clock(), fcu=self.fcu, pose=pose, origin=self.origin,
                                              run=RunContext(run_id=1001, declaration_seq=1)), emit_event=self.events.append)

    def steps(self, n=1, dt=0.1):
        st = None
        for _ in range(n):
            self.clock.t += dt
            st = self.runner.step(self.ctx())
        return st

    def kinds(self):
        return [(e.kind, e.body.get("kind")) for e in self.events if e.kind == "incident"]


def test_t4_core_chain_standalone():
    r = Rig()
    r.runner.start("T4", {}, r.ctx())
    assert r.runner.sub_state == "LISTEN" and r.runner.current_task == "TASK_DYNAMIC_INCIDENT"
    lat, lon = r.origin.from_local(20.0, 0.0)
    msg = r.runner.on_incident({"kind": "assistance_request", "command_seq": 7, "position": {"lat": lat, "lon": lon}}, r.ctx())
    assert msg == "accepted"
    st = r.steps(1)
    assert r.kinds() == [("incident", "ack")] and st.sub_state == "TRANSIT"        # ack within one tick
    assert r.events[0].body["command_seq"] == 7
    r.steps(5)
    assert st.out is None or r.runner.sub_state == "TRANSIT"
    r.move_to(19.5, 0.0)                                                            # arrived (arrive_r 1.0 m)
    st = r.steps(35)   # v0.1.67 (B4): 3 s on station before the readiness
    assert r.runner.sub_state == "WAIT_CONFIRM"
    assert r.kinds()[-1] == ("incident", "readiness") and r.events[-1].body["command_seq"] == 7
    assert r.runner.on_incident({"kind": "readiness_confirm", "command_seq": 7}, r.ctx()) == "confirmed"
    st = r.steps(1)
    assert st.done and not r.runner.active


def test_t4_breakpoint_detour_resumes_goto():
    r = Rig()
    r.runner.start("GOTO", {"x_n": 50.0, "y_e": 0.0}, r.ctx())
    r.steps(3)
    assert r.runner.task_name == "GOTO" and r.runner.sub_state == "GO"
    lat, lon = r.origin.from_local(5.0, 5.0)
    assert r.runner.on_incident({"kind": "assistance_request", "command_seq": 3, "position": {"lat": lat, "lon": lon}}, r.ctx()) == "detour started at breakpoint"
    assert r.runner.task_name == "T4" and r.runner.saved is not None and r.runner.saved.sub_state == "GO"
    r.steps(2)
    assert r.runner.sub_state == "TRANSIT"
    r.move_to(5.0, 5.0)
    r.steps(35)   # v0.1.67 (B4): 3 s on station before the readiness
    assert r.runner.sub_state == "WAIT_CONFIRM"
    r.runner.on_incident({"kind": "readiness_confirm", "command_seq": 3}, r.ctx())
    st = r.steps(1)
    assert st.running and r.runner.task_name == "GOTO" and "resumed after detour" in st.reason
    r.move_to(50.0, 0.0)
    st = r.steps(2)
    assert st.done and not r.runner.active


def test_t4_keep_out_and_all_clear_are_acked_without_motion():
    r = Rig()
    assert r.runner.on_incident({"kind": "keep_out_zone", "command_seq": 11, "polygon": [[1, 2], [3, 4], [5, 6]]}, r.ctx()).startswith("keep-out")
    assert r.runner.task_name == "T4" and r.runner.sub_state == "LISTEN"
    assert r.events[-1].body == {**r.events[-1].body, "kind": "ack", "what": "keep_out_zone", "command_seq": 11}
    r.runner.on_incident({"kind": "all_clear", "command_seq": 12}, r.ctx())
    assert r.events[-1].body["what"] == "all_clear"
    r.runner.on_incident({"kind": "moving_object", "command_seq": 13, "keep_out_radius_m": 5}, r.ctx())
    assert r.events[-1].body["what"] == "all_clear"                     # no ack exists for a moving object


def test_goto_timeout_aborts_task():
    r = Rig()
    r.cfg.limits.goto_timeout_s = 2.0
    r.runner.start("GOTO", {"x_n": 500.0, "y_e": 0.0}, r.ctx())
    st = None
    for _ in range(40):
        st = r.steps(1, dt=0.1)
        if st is None or st.aborted:
            break
    assert st is not None and st.aborted and "timeout" in st.reason and not r.runner.active


def test_test_missions_manual_and_auto():
    """TEST_MANUAL advances on step(), TEST_AUTO on its timers; neither drives (no behaviour); both end in DONE."""
    r = Rig()
    r.runner.start("TEST_MANUAL", {}, r.ctx())
    assert r.runner.sub_state == "S1" and r.runner.current_task == "TASK_NONE"
    st = r.steps(20)
    assert r.runner.sub_state == "S1" and st.out is None                            # nothing happens without a step
    assert r.runner.task.on_step(r.ctx()) == "S1 -> S2"
    assert r.runner.task.on_step(r.ctx()) == "S2 -> S3"
    assert r.runner.task.on_step(r.ctx()) == "S3 -> DONE"
    st = r.steps(1)
    assert st.done and not r.runner.active
    r.cfg.extra["test_auto"] = {"step_s": 0.5}
    r.runner.start("TEST_AUTO", {}, r.ctx())
    assert r.runner.sub_state == "S1"
    r.steps(6)                                                                      # 0.6 s
    assert r.runner.sub_state == "S2"
    r.steps(6)
    assert r.runner.sub_state == "S3"
    assert r.runner.task.on_step(r.ctx()).startswith("S3 -> DONE")                  # step skips the last timer
    st = r.steps(1)
    assert st.done and not r.runner.active
