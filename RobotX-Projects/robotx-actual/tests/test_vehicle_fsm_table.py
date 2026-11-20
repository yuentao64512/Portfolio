"""Table-driven test of the vehicle state model (ARCHITECTURE 3.2, ADR-9 precedence).

Each row: start state, physical inputs, optional command, expected state (and command result).
Changing the state model changes this table; that is the point.
"""
from __future__ import annotations

import pytest

from optiprimex.companion.config import Config
from optiprimex.companion.state import Command, FcuState, Health, VState
from optiprimex.usv.vehicle_fsm import VehicleFsm

V = VState


def fcu(**kw) -> FcuState:
    f = FcuState(seen_heartbeat=True, mode_name="HOLD", armed=True, ekf_pos_ok=True, rc_chancount=16, rc_ok=True)
    f.rc = {5: 1000, 6: 1000, 8: 1500}
    for k, v in kw.items():
        if k == "rc6":
            f.rc[6] = v
        elif k == "rc5":
            f.rc[5] = v
        else:
            setattr(f, k, v)
    return f


def cmd(action: str, task=None, run_id=None, seq=None, **args) -> Command:
    env = {"msg_id": "m-" + action, "run_id": run_id, "declaration_seq": seq, "vehicle_id": "USV1", "kind": "cmd", "body": {}}
    return Command(env=env, action=action, task=task, args=args)


def make(prepared=True, run=True) -> VehicleFsm:
    f = VehicleFsm(Config())
    if prepared:
        f.declaration_seq = 1
    if run:
        f.run_id = 1001
    return f


def drive(f: VehicleFsm, target: VState, inputs: dict = None) -> None:
    """Bring the machine to a start state through legal moves."""
    inputs = inputs or {}
    f.feed(fcu(**inputs), Health(), False, [], [])          # V0 -> V2 (or V1 if kill)
    if target in (V.V0, V.V2, V.V1):
        if target == V.V1:
            f.feed(fcu(kill=True), Health(), False, [], [])
        assert f.state == target, f.state
        return
    assert f.handle_command(cmd("ENABLE_AUTO")) == ("ACCEPTED", ""), f.state
    if target == V.V3:
        return
    if target == V.V6:
        f.feed(fcu(), Health(), False, ["FCU_LOST"], [])
        assert f.state == V.V6
        return
    assert f.handle_command(cmd("START_TASK", task="T4")) == ("ACCEPTED", "")
    if target == V.V4:
        return
    if target == V.V5:
        assert f.handle_command(cmd("HOLD")) == ("ACCEPTED", "")
        return
    raise AssertionError(target)


# start, inputs after start (physical), command, expected state, expected command result
ROWS = [
    # boot
    ("boot_to_manual", V.V0, {}, None, V.V2, None),
    ("boot_with_kill", V.V0, {"kill": True}, None, V.V1, None),
    # kill precedence from every auto state and manual
    ("kill_from_manual", V.V2, {"kill": True}, None, V.V1, None),
    ("kill_from_hold", V.V3, {"kill": True}, None, V.V1, None),
    ("kill_from_task", V.V4, {"kill": True}, None, V.V1, None),
    ("kill_from_safe_hold", V.V5, {"kill": True}, None, V.V1, None),
    ("estop_from_task", V.V4, {"estop": True}, None, V.V1, None),
    ("rc5_estop_from_task", V.V4, {"rc5": 1900}, None, V.V1, None),
    ("kill_beats_rc6", V.V4, {"kill": True, "rc6": 1900}, None, V.V1, None),
    ("kill_beats_fault", V.V4, {"kill": True, "mode_name": None}, None, V.V1, None),
    # RC override
    ("rc6_from_hold", V.V3, {"rc6": 1900}, None, V.V2, None),
    ("rc6_from_task", V.V4, {"rc6": 1900}, None, V.V2, None),
    ("fcu_manual_from_task", V.V4, {"mode_name": "MANUAL"}, None, V.V2, None),
    ("rc6_from_safe_hold", V.V5, {"rc6": 1900}, None, V.V2, None),
    # leaving KILLED / FAULT needs the input cleared AND confirm
    ("kill_cleared_no_confirm", V.V1, {}, None, V.V1, None),
    ("kill_cleared_with_confirm", V.V1, {}, "CONFIRM_SAFE", V.V2, "ACCEPTED"),
    ("confirm_while_kill_still_on", V.V1, {"kill": True}, "CONFIRM_SAFE", V.V1, "ACCEPTED"),
    ("confirm_safe_in_hold_rejected", V.V3, {}, "CONFIRM_SAFE", V.V3, "REJECTED"),
    # commands and guards
    ("enable_auto_from_hold_rejected", V.V3, {}, "ENABLE_AUTO", V.V3, "REJECTED"),
    ("enable_auto_fcu_manual", V.V2, {"mode_name": "MANUAL"}, "ENABLE_AUTO", V.V2, "REJECTED"),
    ("enable_auto_not_armed", V.V2, {"armed": False}, "ENABLE_AUTO", V.V2, "REJECTED"),
    ("enable_auto_ekf_bad", V.V2, {"ekf_pos_ok": False}, "ENABLE_AUTO", V.V2, "REJECTED"),
    ("enable_auto_ok", V.V2, {}, "ENABLE_AUTO", V.V3, "ACCEPTED"),
    ("start_task_ok", V.V3, {}, ("START_TASK", "T4"), V.V4, "ACCEPTED"),
    ("start_task_from_task_rejected", V.V4, {}, ("START_TASK", "T4"), V.V4, "REJECTED"),
    ("start_task_unknown", V.V3, {}, ("START_TASK", "T7"), V.V3, "REJECTED"),
    ("goto_task_not_allowed_by_default", V.V3, {}, ("START_TASK", "GOTO"), V.V3, "REJECTED"),
    ("hold_from_hold", V.V3, {}, "HOLD", V.V5, "ACCEPTED"),
    ("hold_from_task", V.V4, {}, "HOLD", V.V5, "ACCEPTED"),
    ("hold_from_manual_rejected", V.V2, {}, "HOLD", V.V2, "REJECTED"),
    ("abort_from_task", V.V4, {}, "ABORT_TASK", V.V3, "ACCEPTED"),
    ("abort_from_safe_hold", V.V5, {}, "ABORT_TASK", V.V3, "ACCEPTED"),
    ("abort_from_hold_rejected", V.V3, {}, "ABORT_TASK", V.V3, "REJECTED"),
    ("disable_auto_from_hold", V.V3, {}, "DISABLE_AUTO", V.V2, "ACCEPTED"),
    ("disable_auto_from_task_rejected", V.V4, {}, "DISABLE_AUTO", V.V4, "REJECTED"),
    ("resume_from_hold_rejected", V.V3, {}, "RESUME", V.V3, "REJECTED"),
]


@pytest.mark.parametrize("name,start,inputs,command,expected,result", ROWS, ids=[r[0] for r in ROWS])
def test_table(name, start, inputs, command, expected, result):
    f = make()
    if start != V.V0:
        drive(f, start)
    f.feed(fcu(**inputs), Health(), False, ["FCU_LOST"] if inputs.get("mode_name", "HOLD") is None else [], [])
    if command:
        action, task = (command, None) if isinstance(command, str) else command
        res, reason = f.handle_command(cmd(action, task=task))
        assert res == result, (res, reason)
        f.feed(fcu(**inputs), Health(), False, [], [])   # confirm-type commands land on the next tick
    assert f.state == expected, f"{name}: {f.state} != {expected}"


def test_fault_precedence_and_recovery():
    f = make()
    drive(f, V.V4)
    f.feed(fcu(), Health(), False, ["FCU_LOST"], [])
    assert f.state == V.V6 and f.fault_reason == "FCU_LOST"
    f.feed(fcu(), Health(), False, [], [])                 # fault cleared, no confirm yet
    assert f.state == V.V6
    assert f.handle_command(cmd("CONFIRM_SAFE"))[0] == "ACCEPTED"
    f.feed(fcu(), Health(), False, [], [])
    assert f.state == V.V2                                 # never straight back to AUTO
    assert f.handle_command(cmd("ENABLE_AUTO"))[0] == "ACCEPTED" and f.state == V.V3


def test_soft_fault_pauses_task_and_resume_needs_clear():
    f = make()
    drive(f, V.V4)
    f.feed(fcu(), Health(), False, [], ["PERCEPTION_LOST"])
    assert f.state == V.V5 and f.task_paused
    assert f.handle_command(cmd("RESUME"))[0] == "REJECTED"   # still lost (inputs say so)
    f.feed(fcu(), Health(), False, [], [])
    assert f.handle_command(cmd("RESUME")) == ("ACCEPTED", "") and f.state == V.V4


def test_resume_returns_to_task_or_hold():
    f = make()
    drive(f, V.V5)                       # hold from a task -> paused task
    assert f.task_paused
    assert f.handle_command(cmd("RESUME"))[0] == "ACCEPTED" and f.state == V.V4
    f2 = make()
    drive(f2, V.V3)
    f2.handle_command(cmd("HOLD"))
    assert f2.state == V.V5 and not f2.task_paused
    assert f2.handle_command(cmd("RESUME"))[0] == "ACCEPTED" and f2.state == V.V3


def test_guards_prepare_and_run():
    f = make(prepared=False, run=False)
    drive(f, V.V2)
    assert f.cfg.guards.require_prepare_for_auto is False and f.can_enable_auto()[0]   # v0.1.90.8: ENABLE AUTO before DECLARE, the default
    f.cfg.guards.require_prepare_for_auto = True                                     # the guard still works when a profile asks
    assert f.handle_command(cmd("ENABLE_AUTO")) == ("REJECTED", "no PREPARE (declaration_seq unknown)")
    f.declaration_seq = 1
    assert f.handle_command(cmd("ENABLE_AUTO"))[0] == "ACCEPTED"
    assert f.handle_command(cmd("START_TASK", task="T4")) == ("REJECTED", "no START_RUN (run_id unknown)")
    f.run_id = 1001
    assert f.handle_command(cmd("START_TASK", task="T4"))[0] == "ACCEPTED" and f.state == V.V4
    f.task_finished(aborted=False)
    assert f.state == V.V3


def test_tui_counts_as_override():
    f = make()
    drive(f, V.V3)
    f.feed(fcu(), Health(), True, [], [])
    assert f.state == V.V2


def test_transition_table_covers_every_state():
    f = make()
    table = f.m.table()
    dsts = {t["dst"] for t in table}
    srcs = {s for t in table for s in t["src"]}
    for s in V:
        assert s.value in dsts or s.value == V.V0.value, f"{s.value} is never entered"
        assert s.value in srcs or s.value == V.V0.value or "*" in srcs, f"{s.value} is never left"


def test_lab_profile_walks_the_state_machine_indoors():
    """profiles/pool/usv_lab.yaml: a real Pixhawk on the bench - disarmed, no GPS (EKF_BAD), transmitter off (RC_LOST) -
    must still accept ENABLE_AUTO and never park in FAULT; the pool profile refuses / faults in the same situation."""
    import os
    from optiprimex.companion.config import load
    from optiprimex.companion.health import HealthMonitor
    here = os.path.dirname(os.path.abspath(__file__))
    lab = load(os.path.join(here, "..", "profiles", "pool", "usv_lab.yaml"))
    pool = load(os.path.join(here, "..", "profiles", "pool", "usv_pool.yaml"))
    bench_fcu = fcu(armed=False, ekf_pos_ok=False, rc_ok=False, rc_chancount=0)
    bench_fcu.rc = {}
    bench_fcu.ages = {"HEARTBEAT": 0.1, "GLOBAL_POSITION_INT": 0.1, "ATTITUDE": 0.1, "EKF_STATUS_REPORT": 0.2}    # no RC_CHANNELS at all
    for cfg, expect_auto in ((pool, False), (lab, True)):
        hm = HealthMonitor(cfg)
        h = hm.update(bench_fcu, True, float("inf"), 0.0)
        assert "EKF_BAD" in h.faults and "RC_LOST" in h.faults                 # both profiles still SEE the faults
        f = VehicleFsm(cfg)
        f.declaration_seq = 1
        f.run_id = 1001
        f.feed(bench_fcu, h, False, hm.hard_faults(), hm.soft_faults())
        f.feed(bench_fcu, h, False, hm.hard_faults(), hm.soft_faults())
        ok, why = f.can_enable_auto()
        if expect_auto:
            assert f.state == V.V2 and ok, (f.state, why)
            assert f.handle_command(cmd("ENABLE_AUTO"))[0] == "ACCEPTED" and f.state == V.V3
            f.feed(bench_fcu, h, False, hm.hard_faults(), hm.soft_faults())
            assert f.state == V.V3                                              # stays AUTO_HOLD: EKF_BAD is ignored, not a fault
            assert f.handle_command(cmd("HOLD"))[0] == "ACCEPTED" and f.state == V.V5
            assert f.handle_command(cmd("RESUME"))[0] == "ACCEPTED" and f.state == V.V3
            assert f.handle_command(cmd("DISABLE_AUTO"))[0] == "ACCEPTED" and f.state == V.V2
        else:
            # v0.1.59 (A2): EKF_BAD is soft for guards.ekf_bad_hard_s (3 s) - MANUAL stays, enable_auto refused - then FAULT
            assert f.state == V.V2 and not ok, (f.state, why)                   # refused (disarmed here; EKF next)
            h = hm.update(bench_fcu, True, float("inf"), cfg.guards.ekf_bad_hard_s)
            f.feed(bench_fcu, h, False, hm.hard_faults(), hm.soft_faults())
            assert f.state == V.V6 and "EKF_BAD" in f.can_enable_auto()[1]        # the pool profile: FAULT, as it should


def test_link_loss_policy_and_profile_in_mm_state():
    """guards.link_loss_action: continue (default) leaves a running task alone when the hub bridge drops; safe_hold parks it in V5
    after link_loss_s with a sticky hold that clears when the link is back. mm/state carries profile / lab / link_loss_action."""
    import os
    from optiprimex.companion.config import load
    from optiprimex.companion.state import VehicleState
    from optiprimex.usv.mission_manager import MissionManager
    here = os.path.dirname(os.path.abspath(__file__))
    pool = load(os.path.join(here, "..", "profiles", "pool", "usv_pool.yaml"))
    assert pool.profile == "usv_pool.yaml" and pool.guards.link_loss_action == "continue"
    vs = VehicleState(profile="usv_lab.yaml", lab=True, link_loss_action="safe_hold")
    body = vs.local_body()
    assert body["profile"] == "usv_lab.yaml" and body["lab"] is True and body["link_loss_action"] == "safe_hold"

    class _Link:
        def start(self): pass
        def stop(self): pass
        def state(self): return fcu()

    class _Bus:
        host, port = "127.0.0.1", 0
        def subscribe(self, *a, **k): pass
        def watch_bridge(self): pass
        def start(self): pass
        def stop(self): pass
        def bridge_state(self, name=None): return True
        def publish(self, *a, **k): pass
        def publish_env(self, *a, **k): pass
    for action, expect_hold in (("continue", False), ("safe_hold", True)):
        cfg = load(os.path.join(here, "..", "profiles", "pool", "usv_pool.yaml"))
        cfg.guards.link_loss_action = action
        cfg.guards.link_loss_s = 2.0
        mm = MissionManager(cfg, link=_Link(), bus=_Bus())
        assert mm.vs.profile == "usv_pool.yaml" and mm.vs.lab is False and mm.vs.link_loss_action == action
        f = mm.fsm
        f.m.state = V.V4.value                       # a task is running
        f.task_running = True
        mm._link_loss_policy(False, 100.0)           # the bridge drops
        mm._link_loss_policy(False, 101.0)
        assert f.state == V.V4                        # not yet link_loss_s
        mm._link_loss_policy(False, 102.5)
        if expect_hold:
            assert f.state == V.V5 and "OCS_LINK" in f.sticky_holds and not f.hold_cleared()
            assert f.handle_command(cmd("RESUME"))[0] == "REJECTED"           # the link is still down
            mm._link_loss_policy(True, 103.0)                                  # back: the sticky hold clears, the operator resumes
            assert "OCS_LINK" not in f.sticky_holds and f.hold_cleared()
            assert f.handle_command(cmd("RESUME"))[0] == "ACCEPTED" and f.state == V.V4
        else:
            assert f.state == V.V4 and not f.sticky_holds
