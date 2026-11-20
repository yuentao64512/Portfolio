"""ROADMAP WP10d (v0.1.37), the operator layer: TEST MODE (run-free tasks without a run, never on the course), MISSION CONTROL's state
word, the judges' view (the run timeline: Ended when the last vehicle leaves AUTO), STOP / RESET EVERYTHING (order, guards, words),
the judge page as tables, the theme switch."""
from __future__ import annotations

import os
import subprocess

import pytest

from optiprimex.companion.config import Config, load
from optiprimex.companion.state import Command
from optiprimex.usv.vehicle_fsm import RUN_FREE_TASKS, VehicleFsm

from .conftest import ROOT
from .test_ocs_units import intent, make_authority
from .test_vehicle_fsm_table import cmd, drive, fcu
from optiprimex.companion.state import Health, VState

PROF = ROOT / "profiles"


def _fsm(test_mode: bool) -> VehicleFsm:
    cfg = Config()
    cfg.guards.test_mode = test_mode
    cfg.guards.allow_goto_task = True
    return VehicleFsm(cfg)


def test_test_mode_on_the_vehicle():
    f = _fsm(True)
    assert f.run_id is None and f.declaration_seq is None
    for t in ("TEST_PATH", "TEST_SWAY", "TEST_ORBIT", "TEST_MANUAL", "BOX_INSPECT", "GOTO"):
        assert f.can_start_task(cmd("START_TASK", t)) == (True, ""), t
    for t in ("T1", "T3", "T4"):                                                    # the judged missions keep the run
        ok, why = f.can_start_task(cmd("START_TASK", t))
        assert not ok and "START_RUN" in why
    f.inputs.armed, f.inputs.ekf_pos_ok = True, True
    assert "PREPARE" not in f.can_enable_auto()[1]                                    # no declaration needed in test mode
    g = _fsm(False)
    g.cfg.guards.require_prepare_for_auto = True                                       # v0.1.90.8: opt-in now (the default is off)
    g.inputs.armed, g.inputs.ekf_pos_ok = True, True
    assert not g.can_start_task(cmd("START_TASK", "TEST_PATH"))[0]
    assert "PREPARE" in g.can_enable_auto()[1]
    assert set(RUN_FREE_TASKS) == {"TEST_MANUAL", "TEST_AUTO", "TEST_PATH", "TEST_SWAY", "TEST_ORBIT", "GOTO", "RETURN_HOME", "BOX_INSPECT", "TEST_DETECT", "TEST_GUARD", "TEST_YAW"}   # v0.1.79: + m0t8; v0.1.80: + m0t9; v0.1.85: + m0t10


def test_profiles_set_test_mode_and_never_on_the_course():
    for name, want in (("bench/usv_bench.yaml", True), ("pool/usv_pool.yaml", True), ("pool/usv_lab.yaml", True), ("course/usv_course.yaml", False)):
        assert load(str(PROF / name)).guards.test_mode is want, name
    import yaml
    for name, want in (("bench/ocs_bench.yaml", True), ("pool/ocs_pool.yaml", True), ("course/ocs_course.yaml", False)):
        assert yaml.safe_load((PROF / name).read_text())["ocs"]["test_mode"] is want, name
    assert load(str(ROOT / "usv_ui" / "config" / "usv_bench_share.yaml")).guards.test_mode is True


def test_test_mode_at_the_ocs_and_in_the_playbook():
    bus, lc, auth, results = make_authority(bench=False, lc_state="COURSE_VALID")       # R2, the course's rules
    auth.test_mode = True
    assert auth.on_intent("t", intent("START_TASK", "TEST_PATH", {"names": ["A", "B"]})).result == "accepted"
    assert auth.on_intent("t", intent("ENABLE_AUTO")).result == "accepted"
    r = auth.on_intent("t", intent("START_TASK", "T1"))
    assert r.result == "refused" and "R5" in r.reason                                  # m1t1 still needs the run
    auth.test_mode = False                                                             # the course profile
    r = auth.on_intent("t", intent("START_TASK", "TEST_PATH", {"names": ["A", "B"]}))
    assert r.result == "refused" and "declare first" in r.reason
    from optiprimex.tools import playbook as P
    ocs = {"r": "R2", "bench_mode": False, "test_mode": True, "run_id": None, "declaration_seq": 0,
           "vehicles": {"USV1": {"state": "AUTO_HOLD"}}}
    assert P.check("START_TASK", ocs, task="m0t2").ok
    v = P.check("START_TASK", ocs, task="m1t1")
    assert not v.ok and "R5" in v.why and "RunStart" in v.hint                         # (the course's R5 rule fires first here)
    v = P.check("START_TASK", dict(ocs, bench_mode=True), task="m1t1")                 # the pool: the vehicle's run rule
    assert not v.ok and "RunStart" in v.why and "test mode" in v.hint
    st = {"state": "MANUAL", "armed": True, "mode": "HOLD", "kill": False}
    assert P.check("ENABLE_AUTO", dict(ocs, vehicles={"USV1": {"state": "MANUAL"}}), status=st).ok
    assert not P.check("START_TASK", dict(ocs, test_mode=False), task="m0t2").ok


def test_rearm_after_a_kill_keeps_the_run():
    """RE-ARM = enable_auto after a kill: KILLED -> confirm_safe -> MANUAL -> enable_auto -> AUTO_HOLD; run 1001 kept."""
    f = _fsm(True)
    f.declaration_seq, f.run_id = 1, 1001
    drive(f, VState.V3)                                                                # AUTO_HOLD in the run
    f.feed(fcu(kill=True), Health(), False, [], [])
    assert f.state == VState.V1
    f.feed(fcu(), Health(), False, [], [])
    f.handle_command(cmd("CONFIRM_SAFE"))
    f.feed(fcu(), Health(), False, [], [])
    assert f.state == VState.V2 and f.run_id == 1001
    res = f.handle_command(cmd("ENABLE_AUTO", run_id=1001, seq=1))
    assert f.state == VState.V3 and f.run_id == 1001, res


def test_control_word_follows_the_ladder():
    from optiprimex.tools.ocs_web import control_word
    ocs = {"r": "R2", "test_mode": True}
    base = {"armed": True, "mode": "HOLD", "health": {"faults": []}, "kill": False, "estop": False}
    w = lambda st, ui=False, **kw: control_word(st, kw.get("ocs", ocs), ui, kw.get("t0"), kw.get("now", 100.0))
    assert w(None)["word"] == "NOT READY"
    assert "handover" in w(dict(base, state="MANUAL"), ui=True)["hint"]
    assert "transmitter" in w(dict(base, state="MANUAL", armed=False))["hint"]
    assert "HOLD" in w(dict(base, state="MANUAL", mode="MANUAL"))["hint"]
    m = w(dict(base, state="MANUAL"))
    assert m["word"] == "NOT READY" and m["rearm"] and "RE-ARM" in m["hint"]
    assert w(dict(base, state="KILLED"))["hint"].startswith("confirm_safe")
    assert "release" in w(dict(base, state="KILLED", kill=True))["hint"]
    assert w(dict(base, state="AUTO_HOLD"))["word"] == "ARMED" and not w(dict(base, state="AUTO_HOLD"))["judged_ready"]
    assert w(dict(base, state="AUTO_HOLD"), ocs={"r": "R5"})["judged_ready"]
    r = w(dict(base, state="AUTO_TASK", task_name="TEST_PATH", task_state="FOLLOW"), t0=90.0)
    assert r["word"] == "RUNNING" and r["elapsed_s"] == 10 and "FOLLOW" in r["why"]
    assert w(dict(base, state="AUTO_SAFE_HOLD", hold_reason="no gate ahead"))["why"] == "no gate ahead"
    assert w(dict(base, state="FAULT", fault="EKF_BAD"))["word"] == "NOT READY"


def test_run_timeline_as_the_judges_see_it():
    from optiprimex.tools.ocs_web import RunTimeline
    t = RunTimeline()
    auto, killed = {"USV1": {"robot_state": "STATE_AUTO", "state": "AUTO_HOLD"}}, {"USV1": {"robot_state": "STATE_MANUAL", "state": "KILLED"}}
    t.update({"r": "R3", "declaration_seq": 1}, auto)
    t.update({"r": "R4", "declaration_seq": 1}, auto)
    t.update({"r": "R5", "declaration_seq": 1, "run_id": 1001}, auto)
    t.update({"r": "R5", "declaration_seq": 1, "run_id": 1001}, killed)
    kinds = [e["kind"] for e in t.events]
    assert kinds == ["declared", "held", "started", "ended"] and t.ended
    assert "the last vehicle" in t.events[-1]["text"] and t.events[-1]["style"] == "bad"
    t.update({"r": "R5", "declaration_seq": 1, "run_id": 1001}, killed)               # once
    assert [e["kind"] for e in t.events].count("ended") == 1
    t.update({"r": "R3", "declaration_seq": 2}, auto)                                  # a new declaration starts over
    t.update({"r": "R5", "declaration_seq": 2, "run_id": 1002}, auto)
    t.update({"r": "R6", "declaration_seq": 2, "run_id": 1002}, auto)
    assert t.events[-1]["text"].startswith("Run Ended · end_run") and t.body()["run_id"] == 1002


def test_stop_and_reset_everything():
    out = subprocess.run(["bash", str(ROOT / "scripts" / "stop_all.sh"), "--dry-run"], capture_output=True, text=True, timeout=20).stdout
    steps = [ln for ln in out.splitlines() if ln.startswith("[stop_all] ") and ":" in ln and "done" not in ln]
    assert [s.split(":")[0].split("] ")[1] for s in steps] == ["1 boat (NX)", "2 perception (AGX)", "3 judge console", "4 rc-test",
                                                                "5 bench / simulator", "6 OCS (last)"]
    assert "stop_usv.sh" in steps[0] and "stop_ocs.sh" in steps[-1] and "nothing was stopped" in out
    r = subprocess.run(["bash", str(ROOT / "scripts" / "reset_all.sh"), "--dry-run"], capture_output=True, text=True, timeout=20,
                       env=dict(os.environ, SIM_PORT="1"))
    assert "ocs_cli abort, reset" in r.stdout and "dry run" in r.stdout
    from optiprimex.tools.system_actions import BY_ID
    sa, ra = BY_ID["stopall"], BY_ID["resetall"]
    assert sa.confirm == "stop" and ra.confirm == "reset" and sa.group == ra.group == "everything" and sa.detach
    assert "abort first" in sa.guard({"status": {"state": "AUTO_TASK", "task_name": "TEST_PATH"}}) and sa.guard({"status": {"state": "AUTO_HOLD"}}) == ""
    assert "stop_all.sh" in " ".join(sa.build({}, {})) and "reset_all.sh" in " ".join(ra.build({}, {}))


def test_the_pages_carry_the_operator_layer():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('id="t-MISSION"', 'id="cw"', 'id="c-start"', 'id="c-hold"', 'id="c-resume"', 'id="c-abort"', 'id="c-restart"', 'id="c-home"',
                   'id="c-rearm"', 'id="stopallbtn"', 'id="resetallbtn"', 'id="rc-tl"', 'id="rc-veh"', 'id="rc-tasks"', 'id="themesel"',
                   '<link rel="stylesheet" href="/themes.css">', '"MISSION"'):         # v0.1.44: CONTROL lives on MISSION         # v0.1.43: the themes moved to themes.css
        assert needle in page, needle
    from optiprimex.tools import judge_web
    for needle in ("function parseMsg", 'id="fk-rx"', 'id="fk-tx"', 'id="theme"', "the last vehicle out", 'href="/themes.css"'):
        assert needle in judge_web.PAGE, needle
    mp = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert "localStorage.getItem('ocs.theme')" in mp and 'html[data-theme="contrast"]' in mp
