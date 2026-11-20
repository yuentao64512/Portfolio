"""Stage stepping and entry stages (ROADMAP WP7, v0.1.26): START_TASK args.step parks the vehicle in AUTO_SAFE_HOLD at every
stage boundary (hold_reason "step: next <STAGE>", RESUME enters it); args.from starts a task at an entry stage whose precondition
holds, else REJECTED. Plus two RoboNation block-A points: a paused task keeps current_task (A5), the protobuf judge session is
persistent (A1). Fast SIL (optiprimex.sim.fast_sil): fast_sil resumes each step park at once, as the operator would."""
from __future__ import annotations

from optiprimex.companion.config import Config
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import ocs_cli, playbook

from .conftest import ROOT

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
NEAR_DOCK = {"x_n": 32.0, "y_e": 3.0, "hdg_deg": 0.0}
ABEAM_BAY2 = {"x_n": 36.0, "y_e": 5.5, "hdg_deg": 0.0}          # v0.1.34: 9 m off - the real 51 deg camera sees two bays (from 5 m only one)
T3_BOUNDARIES = ["FIND_DOCK", "TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "DOCKED", "REPORT_DOCK", "AIM_FIRE", "REPORT_FIRE", "UNDOCK"]


def test_m3t1_step_parks_at_every_boundary_and_reaches_done():
    r = FastSil(POOL, "T3", {"step": True}, start=NEAR_DOCK).run(400)
    assert r["state"] == "DONE", (r["state"], r["logs"][-4:])
    assert r["steps"] == [f"step: next {s}" for s in T3_BOUNDARIES]          # one park per boundary, in order
    ev = {e["event"]: e for e in r["events"]}
    assert ev["docking"]["bay"] == 2 and ev["firefighting"]["result"] == "green"


def test_step_without_resume_holds_the_vehicle():
    s = FastSil(POOL, "T3", {"step": True}, start=NEAR_DOCK)
    r = s.run(60, resume_steps=False)
    assert r["state"] == "SAFE_HOLD" and r["safe_holds"] == ["step: next FIND_DOCK"] and r["sub_state"] == "FIND_DOCK"
    assert abs(s.pose().x_n - 32.0) < 0.2 and abs(s.pose().y_e - 3.0) < 0.2     # nothing moved: parked before FIND_DOCK's scan


def test_m3t1_from_turn_in_reaches_done_and_refusals():
    s = FastSil(POOL, "T3", {"from": "TURN_IN"}, start=ABEAM_BAY2, warmup_s=3.0)
    assert s.from_refused == ""
    r = s.run(200)
    assert r["state"] == "DONE" and r["states"][0] == "TURN_IN" and "FIND_DOCK" not in r["states"], r["states"]
    assert any(e["event"] == "docking" and e["bay"] == 2 for e in r["events"])
    s = FastSil(POOL, "T3", {"from": "ENTER"}, start={"x_n": 2.0, "y_e": -8.0, "hdg_deg": 180.0}, warmup_s=3.0)
    assert "ENTER needs the dock line" in s.from_refused
    s = FastSil(POOL, "T3", {"from": "DOCKED"}, start=ABEAM_BAY2, warmup_s=3.0)
    assert "not an entry point of T3" in s.from_refused and "TO_LINE, SWAY_SEARCH, SWAY_TO_ABEAM, TURN_IN, ENTER, AIM_FIRE, UNDOCK" in s.from_refused


def test_m1t1_from_transit_and_exit_preconditions():
    s = FastSil(POOL, "T1", {"from": "TRANSIT", "circle_v": 1.0, "transit_v": 1.2}, start={"x_n": 5.0, "y_e": -6.0, "hdg_deg": 0.0}, warmup_s=3.0)
    assert s.from_refused == ""
    r = s.run(300)
    assert r["state"] == "DONE" and r["states"][:2] == ["TRANSIT", "EXIT"] and "CIRCLE_ENTRY_CW" not in r["states"]
    s = FastSil(POOL, "T1", {"from": "EXIT"}, start={"x_n": 0.0, "y_e": 0.0, "hdg_deg": 180.0}, warmup_s=2.0)   # facing away: no exit buoy
    assert "EXIT needs the exit buoy" in s.from_refused


def test_paused_task_keeps_current_task():
    """RoboNation A5: the lights stay on while a task is paused - the heartbeat keeps current_task (the runner keeps the task)."""
    from optiprimex.usv.task_runner import TaskRunner
    s = FastSil(POOL, "T3", {}, start=NEAR_DOCK)
    runner = TaskRunner(Config())
    runner.start("T3", {}, s.ctx())
    assert runner.current_task == "TASK_COORDINATED_LOGISTICS"
    runner.pause()
    assert runner.active and runner.current_task == "TASK_COORDINATED_LOGISTICS"
    runner.resume()
    assert runner.current_task == "TASK_COORDINATED_LOGISTICS"


def test_operator_side_cli_playbook_and_persistent_judge_session():
    spec = ocs_cli.intent_spec(ocs_cli.parse(["start_task", "m3t1sim", "--step", "--from", "turn_in"]))
    assert spec[:3] == ("START_TASK", "T3", {"tier": "core", "sim": True, "step": True, "from": "TURN_IN"})
    assert ocs_cli.intent_spec(ocs_cli.parse(["mission", "m3t1", "--step"]))[2]["step"] is True
    ocs = {"r": "R5", "vehicles": {"USV1": {"state": "AUTO_SAFE_HOLD", "task_name": "T3", "task_state": "TO_LINE", "hold_reason": "step: next TO_LINE"}}}
    assert playbook.suggest(ocs, "USV1").startswith("resume -> TO_LINE")
    import pytest
    from optiprimex import schemas
    schemas.validate_body("cmd", {"action": "START_TASK", "task": "T3", "args": {"step": True, "from": "TURN_IN"}})
    with pytest.raises(Exception):
        schemas.validate_body("cmd", {"action": "START_TASK", "task": "T3", "args": {"step": "yes"}})     # the schema types step
    from optiprimex.ocs.judge_client import ProtobufJudgeClient
    j = ProtobufJudgeClient("127.0.0.1", 1, "SGIT")
    assert j.c._client_id == b"ocs-SGIT-judge" and j.c._clean_session is False      # RoboNation A1: a persistent session
