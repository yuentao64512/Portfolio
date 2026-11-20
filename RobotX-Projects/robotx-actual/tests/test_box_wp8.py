"""m0t4 BOX_INSPECT after the merge (ROADMAP WP8, v0.1.27): the review's blocking fixes (docs/merge/BOX_MISSION_REVIEW.md §4)
and Dmitrii's answers of 2026-11-04 (Task 3 bays = steady indicators, the vehicle picks GREEN, stop 1 m, events OCS-internal).
Fast SIL on world_boxes.yaml (three 2 m faces 50 m north, steady lights, a 25 m camera)."""
from __future__ import annotations

import math

from optiprimex.companion.config import Config
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import ocs_cli, playbook
from optiprimex.usv.box_inspection import BoxInspectionMission, BoxSample, BoxState, BoxTrack, FaceGeometry, MissionSnapshot, PointNE

from .conftest import ROOT

BOXES = str(ROOT / "profiles" / "bench" / "world_boxes.yaml")
FACE = {"red": (50.0, -6.0), "green": (50.0, 0.0), "blue": (50.0, 6.0)}
SEARCH = ("OBSERVE", "STRAFE")


def sil(args, **kw):
    a = {"sim": True, "expected_boxes": 3, "scan_hdg_deg": 0.0}
    a.update(args)
    s = FastSil(BOXES, "BOX_INSPECT", a, **kw)
    s.cmds = []                                           # (sub-state, command type) per tick
    inner = s.task.step

    def step(ctx):
        st = inner(ctx)
        if st.out is not None and st.out.cmd is not None:
            s.cmds.append((s.task.m.state, type(st.out.cmd).__name__))
        return st
    s.task.step = step
    return s


def mode_changes(s):
    kinds = [k for _, k in s.cmds]
    return sum(a != b for a, b in zip(kinds, kinds[1:]))


def done_body(r):
    return next(e for e in r["events"] if e["event"] == "box_mission_complete")


def check_stop(r, colour):
    b = done_body(r)
    assert b["target"]["colour"] == colour
    n, e = FACE[colour]
    x, y = r["track"][-1][1], r["track"][-1][2]
    assert 0.9 <= n - x <= 1.25, (n - x, b)             # stop_range_m 1 (decided 2026-11-04)
    assert abs(y - e) < 0.3, (y, e)                      # on the face's centre line (line of sight on the approach)


def test_off_axis_scan_points_reach_done_without_mode_thrash():
    """Review §4.1 (ALIGN deadlock) + §4.3 (HOLD / frame alternation): from off-axis scan points the far face is chosen,
    ALIGN yaws in place on arrival and the run ends 1 m off the face; the search is mode 20 only (no Hold in it)."""
    for colour, (x, y), side in (("red", (35.0, 8.0), "port"), ("blue", (35.0, -10.0), "starboard")):
        s = sil({"target_colour": colour, "scan_x_n": x, "scan_y_e": y, "strafe_side": side})
        r = s.run(400)
        assert r["state"] == "DONE", (colour, r["state"], s.task.core.reason, r["states"])
        assert r["states"][:3] == ["GO_SCAN", "OBSERVE", "STRAFE"] and "ALIGN" in r["states"]
        check_stop(r, colour)
        assert all(k == "ThrusterFrame" for st, k in s.cmds if st in SEARCH), "a Hold inside the search = a mode change"
        assert mode_changes(s) <= 5, mode_changes(s)     # GUIDED -> 20 (search) -> GUIDED (align) -> 20 (yaw, approach) -> HOLD


def test_the_vehicle_picks_green_and_reports_the_survey():
    s = sil({"scan_x_n": 30.0, "scan_y_e": 0.0})
    r = s.run(400)
    assert r["state"] == "DONE"
    check_stop(r, "green")
    survey = next(e for e in r["events"] if e["event"] == "box_survey")
    assert sorted(b["colour"] for b in survey["boxes"]) == ["blue", "green", "red"]


def test_preflight_reads_the_health_beat_not_detections():
    """Review §4.2: nothing is in camera range at the start (the faces are 50 m off, the camera sees 25 m), yet the task
    drives to its scan point; with no perception beat at all it waits in PREFLIGHT."""
    s = sil({"scan_x_n": 30.0, "scan_y_e": 0.0})
    r = s.run(20)
    assert "GO_SCAN" in r["states"] and s.map.box_tracks == {}
    s = sil({"scan_x_n": 30.0, "scan_y_e": 0.0})
    s.sense = lambda: None                                # the camera is dead: no detections, no beat
    r = s.run(10)
    assert r["state"] == "TIMEOUT" and r["sub_state"] == "PREFLIGHT" and "camera" in s.task.core.reason


def test_strafe_has_a_length_and_a_side_and_resume_sweeps_back():
    """Review §4.4: from (30, 8) looking north the red face is out of view; a starboard search ends after 6 m east in a
    safe hold that says why; RESUME sweeps back to port (twice the length) and the run reaches DONE."""
    s = sil({"target_colour": "red", "scan_x_n": 30.0, "scan_y_e": 8.0, "strafe_side": "starboard"})
    r = s.run(300)
    assert r["state"] == "SAFE_HOLD" and r["sub_state"] == "HOLD", (r["state"], r["states"])
    assert r["safe_holds"][0].startswith("BOX_INSPECT OBSERVE: strafe limit 6 m") and "no red face" in r["safe_holds"][0]
    p = s.pose()
    assert 5.5 <= p.y_e - 8.0 <= 6.8 and abs(p.x_n - 30.0) < 1.0, (p.x_n, p.y_e)     # 6 m to starboard of north
    s.task.resume()
    r = s.run(400)
    assert r["state"] == "DONE", (r["state"], s.task.core.reason)
    check_stop(r, "red")


def test_align_timeout_holds_and_blink_is_a_parameter():
    face = FaceGeometry(PointNE(30, -1), PointNE(30, 0), PointNE(30, 1))
    steady = BoxTrack(1)
    for i in range(22):
        steady.add(BoxSample(i * 0.2, 1, face, "green", True, 0.9))
    assert steady.stable(require_blink=True) is None and steady.stable(require_blink=False).colour == "green"   # review §4.5
    m = BoxInspectionMission(PointNE(0, 0), expected_boxes=1, align_timeout_s=10.0)
    pre = {"rtk": True, "heading": True, "camera": True, "thrusters": True, "ocs": True}
    snap = lambda t, hdg=90.0: MissionSnapshot(now=t, position=PointNE(0, 0), heading_deg=hdg, preflight=pre, tracks=[steady])
    m.step(snap(0.0))                                     # report
    m.step(snap(0.1))                                     # GREEN chosen -> ALIGN
    assert m.state == BoxState.ALIGN
    a = m.step(snap(11.0))
    assert m.state == BoxState.HOLD and m.hold_new and a.body["reason"] == "ALIGN timeout 10 s"
    m.resume(12.0)
    assert m.state == BoxState.ALIGN and not m.hold_new


def test_step_parks_at_stage_boundaries_not_inside_the_search():
    s = sil({"target_colour": "red", "scan_x_n": 35.0, "scan_y_e": 8.0, "strafe_side": "port", "step": True})
    r = s.run(500)
    assert r["state"] == "DONE", (r["state"], r["steps"])
    assert r["steps"] == [f"step: next {x}" for x in ("GO_SCAN", "OBSERVE", "ASK_OCS", "ALIGN", "APPROACH", "STOP")]
    # (VALIDATE passes in the tick that reaches the 15 m range: APPROACH -> STOP in one step, so no park of its own)
    check_stop(r, "red")


def test_from_entry_stages_and_refusals():
    s = sil({"from": "ALIGN", "scan_x_n": 30.0, "scan_y_e": 0.0}, start={"x_n": 30.0, "y_e": 3.0, "hdg_deg": 0.0}, warmup_s=6.0)
    assert s.from_refused == "" and s.task.m.state == "ALIGN"
    r = s.run(300)
    assert r["state"] == "DONE" and "GO_SCAN" not in r["states"] and "OBSERVE" not in r["states"]
    check_stop(r, "green")
    s = sil({"from": "ALIGN"}, start={"x_n": 0.0, "y_e": 0.0, "hdg_deg": 0.0}, warmup_s=6.0)        # nothing in view
    assert "ALIGN needs one stable face of the target colour" in s.from_refused
    s = sil({"from": "APPROACH"}, start={"x_n": 30.0, "y_e": 3.0, "hdg_deg": 0.0}, warmup_s=6.0)    # 15 m off the align point
    assert "APPROACH needs the boat on the approach line" in s.from_refused
    s = sil({"from": "STOP"}, start={"x_n": 30.0, "y_e": 3.0, "hdg_deg": 0.0}, warmup_s=6.0)
    assert "not an entry point of BOX_INSPECT" in s.from_refused and "OBSERVE, ALIGN, APPROACH" in s.from_refused


def test_operator_side_m0t4():
    spec = ocs_cli.intent_spec(ocs_cli.parse(["start_task", "m0t4sim", "--scan-x", "35", "--scan-y", "8", "--scan-hdg", "0",
                                              "--strafe-side", "port", "--target-colour", "red", "--from", "align"]))
    assert spec[:3] == ("START_TASK", "BOX_INSPECT", {"sim": True, "scan_x_n": 35.0, "scan_y_e": 8.0, "scan_hdg_deg": 0.0,
                                                       "strafe_side": "port", "target_colour": "red", "from": "ALIGN"})
    ocs = {"r": "R5", "vehicles": {"USV1": {"state": "AUTO_SAFE_HOLD", "task_name": "BOX_INSPECT", "task_state": "HOLD"}}}
    assert playbook.check("SELECT_TARGET", ocs, "USV1").ok
    ocs["vehicles"]["USV1"].update(state="AUTO_TASK", task_state="APPROACH")
    assert not playbook.check("SELECT_TARGET", ocs, "USV1").ok
    assert Config().extra.get("box_inspect") is None      # defaults live in the task; the profiles carry box_inspect:
    from optiprimex.companion.config import load
    assert load(ROOT / "profiles" / "pool" / "usv_pool.yaml").extra["box_inspect"]["stop_range_m"] == 1.0
    assert math.isclose(load(ROOT / "profiles" / "bench" / "usv_bench.yaml").extra["box_inspect"]["strafe_len_m"], 6.0)
