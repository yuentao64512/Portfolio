"""v0.1.89 (Dmitrii 2026-11-12): the drone's touchdown as a micro-state sequence (flying -> map quiet -> RTL/LAND -> disarmed) releases
the boat from WAIT_UAV_MAP by itself; a repeated identical UAV map is no change (their publisher resends the whole map ~1 Hz)."""
from __future__ import annotations

from optiprimex.usv.tasks import uav_gate as UG

from .test_v0166 import O, _t1, item, msg, pmap


def test_the_sequence_step_by_step():
    latch = {}
    s = UG.touchdown({"flight": {"armed": False}}, -1e9, 100.0, 5.0, latch)
    assert s["step"] == "wait_flight" and not s["go"]                         # never flown: disarmed on the pad is NOT a touchdown
    s = UG.touchdown({"flight": {"armed": True, "mode": "AUTO"}}, 90.0, 100.0, 5.0, latch)
    assert s["step"] == "flying" and s["flying"] and not s["go"]
    s = UG.touchdown({"flight": {"armed": True, "mode": "RTL"}}, 99.0, 100.0, 5.0, latch)
    assert s["step"] == "rtl" and s["rtl"]
    s = UG.touchdown({"flight": {"armed": False, "mode": "LAND"}}, 99.0, 101.0, 5.0, latch)
    assert s["disarmed"] and not s["map_quiet"] and s["step"] == "map_quiet" and not s["go"]   # down, the map changed 2 s ago
    s = UG.touchdown({"flight": {"armed": False, "mode": "LAND"}}, 99.0, 105.0, 5.0, latch)
    assert s["go"] and s["step"] == "touchdown" and "TOUCHDOWN" in s["text"] and s["rtl"]
    s2 = UG.touchdown({"flight": {"armed": False, "armed_seen": True}}, -1e9, 10.0, 5.0, {})
    assert s2["go"]                                                           # landed by hand in LOITER (no RTL): still a touchdown


def test_a_repeated_map_is_quiet_a_moved_buoy_is_not():
    pm = pmap()
    pm.ingest_geo(msg([item(10.0, 0.0, "blue", "flash", "e")]), O, 100.0)
    assert pm.uav_change_t == 100.0
    for k, t in enumerate((101.0, 102.0, 103.0)):
        pm.ingest_geo(msg([item(10.0, 0.0, "blue", "flash", "e")], fid=f"r{k}"), O, t)
    assert pm.uav_change_t == 100.0 and pm.uav_last_t == 103.0               # the same map again: no change
    pm.ingest_geo(msg([item(13.0, 0.0, "blue", "flash", "e")], fid="moved"), O, 104.0)
    assert pm.uav_change_t == 104.0                                           # a buoy 3 m away: a change


def test_m1t2_leaves_wait_uav_map_on_the_touchdown_and_publishes_the_sequence():
    h = _t1({"active": True, "flight": {"armed": True, "mode": "AUTO"}})
    h.run(max_s=3.0)
    h.map.ingest_geo(msg([item(5.0, 0.0, "blue", "flash", "e")]), h.origin, h.t)
    assert not h.task.uav_map_ready(h.ctx())[0] and h.task.uav_gate["step"] == "flying"
    h.map.uav_ocs = {"active": True, "flight": {"armed": False, "mode": "LAND"}}
    for k in range(8):                                                        # their publisher repeats the map while their CV runs
        h.t += 1.0
        h.map.ingest_geo(msg([item(5.0, 0.0, "blue", "flash", "e")], fid=f"rep{k}"), h.origin, h.t)
    ok, why = h.task.uav_map_ready(h.ctx())
    assert ok and "TOUCHDOWN" in why                                          # the repeats did not keep the boat waiting
    out = h.task.step(h.ctx())
    assert out.out is None or "uav_gate" not in out.out.info or out.out.info["uav_gate"]["step"] in ("touchdown", "map_quiet", "flying")


# ---- the console: the mission-1 ladder and the run banner -----------------------------------------------------------------------------
from optiprimex.tools.tkconsole.model import ConsoleModel  # noqa: E402


def _state(r="R5", uav=None, mission=None, status=None, gate=None, uav_map=None):
    st = {"state": "AUTO_TASK", "armed": True, "mode": "GUIDED", "task_name": "T1", "task_state": "WAIT_UAV_MAP"}
    st.update(status or {})
    o = {"r": r, "run_id": 7, "declaration_seq": 3, "mission": mission or {}, "usv_flow": {"heartbeat": True, "ready": True},
         "uav": uav or {}, "declaration_checks": {"tiers": {"T1": "TIER_ADVANCED", "T2": "TIER_NONE", "T3": "TIER_NONE", "T4": "TIER_NONE"}}}
    return {"status": st, "ocs": o, "ocs_age": 0.1, "bar": {"word": "RUNNING"}, "clearance": {"clear": True},
            "uav_gate": gate, "uav_map": uav_map}


UAV = {"configured": True, "declare": True, "declared": True, "vehicle_id": "UAV1", "report_on": True, "pilot_ready": True}


def test_three_shapes():
    m = ConsoleModel()
    v = m.apply(_state(status={"task_name": "SCOUT", "task_state": "ADVANCE"}, mission={"code": "scout", "auto": True}))
    assert v.profile == "USV only" and [s.title.split(" —")[0] for s in v.steps] == [
        "handover", "crew", "ENABLE AUTO", "DECLARE", "RunStart", "the boat", "END RUN"]
    assert v.banner["title"] == "RUN STARTED · run 7" and v.banner["stage"] == "USV · SCOUT · ADVANCE" and v.banner["level"] == "ok"
    assert "T4 NONE" in v.steps[3].detail and "declares USV1 ·" in v.steps[3].detail          # the tiers on the ladder: no JUDGE tab
    assert [b.word for b in v.steps[5].buttons] == ["hold", "abort"] and v.steps[5].buttons[0].key == "lad6-0"
    v = ConsoleModel().apply(_state(uav=UAV, mission={"code": "m1t2", "auto": False}, status={"state": "AUTO_HOLD", "task_state": ""}))
    assert v.profile == "UAV only · boat holds" and [s.title.split(" —")[0] for s in v.steps][-2:] == ["the drone", "END RUN"]
    assert "declares USV1 + UAV1" in v.steps[3].detail and v.banner["stage"] == "UAV · TAKING OFF"
    v = ConsoleModel().apply(_state(uav=UAV, mission={"code": "m1t2", "auto": True},
                                    gate={"step": "flying", "flying": True, "text": "drone flying - mapping", "go": False}))
    assert v.profile == "UAV + USV" and [s.title.split(" —")[0] for s in v.steps][-3:] == ["the drone", "the boat", "END RUN"]
    assert v.banner["stage"] == "UAV · FLYING · MAPPING  ·  boat waits"
    assert any(b.word == "uav scan done" and "override" in b.label for b in v.steps[5].buttons)   # the boat-go override, asks first


def test_the_console_follows_the_touchdown_when_the_boat_runs_no_task():
    m = ConsoleModel()
    fl = lambda **k: dict(UAV, flight=k)
    v = m.apply(_state(uav=fl(armed=True, mode="AUTO"), mission={"code": "m1t2"}, status={"state": "AUTO_HOLD", "task_state": ""}))
    assert v.steps[5].chip == "FLYING" and "✓ FLYING" in v.steps[5].detail
    v = m.apply(_state(uav=fl(armed=True, mode="RTL"), mission={"code": "m1t2"}, status={"state": "AUTO_HOLD", "task_state": ""}))
    assert v.steps[5].chip == "RTL" and v.banner["stage"] == "UAV · RETURNING (RTL)"
    v = m.apply(_state(uav=fl(armed=False, mode="LAND"), mission={"code": "m1t2"}, status={"state": "AUTO_HOLD", "task_state": ""},
                       uav_map={"change_age_s": 2.0}))
    assert v.steps[5].chip == "settling"
    v = m.apply(_state(uav=fl(armed=False, mode="LAND"), mission={"code": "m1t2"}, status={"state": "AUTO_HOLD", "task_state": ""},
                       uav_map={"change_age_s": 9.0}))
    assert v.steps[5].chip == "TOUCHDOWN ✓" and v.steps[5].state == "done" and "✓ DISARMED" in v.steps[5].detail
    v = m.apply(_state(r="R6", uav=UAV, mission={"code": "m1t2"}))
    assert v.banner == {"title": "RUN ENDED", "stage": "run 7", "level": "idle"}
    assert m.apply(_state(r="R4")).banner is None                              # no banner before RunStart



def test_the_crew_step_keeps_a_heartbeat_toggle_per_vehicle():
    """v0.1.90.6 (Dmitrii: "I still need heartbeat button preserved per vehicle, bring it back"): always shown, on or off, no PILOT READY."""
    v = ConsoleModel().apply(_state(uav=UAV, mission={"code": "m1t2", "auto": True}))
    crew = v.steps[1]
    assert crew.sticky and [b.word for b in crew.buttons] == ["report off usv", "report off uav"]
    assert [b.label for b in crew.buttons] == ["✓ HEARTBEAT USV", "✓ HEARTBEAT UAV"] and all(b.confirm for b in crew.buttons)
    st = _state(mission={"code": "m1t1", "auto": True})
    st["ocs"]["usv_flow"] = {"heartbeat": False, "ready": True}
    v = ConsoleModel().apply(st)
    assert [b.word for b in v.steps[1].buttons] == ["report on usv"] and not any("ready" in b.word for b in v.steps[1].buttons)


def test_the_course_order_enable_auto_then_declare():
    """v0.1.90.7 (Dmitrii: "enable auto first before declaration ... Then, I declare"): ocs.auto_before_declare -> ENABLE AUTO (3) before
    DECLARE (4); DECLARE lights once the boat is in AUTO; the drone's mode is shown (its pilot sets GUIDED). The playbook stops
    predicting the "no PREPARE" refusal; the course profiles carry both halves."""
    import yaml
    from optiprimex.tools import playbook
    from .conftest import ROOT
    st = _state(r="R2", uav=dict(UAV, flight={"mode": "LOITER", "armed": False}), mission={"code": "m1t2", "auto": True},
                status={"state": "MANUAL", "task_name": None, "task_state": None})
    st["ocs"]["auto_before_declare"] = True
    v = ConsoleModel().apply(st)
    assert [x.title.split(" —")[0] for x in v.steps][:4] == ["handover", "crew", "ENABLE AUTO", "DECLARE"]
    assert v.steps[2].state == "now" and v.steps[3].state == "next" and "the drone: LOITER" in v.steps[2].detail
    st["status"].update(state="AUTO_HOLD")
    v = ConsoleModel().apply(st)
    assert v.steps[2].state == "done" and v.steps[3].state == "now"
    assert yaml.safe_load((ROOT / "profiles" / "course" / "usv_course.yaml").read_text())["guards"]["require_prepare_for_auto"] is False
    assert yaml.safe_load((ROOT / "profiles" / "course" / "ocs_course.yaml").read_text())["ocs"]["auto_before_declare"] is True
    for f in ("pool/usv_pool.yaml", "pool/usv_lab.yaml", "bench/usv_bench.yaml"):                   # v0.1.90.8: the rule everywhere
        assert yaml.safe_load((ROOT / "profiles" / f).read_text())["guards"]["require_prepare_for_auto"] is False, f
    assert any(b.label == "▶ ENABLE AUTO\nHOLD → LOITER" and b.kind == "big" for b in ConsoleModel().apply(
        dict(st, status=dict(st["status"], state="MANUAL"))).steps[2].buttons)                       # the big two-line button
    src = (ROOT / "optiprimex" / "tools" / "playbook.py").read_text()
    assert 'ocs_state.get("auto_before_declare") is False' in src
