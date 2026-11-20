"""K2: the tk console's view-model on three RECORDED triples (state.json / fast.json / alarms.json from a run_sim.sh pool session,
2026-11-11, the free-run sequence through POST /cmd): r2 = AUTO_HOLD, nothing declared, scoutsim armed · r5_task = R5, SCOUT running
(APPROACH/LOG) · r6_alarm = R6 after end_run, the plant's EKF off -> FAULT EKF_BAD (an ALARM). No Tk, no network."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from optiprimex.tools.tkconsole import model as M
from optiprimex.tools.tkconsole.theme import palette, parse_css

FIX = Path(__file__).parent / "fixtures"


def load(name):
    return json.loads((FIX / f"state_{name}.json").read_text())


@pytest.fixture()
def cm():
    m = M.ConsoleModel(clock=lambda: 1794372934.0)
    m.set_meta(load("meta"))
    m.set_course(load("course"))
    return m


def view(cm, name, **kw):
    t = load(name)
    return cm.apply(t["state"], t["fast"], t["alarms"], **kw)


def steps(v):
    return {s.n: s.state for s in v.steps}


def test_r2_ready_to_declare(cm):
    v = view(cm, "r2")
    assert v.title.startswith("OCS · ocs_bench · v")
    assert v.link == "UP" and not v.stale and not v.down
    assert v.profile == "USV only" and steps(v)[1] == "done" and steps(v)[3] == "done" and steps(v)[4] == "now" and steps(v)[5] == "next" and steps(v)[6] == "next"
    decl = v.steps[3].buttons[0]                                                         # v0.1.90.8: 4 = DECLARE (after ENABLE AUTO)
    assert decl.word == "declare" and decl.ok and decl.kind == "act"
    assert v.header["fcu"].text == "mm · HOLD · armed" and v.header["v"].text == "R2 · AUTO_HOLD"
    assert v.header["guard"].text == "GUARD clear" and v.header["guard"].level == "ok"
    assert v.word.text.startswith("ARMED") and v.usv["state"] == "AUTO_HOLD · HOLD" and v.mission_armed == "scoutsim"
    assert v.rec.text == "REC ○ off" and not v.rec.on                       # the sim's usv_ui runs without --log run
    assert v.alarms == [] and v.alarms_level == "idle"


def test_r5_running_scout(cm):
    v = view(cm, "r5_task")
    assert [steps(v)[n] for n in (1, 3, 4, 5)] == ["done"] * 4 and steps(v)[6] == "now" and steps(v)[7] == "now"
    assert v.steps[5].chip in ("APPROACH", "LOG") and v.banner["title"] == "RUN STARTED · run 1001"
    assert [b.word for b in v.steps[5].buttons] == ["hold", "abort"]
    assert v.word.text.startswith("RUNNING · SCOUT") and v.word.level == "ok"
    assert v.usv["task"].startswith("SCOUT · ")
    start = next(b for b in v.usv_buttons if b.word == "start_task")
    assert not start.ok and "AUTO_HOLD only" in start.why and start.hint                 # greyed WITH its reason, never silently
    assert len(v.map.polygon) == 4 and v.map.polygon_src == "received" and v.map.pose is not None
    assert {x["id"] for x in v.map.visited} >= {"B1", "B2"}                             # the scout log's "B1 logged lat lon"
    assert len(v.log) == len(load("r5_task")["state"]["log"]) <= M.LOG_N
    assert {ln.group for ln in v.log} >= {"ui", "judge", "usv"}


def test_r6_alarm(cm):
    v = view(cm, "r6_alarm")
    assert steps(v)[8 if len(v.steps) == 8 else 7] == "done" and v.banner["title"] == "RUN ENDED"   # R6: the run ended
    assert v.word.text == "NOT READY · fault EKF_BAD" and v.header["v"].text == "R6 · FAULT"
    assert len(v.alarms) == 1 and v.alarms[0]["level"] == "ALARM" and v.alarms_unacked == 1 and v.alarms_level == "bad"
    assert any(ln.sev == "bad" for ln in v.log)


def test_stale_and_down(cm):
    t = load("r2")
    s = dict(t["state"], ocs_age=4.2)
    v = cm.apply(s, t["fast"], t["alarms"])
    assert v.stale and v.link == "STALE" and v.header["link"].level == "warn"
    v = cm.apply(None, None, None, misses=2)                                              # misses keep the last good state
    assert v.link == "STALE" and v.steps[3].state == "now"
    v = cm.apply(None, None, None, misses=3)
    assert v.down and v.link == "DOWN" and v.header["link"].level == "bad" and "3 polls" in v.header["link"].text
    fresh = M.ConsoleModel()
    assert fresh.apply(None, None, None).link == "WAITING"


def test_rec_from_the_backend_key_and_inferred(cm):
    t = load("r5_task")
    clock = [1000.0]
    m = M.ConsoleModel(clock=lambda: clock[0])
    s = dict(t["state"], logging={"usv": {"on": True, "file": "/x/logs/usv/run_2026-11-11_09-14", "bytes": 12e6, "t0": 1}})
    v = m.apply(s, t["fast"], t["alarms"], now=1000.0)
    assert v.rec.text == "REC ● run_2026-11-11_09-14 · 12 MB" and v.rec.level == "ok" and not v.rec.inferred
    v = m.apply(s, t["fast"], t["alarms"], now=1011.0)                                    # not grown for 11 s
    assert v.rec.text.startswith("REC STOPPED") and v.rec.level == "bad"
    s2 = json.loads(json.dumps(t["state"]))
    next(n for n in s2["nodes"]["boat"] if n["node"] == "usv_ui")["body"]["log"] = "logs/usv/run_20261111_0914/"
    v = cm.apply(s2, t["fast"], t["alarms"])
    assert v.rec.text == "REC ● run_20261111_0914 · inferred" and v.rec.level == "warn" and v.rec.inferred


def test_backend_v018305_keys(cm):
    """B's v0.1.83.5: state.logging (usv bytes None until the NX has it) and lat/lon in /fast.json."""
    t = load("r5_task")
    s = dict(t["state"], logging={"usv": {"on": True, "file": "logs/usv/run_20261111_0914", "bytes": None, "t0": None},
                                  "usv_sqlite": {"on": False}, "ocs": {"on": False}, "any": True})
    v = cm.apply(s, t["fast"], t["alarms"])
    assert v.rec.text == "REC ● run_20261111_0914" and v.rec.level == "ok" and not v.rec.inferred
    f = json.loads(json.dumps(t["fast"]))
    f["v"].update(lat=1.5, lon=103.5)
    assert cm.apply(t["state"], f, t["alarms"]).map.pose == (1.5, 103.5)
    assert cm.apply(t["state"], t["fast"], t["alarms"]).map.pose == (t["state"]["status"]["lat"], t["state"]["status"]["lon"])


def test_transition_events(cm):
    a, b = load("r2"), load("r5_task")
    assert cm.apply(a["state"], a["fast"], a["alarms"]).events == []                     # the first poll says nothing
    ev = cm.apply(b["state"], b["fast"], b["alarms"]).events
    assert ("ok", "RunStart received · run 1001") in ev
    s = json.loads(json.dumps(b["state"]))
    s["status"]["state"] = "AUTO_SAFE_HOLD"
    s["status"]["guard"] = {"chip": "stop", "range_m": 1.2}
    s["judge"]["connected"] = False
    ev = cm.apply(s, b["fast"], b["alarms"]).events
    assert ("warn", "AUTO_SAFE_HOLD entered") in ev and ("bad", "judge lost") in ev and ("bad", "GUARD STOP · 1.2 m") in ev


def test_missions_carry_scout_and_the_sim_suffix(cm):
    v = view(cm, "r2")
    codes = [c for c, _ in v.missions]
    assert "scoutsim" in codes and "goto" in codes                                        # the fake FCU: the sim variants
    st = json.loads(json.dumps(load("r2")["state"]))
    st["status"]["fcu_link"] = "uart"
    st["flavour"]["fcu"] = "uart"
    assert "scout" in [c for c, _ in cm.apply(st, None, None).missions]


def test_every_button_names_a_word_and_a_key(cm):
    v = view(cm, "r5_task")
    bs = v.usv_buttons + v.uav_buttons + [b for s in v.steps for b in s.buttons]
    assert all(b.word and b.label and b.key for b in bs)
    assert len({b.key for b in v.usv_buttons + v.uav_buttons}) == len(v.usv_buttons + v.uav_buttons)
    arm = next(b for b in v.usv_buttons if b.word == "arm")
    sr = next(b for b in v.usv_buttons if b.word == "stop_reset")
    assert arm.confirm and sr.confirm                                                     # ARM and STOP & RESET ask first


def test_the_palettes_come_from_themes_css():
    css = (Path(__file__).parents[1] / "optiprimex" / "tools" / "themes.css").read_text()
    assert parse_css(css, "light")["bg"] == "#eef1f4" and parse_css(css, "dark")["bg"] == "#0e1318"
    assert palette("light")["action"] == "#ffd60a" and palette("dark")["alarm"] == "#f94d4e"


def test_the_mission_tab_line_and_stages(cm):
    meta = load("meta")
    t4 = next(m for m in meta["missions"] if m["code"] == "m0t4")
    opts = t4["options"]
    line = M.mission_line("mission", "m0t4sim", opts, {"--target-colour": "green", "--scan-x": "30", "--scan-y": ""}, auto=True)
    assert line == "mission m0t4sim --target-colour green --scan-x 30 --auto"
    assert M.mission_line("start_task", "m0t2", [{"flag": "--names", "text": "EN"}], {"--names": "EN,G1 X"}) == "start_task m0t2 --names 'EN,G1 X'"
    assert M.mission_line("start_task", "m1t1", [{"flag": "--step", "check": True}], {"--step": True}, auto=True) == "start_task m1t1 --step"
    rows = M.mission_stages(meta, "scoutsim", "SCOUT", "LOG")
    assert [n for n, _m, now in rows if now] == ["LOG"] and rows[0][0] == "WAIT_START"
    assert all(not now for _n, _m, now in M.mission_stages(meta, "m1t1", "SCOUT", "LOG"))


def test_the_uav_heartbeat_button(cm):
    """2026-11-11 (Dmitrii: "UAV heartbeat and declare buttons ... so I see both vehicles in the RC page"): report on|off uav."""
    t = load("r2")
    v = cm.apply(t["state"], t["fast"], t["alarms"])
    hb = next(b for b in v.uav_buttons if b.key == "uav-hb")
    assert hb.word == "report on uav" and hb.label == "▶ HEARTBEAT UAV" and not hb.confirm
    assert [b.key for b in v.uav_buttons][:2] == ["uav-hb", "uav-decl"] and v.uav["heartbeat"].startswith("off · telemetry none")
    s = json.loads(json.dumps(t["state"]))
    s["ocs"]["uav"].update(report_on=True, telemetry_age_s=0.4)
    v = cm.apply(s, t["fast"], t["alarms"])
    hb = next(b for b in v.uav_buttons if b.key == "uav-hb")
    assert hb.word == "report off uav" and hb.label == "✓ HEARTBEAT ON" and hb.confirm and v.uav["heartbeat"] == "on · telemetry 0.4 s"


def test_vehicle_cards_imu_compass_heartbeat(cm):
    """2026-11-11 (Dmitrii: "a tab for vehicles ... the important IMU, compass, heartbeat status per each vehicle")."""
    t = load("r5_task")
    usv, uav = M.vehicle_cards(t["state"], t["fast"])
    rows = {k: (txt, lvl) for k, txt, lvl in usv.rows}
    assert usv.vid == "USV1" and usv.link.level == "ok" and usv.hdg == t["fast"]["v"]["hdg_deg"] and usv.roll == 0.0
    assert rows["IMU"][0].startswith("roll 0.0° · pitch 0.0°") and rows["COMPASS"][0].startswith("heading 342.8°")
    assert rows["GPS"] == ("3D · 14 sats · HDOP 1.2 · EKF ok", "ok")
    assert rows["HEARTBEAT"][0].startswith("sending · AUTO · #160") and rows["JUDGE"][1] == "ok"
    urows = {k: (txt, lvl) for k, txt, lvl in uav.rows}
    assert uav.vid == "UAV1" and uav.link.level == "bad" and urows["TELEMETRY"][1] == "bad"
    assert urows["HEARTBEAT"][0].startswith("not sending")
    s = json.loads(json.dumps(t["state"]))
    s["uav"] = {"state": "AUTO", "link": "up", "age_s": 0.3, "mode": "LOITER", "armed": True, "hdg_deg": 90.0, "roll_deg": 2.0,
                "pitch_deg": -1.0, "altitude_hae_m": 22.5, "health": {"telemetry_age_s": 0.2, "telemetry_n": 812, "fence_ok": True}}
    s["ocs"]["reporting"]["hb"]["UAV1"] = {"n": 12, "state": "STATE_AUTO", "age_s": 0.4, "stopped": False}
    s["judge"]["heartbeat_age"]["UAV1"] = 0.6
    uav = M.vehicle_cards(s, t["fast"])[1]
    urows = {k: (txt, lvl) for k, txt, lvl in uav.rows}
    assert uav.link.level == "ok" and uav.hdg == 90.0 and uav.roll == 2.0 and urows["IMU"][1] == "ok"
    assert urows["HEARTBEAT"][0].startswith("sending · AUTO · #12") and urows["JUDGE"] == ("the judge saw it 0.6 s ago", "ok")
    s["status"]["roll_deg"] = 30.0
    assert {k: lvl for k, _t, lvl in M.vehicle_cards(s, None)[0].rows}["IMU"] == "bad"           # a 30° heel is red


def test_the_command_catalogue_has_every_word_its_modifiers_and_rules():
    """2026-11-11 (Dmitrii: "on CMD tab ... all available commands with tooltips and modificators")."""
    cat = {c.word: c for c in M.command_catalogue(load("meta"))}
    from optiprimex.tools import bench_tui as T
    assert set(T.INTENT_WORDS) <= set(cat)                                         # every word /cmd accepts is listed
    st = cat["start_task"]
    assert st.group == "TASK" and st.usage.startswith("start_task") and any(spec.startswith("--planner simple|s6") for spec, _ in st.modifiers)
    assert "run R5" in st.rules and "vehicle AUTO_HOLD" in st.rules and st.action == "START_TASK"
    sk = cat["station_keep"]
    assert sk.group == "OTHER" and sk.usage == "station_keep hold|loiter"         # no usage line in the backend: from ocs_cli's parser
    assert cat["report"].usage == "report on|off [uav|usv]" and cat["usv"].usage == "usv ready on|off"
    assert not any(spec in ("--host", "--port") for c in cat.values() for spec, _ in c.modifiers)   # the shared connection flags left out


def test_the_one_press_logging_view():
    """2026-11-11 (Dmitrii: one button for every log, the clocks checked first) - B's state.logging.session + state.clocks (v0.1.84.5)."""
    s = json.loads(json.dumps(load("r5_task")["state"]))
    lv = M.logging_view(s)
    assert not lv.running and lv.button == "● START LOGGING" and lv.clocks.startswith("clocks: not measured") and lv.clock_problem == ""
    s["clocks"] = {"nx": {"offset_s": 0.03, "ok": True}, "agx": {"offset_s": 0.83, "ok": False}, "uav": {"offset_s": None}}
    lv = M.logging_view(s)
    assert lv.clocks == "clocks: NX +0.03 s ✓ · AGX +0.83 s ✗ · UAV ? (no recent beat)" and lv.clocks_level == "bad"
    assert "AGX +0.83 s vs the Mac" in lv.clock_problem and "UAV" not in lv.clock_problem      # only NX / AGX gate the start
    s["clocks"]["agx"] = {"offset_s": 0.05, "ok": True}
    assert M.logging_view(s).clock_problem == ""
    s["logging"] = dict(s.get("logging") or {}, session={"running": True, "label": "log_20261111_151200",
                                                          "parts": {"fcu": {"ok": True}, "agx": {"ok": True}, "mac": {"ok": False}}})
    lv = M.logging_view(s)
    assert lv.running and lv.button == "■ STOP LOGGING" and lv.label == "log_20261111_151200  (fcu ✓ · agx ✓ · mac ✗)"


def test_the_uav_run_plan_skips_what_is_done_and_never_arms():
    """2026-11-11 (Dmitrii: "boat holds in auto good for now, make it quick")."""
    s = json.loads(json.dumps(load("r2")["state"]))                      # AUTO_HOLD, armed, R2, usv_flow auto (heartbeat + ready on)
    words, why = M.uav_run_plan(s)
    assert why == "" and words == ["station_keep loiter", "report on uav", "uav ready on", "declare uav on", "declare"]
    assert "arm" not in words and "handover" not in words
    s["status"]["armed"] = False
    assert M.uav_run_plan(s) == ([], "the FCU is disarmed: ARM first (it asks), then UAV RUN")
    s["ui_active"] = True
    assert M.uav_run_plan(s)[1].startswith("usv_ui holds the FCU")


def test_the_uav_card_says_why_declare_uav_would_refuse():
    s = json.loads(json.dumps(load("r5_task")["state"]))
    s["ocs"]["declaration_preview"] = {"uav_refusal": "the FCU has no single polygon fence to declare (Mission Planner: Fence -> Read / Write)"}
    uav = M.vehicle_cards(s, None)[1]
    assert dict((k, (t, l)) for k, t, l in uav.rows)["DECLARE"] == ("✗ the FCU has no single polygon fence to declare (Mission Planner: Fence -> Read / Write)", "bad")
    s["ocs"]["declaration_preview"] = {"uav_refusal": None, "body": {"uav_geofence": [[1, 2], [1, 3], [2, 3], [2, 2], [1, 2]]}}
    assert dict((k, (t, l)) for k, t, l in M.vehicle_cards(s, None)[1].rows)["DECLARE"] == ("✓ fence 4 pts from the drone's FCU", "ok")


def test_wp42_follow_ups_hidden_missions_auto_start_and_the_declare_session():
    """WP44 against session B's v0.1.86 contract: Mission.hidden, mission.auto, logging.session.auto."""
    meta = load("meta")
    for m in meta["missions"]:
        if m["code"] in ("m0t0", "m0t1", "goto"):
            m["hidden"] = True
    cm = M.ConsoleModel()
    cm.set_meta(meta)
    t = load("r2")
    codes = [c for c, _ in cm.apply(t["state"], t["fast"], t["alarms"]).missions]
    assert not {"m0t0sim", "m0t1sim", "goto"} & set(codes) and "scoutsim" in codes
    s = json.loads(json.dumps(t["state"]))
    s["ocs"]["mission"] = {"code": "scout", "auto": True, "task": "SCOUT"}
    v = cm.apply(s, t["fast"], t["alarms"])
    assert "starts by itself at RunStart (3-2-1)" in v.steps[5].detail
    s["logging"] = dict(s.get("logging") or {}, session={"running": True, "auto": True, "label": "m1t1-usv_scout_3", "t0": "2026-11-12T03:15:00Z"})
    lv = M.logging_view(s)
    assert lv.auto and lv.t0 == "2026-11-12T03:15:00Z" and lv.label.endswith("automatic at DECLARE")


def test_outside_course_buoys_are_greyed_and_counted():
    from optiprimex.tools.tkconsole import canvasmap as C
    mp = json.loads(json.dumps(load("map")))
    mp["map"]["entities"][0]["outside_course"] = True
    L = C.build_layer(load("course"), mp, None, None)
    assert sum(1 for i in L["items"] if i["outside"]) == 1


def test_the_loop_row_reads_the_tick_instrument():
    s = json.loads(json.dumps(load("r5_task")["state"]))
    rows = {k: (t, l) for k, t, l in M.vehicle_cards(s, None)[0].rows}
    assert rows["LOOP"][0].startswith("—")
    s["status"]["health"]["tick_ms"] = {"p50": 12, "p95": 40, "max": 120, "n": 600, "slow_per_min": 0}
    assert {k: (t, l) for k, t, l in M.vehicle_cards(s, None)[0].rows}["LOOP"] == ("tick p95 40 ms · max 120 ms · 0 slow / min", "ok")
    s["status"]["health"]["tick_ms"].update(p95=180, max=520, slow_per_min=4)
    assert {k: l for k, _t, l in M.vehicle_cards(s, None)[0].rows}["LOOP"] == "bad"


def test_the_guard_lag_row():
    s = json.loads(json.dumps(load("r5_task")["state"]))
    assert {k: t for k, t, _l in M.vehicle_cards(s, None)[0].rows}["GUARD LAG"].startswith("—")
    s["status"]["health"]["guard_ms"] = {"pipe_ms_p50": 90, "pipe_ms_p95": 140, "net_ms_p50": 12, "n": 60}
    t, l = {k: (t, l) for k, t, l in M.vehicle_cards(s, None)[0].rows}["GUARD LAG"]
    assert t.startswith("camera → publish p50 90 · p95 140 ms") and l == "ok"
    s["status"]["health"]["guard_ms"]["pipe_ms_p95"] = 320
    assert {k: l for k, _t, l in M.vehicle_cards(s, None)[0].rows}["GUARD LAG"] == "bad"
