"""Playbook verdicts against the guard tables, ocs_cli parser flexibility, and the TUI renderer (pure, no curses)."""
from __future__ import annotations

import os

import pytest

from optiprimex.tools import bench_tui, ocs_cli, playbook


def ocs(r="R2", v="MANUAL", run_id=None, decl=0, bench=True, task_state=None, stalled=False):
    return {"r": r, "r_state": {"R2": "COURSE_VALID", "R3": "DECLARED", "R4": "READY", "R5": "RUNNING", "R6": "ENDED"}.get(r, r),
            "run_id": run_id, "declaration_seq": decl, "bench_mode": bench, "stalled": stalled, "judge": "fake", "judge_connected": True,
            "vehicles": {"USV1": {"state": v, "robot_state": "STATE_" + ("AUTO" if v.startswith("AUTO") else v), "current_task": "TASK_NONE",
                                  "task_state": task_state, "age_s": 0.1}}, "heartbeats_sent": {}, "reports_sent": 0, "msg_rate": 0}


ROWS = [
    # action, ocs state, expected ok, fragment of why/hint
    ("DECLARE", ocs("R2"), True, ""),
    ("DECLARE", ocs("R5", "AUTO_HOLD", 1001, 1), False, "end_run"),
    ("DECLARE", ocs("R3"), False, "already declared"),
    ("DECLARE", ocs("R4", "AUTO_HOLD", None, 1, stalled=True), True, ""),
    ("ENABLE_AUTO", ocs("R3", "MANUAL", None, 1), True, ""),
    ("ENABLE_AUTO", ocs("R5", "AUTO_HOLD", 1001, 1), False, "already in auto"),
    ("ENABLE_AUTO", ocs("R2", "KILLED"), False, "confirm_safe"),
    ("DISABLE_AUTO", ocs("R5", "AUTO_TASK", 1001, 1), False, "abort"),
    ("START_TASK", ocs("R5", "AUTO_HOLD", 1001, 1), True, ""),
    ("START_TASK", ocs("R4", "AUTO_HOLD", None, 1), False, "RunStart"),
    ("START_TASK", ocs("R4", "AUTO_HOLD", None, 1, bench=False), False, "R5"),
    ("RESUME", ocs("R5", "AUTO_HOLD", 1001, 1), False, "nothing is held"),
    ("ABORT_TASK", ocs("R5", "AUTO_SAFE_HOLD", 1001, 1), True, ""),
    ("CONFIRM_SAFE", ocs("R5", "MANUAL", 1001, 1), False, "nothing to confirm"),
    ("CONFIRM_SAFE", ocs("R5", "FAULT", 1001, 1), True, ""),
    ("END_RUN", ocs("R2"), False, "nothing to end"),
    ("END_RUN", ocs("R5", "AUTO_TASK", 1001, 1), True, ""),
    ("GOTO", ocs("R5", "AUTO_HOLD", 1001, 1, bench=False), False, "bench only"),
    ("GOTO", ocs("R5", "AUTO_HOLD", 1001, 1), True, ""),
    ("ARM", ocs("R5", "AUTO_HOLD", 1001, 1), False, "never"),
]


@pytest.mark.parametrize("action,state,ok,frag", ROWS, ids=[f"{r[0]}-{r[1]['r']}-{r[1]['vehicles']['USV1']['state']}" for r in ROWS])
def test_playbook_matrix(action, state, ok, frag):
    v = playbook.check(action, state)
    assert v.ok == ok, (v.why, v.hint)
    if frag:
        assert frag in (v.why + " " + v.hint)


def test_playbook_tasks_and_injects():
    st = ocs("R5", "AUTO_HOLD", 1001, 1)
    assert playbook.mission_for("m1t2").status == "built"                       # v0.1.66 (WP16): built - needs the UAV (the OCS checks it)
    assert playbook.check("START_TASK", st, task="T3").ok
    assert playbook.check("START_TASK", st, task="T1").ok
    assert playbook.check("START_TASK", st, task="T4").ok
    assert playbook.check("INJECT", st, what="assistance_request").ok
    assert not playbook.check("INJECT", ocs("R3", "AUTO_HOLD", None, 1), what="assistance_request").ok     # no run
    assert not playbook.check("INJECT", ocs("R3", "MANUAL", None, 1), what="keep_out_zone").ok              # not in AUTO
    assert playbook.check("INJECT", ocs("R3", "MANUAL", None, 1), what="run_start").ok
    assert not playbook.check("INJECT", st, what="readiness_confirm").ok
    assert playbook.check("INJECT", ocs("R5", "AUTO_TASK", 1001, 1, task_state="WAIT_CONFIRM"), what="readiness_confirm").ok
    assert "declare" in playbook.suggest(ocs("R2"))
    assert "enable_auto" in playbook.suggest(ocs("R3", "MANUAL", None, 1))
    assert "start_task" in playbook.suggest(ocs("R5", "AUTO_HOLD", 1001, 1))
    assert len(playbook.help_lines()) > 20


def test_cli_parser_accepts_options_anywhere_and_aliases():
    ns = ocs_cli.parse(["--wait", "5", "enable_auto"])
    assert ns.cmd == "enable_auto" and ns.wait == 5
    ns = ocs_cli.parse(["enable_auto", "--wait", "7", "--port", "1999"])
    assert ns.cmd == "enable_auto" and ns.wait == 7 and ns.port == 1999
    ns = ocs_cli.parse(["auto"])
    assert ns.cmd == "enable_auto" and ns.wait == 3.0 and ns.port == 1882
    ns = ocs_cli.parse(["task"])
    assert ns.cmd == "start_task" and ns.task is None                      # no task = the OCS runs the preselected mission, else T4
    assert ocs_cli.intent_spec(ocs_cli.parse(["start_task", "test_auto"]))[1] == "TEST_AUTO"
    assert ocs_cli.intent_spec(ocs_cli.parse(["reset"]))[0] == "RESET_RUN"
    assert ocs_cli.intent_spec(ocs_cli.parse(["mission", "TEST_MANUAL", "--auto"]))[2] == {"task": "TEST_MANUAL", "auto": True}
    ns = ocs_cli.parse(["stop", "--dry-run"])
    assert ns.cmd == "abort" and ns.dry_run
    action, task, args, what = ocs_cli.intent_spec(ocs_cli.parse(["inject", "assistance", "--lat", "1.3", "--lon", "103.8", "--cseq", "3"]))
    assert action == "INJECT" and what == "assistance_request" and args["command_seq"] == 3 and args["lat"] == 1.3
    action, task, args, _ = ocs_cli.intent_spec(ocs_cli.parse(["goto", "--x", "20", "--y", "5"]))
    assert action == "GOTO" and args == {"x_n": 20.0, "y_e": 5.0}


def test_tui_render_without_curses():
    import json
    m = bench_tui.Model("USV1", "usv")
    view = bench_tui.View(m, "USV1")
    c = bench_tui.Canvas(150, 50)
    view.draw(c, 0, "", "")
    text = "\n".join(c.lines())
    assert "no ocs/state" in text and "COMMANDS" in text and "next: start ocs_app" in text
    assert "OCS CONSOLE" in c.lines()[1] and "WARNINGS" in text and "hub broker DOWN" in text      # the banner and the warnings box
    m.on_hub("ocs/state", json.dumps({"body": ocs("R5", "AUTO_TASK", 1001, 1, task_state="TRANSIT")}).encode())
    m.on_hub("team/optiprime/v1/status/usv", json.dumps({"body": {"state": "AUTO_TASK", "robot_state": "STATE_AUTO", "mode": "GUIDED", "armed": True,
                                                                  "kill": False, "estop": False, "current_task": "TASK_DYNAMIC_INCIDENT",
                                                                  "task_name": "T4", "task_state": "TRANSIT", "health": {"bridge_up": True, "fcu_age_s": 0.05, "rc_ok": True},
                                                                  "lat": 1.3, "lon": 103.8, "hdg_deg": 10, "sog_mps": 0.5, "local": {"x_n": 1.0, "y_e": 2.0}}}).encode())
    m.on_hub("team/optiprime/v1/cmd/usv/run", json.dumps({"msg_id": "m1", "body": {"action": "HOLD", "args": {}}}).encode())
    m.on_hub("team/optiprime/v1/ack/usv", json.dumps({"body": {"ack_of": "m1", "result": "ACCEPTED", "state": "AUTO_SAFE_HOLD"}}).encode())
    m.on_hub("ocs/alert", json.dumps({"body": {"event": "alert", "kind": "run_start_stall", "waited_s": 60}}).encode())
    m.on_hub("rc/command", json.dumps({"kind": "run_start", "run_id": 1001}).encode())
    m.hub_up = True
    c = bench_tui.Canvas(150, 50)
    view.draw(c, 0, "star", "last line")
    text = "\n".join(c.lines())
    assert "[R5]" in text and "[V4]" in text and "TRANSIT" in text and " ARMED " in text
    assert "ack ACCEPTED HOLD -> AUTO_SAFE_HOLD" in text and "ALERT run_start_stall" in text
    assert "usage: start_task" in text and "> star" in text and "last: last line" in text
    # the COMMANDS panel: grouped, every word present, verdict colours (green = allowed now) via the canvas attrs
    for word in ("declare", "ready", "end_run", "reset", "mission", "state", "next", "enable_auto", "disable_auto", "manual", "hold", "resume",
                 "confirm_safe", "start_task", "goto", "abort", "step", "inject", "kill", "rc6", "estop", "ekf", "armed", "fcu pause", "teleport",
                 "map", "clear", "help", "quit"):
        assert word in text, word
    def attr_of(word):
        for y, row in enumerate(c.lines()):
            x = row.find(" " + word + " ")
            if x >= 0 and y > 30:
                return c.attrs[y][x + 1]
        return None
    assert attr_of("hold") == "ok" and attr_of("declare") == "dim" and attr_of("resume") == "dim"
    # MISSION box + stage chain, WHERE WE ARE, FLOW box, EVENTS / CONSOLE split, WARNINGS with the stall alert
    assert "MISSION" in text and "T4" in text and "LISTEN" in text and "WHERE WE ARE" in text and "FLOW" in text
    assert "EVENTS" in text and "CONSOLE" in text and "run_start_stall" in text and "stall" in text.lower()
    for i, name in enumerate(bench_tui.SCREENS[1:], start=1):
        c = bench_tui.Canvas(150, 50)
        view.draw(c, i, "", "")
        text = "\n".join(c.lines())
        assert f"[{name}]" in text
    assert "NETWORK" in bench_tui.SCREENS and "MISSION" in bench_tui.SCREENS
    m.on_hub("presence/ocs_app", json.dumps({"node": "ocs_app", "up": True, "host": "mac", "pid": 1}).encode())
    m.on_boat("presence/mission_manager", json.dumps({"node": "mission_manager", "up": False, "host": "orin", "pid": 2}).encode())
    c = bench_tui.Canvas(150, 50); view.draw(c, bench_tui.SCREENS.index("NETWORK"), "", ""); text = "\n".join(c.lines())
    assert "ocs_app" in text and "mission_manager" in text and "DOWN" in text and "presence/<node>" in text
    c = bench_tui.Canvas(150, 50); view.draw(c, bench_tui.SCREENS.index("HELP"), "", ""); assert "intent" in "\n".join(c.lines())
    c = bench_tui.Canvas(80, 20); view.draw(c, 0, "", ""); assert "terminal too small" in "\n".join(c.lines())
    c = bench_tui.Canvas(120, 44); view.draw(c, 0, "", ""); assert "terminal too small" not in "\n".join(c.lines())   # the minimum
    assert bench_tui.usage_for("star").startswith("start_task") and bench_tui.usage_for("sta").startswith("commands:") and bench_tui.usage_for("zzz").startswith("unknown")
    assert bench_tui.usage_for("auto").startswith("enable_auto") and bench_tui.usage_for("e").startswith("commands:")


def test_tui_boat_screen_and_map_sources():
    """BOAT tab: the text map from telemetry/usv/map, the detector line, who has the FCU; usv_ui's digest wins while fresh."""
    import json
    from optiprimex.tools import textmap
    m = bench_tui.Model("USV1", "usv")
    view = bench_tui.View(m, "USV1")
    c = bench_tui.Canvas(150, 50)
    view.draw(c, bench_tui.SCREENS.index("BOAT"), "", "")
    text = "\n".join(c.lines())
    assert "[BOAT]" in text and "no telemetry/usv/map yet" in text and "mission manager (no usv_ui" in text
    digest = {"co": {"lat": 1.3, "lon": 103.8}, "boat": {"x_n": 5.0, "y_e": 0.0, "hdg_deg": 0.0, "trail": [[0, 0], [2.5, 0]]},
              "entities": [{"id": 1, "kind": "buoy", "colour": "red", "x_n": 12.0, "y_e": 2.0, "sigma_m": 0.2, "n_obs": 9, "age_s": 1.0, "state": "tracked"},
                           {"id": 2, "kind": "buoy", "colour": "green", "x_n": 12.0, "y_e": -2.0, "sigma_m": 0.2, "n_obs": 9, "age_s": 1.0, "state": "tracked"},
                           {"id": 3, "kind": "dock", "colour": "white", "x_n": 50.0, "y_e": 6.0, "sigma_m": 0.3, "n_obs": 2, "age_s": 40.0, "state": "stale"}],
              "gates": [{"red": 1, "green": 2, "mid_n": 12.0, "mid_e": 0.0, "through_hdg_deg": 0.0, "width_m": 4.0, "passed": False}],
              "dock": None, "path": [[6, 0], [8, 0]], "carrot": [8, 0], "points": {"A": [20, 0]}, "n_ingested": 18, "ingest_age_s": 0.2}
    m.on_hub("team/optiprime/v1/telemetry/usv/map", json.dumps({"source": "mission_manager", "body": digest}).encode())
    assert m.map_src == "mission_manager"
    ui = dict(digest, task_state="ACTIVE", task="goto A", mode="GUIDED", armed=True, cv={"on": True, "n_rx": 50, "n_used": 40, "health_age_s": 0.4})
    m.on_hub("team/optiprime/v1/telemetry/usv/map", json.dumps({"source": "usv_ui", "body": ui}).encode())
    assert m.map_src == "usv_ui"                                          # the pool tool's digest wins while fresh
    m.on_hub("team/optiprime/v1/telemetry/usv/map", json.dumps({"source": "mission_manager", "body": digest}).encode())
    assert m.map_src == "usv_ui"
    m.on_boat("presence/usv_ui", json.dumps({"node": "usv_ui", "up": True, "pid": 3}).encode())
    m.on_boat("local/usv/perception/health", json.dumps({"body": {"ok": True, "detector": "fake", "rate_hz": 5.0, "pose_age_s": 0.1, "n_published": 100}}).encode())
    c = bench_tui.Canvas(150, 50)
    view.draw(c, bench_tui.SCREENS.index("BOAT"), "", "")
    text = "\n".join(c.lines())
    assert "usv_ui ACTIVE" in text and "source usv_ui" in text and "usv_ui cv ON" in text
    assert "detector fake ok" in text and "gates 1" in text and "entities 2 tracked / 3" in text
    assert "usv_ui: goto A" in text and "GUIDED" in text
    grid, m10 = textmap.render_text_map(60, 20, {"A": (20, 0)}, (5.0, 0.0, 0.0), (8, 0), [(0, 0)], path=[(6, 0)],
                                        entities=[(12, 2, "buoy", "red", False), (12, -2, "buoy", "green", False), (50, 6, "dock", "white", True)], gates=[(12, 0)])
    chars = "".join(ch for row in grid for ch, _ in row)
    assert "R" in chars and "G" in chars and "d" in chars and "|" in chars and "^" in chars and "+" in chars and "A" in chars and m10 > 0
    assert "map" in [w for w, _, _ in bench_tui.USAGE]
    assert bench_tui.map_url("127.0.0.1", 9001).endswith("map.html?host=127.0.0.1&port=9001")
    assert os.path.exists(bench_tui.MAP_HTML)
    # a retained beat from an old run (t_unix long ago) replayed by the broker on subscribe: DOWN at once, not UP for 6 s
    import time
    m.on_boat("presence/usv_ui", json.dumps({"node": "usv_ui", "up": True, "pid": 3, "t_unix": time.time() - 3600}).encode())
    nodes = {n: (up, age) for n, up, age, _ in m.nodes("boat")}
    assert nodes["usv_ui"][0] is False and nodes["usv_ui"][1] > 3000
    c = bench_tui.Canvas(150, 50)
    view.draw(c, bench_tui.SCREENS.index("BOAT"), "", "")
    assert "mission manager (no usv_ui" in "\n".join(c.lines()) and "op-tui v" in c.lines()[0]
    html = open(bench_tui.MAP_HTML).read()
    assert "SOURCE_FORGET_MS" in html and "ageSources()" in html


def test_flow_and_handover_hints():
    """The hints and the FLOW strip know who commands the FCU: usv_ui ACTIVE blocks enable_auto until `handover`."""
    import json
    ocs = {"r": "R3", "r_state": "DECLARED", "run_id": None, "declaration_seq": 1, "bench_mode": True,
           "vehicles": {"USV1": {"state": "MANUAL", "task_state": None}}}
    assert playbook.suggest(ocs, "USV1", ui_active=True).startswith("usv_ui holds the FCU: type `handover`")
    assert playbook.suggest(ocs, "USV1", ui_active=False) == "enable_auto"
    assert playbook.suggest(ocs, "USV1") == "enable_auto"                      # boat broker not visible: the plain hint
    vd = playbook.check("ENABLE_AUTO", ocs, "USV1", ui_active=True)
    assert not vd.ok and "usv_ui ACTIVE" in vd.why and "handover" in vd.hint
    assert playbook.check("ENABLE_AUTO", ocs, "USV1", ui_active=False).ok
    assert playbook.flow_stage(ocs, "USV1", True) == 0 and playbook.flow_stage(ocs, "USV1", False) == 3
    ocs5 = dict(ocs, r="R5", run_id=1001, vehicles={"USV1": {"state": "AUTO_TASK", "task_state": "GO"}})
    assert playbook.FLOW[playbook.flow_stage(ocs5, "USV1", False)][0] == "start_task/goto"
    assert playbook.FLOW[playbook.flow_stage(dict(ocs5, vehicles={"USV1": {"state": "AUTO_SAFE_HOLD"}}), "USV1", False)][0] == "hold<->resume"
    assert playbook.FLOW[playbook.flow_stage(dict(ocs5, r="R6", vehicles={"USV1": {"state": "MANUAL"}}), "USV1", False)][0] == "end_run"
    # Model.ui_state: the mission manager's view (mm/state ui_active) wins; usv_ui's presence + digest is the fallback
    m = bench_tui.Model("USV1", "usv")
    assert m.ui_state() is None
    m.boat_up = True
    assert m.ui_state() is False
    m.on_boat("presence/usv_ui", json.dumps({"node": "usv_ui", "up": True, "pid": 3}).encode())
    assert m.ui_state() is True                                                 # up, no digest yet: ACTIVE
    m.on_hub("team/optiprime/v1/telemetry/usv/map", json.dumps({"source": "usv_ui", "body": {"task_state": "HANDED_OVER", "entities": []}}).encode())
    assert m.ui_state() is False
    m.on_boat("local/usv/mm/state", json.dumps({"body": {"state": "MANUAL", "ui_active": True, "fcu": {}}}).encode())
    assert m.ui_state() is True
    m.on_hub("ocs/state", json.dumps({"body": ocs}).encode())
    view = bench_tui.View(m, "USV1")
    m.hub_up = True
    c = bench_tui.Canvas(150, 50)
    view.draw(c, bench_tui.SCREENS.index("MAIN"), "", "")
    text = "\n".join(c.lines())
    # the FLOW box: boat row, connector, OCS row; the current cell in reverse video ("now"), the FCU note at the right
    rows = c.lines()
    fy = next(i for i, ln in enumerate(rows) if ln.lstrip().startswith("\u250c\u2500 FLOW"))
    assert "boat" in rows[fy + 1] and "usv_ui ACTIVE" in rows[fy + 1] and "handover" in rows[fy + 1] and rows[fy + 1].rstrip().endswith("FCU: usv_ui (boat console) \u2502")
    assert "OCS" in rows[fy + 3] and "declare" in rows[fy + 4] and "enable_auto" in rows[fy + 4] and "takeover/stop" in rows[fy + 4]
    x = rows[fy + 1].find("usv_ui ACTIVE")
    assert c.attrs[fy + 1][x] == "now" and c.attrs[fy + 4][rows[fy + 4].find("declare")] == "dim"
    assert "R2>R3" in rows[fy + 5] and "V3" in rows[fy + 5]
    assert "next: usv_ui holds the FCU: type `handover`" in text
    rows2 = playbook.flow_rows(5)
    assert [s for _, _, s in rows2[0]] == ["done", "done", "done"] and [s for _, _, s in rows2[1]][:3] == ["done", "done", "cur"]
    c = bench_tui.Canvas(150, 50)
    view.draw(c, bench_tui.SCREENS.index("BOAT"), "", "")
    text = "\n".join(c.lines())
    assert "USV_UI / RUN / FLOW" in text and "FLOW usv_ui ACTIVE" in text


def test_mission_tab_and_warnings():
    """MISSION tab: catalogue with the cursor, the highlighted mission's stages, HOW TO SELECT from the live state; WARNINGS self-clear."""
    import json
    m = bench_tui.Model("USV1", "usv")
    view = bench_tui.View(m, "USV1")
    m.hub_up = m.boat_up = True
    m.bridge = True
    body = ocs("R5", "AUTO_TASK", 1001, 1, task_state="S2")
    body["vehicles"]["USV1"]["task_name"] = "TEST_AUTO"
    body["mission"] = {"task": "TEST_AUTO", "auto": True, "started_run": 1001, "armed_run": None}
    m.on_hub("ocs/state", json.dumps({"body": body}).encode())
    m.on_hub("team/optiprime/v1/status/usv", json.dumps({"body": {"state": "AUTO_TASK", "task_name": "TEST_AUTO", "task_state": "S2", "armed": True,
                                                                  "health": {"bridge_up": True, "fcu_age_s": 0.05, "rc_ok": True}}}).encode())
    m.on_boat("local/usv/mm/state", json.dumps({"body": {"state": "AUTO_TASK", "ui_active": False, "profile": "usv_pool.yaml", "lab": False, "fcu": {}}}).encode())
    m.on_boat("presence/usv_ui", json.dumps({"node": "usv_ui", "up": True, "pid": 3, "fcu_link": "fake", "fcu_conn": "udp 14550"}).encode())
    # MAIN: the MISSION box follows the vehicle (RUNNING, stage chain with S2 highlighted), the VEHICLE box names the FCU and the profile
    c = bench_tui.Canvas(150, 50)
    view.draw(c, 0, "", "")
    text = "\n".join(c.lines())
    assert "RUNNING  run 1001  stage S2" in text and "FAKE plant udp 14550" in text and "usv_pool.yaml" in text
    row = next(ln for ln in c.lines() if ln.lstrip().startswith("\u2502 stages"))
    assert " S1 " in row and " S2 " in row and c.attrs[c.lines().index(row)][row.find(" S2 ")] == "now"
    assert m.warnings() == []                                                     # everything up: no warning
    assert "m0t1  (TEST_AUTO)" in text                                            # v0.1.19: the code next to the wire task
    # MISSION tab: cursor 0 = m4t0 (T4) while m0t1 runs -> HOW TO SELECT says abort first, then mission m4t0, start_task
    view.cursor = 0
    c = bench_tui.Canvas(150, 50)
    view.draw(c, bench_tui.SCREENS.index("MISSION"), "", "")
    text = "\n".join(c.lines())
    assert "[MISSION]" in text and "Dynamic Incident Response" in text and "HOW TO SELECT m4t0" in text
    assert "a task runs (m0t1): drop it first" in text and "mission m4t0" in text and "inject assistance" in text
    assert "* running" in text and "ALL MISSIONS AT A GLANCE" in text and "WAIT_START -> SEARCH_ENTRY" in text
    assert "m1t2   T1" in text and "m0t3   TEST_" in text                        # the whole catalogue, codes first
    assert "needs" in text and "R5 ok" in text and "AUTO_HOLD NO" in text
    steps, needs = playbook.how_to_select("m1t2", body, "USV1", False, None)     # v0.1.66: built
    assert "not built yet" not in " ".join(x[1] for x in steps) and steps[0][1] == "a task runs (m0t1): drop it first"   # as any built one
    steps, _ = playbook.how_to_select("m0t0", {"r": "R2", "vehicles": {"USV1": {"state": "MANUAL"}}}, "USV1", True)
    cmds = [cmd for _, _, cmd, _ in steps]
    assert "mission m0t0" in cmds and "declare" in cmds and "enable_auto" in cmds and "step" in cmds and any("handover" in x for x in cmds)
    # the code resolver (operator word -> wire): codes, suffix, old names, case; code_of the other way
    assert playbook.resolve("m4t0") == ("m4t0", "T4", "core", False) and playbook.resolve("M1T2sim") == ("m1t2", "T1", "advanced", True)
    assert playbook.resolve("TEST_AUTO") == ("m0t1", "TEST_AUTO", None, False) and playbook.resolve("T1") == ("m1t1", "T1", "core", False)
    assert playbook.resolve("goto")[1] == "GOTO" and playbook.resolve("m9t9") is None and playbook.resolve("") is None
    assert playbook.code_of("T4") == "m4t0" and playbook.code_of("T1", "TIER_ADVANCED") == "m1t2" and playbook.code_of("TEST_AUTO", None, True) == "m0t1sim"
    assert playbook.mission_for("T4").code == "m4t0" and playbook.mission_for("m1t2").task == "T1" and playbook.mission_for("bogus") is None
    # the playbook predicts the sim guard from the vehicle's fcu_link
    st_real = {"state": "AUTO_HOLD", "armed": True, "fcu_link": "real"}
    ok_state = {"r": "R5", "run_id": 7, "declaration_seq": 1, "vehicles": {"USV1": {"state": "AUTO_HOLD"}}}
    assert not playbook.check("START_TASK", ok_state, "USV1", task="m0t1sim", status=st_real).ok
    assert playbook.check("START_TASK", ok_state, "USV1", task="m0t1sim", status=dict(st_real, fcu_link="fake")).ok
    assert playbook.check("START_TASK", ok_state, "USV1", task="m0t1", status=st_real).ok
    assert playbook.check("START_TASK", ok_state, "USV1", task="m1t2", status=st_real).ok     # v0.1.66: m1t2 built (the OCS checks the UAV)
    # warnings: the bridge drops, the status goes stale, a REJECTED ack arrives -> three lines with ages; they clear by themselves
    m.on_boat("$SYS/broker/connection/ocs/state", b"0")
    m.status_t -= 5
    m.on_hub("team/optiprime/v1/cmd/usv/run", json.dumps({"msg_id": "m9", "body": {"action": "RESUME", "args": {}}}).encode())
    m.on_hub("team/optiprime/v1/ack/usv", json.dumps({"body": {"ack_of": "m9", "result": "REJECTED", "state": "AUTO_TASK", "reason": "RESUME only from AUTO_SAFE_HOLD"}}).encode())
    ws = m.warnings()
    texts = [t for _, t, _ in ws]
    assert any("BRIDGE DOWN" in t for t in texts) and any("stale" in t for t in texts) and any("REJECTED" in t for t in texts)
    c = bench_tui.Canvas(150, 50)
    view.draw(c, 0, "", "")
    assert "3 WARNINGS" in c.lines()[0] and "BRIDGE DOWN" in "\n".join(c.lines())
    m.on_boat("$SYS/broker/connection/ocs/state", b"1")
    m.status_t = __import__("time").time()
    assert not any("BRIDGE DOWN" in t or "stale" in t for _, t, _ in m.warnings())
