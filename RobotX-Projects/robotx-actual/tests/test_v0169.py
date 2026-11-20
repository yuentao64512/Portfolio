"""v0.1.69 (WP25 = docs/OVERNIGHT_2026-11-09_GUI.md N1): the dry / wet sessions - OCS-side, nothing moves."""
from __future__ import annotations

import json
import os

import pytest

from optiprimex.ocs import sessions as SS

from .conftest import ROOT


def _digest(boat=(0.0, 0.0), ents=(), uav=None, points=None, co=(1.2810202, 103.8550097)):
    d = {"boat": {"x_n": boat[0], "y_e": boat[1], "hdg_deg": 0.0}, "co": {"lat": co[0], "lon": co[1]}, "entities": list(ents),
         "gates": [], "markers": {"entry": None, "exit": None}}
    if uav is not None:
        d["uav"] = uav
    if points is not None:
        d["points"] = points
    return d


def test_catalogue_and_options():
    assert set(SS.CATALOGUE) == {"buoys", "dockface", "windows", "uavmap", "cannon", "collect"}
    assert [c.name for c in SS.CATALOGUE.values() if c.course_ok] == ["collect"]
    assert SS.parse_opts("buoys", {"model": "cvt-v2-l"}) == ({"model": "cvt-v2-l"}, "")
    assert "one of students, cvt-v2-l" in SS.parse_opts("buoys", {"model": "yolo"})[1]
    assert "no option --tape" in SS.parse_opts("buoys", {"tape": "3"})[1]
    assert SS.parse_opts("dockface", {})[0] == {"tape": "3,5,8"}
    assert "0.5 .. 40" in SS.parse_opts("dockface", {"tape": "3,99"})[1]
    assert "--confirm aim" in SS.parse_opts("cannon", {"aim10": "yes"})[1]
    assert SS.parse_opts("cannon", {"aim10": "yes", "confirm": "aim"})[0]["aim10"] == "yes"
    assert "letters, digits" in SS.parse_opts("collect", {"label": "a b"})[1]
    assert "unknown session" in SS.parse_opts("nope", {})[1]


def test_plans_real_and_sim():
    env = {"AGX_SSH": "agxorin"}
    real = SS.plan("buoys", {"model": "cvt-v2-l"}, env, False, "py")
    assert real[0].argv[:4] == ["ssh", "-o", "BatchMode=yes", "-o"] and "agxorin" in real[0].argv
    assert "cv/buoys_multi/run.sh --headless --model cv/models/cvt-v2-l" in real[0].argv[-1] and "tmux new-window -d -t agx -n sess" in real[0].argv[-1]
    assert "tmux kill-window -t agx:sess" in real[0].stop_argv[-1] and "pkill" not in json.dumps([s.__dict__ for s in real])
    assert "agx_cam.sh start --source dir" in real[1].argv[-1]
    sim = SS.plan("buoys", {}, env, True, "py")
    assert sim[0].argv is None and "fake CV stands in" in sim[0].note
    u = SS.plan("uavmap", {"path": "udp"}, env, True, "py")
    assert u[0].argv[1:3] == ["-m", "optiprimex.sim.fake_uav"] and "--udp" in u[0].argv and "--offset-km" in u[0].argv and u[0].long
    assert SS.plan("uavmap", {"path": "broker"}, env, True, "py", hub="h:1")[0].argv[-2:] == ["--broker", "h:1"]
    assert SS.plan("uavmap", {}, env, False, "py")[0].argv is None                   # real: their detector publishes
    c = SS.plan("collect", {"label": "day1"}, env, False, "py")
    assert "agx_record.sh start day1" in c[0].argv[-1] and "agx_record.sh stop" in c[0].stop_argv[-1]
    can = SS.plan("cannon", {"aim10": "yes", "confirm": "aim"}, env, False, "py")
    assert any("cv/cannon/run.sh" in (s.argv or [""])[-1] for s in can) and can[-1].note == "aim10"


def test_metrics():
    ents = [{"id": 1, "kind": "buoy", "colour": "red", "x_n": 3.0, "y_e": 0.0, "n_obs": 9, "source": "usv"},
            {"id": 2, "kind": "buoy", "colour": "green", "x_n": 7.0, "y_e": 0.0, "n_obs": 4, "source": "usv"},
            {"id": 3, "kind": "buoy", "colour": "red", "x_n": 20.0, "y_e": 0.0, "n_obs": 2, "source": "usv"},
            {"id": 9, "kind": "buoy", "colour": "blue", "x_n": 16000.0, "y_e": 0.0, "n_obs": 3, "source": "uav"}]
    h = {}
    m = SS.metrics("buoys", {"digest": _digest(ents=ents)}, h)
    assert m["n_buoys"] == 3 and m["per_band"] == {"0-5": 1, "5-10": 1, "10-15": 0, "15-25": 1} and m["colours"] == {"red": 2, "green": 1}
    far = {"items": [{"lat": 1.2810202 + 16000 / 111320.0, "lon": 103.8550097, "class": "buoy"}], "scan": {"state": "scanning"}, "frame_id": "f1"}
    m = SS.metrics("uavmap", {"digest": _digest(ents=ents, uav={"n_linked": 0, "n_uav_only": 1, "assoc_rate": 0.0}), "uav_msg": far,
                              "uav_msg_age": 0.4}, {})
    assert m["items"] == 1 and m["merged"] == 0 and 15900 < m["uav_items_from_course_m"] < 16100 and "16.0 km from the course" in m["caution"]
    faces = [{"id": 5, "kind": "dock", "colour": "white", "x_n": 5.3, "y_e": 0.0, "n_obs": 6}]
    m = SS.metrics("dockface", {"digest": _digest(ents=faces), "opts": {"tape": "3,5,8"}}, {})
    assert m["faces"][0]["tape_m"] == 5.0 and m["faces"][0]["err_m"] == 0.3
    m = SS.metrics("collect", {"digest": _digest(), "points": {"day1_01": [0, 0], "day1_02": [1, 1], "HOME": [0, 0]}, "opts": {"label": "day1"}}, {})
    assert m["points"] == ["day1_01", "day1_02"] and m["next_point"] == "day1_03"
    m = SS.metrics("cannon", {"digest": _digest(), "cannon": {"state": "ok", "n_rx": 40, "event": "AIM", "ready": True,
                                                               "target": {"range_m": 4.2}}}, {})
    assert m["packets"] == 40 and m["lock"] is True and m["target_range_m"] == 4.2 and m["pi_trace"][-1].endswith("AIM")
    mm = SS.mission_metrics("T1", "core", {"markers": {"entry": 4, "exit": None}, "gates": [{"passed": True}, {"passed": False}],
                                           "entities": ents})
    assert (mm["entry_found"], mm["gates_detected"], mm["gates_passed"], mm["exit_found"]) == (True, 2, 1, False)


def test_manager_start_stop_writes_the_result(tmp_path):
    pub = []
    snap = {"digest": _digest(ents=[{"id": 1, "kind": "buoy", "colour": "red", "x_n": 3.0, "y_e": 0.0, "n_obs": 9, "source": "usv"}])}
    m = SS.SessionManager(lambda: snap, pub.append, env={}, sim=lambda: True, dry_run=True, root=str(tmp_path))
    ok, why = m.start("buoys", {"model": "students"})
    assert ok and "simulator" in why and m.cur["state"] == "running"
    assert m.start("dockface", {})[0] is False                                     # one at a time
    m.tick()
    assert pub[-1]["current"]["metrics"]["n_buoys"] == 1
    ok, why = m.stop()
    assert ok and m.cur is None and m.last["result"].startswith("buoys: 1 now")
    md = open(m.last["files"]["md"]).read()
    assert "| model | n buoys" in md and "| students | 1 (1) | 1 |" in md
    assert json.loads(open(tmp_path / "sessions.jsonl").read().splitlines()[-1])["name"] == "buoys"
    assert m.command("result")[1].startswith("buoys:") and m.stop() == (False, "no session running")


def test_collect_track_and_aim(tmp_path):
    aims, recs = [], []
    snap = {"digest": _digest(points={}), "status": {"lat": 1.28, "lon": 103.85, "hdg_deg": 10, "sog_mps": 0.1, "state": "MANUAL"}, "points": {}}
    m = SS.SessionManager(lambda: snap, lambda b: None, env={}, sim=lambda: True, dry_run=True, root=str(tmp_path), aim=aims.append,
                          rec=lambda op, lab: recs.append((op, lab)) or {"ok": True})
    assert m.start("collect", {"label": "day1"})[0]
    m.tick(); m.tick()
    ok, _ = m.stop()
    assert ok and recs == [("start", "day1"), ("stop", "")] and m.last["files"]["track"].endswith("_collect_day1_track.csv")
    assert len(open(m.last["files"]["track"]).read().splitlines()) == 1 + 3          # the header + start's tick + two ticks
    assert m.start("cannon", {"aim10": "yes", "confirm": "aim"})[0]
    m.aim_tick()
    assert aims == [True]
    m.stop()
    assert aims[-1] is False                                                       # the aim off on stop


def test_authority_rules_and_the_hand_off():
    from optiprimex.ocs.cmd_authority import CmdAuthority
    from optiprimex.ocs.run_lifecycle import RunLifecycle

    class B:
        def publish(self, *a, **k): return "id"
    res = []
    for course, bench, name, want in ((True, True, "buoys", "refused"), (True, True, "collect", "accepted"), (False, False, "buoys", "refused"),
                                      (False, False, "collect", "accepted"), (False, True, "buoys", "accepted")):
        a = CmdAuthority(B(), RunLifecycle(), {"USV1": "usv"}, bench, lambda c=course: c, res.append)
        a.on_session = lambda op, args: (True, "ok")
        r = a.on_intent("ui/cmd/x", {"kind": "intent", "msg_id": "i", "vehicle_id": "USV1",
                                     "body": {"action": "SESSION", "args": {"op": "start", "name": name}}})
        assert r.result == want, (course, bench, name, r.reason)
    from .test_v0164 import make_app
    app, bus, judge, _ = make_app()
    sent = []
    bus.publish_raw = lambda topic, payload, qos=0, retain=False: sent.append((topic, json.loads(payload)))
    assert "no session host" in app.on_session("start", {"name": "buoys"})[1]
    app._on_session_state("ocs/session", json.dumps({"current": None, "last": None}).encode())
    assert app.on_session("start", {"name": "buoys", "opts": {"model": "x"}})[0] is False
    ok, text = app.on_session("start", {"name": "buoys", "opts": {"model": "cvt-v2-l"}})
    assert ok and "starting on the page server" in text
    assert sent[-1][0] == "ocs/session/cmd" and sent[-1][1]["name"] == "buoys" and sent[-1][1]["opts"] == {"model": "cvt-v2-l"}
    assert app.state_body()["session"]["current"] is None


def test_the_word_everywhere_and_no_word_lost():
    from optiprimex.tools import bench_tui, ocs_cli, playbook
    ns = ocs_cli.parse(["session", "start", "cannon", "--aim10", "--confirm", "aim"])
    assert ocs_cli.intent_spec(ns) == ("SESSION", None, {"op": "start", "name": "cannon", "opts": {"confirm": "aim", "aim10": "yes"}}, None)
    assert "session" in bench_tui.INTENT_WORDS and "SESSION" in playbook.PLAYBOOK
    v68 = {"declare", "enable_auto", "disable_auto", "start_task", "goto", "hold", "resume", "abort", "confirm_safe", "end_run", "inject", "reset",
           "step", "skip", "mission", "select_target", "save_point", "del_point", "clear_map", "home", "tune", "fcu_mode", "arm", "disarm",
           "rc6_policy", "fence", "ready", "manual", "next", "state", "report", "stop_reset"}
    assert v68 <= bench_tui.INTENT_WORDS                                          # every v0.1.68 word still there


def test_the_page_and_the_map():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('id="sess-card"', 'id="ssel"', 'id="sdrop"', 'id="camfold"', 'id="mf-ov"', 'id="ms-metrics"', 'id="ms-det"',
                   'send("session stop")', '"mf-ov"]', "sessRender();mfCam()"):
        assert needle in page, needle
    mp = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert 'id="b_fituav"' in mp and "e.source === 'uav'" in mp


def test_fake_uav_udp_and_offset_flags():
    import subprocess, sys
    r = subprocess.run([sys.executable, "-m", "optiprimex.sim.fake_uav", "--help"], capture_output=True, text=True, cwd=str(ROOT))
    assert "--udp" in r.stdout and "--offset-km" in r.stdout


def test_the_launcher_uav_buoys_field():
    from optiprimex.tools import launcher as L
    P = L.load_presets()
    pre = {"clocks": {}, "nic": None, "refuse": []}
    for path, want in (("udp", "UAV_BUOY_PATH=relay"), ("broker", "UAV_BUOY_PATH=direct"), (None, "UAV_BUOY_PATH=direct")):
        c = dict(P["lab"], uav="on", preset="lab", **({"uav_buoys": path} if path else {}))
        assert L.validate(c) == []
        nx = [s for s in L.build_plan(c, stamp="x", net_env={}, pre=pre)["steps"] if s["host"] == "nx"][0]["cmd"]
        assert want in nx
    assert "uav_buoys: one of broker, udp" in L.validate(dict(P["lab"], uav="on", uav_buoys="mqtt"))
    off = [s for s in L.build_plan(dict(P["lab"], preset="lab"), stamp="x", net_env={}, pre=pre)["steps"] if s["host"] == "nx"][0]["cmd"]
    assert "UAV_BUOY_PATH" not in off
