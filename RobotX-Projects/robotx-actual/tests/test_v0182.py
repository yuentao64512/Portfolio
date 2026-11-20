"""v0.1.82 (Dmitrii at the venue, 2026-11-11): (1) the USV's heartbeat on a button - ▶ HEARTBEAT USV → PILOT READY → DECLARE (the course
profile; PILOT READY an OCS gate), (2) OUR observer from the OCS page and the launcher (scripts/agx_observer.sh; agx_cam.sh's auto knew
no multi_buoy_observer = no stream), (3) the course dropdown + the RoboCommand network checker (WP39, full), (4) the scout's turns capped
at 10 deg/s. QUICK: no broker."""
from __future__ import annotations

import math
import subprocess
import tempfile

import yaml

from optiprimex.companion.config import Config
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.ocs.judge_client import NullJudgeClient
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import launcher as L
from optiprimex.tools import ocs_cli, playbook
from optiprimex.tools import system_actions as SA

from .conftest import ROOT
from .test_ocs_units import FakeBus

COURSE = {"course_id": "ALPHA", "boundary": [[1.0, 103.0], [1.0, 103.001], [1.001, 103.001], [1.001, 103.0]], "uav_transit_area": [],
          "pinger_hz": 30000}


def course_app():
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "course" / "ocs_course.yaml").read_text())
    cfg.extra["ocs"]["usv_heartbeat"] = "button"           # v0.1.90.5: the course profile is auto now; the button flow stays tested
    app = OcsApp(cfg, bus=FakeBus(), judge=NullJudgeClient())
    app.authority.publish_result = lambda r: None
    app.lifecycle.on_link(True)
    app.lifecycle.on_course(COURSE)
    app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL"}})
    return app


# ---- (1) the USV heartbeat on the button -----------------------------------------------------------------------------------------------
def test_the_course_profile_keeps_the_usv_silent_until_the_button():
    assert yaml.safe_load((ROOT / "profiles" / "course" / "ocs_course.yaml").read_text())["ocs"]["usv_heartbeat"] == "auto"   # v0.1.90.5: ready by itself (the button flow below stays tested)
    app = course_app()
    assert app.usv_flow == "button" and app.hb_vehicles() == [] and app.usv_flow_state()["heartbeat"] is False
    assert "▶ HEARTBEAT USV" in app.declare({})                                # declare refuses: no heartbeat
    ok, why = app.on_usv_ready(True)
    assert not ok and "HEARTBEAT USV" in why                                   # ready needs the heartbeat first
    assert app.on_reporting(True, "usv") is None and app.hb_vehicles() == ["USV1"] and app.heartbeats.enabled
    assert "PILOT READY not pressed" in app.declare({})
    assert app.on_usv_ready(True)[0] and app.usv_flow_state()["ready"] is True
    assert app.declare({}) is None                                             # PREPARE went out
    app.reset_run()
    assert app.usv_ready is False and app.report_usv is True and app.heartbeats.enabled   # ready per attempt; the heartbeat stays (handbook)
    app.on_reporting(False, "usv")
    assert app.hb_vehicles() == []


def test_the_bench_and_pool_keep_the_v0159_rule():
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "pool" / "ocs_pool.yaml").read_text())
    app = OcsApp(cfg, bus=FakeBus(), judge=NullJudgeClient())
    assert app.usv_flow == "auto" and app.hb_vehicles() == ["USV1"] and app.usv_ready


def test_the_words_and_the_strip():
    p = ocs_cli.build_parser()
    assert ocs_cli.intent_spec(p.parse_args(["report", "on", "usv"]))[:3] == ("REPORT", None, {"on": True, "vehicle": "usv"})
    assert ocs_cli.intent_spec(p.parse_args(["usv", "ready", "on"]))[:3] == ("USV_READY", None, {"on": True})
    from optiprimex.ocs import cmd_authority as CA
    assert "USV_READY" in CA.GUARDS and "USV_READY" in playbook.PLAYBOOK
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('send("report "+(on?"off":"on")+" usv")', 'send("usv ready "+(g.ready?"off":"on"))', 'send("declare")', "▶ HEARTBEAT USV",
                   "PILOT READY?", "usvStripsDraw()", 'id="rv-obs"'):
        assert needle in page, needle


# ---- (2) the observer ------------------------------------------------------------------------------------------------------------------
def test_the_observer_from_the_page_and_the_stream_fix():
    a = next(x for x in SA.ACTIONS if x.id == "observer")
    cmd = a.build({"do": "start", "detector": "model-a"}, {"AGX_SSH": "agx@10.1.0.99"})
    assert "agx@10.1.0.99" in " ".join(cmd) and "scripts/agx_observer.sh start --detector model-a" in " ".join(cmd)
    assert "agx_observer.sh stop" in " ".join(a.build({"do": "stop"}, {}))
    assert "a task runs" in a.opt_guard({"do": "stop"}, {"status": {"state": "AUTO_TASK"}}) and a.opt_guard({"do": "start"}, {"status": {"state": "AUTO_TASK"}}) == ""
    sh = (ROOT / "scripts" / "agx_observer.sh").read_text()
    assert "buoys_multi/run.sh --headless" in sh and subprocess.run(["bash", "-n", str(ROOT / "scripts" / "agx_observer.sh")]).returncode == 0
    assert "multi_buoy_observer" in (ROOT / "scripts" / "agx_cam.sh").read_text()
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'id="c-obs"' in page and 'id="c-obsdet"' in page and 'x.id==="observer"' in page


# ---- (3) WP39: the course dropdown and the checker ---------------------------------------------------------------------------------------
def test_the_dropdown_the_env_keys_and_run_ocs():
    assert L.FIELDS["judge"] == ("venue", "auto", "alpha", "bravo", "charlie", "delta", "course", "fake", "rctest")   # v0.1.94: + auto   # v0.1.86: venue first, rctest last
    assert any("needs the real network" in e for e in L.validate({**L.load_presets()["sim"], "judge": "bravo"}))
    sh = (ROOT / "scripts" / "run_ocs.sh").read_text()
    assert 'hv="RC_${RC_COURSE}_HOST"' in sh and "demo) RC_COURSE=DEMO; JUDGE=venue" in sh and "RC_COURSE=${RC_COURSE:-AUTO}" in sh
    assert subprocess.run(["bash", "-n", str(ROOT / "scripts" / "run_ocs.sh")]).returncode == 0
    ne = (ROOT / "network.lan.env").read_text() if (ROOT / "network.lan.env").exists() else ""
    assert not ne or ("RC_ALPHA_HOST=192.168.65.2" in ne and "RC_DELTA_HOST=192.168.68.2" in ne)   # the addresses live in the env file
    page = (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    assert 'venue:"DEMO (venue)"' in page and 'alpha:"ALPHA"' in page and 'id="crstab"' in page and 'fetch("/course_check.json")' in page


def test_the_checker_lights_only_the_plugged_course_and_peeks_without_the_ocs():
    ne = {"RC_ALPHA_HOST": "192.168.65.2", "RC_BRAVO_HOST": "192.168.66.2", "RC_CHARLIE_HOST": "192.168.67.2", "RC_DELTA_HOST": "192.168.68.2"}
    nic = {"iface": "en7", "addr": "192.168.66.40", "gateway": "192.168.66.2", "dhcp": True}
    probed, peeked = [], []
    run = lambda argv: (1, "")                                                   # pgrep: no OCS
    r = L.course_check(ne, nic=nic, run=run, up=lambda h, p: probed.append(h) or True,
                       peeker=lambda h, p: peeked.append(h) or {"course_id": "BRAVO", "boundary": [[0, 0]] * 4, "pinger_hz": 25000})
    rows = {x["course"]: x for x in r["rows"]}
    assert r["on"] == "BRAVO" and rows["BRAVO"]["on_network"] is True and rows["BRAVO"]["broker"] is True
    assert rows["BRAVO"]["rx_course"] == {"course_id": "BRAVO", "corners": 4, "pinger_hz": 25000, "error": None}
    assert rows["ALPHA"]["on_network"] is False and rows["ALPHA"]["note"] == "not on this network" and "192.168.65.2" not in probed
    peeked.clear()
    r2 = L.course_check(ne, nic=nic, run=lambda argv: (0, "123"), up=lambda h, p: True, peeker=lambda h, p: peeked.append(h))
    assert r2["ocs_running"] and not peeked and "the OCS holds the connection" in {x["course"]: x for x in r2["rows"]}["BRAVO"]["note"]


def test_the_plan_notes_a_wrong_cable_and_the_ocs_cautions(monkeypatch):
    cfg = {**L.load_presets()["event"], "judge": "alpha"}
    ne = {"RC_ALPHA_HOST": "192.168.65.2", "RC_ALPHA_PORT": "1883", "OCS_HOST": "192.168.1.13", "BOAT_HOST": "192.168.1.218",
          "AGX_HOST": "192.168.1.99", "ORIN_SSH": "u@192.168.1.218", "AGX_SSH": "a@192.168.1.99"}
    plan = L.build_plan(cfg, stamp="x", net_env=ne, pre={"clocks": {}, "nic": {"iface": "en7", "addr": "192.168.66.40", "dhcp": True}, "refuse": []})
    assert any("selected ALPHA, the cable is on BRAVO" in n for n in plan["notes"])
    assert any("--judge alpha" in st["cmd"] for st in plan["steps"])
    monkeypatch.setenv("RC_COURSE", "ALPHA")
    app = course_app()
    got = app.course_checks({"course_id": "BRAVO"}, local_addr="192.168.66.40")
    assert got == ["course mismatch: selected ALPHA, RxCourse says BRAVO", "OCS address not in the course subnet: 192.168.66.40, ALPHA is 192.168.65.0/24"]
    assert app.course_checks({"course_id": "ALPHA"}, local_addr="192.168.65.40") == []
    monkeypatch.setenv("RC_COURSE", "DEMO")
    assert app.course_checks({"course_id": "x"}, local_addr="10.0.0.1") == []


# ---- (4) the scout's yaw rate --------------------------------------------------------------------------------------------------------------
def test_scout_turns_at_most_10_dps_and_still_finishes():
    world = str(ROOT / "profiles" / "bench" / "world_scout.yaml")
    s = FastSil(world, "SCOUT", {"polygon": [tuple(p) for p in W.load(world)["bounds"]], "log_dir": tempfile.mkdtemp()})
    assert s.task.p["max_yaw_dps"] == 10.0
    r = s.run(max_s=900)
    rates = sorted(abs((b[3] - a[3] + 180) % 360 - 180) / (b[0] - a[0]) for a, b in zip(s.track, s.track[1:]) if b[0] > a[0])
    assert r["state"] == "DONE" and rates[int(0.95 * len(rates))] <= 11.0                 # was ~38 deg/s (the plant's 45 deg/s pivots)
    buoys = [l for l in s.task.lines if l["ev"] == "buoy"]
    assert len(buoys) == 10 and sum(b["visited"] for b in buoys) >= 9
    for f in ("pool/usv_pool.yaml", "course/usv_course.yaml", "bench/usv_bench.yaml"):
        assert yaml.safe_load((ROOT / "profiles" / f).read_text())["scout"]["max_yaw_dps"] == 10.0, f


# ---- v0.1.83: logging on by default (Dmitrii: "video, position, IMU, heading, everything") ------------------------------------------------
def test_logging_is_on_by_default():
    assert 'UI_ARGS="$UI_ARGS --log run"' in (ROOT / "scripts" / "run_usv.sh").read_text()               # usv_ui's FCU tlog / csv / events
    sh = (ROOT / "scripts" / "agx_observer.sh").read_text()
    assert "--record $HOME/Desktop/dm3/recordings --label obs" in sh and "--no-record) REC=0" in sh       # the ZED video + its IMU
    pre = L.load_presets()
    assert pre["event"]["log"] is True and pre["lab"]["log"] is True and pre["custom"]["log"] is True      # LOG=1: every MQTT message
    plan = L.build_plan(dict(pre["event"]), stamp="x", net_env={"ORIN_SSH": "n", "AGX_SSH": "a", "OCS_HOST": "1", "BOAT_HOST": "2", "AGX_HOST": "3"})
    assert plan["steps"][-1]["cmd"].endswith("agx_observer.sh start --detector default'")


def test_the_observer_button_reads_the_fresh_frames_not_the_beat():
    """v0.1.83.1 (the venue): cv_publish beats every second WITHOUT an observer - the button showed ■ and sent `stop` (jobs 2, 3)."""
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "return age!=null&&age<3&&p.ok===true}" in page and "/observer|multi|buoy/i.test" not in page
