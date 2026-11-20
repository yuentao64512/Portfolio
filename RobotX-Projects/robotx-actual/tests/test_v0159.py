"""v0.1.59 - the run-start sequence and the RoboCommand compliance fixes (docs/RUNSTART_AND_RC_BRIEF_2026-11-08.md, the verdict table
docs/ROBOCOMMAND_COMPLIANCE_2026-11-08.md). A1 the counters never reset and persist; A2 EKF_BAD soft with hysteresis + the FAULT
heartbeat grace; A3 the monitors' subscriptions; A4 flight_phase only for a UAV; A5 RC_HOST auto from the course NIC; A6 the clock
check; B1 heartbeats on the judge wire from the first status; B2 the ladder in the handbook's order + the attempt label; B3 the R4
words and the stall as a CAUTION; B4 the event preset's --auto."""
from __future__ import annotations

import json
import re
import types

from optiprimex.companion.config import Config
from optiprimex.companion.health import HealthMonitor
from optiprimex.companion.state import FcuState
from optiprimex.ocs import rc_wire
from optiprimex.ocs.heartbeat_publisher import HeartbeatPublisher
from optiprimex.ocs.judge_client import FakeJudgeClient, ProtobufJudgeClient, seq_file_path
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.tools import bench_tui as T
from optiprimex.tools import launcher as L
from optiprimex.tools import ocs_web as W
from optiprimex.tools import playbook
from optiprimex.tools.rc_tui import monitor_subscriptions

from .conftest import ROOT

NET = {"ORIN_SSH": "nx-alias", "AGX_SSH": "agx-alias"}


class Clock:
    def __init__(self, t=100.0):
        self.t = t

    def __call__(self):
        return self.t


def _client(tmp_path, cls=FakeJudgeClient, **kw):
    kw.setdefault("rate_cap", 100.0)                                       # the 5 msg/s cap is tested elsewhere
    return cls("127.0.0.1", 1, "SGIT", seq_file=str(tmp_path / "rc_seq.json"), **kw)     # never started: publish only queues


# ---- A1: the counters -----------------------------------------------------------------------------------------------------------
def test_heartbeat_seq_keeps_increasing_across_a_runstart_and_a_restart(tmp_path):
    j = _client(tmp_path)
    seqs = [j.publish_report("USV1", {"kind": "heartbeat", "state": "STATE_AUTO"}) for _ in range(3)]
    j.new_run()                                                            # what ocs_app calls on a verified RunStart
    seqs += [j.publish_report("USV1", {"kind": "heartbeat", "state": "STATE_AUTO"}) for _ in range(2)]
    assert seqs == [1, 2, 3, 4, 5]                                         # never reset inside the connection (3.4.8)
    assert j.publish_request({"kind": "declaration"}) == 1
    saved = json.loads((tmp_path / "rc_seq.json").read_text())["SGIT"]
    assert saved["request_seq"] == 1 and saved["report_seq"] == {"USV1": 5}
    j2 = _client(tmp_path, cls=ProtobufJudgeClient)                        # an ocs_app restart: the sequence continues
    assert j2.publish_report("USV1", {"kind": "heartbeat", "state": "STATE_MANUAL"}) == 6
    assert j2.publish_request({"kind": "run_declaration", "vehicle_ids": ["USV1"], "tiers": {}}) == 2
    other = FakeJudgeClient("127.0.0.1", 1, "OTHER", seq_file=str(tmp_path / "rc_seq.json"))   # per team
    assert other.publish_report("USV1", {"kind": "heartbeat"}) == 1
    assert set(json.loads((tmp_path / "rc_seq.json").read_text())) == {"SGIT", "OTHER"}


def test_the_runstart_path_keeps_the_counters_and_the_suite_never_persists(tmp_path, monkeypatch):
    app = OcsApp(Config(), judge=_client(tmp_path))
    app.judge.publish_report("USV1", {"kind": "heartbeat"})
    app.lifecycle.on_run_start = lambda cmd: True
    app.on_rc_command({"kind": "run_start", "command_seq": 1, "run_id": 7, "declaration_seq": 1})
    assert app.judge.publish_report("USV1", {"kind": "heartbeat"}) == 2
    assert seq_file_path() is None                                         # conftest: OPTIPRIMEX_RC_SEQ=off
    monkeypatch.setenv("OPTIPRIMEX_RC_SEQ", str(tmp_path / "x.json"))
    assert seq_file_path() == str(tmp_path / "x.json")
    monkeypatch.delenv("OPTIPRIMEX_RC_SEQ")
    assert seq_file_path().endswith("logs/ocs/rc_seq.json")
    (tmp_path / "bad.json").write_text("{not json")
    assert FakeJudgeClient("127.0.0.1", 1, "SGIT", seq_file=str(tmp_path / "bad.json")).publish_report("USV1", {"kind": "heartbeat"}) == 1


# ---- A2: EKF_BAD soft, then hard; the FAULT grace ----------------------------------------------------------------------------------
def _fcu(ekf_ok):
    f = FcuState(seen_heartbeat=True, rc_chancount=16, rc_ok=True, ekf_pos_ok=ekf_ok, armed=True)
    f.ages = {"HEARTBEAT": 0.1, "GLOBAL_POSITION_INT": 0.05, "ATTITUDE": 0.05, "RC_CHANNELS": 0.2, "EKF_STATUS_REPORT": 0.3}
    return f


def test_ekf_bad_is_soft_for_three_seconds_then_hard():
    cfg = Config()
    assert cfg.guards.ekf_bad_hard_s == 3.0
    hm = HealthMonitor(cfg)
    hm.update(_fcu(False), True, float("inf"), 10.0)
    assert hm.soft_faults() == ["EKF_BAD"] and hm.hard_faults() == []      # V5 AUTO_SAFE_HOLD, STATE_AUTO on the heartbeat
    hm.update(_fcu(False), True, float("inf"), 12.9)
    assert hm.hard_faults() == []
    hm.update(_fcu(False), True, float("inf"), 13.0)
    assert hm.hard_faults() == ["EKF_BAD"] and "EKF_BAD" not in hm.soft_faults()   # continuous 3 s -> V6
    hm.update(_fcu(True), True, float("inf"), 13.5)                        # recovered: the timer restarts
    hm.update(_fcu(False), True, float("inf"), 14.0)
    assert hm.hard_faults() == [] and hm.soft_faults() == ["EKF_BAD"]
    cfg0 = Config()
    cfg0.guards.ekf_bad_hard_s = 0.0                                       # 0 = the old rule: hard at once
    hm0 = HealthMonitor(cfg0)
    hm0.update(_fcu(False), True, float("inf"), 1.0)
    assert hm0.hard_faults() == ["EKF_BAD"]
    lab = Config()
    lab.guards.ignore_faults = ["EKF_BAD"]                                 # the lab profile: neither soft nor hard
    hl = HealthMonitor(lab)
    hl.update(_fcu(False), True, float("inf"), 1.0)
    hl.update(_fcu(False), True, float("inf"), 9.0)
    assert hl.hard_faults() == [] and hl.soft_faults() == [] and "EKF_BAD" in hl.health.faults


def test_a_fault_keeps_the_last_state_for_ten_seconds_then_stops():
    clk = Clock()
    sent = []
    j = types.SimpleNamespace(publish_report=lambda v, b: sent.append((round(clk.t, 1), b["state"])) or len(sent))
    hb = HeartbeatPublisher(j, ["USV1"], hz=2.0, stale_stop_s=10.0, clock=clk)
    hb.enabled = True

    def run(state, rs, secs):
        end = clk.t + secs
        while clk.t < end:
            hb.update("USV1", {"state": state, "robot_state": rs})
            hb.tick()
            clk.t += 0.5
    run("AUTO_TASK", "STATE_AUTO", 2)
    run("FAULT", "STATE_UNKNOWN", 12)                                      # V6 for 12 s, status still fresh
    states = [s for _t, s in sent]
    assert states[:4] == ["STATE_AUTO"] * 4
    assert set(states[4:]) == {"STATE_AUTO"}                               # the last known state, never MANUAL, never invented
    t_fault = 102.0
    assert max(t for t, _s in sent) <= t_fault + 10.0 and hb.stopped_fault["USV1"]   # then the heartbeat stops
    n = len(sent)
    run("KILLED", "STATE_KILLED", 1)                                       # KILLED / MANUAL are reported at once
    assert sent[n][1] == "STATE_KILLED" and not hb.stopped_fault["USV1"]
    assert HeartbeatPublisher.to_heartbeat({"state": "FAULT", "robot_state": "STATE_UNKNOWN"})["state"] == "STATE_MANUAL"   # no history
    v = hb.view()["USV1"]
    assert v["n"] == len(sent) and v["state"] == "STATE_KILLED" and v["seq"] == len(sent) and v["age_s"] is not None


# ---- A3 / A4 --------------------------------------------------------------------------------------------------------------------------
def test_the_monitors_subscribe_our_team_only():
    subs = [t for t, _q in monitor_subscriptions("SGIT")]
    assert subs == ["rc/#", "robocommand/robotx/course", "robocommand/robotx/SGIT/request", "robocommand/robotx/SGIT/command",
                    "robocommand/robotx/SGIT/+/report"]
    for f in ("ocs_web.py", "judge_web.py", "rc_tui.py"):
        src = (ROOT / "optiprimex" / "tools" / f).read_text()
        assert not re.search(r"subscribe\(.*robocommand/robotx/#", src), f
        assert "monitor_subscriptions(" in src, f


def test_flight_phase_only_for_a_uav():
    REP = rc_wire.REP
    m = REP.RxReport()
    m.ParseFromString(rc_wire.encode_report("SGIT", "USV1", 1, {"kind": "heartbeat", "state": "STATE_AUTO", "vehicle_type": "USV",
                                                               "flight_phase": "FLIGHT_PHASE_UNKNOWN"}))
    assert not m.heartbeat.HasField("flight_phase")
    m = REP.RxReport()
    m.ParseFromString(rc_wire.encode_report("SGIT", "UAV1", 1, {"kind": "heartbeat", "state": "STATE_AUTO", "vehicle_type": "UAV",
                                                               "flight_phase": "FLIGHT_PHASE_UNKNOWN"}))
    assert m.heartbeat.HasField("flight_phase")


# ---- A5 / A6: the launcher -------------------------------------------------------------------------------------------------------
PKT = "op = BOOTREPLY\nyiaddr = 192.168.66.40\nlease_time (uint32): 0x15180\nrouter (ip_mult): {192.168.66.2}\n"


def _run(table):
    return lambda argv, timeout=6.0: table.get(tuple(argv), (1, ""))


def test_rc_host_auto_is_the_course_nics_dhcp_gateway():
    t = {("ifconfig", "-l"): (0, "lo0 en0 en7"), ("ipconfig", "getifaddr", "en0"): (0, "192.168.1.13\n"),
         ("ipconfig", "getifaddr", "en7"): (0, "192.168.66.40\n"), ("ipconfig", "getpacket", "en7"): (0, PKT)}
    nic = L.course_nic(_run(t))
    assert nic == {"iface": "en7", "addr": "192.168.66.40", "gateway": "192.168.66.2", "dhcp": True}
    assert L.course_nic(_run({("ifconfig", "-l"): (0, "en0"), ("ipconfig", "getifaddr", "en0"): (0, "192.168.1.13")})) == {}
    static = {**t, ("ipconfig", "getpacket", "en7"): (1, "")}
    assert L.course_nic(_run(static))["dhcp"] is False
    cfg = dict(L.load_presets()["event"], judge="course")
    assert L.resolve_rc_host(cfg, {}, nic) == ("192.168.66.2", None)
    assert L.resolve_rc_host(cfg, {"RC_HOST": "auto"}, nic) == ("192.168.66.2", None)
    assert L.resolve_rc_host(cfg, {"RC_HOST": "192.168.65.2"}, None) == ("192.168.65.2", None)          # the manual override
    assert "no DHCP lease" in L.resolve_rc_host(cfg, {}, L.course_nic(_run(static)))[1]
    assert "no interface" in L.resolve_rc_host(cfg, {}, {})[1]
    p = L.build_plan(cfg, stamp="S", net_env=NET, pre={"clocks": {}, "nic": nic, "refuse": []})
    ocs = next(s["cmd"] for s in p["steps"] if "run_ocs.sh" in s["cmd"])
    assert "RC_HOST=192.168.66.2" in ocs and "--judge course" in ocs and p["refuse"] == []
    assert p["notes"][0].startswith("course NIC: 192.168.66.40 (en7) · gateway 192.168.66.2 · DHCP · RoboCommand 192.168.66.2 (auto)")
    p = L.build_plan(cfg, stamp="S", net_env=NET, pre={"clocks": {}, "nic": {}, "refuse": []})
    assert p["refuse"] and "RC_HOST=" not in " ".join(s["cmd"] for s in p["steps"])                     # refused: no lease
    assert L.validate(dict(cfg, net="manual", ocs_host="a", boat_host="b", nx_ssh="c", agx_ssh="d")) == []  # rc_host empty = auto


def test_the_clock_check_refuses_above_five_seconds(monkeypatch):
    monkeypatch.delenv("OPTIPRIMEX_PREFLIGHT", raising=False)              # the suite's off switch (conftest) - the fakes answer here
    clk = Clock(1000.0)
    t = {tuple(L.SSH.split() + ["nx-alias", "date +%s"]): (0, "1007\n"), tuple(L.SSH.split() + ["agx-alias", "date +%s"]): (0, "1001\n")}
    assert round(L.clock_offset("nx-alias", run=_run(t), now=clk)) == 7
    assert L.clock_offset("moon", run=_run(t), now=clk) is None
    cfg = dict(L.load_presets()["lab"])
    pre = L.preflight(cfg, {"nx": "nx-alias", "agx": "agx-alias"}, run=_run(t), now=clk)
    assert pre["clocks"]["nx"] == {"host": "nx-alias", "offset_s": 7.0, "ok": False} and pre["clocks"]["agx"]["ok"] is True
    assert len(pre["refuse"]) == 1 and "nx (nx-alias) is +7 s" in pre["refuse"][0] and "sudo date -u -s" in pre["refuse"][0]
    p = L.build_plan(cfg, stamp="S", net_env=NET, pre=pre)
    lab = [s["label"] for s in p["steps"] if s["label"].startswith("clock check")]
    assert lab[0].startswith("clock check NX: +7.0 s from this Mac - REFUSED") and lab[1] == "clock check AGX: +1.0 s from this Mac"
    assert p["refuse"] == pre["refuse"]
    ck = next(s["cmd"] for s in p["steps"] if s["label"].startswith("clock check NX"))
    assert "date +%s" in ck and "-le 5" in ck and "exit 1" in ck                                       # the step fails the launch too
    assert L.preflight(dict(cfg, mode="sim"), {"nx": "x"}, run=_run(t))["clocks"] == {}
    st = (ROOT / "scripts" / "status.sh").read_text()
    assert "clocks()" in st and "date +%s" in st and "STATUS_CLOCK" in st
    page = (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    assert "d.clocks" in page and 's.clock.ok===false?" err"' in page


def test_the_launch_endpoint_refuses_a_bad_clock(monkeypatch):
    import threading
    import urllib.request
    from .conftest import free_port
    monkeypatch.setattr(L, "preflight", lambda cfg, hosts, **kw: {"clocks": {"nx": {"host": "nx-alias", "offset_s": 9.0, "ok": False}},
                                                                    "nic": None, "refuse": ["clock: nx (nx-alias) is +9 s from this Mac"]})
    monkeypatch.setattr(L, "preflight_cached", lambda cfg, hosts, **kw: L.preflight(cfg, hosts))
    runner = L.Runner(dry=False)
    watch = L.Watch(runner, ls=lambda h: [])
    port = free_port()
    srv = L.Server(("127.0.0.1", port), L.make_handler(runner, watch))
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def post(path, body):
        req = urllib.request.Request(f"http://127.0.0.1:{port}{path}", data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json", "X-Launch": "1"})
        return json.loads(urllib.request.urlopen(req, timeout=10).read())
    try:
        cfg = dict(L.load_presets()["lab"])
        r = post("/plan", {"cfg": cfg})
        assert not r["ok"] and any("+9 s" in e for e in r["errors"]) and r["plan"]["refuse"]          # shown, LAUNCH off
        r = post("/launch", {"cfg": cfg})
        assert not r["ok"] and "+9 s" in r["why"] and runner.state != "running"                          # refused, nothing ran
    finally:
        srv.shutdown()
        srv.server_close()


# ---- B1: heartbeats on the judge wire from the first status -------------------------------------------------------------------
def _app(judge_kind):
    cfg = Config()
    cfg.extra["ocs"] = {"judge": judge_kind}
    sent = []
    j = types.SimpleNamespace(publish_report=lambda v, b: sent.append(b) or len(sent), publish_request=lambda b: 1, connected=True,
                              name=judge_kind, network="course", on_course=lambda cb: None, on_command=lambda cb: None,
                              on_link=lambda cb: None, new_run=lambda: None, start=lambda: None, stop=lambda: None)
    return OcsApp(cfg, judge=j), sent


def test_heartbeats_before_the_declaration_on_rctest_not_on_fake():
    app, sent = _app("protobuf")                                           # rc-test and the course: judge protobuf
    assert app.reporting and app.heartbeats.enabled and app.state_body()["reporting"]["default_on"]
    app.heartbeats.tick()
    assert sent == []                                                      # no status yet: nothing fabricated
    app.on_status("x", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL"}})
    app.heartbeats.tick()
    assert [b["state"] for b in sent] == ["STATE_MANUAL"] and app.lifecycle.body()["r"] in ("R0", "R1")   # before any declaration
    fake, sent2 = _app("fake")                                             # the bench: from the declaration, as before
    assert not fake.reporting and not fake.heartbeats.enabled
    fake.on_status("x", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL"}})
    fake.heartbeats.tick()
    assert sent2 == []


def test_stop_reset_keeps_reporting_and_report_off_asks():
    assert [s[0] for s in playbook.STOP_RESET_STEPS] == ["cancel_auto", "abort", "disable_auto", "fcu_mode HOLD", "disarm", "reset"]
    app, _sent = _app("protobuf")
    app.reset_run()
    assert app.heartbeats.enabled                                          # a reset keeps the judge wire's heartbeats
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'ask("<b>STOP REPORTING?</b>' in page and 'send("report off")' in page


# ---- B2 / B3 / B5: the ladder -----------------------------------------------------------------------------------------------------
def test_the_ladder_in_the_handbooks_order():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    titles = re.findall(r'<div class="st2" id="lad-(\d)"><div class="n">\d</div><div class="bd"><div class="t">([^<]+?) <span', page)
    assert [n for n, _ in titles] == ["1", "2", "3", "4", "5", "6"] and 'id="lad-7"' not in page
    words = [t.split(" — ")[0] for _, t in titles]
    assert words == ["handover", "declare", "ARM + enable_auto", "RunStart", "run R5", "the mission"]
    assert page.index('id="rs-gate"') < page.index('class="ladder2"') and page.index('id="ck"') < page.index('id="lad-1"')   # the gate above
    assert page.index('id="clearbtn"') < page.index('id="lad-1"')
    assert page.index('id="ck-decl"') > page.index('id="lad-2"') and page.index('id="ck-decl"') < page.index('id="lad-3"')
    assert page.index('id="rs-judge"') > page.index('id="lad-4"') and page.index('id="rscard"') < page.index('id="lad-5"')
    assert 'id="rs-att"' in page and "attempt ${o.attempt}" in page and "for(let n=1;n<=6;n++)" in page
    assert 'if(line==="arm"){$("fb-arm").click();return}' in page      # the ladder's ARM asks first (the bar's ARM)


def test_the_attempt_counts_declarations_in_the_course_slot():
    app, _sent = _app("protobuf")
    app.lifecycle.course = {"course_id": "ALPHA"}

    def declared():
        app.lifecycle.on_prepare_ack = lambda vid, res, reason: True
        app._prepare_msg_ids = {"m1": "USV1"}
        app.on_ack("x", {"vehicle_id": "USV1", "body": {"ack_of": "m1", "result": "DONE", "state": "AUTO_HOLD"}})
    declared()
    assert app.state_body()["attempt"] == 1
    declared()
    assert app.state_body()["attempt"] == 2
    app.lifecycle.course = {"course_id": "BRAVO"}                         # another slot: from 1 again
    declared()
    assert app.state_body()["attempt"] == 1


def test_r4_reads_ready_and_the_stall_is_a_caution():
    o = {"r": "R4", "declaration_seq": 3, "waiting_s": 75.4, "stall_s": 180, "stalled": False}
    w = W.control_word({"state": "AUTO_HOLD", "robot_state": "STATE_AUTO", "armed": True, "mode": "HOLD"}, o, False)
    assert w["word"] == "READY" and w["why"] == "declared seq 3 · heartbeat AUTO · waiting for the course manager — 1:15"
    w = W.control_word({"state": "AUTO_HOLD", "robot_state": "STATE_AUTO"}, dict(o, stalled=True), False)
    assert w["hint"] == "no RunStart after 3:00 — ask the course manager, or redeclare"
    assert W.control_word({"state": "AUTO_HOLD"}, {"r": "R5"}, False)["word"] == "ARMED"
    m = T.BenchModel.__new__(T.BenchModel) if hasattr(T, "BenchModel") else None
    row = {"text": "no RunStart after 3:00 — ask the course manager, or redeclare", "style": "warn"}
    assert W.AlarmBook.level_of(row) == "CAUTION"                          # no ALARM, no sound
    src = (ROOT / "optiprimex" / "tools" / "bench_tui.py").read_text()
    assert '"run_start_stall")' in src and 'f"no RunStart after {ss // 60}:{ss % 60:02d} — ask the course manager, or redeclare", "warn"' in src
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "waiting for the course manager — ${mss(o.waiting_s)}" in page
    del m


def test_the_judge_strip_data_in_ocs_state(tmp_path):
    app = OcsApp(Config(), judge=_client(tmp_path))
    app.heartbeats.enabled = True
    app.on_status("x", {"vehicle_id": "USV1", "body": {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO"}})
    app.heartbeats.tick()
    app.judge.publish_report("USV1", {"kind": "safe_passage"})
    rep = app.state_body()["reporting"]
    assert rep["hb"]["USV1"]["n"] == 1 and rep["hb"]["USV1"]["state"] == "STATE_AUTO" and rep["hb"]["USV1"]["seq"] == 1
    assert rep["last_report"]["kind"] == "safe_passage" and rep["last_report"]["seq"] == 2


# ---- B4: the event preset's --auto --------------------------------------------------------------------------------------------------
def test_the_event_preset_preselects_with_auto():
    P = L.load_presets()
    assert P["event"]["auto_mission"] is True and not P["sim"]["auto_mission"] and not P["lab"]["auto_mission"]
    assert L.flavour(rec={"cfg": {"preset": "event"}})["auto_mission"] is True
    assert L.flavour(rec={"cfg": {"preset": "sim"}})["auto_mission"] is False
    assert L.flavour(rec={"cfg": {"preset": "event", "auto_mission": False}})["auto_mission"] is False   # unticked on the launcher
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'if($("rs-auto").checked&&$("rs-sel").value){let l=qLine("mission").replace(/ --auto\\b/,"")+" --auto";await rsAct(l)}' in page
    assert '$("rs-auto").checked=!!S.flavour.auto_mission' in page


def test_a_soft_ekf_bad_is_shown_at_once():
    src = (ROOT / "optiprimex" / "tools" / "bench_tui.py").read_text()
    assert '"EKF BAD (no absolute position): a task holds; FAULT if it lasts 3 s - wait for GPS, or takeover on the boat"' in src
    from optiprimex.tools import fault_timing as F
    named, action = F.EXPECT["gps"]
    txt = "EKF BAD (no absolute position): a task holds; FAULT if it lasts 3 s - wait for GPS, or takeover on the boat"
    assert re.search(named, txt) and re.search(action, txt)
