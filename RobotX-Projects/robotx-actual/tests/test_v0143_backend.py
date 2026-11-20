"""v0.1.43 (ROADMAP WP10i-lite, OCS_V2_SPEC §6 + §11) - the backend half:

1. TIMED OUT on the OCS: a cmd with no ack within ocs.ack_timeout_s becomes a record in ocs/state (timed_out, timed_out_total) and a
   WARNING on the page; a later ack is noted as late, never silent.
2. The boat refuses a stale cmd (envelope sent_at older than guards.cmd_max_age_s) with ack REJECTED `stale (...)`. The spec's test:
   the bridge goes down, a START_TASK is published from the OCS side, the wait exceeds the max age, the bridge comes back -> the boat
   answers REJECTED stale and does not start the task.
3. The host-health beat: optiprimex/tools/host_health publishes local/usv/host/<host>; the bridge carries it to
   team/optiprime/v1/telemetry/usv/host/<host>; ocs_web's /state.json carries `hosts` and the warnings.
"""
from __future__ import annotations

import json
import logging
import socket
import subprocess
import threading
import time
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from optiprimex import schemas
from optiprimex.companion import config as cfgmod
from optiprimex.companion.config import Config, GuardsCfg
from optiprimex.companion.mqtt import Dedup, Envelope
from optiprimex.companion.state import Command, RunContext, utc_iso
from optiprimex.ocs.judge_client import NullJudgeClient
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.tools import host_health as HH
from optiprimex.tools import ocs_web
from optiprimex.tools.vcmd import VehicleCommander
from optiprimex.usv.command_handler import CommandHandler, sent_at_unix

from .conftest import MOSQUITTO, PY, ROOT, Brokers, Probe, Proc, free_port, make_ocs_config, make_usv_config, wait_until

NS = "team/optiprime/v1"


# ---- 2. the age check (unit) ----------------------------------------------------------------------------------------------
class AckBus:
    def __init__(self):
        self.acks = []

    def ack(self, cmd, result, reason="", state=""):
        self.acks.append((cmd.action, result, reason))


def handler(max_age: float = 10.0, now: float = 1_800_000_000.0):
    h = CommandHandler.__new__(CommandHandler)
    h.cfg = Config()
    h.cfg.guards.cmd_max_age_s = max_age
    h.bus, h.dedup, h.run = AckBus(), Dedup(), RunContext()
    h.fsm = SimpleNamespace(state=SimpleNamespace(value="AUTO_HOLD"))
    h.pending, h.handled, h.rejected = [], 0, 0
    h.wall = lambda: now
    h.dispatched = []

    def dispatch(cmd, ctx):
        h.dispatched.append(cmd.action)
        return "ACCEPTED", "", True
    h._dispatch = dispatch
    return h


def cmd(sent_at, action="START_TASK", msg_id=None):
    env = {"msg_id": msg_id or f"m-{time.monotonic_ns()}", "kind": "cmd", "body": {"action": action, "task": "GOTO", "args": {}}}
    if sent_at is not ...:
        env["sent_at"] = sent_at
    return Command.from_env(env)


def test_cmd_age_check_fresh_old_missing(caplog):
    now = 1_800_000_000.0
    h = handler(10.0, now)
    h._handle(cmd(utc_iso(now - 1.0)), None)                                   # fresh (clocks within 1 s): executed
    assert h.dispatched == ["START_TASK"] and [a[1] for a in h.bus.acks] == ["ACCEPTED", "DONE"]
    h._handle(cmd(utc_iso(now - 12.34)), None)                                 # old: REJECTED stale, never dispatched
    assert h.dispatched == ["START_TASK"]
    assert h.bus.acks[-1] == ("START_TASK", "REJECTED", "stale (sent 12.3 s ago, limit 10 s)")
    with caplog.at_level(logging.INFO, logger="optiprimex.cmd"):
        h._handle(cmd(...), None)                                              # no sent_at: accepted, with a log line
        h._handle(cmd("not a time"), None)
    assert h.dispatched == ["START_TASK"] * 3 and "age not checked" in caplog.text
    h0 = handler(0.0, now)                                                     # 0 = not checked
    h0._handle(cmd(utc_iso(now - 3600)), None)
    assert h0.dispatched == ["START_TASK"]
    # a redelivered stale cmd (same msg_id) is answered from the dedup, not re-checked into something else
    h._handle(cmd(utc_iso(now - 30), msg_id="dup-1"), None)
    h._handle(cmd(utc_iso(now - 30), msg_id="dup-1"), None)
    assert h.bus.acks[-1][1] == "REJECTED" and h.bus.acks[-1][2].startswith("duplicate: stale")


def test_sent_at_parsing_and_the_config_key():
    assert sent_at_unix("2026-11-06T12:00:00.123Z") == pytest.approx(1793966400.123)
    assert sent_at_unix("2026-11-06T12:00:00Z") == pytest.approx(1793966400.0)              # ocs_gateway's form
    assert sent_at_unix(1793966400) == 1793966400.0 and sent_at_unix(None) is None and sent_at_unix("x") is None
    assert abs(time.time() - sent_at_unix(utc_iso())) < 0.01
    assert GuardsCfg().cmd_max_age_s == 10.0
    for f in ("profiles/bench/usv_bench.yaml", "profiles/pool/usv_pool.yaml", "profiles/pool/usv_lab.yaml",
              "profiles/course/usv_course.yaml", "usv_ui/config/usv_bench_share.yaml"):
        assert cfgmod.load(str(ROOT / f)).guards.cmd_max_age_s == 10.0, f
    with pytest.raises(ValueError):
        cfgmod.from_dict({"guards": {"cmd_max_age_s": -1}})
    for f in ("profiles/bench/ocs_bench.yaml", "profiles/pool/ocs_pool.yaml", "profiles/course/ocs_course.yaml"):
        d = yaml.safe_load((ROOT / f).read_text())
        assert d["ocs"]["ack_timeout_s"] == 5 and d["host"] == {"warn_c": 85, "alarm_c": 95}, f
        assert ocs_web.host_thresholds(str(ROOT / f)) == (85.0, 95.0)
    assert ocs_web.host_thresholds(None) == (85.0, 95.0)
    for f in ("profiles/bench/mosquitto/bridge.conf", "profiles/bench/mosquitto/boat.conf"):
        assert "topic host/+ out 0 local/usv/ team/optiprime/v1/telemetry/usv/\n" in (ROOT / f).read_text(), f
    assert "#BRIDGE topic host/+ out 0 local/usv/ team/optiprime/v1/telemetry/usv/\n" in \
        (ROOT / "profiles/pool/mosquitto/boat_pool.conf.in").read_text()


# ---- 1. TIMED OUT (unit) ----------------------------------------------------------------------------------------------------
class FakeBus:
    def __init__(self):
        self.cfg = Config()
        self.cfg.vehicle_id = "OCS"
        self.run_id = self.declaration_seq = None
        self.published, self._n = [], 0

    def publish(self, topic, kind, body, qos=1, intent_id=None, vehicle_id=None, origin=None, **kw):
        self._n += 1
        self.published.append({"topic": topic, "kind": kind, "body": body})
        return f"msg-{self._n:04d}-abcdef"


def ocs_app(timeout=5.0):
    cfg = Config()
    cfg.extra = {"ocs": {"judge": "none", "ack_timeout_s": timeout, "vehicles": {"USV1": "usv"}}}
    return OcsApp(cfg, bus=FakeBus(), judge=NullJudgeClient())


def test_timed_out_record_warning_and_late_ack():
    app = ocs_app(5.0)
    t0 = time.time()
    mid = app.authority.emit("USV1", "run", "START_TASK", "GOTO", {"x_n": 1}, intent_id="int-1")
    assert app.sweep_acks(now=t0 + 4.0) == [] and app.timed_out_total == 0
    new = app.sweep_acks(now=t0 + 5.2)
    assert len(new) == 1 and app.sweep_acks(now=t0 + 9) == []                   # once
    r = new[0]
    assert r["msg_id"] == mid and r["intent_id"] == "int-1" and r["action"] == "START_TASK" and r["vehicle"] == "USV1"
    assert r["text"] == f"command START_TASK {mid[:8]} TIMED OUT (no ack in 5 s): check the link; repeat deliberately"
    st = app.state_body()
    assert st["timed_out_total"] == 1 and st["timed_out"][0]["msg_id"] == mid and st["ack_timeout_s"] == 5.0
    json.dumps(st)                                                             # it rides ocs/state
    # the late ack: noted on the record and in the text, the counter stays (nothing cleared silently)
    app.on_ack("t", {"vehicle_id": "USV1", "body": {"ack_of": mid, "result": "REJECTED", "reason": "stale (sent 12.0 s ago, limit 10 s)"}})
    st = app.state_body()
    la = st["timed_out"][0]["late_ack"]
    assert la["result"] == "REJECTED" and la["reason"].startswith("stale") and la["after_s"] is not None
    assert "late ack REJECTED after" in st["timed_out"][0]["text"] and st["timed_out_total"] == 1
    # acked in time: never TIMED OUT; an ack that beats the sweep to a fresh cmd counts too
    m2 = app.authority.emit("USV1", "run", "HOLD")
    app.sweep_acks(now=time.time())
    app.on_ack("t", {"body": {"ack_of": m2, "result": "ACCEPTED"}})
    m3 = app.authority.emit("USV1", "run", "RESUME")
    app.on_ack("t", {"body": {"ack_of": m3, "result": "ACCEPTED"}})
    assert app.sweep_acks(now=time.time() + 60) == [] and app.timed_out_total == 1
    # the list keeps the last 10, the counter counts all
    for _ in range(12):
        app.authority.emit("USV1", "run", "HOLD")
    app.sweep_acks(now=time.time() + 60)
    st = app.state_body()
    assert st["timed_out_total"] == 13 and len(st["timed_out"]) == 10


def test_ocs_web_state_hosts_and_warnings():
    web = ocs_web.Web(ocs_web.parse_args(["--ocs-config", str(ROOT / "profiles/bench/ocs_bench.yaml")]), env={}, feeds=False)
    now = time.time()
    beat = lambda host, temps, **kw: json.dumps(HH.envelope({"host": host, "cpu_pct": 12.0, "gpu_pct": None, "temp_c": temps,
                                                             "throttling": None, "disk_free_gb": 50.0, "uptime_s": 100, **kw}, 1)).encode()
    assert web.hosts.on_msg(f"{NS}/telemetry/usv/host/nx", beat("nx", {"cpu-thermal": 86.4, "gpu-thermal": 70.0}))
    assert web.hosts.on_msg(f"{NS}/telemetry/usv/host/agx", beat("agx", {"tj-thermal": 96.0}, throttling=True, throttle_src="tj", disk_free_gb=3.2))
    assert not web.hosts.on_msg(f"{NS}/telemetry/usv/map", b"{}")
    web.m.on_hub("ocs/state", json.dumps({"body": {"r": "R2", "judge_connected": True, "timed_out_total": 1, "timed_out": [
        {"msg_id": "abcdef12-x", "action": "START_TASK", "t_unix": now - 3, "late_ack": None,
         "text": "command START_TASK abcdef12 TIMED OUT (no ack in 5 s): check the link; repeat deliberately"},
        {"msg_id": "old", "action": "HOLD", "t_unix": now - 600, "text": "command HOLD old TIMED OUT"}]}}).encode())
    snap = web.state()
    json.dumps(snap)
    h = snap["hosts"]
    assert set(h) == {"nx", "agx"} and h["nx"]["temp_c"]["cpu-thermal"] == 86.4 and h["nx"]["age_s"] < 2 and h["agx"]["gpu_pct"] is None
    assert snap["host_thresholds"] == {"warn_c": 85.0, "alarm_c": 95.0, "disk_min_gb": 15.0, "disk_red_gb": 8.0, "stale_s": 10.0}   # v0.1.86: 15 / 8
    w = {x["text"]: x["style"] for x in snap["warnings"]}
    assert w["NX hot cpu-thermal 86 °C (warn 85): shade it"] == "warn"
    assert w["AGX HOT tj-thermal 96 °C (alarm 95): shade it / stop the recorder"] == "bad"
    assert w["AGX throttling (tj): shade it"] == "warn"
    assert any(t.startswith("AGX disk 3.2 GB free") for t in w)
    assert w["command START_TASK abcdef12 TIMED OUT (no ack in 5 s): check the link; repeat deliberately"] == "warn"
    assert not any("HOLD old" in t for t in w)                                 # older than 60 s: gone from the list
    assert all(set(x) == {"age", "text", "style"} for x in snap["warnings"])
    # a beat older than 10 s raises nothing (its numbers are old); the row stays with its age
    web.hosts.beats["nx"] = (web.hosts.beats["nx"][0], now - 30)
    snap = web.state()
    assert snap["hosts"]["nx"]["age_s"] >= 29 and not any(x["text"].startswith("NX hot") for x in snap["warnings"])


def test_host_sampler_on_this_machine_and_a_fake_sys(tmp_path):
    b = HH.Sampler(disk_path=str(ROOT)).body("mac")                          # the Mac: no /sys, no /proc -> nulls, never a crash
    for k in ("host", "cpu_pct", "gpu_pct", "temp_c", "throttling", "disk_free_gb", "uptime_s", "t", "cpu_load1"):
        assert k in b, k
    assert b["host"] == "mac" and isinstance(b["disk_free_gb"], float) and b["throttling"] in (None, True, False)
    if not Path("/proc/stat").exists():
        assert b["cpu_pct"] is None and b["temp_c"] is None and b["gpu_pct"] is None and b["throttling"] is None
    schemas.validate(HH.envelope(b, 1))
    # a Jetson-like /sys + /proc: two zones (one past its passive trip), the GPU load file, /proc/stat twice
    sysr, proc = tmp_path / "sys", tmp_path / "proc"
    for i, (name, milli, trip) in enumerate((("cpu-thermal", 52500, 99000), ("gpu-thermal", 97100, 95500))):
        z = sysr / "class" / "thermal" / f"thermal_zone{i}"
        z.mkdir(parents=True)
        (z / "type").write_text(name + "\n")
        (z / "temp").write_text(f"{milli}\n")
        (z / "trip_point_0_type").write_text("passive\n")
        (z / "trip_point_0_temp").write_text(f"{trip}\n")
    (sysr / "devices" / "gpu.0").mkdir(parents=True)
    (sysr / "devices" / "gpu.0" / "load").write_text("473\n")
    proc.mkdir()
    (proc / "stat").write_text("cpu  100 0 100 800 0 0 0 0 0 0\n")
    (proc / "uptime").write_text("1234.56 999.0\n")
    s = HH.Sampler(sys_root=str(sysr), proc_root=str(proc), disk_path=str(tmp_path))
    s.cpu_pct()
    (proc / "stat").write_text("cpu  150 0 150 900 0 0 0 0 0 0\n")             # +100 busy, +100 idle -> 50 %
    b = s.body("agx")
    assert b["cpu_pct"] == 50.0 and b["gpu_pct"] == 47.3 and b["uptime_s"] == 1235
    assert b["temp_c"] == {"cpu-thermal": 52.5, "gpu-thermal": 97.1}
    assert b["throttling"] is True and "gpu-thermal" in b["throttle_src"]
    (sysr / "class" / "thermal" / "thermal_zone1" / "temp").write_text("60000\n")
    assert s.throttling() == (False, "checked")
    out = subprocess.run([PY, "-m", "optiprimex.tools.host_health", "--once", "--host", "nx"], capture_output=True, text=True,
                         cwd=str(ROOT), timeout=20)
    env = json.loads(out.stdout)
    schemas.validate(env)
    assert env["kind"] == "telemetry" and env["source"] == "host_health" and env["body"]["host"] == "nx"


def test_launchers_start_and_stop_the_host_window():
    usv = (ROOT / "scripts/run_usv.sh").read_text()
    agx = (ROOT / "scripts/run_agx.sh").read_text()
    assert "-n host" in usv and "optiprimex.tools.host_health --host nx" in usv
    assert "-n host" in agx and "optiprimex.tools.host_health --host agx" in agx
    assert "for w in mm host log fcu boat" in (ROOT / "scripts/stop_usv.sh").read_text()
    assert "host_health" in (ROOT / "scripts/stop_agx.sh").read_text()
    for s in ("run_usv.sh", "run_agx.sh", "stop_usv.sh", "stop_agx.sh"):
        r = subprocess.run(["bash", str(ROOT / "scripts" / s), "--help"], capture_output=True, text=True, timeout=10)
        assert r.returncode == 0 and r.stdout.strip(), s
    assert "host" in subprocess.run(["bash", str(ROOT / "scripts/run_usv.sh"), "--help"], capture_output=True, text=True, timeout=10).stdout


# ---- the live part: two brokers, the bridge through a cuttable TCP proxy, the mission manager, the OCS in-process -------------
class CutProxy:
    """127.0.0.1:<port> -> the hub. cut() closes every connection and refuses new ones (the radio link gone); restore() lets the
    bridge reconnect. Both brokers stay up: the hub keeps the bridge's persistent session and queues its QoS-1 traffic."""

    def __init__(self, target_port: int):
        self.target = target_port
        self.srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", 0))
        self.srv.listen(8)
        self.port = self.srv.getsockname()[1]
        self.open = True
        self.socks = []
        self.lock = threading.Lock()
        self.stopped = False
        threading.Thread(target=self._accept, daemon=True).start()

    def _accept(self):
        while not self.stopped:
            try:
                c, _ = self.srv.accept()
            except OSError:
                return
            if not self.open:
                c.close()
                continue
            try:
                u = socket.create_connection(("127.0.0.1", self.target), timeout=2)
            except OSError:
                c.close()
                continue
            with self.lock:
                self.socks += [c, u]
            for a, b in ((c, u), (u, c)):
                threading.Thread(target=self._pump, args=(a, b), daemon=True).start()

    @staticmethod
    def _pump(a, b):
        try:
            while True:
                d = a.recv(65536)
                if not d:
                    break
                b.sendall(d)
        except OSError:
            pass
        for s in (a, b):
            try:
                s.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def cut(self):
        self.open = False
        with self.lock:
            socks, self.socks = self.socks, []
        for s in socks:
            try:
                s.shutdown(socket.SHUT_RDWR)
                s.close()
            except OSError:
                pass

    def restore(self):
        self.open = True

    def stop(self):
        self.stopped = True
        self.cut()
        self.srv.close()


@pytest.fixture(scope="module")
def live(fake_fcu, tmp_path_factory):
    if not MOSQUITTO:
        pytest.skip("mosquitto binary not found")
    tmp = tmp_path_factory.mktemp("v0143")
    b = Brokers(tmp=tmp, hub_port=free_port(), boat_port=free_port())
    b.start_hub()
    proxy = CutProxy(b.hub_port)
    real_hub = b.hub_port
    b.hub_port = proxy.port                     # start_boat writes the bridge address from hub_port: through the proxy
    b.start_boat()
    b.hub_port = real_hub
    cfg = make_usv_config(tmp, b, fake_fcu, **{"guards.cmd_max_age_s": 2.0, "guards.test_mode": True})
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(cfg)], tmp / "mm.log")
    probe = Probe(b.hub_port, [f"{NS}/#", "ocs/#", "local/#"]).start()
    boatp = Probe(b.boat_port, [f"{NS}/status/usv", "$SYS/broker/connection/#"]).start()
    vc = VehicleCommander(port=b.hub_port)
    vc.start()
    ocs = None
    try:
        assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 20), mm.log()
        ocfg = cfgmod.load(str(make_ocs_config(tmp, b, judge="none", ack_timeout_s=1.0)))
        ocfg.mqtt.client_id = f"ocs-v0143-{free_port()}"
        ocs = OcsApp(ocfg)
        ocs.start()
        yield {"b": b, "proxy": proxy, "mm": mm, "probe": probe, "boatp": boatp, "vc": vc, "ocs": ocs, "fcu": fake_fcu, "tmp": tmp}
    finally:
        if ocs:
            ocs.stop()
        vc.stop()
        probe.stop()
        boatp.stop()
        mm.kill()
        proxy.stop()
        b.stop()


def boat_state(live):
    m = live["boatp"].last(f"{NS}/status/usv")
    return m["env"]["body"]["state"] if m else None


def bridge_up(live) -> bool:
    m = live["boatp"].last("$SYS/broker/connection/")
    return bool(m) and m["payload"].strip() == b"1"


def test_start_task_queued_while_the_bridge_is_down_is_rejected_stale(live):
    vc, ocs, proxy, fcu = live["vc"], live["ocs"], live["proxy"], live["fcu"]
    assert wait_until(lambda: bridge_up(live), 10), "bridge never came up through the proxy"
    acks = vc.send("run", "ENABLE_AUTO", wait_s=4.0)                         # test mode: AUTO_HOLD without a declaration
    assert [a["body"]["result"] for a in acks] == ["ACCEPTED", "DONE"], acks
    assert wait_until(lambda: boat_state(live) == "AUTO_HOLD", 5)
    targets0 = fcu.state()["counters"]["targets"]
    # the bridge goes down; the OCS emits START_TASK GOTO (a run-free task: fresh, the boat would take it) - the hub queues it
    proxy.cut()
    assert wait_until(lambda: not bridge_up(live), 10), "bridge did not notice the cut"
    t_sent = time.time()
    mid = ocs.authority.emit("USV1", "run", "START_TASK", "GOTO", {"x_n": 15.0, "y_e": 0.0}, intent_id="v0143-stale")
    # the OCS: no ack in ack_timeout_s (1 s here) -> TIMED OUT, in ocs/state
    assert wait_until(lambda: any(r["msg_id"] == mid for r in ocs.timed_out), 5), "never TIMED OUT"
    st = live["probe"].wait(lambda m: m["topic"] == "ocs/state" and (m["env"]["body"].get("timed_out_total") or 0) >= 1, 5, include_past=True)
    assert st and st["env"]["body"]["timed_out"][-1]["text"].startswith("command START_TASK ")
    time.sleep(max(0.0, 2.0 + 1.0 - (time.time() - t_sent)))                 # older than cmd_max_age_s (2 s) before it can arrive
    proxy.restore()
    assert wait_until(lambda: bridge_up(live), 20), "bridge did not come back"
    m = live["probe"].wait(lambda m: m["topic"].endswith("ack/usv") and m["env"]["body"].get("ack_of") == mid, 20, include_past=True)
    assert m, "no ack for the queued START_TASK after the bridge came back"
    body = m["env"]["body"]
    assert body["result"] == "REJECTED" and body["reason"].startswith("stale (sent ") and "limit 2 s)" in body["reason"], body
    time.sleep(1.5)
    assert boat_state(live) == "AUTO_HOLD"                                    # the task did not start
    assert fcu.state()["counters"]["targets"] == targets0
    acks = [a for a in live["probe"].all(f"{NS}/ack/usv") if a["env"]["body"].get("ack_of") == mid]
    assert [a["env"]["body"]["result"] for a in acks] == ["REJECTED"]
    # the OCS noted the late ack on the TIMED OUT record (nothing silent)
    assert wait_until(lambda: (next(r for r in ocs.timed_out if r["msg_id"] == mid).get("late_ack") or {}).get("result") == "REJECTED", 5)
    # control: the same command sent fresh is taken (so the refusal above was the age, nothing else); then back to AUTO_HOLD
    acks = vc.send("run", "START_TASK", task="GOTO", args={"x_n": 15.0, "y_e": 0.0}, wait_s=4.0)
    assert [a["body"]["result"] for a in acks] == ["ACCEPTED", "DONE"], acks
    assert [a["body"]["result"] for a in vc.send("run", "ABORT_TASK", wait_s=4.0)][0] == "ACCEPTED"
    assert wait_until(lambda: boat_state(live) == "AUTO_HOLD", 10)
    vc.send("run", "DISABLE_AUTO", wait_s=3.0)


def test_host_beat_is_bridged_to_the_hub(live):
    b = live["b"]
    p = Proc([PY, "-m", "optiprimex.tools.host_health", "--host", "nx", "--mqtt", f"127.0.0.1:{b.boat_port}", "--period", "0.5"],
             live["tmp"] / "host.log")
    try:
        lp = Probe(b.boat_port, ["local/usv/host/#"]).start()
        ml = lp.wait(lambda m: m["topic"] == "local/usv/host/nx", 10, include_past=True)
        lp.stop()
        assert ml, p.log()
        m = live["probe"].wait(lambda m: m["topic"] == f"{NS}/telemetry/usv/host/nx", 10, include_past=True)
        assert m, "the bridge did not carry local/usv/host/nx to the hub"
        env = m["env"]
        schemas.validate(env)
        body = env["body"]
        assert body["host"] == "nx" and env["kind"] == "telemetry" and isinstance(body["disk_free_gb"], float)
        for k in ("cpu_pct", "gpu_pct", "temp_c", "throttling", "uptime_s", "t"):
            assert k in body, k
        if not Path("/sys/class/thermal").exists():
            assert body["temp_c"] is None and body["throttling"] is None and body["gpu_pct"] is None
        assert not live["probe"].all("local/")                                # local/# itself never crosses
        hh = ocs_web.HostHealth("usv")                                        # what ocs_web does with it
        assert hh.on_msg(m["topic"], m["payload"]) and hh.view()["nx"]["host"] == "nx"
    finally:
        p.kill()
