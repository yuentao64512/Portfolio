"""Shared fixtures: two Mosquitto brokers with the real bridge, the fake FCU, subprocess nodes, an MQTT probe."""
from __future__ import annotations

import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Callable, Dict, List, Optional

import paho.mqtt.client as mqtt
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
os.environ.setdefault("OPTIPRIMEX_RC_SEQ", "off")      # v0.1.59 (A1): the suite never continues logs/ocs/rc_seq.json
os.environ.setdefault("OCS_MAP_REC", "off")             # v0.1.81.1: the suite's OCS page servers never write logs/ocs/maps
os.environ.setdefault("OCS_MAP_REC", "off")             # v0.1.81.1: the suite's OCS page servers never write logs/ocs/maps
os.environ.setdefault("OPTIPRIMEX_PREFLIGHT", "off")   # v0.1.59 (A6): the launcher's clock / NIC preflight never ssh-es from a test
PY = sys.executable
# brew puts mosquitto in /opt/homebrew/sbin, which is not always on PATH inside an IDE; MOSQUITTO=... overrides
MOSQUITTO = os.environ.get("MOSQUITTO") or shutil.which("mosquitto") or next(
    (p for p in ("/opt/homebrew/sbin/mosquitto", "/usr/local/sbin/mosquitto", "/usr/sbin/mosquitto") if os.access(p, os.X_OK)), None)


def free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def free_udp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def wait_until(pred: Callable[[], bool], timeout_s: float, period_s: float = 0.1, what: str = "") -> bool:
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        if pred():
            return True
        time.sleep(period_s)
    return False


def tcp_open(port: int) -> bool:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.settimeout(0.2)
        return s.connect_ex(("127.0.0.1", port)) == 0


class Proc:
    """A child process whose output goes to a file; killed on teardown."""

    def __init__(self, args: List[str], log_path: Path, env: Optional[dict] = None):
        self.args = args
        self.log_path = log_path
        self.fh = open(log_path, "w")
        self.p = subprocess.Popen(args, stdout=self.fh, stderr=subprocess.STDOUT, cwd=str(ROOT),
                                  env={**os.environ, **(env or {})}, start_new_session=True)

    def kill(self, sig=signal.SIGTERM, wait_s: float = 3.0) -> None:
        if self.p.poll() is None:
            try:
                os.killpg(os.getpgid(self.p.pid), sig)
            except ProcessLookupError:
                pass
            try:
                self.p.wait(timeout=wait_s)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.p.pid), signal.SIGKILL)
                self.p.wait(timeout=2)
        self.fh.close()

    def log(self) -> str:
        self.fh.flush()
        return Path(self.log_path).read_text(errors="ignore")

    @property
    def alive(self) -> bool:
        return self.p.poll() is None


@dataclass
class Brokers:
    tmp: Path
    hub_port: int
    boat_port: int
    hub: Optional[Proc] = None
    boat: Optional[Proc] = None
    persist: Dict[str, Path] = field(default_factory=dict)

    def _persist_dir(self, name: str) -> Path:
        """Persistence dir in /tmp (not under the 0700 pytest tmp): mosquitto may run as its own user."""
        if name not in self.persist:
            d = Path(tempfile.mkdtemp(prefix=f"optiprimex_{name}_persist_"))
            os.chmod(d, 0o777)
            self.persist[name] = d
        return self.persist[name]

    def _conf(self, name: str, text: str) -> Path:
        p = self.tmp / f"{name}.conf"
        p.write_text(text)
        return p

    def start_hub(self) -> None:
        d = self._persist_dir("hub")
        conf = self._conf("hub", f"listener {self.hub_port} 127.0.0.1\nprotocol mqtt\nallow_anonymous true\n"
                                 f"persistence true\npersistence_location {d}/\nmax_queued_messages 10000\nsys_interval 1\n")
        self.hub = Proc([MOSQUITTO, "-c", str(conf)], self.tmp / "hub.log")
        assert wait_until(lambda: tcp_open(self.hub_port), 5), "hub broker did not start"

    def stop_hub(self) -> None:
        if self.hub:
            self.hub.kill()
            self.hub = None
            wait_until(lambda: not tcp_open(self.hub_port), 3)

    def start_boat(self) -> None:
        d = self._persist_dir("boat")
        bridge = (ROOT / "profiles" / "bench" / "mosquitto" / "bridge.conf").read_text()
        bridge = bridge.replace("address 127.0.0.1:1882", f"address 127.0.0.1:{self.hub_port}")
        conf = self._conf("boat", f"listener {self.boat_port} 127.0.0.1\nprotocol mqtt\nallow_anonymous true\n"
                                  f"persistence true\npersistence_location {d}/\nmax_queued_messages 10000\nsys_interval 1\n\n{bridge}")
        self.boat = Proc([MOSQUITTO, "-c", str(conf)], self.tmp / "boat.log")
        assert wait_until(lambda: tcp_open(self.boat_port), 5), "boat broker did not start"

    def stop(self) -> None:
        for b in (self.boat, self.hub):
            if b:
                b.kill()
        for d in self.persist.values():
            shutil.rmtree(d, ignore_errors=True)


@pytest.fixture(scope="module")
def brokers(tmp_path_factory):
    if not MOSQUITTO:
        pytest.skip("mosquitto binary not found")
    tmp = tmp_path_factory.mktemp("brokers")
    b = Brokers(tmp=tmp, hub_port=free_port(), boat_port=free_port())
    b.start_hub()
    b.start_boat()
    yield b
    b.stop()


class FakeFcuHandle:
    def __init__(self, proc: Proc, mav_port: int, ctrl_port: int):
        self.proc, self.mav_port, self.ctrl_port = proc, mav_port, ctrl_port

    def ctrl(self, msg: dict, reply: bool = True, timeout_s: float = 1.0) -> Optional[dict]:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(timeout_s)
            s.sendto(json.dumps(msg).encode(), ("127.0.0.1", self.ctrl_port))
            if not reply:
                return None
            try:
                data, _ = s.recvfrom(65535)
                return json.loads(data)
            except socket.timeout:
                return None

    def set(self, **kw) -> None:
        self.ctrl({"set": kw})

    def state(self) -> dict:
        return self.ctrl({"get": "state"}) or {}


@pytest.fixture(scope="module")
def fake_fcu(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("fcu")
    mav_port, ctrl_port = free_udp_port(), free_udp_port()
    proc = Proc([PY, "-m", "optiprimex.sim.fake_fcu", "--conn", f"udpout:127.0.0.1:{mav_port}", "--control-port", str(ctrl_port),
                 "--armed"], tmp / "fcu.log")
    h = FakeFcuHandle(proc, mav_port, ctrl_port)
    assert wait_until(lambda: h.ctrl({"get": "state"}, timeout_s=0.3) is not None, 5), "fake FCU did not start"
    yield h
    proc.kill()


def make_usv_config(tmp: Path, brokers: Brokers, fake: FakeFcuHandle, **overrides) -> Path:
    cfg = yaml.safe_load((ROOT / "profiles" / "bench" / "usv_bench.yaml").read_text())
    cfg["mavlink"]["endpoint"] = f"udpin:127.0.0.1:{fake.mav_port}"
    cfg["mqtt"]["port"] = brokers.boat_port
    cfg["ocs_gateway"] = {"enabled": True, "port": free_port()}
    cfg["log_dir"] = str(tmp / "logs")
    for k, v in overrides.items():
        sect, key = k.split(".")
        cfg.setdefault(sect, {})[key] = v
    p = tmp / "usv.yaml"
    p.write_text(yaml.safe_dump(cfg))
    return p


def make_ocs_config(tmp: Path, brokers: Brokers, **ocs_overrides) -> Path:
    cfg = yaml.safe_load((ROOT / "profiles" / "bench" / "ocs_bench.yaml").read_text())
    cfg["mqtt"]["port"] = brokers.hub_port
    cfg["ocs"]["judge_port"] = brokers.hub_port
    cfg["ocs"].update(ocs_overrides)
    cfg["log_dir"] = str(tmp / "logs")
    p = tmp / "ocs.yaml"
    p.write_text(yaml.safe_dump(cfg))
    return p


class Probe:
    """Subscribes to topics on a broker and keeps every message with its arrival time."""

    def __init__(self, port: int, topics: List[str], client_id: Optional[str] = None, clean_session: bool = True):
        self.port = port
        self.topics = topics
        self.msgs: List[dict] = []
        self._lock = threading.Lock()
        self.connected = threading.Event()
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id or f"probe-{free_port()}",
                             clean_session=clean_session, protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = lambda *a, **k: self.connected.clear()
        self.c.on_message = self._on_message
        self.c.reconnect_delay_set(1, 2)

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        for t in self.topics:
            client.subscribe(t, 1)
        self.connected.set()

    def _on_message(self, client, userdata, msg):
        rec = {"t": time.monotonic(), "topic": msg.topic, "payload": msg.payload}
        try:
            rec["env"] = json.loads(msg.payload)
        except Exception:
            rec["env"] = None
        with self._lock:
            self.msgs.append(rec)

    def start(self):
        self.c.connect_async("127.0.0.1", self.port, keepalive=10)
        self.c.loop_start()
        assert self.connected.wait(5), "probe did not connect"
        return self

    def stop(self):
        self.c.loop_stop()
        self.c.disconnect()

    def clear(self):
        with self._lock:
            self.msgs = []

    def all(self, topic_prefix: Optional[str] = None) -> List[dict]:
        with self._lock:
            ms = list(self.msgs)
        if topic_prefix:
            ms = [m for m in ms if m["topic"].startswith(topic_prefix)]
        return ms

    def last(self, topic_prefix: str) -> Optional[dict]:
        ms = self.all(topic_prefix)
        return ms[-1] if ms else None

    def wait(self, pred: Callable[[dict], bool], timeout_s: float, topic_prefix: Optional[str] = None,
             include_past: bool = False) -> Optional[dict]:
        """Wait for a message matching pred; by default only messages arriving after this call count."""
        t0 = time.monotonic()
        seen = 0 if include_past else len(self.all(topic_prefix))
        while time.monotonic() - t0 < timeout_s:
            ms = self.all(topic_prefix)
            for m in ms[seen:]:
                if pred(m):
                    return m
            seen = len(ms)
            time.sleep(0.05)
        return None


# v0.1.86 (WP42 §2.1): the presets changed (event removed; sim · lab · bench-uavmap · m1t2-uav · m1t2-both · m1t1-usv · m1t1-gates · custom).
# The test files written against the v0.1.85 set keep it: tests/fixtures/presets_v0185.yaml via LAUNCH_PRESETS (launcher.load_presets).
LEGACY_PRESET_TESTS = {"test_launcher", "test_launcher_tk", "test_v0147_ab", "test_v0143", "test_v0159", "test_v0165", "test_v0163",
                       "test_v0169", "test_v0168", "test_v0174", "test_v017801", "test_v0182", "test_v017809", "test_v017806", "test_wp9d"}


@pytest.fixture(autouse=True)
def _legacy_presets(request, monkeypatch):
    if request.module.__name__.rsplit(".", 1)[-1] in LEGACY_PRESET_TESTS:
        monkeypatch.setenv("LAUNCH_PRESETS", str(ROOT / "tests" / "fixtures" / "presets_v0185.yaml"))
    yield
