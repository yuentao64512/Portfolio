#!/usr/bin/env python3
"""fake_cannon_pi: the water cannon's Raspberry Pi in the simulator (v0.1.57, ROADMAP WP12, decision D12).

Listens on the cannon sender's UDP port (cv/cannon/cannon_sender.py, `dock_cannon.v1` packets), verifies each packet's
HMAC-SHA256 over the sorted-keys JSON body exactly as the students' receiver does, and publishes what it heard at 2 Hz on the
boat broker:

  local/<v>/cannon/status  {ok, last_t, age_s, target, window_id, hmac_ok, n_rx, n_bad, status, sequence, session, key}

  ok        the last packet verified and it is younger than 2 s (the link works) - says nothing about the target
  last_t    UTC of the last packet that verified (null: none yet); age_s its age in s
  target    the last verified packet's target (null = the sender says "no aim"); window_id its window (1 upper, 2 lower)
  hmac_ok   false while a packet failed its check in the last 2 s (a bad packet among good ones stays visible), else the last
            packet's check; null before any packet
  n_rx      packets received (good + bad); n_bad those that failed (HMAC, JSON, envelope)
  key       "test": the sim's generated key (this program never reads the real key file)

The test key: at start it GENERATES 32 random bytes (hex) into --key-out (default logs/cannon_test.key, mode 0600) and the
sender is pointed at that file (`--key`). It refuses a --key-out named buoy_link.key. `--with-sender "ARGS"` starts
cv/cannon/cannon_sender.py as its child with that key and this listener as its destination, and stops it on exit - one tmux
window (`cannon`) for both in run_bench.sh / run_sim.sh. Informational only: the fake dock still lights on the aim topic (D13).

  python -m optiprimex.sim.fake_cannon_pi --port 1884 --listen 127.0.0.1:5005 --key-out logs/bench/latest/cannon_test.key \\
      --with-sender "--mqtt 127.0.0.1:1884"

v0.1.61 (WP12-rest): by default (`--receiver students`) the Pi IS the students' receiver - cv/cannon/pi/color_cannon_receiver.py,
the verbatim snapshot, copied with the sim config (pi/color_cannon_config.sim.json: allowed colour red, the generated key, this
port) into <key-out's folder>/cannon_pi/ and started WITHOUT --live (their DRY_RUN: no GPIO, no servo, no pump) - and the status is
what cv/cannon/pi/cannon_status_pub.py publishes on the real Pi from that receiver's own log (event, reason, mode, pump, ready,
pan / tilt; the v0.1.57 fields kept). The sender is started with `--protocol dock_color_aim.v1`. `--truth-world WORLD
--target-out FILE` also writes the cannon's target from the world's truth (optiprimex/sim/fake_cannon_target.py: the lit window
from sim/dock/state, the pose from mm/state) for the geometric sim; with the camera path the sim observer writes that file
instead (`--cannon red`). `--receiver ours` = the v0.1.57 verifier (PiSim) and the legacy `dock_cannon.v1` sender.

Nothing here commands the boat or the FCU.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import os
import secrets
import shlex
import signal
import socket
import subprocess
import sys
import shutil
import threading
import time
from pathlib import Path
from typing import Optional, Tuple

LINK_STALE_S = 2.0
STATUS_PERIOD_S = 0.5           # 2 Hz
KIT = Path(__file__).resolve().parents[2]
SENDER = KIT / "cv" / "cannon" / "cannon_sender.py"
PI_DIR = KIT / "cv" / "cannon" / "pi"
RECEIVER = PI_DIR / "color_cannon_receiver.py"
SIM_RX_CONFIG = PI_DIR / "color_cannon_config.sim.json"
TARGET_PERIOD_S = 0.2           # the truth target at 5 Hz (an observer writes every frame)


def receiver_config(listen, key_path, base=SIM_RX_CONFIG) -> dict:
    """The sim config for the students' receiver: theirs keys, our placeholder values, this run's key and port."""
    cfg = json.loads(Path(base).read_text())
    cfg["network"].update(bind_ip=listen[0], port=int(listen[1]), allowed_sender_ip="127.0.0.1", key_file=str(Path(key_path).resolve()))
    return cfg


def stage_receiver(run_dir, listen, key_path) -> Tuple[Path, Path]:
    """Copy the snapshot (unchanged) and its config into run_dir: its ROOT-relative log folder lands there, never in the tree."""
    d = Path(run_dir)
    d.mkdir(parents=True, exist_ok=True)
    rx = d / RECEIVER.name
    shutil.copyfile(RECEIVER, rx)
    cfg = d / "color_cannon_config.json"
    cfg.write_text(json.dumps(receiver_config(listen, key_path), indent=2))
    return rx, cfg


sys.path.insert(0, str(KIT / "cv" / "cannon"))
import aim_core as A  # noqa: E402  (v0.1.61: the target file)


def utc_iso() -> str:
    from ..companion.state import utc_iso as u
    return u()


def verify(packet: bytes, key: bytes):
    """-> (hmac_ok, body or None): the students' receiver-side check (dock_mapper_sender.verify_packet)."""
    try:
        env = json.loads(packet)
        body = env["body"]
        raw = json.dumps(body, separators=(",", ":"), sort_keys=True).encode()
        ok = hmac.compare_digest(hmac.new(key, raw, hashlib.sha256).hexdigest(), str(env["hmac_sha256"]))
        return ok, body if isinstance(body, dict) else None
    except (ValueError, KeyError, TypeError):
        return False, None


def generate_key(path) -> bytes:
    """A fresh random test key (64 hex chars) at path, 0600. Never reads what was there; never the real key's name."""
    p = Path(path)
    if p.name == "buoy_link.key":
        raise SystemExit("fake_cannon_pi: refusing to write a test key named buoy_link.key (the real key's name)")
    p.parent.mkdir(parents=True, exist_ok=True)
    key = secrets.token_hex(32)
    fd = os.open(str(p), os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w") as f:
        f.write(key + "\n")
    return key.encode()


class PiSim:
    """The receiver as a pure state machine: receive(packet, now) per datagram, status(now) for the wire."""

    def __init__(self, key: bytes):
        self.key = key
        self.n_rx = self.n_bad = 0
        self.hmac_ok: Optional[bool] = None
        self.last: Optional[dict] = None          # the last verified body
        self.last_utc: Optional[str] = None
        self.last_t = -1e18
        self.bad_t = -1e18                        # the last failed packet

    def receive(self, packet: bytes, now: float) -> bool:
        self.n_rx += 1
        ok, body = verify(packet, self.key)
        self.hmac_ok = ok
        if not ok or body is None:
            self.n_bad += 1
            self.bad_t = now
            return False
        self.last, self.last_t = body, now
        self.last_utc = utc_iso()
        return True

    def status(self, now: float) -> dict:
        b = self.last or {}
        age = None if self.last is None else round(now - self.last_t, 2)
        tgt = b.get("target") if isinstance(b.get("target"), dict) else None
        h = None if self.hmac_ok is None else bool(self.hmac_ok and now - self.bad_t > LINK_STALE_S)
        return {"ok": bool(h) and age is not None and age <= LINK_STALE_S, "last_t": self.last_utc, "age_s": age,
                "target": tgt, "window_id": tgt.get("window_id") if tgt else None,
                "hmac_ok": h, "n_rx": self.n_rx, "n_bad": self.n_bad, "status": b.get("status"),
                "sequence": b.get("sequence"), "session": b.get("session"), "key": "test"}


class FakeCannonPiNode:
    def __init__(self, a: argparse.Namespace):
        import paho.mqtt.client as mqtt
        self.a = a
        host, _, port = a.listen.partition(":")
        self.listen = (host or "127.0.0.1", int(port or 5005))
        self.key_path = Path(a.key_out)
        key = generate_key(self.key_path)
        self.students = a.receiver == "students"
        self.sim = None if self.students else PiSim(key)
        self.topic = f"local/{a.vehicle}/cannon/status"
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"fake-cannon-pi-{a.vehicle}-{os.getpid()}", protocol=mqtt.MQTTv311)
        self.c.reconnect_delay_set(1, 2)
        self.c.on_connect = self._on_connect
        self.c.on_message = self._on_message
        self.sock = None
        if not self.students:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(self.listen)
            self.sock.settimeout(0.1)
        self.child: Optional[subprocess.Popen] = None
        self.rx: Optional[subprocess.Popen] = None
        self.pub = None
        self.seq = 0
        self.lock = threading.Lock()
        self.world = None
        self.pose: Optional[dict] = None
        self.dock_state: Optional[dict] = None
        if a.truth_world:
            from . import world as W
            self.world = W.load(a.truth_world)
            self.frame = W.WorldFrame(self.world.get("origin"))

    def log(self, text: str) -> None:
        print(f"[fake_cannon_pi] {text}", flush=True)

    def _on_connect(self, c, u, f, rc, p=None):
        c.subscribe(f"local/{self.a.vehicle}/cannon/aim", 0)
        if self.world is not None:
            c.subscribe(f"local/{self.a.vehicle}/mm/state", 0)
            c.subscribe("sim/dock/state", 0)

    def _on_message(self, c, u, msg):
        if msg.topic.endswith("/cannon/aim"):
            if self.pub is not None:
                self.pub.on_aim_message(msg.payload)
            return
        try:
            env = json.loads(msg.payload)
        except ValueError:
            return
        body = env.get("body", env) if isinstance(env, dict) else None
        if not isinstance(body, dict):
            return
        with self.lock:
            if msg.topic.endswith("/mm/state"):
                from . import world as W
                self.pose = W.pose_from_state(body, self.frame)
            elif msg.topic == "sim/dock/state":
                self.dock_state = body

    def write_truth_target(self) -> None:
        from .fake_cannon_target import truth_target
        with self.lock:
            pose, dock = self.pose, self.dock_state
        status, target = truth_target(self.world, pose, dock, self.a.color)
        A.write_target(self.a.target_out, status, target, self.a.color, "truth")

    def start_receiver(self) -> None:
        rx, cfg = stage_receiver(self.key_path.parent / "cannon_pi", self.listen, self.key_path)
        cmd = [sys.executable, str(rx), "--config", str(cfg)]                   # no --live: their DRY_RUN
        self.log(f"receiver (the students', DRY_RUN): {' '.join(cmd)}")
        self.rx = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.STDOUT)
        sys.path.insert(0, str(PI_DIR))
        import cannon_status_pub as P
        self.pub = P.Publisher(self.c, rx.parent / "color_receiver_logs", self.a.vehicle, self.a.vehicle_id, "test",
                               source="fake_cannon_pi", log=print)

    def start_sender(self) -> None:
        if self.a.with_sender is None:
            return
        cmd = [sys.executable, str(SENDER), "--key", str(self.key_path), "--destination", "%s:%d" % self.listen]
        cmd += ["--protocol", "dock_color_aim.v1" if self.students else "dock_cannon.v1"]
        if self.students:
            cmd += ["--color", self.a.color]
            if self.a.target_out:
                cmd += ["--target-file", str(self.a.target_out)]
        cmd += shlex.split(self.a.with_sender)
        self.log("sender: " + " ".join(cmd))
        self.child = subprocess.Popen(cmd)

    def publish(self, now: float) -> None:
        with self.lock:
            st = self.sim.status(now)
        self.seq += 1
        env = {"schema_version": 1, "msg_id": f"fcp-{self.seq:010d}", "seq": self.seq, "sent_at": utc_iso(), "run_id": None,
               "declaration_seq": None, "vehicle_id": self.a.vehicle_id, "kind": "local", "source": "fake_cannon_pi", "body": st}
        self.c.publish(self.topic, json.dumps(env), qos=0)

    def _ours_step(self, state: dict) -> None:
        """--receiver ours: the v0.1.57 verifier on our own socket."""
        try:
            pkt, _addr = self.sock.recvfrom(65535)
            with self.lock:
                good = self.sim.receive(pkt, time.monotonic())
            if not good and time.monotonic() - state["last_bad_log"] >= 1.0:
                state["last_bad_log"] = time.monotonic()
                self.log(f"BAD packet (HMAC / format) - n_bad {self.sim.n_bad}")
        except socket.timeout:
            pass
        now = time.monotonic()
        st = self.sim.status(now)
        sig = (st["ok"], st["window_id"], st["status"])
        if sig != state["last_sig"]:
            self.log(f"link {'ok' if st['ok'] else 'NOT ok'} · status {st['status']} · window_id {st['window_id']} · n_rx {st['n_rx']}")
            state["last_sig"] = sig
        if now - state["last_pub"] >= STATUS_PERIOD_S:
            self.publish(now)
            state["last_pub"] = now

    def run(self) -> None:
        from ..companion.mqtt import start_presence
        self.c.connect_async(self.a.host, self.a.port, keepalive=15)
        self.c.loop_start()
        stop_presence = start_presence(self.c, "fake_cannon_pi", f"{self.a.host}:{self.a.port}")
        self.log(f"udp {self.listen[0]}:{self.listen[1]} ({'the students receiver' if self.students else 'ours, v0.1.57'}); "
                 f"test key {self.key_path} (generated); status -> {self.topic} at 2 Hz"
                 + (f"; truth target ({self.a.color}) -> {self.a.target_out}" if self.world is not None else ""))
        if self.students:
            self.start_receiver()
            time.sleep(0.5)                                     # the receiver binds the port before the sender's first packet
        self.start_sender()
        state = {"last_pub": 0.0, "last_sig": None, "last_bad_log": -1e9}
        last_target = 0.0
        try:
            while True:
                if self.students:
                    self.pub.tick()
                    time.sleep(0.05)
                    if self.rx is not None and self.rx.poll() is not None:
                        self.log(f"the students' receiver exited ({self.rx.returncode})")
                        self.rx = None
                else:
                    self._ours_step(state)
                if self.world is not None and self.a.target_out and time.monotonic() - last_target >= TARGET_PERIOD_S:
                    self.write_truth_target()
                    last_target = time.monotonic()
                if self.child is not None and self.child.poll() is not None:
                    self.log(f"the sender exited ({self.child.returncode})")
                    self.child = None
        except KeyboardInterrupt:
            pass
        finally:
            for name, proc in (("sender", self.child), ("receiver", self.rx)):     # the sender first: its SENDER_STOPPED reaches the receiver
                if proc is not None:
                    proc.terminate()
                    try:
                        proc.wait(5)
                    except subprocess.TimeoutExpired:
                        proc.kill()
            stop_presence()
            self.c.loop_stop()
            if self.sock is not None:
                self.sock.close()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1", help="boat broker host")
    ap.add_argument("--port", type=int, default=1884, help="boat broker")
    ap.add_argument("--listen", default="127.0.0.1:5005", help="udp host:port the sender sends to (cannon_config.json pi_port)")
    ap.add_argument("--key-out", default="logs/cannon_test.key", help="where the generated TEST key is written (never the real key)")
    ap.add_argument("--with-sender", default=None, metavar="ARGS", help='start cv/cannon/cannon_sender.py as a child with the test key '
                    'and this listener, plus ARGS (e.g. "--mqtt 127.0.0.1:1884")')
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--receiver", choices=("students", "ours"), default="students", help="v0.1.61: students = their "
                    "color_cannon_receiver.py in DRY_RUN + the Pi's status publisher (default); ours = the v0.1.57 verifier")
    ap.add_argument("--color", choices=("red", "green"), default="red", help="v0.1.61: the colour aimed at (sender + truth target)")
    ap.add_argument("--truth-world", default=None, help="v0.1.61: write the cannon target from this world's truth (geometric sim)")
    ap.add_argument("--target-out", default=None, help="v0.1.61: the target file (the sender's --target-file)")
    return ap.parse_args(argv)


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None) -> int:
    signal.signal(signal.SIGTERM, _sigterm)
    FakeCannonPiNode(parse_args(argv)).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
