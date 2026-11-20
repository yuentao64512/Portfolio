#!/usr/bin/env python3
"""cannon_sender: the water-cannon aiming link. Sends the aim as HMAC-signed UDP packets to the Raspberry Pi that drives the
cannon (HMAC-SHA256 over the sorted-keys JSON body, SENDER_STOPPED on exit). The contract: README.md beside this file.

v0.1.61 (WP12-rest): the packet is the students' `dock_color_aim.v1` (their receiver, pi/color_cannon_receiver.py, accepts nothing
else) and the target is MEASURED: the observer that holds the ZED writes the window it sees (`--cannon red` on cv/buoys_multi,
cv/docking, cv/sim_observer -> aim_cv.find_target -> runs/latest/cannon_target.json), this program sends it only while the mission's
aim is on and fresh (aim_core.AimGate), with the aim's bay / window / window_id added; `--color red` (default: the handbook's lit
window) | green (the students' bench target). Otherwise `target: null` with the reason as `status` (NO_AIM, AIM_OFF, AIM_STALE,
NO_TARGET) - their receiver then centres the servos with the pump off. `--protocol dock_cannon.v1` = the v0.1.57 packet (legacy).

v0.1.57 (WP12, D11): the source is the mission's aim, not the docking file. `--source aim` (default) subscribes
local/<v>/cannon/aim {on, bay, window, window_id, t} on the boat broker (published by the mission manager at 5 Hz while a task
aims, one `on: false` when it stops) and sends a packet every 1/hz s: the target (bay, window, window_id, plus the dock
observer's geometry when its status file is fresh) only while the last aim is `on` AND fresher than 1 s, else `target: null`.
The body carries `window_id` (null when no target). `--source file` = the legacy path (follows the docking status file,
sends what dock_observer saw, whether or not a task aims). `--key PATH` signs with another key file (the simulator's
generated test key, optiprimex/sim/fake_cannon_pi.py); the real key path stays in cannon_config.json.

Split out on 2026-10-29 so docking (perception, ../docking) and the cannon link can be tested on their own:

    cannon/run.sh --self-test          HMAC round trip + synthetic body (no network, no key needed)
    cannon/run.sh --synthetic-send     one signed packet with a synthetic target -> Pi receiver test (needs the key)
    cannon/run.sh --dry-run            follow the status file, print what would be sent, send nothing
    cannon/run.sh                      follow the aim topic and send (--source aim; needs paho-mqtt and the boat broker)
    cannon/run.sh --source file        legacy: follow the status file and send (dock_observer must be running)
    cannon_sender.py --key logs/.../cannon_test.key --destination 127.0.0.1:5005 --mqtt 127.0.0.1:1884   the simulator

Coordinate convention (unchanged): x_right_m positive to starboard, y_forward_m positive forward from the cannon
pivot; cannon_bearing_deg = atan2(x_right, y_forward). The HMAC key is read from the path in cannon_config.json at
start; it is never printed, logged or copied. Nothing here touches the FCU.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import os
import signal
import socket
import sys
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "cannon_config.json"
sys.path.insert(0, str(ROOT))
import aim_core as A  # noqa: E402  (v0.1.61: the gate, the target file, the dock_color_aim.v1 body; stdlib only)


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


# ---- packet format: verbatim from the students' dock_mapper_sender.py (the Pi verifies exactly this) -------------------
def signed_packet(body: dict, key: bytes) -> bytes:
    raw = json.dumps(body, separators=(",", ":"), sort_keys=True).encode()
    signature = hmac.new(key, raw, hashlib.sha256).hexdigest()
    return json.dumps({"body": body, "hmac_sha256": signature}, separators=(",", ":")).encode()


def verify_packet(packet: bytes, key: bytes) -> bool:
    envelope = json.loads(packet)
    raw = json.dumps(envelope["body"], separators=(",", ":"), sort_keys=True).encode()
    expected = hmac.new(key, raw, hashlib.sha256).hexdigest()
    return hmac.compare_digest(expected, envelope["hmac_sha256"])


def load_config(path) -> dict:
    cfg = json.loads(Path(path).read_text())
    missing = [k for k in ("water_cannon", "network") if k not in cfg]
    if missing:
        raise ValueError(f"Missing config sections: {missing}")
    return cfg


def load_key(cfg: dict, path=None, wait_s: float = 0.0) -> bytes:
    """The key file named by --key (path) or the config; wait_s: wait for it to appear (the sim's fake Pi writes it at start)."""
    f = Path(path or cfg["network"]["key_file"])
    t_end = time.monotonic() + wait_s
    while not f.exists() and time.monotonic() < t_end:
        time.sleep(0.1)
    key = f.read_text().strip().encode()
    if len(key) < 32:
        raise RuntimeError("HMAC key must contain at least 32 characters")
    return key


# ---- geometry: body frame (camera origin, camera yaw applied by dock_observer) -> cannon pivot ---------------------------
def cannon_frame(body_forward_m: float, body_right_m: float, cannon_cfg: dict) -> dict:
    y_forward = body_forward_m - float(cannon_cfg["forward_from_camera_m"])
    x_right = body_right_m - float(cannon_cfg["right_from_camera_m"])
    return {"y_forward_m": y_forward, "x_right_m": x_right, "cannon_bearing_deg": math.degrees(math.atan2(x_right, y_forward))}


def aim_target(observer_target: dict, cannon_cfg: dict) -> dict:
    """dock_observer target (body_* frames + geolocation) -> the dock_cannon.v1 target the Pi expects."""
    t = dict(observer_target)
    frames = cannon_frame(float(t["body_forward_m"]), float(t["body_right_m"]), cannon_cfg)
    t.update(frames)
    t.setdefault("class_name", "dock")
    t.setdefault("color", "red")
    return t


def make_synthetic_body(session: str, sequence: int) -> dict:
    """Identical to the students' synthetic packet (receiver test)."""
    return {
        "protocol": "dock_cannon.v1", "session": session, "sequence": sequence, "sent_unix_ns": time.time_ns(),
        "sent_utc": utc_now(), "source_host": socket.gethostname(), "status": "OK",
        "target": {
            "class_name": "dock", "color": "red", "confidence": 0.95, "red_fraction": 0.50, "range_m": 5.39,
            "x_right_m": -2.0, "y_forward_m": 5.0, "cannon_bearing_deg": math.degrees(math.atan2(-2.0, 5.0)),
            "vehicle_latitude_deg": 1.3, "vehicle_longitude_deg": 103.8, "vehicle_heading_deg": 0.0,
            "latitude_deg": 1.300045, "longitude_deg": 103.799982, "north_offset_m": 5.0, "east_offset_m": -2.0,
        },
    }


def make_synthetic_color_body(session: str, sequence: int, colour: str = "red") -> dict:
    """v0.1.61: the students' synthetic_target (color_dock_sender.py) in the colour aimed at - their receiver's test packet."""
    return {
        "protocol": A.PROTOCOL, "session": session, "sequence": sequence, "sent_unix_ns": time.time_ns(), "sent_utc": utc_now(),
        "source_host": socket.gethostname(), "status": "OK",
        "target": {
            "class_name": "dock", "color": colour, "confidence": 1.0, "color_fraction": 0.8,
            "x_right_m": -0.20, "y_forward_m": 5.0, "z_up_m": 0.10, "horizontal_m": math.hypot(0.20, 5.0),
            "range_m": math.sqrt(0.20 ** 2 + 5.0 ** 2 + 0.10 ** 2), "bearing_deg": math.degrees(math.atan2(-0.20, 5.0)),
            "elevation_deg": math.degrees(math.atan2(0.10, math.hypot(0.20, 5.0))),
        },
    }


def body_from_status(status: dict, session: str, sequence: int, cannon_cfg: dict) -> dict:
    """One dock_observer status -> one dock_cannon.v1 body (status and target carried over, aim added)."""
    target = status.get("target")
    ok = status.get("status") == "OK" and isinstance(target, dict) and "body_forward_m" in target
    return {
        "protocol": "dock_cannon.v1", "session": session, "sequence": sequence, "sent_unix_ns": time.time_ns(),
        "sent_utc": utc_now(), "source_host": socket.gethostname(),
        "status": status.get("status") if ok or status.get("status") else "NO_DOCK",
        "target": aim_target(target, cannon_cfg) if ok else None,
        "observer_sequence": status.get("sequence"), "observer_utc": status.get("sent_utc"),
    }


# ---- v0.1.57 (WP12, D11): the aim topic as the only source ----------------------------------------------------------------
AIM_FRESH_S = 1.0           # an aim older than this is no aim (the mission manager re-publishes at 5 Hz while on)


class AimFollower:
    """The last aim (local/<v>/cannon/aim) and the last fresh observer status -> one dock_cannon.v1 body per call of body(now).
    Pure: the caller gives the monotonic times, so the simulator (fast SIL) can drive it on its own clock."""

    def __init__(self, session: str, cannon_cfg: dict, fresh_s: float = AIM_FRESH_S, host: Optional[str] = None):
        self.session, self.cfg, self.fresh_s = session, cannon_cfg, float(fresh_s)
        self.host = host or socket.gethostname()
        self.aim: Optional[dict] = None
        self.aim_t = -1e18
        self.geo: Optional[dict] = None          # the dock observer's target (body_* frames), when its status file is fresh
        self.sequence = 0
        self.n_aims = 0

    def on_aim(self, body: dict, now: float) -> None:
        if isinstance(body, dict):
            self.aim, self.aim_t = body, now
            self.n_aims += 1

    def on_geometry(self, status: Optional[dict]) -> None:
        t = (status or {}).get("target")
        self.geo = t if (status or {}).get("status") == "OK" and isinstance(t, dict) and "body_forward_m" in t else None

    def state(self, now: float) -> str:
        if self.aim is None:
            return "NO_AIM"
        if now - self.aim_t > self.fresh_s:
            return "AIM_STALE"
        return "OK" if self.aim.get("on") else "AIM_OFF"

    def body(self, now: float) -> dict:
        st = self.state(now)
        target = None
        if st == "OK":
            a = self.aim
            target = aim_target(self.geo, self.cfg) if self.geo else {"class_name": "dock", "color": "red"}
            target.update(bay=a.get("bay"), window=a.get("window"), window_id=a.get("window_id"), geometry=bool(self.geo))
        self.sequence += 1
        return {"protocol": "dock_cannon.v1", "session": self.session, "sequence": self.sequence - 1, "sent_unix_ns": time.time_ns(),
                "sent_utc": utc_now(), "source_host": self.host, "status": st, "target": target,
                "window_id": target.get("window_id") if target else None, "aim_utc": (self.aim or {}).get("t")}


def run_aim(args, cfg: dict, dest, key: Optional[bytes], sock, session: str) -> int:
    """--source aim: the boat broker's local/<v>/cannon/aim -> a packet every 1/hz s (target null unless the aim is on and fresh)."""
    import paho.mqtt.client as mqtt                       # only this path needs it (the file path and --self-test stay stdlib)
    fol = AimFollower(session, cfg["water_cannon"], float(cfg.get("aim_fresh_s", AIM_FRESH_S)))
    topic = f"local/{args.vehicle}/cannon/aim"
    host, _, port = args.mqtt.partition(":")
    status_path = Path(args.status) if args.status else (ROOT / cfg.get("status_file", "../docking/runs/latest/latest_status.json")).resolve()
    stale_s = float(cfg.get("stale_after_s", 1.0))

    def on_message(c, u, msg):
        try:
            env = json.loads(msg.payload)
        except ValueError:
            return
        body = env.get("body", env) if isinstance(env, dict) else None
        fol.on_aim(body, time.monotonic())
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"cannon-sender-{os.getpid()}", protocol=mqtt.MQTTv311)
    c.on_connect = lambda cl, u, f, rc, p=None: cl.subscribe(topic, 0)
    c.on_message = on_message
    c.reconnect_delay_set(1, 2)
    c.connect_async(host, int(port or 1884), keepalive=15)
    c.loop_start()
    period = 1.0 / float(args.hz or cfg.get("aim_rate_hz", 5))
    print(f"[cannon_sender] aim {topic} on {host}:{port or 1884} -> {dest[0]}:{dest[1]}{' (dry run)' if args.dry_run else ''}"
          f" · geometry from {status_path} when fresh", flush=True)
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    last_sig, n_sent = None, 0
    try:
        while not stop["flag"]:
            try:
                fresh = time.time() - status_path.stat().st_mtime <= stale_s
                fol.on_geometry(json.loads(status_path.read_text()) if fresh else None)
            except (OSError, ValueError):
                fol.on_geometry(None)
            body = fol.body(time.monotonic())
            if sock:
                sock.sendto(signed_packet(body, key), dest)
            n_sent += 1
            sig = (body["status"], body["window_id"], (body["target"] or {}).get("bay"))
            if sig != last_sig or args.dry_run and n_sent % 25 == 0:
                print(f"seq={body['sequence']} status={body['status']} target="
                      + (f"bay {body['target']['bay']} window_id {body['window_id']}" if body["target"] else "None")
                      + f" dest={dest[0]}:{dest[1]}", flush=True)
                last_sig = sig
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        c.loop_stop()
        if sock:
            no_target = {"protocol": "dock_cannon.v1", "session": session, "sequence": fol.sequence, "sent_unix_ns": time.time_ns(),
                         "sent_utc": utc_now(), "source_host": socket.gethostname(), "status": "SENDER_STOPPED", "target": None, "window_id": None}
            try:
                sock.sendto(signed_packet(no_target, key), dest)
            except OSError:
                pass
            sock.close()
        print(f"[cannon_sender] stopped after {n_sent} packets", flush=True)
    return 0


def run_color_aim(args, cfg: dict, dest, key: Optional[bytes], sock, session: str) -> int:
    """v0.1.61: --source aim with dock_color_aim.v1 - the aim topic x the observer's target file -> a packet every 1/hz s."""
    import paho.mqtt.client as mqtt
    colour = args.color or cfg.get("aim_colour", "red")
    gate = A.AimGate(session, colour, float(cfg.get("aim_fresh_s", A.AIM_FRESH_S)),
                     float(args.target_fresh or cfg.get("target_fresh_s", A.TARGET_FRESH_S)))
    topic = f"local/{args.vehicle}/cannon/aim"
    host, _, port = args.mqtt.partition(":")
    target_path = Path(args.target_file) if args.target_file else (ROOT / cfg.get("target_file", "runs/latest/cannon_target.json")).resolve()

    def on_message(c, u, msg):
        try:
            env = json.loads(msg.payload)
        except ValueError:
            return
        gate.on_aim(env.get("body", env) if isinstance(env, dict) else None, time.monotonic())
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"cannon-sender-{os.getpid()}", protocol=mqtt.MQTTv311)
    c.on_connect = lambda cl, u, f, rc, p=None: cl.subscribe(topic, 0)
    c.on_message = on_message
    c.reconnect_delay_set(1, 2)
    c.connect_async(host, int(port or 1884), keepalive=15)
    c.loop_start()
    period = 1.0 / float(args.hz or cfg.get("aim_rate_hz", 5))
    print(f"[cannon_sender] {A.PROTOCOL} · colour {colour} · aim {topic} on {host}:{port or 1884} · target {target_path} -> "
          f"{dest[0]}:{dest[1]}{' (dry run)' if args.dry_run else ''}", flush=True)
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    last_sig, n_sent = None, 0
    try:
        while not stop["flag"]:
            gate.on_target(A.read_target(target_path))
            body = gate.body(time.monotonic())
            if sock:
                sock.sendto(signed_packet(body, key), dest)
            n_sent += 1
            t = body["target"]
            sig = (body["status"], body["window_id"], (t or {}).get("bay"), body["why"])
            if sig != last_sig or args.dry_run and n_sent % 25 == 0:
                print(f"seq={body['sequence']} status={body['status']} "
                      + (f"bay {t['bay']} window_id {t['window_id']} x={t['x_right_m']:+.2f} y={t['y_forward_m']:+.2f} z={t['z_up_m']:+.2f}"
                         if t else f"target=None ({body['why']})") + f" dest={dest[0]}:{dest[1]}", flush=True)
                last_sig = sig
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        c.loop_stop()
        if sock:
            no_target = dict(gate.body(time.monotonic()), status="SENDER_STOPPED", target=None, window_id=None, why="stopped")
            try:
                sock.sendto(signed_packet(no_target, key), dest)
            except OSError:
                pass
            sock.close()
        print(f"[cannon_sender] stopped after {n_sent} packets", flush=True)
    return 0


def run_self_test(cfg: dict) -> None:
    key = b"self-test-key-self-test-key-self-test-key"
    packet = signed_packet({"protocol": "dock_cannon.v1", "sequence": 1}, key)
    assert verify_packet(packet, key)
    assert not verify_packet(packet, b"another-key-another-key-another-key-xx")
    f = cannon_frame(10.0, -15.0, {"forward_from_camera_m": 0.0, "right_from_camera_m": 0.0})
    assert abs(f["x_right_m"] + 15.0) < 1e-9 and abs(f["y_forward_m"] - 10.0) < 1e-9 and f["cannon_bearing_deg"] < 0
    f2 = cannon_frame(10.0, 0.0, {"forward_from_camera_m": 2.0, "right_from_camera_m": -0.5})
    assert abs(f2["y_forward_m"] - 8.0) < 1e-9 and abs(f2["x_right_m"] - 0.5) < 1e-9
    body = body_from_status({"status": "OK", "sequence": 7, "sent_utc": utc_now(),
                             "target": {"body_forward_m": 10.0, "body_right_m": -15.0, "confidence": 0.9, "range_m": 18.0}}, "s", 1, cfg["water_cannon"])
    assert body["target"]["x_right_m"] < 0 and body["protocol"] == "dock_cannon.v1" and body["observer_sequence"] == 7
    fol = AimFollower("s", {"forward_from_camera_m": 0.0, "right_from_camera_m": 0.0}, host="h")
    assert fol.body(0.0)["target"] is None and fol.state(0.0) == "NO_AIM"
    fol.on_aim({"on": True, "bay": 2, "window": "upper", "window_id": 1}, 10.0)
    b = fol.body(10.5)
    assert b["status"] == "OK" and b["window_id"] == 1 and b["target"]["bay"] == 2 and verify_packet(signed_packet(b, key), key)
    assert fol.body(11.01)["target"] is None and fol.state(11.01) == "AIM_STALE"
    fol.on_aim({"on": False}, 11.2)
    assert fol.body(11.3)["target"] is None and fol.state(11.3) == "AIM_OFF"
    syn = make_synthetic_body("s", 0)
    assert verify_packet(signed_packet(syn, key), key)
    # v0.1.61: the gate x a target record -> dock_color_aim.v1
    g = A.AimGate("s", "red", host="h")
    rec = {"protocol": A.TARGET_FILE_PROTOCOL, "sent_unix_ns": 10_000_000_000, "status": "OK",
           "target": A.target_record("red", 0.9, 900, [0, 0, 30, 30], A.cannon_coordinates(2.0, 0.1, -0.2, A.MOUNT0))}
    g.on_target(rec)
    assert g.body(0.0, 10_000_000_000)["status"] == "NO_AIM"
    g.on_aim({"on": True, "bay": 2, "window_id": 1}, 5.0)
    b = g.body(5.1, 10_100_000_000)
    assert b["protocol"] == "dock_color_aim.v1" and b["status"] == "OK" and b["target"]["z_up_m"] > 0 and b["window_id"] == 1
    assert g.body(5.2, 10_900_000_000)["status"] == "NO_TARGET"          # the target file 0.9 s old
    assert A.AimGate("s", "green", host="h").target(10_000_000_000)[0] is None
    assert verify_packet(signed_packet(make_synthetic_color_body("s", 1), key), key)
    print(json.dumps({"self_test": "PASS", "left_example_x_right_m": f["x_right_m"], "forward_example_y_forward_m": f["y_forward_m"],
                      "bearing_deg": f["cannon_bearing_deg"]}))


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default=str(DEFAULT_CONFIG))
    p.add_argument("--status", default=None, help="dock_observer status file (default: config status_file, relative to this folder)")
    p.add_argument("--self-test", action="store_true")
    p.add_argument("--synthetic-send", action="store_true", help="send one authenticated synthetic packet for receiver testing")
    p.add_argument("--dry-run", action="store_true", help="print the bodies, send nothing (no key needed)")
    p.add_argument("--hz", type=float, default=None, help="--source file: poll rate of the status file (default: config max_rate_hz); "
                   "--source aim: the packet rate (default: config aim_rate_hz, 5)")
    p.add_argument("--source", choices=("aim", "file"), default="aim", help="v0.1.57: aim = local/<v>/cannon/aim on the boat broker "
                   "(default); file = the legacy path, the dock observer's status file")
    p.add_argument("--mqtt", default="127.0.0.1:1884", help="--source aim: the boat broker host[:port]")
    p.add_argument("--vehicle", default="usv", help="--source aim: local/<vehicle>/cannon/aim")
    p.add_argument("--key", default=None, help="the HMAC key file (default: the config's key_file); the simulator: the fake Pi's test key")
    p.add_argument("--key-wait", type=float, default=0.0, help="seconds to wait for the key file to appear (the sim's fake Pi writes it)")
    p.add_argument("--destination", default=None, help="override pi_ip[:port] (e.g. 127.0.0.1:5005 for a local receiver test)")
    p.add_argument("--protocol", choices=(A.PROTOCOL, A.LEGACY_PROTOCOL), default=A.PROTOCOL, help="v0.1.61: dock_color_aim.v1 = "
                   "the students' receiver (default); dock_cannon.v1 = the v0.1.57 packet (legacy)")
    p.add_argument("--color", choices=A.COLOURS, default=None, help="v0.1.61: the colour aimed at (default: config aim_colour, red - "
                   "the handbook's lit target window; green = the students' bench target)")
    p.add_argument("--target-file", default=None, help="v0.1.61: the observer's cannon_target.json (default: config target_file)")
    p.add_argument("--target-fresh", type=float, default=None, help="v0.1.61: the target file's age limit in s (default: config "
                   "target_fresh_s, 0.5; the sim's camera path 1.5 - its renderer is slower than the ZED)")
    return p.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
        return 0
    dest = (str(cfg["network"]["pi_ip"]), int(cfg["network"]["pi_port"]))
    if args.destination:
        host, _, port = args.destination.partition(":")
        dest = (host, int(port or dest[1]))
    key = None if args.dry_run else load_key(cfg, args.key, args.key_wait)
    sock = None if args.dry_run else socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    session = str(uuid.uuid4())
    if args.synthetic_send:
        body = (make_synthetic_color_body(session, 0, args.color or cfg.get("aim_colour", "red")) if args.protocol == A.PROTOCOL
                else make_synthetic_body(session, 0))
        if sock:
            sock.sendto(signed_packet(body, key), dest)
            sock.close()
        print(json.dumps({"synthetic_send": "PASS" if sock else "DRY", "destination": f"{dest[0]}:{dest[1]}", "body": body}))
        return 0
    if args.source == "aim":
        if args.protocol == A.PROTOCOL:
            return run_color_aim(args, cfg, dest, key, sock, session)
        return run_aim(args, cfg, dest, key, sock, session)
    status_path = Path(args.status) if args.status else (ROOT / cfg.get("status_file", "../docking/runs/latest/latest_status.json")).resolve()
    period = 1.0 / float(args.hz or cfg.get("max_rate_hz", 15))
    stale_s = float(cfg.get("stale_after_s", 1.0))
    print(f"[cannon_sender] status {status_path} -> {dest[0]}:{dest[1]}{' (dry run)' if args.dry_run else ''}", flush=True)
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    last_sig = None
    last_seen = None
    sequence = 0
    sent_stale = False
    try:
        while not stop["flag"]:
            try:
                st = status_path.stat()
                sig = (st.st_mtime_ns, st.st_size)
            except OSError:
                sig = None
            if sig is not None and sig != last_sig:
                last_sig = sig
                try:
                    status = json.loads(status_path.read_text())
                except (OSError, ValueError):
                    time.sleep(period)
                    continue
                body = body_from_status(status, session, sequence, cfg["water_cannon"])
                sequence += 1
                last_seen = time.monotonic()
                sent_stale = False
                if sock:
                    sock.sendto(signed_packet(body, key), dest)
                if sequence == 1 or sequence % 15 == 0 or args.dry_run:
                    t = body["target"]
                    print(f"seq={body['sequence']} status={body['status']} "
                          + (f"x={t['x_right_m']:+.2f} y={t['y_forward_m']:+.2f} brg={t['cannon_bearing_deg']:+.1f}" if t else "target=None")
                          + f" dest={dest[0]}:{dest[1]}", flush=True)
            elif last_seen is not None and not sent_stale and time.monotonic() - last_seen > stale_s:
                # the observer stopped writing: tell the Pi once that there is no target
                sent_stale = True
                body = {"protocol": "dock_cannon.v1", "session": session, "sequence": sequence, "sent_unix_ns": time.time_ns(),
                        "sent_utc": utc_now(), "source_host": socket.gethostname(), "status": "OBSERVER_STALE", "target": None}
                sequence += 1
                if sock:
                    sock.sendto(signed_packet(body, key), dest)
                print("observer status file stale -> OBSERVER_STALE sent", flush=True)
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        if sock:
            no_target = {"protocol": "dock_cannon.v1", "session": session, "sequence": sequence + 1, "sent_unix_ns": time.time_ns(),
                         "sent_utc": utc_now(), "source_host": socket.gethostname(), "status": "SENDER_STOPPED", "target": None}
            try:
                sock.sendto(signed_packet(no_target, key), dest)
            except OSError:
                pass
            sock.close()
        print(f"[cannon_sender] stopped after {sequence} packets", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
