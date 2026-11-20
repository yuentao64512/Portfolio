"""status_core - what the cannon's Pi did, as local/<v>/cannon/status (stdlib only; v0.1.61, WP12-rest).

The students' receiver (color_cannon_receiver.py beside this file, a verbatim snapshot) publishes nothing; it writes one JSON line
per event into color_receiver_logs/green_receiver_<stamp>.jsonl (and stdout). This module turns those lines into the status the
boat window and the OCS already read (v0.1.57's fields, plus what the receiver decided):

  ok, last_t, age_s, target, window_id, hmac_ok, n_rx, n_bad, status, sequence, session, key      (as v0.1.57)
  event, reason, mode, pump, ready, stable_packets, pan_deg, tilt_deg                              (new: the receiver's view)

  ok        the last packet the receiver ACCEPTED (AIM / SAFE_NO_TARGET / ...) is younger than 2 s and HMAC ok
  target    while the receiver aims (its last event AIM / DRY_RUN_READY_10S / PUMP_ON_GREEN): {color, pan_deg, tilt_deg, x_right_m,
            y_forward_m, z_up_m, window_id}; window_id from the aim topic (the receiver does not log it); null otherwise
  hmac_ok   false while a BAD_HMAC drop happened in the last 2 s; null before any packet
  event     the receiver's last event word; reason its reason (a DROP_INVALID_*'s, e.g. RANGE_OUT_OF_BOUNDS, WRONG_COLOR)
  mode      DRY_RUN | LIVE_SERVOS_PUMP_OFF | LIVE_PUMP_10S (from its AIM lines); pump ON | OFF; ready: a stable lock was reached

Two feeders: LogFollower (the real receiver's log, on the Pi by cannon_status_pub.py and in the simulator by fake_cannon_pi.py) and
ReceiverEmu (the receiver's per-packet decision on the sim clock, built from ITS OWN functions decode_verified / target_solution,
for the fast SIL). Nothing here fires or moves anything.
"""
from __future__ import annotations

import importlib.util
import json
from pathlib import Path
from typing import List, Optional

LINK_STALE_S = 2.0
HERE = Path(__file__).resolve().parent
RECEIVER = HERE / "color_cannon_receiver.py"
AIMING = ("AIM", "DRY_RUN_READY_10S", "PUMP_ON_GREEN")
ACCEPTED = AIMING + ("SAFE_NO_TARGET",)
DROPPED = ("DROP_WRONG_SOURCE",)                      # + every DROP_INVALID_* (the word carries the colour: ..._GREEN_PUMP_OFF)


def load_receiver(path=RECEIVER):
    """The students' receiver as a module (its top imports only the standard library; pigpio is imported in --live only)."""
    spec = importlib.util.spec_from_file_location("color_cannon_receiver_snapshot", str(path))
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def is_packet_event(ev: str) -> bool:
    return ev in ACCEPTED or ev in DROPPED or ev.startswith("DROP_INVALID")


class ReceiverStatus:
    def __init__(self, key_label: str = "real"):
        self.key_label = key_label
        self.n_rx = self.n_bad = 0
        self.last: Optional[dict] = None             # the last packet event
        self.accepted_t = -1e18
        self.last_utc: Optional[str] = None
        self.bad_hmac_t = -1e18
        self.hmac_seen = False
        self.mode: Optional[str] = None
        self.pump = "OFF"
        self.ready = False
        self.aim: Optional[dict] = None
        self.aim_t = -1e18

    def on_aim(self, body: dict, now: float) -> None:
        if isinstance(body, dict):
            self.aim, self.aim_t = body, now

    def on_event(self, ev: dict, now: float) -> None:
        word = str(ev.get("event", ""))
        if ev.get("pump") in ("ON", "OFF"):
            self.pump = ev["pump"]
        if ev.get("mode"):
            self.mode = ev["mode"]
        if word in ("DRY_RUN_READY_10S", "PUMP_ON_GREEN"):
            self.ready = True                                   # a stable lock (dry) / the pump started (live)
        elif word not in ("AIM", "PUMP_10S_COMPLETE_OFF"):
            self.ready = False                                  # no target, a drop, the watchdog: the lock is gone
        if not is_packet_event(word):
            if word in ("WATCHDOG_CENTER_PUMP_OFF", "GREEN_FEED_TIMEOUT_PUMP_OFF"):
                self.last = dict(ev, _watchdog=True)
            return
        self.n_rx += 1
        self.hmac_seen = True
        if word in ACCEPTED:
            self.accepted_t, self.last_utc = now, ev.get("utc")
        else:
            self.n_bad += 1
            if ev.get("reason") == "BAD_HMAC":
                self.bad_hmac_t = now
        self.last = ev

    def status(self, now: float) -> dict:
        ev = self.last or {}
        word = ev.get("event")
        age = None if self.last_utc is None else round(now - self.accepted_t, 2)
        h = None if not self.hmac_seen else bool(now - self.bad_hmac_t > LINK_STALE_S)
        aim_on = bool(self.aim and self.aim.get("on") and now - self.aim_t <= 1.0)
        tgt = None
        if word in AIMING and not ev.get("_watchdog"):
            tgt = {k: ev.get(k) for k in ("color", "pan_deg", "tilt_deg", "x_right_m", "y_forward_m", "z_up_m")}
            tgt["window_id"] = self.aim.get("window_id") if aim_on else None
        return {"ok": bool(h) and age is not None and age <= LINK_STALE_S, "last_t": self.last_utc, "age_s": age,
                "target": tgt, "window_id": tgt.get("window_id") if tgt else None, "hmac_ok": h, "n_rx": self.n_rx,
                "n_bad": self.n_bad, "status": "OK" if word in AIMING else ev.get("status"), "sequence": ev.get("sequence"),
                "session": None, "key": self.key_label, "event": word, "reason": ev.get("reason"), "mode": self.mode,
                "pump": self.pump, "ready": self.ready, "stable_packets": ev.get("stable_packets"),
                "pan_deg": ev.get("pan_deg") if tgt else None, "tilt_deg": ev.get("tilt_deg") if tgt else None}


class LogFollower:
    """Tails the newest green_receiver_*.jsonl in the receiver's log folder (a new file = the receiver restarted)."""

    def __init__(self, log_dir):
        self.dir = Path(log_dir)
        self.path: Optional[Path] = None
        self.pos = 0
        self.buf = ""

    def poll(self) -> List[dict]:
        try:
            files = sorted(self.dir.glob("*.jsonl"), key=lambda p: p.stat().st_mtime)
        except OSError:
            return []
        if not files:
            return []
        if files[-1] != self.path:
            self.path, self.pos, self.buf = files[-1], 0, ""
        try:
            with self.path.open() as f:
                f.seek(self.pos)
                chunk = f.read()
                self.pos = f.tell()
        except OSError:
            return []
        self.buf += chunk
        lines, self.buf = self.buf.split("\n")[:-1], self.buf.split("\n")[-1]
        out = []
        for line in lines:
            try:
                ev = json.loads(line)
            except ValueError:
                continue
            if isinstance(ev, dict) and "event" in ev:
                out.append(ev)
        return out


class ReceiverEmu:
    """The receiver's per-packet decision in DRY_RUN (no hardware), on the caller's clock, from the snapshot's own functions:
    decode_verified (HMAC), target_solution (class / colour / confidence / fraction / range / forward checks, pan / tilt, the servo
    limits); the protocol, age, replay and stability rules as its main loop has them. step() -> the events it would log."""

    def __init__(self, cfg: dict, key: bytes, rx=None, sender_ip: str = "127.0.0.1"):
        self.rx = rx or load_receiver()
        self.cfg, self.key, self.sender_ip = cfg, key, sender_ip
        self.last_session, self.last_sequence = None, -1
        self.last_pan = self.last_tilt = None
        self.stable_packets, self.stable_since = 0, 0.0

    def _reset_lock(self):
        self.last_pan = self.last_tilt = None
        self.stable_packets, self.stable_since = 0, 0.0

    def step(self, packet: bytes, now: float, now_ns: int, source_ip: Optional[str] = None) -> List[dict]:
        cfg = self.cfg
        if (source_ip or self.sender_ip) != str(cfg["network"]["allowed_sender_ip"]):
            return [{"event": "DROP_WRONG_SOURCE", "source": source_ip}]
        try:
            body = self.rx.decode_verified(packet, self.key)
            if body.get("protocol") != "dock_color_aim.v1":
                raise ValueError("WRONG_PROTOCOL")
            age_s = (now_ns - int(body["sent_unix_ns"])) / 1e9
            if age_s < -0.5 or age_s > float(cfg["network"]["maximum_packet_age_s"]):
                raise ValueError("STALE_PACKET")
            session, sequence = str(body["session"]), int(body["sequence"])
            if session == self.last_session and sequence <= self.last_sequence:
                raise ValueError("REPLAY_OR_OUT_OF_ORDER")
            if session != self.last_session:
                self.last_session, self.last_sequence = session, -1
                self._reset_lock()
            self.last_sequence = sequence
            if body.get("status") != "OK" or not body.get("target"):
                self._reset_lock()
                return [{"event": "SAFE_NO_TARGET", "sequence": sequence, "status": body.get("status"), "pump": "OFF"}]
            sol = self.rx.target_solution(body["target"], cfg)
            delta = float(cfg["tracking"]["stable_angle_delta_deg"])
            if (self.last_pan is None or abs(sol["pan_deg"] - self.last_pan) > delta or abs(sol["tilt_deg"] - self.last_tilt) > delta):
                self.stable_packets, self.stable_since = 1, now
            else:
                self.stable_packets += 1
            self.last_pan, self.last_tilt = sol["pan_deg"], sol["tilt_deg"]
            stable = (self.stable_packets >= int(cfg["tracking"]["stable_packets_required"])
                      and now - self.stable_since >= float(cfg["tracking"]["settle_time_s"]))
            ev = {"event": "DRY_RUN_READY_10S" if stable else "AIM", "sequence": sequence, "mode": "DRY_RUN", "pump": "OFF",
                  "color": sol["color"], "stable_packets": self.stable_packets}
            ev.update({k: round(sol[k], 3) for k in ("x_right_m", "y_forward_m", "z_up_m", "pan_deg", "tilt_deg")})
            ev.update(pan_pulse_us=sol["pan_pulse_us"], tilt_pulse_us=sol["tilt_pulse_us"])
            return [ev]
        except Exception as exc:                                # their loop: any failure -> pump off, lock reset, logged
            self._reset_lock()
            return [{"event": "DROP_INVALID_GREEN_PUMP_OFF", "reason": str(exc), "pump": "OFF"}]
