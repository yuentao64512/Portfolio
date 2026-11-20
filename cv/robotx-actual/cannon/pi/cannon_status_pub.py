#!/usr/bin/env python3
"""cannon_status_pub - runs on the cannon's Raspberry Pi BESIDE the students' receiver (v0.1.61, WP12-rest; decided with Dmitrii
2026-11-07). Their color_cannon_receiver.py publishes nothing; this reads its JSONL event log (read-only, the receiver is not
touched) and publishes local/<v>/cannon/status at 2 Hz on the boat broker, so the boat window's WATER CANNON callout and the OCS
SYSTEM row show what the Pi did: the link (ok, hmac_ok, n_rx / n_bad), the receiver's last event and reason, the mode, the pump,
the servo angles, and whether a stable lock was reached. It subscribes local/<v>/cannon/aim only to name the window it aims at.

  python3 cannon_status_pub.py --log-dir ~/cannon/color_receiver_logs --mqtt 192.168.6.x:1884        (the NX boat broker)

Needs paho-mqtt (pip install paho-mqtt>=2) and a route from the Pi to the boat broker. Fields: status_core.py. Read-only: it sends
nothing to the receiver, the servos or the pump.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import socket
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import status_core as S  # noqa: E402

PERIOD_S = 0.5


def envelope(body: dict, seq: int, source: str, vehicle_id: str) -> dict:
    return {"schema_version": 1, "msg_id": f"{source}-{seq:010d}", "seq": seq, "sent_at": datetime.now(timezone.utc).isoformat(),
            "run_id": None, "declaration_seq": None, "vehicle_id": vehicle_id, "kind": "local", "source": source, "body": body}


class Publisher:
    """The loop shared with the simulator's fake_cannon_pi: poll the log, feed the status, publish every 0.5 s."""

    def __init__(self, client, log_dir, vehicle: str = "usv", vehicle_id: str = "USV1", key_label: str = "real",
                 source: str = "cannon_pi", log=print):
        self.c, self.vehicle, self.vehicle_id, self.source, self.log = client, vehicle, vehicle_id, source, log
        self.follower = S.LogFollower(log_dir)
        self.status = S.ReceiverStatus(key_label)
        self.topic = f"local/{vehicle}/cannon/status"
        self.aim_topic = f"local/{vehicle}/cannon/aim"
        self.seq = 0
        self.last_pub = 0.0
        self.last_sig = None

    def on_aim_message(self, payload: bytes) -> None:
        try:
            env = json.loads(payload)
        except ValueError:
            return
        self.status.on_aim(env.get("body", env) if isinstance(env, dict) else None, time.monotonic())

    def tick(self) -> dict:
        now = time.monotonic()
        for ev in self.follower.poll():
            self.status.on_event(ev, now)
        st = self.status.status(now)
        sig = (st["ok"], st["event"], st["reason"], st["ready"], st["pump"])
        if sig != self.last_sig:
            self.log(f"[{self.source}] link {'ok' if st['ok'] else 'NOT ok'} · receiver {st['event']}"
                     + (f" ({st['reason']})" if st["reason"] else "") + f" · mode {st['mode']} · pump {st['pump']}"
                     + (" · READY" if st["ready"] else "") + f" · n_rx {st['n_rx']} n_bad {st['n_bad']}")
            self.last_sig = sig
        if now - self.last_pub >= PERIOD_S:
            self.seq += 1
            self.c.publish(self.topic, json.dumps(envelope(st, self.seq, self.source, self.vehicle_id)), qos=0)
            self.last_pub = now
        return st


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log-dir", default=str(Path(__file__).resolve().parent / "color_receiver_logs"),
                    help="the receiver's log folder (color_receiver_logs/ beside color_cannon_receiver.py)")
    ap.add_argument("--mqtt", default="127.0.0.1:1884", help="the boat broker host[:port]")
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--key-label", default="real", help="what `key` says in the status (the sim: test)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    import paho.mqtt.client as mqtt
    a = parse_args(argv)
    host, _, port = a.mqtt.partition(":")
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"cannon-pi-{socket.gethostname()}-{os.getpid()}", protocol=mqtt.MQTTv311)
    pub = Publisher(c, a.log_dir, a.vehicle, a.vehicle_id, a.key_label, log=lambda s: print(s, flush=True))
    c.on_connect = lambda cl, u, f, rc, p=None: cl.subscribe(pub.aim_topic, 0)
    c.on_message = lambda cl, u, msg: pub.on_aim_message(msg.payload)
    c.reconnect_delay_set(1, 5)
    c.connect_async(host, int(port or 1884), keepalive=15)
    c.loop_start()
    print(f"[cannon_pi] following {a.log_dir} -> {pub.topic} on {host}:{port or 1884} at 2 Hz", flush=True)
    stop = {"flag": False}
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("flag", True))
    try:
        while not stop["flag"]:
            pub.tick()
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        c.loop_stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
