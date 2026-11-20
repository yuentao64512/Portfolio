#!/usr/bin/env python3
"""vcmd: publish a vehicle command directly on cmd/<v>/<kind> and wait for its acks (B1 bench tool).

This bypasses the OCS command authority on purpose: it exists to test the vehicle alone. On the course
every command comes from ocs_app (ADR-15). Examples:

  vcmd prepare --seq 1                 vcmd enable_auto            vcmd start_run --run 1001
  vcmd start_task GOTO --x 20 --y 5    vcmd hold / resume / abort  vcmd confirm_safe
  vcmd incident assistance --lat 1.28112 --lon 103.85503 --cseq 7
  vcmd incident confirm --cseq 7       vcmd stop_run
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import uuid
from typing import List, Optional

import paho.mqtt.client as mqtt

from .. import TEAM_NS
from ..companion.state import utc_iso


class VehicleCommander:
    def __init__(self, host: str = "127.0.0.1", port: int = 1882, vehicle: str = "usv", vehicle_id: str = "USV1",
                 ns: str = TEAM_NS, client_id: Optional[str] = None):
        self.host, self.port, self.vehicle, self.vehicle_id, self.ns = host, port, vehicle, vehicle_id, ns
        self.run_id: Optional[int] = None
        self.declaration_seq: Optional[int] = None
        self.acks: List[dict] = []
        self._ack_evt = threading.Event()
        self._seq = 0
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id or f"vcmd-{uuid.uuid4().hex[:6]}",
                             protocol=mqtt.MQTTv311)
        self.c.on_message = self._on_msg
        self._connected = threading.Event()
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = lambda *a, **k: self._connected.clear()
        self.c.reconnect_delay_set(1, 2)

    def _on_connect(self, c, u, f, rc, p=None):
        c.subscribe(f"{self.ns}/ack/{self.vehicle}", 1)
        self._connected.set()

    def start(self) -> None:
        self.c.connect(self.host, self.port, keepalive=15)
        self.c.loop_start()
        time.sleep(0.2)

    def stop(self) -> None:
        self.c.loop_stop()
        self.c.disconnect()

    def _on_msg(self, client, userdata, msg):
        try:
            env = json.loads(msg.payload)
        except Exception:
            return
        self.acks.append(env)
        self._ack_evt.set()

    def send(self, kind: str, action: str, task: Optional[str] = None, args: Optional[dict] = None,
             wait_s: float = 2.0, msg_id: Optional[str] = None, run_id=None, declaration_seq=None) -> List[dict]:
        self._seq += 1
        env = {"schema_version": 1, "msg_id": msg_id or str(uuid.uuid4()), "seq": self._seq, "sent_at": utc_iso(),
               "run_id": self.run_id if run_id is None else run_id,
               "declaration_seq": self.declaration_seq if declaration_seq is None else declaration_seq,
               "vehicle_id": self.vehicle_id, "kind": "cmd", "source": "vcmd",
               "body": {"action": action, "task": task, "args": args or {}}}
        topic = f"{self.ns}/cmd/{self.vehicle}/{kind}"
        n0 = len(self.acks)
        if not self._connected.wait(10.0):
            raise RuntimeError("vcmd: broker not reachable")
        self.c.publish(topic, json.dumps(env), qos=1).wait_for_publish(2.0)
        t0 = time.monotonic()
        got: List[dict] = []
        while time.monotonic() - t0 < wait_s:
            self._ack_evt.wait(0.1)
            self._ack_evt.clear()
            got = [a for a in self.acks[n0:] if a["body"]["ack_of"] == env["msg_id"]]
            if any(a["body"]["result"] in ("DONE", "FAILED", "REJECTED") for a in got):
                break
        return got


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1882)
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--run", type=int, default=None, help="run_id to stamp on the envelope")
    ap.add_argument("--seq", type=int, default=None, help="declaration_seq to stamp on the envelope")
    ap.add_argument("--wait", type=float, default=2.0)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("prepare")
    sub.add_parser("enable_auto"); sub.add_parser("disable_auto"); sub.add_parser("hold"); sub.add_parser("resume")
    sub.add_parser("abort"); sub.add_parser("confirm_safe"); sub.add_parser("stop_run")
    p = sub.add_parser("start_run"); p.add_argument("--run-id", type=int, required=True)
    p = sub.add_parser("start_task"); p.add_argument("task"); p.add_argument("--lat", type=float); p.add_argument("--lon", type=float)
    p.add_argument("--x", type=float); p.add_argument("--y", type=float); p.add_argument("--speed", type=float)
    p = sub.add_parser("incident"); p.add_argument("kind", choices=["assistance", "confirm", "keep_out", "all_clear", "moving"])
    p.add_argument("--lat", type=float); p.add_argument("--lon", type=float); p.add_argument("--cseq", type=int, default=1)
    a = ap.parse_args(argv)

    vc = VehicleCommander(a.host, a.port, a.vehicle)
    vc.run_id, vc.declaration_seq = a.run, a.seq
    vc.start()
    kind, action, task, args = "run", a.cmd.upper(), None, {}
    if a.cmd == "prepare":
        args = {"declaration_seq": a.seq if a.seq is not None else 1, "tiers": {"T1": "TIER_CORE", "T3": "TIER_CORE", "T4": "TIER_CORE"}}
        vc.declaration_seq = args["declaration_seq"]
    elif a.cmd == "start_run":
        args = {"run_id": a.run_id}
    elif a.cmd == "start_task":
        task = a.task
        if a.lat is not None:
            args = {"lat": a.lat, "lon": a.lon}
        elif a.x is not None:
            args = {"x_n": a.x, "y_e": a.y}
        if a.speed:
            args["speed_mps"] = a.speed
    elif a.cmd == "abort":
        action = "ABORT_TASK"
    elif a.cmd == "incident":
        kind = "incident"
        action = "INCIDENT"
        kmap = {"assistance": "assistance_request", "confirm": "readiness_confirm", "keep_out": "keep_out_zone",
                "all_clear": "all_clear", "moving": "moving_object"}
        args = {"kind": kmap[a.kind], "command_seq": a.cseq}
        if a.lat is not None:
            args["position"] = {"lat": a.lat, "lon": a.lon}
            args["domain"] = "surface"
    acks = vc.send(kind, action, task, args, wait_s=a.wait)
    for ack in acks:
        b = ack["body"]
        print(f"{b['result']:9s} state={b.get('state')} {b.get('reason', '')}")
    vc.stop()
    return 0 if acks and acks[-1]["body"]["result"] in ("ACCEPTED", "DONE") else 1


if __name__ == "__main__":
    sys.exit(main())
