#!/usr/bin/env python3
"""fake_dock: the Task 3 dock controller of the simulated pool (ROADMAP WP1; handbook 3.3.4 core tier).

Keeps the dock's light state for sim/fake_perception: which bay shows the GREEN indicator (the others RED) and the
target window. Sequence: idle -> the boat docks in the GREEN bay -> one of that bay's two windows turns RED ("lit") ->
the cannon is aimed at it (local/<v>/cannon/aim {"on": true, "bay": n, "window_id": w}) for fire_on_target_s -> GREEN for green_hold_s
-> off ("done"). Docked = the vehicle's DockingReport event for the GREEN bay (event/<v>/docking: m3t1 sends it, v0.1.22; a report
for another bay lights nothing), or `{"cmd": "docked"}` on sim/dock/cmd, or - with auto on - inside the GREEN bay's box for
docked_dwell_s while slower than 0.3 m/s. Retained sim/dock/state on the boat broker (fake_perception applies it, map.html shows it via sim/truth).

v0.1.48 (WP10j-b, docs/M3T1_SIM_PLAN.md 1.3 / 2.4): auto (the dwell) is the world's `dock.auto` (false in the sim worlds: the window
lights on our DockingReport, as the judges do), `--auto on|off` overrides it (the bench's usv_ui `dockto` sends no report). An aim that
names a window (`window_id` present) turns it GREEN only when it is the lit window (the handbook's "correct window"); an aim without
the key (a hand-made one) is matched on the bay alone, as before.

  sim/dock/cmd   {"cmd": "reset"} | {"cmd": "green_bay", "bay": 1..3} | {"cmd": "docked"} | {"cmd": "fire"}
                 | {"cmd": "auto", "on": false} (docked only by command)

  op-fake-dock --world profiles/bench/world_pool.yaml --port 1884 [--host H]

Nothing here commands the boat; it only reads mm/state for the pose.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import signal
import sys
import threading
import time
from typing import Optional

import paho.mqtt.client as mqtt

from ..companion.mqtt import start_presence
from ..companion.state import utc_iso
from . import world as W

AIM_FRESH_S = 1.0


class DockSim:
    """The light sequence as a pure state machine: tick(now, pose, aim) and command(msg); state() for the wire."""

    def __init__(self, world: dict, seed: int = 1, auto: Optional[bool] = None):
        self.objects = world.get("objects") or []
        d = world.get("dock") or {}
        self.green_bay = int(d.get("green_bay", 2))
        self.dwell_s = float(d.get("docked_dwell_s", 3.0))
        self.depth_m = float(d.get("bay_depth_m", 3.5))
        self.width_m = float(d.get("bay_width_m", 3.5))
        self.fire_s = float(d.get("fire_on_target_s", 4.0))
        self.hold_s = float(d.get("green_hold_s", 5.0))
        st = world.get("start") or {}
        self.normal = W.dock_normal(self.objects, (float(st.get("x_n", 0.0)), float(st.get("y_e", 0.0))) if st else None)
        self.rng = random.Random(seed)
        self.auto = bool(d.get("auto", False)) if auto is None else bool(auto)     # v0.1.48: the report lights it, not the dwell
        self.reset(0.0)

    def reset(self, now: float) -> None:
        self.phase = "idle"                 # idle | lit (window RED) | green | done
        self.window: Optional[int] = None
        self.docked = False
        self.inside_since: Optional[float] = None
        self.on_target_s = 0.0
        self.t_phase = now
        self.last_tick: Optional[float] = None
        self.last_pose: Optional[dict] = None
        self.speed = 0.0

    def faces(self):
        return W.bay_faces(self.objects)

    def inside(self, pose: Optional[dict], bay: Optional[int] = None) -> bool:
        face = self.faces().get(self.green_bay if bay is None else bay)
        return bool(pose and face and W.in_bay(pose, face, self.depth_m, self.width_m, self.normal))

    def command(self, msg: dict, now: float) -> str:
        c = str(msg.get("cmd", "")).lower()
        if c == "reset":
            self.reset(now)
        elif c == "green_bay":
            bay = int(msg["bay"])
            if bay not in self.faces():
                raise ValueError(f"no bay {bay} in this world")
            self.green_bay = bay
            self.reset(now)
        elif c == "docked":
            bay = msg.get("bay")
            if bay is not None and int(bay) != self.green_bay:
                return "docked ignored (bay %s is not the GREEN bay %d)" % (bay, self.green_bay)   # the judge would not light a RED bay
            self._dock(now)
        elif c == "fire":
            if self.phase == "idle":
                self._dock(now)
            self._go("green", now)
        elif c == "auto":
            self.auto = bool(msg.get("on", True))
        else:
            raise ValueError(f"unknown dock cmd {c!r}")
        return c

    def _dock(self, now: float) -> None:
        if self.phase == "idle":
            self.docked = True
            self.window = self.rng.choice([1, 2])
            self._go("lit", now)

    def _go(self, phase: str, now: float) -> None:
        self.phase, self.t_phase = phase, now
        self.on_target_s = 0.0

    def tick(self, now: float, pose: Optional[dict], aim: Optional[dict] = None, aim_age_s: float = 1e9) -> None:
        dt = 0.0 if self.last_tick is None else max(0.0, min(1.0, now - self.last_tick))
        self.last_tick = now
        if pose is not None and self.last_pose is not None and dt > 0:
            self.speed = math.hypot(pose["x_n"] - self.last_pose["x_n"], pose["y_e"] - self.last_pose["y_e"]) / dt
        if pose is not None:
            self.last_pose = pose
        if self.phase == "idle" and self.auto:
            if self.inside(pose) and self.speed < 0.3:
                self.inside_since = self.inside_since if self.inside_since is not None else now
                if now - self.inside_since >= self.dwell_s:
                    self._dock(now)
            else:
                self.inside_since = None
        elif self.phase == "lit":
            aimed = bool(aim and aim.get("on") and aim_age_s <= AIM_FRESH_S
                         and (aim.get("bay") is None or int(aim["bay"]) == self.green_bay)
                         and ("window_id" not in aim or (aim["window_id"] is not None and int(aim["window_id"]) == self.window)))
            self.on_target_s = self.on_target_s + dt if aimed else 0.0
            if self.on_target_s >= self.fire_s:
                self._go("green", now)
        elif self.phase == "green" and now - self.t_phase >= self.hold_s:
            self._go("done", now)

    def state(self, now: float) -> dict:
        colour = {"lit": "red", "green": "green"}.get(self.phase, "black")
        return {"green_bay": self.green_bay, "phase": self.phase, "docked": self.docked, "auto": self.auto,
                "window": {"bay": self.green_bay if self.window else None, "window": self.window, "colour": colour},
                "on_target_s": round(self.on_target_s, 1), "in_phase_s": round(now - self.t_phase, 1),
                "inside_green_bay": self.inside(self.last_pose), "t": utc_iso()}


class FakeDockNode:
    def __init__(self, a):
        self.a = a
        self.world = W.load(a.world)
        self.frame = W.WorldFrame(self.world.get("origin"))
        self.sim = DockSim(self.world, a.seed, {"on": True, "off": False}.get(getattr(a, "auto", "world")))
        self.ns = f"local/{a.vehicle}"
        self.lock = threading.Lock()
        self.pose: Optional[dict] = None
        self.aim: Optional[dict] = None
        self.aim_t = -1e9
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"fake-dock-{a.vehicle}", protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_message = self._on_message
        self.c.reconnect_delay_set(1, 2)
        self.seq = 0
        self.last_sig = None

    def log(self, text: str) -> None:
        print(f"[fake_dock] {text}", flush=True)

    def _on_connect(self, c, u, f, rc, p=None):
        for t in (f"{self.ns}/mm/state", f"{self.ns}/cannon/aim", "sim/dock/cmd", f"team/+/+/event/{self.a.vehicle}/docking"):
            c.subscribe(t, 0)
        self.log(f"connected; dock {sorted(self.sim.faces())} GREEN bay {self.sim.green_bay}")

    def _on_message(self, c, u, msg):
        try:
            env = json.loads(msg.payload)
        except Exception:
            return
        body = env.get("body", env) if isinstance(env, dict) else None
        if not isinstance(body, dict):
            return
        now = time.monotonic()
        with self.lock:
            if msg.topic.endswith("/mm/state"):
                self.pose = W.pose_from_state(body, self.frame)
            elif msg.topic.endswith("/cannon/aim"):
                self.aim, self.aim_t = body, now
            elif msg.topic.endswith("/docking"):          # v0.1.22: the task's DockingReport lights the window (handbook 3.3.4)
                self.log(f"DockingReport heard (bay {body.get('bay')}) -> {self.sim.command({'cmd': 'docked', 'bay': body.get('bay')}, now)}")
            elif msg.topic == "sim/dock/cmd":
                try:
                    self.log(f"cmd {self.sim.command(body, now)} -> {self.sim.phase}")
                except (ValueError, KeyError) as e:
                    self.log(f"bad cmd {body}: {e}")

    def publish(self, now: float) -> None:
        st = self.sim.state(now)
        sig = (st["green_bay"], st["phase"], st["window"]["window"], st["auto"])
        if sig != self.last_sig:
            self.log(f"state: GREEN bay {st['green_bay']} phase {st['phase']} window {st['window']}")
            self.last_sig = sig
        self.seq += 1
        env = {"schema_version": 1, "msg_id": f"fd-{self.seq:010d}", "seq": self.seq, "sent_at": utc_iso(), "run_id": None,
               "declaration_seq": None, "vehicle_id": self.a.vehicle_id, "kind": "local", "source": "fake_dock", "body": st}
        self.c.publish("sim/dock/state", json.dumps(env), qos=0, retain=True)

    def run(self) -> None:
        self.c.connect_async(self.a.host, self.a.port, keepalive=15)
        self.c.loop_start()
        stop_presence = start_presence(self.c, "fake_dock", f"{self.a.host}:{self.a.port}")
        self.log(f"world {self.a.world}; listening to {self.ns}/mm/state, {self.ns}/cannon/aim, sim/dock/cmd")
        last_pub = 0.0
        try:
            while True:
                now = time.monotonic()
                with self.lock:
                    self.sim.tick(now, self.pose, self.aim, now - self.aim_t)
                    changed = self.last_sig is None or (self.sim.phase, self.sim.window) != self.last_sig[1:3]
                    if changed or now - last_pub >= 0.5:
                        self.publish(now)
                        last_pub = now
                time.sleep(0.1)
        except KeyboardInterrupt:
            pass
        finally:
            stop_presence()
            self.c.loop_stop()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="profiles/bench/world_pool.yaml")
    ap.add_argument("--host", default="127.0.0.1", help="boat broker host")
    ap.add_argument("--port", type=int, default=1884, help="boat broker")
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--auto", choices=("world", "on", "off"), default="world", help="v0.1.48: docked by the dwell in the GREEN bay too "
                    "(on) or only by the DockingReport / sim/dock/cmd (off); world = the world file's dock.auto (false in the sim worlds)")
    return ap.parse_args(argv)


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None) -> int:
    signal.signal(signal.SIGTERM, _sigterm)
    FakeDockNode(parse_args(argv)).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
