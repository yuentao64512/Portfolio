#!/usr/bin/env python3
"""fake_perception: a detector stand-in fed by a world file (decision 27, stage 1; ROADMAP WP1 sim CV).

Reads the boat pose from local/<v>/mm/state on the boat broker (5 Hz, what the mission manager publishes), places it in
the world file's frame by its lat/lon (profiles/bench/world_*.yaml) and publishes what a camera on the boat would report:
body-frame range / bearing per object inside the field of view and range limits, with noise, misses and wrong colours,
on local/<v>/perception/buoys, /dock and /window (ARCHITECTURE 5.4 Observation), plus local/<v>/perception/health at
1 Hz with `source: sim` and the world's name. Buoy lights follow the handbook (flash 1 s on / 1 s off -> black in the
off phase; `light: off` = black obstacle); the dock indicators and windows follow sim/dock/state (sim/fake_dock.py).
It also publishes the world as retained sim/truth on the hub (map.html's truth layer) and clears it on exit (v0.1.28.1).

  op-fake-perception --world profiles/bench/world_pool.yaml --port 1884 [--host H] [--hub-host H --hub-port 1882] [--hz 5]

On the AGX (scripts/run_agx.sh --sim) it replaces cv_publish: same topics, same Observation bodies. It never sees the map
and never commands anything: the mission manager's PerceptionMap does the geolocation (D1).
"""
from __future__ import annotations

import argparse
import json
import math
import random
import signal
import socket
import sys
import threading
import time
from typing import Dict, List, Optional

import paho.mqtt.client as mqtt

from ..companion.frames import wrap180
from ..companion.mqtt import start_presence
from ..companion.state import Observation, ObsItem, utc_iso
from . import world as W

TOPIC_OF_KIND = {"buoy": "buoys", "dock": "dock", "window": "window", "box": "boxes"}


def _led(cam: dict, colour: str, has_beacon: bool, q: dict):
    """v0.1.94 (real_13nov, `camera.emit_led`): the observer's per-sighting lamp read as cv_publish passes it (quality.led {on, now});
    with `camera.body_colour` the item's colour is the body's ("white", as the 13 Nov observer reported every course buoy) so only the
    lamp carries the colour. Off at the defaults: the item is unchanged."""
    if not cam.get("emit_led"):
        return colour, q
    on = bool(has_beacon and colour in ("red", "green", "blue"))
    q = dict(q, led={"on": on, "now": colour if on else None, "pattern": "unknown", "colour": None})
    return (cam.get("body_colour") if cam.get("body_colour") else colour), q


def _interp(table, r: float) -> float:
    """v0.1.94: a piecewise-linear [[range_m, p], ...] table (clamped at its ends)."""
    t = sorted((float(a), float(b)) for a, b in table)
    if r <= t[0][0]:
        return t[0][1]
    for (r0, p0), (r1, p1) in zip(t, t[1:]):
        if r <= r1:
            return p0 + (p1 - p0) * (r - r0) / max(1e-9, r1 - r0)
    return t[-1][1]


class FakePerception:
    def __init__(self, a):
        self.a = a
        self.world = W.load(a.world)
        self.name = str(self.world.get("name"))
        self.cam = self.world.get("camera") or {}
        skip = {k.strip() for k in (getattr(a, "skip_kinds", "") or "").split(",") if k.strip()}      # v0.1.36: buoys via the camera path
        self.objects: List[dict] = [o for o in self.world.get("objects") or [] if o.get("sensed", True) is not False and o.get("kind", "buoy") not in skip]
        self.all_objects: List[dict] = list(self.world.get("objects") or [])
        self.frame = W.WorldFrame(self.world.get("origin"))
        self.phases = W.light_phases(self.all_objects, a.seed)
        self.dock_state: Optional[dict] = None
        self.pose: Optional[dict] = None
        self.pose_t = 0.0
        self.lock = threading.Lock()
        self.rng = random.Random(a.seed)
        self.rng_r = random.Random(a.seed ^ 0x13E1)   # v0.1.94: the real_13nov knobs' own stream (unused at the defaults)
        self.rng_b = random.Random(a.seed ^ 0x5EED)   # v0.1.34: the bearing-only band's own stream (it must not shift the ranged noise)
        self.n_obs = 0
        self.ns = f"local/{a.vehicle}"
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"fake-perception-{a.vehicle}", protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_message = self._on_message
        self.c.reconnect_delay_set(1, 2)
        self.hub: Optional[mqtt.Client] = None
        if a.hub_port:
            self.hub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"fake-perception-truth-{a.vehicle}", protocol=mqtt.MQTTv311)
            self.hub.reconnect_delay_set(1, 2)
            self.hub.on_connect = lambda c, u, f, rc, p=None: self.publish_truth()
        self.seq = 0

    # ---- MQTT -------------------------------------------------------------------------------------------------------
    def _on_connect(self, c, u, f, rc, p=None):
        c.subscribe(f"{self.ns}/mm/state", 0)
        c.subscribe("sim/dock/state", 0)
        self.log("connected, subscribed to " + f"{self.ns}/mm/state, sim/dock/state")
        if self.hub is None:
            self.publish_truth()

    def _on_message(self, c, u, msg):
        try:
            body = json.loads(msg.payload).get("body") or {}
        except Exception:
            return
        if msg.topic == "sim/dock/state":
            with self.lock:
                changed = self.dock_state is None or {k: body.get(k) for k in ("green_bay", "phase", "window")} != \
                    {k: self.dock_state.get(k) for k in ("green_bay", "phase", "window")}
                self.dock_state = body
            if changed:
                self.publish_truth()
            return
        pose = W.pose_from_state(body, self.frame)
        if pose is None:
            return
        with self.lock:
            self.pose = pose
            self.pose_t = time.monotonic()

    def log(self, text: str) -> None:
        print(f"[fake_perception] {text}", flush=True)

    def envelope(self, kind: str, body: dict) -> str:
        self.seq += 1
        return json.dumps({"schema_version": 1, "msg_id": f"fp-{self.seq:010d}", "seq": self.seq, "sent_at": utc_iso(), "run_id": None,
                           "declaration_seq": None, "vehicle_id": self.a.vehicle_id, "kind": kind, "source": "fake_perception", "body": body})

    def objects_now(self, which: Optional[List[dict]] = None) -> List[dict]:
        with self.lock:
            ds = self.dock_state
        return W.apply_dock_state(self.objects if which is None else which, ds)

    def publish_truth(self) -> None:
        c = self.hub if self.hub is not None else self.c
        with self.lock:
            ds = self.dock_state
        truth = {"name": self.name, "origin": self.world.get("origin"), "start": self.world.get("start"), "bounds": self.world.get("bounds"),
                 "objects": W.apply_dock_state(self.all_objects, ds), "camera": self.cam, "lights": self.world.get("lights"),
                 "phases": self.phases, "dock_state": ds, "source": self.a.source, "host": socket.gethostname(), "t": utc_iso()}
        c.publish("sim/truth", self.envelope("telemetry", truth), qos=0, retain=True)

    # ---- the camera model ---------------------------------------------------------------------------------------------
    def sigma_range(self, r: float) -> float:
        abc = self.cam.get("noise_range_abc")
        if abc:
            a, b, c = (list(abc) + [0, 0, 0])[:3]
            return float(a) + float(b) * r + float(c) * r * r
        return float(self.cam.get("noise_range_m", 0.0))

    def observe(self, t: Optional[float] = None) -> Dict[str, Observation]:
        with self.lock:
            pose, pose_t = dict(self.pose) if self.pose else None, self.pose_t
        if pose is None or time.monotonic() - pose_t > 2.0:
            return {}
        fov = float(self.cam.get("fov_deg", 100.0)) / 2.0
        rmin, rmax = float(self.cam.get("min_range_m", 0.5)), float(self.cam.get("max_range_m", 25.0))
        bmax = float(self.cam.get("bearing_max_m", 40.0))            # v0.1.34: the bearing-only band (pathing spec: 25-40 m for the 4 mm lens)
        p_drop, p_mis = float(self.cam.get("p_drop_max", 0.0)), float(self.cam.get("p_miscolour_max", 0.0))
        lights = self.world.get("lights")
        t_cam = time.time() if t is None else t
        out: Dict[str, List[ObsItem]] = {}
        for o in self.objects_now():
            dx, dy = o["x_n"] - pose["x_n"], o["y_e"] - pose["y_e"]
            rng = math.hypot(dx, dy)
            brg = wrap180(math.degrees(math.atan2(dy, dx)) - pose["hdg_deg"])
            if rng < rmin or abs(brg) > fov:
                continue
            if rng > rmax:                                 # v0.1.34: beyond the depth, a buoy still gives its bearing (up to bearing_max_m)
                if o.get("kind", "buoy") != "buoy" or rng > bmax or self.rng_b.random() < float(self.cam.get("dropout", 0.0)) + 0.5 * p_drop:
                    continue
                has_beacon = o.get("light") in ("flash", "steady", "off")
                colour = W.observed_colour(o, t_cam, self.phases.get(int(o["id"]), 0.0), lights) if has_beacon else o.get("colour", "unknown")
                if o.get("colour_from_above"):   # v0.1.66 (D23): colour-blind from the water
                    colour, has_beacon = "unknown", False
                if has_beacon and colour not in ("black", "unknown") and self.cam.get("lamp_read") and \
                        self.rng_r.random() >= _interp(self.cam["lamp_read"], rng):
                    colour = "black"                                # v0.1.94 (real_13nov): the far lamp missed
                c_out, q = _led(self.cam, colour, has_beacon, {"bearing_only": True, "reason": "beyond_depth"})
                out.setdefault("buoys", []).append(ObsItem(
                    id=int(o["id"]), cls="buoy", colour=c_out, conf=float(self.cam.get("conf", 0.8)), range_m=0.0,
                    bearing_deg=brg + self.rng_b.gauss(0.0, float(self.cam.get("noise_bearing_deg", 0.0))), elev_deg=0.0,
                    quality=q, lit=(colour != "black") if has_beacon else None))
                continue
            if o.get("kind") == "window" and rng > float(self.cam.get("window_max_range_m", 8.0)):
                continue                                   # a 25 cm target hole is not resolved from far away
            ramp = max(0.0, min(1.0, (rng - 0.6 * rmax) / (0.4 * rmax))) if rmax > 0 else 0.0
            if self.rng.random() < float(self.cam.get("dropout", 0.0)) + p_drop * ramp:
                continue
            rng_n = rng + self.rng.gauss(0.0, self.sigma_range(rng))
            brg_n = brg + self.rng.gauss(0.0, float(self.cam.get("noise_bearing_deg", 0.0)))
            kind = o.get("kind", "buoy")
            has_beacon = kind in ("buoy", "box") and o.get("light") in ("flash", "steady", "off")
            colour = W.observed_colour(o, t_cam, self.phases.get(int(o["id"]), 0.0), lights) if has_beacon else o.get("colour", "unknown")
            if o.get("colour_from_above"):       # v0.1.66 (D23): the boat cannot see this buoy's colour (only the UAV can)
                colour, has_beacon = "unknown", False
            if kind == "buoy" and has_beacon and colour not in ("black", "unknown") and self.cam.get("lamp_read"):
                if self.rng_r.random() >= _interp(self.cam["lamp_read"], rng):      # v0.1.94 (real_13nov): a lit lamp missed = read dark
                    colour = "black"
            if kind == "buoy" and self.cam.get("range_ok") and self.rng_r.random() >= _interp(self.cam["range_ok"], rng):
                # v0.1.94 (real_13nov): no depth on this box (13 Nov neural_light: 43-90 % depth_invalid on near boxes) -> a bearing only
                c_out, q = _led(self.cam, colour, has_beacon, {"bearing_only": True, "reason": "depth_invalid"})
                out.setdefault("buoys", []).append(ObsItem(
                    id=int(o["id"]), cls="buoy", colour=c_out, conf=float(self.cam.get("conf", 0.8)), range_m=0.0, bearing_deg=brg_n,
                    elev_deg=0.0, quality=q, lit=(colour != "black") if has_beacon else None))
                continue
            if kind in ("buoy", "box") and p_mis > 0 and self.rng.random() < p_mis * ramp:
                colour = self.rng.choice([c for c in ("red", "green", "blue", "black") if c != colour])
            quality = {"valid_fraction": 0.7, "depth_spread_m": 0.1}
            if kind == "window" and o.get("window") is not None:
                quality["window"] = int(o["window"])          # v0.1.48: 1 upper / 2 lower, as cv/sim_observer numbers them
            if kind in ("box", "dock"):
                if kind == "box":
                    width = float(o.get("width_m", 2.0))
                    tangent = math.radians(float(o.get("face_hdg_deg", 90.0)))
                else:                                      # v0.1.58 (WP13): a dock face's key points, as the renderer draws it
                    from . import render as R
                    width = min(1.3, max(0.85, float(o.get("size_m") or R.DOCK_SIDE_M * R.DOCK_BAY_SCALE.get(int(o.get("bay", 0) or 0), 1.0))))
                    tangent = math.radians(R.face_heading(self.world, o) - 90.0)      # left -> right as seen from the water
                dn, de = math.cos(tangent), math.sin(tangent)

                def face_point(scale):
                    pn, pe = o["x_n"] + scale * width * dn, o["y_e"] + scale * width * de
                    pdn, pde = pn - pose["x_n"], pe - pose["y_e"]
                    pr = math.hypot(pdn, pde)
                    pb = wrap180(math.degrees(math.atan2(pde, pdn)) - pose["hdg_deg"])
                    return {"range_m": round(pr, 4), "bearing_deg": round(pb, 4)}

                quality["face_points"] = {
                    "left": face_point(-0.5), "midpoint": face_point(0.0), "right": face_point(0.5)
                }
            c_out = colour
            if kind == "buoy":
                c_out, quality = _led(self.cam, colour, has_beacon, quality)
            item = ObsItem(id=int(o["id"]), cls=kind, colour=c_out, conf=float(self.cam.get("conf", 0.8)),
                           range_m=max(rmin, rng_n), bearing_deg=brg_n, elev_deg=0.0,
                           quality=quality, lit=(colour != "black") if has_beacon else o.get("lit"))
            out.setdefault(TOPIC_OF_KIND.get(kind, "buoys"), []).append(item)
        hint = {"lat": pose.get("lat"), "lon": pose.get("lon"), "hdg_deg": pose["hdg_deg"], "t": t_cam}
        return {k: Observation(t_cam=t_cam, kind=k, camera=self.cam.get("name", "zed2i_left"), pose_hint=hint, items=v, t_utc=utc_iso(),
                               source="usv")
                for k, v in out.items() if v}

    def health_body(self, period: float) -> dict:
        with self.lock:
            pose_age = None if not self.pose else round(time.monotonic() - self.pose_t, 2)
        return {"ok": pose_age is not None and pose_age < 2.0, "pose_age_s": pose_age, "rate_hz": 1 / period, "n_published": self.n_obs,
                "camera": self.cam.get("name", "zed2i_left"), "detector": "fake_perception", "source": self.a.source, "world": self.name,
                "host": socket.gethostname(), "t": utc_iso()}

    def run(self) -> None:
        self.c.connect_async(self.a.host, self.a.port, keepalive=15)
        self.c.loop_start()
        stop_presence = start_presence(self.c, "fake_perception", f"{self.a.host}:{self.a.port}")
        if self.hub is not None:
            self.hub.connect_async(self.a.hub_host or self.a.host, self.a.hub_port, keepalive=15)
            self.hub.loop_start()
        period = 1.0 / float(self.a.hz or self.cam.get("hz", 5))
        last_health = 0.0
        self.log(f"world {self.a.world} ({self.name}): {len(self.objects)} sensed objects; publishing at {1 / period:.0f} Hz on {self.ns}/perception/*"
                 f" (boat {self.a.host}:{self.a.port}; truth -> {'hub ' + (self.a.hub_host or self.a.host) + ':' + str(self.a.hub_port) if self.hub else 'boat broker'})")
        try:
            while True:
                t0 = time.monotonic()
                for kind, obs in self.observe().items():
                    self.c.publish(f"{self.ns}/perception/{kind}", self.envelope("local", obs.body()), qos=0)
                    self.n_obs += 1
                if t0 - last_health >= 1.0:
                    last_health = t0
                    self.c.publish(f"{self.ns}/perception/health", self.envelope("local", self.health_body(period)), qos=0)
                time.sleep(max(0.0, period - (time.monotonic() - t0)))
        except KeyboardInterrupt:
            pass
        finally:
            self.clear_truth()
            stop_presence()
            self.c.loop_stop()
            if self.hub is not None:
                self.hub.loop_stop()

    def clear_truth(self) -> None:
        """v0.1.28.1: the world leaves the map with this simulator - an empty retained sim/truth deletes it on the (persistent) hub."""
        try:
            (self.hub if self.hub is not None else self.c).publish("sim/truth", b"", qos=0, retain=True).wait_for_publish(1.0)
        except Exception:
            pass


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="profiles/bench/world_bench.yaml")
    ap.add_argument("--host", default="127.0.0.1", help="boat broker host")
    ap.add_argument("--port", type=int, default=1884, help="boat broker")
    ap.add_argument("--hub-host", default=None, help="hub broker host for the retained sim/truth (default: --host)")
    ap.add_argument("--hub-port", type=int, default=1882, help="hub broker port (0 = publish sim/truth on the boat broker)")
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--hz", type=float, default=None, help="override the world file's camera rate")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--source", default="sim", help="the health beat's `source` tag (the consoles show 'CV sim')")
    ap.add_argument("--skip-kinds", default="", help="v0.1.36: kinds NOT reported (e.g. buoy: the simulator's camera path reports them - "
                    "scripts/run_sim.sh; the dock faces, windows and boxes still come from here)")
    return ap.parse_args(argv)


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None) -> int:
    signal.signal(signal.SIGTERM, _sigterm)        # a clean stop (make stop, pytest teardown) publishes presence up=false
    FakePerception(parse_args(argv)).run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
