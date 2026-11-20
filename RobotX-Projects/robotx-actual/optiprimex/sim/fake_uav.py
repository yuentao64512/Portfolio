"""fake_uav: a simulated UAV survey for m1t2 (v0.1.66, D23 of docs/OVERNIGHT_2026-11-07_UAV.md) - the drone team's buoy detector as the
boat would hear it, per docs/UAV_BUOY_CONTRACT.md.

It waits for our RunStart (ocs/state r == R5 - the handbook's order: RunStart, then the UAV surveys, then the boat transits), then flies a
lawnmower over the field (the Task 1 buoys' box + `--margin-m`) at `--alt-m` 10 m and `--speed` 3 m/s, lanes `--lane-m` apart; every
second it publishes `uav/perception/buoys` (QoS 0, not retained) with the buoys inside its `--footprint-m` 25 m footprint, from the world's
truth (`sim/truth`, retained on the hub): position noise `--noise-m` 0.7 m (a calibrated UAV at 10 m; 1.5 m shows what the 1 m link
radius does), `--dropout` 10 %, the colour right `--colour-ok` 95 % of the time, stable ids per buoy (`u<id>`), `sigma_m` = the noise,
lit flash / steady / off from the light. `scan.state` scanning -> complete at the end of the lawnmower, with the full item list (each
buoy's mean of its sightings) three times, then it lands (`--no-complete`: never sends complete - the operator's word or the telemetry
fallback must release the boat). `--start-now` flies at once. Also `sim/uav` (retained: its position, for the maps' truth layer) and
presence/uav_cv. To the hub (the relay path: the hub -> boat bridge lands it on local/usv/uav/perception/buoys); `--broker` = another
broker (the direct path in a test). Nothing here talks to a FCU.

  python -m optiprimex.sim.fake_uav [--hub 127.0.0.1:1882] [--noise-m 0.7] [--survey-s S] [--no-complete] [--start-now] [--seed 7]
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import threading
import time
from typing import Dict, List, Optional, Tuple

import paho.mqtt.client as mqtt

from . import world as W

T1_ROLES = ("entry", "exit", "obstacle")


def t1_buoys(objects: List[dict]) -> List[dict]:
    """The Task 1 buoys of a world (a gate number or a T1 role); every buoy when none is marked."""
    bs = [o for o in objects if o.get("kind") == "buoy" and not o.get("task")]
    t1 = [o for o in bs if o.get("gate") is not None or o.get("role") in T1_ROLES]
    return t1 or bs


def lawnmower(bs: List[dict], lane_m: float, margin_m: float) -> List[Tuple[float, float]]:
    """Boustrophedon over the buoys' box + margin, lanes along the longer side."""
    xs, ys = [float(o["x_n"]) for o in bs], [float(o["y_e"]) for o in bs]
    x0, x1, y0, y1 = min(xs) - margin_m, max(xs) + margin_m, min(ys) - margin_m, max(ys) + margin_m
    along_x = (x1 - x0) >= (y1 - y0)
    pts: List[Tuple[float, float]] = []
    if along_x:
        n = max(1, int(math.ceil((y1 - y0) / lane_m)))
        for i in range(n + 1):
            y = y0 + i * (y1 - y0) / n
            pts += [(x0, y), (x1, y)] if i % 2 == 0 else [(x1, y), (x0, y)]
    else:
        n = max(1, int(math.ceil((x1 - x0) / lane_m)))
        for i in range(n + 1):
            x = x0 + i * (x1 - x0) / n
            pts += [(x, y0), (x, y1)] if i % 2 == 0 else [(x, y1), (x, y0)]
    return pts


def path_len(pts) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))


def at(pts, d: float) -> Tuple[float, float, float]:
    """The point d metres along the polyline, with the heading of its segment (deg)."""
    for a, b in zip(pts, pts[1:]):
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if d <= L or b == pts[-1]:
            f = 0.0 if L == 0 else min(1.0, d / L)
            return a[0] + f * (b[0] - a[0]), a[1] + f * (b[1] - a[1]), math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) % 360.0
        d -= L
    return pts[-1][0], pts[-1][1], 0.0


class Survey:
    """The pure part: given the truth and a time since the start, the message of that second (tested without a broker)."""

    def __init__(self, truth: dict, noise_m: float = 0.7, dropout: float = 0.10, colour_ok: float = 0.95, footprint_m: float = 25.0,
                 speed: float = 3.0, lane_m: float = 15.0, margin_m: float = 10.0, alt_m: float = 10.0, survey_s: Optional[float] = None,
                 complete: bool = True, seed: int = 7):
        self.frame = W.WorldFrame(truth.get("origin"))
        self.buoys = t1_buoys(truth.get("objects") or [])
        self.noise, self.dropout, self.colour_ok, self.fp = noise_m, dropout, colour_ok, footprint_m
        self.alt, self.complete_on = alt_m, complete
        self.path = lawnmower(self.buoys, lane_m, margin_m) if self.buoys else [(0.0, 0.0)]
        L = path_len(self.path)
        self.speed = (L / survey_s) if survey_s else speed
        self.survey_s = L / max(self.speed, 1e-6)
        self.rng = random.Random(seed)
        self.seen: Dict[int, List[Tuple[float, float]]] = {}
        self.n_complete_sent = 0
        self.seq = 0

    def pos(self, t: float) -> Tuple[float, float, float]:
        return at(self.path, min(t, self.survey_s) * self.speed)

    def item(self, o: dict, x: float, y: float, sigma: float) -> dict:
        lat, lon = self.frame.to_latlon(x, y)
        c = o.get("colour")
        if self.rng.random() > self.colour_ok:
            c = self.rng.choice([k for k in ("red", "green", "blue", "black") if k != c])
        lt = o.get("light")
        lit = "off" if lt in ("off", False) else {"flash": "flash", "steady": "steady"}.get(lt)     # YAML reads `off` as False
        return {"id": f"u{o.get('id')}", "lat": round(lat, 7), "lon": round(lon, 7), "class": "buoy", "colour": c, "lit": lit,
                "conf": 0.9, "sigma_m": round(sigma, 2)}

    def message(self, t: float) -> Optional[dict]:
        """The message at t seconds after the start; None once landed (complete sent three times, or --no-complete past the end)."""
        x, y, hdg = self.pos(t)
        lat, lon = self.frame.to_latlon(x, y)
        done = t >= self.survey_s
        if done and (not self.complete_on or self.n_complete_sent >= 3):
            return None
        self.seq += 1
        items = []
        if not done:
            for o in self.buoys:
                if math.hypot(float(o["x_n"]) - x, float(o["y_e"]) - y) > self.fp / 2.0 or self.rng.random() < self.dropout:
                    continue
                ox, oy = float(o["x_n"]) + self.rng.gauss(0, self.noise), float(o["y_e"]) + self.rng.gauss(0, self.noise)
                self.seen.setdefault(int(o["id"]), []).append((ox, oy))
                items.append(self.item(o, ox, oy, self.noise))
        else:                                                   # complete: every buoy seen, its mean, three times
            self.n_complete_sent += 1
            for o in self.buoys:
                s = self.seen.get(int(o["id"]))
                if s:
                    mx, my = sum(p[0] for p in s) / len(s), sum(p[1] for p in s) / len(s)
                    items.append(self.item(o, mx, my, self.noise / math.sqrt(len(s))))
        cov = min(100.0, 100.0 * min(t, self.survey_s) / max(self.survey_s, 1e-6))
        return {"t": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "source": "uav", "frame_id": f"fake-uav-{self.seq}",
                "uav": {"lat": round(lat, 7), "lon": round(lon, 7), "alt_hae_m": self.alt, "hdg_deg": round(hdg, 1)},
                "scan": {"state": "complete" if done else "scanning", "coverage_pct": round(cov, 1), "n_items": len(items)},
                "items": items}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--hub", default="127.0.0.1:1882")
    ap.add_argument("--broker", default=None, help="publish the buoys to this broker instead (the direct path); the hub is still read")
    ap.add_argument("--noise-m", type=float, default=0.7)
    ap.add_argument("--dropout", type=float, default=0.10)
    ap.add_argument("--colour-ok", type=float, default=0.95)
    ap.add_argument("--footprint-m", type=float, default=25.0)
    ap.add_argument("--speed", type=float, default=3.0)
    ap.add_argument("--lane-m", type=float, default=15.0)
    ap.add_argument("--survey-s", type=float, default=None)
    ap.add_argument("--no-complete", action="store_true")
    ap.add_argument("--start-now", action="store_true")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--udp", default=None, metavar="HOST:PORT", help="v0.1.69: send the buoys as JSON datagrams {type: uav_buoys, ...} to "
                                                                      "our UAV adapter (its 14605) instead of MQTT - the session's udp path")
    ap.add_argument("--offset-km", type=float, default=0.0, help="v0.1.69: move every position this far north - the footage is elsewhere "
                                                                 "(dry uavmap: the far-UAV caution, fit to UAV items)")
    a = ap.parse_args(argv)
    host, _, port = a.hub.rpartition(":")
    state = {"truth": None, "r": None}
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"fake-uav-{os.getpid()}")

    def on_msg(cl, u, m):
        try:
            env = json.loads(m.payload) if m.payload else None
        except ValueError:
            return
        b = (env or {}).get("body") if isinstance(env, dict) else None
        if m.topic == "sim/truth":
            state["truth"] = b
        elif m.topic == "ocs/state" and isinstance(b, dict):
            state["r"] = b.get("r")
    c.on_connect = lambda cl, u, fl, rc, pr=None: (cl.subscribe("sim/truth", 0), cl.subscribe("ocs/state", 0))
    c.on_message = on_msg
    c.connect(host or "127.0.0.1", int(port or 1882), keepalive=15)
    c.loop_start()
    out = c
    if a.broker:
        bh, _, bp = a.broker.rpartition(":")
        out = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"fake-uav-out-{os.getpid()}")
        out.connect(bh or "127.0.0.1", int(bp or 1883), keepalive=15)
        out.loop_start()
    stop = threading.Event()
    udp = None
    if a.udp:
        import socket
        uh, _, up = a.udp.rpartition(":")
        udp = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (uh or "127.0.0.1", int(up or 14605)))
    dlat = a.offset_km * 1000.0 / 111320.0

    def shift(msg: dict) -> dict:
        if not dlat:
            return msg
        msg["uav"]["lat"] = round(msg["uav"]["lat"] + dlat, 7)
        for it in msg.get("items") or []:
            it["lat"] = round(it["lat"] + dlat, 7)
        return msg

    def presence(up: bool):
        c.publish("presence/uav_cv", json.dumps({"up": up, "t": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "source": "fake_uav"}), qos=0, retain=True)
    try:
        while True:                                            # one survey per RunStart (R5 entered)
            print("fake_uav: waiting for the world (sim/truth) and our RunStart (ocs/state R5)" + (" - start now" if a.start_now else ""), flush=True)
            prev = state["r"]
            while not stop.is_set():
                presence(True)
                if state["truth"] and (a.start_now or (state["r"] == "R5" and prev != "R5")):
                    break
                prev = state["r"]
                time.sleep(0.5)
            sv = Survey(state["truth"], a.noise_m, a.dropout, a.colour_ok, a.footprint_m, a.speed, a.lane_m, survey_s=a.survey_s,
                        complete=not a.no_complete, seed=a.seed)
            print(f"fake_uav: survey of {len(sv.buoys)} buoys, {path_len(sv.path):.0f} m at {sv.speed:.1f} m/s = {sv.survey_s:.0f} s, noise {a.noise_m} m", flush=True)
            t0 = time.time()
            while True:
                t = time.time() - t0
                msg = sv.message(t)
                if msg is None:
                    break
                msg = shift(msg)
                if udp is not None:
                    udp[0].sendto(json.dumps(dict(msg, type="uav_buoys")).encode(), udp[1])
                else:
                    out.publish("uav/perception/buoys", json.dumps(msg), qos=0)
                c.publish("sim/uav", json.dumps({"lat": msg["uav"]["lat"], "lon": msg["uav"]["lon"], "alt_m": sv.alt, "t": msg["t"],
                                                 "scan": msg["scan"]}), qos=0, retain=True)
                if msg["scan"]["state"] == "complete":
                    print(f"fake_uav: scan complete at {t:.0f} s ({msg['scan']['n_items']} buoys)", flush=True)
                time.sleep(1.0)
            print(f"fake_uav: landed after {time.time() - t0:.0f} s", flush=True)
            c.publish("sim/uav", b"", qos=0, retain=True)
            if a.start_now:
                break
            while state["r"] == "R5":
                time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    finally:
        presence(False)
        c.publish("sim/uav", b"", qos=0, retain=True)
        time.sleep(0.3)
        c.loop_stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
