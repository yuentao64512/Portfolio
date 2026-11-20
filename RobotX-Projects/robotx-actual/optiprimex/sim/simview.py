#!/usr/bin/env python3
"""simview: our simulator's server - the rendered camera and the SIM page (v0.1.32, ROADMAP WP10). Its own window, not the OCS page.

The boat is the fake FCU's plant (kinematic, no disturbances - decided 2026-11-04); this server asks its control port for the pose
({"get": "state"}), renders what the boat's camera sees with optiprimex/sim/render.py (water, buoys with their lights, box faces,
the pool walls; v0.1.46 the Task 3 dock structure with its indicator and window lights from fake_dock's sim/dock/state, --boat-port),
and serves:

  GET  /                the SIM page (sim.html): a chase view (third person, pseudo-3D), the camera, a top-down map, the controls
  GET  /stream          the camera as MJPEG             ┐ the same interface as cv/stream (the AGX camera): the OCS page's CAMERA
  GET  /snapshot.jpg    the latest camera frame         │ tab and map.html's camera link show it with CAM_URL=http://127.0.0.1:8097/
  GET  /status          frames, age, clients, source    ┘
  GET  /depth.jpg       v0.1.60 (WP10m): the camera's depth, coloured (tools/depthviz: near red -> far blue, nothing black)
  POST /mode            v0.1.60: {"mode": "rgb|depth|both"} - the CAMERA tab's view (X-Sim-Web: 1, as every post here)
  GET  /depth_at?x=&y=  v0.1.60: the range (m) at the normalised picture position (0..1 from the top left)
  GET  /frame.npz?w=&h= one frame for the sim observer (v0.1.36): rgb, depth, ids + the primitives' world ids / kinds, pose, time
  GET  /chase.jpg       the latest chase-view frame;  GET /world.json  the objects, the walls, the boat, the FCU's state
  POST /sticks          {"surge", "yaw", "sway"} -1..1 -> the fake FCU (MANUAL only there; 0.5 s without one = released)
  POST /fcu             the transmitter / the plant: {"set": {"mode": 0|4, "kill", "estop", "rc6", "gps_fix", "ekf_ok", "rc_lost"}},
                        {"pause_stream": S}, {"teleport": {"x_n", "y_e", "hdg"}} - nothing else (no arming from here). v0.1.52: the
                        SIM page shows the transmitter as status only (Dmitrii); this API stays for the tools (fault_timing, sim_day)
  POST /objects         {"op": "add"|"move"|"del", ...} the mockups (in memory; the world file stays as it is): kind buoy (colour, light
                        off|steady|flash) | box | dock (colour, bay); by x_n / y_e or lat / lon (v0.1.35); `mock: true` on added ones
  POST /waypoint        {"name", x_n/y_e | lat/lon} -> save_point through the OCS (v0.1.35)
  GET /markers.json · POST /markers   the OCS page's markers, relayed (v0.1.35)
  POST /teleport        {"name": a teleport or a saved point} | {"x_n", "y_e"[, "hdg"]} | {"lat", "lon"[, "hdg"]}; "snap": true moves
                        a click within 2 m of HOME / a saved point onto it (v0.1.33)
  POST /reset           RESET SIM (v0.1.33): the boat to the world's start, the mockups back to the file, every fault off, the mode
                        switch HOLD, then (over the hub) abort, confirm_safe, reset on both ends, clear_map, the fake judge's counters
  v0.1.53, the course (sim/courses.py):
  GET  /course.json     {course_id (the world's), received {course_id, match, src, age_s, differs, ignored}, banner, courses, worlds,
                        hook} - what the SIM page's and the OCS map's "course X received - move the world there / ignore" need
  POST /world           {"course": ID | "received"[, "world": NAME | FILE][, "keep": true][, "force": true][, "teleport": NAME | false]}
                        = move: the world (the course's own world from the catalogue; "keep": the current one; "world": that one)
                        re-anchored to the course (world.move_to_course; written to --world-dir, fp + dock restarted on it through
                        --world-hook), the boat teleported to its start (or the named teleport); a layout that does not fit the course
                        is refused unless "force". {"world": NAME | FILE} alone = that world as it is (the picker).
                        {"ignore": true} = keep the current world for the course received now (the banner goes until another course comes)
  The received course: the hub's retained rc/course (the fake judge) and robocommand/robotx/course (protobuf), the OCS's COURSE to the
  vehicle (team/.../cmd/<v>/course: the course the OCS accepted, rc-test's included), and --rc-port's robocommand/robotx/course (rc-test).
  every POST needs the header X-Sim-Web: 1 (a cross-site form cannot set it).

  python -m optiprimex.sim.simview --world profiles/bench/world_pool.yaml [--fcu-control 14560] [--port 8097] [--hz 6] [--open]
  scripts/run_sim.sh starts it with the bench.
"""
from __future__ import annotations

import argparse
import copy
import json
import math
import os
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Any, Dict, List, Optional

from . import render as R
from . import world as W

HERE = os.path.dirname(os.path.abspath(__file__))
PAGE = os.path.join(HERE, "sim.html")
BOUNDARY = "simframe"
FCU_SET_KEYS = {"mode", "kill", "estop", "rc6", "rc5", "gps_fix", "ekf_ok", "rc_lost"}
KIT = os.path.dirname(os.path.dirname(HERE))
POINTS = os.path.join(KIT, "usv_ui", "points.json")
SNAP_M = 2.0
FCU_CLEAN = {"kill": False, "estop": False, "rc5": 1000, "rc6": 1000, "gps_fix": 3, "ekf_ok": True, "rc_lost": False, "rc_sys": "absent",
             "rc_chancount": 16, "mode": 4}                                  # RESET SIM: the plant as the bench starts it


def vehicle_view(body: dict) -> dict:
    """v0.1.52: what the SIM page shows of the mission manager (local/<v>/mm/state): its state, the latched fault, the faults now and
    the FCU's EKF / GPS as the mission manager sees them. `latched` = FAULT / KILLED while the cause is already gone: the vehicle waits
    for confirm_safe (by design - a fault never clears itself), which the page says instead of looking like a stuck fault."""
    fcu = body.get("fcu") or {}
    now = list((body.get("health") or {}).get("faults") or [])
    state = body.get("state")
    cause_on = bool(now) or bool(body.get("kill")) or bool(body.get("estop"))
    return {"state": state, "fault": body.get("fault"), "faults_now": now, "hold_reason": body.get("hold_reason"),
            "ekf_pos_ok": fcu.get("ekf_pos_ok"), "gps_fix": fcu.get("gps_fix"), "kill": body.get("kill"), "estop": body.get("estop"),
            "latched": state in ("FAULT", "KILLED") and not cause_on}


class Fcu:
    """The fake FCU's control port (UDP JSON)."""

    def __init__(self, port: int, host: str = "127.0.0.1"):
        self.addr = (host, port)

    def ask(self, msg: dict, timeout_s: float = 0.3) -> Optional[dict]:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.settimeout(timeout_s)
            try:
                s.sendto(json.dumps(msg).encode(), self.addr)
                data, _ = s.recvfrom(65535)
                return json.loads(data)
            except (OSError, ValueError):
                return None


class Sim:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.world_path = a.world
        self.world = W.load(a.world)
        self.frame = W.WorldFrame(self.world.get("origin"))
        self.phases = W.light_phases(self.world.get("objects") or [], 1)
        self.lights = self.world.get("lights") or {}
        if a.cam_height is None:                       # v0.1.79: the world's mount (world_scout: 1.0 m)
            a.cam_height = float((self.world.get("camera") or {}).get("height_m", 0.8))
        self.fcu = Fcu(a.fcu_control)
        self.cam = R.Camera(w=a.width, h=a.height, hfov_deg=a.hfov, height_m=a.cam_height)
        self.chase_cam = R.Camera(w=a.chase_width, h=a.chase_height, hfov_deg=70.0, height_m=5.0, pitch_deg=22.0)
        self.lock = threading.Lock()
        self.cond = threading.Condition(self.lock)
        self.jpeg = b""
        self.chase = b""
        self.depth = None
        self.n = 0
        self.t_frame = 0.0
        self.clients = 0
        self.state: Dict[str, Any] = {}
        self.pose = (0.0, 0.0, 0.0)
        self.t_pose = 0.0
        self.stop = threading.Event()
        self.next_id = 1000 + max([int(o.get("id", 0)) for o in self.world.get("objects") or []] + [0])
        self.dock_state: Optional[dict] = None          # v0.1.46: sim/dock/state (fake_dock, the boat broker) -> the dock's lights
        self.dock_t = 0.0
        self.vehicle: Optional[dict] = None             # v0.1.52: local/<v>/mm/state (the mission manager) -> the SIM page's status line
        self.vehicle_t = 0.0
        self.received: Optional[dict] = None            # v0.1.53: the judge's course as heard ({"course", "src", "t"})
        self.ignored: Optional[str] = None              # the fingerprint of the received course the operator chose to ignore
        self.hook_result: Optional[str] = None          # the last --world-hook run (fp + dock restarted on a moved world)

    # ---- the world --------------------------------------------------------------------------------------------------
    def lit(self, now: float) -> Dict[int, bool]:
        per, on = float(self.lights.get("period_s", 2.0)), float(self.lights.get("on_s", 1.0))
        out = {}
        for o in self.world.get("objects") or []:
            if o.get("light") == "flash":
                out[o.get("id")] = ((now + self.phases.get(o.get("id"), 0.0)) % per) < on
        return out

    def watch_dock(self) -> None:
        """v0.1.46 (WP10j): follow sim/dock/state on the boat broker (--boat-port; 0 = off) so the camera shows the GREEN bay's
        indicator and the lit window as fake_dock has them (the flashing buoy lights come from self.lit the same way)."""
        if not getattr(self.a, "boat_port", 0):
            return
        import paho.mqtt.client as mqtt

        def on_connect(c, u, f, rc, p=None):
            c.subscribe("sim/dock/state", 0)
            c.subscribe("local/+/mm/state", 0)         # v0.1.52: the vehicle's state + latched fault for the SIM page

        def on_message(c, u, msg):
            try:
                env = json.loads(msg.payload)
            except ValueError:
                return
            body = env.get("body", env) if isinstance(env, dict) else None
            if isinstance(body, dict):
                with self.lock:
                    if msg.topic.endswith("/mm/state"):
                        self.vehicle, self.vehicle_t = vehicle_view(body), time.time()
                    else:
                        self.dock_state, self.dock_t = body, time.time()

        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"simview-dock-{os.getpid()}", protocol=mqtt.MQTTv311)
        c.on_connect, c.on_message = on_connect, on_message
        c.reconnect_delay_set(1, 5)
        c.connect_async(self.a.boat_host, int(self.a.boat_port), keepalive=15)
        c.loop_start()
        self.dock_client = c

    def poll_pose(self) -> None:
        st = self.fcu.ask({"get": "state"})
        if not st:
            return
        x, y = self.frame.to_world(float(st["lat"]), float(st["lon"])) if self.frame.ok else (float(st.get("x_n", 0)), float(st.get("y_e", 0)))
        with self.lock:
            self.state, self.pose, self.t_pose = st, (x, y, float(st.get("hdg_deg", 0.0))), time.time()

    def render_once(self) -> None:
        with self.lock:
            world = copy.deepcopy(self.world)
            x, y, hdg = self.pose
            dock = copy.deepcopy(self.dock_state)
        scene = R.scene_from_world(world, self.lit(time.time()), dock_state=dock)
        rgb, depth = R.render(scene, x, y, hdg, self.cam)
        back = math.radians(hdg)
        cx, cy = x - 9.0 * math.cos(back), y - 9.0 * math.sin(back)               # the chase camera: 9 m behind, 5 m up
        boat = R.Box((x, y, 0.25), (0.9, 0.45, 0.25), hdg, (245, 245, 240), tag="boat")
        scene.boxes.append(boat)
        scene.spheres.append(R.Sphere((x + 0.9 * math.cos(back), y + 0.9 * math.sin(back), 0.55), 0.12, (255, 120, 40), emissive=True, tag="bow"))
        chase, _ = R.render(scene, cx, cy, hdg, self.chase_cam)
        j, c = R.jpeg(rgb), R.jpeg(chase, 75)
        with self.cond:
            self.jpeg, self.chase, self.depth, self.n, self.t_frame = j, c, depth, self.n + 1, time.time()
            self.cond.notify_all()

    def loop(self) -> None:
        period = 1.0 / max(0.5, self.a.hz)
        while not self.stop.is_set():
            t0 = time.monotonic()
            try:
                self.poll_pose()
                self.render_once()
            except Exception as exc:                                              # a bad mockup must not stop the camera
                print(f"[simview] render failed: {exc}", flush=True)
            self.stop.wait(max(0.0, period - (time.monotonic() - t0)))

    # ---- v0.1.36 (WP10e): one consistent frame for the sim observer ------------------------------------------------------
    def frame_npz(self, w: int, h: int) -> bytes:
        """The camera at w x h (the ZED's intrinsics scaled) at a fresh pose: the picture, the exact depth, the primitive under each
        pixel and which world object / kind each primitive is, the pose (world frame + lat / lon / heading) and the time - an .npz."""
        import io
        import numpy as np
        st = self.fcu.ask({"get": "state"}) or {}
        t = time.time()
        with self.lock:
            world = copy.deepcopy(self.world)
            x, y, hdg = self.pose
            dock = copy.deepcopy(self.dock_state)
        if st:
            x, y = self.frame.to_world(float(st["lat"]), float(st["lon"])) if self.frame.ok else (x, y)
            hdg = float(st.get("hdg_deg", hdg))
        cam = R.Camera(w=w, h=h, height_m=self.a.cam_height)
        scene = R.scene_from_world(world, self.lit(t), dock_state=dock)
        rgb, depth, pid = R.render_ids(scene, x, y, hdg, cam)
        codes, objs, kinds = [], [], []
        for i, sp in enumerate(scene.spheres):
            k, oid = sp.tag.split(":", 1)
            codes.append(i), objs.append(int(oid)), kinds.append("buoy" if k in ("buoy", "light") else k)
        for j, bx in enumerate(scene.boxes):
            k, oid = bx.tag.split(":", 1)
            codes.append(R.BOX_ID0 + j), objs.append(int(oid)), kinds.append(k)
        lat, lon = (st.get("lat"), st.get("lon")) if st else self.frame.to_latlon(x, y)
        with self.lock:
            self.n_obs_frames = getattr(self, "n_obs_frames", 0) + 1
            n = self.n_obs_frames
        meta = {"t": t, "frame": n, "pose": {"x_n": x, "y_e": y, "hdg_deg": hdg, "lat": lat, "lon": lon, "gps_fix": st.get("gps_fix", 3),
                                             "ekf_ok": st.get("ekf_ok", True)},
                "camera": {"w": w, "h": h, "fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy, "height_m": cam.height_m},
                "world": world.get("name"), "dock_state": dock}
        buf = io.BytesIO()
        np.savez(buf, rgb=rgb, depth=depth, ids=pid, prim_code=np.array(codes, np.int64), prim_obj=np.array(objs, np.int64),
                 prim_kind=np.array(kinds), meta=np.array(json.dumps(meta)))
        return buf.getvalue()

    # ---- teleports, saved points, RESET SIM (v0.1.33) ---------------------------------------------------------------------
    def start(self) -> dict:
        st = self.world.get("start") or {}
        return {"name": "start", "x_n": float(st.get("x_n", 0.0)), "y_e": float(st.get("y_e", 0.0)), "hdg": float(st.get("hdg_deg", 0.0))}

    def teleports(self) -> List[dict]:
        out = [self.start()]
        for t in self.world.get("teleports") or []:
            if t.get("name") and t["name"] != "start":
                out.append({"name": str(t["name"]), "x_n": float(t["x_n"]), "y_e": float(t["y_e"]), "hdg": float(t.get("hdg", 0.0))})
        for t in out:
            t["lat"], t["lon"] = (round(v, 8) for v in self.frame.to_latlon(t["x_n"], t["y_e"]))
        return out

    def points(self) -> List[dict]:
        """usv_ui/points.json (HOME and the saved points) in the world frame - teleport targets and the snap."""
        from ..usv import points_store
        try:
            pts = points_store.load(self.a.points)
        except Exception:
            return []
        out = []
        for name, p in sorted(pts.items()):
            if isinstance(p, dict) and isinstance(p.get("lat"), (int, float)) and isinstance(p.get("lon"), (int, float)):
                x, y = self.frame.to_world(float(p["lat"]), float(p["lon"]))
                out.append({"name": name, "x_n": round(x, 2), "y_e": round(y, 2), "lat": p["lat"], "lon": p["lon"], "hdg": p.get("hdg")})
        return out

    def teleport(self, body: dict) -> dict:
        """One of: name (a teleport or a saved point), x_n / y_e, lat / lon; hdg optional (else the target's, else the boat's)."""
        hdg = body.get("hdg")
        name = body.get("name")
        if name:
            t = next((t for t in self.teleports() + self.points() if t["name"] == name), None)
            if t is None:
                return {"ok": False, "why": f"no teleport or saved point {name!r}"}
            x, y = float(t["x_n"]), float(t["y_e"])
            hdg = hdg if hdg is not None else t.get("hdg")
        elif body.get("lat") is not None and body.get("lon") is not None:
            x, y = self.frame.to_world(float(body["lat"]), float(body["lon"]))
        elif body.get("x_n") is not None and body.get("y_e") is not None:
            x, y = float(body["x_n"]), float(body["y_e"])
        else:
            return {"ok": False, "why": "teleport: name | x_n, y_e | lat, lon"}
        snapped = None
        if body.get("snap") and not name:
            near = sorted(((math.hypot(p["x_n"] - x, p["y_e"] - y), p) for p in self.points()), key=lambda q: q[0])
            if near and near[0][0] <= SNAP_M:
                snapped = near[0][1]
                x, y = snapped["x_n"], snapped["y_e"]
                hdg = hdg if hdg is not None else snapped.get("hdg")
        hdg = float(hdg) if hdg is not None else self.pose[2]
        lat, lon = self.frame.to_latlon(x, y)
        r = self.fcu.ask({"teleport": {"lat": lat, "lon": lon, "hdg": hdg}})
        if r is None:
            return {"ok": False, "why": "the fake FCU did not answer (is the bench up?)"}
        where = name or (snapped["name"] if snapped else f"N {x:.1f} E {y:.1f}")
        return {"ok": True, "to": where, "x_n": round(x, 2), "y_e": round(y, 2), "lat": round(lat, 8), "lon": round(lon, 8), "hdg": round(hdg, 1),
                "snapped": snapped["name"] if snapped else None}

    def reset(self) -> dict:
        """RESET SIM: a clean start without restarting anything. The plant first (faults off, HOLD, the start pose), the mockups from
        the file, then the stack over the hub: abort, confirm_safe, reset (the OCS -> R2 and the boat forgets the run), clear_map, and
        the fake judge's counters. Each step's answer is reported; a refusal (nothing to abort) is not a failure."""
        steps = []
        r = self.fcu.ask({"set": dict(FCU_CLEAN)})
        if r is None:
            return {"ok": False, "why": "the fake FCU did not answer (is the bench up?)", "steps": steps}
        self.fcu.ask({"pause_stream": 0.0})
        self.fcu.ask({"sticks": {"surge": 0.0, "yaw": 0.0, "sway": 0.0}})
        st = self.start()
        lat, lon = self.frame.to_latlon(st["x_n"], st["y_e"])
        self.fcu.ask({"teleport": {"lat": lat, "lon": lon, "hdg": st["hdg"]}})
        steps.append("plant: faults off, HOLD, at start")
        with self.lock:
            fresh = W.load(self.world_path)
            self.world["objects"] = fresh.get("objects") or []
        steps.append(f"mockups: {len(self.world['objects'])} objects from {os.path.basename(self.world_path)}")
        if not self.a.hub_port:                                   # --hub-port 0: the plant and the world only (the tests)
            steps.append("stack: skipped (--hub-port 0)")
            return {"ok": True, "steps": steps}
        for words in (["abort"], ["confirm_safe"], ["reset"], ["clear_map"]):
            steps.append(f"{words[0]}: {self.ocs_cli(words)}")
        steps.append("judge: " + self.judge_reset())
        return {"ok": True, "steps": steps}

    def waypoint(self, body: dict) -> dict:
        """v0.1.35: a waypoint placed on the SIM page = save_point NAME --lat --lon through the OCS (as the OCS map's)."""
        name = str(body.get("name") or "").strip()
        if body.get("op") == "del":
            out = self.ocs_cli(["del_point", name])
            return {"ok": "DONE" in out or "deleted" in out, "why": out, "name": name}
        x, y = self._where(body)
        lat, lon = self.frame.to_latlon(x, y)
        out = self.ocs_cli(["save_point", name, "--lat", f"{lat:.8f}", "--lon", f"{lon:.8f}"])
        return {"ok": "DONE" in out, "why": "" if "DONE" in out else out, "name": name, "lat": round(lat, 8), "lon": round(lon, 8)}

    def ocs_prefs(self, theme: Optional[str] = None) -> dict:
        """v0.1.47 (UI_QOL item 6): the shared theme (tools/prefs.py): the OCS page's /prefs first, the launcher's when the OCS is down."""
        from ..tools import prefs as P
        base = self.a.ocs_url.rstrip("/")
        if theme:
            t = P.norm(theme)
            if t and P.post_theme(base, t, "X-OCS-Web"):
                return {"ok": True, "prefs": {"theme": t}, "to": "ocs"}
            return P.write_shared(theme)
        t = P.get_theme(base)
        if t is None:
            r = P.read_shared()
            return {"ok": r["from"] != "none", "prefs": {"theme": r["theme"]} if r["theme"] else {}, "from": r["from"]}
        return {"ok": True, "prefs": {"theme": t} if t else {}, "from": "ocs"}

    def ocs_markers(self, body: Optional[dict] = None) -> dict:
        """v0.1.35: the OCS page's markers (named notes, OCS-side only) - read, or add / move / delete through its backend."""
        import urllib.error
        import urllib.request
        url = self.a.ocs_url.rstrip("/") + ("/markers" if body is not None else "/markers.json")
        try:
            if body is None:
                with urllib.request.urlopen(url, timeout=3) as r:
                    return json.loads(r.read())
            if body.get("op") in ("add", "move") and body.get("lat") is None:
                lat, lon = self.frame.to_latlon(*self._where(body))
                body = dict(body, lat=lat, lon=lon)
            req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST", headers={"X-OCS-Web": "1", "Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=5) as r:
                return json.loads(r.read())
        except urllib.error.HTTPError as e:
            return {"ok": False, "why": f"OCS page: HTTP {e.code}"}
        except (OSError, ValueError) as exc:
            return {"ok": False, "why": f"no OCS page at {self.a.ocs_url} ({exc.__class__.__name__})", "markers": []}

    def ocs_cli(self, words: List[str]) -> str:
        cmd = [sys.executable, "-m", "optiprimex.tools.ocs_cli", "--port", str(self.a.hub_port)] + words
        try:
            p = subprocess.run(cmd, capture_output=True, text=True, timeout=8, cwd=KIT, env=dict(os.environ, PYTHONPATH=KIT))
        except (OSError, subprocess.TimeoutExpired) as exc:
            return f"not sent ({exc.__class__.__name__})"
        line = ((p.stdout or "").strip().splitlines() or [(p.stderr or "").strip()[-160:] or f"exit {p.returncode}"])[-1]
        return line[:160]

    def judge_reset(self) -> str:
        """The fake judge's counters ({"kind": "sim_reset"} on rc/sim/inject, hub). rc-test keeps its own (not ours to reset)."""
        try:
            import paho.mqtt.publish as publish
            publish.single("rc/sim/inject", json.dumps({"kind": "sim_reset"}), hostname="127.0.0.1", port=int(self.a.hub_port),
                           client_id=f"simview-reset-{os.getpid()}")
            return "sim_reset sent (the fake judge; rc-test ignores it)"
        except Exception as exc:
            return f"not sent ({exc.__class__.__name__})"

    # ---- the page's world view and edits ----------------------------------------------------------------------------------
    def world_json(self) -> dict:
        cv = self.course_view()                       # v0.1.53 (takes the lock itself)
        with self.lock:
            x, y, hdg = self.pose
            st = dict(self.state)
            return {"name": self.world.get("name"), "path": self.world_path, "objects": copy.deepcopy(self.world.get("objects") or []),
                    "bounds": self.world.get("bounds") or [], "origin": self.world.get("origin"), "boat": {"x_n": round(x, 2), "y_e": round(y, 2), "hdg_deg": round(hdg, 1)},
                    "pose_age_s": None if not self.t_pose else round(time.time() - self.t_pose, 1),
                    "fcu": {k: st.get(k) for k in ("mode", "mode_name", "armed", "kill", "estop", "rc5", "rc6", "gps_fix", "ekf_ok", "u", "v", "r", "sticks")},
                    "camera": self.status(), "teleports": self.teleports(), "points": self.points(),
                    "dock_state": copy.deepcopy(self.dock_state), "dock_age_s": round(time.time() - self.dock_t, 1) if self.dock_t else None,
                    "vehicle": dict(self.vehicle, age_s=round(time.time() - self.vehicle_t, 1)) if self.vehicle else None, **cv}

    # ---- v0.1.53: the course - what was received, the world's, move there / ignore --------------------------------------------------
    def world_course(self) -> Optional[dict]:
        """The world's course: {id, corners [[lat, lon]], transit} - its anchor course, else its `course: bounds` / polygon (the pool:
        SIM-POOL), else None (a world without a course: empty, boxes ...)."""
        from . import courses as C
        c = W.anchor_course(self.world)
        if c is not None:
            return {"id": str(self.world.get("course_id") or c.id), "catalogue": c.id,
                    "corners": [list(p) for p in c.corners], "transit": [list(p) for p in c.transit]}
        crs = self.world.get("course")
        pts = self.world.get("bounds") if crs == "bounds" else (crs if isinstance(crs, list) else None)
        if not pts or len(pts) < 3:
            return None
        corners = [list(self.frame.to_latlon(float(p[0]), float(p[1]))) for p in pts]
        return {"id": str(self.world.get("course_id") or f"SIM-{str(self.world.get('name') or 'WORLD').upper()}"),
                "catalogue": C.identify(corners), "corners": corners, "transit": []}

    @staticmethod
    def _fingerprint(crs: Optional[dict]) -> Optional[str]:
        if not crs:
            return None
        b = crs.get("boundary") or []
        return f"{crs.get('course_id')}|" + "|".join(f"{float(p[0]):.6f},{float(p[1]):.6f}" for p in b[:4])

    def on_course(self, body: Optional[dict], src: str) -> None:
        """A course heard (the judge's RxCourse, or the OCS's COURSE to the vehicle): kept with where it came from."""
        if not isinstance(body, dict) or len(body.get("boundary") or []) < 3:
            return
        with self.lock:
            self.received = {"course": {"course_id": body.get("course_id"), "boundary": [list(p)[:2] for p in body["boundary"]],
                                        "uav_transit_area": [list(p)[:2] for p in body.get("uav_transit_area") or []],
                                        "pinger_hz": body.get("pinger_hz")}, "src": src, "t": time.time()}

    def course_view(self) -> dict:
        """The course block of /world.json and /course.json (the SIM page's banner + picker; the OCS map's same banner)."""
        from . import courses as C
        mine = self.world_course()
        with self.lock:
            rec = copy.deepcopy(self.received)
            ignored = self.ignored
        out = {"course_id": mine["id"] if mine else None, "course": None, "received": None, "banner": None,
               "courses": [{k: c[k] for k in ("id", "name", "source", "length_m", "width_m", "tasks", "world")} for c in C.catalogue()],
               "worlds": sorted(WORLD_NAMES), "hook": self.hook_result}
        if mine:
            out["course"] = dict(mine, corners_xy=[[round(v, 2) for v in self.frame.to_world(*p)] for p in mine["corners"]],
                                 transit_xy=[[round(v, 2) for v in self.frame.to_world(*p)] for p in mine["transit"]])
        if rec:
            crs = rec["course"]
            match = C.identify(crs["boundary"])
            same = bool(mine) and C.same_ring(mine["corners"], crs["boundary"])
            fp = self._fingerprint(crs)
            out["received"] = {"course_id": crs.get("course_id"), "match": match, "src": rec["src"], "age_s": round(time.time() - rec["t"], 1),
                               "differs": not same, "ignored": fp == ignored, "pinger_hz": crs.get("pinger_hz"),
                               "corners_xy": [[round(v, 2) for v in self.frame.to_world(*p[:2])] for p in crs["boundary"]]}
            if not same and fp != ignored:
                name = match or crs.get("course_id") or "?"
                out["banner"] = (f"course {name} received ({rec['src']}) - the world is on {out['course_id'] or 'no course'}: "
                                 f"move the world there / ignore")
        return out

    def _world_file(self, name: str) -> Optional[str]:
        if name in WORLD_NAMES:
            return os.path.join(KIT, "profiles", "bench", f"world_{name}.yaml")
        return name if os.path.isfile(name) else None

    def set_world(self, body: dict) -> dict:
        """POST /world: move the world to a course, pick a world, or ignore the received course (see the module doc)."""
        from . import courses as C
        if body.get("ignore"):
            with self.lock:
                self.ignored = self._fingerprint((self.received or {}).get("course"))
                rid = ((self.received or {}).get("course") or {}).get("course_id")
            return {"ok": True, "ignored": rid, "course_id": (self.world_course() or {}).get("id"),
                    "why": "the current world stays (a trial run); the banner comes back when another course is received"}
        crs = None
        cid = body.get("course")
        if cid:
            rec = (self.received or {}).get("course")
            if str(cid).lower() == "received":
                crs = C.from_body(rec) if rec else None
            else:
                crs = C.get(cid) or (C.from_body(rec) if rec and str(rec.get("course_id")) == str(cid) else None)
            if crs is None:
                return {"ok": False, "why": f"course {cid!r}: one of {', '.join(C.COURSES)} or 'received' (a course heard from the judge)"}
        wname = body.get("world")
        if wname:
            path = self._world_file(str(wname))
            if path is None:
                return {"ok": False, "why": f"world {wname!r}: one of {', '.join(sorted(WORLD_NAMES))} or a file"}
        elif crs is not None and crs.world and not body.get("keep"):
            path = self._world_file(crs.world)
        else:
            path = self.world_path
        base = W.load(path)
        report = {"fits": True, "outside": [], "from": None, "to": crs.id if crs else None, "rotation_deg": 0.0}
        new, out_path = base, path
        if crs is not None:
            have = W.anchor_course(base)
            if not (have is not None and have.id == crs.id and crs.source != "received"):
                new, report = W.move_to_course(base, crs)
                if not report["fits"] and not body.get("force"):
                    return dict(report, ok=False, why=f"{base.get('name')} does not fit course {crs.id} ({len(report['outside'])} elements "
                                f"outside: {report['outside'][:8]}); pick the course's own world, or force: true")
                stem = str(base.get("name") or "world").replace("/", "_")
                out_path = W.dump(new, os.path.join(self.a.world_dir, f"{stem}@{crs.id}.yaml"))
                new = W.load(out_path)
        with self.lock:
            self.world, self.world_path = new, out_path
            self.frame = W.WorldFrame(new.get("origin"))
            self.phases = W.light_phases(new.get("objects") or [], 1)
            self.lights = new.get("lights") or {}
            self.next_id = 1000 + max([int(o.get("id", 0)) for o in new.get("objects") or []] + [0])
            if crs is not None:
                self.ignored = None
        hook = self.run_hook(out_path)
        tp = body.get("teleport", "start")
        moved = self.teleport({"name": str(tp)}) if tp else None
        return {"ok": True, "world": new.get("name"), "path": out_path, "course_id": (self.world_course() or {}).get("id"),
                "report": report, "hook": hook, "teleport": moved}

    def run_hook(self, path: str) -> str:
        """--world-hook (run_sim.sh: scripts/sim_world.sh): restart fake_perception + fake_dock on the new world file, in the background."""
        hook = getattr(self.a, "world_hook", "") or ""
        if not hook:
            self.hook_result = "no --world-hook: fake_perception / fake_dock keep their world (the SIM page's camera has the new one)"
            return self.hook_result
        cmd = [hook if os.path.isabs(hook) else os.path.join(KIT, hook), path]

        def go():
            try:
                p = subprocess.run(cmd, capture_output=True, text=True, timeout=30, cwd=KIT)
                line = ((p.stdout or "").strip().splitlines() or [(p.stderr or "").strip()[-160:] or f"exit {p.returncode}"])[-1]
            except (OSError, subprocess.TimeoutExpired) as exc:
                line = f"{os.path.basename(hook)} failed ({exc.__class__.__name__})"
            self.hook_result = line[:200]
        self.hook_result = f"{os.path.basename(hook)} {os.path.basename(path)}: running"
        threading.Thread(target=go, daemon=True).start()
        return self.hook_result

    def watch_courses(self) -> None:
        """Follow the judge's course: the hub (--hub-port: rc/course, robocommand/robotx/course, the OCS's COURSE to the vehicle) and
        --rc-port (rc-test's broker: robocommand/robotx/course)."""
        import paho.mqtt.client as mqtt
        from ..ocs import rc_wire

        def make(port: int, topics, name: str):
            def on_connect(c, u, f, rc, p=None):
                for t in topics:
                    c.subscribe(t, 0)

            def on_message(c, u, msg):
                try:
                    if msg.topic.startswith("robocommand/"):
                        self.on_course(rc_wire.decode_course(msg.payload), f"{name} RxCourse")
                        return
                    env = json.loads(msg.payload)
                    if msg.topic == "rc/course":
                        self.on_course(env, f"{name} rc/course")
                        return
                    body = env.get("body", env) if isinstance(env, dict) else {}
                    if isinstance(body, dict) and str(body.get("action", "")).upper() == "COURSE":
                        self.on_course(body.get("args") or {}, "the OCS's COURSE")
                except Exception:
                    pass

            c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"simview-course-{name}-{os.getpid()}", protocol=mqtt.MQTTv311)
            c.on_connect, c.on_message = on_connect, on_message
            c.reconnect_delay_set(1, 5)
            c.connect_async(self.a.rc_host if name == "rc-test" else "127.0.0.1", int(port), keepalive=15)
            c.loop_start()
            return c
        self.course_clients = []
        if getattr(self.a, "hub_port", 0):
            self.course_clients.append(make(self.a.hub_port, ["rc/course", "robocommand/robotx/course", "+/+/+/cmd/+/course"], "hub"))
        if getattr(self.a, "rc_port", 0):
            self.course_clients.append(make(self.a.rc_port, ["robocommand/robotx/course"], "rc-test"))

    def status(self) -> dict:
        return {"source": "sim", "frames": self.n, "age_s": round(time.time() - self.t_frame, 2) if self.n else None,
                "from": f"simview {self.world.get('name')}", "clients": self.clients, "w": self.cam.w, "h": self.cam.h,
                "hfov_deg": self.cam.hfov_deg,
                # v0.1.60 (WP10m): the CAMERA tab's modes - the simulator always has depth (no ZED to share)
                "modes": ["rgb", "depth", "both"], "mode": getattr(self, "cam_mode", "rgb"), "depth": {"available": self.depth is not None, "why": "" if self.depth is not None else "no frame yet"}}

    def depth_jpeg(self) -> Optional[bytes]:
        from ..tools import depthviz
        with self.lock:
            d = self.depth
        return None if d is None else R.jpeg(depthviz.colourise(d), 75)

    def depth_at(self, x: float, y: float) -> dict:
        from ..tools import depthviz
        with self.lock:
            d = self.depth
        return {"ok": d is not None, "x": x, "y": y, "range_m": depthviz.depth_at(d, x, y), **depthviz.scale_bar()}

    def _where(self, body: dict):
        """x_n, y_e in the world frame from x_n / y_e, or from lat / lon (v0.1.35: the OCS map places by lat / lon)."""
        if body.get("lat") is not None and body.get("lon") is not None:
            return self.frame.to_world(float(body["lat"]), float(body["lon"]))
        return float(body["x_n"]), float(body["y_e"])

    def objects(self, body: dict) -> dict:
        op = body.get("op")
        with self.lock:
            objs = self.world.setdefault("objects", [])
            if op == "add":
                kind = body.get("kind", "buoy")
                if kind not in ("buoy", "box", "dock"):
                    return {"ok": False, "why": "kind: buoy | box | dock"}
                x, y = self._where(body)
                self.next_id += 1
                o = {"id": self.next_id, "kind": kind, "colour": str(body.get("colour", "white")), "x_n": round(x, 3), "y_e": round(y, 3),
                     "mock": True}
                if kind == "buoy":
                    light = str(body.get("light", "off"))
                    if light not in ("off", "steady", "flash"):
                        return {"ok": False, "why": "light: off | steady | flash"}
                    o["light"] = light
                elif kind == "box":
                    o["light"] = "steady"
                if kind == "dock":
                    o["bay"] = int(body.get("bay") or 1)
                for k in ("size_m", "width_m", "face_hdg_deg"):
                    if body.get(k) is not None:
                        o[k] = float(body[k])
                objs.append(o)
                return {"ok": True, "id": o["id"]}
            target = next((o for o in objs if o.get("id") == body.get("id")), None)
            if target is None:
                return {"ok": False, "why": f"no object {body.get('id')}"}
            if op == "move":
                x, y = self._where(body)
                target["x_n"], target["y_e"] = round(x, 3), round(y, 3)
                return {"ok": True}
            if op == "del":
                objs.remove(target)
                return {"ok": True}
        return {"ok": False, "why": "op: add | move | del"}

    def fcu_post(self, body: dict) -> dict:
        if "set" in body:
            s = {k: v for k, v in (body.get("set") or {}).items() if k in FCU_SET_KEYS}
            if "mode" in s and int(s["mode"]) not in (0, 4):
                return {"ok": False, "why": "mode: 0 MANUAL | 4 HOLD (the transmitter's mode switch)"}
            r = self.fcu.ask({"set": s})
        elif "pause_stream" in body:
            r = self.fcu.ask({"pause_stream": float(body["pause_stream"])})
        elif "teleport" in body:
            t = body["teleport"]
            lat, lon = self.frame.to_latlon(float(t["x_n"]), float(t["y_e"]))
            r = self.fcu.ask({"teleport": {"lat": lat, "lon": lon, "hdg": float(t.get("hdg", self.pose[2]))}})
        else:
            return {"ok": False, "why": "set | pause_stream | teleport"}
        return {"ok": r is not None, "reply": r, "why": "" if r is not None else "the fake FCU did not answer (is the bench up?)"}


def make_handler(sim: Sim):
    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj, default=str).encode(), "application/json")

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html", "/sim.html"):
                with open(PAGE, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            elif path == "/themes.css":                     # v0.1.47: the five palettes (tools/themes.css)
                with open(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "tools", "themes.css"), "rb") as fh:
                    self._send(200, fh.read(), "text/css; charset=utf-8")
            elif path == "/prefs.json":                     # v0.1.47: the OCS page's theme, proxied (another port: no CORS)
                self._json(sim.ocs_prefs())
            elif path == "/status":
                self._json(sim.status())
            elif path == "/world.json":
                self._json(sim.world_json())
            elif path == "/course.json":                    # v0.1.53: the course block alone (the OCS map's banner)
                self._json(sim.course_view())
            elif path == "/frame.npz":
                from urllib.parse import parse_qs, urlparse
                q = parse_qs(urlparse(self.path).query)
                w, h = int((q.get("w") or [1104])[0]), int((q.get("h") or [621])[0])
                if not (64 <= w <= 2208 and 36 <= h <= 1242):
                    return self._json({"ok": False, "why": "w 64..2208, h 36..1242"}, 400)
                self._send(200, sim.frame_npz(w, h), "application/octet-stream")
            elif path == "/markers.json":
                self._json(sim.ocs_markers())
            elif path == "/depth.jpg":                      # v0.1.60 (WP10m)
                b = sim.depth_jpeg()
                self._send(200 if b else 503, b or b"no frame yet", "image/jpeg" if b else "text/plain")
            elif path == "/depth_at":
                from urllib.parse import parse_qs, urlparse
                q = parse_qs(urlparse(self.path).query)
                try:
                    self._json(sim.depth_at(float((q.get("x") or ["-1"])[0]), float((q.get("y") or ["-1"])[0])))
                except ValueError:
                    self._json({"ok": False, "why": "x, y: numbers 0..1"}, 400)
            elif path in ("/snapshot.jpg", "/chase.jpg"):
                with sim.lock:
                    b = sim.jpeg if path == "/snapshot.jpg" else sim.chase
                self._send(200 if b else 503, b or b"no frame yet", "image/jpeg" if b else "text/plain")
            elif path == "/stream":
                self.stream()
            else:
                self._send(404, b"not found", "text/plain")

        def stream(self):
            self.send_response(200)
            self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={BOUNDARY}")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            last = -1
            with sim.lock:
                sim.clients += 1
            try:
                while not sim.stop.is_set():
                    with sim.cond:
                        sim.cond.wait_for(lambda: sim.n != last or sim.stop.is_set(), timeout=2.0)
                        j, last = sim.jpeg, sim.n
                    if not j:
                        continue
                    self.wfile.write(f"--{BOUNDARY}\r\nContent-Type: image/jpeg\r\nContent-Length: {len(j)}\r\n\r\n".encode() + j + b"\r\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass
            finally:
                with sim.lock:
                    sim.clients -= 1

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._json({"ok": False, "why": "bad json"}, 400)
            if self.headers.get("X-Sim-Web") != "1":
                return self._json({"ok": False, "why": "missing X-Sim-Web header"}, 403)
            if self.path == "/mode":                         # v0.1.60 (WP10m): the CAMERA tab's mode (the simulator always has depth)
                m = str(body.get("mode", ""))
                if m not in ("rgb", "depth", "both"):
                    return self._json({"ok": False, "why": "mode: rgb, depth or both"}, 400)
                sim.cam_mode = m
                return self._json({"ok": True, "mode": m})
            if self.path == "/sticks":
                r = sim.fcu.ask({"sticks": {k: body.get(k, 0.0) for k in ("surge", "yaw", "sway")}}, timeout_s=0.2)
                self._json({"ok": r is not None, "mode": (r or {}).get("mode")})
            elif self.path == "/fcu":
                self._json(sim.fcu_post(body))
            elif self.path == "/prefs":                      # v0.1.47: this page's theme select writes the OCS /prefs
                self._json(sim.ocs_prefs(str(body.get("theme", "")) or None))
            elif self.path == "/teleport":
                try:
                    self._json(sim.teleport(body))
                except (TypeError, ValueError) as exc:
                    self._json({"ok": False, "why": f"bad teleport: {exc}"}, 400)
            elif self.path == "/waypoint":
                try:
                    self._json(sim.waypoint(body))
                except (KeyError, TypeError, ValueError) as exc:
                    self._json({"ok": False, "why": f"bad waypoint: {exc}"}, 400)
            elif self.path == "/markers":
                self._json(sim.ocs_markers(body))
            elif self.path == "/reset":
                if str(body.get("confirm", "")).strip().lower() != "reset":
                    return self._json({"ok": False, "why": "RESET SIM needs confirm: \"reset\" (typed)"}, 400)
                self._json(sim.reset())
            elif self.path == "/world":                      # v0.1.53: move the world to a course / pick a world / ignore
                try:
                    self._json(sim.set_world(body))
                except (OSError, KeyError, TypeError, ValueError) as exc:
                    self._json({"ok": False, "why": f"bad world: {exc}"}, 400)
            elif self.path == "/objects":
                try:
                    self._json(sim.objects(body))
                except (KeyError, TypeError, ValueError) as exc:
                    self._json({"ok": False, "why": f"bad object: {exc}"}, 400)
            else:
                self._json({"ok": False, "why": "unknown"}, 404)
    return H


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = str(self.server_address[0]), int(self.server_address[1])


WORLD_NAMES = {"empty", "bench", "pool", "por", "boxes", "marina", "extra"}


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="profiles/bench/world_pool.yaml", help="a world file, or empty | bench | pool | por | boxes | marina | extra")
    ap.add_argument("--fcu-control", type=int, default=14560, help="the fake FCU's control port (the bench: 14560)")
    ap.add_argument("--hub-port", type=int, default=1882, help="the hub broker (RESET SIM: ocs_cli and the fake judge; 0 = the plant and the world only)")
    ap.add_argument("--ocs-url", default=os.environ.get("OCS_URL", "http://127.0.0.1:8096/"), help="the OCS page (markers, v0.1.35)")
    ap.add_argument("--points", default=POINTS, help="the saved points (usv_ui/points.json: HOME, the teleport snap)")
    ap.add_argument("--port", type=int, default=int(os.environ.get("SIM_PORT", "8097")))
    ap.add_argument("--bind", default="127.0.0.1")
    ap.add_argument("--hz", type=float, default=6.0, help="frames per second of the camera and the chase view")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=360)
    ap.add_argument("--hfov", type=float, default=None, help="a symmetric pinhole of this field of view; default the ZED's own intrinsics (sim/zed.py, 50.9 deg)")
    ap.add_argument("--cam-height", type=float, default=None, help="the camera above the water, m (default: the world's camera.height_m, else 0.8; v0.1.79)")
    ap.add_argument("--chase-width", type=int, default=640)
    ap.add_argument("--chase-height", type=int, default=360)
    ap.add_argument("--open", action="store_true", help="open the SIM page in the browser (macOS)")
    ap.add_argument("--boat-port", type=int, default=0, help="v0.1.46: the boat broker with fake_dock's sim/dock/state (the bench: 1884) - "
                    "the dock's indicator and window lights in the camera; 0 = off (the world file's colours, windows dark)")
    ap.add_argument("--boat-host", default="127.0.0.1")
    ap.add_argument("--rc-port", type=int, default=0, help="v0.1.53: rc-test's broker (1885) - its RxCourse for the course banner; 0 = off")
    ap.add_argument("--rc-host", default="127.0.0.1")
    ap.add_argument("--world-dir", default=os.path.join(KIT, "logs", "sim", "worlds"), help="v0.1.53: where a world moved to a course is written")
    ap.add_argument("--world-hook", default="", help="v0.1.53: run with the moved world's file (run_sim.sh: scripts/sim_world.sh - fp + dock "
                    "restarted on it); empty = none")
    a = ap.parse_args(argv)
    if a.world in WORLD_NAMES:
        a.world = f"profiles/bench/world_{a.world}.yaml"
    return a


def main(argv=None) -> int:
    a = parse_args(argv)
    sim = Sim(a)
    sim.watch_dock()
    sim.watch_courses()
    th = threading.Thread(target=sim.loop, daemon=True)
    th.start()
    srv = Server((a.bind, a.port), make_handler(sim))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    url = f"http://127.0.0.1:{a.port}/"
    print(f"[simview] the simulator on {url} - world {a.world}, the fake FCU on udp {a.fcu_control}; the camera for the OCS page: CAM_URL={url}", flush=True)
    if a.open and sys.platform == "darwin":
        subprocess.Popen(["open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        srv.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop.set()
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
