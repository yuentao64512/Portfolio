"""fast_sil: one task, the sim camera, the PerceptionMap and the fake FCU's plant in one process, in simulated time.

The same pieces as the bench (sim/fake_fcu.Plant and its GUIDED / velocity / HOLD laws, sim/fake_perception's camera
model on a world file, usv/perception_map, the task layer) without brokers or MAVLink, so a whole Task 1 runs in a few
seconds of wall time - for unit tests and for tuning (ROADMAP WP2). The multi-process SIL (tests/test_t1_sim.py) proves
the same task through the real mission manager.

  from optiprimex.sim.fast_sil import FastSil
  sil = FastSil("profiles/bench/world_pool.yaml", "T1", {"planner": "s6"})
  result = sil.run(max_s=400)        # {"state", "states", "reports", "track", "t"}

v0.1.39 (WP10h): `perception="camera"` puts the buoys through the camera path instead of the geometric fake - the renderer's frame
at the pose (sim/render, the ZED's intrinsics at `cam_size`), cv/sim_observer (the students' measuring code + our rules, the ZED-like
depth), cv_publish's item conversion, then the same PerceptionMap.ingest; docks, windows and boxes stay geometric (as run_sim.sh
--perception camera). The camera runs at `cam_hz` (simulated time).

v0.1.48 (WP10j-b): `cam_dock=True` puts the dock through the camera too (dock_face / window / indicator: cv/sim_observer.observe_dock,
cv_publish.dock_to_items; the geometric fake then skips dock and window); `detector="model", model=DIR` runs a CV_TRAIN model (e.g.
../CV_TRAIN/models/cvt-v2-l) on the rendered frame instead of the renderer's own masks - cam_dock defaults to on with it.
"""
from __future__ import annotations

import math
import time
from typing import Any, Dict, List, Optional

from ..companion.config import Config
from ..companion.frames import ControlOrigin
from ..companion.state import FcuState, GuidedTarget, Hold, LocalPose, RunContext, ThrusterFrame, VelocityYawTarget
from ..usv.behaviours.base import BehaviourContext
from ..usv.perception_map import PerceptionMap
from ..usv.tasks import REGISTRY
from ..usv.tasks.base import TaskContext
from .fake_fcu import Plant
from .fake_perception import FakePerception, parse_args as fp_args
from . import world as W
from .fake_dock import DockSim


class FastSil:
    def __init__(self, world: str, task: str, args: Optional[Dict[str, Any]] = None, cfg: Optional[Config] = None, seed: int = 3,
                 dt: float = 0.1, wp_speed: float = 2.0, start: Optional[Dict[str, float]] = None, warmup_s: float = 0.0,
                 perception: str = "geometric", cam_hz: float = 2.5, cam_size: tuple = (736, 414), depth_noise: str = "zed",
                 detector: str = "synthetic", model: Optional[str] = None, cam_dock: Optional[bool] = None, device: Optional[str] = None,
                 uav: Optional[Dict[str, Any]] = None, guard: Optional[Dict[str, Any]] = None, hull: Optional[str] = None,
                 str_ramp_s: Optional[float] = None, perception_hz: Optional[float] = None,
                 perception_gaps: Optional[List[float]] = None):
        self.cfg = cfg or Config()
        wd = W.load(world)
        o = wd.get("origin") or {"lat": 1.2810202, "lon": 103.8550097}      # v0.1.33: Course Alpha's centre
        st = start or wd.get("start") or {}
        lat, lon = W.WorldFrame(o).to_latlon(float(st.get("x_n", 0.0)), float(st.get("y_e", 0.0)))
        pl = (self.cfg.extra or {}).get("plant") or {}          # v0.1.85 (WP41 T0.4): the hull (ideal | omni_usv) and the steering lag
        self.plant = Plant(lat, lon, float(st.get("hdg_deg", 0.0)), hull=hull or pl.get("hull") or "ideal",
                           str_ramp_s=float(str_ramp_s if str_ramp_s is not None else pl.get("str_ramp_s") or 0.0))
        if pl.get("r0_dps") is not None:                       # v0.1.94: the hull's yaw authority from the profile's plant block
            self.plant.r0_dps = float(pl["r0_dps"])
        if pl.get("r05_dps") is not None:
            self.plant.r05_dps = float(pl["r05_dps"])
        self.origin = ControlOrigin(o["lat"], o["lon"])
        if perception not in ("geometric", "camera"):
            raise ValueError(f"perception {perception!r}: geometric | camera")
        self.perception = perception
        if detector not in ("synthetic", "model") or (detector == "model" and not model):
            raise ValueError("detector synthetic | model (with model=DIR)")
        cam_dock = (detector == "model") if cam_dock is None else bool(cam_dock)
        skip = ("buoy,dock,window" if cam_dock else "buoy") if perception == "camera" else ""
        self.fp = FakePerception(fp_args(["--world", world, "--hub-port", "0", "--seed", str(seed)] + (["--skip-kinds", skip] if skip else [])))
        self.cam = CameraPath(wd, seed, cam_size, depth_noise, detector, model, cam_dock, device) if perception == "camera" else None
        self.cam_period, self.cam_next = 1.0 / max(0.2, cam_hz), 0.0
        # v0.1.92: the observer's real rate (12 Nov: 0.8-2.2 fps) - the geometric camera and the guard only every 1 / perception_hz
        # (None = every tick, as before); the map and the task keep the tick
        self.per_period, self.per_next = (1.0 / float(perception_hz) if perception_hz else 0.0), 0.0
        # v0.1.94 (real_13nov): the frame gaps drawn from a measured list (13 Nov afternoon: 0.13 / 0.34 / 0.87 s at 10 / 50 / 90 %)
        self.per_gaps = list(perception_gaps or [])
        import random as _r
        self._gap_rng = _r.Random(seed ^ 0x6A95)
        self.dock = DockSim(wd, seed) if any(o.get("kind") == "dock" for o in wd.get("objects") or []) else None
        self.aim: Optional[dict] = None
        self.aims: List[dict] = []
        self.map = PerceptionMap(self.cfg)
        # v0.1.66 (WP16, D23): an in-process fake UAV (sim/fake_uav.Survey) - its messages through ingest_geo once a second of sim time,
        # from the task's start (= our RunStart); uav = {noise_m, dropout, colour_ok, survey_s, complete, seed, delay_s}
        self.uav_survey, self.uav_next, self.uav_t0, self.uav_msgs = None, 0.0, None, 0
        if uav is not None:
            from .fake_uav import Survey
            u = dict(uav)
            self.uav_delay = float(u.pop("delay_s", 2.0))
            self.uav_survey = Survey({"origin": o, "objects": wd.get("objects") or []}, **{k: v for k, v in u.items()
                                     if k in ("noise_m", "dropout", "colour_ok", "footprint_m", "speed", "lane_m", "survey_s", "complete", "seed")})
            self.map.uav_ocs = {"active": True, "declared": True, "flight": {}}
        # v0.1.80 (WP36): the ZED guard - sim/fake_guard from the world's geometry at the camera rate, usv/guard's supervisor between the
        # task and the plant (as the mission manager); guard = {fake: {...FakeGuard cfg}, sup: {...zed_guard cfg}, silent_after_s: N}
        self.guard_fake = self.guard_sup = None
        self.guard_msgs: List[dict] = []
        if guard is not None:
            from .fake_guard import FakeGuard
            from ..usv.guard import GuardSupervisor
            self.guard_fake = FakeGuard(wd, guard.get("fake"))
            self.guard_sup = GuardSupervisor(guard.get("sup"))
            self.guard_silent_after = guard.get("silent_after_s")
        self.dt, self.wp_speed = dt, wp_speed
        self.t = 1000.0
        self.events: List[dict] = []
        self.event_log: List[tuple] = []
        self.states: List[str] = []
        self.logs: List[str] = []
        self.track: List[tuple] = []
        self.safe_holds: List[str] = []
        self.task = REGISTRY[task](self.cfg, on_transition=lambda ev: self.states.append(ev.dst),
                                   clock=lambda: self.t)
        if warmup_s:
            self.warmup(warmup_s)                    # the camera fills the map before START_TASK (args.from needs it)
        self.task.start(dict(args or {}), self.ctx())
        self.steps: List[str] = []                 # v0.1.26: the step-mode parks, in order ("step: next X")
        self.from_refused = ""
        if (args or {}).get("from"):                 # as the task runner does (the command handler checked it first)
            self.from_refused = self.task.enter_from(str(args["from"]), self.ctx())
        self.cmd = Hold()
        self.speed = wp_speed

    def pose(self) -> LocalPose:
        x, y = self.origin.to_local(self.plant.lat, self.plant.lon)
        vn, ve = self.plant.vn_ve
        return LocalPose(x, y, self.plant.hdg_deg, self.origin.lat, self.origin.lon, self.t, sog_mps=math.hypot(vn, ve))

    def ctx(self) -> TaskContext:
        fcu = FcuState(t_mono=self.t, lat=self.plant.lat, lon=self.plant.lon, hdg_deg=self.plant.hdg_deg)
        b = BehaviourContext(cfg=self.cfg, now=self.t, fcu=fcu, pose=self.pose(), origin=self.origin, run=RunContext(), map=self.map)
        return TaskContext(b=b, emit_event=self.on_event, log=self.logs.append)

    def on_event(self, ev) -> None:
        body = dict(ev.body)
        self.events.append(body)
        self.event_log.append((body.get("event"), self.task.m.state, round(self.t - 1000.0, 1)))   # v0.1.48: the stage it came from
        if body.get("event") == "docking" and self.dock is not None:     # as fake_dock does on event/<v>/docking
            self.dock.command({"cmd": "docked", "bay": body.get("bay")}, self.t)

    def physics(self) -> None:
        """The fake FCU's control laws (sim/fake_fcu.FakeFcu.physics) for the last behaviour output."""
        p, c = self.plant, self.cmd
        u_t = v_t = r_t = 0.0
        if isinstance(c, GuidedTarget):
            if c.speed_mps:
                self.speed = c.speed_mps
            tx, ty = p.to_local(c.lat, c.lon)
            dx, dy = tx - p.x, ty - p.y
            dist = math.hypot(dx, dy)
            if dist > 1.0:
                err = (math.atan2(dy, dx) - p.psi + math.pi) % (2 * math.pi) - math.pi
                r_t = max(-Plant.R_MAX, min(Plant.R_MAX, 1.5 * err))
                u_t = min(self.speed, 0.5 * dist) if abs(err) < math.radians(60) else 0.15 * self.speed
        elif isinstance(c, ThrusterFrame):                     # mode 20: the fake FCU's mixer (sim/fake_fcu physics)
            m1, m2, m3, m4 = c.as_list()
            u_t, r_t = (m1 + m3) / 2.0 * Plant.U_MAX, (m1 - m3) / 2.0 * Plant.R_MAX
            v_t = (m2 / 0.957 + m4) / 2.0 * Plant.V_MAX
        elif isinstance(c, VelocityYawTarget):
            u_t = max(-Plant.U_MAX, min(Plant.U_MAX, c.vx_mps))
            r_t = max(-Plant.R_MAX, min(Plant.R_MAX, c.yaw_rate_rad_s))
        p.step(self.dt, u_t, v_t, r_t)

    def sense(self) -> None:
        pz = self.pose()
        self.fp.pose, self.fp.pose_t = {"x_n": pz.x_n, "y_e": pz.y_e, "hdg_deg": pz.hdg_deg, "lat": None, "lon": None}, time.monotonic()
        if self.dock is not None:
            self.dock.tick(self.t, {"x_n": pz.x_n, "y_e": pz.y_e, "hdg_deg": pz.hdg_deg}, self.aim, 0.0 if self.aim else 1e9)
            self.fp.dock_state = self.dock.state(self.t)
            if self.cam is not None:
                self.cam.dock_state = self.fp.dock_state          # v0.1.46: the camera shows the same lights
        self.map.last_alive_t = self.t                     # the sim camera's health beat (v0.1.27: m0t4's preflight)
        seen = self.t >= self.per_next
        if seen:
            self.per_next = self.t + (self._gap_rng.choice(self.per_gaps) if self.per_gaps else self.per_period)
        for obs in (self.fp.observe(self.t).values() if seen else ()):
            self.map.ingest(obs, pz, now=self.t)
        if self.cam is not None and self.t >= self.cam_next:
            self.cam_next = self.t + self.cam_period
            self.map.ingest(self.cam.observe(pz, self.t, self.plant.lat, self.plant.lon), pz, now=self.t)
            for obs in self.cam.last_dock_obs:                 # v0.1.48: the dock parts of the same frame
                self.map.ingest(obs, pz, now=self.t)
        if self.uav_survey is not None:
            if self.uav_t0 is None:
                self.uav_t0 = self.t + self.uav_delay
            if self.t >= self.uav_t0 and self.t >= self.uav_next:
                self.uav_next = self.t + 1.0
                msg = self.uav_survey.message(self.t - self.uav_t0)
                if msg is not None:
                    self.uav_msgs += 1
                    self.map.ingest_geo(msg, self.origin, self.t)
        if seen and self.guard_fake is not None and (self.guard_silent_after is None or self.t - 1000.0 < float(self.guard_silent_after)):
            msg = self.guard_fake.frame(pz.x_n, pz.y_e, pz.hdg_deg, self.t)
            self.guard_msgs.append(dict(msg, t_sim=round(self.t - 1000.0, 1)))
            self.guard_sup.on_message(msg, self.t, self.map)
            self.guard_sup.map_far(msg, pz, self.map, self.t)           # v0.1.81.1: the 3-8 m clusters -> depth obstacles
        self.map.age_out(self.t)

    def warmup(self, secs: float) -> None:
        """The boat waits where it is (the camera runs) before the task starts - as on the hosts before start_task."""
        for _ in range(int(secs / (2 * self.dt))):
            self.sense()
            self.t += 2 * self.dt

    def run(self, max_s: float = 600.0, until=None, resume_steps: bool = True) -> Dict[str, Any]:
        n = 0
        t_end = self.t + max_s
        while self.t < t_end:
            if n % 2 == 0:
                self.sense()                                  # the camera at 5 Hz, the tick at 10 Hz
            ctx = self.ctx()
            if self.guard_sup is not None and self.guard_sup.busy:          # v0.1.80: BRAKE / BACKOFF own the boat; the task waits
                self.cmd, _ = self.guard_sup.filter(self.task, None, ctx)
                self.physics()
                self.track.append((round(self.t - 1000.0, 1), round(self.pose().x_n, 2), round(self.pose().y_e, 2), round(self.pose().hdg_deg),
                                   "GUARD_" + self.guard_sup.mode)) if n % 5 == 0 else None
                self.t += self.dt
                n += 1
                continue
            st = self.task.step(ctx)
            if resume_steps and str(getattr(self.task, "safe_hold", "") or "").startswith("step: "):
                self.steps.append(self.task.safe_hold)         # the operator's `resume` at once (the boat held for this tick)
                self.task.safe_hold = None
            if getattr(self.task, "safe_hold", None):
                self.safe_holds.append(self.task.safe_hold)
                self.task.safe_hold = None
                self.task.pause()
                self.cmd = Hold()                             # v0.1.56: the vehicle's AUTO_SAFE_HOLD holds (not the last frame)
                return self.result("SAFE_HOLD")
            if st.done or st.aborted:
                return self.result("DONE" if st.done else "ABORTED", st.reason)
            if st.out is not None and st.out.cmd is not None:
                self.cmd = st.out.cmd
                if self.guard_sup is not None:                                  # v0.1.80: the supervisor between the task and the plant
                    self.cmd, _ = self.guard_sup.filter(self.task, st.out.cmd, ctx)
                    if self.guard_sup.safe_hold:
                        self.safe_holds.append(self.guard_sup.safe_hold)
                        self.guard_sup.safe_hold = None
                        self.task.pause()
                        self.cmd = Hold()
                        return self.result("SAFE_HOLD")
            self.aim = (st.out.info.get("cannon") if st.out is not None else None)
            if self.aim and (not self.aims or self.aims[-1] is not self.aim):
                self.aims.append(self.aim)
            self.physics()
            pz = self.pose()
            if n % 5 == 0:
                self.track.append((round(self.t - 1000.0, 1), round(pz.x_n, 2), round(pz.y_e, 2), round(pz.hdg_deg), self.task.m.state))
            self.t += self.dt
            n += 1
            if until is not None and until(self):
                return self.result(self.task.m.state)
        return self.result("TIMEOUT")

    def result(self, how: str, reason: str = "") -> Dict[str, Any]:
        return {"state": how, "reason": reason, "states": list(self.states), "reports": [e for e in self.events if e.get("event") == "safe_passage"],
                "events": list(self.events), "dock": self.dock.state(self.t) if self.dock else None, "aimed": len(self.aims),
                "track": self.track, "t": round(self.t - 1000.0, 1), "logs": self.logs, "safe_holds": self.safe_holds,
                "sub_state": self.task.m.state, "steps": list(self.steps), "from_refused": self.from_refused,
                "event_log": list(self.event_log), "search": list(getattr(self.task, "search_log", []) or []),
                "reengaged": getattr(self.task, "reengaged", 0),
                "uav": self.map.uav_digest(self.t) if self.uav_survey is not None else None, "uav_msgs": self.uav_msgs}


class CameraPath:
    """The buoys through the camera: render -> cv/sim_observer.observe -> cv_publish's items -> an Observation (v0.1.39)."""

    def __init__(self, wd: dict, seed: int, size: tuple, depth_noise: str = "zed", detector: str = "synthetic", model: Optional[str] = None,
                 dock: bool = False, device: Optional[str] = None):
        import importlib.util
        import os
        import types
        from . import render as R
        kit = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

        def load(name, rel):
            spec = importlib.util.spec_from_file_location(name, os.path.join(kit, rel))
            m = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(m)
            return m
        self.R, self.wd = R, wd
        self.SO = _cached("sim_observer", lambda: load("fast_sil_sim_observer", "cv/sim_observer/sim_observer.py"))
        self.pub = types.SimpleNamespace(a=types.SimpleNamespace(colour="unknown"))
        pubmod = _cached("cv_publish", lambda: load("fast_sil_cv_publish", "cv/publish/cv_publish.py"))
        self.to_items, self.dock_to_items = pubmod.Publisher.frame_to_items, pubmod.dock_to_items
        self.args = self.SO.parse_args(["--depth-noise", depth_noise, "--size", f"{size[0]}x{size[1]}"]
                                       + (["--detector", "model", "--model", model] if detector == "model" else []))
        self.detector, self.dock = detector, dock
        self.det = _cached(f"model:{model}:{device}", lambda: self.SO.ModelDetector(model, device)) if detector == "model" else None
        self.last_dock_obs: List[Any] = []
        self.wall_infer = 0.0
        self.cam = R.Camera(w=int(size[0]), h=int(size[1]), height_m=float((wd.get("camera") or {}).get("height_m", 0.8)))   # v0.1.79: the mount
        self.phases = W.light_phases(wd.get("objects") or [], seed)
        self.lights = wd.get("lights") or {}
        self.n = 0
        self.dock_state = None

    def lit(self, t: float) -> Dict[int, bool]:
        per, on = float(self.lights.get("period_s", 2.0)), float(self.lights.get("on_s", 1.0))
        return {o.get("id"): ((t + self.phases.get(o.get("id"), 0.0)) % per) < on for o in self.wd.get("objects") or [] if o.get("light") == "flash"}

    def frame(self, pz: LocalPose, t: float, lat: float, lon: float) -> dict:
        import json
        import numpy as np
        R, cam = self.R, self.cam
        sc = R.scene_from_world(self.wd, self.lit(t), dock_state=self.dock_state)
        rgb, depth, pid = R.render_ids(sc, pz.x_n, pz.y_e, pz.hdg_deg, cam)
        codes, objs, kinds = [], [], []
        for i, sp in enumerate(sc.spheres):
            k, o = sp.tag.split(":", 1)
            codes.append(i), objs.append(int(o)), kinds.append("buoy" if k in ("buoy", "light") else k)
        for j, bx in enumerate(sc.boxes):
            k, o = bx.tag.split(":", 1)
            codes.append(R.BOX_ID0 + j), objs.append(int(o)), kinds.append(k)
        self.n += 1
        meta = {"t": 1.7e9 + t, "frame": self.n, "pose": {"x_n": pz.x_n, "y_e": pz.y_e, "hdg_deg": pz.hdg_deg, "lat": lat, "lon": lon, "gps_fix": 3},
                "camera": {"w": cam.w, "h": cam.h, "fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy}}
        return {"rgb": rgb, "depth": depth, "ids": pid, "prim_code": np.array(codes), "prim_obj": np.array(objs), "prim_kind": np.array(kinds),
                "meta": np.array(json.dumps(meta))}

    def observe(self, pz: LocalPose, t: float, lat: float, lon: float):
        from ..companion.state import Observation
        fr = self.frame(pz, t, lat, lon)
        if self.det is not None:
            t0 = time.time()
            buoys, dock = self.det.detect_all(fr)
            self.wall_infer += time.time() - t0
        else:
            buoys = self.SO.synthetic_detections(fr)
            dock = self.SO.synthetic_dock_detections(fr) if self.dock else None
        payload = self.SO.observe(fr, buoys, self.args, dock if self.dock else None)
        items = [it for i, b in enumerate(payload["buoys"])
                 for it in self.to_items(self.pub, dict(b, status=payload["status"]), payload["frame"], b.get("colour"), b.get("index", i))]
        self.last = payload
        self.last_dock_obs = []
        if self.dock:
            di = self.dock_to_items(payload.get("dock") or [], payload["frame"])
            self.last_dock_obs = [Observation.from_body({"t_cam": t, "kind": k, "items": v}) for k, v in di.items() if v]
        return Observation.from_body({"t_cam": t, "kind": "buoys", "items": items})


_CACHE: Dict[str, Any] = {}


def _cached(key: str, make):
    if key not in _CACHE:
        _CACHE[key] = make()
    return _CACHE[key]
