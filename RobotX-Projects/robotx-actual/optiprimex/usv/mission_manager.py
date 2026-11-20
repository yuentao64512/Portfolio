"""mission_manager: one process, 10 Hz tick (MODULES 3.2).

tick: fcu = link.state() -> pose -> health -> vehicle_fsm.feed -> commands -> task_runner.step ->
adapter.apply/flush -> publishers. Starts in V0, commands nothing until the FCU is seen, lands in V2.

v0.1.68 (WP17, docs/PROMPT_FREE_RUN_2026-11-08.md): `--free-run` (a profile with free_run.allowed - the lab profile only) runs every
stage against the in-process ghost plant (usv/ghost.py): the pose and the origin come from the ghost, the adapter is muted (mode
requests only to the FCU), the tasks see the ghost's lat / lon / hdg in their FCU snapshot; the real FCU stays in the loop for
heartbeat, armed, RC, kill, RC6, faults. free_run.armed: refuse (START_TASK refused armed, arming mid-task aborts to
AUTO_SAFE_HOLD) | warn (one event + the banner, the task goes on). The companion still never arms or disarms.
"""
from __future__ import annotations

import argparse
import asyncio
import json
import logging
import math
import signal
import queue
import dataclasses
import sys
import time
from typing import Tuple, Optional

from ..companion import config as cfgmod
from ..companion import tuning
from ..companion import speed_k as SK
from ..companion.config import Config
from ..companion.frames import ControlOrigin, PoseBuffer
from ..companion.health import HealthMonitor
from ..companion.link import MavLink
from ..companion.mqtt import Bus
from ..companion.state import Event, GuidedTarget, Hold, LocalPose, Observation, TaskStateEvent, ThrusterFrame, VState, VehicleState, station_keep_mode, utc_iso
from .adapters.rover import RoverAdapter
from .behaviours.base import BehaviourContext
from .command_handler import CommandHandler
from .ocs_gateway import OcsGateway
from . import points_store
from ..uav import sgit_map
from .guard import GuardSupervisor
from .perception_map import PerceptionMap
from .publishers import EventPublisher, LocalStatePublisher, MapDigestPublisher, StatusPublisher
from .task_runner import TaskRunner
from .tasks.base import TaskContext
from .vehicle_fsm import VehicleFsm

log = logging.getLogger("optiprimex.mm")


FREE_RUN_REFUSED = "--free-run needs a profile with free_run.allowed (the lab profile); never at the pool"
FREE_RUN_ARMED = "FREE RUN: FCU armed"


class MissionManager:
    def __init__(self, cfg: Config, link: Optional[MavLink] = None, bus: Optional[Bus] = None, clock=None, ghost=None):
        self.cfg = cfg
        self.clock = clock or time.monotonic
        self.link = link or MavLink(cfg)
        self.bus = bus or Bus(cfg, client_id=cfg.mqtt.client_id, lwt_topic=cfg.team("status", cfg.vehicle), source="mission_manager")
        self.vs = VehicleState()
        self.health = HealthMonitor(cfg)
        self.vs.profile, self.vs.link_loss_action = cfg.profile, cfg.guards.link_loss_action
        if cfg.guards.ignore_faults or not cfg.guards.require_ekf_for_auto or not cfg.guards.require_armed_for_auto:
            self.vs.lab = True
            log.warning("LAB GUARDS: ignore_faults=%s require_ekf_for_auto=%s require_armed_for_auto=%s - bench / lab only, never at the pool",
                        cfg.guards.ignore_faults, cfg.guards.require_ekf_for_auto, cfg.guards.require_armed_for_auto)
        self._link_down_since: Optional[float] = None     # guards.link_loss_action: when the bridge went down (None = up)
        self.origin: Optional[ControlOrigin] = None
        if cfg.frames.origin_lat is not None and cfg.frames.origin_lon is not None:
            self.origin = ControlOrigin(cfg.frames.origin_lat, cfg.frames.origin_lon)
        self.poses = PoseBuffer()
        self.events = EventPublisher(self.bus, cfg)
        self.fsm = VehicleFsm(cfg, on_transition=self._on_transition, clock=self.clock)
        self.runner = TaskRunner(cfg, on_transition=self._on_transition, clock=self.clock)
        self.adapter = RoverAdapter(self.link, cfg)
        self.adapter.clock = self.clock
        # v0.1.68 (WP17): free run - the ghost plant (usv/ghost.GhostNav); None = the FCU's position, as always
        self.ghost = ghost
        self.free_run_armed = str((cfg.extra.get("free_run") or {}).get("armed", "refuse"))
        self._fr_armed_prev = False
        if ghost is not None:
            if self.free_run_armed not in ("refuse", "warn"):
                raise ValueError(f"free_run.armed {self.free_run_armed!r}: refuse | warn")
            self.adapter.ghost = ghost
            self.vs.nav, self.vs.motion = "ghost", "muted"
            if self.origin is None:
                self.origin = ControlOrigin(ghost.start[0], ghost.start[1])
            log.warning("FREE RUN: ghost position from %.7f %.7f hdg %.0f, no motion to the FCU (mode requests only); armed: %s - "
                        "bench only, props off, ESC power off, NEVER at the pool", ghost.start[0], ghost.start[1], ghost.start[2],
                        self.free_run_armed)
        self.cmd = CommandHandler(cfg, self.bus, self.fsm, self.vs.run, self.runner, self.clock)
        self.cmd.link = self.link                           # v0.1.50: FCU_MODE / FCU_ARM from the pages
        self.cmd.on_tuned = self._on_tuned                  # v0.1.49: SET_TUNING -> a `tune` line in the command trace
        if ghost is not None:
            self.cmd.free_run = {"armed": self.free_run_armed}   # v0.1.68: SKIP allowed; START_TASK refused armed (refuse)
        self._tuning_base = {k: dict(v) for k, v in cfg.extra.items() if k in ("t1", "t3") and isinstance(v, dict)}
        self.status_pub = StatusPublisher(self.bus, cfg)
        self.local_pub = LocalStatePublisher(self.bus, cfg)
        self.map_pub = MapDigestPublisher(self.bus, cfg)
        self.map = PerceptionMap(cfg, clock=self.clock)
        self.guard = GuardSupervisor((cfg.extra or {}).get("zed_guard"))     # v0.1.80 (WP36): the ZED guard between the task and the FCU
        from ..companion import tuning as _tuning
        _tuning.seed(cfg)                                                   # v0.1.86.1: limits.max_speed_mps as a knob
        self.cmd.guard = self.guard                                         # v0.1.86.1: guard knobs act at once
        self.points = points_store.Cache(self.cmd.points_file())     # v0.1.29: the saved points into the map digest
        self._obs_q: "queue.Queue[Tuple[str, dict, float]]" = queue.Queue()
        self._tui_seen = -1e9
        self._ui_fcu_link: Optional[str] = None      # v0.1.19: usv_ui's presence beat says fake | real; the sim guard reads it
        self._ui_seen = -1e9
        self._perception_seen = float("-inf")             # v0.1.29.3 (F5): never seen -> age inf -> status null ("never"), not 1e9
        self._cannon_t, self._cannon_on = -1e9, False      # v0.1.22: the task's cannon request (local/<v>/cannon/aim)
        self._stop = asyncio.Event()
        self.ticks = 0
        self._param_asked = float("-inf")
        self.gateway: Optional[OcsGateway] = None
        gw = cfg.extra.get("ocs_gateway", {})
        if gw.get("enabled"):
            self.gateway = OcsGateway(int(gw.get("port", 9102)), lambda: self.vs.status_body(),
                                      lambda env: self.cmd.on_cmd("gateway", env), vehicle_id=cfg.vehicle_id)

    # ---- wiring -----------------------------------------------------------------------
    def _on_transition(self, ev: TaskStateEvent) -> None:
        log.info("%s %s: %s -> %s (%s)", ev.layer, ev.machine, ev.src, ev.dst, ev.reason)
        try:
            self.events.emit(Event(kind="task_state", body=ev.body()))
        except Exception:
            log.exception("task_state event failed")

    def _on_tui(self, topic: str, env: dict) -> None:
        if env.get("body", {}).get("active", True):
            self._tui_seen = self.clock()

    def _on_ui_presence(self, topic: str, payload: bytes) -> None:
        """presence/usv_ui (retained, 2 s, raw JSON): fcu_link fake | real while the console is up."""
        try:
            body = json.loads(payload.decode("utf-8")) if payload else {}
        except Exception:
            return
        if body.get("up"):
            self._ui_fcu_link = body.get("fcu_link")
            self._ui_seen = self.clock()
        else:
            self._ui_fcu_link = None

    def _on_perception(self, topic: str, env: dict) -> None:
        self._perception_seen = self.clock()
        if (env.get("body") or {}).get("ok", True) is not False:
            self.map.last_alive_t = self._perception_seen      # v0.1.27: m0t4's camera preflight (the health beat counts)
        if topic.endswith("/health"):
            return
        if topic.endswith("/guard"):                            # v0.1.80 (WP36): the AGX's ZED guard (cv_publish), at the frame rate
            body = env.get("body") or {}
            self.guard.on_message(body, self.clock(), self.map)
            if body.get("far"):                                 # v0.1.81.1: the 3-8 m clusters -> the map's obstacles (the tick thread)
                self._obs_q.put(("guard/far", body, self.clock()))
            return
        self._obs_q.put((topic, env.get("body") or {}, self.clock()))

    def _on_uav_buoys(self, topic: str, payload: bytes) -> None:
        """v0.1.66 (D20-D21): the UAV's buoys (docs/UAV_BUOY_CONTRACT.md), JSON without an envelope - into the tick thread's queue."""
        try:
            body = json.loads(payload.decode("utf-8")) if payload else {}
        except Exception:
            return
        if isinstance(body, dict):
            self._obs_q.put(("uav/perception/buoys", body, self.clock()))

    def _on_sgit_map(self, topic: str, payload: bytes) -> None:
        """v0.1.78.19: the UAV team's own map (sgit/uav/cv/buoy_map on the boat broker) -> our contract -> the same queue as uav/perception/buoys."""
        msg = sgit_map.decode(payload)
        if msg is None:
            return
        self._sgit_seq = getattr(self, "_sgit_seq", 0) + 1
        if self._sgit_seq == 1:
            log.info("UAV map: first sgit/uav/cv/buoy_map received (%d point(s)) - the UAV team's feed is live", len(msg["points"]))
        self._obs_q.put(("uav/perception/buoys", sgit_map.to_contract(msg, self._sgit_seq), self.clock()))

    def _on_sgit_status(self, topic: str, payload: bytes) -> None:
        st = sgit_map.status_text(payload)
        if st is not None:
            log.info("UAV map: the UAV team's CV publisher is %s", st)
            self._obs_q.put(("sgit/status", {"status": st}, self.clock()))

    def _course_filter(self, run) -> None:
        c = getattr(run, "course", None)
        key = (tuple(getattr(c, "boundary", None) or ()), None if self.origin is None else (self.origin.lat, self.origin.lon))
        if key == getattr(self, "_course_key", None):
            return
        self._course_key = key
        from . import guidance as GD
        self.map.set_course(GD.course_local(run, self.origin), float(GD.params(self.cfg)["course_tol_m"]))

    def _on_ocs_state(self, topic: str, env: dict) -> None:
        """v0.1.66: ocs/state's `uav` block (active, the flight, scan_done) for m1t2's precheck and WAIT_UAV_MAP."""
        u = (env.get("body") or {}).get("uav")
        if isinstance(u, dict):
            self.map.uav_ocs = u
        j = (env.get("body") or {}).get("judge")
        if j is not None:
            self.map.ocs_judge = str(j)                  # v0.1.85 (WP46): protobuf = a real RoboCommand - no RxCourse, no field task

    def _ingest_observations(self, vs: VehicleState, max_n: int = 20) -> None:
        """Map ingest on the tick thread: pose interpolated at the capture time (unix t_cam -> monotonic)."""
        n = 0
        while n < max_n:
            try:
                topic, body, t_recv = self._obs_q.get_nowait()
            except queue.Empty:
                break
            n += 1
            if topic == "sgit/status":                                  # v0.1.78.19: no pose needed for the feed's up / down word
                self.map.uav_feed_status(sgit_map.FEED, body.get("status"), self.clock())
                continue
            if vs.local is None:
                continue
            if topic == "guard/far":                                    # v0.1.81.1: depth obstacles at the pose of the frame
                try:
                    t_cam = float(body.get("t_cam") or 0.0)
                    t_mono = t_recv - max(0.0, time.time() - t_cam) if t_cam else t_recv
                    self.guard.map_far(body, self.poses.at(t_mono) or vs.local, self.map, self.clock())
                except Exception:
                    log.exception("guard far ingest failed")
                continue
            if topic == "uav/perception/buoys":                       # v0.1.66: geo-referenced items -> ingest_geo (the ControlOrigin)
                try:
                    self.map.ingest_geo(body, self.origin, self.clock())
                except Exception:
                    log.exception("UAV buoys ingest failed")
                continue
            try:
                obs = Observation.from_body(body, kind=topic.rsplit("/", 1)[-1])
                t_mono = t_recv - max(0.0, time.time() - obs.t_cam) if obs.t_cam else t_recv
                pose = self.poses.at(t_mono) or vs.local
                self.map.ingest(obs, pose, self.clock(), source=obs.source)
            except Exception:
                log.exception("observation ingest failed")

    def start(self) -> None:
        v = self.cfg.vehicle
        self.bus.subscribe(self.cfg.team("cmd", v, "#"), self.cmd.on_cmd, qos=1)
        self.bus.subscribe(self.cfg.local("tui", "active"), self._on_tui, qos=0)
        self.bus.subscribe(self.cfg.local("perception", "#"), self._on_perception, qos=0)
        self.bus.subscribe(self.cfg.local("uav", "perception", "buoys"), self._on_uav_buoys, qos=0, raw=True)   # v0.1.66 (D21)
        self.bus.subscribe(sgit_map.MAP_TOPIC, self._on_sgit_map, qos=1, raw=True)        # v0.1.78.19: the UAV team's map, as they send it
        self.bus.subscribe(sgit_map.STATUS_TOPIC, self._on_sgit_status, qos=1, raw=True)
        self.bus.subscribe("ocs/state", self._on_ocs_state, qos=0)                                             # v0.1.66: the UAV's state
        self.bus.subscribe("presence/usv_ui", self._on_ui_presence, qos=0, raw=True)
        self.cmd.fcu_link = lambda: self.vs.fcu_link
        self.bus.watch_bridge()
        self.bus.start()
        self.link.start()
        if self.gateway:
            self.gateway.start()
        log.info("mission manager started: %s on %s, broker %s:%s", self.cfg.vehicle_id, self.cfg.mavlink.endpoint,
                 self.bus.host, self.bus.port)

    def stop(self) -> None:
        try:
            if self.runner.active:
                self.runner.abort("shutdown")
            if self.fsm.state in (VState.V3, VState.V4, VState.V5):
                self.adapter.cancel()
        except Exception:
            log.exception("shutdown motion cleanup failed")
        if self.map_pub.recorder is not None:          # v0.1.81.1: the map as it stood (<stamp>_boat_end_*.json) + the timeline closed
            try:
                self.map_pub.recorder.close()
            except Exception:
                log.exception("map recorder close failed")
        if self.gateway:
            self.gateway.stop()
        self.link.stop()
        self.bus.stop()

    # ---- the tick ------------------------------------------------------------------------
    def tick(self) -> None:
        now = self.clock()
        self.ticks += 1
        vs = self.vs
        fcu = self.link.state()
        vs.fcu = fcu
        # v0.1.33 (the sim day): `params_ok` (m0t4's "thrusters" preflight) came only from usv_ui's parameter reads through the share;
        # on the bench nothing asked and m0t4 waited in PREFLIGHT for ever. Read one parameter ourselves (read-only), every 5 s until
        # the FCU answers.
        if fcu.seen_heartbeat and not fcu.params_ok and now - self._param_asked >= 5.0:
            self._param_asked = now
            try:
                self.link.param_request("FRAME_CLASS")
            except Exception as exc:
                log.debug("param_request: %s", exc)
        if self.ghost is not None:                           # v0.1.68 (F2): the ghost's pose; the FCU's lat / lon / hdg never used for it
            self.ghost.step(now)
            x, y = self.origin.to_local(self.ghost.lat, self.ghost.lon)
            pose = LocalPose(x_n=x, y_e=y, hdg_deg=self.ghost.hdg_deg, co_lat=self.origin.lat, co_lon=self.origin.lon, t=now,
                             t_utc=utc_iso(), sog_mps=self.ghost.sog_mps)
            self.poses.push(pose)
            vs.local = pose
            vs.free_run = self._free_run_view(fcu)
        else:
            if self.origin is None and fcu.lat is not None and fcu.gps_fix >= 3:
                self.origin = ControlOrigin(fcu.lat, fcu.lon)
                log.info("control origin set at %.6f %.6f", fcu.lat, fcu.lon)
            if self.origin is not None:
                pose = self.origin.pose_from_fcu(fcu)
                if pose is not None:
                    self.poses.push(pose)
                    vs.local = pose
        tui_active = (now - self._tui_seen) <= self.cfg.timeouts.tui_active_s
        vs.tui_active = tui_active
        vs.fcu_link = self._ui_fcu_link if (now - self._ui_seen) <= 10.0 else None
        self.health.expect_armed = self.fsm.state in (VState.V3, VState.V4, VState.V5)
        slow = self.health.tick_timing(now)
        perception_age = now - self._perception_seen
        bridge_up = self.bus.bridge_state()
        h = self.health.update(fcu, bridge_up, perception_age, now)
        h.loop_slow = slow
        h.tick_ms = self.tick_stats()
        h.guard_ms = self.guard.latency() if hasattr(self.guard, "latency") else None   # v0.1.86.8                         # v0.1.86.4 (WP43 §0): the instrument stays (SYSTEM tab)
        vs.rc6 = {"policy": self.fsm.rc6_policy, "raw": bool(fcu.rc6_high), "holds": bool(self.fsm.inputs.rc6_high)}   # v0.1.50
        vs.health = h
        self._link_loss_policy(bridge_up, now)
        hard = self.health.hard_faults()
        soft = self.health.soft_faults() + (["LOOP_SLOW"] if slow else [])
        before = self.fsm.state
        self.fsm.feed(fcu, h, tui_active, hard, soft)
        after = self.fsm.state
        if before != after:
            self._on_vehicle_change(before, after)
        vs.state = after
        vs.fault = self.fsm.fault_reason if after == VState.V6 else None
        vs.hold_reason = self.fsm.hold_reason if after == VState.V5 else None

        self._free_run_armed(fcu, now)                       # v0.1.68 (F5): before the commands (START_TASK reads the armed state)
        fcu_t = self._ghost_fcu(fcu) if self.ghost is not None else fcu
        self._course_filter(vs.run)                          # v0.1.85 (WP46): the RxCourse boundary -> the map's target filter
        ctx = TaskContext(b=BehaviourContext(cfg=self.cfg, now=now, fcu=fcu_t, pose=vs.local, origin=self.origin,
                                             run=vs.run, map=self.map), emit_event=self.events.emit, log=log.info)
        self.cmd.process(ctx)
        after_cmd = self.fsm.state
        if after_cmd != after:
            self._on_vehicle_change(after, after_cmd)
            vs.state = after_cmd

        # task layer runs only in V4; V3/V5 hold; V2/V1/V6 send nothing
        st = None
        self._trace_obs(fcu, vs)
        if vs.state == VState.V4 and self.guard.busy and self.runner.task is not None:
            cmd, vs.guard = self.guard.filter(self.runner.task, None, ctx)   # v0.1.80: BRAKE / BACKOFF own the boat; the task waits
            self.adapter.apply(cmd, now)
        elif vs.state == VState.V4:
            st = self.runner.step(ctx)
            self.adapter.stage = self.runner.sub_state         # v0.1.68: a free-run refusal is logged once per (stage, mode)
            self._trace_obs(fcu, vs)              # the stage the task is in now, before this tick's command goes out
            if st is not None and (st.done or st.aborted) and not self.runner.active:
                self.fsm.task_finished(aborted=st.aborted, reason=st.reason)
                vs.state = self.fsm.state
                self.adapter.cancel()
            elif st is not None and st.out is not None:
                cmd, vs.guard = self.guard.filter(self.runner.task, st.out.cmd, ctx) if self.runner.task is not None else (st.out.cmd, None)
                self.adapter.apply(cmd, now)
                if self.guard.safe_hold:                        # v0.1.80: the same obstacle again and again -> AUTO_SAFE_HOLD
                    why, self.guard.safe_hold = self.guard.safe_hold, None
                    if self.fsm.request_safe_hold(why):
                        self.runner.pause()
                        self.adapter.hold(now)
                        vs.state = self.fsm.state
                        log.warning("CAUTION %s", why)
            task = self.runner.task
            if task is not None and getattr(task, "safe_hold", None):
                why, task.safe_hold = task.safe_hold, None        # v0.1.21: a task asks for V5 (T1: a scan found nothing)
                if self.fsm.request_safe_hold(why):
                    self.runner.pause()
                    self.adapter.hold(now)
                    vs.state = self.fsm.state
                    log.warning("task asked for a safe hold: %s", why)
        elif vs.state == VState.V3 and station_keep_mode(self.cfg) == "loiter":
            self.adapter.loiter(now)                       # v0.1.81 (WP37): AUTO_HOLD position-holds (handbook 3.4.10); V5 stays HOLD
        elif vs.state in (VState.V3, VState.V5):
            self.adapter.hold(now)
        if vs.state != VState.V4:                          # v0.1.80: a manoeuvre never outlives the task (abort / kill / hold mid-brake)
            self.guard.mode = "CLEAR"
            vs.guard = self.guard.view(now)
        vs.station_keep = station_keep_mode(self.cfg)       # v0.1.81 (WP37): the HOLD / LOITER switch in force (status/usv)
        vs.cmd = self._cmd_view(vs, st, fcu, tui_active)   # v0.1.55 (D1): the commanded side for the boat window
        vs.fence = self.cmd.fence.view(fcu)              # v0.1.53: the FCU geofence - status/usv `fence`
        vs.tuning = dict(self._tuning_view(), speed_k=SK.current(self.cfg.extra))   # v0.1.49: the TUNING card; v0.1.94.3: + the live speed knobs
        if self.ghost is not None:
            vs.free_run = self._free_run_view(fcu)
        vs.current_task = self.runner.current_task
        vs.task_name = self.runner.task_name
        vs.task_state = self.runner.sub_state
        vs.run = self.vs.run
        self.adapter.flush(now)
        self._trace_obs(fcu, vs)
        self.status_pub.maybe(vs, now)
        self.local_pub.maybe(vs, now)
        self._ingest_observations(vs)
        if self.runner.task_name != "T1" and (self.map.colour_source != "camera" or self.map.uav_frozen):
            self.map.colour_source, self.map.uav_frozen = "camera", False      # v0.1.66: m1t2's rules only while T1 runs
        off = self.runner.task_name == "T1" and self.map.colour_source == "camera"   # v0.1.93.2: m1t1 ignores the UAV's map
        if off and not self.map.uav_ingest_off:
            n = self.map.forget_uav()
            log.info("m1t1 (T1, camera): the UAV's map is ignored for this run - %d UAV-only buoy(s) dropped", n)
        self.map.uav_ingest_off = off
        self.map.age_out(now)
        info = (st.out.info if (st is not None and st.out is not None) else {}) if vs.state == VState.V4 else {}
        self._cannon(info.get("cannon"), now)
        path = [[round(a, 2), round(b, 2)] for a, b in (info.get("path") or [])]
        carrot = [round(v, 2) for v in info["carrot"]] if info.get("carrot") else None
        extra = self.map.digest(vs.local, now) if vs.local else None
        if extra is not None and self.origin is not None:
            extra.update(self._points_digest())
        if extra is not None and info.get("scout"):         # v0.1.79 (WP35): the scout's progress - the map ticks the visited buoys
            extra["scout"] = info["scout"]
        if extra is not None and info.get("uav_gate"):      # v0.1.89: the drone's touchdown sequence (the OCS ladder + the run banner)
            extra["uav_gate"] = info["uav_gate"]
        if extra is not None and info.get("micro"):         # v0.1.92: the task's micro-stage {stage, sub, why, next, mods} (the RUN stepper,
            extra["micro"] = info["micro"]                  #   the tk stage line, the run banner / pop-up)
        if extra is not None:                               # v0.1.62: the dock as m3t1 sees it - key points, line, sway path
            try:
                from .dockpath import dock_view
                extra["dock_view"] = dock_view(self.map, vs.local, now, self.cfg.extra.get("t3"), self.cfg.extra.get("t1"))
            except Exception as exc:                        # the map digest must never stop on a drawing aid
                log.debug("dock_view: %s", exc)
        self.map_pub.maybe(vs, now, path=path, carrot=carrot, extra=extra)

    # ---- v0.1.68 (WP17): free run -------------------------------------------------------------------------------------------
    def _ghost_fcu(self, fcu):
        """The tasks' FCU snapshot in free run: the real one with the ghost's position, heading and velocity (T4's position report,
        a saved point's own fix); status/usv keeps the real FCU block."""
        vn, ve = self.ghost.vn_ve
        return dataclasses.replace(fcu, lat=self.ghost.lat, lon=self.ghost.lon, hdg_deg=self.ghost.hdg_deg, sog_mps=self.ghost.sog_mps,
                                   vx_n=vn, vy_e=ve)

    def _free_run_view(self, fcu) -> dict:
        task = self.runner.task
        can = task.skip_check() if (task is not None and hasattr(task, "skip_check")) else "no task"
        if not can and self.fsm.state != VState.V4:
            can = f"{self.fsm.state.value}: skip needs AUTO_TASK"
        d = {"ghost": self.ghost.view(), "armed_policy": self.free_run_armed, "armed": bool(fcu.armed),
             "sent": {k: self.adapter.sent[k] for k in ("frames", "targets", "velocity", "mode")},
             "skip": {"ok": not can, "why": can}}
        if fcu.gps_fix is not None and fcu.gps_fix >= 3:
            d["gps_note"] = f"gps fix {fcu.gps_fix} (ignored: free run)"
        if fcu.armed and self.free_run_armed == "warn":
            d["banner"] = "FREE RUN · FCU ARMED"
        return d

    def _free_run_armed(self, fcu, now: float) -> None:
        """F5: refuse - the FCU arming while a task runs aborts it to AUTO_SAFE_HOLD (FREE RUN: FCU armed); warn - one event, the
        banner line, the task goes on. START_TASK's refusal is the command handler's (cmd.free_run)."""
        if self.ghost is None:
            return
        armed = bool(fcu.armed)
        rising = armed and not self._fr_armed_prev
        self._fr_armed_prev = armed
        if not armed:
            return
        if self.free_run_armed == "warn":
            if rising:
                log.warning("FREE RUN: the FCU is ARMED (free_run.armed warn) - the task goes on; ESC power off!")
                self._event_alert("FREE RUN · FCU ARMED", "warn")
            return
        if self.runner.active and self.fsm.state in (VState.V4, VState.V5):
            name = self.runner.task_name
            self.runner.abort(FREE_RUN_ARMED)
            self.adapter.cancel()
            if self.fsm.state == VState.V4:
                self.fsm.request_safe_hold(FREE_RUN_ARMED)
            else:
                self.fsm.hold_reason = FREE_RUN_ARMED
            self.fsm.task_paused = self.fsm.task_running = False
            self.vs.state = self.fsm.state
            log.warning("FREE RUN: the FCU armed during %s - task aborted, AUTO_SAFE_HOLD (free_run.armed refuse)", name)
            self._event_alert(f"{FREE_RUN_ARMED}: {name} aborted", "refuse")

    def _event_alert(self, text: str, policy: str) -> None:
        try:
            self.events.emit(Event(kind="alert", body={"event": "alert", "alert": "free_run_armed", "text": text, "policy": policy,
                                                      "t": utc_iso()}))
        except Exception:
            log.exception("free-run alert event failed")

    def _cmd_view(self, vs, st, fcu, tui_active) -> dict:
        """v0.1.55 (WP10f-rest, D1): what the mission manager commands now, for local/<v>/mm/state `cmd` (5 Hz; the 10 Hz detail stays
        in mm_trace.jsonl). frame = the last mode-20 τ {surge, sway, yaw} (normalized ±1), target = the last GUIDED target / carrot,
        hdg_goal_deg = the behaviour's heading goal (sway_transit's bow, dock_los's bay normal, scan_yaw's face); all null when
        nothing commands (not AUTO_TASK, HOLD, MANUAL, usv_ui holding the FCU). Never invented: a value the behaviour did not give is null."""
        task = self.runner.task
        beh = getattr(task, "_beh", None) if task is not None else None
        d = {"t": utc_iso(), "mode": self.adapter.mode_wanted or fcu.mode_name, "behaviour": getattr(beh, "name", None),
             "stage": vs.task_state, "frame": None, "target": None, "hdg_goal_deg": None,
             "max_speed_mps": self.cfg.limits.max_speed_mps}         # the speed dial's τ-surge scale
        prev = getattr(self, "_cmd_prev", None)
        if vs.state != VState.V4 or tui_active or st is None:
            self._cmd_prev = None
            return d
        out = st.out
        if out is None or out.cmd is None:                     # nothing this tick: the last command still stands
            return dict(prev, t=d["t"], stage=d["stage"]) if prev else d
        info = out.info or {}
        c = out.cmd
        if isinstance(c, ThrusterFrame):
            tau = info.get("tau")
            d["frame"] = dict(tau) if isinstance(tau, dict) else {"surge": 0.0, "sway": 0.0, "yaw": 0.0} if not any(c.as_list()) else None
        elif isinstance(c, GuidedTarget):
            hdg = None
            if vs.local is not None and self.origin is not None:
                try:
                    tn, te = self.origin.to_local(c.lat, c.lon)
                    hdg = round(math.degrees(math.atan2(te - vs.local.y_e, tn - vs.local.x_n)) % 360.0, 1)
                except Exception:
                    hdg = None
            car = info.get("carrot")
            d["target"] = {"lat": round(c.lat, 7), "lon": round(c.lon, 7), "hdg_deg": hdg, "speed_mps": c.speed_mps,
                           "carrot": {"x_n": round(car[0], 2), "y_e": round(car[1], 2)} if car else None}
        hg = info.get("hdg_goal_deg", info.get("hold_hdg"))
        d["hdg_goal_deg"] = round(float(hg) % 360.0, 1) if hg is not None else None
        if isinstance(c, Hold):
            d["frame"] = d["target"] = d["hdg_goal_deg"] = None
        self._cmd_prev = d
        return d

    def _tuning_view(self) -> dict:
        key = tuple(tuning.current(self.cfg.extra, k) for k in tuning.ORDER)
        if getattr(self, "_tv_key", None) != key:
            self._tv_key = key
            self._tv = tuning.view(self.cfg.extra, self.cfg.gains, getattr(self, "_tuning_base", None))
        return self._tv

    def _on_tuned(self, key: str, old: float, new: float) -> None:
        if self.adapter.trace:
            self.adapter.trace.ev("tune", key=key, old=old, new=new)

    def _points_digest(self) -> dict:
        """usv_ui's map shape (points: name -> [x_n, y_e]) + points_ll (name -> lat, lon, hdg, utc, from) for the OCS page."""
        pts, ll = {}, {}
        for name, p in self.points.get().items():
            try:
                x, y = self.origin.to_local(float(p["lat"]), float(p["lon"]))
            except (KeyError, TypeError, ValueError):
                continue
            pts[name] = [round(x, 2), round(y, 2)]
            ll[name] = {k: p.get(k) for k in ("lat", "lon", "hdg", "utc", "from")}
        return {"points": pts, "points_ll": ll}

    def _cannon(self, aim: Optional[dict], now: float) -> None:
        """v0.1.22: a task asks for the water cannon through its output (info.cannon {on, bay}); local/<v>/cannon/aim at 5 Hz
        while on, one `on: false` when it stops. cv/cannon (AGX) or fake_dock (sim) listen."""
        on = bool(aim and aim.get("on"))
        if on and now - self._cannon_t >= 0.2:
            self._cannon_t = now
            self.bus.publish(self.cfg.local("cannon", "aim"), "local", dict(aim, t=utc_iso()), qos=0, timeout_s=0.0)
        elif not on and self._cannon_on:
            self.bus.publish(self.cfg.local("cannon", "aim"), "local", {"on": False, "t": utc_iso()}, qos=0, timeout_s=0.0)
        self._cannon_on = on

    def _link_loss_policy(self, bridge_up: bool, now: float) -> None:
        """guards.link_loss_action (MODULES S6): continue = nothing; safe_hold = a running task goes to V5 when the hub bridge
        has been down for link_loss_s, with a sticky hold that clears when the link is back (the operator then resumes)."""
        if bridge_up:
            if self._link_down_since is not None:
                self._link_down_since = None
                if "OCS_LINK" in self.fsm.sticky_holds:
                    self.fsm.sticky_holds.discard("OCS_LINK")
                    log.info("OCS link back: the link-loss hold is cleared (RESUME to continue the task)")
            return
        if self._link_down_since is None:
            self._link_down_since = now
        if self.cfg.guards.link_loss_action != "safe_hold":
            return
        if now - self._link_down_since >= self.cfg.guards.link_loss_s and self.fsm.state == VState.V4 and "OCS_LINK" not in self.fsm.sticky_holds:
            self.fsm.sticky_holds.add("OCS_LINK")
            if self.fsm.request_safe_hold("OCS link lost (link_loss_action safe_hold)"):
                log.warning("OCS link down %.0f s: task paused (safe_hold)", now - self._link_down_since)

    def _on_vehicle_change(self, before: VState, after: VState) -> None:
        # entering a non-auto state: no motion from us; the task runner is paused/aborted accordingly
        if after in (VState.V1, VState.V2, VState.V6):
            if self.runner.active:
                self.runner.abort(f"vehicle -> {after.value}")
            if before in (VState.V3, VState.V4, VState.V5):
                try:
                    if self.adapter.direct_active:
                        self.adapter.neutral()
                        self.adapter.direct_active = False
                except Exception:
                    log.exception("neutral on leaving AUTO failed")
        elif after == VState.V5 and self.runner.active:
            self.runner.pause()
        elif after == VState.V3 and before == VState.V5 and self.runner.active:
            # RESUME to AUTO_HOLD with a task still registered means the task was aborted elsewhere
            pass

    TICK_SLOW_MS = 150.0

    def tick_stats(self) -> Optional[dict]:
        """v0.1.86.4 (WP43 §0): p50 / p95 / max of the last 60 s of tick durations (ms) and the ticks > 150 ms in the last minute."""
        d = list(getattr(self, "_tick_ms", []) or [])
        if not d:
            return None
        s = sorted(x for _, x in d)
        q = lambda f: round(s[min(len(s) - 1, int(f * (len(s) - 1) + 0.5))], 1)
        now = d[-1][0]
        return {"p50": q(0.5), "p95": q(0.95), "max": round(s[-1], 1), "n": len(s),
                "slow_per_min": sum(1 for t, x in d if x > self.TICK_SLOW_MS and now - t <= 60.0)}

    def timed_tick(self) -> float:
        """One tick, timed into the 60 s ring; returns its ms."""
        from collections import deque
        if not hasattr(self, "_tick_ms"):
            self._tick_ms = deque(maxlen=max(10, int(60 * self.cfg.rates.tick_hz)))
        t0 = time.perf_counter()
        try:
            self.tick()
        finally:
            ms = (time.perf_counter() - t0) * 1000.0
            self._tick_ms.append((time.monotonic(), ms))
        return ms

    async def run(self) -> None:
        period = 1.0 / self.cfg.rates.tick_hz
        next_t = self.clock()
        while not self._stop.is_set():
            try:
                self.timed_tick()
            except Exception:
                log.exception("tick failed")
            next_t += period
            delay = next_t - self.clock()
            if delay < 0:
                next_t = self.clock()
                delay = 0
            await asyncio.sleep(delay)

    def _trace_obs(self, fcu, vs) -> None:
        """v0.1.27.1: the command trace's 'obs' record (written only on change; tools/mode_log.py)."""
        if self.adapter.trace:
            self.adapter.trace.observe(fcu.mode_name, fcu.armed, vs.state.value, self.runner.task_name, self.runner.sub_state)

    def request_stop(self) -> None:
        self._stop.set()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="optiprimex USV mission manager")
    ap.add_argument("--config", required=True)
    ap.add_argument("--log-level", default="INFO")
    ap.add_argument("--trace", metavar="PATH", help="write the command trace (mode requests, frames, observed modes / stages) as "
                                                    "JSON lines - read it with `mode-log PATH` (tools/mode_log.py)")
    ap.add_argument("--map-rec", metavar="DIR", help="v0.1.81.1: record the USV map here - the timeline (1 Hz jsonl), latest.json, a "
                                                     "snapshot per task end (usv/map_rec.py); the launchers pass $RUN_DIR/maps")
    ap.add_argument("--free-run", action="store_true", help="v0.1.68 (WP17): the stages against an in-process ghost plant, mode requests "
                                                            "only to the FCU - a profile with free_run.allowed (the lab profile) only")
    ap.add_argument("--free-start", metavar="LAT,LON,HDG", help="free run: the ghost's start (else --world's start, else free_run.start)")
    ap.add_argument("--world", metavar="FILE", help="free run: a sim world file - its start is the ghost's start")
    ap.add_argument("--guidance", choices=("fcu", "underway"), help="v0.1.85 (WP41): override the profile's guidance.mode (run_sim.sh: underway "
                                                                     "on the omni_usv plant)")
    a = ap.parse_args(argv)
    logging.basicConfig(level=getattr(logging, a.log_level.upper()), format="%(asctime)s %(name)s %(levelname)s %(message)s")
    cfg = cfgmod.load(a.config)
    if a.guidance:
        cfg.extra.setdefault("guidance", {})
        cfg.extra["guidance"] = dict(cfg.extra.get("guidance") or {}, mode=a.guidance)
    ghost = None
    if a.free_run or a.free_start or a.world:
        fr = cfg.extra.get("free_run") or {}
        if not a.free_run:
            print("--free-start / --world belong to --free-run", file=sys.stderr)
            return 2
        if not (isinstance(fr, dict) and fr.get("allowed") is True):
            print(f"{FREE_RUN_REFUSED} ({cfg.profile or a.config} has no free_run.allowed)", file=sys.stderr)
            return 2
        from .ghost import GhostNav, resolve_start
        try:
            start, where = resolve_start(a.free_start, a.world, fr)
        except (ValueError, OSError) as exc:
            print(f"--free-run: {exc}", file=sys.stderr)
            return 2
        pl = cfg.extra.get("plant") or {}
        ghost = GhostNav(start, hull=str(pl.get("hull") or "ideal"), str_ramp_s=float(pl.get("str_ramp_s") or 0.0))
        log.warning("*** FREE RUN: ghost position, no motion to the FCU (mode requests only) - bench only, props off, ESC power off, "
                    "NEVER at the pool *** (start from %s)", where)
    mm = MissionManager(cfg, ghost=ghost)
    mm.cmd.profile_path = a.config                  # v0.1.49: `tune save` copies this file (never writes it)
    if a.map_rec:
        from .map_rec import MapRecorder
        from .. import __version__
        mm.map_pub.recorder = MapRecorder(a.map_rec, "boat", cfg.vehicle, version=__version__)
        log.info("map recording -> %s", a.map_rec)
    if a.trace:
        from .cmd_trace import CmdTrace
        mm.adapter.trace = CmdTrace(a.trace, profile=cfg.profile, fcu=cfg.mavlink.endpoint)
        log.info("command trace -> %s", a.trace)
    mm.start()
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, mm.request_stop)
        except NotImplementedError:  # pragma: no cover (Windows)
            pass
    try:
        loop.run_until_complete(mm.run())
    finally:
        mm.stop()
        loop.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
