"""command_handler: cmd/<v>/* -> dedup -> run context -> vehicle fsm / tasks -> ack (ARCHITECTURE 5.1).

Every command gets exactly one ACCEPTED/REJECTED within cmd_ack_s and, if accepted, one DONE/FAILED
when the resulting transition lands. A duplicate msg_id is acked again with the stored result and
never re-executed.
"""
from __future__ import annotations

import logging
import os
import queue
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from typing import Callable, Dict, List, Optional, Tuple

from ..companion import fence as F
from ..companion import tuning
from ..companion import speed_k as SK
from ..companion.config import Config
from ..companion.mqtt import Bus, Dedup
from ..companion.state import Command, RunContext, VState, utc_iso
from . import points_store
from . import run_context as rc
from .task_runner import TaskRunner
from .tasks.base import Task, TaskContext
from .vehicle_fsm import VehicleFsm

log = logging.getLogger("optiprimex.cmd")


def sent_at_unix(raw) -> Optional[float]:
    """The envelope's sent_at (UTC ISO, '...Z'; a number = unix seconds) as unix seconds, or None when absent / unreadable."""
    if raw is None or raw == "":
        return None
    if isinstance(raw, (int, float)) and not isinstance(raw, bool):
        return float(raw)
    try:
        s = str(raw).strip()
        if s.endswith("Z"):
            s = s[:-1] + "+00:00"
        dt = datetime.fromisoformat(s)
        if dt.tzinfo is None:
            dt = dt.replace(tzinfo=timezone.utc)
        return dt.timestamp()
    except (TypeError, ValueError):
        return None


@dataclass
class Pending:
    cmd: Command
    done_when: Callable[[], bool]
    fail_when: Callable[[], Optional[str]]
    deadline: float
    done_reason: Optional[Callable[[], str]] = None     # v0.1.53: the DONE ack's text (the fence: what the FCU answered)


class CommandHandler:
    def __init__(self, cfg: Config, bus: Bus, fsm: VehicleFsm, run: RunContext, runner: TaskRunner, clock):
        self.cfg, self.bus, self.fsm, self.run, self.runner, self.clock = cfg, bus, fsm, run, runner, clock
        self.q: "queue.Queue[Tuple[str, dict]]" = queue.Queue()
        self.dedup = Dedup(ttl_s=900)
        self.free_run: Optional[dict] = None          # v0.1.68 (WP17): {"armed": refuse | warn} in free run - SKIP allowed
        self.pending: List[Pending] = []
        self.handled = 0
        self.rejected = 0
        self.fcu_link: Optional[Callable[[], Optional[str]]] = None   # v0.1.19: set by the mission manager - usv_ui's fcu_link (fake | real)
        self.wall: Callable[[], float] = time.time    # v0.1.43: the wall clock the cmd age is measured with (sent_at is UTC wall time)
        self.profile_path: Optional[str] = None       # v0.1.49: the loaded profile (mission_manager main), the base of `tune save`
        self.link = None                    # v0.1.50: the mission manager's MAVLink (FCU_MODE / FCU_ARM), set by the mission manager
        self.on_tuned: Optional[Callable[[str, float, float], None]] = None    # v0.1.49: (key, old, new) -> the trace
        self.fence = F.FenceBook()          # v0.1.53: the FCU geofence (FENCE_APPLY / FENCE_ENABLE) - status/usv `fence`

    # ---- MQTT side (network thread) ----------------------------------------------------
    def on_cmd(self, topic: str, env: dict) -> None:
        if env.get("kind") != "cmd":
            return
        self.q.put((topic, env))

    # ---- tick side -------------------------------------------------------------------
    def process(self, ctx: TaskContext, max_n: int = 10) -> None:
        n = 0
        while n < max_n:
            try:
                topic, env = self.q.get_nowait()
            except queue.Empty:
                break
            n += 1
            self._handle(Command.from_env(env), ctx)
        self._complete(ctx)

    def _handle(self, cmd: Command, ctx: TaskContext) -> None:
        seen, prev = self.dedup.seen(cmd.msg_id)
        if seen:
            result, reason = prev or ("REJECTED", "duplicate")
            self.bus.ack(cmd, result, f"duplicate: {reason}".strip(), self.fsm.state.value)
            return
        why = self.stale(cmd)                         # v0.1.43 (OCS_V2_SPEC §6): before anything else looks at it
        if why:
            self._finish(cmd, "REJECTED", why)
            return
        ok, why = rc.accepts(self.run, cmd)
        if not ok:
            self._finish(cmd, "REJECTED", why)
            return
        try:
            result, reason, done_now = self._dispatch(cmd, ctx)
        except Exception as exc:                      # a command must never take the tick down: FAILED, logged, next
            log.exception("command %s failed", cmd.action)
            result, reason, done_now = "FAILED", f"{type(exc).__name__}: {exc}", False
        self._finish(cmd, result, reason)
        if result == "ACCEPTED":
            if done_now:
                self.bus.ack(cmd, "DONE", "", self.fsm.state.value)
            else:
                self.pending.append(self._pending_for(cmd))

    def stale(self, cmd: Command) -> Optional[str]:
        """v0.1.43 (OCS_V2_SPEC §6): 'stale (sent N s ago, limit L s)' when the envelope's sent_at is older than guards.cmd_max_age_s
        (the bridge delivers queued QoS-1 commands after a link loss; clocks are within 1 s), else None. No / unreadable sent_at:
        accepted with a log line (nothing older breaks)."""
        limit = float(getattr(self.cfg.guards, "cmd_max_age_s", 0.0) or 0.0)
        if limit <= 0:
            return None
        raw = cmd.env.get("sent_at")
        t = sent_at_unix(raw)
        if t is None:
            log.info("cmd %s %s: no readable sent_at (%r) - age not checked", cmd.action, cmd.env.get("msg_id"), raw)
            return None
        age = self.wall() - t
        if age > limit:
            return f"stale (sent {age:.1f} s ago, limit {limit:g} s)"
        return None

    def home_target(self, ctx: TaskContext) -> Tuple[Optional[Tuple[float, float]], str]:
        """((lat, lon), 'HOME point' | 'control origin') or (None, why): the saved point HOME first (decided 2026-11-04)."""
        p = points_store.load(self.points_file()).get("HOME")
        if p is not None:
            try:
                return (float(p["lat"]), float(p["lon"])), "the saved point HOME"
            except (KeyError, TypeError, ValueError):
                pass
        o = ctx.b.origin
        if o is not None:
            return (o.lat, o.lon), "the control origin (no saved point HOME)"
        return None, "no saved point HOME and no control origin (no GPS fix yet)"

    def _home(self, cmd: Command, ctx: TaskContext) -> Tuple[str, str, bool]:
        """v0.1.30 (WP8c) the go-home override: from any AUTO state, with or without a run - abort what runs, RETURN_HOME slowly
        (home.speed_mps, 0.3), stop and hold. MANUAL / KILLED / FAULT refuse: the boat is the operator's (decided 2026-11-04)."""
        s = self.fsm.state
        if s not in (VState.V3, VState.V4, VState.V5):
            return "REJECTED", f"HOME acts in AUTO only (vehicle {s.value}): in MANUAL the boat is yours - drive it home with the transmitter", False
        tgt, src = self.home_target(ctx)
        if tgt is None:
            return "REJECTED", f"HOME: {src}", False
        notes = []
        if self.runner.active:
            notes.append(f"{self.runner.task_name} aborted")
            self.runner.abort("HOME override")
        if self.fsm.state in (VState.V4, VState.V5):
            self.fsm.handle_command(Command(env=cmd.env, action="ABORT_TASK"))
        args = {"lat": tgt[0], "lon": tgt[1]}
        if cmd.args.get("speed_mps") is not None:
            args["speed_mps"] = float(cmd.args["speed_mps"])
        res, why = self.fsm.handle_command(Command(env=cmd.env, action="START_TASK", task="RETURN_HOME", args=args))
        if res != "ACCEPTED":
            return "REJECTED", f"HOME: {why}" + (f" ({', '.join(notes)})" if notes else ""), False
        t = self.runner.start("RETURN_HOME", args, ctx)
        speed = getattr(t, "p", {}).get("speed_mps", 0.3)
        notes.insert(0, f"going home to {src} at {speed} m/s")
        return "ACCEPTED", "; ".join(notes), True

    def points_file(self) -> str:
        return str((self.cfg.extra.get("test_path", {}) or {}).get("points_file", "usv_ui/points.json"))

    def _points(self, a: str, args: dict, ctx: TaskContext) -> Tuple[str, str, bool]:
        """v0.1.29 (WP8b): the OCS's save_point / del_point -> usv_ui's points.json (m0t2 / m0t3 read it by name). Any vehicle state:
        it is data, nothing moves. SAVE_POINT without lat/lon = the boat's own fix (3D fix needed, as usv_ui's `save`)."""
        name = str(args.get("name") or "")
        why = points_store.check_name(name)
        if why:
            return "REJECTED", why, False
        path = self.points_file()
        if a == "DELETE_POINT":
            return ("ACCEPTED", f"deleted {name} from {path}", True) if points_store.delete(path, name) else \
                ("REJECTED", f"no point {name} in {path}", False)
        if args.get("lat") is not None and args.get("lon") is not None:
            lat, lon, hdg, src = float(args["lat"]), float(args["lon"]), None, "ocs-map"
            if not (-90 <= lat <= 90 and -180 <= lon <= 180):
                return "REJECTED", f"lat/lon out of range: {lat} {lon}", False
            extra = {}
        else:
            f = ctx.b.fcu
            if f is None or f.lat is None or f.gps_fix < 3:
                return "REJECTED", f"no 3D fix (gps_fix {getattr(f, 'gps_fix', 0)}): drive where the point is, or drop it on the map", False
            lat, lon, hdg, src = f.lat, f.lon, f.hdg_deg, "ocs-boat"
            extra = {"sats": f.sats, "hdop": f.hdop}
        rec, replaced = points_store.put(path, name, lat, lon, hdg, src=src, **extra)
        return "ACCEPTED", f"{'replaced' if replaced else 'saved'} {name} = {rec['lat']:.7f} {rec['lon']:.7f} ({src})", True

    def _tune(self, args: dict) -> Tuple[str, str, bool]:
        """v0.1.49 (WP10k-b item 22): a runtime knob into the live config (cfg.extra[t1|t3]) - the next task start reads it; a task that
        runs keeps its values, so MANUAL / AUTO_HOLD only (the OCS checks the same). `save`: a copy of the profile with the live values
        under tuning.save_dir (logs/launch)/<stamp>/usv_tuned.yaml, never the profile itself. Never an FCU parameter (ADR-9)."""
        if args.get("save"):
            if not self.profile_path:
                return "REJECTED", "tune save: this mission manager does not know its profile file (started without --config?)", False
            root = str((self.cfg.extra.get("tuning", {}) or {}).get("save_dir", "logs/launch"))
            try:
                path, lines = tuning.save(self.profile_path, self.cfg.extra, root)
            except (OSError, ValueError) as exc:
                return "REJECTED", f"tune save: {exc}", False
            return "ACCEPTED", f"saved {path} ({'; '.join(lines) if lines else 'no change from ' + os.path.basename(self.profile_path)}) - " \
                               f"load it with MM_CONFIG_OVERRIDE={path}", True
        s = self.fsm.state.value
        if s not in tuning.TUNE_STATES:
            return "REJECTED", f"SET_TUNING refused in {s}: MANUAL / AUTO_HOLD only (a running task keeps the values it started with)", False
        why, value = tuning.check(args.get("key"), args.get("value"))
        if why:
            return "REJECTED", why, False
        key = str(args["key"])
        why = tuning.cross_check(self.cfg.extra, key, value)          # v0.1.86.1: guard slow > guard stop
        if why:
            return "REJECTED", why, False
        old, new = tuning.apply(self.cfg.extra, key, value)
        tuning.sync(self.cfg, getattr(self, "guard", None))          # v0.1.86.1: the speed cap and the guard at once
        if self.on_tuned:
            self.on_tuned(key, old, new)
        log.info("tuning %s %g -> %g", key, old, new)
        return "ACCEPTED", f"{key} {old:g} -> {new:g} ({'/'.join(tuning.KNOBS[key][0])}.{tuning.field(key)}; the next task start reads it)", True

    def _speed(self, args: dict) -> Tuple[str, str, bool]:
        """v0.1.94.3: SET_SPEED_K - cfg.extra["speed_k"], read by the task base every tick (tasks/base.speed_class); any state."""
        why, ks = SK.check(args)
        if why:
            return "REJECTED", why, False
        old_k = SK.current(self.cfg.extra)
        new_k = SK.apply(self.cfg.extra, ks)
        if getattr(self, "on_tuned", None):
            for c in SK.CLASSES:
                if old_k[c] != new_k[c]:
                    self.on_tuned("speed_k." + c, old_k[c], new_k[c])
        log.info("speed knobs %s", SK.line(new_k))
        return "ACCEPTED", f"speed {SK.line(new_k)} (at once; cap {float(self.cfg.limits.max_speed_mps):g} m/s x k, max {SK.V_CEIL:g})", True

    def _fcu(self, a: str, args: dict) -> Tuple[str, str, bool]:
        """v0.1.50 (Dmitrii 2026-11-06, lifting ADR-9 for these): the page's FCU mode and ARM / DISARM, sent as Mission Planner would -
        the FCU's own checks decide; SET_RC6 = the RC6 policy (latched | hold). A mode / arm command clears a latched RC6 (last wins)."""
        from ..companion.link import MODE_BY_NAME
        if a == "SET_STATION_KEEP":                     # v0.1.81 (WP37): HOLD / LOITER as the autonomous station keep, kept across runs
            m = str(args.get("mode", "")).lower()
            if m not in ("hold", "loiter"):
                return "REJECTED", "station_keep: hold | loiter", False
            self.cfg.extra.setdefault("station_keep", {})["mode"] = m
            return "ACCEPTED", f"station keep {m.upper()}" + (" (a real position hold - AUTO_HOLD and every stage that holds; the safe hold stays "
                                                              "HOLD)" if m == "loiter" else " (motors neutral, drifting)"), True
        if a == "SET_RC6":
            p = str(args.get("policy", "")).lower()
            if p not in ("latched", "hold"):
                return "REJECTED", "rc6: latched | hold", False
            self.fsm.set_rc6_policy(p)
            return "ACCEPTED", f"RC6 policy {p}" + (" (a later page command wins over a high RC6)" if p == "latched" else
                                                     " (while RC6 is high: no autonomy)"), True
        if self.link is None:
            return "REJECTED", "no FCU link in this mission manager", False
        cleared = self.fsm.clear_rc6_latch()
        tail = " · RC6 latch cleared (last wins)" if cleared else ""
        if a == "FCU_MODE":
            name = str(args.get("mode", "")).upper()
            if name not in MODE_BY_NAME:
                return "REJECTED", f"unknown mode {name or '?'}: one of {', '.join(sorted(MODE_BY_NAME))}", False
            ok = self.link.set_mode(name, timeout_s=2.0)
            return ("ACCEPTED", f"FCU mode {name}{tail}", True) if ok else ("REJECTED", f"the FCU refused mode {name}{tail}", False)
        on = bool(args.get("on"))
        res = self.link.arm(on)
        word = "ARM" if on else "DISARM"
        if res == 0:
            return "ACCEPTED", f"FCU {word}ED{tail}" if on else f"FCU DISARMED{tail}", True
        return "REJECTED", (f"the FCU refused {word} (result {res}: its pre-arm checks)" if res is not None else f"no answer from the FCU to {word}") + tail, False

    MAV_RESULT = {0: "MAV_RESULT_ACCEPTED", 1: "MAV_RESULT_TEMPORARILY_REJECTED", 2: "MAV_RESULT_DENIED", 3: "MAV_RESULT_UNSUPPORTED",
                  4: "MAV_RESULT_FAILED", 5: "MAV_RESULT_IN_PROGRESS"}

    def _fence(self, a: str, args: dict) -> Tuple[str, str, bool]:
        """v0.1.53 (Dmitrii 2026-11-06: "a button to apply to FCU, with acknowledgment from Mavlink ... toggle off, which is by
        default"): FENCE_APPLY {points, source} uploads one inclusion polygon and reads it back (1e-6 deg); FENCE_ENABLE {on} sends
        MAV_CMD_DO_FENCE_ENABLE. Checked here, then a worker talks to the FCU (never the tick): ACCEPTED now, DONE / FAILED with the
        FCU's answer. Refused in AUTO_TASK (a new fence / a fence switched on mid-task could stop the running task); off always passes."""
        bk = self.fence
        if self.link is None:
            return "REJECTED", "no FCU link in this mission manager", False
        if bk.busy:
            return "REJECTED", f"a fence operation is running ({bk.busy}): wait for its answer", False
        task_running = self.fsm.state == VState.V4
        if a == "FENCE_APPLY":
            if task_running:
                return "REJECTED", f"FENCE_APPLY refused in AUTO_TASK ({self.runner.task_name or 'a task'} runs): hold / abort first, or apply before the start", False
            why = F.problem(args.get("points"))
            if why:
                return "REJECTED", f"FENCE_APPLY: {why}", False
            pts = F.ring(args.get("points"))
            source = str(args.get("source") or "") or None
            job, label = (lambda: self._fence_apply(pts, source)), f"apply {len(pts)} points"
        else:
            on = args.get("on")
            if not isinstance(on, bool):
                return "REJECTED", "FENCE_ENABLE: on true | false", False
            if on and task_running:
                return "REJECTED", f"fence on refused in AUTO_TASK ({self.runner.task_name or 'a task'} runs): a breach would stop the task - switch it on before the start", False
            job, label = (lambda: self._fence_enable(on)), f"fence {'on' if on else 'off'}"
        bk.busy, bk.result = label, None

        def run():
            try:
                res = job()
            except Exception as exc:                  # never leave the book busy
                log.exception("fence %s", label)
                res = (False, f"{type(exc).__name__}: {exc}")
            bk.last, bk.result, bk.busy = res[1], res, None
            log.info("fence %s: %s", label, res[1])
        threading.Thread(target=run, name="fence", daemon=True).start()
        return "ACCEPTED", f"{label}: sent to the FCU, waiting for its answer", False

    def _fence_apply(self, pts, source) -> Tuple[bool, str]:
        bk, n = self.fence, len(pts)
        ok, txt = self.link.fence_upload(pts)
        if not ok:
            return False, f"fence {n} points: {txt}"
        bk.applied_t, bk.source, bk.sent, bk.matches, bk.points_n = utc_iso(), source, list(pts), None, n
        items, why = self.link.fence_download()
        if items is None:
            return False, f"fence {n} points uploaded ({txt}), but the read-back failed: {why}"
        return self._compare(items, n)

    def _compare(self, items, n_sent: Optional[int] = None) -> Tuple[bool, str]:
        from ..companion.link import MAV
        bk = self.fence
        bk.points_n = len(items)
        poly = [(lat, lon) for c, _p1, lat, lon in items if c == MAV.MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION]
        sent = getattr(bk, "sent", None)
        if not sent:
            return True, ""
        same, why = F.same(sent, poly)
        if same and len(poly) != len(items):
            same, why = False, f"the FCU holds {len(items) - len(poly)} other fence item(s) too"
        bk.matches = same
        if n_sent is None:
            return True, "" if same else f"the FCU's fence differs from the one applied ({why})"
        if same:
            return True, f"fence {n_sent} points uploaded, read back identical"
        return False, f"fence {n_sent} points uploaded, read back DIFFERENT: {why}"

    def _fence_enable(self, on: bool) -> Tuple[bool, str]:
        bk = self.fence
        note = ""
        if on:
            items, why = self.link.fence_download()
            if items is None:
                return False, f"fence on: {why}"
            if not items:
                bk.points_n = 0
                return False, "fence on refused: the FCU holds no fence - fence apply course|geofence first"
            note = self._compare(items)[1]
        res = self.link.fence_enable(on)
        word = "ON" if on else "OFF"
        if res == 0:
            bk.enable_ack = on
            held = f" (the FCU holds {bk.points_n} points)" if (on and bk.points_n) else ""
            return True, f"fence {word}{held}: MAV_RESULT_ACCEPTED" + (f" · {note}" if note else "")
        if res is None:
            return False, f"fence {word}: no COMMAND_ACK from the FCU to DO_FENCE_ENABLE"
        return False, f"the FCU refused fence {word}: {self.MAV_RESULT.get(res, f'MAV_RESULT {res}')}"

    def _finish(self, cmd: Command, result: str, reason: str) -> None:
        self.dedup.remember(cmd.msg_id, (result, reason))
        self.handled += 1
        if result == "REJECTED":
            self.rejected += 1
            log.info("REJECTED %s: %s", cmd.action, reason)
        self.bus.ack(cmd, result, reason, self.fsm.state.value)

    def _dispatch(self, cmd: Command, ctx: TaskContext) -> Tuple[str, str, bool]:
        a = cmd.action
        if a in ("PREPARE", "START_RUN", "STOP_RUN", "COURSE"):
            changed, why = rc.apply(self.run, cmd)
            if not changed:
                return "REJECTED", why, False
            self.fsm.declaration_seq = self.run.declaration_seq
            self.fsm.run_id = self.run.run_id
            self.bus.run_id, self.bus.declaration_seq = self.run.run_id, self.run.declaration_seq
            if a == "STOP_RUN" and self.runner.active:
                self.runner.abort("STOP_RUN")
                if self.fsm.state in (VState.V4, VState.V5):
                    self.fsm.handle_command(Command(env=cmd.env, action="ABORT_TASK"))
            return "ACCEPTED", "", True
        if a == "RESET":
            # the operator's reset: drop the task, leave AUTO, forget the run - never clears KILLED / FAULT (confirm_safe does)
            notes = []
            if self.runner.active:
                self.runner.abort("RESET")
                notes.append("task aborted")
            if self.fsm.state in (VState.V4, VState.V5):
                self.fsm.handle_command(Command(env=cmd.env, action="ABORT_TASK"))
            if self.fsm.state == VState.V3:
                self.fsm.handle_command(Command(env=cmd.env, action="DISABLE_AUTO"))
                notes.append("auto off")
            self.fsm.sticky_holds.clear()
            self.fsm.hold_reason = None
            rc.apply(self.run, cmd)
            self.fsm.declaration_seq = None
            self.fsm.run_id = None
            self.bus.run_id, self.bus.declaration_seq = None, None
            notes.append("run context dropped")
            if self.fsm.state in (VState.V1, VState.V6):
                notes.append(f"{self.fsm.state.value} stays: clear the input, then CONFIRM_SAFE")
            return "ACCEPTED", ", ".join(notes), True
        if a == "SKIP":                               # v0.1.68 (WP17 F4): end the current stage's behaviour now - free run only
            task = self.runner.task
            if task is None:
                return "REJECTED", f"SKIP needs a running task (now {self.runner.task_name or 'none'})", False
            if not getattr(self, "free_run", None):
                return "REJECTED", "skip: free run only", False
            if self.fsm.state != VState.V4:
                return "REJECTED", f"skip: {self.runner.sub_state} - skip needs AUTO_TASK (now {self.fsm.state.value}; resume first)", False
            why = task.skip_check()
            if why:
                return "REJECTED", f"skip: {why}", False
            return "ACCEPTED", task.skip(ctx), True
        if a == "STEP":
            task = self.runner.task
            if task is None or not hasattr(task, "on_step"):
                return "REJECTED", f"STEP needs a running TEST_MANUAL / TEST_AUTO task (now {self.runner.task_name or 'none'})", False
            return "ACCEPTED", task.on_step(ctx), True
        if a in ("SAVE_POINT", "DELETE_POINT"):
            return self._points(a, cmd.args, ctx)
        if a == "HOME":
            return self._home(cmd, ctx)
        if a == "SET_TUNING":
            return self._tune(cmd.args)
        if a == "SET_SPEED_K":                        # v0.1.94.3 (Dmitrii 14 Nov): the live speed knobs - any state, at once
            return self._speed(cmd.args or {})
        if a == "CLEAR_MAP":                          # v0.1.29.1: forget the detections (a task may be steering by them: not then)
            if self.runner.active or self.fsm.state in (VState.V4, VState.V5) and self.runner.task is not None:
                return "REJECTED", f"CLEAR_MAP: a task uses the map ({self.runner.task_name}); abort it first", False
            m = ctx.b.map
            if m is None:
                return "REJECTED", "no perception map on this vehicle", False
            n = len(getattr(m, "ents", {}))
            m.clear()
            return "ACCEPTED", f"map cleared ({n} entities forgotten; saved points stay)", True
        if a == "SELECT_TARGET":
            task = self.runner.task
            if self.fsm.state not in (VState.V4, VState.V5) or self.runner.task_name != "BOX_INSPECT" or task is None:
                return "REJECTED", "SELECT_TARGET needs a running BOX_INSPECT task", False
            # v0.1.27: the vehicle picks GREEN by itself; the operator's override counts until the face is chosen (ALIGN)
            if getattr(getattr(task, "m", None), "state", None) not in ("PREFLIGHT", "GO_SCAN", "OBSERVE", "STRAFE", "ASK_OCS", "HOLD"):
                return "REJECTED", f"BOX_INSPECT has chosen its face already (state {self.runner.sub_state or '?'})", False
            colour = str(cmd.args.get("target_colour") or "").lower()
            if colour not in ("red", "green", "blue"):
                return "REJECTED", "target_colour must be red, green or blue", False
            task.args["target_colour"] = colour
            return "ACCEPTED", f"target colour {colour}", True
        if a == "INCIDENT":
            if self.fsm.state not in (VState.V3, VState.V4, VState.V5):
                return "REJECTED", f"INCIDENT needs an AUTO state (now {self.fsm.state.value})", False
            inc = dict(cmd.args)
            if self.fsm.state == VState.V3 and inc.get("kind") == "assistance_request":
                ok, why = self.fsm.can_start_task(Command(env=cmd.env, action="START_TASK", task="T4"))
                if not ok:
                    return "REJECTED", why, False
                self.fsm.handle_command(Command(env=cmd.env, action="START_TASK", task="T4"))
            msg = self.runner.on_incident(inc, ctx)
            if self.fsm.state == VState.V5 and inc.get("kind") == "readiness_confirm":
                pass
            return "ACCEPTED", msg, True
        if a == "START_TASK":
            fr = getattr(self, "free_run", None)          # v0.1.68 (WP17 F5): free_run.armed refuse - no task while the FCU is armed
            if fr and fr.get("armed", "refuse") == "refuse" and getattr(ctx.b.fcu, "armed", False):
                return "REJECTED", "FREE RUN: the FCU is armed (free_run.armed refuse; disarm, or warn in the profile for the kill test)", False
            if cmd.args.get("sim"):
                # v0.1.19: a `sim` mission (m1t1sim ...) is the same task behind one more guard - never against a real Pixhawk
                link = self.fcu_link() if self.fcu_link else None
                if link != "fake":
                    return "REJECTED", f"sim mission needs the fake FCU (usv_ui holds: {link or 'unknown - no usv_ui beat'}); start usv_ui with --fcu fake, or start it without the sim suffix", False
            if cmd.task not in self.runner.registry:
                # before the FSM moves: a task the vehicle cannot run must not leave the vehicle in V4 with no task
                return "REJECTED", f"task {cmd.task} not built on this vehicle (available: {', '.join(self.runner.registry)})", False
            if cmd.task == "RETURN_HOME":
                return "REJECTED", "RETURN_HOME starts with HOME (the OCS's go-home switch), not START_TASK", False
            cls = self.runner.registry[cmd.task]
            if getattr(cls, "NEEDS_FIELD", False):   # v0.1.85 (WP46): a field task on a real judge needs the RxCourse on the boat
                from . import guidance as GD
                why = GD.course_refusal(self.runner.cfg, ctx)
                if why:
                    return "REJECTED", f"{cmd.task}: {why}", False
            has_pre = cls.precheck is not Task.precheck
            if cmd.args.get("from") or has_pre:      # v0.1.26 entry stage / v0.1.30 precheck: on a probe task before the FSM moves
                probe = cls(self.runner.cfg, on_transition=None, clock=self.clock)
                probe.start(dict(cmd.args), ctx)
                if has_pre:
                    why = probe.precheck(ctx)
                    if why:
                        return "REJECTED", f"{cmd.task}: {why}", False
                if cmd.args.get("from"):
                    why = probe.enter_from(str(cmd.args["from"]), ctx)
                    if why:
                        return "REJECTED", f"from: {why}", False
            if cmd.task == "T1":                     # v0.1.21: checked before the FSM moves (a failed start would leave V4 without a task)
                if cmd.args.get("tier") not in (None, "core", "advanced"):         # v0.1.66: advanced = m1t2 (the UAV's colours)
                    return "REJECTED", f"T1 tier {cmd.args.get('tier')}: core (m1t1) | advanced (m1t2, with the UAV)", False
                if cmd.args.get("planner") not in (None, "simple", "s6"):
                    return "REJECTED", f"T1 planner {cmd.args.get('planner')!r}: simple | s6", False
            result, reason = self.fsm.handle_command(cmd)
            if result == "ACCEPTED":
                self.runner.start(cmd.task, cmd.args, ctx)
            return result, reason, result == "ACCEPTED"
        if a == "ABORT_TASK":
            result, reason = self.fsm.handle_command(cmd)
            if result == "ACCEPTED":
                self.runner.abort("ABORT_TASK")
            return result, reason, True
        if a == "HOLD":
            result, reason = self.fsm.handle_command(cmd)
            if result == "ACCEPTED":
                self.runner.pause()
            return result, reason, True
        if a == "RESUME":
            result, reason = self.fsm.handle_command(cmd)
            if result == "ACCEPTED":
                self.runner.resume()
            return result, reason, True
        if a in ("FCU_MODE", "FCU_ARM", "SET_RC6", "SET_STATION_KEEP"):
            return self._fcu(a, cmd.args)
        if a in ("FENCE_APPLY", "FENCE_ENABLE"):
            return self._fence(a, cmd.args)
        if a in ("ENABLE_AUTO", "DISABLE_AUTO"):
            if a == "ENABLE_AUTO" and self.fsm.clear_rc6_latch():      # v0.1.50: RC6 latched - this later command wins
                log.info("RC6 latch cleared by ENABLE_AUTO (last wins; the next RC6 flip takes the boat back)")
            result, reason = self.fsm.handle_command(cmd)
            if result == "ACCEPTED" and a == "DISABLE_AUTO" and self.runner.active:
                self.runner.abort("DISABLE_AUTO")
            return result, reason, True
        if a == "CONFIRM_SAFE":
            result, reason = self.fsm.handle_command(cmd)
            return result, reason, False      # DONE when V2 is entered
        if a == "GOTO":
            return "REJECTED", "use START_TASK with task GOTO", False
        return "REJECTED", f"unknown action {a}", False

    def _pending_for(self, cmd: Command) -> Pending:
        if cmd.action in ("FENCE_APPLY", "FENCE_ENABLE"):         # v0.1.53: DONE / FAILED when the fence worker has the FCU's answer
            bk = self.fence
            return Pending(cmd, done_when=lambda: bk.result is not None and bk.result[0],
                           fail_when=lambda: bk.result[1] if (bk.result is not None and not bk.result[0]) else None,
                           deadline=self.clock() + 30.0, done_reason=lambda: bk.result[1] if bk.result else "")
        if cmd.action == "CONFIRM_SAFE":
            return Pending(cmd, done_when=lambda: self.fsm.state == VState.V2,
                           fail_when=lambda: None, deadline=self.clock() + 30.0)
        return Pending(cmd, done_when=lambda: True, fail_when=lambda: None, deadline=self.clock() + 1.0)

    def _complete(self, ctx: TaskContext) -> None:
        if not self.pending:
            return
        now = self.clock()
        keep = []
        for p in self.pending:
            fail = p.fail_when()
            if fail:
                self.bus.ack(p.cmd, "FAILED", fail, self.fsm.state.value)
            elif p.done_when():
                self.bus.ack(p.cmd, "DONE", p.done_reason() if p.done_reason else "", self.fsm.state.value)
            elif now > p.deadline:
                self.bus.ack(p.cmd, "FAILED", "timeout waiting for completion", self.fsm.state.value)
            else:
                keep.append(p)
        self.pending = keep
