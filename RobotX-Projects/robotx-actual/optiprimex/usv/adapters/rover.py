"""RoverAdapter: behaviour outputs -> ArduRover modes and messages (ADR-5). The only motion sender.

GUIDED for position / velocity-yaw targets, mode 20 (CUSTOM_MANUAL) for direct thrust frames,
HOLD for the safe hold. Zero frame on entering mode 20, keepalive every tick (firmware timeout 250 ms),
neutral frame on cancel. Never arms.

v0.1.68 (WP17 F3, free run): with `ghost` set (usv/ghost.GhostNav, the mission manager's --free-run) the adapter is MUTED - it never
calls send_actuator_frame / send_position_target_global / send_velocity_yaw / do_change_speed (not the mode-20 entry zero frame, not
the cancel neutral frame: the firmware's 250 ms frame timeout neutralises the outputs itself); every output goes to the ghost instead
and is traced `ghost`. The mode requests still go to the FCU (GUIDED / CUSTOM_MANUAL / HOLD, the hold_reassert_s rule); a refused
mode is logged once per (stage, mode), traced `mode_req ok:false free_run:true`, re-asked every hold_reassert_s, and never stops
the behaviour.
"""
from __future__ import annotations

import logging
import time
from typing import Optional, Union

from ...companion.config import Config
from ...companion.link import MavLink
from ...companion.state import GuidedTarget, Hold, Loiter, ThrusterFrame, VelocityYawTarget

log = logging.getLogger("optiprimex.rover")
Output = Union[ThrusterFrame, GuidedTarget, VelocityYawTarget, Hold, None]


class RoverAdapter:
    def __init__(self, link: MavLink, cfg: Config):
        self.link = link
        self.cfg = cfg
        self.direct_active = False
        self.mode_wanted: Optional[str] = None
        self._last_hold_assert = 0.0
        self._last_target: Optional[GuidedTarget] = None
        self._last_target_t = 0.0
        self._speed_set: Optional[float] = None
        self._pending: Output = None
        self.sent = {"frames": 0, "targets": 0, "velocity": 0, "mode": 0}
        self.trace = None                  # v0.1.27.1: usv/cmd_trace.CmdTrace when the mission manager runs with --trace
        self.ghost = None                  # v0.1.68 (F3): usv/ghost.GhostNav in free run = muted (nothing but mode requests to the FCU)
        self.clock = time.monotonic
        self.stage: Optional[str] = None   # the task's stage now (the mission manager sets it): a refusal is logged once per (stage, mode)
        self._refused: set = set()
        self._fr_asked: dict = {}          # free run: mode -> when it was last requested (a refused mode is re-asked every hold_reassert_s)
        self._backoff: dict = {}           # v0.1.86.5 (WP43 Tier 1.4): mode -> not asked again before this time (a refusal / no ack)

    @property
    def muted(self) -> bool:
        return self.ghost is not None

    def _frame(self, frame: ThrusterFrame, why: str) -> None:
        if self.muted:                                         # v0.1.68 (F3): the ghost gets it, never the FCU
            self.ghost.set_frame(frame.as_list(), self.clock())
            if self.trace:
                self.trace.ev("ghost", out="frame", m=[round(float(x), 3) for x in frame.as_list()], why=why)
            return
        self.link.send_actuator_frame(frame)
        self.sent["frames"] += 1
        if self.trace:
            self.trace.frame(frame.as_list(), why)

    def _set_mode(self, name: str, timeout_s: float) -> bool:
        """v0.1.86.5 (WP43 Tier 1.4): the tick never blocks more than timeouts.mode_ack_s on the FCU; a refused / unacked mode is not asked
        again for timeouts.mode_backoff_s (False meanwhile, nothing sent), one warning per (mode) streak."""
        now = self.clock()
        if now < self._backoff.get(name, float("-inf")):
            return False
        ok = self.link.set_mode(name, timeout_s=min(float(timeout_s), float(self.cfg.timeouts.mode_ack_s)))
        if ok:
            self._backoff.pop(name, None)
        else:
            if name not in self._backoff:
                log.warning("mode %s: no ack / refused within %.1f s - not asked again for %.1f s", name, self.cfg.timeouts.mode_ack_s,
                            self.cfg.timeouts.mode_backoff_s)
            self._backoff[name] = now + float(self.cfg.timeouts.mode_backoff_s)
        self.sent["mode"] += 1
        if self.trace:
            if self.muted:
                self.trace.ev("mode_req", mode=name, ok=bool(ok), free_run=True, stage=self.stage)
            else:
                self.trace.ev("mode_req", mode=name, ok=bool(ok))
        if self.muted and not ok and (self.stage, name) not in self._refused:
            self._refused.add((self.stage, name))
            log.warning("FREE RUN: the FCU refused %s in %s - ignored, the ghost goes on (logged once per stage and mode)",
                        name, self.stage or "no stage")
        return ok

    def _want(self, name: str) -> None:
        """v0.1.68 (F3): free run - ask the FCU for the mode (a new wish at once, a refused one again every hold_reassert_s); the
        answer never stops the output (the ghost takes it either way)."""
        now = self.clock()
        if self.current_mode() == name:
            self.mode_wanted = name
            return
        if self.mode_wanted != name or now - self._fr_asked.get(name, float("-inf")) >= self.cfg.timeouts.hold_reassert_s:
            self._fr_asked[name] = now
            self._set_mode(name, 0.5)
        self.mode_wanted = name

    # ---- mode management ---------------------------------------------------------------
    def current_mode(self) -> Optional[str]:
        return self.link.state().mode_name

    def ensure_mode(self, name: str, timeout_s: float = 0.5) -> bool:
        if self.current_mode() == name:
            self.mode_wanted = name
            return True
        ok = self._set_mode(name, timeout_s)
        if ok:
            self.mode_wanted = name
        return ok

    # ---- behaviour outputs ---------------------------------------------------------------
    def hold(self, now: Optional[float] = None, mode: str = "HOLD") -> bool:
        """HOLD (v0.1.81: or LOITER - the same reassert rule, `mode`)."""
        now = time.monotonic() if now is None else now
        if self.direct_active:
            self.neutral()
            self.direct_active = False
        if self.muted and self.ghost.kind != "hold":
            self.ghost.set_hold()
            if self.trace:
                self.trace.ev("ghost", out="hold")
        due = (now - self._last_hold_assert) >= self.cfg.timeouts.hold_reassert_s
        if self.current_mode() != mode or due:
            self._last_hold_assert = now
            ok = self._set_mode(mode, 0.5)                     # re-asserted every hold_reassert_s even if already HOLD / LOITER
            if ok:
                self.mode_wanted = mode
            return ok
        return True

    def loiter(self, now: Optional[float] = None) -> bool:
        """v0.1.81 (WP37): the autonomous station keep as a real position hold (LOITER)."""
        return self.hold(now, "LOITER")

    def release(self) -> None:
        self.mode_wanted = None

    def guided_goto(self, t: GuidedTarget, now: Optional[float] = None) -> bool:
        now = time.monotonic() if now is None else now
        if self.direct_active:
            self.neutral()
            self.direct_active = False
        if self.muted:
            return self._ghost_goto(t, now)
        if not self.ensure_mode("GUIDED"):
            return False
        changed = (self._last_target is None or abs(t.lat - self._last_target.lat) > 1e-7
                   or abs(t.lon - self._last_target.lon) > 1e-7)
        if t.speed_mps is not None and t.speed_mps != self._speed_set:
            if self.link.do_change_speed(t.speed_mps, timeout_s=0.3):
                self._speed_set = t.speed_mps
        if changed or (now - self._last_target_t) >= 0.5:   # carrot at most 2 Hz
            self.link.send_position_target_global(t.lat, t.lon)
            self.sent["targets"] += 1
            if self.trace:
                self.trace.ev("target", lat=round(t.lat, 7), lon=round(t.lon, 7))
            self._last_target, self._last_target_t = t, now
        return True

    def _ghost_goto(self, t: GuidedTarget, now: float) -> bool:
        """v0.1.68 (F3): GUIDED to the ghost - the same carrot rule (on change, else at most 2 Hz) and speed, nothing to the FCU."""
        self._want("GUIDED")
        if t.speed_mps is not None and t.speed_mps != self._speed_set:
            self.ghost.set_speed(t.speed_mps)
            self._speed_set = t.speed_mps
        changed = (self._last_target is None or abs(t.lat - self._last_target.lat) > 1e-7
                   or abs(t.lon - self._last_target.lon) > 1e-7)
        if changed or (now - self._last_target_t) >= 0.5:
            self.ghost.set_target(t.lat, t.lon, self.clock())
            if self.trace:
                self.trace.ev("ghost", out="target", lat=round(t.lat, 7), lon=round(t.lon, 7), speed=t.speed_mps)
            self._last_target, self._last_target_t = t, now
        return True

    def guided_velocity(self, t: VelocityYawTarget) -> bool:
        if self.direct_active:
            self.neutral()
            self.direct_active = False
        if self.muted:                                          # v0.1.68 (F3)
            self._want("GUIDED")
            self.ghost.set_vel(max(0.0, t.vx_mps), t.yaw_rate_rad_s, self.clock())
            if self.trace:
                self.trace.ev("ghost", out="vel", vx=round(t.vx_mps, 3), r=round(t.yaw_rate_rad_s, 3))
            return True
        if not self.ensure_mode("GUIDED"):
            return False
        self.link.send_velocity_yaw(max(0.0, t.vx_mps), t.yaw_rate_rad_s)
        self.sent["velocity"] += 1
        if self.trace:
            self.trace.ev("vel", vx=round(t.vx_mps, 3), r=round(t.yaw_rate_rad_s, 3))
        return True

    def direct(self, frame: ThrusterFrame) -> bool:
        if not self.direct_active:
            if self.muted:                                      # v0.1.68 (F3): the mode is asked, the frames go to the ghost
                self._want("CUSTOM_MANUAL")
            elif not self.ensure_mode("CUSTOM_MANUAL"):
                return False
            self._frame(ThrusterFrame.zero(), "entry")             # zero frame on entry (firmware rule)
            self.direct_active = True
        self._frame(self._clip(frame), "cmd")
        return True

    def neutral(self) -> None:
        try:
            self._frame(ThrusterFrame.zero(), "neutral")
        except Exception:
            log.exception("neutral frame failed")

    def cancel(self) -> None:
        """Behaviour cancelled: neutral if direct, then HOLD."""
        if self.direct_active:
            self.neutral()
            self.direct_active = False
        self._last_target = None
        self.hold()

    def allocate(self, u: float, v: float, r: float) -> ThrusterFrame:
        """surge u, sway v, yaw r in [-1, 1] -> M1=u+r, M2=stern*v, M3=u-r, M4=v, jointly scaled to max_amp."""
        f = ThrusterFrame(m1=u + r, m2=self.cfg.gains.stern_factor * v, m3=u - r, m4=v)
        return self._clip(f)

    def _clip(self, f: ThrusterFrame) -> ThrusterFrame:
        peak = max(abs(x) for x in f.as_list()) or 0.0
        amp = self.cfg.limits.max_amp
        if peak > amp:
            s = amp / peak
            return ThrusterFrame(m1=f.m1 * s, m2=f.m2 * s, m3=f.m3 * s, m4=f.m4 * s)
        return f

    # ---- per tick ------------------------------------------------------------------------
    def apply(self, out: Output, now: Optional[float] = None) -> None:
        """Send one behaviour output. None means 'nothing this tick' (keepalive handled below)."""
        if isinstance(out, ThrusterFrame):
            self.direct(out)
        elif isinstance(out, GuidedTarget):
            self.guided_goto(out, now)
        elif isinstance(out, VelocityYawTarget):
            self.guided_velocity(out)
        elif isinstance(out, Loiter):                       # v0.1.81: before Hold (a Loiter is a Hold)
            self.loiter(now)
        elif isinstance(out, Hold):
            self.hold(now)

    def flush(self, now: Optional[float] = None) -> None:
        """Called at the end of every tick: mode-20 keepalive when direct is active and nothing was sent."""
        if self.direct_active and self.mode_wanted == "CUSTOM_MANUAL":
            pass  # frames are sent by direct(); a behaviour that stops sending must cancel()
