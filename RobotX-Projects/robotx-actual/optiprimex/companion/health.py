"""HealthMonitor: link ages, RC, kill, EKF, bridge, loop timing -> Health + fault events (decision 9)."""
from __future__ import annotations

import time
from typing import List, Optional

from .config import Config
from .state import FcuState, Health

FAULTS = ("FCU_LOST", "FCU_FAILSAFE", "EKF_BAD", "RC_LOST", "PERCEPTION_LOST", "CLOCK_BAD", "LOOP_SLOW",
          "UNEXPECTED_DISARM")


class HealthMonitor:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.health = Health()
        self._slow_count = 0
        self._last_tick: Optional[float] = None
        self._armed_before: Optional[bool] = None
        self.expect_armed = False      # set by the vehicle layer in V3-V5
        self._ekf_bad_since: Optional[float] = None   # v0.1.59 (A2): EKF_BAD is soft until continuous for guards.ekf_bad_hard_s
        self._now = 0.0

    def tick_timing(self, now: float) -> bool:
        """Call once per tick; returns True when two consecutive ticks exceeded loop_slow_s."""
        slow = False
        if self._last_tick is not None and (now - self._last_tick) > self.cfg.timeouts.loop_slow_s:
            self._slow_count += 1
            slow = self._slow_count >= 2
        else:
            self._slow_count = 0
        self._last_tick = now
        return slow

    def update(self, fcu: FcuState, bridge_up: bool, perception_age_s: float, now: float,
               fcu_failsafe: bool = False) -> Health:
        t = self.cfg.timeouts
        faults: List[str] = []
        fcu_age = min(fcu.age("HEARTBEAT"), fcu.age("GLOBAL_POSITION_INT"), fcu.age("ATTITUDE")) if fcu.seen_heartbeat else float("inf")
        if fcu_age > t.fcu_s:
            faults.append("FCU_LOST")
        if fcu_failsafe:
            faults.append("FCU_FAILSAFE")
        if fcu.seen_heartbeat and fcu.age("EKF_STATUS_REPORT") < float("inf") and not fcu.ekf_pos_ok:
            faults.append("EKF_BAD")
            if self._ekf_bad_since is None:
                self._ekf_bad_since = now
        else:
            self._ekf_bad_since = None
        self._now = now
        rc_ok = fcu.rc_ok and fcu.age("RC_CHANNELS") <= t.rc_s
        if fcu.seen_heartbeat and not rc_ok:
            faults.append("RC_LOST")
        if t.perception_s > 0 and perception_age_s > t.perception_s:
            faults.append("PERCEPTION_LOST")
        if self.expect_armed and fcu.armed is False and self._armed_before:
            faults.append("UNEXPECTED_DISARM")
        self._armed_before = fcu.armed
        h = Health(fcu_age_s=fcu_age, rc_ok=rc_ok, rc_detail=fcu.rc_detail, kill=fcu.kill, estop=fcu.estop,
                   perception_age_s=perception_age_s, bridge_up=bridge_up, clock_ok=True,
                   loop_slow=self.health.loop_slow, faults=faults)
        self.health = h
        return h

    # convenience predicates on the last update
    def fcu_ok(self) -> bool:
        return "FCU_LOST" not in self.health.faults

    def rc_ok(self) -> bool:
        return self.health.rc_ok

    def pos_ok(self) -> bool:
        return "EKF_BAD" not in self.health.faults and self.fcu_ok()

    def clock_ok(self) -> bool:
        return self.health.clock_ok

    def fault_events(self) -> List[str]:
        return list(self.health.faults)

    def ekf_bad_hard(self) -> bool:
        """v0.1.59 (A2, the compliance pass #12): EKF_BAD hardens to V6 only after guards.ekf_bad_hard_s continuous; before that
        it is a soft fault (a running task -> V5 AUTO_SAFE_HOLD, STATE_AUTO on the judges' heartbeat). 0 = hard at once (as before)."""
        if self._ekf_bad_since is None:
            return False
        return (self._now - self._ekf_bad_since) >= self.cfg.guards.ekf_bad_hard_s

    def hard_faults(self) -> List[str]:
        """Faults that force V6 (FAULT). RC loss is handled by the FCU failsafe (HOLD), not by us.
        guards.ignore_faults (the lab profile) keeps a fault visible in status.health.faults but out of this list."""
        ign = self.cfg.guards.ignore_faults
        return [f for f in self.health.faults if f in ("FCU_LOST", "FCU_FAILSAFE", "EKF_BAD", "UNEXPECTED_DISARM") and f not in ign
                and (f != "EKF_BAD" or self.ekf_bad_hard())]

    def soft_faults(self) -> List[str]:
        """Faults that force V5 (AUTO_SAFE_HOLD) while a task runs; RC_LOST too with guards.rc_lost_action: hold (v0.1.29.3)."""
        ign = self.cfg.guards.ignore_faults
        soft = ("PERCEPTION_LOST", "LOOP_SLOW") + (("RC_LOST",) if self.cfg.guards.rc_lost_action == "hold" else ())
        return [f for f in self.health.faults if (f in soft or (f == "EKF_BAD" and not self.ekf_bad_hard())) and f not in ign]
