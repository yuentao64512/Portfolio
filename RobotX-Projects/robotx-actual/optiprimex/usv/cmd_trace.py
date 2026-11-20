"""cmd_trace: what the mission manager told the FCU, one JSON line per event (v0.1.27.1, the no-hardware part of ROADMAP WP9).

The RoverAdapter records every mode request (with the FCU's answer), every mode-20 frame (entry zero, neutral, command), every
GUIDED target / velocity; the mission manager adds what it observed each tick when it changes (the FCU's mode, armed, the
vehicle state, the task and its stage). tools/mode_log.py reads it back into the GUIDE §6.2 table "mode sequence per stage".

  {"k": "start", "t": 0.0, "unix": ..., "profile": "usv_lab.yaml", "version": "0.1.27.1", "fcu": "..."}
  {"k": "mode_req", "t": 1.23, "mode": "CUSTOM_MANUAL", "ok": true}
  {"k": "frame", "t": 1.24, "m": [0.0, 0.0, 0.0, 0.0], "why": "entry" | "neutral" | "cmd"}
  {"k": "target", "t": ...} / {"k": "vel", "t": ...}
  {"k": "obs", "t": ..., "fcu_mode": "HOLD", "armed": false, "state": "AUTO_TASK", "task": "TEST_SWAY", "sub": "SWAY"}

t = seconds since the trace started (monotonic). Off unless the mission manager gets --trace PATH (the launchers pass one).
"""
from __future__ import annotations

import json
import threading
import time
from typing import Any, Optional

from .. import __version__


class CmdTrace:
    def __init__(self, path: str, profile: str = "", fcu: str = ""):
        self.fh = open(path, "a", encoding="utf-8", buffering=1)
        self.t0 = time.monotonic()
        self.lock = threading.Lock()
        self._obs: Optional[tuple] = None
        self.ev("start", unix=round(time.time(), 3), profile=profile, version=__version__, fcu=fcu)

    def ev(self, kind: str, **fields: Any) -> None:
        rec = {"k": kind, "t": round(time.monotonic() - self.t0, 4), **fields}
        with self.lock:
            self.fh.write(json.dumps(rec, separators=(",", ":")) + "\n")

    def frame(self, m, why: str) -> None:
        self.ev("frame", m=[round(float(x), 3) for x in m], why=why)

    def observe(self, fcu_mode, armed, state, task, sub) -> None:
        """Called every tick; writes only when something changed."""
        cur = (fcu_mode, bool(armed), state, task, sub)
        if cur != self._obs:
            self._obs = cur
            self.ev("obs", fcu_mode=fcu_mode, armed=bool(armed), state=state, task=task, sub=sub)

    def close(self) -> None:
        with self.lock:
            self.fh.close()
