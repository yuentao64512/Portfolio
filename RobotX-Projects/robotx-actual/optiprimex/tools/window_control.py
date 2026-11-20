"""window_control: who may command through a vehicle window (v0.1.63, Dmitrii 2026-11-07: the USV window on the team LAN for a delegate).

Decided (polled): ONE COMMANDER, TAKE OVER at once - everyone connected watches; only the commander's buttons send; `take control` moves
command immediately (the previous holder sees "control taken by NAME (ADDR)" and becomes a viewer). A PIN for commands from another
computer - watching is open; a browser that is not on this host proves the PIN once (its session remembers it). This host itself
(127.0.0.1 / ::1) needs none.

A session = a browser tab: an id it keeps (localStorage) + a name, sent with every request (X-Window-Session / X-Window-Name). It is
connected while it polls (the page asks /state.json every second); `stale_s` without a poll = gone - a gone commander frees control, and
the next session that commands gets it without a take (so one person alone never has to press anything; a remote one still needs the PIN).
Pure bookkeeping, thread-safe, clock injectable."""
from __future__ import annotations

import re
import secrets
import threading
import time
from typing import Dict, Optional, Tuple

LOCAL = ("127.0.0.1", "::1", "localhost")
SID_RE = re.compile(r"^[A-Za-z0-9_-]{8,64}$")


def new_pin() -> str:
    return "%04d" % secrets.randbelow(10000)


class Control:
    def __init__(self, pin: Optional[str] = None, stale_s: float = 15.0, clock=time.monotonic, trust_local: bool = True):
        self.pin = (pin or "").strip() or None
        self.stale_s, self.clock, self.trust_local = stale_s, clock, trust_local
        self.lock = threading.Lock()
        self.sessions: Dict[str, dict] = {}          # sid -> {name, addr, t, pin_ok}
        self.commander: Optional[str] = None
        self.events: list = []                       # (t, text) - the last control changes, for the pages' event line

    # ---- bookkeeping ------------------------------------------------------------------------------------------------------
    @staticmethod
    def clean_name(name: Optional[str]) -> str:
        n = re.sub(r"[^A-Za-z0-9 _.-]", "", str(name or "")).strip()[:24]
        return n or "operator"

    def is_local(self, addr: str) -> bool:
        return self.trust_local and str(addr or "") in LOCAL

    def seen(self, sid: Optional[str], name: Optional[str], addr: str) -> Optional[str]:
        """A request from a page: keeps its session alive. None = no (valid) session id: a read-only caller (curl, the OCS)."""
        if not sid or not SID_RE.match(sid):
            return None
        now = self.clock()
        with self.lock:
            s = self.sessions.get(sid)
            if s is None:
                s = self.sessions[sid] = {"name": self.clean_name(name), "addr": addr, "t": now, "pin_ok": False, "since": now}
            s["t"], s["addr"] = now, addr
            if name:
                s["name"] = self.clean_name(name)
            self._expire(now)
        return sid

    def _expire(self, now: float) -> None:
        gone = [k for k, s in self.sessions.items() if now - s["t"] > self.stale_s]
        for k in gone:
            if k == self.commander:
                self.commander = None
                self.events.append((now, f"control free: {self.sessions[k]['name']} ({self.sessions[k]['addr']}) left"))
            del self.sessions[k]
        self.events = self.events[-20:]

    def _authorised(self, s: dict, pin: Optional[str]) -> Tuple[bool, str]:
        if self.pin is None or self.is_local(s["addr"]) or s["pin_ok"]:
            return True, ""
        if pin is not None and secrets.compare_digest(str(pin).strip(), self.pin):
            s["pin_ok"] = True
            return True, ""
        return False, "PIN needed: commands from another computer need the window's PIN (watching needs none)" if not pin else "wrong PIN"

    # ---- the two questions ------------------------------------------------------------------------------------------------
    def take(self, sid: Optional[str], name: Optional[str], addr: str, pin: Optional[str] = None) -> Tuple[bool, str]:
        """`take control`: at once (decided), the PIN first for a remote browser."""
        sid = self.seen(sid, name, addr)
        if sid is None:
            return False, "no session (reload the page)"
        with self.lock:
            s = self.sessions[sid]
            ok, why = self._authorised(s, pin)
            if not ok:
                return False, why
            prev = self.sessions.get(self.commander) if self.commander and self.commander != sid else None
            self.commander = sid
            txt = f"control taken by {s['name']} ({s['addr']})" + (f" from {prev['name']} ({prev['addr']})" if prev else "")
            self.events.append((self.clock(), txt))
            return True, txt

    def release(self, sid: Optional[str]) -> Tuple[bool, str]:
        with self.lock:
            if sid and sid == self.commander:
                s = self.sessions.get(sid) or {}
                self.commander = None
                self.events.append((self.clock(), f"control released by {s.get('name', '?')}"))
                return True, "released"
            return False, "you do not hold control"

    def may_command(self, sid: Optional[str], name: Optional[str], addr: str) -> Tuple[bool, str]:
        """Before any command: the commander may; with control free, the caller gets it (authorised: local, or PIN proved)."""
        sid0 = sid
        sid = self.seen(sid, name, addr)
        if sid is None:
            if sid0 is None and self.is_local(addr):        # a script on this host (curl, the tests): as before v0.1.63
                return True, ""
            return False, "refused: a command needs the window's session (open the page)"
        with self.lock:
            s = self.sessions[sid]
            if self.commander == sid:
                return True, ""
            if self.commander is None:
                ok, why = self._authorised(s, None)
                if not ok:
                    return False, "refused: " + why
                self.commander = sid
                self.events.append((self.clock(), f"control taken by {s['name']} ({s['addr']}) - it was free"))
                return True, ""
            c = self.sessions.get(self.commander) or {}
            return False, f"refused: you are a viewer - {c.get('name', '?')} ({c.get('addr', '?')}) commands; press take control"

    def view(self, sid: Optional[str] = None) -> dict:
        now = self.clock()
        with self.lock:
            self._expire(now)
            c = self.sessions.get(self.commander) if self.commander else None
            me = self.sessions.get(sid) if sid else None
            return {"commander": None if c is None else {"name": c["name"], "addr": c["addr"]},
                    "you": None if me is None else {"name": me["name"], "addr": me["addr"], "commander": sid == self.commander,
                                                    "needs_pin": self.pin is not None and not self.is_local(me["addr"]) and not me["pin_ok"]},
                    "sessions": [{"name": s["name"], "addr": s["addr"], "commander": k == self.commander, "age_s": round(now - s["t"], 1)}
                                 for k, s in sorted(self.sessions.items(), key=lambda kv: kv[1]["since"])],
                    "pin": self.pin is not None, "events": [{"age_s": round(now - t, 1), "text": x} for t, x in self.events[-5:]]}
