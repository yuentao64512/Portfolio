"""K5: the toasts - every button press shows one top-right, and so do the state transitions the operator must see without a click.

ToastBook is the logic (no Tk, tested alone): add(word) -> a grey "⟶ <word> · sent" toast; resolve(id, ok, line) turns it green
"✓ <word>" (+ the backend's first line) or red "✗ <word> — <why>"; the 3-second timer starts at the FINAL state; a toast still waiting
after 10 s turns amber "… <word> — no answer yet" (and still resolves when the answer comes); event(level, text) = a transition toast,
final at once; dismiss(id) on a click; visible(now) = newest first, at most 5, the rest counted ("+n").
Presses (Dmitrii 2026-11-11: "change the button colour when we press it"): the pressed button itself turns amber "sending" at once,
then green / red for 3 s with the answer, then back to its own colour - PressBook, the same timer rules.
ToastManager draws a ToastBook as frameless Toplevels anchored to the window's top-right (16 px inset, 8 px gaps), re-anchored on
<Configure>; the Tk part only draws."""
from __future__ import annotations

import itertools
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

SHOW_S = 3.0          # a final toast stays this long
WAIT_S = 10.0         # a sent toast still unanswered turns amber
MAX_VISIBLE = 5
INSET, GAP = 16, 8


@dataclass
class Toast:
    id: int
    word: str
    level: str          # sent (grey) | ok (green) | bad (red) | wait (amber) | warn (amber, a transition) | info
    text: str
    t0: float
    t_final: Optional[float] = None
    detail: str = ""

    @property
    def final(self) -> bool:
        return self.t_final is not None


class ToastBook:
    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.items: Dict[int, Toast] = {}
        self._ids = itertools.count(1)
        self.log: List[Tuple[float, str, str]] = []          # (t, level, text): every final line, for the LOG tab (kind ui)

    def add(self, word: str) -> int:
        i = next(self._ids)
        self.items[i] = Toast(i, word, "sent", f"⟶ {word} · sent", self.clock())
        self.log.append((time.time(), "sent", f"⟶ {word} · sent"))
        return i

    def resolve(self, i: int, ok: bool, line: str = "", hint: str = "") -> Optional[Toast]:
        t = self.items.get(i)
        if t is None:
            return None
        line = (line or "").strip()
        if ok:
            t.level, t.text = "ok", f"✓ {t.word}"
            t.detail = line
        else:
            t.level, t.text = "bad", f"✗ {t.word} — {line or 'refused'}"
            t.detail = hint
        t.t_final = self.clock()
        self.log.append((time.time(), t.level, t.text + (f" · {t.detail}" if t.detail else "")))
        return t

    def accept(self, i: int, line: str = "") -> Optional[Toast]:
        """v0.1.86.9: the vehicle ACCEPTED, its DONE / FAILED follows - green "✓ word · accepted", not final yet (no timer, no amber)."""
        t = self.items.get(i)
        if t is None or t.final:
            return t
        t.level, t.text, t.detail = "acc", f"✓ {t.word} · accepted", (line or "").strip()
        return t

    def event(self, level: str, text: str) -> int:
        i = next(self._ids)
        now = self.clock()
        self.items[i] = Toast(i, "", level if level in ("ok", "bad", "warn", "info") else "info", text, now, now)
        self.log.append((time.time(), level, text))
        return i

    def dismiss(self, i: int) -> None:
        self.items.pop(i, None)

    def tick(self, now: Optional[float] = None) -> None:
        now = self.clock() if now is None else now
        for i, t in list(self.items.items()):
            if t.final and now - t.t_final >= SHOW_S:
                del self.items[i]
            elif not t.final and t.level == "sent" and now - t.t0 >= WAIT_S:
                t.level, t.text = "wait", f"… {t.word} — no answer yet"

    def visible(self, now: Optional[float] = None) -> Tuple[List[Toast], int]:
        self.tick(now)
        order = sorted(self.items.values(), key=lambda t: -t.id)            # newest on top
        return order[:MAX_VISIBLE], max(0, len(order) - MAX_VISIBLE)


BUSY_S = 30.0         # v0.1.92: a press still "sending" this long no longer blocks a second press (the worker answers within 25 s)
STUCK_S = 60.0        # v0.1.92: a "sending" look this old is dropped (never a button stuck amber)


class PressBook:
    """key -> sending | ok | bad for the pressed button's own colour; '' = its normal look."""

    def __init__(self, clock=time.monotonic):
        self.clock = clock
        self.s: Dict[str, Tuple[str, Optional[float]]] = {}
        self.t0: Dict[str, float] = {}

    def press(self, key: str) -> None:
        self.s[key] = ("sending", None)
        self.t0[key] = self.clock()

    def busy(self, key: str, now: Optional[float] = None) -> bool:
        """v0.1.92 (Dmitrii: "minimise my error with clicking buttons ... and them being stuck"): True while this key's press is still
        waiting for its answer - a double click sends once; after BUSY_S a press goes again (never stuck)."""
        now = self.clock() if now is None else now
        v = self.s.get(key)
        return bool(v) and v[0] == "sending" and now - self.t0.get(key, now) < BUSY_S

    def resolve(self, key: str, ok: bool) -> None:
        self.s[key] = ("ok" if ok else "bad", self.clock())

    def look(self, key: str, now: Optional[float] = None) -> str:
        now = self.clock() if now is None else now
        v = self.s.get(key)
        if not v:
            return ""
        st, t = v
        if t is not None and now - t >= SHOW_S:
            del self.s[key]
            return ""
        if st == "sending" and now - self.t0.get(key, now) >= STUCK_S:
            del self.s[key]
            return ""
        return st


def verdict(r: Optional[dict]) -> Tuple[bool, str, str]:
    """A /cmd (or /ack, /system, /recording) answer -> (ok, the line to show, a hint). The page's rsVerdict: a refusal line wins."""
    if not r:
        return False, "no answer", ""
    lines = [str(x) for x in (r.get("lines") or []) if str(x).strip()]
    bad = next((x for x in lines if any(w in x.lower() for w in ("refused", "rejected", "failed", "unknown command", "not sent",
                                                                  "cannot parse", "busy:", "no answer"))), None)
    hint = next((x.split("hint:", 1)[1].strip() for x in lines if "hint:" in x), "")
    if not r.get("ok", False):
        return False, (bad or r.get("why") or (lines[-1] if lines else "refused")).strip(), hint
    if bad:
        return False, bad.strip(), hint
    acc = next((x for x in lines if any(w in x for w in ("accepted", "ACCEPTED", "DONE"))), None)
    return True, (acc or (lines[0] if lines else (r.get("why") or r.get("text") or "ok"))).strip(), ""


class ToastManager:
    """Draws a ToastBook (Tk, main thread only)."""

    COL = {"sent": ("chipText", "panel2"), "ok": ("ok", "panel"), "acc": ("ok", "panel2"), "bad": ("alarmText", "alarmBg"), "wait": ("warnText", "warnBg"),
           "warn": ("warnText", "warnBg"), "info": ("accText", "accBg")}

    def __init__(self, root, pal: dict, book: Optional[ToastBook] = None, period_ms: int = 200):
        import tkinter as tk
        self.tk, self.root, self.pal = tk, root, pal
        self.book = book or ToastBook()
        self.wins: Dict[int, tuple] = {}
        self.more = None
        self.period_ms = period_ms
        self._alive = True
        # only the window's own move / resize: every child widget carries the root's bindtag, and place() must not answer those
        root.bind("<Configure>", lambda e: self.place() if e.widget is root else None, add="+")
        self._aid = root.after(period_ms, self._loop)

    def stop(self) -> None:
        self._alive = False
        try:
            self.root.after_cancel(self._aid)
        except Exception:
            pass
        for w, *_ in self.wins.values():
            try:
                w.destroy()
            except Exception:
                pass
        self.wins.clear()

    def _loop(self) -> None:
        if not self._alive:
            return
        self.draw()
        self._aid = self.root.after(self.period_ms, self._loop)

    def _win(self, t: Toast):
        tk, p = self.tk, self.pal
        w = tk.Toplevel(self.root)
        w.overrideredirect(True)
        try:
            w.attributes("-topmost", True)
        except tk.TclError:
            pass
        fr = tk.Frame(w, borderwidth=1, relief="solid", background=p["line"])
        fr.pack(fill="both", expand=True)
        lab = tk.Label(fr, text="", justify="left", anchor="w", font=("Menlo", 13, "bold"), padx=12, pady=8, wraplength=420)
        lab.pack(fill="both", expand=True, padx=1, pady=1)
        sub = tk.Label(fr, text="", justify="left", anchor="w", font=("Helvetica", 11), padx=12, wraplength=420)
        for x in (w, fr, lab, sub):
            x.bind("<Button-1>", lambda e, i=t.id: (self.book.dismiss(i), self.draw()))
        return (w, lab, sub, [None])

    def draw(self) -> None:
        shown, more = self.book.visible()
        ids = {t.id for t in shown}
        for i in [i for i in self.wins if i not in ids]:
            self.wins.pop(i)[0].destroy()
        for t in shown:
            if t.id not in self.wins:
                self.wins[t.id] = self._win(t)
            w, lab, sub, last = self.wins[t.id]
            sig = (t.level, t.text, t.detail)
            if last[0] != sig:
                last[0] = sig
                fg, bg = self.COL.get(t.level, self.COL["info"])
                lab.configure(text=t.text, foreground=self.pal[fg], background=self.pal[bg])
                if t.detail:
                    sub.configure(text=t.detail, foreground=self.pal["muted"], background=self.pal[bg])
                    sub.pack(fill="x", padx=1, pady=(0, 6))
                else:
                    sub.pack_forget()
        if more:
            if self.more is None:
                self.more = self.tk.Toplevel(self.root)
                self.more.overrideredirect(True)
                self.more_lab = self.tk.Label(self.more, font=("Menlo", 12), padx=10, pady=4, background=self.pal["panel2"],
                                              foreground=self.pal["muted"])
                self.more_lab.pack()
            self.more_lab.configure(text=f"+{more}")
        elif self.more is not None:
            self.more.destroy()
            self.more = None
        self.place([t.id for t in shown])

    def place(self, order: Optional[List[int]] = None) -> None:
        if order is None:
            order = [t.id for t in self.book.visible()[0]]
        try:
            right = self.root.winfo_rootx() + self.root.winfo_width() - INSET
            y = self.root.winfo_rooty() + INSET
        except self.tk.TclError:
            return
        for i in order + (["more"] if self.more is not None else []):
            w = self.more if i == "more" else self.wins.get(i, (None,))[0]
            if w is None:
                continue
            ww, hh = w.winfo_reqwidth(), w.winfo_reqheight()
            w.geometry(f"+{right - ww}+{y}")
            y += hh + GAP
