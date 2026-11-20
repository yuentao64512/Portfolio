"""The Tk widgets. Tk is touched only from the main thread: the poller, the command worker and the camera worker hand their results to
`App.deliver` (a thread-safe queue) and `App.pump` (a root.after loop on the main thread) runs them. Widgets are built once and then
written by diff (`App.put`: a text / colour is written only when it changed) - no flicker, no rebuilt buttons.

Every button calls App.press(key) -> App.send(word): the one code path (K4). A press shows a toast at once (K5) and colours the
button itself (amber sending -> green ✓ / red ✗ for 3 s, Dmitrii 2026-11-11). ARM, STOP & RESET and the like ask first in an
in-window dialog (never a native modal: the event loop keeps running)."""
from __future__ import annotations

import io
import queue
import threading
import time
import tkinter as tk
import webbrowser
from tkinter import ttk
from typing import Callable, Dict, List, Optional

from . import theme as TH
from .canvasmap import CanvasMap, build_layer
from .hosts import HostsPanel
from . import panels as PN
from .model import (MOD_FROZEN, Btn, ConsoleModel, View, command_catalogue, logging_view, mission_line, mission_stages, mods_flags,
                    uav_run_plan, vehicle_cards)
from .net import CamWorker, CommandWorker, Poller
from .toasts import PressBook, ToastBook, ToastManager, verdict

PUMP_MS = 50
LOOK_MS = 250
HEADER_KEYS = ("ident", "links", "fcu", "v", "guard", "rec", "link", "clock")
LADDER_MAX = 8                                                              # v0.1.89: the mission-1 ladder (up to 8 steps)
STEP_KEYS = {n: [f"lad{n}-{i}" for i in range(4)] for n in range(1, LADDER_MAX + 1)}   # generic slots: the model fills them per step
# v0.1.90.5: the crew row without PILOT READY (ready by itself)
USV_ROWS = [("crew", ["usv-hb"]), ("autonomy", ["usv-auto"]), ("station keep", ["usv-sk-hold", "usv-sk-loiter"]),
            ("task", ["usv-start", "usv-hold", "usv-resume", "usv-abort"]), ("FCU", ["fcu-manual", "fcu-hold", "fcu-arm", "fcu-disarm"]),
            ("", ["stop-reset"])]
UAV_ROW = ["uav-hb", "uav-decl", "uav-scan"]                                 # v0.1.90.5: no PILOT READY (by itself)
UAV_RUN_ROW = ["uav-run"]
MOD_KEYS_UI = ["mod-start", "mod-look", "mod-lucky", "mod-any", "mod-squeeze", "mod-budget", "mod-exit"]   # v0.1.93.1: + any, squeeze           # v0.1.92: the mission modifiers subpanel
LOG_TAGS = {"bad": "alarm", "warn": "warnText", "ok": "ok", "ui": "accText"}
ALARM_SLOTS = 4
LOG_KEEP = 400
REC_ANSWER_S = 45.0          # /recording + depth: the backend waits up to 20 s for the first pair (+ ssh)


class ScrollFrame:
    """A column that scrolls (wheel / trackpad while the pointer is over it, and a scrollbar) - 2026-11-11 (Dmitrii: "I can scroll
    properly to UAV"). `.inner` is the frame to fill; its width follows the column."""

    def __init__(self, parent, bg: str):
        self.outer = tk.Frame(parent, background=bg)
        self.canvas = tk.Canvas(self.outer, background=bg, highlightthickness=0, borderwidth=0)
        self.bar = ttk.Scrollbar(self.outer, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=self.bar.set)
        self.bar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.inner = tk.Frame(self.canvas, background=bg)
        self.win = self.canvas.create_window(0, 0, window=self.inner, anchor="nw")
        self.inner.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda e: self.canvas.itemconfigure(self.win, width=e.width))
        for w in (self.canvas, self.inner):
            w.bind("<Enter>", lambda e: self._wheel(True), add="+")
            w.bind("<Leave>", lambda e: self._wheel(False), add="+")

    def _wheel(self, on: bool) -> None:
        if on:
            self.canvas.bind_all("<MouseWheel>", self._on_wheel)
            self.canvas.bind_all("<Button-4>", lambda e: self.canvas.yview_scroll(-3, "units"))
            self.canvas.bind_all("<Button-5>", lambda e: self.canvas.yview_scroll(3, "units"))
        else:
            for s in ("<MouseWheel>", "<Button-4>", "<Button-5>"):
                self.canvas.unbind_all(s)

    def _on_wheel(self, e) -> None:
        if self.canvas.yview() == (0.0, 1.0):
            return                                            # everything fits: nothing to scroll
        d = e.delta if abs(e.delta) < 20 else e.delta // 120   # macOS small deltas, Windows 120 steps
        self.canvas.yview_scroll(int(-d) or (-1 if e.delta > 0 else 1), "units")


class Tooltip:
    """A hover tip (2026-11-11: "can we do tooltips?"): `text_fn()` is asked when the pointer rests 450 ms on the widget (a Treeview:
    text_fn(row id)); the tip follows the row under the pointer and goes on leave / click. Main thread only."""

    def __init__(self, widget, text_fn, pal: dict, tree: bool = False):
        self.w, self.fn, self.pal, self.tree = widget, text_fn, pal, tree
        self.tip = None
        self.key = None
        self.job = None
        widget.bind("<Motion>", self._motion, add="+")
        widget.bind("<Leave>", lambda e: self.hide(), add="+")
        widget.bind("<ButtonPress>", lambda e: self.hide(), add="+")

    def _motion(self, e) -> None:
        key = self.w.identify_row(e.y) if self.tree else "w"
        if key == self.key and (self.tip or self.job):
            return
        self.hide()
        self.key = key
        if key:
            self.job = self.w.after(450, lambda: self.show(e.x_root, e.y_root))

    def show(self, x: int, y: int) -> None:
        self.job = None
        try:
            text = self.fn(self.key) if self.tree else self.fn()
        except Exception:
            text = ""
        if not text:
            return
        p = self.pal
        self.tip = tw = tk.Toplevel(self.w)
        tw.overrideredirect(True)
        try:
            tw.attributes("-topmost", True)
        except tk.TclError:
            pass
        tk.Label(tw, text=text, justify="left", background=p["panel2"], foreground=p["text"], font=("Menlo", 11), wraplength=560,
                 padx=10, pady=8, relief="solid", borderwidth=1).pack()
        tw.geometry(f"+{x + 14}+{y + 16}")

    def hide(self) -> None:
        if self.job:
            try:
                self.w.after_cancel(self.job)
            except tk.TclError:
                pass
            self.job = None
        if self.tip is not None:
            try:
                self.tip.destroy()
            except tk.TclError:
                pass
            self.tip = None
        self.key = None


class Confirm:
    """Asks first, inside the window (grab, Enter = yes, Escape = no); `then()` runs on yes. typed = a word the operator must type first
    (SYSTEM's dangerous actions, as on the page)."""

    def __init__(self, app: "App", text: str, yes: str, then: Callable[[], None], typed: str = ""):
        p = app.pal
        self.w = w = tk.Toplevel(app.root, background=p["panel"])
        w.title("confirm")
        w.transient(app.root)
        w.resizable(False, False)
        tk.Label(w, text=text, wraplength=460, justify="left", font=("Helvetica", 14), background=p["panel"], foreground=p["text"],
                 padx=18, pady=16).pack(fill="x")
        self.typed, self.entry = typed, None
        if typed:
            tk.Label(w, text=f"type  {typed}  to confirm:", background=p["panel"], foreground=p["muted"], font=("Helvetica", 12),
                     padx=18).pack(anchor="w")
            self.entry = tk.Entry(w, font=("Menlo", 14), width=16)
            self.entry.pack(anchor="w", padx=18, pady=(4, 0))
        row = tk.Frame(w, background=p["panel"], padx=14, pady=12)
        row.pack(fill="x")

        def yes_cb():
            if self.typed and (self.entry is None or self.entry.get().strip() != self.typed):
                self.entry.configure(background=p["alarmBg"])
                return
            self.close()
            then()
        self.yes = ttk.Button(row, text=yes, style="Bact.TButton", command=yes_cb)
        self.no = ttk.Button(row, text="CANCEL", style="B.TButton", command=self.close)
        self.no.pack(side="right", padx=(8, 0))
        self.yes.pack(side="right")
        w.bind("<Escape>", lambda e: self.close())
        w.bind("<Return>", lambda e: yes_cb())
        w.update_idletasks()
        x = app.root.winfo_rootx() + (app.root.winfo_width() - w.winfo_reqwidth()) // 2
        y = app.root.winfo_rooty() + 140
        w.geometry(f"+{x}+{y}")
        try:
            w.grab_set()
        except tk.TclError:
            pass
        (self.entry or self.no).focus_set()
        app.dialogs.append(self)

    def close(self) -> None:
        try:
            self.w.grab_release()
            self.w.destroy()
        except tk.TclError:
            pass


class App:
    def __init__(self, root: tk.Tk, ocs: str, fast_hz: float = 5.0, theme: str = "light", start: bool = True):
        self.root, self.ocs = root, ocs.rstrip("/")
        self.pal = TH.palette(theme)
        self.model = ConsoleModel()
        self.q: "queue.Queue" = queue.Queue()
        self.state: Optional[dict] = None
        self.fast: Optional[dict] = None
        self.alarms: Optional[dict] = None
        self.misses = 0
        self.view: Optional[View] = None
        self._cache: Dict[str, tuple] = {}
        self.closed = False
        self.dialogs: List[Confirm] = []
        self.buttons: Dict[str, ttk.Button] = {}
        self.btn_model: Dict[str, Btn] = {}
        self.toasts = ToastBook()
        self.presses = PressBook()
        self.cmd_results: List[tuple] = []
        self.sent_lines: List[str] = []                      # every word send() posted (the smoke test reads it)
        self.cam_state: dict = {}
        self.prefs = PN.load_prefs()                             # 2026-11-12: the console's preferences (⚙ SETTINGS · CONSOLE)
        self.mapj: Optional[dict] = None
        self._map_sig = None
        self.rec_state: dict = {}
        self._cam_started = False
        self._photo = None
        self._style()
        self._build()
        self.toaster = ToastManager(root, self.pal, self.toasts)
        self.poller = Poller(self.ocs, fast_hz, deliver=self.deliver, on_state=self._on_state, on_fast=self._on_fast,
                             on_alarms=self._on_alarms, on_meta=self._on_meta, on_course=self._on_course,
                             on_map=self._on_map, on_hosts=self._on_hosts)
        self.worker = CommandWorker(self.ocs, deliver=self.deliver)
        self.cam = CamWorker(self.ocs, deliver=self.deliver, on_cam=self._on_cam, on_frame=self._on_frame, on_rec=self._on_rec)
        root.protocol("WM_DELETE_WINDOW", self.close)
        self._dirty = True
        if start:
            self.poller.start()
            self.worker.start()
            self.cam.start()
        self._after: Dict[str, str] = {}
        self._later("pump", PUMP_MS, self.pump)
        self._later("looks", LOOK_MS, self._looks_loop)

    def _later(self, name: str, ms: int, fn) -> None:
        """root.after with the id kept: close() cancels every pending one (a timer left behind fires into the next Tk)."""
        if not self.closed:
            self._after[name] = self.root.after(ms, fn)

    # ---- threads -> the main thread ----------------------------------------------------------------------------------------
    def deliver(self, fn, *args) -> None:
        """Called from any thread; never touches Tk."""
        self.q.put((fn, args))

    def pump(self) -> None:
        if self.closed:
            return
        try:
            for _ in range(200):
                fn, args = self.q.get_nowait()
                try:
                    fn(*args)
                except Exception as exc:                         # a bad answer never kills the loop
                    print(f"[ocs_tk] handler error: {exc!r}", flush=True)
        except queue.Empty:
            pass
        if self._dirty:
            self._dirty = False
            try:
                self.render()
            except Exception as exc:
                print(f"[ocs_tk] render error: {exc!r}", flush=True)
        self._later("pump", PUMP_MS, self.pump)

    def _looks_loop(self) -> None:
        if self.closed:
            return
        self.refresh_buttons()
        self._later("looks", LOOK_MS, self._looks_loop)

    def _on_state(self, s, misses, why):
        self.misses = misses
        if s is not None:
            self.state = s
        self._dirty = True

    def _on_fast(self, f):
        if f is not None:
            self.fast = f
            self._dirty = True

    def _on_alarms(self, a):
        if a is not None:
            self.alarms = a

    def _on_meta(self, m):
        self.model.set_meta(m)
        self._dirty = True

    def _on_hosts(self, body, why=""):
        self.hosts_body, self.hosts_why = body, why
        try:
            if self.nb.tab(self.nb.select(), "text") == "HOSTS":
                self.hosts.render(body, why, self._link_est())
        except tk.TclError:
            pass

    def _link_est(self) -> Dict[str, dict]:
        """What the OCS already sees of each Jetson's link, for the HOSTS signal bar until /hosts.json measures rtt / loss."""
        s = self.state or {}
        return {"nx": {"age_s": s.get("status_age"), "up": bool(s.get("bridge")), "what": "boat status"},
                "agx": {"age_s": s.get("perception_age"), "up": s.get("perception_age") is not None, "what": "perception beat"}}

    def _on_map(self, mp):
        if mp is not None:
            self.mapj = mp
            self._dirty = True

    def _on_course(self, c):
        self.model.set_course(c)
        self._dirty = True

    def close(self) -> None:
        if self.closed:
            return
        self.closed = True
        for aid in self._after.values():
            try:
                self.root.after_cancel(aid)
            except tk.TclError:
                pass
        for d in self.dialogs:
            d.close()
        self.toaster.stop()
        self.poller.stop()
        self.worker.stop()
        self.cam.stop()
        # the Tk variables die here, on the main thread, before the interpreter (a later GC would call into a dead Tk)
        self.mis_vars, self.mis_opts = {}, []
        self.log_filter = self.mis_auto = self.rec_depth = None
        self.sys_vars, self.pref_vars = {}, {}
        import gc
        gc.collect()
        try:
            self.root.destroy()
        except tk.TclError:
            pass

    def threads_left(self):
        return [t.name for t in threading.enumerate() if t.name.startswith("ocs_tk-") and t.is_alive()]

    # ---- the one command path ----------------------------------------------------------------------------------------------
    def press(self, key: str) -> None:
        """Every button's command. ACK and RECORD post their own endpoints (/ack, /recording) through the same queue and lifecycle."""
        if key.startswith("ack-"):
            b = self.btn_model.get(key)
            if b is not None:
                akey = None if key == "ack-all" else b.word[4:]
                self.post("/ack", {"key": akey or "all"}, "ACK " + (akey or "all"), key)
            return
        if key == "cam-rec":
            self._record()
            return
        if key == "uav-run":
            self._uav_run()
            return
        if key == "big":
            self._big(not self.big_on)
            return
        if key == "settings":
            self._settings(not self.set_on)
            return
        if key in ("cfg-save", "cfg-restore", "cfg-delete"):
            self._config_action(key)
            return
        if key.startswith("tune:"):
            self._tune_set(key[5:])
            return
        if key == "tune-save":
            self.send("tune save", key="tune-save", confirm="Save a copy of the mission manager's profile with the live knob values ON THE BOAT "
                      "(logs/launch/<stamp>/usv_tuned.yaml; the profile itself is never changed)?", yes="SAVE")
            return
        if key.startswith("sys:"):
            self._system_action(key[4:])
            return
        if key in ("log-hdr", "log-cam"):
            self._log_session(key)
            return
        b = self._dynamic(key) or self.btn_model.get(key)
        if b is None:
            return
        if key.startswith("mod-") and not b.ok:                    # v0.1.92: the modifiers are frozen from ENABLE AUTO on
            self.toasts.event("warn", f"{b.label}: {b.why or MOD_FROZEN}")
            self.toaster.draw()
            return
        if self.presses.busy(key):                                 # v0.1.92: a double click sends once (the answer is awaited)
            self.toasts.event("info", f"{b.label.strip('✓▶■❚ ')}: already sent - waiting for its answer")
            self.toaster.draw()
            return
        self.send(b.word, key=key, confirm=b.confirm, yes={"mod-lucky": "LUCKY ON", "mod-any": "LUCKY ANY", "mod-squeeze": "SQUEEZE ON"}.get(key, b.label))   # the label is the state now

    def send(self, word: str, key: str = "", confirm: str = "", yes: str = "") -> None:
        """Every button and the CMD line end here: (ask first) -> toast + the button's colour -> POST /cmd in the worker."""
        word = (word or "").strip()
        if not word:
            return
        if confirm:
            Confirm(self, confirm, "YES, " + (yes or word).strip("✓▶■❚ ").upper(), lambda: self._send(word, key))
            return
        self._send(word, key)

    def _send(self, word: str, key: str) -> None:
        tid = self.toasts.add(word)
        if key:
            self.presses.press(key)
        self.sent_lines.append(word)
        self.toaster.draw()
        self.refresh_buttons()
        self.worker.send(word, lambda r: self._answer(tid, key, word, r), on_final=lambda f: self._final(tid, key, word, f))

    def _answer(self, tid: int, key: str, word: str, r: dict) -> None:
        ok, line, hint = verdict(r)
        if ok and word == "report on uav":                         # 2026-11-11: the course to the UAV's listener as its heartbeats start
            self.root.after(300, lambda: self._send("uav resend_course", ""))
        if ok and (r or {}).get("final") is False:                 # v0.1.86.9: ACCEPTED now, the DONE / FAILED follows (_final)
            self.toasts.accept(tid, line)
        else:
            self.toasts.resolve(tid, ok, line, hint)
        if key:
            self.presses.resolve(key, ok)
        self.cmd_results.insert(0, (time.strftime("%H:%M:%S"), word, ok, list((r or {}).get("lines") or [line])))
        del self.cmd_results[30:]
        self.toaster.draw()
        self.refresh_buttons()
        self._dirty = True

    def _final(self, tid: int, key: str, word: str, f: dict) -> None:
        """v0.1.86.9: the accepted press's DONE (the toast turns final green) or FAILED / no answer (red, the button red)."""
        if self.closed:
            return
        ok, line, hint = verdict(f)
        self.toasts.resolve(tid, ok, line, hint)
        if key and not ok:
            self.presses.resolve(key, False)
        ms = (f or {}).get("ms") or {}
        self.cmd_results.insert(0, (time.strftime("%H:%M:%S"), word + (f" · {ms['final_ms']} ms" if ms.get("final_ms") is not None else ""),
                                    ok, list((f or {}).get("lines") or [line])))
        del self.cmd_results[30:]
        self.toaster.draw()
        self.refresh_buttons()
        self._dirty = True

    def post(self, path: str, body: dict, word: str, key: str = "", then: Optional[Callable[[dict], None]] = None,
             answer_s: Optional[float] = None) -> None:
        """/ack and /recording: the same toast + colour lifecycle as /cmd, through the same queue."""
        tid = self.toasts.add(word)
        if key:
            self.presses.press(key)
        self.toaster.draw()
        self.refresh_buttons()

        def done(r):
            if then and then(r):
                self.toasts.dismiss(tid)                          # `then` took over (a confirm, a resend)
                if key:
                    self.presses.s.pop(key, None)
            else:
                ok, line, hint = verdict(r)
                self.toasts.resolve(tid, ok, line, hint)
                if key:
                    self.presses.resolve(key, ok)
            self.toaster.draw()
            self.refresh_buttons()
            self._dirty = True
        self.worker.post(path, body, done, answer_s)

    def _dynamic(self, key: str) -> Optional[Btn]:
        if key == "usv-mission":
            code = self._picked_mission()
            cur = ((self.state or {}).get("ocs") or {}).get("mission") or {}       # v0.1.90.2: re-arming keeps the run's auto / lucky
            tail = (" --auto" if cur.get("auto") else "") + (" --lucky" if cur.get("lucky") and code.startswith("m1t") else "")
            mf = mods_flags(cur.get("mods")) if code.startswith("m1t") else ""      # v0.1.92: re-arming keeps the mission modifiers
            tail += (" " + mf) if mf else ""
            return Btn(f"mission {code}{tail}", "ARM MISSION", key=key) if code else None
        if key == "cmd-send":
            w = self.cmd_entry.get().strip()
            if w:
                self.cmd_entry.delete(0, "end")
            return Btn(w, w, key=key) if w else None
        if key in ("mis-arm", "mis-start"):
            code = self._mission_sel()
            if not code:
                return None
            vals = {f: v.get() for f, v in self.mis_vars.items()}
            line = mission_line("mission" if key == "mis-arm" else "start_task", code, self.mis_opts, vals, self.mis_auto.get())
            return Btn(line, "ARM" if key == "mis-arm" else "START", key=key)
        return None

    # ---- look ------------------------------------------------------------------------------------------------------------
    def _style(self) -> None:
        p = self.pal
        st = self.style = ttk.Style(self.root)
        try:
            st.theme_use("clam")                                   # the one built-in theme that honours colours on macOS
        except tk.TclError:
            pass
        self.root.configure(background=p["bg"])
        st.configure(".", background=p["bg"], foreground=p["text"], font=("Helvetica", 12))
        st.configure("TFrame", background=p["bg"])
        st.configure("Panel.TFrame", background=p["panel"])
        st.configure("TLabel", background=p["bg"], foreground=p["text"])
        st.configure("Dim.TLabel", background=p["panel"], foreground=p["dim"], font=("Helvetica", 11))
        st.configure("TNotebook", background=p["bg"], borderwidth=0)
        st.configure("TNotebook.Tab", background=p["panel2"], foreground=p["muted"], padding=(9, 5), font=("Helvetica", 11, "bold"))      # layout pass: 8 tabs in 440 px
        st.map("TNotebook.Tab", background=[("selected", p["panel"])], foreground=[("selected", p["text"])])
        st.configure("TCombobox", fieldbackground=p["panel"], background=p["panel2"], foreground=p["text"])
        st.configure("TCheckbutton", background=p["panel"], foreground=p["text"])
        st.configure("TRadiobutton", background=p["panel"], foreground=p["text"])
        for lvl in TH.LEVEL:
            fg, bg = TH.level_colours(p, lvl)
            st.configure(f"Chip{lvl}.TLabel", background=bg, foreground=fg, font=("Menlo", 13, "bold" if lvl in ("bad", "warn") else "normal"),
                         padding=(8, 4), relief="solid", borderwidth=1, bordercolor=p["line"])
            st.configure(f"Word{lvl}.TLabel", background=bg, foreground=fg, font=("Helvetica", 16, "bold"), padding=(10, 6))
        # buttons: the kinds (the page's --action / pale / danger / on) + greyed + the press looks (sending / ok / bad)
        looks = {"": (p["btnBg"], p["text"], p["btnLine"]), "act": (p["action"], p["actionText"], p["actionLine"]),
                 "go": (p["actionPale"], p["text"], p["actionLine"]), "danger": (p["alarmPale"], p["alarmText"], p["alarm"]),
                 "on": (p["accBg"], p["accText"], p["acc"]), "grey": (p["panel2"], p["dim"], p["line"]),
                 "sending": (p["warnBg"], p["warnText"], p["warn"]), "ok": (p["ok"], "#ffffff", p["ok"]), "bad": (p["alarm"], "#ffffff", p["alarm"])}
        looks["big"] = looks["act"]                                    # v0.1.90.8: the big ENABLE AUTO (two lines: HOLD → LOITER)
        for k, (bg, fg, line) in looks.items():
            name = f"B{k}.TButton"
            st.configure(name, background=bg, foreground=fg, bordercolor=line, lightcolor=bg, darkcolor=bg, focuscolor=line,
                         font=("Menlo", 20 if k == "big" else 13, "bold"), padding=(26, 18) if k == "big" else (12, 11), relief="solid",
                         borderwidth=3 if k == "big" else (2 if k in ("act", "sending", "ok", "bad") else 1), justify="center")
            st.map(name, background=[("pressed", p["warn"]), ("active", bg)], foreground=[("pressed", p["actionText"])])

    # ---- widgets (built once) --------------------------------------------------------------------------------------------
    def _label(self, parent, text="", bg="panel", fg="text", font=("Helvetica", 12), **kw) -> tk.Label:
        return tk.Label(parent, text=text, background=self.pal[bg], foreground=self.pal[fg], font=font, anchor="w", justify="left", **kw)

    def _button(self, parent, key: str, text: str = "…") -> ttk.Button:
        b = ttk.Button(parent, text=text, style="B.TButton", command=lambda k=key: self.press(k), takefocus=False)
        self.buttons[key] = b
        Tooltip(b, lambda k=key: self._btn_tip(k), self.pal)
        return b

    def _btn_tip(self, key: str) -> str:
        """A button's tip: the line it sends (what the CMD tab would type) and, greyed, the playbook's why + hint."""
        b = self._dynamic(key) if key in ("usv-mission", "mis-arm", "mis-start") else self.btn_model.get(key)
        if b is None:
            return ""
        out = [f"sends: {b.word}" if not key.startswith(("ack-", "cam-")) else b.label]
        if not b.ok and b.why:
            out.append(f"not now: {b.why}" + (f"\n  → {b.hint}" if b.hint else ""))
        if b.confirm:
            out.append("asks first")
        return "\n".join(out)

    def _build(self) -> None:
        p = self.pal
        self.root.geometry(default_geometry(self.root.winfo_screenwidth(), self.root.winfo_screenheight()))
        self.root.minsize(1200, 760)
        head = tk.Frame(self.root, background=p["panel"], padx=12, pady=8)
        head.pack(side="top", fill="x")
        self.chips: Dict[str, ttk.Label] = {}
        for k in HEADER_KEYS:
            lab = ttk.Label(head, text="…", style="Chipidle.TLabel")
            lab.pack(side="right" if k in ("clock", "link") else "left", padx=(0, 6))
            self.chips[k] = lab
        self._button(head, "log-hdr", "● START LOGGING").pack(side="left", padx=(2, 6))
        self._button(head, "big", "⛶ BIG VIEW").pack(side="left", padx=(0, 6))
        self._button(head, "settings", "⚙ SETTINGS").pack(side="left", padx=(0, 6))
        wrow = tk.Frame(self.root, background=p["panel"], padx=10)
        wrow.pack(side="top", fill="x")
        self.word = ttk.Label(wrow, text="waiting for the OCS…", style="Wordidle.TLabel")
        self.word.pack(side="left", pady=(0, 8))
        self.hint = self._label(wrow, fg="muted")
        self.hint.pack(side="left", padx=10)
        self._build_alarms()
        tk.Frame(self.root, height=1, background=p["line"]).pack(side="top", fill="x")
        body = tk.PanedWindow(self.root, orient="horizontal", background=p["line"], sashwidth=4, borderwidth=0)
        body.pack(side="top", fill="both", expand=True)
        self.body_pw = body
        self.big_fr = PN.build_big(self, self.root)
        self.big_on = False
        self.root.bind("<Escape>", lambda e: self._big(False) if self.big_on else None, add="+")
        self.left_sf = ScrollFrame(body, p["bg"])
        self.centre_sf = ScrollFrame(body, p["bg"])
        left, centre = self.left_sf.inner, self.centre_sf.inner
        left.configure(padx=12, pady=12)
        centre.configure(padx=12, pady=12)
        right = tk.Frame(body, background=p["bg"], padx=6, pady=6)
        body.add(self.left_sf.outer, minsize=330, width=380, stretch="never")       # layout pass: the run's column keeps its width,
        body.add(self.centre_sf.outer, minsize=380, width=470, stretch="middle")    # the tiles give way first,
        body.add(right, minsize=460, stretch="always")                              # the tabs never lose a tab
        self._build_ladder(left)
        self._build_tiles(centre)
        self._build_tabs(right)

    def _build_alarms(self) -> None:
        p = self.pal
        self.al = tk.Frame(self.root, background=p["panel2"], padx=10, pady=4)
        self.al.pack(side="top", fill="x")
        top = tk.Frame(self.al, background=p["panel2"])
        top.pack(fill="x")
        self.al_count = self._label(top, "ALARMS —", bg="panel2", font=("Menlo", 13, "bold"))
        self.al_count.pack(side="left")
        self._button(top, "ack-all", "ACK ALL").pack(side="right")
        self.al_rows = []
        for i in range(ALARM_SLOTS):
            fr = tk.Frame(self.al, background=p["panel2"])
            lab = self._label(fr, bg="panel2", font=("Menlo", 12))
            lab.pack(side="left", fill="x", expand=True)
            b = self._button(fr, f"ack-{i}", "ACK")
            b.pack(side="right", pady=1)
            self.al_rows.append((fr, lab))

    def _build_ladder(self, parent) -> None:
        p = self.pal
        self._label(parent, "RUN", bg="bg", fg="dim", font=("Helvetica", 11, "bold")).pack(anchor="w")
        self.lad_profile = self._label(parent, "", bg="bg", fg="acc", font=("Helvetica", 14, "bold"))   # v0.1.89: USV only | UAV only | UAV + USV
        self.lad_profile.pack(anchor="w", pady=(0, 2))
        self.steps = {}
        for n in range(1, LADDER_MAX + 1):
            fr = tk.Frame(parent, background=p["bg"], highlightthickness=1, highlightbackground=p["line2"], padx=12, pady=8)
            fr.pack(fill="x", pady=4)
            top = tk.Frame(fr, background=p["bg"])
            top.pack(fill="x")
            num = self._label(top, str(n), bg="bg", fg="dim", font=("Menlo", 14, "bold"), width=2)
            num.pack(side="left")
            chip = self._label(top, "—", bg="chipBg", fg="chipText", font=("Menlo", 12), padx=8)
            chip.pack(side="right")                                  # layout pass: the chip first, the title wraps around it
            title = self._label(top, "", bg="bg", font=("Helvetica", 13, "bold"), wraplength=230)
            title.pack(side="left", fill="x", expand=True)
            detail = self._label(fr, "", bg="bg", fg="muted", font=("Helvetica", 11), wraplength=340)
            warn = self._label(fr, "", bg="bg", fg="warnText", font=("Helvetica", 11, "bold"), wraplength=340)
            bts = tk.Frame(fr, background=p["bg"])
            why = self._label(fr, "", bg="bg", fg="alarmText", font=("Helvetica", 11), wraplength=340)
            for k in STEP_KEYS.get(n, []):
                self._button(bts, k)
            fr.pack_forget()                                         # v0.1.89: shown by _render_ladder (the profile decides how many)
            self.steps[n] = {"fr": fr, "top": top, "num": num, "title": title, "chip": chip, "detail": detail, "warn": warn,
                             "bts": bts, "why": why}
        # v0.1.92 (COWORK_HANDOFF addendum 6 §6.3): the "mission modifiers" subpanel - just before ENABLE AUTO, frozen from AUTO on
        mf = tk.Frame(parent, background=p["panel"], highlightthickness=1, highlightbackground=p["line2"], padx=12, pady=8)
        self._label(mf, "MISSION MODIFIERS — set before ENABLE AUTO", bg="panel", fg="dim", font=("Helvetica", 11, "bold")).pack(anchor="w")
        mline = self._label(mf, "", bg="panel", fg="text", font=("Menlo", 11), wraplength=340)
        mline.pack(fill="x", pady=(2, 4))
        mb = tk.Frame(mf, background=p["panel"])
        mb.pack(fill="x")
        self._button_grid(mb, MOD_KEYS_UI)
        mwhy = self._label(mf, "", bg="panel", fg="warnText", font=("Helvetica", 11), wraplength=340)
        mwhy.pack(fill="x", pady=(4, 0))
        self.mods_w = {"fr": mf, "line": mline, "why": mwhy}

    # ---- v0.1.94.3 (Dmitrii 14 Nov: "a velocity modifier on the OCS, which we can click or enter in the box and send ... speed up during
    # the trial on the spot"): cruise / circle / gate x0.5-2.0, - / + in 0.25 steps or a typed value + Enter; each press = one `speed` intent,
    # in force at once on the boat (any state). The boxes show the boat's own values (status tuning.speed_k) unless being typed in.
    SPEED_TIPS = {"cruise": "cruise: every path leg - start, run-ins to the gates, transit, scout, exit",
                  "circle": "circle: the entry / exit orbits - forward speed and yaw rate together (the radius holds)",
                  "gate": "gate: T1 THROUGH - the pass between the gate buoys"}

    def _build_speed(self, parent) -> None:
        from ...companion import speed_k as SK
        p = self.pal
        fr = tk.Frame(parent, background=p["panel"])
        fr.pack(fill="x", pady=(2, 6))
        self._label(fr, "SPEED x (live, at once)", fg="dim", font=("Helvetica", 10, "bold")).grid(row=0, column=0, columnspan=4, sticky="w")
        self.speed_w = {}
        for i, c in enumerate(SK.CLASSES):
            lab = self._label(fr, c, fg="dim", font=("Helvetica", 11), width=7)
            lab.grid(row=1 + i, column=0, sticky="w")
            Tooltip(lab, lambda c=c: self.SPEED_TIPS[c], self.pal)
            m = ttk.Button(fr, text="−", width=3, style="B.TButton", takefocus=False, command=lambda c=c: self._speed_step(c, -SK.K_STEP))
            m.grid(row=1 + i, column=1, padx=(0, 2), pady=1)
            v = tk.StringVar(value="1.00")
            e = tk.Entry(fr, textvariable=v, width=6, font=("Menlo", 12), justify="center", background=p["panel2"], foreground=p["text"],
                         insertbackground=p["text"], relief="flat")
            e.grid(row=1 + i, column=2, padx=2, pady=1)
            e.bind("<Return>", lambda _e, c=c: self._speed_typed(c))
            e.bind("<KP_Enter>", lambda _e, c=c: self._speed_typed(c))
            Tooltip(e, lambda: f"type x{SK.K_MIN:g}-{SK.K_MAX:g}, Enter sends it", self.pal)
            pl = ttk.Button(fr, text="+", width=3, style="B.TButton", takefocus=False, command=lambda c=c: self._speed_step(c, SK.K_STEP))
            pl.grid(row=1 + i, column=3, padx=(2, 0), pady=1)
            self.speed_w[c] = (v, e)
        r = ttk.Button(fr, text="x1 all", style="B.TButton", takefocus=False, command=lambda: self.send("speed reset", key=""))
        r.grid(row=1, column=4, rowspan=3, padx=(8, 0), sticky="ns")
        Tooltip(r, lambda: "sends: speed reset (all three back to x1)", self.pal)
        self.speed_note = self._label(fr, "", fg="dim", font=("Helvetica", 10), wraplength=420)
        self.speed_note.grid(row=4, column=0, columnspan=5, sticky="w")

    def _speed_live(self) -> Dict[str, float]:
        s = self.state or {}
        t = ((s.get("local") or {}).get("tuning")) or ((s.get("status") or {}).get("tuning")) or {}
        return dict(t.get("speed_k") or {})

    def _speed_step(self, c: str, d: float) -> None:
        from ...companion import speed_k as SK
        live = self._speed_live()
        try:
            cur = float(live.get(c, self.speed_w[c][0].get()))
        except ValueError:
            cur = 1.0
        k = min(SK.K_MAX, max(SK.K_MIN, round((cur + d) / SK.K_STEP) * SK.K_STEP))
        if k == cur:
            self.toasts.event("info", f"speed {c}: x{k:g} is the {'top' if d > 0 else 'bottom'} ({SK.K_MIN:g}-{SK.K_MAX:g})")
            self.toaster.draw()
            return
        self.speed_w[c][0].set(f"{k:.2f}")
        self.send(f"speed {c} {k:g}", key="")

    def _speed_typed(self, c: str) -> None:
        from ...companion import speed_k as SK
        v, _e = self.speed_w[c]
        try:
            k = round(float(v.get().strip().lstrip("xX")), 3)
        except ValueError:
            self.toasts.event("bad", f"speed {c}: {v.get()!r} is not a number")
            self.toaster.draw()
            return
        if not (SK.K_MIN <= k <= SK.K_MAX):
            self.toasts.event("bad", f"speed {c}: x{k:g} outside {SK.K_MIN:g}-{SK.K_MAX:g}")
            self.toaster.draw()
            return
        self.root.focus_set()
        self.send(f"speed {c} {k:g}", key="")

    def _render_speed(self) -> None:
        if not getattr(self, "speed_w", None):
            return
        live = self._speed_live()
        focus = self.root.focus_get()
        for c, (v, e) in self.speed_w.items():
            if c in live and focus is not e:
                txt = f"{float(live[c]):.2f}"
                if v.get() != txt:
                    v.set(txt)
        note = ("boat: " + " · ".join(f"{c} x{float(live.get(c, 1.0)):g}" for c in self.speed_w)) if live else "boat: no speed knobs reported yet (an older mission manager?)"
        self.put("speed:note", self.speed_note, text=note)

    def _button_grid(self, row: tk.Frame, keys, first_col: int = 0, per_line: int = 2, first_row: int = 0) -> None:
        """Layout pass 2026-11-12: a tile's buttons two per line, equal widths - every button fully visible at 1280 px too (a row of four
        ran past the column's edge: ABORT / DISARM cut)."""
        for i, k in enumerate(keys):
            r, c = divmod(i, per_line)
            self._button(row, k).grid(row=first_row + r, column=first_col + c, sticky="ew", padx=(0, 4), pady=(0, 4))
        for c in range(per_line):
            row.columnconfigure(first_col + c, weight=1, uniform="btn" + str(id(row)))

    def _tile(self, parent, name: str) -> tk.Frame:
        p = self.pal
        fr = tk.Frame(parent, background=p["panel"], highlightthickness=1, highlightbackground=p["line"], padx=12, pady=8)
        fr.pack(fill="x", pady=(0, 10))
        self._label(fr, name, font=("Helvetica", 14, "bold")).pack(anchor="w")
        return fr

    def _kv(self, parent, keys) -> Dict[str, tk.Label]:
        g = tk.Frame(parent, background=self.pal["panel"])
        g.pack(fill="x", pady=(2, 6))
        out = {}
        for i, (k, lab) in enumerate(keys):
            self._label(g, lab, fg="dim", font=("Helvetica", 11)).grid(row=i, column=0, sticky="w", padx=(0, 10))
            v = self._label(g, "—", font=("Menlo", 12), wraplength=360)
            v.grid(row=i, column=1, sticky="w")
            out[k] = v
        return out

    def _build_tiles(self, parent) -> None:
        p = self.pal
        usv = self._tile(parent, "USV1")
        self.usv_kv = self._kv(usv, [("state", "state"), ("task", "task"), ("meaning", "stage"), ("station_keep", "station keep"),
                                     ("flow", "RoboCommand"), ("sog", "speed"), ("hdg", "heading")])
        mrow = tk.Frame(usv, background=p["panel"])
        mrow.pack(fill="x", pady=(0, 6))
        self._label(mrow, "mission", fg="dim", font=("Helvetica", 11)).pack(side="left", padx=(0, 8))
        self.mpick = ttk.Combobox(mrow, state="readonly", width=34, font=("Menlo", 12))
        self.mpick.pack(side="left", fill="x", expand=True)
        self._button(mrow, "usv-mission", "ARM MISSION").pack(side="left", padx=(6, 0))
        self._build_speed(usv)                                       # v0.1.94.3 (Dmitrii 14 Nov): the live speed knobs
        self.usv_rows = {}
        for label, keys in USV_ROWS:
            row = tk.Frame(usv, background=p["panel"])
            row.pack(fill="x", pady=(4, 0))
            if label:                                                # layout pass: the group's name above its buttons
                self._label(row, label.upper(), fg="dim", font=("Helvetica", 10, "bold")).grid(row=0, column=0, columnspan=2, sticky="w")
            self._button_grid(row, keys, first_col=0, first_row=1)
            why = self._label(usv, "", fg="alarmText", font=("Helvetica", 11), wraplength=460)
            self.usv_rows[label or "stop"] = (row, why, keys)
        uav = self._tile(parent, "UAV")
        self.uav_kv = self._kv(uav, [("ident", "identity"), ("heartbeat", "RoboCommand"), ("state", "declaration"), ("pilot", "pilot")])
        row = tk.Frame(uav, background=p["panel"])
        row.pack(fill="x", pady=(4, 0))
        self._button_grid(row, UAV_ROW, first_col=0)
        row2 = tk.Frame(uav, background=p["panel"])
        row2.pack(fill="x", pady=(4, 2))
        self._button(row2, "uav-run", "▶ UAV RUN (boat holds)").pack(side="left")
        self.uav_why = self._label(uav, "", fg="alarmText", font=("Helvetica", 11), wraplength=460)
        self.uav_why.pack(fill="x")

    def _build_tabs(self, parent) -> None:
        p = self.pal
        self.nb = ttk.Notebook(parent)
        self.nb.pack(fill="both", expand=True)
        self.tabs = {}
        for name in ("LOG", "MAP", "VEHICLES", "JUDGE", "HOSTS", "CMD", "MISSION", "CAMERA"):
            fr = tk.Frame(self.nb, background=p["panel"], padx=8, pady=8)
            self.nb.add(fr, text=name)
            self.tabs[name] = fr
        self.nb.bind("<<NotebookTabChanged>>", lambda e: self._tab_changed())
        self._build_log(self.tabs["LOG"])
        self._build_map(self.tabs["MAP"])
        self._build_cmd(self.tabs["CMD"])
        self._build_mission(self.tabs["MISSION"])
        self._build_camera(self.tabs["CAMERA"])
        self._build_vehicles(self.tabs["VEHICLES"])
        self.hosts_body, self.hosts_why = None, ""
        self.hosts = HostsPanel(self.tabs["HOSTS"], self.pal, self._label)
        PN.build_judge(self, self.tabs["JUDGE"])
        self._build_settings_window()

    def _build_log(self, fr) -> None:
        p = self.pal
        bar = tk.Frame(fr, background=p["panel"])
        bar.pack(fill="x")
        self.log_filter = tk.StringVar(value="all")
        for f in ("all", "ui", "judge", "usv", "uav", "cautions"):
            ttk.Radiobutton(bar, text=f, value=f, variable=self.log_filter, command=self._log_rebuild).pack(side="left", padx=(0, 6))
        self.log_pause = self._label(bar, "", fg="warnText", font=("Helvetica", 11, "bold"))
        self.log_pause.pack(side="right")
        box = tk.Frame(fr, background=p["panel"])
        box.pack(fill="both", expand=True, pady=(6, 0))
        self.log = tk.Text(box, font=("Menlo", 12), background=p["panel"], foreground=p["text"], wrap="word", borderwidth=0,
                           highlightthickness=0, state="disabled")
        sb = ttk.Scrollbar(box, command=self.log.yview)
        self.log.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.log.pack(side="left", fill="both", expand=True)
        for tag, tok in LOG_TAGS.items():
            self.log.tag_configure(tag, foreground=p[tok])
        self.log.tag_configure("t", foreground=p["dim"])
        self.log.bind("<End>", lambda e: self.log.see("end"))
        self._log_last = None
        self._log_n = 0

    def _build_map(self, fr) -> None:
        """K6: the Canvas map; the fence / course editing stays on the web map ↗ (view buttons: not commands, no toast)."""
        bar = tk.Frame(fr, background=self.pal["panel"])
        bar.pack(fill="x")
        ttk.Button(bar, text="fit", style="B.TButton", takefocus=False, command=lambda: self.cmap.fit()).pack(side="left")
        ttk.Button(bar, text="fit all", style="B.TButton", takefocus=False, command=lambda: self.cmap.fit(True)).pack(side="left", padx=4)
        self.follow_btn = ttk.Button(bar, text="F follow", style="B.TButton", takefocus=False, command=lambda: self.cmap.toggle_follow())
        self.follow_btn.pack(side="left")
        ttk.Button(bar, text="web map ↗", style="B.TButton", takefocus=False,
                   command=lambda: webbrowser.open(self.ocs + "/map.html")).pack(side="right")
        self.cmap = CanvasMap(fr, self.pal)
        self.cmap.c.pack(fill="both", expand=True, pady=(6, 0))

    def _build_cmd(self, fr) -> None:
        p = self.pal
        row = tk.Frame(fr, background=p["panel"])
        row.pack(fill="x")
        self.cmd_entry = tk.Entry(row, font=("Menlo", 14), background=p["bg"], foreground=p["text"], insertbackground=p["text"],
                                  relief="solid", borderwidth=1)
        self.cmd_entry.pack(side="left", fill="x", expand=True, ipady=8)
        self.cmd_entry.bind("<Return>", lambda e: self.press("cmd-send"))
        self._button(row, "cmd-send", "SEND").pack(side="left", padx=(6, 0))
        self.cmd_use = self._label(fr, "any op-tui word; pick one below (hover = its modifiers and when it is allowed) - the same path "
                                   "as the buttons", fg="dim", font=("Menlo", 11), wraplength=560)
        self.cmd_use.pack(anchor="w", fill="x", pady=(4, 6))
        split = tk.PanedWindow(fr, orient="vertical", background=p["line"], sashwidth=4, borderwidth=0)
        split.pack(fill="both", expand=True)
        tf = tk.Frame(split, background=p["panel"])
        self.cmd_tree = ttk.Treeview(tf, columns=("does",), show="tree headings", selectmode="browse", height=14)
        self.cmd_tree.heading("#0", text="command")
        self.cmd_tree.heading("does", text="what it does")
        self.cmd_tree.column("#0", width=170, stretch=False)
        self.cmd_tree.column("does", width=330)
        self.cmd_tree.tag_configure("no", foreground=p["dim"])
        self.cmd_tree.tag_configure("group", foreground=p["accText"])
        sb = ttk.Scrollbar(tf, command=self.cmd_tree.yview)
        self.cmd_tree.configure(yscrollcommand=sb.set)
        sb.pack(side="right", fill="y")
        self.cmd_tree.pack(side="left", fill="both", expand=True)
        self.cmd_tree.bind("<<TreeviewSelect>>", lambda e: self._cmd_pick())
        self.cmd_tree.bind("<Double-1>", lambda e: (self._cmd_pick(), self.cmd_entry.focus_set()))
        Tooltip(self.cmd_tree, self._cmd_tip, self.pal, tree=True)
        self.cmd_info: Dict[str, object] = {}
        self._cmd_built = False
        split.add(tf, minsize=160, height=330)
        self.cmd_out = tk.Text(split, font=("Menlo", 12), background=p["panel"], foreground=p["text"], borderwidth=0, highlightthickness=0,
                               state="disabled", wrap="word")
        split.add(self.cmd_out, minsize=80)
        self.cmd_out.tag_configure("bad", foreground=p["alarm"])
        self.cmd_out.tag_configure("ok", foreground=p["ok"])
        self.cmd_out.tag_configure("dim", foreground=p["dim"])

    def _build_mission(self, fr) -> None:
        p = self.pal
        top = tk.Frame(fr, background=p["panel"])
        top.pack(fill="x")
        self.mis_list = tk.Listbox(top, height=8, font=("Menlo", 12), exportselection=False, background=p["bg"], foreground=p["text"],
                                   selectbackground=p["acc"], selectforeground="#ffffff", borderwidth=1, relief="solid")
        self.mis_list.pack(fill="x")
        self.mis_list.bind("<<ListboxSelect>>", lambda e: self._mission_form())
        self.mis_info = self._label(fr, "", fg="muted", font=("Helvetica", 11), wraplength=500)
        self.mis_info.pack(fill="x", pady=(6, 4))
        self.mis_form = tk.Frame(fr, background=p["panel"])
        self.mis_form.pack(fill="x")
        self.mis_vars: Dict[str, tk.Variable] = {}
        self.mis_opts: List[dict] = []
        self.mis_auto = tk.BooleanVar(value=False)
        brow = tk.Frame(fr, background=p["panel"])
        brow.pack(fill="x", pady=6)
        ttk.Checkbutton(brow, text="--auto (start at RunStart)", variable=self.mis_auto, command=self._mission_preview).pack(side="left")
        self._button(brow, "mis-start", "▶ START NOW").pack(side="right")
        self._button(brow, "mis-arm", "ARM").pack(side="right", padx=(0, 6))
        self.mis_line = self._label(fr, "", fg="accText", font=("Menlo", 12), wraplength=500)
        self.mis_line.pack(fill="x")
        self._label(fr, "STAGES", fg="dim", font=("Helvetica", 11, "bold")).pack(anchor="w", pady=(8, 0))
        self.mis_stages = tk.Text(fr, height=9, font=("Menlo", 12), background=p["panel"], foreground=p["text"], borderwidth=0,
                                  highlightthickness=0, state="disabled", wrap="word")
        self.mis_stages.pack(fill="both", expand=True)
        self.mis_stages.tag_configure("now", background=p["action"], foreground=p["actionText"])
        self.mis_stages.tag_configure("dim", foreground=p["dim"])
        self.mis_metrics = self._label(fr, "", fg="muted", font=("Menlo", 11), wraplength=500)
        self.mis_metrics.pack(fill="x")
        self._mis_code = None

    def _build_camera(self, fr) -> None:
        p = self.pal
        lrow = tk.Frame(fr, background=p["panel"])
        lrow.pack(fill="x", pady=(0, 8))
        self._button(lrow, "log-cam", "● START LOGGING").pack(side="left")
        self.log_sess = self._label(lrow, "", font=("Menlo", 12), wraplength=380)
        self.log_sess.pack(side="left", padx=10, fill="x", expand=True)
        self.clock_line = self._label(fr, "clocks: —", font=("Menlo", 11), wraplength=560)
        self.clock_line.pack(fill="x", pady=(0, 8))
        row = tk.Frame(fr, background=p["panel"])
        row.pack(fill="x")
        self._button(row, "cam-rec", "⏺ RECORD").pack(side="left")
        self.rec_depth = tk.BooleanVar(value=True)
        ttk.Checkbutton(row, text="+ depth (RGB + depth pairs)", variable=self.rec_depth).pack(side="left", padx=(8, 0))
        self.rec_text = self._label(row, "recording: —", font=("Menlo", 12), wraplength=330)
        self.rec_text.pack(side="left", padx=10, fill="x", expand=True)
        self.rec_where = self._label(fr, "", fg="muted", font=("Menlo", 11), wraplength=560)
        self.rec_where.pack(fill="x", pady=(6, 0))
        self.cam_text = self._label(fr, "the stream starts when this tab opens", fg="dim", font=("Helvetica", 11), wraplength=500)
        self.cam_text.pack(fill="x", pady=(6, 4))
        self.cam_img = tk.Label(fr, background=p["mapBg"], text="no picture", foreground=p["dim"])
        self.cam_img.pack(fill="both", expand=True)

    # ---- missions --------------------------------------------------------------------------------------------------------
    def _fill_missions(self, v: View) -> None:
        """The picker and the MISSION list - refilled only when the list changes (the sim suffix needs the first state)."""
        codes = [c for c, _ in v.missions]
        if codes == getattr(self, "_mission_codes", None):
            return
        self._mission_codes = codes
        self._cache.pop("armed", None)
        self._mis_code = None
        self.mpick.configure(values=[lab for _, lab in v.missions])
        self.mis_list.delete(0, "end")
        for _, lab in v.missions:
            self.mis_list.insert("end", lab)
        armed = v.mission_armed or ""
        idx = self._mission_codes.index(armed) if armed in self._mission_codes else 0
        if self._mission_codes:
            self.mpick.current(idx)
            self.mis_list.selection_set(idx)
            self._mission_form()

    def _picked_mission(self) -> str:
        i = self.mpick.current()
        codes = getattr(self, "_mission_codes", [])
        return codes[i] if 0 <= i < len(codes) else ""

    def _mission_sel(self) -> str:
        sel = self.mis_list.curselection()
        codes = getattr(self, "_mission_codes", [])
        return codes[sel[0]] if sel and sel[0] < len(codes) else ""

    def _mission_meta(self, code: str) -> dict:
        base = code[:-3] if code.endswith("sim") else code
        return next((m for m in (self.model.meta or {}).get("missions") or [] if m.get("code") == base), {})

    def _mission_form(self) -> None:
        """Rebuilt only when the operator picks another mission (never on a poll)."""
        code = self._mission_sel()
        if code == self._mis_code:
            return
        self._mis_code = code
        m = self._mission_meta(code)
        for w in self.mis_form.winfo_children():
            w.destroy()
        self.mis_vars, self.mis_opts = {}, list(m.get("options") or [])
        self.put("mis-info", self.mis_info, text=f"{m.get('title', '')}\ntier {m.get('tier', '?')} · starts: {m.get('starts', '')}\n"
                                                 f"ends: {m.get('ends', '')}" + (f"\n{m['note']}" if m.get("note") else ""))
        for i, o in enumerate(self.mis_opts):
            self._label(self.mis_form, o.get("label", o.get("flag")), fg="dim", font=("Helvetica", 11)).grid(row=i, column=0, sticky="w", padx=(0, 8))
            if o.get("check"):
                var = tk.BooleanVar(value=False)
                ttk.Checkbutton(self.mis_form, variable=var, command=self._mission_preview, text=o.get("help", "")[:60]).grid(row=i, column=1, sticky="w")
            elif o.get("choices"):
                var = tk.StringVar(value="")
                cb = ttk.Combobox(self.mis_form, state="readonly", values=o["choices"], textvariable=var, width=18)
                cb.grid(row=i, column=1, sticky="w")
                cb.bind("<<ComboboxSelected>>", lambda e: self._mission_preview())
            else:
                var = tk.StringVar(value="")
                e = tk.Entry(self.mis_form, textvariable=var, width=20, font=("Menlo", 12))
                e.grid(row=i, column=1, sticky="w")
                e.bind("<KeyRelease>", lambda ev: self._mission_preview())
            self.mis_vars[o["flag"]] = var
        self._mission_preview()
        self._dirty = True

    def _mission_preview(self) -> None:
        b = self._dynamic("mis-arm")
        self.put("mis-line", self.mis_line, text=("> " + b.word) if b else "")

    # ---- camera ----------------------------------------------------------------------------------------------------------
    def _tab_changed(self) -> None:
        try:
            name = self.nb.tab(self.nb.select(), "text")
        except tk.TclError:
            return
        if name in ("MAP", "VEHICLES"):
            self._map_sig = None
            self._dirty = True
        if name == "HOSTS":
            self.poller.hosts_on.set()
            self.hosts.render(self.hosts_body, self.hosts_why or "waiting for the first answer", self._link_est())
        else:
            self.poller.hosts_on.clear()
        on = name == "CAMERA"
        self.cam.show(on)
        if not on:
            self._cam_started = False

    def _on_cam(self, c: dict) -> None:
        self.cam_state = c or {}
        if c and not c.get("ok") and not self._cam_started and self.cam.active.is_set():
            self._cam_started = True                              # B's contract: POST /system cam start once, then keep asking /cam.json
            self.post("/system", {"action": "cam", "options": {"do": "start", "source": "auto"}}, "camera: start the stream", "")
        self._dirty = True

    def _on_rec(self, r: dict) -> None:
        self.rec_state = r or {}
        self._dirty = True

    def _on_frame(self, jpg) -> None:
        """A frame from the CamWorker: a PIL image already decoded + scaled there (v0.1.86.9), or JPEG bytes (no Pillow / a test)."""
        if not self.cam.active.is_set():
            return
        try:
            from PIL import Image, ImageTk
        except ImportError:
            self.put("cam-text", self.cam_text, text="the picture needs Pillow in the venv (requirements-sim.txt)")
            return
        w, h = max(160, self.cam_img.winfo_width()), max(120, self.cam_img.winfo_height())
        self.cam.target = (w, h)                                     # the worker scales the next frames to this
        im = Image.open(io.BytesIO(jpg)) if isinstance(jpg, (bytes, bytearray)) else jpg
        if im.size[0] > w or im.size[1] > h:
            im.thumbnail((w, h))
        self._photo = ImageTk.PhotoImage(im)
        self.cam_img.configure(image=self._photo, text="")

    def _record(self) -> None:
        on = bool(self.rec_state.get("recording"))
        op = "stop" if on else "start"
        depth = bool(self.rec_depth.get())                    # 2026-11-11: a depth image beside its RGB frame (the Mac frames)

        def answer(r, op=op):
            if r and not r.get("ok") and (r.get("already") or r.get("needs")):
                conf = "new" if r.get("already") else r.get("needs")
                text = (f"Already recording: {r.get('text', '')}. Start a new clip?" if r.get("already") else str(r.get("why") or conf))
                Confirm(self, text, {"new": "YES, NEW CLIP", "restart": "YES, RESTART", "stop": "YES, STOP"}.get(conf, "YES"),
                        lambda: self.post("/recording", {"op": op, "confirm": conf, "depth": depth}, f"recording {op} ({conf})", "cam-rec",
                                          lambda rr: (self._on_rec(rr), self._depth_toast(rr), False)[2], answer_s=REC_ANSWER_S))
                return True
            self._on_rec(r or {})
            self._depth_toast(r)
            return False
        self.post("/recording", {"op": op, "depth": depth}, f"recording {op}" + (" + depth" if depth and op == "start" else ""), "cam-rec", answer,
                  answer_s=REC_ANSWER_S)

    # ---- the ⚙ SETTINGS window (SETTINGS · SYSTEM · HELP; the low-frequency tabs out of the run's column) ----------------------
    def _build_settings_window(self) -> None:
        p = self.pal
        w = self.set_win = tk.Toplevel(self.root, background=p["bg"])
        w.title("⚙ OCS console settings")
        w.geometry("760x820")
        w.withdraw()
        w.protocol("WM_DELETE_WINDOW", lambda: self._settings(False))
        w.bind("<Escape>", lambda e: self._settings(False))
        self.set_nb = ttk.Notebook(w)
        self.set_nb.pack(fill="both", expand=True, padx=8, pady=8)
        self.set_tabs = {}
        for name in ("SETTINGS", "SYSTEM", "HELP"):
            fr = tk.Frame(self.set_nb, background=p["panel"], padx=12, pady=12)
            self.set_nb.add(fr, text=name)
            self.set_tabs[name] = fr
        PN.build_settings(self, self.set_tabs["SETTINGS"])
        PN.build_system(self, self.set_tabs["SYSTEM"])
        PN.build_help(self, self.set_tabs["HELP"])
        self.set_on = False
        self._apply_prefs(start=True)

    def _settings(self, on: bool) -> None:
        self.set_on = on
        if on:
            self.set_win.deiconify()
            self.set_win.lift()
            self._dirty = True
        else:
            self.set_win.withdraw()
        self.btn_model["settings"] = Btn("settings", "⚙ SETTINGS", kind="on" if on else "", key="settings")
        self.refresh_buttons()

    def _apply_prefs(self, start: bool = False) -> None:
        pr = self.prefs
        if self.log_filter is not None and self.log_filter.get() != pr.get("log_filter"):
            self.log_filter.set(pr.get("log_filter", "all"))
            self._log_rebuild()
        if self.rec_depth is not None:
            self.rec_depth.set(pr.get("rec_depth", "on") == "on")
        if start and pr.get("start_tab") in self.tabs:
            self.nb.select(self.tabs[pr["start_tab"]])
        for k, var in (getattr(self, "pref_vars", None) or {}).items():
            if var.get() != pr.get(k):
                var.set(pr.get(k))

    def _pref_changed(self, k: str) -> None:
        self.prefs[k] = self.pref_vars[k].get()
        try:
            PN.save_prefs(self.prefs)
        except OSError as exc:
            self.toasts.event("bad", f"✗ preferences not saved: {exc}")
        self._apply_prefs()
        self.toasts.event("ok", f"✓ {k.replace('_', ' ')} = {self.prefs[k]}" + (" (at the next start)" if k in ("theme", "fast_hz") else ""))
        self.toaster.draw()

    def _cfg_selected(self):
        sel = self.cfg_list.curselection()
        items = PN.list_configs()
        return items[sel[0]] if sel and sel[0] < len(items) else None

    def _config_action(self, key: str) -> None:
        if key == "cfg-save":
            name = self.cfg_name.get().strip()
            why = PN.check_name(name)
            if why:
                self.toasts.event("bad", f"✗ config name: {why}")
                self.toaster.draw()
                return
            exists = name in [c["name"] for c in PN.list_configs()]

            def save():
                from ... import __version__
                path = PN.save_config(PN.make_config(name, self.state, self.prefs, __version__))
                n = len(PN.knobs(self.state))
                self.toasts.event("ok", f"✓ config {name} saved ({n} knobs + the preferences) → {path.split('/logs/')[-1]}")
                PN.render_configs(self)
                self.toaster.draw()
            if exists:
                Confirm(self, f"A config named {name} exists. Overwrite it?", "YES, OVERWRITE", save)
            else:
                save()
            return
        item = self._cfg_selected()
        if item is None:
            self.toasts.event("warn", "pick a config in the list first")
            self.toaster.draw()
            return
        if key == "cfg-delete":
            Confirm(self, f"Delete the config {item['name']} (saved {item['saved']})? It is only on this Mac.", "YES, DELETE",
                    lambda: (PN.delete_config(item["name"]), PN.render_configs(self), self.toasts.event("ok", f"✓ config {item['name']} deleted"),
                             self.toaster.draw()))
            return
        words, notes = PN.restore_plan(item["cfg"], self.state)
        prefs = dict(item["cfg"].get("prefs") or {})

        def restore():
            for k, v in prefs.items():
                if k in PN.PREF_KEYS and v in PN.PREF_KEYS[k][1]:
                    self.prefs[k] = v
            PN.save_prefs(self.prefs)
            self._apply_prefs()
            if words:
                self._chain(words, "cfg-restore", f"✓ config {item['name']} restored ({len(words)} knobs + the preferences)",
                            "✗ RESTORE stopped at '{w}' - the knobs before it are set; fix it and RESTORE again (it sends only what differs)")
            else:
                self.toasts.event("ok", f"✓ config {item['name']}: the knobs already match; the preferences applied")
                self.toaster.draw()
        Confirm(self, f"RESTORE {item['name']} (saved {item['saved']})?\n" + ("sends: " + "; ".join(words) if words else "the knobs already match")
                + ("\n" + "; ".join(notes) if notes else "") + "\nthe preferences: " + ", ".join(f"{k} {v}" for k, v in prefs.items()),
                "YES, RESTORE", restore)

    def _tune_set(self, kkey: str) -> None:
        k = next((x for x in PN.knobs(self.state) if x.key == kkey), None)
        r = self.tune_rows.get(kkey)
        if k is None or r is None:
            return
        word, why = PN.tune_word(k, r["ent"].get())
        if not why:
            why = PN.pair_rule(kkey, float(word.split()[-1]), {x.key: x.value for x in PN.knobs(self.state)})
        if why:
            self.toasts.event("bad", "✗ " + why)
            self.toaster.draw()
            return
        r["ent"].delete(0, "end")
        self.send(word, key="tune:" + kkey)

    def _chain(self, words, key: str, done_text: str, fail_fmt: str) -> None:
        """Send words one after another through the one queue, each toasted; stop at the first ✗ (UAV RUN, RESTORE)."""
        def step(rest):
            if not rest:
                self.toasts.event("ok", done_text)
                self.presses.resolve(key, True)
                self.toaster.draw()
                return
            w = rest[0]
            tid = self.toasts.add(w)
            self.sent_lines.append(w)

            def got(r, w=w, tid=tid):
                ok, line, hint = verdict(r)
                self.toasts.resolve(tid, ok, line, hint)
                self.cmd_results.insert(0, (time.strftime("%H:%M:%S"), w, ok, list((r or {}).get("lines") or [line])))
                self._dirty = True
                if not ok:
                    self.presses.resolve(key, False)
                    self.toasts.event("bad", fail_fmt.format(w=w))
                    self.toaster.draw()
                    return
                self.root.after(1500 if w in ("enable_auto", "report on usv", "report on uav") else 300, lambda: step(rest[1:]))
            self.worker.send(w, got)
            self.toaster.draw()
        self.presses.press(key)
        step(list(words))

    def _big(self, on: bool) -> None:
        """⛶ BIG VIEW: the big-numeral run view over the body (readable from 2 m); Esc or the button = back."""
        self.big_on = on
        if on:
            self.body_pw.pack_forget()
            self.big_fr.pack(side="top", fill="both", expand=True)
            PN.render_big(self)
        else:
            self.big_fr.pack_forget()
            self.body_pw.pack(side="top", fill="both", expand=True)
        self.btn_model["big"] = Btn("big view", "⛶ CONSOLE" if on else "⛶ BIG VIEW", kind="on" if on else "", key="big")
        self.refresh_buttons()

    def _system_action(self, aid: str) -> None:
        """SYSTEM: the page's rule - every action asks; the dangerous ones need their word typed; POST /system through the queue."""
        a = next((x for x in PN.system_actions(self.model.meta, self.state) if x.id == aid), None)
        if a is None:
            return
        values = {k: v.get() for k, v in (self.sys_vars.get(aid) or {}).items()}
        text = f"{a.title}\n{a.what}" + (f"\nnot now: {a.refused}\n(the backend will refuse it)" if a.refused else "") + \
               ("\noptions: " + ", ".join(f"{k} {v}" for k, v in values.items() if v) if values else "")

        def go(typed=""):
            self.post("/system", PN.system_body(a, values, typed), f"system {a.id}", "sys:" + aid, answer_s=30.0)
        if a.confirm:
            c = Confirm(self, text, "YES, RUN", lambda: go(a.confirm), typed=a.confirm)
        else:
            Confirm(self, text, "YES, RUN", go)

    def _uav_run(self) -> None:
        """▶ UAV RUN (boat holds): the remaining words of model.UAV_RUN_WORDS, one after the other through the one queue, each toasted;
        stops at the first ✗. Never ARM, never handover."""
        words, why = uav_run_plan(self.state)
        if why:
            self.toasts.event("bad", "✗ UAV RUN — " + why)
            self.toaster.draw()
            return
        if not words:
            self.toasts.event("ok", "✓ UAV RUN — everything is done: wait for RunStart (the boat holds in AUTO)")
            self.toaster.draw()
            return

        Confirm(self, "UAV RUN with the boat holding in AUTO - sends, in order:\n  " + "\n  ".join(words) +
                "\n(stops at the first refusal; never ARM, never handover)", "YES, UAV RUN",
                lambda: self._chain(words, "uav-run", "✓ UAV RUN sent - declared with the boat holding: wait for RunStart, the UAV team flies",
                                    "✗ UAV RUN stopped at '{w}' - fix it, press UAV RUN again (it skips what is done)"))

    def _log_session(self, key: str, confirm: str = "") -> None:
        """One press = every log (B's POST /logging: usv_ui's FCU log + the AGX video + the Mac frames, one label, one t0); the clocks
        are checked first. Answers: already → new session? · needs clock → start anyway? · needs stop → stop? · 404 → not there yet."""
        lv = logging_view(self.state)
        op = "stop" if lv.running else "start"
        if op == "start" and lv.clock_problem and not confirm:
            Confirm(self, lv.clock_problem, "YES, START ANYWAY", lambda: self._log_session(key, "clock"))
            return
        body = {"op": op, "label": "log", "depth": bool(self.rec_depth.get()) if self.rec_depth is not None else True}
        if confirm:
            body["confirm"] = confirm

        def answer(r):
            r = r or {}
            if str(r.get("error") or "").startswith("HTTP 404"):
                self.toasts.event("bad", "✗ logging: this OCS has no POST /logging yet (session B's backend) - nothing started")
                return True
            asks = {"already": ("new", f"Logging already runs: {r.get('why') or r.get('label') or ''}. Start a new session?", "YES, NEW SESSION"),
                    "clock": ("clock", str(r.get("why") or "the clocks differ") + " - start anyway?", "YES, START ANYWAY"),
                    "restart": ("restart", str(r.get("why") or "the observer restarts to record") + " - restart it now?", "YES, RESTART"),
                    "stop": ("stop", "Stop logging? It ends what this session started (what ran before stays on).", "YES, STOP")}
            if r.get("already") and logging_view(self.state).auto:            # WP42: the DECLARE session runs - no second one
                lv2 = logging_view(self.state)
                self.toasts.event("info", f"logging already running since {lv2.t0 or 'DECLARE'} (automatic at DECLARE) - nothing new started")
                return True
            k = "already" if r.get("already") else r.get("needs")
            if not r.get("ok") and k in asks and not confirm:
                conf, text, yes = asks[k]
                Confirm(self, text, yes, lambda: self._log_session(key, conf))
                return True
            self._depth_toast(r)
            return False
        self.post("/logging", body, f"logging {op}", key, answer, answer_s=REC_ANSWER_S)

    def _depth_toast(self, r: Optional[dict]) -> None:
        """B v0.1.84.4: the answer's `depth` - the observer's first RGB + depth pair on the AGX, or why there is none."""
        d = (r or {}).get("depth")
        if isinstance(d, dict):
            self.toasts.event("ok" if d.get("ok") else "bad", ("✓ depth: " + str(d.get("text") or "pairs saved")) if d.get("ok")
                              else ("✗ depth: " + str(d.get("why") or "no depth")))
            self.toaster.draw()

    # ---- diff writes ------------------------------------------------------------------------------------------------------
    def put(self, key: str, widget, **kw) -> bool:
        """configure(**kw) only when it differs from the last write for this key; True when written."""
        sig = tuple(sorted(kw.items()))
        if self._cache.get(key) == sig:
            return False
        self._cache[key] = sig
        widget.configure(**kw)
        return True

    def _show(self, key: str, widget, on: bool, **pack) -> None:
        if self._cache.get("show:" + key) == on:
            return
        self._cache["show:" + key] = on
        if on:
            widget.pack(**pack)
        else:
            widget.pack_forget()

    def _btn_look(self, key: str) -> None:
        b, w = self.btn_model.get(key), self.buttons.get(key)
        if w is None:
            return
        look = self.presses.look(key)
        if b is None:
            label = w.cget("text")
            base = ""
        else:
            label = b.label
            base = b.kind if b.ok else "grey"
        pre = {"sending": "⟶ ", "ok": "✓ ", "bad": "✗ "}.get(look, "")
        self.put("btn:" + key, w, text=pre + label.lstrip("✓ ") if pre else label, style=f"B{look or base}.TButton")

    def refresh_buttons(self) -> None:
        for k in self.buttons:
            self._btn_look(k)

    def render(self) -> None:
        v = self.view = self.model.apply(self.state, self.fast, self.alarms, self.misses)
        for lvl, text in v.events:
            self.toasts.event(lvl, text)
        if v.events:
            self.toaster.draw()
        if self.root.title() != v.title:
            self.root.title(v.title)
        for k in HEADER_KEYS:
            c = v.header.get(k)
            if c is not None:
                self.put("chip:" + k, self.chips[k], text=c.text, style=f"Chip{c.level}.TLabel")
        self.put("word", self.word, text=v.word.text, style=f"Word{v.word.level}.TLabel")
        self.put("hint", self.hint, text=v.hint)
        self._fill_missions(v)
        self._render_alarms(v)
        self._render_ladder(v)
        self._render_tiles(v)
        self._render_speed()
        self._render_log(v)
        self._render_cmd()
        self._render_mission(v)
        self._render_camera()
        self._render_map(v)
        self._render_vehicles()
        try:
            tab = self.nb.tab(self.nb.select(), "text")
        except tk.TclError:
            tab = ""
        if tab == "JUDGE":
            PN.render_judge(self)
        if self.set_on:
            stab = self.set_nb.tab(self.set_nb.select(), "text")
            if stab == "SETTINGS":
                PN.render_tuning(self)
                PN.render_configs(self)
            elif stab == "SYSTEM":
                PN.render_system(self)
            elif stab == "HELP":
                PN.render_help(self)
        if self.big_on:
            PN.render_big(self)
        self.refresh_buttons()

    def _render_alarms(self, v: View) -> None:
        n = len(v.alarms)
        fg, _bg = TH.level_colours(self.pal, v.alarms_level)
        self.put("al-count", self.al_count, text=(f"ALARMS {n} active · {v.alarms_unacked} unacked" if n else "ALARMS none active"),
                 foreground=fg)
        for i, (fr, lab) in enumerate(self.al_rows):
            a = v.alarms[i] if i < n else None
            self._show(f"al{i}", fr, a is not None, fill="x")
            if a is None:
                self.btn_model.pop(f"ack-{i}", None)
                continue
            lvl = {"ALARM": "bad", "WARNING": "warn"}.get(a.get("level"), "info")
            self.put(f"al{i}", lab, text=f"{a.get('level')} · {a.get('text', '')}" + (" · acked" if a.get("acked") else ""),
                     foreground=TH.level_colours(self.pal, lvl)[0])
            self.btn_model[f"ack-{i}"] = Btn(f"ack {a.get('key')}", "ACK", ok=not a.get("acked"), key=f"ack-{i}")
        self.btn_model["ack-all"] = Btn("ack all", "ACK ALL", ok=v.alarms_unacked > 0, key="ack-all")

    def _render_ladder(self, v: View) -> None:
        p = self.pal
        self.put("lad:profile", self.lad_profile, text=f"MISSION 1 · {v.profile}" if v.profile else "")
        n_shown = len(v.steps)
        if self._cache.get("lad:n") != n_shown:                     # v0.1.89: as many step frames as the profile has, in order
            self._cache["lad:n"] = n_shown
            for n in range(1, LADDER_MAX + 1):
                self.steps[n]["fr"].pack_forget()
            for n in range(1, n_shown + 1):
                self.steps[n]["fr"].pack(fill="x", pady=4)
        self._render_banner(v)
        self._render_mods(v, n_shown)
        for s in v.steps:
            w = self.steps[s.n]
            bg = {"now": p["accBg"], "done": p["panel"]}.get(s.state, p["bg"])
            line = {"now": p["acc"]}.get(s.state, p["line2"])
            self.put(f"st{s.n}:fr", w["fr"], background=bg, highlightbackground=line, highlightthickness=2 if s.state == "now" else 1)
            for part in ("top", "bts"):
                self.put(f"st{s.n}:{part}", w[part], background=bg)
            fgt = p["dim"] if s.state == "next" else p["text"]
            self.put(f"st{s.n}:num", w["num"], text="✓" if s.state == "done" else str(s.n), background=bg,
                     foreground=p["ok"] if s.state == "done" else (p["acc"] if s.state == "now" else p["dim"]))
            self.put(f"st{s.n}:title", w["title"], text=s.title, background=bg, foreground=fgt)
            self.put(f"st{s.n}:chip", w["chip"], text=(f"✓ {s.done_at}" if s.state == "done" and s.done_at else s.chip))
            now = s.state == "now"
            self.put(f"st{s.n}:detail", w["detail"], text=s.detail, background=bg)
            self._show(f"st{s.n}:detail", w["detail"], bool(s.detail) and s.state != "next", fill="x")
            self.put(f"st{s.n}:warn", w["warn"], text=s.warn, background=bg)
            self._show(f"st{s.n}:warn", w["warn"], bool(s.warn), fill="x")
            self._show(f"st{s.n}:bts", w["bts"], (now or s.sticky) and bool(s.buttons), fill="x", pady=(6, 0))
            shown = {b.key for b in s.buttons}
            for k in STEP_KEYS.get(s.n, []):
                b = next((x for x in s.buttons if x.key == k), None)
                if b is not None:
                    self.btn_model[k] = b
                self._show("btn:" + k, self.buttons[k], k in shown, side="left", padx=(0, 6))
            whys = [f"{b.label.strip('✓▶■❚ ')}: {b.why}" + (f" — {b.hint}" if b.hint else "") for b in s.buttons if not b.ok]
            self.put(f"st{s.n}:why", w["why"], text="\n".join(whys), background=bg)
            self._show(f"st{s.n}:why", w["why"], now and bool(whys), fill="x", pady=(4, 0))

    def _render_mods(self, v: View, n_shown: int) -> None:
        """v0.1.92: the mission modifiers subpanel, packed just before the ENABLE AUTO step; its buttons truly disabled when frozen."""
        m = v.mods
        n_auto = next((st.n for st in v.steps if st.title.startswith("ENABLE AUTO")), None)
        pos = (bool(m) and n_auto is not None, n_auto, n_shown)
        if self._cache.get("mods:pos") != pos:
            self._cache["mods:pos"] = pos
            self.mods_w["fr"].pack_forget()
            if pos[0]:
                self.mods_w["fr"].pack(fill="x", pady=4, before=self.steps[n_auto]["fr"])
        if not m:
            for k in MOD_KEYS_UI:
                self.btn_model.pop(k, None)
            return
        self.put("mods:line", self.mods_w["line"], text=m["line"])
        self.put("mods:why", self.mods_w["why"], text=m.get("why") or "")
        for b in m["buttons"]:
            self.btn_model[b.key] = b
            w = self.buttons.get(b.key)
            if w is not None:
                self.put("mods:state:" + b.key, w, state="normal" if b.ok else "disabled")

    # ---- v0.1.89: the run banner - a pop-up in the console's top-right corner (✕ hides it until the stage changes) and an
    # always-on-top window (closing it hides it until the stage changes); green, big font: RUN STARTED -> the mission stage
    BANNER_BG = {"ok": "#1e7a3c", "warn": "#a86b00", "idle": "#3d4a5c"}

    def _banner_widgets(self, parent, closer) -> dict:
        fr = tk.Frame(parent, background=self.BANNER_BG["ok"], padx=18, pady=10)
        top = tk.Frame(fr, background=self.BANNER_BG["ok"])
        top.pack(fill="x")
        title = tk.Label(top, text="", background=self.BANNER_BG["ok"], foreground="white", font=("Helvetica", 20, "bold"), anchor="w")
        title.pack(side="left", fill="x", expand=True)
        x = tk.Label(top, text="✕", background=self.BANNER_BG["ok"], foreground="white", font=("Helvetica", 18, "bold"), cursor="hand2")
        x.pack(side="right", padx=(12, 0))
        x.bind("<Button-1>", lambda e: closer())
        # v0.1.92: line 2 the FCU (mode · ARMED|disarmed · V-state); the stage (big) with its reason; what is next; the modifiers
        fcu = tk.Label(fr, text="", background=self.BANNER_BG["ok"], foreground="white", font=("Menlo", 16, "bold"), anchor="w", justify="left")
        fcu.pack(fill="x")
        stage = tk.Label(fr, text="", background=self.BANNER_BG["ok"], foreground="white", font=("Helvetica", 30, "bold"), anchor="w",
                         justify="left", wraplength=760)
        stage.pack(fill="x")
        why = tk.Label(fr, text="", background=self.BANNER_BG["ok"], foreground="white", font=("Helvetica", 17), anchor="w",
                       justify="left", wraplength=760)
        nxt = tk.Label(fr, text="", background=self.BANNER_BG["ok"], foreground="white", font=("Helvetica", 17, "bold"), anchor="w",
                       justify="left", wraplength=760)
        mods = tk.Label(fr, text="", background=self.BANNER_BG["ok"], foreground="white", font=("Menlo", 11), anchor="w",
                        justify="left", wraplength=760)
        drone = tk.Label(fr, text="", background=self.BANNER_BG["ok"], foreground="white", font=("Menlo", 12), anchor="w",
                         justify="left", wraplength=760)                      # v0.1.93.2: the drone's map as the boat received it
        return {"fr": fr, "top": top, "title": title, "x": x, "fcu": fcu, "stage": stage, "why": why, "next": nxt, "mods": mods, "drone": drone}

    def _banner_paint(self, w: dict, b: dict) -> None:
        bg = self.BANNER_BG.get(b.get("level") or "ok", self.BANNER_BG["ok"])
        for k in ("fr", "top", "title", "x", "fcu", "stage", "why", "next", "mods", "drone"):
            if k in w:
                w[k].configure(background=bg)
        w["title"].configure(text=b.get("title") or "")
        w["stage"].configure(text="→ " + (b.get("stage") or ""))
        for k in ("fcu", "why", "next", "mods", "drone"):           # an older boat / the run's end: the line is simply not there
            if k in w:
                w[k].configure(text=str(b.get(k) or ""))
                w[k].pack_forget()
        if "fcu" in w and b.get("fcu"):
            w["fcu"].pack(fill="x", before=w["stage"])
        for k in ("why", "next", "drone", "mods"):                  # after the stage, in this order
            if k in w and b.get(k):
                w[k].pack(fill="x")

    def _render_banner(self, v: View) -> None:
        b = v.banner
        key = None if not b else (b.get("title"), b.get("stage"), b.get("level"))
        if key == self._cache.get("banner:key"):
            sig = None if not b else tuple(str(b.get(k) or "") for k in ("fcu", "why", "next", "mods", "drone"))
            if b and sig != self._cache.get("banner:sig"):          # v0.1.92: the reason / FCU / next changed - repaint, never re-show
                self._cache["banner:sig"] = sig
                self._banner_paint(self._ban_corner, b)
                if self._ban_win is not None:
                    try:
                        self._banner_paint(self._ban_win, b)
                    except tk.TclError:
                        self._ban_win = None
            return
        self._cache["banner:key"] = key
        self._cache["banner:sig"] = None if not b else tuple(str(b.get(k) or "") for k in ("fcu", "why", "next", "mods", "drone"))
        if not hasattr(self, "_ban_corner"):
            self._ban_corner = self._banner_widgets(self.root, lambda: self._ban_hide("corner"))
            self._ban_win, self._ban_hidden = None, set()
        if b is None:
            self._ban_corner["fr"].place_forget()
            if self._ban_win is not None:
                self._ban_win["top_level"].withdraw()
            self._ban_hidden = set()
            return
        self._ban_hidden = set()                                     # a new stage shows both again
        self._banner_paint(self._ban_corner, b)
        self._ban_corner["fr"].place(relx=1.0, x=-14, y=14, anchor="ne")
        self._ban_corner["fr"].lift()
        try:
            if self._ban_win is None:
                tl = tk.Toplevel(self.root)
                tl.title("RUN")
                tl.attributes("-topmost", True)
                tl.protocol("WM_DELETE_WINDOW", lambda: self._ban_hide("window"))
                w = self._banner_widgets(tl, lambda: self._ban_hide("window"))
                w["fr"].pack(fill="both", expand=True)
                w["top_level"] = tl
                self._ban_win = w
            self._banner_paint(self._ban_win, b)
            self._ban_win["top_level"].deiconify()
            self._ban_win["top_level"].attributes("-topmost", True)
        except tk.TclError:
            self._ban_win = None

    def _ban_hide(self, which: str) -> None:
        self._ban_hidden.add(which)
        if which == "corner":
            self._ban_corner["fr"].place_forget()
        elif self._ban_win is not None:
            self._ban_win["top_level"].withdraw()

    def _render_tiles(self, v: View) -> None:
        for k, lab in self.usv_kv.items():
            self.put("usv:" + k, lab, text=v.usv.get(k) or "—")
        for k, lab in self.uav_kv.items():
            self.put("uav:" + k, lab, text=v.uav.get(k) or "—")
        for b in v.usv_buttons + v.uav_buttons:
            self.btn_model[b.key] = b
        by = {b.key: b for b in v.usv_buttons}
        for name, (row, why, keys) in self.usv_rows.items():
            ws = [f"{by[k].label.strip('✓▶■❚ ')}: {by[k].why}" + (f" — {by[k].hint}" if by[k].hint else "") for k in keys if k in by and not by[k].ok]
            self.put("usvwhy:" + name, why, text="\n".join(ws))
            if self._cache.get("show:usvwhy:" + name) != bool(ws):
                self._cache["show:usvwhy:" + name] = bool(ws)
                if ws:
                    why.pack(fill="x", after=row)
                else:
                    why.pack_forget()
        words, rwhy = uav_run_plan(self.state)
        self.btn_model["uav-run"] = Btn("uav run", "▶ UAV RUN (boat holds)", ok=not rwhy, why=rwhy, kind="go", key="uav-run")
        uws = [f"{b.label.strip('✓▶■❚ ')}: {b.why}" for b in v.uav_buttons if not b.ok] + ([f"UAV RUN: {rwhy}"] if rwhy else [])
        self.put("uav-why", self.uav_why, text="\n".join(uws))
        if v.mission_armed and v.mission_armed in getattr(self, "_mission_codes", []) and self._cache.get("armed") != v.mission_armed:
            self._cache["armed"] = v.mission_armed
            self.mpick.current(self._mission_codes.index(v.mission_armed))
        self.btn_model["usv-mission"] = Btn("mission …", "ARM MISSION", key="usv-mission")

    def _log_lines(self, v: View):
        lines = [(ln.t, ln.kind, ln.text, ln.sev, ln.group) for ln in v.log]
        lines += [(t, "ui", "console " + txt, {"sent": "", "wait": "warn", "warn": "warn", "info": ""}.get(lvl, lvl), "ui")
                  for t, lvl, txt in self.toasts.log[-200:]]
        lines.sort(key=lambda x: x[0])
        f = self.log_filter.get()
        if f == "cautions":
            return [x for x in lines if x[3] in ("warn", "bad")]
        return lines if f == "all" else [x for x in lines if x[4] == f]

    def _log_rebuild(self) -> None:
        self._log_last = None
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")
        self._log_n = 0
        if self.view:
            self._render_log(self.view)

    def _render_log(self, v: View) -> None:
        lines = self._log_lines(v)
        new = lines if self._log_last is None else [x for x in lines if (x[0], x[2]) > self._log_last]
        if not new:
            return
        at_bottom = self.log.yview()[1] >= 0.999
        self.log.configure(state="normal")
        for t, kind, text, sev, group in new:
            self.log.insert("end", time.strftime("%H:%M:%S ", time.localtime(t)), "t")
            tag = sev if sev in LOG_TAGS else ("ui" if group == "ui" else "")
            self.log.insert("end", f"{kind:6} {text}\n", tag)
        self._log_n += len(new)
        if self._log_n > LOG_KEEP:
            self.log.delete("1.0", f"{self._log_n - LOG_KEEP + 1}.0")
            self._log_n = LOG_KEEP
        self.log.configure(state="disabled")
        self._log_last = (new[-1][0], new[-1][2])
        if at_bottom:
            self.log.see("end")
        self.put("log-pause", self.log_pause, text="" if at_bottom else "⏸ paused (scrolled up) · End = follow")

    def _render_cmd(self) -> None:
        sig = tuple((t, w, ok) for t, w, ok, _ in self.cmd_results[:30])
        if self._cache.get("cmd-out") == sig:
            return
        self._cache["cmd-out"] = sig
        self.cmd_out.configure(state="normal")
        self.cmd_out.delete("1.0", "end")
        for t, w, ok, lines in self.cmd_results:
            self.cmd_out.insert("end", f"{t} > {w}  {'✓' if ok else '✗'}\n", "ok" if ok else "bad")
            for ln in lines[-4:]:
                self.cmd_out.insert("end", f"    {ln.strip()}\n", "dim")
        self.cmd_out.configure(state="disabled")
        self.btn_model["cmd-send"] = Btn("…", "SEND", key="cmd-send")
        self._render_catalogue()

    # ---- the CMD tab's catalogue (2026-11-11) ---------------------------------------------------------------------------------
    def _render_catalogue(self) -> None:
        if not self.model.meta:
            return
        tr = self.cmd_tree
        if not self._cmd_built:
            self._cmd_built = True
            groups = {}
            for c in command_catalogue(self.model.meta):
                if c.group not in groups:
                    groups[c.group] = tr.insert("", "end", iid="g:" + c.group, text=c.group, open=True, tags=("group",))
                tr.insert(groups[c.group], "end", iid="w:" + c.word, text=c.word, values=(c.does,))
                self.cmd_info[c.word] = c
        verdicts = (self.state or {}).get("verdicts") or {}
        for w, c in self.cmd_info.items():
            v = verdicts.get(c.action) if c.action else None
            no = bool(v) and not v.get("ok")
            if self._cache.get("cat:" + w) != no:
                self._cache["cat:" + w] = no
                tr.item("w:" + w, tags=("no",) if no else ())

    def _cmd_tip(self, iid: str) -> str:
        if not iid.startswith("w:"):
            return ""
        c = self.cmd_info.get(iid[2:])
        if c is None:
            return ""
        out = [c.usage or c.word]
        if c.does and c.does not in (c.usage or ""):
            out.append(c.does)
        if c.modifiers:
            out.append("modifiers:")
            out += [f"  {spec}" + (f"  — {h}" if h else "") for spec, h in c.modifiers]
        if c.rules:
            out.append(c.rules)
        v = ((self.state or {}).get("verdicts") or {}).get(c.action) if c.action else None
        if v:
            out.append("now: ✓ allowed" if v.get("ok") else f"now: ✗ {v.get('why')}" + (f"\n  → {v.get('hint')}" if v.get("hint") else ""))
        out.append(f"example: {c.example}")
        return "\n".join(out)

    def _cmd_pick(self) -> None:
        sel = self.cmd_tree.selection()
        if not sel or not sel[0].startswith("w:"):
            return
        c = self.cmd_info.get(sel[0][2:])
        if c is None:
            return
        self.cmd_entry.delete(0, "end")
        self.cmd_entry.insert(0, c.example or c.word)
        self.put("cmd-use", self.cmd_use, text=f"usage: {c.usage}" + (f"   ·   {c.rules}" if c.rules else ""), foreground=self.pal["accText"])

    def _render_mission(self, v: View) -> None:
        code = self._mission_sel()
        st = (self.state or {}).get("status") or {}
        rows = mission_stages(self.model.meta, code, st.get("task_name"), st.get("task_state")) if code else []
        sig = tuple(rows)
        if self._cache.get("mis-stages") != sig:
            self._cache["mis-stages"] = sig
            self.mis_stages.configure(state="normal")
            self.mis_stages.delete("1.0", "end")
            for name, meaning, now in rows:
                self.mis_stages.insert("end", f"{'▶' if now else ' '} {name:14}", "now" if now else "")
                self.mis_stages.insert("end", f" {meaning}\n", "dim")
            self.mis_stages.configure(state="disabled")
        mm = (self.state or {}).get("mission_metrics") or {}
        self.put("mis-metrics", self.mis_metrics, text="metrics: " + " · ".join(f"{k} {val}" for k, val in mm.items()) if mm else "")
        self.btn_model["mis-arm"] = Btn("mission …", "ARM", kind="go", key="mis-arm")
        self.btn_model["mis-start"] = Btn("start_task …", "▶ START NOW", kind="act", key="mis-start")

    def _render_map(self, v: View) -> None:
        try:
            if self.nb.tab(self.nb.select(), "text") != "MAP":
                return                                            # drawn only while visible
        except tk.TclError:
            return
        pose = v.map.pose
        sig = (id(self.mapj), id(self.model.course), pose, v.map.hdg_deg)
        if sig == self._map_sig:
            return
        self._map_sig = sig
        self.cmap.set_layer(build_layer(self.model.course, self.mapj, pose, v.map.hdg_deg, v.map.visited))
        self.put("follow", self.follow_btn, style="Bon.TButton" if self.cmap.follow else "B.TButton")

    # ---- VEHICLES (2026-11-11: IMU, compass, heartbeat per vehicle) -----------------------------------------------------------
    def _build_vehicles(self, fr) -> None:
        self.veh_sf = ScrollFrame(fr, self.pal["panel"])
        self.veh_sf.outer.pack(fill="both", expand=True)
        self.veh_cards: Dict[str, dict] = {}

    def _veh_card(self, vid: str, kind: str) -> dict:
        p = self.pal
        fr = tk.Frame(self.veh_sf.inner, background=p["panel"], highlightthickness=1, highlightbackground=p["line"], padx=12, pady=8)
        fr.pack(fill="x", pady=(0, 10))
        top = tk.Frame(fr, background=p["panel"])
        top.pack(fill="x")
        self._label(top, f"{vid}  ·  {kind}", font=("Helvetica", 14, "bold")).pack(side="left")
        link = ttk.Label(top, text="…", style="Chipidle.TLabel")
        link.pack(side="right")
        body = tk.Frame(fr, background=p["panel"])
        body.pack(fill="x", pady=(6, 0))
        cv = tk.Canvas(body, width=150, height=150, background=p["mapBg"], highlightthickness=1, highlightbackground=p["line2"])
        cv.pack(side="left", anchor="n", padx=(0, 10))
        grid = tk.Frame(body, background=p["panel"])
        grid.pack(side="left", fill="x", expand=True)
        c = {"fr": fr, "link": link, "cv": cv, "grid": grid, "rows": {}, "sig": None}
        self.veh_cards[vid] = c
        return c

    def _draw_instruments(self, cv: tk.Canvas, hdg, roll, pitch) -> None:
        """The compass rose (N up, the needle = heading) and, inside it, an attitude line (roll tilts it, pitch moves it)."""
        import math
        p = self.pal
        cv.delete("all")
        cx, cy, r = 75, 75, 62
        cv.create_oval(cx - r, cy - r, cx + r, cy + r, outline=p["line"], width=2)
        for d, lab in ((0, "N"), (90, "E"), (180, "S"), (270, "W")):
            a = math.radians(d)
            cv.create_text(cx + (r - 10) * math.sin(a), cy - (r - 10) * math.cos(a), text=lab, fill=p["muted"], font=("Helvetica", 10, "bold"))
        for d in range(0, 360, 30):
            a = math.radians(d)
            cv.create_line(cx + (r - 3) * math.sin(a), cy - (r - 3) * math.cos(a), cx + r * math.sin(a), cy - r * math.cos(a), fill=p["dim"])
        if roll is not None:
            ra = math.radians(roll or 0.0)
            dy = max(-25.0, min(25.0, (pitch or 0.0) * 1.2))
            L = 34
            cv.create_line(cx - L * math.cos(ra), cy + dy + L * math.sin(ra), cx + L * math.cos(ra), cy + dy - L * math.sin(ra),
                           fill=p["ok"], width=3, tags=("attitude",))
            cv.create_line(cx - 8, cy, cx + 8, cy, fill=p["text"], width=1)
        if hdg is not None:
            a = math.radians(hdg)
            cv.create_line(cx, cy, cx + (r - 16) * math.sin(a), cy - (r - 16) * math.cos(a), fill=p["alarm"], width=3, arrow="last",
                           tags=("needle",))
            cv.create_text(cx, cy + r - 22, text=f"{hdg:05.1f}°", fill=p["text"], font=("Menlo", 11, "bold"), tags=("hdgtext",))
        else:
            cv.create_text(cx, cy, text="no heading", fill=p["dim"], font=("Helvetica", 10))

    def _render_vehicles(self) -> None:
        try:
            if self.nb.tab(self.nb.select(), "text") != "VEHICLES":
                return
        except tk.TclError:
            return
        for vc in vehicle_cards(self.state, self.fast):
            c = self.veh_cards.get(vc.vid) or self._veh_card(vc.vid, vc.kind)
            self.put(f"veh:{vc.vid}:link", c["link"], text=vc.link.text, style=f"Chip{vc.link.level}.TLabel")
            sig = (None if vc.hdg is None else round(vc.hdg, 1), None if vc.roll is None else round(vc.roll, 1),
                   None if vc.pitch is None else round(vc.pitch, 1))
            if sig != c["sig"]:
                c["sig"] = sig
                self._draw_instruments(c["cv"], vc.hdg, vc.roll, vc.pitch)
            for i, (label, text, level) in enumerate(vc.rows):
                if label not in c["rows"]:
                    k = self._label(c["grid"], label, fg="dim", font=("Helvetica", 11, "bold"), width=10)
                    v = self._label(c["grid"], "", font=("Menlo", 12), wraplength=300)
                    k.grid(row=i, column=0, sticky="nw")
                    v.grid(row=i, column=1, sticky="w")
                    c["rows"][label] = v
                fg = TH.level_colours(self.pal, level)[0] if level else self.pal["text"]
                self.put(f"veh:{vc.vid}:{label}", c["rows"][label], text=text, foreground=fg)

    def _render_camera(self) -> None:
        c, r = self.cam_state, self.rec_state
        if not self.cam.active.is_set():
            txt = "the stream starts when this tab opens"
        elif not c:
            txt = "asking the OCS for the camera …"
        elif c.get("ok"):
            txt = f"live · {c.get('url', '')}snapshot.jpg"
        else:
            txt = f"no stream: {c.get('why') or '?'}" + (" · start asked (SYSTEM cam start)" if self._cam_started else "")
        self.put("cam-text", self.cam_text, text=txt)
        on = bool(r.get("recording"))
        why = ((r.get("agx") or {}).get("why") or "") if not on else ""       # B v0.1.83.6: e.g. "the AGX's agx_observer.sh predates …"
        txt = r.get("text") or ("recording: —" if not r else ("● recording" if on else "not recording"))
        self.put("rec-text", self.rec_text, text=txt + (f"\n{why}" if why and why not in txt else ""),
                 foreground=self.pal["alarm"] if on else self.pal["text"])
        self.put("rec-where", self.rec_where, text=rec_where(r))
        lv = logging_view(self.state)
        self.put("log-sess", self.log_sess, text=(f"● logging · {lv.label}" if lv.running else "logging: FCU log + AGX video + Mac frames, one label"),
                 foreground=self.pal["alarm"] if lv.running else self.pal["muted"])
        self.put("clock-line", self.clock_line, text=lv.clocks, foreground=TH.level_colours(self.pal, lv.clocks_level)[0])
        for k in ("log-hdr", "log-cam"):
            self.btn_model[k] = Btn("logging", lv.button, kind="danger" if lv.running else "act", key=k,
                                    why="" if not lv.clock_problem else lv.clock_problem)
        self.btn_model["cam-rec"] = Btn("recording", "■ STOP RECORDING" if on else "⏺ RECORD", kind="danger" if on else "act", key="cam-rec")


def rec_where(r: dict) -> str:
    """Where the recording goes (2026-11-11: "I need to see on the OCS where the images are being stored"): the AGX video folder and
    the Mac frames folder from /recording.json; before a recording, the Mac's default root (frame_rec) and what the AGX said."""
    agx, mac = (r or {}).get("agx") or {}, (r or {}).get("mac") or {}
    tg = (r or {}).get("targets") or {}
    a = agx.get("dir") or tg.get("agx") or ""
    m = mac.get("dir") or tg.get("mac") or ""
    if not m:
        try:
            from ..frame_rec import root_path
            m = root_path("mac_logs") + "/<date>/<time>_<label>/  (when it starts)"
        except Exception:
            m = "?"
    nd = mac.get("depth_frames")
    depth = (f" · depth {nd} of {mac.get('frames', 0)}" if nd is not None else (" · depth on" if mac.get("depth") else ""))
    pairs = agx.get("pairs")
    pdir = agx.get("pairs_dir") or (a.rstrip("/") + "/pairs" if a and pairs else "")
    return (f"stored in:\n  AGX video (svo2: RGB + depth + IMU): {a or 'on the AGX (the folder shows when it records)'}"
            + (f"\n  AGX RGB + depth pairs: {pairs} → {pdir}" if pairs is not None else "")
            + f"\n  Mac frames (2 Hz JPEG{' + depth' if mac.get('depth') else ''}): {m}"
            + (f"\n  {mac.get('frames', 0)} frames · {mac.get('mb', 0)} MB{depth}" if mac.get("state") == "recording" or mac.get("frames") else ""))


def default_geometry(sw: int, sh: int) -> str:
    """Dmitrii 2026-11-12: "default size can be bigger, as I'm using a Mac (however don't eat full space)" - 90 % of the screen, at
    least 1440 x 900 (capped to the screen), centred, the menu bar and the Dock left free."""
    w = max(min(1440, sw), min(int(sw * 0.90), 2200))
    h = max(min(900, sh - 80), min(int(sh * 0.88), 1400))
    return f"{w}x{h}+{max(0, (sw - w) // 2)}+{max(25, (sh - h) // 2 - 10)}"
