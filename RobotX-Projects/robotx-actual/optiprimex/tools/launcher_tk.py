"""launcher_tk: the launcher as a tkinter window (v0.1.83.3, Dmitrii 2026-11-11: "lets also make launcher page as tkinter"; "B, now,
standalone"; "make sure that we use different launcher.sh, I will play with old one now").

A CLIENT of the launcher's own HTTP backend (optiprimex/tools/launcher.py - unchanged): GET /presets.json /status.json /session.json
/network.json /course_check.json /console.json /prefs, POST /plan /launch /stop /adopt /restart /ssh /observer /prefs (X-Launch: 1).
The same plan, the same refusals, the same launch record as the web page. Started by scripts/launch_tk.sh, which runs its OWN backend
on :8093 (LAUNCH_TK_PORT) - scripts/launch.sh and its :8094 page stay as they are.

Reliability rules (the reason for the move): Tk is touched only from the main thread - three worker threads (the 1 s poller, the slow
GETs that ping or ssh, the command queue) hand their results through one queue drained every 50 ms; no HTTP call ever runs in a handler;
widgets are written only when their text changed; every button press shows a toast top-right: grey "sent" at once, then green ✓ / red ✗
with the backend's answer (amber "no answer yet" after 10 s), 3 s after its final state.

The view-model functions (plan_lines, session_view, status_view, net_rows, course_rows, visible) and Toasts are plain Python, tested
without Tk (tests/test_launcher_tk.py).
"""
from __future__ import annotations

import argparse
import json
import queue
import threading
import time
import urllib.error
import urllib.request
import webbrowser
from typing import Any, Callable, Dict, List, Optional, Tuple

from optiprimex.tools.launcher_tips import ROW_TIPS, choice_tip

# ---- the form (the web page's cards and rows, in its order) ----------------------------------------------------------------------
BOOL_KEYS = ("usv_ui", "log", "auto_mission", "t4", "lucky")
NUM_KEYS = ("heartbeat_hz", "status_hz", "map_hz")
MANUAL = (("ocs_host", "OCS host"), ("boat_host", "boat host"), ("agx_host", "AGX host"), ("nx_ssh", "NX ssh"), ("agx_ssh", "AGX ssh"),
          ("rc_host", "RC host"), ("uav_host", "UAV host"), ("uav_listen_port", "UAV listen port"), ("uav_bridge_port", "UAV bridge port"),
          ("uav_broker_host", "UAV broker host"), ("uav_broker_port", "UAV broker port"), ("uav_buoy_path", "UAV buoy path"))
SEG_LABEL = {"judge": {"venue": "DEMO (venue)", "course": "course (auto)", "alpha": "ALPHA", "bravo": "BRAVO", "charlie": "CHARLIE",
                       "delta": "DELTA"}}


def _sim(c: dict) -> bool:
    return c.get("mode") == "sim"


# (card, key, label, kind seg|bool|num|text, shown(cfg))
ROWS: List[Tuple[str, str, str, str, Callable[[dict], bool]]] = [
    ("the run", "mode", "mode", "seg", lambda c: True),
    ("the run", "judge", "judge", "seg", lambda c: True),
    ("the run", "world", "world", "seg", lambda c: _sim(c) or c.get("cv") == "fake" or c.get("free_run") == "on"),
    ("the run", "perception", "perception", "seg", _sim),
    ("the run", "usv_ui", "usv_ui", "bool", lambda c: True),
    ("the run", "mission", "mission", "seg", lambda c: True),                  # v0.1.86 (WP42 §2.1): preselected at launch
    ("the run", "auto_mission", "auto start", "bool", lambda c: True),        # RunStart -> 3-2-1 -> start_task by itself
    ("the run", "t4", "T4 (incident)", "bool", lambda c: True),               # v0.1.88: T4 Dynamic Incident Response - off = TIER_NONE
    ("the run", "lucky", "lucky (gates)", "bool", lambda c: True),            # v0.1.90: uncertain-colour pairs as gates (a risk)
    ("the boat", "record", "record", "seg", lambda c: not _sim(c)),          # the ZED recording at DECLARE
    ("the boat", "fcu", "fcu", "seg", lambda c: not _sim(c)),
    ("the boat", "usv_profile", "profile", "seg", lambda c: not _sim(c)),
    ("the boat", "cv", "cv", "seg", lambda c: not _sim(c)),
    ("the boat", "usv_window", "USV window", "seg", lambda c: not _sim(c)),
    ("the boat", "uav_buoys", "UAV buoys", "seg", lambda c: not _sim(c) and c.get("uav") == "on"),
    ("the boat", "free_run", "free run", "seg", lambda c: not _sim(c) and "free_run" in c),
    ("the boat", "uav", "UAV", "seg", lambda c: not _sim(c)),
    ("network", "net", "net", "seg", lambda c: not _sim(c)),
] + [("network", k, lab, "text", lambda c: not _sim(c) and c.get("net") == "manual") for k, lab in MANUAL] + [
    ("rates and logging", "heartbeat_hz", "heartbeat Hz", "num", lambda c: True),
    ("rates and logging", "status_hz", "status Hz", "num", lambda c: True),
    ("rates and logging", "map_hz", "map Hz", "num", lambda c: True),
    ("rates and logging", "ui", "OCS page", "seg", lambda c: True),
    ("rates and logging", "log", "log", "bool", lambda c: True),
]
CARDS = ("the run", "the boat", "network", "rates and logging")


def visible(cfg: dict) -> Dict[str, bool]:
    """row key -> shown (the web page's render())."""
    return {k: bool(f(cfg)) for _, k, _, _, f in ROWS}


def preset_cfg(presets: dict, name: str) -> dict:
    c = {k: v for k, v in (presets.get(name) or {}).items() if k not in ("title", "about")}
    if c.get("uav") == "on" and not c.get("uav_buoys"):
        c["uav_buoys"] = "broker"
    return c


def seg_label(k: str, c: Any) -> str:
    if k in BOOL_KEYS:
        return "on" if c else "off"
    return (SEG_LABEL.get(k) or {}).get(str(c), str(c))


# ---- the view-model: backend JSON -> display strings ----------------------------------------------------------------------------
def plan_lines(r: Optional[dict]) -> List[Tuple[str, str, Optional[str]]]:
    """/plan's answer -> [(tag, text, cmd to copy or None)]: errors, files, the numbered steps with their commands, the stop, notes."""
    if not r:
        return [("dim", "(no plan yet)", None)]
    out: List[Tuple[str, str, Optional[str]]] = [("err", e, None) for e in (r.get("errors") or [])]
    p = r.get("plan")
    if not p:
        return out
    for f in p.get("files") or []:
        out.append(("step", f"[mac] writes {f.get('path')} (a profile copy with your rates)", None))
    for f in p.get("remote_files") or []:
        out.append(("step", f"[nx] copies {f.get('path')} to {f.get('host')}", None))
    for i, s in enumerate(p.get("steps") or []):
        bad = (s.get("clock") or {}).get("ok") is False
        out.append(("err" if bad else "step", f"[{s.get('host')}] {i + 1}. {s.get('label')}", None))
        out.append(("cmd", str(s.get("cmd", "")), str(s.get("cmd", ""))))
    out.append(("step", "[mac] stop (STOP ALL runs it)", None))
    out.append(("cmd", str(p.get("stop", "")), str(p.get("stop", ""))))
    for n in p.get("notes") or []:
        out.append(("note", f"• {n}", None))
    return out


def status_view(s: Optional[dict]) -> dict:
    if s is None:
        return {"state": "launcher not answering", "tone": "bad", "log": [("bad", "no answer from the launcher backend")]}
    st = str(s.get("state", "?"))
    tone = "bad" if st == "failed" else "ok" if st in ("done", "adopted") else "run" if st == "running" else "idle"
    log = [(str(l.get("kind", "")), str(l.get("text", ""))) for l in (s.get("log") or [])]
    return {"state": st + (" · dry run" if s.get("dry") else ""), "tone": tone, "log": log or [("dim", "nothing launched yet")]}


def session_view(d: Optional[dict]) -> dict:
    """/session.json -> the banner, the LAUNCH block, read-only, the hosts, the page links, the progress lines."""
    if not d:
        return {"banner": False, "banner_text": "", "reconnect": False, "up": False, "readonly": False, "hosts": [], "links": [],
                "progress": [], "adopted": False, "record_cfg": None}
    probe, rec, adopted = d.get("probe") or {}, d.get("record"), bool(d.get("adopted"))
    up = bool(probe.get("up"))
    text, reconnect = "", False
    if up and not adopted:
        if rec:
            hs = " · ".join(f"{k}: {(', '.join(v.get('found') or []) or 'nothing') if v.get('reach') else 'no answer'}"
                            + (f" (missing {', '.join(v['missing'])})" if v.get("reach") and v.get("missing") else "")
                            for k, v in (probe.get("hosts") or {}).items())
            text, reconnect = f"The launch from {rec.get('when')} ({rec.get('title') or ''}) is still running. {hs}", True
        else:
            text = f"Running here without a launch record: {', '.join(probe.get('foreign') or [])} - stop it before a new launch"
    hosts = []
    for k, v in (probe.get("hosts") or {}).items():
        ok = bool(v.get("reach")) and not v.get("missing")
        txt = ((", ".join(v.get("found") or []) or "nothing") + (" · MISSING " + ", ".join(v["missing"]) if v.get("missing") else " · ok")
               if v.get("reach") else "does not answer")
        hosts.append({"host": k, "text": txt, "ok": ok, "restart": adopted and bool(v.get("reach")) and bool(v.get("missing"))})
    links = [{"name": p.get("name"), "url": p.get("url"), "up": bool(p.get("up")), "pin": p.get("pin"),
              "tip": f"{p.get('title', '')} - {p.get('why', '')}"} for p in d.get("pages") or []]
    prog = []
    for k, c in (d.get("clocks") or {}).items():
        off = c.get("offset_s")
        prog.append({"host": k, "text": "clock " + ("no answer" if off is None else f"{'+' if off > 0 else ''}{off} s") + " · limit 5 s",
                     "state": "died" if c.get("ok") is False else "up" if c.get("ok") else "unknown",
                     "word": "OFF" if c.get("ok") is False else "ok" if c.get("ok") else "?"})
    W = {"up": "up", "starting": "starting", "died": "died", "unknown": "?"}
    for p in d.get("progress") or []:
        prog.append({"host": p.get("host"), "text": f"{p.get('label')} · {p.get('why')}", "state": p.get("state"),
                     "word": W.get(p.get("state"), str(p.get("state")))})
    return {"banner": up and not adopted, "banner_text": text, "reconnect": reconnect, "up": up, "readonly": adopted and up,
            "hosts": hosts, "links": links, "progress": prog, "adopted": adopted, "record_cfg": (rec or {}).get("cfg") if adopted else None}


def net_rows(t: Optional[dict]) -> List[dict]:
    out = []
    for r in (t or {}).get("rows") or []:
        hosts = r.get("hosts") or []
        s = r.get("ssh") or {}
        out.append({"role": r.get("role", ""), "key": r.get("key", ""), "lan": r.get("lan") or "-", "ts": r.get("ts") or "-",
                    "now": " ".join(("● " if h.get("up") else "○ ") + str(h.get("addr")) for h in hosts) or "not set",
                    "up": bool(hosts) and bool(hosts[0].get("up")), "ssh": s.get("which") if s.get("target") else None,
                    "auth": s.get("auth"), "pin": r.get("pin"), "observer": r.get("observer")})
    return out


def course_rows(t: Optional[dict]) -> Tuple[str, List[Tuple[str, ...]]]:
    if not t:
        return "no answer from the launcher", []
    n = t.get("nic")
    head = (f"this Mac: {n.get('addr')} ({n.get('iface')}{', DHCP' if n.get('dhcp') else ', no DHCP lease'})" if n
            else "this Mac: no interface in 192.168.65-68.x") + (" · the OCS runs (no RxCourse probe)" if t.get("ocs_running") else "")
    rows = []
    for r in t.get("rows") or []:
        rx = r.get("rx_course")
        rxs = "-" if not rx else rx["error"] if rx.get("error") else \
            f"{rx.get('course_id') or '?'} · {rx.get('corners')} corners · pinger {round((rx.get('pinger_hz') or 0) / 1000)} kHz"
        rows.append((str(r.get("course")), f"{r.get('host') or '-'}:{r.get('port') or '-'}",
                     "● on this network" if r.get("on_network") is True else "not on this network" if r.get("on_network") is False else "-",
                     "up" if r.get("broker") is True else "no answer" if r.get("broker") is False else "-", rxs))
    return head, rows


# ---- the toasts (plain logic; the window only draws what tick() returns) ---------------------------------------------------------
class Toasts:
    HOLD_S, LATE_S, MAX = 3.0, 10.0, 5

    def __init__(self, clock: Callable[[], float] = time.time):
        self.clock, self.items, self.n = clock, [], 0

    def add(self, text: str, state: str = "sent") -> int:
        """state sent (waits for resolve) | ok | bad | info (final at once: a state change nobody clicked)."""
        self.n += 1
        now = self.clock()
        self.items.append({"id": self.n, "text": text, "state": state, "t0": now, "tf": None if state == "sent" else now})
        return self.n

    def resolve(self, tid: int, ok: bool, text: str) -> None:
        for t in self.items:
            if t["id"] == tid:
                t.update(state="ok" if ok else "bad", text=text, tf=self.clock())

    def dismiss(self, tid: int) -> None:
        self.items = [t for t in self.items if t["id"] != tid]

    def tick(self) -> Tuple[List[dict], int]:
        """-> (the visible toasts newest first, how many more are hidden)."""
        now = self.clock()
        keep = []
        for t in self.items:
            if t["tf"] is not None and now - t["tf"] >= self.HOLD_S:
                continue
            if t["state"] == "sent" and now - t["t0"] >= self.LATE_S:
                t["state"] = "late"
            keep.append(t)
        self.items = keep
        vis = list(reversed(keep))
        return vis[:self.MAX], max(0, len(vis) - self.MAX)


TOAST_MARK = {"sent": "⟶", "late": "…", "ok": "✓", "bad": "✗", "info": "●"}

# ---- v0.1.84.1: the OCS console (Dmitrii via session A: "opening the OCS window should start the tk console instead of the HTML page") --
KIT = __import__("pathlib").Path(__file__).resolve().parents[2]
OCS_LINK = "OCS page"


def console_argv(kit, cfg: dict) -> List[str]:
    """scripts/run_ocs_tk.sh with the launch's network profile (sim: none - the OCS is on this Mac either way)."""
    net = cfg.get("net") if cfg.get("mode") == "real" else None
    return [str(kit / "scripts" / "run_ocs_tk.sh")] + (["--lan"] if net == "lan" else ["--ts"] if net == "ts" else [])


def reach_line(d: Optional[dict]) -> str:
    """v0.1.86: /reach.json -> 'AGX ● · USV-NX ○ · UAV-NX - · RoboCommand reachable: venue' (- = no address set)."""
    if not d:
        return "reachable: no answer from the launcher"
    j = " · ".join(f"{v.get('label')} {'●' if v.get('up') else '-' if v.get('up') is None else '○'}" for v in (d.get("jetsons") or {}).values())
    return (j + " · " if j else "") + str(d.get("line") or "")


def tiers_line(cfg: dict) -> str:
    t = dict(cfg.get("tiers")) if isinstance(cfg.get("tiers"), dict) else {}
    t["T4"] = "CORE" if cfg.get("t4") is True else "NONE"            # v0.1.88: the T4 tick box wins
    rest = "" if isinstance(cfg.get("tiers"), dict) else "profile · "
    return "tiers: " + rest + " · ".join(f"{k} {v}" for k, v in t.items())


def judge_target(cfg: dict, env: Dict[str, str]) -> dict:
    """v0.1.84.2 (session A: Dmitrii's 14:54 launch had judge rctest - every heartbeat went to this Mac's rc-test, none to the venue):
    where this launch's OCS reports -> {word, where, host, port} (host None = on our own hub)."""
    j = str(cfg.get("judge") or "fake")
    if j == "fake":
        return {"word": "fake", "where": "our fake judge (on the hub) - nothing leaves this Mac", "host": None, "port": None}
    if j == "rctest":
        return {"word": "rctest", "where": "rc-test on THIS Mac 127.0.0.1:1885 (home practice)", "host": "127.0.0.1", "port": 1885}
    if j == "venue":
        h, pt = env.get("VENUE_RC_HOST"), env.get("VENUE_RC_PORT") or "1883"
        return {"word": "venue", "where": f"DEMO - the venue rc-test {h or '? (VENUE_RC_HOST unset)'}:{pt}", "host": h, "port": pt}
    if j == "course":
        h = env.get("RC_HOST")
        return {"word": "course", "where": f"the course RoboCommand {h + ':' + (env.get('RC_PORT') or '1883') if h else '(the course NIC DHCP gateway)'}",
                "host": h, "port": env.get("RC_PORT") or "1883"}
    c = j.upper()
    h, pt = env.get(f"RC_{c}_HOST"), env.get(f"RC_{c}_PORT") or "1883"
    return {"word": j, "where": f"{c} - the course RoboCommand {h or '? (RC_' + c + '_HOST unset)'}:{pt}", "host": h, "port": pt}


def judge_warning(cfg: dict, t: dict, venue_up: Optional[bool], env: Dict[str, str]) -> str:
    """A real launch reporting to our fake judge / this Mac's rc-test while the venue rc-test answers: say so before LAUNCH."""
    if cfg.get("mode") == "real" and t["word"] in ("fake", "rctest") and venue_up:
        return (f"the venue rc-test answers at {env.get('VENUE_RC_HOST')}:{env.get('VENUE_RC_PORT')} - this launch reports to "
                f"{'our fake judge' if t['word'] == 'fake' else 'this Mac rc-test'}, NOT the venue: pick judge DEMO (venue)")
    return ""


def auto_open(prev_up: Optional[bool], now_up: bool, console: str) -> bool:
    """Open the tk console by itself only when the OCS link goes down -> up (a launch's OCS step came up) and the choice is tk; never
    on the window's first look (prev None: a console may already be open - the script's own guard would refuse a second anyway)."""
    return console == "tk" and prev_up is False and now_up


def spawn_console(argv: List[str], log_path: str, wait_s: float = 2.0) -> dict:
    """Start the console detached (closing the launcher leaves it open); wait up to wait_s: an exit 0 = the script's own guard ("already
    open (pid N)"), an exit != 0 = refused (no Tk / no venv) - the log's last line says why; still running = opened."""
    import os
    import subprocess
    os.makedirs(os.path.dirname(log_path), exist_ok=True)
    with open(log_path, "ab") as fh:
        try:
            p = subprocess.Popen(argv, stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True)
        except OSError as exc:
            return {"ok": False, "why": f"cannot start {argv[0]}: {exc}"}
    try:
        rc = p.wait(timeout=wait_s)
    except subprocess.TimeoutExpired:
        return {"ok": True, "text": f"opened (pid {p.pid})"}
    try:
        with open(log_path, "rb") as fh:
            last = (fh.read()[-400:].decode(errors="ignore").strip().splitlines() or [""])[-1]
    except OSError:
        last = ""
    return {"ok": rc == 0, "text": last or "already open", "why": last or f"exit {rc}"}


# ---- the network side (never on the Tk thread) ------------------------------------------------------------------------------------
class Api:
    def __init__(self, base: str, timeout: float = 2.0):
        self.base, self.timeout = base.rstrip("/"), timeout

    def get(self, path: str, timeout: Optional[float] = None) -> Optional[dict]:
        try:
            with urllib.request.urlopen(self.base + path, timeout=timeout or self.timeout) as r:
                return json.loads(r.read() or b"{}")
        except Exception:
            return None

    def post(self, path: str, body: dict, timeout: float = 60.0) -> dict:
        rq = urllib.request.Request(self.base + path, data=json.dumps(body).encode(), method="POST",
                                    headers={"Content-Type": "application/json", "X-Launch": "1"})
        try:
            with urllib.request.urlopen(rq, timeout=timeout) as r:
                return json.loads(r.read() or b"{}")
        except urllib.error.HTTPError as e:
            try:
                return json.loads(e.read() or b"{}")
            except ValueError:
                return {"ok": False, "why": f"HTTP {e.code}"}
        except Exception as exc:
            return {"ok": False, "why": f"no answer from the launcher ({exc.__class__.__name__})"}


class Worker(threading.Thread):
    """A queue of jobs run one at a time; each result goes to `out` as (kind, payload). `plan` jobs coalesce (only the newest runs)."""

    def __init__(self, out: "queue.Queue", name: str):
        super().__init__(daemon=True, name=name)
        self.q: "queue.Queue" = queue.Queue()
        self.out, self.stop_ev = out, threading.Event()
        self.plan_seq = 0

    def put(self, kind: str, fn: Callable[[], Any], tag: Any = None) -> None:
        if kind == "plan":
            self.plan_seq += 1
            tag = self.plan_seq
        self.q.put((kind, fn, tag))

    def run(self) -> None:
        while not self.stop_ev.is_set():
            try:
                kind, fn, tag = self.q.get(timeout=0.3)
            except queue.Empty:
                continue
            if kind == "plan" and tag != self.plan_seq:
                continue                                     # a newer plan request is queued
            try:
                res = fn()
            except Exception as exc:                         # never into the UI as an exception
                res = {"ok": False, "why": f"{exc.__class__.__name__}: {exc}"}
            self.out.put((kind, (tag, res)))


class Poller(threading.Thread):
    """status 1 s, session 3 s, prefs 2 s, console 2 s (when a window is picked). Misses are None, never raised."""

    def __init__(self, api: Api, out: "queue.Queue"):
        super().__init__(daemon=True, name="poller")
        self.api, self.out, self.stop_ev = api, out, threading.Event()
        self.con_key = ""
        self.next = {"status": 0.0, "session": 0.0, "prefs": 0.0, "console": 0.0, "reach": 0.0}
        self.every = {"status": 1.0, "session": 3.0, "prefs": 2.0, "console": 2.0, "reach": 10.0}   # v0.1.86: the dots

    def kick(self, what: str) -> None:
        self.next[what] = 0.0

    def run(self) -> None:
        from urllib.parse import quote
        while not self.stop_ev.is_set():
            now = time.time()
            for k in self.next:
                if now >= self.next[k]:
                    self.next[k] = now + self.every[k]
                    path = {"status": "/status.json", "session": "/session.json", "prefs": "/prefs", "reach": "/reach.json",
                            "console": "/console.json?key=" + quote(self.con_key)}[k]
                    self.out.put((k, self.api.get(path, timeout=2.0 if k != "session" else 5.0)))
            self.stop_ev.wait(0.2)


class Tooltip:
    """v0.1.94.3 (Dmitrii 14 Nov): a pop-up after a short hover over a choice / a row label; gone on leave or click. Main thread only."""
    DELAY_MS = 450

    def __init__(self, widget, text, pal: Optional[dict] = None, wrap: int = 380):
        self.w, self.text, self.pal, self.wrap = widget, text, pal or {}, wrap
        self.job = None
        self.top = None
        widget.bind("<Enter>", self._enter, add="+")
        widget.bind("<Leave>", self._hide, add="+")
        widget.bind("<ButtonPress>", self._hide, add="+")

    def _enter(self, _e=None) -> None:
        self._cancel()
        self.job = self.w.after(self.DELAY_MS, self._show)

    def _cancel(self) -> None:
        if self.job is not None:
            try:
                self.w.after_cancel(self.job)
            except Exception:
                pass
            self.job = None

    def _show(self) -> None:
        import tkinter as tk
        self.job = None
        text = self.text() if callable(self.text) else self.text
        if not text or self.top is not None:
            return
        x, y = self.w.winfo_rootx() + 8, self.w.winfo_rooty() + self.w.winfo_height() + 4
        self.top = t = tk.Toplevel(self.w)
        t.wm_overrideredirect(True)
        try:
            t.wm_attributes("-topmost", True)
        except Exception:
            pass
        tk.Label(t, text=text, justify="left", wraplength=self.wrap, padx=7, pady=4, borderwidth=1, relief="solid",
                 background=self.pal.get("toast", "#2a3040"), foreground=self.pal.get("toastText", "#ffffff")).pack()
        t.wm_geometry(f"+{x}+{y}")

    def _hide(self, _e=None) -> None:
        self._cancel()
        if self.top is not None:
            try:
                self.top.destroy()
            except Exception:
                pass
            self.top = None


# ---- the window ----------------------------------------------------------------------------------------------------------------
PALETTE = {
    "light": dict(bg="#f4f5f7", panel="#ffffff", panel2="#eceef2", ink="#1d2230", dim="#5d6577", line="#cfd4de", acc="#1f5fbf",
                  ok="#16803c", bad="#c0262d", warn="#a86400", run="#1f5fbf", go="#1f7a3a", goText="#ffffff", stop="#fbe3e4",
                  stopText="#8d1219", toast="#2a3040", toastText="#ffffff"),
    "dark": dict(bg="#14171d", panel="#1c2029", panel2="#262b36", ink="#e6e9ef", dim="#97a0b3", line="#353c4a", acc="#6aa8ff",
                 ok="#4fd27a", bad="#ff6b6b", warn="#f0b43c", run="#6aa8ff", go="#2f9e52", goText="#ffffff", stop="#4a1d21",
                 stopText="#ffb3b3", toast="#e6e9ef", toastText="#14171d"),
}
TOAST_BG = {"sent": "#6b7280", "late": "#b7791f", "ok": "#1f7a3a", "bad": "#b4232a", "info": "#1f5fbf"}


class App:
    def __init__(self, root, api: Api, theme: str = "light"):
        import tkinter as tk
        from tkinter import ttk
        self.tk, self.ttk, self.root, self.api = tk, ttk, root, api
        self.pal = PALETTE["dark" if theme == "dark" else "light"]
        self.inbox: "queue.Queue" = queue.Queue()
        self.cmd = Worker(self.inbox, "commands")
        self.slow = Worker(self.inbox, "slow")
        self.poller = Poller(api, self.inbox)
        self.toasts = Toasts()
        self.presets: dict = {}
        self.fields: dict = {}
        self.placeholders: dict = {}
        self.cur = ""
        self.cfg: dict = {}
        self.theme_word = "dark" if theme == "dark" else "light"   # the pages' theme (POST /prefs)
        self.plan_ok = False
        self.sess = session_view(None)
        self.vars: Dict[str, Any] = {}
        self.rows: Dict[str, Tuple[Any, Any]] = {}
        self.cards: Dict[str, Any] = {}
        self.form_widgets: List[Any] = []
        self.last_text: Dict[str, str] = {}
        self.loaded_rec = False
        self.last_state = None
        self.suppress = False
        self.closing = False
        self.toast_frames: List[Any] = []
        self.spawn: Callable[[List[str], str], dict] = spawn_console   # tests swap it
        self.ocs_up: Optional[bool] = None
        self.judge_probe: Dict[str, Any] = {}
        self._build()
        for w in (self.cmd, self.slow, self.poller):
            w.start()
        self.slow.put("presets", lambda: self.api.get("/presets.json", timeout=5.0))
        self.slow.put("network", lambda: self.api.get("/network.json", timeout=20.0))
        self.root.after(50, self._drain)
        self.root.after(200, self._toast_tick)
        self.root.after(30000, self._net_again)
        self.root.after(15000, self._judge_again)
        self.root.protocol("WM_DELETE_WINDOW", self.close)

    # ---- small helpers --------------------------------------------------------------------------------------------------------
    def set_text(self, key: str, widget, text: str, **kw) -> None:
        """Write a label only when its text changed (no flicker)."""
        if self.last_text.get(key) != text:
            self.last_text[key] = text
            widget.configure(text=text, **kw)

    def set_body(self, key: str, txt, lines: List[Tuple[str, str]], follow: bool = True) -> None:
        body = "\n".join(t for _, t in lines)
        if self.last_text.get(key) == body:
            return
        self.last_text[key] = body
        at_end = txt.yview()[1] >= 0.999
        txt.configure(state="normal")
        txt.delete("1.0", "end")
        for tag, t in lines:
            txt.insert("end", t + "\n", tag)
        txt.configure(state="disabled")
        if follow and at_end:
            txt.see("end")

    def press(self, label: str, path: str, body: dict, then: Optional[Callable[[dict], None]] = None, timeout: float = 60.0) -> None:
        """Every button: a toast at once, the POST on the command worker, the toast resolved with the answer."""
        tid = self.toasts.add(f"{label} · sent")
        self.cmd.put("press", lambda: (tid, label, self.api.post(path, body, timeout=timeout), then))

    # ---- layout ---------------------------------------------------------------------------------------------------------------
    def _build(self) -> None:
        tk, ttk, P = self.tk, self.ttk, self.pal
        r = self.root
        r.title("OptiPrime launcher (tk)")
        r.geometry("1440x900")
        r.minsize(1000, 640)
        r.configure(bg=P["bg"])
        st = ttk.Style(r)
        try:
            st.theme_use("clam")
        except tk.TclError:
            pass
        mono = ("Menlo", 11)
        self.mono = mono
        st.configure(".", background=P["bg"], foreground=P["ink"], fieldbackground=P["panel"], bordercolor=P["line"])
        st.configure("Card.TLabelframe", background=P["panel"], bordercolor=P["line"])
        st.configure("Card.TLabelframe.Label", background=P["panel"], foreground=P["dim"], font=("Helvetica", 11, "bold"))
        st.configure("Card.TFrame", background=P["panel"])
        st.configure("Card.TLabel", background=P["panel"], foreground=P["ink"])
        st.configure("Dim.TLabel", background=P["panel"], foreground=P["dim"])
        st.configure("Head.TLabel", background=P["panel"], foreground=P["ink"], font=("Helvetica", 15, "bold"))
        st.configure("TButton", background=P["panel2"], foreground=P["ink"], padding=(10, 5))
        st.map("TButton", background=[("active", P["line"]), ("disabled", P["panel2"])], foreground=[("disabled", P["dim"])])
        st.configure("Go.TButton", background=P["go"], foreground=P["goText"], font=("Helvetica", 14, "bold"), padding=(18, 10))
        st.map("Go.TButton", background=[("disabled", P["panel2"]), ("active", P["go"])], foreground=[("disabled", P["dim"])])
        st.configure("Stop.TButton", background=P["stop"], foreground=P["stopText"], font=("Helvetica", 12, "bold"), padding=(12, 8))
        st.configure("Seg.Toolbutton", background=P["panel2"], foreground=P["ink"], padding=(7, 3))
        st.map("Seg.Toolbutton", background=[("selected", P["acc"]), ("active", P["line"])],
               foreground=[("selected", "#ffffff"), ("disabled", P["dim"])])
        st.configure("Preset.Toolbutton", background=P["panel2"], foreground=P["ink"], padding=(14, 7), font=("Helvetica", 12, "bold"))
        st.map("Preset.Toolbutton", background=[("selected", P["acc"]), ("active", P["line"])], foreground=[("selected", "#ffffff")])
        st.configure("TNotebook", background=P["bg"])
        st.configure("TNotebook.Tab", padding=(14, 6))
        st.configure("Treeview", background=P["panel"], fieldbackground=P["panel"], foreground=P["ink"], font=mono, rowheight=22)

        head = ttk.Frame(r, style="Card.TFrame", padding=(16, 10))
        head.pack(side="top", fill="x")
        ttk.Label(head, text="OptiPrime launcher", style="Head.TLabel").pack(side="left")
        self.ver = ttk.Label(head, text="", style="Dim.TLabel")
        self.ver.pack(side="left", padx=10)
        self.backend = ttk.Label(head, text=f"backend {self.api.base}", style="Dim.TLabel")
        self.backend.pack(side="left", padx=10)
        self.theme_var = tk.StringVar(value=self.theme_word)
        cb = ttk.Combobox(head, textvariable=self.theme_var, values=["dark", "light", "hc-dark", "hc-light", "fun"], width=9, state="readonly")
        cb.pack(side="right")
        cb.bind("<<ComboboxSelected>>", lambda e: self._theme_pick())
        ttk.Label(head, text="pages' theme", style="Dim.TLabel").pack(side="right", padx=6)
        self.state_lbl = tk.Label(head, text="…", bg=P["panel"], fg=P["dim"], font=("Helvetica", 13, "bold"), padx=10, pady=2,
                                  highlightthickness=2, highlightbackground=P["line"])
        self.state_lbl.pack(side="right", padx=14)

        # the banner (a launch still running, not adopted)
        self.banner = tk.Frame(r, bg=P["panel"], highlightthickness=2, highlightbackground=P["warn"], padx=12, pady=8)
        self.ban_text = tk.Label(self.banner, text="", bg=P["panel"], fg=P["warn"], font=("Helvetica", 12, "bold"), justify="left",
                                 wraplength=1300, anchor="w")
        self.ban_text.pack(fill="x")
        bb = tk.Frame(self.banner, bg=P["panel"])
        bb.pack(fill="x", pady=(6, 0))
        self.btn_reconnect = ttk.Button(bb, text="RECONNECT", style="Go.TButton", command=self._adopt)
        self.btn_reconnect.pack(side="left")
        self.ban_word = tk.StringVar()
        ttk.Entry(bb, textvariable=self.ban_word, width=12).pack(side="left", padx=(16, 4))
        ttk.Label(bb, text="← type stop", style="Dim.TLabel").pack(side="left")
        ttk.Button(bb, text="STOP ALL", style="Stop.TButton", command=lambda: self._stop(self.ban_word)).pack(side="left", padx=8)
        ttk.Button(bb, text="STOP, then start new", command=lambda: self._stop(self.ban_word)).pack(side="left")

        body = ttk.Panedwindow(r, orient="horizontal")
        body.pack(fill="both", expand=True, padx=12, pady=10)
        # left: the form, scrollable
        left = ttk.Frame(body)
        body.add(left, weight=0)
        cv = tk.Canvas(left, bg=P["bg"], highlightthickness=0, width=510)
        sb = ttk.Scrollbar(left, orient="vertical", command=cv.yview)
        self.form = ttk.Frame(cv)
        self.form.bind("<Configure>", lambda e: cv.configure(scrollregion=cv.bbox("all")))
        form_win = cv.create_window((0, 0), window=self.form, anchor="nw", width=500)
        cv.configure(yscrollcommand=sb.set)

        def _form_width(e):                                        # v0.1.90.9 (Dmitrii: "click and drag to expand or shrink"): the form
            w = max(360, e.width - 4)                              # follows the sash - the cards, the preset buttons and the texts widen
            cv.itemconfigure(form_win, width=w)
            try:
                self.about.configure(wraplength=max(300, w - 40))
            except Exception:
                pass
        cv.bind("<Configure>", _form_width)
        cv.pack(side="left", fill="both", expand=True)
        sb.pack(side="right", fill="y")
        cv.bind_all("<MouseWheel>", lambda e: cv.yview_scroll(-1 * (e.delta if abs(e.delta) < 10 else e.delta // 120), "units")
                    if str(e.widget).startswith(str(self.form)) or e.widget is cv else None)
        self.ro_tag = ttk.Label(self.form, text="showing the running launch (read-only) - STOP it to start another", foreground=P["warn"])
        pc = ttk.Labelframe(self.form, text="PRESET", style="Card.TLabelframe", padding=10)
        pc.pack(fill="x", pady=(0, 10))
        self.preset_box = ttk.Frame(pc, style="Card.TFrame")
        self.preset_box.pack(fill="x")
        self.preset_var = tk.StringVar()
        self.about = ttk.Label(pc, text="loading the presets…", style="Dim.TLabel", wraplength=420, justify="left")
        self.about.pack(fill="x", pady=(6, 0))
        for c in CARDS:
            f = ttk.Labelframe(self.form, text=c.upper(), style="Card.TLabelframe", padding=10)
            f.pack(fill="x", pady=(0, 10))
            f.columnconfigure(1, weight=1)
            self.cards[c] = f

        # right: the plan on top, the tabs below - v0.1.90.9: a vertical sash between them (drag to give either more room)
        right = ttk.Panedwindow(body, orient="vertical")
        body.add(right, weight=1)
        pl = ttk.Labelframe(right, text="THE PLAN", style="Card.TLabelframe", padding=10)
        right.add(pl, weight=1)
        self.plan_txt = tk.Text(pl, height=14, bg=P["panel"], fg=P["ink"], font=mono, wrap="word", relief="flat", highlightthickness=0)
        self._tags(self.plan_txt)
        self.plan_txt.pack(fill="both", expand=True)
        self.plan_txt.configure(state="disabled")
        lb = ttk.Frame(pl, style="Card.TFrame")
        lb.pack(fill="x", pady=(8, 0))
        self.btn_launch = ttk.Button(lb, text="LAUNCH", style="Go.TButton", command=self._launch)
        self.btn_launch.pack(side="left")
        self.launch_why = ttk.Label(lb, text="", style="Dim.TLabel")
        self.launch_why.pack(side="left", padx=10)
        ttk.Button(lb, text="STOP ALL", style="Stop.TButton", command=lambda: self._stop(self.stop_word)).pack(side="right")
        ttk.Label(lb, text="type stop →", style="Dim.TLabel").pack(side="right", padx=4)
        self.stop_word = tk.StringVar()
        ttk.Entry(lb, textvariable=self.stop_word, width=12).pack(side="right")

        jb = ttk.Frame(pl, style="Card.TFrame")
        jb.pack(fill="x", pady=(8, 0))
        self.judge_lbl = tk.Label(jb, text="judge: …", bg=P["panel"], fg=P["ink"], font=("Helvetica", 12, "bold"), anchor="w", justify="left")
        self.judge_lbl.pack(fill="x")
        self.judge_warn = tk.Label(jb, text="", bg=P["panel"], fg=P["warn"], font=("Helvetica", 12, "bold"), anchor="w", justify="left",
                                   wraplength=900)
        self.judge_warn.pack(fill="x")
        self.reach_lbl = tk.Label(jb, text="reachable: …", bg=P["panel"], fg=P["dim"], font=("Helvetica", 11), anchor="w", justify="left")
        self.reach_lbl.pack(fill="x")                              # v0.1.86: AGX · USV-NX · UAV-NX ●/○ + the reachable RoboCommand
        nb = ttk.Notebook(right)
        right.add(nb, weight=1)
        self.nb = nb
        # PROGRESS
        pg = ttk.Frame(nb, style="Card.TFrame", padding=8)
        nb.add(pg, text="PROGRESS")
        cbar = ttk.Frame(pg, style="Card.TFrame")
        cbar.pack(fill="x", pady=(0, 6))
        ttk.Label(cbar, text="OCS console", style="Dim.TLabel").pack(side="left")
        self.console_var = tk.StringVar(value="tk")
        for v, lab in (("tk", "tk window (opens when the OCS comes up)"), ("web", "web page (browser)")):
            ttk.Radiobutton(cbar, text=lab, value=v, variable=self.console_var, style="Seg.Toolbutton").pack(side="left", padx=2)
        ttk.Button(cbar, text="open the OCS console", command=self.open_console).pack(side="left", padx=10)
        self.links_box = ttk.Frame(pg, style="Card.TFrame")
        self.links_box.pack(fill="x")
        self.prog_txt = tk.Text(pg, height=6, bg=P["panel"], fg=P["ink"], font=mono, relief="flat", highlightthickness=0, wrap="none")
        self._tags(self.prog_txt)
        self.prog_txt.pack(fill="x", pady=(6, 6))
        self.log_txt = tk.Text(pg, bg=P["bg"], fg=P["ink"], font=mono, relief="flat", highlightthickness=1, highlightbackground=P["line"],
                               wrap="word")
        self._tags(self.log_txt)
        self.log_txt.pack(fill="both", expand=True)
        self.link_widgets: Dict[str, Any] = {}
        # RUNNING
        rn = ttk.Frame(nb, style="Card.TFrame", padding=8)
        nb.add(rn, text="RUNNING")
        self.hosts_box = ttk.Frame(rn, style="Card.TFrame")
        self.hosts_box.pack(fill="both", expand=True)
        # NETWORK
        nt = ttk.Frame(nb, style="Card.TFrame", padding=8)
        nb.add(nt, text="NETWORK")
        bar = ttk.Frame(nt, style="Card.TFrame")
        bar.pack(fill="x")
        ttk.Button(bar, text="↻ check", command=self._net_check).pack(side="left")
        self.btn_ssh = ttk.Button(bar, text="ssh ↗", command=lambda: self._ssh("open"), state="disabled")
        self.btn_ssh.pack(side="left", padx=6)
        self.btn_key = ttk.Button(bar, text="copy my key ↗", command=lambda: self._ssh("copy-id"), state="disabled")
        self.btn_key.pack(side="left")
        self.btn_obs = ttk.Button(bar, text="observer ▶", command=self._observer, state="disabled")
        self.btn_obs.pack(side="left", padx=6)
        self.net_sel = ttk.Label(bar, text="pick a row (NX / AGX / UAV) for ssh / the observer", style="Dim.TLabel")
        self.net_sel.pack(side="left", padx=8)
        cols = ("role", "key", "lan", "ts", "now")
        self.net_tree = ttk.Treeview(nt, columns=cols, show="headings", height=12)
        for c, w in zip(cols, (200, 160, 150, 150, 330)):
            self.net_tree.heading(c, text=c)
            self.net_tree.column(c, width=w, anchor="w")
        self.net_tree.pack(fill="both", expand=True, pady=(6, 0))
        self.net_tree.bind("<<TreeviewSelect>>", lambda e: self._net_pick())
        self.net_tree.tag_configure("up", foreground=P["ok"])
        self.net_tree.tag_configure("down", foreground=P["dim"])
        self.net_data: List[dict] = []
        # ROBOCOMMAND
        rc = ttk.Frame(nb, style="Card.TFrame", padding=8)
        nb.add(rc, text="ROBOCOMMAND")
        rb = ttk.Frame(rc, style="Card.TFrame")
        rb.pack(fill="x")
        ttk.Button(rb, text="↻ check", command=self._course_check).pack(side="left")
        self.crs_head = ttk.Label(rb, text="press ↻ check (one short MQTT look at the RxCourse when the OCS is not running)", style="Dim.TLabel")
        self.crs_head.pack(side="left", padx=8)
        ccols = ("course", "RoboCommand", "here", "broker", "RxCourse")
        self.crs_tree = ttk.Treeview(rc, columns=ccols, show="headings", height=6)
        for c, w in zip(ccols, (110, 200, 180, 110, 340)):
            self.crs_tree.heading(c, text=c)
            self.crs_tree.column(c, width=w, anchor="w")
        self.crs_tree.pack(fill="both", expand=True, pady=(6, 0))
        self.crs_tree.tag_configure("on", foreground=P["ok"])
        # CONSOLE
        co = ttk.Frame(nb, style="Card.TFrame", padding=8)
        nb.add(co, text="CONSOLE")
        self.con_var = tk.StringVar()
        self.con_keys: List[Tuple[str, str]] = []
        self.con_cb = ttk.Combobox(co, textvariable=self.con_var, state="readonly", width=80, values=[])
        self.con_cb.pack(anchor="w")
        self.con_cb.bind("<<ComboboxSelected>>", lambda e: self._con_pick())
        ttk.Label(co, text="a started window's last lines (tmux, read only - type in the window itself)", style="Dim.TLabel").pack(anchor="w")
        self.con_txt = tk.Text(co, bg=P["bg"], fg=P["ink"], font=mono, relief="flat", highlightthickness=1, highlightbackground=P["line"])
        self.con_txt.pack(fill="both", expand=True, pady=(6, 0))

        self.toast_layer = tk.Frame(r, bg=P["bg"])          # placed top-right over everything

    def _tags(self, t) -> None:
        P = self.pal
        t.tag_configure("err", foreground=P["bad"])
        t.tag_configure("bad", foreground=P["bad"])
        t.tag_configure("ok", foreground=P["ok"])
        t.tag_configure("up", foreground=P["ok"])
        t.tag_configure("died", foreground=P["bad"])
        t.tag_configure("warn", foreground=P["warn"])
        t.tag_configure("note", foreground=P["warn"])
        t.tag_configure("step", foreground=P["acc"], font=(self.mono[0], self.mono[1], "bold"))
        t.tag_configure("cmd", foreground=P["dim"], lmargin1=24, lmargin2=24)
        t.tag_configure("dim", foreground=P["dim"])

    # ---- the form -------------------------------------------------------------------------------------------------------------
    def _build_form(self) -> None:
        tk, ttk = self.tk, self.ttk
        for w in self.preset_box.winfo_children():
            w.destroy()
        shown = sorted(((k, p) for k, p in self.presets.items() if not p.get("hidden") or k == self.preset_var.get()),
                       key=lambda kp: (not str(kp[1].get("title") or "")[:1].isdigit(), str(kp[1].get("title") or kp[0])))   # v0.1.90.3
        for i, (k, p) in enumerate(shown):
            b = ttk.Radiobutton(self.preset_box, text=p.get("title") or k, value=k, variable=self.preset_var, style="Preset.Toolbutton",
                                command=lambda k=k: self.choose(k))
            b.grid(row=i // 3, column=i % 3, padx=2, pady=2, sticky="ew")      # v0.1.90.9: three per row (five in one row were cramped)
            self.preset_box.columnconfigure(i % 3, weight=1, uniform="preset")
            self.form_widgets.append(b)
            Tooltip(b, p.get("about") or "", self.pal)                        # v0.1.94.3: the preset's about on hover
        counts = {c: 0 for c in CARDS}
        for card, key, label, kind, _ in ROWS:
            if kind == "seg" and not self.fields.get(key):
                continue
            f = self.cards[card]
            rowi = counts[card]
            counts[card] += 1
            lab = ttk.Label(f, text=label, style="Dim.TLabel", width=14)
            if ROW_TIPS.get(key):
                Tooltip(lab, ROW_TIPS[key], self.pal)                         # v0.1.94.3: the row's tip on hover
            if kind in ("seg", "bool"):
                box = ttk.Frame(f, style="Card.TFrame")
                v = tk.StringVar()
                choices = ["true", "false"] if kind == "bool" else [str(c) for c in self.fields[key]]
                for j, c in enumerate(choices):
                    b = ttk.Radiobutton(box, text=seg_label(key, c == "true" if kind == "bool" else c), value=c, variable=v,
                                        style="Seg.Toolbutton", command=lambda key=key, v=v, kind=kind: self._seg(key, v, kind))
                    b.grid(row=j // 4, column=j % 4, padx=1, pady=1, sticky="ew")
                    self.form_widgets.append(b)
                    Tooltip(b, choice_tip(key, c == "true" if kind == "bool" else c), self.pal)   # v0.1.94.3: this choice's tip
            else:
                v = tk.StringVar()
                box = ttk.Entry(f, textvariable=v)
                v.trace_add("write", lambda *_a, key=key, v=v, kind=kind: self._entry(key, v, kind))
                self.form_widgets.append(box)
            lab.grid(row=rowi, column=0, sticky="w", pady=2)
            box.grid(row=rowi, column=1, sticky="ew", pady=2)
            self.vars[key] = v
            self.rows[key] = (lab, box)
        self.ph = ttk.Label(self.cards["network"], text="", style="Dim.TLabel", wraplength=420, justify="left")
        ttk.Label(self.cards["rates and logging"], style="Dim.TLabel", wraplength=420, justify="left",
                  text="a changed rate goes into a generated copy of the profile under logs/launch/ - the profiles stay as they are"
                  ).grid(row=99, column=0, columnspan=2, sticky="w", pady=(4, 0))

    def choose(self, name: str) -> None:
        self.cur = name
        self.preset_var.set(name)
        self.cfg = preset_cfg(self.presets, name)
        self.render()

    def _seg(self, key: str, v, kind: str) -> None:
        if self.suppress:
            return
        self.cfg[key] = (v.get() == "true") if kind == "bool" else v.get()
        if key == "uav" and self.cfg.get("uav") == "on" and not self.cfg.get("uav_buoys"):
            self.cfg["uav_buoys"] = "broker"
        self.render()

    def _entry(self, key: str, v, kind: str) -> None:
        if self.suppress:
            return
        s = v.get().strip()
        if kind == "num":
            try:
                self.cfg[key] = float(s)
            except ValueError:
                self.cfg[key] = None
        else:
            self.cfg[key] = s
        self._plan_soon()

    def render(self) -> None:
        """cfg -> the widgets (values, which rows show), then a new plan."""
        self.suppress = True
        try:
            for key, v in self.vars.items():
                val = self.cfg.get(key)
                want = ("true" if val else "false") if key in BOOL_KEYS else ("" if val is None else
                                                                             (f"{val:g}" if isinstance(val, float) else str(val)))
                if v.get() != want:
                    v.set(want)
        finally:
            self.suppress = False
        vis = visible(self.cfg)
        for key, (lab, box) in self.rows.items():
            if vis.get(key):
                lab.grid()
                box.grid()
            else:
                lab.grid_remove()
                box.grid_remove()
        sim = self.cfg.get("mode") == "sim"
        for c in ("the boat", "network"):
            if sim:
                self.cards[c].pack_forget()
            elif not self.cards[c].winfo_ismapped():
                self.cards[c].pack(fill="x", pady=(0, 10), before=self.cards["rates and logging"])
        self.set_text("about", self.about, (self.presets.get(self.cur) or {}).get("about", ""))
        self._plan_soon()

    def _plan_soon(self) -> None:
        if getattr(self, "_plan_after", None):
            self.root.after_cancel(self._plan_after)
        self._plan_after = self.root.after(150, self._plan)

    def _judge_check(self) -> None:
        """The judge line: the target from the network files + a TCP look at it and at the venue (the slow worker; never on Tk)."""
        cfg = dict(self.cfg)

        def job():
            from . import launcher as LB
            env = LB.read_net(cfg.get("net") if cfg.get("net") in ("lan", "ts") else "lan")
            t = judge_target(cfg, env)
            up = LB.tcp_up(t["host"], int(t["port"]), 0.5) if t["host"] else None
            vh, vp = env.get("VENUE_RC_HOST"), env.get("VENUE_RC_PORT")
            vup = LB.tcp_up(vh, int(vp), 0.5) if vh and vp else None
            return {"t": t, "up": up, "venue_up": vup, "env": {k: env.get(k) for k in ("VENUE_RC_HOST", "VENUE_RC_PORT")}, "cfg": cfg}
        self.slow.put("judge", job)

    def _on_judge(self, payload) -> None:
        r = payload[1]
        if not isinstance(r, dict) or "t" not in r:
            return
        t, up = r["t"], r["up"]
        P = self.pal
        txt = f"heartbeats + reports go to: {t['where']}" + ("" if up is None else ("  ● answers" if up else "  ○ NO ANSWER"))
        self.set_text("judge", self.judge_lbl, txt, fg=P["bad"] if up is False else P["ink"])
        self.set_text("judge_warn", self.judge_warn, judge_warning(r["cfg"], t, r["venue_up"], r["env"]))

    def _judge_again(self) -> None:
        if not self.closing:
            self._judge_check()
            self.root.after(15000, self._judge_again)

    def _plan(self) -> None:
        self._plan_after = None
        self._judge_check()
        body = {"cfg": dict(self.cfg, theme=self.theme_word)}
        self.cmd.put("plan", lambda: self.api.post("/plan", body, timeout=30.0))

    def _readonly(self, ro: bool) -> None:
        st = "disabled" if ro else "!disabled"
        for w in self.form_widgets:
            try:
                w.state([st])
            except Exception:
                pass
        if ro:
            self.ro_tag.pack(fill="x", pady=(0, 6), before=self.form.winfo_children()[1])
        else:
            self.ro_tag.pack_forget()

    # ---- the buttons ----------------------------------------------------------------------------------------------------------
    def _launch(self) -> None:
        body = {"cfg": dict({"preset": self.cur}, **self.cfg, theme=self.theme_word)}
        self.press("LAUNCH", "/launch", body, then=lambda r: (self.poller.kick("status"), self.poller.kick("session")))

    def _stop(self, var) -> None:
        word = var.get()
        var.set("")
        if word.strip().lower() != "stop":
            self.toasts.add("STOP ALL ✗ type stop in the box first", "bad")
            return
        self.press("STOP ALL", "/stop", {"confirm": word}, then=lambda r: (self.poller.kick("status"), self.poller.kick("session")),
                   timeout=180.0)

    def _adopt(self) -> None:
        def then(r):
            if r.get("ok"):
                self.cfg = dict(r.get("cfg") or {})
                p = self.cfg.pop("preset", None)
                self.cur = p if p in self.presets else self.cur
                self.preset_var.set(self.cur)
                for k in ("title", "about"):
                    self.cfg.pop(k, None)
                self.loaded_rec = True
                self.render()
            self.poller.kick("session")
        self.press("RECONNECT", "/adopt", {}, then=then)

    def _restart(self, host: str) -> None:
        self.press(f"restart {host}", "/restart", {"host": host}, then=lambda r: self.poller.kick("session"), timeout=180.0)

    def _theme_pick(self) -> None:
        self.theme_word = self.theme_var.get()
        self.press(f"pages' theme {self.theme_word}", "/prefs", {"theme": self.theme_word})
        self._plan_soon()

    def _net_pick(self) -> None:
        sel = self.net_tree.selection()
        r = self.net_data[int(sel[0])] if sel and sel[0].isdigit() and int(sel[0]) < len(self.net_data) else None
        self._net_row = r
        ssh_ok = bool(r and r["ssh"] and r["up"])
        self.btn_ssh.state(["!disabled"] if ssh_ok else ["disabled"])
        self.btn_key.state(["!disabled"] if r and r["ssh"] and r["auth"] == "password" else ["disabled"])
        ob = r and r.get("observer")
        self.btn_obs.state(["!disabled"] if ob else ["disabled"])
        if ob:
            self.btn_obs.configure(text="observer ■" if ob.get("state") == "up" else "observer ▶")
        txt = "pick a row (NX / AGX / UAV) for ssh / the observer"
        if r:
            txt = f"{r['key']}: " + (f"ssh {r['ssh']} · {r['auth'] or '?'}" if r["ssh"] else "no ssh target") + \
                  (f" · observer {ob.get('state')} ({ob.get('text')})" if ob else "") + (f" · USV window PIN {r['pin']}" if r.get("pin") else "")
        self.net_sel.configure(text=txt)

    def _ssh(self, action: str) -> None:
        r = getattr(self, "_net_row", None)
        if r and r["ssh"]:
            self.press(f"ssh {r['ssh']} {action}", "/ssh", {"which": r["ssh"], "action": action})

    def _observer(self) -> None:
        r = getattr(self, "_net_row", None)
        ob = (r or {}).get("observer")
        if not ob:
            return
        act = "stop" if ob.get("state") == "up" else "start"
        self.press(f"observer {act}", "/observer", {"action": act}, then=lambda res: self._net_check(), timeout=60.0)

    def _browse(self, url: str, name: str) -> None:
        webbrowser.open(url)
        self.toasts.add(f"opened {name} in the browser", "info")

    def open_console(self, auto: bool = False) -> None:
        """The OCS console: the tk window (scripts/run_ocs_tk.sh, detached, one per Mac) - or the page when the choice is web."""
        if self.console_var.get() == "web":
            u = next((l["url"] for l in self.sess.get("links") or [] if l["name"] == OCS_LINK), None)
            if u:
                self._browse(u, "the OCS page")
            else:
                self.toasts.add("OCS page: no link yet (LAUNCH first)", "bad")
            return
        label = "OCS console (tk)" + (" - the OCS came up" if auto else "")
        tid = self.toasts.add(f"{label} · sent")
        argv, log = console_argv(KIT, self.cfg), str(KIT / "logs" / "ocs" / "ocs_tk.log")
        self.cmd.put("press", lambda: (tid, label, self.spawn(argv, log), None))

    def _net_check(self) -> None:
        self.slow.put("network", lambda: self.api.get("/network.json", timeout=20.0))

    def _net_again(self) -> None:
        if not self.closing:
            self._net_check()
            self.root.after(30000, self._net_again)

    def _course_check(self) -> None:
        self.crs_head.configure(text="checking…")
        self.slow.put("course", lambda: self.api.get("/course_check.json", timeout=20.0))

    def _con_pick(self) -> None:
        lab = self.con_var.get()
        key = next((k for k, l in self.con_keys if l == lab), "")
        self.poller.con_key = key
        self.poller.kick("console")

    # ---- results from the threads (main thread only) --------------------------------------------------------------------------
    def _drain(self) -> None:
        try:
            for _ in range(200):
                kind, payload = self.inbox.get_nowait()
                try:
                    getattr(self, "_on_" + kind)(payload)
                except Exception as exc:                      # a bad answer never kills the window
                    self.toasts.add(f"{kind}: {exc.__class__.__name__} {exc}", "bad")
        except queue.Empty:
            pass
        if not self.closing:
            self.root.after(50, self._drain)

    def _on_reach(self, payload) -> None:
        self.set_text("reach", self.reach_lbl, reach_line(payload) + " · " + tiers_line(getattr(self, "cfg", {}) or {}))

    def _on_presets(self, payload) -> None:
        d = payload[1]
        if not d:
            self.about.configure(text="no answer from the launcher backend - retrying")
            self.root.after(2000, lambda: self.slow.put("presets", lambda: self.api.get("/presets.json", timeout=5.0)))
            return
        self.presets, self.fields, self.placeholders = d.get("presets") or {}, d.get("fields") or {}, d.get("placeholders") or {}
        from optiprimex import __version__ as here
        self.ver.configure(text=f"v{d.get('version', '?')}" + ("" if d.get("version") == here else f" (this window v{here})"))
        self._build_form()
        self.choose("sim" if "sim" in self.presets else next(iter(self.presets), ""))

    def _on_plan(self, payload) -> None:
        tag, r = payload
        if tag != self.cmd.plan_seq:
            return
        self.plan_ok = bool(r.get("ok"))
        lines = plan_lines(r) if (r.get("plan") or r.get("errors")) else [("err", r.get("why") or "no plan", None)]
        key = "\n".join(t for _, t, _ in lines)
        if self.last_text.get("plan") != key:
            self.last_text["plan"] = key
            t = self.plan_txt
            t.configure(state="normal")
            t.delete("1.0", "end")
            for tg, text, cmd in lines:
                if cmd is not None:
                    b = self.tk.Button(t, text="⧉", padx=3, pady=0, relief="flat", cursor="hand2", command=lambda c=cmd: self._copy(c))
                    t.window_create("end", window=b)
                    t.insert("end", " ")
                t.insert("end", text + "\n", tg)
            t.configure(state="disabled")
        self._launch_state()

    def _copy(self, text: str) -> None:
        self.root.clipboard_clear()
        self.root.clipboard_append(text)
        self.toasts.add("copied the command", "ok")

    def _launch_state(self) -> None:
        ok = self.plan_ok and not self.sess["up"]
        self.btn_launch.state(["!disabled"] if ok else ["disabled"])
        why = "" if ok else ("something is running: RECONNECT or STOP first" if self.sess["up"] else "the plan has errors (red, above)")
        self.set_text("launch_why", self.launch_why, why)

    def _on_press(self, payload) -> None:
        _, (tid, label, r, then) = payload
        ok = bool(r.get("ok"))
        extra = r.get("text") or r.get("cmd") or ""
        msg = f"{label}" + (f" - {extra}" if ok and extra else "") if ok else f"{label} - {r.get('why') or r.get('text') or 'refused'}"
        if r.get("dry"):
            msg += " (dry run)"
        self.toasts.resolve(tid, ok, msg)
        if then:
            then(r)

    def _on_status(self, s) -> None:
        v = status_view(s)
        P = self.pal
        col = {"bad": P["bad"], "ok": P["ok"], "run": P["run"], "idle": P["dim"]}[v["tone"]]
        if self.last_text.get("state") != v["state"]:
            self.last_text["state"] = v["state"]
            self.state_lbl.configure(text=v["state"], fg=col, highlightbackground=col)
            st = (s or {}).get("state")
            if self.last_state is not None and st != self.last_state and st in ("done", "failed", "running", "stopped"):
                self.toasts.add(f"launcher: {st}", "ok" if st == "done" else "bad" if st == "failed" else "info")
            self.last_state = st
        self.set_body("log", self.log_txt, v["log"])

    def _on_session(self, d) -> None:
        v = session_view(d)
        was = self.sess
        self.sess = v
        if v["banner"]:
            self.set_text("ban", self.ban_text, v["banner_text"])
            if not self.banner.winfo_ismapped():
                self.banner.pack(side="top", fill="x", padx=16, pady=(10, 0), after=self.ban_anchor())
            if v["reconnect"]:
                self.btn_reconnect.pack(side="left")
            else:
                self.btn_reconnect.pack_forget()
        elif self.banner.winfo_ismapped():
            self.banner.pack_forget()
        if v["readonly"] != was.get("readonly"):
            self._readonly(v["readonly"])
        if v["adopted"] and v["record_cfg"] and not self.loaded_rec:
            self.loaded_rec = True
            self.cfg = {k: x for k, x in v["record_cfg"].items() if k not in ("title", "about")}
            p = self.cfg.pop("preset", None)
            if p in self.presets:
                self.cur = p
                self.preset_var.set(p)
            self.render()
        if not v["adopted"]:
            self.loaded_rec = False
        self._launch_state()
        self._links(v["links"])
        ocs = next((l for l in v["links"] if l["name"] == OCS_LINK), None)
        if ocs is not None:
            if auto_open(self.ocs_up, ocs["up"], self.console_var.get()):
                self.open_console(auto=True)
            self.ocs_up = ocs["up"]
        self.set_body("prog", self.prog_txt, [(p["state"] if p["state"] in ("up", "died") else "dim",
                                               f"{str(p['host']):<5} {p['text']:<70} {p['word']}") for p in v["progress"]] or
                      [("dim", "nothing launched from here yet")], follow=False)
        self._hosts(v["hosts"])

    def ban_anchor(self):
        return self.root.winfo_children()[0]

    def _links(self, links: List[dict]) -> None:
        key = json.dumps([(l["name"], l["url"], l["up"], l["pin"]) for l in links])
        if self.last_text.get("links") == key:
            return
        self.last_text["links"] = key
        for w in self.links_box.winfo_children():
            w.destroy()
        P = self.pal
        for l in links:
            ocs = l["name"] == OCS_LINK
            name = "OCS console" if ocs else str(l["name"])
            t = ("● " if l["up"] else "○ ") + name + (" up" if l["up"] else " down") + (f"  PIN {l['pin']}" if l["pin"] else "")
            b = self.tk.Label(self.links_box, text=t, fg=P["ok"] if l["up"] else P["dim"], bg=P["panel"], cursor="hand2",
                              font=("Helvetica", 12, "underline" if l["up"] else "normal"), padx=6)
            if ocs:                                            # v0.1.84.1: the tk console (or the page, when the choice is web)
                b.bind("<Button-1>", lambda e: self.open_console())
                b.pack(side="left")
                w = self.tk.Label(self.links_box, text="web ↗", fg=P["acc"], bg=P["panel"], cursor="hand2", font=("Helvetica", 11), padx=2)
                w.bind("<Button-1>", lambda e, u=l["url"]: self._browse(u, "the OCS page"))
                w.pack(side="left", padx=(0, 10))
                continue
            b.bind("<Button-1>", lambda e, u=l["url"], n=l["name"]: self._browse(u, n))
            b.pack(side="left")

    def _hosts(self, hosts: List[dict]) -> None:
        key = json.dumps(hosts)
        if self.last_text.get("hosts") == key:
            return
        self.last_text["hosts"] = key
        for w in self.hosts_box.winfo_children():
            w.destroy()
        P, ttk = self.pal, self.ttk
        if not hosts:
            ttk.Label(self.hosts_box, text="nothing running from a launch (checked every 3 s)", style="Dim.TLabel").pack(anchor="w")
        for i, h in enumerate(hosts):
            self.tk.Label(self.hosts_box, text=h["host"], bg=P["panel"], fg=P["ink"], font=("Helvetica", 12, "bold"), width=8,
                          anchor="w").grid(row=i, column=0, sticky="w", pady=2)
            self.tk.Label(self.hosts_box, text=h["text"], bg=P["panel"], fg=P["ok"] if h["ok"] else P["bad"], font=self.mono,
                          anchor="w").grid(row=i, column=1, sticky="w")
            if h["restart"]:
                ttk.Button(self.hosts_box, text="restart this host", command=lambda x=h["host"]: self._restart(x)).grid(row=i, column=2, padx=8)

    def _on_prefs(self, d) -> None:
        t = ((d or {}).get("prefs") or {}).get("theme")
        if t and t != self.theme_word:
            self.theme_word = t
            self.theme_var.set(t)

    def _on_network(self, payload) -> None:
        rows = net_rows(payload[1])
        self.net_data = rows
        key = json.dumps(rows, default=str)
        if self.last_text.get("net") == key:
            return
        self.last_text["net"] = key
        sel = self.net_tree.selection()
        self.net_tree.delete(*self.net_tree.get_children())
        for i, r in enumerate(rows):
            self.net_tree.insert("", "end", iid=str(i), values=(r["role"], r["key"], r["lan"], r["ts"], r["now"]),
                                 tags=("up" if r["up"] else "down",))
        if sel and sel[0] in self.net_tree.get_children():
            self.net_tree.selection_set(sel)
        self._net_pick()

    def _on_course(self, payload) -> None:
        head, rows = course_rows(payload[1])
        self.crs_head.configure(text=head)
        self.crs_tree.delete(*self.crs_tree.get_children())
        for r in rows:
            self.crs_tree.insert("", "end", values=r, tags=("on",) if r[2].startswith("●") or r[3] == "up" else ())

    def _on_console(self, d) -> None:
        if not d:
            return
        wins = [(w["key"], w["label"]) for w in d.get("windows") or []]
        if wins != self.con_keys:
            self.con_keys = wins
            self.con_cb.configure(values=[l for _, l in wins])
        if self.poller.con_key:
            lines = d.get("lines") if d.get("ok") else None
            self.set_body("con", self.con_txt, [("", x) for x in (lines or [])] if lines is not None else
                          [("", "not answering (the window is gone, or the host)")])

    # ---- the toasts (drawn in the window's top-right corner) ------------------------------------------------------------------
    def _toast_tick(self) -> None:
        if self.closing:
            return
        vis, more = self.toasts.tick()
        key = json.dumps([(t["id"], t["state"], t["text"]) for t in vis] + [more])
        if self.last_text.get("toasts") != key:
            self.last_text["toasts"] = key
            for w in self.toast_frames:
                w.destroy()
            self.toast_frames = []
            for i, t in enumerate(vis):
                text = f"{TOAST_MARK[t['state']]} {t['text']}" + (" - no answer yet" if t["state"] == "late" else "")
                lab = self.tk.Label(self.root, text=text, bg=TOAST_BG[t["state"]], fg="#ffffff", font=("Helvetica", 13, "bold"),
                                    padx=14, pady=8, wraplength=520, justify="left", cursor="hand2")
                lab.bind("<Button-1>", lambda e, tid=t["id"]: self.toasts.dismiss(tid))
                lab.place(relx=1.0, x=-16, y=16 + i * 48, anchor="ne")
                lab.lift()
                self.toast_frames.append(lab)
            if more:
                lab = self.tk.Label(self.root, text=f"+{more}", bg="#6b7280", fg="#ffffff", padx=10, pady=4)
                lab.place(relx=1.0, x=-16, y=16 + len(vis) * 48, anchor="ne")
                self.toast_frames.append(lab)
        self.root.after(200, self._toast_tick)

    def close(self) -> None:
        if self.sess.get("up") is False and self.last_state == "running":
            pass
        self.closing = True
        for w in (self.cmd, self.slow, self.poller):
            w.stop_ev.set()
        self.root.destroy()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--url", default="http://127.0.0.1:8093", help="the launcher backend (scripts/launch_tk.sh starts its own on :8093)")
    ap.add_argument("--theme", choices=("light", "dark"), default="light")
    a = ap.parse_args(argv)
    import tkinter as tk
    root = tk.Tk()
    App(root, Api(a.url), theme=a.theme)
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
