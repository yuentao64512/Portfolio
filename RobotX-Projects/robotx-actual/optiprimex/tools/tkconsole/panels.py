"""The console's Phase-2 tabs (Dmitrii 2026-11-12 night: "the elements of GUI we postponed"): JUDGE · RUN VIEW (big numerals) · SYSTEM ·
HELP. Pure view functions first (tested without Tk), from the same /state.json and /meta.json; the Tk builders below them only draw.
Read-only except SYSTEM's actions, which go through POST /system with the page's confirm rule (the dangerous ones need their word typed)."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

Row = Tuple[str, str, str]          # (label, text, level)


def _ago(x) -> str:
    return "—" if not isinstance(x, (int, float)) else (f"{x:.0f} s ago" if x < 120 else f"{x / 60:.0f} min ago")


# ---- JUDGE ------------------------------------------------------------------------------------------------------------------
def judge_rows(s: Optional[dict]) -> Dict[str, List[Row]]:
    """{"link": [...], "run": [...], "vehicles": [...], "reports": [...]} - what RoboCommand sees of us, from the OCS's own view."""
    s = s or {}
    o, j = s.get("ocs") or {}, s.get("judge") or {}
    jv, rep = o.get("judge_view") or {}, o.get("reporting") or {}
    out: Dict[str, List[Row]] = {"link": [], "run": [], "vehicles": [], "reports": []}
    c = jv.get("course") or {}
    out["link"] = [("JUDGE", f"{jv.get('name') or o.get('judge') or '?'} · {'link up' if j.get('connected') else 'link DOWN'}",
                    "ok" if j.get("connected") else "bad"),
                   ("COURSE", (f"{c.get('course_id')} · {c.get('points')} corners · pinger {c.get('pinger_hz')} Hz · "
                               f"{'valid' if c.get('valid') else 'NOT valid'} · {_ago(c.get('age_s'))}") if c else "no course received",
                    "ok" if c.get("valid") else "warn"),
                   ("WIRES", " · ".join(f"{k} {v}" for k, v in (j.get("wires") or {}).items()) or "—", "")]
    d, rs = j.get("declaration") or {}, jv.get("run_start") or {}
    r = o.get("r") or "?"
    out["run"] = [("STATE", f"{r} · {o.get('r_state') or ''}".strip(" ·"), "ok" if r == "R5" else ("warn" if r in ("R3", "R4") else "")),
                  ("DECLARED", (f"seq {o.get('declaration_seq')} · vehicles {', '.join(d.get('vehicles') or []) or '?'} · {_ago(d.get('ago_s'))}"
                                if o.get("declaration_seq") else "not declared"), "ok" if o.get("declaration_seq") else ""),
                  ("RUNSTART", (f"run {rs.get('run_id')} · for our declaration {rs.get('declaration_seq')} · "
                                f"{'accepted' if rs.get('accepted') else 'NOT ours / not accepted'} · {_ago(rs.get('age_s'))}") if rs else "—",
                   "ok" if rs.get("accepted") else ("bad" if rs else "")),
                  ("ANSWER", str(j.get("answer") or "—"), {"bad": "bad", "warn": "warn", "ok": "ok"}.get(j.get("answer_style"), ""))]
    tl = (s.get("timeline") or {}).get("events") or []
    if tl:
        out["run"].append(("TIMELINE", " → ".join(str(e.get("kind")) for e in tl[-6:]), ""))
    hb, seen = rep.get("hb") or {}, j.get("heartbeat_age") or {}
    for vid in sorted(set(hb) | set(seen)):
        h = hb.get(vid) or {}
        a = seen.get(vid)
        sending = bool(h) and not h.get("stopped")
        out["vehicles"].append((vid, (f"sending {str(h.get('state') or '?').replace('STATE_', '')} · #{h.get('n')} · last {h.get('age_s')} s"
                                      if sending else "not sending") + (f" · the judge saw it {a:.1f} s ago" if isinstance(a, (int, float)) else " · the judge has not seen it"),
                                "ok" if sending and isinstance(a, (int, float)) and a < 3 else ("warn" if sending else "bad")))
    lr = rep.get("last_report") or {}
    if lr:
        out["reports"].append(("LAST", f"{lr.get('kind')} · {lr.get('vehicle_id')} · seq {lr.get('seq')}", ""))
    for x in (j.get("reports") or [])[-6:]:
        out["reports"].append(("REPORT", f"{x.get('kind')} · {x.get('vehicle')} · {_ago(x.get('ago_s'))}", ""))
    return out


def judge_log(s: Optional[dict], n: int = 80) -> List[Tuple[float, str, str]]:
    return [(float(x.get("t") or 0), str(x.get("text") or ""), str(x.get("sev") or "")) for x in ((s or {}).get("judge_log") or [])[-n:]]


# ---- RUN VIEW (big numerals, readable from 2 m) ----------------------------------------------------------------------------------
@dataclass
class Big:
    key: str
    label: str
    value: str
    level: str = ""
    sub: str = ""


def run_view(s: Optional[dict], fast: Optional[dict] = None) -> List[Big]:
    s = s or {}
    st, o, bar = s.get("status") or {}, s.get("ocs") or {}, s.get("bar") or {}
    fv = (fast or {}).get("v") or {}
    ctl = s.get("control") or {}
    g = lambda k: fv.get(k, st.get(k))
    task, stage = g("task_name"), g("task_state")
    el = ctl.get("elapsed_s")
    guard = (st.get("guard") or {})
    gchip = str(guard.get("chip") or "—").upper()
    lg = ((s.get("logging") or {}).get("session") or {})
    rec_on = bool(lg.get("running")) or bool(((s.get("logging") or {}).get("usv") or {}).get("on"))
    hdg, sog = g("hdg_deg"), g("sog_mps")
    word = str(bar.get("word") or "—")
    return [Big("state", "STATE", word, {"RUNNING": "ok", "HELD": "warn", "LINK LOST": "bad"}.get(word, "info" if word == "ARMED" else ""),
                f"{o.get('r') or '?'} · {g('state') or '?'} · {g('mode') or '?'}"),
            Big("stage", "STAGE", str(stage or "—"), "ok" if stage else "", str(task or "no task")),
            Big("time", "ELAPSED", "—" if el is None else f"{int(el) // 60}:{int(el) % 60:02d}", "", f"run {o.get('run_id') or '—'}"),
            Big("speed", "SPEED", "—" if sog is None else f"{float(sog):.2f}", "", "m/s"),
            Big("hdg", "HEADING", "—" if hdg is None else f"{float(hdg):03.0f}°", "", "true"),
            Big("guard", "GUARD", gchip, {"CLEAR": "ok", "SLOW": "warn", "STOP": "bad", "LOST": "warn"}.get(gchip, ""),
                "" if guard.get("range_m") is None else f"{float(guard['range_m']):.1f} m"),
            Big("rec", "REC", "● ON" if rec_on else "○ OFF", "ok" if rec_on else "warn", str(lg.get("label") or ""))]


# ---- SYSTEM -----------------------------------------------------------------------------------------------------------------
@dataclass
class SysAction:
    id: str
    host: str
    title: str
    what: str
    confirm: str                     # '' = asks yes / no; else the word the operator types (the page's rule)
    options: List[dict] = field(default_factory=list)
    refused: str = ""


def system_actions(meta: Optional[dict], s: Optional[dict]) -> List[SysAction]:
    refused = (((s or {}).get("system") or {}).get("refused")) or {}
    out = []
    for a in (meta or {}).get("system") or []:
        out.append(SysAction(str(a.get("id")), str(a.get("host") or ""), str(a.get("title") or a.get("id")), str(a.get("what") or ""),
                             str(a.get("confirm") or ""), list(a.get("options") or []), str(refused.get(a.get("id")) or "")))
    return out


def system_body(a: SysAction, values: Dict[str, str], typed: str = "") -> dict:
    """POST /system's body: {"action", "options", "confirm"} - empty option values are left to the defaults."""
    return {"action": a.id, "options": {k: v for k, v in values.items() if v not in ("", None)}, "confirm": typed}


def system_jobs(s: Optional[dict], n: int = 12) -> List[Row]:
    out = []
    for jb in (((s or {}).get("system") or {}).get("jobs") or [])[-n:]:
        st = str(jb.get("state") or jb.get("status") or "?")
        out.append((str(jb.get("action") or jb.get("id") or "?"), f"{st} · {jb.get('why') or jb.get('text') or ''}".strip(" ·"),
                    "ok" if st in ("done", "ok") else ("bad" if st in ("failed", "error") else "warn")))
    return out


# ---- HELP -------------------------------------------------------------------------------------------------------------------
def help_sections(meta: Optional[dict]) -> List[Tuple[str, List[Tuple[str, str]]]]:
    m = meta or {}
    flow = [(str(a), str(b)) for a, b in (m.get("flow") or [])]
    return [("THE RUN SEQUENCE (the hand-over flow)", flow),
            ("RUN STATES (R, the OCS)", [(k, str(v)) for k, v in (m.get("r_meaning") or {}).items()]),
            ("VEHICLE STATES (V, the boat)", [(k, str(v)) for k, v in (m.get("v_meaning") or {}).items()])]


# ==== the Tk side (main thread only; built once, written by diff through app.put) ===========================================
def _rows_box(app, parent, title: str):
    import tkinter as tk
    from .theme import FONT, SPACE
    p = app.pal
    box = tk.Frame(parent, background=p["panel"], highlightthickness=1, highlightbackground=p["line"], padx=SPACE["m"], pady=SPACE["s"])
    box.pack(fill="x", pady=(0, SPACE["s"]))
    app._label(box, title, fg="dim", font=FONT["label"]).pack(anchor="w")
    grid = tk.Frame(box, background=p["panel"])
    grid.pack(fill="x", pady=(SPACE["xs"], 0))
    grid.columnconfigure(1, weight=1)
    return {"grid": grid, "rows": {}}


def _put_rows(app, key: str, box: dict, rows: List[Row]) -> None:
    from .theme import FONT, level_colours
    for i, (label, text, level) in enumerate(rows):
        if label not in box["rows"]:
            k = app._label(box["grid"], label, fg="dim", font=FONT["label"], width=10)
            v = app._label(box["grid"], "", font=FONT["mono"], wraplength=420)
            k.grid(row=i, column=0, sticky="nw", pady=1)
            v.grid(row=i, column=1, sticky="w", pady=1)
            box["rows"][label] = v
        fg = level_colours(app.pal, level)[0] if level else app.pal["text"]
        app.put(f"{key}:{label}", box["rows"][label], text=text, foreground=fg)


def build_judge(app, fr) -> None:
    import tkinter as tk
    from .theme import FONT
    from .views import ScrollFrame
    sf = ScrollFrame(fr, app.pal["panel"])
    sf.outer.pack(fill="both", expand=True)
    inner = sf.inner
    app.judge_boxes = {k: _rows_box(app, inner, t) for k, t in (("link", "ROBOCOMMAND"), ("run", "THE RUN"),
                                                                 ("vehicles", "HEARTBEATS — what we send · what the judge saw"),
                                                                 ("reports", "REPORTS"))}
    app._label(inner, "JUDGE TRAFFIC", fg="dim", font=FONT["label"]).pack(anchor="w")
    app.judge_text = tk.Text(inner, height=12, font=FONT["mono"], background=app.pal["panel"], foreground=app.pal["text"],
                             borderwidth=0, highlightthickness=0, wrap="word", state="disabled")
    app.judge_text.pack(fill="both", expand=True)
    for tag, tok in (("bad", "alarm"), ("warn", "warnText"), ("ok", "ok"), ("t", "dim")):
        app.judge_text.tag_configure(tag, foreground=app.pal[tok])


def render_judge(app) -> None:
    rows = judge_rows(app.state)
    for k, box in app.judge_boxes.items():
        _put_rows(app, "judge:" + k, box, rows.get(k) or [("—", "nothing yet", "")])
    lines = judge_log(app.state)
    sig = (len(lines), lines[-1][0] if lines else None)
    if app._cache.get("judge-log") == sig:
        return
    app._cache["judge-log"] = sig
    t = app.judge_text
    t.configure(state="normal")
    t.delete("1.0", "end")
    for ts, text, sev in reversed(lines):                    # newest first, as the page's feed
        t.insert("end", time.strftime("%H:%M:%S ", time.localtime(ts)), "t")
        t.insert("end", text + "\n", sev if sev in ("bad", "warn", "ok") else "")
    t.configure(state="disabled")


def build_big(app, parent):
    """The big-numeral run view (readable from 2 m): a full-body overlay, toggled by ⛶ BIG VIEW / Escape."""
    import tkinter as tk
    from .theme import FONT, SPACE
    p = app.pal
    fr = tk.Frame(parent, background=p["bg"], padx=SPACE["xl"], pady=SPACE["l"])
    app.big_tiles = {}
    for i, key in enumerate(("state", "stage", "time", "speed", "hdg", "guard", "rec")):
        r, c = divmod(i, 4)
        tile = tk.Frame(fr, background=p["panel"], highlightthickness=2, highlightbackground=p["line"], padx=SPACE["l"], pady=SPACE["m"])
        tile.grid(row=r, column=c, sticky="nsew", padx=SPACE["s"], pady=SPACE["s"])
        lab = app._label(tile, key.upper(), fg="dim", font=FONT["big_label"])
        lab.pack(anchor="w")
        val = app._label(tile, "—", font=FONT["big"])
        val.pack(anchor="w", pady=(SPACE["xs"], 0))
        sub = app._label(tile, "", fg="muted", font=FONT["big_sub"])
        sub.pack(anchor="w")
        app.big_tiles[key] = {"tile": tile, "lab": lab, "val": val, "sub": sub}
    for c in range(4):
        fr.columnconfigure(c, weight=1, uniform="big")
    for r in range(2):
        fr.rowconfigure(r, weight=1, uniform="bigr")
    app._label(fr, "Esc or ⛶ BIG VIEW = back to the console · the buttons stay on the console", fg="dim", font=FONT["small"],
               bg="bg").grid(row=2, column=0, columnspan=4, sticky="w", pady=(SPACE["s"], 0))
    return fr


def render_big(app) -> None:
    from .theme import level_colours
    p = app.pal
    for b in run_view(app.state, app.fast):
        t = app.big_tiles.get(b.key)
        if not t:
            continue
        fg = level_colours(p, b.level)[0] if b.level else p["text"]
        line = {"bad": p["alarm"], "warn": p["warn"], "ok": p["ok"]}.get(b.level, p["line"])
        app.put(f"big:{b.key}:lab", t["lab"], text=b.label)
        app.put(f"big:{b.key}:val", t["val"], text=b.value, foreground=fg)
        app.put(f"big:{b.key}:sub", t["sub"], text=b.sub)
        app.put(f"big:{b.key}:tile", t["tile"], highlightbackground=line)


def build_system(app, fr) -> None:
    import tkinter as tk
    from tkinter import ttk
    from .theme import FONT, SPACE
    from .views import ScrollFrame
    p = app.pal
    sf = ScrollFrame(fr, p["panel"])
    sf.outer.pack(fill="both", expand=True)
    inner = sf.inner
    app._label(inner, "Every action asks first; the dangerous ones need their word typed (as on the page). A greyed action says why.",
               fg="muted", font=FONT["small"], wraplength=480).pack(anchor="w", pady=(0, SPACE["s"]))
    app.sys_cards, app.sys_vars = {}, {}
    app.sys_jobs_box = _rows_box(app, inner, "JOBS (the last actions and their state)")
    app.sys_inner = inner


def _build_sys_cards(app) -> None:
    import tkinter as tk
    from tkinter import ttk
    from .theme import FONT, SPACE
    p, inner = app.pal, app.sys_inner
    for a in system_actions(app.model.meta, app.state):
        card = tk.Frame(inner, background=p["panel"], highlightthickness=1, highlightbackground=p["line"], padx=SPACE["m"], pady=SPACE["s"])
        card.pack(fill="x", pady=(0, SPACE["s"]))
        top = tk.Frame(card, background=p["panel"])
        top.pack(fill="x")
        app._label(top, a.title, font=FONT["head"]).pack(side="left")
        app._label(top, a.host.upper(), fg="dim", font=FONT["label"]).pack(side="left", padx=SPACE["s"])
        app._button(top, "sys:" + a.id, "▶ RUN" if not a.confirm else f"▶ {a.title.split(' (')[0].upper()[:18]}").pack(side="right")
        app._label(card, a.what, fg="muted", font=FONT["small"], wraplength=460).pack(anchor="w")
        opts = tk.Frame(card, background=p["panel"])
        opts.pack(fill="x")
        app.sys_vars[a.id] = {}
        for i, o in enumerate(a.options):
            app._label(opts, o.get("label") or o.get("name"), fg="dim", font=FONT["small"]).grid(row=0, column=2 * i, sticky="w", padx=(0, 4))
            var = tk.StringVar(value=str(o.get("default") or ""))
            if o.get("choices"):
                w = ttk.Combobox(opts, state="readonly", values=[str(c) for c in o["choices"]], textvariable=var, width=9)
            else:
                w = tk.Entry(opts, textvariable=var, width=10, font=FONT["mono"])
            w.grid(row=0, column=2 * i + 1, sticky="w", padx=(0, SPACE["s"]))
            app.sys_vars[a.id][o.get("name")] = var
        why = app._label(card, "", fg="alarmText", font=FONT["small"], wraplength=460)
        why.pack(anchor="w")
        app.sys_cards[a.id] = {"card": card, "why": why}


def render_system(app) -> None:
    from .model import Btn
    if not app.sys_cards and app.model.meta:
        _build_sys_cards(app)
    for a in system_actions(app.model.meta, app.state):
        c = app.sys_cards.get(a.id)
        if not c:
            continue
        app.put("sys-why:" + a.id, c["why"], text=("not now: " + a.refused) if a.refused else "")
        app.btn_model["sys:" + a.id] = Btn(f"system {a.id}", app.buttons["sys:" + a.id].cget("text") if "sys:" + a.id in app.buttons else a.id,
                                           ok=not a.refused, why=a.refused, kind="danger" if a.confirm else "go", key="sys:" + a.id)
    _put_rows(app, "sys-jobs", app.sys_jobs_box, system_jobs(app.state) or [("—", "no action run from here yet", "")])


def build_help(app, fr) -> None:
    import tkinter as tk
    from .theme import FONT
    t = tk.Text(fr, font=FONT["body"], background=app.pal["panel"], foreground=app.pal["text"], borderwidth=0, highlightthickness=0,
                wrap="word", padx=4)
    t.pack(fill="both", expand=True)
    t.tag_configure("h", font=FONT["head"], foreground=app.pal["accText"], spacing1=10, spacing3=4)
    t.tag_configure("k", font=FONT["mono_b"])
    t.tag_configure("d", foreground=app.pal["muted"])
    app.help_text = t
    app._help_done = False


def render_help(app) -> None:
    if app._help_done or not app.model.meta:
        return
    app._help_done = True
    t = app.help_text
    t.configure(state="normal")
    t.delete("1.0", "end")
    t.insert("end", "Hover any button for the line it sends; the CMD tab has every word with its modifiers.\n", "d")
    for title, rows in help_sections(app.model.meta):
        t.insert("end", title + "\n", "h")
        for k, v in rows:
            t.insert("end", f"{k:16}", "k")
            t.insert("end", f" {v}\n", "d")
    t.configure(state="disabled")


# ==== SETTINGS (Dmitrii 2026-11-12: "a settings window, with configs which we could save and restore, as we had in previous gui") ====
# The previous GUI's equivalent: the boat window's TUNING card (the mission manager's runtime knobs via `tune KEY VALUE`, `tune save` =
# a profile copy on the boat) + the page's per-browser prefs. Here: the knobs, the console's own prefs, and NAMED CONFIGS on the Mac
# (logs/ocs/tk_configs/<name>.json: the knobs' values + the prefs) - restore = `tune` only the knobs that differ (through the one queue),
# the prefs at once. Never an FCU parameter (the knobs are the mission manager's, ADR-9).
import json as _json
import os as _os
import re as _re

CONFIG_DIR = _os.path.join(_os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.dirname(_os.path.abspath(__file__))))),
                           "logs", "ocs", "tk_configs")
PREFS_FILE = _os.path.join(_os.path.dirname(CONFIG_DIR), "tk_prefs.json")
PREF_KEYS = {"theme": ("light", ("light", "dark")), "start_tab": ("LOG", ("LOG", "MAP", "VEHICLES", "JUDGE", "HOSTS", "CMD", "MISSION", "CAMERA")),
             "log_filter": ("all", ("all", "ui", "judge", "usv", "uav", "cautions")), "rec_depth": ("on", ("on", "off")),
             "fast_hz": ("5", ("2", "5", "10"))}
_NAME = _re.compile(r"^[A-Za-z0-9_-]{1,32}$")


@dataclass
class Knob:
    key: str
    value: float
    default: float
    lo: float
    hi: float
    unit: str
    tip: str
    changed: bool


def knobs(s: Optional[dict]) -> List[Knob]:
    t = (((s or {}).get("local") or {}).get("tuning")) or ((s or {}).get("status") or {}).get("tuning") or {}
    out = []
    for k in t.get("knobs") or []:
        try:
            out.append(Knob(str(k["key"]), float(k["value"]), float(k.get("default", k["value"])), float(k.get("lo", 0)),
                            float(k.get("hi", 0)), str(k.get("unit") or ""), str(k.get("tip") or ""), bool(k.get("changed"))))
        except (KeyError, TypeError, ValueError):
            continue
    return out


# 2026-11-12 (Dmitrii: "configs for important things ... make evaluation for relevance and proper grouping"): the knob groups, in the
# order they matter tomorrow. A knob the boat adds later lands in its section's group (or OTHER). DOCK is collapsed (not in the presets).
GROUPS = [("SPEEDS", ("max_speed_mps", "v_min_mps", "arc_v_mps", "transit_v", "scout_transit_v", "circle_v", "slow_v")),
          ("TURNING", ("yaw_rate_max_dps", "heading_timeout_s", "bow_clear_m", "scan_rate_dps")),
          ("APPROACH", ("start_fwd_m", "standoff_m_scout", "scout_standoff_m", "arrive_m", "los_m", "la_m", "lead_step_m", "look_s")),
          ("OBSTACLES — safety", ("stop_m_guard", "guard_stop_m", "guard_slow_m", "backoff_m", "ring_m", "clear_m")),
          ("ORBIT / SCAN", ("scan_r_m", "scan_v_mps")),
          ("FIELD", ("margin_m", "course_tol_m", "bnd_slow_m", "bnd_stop_m", "budget_s", "lane_m")),
          ("DOCK (T3)", ("line_v", "sway_v", "enter_v", "standoff_m", "stop_m"))]
COLLAPSED = ("DOCK (T3)",)


def knob_group(k: "Knob", sections: Tuple[str, ...] = ()) -> str:
    for g, keys in GROUPS:
        if k.key in keys:
            return g
    if "t3" in sections:
        return "DOCK (T3)"
    return "OTHER"


def grouped_knobs(s: Optional[dict]) -> List[Tuple[str, List["Knob"]]]:
    t = (((s or {}).get("local") or {}).get("tuning")) or ((s or {}).get("status") or {}).get("tuning") or {}
    sect = {str(k.get("key")): tuple(k.get("sections") or ()) for k in t.get("knobs") or []}
    order = [g for g, _ in GROUPS] + ["OTHER"]
    by: Dict[str, List[Knob]] = {}
    for k in knobs(s):
        by.setdefault(knob_group(k, sect.get(k.key, ())), []).append(k)
    return [(g, by[g]) for g in order if g in by]


def tune_word(k: Knob, value: str) -> Tuple[str, str]:
    """(the line, '') or ('', why) - checked against the knob's range here first; the boat checks again."""
    try:
        v = float(str(value).strip())
    except ValueError:
        return "", f"{k.key}: '{value}' is not a number"
    if not (k.lo <= v <= k.hi):
        return "", f"{k.key}: {v:g} is outside {k.lo:g} … {k.hi:g} {k.unit}"
    return f"tune {k.key} {v:g}", ""


def pair_rule(key: str, value: float, live: Dict[str, float]) -> str:
    """B v0.1.86.1: the guard's SLOW range must stay above its STOP range (the boat REJECTS it otherwise) - said before sending."""
    if key == "guard_slow_m" and "guard_stop_m" in live and value <= live["guard_stop_m"]:
        return f"guard_slow_m {value:g} must stay above guard_stop_m {live['guard_stop_m']:g}"
    if key == "guard_stop_m" and "guard_slow_m" in live and value >= live["guard_slow_m"]:
        return f"guard_stop_m {value:g} must stay below guard_slow_m {live['guard_slow_m']:g}"
    return ""


def check_name(name: str) -> str:
    return "" if _NAME.match(name or "") else "a name: letters, digits, _ and -, up to 32"


def make_config(name: str, s: Optional[dict], prefs: Dict[str, str], version: str = "") -> dict:
    return {"name": name, "saved": time.strftime("%Y-%m-%d %H:%M:%S"), "version": version,
            "knobs": {k.key: k.value for k in knobs(s)}, "prefs": {k: prefs.get(k, d) for k, (d, _c) in PREF_KEYS.items()}}


def save_config(cfg: dict, d: Optional[str] = None) -> str:
    d = d or CONFIG_DIR
    why = check_name(cfg.get("name", ""))
    if why:
        raise ValueError(why)
    _os.makedirs(d, exist_ok=True)
    path = _os.path.join(d, cfg["name"] + ".json")
    tmp = path + ".tmp"
    with open(tmp, "w") as fh:
        _json.dump(cfg, fh, indent=1)
    _os.replace(tmp, path)
    return path


def list_configs(d: Optional[str] = None) -> List[dict]:
    d = d or CONFIG_DIR
    out = []
    for f in sorted(_os.listdir(d)) if _os.path.isdir(d) else []:
        if f.endswith(".json"):
            try:
                with open(_os.path.join(d, f)) as fh:
                    c = _json.load(fh)
                out.append({"name": c.get("name") or f[:-5], "saved": c.get("saved", ""), "n": len(c.get("knobs") or {}), "cfg": c})
            except (OSError, ValueError):
                continue
    return out


def delete_config(name: str, d: Optional[str] = None) -> bool:
    d = d or CONFIG_DIR
    if check_name(name):
        return False
    p = _os.path.join(d, name + ".json")
    if _os.path.exists(p):
        _os.remove(p)
        return True
    return False


def restore_plan(cfg: dict, s: Optional[dict]) -> Tuple[List[str], List[str]]:
    """(the tune words for the knobs whose live value differs, notes) - a knob the boat no longer has, or out of range, is a note."""
    live = {k.key: k for k in knobs(s)}
    words, notes = [], []
    for key, val in (cfg.get("knobs") or {}).items():
        k = live.get(key)
        if k is None:
            notes.append(f"{key}: not on this boat now - skipped")
            continue
        if abs(float(val) - k.value) < 1e-9:
            continue
        w, why = tune_word(k, str(val))
        (words.append(w) if w else notes.append(why))
    return words, notes


def load_prefs(path: Optional[str] = None) -> Dict[str, str]:
    path = path or PREFS_FILE
    out = {k: d for k, (d, _c) in PREF_KEYS.items()}
    try:
        with open(path) as fh:
            for k, v in (_json.load(fh) or {}).items():
                if k in PREF_KEYS and str(v) in PREF_KEYS[k][1]:
                    out[k] = str(v)
    except (OSError, ValueError):
        pass
    return out


def save_prefs(prefs: Dict[str, str], path: Optional[str] = None) -> None:
    path = path or PREFS_FILE
    _os.makedirs(_os.path.dirname(path), exist_ok=True)
    with open(path, "w") as fh:
        _json.dump({k: prefs.get(k, d) for k, (d, _c) in PREF_KEYS.items()}, fh, indent=1)


def build_settings(app, fr) -> None:
    """The SETTINGS tab of the ⚙ window: CONFIGS (save / restore / delete, on the Mac) · TUNING (the knobs, set one, save a profile copy on
    the boat) · CONSOLE (the prefs)."""
    import tkinter as tk
    from tkinter import ttk
    from .theme import FONT, SPACE
    from .views import ScrollFrame
    p = app.pal
    sf = ScrollFrame(fr, p["panel"])
    sf.outer.pack(fill="both", expand=True)
    inner = sf.inner

    def card(title, note=""):
        c = tk.Frame(inner, background=p["panel"], highlightthickness=1, highlightbackground=p["line"], padx=SPACE["m"], pady=SPACE["s"])
        c.pack(fill="x", pady=(0, SPACE["m"]))
        app._label(c, title, font=FONT["head"]).pack(anchor="w")
        if note:
            app._label(c, note, fg="muted", font=FONT["small"], wraplength=640).pack(anchor="w", pady=(0, SPACE["xs"]))
        return c
    # CONFIGS
    c = card("CONFIGS", "A named snapshot of the mission manager's knobs + this console's preferences, kept on this Mac "
                        "(logs/ocs/tk_configs/). RESTORE sends only the knobs that differ (MANUAL / AUTO_HOLD only) and applies the preferences.")
    row = tk.Frame(c, background=p["panel"])
    row.pack(fill="x", pady=(SPACE["xs"], SPACE["xs"]))
    app.cfg_name = tk.Entry(row, font=FONT["mono"], width=24)
    app.cfg_name.pack(side="left", ipady=4)
    app._button(row, "cfg-save", "SAVE CONFIG").pack(side="left", padx=SPACE["s"])
    app.cfg_list = tk.Listbox(c, height=6, font=FONT["mono"], exportselection=False, background=p["bg"], foreground=p["text"],
                              selectbackground=p["acc"], selectforeground="#ffffff", borderwidth=1, relief="solid")
    app.cfg_list.pack(fill="x")
    app.cfg_list.bind("<<ListboxSelect>>", lambda e: render_configs(app, preview=True))
    row2 = tk.Frame(c, background=p["panel"])
    row2.pack(fill="x", pady=(SPACE["xs"], 0))
    app._button(row2, "cfg-restore", "RESTORE").pack(side="left")
    app._button(row2, "cfg-delete", "DELETE").pack(side="left", padx=SPACE["s"])
    app.cfg_preview = app._label(c, "", fg="accText", font=FONT["mono"], wraplength=640)
    app.cfg_preview.pack(anchor="w", pady=(SPACE["xs"], 0))
    # TUNING
    c = card("TUNING — the mission manager's runtime knobs",
             "the next task start reads a new value; a running task keeps its own · MANUAL / AUTO_HOLD only · never an FCU parameter")
    app.tune_grid = tk.Frame(c, background=p["panel"])
    app.tune_grid.pack(fill="x")
    for i, h in enumerate(("knob", "live", "default", "range", "new value", "")):
        app._label(app.tune_grid, h, fg="dim", font=FONT["label"]).grid(row=0, column=i, sticky="w", padx=(0, SPACE["m"]))
    app.tune_rows = {}
    row3 = tk.Frame(c, background=p["panel"])
    row3.pack(fill="x", pady=(SPACE["s"], 0))
    app._button(row3, "tune-save", "SAVE A PROFILE COPY ON THE BOAT").pack(side="left")
    app.tune_note = app._label(c, "", fg="muted", font=FONT["small"])
    app.tune_note.pack(anchor="w")
    # RATES + FCU - read-only here, with where they are changed (the evaluation of 2026-11-12)
    c = card("RATES AND FCU PARAMETERS — read-only here",
             "rates: status 2 Hz · local 5 Hz · map 2 Hz · tick 10 Hz (the boat profile, set at launch); the judge heartbeat is 2 Hz by the "
             "handbook (fixed); this console's own rate is in CONSOLE below. FCU parameters (WP_SPEED, CRUISE_SPEED, SPEED_MAX, ATC_STR_RAT_*, "
             "PIVOT_TURN_ANGLE) are never written from a page: apply_params.py is the only preset writer (docs/fcu/steering_rate.param, the "
             "morning's read-back first); usv_ui's `param NAME VAL` is the one-off, disarmed.")
    app.rates_line = app._label(c, "", fg="text", font=FONT["mono"], wraplength=640)
    app.rates_line.pack(anchor="w")
    # CONSOLE
    c = card("CONSOLE", "kept in logs/ocs/tk_prefs.json; the theme and the fast rate apply at the next start of the console")
    app.pref_vars = {}
    g = tk.Frame(c, background=p["panel"])
    g.pack(fill="x")
    for i, (k, (d, choices)) in enumerate(PREF_KEYS.items()):
        app._label(g, k.replace("_", " "), fg="dim", font=FONT["label"]).grid(row=i, column=0, sticky="w", padx=(0, SPACE["m"]), pady=2)
        var = tk.StringVar(value=app.prefs.get(k, d))
        cb = ttk.Combobox(g, state="readonly", values=list(choices), textvariable=var, width=12)
        cb.grid(row=i, column=1, sticky="w", pady=2)
        cb.bind("<<ComboboxSelected>>", lambda e, k=k: app._pref_changed(k))
        app.pref_vars[k] = var
    app._cfg_names = []


def render_configs(app, preview: bool = False) -> None:
    items = list_configs()
    names = [i["name"] for i in items]
    if names != app._cfg_names:
        app._cfg_names = names
        app.cfg_list.delete(0, "end")
        for i in items:
            app.cfg_list.insert("end", f"{i['name']:24} {i['saved']}  ·  {i['n']} knobs")
    sel = app.cfg_list.curselection()
    if preview and sel:
        cfg = items[sel[0]]["cfg"]
        words, notes = restore_plan(cfg, app.state)
        app.put("cfg-preview", app.cfg_preview, text=("restore would send: " + ("; ".join(words) if words else "nothing (the knobs match)"))
                + ("\n" + "; ".join(notes) if notes else "") + f"\nprefs: " + ", ".join(f"{k} {v}" for k, v in (cfg.get("prefs") or {}).items()))


def render_tuning(app) -> None:
    import tkinter as tk
    from .theme import FONT, SPACE
    from .model import Btn
    ks = knobs(app.state)
    groups = grouped_knobs(app.state)
    sig = tuple((g, tuple(k.key for k in kk)) for g, kk in groups)
    if sig != getattr(app, "_tune_sig", None):                 # the knob set changed: lay the grid out again, grouped
        app._tune_sig = sig
        for w in list(app.tune_grid.grid_slaves()):
            if int(w.grid_info().get("row", 0)) > 0:
                w.destroy()
        for key in [k for k in app.tune_rows if isinstance(k, str)]:     # the knob rows (the group headers are tuple keys)
            app.buttons.pop("tune:" + key, None)
        app.tune_rows = {}
        app._tune_layout = []
        for g, kk in groups:
            app._tune_layout.append(("group", g))
            app._tune_layout += [("knob", k.key) for k in kk]
    by_key = {k.key: k for k in ks}
    i = 0
    for kind, name in getattr(app, "_tune_layout", []):
        i += 1
        if kind == "group":
            if ("hdr", name) not in app.tune_rows:
                h = app._label(app.tune_grid, name + ("  ▸ click to show (not in tomorrow's presets)" if name in COLLAPSED else ""),
                               fg="accText", font=FONT["label"], cursor="hand2" if name in COLLAPSED else "")
                h.grid(row=i, column=0, columnspan=6, sticky="w", pady=(SPACE["s"], 2))
                if name in COLLAPSED:
                    def toggle(e, h=h, name=name):
                        app._dock_open = not getattr(app, "_dock_open", False)
                        h.configure(text=name + ("  ▾ click to hide" if app._dock_open else "  ▸ click to show (not in tomorrow's presets)"))
                        app._tune_sig = None
                        render_tuning(app)
                    h.bind("<Button-1>", toggle)
                app.tune_rows[("hdr", name)] = {"hdr": h}
            continue
        k = by_key.get(name)
        if k is None or (knob_group(k) in COLLAPSED and not getattr(app, "_dock_open", False)):
            continue
        r = app.tune_rows.get(k.key)
        if r is None:
            lab = app._label(app.tune_grid, k.key, font=FONT["mono_b"])
            live = app._label(app.tune_grid, "", font=FONT["mono"])
            dflt = app._label(app.tune_grid, "", fg="dim", font=FONT["mono"])
            rng = app._label(app.tune_grid, "", fg="dim", font=FONT["mono"])
            ent = tk.Entry(app.tune_grid, font=FONT["mono"], width=8)
            for j, w in enumerate((lab, live, dflt, rng, ent)):
                w.grid(row=i, column=j, sticky="w", padx=(0, SPACE["m"]), pady=1)
            app._button(app.tune_grid, "tune:" + k.key, "SET").grid(row=i, column=5, sticky="w", pady=1)
            from .views import Tooltip
            Tooltip(lab, lambda t=k.tip, u=k.unit: f"{t} ({u})", app.pal)
            r = app.tune_rows[k.key] = {"live": live, "dflt": dflt, "rng": rng, "ent": ent}
        app.put(f"tune:{k.key}:live", r["live"], text=f"{k.value:g} {k.unit}" + (" ●" if k.changed else ""),
                foreground=app.pal["warnText"] if k.changed else app.pal["text"])
        app.put(f"tune:{k.key}:dflt", r["dflt"], text=f"{k.default:g}")
        app.put(f"tune:{k.key}:rng", r["rng"], text=f"{k.lo:g} … {k.hi:g}")
        app.btn_model["tune:" + k.key] = Btn("tune …", "SET", kind="go", key="tune:" + k.key)
    app.put("tune-note", app.tune_note, text="" if ks else "no knobs in the state yet (the boat's status carries them)")
    loc = (app.state or {}).get("local") or {}
    rates = loc.get("rates") or ((app.state or {}).get("status") or {}).get("rates") or {}
    app.put("rates-line", app.rates_line, text=("the boat now: " + " · ".join(f"{k} {v}" for k, v in rates.items())) if rates
            else "the boat's rates are not in /state.json (the profile values above apply)")
    for k in ("cfg-save", "cfg-restore", "cfg-delete"):
        app.btn_model[k] = Btn(k, {"cfg-save": "SAVE CONFIG", "cfg-restore": "RESTORE", "cfg-delete": "DELETE"}[k],
                               kind="go" if k != "cfg-delete" else "danger", key=k)
    app.btn_model["tune-save"] = Btn("tune save", "SAVE A PROFILE COPY ON THE BOAT", key="tune-save")
