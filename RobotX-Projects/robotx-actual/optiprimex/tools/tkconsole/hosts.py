"""WP44 (K8): the HOSTS tab - AGX · NX · MAC, "like Activity Monitor" (Dmitrii 2026-11-11), from session B's GET /hosts.json
(docs/PROMPT_PRESETS_NIGHT_2026-11-12.md §2.5; tegrastats on the Jetsons, sampled by the backend - nothing measured in the console).

host_cards(body) is pure (tested without Tk): one card per host with its header, the throttling badge, four sparkline metrics (CPU · GPU ·
RAM · TEMP with the power as a second trace) and the disk row (Dmitrii 2026-11-12: "add free disk remaining") with the minutes of
recording left at the measured rate. HostsPanel draws them on Canvases. Unknown = "—", never a crash; no /hosts.json = one line."""
from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional

ORDER = ("agx", "nx", "mac")
STALE_S = 5.0
CPU_RED, CPU_RED_S = 90.0, 30.0          # > 90 % for > 30 s
GPU_RED, RAM_RED = 95.0, 90.0
TEMP_AMBER, TEMP_RED = 80.0, 85.0        # the Orin throttles near 97 °C
DISK_AMBER_GB, DISK_RED_GB = 15.0, 8.0   # 2026-11-12: the AGX stopped recording at < 5 GB on 11 Nov
NEEDS = "backend v0.1.86 needed (hosts.json)"


@dataclass
class Metric:
    key: str
    label: str
    value: str                  # the big number ("63 %", "21.0 / 29.9 GB", "71.5 °C")
    level: str                  # '' | warn | bad
    hist: List[float] = field(default_factory=list)
    lo: float = 0.0
    hi: float = 100.0
    hist2: List[float] = field(default_factory=list)      # the power trace on TEMP
    note: str = ""


def link_quality(link: Optional[dict], est: Optional[dict] = None):
    """The signal bar (Dmitrii 2026-11-12: "signal quality ... as a status bar with text"): (bars 0-4, text, level). From the backend's
    measured `link` {rtt_ms, loss_pct, rssi_dbm, iface} when /hosts.json carries it; else an estimate from what the OCS already sees
    (est: {age_s, up, what}) - labelled "estimated", never presented as a measurement."""
    if isinstance(link, dict) and (_num(link.get("rtt_ms")) is not None or _num(link.get("loss_pct")) is not None):
        rtt, loss, rssi = _num(link.get("rtt_ms")), _num(link.get("loss_pct")) or 0.0, _num(link.get("rssi_dbm"))
        if rtt is None or loss >= 50:
            bars = 0
        elif loss == 0 and rtt < 20:
            bars = 4
        elif loss < 2 and rtt < 60:
            bars = 3
        elif loss < 10 and rtt < 200:
            bars = 2
        else:
            bars = 1
        if rssi is not None and rssi < -80:
            bars = min(bars, 1)
        text = (f"{'—' if rtt is None else f'{rtt:.0f} ms'} · {loss:.0f} % loss" + (f" · {link.get('iface')}" if link.get("iface") else "")
                + (f" · {rssi:.0f} dBm" if rssi is not None else ""))
    elif est:
        age, up = _num(est.get("age_s")), est.get("up")
        if up is False or age is None:
            bars, text = 0, f"{est.get('what', 'link')}: none"
        else:
            bars = 4 if age < 1.0 else 3 if age < 2.0 else 2 if age < 4.0 else 1
            text = f"{est.get('what', 'link')} {age:.1f} s old"
        text = "estimated · " + text + " (no rtt / loss from the backend yet)"
    else:
        return 0, "link — (not measured)", ""
    word = ["no link", "poor", "fair", "good", "excellent"][bars]
    return bars, f"{word} · {text}", ("bad" if bars <= 1 else "warn" if bars == 2 else "ok")


@dataclass
class HostCard:
    key: str
    title: str                  # "AGX · optiprime-agxorin"
    sub: str                    # "MAXN · 0.7 s"
    stale: bool
    throttle: str               # '' or "THROTTLING · <src>"
    metrics: List[Metric]
    disk: str
    disk_level: str
    bars: int = 0
    link: str = "link —"
    link_level: str = ""


def _num(x) -> Optional[float]:
    return float(x) if isinstance(x, (int, float)) and not isinstance(x, bool) else None


def _sustained_over(hist: List[float], limit: float, seconds: float, dt: float) -> bool:
    n = max(1, int(round(seconds / max(dt, 0.1))))
    tail = [v for v in hist[-n:] if isinstance(v, (int, float))]
    return len(tail) >= n and all(v > limit for v in tail)


def disk_text(free_gb, rate_gb_per_10min) -> (str, str):
    f, r = _num(free_gb), _num(rate_gb_per_10min)
    if f is None:
        return "disk free —", ""
    lvl = "bad" if f < DISK_RED_GB else ("warn" if f < DISK_AMBER_GB else "")
    if r and r > 0:
        mins = f / r * 10.0
        left = f" · ≈ {mins:.0f} min of recording left at {r:.1f} GB / 10 min"
    else:
        left = " · recording rate not measured"
    return f"disk free {f:.1f} GB{left}", lvl


def host_cards(body: Optional[dict], est: Optional[Dict[str, dict]] = None) -> List[HostCard]:
    hosts = (body or {}).get("hosts") or {}
    keys = [k for k in ORDER if k in hosts] + sorted(k for k in hosts if k not in ORDER)
    out: List[HostCard] = []
    for k in keys:
        h = hosts.get(k) or {}
        last, hist = h.get("last") or {}, h.get("hist") or {}
        dt = _num(hist.get("dt_s")) or 1.0
        age = _num(h.get("age_s"))
        stale = age is None or age > STALE_S
        sub = " · ".join(x for x in (str(h.get("power_mode") or "") or None, "no beat" if stale else f"{age:.1f} s") if x)
        thr = h.get("throttling")
        throttle = ("THROTTLING" + (f" · {h.get('throttle_src')}" if h.get("throttle_src") else "")) if thr is True else ""
        cpu, gpu, temp = _num(last.get("cpu_pct")), _num(last.get("gpu_pct")), _num(last.get("temp_max_c"))
        ru, rt = _num(last.get("ram_used_mb")), _num(last.get("ram_total_mb"))
        ram_pct = (ru / rt * 100.0) if ru is not None and rt else None
        pw = last.get("power_w")
        pw_total = _num(pw.get("total")) if isinstance(pw, dict) else _num(pw)
        hcpu = list(hist.get("cpu_pct") or [])
        m = [Metric("cpu", "CPU", "—" if cpu is None else f"{cpu:.0f} %",
                    "bad" if _sustained_over(hcpu, CPU_RED, CPU_RED_S, dt) else ("warn" if (cpu or 0) > CPU_RED else ""), hcpu),
             Metric("gpu", "GPU", "—" if gpu is None else f"{gpu:.0f} %", "bad" if (gpu or 0) > GPU_RED else "", list(hist.get("gpu_pct") or [])),
             Metric("ram", "RAM", "—" if ram_pct is None else f"{ru / 1024:.1f} / {rt / 1024:.1f} GB · {ram_pct:.0f} %",
                    "bad" if (ram_pct or 0) > RAM_RED else "", list(hist.get("ram_pct") or [])),
             Metric("temp", "TEMP", "—" if temp is None else f"{temp:.1f} °C", "bad" if (temp or 0) > TEMP_RED else ("warn" if (temp or 0) >= TEMP_AMBER else ""),
                    list(hist.get("temp_max_c") or []), 20.0, 100.0, list(hist.get("power_w") or []),
                    "" if pw_total is None else f"power {pw_total:.1f} W")]
        d, dl = disk_text(last.get("disk_free_gb"), last.get("rec_gb_per_10min"))
        bars, ltxt, llvl = (4, "this Mac · local", "ok") if k == "mac" else link_quality(h.get("link"), (est or {}).get(k))
        out.append(HostCard(k, f"{k.upper()} · {h.get('host') or '?'}", sub or "—", stale, throttle, m, d, dl, bars, ltxt, llvl))
    return out


class HostsPanel:
    """The HOSTS tab's drawing (Tk, main thread only): three columns, a card each, the sparklines on Canvases, written by diff."""

    W, H = 300, 54

    def __init__(self, parent, pal: dict, label_fn):
        import tkinter as tk
        self.tk, self.pal, self.mk = tk, pal, label_fn
        self.frame = tk.Frame(parent, background=pal["panel"])
        self.frame.pack(fill="both", expand=True)
        self.note = label_fn(self.frame, "", fg="muted", font=("Helvetica", 12))
        self.note.pack(anchor="w")
        self.cols = tk.Frame(self.frame, background=pal["panel"])
        self.cols.pack(fill="both", expand=True)
        self.cards: Dict[str, dict] = {}
        self._sig: Dict[str, tuple] = {}

    def _card(self, c: HostCard) -> dict:
        tk, p = self.tk, self.pal
        fr = tk.Frame(self.cols, background=p["panel"], highlightthickness=1, highlightbackground=p["line"], padx=8, pady=6)
        fr.pack(side="left", fill="both", expand=True, padx=(0, 6))
        title = self.mk(fr, c.title, font=("Helvetica", 13, "bold"))
        title.pack(anchor="w")
        sub = self.mk(fr, c.sub, fg="dim", font=("Menlo", 11))
        sub.pack(anchor="w")
        thr = self.mk(fr, "", fg="alarmText", font=("Helvetica", 11, "bold"))
        thr.pack(anchor="w")
        lrow = tk.Frame(fr, background=p["panel"])
        lrow.pack(fill="x", pady=(4, 0))
        sig = tk.Canvas(lrow, width=34, height=18, background=p["panel"], highlightthickness=0)
        sig.pack(side="left")
        ltxt = self.mk(lrow, c.link, font=("Menlo", 11), wraplength=250)
        ltxt.pack(side="left", padx=(6, 0))
        rows = {}
        for m in c.metrics:
            r = tk.Frame(fr, background=p["panel"])
            r.pack(fill="x", pady=(4, 0))
            head = tk.Frame(r, background=p["panel"])
            head.pack(fill="x")
            lab = self.mk(head, m.label, fg="dim", font=("Helvetica", 11, "bold"))
            lab.pack(side="left")
            val = self.mk(head, m.value, font=("Menlo", 13, "bold"))
            val.pack(side="right")
            cv = tk.Canvas(r, width=self.W, height=self.H, background=p["mapBg"], highlightthickness=0)
            cv.pack(fill="x")
            note = self.mk(r, "", fg="muted", font=("Menlo", 10))
            note.pack(anchor="e")
            rows[m.key] = {"val": val, "cv": cv, "note": note}
        disk = self.mk(fr, c.disk, font=("Menlo", 12, "bold"), wraplength=280)
        disk.pack(anchor="w", pady=(8, 0))
        d = {"fr": fr, "title": title, "sub": sub, "thr": thr, "rows": rows, "disk": disk, "sig": sig, "ltxt": ltxt}
        self.cards[c.key] = d
        return d

    def _spark(self, cv, m: Metric) -> None:
        p = self.pal
        cv.delete("all")
        w = max(cv.winfo_width(), self.W)
        h = self.H
        span = max(1e-6, m.hi - m.lo)
        line = {"bad": p["alarm"], "warn": p["warn"]}.get(m.level, p["acc"])

        def pts(vals, lo, hi):
            vs = [v for v in vals[-600:]]
            n = len(vs)
            if n < 2:
                return []
            out = []
            for i, v in enumerate(vs):
                if not isinstance(v, (int, float)):
                    continue
                x = w - (n - 1 - i) * (w / 599.0)                # newest right, 600 samples = full width
                y = h - 2 - (min(max(v, lo), hi) - lo) / max(1e-6, hi - lo) * (h - 4)
                out += [x, y]
            return out
        if m.key == "temp":                                       # the red band above the limit
            yr = h - 2 - (TEMP_RED - m.lo) / span * (h - 4)
            cv.create_rectangle(0, 0, w, yr, fill=p["alarmBg"], outline="")
        xy = pts(m.hist, m.lo, m.hi)
        if len(xy) >= 4:
            cv.create_line(*xy, fill=line, width=2, tags=("trace",))
        if m.hist2:
            hi2 = max([v for v in m.hist2 if isinstance(v, (int, float))] or [1.0]) * 1.2
            xy2 = pts(m.hist2, 0.0, hi2)
            if len(xy2) >= 4:
                cv.create_line(*xy2, fill=p["mag"], width=1, tags=("trace2",))
        if len(xy) < 4:
            cv.create_text(w / 2, h / 2, text="no history", fill=p["dim"], font=("Helvetica", 10))

    def _bars(self, cv, n: int, level: str) -> None:
        p = self.pal
        cv.delete("all")
        col = {"ok": p["ok"], "warn": p["warn"], "bad": p["alarm"]}.get(level, p["dim"])
        for i in range(4):
            h = 5 + i * 4
            cv.create_rectangle(2 + i * 8, 18 - h, 7 + i * 8, 18, fill=col if i < n else p["line2"], outline="", tags=("bar",))

    def render(self, body: Optional[dict], why: str = "", est: Optional[Dict[str, dict]] = None) -> None:
        p = self.pal
        if body is None:
            self.note.configure(text=NEEDS if "404" in (why or "") else f"hosts: no answer ({why or 'waiting'})")
            return
        cards = host_cards(body, est)
        self.note.configure(text=time.strftime("hosts · %H:%M:%S", time.localtime(body.get("t") or time.time())) +
                            " · 10 min, newest right · red: CPU > 90 % for 30 s, GPU > 95 %, RAM > 90 %, temp > 85 °C · disk amber < 15 GB, red < 8 GB")
        for c in cards:
            d = self.cards.get(c.key) or self._card(c)
            fg_title = p["dim"] if c.stale else p["text"]
            d["title"].configure(text=c.title, foreground=fg_title)
            d["sub"].configure(text=c.sub)
            d["fr"].configure(highlightbackground=p["line2"] if c.stale else (p["alarm"] if c.throttle else p["line"]))
            d["thr"].configure(text=c.throttle)
            for m in c.metrics:
                r = d["rows"][m.key]
                r["val"].configure(text=m.value, foreground={"bad": p["alarm"], "warn": p["warnText"]}.get(m.level, p["text"]))
                r["note"].configure(text=m.note)
                sig = (len(m.hist), m.hist[-1] if m.hist else None, m.level, len(m.hist2), r["cv"].winfo_width())
                if self._sig.get((c.key, m.key)) != sig:
                    self._sig[(c.key, m.key)] = sig
                    self._spark(r["cv"], m)
            d["ltxt"].configure(text=c.link, foreground={"bad": p["alarm"], "warn": p["warnText"]}.get(c.link_level, p["text"]))
            if self._sig.get((c.key, "bars")) != (c.bars, c.link_level):
                self._sig[(c.key, "bars")] = (c.bars, c.link_level)
                self._bars(d["sig"], c.bars, c.link_level)
            d["disk"].configure(text=c.disk, foreground={"bad": p["alarm"], "warn": p["warnText"]}.get(c.disk_level, p["text"]))
