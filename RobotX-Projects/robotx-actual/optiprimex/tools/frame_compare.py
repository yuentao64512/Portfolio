#!/usr/bin/env python3
"""frame_compare: usv_ui's `swayto` frames against m0t3's (TEST_SWAY) on the same two points (v0.1.31.1, ROADMAP WP9 - the lab day).

mode20.py re-implements usv_ui's `sway_lat` allocation; WP9 records that both send the same thruster frames. The two runs are not
simultaneous, so the comparison is per run: the frames while each one sways (usv_ui's CSV of `log on <leg>`: rows with a sway state
and c1..c4; the mission manager's `mm_trace.jsonl`: `frame` events while the stage is SWAY), their rate, and the steady frame -
the per-motor median of the frames, sign and size - side by side, with the largest per-motor difference.

  frame-compare --ui logs/usv/run_<stamp>/<leg>.csv --trace logs/usv/run_<stamp>/mm_trace.jsonl [--tol 0.08] [--md]
  exit 0 = the steady frames agree within --tol on every motor, 1 = they differ, 2 = no frames in one of them
"""
from __future__ import annotations

import argparse
import csv
import json
import statistics
import sys
from typing import Dict, List, Optional, Tuple

Frames = List[Tuple[float, List[float]]]          # (t s, [m1, m2, m3, m4])


def ui_frames(path: str) -> Frames:
    """usv_ui's leg CSV: rows while a sway runs (sway_state set, not done / aborted) with a non-zero frame."""
    out: Frames = []
    with open(path, newline="") as fh:
        for r in csv.DictReader(fh):
            st = (r.get("sway_state") or "").strip()
            if not st or st in ("done", "aborted"):
                continue
            try:
                m = [float(r[f"c{i}"]) for i in range(1, 5)]
                t = float(r["t_mono"])
            except (KeyError, ValueError):
                continue
            if any(abs(x) > 1e-6 for x in m):
                out.append((t, m))
    return out


def trace_frames(path: str, stage: str = "SWAY") -> Frames:
    """The mission manager's trace: `frame` events while the observed stage (`obs` sub) is `stage`, zero frames left out."""
    out: Frames = []
    sub = None
    with open(path) as fh:
        for line in fh:
            try:
                e = json.loads(line)
            except ValueError:
                continue
            if e.get("k") == "obs":
                sub = e.get("sub")
            elif e.get("k") == "frame" and sub == stage and e.get("why") == "cmd":
                m = [float(x) for x in e.get("m") or []]
                if len(m) == 4 and any(abs(x) > 1e-6 for x in m):
                    out.append((float(e.get("t", 0.0)), m))
    return out


def summary(fr: Frames) -> Optional[Dict[str, object]]:
    if not fr:
        return None
    ts = [t for t, _ in fr]
    span = max(ts) - min(ts)
    med = [statistics.median(m[i] for _, m in fr) for i in range(4)]
    return {"n": len(fr), "rate_hz": round((len(fr) - 1) / span, 1) if span > 0 else 0.0, "steady": [round(x, 3) for x in med],
            "max": [round(max(abs(m[i]) for _, m in fr), 3) for i in range(4)]}


def compare(ui: Frames, mm: Frames, tol: float) -> Dict[str, object]:
    a, b = summary(ui), summary(mm)
    if a is None or b is None:
        return {"ok": None, "ui": a, "mm": b, "why": "no sway frames in " + ("usv_ui's CSV" if a is None else "the trace")}
    diff = [round(abs(x - y), 3) for x, y in zip(a["steady"], b["steady"])]
    signs = all((x > 0) == (y > 0) for x, y in zip(a["steady"], b["steady"]) if abs(x) > 0.02 or abs(y) > 0.02)
    ok = signs and max(diff) <= tol
    why = "" if ok else ("motor signs differ" if not signs else f"motor {diff.index(max(diff)) + 1} differs by {max(diff)} (> {tol})")
    return {"ok": ok, "ui": a, "mm": b, "diff": diff, "why": why}


def render(r: Dict[str, object], md: bool) -> str:
    ui, mm = r.get("ui"), r.get("mm")
    f = lambda s: "-" if s is None else f"{s['n']} frames, {s['rate_hz']} Hz, steady {s['steady']}"
    if md:
        verdict = "no data" if r["ok"] is None else ("match" if r["ok"] else "DIFFER: " + r["why"])
        return (f"| usv_ui swayto | {f(ui)} |\n| m0t3 SWAY | {f(mm)} |\n"
                f"| per-motor difference | {r.get('diff', '-')} — **{verdict}** |")
    lines = [f"usv_ui swayto : {f(ui)}", f"m0t3 SWAY     : {f(mm)}"]
    if r["ok"] is None:
        lines.append("verdict: " + r["why"])
    else:
        lines.append(f"difference    : {r['diff']}  -> {'MATCH' if r['ok'] else 'DIFFER: ' + r['why']}")
    return "\n".join(lines)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ui", required=True, help="usv_ui's leg CSV (log on <leg> before swayto)")
    ap.add_argument("--trace", required=True, help="the mission manager's mm_trace.jsonl of the m0t3 run")
    ap.add_argument("--stage", default="SWAY")
    ap.add_argument("--tol", type=float, default=0.08, help="largest per-motor difference of the steady frames that still matches")
    ap.add_argument("--md", action="store_true", help="GUIDE §6.2 table rows")
    a = ap.parse_args(argv)
    r = compare(ui_frames(a.ui), trace_frames(a.trace, a.stage), a.tol)
    print(render(r, a.md))
    return 2 if r["ok"] is None else (0 if r["ok"] else 1)


if __name__ == "__main__":
    sys.exit(main())
