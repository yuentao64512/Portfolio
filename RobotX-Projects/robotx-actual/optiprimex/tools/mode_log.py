"""mode_log: the mode sequence and the frame cadence of a run, per stage, checked against the lab or the pool rules
(v0.1.27.1, the no-hardware part of ROADMAP WP9; the GUIDE §6.2 table is filled from its --md output).

Input: the mission manager's command trace (`--trace PATH`, written by run_usv.sh / run_bench.sh as
logs/<host>/run_<stamp>/mm_trace.jsonl; usv/cmd_trace.py) and, optionally, usv_ui's tlog of the same run (`log on` / `--log`),
which adds the FCU's own mode changes (its HEARTBEATs) as the truth.

  mode-log logs/usv/run_20261105_101500/mm_trace.jsonl                 # the rules follow the profile the run used
  mode-log TRACE --lab | --pool [--tlog usv_ui/logs/pool_x.tlog] [--md] [--json]

Per stage (vehicle state, task, sub-state): the modes requested, the modes the FCU reported, the mode-20 frames (count, rate,
the longest gap) and the verdict. The checks:
  entry   every mode-20 run starts with a zero frame                     (firmware rule)
  exit    every mode-20 run ends with a neutral frame                    (before the next mode)
  gap     frames inside a mode-20 run <= 0.25 s apart (WARN > 0.20 s)    (the firmware goes neutral at 250 ms)
  thrash  no 3 mode changes within 2 s                                   (the review's §4.3 HOLD / frame alternation)
  refused no mode request refused by the FCU                             (WARN on --lab: a disarmed bench FCU may refuse GUIDED)
  follow  the FCU reports the requested mode within 1.5 s (1 Hz heartbeat) (WARN on --lab)
  profile --lab wants usv_lab.yaml, --pool usv_pool.yaml / usv_course.yaml (the wrong profile for the day = FAIL)
  armed   --pool: armed whenever AUTO_TASK; --lab: WARN if armed          (props off on the bench)
Exit code 0 = no FAIL, 1 = a FAIL, 2 = no trace.

v0.1.68 (WP17 F8): a FREE RUN trace (the mission manager's --free-run: `mode_req ... free_run: true`, `ghost` records) gets the heading
FREE RUN, the per-stage mode requests with ok / refused (a refusal is expected indoors: one WARN per stage and mode), the ghost
outputs counted per stage (target / vel / frame / hold) and one more check:
  motion  no frame / target / velocity record at all - in free run nothing but mode requests reaches the FCU (F3; FAIL)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

GAP_FAIL_S, GAP_WARN_S = 0.25, 0.20
THRASH_N, THRASH_S = 3, 2.0
FOLLOW_S = 1.5                      # v0.1.28.1: the mode arrives in the 1 Hz HEARTBEAT - one period + margin
LAB_PROFILES = ("usv_lab.yaml",)
POOL_PROFILES = ("usv_pool.yaml", "usv_course.yaml")


@dataclass
class Stage:
    state: str
    task: Optional[str]
    sub: Optional[str]
    t0: float
    t1: float = 0.0
    modes_req: List[str] = field(default_factory=list)
    modes_fcu: List[str] = field(default_factory=list)
    frames: int = 0
    max_gap: float = 0.0
    armed: List[bool] = field(default_factory=list)
    ghost: Dict[str, int] = field(default_factory=dict)     # v0.1.68: free run - the outputs handed to the ghost, by kind

    @property
    def name(self) -> str:
        if self.task:
            return f"{self.state} {self.task}/{self.sub or '-'}"
        return self.state

    def rate(self) -> float:
        d = self.t1 - self.t0
        return self.frames / d if d > 0 else 0.0


@dataclass
class Finding:
    level: str          # FAIL | WARN | ok
    check: str
    t: Optional[float]
    text: str


def _push(seq: List[str], x: Optional[str]) -> None:
    if x and (not seq or seq[-1] != x):
        seq.append(x)


def load_trace(path: str) -> Tuple[dict, List[dict]]:
    head, evs = {}, []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line:
                continue
            try:
                e = json.loads(line)
            except ValueError:
                continue
            if e.get("k") == "start":
                if head:                                  # a second start = a restarted mission manager: keep the last run
                    evs = []
                head = e
            else:
                evs.append(e)
    return head, evs


def load_tlog(path: str, unix0: float) -> List[Tuple[float, str]]:
    """(t since the trace start, mode name) from the FCU's HEARTBEATs in a tlog, on change."""
    from pymavlink import mavutil
    from ..companion.link import ROVER_MODES
    out: List[Tuple[float, str]] = []
    m = mavutil.mavlink_connection(path)
    while True:
        msg = m.recv_match(type="HEARTBEAT")
        if msg is None:
            break
        if msg.get_srcComponent() != 1 or msg.type == mavutil.mavlink.MAV_TYPE_GCS:
            continue                                      # the autopilot's own heartbeat only
        name = ROVER_MODES.get(int(msg.custom_mode), str(msg.custom_mode))
        if not out or out[-1][1] != name:
            out.append((round(getattr(msg, "_timestamp", 0.0) - unix0, 3), name))
    return out


def analyse(head: dict, evs: List[dict], rules: str, tlog_modes: Optional[List[Tuple[float, str]]] = None) -> Dict[str, Any]:
    lab = rules == "lab"
    free = any(e.get("k") == "ghost" or (e.get("k") == "mode_req" and e.get("free_run")) for e in evs)   # v0.1.68 (F8)
    refused_seen: set = set()
    find: List[Finding] = []
    profile = head.get("profile", "")
    want = LAB_PROFILES if lab else POOL_PROFILES
    other = POOL_PROFILES if lab else LAB_PROFILES
    if profile in other:
        find.append(Finding("FAIL", "profile", 0.0, f"--{rules} checks a run of {profile}: the {rules} day runs {' / '.join(want)}"))
    elif profile not in want:
        find.append(Finding("WARN", "profile", 0.0, f"profile {profile or '?'} is neither lab nor pool"))

    stages: List[Stage] = []
    cur: Optional[Stage] = None
    fcu_mode: Optional[str] = None
    fcu_changes: List[Tuple[float, str]] = list(tlog_modes or [])
    req_changes: List[Tuple[float, str]] = []          # accepted requests that changed the wanted mode
    last_req: Optional[str] = None
    in20, last_frame_t, run_open_t = False, None, None

    def close_run(t, how_ok):
        nonlocal in20, last_frame_t, run_open_t
        if in20 and not how_ok:
            find.append(Finding("FAIL", "exit", t, f"mode-20 run from t {run_open_t:.1f} s left without a neutral frame"))
        in20, last_frame_t, run_open_t = False, None, None

    for e in evs:
        t, k = float(e.get("t", 0.0)), e.get("k")
        if k == "obs":
            key = (e.get("state"), e.get("task"), e.get("sub"))
            if cur is None or (cur.state, cur.task, cur.sub) != key:
                if cur is not None:
                    cur.t1 = t
                cur = Stage(str(e.get("state")), e.get("task"), e.get("sub"), t)
                stages.append(cur)
            if e.get("fcu_mode") != fcu_mode:
                fcu_mode = e.get("fcu_mode")
                if not tlog_modes and fcu_mode:
                    fcu_changes.append((t, fcu_mode))
            _push(cur.modes_fcu, fcu_mode)
            first_armed = bool(e.get("armed")) and not any(cur.armed)
            first_disarmed = not e.get("armed") and all(cur.armed)
            cur.armed.append(bool(e.get("armed")))
            if not lab and e.get("state") == "AUTO_TASK" and first_disarmed:          # once per stage
                find.append(Finding("FAIL", "armed", t, f"AUTO_TASK while disarmed ({cur.name})"))
            if lab and first_armed:
                find.append(Finding("WARN", "armed", t, f"armed on the lab profile - props off / ESC power off? ({cur.name})"))
            continue
        if cur is None:
            cur = Stage("?", None, None, t)
            stages.append(cur)
        if k == "ghost":                                  # v0.1.68: free run - an output the ghost took instead of the FCU
            out = str(e.get("out"))
            cur.ghost[out] = cur.ghost.get(out, 0) + 1
            continue
        if free and k in ("frame", "target", "vel"):      # F3: in free run this is never written - motion reached the FCU
            find.append(Finding("FAIL", "motion", t, f"a {k} record in a FREE RUN trace ({cur.name}): motion reached the FCU"))
        if k == "mode_req":
            mode, ok = e.get("mode"), bool(e.get("ok"))
            _push(cur.modes_req, mode + ("" if ok else " (refused)"))
            if not ok and free:
                if (cur.name, mode) not in refused_seen:
                    refused_seen.add((cur.name, mode))
                    find.append(Finding("WARN", "refused", t, f"the FCU refused {mode} ({cur.name}) - free run: ignored, the ghost went on"))
            elif not ok:
                find.append(Finding("WARN" if lab else "FAIL", "refused", t, f"the FCU refused {mode} ({cur.name})"))
            if mode != "CUSTOM_MANUAL" and in20:
                close_run(t, False)
            if ok and mode != last_req:
                req_changes.append((t, mode))
                last_req = mode
        elif k == "frame":
            why = e.get("why")
            if why == "entry":
                if in20:
                    close_run(t, False)
                in20, last_frame_t, run_open_t = True, t, t
            elif why == "neutral":
                if in20 and last_frame_t is not None:
                    cur.max_gap = max(cur.max_gap, t - last_frame_t)
                close_run(t, True)
            else:
                if not in20:
                    find.append(Finding("FAIL", "entry", t, f"a mode-20 frame without the zero entry frame ({cur.name})"))
                    in20, run_open_t = True, t
                elif last_frame_t is not None:
                    gap = t - last_frame_t
                    cur.max_gap = max(cur.max_gap, gap)
                    if gap > GAP_FAIL_S:
                        find.append(Finding("FAIL", "gap", t, f"{gap:.2f} s between frames ({cur.name}): the firmware went neutral"))
                    elif gap > GAP_WARN_S:
                        find.append(Finding("WARN", "gap", t, f"{gap:.2f} s between frames ({cur.name})"))
                last_frame_t = t
                cur.frames += 1
    end_t = float(evs[-1]["t"]) if evs else 0.0
    if cur is not None:
        cur.t1 = max(cur.t1, end_t)
    if in20:
        find.append(Finding("WARN", "exit", end_t, f"the trace ends inside a mode-20 run (from t {run_open_t:.1f} s)"))

    for i in range(len(req_changes)):                   # thrash: THRASH_N changes inside THRASH_S
        j = i + THRASH_N - 1
        if j < len(req_changes) and req_changes[j][0] - req_changes[i][0] <= THRASH_S:
            seq = " -> ".join(m for _, m in req_changes[i:j + 1])
            find.append(Finding("FAIL", "thrash", req_changes[i][0], f"{THRASH_N} mode changes in "
                                f"{req_changes[j][0] - req_changes[i][0]:.1f} s: {seq}"))
            break
    for t, mode in req_changes:                         # follow: the FCU reports the mode within FOLLOW_S
        before = [m for tf, m in fcu_changes if tf <= t]
        already = bool(before) and before[-1] == mode
        if not already and not any(t <= tf <= t + FOLLOW_S and m == mode for tf, m in fcu_changes):
            later = [tf for tf, m in fcu_changes if m == mode and tf > t]
            how = f"after {later[0] - t:.1f} s" if later else "never"
            if not later and end_t - t < FOLLOW_S:      # v0.1.28.1: the trace ended before the FCU had its second
                find.append(Finding("WARN", "follow", t, f"requested {mode} {end_t - t:.1f} s before the trace ends: not seen yet"))
                continue
            find.append(Finding("WARN" if lab else "FAIL", "follow", t, f"requested {mode}, the FCU reported it {how}"))

    fails = [f for f in find if f.level == "FAIL"]
    warns = [f for f in find if f.level == "WARN"]
    return {"profile": profile, "rules": rules, "version": head.get("version"), "fcu": head.get("fcu"),
            "start_unix": head.get("unix"), "duration_s": round(end_t, 1), "stages": stages, "findings": find,
            "fcu_source": "tlog" if tlog_modes else "trace", "ok": not fails, "n_fail": len(fails), "n_warn": len(warns),
            "mode_changes": len(req_changes), "free_run": free}


def ghost_text(st: Stage) -> str:
    return " ".join(f"{k} {n}" for k, n in sorted(st.ghost.items())) or "-"


def stage_verdict(st: Stage, find: List[Finding]) -> str:
    mine = [f for f in find if f.t is not None and st.t0 <= f.t <= st.t1 and f.check not in ("profile",)]
    if any(f.level == "FAIL" for f in mine):
        return "FAIL " + ", ".join(sorted({f.check for f in mine if f.level == "FAIL"}))
    if any(f.level == "WARN" for f in mine):
        return "warn " + ", ".join(sorted({f.check for f in mine}))
    return "ok"


def render_text(r: Dict[str, Any]) -> str:
    when = time.strftime("%Y-%m-%d %H:%M", time.localtime(r["start_unix"])) if r.get("start_unix") else "?"
    out = (["FREE RUN - ghost position, mode requests only to the FCU (the ghost column = the outputs the ghost took)", ""]
           if r.get("free_run") else [])
    out += [f"mode-log: {when}  profile {r['profile'] or '?'}  rules --{r['rules']}  v{r['version']}  FCU {r['fcu']}  "
           f"{r['duration_s']} s  {r['mode_changes']} mode changes  (FCU modes from the {r['fcu_source']})", ""]
    w = max([len(s.name) for s in r["stages"]] + [10])
    if r.get("free_run"):
        out.append(f"{'stage':<{w}}  {'t s':>7} {'dur':>6}  {'requested':<34} {'FCU':<24} {'ghost':<28}  verdict")
        for s in r["stages"]:
            out.append(f"{s.name:<{w}}  {s.t0:7.1f} {s.t1 - s.t0:6.1f}  {' > '.join(s.modes_req) or '-':<34} "
                       f"{' > '.join(s.modes_fcu) or '-':<24} {ghost_text(s):<28}  {stage_verdict(s, r['findings'])}")
    else:
        out.append(f"{'stage':<{w}}  {'t s':>7} {'dur':>6}  {'requested':<28} {'FCU':<24} {'frames':>6} {'Hz':>5} {'gap':>5}  verdict")
    for s in ([] if r.get("free_run") else r["stages"]):
        out.append(f"{s.name:<{w}}  {s.t0:7.1f} {s.t1 - s.t0:6.1f}  {' > '.join(s.modes_req) or '-':<28} "
                   f"{' > '.join(s.modes_fcu) or '-':<24} {s.frames:6d} {s.rate():5.1f} {s.max_gap:5.2f}  {stage_verdict(s, r['findings'])}")
    out.append("")
    for f in r["findings"]:
        out.append(f"  {f.level:<4} {f.check:<8} t {f.t if f.t is not None else 0:7.1f}  {f.text}")
    out.append(f"\nverdict: {'PASS' if r['ok'] else 'FAIL'} ({r['n_fail']} fail, {r['n_warn']} warn)")
    return "\n".join(out)


def render_md(r: Dict[str, Any]) -> str:
    """Rows for the GUIDE §6.2 table "mode sequence per stage, as observed"."""
    when = time.strftime("%Y-%m-%d", time.localtime(r["start_unix"])) if r.get("start_unix") else "?"
    rows = ["| date | profile | stage | requested | FCU reported | frames (Hz, longest gap) | verdict |", "|---|---|---|---|---|---|---|"]
    if r.get("free_run"):
        rows = ["**FREE RUN** (ghost position, mode requests only to the FCU)", "",
                "| date | profile | stage | requested | FCU reported | ghost outputs | verdict |", "|---|---|---|---|---|---|---|"]
    for s in r["stages"]:
        fr = f"{s.frames} ({s.rate():.1f} Hz, {s.max_gap:.2f} s)" if s.frames else "-"
        if r.get("free_run"):
            fr = ghost_text(s)
        rows.append(f"| {when} | {r['profile'] or '?'} | {s.name} | {' → '.join(s.modes_req) or '-'} | "
                    f"{' → '.join(s.modes_fcu) or '-'} | {fr} | {stage_verdict(s, r['findings'])} |")
    return "\n".join(rows)


def to_json(r: Dict[str, Any]) -> str:
    d = dict(r)
    d["stages"] = [{**s.__dict__, "armed": any(s.armed), "rate_hz": round(s.rate(), 2)} for s in r["stages"]]
    d["findings"] = [f.__dict__ for f in r["findings"]]
    return json.dumps(d, indent=1)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("trace", help="the mission manager's command trace (mm_trace.jsonl)")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--lab", action="store_true", help="the bench rules (real Pixhawk, props off, usv_lab.yaml)")
    g.add_argument("--pool", action="store_true", help="the pool rules (armed, usv_pool.yaml / usv_course.yaml)")
    ap.add_argument("--tlog", help="usv_ui's tlog of the same run: the FCU's own mode changes")
    ap.add_argument("--md", action="store_true", help="print the GUIDE §6.2 table rows")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args(argv)
    try:
        head, evs = load_trace(a.trace)
    except OSError as e:
        print(f"mode-log: {e}", file=sys.stderr)
        return 2
    if not head and not evs:
        print(f"mode-log: {a.trace} has no trace records", file=sys.stderr)
        return 2
    rules = "lab" if a.lab else "pool" if a.pool else ("lab" if head.get("profile") in LAB_PROFILES else "pool")
    tl = load_tlog(a.tlog, float(head.get("unix") or 0.0)) if a.tlog else None
    r = analyse(head, evs, rules, tl)
    print(to_json(r) if a.json else render_md(r) if a.md else render_text(r))
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
