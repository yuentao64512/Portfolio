"""tuning: the mission manager's runtime knobs (v0.1.49, ROADMAP WP10k-b item 22) - one table for the OCS's TUNE intent
(cmd_authority: key whitelist + range), the mission manager's SET_TUNING (the live config `cfg.extra[<task>]`, read by the next
task start - T1 / T3 / TEST_SWAY read their sections in start()), the boat window's TUNING card and `ocs_cli tune`.

Never an FCU parameter (ADR-9): PWM limits and the stern factors are the FCU's - apply_params.py only (READ_ONLY below, shown, never set).
Defaults = the tasks' DEFAULTS (tests/test_v0149.py checks they agree). `save` writes a copy of the loaded profile with the live
values under logs/launch/<stamp>/usv_tuned.yaml (never the profile itself); MM_CONFIG_OVERRIDE=<that file> scripts/run_usv.sh loads it.
"""
from __future__ import annotations

import copy
import os
import time
from typing import Any, Dict, List, Optional, Tuple

from ..usv.dockpath import standoff_lines

# key -> (profile sections it sets, default, lo, hi, unit, tooltip)
KNOBS: Dict[str, Tuple[Tuple[str, ...], float, float, float, str, str]] = {
    "line_v":        (("t3",), 0.5, 0.1, 1.0, "m/s", "T3 TO_LINE: the speed along the path to the standoff line"),
    "sway_v":        (("t3",), 0.25, 0.05, 0.6, "m/s", "T3 SWAY_TO_ABEAM and m0t3 (TEST_SWAY): the sway speed along the dock line (mode 20)"),
    "enter_v":       (("t3",), 0.15, 0.05, 0.4, "m/s", "T3 ENTER: the surge into the bay (tapered with the distance)"),
    "standoff_m":    (("t3",), standoff_lines()[0], 2.0, 10.0, "m", "T3 / m0t3: the standoff line's distance off the dock face (v0.1.56: "
                      "unset = finger_m + hull_len_m/2 + clear_m + standoff_margin_m; a value set here is the override)"),
    "stop_m":        (("t3",), 1.5, 0.5, 3.0, "m", "T3 ENTER: stop this far off the dock face"),
    "lead_step_m":   (("t1",), 12.0, 3.0, 25.0, "m", "T1 SEARCH_ENTRY: one lead step toward a far sighting when no entry is in view"),
    "look_s":        (("t1",), 8.0, 2.0, 30.0, "s", "T1 LOOK: the longest the boat faces an undecided sighting"),
    "transit_v":     (("t1",), 0.6, 0.2, 1.5, "m/s", "T1 TRANSIT / EXIT: the speed through the gates"),
    "circle_v":      (("t1",), 0.5, 0.2, 1.2, "m/s", "T1 CIRCLE_ENTRY / EXIT: the speed round the blue buoy"),
    "la_m":          (("t1",), 2.0, 0.5, 6.0, "m", "the carrot (LOS) lookahead of FollowPath / Orbit in T1 (T3's TO_LINE keeps 1.5 m)"),
    "scan_rate_dps": (("t1", "t3"), 20.0, 5.0, 45.0, "deg/s", "the yaw rate of the scans: T1 SEARCH_ENTRY / LOOK / SCAN, T3 FIND_DOCK"),
    # ---- v0.1.86.1 (WP48, Dmitrii via session A: "configs for important things ... speed limiters, initial approach, obstacle stop distance,
    # orbiting distance"): the under-way guidance, the speed cap, the scout, the ZED guard - the same rules (MANUAL / AUTO_HOLD, the next
    # task start reads them; the guard and the speed cap at once, no task runs then). Safety floors: yaw <= 10, bnd_stop >= 1.5,
    # standoff >= 2, clear >= 1.2, guard stop >= 1.0 (and guard slow > guard stop). Names prefixed where a key exists twice (FIELD below).
    "v_min_mps":         (("guidance",), 0.4, 0.2, 0.8, "m/s", "under way: the forward floor while the target is within the drive cone"),
    "arc_v_mps":         (("guidance",), 0.3, 0.1, 0.6, "m/s", "under way: the slow arc beyond the drive cone (not a pirouette)"),
    "yaw_rate_max_dps":  (("guidance",), 5.0, 2.0, 10.0, "deg/s", "under way: the cap on every yaw command (MAX 10)"),
    "heading_timeout_s": (("guidance",), 15.0, 5.0, 60.0, "s", "under way: forward held at 0 this long without the heading coming round -> drive anyway"),
    "bow_clear_m":       (("guidance",), 3.5, 2.0, 6.0, "m", "under way: an object this close in the bow cone with the heading far off -> turn in place"),
    "los_m":             (("guidance",), 3.0, 1.5, 6.0, "m", "under way: the look-ahead along the planned path"),
    "start_fwd_m":       (("guidance",), 1.0, 0.0, 3.0, "m", "move-first: straight ahead before any turn at a task's start (0 = off)"),
    "scan_r_m":          (("guidance",), 5.0, 3.0, 10.0, "m", "the circle scan's radius (at least what the hull holds at scan_v)"),
    "scan_v_mps":        (("guidance",), 0.3, 0.2, 0.6, "m/s", "the circle scan's speed"),
    "bnd_slow_m":        (("guidance",), 6.0, 3.0, 10.0, "m", "the course boundary this close ahead -> slow to 0.15 m/s"),
    "bnd_stop_m":        (("guidance",), 2.5, 1.5, 5.0, "m", "the course boundary this close ahead with the heading off -> turn in place (MIN 1.5)"),
    "course_tol_m":      (("guidance",), 1.5, 0.5, 3.0, "m", "WP46: a task target only inside the RxCourse boundary + this"),
    "max_speed_mps":     (("limits",), 1.0, 0.3, 1.5, "m/s", "the companion's speed cap on every command (limits.max_speed_mps; at once)"),
    "scout_standoff_m":  (("scout",), 3.0, 2.0, 6.0, "m", "scout: the visit point this far from the buoy (MIN 2)"),
    "arrive_m":          (("scout",), 1.0, 0.5, 2.5, "m", "scout: arrived at the visit point within this"),
    "scout_transit_v":   (("scout",), 0.6, 0.2, 1.2, "m/s", "scout: the transit speed between buoys"),
    "margin_m":          (("scout",), 2.0, 2.0, 8.0, "m", "scout: the field's inset from the RxCourse boundary (pool + course profiles: 4)"),
    "budget_s":          (("scout",), 600.0, 120.0, 1800.0, "s", "scout: the time budget, then RETURN (pool + course profiles: 900)"),
    "lane_m":            (("scout",), 14.0, 6.0, 25.0, "m", "scout: the lawnmower lane spacing"),
    "clear_m":           (("scout",), 1.6, 1.2, 3.0, "m", "scout: the hull-edge clearance it plans with (MIN 1.2)"),
    "guard_stop_m":      (("zed_guard",), 1.5, 1.0, 3.0, "m", "the ZED guard's STOP range (MIN 1.0; at once)"),
    "guard_slow_m":      (("zed_guard",), 3.0, 2.0, 6.0, "m", "the ZED guard's SLOW range (must stay > guard_stop_m; at once)"),
    "slow_v":            (("zed_guard",), 0.3, 0.1, 0.6, "m/s", "the ZED guard's speed cap inside the SLOW range"),
    "backoff_m":         (("zed_guard",), 2.0, 1.0, 4.0, "m", "the ZED guard's back-off after a STOP"),
    "ring_m":            (("zed_guard",), 2.0, 1.0, 4.0, "m", "the keep-clear ring a guard STOP puts on the map"),
}
# v0.1.86.1: the knob's field in its section where the knob's name differs (two sections use the same word)
FIELD = {"scout_standoff_m": "standoff_m", "scout_transit_v": "transit_v", "guard_stop_m": "stop_m", "guard_slow_m": "slow_m"}


def field(key: str) -> str:
    return FIELD.get(key, key)
ORDER = list(KNOBS)
TUNE_STATES = ("MANUAL", "AUTO_HOLD")          # the authority and the vehicle: never while a task runs (AUTO_TASK / SAFE_HOLD)
READ_ONLY = (("PWM limits", "SERVO1..4_MIN / MAX / TRIM"),
             ("stern factors", "the frame's sway mix on the FCU"))
RO_NOTE = "FCU parameters - apply_params.py only (ADR-9: TUNE never touches the FCU)"


def check(key: Any, value: Any) -> Tuple[Optional[str], Optional[float]]:
    """(refusal reason, None) or (None, the value as a float) for a TUNE {key, value}."""
    k = str(key or "")
    if k not in KNOBS:
        return f"TUNE: '{k or '?'}' is not a tuning knob (the whitelist: {' '.join(ORDER)}; PWM limits / stern factors: apply_params.py only)", None
    try:
        v = float(value)
    except (TypeError, ValueError):
        return f"TUNE {k}: value {value!r} is not a number", None
    if v != v or v in (float("inf"), float("-inf")):
        return f"TUNE {k}: value {value!r} is not a number", None
    _, _, lo, hi, unit, _ = KNOBS[k]
    if not (lo <= v <= hi):
        return f"TUNE {k} {v:g}: out of range {lo:g} .. {hi:g} {unit}", None
    return None, v


def cross_check(extra: dict, key: str, value: float) -> Optional[str]:
    """v0.1.86.1: the rules between knobs - the guard's SLOW range stays beyond its STOP range."""
    if key == "guard_slow_m" and value <= current(extra, "guard_stop_m"):
        return f"TUNE guard_slow_m {value:g}: must stay > guard_stop_m {current(extra, 'guard_stop_m'):g}"
    if key == "guard_stop_m" and value >= current(extra, "guard_slow_m"):
        return f"TUNE guard_stop_m {value:g}: must stay < guard_slow_m {current(extra, 'guard_slow_m'):g}"
    return None


def seed(cfg) -> None:
    """v0.1.86.1: the knobs that live outside cfg.extra (limits.max_speed_mps) copied in, so view / save see the profile's value."""
    lim = getattr(cfg, "limits", None)
    if lim is not None and isinstance(getattr(cfg, "extra", None), dict):
        cfg.extra.setdefault("limits", {}).setdefault("max_speed_mps", float(lim.max_speed_mps))


def sync(cfg, guard=None) -> None:
    """v0.1.86.1: the knobs that act at once - limits.max_speed_mps into cfg.limits, zed_guard.* into the running guard's params."""
    ex = getattr(cfg, "extra", None) or {}
    if getattr(cfg, "limits", None) is not None and "max_speed_mps" in (ex.get("limits") or {}):
        cfg.limits.max_speed_mps = float(ex["limits"]["max_speed_mps"])
    if guard is not None and hasattr(guard, "p"):
        guard.p.update({k: v for k, v in (ex.get("zed_guard") or {}).items() if k in guard.p})


def current(extra: dict, key: str) -> float:
    sects, default = KNOBS[key][0], KNOBS[key][1]
    v = (extra.get(sects[0]) or {}).get(field(key), default)
    if v is None and key == "standoff_m":            # v0.1.56 (D6): null = derived from the dock geometry
        return standoff_lines(extra.get("t3"), extra.get("t1"))[0]
    return float(default if v is None else v)


def apply(extra: dict, key: str, value: float) -> Tuple[float, float]:
    """Set the live value in every section the knob belongs to; returns (old, new)."""
    old = current(extra, key)
    for s in KNOBS[key][0]:
        sec = extra.get(s)
        if not isinstance(sec, dict):
            sec = extra[s] = {}
        sec[field(key)] = float(value)
    return old, float(value)


def view(extra: dict, gains=None, base: Optional[dict] = None) -> dict:
    """What the TUNING card shows: every knob with its live value, default, range, unit, tip, and whether it differs from the profile."""
    rows = []
    for k in ORDER:
        sects, d, lo, hi, unit, tip = KNOBS[k]
        v = current(extra, k)
        prof = None if base is None else current(base, k)
        rows.append({"key": k, "sections": list(sects), "field": field(k), "value": v, "default": d, "lo": lo, "hi": hi, "unit": unit, "tip": tip,
                     "changed": prof is not None and abs(prof - v) > 1e-9, "profile": prof})
    ro = [{"name": n, "what": w, "note": RO_NOTE} for n, w in READ_ONLY]
    if gains is not None:
        ro[1]["what"] += " · the companion's mode-20 copy: gains.sway_f_pos %.3g / sway_f_neg %.3g" % (gains.sway_f_pos, gains.sway_f_neg)
    return {"knobs": rows, "read_only": ro, "states": list(TUNE_STATES)}


def diff(base: dict, extra: dict) -> List[str]:
    out = []
    for k in ORDER:
        sects, d = KNOBS[k][0], KNOBS[k][1]
        was = current(base, k)
        now = current(extra, k)
        if abs(was - now) > 1e-9:
            out.append("%s.%s %g -> %g" % ("/".join(sects), field(k), was, now))
    return out


def save(profile_path: str, extra: dict, out_root: str = "logs/launch", stamp: Optional[str] = None) -> Tuple[str, List[str]]:
    """A copy of the loaded profile with the live knob values -> <out_root>/<stamp>/usv_tuned.yaml; (path, diff lines).
    The profile itself is never written."""
    import yaml
    with open(profile_path, "r", encoding="utf-8") as fh:
        base = yaml.safe_load(fh) or {}
    out = copy.deepcopy(base)
    for k in ORDER:
        v = current(extra, k)
        for s in KNOBS[k][0]:
            sec = out.get(s)
            if not isinstance(sec, dict):
                sec = out[s] = {}
            if abs(current(base, k) - v) > 1e-9:
                sec[field(k)] = v
    lines = diff(base, extra)
    d = os.path.join(out_root, stamp or time.strftime("%Y%m%d_%H%M%S"))
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, "usv_tuned.yaml")
    if os.path.abspath(path) == os.path.abspath(profile_path):
        raise ValueError("refusing to overwrite the loaded profile")
    head = ("# usv_tuned.yaml (v0.1.49 TUNING card): a copy of %s with the live tuning values - never the profile itself.\n"
            "# load it: MM_CONFIG_OVERRIDE=%s scripts/run_usv.sh ...   changes: %s\n" % (profile_path, path, "; ".join(lines) or "none"))
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(head + yaml.safe_dump(out, sort_keys=False))
    return path, lines
