"""v0.1.94.3 (Dmitrii 14 Nov: "a velocity modifier on the OCS ... so we could speed up during the trial on the spot"; "everything but
3 different knobs", 0.5-2.0 in 0.25 steps, "cap 1.5 m/s"): three live speed multipliers, in force at once in ANY state (a running task
included - unlike `tune`, whose knobs wait for the next task start).

  cruise - FollowPath legs: the run-ins, transit, approach, scout, exit (every task's path following)
  circle - Orbit: the entry / exit circles (forward speed AND yaw rate, so the circle keeps its radius)
  gate   - T1 THROUGH: the pass between the gate buoys
Scans and station keeping turn in place: never scaled.

The task base applies it to the behaviour's output each tick (after the under-way law, before the speed cap). The cap: k <= 1 keeps
limits.max_speed_mps; k > 1 lifts it to max_speed_mps * k, never above V_CEIL (1.5 m/s). The ZED guard's SLOW / STOP act after the task
command and are not scaled. Stored in cfg.extra["speed_k"]; status/usv tuning.speed_k shows it; never saved into a profile.
"""
from __future__ import annotations

from typing import Any, Dict, Optional, Tuple

CLASSES = ("cruise", "circle", "gate")
K_MIN, K_MAX, K_STEP = 0.5, 2.0, 0.25
V_CEIL = 1.5


def check(args: Dict[str, Any]) -> Tuple[str, Dict[str, float]]:
    """(why, {class: k}) - why empty when fine. args: {cruise|circle|gate|all: k} or {reset: true}."""
    if (args or {}).get("reset"):
        return "", {c: 1.0 for c in CLASSES}
    out: Dict[str, float] = {}
    for key, v in (args or {}).items():
        if key not in CLASSES + ("all",):
            return f"speed: unknown knob {key!r} (cruise | circle | gate | all)", {}
        try:
            k = round(float(v), 3)
        except (TypeError, ValueError):
            return f"speed {key}: {v!r} is not a number", {}
        if not (K_MIN <= k <= K_MAX):
            return f"speed {key}: x{k:g} outside {K_MIN:g}-{K_MAX:g}", {}
        for c in (CLASSES if key == "all" else (key,)):
            out[c] = k
    if not out:
        return "speed: nothing to set (cruise | circle | gate | all K, or reset)", {}
    return "", out


def current(extra: Optional[dict]) -> Dict[str, float]:
    have = (extra or {}).get("speed_k") or {}
    return {c: float(have.get(c, 1.0)) for c in CLASSES}


def apply(extra: dict, ks: Dict[str, float]) -> Dict[str, float]:
    cur = current(extra)
    cur.update(ks)
    extra["speed_k"] = cur
    return cur


def k_of(cfg, cls: Optional[str]) -> float:
    if not cls:
        return 1.0
    return float(current(getattr(cfg, "extra", None)).get(cls, 1.0))


def line(ks: Dict[str, float]) -> str:
    return " · ".join(f"{c} x{ks.get(c, 1.0):g}" for c in CLASSES)
