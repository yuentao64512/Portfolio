"""declaration_checks: the RunDeclaration checks of ROADMAP WP9c (v0.1.41), one pure module.

Input: the tiers we declare (or are about to), the mission (a code such as m1t2, or a wire task + tier), the run state.
Output: a list of rows {code, severity: hard | soft, scope: declaration | mission, text} - each text says what to type next.
No I/O, no clock, no bus: the authority (refuse or warn by `declaration.policy`), the OCS state (the page's RUN CHECKLIST),
playbook / ocs_cli `next` and op-tui's `next:` all call the same functions, at any step of the start sequence (enable_auto,
declare, RunStart, start_task, mission preselect - the order is not assumed, see ROADMAP 7d).

Handbook (handbook/RobotX-2026_Team-Handbook-20261023.md):
- 3.4.10: the RunDeclaration carries the tier attempted for Task 1..4; TIER_NONE when not attempting, never TIER_UNKNOWN.
- 3.4.16: pre-run checklist - "the intended tier for every task is correct in RunDeclaration".
- 3.3.1 (Cross-cutting, Dynamic Adaptability) + 3.3.5: Task 4 (Dynamic Incident Response) is how RoboCommand exercises the
  cross-cutting capability every system must perform during any run - our only task the handbook makes part of every run,
  hence MANDATORY = ("T4",). The handbook makes no other task compulsory (3.3: "teams are encouraged to develop a strategy").
- 4.2 / 4.2 Cross-Cutting: points come from the judged run; a test mission (m0t*) or the bench `goto` scores nothing there.

The mission code (playbook's catalogue, v0.1.19) is m<RobotX task><tier>: m4t0 = Task 4 core, m1t2 = Task 1 advanced,
m0tN = our test missions, `goto` = the bench waypoint. The task and the tier a mission needs are read off its code here
(mission_code below is the OCS's copy of that table, moved from ocs_app unchanged).
"""
from __future__ import annotations

from typing import Dict, Iterable, List, Optional

TASKS = ("T1", "T2", "T3", "T4")
TIERS = ("TIER_NONE", "TIER_CORE", "TIER_ADVANCED", "TIER_DISRUPTIVE")            # rc_pb common.TaskTier minus TIER_UNKNOWN
TASK_NAMES = {"T1": "Safe Passage", "T2": "Infrastructure Survey and Repair", "T3": "Coordinated Logistics",
              "T4": "Dynamic Incident Response"}
DEFAULT_TIERS = {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_CORE"}   # ocs_app's fallback, unchanged
MANDATORY = ("T4",)                                                                   # handbook 3.3.1 + 3.3.5 (see above)
POLICIES = ("refuse", "warn")
DEFAULT_POLICY = "warn"                                                               # a profile without `policy`: as before
TIER_OF_DIGIT = {"1": "TIER_CORE", "2": "TIER_ADVANCED", "3": "TIER_DISRUPTIVE"}
JUDGED_R = ("R3", "R4", "R5")                                                         # declared .. running: a judged run
FIX_PROFILE = "set declaration.tiers in the OCS profile (profiles/<pool|bench|course>/ocs_*.yaml), restart run_ocs.sh (end_run first if declared), then declare"


def mission_code(task: Optional[str], extra: Optional[dict] = None) -> Optional[str]:
    """The operator's code for a wire task + tier (the catalogue lives in tools.playbook; this is the OCS's own small copy
    so ocs/ never imports tools/)."""
    if not task:
        return None
    extra = extra or {}
    tier = str(extra.get("tier") or "core").lower()
    base = {"T4": "m4t0", "T1": "m1t2" if tier == "advanced" else "m1t1", "T3": "m3t1", "GOTO": "goto",
            "TEST_MANUAL": "m0t0", "TEST_AUTO": "m0t1", "TEST_PATH": "m0t2", "TEST_SWAY": "m0t3",
            "BOX_INSPECT": "m0t4", "TEST_DETECT": "m0t8", "TEST_GUARD": "m0t9"}.get(task, task.lower())        # v0.1.79: SCOUT -> scout (its own word)
    return base + ("sim" if extra.get("sim") else "")


def mission_needs(mission: Optional[str], tier: Optional[str] = None) -> Optional[dict]:
    """A mission code (m1t2, m3t1sim, goto, m0t2) or a wire task (+ its args tier) -> {code, kind, task, tier}.
    kind: judged (task + the declared tier it needs) | test | goto | other; None for no mission."""
    if not mission:
        return None
    m = str(mission).strip()
    if not (m[:1].lower() == "m" and len(m) >= 4 and m[1].isdigit() and m[2].lower() == "t") and m.lower() != "goto":
        w = m.upper()
        if w.endswith("SIM") and len(w) > 3:                                               # T1sim: the wire task + the suffix
            w = w[:-3]
        if w.startswith("TEST_") or w == "BOX_INSPECT":                                    # every m0t* (TEST_ORBIT has no code here)
            return {"code": "m0t6" if w == "TEST_ORBIT" else (mission_code(w) or w.lower()), "kind": "test", "task": None, "tier": None}
        m = mission_code(w, {"tier": tier}) or m                                           # T1 + advanced -> m1t2
    code = m.lower()
    base = code[:-3] if code.endswith("sim") and len(code) > 3 else code
    if base == "goto":
        return {"code": code, "kind": "goto", "task": None, "tier": None}
    if base == "scout":                                      # v0.1.79 (WP35): declared as Task 1 - core alone, advanced with the drone
        adv = str(tier or "").lower().replace("tier_", "") == "advanced"
        return {"code": code, "kind": "judged", "task": "T1", "tier": "TIER_ADVANCED" if adv else "TIER_CORE"}
    if len(base) >= 4 and base[0] == "m" and base[1].isdigit() and base[2] == "t":
        if base == "m4t0":                                   # v0.1.71 (WP27): T4 alone, declared as m1t1 + T4 core - it needs T4 at core
            return {"code": code, "kind": "judged", "task": "T4", "tier": "TIER_CORE"}
        if base[1] == "0":
            return {"code": code, "kind": "test", "task": None, "tier": None}
        if base[1] in "1234" and base[3:] in TIER_OF_DIGIT:
            return {"code": code, "kind": "judged", "task": "T" + base[1], "tier": TIER_OF_DIGIT[base[3:]]}
    return {"code": code, "kind": "other", "task": None, "tier": None}


def policy_of(declaration_cfg: Optional[dict]) -> str:
    p = str((declaration_cfg or {}).get("policy") or DEFAULT_POLICY).strip().lower()
    return p if p in POLICIES else DEFAULT_POLICY


def _row(code: str, severity: str, scope: str, text: str) -> dict:
    return {"code": code, "severity": severity, "scope": scope, "text": text}


def check_tiers(tiers: Optional[dict], mandatory: Iterable[str] = MANDATORY, mandatory_hard: bool = False) -> List[dict]:
    """The declaration on its own: every task has a valid tier, TIER_UNKNOWN never, a mandatory task not TIER_NONE."""
    tiers = dict(tiers or {})
    rows: List[dict] = []
    for t in TASKS:
        v = tiers.get(t)
        name = f"{t} ({TASK_NAMES[t]})"
        if v in (None, ""):
            rows.append(_row("tier_missing", "hard", "declaration",
                             f"{name} has no tier in the declaration (the wire would send TIER_NONE silently): {FIX_PROFILE} "
                             f"- {t}: TIER_NONE when not attempted (handbook 3.4.10)"))
        elif str(v).upper() == "TIER_UNKNOWN":
            rows.append(_row("tier_unknown", "hard", "declaration",
                             f"{name} is declared TIER_UNKNOWN, which the handbook forbids (3.4.10: use TIER_NONE when not attempting): {FIX_PROFILE}"))
        elif str(v).upper() not in TIERS:
            rows.append(_row("tier_invalid", "hard", "declaration",
                             f"{name} tier '{v}' is not one of {' / '.join(TIERS)}: {FIX_PROFILE}"))
        elif t in mandatory and str(v).upper() == "TIER_NONE":
            # v0.1.44 (OCS_V2_SPEC §5, §13): T4 is a modifier - soft ("are you sure?") until RoboNation confirms it is required, then
            # `declaration.t4_required: true` in the course profile makes it hard again (one line)
            rows.append(_row("mandatory_none", "hard" if mandatory_hard else "soft", "declaration",
                             f"{name} off (TIER_NONE) while the handbook expects it in every run (3.3.1 Dynamic Adaptability, 3.3.5) - "
                             f"are you sure? To take it: {FIX_PROFILE} - {t}: TIER_CORE"))
    return rows


def check_mission(tiers: Optional[dict], mission: Optional[str], tier: Optional[str] = None, judged: bool = False) -> List[dict]:
    """The mission against the declaration: its task declared with the same tier; a test mission / goto not in a judged run."""
    n = mission_needs(mission, tier)
    if n is None:
        return []
    tiers = dict(tiers or {})
    code = n["code"]
    if n["kind"] in ("test", "goto") and judged:
        what = "goto (the bench waypoint)" if n["kind"] == "goto" else f"{code} (a test mission)"
        return [_row(f"{n['kind']}_in_run", "hard", "mission",
                     f"{what} inside a judged run scores nothing (handbook 4.2) and RoboCommand sees no declared task: "
                     f"`mission m4t0` (or the judged mission you declared), or end_run / reset and run {code} in TEST MODE (no run)")]
    if n["kind"] != "judged":
        return []
    t, need = n["task"], n["tier"]
    have = str(tiers.get(t) or "TIER_NONE").upper()
    if have == need:
        return []
    if have == "TIER_NONE":
        return [_row("task_not_declared", "hard", "mission",
                     f"{code} runs {t} ({TASK_NAMES[t]}) but the declaration says {t} TIER_NONE: {FIX_PROFILE} - {t}: {need}; "
                     f"or pick a declared mission (`mission m4t0`)")]
    better = next((c for c, d in (("1", "TIER_CORE"), ("2", "TIER_ADVANCED"), ("3", "TIER_DISRUPTIVE")) if d == have), None)
    alt = f"`mission m{t[1]}t{better}`" if better else "a mission of the declared tier"
    return [_row("tier_mismatch", "hard", "mission",
                 f"{code} needs {t} {need}, the declaration says {t} {have} (handbook 3.4.16: the intended tier must be correct): "
                 f"pick {alt}, or {FIX_PROFILE} - {t}: {need}")]


def is_judged(r: Optional[str], step: str = "state", test_mode: bool = False, auto: bool = False) -> bool:
    """Is the mission headed for a judged run? Declared .. running (R3-R5), the declare step itself, a mission armed with --auto
    (it starts on RunStart), or no TEST MODE at all (the course: every task runs in a run)."""
    return r in JUDGED_R or step == "declare" or bool(auto) or not test_mode


def evaluate(tiers: Optional[dict], mission: Optional[str] = None, tier: Optional[str] = None, *, r: Optional[str] = None,
             step: str = "state", test_mode: bool = False, auto: bool = False, mandatory: Iterable[str] = MANDATORY,
             mandatory_hard: bool = False) -> List[dict]:
    """Every row for this moment: the declaration rows, then the mission rows. step: declare | start_task | mission | goto | state."""
    rows = check_tiers(tiers, mandatory, mandatory_hard)
    rows += check_mission(tiers, mission, tier, is_judged(r, step, test_mode, auto))
    return rows


def blocking(rows: List[dict], policy: str, scope: Optional[str] = None) -> List[dict]:
    """The rows that refuse (policy refuse, severity hard; only `scope` rows when given)."""
    if policy != "refuse":
        return []
    return [x for x in rows if x["severity"] == "hard" and (scope is None or x["scope"] == scope)]


def summary(rows: List[dict]) -> str:
    return " | ".join(x["text"] for x in rows)


def from_state(ocs_state: Optional[dict], mission: Optional[str] = None, step: str = "state") -> List[dict]:
    """For the consoles: the rows for `mission` (default: the preselected one) from ocs/state's `declaration_checks` block.
    An ocs/state without the block (an older ocs_app) gives no rows."""
    o = ocs_state or {}
    d = o.get("declaration_checks")
    if not isinstance(d, dict):
        return []
    mi = o.get("mission") or {}
    if mission is None:
        mission = mi.get("code") or mi.get("task")
        auto = bool(mi.get("auto"))
    else:
        auto = False
    return evaluate(d.get("tiers"), mission, None, r=o.get("r"), step=step, test_mode=bool(o.get("test_mode")), auto=auto,
                    mandatory=d.get("mandatory") or MANDATORY, mandatory_hard=bool(d.get("mandatory_hard")))


def state_block(tiers: Optional[dict], declared: bool, policy: str, mandatory: Iterable[str], rows: List[dict],
                mandatory_hard: bool = False) -> Dict[str, object]:
    """What ocs/state carries (`declaration_checks`): enough for any console to re-run evaluate() for another mission."""
    return {"policy": policy, "tiers": dict(tiers or {}), "source": "declared" if declared else "profile",
            "mandatory": list(mandatory), "mandatory_hard": bool(mandatory_hard), "rows": rows, "blocking": [x["code"] for x in blocking(rows, policy)]}
