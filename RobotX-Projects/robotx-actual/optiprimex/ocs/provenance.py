"""provenance: where every value on the OCS page comes from, and what every button does (v0.1.73, WP29 = docs/OVERNIGHT_2026-11-09_GUI.md N5).

Dmitrii's point 2: every value, chip and card header carries *topic · publisher · rate · threshold* (the page adds the age it sees), every
button *the word · to whom · what answers*. Generated from `schemas/bodies.schema.json` (the vehicle's status fields) + one hand table for
the rest; the page (ocs_web.html `provApply`) fills a `title` from it wherever an element has none, and a DOM-walk test asserts that no value
element is left without one. Nothing here is operational: text only.
"""
from __future__ import annotations

import json
import os
import re
from typing import Dict

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

STATUS = ("status/usv", "the mission manager, through the boat→hub bridge", "2 Hz", "stale > 3 s")
LOCAL = ("local/usv/mm/state", "the mission manager (the boat broker, never bridged)", "5 Hz", "stale > 2 s")
OCS = ("ocs/state", "ocs_app (this Mac)", "1 Hz", "stale > 3 s")
MAP = ("telemetry/usv/map", "the mission manager's map digest, bridged", "2 Hz", "stale > 3 s")
PERC = ("local/usv/perception/health", "the AGX's observer / cv_publish (or the sim CV)", "1 Hz", "stale > 2 s")
HOSTS = ("local/usv/host/<nx|agx> -> telemetry", "host_health on each Jetson (the bench: simulated)", "0.5 Hz", "stale > 10 s")
CANNON = ("local/usv/cannon/status", "the cannon Pi's status publisher (sim: fake_cannon_pi)", "2 Hz", "stale > 5 s")
UAV = ("status/uav", "our UAV adapter (their Orin's telemetry)", "2 Hz", "stale > 5 s")
JUDGE = ("robocommand/robotx/<team>/...", "the judge client in ocs_app (fake / rc-test / RoboCommand)", "as sent", "-")
SESSION = ("ocs/session", "the page server's SessionManager", "1 Hz", "stale > 6 s")
PAGE = ("/state.json", "the page server (ocs_web)", "1 Hz", "-")


def _fmt(p) -> str:
    return " · ".join(p)


def status_fields() -> Dict[str, str]:
    """status/<v>.<field> for every field of the schema's status body."""
    try:
        with open(os.path.join(HERE, "schemas", "bodies.schema.json")) as fh:
            props = json.load(fh)["$defs"]["status"].get("properties") or {}
    except (OSError, ValueError, KeyError):
        props = {}
    return {k: _fmt((f"status/usv.{k}",) + STATUS[1:]) for k in props}


# a label shown on the page (lower case, no punctuation) -> its source
HAND: Dict[str, tuple] = {
    "state": STATUS, "vehicle": STATUS, "mode": STATUS, "armed": STATUS, "kill": STATUS, "fcu": STATUS, "task": STATUS, "stage": STATUS,
    "hdg": STATUS, "sog": STATUS, "heading sog": STATUS, "lat": STATUS, "lon": STATUS, "fault": STATUS, "hold": STATUS, "rc6": STATUS,
    "run": OCS, "r": OCS, "judge": OCS, "reporting": OCS, "declaration": OCS, "mission": OCS, "course": OCS, "heartbeats": OCS,
    "map age s": MAP, "map age": MAP, "buoys": MAP, "gates": MAP, "gates detected": MAP, "gates passed": MAP, "entry found": MAP,
    "exit found": MAP, "faces": MAP, "windows": MAP, "windows lit": MAP, "source": MAP, "age": MAP, "detections": MAP,
    "perception": PERC, "perception age s": PERC, "camera": PERC, "boxes": PERC,
    "nx": HOSTS, "agx": HOSTS, "cpu": HOSTS, "temp": HOSTS,
    "cannon": CANNON, "pi": CANNON, "packets": CANNON, "lock": CANNON,
    "uav": UAV, "items": UAV, "scan": UAV, "merged": UAV,
    "session": SESSION,
}

# the page's words: to whom and what answers (op-tui's grammar; the OCS's command authority decides first)
TO_VEHICLE = "→ USV1's mission manager (cmd/usv) · ack within 5 s (else TIMED OUT)"
WORDS: Dict[str, str] = {
    "hold": f"intent hold {TO_VEHICLE} · AUTO_SAFE_HOLD, the task kept",
    "resume": f"intent resume {TO_VEHICLE} · back to AUTO_TASK (with --step: the next stage)",
    "abort": f"intent abort {TO_VEHICLE} · AUTO_HOLD, TASK_NONE",
    "skip": f"intent skip {TO_VEHICLE} · free run only: the stage's behaviour ends",
    "start_task": f"intent start_task {TO_VEHICLE} · AUTO_TASK (judged missions need R5)",
    "mission": "intent mission → ocs_app (the preselection) · accepted at once; no vehicle command",
    "declare": "intent declare → ocs_app → PREPARE to each vehicle, then the RunDeclaration to the judge · R3 when every PREPARE is DONE",
    "enable_auto": f"intent enable_auto {TO_VEHICLE} · AUTO_HOLD (STATE_AUTO on the heartbeat)",
    "disable_auto": f"intent disable_auto {TO_VEHICLE} · MANUAL",
    "end_run": "intent end_run → ocs_app → STOP_RUN to every vehicle · R6",
    "reset": "intent reset → ocs_app → RESET to every vehicle · R2",
    "stop_reset": "the STOP & RESET macro (■ STOP RUN): abort · disable_auto · fcu_mode HOLD · disarm · reset - each step's ack shown",
    "fcu_mode": f"intent fcu_mode {TO_VEHICLE} → the FCU (DO_SET_MODE) · the FCU's COMMAND_ACK decides",
    "arm": f"intent arm (asks first) {TO_VEHICLE} → the FCU · its pre-arm checks decide",
    "disarm": f"intent disarm {TO_VEHICLE} → the FCU · COMMAND_ACK",
    "report": "intent report on|off → ocs_app · heartbeats to the judge start / stop at once",
    "session": "intent session → ocs_app (the guard) → the page server on ocs/session/cmd · its state on ocs/state.session",
    "save_point": f"intent save_point {TO_VEHICLE} · the points store (usv_ui/points.json)",
    "home": f"intent home {TO_VEHICLE} · RETURN_HOME, then AUTO_HOLD at HOME",
    "declare uav": "intent declare_uav → ocs_app · the next declaration carries the UAV",
}


def table() -> dict:
    """What the page loads once (META.provenance): {labels: {label: text}, status: {field: text}, words: {word: text}, sources: {...}}."""
    labels = {k: _fmt(v) for k, v in HAND.items()}
    return {"labels": labels, "status": status_fields(), "words": dict(WORDS),
            "sources": {n: _fmt(v) for n, v in (("status", STATUS), ("local", LOCAL), ("ocs", OCS), ("map", MAP), ("perception", PERC),
                                                 ("hosts", HOSTS), ("cannon", CANNON), ("uav", UAV), ("judge", JUDGE), ("session", SESSION),
                                                 ("page", PAGE))},
            "button_default": "a page control - nothing is sent", "value_default": _fmt(PAGE)}


def label_key(text: str) -> str:
    return re.sub(r"[^a-z0-9 ]+", "", str(text).lower()).strip()
