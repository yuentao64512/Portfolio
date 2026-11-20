"""v0.1.92 GUI night (13 Nov 2026, Dmitrii: "recheck every step in declaration logic (including one for drone) ... minimise my error with
clicking buttons"): the declaration end to end on the real OCS (in process, the course profile, each launcher preset's REAL launch block),
table-driven, and the mission modifiers' path (launcher / ocs_cli / the panel's `mission` line -> cmd_authority -> the OCS -> START_TASK).

Covers: the tier derivation (m1t1 -> T1 CORE, m1t2 -> ADVANCED, scout follows `declare uav`, T4 tick), a preset NEVER overriding the armed
mission (12 Nov "critical mission mismatches"), the drone in / out of the declaration and its refusals, the declaration frozen while declared,
and the modifiers reaching the boat in the boat's types (look_lock False travels)."""
from __future__ import annotations

import pytest

from optiprimex.ocs import cmd_authority as CA
from optiprimex.tools import launcher as L
from optiprimex.tools import ocs_cli

from .test_mission1_rehearsal import PRESETS, ocs_for, sent, uav_status
from .test_ocs_units import intent

M1 = ("m1t1-gates", "m1t2-uav", "m1t2-both")        # the launcher's buttons 1 · USV only, 2 · UAV only, 3 · UAV + USV


def test_the_three_mission1_presets_are_the_buttons():
    assert PRESETS["m1t1-gates"]["title"].startswith("1 · USV only") and PRESETS["m1t2-uav"]["title"].startswith("2 · UAV only")
    assert PRESETS["m1t2-both"]["title"].startswith("3 · UAV + USV")


# ---- the tier derivation, every preset x every mission edit x the T4 tick ------------------------------------------------------------
TIERS = [  # preset, edits, T1, T4, declare_uav, auto
    ("m1t1-gates", {}, "TIER_CORE", "TIER_NONE", False, True),
    ("m1t1-gates", {"t4": True}, "TIER_CORE", "TIER_CORE", False, True),
    ("m1t1-gates", {"mission": "m1t2"}, "TIER_ADVANCED", "TIER_NONE", False, True),       # uav off in preset 1: no drone declared by itself
    ("m1t2-uav", {}, "TIER_ADVANCED", "TIER_NONE", True, False),
    ("m1t2-uav", {"t4": True}, "TIER_ADVANCED", "TIER_CORE", True, False),
    ("m1t2-uav", {"mission": "m1t1"}, "TIER_CORE", "TIER_NONE", False, False),
    ("m1t2-both", {}, "TIER_ADVANCED", "TIER_NONE", True, True),
    ("m1t2-both", {"mission": "m1t1"}, "TIER_CORE", "TIER_NONE", False, True),          # the 17:08 mismatch
    ("m1t2-both", {"mission": "m1t1", "t4": True}, "TIER_CORE", "TIER_CORE", False, True),
]


@pytest.mark.parametrize("preset,edits,t1,t4,duav,auto", TIERS)
def test_the_tiers_follow_the_mission_not_the_preset_row(preset, edits, t1, t4, duav, auto):
    app = ocs_for(preset, **edits)
    t = app.effective_tiers()
    assert t["T1"] == t1 and t["T4"] == t4
    assert app.declare_uav is duav and app.mission["auto"] is auto
    pv = app.declaration_preview()
    assert pv["tiers"]["T1"] == t1 and pv["tiers"]["T4"] == t4            # the DECLARE text = what goes on the wire
    assert app.state_body()["declaration_checks"]["tiers"]["T1"] == t1  # the console's DECLARE step reads this


@pytest.mark.parametrize("preset", M1)
def test_re_arming_another_mission_moves_t1_with_it(preset):
    """The operator's `mission` (the console's ARM MISSION / the panel) wins over the preset: T1 follows the ARMED mission."""
    app = ocs_for(preset)
    for code, task, tier, want in (("m1t1", "T1", "core", "TIER_CORE"), ("m1t2", "T1", "advanced", "TIER_ADVANCED"),
                                   ("m1t1", "T1", "core", "TIER_CORE")):
        r = app.authority.on_intent("t", intent("MISSION", args={"task": task, "tier": tier, "auto": True}))
        assert r.result == "accepted", r.reason
        assert app.mission["code"] == code and app.effective_tiers()["T1"] == want


@pytest.mark.parametrize("preset", M1)
def test_apply_launch_never_overrides_an_armed_mission(preset):
    """apply_launch (the OCS start) preselects only when nothing is armed: a second call after the operator armed m1t1 keeps m1t1."""
    app = ocs_for(preset)
    app.set_mission("T1", True, {"tier": "core"})
    app.apply_launch()
    assert app.mission["code"] == "m1t1" and app.effective_tiers()["T1"] == "TIER_CORE"


def test_the_declared_tiers_are_frozen_while_declared():
    app = ocs_for("m1t1-gates")
    assert app.declare({}) is None
    for mid, vid in list(app._prepare_msg_ids.items()):
        app.on_ack("t", {"vehicle_id": vid, "body": {"ack_of": mid, "result": "DONE"}})
    assert app.lifecycle.body()["r"] in ("R3", "R4")
    app.set_mission("T1", True, {"tier": "advanced"})                      # re-armed m1t2 AFTER declaring
    assert app.effective_tiers()["T1"] == "TIER_CORE"                      # the declaration in force is what the judge has


# ---- the drone's declaration ------------------------------------------------------------------------------------------------------------
def test_m1t2_without_the_drone_in_the_declaration_is_refused_with_the_fix():
    app = ocs_for("m1t2-both")
    uav_status(app, ready=True)
    ok, _ = app.on_uav("declare", {"on": False})
    assert ok
    why = app.declare({})
    assert why and "declare uav on" in why


def test_the_drone_is_declared_by_the_m1t2_presets_and_its_fence_refusal_is_shown():
    app = ocs_for("m1t2-uav")
    uav_status(app, ready=True, fence={"points": [], "points_n": 0, "ceiling_hae_m": None})   # no FCU fence
    pv = app.declaration_preview()
    assert pv["uav_declared"] is True and pv["uav_refusal"]                # the VEHICLES card's DECLARE row says why
    assert app.declare({}).startswith("UAV:")


def test_the_drone_toggle_off_leaves_the_next_declaration_and_the_heartbeats_follow():
    app = ocs_for("m1t2-both")
    uav_status(app, ready=True)
    assert app.uav_vid in app.decl_vehicles()
    app.on_uav("declare", {"on": False})
    assert app.decl_vehicles() == ["USV1"] and app.effective_tiers()["T1"] == "TIER_ADVANCED"   # m1t2 armed: DECLARE refuses (above)
    app.on_uav("declare", {"on": True})
    assert set(app.decl_vehicles()) == {"USV1", app.uav_vid}


def test_scout_tier_follows_the_drone_toggle():
    app = ocs_for("m1t1-gates")
    app.set_mission("SCOUT", False, {})
    assert app.effective_tiers()["T1"] == "TIER_CORE"
    app.declare_uav = True
    assert app.effective_tiers()["T1"] == "TIER_ADVANCED"


# ---- the mission modifiers (COWORK_HANDOFF addendum 6 §6.3) -----------------------------------------------------------------------------
MODV = [  # key, given, ok, value on the wire
    ("start_mode", "advance360", True, "advance360"), ("start_mode", "advance+360", True, "advance360"), ("start_mode", "straight", True, "straight"),
    ("start_mode", "crab", False, None),
    ("look_lock", "on", True, True), ("look_lock", "off", True, False), ("look_lock", False, True, False), ("look_lock", "maybe", False, None),
    ("circle_exit_tangent", "on", True, True), ("circle_exit_tangent", "off", True, False), ("circle_exit_tangent", True, True, True),
    ("scout_entry_s", 420, True, 420), ("scout_entry_s", "300", True, 300), ("scout_entry_s", 30, False, None), ("scout_entry_s", "x", False, None),
]


@pytest.mark.parametrize("k,v,ok,want", MODV)
def test_a_modifier_is_normalised_to_the_boats_type(k, v, ok, want):
    why, val = CA.mod_value(k, v)
    assert (not why) is ok and (val == want and type(val) is type(want) if ok else val is None)


def test_the_four_keys_are_mission_args_and_match_the_boats_course_profile():
    """The GUI shows MOD_DEFAULTS for a key the operator did not set: they must be what the boat runs with on the course (the course
    profile's t1 block, else the task's DEFAULTS) - else the mods line would lie. If session A changes a course default, change
    cmd_authority.MOD_DEFAULTS and tkconsole/model.MOD_DEFAULTS and the page's MOD_DEF with it (this test says so)."""
    import yaml
    from optiprimex.tools.tkconsole import model as TM
    from optiprimex.usv.tasks import t1_safe_passage as T1
    from .conftest import ROOT
    assert set(CA.MOD_ARGS) <= set(CA.MISSION_ARGS) and CA.MOD_ARGS == ocs_cli.MOD_KEYS == TM.MOD_KEYS
    assert TM.MOD_DEFAULTS == CA.MOD_DEFAULTS
    t1 = yaml.safe_load((ROOT / "profiles" / "course" / "usv_course.yaml").read_text()).get("t1") or {}
    for k, v in CA.MOD_DEFAULTS.items():
        assert k in T1.DEFAULTS, k
        boat = t1.get(k) if t1.get(k) is not None else T1.DEFAULTS[k]
        assert boat == v, (k, boat, v)


CLI = [  # line, the MISSION / START_TASK args expected after cmd_authority
    ("mission m1t1 --auto --look-lock off", {"look_lock": False}),
    ("mission m1t1 --auto --start-mode straight --scout-entry-s 300 --exit-tangent on",
     {"start_mode": "straight", "scout_entry_s": 300, "circle_exit_tangent": True}),
    ("mission m1t1 --auto --lucky --start-mode advance360 --look-lock on --scout-entry-s 420 --exit-tangent off",
     {"start_mode": "advance360", "look_lock": True, "scout_entry_s": 420, "circle_exit_tangent": False, "lucky": True}),
    ("mission m1t2 --auto", {}),
]


@pytest.mark.parametrize("line,want", CLI)
def test_the_mission_line_reaches_the_ocs_in_the_boats_types(line, want):
    app = ocs_for("m1t1-gates")
    action, _vid, args, _t = ocs_cli.intent_spec(ocs_cli.parse(line.split()))
    r = app.authority.on_intent("t", intent(action, args=args))
    assert r.result == "accepted", r.reason
    mods = {k: v for k, v in want.items() if k in CA.MOD_ARGS}
    assert app.mission["mods"] == mods
    assert {k: app.authority.mission_args.get(k) for k in mods} == mods      # a bare ▶ START carries them (False kept)
    if want.get("lucky"):
        assert app.mission["lucky"] is True


def test_a_bad_modifier_is_refused_with_its_choices_and_nothing_changes():
    app = ocs_for("m1t1-gates")
    before = dict(app.mission)
    r = app.authority.on_intent("t", intent("MISSION", args={"task": "T1", "tier": "core", "auto": True, "look_lock": "maybe"}))
    assert r.result == "refused" and "look_lock: on | off" in r.reason and app.mission == before


def test_the_auto_start_sends_the_modifiers_with_start_task():
    app = ocs_for("m1t1-gates")
    app.authority.on_intent("t", intent("MISSION", args={"task": "T1", "tier": "core", "auto": True, "look_lock": "off",
                                                         "scout_entry_s": 300, "start_mode": "straight"}))
    got = []
    app.authority.emit = lambda vid, kind, action, task=None, args=None, **kw: got.append((action, task, dict(args or {}))) or "m"
    app.alert = lambda *a, **k: None
    app.countdown_s = 0
    app.last_status = {"USV1": {"state": "AUTO_HOLD"}}
    app.lifecycle.run_id = 77
    app.declaration_rows = lambda *a, **k: []                                   # the checks are tested elsewhere: the args here
    app._mission_trigger("RunStart verified")
    st = [g for g in got if g[0] == "START_TASK"]
    assert st and st[0][1] == "T1"
    a = st[0][2]
    assert a["look_lock"] is False and a["scout_entry_s"] == 300 and a["start_mode"] == "straight" and a["tier"] == "core"


def test_a_typed_start_task_normalises_too():
    from .test_ocs_units import make_authority
    bus, lc, auth, results = make_authority(bench=False, lc_state="RUNNING")
    auth.last_vehicle_state["USV1"] = "AUTO_HOLD"
    args = ocs_cli.intent_spec(ocs_cli.parse("start_task m1t1 --look-lock off --exit-tangent on".split()))[2]
    r = auth.on_intent("t", intent("START_TASK", "T1", args=args))
    assert r.result == "accepted", r.reason
    a = [m for m in bus.published if isinstance(m.get("body"), dict) and m["body"].get("action") == "START_TASK"][-1]["body"]["args"]
    assert a["look_lock"] is False and a["circle_exit_tangent"] is True
    r = auth.on_intent("t", intent("START_TASK", "T1", args={"look_lock": "perhaps"}))
    assert r.result == "refused" and "look_lock: on | off" in r.reason


def test_a_bare_start_carries_the_armed_modifiers():
    from .test_ocs_units import make_authority
    bus, lc, auth, results = make_authority(bench=False, lc_state="RUNNING")
    r = auth.on_intent("t", intent("MISSION", args={"task": "T1", "tier": "core", "look_lock": "off", "start_mode": "straight"}))
    assert r.result == "accepted", r.reason
    assert auth.mission_args["look_lock"] is False and "look_lock=False" in r.reason      # the answer line names them
    r = auth.on_intent("t", intent("START_TASK", None, args={}))
    assert r.result == "accepted", r.reason
    a = [m for m in bus.published if isinstance(m.get("body"), dict) and m["body"].get("action") == "START_TASK"][-1]["body"]
    assert a["task"] == "T1" and a["args"]["look_lock"] is False and a["args"]["start_mode"] == "straight"


LAUNCH = [  # the launcher's cfg edits on preset 1 -> the launch block's extra / the OCS's mission mods
    ({}, {}),
    ({"look_lock": False}, {"look_lock": False}),
    ({"start_mode": "straight", "scout_entry_s": 300}, {"start_mode": "straight", "scout_entry_s": 300}),
    ({"circle_exit_tangent": True, "lucky": True}, {"circle_exit_tangent": True}),
    ({"look_lock": "profile"}, {}),
]


@pytest.mark.parametrize("edits,mods", LAUNCH)
def test_the_launcher_carries_the_modifiers_to_the_auto_start(edits, mods):
    cfg = dict(PRESETS["m1t1-gates"], preset="m1t1-gates", **edits)
    assert not [e for e in L.validate(cfg) if any(k in e for k in CA.MOD_ARGS)]
    app = ocs_for("m1t1-gates", **edits)
    assert app.mission["mods"] == mods and {k: app.authority.mission_args.get(k) for k in mods} == mods
    if edits.get("lucky"):
        assert app.mission["lucky"] is True


def test_the_launcher_refuses_a_bad_modifier():
    cfg = dict(PRESETS["m1t1-gates"], preset="m1t1-gates", start_mode="sideways")
    assert any("start_mode" in e for e in L.validate(cfg))


def test_the_uav_only_preset_carries_no_modifiers_and_they_never_reach_a_non_t1_task():
    lb = L.launch_block(dict(PRESETS["m1t2-uav"], preset="m1t2-uav"))
    assert not set(lb.get("extra") or {}) & set(CA.MOD_ARGS)
    lb = L.launch_block(dict(PRESETS["m1t1-gates"], preset="m1t1-gates", mission="m3t1", look_lock=False))
    assert "look_lock" not in (lb.get("extra") or {})
