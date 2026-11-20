"""v0.1.92 GUI night (13 Nov 2026, Dmitrii: "make extensive tests that there are NO PROBLEMS with any button ... everything is streamlined to
minimise my error with clicking buttons in the interface and them being stuck. On the pop-up status bar I want to see not only pixhawk
mode / status, but also the micro stage of the mission and which might be next."). docs/GUI_NOTES_2026-11-13.md is the audit table.

The tk console's mission-1 ladder for the three presets x every phase of the run (which step is live, which buttons it shows, what each
sends, its confirm), the "mission modifiers" subpanel (COWORK_HANDOFF addendum 6 §6.3: shown, frozen from ENABLE AUTO on, each button's
line accepted by the real OCS), the run banner (FCU line, the boat's micro-stage, next, an older boat without micro), the double-click guard
and the never-stuck press colour, the OCS page's twins of the same rules (node), and the window itself (skips without a display)."""
from __future__ import annotations

import copy
import json
import re
import shutil
import subprocess
import time

import pytest

from optiprimex.ocs import cmd_authority as CA
from optiprimex.tools import ocs_cli
from optiprimex.tools.tkconsole import model as M
from optiprimex.tools.tkconsole.model import ConsoleModel
from optiprimex.tools.tkconsole.toasts import BUSY_S, STUCK_S, PressBook, ToastBook, verdict

from .conftest import ROOT

NODE = shutil.which("node")
PAGE = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
UAV = {"configured": True, "declare": True, "declared": False, "vehicle_id": "UAV1", "report_on": True, "pilot_ready": True}
PRESET_MISSION = {"usv": ({"code": "m1t1", "task": "T1", "auto": True}, {}),
                  "uav": ({"code": "m1t2", "task": "T1", "auto": False}, UAV),
                  "both": ({"code": "m1t2", "task": "T1", "auto": True}, UAV)}
PROFILE = {"usv": "USV only", "uav": "UAV only · boat holds", "both": "UAV + USV"}


def state(preset="usv", r="R2", v="MANUAL", armed=True, ui=False, ts=None, tn=None, mission=None, uav=None, micro=None, gate=None,
          stalled=False, clear=True, verdicts=None, mode="HOLD"):
    mi, u = PRESET_MISSION[preset]
    mi = dict(mi, **(mission or {}))
    u = dict(u, **(uav or {})) if (u or uav) else {}
    if u and r in ("R3", "R4", "R5"):
        u["declared"] = True
    st = {"state": v, "armed": armed, "mode": mode, "task_name": tn, "task_state": ts}
    o = {"r": r, "run_id": 7 if r in ("R5", "R6") else None, "declaration_seq": 3 if r != "R2" else 0, "mission": mi,
         "usv_flow": {"heartbeat": True, "ready": True}, "uav": u, "auto_before_declare": True, "stalled": stalled,
         "declaration_checks": {"tiers": {"T1": "TIER_ADVANCED" if mi["code"] == "m1t2" else "TIER_CORE", "T2": "TIER_NONE",
                                          "T3": "TIER_NONE", "T4": "TIER_NONE"}}}
    return {"status": st, "ocs": o, "ocs_age": 0.1, "bar": {"word": "ARMED"}, "clearance": {"clear": clear, "blocked": [] if clear else ["judge"]},
            "ui_active": ui, "uav_gate": gate, "micro": micro, "verdicts": verdicts or {}}


def titles(v):
    return [s.title.split(" —")[0] for s in v.steps]


def step(v, name):
    return next(s for s in v.steps if s.title.startswith(name))


def now(v):
    return [s.title.split(" —")[0] for s in v.steps if s.state == "now"]


def words(s):
    return [b.word for b in s.buttons]


# ---- 1 · the ladder for every preset x every phase ------------------------------------------------------------------------------------------
LADDER = [  # preset, state kwargs, the live steps, {step: its button words}
    ("usv", dict(ui=True), ["handover"], {"crew": ["report off usv"]}),
    ("usv", dict(armed=False), ["ENABLE AUTO"], {"ENABLE AUTO": ["arm", "enable_auto"]}),
    ("usv", dict(), ["ENABLE AUTO"], {"ENABLE AUTO": ["enable_auto"]}),
    ("usv", dict(v="AUTO_HOLD"), ["DECLARE"], {"DECLARE": ["declare"], "ENABLE AUTO": []}),
    ("usv", dict(r="R4", v="AUTO_HOLD"), ["RunStart"], {"DECLARE": []}),
    ("usv", dict(r="R4", v="AUTO_HOLD", stalled=True), ["DECLARE", "RunStart"], {"DECLARE": ["declare"]}),   # redeclare after a stall
    ("usv", dict(r="R5", v="AUTO_HOLD"), ["the boat", "END RUN"], {"the boat": ["start_task"], "END RUN": ["end_run"]}),
    ("usv", dict(r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT"), ["the boat", "END RUN"], {"the boat": ["hold", "abort"], "END RUN": ["end_run"]}),
    ("usv", dict(r="R5", v="AUTO_SAFE_HOLD", tn="T1", ts="TRANSIT"), ["the boat", "END RUN"], {"the boat": ["hold", "abort"]}),
    ("usv", dict(r="R6", v="AUTO_HOLD"), ["DECLARE"], {"DECLARE": ["declare"], "END RUN": []}),                  # attempt 2
    ("uav", dict(), ["ENABLE AUTO"], {"crew": ["report off usv", "report off uav"], "ENABLE AUTO": ["enable_auto"]}),
    ("uav", dict(v="AUTO_HOLD"), ["DECLARE"], {"DECLARE": ["declare"]}),
    ("uav", dict(r="R5", v="AUTO_HOLD"), ["the drone", "END RUN"], {"the drone": [], "END RUN": ["end_run"]}),
    ("both", dict(v="AUTO_HOLD"), ["DECLARE"], {"DECLARE": ["declare"]}),
    ("both", dict(r="R5", v="AUTO_TASK", tn="T1", ts="WAIT_UAV_MAP"), ["the drone", "the boat", "END RUN"],
     {"the drone": ["uav scan done"], "the boat": ["hold", "abort"]}),
]


@pytest.mark.parametrize("preset,kw,live,btns", LADDER)
def test_the_ladder_shows_one_live_path(preset, kw, live, btns):
    v = ConsoleModel().apply(state(preset, **kw))
    assert v.profile == PROFILE[preset]
    assert now(v) == live, [(s.title[:12], s.state) for s in v.steps]
    for name, want in btns.items():
        assert words(step(v, name)) == want, name
    order = titles(v)
    assert order.index("ENABLE AUTO") < order.index("DECLARE") < order.index("RunStart") < order.index("END RUN")   # the course order
    for s in v.steps:
        for b in s.buttons:
            assert b.key.startswith(f"lad{s.n}-")                                              # the generic slots the window builds
            if b.word.split()[0] not in ("mission",):
                assert M._verdict({}, b.word) == (True, "", "")                                # no verdict = pressable (never silently grey)


def test_the_shapes_per_preset():
    assert titles(ConsoleModel().apply(state("usv"))) == ["handover", "crew", "ENABLE AUTO", "DECLARE", "RunStart", "the boat", "END RUN"]
    assert titles(ConsoleModel().apply(state("uav"))) == ["handover", "crew", "ENABLE AUTO", "DECLARE", "RunStart", "the drone", "END RUN"]
    assert titles(ConsoleModel().apply(state("both"))) == ["handover", "crew", "ENABLE AUTO", "DECLARE", "RunStart", "the drone", "the boat",
                                                           "END RUN"]


CONFIRMS = [  # the presses that ask first: preset, state kwargs, step, word
    ("usv", dict(armed=False), "ENABLE AUTO", "arm"),
    ("usv", dict(), "crew", "report off usv"),
    ("uav", dict(), "crew", "report off uav"),
    ("both", dict(r="R5", v="AUTO_TASK", tn="T1", ts="WAIT_UAV_MAP"), "the drone", "uav scan done"),
]


@pytest.mark.parametrize("preset,kw,name,word", CONFIRMS)
def test_the_risky_presses_ask_first(preset, kw, name, word):
    b = next(b for b in step(ConsoleModel().apply(state(preset, **kw)), name).buttons if b.word == word)
    assert b.confirm


def test_no_confirm_on_the_plain_steps():
    v = ConsoleModel().apply(state("usv", v="AUTO_HOLD"))
    assert not step(v, "DECLARE").buttons[0].confirm
    v = ConsoleModel().apply(state("usv"))
    assert not next(b for b in step(v, "ENABLE AUTO").buttons if b.word == "enable_auto").confirm


def test_a_refused_press_is_greyed_with_its_reason_and_the_hint():
    vd = {"ENABLE_AUTO": {"ok": False, "why": "FCU not armed", "hint": "arm: the bar's ARM"}, "DECLARE": {"ok": False, "why": "no course"}}
    v = ConsoleModel().apply(state("usv", verdicts=vd))
    b = next(b for b in step(v, "ENABLE AUTO").buttons if b.word == "enable_auto")
    assert not b.ok and b.why == "FCU not armed" and b.hint.startswith("arm")
    v = ConsoleModel().apply(state("usv", v="AUTO_HOLD", verdicts=vd))
    assert not step(v, "DECLARE").buttons[0].ok and step(v, "DECLARE").buttons[0].why == "no course"


def test_every_ladder_and_tile_word_has_a_verdict_or_is_free():
    """A button whose word maps to no playbook action is never greyed by a verdict: list them, so a typo can't hide one."""
    v = ConsoleModel().apply(state("both", r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT"))
    ws = [b.word for s in v.steps for b in s.buttons] + [b.word for b in v.usv_buttons + v.uav_buttons]
    free = {w for w in ws if M.WORD_ACTION.get(" ".join(w.split()[:2]) if w.startswith("declare uav") else w.split()[0]) is None}
    assert free <= {"stop_reset"}, free


def test_a_drone_still_getting_ready_never_hides_enable_auto_and_declare_says_why():
    """v0.1.92 fix: ENABLE AUTO is the boat's step - the drone's pilot ready not back yet used to leave the step 'next' (no button)."""
    v = ConsoleModel().apply(state("both", uav={"pilot_ready": False}))
    assert step(v, "ENABLE AUTO").state == "now" and "enable_auto" in words(step(v, "ENABLE AUTO"))
    v = ConsoleModel().apply(state("both", v="AUTO_HOLD", uav={"pilot_ready": False}))
    d = step(v, "DECLARE")
    assert d.state == "next" and "waiting for the crew" in d.warn and "UAV pilot ready" in d.warn
    v = ConsoleModel().apply(state("both", v="AUTO_HOLD", uav={"report_on": False, "heartbeats_on": False}))
    assert "UAV heartbeat off (▶ HEARTBEAT UAV)" in step(v, "DECLARE").warn
    assert "report on uav" in words(step(v, "crew"))                                         # the fix is on the crew step, sticky
    st = state("usv", v="AUTO_HOLD")
    st["ocs"]["usv_flow"] = {"heartbeat": False, "ready": True}
    v = ConsoleModel().apply(st)
    assert step(v, "ENABLE AUTO").state == "done" and "USV heartbeat off" in step(v, "DECLARE").warn


def test_the_declare_text_names_the_vehicles_tiers_and_mods():
    v = ConsoleModel().apply(state("usv", v="AUTO_HOLD"))
    d = step(v, "DECLARE").detail
    assert "declares USV1 ·" in d and "T1 CORE" in d and "mission m1t1" in d and "mods: start=advance+360" in d
    v = ConsoleModel().apply(state("both", v="AUTO_HOLD"))
    d = step(v, "DECLARE").detail
    assert "declares USV1 + UAV1" in d and "T1 ADVANCED" in d and "mods:" in d
    v = ConsoleModel().apply(state("uav", v="AUTO_HOLD"))
    assert "mods:" not in step(v, "DECLARE").detail                                          # the boat holds: no task, no modifiers


# ---- 2 · the mission modifiers subpanel ---------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("preset,kw,shown,frozen", [
    ("usv", dict(), True, False), ("usv", dict(armed=False), True, False), ("usv", dict(v="AUTO_HOLD"), True, True),
    ("usv", dict(r="R4", v="AUTO_HOLD"), True, True), ("usv", dict(r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT"), True, True),
    ("usv", dict(r="R6"), True, False), ("both", dict(), True, False), ("both", dict(v="AUTO_HOLD"), True, True),
    ("uav", dict(), False, None), ("usv", dict(mission={"code": "scout", "task": "SCOUT"}), False, None),
    ("usv", dict(mission={"code": ""}), False, None)])
def test_the_modifiers_panel_shown_and_frozen(preset, kw, shown, frozen):
    v = ConsoleModel().apply(state(preset, **kw))
    assert (v.mods is not None) is shown
    if shown:
        assert v.mods["frozen"] is frozen and all(b.ok is (not frozen) for b in v.mods["buttons"])
        assert [b.key for b in v.mods["buttons"]] == ["mod-start", "mod-look", "mod-lucky", "mod-any", "mod-squeeze", "mod-budget", "mod-exit"]   # v0.1.93.1
        assert (v.mods["why"] == M.MOD_FROZEN) is frozen and v.mods["fixed"] == "orbit=bow 2 m · follow=bow"


TOGGLE = [  # key, the armed mission's mods / lucky, what the press sets
    ("mod-start", {}, {"start_mode": "straight"}), ("mod-start", {"mods": {"start_mode": "straight"}}, {"start_mode": "advance360"}),
    ("mod-look", {}, {"look_lock": False}), ("mod-look", {"mods": {"look_lock": False}}, {"look_lock": True}),
    ("mod-budget", {}, {"scout_entry_s": 300}), ("mod-budget", {"mods": {"scout_entry_s": 300}}, {"scout_entry_s": 420}),
    ("mod-exit", {}, {"circle_exit_tangent": True}), ("mod-exit", {"mods": {"circle_exit_tangent": True}}, {"circle_exit_tangent": False}),
    ("mod-lucky", {}, {"lucky": True}), ("mod-lucky", {"lucky": True}, {"lucky": False}),
]


@pytest.mark.parametrize("key,mi,want", TOGGLE)
def test_each_modifier_button_sends_one_absolute_line_the_ocs_accepts(key, mi, want):
    """The press sends the whole `mission` line (absolute values, never a toggle): a double click sends the same line twice = harmless;
    auto / lucky / t4 kept; through ocs_cli + the real authority the mission's mods become exactly what the button says."""
    from .test_mission1_rehearsal import ocs_for
    from .test_ocs_units import intent
    v = ConsoleModel().apply(state("usv", mission=mi))
    b = next(b for b in v.mods["buttons"] if b.key == key)
    assert b.word.startswith("mission m1t1 --auto") and (bool(b.confirm) is (want.get("lucky") is True))
    app = ocs_for("m1t1-gates")
    action, _vid, args, _t = ocs_cli.intent_spec(ocs_cli.parse(b.word.split()))
    r = app.authority.on_intent("t", intent(action, args=args))
    assert r.result == "accepted", r.reason
    exp = M.mods_values(dict(state("usv", mission=mi)["ocs"]["mission"]))
    exp.update(want)
    got = dict(M.MOD_DEFAULTS, **app.mission["mods"], lucky=app.mission["lucky"])
    assert got == exp and app.mission["auto"] is True
    assert set(app.mission["mods"]) == set(CA.MOD_ARGS)                                      # all four written out: WYSIWYG


def test_mission_word_keeps_t4_and_sim():
    w = M.mission_word({"code": "m1t1sim", "auto": True, "t4": "TIER_CORE", "lucky": True}, look_lock=False)
    assert w == "mission m1t1sim --auto --lucky --t4 core --start-mode advance360 --look-lock off --scout-entry-s 420 --exit-tangent off --squeeze off --lucky-pairs one_known"
    assert M.mission_word({"code": "m1t1", "t4": "TIER_NONE"}).split()[2:4] == ["--t4", "off"]
    assert M.mission_word({}) == "" and M.mods_flags({}) == ""


@pytest.mark.parametrize("mi,line", [
    ({}, "mods: start=advance+360 · orbit=bow 2 m · follow=bow · squeeze off · look-lock on · lucky off · 420 s · exit-tangent off"),
    ({"lucky": True, "mods": {"start_mode": "straight", "look_lock": False, "scout_entry_s": 300, "circle_exit_tangent": True}},
     "mods: start=straight · orbit=bow 2 m · follow=bow · squeeze off · look-lock off · lucky on · 300 s · exit-tangent on")])
def test_the_one_mods_line(mi, line):
    assert M.mods_line(mi) == line


def test_re_arming_from_the_mission_picker_keeps_the_modifiers():
    src = (ROOT / "optiprimex" / "tools" / "tkconsole" / "views.py").read_text()
    assert 'mf = mods_flags(cur.get("mods")) if code.startswith("m1t") else ""' in src
    assert M.mods_flags({"look_lock": False, "scout_entry_s": 300}) == "--look-lock off --scout-entry-s 300"


# ---- 3 · the run banner (the corner pop-up + the always-on-top window) ---------------------------------------------------------------------
MICRO = {"stage": "TRANSIT", "sub": "through 2", "why": "0.1 m off the line", "next": "EXIT", "mods": "start=advance+360 · look-lock"}
BANNER = [  # preset, state kwargs, stage, why, next
    ("usv", dict(r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT", mode="GUIDED"), "USV · T1 · TRANSIT", "", ""),               # an older boat
    ("usv", dict(r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT", mode="GUIDED", micro=MICRO), "USV · TRANSIT · through 2",
     "— 0.1 m off the line", "next: EXIT"),
    ("usv", dict(r="R5", v="AUTO_TASK", tn="T1", ts="SEARCH_ENTRY", micro={"stage": "SEARCH_ENTRY", "sub": "LOOK_LOCK",
                                                                           "why": "suspect #10 at 8.4 m", "next": "CHECK_BLUE"}),
     "USV · SEARCH_ENTRY · LOOK_LOCK", "— suspect #10 at 8.4 m", "next: CHECK_BLUE"),
    ("usv", dict(r="R5", v="AUTO_SAFE_HOLD", tn="T1", ts="TRANSIT", micro=dict(MICRO, sub="squeeze")), "USV · TRANSIT · squeeze",
     "— 0.1 m off the line", "next: EXIT"),
    ("usv", dict(r="R5", v="AUTO_HOLD", micro=MICRO), "USV · AUTO_HOLD", "", ""),                                         # stale micro: ignored
    ("usv", dict(r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT", micro={"why": "x"}), "USV · T1 · TRANSIT", "", ""),         # no stage: ignored
    ("both", dict(r="R5", v="AUTO_TASK", tn="T1", ts="WAIT_UAV_MAP", gate={"step": "flying", "flying": True}),
     "UAV · FLYING · MAPPING  ·  boat waits", "", "next: the boat at TOUCHDOWN"),
    ("uav", dict(r="R5", v="AUTO_HOLD"), "UAV · TAKING OFF", "", ""),
]


@pytest.mark.parametrize("preset,kw,stage,why,nxt", BANNER)
def test_the_banner_lines(preset, kw, stage, why, nxt):
    b = ConsoleModel().apply(state(preset, **kw)).banner
    assert b["title"] == "RUN STARTED · run 7" and b["stage"] == stage and b["why"] == why and b["next"] == nxt
    st = kw.get("v")
    assert b["fcu"] == f"FCU {kw.get('mode', 'HOLD')} · ARMED · {st}"
    assert b["level"] == ("warn" if st == "AUTO_SAFE_HOLD" else "ok")
    if preset == "uav":
        assert b["mods"] == ""
    elif kw.get("micro", {}).get("mods") and st in ("AUTO_TASK", "AUTO_SAFE_HOLD"):
        assert b["mods"] == "mods: " + kw["micro"]["mods"]                                    # the boat's own line wins
    else:
        assert b["mods"].startswith("mods: start=")


def test_the_banner_fcu_line_disarmed_and_unknown_and_none_before_runstart():
    assert ConsoleModel().apply(state("usv", r="R5", v="AUTO_HOLD", armed=False)).banner["fcu"] == "FCU HOLD · disarmed · AUTO_HOLD"
    assert ConsoleModel().apply(state("usv", r="R5", v="AUTO_HOLD", armed=None)).banner["fcu"] == "FCU HOLD · armed ? · AUTO_HOLD"
    for r in ("R2", "R3", "R4"):
        assert ConsoleModel().apply(state("usv", r=r)).banner is None
    assert ConsoleModel().apply(state("usv", r="R6")).banner == {"title": "RUN ENDED", "stage": "run 7", "level": "idle"}


def test_the_boat_step_shows_the_micro_stage_and_next():
    v = ConsoleModel().apply(state("usv", r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT", micro=MICRO))
    d = step(v, "the boat").detail
    assert d.splitlines() == ["T1 · TRANSIT", "TRANSIT · through 2 — 0.1 m off the line", "next: EXIT"]
    v = ConsoleModel().apply(state("usv", r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT"))
    assert step(v, "the boat").detail == "T1 · TRANSIT"                                       # an older boat: as before


def test_the_state_carries_the_micro_like_the_uav_gate():
    src = (ROOT / "optiprimex" / "tools" / "ocs_web.py").read_text()
    assert '"micro": (m.map or {}).get("micro") if isinstance(m.map, dict) else None' in src


# ---- 4 · presses: never stuck, a double click sends once ------------------------------------------------------------------------------------
def test_pressbook_busy_and_never_stuck():
    t = [100.0]
    pb = PressBook(clock=lambda: t[0])
    pb.press("k")
    assert pb.busy("k") and pb.look("k") == "sending"
    t[0] += BUSY_S + 0.1
    assert not pb.busy("k")                                                                   # a second press goes after 30 s
    t[0] += STUCK_S
    assert pb.look("k") == ""                                                                 # the amber look never stays
    pb.press("k")
    pb.resolve("k", True)
    assert not pb.busy("k") and pb.look("k") == "ok"
    t[0] += 3.1
    assert pb.look("k") == ""
    assert not pb.busy("never-pressed")


@pytest.mark.parametrize("r,ok,line", [
    (None, False, "no answer"),
    ({"ok": False, "lines": ["no answer: timed out"]}, False, "no answer: timed out"),
    ({"ok": True, "lines": ["intent refused: START_TASK is accepted in AUTO_HOLD only"]}, False, "intent refused: START_TASK is accepted in AUTO_HOLD only"),
    ({"ok": True, "lines": ["intent accepted", "  ack DONE"]}, True, "intent accepted"),
    ({"ok": False, "why": "busy: one command at a time"}, False, "busy: one command at a time"),
    ({"ok": True, "lines": ["MISSION refused: look_lock: on | off (got x)"]}, False, "MISSION refused: look_lock: on | off (got x)"),
])
def test_every_answer_resolves_the_toast(r, ok, line):
    assert verdict(r)[:2] == (ok, line)
    tb = ToastBook(clock=lambda: 0.0)
    i = tb.add("w")
    tb.resolve(i, *verdict(r)[:2])
    assert tb.items[i].final


def test_an_unanswered_toast_turns_amber_and_still_resolves():
    t = [0.0]
    tb = ToastBook(clock=lambda: t[0])
    i = tb.add("declare")
    t[0] = 10.5
    tb.tick()
    assert tb.items[i].level == "wait" and "no answer yet" in tb.items[i].text
    tb.resolve(i, False, "no answer: timed out")
    assert tb.items[i].level == "bad"


# ---- 5 · the OCS page's twins (node) --------------------------------------------------------------------------------------------------------
def _pure_js() -> str:
    a, b = PAGE.index("// ==== v0.1.92 pure"), PAGE.index("// ==== end v0.1.92 pure")
    return PAGE[a:b]


def _node(js: str):
    p = subprocess.run([NODE, "-e", js], capture_output=True, text=True, timeout=30)
    assert p.returncode == 0, p.stderr
    return json.loads(p.stdout)


MIS = [{}, {"code": "m1t1", "auto": True}, {"code": "m1t1sim", "auto": True, "lucky": True, "t4": "TIER_CORE",
                                            "mods": {"look_lock": False, "scout_entry_s": 300}},
       {"code": "m1t2", "mods": {"start_mode": "straight", "circle_exit_tangent": True}}]


@pytest.mark.skipif(NODE is None, reason="no node")
def test_the_page_mods_line_and_mission_word_match_the_console():
    ch = [{}, {"look_lock": True}, {"scout_entry_s": 300}, {"lucky": True}, {"start_mode": "straight"}]
    js = _pure_js() + f"const MIS={json.dumps(MIS)},CH={json.dumps(ch)};" + \
        "console.log(JSON.stringify(MIS.map(m=>[modsLine(m),CH.map(c=>missionWord(m,c))])))"
    out = _node(js)
    for m, (line, ws) in zip(MIS, out):
        assert line == M.mods_line(m)
        assert ws == [M.mission_word(m, **c) for c in ch]


@pytest.mark.skipif(NODE is None, reason="no node")
def test_the_page_banner_matches_the_console():
    cases = [state(p, **kw) for p, kw, *_ in BANNER] + [state("usv", r="R6"), state("usv", r="R4")]
    js = _pure_js() + f"const C={json.dumps(cases)};console.log(JSON.stringify(C.map(runBanner)))"
    out = _node(js)
    for st, b in zip(cases, out):
        py = ConsoleModel().apply(copy.deepcopy(st)).banner
        if py is None:
            assert b is None
            continue
        keys = ("title", "stage", "level", "fcu", "why", "next", "mods") if "fcu" in py else ("title", "stage", "level")
        assert {k: b.get(k) for k in keys} == {k: py.get(k) for k in keys}


def test_the_page_ladder_takes_the_course_order_and_freezes_the_modifiers():
    i = PAGE.index("function drawLadder(){")
    body = PAGE[i:PAGE.index("function drawJudgeStrip", i)]
    assert 'const abd=o.auto_before_declare!==false;' in body
    assert 'ladderSet(3,auto?"done":((declared||(abd&&!ui))?"now":"next")' in body                # ENABLE AUTO live before DECLARE
    assert '(auto||!abd)?"now":"next")' in body                                                   # DECLARE after AUTO
    assert "drawMods();drawRunBanner()" in body and "<br>next: <b>" in body
    assert 'b.disabled=fz||!!MODS_PEND' in PAGE and "MODS_PEND.t>6000" in PAGE                    # frozen; "sending…" at most 6 s
    assert 'c.lucky===true?MOD_ASK.lucky' in PAGE and 'if(q)ask(' in PAGE                          # LUCKY ON asks first (v0.1.93.1: + ANY, squeeze)
    assert ".ladder2{display:flex;flex-direction:column}" in PAGE


@pytest.mark.skipif(NODE is None, reason="no node")
def test_the_page_script_parses():
    js = "\n;\n".join(re.findall(r"<script(?:\s[^>]*)?>(.*?)</script>", PAGE, re.S))
    p = subprocess.run([NODE, "--check", "-"], input=js, capture_output=True, text=True, timeout=30)
    assert p.returncode == 0, p.stderr


@pytest.mark.skipif(NODE is None, reason="no node")
def test_the_page_modifiers_freeze_rule_matches_the_console():
    cases = [("usv", {}), ("usv", dict(v="AUTO_HOLD")), ("usv", dict(r="R4", v="AUTO_HOLD")), ("usv", dict(r="R4", v="MANUAL", stalled=True)),
             ("usv", dict(r="R6")), ("both", dict(v="AUTO_LOITER"))]
    sts = [state(p, **kw) for p, kw in cases]
    i = PAGE.index("function modsFrozen(S){")
    fn = PAGE[i:PAGE.index("\n", PAGE.index("return String(st.state", i)) + 1]
    out = _node(fn + f"const C={json.dumps(sts)};console.log(JSON.stringify(C.map(modsFrozen)))")
    assert out == [ConsoleModel().apply(s).mods["frozen"] for s in sts]


# ---- 6 · the window (Tk; skips without a display) -------------------------------------------------------------------------------------------
tk = pytest.importorskip("tkinter")
from .test_ocs_tk_net import FakeOcs  # noqa: E402
from .test_ocs_tk_smoke import root, spin, tkroot  # noqa: E402,F401


def _fake_m1t1(v="MANUAL", r="R2"):
    f = FakeOcs("r2")
    s = f.bodies["/state.json"]
    s["ocs"]["mission"] = {"task": "T1", "code": "m1t1", "auto": True, "tier": "core", "lucky": False, "mods": {}}
    s["ocs"]["auto_before_declare"] = True
    s["ocs"]["r"] = r
    s["status"]["state"] = v
    return f


def test_the_window_packs_the_modifiers_before_enable_auto_and_freezes_them(root):
    from optiprimex.tools.tkconsole.views import App
    f = _fake_m1t1()
    app = App(root, f.url, fast_hz=5)
    try:
        assert spin(root, lambda: app.view is not None and app.view.mods is not None and app.view.link == "UP")
        root.geometry("1600x1000")
        assert spin(root, lambda: app.mods_w["fr"].winfo_manager() == "pack")
        sl = app.mods_w["fr"].master.pack_slaves()
        n_auto = next(s.n for s in app.view.steps if s.title.startswith("ENABLE AUTO"))
        assert sl.index(app.mods_w["fr"]) == sl.index(app.steps[n_auto]["fr"]) - 1           # just before ENABLE AUTO
        assert app.mods_w["line"].cget("text").startswith("mods: start=advance+360")
        assert str(app.buttons["mod-look"].cget("state")) == "normal"
        app.buttons["mod-look"].invoke()
        app.buttons["mod-look"].invoke()                                                     # a double click
        assert spin(root, lambda: any(p == "/cmd" for p, _ in f.posts))
        time.sleep(0.3)
        lines = [b["line"] for p, b in f.posts if p == "/cmd"]
        assert lines == ["mission m1t1 --auto --start-mode advance360 --look-lock off --scout-entry-s 420 --exit-tangent off --squeeze off --lucky-pairs one_known"]
        f.bodies["/state.json"]["status"]["state"] = "AUTO_HOLD"                             # ENABLE AUTO done -> frozen
        assert spin(root, lambda: str(app.buttons["mod-look"].cget("state")) == "disabled")
        assert app.mods_w["why"].cget("text") == M.MOD_FROZEN
        n = len([1 for p, _ in f.posts if p == "/cmd"])
        app.press("mod-start")                                                               # even a forced press sends nothing
        time.sleep(0.2)
        assert len([1 for p, _ in f.posts if p == "/cmd"]) == n
    finally:
        app.close()
        f.close()


def test_the_window_banner_shows_the_fcu_the_micro_stage_and_next(root):
    from optiprimex.tools.tkconsole.views import App
    f = FakeOcs("r5_task")
    f.bodies["/state.json"]["micro"] = dict(MICRO)
    app = App(root, f.url, fast_hz=5)
    try:
        assert spin(root, lambda: app.view is not None and app.view.banner and app.view.banner.get("why"))
        assert spin(root, lambda: app._ban_corner["why"].cget("text") == "— 0.1 m off the line")
        w = app._ban_corner
        assert w["fcu"].cget("text").startswith("FCU ") and " · ARMED · AUTO_TASK" in w["fcu"].cget("text")
        assert w["stage"].cget("text") == "→ USV · TRANSIT · through 2" and w["next"].cget("text") == "next: EXIT"
        assert w["fcu"].winfo_manager() == "pack" and w["next"].winfo_manager() == "pack"
        order = w["fr"].pack_slaves()
        assert order.index(w["fcu"]) < order.index(w["stage"]) < order.index(w["why"]) < order.index(w["next"]) < order.index(w["mods"])
        app._ban_hide("corner")
        f.bodies["/state.json"]["micro"]["why"] = "0.2 m off the line"                       # the reason changes: repainted, not re-shown
        assert spin(root, lambda: w["why"].cget("text") == "— 0.2 m off the line")
        assert w["fr"].winfo_manager() == ""
        f.bodies["/state.json"]["micro"]["sub"] = "passed 2"                                 # a new micro-stage: shown again
        assert spin(root, lambda: w["fr"].winfo_manager() == "place")
        f.bodies["/state.json"].pop("micro")                                                 # an older boat: the lines go, nothing breaks
        assert spin(root, lambda: w["why"].winfo_manager() == "" and w["stage"].cget("text").startswith("→ USV · SCOUT"))
    finally:
        app.close()
        f.close()


def test_end_run_asks_first_on_the_console_and_the_page():
    v = ConsoleModel().apply(state("usv", r="R5", v="AUTO_TASK", tn="T1", ts="TRANSIT"))
    assert step(v, "END RUN").buttons[0].confirm == M.END_CONFIRM and "HOLD" in M.END_CONFIRM
    assert 'if(line==="end_run"&&!rsAct.asked){ask(' in PAGE


def test_lucky_on_asks_with_the_action_on_the_yes_button(root):
    from optiprimex.tools.tkconsole.views import App
    f = _fake_m1t1()
    app = App(root, f.url, fast_hz=5)
    try:
        assert spin(root, lambda: app.view is not None and app.view.mods is not None and "mod-lucky" in app.btn_model)
        app.buttons["mod-lucky"].invoke()
        assert len(app.dialogs) == 1 and not [p for p in f.posts if p[0] == "/cmd"]
        assert app.dialogs[0].yes.cget("text") == "YES, LUCKY ON"
        app.dialogs[0].yes.invoke()
        assert spin(root, lambda: any(p == "/cmd" and "--lucky" in b["line"] for p, b in f.posts))
    finally:
        app.close()
        f.close()
