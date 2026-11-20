"""ROADMAP WP9c (v0.1.41): the RunDeclaration checks - one pure module (ocs/declaration_checks), used by the command authority
(refuse or warn by `declaration.policy`), the OCS state (the page's RUN CHECKLIST, MISSION CONTROL), playbook / ocs_cli `next`
and op-tui's `next:`. Handbook 3.4.10 / 3.4.16 (tiers), 3.3.1 + 3.3.5 (Task 4 in every run), 4.2 (points only in a judged run)."""
from __future__ import annotations

import json
import time
from types import SimpleNamespace

import pytest
import yaml

from optiprimex.companion.config import Config
from optiprimex.companion.mqtt import Envelope
from optiprimex.ocs import declaration_checks as DC
from optiprimex.ocs.judge_client import NullJudgeClient
from optiprimex.ocs.ocs_app import OcsApp, mission_code
from optiprimex.tools import bench_tui, ocs_cli, ocs_web, playbook

from .conftest import PY, Probe, Proc, ROOT, make_ocs_config
from .test_ocs_units import COURSE, FakeBus, intent

PROF = ROOT / "profiles"
GOOD = {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_CORE"}


def codes(rows):
    return [x["code"] for x in rows]


# ---- the pure module ----------------------------------------------------------------------------------------------------
def test_mission_needs_reads_the_existing_code_table():
    n = DC.mission_needs
    assert n("m1t1") == {"code": "m1t1", "kind": "judged", "task": "T1", "tier": "TIER_CORE"}
    assert n("m1t2")["tier"] == "TIER_ADVANCED" and n("T1", "advanced")["code"] == "m1t2"
    assert n("m4t0")["task"] == "T4" and n("T4")["code"] == "m4t0" and n("m3t1sim")["task"] == "T3"
    for c in ("m0t0", "m0t2sim", "TEST_PATH", "BOX_INSPECT", "TEST_ORBIT"):
        assert n(c)["kind"] == "test", c
    assert n("goto")["kind"] == n("GOTO")["kind"] == "goto" and n(None) is None
    assert mission_code("T1", {"tier": "advanced"}) == "m1t2" and mission_code("TEST_PATH", {"sim": True}) == "m0t2sim"  # the name kept in ocs_app
    for c in playbook.MISSION_ORDER:                                                  # the OCS copy agrees with the catalogue
        m = playbook.MISSIONS[c]
        k = n(c)
        if c == "goto":
            assert k["kind"] == "goto"
        elif m.status == "test" or c.startswith("m0t"):
            assert k["kind"] == "test", c
        elif c == "scout":                                   # v0.1.79: SCOUT declares as Task 1 (core; advanced with the drone)
            assert (k["kind"], k["task"], k["tier"]) == ("judged", "T1", "TIER_CORE"), k
        else:
            assert k["task"] == m.task and k["tier"] == "TIER_" + m.tier.upper(), c


def test_each_declaration_check():
    assert DC.check_tiers(GOOD) == []
    assert codes(DC.check_tiers({"T1": "TIER_CORE", "T3": "TIER_CORE", "T4": "TIER_CORE"})) == ["tier_missing"]        # T2 absent
    assert codes(DC.check_tiers(dict(GOOD, T3="TIER_UNKNOWN"))) == ["tier_unknown"]
    assert codes(DC.check_tiers(dict(GOOD, T1="CORE"))) == ["tier_invalid"]
    rows = DC.check_tiers(dict(GOOD, T4="TIER_NONE"))                                  # the handbook's every-run task
    assert codes(rows) == ["mandatory_none"] and rows[0]["severity"] == "soft" and "are you sure" in rows[0]["text"]   # v0.1.44: T4 a modifier
    rows = DC.check_tiers(dict(GOOD, T4="TIER_NONE"), mandatory_hard=True)             # declaration.t4_required: true (RoboNation confirms)
    assert codes(rows) == ["mandatory_none"] and rows[0]["severity"] == "hard" and "3.3.1" in rows[0]["text"]
    assert DC.check_tiers(dict(GOOD, T4="TIER_NONE"), mandatory=()) == []              # a profile may name its own list
    for r in DC.check_tiers({"T4": "TIER_NONE", "T3": "TIER_UNKNOWN"}):
        assert r["scope"] == "declaration" and "then declare" in r["text"]              # every text: what to do next


def test_each_mission_check():
    cm = DC.check_mission
    assert cm(GOOD, "m1t1") == [] and cm(GOOD, "m4t0") == [] and cm(GOOD, "m3t1") == []
    r = cm(GOOD, "m1t2")
    assert codes(r) == ["tier_mismatch"] and "TIER_ADVANCED" in r[0]["text"] and "`mission m1t1`" in r[0]["text"]
    assert codes(cm(dict(GOOD, T1="TIER_ADVANCED"), "m1t1")) == ["tier_mismatch"]
    assert cm(dict(GOOD, T1="TIER_ADVANCED"), "T1", "advanced") == []
    r = cm(dict(GOOD, T3="TIER_NONE"), "m3t1")
    assert codes(r) == ["task_not_declared"] and "T3: TIER_CORE" in r[0]["text"] and "`mission m4t0`" in r[0]["text"]
    assert cm(GOOD, "m0t2", judged=False) == [] and cm(GOOD, "goto", judged=False) == []
    r = cm(GOOD, "m0t2", judged=True)
    assert codes(r) == ["test_in_run"] and "TEST MODE" in r[0]["text"] and "`mission m4t0`" in r[0]["text"]
    assert codes(cm(GOOD, "goto", judged=True)) == ["goto_in_run"]
    assert all(x["scope"] == "mission" and x["severity"] == "hard" for x in cm(GOOD, "m0t0", judged=True) + cm(GOOD, "m1t2"))


def test_judged_run_and_the_order_of_the_steps():
    j = DC.is_judged
    assert not j("R2", "mission", test_mode=True) and not j("R6", "start_task", test_mode=True)
    assert j("R3", "mission", test_mode=True) and j("R5", "start_task", test_mode=True)
    assert j("R2", "declare", test_mode=True) and j("R2", "mission", test_mode=True, auto=True)   # armed for the next run
    assert j("R2", "mission", test_mode=False)                                         # the course: every task runs in a run
    # the same rows at any step: declaration rows always, the mission's rows for that mission
    for step in ("declare", "mission", "start_task", "state"):
        rows = DC.evaluate(dict(GOOD, T4="TIER_NONE"), "m1t2", r="R5", step=step, test_mode=True)
        assert codes(rows) == ["mandatory_none", "tier_mismatch"], step


def test_policy_and_blocking():
    assert DC.policy_of({}) == "warn" and DC.policy_of({"policy": "REFUSE"}) == "refuse" and DC.policy_of({"policy": "x"}) == "warn"
    rows = DC.evaluate(dict(GOOD, T4="TIER_NONE"), "m1t2", r="R2", step="mission", mandatory_hard=True)
    assert DC.blocking(rows, "warn") == []
    assert codes(DC.blocking(rows, "refuse")) == ["mandatory_none", "tier_mismatch"]
    assert codes(DC.blocking(rows, "refuse", "mission")) == ["tier_mismatch"]


def test_profiles_course_refuses_pool_and_bench_warn():
    for name, want in (("bench/ocs_bench.yaml", "warn"), ("pool/ocs_pool.yaml", "warn"), ("course/ocs_course.yaml", "refuse")):
        d = yaml.safe_load((PROF / name).read_text())["ocs"]["declaration"]
        assert DC.policy_of(d) == want, name
        assert DC.check_tiers(d["tiers"]) == [], name                                  # every shipped profile declares cleanly
    assert "lab" not in [p.name for p in PROF.iterdir()] and "ocs_pool.yaml" in (ROOT / "scripts" / "run_ocs.sh").read_text()  # lab = the pool's OCS


# ---- the authority inside the OCS app (refuse / warn) -------------------------------------------------------------------
def make_app(profile: str = "course/ocs_course.yaml", r: str = "R2", **decl):
    cfg = Config()
    cfg.extra = yaml.safe_load((PROF / profile).read_text())
    cfg.extra["ocs"]["declaration"].update(decl)
    bus = FakeBus()
    results = []
    app = OcsApp(cfg, bus=bus, judge=NullJudgeClient())
    app.authority.publish_result = results.append
    app.lifecycle.on_link(True)
    app.lifecycle.on_course(COURSE)
    app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL"}})
    app.on_reporting(True, "usv")                                         # v0.1.82: ▶ HEARTBEAT USV + PILOT READY (the course profile)
    app.on_usv_ready(True)
    if r in ("R3", "R5"):
        assert app.authority.on_intent("t", intent("DECLARE")).result == "accepted"
        app.on_ack("t", {"vehicle_id": "USV1", "body": {"ack_of": list(app._prepare_msg_ids)[0], "result": "DONE"}})
    if r == "R5":
        app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO"}})
        app.on_rc_command({"kind": "run_start", "declaration_seq": app.lifecycle.declaration_seq, "run_id": 1001, "command_seq": 1})
    assert app.lifecycle.body()["r"] == r
    return app, bus, results


def test_course_policy_refuses_a_bad_declaration_and_says_what_to_do():
    app, bus, _ = make_app(tiers={"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_NONE"}, t4_required=True)
    assert app.authority.declaration_policy == "refuse"
    r = app.authority.on_intent("t", intent("DECLARE"))
    assert r.result == "refused" and "policy refuse" in r.reason and "T4" in r.reason and "then declare" in r.reason
    assert not any(p["body"].get("action") == "PREPARE" for p in bus.published)        # nothing went out
    assert app.lifecycle.declaration_seq == 0
    st = app.state_body()["declaration_checks"]
    assert st["policy"] == "refuse" and st["blocking"] == ["mandatory_none"] and st["source"] == "profile"


def test_course_policy_refuses_mission_and_start_task_mismatches():
    app, bus, _ = make_app()
    r = app.authority.on_intent("t", intent("MISSION", args={"task": "T1", "tier": "advanced"}))
    assert r.result == "accepted" and app.effective_tiers()["T1"] == "TIER_ADVANCED"   # v0.1.66 (D22b): before a declaration the T1 tier
    app.authority.on_intent("t", intent("MISSION", args={"task": "off"}))              # follows the mission (was: refused, a mismatch)
    r = app.authority.on_intent("t", intent("MISSION", args={"task": "TEST_PATH"}))
    assert r.result == "refused" and "test mission" in r.reason                         # the course: no TEST MODE, every task is judged
    assert app.authority.on_intent("t", intent("MISSION", args={"task": "T1"})).result == "accepted"
    assert app.authority.on_intent("t", intent("MISSION", args={"task": "off"})).result == "accepted"   # off is never refused
    app, bus, _ = make_app(r="R5")
    n = len(bus.published)
    r = app.authority.on_intent("t", intent("START_TASK", "T1", {"tier": "advanced"}))
    assert r.result == "refused" and "START_TASK m1t2 refused" in r.reason and len(bus.published) == n
    r = app.authority.on_intent("t", intent("START_TASK", "TEST_SWAY"))
    assert r.result == "refused" and "m0t3" in r.reason and "TEST MODE" in r.reason
    r = app.authority.on_intent("t", intent("START_TASK", "T4"))
    assert r.result == "accepted" and r.reason == "" and bus.published[-1]["body"]["task"] == "T4"   # a clean start: unchanged reason


def test_course_policy_refuses_a_declare_with_a_test_mission_armed():
    app, bus, _ = make_app()
    app.set_mission("TEST_PATH", True, {})                                             # set without the intent (an older preselect)
    r = app.authority.on_intent("t", intent("DECLARE"))
    assert r.result == "refused" and "m0t2" in r.reason and "judged run" in r.reason


def test_the_auto_start_obeys_the_same_checks():
    app, bus, _ = make_app(r="R3")
    app.mission.update(task="TEST_PATH", auto=True, code="m0t2")                       # armed some other way
    app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO"}})
    app.on_rc_command({"kind": "run_start", "declaration_seq": app.lifecycle.declaration_seq, "run_id": 1002, "command_seq": 2})
    assert not any(p["body"].get("action") == "START_TASK" for p in bus.published)
    assert app.alerts[-1]["kind"] == "mission_refused" and "TEST MODE" in app.alerts[-1]["action"]


def test_pool_and_bench_policy_warn_and_never_refuse():
    for prof in ("pool/ocs_pool.yaml", "bench/ocs_bench.yaml"):
        app, bus, _ = make_app(prof, tiers={"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_NONE"})
        assert app.authority.declaration_policy == "warn" and app.authority.test_mode
        r = app.authority.on_intent("t", intent("MISSION", args={"task": "T1", "tier": "advanced"}))
        assert r.result == "accepted" and "m1t2 needs T1" not in r.reason, prof        # v0.1.66 (D22b): the next declaration follows m1t2
        r = app.authority.on_intent("t", intent("DECLARE"))
        assert r.result == "refused" and "m1t2 needs declare uav on" in r.reason, r.reason    # v0.1.64 (D7): not a tier check - the UAV rule
        st = app.state_body()["declaration_checks"]
        assert st["policy"] == "warn" and st["blocking"] == [] and codes(st["rows"]) == ["mandatory_none"]   # v0.1.66 (D22b): T1 follows m1t2
        app.authority.on_intent("t", intent("MISSION", args={"task": "T1"}))                   # m1t1: the warn policy as before
        r = app.authority.on_intent("t", intent("DECLARE"))
        assert r.result == "accepted" and r.reason.startswith("PREPARE sent") and "check:" in r.reason and "T4" in r.reason


def test_a_profile_without_policy_keeps_the_old_behaviour():
    cfg = Config()
    cfg.extra = yaml.safe_load((PROF / "bench" / "ocs_bench.yaml").read_text())
    del cfg.extra["ocs"]["declaration"]["policy"]
    app = OcsApp(cfg, bus=FakeBus(), judge=NullJudgeClient())
    assert app.authority.declaration_policy == "warn"
    cfg.extra["ocs"].pop("declaration")
    app = OcsApp(cfg, bus=FakeBus(), judge=NullJudgeClient())                          # no declaration block at all: the defaults
    assert app.authority.declaration_policy == "warn" and app.effective_tiers() == DC.DEFAULT_TIERS
    assert app.state_body()["declaration_checks"]["rows"] == []


def test_test_mode_unchanged_by_the_checks():
    """TEST MODE (bench / pool / lab): m0t*, goto, home without a run - accepted with no reason change (test_wp10d's path)."""
    app, bus, _ = make_app("pool/ocs_pool.yaml")
    r = app.authority.on_intent("t", intent("START_TASK", "TEST_PATH", {"names": ["A", "B"]}))
    assert r.result == "accepted" and r.reason == ""
    assert app.authority.on_intent("t", intent("GOTO", args={"x_n": 1, "y_e": 2})).reason == ""
    assert app.authority.on_intent("t", intent("HOME")).result == "accepted"
    r = app.authority.on_intent("t", intent("MISSION", args={"task": "TEST_PATH"}))
    assert r.result == "accepted" and "check" not in r.reason
    # in a judged run the same test mission is flagged - a warning only on the pool
    app, bus, _ = make_app("pool/ocs_pool.yaml", r="R5")
    r = app.authority.on_intent("t", intent("START_TASK", "TEST_PATH", {"names": ["A", "B"]}))
    assert r.result == "accepted" and "check: m0t2 (a test mission) inside a judged run" in r.reason
    r = app.authority.on_intent("t", intent("GOTO", args={"x_n": 1, "y_e": 2}))
    assert r.result == "accepted" and "goto (the bench waypoint) inside a judged run" in r.reason


def test_the_declared_tiers_win_once_declared():
    app, bus, _ = make_app("pool/ocs_pool.yaml", r="R3")
    app.declaration_cfg["tiers"] = dict(GOOD, T1="TIER_ADVANCED")                     # the profile changed after the declaration
    st = app.state_body()["declaration_checks"]
    assert st["source"] == "declared" and st["tiers"]["T1"] == "TIER_CORE"


# ---- the state rows: the page's RUN CHECKLIST, MISSION CONTROL ----------------------------------------------------------
def ocs_state(policy="refuse", tiers=None, mission=None, r="R2", test_mode=False, v="MANUAL", run_id=None, decl=0, hard=True):
    rows = DC.evaluate(tiers or GOOD, (mission or {}).get("code"), r=r, step="state", test_mode=test_mode, auto=bool((mission or {}).get("auto")),
                       mandatory_hard=hard)       # v0.1.44: these tests take t4_required (hard); the default is soft
    return {"r": r, "r_state": "x", "run_id": run_id, "declaration_seq": decl, "bench_mode": policy == "warn", "test_mode": test_mode,
            "stalled": False, "judge": "fake", "judge_connected": True, "course_id": "C1", "mission": mission or {},
            "vehicles": {"USV1": {"state": v, "robot_state": "STATE_AUTO" if v.startswith("AUTO") else "STATE_MANUAL", "current_task": "TASK_NONE"}},
            "declaration_checks": DC.state_block(tiers or GOOD, r in DC.JUDGED_R, policy, DC.MANDATORY, rows, hard)}


def snap(ocs):
    return {"hub_up": True, "status_age": 0.4, "ui_active": False, "perception_age": 0.5, "ocs": ocs,
            "status": {"state": ocs["vehicles"]["USV1"]["state"], "armed": True, "fcu_link": "real", "mode": "HOLD"}}


def test_the_run_checklist_rows_and_clear_for_run():
    c = ocs_web.clearance(snap(ocs_state()))
    row = next(x for x in c["checks"] if x["id"] == "declaration")
    assert row["ok"] and row["hard"] and "T4:core" in row["detail"] and c["clear"]
    c = ocs_web.clearance(snap(ocs_state(tiers=dict(GOOD, T4="TIER_NONE"), mission={"code": "m1t2"})))
    ids = [x["id"] for x in c["checks"] if x["id"].startswith("decl")]
    assert ids == ["decl:mandatory_none", "decl:tier_mismatch"] and not c["clear"]
    assert c["blocked"] == ["declaration: mandatory none", "declaration: tier mismatch"]
    assert "then declare" in c["checks"][-2]["fix"]
    c = ocs_web.clearance(snap(ocs_state("warn", tiers=dict(GOOD, T4="TIER_NONE"))))      # the pool: shown, not blocking
    row = next(x for x in c["checks"] if x["id"] == "decl:mandatory_none")
    assert not row["ok"] and not row["hard"] and c["clear"]
    o = ocs_state()
    del o["declaration_checks"]
    assert not any(x["id"].startswith("decl") for x in ocs_web.clearance(snap(o))["checks"])     # an older ocs_app: no line


def test_mission_control_sees_a_blocking_mission():
    md = ocs_web.mission_declaration(ocs_state(r="R5", v="AUTO_HOLD", run_id=1001, decl=1))
    assert md["m1t2"]["block"] and md["m0t2"]["block"] and md["goto"]["block"] and "m4t0" not in md and "m1t1" not in md
    md = ocs_web.mission_declaration(ocs_state("warn", r="R2", test_mode=True))
    assert "m0t2" not in md and not md["m1t2"]["block"]                                # TEST MODE, no run: m0t2 is fine; a warning for m1t2
    assert ocs_web.mission_declaration({"r": "R2"}) == {}


def test_the_page_has_the_rows():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ("S.mission_declaration", "declaration: ${m.code} would be refused", "the RunDeclaration checks (handbook 3.4.10 / 3.4.16)",   # v0.1.59: the rows in step 2 (ck-decl)
                   'id="ck"', 'id="clearbtn"'):
        assert needle in page, needle


# ---- playbook / ocs_cli next / op-tui next: ------------------------------------------------------------------------------
def test_playbook_check_and_suggest():
    o = ocs_state(r="R5", v="AUTO_HOLD", run_id=1001, decl=1)
    assert "m1t2 needs T1 TIER_ADVANCED" in playbook.check("START_TASK", o, task="m1t2").why   # v0.1.66: built - the tier verdict
    v = playbook.check("START_TASK", ocs_state(tiers=dict(GOOD, T1="TIER_ADVANCED"), r="R5", v="AUTO_HOLD", run_id=1001, decl=1), task="m1t1")
    assert not v.ok and "m1t1 needs T1 TIER_CORE" in v.why and "policy refuse" in v.hint
    assert not playbook.check("GOTO", dict(o, bench_mode=True)).ok
    assert playbook.check("START_TASK", o, task="m4t0").ok
    assert playbook.check("START_TASK", o, task="m0t2").why.startswith("m0t2 (a test mission)")
    bad = ocs_state(tiers=dict(GOOD, T4="TIER_NONE"))
    v = playbook.check("DECLARE", bad)
    assert not v.ok and "T4" in v.why
    assert playbook.check("DECLARE", ocs_state()).ok
    assert playbook.suggest(bad).startswith("declaration check (refused until fixed): T4")
    s = playbook.suggest(ocs_state("warn", tiers=dict(GOOD, T4="TIER_NONE")))
    assert s.startswith("declare") and "· check: T4" in s
    assert playbook.suggest(ocs_state()) == "declare"                                   # clean: as before
    assert playbook.check("DECLARE", ocs_state("warn", tiers=dict(GOOD, T4="TIER_NONE"))).ok   # warn never predicts a refusal


def test_ocs_cli_next_lists_the_check():
    bad = ocs_state(tiers=dict(GOOD, T4="TIER_NONE"))
    con = SimpleNamespace(wait_state=lambda s: bad, vehicle_id="USV1", status=None, state=bad)
    code, lines = ocs_cli.execute(con, SimpleNamespace(cmd="next", wait=0.1))
    text = "\n".join(lines)
    assert code == 0 and "-- declare" in text and "suggested next: declaration check (refused until fixed): T4" in text


def test_op_tui_next_line_200x60():
    m = bench_tui.Model("USV1", "usv")
    view = bench_tui.View(m, "USV1")
    m.on_hub("ocs/state", json.dumps({"body": ocs_state("warn", tiers=dict(GOOD, T4="TIER_NONE"))}).encode())
    m.hub_up = True
    c = bench_tui.Canvas(200, 60)
    view.draw(c, 0, "", "")
    nxt = [ln for ln in c.lines() if ln.startswith("next: ")]
    assert nxt and nxt[0].startswith("next: declare") and "check: T4" in nxt[0]


# ---- live: ocs_app as a process, the course policy over a real broker -----------------------------------------------------
def test_live_ocs_app_refuses_by_policy(brokers, tmp_path):
    cfg = make_ocs_config(tmp_path, brokers,
                          declaration={"tiers": dict(GOOD, T4="TIER_NONE"), "policy": "refuse", "t4_required": True})
    probe = Probe(brokers.hub_port, ["ocs/#"]).start()
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(cfg)], tmp_path / "ocs.log")
    con = ocs_cli.OcsConsole(port=brokers.hub_port)
    con.start()
    try:
        assert probe.wait(lambda m: m["topic"] == "ocs/state" and "declaration_checks" in m["env"]["body"], 15, include_past=True), ocs.log()
        st = probe.last("ocs/state")["env"]["body"]["declaration_checks"]
        assert st["policy"] == "refuse" and st["blocking"] == ["mandatory_none"]
        assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] in ("R1", "R2"), 15, include_past=True), ocs.log()
        con.intent("INJECT", args={"what": "course"})                                  # bench_mode: the course by hand -> R2
        assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 10, include_past=True), ocs.log()
        r = con.intent("DECLARE", wait_s=3)
        assert r["result"]["result"] == "refused" and "policy refuse" in r["result"]["reason"] and "T4" in r["result"]["reason"], r
    finally:
        con.stop()
        probe.stop()
        ocs.kill()
