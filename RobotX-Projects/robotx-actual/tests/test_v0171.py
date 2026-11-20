"""v0.1.71 (WP27 = docs/OVERNIGHT_2026-11-09_GUI.md N3): the picker in groups, T4 as a tier, m4t1 -> m4t0, MISSION METRICS on the
right rail, the map's TOOLS fold."""
from __future__ import annotations

import pytest

from optiprimex.ocs.ocs_app import t4_tier
from optiprimex.tools import ocs_cli, playbook

from .conftest import ROOT
from .test_v0164 import _declare_done, make_app
from .test_v0170 import _sent_fields


def test_m4t1_is_m4t0_and_the_old_word_keeps_its_verdict():
    assert playbook.resolve("m4t0") == ("m4t0", "T4", "core", False)
    assert playbook.resolve("m4t1") == playbook.resolve("m4t0") == playbook.resolve("T4")[:4]       # the alias, the wire name
    assert playbook.resolve("m4t1sim") == ("m4t0", "T4", "core", True)
    assert "m4t1" not in playbook.MISSIONS and playbook.MISSION_ORDER[0] == "m4t0"
    from optiprimex.ocs.declaration_checks import mission_code
    assert mission_code("T4") == "m4t0"
    ns = ocs_cli.parse(["mission", "m4t1", "--auto"])
    assert ocs_cli.intent_spec(ns)[2]["task"] == "T4"                              # the same intent as before
    from optiprimex.companion.state import RobotXTask
    assert RobotXTask.T4.value == "TASK_DYNAMIC_INCIDENT"                          # the wire name unchanged (the brief's TASK_4)


@pytest.mark.parametrize("word,tier", [("off", "TIER_NONE"), ("core", "TIER_CORE"), ("advanced", "TIER_ADVANCED"), ("on", "TIER_CORE"),
                                       (None, None), (True, "TIER_CORE"), (False, "TIER_NONE")])
def test_t4_words_to_tiers(word, tier):
    assert t4_tier(word) == tier


def test_t4_as_a_tier_in_the_declaration_and_m4t0_declared_as_m1t1():
    app, bus, judge, _ = make_app("course/ocs_course.yaml")
    app.set_mission("T1", False, {"t4": "advanced"})
    p = app.declaration_preview()
    assert p["fields"]["tiers"]["T4"] == "TIER_ADVANCED"                           # the T4 tick changes the preview live
    app.set_mission("T1", False, {"t4": "off"})
    assert app.declaration_preview()["fields"]["tiers"]["T4"] == "TIER_NONE"
    app.set_mission("T1", False, {})
    m1t1 = app.declaration_preview()["fields"]
    app.set_mission("T4", False, {})
    assert app.mission["code"] == "m4t0"
    m4t0 = app.declaration_preview()["fields"]
    assert m4t0 == m1t1 and m4t0["tiers"]["T1"] == "TIER_CORE" and m4t0["tiers"]["T4"] == "TIER_CORE"   # m4t0 = m1t1 + T4 core
    assert _declare_done(app, bus).result == "accepted" and len(judge.requests) == 1 and _sent_fields(judge) == m4t0


def test_ocs_cli_t4_choices():
    for w in ("off", "core", "advanced", "on"):
        assert ocs_cli.intent_spec(ocs_cli.parse(["mission", "m1t1", "--t4", w]))[2]["t4"] == w
    with pytest.raises(SystemExit):
        ocs_cli.parse(["mission", "m1t1", "--t4", "maybe"])


def test_the_page_and_the_map():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('id="cgrp"', 'data-g="event"', 'data-g="pool"', 'data-g="dry"', 'id="t4sel"', '<option value="core" selected>core</option>',
                   'const GRP_EVENT=["m1t1","m1t2","scout","m3t1"]', '"m4t0","m0t0","m0t1","m0t8","m0t9"]',     # v0.1.79: + scout, m0t8; v0.1.80: m0t9
                   'value="sess:collect">m0t7 · collect',
                   '` --t4 ${$("t4sel").value}`'):
        assert needle in page, needle
    assert page.index('id="mm-card"') > page.index("STAGES <span") and page.index('id="mm-card"') < page.index('id="sess-card"')
    mp = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    t0, t1 = mp.index('<details id="tools"'), mp.index("</details>", mp.index('<details id="tools"'))
    for idn in ("b_clear", "b_drop", "fencebar", "fcubox", "b_add", "tpbar"):
        assert f'id="{idn}"' in mp[t0:t1], idn                                     # every tool inside TOOLS ▾
    assert "body.judge #tools, body.mini #tools{display:none" in mp
