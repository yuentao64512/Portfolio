"""v0.1.53 (Dmitrii's round after v0.1.52) - the OCS page's part: START / STOP REPORTING (heartbeats before a declaration, kept on
through end / reset), RUN SEQUENCE grouped (one action bar, the blockers as lines with quick links, a jump flashes its target), the
FCU fence row, the rc-test console link = rc-tui in the launcher's CONSOLE. The fence loop: test_v0153_wf.py; the courses:
test_v0153_wc.py."""
from __future__ import annotations

from optiprimex.companion.config import Config
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.tools import launcher as L
from optiprimex.tools import ocs_cli

from .conftest import ROOT

TOOLS = ROOT / "optiprimex" / "tools"


def page():
    return (TOOLS / "ocs_web.html").read_text()


def test_reporting_switch():
    assert ocs_cli.intent_spec(ocs_cli.parse(["report", "on"]))[:3] == ("REPORT", None, {"on": True})
    assert ocs_cli.intent_spec(ocs_cli.parse(["report", "off"]))[:3] == ("REPORT", None, {"on": False})
    from optiprimex.ocs.cmd_authority import GUARDS
    assert GUARDS["REPORT"][2] is None                                         # nothing goes to the vehicle
    app = OcsApp(Config())
    assert app.heartbeats.enabled is False                                     # R2: no declaration, no heartbeats (as before)
    assert app.on_reporting(True) is None and app.heartbeats.enabled is True
    app.reset_run()
    assert app.heartbeats.enabled is True                                      # the switch keeps them through a reset
    app.on_reporting(False)
    assert app.heartbeats.enabled is False
    import optiprimex.tools.bench_tui as T
    assert "report" in T.INTENT_WORDS                                          # the page's /cmd goes through op-tui's table
    p = page()
    assert 'id="repbtn"' in p and 'ask("<b>STOP REPORTING?</b>' in p and '()=>send("report off")):send("report on")' in p   # v0.1.59: asks first


def test_run_sequence_grouped_with_quick_links():
    p = page()
    assert 'id="rs-acts"' in p and 'id="rs-fix"' in p and "function groupActs()" in p and "function rsFix()" in p
    assert "function jumpTo(tab,id,fill)" in p and ".jumped{outline:3px solid var(--actionLine)" in p
    assert "groupActs();rsFix();" in p and ".ladder2 .ct{display:none}" in p
    i = p.index("function rsFix()")
    body = p[i:p.index("\n}", i)]
    for need in ("usv_ui holds the FCU", "RC6 holds the boat", "the FCU is in MANUAL", 'jumpTo("MISSION","qsel")', "mission parameters",
                 'jumpTo("OVERVIEW","cli","confirm_safe")'):
        assert need in body, need


def test_the_fence_row_and_the_rc_test_console_link():
    p = page()
    assert 'sysRow("USV · FCU fence"' in p and "DIFFERS" in p and "BREACH" in p and "const fbr=fe=>" in p
    links = {x["name"]: x for x in L.page_links({"judge": "rctest"})}
    assert links["rc-test console"]["url"].endswith("/?con=mac:judge:tui") and links["judge page"]["url"] != links["rc-test console"]["url"]
    h = (TOOLS / "launcher.html").read_text()
    assert 'new URLSearchParams(location.search).get("con")' in h


def test_the_course_banner_on_the_map():
    p = page()
    assert 'id="crsban"' in p and "function crsPoll()" in p and '"/course.json"' in p
    assert 'post("/sim",{path:"/world",body:{course:r.match||"received"}})' in p and 'post("/sim",{path:"/world",body:{ignore:true}})' in p
    from optiprimex.tools.ocs_web import sim_post
    assert not str(sim_post("http://127.0.0.1:1", "/world", {"ignore": True}).get("why", "")).startswith("sim: /teleport")   # /world passes
    assert str(sim_post("http://127.0.0.1:1", "/nope", {}).get("why", "")).startswith("sim: /teleport")                   # the whitelist holds
