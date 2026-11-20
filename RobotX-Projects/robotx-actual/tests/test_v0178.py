"""v0.1.78 (WP34, by hand - Dmitrii 2026-11-09): the declutter pass after the night - the OCS bar never wraps (a width budget, the theme
select in ⚙ only), RUN SEQUENCE is a stepper (the gate and the declaration constructor inside step 2, the picker in the MISSION card, the
step buttons in their step), the course banner out of the map, SYSTEM's hosts table first (NOW · CONNECTIONS · EVERYTHING folded), ALARMS
one line when quiet; the map folds its side column when it sits in the OCS page; the boat window's row 1 never wraps (the commander compact,
the list on hover), the "click an element" box only after a click, the wireframe short enough for the dials on the first screen.
Nothing removed from the DOM: every v0.1.77 id and every pinned source string stays."""
from __future__ import annotations

import os
import re
import subprocess
import threading
import time

import pytest

from .conftest import ROOT, free_port

TOOLS = ROOT / "optiprimex" / "tools"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def test_the_markup_moves_and_hides_only():
    p = (TOOLS / "ocs_web.html").read_text()
    # the night's anchors are all still there (the agent's tests pin them)
    for s in ('id="rs-acts"', 'id="rs-fix"', 'id="rs-mis"', 'id="rs-gate"', "function groupActs()", "for(let n=1;n<=6;n++)", ".ladder2 .ct{display:none}",
              'for(const id of ["simban","flavour","themebar"])', "S.flavour.text", 'id="decl-con"', 'id="x-every"', 'id="themebar"'):
        assert s in p, s
    # the pass itself
    assert "function n7Init()" in p and "function drawN7()" in p and 'try{drawN7()}catch(e)' in p
    assert "header .top{flex-wrap:nowrap;gap:5px}" in p and "#ctx #themebar{display:none}" in p                  # the bar: one row by construction
    assert 'b2.appendChild($("rs-gate"))' in p and 'b2.appendChild($("decl-con"))' in p                          # step 2 holds the gate + the constructor
    assert '$("run-mis").insertBefore($("rs-mis")' in p                                                          # the picker row in the MISSION card
    assert 'bar.style.display="none"' in p and 'c.querySelectorAll("[data-send]")' in p                          # the buttons stay in their step
    assert ".ladder2 .st2.now .ct{display:flex" in p and ".ladder2 .st2:not(.now) .ev2" in p                      # the live step expanded, the rest one line
    assert ".crsban.on{position:static" in p and "#t-MISSION .cword{font-size:22px" in p
    assert '"<summary>NOW ▾' in p and '"<summary>CONNECTIONS ▾' in p and '"<summary>EVERYTHING ▾' in p
    assert "#ov-alcard.quiet .alt,#ov-alcard.quiet .alctl{display:none!important}" in p
    assert '({"mission manager":"mm"})[B.holder]' in p and 'judge ok' in p                                         # the chips' words
    assert 'filter(p=>!/\\?/.test(p))' in p                                                                        # no "?" chips in the context line
    b = (TOOLS / "boat_web.html").read_text()
    assert "#n6r1{flex-wrap:nowrap;gap:6px}" in b and "#wf{max-height:360px;width:100%}" in b and "#wfd:not(.has){display:none}" in b
    assert "$('ctl-who').title=whoFull" in b and "$('wfd').classList.add('has')" in b
    m = (TOOLS / "map.html").read_text()
    assert 'id="b_details"' in m and "body.embed aside { display:none; }" in m and "document.body.classList.add('embed')" in m
    assert "body.embed.details aside { display:block; }" in m
    v1 = (TOOLS / "boat_web_v1.html").read_text()
    assert "n6r1" not in v1                                                                                        # the classic window untouched


def _walk(web, a, url_q="?probe=1"):
    from optiprimex.tools import ocs_web as OW
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{a.web_port}/{url_q}"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    assert not re.search(r'"(N7 |N2|N4|N5) [A-Za-z]*Error', r.stderr), r.stderr[-1500:]
    return r.stdout


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_bar_holds_one_row_under_load_and_run_is_a_stepper(tmp_path):
    """R4 with its time in the chip, a long reason, reporting on, the FCU armed: the state the v0.1.71 capture wrapped in."""
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    now = time.time()
    with web.m.lock:
        web.m.status = {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO", "current_task": "TASK_NONE", "mode": "HOLD", "armed": True, "kill": False,
                        "estop": False, "lat": 1.2810202, "lon": 103.8550097, "hdg_deg": 87.0, "sog_mps": 0.0, "health": {"faults": [], "rc_ok": True},
                        "rc6": {"holds": False, "pwm": 1000}}
        web.m.status_t = now
        web.m.ocs = {"r": "R4", "state": "READY", "judge": "fake", "judge_connected": True, "mission": {"code": "m1t1", "task": "T1", "auto": True, "t4": "TIER_CORE"},
                     "vehicles": {"USV1": {"state": "STATE_AUTO", "current_task": "TASK_NONE", "age_s": 0.4}}, "reporting": {"on": True, "hb": {"USV1": {"sent": 55, "age_s": 0.5}}},
                     "declaration_seq": 224, "waiting_s": 42, "declared_s": 60, "attempt": 1, "judge_view": {"course_age_s": 87}, "uav": {"configured": False},
                     "declaration_preview": {"tiers": {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_CORE"}, "body": {}, "text": "task1_tier: TIER_CORE"}}
        web.m.ocs_t = now
        web.m.hub_up = web.m.boat_up = web.m.bridge = True
    dom = _walk(web, a)
    probe = re.search(r'<pre id="n2probe">(.*?)</pre>', dom, flags=re.S).group(1)
    tops = {int(m.group(1)) for m in re.finditer(r"(?:flex|block) \d+ @\d+,(\d+)", probe)}
    assert max(tops) - min(tops) < 20, probe                                                                       # one row, with R4 READY · 0:42 in the chip
    assert re.search(r'id="rchip"[^>]*>R4 READY · 0:42<', dom), "the R chip carries the time"
    assert "mm · HOLD · armed" in dom and "judge ok" in dom                                                        # the budgeted chips
    lad2 = dom[dom.index('id="lad-2"'):dom.index('id="lad-3"')]
    assert 'id="rs-gate"' in lad2 and 'id="decl-con"' in lad2                                                     # the gate + the constructor live in step 2
    assert 'id="rs-mis"' in dom[dom.index('id="run-mis"'):dom.index('id="ov-alcard"')]                            # the picker row in the MISSION card
    assert re.search(r'id="rs-acts"[^>]*style="display: none;"', dom)                                             # the button bar hidden, the buttons in their step
    lad4 = dom[dom.index('id="lad-4"'):dom.index('id="lad-5"')]
    assert "now" in re.search(r'<div class="([^"]*)" id="lad-4"', dom).group(1).split()                           # step 4 is the live one
    al = dom[dom.index('id="ov-alcard"') - 60:]
    al = al[:al.index('id="al-histbox"')]
    quiet = bool(re.search(r'class="card quiet"[^>]*id="ov-alcard"', al))
    assert quiet == ("nothing to act on" in al), al[:300]                                                          # quiet (one line) exactly when nothing is active
    assert "started by hand" in dom or "judge fake" in dom                                                         # the flavour chip without its "?" parts
    assert "net ?" not in dom[dom.index('id="ctx"'):dom.index('id="attn"')]
