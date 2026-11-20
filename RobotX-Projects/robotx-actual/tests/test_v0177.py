"""v0.1.77 (WP33 = N5's JUDGE, docs/OVERNIGHT_2026-11-09_GUI.md, board 06): REPORTING (the switch) + ROBOCOMMAND on top, the run timeline as
a strip, the judges' table (one row per vehicle) beside TASKS, the feed full width; the OCS's judge view / the fake judge / the traffic in a
fold. Every older id kept (the tests of v0.1.29 / WP8b / WP10d pin them)."""
from __future__ import annotations

import os
import re
import subprocess
import threading

import pytest

from .conftest import ROOT, free_port

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def test_the_markup_keeps_the_ids():
    p = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ("function n9Init()", "function drawN9()", 'id="rc-tl"', 'id="rc-veh"', 'id="rc-tasks"', 'id="rc-feed"', 'id="rc-kv"',
                   'id="rc-ans"', 'id="j-view"', 'id="j-stats"', 'id="j-log"', '"jd-rep"', "THE JUDGES' TABLE",
                   "const jf=(S.judge||{}).feed||[]"):                                  # v0.1.77: the LOG's judge kind = the JUDGE feed (found live)
        assert needle in p, needle


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_judge_tab_order(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{a.web_port}/?probe=1#JUDGE"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    assert not re.search(r'"N[0-9] [A-Za-z]*Error', r.stderr), r.stderr[-1500:]
    dom = r.stdout
    sec = dom[dom.index('id="t-JUDGE"'):]
    sec = sec[:sec.index("</section>")]
    order = [sec.index(f'id="{i}"') for i in ("jd-top", "jd-rep", "repbtn", "jd-rc", "rc-ans", "jd-tl", "rc-tl", "jd-tbl", "rc-jt", "rc-tasks", "jd-feed", "rc-feed",
                                             "jd-more", "j-view", "j-stats", "j-log")]
    assert order == sorted(order), order                               # reporting + RoboCommand · the strip · the table + tasks · the feed · the fold
    jt = sec[sec.index('id="rc-jt"'):sec.index('id="rc-tasks"')]
    assert re.search(r"<td><b>USV1</b></td>", jt) and "at the judge" in jt         # one row per vehicle, the boat always
    assert '<div class="card" id="jd-feed">' in sec                     # the feed: its own card, a direct child of the tab (full width)
    assert '"missing":0' in dom
