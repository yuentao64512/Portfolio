"""v0.1.75 (WP31 = N5's LOG, docs/OVERNIGHT_2026-11-09_GUI.md): the unified LOG - events · commands · alarms history · judge · system in one
list, newest first, kind chips with counts, search, a time range, at least 20 rows on the screen."""
from __future__ import annotations

import json
import os
import re
import subprocess
import threading
import time

import pytest

from .conftest import ROOT, free_port

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def test_the_markup():
    p = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('id="ulog-card"', 'id="ul-chips"', 'id="ul-q"', 'id="ul-range"', '<option value="1800" selected>last 30 min</option>',
                   'id="ul-prob"', 'id="ul-list" style="min-height:430px', "function drawULog()", 'fetch("/alarms.json"', 'id="logall"'):
        assert needle in p, needle


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_one_list_from_five_sources(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    m = web.m
    for i in range(25):
        m.add("event", f"vehicle AUTO_TASK step {i}")
    m.add("intent", "intent hold accepted")
    m.add("ack", "ack DONE HOLD")
    m.add("judge", "RunStart run 1001")
    web.runner.run("stop_simcv", {}, "stop", {})                      # a dry-run job: the system kind
    web.alarms.history.append({"first_t": time.time() - 60, "cleared_t": time.time() - 30, "level": "WARNING", "text": "RC LOST (test)", "acked": False})
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=7000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{a.web_port}/?probe=1#LOG"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    dom, err = r.stdout, r.stderr
    assert not re.search(r'"(N7|N2|N4|N5) [A-Za-z]*Error', err), err[-1500:]
    lst = dom[dom.index('id="ul-list"'):dom.index("</div></div>", dom.index('id="ul-list"'))]
    kinds = re.findall(r'<b class="dim">(\w+)</b>', lst)
    assert len(kinds) >= 20 and {"events", "commands", "judge", "system", "alarms"} <= set(kinds), set(kinds)
    for k in ("events", "commands", "alarms", "judge", "system"):
        assert re.search(rf'id="ulc-{k}">[1-9]', dom), k                            # the chips count their kind
    txt = re.sub(r"<[^>]+>", "", lst)
    assert "RC LOST (test)" in txt and "step 24" in txt and txt.index("step 24") < txt.index("step 0")     # the history, newest first
