"""v0.1.72 (WP28 = docs/OVERNIGHT_2026-11-09_GUI.md N4): SYSTEM - one hosts table (settings ▾ / actions ▾ per host), bench & tools,
the jobs streamed to logs/ocs/system_actions/<job>.log and the drawer; every action of v0.1.71 reachable with the same word and guard."""
from __future__ import annotations

import os
import re
import subprocess
import threading

import pytest

from optiprimex.tools import system_actions as SA

from .conftest import ROOT, free_port

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
V71_ACTIONS = {"ocs": "", "rctest": "", "nx": "", "agx": "", "fcu": "fcu", "nx_reboot": "nx", "agx_reboot": "agx", "simworld": "", "cam": "",
               "rec": "", "simreset": "reset", "stopall": "stop", "resetall": "reset", "stop_bench": "stop", "stop_simcv": "stop", "stop_nx": "stop",
               "stop_agx": "stop", "stop_ocs": "stop"}


def test_every_action_with_the_same_word():
    have = {a.id: a.confirm for a in SA.ACTIONS}
    assert V71_ACTIONS.items() <= have.items(), {k: (v, have.get(k)) for k, v in V71_ACTIONS.items() if have.get(k) != v}


def test_the_guards_unchanged_fcu_reboot_refused_armed(tmp_path):
    r = SA.Runner(env={}, log_path=str(tmp_path / "sa.jsonl"), dry_run=True)
    armed = {"status": {"armed": True, "state": "MANUAL", "fcu_link": "real"}}
    assert r.run("fcu", {}, "", armed)["why"].startswith("refused:")
    assert r.run("nx_reboot", {}, "wrong", {})["why"] == "type 'nx' to confirm"
    auto = {"status": {"state": "AUTO_TASK", "armed": True}}
    assert r.run("stop_nx", {}, "stop", auto)["why"].startswith("refused:")


def test_dry_run_shows_every_command_and_the_job_log(tmp_path):
    r = SA.Runner(env={"AGX_SSH": "agxorin"}, log_path=str(tmp_path / "sa.jsonl"), dry_run=True)
    out = r.run("stop_simcv", {}, "stop", {})
    assert out["ok"]
    j = r.job_list()[-1]
    assert j["lines"][0].startswith("(dry run) ") and j["rc"] == 0 and not j["failed"]
    assert j["log"].startswith(str(tmp_path / "system_actions")) and "(dry run)" in open(j["log"]).read()


def test_a_live_job_streams_into_its_log_and_a_failure_keeps_20_lines(tmp_path, monkeypatch):
    a = SA.Action("zz", "mac", "t", "t", build=lambda o, e: ["bash", "-c", "for i in $(seq 1 30); do echo line$i; done; exit 3"])
    monkeypatch.setitem(SA.BY_ID, "zz", a)
    r = SA.Runner(env=dict(os.environ), log_path=str(tmp_path / "sa.jsonl"), dry_run=False)
    assert r.run("zz", {}, "", {})["ok"]
    for _ in range(100):
        j = r.job_list()[-1]
        if j["done"]:
            break
        threading.Event().wait(0.05)
    assert j["failed"] and j["rc"] == 3 and len(j["tail20"]) == 20 and j["tail20"][-1] == "line30"
    log = open(j["log"]).read()
    assert "line1\n" in log and "line30" in log and "exit status 3" in log


def test_the_page_markup():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('COMMS:"SYSTEM"', 'id="hosts"', "settings ▾", "actions ▾", 'id="bench-tools"', 'id="jobs-n4"', "function drawN4()",
                   '["pi","the cannon Pi"]', '["uav","the UAV adapter"]', '["rctest","rc-test"]'):
        assert needle in page, needle


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_chrome_walk_every_action_on_system(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{a.web_port}/#COMMS"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    dom, err = r.stdout, r.stderr
    assert not re.search(r'"(N2|N4|sessions|v2 cards) [A-Za-z]*Error', err), err[-2000:]
    t0 = dom.index('id="t-COMMS"')
    sysdom = dom[t0:dom.index("</section>", t0)]
    for a_ in SA.ACTIONS:                                                          # every action reachable on SYSTEM (its button)
        if a_.id in ("stopall", "resetall"):
            assert f'id="{a_.id}btn"' in sysdom or f'id="{a_.id[:-3]}allbtn"' in sysdom or "stopallbtn" in sysdom, a_.id
            continue
        assert re.search(rf'id="x?sb-{a_.id}"', sysdom), a_.id
    for h in ("nx", "agx", "mac", "fcu", "pi", "uav", "rctest"):
        assert f'id="hr-{h}"' in sysdom
