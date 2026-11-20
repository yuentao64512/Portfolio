"""v0.1.78.13 / .15 / .16 (Dmitrii 2026-11-09, Team Village): the drone's declare + pilot ready on the main page and the VEHICLES tab -
.16: "jerky, pilot ready not even pressing - big clear buttons, not buried": a strip built ONCE per place (RUN tile, VEHICLES tab) whose
two big buttons only change text / class / disabled, "sending…" until the OCS state answers; the words declare uav on|off, uav ready on|off."""
from __future__ import annotations

import os
import re
import subprocess
import threading
import time

import pytest

from .conftest import ROOT, free_port

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def _dom(tmp_path, uav, tab):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    with web.m.lock:
        web.m.ocs, web.m.ocs_t = {"r": "R2", "vehicles": {"USV1": {"state": "MANUAL", "age_s": 0.2}}, "uav": uav,
                                  "reporting": {"hb": {"UAV1": {"n": 7}}}}, time.time()
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=8000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{a.web_port}/#{tab}"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    assert not re.search(r'"(N[0-9]+|UAVS) [A-Za-z]*Error|Uncaught', r.stderr), r.stderr[-1500:]
    return r.stdout


def _strip(d, px):
    i = d.index(f'id="{px}-ustrip"')
    return d[d.rindex("<div", 0, i):d.index("</div>", i)]


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_undeclared_then_declared(tmp_path):
    d = _dom(tmp_path, {"configured": True, "vehicle_id": "UAV1", "declare": False, "declared": False, "pilot_ready": False}, "OVERVIEW")
    rv = _strip(d, "rv")
    assert re.search(r'id="rv-bdecl"[^>]*>DECLARE UAV<', rv) and re.search(r'id="rv-bready"[^>]*>PILOT READY\?<', rv)
    assert not re.search(r'<button[^>]*id="rv-bready"[^>]*disabled', rv)         # v0.1.78.17: clickable before the declaration too
    assert "not declared: silent on RoboCommand" in rv and "UAV1" in rv
    built = sorted(int(x) for x in re.findall(r'data-built="(\d+)"', d))
    assert built == [1, 2], built                                        # two strips, each built once over ~8 s of refreshes
    d = _dom(tmp_path, {"configured": True, "vehicle_id": "UAV1", "declare": True, "declared": True, "pilot_ready": True}, "UAV")
    vc = _strip(d, "vc")
    assert re.search(r'id="vc-bdecl"[^>]*>✓ UAV DECLARED<', vc) and re.search(r'id="vc-bready"[^>]*>✓ PILOT READY<', vc)
    assert not re.search(r'<button[^>]*id="vc-bready"[^>]*disabled', vc) and "declared · heartbeats 7" in vc


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_no_uav_configured(tmp_path):
    d = _dom(tmp_path, {"configured": False}, "OVERVIEW")
    assert re.search(r'id="rv-ustrip"[^>]*style="display: none;"', d)


def test_the_words_and_no_checkboxes():
    p = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'send("declare uav "+(on?"off":"on"))' in p and 'send("uav ready "+(u.pilot_ready?"off":"on"))' in p
    assert "rv-udecl" not in p and "uavCtlHtml" not in p
    dc = p[p.index('$("dc-in").innerHTML='):]
    dc = dc[:dc.index("\n")]
    assert 'type="checkbox"' not in dc and 'id="dc-uav"' not in p                  # v0.1.78.17: the constructor has no checkbox
