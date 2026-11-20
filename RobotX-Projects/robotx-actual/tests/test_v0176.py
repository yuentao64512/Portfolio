"""v0.1.76 (WP32 = N5's VEHICLES, docs/OVERNIGHT_2026-11-09_GUI.md, board 04): one card per vehicle - USV1 and the UAV - the same rows in
the same order; the tab is VEHICLES and is never greyed (the boat's card is always there); the old UAV detail stays below."""
from __future__ import annotations

import json
import os
import re
import subprocess
import threading
import time

import pytest

from .conftest import ROOT, free_port
from .test_v0165 import UID, uav_body

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
ROWS = ["state", "task · stage", "FCU mode · armed", "kill · E-stop", "position", "heading · SOG", "height", "link age",
        "heartbeat → judge", "faults", "fence"]


def test_the_markup():
    p = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('UAV:"VEHICLES"', 'id="veh-cards"', "function drawVeh()", 'id="uav-kv"'):
        assert needle in p, needle
    assert 'classList.toggle("greyed",!up)' not in p                                     # the tab is never greyed now
    for r in ROWS:
        assert p.count(f'row("{r}"') >= 2, r                                             # each row once per vehicle


def _walk(web, a):
    from optiprimex.tools import ocs_web as OW
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{a.web_port}/?probe=1#UAV"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    assert not re.search(r'"(N8|N2|N4|N5|N7) [A-Za-z]*Error', r.stderr), r.stderr[-1500:]
    dom = r.stdout
    box = dom[dom.index('id="veh-cards"'):]
    box = box[:box.index("</section>")]
    cards = re.findall(r'<div class="card vcard2">(.*?)</div></div>', box)
    return dom, cards


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_two_cards_the_same_columns(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    with web.m.lock:
        web.m.status = {"state": "AUTO_TASK", "robot_state": "STATE_AUTO", "current_task": "TASK_NAV_CHANNEL", "task_state": "GATE_1",
                        "mode": "GUIDED", "armed": True, "kill": False, "estop": False, "lat": 1.3001, "lon": 103.8002, "hdg_deg": 87.0,
                        "sog_mps": 0.42, "health": {"faults": []}}
        web.m.status_t = time.time()
        web.m.ocs, web.m.ocs_t = {"r": "R2", "uav": {"configured": True, "vehicle_id": UID}}, time.time()
    web.uav.on_message(json.dumps({"vehicle_id": UID, "body": uav_body()}).encode(), now=time.time())
    dom, cards = _walk(web, a)
    assert len(cards) == 2, cards
    lab = [re.findall(r"<span>([^<]+)</span><span", c) for c in cards]
    assert lab[0] == ROWS and lab[1] == ROWS, lab                                         # the same rows, the same order, both vehicles
    txt = [re.sub(r"<[^>]+>", " ", c) for c in cards]
    assert "USV1" in txt[0] and "AUTO_TASK" in txt[0] and "TASK_NAV_CHANNEL · GATE_1" in txt[0] and "ARMED" in txt[0] and "0.42 m/s" in txt[0]
    assert UID in txt[1] and "AUTO_HOLD" in txt[1] and "8.0 m HAE" in txt[1] and "4 pts · ceiling 20.0 m" in txt[1]
    nav = re.search(r'<button[^>]*data-t="UAV"[^>]*>', dom).group(0)
    assert "greyed" not in nav and 'id="uav-kv"' in dom                                  # never greyed; the UAV detail stays below
    kv = dom[dom.index('id="uav-kv"'):]
    kv = kv[:kv.index("</div>")]
    labels = re.findall(r'<span>([a-zA-Z ·]+)</span><span[ >]', kv)
    assert labels[:5] == ["identity", "link", "state", "FCU", "HAE · speed"], labels   # the UAV detail's columns fixed (v0.1.65's state took two cells)
    assert re.search(r'data-t="UAV"[^>]*>VEHICLES<', dom)


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_no_uav_still_two_cards(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    dom, cards = _walk(web, a)
    assert len(cards) == 2 and "no UAV adapter on the hub" in cards[1]
