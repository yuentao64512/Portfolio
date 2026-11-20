"""v0.1.74 (WP30 = docs/OVERNIGHT_2026-11-09_GUI.md N6): the boat window redesign - /v1/ frozen first (= the v0.1.68 page byte for
byte), the two-row header, the dials under the wireframe, the FCU words card (other ▾), CONTROL & ACTUATION, the MAVLINK console, the GUI
switch + the launcher's classic; the whitelist, the 2 Hz FCU block, the commander lock + PIN and the plate menus' words unchanged."""
from __future__ import annotations

import hashlib
import json
import os
import re
import subprocess
import threading
import time

import pytest

from optiprimex.tools import boat_web as BW

from .conftest import ROOT, free_port

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
TOOLS = ROOT / "optiprimex" / "tools"
V068_SHA = "f1c9a8c4c2be24b4b3048f191e51517d78192b68b48d1347f918be249d3179e6"     # the v0.1.68 boat_web.html


def test_v1_is_the_v0168_page_byte_for_byte():
    assert hashlib.sha256((TOOLS / "boat_web_v1.html").read_bytes()).hexdigest() == V068_SHA


def test_the_unchanged_contracts():
    assert BW.OCS_LINES == ("rc6_policy latched", "rc6_policy hold", "stop_reset", "enable_auto", "disable_auto", "confirm_safe",
                            "station_keep hold", "station_keep loiter")                     # v0.1.81 (WP37)
    assert len(BW.WHITELIST) == 12
    new, old = (TOOLS / "boat_web.html").read_text(), (TOOLS / "boat_web_v1.html").read_text()
    plm = lambda p: p[p.index("const PLM="):p.index("\n", p.index("const PLM=") + 2000)]
    assert plm(new) == plm(old)                                                    # the plate menus' words
    for idn in ("pl-vehicle", "pl-mode", "pl-arm", "pl-holder", "pl-log", "pl-cv", "ctlbar", "ctl-take", "ctl-pin", "fcu-modes", "fcu-arm",
                "fcu-disarm", "b-sr", "wf", "wfd", "p-ev", "p-con", "camc", "radar", "compass", "speed", "lmap"):
        assert f'id="{idn}"' in new, idn                                           # the ids of v0.1.68 still there (the cards move)


def _model():
    from .test_v0152_wa import FCU
    m = BW.Model("usv")
    m.connected, m.broker = True, "127.0.0.1:1884"
    f = dict(FCU, groundspeed_mps=0.42, heading_deg=87, servo=[1500, 1510, 1490, 1500], ekf_pos_ok=True, fix=3, rc6_override=False,
             rates_hz={"HEARTBEAT": 1.0}, log="bench_x.tlog", tail=[[time.time(), "ack cmd 176 result 0", "event"],
                                                                    [time.time(), "mode GUIDED requested", "event"]])
    m.on_message("local/usv/ui/fcu", json.dumps({"body": f}).encode(), now=time.monotonic())
    m.on_message("team/optiprime/v1/status/usv", json.dumps({"body": {"state": "MANUAL", "health": {"faults": ["RC_LOST"], "rc_ok": False}}}).encode(),
                 now=time.monotonic())
    return m


def test_the_n6_block():
    n = _model().snapshot()["n6"]
    assert (n["sog"], n["hdg"], n["fix"], n["pwm"], n["log"], n["faults"]) == (0.42, 87, 3, [1500, 1510, 1490, 1500], "bench_x.tlog", ["RC_LOST"])


def _serve(model, env=None):
    port = free_port()
    old = dict(os.environ)
    os.environ.update(env or {})
    srv = BW.Server(("127.0.0.1", port), BW.make_handler(model, None, bar=None))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, port, old


def test_the_routes_v1_and_classic():
    import urllib.request
    m = _model()
    srv, port, old = _serve(m)
    try:
        get = lambda p: urllib.request.urlopen(f"http://127.0.0.1:{port}{p}", timeout=5).read()
        assert get("/v1/") == (TOOLS / "boat_web_v1.html").read_bytes() and get("/") == (TOOLS / "boat_web.html").read_bytes()
        assert json.loads(get("/v1/state.json"))["n6"]["sog"] == 0.42                 # the classic page's requests reach the same endpoints
        os.environ["BOAT_WEB_UI"] = "classic"
        assert get("/") == (TOOLS / "boat_web_v1.html").read_bytes()                  # --classic
    finally:
        os.environ.clear(); os.environ.update(old)
        srv.shutdown()


def test_the_page_markup():
    p = (TOOLS / "boat_web.html").read_text()
    for needle in ("['MANUAL','HOLD','GUIDED','MODE 20']", "other ▾", "AUTO: the FCU flies its own mission list - the mission manager does not expect this mode",
                   "ACRO:", "STEERING:", "LOITER:", "RTL:", "SMART_RTL:", "if(Date.now()-armed<4000)", "CONTROL & ACTUATION", 'id="d-mavcard"',
                   "MAVLINK console", 'id="d-mavp"', 'id="d-mavf"', 'name="n6ui" value="classic"', "localStorage.getItem('boat.ui')==='classic'",
                   "LOCALLY ON THE NX", "id=\"n6dials\"" if False else "dl.id='n6dials'", "the OCS state line goes"):
        assert needle in p, needle


def test_the_launcher_classic():
    from optiprimex.tools import launcher as L
    P = L.load_presets()
    pre = {"clocks": {}, "nic": None, "refuse": []}
    for uw, host in (("mac-classic", "mac"), ("nx-classic", "nx")):
        c = dict(P["lab"], usv_window=uw, preset="lab")
        assert L.validate(c) == []
        st = [s for s in L.build_plan(c, stamp="x", net_env={}, pre=pre)["steps"] if "USV window" in s["label"]]
        assert st and st[0]["host"] == host and "--classic" in st[0]["cmd"]
    st = [s for s in L.build_plan(dict(P["lab"], usv_window="mac", preset="lab"), stamp="x", net_env={}, pre=pre)["steps"] if "USV window" in s["label"]]
    assert "--classic" not in st[0]["cmd"]
    h = subprocess.run(["bash", str(ROOT / "scripts" / "run_boat_window.sh"), "--help"], capture_output=True, text=True, timeout=10)
    assert "--classic" in (ROOT / "scripts" / "run_boat_window.sh").read_text()


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_chrome_walk_at_1440():
    m = _model()
    srv, port, old = _serve(m)
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{port}/"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    dom, err = r.stdout, r.stderr
    assert not re.search(r'"N6 [A-Za-z]*Error|Uncaught', err), err[-1500:]
    r1 = dom[dom.index('id="n6r1"'):dom.index('id="n6r2"')]
    for idn in ("pl-vehicle", "pl-mode", "pl-arm", "pl-holder", "n6links", "n6al", "n6sog", "n6hdg", "b-sr", "n6g", "ctlbar"):
        assert f'id="{idn}"' in r1, idn                                               # row 1 = the plate
    assert "0.42" in r1 and "087°" in r1
    for w in ("handover", "takeover", "stop", "log", "cv", "points", "note", "save"):
        assert f'data-word="{w}"' in dom, w                                           # usv_ui's whitelist words, as they were
    ov = dom[dom.index('id="t-OVERVIEW"'):]
    assert ov.index('id="wf"') < ov.index('id="n6dials"') < ov.index('id="compass"') < ov.index('id="n6fcl"') < ov.index('id="fcu-modes"')
    modes = re.findall(r'data-label="([^"]+)"', dom[dom.index('id="fcu-modes"'):dom.index('</details>', dom.index('id="n6other"'))])
    assert modes[:4] == ["MANUAL", "HOLD", "GUIDED", "MODE 20"] and set(modes[4:]) == {"ACRO", "STEERING", "LOITER", "AUTO", "RTL", "SMART_RTL"}
    con = dom[dom.index('id="p-con"'):dom.index('</div></div>', dom.index('id="p-con"'))]
    ack = re.search(r'<div class="evr con[^"]*"[^>]*style="display: none;"[^>]*>.*?ack cmd 176 result 0', con, flags=re.S)
    assert ack, con[:600]                                                             # the ack is out of the console feed ...
    mav = dom[dom.index('id="d-mavc"'):dom.index('id="d-mavc"') + 600]
    assert "ack cmd 176 result 0" in mav                                               # ... and in the MAVLINK console
    assert "mode GUIDED requested" in re.sub(r"<[^>]+>", "", con)                     # the rest of the feed as it was
