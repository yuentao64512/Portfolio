"""v0.1.51 (ROADMAP WP10k-c, UI_QOL section F) - the OCS page's part: .kpi numerals with three presets (Dmitrii: "all three, select
preset in settings"), the three header status blocks, the HUD box around the camera (default on, a toggle, no layout change -
Dmitrii's answer), the mission status card on the map, the alert table, the LINKS card, the command row's icons + --action;
the boat window's header numerals; status `yaw_rate_dps` (ATTITUDE.yawspeed) for the HUD. Items 28 / 30: tests/test_v0151_kc.py."""
from __future__ import annotations

import re

from optiprimex.companion.state import FcuState, VehicleState

from .conftest import ROOT

TOOLS = ROOT / "optiprimex" / "tools"


def page():
    return (TOOLS / "ocs_web.html").read_text()


def test_the_kpi_presets_never_more_than_six():
    p = page()
    sets = re.search(r"const KPI_SETS=(\{[^}]*\})", p).group(1)
    for k in ("A", "B", "C"):
        keys = re.search(k + r':\[([^\]]*)\]', sets).group(1).split(",")
        assert 1 <= len(keys) <= 6, k
    assert 'A:["clock","run","sog","hdg","hb","alarms"]' in sets and '"stage"' in sets and '"dist"' in sets
    assert 'id="kpiset"' in p and 'localStorage.getItem("ocs.kpi")' in p and 'id="ov-kpi"' in p
    assert ".kpi .v{font:700 30px ui-monospace" in p and "tabular-nums" in p
    b = (TOOLS / "boat_web.html").read_text()
    assert 'id="bkpi"' in b and "function drawBkpi(S)" in b


def test_the_header_blocks_replace_the_chips():
    p = page()
    for i in ("hb-link", "hb-fcu", "hb-state"):
        assert f'id="{i}"' in p and f'$("{i}")' in p, i
    assert 'id="links" style="display:none"' in p and 'id="fcuchip" style="display:none"' in p
    assert ".sblk{display:inline-flex;flex-direction:column;justify-content:center;height:36px;width:236px" in p
    assert ".sblk.bad{background:var(--alarmBg)" in p and "LINK LOST" in p and "KILLED" in p
    assert 'id="sb-link"' not in p                                                 # sb-<id> are the system cards' buttons


def test_the_hud_box_default_on_and_no_resize():
    p = page()
    assert 'id="hudbox"' in p and 'id="hud-on"' in p and 'localStorage.getItem("ocs.hud")||"1"' in p
    assert ".hudbox{display:grid;grid-template-rows:34px auto 30px" in p and ".hudbox.off .hud-side>*" in p and "visibility:hidden" in p
    assert "yaw_rate_dps" in p and "roll_deg" in p
    assert "gradient" not in p[p.index("function drawHud()"):p.index("function drawLinks()")]
    vs = VehicleState()
    assert vs.status_body()["yaw_rate_dps"] is None                                # never fabricated: none until an ATTITUDE came
    vs.fcu = FcuState(yaw_rate_dps=12.345)
    assert vs.status_body()["yaw_rate_dps"] == 12.3


def test_the_mission_card_the_alert_table_links_and_icons():
    p = page()
    assert 'class="msc" id="msc"' in p and "function drawMsc()" in p and "elapsed / typical" in p
    assert 'table class="alt"' in p and "albig" in p and 'id="al-sort"' in p and "acknowledged" in p
    assert 'id="n-links"' in p and '"tailnet"' in p and '"boat radio"' in p and '"judge wire"' in p and "RSSI n/p" in p
    for k in ("c-start", "c-hold", "c-resume", "c-abort", "c-restart", "c-home"):
        i = p.index(f'id="{k}"')
        assert '<svg class="ic"' in p[i:p.index("</button>", i)], k
    assert "function cmdAction()" in p and "cmdAction();" in p and ".cbtns button[hidden]{display:none}" in p


def test_the_fast_feed(tmp_path):
    """v0.1.51.1: /fast.json = only the fast subset (< 1 KB), from the fresher of mm/state (5 Hz) and status (2 Hz); the page
    polls it at 5 Hz for the HUD, the numerals, the blocks and the map card; /state.json stays at 1 Hz."""
    import json
    import threading
    import time
    import urllib.request
    from optiprimex.tools import ocs_web as OW
    from .conftest import free_port
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    u = f"http://127.0.0.1:{a.web_port}/fast.json"
    try:
        f = json.loads(urllib.request.urlopen(u, timeout=5).read())
        assert f["v"] == {} and f["src"] is None                                     # nothing seen: nothing invented
        now = time.time()
        with web.m.lock:
            web.m.status, web.m.status_t = {"state": "MANUAL", "hdg_deg": 10.0, "health": {"fcu_age_s": 0.3}}, now - 0.4
            web.m.local, web.m.local_t = {"state": "MANUAL", "hdg_deg": 12.5, "roll_deg": 1.0, "yaw_rate_dps": 3.2, "profile": "x",
                                          "fcu": {"big": "x" * 3000}, "health": {"fcu_age_s": 0.1}}, now - 0.1
        raw = urllib.request.urlopen(u, timeout=5).read()
        f = json.loads(raw)
        assert len(raw) < 1024 and f["src"] == "local" and f["v"]["hdg_deg"] == 12.5 and f["v"]["yaw_rate_dps"] == 3.2
        assert "profile" not in f["v"] and "fcu" not in f["v"] and f["v"]["fcu_age_s"] == 0.1
    finally:
        srv.shutdown()
    p = page()
    assert "setInterval(fastTick,200)" in p and 'fetch("/fast.json"' in p and "setInterval(tick,1000)" in p
    assert "hdrBlocks();drawKpi();drawMsc();drawHud()" in p[p.index("async function fastTick"):]
