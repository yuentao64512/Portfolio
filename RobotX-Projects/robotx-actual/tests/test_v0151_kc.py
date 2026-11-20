"""v0.1.51 (WP10k-c, docs/UI_QOL_2026-11-07.md §F items 28 and 30): the boat window's compass rose (hull glyph, HDG solid, COG open,
the commanded yaw dashed or hatched "not published", set vs actual, the ROT scale from ATTITUDE.yawspeed, the speed dial's commanded
surge tick) and the MAP's waypoint route editor (add waypoint, the list with up / down / delete, save as points, start path = TEST_PATH
--names W1..Wn through the OCS page's /cmd, generate pattern = the existing orbit / sway pickers)."""
from __future__ import annotations

import json
import os
import re
import shutil
import socket
import subprocess
import threading
import time
import types

import pytest

from optiprimex.tools import boat_web as BW

from .conftest import ROOT, free_port
from .test_usv_ui import load_usv_ui
from .test_v0149 import CHROME, _rich_model, _serve

TOOLS = ROOT / "optiprimex" / "tools"


def _fcu(m, now, **kw):
    body = {"hb_age_s": 0.3, "mode": "GUIDED", "armed": True, "heading_deg": 40, "groundspeed_mps": 0.8, "lat": 1.28, "lon": 103.85,
            "yawspeed_dps": -7.53, "nav": {"nav_bearing_deg": 75.4, "target_bearing_deg": 80, "xtrack_m": 0.1, "wp_dist_m": 5},
            "ages_s": {"ATTITUDE": 0.1, "NAV_CONTROLLER_OUTPUT": 0.2}}
    body.update(kw)
    m.on_message("local/usv/ui/fcu", json.dumps({"body": body}).encode(), now=now)


# ---- item 28: the model -----------------------------------------------------------------------------------------------------------
def test_compass_carries_cmd_yaw_set_act_and_rot_only_where_published():
    now = 1000.0
    m = _rich_model(now)
    _fcu(m, now)
    s = m.snapshot(now=now)
    c = s["dials"]["compass"]
    assert c["hdg"] == 40 and c["cmd"] == pytest.approx(75.4) and c["rot"] == -7.5 and c["rot_max"] == 30.0
    assert c["set"] == "075°" and c["act"] == "040°"
    assert c["text"][3] == "cmd yaw 75° (FCU GUIDED)" and c["text"][4] == "ROT -7.5°/s" and len(c["text"]) == 5
    assert dict(s["panels"]["nav"])["surge u · sway v · yaw rate r"] == [{"u": "u not published"}, {"u": "v not published"}, "r -7.5°/s"]
    # mode 20 (our frame) / MANUAL: the FCU's nav_bearing is not a set point -> hatched, never a number
    for mode in ("CUSTOM_MANUAL", "MANUAL", "HOLD", None):
        _fcu(m, now + 1, mode=mode)
        c = m.snapshot(now=now + 1)["dials"]["compass"]
        assert c["cmd"] is None and c["set"] == {"u": "not published"} and c["text"][3] == {"u": "SET: nothing commands a heading (hold / manual / handed over)"}, mode   # v0.1.55: the wording of D2
        assert c["rot"] == -7.5                                                     # ROT does not depend on the mode
    # NAV_CONTROLLER_OUTPUT / ATTITUDE old, or yawspeed absent (an older usv_ui) -> not published, never 0
    _fcu(m, now + 2, ages_s={"ATTITUDE": 3.0, "NAV_CONTROLLER_OUTPUT": 3.0})
    c = m.snapshot(now=now + 2)["dials"]["compass"]
    assert c["cmd"] is None and c["rot"] is None and c["text"][4] == {"u": "ROT: not published (no ATTITUDE.yawspeed)"}
    _fcu(m, now + 3, yawspeed_dps=None)
    s = m.snapshot(now=now + 3)
    assert s["dials"]["compass"]["rot"] is None and dict(s["panels"]["nav"])["surge u · sway v · yaw rate r"][2] == {"u": "r not published"}
    # the block itself stale -> nothing from it
    c = m.snapshot(now=now + 3 + BW.BLOCK_STALE_S + 0.5)["dials"]["compass"]
    assert c["rot"] is None and c["cmd"] is None
    # a fresh model with no FCU block: every value unknown, the actual heading too
    c = BW.Model("usv").snapshot(now=5.0)["dials"]["compass"]
    assert c["hdg"] is None and c["act"] == {"u": "no heading"} and c["set"] == {"u": "not published"} and c["rot"] is None
    sp = s["dials"]["speed"]
    assert sp["cmd_surge"] is None and sp["text"][1] == {"u": "cmd surge: no mm/state"}   # v0.1.55 (D2)


def test_usv_ui_publishes_attitude_yawspeed_in_its_fcu_block():
    """End-to-end source of the ROT scale: usv_ui's FCU block carries ATTITUDE.yawspeed in °/s (None without ATTITUDE, never 0)."""
    mod = load_usv_ui()
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.bind(("127.0.0.1", 0))
    port = so.getsockname()[1]
    so.close()
    app = mod.App(mod.build_parser().parse_args([f"udpin:127.0.0.1:{port}", "--mqtt", ""]))
    try:
        app.mq = None
        now = time.monotonic()
        assert app.fcu_block(now)["yawspeed_dps"] is None
        att = types.SimpleNamespace(roll=0.01, pitch=-0.02, yaw=1.0, yawspeed=0.1309)        # rad/s
        import collections
        app.mstats["ATTITUDE"] = {"n": 1, "times": collections.deque([now]), "last": att}
        b = app.fcu_block(now)
        assert b["yawspeed_dps"] == pytest.approx(7.5, abs=0.01) and b["roll_deg"] == pytest.approx(0.6, abs=0.05)
    finally:
        try:
            app.close_links()
        except Exception:
            pass
    src = (ROOT / "optiprimex" / "sim" / "fake_fcu.py").read_text()
    assert '"attitude_send", ms, 0.0, 0.0, p.psi if p.psi <= math.pi else p.psi - 2 * math.pi, 0.0, 0.0, p.r' in src   # the fake FCU sends r


def test_boat_page_static_rules_for_the_rose():
    page = (TOOLS / "boat_web.html").read_text()
    for needle in ('id="compass"', 'id="rot"', 'id="c-set"', 'id="c-act"', 'data-v="hdg"', 'data-v="cog"', 'data-v="cmd"', 'data-v="cmd-unknown"',
                   'data-v="rot-unknown"', 'data-v="surge-unknown"', 'data-v="hull"', "stroke-dasharray=\".07 .05\"", "fill:var(--panel);stroke:var(--text)"):
        assert needle in page, needle
    a, b = page.index("const HU="), page.index("// ---- plotter")
    block = page[a:b]
    cols = set(re.findall(r"#[0-9a-fA-F]{3,6}\b", block))
    assert not cols, cols                                                           # theme tokens only in the dials
    assert ".setact .num{" in page and "height:26px" in page and "grid-template-columns:minmax(0,1fr) 58px" in page   # fixed sizes: no jump


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_rose_in_chrome_light_and_unknown():
    def dom_of(m, theme):
        srv, url = _serve(m)
        try:
            return subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=8000", "--window-size=1440,1000", "--dump-dom",
                                   url + "/?theme=" + theme], capture_output=True, text=True, timeout=90).stdout
        finally:
            srv.shutdown()
    m = _rich_model(time.monotonic())
    _fcu(m, time.monotonic())
    dom = re.sub(r"<script\b.*?</script>", "", dom_of(m, "light"), flags=re.S)
    for v in ("hull", "hdg", "cog", "cmd", "rot"):
        assert f'data-v="{v}"' in dom, v
    assert re.search(r'<svg id="rot"[^>]*data-rot="-7.5"', dom) and re.search(r'<svg id="compass"[^>]*data-cmd="75.4"', dom)
    assert re.search(r'id="c-set">075°<', dom) and re.search(r'id="c-act">040°<', dom) and 'data-v="surge-unknown"' in dom
    groups = re.findall(r'<g id="co-([a-z-]+)" data-x="([\d.]+)" data-y="([\d.]+)" data-w="([\d.]+)" data-h="([\d.]+)"[^>]*data-fit="(\d)"', dom)
    boxes = [{"id": g[0], "x": float(g[1]), "y": float(g[2]), "w": float(g[3]), "h": float(g[4])} for g in groups]
    assert len(groups) == 9 and BW.overlaps(boxes) == [] and all(g[5] == "1" for g in groups)              # the callouts still fit at 1440
    m2 = _rich_model(time.monotonic())                                                  # MANUAL, no yawspeed: hatched, never 0
    _fcu(m2, time.monotonic(), mode="MANUAL", yawspeed_dps=None)
    dom = re.sub(r"<script\b.*?</script>", "", dom_of(m2, "dark"), flags=re.S)
    assert 'data-v="cmd-unknown"' in dom and 'data-v="rot-unknown"' in dom and 'data-v="cmd"' not in dom and 'data-v="rot"' not in dom
    assert re.search(r'id="c-set"><span class="u"[^>]*>not published</span>', dom) and re.search(r'<svg id="rot"[^>]*data-rot=""', dom)


# ---- item 30: the route editor on the MAP -------------------------------------------------------------------------------------------
def _route_js():
    html = (TOOLS / "map.html").read_text()
    a, b = html.index("// ---- route: pure"), html.index("// ---- route: end pure ----")
    return html[a:b]


def test_route_pure_functions_in_node():
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed")
    js = _route_js() + """
const P=[{lat:1.281,lon:103.855},{lat:1.2811,lon:103.8551},{lat:1.2812,lon:103.8552}];
const out={names:routeNames(P),save:routeSaveLines(P),start:routeStartLine(P),
  up:routeNames(routeMove(P,1,-1)).length, upll:routeMove(P,1,-1)[0], dn_edge:routeMove(P,2,1)[2], del:routeDel(P,1),
  miss:routeMissing(P,[{points:{W1:[0,0]}},{points_ll:{W2:{lat:1,lon:2}}}]), none:routeMissing(P,[]),
  load_bad:routeLoad('{"pts":[{"lat":"x","lon":1},{"lat":1.5,"lon":2.5}],"saved":true}'), load_junk:routeLoad('not json'), load_null:routeLoad(null)};
console.log(JSON.stringify(out));"""
    out = json.loads(subprocess.run([node, "-e", js], capture_output=True, text=True, timeout=20).stdout)
    assert out["names"] == ["W1", "W2", "W3"]
    assert out["save"] == ["save_point W1 --lat 1.28100000 --lon 103.85500000", "save_point W2 --lat 1.28110000 --lon 103.85510000",
                           "save_point W3 --lat 1.28120000 --lon 103.85520000"]
    assert out["start"] == "start_task m0t2 --names W1,W2,W3"
    assert out["upll"] == {"lat": 1.2811, "lon": 103.8551} and out["dn_edge"] == {"lat": 1.2812, "lon": 103.8552}      # reorder; the edge stays
    assert out["del"] == [{"lat": 1.281, "lon": 103.855}, {"lat": 1.2812, "lon": 103.8552}]
    assert out["miss"] == ["W3"] and out["none"] == ["W1", "W2", "W3"]
    assert out["load_bad"] == {"pts": [{"lat": 1.5, "lon": 2.5}], "saved": True} and out["load_junk"] == {"pts": [], "saved": False}
    assert out["load_null"] == {"pts": [], "saved": False}


def test_route_lines_parse_as_the_ocs_commands_and_test_path_is_m0t2():
    """The lines the editor posts are the OCS page's own words: ocs_cli parses them, m0t2 is TEST_PATH, a run-free TEST MODE task."""
    from optiprimex.ocs.cmd_authority import RUN_FREE_TASKS
    from optiprimex.tools import ocs_cli, playbook
    assert playbook.MISSIONS["m0t2"].task == "TEST_PATH" and "TEST_PATH" in RUN_FREE_TASKS
    ns = ocs_cli.parse(["start_task", "m0t2", "--names", "W1,W2,W3"])
    assert ocs_cli.split_names(ns.names) == ["W1", "W2", "W3"]
    ns = ocs_cli.parse(["save_point", "W1", "--lat", "1.28100000", "--lon", "103.85500000"])
    assert ns.lat == pytest.approx(1.281) and ns.lon == pytest.approx(103.855)
    assert ocs_cli.check_names(["W1", "W3"], {"W1", "W2"}).startswith("point W3 not saved on the boat")


def test_route_editor_static():
    html = (TOOLS / "map.html").read_text()
    head = html.split("<script>")[0]
    for bid in ("r_add", "r_save", "r_start", "r_clear", "r_pick"):
        btn = re.search(r'<button id="%s"[^>]*>' % bid, head).group(0)
        assert "title=" in btn, bid
    assert "body.mini #route, body.judge #route { display:none !important; }" in html
    assert "localStorage.setItem(ROUTE_KEY" in html and "try { localStorage.setItem(ROUTE_KEY" in html     # per browser, guarded
    assert "OCSPOST('/cmd', { line: lines[i] })" in html and "OCSPOST('/cmd', { line })" in html             # the drop-point / add-waypoint path
    assert "if (!ROUTE.saved && !(await routeSave())) return;" in html and "routeWaitDigest(8000)" in html     # save first, then start
    assert "getElementById('csel')" in html and 'value="m0t6"' in html and 'value="m0t3"' in html              # the existing pickers
    assert "if (m && !dropMode && !ROUTE.on)" in html and "ROUTE.on && !JUDGE && !MINI" in html
    a, b = html.index("// ---- v0.1.51 (WP10k-c item 30)"), html.index("// ---- end v0.1.51 item 30")
    assert not re.findall(r"#[0-9a-fA-F]{6}\b", html[a:b].replace("'#4bb9d2'", "").replace("'#151c24'", ""))   # tokens (fallbacks only)


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_route_editor_renders_in_the_ocs_served_map(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json")}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        dom = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--dump-dom",
                              f"http://127.0.0.1:{a.web_port}/map.html?host=127.0.0.1&port=1"], capture_output=True, text=True, timeout=90).stdout
        assert '<div id="rlist"><div class="dim">no waypoints: add waypoint, then click the map</div></div>' in dom
        body = re.sub(r"<script\b.*?</script>", "", dom, flags=re.S)
        assert [x for x in re.findall(r"<button\b[^>]*>", body) if "title=" not in x] == []
    finally:
        srv.shutdown()
