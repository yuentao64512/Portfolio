"""v0.1.52 (the BOAT window stream, Dmitrii 2026-11-06): the top plate (VEHICLE / FCU MODE / ARM colour classes, the HOLDER / LOG / CV
boxes incl. unknown), the SIMULATION flag, EVENTS + usv_ui's CONSOLE on OVERVIEW with levels, the light map's data, the wireframe's
short callouts (no "to verify", the detail in the tooltip / click panel) and the page at 1440 x 900 in light and dark: the plate,
EVENTS and CONSOLE inside the first 900 px, nothing overlapping."""
from __future__ import annotations

import json
import os
import re
import socket
import subprocess
import threading
import time

import pytest

from optiprimex.tools import boat_web as BW

from .conftest import ROOT
from .test_usv_ui import load_usv_ui
from .test_v0143 import _cr
from .test_v0149 import CHROME, _rich_model, _serve

NS = "team/optiprime/v1"
TOOLS = ROOT / "optiprimex" / "tools"


def _m(fcu=None, status=None, now=100.0):
    m = BW.Model("usv")
    m.connected, m.broker = True, "127.0.0.1:x"
    if fcu is not None:
        m.on_message("local/usv/ui/fcu", json.dumps({"body": fcu}).encode(), now=now)
    if status is not None:
        m.on_message(f"{NS}/status/usv", json.dumps({"body": status}).encode(), now=now)
    return m


FCU = {"hb_age_s": 0.3, "active": False, "fcu_link": "fake", "mode": "GUIDED", "armed": True, "kill": False, "estop": False}


# ---- the top plate ------------------------------------------------------------------------------------------------------------------
def test_plate_mode_and_arm_colour_classes():
    assert [BW.vehicle_class(v) for v in ("AUTO_HOLD", "AUTO_TASK", "AUTO_SAFE_HOLD", "MANUAL", "KILLED", "FAULT", "BOOT", None)] == \
        ["auto", "auto", "auto", "manual", "alarm", "alarm", "neutral", "unk"]
    assert BW.fcu_mode_class("AUTO") == BW.fcu_mode_class("GUIDED") == "auto" and BW.fcu_mode_class("MANUAL") == "manual"
    assert BW.fcu_mode_class("CUSTOM_MANUAL", "mission manager") == "auto" and BW.fcu_mode_class("CUSTOM_MANUAL", "usv_ui") == "manual"
    assert BW.fcu_mode_class("HOLD") == "hold" and BW.fcu_mode_class("LOITER") in ("auto", "neutral") and BW.fcu_mode_class(None) == "unk"   # v0.1.78.10: HOLD green
    p = _m(FCU, {"state": "AUTO_TASK", "task_name": "T1", "task_state": "TRANSIT", "kill": False, "estop": False}).snapshot(now=100.2)["plate"]
    assert (p["vehicle"]["cls"], p["mode"]["cls"], p["arm"]["cls"]) == ("auto", "auto", "armed")
    assert p["vehicle"]["text"] == "AUTO_TASK" and p["mode"]["text"] == "GUIDED" and p["arm"]["text"] == "ARMED" and "T1" in p["vehicle"]["sub"]
    assert p["arm"]["sub"] == "no kill · no E-stop" and not p["arm"]["alarm"] and not p["mode"]["stale"]
    p = _m(dict(FCU, mode="MANUAL", armed=False, active=True), {"state": "MANUAL"}).snapshot(now=100.2)["plate"]
    assert (p["vehicle"]["cls"], p["mode"]["cls"], p["arm"]["cls"], p["arm"]["text"]) == ("manual", "manual", "disarmed", "DISARMED")
    p = _m(dict(FCU, mode="CUSTOM_MANUAL", active=True, kill=True), {"state": "KILLED", "kill": True}).snapshot(now=100.2)["plate"]
    assert p["mode"]["text"] == "MODE 20" and p["mode"]["cls"] == "manual" and p["vehicle"]["cls"] == "alarm" and p["arm"]["alarm"]
    # the heartbeat is old: the last values, marked stale (never a fresh-looking guess)
    p = _m(dict(FCU, hb_age_s=4.0), {"state": "AUTO_HOLD"}).snapshot(now=100.2)["plate"]
    assert p["mode"]["stale"] and p["arm"]["stale"] and "NO HB" in p["mode"]["sub"]
    # nothing heard: every block unknown (hatched), no colour
    p = BW.Model("usv").snapshot(now=5.0)["plate"]
    assert [p[k]["cls"] for k in ("vehicle", "mode", "arm")] == ["unk"] * 3 and [t["cls"] for t in p["toggles"]] == ["unk"] * 3
    assert [t["text"] for t in p["toggles"]] == ["unknown"] * 3


def test_plate_toggles_holder_log_cv_and_their_sources():
    m = _m(dict(FCU, log="20261106_1012", cv_on=True))
    tg = {t["id"]: t for t in m.snapshot(now=100.2)["plate"]["toggles"]}
    assert tg["holder"]["text"] == "mission manager" and tg["holder"]["sub"] == "after handover"
    assert (tg["log"]["text"], tg["log"]["cls"], tg["log"]["sub"], tg["log"]["src"]) == ("ON", "on", "20261106_1012", "usv_ui FCU block")
    assert (tg["cv"]["text"], tg["cv"]["cls"]) == ("ON", "on")
    tg = {t["id"]: t for t in _m(dict(FCU, active=True, log=None, cv_on=False)).snapshot(now=100.2)["plate"]["toggles"]}
    assert tg["holder"]["text"] == "usv_ui" and (tg["log"]["text"], tg["log"]["cls"]) == ("off", "off") and tg["cv"]["cls"] == "off"
    # an older usv_ui (no cv_on in its block): the presence beat, then its map's cv.on; neither = unknown, never "off"
    m = _m({k: v for k, v in FCU.items()})
    tg = {t["id"]: t for t in m.snapshot(now=100.2)["plate"]["toggles"]}
    assert tg["cv"]["cls"] == "unk" and tg["log"]["cls"] == "unk" and tg["cv"]["sub"] == "not published"
    m.on_message("presence/usv_ui", json.dumps({"node": "usv_ui", "up": True, "log": "a.tlog", "cv_on": False, "t_unix": time.time()}).encode(), now=100.1)
    tg = {t["id"]: t for t in m.snapshot(now=100.2)["plate"]["toggles"]}
    assert (tg["log"]["cls"], tg["log"]["src"], tg["cv"]["cls"], tg["cv"]["src"]) == ("on", "usv_ui presence", "off", "usv_ui presence")
    m2 = _m()
    m2.on_message(f"{NS}/telemetry/usv/map", json.dumps({"body": {"co": {"lat": 1.0, "lon": 2.0}, "points": {}, "cv": {"on": True}}}).encode(), now=100.0)
    tg = {t["id"]: t for t in m2.snapshot(now=100.5)["plate"]["toggles"]}
    assert (tg["cv"]["cls"], tg["cv"]["src"]) == ("on", "usv_ui map") and tg["holder"]["cls"] == "unk"
    assert {t["id"]: t for t in m2.snapshot(now=110.0)["plate"]["toggles"]}["cv"]["cls"] == "unk"           # the map went old


def test_usv_ui_publishes_cv_on_in_its_block_and_presence():
    mod = load_usv_ui()
    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.bind(("127.0.0.1", 0))
    port = so.getsockname()[1]
    so.close()
    app = mod.App(mod.build_parser().parse_args([f"udpin:127.0.0.1:{port}", "--mqtt", ""]))
    try:
        app.mq = None
        b = app.fcu_block(time.monotonic())
        assert b["cv_on"] is False and b["log"] is None and b["fcu_link"] in ("fake", "real")
        app.cv_on = True
        assert app.fcu_block(time.monotonic())["cv_on"] is True
    finally:
        try:
            app.close_links()
        except Exception:
            pass
    src = (ROOT / "usv_ui" / "usv_ui.py").read_text()
    assert '"active": self.active, "cv_on": self.cv_on,' in src                                        # the presence beat too (v0.1.83.5: + log_t0 / log_bytes)


def test_plate_tokens_and_text_contrast():
    """The plate's fills are theme tokens (no hex in the page) and every text on its fill reads at >= 4.5:1 in all five themes."""
    page = (TOOLS / "boat_web.html").read_text()
    css = page[page.index("/* v0.1.52: the top plate"):page.index("#sim{")]
    assert not re.findall(r"#[0-9a-fA-F]{3,6}\b", css)
    for rule in (".pb.auto{background:var(--ok);border-color:var(--ok);color:var(--panel)}",
                 ".pb.alarm,.pb.armed{background:var(--alarm);border-color:var(--alarm);color:var(--panel)}",
                 ".pb.manual{background:var(--tpY);border-color:var(--tpY);color:var(--tpYText)}",
                 ".pb.on{background:var(--accBg);border-color:var(--acc);color:var(--accText)}",
                 '[data-theme="fun"]{--tpY:var(--warn);--tpYText:var(--bg)}', '[data-theme="hc-light"]{--tpY:var(--action);--tpYText:var(--actionText)}'):
        assert rule in css, rule
    tcss = (TOOLS / "themes.css").read_text()
    for t in ("dark", "light", "hc-dark", "hc-light", "fun"):
        v = dict(re.findall(r"--(\w+):(#[0-9a-fA-F]{6})", ";".join(b for n, b in re.findall(r'\[data-theme="([^"]+)"\]\{([^}]*)\}', tcss) if n == t)))
        y, yt = ("action", "actionText") if t in ("light", "hc-light") else ("warn", "bg")
        for fill, text in (("ok", "panel"), ("alarm", "panel"), (y, yt), ("panel2", "text"), ("panel2", "muted"), ("accBg", "accText"), ("panel", "text")):
            assert _cr(v[text], v[fill]) >= 4.5, (t, fill, text, _cr(v[text], v[fill]))


# ---- the SIM flag ---------------------------------------------------------------------------------------------------------------------
def test_sim_flag_from_the_fake_link_or_the_sim_camera():
    s = _m(FCU).snapshot(now=100.2)["sim"]
    assert s["on"] and s["link"] == "fake" and s["src"] == "usv_ui FCU block" and s["text"].startswith("SIMULATION · FCU link fake")
    assert not _m(dict(FCU, fcu_link="real")).snapshot(now=100.2)["sim"]["on"]
    e = BW.Model("usv").snapshot(now=5.0)["sim"]
    assert not e["on"] and e["link"] is None                                                       # unknown link: no banner, no guess
    s = _m(None, {"state": "MANUAL", "fcu_link": "fake"}).snapshot(now=100.2)["sim"]
    assert s["on"] and s["src"] == "status/usv"
    m = _m(dict(FCU, fcu_link="real"))
    m.cam = (100.0, {"source": "sim", "age_s": 0.1, "clients": 1}, "http://127.0.0.1:8097/status", None)
    s = m.snapshot(now=100.2)["sim"]
    assert s["on"] and s["why"] == ["camera = the simulator's stream"]
    page = (TOOLS / "boat_web.html").read_text()
    assert '<div id="sim" hidden title="SIMULATION:' in page and "background:var(--accBg)" in page[page.index("#sim{"):page.index("#sim b")]


# ---- EVENTS / CONSOLE --------------------------------------------------------------------------------------------------------------------
def test_event_levels_console_tail_and_replies():
    assert BW.ev_level("KILL asserted") == "alarm" and BW.ev_level("FCU ARMED") == "alarm" and BW.ev_level("bridge to the OCS DOWN (ocs)") == "alarm"
    assert BW.ev_level("usv_ui: cv -> REFUSED: no AGX") == "alarm" and BW.ev_level("FCU disarmed") == "ok" and BW.ev_level("FCU DISARMED") == "ok"
    assert BW.ev_level("usv_ui HANDED OVER: the mission manager may hold the FCU") == "warn" and BW.ev_level("vehicle MANUAL") == "warn"
    assert BW.ev_level("vehicle AUTO_TASK") == "ok" and BW.ev_level("usv_ui: log -> log ON x") == "ok" and BW.ev_level("task stage TRANSIT") == "info"
    m = _m(dict(FCU, tail=[[3.0, "> handover", "cmd"], [1.0, "FCU ARMED", "event"]]))
    m.event("FCU ARMED")
    m.cmds.appendleft({"t": 1.0, "word": "log", "ok": True, "text": "log ON"})
    s = m.snapshot(now=100.5)
    assert s["events"][0]["level"] == "alarm"
    c = s["console"]
    assert c["state"] == "live" and [r["text"] for r in c["rows"]] == ["FCU ARMED", "> handover"]               # newest first
    assert c["rows"][0]["age_s"] == 1.5 and c["rows"][1]["kind"] == "cmd" and c["replies"][0]["word"] == "log"
    assert m.console(103.0)["state"] == "stale" and BW.Model("usv").console(1.0) == {"state": "none", "age_s": None, "rows": [], "replies": []}


# ---- the light map ----------------------------------------------------------------------------------------------------------------------
def test_light_map_data_one_frame_and_auto_fit():
    m = _m(FCU)
    assert m.light_map(100.0)["state"] == "unknown" and m.light_map(100.0)["bounds"] is None
    m.on_message(f"{NS}/telemetry/usv/map", json.dumps({"body": {"co": {"lat": 1.0, "lon": 103.0}, "boat": {"x_n": 4, "y_e": 2, "hdg_deg": 30,
        "trail": [[0, 0], [2, 1], [4, 2]]}, "entities": [{"id": 1, "kind": "buoy", "colour": "red", "x_n": 30, "y_e": -5},
                                                       {"id": 2, "kind": "dock", "colour": "green", "x_n": 10, "y_e": 10}]}}).encode(), now=100.0)
    # usv_ui's map: its CO 1e-5 deg (1.11 m) north of the mission manager's -> its points shift by that into one frame
    m.on_message(f"{NS}/telemetry/usv/map", json.dumps({"body": {"co": {"lat": 1.00001, "lon": 103.0}, "points": {"P1": [5, 5]}, "cv": {"on": True},
                                                                  "boat": {"x_n": 3, "y_e": 2, "hdg_deg": 31}, "entities": []}}).encode(), now=100.1)
    m.course = [[0.99995, 102.99995], [1.00035, 102.99995], [1.00035, 103.00015], [0.99995, 103.00015]]
    L = m.light_map(100.3)
    assert L["state"] == "live" and L["boat"] == {"n": 4.0, "e": 2.0, "hdg": 30.0, "age_s": 0.3}           # the mission manager's while current
    assert L["trail"][-1] == [4.0, 2.0] and [i["colour"] for i in L["items"]] == ["red", "green"] and L["items"][0]["hex"] == BW.OBJ["red"]
    assert L["points"] == [{"name": "P1", "n": 6.11, "e": 5.0}] and L["home"]["n"] == 0.0
    assert len(L["edge"]) == 4 and L["edge"][0] == [pytest.approx(-5.57, abs=0.01), pytest.approx(-5.57, abs=0.01)]
    b = L["bounds"]
    allp = [[4, 2], [30, -5], [10, 10], [6.11, 5]] + L["edge"]
    assert all(b[0] <= p[0] <= b[1] and b[2] <= p[1] <= b[3] for p in allp) and b[1] - b[0] >= 20 and b[3] - b[2] >= 20
    assert BW.fit_bounds([[0, 0]]) == [-10.0, 10.0, -10.0, 10.0] and BW.fit_bounds([]) is None
    assert m.light_map(104.0)["state"] == "stale" and m.light_map(104.0)["text"][1] == {"w": "map STALE 3.9 s"}
    # the radar draws the same digest (no flip-flop between the two publishers)
    assert m.radar(100.3)["hdg"] == 30.0 and len(m.radar(100.3)["items"]) == 2


# ---- the wireframe ------------------------------------------------------------------------------------------------------------------------
def test_short_callouts_detail_and_no_to_verify():
    m = _rich_model(1000.0)
    s = m.snapshot(now=1000.0)
    assert "to verify" not in json.dumps(s) and "to verify" not in (TOOLS / "boat_web.html").read_text()
    co = {b["id"]: b for b in s["callouts"]}
    assert len(co) == 9 and BW.overlaps(s["callouts"]) == [] and all(len(b["lines"]) == 1 for b in co.values())
    assert co["thr-mp"]["title"] == "M-P · ch 1" and co["thr-mp"]["lines"] == ["+40% · 1660"] and co["thr-sa"]["lines"] == ["idle 1500"]
    assert co["cam"]["lines"] == ["up 4.0 Hz · 2 src"] and co["lamp"]["lines"] == ["GREEN · expected"] and co["ant"]["lines"][0].startswith("ok · status")
    assert co["box"]["lines"] == ["ok · hosts unknown"] and co["box"]["state"] == "available"
    assert any("box → m0t4" in x for x in co["cam"]["detail"]) and any("stream: sim" in x for x in co["cam"]["detail"])
    assert {k: b["tab"] for k, b in co.items()} == {"thr-mp": "ACTUATION", "thr-ms": "ACTUATION", "thr-sf": "ACTUATION", "thr-sa": "ACTUATION",
                                                    "ant": "DIAG", "box": "DIAG", "cam": "NAV", "lamp": None, "cannon": None}
    page = (TOOLS / "boat_web.html").read_text()
    for eid in ("thr-mp", "thr-ms", "thr-sf", "thr-sa", "cam", "lamp", "ant", "box", "cannon"):
        assert re.search(r'id="%s"[^>]*data-el="%s"><title>' % (eid, eid), page), eid                   # clickable, with a tooltip
    assert "window.setState=setState" in page and 'id="wfd"' in page and "function pick(id)" in page


# ---- the page in Chrome at 1440 x 900 ---------------------------------------------------------------------------------------------------
def _layout(dom):
    m = re.search(r'<body[^>]*data-layout="([^"]+)"', dom)
    return json.loads(m.group(1).replace("&quot;", '"')) if m else None


def _hit(a, b):
    return a[0] < b[0] + b[2] and b[0] < a[0] + a[2] and a[1] < b[1] + b[3] and b[1] < a[1] + a[3]


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_plate_events_and_console_above_the_fold_in_light_and_dark():
    m = _rich_model(time.monotonic())
    for i in range(30):
        m.event("vehicle AUTO_TASK %d" % i)
    stop = threading.Event()

    def feed():                                   # the 2 Hz streams kept current while Chrome renders
        while not stop.is_set():
            now = time.monotonic()
            m.on_message("local/usv/ui/fcu", json.dumps({"body": dict(FCU, servo=[1660, 1500, 1890, 1340], tail=[[1.0, "[page] points", "cmd"]],
                                                                      heading_deg=20, groundspeed_mps=0.4)}).encode(), now=now)
            m.on_message(f"{NS}/status/usv", json.dumps({"body": {"state": "AUTO_TASK", "task_name": "T1", "task_state": "TRANSIT"}}).encode(), now=now)
            stop.wait(0.3)
    threading.Thread(target=feed, daemon=True).start()
    srv, url = _serve(m)
    try:
        for theme in ("light", "dark"):
            dom = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=8000", "--window-size=1440,900", "--dump-dom",
                                  url + "/?theme=" + theme], capture_output=True, text=True, timeout=90).stdout
            L = _layout(dom)
            assert L, theme
            # v0.1.74 (N6): row 1 = the plate's four boxes + the numerals; LOG / CV FOLD in the tool row above the wireframe; SIMULATION
            # leads row 2 (the context line) - everything still above the fold at 1440 x 900, one row / one height, no overlap
            for k in ("n6r1", "pl-vehicle", "pl-mode", "pl-arm", "pl-holder", "pl-log", "pl-cv", "n6sog", "p-ev", "p-con"):
                x, y, w, h = L[k]
                assert w > 0 and h > 0 and y + h <= 900 and x + w <= 1440, (theme, k, L[k])
            boxes = ["pl-vehicle", "pl-mode", "pl-arm", "pl-holder", "pl-log", "pl-cv", "n6sog"]
            assert len({L[k][1] for k in boxes[:4]}) == 1 and len({L[k][3] for k in boxes[:4]}) == 1, theme       # one row, one height
            for i, a in enumerate(boxes + ["p-ev", "p-con", "wf"]):
                for b in (boxes + ["p-ev", "p-con", "wf"])[i + 1:]:
                    assert not _hit(L[a], L[b]), (theme, a, b)
            assert L["n6r1"][1] < L["sim"][1]                                                         # the plate first, the context line under it
            body = re.sub(r"<script\b.*?</script>", "", dom, flags=re.S)
            assert re.search(r'<div id="sim"(?![^>]*hidden)', body) and "FCU link fake" in body
            assert re.search(r'id="pl-arm" [^>]*class="pb arm armed"|class="pb arm armed"[^>]*id="pl-arm"', body)
            assert re.search(r'class="pb big auto"[^>]*id="pl-vehicle"', body) and 'data-cls="unk"' in body             # LOG / CV not published here
            assert len(re.findall(r'class="evr ', body)) >= 20 and '<b>AUTO_TASK</b>' in body and '[page] points' in body
            groups = re.findall(r'<g id="co-([a-z-]+)" data-x="([\d.]+)" data-y="([\d.]+)" data-w="([\d.]+)" data-h="([\d.]+)"[^>]*data-fit="(\d)"', body)
            assert len(groups) == 9 and all(g[5] == "1" for g in groups)
            assert re.search(r'<canvas id="lmap"[^>]*data-state="unknown"', body) or re.search(r'<canvas id="lmap"[^>]*data-items="0"', body)
            buttons = re.findall(r"<button\b[^>]*>", body)
            assert [x for x in buttons if "title=" not in x] == []
    finally:
        stop.set()
        srv.shutdown()
