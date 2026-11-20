"""v0.1.43 (ROADMAP WP10i-lite, OCS_V2_SPEC §3-§8, §11, §13): the v1 freeze at /v1/, the global bar (the state word + why, the FCU,
LINK LOST), the RunStart countdown (cancel_auto / hold_on_start), the stage list from the playbook + the campaign's typical times, the
theme pack and its contrast rule, the safety rows of the fault-injection test, the launcher's ui field, the fake judge's refusal switch."""
from __future__ import annotations

import json
import re
import subprocess
import threading
import time
import urllib.request

import pytest

from .conftest import ROOT, free_port

TOOLS = ROOT / "optiprimex" / "tools"
THEMES = ("dark", "light", "hc-dark", "hc-light", "fun")


def _lum(h):
    r, g, b = [int(h[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    f = lambda c: c / 12.92 if c <= 0.03928 else ((c + 0.055) / 1.055) ** 2.4
    return 0.2126 * f(r) + 0.7152 * f(g) + 0.0722 * f(b)


def _cr(a, b):
    la, lb = sorted([_lum(a), _lum(b)], reverse=True)
    return (la + 0.05) / (lb + 0.05)


def _lstar(h):
    y = _lum(h)
    return 116 * (y ** (1 / 3) if y > 0.008856 else 7.787 * y + 16 / 116) - 16


def test_every_theme_passes_the_contrast_rule():
    css = (TOOLS / "themes.css").read_text()
    found = {}
    for name, body in re.findall(r'\[data-theme="([^"]+)"\]\{([^}]*)\}', css):     # v0.1.47: a theme may have several blocks (--action, --btnBg)
        found[name] = found.get(name, "") + ";" + body
    assert set(found) == set(THEMES)
    for name, body in found.items():
        v = dict(re.findall(r"--(\w+):(#[0-9a-f]{6})", body))
        assert min(_cr(v["text"], v["bg"]), _cr(v["text"], v["panel"])) >= 4.5, name               # body text 4.5:1
        assert _cr(v["chipText"], v["chipBg"]) >= 3.0, name                                         # chips 3:1
        for k in ("warn", "alarm", "acc"):
            assert _cr(v[k], v["panel"]) >= 3.0, (name, k)
        for k, bg in (("warnText", "warnBg"), ("alarmText", "alarmBg")):
            assert _cr(v[k], v[bg]) >= 4.5, (name, k)
        ls = sorted(_lstar(v[k]) for k in ("warn", "alarm", "acc"))
        assert min(ls[1] - ls[0], ls[2] - ls[1]) >= 8.0, (name, ls)                                # differ in lightness, not hue only
    page = (TOOLS / "ocs_web.html").read_text()
    for t in THEMES:
        assert f'<option value="{t}">' in page
    assert 'PRESET_THEME={sim:"dark",lab:"light",event:"hc-light"}' in page and '<link rel="stylesheet" href="/themes.css">' in page
    assert 'href="/themes.css"' in (TOOLS / "map.html").read_text()


def _web(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return web, srv, f"http://127.0.0.1:{a.web_port}"


def _get(u):
    with urllib.request.urlopen(u, timeout=5) as r:
        return r.read().decode()


def _post(u, body):
    req = urllib.request.Request(u, data=json.dumps(body).encode(), method="POST", headers={"Content-Type": "application/json", "X-OCS-Web": "1"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def test_the_v1_page_is_frozen_and_served_beside_v2(tmp_path):
    v1 = (TOOLS / "ocs_web_v1.html").read_text()
    assert "classic · v0.1.42 page frozen 2026-11-06" in v1 and 'id="tw"' not in v1
    web, srv, u = _web(tmp_path)
    try:
        assert _get(u + "/v1/") == v1 and 'id="tw"' in _get(u + "/")
        assert _get(u + "/v1/map.html") == _get(u + "/map.html") and "--alarm" in _get(u + "/v1/themes.css")
        st = json.loads(_get(u + "/v1/state.json"))
        assert "bar" in st and st["bar"]["word"]
        assert _post(u + "/v1/prefs", {"theme": "hc-light"})["prefs"] == {"theme": "hc-light"}         # POSTs from /v1/ reach the same server
        assert json.loads(_get(u + "/state.json"))["bar"]["theme"] == "hc-light"
    finally:
        srv.shutdown()
    page = (TOOLS / "ocs_web.html").read_text()
    assert 'id="v1open"' in page and 'id="v1pref"' in page and 'localStorage.getItem("ocs.ui")==="v1"' in page
    h = subprocess.run(["bash", str(ROOT / "scripts" / "run_ocs.sh"), "--help"], capture_output=True, text=True, timeout=10).stdout
    assert "--ui v1 | v2" in h


def test_the_bar_and_the_safety_rows():
    from optiprimex.tools.ocs_web import safety_rows, top_bar
    st = {"state": "AUTO_HOLD", "mode": "HOLD", "armed": True, "kill": False, "health": {"rc_ok": True, "fcu_age_s": 0.1}}
    snap = {"status": st, "status_age": 0.4, "ocs": {"r": "R5", "run_id": 1001}, "control": {"word": "ARMED", "why": "AUTO_HOLD"},
            "ui_active": False, "warnings": [], "perception_by": {"sim_observer": 0.5, "fake_perception": 0.3}}
    safety_rows(snap)
    b = top_bar(snap)
    assert snap["warnings"] == [] and b["word"] == "ARMED" and b["holder"] == "mission manager" and b["r"] == "R5" and not b["override"]
    s2 = json.loads(json.dumps(snap)); s2["status"]["health"]["rc_ok"] = False
    safety_rows(s2)
    assert s2["warnings"][0]["style"] == "bad" and "RC LOST" in s2["warnings"][0]["text"] and "transmitter" in s2["warnings"][0]["text"]
    s3 = json.loads(json.dumps(snap)); s3["status"]["health"]["fcu_age_s"] = 2.6
    safety_rows(s3)
    assert "FCU LINK LOST" in s3["warnings"][0]["text"] and "takeover" in s3["warnings"][0]["text"]
    s4 = json.loads(json.dumps(snap)); s4["status_age"] = 4.2
    safety_rows(s4)
    b4 = top_bar(s4)
    assert b4["word"] == "LINK LOST" and b4["override"] and "radio" in b4["hint"] and "BOAT LINK LOST" in s4["warnings"][0]["text"]
    assert b4["alarms"] == 1
    s5 = json.loads(json.dumps(snap)); s5["perception_by"]["sim_observer"] = 2.6                    # one source silent, the fake still beats
    safety_rows(s5)
    assert any("PERCEPTION STALE: sim_observer" in w["text"] and "CAMERA" in w["text"] for w in s5["warnings"])
    s6 = json.loads(json.dumps(snap)); s6["status"]["kill"] = True
    assert top_bar(s6)["kill"] is True and top_bar(dict(snap, ui_active=True))["holder"] == "usv_ui"


def test_the_runstart_countdown_cancel_and_hold_on_start():
    from optiprimex.companion.config import Config
    from optiprimex.ocs.ocs_app import OcsApp
    app = OcsApp(Config())
    sent = []
    app.authority.emit = lambda vid, kind, action, task=None, args=None, **kw: sent.append((action, task)) or "m"
    app.alert = lambda *a, **k: None
    app.last_status = {"USV1": {"state": "AUTO_HOLD"}}
    app.lifecycle.run_id = 1001
    assert app.countdown_s == 3.0
    app.set_mission("T1", True, {"tier": "core", "sim": True})
    app._mission_trigger("RunStart verified")
    assert sent == [] and app._mission_body()["countdown"]["s"] == 3.0                     # nothing yet: the 3-2-1 runs
    assert app.on_countdown("HOLD_ON_START") is None and app._mission_body()["countdown"]["hold_on_start"]
    app.mission["countdown"]["until"] = time.time() - 0.1
    app._countdown_tick()
    assert sent == [("START_TASK", "T1"), ("HOLD", None)] and app._mission_body()["countdown"] is None
    sent.clear()
    app.lifecycle.run_id = 1002
    app._mission_trigger("RunStart verified")
    assert app.on_countdown("CANCEL_AUTO") is None and not app.mission["auto"]
    app._countdown_tick()
    assert sent == [] and "no countdown" in app.on_countdown("CANCEL_AUTO")
    from .test_ocs_units import intent, make_authority
    bus, lc, auth, results = make_authority(bench=False, lc_state="COURSE_VALID")
    r = auth.on_intent("t", intent("CANCEL_AUTO"))
    assert r.result == "refused" and "no auto-start countdown" in r.reason
    from optiprimex.tools import ocs_cli
    for w in ("cancel_auto", "hold_on_start"):
        assert ocs_cli.intent_spec(ocs_cli.parse([w]))[0] == w.upper()
    for p in ("bench/ocs_bench.yaml", "pool/ocs_pool.yaml", "course/ocs_course.yaml"):
        assert "autostart_countdown_s: 3" in (ROOT / "profiles" / p).read_text()
    page = (TOOLS / "ocs_web.html").read_text()
    for needle in ('id="rscard"', 'id="cd"', 'id="cd-hold"', 'id="cd-cancel"', 'send("hold_on_start")', 'send("cancel_auto")',
                   "a refusal is silent: if nothing comes, ask the course manager"):   # v0.1.59 (B3)
        assert needle in page, needle


def test_the_stage_list_comes_from_the_playbook_and_the_campaign():
    from optiprimex.tools import playbook as P
    m1 = {s.name: s for s in P.MISSIONS["m1t1"].stages}
    assert m1["LOOK"].detour == "TRANSIT" and m1["APPROACH_LEAD"].detour == "SEARCH_ENTRY" and m1["TRANSIT"].leaves_on
    from optiprimex.usv.tasks.t1_safe_passage import STATES
    assert {"LOOK", "APPROACH_LEAD"} <= set(STATES)
    typ = json.loads((TOOLS / "stage_typical.json").read_text())["median_s"]
    for code in ("m1t1", "m3t1", "m4t0", "m0t4"):
        assert typ[code], code
    assert set(typ["m1t1"]) <= set(m1) and set(typ["m3t1"]) <= {s.name for s in P.MISSIONS["m3t1"].stages}
    from optiprimex.sim.campaign import stage_times, stage_typical
    assert stage_times([(0, 0, 0, 0, "A"), (0.5, 0, 0, 0, "A"), (1.0, 0, 0, 0, "B"), (3.0, 0, 0, 0, "B")]) == {"A": 1.0, "B": 2.0}
    assert stage_typical([{"ok": True, "mission": "m", "stage_s": {"A": 1}}, {"ok": True, "mission": "m", "stage_s": {"A": 3}},
                          {"ok": False, "mission": "m", "stage_s": {"A": 99}}]) == {"m": {"A": 3}}
    page = (TOOLS / "ocs_web.html").read_text()
    assert "stage_typical" in page and 'class="slist"' in page and "function drawStrip" in page and 'id="c-prog"' in page


def test_the_launcher_ui_field_and_the_judge_refusal_switch(monkeypatch):
    from optiprimex.tools import launcher as L
    P = L.load_presets()
    assert all(c["ui"] == "v2" for c in P.values())
    p = L.build_plan(dict(P["event"], ui="v1"), stamp="S", net_env={"ORIN_SSH": "nx", "AGX_SSH": "agx"})
    assert p["links"]["OCS page"].endswith("/v1/") and any("run_ocs.sh --lan --judge rctest --ui v1" in s["cmd"] for s in p["steps"])
    assert L.validate(dict(P["lab"], ui="v3")) and not L.validate(P["lab"])
    monkeypatch.setenv("LAUNCH_UI", "v1")
    assert all(c["ui"] == "v1" for c in L.load_presets().values())
    src = (ROOT / "optiprimex" / "sim" / "fake_robocommand.py").read_text()
    assert 'self.a.refuse = bool(body["refuse"])' in src and 'self.a.refuse = getattr(self, "_refuse0", False)' in src


def test_the_fault_timing_reads_what_the_operator_sees():
    from optiprimex.tools import fault_timing as F
    st = {"bar": {"word": "LINK LOST", "why": "status 4 s old", "hint": "check the radio", "holder": "mission manager", "v": "AUTO_HOLD"},
          "warnings": [{"style": "warn", "text": "x"}, {"style": "bad", "text": "FCU LINK LOST: takeover on the boat"}, {"style": "warn", "text": "y"}],
          "ocs": {"r": "R4"}}
    txt = " | ".join(F.visible(st))
    assert "LINK LOST" in txt and "FCU LINK LOST" in txt and " | y" not in txt and "ask the course manager" in txt   # v0.1.59 (B3)          # the 2 worst rows only
    for fault, (named, action) in F.EXPECT.items():
        assert re.compile(named) and re.compile(action), fault
    assert set(F.EXPECT) == {"kill", "rc6", "rc_lost", "gps", "fcu", "bridge", "refusal", "perception",
                             "uav_tel", "uav_fence", "uav_adapter"}                       # v0.1.65 (D18): the UAV's three, only with --uav
    from optiprimex.tools import sim_day
    assert "PAGE" in sim_day.ORDER and hasattr(sim_day.Walk, "PAGE")


CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


@pytest.mark.skipif(not __import__("os").path.exists(CHROME), reason="no Chrome")
def test_no_button_without_a_title_on_both_pages(tmp_path):
    web, srv, u = _web(tmp_path)
    try:
        for path in ("/", "/v1/"):
            dom = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=10000", "--dump-dom", u + path],
                                 capture_output=True, text=True, timeout=90).stdout
            dom = re.sub(r"<script\b.*?</script>", "", dom, flags=re.S)
            buttons = re.findall(r"<button\b[^>]*>", dom)
            assert len(buttons) > 40, (path, len(buttons))
            assert [b for b in buttons if "title=" not in b] == [], path
    finally:
        srv.shutdown()
