"""v0.1.47 (ROADMAP WP10k-a, docs/UI_QOL_2026-11-07.md A + B): the launcher (copy buttons, titled choices, the manual network's
placeholders, PROGRESS per process / page, the five links with their dots, the rc-test judge page, the theme as the source of truth) and
the judge page (tokens only, three columns + the strip, the judge map ?judge=1, the liveness dot, --up in every theme)."""
from __future__ import annotations

import html
import json
import os
import re
import subprocess
import threading
import time
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler

import pytest

from optiprimex.tools import judge_web as J
from optiprimex.tools import launcher as L
from optiprimex.tools import prefs as PF
from optiprimex.tools.rc_tui import RcModel

from .conftest import ROOT, free_port
from .test_v0143 import THEMES, _cr, _lstar

TOOLS = ROOT / "optiprimex" / "tools"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
needs_chrome = pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
TEAM = "SGIT"


# ---- helpers: an OCS page, a judge page, a launcher - each in-process on a free port -------------------------------------------
def _serve(srv):
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.1}, daemon=True).start()
    return srv


def _ocs(tmp_path, prof=None):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())] + (["--ocs-config", str(prof)] if prof else []))
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    return web, _serve(OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))), a.web_port


def _judge(broker=("127.0.0.1", 1885), model=None):
    model = model or RcModel(TEAM)
    srv = _serve(J.Server(("127.0.0.1", free_port()), J.make_handler(model, None, broker=broker)))
    return model, srv, srv.server_address[1]


def _launcher(tmp_path, ocs_port, ls=lambda s: [], lw=None, kit=None):
    runner = L.Runner(dry=True)
    runner.record_path = str(tmp_path / "last.json")
    prefs = L.ThemePrefs(str(tmp_path / "prefs.json"), runner.record_path, lambda: f"http://127.0.0.1:{ocs_port}")
    watch = L.Watch(runner, ls=ls, lw=lw, prefs=prefs)
    srv = _serve(L.Server(("127.0.0.1", free_port()), L.make_handler(runner, watch, prefs, kit=str(kit or tmp_path))))
    return runner, watch, prefs, srv, srv.server_address[1]


def _get(u):
    with urllib.request.urlopen(u, timeout=5) as r:
        return r.read()


def _post(u, body, headers):
    req = urllib.request.Request(u, data=json.dumps(body).encode(), method="POST", headers=dict({"Content-Type": "application/json"}, **headers))
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {"code": e.code}


def _ports(monkeypatch, ocs=None, judge=None):
    """Every page port the launcher / judge page probe points at a test server or a closed port (never the live 8094-8098)."""
    for var, p in (("OCS_WEB_PORT", ocs), ("JUDGE_WEB_PORT", judge), ("SIM_PORT", None), ("BOAT_WEB_PORT", None), ("LAUNCH_PORT", None)):
        monkeypatch.setenv(var, str(p or free_port()))
    monkeypatch.setenv("RCTEST_PORT", "1885")
    monkeypatch.setenv("HUB_WS", str(free_port()))              # the judge map's websocket: a closed port, never the live hub
    monkeypatch.delenv("LAUNCH_THEME", raising=False)


def _chrome(url, budget=8000):
    return subprocess.run([CHROME, "--headless=new", "--disable-gpu", f"--virtual-time-budget={budget}", "--dump-dom", url],
                          capture_output=True, text=True, timeout=120).stdout


def _feed(model):
    """A judge's view with something in it: the course, a declaration, two heartbeats, a keep-out and an assist (JSON bench wire)."""
    b = [[1.2810, 103.8545], [1.2810, 103.8556], [1.2817, 103.8556], [1.2817, 103.8545]]
    model.on_wire("rc/course", json.dumps({"course_id": "ALPHA", "boundary": b, "pinger_hz": 30}).encode())
    model.on_wire("rc/request", json.dumps({"team_id": TEAM, "request_seq": 1, "vehicle_ids": ["USV1"], "tiers": {"T1": "TIER_CORE"}}).encode())
    for i in range(2):
        model.on_wire(f"rc/report/{TEAM}/USV1", json.dumps({"kind": "heartbeat", "team_id": TEAM, "report_seq": i + 1, "state": "STATE_AUTO",
                                                           "current_task": "TASK_NONE", "lat": 1.2813, "lon": 103.8550, "heading_deg": 90}).encode())
    model.on_wire(f"rc/command/{TEAM}", json.dumps({"kind": "keep_out_zone", "team_id": TEAM, "command_seq": 3, "position": {"lat": 1.2815, "lon": 103.8552},
                                                     "radius_m": 8, "domain": "surface"}).encode())
    model.on_wire(f"rc/command/{TEAM}", json.dumps({"kind": "assistance_request", "team_id": TEAM, "command_seq": 4, "position": {"lat": 1.2812, "lon": 103.8548},
                                                     "domain": "surface"}).encode())
    return b


# ---- A1 / A2: copy buttons, titled choices -----------------------------------------------------------------------------------------
@needs_chrome
def test_every_option_button_is_titled_and_every_command_has_a_copy_button(tmp_path, monkeypatch):
    web, osrv, op = _ocs(tmp_path)
    _ports(monkeypatch, ocs=op)
    runner, watch, prefs, srv, lp = _launcher(tmp_path, op)
    try:
        dom = _chrome(f"http://127.0.0.1:{lp}/")
        body = re.sub(r"<script\b.*?</script>", "", dom, flags=re.S)
        seg = re.findall(r'<div class="seg" data-k="(\w+)"[^>]*>(.*?)</div>', body, flags=re.S)
        assert {k for k, _ in seg} >= {"mode", "judge", "world", "perception", "usv_ui", "fcu", "usv_profile", "cv", "net", "ui", "log"}
        opts = [b for _, inner in seg for b in re.findall(r"<button\b[^>]*>", inner)]
        assert len(opts) > 30 and all('title="' in b and 'title=""' not in b for b in opts), [b for b in opts if "title" not in b]
        assert 'title="rctest: RoboNation' in body and 'title="geometric: the fast' in body and 'title="lab: run_usv.sh --lab' in body
        buttons = re.findall(r"<button\b[^>]*>", body)
        assert [b for b in buttons if 'title="' not in b or 'title=""' in b] == []                # no button without a title (the page)
        plan = body[body.index('id="plan"'):body.index('id="launch"')]
        pres, cps = plan.count("<pre>"), plan.count('class="cp"')
        assert pres >= 2 and pres == cps, (pres, cps)                                              # the run_sim.sh step + the stop line
        assert "scripts/run_sim.sh pool --no-open --judge fake --perception camera" in plan
        assert 'class="chip hide">copied<' in plan
    finally:
        srv.shutdown(); osrv.shutdown()
    page = (TOOLS / "launcher.html").read_text()
    assert 'box.querySelector("pre").textContent' in page and "navigator.clipboard" in page and "setTimeout(()=>chip.classList.add(\"hide\"),1000)" in page


# ---- A3: the manual network's placeholders --------------------------------------------------------------------------------------
def test_the_manual_placeholders_come_from_the_env_files_or_the_format_example(tmp_path, monkeypatch):
    ph = L.net_placeholders(str(tmp_path))                                          # no files: the format examples
    assert ph == L.MANUAL_EXAMPLE and set(ph) == set(L.MANUAL_KEYS)
    (tmp_path / "network.ts.env").write_text("# made-up test addresses\nOCS_HOST=100.64.0.10\nBOAT_HOST=100.64.0.11\nORIN_SSH=nx-test-ts\n")
    (tmp_path / "network.lan.env").write_text("export OCS_HOST=192.168.50.7\nAGX_HOST=\"192.168.50.9\"  # agx\nRC_HOST=192.168.50.2\n")
    ph = L.net_placeholders(str(tmp_path))
    assert ph["ocs_host"] == "100.64.0.10 (ts) · 192.168.50.7 (lan)" and ph["boat_host"] == "100.64.0.11 (ts)"
    assert ph["agx_host"] == "192.168.50.9 (lan)" and ph["nx_ssh"] == "nx-test-ts (ts)" and ph["rc_host"] == "192.168.50.2 (course)"
    assert ph["agx_ssh"] == L.MANUAL_EXAMPLE["agx_ssh"]                               # in neither file
    web, osrv, op = _ocs(tmp_path)
    _ports(monkeypatch, ocs=op)
    runner, watch, prefs, srv, lp = _launcher(tmp_path, op, kit=tmp_path)
    try:
        d = json.loads(_get(f"http://127.0.0.1:{lp}/presets.json"))
        assert d["placeholders"]["ocs_host"] == "100.64.0.10 (ts) · 192.168.50.7 (lan)" and d["themes"] == list(THEMES)
    finally:
        srv.shutdown(); osrv.shutdown()
    assert "el.placeholder=PH[k]" in (TOOLS / "launcher.html").read_text()


# ---- A4: PROGRESS -------------------------------------------------------------------------------------------------------------------
def test_progress_states_from_the_probe():
    procs = L.plan_procs(dict(L.load_presets()["sim"]))
    keys = [p["key"] for p in procs]
    assert "mac:optiprimex:hub" in keys and "mac:optiprimex:sim" in keys and "page:SIM page" in keys and "mac:optiprimex:obs" in keys
    win = {"mac": ["optiprimex:hub:0:mosquitto", "optiprimex:mm:0:zsh", "optiprimex:fcu:1:Python", "optiprimex:sim:0:Python"]}
    seen = {}
    rows = {r["key"]: r for r in L.progress(procs, win, {"OCS page": True, "SIM page": False}, seen)}
    assert rows["mac:optiprimex:hub"]["state"] == "up" and "mosquitto" in rows["mac:optiprimex:hub"]["why"]
    assert rows["mac:optiprimex:mm"]["state"] == "died" and rows["mac:optiprimex:fcu"]["state"] == "died"       # back at the shell / dead pane
    assert rows["mac:optiprimex:fp"]["state"] == "starting" and rows["page:SIM page"]["state"] == "starting"
    assert rows["page:OCS page"]["state"] == "up"
    rows = {r["key"]: r for r in L.progress(procs, {"mac": ["optiprimex:sim:0:Python"]}, {"OCS page": False}, seen)}
    assert rows["mac:optiprimex:hub"]["state"] == "died" and rows["page:OCS page"]["state"] == "died"         # was up, now gone
    assert all(r["state"] == "unknown" for r in L.progress(procs, {"mac": None}, {}, {}) if r["kind"] == "tmux")
    real = L.plan_procs(dict(L.load_presets()["event"]), "nx-a", "agx-a")
    assert {("nx", "usv", "mm"), ("agx", "agx", "cv"), ("mac", "rctest", "server"), ("mac", "judge", "tui")} <= {(p["host"], p.get("session"), p.get("window")) for p in real}
    assert all(p["ssh"] == "nx-a" for p in real if p["host"] == "nx")
    assert L.build_plan(dict(L.load_presets()["lab"]), stamp="S", net_env={})["procs"]


@needs_chrome
def test_progress_fills_in_from_the_watch_and_shows_on_the_page(tmp_path, monkeypatch):
    web, osrv, op = _ocs(tmp_path)
    _ports(monkeypatch, ocs=op)
    windows = {"now": ["optiprimex:hub:0:mosquitto", "optiprimex:mm:0:zsh"]}
    runner, watch, prefs, srv, lp = _launcher(tmp_path, op, ls=lambda s: ["optiprimex"], lw=lambda s: windows["now"])
    try:
        cfg = dict(L.load_presets()["sim"], preset="sim")
        L.save_record(cfg, L.build_plan(cfg, stamp="S"), runner.record_path)
        d = watch.refresh()
        st = {p["key"]: p["state"] for p in d["progress"]}
        assert st["mac:optiprimex:hub"] == "up" and st["mac:optiprimex:mm"] == "died" and st["mac:optiprimex:sim"] == "starting"
        assert st["page:OCS page"] == "up" and st["page:judge page"] == "starting"
        windows["now"] = ["optiprimex:hub:0:mosquitto", "optiprimex:mm:0:Python", "optiprimex:sim:0:Python"]
        st = {p["key"]: p["state"] for p in watch.refresh()["progress"]}
        assert st["mac:optiprimex:mm"] == "up" and st["mac:optiprimex:sim"] == "up"
        dom = _chrome(f"http://127.0.0.1:{lp}/")
        assert dom.count('class="pl up"') >= 4 and 'class="pl starting"' in dom and "mission manager" in dom
    finally:
        srv.shutdown(); osrv.shutdown()
    css = (TOOLS / "launcher.html").read_text()
    assert ".pl.up{color:var(--up);font-weight:700}" in css and ".pl.died{color:var(--alarm" in css and "color:var(--dim)" in css


# ---- A5: the five links and their dots; the rc-test judge page --------------------------------------------------------------------
def test_the_five_links_show_up_and_down(tmp_path, monkeypatch):
    web, osrv, op = _ocs(tmp_path)
    model, jsrv, jp = _judge(("127.0.0.1", 1885))
    _ports(monkeypatch, ocs=op, judge=jp)
    pages = {p["name"]: p for p in L.probe_pages(L.page_links())}
    assert list(pages) == ["OCS page", "judge page", "SIM page", "USV window", "rc-test console"]
    assert pages["OCS page"]["up"] and pages["judge page"]["up"] and pages["rc-test console"]["up"]
    assert not pages["SIM page"]["up"] and not pages["USV window"]["up"] and "tmux attach -t judge" in pages["rc-test console"]["title"]
    st = json.loads(_get(f"http://127.0.0.1:{jp}/status.json"))
    assert st["broker_port"] == 1885 and st["page"] == "judge"
    jsrv.shutdown(); jsrv.server_close()
    model2, jsrv2, jp2 = _judge(("127.0.0.1", 1883))                                 # the fake judge's page: not rc-test's
    monkeypatch.setenv("JUDGE_WEB_PORT", str(jp2))
    try:
        pages = {p["name"]: p for p in L.probe_pages(L.page_links({"ui": "v1"}))}
        assert pages["judge page"]["up"] and not pages["rc-test console"]["up"] and "1883" in pages["rc-test console"]["why"]
        assert pages["OCS page"]["url"].endswith("/v1/")
    finally:
        jsrv2.shutdown(); jsrv2.server_close()
    osrv.shutdown(); osrv.server_close()                                               # after STOP: every dot down
    assert not any(p["up"] for p in L.probe_pages(L.page_links()))
    runner, watch, prefs, srv, lp = _launcher(tmp_path, op)
    try:
        d = json.loads(_get(f"http://127.0.0.1:{lp}/session.json"))                    # before any launch: the dots are there
        watch.refresh()
        d = json.loads(_get(f"http://127.0.0.1:{lp}/session.json"))
        assert [p["name"] for p in d["pages"]] == ["OCS page", "judge page", "SIM page", "USV window", "rc-test console"] and d["progress"] == []
    finally:
        srv.shutdown()
    page = (TOOLS / "launcher.html").read_text()
    assert ".plinks a.up i{background:var(--up)" in page and 'p.up?"up":"down"' in page and "border:1px solid var(--dim)" in page


def test_the_rctest_plan_starts_the_judge_page_wired_to_1885():
    p = L.build_plan(dict(L.load_presets()["event"]), stamp="S", net_env={"ORIN_SSH": "nx", "AGX_SSH": "agx"})
    (step,) = [s for s in p["steps"] if "run_rctest.sh" in s["cmd"]]
    assert "JUDGE_WEB=0" not in step["cmd"] and "JUDGE_OPEN=0 scripts/run_rctest.sh" in step["cmd"]
    assert "scripts/run_judge.sh --port ${RCTEST_PORT:-1885} --as-judge --owner rctest" in step["cmd"] and "/status.json" in step["cmd"]
    assert subprocess.run(["bash", "-n", "-c", step["cmd"]]).returncode == 0
    grep = step["cmd"].split("| ", 1)[1].split(" &&", 1)[0]                         # the check: the judge page answers wired to 1885
    for body, ok in (('{"ok": true, "page": "judge", "broker_host": "127.0.0.1", "broker_port": 1885, "mode": "judge"}', 0),
                     ('{"ok": true, "broker_port": 1883}', 1), ('{"ok": true, "broker_port": 18850}', 1)):
        assert subprocess.run(["bash", "-c", grep], input=body, text=True).returncode == ok, body
    rc = (ROOT / "scripts" / "run_rctest.sh").read_text()
    assert '"${JUDGE_OPEN:-${JUDGE_WEB:-1}}" = "1"' in rc and "always" in rc
    assert any(p_["key"] == "mac:judge:tui" for p_ in p["procs"])


# ---- A6: the theme round trip -------------------------------------------------------------------------------------------------------
def test_the_theme_round_trip_launcher_ocs_judge_and_back(tmp_path, monkeypatch):
    web, osrv, op = _ocs(tmp_path)
    model, jsrv, jp = _judge()
    _ports(monkeypatch, ocs=op, judge=jp)
    runner, watch, prefs, srv, lp = _launcher(tmp_path, op)
    monkeypatch.setenv("LAUNCH_PORT", str(lp))
    LU, OU, JU = (f"http://127.0.0.1:{p}" for p in (lp, op, jp))
    XL, XJ = {"X-Launch": "1"}, {"X-Judge-Web": "1"}
    try:
        assert json.loads(_get(OU + "/prefs")) == {"ok": True, "prefs": {}}                                       # the new GET
        assert json.loads(_get(LU + "/prefs"))["prefs"]["theme"] == "dark"
        assert json.loads(_get(OU + "/prefs"))["prefs"] == {"theme": "dark"}                                       # first contact: pushed
        assert _post(LU + "/prefs", {"theme": "hc-light"}, {}) == {"code": 403}
        assert _post(LU + "/prefs", {"theme": "hc-light"}, XL)["ok"]
        assert json.loads(_get(OU + "/prefs"))["prefs"]["theme"] == "hc-light"                                     # launcher -> OCS
        assert json.loads(_get(JU + "/prefs")) == {"ok": True, "prefs": {"theme": "hc-light"}, "from": "ocs"}       # -> the judge page
        assert json.loads(_get(OU + "/state.json"))["bar"]["theme"] == "hc-light"
        assert _post(JU + "/prefs", {"theme": "fun"}, {}).get("code") == 403
        assert _post(JU + "/prefs", {"theme": "fun"}, XJ) == {"ok": True, "prefs": {"theme": "fun"}, "to": "ocs"}   # the judge's selector
        assert json.loads(_get(LU + "/prefs"))["prefs"]["theme"] == "fun"                                          # ... and back
        assert json.loads((tmp_path / "prefs.json").read_text())["theme"] == "fun"
        assert not _post(LU + "/prefs", {"theme": "neon"}, XL)["ok"]
        cfg = dict(L.load_presets()["sim"], theme="light")
        assert L.validate(cfg) == [] and L.validate(dict(cfg, theme="neon"))
        assert "LAUNCH_THEME=light scripts/run_sim.sh" in L.build_plan(cfg, stamp="S")["steps"][0]["cmd"]
        real = L.build_plan(dict(L.load_presets()["lab"], theme="hc-dark"), stamp="S", net_env={})
        assert any("LAUNCH_THEME=hc-dark scripts/run_ocs.sh" in s["cmd"] for s in real["steps"])
        assert _post(LU + "/launch", {"cfg": cfg}, XL)["ok"]                                                         # the launch sets it
        assert json.loads(_get(OU + "/prefs"))["prefs"]["theme"] == "light"
        osrv.shutdown(); osrv.server_close()                                                                        # the OCS down: the launcher's
        assert json.loads(_get(JU + "/prefs")) == {"ok": True, "prefs": {"theme": "light"}, "from": "launcher"}
        assert _post(JU + "/prefs", {"theme": "hc-dark"}, XJ)["to"] == "launcher" and prefs.theme == "hc-dark"
        web2, osrv2, op2 = _ocs(tmp_path)                                                                            # a relaunched OCS: ours holds
        prefs.ocs = lambda: f"http://127.0.0.1:{op2}"
        assert json.loads(_get(LU + "/prefs"))["prefs"]["theme"] == "hc-dark" and web2.prefs == {"theme": "hc-dark"}
        osrv2.shutdown()
    finally:
        srv.shutdown(); jsrv.shutdown()
    monkeypatch.setenv("LAUNCH_THEME", "fun")
    assert L.ThemePrefs(str(tmp_path / "x.json"), str(tmp_path / "none.json"), None).theme == "fun"
    monkeypatch.delenv("LAUNCH_THEME")
    L.save_record({"theme": "hc-light"}, {}, str(tmp_path / "r" / "last.json"))
    assert L.ThemePrefs(str(tmp_path / "x.json"), str(tmp_path / "r" / "last.json"), None).theme == "hc-light"   # the record
    assert PF.norm("contrast") == "hc-dark" and PF.norm("neon") == ""


@needs_chrome
def test_the_pages_read_the_shared_theme_and_the_select_sits_right_most(tmp_path, monkeypatch):
    web, osrv, op = _ocs(tmp_path)
    model, jsrv, jp = _judge()
    _ports(monkeypatch, ocs=op, judge=jp)
    runner, watch, prefs, srv, lp = _launcher(tmp_path, op)
    try:
        web.prefs["theme"] = "hc-light"
        dom = _chrome(f"http://127.0.0.1:{jp}/")
        assert re.search(r'<html[^>]*data-theme="hc-light"', dom), dom[:300]
        prefs.set("fun")
        assert web.prefs["theme"] == "fun"
        dom = _chrome(f"http://127.0.0.1:{lp}/")
        assert re.search(r'<html[^>]*data-theme="fun"', dom)
    finally:
        srv.shutdown(); jsrv.shutdown(); osrv.shutdown()
    for page in (J.PAGE, (TOOLS / "launcher.html").read_text()):
        bar = page[page.index('<div id="bar"'):page.index("</div>", page.index('<div id="bar"'))] if 'id="bar"' in page else page[page.index("<header>"):page.index("</header>")]
        last = re.findall(r"<(select|button|input)\b[^>]*>", bar)[-1]
        assert last == "select" and bar.rstrip().endswith("</select>"), bar[-200:]
        assert "localStorage.setItem" in page and "/prefs" in page
    assert 'fetch(\'/prefs\',{method:\'POST\',headers:{\'Content-Type\':\'application/json\',\'X-Judge-Web\':\'1\'}' in J.PAGE


# ---- B7 / B8: the judge page: tokens, the columns, the judge map -----------------------------------------------------------------
def test_the_judge_page_has_no_hard_coded_fills_and_three_columns():
    p = J.PAGE
    style = p[p.index("<style>"):p.index("</style>")]
    assert re.findall(r"#[0-9a-fA-F]{3,8}\b", style) == []
    for dark in ("#11161d", "#0d1117", "#1c2330", "#111823", "#1c232c", "#238636", "#132a1c", "#1f3a5f", "#3b2a12"):
        assert dark not in p, dark
    assert 'href="/themes.css"' in p
    strip = p[p.index('<div class="strip">'):p.index("<main>")]
    assert [h for h in re.findall(r"<h2>([^<]+)</h2>", strip)] == ["DECLARATION", "COURSE", "RUN"]
    assert all(f'id="{i}"' in strip for i in ("decl", "course", "run"))
    main = p[p.index("<main>"):p.index("</main>")]
    heads = re.findall(r"<section[^>]*><h2>([A-Z ]+)", main)
    assert [h.strip() for h in heads] == ["JUDGE ACTIONS", "JUDGE RECEIVES", "JUDGE SENDS"]
    assert "grid-template-columns:minmax(250px,.85fr) minmax(0,1.3fr) minmax(0,1.3fr)" in style
    assert '<iframe id="jmapf" src="/map.html?judge=1"' in p and 'id="place"' in p and '<script src="/live_dot.js"></script>' in p


def test_the_judge_server_carries_incidents_liveness_and_proxies_the_map(tmp_path, monkeypatch):
    prof = tmp_path / "ocs.yaml"
    prof.write_text((ROOT / "profiles" / "bench" / "ocs_bench.yaml").read_text())
    web, osrv, op = _ocs(tmp_path, prof)
    model, jsrv, jp = _judge()
    _ports(monkeypatch, ocs=op, judge=jp)
    JU = f"http://127.0.0.1:{jp}"
    try:
        _feed(model)
        model.on_wire(f"rc/command/{TEAM}", json.dumps({"kind": "all_clear", "team_id": TEAM, "command_seq": 5, "domain": "surface"}).encode())
        model.on_wire(f"rc/command/{TEAM}", json.dumps({"kind": "moving_object", "team_id": TEAM, "command_seq": 6, "position": {"lat": 1.2814, "lon": 103.8551},
                                                         "heading_deg": 45, "speed_mps": 0.5, "domain": "surface"}).encode())
        s = json.loads(_get(JU + "/state.json"))
        inc = {i["kind"]: i for i in s["incidents"]}
        assert inc["keepout"]["r"] == 8 and inc["keepout"]["cleared"] and inc["assist"]["cleared"] and inc["moving"]["hdg"] == 45
        assert inc["assist"]["lat"] == pytest.approx(1.2812) and not inc["moving"]["cleared"]
        live = s["live"]["USV1"]
        assert len(live["times"]) == 2 and live["limit_s"] == J.HB_LIMIT_S and "heartbeats" in live["topic"]
        assert b"map.html" in _get(JU + "/map.html")[:400] or b"USV map" in _get(JU + "/map.html")
        assert b"LiveDot" in _get(JU + "/live_dot.js")
        c = json.loads(_get(JU + "/course.json"))                                                 # proxied: the OCS profile's course
        assert c["default"]["course_id"] == "ALPHA (default)" and c["geofence"] == []
        fence = [[1.2810, 103.8545], [1.2810, 103.8556], [1.2817, 103.8550]]
        assert _post(JU + "/geofence", {"points": fence}, {}).get("code") == 403
        r = _post(JU + "/geofence", {"points": fence}, {"X-OCS-Web": "1"})                    # the map's editor (its own header)
        assert r["ok"] and r["corners"] == 3 and json.loads(_get(JU + "/course.json"))["geofence"] == fence
        assert prof.read_text().count("\ngeofence:") == 1
        assert _post(JU + "/cmd", {"line": "allclear"}, {"X-OCS-Web": "1"}).get("code") == 403   # the map's other POSTs never reach /cmd
    finally:
        jsrv.shutdown(); osrv.shutdown()


class _Static(BaseHTTPRequestHandler):
    """A harness server: the tools' files, a judge state and a course stub, and one harness page."""
    files: dict = {}

    def log_message(self, *a):
        pass

    def do_GET(self):
        path = self.path.split("?")[0]
        body, ct = self.files.get(path, (None, None))
        if body is None:
            self.send_response(404); self.end_headers(); return
        self.send_response(200)
        self.send_header("Content-Type", ct)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def _harness(files):
    H = type("H", (_Static,), {"files": files})
    return _serve(L.Server(("127.0.0.1", free_port()), H))


MAP_HARNESS = """<!doctype html><html><body><iframe id="f" src="/map.html?judge=1&port=%d" style="width:800px;height:600px"></iframe><div id="out">waiting</div>
<script>let got=null;window.addEventListener('message',e=>{if(e.data&&e.data.type==='judge-pos')got=e.data});
setTimeout(()=>{const w=document.getElementById('f').contentWindow,d=w.document,c=d.getElementById('c'),r=c.getBoundingClientRect(),x=r.left+r.width/2,y=r.top+r.height/2;
 c.dispatchEvent(new w.MouseEvent('mousedown',{clientX:x,clientY:y,bubbles:true}));c.dispatchEvent(new w.MouseEvent('mouseup',{clientX:x,clientY:y,bubbles:true}));
 setTimeout(()=>{document.getElementById('out').textContent=JSON.stringify({got,judge:d.body.classList.contains('judge'),fence:d.getElementById('fencebar').style.display,
  aside:w.getComputedStyle(d.querySelector('aside')).display,add:w.getComputedStyle(d.getElementById('b_add')).display,src:Object.keys(w.eval('maps'))})},400)},3000);
</script></body></html>"""


@needs_chrome
def test_the_judge_map_places_the_position_of_the_next_command():
    b = [[1.0000, 103.0000], [1.0000, 103.0010], [1.0010, 103.0010], [1.0010, 103.0000]]
    st = {"vehicles": {"USV1": {"heartbeat": {"lat": 1.0004, "lon": 103.0004, "state": "STATE_AUTO", "heading_deg": 90}, "age_s": 0.4}},
          "incidents": [{"kind": "keepout", "seq": 3, "lat": 1.0006, "lon": 103.0006, "r": 10, "cleared": False}], "course": None}
    crs = {"default": {"course_id": "X", "boundary": b}, "received": None, "geofence": [], "profile": "p", "scene": None}
    srv = _harness({"/map.html": ((TOOLS / "map.html").read_bytes(), "text/html"), "/themes.css": ((TOOLS / "themes.css").read_bytes(), "text/css"),
                    "/state.json": (json.dumps(st).encode(), "application/json"), "/course.json": (json.dumps(crs).encode(), "application/json"),
                    "/h.html": ((MAP_HARNESS % free_port()).encode(), "text/html")})
    try:
        dom = _chrome(f"http://127.0.0.1:{srv.server_address[1]}/h.html", budget=9000)
    finally:
        srv.shutdown()
    out = json.loads(html.unescape(re.search(r'<div id="out">(.*?)</div>', dom, flags=re.S).group(1)))
    assert out["judge"] and out["fence"] == "" and out["aside"] == "none" and out["add"] == "none" and out["src"] == ["judge"]
    assert out["got"]["type"] == "judge-pos"
    assert out["got"]["lat"] == pytest.approx(1.0005, abs=2e-5) and out["got"]["lon"] == pytest.approx(103.0005, abs=2e-5)   # the canvas centre
    p = J.PAGE
    for needle in ("d.type!=='judge-pos'", "$('a-'+w+'-lat').value=Number(d.lat).toFixed(7)", "type:'judge-marks'", "e.source!==$('jmapf').contentWindow"):
        assert needle in p, needle
    mp = (TOOLS / "map.html").read_text()
    assert "if (JUDGE) drawJudge(m);" in mp and "type: 'judge-pos'" in mp and "OCSPOST('/geofence'" in mp


# ---- B9: the liveness dot -------------------------------------------------------------------------------------------------------
DOT_HARNESS = """<!doctype html><html data-theme="dark"><head><link rel="stylesheet" href="/themes.css"></head><body><div id="out">waiting</div>
<script src="/live_dot.js"></script><script>
const d=LiveDot.create('t');document.body.appendChild(d.el);const R={};const lit=()=>d.el.classList.contains('lit');
d.feed([100.0],100.2,2);R.first=lit();R.age=d.el.textContent;
d.feed([100.0,100.5,101.0],101.1,2);R.lit0=lit();
setTimeout(()=>{R.lit100=lit()},100);setTimeout(()=>{R.lit300=lit()},300);setTimeout(()=>{R.lit600=lit()},600);
setTimeout(()=>{R.lit850=lit();R.idle=getComputedStyle(d.el.querySelector('i')).backgroundColor;R.stale0=d.el.classList.contains('stale');
  d.feed([101.0],103.5,2);R.stale=d.el.classList.contains('stale');R.txt=d.el.textContent;R.flashes=d.state.flashes;
  R.col=getComputedStyle(d.el.querySelector('i')).backgroundColor;R.at=LiveDot.ageText(null,2);R.ms=LiveDot.LIGHT_MS;
  document.getElementById('out').textContent=JSON.stringify(R)},850);
</script></body></html>"""


@needs_chrome
def test_the_liveness_dot_lights_200_ms_per_message_and_alarms_past_the_limit():
    srv = _harness({"/live_dot.js": ((TOOLS / "live_dot.js").read_bytes(), "application/javascript"),
                    "/themes.css": ((TOOLS / "themes.css").read_bytes(), "text/css"), "/h.html": (DOT_HARNESS.encode(), "text/html; charset=utf-8")})
    try:
        dom = _chrome(f"http://127.0.0.1:{srv.server_address[1]}/h.html")
    finally:
        srv.shutdown()
    R = json.loads(html.unescape(re.search(r'<div id="out">(.*?)</div>', dom, flags=re.S).group(1)))
    assert R["ms"] == 200 and R["first"] is False and R["age"] == "0.2\u00a0s"              # the first feed: taken in, no replay
    assert R["lit0"] and R["lit100"] and not R["lit300"]                                 # 200 ms on the message ...
    assert R["lit600"] and not R["lit850"] and R["flashes"] == 2                         # ... and again with the next one (500 ms later)
    assert not R["stale0"] and R["idle"] == "rgba(0, 0, 0, 0)"                            # between messages: nothing lit, nothing moves
    assert R["stale"] and R["txt"] == "stale 2.5\u00a0s" and R["col"] == "rgb(249, 77, 78)"   # past the limit: --alarm + the word
    assert R["at"] == {"text": "no message", "stale": True}
    js = (TOOLS / "live_dot.js").read_text()
    assert "transition" not in js and "animation" not in js and "var(--up" in js and "var(--alarm" in js
    assert "DOTS[v]||(DOTS[v]=LiveDot.create(" in J.PAGE and "d.feed(L.times,s.now,L.limit_s)" in J.PAGE


# ---- the token -----------------------------------------------------------------------------------------------------------------------
def test_up_passes_the_contrast_rule_in_every_theme():
    css = (TOOLS / "themes.css").read_text()
    found = {}
    for name, body in re.findall(r'\[data-theme="([^"]+)"\]\{([^}]*)\}', css):     # a theme may have several blocks (--action, --btnBg)
        found[name] = found.get(name, "") + ";" + body
    assert set(found) == set(THEMES)
    for name, body in found.items():
        v = dict(re.findall(r"--(\w+):(#[0-9a-f]{6})", body))
        assert _cr(v["up"], v["panel"]) >= 3.0 and _cr(v["up"], v["bg"]) >= 3.0, name
        for k in ("warn", "alarm", "acc"):
            assert abs(_lstar(v["up"]) - _lstar(v[k])) >= 8.0, (name, k, _lstar(v["up"]), _lstar(v[k]))
