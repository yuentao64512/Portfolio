"""v0.1.45 (ROADMAP WP10f phases 1-2, BOAT_WINDOW_SPEC §5): usv_ui's FCU block at 2 Hz on local/usv/ui/fcu (nulls where the FCU says
nothing, never bridged), the whitelist listener on local/usv/ui/cmd (decision 25: handover / takeover from the page, a refused word
says why, the flood guard), boat-web (/, /state.json, /themes.css, the wireframe ids, the setState mapping, no button without a title)
and usv_ui still drawing at 200 x 60."""
from __future__ import annotations

import collections
import json
import os
import re
import socket
import subprocess
import threading
import time
import urllib.request

import pytest

from optiprimex.tools import boat_web as BW
from optiprimex.tools.vcmd import VehicleCommander

from .conftest import PY, ROOT, Probe, Proc, free_port, free_udp_port, make_usv_config, wait_until
from .test_usv_ui import UI_PY, Headless, load_usv_ui

NS = "team/optiprime/v1"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
SVG_IDS = ("thr-mp", "thr-ms", "thr-sf", "thr-sa", "cam", "lamp", "ant", "box", "cannon")


def _get(u):
    with urllib.request.urlopen(u, timeout=5) as r:
        return r.read().decode()


def _post(u, body, header=True):
    h = {"Content-Type": "application/json"}
    if header:
        h["X-Boat-Web"] = "1"
    req = urllib.request.Request(u, data=json.dumps(body).encode(), method="POST", headers=h)
    try:
        with urllib.request.urlopen(req, timeout=8) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def _serve(model, link=None):
    port = free_port()
    srv = BW.Server(("127.0.0.1", port), BW.make_handler(model, link, bar=None))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{port}"


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    if not UI_PY.exists():
        pytest.skip(f"{UI_PY} not found")
    tmp = tmp_path_factory.mktemp("bw")
    mod = load_usv_ui()
    share_port = free_udp_port()
    ui = Headless(mod, [f"udpin:127.0.0.1:{fake_fcu.mav_port}", "--share", f"udpout:127.0.0.1:{share_port}",
                         "--mqtt", f"127.0.0.1:{brokers.boat_port}", "--points", str(tmp / "points.json"), "--co", str(tmp / "co.json"),
                         "--log-dir", str(tmp / "logs")])
    ui.start()
    cfg = make_usv_config(tmp, brokers, fake_fcu)
    import yaml
    d = yaml.safe_load(cfg.read_text())
    d["mavlink"]["endpoint"] = f"udpin:127.0.0.1:{share_port}"
    cfg.write_text(yaml.safe_dump(d))
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(cfg)], tmp / "mm.log")
    hub = Probe(brokers.hub_port, [f"{NS}/#", "local/#"]).start()
    boat = Probe(brokers.boat_port, ["presence/#", "local/#", f"{NS}/status/#"]).start()
    vc = VehicleCommander(port=brokers.hub_port)
    vc.start()
    model = BW.Model("usv")
    link = BW.Link(model, "127.0.0.1", brokers.boat_port)
    srv, url = _serve(model, link)
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 20, include_past=True), mm.log()
    assert wait_until(lambda: model.connected and "fcu" in model.last and "status" in model.last, 10)
    yield {"ui": ui, "mm": mm, "hub": hub, "boat": boat, "vc": vc, "fcu": fake_fcu, "model": model, "link": link, "url": url, "tmp": tmp,
           "brokers": brokers}
    srv.shutdown(); link.close(); vc.stop(); hub.stop(); boat.stop(); mm.kill(); ui.stop()


def _pubs(port, *bodies):
    import paho.mqtt.client as mqtt
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"t-{free_port()}", protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", port)
    c.loop_start()
    for body in bodies:
        c.publish("local/usv/ui/cmd", json.dumps(body), qos=0).wait_for_publish(2)
    c.loop_stop()
    c.disconnect()


def _pub(port, word, **kw):
    _pubs(port, dict(word=word, **kw))


def _result(boat, cid, timeout=3):
    m = boat.wait(lambda m: m["topic"] == "local/usv/ui/cmd/result" and (m["env"].get("body") or {}).get("id") == cid, timeout, include_past=True)
    return m["env"]["body"] if m else None


def test_the_fcu_block_at_2hz_with_nulls_where_unknown(stack):
    boat, hub = stack["boat"], stack["hub"]
    boat.clear()
    time.sleep(3.2)
    ms = boat.all("local/usv/ui/fcu")
    assert 5 <= len(ms) <= 8, len(ms)                                                   # 2 Hz
    gaps = [b["t"] - a["t"] for a, b in zip(ms, ms[1:])]
    assert 0.35 < sum(gaps) / len(gaps) < 0.7, gaps
    env = ms[-1]["env"]
    assert env["kind"] == "local" and env["source"] == "usv_ui" and env["schema_version"] == 1           # usv_ui's envelope
    b = env["body"]
    for k in ("hb_age_s", "sysid", "compid", "mode", "armed", "kill", "estop", "rc", "servo", "ekf_flags", "voltage_v", "current_a",
              "heading_deg", "groundspeed_mps", "nav", "lat", "lon", "fcu_link", "active", "holder"):
        assert k in b, k
    assert b["hb_age_s"] is not None and b["hb_age_s"] < 2.0 and b["sysid"] == 1 and b["armed"] is True and b["fcu_link"] == "fake"
    assert len(b["servo"]) == 8 and b["servo"][0] == 1500 and len(b["rc"]) == 8
    assert b["kill"] is False and b["estop"] is False                                  # NAMED_VALUE_INT from the fake FCU
    assert b["voltage_v"] is not None and b["voltage_v"] > 5
    assert b["current_a"] is None                                                      # SYS_STATUS current -1 = not measured: null, never 0
    assert b["heading_deg"] is None and b["groundspeed_mps"] is None and b["nav"] is None      # the fake FCU sends no VFR_HUD / NAV_CONTROLLER_OUTPUT
    assert b["active"] is True and b["holder"] == "usv_ui" and b["ekf_pos_ok"] is True
    # local only: never on the hub
    assert not [m for m in hub.all() if "ui/fcu" in m["topic"]]


def test_handover_and_takeover_from_the_page(stack):
    ui, vc, hub, boat, url, model = stack["ui"], stack["vc"], stack["hub"], stack["boat"], stack["url"], stack["model"]
    app = ui.app
    assert app.active
    vc.declaration_seq = 1
    assert [a["body"]["result"] for a in vc.send("run", "PREPARE", args={"declaration_seq": 1, "tiers": {"T4": "TIER_CORE"}})][-1] == "DONE"
    time.sleep(0.6)
    code, r = _post(url + "/cmd", {"word": "handover"})
    assert code == 200 and r["ok"] and r["text"].startswith("HANDED OVER"), r
    assert not app.active and r["active"] is False
    assert any(t == "[page] handover" and kind == "cmd" for _, t, kind in app.log[-6:]), ui.lines()      # in usv_ui's console
    assert wait_until(lambda: model.snapshot()["holder"] == "mission manager", 3)
    time.sleep(2.5)                                                                     # tui_active_s: the beat has stopped
    acks = vc.send("run", "ENABLE_AUTO", wait_s=5)
    assert acks and acks[-1]["body"]["result"] == "DONE", (acks, stack["mm"].log()[-2000:])
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10, include_past=True)
    # takeover back within 2 s: the answer says active, the beat is on the boat broker again
    boat.clear()
    t0 = time.monotonic()
    code, r = _post(url + "/cmd", {"word": "takeover"})
    assert r["ok"] and r["active"] is True and app.active, r
    assert boat.wait(lambda m: m["topic"] == "local/usv/tui/active" and m["env"]["body"]["active"] is True, 2.0, include_past=True)
    assert time.monotonic() - t0 < 2.0
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 10, include_past=True)
    assert wait_until(lambda: model.snapshot()["holder"] == "usv_ui", 3)
    snap = model.snapshot()
    assert any("usv_ui: takeover" in e["text"] for e in snap["events"]) and snap["cmds"][0]["word"] == "takeover"


def test_refused_words_and_the_flood_guard(stack):
    ui, boat, url, port = stack["ui"], stack["boat"], stack["url"], stack["brokers"].boat_port
    app = ui.app
    time.sleep(0.6)
    # v0.1.50 (Dmitrii 2026-11-06): mode / arm / disarm became page words (tests/test_v0150.py); the rest stays refused
    for i, (word, args) in enumerate((("surge", ["0.3", "2"]), ("burst", ["1", "1", "1", "1"]),
                                      ("motor", ["1", "0.5"]), ("param", ["X", "1"]), ("yaw", ["0.2", "1"]))):
        cid = "r%d" % i
        _pub(port, word, args=args or [], id=cid)
        r = _result(boat, cid)
        assert r and r["ok"] is False and r["word"] == word and "not a page word" in r["text"] and "decision 25" in r["text"], (word, r)
    assert app.active and app.tel.armed                                                  # nothing happened on the FCU
    assert any(t.startswith("[page] burst") for _, t, _k in app.log)
    # the page refuses them too (defence in depth), with the reason
    code, r = _post(url + "/cmd", {"word": "burst"})
    assert code == 200 and not r["ok"] and "not a page word" in r["text"]
    code, r = _post(url + "/cmd", {"word": "handover"}, header=False)
    assert code == 403 and "X-Boat-Web" in r["text"]
    # a stale word (t older than 10 s) is refused
    time.sleep(0.6)
    _pub(port, "points", id="st", t=time.time() - 30)
    r = _result(boat, "st")
    assert r and not r["ok"] and "stale" in r["text"]
    # the flood guard: two accepted words 0.5 s apart at least
    time.sleep(0.6)
    _pubs(port, {"word": "points", "id": "f1"}, {"word": "points", "id": "f2"})
    r1, r2 = _result(boat, "f1"), _result(boat, "f2")
    assert r1["ok"] and not r2["ok"] and "too fast" in r2["text"], (r1, r2)
    # the other whitelisted words run as typed and answer with usv_ui's own line
    time.sleep(0.6)
    _pub(port, "note", args=["from", "the", "page"], id="n1")
    r = _result(boat, "n1")
    assert r["ok"] and r["text"].startswith("note: from the page"), r
    time.sleep(0.6)
    _pub(port, "log", id="l1")
    assert _result(boat, "l1")["text"].startswith("logging off")
    # a retained word is never replayed (at a reconnect the broker would hand it out with the retain flag)
    class Msg:
        topic, payload, retain = "local/usv/ui/cmd", json.dumps({"word": "points", "id": "ret"}).encode(), True
    n = len(app.mq.inbox)
    app.mq._on_message(None, None, Msg())
    assert len(app.mq.inbox) == n
    assert set(BW.WHITELIST) == set(ui.mod.App.PAGE_WORDS)


def test_boat_web_serves_the_page_the_state_and_the_themes(stack):
    url = stack["url"]
    page = _get(url + "/")
    for i in SVG_IDS:
        assert f'id="{i}"' in page, i
    assert '<link rel="stylesheet" href="/themes.css">' in page and "localStorage.getItem('boat.theme')" in page and "||'dark'" in page
    assert "window.setState=setState" in page and "FCU output (PWM)" in page
    for t in ("dark", "light", "hc-dark", "hc-light", "fun"):
        assert f'<option value="{t}">' in page
    for tab in ("CONTROL", "ACTUATION", "TELEMETRY", "NAV", "DIAG"):          # v0.1.49 (WP10k-b): the stubs have content now
        assert f'id="t-{tab}" class="grid" hidden' in page and "phase 3–4" not in page
    sim = (ROOT / "scripts" / "run_sim.sh").read_text()
    assert "-n bweb" in sim and "optiprimex.tools.boat_web --boat-host 127.0.0.1 --boat-port 1884 --port $BWEB_PORT" in sim and 'echo "boat window: http' in sim
    assert "local/usv/ui" not in (ROOT / "profiles" / "bench" / "mosquitto" / "bridge.conf").read_text()           # never bridged
    assert _get(url + "/themes.css") == (ROOT / "optiprimex" / "tools" / "themes.css").read_text()
    s = json.loads(_get(url + "/state.json"))
    assert s["connected"] and set(SVG_IDS) - {"thr-mp", "thr-ms", "thr-sf", "thr-sa"} <= set(s["wf"]) and len(s["hdr"]) == 8
    assert any("FCU link fake" in c["text"] for c in s["hdr"]) and any("held by" in c["text"] for c in s["hdr"])
    ctl = dict((k, v) for k, v in s["panels"]["ctl"])
    assert ctl["realized force"] == {"u": "not measured"}
    assert s["wf"]["lamp"]["state"] == "expected" and s["wf"]["lamp"]["colour"] in ("YELLOW", "GREEN")
    assert all(o["state"] in ("available", "active", "warning") for o in s["panels"]["act"])


def _model_with(fcu=None, status=None, now=100.0, **kw):
    m = BW.Model("usv", **kw)
    m.connected = True
    if fcu is not None:
        m.on_message("local/usv/ui/fcu", json.dumps({"body": fcu}).encode(), now=now)
    if status is not None:
        m.on_message(f"{NS}/status/usv", json.dumps({"body": status}).encode(), now=now)
    return m


def test_set_state_mapping():
    fcu = {"hb_age_s": 0.3, "servo": [1660, 1500, 1890, 1340, 1500, 1500, 1500, 1500], "active": False, "fcu_link": "fake", "mode": "HOLD"}
    m = _model_with(fcu, {"state": "AUTO_HOLD", "health": {"fcu_age_s": 0.2}})
    wf = m.snapshot(now=100.2)["wf"]
    assert wf["thr-mp"]["state"] == "active" and wf["thr-mp"]["frac"] == 0.4 and wf["thr-mp"]["dir"] == 1 and wf["thr-mp"]["pwm"] == 1660
    assert wf["thr-ms"]["state"] == "warning" and wf["thr-ms"]["frac"] > 0.9 and "SAT" in wf["thr-ms"]["label"]       # amber > 90 %
    assert wf["thr-sf"]["dir"] == -1 and wf["thr-sf"]["frac"] == 0.4                     # ch 4 = SW-F (M4 BOW)
    assert wf["thr-sa"]["state"] == "available" and wf["thr-sa"]["label"] == "idle 1500"   # ch 2 = SW-A (M2 STERN), outline
    assert wf["lamp"] == dict(wf["lamp"], state="expected", colour="GREEN") and "expected GREEN" in wf["lamp"]["l1"]
    # unmapped -> dashed (unavailable)
    m2 = _model_with(fcu, {"state": "MANUAL"}, thr_map={"thr-mp": 1, "thr-ms": 3, "thr-sf": 4})
    s2 = m2.snapshot(now=100.2)
    assert s2["wf"]["thr-sa"]["state"] == "unavailable" and "not mapped" in s2["wf"]["thr-sa"]["label"]
    assert s2["wf"]["lamp"]["colour"] == "YELLOW" and s2["holder"] == "mission manager"
    assert _model_with(fcu, {"state": "KILLED"}).snapshot(now=100.2)["wf"]["lamp"]["colour"] == "RED"
    assert _model_with(fcu, {"state": "FAULT"}).snapshot(now=100.2)["wf"]["lamp"]["state"] == "unknown"
    # unknown after 2 s: the heartbeat is old -> the lamp and the thrusters say unknown, nothing is guessed
    late = m.snapshot(now=102.5)
    assert late["wf"]["lamp"]["state"] == "unknown" and late["wf"]["lamp"]["colour"] is None
    assert all(late["wf"][k]["state"] == "unknown" for k in ("thr-mp", "thr-ms", "thr-sf", "thr-sa"))
    m3 = _model_with(dict(fcu, hb_age_s=2.4), {"state": "AUTO_HOLD"})
    s3 = m3.snapshot(now=100.1)
    assert s3["wf"]["lamp"]["state"] == "unknown" and "no FCU heartbeat" in s3["wf"]["lamp"]["l1"]
    assert s3["wf"]["box"]["state"] == "fault" and any("FCU LINK LOST" in a["text"] for a in s3["alarms"])
    # nothing heard at all: every element unknown, every value a hatched 'unknown', no zero
    e = BW.Model("usv").snapshot(now=5.0)
    assert all(e["wf"][k]["state"] in ("unknown", "unavailable", "fault") for k in ("thr-mp", "lamp", "ant", "box"))
    assert dict(e["panels"]["health"])["power"] == [{"u": "V not reported"}, {"u": "I not reported"}]
    # current reported -> shown; not reported -> hatched (never 0)
    s4 = _model_with(dict(fcu, voltage_v=15.9, current_a=None)).snapshot(now=100.1)
    assert dict(s4["panels"]["health"])["power"] == ["15.9 V", {"u": "I not reported"}]
    # antenna good / weak / lost from the status age and the bridge
    assert m.antenna(100.3)["q"] == "good" and m.antenna(101.8)["q"] == "weak" and m.antenna(106.0)["q"] == "lost"
    m.on_message("$SYS/broker/connection/ocs/state", b"0", now=100.3)
    assert m.antenna(100.4)["q"] == "lost" and any("OCS LINK DOWN" in a["text"] for a in m.alarms(100.4))
    # electronics: the host beats
    m.on_message("local/usv/host/agx", json.dumps({"body": {"temp_c": {"gpu": 91.0}, "throttling": True, "disk_free_gb": 23}}).encode(), now=100.3)
    assert m.electronics(100.4)["state"] == "warning" and "throttling" in m.electronics(100.4)["l1"]
    assert BW.parse_thr_map("mp=1,ms=3") == {"thr-mp": 1, "thr-ms": 3}
    with pytest.raises(ValueError):
        BW.parse_thr_map("xx=1")


def _chrome_dom(url):
    dom = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=8000", "--dump-dom", url],
                         capture_output=True, text=True, timeout=90).stdout
    return dom


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_page_draws_set_state_and_every_button_has_a_title():
    fcu = {"hb_age_s": 0.3, "servo": [1660, 1500, 1890, 0, 1500, 1500, 1500, 1500], "active": True, "fcu_link": "fake", "mode": "HOLD",
           "armed": True, "kill": False, "estop": False}
    m = _model_with(fcu, {"state": "MANUAL", "health": {"fcu_age_s": 0.2}}, now=time.monotonic(), thr_map={"thr-mp": 1, "thr-ms": 3, "thr-sf": 4})
    srv, url = _serve(m)
    try:
        dom = _chrome_dom(url + "/")
    finally:
        srv.shutdown()
    tag = lambda i: re.search(r'<[a-z]+ [^>]*id="%s"[^>]*>' % re.escape(i), dom).group(0)
    assert 'data-state="active"' in tag("thr-mp") and 'fill-opacity="0.490"' in tag("thr-mp")            # 0.25 + 0.6 * 0.4
    assert 'data-state="warning"' in tag("thr-ms") and "var(--warn)" in tag("thr-ms")
    assert 'data-state="unavailable"' in tag("thr-sf") and 'stroke-dasharray="4 3"' in tag("thr-sf")      # ch 4 = 0 -> dashed
    assert 'data-state="unavailable"' in tag("thr-sa") and 'stroke-dasharray="4 3"' in tag("thr-sa")      # unmapped -> dashed
    assert 'data-colour="YELLOW"' in tag("lamp") and "expected YELLOW" in dom
    assert "held by usv_ui" in dom
    body = re.sub(r"<script\b.*?</script>", "", dom, flags=re.S)
    buttons = re.findall(r"<button\b[^>]*>", body)
    assert len(buttons) >= 8 and [b for b in buttons if "title=" not in b] == []
    assert len(re.findall(r'class="u"', body)) >= 5                                    # the hatched 'unknown / not published' values


def test_usv_ui_still_draws_at_200x60(monkeypatch):
    m = load_usv_ui()
    for k in ("HLINE", "VLINE", "ULCORNER", "URCORNER", "LLCORNER", "LRCORNER", "LTEE", "RTEE", "TTEE", "BTEE", "PLUS", "BLOCK",
              "CKBOARD", "BULLET", "DIAMOND", "DEGREE"):
        if not hasattr(m.curses, "ACS_" + k):
            monkeypatch.setattr(m.curses, "ACS_" + k, ord("+"), raising=False)

    class Scr:
        def __init__(s, h, w):
            s.h, s.w = h, w
            s.erase()

        def erase(s):
            s.buf = [[" "] * s.w for _ in range(s.h)]

        def getmaxyx(s):
            return s.h, s.w

        def addnstr(s, y, x, t, n, a=0):
            for i, ch in enumerate(t[:n]):
                if x + i < s.w:
                    s.buf[y][x + i] = ch

        def __getattr__(s, k):
            return lambda *a, **kw: None

    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.bind(("127.0.0.1", 0))
    port = so.getsockname()[1]
    so.close()
    app = m.App(m.build_parser().parse_args([f"udpin:127.0.0.1:{port}", "--mqtt", ""]))
    try:
        app.mq = None
        app.remote_cmd(time.monotonic(), {"word": "note", "args": ["hello"], "id": "x"})       # a page word without a broker: runs, no reply
        app.remote_cmd(time.monotonic(), {"word": "arm", "id": "y"})
        assert any(t == "[page] note hello" for _, t, _k in app.log) and any(t == "[page] arm" for _, t, _k in app.log)
        assert not app.tel.armed
        b = app.fcu_block(time.monotonic())
        assert b["hb_age_s"] is None and b["mode"] is None and b["armed"] is None and b["servo"] is None and b["voltage_v"] is None
        for screen in range(3):
            app.scr, app.screen, app.log_scroll, app.unicode, app.C = Scr(60, 200), screen, 0, False, collections.defaultdict(int)
            app.draw()
            assert "".join(app.scr.buf[0]).startswith(" usv_ui v")
    finally:
        try:
            app.close_links()
        except Exception:
            pass
