"""v0.1.54 part B (Dmitrii's round after v0.1.53): item 9 STOP & RESET (the OCS macro `stop_reset`: abort -> disable_auto -> FCU HOLD ->
DISARM -> report off (heartbeats off even when declared) -> reset; the boat window's red button through the OCS page's /cmd), item 7's
backend (`report off` = heartbeats OFF now, whatever R), item 10 the camera on the boat window (snapshot.jpg loop from the --cam-status
address), item 11 the boxes (sim_observer's pixel detections -> cv_publish local/<v>/perception/frame_dets -> the boat window's overlay
and the OCS page's GET /dets.json)."""
from __future__ import annotations

import io
import json
import os
import re
import subprocess
import sys
import threading
import time
import types
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from optiprimex.companion.config import Config
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.tools import boat_web as BW
from optiprimex.tools import bench_tui as T
from optiprimex.tools import ocs_cli, playbook

from .conftest import PY, ROOT, Probe, Proc, free_port, make_ocs_config, make_usv_config, wait_until

NS = "team/optiprime/v1"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
SHOTS = ROOT / "docs" / "screens" / "v0.1.54"


# ---- item 9 / 7: the words, the playbook entry, report off -----------------------------------------------------------------------
def test_the_stop_reset_word_everywhere():
    assert ocs_cli.parse(["stop_reset"]).cmd == "stop_reset"
    assert "stop_reset" in T.INTENT_WORDS and any(w == "stop_reset" for w, _u, _d in T.USAGE) and T.EXAMPLES["stop_reset"] == "stop_reset"
    assert "stop_reset" not in T.WORD_TO_ACTION                         # a macro, like ready / manual: no single intent behind it
    e = playbook.MACROS["STOP_RESET"]
    assert e.cli == "stop_reset" and "STOP_RESET" not in playbook.PLAYBOOK  # not an intent: allowed_now / check stay on GUARDS
    assert [s[0] for s in playbook.STOP_RESET_STEPS] == ["cancel_auto", "abort", "disable_auto", "fcu_mode HOLD", "disarm", "reset"]   # v0.1.59 (B1): no report off
    assert any(ln.startswith("stop_reset") and "macro" in ln for ln in playbook.help_lines())
    assert "ocs_cli stop_reset" in ocs_cli.__doc__


def test_report_off_is_a_hard_off():
    """item 7: `report off` = heartbeats off now even when declared (was: back to the declaration rule, still on in R3+)."""
    app = OcsApp(Config())
    app.on_reporting(True)
    assert app.heartbeats.enabled
    app.lifecycle.body = lambda: {"r": "R5"}                           # declared and running: the old rule kept them on here
    app.on_reporting(False)
    assert app.heartbeats.enabled is False and {k: app.state_body()["reporting"][k] for k in ("on", "heartbeats", "off")} == {"on": False, "heartbeats": False, "off": True}
    app.lifecycle.on_run_start = lambda cmd: True                       # a RunStart of the same declaration: still off
    app.on_rc_command({"kind": "run_start", "command_seq": 1, "run_id": 5, "declaration_seq": 1})
    assert app.heartbeats.enabled is False
    sent = []
    app.heartbeats.judge = types.SimpleNamespace(publish_report=lambda v, b: sent.append(v) or 1)
    app.heartbeats.update("USV1", {"state": "AUTO_HOLD"})
    app.heartbeats.tick()
    assert sent == []                                                   # the judge sees none
    app.reset_run()
    assert app.heartbeats.enabled is False                              # a reset keeps them off
    src = (ROOT / "optiprimex" / "ocs" / "cmd_authority.py").read_text()
    assert "reporting OFF: heartbeats stopped now, whatever R" in src


# ---- the boat window's whitelist: exactly `stop_reset` added ------------------------------------------------------------------
def test_the_boat_window_forwards_stop_reset_only():
    got = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            got.append((self.path, self.headers.get("X-OCS-Web"), body))
            out = json.dumps({"ok": True, "lines": ["> reset", "steps: abort ok", "now: OCS R2", "next: rerun"]}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)
    port = free_port()
    srv = HTTPServer(("127.0.0.1", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        assert set(BW.OCS_LINES) == {"rc6_policy latched", "rc6_policy hold", "stop_reset", "enable_auto", "disable_auto", "confirm_safe",
                                     "station_keep hold", "station_keep loiter"}   # v0.1.62: + the VEHICLE menu; v0.1.81: + the station keep
        r = BW.ocs_line("http://127.0.0.1:%d" % port, " STOP_RESET ")
        assert r["ok"] and got[-1] == ("/cmd", "1", {"line": "stop_reset"}) and r["lines"][-1] == "next: rerun"
        n = len(got)
        for bad in ("stop_reset now", "reset", "abort", "disarm", "report off", "stop_reset; arm"):
            assert BW.ocs_line("http://127.0.0.1:%d" % port, bad)["text"].startswith("refused by the page"), bad
        assert len(got) == n                                            # nothing else travels
        assert BW.OCS_LINE_TIMEOUT_S["stop_reset"] >= 30                # seven steps with their acks
        # through the boat window's POST /ocs: the reply lands in the console + an event
        m = BW.Model("usv")
        wport = free_port()
        ws = BW.Server(("127.0.0.1", wport), BW.make_handler(m, None, bar=None, ocs_url="http://127.0.0.1:%d" % port))
        threading.Thread(target=ws.serve_forever, daemon=True).start()
        try:
            req = urllib.request.Request("http://127.0.0.1:%d/ocs" % wport, data=b'{"line": "stop_reset"}', method="POST",
                                         headers={"Content-Type": "application/json", "X-Boat-Web": "1"})
            out = json.loads(urllib.request.urlopen(req, timeout=10).read())
            assert out["ok"] and m.cmds[0]["word"] == "stop_reset" and any("STOP & RESET" in t for _, t in m.events)
        finally:
            ws.shutdown()
    finally:
        srv.shutdown()
    page = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text()
    assert 'id="b-sr"' in page and "■ STOP &amp; RESET" in page and "JSON.stringify({line:'stop_reset'})" in page
    assert "Date.now()-SR_T>=4000" in page                              # two clicks within 4 s, like ARM


# ---- item 11: the frame_dets payload from sim_observer's code path, through cv_publish ---------------------------------------------
def test_frame_dets_from_the_observer_through_cv_publish():
    np = pytest.importorskip("numpy")
    pytest.importorskip("PIL")
    from .test_wp10e import _frame, observer
    SO = observer()
    fr, wd, _ = _frame()
    dets = SO.synthetic_detections(fr)
    p = SO.observe(fr, dets, SO.parse_args(["--depth-noise", "none"]))
    fd = p["frame_dets"]
    assert fd["frame_w"] == 1104 and fd["frame_h"] == 621 and fd["frame_id"] == 1 and fd["t"] == 0.0
    assert len(fd["dets"]) == len(p["buoys"]) >= 7                      # one box per buoy record, nothing invented
    by_obj = {d["obj"]: d for d in dets}
    for b, d in zip(p["buoys"], fd["dets"]):
        assert set(d) == {"cls", "colour", "light", "x0", "y0", "x1", "y1", "range_m", "conf"}
        assert d["cls"] == "buoy" and d["colour"] == b["colour"] and d["light"] is None
        ys, xs = np.nonzero(by_obj[b["truth_obj"]]["mask"] > 0.5)
        assert [d["x0"], d["y0"], d["x1"], d["y1"]] == [xs.min(), ys.min(), xs.max(), ys.max()]       # the mask's own box
        assert 0 <= d["x0"] <= d["x1"] < 1104 and 0 <= d["y0"] <= d["y1"] < 621
        assert (d["range_m"] is None) == (b.get("range_m") is None)
    # the dock parts carry their box too (the record's `px`)
    rec = SO._dock_rec("window", {"conf": 0.9, "mask": np.pad(np.ones((4, 6), np.float32), ((10, 5), (20, 3)))}, (5.0, 1.0, 0.2, 30), "red", 0)
    assert rec["px"] == [20, 10, 25, 13]
    fd2 = SO.frame_dets(fr, {"t": 1.0, "frame": 9}, [], [dict(rec, light="red")])
    assert fd2["dets"][0]["cls"] == "window" and fd2["dets"][0]["light"] == "red" and fd2["dets"][0]["range_m"] == 5.1
    # cv_publish: perception/frame_dets as it came + health.boxes
    sys.path.insert(0, str(ROOT / "cv" / "publish"))
    try:
        import cv_publish as CP
    finally:
        sys.path.pop(0)
    sent = []
    pub = CP.Publisher(types.SimpleNamespace(mqtt="127.0.0.1:1", vehicle="usv", vehicle_id="USV1", camera="zed2i_left", detector="sim_observer",
                                             model=None, demo=False))
    pub.c = types.SimpleNamespace(publish=lambda topic, payload, qos=0: sent.append((topic, json.loads(payload)["body"])))
    pub.health()
    assert sent[-1][1]["boxes"] is False                                # before a frame with boxes: the overlay says "not published"
    CP.publish_payload(pub, json.loads(json.dumps(p, default=float)))
    fdm = [b for t, b in sent if t == "local/usv/perception/frame_dets"]
    assert len(fdm) == 1 and fdm[0]["dets"] == json.loads(json.dumps(fd["dets"])) and fdm[0]["frame_w"] == 1104 and fdm[0]["source"] == "synthetic"
    pub.health()
    assert sent[-1][1]["boxes"] is True
    n = len(sent)
    CP.publish_payload(pub, {"updated_utc": "2026-11-06T00:00:00+00:00", "frame": 3, "status": "OK", "buoys": []})   # the AGX observer: none
    assert not [t for t, _ in sent[n:] if t.endswith("/frame_dets")]


def test_the_boat_window_model_camera_and_dets():
    m = BW.Model("usv")
    assert "local/usv/perception/frame_dets" in m.topics()
    now = 100.0
    assert m.camview(now)["why"].startswith("no camera address")
    m.cam = (now - 1, None, "http://10.0.0.5:8097/status", "URLError")
    assert m.camview(now) == {"ok": False, "base": "http://10.0.0.5:8097/", "why": "camera not answering at http://10.0.0.5:8097/ (URLError)"}
    m.cam = (now - 1, {"source": "sim", "frames": 12, "age_s": 0.2, "clients": 0}, "http://10.0.0.5:8097/status", None)
    v = m.camview(now)
    assert v["ok"] and v["base"] == "http://10.0.0.5:8097/" and v["frames"] == 12
    assert m.camview(now + 10)["ok"] is False                           # an old status: no loop against a dead address
    d = m.dets_view(now)
    assert d["state"] == "none" and d["dets"] == [] and d["why"] == "boxes not published"
    env = {"body": {"ok": True, "detector": "actual_buoy_observer", "boxes": False}}
    m.on_message("local/usv/perception/health", json.dumps(env).encode(), now=now)
    assert m.dets_view(now)["why"] == "boxes not published (actual_buoy_observer gives no pixel boxes)"
    body = {"frame_w": 1104, "frame_h": 621, "frame_id": 5, "source": "synthetic",
            "dets": [{"cls": "buoy", "colour": "red", "light": None, "x0": 1, "y0": 2, "x1": 30, "y1": 40, "range_m": 6.0, "conf": 0.9}]}
    m.on_message("local/usv/perception/frame_dets", json.dumps({"body": body}).encode(), now=now)
    d = m.dets_view(now + 0.5)
    assert d["state"] == "live" and d["n"] == 1 and d["age_s"] == 0.5 and d["dets"][0]["x1"] == 30
    assert m.dets_view(now + 3)["state"] == "stale" and m.dets_view(now + 3)["dets"] == []     # nothing old drawn
    snap = m.snapshot()
    assert "camv" in snap and "dets" in snap


def test_the_ocs_page_dets_json_tap():
    from optiprimex.tools import ocs_web as OW
    tap = OW.DetsTap("usv")
    assert tap.view({"detector": "actual_buoy_observer"})["why"] == "boxes not published (actual_buoy_observer gives no pixel boxes)"
    calls = {}

    class FakeClient:
        on_connect = staticmethod(lambda *a: calls.setdefault("prev", True))

        def message_callback_add(self, topic, cb):
            calls["cb"] = (topic, cb)

        def subscribe(self, topic, qos):
            calls["sub"] = topic
    c = FakeClient()
    tap.attach(c)
    c.on_connect(c, None, None, 0)
    assert calls["prev"] and calls["sub"] == "local/usv/perception/frame_dets" and calls["cb"][0] == calls["sub"]
    calls["cb"][1](c, None, types.SimpleNamespace(payload=json.dumps({"body": {"frame_w": 10, "frame_h": 5, "dets": [{"cls": "buoy"}]}}).encode()))
    v = tap.view()
    assert v["state"] == "live" and v["dets"] == [{"cls": "buoy"}] and v["frame_w"] == 10
    tap.last = (time.monotonic() - 5, tap.last[1])
    assert tap.view()["state"] == "stale" and tap.view()["dets"] == []
    src = (ROOT / "optiprimex" / "tools" / "ocs_web.py").read_text()
    assert 'elif path == "/dets.json":' in src and "self.dets.attach(self.feeds.boat)" in src


# ---- the live chain: STOP & RESET through the OCS page's /cmd (as the boat window posts it) ---------------------------------------
@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("v154b")
    usv_cfg = make_usv_config(tmp, brokers, fake_fcu)
    ocs_cfg = make_ocs_config(tmp, brokers)
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(usv_cfg)], tmp / "mm.log")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--start-delay", "1.0",
               "--log", str(tmp / "rc.jsonl")], tmp / "rc.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#", "rc/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 20, include_past=True), mm.log()
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(ocs_cfg)], tmp / "ocs.log")
    assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 20, include_past=True), ocs.log()
    wport = free_port()
    web = Proc([PY, "-m", "optiprimex.tools.ocs_web", "--port", str(brokers.hub_port), "--boat-port", str(brokers.boat_port),
                "--web-port", str(wport), "--judge-port", str(brokers.hub_port), "--ws-port", str(free_port())], tmp / "web.log",
               env=dict(os.environ, OCS_MARKERS=str(tmp / "markers.json")))
    url = "http://127.0.0.1:%d" % wport
    assert wait_until(lambda: _get(url + "/state.json") is not None, 20), web.log()
    con = ocs_cli.OcsConsole(port=brokers.hub_port)
    con.start()
    yield {"probe": probe, "con": con, "url": url, "fcu": fake_fcu, "mm": mm, "ocs": ocs, "web": web, "brokers": brokers}
    con.stop()
    probe.stop()
    for p in (web, ocs, rc, mm):
        p.kill()


def _get(u):
    try:
        with urllib.request.urlopen(u, timeout=3) as r:
            return json.loads(r.read())
    except Exception:
        return None


def _hb_count(p) -> int:
    return len([m for m in p.all("rc/report/") if (m["env"] or {}).get("kind") == "heartbeat"])


def _hb_quiet(p, s: float = 2.0) -> bool:
    time.sleep(0.8)                                                     # one in flight may still land
    n = _hb_count(p)
    time.sleep(s)
    return _hb_count(p) == n


def test_report_off_stops_heartbeats_when_declared(stack):
    con, p = stack["con"], stack["probe"]
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10)
    assert p.wait(lambda m: m["topic"].startswith("rc/report/") and (m["env"] or {}).get("kind") == "heartbeat", 5), "no heartbeat after declare"
    code, lines = ocs_cli.execute(con, ocs_cli.parse(["report", "off"]))
    assert code == 0 and any("reporting OFF: heartbeats stopped now, whatever R" in ln for ln in lines), lines
    assert _hb_quiet(p), ("heartbeats still sent after report off in R3", [m["env"]["body"].get("reporting") for m in p.all("ocs/state")][-3:])
    time.sleep(1.5)                                                     # the fake judge's RunStart (1 s after the declaration) came:
    assert _hb_quiet(p, 1.5), "a RunStart of the same declaration turned them back on"     # still off (only report on / declare)
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] in ("R3", "R4", "R5")          # declared (the fake judge's
                  and {k: m["env"]["body"]["reporting"][k] for k in ("on", "heartbeats", "off")} == {"on": False, "heartbeats": False, "off": True}, 5)   # RunStart: R5)
    code, lines = ocs_cli.execute(con, ocs_cli.parse(["report", "on"]))
    assert code == 0 and p.wait(lambda m: m["topic"].startswith("rc/report/") and (m["env"] or {}).get("kind") == "heartbeat", 5)


def test_stop_reset_on_the_live_chain(stack):
    """declared, AUTO, RunStart, a task running -> STOP & RESET from the page's /cmd -> the task aborted, AUTO off, the fake FCU in HOLD
    and disarmed, no heartbeat any more, the OCS in R2 with the run forgotten; then the sequence runs again (arm, declare)."""
    con, p, fcu, url = stack["con"], stack["probe"], stack["fcu"], stack["url"]
    if (con.wait_state(3) or {}).get("r") not in ("R3", "R4", "R5"):
        assert con.intent("DECLARE")["result"]["result"] == "accepted"
        assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10, include_past=True)
    assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15, include_past=True), stack["ocs"].log()[-1500:]
    r = con.intent("START_TASK", "TEST_AUTO", wait_s=5)
    assert r["result"]["result"] == "accepted", r
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_TASK", 10, include_past=True)
    assert fcu.state()["armed"] is True
    p.clear()
    t0 = time.monotonic()
    req = urllib.request.Request(url + "/cmd", data=json.dumps({"line": "stop_reset"}).encode(), method="POST",
                                 headers={"Content-Type": "application/json", "X-OCS-Web": "1"})
    rec = json.loads(urllib.request.urlopen(req, timeout=60).read())
    took = time.monotonic() - t0
    lines = rec["lines"]
    text = "\n".join(lines)
    assert rec["ok"], text
    # the order: each step announced in turn, its verdict right under it
    order = [ln[2:].split(":")[0] for ln in lines if ln.startswith("> ")]
    assert order == ["cancel_auto", "abort", "disable_auto", "fcu_mode HOLD", "disarm", "reset"]   # v0.1.59 (B1): no report off, order
    assert "> cancel_auto: skipped - no auto-start countdown" in lines
    steps = next(ln for ln in lines if ln.startswith("steps: "))
    for s in ("abort ok", "disable_auto ok", "fcu_mode HOLD ok", "disarm ok", "reset ok"):     # v0.1.59: report off left the chain
        assert s in steps, (s, steps)
    cmds = [m["env"]["body"]["action"] for m in p.all(f"{NS}/cmd/usv/run")]
    i = [cmds.index(a) for a in ("ABORT_TASK", "DISABLE_AUTO", "FCU_MODE", "FCU_ARM", "RESET")]
    assert i == sorted(i), cmds                                          # on the wire in that order
    # the effects
    assert wait_until(lambda: fcu.state().get("mode_name") == "HOLD" and fcu.state().get("armed") is False, 5), fcu.state()
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 5, include_past=True)
    # v0.1.59 (B1): STOP & RESET leaves reporting as it is - the test before ended with `report on`, so the heartbeats go on
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2" and m["env"]["body"]["run_id"] is None
                  and m["env"]["body"]["reporting"]["on"] is True and m["env"]["body"]["reporting"]["heartbeats"] is True, 5, include_past=True)
    assert p.wait(lambda m: m["topic"].startswith("rc/report/") and (m["env"] or {}).get("kind") == "heartbeat", 3), "no heartbeat after STOP & RESET"
    assert next(ln for ln in lines if ln.startswith("now: ")).endswith("heartbeats ON"), text
    assert took < 40, took
    # a second press (nothing running, R2): every not-applicable step said so, the FCU commands still go out, the chain ends
    rec2 = json.loads(urllib.request.urlopen(urllib.request.Request(url + "/cmd", data=b'{"line": "stop_reset"}', method="POST",
                                             headers={"Content-Type": "application/json", "X-OCS-Web": "1"}), timeout=60).read())
    l2 = rec2["lines"]
    assert any(ln.startswith("> abort: skipped - no task runs (vehicle MANUAL)") for ln in l2), l2
    assert any(ln.startswith("> disable_auto: skipped") for ln in l2) and "> fcu_mode HOLD" in l2 and "> disarm" in l2 and "> reset" in l2
    # rerun the sequence: arm, declare -> heartbeats again
    assert con.intent("FCU_ARM", args={"on": True}, wait_s=5)["result"]["result"] == "accepted"
    assert wait_until(lambda: fcu.state().get("armed") is True, 5)
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"].startswith("rc/report/") and (m["env"] or {}).get("kind") == "heartbeat", 8)
    assert con.intent("RESET_RUN", wait_s=5)["result"]["result"] == "accepted"


def test_dets_json_on_the_live_ocs_page(stack):
    """item 11: the OCS page's own boat-broker client takes perception/frame_dets; /dets.json serves it with its age."""
    import paho.mqtt.client as mqtt
    url = stack["url"]
    assert _get(url + "/dets.json")["state"] in ("none", "stale")
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t-dets-%d" % free_port(), protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", stack["brokers"].boat_port)
    c.loop_start()
    body = {"t_cam": 1.0, "frame_w": 1104, "frame_h": 621, "frame_id": 77, "source": "synthetic",
            "dets": [{"cls": "buoy", "colour": "green", "light": None, "x0": 10, "y0": 20, "x1": 30, "y1": 50, "range_m": 4.2, "conf": 0.9}]}
    try:
        assert wait_until(lambda: (c.publish("local/usv/perception/frame_dets", json.dumps({"body": body}), qos=0) and False)
                          or (_get(url + "/dets.json") or {}).get("frame_id") == 77, 8)
    finally:
        c.loop_stop()
        c.disconnect()
    d = _get(url + "/dets.json")
    assert d["state"] == "live" and d["dets"] == body["dets"] and d["frame_w"] == 1104 and d["age_s"] < 2


# ---- item 10 + 11 in Chrome at 1440 x 900: the camera card with the boxes, the button; screenshots ----------------------------------
def _jpeg(n):
    from PIL import Image, ImageDraw
    im = Image.new("RGB", (1104, 621), (40, 90, 140))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 330, 1104, 621], fill=(20, 60, 100))
    d.ellipse([300, 280, 360, 350], fill=(214, 69, 65))
    d.ellipse([700, 300, 740, 345], fill=(46, 158, 91))
    d.rectangle([880, 200, 1060, 330], fill=(230, 230, 230))
    d.rectangle([910, 230, 950, 270], fill=(200, 40, 40))
    d.text((10, 10), "frame %d" % n, fill=(255, 255, 255))
    b = io.BytesIO()
    im.save(b, "JPEG")
    return b.getvalue()


DETS = {"t_cam": 0, "frame_w": 1104, "frame_h": 621, "frame_id": 42, "source": "synthetic", "dets": [
    {"cls": "buoy", "colour": "red", "light": None, "x0": 300, "y0": 280, "x1": 360, "y1": 350, "range_m": 7.4, "conf": 0.9},
    {"cls": "buoy", "colour": "green", "light": None, "x0": 700, "y0": 300, "x1": 740, "y1": 345, "range_m": None, "conf": 0.9},
    {"cls": "dock_face", "colour": "white", "light": None, "x0": 880, "y0": 200, "x1": 1060, "y1": 330, "range_m": 12.1, "conf": 0.9},
    {"cls": "window", "colour": "red", "light": "red", "x0": 910, "y0": 230, "x1": 950, "y1": 270, "range_m": 12.0, "conf": 0.9}]}


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_camera_card_and_the_boxes_in_chrome_at_1440():
    pytest.importorskip("PIL")
    from .test_v0149 import _rich_model
    n = [0]

    class Cam(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_GET(self):
            if self.path.startswith("/status"):
                body, ct = json.dumps({"source": "sim", "frames": n[0], "age_s": 0.1, "clients": 0}).encode(), "application/json"
            elif self.path.startswith("/snapshot.jpg"):
                n[0] += 1
                body, ct = _jpeg(n[0]), "image/jpeg"
            else:
                self.send_response(404)
                self.end_headers()
                return
            self.send_response(200)
            self.send_header("Content-Type", ct)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
    cport = free_port()
    cam = BW.Server(("127.0.0.1", cport), Cam)
    threading.Thread(target=cam.serve_forever, daemon=True).start()
    m = _rich_model(time.monotonic())
    m.cam = (time.monotonic(), {"source": "sim", "frames": 1, "age_s": 0.1, "clients": 0}, "http://127.0.0.1:%d/status" % cport, None)
    BW.cam_poller(m, "http://127.0.0.1:%d/status" % cport, period_s=0.5)
    stop = threading.Event()

    def feed():
        while not stop.is_set():
            m.dets = (time.monotonic(), dict(DETS))
            time.sleep(0.25)
    threading.Thread(target=feed, daemon=True).start()
    wport = free_port()
    srv = BW.Server(("127.0.0.1", wport), BW.make_handler(m, None, bar=None))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = "http://127.0.0.1:%d/" % wport
    SHOTS.mkdir(parents=True, exist_ok=True)
    try:
        for theme in ("dark", "light"):
            args = [CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--hide-scrollbars"]
            dom = subprocess.run(args + ["--dump-dom", url + "?theme=" + theme], capture_output=True, text=True, timeout=90).stdout
            ov = re.search(r'<svg id="cam-ov"[^>]*data-n="(\d+)"[^>]*data-state="(\w+)"', dom)
            assert ov and ov.group(1) == "4" and ov.group(2) == "live", (theme, ov and ov.group(0))
            assert re.search(r'<img id="cam-img" alt="" src="http://127\.0\.0\.1:%d/snapshot\.jpg\?t=\d+"' % cport, dom), theme
            for lab in ("buoy red · 7.4 m", "buoy green · no range", "dock_face · 12.1 m", "window · red · 12.0 m"):
                assert lab in dom, (theme, lab)
            assert 'stroke-dasharray' in dom.split('id="cam-ov"')[1].split("</svg>")[0]      # the bearing-only box dashed
            L = json.loads(re.search(r'<body[^>]*data-layout="([^"]+)"', dom).group(1).replace("&quot;", '"'))
            for k in ("camw", "b-sr", "p-ev", "p-con"):
                x, y, w, h = L[k]
                assert w > 0 and h > 0 and y + h <= 900 and x + w <= 1440, (theme, k, L[k])            # on the first screen
            assert L["camw"][3] == 300                                                                 # fixed: no jump
            assert re.search(r'id="cam-dt"[^>]*>4 boxes · frame 42 · dets \d\.\d s · synthetic · picture', dom), theme
            subprocess.run(args + ["--screenshot=%s" % (SHOTS / ("boat_window_camera_%s.png" % theme)), url + "?theme=" + theme],
                           capture_output=True, timeout=90)
        # no boxes published: said, never drawn
        stop.set()
        m.dets = None
        dom = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=4000", "--window-size=1440,900", "--dump-dom", url],
                             capture_output=True, text=True, timeout=90).stdout
        assert re.search(r'<svg id="cam-ov"[^>]*data-n="0"[^>]*data-state="none"', dom) and "boxes not published" in dom
    finally:
        stop.set()
        srv.shutdown()
        cam.shutdown()
