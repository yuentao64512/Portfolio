"""v0.1.86.9 (the venue analysis of 2026-11-12, B1 / B2 / B5 / B6 / C6): /cmd answers at the vehicle's ACCEPTED, the final DONE /
FAILED is followed and served at /cmd_result, presses queue instead of "busy", every press's timing in cmd_timing.jsonl, /state.json
shared between clients for 0.25 s, the page's poll guard, the tk console's follow-up toast and its camera decode off the main thread."""
from __future__ import annotations

import io
import json
import os
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer

import pytest

from optiprimex.tools import ocs_web
from optiprimex.tools.tkconsole.net import CamWorker, CommandWorker
from optiprimex.tools.tkconsole.toasts import ToastBook, verdict

from .conftest import PY, ROOT, Probe, Proc, free_port, make_ocs_config, make_usv_config, wait_until

NS = "team/optiprime/v1"


def _get(u):
    try:
        with urllib.request.urlopen(u, timeout=3) as r:
            return json.loads(r.read())
    except Exception:
        return None


def _post(u, body, timeout=30):
    req = urllib.request.Request(u, data=json.dumps(body).encode(), headers={"Content-Type": "application/json", "X-OCS-Web": "1"})
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read())


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("v18609")
    usv_cfg = make_usv_config(tmp, brokers, fake_fcu)
    ocs_cfg = make_ocs_config(tmp, brokers)
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(usv_cfg)], tmp / "mm.log")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--start-delay", "1.0"], tmp / "rc.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 20, include_past=True), mm.log()
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(ocs_cfg)], tmp / "ocs.log")
    assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 20, include_past=True), ocs.log()
    wport = free_port()
    timing = tmp / "cmd_timing.jsonl"
    web = Proc([PY, "-m", "optiprimex.tools.ocs_web", "--port", str(brokers.hub_port), "--boat-port", str(brokers.boat_port),
                "--web-port", str(wport), "--judge-port", str(brokers.hub_port), "--ws-port", str(free_port())], tmp / "web.log",
               env=dict(os.environ, OCS_MARKERS=str(tmp / "markers.json"), OCS_CMD_TIMING=str(timing)))
    url = "http://127.0.0.1:%d" % wport
    assert wait_until(lambda: (_get(url + "/state.json") or {}).get("status") is not None, 20), web.log()
    yield {"url": url, "timing": timing, "web": web}
    probe.stop()
    for p in (web, ocs, rc, mm):
        p.kill()


def _final(url, rid, timeout_s=15.0):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s:
        f = _get(f"{url}/cmd_result?id={rid}")
        if f and f.get("final"):
            return f
        time.sleep(0.1)
    return None


def test_cmd_answers_at_accepted_then_the_final_follows(stack):
    url = stack["url"]
    t0 = time.monotonic()
    r = _post(url + "/cmd", {"line": "fcu_mode HOLD"})
    took = time.monotonic() - t0
    assert r["ok"], r
    assert "accepted_ms" in r["ms"] and r["ms"]["reply_ms"] < 1500 and took < 2.0, (r, took)
    f = _final(url, r["id"])                                       # final at once or followed: /cmd_result has it either way
    assert f and f["final"] and f["verdict"] == "ok" and any("DONE" in x for x in f["lines"]), f
    assert ok_line(f), f["lines"]
    rows = [json.loads(x) for x in stack["timing"].read_text().splitlines()]
    row = next(x for x in rows if x["word"] == "fcu_mode")
    assert row["until"] == "accepted" and row["verdict"] == "ok" and "final_ms" in row and row["accepted_ms"] <= row["final_ms"], row


def ok_line(f):
    ok, line, _ = verdict(f)
    return ok and line


def test_presses_queue_never_busy(stack):
    url = stack["url"]
    out = []

    def press(line):
        out.append(_post(url + "/cmd", {"line": line}))

    th = [threading.Thread(target=press, args=(w,)) for w in ("fcu_mode HOLD", "station_keep hold", "fcu_mode HOLD")]
    t0 = time.monotonic()
    for t in th:
        t.start()
    for t in th:
        t.join(20)
    assert len(out) == 3 and time.monotonic() - t0 < 8.0
    assert not any("busy" in " ".join(r.get("lines") or []) for r in out), out


def test_refused_is_final_at_once(stack):
    r = _post(stack["url"] + "/cmd", {"line": "start_task m1t1"})      # no run: refused by the OCS or the vehicle
    assert r["final"] is True and not r["ok"]
    assert _get(stack["url"] + "/cmd_result?id=" + r["id"])["verdict"] == "refused"
    assert _get(stack["url"] + "/cmd_result?id=nope") == {"id": "nope", "final": False}


def test_state_json_is_shared_for_a_quarter_second_and_dropped_by_a_press():
    web = ocs_web.Web(ocs_web.parse_args([]), env={}, feeds=False)
    a = web.state_json()
    assert web.state_json() is a                                   # the same bytes for a second client
    time.sleep(ocs_web.STATE_TTL_S + 0.05)
    b = web.state_json()
    assert b is not a and json.loads(b)["t"] > json.loads(a)["t"]
    web.command("hold")                                            # no feeds: the line fails, the cache is dropped all the same
    assert web.state_json() is not b
    assert web.cmd_until == "accepted"
    assert ocs_web.Web(ocs_web.parse_args([]), env={"OCS_CMD_UNTIL": "done"}, feeds=False).cmd_until == "done"


def test_the_page_polls_one_state_at_a_time():
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "if(TICK_BUSY)return;TICK_BUSY=true;" in html and "finally{TICK_BUSY=false}" in html


# ---- the tk console --------------------------------------------------------------------------------------------------------
def test_toast_accepted_then_final():
    tb = ToastBook()
    i = tb.add("fcu_mode HOLD")
    tb.accept(i, "intent accepted")
    t = tb.items[i]
    assert t.level == "acc" and not t.final and "accepted" in t.text
    tb.tick(time.monotonic() + 30)                                  # an accepted toast never turns amber "no answer yet"
    assert tb.items[i].level == "acc"
    tb.resolve(i, False, "ack FAILED state=MANUAL FCU refused")
    assert tb.items[i].level == "bad" and tb.items[i].final


class _Fake(BaseHTTPRequestHandler):
    polls = 0

    def log_message(self, *a):
        pass

    def _j(self, obj):
        b = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(b)))
        self.end_headers()
        self.wfile.write(b)

    def do_POST(self):
        self.rfile.read(int(self.headers.get("Content-Length") or 0))
        self._j({"ok": True, "final": False, "id": "42", "lines": ["intent accepted", "  ack ACCEPTED"]})

    def do_GET(self):
        _Fake.polls += 1
        self._j({"id": "42", "final": _Fake.polls >= 3, "ok": True, "lines": ["  ack DONE state=MANUAL"], "ms": {"final_ms": 812}})


def test_command_worker_follows_to_the_final():
    srv = HTTPServer(("127.0.0.1", 0), _Fake)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    w = CommandWorker(f"http://127.0.0.1:{srv.server_port}").start()
    got = {}
    ev = threading.Event()
    w.send("fcu_mode HOLD", lambda r: got.setdefault("first", r), on_final=lambda f: (got.setdefault("final", f), ev.set()))
    assert ev.wait(5), got
    assert got["first"]["final"] is False and got["final"]["final"] and got["final"]["ms"]["final_ms"] == 812
    w.stop()
    srv.shutdown()


def test_cam_worker_decodes_and_scales_off_the_main_thread():
    PIL = pytest.importorskip("PIL.Image")
    buf = io.BytesIO()
    PIL.new("RGB", (960, 540), (10, 120, 200)).save(buf, "JPEG")
    w = CamWorker("http://127.0.0.1:9")
    w.target = (480, 300)
    im = w.decode(buf.getvalue())
    assert not isinstance(im, bytes) and im.size[0] <= 480 and im.size[1] <= 300
    assert w.decode(b"not a jpeg") == b"not a jpeg"
