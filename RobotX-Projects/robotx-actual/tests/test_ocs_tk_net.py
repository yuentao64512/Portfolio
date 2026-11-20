"""K3: the tk console's network against a FakeOcs (http.server on a spare port serving the recorded fixtures, recording POST bodies).
No brokers, no Tk."""
from __future__ import annotations

import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

from optiprimex.tools.tkconsole.net import CommandWorker, Http, Poller

from .conftest import free_port

FIX = Path(__file__).parent / "fixtures"


class FakeOcs:
    """GET /state.json /fast.json /alarms.json /meta.json /course.json from a fixture triple; POST /cmd /ack /system recorded.
    mode: ok | slow (sleeps `delay` before answering) | err (500) - per path prefix via .modes."""

    def __init__(self, triple: str = "r5_task", cmd_delay: float = 0.0):
        t = json.loads((FIX / f"state_{triple}.json").read_text())
        self.bodies = {"/state.json": t["state"], "/fast.json": t["fast"], "/alarms.json": t["alarms"],
                       "/meta.json": json.loads((FIX / "state_meta.json").read_text()),
                       "/course.json": json.loads((FIX / "state_course.json").read_text()),
                       "/map.json": json.loads((FIX / "state_map.json").read_text()),
                       "/hosts.json": json.loads((FIX / "hosts_sample.json").read_text())}      # WP44: B's v0.1.86 contract
        self.posts, self.gets = [], []
        self.modes, self.delay, self.cmd_delay = {}, 0.0, cmd_delay
        self.in_flight = self.max_in_flight = 0
        self.lock = threading.Lock()
        fake = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def _json(self, obj, code=200):
                b = json.dumps(obj).encode()
                self.send_response(code)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(b)))
                self.end_headers()
                self.wfile.write(b)

            def do_GET(self):
                p = self.path.split("?")[0]
                fake.gets.append(p)
                m = fake.modes.get(p, "ok")
                if m == "slow":
                    time.sleep(fake.delay)
                if m == "err":
                    return self._json({"ok": False, "why": "boom"}, 500)
                return self._json(fake.bodies[p]) if p in fake.bodies else self._json({"why": "not found"}, 404)

            def do_POST(self):
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}")
                if self.headers.get("X-OCS-Web") != "1":
                    return self._json({"ok": False, "why": "missing X-OCS-Web header"}, 403)
                with fake.lock:
                    fake.in_flight += 1
                    fake.max_in_flight = max(fake.max_in_flight, fake.in_flight)
                fake.posts.append((self.path, body))
                try:
                    m = fake.modes.get(self.path, "ok")
                    time.sleep(fake.delay if m == "slow" else fake.cmd_delay)
                    if m == "err":
                        return self._json({"ok": False, "why": "boom"}, 500)
                    if self.path == "/cmd":
                        line = body.get("line", "")
                        ok = not line.startswith("start_task")
                        return self._json({"t": time.time(), "line": line, "ok": ok,
                                           "lines": ["intent accepted", "  ack DONE      state=AUTO_HOLD"] if ok else
                                           ["intent refused: START_TASK is accepted in AUTO_HOLD only"]})
                    if self.path == "/ack":
                        return self._json({"ok": True, "acked": 1})
                    return self._json({"ok": True, "why": "started"})
                finally:
                    with fake.lock:
                        fake.in_flight -= 1

        self.port = free_port()
        self.srv = HTTPServer(("127.0.0.1", self.port), H)
        self.srv.daemon_threads = True
        self.url = f"http://127.0.0.1:{self.port}"
        self.th = threading.Thread(target=self.srv.serve_forever, kwargs={"poll_interval": 0.05}, daemon=True)
        self.th.start()

    def close(self):
        self.srv.shutdown()
        self.srv.server_close()


def wait(pred, s=3.0):
    t = time.time() + s
    while time.time() < t:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


def test_send_posts_the_line_and_the_answer_reaches_the_callback():
    f = FakeOcs()
    w = CommandWorker(f.url).start()
    got = []
    try:
        w.send("declare", got.append)
        assert wait(lambda: got)
        assert f.posts == [("/cmd", {"line": "declare"})]
        assert got[0]["ok"] and got[0]["lines"][0] == "intent accepted"
        w.send("start_task", got.append)                                  # a refusal is an answer, not an error
        assert wait(lambda: len(got) == 2) and not got[1]["ok"] and "refused" in got[1]["lines"][0]
        w.ack("vehicle FAULT EKF_BAD", got.append)
        w.ack(None, got.append)
        assert wait(lambda: len(got) == 4)
        assert f.posts[2:] == [("/ack", {"key": "vehicle FAULT EKF_BAD"}), ("/ack", {"key": "all"})]
    finally:
        assert w.stop()
        f.close()


def test_the_queue_serialises_commands():
    f = FakeOcs(cmd_delay=0.15)
    w = CommandWorker(f.url).start()
    got = []
    try:
        for word in ("declare", "enable_auto", "hold", "resume"):
            w.send(word, got.append)
        assert wait(lambda: len(got) == 4, 5)
        assert [b["line"] for _p, b in f.posts] == ["declare", "enable_auto", "hold", "resume"]
        assert f.max_in_flight == 1                                       # never two in flight
    finally:
        w.stop()
        f.close()


def test_a_timeout_a_500_and_a_dead_port_never_raise():
    f = FakeOcs()
    f.modes["/state.json"], f.delay = "slow", 1.6
    h = Http(f.url)
    t0 = time.time()
    body, why = h.get("/state.json")                                      # 1 s timeout
    assert body is None and "timeout" in why.lower() and time.time() - t0 < 1.5
    f.modes["/state.json"] = "err"
    body, why = h.get("/state.json")
    assert body is None and why == "HTTP 500"
    f.modes["/cmd"] = "err"
    got = []
    w = CommandWorker(f.url).start()
    w.send("declare", got.append)
    assert wait(lambda: got) and not got[0]["ok"] and got[0]["lines"][0].startswith("no answer: HTTP 500")
    dead = CommandWorker(f"http://127.0.0.1:{free_port()}").start()
    dead.send("declare", got.append)
    assert wait(lambda: len(got) == 2) and got[1]["lines"][0].startswith("no answer:")
    f.modes["/cmd"], f.delay = "slow", 0.6
    slow = CommandWorker(f.url, answer_s=0.3).start()                     # the answer timeout, past the connect
    slow.send("declare", got.append)
    assert wait(lambda: len(got) == 3) and "timeout" in got[2]["lines"][0].lower()
    for x in (w, dead, slow):
        assert x.stop()
    f.close()


def test_the_poller_counts_misses_and_recovers():
    f = FakeOcs("r2")
    seen = {"state": [], "fast": 0, "meta": 0, "course": 0}
    p = Poller(f.url, fast_hz=10, slow_period_s=0.1,
               on_state=lambda s, n, why: seen["state"].append((s is not None, n)),
               on_fast=lambda x: seen.__setitem__("fast", seen["fast"] + (x is not None)),
               on_meta=lambda m: seen.__setitem__("meta", seen["meta"] + 1),
               on_course=lambda c: seen.__setitem__("course", seen["course"] + 1)).start()
    try:
        assert wait(lambda: len(seen["state"]) >= 2 and seen["fast"] >= 3 and seen["meta"] and seen["course"])
        assert seen["state"][0] == (True, 0)
        f.modes["/state.json"] = "err"
        assert wait(lambda: seen["state"][-1][1] >= 3)                    # three misses in a row -> the model says DOWN
        f.modes["/state.json"] = "ok"
        assert wait(lambda: seen["state"][-1] == (True, 0))               # recovers without a restart
    finally:
        assert p.stop()
        f.close()
    assert not p.thread.is_alive()
