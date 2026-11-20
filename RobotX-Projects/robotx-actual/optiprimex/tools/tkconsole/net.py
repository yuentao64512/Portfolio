"""K3: the network - every HTTP call in a worker thread, never in a Tk handler.

Poller: one thread; GET /state.json and /alarms.json at 1 Hz, /fast.json at --fast-hz; /meta.json and /course.json once and then
every 30 s. Timeout 1 s per request; a miss never raises - it is counted (misses: consecutive failed /state.json polls) and reported.
CommandWorker: one thread and a FIFO queue; send(line, cb) -> POST /cmd {"line"} -> cb(result); /ack and /system go through the same
queue, so the console never has two commands in flight (the backend's busy lock would answer "busy: ..." - shown, never retried).
/cmd is synchronous on the backend (it waits for the acks, up to ~20 s for stop_reset): the connect has the 1 s timeout, the answer
gets cmd_timeout_s (default 25 s); the UI turns a toast amber at 10 s on its own clock.

Results are handed over through `deliver(fn, *args)` - in the app a queue that the Tk main loop drains (views.App.pump); in the tests
a direct call. Nothing here imports tkinter."""
from __future__ import annotations

import http.client
import json
import queue
import threading
import time
from typing import Callable, Optional, Tuple
from urllib.parse import urlsplit

GET_TIMEOUT_S = 1.0
CMD_CONNECT_S = 1.0
CMD_TIMEOUT_S = 25.0
HEADERS = {"Content-Type": "application/json", "X-OCS-Web": "1"}     # ocs_web refuses a POST without X-OCS-Web (cross-site guard)


class Http:
    def __init__(self, base: str):
        u = urlsplit(base if "://" in base else "http://" + base)
        self.host, self.port = u.hostname or "127.0.0.1", u.port or 80
        self.base = f"http://{self.host}:{self.port}"

    def get(self, path: str, timeout: float = GET_TIMEOUT_S) -> Tuple[Optional[dict], str]:
        """(body, "") or (None, why) - never raises."""
        c = http.client.HTTPConnection(self.host, self.port, timeout=timeout)
        try:
            c.request("GET", path, headers={"Cache-Control": "no-store"})
            r = c.getresponse()
            data = r.read()
            if r.status != 200:
                return None, f"HTTP {r.status}"
            return json.loads(data), ""
        except Exception as exc:                                   # timeout, refused, reset, bad json: a miss
            return None, f"{type(exc).__name__}: {exc}" if str(exc) else type(exc).__name__
        finally:
            c.close()

    def post(self, path: str, body: dict, connect_s: float = CMD_CONNECT_S, answer_s: float = CMD_TIMEOUT_S) -> Tuple[Optional[dict], str]:
        c = http.client.HTTPConnection(self.host, self.port, timeout=connect_s)
        try:
            c.connect()
            c.sock.settimeout(answer_s)                            # connected in 1 s; the acks may take longer
            c.request("POST", path, body=json.dumps(body).encode(), headers=HEADERS)
            r = c.getresponse()
            data = r.read()
            try:
                obj = json.loads(data)
            except ValueError:
                obj = None
            if r.status != 200:
                why = (obj or {}).get("why") if isinstance(obj, dict) else None
                return None, f"HTTP {r.status}" + (f": {why}" if why else "")
            return obj, ""
        except Exception as exc:
            return None, f"{type(exc).__name__}: {exc}" if str(exc) else type(exc).__name__
        finally:
            c.close()


def _direct(fn, *a):
    fn(*a)


class Poller:
    """on_state(state|None, misses, why) · on_fast(fast|None) · on_alarms(alarms|None) · on_meta(meta) · on_course(course) ·
    on_map(map.json|None) when given."""

    def __init__(self, base: str, fast_hz: float = 5.0, deliver: Callable = _direct, slow_period_s: float = 1.0,
                 rare_period_s: float = 30.0, **cbs):
        self.http = Http(base)
        self.fast_period = 1.0 / max(0.5, min(float(fast_hz), 20.0))
        self.slow_period, self.rare_period = slow_period_s, rare_period_s
        self.deliver, self.cbs = deliver, cbs
        self.misses = 0
        self.stop_ev = threading.Event()
        self.hosts_on = threading.Event()                        # WP44: /hosts.json only while the HOSTS tab is visible
        self.thread = threading.Thread(target=self._run, name="ocs_tk-poller", daemon=True)

    def start(self) -> "Poller":
        self.thread.start()
        return self

    def stop(self, join_s: float = 2.0) -> bool:
        self.stop_ev.set()
        if self.thread.is_alive():
            self.thread.join(join_s)
        return not self.thread.is_alive()

    def _cb(self, name, *a):
        fn = self.cbs.get(name)
        if fn:
            self.deliver(fn, *a)

    def _run(self) -> None:
        t_fast = t_slow = t_rare = 0.0
        have_meta = have_course = False
        while not self.stop_ev.is_set():
            now = time.monotonic()
            if now >= t_rare or not (have_meta and have_course):
                t_rare = now + (self.rare_period if (have_meta and have_course) else 2.0)
                m, _ = self.http.get("/meta.json")
                if m is not None:
                    have_meta = True
                    self._cb("on_meta", m)
                c, _ = self.http.get("/course.json")
                if c is not None:
                    have_course = True
                    self._cb("on_course", c)
            if now >= t_slow:
                t_slow = now + self.slow_period
                s, why = self.http.get("/state.json")
                self.misses = 0 if s is not None else self.misses + 1
                self._cb("on_state", s, self.misses, why)
                a, _ = self.http.get("/alarms.json")
                self._cb("on_alarms", a)
                if self.hosts_on.is_set() and "on_hosts" in self.cbs:   # WP44 (B v0.1.86): the hosts feed, 600 samples per host
                    hb, why = self.http.get("/hosts.json", timeout=1.5)
                    self._cb("on_hosts", hb, why)
                if "on_map" in self.cbs:                             # B v0.1.83.5: the boat's map digest (the MAP tab)
                    mp, _ = self.http.get("/map.json")
                    self._cb("on_map", mp)
            if now >= t_fast:
                t_fast = now + self.fast_period
                f, _ = self.http.get("/fast.json")
                self._cb("on_fast", f)
            self.stop_ev.wait(max(0.01, min(t_fast, t_slow) - time.monotonic()))


class CamWorker:
    """The CAMERA tab's own thread (B v0.1.83.2 contract): while `active` (the tab is visible) it asks /cam.json until the stream
    answers (on_cam(cam)), pulls <url>snapshot.jpg at `fps` (on_frame(jpeg bytes) - decoded on the main thread's side) and
    /recording.json every `rec_period_s` (it runs an ssh on the backend: own timeout, never the 1 s poller, never the command queue)."""

    def __init__(self, base: str, deliver: Callable = _direct, fps: float = 10.0, rec_period_s: float = 2.0, rec_timeout_s: float = 8.0,
                 **cbs):
        # v0.1.86.9 (the venue analysis C6): 10 fps (was 4), one kept-alive connection to the stream, an unchanged frame skipped, and
        # the JPEG decoded + scaled HERE to `target` (the picture's size, set by the main thread) - on_frame gets a PIL image (bytes
        # without Pillow); the main thread only wraps it in a PhotoImage.
        self.target = (640, 360)
        self._conn = None
        self._conn_key = None
        self._last_sig = None
        self.http = Http(base)
        self.deliver, self.cbs = deliver, cbs
        self.period, self.rec_period, self.rec_timeout = 1.0 / max(0.5, fps), rec_period_s, rec_timeout_s
        self.active = threading.Event()
        self.stop_ev = threading.Event()
        self.cam: Optional[dict] = None
        self.thread = threading.Thread(target=self._run, name="ocs_tk-cam", daemon=True)

    def start(self) -> "CamWorker":
        self.thread.start()
        return self

    def stop(self, join_s: float = 2.0) -> bool:
        self.stop_ev.set()
        self.active.set()
        if self.thread.is_alive():
            self.thread.join(join_s)
        return not self.thread.is_alive()

    def show(self, on: bool) -> None:
        if on:
            self.active.set()
        else:
            self.active.clear()

    def _cb(self, name, *a):
        fn = self.cbs.get(name)
        if fn:
            self.deliver(fn, *a)

    def _snapshot(self, url: str) -> Tuple[Optional[bytes], str]:
        u = urlsplit(url if "://" in url else "http://" + url)
        key = (u.hostname or "127.0.0.1", u.port or 80)
        for attempt in (0, 1):                                       # a kept-alive connection the server closed: one fresh retry
            if self._conn is None or self._conn_key != key:
                self._close_conn()
                self._conn, self._conn_key = http.client.HTTPConnection(key[0], key[1], timeout=GET_TIMEOUT_S), key
            try:
                self._conn.request("GET", (u.path.rstrip("/") or "") + f"/snapshot.jpg?t={time.time():.3f}")
                r = self._conn.getresponse()
                data = r.read()
                if r.getheader("Connection", "").lower() == "close" or r.version < 11:
                    self._close_conn()
                return (data, "") if r.status == 200 and data[:2] == b"\xff\xd8" else (None, f"HTTP {r.status}")
            except Exception as exc:
                self._close_conn()
                if attempt:
                    return None, type(exc).__name__
        return None, "unreachable"

    def _close_conn(self) -> None:
        if self._conn is not None:
            try:
                self._conn.close()
            except Exception:
                pass
        self._conn = None

    def decode(self, jpg: bytes):
        """The JPEG -> a PIL image no larger than `target` (draft mode first: a 960 px JPEG decodes at half size when that is enough);
        the bytes themselves when Pillow is missing (the main thread then says so)."""
        try:
            from PIL import Image
            import io
        except ImportError:
            return jpg
        try:
            im = Image.open(io.BytesIO(jpg))
            w, h = self.target
            im.draft("RGB", (w, h))
            im.thumbnail((w, h))
            im.load()
            return im
        except Exception:
            return jpg

    def _run(self) -> None:
        t_cam = t_rec = 0.0
        while not self.stop_ev.is_set():
            if not self.active.is_set():
                self.active.wait(0.5)
                t_cam = 0.0                                          # re-ask the stream when the tab opens again
                continue
            now = time.monotonic()
            if now >= t_cam and not (self.cam or {}).get("ok"):
                t_cam = now + 3.0
                c, why = self.http.get("/cam.json", timeout=4.0)     # cam_status tries each address with 1 s
                self.cam = c or {"ok": False, "why": why, "url": ""}
                self._cb("on_cam", self.cam)
            if now >= t_rec:
                t_rec = now + self.rec_period
                r, why = self.http.get("/recording.json", timeout=self.rec_timeout)
                self._cb("on_rec", r or {"ok": False, "recording": False, "text": f"no answer: {why}"})
            if (self.cam or {}).get("ok") and self.cam.get("url"):
                jpg, why = self._snapshot(self.cam["url"])
                if jpg is None:
                    self.cam = dict(self.cam, ok=False, why=f"snapshot: {why}")
                    self._cb("on_cam", self.cam)
                else:
                    sig = (len(jpg), jpg[-64:])
                    if sig != self._last_sig:                        # the stream's latest frame unchanged: nothing to draw
                        self._last_sig = sig
                        self._cb("on_frame", self.decode(jpg))
            self.stop_ev.wait(max(0.0, self.period - (time.monotonic() - now)))
        self._close_conn()


class CommandWorker:
    """The one command path. send(line, cb) / ack(key, cb) / system(body, cb); cb(result: dict) with result = the backend's answer
    or {"ok": False, "lines": ["no answer: <why>"], "error": why}."""

    def __init__(self, base: str, deliver: Callable = _direct, answer_s: float = CMD_TIMEOUT_S):
        self.http = Http(base)
        self.deliver, self.answer_s = deliver, answer_s
        self.q: "queue.Queue" = queue.Queue()
        self.thread = threading.Thread(target=self._run, name="ocs_tk-cmd", daemon=True)
        self.sent = 0

    def start(self) -> "CommandWorker":
        self.thread.start()
        return self

    def stop(self, join_s: float = 2.0) -> bool:
        self.q.put(None)
        if self.thread.is_alive():
            self.thread.join(join_s)
        return not self.thread.is_alive()

    def pending(self) -> int:
        return self.q.qsize()

    def send(self, line: str, cb: Optional[Callable] = None, on_final: Optional[Callable] = None) -> None:
        """cb(the /cmd answer); v0.1.86.9: when that answer is an ACCEPTED with the DONE / FAILED still to come ("final": false),
        on_final(the final outcome) follows from GET /cmd_result?id= (polled in its own thread: the queue goes on)."""
        self.q.put(("/cmd", {"line": line}, cb, None, on_final))

    def ack(self, key: Optional[str], cb: Optional[Callable] = None) -> None:
        self.q.put(("/ack", {"key": key} if key else {"key": "all"}, cb))

    def system(self, body: dict, cb: Optional[Callable] = None) -> None:
        self.q.put(("/system", body, cb))

    def post(self, path: str, body: dict, cb: Optional[Callable] = None, answer_s: Optional[float] = None) -> None:
        """Any other POST of the contract (B v0.1.83.2: /recording, /logging) - the same queue, the same answer rules; answer_s = a longer
        wait for an endpoint that does more (B v0.1.84.4: /recording + depth waits up to 20 s for the first pair)."""
        self.q.put((path, body, cb, answer_s))

    def _run(self) -> None:
        while True:
            job = self.q.get()
            if job is None:
                return
            path, body, cb = job[:3]
            wait = job[3] if len(job) > 3 and job[3] else self.answer_s
            r, why = self.http.post(path, body, answer_s=wait)
            self.sent += 1
            if r is None:
                r = {"ok": False, "lines": [f"no answer: {why}"], "error": why}
            if cb:
                self.deliver(cb, r)
            on_final = job[4] if len(job) > 4 else None
            if on_final and r.get("final") is False and r.get("id"):
                threading.Thread(target=self._follow, args=(str(r["id"]), on_final), name="ocs_tk-follow", daemon=True).start()

    FOLLOW_PERIOD_S = 0.15

    def _follow(self, rid: str, on_final: Callable) -> None:
        t0 = time.monotonic()
        while time.monotonic() - t0 < self.answer_s:
            f, _why = self.http.get(f"/cmd_result?id={rid}", timeout=2.0)
            if isinstance(f, dict) and f.get("final"):
                self.deliver(on_final, f)
                return
            time.sleep(self.FOLLOW_PERIOD_S)
        self.deliver(on_final, {"ok": False, "final": True, "lines": [f"no DONE / FAILED within {self.answer_s:.0f} s (see the LOG)"]})
