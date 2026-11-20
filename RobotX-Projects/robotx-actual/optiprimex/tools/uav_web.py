"""uav_web: the UAV window (v0.1.65, D16 of docs/OVERNIGHT_2026-11-07_UAV.md) - a monitoring page for the drone, http://127.0.0.1:8099/.

Dmitrii 2026-11-07: "a monitoring page for the drone ... and the data on the OCS page as for the boat". Read-only: it reads the hub
(`status/uav`, `event/uav/#`, `ocs/state`, `ocs/alert`, `presence/uav_adapter`, `status/usv` for the boat's glyph) and paints, in the
boat window's idiom (a plate, blocks, no fabricated value - hatched *unknown*, dashed *unavailable by design*):
  plate      LINK · STATE · MODE · ARMED · DECLARED · PILOT READY
  TELEMETRY  HAE altitude, speed, heading, flight phase, position, telemetry age, FCU fake | real
  FENCE      the FCU polygon (n points) + ceiling vs the declared one (✓ matches / ✗ differs / change pending), the last re-declaration,
             fence_rejected sent at ...
  ROBOCOMMAND the course forwarded + acked, its fingerprint; the last command forwarded; RunStart; reports n / last; heartbeats n · age;
             rate_dropped
  MAP        the course, the UAV transit area, the FCU fence, the declared fence (dashed), the UAV (a triangle), the USV (a boat glyph)
  EVENTS     event/uav/#, the forwards, the UAV alarms
  HELP       docs/UAV_TEAM_CHECKLIST.md
The only things it sends are four whitelisted OCS lines (UAV_LINES, through ocs_cli on the hub - the OCS app's authority decides):
`uav ready on|off`, `declare uav on|off`, `uav task none|1..4`, `uav resend_course`. The window_control
lock of v0.1.63 applies (one commander, take over at once, a PIN for other computers). NOTHING here commands the drone."""
from __future__ import annotations

import argparse
import collections
import json
import os
import socket
import socketserver
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Optional

import paho.mqtt.client as mqtt

HERE = os.path.dirname(os.path.abspath(__file__))
KIT = os.path.dirname(os.path.dirname(HERE))
NS = "team/optiprime/v1"
UAV_LINES = ("uav ready on", "uav ready off", "declare uav on", "declare uav off", "uav task none", "uav task 1", "uav task 2",
             "uav task 3", "uav task 4", "uav resend_course", "uav scan done")     # v0.1.66: scan done (m1t2)
TEL_CAUTION_S, TEL_ALARM_S = 2.0, 5.0                  # D18: UAV TELEMETRY > 2 s CAUTION, > 5 s ALARM (only while declared)


def _env(payload: bytes):
    try:
        d = json.loads(payload)
    except ValueError:
        return None
    return d if isinstance(d, dict) else None


class Model:
    """What the hub says about the UAV, with the time each piece arrived (monotonic)."""

    def __init__(self, uav_tok: str = "uav", clock=time.monotonic):
        self.tok, self.clock = uav_tok, clock
        self.lock = threading.Lock()
        self.status: Optional[dict] = None
        self.status_t: Optional[float] = None
        self.ocs: Optional[dict] = None
        self.ocs_t: Optional[float] = None
        self.usv: Optional[dict] = None
        self.usv_t: Optional[float] = None
        self.presence: Optional[dict] = None
        self.events = collections.deque(maxlen=60)
        self.cmds = collections.deque(maxlen=20)

    def topics(self):
        return [f"{NS}/status/{self.tok}", f"{NS}/event/{self.tok}/#", f"{NS}/ack/{self.tok}", "ocs/state", "ocs/alert",
                f"{NS}/status/usv", "presence/uav_adapter"]

    def on_message(self, topic: str, payload: bytes, now: Optional[float] = None) -> None:
        now = self.clock() if now is None else now
        d = _env(payload)
        if d is None:
            return
        with self.lock:
            if topic == f"{NS}/status/{self.tok}":
                self.status, self.status_t = d.get("body") or {}, now
            elif topic == "ocs/state":
                self.ocs, self.ocs_t = d.get("body") or {}, now
            elif topic == f"{NS}/status/usv":
                self.usv, self.usv_t = d.get("body") or {}, now
            elif topic == "presence/uav_adapter":
                self.presence = dict(d, _t=now)
            elif topic.startswith(f"{NS}/event/{self.tok}/"):
                b = d.get("body") or {}
                kind = b.get("event")
                if kind == "fence_change":
                    txt = f"fence change {b.get('id')}: {max(0, len(b.get('vertices') or []) - 1)} points, ceiling {b.get('ceiling_hae_m')} m"
                elif kind == "uav_report":
                    txt = f"report {b.get('kind')} {json.dumps(b.get('body'))[:80]}"
                else:
                    txt = f"{kind} {json.dumps(b)[:80]}"
                self.events.appendleft({"t": time.time(), "level": "info", "text": txt})
            elif topic == f"{NS}/ack/{self.tok}":
                b = d.get("body") or {}
                if b.get("result") in ("FAILED", "REJECTED"):
                    self.events.appendleft({"t": time.time(), "level": "warn", "text": f"{b.get('result')}: {b.get('reason') or ''}"[:160]})
            elif topic == "ocs/alert":
                b = d.get("body") or {}
                k = str(b.get("kind") or "")
                if "uav" in k or k in ("rate_dropped", "foreign_declaration") or b.get("vehicle") == (self.status or {}).get("uav", {}).get("vehicle_id"):
                    lvl = "alarm" if k in ("declaration_refused_uav", "uav_forward_failed", "foreign_declaration") else "warn"
                    self.events.appendleft({"t": time.time(), "level": lvl, "text": f"{k}: {b.get('reason') or b.get('text') or b.get('action') or ''}"[:200]})

    # ---- the blocks -------------------------------------------------------------------------------------------------------
    def snapshot(self, now: Optional[float] = None) -> dict:
        now = self.clock() if now is None else now
        with self.lock:
            st = dict(self.status or {})
            st_age = None if self.status_t is None else now - self.status_t
            o = dict(self.ocs or {})
            ocs_age = None if self.ocs_t is None else now - self.ocs_t
            usv = dict(self.usv or {})
            usv_age = None if self.usv_t is None else now - self.usv_t
            events = list(self.events)
            cmds = list(self.cmds)
        u = dict(o.get("uav") or {})
        h, f, rc = st.get("health") or {}, st.get("fence") or {}, st.get("robocommand") or {}
        tel_age = h.get("telemetry_age_s")
        link = ("down" if st_age is None or st_age > 3.0 or st.get("state") == "OFFLINE" else
                "stale" if tel_age is None or tel_age > TEL_ALARM_S else "slow" if tel_age > TEL_CAUTION_S else "up")
        decl = bool(u.get("declared"))
        alarms = []
        if (decl or u.get("declare")) and link != "up":
            if link == "down":
                alarms.append({"level": "alarm", "text": "UAV adapter gone (no status/uav): the window / OCS hear nothing from the drone"})
            elif tel_age is None or tel_age > TEL_ALARM_S:
                alarms.append({"level": "alarm", "text": "UAV heartbeat stopped - Remote ID (telemetry %s)" % ("none" if tel_age is None else "%.0f s old" % tel_age)})
            else:
                alarms.append({"level": "warn", "text": "UAV telemetry %.1f s old" % tel_age})
        if f.get("matches_declared") is False:
            alarms.append({"level": "warn", "text": "the FCU fence differs from the declared one"})
        if u.get("last_refusal"):
            alarms.append({"level": "alarm", "text": "DECLARATION REFUSED (UAV): " + str(u["last_refusal"].get("reason"))})
        if int(o.get("rate_dropped") or 0) > 0:
            alarms.append({"level": "warn", "text": "RATE: %d message(s) dropped by the 5 msg/s cap" % int(o["rate_dropped"])})
        hb = ((o.get("reporting") or {}).get("hb") or {}).get(u.get("vehicle_id") or "UAV1") or {}
        course = ((o.get("judge_view") or {}).get("course") or {})
        return {
            "t": time.time(),
            "plate": {"link": link, "state": st.get("state") if st else None, "mode": st.get("mode"), "armed": st.get("armed"),
                      "declared": decl, "declare": bool(u.get("declare")), "pilot_ready": h.get("pilot_ready"),
                      "robot_state": st.get("robot_state"), "current_task": st.get("current_task")},
            "telemetry": {"alt_hae_m": st.get("altitude_hae_m"), "alt_source": st.get("altitude_source"), "sog_mps": st.get("sog_mps"),
                          "hdg_deg": st.get("hdg_deg"), "phase": st.get("flight_phase"), "lat": st.get("lat"), "lon": st.get("lon"),
                          "age_s": tel_age, "n": h.get("telemetry_n"), "fcu_link": st.get("fcu_link"), "status_age_s": st_age},
            "fence": {"points_n": f.get("points_n"), "ceiling_hae_m": f.get("ceiling_hae_m"), "matches": f.get("matches_declared"),
                      "pending": f.get("change_pending"), "last_result": f.get("last_result") or u.get("last_fence_change"),
                      "rejected_sent": f.get("rejected_sent"), "ok": h.get("fence_ok"),
                      "declared_n": max(0, len(u.get("declared_fence") or []) - 1) if u.get("declared_fence") else None,
                      "declared_ceiling": u.get("declared_ceiling_hae_m"), "redeclared": u.get("redeclared")},
            "robocommand": {"course_id": course.get("course_id"), "course_fwd": u.get("course_forward"), "course_fp": rc.get("course_fp") or u.get("course_fp"),
                            "last_command": u.get("last_command") or rc.get("last_forward"), "run_id": o.get("run_id"), "r": o.get("r"),
                            "reports_n": rc.get("reports_n"), "last_report": rc.get("last_report"), "hb": hb,
                            "rate_dropped": o.get("rate_dropped"), "judge": o.get("judge"), "judge_connected": o.get("judge_connected")},
            "uav": {"vehicle_id": u.get("vehicle_id") or (st.get("uav") or {}).get("vehicle_id"), "registration": u.get("registration"),
                    "pilot": u.get("pilot_callsign"), "configured": u.get("configured"), "adapter": (st.get("listener") or {})},
            "map": map_data(st, o, usv, usv_age),
            "alarms": alarms, "events": events[:30], "cmds": cmds[:10], "ocs_age_s": ocs_age,
            "uav_events": st.get("events") or [],
        }


def map_data(st: dict, o: dict, usv: dict, usv_age: Optional[float]) -> dict:
    """Lat / lon lists for the page's SVG: the course + transit area (ocs/state's judge view or the declaration's course), the FCU fence,
    the declared fence, the UAV and the USV."""
    jv = (o.get("judge_view") or {})
    u = o.get("uav") or {}
    c = u.get("course") or {}                                  # ocs/state's uav block carries the received course (v0.1.65)
    return {"course": c.get("boundary") or [], "transit": c.get("uav_transit_area") or [],
            "fence": (st.get("fence") or {}).get("points") or [], "declared": u.get("declared_fence") or [],
            "uav": [st.get("lat"), st.get("lon"), st.get("hdg_deg")] if st.get("lat") is not None else None,
            "usv": [usv.get("lat"), usv.get("lon"), usv.get("hdg_deg")] if usv.get("lat") is not None and (usv_age or 99) < 5 else None,
            "course_id": (jv.get("course") or {}).get("course_id")}


def ocs_line(hub: str, line: str) -> dict:
    """Only UAV_LINES travel - through ocs_cli on the hub (the OCS app's authority decides, the intent answers)."""
    line = " ".join(str(line or "").split())
    if line not in UAV_LINES:
        return {"ok": False, "text": "refused by the page: only " + " | ".join(UAV_LINES), "line": line}
    from .boat_web import ocs_mqtt_line
    return ocs_mqtt_line(hub, line, vehicle="usv", vehicle_id="USV1")


def checklist_text() -> str:
    try:
        with open(os.path.join(KIT, "docs", "UAV_TEAM_CHECKLIST.md"), encoding="utf-8") as fh:
            return fh.read()
    except OSError:
        return "docs/UAV_TEAM_CHECKLIST.md not found"


def make_handler(model: Model, hub: str, control=None, where: Optional[dict] = None, ocs_url: Optional[str] = None, send=ocs_line):
    from .window_control import Control
    control = control or Control()

    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _who(self):
            return self.headers.get("X-Window-Session"), self.headers.get("X-Window-Name"), self.client_address[0]

        def _send(self, code, body, ctype):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                with open(os.path.join(HERE, "uav_web.html"), "rb") as fh:
                    return self._send(200, fh.read(), "text/html; charset=utf-8")
            if path == "/themes.css":
                with open(os.path.join(HERE, "themes.css"), "rb") as fh:
                    return self._send(200, fh.read(), "text/css; charset=utf-8")
            if path == "/checklist.txt":
                return self._send(200, checklist_text().encode(), "text/plain; charset=utf-8")
            if path == "/prefs":
                from . import prefs as P
                t = P.get_theme(ocs_url) if ocs_url else None
                r = {"theme": t, "from": "ocs"} if t is not None else P.read_shared()
                return self._send(200, json.dumps({"ok": True, "prefs": {"theme": r["theme"]} if r["theme"] else {}, "from": r["from"]}).encode(),
                                  "application/json")
            if path == "/state.json":
                sid, name, addr = self._who()
                control.seen(sid, name, addr)
                snap = model.snapshot()
                snap["control"] = dict(control.view(sid), where=where or {})
                return self._send(200, json.dumps(snap, default=str).encode(), "application/json")
            self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                req = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                req = {}
            ours = self.headers.get("X-Uav-Web") == "1"
            if ours and self.path == "/prefs":
                from . import prefs as P
                t = P.norm(req.get("theme"))
                ok = bool(t) and bool(ocs_url) and P.post_theme(ocs_url, t, "X-OCS-Web")
                return self._send(200, json.dumps({"ok": True, "prefs": {"theme": t}, "to": "ocs"} if ok else P.write_shared(req.get("theme"))).encode(),
                                  "application/json")
            sid, name, addr = self._who()
            if ours and self.path == "/control":
                op = str(req.get("op") or "")
                ok, why = control.take(sid, name, addr, req.get("pin")) if op == "take" else control.release(sid) if op == "release" else (False, "op: take | release")
                return self._send(200, json.dumps({"ok": ok, "text": why, "control": control.view(sid)}).encode(), "application/json")
            if not ours or self.path != "/ocs":
                return self._send(403, json.dumps({"ok": False, "text": "refused (needs POST /ocs with X-Uav-Web: 1)"}).encode(), "application/json")
            line = " ".join(str(req.get("line") or "").split())
            if line not in UAV_LINES:                                # the whitelist first, before the lock and any connection
                return self._send(200, json.dumps({"ok": False, "text": "refused by the page: only " + " | ".join(UAV_LINES)}).encode(), "application/json")
            ok, why = control.may_command(sid, name, addr)
            if not ok:
                return self._send(200, json.dumps({"ok": False, "text": why, "control": control.view(sid)}).encode(), "application/json")
            out = send(hub, line)
            with model.lock:
                model.cmds.appendleft({"t": time.time(), "line": line, "ok": out.get("ok"), "text": out.get("text")})
            self._send(200, json.dumps(out).encode(), "application/json")
    return H


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = str(self.server_address[0]), int(self.server_address[1])


def start_mqtt(model: Model, hub: str) -> mqtt.Client:
    host, _, port = hub.rpartition(":")
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"uav-web-{os.getpid()}")
    c.on_connect = lambda cl, u, fl, rc, pr=None: [cl.subscribe(t, 0) for t in model.topics()]
    c.on_message = lambda cl, u, msg: model.on_message(msg.topic, msg.payload)
    c.reconnect_delay_set(1, 5)
    c.connect_async(host or "127.0.0.1", int(port or 1882), keepalive=15)
    c.loop_start()
    return c


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="the UAV window (8099): read-only, four whitelisted OCS lines; nothing commands the drone")
    ap.add_argument("--hub", default=os.environ.get("UAV_WEB_HUB", "127.0.0.1:1882"), help="the hub broker HOST:PORT")
    ap.add_argument("--port", type=int, default=int(os.environ.get("UAV_WEB_PORT", 8099)))
    ap.add_argument("--bind", default=os.environ.get("UAV_WEB_BIND", "127.0.0.1"), help="127.0.0.1, or the team-LAN address (never 0.0.0.0)")
    ap.add_argument("--pin", default=os.environ.get("UAV_WINDOW_PIN"), help="the PIN for commands from another computer")
    ap.add_argument("--ocs-url", default=os.environ.get("OCS_URL", "http://127.0.0.1:%s/" % os.environ.get("OCS_WEB_PORT", "8096")))
    a = ap.parse_args(argv)
    from .window_control import Control, new_pin
    pin = a.pin
    if not pin and a.bind not in ("127.0.0.1", "::1", "localhost"):
        pin = new_pin()
        print(f"uav window: commands from other computers need the PIN {pin}", flush=True)
    model = Model()
    cl = start_mqtt(model, a.hub)
    control = Control(pin)
    where = {"where": "mac", "host": socket.gethostname(), "url": f"http://{a.bind}:{a.port}/"}
    srv = Server((a.bind, a.port), make_handler(model, a.hub, control, where=where, ocs_url=a.ocs_url))

    def beat():                                         # ocs/uav_window 1 Hz: the OCS SYSTEM row "UAV · window" (as the USV window's)
        while True:
            v = control.view()
            try:
                cl.publish("ocs/uav_window", json.dumps(dict(where, commander=v["commander"], sessions=v["sessions"], pin=v["pin"])), qos=0)
            except Exception:
                pass
            time.sleep(1.0)
    threading.Thread(target=beat, name="uav-window-beat", daemon=True).start()
    print(f"uav window: http://{a.bind}:{a.port}/ (hub {a.hub}; read-only, the whitelisted OCS lines only)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        cl.loop_stop()
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
