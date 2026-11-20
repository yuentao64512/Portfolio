#!/usr/bin/env python3
"""bench_tui: the bench console, in the pool_test2 style (typed commands, boxed panels, usage tooltip, Tab screens).

`op-tui` or `ocs_cli tui`. Live from the hub (and the boat broker when reachable): the run ladder R0-R6, the vehicle
ladder V0-V6, task sub-state, FCU flags and position, the judge's view (fake RoboCommand stats), the log of intents,
emitted commands, acks, transitions, judge traffic and alerts. Commands are typed at the prompt with the exact
ocs_cli grammar; the COMMANDS panel marks what the playbook predicts will pass in the current state and the line
above the prompt shows the usage of the command being typed (or the obvious next step).

It sends intents only (ADR-15) and plant inputs only to the fake FCU's UDP control port (bench). The companion's own
curses TUI stays the safety console on the boat (decision 25).

Screens (Tab / Shift-Tab): MAIN  MISSION  BOAT  NETWORK  JUDGE  LOG  HELP.   Esc clears the line, Ctrl-L repaints, PgUp/PgDn
scroll the events. MAIN is the shared skeleton of both consoles (usv_ui on the boat draws the same rows): RUN | VEHICLE,
MISSION | WHERE WE ARE, FLOW, FROM THE JUDGE | NETWORK | WARNINGS, EVENTS | CONSOLE | COMMANDS. MISSION lists every
mission with its stages and how to select it from the current state (Up/Down, Enter puts the command in the prompt). BOAT is the boat's own view seen from shore (vehicle, position, the perception map as text, detector
health, who has the FCU - usv_ui or the mission manager); `map` opens the browser map (tools/map.html) on the hub's
websocket listener.
"""
from __future__ import annotations

import argparse
import collections
import curses
import json
import os
import shlex
import socket
import subprocess
import sys
import threading
import time
import webbrowser
from typing import Deque, Dict, List, Optional, Tuple

import paho.mqtt.client as mqtt

from .. import TEAM_NS, __version__
from ..companion.mqtt import start_presence
from . import ocs_cli, playbook
from .textmap import digest_layers, render_text_map

MAP_HTML = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map.html")

R_ORDER = playbook.R_ORDER
V_ORDER = playbook.V_ORDER
SCREENS = ("MAIN", "MISSION", "BOAT", "NETWORK", "JUDGE", "LOG", "HELP")
MIN_W, MIN_H = 120, 44          # the shared skeleton needs this much; spacers between the rows appear from 48 rows


# ---- model and feeds ------------------------------------------------------------------------------------------------
class Model:
    """Everything the screen shows; updated from MQTT threads under a lock, read by the UI loop."""

    def __init__(self, vehicle_id: str, vehicle: str, log_len: int = 600):
        self.vehicle_id, self.vehicle = vehicle_id, vehicle
        self.lock = threading.Lock()
        self.ocs: Optional[dict] = None
        self.ocs_t = 0.0
        self.status: Optional[dict] = None
        self.status_t = 0.0
        self.local: Optional[dict] = None
        self.local_t = 0.0
        self.stats: Optional[dict] = None
        self.bridge: Optional[bool] = None
        self.hub_up = False
        self.boat_up = False
        self.cmd_actions: Dict[str, str] = {}
        self.presence: Dict[Tuple[str, str], Tuple[dict, float]] = {}      # (broker label, node) -> (body, t_recv)
        self.sys: Dict[Tuple[str, str], str] = {}                            # (broker label, $SYS suffix) -> value
        self.clock_samples: Dict[str, Deque[float]] = {}   # v0.1.84.5: host -> the last 10 (t_recv - beat t_unix) of its presence beats
        self.clock_last: Dict[str, float] = {}             # host -> when its last sample came
        self.ui_results: Dict[str, dict] = {}              # v0.1.84.5: usv_ui's local/<v>/ui/cmd/result by id (the OCS's /logging)
        self.map: Optional[dict] = None          # the map digest in use (usv_ui's while it publishes, else the mission manager's)
        self.map_t = 0.0
        self.map_src = ""
        self.maps: Dict[str, Tuple[dict, float]] = {}   # source -> (body, t_recv): both programs may publish telemetry/<v>/map
        self.perc_health: Optional[dict] = None
        self.perc_t = 0.0
        self.perc_by: Dict[str, Tuple[dict, float]] = {}   # v0.1.43: detector -> (health, t): the sim mixes the camera path + the fake
        self.log: Deque[Tuple[float, str, str]] = collections.deque(maxlen=log_len)   # (t, kind, text)
        self.judge_log: Deque[Tuple[float, str]] = collections.deque(maxlen=200)
        self.counts = collections.Counter()
        self.alerts: Deque[Tuple[float, str, str]] = collections.deque(maxlen=50)     # (t, kind, text) from ocs/alert
        self.last_bad: Tuple[float, str] = (0.0, "")                                   # the last refused intent / REJECTED ack
        self.cond_since: Dict[str, float] = {}                                         # warning key -> first seen (for the age column)
        self.net_profile = os.environ.get("NET_PROFILE", "")

    def add(self, kind: str, text: str) -> None:
        with self.lock:
            self.log.append((time.time(), kind, text))
            self.counts[kind] += 1
            if kind == "judge":
                self.judge_log.append((time.time(), text))

    def on_sys_or_presence(self, broker: str, topic: str, payload: bytes) -> bool:
        """Shared by both feeds: $SYS counters and presence beats. Returns True when handled."""
        if topic.startswith("$SYS/broker/connection/"):
            return False
        if topic.startswith("$SYS/"):
            with self.lock:
                self.sys[(broker, topic[len("$SYS/broker/"):])] = payload.decode(errors="ignore").strip()
            return True
        if topic.startswith("presence/"):
            try:
                body = json.loads(payload)
            except Exception:
                return True
            node = topic[len("presence/"):]
            t_recv = time.time()
            tu = body.get("t_unix")
            host = str(body.get("host") or "")
            if isinstance(tu, (int, float)) and abs(t_recv - tu) <= 60.0 and host and body.get("up", True):
                with self.lock:                # v0.1.84.5: the clock offset of the beat's host (the min over 10 strips the latency)
                    self.clock_samples.setdefault(host, collections.deque(maxlen=10)).append(t_recv - float(tu))
                    self.clock_last[host] = t_recv
            if isinstance(tu, (int, float)) and t_recv - tu > 60.0:
                t_recv = float(tu)            # a retained beat from an earlier run, replayed by the broker on subscribe: already old
            with self.lock:
                prev = self.presence.get((broker, node))
                self.presence[(broker, node)] = (body, t_recv)
            if prev is not None and bool(prev[0].get("up")) != bool(body.get("up")):
                self.add("bridge", f"{node} on {broker}: {'up' if body.get('up') else 'DOWN (clean stop)'}")
            return True
        return False

    def on_hub(self, topic: str, payload: bytes) -> None:
        if self.on_sys_or_presence("hub", topic, payload):
            return
        try:
            env = json.loads(payload)
        except Exception:
            return
        body = env.get("body", env) if isinstance(env, dict) else env
        ns = TEAM_NS + "/"
        with self.lock:
            if topic.endswith("/telemetry/" + self.vehicle + "/map"):
                src = str(env.get("source") or "mission_manager") if isinstance(env, dict) else "mission_manager"
                now = time.time()
                self.maps[src] = (body, now)
                # usv_ui's digest wins while it is fresh (it carries points, cv counters and who has the FCU) - unless the mission
                # manager's holds detections (v0.1.29.1: usv_ui's map takes none in; at the pool it hid the buoys)
                ui, mm = self.maps.get("usv_ui"), self.maps.get("mission_manager")
                det = lambda b: bool((b or {}).get("entities") or (b or {}).get("n_ingested"))
                ui_live = ui is not None and now - ui[1] <= 3.0
                mm_live = mm is not None and now - mm[1] <= 3.0 and det(mm[0]) and not (ui_live and det(ui[0]))
                if mm_live:
                    pick = "mission_manager"
                elif ui is not None and now - ui[1] <= 3.0:
                    pick = "usv_ui"
                else:
                    pick = src
                if src == pick:
                    self.map, self.map_t, self.map_src = body, now, src
                return
            if topic == "ocs/state":
                self.ocs, self.ocs_t = body, time.time()
                return
            if topic.endswith(f"/status/{self.vehicle}"):
                self.status, self.status_t = body, time.time()
                return
            if topic == "rc/sim/stats":
                self.stats = env
                return
            if topic.startswith(ns + "cmd/"):
                self.cmd_actions[env.get("msg_id", "")] = f"{body.get('action')}{' ' + str(body.get('task')) if body.get('task') else ''}"
        if topic == "ocs/intent_result":
            self.add("intent", f"intent {body.get('result')}: {body.get('reason') or ''}".rstrip(": "))
            if body.get("result") not in ("accepted", "ACCEPTED"):
                self.last_bad = (time.time(), f"intent {body.get('result')}: {body.get('reason') or ''}")
        elif topic == "ocs/alert":
            text = f"ALERT {body.get('kind')} {_short({k: v for k, v in body.items() if k not in ('event', 'kind', 't')})}"
            self.add("alert", text)
            self.alerts.append((time.time(), str(body.get("kind")), text))
        elif topic.startswith(ns + "ack/"):
            act = self.cmd_actions.get(body.get("ack_of", ""), "?")
            self.add("ack", f"ack {body.get('result')} {act} -> {body.get('state')} {body.get('reason') or ''}".rstrip())
            if body.get("result") in ("REJECTED", "FAILED"):
                self.last_bad = (time.time(), f"{act} {body.get('result')}: {body.get('reason') or ''}")
        elif topic.startswith(ns + "cmd/"):
            origin = f" [{env.get('origin')}]" if env.get("origin") else ""
            self.add("cmd", f"cmd {body.get('action')} {body.get('task') or ''} {_short(body.get('args'))}{origin}".rstrip())
        elif topic.startswith(ns + "event/"):
            kind = topic.rsplit("/", 1)[-1]
            if kind == "task_state":
                who = body.get("layer") if body.get("layer") == body.get("machine") else f"{body.get('layer')} {body.get('machine')}"
                self.add("event", f"{who}: {body.get('from')} -> {body.get('to')} ({body.get('reason')})")
            elif kind == "scout" and body.get("ev") == "buoy":         # v0.1.79 (WP35): the scout's log lines, readable
                self.add("event", f"scout: {body.get('id')} {'logged' if body.get('visited') else 'blocked'} {body.get('lat')} {body.get('lon')}"
                         f" · {body.get('colour')} · {body.get('n_obs')} obs · {body.get('source')}" + (f" ({body['why']})" if body.get("why") else ""))
            elif kind == "detect" and body.get("ev") in ("seen", "lost", "back"):      # v0.1.79: m0t8
                self.add("event", f"detect: #{body.get('id')} {body.get('ev')} at "
                         f"{body.get('range_m', body.get('last_seen_m'))} m" + (f" (farthest {body.get('max_m')} m)" if body.get("ev") == "lost" else ""))
            else:
                self.add("event", f"event {kind} {_short({k: v for k, v in body.items() if k != 't'})}")
        elif topic == "rc/command":
            self.add("judge", f"judge -> {env.get('kind')} {_short({k: v for k, v in env.items() if k not in ('kind', 'team_id', 'sent_at')})}")
        elif topic.startswith("rc/report/"):
            if env.get("kind") != "heartbeat":
                self.add("judge", f"judge <- {env.get('kind')} {_short({k: v for k, v in env.items() if k not in ('kind', 'vehicle', 'report_seq')})}")
        elif topic == "rc/request":
            self.add("judge", f"judge <- {env.get('kind')} seq {env.get('declaration_seq')}")

    def on_boat(self, topic: str, payload: bytes) -> None:
        if topic.startswith("$SYS/broker/connection/"):
            with self.lock:
                self.bridge = payload.strip() == b"1"
            self.add("bridge", f"boat bridge {'up' if self.bridge else 'DOWN'}")
            return
        if self.on_sys_or_presence("boat", topic, payload):
            return
        try:
            env = json.loads(payload)
        except Exception:
            return
        if topic.endswith("/ui/cmd/result"):                      # v0.1.84.5: usv_ui's answer to the OCS's own word (log on / off)
            b = env.get("body", env) if isinstance(env, dict) else {}
            if isinstance(b, dict) and b.get("id"):
                with self.lock:
                    self.ui_results[str(b["id"])] = dict(b, t_recv=time.time())
            return
        with self.lock:
            if topic.endswith("/perception/health"):
                self.perc_health, self.perc_t = env.get("body", env), time.time()
                self.perc_by[str((self.perc_health or {}).get("detector") or "?")] = (self.perc_health, self.perc_t)
            else:
                self.local, self.local_t = env.get("body", env), time.time()

    def ui_state(self, now: Optional[float] = None) -> Optional[bool]:
        """Does usv_ui on the boat command the FCU? True = ACTIVE (ENABLE_AUTO refused until `handover`), False = handed over or
        not running (the mission manager may take it), None = the boat broker is not visible. The mission manager's own view
        (local/<v>/mm/state ui_active, 5 Hz) wins; usv_ui's presence + map digest is the fallback."""
        now = time.time() if now is None else now
        with self.lock:
            loc, loc_t, boat_up = self.local, self.local_t, self.boat_up
            pres = self.presence.get(("boat", "usv_ui"))
            ts = self.maps.get("usv_ui")
        if loc and now - loc_t < 3 and "ui_active" in loc:
            return bool(loc.get("ui_active"))
        ui_up = bool(pres) and bool(pres[0].get("up")) and now - pres[1] < 6.0
        if ui_up and ts and now - ts[1] < 3.0:
            return ts[0].get("task_state") == "ACTIVE"
        if ui_up:
            return True                                   # up but no digest yet: it starts ACTIVE
        return False if boat_up else None

    INFO_ALERTS = ("mission_started", "mission_ready", "mission_waiting", "run_start_stall")   # v0.1.59 (B3): the stall row is the CAUTION below

    def warnings(self, now: Optional[float] = None) -> List[Tuple[float, str, str]]:
        """The active conditions an operator must not miss: [(age_s, text, style)], oldest first. A condition disappears
        the moment it clears (links back, status fresh); a refusal stays 10 s, an OCS alert 20 s."""
        now = time.time() if now is None else now
        with self.lock:
            ocs, ocs_t, st, status_t = self.ocs, self.ocs_t, self.status, self.status_t
            hub_up, boat_up, bridge = self.hub_up, self.boat_up, self.bridge
            alerts, last_bad = list(self.alerts), self.last_bad
        conds: List[Tuple[str, str, str, Optional[float]]] = []      # key, text, style, own time (None = since first seen)
        if not hub_up:
            conds.append(("hub", "hub broker DOWN (this console's own link to the hub)", "bad", None))
        if ocs is None or now - ocs_t > 3:
            conds.append(("ocs", "no ocs/state" + (f" for {now - ocs_t:.0f} s" if ocs else "") + ": ocs_app down?", "bad", None))
        if bridge is False:
            conds.append(("bridge", "BRIDGE DOWN: hub <-> boat broker (the boat is on its own)", "bad", None))
        elif not boat_up and hub_up:
            conds.append(("boat", "boat broker not reachable from here (this console's link; the bridge may be fine)", "warn", None))
        if st and now - status_t > 3:
            conds.append(("status", f"status {self.vehicle_id} stale {now - status_t:.0f} s", "bad", None))
        if ocs and not ocs.get("judge_connected"):
            conds.append(("judge", "judge link DOWN", "bad", None))
        v = (st or {}).get("state")
        if v == "KILLED":
            conds.append(("kill", "vehicle KILLED: release the kill / E-stop, then confirm_safe", "bad", None))
        elif v == "FAULT":
            conds.append(("fault", f"vehicle FAULT {(st or {}).get('fault') or ''}: clear it, then confirm_safe", "bad", None))
        elif "EKF_BAD" in (((st or {}).get("health") or {}).get("faults") or []):   # v0.1.59 (A2): soft for 3 s - said at once
            conds.append(("ekf", "EKF BAD (no absolute position): a task holds; FAULT if it lasts 3 s - wait for GPS, or takeover on the boat",
                          "bad", None))
        if (ocs or {}).get("prepare_pending"):
            conds.append(("prepare", "waiting for PREPARE acks from " + ",".join(ocs["prepare_pending"]), "warn", None))
        if (ocs or {}).get("stalled"):                  # v0.1.59 (B3): a CAUTION (no ALARM, no sound) - the course manager may take minutes
            ss = int((ocs or {}).get("stall_s") or 0)
            conds.append(("stall", f"no RunStart after {ss // 60}:{ss % 60:02d} — ask the course manager, or redeclare", "warn", None))
        mi = (ocs or {}).get("mission") or {}
        if mi.get("armed_run") and mi.get("armed_run") == (ocs or {}).get("run_id"):
            conds.append(("armed", f"mission {mi.get('code') or mi.get('task')} armed: waiting for the vehicle to reach AUTO_HOLD", "warn", None))
        for t, kind, text in alerts:
            if now - t < 20 and kind not in self.INFO_ALERTS:
                conds.append((f"alert:{kind}:{t:.0f}", text, "bad", t))
        if last_bad[0] and now - last_bad[0] < 10:
            conds.append(("rejected", last_bad[1], "warn", last_bad[0]))
        keys = set()
        out = []
        with self.lock:
            for key, text, style, t0 in conds:
                keys.add(key)
                since = t0 if t0 is not None else self.cond_since.setdefault(key, now)
                out.append((max(0.0, now - since), text, style))
            for k in [k for k in self.cond_since if k not in keys]:
                del self.cond_since[k]
        out.sort(key=lambda it: -it[0])
        return out

    def nodes(self, broker: str, now: Optional[float] = None) -> List[Tuple[str, bool, float, dict]]:
        """(node, up, age_s, body) for every presence beat seen on a broker; up = said up and beat younger than 6 s."""
        now = time.time() if now is None else now
        with self.lock:
            items = [(n, b, t) for (br, n), (b, t) in self.presence.items() if br == broker]
        return sorted([(n, bool(b.get("up")) and now - t < 6.0, now - t, b) for n, b, t in items], key=lambda x: x[0])


def _short(d, n: int = 70) -> str:
    if not d:
        return ""
    s = json.dumps(d, separators=(",", ":"))
    return s if len(s) <= n else s[:n - 3] + "..."


class Feeds:
    """Two paho clients (hub, boat) feeding the model."""

    def __init__(self, model: Model, host: str, port: int, boat_host: str, boat_port: int):
        self.m = model
        tag = int(time.time()) % 100000
        self.hub = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"tui-hub-{tag}", protocol=mqtt.MQTTv311)
        self.hub.on_connect = self._hub_connect
        self.hub.on_disconnect = lambda *a, **k: setattr(self.m, "hub_up", False)
        self.hub.on_message = lambda c, u, msg: self.m.on_hub(msg.topic, msg.payload)
        self.hub.reconnect_delay_set(1, 2)
        self.boat = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"tui-boat-{tag}", protocol=mqtt.MQTTv311)
        self.boat.on_connect = self._boat_connect
        self.boat.on_disconnect = lambda *a, **k: setattr(self.m, "boat_up", False)
        self.boat.on_message = lambda c, u, msg: self.m.on_boat(msg.topic, msg.payload)
        self.boat.reconnect_delay_set(1, 2)
        self.host, self.port, self.boat_host, self.boat_port = host, port, boat_host, boat_port

    def _hub_connect(self, c, u, f, rc, p=None):
        self.m.hub_up = True
        for t, q in (("ocs/#", 0), (f"{TEAM_NS}/#", 1), ("rc/command", 0), ("rc/request", 0), ("rc/report/#", 0), ("rc/sim/stats", 0),
                     ("presence/#", 0), ("$SYS/broker/clients/connected", 0), ("$SYS/broker/load/messages/received/1min", 0),
                     ("$SYS/broker/load/messages/sent/1min", 0), ("$SYS/broker/uptime", 0)):
            c.subscribe(t, q)

    def _boat_connect(self, c, u, f, rc, p=None):
        self.m.boat_up = True
        c.subscribe(f"local/{self.m.vehicle}/mm/state", 0)
        c.subscribe(f"local/{self.m.vehicle}/perception/health", 0)
        c.subscribe(f"local/{self.m.vehicle}/ui/cmd/result", 0)             # v0.1.84.5: the answers to the OCS's /logging words
        for t in ("$SYS/broker/connection/#", "presence/#", "$SYS/broker/clients/connected", "$SYS/broker/load/messages/received/1min",
                  "$SYS/broker/load/messages/sent/1min", "$SYS/broker/uptime"):
            c.subscribe(t, 0)

    def start(self) -> None:
        self.hub.connect_async(self.host, self.port, keepalive=10)
        self.hub.loop_start()
        self._presence_stop = start_presence(self.hub, "tui", f"{self.host}:{self.port}")
        self.boat.connect_async(self.boat_host, self.boat_port, keepalive=10)
        self.boat.loop_start()

    def stop(self) -> None:
        self._presence_stop()
        for c in (self.hub, self.boat):
            c.loop_stop()
            try:
                c.disconnect()
            except Exception:
                pass


def map_url(host: str, ws_port: int, source: str = "") -> str:
    q = f"host={host}&port={ws_port}" + (f"&source={source}" if source else "")
    cam = os.environ.get("CAM_URL")                    # v0.1.24: the AGX camera stream, a link in map.html's header
    if cam:
        q += f"&cam={cam}"
    return f"file://{MAP_HTML}?{q}"


def open_map(host: str, ws_port: int, source: str = "") -> str:
    """Open tools/map.html in the default browser (macOS 'open', else webbrowser); returns a one-line report."""
    url = map_url(host, ws_port, source)
    if not os.path.exists(MAP_HTML):
        return f"map.html missing at {MAP_HTML}"
    try:
        if sys.platform == "darwin":
            subprocess.Popen(["open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            return f"map opened in the browser: {url}"
        if os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"):
            if webbrowser.open(url):
                return f"map opened in the browser: {url}"
        return f"no browser here - open on a desktop: {url}"
    except Exception as exc:
        return f"could not open a browser ({exc}): {url}"


def fcu_control(port: int, msg: dict) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.sendto(json.dumps(msg).encode(), ("127.0.0.1", port))


# ---- the command grammar (typed, like pool_test2) --------------------------------------------------------------------
# (word, usage line, description for the COMMANDS panel). Intent words are run through ocs_cli; plant words go to the fake FCU.
USAGE: List[Tuple[str, str, str]] = [
    ("declare", "declare                 RunDeclaration + PREPARE; R2/R4/R6 -> R3", "declare the run"),
    ("enable_auto", "enable_auto  (auto)     MANUAL -> AUTO_HOLD; needs armed, EKF, PREPARE", "vehicle to auto"),
    ("disable_auto", "disable_auto            AUTO_HOLD -> MANUAL (abort a task first)", "vehicle to manual"),
    ("start_task", "start_task [m4t0|goto|m0t0|m0t1|...][sim]   needs AUTO_HOLD + RunStart; no code = the preselected mission; old names (T4) work", "start a mission"),
    ("goto", "goto --x N --y E [--speed V]   bench task: drive to local metres", "bench: go to a point"),
    ("hold", "hold                    AUTO_HOLD/AUTO_TASK -> AUTO_SAFE_HOLD (task kept)", "pause"),
    ("resume", "resume                  AUTO_SAFE_HOLD -> back to the task or AUTO_HOLD", "unpause"),
    ("abort", "abort  (stop)           drop the task -> AUTO_HOLD", "drop the task"),
    ("confirm_safe", "confirm_safe (confirm)  leave KILLED / FAULT after the input cleared -> MANUAL", "operator confirmation"),
    ("end_run", "end_run  (end)          STOP_RUN; R -> R6", "end the run"),
    ("reset", "reset                   both ends to the start: RESET to the vehicle (abort, manual, run forgotten), OCS -> R2", "reset both ends"),
    ("select_target", "select_target red|green|blue   m0t4: the face to approach (the vehicle picks GREEN by itself)", "m0t4: override the colour"),
    ("save_point", "save_point NAME [--lat LAT --lon LON]   the boat's fix (or that position) as NAME in points.json: m0t2 --names", "m0t2: save a point"),
    ("del_point", "del_point NAME          remove a saved point", "delete a point"),
    ("clear_map", "clear_map               the boat's map forgets its detections (not while a task runs)", "clear the boat map"),
    ("home", "home [--speed V]        GO HOME: abort what runs, 0.3 m/s to the saved point HOME (else the origin), hold - any AUTO state", "go home (override)"),
    ("fcu_mode", "fcu_mode MODE            v0.1.50: the FCU's mode (MANUAL / HOLD / ...) through the mission manager; last wins", "FCU mode"),
    ("arm", "arm                     v0.1.50: ARM the FCU (its pre-arm checks decide)", "FCU arm"),
    ("disarm", "disarm                  v0.1.50: DISARM the FCU", "FCU disarm"),
    ("rc6_policy", "rc6_policy latched|hold  v0.1.50: RC6 latched = a later command wins; hold = no autonomy while RC6 is high", "RC6 rule"),
    ("fence", "fence apply course|geofence | fence on|off   v0.1.53: the USV's FCU geofence (upload + read back; DO_FENCE_ENABLE; off by default)", "FCU geofence"),
    ("speed", "speed cruise|circle|gate|all K | reset   v0.1.94.3: the live speed knobs x0.5-2.0, any state, at once", "speed knobs"),
    ("tune", "tune [KEY VALUE | save]  v0.1.49: a mission-manager knob (MANUAL / AUTO_HOLD); save = usv_tuned.yaml on the boat", "tune a knob"),
    ("step", "step                    advance the running m0t0 (TEST_MANUAL) one stage; m0t1: skip its timer", "test mission step"),
    ("skip", "skip                    v0.1.68 free run only: end the current stage's behaviour now (the next stage)", "free run: skip a stage"),
    ("session", "session start NAME [--model --tape --path --aim10 --confirm aim --label] | stop | result   v0.1.69: dry buoys / dockface / windows / uavmap / cannon, wet collect - OCS-side, nothing moves", "dry / wet sessions"),
    ("mission", "mission [m4t0|goto|m0t0|m0t1|off][sim] [--auto]   preselect what start_task runs; --auto = start it on RunStart; sim = fake FCU only", "preselect the mission"),
    ("inject", "inject run_start|assistance|keep_out|all_clear|moving|confirm [--lat --lon --cseq N]", "bench: judge messages"),
    ("ready", "ready                   macro: declare + enable_auto + wait for RunStart", "macro to R5"),
    ("manual", "manual                  macro: abort if a task runs, then disable_auto", "macro to MANUAL"),
    ("next", "next                    what is allowed now and why not (into the log)", "allowed now"),
    ("state", "state                   the ocs/state line (into the log)", "state line"),
    ("kill", "kill on|off             plant: assert / release the kill line (V1)", "plant: kill"),
    ("rc6", "rc6 on|off              plant: RC6 MANUAL override (V2)", "plant: RC6"),
    ("estop", "estop on|off            plant: RC5 motor E-stop (V1)", "plant: E-stop"),
    ("fcu", "fcu pause [S]           plant: stop the MAVLink stream S s (default 4) -> FCU_LOST (V6)", "plant: stream loss"),
    ("ekf", "ekf on|off              plant: EKF position ok flag (EKF_BAD -> V6)", "plant: EKF"),
    ("armed", "armed on|off            plant: the transmitter arms / disarms (the fake's arm line; `arm` / `disarm` = MAVLink)", "plant: arm state"),
    ("teleport", "teleport LAT LON [HDG]  plant: move the boat", "plant: teleport"),
    ("map", "map                     open the browser map (tools/map.html) on the hub websocket; BOAT tab = text map", "browser map"),
    ("clear", "clear                   clear the log", "clear log"),
    ("help", "help                    HELP screen (the playbook)", "help screen"),
    ("quit", "quit  (q)               leave the console; the bench keeps running (make stop stops it)", "quit"),
]
ALIAS = {"auto": "enable_auto", "task": "start_task", "confirm": "confirm_safe", "end": "end_run", "stop": "abort", "q": "quit", "x": "abort"}
INTENT_WORDS = {"declare", "enable_auto", "disable_auto", "start_task", "goto", "hold", "resume", "abort", "confirm_safe", "end_run", "inject", "reset", "step", "skip", "session", "mission",
                "select_target", "save_point", "del_point", "clear_map", "home", "tune",
                "fcu_mode", "arm", "disarm", "rc6_policy",          # v0.1.50
                "fence",                                            # v0.1.53: fence apply course|geofence, fence on|off
                "ready", "manual", "next", "state"}
PLANT_WORDS = {"kill", "rc6", "estop", "fcu", "ekf", "armed", "teleport"}
INTENT_WORDS = INTENT_WORDS | {"report"}                 # v0.1.53: START / STOP REPORTING
INTENT_WORDS = INTENT_WORDS | {"stop_reset"}             # v0.1.54: the STOP & RESET macro (ocs_cli.stop_reset)
INTENT_WORDS = INTENT_WORDS | {"speed"}                  # v0.1.94.3: the live speed knobs (speed cruise|circle|gate|all K | reset)
INTENT_WORDS = INTENT_WORDS | {"station_keep", "usv", "uav", "cancel_auto", "hold_on_start"}   # v0.1.83.4: ocs_cli parsed them, the page's /cmd
# refused them as "unknown command" (session A, pool sim 12:53): STATION KEEP HOLD|LOITER, USV / UAV PILOT READY, the 3-2-1 HOLD / CANCEL
USAGE.append(("stop_reset", "stop_reset              macro STOP & RESET: abort, disable_auto, fcu_mode HOLD, disarm, report off, reset",
              "macro: hold, disarm, reset"))
# v0.1.84.4 (session A: the CMD catalogue's descriptions come from here): the words ocs_cli parses that had no usage line
USAGE.extend([
    ("station_keep", "station_keep hold|loiter  the autonomous station keep: FCU HOLD (neutral) or LOITER (position hold)", "station keep"),
    ("usv", "usv ready on|off        the USV's PILOT READY (an OCS gate: declare refuses the USV without it)", "USV pilot ready"),
    ("uav", "uav ready on|off | uav scan done   the UAV's PILOT READY / its buoy scan finished (m1t2)", "UAV words"),
    ("report", "report on|off [usv|uav]  START / STOP REPORTING: the heartbeats to RoboCommand (all, or one vehicle)", "reporting"),
    ("cancel_auto", "cancel_auto             in the 3-2-1 after RunStart: no auto start (start_task to start by hand)", "cancel auto start"),
    ("hold_on_start", "hold_on_start           in the 3-2-1 after RunStart: the mission starts at 0, then HOLD (resume to go on)", "hold on start"),
])
# the COMMANDS panel of MAIN: every word, in six groups (modifiers appear in `usage:` as you type)
COMMAND_GROUPS: List[Tuple[str, List[str]]] = [
    ("RUN", ["declare", "ready", "end_run", "reset", "mission", "state", "next"]),
    ("VEHICLE", ["enable_auto", "disable_auto", "manual", "hold", "resume", "confirm_safe"]),
    ("TASK", ["start_task", "goto", "abort", "step", "select_target"]),   # skip (v0.1.68, free run): typed, not in the panel (150-wide)
    ("JUDGE", ["inject run_start|assistance|", "keep_out|all_clear|moving|confirm"]),
    ("PLANT", ["kill", "rc6", "estop", "ekf", "armed", "fcu pause", "teleport"]),
    ("TUI", ["map", "clear", "help", "quit"]),
]
WORD_TO_ACTION = {"declare": "DECLARE", "enable_auto": "ENABLE_AUTO", "disable_auto": "DISABLE_AUTO", "start_task": "START_TASK", "goto": "GOTO",
                  "hold": "HOLD", "resume": "RESUME", "abort": "ABORT_TASK", "confirm_safe": "CONFIRM_SAFE", "end_run": "END_RUN", "inject": "INJECT",
                  "reset": "RESET_RUN", "step": "STEP", "skip": "SKIP", "select_target": "SELECT_TARGET",
                  "save_point": "SAVE_POINT", "del_point": "DELETE_POINT", "clear_map": "CLEAR_MAP", "home": "HOME",
                  "fcu_mode": "FCU_MODE", "arm": "FCU_ARM", "disarm": "FCU_ARM", "rc6_policy": "RC6", "station_keep": "STATION_KEEP",   # v0.1.50 / v0.1.81
                  "fence": "FENCE_APPLY"}                                                                # v0.1.53 (fence on|off = FENCE_ENABLE)


def _emit_words(c, yy: int, x: int, label: str, line) -> None:
    c.put(yy, x + 2, f"{label:<8}", "title")
    cx = x + 11
    for j, (wd, st) in enumerate(line):
        c.put(yy, cx, wd, st)
        cx += len(wd)
        if j < len(line) - 1:
            c.put(yy, cx, " \u2502 ", "dim")
            cx += 3


EXAMPLES: Dict[str, str] = {
    "declare": "declare", "enable_auto": "enable_auto", "disable_auto": "disable_auto", "start_task": "start_task m0t1",
    "goto": "goto --x 10 --y 0 --speed 0.6", "hold": "hold", "resume": "resume", "abort": "abort", "confirm_safe": "confirm_safe",
    "fence": "fence apply course",
    "end_run": "end_run", "reset": "reset", "step": "step", "skip": "skip", "session": "session start buoys", "select_target": "select_target red", "save_point": "save_point A", "del_point": "del_point A", "clear_map": "clear_map", "home": "home", "mission": "mission m0t0 --auto",
    "inject": "inject assistance --lat 1.28112 --lon 103.85501 --cseq 1", "ready": "ready", "manual": "manual", "next": "next",
    "state": "state", "kill": "kill on", "rc6": "rc6 on", "estop": "estop off", "fcu": "fcu pause 4", "ekf": "ekf off",
    "armed": "armed on", "teleport": "teleport 1.28108 103.85502 90", "map": "map", "clear": "clear", "help": "help", "quit": "quit",
}


EXAMPLES["stop_reset"] = "stop_reset"                    # v0.1.54


def example_for(line: str) -> Optional[str]:
    """A typical invocation of the command being typed (exact word or a unique prefix)."""
    word = line.strip().split(" ")[0].lower() if line.strip() else ""
    if not word:
        return None
    word = ALIAS.get(word, word)
    if word in EXAMPLES:
        return EXAMPLES[word]
    pref = [k for k, _, _ in USAGE if k.startswith(word)]
    return EXAMPLES.get(pref[0]) if len(pref) == 1 else None


def usage_for(line: str) -> Optional[str]:
    word = line.strip().split(" ")[0].lower() if line.strip() else ""
    if not word:
        return None
    word = ALIAS.get(word, word)
    exact = [u for k, u, _ in USAGE if k == word]
    if exact:
        return exact[0]
    pref = [k for k, _, _ in USAGE if k.startswith(word)]
    if len(pref) == 1:
        return [u for k, u, _ in USAGE if k == pref[0]][0]
    if pref:
        return "commands: " + "  ".join(pref)
    return "unknown command - see the COMMANDS panel or the HELP screen"


def _onoff(s: str) -> Optional[bool]:
    return {"on": True, "1": True, "true": True, "yes": True, "off": False, "0": False, "false": False, "no": False}.get(s.lower())


# ---- a canvas so the drawing is pure and testable (no curses needed) -----------------------------------------------
class Canvas:
    def __init__(self, w: int, h: int):
        self.w, self.h = w, h
        self.rows = [[" "] * w for _ in range(h)]
        self.attrs = [[""] * w for _ in range(h)]

    def put(self, y: int, x: int, s: str, attr: str = "") -> None:
        if not (0 <= y < self.h) or x >= self.w - 1:
            return
        for i, ch in enumerate(s):
            if x + i >= self.w - 1:
                break
            if x + i >= 0:
                self.rows[y][x + i] = ch
                self.attrs[y][x + i] = attr
        return

    def box(self, y: int, x: int, h: int, w: int, title: str) -> None:
        """Bordered panel with a title; clipped to the canvas (ACS line characters are substituted by the painter)."""
        w = min(w, self.w - x - 1)
        h = min(h, self.h - y - 1)
        if h < 2 or w < 4 or y >= self.h - 1 or x >= self.w - 1:
            return
        self.put(y, x, "\u250c" + "\u2500" * (w - 2) + "\u2510", "dim")
        self.put(y + h - 1, x, "\u2514" + "\u2500" * (w - 2) + "\u2518", "dim")
        for yy in range(y + 1, y + h - 1):
            self.put(yy, x, "\u2502", "dim")
            self.put(yy, x + w - 1, "\u2502", "dim")
        self.put(y, x + 2, f" {title} ", "title")

    def kv(self, y: int, x: int, key: str, val: str, attr: str = "", kw: int = 6) -> None:
        self.put(y, x, f"{key:<{kw}}", "dim")
        self.put(y, x + kw, val, attr)

    def lines(self) -> List[str]:
        return ["".join(r).rstrip() for r in self.rows]


def _num(v, nd: int) -> str:
    try:
        return f"{float(v):.{nd}f}"
    except (TypeError, ValueError):
        return "--"


def _age(t: float) -> str:
    return "-" if not t else f"{time.time() - t:.1f} s"


def _ladder(c: Canvas, y: int, x: int, order: List[str], current: Optional[str], codes: Optional[Dict[str, str]] = None) -> None:
    for name in order:
        cell = (codes or {}).get(name, name) if codes else name
        if name == current:
            c.put(y, x, f"[{cell}]", "now")
        else:
            c.put(y, x, f" {cell} ", "dim")
        x += len(cell) + 3


class View:
    """Draws one screen of the model into a Canvas."""

    def __init__(self, m: Model, vehicle_id: str):
        self.m, self.vehicle_id = m, vehicle_id
        self.cursor = 0                      # the highlighted mission on the MISSION tab

    def draw(self, c: Canvas, screen: int, cmdline: str, last: str, log_scroll: int = 0, conn_label: str = "") -> None:
        w, h = c.w, c.h
        tabs = "  ".join((f"[{n}]" if i == screen else f" {n} ") for i, n in enumerate(SCREENS))
        nw = len(self.m.warnings())
        flag = f" {nw} WARNING{'S' if nw != 1 else ''} " if nw else ""
        bar = f" op-tui v{__version__}  {conn_label}  {time.strftime('%H:%M:%S')}  "
        c.put(0, 0, bar.ljust(w - 1), "bar")
        if flag:
            c.put(0, len(bar), flag, "armed")
        tabs_txt = f"Tab: {tabs} "
        c.put(0, max(len(bar) + len(flag) + 2, w - 1 - len(tabs_txt)), tabs_txt, "bar")
        if w < MIN_W or h < MIN_H:
            c.put(2, 0, f"terminal too small: need {MIN_W}x{MIN_H}, have {w}x{h}  (the shared skeleton of both consoles; 150x50 is comfortable)", "bad")
            self.prompt(c, cmdline, last)
            return
        [self.main, self.mission_screen, self.boat, self.network, self.judge, self.logscreen, self.helpscreen][screen](c, log_scroll)
        self.prompt(c, cmdline, last)

    # -- bottom three lines --
    def prompt(self, c: Canvas, cmdline: str, last: str) -> None:
        h = c.h
        c.put(h - 4, 0, ("last: " + last)[: c.w - 1], "warn" if any(k in last for k in ("refused", "REJECTED", "FAILED", "unknown", "bad ", "playbook", "cannot", "busy")) else "")
        u = usage_for(cmdline)
        if u:
            c.put(h - 3, 0, "usage: " + u, "warn" if u.startswith("unknown") else "title")
            ex = example_for(cmdline)
            c.put(h - 2, 0, ("example: " + ex) if ex else "", "dim")
        else:
            with self.m.lock:
                ocs = self.m.ocs
            c.put(h - 3, 0, "next: " + playbook.suggest(ocs, self.vehicle_id, self.m.ui_state()), "title")
            c.put(h - 2, 0, "example: declare  |  start_task m0t1  |  mission m4t0 --auto  |  goto --x 10 --y 0 --speed 0.6  |  reset", "dim")
        c.put(h - 1, 0, "> " + cmdline, "bold")

    # -- MAIN (the shared skeleton: banner, RUN | VEHICLE, MISSION | WHERE WE ARE, FLOW, JUDGE | NETWORK | WARNINGS, EVENTS | CONSOLE | COMMANDS) --
    def geometry(self, w: int, h: int) -> dict:
        """Row and column plan of MAIN for this terminal size; spacers appear from 48 rows."""
        sp1, sp2, sp3 = int(h >= 46), int(h >= 52), int(h >= 54)       # spacers after row 1, after FLOW, after row 4
        g = {"y1": 2, "r1h": 9, "r2h": 8, "fh": 7, "r4h": 7}
        g["y2"] = g["y1"] + g["r1h"] + sp1
        g["yf"] = g["y2"] + g["r2h"]
        g["y4"] = g["yf"] + g["fh"] + sp2
        g["y5"] = g["y4"] + g["r4h"] + sp3
        g["bottom"] = h - 4 - g["y5"]                                     # 12 rows at 150x50 (10 lines of EVENTS / CONSOLE / COMMANDS)
        g["host_w"] = 50
        g["where_w"] = 52 if w >= 140 else 44
        g["a_w"] = int(w * 0.37)
        g["net_w"] = int(w * 0.35)
        g["ev_w"] = int(w * 0.29)
        g["con_w"] = int(w * 0.29)
        return g

    def main(self, c: Canvas, log_scroll: int) -> None:
        m = self.m
        with m.lock:
            ocs, st, loc = m.ocs, m.status, m.local
            hub_up, boat_up, bridge = m.hub_up, m.boat_up, m.bridge
            log = list(m.log)
            ocs_t, status_t, local_t = m.ocs_t, m.status_t, m.local_t
            ui_beat = m.presence.get(("boat", "usv_ui"))
        w, h = c.w, c.h
        g = self.geometry(w, h)
        now = time.time()
        veh = ((ocs or {}).get("vehicles") or {}).get(self.vehicle_id) or {}
        ui_act = m.ui_state(now)
        self.banner(c)
        # ---- row 1: RUN | VEHICLE group
        y, lw = g["y1"], g["host_w"]
        c.box(y, 0, g["r1h"], lw, "RUN  (OCS)")
        r = (ocs or {}).get("r")
        _ladder(c, y + 1, 2, R_ORDER, r)
        ocs_ok = bool(ocs) and now - ocs_t < 3
        c.kv(y + 2, 2, "state", f"{r or '--'} {(ocs or {}).get('r_state') or 'no ocs/state'}"[:26], "warn" if ocs_ok else "bad", 8)
        c.put(y + 2, 30, f"ocs/state {_age(ocs_t)}"[: lw - 32], "ok" if ocs_ok else "bad")
        pp = (ocs or {}).get("prepare_pending") or []
        c.kv(y + 3, 2, "run", f"{(ocs or {}).get('run_id')}   decl {(ocs or {}).get('declaration_seq')}   " +
             (f"PREPARE? {','.join(pp)}" if pp else ("STALLED" if (ocs or {}).get("stalled") else "stalled no"))[: lw - 12], "warn" if pp else "bold", 8)
        c.kv(y + 4, 2, "judge", f"{(ocs or {}).get('judge')}  {'up' if (ocs or {}).get('judge_connected') else 'DOWN'}   rate {(ocs or {}).get('msg_rate')}/5   reports {(ocs or {}).get('reports_sent')}"[: lw - 12],
             "ok" if (ocs or {}).get("judge_connected") else "bad", 8)
        c.kv(y + 5, 2, "guards", ("BENCH (goto / inject allowed)" if (ocs or {}).get("bench_mode") else "course (no bench intents)") if ocs else "--",
             "warn" if (ocs or {}).get("bench_mode") else "ok", 8)
        c.kv(y + 6, 2, "means", (playbook.R_MEANING.get(r, "").split(": ", 1)[-1] if r else "start ocs_app")[: lw - 12], "dim", 8)
        c.kv(y + 7, 2, "next", playbook.suggest(ocs, self.vehicle_id, ui_act)[: lw - 12], "dim", 8)
        self.vehicle_group(c, y, lw + 2, w - lw - 3, g["r1h"], st, status_t, veh.get("state"), loc, local_t, ui_beat, now)
        # ---- row 2: MISSION | WHERE WE ARE
        y = g["y2"]
        mw = w - g["where_w"] - 2
        self.mission_box(c, y, 0, g["r2h"], mw, ocs, st, status_t, now)
        self.where_box(c, y, mw + 1, g["r2h"], w - mw - 2, ocs, st, loc, local_t, ui_beat, ui_act, hub_up, boat_up, bridge, now)
        # ---- FLOW
        self.flow_box(c, g["yf"], 0, g["fh"], w - 1, ocs, ui_act)
        # ---- row 4: JUDGE | NETWORK | WARNINGS
        y = g["y4"]
        aw, nw = g["a_w"], g["net_w"]
        self.judge_box(c, y, 0, g["r4h"], aw, ocs)
        self.network_small(c, y, aw + 1, g["r4h"], nw, now)
        self.warnings_box(c, y, aw + nw + 2, g["r4h"], w - aw - nw - 3, now)
        # ---- row 5: EVENTS | CONSOLE | COMMANDS
        y = g["y5"]
        bh = g["bottom"]
        ew, cw = g["ev_w"], g["con_w"]
        c.box(y, 0, bh, ew, "EVENTS  (what happened; LOG tab = all)")
        ev = [e for e in log if e[1] in ("event", "alert", "judge", "bridge", "cmd")]
        n_ev = max(1, bh - 2)
        show = ev[-n_ev:] if not log_scroll else ev[max(0, len(ev) - n_ev - log_scroll): max(1, len(ev) - log_scroll)]
        for i, (t, kind, text) in enumerate(show):
            attr = {"alert": "bad", "event": "warn", "judge": "judge", "cmd": "cmd", "bridge": "bad"}.get(kind, "")
            c.put(y + 1 + i, 2, f"{now - t:4.0f}s {text}"[: ew - 4], attr)
        c.box(y, ew + 1, bh, cw, "CONSOLE  (your commands -> result, hint)")
        con = [e for e in log if e[1] in ("ui", "intent", "ack")]
        for i, (t, kind, text) in enumerate(con[-n_ev:]):
            attr = {"ack": "ok", "intent": "ok", "ui": "title"}.get(kind, "")
            if ("REJECTED" in text or "FAILED" in text or "refused" in text or text.startswith("error")):
                attr = "bad"
            if kind == "ui" and not text.startswith("> "):
                attr = "dim"
            pre = "" if text.startswith("> ") else "  "
            c.put(y + 1 + i, ew + 3, (pre + text)[: cw - 4], attr)
        self.commands_box(c, y, ew + cw + 2, bh, w - ew - cw - 3, ocs, st, ui_act)

    def banner(self, c: Canvas) -> None:
        txt = " OCS CONSOLE  ·  op-tui on the Mac (shore)  ·  run lifecycle, the judge, intents to the vehicles  ·  the boat side is usv_ui "
        st = getattr(self.m, "status", None) or {}
        if st.get("nav") == "ghost":                     # v0.1.68 (WP17 F6): the FREE RUN banner where the console's banner goes
            fr = st.get("free_run") or {}
            txt = (" FREE RUN  ·  ghost position, no motion to the FCU (mode requests only)  ·  bench only, props off, ESC power off, NEVER at the pool "
                   + (f" ·  {fr['banner']} " if fr.get("banner") else ""))
        c.put(1, 0, txt.ljust(c.w - 1)[: c.w - 1], "banner")

    def vehicle_group(self, c: Canvas, y: int, gx: int, w: int, h: int, st: Optional[dict], status_t: float, ocs_state: Optional[str],
                      loc: Optional[dict], local_t: float, ui_beat, now: float) -> None:
        """VEHICLE box with two halves: state (ladder, flags, task, means, stage) | position / FCU (incl. which FCU and the mm profile)."""
        c.box(y, gx, h, w, f"VEHICLE  {self.vehicle_id}")
        split = min(52, max(40, w // 2 + 4))
        c.put(y + 1, gx, "├" + "─" * (w - 2) + "┤", "dim")
        c.put(y + 1, gx + 2, " state ", "title")
        c.put(y + 1, gx + split + 2, " position / FCU ", "title")
        px = gx + split
        for yy in range(y + 2, y + h - 1):
            c.put(yy, px, "│", "dim")
        c.put(y + 1, px, "┬", "dim")
        c.put(y + h - 1, px, "┴", "dim")
        x = gx + 2
        lwid = split - 3
        v = (st or {}).get("state") or ocs_state
        _ladder(c, y + 2, x, V_ORDER, v, playbook.V_CODE)
        st_ok = bool(st) and now - status_t < 3
        c.kv(y + 3, x, "state", f"{playbook.V_CODE.get(v, '--')} {v or 'no status'}  {(st or {}).get('robot_state') or ''}"[: lwid - 20], "warn" if st_ok else "bad", 7)
        c.put(y + 3, x + lwid - 13, f"status {_age(status_t)}"[:13], "ok" if st_ok else "bad")
        armed = (st or {}).get("armed")
        c.put(y + 4, x, " ARMED " if armed else " disarmed ", "armed" if armed else "disarmed")
        kill, estop = (st or {}).get("kill"), (st or {}).get("estop")
        c.put(y + 4, x + 11, " KILL " if kill else " kill n ", "armed" if kill else "dim")
        c.put(y + 4, x + 20, " ESTOP " if estop else " estop n ", "armed" if estop else "dim")
        c.put(y + 4, x + 30, f"mode {(st or {}).get('mode')}"[: lwid - 30], "warn")
        tn, ts_ = (st or {}).get("task_name"), (st or {}).get("task_state")
        c.kv(y + 5, x, "task", f"{tn or '-'}  {ts_ or ''}".rstrip()[:16], "bold", 7)
        fault, hold = (st or {}).get("fault"), (st or {}).get("hold_reason")
        c.put(y + 5, x + 24, f"fault {fault or '-'}   hold {hold or '-'}"[: lwid - 24], "bad" if fault else ("warn" if hold else "dim"))
        c.kv(y + 6, x, "means", (playbook.V_MEANING.get(v, "").split(" ", 1)[-1] if v else "no status from the vehicle")[: lwid - 7], "dim", 7)
        stage = playbook.task_stage_meaning(tn, ts_) if tn else ""
        c.kv(y + 7, x, "stage", (f"{tn} {ts_}: {stage}" if stage else (f"{tn} {ts_ or ''}" if tn else "no task"))[: lwid - 7], "warn" if tn else "dim", 7)
        # position / FCU half
        px += 2
        rwid = gx + w - 1 - px
        hl = (st or {}).get("health") or {}
        c.kv(y + 2, px, "lat", _num((st or {}).get("lat"), 7), "bold", 5)
        c.put(y + 2, px + 18, f"lon {_num((st or {}).get('lon'), 7)}"[: rwid - 18], "bold")
        lcl = (st or {}).get("local") or {}
        c.kv(y + 3, px, "hdg", f"{_num((st or {}).get('hdg_deg'), 1)} deg", "", 5)
        c.put(y + 3, px + 16, f"sog {_num((st or {}).get('sog_mps'), 2)} m/s", "")
        c.put(y + 3, px + 30, f"CO N {_num(lcl.get('x_n'), 1)} E {_num(lcl.get('y_e'), 1)}"[: rwid - 30], "co" if lcl else "dim")
        fa = hl.get("fcu_age_s")
        c.kv(y + 4, px, "fcu", f"{fa} s" if fa is not None else "--", "ok" if (fa is not None and fa < 1.0) else "bad", 5)
        c.put(y + 4, px + 13, f"rc {'ok' if hl.get('rc_ok') else 'NO'}", "ok" if hl.get("rc_ok") else "bad")
        c.put(y + 4, px + 20, f"bridge {'up' if hl.get('bridge_up') else 'DOWN'}", "ok" if hl.get("bridge_up") else "bad")
        f = (loc or {}).get("fcu") or {}
        if loc and now - local_t < 3:
            c.put(y + 4, px + 32, f"ekf {'ok' if f.get('ekf_pos_ok') else 'BAD'}"[: rwid - 32], "ok" if f.get("ekf_pos_ok") else "bad")
            rc = f.get("rc") or {}
            c.kv(y + 5, px, "gps", f"{f.get('gps_fix')}/{f.get('sats')} sats", "", 5)
            c.put(y + 5, px + 16, f"batt {_num(f.get('batt_v'), 1)} V", "")
            c.put(y + 5, px + 30, f"rc6 {rc.get('6', rc.get(6, '-'))}"[: rwid - 30], "")
        else:
            c.kv(y + 5, px, "gps", ("boat broker: waiting for local/usv/mm/state" if self.m.boat_up else "boat broker 5 Hz state: n/a")[: rwid - 5], "dim", 5)
        link = (ui_beat[0].get("fcu_link") if ui_beat and now - ui_beat[1] < 6 else None)
        conn = str((ui_beat[0].get("fcu_conn") if ui_beat else "") or "").replace("udpin:127.0.0.1:", "udp ")
        c.kv(y + 6, px, "FCU", (f"{'FAKE plant' if link == 'fake' else 'REAL Pixhawk' if link == 'real' else '?'} {conn} (held by usv_ui)" if link
                                else "link unknown (usv_ui not seen on the boat broker)")[: rwid - 5], "warn" if link == "fake" else ("ok" if link == "real" else "dim"), 5)
        prof = (loc or {}).get("profile") if loc and now - local_t < 3 else None
        c.kv(y + 7, px, "mm", (f"{prof}{' - LAB GUARDS' if (loc or {}).get('lab') else ' (full guards)'}" if prof else "profile unknown (mm/state)")[: rwid - 5],
             "warn" if (loc or {}).get("lab") else "dim", 5)

    def mission_state(self, ocs: Optional[dict], st: Optional[dict], status_t: float, now: float) -> Tuple[str, str, str]:
        """(task, state line, style) of the mission the OCS holds, against what the vehicle reports."""
        mi = (ocs or {}).get("mission") or {}
        task = mi.get("task")
        run_id = (ocs or {}).get("run_id")
        tn, ts_, v = (st or {}).get("task_name"), (st or {}).get("task_state"), (st or {}).get("state")
        if tn and v in ("AUTO_TASK", "AUTO_SAFE_HOLD") and (task is None or tn == task):
            return tn or task, f"RUNNING  run {run_id}  stage {ts_}" + ("  (paused)" if v == "AUTO_SAFE_HOLD" else "") + ("" if task else "  (started with start_task, no preselection)"), "ok"
        if not task:
            return "", "none selected   (ocs_cli mission <code> [--auto]; start_task <code> runs one directly)", "dim"
        if mi.get("armed_run") and mi.get("armed_run") == run_id:
            return task, f"ARMED  run {run_id}: starts on the next AUTO_HOLD status (vehicle is {v})", "warn"
        if mi.get("started_run") and mi.get("started_run") == run_id:
            return task, f"STARTED  run {run_id}  (task {'done' if ts_ == 'DONE' else 'over'}: vehicle {v})", "ok"
        if mi.get("auto"):
            return task, "preselected, auto-start on the judge's RunStart (declare -> enable_auto -> RunStart)", "warn"
        return task, "preselected: start_task (no task) runs it when the vehicle is AUTO_HOLD in R5", "warn"

    def mission_box(self, c: Canvas, y: int, x: int, h: int, w: int, ocs: Optional[dict], st: Optional[dict], status_t: float, now: float) -> None:
        c.box(y, x, h, w, "MISSION  (ocs_cli mission <code> [--auto]; start_task runs it; MISSION tab for every mission)")
        task, state, attr = self.mission_state(ocs, st, status_t, now)
        mi = (ocs or {}).get("mission") or {}
        mw = w - 4
        code = (mi.get("code") if mi.get("task") == task and mi.get("code") else playbook.code_of(task, mi.get("tier") if mi.get("task") == task else None)) if task else ""
        c.kv(y + 1, x + 2, "selected", (f"{code}  ({task})   auto-start on RunStart: {'yes' if mi.get('auto') else 'no'}" if task else "none")[: mw - 10], "warn" if task else "dim", 10)
        hold = str((st or {}).get("hold_reason") or "")
        if hold.startswith("step: next "):           # v0.1.26 stage stepping
            state, attr = f"STEP · next {hold[len('step: next '):]}   (resume enters it; abort ends the task)", "warn"
        c.kv(y + 2, x + 2, "state", state[: mw - 10], attr, 10)
        m = playbook.mission_for(task, mi.get("tier") if mi.get("task") == task else None)
        if m:
            c.kv(y + 3, x + 2, "what", f"{m.title}: {m.what[0]}"[: mw - 10], "dim", 10)
            c.put(y + 4, x + 12, (m.what[1] if len(m.what) > 1 else "")[: mw - 10], "dim")
        else:
            c.kv(y + 3, x + 2, "what", "m4t0 (T4 incident, built) | m0t0 / m0t1 (state-machine tests) | goto (bench) | m1t1 m1t2 m3t1 m0t2 m0t3 planned"[: mw - 10], "dim", 10)
            c.put(y + 4, x + 12, "Tab -> MISSION: each mission with its stages and how to select it; +sim = fake FCU only"[: mw - 10], "dim")
        names = playbook.mission_stage_names(task)
        c.put(y + 5, x + 2, "stages", "dim")
        cx = x + 12
        cur = (st or {}).get("task_state") if ((st or {}).get("task_name") == task and (st or {}).get("state") in ("AUTO_TASK", "AUTO_SAFE_HOLD")) else None
        passed = True
        for i, nme in enumerate(names):
            if cx + len(nme) + 6 > x + w - 2:
                break
            if nme == cur:
                c.put(y + 5, cx, f" {nme} ", "now"); passed = False
            else:
                c.put(y + 5, cx, f" {nme} ", "ok" if (cur and passed) else "dim")
            cx += len(nme) + 2
            if i < len(names) - 1:
                c.put(y + 5, cx, " > ", "dim"); cx += 3
        if not names:
            c.put(y + 5, cx, "-", "dim")
        if m:
            c.kv(y + 6, x + 2, "ends", m.ends[: mw - 10], "dim", 10)

    def where_box(self, c: Canvas, y: int, x: int, h: int, w: int, ocs, st, loc, local_t, ui_beat, ui_act, hub_up, boat_up, bridge, now) -> None:
        c.box(y, x, h, w, "WHERE WE ARE  (flavour; help)")
        xx, mw = x + 2, w - 4
        r = (ocs or {}).get("r")
        veh = ((ocs or {}).get("vehicles") or {}).get(self.vehicle_id) or {}
        v = (st or {}).get("state") or veh.get("state")
        tn, ts_ = (st or {}).get("task_name"), (st or {}).get("task_state")
        if not ocs:
            l1, l2 = "no ocs/state: ocs_app is not up on the hub", "scripts/run_ocs.sh starts it"
        elif v in ("AUTO_TASK", "AUTO_SAFE_HOLD") and tn:
            l1, l2 = f"{r}, vehicle {v}: the mission manager", f"drives {tn} ({ts_}); usv_ui watches"
        elif ui_act:
            l1, l2 = f"{r}, vehicle {v or '?'}: usv_ui commands the FCU", "handover (in usv_ui) before enable_auto"
        else:
            l1, l2 = f"{r} {(ocs or {}).get('r_state') or ''}, vehicle {v or '?'}", playbook.suggest(ocs, self.vehicle_id, ui_act)
        c.kv(y + 1, xx, "now", l1[: mw - 6], "", 6)
        c.put(y + 2, xx + 6, l2[: mw - 6], "")
        link = (ui_beat[0].get("fcu_link") if ui_beat and now - ui_beat[1] < 6 else None)
        prof = (loc or {}).get("profile") if loc and now - local_t < 3 else None
        c.kv(y + 3, xx, "FCU", ("fake plant" if link == "fake" else "real Pixhawk" if link == "real" else "?"), "warn" if link == "fake" else ("ok" if link == "real" else "dim"), 6)
        c.put(y + 3, xx + 18, f"mm {prof or '?'}"[:18], "warn" if (loc or {}).get("lab") else "ok")
        c.put(y + 3, xx + 36, f"judge {(ocs or {}).get('judge') or '?'}"[: mw - 36], "warn" if (ocs or {}).get("judge") == "fake" else "ok")
        c.kv(y + 4, xx, "net", f"{self.m.net_profile or '?'}", "ok" if self.m.net_profile else "dim", 6)
        br = "bridge up" if bridge else ("bridge DOWN" if bridge is False else "bridge ?")
        c.put(y + 4, xx + 12, br, "ok" if bridge else "bad")
        c.put(y + 4, xx + 26, f"hub {'UP' if hub_up else 'DOWN'}  boat {'UP' if boat_up else 'n/a'}"[: mw - 26], "ok" if (hub_up and boat_up) else "bad")
        nodes = {n: up for n, up, _, _ in self.m.nodes("hub", now)}
        nodes.update({n: up for n, up, _, _ in self.m.nodes("boat", now)})
        ui_log = (ui_beat[0].get("log") if ui_beat and now - ui_beat[1] < 6 else None)
        c.kv(y + 5, xx, "log", f"SQLite hub {'ON' if nodes.get('logger-hub') else 'OFF'}  boat {'ON' if nodes.get('logger-boat') else 'OFF'}   usv_ui {('ON ' + str(ui_log)) if ui_log else 'OFF'}"[: mw - 6],
             "warn" if (nodes.get("logger-hub") or nodes.get("logger-boat") or ui_log) else "dim", 6)
        with self.m.lock:
            perc, perc_t = self.m.perc_health, self.m.perc_t
        cvt, cvs = playbook.cv_source(perc, (time.time() - perc_t) if perc else None, bool(nodes.get("cv_publish")))
        c.kv(y + 6, xx, "cv", cvt[: mw - 6], cvs, 6)
        c.put(y + 6, xx + 6 + len(cvt) + 3, "HELP tab; `next` = what is allowed now"[: max(0, mw - 9 - len(cvt))], "dim")

    def flow_box(self, c: Canvas, y: int, x: int, h: int, w: int, ocs: Optional[dict], ui_act: Optional[bool]) -> None:
        """The two-row snake: boat row, connector, OCS row; the current stage in reverse video, passed stages green."""
        c.box(y, x, h, w, "FLOW  (where the run is; reverse = now, green = done)")
        stage = playbook.flow_stage(ocs, self.vehicle_id, ui_act)
        wide = w >= 140
        rows = playbook.flow_rows(stage, wide)
        arrow = " ───> " if wide else " ─> "
        styles = {"cur": "now", "done": "ok", "future": "dim"}

        def draw_row(yy: int, x0: int, cells) -> int:
            cx = x0
            for i, (label, cap, stt) in enumerate(cells):
                cell = f" {label} "
                c.put(yy, cx, cell, styles[stt])
                c.put(yy + 1, cx + 1, cap, "dim")
                cx += len(cell)
                if i < len(cells) - 1:
                    c.put(yy, cx, arrow, "dim")
                    cx += len(arrow)
            return cx
        c.put(y + 1, x + 2, "boat ", "title")
        end = draw_row(y + 1, x + 8, rows[0])
        c.put(y + 1, end, " ──┐", "dim")
        c.put(y + 2, end + 3, "│", "dim")
        c.put(y + 3, x + 2, "OCS  ", "title")
        c.put(y + 3, x + 8, "┌" + "─" * max(0, end + 3 - (x + 8) - 1) + "┘", "dim")
        draw_row(y + 4, x + 8, rows[1])
        who = "FCU: usv_ui (boat console)" if ui_act else ("FCU: mission manager" if ui_act is False else "FCU: ? (boat broker)")
        c.put(y + 1, x + w - 2 - len(who), who, "warn" if ui_act else ("ok" if ui_act is False else "dim"))

    def judge_box(self, c: Canvas, y: int, x: int, h: int, w: int, ocs: Optional[dict]) -> None:
        c.box(y, x, h, w, "FROM THE JUDGE  (last of each payload)")
        jv = (ocs or {}).get("judge_view") or {}
        xx, mw = x + 2, w - 4
        if not jv:
            c.put(y + 1, xx, "no ocs/state yet", "dim")
            return
        co_ = jv.get("course")
        c.kv(y + 1, xx, "course", (f"{co_.get('course_id')}  {co_.get('points')} pts  pinger {co_.get('pinger_hz') or '-'} Hz  {'valid' if co_.get('valid') else 'INVALID'}  {co_.get('age_s')} s"
                                   if co_ else "none yet (retained RxCourse expected)")[: mw - 10], ("ok" if co_.get("valid") else "bad") if co_ else "bad", 10)
        rs = jv.get("run_start")
        c.kv(y + 2, xx, "run_start", (f"run {rs.get('run_id')}  decl {rs.get('declaration_seq')} vs ours {rs.get('ours')}  {'ACCEPTED' if rs.get('accepted') else 'IGNORED'}  {rs.get('age_s')} s ({rs.get('origin')})"
                                      if rs else "none yet")[: mw - 10], ("ok" if rs.get("accepted") else "bad") if rs else "dim", 10)
        inc = None
        for k in ("assistance_request", "keep_out_zone", "all_clear", "moving_object"):
            if k in jv and (inc is None or jv[k].get("age_s", 1e9) < inc[1].get("age_s", 1e9)):
                inc = (k, jv[k])
        if inc:
            k, d = inc
            c.kv(y + 3, xx, "incident", (f"{k} cseq {d.get('command_seq')} {d.get('domain') or ''} "
                                         f"{('lat %.5f lon %.5f' % (d['lat'], d['lon'])) if d.get('lat') is not None else ''}"
                                         f"{(' polygon %d pts' % d['polygon_points']) if d.get('polygon_points') else ''}  {d.get('age_s')} s")[: mw - 10], "warn", 10)
        else:
            c.kv(y + 3, xx, "incident", "none yet", "dim", 10)
        rc_ = jv.get("readiness_confirm")
        pc = jv.get("pending_confirm") or []
        c.kv(y + 4, xx, "confirm", ((f"cseq {rc_.get('command_seq')} {rc_.get('age_s')} s ago" if rc_ else "none yet") + (f"   WAITING for cseq {pc}" if pc else ""))[: mw - 10],
             "warn" if pc else ("ok" if rc_ else "dim"), 10)
        c.kv(y + 5, xx, "link", (f"{'up' if jv.get('link') else 'DOWN'}   5 msg/s cap: {(ocs or {}).get('msg_rate')} used")[: mw - 10], "ok" if jv.get("link") else "bad", 10)

    def network_small(self, c: Canvas, y: int, x: int, h: int, w: int, now: float) -> None:
        """The compact NETWORK box of MAIN: brokers, nodes per broker, links (the NETWORK tab has the detail)."""
        m = self.m
        with m.lock:
            hub_up, boat_up, bridge = m.hub_up, m.boat_up, m.bridge
            st, ocs, mp, ph = m.status, m.ocs, m.map, m.perc_health
            status_t, ocs_t, map_t, perc_t = m.status_t, m.ocs_t, m.map_t, m.perc_t
        c.box(y, x, h, w, "NETWORK  (NETWORK tab for the detail)")
        xx, mw = x + 2, w - 4
        c.put(y + 1, xx, f"hub  {'UP' if hub_up else 'DOWN'}  {self._sys('hub', 'clients/connected')} cl  {self._sys('hub', 'load/messages/received/1min')}/{self._sys('hub', 'load/messages/sent/1min')} msg/min"[: mw - 12],
              "ok" if hub_up else "bad")
        br = "bridge up" if bridge else ("bridge DOWN" if bridge is False else "bridge ?")
        c.put(y + 1, x + w - 2 - len(br), br, "ok" if bridge else "bad")
        c.put(y + 2, xx, f"boat {'UP' if boat_up else 'n/a'}  {self._sys('boat', 'clients/connected')} cl  {self._sys('boat', 'load/messages/received/1min')}/{self._sys('boat', 'load/messages/sent/1min')} msg/min"[: mw],
              "ok" if boat_up else "bad")
        for row, broker in ((y + 3, "hub"), (y + 4, "boat")):
            c.put(row, xx, f"{broker:5s}", "dim")
            cx = xx + 5
            nodes = m.nodes(broker, now)
            if not nodes:
                c.put(row, cx, "no presence beats yet"[: mw - 5], "dim")
            for name, up, age, body in nodes:
                short = name.replace("fake_", "f_").replace("mission_manager", "mm").replace("cv_publish", "cv_pub")
                txt = f"{short} {age:.0f}s" if up else f"{short} DOWN"
                if cx + len(txt) > x + w - 2:
                    c.put(row, cx, "..", "dim")
                    break
                c.put(row, cx, txt, "ok" if up else "dim")
                cx += len(txt) + 2
        fa = ((st or {}).get("health") or {}).get("fcu_age_s")
        links = [("MAVLink", f"{fa} s" if fa is not None else "none", fa is not None and fa < 1.0),
                 ("status", f"{now - status_t:.1f} s" if st else "none", bool(st) and now - status_t < 3),
                 ("ocs/state", f"{now - ocs_t:.1f} s" if ocs else "none", bool(ocs) and now - ocs_t < 3),
                 ("judge", "up" if (ocs or {}).get("judge_connected") else "DOWN", bool((ocs or {}).get("judge_connected"))),
                 ("perception", f"{now - perc_t:.0f} s" if ph else "none", bool(ph) and now - perc_t < 3),
                 ("map", f"{now - map_t:.0f} s" if mp else "none", bool(mp) and now - map_t < 3),
                 ("UAV relay", "none", None)]
        cx = xx
        c.put(y + 5, cx, "links", "dim"); cx += 6
        for name, val, ok in links:
            txt = f"{name} {val}"
            if cx + len(txt) > x + w - 2:
                break
            c.put(y + 5, cx, txt, "dim" if ok is None else ("ok" if ok else "bad"))
            cx += len(txt) + 2

    def warnings_box(self, c: Canvas, y: int, x: int, h: int, w: int, now: float) -> None:
        items = self.m.warnings(now)
        c.box(y, x, h, w, "WARNINGS  (active; self-clearing)")
        if items:
            c.put(y, x, "┌", "bad")
        if not items:
            c.put(y + 1, x + 2, "none", "ok")
        for i, (age, text, attr) in enumerate(items[: h - 2]):
            c.put(y + 1 + i, x + 2, f"{age:4.0f} s  {text}"[: w - 4], attr)
        if len(items) > h - 2:
            c.put(y + h - 2, x + w - 8, f"+{len(items) - (h - 2)}", "bad")

    def commands_box(self, c: Canvas, y: int, x: int, h: int, w: int, ocs, st, ui_act) -> None:
        """Every typed word in six groups; green = the playbook predicts it passes now, dim = not now, yellow = bench / plant."""
        c.box(y, x, h, w, "COMMANDS  (green = allowed now)")
        verdicts = {vd.action: vd for vd in playbook.allowed_now(ocs, self.vehicle_id, st, ui_active=ui_act)}

        def style(word: str) -> str:
            act = WORD_TO_ACTION.get(word)
            if act:
                vd = verdicts.get(act)
                return "ok" if (vd and vd.ok) else "dim"
            if word in PLANT_WORDS or word.startswith("inject"):
                return "warn"
            if word in ("ready", "manual", "mission", "state", "next"):
                return "ok" if ocs else "dim"
            return ""
        maxw = w - 13
        row = 0
        for label, words in COMMAND_GROUPS:
            line: List[Tuple[str, str]] = []
            cur = 0
            first = True
            for wd in words:
                add = len(wd) + (3 if line else 0)
                if cur + add > maxw and line:
                    if row >= h - 2:
                        return
                    _emit_words(c, y + 1 + row, x, label if first else "", line)
                    row += 1
                    first = False
                    line, cur = [], 0
                line.append((wd, style(wd)))
                cur += len(wd) + (3 if len(line) > 1 else 0)
            if line and row < h - 2:
                _emit_words(c, y + 1 + row, x, label if first else "", line)
                row += 1

    # -- MISSION tab --
    def mission_screen(self, c: Canvas, log_scroll: int) -> None:
        m = self.m
        with m.lock:
            ocs, st, status_t = m.ocs, m.status, m.status_t
        w, h = c.w, c.h
        now = time.time()
        ui_act = m.ui_state(now)
        self.banner(c)
        veh = ((ocs or {}).get("vehicles") or {}).get(self.vehicle_id) or {}
        v = (st or {}).get("state") or veh.get("state")
        running = (st or {}).get("task_name") if v in ("AUTO_TASK", "AUTO_SAFE_HOLD") else None
        mi = (ocs or {}).get("mission") or {}
        order = playbook.MISSION_ORDER
        cur_task = order[self.cursor % len(order)]                      # a mission code (m4t0 ...)
        rcode = playbook.code_of(running) if running else None            # the running wire task as a code
        pcode = (mi.get("code") or playbook.code_of(mi.get("task"), mi.get("tier")) or "").replace("sim", "") if mi.get("task") else ""
        top = min(30, h - 4 - 2 - 8)
        lw = 48
        c.box(2, 0, top, lw, "MISSIONS  (Up/Down highlight; Enter = select command)")
        c.put(3, 2, "code   wire  what                       tier", "dim")
        for i, k in enumerate(order):
            ms = playbook.MISSIONS[k]
            yy = 4 + i * 2
            if yy + 1 >= 2 + top - 1:
                break
            attr = {"built": "ok", "planned": "dim", "bench": "warn", "test": "ok"}[ms.status]
            c.put(yy, 2, f"{k:<6} {ms.task[:5]:<5} {ms.title[:25]:<25} {ms.tier[:5]}"[: lw - 4], "now" if k == cur_task else attr)
            flag = "  * running" if rcode == k else ("  preselected" + (" (sim)" if mi.get("sim") else "") if pcode == k else "")
            c.put(yy + 1, 15, (ms.status + flag)[: lw - 17], "warn" if flag else "dim")
        yy = 4 + len(order) * 2 + 1
        c.put(yy, 2, f"live         {rcode + ' ' + str((st or {}).get('task_state')) if running else 'no task running'}"[: lw - 4], "warn" if running else "dim")
        c.put(yy + 1, 2, f"preselected  {mi.get('code') or pcode or '-'}{' --auto' if mi.get('auto') else ''}"[: lw - 4], "")
        c.put(yy + 2, 2, "m<task>t<tier>: m4t0 = Task 4 core; +sim = fake FCU only", "dim")
        c.put(yy + 3, 2, "green built  grey planned  yellow bench", "dim")
        c.put(yy + 4, 2, "Enter puts the select command in the prompt", "dim")
        c.put(yy + 5, 2, "T2 and the UUV: not attempted (TIER_NONE)", "dim")
        # the highlighted mission
        rx = lw + 1
        rw = w - rx - 1
        ms = playbook.MISSIONS[cur_task]
        c.box(2, rx, top, rw, f"{cur_task}  -  {ms.title} ({ms.task} {ms.tier})" + (f"   [{ms.status}]" if ms.status != "built" else "") +
              (f"   * running, {(st or {}).get('task_state')}" if rcode == cur_task else ""))
        x = rx + 2
        mw = rw - 4
        c.kv(3, x, "what", ms.what[0][: mw - 8], "", 8)
        c.put(4, x + 8, (ms.what[1] if len(ms.what) > 1 else "")[: mw - 8], "")
        c.kv(5, x, "starts", ms.starts[: mw - 8], "", 8)
        c.kv(6, x, "ends", ms.ends[: mw - 8], "", 8)
        c.kv(7, x, "judge", f"in: {ms.judge_in}   out: {ms.judge_out}"[: mw - 8], "dim", 8)
        c.kv(8, x, "config", ms.config[: mw - 8], "dim", 8)
        ty = 10
        c.put(ty, x, f"{'stage':<13} {'meaning':<34} {'behaviour':<11} {'leaves on':<23} judge"[: mw], "title")
        cur_stage = (st or {}).get("task_state") if rcode == cur_task else None
        passed = cur_stage is not None
        for i, stg in enumerate(ms.stages):
            if ty + 1 + i >= 2 + top - 2:
                break
            if stg.name == cur_stage:
                attr, passed = "now", False
            else:
                attr = "ok" if passed else "dim"
            c.put(ty + 1 + i, x, f" {stg.name:<12}"[:14], attr)
            c.put(ty + 1 + i, x + 14, f"{stg.meaning[:33]:<34} {stg.behaviour[:11]:<11} {stg.leaves_on[:23]:<23} {stg.judge}"[: mw - 14], "" if attr == "now" else "dim")
        ny = ty + 1 + min(len(ms.stages), 11)
        if rcode == cur_task:
            c.put(ny, x, f"now      {cur_stage}: {playbook.task_stage_meaning(ms.task, cur_stage)}"[: mw], "warn")
        else:
            c.put(ny, x, f"now      {cur_task} is not running" + (f" ({rcode} is, in {(st or {}).get('task_state')})" if running else "") + " - the stages light up once it runs", "dim")
        if ms.note:
            c.put(ny + 1, x, ("note     " + ms.note)[: mw], "dim")
        # how to select
        hy = ny + 3
        hh = 2 + top - hy - 1
        if hh >= 4:
            steps, needs = playbook.how_to_select(cur_task, ocs, self.vehicle_id, ui_act, st)
            state_txt = f"{(ocs or {}).get('r') or '--'}, vehicle {v or '?'}" + (f" ({running})" if running else "") + (", usv_ui ACTIVE" if ui_act else ", handed over" if ui_act is False else "")
            c.box(hy, rx + 1, hh, rw - 2, f"HOW TO SELECT {cur_task}  from this state: {state_txt}"[: rw - 6])
            hx = rx + 3
            c1 = min(46, (rw - 6) // 2 + 4)
            for i, (num, text, cmd, note) in enumerate(steps[: hh - 3]):
                c.put(hy + 1 + i, hx, num, "warn" if num not in ("boat", "*", "-") else "title")
                c.put(hy + 1 + i, hx + 5, text[: (c1 - 6) if (cmd or note) else (rw - 10)], "")
                if cmd:
                    c.put(hy + 1 + i, hx + c1, cmd[: rw - 8 - c1], "ok" if not cmd.startswith("inject") else "warn")
                    if note and hx + c1 + len(cmd) + 3 + len(note) < rx + rw - 3:
                        c.put(hy + 1 + i, hx + c1 + len(cmd) + 3, note, "dim")
                elif note:
                    c.put(hy + 1 + i, hx + c1, note[: rw - 8 - c1], "dim")
            if needs:
                cx = hx
                c.put(hy + hh - 2, cx, "needs", "title"); cx += 7
                for label, ok in needs:
                    txt = f"{label} {'ok' if ok else ('?' if ok is None else 'NO')}"
                    if cx + len(txt) > rx + rw - 3:
                        break
                    c.put(hy + hh - 2, cx, txt, "dim" if ok is None else ("ok" if ok else "bad"))
                    cx += len(txt) + 3
        # all missions at a glance
        y2 = 2 + top
        c.box(y2, 0, h - 4 - y2, w - 1, "ALL MISSIONS AT A GLANCE  (stages in order; grey = not built yet)")
        yy = y2 + 1
        for k in order:
            ms2 = playbook.MISSIONS[k]
            if yy + 1 >= h - 5:
                break
            attr = {"built": "ok", "planned": "dim", "bench": "warn", "test": "ok"}[ms2.status]
            c.put(yy, 2, f"{k:<6} {ms2.task[:5]}", attr)
            c.put(yy, 14, " -> ".join(s.name for s in ms2.stages)[: w - 16], "" if attr != "dim" else "dim")
            c.put(yy + 1, 14, (ms2.note or f"{ms2.starts}; {ms2.judge_out}")[: w - 16], "dim")
            yy += 2

    def flow_strip(self, c: Canvas, y: int, x: int, w: int, ocs: Optional[dict], ui_act: Optional[bool]) -> None:
        """One line: usv_ui ACTIVE > handover > MANUAL (mm) > declare > enable_auto > start_task / goto > hold <-> resume > end_run >
        takeover / stop, the current stage in reverse video, the FCU holder named at the end."""
        stage = playbook.flow_stage(ocs, self.vehicle_id, ui_act)
        who = "FCU: usv_ui" if ui_act else ("FCU: mission manager" if ui_act is False else "FCU: ? (boat broker)")
        c.put(y, x, "FLOW ", "title")
        parts = playbook.flow_line(stage)
        n = sum(len(t) for t, _ in parts)
        if 5 + n + 2 + len(who) > w:                        # tight: drop the spaces around the arrows, then the FCU note
            parts = playbook.flow_line(stage, ">")
            n = sum(len(t) for t, _ in parts)
        show_who = 5 + n + 2 + len(who) <= w
        cx = x + 5
        for text, on in parts:
            if cx + len(text) > x + w:
                break
            c.put(y, cx, text, "bar" if on else ("dim" if text.strip(" >") in playbook.TRANSITIONS or text.strip() == ">" else ""))
            cx += len(text)
        if show_who:
            c.put(y, x + w - len(who), who, "warn" if ui_act else ("ok" if ui_act is False else "dim"))

    def vehicle_box(self, c: Canvas, y: int, x0: int, w: int, st: Optional[dict], status_t: float, ocs_state: Optional[str], now: float) -> None:
        """The V ladder, flags, task and fault of status/<v>, plus what the state and the task stage mean (shared by MAIN and BOAT)."""
        c.box(y, x0, 9, w, f"VEHICLE {self.vehicle_id}")
        x = x0 + 2
        v = (st or {}).get("state") or ocs_state
        _ladder(c, y + 1, x, V_ORDER, v, playbook.V_CODE)
        st_ok = bool(st) and now - status_t < 3
        c.kv(y + 2, x, "state", f"{playbook.V_CODE.get(v, '--')} {v or 'no status'}   {(st or {}).get('robot_state') or ''}", "warn" if st_ok else "bad", 7)
        c.put(y + 2, min(x + 44, x0 + w - 15), f"status {_age(status_t)}"[: max(0, x0 + w - min(x + 44, x0 + w - 15) - 1)], "ok" if st_ok else "bad")
        armed = (st or {}).get("armed")
        c.put(y + 3, x, " ARMED " if armed else " disarmed ", "armed" if armed else "disarmed")
        kill = (st or {}).get("kill")
        estop = (st or {}).get("estop")
        c.put(y + 3, x + 12, " KILL " if kill else " kill n ", "armed" if kill else "dim")
        c.put(y + 3, x + 21, " ESTOP " if estop else " estop n ", "armed" if estop else "dim")
        c.put(y + 3, x + 31, f"mode {(st or {}).get('mode')}", "warn")
        c.kv(y + 4, x, "task", f"{(st or {}).get('current_task') or '-'}  {(st or {}).get('task_name') or ''} {(st or {}).get('task_state') or ''}".rstrip(), "bold", 7)
        fault, hold = (st or {}).get("fault"), (st or {}).get("hold_reason")
        c.kv(y + 5, x, "fault", f"{fault or '-'}", "bad" if fault else "dim", 7)
        c.put(y + 5, x + 24, f"hold {hold or '-'}", "warn" if hold else "dim")
        wmax = max(10, x0 + w - x - 2)
        c.kv(y + 6, x, "means", (playbook.V_MEANING.get(v, "") if v else "no status from the vehicle")[:wmax - 7], "dim", 7)
        tn, ts_ = (st or {}).get("task_name"), (st or {}).get("task_state")
        stage = playbook.task_stage_meaning(tn, ts_) if tn else ""
        c.kv(y + 7, x, "stage", (f"{tn} {ts_}: {stage}" if stage else (f"{tn} {ts_ or ''}" if tn else "no task"))[:wmax - 7], "warn" if tn else "dim", 7)

    # -- BOAT: the boat's own view from shore --
    def boat(self, c: Canvas, log_scroll: int) -> None:
        m = self.m
        with m.lock:
            ocs, st, loc, mp, mp_t, mp_src = m.ocs, m.status, m.local, m.map, m.map_t, m.map_src
            maps = dict(m.maps)
            perc, perc_t = m.perc_health, m.perc_t
            status_t, local_t = m.status_t, m.local_t
            pres = {n: (b, t) for (br, n), (b, t) in m.presence.items() if br == "boat"}
        w, h = c.w, c.h
        now = time.time()
        veh = ((ocs or {}).get("vehicles") or {}).get(self.vehicle_id) or {}
        lw = 50
        self.vehicle_box(c, 1, 0, lw, st, status_t, veh.get("state"), now)
        # POSITION / CO
        c.box(1, lw + 1, 9, w - lw - 2, "POSITION / CONTROL ORIGIN")
        x = lw + 3
        c.kv(2, x, "lat", _num((st or {}).get("lat"), 7), "bold", 5)
        c.put(2, x + 22, f"lon {_num((st or {}).get('lon'), 7)}", "bold")
        c.kv(3, x, "hdg", f"{_num((st or {}).get('hdg_deg'), 1)} deg", "", 5)
        c.put(3, x + 18, f"sog {_num((st or {}).get('sog_mps'), 2)} m/s", "")
        lcl = (st or {}).get("local") or {}
        c.put(4, x, f"local N {_num(lcl.get('x_n'), 1)} E {_num(lcl.get('y_e'), 1)}", "co" if lcl else "dim")
        co = (mp or {}).get("co") or {}
        c.put(4, x + 26, f"CO {_num(co.get('lat'), 6)} {_num(co.get('lon'), 6)}" if co else "CO: no map digest yet", "co" if co else "dim")
        f = (loc or {}).get("fcu") or {}
        if loc and now - local_t < 3:
            c.kv(5, x, "fcu", f"gps {f.get('gps_fix')}/{f.get('sats')}  ekf {'ok' if f.get('ekf_pos_ok') else 'BAD'}  batt {_num(f.get('batt_v'), 1)} V  rc6 {(f.get('rc') or {}).get('6', (f.get('rc') or {}).get(6, '-'))}", "", 5)
        else:
            c.kv(5, x, "fcu", "local/usv/mm/state not reachable (boat broker)" if not m.boat_up else "waiting for local/usv/mm/state", "dim", 5)
        # who has the FCU
        ui = pres.get("usv_ui")
        ui_up = bool(ui) and bool(ui[0].get("up")) and now - ui[1] < 6.0
        ts = (maps.get("usv_ui") or (None, 0.0))
        ui_mode = (ts[0] or {}).get("task_state") if ts[0] and now - ts[1] < 3.0 else None
        if ui_up and ui_mode == "ACTIVE":
            who, attr = "usv_ui ACTIVE: it commands the FCU (ENABLE_AUTO refused until `handover` is typed there)", "warn"
        elif ui_up and ui_mode in ("HANDED_OVER", "RELEASED"):
            who, attr = "usv_ui handed over -> the mission manager commands (ocs_cli ready / goto / start_task)", "ok"
        elif ui_up:
            who, attr = "usv_ui up (no map digest yet)", "dim"
        else:
            who, attr = "mission manager (no usv_ui on the boat broker)", "ok" if m.boat_up else "dim"
        c.kv(6, x, "FCU", who[: w - lw - 10], attr, 5)
        # MAP (text) | PERCEPTION column
        y0, cw = 10, 44
        mw = w - cw - 1
        mh = h - 4 - y0 - 5
        c.box(y0, 0, mh, mw, "MAP  N up, E right   + CO   A.. points   R G buoys  D dock  | gate  = dock line   arrow = boat"[: mw - 4])
        if not mp:
            c.put(y0 + 2, 3, "no telemetry/usv/map yet: the mission manager publishes it at 2 Hz once the FCU has a fix; usv_ui too", "dim")
        else:
            vehicle, trail, path, carrot, points, entities, gates, dock = digest_layers(mp)
            grid, m10 = render_text_map(mw - 2, mh - 2, points, vehicle, carrot, trail, path=path, entities=entities, gates=gates, dock=dock)
            for r, row in enumerate(grid):
                col = 0
                while col < len(row):
                    style = row[col][1]
                    j = col
                    seg = ""
                    while j < len(row) and row[j][1] == style:
                        seg += row[j][0]
                        j += 1
                    c.put(y0 + 1 + r, 1 + col, seg, style)
                    col = j
            c.put(y0 + mh - 1, 3, f" |<--------->| {m10:.1f} m ", "dim")
        px = mw + 3
        c.box(y0, mw + 1, mh, cw, "PERCEPTION / MAP")
        cwi = cw - 4
        c.put(y0 + 1, px, f"source {mp_src or '-'}  {_age(mp_t) if mp else 'no digest'}"[:cwi], "ok" if (mp and now - mp_t < 3) else "dim")
        p_ok = bool(perc) and now - perc_t < 3
        det = (perc or {}).get("detector") or "-"
        c.put(y0 + 2, px, f"detector {det} {'ok' if (perc or {}).get('ok') else 'NOT ok'} {_age(perc_t) if perc else '(none)'}"[:cwi], "ok" if p_ok else "dim")
        c.put(y0 + 3, px, f"rate {_num((perc or {}).get('rate_hz'), 1)} Hz  pose age {_num((perc or {}).get('pose_age_s'), 2)} s  n {(perc or {}).get('n_published', '-')}"[:cwi], "" if p_ok else "dim")
        cv = (mp or {}).get("cv")
        if cv:
            c.put(y0 + 4, px, f"usv_ui cv {'ON' if cv.get('on') else 'off'}  rx {cv.get('n_rx')} used {cv.get('n_used')}"[:cwi], "ok" if cv.get("on") else "dim")
        else:
            c.put(y0 + 4, px, "mission manager map (always ingesting)"[:cwi] if mp else "", "dim")
        ents = (mp or {}).get("entities") or []
        n_tr = sum(1 for e in ents if e.get("state") == "tracked")
        c.put(y0 + 5, px, f"entities {n_tr} tracked / {len(ents)}  ingested {(mp or {}).get('n_ingested', '-')}"[:cwi], "bold" if ents else "dim")
        gs = (mp or {}).get("gates") or []
        c.put(y0 + 6, px, (f"gates {len(gs)}" + (f"  next thr {gs[0].get('through_hdg_deg', 0):.0f} deg w {gs[0].get('width_m', 0):.1f} m" if gs else ""))[:cwi], "gate" if gs else "dim")
        d = (mp or {}).get("dock")
        c.put(y0 + 7, px, (f"dock N {_num(d.get('x_n'), 1)} E {_num(d.get('y_e'), 1)} normal {_num(d.get('psi_n'), 0)} faces {len(d.get('faces') or [])}" if d else "dock: none")[:cwi], "gate" if d else "dim")
        c.put(y0 + 9, px, f"{'#':>3} {'kind':<5} {'colour':<7} {'N':>6} {'E':>6} {'sig':>3} {'n':>3} {'age':>4}"[:cwi], "title")
        for i, e in enumerate(ents[: mh - 12]):
            attr = "ent_red" if e.get("colour") == "red" else "ent_green" if e.get("colour") == "green" else ""
            if e.get("state") != "tracked":
                attr = "ent_stale"
            c.put(y0 + 10 + i, px, f"{e.get('id'):>3} {str(e.get('kind'))[:5]:<5} {str(e.get('colour'))[:7]:<7} {_num(e.get('x_n'), 1):>6} {_num(e.get('y_e'), 1):>6} "
                                   f"{_num(e.get('sigma_m'), 1):>3} {e.get('n_obs', 0):>3} {_num(e.get('age_s'), 0):>4}"[:cwi], attr)
        # bottom strip: the pool tool's own line + commands
        yb = y0 + mh
        c.box(yb, 0, 5, w - 1, "USV_UI / RUN / FLOW")
        task = (mp or {}).get("task") if mp_src == "usv_ui" else None
        c.put(yb + 1, 2, f"usv_ui: {task or '-'}   mode {(mp or {}).get('mode', '-')}  {'ARMED' if (mp or {}).get('armed') else 'disarmed'}" if mp_src == "usv_ui" else
              "usv_ui: not publishing (its map digest carries bursts / goto / path / sway state when it runs)", "bold" if mp_src == "usv_ui" else "dim")
        r = (ocs or {}).get("r")
        c.put(yb + 2, 2, f"run: {r or '--'} {(ocs or {}).get('r_state') or ''}   run_id {(ocs or {}).get('run_id')}   vehicle {veh.get('state') or '-'}", "")
        self.flow_strip(c, yb + 3, 2, w - 4, ocs, m.ui_state(now))

    # -- NETWORK --
    def _sys(self, broker: str, key: str) -> str:
        with self.m.lock:
            return self.m.sys.get((broker, key), "-")

    def network_box(self, c: Canvas, y: int, x: int, h: int, w: int, compact: bool = False) -> None:
        m = self.m
        now = time.time()
        with m.lock:
            hub_up, boat_up, bridge = m.hub_up, m.boat_up, m.bridge
            st, ocs, mp, ph = m.status, m.ocs, m.map, m.perc_health
            status_t, ocs_t, map_t, perc_t = m.status_t, m.ocs_t, m.map_t, m.perc_t
        c.box(y, x, h, w, "NETWORK  (brokers, nodes, links; NETWORK tab for the detail)")
        # brokers
        line = f"hub {'UP' if hub_up else 'DOWN'}  clients {self._sys('hub', 'clients/connected')}  rx {self._sys('hub', 'load/messages/received/1min')}/min  tx {self._sys('hub', 'load/messages/sent/1min')}/min"
        c.put(y + 1, x + 2, line, "ok" if hub_up else "bad")
        line2 = f"boat {'UP' if boat_up else 'n/a'}  clients {self._sys('boat', 'clients/connected')}  rx {self._sys('boat', 'load/messages/received/1min')}/min  tx {self._sys('boat', 'load/messages/sent/1min')}/min"
        c.put(y + 1, x + 52, line2, "ok" if boat_up else "bad")
        br = "bridge up" if bridge else ("bridge DOWN" if bridge is False else "bridge ?")
        c.put(y + 1, x + 100, br, "ok" if bridge else "bad")
        # nodes per broker
        for row, broker in ((y + 2, "hub"), (y + 3, "boat")):
            c.put(row, x + 2, f"{broker:5s}", "dim")
            cx = x + 8
            nodes = m.nodes(broker, now)
            if not nodes:
                c.put(row, cx, "no presence beats yet", "dim")
            for name, up, age, body in nodes:
                txt = f"{name} {age:.0f}s" if up else f"{name} DOWN {age:.0f}s"
                c.put(row, cx, txt, "ok" if up else "bad")
                cx += len(txt) + 3
                if cx > x + w - 20:
                    break
        # links
        fa = ((st or {}).get("health") or {}).get("fcu_age_s")
        links = [("MAVLink", f"fcu {fa} s" if fa is not None else "no status", fa is not None and fa < 1.0),
                 ("status->hub", f"{now - status_t:.1f} s" if st else "none", bool(st) and now - status_t < 3),
                 ("ocs/state", f"{now - ocs_t:.1f} s" if ocs else "none", bool(ocs) and now - ocs_t < 3),
                 ("judge", "up" if (ocs or {}).get("judge_connected") else "DOWN", bool((ocs or {}).get("judge_connected"))),
                 ("perception", f"{now - perc_t:.1f} s" if ph else "none", bool(ph) and now - perc_t < 3),
                 ("map", f"{len((mp or {}).get('entities') or [])} ent {len((mp or {}).get('gates') or [])} gates {'dock' if (mp or {}).get('dock') else 'no dock'} {now - map_t:.1f} s" if mp else "none", bool(mp) and now - map_t < 3)]
        cx = x + 2
        for name, val, ok in links:
            txt = f"{name} {val}"
            c.put(y + 4, cx, txt, "ok" if ok else "bad")
            cx += len(txt) + 3

    def network(self, c: Canvas, log_scroll: int) -> None:
        w, h = c.w, c.h
        self.network_box(c, 1, 0, 6, w - 1)
        now = time.time()
        c.box(7, 0, 12, w - 1, "nodes  (presence/<node> retained beats every 2 s; down = last beat older than 6 s or a clean stop)")
        c.put(8, 2, f"{'broker':6s} {'node':20s} {'state':6s} {'beat':8s} {'host':24s} {'pid':8s} {'vehicle':8s}", "dim")
        row = 9
        for broker in ("hub", "boat"):
            for name, up, age, body in self.m.nodes(broker, now):
                if row >= 18:
                    break
                c.put(row, 2, f"{broker:6s} {name:20s} {'UP' if up else 'DOWN':6s} {age:6.1f} s {str(body.get('host', ''))[:23]:24s} {str(body.get('pid', '')):8s} {str(body.get('vehicle_id', ''))[:8]:8s}",
                      "ok" if up else "bad")
                row += 1
        c.box(19, 0, h - 4 - 19, w - 1, "brokers  ($SYS)  and what each link carries")
        y = 20
        for broker in ("hub", "boat"):
            keys = [k for (b, k) in self.m.sys if b == broker]
            c.put(y, 2, f"{broker}: " + "  ".join(f"{k.split('/')[-2] if '/' in k else k}={self._sys(broker, k)}" for k in sorted(keys))[: w - 6], "")
            y += 1
        for ln in ("boat -> hub (bridge): status/usv 2 Hz QoS 0, telemetry/usv/map 2 Hz, ack/usv QoS 1, event/usv/# QoS 1, relay/usv/# (UAV copies)",
                   "hub -> boat (bridge): cmd/usv/# QoS 1, cmd/uav/# QoS 1, event/uav/# QoS 1",
                   "boat only (never bridged): local/usv/mm/state 5 Hz, local/usv/perception/* (fake_perception), local/usv/tui/active",
                   "hub only: ocs/state 1 Hz, ocs/alert, ocs/intent_result, ui/cmd/#, rc/* (fake judge), sim/truth, presence/#",
                   "MAVLink: fake FCU udp 14550 -> mission manager; udp 14560 JSON control (bench); the fake FCU is not an MQTT client"):
            if y < h - 4:
                c.put(y, 2, ln[: w - 4], "dim")
                y += 1

    # -- JUDGE --
    def judge(self, c: Canvas, log_scroll: int) -> None:
        m = self.m
        with m.lock:
            stats, jl = m.stats, list(m.judge_log)
        w, h = c.w, c.h
        c.box(1, 0, 9, w - 1, "RoboCommand (fake) statistics  rc/sim/stats, retained 1 Hz")
        if stats:
            y = 2
            for k in ("declarations", "run_starts", "heartbeats", "last_state", "last_task", "states_seen", "tasks_seen", "incident_acks", "readiness", "confirms",
                      "rate_violations", "max_rate"):
                if k in stats and y < 9:
                    c.kv(y, 2, k, _short(stats[k], w - 24), "bold" if k in ("last_state", "last_task") else "", 18)
                    y += 1
        else:
            c.put(2, 2, "no rc/sim/stats on this hub", "dim")
        c.box(10, 0, h - 4 - 10, w - 1, "judge traffic  (-> commands to the team, <- requests and reports from the OCS)")
        n = h - 4 - 10 - 2
        now = time.time()
        for i, (t, text) in enumerate(jl[-max(1, n):]):
            c.put(11 + i, 2, f"{now - t:4.0f}s {text}"[: w - 4], "judge")

    # -- LOG --
    def logscreen(self, c: Canvas, log_scroll: int) -> None:
        with self.m.lock:
            log = list(self.m.log)
        w, h = c.w, c.h
        c.box(1, 0, h - 4 - 1, w - 1, f"LOG  (PgUp/PgDn scroll, newest last; {len(log)} lines; clear empties it)")
        n = h - 4 - 1 - 2
        end = max(1, len(log) - log_scroll)
        show = log[max(0, end - n): end]
        now = time.time()
        for i, (t, kind, text) in enumerate(show):
            attr = {"alert": "bad", "ack": "ok", "event": "warn", "judge": "judge", "intent": "title", "cmd": "cmd", "bridge": "bad"}.get(kind, "")
            if ("REJECTED" in text or "FAILED" in text or "refused" in text):
                attr = "bad"
            c.put(2 + i, 2, f"{time.strftime('%H:%M:%S', time.localtime(t))} {now - t:4.0f}s {text}"[: w - 4], attr)

    # -- HELP --
    def helpscreen(self, c: Canvas, log_scroll: int) -> None:
        w, h = c.w, c.h
        c.box(1, 0, h - 4 - 1, w - 1, "HELP  (the playbook: intent, OCS R states, vehicle V states, needs / then / next)")
        lines = ["commands are typed at the prompt; the COMMANDS panel marks what the playbook predicts will pass now",
                 "Tab / Shift-Tab screens   Esc clear the line   Ctrl-L repaint   PgUp/PgDn scroll the log   quit leaves (bench keeps running)", ""]
        lines += playbook.help_lines()
        lines += ["", "plant (fake FCU) words: kill on|off   rc6 on|off   estop on|off   fcu pause [s]   ekf on|off   armed on|off   teleport lat lon [hdg]",
                  "map = the browser map (tools/map.html over the hub websocket); BOAT tab = the same map as text + detector + who has the FCU",
                  "", "COMMANDS groups (MAIN): " + "   ".join(f"{g}: {' '.join(ws)}" for g, ws in COMMAND_GROUPS),
                  "MISSION tab: Up/Down highlight a mission, Enter puts its select command in the prompt; WARNINGS box = active conditions with their age",
                  "EVENTS = what happened (state changes, judge, links); CONSOLE = your commands with their result and hint; LOG tab = everything"]
        for i, ln in enumerate(lines[: h - 4 - 3]):
            c.put(2 + i, 2, ln[: w - 4], "bold" if ln.startswith("intent ") else "")


BOX_CHARS = {"\u2500": "ACS_HLINE", "\u2502": "ACS_VLINE", "\u250c": "ACS_ULCORNER", "\u2510": "ACS_URCORNER",
             "\u2514": "ACS_LLCORNER", "\u2518": "ACS_LRCORNER", "\u251c": "ACS_LTEE", "\u2524": "ACS_RTEE",
             "\u252c": "ACS_TTEE", "\u2534": "ACS_BTEE"}


# ---- the curses app ------------------------------------------------------------------------------------------------
class App:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.m = Model(a.vehicle_id, a.vehicle)
        self.feeds = Feeds(self.m, a.host, a.port, a.boat_host, a.boat_port)
        self.con = ocs_cli.OcsConsole(a.host, a.port, vehicle_id=a.vehicle_id, vehicle=a.vehicle, operator="tui")
        self.view = View(self.m, a.vehicle_id)
        self.busy = threading.Lock()
        self.cmdline = ""
        self.last = "type a command; help for the playbook"
        self.screen = 0
        self.log_scroll = 0
        self.C: Dict[str, int] = {}
        self.quit = False

    # -- commands --
    def do_command(self, line: str) -> None:
        line = line.strip()
        if not line:
            return
        self.m.add("ui", "> " + line)
        try:
            argv = shlex.split(line)
        except ValueError as exc:
            self.last = f"bad line: {exc}"
            return
        word = ALIAS.get(argv[0].lower(), argv[0].lower())
        argv[0] = word
        if word == "quit":
            self.quit = True
            return
        if word == "clear":
            with self.m.lock:
                self.m.log.clear()
            self.last = "log cleared"
            return
        if word == "help":
            self.screen = SCREENS.index("HELP")
            self.last = "HELP screen (Tab to come back)"
            return
        if word == "map":
            self.last = open_map(self.a.host, self.a.ws_port)
            self.m.add("ui", self.last)
            return
        if word in PLANT_WORDS:
            self.plant(word, argv[1:])
            return
        if word in INTENT_WORDS:
            threading.Thread(target=self.run_cli, args=(argv,), daemon=True).start()
            return
        self.last = f"unknown command {word} - see the COMMANDS panel or help"

    def plant(self, word: str, args: List[str]) -> None:
        msg: Optional[dict] = None
        try:
            if word in ("kill", "rc6", "estop", "ekf", "armed"):
                v = _onoff(args[0]) if args else None
                if v is None:
                    self.last = f"usage: {word} on|off"
                    return
                key = {"kill": "kill", "rc6": "rc6", "estop": "rc5", "ekf": "ekf_ok", "armed": "armed"}[word]
                val = (1900 if v else 1000) if word in ("rc6", "estop") else v
                msg = {"set": {key: val}}
            elif word == "fcu":
                if not args or args[0] != "pause":
                    self.last = "usage: fcu pause [S]"
                    return
                msg = {"pause_stream": float(args[1]) if len(args) > 1 else 4.0}
            elif word == "teleport":
                if len(args) < 2:
                    self.last = "usage: teleport LAT LON [HDG]"
                    return
                msg = {"teleport": {"lat": float(args[0]), "lon": float(args[1]), "hdg": float(args[2]) if len(args) > 2 else 0.0}}
        except ValueError as exc:
            self.last = f"bad number: {exc}"
            return
        if msg is None:
            return
        try:
            fcu_control(self.a.fcu_control, msg)
            self.last = f"plant <- {json.dumps(msg)}"
            self.m.add("ui", self.last)
        except Exception as exc:
            self.last = f"plant control failed: {exc}"

    def run_cli(self, argv: List[str]) -> None:
        if not self.busy.acquire(blocking=False):
            self.last = "busy: the previous command is still waiting for its acks"
            return
        try:
            try:
                ns = ocs_cli.parse(argv)
            except SystemExit:
                u = next((x for w, x, _ in USAGE if w == (argv[0] if argv else "")), None)       # v0.1.30: save_point with no name
                self.last = f"cannot parse: {' '.join(argv)}" + (f"  usage: {u}" if u else "  (usage above the prompt)")
                self.m.add("ui", self.last)
                return
            if ns.cmd in ("mission", "start_task"):              # v0.1.30 (pool report F2): names checked before they are sent
                with self.m.lock:
                    d = self.m.map or {}
                    known = set((d.get("points") or {})) if "points" in d else None
                why = ocs_cli.check_names(ocs_cli.split_names(getattr(ns, "names", None)) + ([ns.centre] if getattr(ns, "centre", None) else []), known)
                if why:
                    self.last = "not sent: " + why
                    self.m.add("ui", self.last)
                    return
            with self.m.lock:
                self.con.state, self.con.status = self.m.ocs, self.m.status
            code, lines = ocs_cli.execute(self.con, ns)
            kept = []
            for ln in lines:
                if ln.startswith("intent ") or ln.startswith("  ack "):     # the feed shows these with their action names
                    continue
                kept.append(ln.strip())
                self.m.add("ui", ln)
            if ns.cmd in ("next", "state"):
                self.last = kept[-1] if kept else ns.cmd
                return
            verdicts = [ln for ln in lines if ln.startswith("intent ") or ln.startswith("  ack ")]
            tail = verdicts[-1].strip() if verdicts else (kept[-1] if kept else "")
            hint = next((ln.strip() for ln in lines if ln.strip().startswith("hint:")), "")
            self.last = (tail + ("   " + hint if hint else ""))[:400]
        except Exception as exc:
            self.last = f"error: {exc}"
        finally:
            self.busy.release()

    # -- keys --
    def poll_keys(self, scr) -> None:
        while True:
            try:
                ch = scr.getch()
            except curses.error:
                return
            if ch == -1:
                return
            if ch in (10, 13, curses.KEY_ENTER):
                if not self.cmdline.strip() and SCREENS[self.screen] == "MISSION":
                    task = playbook.MISSION_ORDER[self.view.cursor % len(playbook.MISSION_ORDER)]
                    self.cmdline = f"mission {task}" if task != "goto" else "goto --x 10 --y 0 --speed 0.6"
                    self.last = f"{self.cmdline}  - Enter sends it, Esc clears"
                    continue
                line, self.cmdline = self.cmdline, ""
                self.do_command(line)
            elif ch == curses.KEY_UP and SCREENS[self.screen] == "MISSION":
                self.view.cursor = (self.view.cursor - 1) % len(playbook.MISSION_ORDER)
            elif ch == curses.KEY_DOWN and SCREENS[self.screen] == "MISSION":
                self.view.cursor = (self.view.cursor + 1) % len(playbook.MISSION_ORDER)
            elif ch in (curses.KEY_BACKSPACE, 127, 8):
                self.cmdline = self.cmdline[:-1]
            elif ch == 9:
                self.screen = (self.screen + 1) % len(SCREENS)
            elif ch == curses.KEY_BTAB:
                self.screen = (self.screen - 1) % len(SCREENS)
            elif ch == curses.KEY_PPAGE:
                self.log_scroll = min(max(0, len(self.m.log) - 5), self.log_scroll + 5)
            elif ch == curses.KEY_NPAGE:
                self.log_scroll = max(0, self.log_scroll - 5)
            elif ch == 27:
                self.cmdline = ""
            elif ch == 12:
                scr.clearok(True)
            elif ch == curses.KEY_RESIZE:
                pass
            elif 32 <= ch < 127:
                self.cmdline += chr(ch)

    # -- painting --
    def init_colors(self) -> None:
        keys = ("ok", "bad", "warn", "title", "dim", "armed", "disarmed", "now", "bar", "bold", "co", "judge", "cmd",
                "ent_red", "ent_green", "ent", "ent_stale", "gate", "path", "trail", "point", "target", "vehicle", "banner")
        self.C = {k: 0 for k in keys}
        self.C["bold"] = self.C["ent"] = self.C["vehicle"] = curses.A_BOLD
        self.C["bar"] = self.C["banner"] = curses.A_REVERSE
        self.C["dim"] = self.C["ent_stale"] = curses.A_DIM
        if not curses.has_colors():
            self.C["armed"] = self.C["now"] = curses.A_REVERSE
            return
        curses.start_color()
        try:
            curses.use_default_colors()
            bg = -1
        except curses.error:
            bg = curses.COLOR_BLACK
        pairs = [(1, curses.COLOR_GREEN, bg), (2, curses.COLOR_RED, bg), (3, curses.COLOR_YELLOW, bg), (4, curses.COLOR_CYAN, bg),
                 (5, curses.COLOR_WHITE, curses.COLOR_RED), (6, curses.COLOR_BLACK, curses.COLOR_GREEN), (7, curses.COLOR_MAGENTA, bg),
                 (8, curses.COLOR_BLUE, bg), (9, curses.COLOR_BLACK, curses.COLOR_CYAN)]
        for n, f, b in pairs:
            curses.init_pair(n, f, b)
        cp = curses.color_pair
        self.C.update({"ok": cp(1) | curses.A_BOLD, "bad": cp(2) | curses.A_BOLD, "warn": cp(3) | curses.A_BOLD, "title": cp(4) | curses.A_BOLD,
                       "armed": cp(5) | curses.A_BOLD, "disarmed": cp(6), "now": cp(6) | curses.A_BOLD, "co": cp(3) | curses.A_BOLD,
                       "judge": cp(7), "cmd": cp(8), "ent_red": cp(2) | curses.A_BOLD, "ent_green": cp(1) | curses.A_BOLD,
                       "gate": cp(7) | curses.A_BOLD, "path": cp(4) | curses.A_BOLD, "trail": cp(1), "point": cp(8) | curses.A_BOLD,
                       "target": cp(4) | curses.A_BOLD, "vehicle": cp(1) | curses.A_BOLD, "banner": cp(9) | curses.A_BOLD})

    def paint(self, scr) -> None:
        h, w = scr.getmaxyx()
        canvas = Canvas(w, h)
        label = f"hub {self.a.host}:{self.a.port} {'UP' if self.m.hub_up else 'down'}  boat :{self.a.boat_port} {'UP' if self.m.boat_up else 'n/a'}  bridge {'up' if self.m.bridge else ('DOWN' if self.m.bridge is False else '?')}"
        self.view.draw(canvas, self.screen, self.cmdline, self.last, self.log_scroll, label)
        scr.erase()
        for y in range(h):
            row, attrs = canvas.rows[y], canvas.attrs[y]
            x = 0
            while x < w - 1:
                a = attrs[x]
                x2 = x
                while x2 < w - 1 and attrs[x2] == a:
                    x2 += 1
                seg = "".join(row[x:x2])
                try:
                    if any(ch in BOX_CHARS for ch in seg):
                        # box borders: ACS line drawing (works on every terminal, incl. ssh without UTF-8)
                        for i, ch in enumerate(seg):
                            acs = BOX_CHARS.get(ch)
                            if acs is not None:
                                scr.addch(y, x + i, getattr(curses, acs), self.C.get(a, self.C["dim"]) if a and a != "dim" else self.C["dim"])
                            else:
                                scr.addstr(y, x + i, ch, self.C.get(a, 0))
                    else:
                        scr.addnstr(y, x, seg, w - x - 1, self.C.get(a, 0))
                except curses.error:
                    pass
                x = x2
        scr.refresh()

    def main(self, scr) -> None:
        curses.curs_set(1)
        scr.nodelay(True)
        scr.keypad(True)
        self.init_colors()
        self.feeds.start()
        self.con.start()
        self.m.add("ui", f"hub {self.a.host}:{self.a.port}, boat {self.a.boat_host}:{self.a.boat_port}, fake FCU control udp {self.a.fcu_control}")
        last_draw = 0.0
        try:
            while not self.quit:
                self.poll_keys(scr)
                now = time.monotonic()
                if now - last_draw >= 0.1:
                    self.paint(scr)
                    last_draw = now
                time.sleep(0.02)
        finally:
            self.feeds.stop()
            self.con.stop()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1882)
    ap.add_argument("--boat-host", default="127.0.0.1")
    ap.add_argument("--boat-port", type=int, default=1884)
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--fcu-control", type=int, default=14560, help="fake FCU UDP control port (bench only)")
    ap.add_argument("--ws-port", type=int, default=9001, help="the hub broker's websocket listener, for the 'map' command")
    ap.add_argument("--map", action="store_true", help="open the browser map at start")
    a = ap.parse_args(argv)
    if a.map:
        print(open_map(a.host, a.ws_port))
    app = App(a)
    curses.wrapper(app.main)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
