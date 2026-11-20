"""JudgeClient: the OCS side of the RoboCommand link (ADR-2). Pluggable:

- FakeJudgeClient: JSON over MQTT on rc/* topics, spoken by sim/fake_robocommand.py (laptop bench).
- (later) ProtobufJudgeClient: robocommand_protocol + mqtt_link from the groundstation repo, same interface.

Both enforce the handbook rate cap (<= 5 msg/s per system) with a token bucket, keep independent report
sequence counters per vehicle and a team-wide request counter, and never retain anything.

v0.1.59 (A1, the compliance pass #4; handbook 3.4.8 / 3.4.15, RoboNation A3): the counters never reset inside a connection - a
RunStart no longer touches them (the run boundary is run_id / declaration_seq) - and they persist per team in `seq_file`
(`logs/ocs/rc_seq.json` from ocs_app; written on every change, read at start), so an ocs_app restart continues the sequence.
The environment's OPTIPRIMEX_RC_SEQ overrides the path; `off` = in memory only (the test suite).
"""
from __future__ import annotations

import json
import logging
import os
import threading
import time
from collections import deque
from typing import Callable, Deque, Dict, Optional

import paho.mqtt.client as mqtt

log = logging.getLogger("optiprimex.judge")

SEQ_FILE_DEFAULT = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "logs", "ocs",
                                "rc_seq.json")


def seq_file_path(default: Optional[str] = SEQ_FILE_DEFAULT) -> Optional[str]:
    """v0.1.59: where the counters persist - OPTIPRIMEX_RC_SEQ (a path, or `off`) beats the default."""
    env = os.environ.get("OPTIPRIMEX_RC_SEQ")
    if env is not None:
        return None if env.strip().lower() in ("", "off", "0", "none") else env
    return default


CourseCb = Callable[[dict], None]
CommandCb = Callable[[dict], None]


class RateCap:
    def __init__(self, per_s: float):
        self.per_s = per_s
        self._times: Deque[float] = deque()
        self._lock = threading.Lock()
        self.dropped = 0

    def allow(self) -> bool:
        now = time.monotonic()
        with self._lock:
            while self._times and now - self._times[0] > 1.0:
                self._times.popleft()
            if len(self._times) >= self.per_s:
                self.dropped += 1
                return False
            self._times.append(now)
            return True

    @property
    def rate(self) -> int:
        now = time.monotonic()
        with self._lock:
            return sum(1 for t in self._times if now - t <= 1.0)


class JudgeClient:
    """Interface. network is 'bench' or 'course'; bench mode is refused on the course network."""
    name = "abstract"
    network = "bench"

    def start(self) -> None: ...
    def stop(self) -> None: ...
    @property
    def connected(self) -> bool: return False
    def on_course(self, cb: CourseCb) -> None: ...
    def on_command(self, cb: CommandCb) -> None: ...
    def on_link(self, cb: Callable[[bool], None]) -> None: ...
    def publish_request(self, body: dict) -> Optional[int]: ...
    def publish_report(self, vehicle_id: str, body: dict) -> Optional[int]: ...
    def new_run(self) -> None: ...


class NullJudgeClient(JudgeClient):
    """No RoboCommand at all: the OCS runs the team side only (R0 forever unless bench injectors move it)."""
    name = "none"

    def __init__(self):
        self._link_cb = None

    def on_course(self, cb): pass
    def on_command(self, cb): pass
    def on_link(self, cb): self._link_cb = cb
    def publish_request(self, body): return None
    def publish_report(self, vehicle_id, body): return None


class FakeJudgeClient(JudgeClient):
    name = "fake"

    def __init__(self, host: str, port: int, team_id: str, rate_cap: float = 5.0, client_id: str = "ocs-judge",
                 network: str = "bench", clean_session: bool = True, seq_file: Optional[str] = None):
        self.host, self.port, self.team_id = host, port, team_id
        self.seq_file = seq_file
        self.network = network
        self.cap = RateCap(rate_cap)
        self._course_cb: Optional[CourseCb] = None
        self._cmd_cb: Optional[CommandCb] = None
        self._link_cb: Optional[Callable[[bool], None]] = None
        self._connected = threading.Event()
        self._req_seq = 0
        self._rep_seq: Dict[str, int] = {}
        self._lock = threading.Lock()
        self._load_seq()
        self.stats = {"requests": 0, "reports": 0, "heartbeats": 0, "commands": 0, "dropped": 0}
        self.last_report: Optional[dict] = None      # v0.1.59 (B5): the last task report (kind, seq, vehicle, time) - not heartbeats
        self.log_cb: Optional[Callable[[dict], None]] = None
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, protocol=mqtt.MQTTv311, clean_session=clean_session)
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = self._on_disconnect
        self.c.on_message = self._on_message
        self.c.reconnect_delay_set(1, 5)

    # ---- lifecycle --------------------------------------------------------------------
    def start(self) -> None:
        self.c.connect_async(self.host, self.port, keepalive=15)
        self.c.loop_start()

    def stop(self) -> None:
        try:
            self.c.disconnect()
        finally:
            self.c.loop_stop()

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def wait_connected(self, timeout_s: float = 5.0) -> bool:
        return self._connected.wait(timeout_s)

    def on_course(self, cb: CourseCb) -> None:
        self._course_cb = cb

    def on_command(self, cb: CommandCb) -> None:
        self._cmd_cb = cb

    def on_link(self, cb: Callable[[bool], None]) -> None:
        self._link_cb = cb

    def new_run(self) -> None:
        """v0.1.59 (A1): a verified RunStart does NOT reset the counters (3.4.8: seq increasing, never reset during the active run;
        the run boundary is run_id / declaration_seq). Kept as the hook ocs_app calls."""

    # ---- the counters' persistence (v0.1.59, A1) -----------------------------------------
    def _load_seq(self) -> None:
        if not self.seq_file:
            return
        try:
            with open(self.seq_file) as f:
                d = (json.load(f) or {}).get(self.team_id) or {}
            self._req_seq = int(d.get("request_seq", 0))
            self._rep_seq = {str(k): int(v) for k, v in (d.get("report_seq") or {}).items()}
            log.info("rc counters continued from %s: request %d, reports %s", self.seq_file, self._req_seq, self._rep_seq)
        except FileNotFoundError:
            pass
        except Exception as e:                         # a broken file never stops the OCS: start from 0, say so
            log.warning("rc counters: %s unreadable (%s) - starting from 0", self.seq_file, e)

    def _save_seq(self) -> None:
        """Called with self._lock held, after every change."""
        if not self.seq_file:
            return
        try:
            try:
                with open(self.seq_file) as f:
                    allt = json.load(f) or {}
            except Exception:
                allt = {}
            allt[self.team_id] = {"request_seq": self._req_seq, "report_seq": dict(self._rep_seq), "t": time.time()}
            os.makedirs(os.path.dirname(self.seq_file) or ".", exist_ok=True)
            tmp = self.seq_file + ".tmp"
            with open(tmp, "w") as f:
                json.dump(allt, f)
            os.replace(tmp, self.seq_file)
        except Exception as e:
            log.warning("rc counters not saved to %s: %s", self.seq_file, e)

    # ---- publishing (rate capped) -------------------------------------------------------
    def publish_request(self, body: dict) -> Optional[int]:
        if not self.cap.allow():
            self.stats["dropped"] += 1
            log.warning("rate cap: request dropped")
            return None
        with self._lock:
            self._req_seq += 1
            seq = self._req_seq
            self._save_seq()
        msg = dict(body, request_seq=seq, team_id=self.team_id, sent_at=time.time())
        self.c.publish(f"rc/request/{self.team_id}", json.dumps(msg), qos=0)
        self.stats["requests"] += 1
        self._log({"dir": "out", "topic": "request", "body": msg})
        return seq

    def publish_report(self, vehicle_id: str, body: dict) -> Optional[int]:
        if not self.cap.allow():
            self.stats["dropped"] += 1
            if body.get("kind") != "heartbeat":
                log.warning("rate cap: report %s dropped", body.get("kind"))
            return None
        with self._lock:
            seq = self._rep_seq.get(vehicle_id, 0) + 1
            self._rep_seq[vehicle_id] = seq
            self._save_seq()
        msg = dict(body, report_seq=seq, vehicle_id=vehicle_id, team_id=self.team_id, sent_at=time.time())
        self.c.publish(f"rc/report/{self.team_id}/{vehicle_id}", json.dumps(msg), qos=0)
        if body.get("kind") == "heartbeat":
            self.stats["heartbeats"] += 1
        else:
            self.stats["reports"] += 1
            self.last_report = {"kind": body.get("kind"), "seq": seq, "vehicle_id": vehicle_id, "t": time.time()}
            self._log({"dir": "out", "topic": "report", "body": msg})
        return seq

    # ---- paho -------------------------------------------------------------------------------
    def _on_connect(self, client, userdata, flags, rc, properties=None):
        client.subscribe("rc/course", 0)
        client.subscribe(f"rc/command/{self.team_id}", 0)
        self._connected.set()
        log.info("judge link up (%s) %s:%s", self.name, self.host, self.port)
        if self._link_cb:
            self._link_cb(True)

    def _on_disconnect(self, client, userdata, flags, rc, properties=None):
        self._connected.clear()
        log.info("judge link down")
        if self._link_cb:
            self._link_cb(False)

    def _on_message(self, client, userdata, msg):
        try:
            body = json.loads(msg.payload)
        except Exception:
            log.warning("judge: malformed payload on %s ignored", msg.topic)
            return
        if msg.topic == "rc/course":
            if self._course_cb:
                self._course_cb(body)
        else:
            self.stats["commands"] += 1
            self._log({"dir": "in", "topic": "command", "body": body})
            if self._cmd_cb:
                self._cmd_cb(body)

    def _log(self, rec: dict) -> None:
        if self.log_cb:
            try:
                self.log_cb(rec)
            except Exception:
                pass


class ProtobufJudgeClient(FakeJudgeClient):
    """The real wire (ROADMAP WP4; handbook 3.4): RoboNation's RobotX 2026 protobuf messages on robocommand/robotx/... topics
    (ocs/rc_wire translates our dicts). The same interface, rate cap and per-vehicle report counters as FakeJudgeClient.

    Two references are translated here so the rest of the OCS keeps its own numbering:
      RunStart.declaration_seq is the RxRequest.seq that carried the RunDeclaration -> mapped back to our declaration_seq;
      ReadinessConfirm.report_seq is the RxReport.seq that carried the ReadinessReport -> mapped back to its command_seq.
    Reports go out with QoS 1 (heartbeats QoS 0); nothing is retained; one connection, re-subscribed on every reconnect."""
    name = "protobuf"

    def __init__(self, host: str, port: int, team_id: str, rate_cap: float = 5.0, client_id: Optional[str] = None,
                 network: str = "course", seq_file: Optional[str] = None):
        # RoboNation A1 (2026-11-04): RunStart is published once, not retained -> a persistent session (clean_session False, a
        # stable client id) so a command sent during a short disconnect is queued by the broker and delivered (QoS 1 subscription)
        super().__init__(host, port, team_id, rate_cap=rate_cap, client_id=client_id or f"ocs-{team_id}-judge", network=network,
                         clean_session=False, seq_file=seq_file)
        from . import rc_wire                      # protobuf runtime only where the real wire is used
        self.w = rc_wire
        self.decl_of_request: Dict[int, int] = {}
        self.command_of_readiness: Dict[tuple, int] = {}
        self.last_course: Optional[dict] = None
        self.own_requests: set = set()               # v0.1.65 (D18): the RxRequest seqs we published - any other on our topic is foreign
        self.foreign_cb: Optional[Callable[[dict], None]] = None

    def publish_request(self, body: dict) -> Optional[int]:
        if not self.cap.allow():
            self.stats["dropped"] += 1
            log.warning("rate cap: request dropped")
            return None
        with self._lock:
            self._req_seq += 1
            seq = self._req_seq
            self._save_seq()
        if body.get("declaration_seq") is not None:
            self.decl_of_request[seq] = int(body["declaration_seq"])
        self.own_requests.add(seq)
        self.c.publish(self.w.t_request(self.team_id), self.w.encode_request(self.team_id, seq, body), qos=1)
        self.stats["requests"] += 1
        self._log({"dir": "out", "topic": "request", "body": dict(body, request_seq=seq)})
        return seq

    def publish_report(self, vehicle_id: str, body: dict) -> Optional[int]:
        if not self.cap.allow():
            self.stats["dropped"] += 1
            if body.get("kind") != "heartbeat":
                log.warning("rate cap: report %s dropped", body.get("kind"))
            return None
        with self._lock:
            seq = self._rep_seq.get(vehicle_id, 0) + 1
            self._rep_seq[vehicle_id] = seq
            self._save_seq()
        try:
            payload = self.w.encode_report(self.team_id, vehicle_id, seq, body)
        except ValueError as e:
            log.warning("report not sent: %s", e)
            return None
        hb = body.get("kind") == "heartbeat"
        if body.get("kind") == "readiness":
            self.command_of_readiness[(vehicle_id, seq)] = int(body.get("command_seq") or 0)
        self.c.publish(self.w.t_report(self.team_id, vehicle_id), payload, qos=0 if hb else 1)
        if hb:
            self.stats["heartbeats"] += 1
        else:
            self.stats["reports"] += 1
            self.last_report = {"kind": body.get("kind"), "seq": seq, "vehicle_id": vehicle_id, "t": time.time()}
            self._log({"dir": "out", "topic": "report", "body": dict(body, report_seq=seq, vehicle_id=vehicle_id)})
        return seq

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        client.subscribe(self.w.t_course(), 1)
        client.subscribe(self.w.t_command(self.team_id), 1)
        client.subscribe(self.w.t_request(self.team_id), 1)    # v0.1.65 (D18): a declaration we did not send = FOREIGN DECLARATION
        self._connected.set()
        log.info("judge link up (protobuf) %s:%s team %s", self.host, self.port, self.team_id)
        if self._link_cb:
            self._link_cb(True)

    def _on_message(self, client, userdata, msg):
        try:
            if msg.topic == self.w.t_course():
                course = self.w.decode_course(msg.payload)
                try:
                    course["_pb"] = self.w.pb_dict("course", msg.payload)   # v0.1.64 (D12): forwarded to the UAV's Orin as received
                except Exception:
                    pass
                b = course.get("boundary") or []
                if len(b) > 1 and b[0] == b[-1]:
                    course["boundary"] = b[:-1]        # our course dicts keep the open ring (the wire closes it)
                self.last_course = course
                if self._course_cb:
                    self._course_cb(course)
                return
            if msg.topic == self.w.t_request(self.team_id):
                req = self.w.decode_request(msg.payload)
                seq = int(req.get("request_seq") or 0)
                if seq not in self.own_requests and self.foreign_cb:
                    self.foreign_cb({"seq": seq, "vehicle_ids": req.get("vehicle_ids"), "t": time.time()})
                return
            cmd = self.w.decode_command(msg.payload)
            try:
                cmd["_pb"] = self.w.pb_dict("command", msg.payload)         # v0.1.64 (D12)
            except Exception:
                pass
        except Exception as e:
            log.warning("judge: undecodable protobuf on %s ignored (%s)", msg.topic, e)
            return
        if cmd.get("team_id") and cmd["team_id"] != self.team_id:
            return                                     # 3.4.6: never act on another team's traffic
        if cmd.get("kind") == "run_start":
            cmd["request_seq"] = cmd.get("declaration_seq")
            cmd["declaration_seq"] = self.decl_of_request.get(int(cmd.get("declaration_seq") or 0), cmd.get("declaration_seq"))
        elif cmd.get("kind") == "readiness_confirm":
            key = (cmd.get("vehicle_id"), int(cmd.get("report_seq") or 0))
            if key in self.command_of_readiness:
                cmd["ref_command_seq"] = self.command_of_readiness[key]   # the same name the JSON fake judge uses
        self.stats["commands"] += 1
        self._log({"dir": "in", "topic": "command", "body": cmd})
        if self._cmd_cb:
            self._cmd_cb(cmd)
