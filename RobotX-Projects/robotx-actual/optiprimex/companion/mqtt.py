"""Team bus: envelopes, acks, dedup, sequence numbers, Last Will (ADR-10) over paho-mqtt 2.x.

Every message on the team namespace is an Envelope (schemas/envelope.schema.json). Local topics
(local/<v>/...) and telemetry carry the same envelope so the logger and the GUI need one parser.
"""
from __future__ import annotations

import json
import socket
import os
import logging
import threading
import time
import uuid
from collections import OrderedDict
from typing import Any, Callable, Dict, List, Optional, Tuple

import paho.mqtt.client as mqtt

from .config import Config
from .state import Ack, Command, utc_iso
from .. import schemas

log = logging.getLogger("optiprimex.mqtt")

Callback = Callable[[str, dict], None]


class Seq:
    """Per-publisher monotonically increasing sequence."""

    def __init__(self, start: int = 0):
        self._n = start
        self._lock = threading.Lock()

    def next(self) -> int:
        with self._lock:
            self._n += 1
            return self._n


class Dedup:
    """Remembers msg_ids for ttl seconds; seen() returns the stored value if already seen."""

    def __init__(self, ttl_s: float = 600.0, maxlen: int = 5000):
        self.ttl_s = ttl_s
        self.maxlen = maxlen
        self._d: "OrderedDict[str, Tuple[float, Any]]" = OrderedDict()
        self._lock = threading.Lock()

    def seen(self, msg_id: str) -> Tuple[bool, Any]:
        now = time.monotonic()
        with self._lock:
            self._evict(now)
            if msg_id in self._d:
                return True, self._d[msg_id][1]
            return False, None

    def remember(self, msg_id: str, value: Any = None) -> None:
        with self._lock:
            self._d[msg_id] = (time.monotonic(), value)
            self._d.move_to_end(msg_id)
            while len(self._d) > self.maxlen:
                self._d.popitem(last=False)

    def _evict(self, now: float) -> None:
        while self._d:
            k, (t, _) = next(iter(self._d.items()))
            if now - t > self.ttl_s:
                self._d.popitem(last=False)
            else:
                break


class Envelope:
    @staticmethod
    def wrap(kind: str, body: dict, vehicle_id: str, seq: int, run_id: Optional[int] = None,
             declaration_seq: Optional[int] = None, source: Optional[str] = None,
             intent_id: Optional[str] = None, origin: Optional[str] = None, msg_id: Optional[str] = None) -> dict:
        env = {
            "schema_version": 1,
            "msg_id": msg_id or str(uuid.uuid4()),
            "seq": seq,
            "sent_at": utc_iso(),
            "run_id": run_id,
            "declaration_seq": declaration_seq,
            "vehicle_id": vehicle_id,
            "kind": kind,
            "body": body,
        }
        if source:
            env["source"] = source
        if intent_id:
            env["intent_id"] = intent_id
        if origin:
            env["origin"] = origin
        return env

    @staticmethod
    def parse(payload: bytes, validate: bool = True) -> dict:
        env = json.loads(payload.decode("utf-8"))
        if validate:
            schemas.validate(env)
        return env


def start_presence(client: mqtt.Client, node: str, broker: str, period_s: float = 2.0, extra: Optional[dict] = None) -> Callable[[], None]:
    """Publish retained presence/<node> every period_s while the client is connected; returns a stop function that
    publishes up=false. A node that dies leaves its last beat behind: consumers treat a beat older than ~3 periods
    as down (the bench TUI's NETWORK box). One topic per node per broker; the mission manager keeps its status LWT too."""
    stop = threading.Event()
    body = {"node": node, "broker": broker, "host": socket.gethostname(), "pid": os.getpid(), **(extra or {})}

    def beat(up: bool, wait_s: float = 0.0) -> None:
        try:
            info = client.publish(f"presence/{node}", json.dumps({**body, "up": up, "t": utc_iso(), "t_unix": round(time.time(), 2)}), qos=0, retain=True)
            if wait_s:
                info.wait_for_publish(wait_s)
        except Exception:
            pass

    def loop() -> None:
        while not stop.wait(period_s):
            if client.is_connected():
                beat(True)

    threading.Thread(target=loop, name=f"presence-{node}", daemon=True).start()

    def end() -> None:
        stop.set()
        beat(False, wait_s=1.0)
    return end


class Bus:
    """One MQTT client with the team envelope conventions.

    publish() blocks for at most timeout_s until the client is connected (bounded wait), then hands the
    message to paho; QoS 1 messages are queued by paho while disconnected (persistent session).
    """

    def __init__(self, cfg: Config, client_id: Optional[str] = None, lwt_topic: Optional[str] = None,
                 lwt_body: Optional[dict] = None, clean_session: bool = True, host: Optional[str] = None,
                 port: Optional[int] = None, user: Optional[str] = None, password: Optional[str] = None,
                 source: Optional[str] = None):
        self.cfg = cfg
        self.host = host or cfg.mqtt.host
        self.port = port or cfg.mqtt.port
        self.client_id = client_id or cfg.mqtt.client_id
        self.source = source or cfg.node
        self.seq = Seq()
        self.dedup = Dedup()
        self.run_id: Optional[int] = None
        self.declaration_seq: Optional[int] = None
        self._subs: List[Tuple[str, int, Callback, bool]] = []
        self._connected = threading.Event()
        self._bridge_state: Dict[str, bool] = {}
        self._lock = threading.Lock()
        self.stats = {"tx": 0, "rx": 0, "bad": 0}
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=self.client_id,
                                   clean_session=clean_session, protocol=mqtt.MQTTv311)
        u = user if user is not None else cfg.mqtt.user
        p = password if password is not None else cfg.mqtt.password
        if u:
            self._client.username_pw_set(u, p)
        if lwt_topic:
            body = lwt_body or {"state": "OFFLINE", "current_task": "TASK_NONE", "kill": False, "estop": False}
            env = Envelope.wrap("status", body, cfg.vehicle_id, 0, source=self.source)
            self._client.will_set(lwt_topic, json.dumps(env), qos=0, retain=False)
        self._client.on_connect = self._on_connect
        self._client.on_disconnect = self._on_disconnect
        self._client.on_message = self._on_message
        self._client.reconnect_delay_set(min_delay=1, max_delay=5)
        self._client.max_queued_messages_set(10000)

    # ---- lifecycle ----------------------------------------------------------
    def start(self) -> None:
        self._client.connect_async(self.host, self.port, keepalive=self.cfg.mqtt.keepalive_s)
        self._client.loop_start()
        self._presence_stop = start_presence(self._client, self.source or "node", f"{self.host}:{self.port}",
                                             extra={"vehicle_id": self.cfg.vehicle_id})

    def stop(self, drain_s: float = 0.5) -> None:
        if getattr(self, "_presence_stop", None):
            self._presence_stop()
        t0 = time.monotonic()
        while time.monotonic() - t0 < drain_s and self._client.want_write():
            time.sleep(0.02)
        try:
            self._client.disconnect()
        finally:
            self._client.loop_stop()
        self._connected.clear()

    def wait_connected(self, timeout_s: Optional[float] = None) -> bool:
        return self._connected.wait(timeout_s if timeout_s is not None else self.cfg.mqtt.connect_timeout_s)

    @property
    def connected(self) -> bool:
        return self._connected.is_set()

    def bridge_state(self, name: Optional[str] = None) -> bool:
        return self._bridge_state.get(name or self.cfg.mqtt.bridge_name, False)

    # ---- subscriptions --------------------------------------------------------
    def subscribe(self, topic: str, cb: Callback, qos: int = 1, raw: bool = False) -> None:
        """cb(topic, env) for envelope topics; raw=True gives cb(topic, payload_bytes)."""
        with self._lock:
            self._subs.append((topic, qos, cb, raw))
        if self.connected:
            self._client.subscribe(topic, qos)

    def watch_bridge(self, name: Optional[str] = None) -> None:
        name = name or self.cfg.mqtt.bridge_name
        self.subscribe(f"$SYS/broker/connection/{name}/state", self._on_bridge, qos=0, raw=True)

    # ---- publishing -----------------------------------------------------------
    def publish(self, topic: str, kind: str, body: dict, qos: int = 1, timeout_s: float = 2.0,
                retain: bool = False, intent_id: Optional[str] = None, origin: Optional[str] = None,
                vehicle_id: Optional[str] = None, msg_id: Optional[str] = None) -> str:
        env = Envelope.wrap(kind, body, vehicle_id or self.cfg.vehicle_id, self.seq.next(), self.run_id,
                            self.declaration_seq, source=self.source, intent_id=intent_id, origin=origin,
                            msg_id=msg_id)
        schemas.validate(env)
        self.publish_env(topic, env, qos=qos, timeout_s=timeout_s, retain=retain)
        return env["msg_id"]

    def publish_env(self, topic: str, env: dict, qos: int = 1, timeout_s: float = 2.0, retain: bool = False) -> None:
        self._connected.wait(timeout_s)
        payload = json.dumps(env, separators=(",", ":"))
        info = self._client.publish(topic, payload, qos=qos, retain=retain)
        if info.rc not in (mqtt.MQTT_ERR_SUCCESS, mqtt.MQTT_ERR_NO_CONN):
            log.warning("publish %s rc=%s", topic, info.rc)
        self.stats["tx"] += 1

    def publish_raw(self, topic: str, payload: bytes | str, qos: int = 0, retain: bool = False) -> None:
        self._client.publish(topic, payload, qos=qos, retain=retain)
        self.stats["tx"] += 1

    def ack(self, cmd: Command, result: str, reason: str = "", state: str = "") -> str:
        topic = self.cfg.team("ack", self.cfg.vehicle)
        ack = Ack(ack_of=cmd.msg_id, result=result, reason=reason, state=state)
        return self.publish(topic, "ack", ack.body(), qos=1, intent_id=cmd.intent_id)

    # ---- paho callbacks ---------------------------------------------------------
    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        if reason_code != 0 and str(reason_code) != "Success":
            log.warning("%s connect refused: %s", self.client_id, reason_code)
            return
        with self._lock:
            subs = list(self._subs)
        for topic, qos, _, _ in subs:
            client.subscribe(topic, qos)
        self._connected.set()
        log.info("%s connected to %s:%s (%d subscriptions)", self.client_id, self.host, self.port, len(subs))

    def _on_disconnect(self, client, userdata, flags, reason_code, properties=None):
        self._connected.clear()
        log.info("%s disconnected (%s)", self.client_id, reason_code)

    def _on_message(self, client, userdata, msg):
        self.stats["rx"] += 1
        with self._lock:
            subs = list(self._subs)
        for pattern, _, cb, raw in subs:
            if not mqtt.topic_matches_sub(pattern, msg.topic):
                continue
            try:
                if raw:
                    cb(msg.topic, msg.payload)
                else:
                    cb(msg.topic, Envelope.parse(msg.payload))
            except schemas.ContractError as e:
                self.stats["bad"] += 1
                log.warning("contract violation on %s: %s", msg.topic, e)
            except Exception:  # a bad subscriber must not kill the network thread
                self.stats["bad"] += 1
                log.exception("subscriber failed on %s", msg.topic)

    def _on_bridge(self, topic: str, payload: bytes):
        name = topic.split("/")[3] if topic.count("/") >= 4 else "?"
        self._bridge_state[name] = payload.strip() == b"1"
