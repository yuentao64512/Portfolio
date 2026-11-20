"""MqttSqliteLogger: everything on a broker -> one SQLite file per run (ADR-11), plus replay."""
from __future__ import annotations

import json
import logging
import os
import sqlite3
import threading
import time
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import paho.mqtt.client as mqtt

from .mqtt import start_presence

log = logging.getLogger("optiprimex.logger")

SCHEMA = """
CREATE TABLE IF NOT EXISTS messages (
  id INTEGER PRIMARY KEY,
  t_recv REAL NOT NULL,          -- unix seconds at the logger
  topic TEXT NOT NULL,
  qos INTEGER, retain INTEGER,
  payload BLOB NOT NULL,
  kind TEXT, msg_id TEXT, seq INTEGER, vehicle_id TEXT, run_id INTEGER, sent_at TEXT, source TEXT
);
CREATE INDEX IF NOT EXISTS ix_topic ON messages(topic);
CREATE INDEX IF NOT EXISTS ix_msg ON messages(msg_id);
CREATE TABLE IF NOT EXISTS meta (k TEXT PRIMARY KEY, v TEXT);
"""


class MqttSqliteLogger:
    def __init__(self, host: str, port: int, run_dir: str | Path, client_id: str = "logger",
                 user: Optional[str] = None, password: Optional[str] = None, topics: Iterable[str] = ("#", "$SYS/broker/connection/#"),
                 flush_s: float = 0.1, name: str = "boat"):
        self.host, self.port = host, port
        self.run_dir = Path(run_dir)
        self.run_dir.mkdir(parents=True, exist_ok=True)
        self.name = name
        self.db_path = self.run_dir / f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.sqlite"
        self.topics = list(topics)
        self.flush_s = flush_s
        self._buf: List[Tuple] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, clean_session=False,
                                   protocol=mqtt.MQTTv311)
        if user:
            self._client.username_pw_set(user, password)
        self._client.on_connect = self._on_connect
        self._client.on_message = self._on_message
        self._db: Optional[sqlite3.Connection] = None
        self._writer: Optional[threading.Thread] = None
        self.count = 0

    def start(self) -> None:
        self._db = sqlite3.connect(self.db_path, check_same_thread=False)
        self._db.executescript(SCHEMA)
        self._db.execute("INSERT OR REPLACE INTO meta VALUES ('started_at', ?)", (time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),))
        self._db.execute("INSERT OR REPLACE INTO meta VALUES ('broker', ?)", (f"{self.host}:{self.port}",))
        self._db.commit()
        self._writer = threading.Thread(target=self._flush_loop, name="logger-writer", daemon=True)
        self._writer.start()
        self._client.connect_async(self.host, self.port, keepalive=15)
        self._client.loop_start()
        self._presence_stop = start_presence(self._client, f"logger-{self.name}", f"{self.host}:{self.port}",
                                             extra={"file": str(self.db_path), "t0": round(time.time(), 1)})   # v0.1.83.5: the OCS's REC

    def stop(self) -> None:
        if getattr(self, "_presence_stop", None):
            self._presence_stop()
        self._stop.set()
        try:
            self._client.disconnect()
        finally:
            self._client.loop_stop()
        if self._writer:
            self._writer.join(timeout=2.0)
        self._flush()
        if self._db:
            self._db.close()

    def _on_connect(self, client, userdata, flags, reason_code, properties=None):
        for t in self.topics:
            client.subscribe(t, 1)

    def _on_message(self, client, userdata, msg):
        row = [time.time(), msg.topic, msg.qos, int(msg.retain), bytes(msg.payload), None, None, None, None, None, None, None]
        if msg.payload[:1] == b"{":
            try:
                env = json.loads(msg.payload)
                row[5:12] = [env.get("kind"), env.get("msg_id"), env.get("seq"), env.get("vehicle_id"),
                             env.get("run_id"), env.get("sent_at"), env.get("source")]
            except Exception:
                pass
        with self._lock:
            self._buf.append(tuple(row))

    def _flush_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(self.flush_s)
            self._flush()

    def _flush(self) -> None:
        with self._lock:
            rows, self._buf = self._buf, []
        if rows and self._db:
            self._db.executemany("INSERT INTO messages (t_recv, topic, qos, retain, payload, kind, msg_id, seq, vehicle_id, run_id, sent_at, source) VALUES (?,?,?,?,?,?,?,?,?,?,?,?)", rows)
            self._db.commit()
            self.count += len(rows)


def read_rows(db_path: str | Path, topic_like: Optional[str] = None) -> List[dict]:
    db = sqlite3.connect(db_path)
    q = "SELECT t_recv, topic, qos, payload, kind, msg_id, seq, vehicle_id, run_id FROM messages"
    args: Tuple = ()
    if topic_like:
        q += " WHERE topic LIKE ?"
        args = (topic_like,)
    q += " ORDER BY id"
    out = []
    for r in db.execute(q, args):
        out.append({"t_recv": r[0], "topic": r[1], "qos": r[2], "payload": r[3], "kind": r[4], "msg_id": r[5],
                    "seq": r[6], "vehicle_id": r[7], "run_id": r[8]})
    db.close()
    return out


def replay(db_path: str | Path, host: str, port: int, speed: float = 1.0, topics: Optional[List[str]] = None,
           client_id: str = "replay") -> int:
    """Re-publish the logged messages with the original relative timing (speed x)."""
    rows = read_rows(db_path)
    if topics:
        rows = [r for r in rows if any(mqtt.topic_matches_sub(t, r["topic"]) for t in topics)]
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id, protocol=mqtt.MQTTv311)
    c.connect(host, port, keepalive=15)
    c.loop_start()
    t_prev = None
    n = 0
    for r in rows:
        if t_prev is not None:
            dt = (r["t_recv"] - t_prev) / max(speed, 1e-3)
            if dt > 0:
                time.sleep(min(dt, 5.0))
        t_prev = r["t_recv"]
        c.publish(r["topic"], r["payload"], qos=r["qos"] or 0)
        n += 1
    time.sleep(0.2)
    c.loop_stop()
    c.disconnect()
    return n


def main(argv: Optional[List[str]] = None) -> int:
    import argparse
    ap = argparse.ArgumentParser(description="MQTT -> SQLite logger (one file per run)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=1882)
    ap.add_argument("--run-dir", default="./logs")
    ap.add_argument("--name", default="boat")
    ap.add_argument("--client-id", default="logger")
    ap.add_argument("--replay", help="replay this sqlite file instead of logging")
    ap.add_argument("--speed", type=float, default=1.0)
    a = ap.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(levelname)s %(message)s")
    if a.replay:
        n = replay(a.replay, a.host, a.port, a.speed)
        print(f"replayed {n} messages")
        return 0
    lg = MqttSqliteLogger(a.host, a.port, a.run_dir, client_id=a.client_id, name=a.name)
    lg.start()
    print(f"logging {a.host}:{a.port} -> {lg.db_path}", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        lg.stop()
        print(f"{lg.count} rows", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
