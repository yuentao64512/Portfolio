"""Tiny JSON bus built on Gazebo Transport StringMsg topics."""

from __future__ import annotations

import json
import threading
import time
from typing import Any, Callable

from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.transport13 import Node


class JsonBus:
    def __init__(self) -> None:
        self.node = Node()
        self._publishers: dict[str, Any] = {}
        self._callbacks: list[Callable[..., None]] = []
        self._lock = threading.Lock()

    def publish(self, topic: str, payload: dict[str, Any]) -> bool:
        with self._lock:
            publisher = self._publishers.get(topic)
            if publisher is None:
                publisher = self.node.advertise(topic, StringMsg)
                self._publishers[topic] = publisher
        msg = StringMsg()
        msg.data = json.dumps(payload, separators=(",", ":"), sort_keys=True)
        return bool(publisher.publish(msg))

    def publish_reliable(self, topic: str, payload: dict[str, Any], repeat: int = 5, interval: float = 0.1) -> bool:
        """Repeat low-rate control messages so first-advertisement discovery cannot lose them."""
        sent = False
        for _ in range(max(1, repeat)):
            sent = self.publish(topic, payload) or sent
            time.sleep(interval)
        return sent

    def subscribe(self, topic: str, callback: Callable[[dict[str, Any]], None]) -> bool:
        def wrapped(msg: StringMsg, *_: Any) -> None:
            try:
                value = json.loads(msg.data)
                if isinstance(value, dict):
                    callback(value)
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                print(f"WARN invalid JSON on {topic}: {exc}", flush=True)

        self._callbacks.append(wrapped)
        return bool(self.node.subscribe(StringMsg, topic, wrapped))

    @staticmethod
    def envelope(source: str, **payload: Any) -> dict[str, Any]:
        return {"source": source, "ts": time.time(), **payload}

