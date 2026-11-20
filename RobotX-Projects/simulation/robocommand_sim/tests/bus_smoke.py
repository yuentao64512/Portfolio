#!/usr/bin/env python3
"""Local-process Gazebo Transport publish/subscribe smoke test."""

from __future__ import annotations

import threading
import time

from robocommand.bus import JsonBus


def main() -> int:
    tx, rx = JsonBus(), JsonBus()
    event = threading.Event()
    value = {}

    def callback(msg: dict) -> None:
        value.update(msg)
        event.set()

    assert rx.subscribe("/robocommand/test/smoke", callback)
    time.sleep(0.3)
    for _ in range(10):
        tx.publish("/robocommand/test/smoke", {"ok": True, "value": 42})
        if event.wait(0.2):
            break
    assert value == {"ok": True, "value": 42}, value
    print("BUS_SMOKE_PASS")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

