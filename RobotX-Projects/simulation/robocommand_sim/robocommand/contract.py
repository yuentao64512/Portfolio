"""Shared message validation and course-coordinate helpers."""

from __future__ import annotations

import math
from typing import Any

TIERS = {"core", "advanced", "disruptive"}
REQUIRED = {"core": ("USV1",), "advanced": ("USV1", "UAV1"), "disruptive": ("USV1", "UAV1")}


def validate_declaration(msg: dict[str, Any]) -> tuple[bool, str]:
    tier = str(msg.get("tier", "")).lower()
    if tier not in TIERS:
        return False, f"unknown tier {tier!r}"
    declared = tuple(str(v).upper() for v in msg.get("vehicles", REQUIRED[tier]))
    missing = [v for v in REQUIRED[tier] if v not in declared]
    if missing:
        return False, f"tier {tier} requires {', '.join(missing)}"
    return True, "ok"


def heartbeat_ok(msg: dict[str, Any] | None, now: float, max_age: float = 3.0) -> tuple[bool, str]:
    if not msg:
        return False, "no heartbeat"
    age = now - float(msg.get("ts", 0.0))
    if age > max_age:
        return False, f"heartbeat age {age:.1f}s"
    if str(msg.get("mode", "")).upper() != "AUTO":
        return False, f"mode is {msg.get('mode', 'UNKNOWN')}, not AUTO"
    return True, "ok"


def xy_to_latlon(x: float, y: float, lat0: float, lon0: float) -> tuple[float, float]:
    lat = lat0 + y / 111_320.0
    lon = lon0 + x / (111_320.0 * math.cos(math.radians(lat0)))
    return lat, lon


def latlon_to_xy(lat: float, lon: float, lat0: float, lon0: float) -> tuple[float, float]:
    y = (lat - lat0) * 111_320.0
    x = (lon - lon0) * 111_320.0 * math.cos(math.radians(lat0))
    return x, y


def validate_report(msg: dict[str, Any]) -> tuple[bool, str]:
    scan = msg.get("scan")
    items = msg.get("items")
    if not isinstance(scan, dict) or not isinstance(items, list):
        return False, "report needs scan object and items array"
    for index, item in enumerate(items):
        required = {"id", "lat", "lon", "class", "colour", "lit", "conf", "sigma_m"}
        missing = required.difference(item)
        if missing:
            return False, f"items[{index}] missing {sorted(missing)}"
    return True, "ok"

