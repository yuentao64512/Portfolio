"""sgit_map: the UAV team's own buoy map -> our UAV buoy contract (v0.1.78.19).

Their `buoy-detection/map_mqtt_publisher.py` (UAV Jetson, ~/Desktop/UAV software, seen 2026-11-10 09:06) publishes, read-only to us:
    sgit/uav/cv/buoy_map   QoS 1, not retained, the WHOLE map so far about once a second (a copy per cv_map.json rewrite):
        {"points": [{"id": 1, "color": "RED", "led_type": "FLASHING", "coords": [lon, lat]}, ...]}
        only classified buoys (RED / GREEN / BLUE, FLASHING / STATIC); an empty map is {"points": []}
    sgit/uav/cv/status     retained "online" / "offline" (their MQTT last will)
They aim it at the boat's NX with THEIR DEFAULT `--broker <NX> --port 1883` (v0.1.90.12: the boat broker's UAV map listener on
1883; our hub moved to 1882 on the Mac; 1884 = the same boat broker) (was: `--port 1884` (our boat broker; the NX's
system mosquitto on 1883 stays disabled). The mission manager subscribes both topics on the boat broker and feeds the converted
message into the same ingest_geo as docs/UAV_BUOY_CONTRACT.md: the boat keeps its own position, a UAV item links to a boat buoy within
uav.assoc_m, otherwise it stays a separate UAV-only buoy (source "uav", a lead, never planned through).

Their map has no "survey complete": scan.state stays "scanning"; WAIT_UAV_MAP leaves on the operator's `uav scan done` or the
landed-and-quiet fallback. Their ids are numbers of a map rebuilt every second (not guaranteed stable), so they are kept as `sgit_id`
for display and NOT used as the tracker id: items associate by position.
"""
from __future__ import annotations

import json
import time
from typing import Optional

MAP_TOPIC = "sgit/uav/cv/buoy_map"
STATUS_TOPIC = "sgit/uav/cv/status"
FEED = "sgit"
COLOUR = {"RED": "red", "GREEN": "green", "BLUE": "blue", "BLACK": "black", "YELLOW": "yellow"}
LIT = {"FLASHING": "flash", "STATIC": "steady", "STEADY": "steady", "OFF": "off"}


def decode(payload) -> Optional[dict]:
    """bytes / str / dict -> their message dict, or None when it is not JSON with a `points` list."""
    try:
        msg = payload if isinstance(payload, dict) else json.loads(payload.decode("utf-8") if isinstance(payload, (bytes, bytearray)) else payload)
    except (ValueError, UnicodeDecodeError, AttributeError, TypeError):
        return None
    return msg if isinstance(msg, dict) and isinstance(msg.get("points"), list) else None


def to_contract(msg: dict, seq: int, t_unix: Optional[float] = None) -> dict:
    """Their map message -> one `uav/perception/buoys` message (docs/UAV_BUOY_CONTRACT.md). A point without valid [lon, lat] becomes an
    item without lat / lon, which ingest_geo drops and counts (the rest of the message is kept)."""
    t_unix = time.time() if t_unix is None else t_unix
    items = []
    for p in msg.get("points") or []:
        if not isinstance(p, dict):
            continue
        it = {"class": "buoy", "colour": COLOUR.get(str(p.get("color") or "").upper()),
              "lit": LIT.get(str(p.get("led_type") or "").upper()), "conf": None, "sgit_id": p.get("id")}
        c = p.get("coords")
        try:
            lon, lat = float(c[0]), float(c[1])
            if -90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0:
                it["lat"], it["lon"] = lat, lon
        except (TypeError, ValueError, IndexError, KeyError):
            pass
        items.append(it)
    return {"t": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime(t_unix)), "source": "uav", "feed": FEED, "frame_id": f"{FEED}-{seq}",
            "uav": None, "scan": {"state": "scanning", "coverage_pct": None, "n_items": len(items)}, "items": items}


def status_text(payload) -> Optional[str]:
    """Their retained status -> "online" / "offline" (anything else: None)."""
    try:
        s = (payload.decode("utf-8") if isinstance(payload, (bytes, bytearray)) else str(payload)).strip().strip('"').lower()
    except UnicodeDecodeError:
        return None
    return s if s in ("online", "offline") else None
