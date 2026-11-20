"""points_store: the saved points file (usv_ui/points.json) written from the OCS (v0.1.29, ROADMAP WP8b).

usv_ui's `save NAME` and the mission manager's SAVE_POINT / DELETE_POINT (the OCS's `save_point` / `del_point`) share one file:
every write here re-reads the file, changes one name and replaces the file atomically (tmp + os.replace), and usv_ui reloads it
when it changes - so neither side overwrites the other's points. The record is usv_ui's: lat, lon, hdg, utc (+ sats, hdop, from).
m0t2 (TEST_PATH) and m0t3 (TEST_SWAY) read it through START_TASK args.names.
"""
from __future__ import annotations

import json
import os
import re
import time
from typing import Dict, Optional, Tuple

NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,16}$")


def load(path: str) -> Dict[str, dict]:
    try:
        with open(path) as fh:
            d = json.load(fh)
        return d if isinstance(d, dict) else {}
    except (OSError, ValueError):
        return {}


def _write(path: str, data: Dict[str, dict]) -> None:
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    tmp = os.path.join(d, f".{os.path.basename(path)}.{os.getpid()}.tmp")
    with open(tmp, "w") as fh:
        json.dump(data, fh, indent=1)
    os.replace(tmp, path)


def check_name(name: str) -> Optional[str]:
    return None if NAME_RE.match(name or "") else f"point name {name!r}: 1-16 letters, digits, _ or -"


def put(path: str, name: str, lat: float, lon: float, hdg: Optional[float] = None, src: str = "ocs", **extra) -> Tuple[dict, bool]:
    """Store NAME (overwrites); returns (the record, whether it replaced one)."""
    data = load(path)
    rec = {"lat": round(float(lat), 8), "lon": round(float(lon), 8), "hdg": None if hdg is None else round(float(hdg), 1),
           "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "from": src}
    rec.update({k: v for k, v in extra.items() if v is not None})
    replaced = name in data
    data[name] = rec
    _write(path, data)
    return rec, replaced


def delete(path: str, name: str) -> bool:
    data = load(path)
    if name not in data:
        return False
    del data[name]
    _write(path, data)
    return True


class Cache:
    """The file's points, re-read only when its mtime changes (the mission manager asks every map digest)."""

    def __init__(self, path: str):
        self.path, self.mtime, self.data = path, None, {}

    def get(self) -> Dict[str, dict]:
        try:
            m = os.path.getmtime(self.path)
        except OSError:
            self.mtime, self.data = None, {}
            return self.data
        if m != self.mtime:
            self.mtime, self.data = m, load(self.path)
        return self.data
