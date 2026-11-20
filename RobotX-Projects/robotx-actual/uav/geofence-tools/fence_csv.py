"""
Fence vertex type + CSV loading - deliberately has ZERO dependency on
pymavlink or anything MAVLink-related.

geofence_udp_publisher.py (meant to run on a GCS laptop that may have no
MAVLink stack installed at all, and isn't a sibling of drone-heartbeat-gui/)
imports only this module. mavlink_param_link.py (the Orin/MAVLink side)
re-exports these same names so nothing else needs to change.
"""

from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

# Optional line in a fence CSV setting the fence's maximum altitude, e.g.
# "max_alt,20" (metres above home). Lines without it work exactly as before.
MAX_ALT_KEY = "max_alt"


@dataclass(frozen=True)
class FenceVertex:
    lat: float
    lon: float


def same_point(a: FenceVertex, b: FenceVertex, tolerance_deg: float = 1e-7) -> bool:
    return abs(a.lat - b.lat) < tolerance_deg and abs(a.lon - b.lon) < tolerance_deg


def load_fence_csv(path: Path) -> List[FenceVertex]:
    """lat,lon per line. Blank lines and lines starting with # are ignored."""
    vertices = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        lat_str, lon_str = (p.strip() for p in line.split(","))
        if lat_str.lower() == MAX_ALT_KEY:
            continue  # read by load_fence_max_alt()
        vertices.append(FenceVertex(lat=float(lat_str), lon=float(lon_str)))
    return vertices


def load_fence_max_alt(path: Path) -> Optional[float]:
    """The "max_alt,<metres>" value from a fence CSV, or None if it has none."""
    for line in path.read_text().splitlines():
        parts = [p.strip() for p in line.strip().split(",")]
        if len(parts) == 2 and parts[0].lower() == MAX_ALT_KEY:
            value = float(parts[1])
            if value <= 0:
                raise ValueError(f"{MAX_ALT_KEY} must be positive, got {value:g}")
            return value
    return None
