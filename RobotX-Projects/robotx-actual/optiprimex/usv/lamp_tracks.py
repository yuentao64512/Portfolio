"""lamp_tracks (v0.1.94, Dmitrii 2026-11-14): the lamp blink decision on the boat, by WORLD bearing.

13 Nov afternoon: the observer read 496 lit lamps (red 192, blue 155, green 149) but decided no light: its blink memory
(cv/buoys_multi/led_core.LedCache) follows boxes in the IMAGE within 60 px, and at 2.3 fps with the boat yawing the typical box
moved 68 px between frames -> a new track nearly every frame (n <= 11 < 12). And 95 % of the sightings had no range (beyond the
25 m depth / invalid), so their lamp readings never reached an entity either.

Here every buoy sighting - ranged or bearing-only - gives one sample (t_cam, on, colour) at its world azimuth (heading + camera yaw +
bearing) from the camera's position. A track = the samples of one azimuth: a sample joins the live track nearest in azimuth within
az_tol_deg (one sample per track per frame, greedy by difference); the track's azimuth / origin follow the newest sample, so the
slow parallax of a moving boat is followed. A track not fed for max_gap_s is dropped. The pattern = led_core's run-length rule
(pattern_runs, ported verbatim below and pinned against it by tests/test_v0194.py) over window_s with min_samples; the colour = the on
samples' colour when >= colour_share agree and >= min_on of them.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Tuple

DEFAULTS = {"az_tol_deg": 4.0, "r_ratio": 1.6, "max_gap_s": 4.0, "window_s": 8.0, "min_samples": 8, "min_on": 3,
            # led_core.DEFAULTS' run-length rule (v0.1.92), unchanged
            "steady_frac": 0.8, "grid_s": 0.1, "median_s": 0.5, "run_min_s": 0.5, "run_max_s": 1.6, "flash_runs": 3,
            "flash_frac": (0.3, 0.7), "steady_gap_s": 0.7, "steady_span_s": 3.0, "colour_share": 0.8}


def wrap180(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


@dataclass
class LampTrack:
    tid: int
    az_deg: float
    x0: float
    y0: float
    last_t: float
    samples: Deque[Tuple[float, bool, Optional[str]]] = field(default_factory=deque)
    r_hint: Optional[float] = None       # the box-size range (m): two lamps on one bearing at 10 m and 28 m are two tracks
    ent: Optional[int] = None            # the map entity this lamp belongs to (a ranged sighting of the same frame, or by azimuth)
    result: dict = field(default_factory=dict)


def pattern_runs(samples, p: dict) -> dict:
    """led_core.LedCache.pattern_runs (v0.1.92) on a list of (t, on, colour) - the same rule, the same keys (no `track`)."""
    s = list(samples)
    n = len(s)
    ons = [c for _, on, c in s if on]
    cols = [c for c in ons if c]
    top = max(set(cols), key=cols.count) if cols else None
    share = (cols.count(top) / len(cols)) if cols else 0.0
    colour = top if share >= float(p["colour_share"]) else None
    out = {"pattern": "unknown", "colour": colour, "colour_share": round(share, 2), "n": n}
    if n < int(p["min_samples"]):
        out.update(on_frac=round(len(ons) / n, 2) if n else 0.0, changes=0)
        return out
    g, t0, t1 = float(p["grid_s"]), s[0][0], s[-1][0]
    grid, k = [], 0
    t = t0
    while t <= t1 + 1e-9:
        while k + 1 < n and s[k + 1][0] <= t + 1e-9:
            k += 1
        grid.append(1 if s[k][1] else 0)
        t += g
    h = max(1, int(round(float(p["median_s"]) / g)) // 2)
    med = [1 if 2 * sum(grid[max(0, i - h):i + h + 1]) > len(grid[max(0, i - h):i + h + 1]) else 0 for i in range(len(grid))]
    runs, cur, ln = [], med[0], 0
    for v in med:
        if v == cur:
            ln += 1
        else:
            runs.append((cur, ln * g)); cur, ln = v, 1
    runs.append((cur, ln * g))
    frac = sum(med) / len(med)
    good = [(v, d) for v, d in runs if float(p["run_min_s"]) <= d <= float(p["run_max_s"])]
    span = t1 - t0
    lo, hi = p["flash_frac"]
    if not ons:
        pat = "off"
    elif frac >= float(p["steady_frac"]) and not any(v == 0 and d >= float(p["steady_gap_s"]) for v, d in runs) and span >= float(p["steady_span_s"]):
        pat = "steady"
    elif len(good) >= int(p["flash_runs"]) and {v for v, _ in good} == {0, 1} and lo <= frac <= hi:
        pat = "flashing"
    else:
        pat = "unknown"
    out.update(pattern=pat, on_frac=round(frac, 2), changes=len(runs) - 1)
    return out


class LampTracker:
    def __init__(self, p: Optional[dict] = None):
        self.p = {**DEFAULTS, **{k: v for k, v in (p or {}).items() if v is not None}}
        if isinstance(self.p.get("flash_frac"), list):
            self.p["flash_frac"] = tuple(self.p["flash_frac"])
        self.tracks: Dict[int, LampTrack] = {}
        self._next = 1
        self.stats = {"samples": 0, "tracks": 0, "decided": 0}

    def _r_ok(self, a: Optional[float], b: Optional[float]) -> bool:
        if not a or not b:
            return True
        q = float(self.p["r_ratio"])
        return 1.0 / q <= a / b <= q

    def frame(self, t: float, obs: List[tuple]) -> List[LampTrack]:
        """One camera frame: obs = [(x0, y0, az_deg, on, colour[, r_hint])] -> the track of each, in order. v0.1.94 (truth, 13 Nov pm):
        the exit's steady blue at ~28 m sat on the bearing of the flashing entry at ~10 m and turned it "steady" - with r_hint (the box
        height's range) a sample joins only a track of a range within x/÷ r_ratio."""
        obs = [tuple(o) + (None,) * (6 - len(o)) for o in obs]
        for k in [k for k, tr in self.tracks.items() if t - tr.last_t > float(self.p["max_gap_s"])]:
            del self.tracks[k]
        tol = float(self.p["az_tol_deg"])
        pairs = sorted(((abs(wrap180(o[2] - tr.az_deg)), i, tr.tid) for i, o in enumerate(obs) for tr in self.tracks.values()
                        if abs(wrap180(o[2] - tr.az_deg)) <= tol and self._r_ok(o[5], tr.r_hint)), key=lambda q: q[0])
        got: Dict[int, LampTrack] = {}
        used = set()
        for _, i, tid in pairs:
            if i in got or tid in used:
                continue
            got[i] = self.tracks[tid]
            used.add(tid)
        out = []
        for i, (x0, y0, az, on, colour, rh) in enumerate(obs):
            tr = got.get(i)
            if tr is None:
                tr = LampTrack(self._next, az, x0, y0, t, r_hint=rh)
                self.tracks[tr.tid] = tr
                self._next += 1
                self.stats["tracks"] += 1
            tr.az_deg, tr.x0, tr.y0, tr.last_t = az, x0, y0, t
            if rh:
                tr.r_hint = rh if not tr.r_hint else 0.7 * tr.r_hint + 0.3 * rh
            tr.samples.append((t, bool(on), colour if on else None))
            while tr.samples and t - tr.samples[0][0] > float(self.p["window_s"]):
                tr.samples.popleft()
            was = tr.result.get("pattern")
            tr.result = pattern_runs(tr.samples, self.p)
            n_on = sum(1 for _, o, _c in tr.samples if o)
            if n_on < int(self.p["min_on"]):
                tr.result["colour"] = None
            if tr.result["pattern"] in ("flashing", "steady") and was not in ("flashing", "steady"):
                self.stats["decided"] += 1
            self.stats["samples"] += 1
            out.append(tr)
        return out

    def near(self, az: float, tol: Optional[float] = None) -> Optional[LampTrack]:
        tol = float(self.p["az_tol_deg"]) if tol is None else tol
        best = min(self.tracks.values(), key=lambda tr: abs(wrap180(az - tr.az_deg)), default=None)
        return best if best is not None and abs(wrap180(az - best.az_deg)) <= tol else None
