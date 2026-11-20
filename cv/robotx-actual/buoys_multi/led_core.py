"""led_core (OURS, v0.1.91): three pure helpers for the observer's buoy boxes - no model, no map (Dmitrii 2026-11-12).

"drop everything which is above certain line (I see that a build has been detected)":
  horizon_y / above_horizon - a box whose BOTTOM (the waterline) is above the horizon is not a buoy (12 Nov: the 6 sky / building
  detections; real buoys' bottoms sit at >= 49 % of the image height). The horizon row comes from the ZED IMU (pitch, roll) when given,
  else a fixed fraction of the height (0.45: drops those 6 and nothing with depth, on 289 snapshots).

"can we detect the color (without mapping)":
  led_sample - the LED band under the buoy's hat (the top part of its box, the middle of its width): pixels bright AND strongly
  coloured, binned by hue into red / green / blue. "on" = one colour holds >= min_px pixels and >= min_share of the band; else off.

"we need some cache for the blinking, and distinguishing the blue (constant vs blinking)":
  LedCache - follows the boxes from frame to frame in the image (the nearest centre of the last frame within track_px, the box size
  alike), keeps each track's samples for window_s, and calls the pattern: steady (on >= steady_frac of the samples), flashing (on and
  off both seen, >= 2 changes), off, or unknown (fewer than min_samples). The colour = the most frequent colour among the on samples.
  Blue steady = Task 1's exit, blue flashing = its entry. The observer's frame rate must be >= 2x the blink rate (5 fps for 1 Hz).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, List, Optional, Sequence, Tuple

HUES = {"red": ((0, 10), (170, 180)), "green": ((40, 85),), "blue": ((95, 130),)}   # OpenCV hue 0..180
# v0.1.93.2 (13 Nov run): the lantern sits ABOVE our detector's box (best.pt boxes the pyramid): the band -45 %..+5 % of the box height
# over its middle 40 % read 4 lit lanterns (red, green x2, blue) on the run's 108 masks; the old 5..40 % band (the pyramid) read 1
# v0.1.93.2 (13 Nov, 47 lit lanterns labelled by eye + 553 unlit markers, logs/cv_review/run_2026-11-13/lantern_score2.py): V >= 180,
# S >= 70, >= 3 px and >= 4 % of the band reads 15 / 47 (red 5, green 8, blue 2), no wrong colour, no false light; the old 215 / 100 / 6 /
# 2 % read 5; V 150 / S 70 read 27 but lit 3 far markers' foam floats "blue" (the band above a near marker)
# band (lantern_band_sweep.py, the same 47 + 553): -45 %..+20 % of the box height over its middle 30 % reads 25 / 47 (red 9, green 13, blue 3),
# 0 wrong, 0 false; ..+5 % / 40 % read 15 (the far lanterns straddle the box's top edge); deeper / wider bands lit far floats "blue"
DEFAULTS = {"horizon_frac": 0.45, "band_top": -0.45, "band_bottom": 0.20, "band_width": 0.3, "v_min": 215, "s_min": 70,
            "min_px": 3, "min_share": 0.04, "window_s": 6.0, "min_samples": 12, "steady_frac": 0.8, "track_px": 60.0, "track_max_s": 1.5,
            # v0.1.92 (Cowork consult 2026-11-13): brightness relative to the hull, a blown white core = on (dark hulls), the run-length rule
            # brightness: v_floor = v_cap = 215 = v0.1.91's absolute threshold - the relative rule (Cowork: hull + 60 in 170..230) and the
            # white core (dark_hull_v 150) were tried on 12 Nov's real crops (logs/cv_review/overnight_2026-11-13/led/real.jpg): they missed the
            # one real blue lantern (marker 1, 7 m) and read 11 shaded white markers "on"; the generated lamps gave nothing more. Kept as knobs.
            "rule": "runs", "v_rel": 60, "v_floor": 180, "v_cap": 180, "white_v": 240, "white_s": 60, "dark_hull_v": 0,
            "grid_s": 0.1, "median_s": 0.5, "run_min_s": 0.5, "run_max_s": 1.6, "flash_runs": 3, "flash_frac": (0.3, 0.7),
            "steady_gap_s": 0.7, "steady_span_s": 3.0, "colour_share": 0.8}


def horizon_y(height: int, fy: float = 0.0, cy: Optional[float] = None, pitch_deg: Optional[float] = None, frac: float = 0.45) -> float:
    """The horizon's row: from the camera's pitch (+ = nose up: the horizon moves DOWN in the image) when known, else frac x height."""
    if pitch_deg is None or not fy:
        return frac * height
    cy = height / 2.0 if cy is None else cy
    return cy + fy * math.tan(math.radians(pitch_deg))


def above_horizon(box: Sequence[float], hy: float, margin_px: float = 0.0) -> bool:
    """True = drop: the box's bottom edge (the waterline) is above the horizon line (minus a margin)."""
    return float(box[3]) < hy - margin_px


def led_band(box: Sequence[float], p: Optional[dict] = None) -> Tuple[int, int, int, int]:
    """The LED band under the hat: the top part of the box (band_top..band_bottom of its height), the middle band_width of its width."""
    p = {**DEFAULTS, **(p or {})}
    x1, y1, x2, y2 = (float(v) for v in box)
    w, h = x2 - x1, y2 - y1
    cx = (x1 + x2) / 2.0
    return (int(cx - w * p["band_width"] / 2), int(y1 + h * p["band_top"]), int(cx + w * p["band_width"] / 2), int(y1 + h * p["band_bottom"]))


def bgr_to_hsv(bgr):
    """OpenCV's BGR -> HSV for uint8 (H 0..180, S and V 0..255) in numpy alone (no cv2: the observer's tests run it on a stand-in)."""
    import numpy as np
    x = bgr.astype(np.float32)
    b, g, r = x[..., 0], x[..., 1], x[..., 2]
    v = np.maximum(np.maximum(r, g), b)
    mn = np.minimum(np.minimum(r, g), b)
    d = v - mn
    s = np.where(v > 0, d * 255.0 / np.maximum(v, 1e-6), 0.0)
    dz = np.where(d > 0, d, 1.0)
    h = np.where(v == r, 60.0 * (g - b) / dz, np.where(v == g, 120.0 + 60.0 * (b - r) / dz, 240.0 + 60.0 * (r - g) / dz))
    h = np.where(d > 0, np.mod(h, 360.0), 0.0) / 2.0
    return np.mod(np.round(h), 180.0), np.round(s), v


_MODEL: Dict[str, object] = {}


def lamp_model(path: Optional[str] = None):
    """v0.1.93.2 (13 Nov, logs/cv_review/led_test_2026-11-13/lamp_model.py): the learned lamp colour - a softmax over none / red / green /
    blue on the band's features (weights JSON beside this file); None when the file is missing (then the threshold rule alone)."""
    import json
    import os
    path = path or os.path.join(os.path.dirname(os.path.abspath(__file__)), "lamp_model.json")
    if path not in _MODEL:
        try:
            with open(path, encoding="utf-8") as f:
                _MODEL[path] = json.load(f)
        except (OSError, ValueError):
            _MODEL[path] = None
    return _MODEL[path]


def lamp_feats(hh, ss, vv):
    """The model's features (as lamp_model.py): 18 hue bins x 4 saturation layers of the bright (V >= 150) pixels as shares of the band,
    + the bright share, the bright-and-saturated share, the near-white share."""
    import numpy as np
    h, s, v = hh.ravel(), ss.ravel(), vv.ravel()
    n = max(1, h.size)
    br = v >= 150
    f = []
    for lo, hi in ((40, 80), (80, 120), (120, 170), (170, 256)):
        m = br & (s >= lo) & (s < hi)
        f += list(np.histogram(h[m], bins=18, range=(0, 180))[0] / n)
    f += [float(br.mean()), float((br & (s >= 80)).mean()), float((v >= 230).mean())]
    return np.array(f, float)


def lamp_probs(model, hh, ss, vv):
    import numpy as np
    z = (lamp_feats(hh, ss, vv) - np.array(model["mu"])) / np.array(model["sd"])
    e = z @ np.array(model["W"]) + np.array(model["b"])
    e = np.exp(e - e.max())
    return dict(zip(model["classes"], (e / e.sum()).tolist()))


def led_sample(bgr, box: Sequence[float], p: Optional[dict] = None) -> Tuple[Optional[str], bool, Dict[str, float]]:
    """(colour or None, on, shares per colour) for one box in one BGR frame."""
    import numpy as np
    p = {**DEFAULTS, **(p or {})}
    x1, y1, x2, y2 = led_band(box, p)
    h, w = bgr.shape[:2]
    x1, y1, x2, y2 = max(0, x1), max(0, y1), min(w, x2), min(h, y2)
    if x2 - x1 < 2 or y2 - y1 < 2:
        return None, False, {}
    hh, ss, vv = bgr_to_hsv(bgr[y1:y2, x1:x2])
    bx1, by1, bx2, by2 = (int(float(v)) for v in box)
    by0 = int(by1 + (by2 - by1) * 0.5)                                  # the hull: the box's lower half
    body = bgr[max(0, by0):min(h, by2), max(0, bx1):min(w, bx2)]
    hull_v = float(np.median(bgr_to_hsv(body)[2])) if body.size else 128.0
    if p.get("rule") == "runs":                                         # v0.1.92: relative to the hull (the pale float, EV -0.35)
        v_thr = min(float(p["v_cap"]), max(float(p["v_floor"]), hull_v + float(p["v_rel"])))
    else:
        v_thr = float(p["v_min"])
    lit = (vv >= v_thr) & (ss >= p["s_min"])
    n = float(hh.size)
    counts = {}
    for c, ranges in HUES.items():
        m = np.zeros_like(lit)
        for lo, hi in ranges:
            m |= (hh >= lo) & (hh <= hi)
        counts[c] = int((lit & m).sum())
    best = max(counts, key=counts.get)
    on = counts[best] >= p["min_px"] and counts[best] / n >= p["min_share"]
    shares = {c: round(k / n, 4) for c, k in counts.items()}
    if not on and p.get("rule") == "runs" and hull_v < float(p["dark_hull_v"]):    # a blown lantern reads white: on, its colour from the fringe
        white = int(((vv >= p["white_v"]) & (ss < p["white_s"])).sum())
        if white >= p["min_px"] and white / n >= p["min_share"]:
            fringe = {c: int((((vv >= v_thr - 40) & (ss >= p["s_min"])) & np.isin(np.round(hh), [x for lo, hi in rs for x in range(lo, hi + 1)])).sum())
                      for c, rs in HUES.items()}
            top = max(fringe, key=fringe.get)
            shares["white"] = round(white / n, 4)
            return (top if fringe[top] >= 2 else None), True, shares
    col = best if on else None
    if col == "blue" and float(p.get("blue_strong_min", 0.02)) > 0:   # v0.1.93.2 (13 Nov grass test, the flashing phases): the OFF lantern's
        strong = ((hh >= 95) & (hh <= 130) & (vv >= 150) & (ss >= float(p.get("blue_strong_s", 120)))).mean()   # pale blue-tinted housing
        shares["blue_strong"] = round(float(strong), 4)                # read "blue" - a flash looked steady (the exit). Lit: ~6 % of the band
        if strong < float(p.get("blue_strong_min", 0.02)):             # saturated >= 120, off: 0.0 %
            col, on = None, False
    if p.get("model", True):                                            # v0.1.93.2 hybrid (13 Nov grass test + the course lanterns):
        mdl = lamp_model(p.get("model_path"))                           # the model decides red / green (p >= model_thr), the rule blue -
        if mdl is not None:                                             # unless the model is sure it is green (the teal greens close up)
            pr = lamp_probs(mdl, hh, ss, vv)
            shares["model"] = {k: round(v, 3) for k, v in pr.items()}
            thr = float(p.get("model_thr", 0.9))
            lit_h = hh[(vv >= 150) & (ss >= 40)]                        # the hue guards: the model never overrules what the pixels say
            med = float(np.median(lit_h)) if lit_h.size >= 3 else None  # (a saturated pure blue lamp read 0.96 "green" - never trained on)
            red_share = float(((lit_h <= 10) | (lit_h >= 160)).mean()) if lit_h.size else 0.0
            ok = {"green": med is not None and med <= float(p.get("model_green_hue_max", 102)),
                  "red": red_share >= float(p.get("model_red_share", 0.15))}
            top = max(("red", "green"), key=lambda k: pr[k])
            if pr[top] >= thr and ok[top]:
                return top, True, shares
            if col == "blue" and pr["green"] >= float(p.get("model_veto", 0.9)) and ok["green"]:
                return "green", True, shares
    return col, on, shares


def burst(n: int, grab, frame, stamp, boxes, cache: "LedCache", p: Optional[dict] = None) -> int:
    """v0.1.92 (Dmitrii: "can we go faster than 5 fps?" - 12 Nov's observer analysed 2.2 fps in the morning, 0.8 in the afternoon; the blink
    rule needs >= 5): up to n light-only frames between two full ones - grab() (the caller grabs WITHOUT depth), frame() -> the BGR image,
    stamp() -> its time (s); the lamp read at the last full frame's boxes into the blink memory. The camera's own rate (15 fps) bounds it;
    the boxes are ~0.1-0.3 s old (the boat at <= 0.6 m/s, the buoys at >= 3 m: a few pixels). Returns the frames read."""
    done = 0
    if not boxes or n <= 0:
        return 0
    for _ in range(int(n)):
        if not grab():
            break
        img = frame()
        samples = [led_sample(img, bb, p) for bb in boxes]
        cache.update(stamp(), [(bb, s[1], s[0]) for bb, s in zip(boxes, samples)])
        done += 1
    return done


@dataclass
class Track:
    tid: int
    cx: float
    cy: float
    size: float
    last_t: float
    samples: Deque[Tuple[float, bool, Optional[str]]] = field(default_factory=deque)


class LedCache:
    """The per-buoy blink memory in the image (no map). update(t, [(box, on, colour)]) -> [(track id, pattern)] in the boxes' order."""

    def __init__(self, p: Optional[dict] = None):
        self.p = {**DEFAULTS, **(p or {})}
        self.tracks: Dict[int, Track] = {}
        self._next = 1

    def update(self, t: float, obs: List[Tuple[Sequence[float], bool, Optional[str]]]) -> List[Tuple[int, dict]]:
        for k in [k for k, tr in self.tracks.items() if t - tr.last_t > self.p["track_max_s"]]:
            del self.tracks[k]
        taken, out = set(), []
        for box, on, colour in obs:
            x1, y1, x2, y2 = (float(v) for v in box)
            cx, cy, size = (x1 + x2) / 2, (y1 + y2) / 2, max(1.0, x2 - x1)
            best, bd = None, 1e9
            for tr in self.tracks.values():
                if tr.tid in taken or not (0.5 <= size / tr.size <= 2.0):
                    continue
                d = math.hypot(cx - tr.cx, cy - tr.cy)
                if d < bd and d <= self.p["track_px"]:
                    best, bd = tr, d
            if best is None:
                best = Track(self._next, cx, cy, size, t)
                self.tracks[best.tid] = best
                self._next += 1
            taken.add(best.tid)
            best.cx, best.cy, best.size, best.last_t = cx, cy, size, t
            best.samples.append((t, bool(on), colour))
            while best.samples and t - best.samples[0][0] > self.p["window_s"]:
                best.samples.popleft()
            out.append((best.tid, self.pattern(best)))
        return out

    def find(self, box) -> Optional[Track]:
        """The live track nearest this box's centre (track_px), or None - the pattern after a burst."""
        x1, y1, x2, y2 = (float(v) for v in box)
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        best = min(self.tracks.values(), key=lambda tr: math.hypot(cx - tr.cx, cy - tr.cy), default=None)
        return best if best is not None and math.hypot(cx - best.cx, cy - best.cy) <= self.p["track_px"] else None

    def pattern(self, tr: Track) -> dict:
        if self.p.get("rule") == "runs":
            return self.pattern_runs(tr)
        return self.pattern_v091(tr)

    def pattern_runs(self, tr: Track) -> dict:
        """v0.1.92 (Cowork consult 2026-11-13: the v0.1.91 rule reads a steady light flashing 16-27 % of the time at 10 % false-off
        samples): the samples held (zero-order) onto a grid_s grid, a median_s majority filter, then the runs: flashing = >= flash_runs
        runs of run_min_s..run_max_s (on and off both among them) and the on share within flash_frac; steady = on share >= steady_frac, no off
        run >= steady_gap_s, over >= steady_span_s; off = never on; else unknown. The colour = the on samples' colour when >= colour_share
        of them agree, else None (mixed)."""
        p, s = self.p, list(tr.samples)
        n = len(s)
        ons = [c for _, on, c in s if on]
        cols = [c for c in ons if c]
        top = max(set(cols), key=cols.count) if cols else None
        share = (cols.count(top) / len(cols)) if cols else 0.0
        colour = top if share >= float(p["colour_share"]) else None
        out = {"track": tr.tid, "pattern": "unknown", "colour": colour, "colour_share": round(share, 2), "n": n}
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

    def pattern_v091(self, tr: Track) -> dict:
        s = list(tr.samples)
        n = len(s)
        ons = [c for _, on, c in s if on]
        colour = max(set(ons), key=ons.count) if ons else None
        changes = sum(1 for a, b in zip(s, s[1:]) if a[1] != b[1])
        frac = len(ons) / n if n else 0.0
        if n < self.p["min_samples"]:
            pat = "unknown"
        elif frac >= self.p["steady_frac"]:
            pat = "steady"
        elif ons and changes >= 2:
            pat = "flashing"
        elif not ons:
            pat = "off"
        else:
            pat = "unknown"
        return {"track": tr.tid, "pattern": pat, "colour": colour, "on_frac": round(frac, 2), "changes": changes, "n": n}
