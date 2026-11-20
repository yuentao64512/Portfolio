"""number_core (OURS, v0.1.94): the buoy marker NUMBER per box - our fine-tuned 12-class classifier (digits 0-9, C, D) on near boxes,
one-to-one per frame (Dmitrii 2026-11-14: "bringing back number classifier ... merge radius by detected number").

  near(box, range_m)  - only near boxes are read: range_m <= max_m, or (bearing-only boxes have no range) a box at least min_h_px tall
                        (in the full frame; calibrated on 12 / 13 Nov, logs/cv_review/number_2026-11-14/NOTES.md); never a box cut by
                        the frame's left / right edge (within edge_px): 13 Nov's only 2 wrong reads of 60 were such half-markers ("0").
  crop(frame, box)    - the training recipe (logs/cv_review/overnight_2026-11-13/cls/crops.py): the box padded by pad x its longer side.
  NumberReader.read(frame, boxes, ranges) -> one dict per box: {"digit": int|None, "p": float, "cls": str} or None (not near / off).
                        The crops go through the classifier in ONE batch; then unique_assignment (no two buoys of a frame share a digit,
                        a digit below min_p -> None, "?"). "cls" = the classifier's top class (a digit, "C", "D") whatever the assignment.
  unique_assignment   - the student's bitmask DP (agx_track_10_buoys.py `unique_digit_assignment`, sha256 3f4efe0c003b7055), ported
                        verbatim; the only change: it takes the per-box score vectors (a list of 10 floats) instead of TrackState objects,
                        and returns {index: digit}. Pinned by tests/test_v0194_number.py.
No ZED, no map: pure numpy + the ultralytics classifier (loaded lazily, so --number off costs nothing)."""
from __future__ import annotations

import os
import time
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_WEIGHTS = os.path.join(HERE, "models", "number_cls.pt")
DEFAULTS = {"max_m": 8.0, "min_h_px": 110.0, "pad": 0.15, "min_p": 0.5, "imgsz": 320, "edge_px": 4.0, "width": 1920}


def near(box: Sequence[float], range_m: Optional[float], p: Optional[dict] = None, width: Optional[int] = None) -> bool:
    c = {**DEFAULTS, **(p or {})}
    w, e = float(width or c["width"]), float(c["edge_px"])
    if float(box[0]) <= e or float(box[2]) >= w - e:
        return False
    if range_m is not None and range_m > 0:
        return float(range_m) <= float(c["max_m"])
    return float(box[3]) - float(box[1]) >= float(c["min_h_px"])


def crop(frame, box: Sequence[float], pad: float = DEFAULTS["pad"]):
    x1, y1, x2, y2 = (float(v) for v in box)
    d = pad * max(x2 - x1, y2 - y1)
    h, w = frame.shape[:2]
    c = frame[max(0, int(y1 - d)):min(h, int(y2 + d)), max(0, int(x1 - d)):min(w, int(x2 + d))]
    return c if c.size else None


def unique_assignment(scores: Sequence[Sequence[float]], minimum_score: float) -> Dict[int, int]:
    """Maximum-score one-to-one assignment using a small bitmask DP (the student's, verbatim but for the input type)."""
    if not scores:
        return {}
    count = min(10, len(scores))
    chosen = list(range(count))
    # Each DP entry maps used-digit mask to (score, assigned digit list).
    dp: Dict[int, Tuple[float, List[Optional[int]]]] = {0: (0.0, [])}
    for state in chosen:
        next_dp: Dict[int, Tuple[float, List[Optional[int]]]] = {}
        for mask, (total, assignment) in dp.items():
            # Unknown is always allowed and does not consume a digit.
            current = next_dp.get(mask)
            candidate = (total, assignment + [None])
            if current is None or candidate[0] > current[0]:
                next_dp[mask] = candidate
            for digit in range(10):
                if mask & (1 << digit):
                    continue
                score = float(scores[state][digit])
                if score < minimum_score:
                    continue
                new_mask = mask | (1 << digit)
                candidate = (total + score, assignment + [digit])
                current = next_dp.get(new_mask)
                if current is None or candidate[0] > current[0]:
                    next_dp[new_mask] = candidate
        dp = next_dp
    _, best_assignment = max(dp.values(), key=lambda item: item[0])
    return {
        state: digit
        for state, digit in zip(chosen, best_assignment)
        if digit is not None
    }


class NumberReader:
    def __init__(self, weights: str = DEFAULT_WEIGHTS, p: Optional[dict] = None, device: Optional[str] = None):
        self.p = {**DEFAULTS, **(p or {})}
        self.weights, self.device = weights, device
        self.model, self.names = None, None
        self.last_ms = 0.0

    def _load(self):
        if self.model is None:
            from ultralytics import YOLO
            self.model = YOLO(self.weights, task="classify")
            self.names = None

    def probs(self, crops: List) -> List[Tuple[List[float], str]]:
        """One batch -> per crop (the 10 digit probabilities, the top class name)."""
        self._load()
        kw = {"imgsz": int(self.p["imgsz"]), "verbose": False}
        if self.device:
            kw["device"] = self.device
        res = self.model.predict(crops, **kw)
        out = []
        for r in res:
            names = r.names
            pr = r.probs.data.cpu().numpy().astype(float)
            digit = [0.0] * 10
            for k, n in names.items():
                if str(n).isdigit() and 0 <= int(n) <= 9:
                    digit[int(n)] = float(pr[k])
            out.append((digit, str(names[int(pr.argmax())])))
        return out

    def read(self, frame, boxes: Sequence[Sequence[float]], ranges: Sequence[Optional[float]]) -> List[Optional[dict]]:
        t0 = time.monotonic()
        out: List[Optional[dict]] = [None] * len(boxes)
        idx, crops = [], []
        for i, (bb, r) in enumerate(zip(boxes, ranges)):
            if near(bb, r, self.p, frame.shape[1]):
                c = crop(frame, bb, float(self.p["pad"]))
                if c is not None:
                    idx.append(i)
                    crops.append(np.ascontiguousarray(c))
        if crops:
            pr = self.probs(crops)
            asg = unique_assignment([d for d, _ in pr], float(self.p["min_p"]))
            for j, (i, (d, top)) in enumerate(zip(idx, pr)):
                dg = asg.get(j)
                out[i] = {"digit": dg, "p": round(d[dg] if dg is not None else max(d), 3), "cls": top}
        self.last_ms = (time.monotonic() - t0) * 1000.0
        return out
