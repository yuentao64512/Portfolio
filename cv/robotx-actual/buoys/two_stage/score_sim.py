#!/usr/bin/env python3
"""score_sim (OURS, v0.1.78.18): score any YOLO model (detect or segment) on a CV_TRAIN dataset's test split, with a class map.

Each truth box (the label polygon's bounds) is matched to the best detection by IoU >= --iou-match; the detection's class goes
through --map (model name -> our name; unmapped names are kept). Per our class: truth count, found (any class), found with the right
class; plus the detections that matched nothing (false positives) by mapped class. Pure counting, no training; read-only on the
weights and the data. CV_TRAIN's eval/confuse.py is the segment-only, 768-px original this follows (same IoU rule, same box rule).

  python score_sim.py WEIGHTS DATASET_DIR --imgsz 1280 --conf 0.25 \
      --map buoy_led_red=buoy buoy_led_green=buoy buoy_no_led=buoy dock=dock_face [--device 0] [--limit N] [--out FILE.json]
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path


def iou(a, b) -> float:
    x1, y1, x2, y2 = max(a[0], b[0]), max(a[1], b[1]), min(a[2], b[2]), min(a[3], b[3])
    i = max(0.0, x2 - x1) * max(0.0, y2 - y1)
    return i / ((a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - i + 1e-9)


def parse_map(items) -> dict:
    out = {}
    for it in items or []:
        k, _, v = it.partition("=")
        if not k or not v:
            raise ValueError(f"--map {it!r}: expected model_name=our_name")
        out[k] = v
    return out


def truth_boxes(label_text: str, W: int, H: int, names: dict) -> list:
    """YOLO label lines (segment polygons or detect boxes) -> [(our_name, [x1, y1, x2, y2])] in pixels."""
    out = []
    for line in label_text.splitlines():
        v = line.split()
        if not v:
            continue
        c, vals = int(v[0]), [float(x) for x in v[1:]]
        if len(vals) == 4:                                            # detect: cx cy w h
            cx, cy, w, h = vals
            box = [(cx - w / 2) * W, (cy - h / 2) * H, (cx + w / 2) * W, (cy + h / 2) * H]
        else:                                                         # segment: x y x y ...
            xs, ys = vals[0::2], vals[1::2]
            box = [min(xs) * W, min(ys) * H, max(xs) * W, max(ys) * H]
        out.append((names[c], box))
    return out


def score_frame(truths: list, dets: list, cmap: dict, iou_match: float = 0.5):
    """truths [(name, box)], dets [(model_name, box)] -> (per-truth [(truth, pred_or_'missed')], unmatched mapped names)."""
    used, pairs = set(), []
    for tname, tb in truths:
        best = max(((iou(tb, db), j) for j, (_, db) in enumerate(dets)), default=(0.0, -1))
        if best[0] >= iou_match:
            used.add(best[1])
            pairs.append((tname, cmap.get(dets[best[1]][0], dets[best[1]][0])))
        else:
            pairs.append((tname, "missed"))
    extra = [cmap.get(n, n) for j, (n, _) in enumerate(dets) if j not in used]
    return pairs, extra


def summarise(pairs: Counter, extra: Counter) -> dict:
    per = {}
    for (t, p), n in pairs.items():
        d = per.setdefault(t, {"truth": 0, "found": 0, "right_class": 0})
        d["truth"] += n
        if p != "missed":
            d["found"] += n
        if p == t:
            d["right_class"] += n
    for d in per.values():
        d["recall"] = round(d["found"] / d["truth"], 3) if d["truth"] else None
        d["class_recall"] = round(d["right_class"] / d["truth"], 3) if d["truth"] else None
    return {"per_class": per, "unmatched_detections": dict(extra)}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("weights"); ap.add_argument("dataset")
    ap.add_argument("--imgsz", type=int, default=768); ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou-match", type=float, default=0.5); ap.add_argument("--map", nargs="*", default=[])
    ap.add_argument("--device", default=None); ap.add_argument("--limit", type=int, default=0); ap.add_argument("--out", default=None)
    a = ap.parse_args(argv)
    import yaml
    from ultralytics import YOLO
    cmap = parse_map(a.map)
    ds = Path(a.dataset)
    names = {int(k): v for k, v in yaml.safe_load((ds / "data.yaml").read_text())["names"].items()}
    m = YOLO(a.weights)
    pairs, extra, n = Counter(), Counter(), 0
    for p in sorted((ds / "images" / "test").glob("*")):
        lab = ds / "labels" / "test" / f"{p.stem}.txt"
        if not lab.exists():
            continue
        kw = dict(imgsz=a.imgsz, conf=a.conf, verbose=False)
        if a.device is not None:
            kw["device"] = a.device
        r = m.predict(str(p), **kw)[0]
        H, W = r.orig_shape
        dets = [(m.names[int(c)], b) for c, b in zip(r.boxes.cls.cpu().numpy().astype(int), r.boxes.xyxy.cpu().numpy().tolist())]
        fp, fx = score_frame(truth_boxes(lab.read_text(), W, H, names), dets, cmap, a.iou_match)
        pairs.update(fp); extra.update(fx); n += 1
        if a.limit and n >= a.limit:
            break
    h = hashlib.sha256(Path(a.weights).read_bytes()).hexdigest()
    res = {"weights": a.weights, "sha256": h, "dataset": ds.name, "frames": n, "imgsz": a.imgsz, "conf": a.conf,
           "map": cmap, "model_names": dict(m.names), **summarise(pairs, extra),
           "truth->pred": {f"{t}->{p}": k for (t, p), k in sorted(pairs.items())}}
    txt = json.dumps(res, indent=1)
    if a.out:
        Path(a.out).write_text(txt)
    print(txt)
    return 0


if __name__ == "__main__":
    sys.exit(main())
