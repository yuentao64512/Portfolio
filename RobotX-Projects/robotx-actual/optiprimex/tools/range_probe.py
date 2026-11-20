"""range_probe (v0.1.79, Dmitrii 2026-11-11: "note the distance at which a 40 cm object disappears"): one buoy of a world file put at
growing ranges dead ahead, through the simulated camera path (sim/render at the world's camera.height_m -> cv/sim_observer, the students'
measuring code + our rules -> cv_publish's items): its mask size, pixel height, and whether it comes out RANGED (a position), as a
bearing-only ray, or not at all.

  python -m optiprimex.tools.range_probe [--world profiles/bench/world_scout.yaml] [--size 1104x621] [--ranges 5,10,15,20,22,23,24,25,30,40,50]

Result 2026-11-11 (world_scout: ZED 2i hd2k 50.9 deg, 1.0 m above the water, a 0.40 m object): ranged to 23.0 m (the observer's
--max-depth 25 on the median depth), bearing-only beyond (to 50 m: still 9 px high at half hd2k); the real limit on the water is the
model's recall on a ~20 px object - m0t8 measures it.
"""
from __future__ import annotations

import argparse
import copy

from ..companion.state import LocalPose
from ..sim import world as W
from ..sim.fast_sil import CameraPath


def probe(world: str, size: str, ranges):
    wd0 = W.load(world)
    w, h = (int(x) for x in size.lower().split("x"))
    obj = next(o for o in wd0.get("objects") or [] if o.get("kind") == "buoy")
    pz = LocalPose(0.0, 0.0, 0.0, 1.28, 103.85, 0.0)
    rows = []
    for r in ranges:
        wd = copy.deepcopy(wd0)
        wd["objects"], wd["bounds"] = [dict(obj, x_n=float(r), y_e=0.0)], []
        cp = CameraPath(wd, 3, (w, h), "zed")
        ids = cp.frame(pz, 1.0, 1.28, 103.85)["ids"]
        ys = (ids == 0).nonzero()[0]
        ob = cp.observe(pz, 1.0, 1.28, 103.85)
        it = ob.items[0] if ob.items else None
        rng = getattr(it, "range_m", None) if it else None
        rows.append({"range_m": float(r), "mask_px": int((ids == 0).sum()), "height_px": int(ys.max() - ys.min() + 1) if len(ys) else 0,
                     "result": "none" if it is None else ("ranged" if rng else "bearing"), "measured_m": round(rng, 2) if rng else None})
    return rows


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default="profiles/bench/world_scout.yaml")
    ap.add_argument("--size", default="1104x621", help="the observer frame WxH (the AGX observer: 2208x1242 = hd2k; the sim: half)")
    ap.add_argument("--ranges", default="5,10,15,20,22,23,24,25,30,40,50")
    a = ap.parse_args(argv)
    cam = W.load(a.world).get("camera") or {}
    print(f"{a.world}: camera {cam.get('intrinsics')} at {cam.get('height_m', 0.8)} m, frame {a.size}")
    for row in probe(a.world, a.size, [float(x) for x in a.ranges.split(",")]):
        print(f"  {row['range_m']:5.1f} m  mask {row['mask_px']:6d} px  {row['height_px']:4d} px high  -> {row['result']}"
              + (f" {row['measured_m']} m" if row["measured_m"] is not None else ""))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
