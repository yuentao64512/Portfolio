"""buoys_multi/run.sh --self-test: the colour classifier on synthetic pixels and the frame payload (no camera, no model)."""
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import multi_core as M  # noqa: E402
import numpy as np  # noqa: E402
import guard_core as GC  # noqa: E402

red = [(30, 30, 200)] * 200 + [(40, 40, 40)] * 50
green = [(60, 180, 40)] * 200
assert M.classify_bgr(red)[0] == "red" and M.classify_bgr(green)[0] == "green" and M.classify_bgr([(20, 20, 20)] * 100)[0] == "black"
p = M.frame_payload("2026-01-01T00:00:00Z", 3, [M.buoy_record({"range_m": 6.0, "bearing_deg": 10.0}, 0.8, "green", {}, None, 1),
                                               M.buoy_record({"range_m": 4.0, "bearing_deg": -8.0}, 0.9, "red", {}, None, 0)], {})
assert p["n_buoys"] == 2 and p["buoys"][0]["colour"] == "red" and p["measurement"]["range_m"] == 4.0
# v0.1.80 (WP36): the guard - a board 1.2 m ahead (0.2-1.0 m above the water, camera 1.0 m up) -> STOP on the 3rd frame
xs, hs = np.meshgrid(np.linspace(-0.3, 0.3, 40), np.linspace(0.2, 1.0, 40))
cloud = np.stack([xs, 1.0 - hs, np.full_like(xs, 1.2)], axis=2).astype(np.float32)
g = GC.GuardFilter({"step": 1, "min_px": 100})
zones = [g.frame(cloud)["zone"] for _ in range(3)]
assert zones == ["slow", "slow", "stop"], zones
# v0.1.81.1: a 0.4 m buoy face 6 m ahead -> one `far` cluster for the boat's obstacle map
xs, hs = np.meshgrid(np.linspace(-0.2, 0.2, 30), np.linspace(0.2, 0.6, 30))
far = GC.GuardFilter({"step": 1}).frame(np.stack([xs, 1.0 - hs, np.full_like(xs, 6.0)], axis=2).astype(np.float32)).get("far") or []
assert len(far) == 1 and abs(far[0]["range_m"] - 6.0) < 0.1, far
print(json.dumps({"self_test": "PASS", "colours": ["red", "green", "black"], "n_buoys": p["n_buoys"], "status": p["status"], "guard": zones, "far": len(far)}))
