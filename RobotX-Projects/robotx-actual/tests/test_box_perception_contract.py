from types import SimpleNamespace

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin
from optiprimex.companion.state import LocalPose
from optiprimex.sim.fake_perception import FakePerception
from optiprimex.usv.perception_map import PerceptionMap


WORLD = "profiles/bench/world_boxes.yaml"


def fake():
    a = SimpleNamespace(world=WORLD, seed=3, vehicle="usv", vehicle_id="USV1", host="127.0.0.1", port=1884,
                        hub_host=None, hub_port=0, hz=5, source="sim")
    f = FakePerception(a)
    f.cam["p_drop_max"] = 0.0
    f.cam["p_miscolour_max"] = 0.0
    f.cam["noise_range_abc"] = [0.0, 0.0, 0.0]
    f.cam["noise_bearing_deg"] = 0.0
    f.pose = {"x_n": 30.0, "y_e": 0.0, "hdg_deg": 0.0, "lat": 1.3, "lon": 103.8}     # v0.1.27: 20 m off (the camera sees 25 m)
    f.pose_t = __import__("time").monotonic()
    return f


def test_box_world_emits_face_geometry_and_steady_samples():
    f = fake()
    pm = PerceptionMap(Config(), clock=lambda: 10.0)
    pose = LocalPose(30, 0, 0, 1.3, 103.8, 0)

    # Fixed frame times cover 4.2 s (v0.1.27: the Task 3 indicators are steady; the blink rule is off by default).
    for i in range(22):
        obs = f.observe(t=1000.0 + i * 0.2)["boxes"]
        assert len(obs.items) == 3
        assert all(set(it.quality["face_points"]) == {"left", "midpoint", "right"} for it in obs.items)
        pm.ingest(obs, pose, now=10.0 + i * 0.2, source="sim")

    boxes = sorted(pm.stable_boxes(), key=lambda b: b.box_id)
    assert [b.colour for b in boxes] == ["red", "green", "blue"]
    assert all(abs(b.face.width_m - 2.0) < 0.05 for b in boxes)
    assert all(b.colour_share >= 0.8 for b in boxes)

