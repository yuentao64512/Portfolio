import math

import pytest

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin, PoseBuffer, bearing_deg, body_to_local, dist_m, wrap180
from optiprimex.companion.health import HealthMonitor
from optiprimex.companion.state import FcuState, GuidedTarget, LocalPose, ThrusterFrame
from optiprimex.usv.adapters.rover import RoverAdapter


def test_origin_round_trip_and_axes():
    o = ControlOrigin(1.3, 103.8)
    x, y = o.to_local(1.3 + 0.001, 103.8)
    assert 110.0 < x < 112.0 and abs(y) < 1e-6          # north is x
    lat, lon = o.from_local(50.0, -20.0)
    x2, y2 = o.to_local(lat, lon)
    assert abs(x2 - 50) < 1e-6 and abs(y2 + 20) < 1e-6
    assert abs(bearing_deg((0, 0), (0, 10)) - 90.0) < 1e-9   # east = 090
    assert abs(bearing_deg((0, 0), (-10, 0)) - 180.0) < 1e-9
    assert wrap180(350) == -10 and wrap180(-190) == 170
    assert dist_m((0, 0), (3, 4)) == 5


def test_pose_buffer_interpolates_heading_shortest_arc():
    pb = PoseBuffer()
    pb.push(LocalPose(0, 0, 350, 1.3, 103.8, t=0.0))
    pb.push(LocalPose(10, 0, 10, 1.3, 103.8, t=1.0))
    p = pb.at(0.5)
    assert abs(p.x_n - 5) < 1e-9 and abs(p.hdg_deg - 0.0) < 1e-9
    assert pb.at(-1).t == 0.0 and pb.at(5).t == 1.0


def test_body_to_local_starboard_is_right_of_heading():
    pose = LocalPose(0, 0, 90, 1.3, 103.8, t=0)                   # heading east
    x, y = body_to_local(10, 90, pose)                             # 10 m to starboard = south
    assert abs(x + 10) < 1e-6 and abs(y) < 1e-6
    x, y = body_to_local(10, 0, pose, cam_forward_m=1.0)           # dead ahead with a 1 m lever arm
    assert abs(y - 11) < 1e-6 and abs(x) < 1e-6


def test_health_faults():
    cfg = Config()
    hm = HealthMonitor(cfg)
    f = FcuState(seen_heartbeat=True, rc_chancount=16, rc_ok=True, ekf_pos_ok=True, armed=True)
    f.ages = {"HEARTBEAT": 0.1, "GLOBAL_POSITION_INT": 0.05, "ATTITUDE": 0.05, "RC_CHANNELS": 0.2, "EKF_STATUS_REPORT": 0.3}
    h = hm.update(f, True, float("inf"), 0.0)
    assert h.faults == [] and hm.fcu_ok() and hm.rc_ok()
    f.ages["HEARTBEAT"] = f.ages["GLOBAL_POSITION_INT"] = f.ages["ATTITUDE"] = 5.0
    assert "FCU_LOST" in hm.update(f, True, float("inf"), 1.0).faults
    f.ages["HEARTBEAT"] = f.ages["GLOBAL_POSITION_INT"] = f.ages["ATTITUDE"] = 0.1
    f.ekf_pos_ok = False
    assert hm.hard_faults() == ["EKF_BAD"] or "EKF_BAD" in hm.update(f, True, float("inf"), 2.0).faults
    f.ekf_pos_ok = True
    f.ages["RC_CHANNELS"] = 5.0
    h = hm.update(f, True, float("inf"), 3.0)
    assert "RC_LOST" in h.faults and "RC_LOST" not in hm.hard_faults()   # RC loss is the FCU's failsafe, not ours
    hm.expect_armed = True
    f.ages["RC_CHANNELS"] = 0.1
    hm.update(f, True, float("inf"), 4.0)
    f.armed = False
    assert "UNEXPECTED_DISARM" in hm.update(f, True, float("inf"), 5.0).faults
    cfg.timeouts.perception_s = 1.0
    assert "PERCEPTION_LOST" in hm.update(f, True, 2.0, 6.0).faults and "PERCEPTION_LOST" in hm.soft_faults()
    # the lab profile: a fault in guards.ignore_faults stays visible but no longer forces FAULT / SAFE_HOLD
    cfg.guards.ignore_faults = ["EKF_BAD", "RC_LOST", "PERCEPTION_LOST"]
    f.ekf_pos_ok = False
    f.ages["RC_CHANNELS"] = 5.0
    h = hm.update(f, True, 2.0, 7.0)
    assert {"EKF_BAD", "RC_LOST", "PERCEPTION_LOST"} <= set(h.faults)
    assert "EKF_BAD" not in hm.hard_faults() and hm.soft_faults() == []
    from optiprimex.companion.config import from_dict
    with pytest.raises(ValueError):
        from_dict({"guards": {"ignore_faults": ["GPS_BAD"]}})
    assert from_dict({"guards": {"ignore_faults": ["EKF_BAD"]}}).guards.ignore_faults == ["EKF_BAD"]


class FakeLink:
    def __init__(self):
        self.mode = "HOLD"
        self.frames = []
        self.targets = []
        self.modes = []
        self.speed = []

    def state(self):
        return FcuState(mode_name=self.mode)

    def set_mode(self, name, timeout_s=0.5):
        self.modes.append(name)
        self.mode = name
        return True

    def send_actuator_frame(self, f):
        self.frames.append(f.as_list())

    def send_position_target_global(self, lat, lon, alt_m=0.0):
        self.targets.append((lat, lon))

    def send_velocity_yaw(self, vx, r):
        self.targets.append(("vel", vx, r))

    def do_change_speed(self, mps, timeout_s=1.0):
        self.speed.append(mps)
        return True


def test_adapter_allocation_and_direct_entry():
    cfg = Config()
    cfg.limits.max_amp = 0.5
    link = FakeLink()
    a = RoverAdapter(link, cfg)
    f = a.allocate(1.0, 0.0, 0.0)                         # pure surge: M1 = M3, M2 = M4 = 0, scaled to 0.5
    assert f.m1 == f.m3 == 0.5 and f.m2 == f.m4 == 0
    f = a.allocate(0.0, 1.0, 0.0)                         # pure sway: M2 = 0.957 v scaled jointly with M4
    assert abs(f.m4 - 0.5) < 1e-9 and abs(f.m2 - 0.5 * 0.957) < 1e-9
    f = a.allocate(0.0, 0.0, 1.0)                         # pure yaw: M1 = -M3
    assert f.m1 == 0.5 and f.m3 == -0.5
    a.direct(ThrusterFrame(0.3, 0, 0.3, 0))
    assert link.modes == ["CUSTOM_MANUAL"] and link.frames[0] == [0, 0, 0, 0] and link.frames[1] == [0.3, 0, 0.3, 0]
    a.cancel()
    assert link.frames[-1] == [0, 0, 0, 0] and link.mode == "HOLD" and not a.direct_active


def test_adapter_guided_carrot_rate_and_hold_reassert():
    cfg = Config()
    link = FakeLink()
    a = RoverAdapter(link, cfg)
    t = GuidedTarget(1.3, 103.8, 0.8)
    a.guided_goto(t, now=0.0)
    a.guided_goto(t, now=0.1)
    a.guided_goto(t, now=0.6)
    assert link.mode == "GUIDED" and len(link.targets) == 2 and link.speed == [0.8]   # <= 2 Hz for the same target
    a.guided_goto(GuidedTarget(1.3001, 103.8, 0.8), now=0.65)
    assert len(link.targets) == 3                                                  # a new target goes out at once
    a.hold(now=1.0)
    n = len(link.modes)
    a.hold(now=2.0)
    assert len(link.modes) == n                                                    # within reassert period
    a.hold(now=1.0 + cfg.timeouts.hold_reassert_s + 0.1)
    assert len(link.modes) == n + 1
