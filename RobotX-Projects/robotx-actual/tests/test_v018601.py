"""v0.1.86.1 (WP48, Dmitrii via session A: "configs for important things ... speed limiters, initial approach, obstacle stop distance,
orbiting distance"): the runtime knobs for the under-way guidance, the speed cap, the scout and the ZED guard; the boat's rates in its
status. QUICK: no broker."""
from __future__ import annotations

import yaml

from optiprimex.companion import tuning as TUN
from optiprimex.companion.config import Config
from optiprimex.usv.guard import GuardSupervisor

from .conftest import ROOT

NEW = {"v_min_mps", "arc_v_mps", "yaw_rate_max_dps", "heading_timeout_s", "bow_clear_m", "los_m", "start_fwd_m", "scan_r_m", "scan_v_mps",
       "bnd_slow_m", "bnd_stop_m", "course_tol_m", "max_speed_mps", "scout_standoff_m", "arrive_m", "scout_transit_v", "margin_m", "budget_s",
       "lane_m", "clear_m", "guard_stop_m", "guard_slow_m", "slow_v", "backoff_m", "ring_m"}


def test_the_new_knobs_their_floors_and_their_fields():
    assert NEW <= set(TUN.KNOBS)
    assert TUN.check("yaw_rate_max_dps", 12)[0] and TUN.check("bnd_stop_m", 1.0)[0] and TUN.check("guard_stop_m", 0.8)[0]   # the floors
    assert TUN.check("scout_standoff_m", 1.5)[0] and TUN.check("clear_m", 1.0)[0]
    assert TUN.field("scout_transit_v") == "transit_v" and TUN.field("guard_slow_m") == "slow_m" and TUN.field("v_min_mps") == "v_min_mps"
    ex = {"t1": {"transit_v": 0.6}, "scout": {"transit_v": 0.6}}
    TUN.apply(ex, "scout_transit_v", 0.4)
    assert ex["scout"]["transit_v"] == 0.4 and ex["t1"]["transit_v"] == 0.6                 # the prefix keeps T1's word apart
    assert TUN.cross_check({"zed_guard": {"stop_m": 1.5}}, "guard_slow_m", 1.5) and TUN.cross_check({"zed_guard": {"stop_m": 1.5}}, "guard_slow_m", 2.5) is None


def test_the_profiles_values_show_and_the_speed_cap_and_guard_act_at_once(tmp_path):
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "course" / "usv_course.yaml").read_text())
    for k in ("limits", "rates", "guards", "gains", "map", "mqtt"):
        cfg.extra.pop(k, None)
    TUN.seed(cfg)
    v = {r["key"]: r for r in TUN.view(cfg.extra)["knobs"]}
    assert v["budget_s"]["value"] == 900 and v["margin_m"]["value"] == 4 and v["v_min_mps"]["value"] == 0.4 and v["scout_transit_v"]["field"] == "transit_v"
    g = GuardSupervisor((cfg.extra or {}).get("zed_guard"))
    TUN.apply(cfg.extra, "guard_stop_m", 2.0)
    TUN.apply(cfg.extra, "max_speed_mps", 0.5)
    TUN.sync(cfg, g)
    assert g.p["stop_m"] == 2.0 and cfg.limits.max_speed_mps == 0.5


def test_the_command_handler_refuses_a_slow_range_inside_the_stop_range():
    src = (ROOT / "optiprimex" / "usv" / "command_handler.py").read_text()
    assert "tuning.cross_check(self.cfg.extra, key, value)" in src and 'tuning.sync(self.cfg, getattr(self, "guard", None))' in src
    mm = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text()
    assert "_tuning.seed(cfg)" in mm and "self.cmd.guard = self.guard" in mm


def test_the_status_carries_the_boats_rates():
    from optiprimex.usv.publishers import StatusPublisher
    sent = []
    bus = type("B", (), {"publish": lambda self, topic, kind, body, **kw: sent.append(body)})()
    cfg = Config()
    sp = StatusPublisher(bus, cfg)
    vs = type("VS", (), {"status_body": lambda self: {"state": "MANUAL"}})()
    sp.maybe(vs, 100.0)
    assert sent and sent[0]["rates"] == {"tick_hz": 10.0, "status_hz": 2.0, "local_hz": 5.0, "map_hz": 2.0}


# ---- v0.1.86.2: link {rtt_ms, loss_pct, rssi_dbm, iface} per host in /hosts.json (session A's signal bar) ---------------------------
def test_the_link_from_the_ocs_and_the_hosts_wifi(tmp_path):
    import time
    from optiprimex.tools import host_health as HH
    from optiprimex.tools.ocs_web import HostHealth
    (tmp_path / "net").mkdir()
    (tmp_path / "net" / "wireless").write_text("Inter-| sta-|   Quality        |   Discarded packets\n face | tus | link level noise |  nwid\n"
                                               " wlan0: 0000   52.  -58.  -256        0      0      0      0     12        0\n")
    assert HH.wifi(str(tmp_path)) == {"iface": "wlan0", "rssi_dbm": -58.0, "quality": 52.0}
    assert HH.wifi(str(tmp_path / "none")) is None
    hh = HostHealth()
    seq = iter([12.0, None, 14.0] * 10)
    hh.start_link_probe({"nx": ("10.0.0.3", 22)}, period_s=0.01, connect=lambda a, p: next(seq))
    time.sleep(0.2)
    hh.put("nx", {"cpu_pct": 1.0, "wifi": {"iface": "wlan0", "rssi_dbm": -58.0}})
    hh.put("agx", {"cpu_pct": 1.0})
    d = hh.hosts_body()["hosts"]
    ln = d["nx"]["link"]
    assert ln["samples"] == 10 and 20 <= ln["loss_pct"] <= 40 and 12.0 <= ln["rtt_ms"] <= 14.0 and ln["rssi_dbm"] == -58.0 and ln["iface"] == "wlan0"
    assert d["agx"]["link"] is None                                              # nothing measured: null, never invented


# ---- v0.1.86.3 (WP47): smaller recordings -------------------------------------------------------------------------------------------
def test_the_mac_frames_shrink_to_640(tmp_path):
    import io
    from PIL import Image
    from optiprimex.tools import frame_rec as FR
    b = io.BytesIO()
    Image.new("RGB", (1920, 1080), (40, 90, 160)).save(b, "JPEG", quality=92)
    big = b.getvalue()
    small = FR.shrink(big, 640)
    assert Image.open(io.BytesIO(small)).size == (640, 360) and len(small) < len(big)
    assert FR.shrink(big, 0) == big and FR.shrink(b"not a jpeg", 640) == b"not a jpeg" and FR.FRAME_WIDTH == 640


def test_the_observer_flags_and_the_h265_fallback():
    src = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert '"--svo-codec", choices=("h265", "h264"), default="h265"' in src and '"--svo-bitrate", type=int, default=8000' in src
    assert '"--pairs-width", type=int, default=960' in src and 'used = "h264 (h265 refused)"' in src
    assert "cv2.resize(z, (width, h), interpolation=cv2.INTER_NEAREST)" in src                # depth values stay real millimetres
