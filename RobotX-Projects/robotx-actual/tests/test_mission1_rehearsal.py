"""Mission-1 rehearsal (2026-11-12 evening, session A; Dmitrii: "test everything thoroughly that tomorrow we don't be stuck").

The real OCS (in process, the course profile) started from each launcher preset's REAL launch block, then walked through tomorrow's order:
crew (heartbeats, ready by itself) -> ENABLE AUTO -> DECLARE -> RunStart -> the mission. Every failure of 12 Nov's afternoon is a case here:
the preset + an edited mission (T1 mismatch), the auto-start, the UAV declared by itself, pilot ready by itself, the per-vehicle heartbeat
toggles, T4 / lucky reaching the wire, a bare START = the preselected mission, ENABLE AUTO before DECLARE on the boat."""
from __future__ import annotations

import time

import pytest
import yaml

from optiprimex.companion.config import Config
from optiprimex.ocs.judge_client import NullJudgeClient
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.tools import launcher as L

from .conftest import ROOT
from .test_ocs_units import FakeBus

COURSE = {"course_id": "ALPHA", "boundary": [[1.0, 103.0], [1.0, 103.001], [1.001, 103.001], [1.001, 103.0]], "uav_transit_area": [],
          "pinger_hz": 30000}
PRESETS = L.load_presets(str(ROOT / "profiles" / "presets.yaml"))


def ocs_for(preset: str, **edits) -> OcsApp:
    """The OCS as the launcher would start it: the course profile + the preset's launch block (with the operator's edits)."""
    lb = L.launch_block(dict(PRESETS[preset], preset=preset, **edits))
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "course" / "ocs_course.yaml").read_text())
    if lb:
        cfg.extra["launch"] = lb
    app = OcsApp(cfg, bus=FakeBus(), judge=NullJudgeClient())
    app.authority.publish_result = lambda r: None
    app.lifecycle.on_link(True)
    app.lifecycle.on_course(COURSE)
    app.apply_launch()
    app.on_status("t", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL", "armed": True, "mode": "HOLD"}})
    return app


def uav_status(app: OcsApp, ready: bool = False, **body):
    b = {"state": "MANUAL", "robot_state": "STATE_MANUAL", "armed": False, "mode": "LOITER", "flight_phase": "FLIGHT_PHASE_GROUNDED",
         "health": {"telemetry_age_s": 0.2, "fence_ok": True, "pilot_ready": ready},
         "fence": {"points": [[1.0, 103.0], [1.0, 103.001], [1.001, 103.001]], "points_n": 3, "ceiling_hae_m": 50.0}}
    b.update(body)
    app.on_status("t", {"vehicle_id": app.uav_vid, "body": b})


def sent(app: OcsApp, action: str):
    return [m for m in app.bus.published if isinstance(m.get("body"), dict) and m["body"].get("action") == action]


# ---- 1 · USV only -------------------------------------------------------------------------------------------------------------------------
def test_usv_only_as_launched():
    app = ocs_for("m1t1-gates")
    assert app.mission["code"] == "m1t1" and app.mission["auto"] is True                    # auto-start from the preset
    assert app.authority.mission_task == "T1"                                                # a bare ▶ START = m1t1, never T4
    assert not app.declare_uav and app.usv_flow == "auto" and app.usv_ready                   # USV ready by itself, no UAV
    t = app.effective_tiers()
    assert t["T1"] == "TIER_CORE" and t["T4"] == "TIER_NONE"
    assert app.declare({}) is None                                                           # DECLARE goes (PREPARE out)
    prep = sent(app, "PREPARE")
    assert [m["vehicle_id"] for m in prep] == ["USV1"] and prep[0]["body"]["args"]["tiers"]["T1"] == "TIER_CORE"
    assert prep[0]["body"]["args"]["tiers"]["T4"] == "TIER_NONE"


def test_usv_only_with_t4_and_lucky_ticked():
    app = ocs_for("m1t1-gates", t4=True, lucky=True)
    assert app.effective_tiers()["T4"] == "TIER_CORE"
    assert app.mission["lucky"] is True and app.authority.mission_args.get("lucky") is True   # the auto start and a bare START carry it
    app.declare({})
    assert sent(app, "PREPARE")[0]["body"]["args"]["tiers"]["T4"] == "TIER_CORE"


def test_the_afternoon_mismatch_preset_3_edited_to_m1t1():
    """17:08 / 17:10: preset m1t2-both with the mission field changed to m1t1 -> DECLARE refused (T1 ADVANCED). Now T1 follows m1t1."""
    app = ocs_for("m1t2-both", mission="m1t1")
    assert app.effective_tiers()["T1"] == "TIER_CORE" and not app.declare_uav               # m1t1: not a UAV run
    assert app.declare({}) is None and sent(app, "PREPARE")[0]["body"]["args"]["tiers"]["T1"] == "TIER_CORE"


def test_re_arming_in_the_console_keeps_the_auto_start():
    """16:55: ARM MISSION re-sent `mission m1t2` without --auto -> the auto-start lost. The OCS side: a MISSION intent with auto keeps it."""
    app = ocs_for("m1t1-gates")
    app.set_mission("T1", True, {"tier": "core"})
    assert app.mission["auto"] is True
    src = (ROOT / "optiprimex" / "tools" / "tkconsole" / "views.py").read_text()
    assert '(" --auto" if cur.get("auto") else "")' in src                                   # the console appends it


# ---- 2 · UAV only (the boat holds) / 3 · UAV + USV ---------------------------------------------------------------------------------------
@pytest.mark.parametrize("preset,auto", [("m1t2-uav", False), ("m1t2-both", True)])
def test_uav_runs_declare_the_drone_and_ready_comes_by_itself(preset, auto):
    app = ocs_for(preset)
    assert app.declare_uav is True and app.mission["code"] == "m1t2" and app.mission["auto"] is auto
    assert app.effective_tiers()["T1"] == "TIER_ADVANCED" and app.effective_tiers()["T4"] == "TIER_NONE"
    uav_status(app, ready=False)
    assert app.uav_active()
    app.uav_ready_tick()                                                                     # pilot ready by itself
    assert [m["body"]["args"] for m in sent(app, "UAV_READY")] == [{"on": True}]
    assert set(app.hb_vehicles()) == {"USV1", app.uav_vid}                                   # both on the RC page before DECLARE
    assert app.on_reporting(False, "uav") is None and app.hb_vehicles() == ["USV1"]          # the per-vehicle toggle
    app.on_reporting(True, "uav")
    uav_status(app, ready=True)
    why = app.declare({})
    assert why is None, why
    assert {m["vehicle_id"] for m in sent(app, "PREPARE")} == {"USV1", app.uav_vid}
    assert all(m["body"]["args"]["tiers"]["T1"] == "TIER_ADVANCED" for m in sent(app, "PREPARE"))


def test_uav_run_without_the_drone_says_why():
    app = ocs_for("m1t2-both")
    why = app.declare({})                                                                    # no UAV status at all
    assert why and app.uav_vid in why


# ---- the order: ENABLE AUTO before DECLARE, everywhere -------------------------------------------------------------------------------
def test_enable_auto_before_declare_is_the_rule_everywhere():
    from optiprimex.usv.vehicle_fsm import VehicleFsm
    for f in ("course/usv_course.yaml", "pool/usv_pool.yaml", "pool/usv_lab.yaml", "bench/usv_bench.yaml"):
        assert yaml.safe_load((ROOT / "profiles" / f).read_text())["guards"]["require_prepare_for_auto"] is False, f
    share = yaml.safe_load((ROOT / "usv_ui" / "config" / "usv_bench_share.yaml").read_text())     # the sim / bench boat's copy
    assert share["guards"]["require_prepare_for_auto"] is False
    fsm = VehicleFsm(Config())
    fsm.inputs.armed, fsm.inputs.ekf_pos_ok = True, True
    assert fsm.can_enable_auto() == (True, "")                                               # no declaration needed
    app = ocs_for("m1t1-gates")
    assert app.state_body()["auto_before_declare"] is True                                   # the console's ladder + playbook follow it


def test_no_pilot_ready_button_anywhere():
    for f in ("tkconsole/views.py", "tkconsole/model.py"):
        src = (ROOT / "optiprimex" / "tools" / f).read_text()
        assert 'key="usv-ready"' not in src and 'key="uav-ready"' not in src and '"usv-ready"' not in src.split("USV_ROWS")[-1][:200], f
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert page.count('br.style.display="none"') == 2


def test_the_launcher_refuses_when_the_mac_lost_ocs_host():
    """12 Nov 18:20 at the venue: the USB LAN back on DHCP (.6), the boat bridging to OCS_HOST .13 -> no status, no USV heartbeat."""
    ifc = ("lo0: flags=8049<UP>\n\tinet 127.0.0.1 netmask 0xff000000\n"
           "en20: flags=8863<UP>\n\tinet 192.168.1.6 netmask 0xffffff00 broadcast 192.168.1.255\n")
    order = "(17) USB 10/100/1G/2.5G LAN\n(Hardware Port: USB 10/100/1G/2.5G LAN, Device: en20)\n"
    run = lambda argv: (0, ifc) if argv[0] == "ifconfig" else (0, order)
    cfg = {"mode": "real", "net": "lan"}
    why = L.ocs_host_check(cfg, run=run, net_env={"OCS_HOST": "192.168.1.13"})
    assert why and "192.168.1.13" in why and "en20 has 192.168.1.6" in why
    assert 'sudo networksetup -setmanual "USB 10/100/1G/2.5G LAN" 192.168.1.13 255.255.255.0' in why
    ok = lambda argv: (0, ifc.replace("192.168.1.6", "192.168.1.13"))
    assert L.ocs_host_check(cfg, run=ok, net_env={"OCS_HOST": "192.168.1.13"}) is None
    assert L.ocs_host_check({"mode": "sim"}, run=run, net_env={"OCS_HOST": "192.168.1.13"}) is None


def test_one_port_number_per_job_and_1883_only_for_the_drone():
    """v0.1.90.12 (Dmitrii: "I don't want ANY ambiguity with 1883 ... assign unique to each one"): GUIDE's table of OUR ports has no
    number twice; the hub is 1882 everywhere it is configured; 1883 of ours is only the boat broker's drone-map listener."""
    import re
    guide = (ROOT / "GUIDE.md").read_text()
    ours = guide[guide.index("**Ports** (v0.1.90.12"):guide.index("Not ours, fixed by others")]
    nums = re.findall(r"^\| \**(\d{4,5})\**(?: udp)? \|", ours, re.M)
    assert len(nums) >= 18 and len(nums) == len(set(nums)), nums
    assert "HUB_PORT=1882" in (ROOT / "network.env").read_text() and "BOAT_WS=9002" in (ROOT / "network.env").read_text()
    for f in ("course/ocs_course.yaml", "pool/ocs_pool.yaml", "bench/ocs_bench.yaml"):
        assert yaml.safe_load((ROOT / "profiles" / f).read_text())["mqtt"]["port"] == 1882, f
    assert "listener 1882 127.0.0.1" in (ROOT / "profiles" / "bench" / "mosquitto" / "hub.conf").read_text()
    for sh in (ROOT / "scripts").glob("*.sh"):
        t = sh.read_text()
        assert "HUB_PORT:-1883" not in t and "BOAT_WS:-9001" not in t, sh.name
    assert 'BOAT_UAV_PORT="${BOAT_UAV_PORT:-1883}"' in (ROOT / "scripts" / "run_usv.sh").read_text()
    for py in (ROOT / "optiprimex").rglob("*.py"):                          # a hub default of 1883 anywhere in the package
        for ln in py.read_text().splitlines():
            if "1883" in ln and re.search(r"default=1883|port=1883|port or 1883|:1883\"\)|port: int = 1883", ln):
                assert re.search(r"RC_|judge|UAV_BROKER|bp or 1883", ln), f"{py.name}: {ln.strip()[:120]}"
