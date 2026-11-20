"""v0.1.29.3 - the pool-log fixes (docs/POOL_REPORT_2026-11-04.md §7): F1 the RC health read "lost" all day on a live
transmitter (the real link sent RC_CHANNELS with chancount 0; the FCU's own receiver bit now decides when it reports one) and the
decided `guards.rc_lost_action` (report | hold); F5 the perception age is null ("never") before the first detection, not 1e9;
F6 the "RC override active" refusal says what to do, the playbook predicts it, MAIN's checklist has the row."""
from __future__ import annotations

import os

from optiprimex.companion.config import Config, load
from optiprimex.companion.health import HealthMonitor
from optiprimex.companion.state import FcuState, Health, rc_health
from optiprimex.tools import ocs_web, playbook
from optiprimex.usv.vehicle_fsm import VehicleFsm

from .conftest import PY, ROOT, Probe, Proc, make_usv_config, wait_until

NS = "team/optiprime/v1"
INF = float("inf")


def fcu(age=0.2, chancount=16, plausible=True, sys_present=None, sys_healthy=None, sys_age=0.5):
    s = FcuState(seen_heartbeat=True, rc_chancount=chancount, rc_last_plausible=plausible, rc_rx_present=sys_present, rc_rx_healthy=sys_healthy)
    s.ages = {"RC_CHANNELS": age, "SYS_STATUS": sys_age, "HEARTBEAT": 0.1}
    return s


def test_rc_health_per_source():
    ok, why = rc_health(fcu(chancount=0), 2.0)                                       # the pool: chancount 0, good channels, no bit
    assert ok and "chancount 0" in why and "plausible" in why
    assert rc_health(fcu(chancount=0, sys_present=True, sys_healthy=True), 2.0)[0]   # ArduPilot: the receiver bit says healthy
    ok, why = rc_health(fcu(sys_present=True, sys_healthy=False), 2.0)                # an RC failsafe: the bit wins over the values
    assert not ok and "UNHEALTHY" in why
    ok, why = rc_health(fcu(age=3.0), 2.0)                                            # no fresh RC_CHANNELS: no RC5 / RC6 either
    assert not ok and "not fresh" in why
    assert not rc_health(fcu(age=INF), 2.0)[0] and "never" in rc_health(fcu(age=INF), 2.0)[1]
    assert not rc_health(fcu(chancount=0, plausible=False), 2.0)[0]                   # nothing in the channels, no bit
    assert rc_health(fcu(chancount=0, sys_present=True, sys_healthy=False, sys_age=10.0), 2.0)[0]   # a stale bit is not used
    assert rc_health(fcu(chancount=16, plausible=False), 2.0)[0]                      # the fake FCU's case (chancount 16)


def test_rc_lost_action_report_or_hold():
    cfg = Config()
    assert cfg.guards.rc_lost_action == "report"
    hm = HealthMonitor(cfg)
    lost = fcu(age=5.0)
    lost.rc_ok, lost.rc_detail = rc_health(lost, 2.0)
    h = hm.update(lost, True, 0.5, 100.0)
    assert "RC_LOST" in h.faults and "RC_LOST" not in hm.soft_faults() and "RC_LOST" not in hm.hard_faults() and h.rc_detail
    cfg.guards.rc_lost_action = "hold"
    hm.update(lost, True, 0.5, 100.1)
    assert "RC_LOST" in hm.soft_faults() and "RC_LOST" not in hm.hard_faults()
    f = VehicleFsm(cfg)
    f.inputs.soft_faults = ["RC_LOST"]
    ok, why = f.can_enable_auto()
    assert not ok and "transmitter is lost" in why
    here = os.path.dirname(__file__)
    for name in ("pool/usv_pool.yaml", "course/usv_course.yaml"):
        assert load(os.path.join(here, "..", "profiles", name)).guards.rc_lost_action == "report"      # on after the lab day
    assert "RC_LOST" in load(os.path.join(here, "..", "profiles", "pool", "usv_lab.yaml")).guards.ignore_faults


def test_f5_never_and_f6_what_to_do():
    assert Health().body()["perception_age_s"] is None                               # never seen -> null, the consoles say never
    src = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text()
    assert 'self._perception_seen = float("-inf")' in src
    f = VehicleFsm(Config())
    i = f.inputs
    i.rc_override, i.rc6_high, i.mode_name = True, False, "MANUAL"
    ok, why = f.can_enable_auto()
    assert not ok and why.startswith("the FCU is in MANUAL") and "put it in HOLD, then enable_auto" in why
    i.rc6_high = True
    assert "RC6 override switch is high" in f.can_enable_auto()[1]
    i.rc6_high, i.tui_active = False, True
    assert "handover" in f.can_enable_auto()[1]
    st = {"r": "R3", "declaration_seq": 1, "vehicles": {"USV1": {"state": "MANUAL"}}}
    v = playbook.check("ENABLE_AUTO", st, "USV1", status={"state": "MANUAL", "armed": True, "mode": "MANUAL"})
    assert not v.ok and "MANUAL" in v.why and "HOLD" in v.hint
    snap = {"hub_up": True, "status_age": 0.3, "ui_active": False, "perception_age": 0.5,
            "ocs": {"r": "R3", "judge_connected": True, "course_id": "C"}, "status": {"state": "MANUAL", "armed": True, "mode": "MANUAL"}}
    c = ocs_web.clearance(snap)
    assert "FCU not in MANUAL, RC6 low" in c["blocked"] and not c["clear"]
    snap["status"]["mode"] = "HOLD"
    snap["local"] = {"fcu": {"rc": {"6": 1900}}}
    assert "FCU not in MANUAL, RC6 low" in ocs_web.clearance(snap)["blocked"]
    snap["local"]["fcu"]["rc"]["6"] = 1000
    assert ocs_web.clearance(snap)["clear"]


def test_rc_health_live_against_the_fake_fcu(brokers, fake_fcu, tmp_path):
    """The pool's case end to end: the fake sends chancount 0 (as the real Pixhawk did) -> still rc ok; the SYS_STATUS receiver
    bit healthy / unhealthy decides when present; the stream stops -> rc NO within rc_s. status/usv carries rc_detail."""
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(make_usv_config(tmp_path, brokers, fake_fcu))], tmp_path / "mm.log")
    p = Probe(brokers.hub_port, [f"{NS}/status/usv"]).start()

    def health_now(pred, what):
        def last_ok():
            m = p.last(f"{NS}/status/usv")
            return m is not None and pred(m["env"]["body"].get("health") or {})
        assert wait_until(last_ok, 8), f"{what}: {(p.last(f'{NS}/status/usv') or {}).get('env', {}).get('body', {}).get('health')}\n{mm.log()[-1500:]}"
        return p.last(f"{NS}/status/usv")["env"]["body"]["health"]
    try:
        h = health_now(lambda h: h.get("rc_ok") is True, "the bench default")
        assert h["perception_age_s"] is None                                          # F5: no detection yet -> null
        fake_fcu.set(rc_chancount=0)
        h = health_now(lambda h: h.get("rc_ok") is True and "chancount 0" in (h.get("rc_detail") or ""), "chancount 0 (the pool)")
        fake_fcu.set(rc_sys="unhealthy")
        h = health_now(lambda h: h.get("rc_ok") is False and "UNHEALTHY" in (h.get("rc_detail") or ""), "receiver bit unhealthy")
        assert "RC_LOST" in h["faults"]
        fake_fcu.set(rc_sys="healthy")
        health_now(lambda h: h.get("rc_ok") is True and "healthy" in (h.get("rc_detail") or ""), "receiver bit healthy")
        fake_fcu.set(rc_lost=True)
        health_now(lambda h: h.get("rc_ok") is False and "not fresh" in (h.get("rc_detail") or ""), "the stream stopped")
        fake_fcu.set(rc_lost=False)
        health_now(lambda h: h.get("rc_ok") is True, "back")
    finally:
        fake_fcu.set(rc_lost=False, rc_sys="absent", rc_chancount=16)
        p.stop()
        mm.kill()
