"""v0.1.87.2 (Dmitrii 2026-11-12): on the water profiles KILLED clears to MANUAL by itself once the kill releases (no confirm_safe on
the page). FAULT still needs the confirm; the bench / lab profiles keep the confirm for KILLED too."""
from __future__ import annotations

from pathlib import Path

import pytest

from optiprimex.companion.config import load
from optiprimex.companion.state import FcuState, Health
from optiprimex.usv.vehicle_fsm import VehicleFsm

ROOT = Path(__file__).resolve().parents[1]


def fcu(kill=False) -> FcuState:
    f = FcuState(seen_heartbeat=True, mode_name="HOLD", armed=True, ekf_pos_ok=True, rc_chancount=16, rc_ok=True)
    f.rc = {5: 1000, 6: 1000, 8: 1500}
    f.kill = kill
    return f


def run(cfg_path: str, faults=()):
    fsm = VehicleFsm(load(ROOT / cfg_path))
    fsm.feed(fcu(), Health(), False, [], [])
    assert fsm.state == "MANUAL"
    fsm.feed(fcu(kill=True), Health(), False, [], [])
    assert fsm.state == "KILLED"
    fsm.feed(fcu(kill=True), Health(), False, [], [])
    assert fsm.state == "KILLED"                          # never while the kill is still asserted
    fsm.feed(fcu(kill=False), Health(), False, list(faults), [])
    return fsm.state


@pytest.mark.parametrize("p", ["profiles/pool/usv_pool.yaml", "profiles/course/usv_course.yaml"])
def test_killed_clears_to_manual_by_itself_on_the_water_profiles(p):
    assert run(p) == "MANUAL"


@pytest.mark.parametrize("p", ["profiles/bench/usv_bench.yaml", "profiles/pool/usv_lab.yaml"])
def test_bench_and_lab_still_wait_for_confirm_safe(p):
    assert run(p) == "KILLED"


def test_fault_still_needs_the_confirm_on_the_course():
    cfg = load(ROOT / "profiles/course/usv_course.yaml")
    assert cfg.guards.v1_exit_requires_confirm is False and cfg.guards.v6_exit_requires_confirm is True
