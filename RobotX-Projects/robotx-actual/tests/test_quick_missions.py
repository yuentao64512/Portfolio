"""`make quick` (v0.1.26, WP7): every built mission once in the fast SIL (no brokers, simulated time) - the ones the SIL modules
do not already cover: m4t0 (T4: AssistanceRequest -> transit -> readiness -> confirm), goto, m0t0 (TEST_MANUAL by STEP), m0t1
(TEST_AUTO on its timers). m1t1 / m0t2 / m3t1 / m0t3 run in tests/test_t1_sim.py, test_t3_sim.py, test_stage_stepping.py."""
from __future__ import annotations

import math

from optiprimex.companion.config import Config
from optiprimex.sim.fast_sil import FastSil

from .conftest import ROOT

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")


def test_m4t0_t4_chain_fast():
    s = FastSil(POOL, "T4", {})
    lat, lon = s.origin.from_local(20.0, 5.0)
    s.run(1.0)
    assert s.task.m.state == "LISTEN"
    s.task.on_incident({"kind": "assistance_request", "command_seq": 7, "position": {"lat": lat, "lon": lon}}, s.ctx())
    s.run(120, until=lambda x: x.task.m.state == "WAIT_CONFIRM")
    assert s.task.m.state == "WAIT_CONFIRM", s.task.m.state
    p = s.pose()
    assert math.hypot(p.x_n - 20.0, p.y_e - 5.0) < 3.0                       # at the request position
    kinds = [e.get("kind") for e in s.events if e.get("event") == "incident"]
    assert kinds == ["ack", "readiness"]
    s.task.on_incident({"kind": "readiness_confirm", "command_seq": 7}, s.ctx())
    assert s.run(5)["state"] == "DONE"


def test_goto_fast():
    r = FastSil(POOL, "GOTO", {"x_n": 15.0, "y_e": -4.0, "speed_mps": 1.0}).run(120)
    assert r["state"] == "DONE" and math.hypot(r["track"][-1][1] - 15.0, r["track"][-1][2] + 4.0) < 2.0


def test_m0t1_timers_and_m0t0_steps_fast():
    cfg = Config()
    cfg.extra["test_auto"] = {"step_s": 1.0}
    r = FastSil(POOL, "TEST_AUTO", {}, cfg=cfg).run(10)
    assert r["state"] == "DONE" and r["states"] == ["S2", "S3", "DONE"]
    s = FastSil(POOL, "TEST_MANUAL", {})
    for want in ("S1 -> S2", "S2 -> S3", "S3 -> DONE"):
        assert s.task.on_step(s.ctx()) == want
    assert s.run(1)["state"] == "DONE"
