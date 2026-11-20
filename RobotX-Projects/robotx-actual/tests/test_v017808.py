"""v0.1.78.8: the boat missed the course (Dmitrii 2026-11-09 13:02, Team Village): the OCS forwarded course ALPHA at 13:02:10.2, the boat's
bridge came up at 13:02:11 and its mission manager at 13:02:14 - COURSE TIMED OUT, never repeated. Now: per boat, the course it acknowledged;
re-sent every COURSE_RESEND_S while the boat is online and has not; again after a status gap (a reconnect / a restart); not after a refusal."""
from __future__ import annotations

import yaml

from optiprimex.companion.config import Config
from optiprimex.ocs.ocs_app import OcsApp

from .conftest import ROOT
from .test_ocs_units import FakeBus
from .test_v0164 import RJudge, SQ

COURSE = {"course_id": "ALPHA", "boundary": SQ, "uav_transit_area": [], "pinger_hz": 25000}


def _app():
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "course" / "ocs_course.yaml").read_text())
    app = OcsApp(cfg, bus=FakeBus(), judge=RJudge())
    app.lifecycle.on_link(True)
    return app


def _courses(app):
    return [e for e in app.authority.sent if e["action"] == "COURSE" and e["vehicle"] == "USV1"]


def _status(app, t):
    app._status_mono["USV1"] = t                                        # what on_status records (its own clock)


def test_the_13_02_sequence(monkeypatch):
    app = _app()
    app.on_course(dict(COURSE))                                         # 13:02:10 - the boat is not up yet
    assert len(_courses(app)) == 1
    t0 = app._course_sync["USV1"]["t_sent"]
    assert app.course_resend_tick(now=t0 + 30) == []                    # offline (no status): nothing re-sent
    app.on_status("s", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL"}})   # 13:02:14 - its first status
    t1 = app._status_mono["USV1"]
    assert app.course_resend_tick(now=t1 + 0.5) == ["USV1"] and len(_courses(app)) == 2      # sent before it was up = lost: at once
    _status(app, t1 + 2)
    assert app.course_resend_tick(now=t1 + 2) == []                     # < 6 s after the re-send: wait for the ack
    _status(app, t1 + 8)
    assert app.course_resend_tick(now=t1 + 8) == ["USV1"] and len(_courses(app)) == 3        # no ack in 6 s: again
    app._course_ack({"ack_of": app._course_sync["USV1"]["msg_id"], "result": "DONE"})
    _status(app, t1 + 40)
    assert app.course_resend_tick(now=t1 + 40) == [] and len(_courses(app)) == 3              # acknowledged: never again


def test_a_status_gap_asks_again_and_a_refusal_stops():
    app = _app()
    app.on_course(dict(COURSE))
    app._course_ack({"ack_of": app._course_sync["USV1"]["msg_id"], "result": "ACCEPTED"})
    app.on_status("s", {"vehicle_id": "USV1", "body": {"state": "MANUAL", "robot_state": "STATE_MANUAL"}})
    app._status_mono["USV1"] -= 10                                     # the 13:02:40-13:03:15 drop: no status for 10 s
    app.on_status("s", {"vehicle_id": "USV1", "body": {"state": "KILLED", "robot_state": "STATE_KILLED"}})
    now = app._status_mono["USV1"]
    assert app.course_resend_tick(now=now) == ["USV1"]                 # it may have lost it: again
    app._course_ack({"ack_of": app._course_sync["USV1"]["msg_id"], "result": "REJECTED", "reason": "test"})
    assert app.course_resend_tick(now=now + 60) == []                   # refused: not asked again for this course
    app.on_course(dict(COURSE, course_id="BRAVO"))                      # a new course: sent, and pending again
    assert _courses(app)[-1]["msg_id"] == app._course_sync["USV1"]["msg_id"]
