"""v0.1.53 (Dmitrii 2026-11-06): the USV's FCU geofence - "USV only, with a button to apply to FCU, with acknowledgment from Mavlink
(... we click apply (in map page), we change geofence, we click apply, we can toggle off, which is by default)".

The MAVLink fence protocol against the fake FCU (upload, read back identical within 1e-6 deg, DO_FENCE_ENABLE acks, a breach in
FENCE_STATUS + HOLD), the vehicle's FENCE_APPLY / FENCE_ENABLE end to end (status/usv `fence`), the authority's checks (inside the
received course, AUTO_TASK refusals), the words, the map's and the boat window's boxes."""
from __future__ import annotations

import json
import time
import types

import pytest

from optiprimex.companion import fence as F
from optiprimex.companion.config import Config
from optiprimex.companion.mqtt import Envelope
from optiprimex.ocs.cmd_authority import GUARDS, CmdAuthority
from optiprimex.ocs.run_lifecycle import RunLifecycle
from optiprimex.tools import ocs_cli

from .conftest import PY, ROOT, FakeFcuHandle, Proc, free_udp_port, wait_until
from .test_v0145 import stack  # noqa: F401  (the live chain: fake FCU, usv_ui, the mission manager, boat_web)

TOOLS = ROOT / "optiprimex" / "tools"
LAT0, LON0 = 1.2810202, 103.8550097                          # the fake FCU's start (Course Alpha's centre)
D = 0.0004                                                   # ~45 m
SQUARE = [[LAT0 - D, LON0 - D], [LAT0 - D, LON0 + D], [LAT0 + D, LON0 + D], [LAT0 + D, LON0 - D]]
COURSE = {"course_id": "C1", "boundary": [[LAT0 - 2 * D, LON0 - 2 * D], [LAT0 - 2 * D, LON0 + 2 * D], [LAT0 + 2 * D, LON0 + 2 * D],
                                          [LAT0 + 2 * D, LON0 - 2 * D], [LAT0 - 2 * D, LON0 - 2 * D]]}     # 3.4.9: first == last


# ---- the rules (pure) ------------------------------------------------------------------------------------------------------
def test_the_polygon_rules():
    closed = SQUARE + [SQUARE[0]]
    assert F.ring(closed) == [tuple(p) for p in SQUARE]                       # first == last: the closing point is dropped
    assert F.problem(SQUARE) is None and F.problem(closed) is None
    assert "3 distinct" in F.problem(SQUARE[:2]) and "3 distinct" in F.problem([SQUARE[0]] * 4)
    assert "range" in F.problem([[95, 0], [0, 1], [1, 1]]) and "lat, lon" in F.problem([["x", 1], [1, 2], [2, 3]])
    assert F.same(SQUARE, closed)[0]
    near = [[a + 5e-7, b] for a, b in SQUARE]
    assert F.same(SQUARE, near)[0]                                            # within 1e-6 deg
    far = [[a + 2e-6, b] for a, b in SQUARE]
    ok, why = F.same(SQUARE, far)
    assert not ok and "point 1" in why
    assert not F.same(SQUARE, SQUARE[:3])[0]
    assert F.contains(SQUARE, LAT0, LON0) and not F.contains(SQUARE, LAT0 + 2 * D, LON0)
    assert F.outside_course(SQUARE, COURSE["boundary"]) is None
    big = [[a * 1.0 + (a - LAT0) * 2, b + (b - LON0) * 2] for a, b in SQUARE]    # 3x the square: leaves the 2x course
    why = F.outside_course(big, COURSE["boundary"])
    assert why and "leaves the course boundary by" in why


def test_the_book_never_fabricates():
    bk = F.FenceBook()
    fcu = types.SimpleNamespace(ages={}, fence_sys_present=None, fence_sys_enabled=None, fence_breach_status=None)
    v = bk.view(fcu)
    assert v["enabled"] is None and v["points_n"] is None and v["matches"] is None and v["breach"] is None and v["source"] is None
    bk.enable_ack = True
    assert bk.view(fcu)["enabled"] is True and bk.view(fcu)["enabled_from"] == "ack"
    fcu = types.SimpleNamespace(ages={"SYS_STATUS": 0.5, "FENCE_STATUS": 0.4}, fence_sys_present=True, fence_sys_enabled=False,
                                fence_breach_status=1, fence_breach_count=2, fence_breach_type=3)
    v = bk.view(fcu)
    assert v["enabled"] is False and v["enabled_from"] == "sys_status" and v["breach"] == {"status": True, "count": 2, "type": 3}
    fcu.ages = {"SYS_STATUS": 9.0, "FENCE_STATUS": 9.0}                       # stale -> the FCU flags do not count
    v = bk.view(fcu)
    assert v["enabled"] is True and v["breach"] is None


# ---- the protocol against the fake FCU (a MavLink straight on it) ---------------------------------------------------------------
@pytest.fixture(scope="module")
def direct(tmp_path_factory):
    from optiprimex.companion.link import MavLink
    tmp = tmp_path_factory.mktemp("fence")
    mav_port, ctrl_port = free_udp_port(), free_udp_port()
    proc = Proc([PY, "-m", "optiprimex.sim.fake_fcu", "--conn", f"udpout:127.0.0.1:{mav_port}", "--control-port", str(ctrl_port),
                 "--armed", "--mode", "15"], tmp / "fcu.log")
    fcu = FakeFcuHandle(proc, mav_port, ctrl_port)
    assert wait_until(lambda: fcu.ctrl({"get": "state"}, timeout_s=0.3) is not None, 5), "fake FCU did not start"
    link = MavLink(Config(), endpoint=f"udpin:127.0.0.1:{mav_port}")
    link.start()
    assert wait_until(lambda: link.state().seen_heartbeat, 5)
    yield link, fcu
    link.stop()
    proc.kill()


def test_the_fence_protocol_end_to_end(direct):
    link, fcu = direct
    assert link.fence_enable(True) == 4                                       # MAV_RESULT_FAILED: no polygon yet (the fake's rule)
    assert wait_until(lambda: link.state().ages.get("SYS_STATUS", 99) < 1.5, 3)
    assert link.state().fence_sys_present is False                            # no polygon: the geofence bit is not present
    ok, txt = link.fence_upload(SQUARE + [SQUARE[0]])                         # a closed ring: uploaded as 4 vertices
    assert ok and "4 points accepted" in txt and "MAV_MISSION_ACCEPTED" in txt, txt
    st = fcu.state()["fence"]
    assert len(st["points"]) == 4 and st["enabled"] is False                  # uploading never enables
    items, why = link.fence_download()
    assert items is not None, why
    assert len(items) == 4 and all(c == 5001 and p1 == 4.0 for c, p1, _a, _b in items)     # NAV_FENCE_POLYGON_VERTEX_INCLUSION, count
    assert F.same(SQUARE, [(a, b) for _c, _p, a, b in items]) == (True, "")
    assert wait_until(lambda: link.state().ages.get("FENCE_STATUS", 99) < 1.5, 3)          # 1 Hz while a polygon is loaded
    assert link.state().fence_breach_status == 0 and wait_until(lambda: link.state().fence_sys_present is True, 3)
    assert link.state().fence_sys_enabled is False
    assert link.fence_enable(True) == 0                                       # MAV_RESULT_ACCEPTED
    assert wait_until(lambda: link.state().fence_sys_enabled is True, 3)
    # a breach: the plant outside the polygon -> FENCE_STATUS breach 1, count 1, BOUNDARY; armed -> HOLD (FENCE_ACTION hold)
    assert fcu.state()["mode"] == 15
    fcu.ctrl({"teleport": {"lat": LAT0 + 3 * D, "lon": LON0}})
    assert wait_until(lambda: link.state().fence_breach_status == 1, 4)
    s = link.state()
    assert s.fence_breach_count == 1 and s.fence_breach_type == 3
    assert wait_until(lambda: link.state().fence_sys_healthy is False, 3)    # the next 1 Hz SYS_STATUS: the geofence unhealthy
    assert wait_until(lambda: fcu.state()["mode"] == 4, 2)
    fcu.ctrl({"teleport": {"lat": LAT0, "lon": LON0}})
    assert wait_until(lambda: link.state().fence_breach_status == 0, 4)
    assert link.fence_enable(False) == 0 and wait_until(lambda: link.state().fence_sys_enabled is False, 3)
    # the FCU refuses an upload: the result is named, the old fence stays
    fcu.set(fence_reject=4)
    ok, txt = link.fence_upload(SQUARE[:3])
    assert not ok and txt == "the FCU refused: MAV_MISSION_NO_SPACE"
    assert len(fcu.state()["fence"]["points"]) == 4


def test_no_mission_ack_is_said(direct):
    link, fcu = direct
    real = link._conn.mav.mission_count_send
    link._conn.mav.mission_count_send = lambda *a, **k: None                  # the FCU never hears it
    try:
        ok, txt = link.fence_upload(SQUARE, timeout_s=0.3, retries=2)
    finally:
        link._conn.mav.mission_count_send = real
    assert not ok and txt == "no MISSION_ACK after 2 attempts"


# ---- the vehicle's commands --------------------------------------------------------------------------------------------------
class _Link:
    def __init__(self):
        self.calls = []

    def fence_upload(self, pts):
        self.calls.append(("up", list(pts)))
        return True, "ok"

    def fence_download(self):
        return [(5001, 3.0, a, b) for a, b in self.calls[-1][1]] if self.calls else [], ""

    def fence_enable(self, on):
        self.calls.append(("en", on))
        return 0


def _handler(state):
    from optiprimex.usv.command_handler import CommandHandler
    fsm = types.SimpleNamespace(state=state)
    h = CommandHandler(Config(), bus=None, fsm=fsm, run=None, runner=types.SimpleNamespace(task_name="T1"), clock=time.monotonic)
    h.link = _Link()
    return h, fsm


def test_the_vehicle_refuses_in_auto_task_and_checks_the_polygon():
    from optiprimex.companion.state import VState
    h, fsm = _handler(VState.V4)
    r = h._fence("FENCE_APPLY", {"points": SQUARE, "source": "course"})
    assert r[0] == "REJECTED" and "AUTO_TASK" in r[1]
    assert h._fence("FENCE_ENABLE", {"on": True})[0] == "REJECTED"
    r = h._fence("FENCE_ENABLE", {"on": False})                               # off always passes
    assert r[0] == "ACCEPTED" and wait_until(lambda: h.fence.result is not None, 3)
    assert h.fence.result == (True, "fence OFF: MAV_RESULT_ACCEPTED") and h.fence.enable_ack is False
    fsm.state = VState.V3
    assert "3 distinct" in h._fence("FENCE_APPLY", {"points": SQUARE[:2]})[1]
    assert h._fence("FENCE_ENABLE", {"on": "yes"})[0] == "REJECTED"
    r = h._fence("FENCE_APPLY", {"points": SQUARE + [SQUARE[0]], "source": "geofence"})
    assert r[0] == "ACCEPTED" and not r[2]
    assert wait_until(lambda: h.fence.result is not None, 3)
    assert h.fence.result == (True, "fence 4 points uploaded, read back identical")
    assert h.fence.points_n == 4 and h.fence.matches is True and h.fence.source == "geofence" and h.fence.applied_t
    h.link = None
    assert h._fence("FENCE_ENABLE", {"on": True})[1] == "no FCU link in this mission manager"


def test_fence_apply_and_enable_through_the_mission_manager(stack):  # noqa: F811
    vc, fcu, hub = stack["vc"], stack["fcu"], stack["hub"]

    def fence():
        m = hub.last("team/optiprime/v1/status/usv")
        return ((m or {}).get("env", {}).get("body") or {}).get("fence")

    assert wait_until(lambda: fence() is not None, 5)
    f0 = fence()
    assert f0["points_n"] is None and f0["matches"] is None and f0["source"] is None          # nothing applied: unknown, not 0
    acks = vc.send("run", "FENCE_APPLY", args={"points": SQUARE, "source": "geofence"}, wait_s=12)
    res = [a["body"]["result"] for a in acks]
    assert res[0] == "ACCEPTED" and res[-1] == "DONE", acks
    assert acks[-1]["body"]["reason"] == "fence 4 points uploaded, read back identical"
    assert wait_until(lambda: (fence() or {}).get("matches") is True, 5)
    f = fence()
    assert f["points_n"] == 4 and f["source"] == "geofence" and f["applied_t"]
    assert wait_until(lambda: (fence() or {}).get("enabled") is False, 5), fence()      # the next SYS_STATUS: present, off
    acks = vc.send("run", "FENCE_ENABLE", args={"on": True}, wait_s=12)
    assert acks[-1]["body"]["result"] == "DONE" and "fence ON (the FCU holds 4 points): MAV_RESULT_ACCEPTED" in acks[-1]["body"]["reason"], acks
    assert wait_until(lambda: (fence() or {}).get("enabled") is True, 5) and fence()["enabled_from"] == "sys_status"
    assert wait_until(lambda: (fence() or {}).get("breach") == {"status": False, "count": 0, "type": 0}, 5), fence()
    acks = vc.send("run", "FENCE_ENABLE", args={"on": False}, wait_s=12)
    assert acks[-1]["body"]["result"] == "DONE" and wait_until(lambda: (fence() or {}).get("enabled") is False, 5)
    fcu.set(fence_reject=5)
    acks = vc.send("run", "FENCE_APPLY", args={"points": SQUARE[:3], "source": "geofence"}, wait_s=12)
    assert acks[-1]["body"]["result"] == "FAILED" and acks[-1]["body"]["reason"] == "fence 3 points: the FCU refused: MAV_MISSION_INVALID", acks


# ---- the authority ----------------------------------------------------------------------------------------------------------
class _Bus:
    def __init__(self, geofence=None):
        self.cfg = Config()
        self.cfg.vehicle_id = "OCS"
        self.cfg.extra["geofence"] = geofence or []
        self.run_id = self.declaration_seq = None
        self.published = []

    def publish(self, topic, kind, body, qos=1, intent_id=None, vehicle_id=None, origin=None, **kw):
        self.published.append({"topic": topic, "body": body})
        return f"m{len(self.published)}"


def _intent(action, args=None):
    return Envelope.wrap("intent", {"action": action, "task": None, "args": args or {}, "operator": "t"}, "USV1", 1)


def _auth(course=True, geofence=None):
    lc = RunLifecycle()
    lc.on_link(True)
    if course:
        lc.on_course(COURSE)
    bus = _Bus(geofence)
    return bus, CmdAuthority(bus, lc, {"USV1": "usv"}, False, lambda: True, lambda r: None)


def test_the_authority_checks_the_polygon(tmp_path, monkeypatch):
    monkeypatch.delenv("OCS_CONFIG", raising=False)
    assert GUARDS["FENCE_APPLY"][:3] == (0, 0, "FENCE_APPLY") and GUARDS["FENCE_ENABLE"][:3] == (0, 0, "FENCE_ENABLE")
    bus, a = _auth()
    r = a.on_intent("t", _intent("FENCE_APPLY", {"source": "course"}))
    assert r.result == "accepted" and "4 points (course)" in r.reason, r
    b = bus.published[-1]["body"]
    assert b["action"] == "FENCE_APPLY" and b["args"]["source"] == "course" and len(b["args"]["points"]) == 4   # the closing point dropped
    assert bus.published[-1]["topic"].endswith("cmd/usv/run")
    r = a.on_intent("t", _intent("FENCE_APPLY", {"source": "geofence"}))
    assert r.result == "refused" and "no geofence" in r.reason
    big = [[a_ + (a_ - LAT0) * 2, b_ + (b_ - LON0) * 2] for a_, b_ in SQUARE]
    r = a.on_intent("t", _intent("FENCE_APPLY", {"source": "geofence", "points": big}))
    assert r.result == "refused" and "leaves the course boundary" in r.reason and "inside the received course" in r.reason
    r = a.on_intent("t", _intent("FENCE_APPLY", {"source": "geofence", "points": SQUARE}))
    assert r.result == "accepted" and bus.published[-1]["body"]["args"]["points"][0] == [round(SQUARE[0][0], 7), round(SQUARE[0][1], 7)]
    prof = tmp_path / "ocs.yaml"                                              # the profile's geofence, read at the intent (the MAP saved it)
    prof.write_text("geofence: " + json.dumps(SQUARE[:3]) + "\n")
    a.geofence_file = str(prof)
    r = a.on_intent("t", _intent("FENCE_APPLY", {"source": "geofence"}))
    assert r.result == "accepted" and len(bus.published[-1]["body"]["args"]["points"]) == 3
    assert "course | geofence" in a.on_intent("t", _intent("FENCE_APPLY", {"source": "x"})).reason
    a.last_vehicle_state["USV1"] = "AUTO_TASK"
    r = a.on_intent("t", _intent("FENCE_APPLY", {"source": "course"}))
    assert r.result == "refused" and "AUTO_TASK" in r.reason
    assert a.on_intent("t", _intent("FENCE_ENABLE", {"on": True})).result == "refused"
    n = len(bus.published)
    r = a.on_intent("t", _intent("FENCE_ENABLE", {"on": False}))
    assert r.result == "accepted" and bus.published[-1]["body"]["args"] == {"on": False} and len(bus.published) == n + 1
    # no course received: course refused, a geofence goes without the inside rule
    bus, a = _auth(course=False, geofence=big)
    assert "no course received" in a.on_intent("t", _intent("FENCE_APPLY", {"source": "course"})).reason
    assert a.on_intent("t", _intent("FENCE_APPLY", {"source": "geofence"})).result == "accepted"      # the profile at start (cfg.extra)


# ---- the words, the tables, the pages ----------------------------------------------------------------------------------------
def test_the_words():
    sp = lambda argv: ocs_cli.intent_spec(ocs_cli.parse(argv))[:3]           # noqa: E731
    assert sp(["fence", "apply", "course"]) == ("FENCE_APPLY", None, {"source": "course"})
    assert sp(["fence", "apply", "geofence", "--points", "1.1,103.1;1.2,103.2;1.3,103.1"]) == (
        "FENCE_APPLY", None, {"source": "geofence", "points": [[1.1, 103.1], [1.2, 103.2], [1.3, 103.1]]})
    assert sp(["fence", "on"]) == ("FENCE_ENABLE", None, {"on": True}) and sp(["fence", "off"]) == ("FENCE_ENABLE", None, {"on": False})
    assert ocs_cli.execute(None, ocs_cli.parse(["fence", "apply"]))[1][0].startswith("usage: fence apply")
    from optiprimex.tools import bench_tui as T
    assert "fence" in T.INTENT_WORDS and T.WORD_TO_ACTION["fence"] == "FENCE_APPLY" and any(w == "fence" for w, _u, _ in T.USAGE)
    from optiprimex.tools import playbook
    assert {"FENCE_APPLY", "FENCE_ENABLE"} <= set(playbook.PLAYBOOK)
    st = {"r": "R2", "vehicles": {"USV1": {"state": "AUTO_TASK"}}}
    assert not playbook.check("FENCE_APPLY", st).ok and playbook.check("FENCE_ENABLE", st).ok
    st["vehicles"]["USV1"]["state"] = "MANUAL"
    assert playbook.check("FENCE_APPLY", st).ok
    from optiprimex import schemas
    from optiprimex.usv.run_context import RUN_FREE_ACTIONS
    assert {"FENCE_APPLY", "FENCE_ENABLE"} <= set(schemas.COMMAND_ACTIONS) and {"FENCE_APPLY", "FENCE_ENABLE"} <= set(RUN_FREE_ACTIONS)


def test_the_map_and_the_boat_window_boxes():
    m = (TOOLS / "map.html").read_text()
    for i in ("fcubox", "fcu_src", "b_fapply", "b_fon", "b_foff", "fcustat"):
        assert f'id="{i}"' in m, i
    import re
    box = m.split('<span id="fcubox"', 1)[1].split("</span></span>", 1)[0]
    for b in re.findall(r"<(?:button|select)[^>]*>", box):
        assert 'title="' in b, b                                              # every control has a title
    assert "body.mini #fcubox, body.judge #fcubox { display:none !important; }" in m
    assert "if (!JUDGE && !MINI) {" in m and "'fence apply course'" in m and "fenceSend('fence on'" in m and "fenceSend('fence off'" in m
    assert "/state.json" in m and "st.status.fence" in m
    b = (TOOLS / "boat_web.html").read_text()
    assert 'id="fence-now"' in b and "c.fence" in b
    from optiprimex.tools import boat_web as BW
    model = BW.Model("usv")
    assert model.snapshot()["fcu_ctl"]["fence"] is None                       # nothing seen: null, never invented
