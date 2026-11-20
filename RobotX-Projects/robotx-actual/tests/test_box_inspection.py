import math

from optiprimex.usv.box_inspection import (
    ActionKind,
    BoxInspectionMission,
    BoxSample,
    BoxState,
    BoxTrack,
    FaceGeometry,
    MissionSnapshot,
    PointNE,
)


def face(n, e, width=2.0):
    return FaceGeometry(PointNE(n, e - width / 2), PointNE(n, e), PointNE(n, e + width / 2))


def track(box_id, colour, n, e, wrong_every=0):
    t = BoxTrack(box_id)
    # 5 Hz for 4.2 s; one-second on / one-second off.
    for i in range(22):
        ts = i * 0.2
        lit = int(ts) % 2 == 0
        seen = colour if not wrong_every or i % wrong_every else "blue"
        t.add(BoxSample(ts, box_id, face(n, e), seen, lit, 0.9))
    return t


def snap(pos, tracks, *, target=None, preflight=None, heading=0.0, visible=True, ocs=True, rtk=True):
    return MissionSnapshot(
        now=5.0,
        position=pos,
        heading_deg=heading,
        preflight=preflight or {"rtk": True, "heading": True, "camera": True, "thrusters": True, "ocs": True},
        tracks=tracks,
        target_colour=target,
        target_visible=visible,
        ocs_link=ocs,
        rtk_ok=rtk,
    )


def test_four_second_colour_and_blink_gate():
    good = track(1, "red", 30, 0)
    b = good.stable()
    assert b is not None and b.colour == "red" and b.colour_share == 1.0
    assert 0.4 <= b.transition_rate_hz <= 1.6

    short = BoxTrack(2)
    for i in range(10):
        short.add(BoxSample(i * 0.2, 2, face(30, 3), "green", i % 5 < 3, 0.9))
    assert short.stable() is None

    mixed = track(3, "green", 30, 6, wrong_every=2)
    assert mixed.stable() is None                 # under the 80% lit-frame colour rule


def test_approach_line_is_perpendicular_and_on_boat_side():
    f = face(30, 0, 4)
    p, hdg = BoxInspectionMission._approach_geometry(f, PointNE(0, 0), 15)
    assert p == PointNE(15, 0)
    assert abs(hdg - 0.0) < 1e-6
    assert math.isclose(math.hypot(p.n - f.midpoint.n, p.e - f.midpoint.e), 15.0)


def test_scan_report_select_align_validate_stop_flow():
    tracks = [track(1, "red", 30, -5), track(2, "green", 30, 0), track(3, "blue", 30, 5)]
    m = BoxInspectionMission(PointNE(0, 0), expected_boxes=3, stop_range_m=5.0)   # v0.1.27: the default is 1 m now

    # Preflight passes at the scan point and stable observations are already available.
    a = m.step(snap(PointNE(0, 0), tracks))
    assert a.kind == ActionKind.REPORT and m.state == BoxState.ASK_OCS
    assert len(a.body["boxes"]) == 3

    # OCS selects exactly one colour; mission computes the 15 m align and 5 m stop points.
    a = m.step(snap(PointNE(0, 0), tracks, target="green"))
    assert a.kind == ActionKind.GOTO and m.state == BoxState.ALIGN
    assert a.target == PointNE(15, 0)

    # Alignment includes both position and heading. Zero-duration validation is
    # collapsed in the same tick, so the next output is the 5 m approach target.
    a = m.step(snap(PointNE(15, 0), tracks, target="green", heading=0))
    assert a.kind == ActionKind.APPROACH and m.state == BoxState.STOP
    assert a.target == PointNE(25, 0)
    a = m.step(snap(PointNE(25.5, 0), tracks, target="green", heading=0))
    assert a.kind == ActionKind.DONE and m.state == BoxState.DONE


def test_faults_hold_and_wrong_colour_backs_away():
    tracks = [track(1, "red", 30, 0)]
    m = BoxInspectionMission(PointNE(0, 0), expected_boxes=1)
    assert m.step(snap(PointNE(0, 0), tracks, ocs=False)).kind == ActionKind.HOLD
    assert m.state == BoxState.HOLD

    m = BoxInspectionMission(PointNE(0, 0), expected_boxes=1)
    m.step(snap(PointNE(0, 0), tracks))                 # report
    m.step(snap(PointNE(0, 0), tracks, target="red")) # align command
    changed = [track(1, "blue", 30, 0)]
    a = m.step(snap(PointNE(15, 0), changed, target="red", heading=0))
    assert a.kind == ActionKind.BACK_AWAY and m.state == BoxState.ABORTED

