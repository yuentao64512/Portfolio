from types import SimpleNamespace

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin
from optiprimex.companion.state import Command, FcuState, LocalPose, RunContext
from optiprimex.tools import ocs_cli, playbook
from optiprimex.ocs.report_mapper import event_to_report, internal_event
from optiprimex.usv.behaviours.base import BehaviourContext
from optiprimex.usv.box_inspection import BoxSample, BoxTrack, FaceGeometry, PointNE
from optiprimex.usv.tasks import REGISTRY
from optiprimex.usv.tasks.base import TaskContext
from optiprimex.usv.vehicle_fsm import VehicleFsm


def stable_track(box_id, colour, n, e):
    tr = BoxTrack(box_id)
    face = FaceGeometry(PointNE(n, e - 1), PointNE(n, e), PointNE(n, e + 1))
    for i in range(22):
        ts = i * 0.2
        tr.add(BoxSample(ts, box_id, face, colour, int(ts) % 2 == 0, 0.95))
    return tr


def test_registry_cli_and_task_wrapper_vertical_slice():
    assert "BOX_INSPECT" in REGISTRY
    assert playbook.resolve("m0t4sim") == ("m0t4", "BOX_INSPECT", None, True)
    ns = ocs_cli.parse(["start_task", "m0t4sim", "--target-colour", "green", "--expected-boxes", "3"])
    assert ocs_cli.intent_spec(ns)[:3] == (
        "START_TASK", "BOX_INSPECT", {"sim": True, "target_colour": "green", "expected_boxes": 3}
    )

    cfg = Config()
    fcu = FcuState(gps_fix=4, hdg_deg=0, params_ok=True)
    pose = LocalPose(0, 0, 0, 1.3, 103.8, 5.0)
    pmap = SimpleNamespace(
        last_ingest_t=5.0,
        box_tracks={
            1: stable_track(1, "red", 50, -6),
            2: stable_track(2, "green", 50, 0),
            3: stable_track(3, "blue", 50, 6),
        },
    )
    pmap.stable_boxes = lambda: [t.stable() for t in pmap.box_tracks.values()]
    events = []
    tc = TaskContext(
        b=BehaviourContext(cfg=cfg, now=5.0, fcu=fcu, pose=pose, origin=ControlOrigin(1.3, 103.8),
                           run=RunContext(run_id=1), map=pmap),
        emit_event=events.append,
    )
    task = REGISTRY["BOX_INSPECT"](cfg)
    task.start({"sim": True, "target_colour": "green", "expected_boxes": 3}, tc)
    first = task.step(tc)
    assert first.sub_state == "ASK_OCS" and events[-1].kind == "box_survey"
    second = task.step(tc)
    assert second.sub_state == "ALIGN"
    assert second.out is not None and second.out.cmd.__class__.__name__ == "GuidedTarget"


def test_vehicle_layer_accepts_box_inspection_task():
    fsm = VehicleFsm(Config())
    fsm.run_id = 1001
    cmd = Command(
        env={"msg_id": "box-start", "body": {}},
        action="START_TASK",
        task="BOX_INSPECT",
        args={"target_colour": "green"},
    )
    assert fsm.can_start_task(cmd) == (True, "")


def test_box_events_stay_ocs_internal():
    # v0.1.27 (Dmitrii 2026-11-04): not RoboNation reports - the OCS keeps them, the judge never sees them
    survey_ev = {"event": "box_survey", "boxes": [{"box_id": 2, "colour": "green"}]}
    done_ev = {"event": "box_mission_complete", "target": {"box_id": 2}, "stop_error_m": 0.9}
    assert event_to_report("USV1", survey_ev) is None and event_to_report("USV1", done_ev) is None
    survey = internal_event(survey_ev)
    assert survey["kind"] == "box_survey" and survey["boxes"][0]["colour"] == "green"
    assert internal_event(done_ev)["complete"] is True and internal_event({"event": "docking"}) is None


def test_select_target_cli_contract():
    ns = ocs_cli.parse(["select_target", "green"])
    assert ocs_cli.intent_spec(ns) == ("SELECT_TARGET", None, {"target_colour": "green"}, None)

