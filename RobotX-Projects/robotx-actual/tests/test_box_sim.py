"""m0t4sim live (ROADMAP WP8, v0.1.27): BOX_INSPECT through the mission manager, the OCS and the fake judge, with
fake_perception on world_boxes.yaml as a process (the camera's health beat for PREFLIGHT, the face points for the map).
The survey and the completion stay in the OCS (ocs/state `internal`); the judge never gets them."""
from __future__ import annotations

import json
import time

import pytest

from optiprimex.sim import world as W

from .conftest import PY, ROOT, Probe, Proc, make_ocs_config, make_usv_config, wait_until

BOXES = str(ROOT / "profiles" / "bench" / "world_boxes.yaml")
SCAN = {"x_n": 30.0, "y_e": 0.0, "hdg_deg": 0.0}
NS = "team/optiprime/v1"


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    import paho.mqtt.client as mqtt
    tmp = tmp_path_factory.mktemp("box")
    lat, lon = W.WorldFrame({"lat": 1.2810202, "lon": 103.8550097}).to_latlon(SCAN["x_n"], SCAN["y_e"])
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(make_usv_config(tmp, brokers, fake_fcu)),
               "--trace", str(tmp / "mm_trace.jsonl")], tmp / "mm.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#", "rc/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15, include_past=True), mm.log()
    fake_fcu.ctrl({"teleport": {"lat": lat, "lon": lon, "hdg": 0.0}})     # after the mission manager took its origin at the start
    fp = Proc([PY, "-m", "optiprimex.sim.fake_perception", "--world", BOXES, "--port", str(brokers.boat_port), "--hub-port", str(brokers.hub_port),
               "--seed", "3"], tmp / "fp.log")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--start-delay", "1.0", "--log", str(tmp / "rc.jsonl")], tmp / "rc.log")
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(make_ocs_config(tmp, brokers))], tmp / "ocs.log")
    from optiprimex.tools.ocs_cli import OcsConsole
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 15, include_past=True), ocs.log()
    beat = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="box-ui-beat", protocol=mqtt.MQTTv311)
    beat.connect("127.0.0.1", brokers.boat_port)
    beat.loop_start()
    beat.publish("presence/usv_ui", json.dumps({"node": "usv_ui", "up": True, "pid": 1, "fcu_link": "fake", "t_unix": time.time()}),
                 qos=0, retain=True).wait_for_publish(2)
    yield {"mm": mm, "probe": probe, "con": con, "ocs": ocs, "trace": tmp / "mm_trace.jsonl"}
    beat.publish("presence/usv_ui", None, qos=0, retain=True).wait_for_publish(2)
    beat.loop_stop(); beat.disconnect()
    con.stop(); probe.stop()
    for p in (ocs, rc, fp, mm):
        p.kill()


def test_m0t4sim_live_survey_green_stop_internal_events(stack):
    con, p = stack["con"], stack["probe"]
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"].get("fcu_link") == "fake", 10)
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R3", 10)
    assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R5", 15)
    r = con.intent("START_TASK", "BOX_INSPECT", args={"sim": True, "scan_x_n": 30.0, "scan_y_e": 0.0, "scan_hdg_deg": 0.0}, wait_s=5)
    assert [a["body"]["result"] for a in r["acks"]][-1] == "DONE", [a["body"] for a in r["acks"]]
    assert p.wait(lambda m: m["topic"].endswith("event/usv/box_survey"), 30), stack["mm"].log()[-2500:]
    assert p.wait(lambda m: m["topic"].endswith("event/usv/box_mission_complete")
                  and m["env"]["body"]["target"]["colour"] == "green", 150, include_past=True), stack["mm"].log()[-2500:]
    done = p.last(f"{NS}/event/usv/box_mission_complete")["env"]["body"]
    assert 0.9 <= done["face_distance_m"] <= 1.25, done
    assert p.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD"
                  and m["env"]["body"].get("task_name") in (None, "BOX_INSPECT"), 30), stack["mm"].log()[-2000:]
    assert p.wait(lambda m: m["topic"] == "ocs/state" and "box_mission_complete" in (m["env"]["body"].get("internal") or {}).get("USV1", {}),
                  10, include_past=True), stack["ocs"].log()[-2000:]

    def stats():
        m = p.last("rc/sim/stats")
        return json.loads(m["payload"]) if m else {}
    assert wait_until(lambda: bool(stats()), 10)
    assert not {"box_survey", "box_mission_complete"} & {x["kind"] for x in stats().get("reports", [])}, stats().get("reports")

    # v0.1.27.1 (WP9's analyzer) on the same run: the mode sequence per stage and the frame cadence, pool rules (the fake arms)
    from optiprimex.tools import mode_log
    head, evs = mode_log.load_trace(str(stack["trace"]))
    r = mode_log.analyse(head, evs, "pool")
    assert r["n_fail"] == 0, mode_log.render_text(r)
    by = {s.sub: s for s in r["stages"] if s.task == "BOX_INSPECT"}
    assert by["OBSERVE"].modes_req == [] or by["OBSERVE"].modes_req[-1] == "CUSTOM_MANUAL", mode_log.render_text(r)
    assert by["STOP"].frames > 100 and by["STOP"].max_gap <= mode_log.GAP_FAIL_S and 8.0 <= by["STOP"].rate() <= 12.0
    assert "GUIDED" in by["ALIGN"].modes_req and r["mode_changes"] <= 8, mode_log.render_text(r)
