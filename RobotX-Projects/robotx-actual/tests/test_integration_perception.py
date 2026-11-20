"""Fake perception -> mission manager PerceptionMap -> telemetry/usv/map digest on the hub; presence beats on both brokers."""
from __future__ import annotations

import math

import pytest
import yaml

from optiprimex.tools.vcmd import VehicleCommander

from .conftest import PY, ROOT, Probe, Proc, make_usv_config, wait_until

NS = "team/optiprime/v1"


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("perc")
    cfg = make_usv_config(tmp, brokers, fake_fcu)
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(cfg)], tmp / "mm.log")
    fp = Proc([PY, "-m", "optiprimex.sim.fake_perception", "--world", str(ROOT / "profiles" / "bench" / "world_bench.yaml"), "--port", str(brokers.boat_port),
               "--hub-port", str(brokers.hub_port), "--hz", "5"], tmp / "fp.log")
    hub = Probe(brokers.hub_port, [f"{NS}/#", "presence/#", "sim/truth"]).start()
    boat = Probe(brokers.boat_port, ["presence/#", "local/#"]).start()
    vc = VehicleCommander(port=brokers.hub_port)
    vc.start()
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15, include_past=True), mm.log()
    yield {"mm": mm, "fp": fp, "hub": hub, "boat": boat, "vc": vc, "tmp": tmp}
    vc.stop(); hub.stop(); boat.stop(); fp.kill(); mm.kill()


def world():
    return yaml.safe_load((ROOT / "profiles" / "bench" / "world_bench.yaml").read_text())["objects"]


def test_map_builds_from_fake_detections_while_driving(stack):
    hub, vc = stack["hub"], stack["vc"]
    vc.declaration_seq = 1
    assert [a["body"]["result"] for a in vc.send("run", "PREPARE", args={"declaration_seq": 1, "tiers": {"T4": "TIER_CORE"}})] == ["ACCEPTED", "DONE"]
    assert [a["body"]["result"] for a in vc.send("run", "ENABLE_AUTO")][-1] == "DONE"
    assert [a["body"]["result"] for a in vc.send("run", "START_RUN", args={"run_id": 1001})][-1] == "DONE"
    vc.run_id = 1001
    assert [a["body"]["result"] for a in vc.send("run", "START_TASK", task="GOTO", args={"x_n": 30.0, "y_e": 0.0})][-1] == "DONE"
    # the boat drives north through the channel; the map should hold every object within 0.6 m
    truth = {o["id"]: o for o in world()}

    def mapped_well() -> bool:
        m = hub.last(f"{NS}/telemetry/usv/map")
        if not m:
            return False
        ents = m["env"]["body"].get("entities") or []
        if len(ents) < len(truth):
            return False
        for o in truth.values():
            if not any(e["kind"] == o["kind"] and e["colour"] == o["colour"] and math.hypot(e["x_n"] - o["x_n"], e["y_e"] - o["y_e"]) < 0.6 for e in ents):
                return False
        return True

    assert wait_until(mapped_well, 60), hub.last(f"{NS}/telemetry/usv/map")["env"]["body"].get("entities")
    body = hub.last(f"{NS}/telemetry/usv/map")["env"]["body"]
    assert len(body["gates"]) == 3 and all(1.5 <= g["width_m"] <= 6.0 for g in body["gates"])
    d = body["dock"]
    assert d and abs(d["x_n"] - 50.0) < 0.5 and abs(d["y_e"] - 10.0) < 0.5 and len(d["faces"]) == 3
    assert body["ingest_age_s"] is not None and body["ingest_age_s"] < 2.0
    assert hub.last("sim/truth") is not None                                # the truth layer for the GUI


def test_presence_beats_on_both_brokers(stack):
    hub, boat = stack["hub"], stack["boat"]
    assert boat.wait(lambda m: m["topic"] == "presence/mission_manager" and m["env"]["up"] is True, 6, include_past=True)
    assert boat.wait(lambda m: m["topic"] == "presence/fake_perception" and m["env"]["up"] is True, 6, include_past=True)
    stack["fp"].kill()                                                       # a clean stop (SIGTERM) publishes up=false
    # kill() blocks until the process is gone, so the final beat may already be in the probe: include_past
    assert boat.wait(lambda m: m["topic"] == "presence/fake_perception" and m["env"]["up"] is False, 6, include_past=True)
