"""ADR-8 stall path: a refused (silent) declaration never yields RunStart; R4 raises the operator alert after
stall_s and the only re-arm is a new declaration (declaration_seq + 1)."""
from __future__ import annotations

import pytest

from optiprimex.tools.ocs_cli import OcsConsole

from .conftest import PY, Probe, Proc, make_ocs_config, make_usv_config, wait_until

NS = "team/optiprime/v1"


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("stall")
    usv_cfg = make_usv_config(tmp, brokers, fake_fcu)
    ocs_cfg = make_ocs_config(tmp, brokers, stall_s=3.0)
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(usv_cfg)], tmp / "mm.log")
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(brokers.hub_port), "--refuse"], tmp / "rc.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#", "rc/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 15, include_past=True), mm.log()
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(ocs_cfg)], tmp / "ocs.log")
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    assert probe.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R2", 15, include_past=True), ocs.log()
    yield {"probe": probe, "con": con}
    con.stop()
    probe.stop()
    for p in (ocs, rc, mm):
        p.kill()


def test_stall_alert_then_redeclare(stack):
    con, p = stack["con"], stack["probe"]
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert con.intent("ENABLE_AUTO", wait_s=5)["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["r"] == "R4", 10)
    m = p.wait(lambda m: m["topic"] == "ocs/alert" and m["env"]["body"]["kind"] == "run_start_stall", 10)
    assert m and m["env"]["body"]["declaration_seq"] == 1
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["stalled"] is True and m["env"]["body"]["r"] == "R4", 5)
    r = con.intent("START_TASK", "T4", wait_s=5)
    assert r["result"]["result"] == "accepted"                                              # bench mode: OCS guard relaxed...
    assert [a["body"]["result"] for a in r["acks"]] == ["REJECTED"] and "START_RUN" in r["acks"][0]["body"]["reason"]   # ...the vehicle still refuses
    assert con.intent("DECLARE")["result"]["result"] == "accepted"
    assert p.wait(lambda m: m["topic"] == "ocs/state" and m["env"]["body"]["declaration_seq"] == 2 and not m["env"]["body"]["stalled"], 5)
