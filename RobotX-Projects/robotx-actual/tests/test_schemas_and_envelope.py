import json
import time

import pytest

from optiprimex import schemas
from optiprimex.companion.config import Config
from optiprimex.companion.mqtt import Dedup, Envelope, Seq


def test_envelope_round_trip_validates():
    env = Envelope.wrap("cmd", {"action": "HOLD"}, "USV1", 1, run_id=1001, declaration_seq=1, source="test")
    schemas.validate(env)
    parsed = Envelope.parse(json.dumps(env).encode())
    assert parsed["body"]["action"] == "HOLD" and parsed["run_id"] == 1001


@pytest.mark.parametrize("bad", [
    {"action": "ARM"},                       # never a command
    {"action": "START_TASK", "task": "T9"},  # unknown task
    {"task": "T4"},                          # no action
    {"action": "HOLD", "extra": 1},          # additionalProperties
])
def test_command_bodies_rejected(bad):
    env = Envelope.wrap("cmd", bad, "USV1", 1)
    with pytest.raises(schemas.ContractError):
        schemas.validate(env)


def test_ack_and_status_bodies():
    schemas.validate(Envelope.wrap("ack", {"ack_of": "x" * 8, "result": "DONE", "reason": "", "state": "AUTO_HOLD"}, "USV1", 2))
    schemas.validate(Envelope.wrap("status", {"state": "AUTO_TASK", "current_task": "TASK_DYNAMIC_INCIDENT", "kill": False, "estop": False}, "USV1", 3))
    with pytest.raises(schemas.ContractError):
        schemas.validate(Envelope.wrap("status", {"state": "FLYING", "current_task": "TASK_NONE"}, "USV1", 4))
    with pytest.raises(schemas.ContractError):
        schemas.validate(Envelope.wrap("ack", {"ack_of": "abcdefgh", "result": "MAYBE"}, "USV1", 5))


def test_envelope_fields_are_strict():
    env = Envelope.wrap("event", {"event": "task_state"}, "USV1", 1)
    env["unexpected"] = 1
    with pytest.raises(schemas.ContractError):
        schemas.validate(env)
    env.pop("unexpected")
    env["kind"] = "banana"
    with pytest.raises(schemas.ContractError):
        schemas.validate(env)


def test_dedup_and_seq():
    d = Dedup(ttl_s=0.2)
    seen, _ = d.seen("m1")
    assert not seen
    d.remember("m1", ("ACCEPTED", ""))
    assert d.seen("m1") == (True, ("ACCEPTED", ""))
    time.sleep(0.3)
    assert d.seen("m1")[0] is False      # evicted after ttl
    s = Seq()
    assert [s.next(), s.next(), s.next()] == [1, 2, 3]


def test_config_helpers():
    cfg = Config()
    assert cfg.team("cmd", "usv", "run") == "team/optiprime/v1/cmd/usv/run"
    assert cfg.local("mm", "state") == "local/usv/mm/state"
    assert "GOTO" in schemas.COMMAND_ACTIONS and "AUTO_SAFE_HOLD" in schemas.V_STATES
