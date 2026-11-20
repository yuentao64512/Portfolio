"""JSON-schema contracts for every team message. The same files feed the OCS, the vehicles,
the tests and (later) the mission GUI, so a contract change is one edit."""
from __future__ import annotations

import json
from functools import lru_cache
from pathlib import Path
from typing import Any

import jsonschema

_HERE = Path(__file__).parent


@lru_cache(maxsize=None)
def _load(name: str) -> dict:
    with open(_HERE / name, "r", encoding="utf-8") as fh:
        return json.load(fh)


@lru_cache(maxsize=None)
def _validator(kind: str) -> jsonschema.Draft202012Validator:
    bodies = _load("bodies.schema.json")
    if kind == "envelope":
        schema = _load("envelope.schema.json")
    else:
        if kind not in bodies["$defs"]:
            raise KeyError(f"no body schema for kind {kind!r}")
        schema = dict(bodies["$defs"][kind])
        schema["$defs"] = bodies["$defs"]
    return jsonschema.Draft202012Validator(schema)


class ContractError(ValueError):
    """A message does not satisfy its schema."""


def validate_envelope(env: dict) -> None:
    errs = sorted(_validator("envelope").iter_errors(env), key=lambda e: e.path)
    if errs:
        raise ContractError("envelope: " + "; ".join(e.message for e in errs))


def validate_body(kind: str, body: Any) -> None:
    if kind in ("telemetry", "local"):
        if not isinstance(body, dict):
            raise ContractError(f"{kind}: body must be an object")
        return
    errs = sorted(_validator(kind).iter_errors(body), key=lambda e: e.path)
    if errs:
        raise ContractError(f"{kind} body: " + "; ".join(e.message for e in errs))


def validate(env: dict) -> None:
    """Validate an envelope and its body against the kind it declares."""
    validate_envelope(env)
    validate_body(env["kind"], env["body"])


COMMAND_ACTIONS = tuple(_load("bodies.schema.json")["$defs"]["cmd"]["properties"]["action"]["enum"])
V_STATES = tuple(_load("bodies.schema.json")["$defs"]["status"]["properties"]["state"]["enum"])
