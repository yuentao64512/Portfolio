#!/usr/bin/env python3
"""Convert a crab/logger JSONL file into a Mermaid state diagram."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path,
                        help="fcu_crab_state.jsonl or fcu_state.jsonl")
    parser.add_argument("--output", type=Path,
                        help="write Mermaid text to this file instead of stdout")
    args = parser.parse_args()

    states: list[str] = []
    transitions: list[tuple[str, str, str]] = []
    previous = None
    for line in args.input.read_text(encoding="utf-8").splitlines():
        if not line.strip():
            continue
        item = json.loads(line)
        state = item.get("state")
        if not state:
            continue
        if state not in states:
            states.append(state)
        if previous is not None and state != previous:
            reason = item.get("reason", "")
            transitions.append((previous, state, reason))
        previous = state

    lines = ["stateDiagram-v2"]
    if states:
        lines.append(f"    [*] --> {states[0]}")
    for source, target, reason in transitions:
        label = f" : {reason}" if reason else ""
        lines.append(f"    {source} --> {target}{label}")
    if states:
        lines.append(f"    {states[-1]} --> [*]")
    diagram = "\n".join(lines) + "\n"

    if args.output:
        args.output.write_text(diagram, encoding="utf-8")
        print(f"Wrote {args.output}")
    else:
        print(diagram, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
