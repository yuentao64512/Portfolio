"""v0.1.78.5: the first real launch with judge venue stopped at step 5 (Dmitrii 2026-11-09 12:45, Team Village): `run_ocs.sh: line 109:
RC_PORT: unbound variable` - the `${VENUE_RC_HOST:?...}` word carried an apostrophe ("rc-test's"), which bash takes as an opening quote, so
the RC_PORT line was swallowed; and a false `OCS_HOST ... will not find the hub` warning (the Mac on the switch through a USB LAN, en16, not
en0). Each judge's branch of run_ocs.sh is now executed here with set -u, and the address check looks at every interface."""
from __future__ import annotations

import re
import subprocess

import pytest

from .conftest import ROOT

SRC = (ROOT / "scripts" / "run_ocs.sh").read_text()


def _branch(judge: str) -> str:
    body = SRC[SRC.index('case "$JUDGE" in       # v0.1.26'):]
    body = body[:body.index("\nesac") + 5]
    return body


@pytest.mark.parametrize("judge,env,want", [
    ("venue", {"VENUE_RC_HOST": "10.7.0.4", "VENUE_RC_PORT": "1910"}, "10.7.0.4:1910"),
    ("course", {"RC_HOST": "10.6.5.2"}, "10.6.5.2:1883"),
    ("rctest", {}, "127.0.0.1:1885"),
])
def test_each_judge_branch_runs_under_set_u(judge, env, want):
    script = ("set -euo pipefail\nJUDGE=" + judge + "\nKIT=/nonexistent; SESSION=x; BENCH=/tmp; PY=python3; HUB_PORT=1883; RUN_DIR=/tmp; RC_ARGS=''\n"
              "OCS_CONFIG=x; RCENV=''; JUDGE_ARGS=''\n"
              "tmux() { :; }\n" + _branch(judge) + '\necho "OUT $RC_HOST:$RC_PORT|$RCENV|$JUDGE_ARGS"\n')
    r = subprocess.run(["bash", "-c", script], env={"PATH": "/usr/bin:/bin", **env}, capture_output=True, text=True, timeout=10)
    assert r.returncode == 0, r.stderr
    out = [l for l in r.stdout.splitlines() if l.startswith("OUT ")][0]
    assert out.startswith("OUT " + want), out
    if judge in ("venue", "course"):
        assert f"RC_HOST={want.split(':')[0]} RC_PORT={want.split(':')[1]}" in out and "--monitor" in out


def test_no_apostrophe_in_a_parameter_word():
    for f in ("run_ocs.sh", "run_sim.sh", "run_bench.sh", "run_usv.sh", "run_agx.sh", "_netenv.sh"):
        src = (ROOT / "scripts" / f).read_text()
        for m in re.finditer(r'\$\{[A-Za-z_]+:\?[^}]*\}', src):                      # a :? message word (a $(...) inside is code, not a word)
            assert "'" not in m.group(0) or "$(" in m.group(0), (f, m.group(0))


def test_the_address_check_looks_at_every_interface():
    assert '{ ifconfig 2>/dev/null || ip -4 addr 2>/dev/null; } | grep -qE "inet ${OCS_HOST//./\\\\.}[ /]"' in SRC
