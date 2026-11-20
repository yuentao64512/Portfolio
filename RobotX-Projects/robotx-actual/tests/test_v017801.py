"""v0.1.78.1: the venue rc-test as a judge of its own (Dmitrii 2026-11-09, Team Village: "our team broker on 1910, the page on 8010 - a venue
rc-test vs the home rc-test"): `--judge venue` = RoboNation's practice server on our team broker, VENUE_RC_HOST / VENUE_RC_PORT /
VENUE_RC_WEB from network.env (never in the code), the course profile (protobuf), the console read-only; real mode only; never with FREE RUN."""
from __future__ import annotations

import os
import subprocess

from optiprimex.tools import launcher as L

from .conftest import ROOT

NET = {"ORIN_SSH": "nx-alias", "AGX_SSH": "agx-alias", "VENUE_RC_HOST": "10.9.8.7", "VENUE_RC_PORT": "1999", "VENUE_RC_WEB": "http://10.9.8.7:8888/"}


def test_the_field_and_the_rules():
    assert L.FIELDS["judge"] == ("venue", "auto", "alpha", "bravo", "charlie", "delta", "course", "fake", "rctest")   # v0.1.94: + auto   # v0.1.86: venue first, rctest last   # v0.1.82 (WP39): + the courses
    P = L.load_presets()
    assert not any("judge" in e for e in L.validate(dict(P["sim"], judge="venue")))     # v0.1.78.2: the simulator too (test_v017802)
    assert any("course" in e for e in L.validate(dict(P["sim"], judge="course")))
    assert not any("judge" in e for e in L.validate(dict(P["event"], judge="venue")))


def test_the_plan_uses_network_env(monkeypatch):
    P = L.load_presets()
    p = L.build_plan(dict(P["event"], judge="venue"), stamp="S", net_env=NET)
    ocs = [s["cmd"] for s in p["steps"] if "run_ocs.sh" in s["cmd"]]
    assert len(ocs) == 1 and "--judge venue" in ocs[0], ocs
    assert "RC_HOST=" not in ocs[0]                                                     # run_ocs.sh reads VENUE_RC_* itself
    assert p["links"].get("venue RoboCommand page") == "http://10.9.8.7:8888/"
    assert any("venue rc-test: team broker 10.9.8.7:1999" in n for n in p["notes"])
    assert not p.get("refuse")
    monkeypatch.setattr(L, "read_net", lambda prof: {})
    p = L.build_plan(dict(P["event"], judge="venue"), stamp="S", net_env={"ORIN_SSH": "nx-alias"})
    assert any("VENUE_RC_HOST / VENUE_RC_PORT not set" in r for r in p["refuse"])


def test_free_run_never_with_the_venue_judge():
    P = L.load_presets()
    p = L.build_plan(dict(P["lab"], judge="venue", free_run="on"), stamp="S", net_env=NET)
    assert any("FREE RUN with judge venue: refused" in r for r in p["refuse"])


def test_run_ocs_refuses_before_starting_anything(tmp_path):
    env = {k: v for k, v in os.environ.items() if not k.startswith("VENUE_RC_")}
    env.update(NETENV=str(tmp_path / "network.env"), NET_PROFILE="none")                 # an empty network: no VENUE_RC_*
    (tmp_path / "network.env").write_text("# empty\n")
    r = subprocess.run(["bash", str(ROOT / "scripts" / "run_ocs.sh"), "--judge", "venue"], env=env, capture_output=True, text=True, timeout=20)
    assert r.returncode == 2 and "VENUE_RC_HOST / VENUE_RC_PORT not set" in r.stderr, r.stderr
    sh = (ROOT / "scripts" / "run_ocs.sh").read_text()
    assert 'RC_HOST="${VENUE_RC_HOST:?' in sh and "--monitor" in sh.split("venue)")[1].split(";;")[0]
    assert "192.168." not in sh.split("venue)")[1].split(";;")[0]                         # no address in the code
