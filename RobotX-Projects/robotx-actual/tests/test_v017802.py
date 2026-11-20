"""v0.1.78.2: our simulator against the venue rc-test (Dmitrii 2026-11-09: "add a possibility to test our sim against their rc-test venue"):
`run_sim.sh --judge venue` / `run_bench.sh --judge venue` / the launcher's sim + venue - the course profile, the OCS on VENUE_RC_HOST:
VENUE_RC_PORT, the peek and the SIM page on that broker, the judge console read-only there; only the VENUE_RC_* keys are read from the network
files; refused before anything starts when they are missing."""
from __future__ import annotations

import os
import subprocess

from optiprimex.tools import launcher as L

from .conftest import ROOT

NET = {"VENUE_RC_HOST": "10.9.8.7", "VENUE_RC_PORT": "1999", "VENUE_RC_WEB": "http://10.9.8.7:8888/"}


def test_the_sim_plan():
    P = L.load_presets()
    cfg = dict(P["sim"], judge="venue")
    assert L.validate(cfg) == []
    p = L.build_plan(cfg, stamp="S", net_env=NET)
    sim = [s["cmd"] for s in p["steps"] if "run_sim.sh" in s["cmd"]]
    assert len(sim) == 1 and "--judge venue" in sim[0] and "RC_HOST" not in sim[0]
    ocs = [f for f in p["files"] if f["path"].endswith("/ocs.yaml")]
    assert not ocs or "course" in ocs[0]["text"].lower() or "protobuf" in ocs[0]["text"]          # the course profile's copy
    assert p["links"]["venue RoboCommand page"] == "http://10.9.8.7:8888/" and p["refuse"] == []
    assert any("SIMULATED boat is declared and reported to it" in n for n in p["notes"])


def test_the_sim_plan_refused_without_the_address(monkeypatch):
    monkeypatch.setattr(L, "read_net", lambda prof: {})
    p = L.build_plan(dict(L.load_presets()["sim"], judge="venue"), stamp="S", net_env={})
    assert any("VENUE_RC_HOST / VENUE_RC_PORT not set" in r for r in p["refuse"])


def _env(tmp_path, text):
    (tmp_path / "network.env").write_text(text)
    env = {k: v for k, v in os.environ.items() if not k.startswith("VENUE_RC_")}
    env.update(NETENV=str(tmp_path / "network.env"), NET_PROFILE="none", OPTIPRIMEX_SESSION="v017802-never")
    return env


def test_the_scripts_refuse_before_starting_anything(tmp_path):
    env = _env(tmp_path, "# nothing\n")
    for sh in (["scripts/run_sim.sh", "pool", "--no-open", "--judge", "venue"], ["scripts/run_bench.sh", "--judge", "venue"]):
        r = subprocess.run(["bash", str(ROOT / sh[0])] + sh[1:], env=env, capture_output=True, text=True, timeout=30, cwd=ROOT)
        assert r.returncode == 2 and "VENUE_RC_HOST / VENUE_RC_PORT not set" in r.stderr, (sh, r.stderr[-400:])
    r = subprocess.run(["tmux", "has-session", "-t", "v017802-never"], capture_output=True)
    assert r.returncode != 0                                                             # no session was created


def test_the_helper_reads_only_the_venue_keys(tmp_path):
    env = _env(tmp_path, "BOAT_HOST=1.2.3.4\nVENUE_RC_HOST=10.0.0.9 # the venue\nVENUE_RC_PORT=1910\n")
    r = subprocess.run(["bash", "-c", f'. "{ROOT}/scripts/_netenv.sh"; load_venue_rc "{ROOT}" && echo "$VENUE_RC_HOST:$VENUE_RC_PORT:${{BOAT_HOST:-unset}}"'],
                       env={k: v for k, v in env.items() if k != "BOAT_HOST"}, capture_output=True, text=True, timeout=10)
    assert r.stdout.strip() == "10.0.0.9:1910:unset", r.stdout + r.stderr


def test_the_wiring_in_the_scripts():
    sim = (ROOT / "scripts" / "run_sim.sh").read_text()
    assert 'PEEK="--judge-host $VENUE_RC_HOST --judge-port $VENUE_RC_PORT"' in sim and 'SIM_RC="--rc-host $VENUE_RC_HOST --rc-port $VENUE_RC_PORT"' in sim
    assert 'run_judge.sh --host "$VENUE_RC_HOST" --port "$VENUE_RC_PORT" --monitor --owner bench' in sim
    bench = (ROOT / "scripts" / "run_bench.sh").read_text()
    assert 'RCENV="RC_HOST=$VENUE_RC_HOST RC_PORT=$VENUE_RC_PORT "' in bench and "profiles/course/ocs_course.yaml" in bench.split("venue)")[1]
    assert "stop_judge.sh\" --owner bench" in (ROOT / "scripts" / "stop_bench.sh").read_text()
    for f in ("run_sim.sh", "run_bench.sh", "_netenv.sh"):
        assert "192.168." not in (ROOT / "scripts" / f).read_text().split("venue")[-1], f       # no address in the code
