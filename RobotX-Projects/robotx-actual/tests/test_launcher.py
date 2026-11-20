"""v0.1.38, the one-script launcher (scripts/launch.sh -> optiprimex/tools/launcher.py, the page on 8094): the presets, the plan per host
for each, the generated profile copies for changed rates, the typed network, the checks, the server (dry run) and the scripts' hooks."""
from __future__ import annotations

import json
import sys
import subprocess
import threading
import time
import urllib.error
import urllib.request

import pytest
import yaml

from optiprimex.tools import launcher as L

from .conftest import ROOT, free_port

NET = {"ORIN_SSH": "nx-alias", "AGX_SSH": "agx-alias"}


def _plan(name, **kw):
    cfg = dict(L.load_presets()[name], **kw)
    assert L.validate(cfg) == [], cfg
    return L.build_plan(cfg, stamp="S", net_env=NET)


def test_the_presets_are_complete_and_valid():
    P = L.load_presets()
    assert list(P) == ["sim", "lab", "event", "custom"]
    for name, c in P.items():
        assert set(L.FIELDS) | set(L.RATE_KEYS) | {"usv_ui", "log", "title", "about"} <= set(c), name
        assert L.validate(c) == [], name
    m = P["event"]
    assert (m["mode"], m["judge"], m["net"], m["fcu"], m["usv_profile"], m["cv"]) == ("real", "rctest", "lan", "uart", "course", "real")
    assert (P["sim"]["mode"], P["lab"]["usv_profile"]) == ("sim", "lab")


def test_the_sim_plan_is_this_mac_only():
    p = _plan("sim")
    assert [s["host"] for s in p["steps"]] == ["mac"] and p["files"] == [] and p["remote_files"] == []
    assert p["steps"][0]["cmd"] == "scripts/run_sim.sh pool --no-open --judge fake --perception camera"
    assert "stop_all" not in p["stop"] and "stop_bench.sh" in p["stop"] and "SIM page" in p["links"]
    p = _plan("sim", usv_ui=False, perception="geometric", world="marina", judge="rctest")
    assert p["steps"][0]["cmd"].endswith("marina --no-open --judge rctest --perception geometric --no-ui")


def test_the_real_plans_in_order():
    p = _plan("event")
    cmds = [s["cmd"] for s in p["steps"]]
    # v0.1.59 (A6): a clock check after each look-around
    assert [s["host"] for s in p["steps"]] == ["mac", "mac", "mac", "mac", "mac", "mac", "nx", "nx", "agx", "agx"]   # v0.1.63: + the USV window (nx); v0.1.83: + the recording observer
    assert "nx-alias" in cmds[0] and "date +%s" in cmds[1] and "agx-alias" in cmds[2] and "date +%s" in cmds[3] and "run_rctest.sh" in cmds[4]
    assert cmds[5] == "MAP=0 LOG=1 scripts/run_ocs.sh --lan --judge rctest"
    assert cmds[6] == "ssh -o ConnectTimeout=6 -o BatchMode=yes nx-alias 'cd ~/Desktop/dm3/MASTER_SCRIPTS && LOG=1 scripts/run_usv.sh --fcu uart --course --lan'"
    assert "run_boat_window.sh --lan" in cmds[7] and cmds[8].endswith("'cd ~/Desktop/dm3/MASTER_SCRIPTS && scripts/run_agx.sh --lan'")
    assert cmds[9].endswith("'cd ~/Desktop/dm3/MASTER_SCRIPTS && bash scripts/agx_observer.sh start --detector default'")   # v0.1.83: records by default
    assert any("props off" in n for n in p["notes"]) and "ORIN_SSH=nx-alias AGX_SSH=agx-alias scripts/stop_all.sh" == p["stop"]
    p = _plan("custom")
    assert "run_agx.sh --ts --sim --world profiles/bench/world_pool.yaml" in p["steps"][-1]["cmd"] and "--fcu fake --pool --ts" in p["steps"][-2]["cmd"]
    assert not any("props off" in n for n in p["notes"])
    p = _plan("lab", cv="off", judge="course")                       # v0.1.59 (A5): no course NIC measured -> refused, no RC_HOST
    assert [s["host"] for s in p["steps"]] == ["mac", "mac", "mac", "mac", "nx"] and "--judge course" in p["steps"][2]["cmd"]   # + the USV window (mac)
    assert any("course NIC" in r for r in p["refuse"]) and "RC_HOST=" not in p["steps"][2]["cmd"]
    assert any("bench guards" in n for n in p["notes"])


def test_changed_rates_become_profile_copies():
    p = _plan("lab", heartbeat_hz=1, status_hz=5, map_hz=4)
    (f,), (r,) = p["files"], p["remote_files"]
    assert f["path"] == "logs/launch/S/ocs.yaml" and "OCS_CONFIG_OVERRIDE=logs/launch/S/ocs.yaml" in p["steps"][4]["cmd"]
    assert r["host"] == "nx-alias" and "MM_CONFIG_OVERRIDE=logs/launch/usv_S.yaml" in p["steps"][6]["cmd"]   # v0.1.63: [5] = the USV window
    base = yaml.safe_load((ROOT / "profiles" / "pool" / "ocs_pool.yaml").read_text())
    got = yaml.safe_load(f["text"])
    assert got["ocs"]["heartbeat_hz"] == 1 and f["text"].startswith("# GENERATED")
    got["ocs"]["heartbeat_hz"] = base["ocs"]["heartbeat_hz"]
    assert got == base                                                                     # only the rate differs
    base = yaml.safe_load((ROOT / "profiles" / "pool" / "usv_lab.yaml").read_text())
    got = yaml.safe_load(r["text"])
    assert (got["rates"]["status_hz"], got["rates"]["map_hz"]) == (5, 4)
    got["rates"].update(status_hz=base["rates"]["status_hz"], map_hz=base["rates"]["map_hz"])
    assert got == base
    p = _plan("sim", status_hz=4)                                                   # sim + usv_ui: the share's profile
    (f,) = p["files"]
    assert "endpoint: udpin:127.0.0.1:14552" in f["text"] and p["steps"][0]["cmd"].startswith("USV_CONFIG=logs/launch/S/usv.yaml scripts/run_sim.sh")
    p = _plan("sim", heartbeat_hz=1, judge="rctest")
    assert "ocs_course.yaml" in p["files"][0]["text"].splitlines()[0] and p["steps"][0]["cmd"].startswith("OCS_CONFIG=")


def test_the_typed_network_and_the_checks():
    c = dict(L.load_presets()["lab"], net="manual", ocs_host="10.0.0.2", boat_host="10.0.0.3", agx_host="10.0.0.4",
             nx_ssh="me@10.0.0.3", agx_ssh="me@10.0.0.4")
    assert L.validate(c) == []
    p = L.build_plan(c, stamp="S", net_env={})
    cmds = [s["cmd"] for s in p["steps"]]
    assert "OCS_HOST=10.0.0.2 BOAT_HOST=10.0.0.3 AGX_HOST=10.0.0.4 scripts/run_ocs.sh --judge fake" in cmds[4]
    assert "me@10.0.0.3 'cd ~/Desktop/dm3/MASTER_SCRIPTS && OCS_HOST=10.0.0.2 LOG=1 scripts/run_usv.sh --fcu uart --lab'" in cmds[6]
    assert "BOAT_HOST=10.0.0.3 scripts/run_agx.sh'" in cmds[7] and "--ts" not in " ".join(cmds)   # v0.1.63: cmds[5] = the USV window
    bad = dict(c, boat_host="", agx_ssh="x; rm -rf /", heartbeat_hz=50, fcu="serial")
    errs = " | ".join(L.validate(bad))
    for needle in ("boat_host needed", "agx_ssh: letters", "heartbeat_hz: 0.5 .. 5.0", "fcu: one of"):
        assert needle in errs, needle
    assert any("course needs the real network" in e for e in L.validate(dict(L.load_presets()["sim"], judge="course")))
    assert not any("rc_host" in e for e in L.validate(dict(c, judge="course")))    # v0.1.59 (A5): empty = auto (the NIC's gateway)


def _get(url):
    with urllib.request.urlopen(url, timeout=5) as r:
        return r.read()


def _post(url, body, header=True):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers=dict({"Content-Type": "application/json"}, **({"X-Launch": "1"} if header else {})))
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return {"code": e.code}


def test_the_server_in_a_dry_run():
    port = free_port()
    runner = L.Runner(dry=True)
    srv = L.Server(("127.0.0.1", port), L.make_handler(runner))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    u = f"http://127.0.0.1:{port}"
    try:
        page = _get(u + "/").decode()
        for needle in ('id="presets"', 'id="launch"', 'id="stop"', 'data-k="heartbeat_hz"', 'data-k="nx_ssh"', '"X-Launch":"1"'):
            assert needle in page, needle
        d = json.loads(_get(u + "/presets.json"))
        assert set(d["presets"]) == {"sim", "lab", "event", "custom"} and d["fields"]["net"] == ["ts", "lan", "manual"] and d["version"]
        cfg = dict(d["presets"]["lab"])
        assert _post(u + "/plan", {"cfg": cfg}, header=False) == {"code": 403}
        r = _post(u + "/plan", {"cfg": dict(cfg, fcu="nope")})
        assert not r["ok"] and r["plan"] is None
        r = _post(u + "/launch", {"cfg": cfg})
        assert r["ok"]
        for _ in range(50):
            s = json.loads(_get(u + "/status.json"))
            if s["state"] != "running":
                break
            time.sleep(0.05)
        texts = [x["text"] for x in s["log"]]
        assert s["state"] == "done" and s["dry"] and "dry run" in texts[-1] and any("run_usv.sh --fcu uart --lab" in t for t in texts)
        assert not _post(u + "/stop", {"confirm": "no"})["ok"]
        assert _post(u + "/stop", {"confirm": "stop"})["ok"]
        time.sleep(0.2)
        s = json.loads(_get(u + "/status.json"))
        assert any("stop_all.sh" in x["text"] for x in s["log"])
    finally:
        srv.shutdown()
        srv.server_close()


def test_the_scripts_take_the_launchers_files():
    h = subprocess.run(["bash", str(ROOT / "scripts" / "launch.sh"), "--help"], capture_output=True, text=True, timeout=10).stdout
    assert "http://127.0.0.1:8094/" in h and "--dry-run" in h and "never arms" in h
    r = subprocess.run(["bash", str(ROOT / "scripts" / "launch.sh"), "--bogus"], capture_output=True, text=True, timeout=10)
    assert r.returncode == 2
    assert 'MM_CONFIG="${MM_CONFIG_OVERRIDE:-$MM_CONFIG}"' in (ROOT / "scripts" / "run_usv.sh").read_text()
    assert 'OCS_CONFIG="${OCS_CONFIG_OVERRIDE:-$OCS_CONFIG}"' in (ROOT / "scripts" / "run_ocs.sh").read_text()
    assert 'USV_CONFIG="${USV_CONFIG:-usv_ui/config/usv_bench_share.yaml}"' in (ROOT / "scripts" / "run_sim.sh").read_text()


def test_the_script_starts_under_macos_bash():
    """v0.1.39.1: `scripts/launch.sh` with no flags died under /bin/bash 3.2 + set -u ("ARGS[@]: unbound variable")."""
    import os
    import signal
    port = free_port()
    for args in ([], ["--dry-run"]):
        p = subprocess.Popen(["/bin/bash", str(ROOT / "scripts" / "launch.sh"), "--no-open", *args], cwd=str(ROOT),
                             env=dict(os.environ, LAUNCH_PORT=str(port)), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        try:
            line = p.stdout.readline()
            assert f"http://127.0.0.1:{port}/" in line and (("dry run" in line) == bool(args)), line
            assert b'id="launch"' in urllib.request.urlopen(f"http://127.0.0.1:{port}/", timeout=5).read()
        finally:
            p.send_signal(signal.SIGTERM)
            p.wait(10)


# ---- v0.1.39.2: reconnect, the guard, Ctrl-C ----------------------------------------------------------------------------------
def test_the_plans_know_their_sessions_per_host():
    p = _plan("sim", judge="rctest")
    assert p["ops"] == {"mac": {"ssh": None, "expect": ["optiprimex", "rctest"], "stop": p["stop"], "steps": [0]}}
    p = _plan("event")
    assert set(p["ops"]) == {"mac", "nx", "agx"} and p["ops"]["nx"]["expect"] == ["usv"] and p["ops"]["agx"]["expect"] == ["agx"]
    assert [p["steps"][i]["host"] for i in p["ops"]["mac"]["steps"]] == ["mac", "mac"]           # rc-test + the OCS, not the look-arounds
    assert "nx-alias" in p["ops"]["nx"]["stop"] and "stop_usv.sh" in p["ops"]["nx"]["stop"] and p["ops"]["nx"]["steps"] == [6, 7]
    assert "agx" not in _plan("lab", cv="off")["ops"]


def test_probe_and_the_summary():
    rec = {"when": "22:51", "title": "marina", "plan": _plan("event")}
    seen = {None: ["ocs", "judge", "other"], "nx-alias": ["usv"], "agx-alias": None}
    pr = L.probe(rec, ls=lambda h: seen[h])
    assert pr["up"] and pr["hosts"]["mac"]["missing"] == ["rctest"] and pr["hosts"]["nx"]["missing"] == [] and not pr["hosts"]["agx"]["reach"]
    txt = L.summary(rec, pr)
    assert "the launch from 22:51 (marina) is up" in txt and "MISSING rctest" in txt and "AGX (agx-alias): does not answer" in txt
    assert "stop_all.sh" in txt and "RECONNECT" in txt
    assert not L.probe(rec, ls=lambda h: [])["up"]
    pr = L.probe(None, ls=lambda h: ["optiprimex", "mine"])
    assert pr["up"] and pr["foreign"] == ["optiprimex"] and "without a launch record" in L.summary(None, pr)
    assert L.summary(None, L.probe(None, ls=lambda h: [])) == "nothing of ours is running"


def test_reconnect_restart_and_the_guard(tmp_path):
    rec_path = str(tmp_path / "last.json")
    port = free_port()
    runner = L.Runner(dry=True)
    runner.record_path = rec_path
    up = {"v": []}
    watch = L.Watch(runner, ls=lambda h: up["v"])
    srv = L.Server(("127.0.0.1", port), L.make_handler(runner, watch))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    u = f"http://127.0.0.1:{port}"
    try:
        assert not _post(u + "/adopt", {})["ok"]                                             # no record
        cfg = dict(L.load_presets()["event"], preset="event")
        L.save_record(cfg, L.build_plan(cfg, stamp="S", net_env=NET), rec_path)
        up["v"] = ["ocs", "usv"]                                                            # the agx session is gone
        s = json.loads(_get(u + "/session.json"))
        watch.refresh()
        s = json.loads(_get(u + "/session.json"))
        assert s["probe"]["up"] and s["record"]["title"].startswith("event") and not s["adopted"] and s["probe"]["hosts"]["agx"]["missing"] == ["agx"]
        runner.dry = False                                                                  # the guard is for real launches
        r = _post(u + "/launch", {"cfg": cfg})
        assert not r["ok"] and "still running: RECONNECT or STOP first" in r["why"]
        runner.dry = True
        r = _post(u + "/adopt", {})
        assert r["ok"] and r["cfg"]["usv_profile"] == "course"
        st = json.loads(_get(u + "/status.json"))
        assert st["state"] == "adopted" and "OCS page" in st["links"] and set(st["hosts"]) == {"mac", "nx", "agx"}
        assert json.loads(_get(u + "/session.json"))["adopted"]
        assert _post(u + "/restart", {"host": "agx"})["ok"]
        for _ in range(50):
            if json.loads(_get(u + "/status.json"))["state"] != "running":
                break
            time.sleep(0.05)
        texts = [x["text"] for x in json.loads(_get(u + "/status.json"))["log"]]
        assert any("stop_agx.sh" in t for t in texts) and any("run_agx.sh --lan" in t for t in texts) and not any("run_usv.sh" in t for t in texts)
        assert not _post(u + "/restart", {"host": "moon"})["ok"]
        assert _post(u + "/stop", {"confirm": "stop"})["ok"]
        time.sleep(0.2)
        assert any("stop_all.sh" in x["text"] for x in json.loads(_get(u + "/status.json"))["log"])
    finally:
        srv.shutdown()
        srv.server_close()


def test_ctrl_c_mid_launch_skips_the_rest_and_steps_get_their_own_group():
    runner = L.Runner(dry=True)
    plan = _plan("event")
    runner.state = "running"
    runner.abort = True
    runner._run(plan)
    texts = [x["text"] for x in runner.log]
    assert runner.state == "failed" and "Ctrl-C: steps 1..10 skipped" in texts[-2]
    src = (ROOT / "optiprimex" / "tools" / "launcher.py").read_text()
    assert "start_new_session=True" in src


def test_ctrl_c_when_idle_says_what_is_up(tmp_path):
    import os
    import signal
    port = free_port()
    p = subprocess.Popen([sys.executable, "-m", "optiprimex.tools.launcher", "--port", str(port), "--dry-run"], cwd=str(ROOT),
                         stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        assert "http://127.0.0.1:%d/" % port in p.stdout.readline()
        p.send_signal(signal.SIGINT)
        out = p.communicate(timeout=20)[0]
        assert "page closed." in out and ("nothing of ours is running" in out or "is up" in out or "without a launch record" in out), out
        assert p.returncode == 0
    finally:
        if p.poll() is None:
            p.kill()
    st = subprocess.run([sys.executable, "-m", "optiprimex.tools.launcher", "--status"], cwd=str(ROOT), capture_output=True, text=True, timeout=30)
    assert st.returncode == 0 and st.stdout.strip()
