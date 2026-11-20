"""v0.1.84.5 (Dmitrii via session A: "is there an explicit button to start logging? ... so we engage with synced clock"): the clocks
from the presence beats, POST /logging {start|stop|status} = the clocks checked, then usv_ui's FCU log + the AGX recording + the Mac
frames under one label and t0, a session file; parts that already ran are kept and left on at stop."""
from __future__ import annotations

import json
import time

from .conftest import free_port
from .test_v018302 import FakeAgx, FakeRec


def _web(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1", "OCS_LOG_SESSIONS": str(tmp_path / "sessions")},
                 feeds=False)
    web.camtab.ssh_run, web.camtab.rec = FakeAgx(), FakeRec()
    web.camtab.DEPTH_WAIT_S = 0.3
    return web


def _beats(web, host, offsets, now):
    """presence beats from host whose clock is ahead of the Mac by offsets (s), each received with a little latency."""
    for i, off in enumerate(offsets):
        lat = 0.01 + 0.02 * (i % 3)
        body = {"node": "usv_ui", "host": host, "up": True, "t_unix": now - lat + off}
        web.m.on_sys_or_presence("boat", f"presence/n{i}", json.dumps(body).encode())


def test_the_clocks_from_the_beats(tmp_path):
    web = _web(tmp_path)
    now = time.time()
    _beats(web, "usv-pixhawk", [0.83] * 5, now)
    _beats(web, "optiprimeagxorin-desktop", [0.02] * 5, now)
    c = web.clocks_state()
    assert c["nx"]["host"] == "usv-pixhawk" and abs(c["nx"]["offset_s"] - 0.82) < 0.05 and c["nx"]["ok"] is False and c["nx"]["n"] == 5
    assert c["agx"]["ok"] is True and abs(c["agx"]["offset_s"]) < 0.05
    assert "NX +0.8" in web.clock_refusal(c) and "AGX" not in web.clock_refusal(c)
    assert "AGX unknown" in web.clock_refusal({"nx": c["nx"]})
    assert "clocks" in web.state() and web.state()["logging"]["session"] == {"running": False, "auto": False}   # v0.1.86: + auto


def test_start_refuses_on_the_clock_then_starts_the_parts(tmp_path):
    web = _web(tmp_path)
    now = time.time()
    _beats(web, "usv-pixhawk", [0.5] * 3, now)
    sent = []
    web.boat_word = lambda word, args, wait_s=5.0: sent.append((word, args)) or {"ok": True, "text": "logging ON -> pool_x_" + "_".join(args[1:])}
    r = web.logging_cmd({"op": "start", "label": "dry1"})
    assert r["ok"] is False and r["needs"] == "clock" and "NX +0.49 s" in r["why"] and "AGX unknown" in r["why"] and not sent
    r = web.logging_cmd({"op": "start", "label": "dry1", "confirm": "clock"})
    assert r["ok"] and r["label"].startswith("dry1_") and sent[0][0] == "log" and sent[0][1][0] == "on"
    assert r["parts"]["fcu"]["started"] and r["parts"]["agx"]["ok"] and r["parts"]["mac"]["ok"]
    assert web.state()["logging"]["session"]["running"] is True
    f = tmp_path / "sessions" / f"{r['label']}.json"
    assert json.loads(f.read_text())["clock_override"] is True
    r2 = web.logging_cmd({"op": "start"})
    assert r2["already"] is True
    r3 = web.logging_cmd({"op": "stop"})
    assert r3["needs"] == "stop"
    r3 = web.logging_cmd({"op": "stop", "confirm": "stop"})
    assert r3["ok"] and ("log", ["off"]) in sent and web.log_session["running"] is False
    assert json.loads(f.read_text())["running"] is False


def test_parts_that_already_run_are_kept(tmp_path):
    web = _web(tmp_path)
    now = time.time()
    _beats(web, "usv-pixhawk", [0.01] * 3, now)
    _beats(web, "optiprimeagxorin-desktop", [0.01] * 3, now)
    with web.m.lock:                                                      # usv_ui logs by default (0.1.83)
        web.m.presence[("boat", "usv_ui")] = ({"up": True, "log": "pool_20261111T150000Z"}, now)
    agx = FakeAgx(up=True, rec=True)                                      # the observer records by default
    web.camtab.ssh_run = agx
    sent = []
    web.boat_word = lambda word, args, wait_s=5.0: sent.append((word, args)) or {"ok": True, "text": "noted"}
    r = web.logging_cmd({"op": "start", "label": "w1"})
    assert r["ok"] and r["parts"]["fcu"] == {"ok": True, "file": "pool_20261111T150000Z", "started": False, "note": "already logging (left on at stop)"}
    assert sent and sent[0][0] == "note" and r["parts"]["agx"]["started"] is False and r["parts"]["mac"]["started"] is True
    r = web.logging_cmd({"op": "stop", "confirm": "stop"})
    assert ("log", ["off"]) not in sent and any("left on" in x for x in r["lines"])
    assert not any(c.endswith(" stop") for c in agx.cmds) and agx.rec is True            # the AGX recording left as found


def test_a_sim_run_never_touches_the_real_agx(tmp_path):
    """v0.1.84.6 (session A): in a sim run CamTab's ssh defaulted to the real AGX (agxorin) - RECORD / START LOGGING would restart the
    real observer. OCS_SIM=1 (run_sim.sh), AGX_SSH empty, or a camera only on this Mac -> refused before any ssh."""
    from optiprimex.tools import ocs_web as OW
    from .conftest import ROOT
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    for env, word in (({"OCS_SIM": "1"}, "simulator run"), ({"AGX_SSH": ""}, "no AGX_SSH"),
                      ({"CAM_URL": "http://127.0.0.1:8097/", "AGX_SSH": "u@h"}, "camera is on this Mac")):
        web = OW.Web(a, env=dict(env, OCS_MARKERS=str(tmp_path / "m.json"), OCS_WEB_DRYRUN="1"), feeds=False)
        why = web.camtab.agx_refusal()
        assert word in why, (env, why)
        assert web.camtab._ssh_run("bash scripts/agx_observer.sh start") == why          # no ssh at all
        st = web.camtab.recording_status()
        assert st["agx"]["observer"] == "unknown" and word in st["agx"]["why"]
    web = OW.Web(a, env={"AGX_SSH": "u@h", "CAM_URL": "http://192.168.1.99:8090/", "OCS_MARKERS": str(tmp_path / "m.json")}, feeds=False)
    assert web.camtab.agx_refusal() == ""
    assert "OCS_SIM=1 CAM_URL=" in (ROOT / "scripts" / "run_sim.sh").read_text()


def test_the_mac_frames_carry_the_session_label(tmp_path):
    web = _web(tmp_path)
    now = time.time()
    _beats(web, "usv-pixhawk", [0.0] * 3, now)
    _beats(web, "optiprimeagxorin-desktop", [0.0] * 3, now)
    seen = []

    def start(label, root, hz, max_min, depth):
        seen.append(label)
        web.camtab.rec.st = {"state": "recording", "label": label, "frames": 0, "mb": 0}
        return web.camtab.rec.st
    web.camtab.rec.start = start
    web.boat_word = lambda word, args, wait_s=5.0: {"ok": True, "text": "ok"}
    r = web.logging_cmd({"op": "start", "label": "dry2"})
    assert r["ok"] and seen == [r["label"]] and r["label"].startswith("dry2_")
