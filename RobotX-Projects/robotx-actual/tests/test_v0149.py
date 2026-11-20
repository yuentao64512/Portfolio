"""v0.1.49 (ROADMAP WP10k-b, docs/UI_QOL_2026-11-07.md §E items 17-23): the boat window's tabs, dials, whitelist buttons, the
BlueBoat wireframe with placed callouts, the TUNING knobs (TUNE intent -> SET_TUNING -> the live config -> the next m0t3's frames in
mm_trace; `tune save` -> a profile copy that loads like MM_CONFIG_OVERRIDE's), and the CV-model hint (cv_publish -> perception/health
-> boat window / OCS /state.json `cv_hint`)."""
from __future__ import annotations

import json
import math
import os
import re
import subprocess
import sys
import threading
import time
import types
import urllib.request
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import pytest
import yaml

from optiprimex.companion import config as cfgmod
from optiprimex.companion import tuning as TUN
from optiprimex.ocs.cmd_authority import GUARDS
from optiprimex.tools import boat_web as BW
from optiprimex.tools import cv_hint as CVH

from .conftest import PY, ROOT, Probe, Proc, free_port, free_udp_port, make_ocs_config, make_usv_config, wait_until
from .test_ocs_units import intent, make_authority
from .test_usv_ui import UI_PY, Headless, load_usv_ui

NS = "team/optiprime/v1"
CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")


# ---- item 22: the knob table, the authority's guards, the vehicle's handler, save ------------------------------------
def test_the_knob_table_is_the_tasks_defaults_and_never_an_fcu_parameter():
    from optiprimex.usv.tasks import t1_safe_passage as T1, t3_logistics as T3, scout as SC
    from optiprimex.usv import guidance as GD, guard as GU
    from optiprimex.companion.config import LimitsCfg
    secs = {"t1": T1.DEFAULTS, "t3": T3.DEFAULTS, "guidance": GD.DEFAULTS, "scout": SC.DEFAULTS, "zed_guard": GU.DEFAULTS,
            "limits": {"max_speed_mps": LimitsCfg().max_speed_mps}}                     # v0.1.86.1 (WP48): + guidance / scout / guard / limits
    for k, (sects, d, lo, hi, unit, tip) in TUN.KNOBS.items():
        for s_ in sects:
            want = secs[s_][TUN.field(k)]
            if k == "standoff_m":                                                       # v0.1.56 (D6): null = derived from the dock
                assert want is None and d == T3.standoff_lines(T3.DEFAULTS)[0]
                continue
            assert want == d, k
        assert lo <= d <= hi and tip and unit
    # Dmitrii's whitelist (2026-11-07): the spec's list + T1's transit_v / circle_v
    assert set(TUN.KNOBS) >= {"line_v", "sway_v", "enter_v", "standoff_m", "stop_m", "lead_step_m", "look_s", "transit_v", "circle_v",
                              "la_m", "scan_rate_dps"}                                   # v0.1.86.1: + WP48's (tests/test_v018601.py)
    assert not [k for k in TUN.KNOBS if k.upper().startswith(("SERVO", "PWM", "MOT_", "FRAME"))]
    assert "apply_params.py only" in TUN.RO_NOTE and [n for n, _w in TUN.READ_ONLY] == ["PWM limits", "stern factors"]
    assert TUN.TUNE_STATES == ("MANUAL", "AUTO_HOLD")


def test_the_tune_intent_guards_and_range_checks():
    bus, lc, auth, results = make_authority(bench=False, lc_state="COURSE_VALID")
    assert GUARDS["TUNE"][2] == "SET_TUNING"
    r = auth.on_intent("t", intent("TUNE", args={"key": "sway_v", "value": 0.3}))
    assert r.result == "refused" and "unknown vehicle state" in r.reason                 # no status seen: never guessed
    auth.last_vehicle_state["USV1"] = "AUTO_TASK"
    r = auth.on_intent("t", intent("TUNE", args={"key": "sway_v", "value": 0.3}))
    assert r.result == "refused" and "AUTO_TASK" in r.reason and "MANUAL / AUTO_HOLD" in r.reason
    for st in ("AUTO_SAFE_HOLD", "KILLED", "FAULT"):
        auth.last_vehicle_state["USV1"] = st
        assert auth.on_intent("t", intent("TUNE", args={"key": "sway_v", "value": 0.3})).result == "refused"
    auth.last_vehicle_state["USV1"] = "MANUAL"
    n = len(bus.published)
    for args, word in (({"key": "SERVO1_MAX", "value": 1900}, "not a tuning knob"), ({"key": "sway_v", "value": 0.9}, "out of range 0.05 .. 0.6"),
                       ({"key": "sway_v", "value": "fast"}, "not a number"), ({"key": "sway_v", "value": "nan"}, "not a number"),
                       ({"value": 1}, "not a tuning knob")):
        r = auth.on_intent("t", intent("TUNE", args=args))
        assert r.result == "refused" and word in r.reason, (args, r.reason)
    assert len(bus.published) == n                                                      # nothing went to the boat
    for st in ("MANUAL", "AUTO_HOLD"):
        auth.last_vehicle_state["USV1"] = st
        r = auth.on_intent("t", intent("TUNE", args={"key": "sway_v", "value": "0.4", "extra": 1}))
        assert r.result == "accepted" and bus.published[-1]["body"] == {"action": "SET_TUNING", "task": None, "args": {"key": "sway_v", "value": 0.4}}
        assert bus.published[-1]["topic"].endswith("cmd/usv/run")
    auth.last_vehicle_state["USV1"] = "AUTO_TASK"                                         # save changes nothing on the boat: any state
    r = auth.on_intent("t", intent("TUNE", args={"save": True, "key": "x"}))
    assert r.result == "accepted" and bus.published[-1]["body"]["args"] == {"save": True}


def test_set_tuning_on_the_vehicle_and_the_saved_copy(tmp_path):
    from optiprimex.companion.state import Command, VState
    from optiprimex.usv.command_handler import CommandHandler
    prof = tmp_path / "usv_x.yaml"
    prof.write_text((ROOT / "profiles" / "bench" / "usv_bench.yaml").read_text())
    cfg = cfgmod.load(prof)
    cfg.extra["tuning"] = {"save_dir": str(tmp_path / "launch")}
    fsm = types.SimpleNamespace(state=VState.V2)
    h = CommandHandler(cfg, bus=None, fsm=fsm, run=None, runner=None, clock=time.monotonic)
    seen = []
    h.on_tuned = lambda *a: seen.append(a)
    res, why, done = h._tune({"key": "sway_v", "value": 0.45})
    assert res == "ACCEPTED" and done and "sway_v 0.25 -> 0.45" in why and cfg.extra["t3"]["sway_v"] == 0.45 and seen == [("sway_v", 0.25, 0.45)]
    res, why, _ = h._tune({"key": "scan_rate_dps", "value": 30})
    assert res == "ACCEPTED" and cfg.extra["t1"]["scan_rate_dps"] == 30.0 and cfg.extra["t3"]["scan_rate_dps"] == 30.0       # both scans
    for st in (VState.V4, VState.V5, VState.V1, VState.V6):
        fsm.state = st
        res, why, _ = h._tune({"key": "sway_v", "value": 0.3})
        assert res == "REJECTED" and "MANUAL / AUTO_HOLD" in why and cfg.extra["t3"]["sway_v"] == 0.45
    fsm.state = VState.V3
    assert h._tune({"key": "sway_v", "value": 7})[0] == "REJECTED" and h._tune({"key": "MOT_PWM_MAX", "value": 1})[0] == "REJECTED"
    assert h._tune({"save": True})[0] == "REJECTED"                                     # no profile path known
    h.profile_path = str(prof)
    before = prof.read_text()
    fsm.state = VState.V4                                                               # save: any state
    res, why, _ = h._tune({"save": True})
    assert res == "ACCEPTED" and "t3.sway_v 0.25 -> 0.45" in why and "t1/t3.scan_rate_dps 20 -> 30" in why and "MM_CONFIG_OVERRIDE=" in why
    assert prof.read_text() == before                                                   # never the profile itself
    saved = Path(why.split("saved ", 1)[1].split(" (", 1)[0])
    assert saved.name == "usv_tuned.yaml" and saved.parent.parent == tmp_path / "launch" and re.fullmatch(r"\d{8}_\d{6}", saved.parent.name)
    assert "a copy of" in saved.read_text().splitlines()[0]
    # the copy loads like MM_CONFIG_OVERRIDE's file (run_usv.sh: MM_CONFIG="${MM_CONFIG_OVERRIDE:-$MM_CONFIG}") and the tasks read it
    assert 'MM_CONFIG="${MM_CONFIG_OVERRIDE:-$MM_CONFIG}"' in (ROOT / "scripts" / "run_usv.sh").read_text()
    c2 = cfgmod.load(saved)
    assert c2.extra["t3"]["sway_v"] == 0.45 and c2.extra["t3"]["scan_rate_dps"] == 30.0 and c2.extra["t3"]["standoff_m"] == cfg.extra["t3"]["standoff_m"]
    assert c2.mqtt.port == cfg.mqtt.port and c2.guards.test_mode == cfg.guards.test_mode
    from optiprimex.sim import world as Wd
    from optiprimex.sim.fast_sil import FastSil
    o = Wd.WorldFrame({"lat": 1.2810202, "lon": 103.8550097})
    sil = FastSil(POOL, "TEST_SWAY", {"points": [list(o.to_latlon(10.0, 0.0)), list(o.to_latlon(10.0, -6.0))]}, cfg=c2,
                  start={"x_n": 10.0, "y_e": 0.0, "hdg_deg": 180.0})
    assert sil.task.sway_v == 0.45
    r = sil.run(200)
    assert r["state"] == "DONE"
    v = TUN.view(c2.extra, c2.gains, {"t3": {}})
    assert [x for x in v["knobs"] if x["key"] == "sway_v"][0]["changed"] and "sway_f_pos" in v["read_only"][1]["what"]


def test_ocs_cli_tune_word_and_the_playbook():
    from optiprimex.tools import bench_tui as T, ocs_cli, playbook
    a = ocs_cli.parse(["tune", "sway_v", "0.3"])
    assert ocs_cli.intent_spec(a) == ("TUNE", None, {"key": "sway_v", "value": "0.3"}, None)
    assert ocs_cli.intent_spec(ocs_cli.parse(["tune", "save"])) == ("TUNE", None, {"save": True}, "save")
    code, lines = ocs_cli.execute(None, ocs_cli.parse(["tune"]))
    assert code == 0 and len(lines) == len(TUN.KNOBS) + 2 and "apply_params.py only" in lines[-1]
    assert "tune" in T.INTENT_WORDS and any(w == "tune" for w, _u, _d in T.USAGE)
    st = {"r": "R2", "vehicles": {"USV1": {"state": "AUTO_TASK"}}}
    assert not playbook.check("TUNE", st, "USV1").ok and playbook.check("TUNE", st, "USV1", what="save").ok
    st["vehicles"]["USV1"]["state"] = "MANUAL"
    assert playbook.check("TUNE", st, "USV1").ok


def test_the_boat_window_passes_tune_to_the_ocs_page():
    got = []

    class H(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def do_POST(self):
            body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
            got.append((self.path, self.headers.get("X-OCS-Web"), body))
            out = json.dumps({"ok": True, "lines": ["intent accepted", "  ack DONE      state=MANUAL sway_v 0.25 -> 0.4"]}).encode()
            self.send_response(200)
            self.send_header("Content-Length", str(len(out)))
            self.end_headers()
            self.wfile.write(out)
    port = free_port()
    srv = HTTPServer(("127.0.0.1", port), H)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = BW.ocs_tune("http://127.0.0.1:%d" % port, {"key": "sway_v", "value": "0.4"})
        assert r["ok"] and got[-1] == ("/cmd", "1", {"line": "tune sway_v 0.4"}) and "0.25 -> 0.4" in r["text"]
        assert BW.ocs_tune("http://127.0.0.1:%d" % port, {"save": True})["line"] == "tune save"
        n = len(got)
        r = BW.ocs_tune("http://127.0.0.1:%d" % port, {"key": "sway_v", "value": "9"})
        assert not r["ok"] and "out of range" in r["text"] and len(got) == n                         # a typo never travels
    finally:
        srv.shutdown()
    r = BW.ocs_tune("http://127.0.0.1:%d" % free_port(), {"key": "sway_v", "value": 0.4})
    assert not r["ok"] and "not reachable" in r["text"] and "authority is the OCS" in r["text"]


# ---- the live chain: the TUNE intent through the OCS, SET_TUNING on the vehicle, the next m0t3's frames; the whitelist words ----
@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    if not UI_PY.exists():
        pytest.skip(f"{UI_PY} not found")
    tmp = tmp_path_factory.mktemp("v149")
    mod = load_usv_ui()
    share_port = free_udp_port()
    ui = Headless(mod, [f"udpin:127.0.0.1:{fake_fcu.mav_port}", "--share", f"udpout:127.0.0.1:{share_port}",
                         "--mqtt", f"127.0.0.1:{brokers.boat_port}", "--points", str(tmp / "points.json"), "--co", str(tmp / "co.json"),
                         "--log-dir", str(tmp / "logs")])
    ui.start()
    cfg = make_usv_config(tmp, brokers, fake_fcu)
    d = yaml.safe_load(cfg.read_text())
    d["mavlink"]["endpoint"] = f"udpin:127.0.0.1:{share_port}"
    d["tuning"] = {"save_dir": str(tmp / "launch")}
    cfg.write_text(yaml.safe_dump(d))
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(cfg), "--trace", str(tmp / "mm_trace.jsonl")], tmp / "mm.log")
    hub = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#"]).start()
    boat = Probe(brokers.boat_port, ["local/#", f"{NS}/status/#"]).start()
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(make_ocs_config(tmp, brokers))], tmp / "ocs.log")
    from optiprimex.tools.ocs_cli import OcsConsole
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    model = BW.Model("usv")
    link = BW.Link(model, "127.0.0.1", brokers.boat_port)
    port = free_port()
    srv = BW.Server(("127.0.0.1", port), BW.make_handler(model, link, bar=None))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 20, include_past=True), mm.log()
    assert hub.wait(lambda m: m["topic"] == "ocs/state", 15, include_past=True), ocs.log()
    assert wait_until(lambda: model.connected and "fcu" in model.last, 10)
    yield {"ui": ui, "mm": mm, "hub": hub, "boat": boat, "con": con, "ocs": ocs, "tmp": tmp, "cfg": cfg, "model": model, "url": f"http://127.0.0.1:{port}"}
    srv.shutdown(); link.close(); con.stop(); hub.stop(); boat.stop(); ocs.kill(); mm.kill(); ui.stop()


def _post(u, body):
    req = urllib.request.Request(u, data=json.dumps(body).encode(), method="POST", headers={"Content-Type": "application/json", "X-Boat-Web": "1"})
    with urllib.request.urlopen(req, timeout=8) as r:
        return json.loads(r.read())


def test_every_whitelist_button_round_trips_through_usv_ui(stack):
    url, ui, model = stack["url"], stack["ui"], stack["model"]
    page = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text()
    for bid, word, args, _tip in BW.BUTTONS:
        assert f'id="b-{bid}" data-word="{word}"' in page, bid
        assert word in BW.WHITELIST
    for bid, word, args, _tip in BW.BUTTONS:
        a = ["from", "the", "page"] if args == "text" and word == "note" else (["V149"] if args == "text" else (args or []))
        time.sleep(0.6)                                                                   # usv_ui's flood guard: one word per 0.5 s
        r = _post(url + "/cmd", {"word": word, "args": a})
        assert r["word"] == word and r["text"] and "no answer" not in r["text"], (bid, r)
        assert any(t == ("[page] " + " ".join([word] + a)).strip() for _, t, _k in ui.app.log[-12:]), (bid, ui.app.log[-6:])
        if word in ("handover", "takeover", "note", "points", "log", "cv", "stop"):
            assert r["ok"], (bid, r)
    assert model.cmds[0]["word"] == "points" and len(model.cmds) >= len(BW.BUTTONS)
    time.sleep(0.6)
    assert _post(url + "/cmd", {"word": "takeover"})["active"] is True
    snap = model.snapshot()
    assert snap["tabs"]["diag"]["cmds"][0]["word"] == "takeover"
    assert wait_until(lambda: any("[page] takeover" in x[1] for x in (model.last["fcu"][1].get("tail") or [])), 3)   # usv_ui's console tail


def test_tune_reaches_the_live_config_and_the_next_m0t3s_frames(stack):
    con, hub, boat, ui, tmp = stack["con"], stack["hub"], stack["boat"], stack["ui"], stack["tmp"]
    from optiprimex.tools import ocs_cli
    time.sleep(0.6)
    assert _post(stack["url"] + "/cmd", {"word": "handover"})["active"] is False          # the mission manager may take the FCU
    code, lines = ocs_cli.execute(con, ocs_cli.parse(["tune", "sway_v", "9"]))
    assert code == 1 and any("out of range" in x for x in lines), lines
    code, lines = ocs_cli.execute(con, ocs_cli.parse(["tune", "sway_v", "0.45"]))
    assert code == 0 and any("ack DONE" in x for x in lines) and any("sway_v 0.25 -> 0.45" in x for x in lines), (lines, stack["mm"].log()[-1500:])
    assert boat.wait(lambda m: m["topic"] == "local/usv/mm/state" and any(k["key"] == "sway_v" and k["value"] == 0.45 and k["changed"]
                                                                        for k in ((m["env"]["body"].get("tuning") or {}).get("knobs") or [])), 3)
    assert wait_until(lambda: any(k["value"] == 0.45 for k in stack["model"].snapshot()["tabs"]["control"]["tuning"]["knobs"] if k["key"] == "sway_v"), 3)
    time.sleep(2.5)                                                                       # tui_active_s: usv_ui's beat has stopped
    r = con.intent("ENABLE_AUTO", wait_s=5)
    assert r["result"]["result"] == "accepted", r
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10, include_past=True), stack["mm"].log()[-2000:]
    st = hub.last(f"{NS}/status/usv")["env"]["body"]
    lat, lon = st["lat"], st["lon"]
    b = [lat, lon + 6.0 / (111320.0 * math.cos(math.radians(lat)))]                      # 6 m east, starboard to B: bow north
    r = con.intent("START_TASK", "TEST_SWAY", args={"points": [[lat, lon], b]}, wait_s=5)
    assert r["result"]["result"] == "accepted" and [a["body"]["result"] for a in r["acks"]][-1] == "DONE", (r, stack["mm"].log()[-2000:])
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_TASK", 5, include_past=True)
    time.sleep(1.0)
    r = con.intent("TUNE", args={"key": "sway_v", "value": 0.3})                           # a running task keeps its values
    assert r["result"]["result"] == "refused" and "AUTO_TASK" in r["result"]["reason"], r
    code, lines = ocs_cli.execute(con, ocs_cli.parse(["tune", "save"]))
    assert code == 0 and any("t3.sway_v 0.25 -> 0.45" in x for x in lines), lines
    saved = next((tmp / "launch").glob("*/usv_tuned.yaml"))
    assert cfgmod.load(saved).extra["t3"]["sway_v"] == 0.45 and yaml.safe_load(stack["cfg"].read_text())["t3"]["sway_v"] == 0.25
    trace = tmp / "mm_trace.jsonl"

    def sway_frames():
        sub, out = None, []
        for line in trace.read_text().splitlines():
            e = json.loads(line)
            if e["k"] == "obs":
                sub = (e.get("task"), e.get("sub"))
            elif e["k"] == "frame" and e.get("why") == "cmd" and sub == ("TEST_SWAY", "SWAY"):
                out.append(e["m"])
        return out
    assert wait_until(lambda: max([max(abs(m[1]), abs(m[3])) for m in sway_frames()] or [0]) > 0.4, 60, 0.5), (sway_frames()[-5:], stack["mm"].log()[-2000:])
    peak = max(max(abs(m[1]), abs(m[3])) for m in sway_frames())
    assert 0.4 < peak <= 0.4501, peak                                                     # the tuned 0.45, not the profile's 0.25
    assert any(json.loads(x).get("k") == "tune" and json.loads(x).get("new") == 0.45 for x in trace.read_text().splitlines())
    con.intent("ABORT_TASK", wait_s=3)
    con.intent("DISABLE_AUTO", wait_s=3)


# ---- items 18 / 19 / 21: the dials' projection, the placer, the tabs' content ------------------------------------------
def test_the_radar_projection_and_rings():
    assert BW.RINGS_M == (5.0, 10.0, 25.0, 40.0)
    assert BW.radar_xy(10.0, 0.0, 0.0) == [0.0, -0.5]                                    # 10 m dead ahead: half way up (sqrt scale)
    assert BW.radar_xy(0.0, 10.0, 0.0) == [0.5, 0.0]                                     # 10 m to starboard
    assert BW.radar_xy(0.0, 40.0, 90.0) == pytest.approx([0.0, -1.0], abs=1e-4)          # heading east: east is ahead (heading-up)
    assert BW.radar_xy(-40.0, 0.0, 90.0) == pytest.approx([1.0, 0.0], abs=1e-4)          # south = starboard when heading east
    assert BW.radar_r(5.0) == pytest.approx(math.sqrt(1 / 8)) and BW.radar_r(80.0) == 1.0
    m = BW.Model("usv")
    m.connected = True
    m.on_message(f"{NS}/telemetry/usv/map", json.dumps({"body": {"boat": {"x_n": 10, "y_e": 10, "hdg_deg": 90}, "entities": [
        {"id": 1, "kind": "buoy", "colour": "red", "x_n": 10, "y_e": 20}, {"id": 2, "kind": "dock", "colour": "red", "x_n": 0, "y_e": 10}],
        "rays": [{"colour": "green", "x_n": 10, "y_e": 10, "az_deg": 0, "n": 2}], "dock": {"x_n": 0, "y_e": 10, "line_hdg_deg": 90, "length_m": 6}}}).encode(), now=50.0)
    R = m.radar(50.5)
    red = R["items"][0]
    assert red["kind"] == "buoy" and red["x"] == 0.0 and red["y"] == -0.5 and red["hex"] == BW.OBJ["red"] and R["state"] == "live"
    assert R["items"][1]["x"] == pytest.approx(0.5) and R["rays"][0]["pts"][-1][0] < -0.95           # south = starboard; the ray (north = port) reaches the 40 m edge
    assert R["dock"] and abs(R["dock"]["pts"][len(R["dock"]["pts"]) // 2][0] - 0.5) < 0.01
    assert [g["m"] for g in R["rings"]] == [5.0, 10.0, 25.0, 40.0] and m.radar(53.0)["state"] == "stale"
    assert BW.Model("usv").radar(1.0)["text"][0] == {"u": "no map digest (the mission manager's telemetry/usv/map)"}


def _rich_model(now):
    m = BW.Model("usv")
    m.connected, m.broker = True, "127.0.0.1:x"
    fcu = {"hb_age_s": 0.3, "servo": [1660, 1500, 1890, 1340, 1500, 1500, 1500, 1500], "active": False, "fcu_link": "fake", "mode": "CUSTOM_MANUAL",
           "armed": True, "lat": 1.28102, "lon": 103.85501, "heading_deg": 350, "groundspeed_mps": 0.42, "gps": {"fix": 3, "sats": 12, "hdop": 0.9},
           "ekf_flags": 0x33f, "ekf_pos_ok": True, "nav": {"target_bearing_deg": 10, "xtrack_m": 0.2, "wp_dist_m": 9},
           "rates_hz": {"HEARTBEAT": 1.0}, "ages_s": {"HEARTBEAT": 0.2}, "tail": [[1.0, "[page] points", "cmd"]]}
    for i in range(30):
        m.on_message("local/usv/ui/fcu", json.dumps({"body": dict(fcu, lat=1.28102 + i * 3e-6, heading_deg=(350 + 2 * i) % 360)}).encode(), now=now - 15 + 0.5 * i)
    m.on_message(f"{NS}/status/usv", json.dumps({"body": {"state": "AUTO_TASK", "task_name": "T1", "task_state": "TRANSIT", "health": {"fcu_age_s": 0.2}}}).encode(), now=now)
    for d in ("sim_observer", "fake_perception"):
        b = {"ok": True, "detector": d, "rate_hz": 4.0}
        if d == "sim_observer":
            b.update(model="cvt-v2-l", classes=["buoy", "dock", "window", "indicator", "box"])
        m.on_message("local/usv/perception/health", json.dumps({"body": b}).encode(), now=now - (0.9 if d == "sim_observer" else 0.7))
    m.cam = (now, {"source": "sim", "frames": 10, "age_s": 0.2, "clients": 2}, "http://127.0.0.1:8097/status", None)
    return m


def test_the_tabs_carry_live_values_and_the_callouts_never_overlap():
    now = 1000.0
    m = _rich_model(now)
    s = m.snapshot(now=now)
    t = s["tabs"]
    assert t["control"]["chain"][2][1] == {"u": "no mm/state"}          # v0.1.55 (D2): the commanded side comes from mm/state cmd
    assert "implied u · v · r" in t["control"]["alloc"][-1][0] and t["control"]["alloc"][0][1][0] == "+0.40"
    assert "MAE" in t["control"]["response"][3][0] and t["control"]["response"][3][1][0].endswith("°")
    assert t["control"]["tuning"]["live"] is False and t["control"]["tuning_ok"] is False                 # AUTO_TASK, no mm state
    assert [x["ch"] for x in t["actuation"]["thr"]] == [1, 3, 4, 2] and all("usv_ui MOTOR_NAMES" in r[1] and "to verify" not in r[1] for r in t["actuation"]["map"])   # v0.1.52: "to verify" removed
    assert t["actuation"]["thr"][1]["rows"][3][1] == {"w": "SATURATED"}
    assert dict(t["nav"]["ekf"])["set"].startswith("attitude") and "COG" in str(t["nav"]["att"][1][1])
    assert t["diag"]["tail"] == [[1.0, "[page] points", "cmd"]] and dict(t["diag"]["mavlink"])["HEARTBEAT"] == ["1.0 Hz", "0.2 s"]
    tele = s["tele"]
    assert tele["n"] == 30 and tele["t"][-1] == -0.5 and tele["t"][0] == -15.0 and tele["series"]["pwm_M-P"][-1] == 1660
    assert tele["series"]["pwm_SW-F"][-1] == 1340 and tele["series"]["hb"][-1] == 0.3
    m.on_message("local/usv/ui/fcu", json.dumps({"body": {"hb_age_s": 0.1}}).encode(), now=now + 70)
    assert m.telemetry(now + 70)["n"] == 1                                                # a 60 s window
    c = s["dials"]["compass"]
    assert c["hdg"] == 48 and c["cog"] is not None and min(c["cog"], 360 - c["cog"]) < 1.0
    assert s["dials"]["speed"]["sog"] == 0.42 and s["dials"]["speed"]["text"][1] == {"u": "cmd surge: no mm/state"}
    # the camera callout: the stream source and every perception source with its age, and the CV hint
    cam = s["wf"]["cam"]
    assert cam["l2"] == "sources: fake_perception 0.7 s · sim_observer 0.9 s" and cam["l3"].startswith("stream: sim @ 127.0.0.1:8097")
    assert cam["l4"] == "CV: cvt-v2-l · 5 classes · buoys → T1 gates · dock / window / indicator → T3 · box → m0t4"
    # the placer: two columns beside the hull, nothing overlapping, inside the frame, fixed slots
    co = s["callouts"]
    assert len(co) == 9 and BW.overlaps(co) == []
    assert all(0 <= b["x"] and b["x"] + b["w"] <= BW.WF_W and 0 <= b["y"] and b["y"] + b["h"] <= BW.WF_H for b in co)
    camb = next(b for b in co if b["id"] == "cam")
    assert any("box → m0t4" in x for x in camb["detail"]) and all(len(x) <= int((camb["w"] - 12) / BW.CO_CHAR) for x in camb["lines"])   # v0.1.52: the detail
    long = {k: dict(v) for k, v in s["wf"].items()}
    long["cam"]["l1"] = "x " * 300
    long["box"]["l1"] = "y" * 400
    co2 = m.callouts(long)
    assert BW.overlaps(co2) == [] and [(b["id"], b["y"], b["h"]) for b in co2] == [(b["id"], b["y"], b["h"]) for b in co]   # no jumps
    assert BW.HULL["x1"] - BW.HULL["x0"] == pytest.approx(0.76 / 1.19 * (BW.HULL["y1"] - BW.HULL["y0"]), rel=0.01)    # BlueBoat aspect


def test_cv_hint_and_cv_publish_health(tmp_path):
    assert CVH.hint([]) is None
    h = CVH.hint([("fake_perception", {"detector": "fake_perception"}, 0.5)])
    assert h["text"] == "CV: fake_perception (no model: world-file stand-in)" and h["classes"] is None
    h = CVH.hint([("cv_publish", {"detector": "actual_buoy_observer", "model": "students best.pt", "classes": ["buoy"]}, 3.0)])
    assert h["text"] == "CV: students best.pt · 1 class · buoys → T1 gates" and h["stale"] and h["by_task"] == {"T1": ["buoy"], "TEST_PATH": ["buoy"]}
    sys.path.insert(0, str(ROOT / "cv" / "publish"))
    try:
        import cv_publish as CP
    finally:
        sys.path.pop(0)
    d = tmp_path / "cvt-folder"
    d.mkdir()
    (d / "classes.yaml").write_text("names:\n  0: buoy\n  1: dock\n  2: window\n  3: indicator\n  4: box\n")
    (d / "PROVENANCE.md").write_text("# cvt-v2-l (CV_TRAIN 2026-11-06)\n\nweights ...\n")
    a = types.SimpleNamespace(model=str(d), demo=False)
    assert CP.model_info(a) == ("cvt-v2-l", ["buoy", "dock", "window", "indicator", "box"])
    assert CP.model_info(types.SimpleNamespace(model=None, demo=True)) == ("synthetic", ["buoy"])
    assert CP.model_info(types.SimpleNamespace(model=None, demo=False)) == ("students best.pt", ["buoy"])
    # the health body carries detector + model + classes; the observer's payload switches it without a restart (run_sim --detector)
    sent = []
    pub = CP.Publisher(types.SimpleNamespace(mqtt="127.0.0.1:1", vehicle="usv", vehicle_id="USV1", camera="zed2i_left", detector="sim_observer",
                                             model=None, demo=False))
    pub.c = types.SimpleNamespace(publish=lambda topic, payload, qos=0: sent.append((topic, json.loads(payload)["body"])))
    CP.observer_model(pub, {"detector": "synthetic"})
    pub.health()
    assert sent[-1][0] == "local/usv/perception/health" and sent[-1][1]["detector"] == "sim_observer" and sent[-1][1]["model"] == "synthetic"
    CP.observer_model(pub, {"detector": "model", "model": "cvt-v2-l", "classes": ["buoy", "dock", "window", "indicator", "box"]})
    pub.health()
    b = sent[-1][1]
    assert b["model"] == "cvt-v2-l" and b["observer_detector"] == "model" and len(b["classes"]) == 5
    assert CVH.hint([("sim_observer", b, 0.4)])["text"] == "CV: cvt-v2-l · 5 classes · buoys → T1 gates · dock / window / indicator → T3 · box → m0t4"
    # the sim observer writes model / classes into its payload when --detector model
    so_src = (ROOT / "cv" / "sim_observer" / "sim_observer.py").read_text()
    assert 'payload["model"] = os.path.basename(os.path.normpath(a.model))' in so_src
    # the OCS page's /state.json carries it (the parent draws it on OVERVIEW / MISSION CONTROL)
    src = (ROOT / "optiprimex" / "tools" / "ocs_web.py").read_text()
    assert 'snap["cv_hint"] = _cv_hint(srcs)' in src


# ---- the page in Chrome: tabs, buttons with titles, callouts fit at 1440 px, the plotter -----------------------------
def _serve(model):
    port = free_port()
    srv = BW.Server(("127.0.0.1", port), BW.make_handler(model, None, bar=None))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{port}"


def _dom(url):
    return subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=8000", "--window-size=1440,1000", "--dump-dom", url],
                          capture_output=True, text=True, timeout=90).stdout


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_page_in_chrome_at_1440():
    page = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text()
    a, b = page.index("// ---- plotter"), page.index("// ---- end plotter")
    assert page[a:b].count("\n") <= 100 and "<script src" not in page                     # no library beyond the 100-line plotter
    assert "transition" not in re.sub(r"<script\b.*?</script>", "", page, flags=re.S) and "@keyframes" not in page      # no decorative motion
    m = _rich_model(time.monotonic())
    m.hist.clear()
    t0 = time.monotonic()
    for i in range(20):
        m.hist.append((t0 - 10 + 0.5 * i, 0.4, 10.0 * i, [1600, 1500, 1450, 1520], 0.3))
    srv, url = _serve(m)
    try:
        for theme in ("dark", "hc-light"):
            dom = _dom(url + "/?theme=" + theme)
            groups = re.findall(r'<g id="co-([a-z-]+)" data-x="([\d.]+)" data-y="([\d.]+)" data-w="([\d.]+)" data-h="([\d.]+)"[^>]*data-fit="(\d)"', dom)
            assert len(groups) == 9, theme
            boxes = [{"id": g[0], "x": float(g[1]), "y": float(g[2]), "w": float(g[3]), "h": float(g[4])} for g in groups]
            assert BW.overlaps(boxes) == [] and all(g[5] == "1" for g in groups), (theme, groups)          # no overlap, no text outside a box
            assert "box → m0t4" in dom and "stream: sim @ 127.0.0.1:8097" in dom
            assert re.search(r'<svg id="radar"[^>]*data-items="\d+"', dom) and 'data-hdg="' in dom
            body = re.sub(r"<script\b.*?</script>", "", dom, flags=re.S)
            buttons = re.findall(r"<button\b[^>]*>", body)
            assert len(buttons) >= 6 + 10 + 11 + 2 and [x for x in buttons if "title=" not in x] == []    # tabs, words, apply x11, save, pause
            for tab in ("CONTROL", "TELEMETRY", "NAV", "DIAG"):            # v0.1.74 (N6): ACTUATION's cards live in CONTROL & ACTUATION
                sec = re.search(r'<section id="t-%s"[^>]*>(.*?)</section>' % tab, body, re.S).group(1)
                assert "phase 3" not in sec and len(re.findall(r'class="(?:row|ev|plot)', sec)) >= 4, tab
        dom = _dom(url + "/?tab=TELEMETRY")
        pts = [int(x) for x in re.findall(r'<canvas class="plot" id="pl-[a-z]+"[^>]*data-points="(\d+)"', dom)]
        assert len(pts) == 4 and min(pts) >= 19, pts
        assert '<section id="t-TELEMETRY" class="grid">' in dom and '<section id="t-OVERVIEW" class="grid" hidden="">' in dom
    finally:
        srv.shutdown()


def test_the_ocs_page_draws_the_cv_hint_and_the_boat_window_follows_the_theme():
    """v0.1.49 (merge): cv_hint on OVERVIEW's system card + MISSION CONTROL's mission line; the boat window's /prefs."""
    from .conftest import ROOT
    p = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'sysRow("AGX · CV model"' in p and 'id="c-cv"' in p and "function drawCvLine()" in p and "(cv.by_task||{})[m.task]" in p
    b = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text()
    assert "async function followPrefs()" in b and "'X-Boat-Web':'1'},body:JSON.stringify({theme:" in b
    w = (ROOT / "optiprimex" / "tools" / "boat_web.py").read_text()
    assert 'if path == "/prefs":' in w and 'self.path == "/prefs":' in w
