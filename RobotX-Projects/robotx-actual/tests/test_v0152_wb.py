"""v0.1.52 (helper wb): the simulator-side fixes from Dmitrii's run_sim day.

(a) the boat glyph: map.html's hull pentagon read as a rotated square -> the BlueBoat catamaran (two hulls, the cross beam, a heading
    line; to scale zoomed in, >= 22 px long zoomed out; the theme's --text); the SIM page's WORLD canvas the same (was a triangle);
    map.html is served no-store by ocs_web and judge_web (no stale copy).
(b) GPS lost on the SIM page: the fake FCU brings the fix and the EKF position back by itself; the vehicle's FAULT EKF_BAD stays
    latched until confirm_safe (by design) - the SIM page now shows the vehicle's state and says "latched: confirm_safe".
(c) the SIM page's transmitter switches (mode, kill, E-stop RC5, RC6) are status chips; the faults and RESET SIM stay buttons;
    simview's POST /fcu keeps the switches for the tools (fault_timing, sim_day).
(d) rc-test auto-sent RunStart the moment the boat reported AUTO, so the judge page's RunStart did nothing (ignored in R5) and an
    m1t1 preselected without --auto waited for start_task: RunStart is now manual with rc-test (tools/rctest_manual.py, the server
    file unmodified; RCTEST_RUNSTART=auto = as before), and the judge page says when a RunStart is a duplicate.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import textwrap
import time
from pathlib import Path

import pytest

from .conftest import PY, ROOT, Probe, Proc, free_port, free_udp_port, make_usv_config, wait_until

TOOLS = ROOT / "optiprimex" / "tools"
MAP = (TOOLS / "map.html").read_text()
SIM = (ROOT / "optiprimex" / "sim" / "sim.html").read_text()
NS = "team/optiprime/v1"
NODE = shutil.which("node")


def _boat_block(page: str, start: str, end: str) -> str:
    i = page.index(start)
    return page[i:page.index(end, i)]


# ---- (a) the boat glyph ------------------------------------------------------------------------------------------------------------
def test_the_map_boat_is_the_blueboat_catamaran():
    boat = _boat_block(MAP, "  // boat + track (v0.1.46.1", "  // scale bar")
    assert "P(11, 0), P(4, 5)" not in boat                                          # the pentagon is gone
    assert "catamaran()" in boat and "boatPxPerM(view.scale)" in boat and "themeVar('--text'" in boat
    assert "const hd = P(BOAT.L / 2 * k" in boat and "b.trail" in boat               # heading line ahead of the bow, the track
    for col in ("#3fd16b", "#ff5a5a", "#5aa0ff", "#2d7a44", "#f5f5f0"):
        assert col not in boat, col                                                  # v0.1.46.1: no object colour on the boat
    assert "const BOAT = { L: 1.19, B: 0.76, hull_w: 0.20, min_px: 22 };" in MAP
    assert "function boatPxPerM(scale) { return Math.max(scale, BOAT.min_px / BOAT.L); }" in MAP
    assert "ctx.strokeStyle = themeVar('--acc', '#4cc2ff'); ctx.lineWidth = 2; ctx.beginPath(); ctx.arc(px, py, 5, 0, 6.283)" in MAP  # the carrot


def test_the_sim_world_boat_is_the_same_catamaran():
    assert "ctx.moveTo(0,-14);ctx.lineTo(8,10);ctx.lineTo(-8,10)" not in SIM          # the triangle is gone
    assert "const BOAT={L:1.19,B:0.76,hull_w:0.20,min_px:22};" in SIM and "for(const poly of catamaran())" in SIM
    assert 'themeVar("--text","#e6ebf0")' in SIM and "boatPxPerM(view.s)" in SIM
    assert "camera cone" in SIM and "Math.tan(50*Math.PI/180)*60" in SIM               # the cone stays


def _js_fn(page: str, start: str, end: str) -> str:
    i = page.index(start)
    return page[i:page.index(end, i)]


@pytest.mark.skipif(NODE is None, reason="no node")
def test_the_catamaran_geometry_and_the_minimum_size(tmp_path):
    """Both pages' polygons, evaluated: 1.19 m long, 0.76 m beam, hulls 0.20 m; px per metre = the scale, never under 22 px long."""
    out = {}
    for name, page, a, b in (("map", MAP, "const BOAT = {", "function themeVar(n, d) {"),
                             ("sim", SIM, "const BOAT={", "function themeVar(n,d)")):
        js = _js_fn(page, a, b) + "\nconsole.log(JSON.stringify({polys: catamaran(), small: boatPxPerM(2), big: boatPxPerM(80)}));"
        f = tmp_path / f"{name}.js"
        f.write_text(js)
        out[name] = json.loads(subprocess.run([NODE, str(f)], capture_output=True, text=True, timeout=20, check=True).stdout)
    assert out["map"] == out["sim"]                                                  # the same shape on both pages
    polys = out["map"]["polys"]
    xs = [p[0] for poly in polys for p in poly]
    ys = [p[1] for poly in polys for p in poly]
    assert len(polys) == 3 and max(xs) - min(xs) == pytest.approx(1.19) and max(ys) - min(ys) == pytest.approx(0.76)
    for hull in polys[:2]:
        w = max(p[1] for p in hull) - min(p[1] for p in hull)
        assert w == pytest.approx(0.20)                                              # each hull 0.2 m wide
        assert max(p[0] for p in hull) == pytest.approx(0.595)                       # a pointed bow ahead
    assert out["map"]["small"] * 1.19 == pytest.approx(22) and out["map"]["big"] == 80   # >= 22 px zoomed out, to scale zoomed in


def test_map_html_is_served_no_store():
    ow = (TOOLS / "ocs_web.py").read_text()
    send = _js_fn(ow, "        def _send(self, code: int, body: bytes, ctype: str) -> None:", "        def _json(")
    assert 'self.send_header("Cache-Control", "no-store")' in send
    route = _js_fn(ow, '            elif path == "/map.html":', '            elif path == "/meta.json":')
    assert "self._send(200" in route                                                 # map.html goes through _send: no-store
    jw = (TOOLS / "judge_web.py").read_text()
    assert jw.count('self.send_header("Cache-Control", "no-store")') >= 2 and 'elif path in ("/map.html", "/live_dot.js"):' in jw


# ---- (c) the transmitter as status ---------------------------------------------------------------------------------------------------
def test_the_sim_page_shows_the_transmitter_and_keeps_the_faults():
    head = SIM[SIM.index("<header>"):SIM.index("</header>")]
    for gone in ('id="m0"', 'id="m4"', 'id="kill"', 'id="estop"', 'id="rc6"'):
        assert gone not in head, gone                                                # no switch from this page
    for chip in ("c-mode", "c-arm", "c-kill", "c-estop", "c-rc6"):
        assert f'<span class="chip st unk" id="{chip}">' in head, chip
    for btn in ('id="gps"', 'id="pause"', 'id="rst"'):
        assert btn in head, btn                                                      # the faults and RESET SIM stay
    js = SIM[SIM.index("<script>"):]
    assert "FCUSET({mode:" not in js and "FCUSET({kill:" not in js and "FCUSET({rc6:" not in js and "FCUSET({rc5:" not in js
    assert 'FCUSET({gps_fix:' in js and 'post("/fcu",{pause_stream:4})' in js
    assert "function txChips(f)" in js and "txChips(f);vehLine(W.vehicle)" in js
    assert [b for b in re.findall(r"<button\b[^>]*>", SIM) if "title=" not in b] == []


def test_the_fcu_api_keeps_the_switches_for_the_tools():
    from optiprimex.sim import simview as SV
    sent = []

    class FakeFcu:
        def ask(self, msg, timeout_s=0.3):
            sent.append(msg)
            return {"ok": True}
    sim = SV.Sim.__new__(SV.Sim)
    sim.fcu = FakeFcu()
    for s in ({"kill": True}, {"rc6": 1900}, {"rc5": 1900}, {"estop": False}, {"mode": 0}, {"gps_fix": 0}):
        assert sim.fcu_post({"set": s})["ok"], s
    assert sent == [{"set": s} for s in ({"kill": True}, {"rc6": 1900}, {"rc5": 1900}, {"estop": False}, {"mode": 0}, {"gps_fix": 0})]
    assert not sim.fcu_post({"set": {"mode": 15}})["ok"]                              # still only the transmitter's MANUAL / HOLD
    assert sim.fcu_post({"set": {"armed": True}})["reply"] == {"ok": True} and sent[-1] == {"set": {}}   # never arming from here
    from optiprimex.tools import fault_timing, sim_day                               # the tools that drive kill / RC6 through it
    assert "/fcu" in Path(fault_timing.__file__).read_text() and "/fcu" in Path(sim_day.__file__).read_text()


# ---- (b) GPS lost: back by itself, the latch said ------------------------------------------------------------------------------------
def test_the_vehicle_view_says_latched_only_when_the_cause_is_gone():
    from optiprimex.sim.simview import vehicle_view
    fcu = {"ekf_pos_ok": True, "gps_fix": 3}
    v = vehicle_view({"state": "FAULT", "fault": "EKF_BAD", "health": {"faults": []}, "fcu": fcu})
    assert v["latched"] and v["fault"] == "EKF_BAD" and v["ekf_pos_ok"] is True and v["gps_fix"] == 3
    v = vehicle_view({"state": "FAULT", "fault": "EKF_BAD", "health": {"faults": ["EKF_BAD"]}, "fcu": {"ekf_pos_ok": False, "gps_fix": 0}})
    assert not v["latched"] and v["faults_now"] == ["EKF_BAD"] and v["ekf_pos_ok"] is False
    assert not vehicle_view({"state": "KILLED", "kill": True, "health": {"faults": []}})["latched"]      # the kill still on
    assert vehicle_view({"state": "KILLED", "kill": False, "health": {"faults": []}})["latched"]
    assert not vehicle_view({"state": "AUTO_HOLD", "health": {"faults": []}})["latched"]
    js = SIM[SIM.index("function vehLine(v)"):]
    assert "latched - the cause is gone: confirm_safe, then enable_auto" in js and 'id="veh"' in SIM


def test_the_fake_fcu_brings_the_ekf_position_back_by_itself(fake_fcu):
    os.environ.setdefault("MAVLINK20", "1")
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(f"udpin:127.0.0.1:{fake_fcu.mav_port}")

    def flags(n=3):
        seen = []
        t0 = time.time()
        while len(seen) < n and time.time() - t0 < 6:
            x = m.recv_match(type="EKF_STATUS_REPORT", blocking=True, timeout=1)
            if x is not None:
                seen.append(int(x.flags))
        return seen
    try:
        assert flags(1)[-1] & 0x10                                                   # pos_horiz_abs
        fake_fcu.set(gps_fix=0)
        assert wait_until(lambda: not (flags(1)[-1] & 0x10), 6)
        fake_fcu.set(gps_fix=3)
        assert wait_until(lambda: flags(1)[-1] & 0x10, 6)                            # no latch in the fake: back with the fix
        assert fake_fcu.state()["gps_fix"] == 3 and fake_fcu.state()["ekf_ok"] is True
    finally:
        fake_fcu.set(gps_fix=3)
        m.close()


def test_gps_lost_and_back_on_the_live_chain(brokers, fake_fcu, tmp_path):
    """fake FCU + the mission manager + simview's feed: GPS lost -> FAULT EKF_BAD (the cause on) -> GPS back -> the EKF position
    returns, FAULT stays latched (the SIM page says confirm_safe) -> confirm_safe -> MANUAL."""
    pytest.importorskip("numpy")
    pytest.importorskip("PIL")
    from optiprimex.sim import simview as SV
    from optiprimex.tools.vcmd import VehicleCommander
    cfg = make_usv_config(tmp_path, brokers, fake_fcu)
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(cfg)], tmp_path / "mm.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#"]).start()
    vc = VehicleCommander(port=brokers.hub_port)
    vc.start()
    a = SV.parse_args(["--world", "pool", "--fcu-control", str(fake_fcu.ctrl_port), "--port", str(free_port()), "--hub-port", "0",
                       "--boat-port", str(brokers.boat_port), "--width", "160", "--height", "90"])
    sim = SV.Sim(a)
    sim.watch_dock()
    veh = lambda: (sim.world_json().get("vehicle") or {})                            # noqa: E731
    try:
        assert wait_until(lambda: veh().get("state") == "MANUAL" and veh().get("ekf_pos_ok") is True, 25), mm.log()
        assert sim.fcu_post({"set": {"gps_fix": 0}})["ok"]
        assert wait_until(lambda: veh().get("state") == "FAULT" and "EKF_BAD" in veh().get("faults_now", []), 10), (veh(), mm.log()[-1500:])
        assert veh()["latched"] is False and veh()["ekf_pos_ok"] is False
        assert sim.fcu_post({"set": {"gps_fix": 3}})["ok"]
        assert wait_until(lambda: veh().get("ekf_pos_ok") is True and veh().get("latched") is True, 10), veh()
        assert veh()["state"] == "FAULT" and veh()["fault"] == "EKF_BAD" and veh()["faults_now"] == []
        assert [x["body"]["result"] for x in vc.send("run", "CONFIRM_SAFE", wait_s=4.0)][-1] == "DONE"
        assert wait_until(lambda: veh().get("state") == "MANUAL" and not veh().get("latched"), 10), veh()
    finally:
        fake_fcu.set(gps_fix=3)
        sim.stop.set()
        getattr(sim, "dock_client", None) and sim.dock_client.loop_stop()
        vc.stop()
        probe.stop()
        mm.kill()


# ---- (d) RunStart by hand with rc-test ---------------------------------------------------------------------------------------------
FAKE_SERVER = textwrap.dedent('''
    import logging
    log = logging.getLogger("fake-rc")
    run_declarations = {}
    published = []
    def start_run(client, team_id, pending):
        pending["started"] = True
        client.publish("robocommand/robotx/%s/command" % team_id, b"RunStart")
    def handle_heartbeat(client, team_id, vid, auto):
        p = run_declarations.get(team_id)
        if p is None or p["started"]:
            return
        if auto:
            p["auto_vehicles"].add(vid)
        if p["auto_vehicles"] == p["vehicle_ids"]:
            start_run(client, team_id, p)
    def main():
        return "main ran"
''')


class _Client:
    def __init__(self):
        self.pubs = []

    def publish(self, topic, payload, *a, **k):
        self.pubs.append((topic, payload))


def test_rctest_manual_replaces_only_the_auto_runstart(tmp_path, caplog):
    from optiprimex.tools import rctest_manual as RM
    f = tmp_path / "test_server.py"
    f.write_text(FAKE_SERVER)
    mod = RM.load(str(f))
    c = _Client()
    mod.run_declarations["SGIT"] = {"seq": 4, "vehicle_ids": {"USV1"}, "auto_vehicles": set(), "started": False}
    mod.handle_heartbeat(c, "SGIT", "USV1", True)
    assert c.pubs == [("robocommand/robotx/SGIT/command", b"RunStart")]               # RoboNation's behaviour, unpatched
    RM.make_manual(mod)
    c = _Client()
    mod.run_declarations["SGIT"] = {"seq": 5, "vehicle_ids": {"USV1"}, "auto_vehicles": set(), "started": False}
    with caplog.at_level("INFO", logger="fake-rc"):
        mod.handle_heartbeat(c, "SGIT", "USV1", True)
        mod.handle_heartbeat(c, "SGIT", "USV1", True)
    assert c.pubs == [] and mod.run_declarations["SGIT"]["started"] and mod.RUNSTART_MODE == "manual"
    assert [r.getMessage() for r in caplog.records if "[manual-start]" in r.getMessage()] == [RM.MANUAL_LINE.format(team="SGIT", seq=5)]
    assert mod.main() == "main ran"
    assert RM.main([str(tmp_path / "nope.py")]) == 2


RC_REPO = Path(os.environ.get("RC_REPO") or (ROOT.parent / "robocommand"))
RC_SERVER = RC_REPO / "RobotX_2026" / "rc_test" / "test_server.py"


@pytest.mark.skipif(not RC_SERVER.exists(), reason="RoboNation's robocommand checkout not found (RC_REPO)")
def test_rctest_manual_on_roboNations_own_server(tmp_path):
    """The real test_server.py (unmodified): a declaration + an AUTO heartbeat -> no RunStart, the manual line in its log; the
    unpatched module still auto-starts (RCTEST_RUNSTART=auto)."""
    code = textwrap.dedent(f'''
        import sys, json
        from optiprimex.tools import rctest_manual as RM
        import common_pb2
        from robotx import rx_requests_pb2 as Q, rx_reports_pb2 as P
        mod = RM.load({str(RC_SERVER)!r})
        if sys.argv[1] == "manual":
            RM.make_manual(mod)
        class C:
            pubs = []
            def publish(self, topic, payload, *a, **k): self.pubs.append(topic)
        c = C()
        mod.handle_run_declaration(Q.RxRequest(team_id="SGIT", seq=3, run_declaration=Q.RunDeclaration(vehicle_ids=["USV1"])))
        mod.handle_heartbeat(c, P.RxReport(team_id="SGIT", vehicle_id="USV1", heartbeat=P.Heartbeat(state=common_pb2.STATE_AUTO)))
        print(json.dumps(c.pubs))
    ''')
    env = dict(os.environ, LOG_DIR=str(tmp_path / "logs"), PYTHONDONTWRITEBYTECODE="1",
               PYTHONPATH=os.pathsep.join([str(RC_REPO / "RobotX_2026" / "gen" / "python"), str(ROOT)]))
    out = {}
    for mode in ("manual", "auto"):
        p = subprocess.run([sys.executable, "-c", code, mode], capture_output=True, text=True, timeout=60, env=env, cwd=str(tmp_path))
        assert p.returncode == 0, p.stderr[-1500:]
        out[mode] = json.loads(p.stdout.strip().splitlines()[-1])
    assert out["manual"] == [] and out["auto"] == ["robocommand/robotx/SGIT/command"]
    logs = "".join(f.read_text() for f in (tmp_path / "logs").glob("robocommand_*.log"))
    assert "[manual-start] All declared vehicles autonomous for SGIT" in logs and "declaration_seq=3" in logs


def test_run_rctest_defaults_to_the_manual_runstart():
    rc = (ROOT / "scripts" / "run_rctest.sh").read_text()
    assert subprocess.run(["bash", "-n", str(ROOT / "scripts" / "run_rctest.sh")]).returncode == 0
    assert 'RUNSTART="${RCTEST_RUNSTART:-manual}"' in rc
    assert "$PY -m optiprimex.tools.rctest_manual test_server.py" in rc and 'SERVER_CMD="PYTHONPATH=$RCX/gen/python $PY test_server.py"' in rc
    assert rc.index('PY="$KIT/.venv/bin/python"') < rc.index('RUNSTART="${RCTEST_RUNSTART:-manual}"')   # PY first (set -u)
    assert "LOG_DIR=$RUN_DIR $SERVER_CMD 2>&1" in rc
    assert '"${JUDGE_OPEN:-${JUDGE_WEB:-1}}" = "1"' in rc and "always" in rc          # test_v0147_ab's lines stay
    r = subprocess.run(["bash", str(ROOT / "scripts" / "run_rctest.sh")], capture_output=True, text=True, timeout=20,
                       env=dict(os.environ, RCTEST_RUNSTART="sometimes", RC_REPO=str(RC_REPO) if RC_SERVER.exists() else "/nonexistent"))
    assert r.returncode != 0
    sim = (ROOT / "scripts" / "run_sim.sh").read_text()
    assert "RunStart is the judge page's button" in sim and "ocs_cli mission m1t1 --auto" in sim


def test_the_judge_page_says_a_second_runstart_is_ignored():
    from optiprimex.tools import judge_web as JW

    class M:
        declarations = [{"request_seq": 2}]
        run_start = None
    m = M()
    assert JW.start_note(m, "start") == "" and JW.start_note(m, "assist 1 2") == ""
    m.run_start = {"declaration_seq": 2, "run_id": 7002}
    assert "already had its RunStart (run 7002)" in JW.start_note(m, "start") and "start_task" in JW.start_note(m, "start")
    assert JW.start_note(m, "start 3") == ""                                           # another declaration: a real start
    assert "RCTEST_RUNSTART=manual, the default since v0.1.52" in JW.PAGE
