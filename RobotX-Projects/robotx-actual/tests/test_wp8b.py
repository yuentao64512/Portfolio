"""ROADMAP WP8b (v0.1.29): the OCS at the pool - m0t2's points from the OCS (save_point / del_point -> the mission manager ->
usv_ui's points.json, merge-safe with usv_ui; the map digest carries them), the judge page's buttons (rc-tui's commands through
one JudgeSender), and MAIN's run checklist / next step / `clear for run`, the RoboCommand peek and the log severities."""
from __future__ import annotations

import importlib.util
import json
import threading
import urllib.request
from types import SimpleNamespace

import pytest

from optiprimex.tools import judge_web, ocs_web, rc_tui
from optiprimex.usv import points_store

from .conftest import PY, ROOT, Probe, Proc, free_port, make_ocs_config, make_usv_config, wait_until

NS = "team/optiprime/v1"


# ---- points --------------------------------------------------------------------------------------------------------
def test_points_store_and_usv_ui_share_the_file(tmp_path):
    path = str(tmp_path / "points.json")
    assert points_store.check_name("G1") is None and points_store.check_name("a b") and points_store.check_name("x" * 17)
    rec, replaced = points_store.put(path, "A", 1.3001, 103.8002, 90.0, src="ocs-map")
    assert not replaced and rec["from"] == "ocs-map" and points_store.load(path)["A"]["lat"] == 1.3001
    spec = importlib.util.spec_from_file_location("usv_ui_wp8b", ROOT / "usv_ui" / "usv_ui.py")
    ui = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(ui)
    pts = ui.Points(path)
    assert set(pts.data) == {"A"}
    import os
    import time
    time.sleep(0.02)
    points_store.put(path, "B", 1.3002, 103.8003)                       # the OCS writes while usv_ui runs
    os.utime(path, (time.time() + 1, time.time() + 1))                  # (an mtime step even on coarse clocks)
    assert set(pts.data) == {"A", "B"}                                   # usv_ui sees it ...
    pts.data["C"] = {"lat": 1.3003, "lon": 103.8004, "hdg": None}
    pts.save()                                                          # ... and its own save keeps it
    assert set(points_store.load(path)) == {"A", "B", "C"}
    assert points_store.delete(path, "B") and not points_store.delete(path, "B")
    cache = points_store.Cache(path)
    assert set(cache.get()) == {"A", "C"}
    assert not [f for f in os.listdir(tmp_path) if f.endswith(".tmp")]  # atomic writes leave nothing behind


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    tmp = tmp_path_factory.mktemp("wp8b")
    pts = tmp / "points.json"
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config",
               str(make_usv_config(tmp, brokers, fake_fcu, **{"test_path.points_file": str(pts)}))], tmp / "mm.log")
    ocs = Proc([PY, "-m", "optiprimex.ocs.ocs_app", "--config", str(make_ocs_config(tmp, brokers))], tmp / "ocs.log")
    probe = Probe(brokers.hub_port, [f"{NS}/#", "ocs/#"]).start()
    assert probe.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] in ("MANUAL", "AUTO_HOLD"), 15, include_past=True), mm.log()
    from optiprimex.tools.ocs_cli import OcsConsole
    con = OcsConsole(port=brokers.hub_port)
    con.start()
    yield {"mm": mm, "ocs": ocs, "probe": probe, "con": con, "pts": pts}
    con.stop(); probe.stop()
    for p in (ocs, mm):
        p.kill()


def test_save_point_from_the_ocs_reaches_the_file_and_the_map(stack):
    from optiprimex.tools import ocs_cli
    con, p, pts = stack["con"], stack["probe"], stack["pts"]

    def run(*argv):
        code, lines = ocs_cli.execute(con, ocs_cli.parse(list(argv) + ["--wait", "5"]))
        return code, " | ".join(lines)
    code, out = run("save_point", "EN")                                  # the boat's own fix (the fake FCU has one)
    assert code == 0 and "DONE" in out, out
    rec = json.loads(pts.read_text())["EN"]
    assert rec["from"] == "ocs-boat" and abs(rec["lat"] - 1.2810202) < 0.01 and rec["sats"]
    code, out = run("save_point", "G1", "--lat", "1.2811202", "--lon", "103.85506")    # the map's drop point
    assert code == 0 and json.loads(pts.read_text())["G1"] == dict(json.loads(pts.read_text())["G1"], lat=1.2811202, lon=103.85506, **{"from": "ocs-map"})
    code, out = run("save_point", "bad name!")
    assert code != 0 and "point name" in out, out
    assert p.wait(lambda m: m["topic"].endswith("telemetry/usv/map") and set(m["env"]["body"].get("points") or {}) == {"EN", "G1"}, 10), "no points in the map digest"
    m = p.last(f"{NS}/telemetry/usv/map")["env"]["body"]
    assert abs(m["points"]["G1"][0] - 11.1) < 0.5 and m["points_ll"]["G1"]["from"] == "ocs-map"     # 0.0001 deg north of the origin
    code, out = run("del_point", "EN")
    assert code == 0 and set(json.loads(pts.read_text())) == {"G1"}
    code, out = run("del_point", "EN")
    assert code != 0 and "no point EN" in out, out
    code, out = run("clear_map")                                         # v0.1.29.1: no task -> the boat's map forgets its detections
    assert code == 0 and "map cleared" in out and "saved points stay" in out, out
    assert set(json.loads(pts.read_text())) == {"G1"}


# ---- the judge page's buttons ----------------------------------------------------------------------------------------
class Cap:
    def __init__(self):
        self.pubs = []

    def publish(self, topic, payload, qos=0, retain=False):
        self.pubs.append((topic, payload, retain))


def serve(model, sender):
    srv = judge_web.Server(("127.0.0.1", free_port()), judge_web.make_handler(model, sender))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{srv.server_address[1]}"


def post(url, body, headers):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST", headers={"Content-Type": "application/json", **headers})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_judge_page_buttons_use_rc_tuis_sender():
    m = rc_tui.RcModel("SGIT")
    cap = Cap()
    sender = rc_tui.JudgeSender(cap, m, "fake", "SGIT")
    srv, url = serve(m, sender)
    try:
        H = {"X-Judge-Web": "1"}
        assert post(url + "/cmd", {"line": "allclear"}, {})[0] == 403                          # a cross-site form cannot
        code, r = post(url + "/cmd", {"line": "assist 1.31 103.81 air"}, H)
        assert r["ok"] and cap.pubs[-1][0] == "rc/sim/inject" and json.loads(cap.pubs[-1][1])["domain"] == "air"
        assert not post(url + "/cmd", {"line": "start"}, H)[1]["ok"]                            # no declaration seen yet
        assert not post(url + "/cmd", {"line": "course"}, H)[1]["ok"]                           # only --as-judge publishes a course
        st = json.loads(urllib.request.urlopen(url + "/state.json", timeout=5).read())
        assert st["mode"] == "fake" and st["has_readiness"] is False and "keepout" in st["help"]
        sender.mode = "monitor"
        n = len(cap.pubs)
        assert not post(url + "/cmd", {"line": "allclear"}, H)[1]["ok"] and len(cap.pubs) == n
        sender.mode, sender.wire = "judge", "json"
        assert post(url + "/cmd", {"line": "keepout 1.3 103.8 15"}, H)[1]["ok"] and cap.pubs[-1][0] == "rc/command/SGIT"
    finally:
        srv.shutdown()
    page = judge_web.PAGE
    assert "JUDGE ACTIONS" in page and "X-Judge-Web" in page and all(f"w:'{w}'" in page for w in ("course", "start", "assist", "confirm", "keepout", "allclear", "moving"))
    run = (ROOT / "scripts" / "run_judge.sh").read_text()
    assert "judge_web --host $HOST --port $PORT --team $TEAM $MODE $WIRE" in run


# ---- MAIN: checklist, next step, clear for run, the peek, severities ----------------------------------------------------
def snap(r="R2", state="MANUAL", armed=True, ui=False, judge=True, mission=None, stalled=False, perception=0.5):
    return {"hub_up": True, "status_age": 0.4, "ui_active": ui, "perception_age": perception,
            "ocs": {"r": r, "judge_connected": judge, "judge": "fake", "course_id": "C1", "stalled": stalled, "mission": mission or {}},
            "status": {"state": state, "armed": armed, "fcu_link": "real"}}


def test_the_run_checklist_and_its_next_step():
    c = ocs_web.clearance(snap())
    assert c["clear"] and c["next"] == "declare" and {x["id"] for x in c["checks"]} >= {"hub", "boat", "judge", "armed", "handover", "mission"}
    c = ocs_web.clearance(snap(armed=False, ui=True))
    assert not c["clear"] and c["blocked"] == ["FCU armed (by you)", "usv_ui handed over"]
    assert ocs_web.clearance(snap(r="R3"))["next"] == "enable_auto"
    c = ocs_web.clearance(snap(r="R4", state="AUTO_HOLD"))
    assert c["next"] is None and "RunStart" in c["why"]
    assert ocs_web.clearance(snap(r="R4", state="AUTO_HOLD", stalled=True))["next"] == "declare"
    assert ocs_web.clearance(snap(r="R5", state="AUTO_HOLD", mission={"code": "m0t2"}))["next"] == "start_task"
    c = ocs_web.clearance(snap(r="R5", state="AUTO_HOLD"))
    assert c["next"] is None and "preselect" in c["why"]
    soft = {x["id"]: x for x in ocs_web.clearance(snap(perception=None))["checks"]}
    assert soft["perception"]["ok"] is False and not soft["perception"]["hard"] and ocs_web.clearance(snap(perception=None))["clear"]


def test_clear_for_run_refuses_until_clear_and_the_peek_reads_the_judge():
    web = ocs_web.Web(ocs_web.parse_args([]), env={}, feeds=False)
    web.m.on_hub("ocs/state", json.dumps({"body": {"r": "R2", "judge_connected": True, "course_id": "C1", "vehicles": {}}}).encode())
    web.m.hub_up = True
    r = web.clear_for_run()
    assert not r["ok"] and "boat answers" in r["why"] and "FCU armed" in r["why"]
    ran = []
    web.command = lambda line: ran.append(line)
    web.state = lambda: {"clearance": {"blocked": []}}
    assert web.clear_for_run()["ok"] and wait_until(lambda: ran == ["ready"], 2)
    pk = web.peek.m
    import time
    now = time.time()
    pk.declarations.append({"request_seq": 3, "vehicle_ids": ["USV1"], "_t": now - 40})
    s = web.peek.summary(now)
    assert s["answer_style"] == "bad" and "no RunStart for 40 s" in s["answer"]
    pk.run_start = {"declaration_seq": 3, "run_id": 7003, "_t": now - 38}
    pk.received.append((now - 40, "RunDeclaration seq 3: USV1"))
    pk.sent.append((now - 38, "RxCommand RunStart run 7003 REFUSED? no"))
    s = web.peek.summary(now)
    assert s["answer_style"] == "ok" and "RunStart run 7003 for declaration 3 after 2.0 s" in s["answer"]
    assert [x["dir"] for x in s["feed"]] == ["to", "from"]
    web2 = ocs_web.Web(ocs_web.parse_args([]), env={}, feeds=False)              # a page started after the declaration
    web2.m.on_hub("ocs/state", json.dumps({"body": {"r": "R5", "run_id": 1001, "declaration_seq": 1, "vehicles": {}}}).encode())
    assert "run 1001 running" in web2.state()["judge"]["answer"]
    assert ocs_web.severity("ack REJECTED START_TASK") == "bad" and ocs_web.severity("vehicle AUTO_SAFE_HOLD") == "warn" \
        and ocs_web.severity("intent accepted") == "ok" and ocs_web.severity("course C1") == ""


def test_points_reach_main_and_the_page_has_the_new_parts():
    web = ocs_web.Web(ocs_web.parse_args([]), env={}, feeds=False)
    web.m.map = {"points": {"G1": [11.1, 0.5], "EN": [0.0, 0.0]}, "points_ll": {"G1": {"lat": 1.3001, "lon": 103.8, "from": "ocs-map"}}}
    pts = web._points()
    assert [p["name"] for p in pts] == ["EN", "G1"] and pts[1]["from"] == "ocs-map" and pts[0]["lat"] is None
    meta = web._meta
    assert ("POINTS", ["save_point", "del_point", "clear_map"]) in [tuple(g) for g in meta["groups"]] and {"save_point", "del_point"} <= {u["word"] for u in meta["usage"]}
    assert meta["word_to_action"]["save_point"] == "SAVE_POINT" and meta["examples"]["del_point"] == "del_point A"     # the page's whole catalogue
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for part in ('id="ck"', 'id="nextbtn"', 'id="clearbtn"', 'id="qsel"', 'id="qctl"', 'id="rc-feed"', "ocs-drop-point", "sev-bad", 'id="lprob"'):
        assert part in html, part
    mp = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert "ocs-drop-point" in mp and "b_drop" in mp
    run = (ROOT / "scripts" / "run_ocs.sh").read_text()
    assert "--judge-host $PEEK_HOST --judge-port $PEEK_PORT" in run
    a = ocs_web.parse_args(["--judge-port", "1885"])
    assert a.judge_port == 1885 and a.judge_host is None


def test_the_map_shown_is_the_one_with_the_detections():
    """v0.1.29.1 (the pool, 2026-11-04): usv_ui's map takes no detections in, yet it was preferred while it published - the buoys
    were in the mission manager's map and invisible. Now the mission manager's wins once it holds detections."""
    from optiprimex.tools import bench_tui as T
    m = T.Model("USV1", "usv")
    def feed(src, body):
        m.on_hub(f"{NS}/telemetry/usv/map", json.dumps({"source": src, "body": body}).encode())
    feed("usv_ui", {"entities": [], "n_ingested": 0, "points": {"H": [0, 0]}})
    feed("mission_manager", {"entities": [], "n_ingested": 0})
    assert m.map_src == "usv_ui"                                                  # nothing detected: usv_ui's (its points, the FCU holder)
    feed("mission_manager", {"entities": [{"kind": "buoy", "colour": "unknown"}], "n_ingested": 543})
    feed("usv_ui", {"entities": [], "n_ingested": 0})
    assert m.map_src == "mission_manager" and len(m.map["entities"]) == 1
    feed("usv_ui", {"entities": [{"kind": "buoy"}], "n_ingested": 9, "cv": {"on": True}})       # usv_ui with `cv on`: its own wins again
    assert m.map_src == "usv_ui"
    html = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    assert "fresh.includes('mission_manager') && det(mm)" in html and 'id="b_refresh"' in html and "ocs-clear-map" in html
    assert "ocs-clear-map" in (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
