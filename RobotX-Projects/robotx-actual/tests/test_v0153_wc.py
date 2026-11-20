"""v0.1.53 (WC): the course catalogue (sim/courses.py: ALPHA = rc-test's demo course, EXTRA = ours, the default), the worlds anchored
to a course and moved between courses (world.move_to_course), the fake judge's / rc-test's / the judge page's course, and the SIM
page's "course X received - move the world there / ignore" (simview POST /world, GET /course.json)."""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import textwrap
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from optiprimex.sim import courses as C
from optiprimex.sim import world as W

from .conftest import PY, ROOT, Probe, Proc, free_port, wait_until

BENCH = ROOT / "profiles" / "bench"


def _inside_ring(pt, ring):
    """Point in polygon (x, y lists)."""
    x, y = pt
    inside = False
    for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


# ---- the catalogue ---------------------------------------------------------------------------------------------------------------
def test_the_catalogue_rings_sizes_and_ids():
    assert C.DEFAULT == "EXTRA" and set(C.COURSES) == {"ALPHA", "EXTRA"}            # no invented Bravo / Charlie / Delta
    for c in C.COURSES.values():
        r = c.ring()
        assert len(r) == 5 and r[0] == r[-1] and len(c.corners) == 4                   # closed for RxCourse, open in the catalogue
        t = c.transit_ring()
        assert len(t) == 5 and t[0] == t[-1]
        ne = [c.uv_to_ne(*c.uv_of(*p)) for p in c.corners]
        sides = sorted(round(math.dist(ne[i], ne[i - 1]), 1) for i in range(4))
        assert sides[0] == pytest.approx(c.width_m, abs=0.2) and sides[-1] == pytest.approx(c.length_m, abs=0.2)
        assert c.body()["boundary"][0] != c.body()["boundary"][-1] and c.body()["course_id"] == c.id
        assert 25000 <= c.pinger_hz <= 40000                                           # handbook 2293
    a, e = C.ALPHA, C.EXTRA
    assert [tuple(p) for p in a.corners] == list(C.marina.COURSE_ALPHA) and a.pinger_hz == 25000 and a.world == "marina"
    assert (e.length_m, e.width_m) == (120.0, 80.0) and e.axis_deg == pytest.approx(a.axis_deg) and e.world == "extra"
    assert a.axis_deg == pytest.approx(32.08, abs=0.01)
    assert set(e.tasks) >= {"T1", "T2", "T3", "T4", "UAV"}
    for p in a.corners:                                                                 # Extra contains Alpha (shares its south corner)
        assert e.margin_uv(*e.uv_of(*p)) > -0.05
    assert e.corners[1] == a.corners[1]
    for p in e.transit:                                                                 # its transit area: beyond the shore-side edge
        u, v = e.uv_of(*p)
        assert 40.0 - 0.05 <= v <= 60.05 and -60.05 <= u <= -19.95
    assert C.get("alpha") is a and C.get("COURSE-ALPHA") is a and C.get("course_alpha") is a and C.get("Extra") is e and C.get("bravo") is None
    assert C.identify(a.ring()) == "ALPHA" and C.identify(list(reversed(e.corners))) == "EXTRA" and C.identify([[1, 2], [1, 3], [2, 3]]) is None
    # a course we have never seen: the rectangle fitted to it
    pts = [list(e.latlon(u, v)) for u, v in ((-20, -10), (-20, 10), (30, 10), (30, -10))]
    f = C.from_body({"course_id": "REAL-ALPHA", "boundary": pts + [pts[0]], "pinger_hz": 27000})
    assert f.id == "REAL-ALPHA" and f.source == "received" and f.length_m == pytest.approx(50, abs=0.1) and f.width_m == pytest.approx(20, abs=0.1)
    assert e.uv_of(*f.centre) == pytest.approx((5.0, 0.0), abs=0.05) and f.axis_deg % 180 == pytest.approx(e.axis_deg % 180, abs=0.05)
    assert C.from_body({"boundary": e.ring()}) is e and C.from_body({"boundary": []}) is None
    out = subprocess.run([PY, "-m", "optiprimex.sim.courses"], capture_output=True, text=True, cwd=str(ROOT), timeout=30).stdout
    assert "EXTRA" in out and "ALPHA" in out and "120.0 x  80.0" in out


def test_every_element_inside_its_course_with_margins():
    e = C.EXTRA
    w = W.load(str(BENCH / "world_extra.yaml"))
    assert w["origin"] == {"lat": round(e.centre[0], 7), "lon": round(e.centre[1], 7)} and w["bounds"] == e.corners_ne() and w["walls"] is False
    assert w["course"] == "EXTRA" and W.anchor_course(w) is e
    kinds = {o["kind"] for o in w["objects"]}
    assert kinds >= {"buoy", "dock", "window", "pontoon", "box", "pipeline", "platform", "uav_pad"}
    assert len([o for o in w["objects"] if o["kind"] == "buoy" and o.get("task") != "T2"]) == 10                 # T1's ten (1376)
    transit = [list(e.uv_of(*p)) for p in e.transit]
    for o in w["objects"]:
        u, v = e.ne_to_uv(o["x_n"], o["y_e"])
        if o["kind"] == "uav_pad":
            assert not e.contains_uv(u, v) and _inside_ring((u, v), transit)              # the pad: in the transit area (2352)
            continue
        assert e.margin_uv(u, v) >= C.MARGIN_M - 1e-6, (o["id"], u, v)
        assert o["kind"] not in ("pipeline", "platform", "pontoon") or o.get("sensed") is False   # truth only: no perception class
    st = w["start"]
    assert e.margin_uv(*e.ne_to_uv(st["x_n"], st["y_e"])) >= C.MARGIN_M and st["hdg_deg"] == pytest.approx(e.axis_deg, abs=0.01)
    for t in w["teleports"]:
        assert e.contains_uv(*e.ne_to_uv(t["x_n"], t["y_e"]), C.MARGIN_M), t["name"]
    assert {t["name"].split(":")[0] for t in w["teleports"]} >= {"T1", "T2", "T3", "T4", "boxes"}            # a start point per mission
    # T1: GREEN to port, RED to starboard going up the course (1386, 1388); the dock faces look back down the course; bay 1 = left
    gates = {}
    for o in w["objects"]:
        if o.get("gate"):
            gates.setdefault(o["gate"], {})[o["colour"]] = e.ne_to_uv(o["x_n"], o["y_e"])[1]
    assert all(g["green"] < g["red"] and 2.5 <= g["red"] - g["green"] <= 3.5 for g in gates.values()) and len(gates) == 3
    docks = sorted((o for o in w["objects"] if o["kind"] == "dock"), key=lambda o: o["bay"])
    assert all(o["face_hdg_deg"] == pytest.approx((e.axis_deg + 180) % 360, abs=0.01) for o in docks)
    assert [round(e.ne_to_uv(o["x_n"], o["y_e"])[1], 1) for o in docks] == [16.5, 20.0, 23.5]
    nn, ne_ = W.dock_normal(w["objects"], (st["x_n"], st["y_e"]))                                    # the bays open toward the start
    assert e.ne_to_uv(nn, ne_)[0] == pytest.approx(-1.0, abs=0.01)
    # Course Alpha's own world (the compact v0.1.33 layout): inside Alpha, >= 1 m
    m = W.load(str(BENCH / "world_marina.yaml"))
    a = C.ALPHA
    assert W.anchor_course(m) is a
    for o in m["objects"]:
        assert a.margin_uv(*a.ne_to_uv(o["x_n"], o["y_e"])) >= 1.0, o["id"]
    # the fake FCU's start for a course world (run_bench.sh), none for the others
    sa = C.start_args(str(BENCH / "world_extra.yaml")).split()
    lat, lon = W.WorldFrame(w["origin"]).to_latlon(st["x_n"], st["y_e"])
    assert sa[0] == "--lat" and float(sa[1]) == pytest.approx(lat, abs=1e-7) and float(sa[3]) == pytest.approx(lon, abs=1e-7) and sa[4:] == ["--hdg", "32.1"]
    assert C.start_args(str(BENCH / "world_pool.yaml")) == "" and C.start_args(str(BENCH / "world_marina.yaml")) == ""


def _dists(w):
    pts = [(o["x_n"], o["y_e"]) for o in w["objects"]] + [(w["start"]["x_n"], w["start"]["y_e"])]
    return [math.dist(p, q) for i, p in enumerate(pts) for q in pts[i + 1:]]


def test_reanchoring_keeps_the_relative_geometry(tmp_path):
    e, a = C.EXTRA, C.ALPHA
    for name, to, force in (("pool", e, False), ("marina", e, False), ("extra", a, True), ("boxes", e, False), ("extra", e, False)):
        w = W.load(str(BENCH / f"world_{name}.yaml"))
        m, rep = W.move_to_course(w, to)
        assert rep["to"] == to.id and m["anchor"] == {"course": to.id} and m["course_id"] == to.id
        assert m["origin"] == {"lat": round(to.centre[0], 7), "lon": round(to.centre[1], 7)}
        assert max(abs(x - y) for x, y in zip(_dists(w), _dists(m))) < 0.01, name                  # every distance kept
        d = rep["rotation_deg"]
        for o0, o1 in zip(w["objects"], m["objects"]):
            if "face_hdg_deg" in o0:
                assert (o1["face_hdg_deg"] - o0["face_hdg_deg"] - d) % 360 == pytest.approx(0, abs=0.02)
        assert rep["fits"] is (not force), (name, rep)
        p = W.dump(m, str(tmp_path / f"{name}@{to.id}.yaml"))
        back = W.load(p)                                                                            # the run file reloads the same
        assert all(math.dist((o["x_n"], o["y_e"]), (q["x_n"], q["y_e"])) < 0.01 for o, q in zip(m["objects"], back["objects"]))
        assert back["camera"].get("fov_deg") == pytest.approx(w["camera"].get("fov_deg"), abs=0.01) if w["camera"].get("intrinsics") else True
    # the pool moved into Extra: its walls move with it (still walls), its layout centred on the course, its x_n along the axis
    pool = W.load(str(BENCH / "world_pool.yaml"))
    m, rep = W.move_to_course(pool, e)
    assert rep["rotation_deg"] == pytest.approx(e.axis_deg, abs=0.01) and m.get("walls", True) and m["bounds_course"] is False
    uv = [e.ne_to_uv(*p) for p in m["bounds"]]
    assert sorted(round(abs(p[0]), 1) for p in uv) == [25.0] * 4 and sorted(round(abs(p[1]), 1) for p in uv) == [12.5] * 4
    # the marina moved into Extra: its bounds (Alpha's corners) become Extra's corners; moved again it stays consistent
    m, _ = W.move_to_course(W.load(str(BENCH / "world_marina.yaml")), e)
    assert m["bounds"] == e.corners_ne() and m["bounds_course"] is True
    m2, rep2 = W.move_to_course(m, a)
    assert m2["bounds"] == a.corners_ne() and rep2["fits"] and rep2["rotation_deg"] == pytest.approx(0, abs=0.01)
    # Extra into Alpha does not fit: the report names the elements outside
    _, rep = W.move_to_course(W.load(str(BENCH / "world_extra.yaml")), a)
    assert not rep["fits"] and 1 in rep["outside"] and 71 not in rep["outside"]


# ---- the fake judge ------------------------------------------------------------------------------------------------------------
def test_the_fake_judges_course_selection():
    from optiprimex.sim import fake_robocommand as F
    fr = F.FakeRoboCommand.__new__(F.FakeRoboCommand)
    fr.a = F.parse_args([])
    c = fr.course()
    assert C.same_ring(c["boundary"], C.EXTRA.corners) and c["course_id"] == "BENCH-COURSE-1" and c["pinger_hz"] == 30000   # the default
    fr.a = F.parse_args(["--course", "alpha", "--world", str(BENCH / "world_pool.yaml")])
    c = fr.course()
    assert c["course_id"] == "ALPHA" and [tuple(p) for p in c["boundary"]] == list(C.marina.COURSE_ALPHA) and c["pinger_hz"] == 25000
    fr.a = F.parse_args(["--course", "world", "--world", str(BENCH / "world_pool.yaml")])
    assert fr.course()["course_id"] == "SIM-POOL"                                                    # v0.1.46.1's way
    fr.a = F.parse_args(["--world", str(BENCH / "world_extra.yaml")])
    assert fr.course()["course_id"] == "EXTRA" and F.world_course(str(BENCH / "world_extra.yaml"))[2] == "EXTRA"
    pubs, logs = [], []
    fr.pb, fr.log = None, lambda k, b: logs.append((k, b))
    fr.c = type("Cl", (), {"publish": lambda self, *a, **k: pubs.append((a, k))})()
    fr.inject({"kind": "course", "course": "alpha"})
    assert fr.a.course == "ALPHA" and pubs[-1][0][0] == "rc/course" and pubs[-1][1]["retain"] and json.loads(pubs[-1][0][1])["course_id"] == "ALPHA"
    fr.inject({"kind": "course", "course": "bravo"})
    assert fr.a.course == "ALPHA" and logs[-1][0] == "course_refused" and len(pubs) == 1
    rb = (ROOT / "scripts" / "run_bench.sh").read_text()
    assert "fake_robocommand --port 1882 --world $WORLD --course $RC_COURSE" in rb and 'RC_COURSE="${COURSE:-extra}"' in rb
    assert "$START_ARGS $FCU_ARGS" in rb and "optiprimex.sim.courses --start-args" in rb
    for sh in ("run_bench.sh", "run_sim.sh", "sim_world.sh", "run_rctest.sh"):
        assert subprocess.run(["bash", "-n", str(ROOT / "scripts" / sh)]).returncode == 0, sh


def test_the_fake_judge_publishes_the_selected_course_live(brokers, tmp_path):
    port = brokers.hub_port
    rc = Proc([PY, "-m", "optiprimex.sim.fake_robocommand", "--port", str(port), "--course", "alpha", "--world", str(BENCH / "world_pool.yaml")],
              tmp_path / "rc.log")
    probe = Probe(port, ["rc/course"]).start() if hasattr(Probe, "start") else None
    try:
        def last():
            with probe._lock:
                ms = [m for m in probe.msgs if m["topic"] == "rc/course" and m["env"]]
            return ms[-1]["env"] if ms else None
        assert wait_until(lambda: (last() or {}).get("course_id") == "ALPHA", 10), rc.log()[-800:]
        import paho.mqtt.publish as publish
        publish.single("rc/sim/inject", json.dumps({"kind": "course", "course": "extra"}), hostname="127.0.0.1", port=port)
        assert wait_until(lambda: (last() or {}).get("course_id") == "EXTRA", 10), rc.log()[-800:]
        assert C.same_ring(last()["boundary"], C.EXTRA.corners) and last()["pinger_hz"] == 30000
        late = Probe(port, ["rc/course"]).start()                                                    # retained: a late OCS gets EXTRA
        try:
            assert wait_until(lambda: any((m["env"] or {}).get("course_id") == "EXTRA" for m in late.msgs), 5)
        finally:
            late.stop()
    finally:
        probe.stop()
        rc.kill()


# ---- the judge page ----------------------------------------------------------------------------------------------------------------
class _Cap:
    def __init__(self):
        self.pubs = []

    def publish(self, topic, payload, qos=0, retain=False):
        self.pubs.append((topic, payload, retain))


def test_the_judge_pages_course_select():
    from optiprimex.tools import judge_web as J
    from optiprimex.tools import rc_tui
    from optiprimex.ocs import rc_wire
    p = J.PAGE
    assert 'id="crs-sel"' in p and 'id="crs-pub"' in p and "fetch('/courses.json'" in p and "fetch('/course',{method:'POST'" in p
    m = rc_tui.RcModel("SGIT")
    cap = _Cap()
    sender = rc_tui.JudgeSender(cap, m, "fake", "SGIT")
    srv = J.Server(("127.0.0.1", free_port()), J.make_handler(m, sender))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{srv.server_address[1]}"

    def post(path, body, headers):
        req = urllib.request.Request(url + path, data=json.dumps(body).encode(), method="POST", headers={"Content-Type": "application/json", **headers})
        try:
            with urllib.request.urlopen(req, timeout=5) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())
    try:
        cs = json.loads(urllib.request.urlopen(url + "/courses.json", timeout=5).read())
        assert cs["default"] == "EXTRA" and [c["id"] for c in cs["courses"]] == ["EXTRA", "ALPHA"] and cs["mode"] == "fake"
        assert post("/course", {"course": "alpha"}, {})[0] == 403                                   # no header: refused
        code, r = post("/course", {"course": "alpha"}, {"X-Judge-Web": "1"})
        assert r["ok"] and cap.pubs[-1][0] == "rc/sim/inject" and json.loads(cap.pubs[-1][1]) == {"kind": "course", "course": "ALPHA"}
        assert not post("/course", {"course": "bravo"}, {"X-Judge-Web": "1"})[1]["ok"]
        sender.mode, sender.wire = "judge", "protobuf"                                              # --as-judge (rc-test): the RxCourse itself
        r = post("/course", {"course": "extra"}, {"X-Judge-Web": "1"})[1]
        topic, payload, retain = cap.pubs[-1]
        crs = rc_wire.decode_course(payload)
        assert r["ok"] and topic == rc_wire.t_course() and retain and crs["course_id"] == "EXTRA" and crs["boundary"][0] == crs["boundary"][-1]
        assert C.same_ring(crs["boundary"], C.EXTRA.corners) and crs["pinger_hz"] == 30000 and "RCTEST_COURSE=extra" in r["text"]
        sender.wire = "json"
        assert post("/course", {"course": "alpha"}, {"X-Judge-Web": "1"})[1]["ok"] and cap.pubs[-1][0] == "rc/course" and cap.pubs[-1][2]
        sender.mode = "monitor"
        n = len(cap.pubs)
        assert not post("/course", {"course": "alpha"}, {"X-Judge-Web": "1"})[1]["ok"] and len(cap.pubs) == n
    finally:
        srv.shutdown()


# ---- rc-test through rctest_manual -----------------------------------------------------------------------------------------------------
STAND_IN = textwrap.dedent('''
    import json, logging, os
    log = logging.getLogger("stand-in-rc")
    run_declarations = {}
    COURSE_ALPHA = [(1.0, 2.0), (1.0, 3.0), (2.0, 3.0), (2.0, 2.0), (1.0, 2.0)]
    TRANSIT_A = []
    def start_run(client, team_id, pending):
        pending["started"] = True
    def on_message(client, userdata, msg):
        pass
    def main():
        with open(os.environ["STAND_IN_OUT"], "w") as f:
            json.dump({"course": COURSE_ALPHA, "transit": TRANSIT_A, "id": os.environ.get("COURSE_ID"), "hz": os.environ.get("PINGER_FREQ_HZ"),
                       "follows": on_message.__name__ == "on_message" and on_message.__module__ != __name__}, f)
''')


def test_rctest_manual_sets_the_course_on_a_stand_in(tmp_path):
    f = tmp_path / "test_server.py"
    f.write_text(STAND_IN)
    for course, want in (("extra", C.EXTRA), ("alpha", C.ALPHA)):
        out = tmp_path / f"{course}.json"
        env = {k: v for k, v in os.environ.items() if k not in ("COURSE_ID", "PINGER_FREQ_HZ", "RCTEST_RUNSTART")}
        env.update(RCTEST_COURSE=course, STAND_IN_OUT=str(out), PYTHONPATH=str(ROOT))
        p = subprocess.run([PY, "-m", "optiprimex.tools.rctest_manual", str(f)], capture_output=True, text=True, timeout=60, env=env, cwd=str(tmp_path))
        assert p.returncode == 0, p.stderr[-1500:]
        d = json.loads(out.read_text())
        assert [tuple(x) for x in d["course"]] == [tuple(x) for x in want.ring()] and d["id"] == want.id and d["hz"] == str(want.pinger_hz)
        assert [tuple(x) for x in d["transit"]] == [tuple(x) for x in want.transit_ring()] and d["follows"]
    env.update(RCTEST_COURSE="bravo")
    p = subprocess.run([PY, "-m", "optiprimex.tools.rctest_manual", str(f)], capture_output=True, text=True, timeout=60, env=env, cwd=str(tmp_path))
    assert p.returncode != 0 and "RCTEST_COURSE='bravo'" in p.stderr
    rc = (ROOT / "scripts" / "run_rctest.sh").read_text()
    assert 'COURSE="$(echo "${RCTEST_COURSE:-extra}"' in rc and "RCTEST_COURSE=$COURSE LOG_DIR=$RUN_DIR $SERVER_CMD 2>&1" in rc
    assert 'SERVER_CMD="RCTEST_RUNSTART=auto PYTHONPATH=$RCX/gen/python:$KIT $PY -m optiprimex.tools.rctest_manual test_server.py"' in rc


def _rc_repo():
    for c in (os.environ.get("RC_REPO"), str(ROOT.parent / "robocommand"), str(ROOT / "robotx-uav-robocommand" / "robocommand")):
        if c and (Path(c) / "RobotX_2026" / "rc_test" / "test_server.py").exists():
            return Path(c)
    return None


RC = _rc_repo()


@pytest.mark.skipif(RC is None, reason="RoboNation's robocommand checkout not found (RC_REPO)")
def test_rctest_manual_course_on_roboNations_own_server(tmp_path):
    """The real test_server.py, unmodified: Extra's rings in its module, a UAV geofence inside Extra but outside Alpha passes its own
    check, and a re-published RxCourse (Alpha) is followed - the same fence is then refused."""
    server = RC / "RobotX_2026" / "rc_test" / "test_server.py"
    code = textwrap.dedent(f'''
        import json, os
        from optiprimex.tools import rctest_manual as RM
        from optiprimex.sim import courses as C
        from robotx import rx_requests_pb2 as Q, rx_course_pb2 as K
        import common_pb2
        mod = RM.load({str(server)!r})
        cid = RM.apply_course(mod, "extra")
        RM.follow_courses(mod)
        e = C.EXTRA
        fence = [e.latlon(u, v) for u, v in ((20, -30), (20, -10), (40, -10), (40, -30), (20, -30))]
        decl = Q.RunDeclaration(vehicle_ids=["USV1", "UAV1"], uav_geofence_max_altitude_hae_m=30,
                                uav_geofence=[common_pb2.LatLng(latitude=a, longitude=b) for a, b in fence])
        p1 = mod.geofence_problems(decl)
        class M: pass
        msg = M(); msg.topic = "robocommand/robotx/course"
        msg.payload = K.RxCourse(course_id="ALPHA", corners=[common_pb2.LatLng(latitude=a, longitude=b) for a, b in C.ALPHA.ring()]).SerializeToString()
        mod.on_message(None, None, msg)
        p2 = mod.geofence_problems(decl)
        print(json.dumps({{"cid": cid, "env": os.environ.get("COURSE_ID"), "p1": p1, "p2": p2, "now": mod.RCTEST_COURSE}}))
    ''')
    env = {k: v for k, v in os.environ.items() if k not in ("COURSE_ID", "PINGER_FREQ_HZ")}
    env.update(LOG_DIR=str(tmp_path / "logs"), PYTHONDONTWRITEBYTECODE="1",
               PYTHONPATH=os.pathsep.join([str(RC / "RobotX_2026" / "gen" / "python"), str(ROOT)]))
    p = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=60, env=env, cwd=str(tmp_path))
    assert p.returncode == 0, p.stderr[-1500:]
    d = json.loads(p.stdout.strip().splitlines()[-1])
    assert d["cid"] == "EXTRA" and d["env"] == "EXTRA" and d["p1"] == [] and d["now"] == "ALPHA"
    assert d["p2"] and "leaves the course" in d["p2"][0]
    logs = "".join(f.read_text() for f in (tmp_path / "logs").glob("robocommand_*.log"))
    assert "[course] following RxCourse ALPHA" in logs


@pytest.mark.skipif(RC is None, reason="RoboNation's robocommand checkout not found (RC_REPO)")
def test_run_rctest_refuses_an_unknown_course():
    env = dict(os.environ, RC_REPO=str(RC), RCTEST_COURSE="bravo", RCTEST_PORT=str(free_port()), RCTEST_SESSION=f"wc-test-{os.getpid()}")
    r = subprocess.run(["bash", str(ROOT / "scripts" / "run_rctest.sh")], capture_output=True, text=True, timeout=30, env=env)
    assert r.returncode == 2 and "RCTEST_COURSE=extra | alpha" in r.stderr


# ---- the SIM page: the banner, move there / ignore, the picker -----------------------------------------------------------------------
def test_simview_course_banner_and_post_world(fake_fcu, tmp_path):
    from optiprimex.sim import simview as SV
    hook_out = tmp_path / "hook.txt"
    hook = tmp_path / "hook.sh"
    hook.write_text(f'#!/bin/sh\necho "$1" > {hook_out}\necho "fake CV on $1"\n')
    hook.chmod(0o755)
    a = SV.parse_args(["--world", "pool", "--fcu-control", str(fake_fcu.ctrl_port), "--port", str(free_port()), "--hub-port", "0",
                       "--width", "160", "--height", "90", "--world-dir", str(tmp_path / "worlds"), "--world-hook", str(hook)])
    sim = SV.Sim(a)
    srv = SV.Server(("127.0.0.1", a.port), SV.make_handler(sim))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    url = f"http://127.0.0.1:{a.port}"

    def get(path):
        return json.loads(urllib.request.urlopen(url + path, timeout=5).read())

    def post(path, body, header=True):
        req = urllib.request.Request(url + path, data=json.dumps(body).encode(), method="POST",
                                     headers={"Content-Type": "application/json", **({"X-Sim-Web": "1"} if header else {})})
        try:
            with urllib.request.urlopen(req, timeout=10) as r:
                return r.status, json.loads(r.read())
        except urllib.error.HTTPError as e:
            return e.code, json.loads(e.read())
    try:
        cv = get("/course.json")
        assert cv["course_id"] == "SIM-POOL" and cv["received"] is None and cv["banner"] is None and "extra" in cv["worlds"]
        assert [c["id"] for c in cv["courses"]] == ["EXTRA", "ALPHA"]
        sim.on_course(C.EXTRA.body(), "hub rc/course")                                          # the judge publishes EXTRA
        cv = get("/course.json")
        assert cv["received"]["match"] == "EXTRA" and cv["received"]["differs"] and "course EXTRA received" in cv["banner"] and "ignore" in cv["banner"]
        assert get("/world.json")["banner"] == cv["banner"]
        assert post("/world", {"ignore": True}, header=False)[0] == 403
        r = post("/world", {"ignore": True})[1]
        assert r["ok"] and r["ignored"] == "EXTRA" and get("/course.json")["banner"] is None and get("/course.json")["received"]["ignored"]
        assert get("/course.json")["course_id"] == "SIM-POOL"                                   # ignore = the pool stays (a trial run)
        # move there: the course's own world, the boat at its start
        r = post("/world", {"course": "EXTRA"})[1]
        assert r["ok"] and r["world"] == "world_extra" and r["course_id"] == "EXTRA" and r["teleport"]["ok"], r
        wj = get("/world.json")
        assert wj["banner"] is None and not wj["received"]["differs"] and wj["course"]["id"] == "EXTRA" and len(wj["course"]["corners_xy"]) == 4
        st = fake_fcu.state()
        x, y = W.WorldFrame(wj["origin"]).to_world(st["lat"], st["lon"])
        assert math.hypot(x - wj["teleports"][0]["x_n"], y - wj["teleports"][0]["y_e"]) < 0.3 and abs(st["hdg_deg"] - C.EXTRA.axis_deg) < 0.5
        assert wait_until(lambda: hook_out.exists(), 5) and hook_out.read_text().strip().endswith("world_extra.yaml")
        assert wait_until(lambda: (sim.hook_result or "").startswith("fake CV on"), 5)
        # a per-mission start point
        r = post("/teleport", {"name": "T3: before the dock"})[1]
        assert r["ok"] and r["to"] == "T3: before the dock"
        # the judge switches to ALPHA: the banner again; Extra's layout does not fit Alpha - refused unless forced
        sim.on_course(C.ALPHA.body(), "the OCS's COURSE")
        assert "course ALPHA received" in get("/course.json")["banner"]
        r = post("/world", {"course": "ALPHA", "keep": True})[1]
        assert not r["ok"] and not r["fits"] and "does not fit" in r["why"]
        r = post("/world", {"course": "ALPHA"})[1]                                               # the course's own world: the marina
        assert r["ok"] and r["world"] == "world_marina" and r["course_id"] == "COURSE-ALPHA" and get("/course.json")["banner"] is None
        # the picker: the pool re-anchored into Extra (a run file), then the pool as it is
        hook_out.unlink()
        r = post("/world", {"course": "EXTRA", "world": "pool", "teleport": False})[1]
        assert r["ok"] and r["path"].endswith("world_pool@EXTRA.yaml") and r["report"]["rotation_deg"] == pytest.approx(C.EXTRA.axis_deg, abs=0.01)
        assert Path(r["path"]).exists() and r["teleport"] is None and wait_until(lambda: hook_out.exists(), 5)
        assert get("/world.json")["course_id"] == "EXTRA" and len(get("/world.json")["bounds"]) == 4
        assert post("/reset", {"confirm": "reset"})[1]["ok"]                                      # RESET SIM reloads the moved file
        assert get("/world.json")["path"] == r["path"]
        r = post("/world", {"world": "pool", "teleport": False})[1]
        assert r["ok"] and r["course_id"] == "SIM-POOL" and r["path"].endswith("world_pool.yaml")
        # a received course not in the catalogue: "move there" still works (the fitted rectangle)
        pts = [list(C.EXTRA.latlon(u, v)) for u, v in ((-30, -20), (-30, 20), (30, 20), (30, -20))]
        sim.on_course({"course_id": "REAL-X", "boundary": pts}, "rc-test RxCourse")
        cv = get("/course.json")
        assert cv["received"]["match"] is None and "course REAL-X received" in cv["banner"]
        r = post("/world", {"course": "received", "keep": True, "teleport": False})[1]
        assert r["ok"] and r["course_id"] == "REAL-X" and get("/course.json")["banner"] is None
        assert not post("/world", {"course": "bravo"})[1]["ok"] and not post("/world", {"world": "nowhere"})[1]["ok"]
        sim.render_once()                                                                        # the moved world renders
        assert sim.jpeg[:2] == b"\xff\xd8"
    finally:
        srv.shutdown()
        fake_fcu.set(mode=4)
    page = (ROOT / "optiprimex" / "sim" / "sim.html").read_text()
    assert 'id="crsbanner"' in page and 'id="crsmove"' in page and 'id="crsign"' in page and 'post("/world",{ignore:true})' in page
    assert 'id="crssel"' in page and 'id="wsel"' in page and "W.course.corners_xy" in page
    sh = (ROOT / "scripts" / "run_sim.sh").read_text()
    assert "--world-hook scripts/sim_world.sh --world-dir $RUN_DIR/worlds $SIM_RC" in sh and "--course) COURSE_ARG=" in sh
