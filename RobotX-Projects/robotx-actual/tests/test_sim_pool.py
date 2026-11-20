"""The simulated pool (ROADMAP WP1): world_pool / world_por, fake_perception's lights and camera model into the
PerceptionMap (gates, entry / exit markers, dock line, windows), fake_dock's RED -> GREEN sequence, the consoles' CV
source line, and usv_ui's pathgen / dockto live on the bench (fake FCU, sim CV, fake dock)."""
from __future__ import annotations

import json
import time

import paho.mqtt.client as mqtt
import pytest

from optiprimex.companion.config import Config
from optiprimex.companion.state import LocalPose
from optiprimex.sim import world as W
from optiprimex.sim.fake_dock import DockSim
from optiprimex.sim.fake_perception import FakePerception, parse_args as fp_args
from optiprimex.tools.playbook import cv_source
from optiprimex.usv.perception_map import PerceptionMap, classify_light

from .conftest import PY, ROOT, Probe, Proc, free_udp_port, make_usv_config, wait_until

POOL = ROOT / "profiles" / "bench" / "world_pool.yaml"
POR = ROOT / "profiles" / "bench" / "world_por.yaml"


class Drive:
    """fake_perception's camera model fed with scripted poses, its observations folded into a PerceptionMap (no brokers)."""

    def __init__(self, world, seed=3):
        self.fp = FakePerception(fp_args(["--world", str(world), "--hub-port", "0", "--seed", str(seed)]))
        self.map = PerceptionMap(Config())
        self.t = 1000.0

    def at(self, x, y, hdg, secs, dock_state=None):
        if dock_state is not None:
            self.fp.dock_state = dock_state
        for _ in range(int(secs * 5)):
            self.t += 0.2
            self.fp.pose, self.fp.pose_t = {"x_n": x, "y_e": y, "hdg_deg": hdg, "lat": None, "lon": None}, time.monotonic()
            pose = LocalPose(x, y, hdg, 1.3, 103.8, self.t)
            for obs in self.fp.observe(self.t).values():
                self.map.ingest(obs, pose, now=self.t)
        return self


def test_world_pool_layout():
    w = W.load(str(POOL))
    buoys = [o for o in w["objects"] if o["kind"] == "buoy"]
    assert len(buoys) == 10                                           # handbook 3.3.2: ten buoys
    roles = sorted(o.get("role", o["colour"]) for o in buoys)
    assert roles.count("entry") == 1 and roles.count("exit") == 1 and roles.count("obstacle") == 2
    assert [o["light"] for o in buoys if o.get("role") == "entry"] == ["flash"] and [o["light"] for o in buoys if o.get("role") == "exit"] == ["steady"]
    assert sorted(W.bay_faces(w["objects"])) == [1, 2, 3] and w["dock"]["green_bay"] == 2
    assert len([o for o in w["objects"] if o["kind"] == "window"]) == 6
    xs = [p[0] for p in w["bounds"]]
    ys = [p[1] for p in w["bounds"]]
    assert max(xs) - min(xs) == 50.0 and max(ys) - min(ys) == 25.0  # the 50 x 25 m pool (decided 2026-11-02)
    por = W.load(str(POR))
    assert len([o for o in por["objects"] if o["kind"] == "buoy"]) == 4


def test_lights_and_classification():
    o = {"id": 1, "colour": "red", "light": "flash"}
    seen = [W.observed_colour(o, t / 10.0, 0.0, {"period_s": 2.0, "on_s": 1.0}) for t in range(40)]
    assert seen.count("red") == 20 and seen.count("black") == 20      # 1 s on / 1 s off
    assert W.observed_colour({"colour": "blue", "light": "off"}, 0, 0) == "black"
    assert classify_light({"red": 10, "black": 9}) == ("red", "flash")
    assert classify_light({"blue": 30, "black": 1}) == ("blue", "steady")
    assert classify_light({"black": 25, "red": 2, "green": 1}) == ("black", "off")
    assert classify_light({"green": 2}) == ("green", "?")


def test_sim_cv_on_world_pool_builds_gates_markers_and_dock():
    d = Drive(POOL)
    d.at(-2, -6, 0, 4).at(10, -6, 0, 4).at(20, -5, 0, 4)              # down the channel
    gates = d.map.gates(LocalPose(0, 0, 0, 1.3, 103.8, d.t))
    assert len(gates) == 3, [(g.red.id, g.green.id) for g in gates]
    for g in gates:                                                   # through-heading north: red to starboard (east)
        assert abs(((g.through_hdg_deg + 180) % 360) - 180) < 20 and g.red.y_e > g.green.y_e
    mk = d.map.markers()
    assert mk["entry"] is not None and abs(mk["entry"].x_n - 5.0) < 1.0 and mk["entry"].light == "flash"
    assert mk["exit"] is not None and abs(mk["exit"].x_n - 33.0) < 1.0 and mk["exit"].light == "steady"
    blacks = [e for e in d.map.entities("buoy") if e.colour == "black" and e.n_obs >= 6]
    assert len(blacks) >= 2                                           # the two obstacles
    d.at(36, 5.5, 0, 3)                                               # look at the dock from 9 m
    dock = d.map.dock(LocalPose(36, 5.5, 0, 1.3, 103.8, d.t), now=d.t)
    assert dock and len(dock["faces"]) == 3 and abs(dock["x_n"] - 45.0) < 0.5 and abs(dock["psi_n"] - 180.0) < 10
    greens = [e for e in d.map.entities("dock") if e.colour == "green"]
    assert len(greens) == 1 and abs(greens[0].y_e - 5.5) < 0.5       # bay 2's indicator
    assert not d.map.entities("window")                               # 25 cm windows are not resolved from 9 m (window_max_range_m 8)


def test_sim_cv_on_world_por_two_gates():
    d = Drive(POR).at(0, 0, 0, 4)
    assert len(d.map.gates(LocalPose(0, 0, 0, 1.3, 103.8, d.t))) == 2


def test_fake_dock_red_to_green_sequence():
    sim = DockSim(W.load(str(POOL)), seed=1, auto=True)             # v0.1.48: the dwell is off in the world (dock.auto: false)
    t = 0.0

    def run(secs, pose, aim=None):
        nonlocal t
        for _ in range(int(secs * 10)):
            t += 0.1
            sim.tick(t, pose, aim, 0.05)

    run(5, {"x_n": 42.5, "y_e": 2.0, "hdg_deg": 0})                   # in bay 1 (RED indicator): nothing
    assert sim.phase == "idle"
    run(2, {"x_n": 42.5, "y_e": 5.5, "hdg_deg": 0})                   # bay 2, not long enough
    assert sim.phase == "idle"
    run(2, {"x_n": 42.5, "y_e": 5.5, "hdg_deg": 0})
    st = sim.state(t)
    assert st["phase"] == "lit" and st["window"]["colour"] == "red" and st["window"]["bay"] == 2 and st["window"]["window"] in (1, 2)
    run(5, {"x_n": 42.5, "y_e": 5.5, "hdg_deg": 0}, {"on": True, "bay": 3})   # aimed at the wrong bay
    assert sim.phase == "lit"
    run(4.2, {"x_n": 42.5, "y_e": 5.5, "hdg_deg": 0}, {"on": True, "bay": 2})
    assert sim.phase == "green"
    objs = W.apply_dock_state(sim.objects, sim.state(t))
    lit = [o for o in objs if o["kind"] == "window" and o["lit"]]
    assert len(lit) == 1 and lit[0]["colour"] == "green" and lit[0]["bay"] == 2
    run(5.1, None)
    assert sim.phase == "done"
    sim.command({"cmd": "green_bay", "bay": 3}, t)
    assert sim.phase == "idle" and sim.green_bay == 3
    assert [o["colour"] for o in W.apply_dock_state(sim.objects, sim.state(t)) if o["kind"] == "dock"] == ["red", "red", "green"]
    sim.command({"cmd": "fire"}, t)
    assert sim.phase == "green" and sim.docked
    with pytest.raises(ValueError):
        sim.command({"cmd": "green_bay", "bay": 7}, t)


def test_window_colour_reaches_the_map_close_up():
    sim = DockSim(W.load(str(POOL)), seed=1)
    sim.command({"cmd": "docked"}, 0.0)
    d = Drive(POOL).at(43.5, 5.5, 0, 2, dock_state=sim.state(0.0))
    wins = d.map.entities("window")
    assert len(wins) == 2 and sorted(e.colour_now for e in wins) == ["black", "red"]
    sim.command({"cmd": "fire"}, 1.0)
    d.at(43.5, 5.5, 0, 1, dock_state=sim.state(1.0))
    assert "green" in [e.colour_now for e in d.map.entities("window")]


def test_cv_source_line():
    assert cv_source({"source": "sim", "host": "optiprimeagxorin-desktop", "world": "world_pool"}, 0.5) == ("CV sim (AGX, world_pool)", "warn")
    assert cv_source({"source": "sim", "host": "Dmitrii-M3.local", "world": "world_bench"}, 0.5)[0] == "CV sim (Dmitrii-M3, world_bench)"
    assert cv_source({"detector": "actual_buoy_observer", "ok": True}, 1.0) == ("CV real (actual_buoy_observer)", "ok")
    assert cv_source({"source": "sim"}, 30.0, False) == ("CV none", "dim")
    assert cv_source(None, None, True)[0].startswith("CV cv_publish up")


# ---- live: usv_ui + mission manager on the fake FCU, sim CV + fake dock on world_pool ----------------------------------
@pytest.fixture(scope="module")
def pool(brokers, fake_fcu, tmp_path_factory):
    from .test_usv_ui import UI_PY, Headless, load_usv_ui
    import yaml
    if not UI_PY.exists():
        pytest.skip(f"{UI_PY} not found")
    tmp = tmp_path_factory.mktemp("pool")
    share_port = free_udp_port()
    ui = Headless(load_usv_ui(), [f"udpin:127.0.0.1:{fake_fcu.mav_port}", "--share", f"udpout:127.0.0.1:{share_port}",
                                  "--mqtt", f"127.0.0.1:{brokers.boat_port}", "--points", str(tmp / "points.json"), "--co", str(tmp / "co.json"),
                                  "--log-dir", str(tmp / "logs")])
    ui.start()
    cfg = make_usv_config(tmp, brokers, fake_fcu)
    d = yaml.safe_load(cfg.read_text())
    d["mavlink"]["endpoint"] = f"udpin:127.0.0.1:{share_port}"
    cfg.write_text(yaml.safe_dump(d))
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(cfg)], tmp / "mm.log")
    fp = Proc([PY, "-m", "optiprimex.sim.fake_perception", "--world", str(POOL), "--port", str(brokers.boat_port),
               "--hub-port", str(brokers.hub_port), "--seed", "3"], tmp / "fp.log")
    fd = Proc([PY, "-m", "optiprimex.sim.fake_dock", "--world", str(POOL), "--port", str(brokers.boat_port), "--auto", "on"], tmp / "fd.log")   # dockto sends no report
    boat = Probe(brokers.boat_port, ["sim/#", "local/usv/perception/health"]).start()
    hub = Probe(brokers.hub_port, ["sim/truth"]).start()
    assert wait_until(lambda: ui.app.origin.is_set and ui.app.local() is not None, 15), ui.lines()
    yield {"ui": ui, "fcu": fake_fcu, "boat": boat, "hub": hub, "tmp": tmp, "port": brokers.boat_port, "fp": fp, "mm": mm, "fd": fd}
    boat.stop(); hub.stop(); fd.kill(); fp.kill(); mm.kill(); ui.stop()


def _teleport(fcu, x, y, hdg, ui=None):
    """Move the fake plant; with ui, the cv ingest pauses around the jump (a sighting taken just before it would get the
    pose from just after it and leave a ghost - the real boat never jumps)."""
    if ui is not None:
        ui.cmd("cv off")
    lat, lon = W.WorldFrame({"lat": 1.2810202, "lon": 103.8550097}).to_latlon(x, y)   # v0.1.33: Marina Bay
    fcu.ctrl({"teleport": {"lat": lat, "lon": lon, "hdg": hdg}})
    if ui is not None:
        time.sleep(1.0)
        ui.cmd("cv on")


def test_sim_health_truth_and_pathgen_live(pool):
    ui, fcu = pool["ui"], pool["fcu"]
    assert pool["boat"].wait(lambda m: m["topic"].endswith("perception/health") and m["env"]["body"].get("source") == "sim"
                             and m["env"]["body"].get("world") == "world_pool", 15), pool["fp"].log()
    assert pool["hub"].wait(lambda m: m["topic"] == "sim/truth" and m["env"]["body"]["name"] == "world_pool", 10, include_past=True)
    ui.cmd("cv on")
    for x, y in ((-2, -6), (10, -6), (20, -5)):
        _teleport(fcu, x, y, 0.0, ui)
        time.sleep(4.0)
    ui.cmd("pathgen")
    text = ui.lines(6)
    assert "pathgen: entry #" in text and "3 gates" in text and ", exit #" in text, text
    for name in ("EN", "G1a", "G1", "G1b", "G3", "EX"):
        assert name in ui.app.points.data, (name, sorted(ui.app.points.data))
    p = ui.app.path
    assert p and p["state"] == "planned" and p["anchors"][0] == "EN" and p["anchors"][-1] == "EX" and "WARNING" not in text
    g1 = ui.app.origin.to_local(ui.app.points.data["G1"]["lat"], ui.app.points.data["G1"]["lon"])
    assert abs(g1[0] - 12.0) < 0.8 and abs(g1[1] + 6.0) < 0.8                 # gate 1 midpoint of the world file
    assert ui.app.mm is not None


def test_dockto_live_then_cannon_turns_the_window_green(pool):
    ui, fcu, port = pool["ui"], pool["fcu"], pool["port"]
    _teleport(fcu, 36.0, 5.5, 0.0, ui)                                            # look at the dock (v0.1.34: 9 m off - the real 51 deg camera sees the side bays)
    assert wait_until(lambda: ui.app.cvmap.dock(ui.app.poses.latest(), time.monotonic()) is not None
                      and any(e.colour == "green" for e in ui.app.cvmap.entities("dock")), 10), ui.lines()
    _teleport(fcu, 41.0, 1.0, 0.0, ui)                                        # on the standoff line, abeam bay 1
    ui.cmd("dockto")
    assert "dockto bay 2" in ui.lines(3), ui.lines()
    assert wait_until(lambda: ui.app.dock and ui.app.dock["state"] in ("docked", "aborted"), 120, 0.5), ui.lines(20)
    assert ui.app.dock["state"] == "docked", ui.lines(20)
    st = fcu.state()
    assert abs(st["y_e"] - 5.5) < 1.0 and 43.0 < st["x_n"] < 45.0, (st, ui.lines(25))   # inside bay 2
    boat = pool["boat"]
    assert boat.wait(lambda m: m["topic"] == "sim/dock/state" and m["env"]["body"]["phase"] == "lit", 15, include_past=True), pool["fd"].log()
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="test-cannon", protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", port)
    c.loop_start()
    try:
        t0 = time.monotonic()
        while time.monotonic() - t0 < 6.0 and not boat.last("sim/dock/state")["env"]["body"]["phase"] == "green":
            c.publish("local/usv/cannon/aim", json.dumps({"body": {"on": True, "bay": 2}}))
            time.sleep(0.2)
    finally:
        c.loop_stop()
        c.disconnect()
    assert boat.wait(lambda m: m["topic"] == "sim/dock/state" and m["env"]["body"]["phase"] == "green", 5, include_past=True), pool["fd"].log()
    assert wait_until(lambda: "green" in [e.colour_now for e in ui.app.cvmap.entities("window")], 5), [e.body() for e in ui.app.cvmap.entities("window")]
    assert boat.wait(lambda m: m["topic"] == "sim/dock/state" and m["env"]["body"]["phase"] == "done", 8, include_past=True)


def test_op_tui_where_box_names_the_sim_cv_200x60():
    from optiprimex.tools import bench_tui
    from .test_playbook_tui import ocs
    m = bench_tui.Model("USV1", "usv")
    view = bench_tui.View(m, "USV1")
    m.on_hub("ocs/state", json.dumps({"body": ocs("R2", "IDLE", None, None)}).encode())
    m.hub_up = True
    m.on_boat("local/usv/perception/health", json.dumps({"body": {"ok": True, "source": "sim", "host": "optiprimeagxorin-desktop",
                                                                  "world": "world_pool", "detector": "fake_perception"}}).encode())
    c = bench_tui.Canvas(200, 60)
    view.draw(c, 0, "", "")
    text = "\n".join(c.lines())
    assert "CV sim (AGX, world_pool)" in text, text


def test_sim_cv_imports_need_only_the_agx_venv():
    """run_agx.sh --sim runs on the AGX's small venv (setup_agx.sh: paho-mqtt, pyyaml, jsonschema): the sim must not pull in
    pymavlink, pyserial or anything else (found live 2026-11-02: jsonschema was missing there)."""
    import subprocess
    code = ("import sys\n"
            "for m in ('pymavlink', 'serial', 'numpy', 'curses'): sys.modules[m] = None\n"
            "import optiprimex.sim.fake_perception, optiprimex.sim.fake_dock\n"
            "print('ok')\n")
    r = subprocess.run([PY, "-c", code], cwd=str(ROOT), capture_output=True, text=True, timeout=60)
    assert r.returncode == 0 and "ok" in r.stdout, r.stderr
    setup = (ROOT / "scripts" / "setup_agx.sh").read_text()
    assert "pip\" install paho-mqtt pyyaml jsonschema" in setup
