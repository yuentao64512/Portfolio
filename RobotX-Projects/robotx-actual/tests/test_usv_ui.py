"""usv_ui (MASTER_SCRIPTS/usv_ui) on the laptop bench: it holds the FCU link and shares it, the mission
manager runs through the share, the tui/active handover decides who commands, 'cv on' builds the map from the fake
detector and publishes it as telemetry/usv/map. Skipped when the usv_ui folder is missing from the tree."""
from __future__ import annotations

import importlib.util
import math
import threading
import time

import pytest

from optiprimex.tools.vcmd import VehicleCommander

from .conftest import PY, ROOT, Probe, Proc, free_udp_port, make_usv_config, wait_until

NS = "team/optiprime/v1"
UI_PY = ROOT / "usv_ui" / "usv_ui.py"


def load_usv_ui():
    spec = importlib.util.spec_from_file_location("usv_ui", UI_PY)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Headless:
    """usv_ui.App driven without curses: start_links, wait for the heartbeat, then step() in a thread."""

    def __init__(self, mod, argv):
        self.mod = mod
        self.args = mod.build_parser().parse_args(argv)
        self.args.conn, self.args.baud = mod.resolve_link(self.args.conn, self.args.baud)
        self.app = mod.App(self.args)
        self.stop_evt = threading.Event()
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.app.start_links()
        assert self.app.wait_heartbeat(15), "usv_ui: no heartbeat from the fake FCU"
        self.thread.start()

    def _loop(self):
        while not self.stop_evt.is_set() and not self.app.quit:
            self.app.step()
            time.sleep(0.02)

    def cmd(self, line):
        self.app.do_command(line)
        return self.app.log[-1][1] if self.app.log else ""

    def lines(self, n=12):
        return "\n".join(e[1] for e in self.app.log[-n:])

    def stop(self):
        self.stop_evt.set()
        self.thread.join(3)
        self.app.close_links()


@pytest.fixture(scope="module")
def stack(brokers, fake_fcu, tmp_path_factory):
    if not UI_PY.exists():
        pytest.skip(f"{UI_PY} not found")
    tmp = tmp_path_factory.mktemp("ui")
    mod = load_usv_ui()
    share_port = free_udp_port()
    ui = Headless(mod, [f"udpin:127.0.0.1:{fake_fcu.mav_port}", "--share", f"udpout:127.0.0.1:{share_port}",
                         "--mqtt", f"127.0.0.1:{brokers.boat_port}", "--points", str(tmp / "points.json"), "--co", str(tmp / "co.json"),
                         "--log-dir", str(tmp / "logs")])
    ui.start()
    # the mission manager runs THROUGH usv_ui's share, not on the FCU port
    cfg = make_usv_config(tmp, brokers, fake_fcu)
    import yaml
    d = yaml.safe_load(cfg.read_text())
    d["mavlink"]["endpoint"] = f"udpin:127.0.0.1:{share_port}"
    cfg.write_text(yaml.safe_dump(d))
    mm = Proc([PY, "-m", "optiprimex.usv.mission_manager", "--config", str(cfg)], tmp / "mm.log")
    fp = Proc([PY, "-m", "optiprimex.sim.fake_perception", "--world", str(ROOT / "profiles" / "bench" / "world_bench.yaml"), "--port", str(brokers.boat_port),
               "--hub-port", str(brokers.hub_port), "--hz", "5"], tmp / "fp.log")
    hub = Probe(brokers.hub_port, [f"{NS}/#"]).start()
    boat = Probe(brokers.boat_port, ["presence/#", "local/#"]).start()
    vc = VehicleCommander(port=brokers.hub_port)
    vc.start()
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 20, include_past=True), mm.log()
    yield {"ui": ui, "mm": mm, "fp": fp, "hub": hub, "boat": boat, "vc": vc, "fcu": fake_fcu, "tmp": tmp}
    vc.stop(); hub.stop(); boat.stop(); fp.kill(); mm.kill(); ui.stop()


def test_share_carries_the_mission_manager(stack):
    ui, boat = stack["ui"], stack["boat"]
    app = ui.app
    assert app.tel.hb_t is not None and app.share is not None
    # the mission manager's own heartbeats (200/191) came in through the share and went to the FCU
    assert wait_until(lambda: app.share.n_in > 0 and app.share.peer == (200, 191), 10), ui.lines()
    assert app.share.n_out > 50                                              # the FCU stream was relayed
    assert "mission manager" in app.share_line()
    # the mission manager saw the FCU through the share: its local state is on the boat broker
    assert wait_until(lambda: app.mm is not None, 10)
    assert "mm:" in app.mm_line() and "not seen" not in app.mm_line()
    # usv_ui is a node on the boat broker
    assert boat.wait(lambda m: m["topic"] == "presence/usv_ui" and m["env"].get("up") is True, 10, include_past=True)
    # the first log line says which kit runs (the answer to "is this a stale usv_ui?")
    assert app.log[0][1].startswith("usv_ui kit v%s " % ui.mod.KIT_VERSION) and ui.mod.KIT_VERSION != "?"


def test_handover_then_stop_takes_back(stack):
    ui, vc, hub, fcu = stack["ui"], stack["vc"], stack["hub"], stack["fcu"]
    app = ui.app
    assert app.active
    vc.declaration_seq = 1
    assert [a["body"]["result"] for a in vc.send("run", "PREPARE", args={"declaration_seq": 1, "tiers": {"T4": "TIER_CORE"}})][-1] == "DONE"
    # while usv_ui beats tui/active the mission manager refuses to take the FCU
    acks = vc.send("run", "ENABLE_AUTO")
    assert acks and acks[-1]["body"]["result"] == "REJECTED", acks
    assert "usv_ui active" in acks[-1]["body"].get("reason", "") or "override" in acks[-1]["body"].get("reason", "").lower(), acks[-1]["body"]
    # FCU commands here still work (we have the FCU)
    ui.cmd("mode hold")
    # handover -> the beat stops -> after tui_active_s (2 s) the mission manager may go AUTO; we stay in usv_ui
    assert ui.cmd("handover").startswith("HANDED OVER"), ui.lines()
    assert not app.active
    assert ui.cmd("surge 0.2 1").startswith("refused: handed over"), ui.lines()
    assert ui.cmd("release") == "already handed over"                     # the old word is an alias
    assert app.next_hint().startswith("HANDED OVER")
    time.sleep(2.5)
    acks = vc.send("run", "ENABLE_AUTO", wait_s=5)
    assert acks and acks[-1]["body"]["result"] == "DONE", (acks, stack["mm"].log()[-2000:])
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "AUTO_HOLD", 10, include_past=True)
    # 'stop' takes the FCU back at once: the beat resumes, the mission manager steps aside (RC override -> MANUAL)
    assert ui.cmd("stop").startswith("stop") or app.active
    assert app.active
    assert hub.wait(lambda m: m["topic"].endswith("status/usv") and m["env"]["body"]["state"] == "MANUAL", 10)
    acks = vc.send("run", "ENABLE_AUTO")
    assert acks and acks[-1]["body"]["result"] == "REJECTED"
    assert "refused" not in ui.cmd("mode hold")


def test_cv_on_builds_the_map_and_publishes_it(stack):
    ui, hub, fcu, boat = stack["ui"], stack["hub"], stack["fcu"], stack["boat"]
    app = ui.app
    assert wait_until(lambda: app.origin.is_set, 10)                        # CO auto-set at the first fix
    assert ui.cmd("cv on").startswith("cv ON"), ui.lines()
    assert boat.wait(lambda m: "/perception/health" in m["topic"], 10, include_past=True)
    # drive the boat north through the channel with usv_ui's own goto (GUIDED, 2 m/s on the fake plant)
    end_lat, end_lon = app.origin.from_local(46.0, 2.0)
    app.points.data["END"] = {"lat": end_lat, "lon": end_lon, "hdg": 0.0, "utc": "test", "sats": 14, "hdop": 1.0}
    app.points.save()
    assert "refused" not in ui.cmd("goto END 2.0"), ui.lines()
    assert wait_until(lambda: app.local() is not None and app.local()[0] > 44.0, 60), (app.local(), ui.lines())
    truth = __import__("yaml").safe_load((ROOT / "profiles" / "bench" / "world_bench.yaml").read_text())["objects"]

    def mapped() -> bool:
        ents = app.cvmap.entities(include_stale=True)
        return all(any(e.kind == o["kind"] and e.colour == o["colour"] and math.hypot(e.x_n - o["x_n"], e.y_e - o["y_e"]) < 1.0 for e in ents)
                   for o in truth)

    assert wait_until(mapped, 15), [(e.kind, e.colour, round(e.x_n, 1), round(e.y_e, 1)) for e in app.cvmap.entities(include_stale=True)]
    assert len(app.cvmap.entities(include_stale=True)) <= len(truth) + 2        # no entity multiplication while moving
    assert len(app.cvmap.gates(app.poses.latest())) >= 2
    ui.cmd("stop")
    assert "map: new buoy" in ui.lines(60)
    # the map digest goes out on the team namespace with source usv_ui and reaches the hub through the bridge
    m = hub.wait(lambda m: m["topic"].endswith("telemetry/usv/map") and m["env"].get("source") == "usv_ui"
                 and len(m["env"]["body"].get("entities") or []) >= 6, 15, include_past=True)
    assert m, hub.last(f"{NS}/telemetry/usv/map")
    body = m["env"]["body"]
    assert body["boat"] is not None and body["task_state"] == "ACTIVE" and body["cv"]["on"] is True
    assert body["gates"] and body["co"]["lat"] == pytest.approx(app.origin.rec["lat"])
    # the MAP screen renderer shows R/G marks for the entities
    ents, gates = app.map_entities()
    grid, _ = ui.mod.render_map(80, 30, {}, (0.0, 0.0, 0.0), None, [], False, entities=ents, gates=gates)
    chars = "".join(ch for row in grid for ch, _ in row)
    assert "R" in chars and "G" in chars and "|" in chars
    # cv status / off / clear
    face = next(e for e in app.cvmap.ents.values() if e.kind == "dock")      # v0.1.48: the bench's faces are white - one indicator vote
    app.cvmap._vote_indicator(face, "green", time.monotonic())
    ui.cmd("cv status")
    assert "observations rx" in ui.lines(40)
    assert "votes R0 G1" in ui.lines(80)
    assert ui.cmd("cv off").startswith("cv OFF")
    n = len(app.cvmap.ents)
    assert n >= 6
    assert ui.cmd("cv clear") == "cv: map cleared" and not app.cvmap.ents


def test_fcu_switch_live(stack):
    """`fcu real <endpoint>` / `fcu fake`: the link is swapped under the share without restarting; the fake plant is found
    again on udp 14550 (nothing of ours is spawned when a plant already answers); guards: handed over / running / unknown."""
    ui = stack["ui"]
    app = ui.app
    assert app.fcu_kind() == "fake" and app.fake_proc is None
    port = app.fake_port()
    assert ui.cmd("fcu").startswith("fcu: fake link udpin:127.0.0.1:%d, HB" % port)
    assert ui.cmd("fcu fake") == "fcu: already on udpin:127.0.0.1:%d" % port
    assert ui.cmd("fcu bogus").startswith("usage: fcu")
    # to a 'real' endpoint with nothing behind it: heartbeat gone, telemetry reset, the switch waits
    assert ui.cmd("fcu real udpin:0.0.0.0:14577").startswith("fcu: switching to real on udpin:0.0.0.0:14577"), ui.lines()
    assert app.fcu_kind() == "real" and app.await_hb is not None and app.tel.hb_t is None
    time.sleep(0.5)
    assert ui.cmd("surge 0.2 1").startswith(("refused", "bad", "no heartbeat", "surge")) and app.action is None   # nothing to drive
    # and back to the plant that is still sending on 14550: up within 2 s, no child plant of ours
    assert ui.cmd("fcu fake").startswith("fcu: switching to fake")
    assert wait_until(lambda: app.await_hb is None and app.tel.hb_t is not None, 5), ui.lines()
    assert app.fake_proc is None and "fake link up" in ui.lines(60), ui.lines(20)   # 60: earlier cv output can push it out of 12 (v0.1.26)
    assert ui.cmd("handover").startswith("HANDED OVER")
    assert ui.cmd("fcu real").startswith("refused: handed over")
    assert ui.cmd("takeover").startswith("ACTIVE")
    time.sleep(2.5)                                       # the mission manager settles (FCU_LOST cleared by the new link)


def test_reset_and_step_from_usv_ui(stack):
    """`reset` / `step` typed in usv_ui reach the mission manager as RESET / STEP commands on the boat broker and are acked."""
    ui, hub = stack["ui"], stack["hub"]                                            # acks are bridged to the hub
    assert ui.cmd("reset").startswith("RESET sent to the mission manager"), ui.lines()
    m = hub.wait(lambda m: m["topic"].endswith("ack/usv") and str(m["env"]["body"].get("ack_of", "")).startswith("ui-")
                  and m["env"]["body"]["result"] == "DONE", 5)
    assert m and m["env"]["body"]["state"] == "MANUAL"
    assert ui.cmd("step").startswith("STEP sent to the mission manager")
    m = hub.wait(lambda m: m["topic"].endswith("ack/usv") and str(m["env"]["body"].get("ack_of", "")).startswith("ui-")
                  and m["env"]["body"]["result"] == "REJECTED", 5)
    assert m and "TEST_MANUAL" in m["env"]["body"]["reason"]
    assert ui.app.mm is not None and "ui_active" in ui.app.mm[1]                   # the MISSION MANAGER box has what it shows


def test_ocs_link_visibility_events_console_and_presence(stack):
    """v0.1.18: the boat console sees the OCS link (bridge state + ocs/state bridged in), splits EVENTS / CONSOLE, and its presence
    beat says which FCU it holds."""
    import json
    import paho.mqtt.client as mqtt
    ui, boat, brokers_hub = stack["ui"], stack["boat"], stack["hub"]
    app = ui.app
    # the presence beat carries fcu_link / active / log for op-tui's WHERE WE ARE
    assert boat.wait(lambda m: m["topic"] == "presence/usv_ui" and m["env"].get("fcu_link") == "fake" and "active" in m["env"], 10, include_past=True)
    # ocs/state published on the hub arrives on the boat broker through the bridge (topic ocs/state in 0)
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="test-ocs", protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", stack["vc"].port)
    c.loop_start()
    body = {"r": "R5", "r_state": "RUNNING", "run_id": 1001, "declaration_seq": 1, "judge": "fake", "judge_connected": True,
            "mission": {"task": "TEST_AUTO", "auto": True, "started_run": 1001, "armed_run": None}, "vehicles": {"USV1": {"state": "AUTO_HOLD"}}}
    for _ in range(3):
        c.publish("ocs/state", json.dumps({"schema_version": 1, "kind": "telemetry", "vehicle_id": "OCS", "body": body}), qos=0)
        time.sleep(0.3)
    assert wait_until(lambda: app.ocs_body() is not None and app.ocs_body().get("r") == "R5", 8), "ocs/state did not cross the bridge"
    assert (app.ocs_body().get("mission") or {}).get("task") == "TEST_AUTO"
    c.loop_stop(); c.disconnect()
    # the bridge state: $SYS cannot be published by a client, so feed the notification the broker would send
    now = time.monotonic()
    app._on_mqtt(now, "$SYS/broker/connection/ocs/state", {"raw": "0"})
    assert app.bridge is False
    ws = [t for _, t, _ in app.warnings(time.monotonic())]
    assert any(t.startswith("OCS LINK DOWN") for t in ws), ws
    assert any(e[1].startswith("OCS LINK DOWN") and e[2] == "event" for e in app.log[-5:]), ui.lines()
    assert app.ocs_link_text(time.monotonic()).startswith("LINK DOWN")
    app._on_mqtt(time.monotonic(), "$SYS/broker/connection/ocs/state", {"raw": "1"})
    assert app.bridge is True and not any(t.startswith("OCS LINK DOWN") for _, t, _ in app.warnings(time.monotonic()))
    # EVENTS / CONSOLE split: a typed command and its result are kind cmd / result, system lines are events
    ui.cmd("share")
    kinds = [e[2] for e in app.log[-4:]]
    assert "cmd" in kinds and "result" in kinds and app.log[-1][2] == "result"
    assert any(e[1].startswith("> share") for e in app.log[-4:])
    assert all(k in ("event", "cmd", "result", "mav") for k in (e[2] for e in app.log))
    # the mission catalogue is shared with op-tui (MISSION tab data) and the flow stage follows the handover
    assert ui.mod.MISSIONS and "m4t0" in ui.mod.MISSIONS and ui.mod.MISSION_ORDER[0] == "m4t0" and ui.mod.code_of("TEST_AUTO") == "m0t1"
    assert ui.mod.flow_stage(app.flow_state(), app.args.vehicle_id, app.active) >= 0
