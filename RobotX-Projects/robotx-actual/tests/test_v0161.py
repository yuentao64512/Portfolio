"""v0.1.61 (WP12-rest, decided with Dmitrii 2026-11-07): the cannon link speaks the students' receiver.
Their color_cannon_receiver.py (a verbatim snapshot, cv/cannon/pi/) is the contract: dock_color_aim.v1, a measured target (x / y / z,
confidence, color_fraction, range) of an allowed colour - RED by default (handbook 3.3.4). One camera owner: the observers measure
the window (cv/cannon/aim_cv.py, their segmentation ported) into a target file; the sender sends it only while the mission aims; the
Pi's status comes from the receiver's own log (cv/cannon/pi/cannon_status_pub.py). Quick: the pure parts, the rendered frames, the
receiver's own functions, m3t1 in the fast SIL. Live (mosquitto): the fake Pi running their receiver as a process."""
from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import sys
import threading
import time

import pytest

from .conftest import ROOT

CANNON = ROOT / "cv" / "cannon"
PI = CANNON / "pi"
sys.path.insert(0, str(CANNON))
sys.path.insert(0, str(PI))
import aim_core as A  # noqa: E402
import status_core as SC  # noqa: E402

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
NEAR_DOCK = {"x_n": 32.0, "y_e": 3.0, "hdg_deg": 0.0}
DOCKED = (43.5, 5.5, 0.0)                          # 1.5 m off bay 2's face, straight on (test_v0146)
SHA = {"pi/color_cannon_receiver.py": "cd9db52b2cdba9a43cd52124d90493a7225f92dd3a522db13a680d00dfc566a0",
       "color_dock_sender.py": "5e23b6994e69222587a8b60a9ebe09e5c167795c9c692c223133d788e2ceed5f"}


def _sender():
    spec = importlib.util.spec_from_file_location("cannon_sender_t160", CANNON / "cannon_sender.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


CS = _sender()


def _rx_cfg(**net):
    cfg = json.loads((PI / "color_cannon_config.sim.json").read_text())
    cfg["network"].update(net)
    return cfg


def _rec(target, ns, status="OK"):
    return {"protocol": A.TARGET_FILE_PROTOCOL, "sent_unix_ns": ns, "status": status, "target": target}


def _tgt(x=-0.2, y=1.5, z=0.05, colour="red"):
    return A.target_record(colour, 0.9, 900, [0, 0, 30, 30], A.cannon_coordinates(y, x, -z, A.MOUNT0))


def test_the_snapshots_are_verbatim_and_listed():
    prov = (ROOT / "cv" / "PROVENANCE.md").read_text()
    for rel, sha in SHA.items():
        assert hashlib.sha256((CANNON / rel).read_bytes()).hexdigest() == sha, rel
        assert sha[:16] in prov and f"cannon/{rel}" in prov
    rx = SC.load_receiver()                                         # imports with the standard library only (pigpio: --live)
    assert rx.PUMP_CONFIRM == "I_ACCEPT_GREEN_10_SECOND_WATER_CANNON" and callable(rx.target_solution)
    assert "pigpio" not in sys.modules
    assert "destination" not in (ROOT / "cv" / "docking" / "dock_observer.py").read_text()   # the NameError at the first frame


def test_the_gate_sends_only_a_fresh_measured_target_while_the_aim_is_on():
    g = A.AimGate("s", "red", host="h")
    ns = 50_000_000_000
    b = g.body(0.0, ns)
    assert b["protocol"] == "dock_color_aim.v1" and b["status"] == "NO_AIM" and b["target"] is None and b["color"] == "red"
    g.on_aim({"on": True, "bay": 2, "window": "upper", "window_id": 1, "t": "x"}, 10.0)
    assert g.body(10.1, ns)["status"] == "NO_TARGET" and g.body(10.1, ns)["why"] == "no target file"
    g.on_target(_rec(_tgt(), ns))
    b = g.body(10.2, ns + 100_000_000)
    assert b["status"] == "OK" and b["window_id"] == 1 and b["target"]["bay"] == 2 and b["target"]["window"] == "upper"
    for k in ("class_name", "color", "confidence", "color_fraction", "x_right_m", "y_forward_m", "z_up_m", "range_m"):
        assert k in b["target"], k                                  # every field their target_solution reads
    assert g.body(10.3, ns + 600_000_000)["status"] == "NO_TARGET"   # the file older than 0.5 s
    assert "old" in g.body(10.3, ns + 600_000_000)["why"]
    g.on_target(_rec(None, ns + 700_000_000, "NO_RED_TARGET"))
    assert g.body(10.4, ns + 700_000_000)["why"] == "observer: NO_RED_TARGET"
    g.on_target(_rec(_tgt(colour="green"), ns + 800_000_000))
    assert "green" in g.body(10.5, ns + 800_000_000)["why"]          # a green blob is never sent when aiming red
    g.on_target(_rec(_tgt(), ns + 900_000_000))
    assert g.body(11.01, ns + 900_000_000)["status"] == "AIM_STALE"  # the aim > 1 s old
    g.on_aim({"on": False}, 11.1)
    assert g.body(11.1, ns + 900_000_000)["status"] == "AIM_OFF"
    with pytest.raises(ValueError):
        A.AimGate("s", "blue")


def test_the_target_file_round_trip(tmp_path):
    p = tmp_path / "runs" / "latest" / "cannon_target.json"
    rec = A.write_target(p, "OK", _tgt(), "red", "test")
    back = A.read_target(p)
    assert back["status"] == "OK" and back["target"]["color"] == "red" and back["sent_unix_ns"] == rec["sent_unix_ns"]
    p.write_text("{not json")
    assert A.read_target(p) is None and A.read_target(tmp_path / "none.json") is None


@pytest.mark.parametrize("win", [1, 2])
@pytest.mark.parametrize("pose", [DOCKED, (41.0, 5.5, 0.0), (43.0, 6.5, -15.0)])
def test_the_ported_segmentation_finds_the_lit_window_on_rendered_frames(win, pose):
    """The students' colour segmentation + measure_mask (ported, aim_cv) on the renderer's dock with ZED-like depth noise lands on
    the window the renderer drew: within 5 cm of the truth (fake_cannon_target) in x / y / z."""
    np = pytest.importorskip("numpy")
    pytest.importorskip("cv2")
    import aim_cv
    from optiprimex.sim import render as R, world as W, zed
    from optiprimex.sim.fake_cannon_target import truth_target
    w = W.load(POOL)
    st = {"green_bay": 2, "phase": "lit", "window": {"bay": 2, "window": win, "colour": "red"}}
    cam = R.Camera(w=1104, h=621)
    rgb, depth, pid = R.render_ids(R.scene_from_world(w, dock_state=st), *pose, cam)
    sys.path.insert(0, str(ROOT / "cv" / "sim_observer"))
    from sim_observer import point_cloud
    xyz = point_cloud(zed.stereo_noise(depth, pid, cam.fx, np.random.default_rng(3)), {"fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy})
    status, t, n = aim_cv.find_target(np.ascontiguousarray(rgb[:, :, ::-1]), xyz, "red")
    ts, tt = truth_target(w, {"x_n": pose[0], "y_e": pose[1], "hdg_deg": pose[2]}, st)
    assert status == "OK" and ts == "OK" and n == 1, (status, n)
    for k in ("x_right_m", "y_forward_m", "z_up_m"):
        assert abs(t[k] - tt[k]) < 0.05, (k, t[k], tt[k])
    assert (t["z_up_m"] > 0) == (win == 1)                            # window 1 upper, 2 lower (relative to the camera)
    dark = dict(st, phase="idle", window={"bay": None, "window": None, "colour": "black"})
    rgb2, depth2, pid2 = R.render_ids(R.scene_from_world(w, dock_state=dark), *pose, cam)
    xyz2 = point_cloud(depth2, {"fx": cam.fx, "fy": cam.fy, "cx": cam.cx, "cy": cam.cy})
    assert aim_cv.find_target(np.ascontiguousarray(rgb2[:, :, ::-1]), xyz2, "red")[0] == "NO_RED_TARGET"
    assert truth_target(w, {"x_n": pose[0], "y_e": pose[1], "hdg_deg": pose[2]}, dark)[0] == "NO_RED_TARGET"


def test_the_students_receiver_accepts_our_packet():
    """Their own decode_verified / target_solution, with the receiver loop's protocol, age, replay and stability rules."""
    key = b"k" * 64
    cfg = _rx_cfg()
    emu = SC.ReceiverEmu(cfg, key)
    g = A.AimGate("sess", "red", host="h")
    g.on_aim({"on": True, "bay": 2, "window_id": 1}, 0.0)
    ns0 = 1_000_000_000_000
    evs = []
    for k in range(8):                                               # 5 Hz for 1.4 s on a still target
        ns = ns0 + k * 200_000_000
        g.on_target(_rec(_tgt(x=-0.21, y=1.48, z=0.04), ns))
        g.aim_t = k * 0.2
        evs += emu.step(CS.signed_packet(g.body(k * 0.2, ns), key), k * 0.2, ns)
    assert [e["event"] for e in evs[:5]] == ["AIM"] * 5 and evs[-1]["event"] == "DRY_RUN_READY_10S", evs
    assert evs[0]["pan_deg"] == pytest.approx(math.degrees(math.atan2(-0.21, 1.48)), abs=0.01) and evs[0]["color"] == "red"
    assert evs[0]["tilt_deg"] > 0 and evs[0]["pump"] == "OFF" and evs[0]["mode"] == "DRY_RUN"
    g.on_aim({"on": False}, 2.0)
    ev = emu.step(CS.signed_packet(g.body(2.0, ns0 + 2_000_000_000), key), 2.0, ns0 + 2_000_000_000)[0]
    assert ev["event"] == "SAFE_NO_TARGET" and ev["status"] == "AIM_OFF"
    # what it refuses: the v0.1.57 packet, another key, a green target, a stale packet, a replay
    legacy = CS.AimFollower("s2", {"forward_from_camera_m": 0.0, "right_from_camera_m": 0.0}, host="h").body(0.0)
    assert emu.step(CS.signed_packet(legacy, key), 3.0, time.time_ns())[0]["reason"] == "WRONG_PROTOCOL"
    g.on_aim({"on": True, "bay": 2, "window_id": 1}, 4.0)
    ns = ns0 + 4_000_000_000
    g.on_target(_rec(_tgt(), ns))
    assert emu.step(CS.signed_packet(g.body(4.0, ns), b"x" * 64), 4.0, ns)[0]["reason"] == "BAD_HMAC"
    green = A.AimGate("s3", "green", host="h")
    green.on_aim({"on": True, "bay": 2}, 4.0)
    green.on_target(_rec(_tgt(colour="green"), ns))
    assert emu.step(CS.signed_packet(green.body(4.0, ns), key), 4.0, ns)[0]["reason"] == "WRONG_COLOR"
    pkt = CS.signed_packet(g.body(4.1, ns), key)
    assert emu.step(pkt, 4.1, ns + 1_500_000_000)[0]["reason"] == "STALE_PACKET"
    assert emu.step(pkt, 4.1, ns)[0]["event"] == "AIM"
    assert emu.step(pkt, 4.2, ns)[0]["reason"] == "REPLAY_OR_OUT_OF_ORDER"
    far = A.AimGate("s4", "red", host="h")
    far.on_aim({"on": True}, 5.0)
    far.on_target(_rec(_tgt(y=20.0), ns))
    assert emu.step(CS.signed_packet(far.body(5.0, ns), key), 5.0, ns)[0]["reason"] == "RANGE_OUT_OF_BOUNDS"


def test_the_status_from_the_receivers_events():
    s = SC.ReceiverStatus("test")
    assert s.status(0.0)["hmac_ok"] is None and s.status(0.0)["ok"] is False and s.status(0.0)["n_rx"] == 0
    s.on_aim({"on": True, "bay": 2, "window_id": 2}, 1.0)
    aim = {"utc": "u1", "event": "AIM", "sequence": 4, "mode": "DRY_RUN", "pump": "OFF", "color": "red", "stable_packets": 1,
           "x_right_m": 0.14, "y_forward_m": 1.48, "z_up_m": -0.2, "pan_deg": 5.4, "tilt_deg": -7.7}
    s.on_event(aim, 1.0)
    st = s.status(1.2)
    assert st["ok"] and st["hmac_ok"] and st["event"] == "AIM" and st["status"] == "OK" and st["window_id"] == 2
    assert st["target"]["pan_deg"] == 5.4 and st["mode"] == "DRY_RUN" and st["pump"] == "OFF" and not st["ready"]
    s.on_event(dict(aim, event="DRY_RUN_READY_10S", sequence=9), 2.0)
    assert s.status(2.0)["ready"] is True
    s.on_event({"utc": "u2", "event": "DROP_INVALID_GREEN_PUMP_OFF", "reason": "BAD_HMAC", "pump": "OFF"}, 2.5)
    st = s.status(2.6)
    assert st["hmac_ok"] is False and st["ok"] is False and st["n_bad"] == 1 and st["reason"] == "BAD_HMAC" and st["target"] is None
    assert st["ready"] is False
    s.on_event({"utc": "u3", "event": "SAFE_NO_TARGET", "sequence": 12, "status": "AIM_OFF", "pump": "OFF"}, 5.0)
    st = s.status(5.0)
    assert st["hmac_ok"] is True and st["ok"] is True and st["status"] == "AIM_OFF" and st["target"] is None and st["n_rx"] == 4
    s.on_event({"utc": "u4", "event": "DROP_INVALID_GREEN_PUMP_OFF", "reason": "RANGE_OUT_OF_BOUNDS", "pump": "OFF"}, 5.5)
    assert s.status(5.6)["hmac_ok"] is True and s.status(5.6)["reason"] == "RANGE_OUT_OF_BOUNDS"      # signed fine, refused target
    s.on_event({"utc": "u5", "event": "WATCHDOG_CENTER_PUMP_OFF", "pump": "OFF"}, 9.0)
    assert s.status(9.0)["ok"] is False and s.status(9.0)["event"] == "WATCHDOG_CENTER_PUMP_OFF" and s.status(9.0)["n_rx"] == 5


def test_the_log_follower_and_the_pi_publisher(tmp_path):
    import cannon_status_pub as P
    d = tmp_path / "color_receiver_logs"
    d.mkdir()
    f1 = d / "green_receiver_20261107_120000.jsonl"
    f1.write_text(json.dumps({"utc": "a", "event": "SAFE_NO_TARGET", "status": "NO_AIM", "pump": "OFF", "sequence": 0}) + "\n" + '{"utc": "b", "ev')
    fol = SC.LogFollower(d)
    assert [e["event"] for e in fol.poll()] == ["SAFE_NO_TARGET"]          # the half line waits
    with f1.open("a") as fh:
        fh.write('ent": "AIM", "sequence": 1, "pan_deg": 1.0, "tilt_deg": 2.0, "mode": "DRY_RUN", "pump": "OFF"}\n')
    assert [e["event"] for e in fol.poll()] == ["AIM"]

    class Client:
        def __init__(self):
            self.sent = []

        def publish(self, topic, payload, qos=0):
            self.sent.append((topic, json.loads(payload)))
    c = Client()
    pub = P.Publisher(c, d, key_label="test", source="t160", log=lambda s: None)
    pub.on_aim_message(json.dumps({"body": {"on": True, "bay": 1, "window_id": 1}}).encode())
    st = pub.tick()
    assert st["event"] == "AIM" and st["window_id"] == 1 and st["key"] == "test" and st["n_rx"] == 2
    topic, env = c.sent[-1]
    assert topic == "local/usv/cannon/status" and env["source"] == "t160" and env["body"]["event"] == "AIM"
    time.sleep(0.01)
    f2 = d / "green_receiver_20261107_120500.jsonl"                         # the receiver restarted: a new file
    f2.write_text(json.dumps({"utc": "c", "event": "SAFE_NO_TARGET", "status": "AIM_OFF", "pump": "OFF", "sequence": 0}) + "\n")
    assert [e["event"] for e in fol.poll()] == ["SAFE_NO_TARGET"]


def test_m3t1_fast_sil_through_the_students_receiver():
    """m3t1 docks and fires as before; on the cannon wire, on the sim clock: the truth target of the lit window (what the camera
    measures, test above) -> the gate at the sender's 5 Hz -> their receiver's decision: it aims at the reported window during
    AIM_FIRE, reaches a stable lock (DRY_RUN_READY_10S), never accepts a target outside AIM_FIRE, and is safe within 1 s of the end."""
    from optiprimex.sim import world as W
    from optiprimex.sim.fake_cannon_target import truth_target
    from optiprimex.sim.fast_sil import FastSil
    key = b"s" * 64
    world = W.load(POOL)
    gate, emu, status = A.AimGate("sil", "red", host="sil"), SC.ReceiverEmu(_rx_cfg(), key), SC.ReceiverStatus("test")
    st = {"mm_t": -1e9, "on": False, "tx_t": -1e9, "off_t": None}
    log = []

    def tick(sil):
        t, aim = sil.t, sil.aim
        ns = int(t * 1e9)
        on = bool(aim and aim.get("on"))
        if on and t - st["mm_t"] >= 0.2:
            st["mm_t"] = t
            gate.on_aim(dict(aim, t="sim"), t)
            status.on_aim(aim, t)
        elif not on and st["on"]:
            gate.on_aim({"on": False}, t)
            status.on_aim({"on": False}, t)
            st["off_t"] = t
        st["on"] = on
        if t - st["tx_t"] >= 0.2 - 1e-9:
            st["tx_t"] = t
            pz = sil.pose()
            s, tg = truth_target(world, {"x_n": pz.x_n, "y_e": pz.y_e, "hdg_deg": pz.hdg_deg}, sil.dock.state(t) if sil.dock else None)
            gate.on_target(_rec(tg, ns, s))
            for ev in emu.step(CS.signed_packet(gate.body(t, ns), key), t, ns):
                status.on_event(ev, t)
                log.append((t, on, ev))
        return False

    r = FastSil(POOL, "T3", {}, start=NEAR_DOCK).run(400, until=tick)
    assert r["state"] == "DONE", (r["state"], r["reason"], r["states"])
    fire = [e for e in r["events"] if e.get("event") == "firefighting"]
    assert fire and fire[-1].get("window_id") in (1, 2), r["events"]
    aimed = [ev for _t, on, ev in log if ev["event"] in SC.AIMING]
    assert len(aimed) >= 5 and all(on for _t, on, ev in log if ev["event"] in SC.AIMING)
    assert any(ev["event"] == "DRY_RUN_READY_10S" for _t, _on, ev in log), [ev["event"] for _t, _on, ev in log][-40:]
    assert not [ev for _t, _on, ev in log if ev["event"].startswith("DROP")], [ev for _t, _on, ev in log if ev["event"].startswith("DROP")][:3]
    assert all(ev["color"] == "red" for ev in aimed)
    assert all((ev["z_up_m"] > 0) == (fire[-1]["window_id"] == 1) for ev in aimed)      # the reported window is the one aimed at
    t_off = st["off_t"]
    assert t_off is not None
    assert all(ev["event"] == "SAFE_NO_TARGET" for t, _on, ev in log if t > t_off + 1.0)
    assert status.status(log[-1][0])["hmac_ok"] is True and status.n_bad == 0


def test_the_launchers_wire_the_new_link():
    sh = (ROOT / "scripts" / "run_bench.sh").read_text()
    assert "--truth-world $WORLD" in sh and "--target-out $RUN_DIR/cannon_target.json" in sh and "buoy_link" not in sh
    sim = (ROOT / "scripts" / "run_sim.sh").read_text()
    assert "--cannon red --cannon-out" in sim
    readme = (CANNON / "README.md").read_text()
    for w in ("dock_color_aim.v1", "cannon_target.json", "color_cannon_receiver.py", "cannon_status_pub.py", "RED", "DRY_RUN_READY_10S"):
        assert w in readme, w
    a = CS.parse_args([])
    assert a.protocol == "dock_color_aim.v1" and a.color is None and CS.parse_args(["--protocol", "dock_cannon.v1"]).protocol == "dock_cannon.v1"


# ---- live: their receiver as a process behind the fake Pi ---------------------------------------------------------------------
@pytest.fixture(scope="module")
def students_pi(brokers, tmp_path_factory):
    from .conftest import PY, Probe, Proc, free_udp_port
    tmp = tmp_path_factory.mktemp("can160")
    port = free_udp_port()
    target = tmp / "cannon_target.json"
    stop = threading.Event()

    def observer():                                                   # the observer's --cannon red, without a camera: 10 Hz
        while not stop.is_set():
            A.write_target(target, "OK", _tgt(x=0.14, y=1.48, z=-0.2), "red", "t160")
            stop.wait(0.1)
    th = threading.Thread(target=observer, daemon=True)
    th.start()
    pi = Proc([PY, "-m", "optiprimex.sim.fake_cannon_pi", "--port", str(brokers.boat_port), "--listen", f"127.0.0.1:{port}",
               "--key-out", str(tmp / "cannon_test.key"), "--target-out", str(target),
               "--with-sender", f"--mqtt 127.0.0.1:{brokers.boat_port} --hz 10"], tmp / "pi.log")
    probe = Probe(brokers.boat_port, ["local/usv/cannon/#"]).start()
    yield {"pi": pi, "probe": probe, "tmp": tmp, "boat_port": brokers.boat_port, "stop": stop}
    stop.set()
    probe.stop()
    pi.kill()


def test_cannon_live_through_the_students_receiver(students_pi):
    import paho.mqtt.client as mqtt
    s = students_pi
    p = s["probe"]
    st = lambda pred, to: p.wait(lambda m: m["topic"] == "local/usv/cannon/status" and pred(m["env"]["body"]), to)
    assert st(lambda b: b["ok"] and b["hmac_ok"] and b["event"] == "SAFE_NO_TARGET" and b["status"] == "NO_AIM", 15), s["pi"].log()
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t160-aim", protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", s["boat_port"])
    c.loop_start()
    for _ in range(12):
        c.publish("local/usv/cannon/aim", json.dumps({"body": {"on": True, "bay": 2, "window": "lower", "window_id": 2, "t": "x"}}), qos=0)
        time.sleep(0.2)
    assert st(lambda b: b["event"] == "DRY_RUN_READY_10S" and b["ready"] and b["window_id"] == 2 and b["mode"] == "DRY_RUN"
              and b["pump"] == "OFF" and b["tilt_deg"] < 0, 3), s["pi"].log()
    c.publish("local/usv/cannon/aim", json.dumps({"body": {"on": False, "t": "x"}}), qos=0).wait_for_publish(2)
    t_off = time.monotonic()
    assert st(lambda b: b["event"] == "SAFE_NO_TARGET" and b["status"] == "AIM_OFF" and b["target"] is None and not b["ready"], 3)
    assert time.monotonic() - t_off <= 1.0 + 0.6
    c.loop_stop()
    c.disconnect()
    logs = list((s["tmp"] / "cannon_pi" / "color_receiver_logs").glob("*.jsonl"))
    assert logs and "DRY_RUN_READY_10S" in logs[0].read_text()          # their receiver, their log, in the run folder
    assert not (PI / "color_receiver_logs").exists()                     # never inside the tree
    assert "buoy_link" not in s["pi"].log()


def test_the_pages_show_the_receivers_view():
    from optiprimex.tools import boat_web as B
    from optiprimex.tools.ocs_web import CannonTap
    env = lambda b: json.dumps({"body": b}).encode()
    m = B.Model()
    m.on_message("local/usv/cannon/aim", env({"on": True, "bay": 2, "window_id": 1}), now=10.0)
    good = {"ok": True, "age_s": 0.1, "hmac_ok": True, "target": {"window_id": 1}, "window_id": 1, "n_rx": 9, "n_bad": 0,
            "event": "DRY_RUN_READY_10S", "pan_deg": -8.1, "tilt_deg": 1.5, "ready": True, "mode": "DRY_RUN", "pump": "OFF"}
    m.on_message("local/usv/cannon/status", env(good), now=10.0)
    c = m.cannon(10.1)
    assert c["state"] == "active" and "rx DRY_RUN_READY_10S pan -8.1° tilt +1.5°" in c["l2"] and "LOCK" in c["l2"] and "DRY_RUN ·" not in c["l2"]
    m.on_message("local/usv/cannon/status", env(dict(good, event="DROP_INVALID_GREEN_PUMP_OFF", reason="WRONG_COLOR", target=None,
                                                     pan_deg=None, tilt_deg=None, ready=False)), now=10.2)
    c = m.cannon(10.3)
    assert c["state"] == "warning" and "Pi REFUSES: WRONG_COLOR" in c["l1"] and "(WRONG_COLOR)" in c["l2"]
    m.on_message("local/usv/cannon/status", env(dict(good, mode="LIVE_PUMP_10S", pump="ON", event="PUMP_ON_GREEN")), now=10.4)
    assert "LIVE_PUMP_10S" in m.cannon(10.5)["l2"] and "PUMP ON" in m.cannon(10.5)["l2"]
    tap = CannonTap("usv")
    tap.on_message(env(good), now=5.0)
    v = tap.view(now=5.2)
    assert v["state"] == "ok" and v["event"] == "DRY_RUN_READY_10S" and v["ready"] is True and v["pump"] == "OFF"
    assert 'cn.event?" · rx "' in (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
