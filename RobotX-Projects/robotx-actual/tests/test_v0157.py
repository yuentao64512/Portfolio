"""v0.1.57 (ROADMAP WP12, docs/OVERNIGHT_2026-11-07.md D11-D13): the cannon contract and link in the simulator.
The sender follows local/<v>/cannon/aim (target null unless the aim is on and fresher than 1 s, `window_id` in the packet), the
fake Pi verifies the HMAC with a key it generates and publishes local/<v>/cannon/status; the boat window and the OCS state show it.
Quick: the pure parts + m3t1 in the fast SIL with the sender and the fake Pi in the loop. Live (mosquitto): the processes."""
from __future__ import annotations

import importlib.util
import json
import socket
import stat
import time

import pytest

from optiprimex.sim.fake_cannon_pi import PiSim, generate_key, verify
from optiprimex.sim.fast_sil import FastSil

from .conftest import ROOT

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
NEAR_DOCK = {"x_n": 32.0, "y_e": 3.0, "hdg_deg": 0.0}
CFG0 = {"forward_from_camera_m": 0.0, "right_from_camera_m": 0.0}


def _sender():
    spec = importlib.util.spec_from_file_location("cannon_sender_t157", ROOT / "cv" / "cannon" / "cannon_sender.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


CS = _sender()


def test_the_generated_key_and_the_refusal(tmp_path):
    k = generate_key(tmp_path / "a" / "cannon_test.key")
    assert len(k) == 64 and (tmp_path / "a" / "cannon_test.key").read_text().strip().encode() == k
    assert stat.S_IMODE((tmp_path / "a" / "cannon_test.key").stat().st_mode) == 0o600
    assert generate_key(tmp_path / "a" / "cannon_test.key") != k                  # fresh each start
    with pytest.raises(SystemExit):
        generate_key(tmp_path / "buoy_link.key")


def test_the_follower_freshness_and_window_id():
    fol = CS.AimFollower("s", CFG0, host="h")
    b = fol.body(0.0)
    assert b["target"] is None and b["status"] == "NO_AIM" and b["window_id"] is None and b["protocol"] == "dock_cannon.v1"
    fol.on_aim({"on": True, "bay": 2, "window": "lower", "window_id": 2, "t": "x"}, 10.0)
    b = fol.body(10.99)
    assert b["status"] == "OK" and b["window_id"] == 2 and b["target"]["bay"] == 2 and b["target"]["window"] == "lower"
    assert b["target"]["geometry"] is False and b["aim_utc"] == "x"
    assert fol.body(11.01)["target"] is None and fol.body(11.01)["status"] == "AIM_STALE"     # > 1 s: no aim
    fol.on_aim({"on": True, "bay": 2, "window_id": 2}, 12.0)
    fol.on_aim({"on": False}, 12.1)
    assert fol.body(12.1)["target"] is None and fol.body(12.1)["status"] == "AIM_OFF"
    # the observer's geometry when fresh: the cannon frame added
    fol.on_aim({"on": True, "bay": 1, "window_id": 1}, 20.0)
    fol.on_geometry({"status": "OK", "target": {"body_forward_m": 4.0, "body_right_m": -1.0, "range_m": 4.1}})
    t = fol.body(20.1)["target"]
    assert t["geometry"] is True and t["y_forward_m"] == pytest.approx(4.0) and t["x_right_m"] == pytest.approx(-1.0) and t["window_id"] == 1
    assert [fol.body(20.2)["sequence"], fol.body(20.2)["sequence"]] == [fol.sequence - 2, fol.sequence - 1]


def test_the_pi_verifies_like_the_students_receiver():
    key = b"k" * 64
    pi = PiSim(key)
    assert pi.status(0.0)["ok"] is False and pi.status(0.0)["hmac_ok"] is None and pi.status(0.0)["last_t"] is None
    body = {"protocol": "dock_cannon.v1", "sequence": 3, "status": "OK", "window_id": 1, "target": {"bay": 2, "window_id": 1}}
    assert pi.receive(CS.signed_packet(body, key), 1.0)
    s = pi.status(1.5)
    assert s["ok"] and s["hmac_ok"] and s["window_id"] == 1 and s["n_rx"] == 1 and s["n_bad"] == 0 and s["last_t"] and s["key"] == "test"
    assert pi.status(3.6)["ok"] is False                                           # no packet for > 2 s: the link is not ok
    assert not pi.receive(CS.signed_packet(body, b"x" * 64), 4.0)                  # another key
    s = pi.status(4.0)
    assert s["hmac_ok"] is False and s["ok"] is False and s["n_bad"] == 1 and s["window_id"] == 1     # the last GOOD target kept
    assert not pi.receive(b"not json", 4.1) and pi.status(4.1)["n_bad"] == 2
    assert pi.receive(CS.signed_packet(body, key), 4.5) and pi.status(4.5)["hmac_ok"] is False    # a bad one in the last 2 s
    assert pi.status(6.2)["hmac_ok"] is True and pi.status(6.2)["ok"] is True
    tampered = json.loads(CS.signed_packet(body, key))
    tampered["body"]["window_id"] = 2
    assert verify(json.dumps(tampered).encode(), key)[0] is False


def test_m3t1_fast_sil_with_the_sender_and_the_fake_pi_in_the_loop(tmp_path):
    """The accept of D13: m3t1 docks and fires as before (the fake dock lights on the aim), and on the cannon wire, emulated at
    the mission manager's 5 Hz aim and the sender's 5 Hz packets on the sim clock: hmac_ok, the reported window_id, target null
    within 1 s of `on: false`."""
    key = generate_key(tmp_path / "cannon_test.key")
    fol, pi = CS.AimFollower("sil", CFG0, host="sil"), PiSim(key)
    st = {"mm_t": -1e9, "on": False, "tx_t": -1e9, "off_t": None}
    log = []

    def tick(sil):
        t, aim = sil.t, sil.aim
        on = bool(aim and aim.get("on"))
        if on and t - st["mm_t"] >= 0.2:                           # mission_manager._cannon: 5 Hz while on
            st["mm_t"] = t
            fol.on_aim(dict(aim, t="sim"), t)
        elif not on and st["on"]:
            fol.on_aim({"on": False, "t": "sim"}, t)              # one on:false when it stops
            st["off_t"] = t
        st["on"] = on
        if t - st["tx_t"] >= 0.2 - 1e-9:                           # the sender: 5 Hz
            st["tx_t"] = t
            pi.receive(CS.signed_packet(fol.body(t), key), t)
            log.append((t, on, pi.status(t)))
        return False

    r = FastSil(POOL, "T3", {}, start=NEAR_DOCK).run(400, until=tick)
    assert r["state"] == "DONE", (r["state"], r["reason"], r["states"])
    fire = [e for e in r["events"] if e.get("event") == "firefighting"]
    assert fire and fire[-1].get("window_id") in (1, 2), r["events"]
    wid = fire[-1]["window_id"]
    aimed = [s for _t, on, s in log if on and s["target"] is not None]
    assert len(aimed) >= 5 and all(s["hmac_ok"] and s["ok"] and s["window_id"] == wid for s in aimed)
    assert all(s["hmac_ok"] for _t, _on, s in log) and pi.n_bad == 0
    t_off = st["off_t"]
    assert t_off is not None
    first_null = next(t for t, _on, s in log if t >= t_off and s["target"] is None)
    assert first_null - t_off <= 1.0
    assert all(s["target"] is None for t, _on, s in log if t > t_off + 1.0)


def test_the_boat_window_and_the_ocs_state_read_the_status():
    from optiprimex.tools import boat_web as B
    from optiprimex.tools.ocs_web import CannonTap
    m = B.Model()
    assert "local/usv/cannon/status" in m.topics()
    env = lambda b: json.dumps({"body": b}).encode()
    assert "Pi status: none" in m.cannon(0.0)["l2"]
    m.on_message("local/usv/cannon/aim", env({"on": True, "bay": 2, "window_id": 1}), now=10.0)
    good = {"ok": True, "age_s": 0.1, "hmac_ok": True, "target": {"window_id": 1}, "window_id": 1, "n_rx": 9, "n_bad": 0}
    m.on_message("local/usv/cannon/status", env(good), now=10.0)
    c = m.cannon(10.1)
    assert c["state"] == "active" and "w1" in c["l1"] and "hmac ok" in c["l2"] and "last packet 0.1 s" in c["l2"]
    m.on_message("local/usv/cannon/status", env(dict(good, ok=False, hmac_ok=False, n_bad=1)), now=10.2)
    c = m.cannon(10.3)
    assert c["state"] == "warning" and "HMAC BAD" in c["l2"]
    assert "STALE" in m.cannon_link(20.0)[1]
    tap = CannonTap("usv")
    assert tap.topic == "local/usv/cannon/status" and tap.view(now=0.0)["state"] == "none"
    tap.on_message(env(good), now=5.0)
    v = tap.view(now=5.5)
    assert v["state"] == "ok" and v["hmac_ok"] is True and v["window_id"] == 1 and v["target_on"] is True and v["n_rx"] == 9
    tap.on_message(env(dict(good, ok=False, hmac_ok=False)), now=6.0)
    assert tap.view(now=6.1)["state"] == "bad" and "HMAC" in tap.view(now=6.1)["why"]
    assert tap.view(now=10.0)["state"] == "stale"


def test_the_launchers_start_the_cannon_window():
    sh = (ROOT / "scripts" / "run_bench.sh").read_text()
    assert "fake_dock cannon" not in sh and "dock cannon rc ocs" in sh and "dock cannon ocs" in sh
    assert "optiprimex.sim.fake_cannon_pi" in sh and "--key-out $RUN_DIR/cannon_test.key" in sh and "--with-sender" in sh
    assert "buoy_link" not in sh
    assert "window cannon" in (ROOT / "scripts" / "run_sim.sh").read_text()
    readme = (ROOT / "cv" / "cannon" / "README.md").read_text()
    for w in ("local/<v>/cannon/aim", "local/<v>/cannon/status", "window_id", "hmac_sha256", "1 s"):      # v0.1.61: the snapshot is in
        assert w in readme
    a = CS.parse_args([])
    assert a.source == "aim" and CS.parse_args(["--source", "file"]).source == "file"


# ---- live: the processes on a real broker ---------------------------------------------------------------------------------------
@pytest.fixture(scope="module")
def cannon_stack(brokers, tmp_path_factory):
    from .conftest import PY, Probe, Proc, free_udp_port
    tmp = tmp_path_factory.mktemp("can")
    port = free_udp_port()
    key_path = tmp / "cannon_test.key"
    pi = Proc([PY, "-m", "optiprimex.sim.fake_cannon_pi", "--port", str(brokers.boat_port), "--listen", f"127.0.0.1:{port}",
               "--key-out", str(key_path), "--receiver", "ours", "--with-sender", f"--mqtt 127.0.0.1:{brokers.boat_port} --hz 10"], tmp / "pi.log")
    probe = Probe(brokers.boat_port, ["local/usv/cannon/#"]).start()
    yield {"pi": pi, "probe": probe, "port": port, "key": key_path, "boat_port": brokers.boat_port}
    probe.stop()
    pi.kill()


def _pub_aim(port, body):
    import paho.mqtt.client as mqtt
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"t157-aim-{time.time_ns()}", protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", port)
    c.loop_start()
    c.publish("local/usv/cannon/aim", json.dumps({"schema_version": 1, "body": body}), qos=0).wait_for_publish(2)
    c.loop_stop()
    c.disconnect()


def test_cannon_live_sender_and_fake_pi(cannon_stack):
    s = cannon_stack
    p = s["probe"]
    st = lambda pred, to: p.wait(lambda m: m["topic"] == "local/usv/cannon/status" and pred(m["env"]["body"]), to)
    assert st(lambda b: b["ok"] and b["hmac_ok"] and b["target"] is None and b["status"] == "NO_AIM", 15), s["pi"].log()
    # the aim at 5 Hz for 2 s: the target with its window_id reaches the Pi, HMAC ok
    import paho.mqtt.client as mqtt
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="t157-aim", protocol=mqtt.MQTTv311)
    c.connect("127.0.0.1", s["boat_port"])
    c.loop_start()
    for _ in range(10):
        c.publish("local/usv/cannon/aim", json.dumps({"body": {"on": True, "bay": 3, "window": "upper", "window_id": 1, "t": "x"}}), qos=0)
        time.sleep(0.2)
    assert st(lambda b: b["ok"] and b["hmac_ok"] and b["window_id"] == 1 and (b["target"] or {}).get("bay") == 3, 3), s["pi"].log()
    c.publish("local/usv/cannon/aim", json.dumps({"body": {"on": False, "t": "x"}}), qos=0).wait_for_publish(2)
    t_off = time.monotonic()
    assert st(lambda b: b["target"] is None and b["status"] == "AIM_OFF", 3)
    assert time.monotonic() - t_off <= 1.0 + 0.6                        # + the status' 2 Hz
    # the aim simply stops (no on:false): null within 1 s of the last aim
    for _ in range(5):
        c.publish("local/usv/cannon/aim", json.dumps({"body": {"on": True, "bay": 3, "window_id": 2}}), qos=0)
        time.sleep(0.2)
    assert st(lambda b: b["window_id"] == 2, 3)
    t_last = time.monotonic()
    assert st(lambda b: b["target"] is None and b["status"] == "AIM_STALE", 3)
    assert time.monotonic() - t_last <= 1.0 + 0.2 + 0.6
    c.loop_stop()
    c.disconnect()
    # a bad-HMAC packet -> hmac_ok false (the sender's next good one clears it, so look for the false one)
    p.clear()
    bad = CS.signed_packet({"protocol": "dock_cannon.v1", "status": "OK", "target": None}, b"wrong-key-" * 5)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(bad, ("127.0.0.1", s["port"]))
    tx.close()
    assert st(lambda b: b["hmac_ok"] is False and b["ok"] is False and b["n_bad"] == 1, 3), s["pi"].log()
    assert st(lambda b: b["hmac_ok"] is True and b["ok"] is True and b["n_bad"] == 1, 4)             # the good packets again
    assert "buoy_link" not in s["pi"].log()


def test_cannon_live_stop_stops_the_child_sender(cannon_stack):
    """SIGTERM to the fake Pi (stop_bench.sh's Ctrl-C / kill) stops its sender too: one window, nothing left behind."""
    import os
    import signal
    import subprocess
    s = cannon_stack
    os.kill(s["pi"].p.pid, signal.SIGTERM)              # the Pi alone (not its process group): it must stop its child itself
    s["pi"].p.wait(8)
    log = s["pi"].log()
    assert "[cannon_sender] stopped after" in log, log[-2000:]
    ps = subprocess.run(["ps", "-ax", "-o", "command"], capture_output=True, text=True).stdout
    assert str(s["key"]) not in ps


def test_ocs_system_card_draws_the_cannon_link():
    """v0.1.57 (D12): the OCS page's SYSTEM card has the cannon row from state.json `cannon` (main agent's merge)."""
    from .conftest import ROOT
    h = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'sysRow("USV · cannon link"' in h and "cn.hmac_ok" in h and "no cannon status (local/usv/cannon/status)" in h
