"""v0.1.64 - the UAV team's five sim-test scenarios (MERGE_GUIDE §9) re-run against OUR stack in his bridge's place: his fake copter and
his geofence listener, unmodified, from the snapshot (uav/sim-tests, uav/geofence-tools - sha256-checked), our adapter + ocs_app on a
mosquitto of their own, a recording judge as RoboCommand. The five run side by side (a broker and the ports each), ~45 s in all.

  mpfence_alt20  the declaration's ceiling 20.0 = FENCE_ALT_MAX 20 m; Mission Planner's fence write -> re-declared
  mpalt_alt20    FENCE_ALT_MAX 20 -> 25 in Mission Planner -> the declaration re-sent with 25.0
  mpfence        FENCE_ALT_MAX 100 -> refused (> 50), fence_rejected -> the listener clears the fence; nothing published
  mpsilent       an MP write the listener does not see acked -> its 10 s re-read catches it -> re-declared
  mpsilentfix    an outside write (refused, cleared), then a good one -> accepted and sent
Live tier (real processes and brokers; not in make quick)."""
from __future__ import annotations

import hashlib
import json
import os
import subprocess
import threading
import time
from pathlib import Path

import pytest

from optiprimex.companion.config import Config, MqttCfg
from optiprimex.companion.mqtt import Bus
from optiprimex.ocs.judge_client import NullJudgeClient
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.uav.adapter import UavAdapter

from .conftest import MOSQUITTO, PY, ROOT, free_port, free_udp_port, tcp_open, wait_until
from .test_ocs_units import intent

UAV = ROOT / "uav"
UID = "UAV1"
HIS_COURSE = [[1.4120, 103.9130], [1.4130, 103.9130], [1.4130, 103.9140], [1.4120, 103.9140]]     # test_mp_fence.py's course
SCENARIOS = ("mpfence_alt20", "mpalt_alt20", "mpfence", "mpsilent", "mpsilentfix", "mpsilent_alt20", "mpsilentfix_alt20")
# his stack's own result (his test_mp_fence.py run unmodified in a scratch copy, 2026-11-08 01:2x): the declared course fence, then the
# fence Mission Planner writes (inside); mpsilent / mpsilentfix carry FENCE_ALT_MAX 100, so his bridge sends NOTHING in both (every
# attempt refused for the ceiling) - his guide's "re-declared / accepted" holds for the _alt20 variants, which we run as well
HIS_FIRST = [(1.412, 103.914), (1.413, 103.914), (1.413, 103.913), (1.412, 103.913), (1.412, 103.914)]
HIS_MP = [(1.4122, 103.9132), (1.4128, 103.9132), (1.4128, 103.9138), (1.4122, 103.9138), (1.4122, 103.9132)]


def _r(fence, nd=4):
    return [(round(a, nd), round(b, nd)) for a, b in fence]


class RJudge(NullJudgeClient):
    name = "recording"

    def __init__(self):
        super().__init__()
        self.requests = []

    @property
    def connected(self):
        return True

    def publish_request(self, body):
        self.requests.append((time.time(), json.loads(json.dumps(body))))
        return len(self.requests)

    def publish_report(self, vehicle_id, body):
        return 1


def _cfg(port: int) -> Config:
    cfg = Config()
    cfg.vehicle_id, cfg.node = "OCS", "ocs_app"
    cfg.mqtt = MqttCfg(host="127.0.0.1", port=port, client_id=f"ocs-sc-{port}")
    cfg.extra = {"ocs": {"team_id": "SGIT", "vehicles": {UID: "uav"}, "judge": "none", "bench_mode": True, "status_fresh_s": 3.0,
                         "prepare_ack_s": 4.0, "ack_timeout_s": 0,
                         "uav": {"vehicle_id": UID, "registration": "UAV1", "pilot_callsign": "Calwin-U2332301", "topic": "uav",
                                 "telemetry_stale_s": 5, "stale_stop_s": 5, "declare_default": True},
                         "declaration": {"tiers": {"T1": "TIER_NONE", "T2": "TIER_NONE", "T3": "TIER_NONE", "T4": "TIER_NONE"}, "policy": "warn"}}}
    return cfg


def run_scenario(name: str, tmp: Path, out: dict) -> None:
    port, cport, lport, bport = free_port(), free_udp_port(), free_udp_port(), free_udp_port()
    d = tmp / name
    d.mkdir()
    (d / "broker.conf").write_text(f"listener {port} 127.0.0.1\nallow_anonymous true\npersistence false\n")
    procs = [subprocess.Popen([MOSQUITTO, "-c", str(d / "broker.conf")], stdout=open(d / "broker.log", "w"), stderr=subprocess.STDOUT)]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE="1")
    app = ad = None
    try:
        assert wait_until(lambda: tcp_open(port), 5)
        t0 = time.time()
        procs.append(subprocess.Popen([PY, "-u", "fake_copter.py", str(cport), name], cwd=UAV / "sim-tests", env=env,
                                      stdout=open(d / "fcu.log", "w"), stderr=subprocess.STDOUT))
        procs.append(subprocess.Popen([PY, "-u", str(UAV / "geofence-tools" / "geofence_udp_listener.py"), "--connection", f"udp:127.0.0.1:{cport}",
                                       "--listen-port", str(lport), "--vehicle-id", UID, "--preset-fence-file",
                                       str(UAV / "geofence-tools" / "preset_fence_example.csv"), "--notify", "",
                                       "--robocommand-state", str(d / "rc_state.json")], cwd=d, env=env,
                                      stdout=open(d / "listener.log", "w"), stderr=subprocess.STDOUT))
        cfg = _cfg(port)
        judge = RJudge()
        app = OcsApp(cfg, bus=Bus(cfg, client_id=f"ocs-sc-{port}", source="ocs_app"), judge=judge)
        app.start()
        app.lifecycle.on_link(True)
        app.on_course({"course_id": "TEST", "boundary": HIS_COURSE, "uav_transit_area": [], "pinger_hz": 0})
        ad = UavAdapter(cfg, ("127.0.0.1", lport), bport, "127.0.0.1", retry_for_s=15.0, fcu_link="fake")
        ad.start()
        # as his test: the declaration at ~7 s (the course forwarded and applied first), then the scripted Mission Planner writes
        assert wait_until(lambda: (ad.course_acked_t is not None) and ad.tel_n > 2, 12), f"{name}: no course ack / telemetry"
        while time.time() - t0 < 7.0:
            time.sleep(0.1)
        app.authority.on_intent("t", intent("UAV", args={"op": "ready", "on": True}, vehicle_id=UID))
        assert wait_until(lambda: ad.pilot_ready, 3)
        out[name] = {"declare": app.authority.on_intent("t", intent("DECLARE", vehicle_id=UID)).reason}
        until = 40.0 if name.startswith("mpsilent") else 32.0
        while time.time() - t0 < until:
            time.sleep(0.2)
            if not ad.pilot_ready and app.lifecycle.state in ("DECLARED", "READY"):       # a fence change clears it: the pilot says ready again
                app.authority.on_intent("t", intent("UAV", args={"op": "ready", "on": True}, vehicle_id=UID))
        out[name].update(requests=[{"t": round(t - t0, 1), "fence": r["uav_geofence"], "ceiling": r["uav_geofence_max_altitude_hae_m"]}
                                   for t, r in judge.requests],
                         rejected=[c for c in app.authority.sent if c["action"] == "FENCE_REJECTED"],
                         alerts=[a["kind"] for a in app.alerts], fence_change=app.uav_view.get("last_fence_change"))
    except BaseException as e:                                       # noqa: BLE001 - reported by the test
        out[name] = {"error": repr(e)}
    finally:
        for o in (ad, app):
            try:
                if o:
                    o.stop()
            except Exception:
                pass
        for p in reversed(procs):
            p.terminate()
        for p in procs:
            try:
                p.wait(5)
            except subprocess.TimeoutExpired:
                p.kill()
        out.setdefault(name, {})["listener"] = (d / "listener.log").read_text() if (d / "listener.log").exists() else ""
        out[name]["fcu"] = (d / "fcu.log").read_text() if (d / "fcu.log").exists() else ""


@pytest.fixture(scope="module")
def runs(tmp_path_factory):
    tmp = tmp_path_factory.mktemp("uav_scenarios")
    out: dict = {}
    th = [threading.Thread(target=run_scenario, args=(n, tmp, out)) for n in SCENARIOS]
    for t in th:
        t.start()
    for t in th:
        t.join(120)
    (tmp / "summary.json").write_text(json.dumps({k: {kk: vv for kk, vv in v.items() if kk not in ("listener", "fcu")} for k, v in out.items()}, indent=1))
    return out


def test_live_his_files_are_unmodified():
    for line in (UAV / "MANIFEST.sha256").read_text().splitlines():
        h, f = line.split("  ", 1)
        if (UAV / f).exists():
            assert hashlib.sha256((UAV / f).read_bytes()).hexdigest() == h, f


def test_live_mpfence_alt20_declares_the_ceiling_and_redeclares_the_mp_fence(runs):
    r = runs["mpfence_alt20"]
    assert "error" not in r, r
    req = r["requests"]
    assert req and req[0]["ceiling"] == 20.0 and _r(req[0]["fence"], 3) == HIS_FIRST, r     # ceiling 20.0 = FENCE_ALT_MAX 20 m
    assert len(req) >= 2 and req[-1]["fence"] != req[0]["fence"], r             # Mission Planner's write -> the re-declaration


def test_live_mpalt_alt20_resends_with_the_new_ceiling(runs):
    r = runs["mpalt_alt20"]
    assert "error" not in r, r
    cs = [q["ceiling"] for q in r["requests"]]
    assert cs and cs[0] == 20.0 and 25.0 in cs, r


def test_live_mpfence_is_refused_cleared_and_nothing_published(runs):
    r = runs["mpfence"]
    assert "error" not in r, r
    assert r["requests"] == [] and r["rejected"], r                               # > 50 m: refused, fence_rejected sent
    assert "at most 50 m HAE" in r["declare"], r
    assert "cleared" in r["listener"].lower() or "clear" in r["listener"].lower(), r["listener"][-1500:]


def test_live_mpsilent_as_his_stack_nothing_sent_but_the_reread_caught_the_write(runs):
    r = runs["mpsilent"]
    assert "error" not in r, r
    assert r["requests"] == [] and r["rejected"], r                               # his bridge: nothing published (ceiling 100)
    assert r["fence_change"] and "would REJECT" in r["fence_change"]["text"], r  # the 10 s re-read caught the silent write
    assert "Geofence changed by another program" in r["listener"]


def test_live_mpsilentfix_as_his_stack_nothing_sent(runs):
    r = runs["mpsilentfix"]
    assert "error" not in r, r
    assert r["requests"] == [] and r["rejected"] and r["fence_change"], r


def test_live_mpsilent_alt20_redeclares_the_silent_write_as_his(runs):
    r = runs["mpsilent_alt20"]
    assert "error" not in r, r
    req = r["requests"]
    assert len(req) == 2 and [q["ceiling"] for q in req] == [20.0, 20.0], r
    assert _r(req[0]["fence"], 3) == HIS_FIRST and _r(req[1]["fence"]) == HIS_MP, r


def test_live_mpsilentfix_alt20_refuses_the_outside_write_then_sends_the_good_one(runs):
    r = runs["mpsilentfix_alt20"]
    assert "error" not in r, r
    req = r["requests"]
    assert r["rejected"] and len(req) == 2 and _r(req[1]["fence"]) == HIS_MP and req[1]["t"] > 20.0, r
