"""v0.1.86.4-0.1.86.9 (WP43, the compute tiers - Mac-side tonight; docs/PROMPT_COMPUTE_NIGHT_2026-11-11.md §0-§1): the tick instrument,
the bounded mode acks, the /recording.json cache, cv_publish at 20 Hz, the observer's writer thread, the bridge's reconnect params.
Every fix with its number before -> after (the DEVLOG blocks). QUICK: no broker."""
from __future__ import annotations

import time

from optiprimex.companion.config import Config
from optiprimex.usv.adapters.rover import RoverAdapter

from .test_frames_health_adapter import FakeLink


class DeafLink(FakeLink):
    """An FCU that never acks a mode: set_mode waits its whole timeout and fails (the real link's _wait_ack)."""

    def __init__(self):
        super().__init__()
        self.asked = []

    def set_mode(self, name, timeout_s=None):
        self.asked.append((name, timeout_s))
        time.sleep(timeout_s or 0.5)
        return False


def _ticks(a, link, n=20, dt=0.1):
    """n ticks of ensure_mode('GUIDED') at 10 Hz on a fake clock; -> (worst tick ms, total blocked s, link calls)."""
    t, worst, tot = [0.0], 0.0, 0.0
    a.clock = lambda: t[0]
    for _ in range(n):
        t0 = time.perf_counter()
        a.ensure_mode("GUIDED")
        ms = (time.perf_counter() - t0) * 1000.0
        worst, tot = max(worst, ms), tot + ms / 1000.0
        t[0] += dt
    return worst, tot, len(link.asked)


def test_a_deaf_fcu_blocks_the_tick_at_most_0_2_s_once_per_backoff(caplog):
    before = Config()
    before.timeouts.mode_ack_s, before.timeouts.mode_backoff_s = 0.5, 0.0           # the v0.1.86.4 behaviour: 0.5 s each tick, no backoff
    lb = DeafLink()
    w0, tot0, n0 = _ticks(RoverAdapter(lb, before), lb, n=6)
    la = DeafLink()
    w1, tot1, n1 = _ticks(RoverAdapter(la, Config()), la, n=20)                       # 2 s of ticks
    assert w0 > 450 and n0 == 6 and tot0 > 2.9                                         # before: every tick ~500 ms
    assert w1 < 260 and n1 == 1 and tot1 < 0.3, (w1, n1, tot1)                         # after: one 0.2 s wait, then the backoff
    assert la.asked[0][1] == 0.2
    print(f"\nDEAF FCU 10 Hz: before worst {w0:.0f} ms, {n0}/6 ticks asked; after worst {w1:.0f} ms, {n1}/20 ticks asked (backoff 2 s)")


def test_the_backoff_ends_and_an_ack_clears_it():
    cfg = Config()
    link = DeafLink()
    a = RoverAdapter(link, cfg)
    t = [0.0]
    a.clock = lambda: t[0]
    assert not a.ensure_mode("GUIDED") and len(link.asked) == 1
    t[0] = 1.9
    assert not a.ensure_mode("GUIDED") and len(link.asked) == 1                         # within the backoff: not asked
    t[0] = 2.1
    link.set_mode = lambda name, timeout_s=None: (link.asked.append((name, timeout_s)), setattr(link, "mode", name))[0] is None
    assert a.ensure_mode("GUIDED") and "GUIDED" not in a._backoff and len(link.asked) == 2


def test_the_tick_instrument():
    from optiprimex.usv.mission_manager import MissionManager
    mm = MissionManager.__new__(MissionManager)
    mm.cfg = Config()
    mm.tick = lambda: time.sleep(0.002)
    for _ in range(5):
        mm.timed_tick()
    st = mm.tick_stats()
    assert st["n"] == 5 and 1.5 <= st["p50"] <= 30 and st["slow_per_min"] == 0 and st["max"] >= st["p95"] >= st["p50"]
    mm.tick = lambda: time.sleep(0.16)
    mm.timed_tick()
    assert mm.tick_stats()["slow_per_min"] == 1


# ---- v0.1.86.5: GET /recording.json from memory ------------------------------------------------------------------------------------
def test_recording_json_never_waits_on_a_dead_agx():
    from optiprimex.tools import ocs_web as OW
    cam = OW.CamTab.__new__(OW.CamTab)
    calls = []

    def slow_ssh(cmd, timeout_s=None):                                  # a dead AGX: ssh hangs until its timeout
        calls.append(timeout_s)
        time.sleep(1.0)
        return "ssh: no answer"
    cam.ssh_run = slow_ssh
    cam.rec = type("R", (), {"status": lambda self: {"state": "idle"}})()
    cam.rec_targets = lambda: {}
    cam.REC_REFRESH_S = 0.05
    t0 = time.perf_counter()
    before = cam.recording_status()                                     # the v0.1.86.4 handler: the ssh in the request
    t_before = time.perf_counter() - t0
    lat = []
    for _ in range(5):
        t0 = time.perf_counter()
        r = cam.recording_cached()
        lat.append(time.perf_counter() - t0)
    assert t_before >= 1.0 and max(lat) < 0.05, (t_before, lat)
    assert r["agx"]["recording"] is False and "age_s" in r
    time.sleep(1.3)
    r = cam.recording_cached()
    assert r["age_s"] is not None and calls[-1] == cam.REC_SSH_TIMEOUT_S and before["agx"]["observer"] == "unknown"
    print(f"\n/recording.json with a dead AGX: before {t_before * 1000:.0f} ms per poll (ssh {1.0:g} s here, up to 20 s real), after {max(lat) * 1000:.1f} ms")


# ---- v0.1.86.6: cv_publish polls latest_status.json at 20 Hz ------------------------------------------------------------------------
def _poll_latency(tmp_path, hz: float, n: int = 40) -> float:
    """Mean ms from the observer's os.replace of latest_status.json to cv_publish's publish_payload (its real watch loop, a fake pub)."""
    import importlib.util
    import json as _json
    import os
    import threading
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    spec = importlib.util.spec_from_file_location(f"cvp_{int(hz)}", root / "cv" / "publish" / "cv_publish.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    out = tmp_path / f"out{int(hz)}"
    run = out / "run1"
    run.mkdir(parents=True)
    got = []
    m.publish_payload = lambda pub, payload: got.append((payload["k"], time.perf_counter()))
    pub = type("P", (), {"log": lambda self, s: None, "health": lambda self: None, "presence": lambda self: None})()
    a = type("A", (), {"output": str(out), "dock_output": None, "hz": hz})()
    threading.Thread(target=m.watch_observer, args=(pub, a), daemon=True).start()
    time.sleep(0.3)
    sent = {}
    import random
    rng = random.Random(4)
    for k in range(n):
        tmp = run / "latest_status.json.tmp"
        tmp.write_text(_json.dumps({"k": k, "pad": "x" * k}))
        sent[k] = time.perf_counter()
        os.replace(tmp, run / "latest_status.json")
        time.sleep(0.12 + rng.random() * 0.08)
    lat = [(t - sent[k]) * 1000.0 for k, t in got if k in sent]
    return sum(lat) / len(lat)


def test_cv_publish_at_20_hz_halves_the_poll_wait(tmp_path):
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / "cv" / "publish" / "cv_publish.py").read_text()
    assert 'ap.add_argument("--hz", type=float, default=20.0' in src
    l10, l20 = _poll_latency(tmp_path, 10.0), _poll_latency(tmp_path, 20.0)
    print(f"\ncv_publish file poll: 10 Hz mean {l10:.0f} ms -> 20 Hz mean {l20:.0f} ms (file written -> published)")
    assert l20 < l10 and l20 < 40


# ---- v0.1.86.7: the observer's slow writes off the loop ---------------------------------------------------------------------------
def _bgw():
    import importlib.util
    from pathlib import Path
    spec = importlib.util.spec_from_file_location("bg_writer", Path(__file__).resolve().parents[1] / "cv" / "buoys_multi" / "bg_writer.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


def test_a_stalled_disk_does_not_move_the_loop_and_never_loses_a_pair():
    B = _bgw()
    w = B.BgWriter(maxsize=4, keep_wait_s=0.5)
    done = []

    def slow(tag):
        def f():
            time.sleep(1.0)                                     # the disk stalls 1 s per write
            done.append(tag)
        return f
    per = []
    for k in range(20):                                         # 20 frames, a preview each, a pair on frames 0 and 10
        t0 = time.perf_counter()
        w.put("preview", slow(f"p{k}"))
        if k in (0, 10):
            w.put("pair", slow(f"pair{k}"), keep=True)
        per.append((time.perf_counter() - t0) * 1000.0)
    inline_before = 20 * 1000.0                                  # the v0.1.86.6 loop: every write inside the frame
    assert max(per) < 600 and sorted(per)[len(per) // 2] < 5, per   # the median frame pays nothing; a pair behind a full queue waits <= 0.5 s
    w.close(timeout_s=10.0)
    assert "pair0" in done and "pair10" in done and w.public()["dropped_previews"] > 10
    print(f"\nstalled disk (1 s / write): before {inline_before:.0f} ms of writes in 20 frames; after median {sorted(per)[10]:.2f} ms, "
          f"worst {max(per):.0f} ms per frame; pairs kept 2/2, previews dropped {w.public()['dropped_previews']}")


def test_the_observer_uses_the_writer_for_previews_and_pairs():
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert 'bgw.put("preview"' in src and 'bgw.put("pair"' in src and "keep=True" in src and 'payload["writer"] = bgw.public()' in src
    assert '"--bg-writer", choices=("on", "off"), default="on"' in src


# ---- v0.1.86.7: the bridges' keepalive ---------------------------------------------------------------------------------------------
def test_the_boat_bridges_notice_a_dead_peer_in_seconds():
    from pathlib import Path
    root = Path(__file__).resolve().parents[1]
    t = (root / "profiles" / "pool" / "mosquitto" / "boat_pool.conf.in").read_text()
    assert "#BRIDGE keepalive_interval 5\n" in t and "#UAVBRIDGE keepalive_interval 5\n" in t and "#BRIDGE restart_timeout 1 5" in t
    for f in ("boat.conf", "bridge.conf"):
        lines = (root / "profiles" / "bench" / "mosquitto" / f).read_text().splitlines()
        assert "keepalive_interval 5" in lines                                    # a line of its own (mosquitto has no trailing comments)
    assert (root / "scripts" / "bridge_keepalive_check.py").exists()


# ---- v0.1.86.8: the guard pipeline latency ----------------------------------------------------------------------------------------
def test_pipe_ms_from_cv_publish_to_the_boats_health():
    import importlib.util
    from pathlib import Path
    from optiprimex.usv.guard import GuardSupervisor
    spec = importlib.util.spec_from_file_location("cvp_lat", Path(__file__).resolve().parents[1] / "cv" / "publish" / "cv_publish.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    sent = []
    pub = type("P", (), {"c": type("C", (), {"publish": lambda self, t, env, qos=0: sent.append((t, env))})(), "ns": "local/usv",
                         "envelope": lambda self, b: b, "frame_to_items": lambda self, *a, **k: [], "pose_hint_from": lambda self, *a: None,
                         "observation": lambda self, *a, **k: None, "n_pub": 2, "log": lambda self, s: None})()
    m.observer_model = lambda pub, payload: None
    utc = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(time.time() - 0.12)) + ".%03dZ" % int(((time.time() - 0.12) % 1) * 1000)
    m.publish_payload(pub, {"updated_utc": utc, "guard": {"clear": True}, "buoys": []})
    g = [b for t, b in sent if t.endswith("/perception/guard")][0]
    assert 80 <= g["pipe_ms"] <= 1200 and "t_pub" in g
    sup = GuardSupervisor({})
    for k in range(10):
        sup.on_message({"clear": True, "pipe_ms": 100.0 + k, "t_pub": time.time() - 0.02}, 0.0)
    L = sup.latency()
    assert L["n"] == 10 and 100 <= L["pipe_ms_p50"] <= 109 and L["pipe_ms_p95"] >= L["pipe_ms_p50"] and 15 <= L["net_ms_p50"] <= 200
    assert GuardSupervisor({}).latency() is None
