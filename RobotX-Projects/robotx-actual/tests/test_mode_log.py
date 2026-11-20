"""The command trace and its analyzer (v0.1.27.1, the no-hardware part of ROADMAP WP9): the RoverAdapter records mode requests and
frames, the mission manager the stages; tools/mode_log.py turns them into the mode sequence per stage and checks the lab / pool rules.
(The live run through the mission manager is in tests/test_box_sim.py.)"""
from __future__ import annotations

import json
import subprocess

from optiprimex.companion.config import Config
from optiprimex.companion.state import Hold, ThrusterFrame
from optiprimex.tools import mode_log
from optiprimex.usv.adapters.rover import RoverAdapter
from optiprimex.usv.cmd_trace import CmdTrace

from .conftest import PY, ROOT


def write(path, head, evs):
    with open(path, "w") as fh:
        fh.write(json.dumps({"k": "start", "t": 0.0, "unix": 1.7e9, "profile": head, "version": "x", "fcu": "uart"}) + "\n")
        for e in evs:
            fh.write(json.dumps(e) + "\n")
    return str(path)


def obs(t, state="AUTO_TASK", task="TEST_SWAY", sub="SWAY", mode="CUSTOM_MANUAL", armed=False):
    return {"k": "obs", "t": t, "fcu_mode": mode, "armed": armed, "state": state, "task": task, "sub": sub}


def good_run(armed=False):
    """HOLD -> mode 20 (zero frame, 10 Hz frames, neutral) -> HOLD: the GUIDE §6.2 sequence for m0t3."""
    ev = [obs(0.0, "AUTO_HOLD", None, None, "HOLD", armed), {"k": "mode_req", "t": 0.1, "mode": "HOLD", "ok": True},
          obs(1.0, "AUTO_TASK", "TEST_SWAY", "SWAY", "HOLD", armed), {"k": "mode_req", "t": 1.0, "mode": "CUSTOM_MANUAL", "ok": True},
          {"k": "frame", "t": 1.01, "m": [0, 0, 0, 0], "why": "entry"}, obs(1.1, mode="CUSTOM_MANUAL", armed=armed)]
    ev += [{"k": "frame", "t": round(1.02 + 0.1 * i, 2), "m": [0, 0.2, 0, 0.2], "why": "cmd"} for i in range(30)]
    ev += [obs(4.2, "AUTO_HOLD", None, None, "CUSTOM_MANUAL", armed), {"k": "frame", "t": 4.25, "m": [0, 0, 0, 0], "why": "neutral"},
           {"k": "mode_req", "t": 4.26, "mode": "HOLD", "ok": True}, obs(4.5, "AUTO_HOLD", None, None, "HOLD", armed)]
    return ev


def test_a_clean_lab_run_passes_and_gives_the_table(tmp_path):
    r = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "t.jsonl", "usv_lab.yaml", good_run())), "lab")
    assert r["ok"] and r["n_warn"] == 0, mode_log.render_text(r)
    sway = next(s for s in r["stages"] if s.sub == "SWAY")
    assert sway.modes_req == ["CUSTOM_MANUAL"] and sway.modes_fcu == ["HOLD", "CUSTOM_MANUAL"]
    assert sway.frames == 30 and 9.0 <= sway.rate() <= 10.5 and sway.max_gap <= 0.11
    md = mode_log.render_md(r)
    assert "| AUTO_TASK TEST_SWAY/SWAY | CUSTOM_MANUAL | HOLD → CUSTOM_MANUAL | 30 (" in md and md.count("\n") == 4


def test_the_rules_catch_thrash_gaps_missing_zero_frames_and_the_wrong_profile(tmp_path):
    ev = good_run()
    ev.insert(8, {"k": "frame", "t": 1.29, "m": [0, 0.2, 0, 0.2], "why": "cmd"})
    bad = [e for e in ev if not (e["k"] == "frame" and 1.5 < e["t"] < 1.85)]               # a 0.4 s hole
    r = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "gap.jsonl", "usv_lab.yaml", bad)), "lab")
    assert not r["ok"] and any(f.check == "gap" and f.level == "FAIL" for f in r["findings"])
    no_zero = [e for e in good_run() if e.get("why") != "entry"]
    r = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "z.jsonl", "usv_lab.yaml", no_zero)), "lab")
    assert any(f.check == "entry" for f in r["findings"]) and not r["ok"]
    flap = [obs(0.0, mode="HOLD")] + [{"k": "mode_req", "t": 0.2 * i, "mode": ("HOLD", "CUSTOM_MANUAL")[i % 2], "ok": True} for i in range(1, 6)]
    r = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "f.jsonl", "usv_lab.yaml", flap)), "lab")
    assert any(f.check == "thrash" and "HOLD -> CUSTOM_MANUAL" in f.text for f in r["findings"]) and not r["ok"]
    r = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "p.jsonl", "usv_lab.yaml", good_run(armed=True))), "pool")
    assert any(f.check == "profile" and f.level == "FAIL" for f in r["findings"])            # --pool on a lab run


def test_lab_and_pool_rules_differ(tmp_path):
    """Pool: AUTO_TASK disarmed or a refused mode = FAIL; lab: armed = WARN (props off?), a refused GUIDED = WARN."""
    p = write(tmp_path / "pool.jsonl", "usv_pool.yaml", good_run(armed=False))
    r = mode_log.analyse(*mode_log.load_trace(p), "pool")
    assert [f.check for f in r["findings"] if f.level == "FAIL"] == ["armed"]                # once per stage
    assert mode_log.analyse(*mode_log.load_trace(write(tmp_path / "pa.jsonl", "usv_pool.yaml", good_run(armed=True))), "pool")["ok"]
    ev = good_run(armed=True) + [{"k": "mode_req", "t": 5.0, "mode": "GUIDED", "ok": False}]
    lab = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "l.jsonl", "usv_lab.yaml", ev)), "lab")
    assert lab["ok"] and {f.check for f in lab["findings"]} == {"armed", "refused"}
    pool = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "pr.jsonl", "usv_pool.yaml", ev)), "pool")
    assert not pool["ok"] and any(f.check == "refused" and f.level == "FAIL" for f in pool["findings"])


def test_the_fcu_must_follow_a_request(tmp_path):
    ev = good_run()
    for e in ev:                                          # the FCU never leaves HOLD
        if e["k"] == "obs":
            e["fcu_mode"] = "HOLD"
    r = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "nf.jsonl", "usv_pool.yaml", [dict(e, armed=True) for e in ev])), "pool")
    assert any(f.check == "follow" and "CUSTOM_MANUAL, the FCU reported it never" in f.text for f in r["findings"]) and not r["ok"]
    # v0.1.28.1: a request in the trace's last second (the run stopped right after it) is not seen yet - WARN, not FAIL
    tail = [dict(e, armed=True) for e in good_run()][:-1]                     # cut the FCU's HOLD at 4.5 s; the trace ends at 4.26
    r = mode_log.analyse(*mode_log.load_trace(write(tmp_path / "tail.jsonl", "usv_pool.yaml", tail)), "pool")
    assert r["ok"] and any(f.check == "follow" and f.level == "WARN" and "not seen yet" in f.text for f in r["findings"]), mode_log.render_text(r)


class _Link:
    def __init__(self):
        self.mode = "HOLD"

    def state(self):
        from types import SimpleNamespace
        return SimpleNamespace(mode_name=self.mode)

    def set_mode(self, name, timeout_s=None):
        self.mode = name
        return True

    def send_actuator_frame(self, f):
        pass


def test_the_adapter_writes_the_trace_and_the_cli_reads_it(tmp_path):
    ad = RoverAdapter(_Link(), Config())
    ad.trace = CmdTrace(str(tmp_path / "mm_trace.jsonl"), profile="usv_lab.yaml", fcu="uart")
    ad.trace.observe("HOLD", False, "AUTO_TASK", "TEST_SWAY", "SWAY")
    for _ in range(3):
        ad.apply(ThrusterFrame(0.0, 0.2, 0.0, 0.2))
    ad.apply(Hold())
    ad.trace.close()
    lines = [json.loads(x) for x in open(tmp_path / "mm_trace.jsonl")]
    assert [x["k"] for x in lines] == ["start", "obs", "mode_req", "frame", "frame", "frame", "frame", "frame", "mode_req"]
    assert [x.get("why") for x in lines if x["k"] == "frame"] == ["entry", "cmd", "cmd", "cmd", "neutral"]
    assert lines[0]["profile"] == "usv_lab.yaml" and lines[2]["mode"] == "CUSTOM_MANUAL" and lines[-1]["mode"] == "HOLD"
    out = subprocess.run([PY, "-m", "optiprimex.tools.mode_log", str(tmp_path / "mm_trace.jsonl"), "--md"], cwd=ROOT,
                         capture_output=True, text=True, timeout=30)
    assert out.returncode == 0 and "| usv_lab.yaml | AUTO_TASK TEST_SWAY/SWAY | CUSTOM_MANUAL → HOLD |" in out.stdout, out.stdout + out.stderr
    bad = subprocess.run([PY, "-m", "optiprimex.tools.mode_log", str(tmp_path / "none.jsonl")], cwd=ROOT, capture_output=True, text=True)
    assert bad.returncode == 2


def test_run_usv_takes_one_profile_flag():
    out = subprocess.run(["bash", str(ROOT / "scripts" / "run_usv.sh"), "--pool", "--lab"], capture_output=True, text=True, timeout=20)
    assert out.returncode == 2 and "pick one profile" in out.stderr
    h = subprocess.run(["bash", str(ROOT / "scripts" / "run_usv.sh"), "--help"], capture_output=True, text=True, timeout=20).stdout
    assert "--pool" in h and "mm_trace.jsonl" in h
