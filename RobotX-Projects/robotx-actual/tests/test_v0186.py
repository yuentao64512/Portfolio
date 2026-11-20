"""v0.1.86 (WP42): the hosts feed (/hosts.json for session A's HOSTS tab), the disk warning, presets, the judge order, the defaults
(handover, auto start, logging at DECLARE), the hidden missions. docs/PROMPT_PRESETS_NIGHT_2026-11-12.md §2 + §6. QUICK: no broker."""
from __future__ import annotations

import json
import os

import pytest

from optiprimex.tools import host_health as HH
from optiprimex.tools.ocs_web import HostHealth

from .conftest import ROOT

NX_LINE = ("11-12-2026 01:02:03 RAM 5120/15600MB (lfb 2x4MB) SWAP 12/7800MB (cached 0MB) CPU [41%@1420,38%@1420,off,45%@1420,40%@1420,42%@1420] "
           "EMC_FREQ 9%@2133 GR3D_FREQ 7%@[305,305] NVDEC off NVJPG off VIC off OFA off APE 200 cpu@55C soc2@54.4C soc0@56.5C gpu@54.5C "
           "tj@56.5C soc1@55C VDD_IN 11200mW/11000mW VDD_CPU_GPU_CV 3100mW/3000mW VDD_SOC 2900mW/2800mW")
AGX_LINE = ("RAM 21500/30600MB (lfb 1x4MB) SWAP 0/15300MB (cached 0MB) CPU [63%@2201,60%@2201,70%@2201,65%@2201,61%@2201,59%@2201,62%@2201,66%@2201,"
            "60%@2201,64%@2201,63%@2201,61%@2201] EMC_FREQ 31%@3199 GR3D_FREQ 88%@1300 VIC_FREQ 0%@115 APE 174 CV0@-256C CPU@68C SOC2@66C SOC0@66C "
            "CV1@-256C GPU@71.5C tj@71.5C SOC1@66C CV2@-256C VDD_GPU_SOC 15200mW/15000mW VDD_CPU_CV 6000mW/6000mW VIN_SYS_5V0 7200mW/7100mW "
            "VDDQ_VDD2_1V8AO 4000mW/4000mW")


# ---- §2.5 the hosts feed ----------------------------------------------------------------------------------------------------------------
def test_tegrastats_lines_of_the_nx_and_the_agx():
    nx = HH.parse_tegrastats(NX_LINE)
    assert nx["ram_used_mb"] == 5120 and nx["ram_total_mb"] == 15600 and nx["swap_used_mb"] == 12
    assert nx["cpu_core_pct"] == [41, 38, None, 45, 40, 42] and nx["cpu_pct"] == pytest.approx(41.2)
    assert nx["gpu_pct"] == 7.0 and nx["emc_pct"] == 9.0
    assert nx["temp_c"]["gpu"] == 54.5 and nx["temp_c"]["tj"] == 56.5
    assert nx["power_w"] == {"total": 11.2, "cpu_gpu_cv": 3.1, "soc": 2.9}
    agx = HH.parse_tegrastats(AGX_LINE)
    assert agx["gpu_pct"] == 88.0 and len(agx["cpu_core_pct"]) == 12
    assert "cv0" not in agx["temp_c"] and agx["temp_c"]["gpu"] == 71.5                 # -256C = no sensor, never a reading
    assert agx["power_w"]["total"] == pytest.approx(32.4) and agx["power_w"]["cpu_gpu_cv"] == 6.0 and agx["power_w"]["soc"] == 15.2
    assert HH.parse_tegrastats("garbage") == {}


def test_the_recording_rate_from_the_folder_growth(tmp_path):
    r = HH.RecRate(str(tmp_path), every_s=0.0)
    (tmp_path / "a.svo2").write_bytes(b"x" * 1000)
    assert r.rate(now=0.0) is None
    (tmp_path / "a.svo2").write_bytes(b"x" * (1000 + 10_000_000))
    assert r.rate(now=30.0) is None                                     # < 60 s of samples
    (tmp_path / "a.svo2").write_bytes(b"x" * (1000 + 20_000_000))
    assert r.rate(now=60.0) == pytest.approx(0.2)                       # 20 MB / 60 s = 0.2 GB / 10 min
    assert HH.RecRate(str(tmp_path / "missing")).rate() is None


def test_a_beat_lands_in_hosts_json_in_the_consoles_shape():
    hh = HostHealth()
    body = dict(HH.parse_tegrastats(AGX_LINE), host="agx", hostname="optiprime-agxorin", cpu_pct=63.0, disk_free_gb=12.4, rec_gb_per_10min=1.6,
                power_mode="MAXN", throttling=False, throttle_src=None, fan_pct=70)
    env = {"kind": "telemetry", "body": body}
    for k in range(5):
        assert hh.on_msg("team/optiprime/v1/telemetry/usv/host/agx", json.dumps(env).encode(), now=100.0 + k)
    d = hh.hosts_body(now=105.0)
    a = d["hosts"]["agx"]
    want = json.loads((ROOT / "tests" / "fixtures" / "hosts_sample.json").read_text())["hosts"]["agx"]
    assert set(want) <= set(a) and set(want["last"]) <= set(a["last"]) and set(want["hist"]) <= set(a["hist"])   # session A's fixture = the spec
    assert a["host"] == "optiprime-agxorin" and a["age_s"] == 1.0 and a["power_mode"] == "MAXN"
    assert a["last"]["disk_free_gb"] == 12.4 and a["last"]["rec_gb_per_10min"] == 1.6 and a["last"]["temp_max_c"] == 71.5
    assert len(a["hist"]["cpu_pct"]) == 5 and a["hist"]["ram_pct"][-1] == pytest.approx(70.3) and a["hist"]["power_w"][-1] == pytest.approx(32.4)
    for k in range(700):                                                # the ring: 600 newest
        hh.put("nx", {"cpu_pct": float(k)}, now=200.0 + k)
    h = hh.hosts_body(now=900.0)["hosts"]["nx"]["hist"]["cpu_pct"]
    assert len(h) == 600 and h[-1] == 699.0
    assert hh.hosts_body()["disk_amber_gb"] == 15.0


def test_the_disk_warning_is_amber_below_15_and_red_below_8_with_minutes_left():
    hh = HostHealth()
    hh.put("agx", {"disk_free_gb": 12.4, "rec_gb_per_10min": 1.6}, now=10.0)
    hh.put("nx", {"disk_free_gb": 4.7}, now=10.0)
    w = {x["text"].split()[0]: x for x in hh.warnings(now=11.0)}
    assert w["AGX"]["style"] == "warn" and "~78 min of recording" in w["AGX"]["text"]
    assert w["NX"]["style"] == "bad" and "< 8 GB" in w["NX"]["text"]


def test_the_agx_launcher_says_the_disk_and_passes_the_rec_dir():
    s = (ROOT / "scripts" / "run_agx.sh").read_text()
    assert "--rec-dir $REC_DIR" in s and "AGX DISK RED" in s and "< 15 GB" in s


def test_host_health_once_on_this_machine_has_the_new_fields():
    b = HH.Sampler(disk_path=str(ROOT))
    b.tegra, b.rec = None, None
    body = b.body("mac")
    for k in ("ram_used_mb", "ram_total_mb", "cpu_core_pct", "power_w", "fan_pct", "power_mode", "temp_max_c", "rec_gb_per_10min"):
        assert k in body
    assert isinstance(body["disk_free_gb"], float) and os.path.isdir(str(ROOT))


# ---- §2.1 presets, §2.2 the judge order, the launch block ----------------------------------------------------------------------------
from optiprimex.tools import launcher as L                                   # noqa: E402

NET = {"ORIN_SSH": "nx", "AGX_SSH": "agx", "BOAT_HOST": "10.0.0.3", "AGX_HOST": "10.0.0.4", "OCS_HOST": "10.0.0.2", "VENUE_RC_HOST": "192.168.70.74",
       "VENUE_RC_PORT": "1910"}


def test_the_night_presets_render_and_validate():
    P = L.load_presets(str(ROOT / "profiles" / "presets.yaml"))
    assert list(P) == ["sim", "lab", "bench-uavmap", "m1t2-uav", "m1t2-both", "m1t1-usv", "m1t1-gates", "custom"]   # event removed
    for k, c in P.items():
        assert L.validate(dict(c, preset=k)) == [], k
    assert P["lab"]["title"].startswith("lab (bench)") and P["sim"]["world"] == "marina" and P["sim"]["mission"] == "scout"
    assert (P["m1t2-uav"]["mission"], P["m1t2-uav"]["auto_mission"], P["m1t2-both"]["auto_mission"]) == ("m1t2", False, True)
    assert P["m1t1-usv"]["mission"] == "scout" and P["m1t1-gates"]["mission"] == "m1t1" and P["m1t1-usv"]["tiers"]["T1"] == "CORE"
    assert all(P[k]["judge"] == "venue" and P[k]["record"] == "auto" for k in ("m1t2-uav", "m1t2-both", "m1t1-usv", "m1t1-gates"))
    assert L.FIELDS["judge"][0] == "venue" and L.FIELDS["judge"][-1] == "rctest"
    assert L.validate(dict(P["custom"], mission="banana"))[0].startswith("mission:")
    assert L.validate(dict(P["custom"], tiers={"T9": "CORE"}))[0].startswith("tiers:")


def test_the_launch_block_reaches_the_ocs_profile_copy_and_the_observer_records_pairs():
    P = L.load_presets(str(ROOT / "profiles" / "presets.yaml"))
    cfg = dict(P["m1t1-usv"], preset="m1t1-usv")
    lb = L.launch_block(cfg)
    assert lb == {"preset": "m1t1-usv", "mission": "scout", "auto": True, "record": "auto", "task": "SCOUT", "extra": {},
                  "tiers": {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_NONE", "T4": "TIER_NONE"}}
    plan = L.build_plan(cfg, stamp="S", net_env=NET, pre={"clocks": {}, "nic": None, "refuse": []})
    f = [x for x in plan["files"] if x["path"].endswith("ocs.yaml")]
    assert f and "launch: " in f[0]["text"] and any("OCS_CONFIG_OVERRIDE=logs/launch/S/ocs.yaml" in s["cmd"] for s in plan["steps"])
    assert any("agx_observer.sh start --detector default --pairs-s 10" in s["cmd"] for s in plan["steps"])
    assert any("starts by itself at RunStart" in n for n in plan["notes"])
    import yaml
    assert yaml.safe_load(f[0]["text"])["launch"]["mission"] == "scout"
    u = L.launch_block(dict(P["m1t2-uav"], preset="m1t2-uav"))
    assert u["auto"] is False and u["extra"] == {"tier": "advanced"}
    assert L.record_mode(dict(P["sim"])) == "off" and L.record_mode({"cv": "fake", "record": "auto", "mode": "real"}) == "off"
    assert L.launch_block({"mode": "real", "cv": "real", "mission": "none"}) is None          # a v0.1.85 config: no block, as before


def test_reach_dots_never_carry_an_address():
    seen = []
    r = L.reach(ne={"AGX_HOST": "10.0.0.4", "BOAT_HOST": "10.0.0.3", "RC_BRAVO_HOST": "192.168.66.2"}, up=lambda h, p: seen.append((h, p)) or h == "10.0.0.3")
    assert r["jetsons"]["usv_nx"] == {"label": "USV-NX", "up": True} and r["jetsons"]["agx"]["up"] is False
    assert r["judges"]["bravo"] is False and ("192.168.66.2", 1883) in seen and ("10.0.0.4", 22) in seen
    assert "10.0." not in json.dumps(r) and "192.168" not in json.dumps(r)
    from optiprimex.tools import launcher_tk as LT
    assert LT.reach_line(r).startswith("AGX ○ · USV-NX ● · UAV-NX") and LT.tiers_line({"tiers": {"T1": "CORE"}}) == "tiers: T1 CORE · T4 NONE"
    assert {k for _, k, *_ in LT.ROWS} >= {"mission", "auto_mission", "record"}


# ---- §2.3 the defaults --------------------------------------------------------------------------------------------------------------
def _app(launch):
    from optiprimex.companion.config import Config
    from optiprimex.ocs.ocs_app import OcsApp
    from .test_ocs_units import FakeBus
    from .test_v0164 import RJudge
    import yaml
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "course" / "ocs_course.yaml").read_text())
    cfg.extra["launch"] = launch
    return OcsApp(cfg, bus=FakeBus(), judge=RJudge())


def test_the_ocs_preselects_the_presets_mission_and_declares_its_tier_row_without_the_t4_prompt():
    app = _app({"preset": "m1t1-usv", "mission": "scout", "task": "SCOUT", "extra": {}, "auto": True,
                "tiers": {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_NONE", "T4": "TIER_NONE"}})
    app.apply_launch()
    assert app.mission["task"] == "SCOUT" and app.mission["auto"] is True and app._mission_body()["auto"] is True
    assert app.effective_tiers() == {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_NONE", "T4": "TIER_NONE"}
    rows = app.declaration_rows("declare")
    assert not [r for r in rows if r["code"] == "mandatory_none"], rows                  # the preset's answer to "are you sure?" is yes
    plain = _app({})
    assert [r for r in plain.declaration_rows("declare", args={"tiers": {"T4": "TIER_NONE"}}) if r["code"] == "mandatory_none"]   # without a preset: asked
    app.set_mission(None, False)                                                         # `ocs_cli mission off` still clears it
    assert app.mission["task"] is None


def test_logging_starts_by_itself_at_declare_and_stops_at_the_runs_end(tmp_path, monkeypatch):
    from optiprimex.tools import ocs_web as OW
    monkeypatch.setenv("OCS_LOG_SESSIONS", str(tmp_path))
    web = OW.Web.__new__(OW.Web)
    web.env, web.launch, web.log_session = {"OCS_LOG_SESSIONS": str(tmp_path)}, {"preset": "m1t1-usv", "record": "off"}, {"running": False, "auto": False}
    web.m = type("M", (), {"ocs": None, "add": lambda self, *a: None})()
    calls = []
    web._boat_word = lambda w, a: calls.append((w, a)) or {"ok": True, "text": f"{w} {' '.join(a)}"}
    web.logging_state = lambda now=None: {"usv": {"on": False, "file": None}}
    web.clocks_state = lambda now=None: {"nx": None}
    web.clock_refusal = lambda c: "NX unknown"                                          # the clocks never block the automatic session
    web.camtab = None
    assert web.auto_log_tick({"r": "R2", "declaration_seq": None}) is None
    r = web.auto_log_tick({"r": "R3", "declaration_seq": 7, "mission": {"code": "scout"}})
    assert r["ok"] and web.log_session["auto"] and web.log_session["label"].startswith("m1t1-usv_scout_7_")
    assert calls[0][0] == "log" and web.log_session["parts"]["agx"]["note"].startswith("record off")
    assert web.auto_log_tick({"r": "R5", "declaration_seq": 7}) is None                  # the same declaration: nothing new
    r = web.auto_log_tick({"r": "R6", "declaration_seq": 7})
    assert r["ok"] and not web.log_session["running"] and calls[-1] == ("log", ["off"])
    files = list(tmp_path.glob("m1t1-usv_scout_7_*.json"))
    assert files and json.loads(files[0].read_text())["auto"] is True
    web.log_session = {"running": True, "label": "mine", "t0": "x", "auto": False}        # a session the operator started: kept, not doubled
    r = web.auto_log_tick({"r": "R3", "declaration_seq": 8})
    assert r["already"] and web.log_session["label"] == "mine"


def test_handover_at_stack_start_and_the_boat_logger_on_by_default():
    s = (ROOT / "scripts" / "run_usv.sh").read_text()
    assert 'HANDOVER="${USV_HANDOVER:-1}"' in s and "--no-handover) HANDOVER=0" in s and 'UI_ARGS="$UI_ARGS --handover"' in s
    assert 'LOG="${LOG:-1}"' in s and "--client-id logger-boat" in s


def test_the_hidden_missions_and_meta():
    from optiprimex.tools import playbook
    assert playbook.visible_missions() == ["m4t0", "m1t1", "m1t2", "scout", "m3t1", "m0t4", "m0t8", "m0t9", "m0t10"]
    assert all(playbook.MISSIONS[c].hidden for c in ("goto", "m0t0", "m0t1", "m0t2", "m0t3", "m0t6"))
    assert playbook.resolve("m0t2")[1] == "TEST_PATH"                                    # typed still works
    src = (ROOT / "optiprimex" / "tools" / "ocs_web.py").read_text()
    assert '"hidden": bool(getattr(m, "hidden", False))' in src


def test_make_day_exists():
    mk = (ROOT / "Makefile").read_text()
    assert "\nday: venv check-mosquitto" in mk and "DAY_LIVE =" in mk
