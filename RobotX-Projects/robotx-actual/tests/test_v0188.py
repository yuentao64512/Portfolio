"""v0.1.88 (Dmitrii 2026-11-12: "I want to have in launcher in presets tick box for T4, off by default for every mission"): the launcher's
`t4` field (T4 Dynamic Incident Response) - off in every preset, a tick box in the tk and the web launcher, always sent to the OCS as the
T4 tier (on = TIER_CORE, off = TIER_NONE), winning over the preset's tier row; an operator's `mission ... --t4` in the OCS still wins."""
from __future__ import annotations

from optiprimex.tools import launcher as L
from optiprimex.tools import launcher_tk as K

from .conftest import ROOT


def test_t4_off_in_every_preset():
    P = L.load_presets(str(ROOT / "profiles" / "presets.yaml"))
    assert len(P) >= 8
    for name, p in P.items():
        assert p.get("t4") is False, name
        lb = L.launch_block(dict(p, preset=name))
        assert lb["tiers"]["T4"] == "TIER_NONE", name


def test_the_tick_box_sets_t4_core_and_wins_over_the_tier_row():
    P = L.load_presets(str(ROOT / "profiles" / "presets.yaml"))
    on = L.launch_block(dict(P["m1t2-both"], preset="m1t2-both", t4=True))
    assert on["tiers"] == {"T1": "TIER_ADVANCED", "T2": "TIER_NONE", "T3": "TIER_NONE", "T4": "TIER_CORE"}
    sim = L.launch_block(dict(P["sim"], preset="sim"))                       # tiers: profile -> only T4 sent, the rest the profile's
    assert sim["tiers"] == {"T1": "TIER_CORE", "T4": "TIER_NONE"}                           # v0.1.90.4: T1 from the mission (scout)
    assert L.validate(dict(P["custom"], t4="yes"))[0].startswith("t4:")
    assert L.validate(dict(P["custom"], t4=True)) == []


def test_the_tick_box_in_both_launchers():
    assert "t4" in K.BOOL_KEYS and any(r[1] == "t4" and r[3] == "bool" for r in K.ROWS)
    assert K.tiers_line({"tiers": "profile"}) == "tiers: profile · T4 NONE"
    assert K.tiers_line({"tiers": {"T1": "CORE", "T4": "NONE"}, "t4": True}) == "tiers: T1 CORE · T4 CORE"
    html = (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    assert 'data-k="t4" data-bool="1"' in html


def test_the_operators_t4_wins_over_the_launchers():
    src = (ROOT / "optiprimex" / "ocs" / "ocs_app.py").read_text()
    assert src.count("tiers = self._t4_flag(self._t1_flag(tiers))") == 2                 # v0.1.90.2: the armed mission's T1 / T4 last
    k = src.index("def _declaration_inputs")
    body = src[k:src.index("def ", k + 10)]
    assert body.index("tiers.update(self.launch_tiers())") < body.index("tiers = self._t4_flag(self._t1_flag(tiers))")   # wire = preview


def test_the_armed_mission_decides_t1_over_the_presets_row():
    """v0.1.90.2 (venue 17:10): launched m1t2-both (row T1 ADVANCED), then `mission m1t1` armed -> DECLARE was refused (m1t1 needs T1
    CORE, the declaration said ADVANCED). The armed mission's T1 (and --t4) now come after the preset's row, preview and wire alike."""
    import types
    from optiprimex.ocs.ocs_app import OcsApp
    o = object.__new__(OcsApp)
    o.lifecycle = types.SimpleNamespace(declaration=None)
    o.declaration_cfg = {"tiers": {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_CORE"}}
    o.launch = {"tiers": {"T1": "ADVANCED", "T2": "NONE", "T3": "NONE", "T4": "NONE"}}
    o.declare_uav = False
    o.mission = {"code": "m1t1", "t4": None}
    assert o.effective_tiers() == {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_NONE", "T4": "TIER_NONE"}
    o.mission = {"code": "m1t2", "t4": "TIER_CORE"}
    assert o.effective_tiers()["T1"] == "TIER_ADVANCED" and o.effective_tiers()["T4"] == "TIER_CORE"


def test_the_launcher_derives_t1_from_the_mission_and_declares_the_uav_for_m1t2():
    """v0.1.90.4 (venue 17:08 / 17:09): preset m1t2-both with its mission field changed to m1t1 sent T1 ADVANCED; and every m1t2 preset
    needed a separate DECLARE UAV press (five refusals 16:40-17:06)."""
    P = L.load_presets(str(ROOT / "profiles" / "presets.yaml"))
    lb = L.launch_block(dict(P["m1t2-both"], preset="m1t2-both", mission="m1t1"))
    assert lb["tiers"]["T1"] == "TIER_CORE" and "declare_uav" not in lb
    for k in ("m1t2-both", "m1t2-uav"):
        lb = L.launch_block(dict(P[k], preset=k))
        assert lb["tiers"]["T1"] == "TIER_ADVANCED" and lb["declare_uav"] is True, k
    assert L.launch_block(dict(P["m1t1-gates"], preset="m1t1-gates"))["tiers"]["T1"] == "TIER_CORE"
    assert "declare_uav" not in L.launch_block(dict(P["m1t2-both"], preset="m1t2-both", uav="off"))


def test_the_uav_pilot_ready_comes_by_itself():
    """v0.1.90.5 (Dmitrii: "remove the pilot ready buttons completely"): UAV_READY on by itself - declared, adapter up, before DECLARE
    (R1 / R2 / R6), not ready yet; every 5 s at most; never during a declared run; `uav_pilot_auto: false` = off."""
    import types
    from optiprimex.ocs.ocs_app import OcsApp
    sent = []
    o = object.__new__(OcsApp)
    o.uav_vid, o.declare_uav, o.uav_auto_ready, o._uav_ready_t = "UAV1", True, True, -1e9
    o.last_status = {"UAV1": {"health": {"pilot_ready": False}}}
    r = {"r": "R2"}
    o.lifecycle = types.SimpleNamespace(body=lambda: r)
    o.uav_active = lambda: True
    o.on_uav = lambda op, a: sent.append((op, a)) or (True, "ok")
    o.uav_ready_tick()
    o.uav_ready_tick()                                                   # within 5 s: once
    assert sent == [("ready", {"on": True})]
    o._uav_ready_t, r["r"] = -1e9, "R5"
    o.uav_ready_tick()
    assert len(sent) == 1                                                # never during a run
    r["r"], o.last_status["UAV1"]["health"]["pilot_ready"] = "R2", True
    o.uav_ready_tick()
    assert len(sent) == 1                                                # already ready
    src = (ROOT / "optiprimex" / "tools" / "tkconsole" / "views.py").read_text()
    assert '"usv-ready"' not in src and '"uav-ready"' not in src and 'USV_ROWS = [("crew", ["usv-hb"]), ("autonomy"' in src



def test_the_uav_heartbeat_toggle_works_while_declare_uav_is_on():
    """v0.1.90.6: an m1t2 launch turns declare uav on (heartbeats with it); `report off uav` must still silence the drone before DECLARE."""
    import types
    from optiprimex.ocs.ocs_app import OcsApp
    o = object.__new__(OcsApp)
    o.vehicles, o.uav_vid, o.primary_vid = ["USV1", "UAV1"], "UAV1", "USV1"
    o.report_usv, o.report_uav, o.declare_uav = True, False, True
    o.lifecycle = types.SimpleNamespace(declaration=None, body=lambda: {"r": "R2"})
    o.uav_declared = lambda: False
    o.heartbeats = types.SimpleNamespace(vehicles=[])
    assert o.hb_vehicles() == ["USV1", "UAV1"]
    assert o.on_reporting(False, "uav") is None and o.hb_vehicles() == ["USV1"]
    assert o.on_reporting(True, "uav") is None and o.hb_vehicles() == ["USV1", "UAV1"]
    o.on_reporting(False, "uav")
    o.uav_declared = lambda: True
    assert o.hb_vehicles() == ["USV1", "UAV1"]                           # declared: at all times (handbook)
