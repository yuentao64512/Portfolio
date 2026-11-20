"""v0.1.66 - WP16: the UAV's buoys fused into the boat's map, m1t2 = T1 Advanced (docs/OVERNIGHT_2026-11-07_UAV.md D20-D23).
ingest_geo (lat/lon -> local, the 1 m link, the boat keeps its position, the UAV's colour under colour_source uav and for an unknown
camera vote, the majority of the UAV's sightings, the disagreement flag, UAV-only buoys, the merge, forget / frozen), the planner's view
(UAV-only out of gates / markers, in the obstacles with their sigma), the leads, WAIT_UAV_MAP's three exits and its hold, m1t2's precheck
and the T1 tier following the mission, the colour-blind world, the bridge rendering, the adapter's relay, the fake UAV, the regression
guard (no UAV item: m1t1 unchanged), m1t2 DONE in the fast SIL. QUICK: no broker."""
from __future__ import annotations

import json
import math
import subprocess

import pytest
import yaml

from optiprimex.companion.config import Config
from optiprimex.companion.frames import ControlOrigin
from optiprimex.companion.state import LocalPose, Observation, ObsItem
from optiprimex.ocs.ocs_app import OcsApp
from optiprimex.sim import fake_uav as FU
from optiprimex.sim import world as W
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import ocs_cli
from optiprimex.usv.gatepath import planner as gp
from optiprimex.usv.perception_map import PerceptionMap

from .conftest import ROOT
from .test_ocs_units import FakeBus, intent
from .test_v0164 import RJudge, SQ, UID

O = ControlOrigin(1.2810202, 103.8550097)
T1ADV = str(ROOT / "profiles" / "bench" / "world_extra_t1adv.yaml")


def ll(x, y):
    k = 6378137.0 * math.pi / 180.0
    return O.lat + x / k, O.lon + y / (k * math.cos(math.radians(O.lat)))


def msg(items, fid="f1", state="scanning"):
    return {"t": "2026-11-08T00:00:00Z", "source": "uav", "frame_id": fid, "uav": {"lat": O.lat, "lon": O.lon, "alt_hae_m": 10, "hdg_deg": 0},
            "scan": {"state": state, "coverage_pct": 50, "n_items": len(items)}, "items": items}


def item(x, y, colour, lit="flash", uid=None):
    lat, lon = ll(x, y)
    return {"id": uid, "lat": lat, "lon": lon, "class": "buoy", "colour": colour, "lit": lit, "conf": 0.9, "sigma_m": 0.7}


def camera(pm, x, y, colour="unknown", n=6, t0=100.0):
    pose = LocalPose(0.0, 0.0, 0.0, O.lat, O.lon, t0)
    for i in range(n):
        r, b = math.hypot(x, y), math.degrees(math.atan2(y, x))
        pm.ingest(Observation(kind="buoys", t_cam=None, items=[ObsItem(id=0, cls="buoy", colour=colour, conf=0.9, range_m=r, bearing_deg=b)]),
                  pose, now=t0 + i)


def pmap():
    cfg = Config()
    cfg.frames.cam_forward_m = cfg.frames.cam_right_m = cfg.frames.cam_yaw_deg = 0.0
    return PerceptionMap(cfg, clock=lambda: 200.0)


# ---- ingest_geo -------------------------------------------------------------------------------------------------------------------
def test_ingest_geo_links_within_1m_and_the_boat_keeps_its_position():
    pm = pmap()
    camera(pm, 20.0, 5.0)
    e = pm.entities("buoy")[0]
    x0, y0 = e.x_n, e.y_e
    assert e.colour == "unknown"
    assert pm.ingest_geo(msg([item(20.6, 5.3, "red", uid="u3")]), O, 120.0) == 1
    assert (e.x_n, e.y_e) == (x0, y0) and e.colour == "red" and e.light == "flash"     # position kept, colour taken (camera unknown)
    assert e.uav["id"] == "u3" and pm.uav_stats["linked"] == 1 and not pm.uav_entities()
    assert pm.ingest_geo(msg([item(20.6, 5.3, "red")], fid="f1"), O, 121.0) == 0      # the same frame_id twice (two paths): once
    # 1.4 m away: not linked -> a UAV-only buoy at the UAV's position
    pm.ingest_geo(msg([item(21.4, 5.0, "green", uid="u4")], fid="f2"), O, 122.0)
    uo = [q for q in pm.ents.values() if q.source == "uav"]
    assert len(uo) == 1 and abs(uo[0].x_n - 21.4) < 0.01 and uo[0].colour == "green" and uo[0].sigma_m == 0.7
    d = pm.digest(now=125.0)
    assert d["uav"]["stats"]["linked"] == 1 and d["uav"]["stats"]["uav_only"] == 1
    ent = next(x for x in d["entities"] if x["id"] == e.id)
    assert ent["uav_colour"] == "red" and ent["linked"] is True and ent["source"] == "usv"


def test_the_camera_keeps_its_colour_unless_m1t2_and_the_uav_colour_is_a_vote():
    pm = pmap()
    camera(pm, 15.0, 0.0, colour="green", n=8)
    e = pm.entities("buoy")[0]
    assert e.colour == "green"
    pm.ingest_geo(msg([item(15.2, 0.1, "red")]), O, 120.0)
    assert e.colour == "green" and e.uav_disagree and pm.digest(now=121.0)["entities"][0]["disagree"]   # camera stays; flagged
    pm.colour_source = "uav"
    pm.ingest_geo(msg([item(15.2, 0.1, "red")], fid="f2"), O, 121.0)
    assert e.colour == "red" and not e.uav_disagree
    for i in range(3):                                           # the majority, not the last message
        pm.ingest_geo(msg([item(15.2, 0.1, "red")], fid=f"r{i}"), O, 122.0 + i)
    pm.ingest_geo(msg([item(15.2, 0.1, "blue")], fid="w"), O, 130.0)
    assert e.colour == "red" and e.uav["votes"] == {"red": 5, "blue": 1}
    camera(pm, 15.0, 0.0, colour="green", n=3, t0=131.0)             # later camera frames do not overwrite it under m1t2
    assert e.colour == "red"


def test_uav_only_buoys_merge_forget_or_freeze_and_stay_out_of_the_plan():
    pm = pmap()
    pm.ingest_geo(msg([item(30.0, 0.0, "blue", "flash", "u1"), item(30.0, 3.0, "red", "flash", "u2"), item(30.0, -0.5, "green", "flash", "u3")]), O, 100.0)
    pm.ingest_geo(msg([item(30.0, 0.0, "blue", "flash", "u1"), item(30.0, 3.0, "red", "flash", "u2"), item(30.0, -0.5, "green", "flash", "u3")], fid="f2"), O, 101.0)
    assert len(pm.uav_entities()) == 3 and pm.entities("buoy") == [] and pm.gates() == [] and pm.markers()["entry"] is None
    assert pm.uav_entry_candidate() is not None and len(pm.provisional_gates()) == 1
    obs = gp.map_obstacles(pm, 5)
    assert len(obs) == 3 and all(abs(o[2] - (gp.OBJ_R["buoy"] + 0.7)) < 1e-6 for o in obs)       # sigma added to the keep radius
    camera(pm, 30.4, 0.3, t0=102.0)                              # the camera sees the blue one within 1 m: merged, the camera's position
    m = [e for e in pm.ents.values() if e.source == "usv"]
    assert len(m) == 1 and m[0].uav["id"] == "u1" and abs(m[0].x_n - 30.4) < 0.3 and m[0].colour == "blue" and pm.uav_stats["merged"] == 1
    pm.uav_frozen = True
    pm.age_out(500.0)
    assert len(pm.uav_entities()) == 2                           # m1t2: frozen for the attempt
    pm.uav_frozen = False
    pm.age_out(500.0)
    assert pm.uav_entities() == []                               # else forgotten after uav.forget_s (30 s)


def test_a_camera_track_absorbs_its_uav_only_twin_never_two_tracks():
    """Found live (m1t1 S6 on extra with --uav: the judge's report had 11 buoys, one OFF): a camera sighting was folded into a nearer
    UAV-only twin while the buoy already had its camera track - two camera tracks, the votes split. The camera's track comes first and
    absorbs the twin (its UAV votes); a different UAV id is never absorbed."""
    pm = pmap()
    camera(pm, 20.0, 5.0, colour="green", n=6)
    pm.ingest_geo(msg([item(21.6, 5.0, "green", uid="g1")]), O, 110.0)          # 1.6 m off: a UAV-only twin
    pm.ingest_geo(msg([item(21.6, 5.0, "green", uid="g1")], fid="f2"), O, 111.0)
    assert len(pm.uav_entities()) == 1
    camera(pm, 20.8, 5.0, colour="green", n=6, t0=112.0)                         # the camera now sights it between the two
    buoys = [e for e in pm.ents.values() if e.kind == "buoy"]
    assert len(buoys) == 1 and buoys[0].source == "usv" and buoys[0].uav["id"] == "g1" and buoys[0].uav["votes"] == {"green": 2}
    assert buoys[0].colour == "green" and pm.uav_stats["merged"] == 1


def test_a_bad_item_is_dropped_the_rest_kept():
    pm = pmap()
    m = msg([{"id": "x", "lat": "nope", "lon": 1}, item(10.0, 0.0, "red")])
    assert pm.ingest_geo(m, O, 100.0) == 1 and pm.uav_stats["dropped"] == 1


# ---- T1: WAIT_UAV_MAP, the leads, the precheck --------------------------------------------------------------------------------------
def _t1(uav_ocs=None):
    f = FastSil(T1ADV, "T1", {"tier": "advanced"}, seed=3)
    f.map.uav_ocs = uav_ocs if uav_ocs is not None else {"active": True}
    return f


def test_wait_uav_map_exits_on_complete_on_landing_on_the_word_and_not_before():
    f = _t1()
    f.run(max_s=3.0)
    assert f.task.m.state == "WAIT_UAV_MAP" and f.map.colour_source == "uav" and f.map.uav_frozen
    ctx = f.ctx()
    assert f.task.uav_map_ready(ctx) == (False, "")
    f.map.ingest_geo(msg([item(5.0, 0.0, "blue", "flash", "e")], state="complete"), f.origin, f.t)
    assert f.task.uav_map_ready(ctx)[0]
    g = _t1()
    g.run(max_s=3.0)
    g.map.ingest_geo(msg([item(5.0, 0.0, "red", "flash", "r")], state="complete"), g.origin, g.t)
    ok, why = g.task.uav_map_ready(g.ctx())
    assert not ok and "no entry" in why                                     # complete, but no flashing blue: wait (a CAUTION)
    g.map.uav_ocs = {"active": True, "scan_done": "now"}
    assert g.task.uav_map_ready(g.ctx())[0]                                 # `uav scan done` forces it
    h = _t1({"active": True, "flight": {"armed": False, "airborne_seen": True, "flight_phase": "FLIGHT_PHASE_GROUNDED"}})
    h.run(max_s=3.0)
    h.map.ingest_geo(msg([item(5.0, 0.0, "blue", "flash", "e")]), h.origin, h.t)
    assert not h.task.uav_map_ready(h.ctx())[0]                             # items still arriving
    h.t += 6.0
    ok, why = h.task.uav_map_ready(h.ctx())
    assert ok and "TOUCHDOWN" in why                                        # the telemetry fallback (v0.1.89: the touchdown sequence)
    k = _t1()
    k.task.p["uav_wait_s"] = 5.0
    r = k.run(max_s=20.0)
    assert r["state"] == "SAFE_HOLD" and "no usable UAV map after 5 s" in r["safe_holds"][0]


def test_m1t2_precheck_and_m1t1_unchanged():
    f = FastSil(T1ADV, "T1", {"tier": "advanced"}, seed=3)
    f.map.uav_ocs = {}
    assert "m1t2 needs the UAV" in f.task.precheck(f.ctx())
    g = FastSil(str(ROOT / "profiles" / "bench" / "world_extra.yaml"), "T1", {}, seed=3)
    assert g.task.precheck(g.ctx()) == "" and not g.task.uav_mode and g.map.colour_source == "camera"
    g.run(max_s=2.0)
    assert g.task.m.state == "SEARCH_ENTRY"                                 # m1t1 never waits for a UAV


def test_m1t2_done_on_the_colour_blind_world_in_the_fast_sil():
    r = FastSil(T1ADV, "T1", {"tier": "advanced"}, seed=4, uav={"noise_m": 0.7, "seed": 4}).run(max_s=900)
    assert r["state"] == "DONE", (r["state"], r["logs"][-6:])
    states = list(dict.fromkeys(r["states"]))
    assert states[:2] == ["WAIT_UAV_MAP", "SEARCH_ENTRY"] and "TRANSIT" in states
    assert any("WAIT_UAV_MAP done after" in l and "scan.complete" in l for l in r["logs"])
    rep = r["reports"][-1]
    known = [b for b in rep["buoys"] if b["state"] != "unknown"]           # the course's ten (earlier camera tracks stay listed, undecided)
    assert rep["complete"] and len(known) == 10 and {(b["colour"], b["state"]) for b in known} == {
        ("blue", "flashing"), ("green", "flashing"), ("red", "flashing"), ("black", "off"), ("blue", "solid")}
    assert sum(1 for b in known if (b["colour"], b["state"]) == ("blue", "flashing")) == 1 and sum(1 for b in known if b["state"] == "solid") == 1
    assert r["uav"]["stats"]["merged"] >= 9


def test_the_colour_blind_world_and_its_renderer():
    w = W.load(T1ADV)
    flagged = [o for o in w["objects"] if o.get("colour_from_above")]
    assert len(flagged) == 8 and {o["colour"] for o in flagged} == {"blue", "red", "green"}
    plain = W.load(str(ROOT / "profiles" / "bench" / "world_extra.yaml"))
    assert not any(o.get("colour_from_above") for o in plain["objects"])                       # world_extra unchanged
    from optiprimex.sim import render as R
    sc = R.scene_from_world({"objects": [dict(flagged[0], x_n=0.0, y_e=0.0)]}) if hasattr(R, "scene_from_world") else None
    src = (ROOT / "optiprimex" / "sim" / "render.py").read_text()
    assert 'if o.get("colour_from_above"):' in src and 'and not o.get("colour_from_above")' in src
    fp = (ROOT / "optiprimex" / "sim" / "fake_perception.py").read_text()
    assert fp.count('if o.get("colour_from_above")') == 2


# ---- the paths, the adapter, the fake UAV ---------------------------------------------------------------------------------------
def test_the_bridges_render_the_uav_path_only_when_asked():
    conf = (ROOT / "profiles" / "pool" / "mosquitto" / "boat_pool.conf.in").read_text()
    assert "#BRIDGE topic perception/# in 0 local/usv/uav/ uav/" in conf and "#UAVBRIDGE topic perception/buoys in 0 local/usv/uav/ uav/" in conf
    for f in ("bridge.conf", "boat.conf"):            # every copy of the bridge: the tests' and the bench's (found live: boat.conf had none)
        assert "topic perception/# in 0 local/usv/uav/ uav/" in (ROOT / "profiles" / "bench" / "mosquitto" / f).read_text(), f

    def render(env):
        sh = ("UAV_BUOY_PATH=\"${UAV_BUOY_PATH:-$( [ -n \"${UAV_BROKER_HOST:-}\" ] && echo direct || echo off )}\"\n"
              "if [ -n \"${UAV_BROKER_HOST:-}\" ] && [ \"$UAV_BUOY_PATH\" = direct ]; then UAVSED=\"-e s|@UAV_BROKER@|${UAV_BROKER_HOST}:${UAV_BROKER_PORT:-1883}| -e s|^#UAVBRIDGE.||\"; "
              "else UAVSED=\"-e /^#UAVBRIDGE/d\"; fi\nsed -e '/^#BRIDGE /d' $UAVSED \"$1\"")
        return subprocess.run(["bash", "-c", sh, "x", str(ROOT / "profiles" / "pool" / "mosquitto" / "boat_pool.conf.in")],
                              capture_output=True, text=True, env=env, timeout=10).stdout
    assert "connection uav" not in render({"PATH": "/usr/bin:/bin"})
    out = render({"PATH": "/usr/bin:/bin", "UAV_BROKER_HOST": "10.1.2.3"})
    assert "connection uav" in out and "address 10.1.2.3:1883" in out and "topic perception/buoys in 0 local/usv/uav/ uav/" in out
    assert "connection uav" not in render({"PATH": "/usr/bin:/bin", "UAV_BROKER_HOST": "10.1.2.3", "UAV_BUOY_PATH": "relay"})
    assert '$UAVSED profiles/pool/mosquitto/boat_pool.conf.in' in (ROOT / "scripts" / "run_usv.sh").read_text()


def test_the_adapter_relays_the_udp_form():
    from optiprimex.uav.adapter import UavAdapter
    from .test_v0164 import FBus

    class RB(FBus):
        def publish_raw(self, topic, payload, qos=0, retain=False):
            self.pub.append((topic, "raw", json.loads(payload)))
    cfg = Config()
    cfg.extra = {"ocs": {"uav": {}}}
    bus = RB()
    ad = UavAdapter(cfg, ("127.0.0.1", 9), 0, "127.0.0.1", bus=bus)
    ad.on_datagram(dict(msg([item(1.0, 1.0, "red")]), type="uav_buoys"))
    ad.on_datagram({"type": "uav_buoys", "items": "nope"})
    raw = [p for p in bus.pub if p[1] == "raw"]
    assert len(raw) == 1 and raw[0][0] == "uav/perception/buoys" and "type" not in raw[0][2] and raw[0][2]["items"][0]["colour"] == "red"
    ad.sender.sock.close()


def test_the_fake_uav_survey():
    w = W.load(T1ADV)
    sv = FU.Survey({"origin": w["origin"], "objects": w["objects"]}, noise_m=0.7, dropout=0.0, colour_ok=1.0, seed=1)
    assert len(sv.buoys) == 10 and 60 < sv.survey_s < 120
    first = sv.message(1.0)
    assert first["scan"]["state"] == "scanning" and first["source"] == "uav" and all(it["class"] == "buoy" for it in first["items"])
    msgs = [sv.message(float(t)) for t in range(2, int(sv.survey_s) + 6)]
    comp = [m for m in msgs if m and m["scan"]["state"] == "complete"]
    assert len(comp) == 3 and comp[0]["scan"]["n_items"] == 10 and msgs[-1] is None          # complete three times, then landed
    truth = {f"u{o['id']}": o for o in sv.buoys}
    errs = []
    for it in comp[0]["items"]:
        x, y = W.WorldFrame(w["origin"]).to_world(it["lat"], it["lon"])
        o = truth[it["id"]]
        errs.append(math.hypot(x - o["x_n"], y - o["y_e"]))
        assert it["colour"] == o["colour"] and it["lit"] == {"flash": "flash", "steady": "steady", "off": "off", False: "off"}[o["light"]]
    assert max(errs) < 1.0                                       # the mean of the sightings
    nc = FU.Survey({"origin": w["origin"], "objects": w["objects"]}, complete=False, seed=1)
    assert nc.message(nc.survey_s + 1) is None                   # --no-complete: never says complete


# ---- the OCS: the T1 tier, m1t2's start, `uav scan done` ----------------------------------------------------------------------------
def test_the_t1_tier_follows_the_mission_and_m1t2_needs_the_uav():
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "pool" / "ocs_pool.yaml").read_text())
    bus = FakeBus()
    app = OcsApp(cfg, bus=bus, judge=RJudge())
    app.lifecycle.on_link(True)
    app.on_course({"course_id": "SQ", "boundary": SQ, "uav_transit_area": [], "pinger_hz": 1})
    app.authority.on_intent("t", intent("MISSION", args={"task": "T1", "tier": "advanced"}))
    assert app.effective_tiers()["T1"] == "TIER_ADVANCED"
    app.authority.on_intent("t", intent("MISSION", args={"task": "T1"}))
    assert app.effective_tiers()["T1"] == "TIER_CORE"
    assert "m1t2 needs the UAV" in app.start_check("T1", {"tier": "advanced"}) and app.start_check("T1", {}) is None
    r = app.authority.on_intent("t", intent("START_TASK", task="T1", args={"tier": "advanced"}))
    assert r.result == "refused" and "m1t2 needs the UAV" in r.reason
    app.on_status("t", {"vehicle_id": UID, "body": {"state": "AUTO_HOLD", "robot_state": "STATE_AUTO", "armed": True, "flight_phase": "FLIGHT_PHASE_AIRBORNE",
                                                    "health": {"telemetry_age_s": 0.4}, "flown": {"armed_seen": True, "airborne_seen": True}}})
    r = app.authority.on_intent("t", intent("UAV", args={"op": "scan_done"}))
    assert r.result == "accepted" and app.state_body()["uav"]["scan_done"]
    fl = app.state_body()["uav"]["flight"]
    assert fl["armed"] is True and fl["airborne_seen"] is True and fl["flight_phase"] == "FLIGHT_PHASE_AIRBORNE"
    app.reset_run()
    assert app.state_body()["uav"]["scan_done"] is None
    assert ocs_cli.intent_spec(ocs_cli.parse(["uav", "scan", "done"]))[2] == {"op": "scan_done"}


def test_m1t2_preselects_under_the_course_policy_refuse():
    """Found live (rc-test, the course profile): `mission m1t2` was refused - the check compared the profile's T1 CORE, before the mission
    that sets ADVANCED (D22b) was preselected. The mission check now applies the rule to the mission it checks."""
    cfg = Config()
    cfg.extra = yaml.safe_load((ROOT / "profiles" / "course" / "ocs_course.yaml").read_text())
    app = OcsApp(cfg, bus=FakeBus(), judge=RJudge())
    app.lifecycle.on_link(True)
    app.on_course({"course_id": "SQ", "boundary": SQ, "uav_transit_area": [], "pinger_hz": 1})
    assert app.authority.declaration_policy == "refuse"
    r = app.authority.on_intent("t", intent("MISSION", args={"task": "T1", "tier": "advanced", "auto": True}))
    assert r.result == "accepted" and app.effective_tiers()["T1"] == "TIER_ADVANCED", r.reason
    r = app.authority.on_intent("t", intent("MISSION", args={"task": "T1"}))
    assert r.result == "accepted" and app.effective_tiers()["T1"] == "TIER_CORE"


def test_the_contract_doc_and_the_boat_side_wiring():
    doc = (ROOT / "docs" / "UAV_BUOY_CONTRACT.md").read_text()
    for needle in ("uav/perception/buoys", "scan", "complete", "sigma_m", "mosquitto_pub", "UAV_BRIDGE_PORT", "uav_buoys", "colour_from_above"):
        assert needle in doc, needle
    mm = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text()
    assert 'self.cfg.local("uav", "perception", "buoys")' in mm and "self.map.ingest_geo(body, self.origin" in mm and '"ocs/state", self._on_ocs_state' in mm
    ch = (ROOT / "optiprimex" / "usv" / "command_handler.py").read_text()
    assert '(None, "core", "advanced")' in ch


def test_quick_list_has_this_file():
    assert "tests/test_v0166.py" in (ROOT / "Makefile").read_text()
