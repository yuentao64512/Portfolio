"""v0.1.92 (2026-11-13 night: Dmitrii's entry-first plan, the Cowork consult docs/CONSULT_ANSWER_2026-11-13.md, the overnight sim
logs/sim_explore_2026-11-13/FINDINGS_CLAUDE.md): the guard's corridor, the run-length blink rule, the LED -> map wiring, the T1 entry
search (scout + check), the lucky pairs, the map's memory, the gate run-in / post point, the camera height."""
from __future__ import annotations

import importlib.util
import math
import random
import sys

import numpy as np

from optiprimex.companion.config import Config, load as load_cfg
from optiprimex.companion.state import LocalPose, MapEntity, Observation
from optiprimex.sim.fast_sil import FastSil
from optiprimex.usv.gatepath import planner as gp
from optiprimex.usv.guard import GuardSupervisor
from optiprimex.usv.perception_map import PerceptionMap

from .conftest import ROOT


def _mod(name, rel):
    spec = importlib.util.spec_from_file_location(name, ROOT / rel)
    m = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = m
    spec.loader.exec_module(m)
    return m


LC = _mod("led_core_v0192", "cv/buoys_multi/led_core.py")
BOX = [100, 100, 200, 260]


# ---- the guard: the hull's corridor, not the +-35 deg cone -------------------------------------------------------------------------
def test_the_guard_stops_for_what_is_ahead_not_for_a_gate_buoy_beside():
    g = GuardSupervisor({})
    assert g.p["stop_rule"] == "corridor"
    beside = {"clear": False, "zone": "stop", "range_m": 1.4, "bearing_deg": 40.0, "frames": 3}      # lateral 0.9 m: a gate buoy
    ahead = {"clear": False, "zone": "stop", "range_m": 1.4, "bearing_deg": 10.0, "frames": 3}       # lateral 0.24 m
    assert not g.in_corridor(beside, "stop") and g.in_corridor(beside, "slow")
    assert g.in_corridor(ahead, "stop")
    wide = dict(beside, bearing_deg=60.0, bearing_min_deg=-5.0, bearing_max_deg=60.0)               # an extent across the bow: a wall
    assert g.in_corridor(wide, "stop")
    assert GuardSupervisor({"stop_rule": "cone"}).in_corridor(beside, "stop")                       # v0.1.91 kept behind the flag


# ---- the blink rule ------------------------------------------------------------------------------------------------------------------
def _draw(kind, fps=5.0, win=6.0, drop=0.3, seed=0, p=None):
    rnd = random.Random(seed)
    c, ph, t, out = LC.LedCache(p), rnd.random() * 2.0, 0.0, None
    while t < win:
        if rnd.random() > drop:
            lit = ((t + ph) % 2.0) < 1.0 if kind == "flash" else True
            obs = (rnd.random() > 0.10) if lit else (rnd.random() < 0.03)          # 10 % false off, 3 % false on
            out = c.update(t, [(BOX, obs, "blue" if obs else None)])
        t += 1.0 / fps
    return out[0][1]["pattern"] if out else "unknown"


def test_a_steady_light_is_almost_never_read_flashing_and_a_flasher_is_found():
    steady = [_draw("steady", seed=k) for k in range(300)]
    flash = [_draw("flash", seed=k) for k in range(300)]
    assert steady.count("flashing") <= 6                                             # <= 2 % (v0.1.91's rule: ~24 %)
    assert steady.count("steady") >= 220 and flash.count("flashing") >= 255         # ~87 % / ~93 % at 5 fps, 6 s, 30 % dropout
    assert flash.count("steady") == 0
    v091 = {"rule": "v091", "window_s": 4.0, "min_samples": 6, "steady_frac": 0.85}
    assert [_draw("steady", seed=k, p=v091) for k in range(300)].count("flashing") >= 30   # why the rule changed


def test_the_colour_needs_most_on_samples_to_agree():
    c = LC.LedCache()
    for k in range(30):
        out = c.update(k * 0.2, [(BOX, True, "blue" if k % 4 else "green")])     # 75 % blue
    assert out[0][1]["colour"] is None and out[0][1]["colour_share"] < 0.8
    c = LC.LedCache()
    for k in range(30):
        out = c.update(k * 0.2, [(BOX, True, "blue")])
    assert out[0][1]["colour"] == "blue" and out[0][1]["pattern"] == "steady"


def test_brightness_stays_absolute_by_default_and_the_relative_rule_is_a_knob():
    """12 Nov's real crops decided it (led/real.jpg): the relative rule missed the real blue lantern and the white core read shaded white
    markers "on" - the defaults keep v0.1.91's absolute 215; the relative rule / the white core stay available."""
    im = np.full((400, 400, 3), (60, 90, 70), np.uint8)
    im[100:260, 100:200] = (40, 40, 40)                                                # a dark hull
    im[50:80, 140:160] = (170, 100, 50)                                              # a blue lamp at V 170
    assert not LC.led_sample(im, BOX)[1]                                               # the default: absolute (v0.1.93.2: 180)
    rel = {"v_floor": 170, "v_cap": 230, "dark_hull_v": 150}
    col, on, _ = LC.led_sample(im, BOX, rel)
    assert on and col == "blue"                                                        # the knob: relative to the hull
    im[50:80, 140:160] = (250, 250, 250)                                             # blown: white core ...
    im[48:50, 140:160] = (255, 80, 30)                                               # ... blue fringe
    col, on, shares = LC.led_sample(im, BOX, rel)
    assert on and col == "blue" and shares.get("white", 0) > 0
    pale = np.full((400, 400, 3), (60, 90, 70), np.uint8)
    pale[100:260, 100:200] = (235, 235, 235)
    pale[50:80, 140:160] = (205, 190, 150)                                           # the pale float on a white hull: still off
    assert not LC.led_sample(pale, BOX)[1]


# ---- the LED into the map ---------------------------------------------------------------------------------------------------------
def _obs(t, led, colour="unknown"):
    return Observation.from_body({"t_cam": t, "items": [{"id": 1, "class": "buoy", "colour": colour, "confidence": 0.9, "range_m": 6.0,
                                                        "bearing_deg": 0.0, "quality": {"led": led}}]}, "buoys")


def test_the_observer_led_decides_the_map_light_and_the_uav_mode_ignores_it():
    pm = PerceptionMap(Config())
    pose = LocalPose(0.0, 0.0, 0.0, 1.2810202, 103.8550097, 0.0)
    for k in range(4):
        pm.ingest(_obs(k, {"pattern": "flashing", "colour": "blue"}, colour="black" if k % 2 else "blue"), pose, now=float(k))
    e = pm.entities("buoy", include_stale=True)[0]
    assert (e.colour, e.light) == ("blue", "flash") and e.led["pattern"] == "flashing"
    pm2 = PerceptionMap(Config())
    for k in range(4):
        pm2.ingest(_obs(k, {"pattern": "steady", "colour": "blue"}, colour="black" if k % 2 else "blue"), pose, now=float(k))
    assert pm2.entities("buoy", include_stale=True)[0].light == "steady"      # the votes alone would say flash (black half the time)
    pm3 = PerceptionMap(Config())
    pm3.colour_source = "uav"
    for k in range(4):
        pm3.ingest(_obs(k, {"pattern": "steady", "colour": "blue"}, colour="red"), pose, now=float(k))
    assert pm3.entities("buoy", include_stale=True)[0].colour == "red"


def test_cv_publish_passes_the_led_through():
    cvp = _mod("cvp_v0192", "cv/publish/cv_publish.py")

    class A:
        colour = "unknown"
    pub = cvp.CvPublisher.__new__(cvp.CvPublisher) if hasattr(cvp, "CvPublisher") else None
    fn = None
    for v in vars(cvp).values():
        if isinstance(v, type) and hasattr(v, "frame_to_items"):
            pub = v.__new__(v)
            fn = v.frame_to_items
    pub.a = A()
    items = fn(pub, {"range_m": 6.0, "bearing_deg": 1.0, "led": {"pattern": "flashing", "colour": "blue", "on": True, "now": "blue", "n": 20}},
               7, "blue", 0)
    assert items[0]["quality"]["led"] == {"pattern": "flashing", "colour": "blue", "on": True, "now": "blue"}
    assert "led" not in fn(pub, {"range_m": 6.0, "bearing_deg": 1.0}, 7, "blue", 0)[0]["quality"]


# ---- lucky, memory, planner -----------------------------------------------------------------------------------------------------
def _pm(buoys):
    pm = PerceptionMap(Config())
    for i, (x, y, c) in enumerate(buoys, 1):
        pm.ents[i] = MapEntity(id=i, kind="buoy", colour=c, x_n=x, y_e=y, sigma_m=0.3, n_obs=9, t_first=0.0, t_last=pm.clock())
    pm._next_id = len(buoys) + 1
    return pm


def test_lucky_never_pairs_two_unknowns_by_default():
    pm = _pm([(10, 0, "white"), (10, 1.8, "unknown"), (30, 0, "red"), (30, 2.5, "white")])
    assert [(g.red.id, g.green.id) for g in pm.lucky_gates(1)] == [(3, 4)]
    assert len(pm.lucky_gates(1, mode="any")) == 2


def test_the_course_profile_keeps_buoys_for_the_run_and_the_slow_orbit():
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    assert cfg.map.max_age_s >= 600 and cfg.map.drop_age_s >= 900
    assert cfg.limits.max_speed_mps == 0.6 and cfg.extra["zed_guard"]["slow_v"] < cfg.limits.max_speed_mps      # v0.1.94 confident_fast; SLOW still slows
    t1 = cfg.extra["t1"]
    assert t1["circle_v"] <= 0.2 and t1["lead_in_m"] == 3.0 and t1["circle_v"] == 0.12 and t1["transit_v"] == 0.30 and t1["scout_entry_s"] == 420 and t1["planner"] == "simple"


def test_the_gate_leg_runs_in_on_the_centre_line_and_pulls_the_post_point_off_an_object():
    it = gp.GateItem((1, 2), (10.0, 1.25), (10.0, -1.25), (10.0, 0.0), (1.0, 0.0), 2.5)
    cfg = gp.PlannerCfg(kind="simple", d=2.0, lead_in_m=5.0, obstacles=[])
    path, how = gp.leg(cfg, (0.0, 6.0, 0.0), it)                                   # beside the line: a run-in at N 5
    assert how.startswith("run-in") and min(math.hypot(p[0] - 5.0, p[1]) for p in path) < 0.3
    path, how = gp.leg(cfg, (7.0, 0.0, 0.0), it)                                   # lined up already: straight
    assert how == "straight"
    cfg2 = gp.PlannerCfg(kind="simple", d=2.0, obstacles=[(12.0, 0.3, 0.25)])      # an object just past the gate on the line
    path, _ = gp.leg(cfg2, (0.0, 0.0, 0.0), it)
    assert path[-1][0] < 12.0 - 0.8


def test_the_observer_assumes_the_measured_camera_height():
    src = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert 'p.add_argument("--h-cam", type=float, default=0.55' in src


# ---- the T1 entry search in the fast SIL (the real hull, the guard on) ------------------------------------------------------------
def _world(tmp_path, entry, exit_, extra=(), lights=True):
    import yaml
    objs = [{"id": 1, "kind": "buoy", "colour": "blue" if lights else "black", "light": "flash" if lights else "off", "body": "black",
             "role": "entry", "x_n": entry[0], "y_e": entry[1]}]
    i = 2
    for g, x in enumerate([9.0, 12.5, 16.0, 19.5], 1):
        objs += [{"id": i, "kind": "buoy", "colour": "green", "light": "flash", "body": "black", "gate": g, "x_n": x, "y_e": -1.25},
                 {"id": i + 1, "kind": "buoy", "colour": "red", "light": "flash", "body": "black", "gate": g, "x_n": x, "y_e": 1.25}]
        i += 2
    objs.append({"id": 20, "kind": "buoy", "colour": "blue" if lights else "black", "light": "steady" if lights else "off", "body": "black",
                 "role": "exit", "x_n": exit_[0], "y_e": exit_[1]})
    w = {"name": "t", "origin": {"lat": 1.2810202, "lon": 103.8550097}, "walls": False, "course": "bounds", "course_id": "SIM-T",
         "bounds": [[-14.0, -12.5], [34.0, -12.5], [34.0, 12.5], [-14.0, 12.5]], "start": {"x_n": 0.0, "y_e": 0.0, "hdg_deg": 0.0},
         "camera": {"name": "zed2i_left", "intrinsics": "zed2i_hd2k", "height_m": 0.55, "min_range_m": 0.5, "max_range_m": 20.0, "hz": 5,
                    "noise_range_abc": [0.02, 0.0, 0.0015], "noise_bearing_deg": 0.25, "p_drop_max": 0.7, "p_miscolour_max": 0.5, "conf": 0.85},
         "lights": {"period_s": 2.0, "on_s": 1.0}, "objects": objs}
    p = tmp_path / "w.yaml"
    p.write_text(yaml.safe_dump(w, sort_keys=False))
    return str(p)


def _run(world, seed=1):
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    s = FastSil(world, "T1", {"planner": "simple"}, seed=seed, cfg=cfg, guard={"fake": {}, "sup": {}})
    return s, s.run(max_s=1500)


def test_the_steady_decoy_near_the_start_is_never_circled_as_the_entry(tmp_path):
    w = _world(tmp_path, entry=(2.0, -9.0), exit_=(5.0, 3.5))
    for seed in (1, 2):
        s, r = _run(w, seed)
        ex = math.hypot(s.task.entry_xy[0] - 5.0, s.task.entry_xy[1] - 3.5) if s.task.entry_xy else 99
        assert s.task.entry_xy is not None and ex > 3.0, r["logs"][-6:]
        assert len(s.task.passed) >= 3                                          # it went on to the gates after the real entry
        if s.task.exit_xy is not None:                                          # the decoy is what it later takes as the exit
            assert math.hypot(s.task.exit_xy[0] - 5.0, s.task.exit_xy[1] - 3.5) < 1.5


def test_the_whole_m1t1_on_the_real_hull_with_the_guard(tmp_path):
    s, r = _run(_world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0)))
    track_clear = min(math.hypot(x - ox, y - oy) - 0.63 for (_, x, y, *_r) in r["track"]
                      for ox, oy in [(9.0, -1.25), (9.0, 1.25), (12.5, -1.25), (12.5, 1.25), (16.0, -1.25), (16.0, 1.25), (19.5, -1.25), (19.5, 1.25)])
    assert r["state"] == "DONE" and len(s.task.passed) == 4 and all(g["correct"] for g in s.task.passed), (r["state"], r["safe_holds"])
    assert track_clear > 0.3


def test_no_light_anywhere_never_keeps_a_false_entry(tmp_path):
    """With confirm_entry off (the final sweep's choice) a far vote can lock a dark buoy; the near sightings reject it (NOT_ENTRY). What
    must hold: no entry is kept, no gate run is declared done, every circle started is undone."""
    s, r = _run(_world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0), lights=False))
    assert s.task.entry_xy is None and r["state"] in ("SAFE_HOLD", "TIMEOUT", "DONE")
    if r["state"] == "DONE":                                                    # v0.1.93.2: the no-entry finale ends the mission
        assert "FINAL_ADVANCE" in r["states"] and "FINAL_SCAN" in r["states"] and not s.task.passed
    circles = r["states"].count("CIRCLE_ENTRY_CW")
    assert all(r["states"][i + 1] == "SEARCH_ENTRY" for i, x in enumerate(r["states"][:-1]) if x == "CIRCLE_ENTRY_CW"), r["states"]
    assert circles == len(s.task.rejected)


def test_near_a_lit_colour_never_joins_a_buoy_decided_another_and_the_range_gate_stays_off():
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    assert cfg.map.assoc_colour_m == 10.0 and cfg.map.assoc_gate_a == 0.0 and cfg.map.assoc_gate_m == 2.0
    pose = LocalPose(0.0, 0.0, 0.0, 1.2810202, 103.8550097, 0.0)

    def feed(pm, colour, brg, t):
        pm.ingest(Observation.from_body({"t_cam": t, "items": [{"id": 1, "class": "buoy", "colour": colour, "confidence": 0.9,
                                                                 "range_m": 6.0, "bearing_deg": brg}]}, "buoys"), pose, now=t)
    for c, n_expected in ((cfg, 2), (Config(), 1)):                               # 1.5 m apart at 6 m: red then blue
        pm = PerceptionMap(c)
        for k in range(4):
            feed(pm, "red", 0.0, float(k))
        for k in range(4):
            feed(pm, "blue", 14.0, float(4 + k))
        assert len([e for e in pm.ents.values() if e.kind == "buoy"]) == n_expected


def test_the_guard_backs_off_only_as_far_as_the_mapped_water_astern_is_free():
    from types import SimpleNamespace as NS
    pm = _pm([(-1.5, 0.0, "green"), (-1.5, 3.0, "red")])                         # a gate buoy 1.5 m astern, another well aside
    g = GuardSupervisor({})
    ctx = NS(b=NS(pose=LocalPose(0.0, 0.0, 0.0, 1.2810202, 103.8550097, 0.0), map=pm))
    assert abs(g.astern_free(ctx) - 0.35) < 0.01                                 # 1.5 - (0.6 stern + 0.25 buoy + 0.3)
    assert g.astern_free(NS(b=NS(pose=ctx.b.pose, map=_pm([(5.0, 0.0, "red")])))) == 2.0      # nothing astern: the full 2 m


def test_a_far_entry_from_the_scan_is_confirmed_close_before_it_is_circled_when_asked(tmp_path):
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")                  # confirm_entry is off by default (the final sweep)
    s = FastSil(_world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0)), "T1", {"planner": "simple", "confirm_entry": True}, seed=1, cfg=cfg,
                guard={"fake": {}, "sup": {}})
    r = s.run(max_s=900)
    assert any("confirm" in l and "entry candidate" in l for l in r["logs"]), [l for l in r["logs"] if "T1" in l][:6]
    i = r["states"].index("CIRCLE_ENTRY_CW")
    assert "CHECK_BLUE" in r["states"][:i]


def test_boxes_are_drawn_in_the_buoys_colour_and_the_led_is_shown():
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "function detCol(d)" in html and html.count("const c=detCol(d)") == 2 and 'BCOL={red:"#ff4040"' in html
    src = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert 'led = buoys[j].get("led")' in src and src.index('b["led"] = dict(pat, on=on, now=colour)') < src.index('led = buoys[j].get("led")')


def frame(led=None):
    """A grey water / white buoy frame with a bright lamp painted in the LED band (as test_v0191)."""
    im = np.full((400, 400, 3), (60, 90, 70), np.uint8)
    im[100:260, 100:200] = (235, 235, 235)
    if led:
        im[50:80, 140:160] = {"red": (20, 20, 255), "green": (20, 255, 20), "blue": (255, 40, 20)}[led]
    return im


def test_the_led_burst_samples_the_lamp_between_full_frames():
    """Dmitrii: "can we go faster than 5 fps?" - 12 Nov's observer ran 0.8-2.2 full fps; the burst reads the lamps at the last boxes on
    image-only grabs (the camera's 15 fps) so the blink memory sees enough samples."""
    lit = frame("blue")
    dark = frame(None)
    t = {"now": 0.0}
    seq = []

    def grab():
        t["now"] += 1.0 / 15.0
        return True

    def img():
        on = (t["now"] % 2.0) < 1.0                                                  # 1 s on / 1 s off
        seq.append(on)
        return lit if on else dark
    c = LC.LedCache()
    for full in range(12):                                                           # 12 full frames at ~2 fps (6 s), 6 burst frames each
        c.update(t["now"], [(BOX, *reversed(LC.led_sample(img(), BOX)[:2]))])
        assert LC.burst(6, grab, img, lambda: t["now"], [BOX], c) == 6
        t["now"] += 0.1
    pat = c.pattern(c.find(BOX))
    assert pat["n"] >= 12 and pat["pattern"] == "flashing" and pat["colour"] == "blue", pat
    assert LC.burst(6, lambda: False, img, lambda: 0.0, [BOX], c) == 0 and LC.burst(0, grab, img, lambda: 0.0, [BOX], c) == 0
    src = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert '"--led-burst", type=int, default=0' in src and "runtime_fast.enable_depth = False" in src
    assert "LC.burst(a.led_burst" in src and src.index("LC.burst(a.led_burst") < src.index("payload = M.frame_payload(")


def test_the_observer_reports_where_its_time_goes():
    src = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    for k in ('"grab_depth"', '"retrieve"', '"detector"', '"measure_led"', '"guard_write"', '"loop"', 'payload["timing_ms"]'):
        assert k in src, k
    assert src.index("_t0 = time.monotonic()") < src.index("err = camera.grab(runtime)") < src.index('payload["timing_ms"]') < src.index('tmp.write_text(json.dumps(payload')
    sh = (ROOT / "scripts" / "agx_observer.sh").read_text()
    assert "--led-burst) BURST=" in sh and "--depth-mode) DMODE=" in sh and "--resolution) RES=" in sh


def test_the_speed_cap_holds_every_task_command():
    """Dmitrii: "cap the speed ... so if any skipping or freezing is happening, we still good" - limits.max_speed_mps was a label only."""
    from types import SimpleNamespace as NS
    from optiprimex.companion.state import GuidedTarget, VelocityYawTarget
    from optiprimex.usv.tasks.base import cap_speed
    cfg = Config()
    cfg.limits.max_speed_mps = 0.3
    out = NS(cmd=GuidedTarget(1.0, 2.0, 0.6))
    cap_speed(out, cfg)
    assert out.cmd.speed_mps == 0.3 and (out.cmd.lat, out.cmd.lon) == (1.0, 2.0)
    out = NS(cmd=GuidedTarget(1.0, 2.0, None))                                  # no speed = the FCU's default: capped too
    cap_speed(out, cfg)
    assert out.cmd.speed_mps == 0.3
    out = NS(cmd=VelocityYawTarget(0.55, 0.05))
    cap_speed(out, cfg)
    assert out.cmd.vx_mps == 0.3 and out.cmd.yaw_rate_rad_s == 0.05
    out = NS(cmd=VelocityYawTarget(0.2, 0.0))
    cap_speed(out, cfg)
    assert out.cmd.vx_mps == 0.2                                                 # under the cap: untouched


def test_the_course_run_never_exceeds_the_cap_and_slow_perception_is_simulated(tmp_path):
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    cfg.limits.max_speed_mps = 0.3
    w = _world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0))
    s = FastSil(w, "T1", {"planner": "simple"}, seed=1, cfg=cfg, guard={"fake": {}, "sup": {}}, perception_hz=1.0)
    r = s.run(max_s=900)
    tr = r["track"]
    v = [math.hypot(b[1] - a[1], b[2] - a[2]) / max(1e-6, b[0] - a[0]) for a, b in zip(tr, tr[1:]) if b[0] > a[0]
         and a[4] == b[4] and not a[4].startswith("GUARD")]
    assert max(v) <= 0.36, max(v)                                              # the plant's lag + sampling: a few cm/s over
    assert len(s.guard_msgs) < 0.15 * len(tr) * 5                              # ~1 guard frame a second, not 10


# ---- v0.1.92 FINAL STRATEGY (Dmitrii + Cowork addendum 6 §6.4) ------------------------------------------------------------------
def test_the_final_strategy_runs_start_lookround_gates_on_their_lines(tmp_path):
    s, r = _run(_world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0)))
    st = [x for i, x in enumerate(r["states"]) if i == 0 or x != r["states"][i - 1]]
    assert st[:3] == ["START_ADVANCE", "START_SCAN", "SEARCH_ENTRY"], st
    assert r["state"] == "DONE" and len(s.task.passed) == 4
    assert all(q["correct"] and abs(q["off_m"]) <= 0.15 for q in s.task.passed), s.task.passed      # on the gate line, not near it
    assert st.count("ALIGN_GATE") >= 1 and st.count("THROUGH") == 4
    m = s.task.micro(None)
    assert m["stage"] == "DONE" and m["gates"].startswith("G1 ✓ G2 ✓") and "start=advance+360" in m["mods"]


def test_a_boat_off_the_line_at_the_pre_point_still_passes_on_the_centre(tmp_path):
    """Cowork 6.4's contract: 0.4 m off the normal at the pre-point -> through within 0.1 m of the mid at 0.12 m/s."""
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    w = _world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0))
    s = FastSil(w, "T1", {"planner": "simple"}, seed=2, cfg=cfg, guard={"fake": {}, "sup": {}},
                start={"x_n": 6.0, "y_e": 0.4, "hdg_deg": 0.0})
    s.task.p["start_mode"] = "straight"
    r = s.run(max_s=1500)
    assert s.task.passed and abs(s.task.passed[0]["off_m"]) <= 0.1, s.task.passed


def test_the_pre_point_never_sits_behind_the_previous_gate():
    from optiprimex.usv.tasks.t1_safe_passage import T1SafePassage, DEFAULTS
    t = T1SafePassage.__new__(T1SafePassage)
    t.p = dict(DEFAULTS, lead_max_m=3.0, lead_min_m=1.5)
    t.passed = []
    it = gp.GateItem((1, 2), (10.0, 1.25), (10.0, -1.25), (10.0, 0.0), (1.0, 0.0), 2.5)
    assert t._lead(it) == 3.0                                                      # the first gate: 3 m
    t.passed = [{"mid": (6.5, 0.0)}]                                               # gates 3.5 m apart
    assert t._lead(it) == 2.0 and t._lead(it) <= 3.5 - 1.5
    t.passed = [{"mid": (8.5, 0.0)}]                                               # 1.5 m apart: never behind it - the 1.5 m floor
    assert t._lead(it) == 1.5


def test_turn_first_holds_and_yaws_when_the_carrot_is_far_off_the_bow():
    from optiprimex.usv import guidance as GD
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    st = GD.Steer(cfg)
    st.turn_first = 25.0
    cmd = st.law(0.0, (0.0, 0.0), 0.0, (0.0, 5.0), 0.15)                            # 90 deg off: hold and turn
    assert cmd.vx_mps == 0.0 and cmd.yaw_rate_rad_s > 0
    cmd = st.law(1.0, (0.0, 0.0), 85.0, (0.0, 5.0), 0.15)                           # 5 deg off: hysteresis released (< 25 - 15)
    assert cmd.vx_mps > 0
    st2 = GD.Steer(cfg)                                                             # other tasks: no turn-first (the profile's guidance)
    assert st2.law(0.0, (0.0, 0.0), 0.0, (0.0, 5.0), 0.15).vx_mps >= 0.0 and getattr(st2, "turn_first", None) is None


def test_a_mapped_buoy_stops_the_guard_only_on_a_collision_course():
    g = GuardSupervisor({})
    pm = _pm([(1.4, 0.8, "green")])                                                 # a gate buoy 1.4 m ahead, 0.8 m aside
    g.on_message({"clear": True}, 0.0, pm)
    pose = LocalPose(0.0, 0.0, 0.0, 1.2810202, 103.8550097, 0.0)
    beside = {"clear": False, "zone": "stop", "range_m": 1.6, "bearing_deg": 30.0, "frames": 3}
    assert g.is_mapped(pose, beside)
    assert not g.in_corridor(beside, "stop", mapped=True) and g.in_corridor(beside, "slow", mapped=True)
    ahead = {"clear": False, "zone": "stop", "range_m": 0.9, "bearing_deg": 10.0, "frames": 3}
    assert g.in_corridor(ahead, "stop", mapped=True)                                # a collision course: STOP
    unknown = {"clear": False, "zone": "stop", "range_m": 1.4, "bearing_deg": -15.0, "frames": 3}   # the other side: 1.2 m from the buoy
    assert not g.is_mapped(pose, unknown) and g.in_corridor(unknown, "stop")        # nothing mapped there: the 1.5 / 0.7 corridor
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    assert cfg.extra["zed_guard"]["repeat_r_m"] == 1.0                               # the hold counts stops on the same object


def test_the_gate_memory_outlives_the_map():
    from types import SimpleNamespace as NS
    from optiprimex.usv.tasks.t1_safe_passage import T1SafePassage, DEFAULTS
    pm = _pm([(9.0, -1.25, "green"), (9.0, 1.25, "red"), (12.5, -1.25, "green"), (12.5, 1.25, "red")])
    t = T1SafePassage.__new__(T1SafePassage)
    t.p = dict(DEFAULTS, gate_steps=True, min_obs=1)
    t.passed = []
    ctx = NS(b=NS(map=pm))
    assert len(t.items(ctx)) == 2
    del pm.ents[3], pm.ents[4]                                                      # the map forgets gate 2
    assert len(t.items(ctx)) == 2                                                   # the memory does not


def test_targets_stay_inside_the_course_fence():
    from optiprimex.usv.tasks.t1_safe_passage import _seg_near
    poly = [(0.0, -10.0), (30.0, -10.0), (30.0, 10.0), (0.0, 10.0)]
    assert _seg_near((15.0, 10.8), poly, 1.0) and not _seg_near((15.0, 11.5), poly, 1.0)
