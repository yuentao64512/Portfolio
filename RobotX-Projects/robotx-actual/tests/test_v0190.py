"""v0.1.90 (Dmitrii 2026-11-12: a "lucky" flag at startup - "our model is not perfect now, so the boat will proceed through closest buoys
pair inside course boundary to be considered as a gate, if the colour of buoys is uncertain ... we have to take a risk"): m1t1 / m1t2
pair uncertain-colour buoys (not red / green / blue / black) inside the course into gates after the real ones, cross them either way,
and go straight to the gates when no flashing blue entry turns up. Off in every preset; the launcher's tick box -> START_TASK lucky."""
from __future__ import annotations

import re

from optiprimex.companion.config import Config
from optiprimex.companion.state import MapEntity
from optiprimex.sim import world as Wm
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import launcher as L
from optiprimex.tools import launcher_tk as K
from optiprimex.usv.gatepath import planner as gp
from optiprimex.usv.perception_map import PerceptionMap

from .conftest import ROOT


def _pm(buoys):
    pm = PerceptionMap(Config())
    for i, (x, y, c) in enumerate(buoys, 1):
        pm.ents[i] = MapEntity(id=i, kind="buoy", colour=c, x_n=x, y_e=y, sigma_m=0.3, n_obs=9, t_first=0.0, t_last=pm.clock())
    pm._next_id = len(buoys) + 1
    return pm


def test_lucky_pairs_the_closest_uncertain_buoys_never_blue_black_or_a_real_pair():
    pm = _pm([(10, 0, "white"), (10, 3, "unknown"),          # 1-2: a lucky gate, 3 m
              (20, 0, "red"), (20, 3, "green"),                # 3-4: a real gate (not lucky)
              (30, 0, "red"), (30, 2.5, "white"),              # 5-6: red + uncertain: red stays red
              (40, 0, "blue"), (40, 3, "white"),               # 7: blue = a marker, never paired
              (50, 0, "black"), (50, 3, "white"),              # 9: black = an obstacle, never paired
              (60, 0, "white"), (60, 9, "white")])             # 11-12: too wide (> 6 m)
    real = pm.gates(None, 1)
    assert [(g.red.id, g.green.id) for g in real] == [(3, 4)]
    lucky = pm.lucky_gates(1, used_ids={3, 4}, mode="any")             # v0.1.92: "any" = this version's rule (default one_known)
    assert [(g.red.id, g.green.id) for g in lucky] == [(5, 6), (1, 2)] and all(g.lucky for g in lucky)   # the narrowest first
    assert pm.lucky_gates(9, used_ids=(), mode="any") and not pm.lucky_gates(10, mode="any")       # min_obs obeyed
    items = gp.gate_items(pm, 1, lucky=True, lucky_pairs="any")
    assert [it.lucky for it in items] == [False, True, True]               # the real gates first
    assert len(gp.gate_items(pm, 1)) == 1                                   # lucky off: as before


def test_a_lucky_gate_is_crossed_the_way_we_come_and_a_real_gate_nearby_wins():
    it = gp.GateItem((1, 2), (0.0, 1.5), (0.0, -1.5), (0.0, 0.0), (-1.0, 0.0), 3.0, lucky=True)   # "dir" points back at the boat
    ch = gp.choose_next([it], (-10.0, 0.0), None)
    assert ch.item is not None and not ch.wrong_side and ch.item.dir == (1.0, 0.0) and ch.item.red == (0.0, -1.5)
    real = gp.GateItem((3, 4), (5.0, -1.5), (5.0, 1.5), (5.0, 0.0), (1.0, 0.0), 3.0)
    assert gp.choose_next([it, real], (-10.0, 0.0), None).item.key == (3, 4)          # 15 m real < 10 m lucky + 10 m


def test_the_fast_sil_lucky_run_passes_the_white_gates(tmp_path):
    src = (ROOT / "profiles" / "bench" / "world_extra.yaml").read_text()
    w = tmp_path / "world_white.yaml"
    w.write_text(re.sub(r"(id: [1-7], +kind: buoy, colour: )(blue|green|red)( *),", r"\1white\3,", src))
    tp = Wm.load(str(w))["teleports"][0]
    st = {"x_n": tp["x_n"], "y_e": tp["y_e"], "hdg_deg": tp.get("hdg_deg", 0.0)}
    no = FastSil(str(w), "T1", {}, seed=4, start=st).run(max_s=500)
    assert no["state"] == "SAFE_HOLD"                                       # without the flag: no entry -> safe hold, as before
    f = FastSil(str(w), "T1", {"lucky": True, "lucky_pairs": "any", "scout_entry_s": 0, "gate_steps": False, "start_mode": "straight"},
                 seed=4, start=st)   # v0.1.92: pinned to v0.1.90's ladder and gate leg
    r = f.run(max_s=500)
    assert r["state"] == "DONE" and len(f.task.passed) == 3 and all(p["correct"] for p in f.task.passed), r["logs"][-5:]
    assert any("T1 LUCKY" in x for x in r["logs"])


def test_the_launcher_flag_reaches_the_mission():
    P = L.load_presets(str(ROOT / "profiles" / "presets.yaml"))
    assert all(p.get("lucky") is False for p in P.values())
    assert L.launch_block(dict(P["m1t1-gates"], preset="m1t1-gates", lucky=True))["extra"].get("lucky") is True
    assert "lucky" not in L.launch_block(dict(P["m1t1-gates"], preset="m1t1-gates"))["extra"]
    assert "lucky" not in (L.launch_block(dict(P["m1t1-usv"], preset="m1t1-usv", lucky=True)).get("extra") or {})   # scout: not a gate task
    assert L.validate(dict(P["custom"], lucky="maybe"))[0].startswith("lucky:")
    assert "lucky" in K.BOOL_KEYS and 'data-k="lucky" data-bool="1"' in (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    src = (ROOT / "optiprimex" / "ocs" / "ocs_app.py").read_text()
    assert 'for k in ("tier", "sim", "lucky")' in src and "self.authority.mission_task = str(task)" in src   # + the bare ▶ START fix
