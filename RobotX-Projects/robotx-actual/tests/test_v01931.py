"""v0.1.93.1 (Dmitrii 13 Nov morning: "add that any button in modifiers ... and also, I want squeeze tick box. After I tick, I want to
reconfirm the mission"): two more mission modifiers - squeeze (on | off) and lucky_pairs (one_known | any) - on the wire, the CLI, the tk
console and the page; the risky switches ask first and name the mission as it will be re-armed; the boat runs with them."""
from __future__ import annotations

import pytest

from optiprimex.companion.config import load as load_cfg
from optiprimex.ocs import cmd_authority as CA
from optiprimex.sim.fast_sil import FastSil
from optiprimex.tools import ocs_cli
from optiprimex.tools.tkconsole import model as M
from optiprimex.tools.tkconsole.model import ConsoleModel

from .conftest import ROOT
from .test_v0192 import _world
from .test_v0192_gui import PAGE, state


@pytest.mark.parametrize("k,v,want", [
    ("squeeze", "on", True), ("squeeze", "off", False), ("squeeze", True, True), ("squeeze", False, False),
    ("lucky_pairs", "any", "any"), ("lucky_pairs", "one_known", "one_known"), ("lucky_pairs", "one-known", "one_known"),
    ("lucky_pairs", "ANY", "any")])
def test_the_new_modifiers_normalise_to_the_boats_types(k, v, want):
    assert CA.mod_value(k, v) == ("", want)


@pytest.mark.parametrize("k,v", [("squeeze", "maybe"), ("lucky_pairs", "two"), ("lucky_pairs", "")])
def test_a_typo_is_refused_not_guessed(k, v):
    why, val = CA.mod_value(k, v) if v != "" else ("x", None)
    assert why and val is None


def test_the_keys_agree_everywhere_and_false_survives():
    assert CA.MOD_ARGS == ocs_cli.MOD_KEYS == M.MOD_KEYS and M.MOD_DEFAULTS == CA.MOD_DEFAULTS
    assert CA.MOD_DEFAULTS["squeeze"] is False and CA.MOD_DEFAULTS["lucky_pairs"] == "one_known"
    assert set(CA.MOD_ARGS) <= set(CA.MISSION_ARGS)
    assert CA.mod_args({"squeeze": "off", "lucky_pairs": "any"}) == ("", {"squeeze": False, "lucky_pairs": "any"})
    assert 'squeeze:false,lucky_pairs:"one_known"' in PAGE                                    # the page's MOD_DEF


def test_the_cli_takes_the_flags():
    p = ocs_cli.build_parser() if hasattr(ocs_cli, "build_parser") else None
    if p is None:
        pytest.skip("no build_parser")
    a = p.parse_args(["mission", "m1t1", "--lucky", "--lucky-pairs", "any", "--squeeze", "on"])
    assert a.lucky_pairs == "any" and a.squeeze == "on"


def _buttons(**mods):
    v = ConsoleModel().apply(state("usv", mission={"mods": mods} if mods else None))
    return {b.key: b for b in v.mods["buttons"]}


def test_squeeze_on_asks_and_names_the_rearmed_mission():
    b = _buttons()
    sq = b["mod-squeeze"]
    assert sq.label == "squeeze off" and "--squeeze on" in sq.word
    assert sq.confirm.startswith("SQUEEZE ON?") and "Re-arm m1t1 as:" in sq.confirm and "squeeze on" in sq.confirm
    off = _buttons(squeeze=True)["mod-squeeze"]
    assert off.label == "SQUEEZE ON" and "--squeeze off" in off.word and off.confirm == ""          # turning it off never asks


def test_any_turns_lucky_on_asks_and_lucky_off_drops_it():
    b = _buttons()
    a = b["mod-any"]
    assert a.label == "lucky pairs: one known" and "--lucky " in a.word + " " and "--lucky-pairs any" in a.word
    assert a.confirm.startswith("LUCKY ANY?") and "lucky ANY" in a.confirm
    on = ConsoleModel().apply(state("usv", mission={"lucky": True, "mods": {"lucky_pairs": "any"}}))
    bb = {x.key: x for x in on.mods["buttons"]}
    assert bb["mod-any"].label == "lucky pairs: ANY" and "--lucky-pairs one_known" in bb["mod-any"].word
    assert "--lucky " not in bb["mod-lucky"].word + " " and "--lucky-pairs one_known" in bb["mod-lucky"].word   # lucky off = one_known
    assert "lucky ANY" in on.mods["line"] and "squeeze off" in on.mods["line"]


def test_lucky_on_still_asks_with_the_mission_named():
    lk = _buttons()["mod-lucky"]
    assert lk.confirm.startswith("LUCKY ON?") and "Re-arm m1t1 as:" in lk.confirm and "lucky on" in lk.confirm


def test_the_page_has_both_buttons_and_asks():
    assert '["any",' in PAGE and '["squeeze",' in PAGE
    assert 'c.lucky_pairs==="any"?MOD_ASK.any' in PAGE and "c.squeeze===true?MOD_ASK.squeeze" in PAGE
    assert "Re-arm <b>${esc(mi.code" in PAGE and "`--squeeze ${oo(v.squeeze)}`" in PAGE


def test_the_boat_runs_with_squeeze_and_lucky_any(tmp_path):
    cfg = load_cfg(ROOT / "profiles" / "course" / "usv_course.yaml")
    s = FastSil(_world(tmp_path, entry=(3.0, -3.0), exit_=(26.0, 2.0)), "T1",
                {"planner": "simple", "squeeze": True, "lucky": True, "lucky_pairs": "any"}, seed=1, cfg=cfg, guard={"fake": {}, "sup": {}})
    r = s.run(max_s=1500)
    assert s.task.p["squeeze"] is True and s.task.p["lucky_pairs"] == "any"
    assert "squeeze on" in s.task.mods() and "lucky ANY" in s.task.mods()
    assert r["state"] == "DONE" and len(s.task.passed) == 4, (r["state"], r["logs"][-6:])
