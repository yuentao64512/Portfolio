"""v0.1.81 (WP37): no fake autonomous mode - the HOLD / LOITER station keep (docs/PROMPT_SCOUT_BUILD_2026-11-11.md §3, brief §5.2):
Loiter() beside Hold(), StationKeep and AUTO_HOLD follow `station_keep hold|loiter` (default hold until the water drift check - Dmitrii
2026-11-11), the safe hold stays HOLD, the adapter requests LOITER with the HOLD reassert rule, the operator word end to end (ocs_cli,
the authority, the vehicle's handler, status), the buttons, the words (HANDED OVER · HOLD · enable_auto to go / MANUAL (RC) / AUTO · LOITER),
HOLD yellow, the FCU preset file (a diff, FS_ACTION untouched) and the Lua v5 table. QUICK: no broker."""
from __future__ import annotations

import yaml

from optiprimex.companion.config import Config
from optiprimex.companion.state import FcuState, Hold, Loiter, RunContext, VehicleState, station_keep_mode
from optiprimex.tools import ocs_cli, playbook
from optiprimex.usv.behaviours import StationKeep
from optiprimex.usv.behaviours.base import BehaviourContext

from .conftest import ROOT


def _bctx(cfg):
    return BehaviourContext(cfg=cfg, now=1.0, fcu=FcuState(), pose=None, origin=None, run=RunContext())


def test_the_default_is_hold_and_the_word_switches_station_keep():
    cfg = Config()
    assert station_keep_mode(cfg) == "hold"
    sk = StationKeep()
    sk.start({}, _bctx(cfg))
    out = sk.step(_bctx(cfg)).cmd
    assert type(out) is Hold
    cfg.extra["station_keep"] = {"mode": "loiter"}
    out = sk.step(_bctx(cfg)).cmd
    assert isinstance(out, Loiter) and isinstance(out, Hold)                 # a Hold to every other reader (muted stages, the guard)
    cfg.extra["station_keep"] = {"mode": "drift"}
    assert station_keep_mode(cfg) == "hold"                                  # anything else = hold
    for f in ("pool/usv_pool.yaml", "course/usv_course.yaml", "bench/usv_bench.yaml", "pool/usv_lab.yaml"):
        assert yaml.safe_load((ROOT / "profiles" / f).read_text())["station_keep"]["mode"] == "hold", f


def test_the_adapter_requests_loiter_with_the_hold_rule():
    from optiprimex.usv.adapters.rover import RoverAdapter
    asked = []

    class Link:
        mode_name = "HOLD"

        def set_mode(self, name, timeout=0.5):
            asked.append(name)
            Link.mode_name = name
            return True
    ad = RoverAdapter.__new__(RoverAdapter)
    ad.cfg, ad.direct_active, ad.ghost, ad.mode_wanted, ad._last_hold_assert, ad.trace = Config(), False, None, None, -1e9, None
    ad.current_mode = lambda: Link.mode_name
    ad._set_mode = lambda name, t: Link().set_mode(name, t)
    ad.apply(Loiter(), 10.0)
    assert asked == ["LOITER"] and ad.mode_wanted == "LOITER"
    ad.apply(Loiter(), 10.2)
    assert asked == ["LOITER"]                                                # already LOITER, not due yet
    ad.apply(Hold(), 10.4)
    assert asked == ["LOITER", "HOLD"] and ad.mode_wanted == "HOLD"


def test_auto_hold_loiters_the_safe_hold_holds_and_status_says_so():
    mm = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text(encoding="utf-8")
    i = mm.index('elif vs.state == VState.V3 and station_keep_mode(self.cfg) == "loiter":')
    assert "self.adapter.loiter(now)" in mm[i:i + 200] and mm.index("elif vs.state in (VState.V3, VState.V5):") > i
    vs = VehicleState()
    vs.station_keep = "loiter"
    assert vs.status_body()["station_keep"] == "loiter"


def test_the_word_end_to_end():
    p = ocs_cli.build_parser()
    assert ocs_cli.intent_spec(p.parse_args(["station_keep", "loiter"]))[:3] == ("STATION_KEEP", None, {"mode": "loiter"})
    from optiprimex.ocs import cmd_authority as CA
    assert CA.GUARDS["STATION_KEEP"][2] == "SET_STATION_KEEP"
    from optiprimex.usv.command_handler import CommandHandler
    h = CommandHandler.__new__(CommandHandler)
    h.cfg, h.link = Config(), None
    assert h._fcu("SET_STATION_KEEP", {"mode": "loiter"})[0] == "ACCEPTED" and station_keep_mode(h.cfg) == "loiter"   # no FCU link needed
    assert h._fcu("SET_STATION_KEEP", {"mode": "auto"})[0] == "REJECTED"
    assert "SET_STATION_KEEP" in (ROOT / "optiprimex" / "schemas" / "bodies.schema.json").read_text()
    assert "STATION_KEEP" in playbook.PLAYBOOK


def test_the_words_and_the_colours():
    W = playbook.vehicle_words
    assert W("MANUAL", "HOLD") == "HANDED OVER · HOLD · enable_auto to go"
    assert W("MANUAL", "MANUAL") == W("MANUAL", "HOLD", rc6_holds=True) == "MANUAL (RC)"
    assert W("AUTO_HOLD", "LOITER", station_keep="loiter") == "AUTO · LOITER" and W("AUTO_HOLD", "HOLD", station_keep="hold") == "AUTO · HOLD"
    assert W("AUTO_TASK", "GUIDED") is None
    from optiprimex.tools import boat_web as BW
    assert BW.fcu_mode_class("HOLD") == "hold" and BW.fcu_mode_class("LOITER") == "auto"
    bw = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text(encoding="utf-8")
    assert ".pb.hold{background:var(--warn)" in bw and 'id="sk-loiter"' in bw and "skSend('loiter')" in bw
    assert "station_keep hold" in BW.OCS_LINES and "station_keep loiter" in BW.OCS_LINES
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text(encoding="utf-8")
    assert "#fb-hold.act{background:var(--warn)" in page and "send('station_keep loiter')" in page and "HANDED OVER · HOLD · enable_auto to go" in page


def test_the_fcu_preset_is_a_diff_and_never_touches_the_failsafe():
    lines = [x for x in (ROOT / "docs" / "fcu" / "loiter_rc6.param").read_text().splitlines() if x and not x.startswith("#")]
    params = dict(x.split(",") for x in lines)
    assert params == {"LOIT_TYPE": "0", "LOIT_RADIUS": "2", "LOIT_SPEED_GAIN": "0.5", "MODE_CH": "6", "MODE1": "4", "MODE2": "4", "MODE3": "4",
                      "MODE4": "4", "MODE5": "4", "MODE6": "0", "RC6_OPTION": "0"}
    assert not any(k.startswith(("FS_", "ARMING", "FENCE")) for k in params)
    md = (ROOT / "docs" / "fcu" / "PRESET_LOITER_RC6.md").read_text(encoding="utf-8")
    assert "Not applied." in md and "apply_params.py" in md and "| HOLD | 4 | **YELLOW**" in md and "| LOITER | 5 | GREEN |" in md
