"""v0.1.78.10 (Dmitrii 2026-11-09, Team Village): "HOLD should appear green - we start from this mode": the USV window's FCU MODE box
green for HOLD (its own class `hold`, the tokens of `auto`), and the OCS bar's HOLD button green while HOLD is the FCU's mode."""
from __future__ import annotations

from optiprimex.tools import boat_web as BW

from .conftest import ROOT


def test_hold_is_green_superseded_yellow_since_v0181():
    """v0.1.81 (WP37, Dmitrii 2026-11-11: HOLD = standby, not autonomous -> yellow; LOITER / GUIDED / AUTO / mode 20 green): the class stays
    `hold`, its colour token is --warn now."""
    assert BW.fcu_mode_class("HOLD") == "hold" and BW.fcu_mode_class("MANUAL") == "manual" and BW.fcu_mode_class("GUIDED") == "auto"
    w = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text()
    assert ".pb.hold{background:var(--warn);border-color:var(--warn);color:var(--panel)}" in w
    o = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert "#fb-hold.act{background:var(--warn);border-color:var(--warn)" in o and '$("fb-hold").classList.toggle("act",B.mode==="HOLD")' in o
