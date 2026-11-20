"""v0.1.78.13 (Dmitrii 2026-11-09, Team Village: "surface declare uav on on the main page"): the drone's controls on RUN's VEHICLES tile.
Since v0.1.78.16 they are two big buttons in a strip built once (tests/test_v0178015.py covers the strip in Chrome); this file keeps the
v0.1.78.13 promise: the RUN tile carries them, with the existing words."""
from __future__ import annotations

from .conftest import ROOT


def test_the_run_tile_carries_the_drone_controls():
    p = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'uavStrip("rv",$("run-veh"),$("rv-rows"))' in p                 # the RUN tile, above its rows
    assert 'send("declare uav "+(on?"off":"on"))' in p and 'send("uav ready "+(u.pilot_ready?"off":"on"))' in p
