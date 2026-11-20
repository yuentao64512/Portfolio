"""v0.1.78.6: the launcher said the USV window was down while it answered on the NX (Dmitrii 2026-11-09 ≈12:55, Team Village): PROGRESS
probed 127.0.0.1:8098, but a real launch binds the window to the team LAN - the NX's BOAT_HOST (usv_window nx) or this Mac's OCS_HOST (mac)."""
from __future__ import annotations

from optiprimex.tools import launcher as L

NET = {"BOAT_HOST": "10.1.0.218", "OCS_HOST": "10.1.0.13"}


def _uw(cfg, net=NET):
    return {x["name"]: x for x in L.page_links(cfg, net_env=net)}["USV window"]


def test_where_the_window_answers():
    P = L.load_presets()
    ev = dict(P["event"], net="lan")
    assert ev["usv_window"] == "nx" and _uw(ev)["url"] == "http://10.1.0.218:8098/" and _uw(ev)["probe"] == "http://10.1.0.218:8098/state.json"
    assert _uw(dict(ev, usv_window="nx-classic"))["url"] == "http://10.1.0.218:8098/"
    assert _uw(dict(ev, usv_window="mac"))["url"] == "http://10.1.0.13:8098/"
    assert _uw(dict(ev, usv_window="off"))["url"] == "http://127.0.0.1:8098/"
    assert _uw(P["sim"])["url"] == "http://127.0.0.1:8098/"                                   # the simulator: this Mac
    assert _uw(dict(ev, net="manual", boat_host="10.9.9.9"), net={})["url"] == "http://10.9.9.9:8098/"
    assert _uw({})["url"] == "http://127.0.0.1:8098/"                                         # no launch record
