"""v0.1.78.3: the launcher's NETWORK reference table (Dmitrii 2026-11-09, Team Village: "update our launcher with correct addresses and a table
with them for reference") - every address the launchers use, per profile, from network*.env read at each request, a ping per address (this
Mac's own addresses = on an interface); /network.json and the page's card; the values never written into the code."""
from __future__ import annotations

from optiprimex.tools import launcher as L

from .conftest import ROOT


def test_the_table_from_the_files(tmp_path):
    (tmp_path / "network.env").write_text("NET_PROFILE=lan\nVENUE_RC_HOST=10.7.0.4\nVENUE_RC_PORT=1910\nVENUE_RC_WEB=http://10.7.0.4:8010/\n")
    (tmp_path / "network.lan.env").write_text("OCS_HOST=10.1.0.13   # this Mac\nBOAT_HOST=10.1.0.218\nRADIO_HOSTS=10.1.0.5 10.1.0.20\nSWITCH_HOST=10.1.0.1\n")
    (tmp_path / "network.ts.env").write_text("BOAT_HOST=100.9.9.9\n")
    seen = []
    t = L.network_table(str(tmp_path), ping=lambda a: seen.append(a) or a != "10.1.0.20")
    assert t["active"] == "lan" and [r["key"] for r in t["rows"]] == [k for _, k in L.NET_TABLE]
    R = {r["key"]: r for r in t["rows"]}
    assert R["OCS_HOST"]["lan"] == "10.1.0.13" and R["BOAT_HOST"]["ts"] == "100.9.9.9"
    assert [h["addr"] for h in R["BOAT_HOST"]["hosts"]] == ["10.1.0.218", "100.9.9.9"]          # the active profile first
    assert [(h["addr"], h["up"]) for h in R["RADIO_HOSTS"]["hosts"]] == [("10.1.0.5", True), ("10.1.0.20", False)]
    assert R["VENUE_RC_HOST"]["port"] == "1910" and R["VENUE_RC_WEB"]["hosts"] == [{"addr": "10.7.0.4", "up": True}]
    assert R["UAV_HOST"]["hosts"] == [] and len(seen) == len(set(seen))                           # unset = no row host; one ping per address


def test_the_route_and_the_card():
    src = (ROOT / "optiprimex" / "tools" / "launcher.py").read_text()
    assert 'elif path == "/network.json":' in src and "network_table(kit)" in src
    page = (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    for needle in ('id="netcard"', 'id="nettab"', 'fetch("/network.json")', "function netTab()"):
        assert needle in page, needle
    assert "192.168." not in src.split("NET_TABLE = ")[1].split("def net_placeholders")[0]          # no address in the code
