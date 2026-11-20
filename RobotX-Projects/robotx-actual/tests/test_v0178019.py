"""v0.1.78.19: the UAV team's own buoy map (sgit/uav/cv/buoy_map, their buoy-detection/map_mqtt_publisher.py of 2026-11-10) received
on the boat broker, converted to our contract and fused with the source kept apart: feed "sgit" vs "contract", a linked buoy keeps the
boat's position, an unlinked one stays UAV-only (source "uav"); the OCS UAV card's "UAV map" row. QUICK: no broker."""
from __future__ import annotations

import json

from optiprimex.uav import sgit_map as SG

from .conftest import ROOT
from .test_v0166 import O, camera, item, ll, msg, pmap

# their docstring's example, verbatim (map_mqtt_publisher.py, the UAV Jetson, 2026-11-10 09:06) - the wire we read
THEIRS = '{"points": [{"id": 1, "color": "RED", "led_type": "FLASHING", "coords": [103.85210, 1.28450]}]}'


def sg(points):
    return json.dumps({"points": [{"id": i + 1, "color": c, "led_type": l, "coords": [ll(x, y)[1], ll(x, y)[0]]}
                                  for i, (x, y, c, l) in enumerate(points)]}).encode()


def test_their_example_converts_lon_lat_and_words():
    m = SG.to_contract(SG.decode(THEIRS.encode()), 7, t_unix=0)
    it = m["items"][0]
    assert (it["lat"], it["lon"]) == (1.28450, 103.85210)                      # coords are [lon, lat]
    assert (it["colour"], it["lit"], it["class"], it["sgit_id"]) == ("red", "flash", "buoy", 1)
    assert m["feed"] == "sgit" and m["frame_id"] == "sgit-7" and m["scan"] == {"state": "scanning", "coverage_pct": None, "n_items": 1}
    assert "id" not in it                                                     # their ids are rebuilt each second: association by position
    m = SG.to_contract({"points": [{"color": "GREEN", "led_type": "STATIC", "coords": [103.8, 1.2]}, {"color": "BLUE", "coords": None},
                                   {"color": "BLUE", "coords": [1.2, 103.8]}]}, 1)
    assert m["items"][0]["lit"] == "steady" and "lat" not in m["items"][1] and "lat" not in m["items"][2]   # bad / swapped coords: dropped
    assert SG.decode(b"not json") is None and SG.decode(b'{"x": 1}') is None and SG.decode(b'{"points": []}') == {"points": []}
    assert SG.status_text(b"online") == "online" and SG.status_text(b"offline") == "offline" and SG.status_text(b"??") is None


def test_the_boat_associates_their_map_and_keeps_the_sources_apart():
    pm = pmap()
    camera(pm, 20.0, 5.0)                                                     # a buoy the boat's own camera tracks
    e = pm.entities("buoy")[0]
    x0, y0 = e.x_n, e.y_e
    pts = [(20.5, 5.2, "RED", "FLASHING"), (40.0, -8.0, "BLUE", "STATIC")]
    for n in range(1, 4):                                                     # the whole map, once a second
        assert pm.ingest_geo(SG.to_contract(SG.decode(sg(pts)), n), O, 120.0 + n) == 2
    assert (e.x_n, e.y_e) == (x0, y0) and e.source != "uav" and e.colour == "red" and e.uav["feed"] == "sgit"   # the boat's position
    uo = pm.uav_entities()
    assert len(uo) == 1 and uo[0].source == "uav" and uo[0].colour == "blue" and uo[0].light == "steady"       # one, not three
    pm.ingest_geo(msg([item(60.0, 0.0, "green")]), O, 125.0)                  # our contract's feed beside it
    pm.uav_feed_status("sgit", "online", 125.0)
    d = pm.digest(now=126.0)["uav"]
    assert d["feeds"]["sgit"]["messages"] == 3 and d["feeds"]["sgit"]["with_position"] == 2 and d["feeds"]["sgit"]["status"] == "online"
    assert d["feeds"]["sgit"]["age_s"] == 3.0 and d["feeds"]["contract"]["messages"] == 1
    assert d["n_linked"] == 1 and d["n_uav_only"] == 1                        # the contract item: seen once < uav.min_obs 2
    json.dumps(d, allow_nan=False)


def test_mission_manager_and_ocs_wiring():
    src = (ROOT / "optiprimex" / "usv" / "mission_manager.py").read_text(encoding="utf-8")
    assert "self.bus.subscribe(sgit_map.MAP_TOPIC, self._on_sgit_map, qos=1, raw=True)" in src
    assert "self.bus.subscribe(sgit_map.STATUS_TOPIC, self._on_sgit_status, qos=1, raw=True)" in src
    assert src.index('if topic == "sgit/status"') < src.index("if vs.local is None:\n                continue")   # status without a pose
    assert (SG.MAP_TOPIC, SG.STATUS_TOPIC) == ("sgit/uav/cv/buoy_map", "sgit/uav/cv/status")
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text(encoding="utf-8")
    assert "<span>UAV map</span>" in html and '$("ov-uav").innerHTML=U+UM;' in html
