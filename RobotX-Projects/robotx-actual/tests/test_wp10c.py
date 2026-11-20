"""ROADMAP WP10c (v0.1.35), map interaction: add / move / delete objects (sim buoys, dock bays, box faces; waypoints = save_point;
markers = OCS-side notes), the hover readout (the same lat / lon <-> N / E model as teleport), tooltips on every button, the mini-map
on MAIN (map.html?mini=1)."""
from __future__ import annotations

import json
import re
import shutil
import subprocess
import threading
import urllib.error
import urllib.request
from pathlib import Path

import pytest

from .conftest import ROOT, free_port

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def _post(url, body, header):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST", headers={"Content-Type": "application/json", **header})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return dict(json.loads(e.read()), code=e.code)


def test_markers_store(tmp_path):
    from optiprimex.tools.ocs_web import Markers
    m = Markers(str(tmp_path / "ocs" / "markers.json"))
    assert m.apply({"op": "add", "name": "look here", "lat": 1.2811, "lon": 103.855, "note": "a crane"})["ok"]
    assert m.apply({"op": "move", "name": "look here", "lat": 1.2812, "lon": 103.8551})["ok"]
    assert m.load()["look here"] == dict(m.load()["look here"], lat=1.2812, lon=103.8551, note="a crane")     # the note kept on a move
    assert not m.apply({"op": "add", "name": "a/b", "lat": 1, "lon": 1})["ok"] and not m.apply({"op": "add", "name": "x", "lat": 91, "lon": 1})["ok"]
    assert not m.apply({"op": "move", "name": "nope", "lat": 1, "lon": 1})["ok"] and not m.apply({"op": "add", "name": "x"})["ok"]
    assert m.apply({"op": "del", "name": "look here"})["ok"] and m.load() == {} and not m.apply({"op": "del", "name": "look here"})["ok"]
    assert not (tmp_path / "ocs" / "markers.json.tmp").exists()


def test_ocs_page_markers_and_the_sim_proxy(tmp_path, fake_fcu):
    pytest.importorskip("numpy")
    from optiprimex.sim import simview as SV
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "markers.json")}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    ocs = f"http://127.0.0.1:{a.web_port}"
    sa = SV.parse_args(["--world", "pool", "--fcu-control", str(fake_fcu.ctrl_port), "--port", str(free_port()), "--hub-port", "0",
                        "--ocs-url", ocs + "/", "--width", "160", "--height", "90"])
    sim = SV.Sim(sa)
    ssrv = SV.Server(("127.0.0.1", sa.port), SV.make_handler(sim))
    threading.Thread(target=ssrv.serve_forever, daemon=True).start()
    simu = f"http://127.0.0.1:{sa.port}"
    web._meta["sim_url"] = simu + "/"
    H, S = {"X-OCS-Web": "1"}, {"X-Sim-Web": "1"}
    try:
        # sim objects through the OCS page (lat / lon), every kind, the light checked
        lat, lon = sim.frame.to_latlon(14.0, 0.5)
        r = _post(ocs + "/sim", {"path": "/objects", "body": {"op": "add", "kind": "buoy", "colour": "red", "light": "flash", "lat": lat, "lon": lon}}, H)
        assert r["ok"]
        o = next(x for x in sim.world["objects"] if x["id"] == r["id"])
        assert o["mock"] and o["light"] == "flash" and abs(o["x_n"] - 14.0) < 0.01 and abs(o["y_e"] - 0.5) < 0.01
        d = _post(ocs + "/sim", {"path": "/objects", "body": {"op": "add", "kind": "dock", "colour": "green", "bay": 2, "x_n": 20.0, "y_e": 4.0}}, H)
        assert d["ok"] and next(x for x in sim.world["objects"] if x["id"] == d["id"])["bay"] == 2
        assert _post(ocs + "/sim", {"path": "/objects", "body": {"op": "add", "kind": "box", "colour": "blue", "x_n": 30.0, "y_e": 0.0}}, H)["ok"]
        assert not _post(ocs + "/sim", {"path": "/objects", "body": {"op": "add", "kind": "buoy", "light": "blink", "x_n": 1, "y_e": 1}}, H)["ok"]
        assert not _post(ocs + "/sim", {"path": "/objects", "body": {"op": "add", "kind": "tree", "x_n": 1, "y_e": 1}}, H)["ok"]
        lat2, lon2 = sim.frame.to_latlon(16.0, 1.0)
        assert _post(ocs + "/sim", {"path": "/objects", "body": {"op": "move", "id": r["id"], "lat": lat2, "lon": lon2}}, H)["ok"]
        assert abs(o["x_n"] - 16.0) < 0.01 and abs(o["y_e"] - 1.0) < 0.01
        assert _post(ocs + "/sim", {"path": "/objects", "body": {"op": "del", "id": r["id"]}}, H)["ok"]
        assert not _post(ocs + "/sim", {"path": "/fcu", "body": {}}, H)["ok"]                         # the proxy forwards only its own paths
        # markers: from the OCS page and from the SIM page (relayed; the SIM page places by x_n / y_e)
        assert _post(ocs + "/markers", {"op": "add", "name": "check here", "lat": lat, "lon": lon}, H)["ok"]
        assert _post(simu + "/markers", {"op": "move", "name": "check here", "x_n": 5.0, "y_e": 5.0}, S)["ok"]
        mk = json.loads(urllib.request.urlopen(simu + "/markers.json", timeout=5).read())["markers"]["check here"]
        assert abs(sim.frame.to_world(mk["lat"], mk["lon"])[0] - 5.0) < 0.01
        assert json.loads(urllib.request.urlopen(ocs + "/markers.json", timeout=5).read())["markers"]["check here"]["lat"] == mk["lat"]
        assert _post(ocs + "/markers", {"op": "add", "name": "x"}, {})["code"] == 403                    # the page's header
    finally:
        srv.shutdown()
        ssrv.shutdown()


def test_hover_readout_uses_the_teleport_model():
    """map.html's localToLatLon is the control origin's from_local (the same equirectangular model teleport and the boat use)."""
    node = shutil.which("node")
    if node is None:
        pytest.skip("node not installed")
    html = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    fn = re.search(r"function localToLatLon\(m, x, y\) \{.*?\}", html).group(0)
    from optiprimex.companion.frames import ControlOrigin
    co = ControlOrigin(1.2810202, 103.8550097)
    js = fn + "\nconst m={co:{lat:1.2810202,lon:103.8550097}};console.log(JSON.stringify([localToLatLon(m,16.05,0.99),localToLatLon(m,-12.3,40.2)]));"
    out = json.loads(subprocess.run([node, "-e", js], capture_output=True, text=True, timeout=20).stdout)
    for (lat, lon), (x, y) in zip(out, [(16.05, 0.99), (-12.3, 40.2)]):
        elat, elon = co.from_local(x, y)
        assert abs(lat - elat) < 1e-9 and abs(lon - elon) < 1e-9


def test_pages_carry_the_new_controls():
    html = (ROOT / "optiprimex" / "tools" / "map.html").read_text()
    for needle in ('id="b_add"', 'id="addkind"', "waypoint (save_point)", "marker (a note)", 'id="hover"', 'id="b_help"', "Q.get('mini') === '1'",
                   "ocs-show-map", "/markers.json", "del_point", "save_point ${it.key} --lat"):
        assert needle in html, needle
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'id="minimap"' in page and "mini=1" in page and 'd.type==="ocs-show-map"' in page and "function fillTitles()" in page
    sim = (ROOT / "optiprimex" / "sim" / "sim.html").read_text()
    assert 'value="dock">dock bay' in sim and 'value="waypoint"' in sim and 'value="marker"' in sim and 'id="hover"' in sim and "SIM_TIP" in sim
    for f in ("optiprimex/tools/map.html", "optiprimex/sim/sim.html"):                     # static buttons carry a title already
        s = (ROOT / f).read_text()
        static = [b for b in re.findall(r"<button\b[^>]*>", s.split("<script>")[0]) if "title=" not in b]
        dynamic_ok = f.endswith("sim.html")                                                    # the SIM page titles its own by id at load
        assert not static or dynamic_ok, (f, static)


@pytest.mark.skipif(not Path(CHROME).exists(), reason="no Google Chrome")
def test_every_button_has_a_title_live(tmp_path):
    """The headless-Chrome check of WP10c's acceptance: the OCS page rendered (its META drawn, every generated button) and the map
    page - no <button> without a title."""
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json")}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        for path in ("/", "/map.html?host=127.0.0.1&port=1"):
            dom = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=10000", "--dump-dom", f"http://127.0.0.1:{a.web_port}{path}"],
                                 capture_output=True, text=True, timeout=90).stdout
            dom = re.sub(r"<script\b.*?</script>", "", dom, flags=re.S)
            buttons = re.findall(r"<button\b[^>]*>", dom)
            assert len(buttons) > (40 if path == "/" else 10), (path, len(buttons))
            assert [b for b in buttons if "title=" not in b] == [], path
    finally:
        srv.shutdown()
