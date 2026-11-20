"""v0.1.83.5 (session A's asks, docs/TK_CONSOLE.md §3): state.logging {usv, usv_sqlite, ocs, any}, lat / lon in /fast.json, GET /map.json;
usv_ui's presence carries log_bytes / log_t0, the SQLite logger's its file / t0."""
from __future__ import annotations

import json
import threading
import time
import types
import urllib.request

from .conftest import ROOT, free_port


def _web(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    return OW, a, OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)


def test_logging_state_from_the_presence_beats(tmp_path):
    _, _, web = _web(tmp_path)
    db = tmp_path / "hub_20261111_120000.sqlite"
    db.write_bytes(b"x" * 4096)
    now = time.time()
    with web.m.lock:
        web.m.presence[("boat", "usv_ui")] = ({"up": True, "log": "pool_20261111T120000Z", "log_bytes": 123456, "log_t0": now - 60}, now - 1)
        web.m.presence[("hub", "logger-hub")] = ({"up": True, "file": str(db), "t0": now - 30}, now - 1)
        web.m.presence[("boat", "logger-boat")] = ({"up": True, "file": "/x/boat.sqlite", "t0": now - 30}, now - 20)   # stale beat
    d = web.logging_state(now)
    assert d["usv"]["on"] and d["usv"]["file"] == "pool_20261111T120000Z" and d["usv"]["bytes"] == 123456 and d["usv"]["t0"] == now - 60
    assert d["ocs"]["on"] and d["ocs"]["bytes"] == 4096 and d["ocs"]["file"] == db.name           # this Mac stats its own file
    assert d["usv_sqlite"]["on"] is False and "old" in d["usv_sqlite"]["why"] and d["any"] is True
    with web.m.lock:                                                       # an older usv_ui: no log_bytes / t0 -> None, still on
        web.m.presence[("boat", "usv_ui")] = ({"up": True, "log": "pool_x"}, now)
        web.m.presence[("hub", "logger-hub")] = ({"up": False, "file": str(db)}, now)
    d = web.logging_state(now)
    assert d["usv"]["on"] and d["usv"]["bytes"] is None and d["usv"]["t0"] is None
    assert d["ocs"]["on"] is False and d["ocs"]["why"] == "stopped"
    with web.m.lock:
        web.m.presence.clear()
    d = web.logging_state(now)
    assert d["any"] is False and d["usv"]["why"] == "no usv_ui beat"
    assert "logging" in web.state()


def test_fast_lat_lon_and_the_map_json(tmp_path):
    OW, a, web = _web(tmp_path)
    with web.m.lock:
        web.m.status = {"state": "IDLE", "lat": 1.2345678, "lon": 103.1234567, "hdg_deg": 90.0}
        web.m.status_t = time.time()
        web.m.map, web.m.map_t, web.m.map_src = {"entities": [{"kind": "buoy", "x_n": 1, "y_e": 2}], "co": {"lat": 1.0, "lon": 103.0}}, time.time(), "mm"
    v = web.fast()["v"]
    assert v["lat"] == 1.2345678 and v["lon"] == 103.1234567
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        j = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{a.web_port}/map.json", timeout=5).read())
        assert j["source"] == "mm" and j["map"]["entities"][0]["kind"] == "buoy" and j["age"] < 5
        s = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{a.web_port}/state.json", timeout=5).read())
        assert set(s["logging"]) == {"usv", "usv_sqlite", "ocs", "any", "session"}          # v0.1.84.5: + session
    finally:
        srv.shutdown()


def test_usv_ui_and_the_logger_say_it(tmp_path):
    src = (ROOT / "usv_ui" / "usv_ui.py").read_text()
    assert '"log_t0": getattr(self, "log_t0", None) if self.base else None, "log_bytes": self.log_bytes()' in src
    assert "self.log_t0 = time.time()" in src and "self.log_t0 = None" in src
    ns: dict = {}
    code = src[src.index("    def log_bytes(self):"):src.index("    def log_stop(self):")]
    exec("import os\nclass X:\n" + code, ns)
    base = tmp_path / "pool_x"
    for ext, n in ((".tlog", 100), (".csv", 20), (".events", 3)):
        (tmp_path / f"pool_x{ext}").write_bytes(b"x" * n)
    assert ns["X"].log_bytes(types.SimpleNamespace(base=str(base))) == 123
    assert ns["X"].log_bytes(types.SimpleNamespace(base=None)) is None
    lg = (ROOT / "optiprimex" / "companion" / "logger.py").read_text()
    assert 'extra={"file": str(self.db_path), "t0": round(time.time(), 1)}' in lg


def test_the_uav_view_keeps_roll_and_pitch():
    """v0.1.84.3 (session A: the VEHICLES tab's UAV IMU row): the adapter's roll_deg / pitch_deg reach /state.json's UAV view."""
    from optiprimex.tools import ocs_web as OW
    t = OW.UavTap()
    t.on_message(json.dumps({"body": {"state": "IDLE", "hdg_deg": 90.0, "roll_deg": 1.5, "pitch_deg": -2.0}}).encode(), now=100.0)
    v = t.view(now=100.5)
    assert v["roll_deg"] == 1.5 and v["pitch_deg"] == -2.0 and v["hdg_deg"] == 90.0
