"""v0.1.78.12 (Dmitrii 2026-11-09, Team Village: "a quick ssh link to the UAV Jetson - add it to our network config"): UAV_SSH in the
network files (uav-pixhawk@192.168.1.153 at the venue), the launcher's UAV row carries ssh ↗ / key ok · password needed / copy my key ↗ like
the NX and AGX rows; /ssh takes which=uav (the target from the files only)."""
from __future__ import annotations

import json
import threading
import urllib.request

from optiprimex.tools import launcher as L

from .conftest import free_port


def test_the_uav_row_and_target(tmp_path):
    (tmp_path / "network.env").write_text("NET_PROFILE=lan\n")
    (tmp_path / "network.lan.env").write_text("UAV_HOST=10.1.0.153\nUAV_SSH=uav-pixhawk@10.1.0.153\n")
    t = L.network_table(str(tmp_path), ping=lambda a: True, auth=lambda tgt: "password")
    r = {x["key"]: x for x in t["rows"]}["UAV_HOST"]
    assert r["ssh"] == {"which": "uav", "target": "uav-pixhawk@10.1.0.153", "auth": "password"} and "observer" not in r and "pin" not in r
    assert L.ssh_target("uav", str(tmp_path)) == "uav-pixhawk@10.1.0.153"


def test_the_route_takes_uav(monkeypatch):
    monkeypatch.setattr(L, "ssh_target", lambda which, kit=None: {"uav": "uav-pixhawk@10.1.0.153"}.get(which, ""))
    port = free_port()
    srv = L.Server(("127.0.0.1", port), L.make_handler(L.Runner(dry=True)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        rq = urllib.request.Request(f"http://127.0.0.1:{port}/ssh", data=json.dumps({"which": "uav"}).encode(), method="POST",
                                    headers={"Content-Type": "application/json", "X-Launch": "1"})
        j = json.loads(urllib.request.urlopen(rq, timeout=5).read())
    finally:
        srv.shutdown()
    assert j == {"ok": True, "dry": True, "cmd": "ssh -o ConnectTimeout=6 uav-pixhawk@10.1.0.153"}
