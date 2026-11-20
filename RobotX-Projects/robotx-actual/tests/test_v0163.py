"""v0.1.63 - the USV window on the team LAN for a delegate (Dmitrii 2026-11-07, polled: on the NX or the Mac, picked in the launcher, the
event's default nx; ONE commander, take over at once; a PIN for commands from another computer). window_control, boat_web's /control and
the command gate, the OCS lines through the hub (the NX), run_boat_window.sh, the launcher's field, the OCS SYSTEM row. No broker."""
from __future__ import annotations

import json
import threading
import urllib.request

from optiprimex.tools import boat_web as BW
from optiprimex.tools import launcher as L
from optiprimex.tools.window_control import Control, new_pin

from .conftest import ROOT, free_port

NET = {"ORIN_SSH": "nx-alias", "AGX_SSH": "agx-alias"}


class Clock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


A, B = "a" * 16, "b" * 16
LAN_A, LAN_B = "192.168.1.20", "192.168.1.30"


# ---- the lock ----------------------------------------------------------------------------------------------------------------------
def test_one_commander_take_over_and_the_pin():
    clk = Clock()
    c = Control("4321", stale_s=15.0, clock=clk)
    ok, why = c.may_command(A, "dmitrii", "127.0.0.1")
    assert ok and c.view(A)["you"]["commander"]                                       # alone on this host: the first command takes it
    ok, why = c.may_command(B, "jaison", LAN_B)
    assert not ok and "you are a viewer - dmitrii (127.0.0.1) commands" in why
    assert c.view(B)["you"]["needs_pin"] is True and c.view(A)["you"]["needs_pin"] is False
    ok, why = c.take(B, "jaison", LAN_B)
    assert not ok and why.startswith("PIN needed")
    assert c.take(B, "jaison", LAN_B, pin="0000") == (False, "wrong PIN")
    ok, why = c.take(B, "jaison", LAN_B, pin="4321")
    assert ok and why == "control taken by jaison (192.168.1.30) from dmitrii (127.0.0.1)"     # at once (decided)
    assert c.may_command(B, "jaison", LAN_B)[0] and not c.may_command(A, "dmitrii", "127.0.0.1")[0]
    assert c.view(B)["you"]["needs_pin"] is False                                      # the PIN is proved once per browser
    assert c.take(A, "dmitrii", "127.0.0.1")[0]                                        # this host takes it back without a PIN
    v = c.view()
    assert v["commander"] == {"name": "dmitrii", "addr": "127.0.0.1"} and len(v["sessions"]) == 2 and v["pin"]
    assert any("control taken by jaison" in e["text"] for e in v["events"])


def test_a_gone_commander_frees_control_and_scripts_on_this_host_pass():
    clk = Clock()
    c = Control(None, stale_s=15.0, clock=clk)
    assert c.may_command(A, "x", LAN_A)[0]                                             # no PIN set: a LAN browser may command
    clk.t += 16                                                                         # A stopped polling
    c.seen(B, "y", LAN_B)
    assert c.view()["commander"] is None and len(c.view()["sessions"]) == 1
    assert c.may_command(B, "y", LAN_B)[0]
    assert c.may_command(None, None, "127.0.0.1") == (True, "")                        # curl / the tests on this host, as before
    assert c.may_command(None, None, LAN_A)[0] is False                                # but never from another computer
    assert c.may_command("bad id!", None, "127.0.0.1")[0] is False
    assert c.release(B) == (True, "released") and c.view()["commander"] is None
    assert len(new_pin()) == 4 and new_pin().isdigit()
    assert Control.clean_name("<script>alert(1)</script>Bob") == "scriptalert1scriptBob"[:24]


# ---- boat_web: /control and the gate --------------------------------------------------------------------------------------------
def _server(control):
    port = free_port()
    srv = BW.Server(("127.0.0.1", port), BW.make_handler(BW.Model("usv"), None, bar=None, control=control, where={"where": "nx", "url": "u"}))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, f"http://127.0.0.1:{port}"


def _req(url, path, body=None, sid=None, name=None):
    h = {"Content-Type": "application/json", "X-Boat-Web": "1"}
    if sid:
        h.update({"X-Window-Session": sid, "X-Window-Name": name or "n"})
    r = urllib.request.Request(url + path, data=None if body is None else json.dumps(body).encode(), method="GET" if body is None else "POST", headers=h)
    with urllib.request.urlopen(r, timeout=5) as fh:
        return json.loads(fh.read())


def test_the_window_gates_commands_and_takes_control():
    c = Control("2468", trust_local=False)                                              # the test's 127.0.0.1 counts as another computer
    srv, url = _server(c)
    try:
        st = _req(url, "/state.json", sid=A, name="dmitrii")
        assert st["control"]["you"]["needs_pin"] and st["control"]["where"]["where"] == "nx"
        r = _req(url, "/cmd", {"word": "handover"}, sid=A, name="dmitrii")
        assert not r["ok"] and "PIN needed" in r["text"]
        r = _req(url, "/control", {"op": "take", "pin": "2468"}, sid=A, name="dmitrii")
        assert r["ok"] and r["control"]["you"]["commander"]
        r = _req(url, "/cmd", {"word": "handover"}, sid=A, name="dmitrii")
        assert r["text"] == "this page is read-only (no boat broker link)"              # past the gate (no link in this test)
        r = _req(url, "/ocs", {"line": "stop_reset"}, sid=B, name="jaison")
        assert not r["ok"] and "you are a viewer - dmitrii" in r["text"]
        r = _req(url, "/tune", {"key": "x", "value": 1}, sid=B, name="jaison")
        assert not r["ok"] and "viewer" in r["text"]
        assert len(_req(url, "/state.json", sid=B)["control"]["sessions"]) == 2
    finally:
        srv.shutdown()
        srv.server_close()


def test_the_ocs_lines_go_through_the_hub_without_an_ocs_page():
    r = BW.ocs_send(None, None, "enable_auto")
    assert not r["ok"] and "no OCS configured" in r["text"]
    r = BW.ocs_send(None, "127.0.0.1:1", "start_task m3t1")
    assert not r["ok"] and r["text"].startswith("refused by the page: only")           # the allow-list before any connection
    r = BW.ocs_send(None, "127.0.0.1:1", "enable_auto")
    assert not r["ok"] and "not reachable" in r["text"]                                 # the hub down: said
    r = BW.ocs_send(None, "127.0.0.1:1", "", tune={"key": "nonsense", "value": 1})
    assert not r["ok"] and r["text"].startswith("refused by the page")


# ---- the launcher, the scripts -----------------------------------------------------------------------------------------------
def test_the_launcher_starts_the_window_where_picked():
    P = L.load_presets()
    assert P["event"]["usv_window"] == "nx" and P["lab"]["usv_window"] == "mac" and P["sim"]["usv_window"] == "mac"
    assert "usv_window" in L.FIELDS and L.FIELDS["usv_window"] == ("mac", "nx", "off", "mac-classic", "nx-classic")     # v0.1.74: + classic
    p = L.build_plan(dict(P["event"]), stamp="S", net_env=NET)
    nx_steps = [s for s in p["steps"] if "run_boat_window.sh" in s["cmd"]]
    assert len(nx_steps) == 1 and nx_steps[0]["host"] == "nx" and "nx-alias" in nx_steps[0]["cmd"] and "--lan" in nx_steps[0]["cmd"]
    assert any(x["key"] == "nx:bwin:web" for x in p["procs"]) and any("8098" in n for n in p["notes"])
    p = L.build_plan(dict(P["lab"]), stamp="S", net_env=NET)
    st = [s for s in p["steps"] if "run_boat_window.sh" in s["cmd"]]
    assert st[0]["host"] == "mac" and st[0]["cmd"] == "scripts/run_boat_window.sh --ts" and any(x["key"] == "mac:bwin:web" for x in p["procs"])
    p = L.build_plan(dict(P["lab"], usv_window="off"), stamp="S", net_env=NET)
    assert not any("run_boat_window.sh" in s["cmd"] for s in p["steps"])
    assert 'data-k="usv_window"' in (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()


def test_the_window_script_binds_the_team_lan_never_everything():
    sh = (ROOT / "scripts" / "run_boat_window.sh").read_text()
    assert 'BIND="${BIND:-${BOAT_HOST:-127.0.0.1}}"' in sh and 'BIND="${BIND:-${OCS_HOST:-127.0.0.1}}"' in sh
    assert "0.0.0.0" not in sh.split("set -uo")[1]
    assert 'OCSMQ="${OCS_HOST:-127.0.0.1}:${HUB_PORT:-1882}"' in sh and "--ocs-mqtt" in sh and "a USV window runs already here" in sh
    for f in ("stop_usv.sh", "stop_ocs.sh"):
        assert "run_boat_window.sh\" stop" in (ROOT / "scripts" / f).read_text(), f
    src = (ROOT / "optiprimex" / "tools" / "boat_web.py").read_text()
    assert "commands from other computers need the PIN %s" in src and 'DEFAULT_PIN = "0000"' in src and '"local/%s/ui/window"' in src   # v0.1.78.9: 0000


# ---- the pages ---------------------------------------------------------------------------------------------------------------------
def test_the_pages_show_who_commands():
    page = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text()
    for needle in ('id="ctlbar"', 'id="ctl-take"', 'id="ctl-pin"', 'id="ctl-name"', "'X-Window-Session':WIN.sid", "YOU COMMAND", "VIEWER",
                   "fetch('/control'"):
        assert needle in page, needle
    from optiprimex.tools import ocs_web as OW
    t = OW.WindowTap("usv")
    assert t.view()["state"] == "none"
    t.on_message(json.dumps({"where": "nx", "url": "http://10.0.0.3:8098/", "commander": {"name": "j", "addr": "10.0.0.9"},
                             "sessions": [{"name": "j"}], "pin": True}).encode(), now=10.0)
    v = t.view(now=11.0)
    assert v["state"] == "up" and v["where"] == "nx" and v["commander"]["name"] == "j" and t.view(now=20.0)["state"] == "stale"
    assert 'sysRow("USV · window (team LAN)"' in (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()


def test_quick_list_has_this_file():
    assert "tests/test_v0163.py" in (ROOT / "Makefile").read_text()
