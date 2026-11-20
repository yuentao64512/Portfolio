"""v0.1.78.9 (Dmitrii 2026-11-09, Team Village): "remove the PIN and make CONFIRM SAFE on the USV side as well", then "what is the pin?
let's make it 0000 - clearly visible in the launcher": the USV window's PIN is a fixed 0000 (USV_WINDOW_PIN / --pin change it), shown on
the launcher's PROGRESS link, step and note; one commander at a time; ✓ CONFIRM SAFE in its header while KILLED / FAULT (disabled while the
kill / E-stop is held; two clicks within 4 s; the existing OCS line)."""
from __future__ import annotations

import json
import os
import re
import subprocess
import time

import pytest

from optiprimex.tools import boat_web as BW
from optiprimex.tools.window_control import Control

from .conftest import ROOT
from .test_v0174 import CHROME, _model, _serve


def _killed(kill=False):
    m = _model()
    m.on_message("team/optiprime/v1/status/usv", json.dumps({"body": {"state": "KILLED", "kill": kill, "estop": False,
                                                                       "health": {"faults": [], "rc_ok": True}}}).encode(), now=time.monotonic())
    return m


def test_the_n6_block_carries_the_latch():
    n = _killed().snapshot()["n6"]
    assert n["vstate"] == "KILLED" and n["sf"] is False and n["estop"] is False
    assert _killed(kill=True).snapshot()["n6"]["sf"] is True


def test_the_pin_is_0000_and_shown_in_the_launcher(monkeypatch):
    src = (ROOT / "optiprimex" / "tools" / "boat_web.py").read_text()
    assert BW.DEFAULT_PIN == "0000" and "pin = a.pin.strip() or DEFAULT_PIN" in src and "new_pin" not in src.split("def main")[1]
    c = Control(BW.DEFAULT_PIN)
    sid = c.seen("a1b2c3d4e5f6a7b8", "laptop", "192.168.1.120")         # a browser on another computer
    assert c.take(sid, "laptop", "192.168.1.120")[0] is False                   # needs the PIN
    assert c.take(sid, "laptop", "192.168.1.120", pin="0000")[0] is True
    sid2 = c.seen("b1b2c3d4e5f6a7b8", "other", "192.168.1.5")
    assert not c.may_command(sid2, "other", "192.168.1.5")[0]          # one commander at a time
    from optiprimex.tools import launcher as L
    monkeypatch.delenv("USV_WINDOW_PIN", raising=False)
    assert L.usv_window_pin({}) == "0000" and L.usv_window_pin({"USV_WINDOW_PIN": "4321"}) == "4321"
    P = L.load_presets()
    p = L.build_plan(dict(P["event"], net="lan"), stamp="S", net_env={"BOAT_HOST": "10.1.0.218", "ORIN_SSH": "u@10.1.0.218"})
    st = [s for s in p["steps"] if "run_boat_window.sh" in s["cmd"]][0]
    assert st["label"].endswith("PIN 0000") and any("with the PIN 0000" in n for n in p["notes"])
    uw = {x["name"]: x for x in L.page_links(dict(P["event"], net="lan"), net_env={"BOAT_HOST": "10.1.0.218"})}["USV window"]
    assert uw["pin"] == "0000"
    page = (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    assert '<b class="pin"' in page and "PIN ${esc(p.pin)}" in page and "USV window PIN ${E(r.pin)}" in page
    import pathlib, tempfile
    d = pathlib.Path(tempfile.mkdtemp())
    (d / "network.env").write_text("NET_PROFILE=lan\n")
    (d / "network.lan.env").write_text("BOAT_HOST=10.1.0.218\nORIN_SSH=u@10.1.0.218\n")
    t = L.network_table(str(d), ping=lambda a: False, auth=False)
    assert {r["key"]: r for r in t["rows"]}["BOAT_HOST"]["pin"] == "0000"                       # the NX row carries it
    assert "confirm_safe" in BW.OCS_LINES and len(BW.WHITELIST) == 12                       # the words unchanged (v0.1.81: + station_keep after it)


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
@pytest.mark.parametrize("kill", [False, True])
def test_the_button_on_the_window(kill):
    srv, port, old = _serve(_killed(kill))
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{port}/"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    assert not re.search(r'N6CS|Uncaught', r.stderr), r.stderr[-1500:]
    dom = r.stdout
    r2 = dom[dom.index('id="n6r2"'):]
    b = re.search(r'<button[^>]*id="n6csafe"[^>]*>', r2).group(0)
    assert "hidden" not in b and ("disabled" in b) == kill, b
    assert ">✓ CONFIRM SAFE<" in r2
    page = (ROOT / "optiprimex" / "tools" / "boat_web.html").read_text()
    assert "body:JSON.stringify({line:'confirm_safe'})" in page and "Date.now()-N6CS>4000" in page
