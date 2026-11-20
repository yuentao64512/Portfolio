"""v0.1.78.7 (Dmitrii 2026-11-09, Team Village): "confirm safe - hard to find" -> ✓ CONFIRM SAFE in the ALARMS card while KILLED / FAULT
(disabled while the kill / E-stop is held; asks first; the existing word confirm_safe); "after a warning is resolved I need to see that it is
resolved" -> the alarms cleared in the last 15 min under the active ones; "the AGX observer - wire the button to the launcher" -> observer
▶ / ■ on the launcher's AGX row (cv/buoys/run.sh --headless in tmux agx window obs; refused without the agx session or with the ZED held)."""
from __future__ import annotations

import json
import os
import re
import subprocess
import threading
import time
import urllib.request

import pytest

from optiprimex.tools import launcher as L

from .conftest import ROOT, free_port

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def test_recent_resolved_in_the_alarm_body():
    from optiprimex.tools import ocs_web as OW
    b = OW.AlarmBook()
    now = time.time()
    b.history += [{"key": "a", "level": "WARNING", "text": "RC LOST (test)", "first_t": now - 60, "cleared_t": now - 52, "acked": False},
                  {"key": "b", "level": "ALARM", "text": "old one", "first_t": now - 2000, "cleared_t": now - 1900, "acked": True}]
    rec = b.body(now=now)["recent"]
    assert [r["text"] for r in rec] == ["RC LOST (test)"] and rec[0]["dur_s"] == 8.0


def _walk(tmp_path, kill):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    with web.m.lock:
        web.m.status = {"state": "KILLED", "robot_state": "STATE_KILLED", "mode": "MANUAL", "armed": True, "kill": kill, "estop": False, "health": {}}
        web.m.status_t = time.time()
    web.alarms.history.append({"key": "rc", "level": "WARNING", "text": "RC LOST (the receiver) (test)", "first_t": time.time() - 40,
                               "cleared_t": time.time() - 32, "acked": False})
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{a.web_port}/?probe=1#OVERVIEW"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    assert not re.search(r'"N[0-9]+ [A-Za-z]*Error|Uncaught', r.stderr), r.stderr[-1500:]
    return r.stdout


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_confirm_safe_and_resolved_on_the_page(tmp_path):
    dom = _walk(tmp_path, kill=False)
    card = dom[dom.index('id="ov-alcard"'):]
    card = card[:card.index('id="al-histbox"')]
    btn = re.search(r'<button[^>]*id="al-csafe"[^>]*>', card).group(0)
    assert "disabled" not in btn and "✓ CONFIRM SAFE" in card and "KILLED latched" in card
    assert 'class="resolved"' in card and "RC LOST (the receiver) (test)" in card and re.search(r"resolved \d\d:\d\d:\d\d", card)
    assert 'id="ov-alcard" class="card"' in dom or 'class="card quiet"' not in card                          # not folded to one line
    dom = _walk(tmp_path, kill=True)
    btn = re.search(r'<button[^>]*id="al-csafe"[^>]*>', dom).group(0)
    assert "disabled" in btn and "release it on the transmitter first" in dom


def test_the_page_sends_the_existing_word():
    p = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'ask(`<b>confirm_safe</b>' in p and ',null,()=>send("confirm_safe"))' in p
    assert "jumpTo(\"OVERVIEW\",\"al-csafe\")" in p


def test_the_observer_commands():
    # v0.1.82: OUR observer (buoys_multi) through scripts/agx_observer.sh (the same script as the OCS page's OBSERVER button)
    s = L.observer_cmd("start")
    assert s == "cd ~/Desktop/dm3/MASTER_SCRIPTS && bash scripts/agx_observer.sh start --detector default"
    assert L.observer_cmd("start", detector="model-a").endswith("--detector model-a")
    sh = (ROOT / "scripts" / "agx_observer.sh").read_text()
    for needle in ("tmux has-session -t \"$SESSION\"", "fuser /dev/video0", "-n obs", "buoys_multi/run.sh --headless", "multi_buoy_observe[r]",
                   "agx_cam.sh\" start --source dir", "tmux send-keys -t \"$SESSION:obs\" C-c", "kill-window -t \"$SESSION:obs\""):
        assert needle in sh, needle
    assert subprocess.run(["bash", "-n", str(ROOT / "scripts" / "agx_observer.sh")]).returncode == 0
    assert "multi_buoy_observer" in (ROOT / "scripts" / "agx_cam.sh").read_text()      # auto = dir while ours runs (was: zed = no stream)
    for a in ("start", "stop", "status"):
        assert subprocess.run(["bash", "-n"], input=L.observer_cmd(a), text=True).returncode == 0, a
    with pytest.raises(ValueError):
        L.observer_cmd("rm")

    class R:
        def __init__(self, out, rc=0):
            self.stdout, self.stderr, self.returncode = out, "", rc
    seen = []
    assert L.agx_observer("status", "agx@10.1.0.99", run=lambda argv: seen.append(argv) or R("observer: up\n")) == {"ok": True, "state": "up", "text": "observer: up"}
    assert "BatchMode=yes" in seen[0] and seen[0][-2] == "agx@10.1.0.99"
    assert L.agx_observer("start", "agx@10.1.0.99", run=lambda argv: R("refused: the ZED is held by pid 4242 (...)\n", 5))["ok"] is False
    assert L.agx_observer("start", "x; ls")["ok"] is False


def test_the_table_and_the_route(monkeypatch, tmp_path):
    (tmp_path / "network.env").write_text("NET_PROFILE=lan\n")
    (tmp_path / "network.lan.env").write_text("AGX_HOST=10.1.0.99\nAGX_SSH=agx@10.1.0.99\n")
    t = L.network_table(str(tmp_path), ping=lambda a: True, auth=lambda tgt: "key", observer=lambda a, tgt: {"ok": True, "state": "down", "text": "observer: down"})
    R = {r["key"]: r for r in t["rows"]}
    assert R["AGX_HOST"]["observer"]["state"] == "down" and "observer" not in R["BOAT_HOST"]
    monkeypatch.setattr(L, "ssh_target", lambda which, kit=None: "agx@10.1.0.99")
    port = free_port()
    srv = L.Server(("127.0.0.1", port), L.make_handler(L.Runner(dry=True)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    def post(body):
        rq = urllib.request.Request(f"http://127.0.0.1:{port}/observer", data=json.dumps(body).encode(), method="POST",
                                    headers={"Content-Type": "application/json", "X-Launch": "1"})
        try:
            return json.loads(urllib.request.urlopen(rq, timeout=5).read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read())
    try:
        j = post({"action": "start"})
        assert j["dry"] and "agx@10.1.0.99" in j["text"] and "agx_observer.sh start --detector default" in j["text"]   # v0.1.82
        assert post({"action": "reboot"})["ok"] is False
    finally:
        srv.shutdown()
    page = (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    assert 'data-obs="${r.observer.state==="up"?"stop":"start"}"' in page and 'fetch("/observer"' in page
