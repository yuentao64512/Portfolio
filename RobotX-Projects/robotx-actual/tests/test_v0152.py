"""v0.1.52 (Dmitrii's round after v0.1.51.1) - the OCS page's part: OVERVIEW rearranged (left SYSTEM + MISSION · VEHICLE + USV / UAV;
middle EVENTS, LAST COMMANDS, CONSOLE, the mini-map, the CV stream; right ALARMS above RUN SEQUENCE), the messages highlighted, the
SYSTEM card's rows with a status dot and their detail in a tooltip, one tooltip instead of two, the SIMULATION banner; the launcher's
CONSOLE card (a started tmux window's last lines, read only). The boat window: tests/test_v0152_wa.py; the sim fixes: test_v0152_wb.py."""
from __future__ import annotations

import json
import subprocess
import threading
import urllib.request

from optiprimex.tools import launcher as L

from .conftest import ROOT, free_port

TOOLS = ROOT / "optiprimex" / "tools"


def page():
    return (TOOLS / "ocs_web.html").read_text()


def _ov(p):
    i = p.index('<section class="tab" id="t-OVERVIEW">')
    return p[i:p.index("</section>", i)]


def test_overview_columns_in_the_asked_order():
    ov = _ov(page())
    cols = ov.split('\n   <div class="grid">')
    assert len(cols) >= 4, len(cols)
    left, mid, right = cols[1], cols[2], cols[3]
    assert "SYSTEM · PLATFORMS · LINKS" in left and "MISSION · VEHICLE" in left and 'id="ov-usv"' in left and "LAST COMMANDS" in left
    # v0.1.54 (Dmitrii): COMMANDS under the CONSOLE; FLOW above RUN SEQUENCE; the warnings below it
    assert mid.index("EVENTS") < mid.index('id="cli"') < mid.index("<h2>COMMANDS") < mid.index('id="minimap"') < mid.index('id="ov-cam"')
    assert right.index("<h2>FLOW") < right.index("RUN SEQUENCE") < right.index("ALARMS · WARNINGS · CAUTIONS")
    assert 'id="cli"' in ov and page().count('id="cli"') == 1


def test_messages_rows_dots_and_one_tooltip():
    p = page()
    assert "function hiText(t)" in p and "hiText(x.text)" in p and ".log .lg:nth-child(even)" in p and ".hb-bad{color:var(--alarm)}" in p
    assert 'class="sd ${d}"' in p and ".sd.warn{background:var(--warn)}" in p and "tipv" in p
    assert 'sysRow("USV · RC receiver",h.rc_ok==null?NP("unknown"):(h.rc_ok?"ok":"RC LOST")' in p   # the detail in the tooltip
    assert "function tipPark(el)" in p and 'el.removeAttribute("title")' in p and "TIP_EL.title=TIP_EL.dataset.t0" in p
    i = p.index("<h2>SYSTEM · PLATFORMS · LINKS")
    assert "presence · $SYS" in p[i:i + 400] and 'data-tip="presence' in p[i:i + 400]                 # the legend moved to a tooltip


def test_the_simulation_banner_and_the_overview_camera():
    p = page()
    assert 'id="simban"' in p and "function simBanner()" in p and 'f.preset==="sim"||f.fcu==="fake"' in p
    assert "function ovCam()" in p and 'snapshot.jpg?t="+Date.now()' in p[p.index("function ovCam()"):]


def test_the_launcher_console(tmp_path, monkeypatch):
    assert L.pane_tail(None, "bad;rm -rf", "x") is None and L.pane_tail(None, "a", "b c") is None    # only plain names travel
    sess = f"t52_{free_port()}"
    subprocess.run(["tmux", "new-session", "-d", "-s", sess, "-n", "w1", "bash -c 'echo hello-console-52; sleep 30'"], check=True)
    try:
        import time
        time.sleep(0.8)
        lines = L.pane_tail(None, sess, "w1")
        assert lines is not None and any("hello-console-52" in x for x in lines)
        runner = L.Runner(dry=True)
        runner.record_path = str(tmp_path / "last.json")
        plan = {"procs": [{"key": f"mac:{sess}:w1", "kind": "tmux", "host": "mac", "ssh": None, "session": sess, "window": "w1", "label": "test"}]}
        (tmp_path / "last.json").write_text(json.dumps({"cfg": {}, "plan": plan, "t": 1}))
        srv = L.Server(("127.0.0.1", free_port()), L.make_handler(runner, None, None, kit=str(tmp_path)))
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        u = f"http://127.0.0.1:{srv.server_address[1]}/console.json"
        try:
            r = json.loads(urllib.request.urlopen(u, timeout=5).read())
            assert [w["key"] for w in r["windows"]] == [f"mac:{sess}:w1"] and "lines" not in r
            r = json.loads(urllib.request.urlopen(u + f"?key=mac:{sess}:w1", timeout=5).read())
            assert r["ok"] and any("hello-console-52" in x for x in r["lines"])
            r = json.loads(urllib.request.urlopen(u + "?key=mac:other:x", timeout=5).read())
            assert "lines" not in r                                                    # only the plan's windows
        finally:
            srv.shutdown()
    finally:
        subprocess.run(["tmux", "kill-session", "-t", sess])
    h = (TOOLS / "launcher.html").read_text()
    assert 'id="conwin"' in h and 'id="con"' in h and "setInterval(conPoll,2000)" in h


def test_the_fault_word_says_when_the_cause_is_gone():
    from optiprimex.tools.ocs_web import control_word
    w = control_word({"state": "FAULT", "fault": "EKF_BAD", "health": {"faults": ["EKF_BAD"]}}, {"r": "R2"}, False)
    assert w["why"] == "fault EKF_BAD" and "clear the cause" in w["hint"]
    w = control_word({"state": "FAULT", "fault": "EKF_BAD", "health": {"faults": []}}, {"r": "R2"}, False)
    assert w["why"] == "fault EKF_BAD latched - the cause is gone" and w["hint"] == "confirm_safe, then enable_auto"
