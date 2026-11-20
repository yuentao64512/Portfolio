"""v0.1.54 (Dmitrii's round after v0.1.53) - the OCS page's part: the OVERVIEW camera (the v0.1.52 bug: /cam.json keeps the source
under `status`), OVERVIEW re-ordered (COMMANDS under the CONSOLE, FLOW above RUN SEQUENCE, the warnings below), the mission picker +
preselect (--auto) + start_task inside RUN SEQUENCE, every RUN SEQUENCE command says sent → answer (declare follows R3), START
REPORTING + STOP & RESET under the FCU buttons, newest-on-top EVENTS / LOG, the map controls (pan, zoom, box zoom, fit) on the
OCS map and the SIM page. The boat window / CV boxes / the STOP & RESET macro: test_v0154_b.py; buoys / S6 / clearance: test_v0154_c.py."""
from __future__ import annotations

from .conftest import ROOT

TOOLS = ROOT / "optiprimex" / "tools"


def page():
    return (TOOLS / "ocs_web.html").read_text()


def test_the_overview_camera_reads_the_status():
    p = page()
    assert "const camSt=()=>" in p and "(CAM.status||CAM)" in p and "CAM&&CAM.source" not in p
    assert "!!camSt();if(want===OVCAM_ON)return;" in p and 'const cs=camSt();r+=sysRow("camera stream"' in p


def test_run_sequence_mission_row_and_feedback():
    p = page()
    for i in ("rs-sel", "rs-auto", "rs-pre", "rs-go", "rs-note"):
        assert f'id="{i}"' in p, i
    assert 'rsAct(qLine("start_task"))' in p and 'l+=" --auto"' in p and "function rsAct(line)" in p
    assert "function rsWatch()" in p and "✓ declared · RunDeclaration seq" in p and "no R3 after 15 s" in p
    assert "let r=null;try{r=await post(\"/cmd\",{line})}catch(e){} tick();return r}" in p   # send() returns the reply
    assert 'if($("rs-sel"))$("rs-sel").value=$("qsel").value' in p                            # kept in sync with MISSION's


def test_the_bar_group_and_newest_on_top():
    p = page()
    i = p.index('<span class="fcugrp">')
    grp = p[i:p.index('id="srbtn"', i) + 20]
    assert grp.index('id="fb-manual"') < grp.index('id="repbtn"') and 'id="srbtn"' in grp
    assert 'rsAct("stop_reset")' in p
    assert "S.log.slice(-60).reverse().map(logLine)" in p and ".slice().reverse().map(logLine)" in p


def test_the_map_controls():
    for f in (TOOLS / "map.html", ROOT / "optiprimex" / "sim" / "sim.html"):
        s = f.read_text()
        for b in ("nv-u", "nv-d", "nv-l", "nv-r", "nv-in", "nv-out", "nv-box", "nv-fit"):
            assert f'id="{b}"' in s, (f.name, b)
        assert 'id="boxsel"' in s and "navBox" in s and "navZoom(1.4)" in s


def test_the_boxes_on_the_ocs_camera():
    p = page()
    assert 'id="c-ov"' in p and 'id="ov-ov"' in p and 'preserveAspectRatio="xMidYMid meet"' in p and 'id="det-on"' in p
    assert 'fetch("/dets.json"' in p and "function drawDets(id)" in p and "boxes not published" in p
    import optiprimex.tools.ocs_web as OW
    src = (TOOLS / "ocs_web.py").read_text()
    assert "timeout=25.0 if word == \"stop_reset\" else 10.0" in src   # v0.1.86.9: every press queues (10 s), STOP & RESET 25 s                                  # STOP & RESET is never "busy"


def test_the_judge_page_marks_a_silent_vehicle():
    j = (TOOLS / "judge_web.py").read_text()
    assert "const silent=o.age_s==null||o.age_s>3;" in j and "SILENT" in j and ".veh.silent{opacity:.55}" in j
