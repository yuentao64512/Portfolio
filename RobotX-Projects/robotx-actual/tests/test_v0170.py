"""v0.1.70 (WP26 = docs/OVERNIGHT_2026-11-09_GUI.md N2): the one-row bar, the context line, RUN (L1) rebuilt, the console drawer, the
declaration constructor with the RunDeclaration "as it goes on the wire" - equal, field by field, to what the judge receives."""
from __future__ import annotations

import json
import os
import re
import subprocess
import threading

import pytest

from optiprimex.ocs import rc_wire

from .conftest import ROOT, free_port
from .test_v0164 import INSIDE, UID, _declare_done, intent, make_app, uav_status

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
FIELDS = ("vehicle_ids", "tiers", "uav_geofence", "uav_geofence_max_altitude_hae_m", "uav_pilot_callsign")


def _sent_fields(judge) -> dict:
    """What the judge receives: the last RxRequest body through the same encoder the protobuf client uses, decoded back."""
    dec = rc_wire.decode_request(rc_wire.encode_request("SGIT", 9, judge.requests[-1], t=0.0))
    return {k: dec[k] for k in FIELDS}


@pytest.mark.parametrize("prof", ["pool/ocs_pool.yaml", "course/ocs_course.yaml"])
def test_the_preview_is_what_the_judge_receives(prof):
    app, bus, judge, _ = make_app(prof)
    p = app.declaration_preview()
    assert p["fields"] and p["text"].startswith('vehicle_ids: "USV1"') and p["bytes"] > 0
    assert _declare_done(app, bus).result == "accepted"
    assert _sent_fields(judge) == p["fields"]                                    # field by field
    st = app.state_body()
    assert st["declaration_sent"]["vehicle_ids"] == ["USV1"] and st["declaration_preview"]["fields"]["vehicle_ids"] == ["USV1"]


def test_the_preview_follows_the_inputs_uav_and_t4():
    app, bus, judge, _ = make_app()
    app.set_mission("T1", False, {"t4": "off"})
    assert app.declaration_preview()["fields"]["tiers"]["T4"] == "TIER_NONE"      # the T4 tick (N3; today --t4) changes it live
    app.set_mission("T1", False, {"tier": "advanced"})
    assert app.declaration_preview()["fields"]["tiers"]["T1"] == "TIER_ADVANCED"
    assert app.authority.on_intent("t", intent("DECLARE_UAV", args={"on": True})).result == "accepted"
    p = app.declaration_preview()
    assert p["uav_declared"] and p["uav_refusal"]                                # not heard yet: the refusal shows in the constructor
    uav_status(app)
    p = app.declaration_preview()
    assert p["uav_refusal"] is None and p["fields"]["vehicle_ids"] == ["USV1", UID] and p["fields"]["uav_geofence"] == INSIDE
    assert _declare_done(app, bus).result == "accepted"
    assert _sent_fields(judge) == p["fields"]


def test_declare_unchanged_by_the_refactor():
    """declare() and the preview share _declaration_inputs: a UAV refusal still alerts and FENCE_REJECTs (the v0.1.64 rules)."""
    from .test_v0164 import OUTSIDE, cmds
    app, bus, judge, _ = make_app()
    app.authority.on_intent("t", intent("DECLARE_UAV", args={"on": True}))
    uav_status(app, points=OUTSIDE)
    r = app.authority.on_intent("t", intent("DECLARE"))
    assert r.result == "refused" and "RoboCommand would REJECT it" in r.reason and len(cmds(bus, "FENCE_REJECTED", UID)) == 1
    assert not judge.requests


def test_the_state_contract_only_gains_keys():
    app, *_ = make_app()
    st = app.state_body()
    for k in ("r", "r_state", "declaration_seq", "judge", "vehicles", "uav", "reporting", "mission", "judge_view", "timed_out",
              "declaration_checks", "session"):
        assert k in st, k
    assert {"declaration_preview", "declaration_sent"} <= set(st)


def test_the_page_markup():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ('const TAB_LABEL={OVERVIEW:"RUN"', 'id="drawer"', 'data-pane="dp-cmd"', 'data-pane="dp-sys"', 'data-pane="dp-wire"',
                   'if(e.key===":"', 'localStorage.setItem("ocs.drawerH"', '"■ STOP RUN"', 'rsAct("stop_reset")', 'mv("rs-card","n2-l")',
                   'mv("minimap","n2-r")', '"stopallbtn","resetallbtn"', 'appendChild($("repbtn"))', 'id="decl-con"', 'data-copy="dc-pre"',
                   'send("declare uav "', 'for(const id of ["simban","flavour","themebar"])'):
        assert needle in page, needle
    v1 = (ROOT / "optiprimex" / "tools" / "ocs_web_v1.html").read_text()
    assert "classic · v0.1.42 page frozen 2026-11-06" in v1 and "drawN2" not in v1      # the classic page untouched


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_chrome_walk_at_1440(tmp_path):
    """Every tab reachable, the bar in one row, the drawer opens, every v0.1.68 word in the console's list, no script error."""
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                            "--v=0", "--dump-dom", f"http://127.0.0.1:{a.web_port}/?probe=1"], capture_output=True, text=True, timeout=90)
    finally:
        srv.shutdown()
    dom, err = r.stdout, r.stderr
    assert not re.search(r'"(N2|sessions|v2 cards) [A-Za-z]*Error', err), err[-2000:]
    for t in ("OVERVIEW", "MISSION", "CAMERA", "USV", "UAV", "COMMS", "JUDGE", "LOG"):
        assert f'data-t="{t}"' in dom, t
    probe = re.search(r'<pre id="n2probe">(.*?)</pre>', dom, flags=re.S).group(1)
    tops = {int(m.group(1)) for m in re.finditer(r"flex \d+ @\d+,(\d+)", probe)} | {int(m.group(1)) for m in re.finditer(r"block \d+ @\d+,(\d+)", probe)}
    assert max(tops) - min(tops) < 20, probe                                      # one row at 1440
    for idn in ("ctx", "rchip", "vchip", "repchip", "n2-l", "n2-r", "run-mis", "run-veh", "run-links", "run-last", "decl-con", "sys-stacks", "jd-rep"):
        assert f'id="{idn}"' in dom, idn
    words = set(re.findall(r'<option value="([a-z_0-9]+)"', dom.split('id="words"', 1)[1].split("</datalist>", 1)[0]))
    v68 = {"declare", "enable_auto", "disable_auto", "start_task", "goto", "hold", "resume", "abort", "confirm_safe", "end_run", "inject", "reset",
           "step", "skip", "mission", "select_target", "save_point", "del_point", "clear_map", "home", "tune", "fcu_mode", "arm", "disarm",
           "rc6_policy", "fence", "ready", "manual", "next", "state", "report", "stop_reset"}
    assert v68 - {"report"} <= words, v68 - words                                  # `report` was never in the list (v0.1.53): typed only
    from optiprimex.tools import bench_tui as T
    assert v68 <= T.INTENT_WORDS                                                 # what the page's console accepts (Web.command)
