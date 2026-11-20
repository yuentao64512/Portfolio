"""v0.1.44 (ROADMAP WP10i, OCS_V2_SPEC §3-§10, §13): the regrouping (OVERVIEW · MISSION · USV CONTROL ↗ · UAV · COMMS & DIAGNOSTICS ·
JUDGE · LOG · ⚙ SETTINGS, HELP behind the bar's ?), the six cards, the alarm model (levels, escalation, acknowledgement, history), the
T4 modifier (the picker flag -> the declared T4 tier; the WP9c row soft unless t4_required), the scene layer, the picker on MISSION only."""
from __future__ import annotations

import json
import re
import threading
import urllib.request

from .conftest import ROOT, free_port

TOOLS = ROOT / "optiprimex" / "tools"


def page():
    return (TOOLS / "ocs_web.html").read_text()


def test_the_regrouped_tabs_and_their_old_names():
    p = page()
    assert 'const TABS=["OVERVIEW","MISSION","CAMERA","USV","UAV","COMMS","JUDGE","LOG","HELP","SETTINGS"]' in p   # v0.1.60: CAMERA
    alias = json.loads(re.search(r"const TAB_ALIAS=(\{[^}]*\})", p).group(1).replace("MAIN", '"MAIN"').replace("CONTROL:", '"CONTROL":')
                       .replace("MAP:", '"MAP":').replace("CAMERA:", '"CAMERA":').replace("BOAT:", '"BOAT":').replace("NETWORK:", '"NETWORK":')
                       .replace("SYSTEM:", '"SYSTEM":').replace('""MAIN""', '"MAIN"'))
    assert alias == {"MAIN": "OVERVIEW", "CONTROL": "MISSION", "MAP": "MISSION", "BOAT": "USV", "NETWORK": "COMMS", "SYSTEM": "COMMS"}
    for t in ("OVERVIEW", "MISSION", "CAMERA", "USV", "UAV", "COMMS", "JUDGE", "LOG", "HELP", "SETTINGS"):     # v0.1.60: CAMERA its own tab
        assert p.count(f'<section class="tab" id="t-{t}">') == 1, t
    for gone in ("MAIN", "CONTROL", "BOAT", "NETWORK", "SYSTEM", "MAP"):
        assert f'id="t-{gone}"' not in p, gone
    assert '"USV CONTROL ↗"' in p and 'COMMS:"SYSTEM"' in p and 'id="helpbtn"' in p     # v0.1.72 (N4): COMMS & DIAGNOSTICS -> SYSTEM
    for el in ("ck", "rscard", "run", "veh", "qsel", "minimap", "rc-ans", "cmds", "cli", "ev", "mapf", "c-img", "cw", "csel", "c-chain", "b-st",
               "n-hub", "sys", "j-log", "warns", "flow"):
        assert p.count(f'id="{el}"') == 1, el                                       # every moved element still exists once


def _section(p, sid):
    i = p.index(f'<section class="tab" id="t-{sid}">')
    return p[i:p.index("</section>", p.index("</details>", i) if sid == "MISSION" else i)]


def test_the_six_cards_and_the_picker_on_mission_only():
    p = page()
    ov = _section(p, "OVERVIEW")
    for needle in ("SYSTEM · PLATFORMS · LINKS", "RUN SEQUENCE", 'class="card minicard"', "ALARMS · WARNINGS · CAUTIONS",
                   "MISSION · VEHICLE", "LAST COMMANDS · LIFECYCLE", 'id="ov-usv"', 'id="ov-uav"'):
        assert needle in ov, needle
    for n in range(1, 7):                                                            # v0.1.59 (B2): six steps, the checklist the gate above
        assert f'id="lad-{n}"' in ov
    assert 'id="lad-7"' not in ov and 'id="ck"' in ov and 'id="rscard"' in ov        # the RunStart slot in step 4
    assert 'id="qsel"' not in ov and 'id="csel"' not in ov                            # §13: the picker on MISSION only
    ms = _section(p, "MISSION")
    assert 'id="csel"' in ms and 'id="qsel"' in ms and 'id="mapf"' in ms and 'id="c-img"' in ms and 'id="t4flag"' in ms and 'id="ms-det"' in ms
    assert '<section class="tab" id="t-COMMS">' in p and 'id="cd-streams"' in p and 'id="copydiag"' in p
    assert "Garuda" in p and 'sysRow("UAV · adapter"' in p and "not connected · unknown" not in p      # v0.1.65: the UAV card is live
    assert '` --t4 ${$("t4sel").value}`' in p                                   # v0.1.71: T4 off | core | advanced


def test_the_alarm_model():
    from optiprimex.tools.ocs_web import ESCALATE_S, AlarmBook
    t = [1000.0]
    ab = AlarmBook(clock=lambda: t[0])
    b = ab.update([{"style": "bad", "text": "RC LOST (the receiver) while AUTO: the transmitter is the safety"},
                   {"style": "warn", "text": "PERCEPTION STALE: sim_observer silent for 3 s - CAMERA"},
                   {"style": "warn", "text": "the mission manager loop is slow: loop_slow 120 ms"}])
    lv = [a["level"] for a in b["active"]]
    assert lv == ["ALARM", "WARNING", "CAUTION"] and b["unacked"] == 2 and b["n_caution"] == 1
    t[0] += 1
    b = ab.update([{"style": "bad", "text": "RC LOST (the receiver) while AUTO: x"}, {"style": "warn", "text": "PERCEPTION STALE: sim_observer silent for 4 s"}])
    assert len(b["active"]) == 2 and b["history_n"] == 1                            # the caution cleared itself -> history; the numbers do not split keys
    assert ab.ack(AlarmBook.key("RC LOST (the receiver) while AUTO")) == 1
    b = ab.update([{"style": "bad", "text": "RC LOST (the receiver) while AUTO: x"}, {"style": "warn", "text": "PERCEPTION STALE: sim_observer silent for 9 s"}])
    rc = next(a for a in b["active"] if a["key"].startswith("RC LOST"))
    assert rc["acked"] and rc["level"] == "ALARM"                                   # acknowledged, the condition stays
    t[0] += ESCALATE_S + 1
    b = ab.update([{"style": "bad", "text": "RC LOST (the receiver) while AUTO: x"}, {"style": "warn", "text": "PERCEPTION STALE: sim_observer silent for 300 s"}])
    ps = next(a for a in b["active"] if a["key"].startswith("PERCEPTION"))
    assert ps["level"] == "ALARM" and ps["escalated"]                               # a warning unacknowledged 5 min -> ALARM
    assert ab.ack() == 1 and ab.update([])["active"] == [] and len(ab.history_rows()) == 3


def test_the_alarm_endpoints(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    u = f"http://127.0.0.1:{a.web_port}"
    try:
        web.alarms.update([{"style": "bad", "text": "BOAT LINK LOST: status 5 s old"}])
        st = json.loads(urllib.request.urlopen(u + "/state.json", timeout=5).read())
        assert "alarms" in st and "active" in st["alarms"]
        req = urllib.request.Request(u + "/ack", data=b"{}", method="POST", headers={"Content-Type": "application/json", "X-OCS-Web": "1"})
        assert json.loads(urllib.request.urlopen(req, timeout=5).read())["ok"]
        d = json.loads(urllib.request.urlopen(u + "/alarms.json", timeout=5).read())
        assert "history" in d and "active" in d
        cv = json.loads(urllib.request.urlopen(u + "/course.json", timeout=5).read())
        assert cv["scene"]["illustrative"] is True and len(cv["scene"]["lines"]) >= 1
    finally:
        srv.shutdown()


def test_t4_is_a_modifier():
    from optiprimex.companion.config import Config
    from optiprimex.ocs import declaration_checks as DC
    from optiprimex.ocs.ocs_app import OcsApp
    from optiprimex.tools import ocs_cli
    good = {"T1": "TIER_CORE", "T2": "TIER_NONE", "T3": "TIER_CORE", "T4": "TIER_NONE"}
    r = DC.check_tiers(good)
    assert r[0]["code"] == "mandatory_none" and r[0]["severity"] == "soft" and DC.blocking(r, "refuse") == []
    assert DC.check_tiers(good, mandatory_hard=True)[0]["severity"] == "hard"
    assert "t4_required: false" in (ROOT / "profiles" / "course" / "ocs_course.yaml").read_text()
    assert ocs_cli.intent_spec(ocs_cli.parse(["mission", "m1t1", "--t4", "off"]))[2]["t4"] == "off"
    app = OcsApp(Config())
    app.set_mission("T1", False, {"tier": "core", "t4": "off"})
    assert app.effective_tiers()["T4"] == "TIER_NONE" and app.mission["t4"] == "TIER_NONE"     # v0.1.71: the tier itself
    app.set_mission("T1", False, {"tier": "core", "t4": "on"})
    assert app.effective_tiers()["T4"] == "TIER_CORE"
    app.set_mission("T1", False, {"tier": "core"})
    assert app.mission["t4"] is None


def test_the_scene_layer_is_illustrative_and_open():
    sc = json.loads((ROOT / "profiles" / "course" / "scene_marina.json").read_text())
    assert sc["illustrative"] is True and all(len(ln) >= 2 for ln in sc["lines"])
    mp = (TOOLS / "map.html").read_text()
    assert "function poly(m, pts, col, dash, width, label, open)" in mp and "'scene · illustrative'" in mp and "if (!open) ctx.closePath();" in mp
