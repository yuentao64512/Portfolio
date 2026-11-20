"""v0.1.50 (Dmitrii 2026-11-06): the FCU's mode and ARM / DISARM from the pages (ADR-9 / decision 25 lifted for these: the boat
window through usv_ui whoever holds the FCU, the OCS page through the mission manager), RC6 latched = last wins (a USV-side policy,
`rc6_policy hold` = the old rule), the camera by snapshot loop (no held MJPEG stream), the button design (--actionPale / --alarmPale)."""
from __future__ import annotations

import re
import time

from optiprimex.companion.config import Config
from optiprimex.tools import ocs_cli
from optiprimex.usv.vehicle_fsm import VehicleFsm

from .conftest import ROOT, wait_until
from .test_v0145 import _pub, _result, stack  # noqa: F401  (the live chain: fake FCU, usv_ui, the mission manager, boat_web)

TOOLS = ROOT / "optiprimex" / "tools"


def test_rc6_latched_last_wins_and_the_hold_policy():
    f = VehicleFsm(Config())
    assert f.rc6_policy == "latched"
    assert f._rc6_effective(False, "HOLD") is False
    assert f._rc6_effective(True, "HOLD") is True                   # the flip up: MANUAL at once
    assert f._rc6_effective(True, "HOLD") is True                   # held: still holds
    f.inputs.mode_name = "HOLD"
    assert f.clear_rc6_latch() is True and f._rc6_effective(True, "HOLD") is False   # a later page command wins
    assert f._rc6_effective(False, "HOLD") is False and f._rc6_effective(True, "HOLD") is True   # the next flip takes it back
    assert f._rc6_effective(True, "GUIDED") is False                # a mode commanded after the flip (a page, a GCS) wins too
    f.set_rc6_policy("hold")
    assert f._rc6_effective(True, "HOLD") is True and f.clear_rc6_latch() is False and f._rc6_effective(True, "HOLD") is True
    assert f._rc6_effective(False, "HOLD") is False


def test_the_words_and_the_authority():
    assert ocs_cli.intent_spec(ocs_cli.parse(["fcu_mode", "hold"]))[:3] == ("FCU_MODE", None, {"mode": "HOLD"})
    assert ocs_cli.intent_spec(ocs_cli.parse(["arm"]))[:3] == ("FCU_ARM", None, {"on": True})
    assert ocs_cli.intent_spec(ocs_cli.parse(["disarm"]))[:3] == ("FCU_ARM", None, {"on": False})
    assert ocs_cli.intent_spec(ocs_cli.parse(["rc6_policy", "hold"]))[:3] == ("RC6", None, {"policy": "hold"})
    from optiprimex.ocs.cmd_authority import GUARDS
    assert GUARDS["FCU_MODE"][2] == "FCU_MODE" and GUARDS["FCU_ARM"][2] == "FCU_ARM" and GUARDS["RC6"][2] == "SET_RC6"
    from optiprimex.usv.run_context import RUN_FREE_ACTIONS
    assert {"FCU_MODE", "FCU_ARM", "SET_RC6"} <= set(RUN_FREE_ACTIONS)
    assert Config().guards.rc6_policy == "latched"


def test_the_page_words_mode_and_arm_run_while_handed_over(stack):  # noqa: F811
    ui, boat, port, fcu = stack["ui"], stack["boat"], stack["brokers"].boat_port, stack["fcu"]
    app = ui.app
    time.sleep(0.6)
    _pub(port, "handover", id="h0")
    assert _result(boat, "h0") and wait_until(lambda: not app.active, 5)
    time.sleep(0.6)
    _pub(port, "mode", args=["hold"], id="m0")
    r = _result(boat, "m0")
    assert r and r["ok"], r
    assert wait_until(lambda: (fcu.ctrl({"get": "state"}) or {}).get("mode") == 4, 5) and not app.active   # no takeover
    time.sleep(0.6)
    _pub(port, "disarm", id="d0")
    assert _result(boat, "d0") and wait_until(lambda: (fcu.ctrl({"get": "state"}) or {}).get("armed") is False, 5) and not app.active
    time.sleep(0.6)
    _pub(port, "arm", id="a0")
    assert _result(boat, "a0") and wait_until(lambda: (fcu.ctrl({"get": "state"}) or {}).get("armed") is True, 5)


def test_fcu_mode_through_the_mission_manager_clears_a_latched_rc6(stack):  # noqa: F811
    vc, fcu, hub = stack["vc"], stack["fcu"], stack["hub"]

    def last_status():
        m = hub.last("team/optiprime/v1/status/usv")
        return (m or {}).get("env", {}).get("body") or {}

    fcu.ctrl({"set": {"rc6": 1900}})
    assert wait_until(lambda: (last_status().get("rc6") or {}).get("holds") is True, 5), last_status().get("rc6")
    acks = vc.send("run", "FCU_MODE", args={"mode": "HOLD"}, wait_s=5)
    assert acks and acks[-1]["body"]["result"] in ("ACCEPTED", "DONE") and "RC6 latch cleared" in acks[0]["body"].get("reason", ""), acks
    assert wait_until(lambda: (last_status().get("rc6") or {}).get("holds") is False, 5)              # last wins
    acks = vc.send("run", "SET_RC6", args={"policy": "hold"}, wait_s=5)
    assert acks and acks[-1]["body"]["result"] in ("ACCEPTED", "DONE"), acks
    assert wait_until(lambda: (last_status().get("rc6") or {}) == {"policy": "hold", "raw": True, "holds": True}, 5)
    acks = vc.send("run", "FCU_MODE", args={"mode": "HOLD"}, wait_s=5)
    assert "RC6 latch cleared" not in acks[0]["body"].get("reason", "")                                # hold: the switch rules
    assert (last_status().get("rc6") or {}).get("holds") is True
    fcu.ctrl({"set": {"rc6": 1000}})
    vc.send("run", "SET_RC6", args={"policy": "latched"}, wait_s=5)
    acks = vc.send("run", "FCU_MODE", args={"mode": "NOPE"}, wait_s=5)
    assert acks[-1]["body"]["result"] == "REJECTED" and "unknown mode" in acks[-1]["body"]["reason"]


def test_the_pages_have_the_buttons():
    p = (TOOLS / "ocs_web.html").read_text()
    for i in ("fb-manual", "fb-hold", "fb-arm", "fb-disarm", "rc6w"):
        assert f'id="{i}"' in p, i
    assert 'send("fcu_mode MANUAL")' in p and 'send("fcu_mode HOLD")' in p and "ARM the FCU?" in p
    b = (TOOLS / "boat_web.html").read_text()
    assert 'id="fcu-modes"' in b and 'id="fcu-arm"' in b and 'id="rc6-pol"' in b and "confirm ARM?" in b
    from optiprimex.tools import boat_web as BW
    assert {"mode", "arm", "disarm"} <= set(BW.WHITELIST) and "burst" not in BW.WHITELIST and len(BW.FCU_MODES) >= 8
    assert BW.ocs_line("http://127.0.0.1:1", "arm")["text"].startswith("refused by the page")      # only the RC6 rule travels


def test_the_camera_is_a_snapshot_loop_and_the_button_tokens():
    p = (TOOLS / "ocs_web.html").read_text()
    assert 'stream?t=' not in p and 'snapshot.jpg?t="+Date.now()' in p and "function camLoop(on)" in p
    sim = (ROOT / "optiprimex" / "sim" / "sim.html").read_text()
    assert 'src="/stream"' not in sim and '"/snapshot.jpg?t="+Date.now()' in sim
    css = (TOOLS / "themes.css").read_text()
    for t in ("dark", "light", "hc-dark", "hc-light", "fun"):
        assert re.search(r'\[data-theme="%s"\]\{--actionPale:#[0-9a-f]{6};--alarmPale:#[0-9a-f]{6}\}' % re.escape(t), css), t
    assert "button.b.go,button.b.primary,button.b.big{background:var(--actionPale)" in p


def test_the_pale_buttons_pass_the_contrast_rule():
    from .test_v0143 import _cr
    css = (TOOLS / "themes.css").read_text()
    for t in ("dark", "light", "hc-dark", "hc-light", "fun"):
        body = ";".join(b for n, b in re.findall(r'\[data-theme="([^"]+)"\]\{([^}]*)\}', css) if n == t)
        v = dict(re.findall(r"--(\w+):(#[0-9a-fA-F]{6})", body))
        assert _cr(v["text"], v["actionPale"]) >= 4.5 and _cr(v["alarmText"], v["alarmPale"]) >= 4.5, t
        assert _cr(v["actionLine"], v["actionPale"]) >= 3.0, t                      # the outline reads against its fill
