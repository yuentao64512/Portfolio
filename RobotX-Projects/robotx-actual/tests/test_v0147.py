"""v0.1.47 (ROADMAP WP10k-a, docs/UI_QOL_2026-11-07.md sections C-D): COMMS & DIAGNOSTICS rearranged (SYSTEM ACTIONS first, per-host
cards, two lanes, zebra node tables with liveness dots), the launched settings preselected, RUN SEQUENCE with history buttons and the
--action token pair, the picker + stage list back on OVERVIEW, the no-jump rules + the headless check, the theme followed by the SIM
page and the map, the SIM page camera released by hidden tabs (the cause: the browser's ~6 connections per host)."""
from __future__ import annotations

import json
import re

from optiprimex.tools import launcher
from optiprimex.tools import ocs_web as OW
from optiprimex.tools import nojump

from .conftest import ROOT

TOOLS = ROOT / "optiprimex" / "tools"
THEMES = ("dark", "light", "hc-dark", "hc-light", "fun")


def page():
    return (TOOLS / "ocs_web.html").read_text()


def _vars(theme):
    css = (TOOLS / "themes.css").read_text()
    out = {}
    sel = ':root,:root[data-theme="dark"]' if theme == "dark" else f':root[data-theme="{theme}"]'
    for m in re.finditer(r"([^{}]+)\{([^}]*)\}", css):
        heads = [h.strip() for h in m.group(1).split("*/")[-1].split(",")]
        if f':root[data-theme="{theme}"]' in heads or (theme == "dark" and m.group(1).strip().endswith(sel)):
            for k, v in re.findall(r"(--[\w-]+):([^;]+)", m.group(2)):
                out[k] = v.strip()
    return out


def test_comms_order_host_cards_lanes_and_zebra_tables():
    p = page()
    i = p.index('<section class="tab" id="t-COMMS">')
    c = p[i:p.index("</section>", i)]
    assert c.index("SYSTEM ACTIONS") < c.index('id="sys"') < c.index("STREAMS") < c.index("HOST")
    assert 'class="hostcards" id="sys"' in c and 'class="grid lanes"' in c
    assert 'const HG=[["nx","Orin NX",["nx","nx_reboot"]],["agx","AGX Orin",["agx","agx_reboot"]],["mac","this Mac",["ocs","rctest"]],["fcu","the FCU",["fcu"]]]' in p
    assert "table.zt" in p and "nth-child(even)" in p and "<colgroup>" in p
    assert 'class="selw"' in p                                                       # select + (changed) = one grid cell: the rows line up


def test_liveness_dots_flash_and_go_alarm_past_the_limit():
    p = page()
    assert "function liveDot(key,age,limit)" in p
    assert ".ldot.lit" in p and ".ldot.dead" in p and "var(--alarm" in p[p.index(".ldot.dead"):p.index(".ldot.dead") + 80]


def test_the_launched_settings_are_the_defaults(tmp_path, monkeypatch):
    rec = tmp_path / "last.json"
    monkeypatch.setattr(launcher, "RECORD", str(rec))
    OW._LAUNCH_CACHE.update(mtime=None, body=None)
    assert OW.launch_defaults() is None                                              # nothing launched: the cards keep their defaults
    rec.write_text(json.dumps({"when": "2026-11-07 09:12", "cfg": {"preset": "sim", "judge": "fake", "net": "ts", "log": True, "fcu": "fake",
                                                                  "usv_profile": "pool", "cv": "fake", "world": "pool"},
                               "plan": {"stamp": "20261107-0912"}}))
    d = OW.launch_defaults()
    assert d["preset"] == "sim" and d["when"] == "2026-11-07 09:12" and d["stamp"] == "20261107-0912"
    assert d["options"]["ocs"] == {"judge": "fake", "net": "ts", "log": "on"}
    assert d["options"]["nx"] == {"fcu": "fake", "profile": "pool", "net": "ts", "log": "on"}
    assert d["options"]["agx"] == {"mode": "sim", "world": "pool", "net": "ts"}
    p = page()
    assert "function applyLaunch()" in p and "(changed)" in p and "launched " in p


def test_run_sequence_history_buttons_and_the_action_pair():
    p = page()
    assert "RUN SEQUENCE" in p and "RUN · ONE STEP, ONE CONTROL" not in p
    assert "DONE_T" in p and "✓ " in p
    assert "button.b.act{background:var(--action);border:2px solid var(--actionLine);color:var(--actionText);font-weight:800;min-height:36px" in p
    want = {"dark": ("#7a5a00", "#ff5a5a"), "light": ("#ffd60a", "#c20d16"), "hc-dark": ("#8a6600", "#ff3b3b"),
            "hc-light": ("#ffd60a", "#a80000"), "fun": ("#187a42", "#ff8c42")}                # Dmitrii: darker amber fill, red outline on dark
    css = (TOOLS / "themes.css").read_text()
    for t, (fill, line) in want.items():
        assert re.search(r'\[data-theme="%s"\]\{--action:%s;--actionLine:%s;' % (re.escape(t), fill, line), css), t
    fun = _vars("fun")
    assert fun["--btnBg"] != fun["--chipBg"]                                         # the fun theme's buttons are not chips
    assert "background:var(--btnBg,var(--panel2))" in p


def test_the_picker_and_the_stage_list_back_on_overview():
    p = page()
    i = p.index('<section class="tab" id="t-OVERVIEW">')
    ov = p[i:p.index("</section>", i)]
    assert 'id="ov-sel"' in ov and 'id="ov-stages"' in ov
    assert "function drawOvStages" in p


def test_no_jump_rules_and_the_check():
    p = page()
    assert "tabular-nums" in p and "white-space:nowrap" in p
    assert "jumpcheck" in p
    r = nojump.parse('<pre id="jumpcheck">x</pre> ... <pre id="jumpcheck">{"OVERVIEW": {"changed": []}}</pre>')
    assert r == {"OVERVIEW": {"changed": []}}


def test_every_page_follows_the_shared_theme():
    sim = (ROOT / "optiprimex" / "sim" / "sim.html").read_text()
    assert '<link rel="stylesheet" href="/themes.css">' in sim and 'id="themebar"' in sim and 'post("/prefs",{theme:sel.value})' in sim
    assert 'fetch("/prefs.json"' in sim
    mp = (TOOLS / "map.html").read_text()
    assert "fetch('/prefs'" in mp and "mapColours()" in mp[mp.index("async function followPrefs"):mp.index("followPrefs();")]
    p = page()
    assert "function followPrefs" in p and "function postTheme" in p


def test_the_sim_page_and_the_ocs_page_release_the_camera_when_hidden():
    sim = (ROOT / "optiprimex" / "sim" / "sim.html").read_text()
    assert '"/snapshot.jpg?t="' in sim and 'src="/stream"' not in sim            # v0.1.50: a snapshot loop, no held stream
    p = page()
    assert 'document.addEventListener("visibilitychange",()=>camApply())' in p and "!document.hidden" in p
    assert '["MISSION","SETTINGS","OVERVIEW"]' in p                                            # camPoll on the tab the camera lives on since v0.1.44


def test_the_sim_page_buttons_have_titles():
    sim = (ROOT / "optiprimex" / "sim" / "sim.html").read_text()
    assert [b for b in re.findall(r"<button\b[^>]*>", sim) if "title=" not in b] == []


def test_the_action_and_button_colours_pass_the_contrast_rule():
    from .test_v0143 import _cr
    css = (TOOLS / "themes.css").read_text()
    for t in THEMES:
        body = ";".join(b for n, b in re.findall(r'\[data-theme="([^"]+)"\]\{([^}]*)\}', css) if n == t)
        v = dict(re.findall(r"--(\w+):(#[0-9a-fA-F]{6})", body))
        assert _cr(v["actionText"], v["action"]) >= 4.5, t                          # the word on the live control
        assert _cr(v["actionLine"], v["panel"]) >= 3.0, t                           # its outline against the card
        assert _cr(v["text"], v["btnBg"]) >= 4.5, t


def test_one_live_control_only():
    p = page()
    assert "function oneAction(){" in p and ".slice(1).forEach(b=>b.classList.remove(\"act\"))" in p
    assert "drawLadder();oneAction();" in p
