"""v0.1.73 (WP29 = docs/OVERNIGHT_2026-11-09_GUI.md N5): provenance tooltips everywhere (ocs/provenance.py + the page's provApply, a
DOM walk: no value element without one), the ⚙ panel over any tab (nothing operational in it), the GUI switch (classic, per browser)."""
from __future__ import annotations

import json
import os
import re
import subprocess
import threading

import pytest

from optiprimex.ocs import provenance as PV

from .conftest import ROOT, free_port

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def test_the_table_from_the_schema_and_the_hand_table():
    t = PV.table()
    schema = json.load(open(ROOT / "optiprimex" / "schemas" / "bodies.schema.json"))["$defs"]["status"]["properties"]
    assert set(t["status"]) == set(schema) and t["status"]["mode"].startswith("status/usv.mode · the mission manager")
    for k in ("state", "map age s", "cannon", "uav", "session", "judge"):
        assert " · " in t["labels"][k], k
    for w in ("hold", "resume", "abort", "skip", "declare", "stop_reset", "arm", "session"):
        assert "→" in t["words"][w] or "macro" in t["words"][w], w
    assert "ack within 5 s" in t["words"]["hold"] and "nothing is sent" in t["button_default"]
    assert PV.label_key("Map age (s)") == "map age s"


def test_the_page_markup():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    for needle in ("function provApply()", "function provProbe()", "body.gear #t-SETTINGS", 'id="camlab"', "ocs.camlab",
                   "if(n===\"SETTINGS\"){document.body.classList.add(\"gear\");return}", 'id="v1pref"', "<title>${esc(full)} · local/usv/perception/frame_dets"):
        assert needle in page, needle
    from optiprimex.tools import ocs_web as OW
    assert "provenance" in OW.meta({}, OW.Runner({}, dry_run=True))


def _serve(tmp_path):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    srv = OW.Server(("127.0.0.1", a.web_port), OW.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv, a.web_port


def _dom(url):
    r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--virtual-time-budget=6000", "--window-size=1440,900", "--enable-logging=stderr",
                        "--v=0", "--dump-dom", url], capture_output=True, text=True, timeout=90)
    return r.stdout, r.stderr


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
@pytest.mark.parametrize("tab", ["OVERVIEW", "MISSION", "CAMERA", "COMMS", "JUDGE", "LOG"])
def test_the_dom_walk_no_value_without_its_source(tmp_path, tab):
    srv, port = _serve(tmp_path)
    try:
        dom, err = _dom(f"http://127.0.0.1:{port}/?probe=1#{tab}")
    finally:
        srv.shutdown()
    assert not re.search(r'"(N2|N4|N5|sessions|v2 cards) [A-Za-z]*Error', err), err[-1500:]
    out = json.loads(re.search(r'<pre id="provprobe"[^>]*>(.*?)</pre>', dom, flags=re.S).group(1))
    assert out["missing"] == 0, out


@pytest.mark.skipif(not os.path.exists(CHROME), reason="no Chrome")
def test_the_gear_panel_is_over_the_tab_and_holds_nothing_operational(tmp_path):
    srv, port = _serve(tmp_path)
    try:
        dom, err = _dom(f"http://127.0.0.1:{port}/?gear=1&probe=1#MISSION")
    finally:
        srv.shutdown()
    assert 'class="gear"' in dom or "gear" in re.search(r"<body[^>]*>", dom).group(0)
    s0 = dom.index('id="t-SETTINGS"')
    st = dom[s0:dom.index("</section>", s0)]
    assert 'id="themesel"' in st and 'id="soundsel"' in st and 'id="kpiset"' in st and 'id="v1pref"' in st and 'id="camlab"' in st
    assert 'id="gear-sess"' in st and "SYSTEM →" in st
    out = json.loads(re.search(r'<pre id="provprobe"[^>]*>(.*?)</pre>', dom, flags=re.S).group(1))
    assert out["gear_ops"] == [], out["gear_ops"]                                  # nothing operational shown in ⚙ (SYSTEM has it)
