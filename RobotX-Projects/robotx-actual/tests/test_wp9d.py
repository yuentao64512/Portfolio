"""ROADMAP WP9d (v0.1.42), the light cut (polled 2026-11-06): the launcher's presets sim / lab / event / custom, LAUNCH_PRESET to the
consoles, the flavour line (preset · judge · net · FCU · profile · CV) on the OCS page, the judge page and usv_ui, the ten-words card,
the refusal sweep (every refusal says what to do; MAIN's next button says when it is blocked)."""
from __future__ import annotations

import collections
import importlib.util
import os
import socket

from optiprimex.tools import launcher as L

from .conftest import ROOT

NET = {"ORIN_SSH": "nx-alias", "AGX_SSH": "agx-alias"}


def test_the_presets_are_sim_lab_event_custom():
    P = L.load_presets()
    assert list(P) == ["sim", "lab", "event", "custom"]
    e = P["event"]
    assert e["title"].startswith("event") and (e["mode"], e["judge"], e["net"], e["fcu"], e["usv_profile"], e["cv"]) == \
        ("real", "rctest", "lan", "uart", "course", "real")
    p = L.build_plan(dict(P["event"], preset="event"), stamp="S", net_env=NET)
    assert "LAUNCH_PRESET=event scripts/run_usv.sh --fcu uart --course --lan" in [s for s in p["steps"] if s["host"] == "nx"][0]["cmd"]
    p = L.build_plan(dict(P["sim"], preset="sim"), stamp="S", net_env=NET)
    assert p["steps"][0]["cmd"].startswith("LAUNCH_PRESET=sim scripts/run_sim.sh")
    assert 'LAUNCH_PRESET=${LAUNCH_PRESET:-} $PY usv_ui.py' in (ROOT / "scripts" / "run_usv.sh").read_text()
    assert 'LAUNCH_PRESET=${LAUNCH_PRESET:-} $PY usv_ui.py' in (ROOT / "scripts" / "run_sim.sh").read_text()
    assert 'P.sim?"sim"' in (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()


def test_the_flavour_line():
    rec = {"cfg": dict(L.load_presets()["event"], preset="event")}
    f = L.flavour(rec=rec)
    assert f["text"] == "event · judge rctest · net lan · FCU uart · course · CV real"
    f = L.flavour("course", "ts", "fake", rec=rec)                      # what the console knows live wins
    assert f["text"] == "event · judge course · net ts · FCU fake · course · CV real"
    f = L.flavour(rec={"cfg": dict(L.load_presets()["sim"], preset="sim")})
    assert f["text"] == "sim · judge fake · net local · FCU sim · bench · CV sim camera"
    assert L.flavour("fake", None, None, rec=None)["text"] == "? · judge fake · net ? · FCU ? · ? · CV ?"   # started by hand


def test_the_consoles_show_it():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'id="flavour"' in page and "S.flavour.text" in page
    src = (ROOT / "optiprimex" / "tools" / "ocs_web.py").read_text()
    assert 'snap["flavour"] = flavour(' in src
    from optiprimex.tools import judge_web
    assert 'id="flav"' in judge_web.PAGE and "d.flavour.text" in judge_web.PAGE
    assert judge_web._flavour(None) is None or "text" in judge_web._flavour(None)


def test_usv_ui_names_the_preset_at_200x60(monkeypatch):
    spec = importlib.util.spec_from_file_location("usv_ui_wp9d", ROOT / "usv_ui" / "usv_ui.py")
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    for k in ("HLINE", "VLINE", "ULCORNER", "URCORNER", "LLCORNER", "LRCORNER", "LTEE", "RTEE", "TTEE", "BTEE", "PLUS", "BLOCK",
              "CKBOARD", "BULLET", "DIAMOND", "DEGREE"):
        if not hasattr(m.curses, "ACS_" + k):
            monkeypatch.setattr(m.curses, "ACS_" + k, ord("+"), raising=False)

    class Scr:                                           # a curses screen stand-in: draw() at 200 x 60 into a buffer
        def __init__(s, h, w):
            s.h, s.w = h, w
            s.erase()

        def erase(s):
            s.buf = [[" "] * s.w for _ in range(s.h)]

        def getmaxyx(s):
            return s.h, s.w

        def addnstr(s, y, x, t, n, a=0):
            for i, ch in enumerate(t[:n]):
                if x + i < s.w:
                    s.buf[y][x + i] = ch

        def __getattr__(s, k):
            return lambda *a, **kw: None

    so = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    so.bind(("127.0.0.1", 0))
    port = so.getsockname()[1]
    so.close()
    app = m.App(m.build_parser().parse_args([f"udpin:127.0.0.1:{port}"]))
    try:
        for preset, want in (("event", "[event]"), (None, "[?]")):
            if preset:
                monkeypatch.setenv("LAUNCH_PRESET", preset)
            else:
                monkeypatch.delenv("LAUNCH_PRESET", raising=False)
            app.scr, app.screen, app.log_scroll, app.unicode, app.C = Scr(60, 200), 0, 0, False, collections.defaultdict(int)
            app.draw()
            top = "".join(app.scr.buf[0])
            assert top.startswith(" usv_ui v") and want in top, top
    finally:
        try:
            app.close_links()
        except Exception:
            pass


def test_the_ten_words_and_the_refusal_sweep():
    page = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert 'id="h-ten"' in page and '"ready","start_task","hold","resume","abort","manual","end_run","reset","save_point","home"' in page
    assert '" · blocked"' in page                                       # MAIN's next button says so (seen with WP9c's refusal)
    from optiprimex.tools import bench_tui as T
    assert {"ready", "start_task", "hold", "resume", "abort", "manual", "end_run", "reset", "save_point", "home"} <= {w for w, _, _ in T.USAGE}
    from .test_ocs_units import intent, make_authority
    bus, lc, auth, results = make_authority(bench=False, lc_state="COURSE_VALID")
    for action, needle in (("ARM", "the transmitter"), ("FLY", "HELP tab"), ("INJECT", "bench only")):
        r = auth.on_intent("t", intent(action))
        assert r.result == "refused" and needle in r.reason, (action, r.reason)
