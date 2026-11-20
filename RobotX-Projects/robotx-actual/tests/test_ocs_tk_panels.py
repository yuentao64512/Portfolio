"""The console's Phase-2 tabs (2026-11-12 night): JUDGE · BIG VIEW · SYSTEM · HELP - the pure views on the recorded fixtures, then the
window: the tabs exist, BIG VIEW toggles, a SYSTEM action asks first and a dangerous one needs its word typed before POST /system."""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from optiprimex.tools.tkconsole import panels as P

FIX = Path(__file__).parent / "fixtures"


def load(n):
    return json.loads((FIX / f"state_{n}.json").read_text())


def test_judge_rows_say_what_robocommand_sees():
    t = load("r5_task")
    r = P.judge_rows(t["state"])
    assert dict((k, (v, l)) for k, v, l in r["link"])["JUDGE"] == ("fake · link up", "ok")
    run = dict((k, (v, l)) for k, v, l in r["run"])
    assert run["RUNSTART"][0].startswith("run 1001 · for our declaration 1 · accepted") and run["RUNSTART"][1] == "ok"
    assert run["TIMELINE"][0] == "declared → held → started"
    assert r["vehicles"] == [("USV1", "sending AUTO · #160 · last 0.2 s · the judge saw it 0.5 s ago", "ok")]
    assert P.judge_rows(None)["run"][0][0] == "STATE" and P.judge_log(t["state"])[-1][1].startswith("judge <- safe_passage")


def test_the_big_view_tiles():
    t = load("r5_task")
    b = {x.key: x for x in P.run_view(t["state"], t["fast"])}
    assert b["state"].value == "RUNNING" and b["state"].level == "ok" and b["stage"].value in ("LOG", "APPROACH")
    assert b["time"].value == "1:07" and b["hdg"].value == "343°" and b["guard"].value == "CLEAR" and b["rec"].value == "○ OFF"
    assert all(x.value for x in P.run_view(None))                          # nothing at all: dashes, no crash


def test_system_actions_carry_refusals_and_the_typed_word():
    t, meta = load("r5_task"), load("meta")
    acts = {a.id: a for a in P.system_actions(meta, t["state"])}
    assert len(acts) == 19 and acts["ocs"].refused.startswith("a run is declared") and acts["stopall"].confirm == "stop"
    assert P.system_body(acts["ocs"], {"do": "restart", "judge": "", "net": "lan"}) == \
        {"action": "ocs", "options": {"do": "restart", "net": "lan"}, "confirm": ""}
    assert [s for s, _ in P.help_sections(meta)][1] == "RUN STATES (R, the OCS)"


tk = pytest.importorskip("tkinter")
from .test_ocs_tk_smoke import shared_root, spin  # noqa: E402
from .test_ocs_tk_net import FakeOcs  # noqa: E402


@pytest.fixture()
def root():
    top = tk.Toplevel(shared_root())
    top.withdraw()
    top.quit, top.mainloop = shared_root().quit, shared_root().mainloop
    yield top
    try:
        top.destroy()
    except tk.TclError:
        pass


def test_the_tabs_the_big_view_and_a_system_action(root):
    from optiprimex.tools.tkconsole.views import App
    f = FakeOcs("r2")
    app = App(root, f.url, fast_hz=5)
    try:
        assert spin(root, lambda: app.view is not None and app.view.link == "UP" and app.model.meta)
        names = [app.nb.tab(t, "text") for t in app.nb.tabs()]
        assert names == ["LOG", "MAP", "VEHICLES", "JUDGE", "HOSTS", "CMD", "MISSION", "CAMERA"]      # SYSTEM / HELP: the ⚙ window
        app.nb.select(app.tabs["JUDGE"])
        assert spin(root, lambda: "JUDGE" in app.judge_boxes["link"]["rows"])
        app.buttons["big"].invoke()
        assert app.big_on and app.big_fr.winfo_manager() == "pack" and not app.body_pw.winfo_manager()
        assert spin(root, lambda: app.big_tiles["state"]["val"].cget("text") == "ARMED")
        app._big(False)
        assert app.body_pw.winfo_manager() == "pack"
        app._settings(True)
        app.set_nb.select(app.set_tabs["SYSTEM"])
        assert spin(root, lambda: "sys:stopall" in app.buttons and "sys:cam" in app.buttons)
        app.buttons["sys:cam"].invoke()                                     # a plain one: yes / no
        assert len(app.dialogs) == 1 and not [p for p, _ in f.posts if p == "/system"]
        app.dialogs[-1].yes.invoke()
        assert spin(root, lambda: any(p == "/system" for p, _ in f.posts))
        assert next(b for p, b in f.posts if p == "/system")["action"] == "cam"
        app.buttons["sys:stopall"].invoke()                                 # a dangerous one: its word typed
        d = app.dialogs[-1]
        d.yes.invoke()                                                      # nothing typed -> nothing sent
        assert sum(1 for p, _ in f.posts if p == "/system") == 1 and d.w.winfo_exists()
        d.entry.insert(0, "stop")
        d.yes.invoke()
        assert spin(root, lambda: sum(1 for p, _ in f.posts if p == "/system") == 2)
        assert [b for p, b in f.posts if p == "/system"][-1] == {"action": "stopall", "options": {}, "confirm": "stop"}
    finally:
        app.close()
        f.close()


@pytest.fixture()
def cfgdir(tmp_path, monkeypatch):
    monkeypatch.setattr(P, "CONFIG_DIR", str(tmp_path / "tk_configs"))
    monkeypatch.setattr(P, "PREFS_FILE", str(tmp_path / "tk_prefs.json"))
    return tmp_path


def test_configs_save_list_restore_plan_delete(cfgdir):
    """2026-11-12 (Dmitrii: "a settings window, with configs which we could save and restore, as we had in previous gui")."""
    s = load("r5_task")["state"]
    ks = P.knobs(s)
    assert len(ks) == 11 and ks[0].key == "line_v"
    assert P.tune_word(ks[0], "0.7") == ("tune line_v 0.7", "") and P.tune_word(ks[0], "9")[1].startswith("line_v: 9 is outside")
    assert P.tune_word(ks[0], "abc")[1] == "line_v: 'abc' is not a number"
    prefs = P.load_prefs()
    assert prefs["theme"] == "light" and prefs["start_tab"] == "LOG"
    path = P.save_config(P.make_config("night_1", s, dict(prefs, log_filter="judge"), "0.1.85"))
    assert path.endswith("tk_configs/night_1.json")
    with pytest.raises(ValueError):
        P.save_config(P.make_config("bad name!", s, prefs))
    lst = P.list_configs()
    assert [c["name"] for c in lst] == ["night_1"] and lst[0]["n"] == 11 and lst[0]["cfg"]["prefs"]["log_filter"] == "judge"
    s2 = json.loads(json.dumps(s))
    s2["local"]["tuning"]["knobs"][0]["value"] = 0.8                       # the live line_v moved
    s2["local"]["tuning"]["knobs"] = s2["local"]["tuning"]["knobs"][:-1]   # and one knob is gone
    words, notes = P.restore_plan(lst[0]["cfg"], s2)
    assert words == ["tune line_v 0.5"] and len(notes) == 1 and "not on this boat now" in notes[0]
    assert P.restore_plan(lst[0]["cfg"], s) == ([], [])                    # all equal: nothing to send
    P.save_prefs(dict(prefs, theme="dark", fast_hz="10"))
    assert P.load_prefs()["theme"] == "dark" and P.load_prefs()["fast_hz"] == "10"
    assert P.delete_config("night_1") and P.list_configs() == []


def test_the_settings_window_saves_restores_and_tunes(root, cfgdir):
    from optiprimex.tools.tkconsole.views import App
    f = FakeOcs("r2")
    app = App(root, f.url, fast_hz=5)
    try:
        assert spin(root, lambda: app.view is not None and app.view.link == "UP" and app.model.meta)
        assert [app.nb.tab(t, "text") for t in app.nb.tabs()] == ["LOG", "MAP", "VEHICLES", "JUDGE", "HOSTS", "CMD", "MISSION", "CAMERA"]
        assert [app.set_nb.tab(t, "text") for t in app.set_nb.tabs()] == ["SETTINGS", "SYSTEM", "HELP"]
        app.buttons["settings"].invoke()
        assert app.set_on and spin(root, lambda: "transit_v" in app.tune_rows)
        assert "line_v" not in app.tune_rows                                # DOCK (T3) collapsed: not in tomorrow's presets
        hdr = app.tune_rows[("hdr", "DOCK (T3)")]["hdr"]
        hdr.event_generate("<Button-1>")
        assert spin(root, lambda: "line_v" in app.tune_rows)                # a click on its header opens it
        groups = [n for kind, n in app._tune_layout if kind == "group"]
        assert groups[0] == "SPEEDS" and groups[-1] == "DOCK (T3)"
        app.tune_rows["transit_v"]["ent"].insert(0, "0.7")
        app.buttons["tune:transit_v"].invoke()
        assert spin(root, lambda: ("/cmd", {"line": "tune transit_v 0.7"}) in f.posts)
        app.tune_rows["transit_v"]["ent"].insert(0, "9")                    # out of range: refused here, nothing sent
        app.buttons["tune:transit_v"].invoke()
        assert not any(b.get("line") == "tune transit_v 9" for _p, b in f.posts)
        app.cfg_name.insert(0, "dock_day")
        app.buttons["cfg-save"].invoke()
        assert [c["name"] for c in P.list_configs()] == ["dock_day"]
        assert spin(root, lambda: app.cfg_list.size() == 1)
        cfg = P.list_configs()[0]["cfg"]
        cfg["knobs"]["transit_v"] = 0.3                                     # the saved one differs from the live 0.6
        cfg["prefs"]["log_filter"] = "judge"
        P.save_config(cfg)
        app.cfg_list.selection_set(0)
        app.buttons["cfg-restore"].invoke()
        app.dialogs[-1].yes.invoke()
        assert spin(root, lambda: ("/cmd", {"line": "tune transit_v 0.3"}) in f.posts)
        assert app.prefs["log_filter"] == "judge" and app.log_filter.get() == "judge" and P.load_prefs()["log_filter"] == "judge"
        app.set_nb.select(app.set_tabs["SYSTEM"])
        assert spin(root, lambda: "sys:stopall" in app.buttons)
        app.set_nb.select(app.set_tabs["HELP"])
        assert spin(root, lambda: "RUN STATES" in app.help_text.get("1.0", "end"))
        app._settings(False)
        assert not app.set_on
    finally:
        app.close()
        f.close()


def test_the_knob_groups_follow_tomorrows_relevance():
    s = load("r5_task")["state"]
    g = dict((n, [k.key for k in kk]) for n, kk in P.grouped_knobs(s))
    assert list(g)[0] == "SPEEDS" and g["SPEEDS"] == ["transit_v", "circle_v"] and g["DOCK (T3)"][0] == "line_v"
    s2 = json.loads(json.dumps(s))                                         # a knob the boat adds later lands in its group
    s2["local"]["tuning"]["knobs"].append({"key": "yaw_rate_max_dps", "sections": ["guidance"], "value": 5, "default": 5, "lo": 2, "hi": 10})
    s2["local"]["tuning"]["knobs"].append({"key": "mystery_k", "sections": ["x"], "value": 1, "default": 1, "lo": 0, "hi": 2})
    g2 = dict((n, [k.key for k in kk]) for n, kk in P.grouped_knobs(s2))
    assert g2["TURNING"] == ["scan_rate_dps", "yaw_rate_max_dps"] and g2["OTHER"] == ["mystery_k"]


def test_the_guard_pair_rule_is_said_before_sending():
    live = {"guard_stop_m": 1.5, "guard_slow_m": 3.0}
    assert P.pair_rule("guard_slow_m", 1.5, live).startswith("guard_slow_m 1.5 must stay above")
    assert P.pair_rule("guard_stop_m", 3.0, live).startswith("guard_stop_m 3 must stay below")
    assert P.pair_rule("guard_slow_m", 4.0, live) == "" and P.pair_rule("v_min_mps", 0.1, live) == ""
