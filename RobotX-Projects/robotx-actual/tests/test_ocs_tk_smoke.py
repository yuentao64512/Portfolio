"""The tk console's window against the FakeOcs (the recorded fixtures): it renders, every button exists and goes through the one
press -> send path, one click = one POST /cmd and one toast, the pressed button changes colour, ARM asks first, ACK posts /ack, DOWN
and recovery, and closing leaves no thread. Skips without tkinter / a display (CI); runs on the Mac.

Driven by the real mainloop() with an after() checker - the program's own loop. (Tk 8.6.18 on macOS: a test loop of root.update()
calls can stall inside update() with this window; mainloop() does not - measured 2026-11-11, 0.25 s heartbeats, two roots in a row.)"""
from __future__ import annotations

import time

import pytest

tk = pytest.importorskip("tkinter")

from .test_ocs_tk_net import FakeOcs  # noqa: E402


def shared_root():
    """ONE Tk for the whole test process, never destroyed - as the program has one (creating and destroying a Tk per test crashed
    macOS Tk 8.6.18 ~1 in 8 runs). test_ocs_tk_map.py uses it too."""
    r = getattr(tk, "_default_root", None)
    if r is not None:
        try:
            r.winfo_exists()
            return r
        except tk.TclError:
            pass
    try:
        r = tk.Tk()
    except tk.TclError as exc:
        pytest.skip(f"no display: {exc}")
    r.withdraw()
    return r


@pytest.fixture(scope="module")
def tkroot():
    return shared_root()


@pytest.fixture()
def root(tkroot):
    """Each App gets its own Toplevel (App.close destroys it); spin() runs the one mainloop."""
    top = tk.Toplevel(tkroot)
    top.withdraw()
    top.quit = tkroot.quit
    top.mainloop = tkroot.mainloop
    yield top
    try:
        top.destroy()
    except tk.TclError:
        pass


def spin(root, pred, s=5.0) -> bool:
    """Run the mainloop until pred() is true or s seconds passed."""
    end, out = time.time() + s, [False]

    def check():
        try:
            ok = bool(pred())
        except Exception:
            ok = False
        if ok or time.time() > end:
            out[0] = ok
            root.quit()
        else:
            root.after(40, check)
    root.after(40, check)
    root.mainloop()
    return out[0]


@pytest.fixture()
def app5(root):
    from optiprimex.tools.tkconsole.views import App
    f = FakeOcs("r5_task")
    app = App(root, f.url, fast_hz=5)
    assert spin(root, lambda: app.view is not None and app.view.link == "UP" and app.model.meta and app.view.steps[5].state == "now")   # v0.1.89: 6 = the boat's mission
    yield app, f
    app.close()
    f.close()
    assert app.threads_left() == []                                                   # closing stops the pollers


def test_the_window_renders_the_recorded_run(root, app5):
    app, f = app5
    assert root.title().startswith("OCS · ocs_bench · v")
    assert app.chips["fcu"].cget("text") == "mm · HOLD · armed" and app.chips["v"].cget("text") == "R5 · AUTO_TASK"
    assert app.chips["guard"].cget("text") == "GUARD clear" and app.chips["link"].cget("text") == "UP"
    assert app.word.cget("text").startswith("RUNNING · SCOUT")
    assert app.steps[5]["num"].cget("text") == "✓" and app.steps[1]["num"].cget("text") == "✓" and app.steps[6]["chip"].cget("text") != "—"
    assert app.view.profile == "USV only" and app.lad_profile.cget("text") == "MISSION 1 · USV only"   # v0.1.89: the mission-1 ladder
    assert str(app.buttons["lad6-0"].winfo_manager()) == "pack" and app.btn_model["lad6-0"].word == "hold"
    assert app.btn_model["lad6-1"].word == "abort" and not app.buttons["lad6-2"].winfo_manager()
    assert app.view.banner and app.view.banner["title"].startswith("RUN STARTED") and app.view.banner["stage"].startswith("USV · SCOUT")
    assert app._ban_corner["fr"].winfo_manager() == "place"                       # the corner pop-up is up
    assert app.usv_kv["task"].cget("text").startswith("SCOUT · ")
    assert "scoutsim" in app._mission_codes and app.mpick.get().startswith("scoutsim")
    assert int(app.log.index("end-1c").split(".")[0]) > 20                            # the LOG tab holds the state's lines
    n = len(app._cache)
    app.render()
    assert len(app._cache) == n                                                       # by diff: nothing new to write


def test_every_button_is_bound_to_the_one_press(root, app5):
    app, f = app5
    for k, b in app.buttons.items():
        assert "press" in str(b.cget("command")) or b.cget("command"), k
    keys = set(app.buttons)
    assert {"lad1-0", "lad4-0", "lad8-3", "usv-hb", "usv-auto",
            "usv-sk-hold", "usv-sk-loiter", "usv-start", "usv-hold", "usv-resume", "usv-abort", "fcu-manual", "fcu-hold", "fcu-arm",
            "fcu-disarm", "stop-reset", "uav-decl", "uav-scan", "usv-mission", "ack-all", "cmd-send", "mis-arm",
            "mis-start", "cam-rec"} <= keys


def test_one_click_one_cmd_one_toast_and_the_button_colour(root, app5):
    app, f = app5
    app.buttons["usv-hold"].invoke()                                                  # ❚❚ HOLD on the USV tile
    assert app.presses.look("usv-hold") == "sending" and "Bsending" in str(app.buttons["usv-hold"].cget("style"))
    toasts = list(app.toasts.items.values())
    assert len(toasts) == 1 and toasts[0].text == "⟶ hold · sent"
    assert spin(root, lambda: toasts[0].final)
    assert [b for p, b in f.posts if p == "/cmd"] == [{"line": "hold"}]
    assert toasts[0].level == "ok" and toasts[0].text == "✓ hold"
    assert app.presses.look("usv-hold") == "ok"
    assert spin(root, lambda: "Bok.TButton" == str(app.buttons["usv-hold"].cget("style")), 1)
    assert spin(root, lambda: app.presses.look("usv-hold") == "", 4.5)                # back to its own colour after 3 s
    app.buttons["usv-start"].invoke()                                                 # the fake refuses start_task
    assert spin(root, lambda: app.presses.look("usv-start") == "bad")
    bad = max(app.toasts.items.values(), key=lambda t: t.id)
    assert bad.level == "bad" and bad.text.startswith("✗ start_task — intent refused")
    assert any("console ✗ start_task" in ln for ln in app.log.get("1.0", "end").splitlines())    # the same line in the LOG tab


def test_arm_asks_first_and_ack_posts(root, app5):
    app, f = app5
    app.buttons["fcu-arm"].invoke()
    assert len(app.dialogs) == 1 and not [p for p in f.posts if p[0] == "/cmd"]       # nothing sent before the yes
    app.dialogs[0].yes.invoke()
    assert spin(root, lambda: ("/cmd", {"line": "arm"}) in f.posts)
    app.btn_model["ack-all"].ok = True
    app.buttons["ack-all"].invoke()
    assert spin(root, lambda: ("/ack", {"key": "all"}) in f.posts)


def test_the_cmd_line_uses_the_same_path(root, app5):
    app, f = app5
    app.cmd_entry.insert(0, "station_keep loiter")
    app.buttons["cmd-send"].invoke()
    assert spin(root, lambda: ("/cmd", {"line": "station_keep loiter"}) in f.posts)
    assert spin(root, lambda: "station_keep loiter" in app.cmd_out.get("1.0", "end"))


def test_down_when_the_ocs_goes_away(root):
    from optiprimex.tools.tkconsole.views import App
    f = FakeOcs("r2")
    app = App(root, f.url, fast_hz=5)
    try:
        assert spin(root, lambda: app.view is not None and app.view.link == "UP")
        f.modes["/state.json"] = "err"
        assert spin(root, lambda: app.view.link == "DOWN", 6)
        assert app.chips["link"].cget("text").startswith("DOWN")
        f.modes["/state.json"] = "ok"
        assert spin(root, lambda: app.view.link == "UP", 4)                           # recovers without a restart
    finally:
        app.close()
        f.close()
    assert app.threads_left() == []


def test_the_vehicles_tab_and_the_tiles_scroll_to_the_uav(root, app5):
    app, f = app5
    root.deiconify()
    root.geometry("1440x700")                                                         # short: the UAV tile is below the fold
    app.nb.select(app.tabs["VEHICLES"])
    assert spin(root, lambda: {"USV1", "UAV1"} <= set(app.veh_cards))
    usv = app.veh_cards["USV1"]
    assert usv["cv"].find_withtag("needle") and usv["cv"].find_withtag("attitude")
    assert usv["rows"]["HEARTBEAT"].cget("text").startswith("sending")
    sf = app.centre_sf
    assert spin(root, lambda: sf.canvas.yview()[1] < 1.0, 3)                          # the column is taller than the pane
    sf.canvas.yview_moveto(1.0)
    root.update_idletasks()
    assert sf.canvas.yview()[1] == 1.0                                               # scrolled to the bottom: the UAV tile is reachable
    root.withdraw()


def test_the_cmd_catalogue_tooltips_and_pick(root, app5):
    app, f = app5
    app.nb.select(app.tabs["CMD"])
    assert spin(root, lambda: app._cmd_built and app.cmd_tree.exists("w:start_task"))
    assert app.cmd_tree.exists("g:RUN") and app.cmd_tree.exists("w:station_keep")
    assert "no" in app.cmd_tree.item("w:start_task", "tags")                         # R5 AUTO_TASK: START_TASK is refused now -> grey
    tip = app._cmd_tip("w:start_task")
    assert "modifiers:" in tip and "--planner simple|s6" in tip and "now: ✗ START_TASK is accepted in AUTO_HOLD only" in tip
    app.cmd_tree.selection_set("w:start_task")
    assert spin(root, lambda: app.cmd_entry.get() == "start_task m0t1")
    assert app.cmd_use.cget("text").startswith("usage: start_task")
    assert app._btn_tip("usv-hold") == "sends: hold" and "not now: START_TASK" in app._btn_tip("usv-start")


def test_record_sends_the_depth_flag_and_shows_where(root, app5):
    from optiprimex.tools.tkconsole.views import rec_where
    app, f = app5
    assert app.rec_depth.get() is True
    app._record()
    assert spin(root, lambda: any(p == "/recording" for p, _ in f.posts))
    assert next(b for p, b in f.posts if p == "/recording") == {"op": "start", "depth": True}
    w = rec_where({"agx": {"dir": "/home/agx/Desktop/dm3/recordings/obs_1"}, "mac": {"dir": "/x/logs/recordings/2026-11-11/1501_obs",
                   "state": "recording", "frames": 12, "mb": 3.1, "depth": True, "depth_frames": 12}})
    assert "AGX video (svo2: RGB + depth + IMU): /home/agx/Desktop/dm3/recordings/obs_1" in w
    assert "Mac frames (2 Hz JPEG + depth): /x/logs/recordings/2026-11-11/1501_obs" in w and "depth 12 of 12" in w
    assert "logs/recordings/<date>" in rec_where({})                                 # before a recording: where it will go


def test_the_logging_buttons_post_logging_and_handle_404(root, app5):
    app, f = app5
    assert app.buttons["log-hdr"].cget("text") == "● START LOGGING" and app.buttons["log-cam"].cget("text") == "● START LOGGING"
    app.buttons["log-hdr"].invoke()
    assert spin(root, lambda: any(p == "/logging" for p, _ in f.posts))
    assert next(b for p, b in f.posts if p == "/logging") == {"op": "start", "label": "log", "depth": True}
    f.modes["/logging"] = "err"                                                      # the old backend: an error -> said, nothing started
    app.buttons["log-cam"].invoke()
    assert spin(root, lambda: sum(1 for p, _ in f.posts if p == "/logging") == 2)


def test_uav_run_chains_the_words_and_stops_at_a_refusal(root):
    from optiprimex.tools.tkconsole.views import App
    f = FakeOcs("r2")
    app = App(root, f.url, fast_hz=5)
    try:
        assert spin(root, lambda: app.view is not None and app.view.link == "UP")
        app.buttons["uav-run"].invoke()
        assert len(app.dialogs) == 1 and not [p for p in f.posts if p[0] == "/cmd"]   # asks first
        app.dialogs[0].yes.invoke()
        assert spin(root, lambda: len([1 for p, _ in f.posts if p == "/cmd"]) == 5, 12)
        assert [b["line"] for p, b in f.posts if p == "/cmd"] == ["station_keep loiter", "report on uav", "uav ready on", "declare uav on", "declare"]
        assert spin(root, lambda: app.presses.look("uav-run") == "ok", 3)
    finally:
        app.close()
        f.close()


def test_heartbeat_uav_resends_the_course(root, app5):
    app, f = app5
    app.buttons["uav-hb"].invoke()
    assert spin(root, lambda: ("/cmd", {"line": "uav resend_course"}) in f.posts, 5)
    lines = [b["line"] for p, b in f.posts if p == "/cmd"]
    assert lines.index("report on uav") < lines.index("uav resend_course")
