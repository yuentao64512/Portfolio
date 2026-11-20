"""v0.1.83.3: the launcher as a tkinter window (optiprimex/tools/launcher_tk.py + scripts/launch_tk.sh) - a client of the launcher's
own backend, on its own port (8093), scripts/launch.sh untouched."""
from __future__ import annotations

import queue
import subprocess
import threading
import time

import pytest

from optiprimex.tools import launcher as L
from optiprimex.tools import launcher_tk as K

from .conftest import ROOT, free_port


def test_rows_follow_the_web_page_render():
    v = K.visible({"mode": "sim"})
    assert v["world"] and v["perception"] and not v["fcu"] and not v["net"] and not v["ocs_host"]
    v = K.visible({"mode": "real", "cv": "real", "net": "lan", "uav": "off"})
    assert v["fcu"] and v["net"] and not v["world"] and not v["perception"] and not v["ocs_host"] and not v["uav_buoys"]
    v = K.visible({"mode": "real", "cv": "fake", "net": "manual", "uav": "on", "free_run": "off"})
    assert v["world"] and v["ocs_host"] and v["uav_buoys"] and v["free_run"]
    assert K.preset_cfg({"x": {"title": "X", "about": "a", "uav": "on"}}, "x") == {"uav": "on", "uav_buoys": "broker"}
    assert K.seg_label("judge", "venue") == "DEMO (venue)" and K.seg_label("log", True) == "on"


def test_plan_status_session_views():
    r = {"ok": False, "errors": ["clock off"], "plan": {"files": [{"path": "logs/launch/p.yaml"}], "remote_files": [],
         "steps": [{"host": "nx", "label": "boat", "cmd": "ssh nx run_usv.sh", "clock": {"ok": False}}], "stop": "stop_all.sh", "notes": ["n1"]}}
    lines = K.plan_lines(r)
    assert lines[0] == ("err", "clock off", None) and ("err", "[nx] 1. boat", None) in lines
    assert ("cmd", "ssh nx run_usv.sh", "ssh nx run_usv.sh") in lines and lines[-1] == ("note", "• n1", None)
    assert K.status_view(None)["tone"] == "bad"
    s = K.status_view({"state": "done", "dry": True, "log": [{"kind": "ok", "text": "up"}]})
    assert s["state"] == "done · dry run" and s["tone"] == "ok" and s["log"] == [("ok", "up")]
    d = {"probe": {"up": True, "hosts": {"nx": {"reach": True, "found": ["usv"], "missing": ["mm"]}}, "foreign": []},
         "record": {"when": "12:00", "title": "event", "cfg": {"mode": "real", "preset": "event"}}, "adopted": False,
         "pages": [{"name": "OCS", "url": "http://x", "up": True, "title": "t", "why": "w"}], "progress": [{"host": "mac", "label": "ocs",
         "why": "tmux", "state": "up", "key": "k"}], "clocks": {"nx": {"offset_s": 0.4, "ok": True}}}
    v = K.session_view(d)
    assert v["banner"] and v["reconnect"] and "12:00" in v["banner_text"] and "missing mm" in v["banner_text"] and not v["readonly"]
    assert v["hosts"][0]["restart"] is False and v["links"][0]["up"] and [p["word"] for p in v["progress"]] == ["ok", "up"]
    d["adopted"] = True
    v = K.session_view(d)
    assert not v["banner"] and v["readonly"] and v["hosts"][0]["restart"] and v["record_cfg"]["preset"] == "event"


def test_net_and_course_rows():
    t = {"rows": [{"role": "AGX", "key": "AGX_HOST", "lan": "192.168.1.99", "ts": "", "hosts": [{"addr": "192.168.1.99", "up": True}],
                   "ssh": {"which": "agx", "target": "u@h", "auth": "key"}, "observer": {"state": "down", "text": "observer: down"}}]}
    r = K.net_rows(t)[0]
    assert r["ssh"] == "agx" and r["up"] and r["ts"] == "-" and r["now"] == "● 192.168.1.99" and r["observer"]["state"] == "down"
    head, rows = K.course_rows({"nic": {"addr": "192.168.66.144", "iface": "en18", "dhcp": True}, "rows": [
        {"course": "BRAVO", "host": "192.168.66.2", "port": 1883, "on_network": True, "broker": True,
         "rx_course": {"course_id": "BRAVO", "corners": 5, "pinger_hz": 32000}}]})
    assert "en18" in head and rows[0] == ("BRAVO", "192.168.66.2:1883", "● on this network", "up", "BRAVO · 5 corners · pinger 32 kHz")


def test_toasts():
    now = [0.0]
    t = K.Toasts(clock=lambda: now[0])
    a = t.add("LAUNCH · sent")
    now[0] = 1.0
    t.resolve(a, True, "LAUNCH")
    now[0] = 3.9
    assert t.tick()[0][0]["state"] == "ok"
    now[0] = 4.1
    assert t.tick() == ([], 0)                                     # 3 s after its final state
    b = t.add("STOP ALL · sent")
    now[0] = 14.5
    assert t.tick()[0][0]["state"] == "late"                        # 10 s without an answer
    t.resolve(b, False, "STOP ALL - type stop")
    assert t.tick()[0][0]["text"] == "STOP ALL - type stop"
    for i in range(6):
        t.add(f"x{i}", "info")
    vis, more = t.tick()
    assert len(vis) == 5 and more == 2 and vis[0]["text"] == "x5"


def test_the_client_against_a_dry_backend():
    port = free_port()
    srv = L.Server(("127.0.0.1", port), L.make_handler(L.Runner(dry=True)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    try:
        api = K.Api(f"http://127.0.0.1:{port}")
        p = api.get("/presets.json")
        assert p and "sim" in p["presets"] and "judge" in p["fields"]
        r = api.post("/plan", {"cfg": K.preset_cfg(p["presets"], "sim")})
        assert r["ok"] and r["plan"]["steps"] and K.plan_lines(r)[0][0] == "step"
        assert api.post("/stop", {"confirm": "no"})["ok"] is False            # the backend's own refusal comes back, never raised
        assert K.Api(f"http://127.0.0.1:{free_port()}").post("/stop", {}, timeout=1)["ok"] is False
        assert K.Api(f"http://127.0.0.1:{free_port()}").get("/status.json", timeout=1) is None
        out: queue.Queue = queue.Queue()
        w = K.Worker(out, "t")
        w.start()
        for i in range(3):
            w.put("plan", lambda i=i: i)                                         # coalesced: only the newest plan runs
        time.sleep(0.8)
        got = []
        while not out.empty():
            got.append(out.get())
        w.stop_ev.set()
        assert got == [("plan", (3, 2))]
    finally:
        srv.shutdown()


def test_the_script_is_separate_from_launch_sh():
    sh = (ROOT / "scripts" / "launch_tk.sh").read_text()
    assert subprocess.run(["bash", "-n", str(ROOT / "scripts" / "launch_tk.sh")]).returncode == 0
    assert 'LAUNCH_TK_PORT:-8093' in sh and "optiprimex.tools.launcher --port" in sh and "optiprimex.tools.launcher_tk" in sh
    assert "start_new_session=True" in sh and "nohup \"$PY\"" not in sh                # v0.1.84.1: a Ctrl-C in the terminal leaves the backend
    assert "8094 is the web launcher's port" in sh and "launch.sh" not in sh.replace("launch_tk.sh", "").replace("scripts/launch.sh", "")
    assert "launcher_tk" not in (ROOT / "scripts" / "launch.sh").read_text()


def test_the_console_helpers(tmp_path):
    from pathlib import Path
    kit = Path("/k")
    assert K.console_argv(kit, {"mode": "real", "net": "lan"}) == ["/k/scripts/run_ocs_tk.sh", "--lan"]
    assert K.console_argv(kit, {"mode": "real", "net": "ts"})[-1] == "--ts"
    assert K.console_argv(kit, {"mode": "sim", "net": "lan"}) == ["/k/scripts/run_ocs_tk.sh"]
    assert K.auto_open(False, True, "tk") and not K.auto_open(None, True, "tk") and not K.auto_open(False, True, "web")
    assert not K.auto_open(True, True, "tk") and not K.auto_open(False, False, "tk")
    log = str(tmp_path / "ocs_tk.log")
    sh = tmp_path / "s.sh"
    sh.write_text("#!/bin/bash\necho 'the tk console is already open (pid 42)'\n")
    sh.chmod(0o755)
    r = K.spawn_console([str(sh)], log)
    assert r["ok"] and "already open (pid 42)" in r["text"]
    sh.write_text("#!/bin/bash\necho 'no Tk 8.6' >&2; exit 1\n")
    r = K.spawn_console([str(sh)], log)
    assert r["ok"] is False and "no Tk 8.6" in r["why"]
    sh.write_text("#!/bin/bash\nsleep 5\n")
    r = K.spawn_console([str(sh)], log, wait_s=0.3)
    assert r["ok"] and r["text"].startswith("opened (pid")
    assert "OCS_LINK" in (ROOT / "optiprimex" / "tools" / "launcher_tk.py").read_text()


def test_the_judge_line_and_the_venue_warning():
    env = {"VENUE_RC_HOST": "192.168.70.74", "VENUE_RC_PORT": "1910", "RC_BRAVO_HOST": "192.168.66.2", "RC_BRAVO_PORT": "1883"}
    t = K.judge_target({"judge": "rctest"}, env)
    assert t["host"] == "127.0.0.1" and "THIS Mac" in t["where"]
    assert K.judge_target({"judge": "venue"}, env)["where"] == "DEMO - the venue rc-test 192.168.70.74:1910"
    assert K.judge_target({"judge": "bravo"}, env)["host"] == "192.168.66.2"
    assert K.judge_target({"judge": "fake"}, env)["host"] is None
    assert "unset" in K.judge_target({"judge": "delta"}, env)["where"]
    w = K.judge_warning({"mode": "real"}, t, True, env)
    assert "NOT the venue" in w and "DEMO (venue)" in w                     # Dmitrii's 14:54 launch: rctest while the venue answered
    assert K.judge_warning({"mode": "real"}, t, False, env) == ""             # the venue does not answer: nothing to warn about
    assert K.judge_warning({"mode": "sim"}, t, True, env) == ""
    assert K.judge_warning({"mode": "real"}, K.judge_target({"judge": "venue"}, env), True, env) == ""


def test_the_window_smoke():
    """One shared Tk for the process + a Toplevel per window, driven by the real mainloop() (session A's finding: a Tk per test or
    root.update() loops stall / crash macOS Tk 8.6.18)."""
    pytest.importorskip("tkinter")
    import tkinter as tk
    from .test_ocs_tk_smoke import shared_root
    root = shared_root()
    top = tk.Toplevel(root)
    top.withdraw()
    port = free_port()
    srv = L.Server(("127.0.0.1", port), L.make_handler(L.Runner(dry=True)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()

    def spin(pred, s=8.0):
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
    try:
        app = K.App(top, K.Api(f"http://127.0.0.1:{port}"))
        spawned = []
        app.spawn = lambda argv, log: spawned.append(argv) or {"ok": True, "text": "opened (pid 1)"}
        assert spin(lambda: app.presets and "1." in (app.last_text.get("plan") or ""))
        assert spin(lambda: (app.last_text.get("judge") or "").startswith("heartbeats + reports go to: our fake judge"))
        assert "mode" in app.vars and app.cur == "sim"
        app.vars["mode"].set("real")
        app._seg("mode", app.vars["mode"], "seg")
        assert spin(lambda: app.rows["fcu"][0].winfo_manager() == "grid")
        app._stop(app.stop_word)                                               # nothing typed -> a red toast, nothing sent
        vis, _ = app.toasts.tick()
        assert vis and vis[0]["state"] == "bad" and "type stop" in vis[0]["text"]
        app.open_console()                                                     # the tk console through the swapped spawn
        assert spin(lambda: spawned and any(t["state"] == "ok" and "OCS console (tk)" in t["text"] for t in app.toasts.items))
        assert spawned[0][0].endswith("scripts/run_ocs_tk.sh")
        app.ocs_up = False                                                      # the OCS link comes up -> opened by itself
        app._on_session({"probe": {"up": False, "hosts": {}, "foreign": []}, "pages": [{"name": "OCS page", "url": "http://x/", "up": True}]})
        assert spin(lambda: len(spawned) == 2)
        app.close()
    finally:
        srv.shutdown()
