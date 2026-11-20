"""sim_day: the sim-day sheet (docs/SIM_DAY.md §2) walked by a script against a running simulator (v0.1.33, ROADMAP 7a: "re-run the
sim-day sheet after WP10b and WP10e").

Drives the stack the way the sheet's operator does - ocs_cli for the intents, the SIM page's API (simview) for the transmitter, the
sticks, the teleports and RESET SIM - and records one row per check: scenario · check · pass / FINDING · what was seen. It never
starts or stops anything: `scripts/run_sim.sh pool` first (S10 needs `boxes`: run it again with `--only S10` after restarting on boxes).

  python -m optiprimex.tools.sim_day [--only S1,S2,...] [--md logs/bench/latest/sim_day.md] [--sim http://127.0.0.1:8097]

Exit 0 = every check passed, 1 = findings.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
import urllib.request
from typing import Callable, Dict, List, Optional

KIT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class Walk:
    def __init__(self, a: argparse.Namespace):
        self.a = a
        self.rows: List[dict] = []
        self.scn = ""

    # ---- the operator's hands ----------------------------------------------------------------------------------------------
    def cli(self, *words: str, wait: float = 5.0) -> str:
        cmd = [sys.executable, "-m", "optiprimex.tools.ocs_cli", "--port", str(self.a.hub_port), *words, "--wait", str(wait)]
        p = subprocess.run(cmd, capture_output=True, text=True, timeout=wait + 30, cwd=KIT, env=dict(os.environ, PYTHONPATH=KIT))
        return ((p.stdout or "") + (p.stderr or "")).strip()

    def state(self) -> dict:
        out = self.cli("state", wait=2)
        try:
            return json.loads(out[out.index("{"):])
        except ValueError:
            return {}

    def veh(self) -> dict:
        return (self.state().get("vehicles") or {}).get("USV1") or {}

    def sim(self, path: str, body: Optional[dict] = None, timeout: float = 60.0) -> dict:
        url = self.a.sim.rstrip("/") + path
        if body is None:
            return json.loads(urllib.request.urlopen(url, timeout=timeout).read())
        req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST", headers={"X-Sim-Web": "1", "Content-Type": "application/json"})
        try:
            return json.loads(urllib.request.urlopen(req, timeout=timeout).read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read())

    def map(self, timeout: float = 4.0) -> dict:
        """v0.1.79: one boat map digest (team/optiprime/v1/telemetry/usv/map on the hub) - its body; {} when none came."""
        import paho.mqtt.client as mqtt
        got: dict = {}
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"sim-day-map-{os.getpid()}", protocol=mqtt.MQTTv311)
        c.on_connect = lambda cl, u, f, rc, pr=None: cl.subscribe("team/optiprime/v1/telemetry/usv/map", 0)

        def on_msg(cl, u, m):
            try:
                env = json.loads(m.payload)
                got.update(env.get("body") or env)
            except ValueError:
                pass
        c.on_message = on_msg
        try:
            c.connect("127.0.0.1", self.a.hub_port, 10)
        except OSError:
            return {}
        c.loop_start()
        t0 = time.time()
        while not got and time.time() - t0 < timeout:
            time.sleep(0.1)
        c.loop_stop()
        c.disconnect()
        return got

    def fcu(self, **s) -> dict:
        return self.sim("/fcu", {"set": s})

    def wait(self, pred: Callable[[], bool], timeout: float, step: float = 1.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout:
            try:
                if pred():
                    return True
            except Exception:
                pass
            time.sleep(step)
        return False

    def wait_task(self, timeout: float) -> dict:
        """Until the vehicle leaves AUTO_TASK (or the timeout): the last vehicle record."""
        last = {}
        t0 = time.time()
        while time.time() - t0 < timeout:
            last = self.veh()
            if last.get("state") != "AUTO_TASK":
                break
            time.sleep(1.0)
        return last

    def trace_tail(self, n: int = 400) -> List[dict]:
        p = os.path.join(KIT, "logs", "bench", "latest", "mm_trace.jsonl")
        try:
            with open(p) as f:
                lines = f.readlines()[-n:]
            return [json.loads(x) for x in lines if x.strip()]
        except OSError:
            return []

    def mm_log_since(self, t0: float) -> str:
        p = os.path.join(KIT, "logs", "bench", "latest", "mm.log")
        try:
            with open(p) as f:
                return "".join(x for x in f.readlines() if x[:19] >= time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(t0)))
        except OSError:
            return ""

    def check(self, what: str, ok: bool, seen: str = "") -> bool:
        self.rows.append({"scn": self.scn, "check": what, "ok": bool(ok), "seen": seen[:300]})
        print(f"[{self.scn}] {'pass   ' if ok else 'FINDING'} {what}{(' - ' + seen[:200]) if seen else ''}", flush=True)
        return ok

    def clean(self) -> None:
        r = self.sim("/reset", {"confirm": "reset"})
        if not r.get("ok"):
            self.check("RESET SIM before the scenario", False, json.dumps(r)[:200])
        time.sleep(1.5)

    def handover(self) -> None:
        """v0.1.36: usv_ui holds the fake plant in run_sim.sh (tmux window `ui`): hand the FCU over, as the operator types it there."""
        try:
            wins = subprocess.run(["tmux", "list-windows", "-t", self.a.session, "-F", "#W"], capture_output=True, text=True, timeout=5).stdout.split()
        except (OSError, subprocess.TimeoutExpired):
            return
        if "ui" in wins:
            subprocess.run(["tmux", "send-keys", "-t", f"{self.a.session}:ui", "handover", "C-m"], timeout=5)
            time.sleep(1.5)

    def ready(self) -> dict:
        self.handover()
        self.fcu(mode=4)
        out = self.cli("ready", wait=15)
        return {"out": out, "r": self.state().get("r")}

    # ---- the scenarios (docs/SIM_DAY.md §2) -----------------------------------------------------------------------------------
    def S0(self):
        """v0.1.33: RESET SIM and the teleports."""
        w = self.sim("/world.json")
        self.check("world.json: teleports listed", len(w.get("teleports") or []) >= 1, ", ".join(t["name"] for t in w.get("teleports") or []))
        r = self.sim("/teleport", {"name": (w["teleports"][-1])["name"]})
        self.check("teleport by name", r.get("ok"), json.dumps(r)[:160])
        self.fcu(kill=True)
        time.sleep(2.0)
        self.check("kill -> KILLED", self.wait(lambda: self.veh().get("state") == "KILLED", 6), self.veh().get("state", ""))
        t0 = time.time()
        r = self.sim("/reset", {"confirm": "reset"})
        time.sleep(1.5)
        st, v, b = self.state(), self.veh(), self.sim("/world.json")["boat"]
        self.check("RESET SIM: R2 / MANUAL / at start", r.get("ok") and st.get("r") in ("R2", "R1") and v.get("state") == "MANUAL" and abs(b["x_n"]) < 0.3 and abs(b["y_e"]) < 0.3,
                   f"{time.time() - t0:.1f} s · {st.get('r')} · {v.get('state')} · boat {b}")

    def S1(self):
        self.clean()
        self.fcu(mode=0)
        b0 = self.sim("/world.json")["boat"]
        t0 = time.time()
        while time.time() - t0 < 4.0:
            self.sim("/sticks", {"surge": 0.6})
            time.sleep(0.1)
        self.sim("/sticks", {"surge": 0.0})
        b1 = self.sim("/world.json")["boat"]
        self.check("arrow keys (sticks) move the boat in MANUAL", b1["x_n"] - b0["x_n"] > 1.0, f"N {b0['x_n']} -> {b1['x_n']}")
        self.check("the mission manager sees MANUAL", self.veh().get("state") == "MANUAL", self.veh().get("state", ""))
        cam = self.sim("/status")
        self.check("the camera renders (source sim)", cam.get("source") == "sim" and cam.get("frames", 0) > 5, json.dumps(cam)[:120])
        # the saved points: HOME at the start, A 10 m north, B 1 m east of A, C 12 m further north
        for name, (x, y) in (("HOME", (0.0, 0.0)), ("A", (10.0, 0.0)), ("B", (10.0, 1.0)), ("C", (22.0, 1.0))):
            self.sim("/teleport", {"x_n": x, "y_e": y, "hdg": 0.0})
            time.sleep(1.2)
            out = self.cli("save_point", name)
            self.check(f"save_point {name}", "DONE" in out, out.splitlines()[-1] if out else "")
        pts = {p["name"] for p in self.sim("/world.json").get("points") or []}
        self.check("the points reach the simulator's teleport list", {"HOME", "A", "B", "C"} <= pts, ",".join(sorted(pts)))
        self.sim("/teleport", {"name": "start"})

    def S2(self):
        self.clean()
        self.fcu(mode=0)
        time.sleep(1.5)                  # v0.1.43: the boat sees the switch on the FCU's next heartbeat (1 Hz) - as the operator sees it
        out = self.cli("enable_auto", wait=4)
        self.check("enable_auto with the mode switch in MANUAL: refused with the cause", "REJECTED" in out or "refused" in out.lower(),
                   out.splitlines()[-1] if out else "")
        self.fcu(mode=4)
        t0 = time.time()
        r = self.ready()
        self.check("ready -> R5 (the fake judge's RunStart)", r["r"] == "R5", f"{r['r']} after {time.time() - t0:.1f} s")
        out = self.cli("end_run")
        self.check("end_run -> R6", self.wait(lambda: self.state().get("r") == "R6", 5), self.state().get("r", ""))
        self.cli("reset")
        self.check("reset -> R2 / MANUAL", self.wait(lambda: self.state().get("r") == "R2" and self.veh().get("state") == "MANUAL", 6),
                   f"{self.state().get('r')} {self.veh().get('state')}")

    def S3(self):
        self.clean()
        self.ready()
        self.cli("start_task", "m0t0")
        for _ in range(3):
            time.sleep(0.8)
            self.cli("step")
        v = self.wait_task(10)
        self.check("m0t0: step x3 -> DONE (AUTO_HOLD)", v.get("state") == "AUTO_HOLD", json.dumps(v)[:160])

    def S4(self):
        self.clean()
        self.ready()
        out = self.cli("start_task", "m0t2", "--names", "A,X")
        self.check("m0t2 --names A,X refused (point X not saved)", "X" in out and ("not saved" in out or "REJECTED" in out or "refused" in out.lower()), out.splitlines()[-1] if out else "")
        out = self.cli("start_task", "m0t2", "--names", "A B C", "--step")
        self.check("m0t2 --names \"A B C\" --step accepted", "AUTO_TASK" in out or "DONE" in out, out.splitlines()[-1] if out else "")
        self.wait(lambda: self.veh().get("state") == "AUTO_SAFE_HOLD", 10)
        self.cli("resume")
        v = self.wait_task(120)
        b = self.sim("/world.json")["boat"]
        self.check("m0t2 FOLLOW -> DONE near C (N 22 E 1)", v.get("state") == "AUTO_HOLD" and abs(b["x_n"] - 22.0) < 1.5 and abs(b["y_e"] - 1.0) < 1.5,
                   f"{v.get('state')} boat {b}")

    def S5(self):
        self.clean()
        self.sim("/teleport", {"name": "A"})
        time.sleep(1.0)
        self.ready()
        t0 = time.time()
        self.cli("start_task", "m0t3", "--names", "A,B")
        v = self.wait_task(90)
        self.check("m0t3 A,B -> DONE", v.get("state") == "AUTO_HOLD", f"{v.get('state')} in {time.time() - t0:.0f} s")
        p = subprocess.run([sys.executable, "-m", "optiprimex.tools.mode_log", os.path.join(KIT, "logs", "bench", "latest", "mm_trace.jsonl"), "--lab"],
                           capture_output=True, text=True, timeout=60, cwd=KIT)
        verdict = [x for x in p.stdout.splitlines() if "PASS" in x or "FAIL" in x]
        self.check("mode_log --lab: PASS", p.returncode == 0 and any("PASS" in x for x in verdict), (verdict or [p.stdout[-200:]])[-1])

    def S6(self):
        self.clean()
        self.ready()
        self.cli("start_task", "m0t6", "--centre", "A", "--radius", "3", "--dir", "cw", "--turns", "1")
        v = self.wait_task(150)
        self.check("m0t6 orbit round A -> DONE", v.get("state") == "AUTO_HOLD", json.dumps(v)[:160])

    def S7(self):
        self.clean()
        self.ready()
        self.cli("start_task", "m0t2", "--names", "A,B,C")
        time.sleep(6)
        out = self.cli("home", wait=6)
        self.check("⌂ HOME mid-path: TEST_PATH aborted, RETURN_HOME", "HOME" in out.upper(), out.splitlines()[-1] if out else "")
        v = self.wait_task(150)
        b = self.sim("/world.json")["boat"]
        self.check("RETURN_HOME -> AUTO_HOLD at HOME", v.get("state") == "AUTO_HOLD" and abs(b["x_n"]) < 2.0 and abs(b["y_e"]) < 2.0, f"{v.get('state')} boat {b}")
        self.cli("manual")
        self.fcu(mode=0)
        out = self.cli("home", wait=4)
        self.check("HOME from MANUAL refused", "REJECTED" in out or "refused" in out.lower() or "MANUAL" in out, out.splitlines()[-1] if out else "")
        self.fcu(mode=4)

    def S8(self):
        self.clean()
        self.ready()
        t0 = time.time()
        self.cli("start_task", "m1t1")
        v = self.wait_task(300)
        log = self.mm_log_since(t0)
        stages = [s for s in ("SEARCH_ENTRY", "CIRCLE_ENTRY_CW", "TRANSIT", "EXIT", "CIRCLE_EXIT_CCW", "DONE") if s in log]
        self.check("m1t1 on pool -> DONE through every stage", v.get("state") == "AUTO_HOLD" and len(stages) == 6, f"{v.get('state')} in {time.time() - t0:.0f} s · {' > '.join(stages)}")

    T3_STAGES = ("FIND_DOCK", "TO_LINE", "SWAY_SEARCH", "SWAY_TO_ABEAM", "TURN_IN", "ENTER", "DOCKED", "REPORT_DOCK", "AIM_FIRE",
                 "REPORT_FIRE", "UNDOCK")

    def dock_cmd(self, body: dict) -> None:
        """v0.1.48: sim/dock/cmd on the boat broker (fake_dock) - the GREEN bay moved by hand, as the sheet's operator does."""
        import paho.mqtt.client as mqtt
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"sim-day-dock-{os.getpid()}", protocol=mqtt.MQTTv311)
        c.connect("127.0.0.1", self.a.boat_port, 10)
        c.loop_start()
        try:
            c.publish("sim/dock/cmd", json.dumps({"body": body}), qos=1).wait_for_publish(3)
        finally:
            c.loop_stop()
            c.disconnect()

    def _m3t1(self, *words: str, during: Optional[Callable[[], None]] = None, timeout: float = 320) -> str:
        self.clean()
        self.dock_cmd({"cmd": "green_bay", "bay": 2})
        self.sim("/teleport", {"name": "N32 E3 dock"})
        time.sleep(1.0)
        self.ready()
        t0 = time.time()
        self.cli("start_task", "m3t1", *words)
        if during is not None:
            during()
        self.last_v = self.wait_task(timeout)
        return self.mm_log_since(t0)

    def S9(self):
        """v0.1.48 (WP10j-b): the sway search, the reports strictly in order; the GREEN bay moved mid-search; a wrong call -> BACK_OUT."""
        t0 = time.time()
        log = self._m3t1()
        v = self.last_v
        stages = [s for s in self.T3_STAGES if s in log]
        self.check("m3t1 from N32 E3 -> DONE through every stage (SWAY_SEARCH incl.)", v.get("state") == "AUTO_HOLD" and len(stages) == len(self.T3_STAGES),
                   f"{v.get('state')} in {time.time() - t0:.0f} s · {' > '.join(stages)}")
        i_d, i_f = log.find("DockingReport: bay 2"), log.find("FirefightingReport:")
        ff = log[i_f:].splitlines()[0] if i_f >= 0 else ""
        self.check("DockingReport bay 2, then FirefightingReport with window_id 1|2, complete",
                   0 <= i_d < i_f and ("'window_id': 1" in ff or "'window_id': 2" in ff) and "'complete': True" in ff, ff[:200])

        def move_green():
            if self.wait(lambda: "search pass 1" in self.mm_log_since(t1), 90):
                self.dock_cmd({"cmd": "green_bay", "bay": 3})
        t1 = time.time()
        log = self._m3t1(during=move_green)
        self.check("GREEN moved to bay 3 mid-SWAY_SEARCH -> DockingReport bay 3", self.last_v.get("state") == "AUTO_HOLD" and "DockingReport: bay 3" in log,
                   " | ".join(x for x in log.splitlines() if "search pass" in x or "DockingReport" in x)[-300:])
        log = self._m3t1("--bay", "1")
        self.check("--bay 1 on GREEN 2: BACK_OUT -> search -> DockingReport bay 2", self.last_v.get("state") == "AUTO_HOLD" and "BACK_OUT" in log
                   and "DockingReport: bay 2" in log and "DockingReport: bay 1" not in log,
                   " | ".join(x for x in log.splitlines() if "wrong call" in x or "DockingReport" in x)[-300:])
        self.dock_cmd({"cmd": "green_bay", "bay": 2})

    def S10(self):
        w = self.sim("/world.json")
        if w.get("name") != "world_boxes":
            self.check("m0t4 needs the boxes world (scripts/run_sim.sh boxes, then --only S10)", True, f"skipped on {w.get('name')}")
            return
        self.clean()
        self.ready()
        t0 = time.time()
        self.cli("start_task", "m0t4", "--scan-x", "40", "--scan-y", "0")      # the boxes stand at N 50: scan from 10 m off
        v = self.wait_task(200)
        log = self.mm_log_since(t0)
        stages = [x for x in ("GO_SCAN", "OBSERVE", "ALIGN", "APPROACH", "STOP", "DONE") if f"-> {x} " in log]
        self.check("m0t4 on boxes -> DONE", v.get("state") == "AUTO_HOLD" and len(stages) == 6, f"{v.get('state')} in {time.time() - t0:.0f} s · {' > '.join(stages)}")

    def S11(self):
        self.clean()
        self.ready()
        self.cli("start_task", "m4t0")
        time.sleep(2)
        cseq = str(int(time.time()) % 100000)     # v0.1.43: a fresh command_seq - the OCS drops a repeated one (the sheet run twice on one stack)
        self.cli("inject", "assistance", "--lat", "1.28112", "--lon", "103.85501", "--cseq", cseq)
        ok = self.wait(lambda: "WAIT_CONFIRM" in self.mm_log_since(time.time() - 120), 60)
        self.check("m4t0: LISTEN > ACK > TRANSIT > READY > WAIT_CONFIRM", ok)
        v = self.wait_task(30)                                 # the fake judge confirms by itself after 1 s
        self.check("m4t0 -> DONE on the judge's confirm", v.get("state") == "AUTO_HOLD", json.dumps(v)[:160])

    def S14(self):
        """v0.1.54 (items 12 / 13): m1t1 with the S6 planner on Course Extra from the T1 teleport -> DONE; the entry locked is the
        world's FLASHING BLUE (not a dark buoy); the judge page's last SafePassageReport lists every Task 1 buoy within 1 m of the
        truth with the truth's BeaconState, entry / exit set (judge_web /state.json `passage`)."""
        w = self.sim("/world.json")
        if w.get("name") != "world_extra":
            self.check("S6 on Course Extra needs the extra world (scripts/run_sim.sh extra, then --only S14)", True, f"skipped on {w.get('name')}")
            return
        self.clean()
        tp = next((t["name"] for t in w.get("teleports") or [] if str(t.get("name", "")).startswith("T1: before")), None)
        self.check("the T1 teleport", tp is not None and self.sim("/teleport", {"name": tp}).get("ok"), str(tp))
        time.sleep(1.5)
        self.ready()
        t0 = time.time()
        self.cli("start_task", "m1t1", "--planner", "s6")
        v = self.wait_task(400)
        log = self.mm_log_since(t0)
        stages = [x for x in ("SEARCH_ENTRY", "CIRCLE_ENTRY_CW", "TRANSIT", "EXIT", "CIRCLE_EXIT_CCW", "DONE") if x in log]
        self.check("m1t1 --planner s6 on extra -> DONE through every stage", v.get("state") == "AUTO_HOLD" and len(stages) == 6,
                   f"{v.get('state')} in {time.time() - t0:.0f} s · {' > '.join(stages)}")
        locks = [x.split("entry locked: ", 1)[1][:60] for x in log.splitlines() if "entry locked: " in x]
        rejected = [x.split("T1 ", 1)[1][:90] for x in log.splitlines() if "is not blue after all" in x]
        try:
            jw = json.loads(urllib.request.urlopen(f"http://127.0.0.1:{os.environ.get('JUDGE_WEB_PORT', '8095')}/state.json", timeout=5).read())
        except OSError as exc:
            self.check("the judge page answers", False, str(exc))
            return
        p = next(iter(jw.get("passage") or []), None)
        ent = (p or {}).get("entry") or {}
        self.check("the entry circled = the FLASHING BLUE (the reported entry within 2 m of the truth's)",
                   ent.get("d_m") is not None and ent["d_m"] < 2.0, f"entry {ent.get('d_m')} m off · locks {locks} · rejected {rejected}")
        rows = (p or {}).get("buoys") or []
        bad = [r for r in rows if r.get("d_m") is None or r["d_m"] > 1.0 or r.get("truth_beacon") != r.get("beacon")]
        self.check("the judge's last SafePassageReport: every T1 buoy, state = truth, within 1 m; entry / exit set",
                   bool(p) and len(rows) >= 10 and not bad and not p.get("missing") and p.get("entry") and p.get("exit"),
                   f"{len(rows)} buoys, off {[(r.get('truth_id'), r.get('beacon'), r.get('d_m')) for r in bad]}, missing {(p or {}).get('missing')}, "
                   f"max {max([r.get('d_m') or 0 for r in rows] or [0]):.2f} m")

    def S15(self):
        """v0.1.79 (WP18's colour-blind world, WP35): world_scout - every light off, the ZED 4 mm at 1.0 m: the boat's map fills with
        buoys of no colour (black / unknown) within 20 s of standing at the start."""
        w = self.sim("/world.json")
        if w.get("name") != "world_scout":
            self.check("S15 needs the scout world (scripts/run_sim.sh scout, then --only S15,S16)", True, f"skipped on {w.get('name')}")
            return
        self.clean()
        ok = self.wait(lambda: len([e for e in (self.map().get("entities") or []) if e.get("kind") == "buoy"]) >= 3, 20)
        ents = [e for e in (self.map().get("entities") or []) if e.get("kind") == "buoy"]
        self.check("colour-blind: the map's buoys carry no light colour", ok and all(e.get("colour") in ("black", "unknown") for e in ents),
                   f"{len(ents)} buoys: {sorted(set(e.get('colour') for e in ents))}")

    def S16(self):
        """v0.1.79 (WP35): mission scout on S15's world -> DONE (AUTO_HOLD) with every buoy logged (the mission manager's log: one
        `scout: Bn logged` line each and the summary), inside the budget."""
        w = self.sim("/world.json")
        if w.get("name") != "world_scout":
            self.check("S16 needs the scout world (scripts/run_sim.sh scout)", True, f"skipped on {w.get('name')}")
            return
        self.clean()
        self.ready()
        t0 = time.time()
        self.cli("start_task", "scout")
        v = self.wait_task(660)
        log = self.mm_log_since(t0)
        n = len([x for x in log.splitlines() if "scout: B" in x and (" logged " in x or " blocked " in x)])
        nb = len([x for x in log.splitlines() if "scout: B" in x and " blocked " in x])
        stages = [x for x in ("ADVANCE", "APPROACH", "LOG", "RETURN") if x in log]
        guard = [x.split("guard ", 1)[1][:60] for x in log.splitlines() if "guard STOP" in x]
        self.check("scout -> DONE with every buoy of the field logged (v0.1.80: blocked by the guard counts)",
                   v.get("state") == "AUTO_HOLD" and n >= 10 and "scout: summary" in log,
                   f"{v.get('state')} in {time.time() - t0:.0f} s · {n} logged ({nb} blocked) · {' > '.join(stages)} · guard {guard[:3]}")

    def S12(self):
        self.clean()
        self.ready()
        self.cli("start_task", "m0t2", "--names", "A,B,C")
        time.sleep(4)
        self.fcu(kill=True)
        self.check("kill during a task -> KILLED", self.wait(lambda: self.veh().get("state") == "KILLED", 6), self.veh().get("state", ""))
        self.fcu(kill=False)
        self.cli("confirm_safe")
        self.check("kill off + confirm_safe -> MANUAL", self.wait(lambda: self.veh().get("state") == "MANUAL", 6), self.veh().get("state", ""))
        self.fcu(mode=4)
        self.handover()
        self.cli("enable_auto")
        self.cli("start_task", "m0t2", "--names", "A,B,C")
        time.sleep(3)
        self.fcu(rc6=1900)
        time.sleep(3)
        v = self.veh()
        self.check("RC6 override during a task: the vehicle leaves AUTO", v.get("state") not in ("AUTO_TASK", "AUTO_HOLD"), v.get("state", ""))
        self.fcu(rc6=1000)
        self.clean()
        self.ready()
        self.fcu(gps_fix=0)
        self.check("GPS lost in AUTO_HOLD -> FAULT", self.wait(lambda: self.veh().get("state") == "FAULT", 8), json.dumps(self.veh())[:160])
        self.fcu(gps_fix=3)
        self.clean()
        self.ready()
        self.sim("/fcu", {"pause_stream": 4.0})
        self.check("FCU pause 4 s -> FAULT (FCU_LOST)", self.wait(lambda: self.veh().get("state") == "FAULT", 8, 0.5), json.dumps(self.veh())[:160])
        time.sleep(4)
        self.clean()

    def PAGE(self):
        """v0.1.43 (OCS_V2_SPEC §12a): the OCS page's own steps of the sheet through --page (/ = v2, /v1/ = the classic page): the page
        loads with its controls, its /state.json carries the bar, a teleport and RESET SIM through the page's /sim proxy, a command
        through its /cmd, the camera / course / markers feeds."""
        if not self.a.page:
            return
        base = self.a.page.rstrip("/") + "/"

        def get(path):
            with urllib.request.urlopen(base + path, timeout=10) as r:
                return r.read().decode("utf-8", "replace")

        def post(path, body):
            req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST",
                                         headers={"Content-Type": "application/json", "X-OCS-Web": "1"})
            with urllib.request.urlopen(req, timeout=60) as r:
                return json.loads(r.read())
        html = get("")
        v1 = "/v1/" in base
        self.check(f"page {base}: loads with MISSION CONTROL and the RUN CHECKLIST",
                   'id="cw"' in html and 'id="ck"' in html and ((not v1 and 'id="tw"' in html) or (v1 and "classic" in html)), f"{len(html)} bytes")
        st = json.loads(get("state.json"))
        self.check("page /state.json: the bar and the vehicle", bool(st.get("bar")) and bool(st.get("status")), json.dumps(st.get("bar"))[:160])
        w = self.sim("/world.json")
        r = post("sim", {"path": "/teleport", "body": {"name": (w["teleports"][-1])["name"]}})
        self.check("page /sim: teleport by name (the MAP tab's bar)", bool(r.get("ok")), json.dumps(r)[:160])
        r = post("sim", {"path": "/reset", "body": {"confirm": "reset"}})
        time.sleep(1.5)
        self.check("page /sim: RESET SIM (SETTINGS)", bool(r.get("ok")) and self.state().get("r") in ("R1", "R2"), json.dumps(r)[:160])
        r = post("cmd", {"line": "next"})
        self.check("page /cmd: a word from the console", "next" in json.dumps(r) or r.get("ok") is not None, json.dumps(r)[:160])
        feeds = [p for p in ("cam.json", "course.json", "markers.json", "map.html?mini=1", "themes.css") if get(p) is not None]
        self.check("page feeds: camera, course, markers, mini-map, themes", len(feeds) == 5, ", ".join(feeds))

    # ---- the report ----------------------------------------------------------------------------------------------------------
    def md(self) -> str:
        from .. import __version__
        n_ok = sum(r["ok"] for r in self.rows)
        out = [f"# sim day re-run - {time.strftime('%Y-%m-%d %H:%M')} - kit v{__version__} - world {self.world}", "",
               f"{n_ok} / {len(self.rows)} checks passed", "", "| scenario | check | result | seen |", "|---|---|---|---|"]
        out += [f"| {r['scn']} | {r['check']} | {'pass' if r['ok'] else '**FINDING**'} | {r['seen'].replace('|', '/')} |" for r in self.rows]
        return "\n".join(out) + "\n"


ORDER = ["S0", "S1", "S2", "S3", "S4", "S5", "S6", "S7", "S8", "S9", "S10", "S11", "S12", "S14", "S15", "S16", "PAGE"]     # PAGE (v0.1.43): with --page; S14 (v0.1.54): extra (S13 = the sheet's end)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", default="", help="e.g. S1,S5 (S1 saves the points the later scenarios use)")
    ap.add_argument("--sim", default="http://127.0.0.1:8097")
    ap.add_argument("--hub-port", type=int, default=1882)
    ap.add_argument("--boat-port", type=int, default=1884, help="v0.1.48: the boat broker (S9 moves the GREEN bay on sim/dock/cmd)")
    ap.add_argument("--session", default=os.environ.get("OPTIPRIMEX_SESSION", "optiprimex"), help="the bench's tmux session (usv_ui's window ui)")
    ap.add_argument("--md", default="", help="write the result table here (markdown)")
    ap.add_argument("--page", default="", help="v0.1.43: also walk the OCS page's own steps here (http://127.0.0.1:8096/ = v2, .../v1/ = classic)")
    a = ap.parse_args(argv)
    w = Walk(a)
    try:
        w.world = w.sim("/world.json").get("name")
    except OSError:
        print("no simulator at " + a.sim + " (scripts/run_sim.sh pool)", file=sys.stderr)
        return 2
    todo = [s.strip().upper() for s in a.only.split(",") if s.strip()] or ORDER
    for s in todo:
        w.scn = s
        try:
            getattr(w, s)()
        except Exception as exc:
            w.check("the scenario ran", False, f"{exc.__class__.__name__}: {exc}")
    text = w.md()
    print(text)
    if a.md:
        with open(a.md, "w") as f:
            f.write(text)
    return 0 if all(r["ok"] for r in w.rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
