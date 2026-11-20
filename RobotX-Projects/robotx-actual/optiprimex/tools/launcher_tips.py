"""v0.1.94.3 (Dmitrii 14 Nov: "tooltips pop-up during launcher mode select upon hovering"): the hover texts of the Tk launcher.

ROW_TIPS = a row label's tip (the web page's <label title=...>), CHOICE_TIPS = one choice's tip (the web page's TIPS, plus the
choices it had no text for). Pure data: launcher_tk.py shows them in a pop-up after a short hover.
"""
from typing import Any, Dict

ROW_TIPS: Dict[str, str] = {
    "mode": "sim: this Mac only (run_sim.sh); real: run_ocs.sh here, run_usv.sh on the NX, run_agx.sh on the AGX over ssh",
    "judge": "who judges: our fake, RoboNation's rctest on this Mac, the venue DEMO, or a course RoboCommand (ALPHA-DELTA, auto, typed)",
    "world": "the world file the simulator runs",
    "perception": "camera: rendered frames through the students' measuring code to the map; geometric: the fast fake",
    "usv_ui": "usv_ui on the boat (sim: on the simulated boat)",
    "mission": "the mission preselected at launch (the OCS shows it; ocs_cli mission off clears it)",
    "auto_mission": "on = RunStart -> 3-2-1 -> start_task by itself (once per run); off = RunStart says mission_ready, the boat stays in AUTO_HOLD",
    "t4": "T4 Dynamic Incident Response in the declaration - on = TIER_CORE, off = TIER_NONE (an operator's mission --t4 in the OCS still wins)",
    "lucky": "m1t1 / m1t2 fallback: uncertain-colour pairs count as gates, blue is checked as usual but any colour passes; no entry found -> "
             "the exit is orbited instead; an entry found later -> orbit it and run the gates again (once); speeds x1.25. A risk - off in every preset",
    "record": "the ZED recording (observer --record --pairs-s 10 from launch) and the logging session at DECLARE; off unless cv real",
    "fcu": "fake: the software FCU on the NX; uart / usb / eth: the real Pixhawk (props off on the bench)",
    "usv_profile": "run_usv.sh --pool | --lab (bench guards, never on the water) | --course",
    "cv": "real: cv_publish + the camera on the AGX; fake: run_agx.sh --sim on the world; off: the AGX is not started",
    "usv_window": "where the USV window (8098) serves the team LAN - nx: on the boat (survives this Mac), mac: here, off: none; "
                  "commands from another computer need the PIN",
    "uav_buoys": "the UAV's buoys - broker: the NX's broker bridges their Orin mosquitto; udp: their detector -> our adapter's 14605 -> the hub",
    "free_run": "FREE RUN: every stage against a ghost plant, the real FCU gets mode requests only. Bench only, props off, ESC power off",
    "uav": "on = the UAV adapter and the UAV window (8099). Only starts processes: the UAV is declared only with `declare uav on`",
    "net": "ts: the tailnet (network.ts.env); lan: the event / pool switch (network.lan.env); manual: type the addresses",
    "heartbeat_hz": "the OCS heartbeat to RoboCommand (ocs.heartbeat_hz)",
    "status_hz": "the boat's status rate on the broker (rates.status_hz)",
    "map_hz": "the boat's perception-map rate on the broker (rates.map_hz)",
    "ui": "the OCS page: v2 (default) or the classic page frozen at /v1/",
    "log": "LOG=1: every MQTT message to SQLite (real stacks)",
}

CHOICE_TIPS: Dict[str, Dict[str, str]] = {
    "mode": {"sim": "sim: this Mac only - scripts/run_sim.sh (the simulator, the bench, the OCS / SIM / judge pages); the Jetsons are not touched",
             "real": "real: run_ocs.sh here, run_usv.sh on the NX and run_agx.sh on the AGX over ssh"},
    "judge": {"venue": "DEMO (venue): the venue's rc-test on our team broker (VENUE_RC_HOST in network.env); protobuf, the console read-only",
              "auto": "auto: the course (ALPHA-DELTA) whose network a Mac adapter is on now (scripts/net_roles.sh status); never the venue DEMO",
              "alpha": "ALPHA: the course RoboCommand RC_ALPHA_HOST (network.lan.env; handbook .65.2:1883)",
              "bravo": "BRAVO: the course RoboCommand RC_BRAVO_HOST (handbook .66.2:1883)",
              "charlie": "CHARLIE: the course RoboCommand RC_CHARLIE_HOST (handbook .67.2:1883)",
              "delta": "DELTA: the course RoboCommand RC_DELTA_HOST (handbook .68.2:1883)",
              "course": "course: the RoboCommand at RC_HOST (network.lan.env) or the course NIC's DHCP gateway - the console read-only",
              "fake": "fake: our fake RoboCommand on the hub (JSON wire), RunStart 2 s after a valid declaration",
              "rctest": "rctest: RoboNation's test server on this Mac (1885, the protobuf wire) + its judge console"},
    "world": {"pool": "pool: the pool world (profiles/bench/world_pool.yaml)", "marina": "marina: the marina world (the event's layout)",
              "boxes": "boxes: the boxes world (m0t4)", "por": "por: the port world", "empty": "empty: no objects - the plant only",
              "bench": "bench: the small bench world"},
    "perception": {"camera": "camera: rendered frames through the students' measuring code (sim_observer -> cv_publish) to the map",
                   "geometric": "geometric: the fast fake perception (no camera path)"},
    "usv_ui": {"true": "on: usv_ui runs on the boat and holds the FCU link", "false": "off: no usv_ui (--no-ui); the mission manager holds the FCU link"},
    "mission": {"none": "none: no mission preselected - pick it in the OCS",
                "m4t0": "m4t0: T4 alone - LISTEN for the AssistanceRequest -> ACK -> TRANSIT -> READY",
                "m1t1": "m1t1: T1 Core - entry circle -> gates -> exit circle, SafePassageReport",
                "m1t2": "m1t2: T1 Advanced with the UAV's buoy map (declare uav on)",
                "scout": "scout: T1 as scout - colour-blind, buoy to buoy inside our course, under way",
                "m3t1": "m3t1: T3 docking + the water cannon",
                "m0t4": "m0t4: the boxes ground test",
                "m0t8": "m0t8: ground - the detection range", "m0t9": "m0t9: ground - the ZED guard on a stand"},
    "auto_mission": {"true": "on: RunStart -> 3-2-1 -> start_task by itself", "false": "off: RunStart says mission_ready, the boat stays in AUTO_HOLD"},
    "t4": {"true": "on: T4 declared TIER_CORE", "false": "off: T4 declared TIER_NONE"},
    "lucky": {"true": "on: the lucky fallback for m1t1 / m1t2 (see the row's tip) - a risk",
              "false": "off: the normal T1 (a gate needs both colours sure)"},
    "record": {"auto": "auto: the ZED recording from launch + the logging session at DECLARE", "off": "off: no recording"},
    "fcu": {"fake": "fake: the software FCU on the NX (no Pixhawk)", "uart": "uart: the real Pixhawk on /dev/ttyTHS1 (props off on the bench)",
            "usb": "usb: the real Pixhawk over USB (props off on the bench)", "eth": "eth: the real Pixhawk over ethernet (props off on the bench)"},
    "usv_profile": {"pool": "pool: run_usv.sh --pool (usv_pool.yaml)", "lab": "lab: run_usv.sh --lab - bench guards, never on the water",
                    "course": "course: run_usv.sh --course (usv_course.yaml, no GOTO)"},
    "cv": {"real": "real: cv_publish + the camera stream on the AGX", "fake": "fake: run_agx.sh --sim on the world file", "off": "off: the AGX is not started"},
    "usv_window": {"mac": "mac: the USV window served from this Mac", "nx": "nx: served from the boat (survives this Mac; the event's default)",
                   "off": "off: no USV window", "mac-classic": "mac-classic: the classic boat window, from this Mac",
                   "nx-classic": "nx-classic: the classic boat window, from the boat"},
    "uav_buoys": {"broker": "broker: the NX's broker bridges the UAV Orin's mosquitto (direct)",
                  "udp": "udp: their detector sends JSON to our adapter's 14605, relayed through the hub"},
    "free_run": {"off": "off: the FCU's own position (the default)", "on": "on: FREE RUN - ghost position, no motion to the FCU; never judge course"},
    "uav": {"off": "off: nothing UAV runs (the default of every preset)", "on": "on: the UAV adapter + the UAV window (8099)"},
    "net": {"ts": "ts: the tailnet (network.ts.env)", "lan": "lan: the event / pool switch (network.lan.env)", "manual": "manual: type the addresses below"},
    "ui": {"v2": "v2: the OCS page (default)", "v1": "v1: the classic page frozen at /v1/ (same state, same commands)"},
    "log": {"true": "on: LOG=1 - every MQTT message to SQLite", "false": "off: no SQLite log"},
}


def choice_tip(key: str, choice: Any) -> str:
    c = ("true" if choice else "false") if isinstance(choice, bool) else str(choice)
    return (CHOICE_TIPS.get(key) or {}).get(c) or ROW_TIPS.get(key) or f"{key}: {c}"
