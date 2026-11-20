"""system_actions: the restarts and reboots the OCS page's SYSTEM tab offers (v0.1.28, ROADMAP WP8a).

Each action: what it restarts / reboots, its options (drop-downs), a guard on the live state (nothing that touches the boat while
the vehicle is in AUTO, the OCS stack not during a run, the FCU and the Jetsons not while armed), the confirmation it needs (a
click, or typing the target's name for a reboot) and the command it runs - locally on the Mac (detached, so a restart of the OCS
stack survives the page's own server going down) or over ssh (ORIN_SSH / AGX_SSH from network*.env via the launcher's
environment, BatchMode: no password prompt can hang it). Every run is appended to logs/ocs/system_actions.jsonl.
The ADR-9 NEVER list stays: nothing here arms, disarms, sets a flight mode or sends actuator values.

v0.1.28.1: the page's SETTINGS tab adds the bench's sim scenario (sim_world.sh: the fake CV world, empty by default), the AGX
camera stream on / off (agx_cam.sh, the CAMERA tab's toggle) and quick stops - each needs `stop` typed (group "stop").
v0.1.40 (WP9b): the ZED recorder (agx_record.sh start LABEL | stop | status, SETTINGS' RECORDER card); an Option with `free` (a
regular expression) is a typed field (the label) instead of a drop-down - anything else falls back to its default.

OCS_WEB_DRYRUN=1: commands are recorded, not run (the tests; a rehearsal of the page).
"""
from __future__ import annotations

import json
import os
import re
import shlex
import subprocess
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
AUTO = ("AUTO_HOLD", "AUTO_TASK", "AUTO_SAFE_HOLD")
WORLDS = {"empty": "profiles/bench/world_empty.yaml", "pool": "profiles/bench/world_pool.yaml", "por": "profiles/bench/world_por.yaml", "bench": "profiles/bench/world_bench.yaml",
          "boxes": "profiles/bench/world_boxes.yaml"}


@dataclass
class Option:
    name: str
    label: str
    choices: List[str]
    default: str
    help: str = ""
    free: str = ""                    # v0.1.40: a typed value matching this regular expression (choices then = suggestions)

    def accept(self, value) -> str:
        if self.free:
            v = str(value or "").strip()
            return v if re.fullmatch(self.free, v) else self.default
        return value if value in self.choices else self.default


@dataclass
class Action:
    id: str
    host: str                         # mac | nx | agx | fcu
    title: str
    what: str
    options: List[Option] = field(default_factory=list)
    confirm: str = ""                 # "" = a click; a word = type it to confirm
    guard: Callable[[dict], str] = lambda s: ""
    build: Callable[[Dict[str, str], Dict[str, str]], List[str]] = lambda o, e: []
    group: str = "system"             # system = the SYSTEM tab (and SETTINGS); stop = SETTINGS' quick stops; settings = SETTINGS only
    opt_guard: Callable[[Dict[str, str], dict], str] = lambda o, s: ""      # a guard that depends on the chosen options
    detach: bool = False              # it stops this page's own server (the OCS stack): run in its own session

    def public(self, state: dict) -> dict:
        return {"id": self.id, "host": self.host, "title": self.title, "what": self.what, "confirm": self.confirm, "group": self.group,
                "options": [o.__dict__ for o in self.options], "refused": self.guard(state)}


# ---- guards --------------------------------------------------------------------------------------------------------
def _veh(s: dict) -> dict:
    return s.get("status") or {}


def _boat_guard(s: dict) -> str:
    v = _veh(s)
    if v.get("state") in AUTO:
        return f"the vehicle is {v['state']}: take it back first (ocs_cli manual, or stop in usv_ui)"
    if v.get("armed") and v.get("fcu_link") != "fake":
        link = "the real FCU" if v.get("fcu_link") == "real" else "the FCU (link unknown: no usv_ui beat)"
        return f"{link} is armed: disarm from the transmitter / usv_ui first"
    return ""


def _fcu_guard(s: dict) -> str:
    v = _veh(s)
    if v.get("state") in AUTO:
        return f"the vehicle is {v['state']}: take it back first"
    if v.get("armed"):
        return "the FCU is armed: disarm first (ArduPilot refuses a reboot while armed anyway)"
    return ""


def _agx_guard(s: dict) -> str:
    v = _veh(s)
    if v.get("state") == "AUTO_TASK":
        return "a task is running: the map would lose its perception (hold or abort first)"
    return ""


def _task_guard(s: dict) -> str:
    v = _veh(s)
    if v.get("state") == "AUTO_TASK":
        return f"a task is running ({v.get('task_name') or '?'}): abort first, then STOP EVERYTHING"
    return ""


def _ocs_guard(s: dict) -> str:
    r = (s.get("ocs") or {}).get("r")
    if r in ("R3", "R4", "R5"):
        return f"a run is declared / running ({r}): end_run or reset first - a restart loses the RunStart"
    return ""


# ---- commands --------------------------------------------------------------------------------------------------------
def _ssh(host: str, cmd: str) -> List[str]:
    return ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=6", host, "bash -lc " + shlex.quote(cmd)]


def _kit(env: Dict[str, str]) -> str:
    return env.get("JETSON_KIT") or "~/Desktop/dm3/MASTER_SCRIPTS"


def _net(o: Dict[str, str]) -> str:
    return {"ts": " --ts", "lan": " --lan"}.get(o.get("net", ""), "")


def _py() -> str:
    return 'PY=.venv/bin/python; [ -x "$PY" ] || PY=python3; '


def _local(cmd: str) -> List[str]:
    return ["bash", "-c", f"cd {shlex.quote(ROOT)} && {cmd}"]


def b_ocs(o, e):
    if o.get("do") == "stop":
        return _local("scripts/stop_ocs.sh")
    log = "LOG=1 " if o.get("log") == "on" else ""
    return _local(f"scripts/stop_ocs.sh; sleep 1; MAP=0 {log}scripts/run_ocs.sh --judge {o.get('judge', 'fake')}{_net(o)}")


def b_rctest(o, e):
    return _local("scripts/stop_rctest.sh" if o.get("do") == "stop" else "JUDGE_WEB=0 scripts/run_rctest.sh")


def b_nx(o, e):
    if o.get("do") == "stop":
        return _ssh(e.get("ORIN_SSH", "usv-pixhawk-ts"), f"cd {_kit(e)} && scripts/stop_usv.sh")
    log = "LOG=1 " if o.get("log") == "on" else ""
    return _ssh(e.get("ORIN_SSH", "usv-pixhawk-ts"),
                f"cd {_kit(e)} && scripts/stop_usv.sh; sleep 1; {log}scripts/run_usv.sh --fcu {o.get('fcu', 'fake')} --{o.get('profile', 'pool')}{_net(o)}")


def b_agx(o, e):
    if o.get("do") == "stop":
        return _ssh(e.get("AGX_SSH", "agxorin"), f"cd {_kit(e)} && scripts/stop_agx.sh")
    mode = o.get("mode", "real")
    args = {"real": "", "sim": f" --sim --world {WORLDS.get(o.get('world', 'pool'), WORLDS['pool'])}", "demo": " --demo"}[mode]
    cam = o.get("cam", "dir")
    args += " --no-cam" if cam == "off" else f" --cam-source {cam}"
    return _ssh(e.get("AGX_SSH", "agxorin"), f"cd {_kit(e)} && scripts/stop_agx.sh; sleep 1; scripts/run_agx.sh{args}{_net(o)}")


def b_simworld(o, e):
    return _local(f"scripts/sim_world.sh {o.get('world', 'empty')}")


def b_cam(o, e):
    if o.get("do") == "stop":
        return _ssh(e.get("AGX_SSH", "agxorin"), f"cd {_kit(e)} && scripts/agx_cam.sh stop")
    return _ssh(e.get("AGX_SSH", "agxorin"), f"cd {_kit(e)} && scripts/agx_cam.sh start --source {o.get('source', 'auto')}")


def b_observer(o, e):
    """v0.1.82 (Dmitrii: "add start directly observer to the OCS page"): OUR buoy observer on the AGX - scripts/agx_observer.sh (the
    launcher's observer ▶ / ■ runs the same script): cv/buoys_multi --headless in tmux agx:obs, the stream on its frames."""
    do = o.get("do", "status")
    det = (" --detector " + (o.get("detector") if o.get("detector") in ("model-a", "boundary-v14", "six-class") else "default")) if do == "start" else ""
    return _ssh(e.get("AGX_SSH", "agxorin"), f"cd {_kit(e)} && bash scripts/agx_observer.sh {do}{det}")


def _obs_opt_guard(o: Dict[str, str], s: dict) -> str:
    if o.get("do") == "stop" and _veh(s).get("state") == "AUTO_TASK":
        return "stopping the observer while a task runs leaves the boat blind (and the guard lost): hold / abort first"
    cur = (s.get("perception") or {}).get("observer_detector")          # v0.1.87: start with another model = a switch (~10 s blind)
    if o.get("do") == "start" and _veh(s).get("state") == "AUTO_TASK" and cur and cur != (o.get("detector") or "default"):
        return f"switching the model ({cur} -> {o.get('detector') or 'default'}) restarts the observer: not while a task runs - hold / abort first"
    return ""


def _cam_opt_guard(o: Dict[str, str], s: dict) -> str:
    if o.get("do") == "start" and o.get("source") == "zed" and _veh(s).get("state") == "AUTO_TASK":
        return "source zed takes the ZED from the observers while a task runs: dir, or hold / abort first"
    return ""


def b_rec(o, e):
    """v0.1.40 (WP9b): the ZED recorder on the AGX (scripts/agx_record.sh): start LABEL with its limits, stop, status."""
    do = o.get("do", "status")
    if do != "start":
        return _ssh(e.get("AGX_SSH", "agxorin"), f"cd {_kit(e)} && scripts/agx_record.sh {'stop' if do == 'stop' else 'status'}")
    args = f" --max-min {o.get('max_min', '10')}" + (" --replace" if o.get("replace") == "yes" else "") + \
        (" --no-pixhawk" if o.get("pixhawk") == "no" else "")
    return _ssh(e.get("AGX_SSH", "agxorin"), f"cd {_kit(e)} && scripts/agx_record.sh start {shlex.quote(o.get('label', 'event'))}{args}")


def _rec_opt_guard(o: Dict[str, str], s: dict) -> str:
    if _veh(s).get("state") == "AUTO_TASK" and (o.get("do") == "stop" or (o.get("do") == "start" and o.get("replace") == "yes")):
        return "the recorder IS the buoy observer: stopping / replacing it now drops the map's perception (status is fine; hold or abort first)"
    return ""


def b_fcu(o, e):
    return _ssh(e.get("ORIN_SSH", "usv-pixhawk-ts"), f"cd {_kit(e)} && {_py()}$PY -m optiprimex.tools.fcu_reboot --wait 20")


def b_reboot(which):
    def build(o, e):
        host = e.get("ORIN_SSH", "usv-pixhawk-ts") if which == "nx" else e.get("AGX_SSH", "agxorin")
        return _ssh(host, 'sudo -n /sbin/reboot || { echo "reboot needs passwordless sudo on this Jetson (a sudoers line: '
                          'the user ALL=(root) NOPASSWD: /sbin/reboot)"; exit 1; }')
    return build


DO = Option("do", "action", ["restart", "stop"], "restart", "restart = stop, then start with these options")
NET = Option("net", "network", ["profile", "ts", "lan"], "profile", "profile = NET_PROFILE of network.env; ts = tailnet, lan = pool switch")
LOGO = Option("log", "SQLite log", ["off", "on"], "off", "LOG=1: every MQTT message of that broker into SQLite")

ACTIONS: List[Action] = [
    Action("ocs", "mac", "OCS stack (this Mac)", "hub broker, OCS app, fake judge or the real wire, op-tui, the judge console, this page "
           "(it comes back by itself in a few seconds)",
           [DO, Option("judge", "judge", ["fake", "rctest", "course"], "fake",
                       "fake = our fake RoboCommand; rctest = RoboNation's test server (start it below first); course = the competition network"),
            NET, LOGO], "", _ocs_guard, b_ocs, detach=True),
    Action("rctest", "mac", "rc-test server (this Mac)", "RoboNation's own test server on mosquitto 1885 (scripts/run_rctest.sh)",
           [Option("do", "action", ["start", "stop"], "start")], "", lambda s: "", b_rctest),
    Action("nx", "nx", "Boat stack (Orin NX)", "boat broker, usv_ui (holds the FCU), the mission manager - choose the FCU and the profile",
           [DO, Option("fcu", "FCU", ["fake", "uart", "usb", "eth"], "fake",
                       "fake = the bench plant, no Pixhawk; uart = the real Pixhawk on /dev/ttyTHS1; usb = the FMU USB; eth = udp 14550"),
            Option("profile", "profile", ["pool", "lab", "course"], "pool",
                   "pool = usv_pool.yaml (real guards); lab = the bench with the real Pixhawk, props off, never at the pool; course = the competition"),
            NET, LOGO], "", _boat_guard, b_nx),
    Action("agx", "agx", "Perception stack (AGX)", "cv_publish (real detections) or the simulated camera, and the camera stream",
           [DO, Option("mode", "perception", ["real", "sim", "demo"], "real",
                       "real = the observers' detections; sim = fake_perception on a world file; demo = one synthetic buoy 8 m ahead"),
            Option("world", "world (sim)", list(WORLDS), "pool", "the world file for --sim"),
            Option("cam", "camera page", ["dir", "zed", "test", "off"], "dir",
                   "dir = the observers' preview frames; zed = the ZED itself (no observer can run then); test = a pattern"),
            NET], "", _agx_guard, b_agx),
    Action("fcu", "fcu", "Reboot the FCU (Pixhawk or the fake)", "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN through usv_ui's share on the NX "
           "(tools/fcu_reboot.py); refused when armed; usv_ui shows the link drop and reconnects", [], "fcu", _fcu_guard, b_fcu),
    Action("nx_reboot", "nx", "Reboot the Orin NX", "the whole boat computer (sudo -n reboot); ~1 min without the boat stack", [], "nx",
           _boat_guard, b_reboot("nx")),
    Action("agx_reboot", "agx", "Reboot the AGX", "the perception computer (sudo -n reboot)", [], "agx", _agx_guard, b_reboot("agx")),
    # v0.1.28.1, the SETTINGS tab
    Action("simworld", "mac", "Sim scenario (bench fake CV)", "restarts the laptop bench's fake_perception + fake_dock on a world "
           "(scripts/sim_world.sh); empty = nothing in view, off = no fake CV and no truth layer; the boat's map ages old objects out. "
           "On the Jetsons: the AGX card, perception sim + world",
           [Option("world", "world", ["empty", "bench", "pool", "por", "boxes", "off"], "empty",
                   "empty = no objects (the default since v0.1.28.1); bench = 3 gates + a dock; pool = the 50 x 25 m pool (Task 1 + dock); "
                   "por = the PoR course; boxes = the m0t4 boxes; off = stop the fake CV")], "", _agx_guard, b_simworld, group="settings"),
    Action("cam", "agx", "Camera stream (AGX)", "only the MJPEG stream (cv/stream, window cam of tmux agx; scripts/agx_cam.sh) - "
           "the detectors are not touched; the CAMERA tab's toggle uses this",
           [Option("do", "action", ["start", "stop"], "start"),
            Option("source", "source", ["auto", "dir", "zed", "test"], "auto",
                   "auto = zed when no observer runs, else dir (v0.1.28.2); dir = the observers' preview frames (BLANK until one runs); "
                   "zed = the ZED itself (it holds the camera: stop the stream before starting an observer); test = a pattern")],
           "", lambda s: "", b_cam, group="settings", opt_guard=_cam_opt_guard),
    Action("observer", "agx", "Buoy observer (AGX)", "v0.1.82: OUR observer cv/buoys_multi --headless (buoys + the ZED guard + the 3-8 m "
           "obstacles; scripts/agx_observer.sh, tmux agx window obs) - the camera stream shows its preview frames (a stream holding the ZED "
           "is restarted on them); needs the perception launch (run_agx.sh: cv_publish). The RUN tile's and the CAMERA card's OBSERVER button",
           [Option("do", "action", ["status", "start", "stop"], "status"),
            Option("detector", "detector", ["default", "model-a", "boundary-v14", "six-class"], "default", "default = our best.pt (cv/buoys/models); model-a = the "
                   "colleague's Model A (read-only from ~/buoy_robotx); boundary-v14 (v0.1.87) = their boundary-v14: coloured buoys + obstacles; six-class = their 11 Nov model (red / green / blue / buoy)")],
           "", lambda s: "", b_observer, group="settings", opt_guard=_obs_opt_guard),
    Action("rec", "agx", "ZED recorder (AGX)", "v0.1.40: our buoy observer with --record (scripts/agx_record.sh, tmux agxrec): it "
           "detects as usual and records ~/Desktop/dm3/recordings/<stamp>_<label>/ cam.svo2 + frames.jsonl + session.json; stops by "
           "itself at max min or below 5 GB free (status says so). Bring home: scripts/agx_pull_clips.sh on the Mac",
           [Option("do", "action", ["status", "start", "stop"], "status", "status = up / down, the newest session, why it stopped, "
                   "free disk; start = (re)start the observer recording; stop = Ctrl-C it (the SVO is closed; perception from it ends too)"),
            Option("label", "label", ["event"], "event", "the session's name: letters, digits, _ and - (e.g. day1_t1_gate, practice_dock); "
                   "anything else -> event", free=r"[A-Za-z0-9][A-Za-z0-9_-]{0,39}"),
            Option("max_min", "max min", ["10", "5", "20", "30", "60", "0"], "10", "stop recording after N minutes (0 = until stop); "
                   "detecting goes on"),
            Option("replace", "ZED in use", ["no", "yes"], "no", "yes = Ctrl-C the observer / ZED stream holding the camera first "
                   "(refused while a task runs); no = refused when the ZED is held"),
            Option("pixhawk", "telemetry", ["yes", "no"], "yes", "no = a camera-only bench clip (--no-pixhawk: no lat / lon / heading)")],
           "", lambda s: "", b_rec, group="settings", opt_guard=_rec_opt_guard),
    Action("simreset", "mac", "Reset the simulator (RESET SIM)", "v0.1.33: the boat to the world's start, the mockups back to the file, "
           "every fault off, the mode switch HOLD, then abort, confirm_safe, reset on both ends, clear_map and the fake judge's counters "
           "(scripts/sim_reset.sh; the SIM page's RESET SIM… is the same). The stacks stay up", [], "reset", lambda s: "",
           lambda o, e: _local(f"SIM_PORT={shlex.quote(e.get('SIM_PORT', '8097'))} scripts/sim_reset.sh"), group="settings"),
    # v0.1.37 (WP10d): STOP / RESET EVERYTHING - the top bar and SETTINGS; typed words; STOP refused while a task runs
    Action("stopall", "mac", "STOP EVERYTHING", "every stack down in order: the boat (usv_ui quit first: outputs zeroed, the FCU as the "
           "transmitter left it), perception + camera, the judge console, rc-test, the bench / simulator, the OCS last (scripts/stop_all.sh)",
           [], "stop", _task_guard, lambda o, e: _local("scripts/stop_all.sh"), group="everything", detach=True),
    Action("resetall", "mac", "RESET EVERYTHING", "a clean start, the stacks stay up: RESET SIM when a simulator answers, else reset on "
           "both ends + the fake judge's counters (scripts/reset_all.sh)", [], "reset", lambda s: "",
           lambda o, e: _local("scripts/reset_all.sh"), group="everything"),
    # quick stops: type `stop`; the boat and the AGX keep their guards (a stack stopped under a moving vehicle)
    Action("stop_bench", "mac", "Stop the bench", "scripts/stop_bench.sh: every node of the laptop bench (brokers, fake FCU, mm, judge, op-tui)",
           [], "stop", lambda s: "", lambda o, e: _local("scripts/stop_bench.sh"), group="stop"),
    Action("stop_simcv", "mac", "Stop the fake CV (bench)", "scripts/sim_world.sh off: no detections, the truth layer cleared",
           [], "stop", lambda s: "", lambda o, e: _local("scripts/sim_world.sh off"), group="stop"),
    Action("stop_nx", "nx", "Stop the boat stack (NX)", "scripts/stop_usv.sh over ssh: boat broker, usv_ui, the mission manager",
           [], "stop", _boat_guard, lambda o, e: _ssh(e.get("ORIN_SSH", "usv-pixhawk-ts"), f"cd {_kit(e)} && scripts/stop_usv.sh"), group="stop"),
    Action("stop_agx", "agx", "Stop perception (AGX)", "scripts/stop_agx.sh over ssh: cv_publish or the sim CV, the observers, the camera stream",
           [], "stop", _agx_guard, lambda o, e: _ssh(e.get("AGX_SSH", "agxorin"), f"cd {_kit(e)} && scripts/stop_agx.sh"), group="stop"),
    Action("stop_ocs", "mac", "Stop the OCS stack (this Mac)", "scripts/stop_ocs.sh: hub, OCS app, judge, op-tui - and this page",
           [], "stop", lambda s: "", lambda o, e: _local("scripts/stop_ocs.sh"), group="stop", detach=True),
]
BY_ID = {a.id: a for a in ACTIONS}


@dataclass
class Job:
    id: int
    action: str
    options: Dict[str, str]
    argv: List[str]
    t0: float
    lines: List[str] = field(default_factory=list)
    rc: Optional[int] = None
    log: Optional[str] = None          # v0.1.72 (WP28): logs/ocs/system_actions/<id>_<action>.log - the output as it streams
    t1: Optional[float] = None

    def public(self) -> dict:
        return {"id": self.id, "action": self.action, "options": self.options, "cmd": " ".join(shlex.quote(a) for a in self.argv),
                "t0": self.t0, "lines": self.lines[-200:], "rc": self.rc, "done": self.rc is not None, "log": self.log,
                "t1": self.t1, "failed": self.rc not in (None, 0), "tail20": self.lines[-20:] if self.rc not in (None, 0) else []}


class Runner:
    def __init__(self, env: Optional[Dict[str, str]] = None, log_path: Optional[str] = None, dry_run: Optional[bool] = None):
        self.env = dict(os.environ if env is None else env)
        self.dry = (os.environ.get("OCS_WEB_DRYRUN") == "1") if dry_run is None else dry_run
        self.log_path = log_path or os.path.join(ROOT, "logs", "ocs", "system_actions.jsonl")
        self.jobs: List[Job] = []
        self.lock = threading.Lock()

    def catalogue(self, state: dict) -> List[dict]:
        return [a.public(state) for a in ACTIONS]

    def run(self, action_id: str, options: Dict[str, str], confirm: str, state: dict) -> Dict[str, Any]:
        a = BY_ID.get(action_id)
        if a is None:
            return {"ok": False, "why": f"unknown action {action_id}"}
        why = a.guard(state)
        if why:
            return {"ok": False, "why": f"refused: {why}"}
        if a.confirm and confirm.strip().lower() != a.confirm:
            return {"ok": False, "why": f"type '{a.confirm}' to confirm"}
        opts = {o.name: o.accept(options.get(o.name)) for o in a.options}
        why = a.opt_guard(opts, state)
        if why:
            return {"ok": False, "why": f"refused: {why}"}
        argv = a.build(opts, self.env)
        with self.lock:
            job = Job(len(self.jobs) + 1, action_id, opts, argv, time.time())
            job.log = os.path.join(os.path.dirname(self.log_path), "system_actions", f"{time.strftime('%Y%m%d_%H%M%S')}_{job.id}_{action_id}.log")
            self.jobs.append(job)
        self._log({"t": job.t0, "action": action_id, "options": opts, "cmd": job.public()["cmd"], "dry_run": self.dry})
        if self.dry:
            job.lines.append("(dry run) " + job.public()["cmd"])
            job.rc = 0
            self._jlog(job, job.lines[-1], header=True)
        else:
            threading.Thread(target=self._exec, args=(job, a.detach), daemon=True).start()
        return {"ok": True, "job": job.id}

    def _jlog(self, job: Job, line: str, header: bool = False) -> None:
        """v0.1.72 (WP28): the job's own log file, line by line as it streams (the page's drawer shows the same lines)."""
        try:
            os.makedirs(os.path.dirname(job.log), exist_ok=True)
            with open(job.log, "a") as fh:
                if header:
                    fh.write(f"# job {job.id} {job.action} {json.dumps(job.options)} - {time.strftime('%Y-%m-%d %H:%M:%S')}\n# $ {job.public()['cmd']}\n")
                fh.write(line.rstrip() + "\n")
        except (OSError, TypeError):
            pass

    def _exec(self, job: Job, detach: bool) -> None:
        self._jlog(job, "started", header=True)
        try:
            if detach:            # the OCS stack restart kills this server: run it in its own session, its output to a file
                out = os.path.join(os.path.dirname(self.log_path), f"system_job_{job.id}.log")
                with open(out, "w") as fh:
                    subprocess.Popen(job.argv, stdout=fh, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, start_new_session=True, env=self.env)
                job.lines.append(f"started detached; output in {out}; the page reconnects when the stack is back")
                job.rc = 0
                return
            p = subprocess.Popen(job.argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL, text=True, env=self.env)
            for line in p.stdout:
                job.lines.append(line.rstrip())
                self._jlog(job, line)
            job.rc = p.wait(timeout=180)
        except Exception as exc:
            job.lines.append(f"failed: {exc}")
            job.rc = -1
        job.t1 = time.time()
        self._jlog(job, f"exit status {job.rc} after {job.t1 - job.t0:.1f} s")
        self._log({"t": time.time(), "job": job.id, "rc": job.rc, "tail": job.lines[-3:]})

    def _log(self, rec: dict) -> None:
        try:
            os.makedirs(os.path.dirname(self.log_path), exist_ok=True)
            with open(self.log_path, "a") as fh:
                fh.write(json.dumps(rec) + "\n")
        except OSError:
            pass

    def job_list(self) -> List[dict]:
        with self.lock:
            return [j.public() for j in self.jobs[-10:]]
