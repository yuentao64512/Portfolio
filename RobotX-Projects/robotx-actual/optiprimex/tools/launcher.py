#!/usr/bin/env python3
"""launcher: one script, one settings page, then every stack started in order (v0.1.38; scripts/launch.sh).

  scripts/launch.sh            -> http://127.0.0.1:8094/ : pick a preset (profiles/presets.yaml: simulation, lab, marina, custom), change
                                  any field, look at the plan (every command, per host), LAUNCH; the page streams each step's output and
                                  then links the OCS / SIM / judge pages. STOP ALL = scripts/stop_all.sh.

It composes today's launchers - nothing new runs on the hosts: sim = scripts/run_sim.sh on this Mac; real = scripts/run_ocs.sh here (+
run_rctest.sh for rc-test), then over ssh scripts/run_usv.sh on the NX and scripts/run_agx.sh on the AGX, with the flags the fields
choose. A changed rate (the heartbeat to RoboCommand, the boat's status / map rates) is written into a COPY of the profile under
logs/launch/<stamp>/ (copied to the NX's logs/launch/ when the boat needs it) and passed with OCS_CONFIG(_OVERRIDE) / MM_CONFIG_OVERRIDE /
USV_CONFIG - the profiles themselves are never edited. The addresses come from network.env / network.<ts|lan>.env (read here; the
plan on the page shows the ssh hosts it will use, the files themselves never leave the Mac), or from the page in `manual` mode.

  python -m optiprimex.tools.launcher [--port 8094] [--open] [--dry-run]   (--dry-run: LAUNCH shows the plan, runs nothing)
  python -m optiprimex.tools.launcher --status                              (what runs where, from the last launch; no page)

v0.1.39.2 - reconnect: every LAUNCH is remembered in logs/launch/last.json (the settings, the plan, the time); on start the launcher
looks for its tmux sessions on each host (here, and over ssh on the NX / AGX) and the page offers RECONNECT (adopt the running launch:
its links, its STOP, a per-host status every 5 s with `restart this host`) / STOP ALL / STOP, then start new. LAUNCH is refused while
something is up. Ctrl-C: the steps run in their own process groups (a Ctrl-C never cuts a launcher in half); during a launch the
current step finishes and the rest is skipped (a second Ctrl-C leaves at once); idle, it prints what is up and how to stop it.

v0.1.47 (WP10k-a, docs/UI_QOL_2026-11-07.md A): a copy button on every command; a title on every choice; the manual network's
placeholders read from network.ts.env / network.lan.env at runtime (shown on this local page only); PROGRESS = one line per process /
page the plan started (up / starting / died, from the 5 s Watch: tmux windows per host, the pages' HTTP answer); the five page links
with an up / down dot, probed every 5 s even before a launch; the theme: this page's select is the source of truth (LAUNCH_THEME, the
launch record, logs/launch/prefs.json) and is synced with the OCS page's /prefs (tools/prefs.py) - every page follows it.
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import signal
import socketserver
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from typing import Callable, Dict, List, Optional, Tuple

import yaml

from .prefs import THEMES, get_theme, norm as norm_theme, ocs_url, post_theme

KIT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
PAGE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "launcher.html")
PRESETS = os.path.join(KIT, "profiles", "presets.yaml")
REMOTE_KIT = "~/Desktop/dm3/MASTER_SCRIPTS"
WORLDS = ("pool", "marina", "boxes", "por", "empty", "bench")
FIELDS: Dict[str, Tuple] = {
    # v0.1.86 (WP42 §2.2, Dmitrii: "make Demo venue default choice, keep rctest as last option"): venue first = the page's default
    "mode": ("sim", "real"), "judge": ("venue", "auto", "alpha", "bravo", "charlie", "delta", "course", "fake", "rctest"), "net": ("ts", "lan", "manual"), "fcu": ("fake", "uart", "usb", "eth"),
    "usv_profile": ("pool", "lab", "course"), "usv_window": ("mac", "nx", "off", "mac-classic", "nx-classic"), "cv": ("real", "fake", "off"), "perception": ("camera", "geometric"), "world": WORLDS,
    "ui": ("v2", "v1"),                              # v0.1.43: the OCS page - v2, or the classic one frozen at /v1/ (LAUNCH_UI sets the default)
    "uav": ("off", "on"),                            # v0.1.65 (D15): the UAV adapter + the UAV window (8099); every preset off. NOT the "live
}                                                    # without the UAV" switch - `declare uav` (off by default) is (D6/D7)
# v0.1.68 (WP17 F7): FREE RUN - only the lab and custom presets carry the field (sim and event do not); on = the NX's
# run_usv.sh --fcu <real> --lab --free-run --world <world>, the AGX's fake CV on the same world; never with judge course
OPTIONAL_FIELDS: Dict[str, Tuple] = {"free_run": ("off", "on"),
                                     "uav_buoys": ("broker", "udp"),     # v0.1.69 (WP25): the UAV's buoy path when UAV on (default broker)
                                     # v0.1.86 (WP42 §2.1): the mission preselected at launch, the ZED recording at DECLARE
                                     "mission": ("none", "m4t0", "m1t1", "m1t2", "scout", "m3t1", "m0t4", "m0t8", "m0t9"),
                                     "record": ("auto", "off")}
JUDGE_LABEL = {"venue": "venue = DEMO rc-test VLAN 70", "alpha": "ALPHA = course RoboCommand .2:1883", "bravo": "BRAVO = course RoboCommand .2:1883",
               "charlie": "CHARLIE = course RoboCommand .2:1883", "delta": "DELTA = course RoboCommand .2:1883",
               "course": "course = RC_HOST typed / network.lan.env", "fake": "fake = ours", "rctest": "rctest = RoboNation's test server on this Mac"}
TIER_TASKS = ("T1", "T2", "T3", "T4")
TIER_WORDS = ("NONE", "CORE", "ADVANCED", "DISRUPTIVE")
FREE_RUN_LINE = "FREE RUN (ghost position, no motion to the FCU)"
# v0.1.74 (WP30): usv_window mac-classic / nx-classic = the v0.1.68 boat window (/v1/) at / (run_boat_window.sh --classic)
RATE_KEYS = {"heartbeat_hz": (0.5, 5.0), "status_hz": (0.5, 10.0), "map_hz": (0.2, 10.0)}
MANUAL_KEYS = ("ocs_host", "boat_host", "agx_host", "nx_ssh", "agx_ssh", "rc_host",
               "uav_host", "uav_listen_port", "uav_bridge_port", "uav_broker_host", "uav_broker_port", "uav_buoy_path")   # v0.1.65: the UAV's
RECORD = os.path.join(KIT, "logs", "launch", "last.json")
PREFS_FILE = os.path.join(KIT, "logs", "launch", "prefs.json")   # v0.1.47: the launcher's theme between runs (logs/: never archived)
MANUAL_ENV = {"ocs_host": "OCS_HOST", "boat_host": "BOAT_HOST", "agx_host": "AGX_HOST", "nx_ssh": "ORIN_SSH", "agx_ssh": "AGX_SSH",
              "rc_host": "RC_HOST", "uav_host": "UAV_HOST", "uav_listen_port": "UAV_LISTEN_PORT", "uav_bridge_port": "UAV_BRIDGE_PORT",
              "uav_broker_host": "UAV_BROKER_HOST", "uav_broker_port": "UAV_BROKER_PORT", "uav_buoy_path": "UAV_BUOY_PATH"}
MANUAL_EXAMPLE = {"ocs_host": "e.g. 100.x.y.z (ts) · 192.168.1.x (lan) - this Mac, as the boat reaches it",
                  "boat_host": "e.g. 100.x.y.z (ts) · 192.168.1.x (lan) - the NX (its broker 1884)",
                  "agx_host": "e.g. 100.x.y.z (ts) · 192.168.1.x (lan) - the AGX",
                  "nx_ssh": "e.g. usv-pixhawk-ts (an ssh alias) or user@host",
                  "agx_ssh": "e.g. agxorin (an ssh alias) or user@host",
                  "rc_host": "empty / auto = the course NIC's DHCP gateway (v0.1.59); an address = the override - judge course only",
                  "uav_host": "e.g. 192.168.1.x - the UAV team's Orin (its listener 14600), or their laptop's relay",
                  "uav_listen_port": "14600 - the Orin's geofence listener",
                  "uav_bridge_port": "14605 - our adapter's UDP port (the listener answers it)",
                  "uav_broker_host": "e.g. 192.168.1.x - the mosquitto on the UAV Orin (v0.1.66: the buoys)",
                  "uav_broker_port": "1883 - the UAV Orin's broker",
                  "uav_buoy_path": "direct | relay | off (v0.1.66: how the UAV's buoys reach the boat)"}
KNOWN_LOCAL = ("optiprimex", "ocs", "rctest", "judge")        # the Mac's sessions of our launchers (a stack this launcher did not start)
SSH = "ssh -o ConnectTimeout=4 -o BatchMode=yes"


def load_presets(path: Optional[str] = None) -> Dict[str, dict]:
    path = path or os.environ.get("LAUNCH_PRESETS") or PRESETS    # v0.1.86: LAUNCH_PRESETS = another presets file (the tests' v0.1.85 copy)
    with open(path) as f:
        P = yaml.safe_load(f) or {}
    ui = os.environ.get("LAUNCH_UI")
    for c in P.values():                             # v0.1.43: an older presets file without `ui`, or LAUNCH_UI, decides the default
        c["ui"] = ui if ui in FIELDS["ui"] else c.get("ui", "v2")
        if "free_run" in c:                          # v0.1.68: YAML reads a bare on / off as a boolean
            c["free_run"] = _onoff(c["free_run"])
    return P


def _onoff(v) -> str:
    return "on" if v in (True, "on", "true", 1) else "off" if v in (False, "off", "false", 0, None) else str(v)


def free_run_on(cfg: dict) -> bool:
    return _onoff(cfg.get("free_run")) == "on"


def read_net(profile: str) -> Dict[str, str]:
    """network.env + network.<profile>.env (KEY=VALUE; the launchers' own sources). Read only; never sent to the page."""
    out: Dict[str, str] = {}
    for name in ("network.env", f"network.{profile}.env"):
        p = os.path.join(KIT, name)
        if not os.path.exists(p):
            continue
        for line in open(p):
            line = line.split("#", 1)[0].strip()
            if "=" in line:
                k, v = line.split("=", 1)
                out[k.strip()] = v.strip().strip('"').strip("'")
    return out


# v0.1.78.3: the launcher's NETWORK reference table - every address our launchers use (and the devices kept for reference), per
# profile, with a ping now. Read from the network files at each request; the values go to this local page only (never a log, never a record).
NET_TABLE = (("this Mac (the OCS, hub 1882)", "OCS_HOST"), ("Orin NX - the boat (broker 1884, ssh)", "BOAT_HOST"),
             ("AGX Orin - perception (ssh)", "AGX_HOST"), ("UAV computer (uav-pixhawk, the UAV team's listener)", "UAV_HOST"),
             ("the switch / router", "SWITCH_HOST"), ("radios (Ubiquiti, reference)", "RADIO_HOSTS"),
             ("course RoboCommand (judge course)", "RC_HOST"), ("venue rc-test - our team broker (judge venue)", "VENUE_RC_HOST"),
             ("venue rc-test - its page", "VENUE_RC_WEB"))


def _addr_of(v: str) -> str:
    v = v.split("://", 1)[-1].split("/", 1)[0]
    return v.split("@", 1)[-1].rsplit(":", 1)[0] if v.count(":") == 1 else v.split("@", 1)[-1]


def network_table(kit: Optional[str] = None, ping: Optional[Callable[[str], bool]] = None, auth=None, observer=None) -> dict:
    """{"active": NET_PROFILE, "rows": [{role, key, base, ts, lan, port, hosts: [{addr, up}]}]} - ping None = the system's ping, 1 s."""
    kit = kit or KIT
    def one(name):
        out = {}
        p = os.path.join(kit, name)
        if os.path.exists(p):
            for line in open(p):
                line = line.split("#", 1)[0].strip()
                if line.startswith("export "):
                    line = line[7:].strip()
                if "=" in line:
                    k, v = line.split("=", 1)
                    out[k.strip()] = v.strip().strip('"').strip("'")
        return out
    base, per = one("network.env"), {p: one(f"network.{p}.env") for p in ("ts", "lan")}
    active = os.environ.get("NET_PROFILE") or base.get("NET_PROFILE") or "ts"
    if ping is None:
        try:                                                           # this Mac's own addresses: on an interface = up (it may not answer its own ping)
            local = set(re.findall(r"inet (\d+\.\d+\.\d+\.\d+)", subprocess.run(["ifconfig"], capture_output=True, text=True, timeout=3).stdout))
        except Exception:
            local = set()
        def ping(a):
            if a in local:
                return True
            try:
                return subprocess.run(["ping", "-c", "1", "-t", "1", a], capture_output=True, timeout=3).returncode == 0
            except Exception:
                return False
    rows, addrs = [], set()
    for role, key in NET_TABLE:
        r = {"role": role, "key": key, "base": base.get(key, ""), "ts": per["ts"].get(key, ""), "lan": per["lan"].get(key, ""),
             "port": base.get("VENUE_RC_PORT", "") if key == "VENUE_RC_HOST" else ""}
        vals = [r["lan"] or r["base"], r["ts"] or r["base"]] if active == "lan" else [r["ts"] or r["base"], r["lan"] or r["base"]]
        r["hosts"] = [{"addr": a} for a in dict.fromkeys(_addr_of(x) for v in vals if v for x in v.replace(",", " ").split())]
        addrs.update(h["addr"] for h in r["hosts"])
        rows.append(r)
    with ThreadPoolExecutor(max_workers=16) as ex:
        up = dict(zip(sorted(addrs), ex.map(ping, sorted(addrs))))
    for r in rows:
        for h in r["hosts"]:
            h["up"] = up.get(h["addr"], False)
    # v0.1.78.4: the Jetsons' ssh targets (the active profile's ORIN_SSH / AGX_SSH) and whether a key logs in from this laptop
    prof = per.get(active, {})
    for r in rows:
        sk = {"BOAT_HOST": ("ORIN_SSH", "nx"), "AGX_HOST": ("AGX_SSH", "agx"), "UAV_HOST": ("UAV_SSH", "uav")}.get(r["key"])   # v0.1.78.12: + the UAV
        if not sk:
            continue
        tgt = prof.get(sk[0]) or base.get(sk[0]) or ""
        r["ssh"] = {"which": sk[1], "target": tgt, "auth": None}
        if sk[1] == "nx":
            r["pin"] = usv_window_pin({**base, **prof})                    # v0.1.78.9: the USV window's PIN, on the NX's row
        if tgt and r["hosts"] and r["hosts"][0]["up"] and auth is not False:
            r["ssh"]["auth"] = (auth or ssh_auth)(tgt)
            if sk[1] == "agx" and r["ssh"]["auth"] == "key":                  # v0.1.78.7: the observer's state
                r["observer"] = (observer or agx_observer)("status", tgt)
    return {"active": active, "rows": rows}


def ssh_target(which: str, kit: Optional[str] = None) -> str:
    """nx | agx | uav -> the active profile's ORIN_SSH / AGX_SSH / UAV_SSH (from the files, never from the page)."""
    t = network_table(kit, ping=lambda a: False, auth=False)
    for r in t["rows"]:
        if (r.get("ssh") or {}).get("which") == which:
            return r["ssh"]["target"]
    return ""


def net_placeholders(kit: Optional[str] = None) -> Dict[str, str]:
    """v0.1.47 (A3): the manual network's placeholders from network.ts.env / network.lan.env, read now (the files are never archived and
    never leave this Mac: the values go to the local page only, never to a log). A key in neither file -> the format example."""
    kit = kit or KIT
    per: Dict[str, Dict[str, str]] = {}
    for prof in ("ts", "lan"):
        p = os.path.join(kit, f"network.{prof}.env")
        if not os.path.exists(p):
            continue
        vals: Dict[str, str] = {}
        for line in open(p):
            line = line.split("#", 1)[0].strip()
            if line.startswith("export "):
                line = line[7:].strip()
            if "=" in line:
                k, v = line.split("=", 1)
                vals[k.strip()] = v.strip().strip('"').strip("'")
        per[prof] = vals
    out = {}
    for key, var in MANUAL_ENV.items():
        seen = [f"{per[p][var]} ({'course' if key == 'rc_host' and p == 'lan' else p})" for p in ("ts", "lan") if per.get(p, {}).get(var)]
        out[key] = " · ".join(seen) if seen else MANUAL_EXAMPLE[key]
    return out


def validate(cfg: dict) -> List[str]:
    errs = []
    for k, choices in FIELDS.items():
        if cfg.get(k) not in choices:
            errs.append(f"{k}: one of {', '.join(choices)}")
    for k, (lo, hi) in RATE_KEYS.items():
        try:
            v = float(cfg.get(k))
            if not lo <= v <= hi:
                errs.append(f"{k}: {lo} .. {hi} Hz")
        except (TypeError, ValueError):
            errs.append(f"{k}: a number")
    if cfg.get("mode") == "sim" and cfg.get("judge") in ("course",) + COURSES:
        errs.append(f"judge {cfg.get('judge')} needs the real network: in simulation use fake, rctest or venue (DEMO)")
    for k in ("mission", "record"):                  # v0.1.86 (WP42 §2.1)
        if cfg.get(k) not in (None, "") and cfg.get(k) not in OPTIONAL_FIELDS[k]:
            errs.append(f"{k}: one of {', '.join(OPTIONAL_FIELDS[k])}")
    if "auto_mission" in cfg and cfg.get("auto_mission") not in (True, False):
        errs.append("auto_mission: true | false")
    if "t4" in cfg and cfg.get("t4") not in (True, False):           # v0.1.88: the T4 tick box
        errs.append("t4: true | false")
    if "lucky" in cfg and cfg.get("lucky") not in (True, False):     # v0.1.90: the lucky tick box
        errs.append("lucky: true | false")
    from ..ocs import cmd_authority as _CA                            # v0.1.92: the mission modifiers (absent = the boat's profile)
    for k in _CA.MOD_ARGS:
        if cfg.get(k) not in (None, "", "profile") and _CA.mod_value(k, cfg.get(k))[0]:
            errs.append(_CA.mod_value(k, cfg.get(k))[0])
    t = cfg.get("tiers")
    if t not in (None, "", "profile"):
        if not isinstance(t, dict) or any(str(k).upper() not in TIER_TASKS or str(v).upper().replace("TIER_", "") not in TIER_WORDS for k, v in t.items()):
            errs.append("tiers: profile, or {T1..T4: NONE | CORE | ADVANCED | DISRUPTIVE}")
    if cfg.get("uav_buoys") not in (None, "") and cfg.get("uav_buoys") not in OPTIONAL_FIELDS["uav_buoys"]:
        errs.append("uav_buoys: one of broker, udp")
    if "free_run" in cfg and _onoff(cfg.get("free_run")) not in OPTIONAL_FIELDS["free_run"]:
        errs.append("free_run: one of off, on")
    elif free_run_on(cfg):                           # v0.1.68 (WP17 F7)
        if cfg.get("mode") != "real":
            errs.append("free run: real mode only (the simulator has its own plant)")
        if cfg.get("usv_profile") != "lab":
            errs.append("free run: the lab profile only (profile lab)")
        if cfg.get("fcu") == "fake":
            errs.append("free run: a real FCU (uart / usb / eth) - the fake plant already has a position")
    if cfg.get("net") == "manual" and cfg.get("mode") == "real":
        for k in ("ocs_host", "boat_host", "nx_ssh") + (("agx_ssh",) if cfg.get("cv") != "off" else ()):
            if not str(cfg.get(k) or "").strip():
                errs.append(f"manual network: {k} needed")
    if cfg.get("theme") not in (None, "") and cfg.get("theme") not in THEMES:
        errs.append(f"theme: one of {', '.join(THEMES)}")
    for k in MANUAL_KEYS:
        v = str(cfg.get(k) or "")
        if v and not re.match(r"^[A-Za-z0-9._@:-]{1,80}$", v):
            errs.append(f"{k}: letters, digits, . _ @ : - only")
    return errs


def _set_rate(text: str, key: str, value: float) -> str:
    """Replace `key: <number>` keeping its comment and indentation (the first one: rates: / ocs:)."""
    new, n = re.subn(rf"^(\s*{key}:\s*)[0-9.]+", lambda m: f"{m.group(1)}{value:g}", text, count=1, flags=re.M)
    return new if n else text


def _rate(base_text: str, key: str) -> Optional[float]:
    m = re.search(rf"^\s*{key}:\s*([0-9.]+)", base_text, flags=re.M)
    return float(m.group(1)) if m else None


def _copy_with_rates(base: str, rates: Dict[str, float]) -> Optional[str]:
    """The base profile's text with the changed rates, or None when nothing differs."""
    text = open(os.path.join(KIT, base)).read()
    changed = False
    for k, v in rates.items():
        cur = _rate(text, k)
        if cur is not None and abs(cur - float(v)) > 1e-9:
            text, changed = _set_rate(text, k, float(v)), True
    if not changed:
        return None
    return f"# GENERATED by scripts/launch.sh from {base} (rates changed on the launcher page) - the profile itself is unchanged\n" + text


# v0.1.47 (A5): rc-test, then its judge page - started when 8095 does not answer as the page wired to 1885 (JUDGE_OPEN=0: no browser tab;
# the launcher's links open it). run_rctest.sh starts the page itself; the second half covers "rc-test already up" without its page.
RCTEST_STEP = ("if lsof -nP -iTCP:${RCTEST_PORT:-1885} -sTCP:LISTEN >/dev/null 2>&1; then echo 'rc-test already up'; "
               "curl -sf -m 2 http://127.0.0.1:${JUDGE_WEB_PORT:-8095}/status.json | grep -qE '\"broker_port\": '\"${RCTEST_PORT:-1885}\"'[,}]' "
               "&& echo 'its judge page answers (wired to rc-test)' || scripts/run_judge.sh --port ${RCTEST_PORT:-1885} --as-judge --owner rctest; "
               "else JUDGE_OPEN=0 scripts/run_rctest.sh; fi")


# ---- v0.1.59 (A5 / A6, docs/ROBOCOMMAND_COMPLIANCE_2026-11-08.md #16a and fix 7): the course NIC and the clocks -------------------
RC_SUBNET = re.compile(r"^192\.168\.6[5-8]\.\d{1,3}$")         # the four courses' RoboCommand networks (DHCP, gateway .2)
# v0.1.82 (WP39, docs/SCOUT_BRIEF_2026-11-11.md §5.3): the course dropdown - ALPHA .65, BRAVO .66, CHARLIE .67, DELTA .68 (handbook 3.4.2);
# each course's RoboCommand address = RC_<COURSE>_HOST / _PORT in network.lan.env (never in code); DEMO = the venue rc-test (VLAN 70)
COURSES = ("alpha", "bravo", "charlie", "delta")
COURSE_NET = {"alpha": "192.168.65.", "bravo": "192.168.66.", "charlie": "192.168.67.", "delta": "192.168.68."}
COURSE_TOPIC = "robocommand/robotx/course"
CLOCK_MAX_S = 5.0                                                 # above this the launch is refused (cmd_max_age_s is 10 s, no NTP there)
CLOCK_FIX = "the GUIDE §1 date line: ssh <host> sudo date -u -s \"'$(date -u '+%Y-%m-%d %H:%M:%S')'\""


def _cmd(argv: List[str], timeout: float = 6.0) -> Tuple[int, str]:
    try:
        r = subprocess.run(argv, capture_output=True, text=True, timeout=timeout)
        return r.returncode, r.stdout
    except Exception as e:                                        # not found / timed out: no answer
        return 255, str(e)


def course_nic(run=_cmd) -> dict:
    """The RoboCommand-facing interface on this Mac: the one with a 192.168.65-68.x address. Its DHCP packet gives the lease and
    the router (= RoboCommand, the course's .2). {} when no interface is in a course subnet."""
    rc, out = run(["ifconfig", "-l"])
    for ifc in (out.split() if rc == 0 else []):
        rc, addr = run(["ipconfig", "getifaddr", ifc])
        addr = addr.strip()
        if rc != 0 or not RC_SUBNET.match(addr):
            continue
        rc, pkt = run(["ipconfig", "getpacket", ifc])
        dhcp = rc == 0 and "yiaddr" in pkt
        m = re.search(r"router \(ip_mult\): \{([0-9.]+)", pkt or "") if dhcp else None
        gw = m.group(1) if m else None
        if gw is None:                                           # no router in the packet: the default route, if it is this NIC
            rc, rt = run(["route", "-n", "get", "default"])
            mi, mg = re.search(r"interface: (\S+)", rt or ""), re.search(r"gateway: ([0-9.]+)", rt or "")
            gw = mg.group(1) if rc == 0 and mi and mg and mi.group(1) == ifc else None
        return {"iface": ifc, "addr": addr, "gateway": gw, "dhcp": dhcp}
    return {}


def resolve_rc_host(cfg: dict, ne: Dict[str, str], nic: Optional[dict]) -> Tuple[Optional[str], Optional[str]]:
    """judge course: (RC_HOST, refusal). A set address (the manual field, or network.lan.env's RC_HOST) is the manual override;
    empty / `auto` = the course NIC's DHCP gateway, and no lease there refuses the launch."""
    manual = str((cfg.get("rc_host") if cfg.get("net") == "manual" else None) or ne.get("RC_HOST") or "").strip()
    if manual and manual.lower() != "auto":
        return manual, None
    if not nic:
        return None, ("course NIC: no interface with a 192.168.65-68.x address - plug the RoboCommand cable (DHCP on, bridging / "
                      "sharing off), or set RC_HOST in network.lan.env")
    if not nic.get("dhcp"):
        return None, f"course NIC {nic['iface']} ({nic['addr']}): no DHCP lease - the event NIC must be a DHCP client"
    if not nic.get("gateway"):
        return None, f"course NIC {nic['iface']} ({nic['addr']}): DHCP lease without a gateway - RoboCommand's address unknown"
    return nic["gateway"], None


def clock_offset(host: str, run=_cmd, now=time.time) -> Optional[float]:
    """ssh <host> date +%s minus this Mac's clock (s); None when the host does not answer."""
    t0 = now()
    rc, out = run(SSH.split() + [host, "date +%s"])
    t1 = now()
    try:
        return float(out.strip().splitlines()[-1]) - (t0 + t1) / 2.0 if rc == 0 else None
    except (ValueError, IndexError):
        return None


SSH_TARGET_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*(@[A-Za-z0-9][A-Za-z0-9._-]*)?$")      # no leading "-" (an ssh option)


def ssh_auth(target: str, probe=None) -> str:
    """v0.1.78.4: 'key' (a key logs in, BatchMode), 'password' (the host answers but asks for a password / refuses the key - another
    laptop), 'down' (no answer / bad target). probe(argv) -> (rc, stderr) for the tests."""
    if not SSH_TARGET_RE.match(target or ""):
        return "down"
    if probe is None:
        def probe(argv):
            try:
                r = subprocess.run(argv, capture_output=True, text=True, timeout=8)
                return r.returncode, r.stderr
            except Exception as e:
                return 255, str(e)
    rc, err = probe(SSH.split() + ["-o", "StrictHostKeyChecking=accept-new", target, "true"])
    if rc == 0:
        return "key"
    return "password" if re.search(r"Permission denied|password|publickey|keyboard-interactive", err or "", re.I) else "down"


def ssh_terminal_cmd(target: str, action: str = "open") -> str:
    """The shell line a Terminal window runs: an interactive ssh (a password prompt is fine), or the key copy (ssh-keygen only when this
    laptop has no ed25519 key - it asks for a passphrase there, in the open; then ssh-copy-id asks the Jetson's password once)."""
    if not SSH_TARGET_RE.match(target or ""):
        raise ValueError(f"not an ssh target: {target!r}")
    if action == "copy-id":
        return ("[ -f ~/.ssh/id_ed25519.pub ] || ssh-keygen -t ed25519 -f ~/.ssh/id_ed25519; "
                f"ssh-copy-id -i ~/.ssh/id_ed25519.pub {target} && echo 'key copied: the launcher logs in without a password now'")
    return f"ssh -o ConnectTimeout=6 {target}"


def open_terminal(line: str, run=subprocess.run) -> Tuple[bool, str]:
    """macOS: a new Terminal window running `line` (the launcher listens on 127.0.0.1: the window opens on this laptop)."""
    if sys.platform != "darwin":
        return False, "not macOS: run it yourself - " + line
    esc = line.replace("\\", "\\\\").replace('"', '\\"')
    r = run(["osascript", "-e", f'tell application "Terminal" to do script "{esc}"', "-e", 'tell application "Terminal" to activate'],
            capture_output=True, text=True, timeout=10)
    return r.returncode == 0, (r.stderr or "").strip()


# v0.1.78.7: the AGX buoy observer from the launcher, as window `obs` of the launch's tmux `agx` (stop_agx.sh stops it with the rest).
# v0.1.82: OUR observer (cv/buoys_multi: buoys + the ZED guard + the 3-8 m obstacle clusters - the students' cv/buoys one has no guard,
# so the pool / course profiles' legs would crawl at 0.3 m/s) through scripts/agx_observer.sh (the OCS page's OBSERVER button runs the
# same script): a stream holding the ZED is restarted on the observer's frames (dir), any other holder refused. Over ssh with the key.
OBS_PAT = "multi_buoy_observe[r]|actual_buoy_observe[r]"
OBSERVER_CMDS = {
    "status": "cd {kit} && bash scripts/agx_observer.sh status",
    "start": "cd {kit} && bash scripts/agx_observer.sh start --detector {detector}",
    "stop": "cd {kit} && bash scripts/agx_observer.sh stop",
}


def observer_cmd(action: str, kit: str = "~/Desktop/dm3/MASTER_SCRIPTS", detector: str = "default") -> str:
    if action not in OBSERVER_CMDS:
        raise ValueError("observer: start | stop | status")
    if detector not in ("default", "model-a", "boundary-v14", "six-class"):
        raise ValueError("observer --detector default | model-a | boundary-v14 | six-class")
    return OBSERVER_CMDS[action].replace("{kit}", kit).replace("{detector}", detector)


def agx_observer(action: str, target: str, run=None, detector: str = "default") -> dict:
    """{ok, state: up|down|?, text} - the observer on the AGX over ssh (key login)."""
    if not SSH_TARGET_RE.match(target or ""):
        return {"ok": False, "state": "?", "text": "no AGX_SSH in the network files"}
    run = run or (lambda argv: subprocess.run(argv, capture_output=True, text=True, timeout=30))
    try:
        r = run(SSH.split() + [target, observer_cmd(action, detector=detector)])
        out = ((r.stdout or "") + (r.stderr or "")).strip().splitlines()
    except Exception as e:
        return {"ok": False, "state": "?", "text": f"ssh: {e}"}
    text = out[-1] if out else f"ssh exit {getattr(r, 'returncode', '?')}"
    state = "up" if text.startswith("observer: up") or "still up" in text else "down" if text.startswith("observer: down") else "?"
    return {"ok": r.returncode == 0, "state": state, "text": text}


def ocs_host_check(cfg: dict, run=_cmd, net_env: Optional[Dict[str, str]] = None) -> Optional[str]:
    """v0.1.90.10 (venue 12 Nov, 18:20: the Mac's USB LAN fell back to DHCP - .6 instead of OCS_HOST .13 - the boat's bridge said "No
    route to host", no boat status, no USV heartbeat on the RC page): a refusal text when this Mac does not hold OCS_HOST (the address
    the boat bridges to), with the fix; None when it does (or the check does not apply: the simulator, no IPv4 OCS_HOST)."""
    if (cfg or {}).get("mode") != "real":
        return None
    net = cfg.get("net")
    ne = net_env if net_env is not None else (read_net(net) if net in ("ts", "lan") else {})
    want = str((cfg.get("ocs_host") if net == "manual" else None) or ne.get("OCS_HOST") or "").strip()
    if not re.fullmatch(r"\d{1,3}(\.\d{1,3}){3}", want) or want.startswith("127."):
        return None
    rc, out = run(["ifconfig"])
    if rc != 0:
        return None
    have = re.findall(r"\binet (\d{1,3}(?:\.\d{1,3}){3})\b", out or "")
    if want in have:
        return None
    pre = want.rsplit(".", 1)[0] + "."
    near = [a for a in have if a.startswith(pre)]
    ifc = None
    if near:
        blocks = re.split(r"\n(?=\S)", out or "")
        ifc = next((b.split(":", 1)[0] for b in blocks if f"inet {near[0]} " in b), None)
    svc = None
    if ifc:
        rc2, order = run(["networksetup", "-listnetworkserviceorder"])
        m = re.search(r"\(\d+\) ([^\n]+)\n\(Hardware Port: [^,]*, Device: " + re.escape(ifc) + r"\)", order or "") if rc2 == 0 else None
        svc = m.group(1).strip() if m else None
    fix = (f'sudo networksetup -setmanual "{svc}" {want} 255.255.255.0' if svc else
           f"give the team-LAN adapter its manual address {want} (System Settings -> Network), or `networksetup -listnetworkserviceorder`")
    return (f"network: this Mac does not hold OCS_HOST {want} - the boat bridges to it (no boat status, no USV heartbeat on the RC page)"
            + (f"; the team-LAN adapter {ifc} has {near[0]} (DHCP?)" if near else "; no address in that network at all (the cable?)")
            + f" - fix: {fix}")


def preflight(cfg: dict, hosts: Dict[str, str], run=_cmd, now=time.time, nic=None) -> dict:
    """Before a real launch: the clocks of the Jetsons the plan uses (|offset| > CLOCK_MAX_S refuses), the course NIC (judge course).
    An unreachable host is not refused here: the plan's `reach the ...` step shows it (as before)."""
    out = {"clocks": {}, "nic": None, "refuse": []}
    if cfg.get("mode") != "real" or os.environ.get("OPTIPRIMEX_PREFLIGHT", "").lower() in ("off", "0"):   # the test suite: no ssh
        return out
    for name, h in hosts.items():
        off = clock_offset(h, run=run, now=now)
        bad = off is not None and abs(off) > CLOCK_MAX_S
        out["clocks"][name] = {"host": h, "offset_s": None if off is None else round(off, 1), "ok": None if off is None else not bad}
        if bad:
            out["refuse"].append(f"clock: {name} ({h}) is {off:+.0f} s from this Mac (> {CLOCK_MAX_S:.0f} s: every command would be "
                                 f"REJECTED stale) - fix: {CLOCK_FIX.replace('<host>', h)}")
        if off is None and ssh_auth(h) == "password":                  # v0.1.78.4: another laptop - the plan's ssh steps cannot type a password
            out["clocks"][name]["auth"] = "password"
            out["refuse"].append(f"ssh: {name} ({h}) asks this laptop for a password - the launch's steps log in without one: the network card "
                                 f"-> copy my key ↗ (asks the password once), or ssh ↗ to log in by hand")
    why = ocs_host_check(cfg, run=run)                              # v0.1.90.10: the address the boat bridges to
    if why:
        out["refuse"].append(why)
    if cfg.get("judge") in ("course",) + COURSES:
        out["nic"] = nic if nic is not None else course_nic(run)
    return out


def nic_course(nic: Optional[dict]) -> Optional[str]:
    """v0.1.82: which course network this Mac is on (from the course NIC's DHCP address), else None."""
    a = str((nic or {}).get("addr") or "")
    return next((c for c, pre in COURSE_NET.items() if a.startswith(pre)), None)


def tcp_up(host: str, port: int, timeout: float = 0.3) -> bool:
    import socket
    try:
        with socket.create_connection((host, int(port)), timeout=timeout):
            return True
    except (OSError, ValueError):
        return False


def ocs_running(run=_cmd) -> bool:
    rc, _ = run(["pgrep", "-f", "optiprimex.ocs.ocs_ap[p]"])
    return rc == 0


def peek_course(host: str, port: int, wait_s: float = 1.5) -> Optional[dict]:
    """v0.1.82 (WP39): ONE short MQTT connection - subscribe robocommand/robotx/course, take the retained RxCourse, disconnect. Only when
    the OCS is not running (the handbook's one OCS connection, 3.4.5): the caller checks. None = nothing retained / no broker."""
    import threading
    try:
        import paho.mqtt.client as mqtt
        from ..ocs import rc_wire
    except Exception:
        return None
    got, ev = {}, threading.Event()

    def on_msg(c, u, m):
        try:
            got.update(rc_wire.decode_course(m.payload))
        except Exception:
            got["error"] = "not an RxCourse"
        ev.set()
    try:
        c = mqtt.Client(client_id=f"sgit-launcher-check-{os.getpid()}")
    except TypeError:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=f"sgit-launcher-check-{os.getpid()}")
    c.on_message = on_msg
    c.on_connect = lambda cl, u, f, rc, *a: cl.subscribe(COURSE_TOPIC, qos=0)
    try:
        c.connect(host, int(port), keepalive=5)
        c.loop_start()
        ev.wait(wait_s)
    except Exception:
        return None
    finally:
        try:
            c.loop_stop()
            c.disconnect()
        except Exception:
            pass
    return got or None


def course_check(ne: Dict[str, str], nic: Optional[dict] = None, peek: bool = True, run=_cmd, up=tcp_up, peeker=peek_course) -> dict:
    """v0.1.82 (WP39): the RoboCommand network checker - per course: its address (network.lan.env), on this network (the NIC's DHCP
    address in its subnet), broker up (TCP 300 ms, only the course this Mac is on: the networks are isolated) and, with the OCS NOT
    running, the retained RxCourse (course_id, corners, pinger); DEMO = the venue rc-test (VENUE_RC_* in network.env)."""
    nic = course_nic(run) if nic is None else nic
    here = nic_course(nic)
    ocs = ocs_running(run)
    rows = []
    vb = {**read_net("-"), **ne}
    for c in COURSES + ("demo",):
        if c == "demo":
            host, port = vb.get("VENUE_RC_HOST"), vb.get("VENUE_RC_PORT")
            on = None                                                    # VLAN 70: no subnet rule of ours - the probe says
        else:
            host, port = ne.get(f"RC_{c.upper()}_HOST"), ne.get(f"RC_{c.upper()}_PORT") or "1883"
            on = c == here
        r = {"course": c.upper(), "host": host, "port": port, "on_network": on, "broker": None, "rx_course": None, "note": ""}
        if not host:
            r["note"] = f"RC_{c.upper()}_HOST not set in network.lan.env" if c != "demo" else "VENUE_RC_HOST not set in network.env"
        elif on is False:
            r["note"] = "not on this network"
        else:
            r["broker"] = up(host, int(port))
            if r["broker"] and peek and not ocs:
                rx = peeker(host, int(port))
                r["rx_course"] = None if not rx else {"course_id": rx.get("course_id"), "corners": len(rx.get("boundary") or []),
                                                      "pinger_hz": rx.get("pinger_hz"), "error": rx.get("error")}
            elif r["broker"] and ocs:
                r["note"] = "the OCS holds the connection: its RoboCommand card shows the course (no probe)"
            r["note"] = r["note"] or ("● on this network · broker up" if r["broker"] else "broker not reachable")
        rows.append(r)
    return {"nic": nic or None, "on": here.upper() if here else None, "ocs_running": ocs, "rows": rows}


# ---- v0.1.86 (WP42 §2.2 + §6 item 6): what answers - the judges' RoboCommand and our three Jetsons, a dot each -------------------------
REACH_JETSONS = (("agx", "AGX", "AGX_HOST"), ("usv_nx", "USV-NX", "BOAT_HOST"), ("uav_nx", "UAV-NX", "UAV_SSH"))
_REACH: Dict[str, Tuple[float, dict]] = {}


def _host_of(v: Optional[str]) -> Optional[str]:
    """'user@host' / 'host:port' / an alias -> the host part (an ssh alias without a dot stays as typed: the resolver may know it)."""
    if not v:
        return None
    v = str(v).strip().split("@")[-1]
    return v.split(":")[0] or None


def reach(ne: Optional[Dict[str, str]] = None, up=None, ttl: float = 10.0, now=time.time) -> dict:
    """{judges: {venue|alpha|..: True | False | None}, jetsons: {agx|usv_nx|uav_nx: {label, up}}, reachable: [names]} - one TCP connect
    each (0.5 s; RoboCommand on its port, a Jetson on ssh :22), cached ttl s; addresses from the env files and never in the answer
    (None = no address set). No auto-switching: the page only colours the options."""
    key = "reach"
    hit = _REACH.get(key)
    if hit and now() - hit[0] < ttl and up is None:
        return hit[1]
    up = up or (lambda h, p: tcp_up(h, p, timeout=0.5))
    prof = os.environ.get("NET_PROFILE") or read_net("-").get("NET_PROFILE") or "lan"
    ne = ne if ne is not None else {**read_net("lan"), **read_net(prof if prof in ("ts", "lan") else "lan")}
    vb = {**read_net("-"), **ne}
    targets = {"venue": (vb.get("VENUE_RC_HOST"), vb.get("VENUE_RC_PORT") or "1883")}
    for c in COURSES:
        targets[c] = (ne.get(f"RC_{c.upper()}_HOST"), ne.get(f"RC_{c.upper()}_PORT") or "1883")
    judges, jet = {}, {}
    work = [(("j", k), h, int(p)) for k, (h, p) in targets.items() if h] + \
           [(("h", k), _host_of(vb.get(var)), 22) for k, _l, var in REACH_JETSONS if _host_of(vb.get(var))]
    res: Dict[tuple, bool] = {}
    ths = [threading.Thread(target=lambda w=w: res.__setitem__(w[0], bool(up(w[1], w[2]))), daemon=True) for w in work]
    for t in ths:
        t.start()
    for t in ths:
        t.join(1.0)
    for k in targets:
        judges[k] = res.get(("j", k)) if targets[k][0] else None
    for k, label, var in REACH_JETSONS:
        jet[k] = {"label": label, "up": res.get(("h", k)) if _host_of(vb.get(var)) else None}
    out = {"judges": judges, "jetsons": jet, "reachable": [k for k, v in judges.items() if v],
           "line": "RoboCommand reachable: " + (", ".join(k for k, v in judges.items() if v) or "none")}
    _REACH[key] = (now(), out)
    return out


_PRE_CACHE: Dict[str, Tuple[float, dict]] = {}


def preflight_cached(cfg: dict, hosts: Dict[str, str], ttl: float = 10.0, **kw) -> dict:
    """/plan is asked on every field change: the ssh round-trips at most every `ttl` s per (mode, judge, hosts)."""
    key = json.dumps([cfg.get("mode"), cfg.get("judge"), sorted(hosts.items())])
    hit = _PRE_CACHE.get(key)
    if hit and time.time() - hit[0] < ttl:
        return hit[1]
    pre = preflight(cfg, hosts, **kw)
    _PRE_CACHE[key] = (time.time(), pre)
    return pre


def plan_hosts(cfg: dict, net_env: Optional[Dict[str, str]] = None) -> Dict[str, str]:
    """The Jetsons a real plan uses (the same rule as build_plan)."""
    if cfg.get("mode") != "real":
        return {}
    net = cfg.get("net")
    ne = net_env if net_env is not None else (read_net(net) if net in ("ts", "lan") else {})
    nx = (cfg.get("nx_ssh") if net == "manual" else None) or ne.get("ORIN_SSH") or "usv-pixhawk-ts"
    agx = (cfg.get("agx_ssh") if net == "manual" else None) or ne.get("AGX_SSH") or "agxorin"
    return {"nx": nx, **({} if cfg.get("cv") == "off" else {"agx": agx})}


def plan_procs(cfg: dict, nx: Optional[str] = None, agx: Optional[str] = None) -> List[dict]:
    """v0.1.47 (A4): what the plan starts, one line each on PROGRESS - tmux windows per host (session:window) and the pages."""
    out: List[dict] = []

    def tm(host, ssh, sess, win, label):
        out.append({"key": f"{host}:{sess}:{win}", "kind": "tmux", "host": host, "ssh": ssh, "session": sess, "window": win, "label": label})

    def pg(name):
        out.append({"key": f"page:{name}", "kind": "page", "host": "mac", "page": name, "label": name})

    rct = cfg.get("judge") == "rctest"
    if rct:
        tm("mac", None, "rctest", "broker", "rc-test broker (1885)")
        tm("mac", None, "rctest", "server", "RoboNation's test_server.py")
    if cfg.get("mode") == "sim":
        for w, label in (("hub", "hub broker (1882)"), ("boat", "boat broker (1884)"), ("fcu", "fake FCU (the simulated plant)"), ("mm", "mission manager"),
                         ("fp", "fake perception"), ("dock", "fake dock"), ("rc", "fake judge"), ("ocs", "OCS app"), ("sim", "the simulator")):
            if w == "rc" and rct:
                continue
            tm("mac", None, "optiprimex", w, label)
        if cfg.get("usv_ui"):
            tm("mac", None, "optiprimex", "ui", "usv_ui (the simulated boat)")
        if cfg.get("perception") == "camera":
            tm("mac", None, "optiprimex", "obs", "sim_observer (the rendered camera)")
            tm("mac", None, "optiprimex", "cvpub", "cv_publish")
        if rct:
            tm("mac", None, "judge", "tui", "rc-tui (the judge console, tmux judge)")
        if cfg.get("uav") == "on":                   # v0.1.65 (D15): uav:adapter, uav:web (+ their two programs in the sim)
            tm("mac", None, "optiprimex", "copter", "the UAV team's fake copter")
            tm("mac", None, "optiprimex", "uavlst", "the UAV team's geofence listener")
            tm("mac", None, "optiprimex", "uav", "uav:adapter - our UAV adapter")
            tm("mac", None, "uavwin", "web", "uav:web - the UAV window (8099)")
        for name in ("OCS page", "judge page", "SIM page", "USV window") + (("UAV window",) if cfg.get("uav") == "on" else ()):
            pg(name)
        return out
    tm("mac", None, "ocs", "hub", "hub broker (1882)")
    if cfg.get("judge") == "fake":
        tm("mac", None, "ocs", "rc", "fake judge")
    tm("mac", None, "ocs", "ocs", "OCS app")
    if cfg.get("log"):
        tm("mac", None, "ocs", "log", "SQLite logger (hub)")
    tm("mac", None, "judge", "tui", "rc-tui (the judge console, tmux judge)")
    if cfg.get("uav") == "on":                       # v0.1.65 (D15)
        tm("mac", None, "ocs", "uav", "uav:adapter - our UAV adapter")
        tm("mac", None, "uavwin", "web", "uav:web - the UAV window (8099)")
    pg("OCS page")
    pg("judge page")
    if cfg.get("uav") == "on":
        pg("UAV window")
    tm("nx", nx, "usv", "boat", "boat broker (1884)")
    if cfg.get("fcu") == "fake":
        tm("nx", nx, "usv", "fcu", "fake FCU")
    if cfg.get("usv_ui", True):
        tm("nx", nx, "usv", "ui", "usv_ui (holds the FCU link)")
    tm("nx", nx, "usv", "mm", "mission manager")
    tm("nx", nx, "usv", "host", "host health (NX)")
    if (cfg.get("usv_window") or "").startswith("nx"):    # v0.1.63; v0.1.74: nx-classic too
        tm("nx", nx, "bwin", "web", "USV window (team LAN)")
    elif (cfg.get("usv_window") or "mac").startswith("mac"):
        tm("mac", None, "bwin", "web", "USV window (team LAN)")
    if cfg.get("cv") != "off":
        tm("agx", agx, "agx", "cv", "fake perception" if cfg.get("cv") == "fake" else "cv_publish")
        if cfg.get("cv") == "fake":
            tm("agx", agx, "agx", "dock", "fake dock")
        tm("agx", agx, "agx", "host", "host health (AGX)")
    return out


def _venue_judge(ne: Dict[str, str], refuse: List[str], notes: List[str], links: Dict[str, str], sim: bool = False) -> None:
    """v0.1.78.1 / .2: the venue's rc-test (RoboNation's practice server, our team broker on it) - real or simulated boat."""
    vb = {**read_net("-"), **ne}                                       # network.env carries VENUE_RC_* whatever the profile
    if not vb.get("VENUE_RC_HOST") or not vb.get("VENUE_RC_PORT"):
        refuse.append("judge venue: VENUE_RC_HOST / VENUE_RC_PORT not set in network.env (the venue rc-test's team broker)")
        return
    notes.insert(0, f"venue rc-test: team broker {vb['VENUE_RC_HOST']}:{vb['VENUE_RC_PORT']} (protobuf) · the console read-only"
                 + (" · the SIMULATED boat is declared and reported to it under our team" if sim else "")
                 + (f" · its page {vb['VENUE_RC_WEB']}" if vb.get("VENUE_RC_WEB") else ""))
    if vb.get("VENUE_RC_WEB"):
        links["venue RoboCommand page"] = vb["VENUE_RC_WEB"]


def record_mode(cfg: dict) -> str:
    """v0.1.86: the ZED recording at DECLARE - the preset's record (default auto), forced off when the CV is not real (or in sim)."""
    if cfg.get("record") is None:
        return "legacy"                              # a v0.1.85 config without the field: the observer as before, no launch block
    if cfg.get("mode") == "sim" or cfg.get("cv") != "real":
        return "off"
    return "off" if cfg.get("record") == "off" else "auto"


def launch_block(cfg: dict) -> Optional[dict]:
    """v0.1.86 (WP42 §2.1): what the OCS reads at start from its profile copy (`launch:`) - the preset, the mission preselected (code ->
    the wire task + tier through the playbook), auto (RunStart -> 3-2-1 -> start_task), the tier row, record. None = nothing to add."""
    code = str(cfg.get("mission") or "none")
    tiers = dict(cfg["tiers"]) if isinstance(cfg.get("tiers"), dict) else {}
    tiers = {str(k).upper(): v for k, v in tiers.items()}
    if "t4" in cfg:                                                  # v0.1.88 (Dmitrii: "tick box for T4, off by default for every mission"):
        tiers["T4"] = "CORE" if cfg.get("t4") is True else "NONE"     # every preset carries t4: false; a config without the field = as before
    if code == "none" and not tiers and record_mode(cfg) in ("off", "legacy"):
        return None                                  # nothing for the OCS to do at start (the record-off bench presets: as before)
    rm = record_mode(cfg)
    out = {"preset": str(cfg.get("preset") or "custom"), "mission": None if code == "none" else code, "auto": False,
           "record": "auto" if rm == "legacy" else rm}
    if code != "none":
        from . import playbook
        r = playbook.resolve(code)
        if r:
            out["task"] = r[1]
            out["extra"] = {k: v for k, v in (("tier", r[2]),) if v}
            if cfg.get("lucky") is True and r[1] == "T1":           # v0.1.90: m1t1 / m1t2 take uncertain-colour pairs as gates
                out["extra"]["lucky"] = True
            if r[1] == "T1":                                         # v0.1.92: the mission modifiers (the subpanel's keys) - set = sent
                from ..ocs import cmd_authority as _CA
                out["extra"].update(_CA.mod_args({k: v for k, v in cfg.items() if k in _CA.MOD_ARGS and v != "profile"})[1])
            if r[1] in ("T1", "SCOUT"):                              # v0.1.90.4 (venue 17:08: preset m1t2-both, mission changed to m1t1,
                tiers["T1"] = "ADVANCED" if str(r[2] or "").lower() == "advanced" else "CORE"   # the row kept ADVANCED): T1 = the mission
            if code.startswith("m1t2") and cfg.get("uav") == "on":   # v0.1.90.4: an m1t2 launch declares the UAV (16:40-17:06: "m1t2
                out["declare_uav"] = True                            # needs declare uav on" - five refusals)
            out["auto"] = bool(cfg.get("auto_mission"))
    if tiers:
        out["tiers"] = {str(k).upper(): "TIER_" + str(v).upper().replace("TIER_", "") for k, v in tiers.items()}
    return out


def _ocs_copy(base: str, rates: Dict[str, float], launch: Optional[dict]) -> Optional[str]:
    """The OCS profile copy: the changed rates (as before) + the `launch:` block (v0.1.86); None when neither applies."""
    t = _copy_with_rates(base, rates)
    if launch is None:
        return t
    text = t if t is not None else (f"# GENERATED by scripts/launch.sh from {base} (the preset's launch block) - the profile itself is unchanged\n"
                                    + open(os.path.join(KIT, base)).read())
    return text.rstrip("\n") + "\n\n# v0.1.86 (WP42): the launcher's preset - the OCS preselects the mission, declares this tier row, records at DECLARE\nlaunch: " \
        + json.dumps(launch) + "\n"


def build_plan(cfg: dict, stamp: Optional[str] = None, net_env: Optional[Dict[str, str]] = None, pre: Optional[dict] = None) -> dict:
    """cfg (a preset + the page's changes) -> {steps: [{host, label, cmd}], files: [{path, text}], remote_files: [{host, path, text}],
    links, notes}. Pure: nothing runs here."""
    stamp = stamp or time.strftime("%Y%m%d_%H%M%S")
    q = shlex.quote
    steps: List[dict] = []
    files: List[dict] = []
    remote_files: List[dict] = []
    notes: List[str] = []
    rates_ocs = {"heartbeat_hz": float(cfg["heartbeat_hz"])}
    rates_usv = {"status_hz": float(cfg["status_hz"]), "map_hz": float(cfg["map_hz"])}
    gen_dir = f"logs/launch/{stamp}"
    log = "LOG=1 " if cfg.get("log") else ""
    links = {"OCS page": "http://127.0.0.1:8096/" + ("v1/" if cfg.get("ui") == "v1" else ""), "judge page": "http://127.0.0.1:8095/"}

    if cfg["mode"] == "sim":
        ocs_base = "profiles/course/ocs_course.yaml" if cfg["judge"] in ("rctest", "venue") else "profiles/bench/ocs_bench.yaml"
        sim_refuse: List[str] = []
        if cfg["judge"] == "venue":                                   # v0.1.78.2: the simulated boat against the venue rc-test
            _venue_judge(net_env if net_env is not None else {}, sim_refuse, notes, links, sim=True)
        usv_base = "usv_ui/config/usv_bench_share.yaml" if cfg.get("usv_ui") else "profiles/bench/usv_bench.yaml"
        env = []
        t = _ocs_copy(ocs_base, rates_ocs, launch_block(cfg))
        if t:
            files.append({"path": f"{gen_dir}/ocs.yaml", "text": t})
            env.append(f"OCS_CONFIG={gen_dir}/ocs.yaml")
        t = _copy_with_rates(usv_base, rates_usv)
        if t:
            files.append({"path": f"{gen_dir}/usv.yaml", "text": t})
            env.append(f"USV_CONFIG={gen_dir}/usv.yaml")
        if cfg.get("preset"):
            env.append(f"LAUNCH_PRESET={q(str(cfg['preset']))}")         # v0.1.42: usv_ui's top bar names the preset
        if cfg.get("theme") in THEMES:
            env.append(f"LAUNCH_THEME={cfg['theme']}")                     # v0.1.47: the launcher's theme (A6)
        args = [cfg["world"], "--no-open", "--judge", cfg["judge"], "--perception", cfg["perception"]] + ([] if cfg.get("usv_ui") else ["--no-ui"])
        if cfg.get("uav") == "on":                   # v0.1.65 (D15): the UAV team's fake copter + listener, our adapter, the UAV window
            args.append("--uav")
            links["UAV window"] = "http://127.0.0.1:8099/"
            notes.append("UAV: their fake copter + listener (uav/, unmodified) + our adapter + the UAV window (8099); `declare uav on` declares it")
        steps.append({"host": "mac", "label": "the simulator (run_sim.sh: the bench, the SIM page, the OCS page, the judge page)",
                      "cmd": " ".join(env + ["scripts/run_sim.sh"] + [q(a) for a in args])})
        links["SIM page"] = "http://127.0.0.1:8097/"
        notes.append("everything on this Mac; the Jetsons are not touched")
        if log:
            notes.append("log: the simulator keeps its own run folder (logs/bench/run_*); the SQLite logger is for the real stacks")
        stop = "scripts/stop_judge.sh; scripts/stop_rctest.sh; scripts/stop_bench.sh; scripts/stop_ocs.sh"
        ops = {"mac": {"ssh": None, "expect": ["optiprimex"] + (["rctest"] if cfg["judge"] == "rctest" else []), "stop": stop, "steps": [0]}}
        return {"steps": steps, "files": files, "remote_files": remote_files, "links": links, "notes": notes, "stamp": stamp, "stop": stop,
                "ops": ops, "procs": plan_procs(cfg), "refuse": sim_refuse}

    # ---- real: three hosts --------------------------------------------------------------------------------------------------
    net = cfg["net"]
    ne = net_env if net_env is not None else (read_net(net) if net in ("ts", "lan") else {})
    nx = (cfg.get("nx_ssh") if net == "manual" else None) or ne.get("ORIN_SSH") or "usv-pixhawk-ts"
    agx = (cfg.get("agx_ssh") if net == "manual" else None) or ne.get("AGX_SSH") or "agxorin"
    netflag = [f"--{net}"] if net in ("ts", "lan") else []
    host_env = []
    if net == "manual":
        for k, var in (("ocs_host", "OCS_HOST"), ("boat_host", "BOAT_HOST"), ("agx_host", "AGX_HOST")):
            if cfg.get(k):
                host_env.append(f"{var}={q(str(cfg[k]))}")
    ocs_base = "profiles/pool/ocs_pool.yaml" if cfg["judge"] == "fake" else "profiles/course/ocs_course.yaml"
    env = list(host_env)
    lb = launch_block(cfg)
    if lb and lb.get("mission"):
        notes.insert(0, f"mission {lb['mission']} preselected at launch" + (" - starts by itself at RunStart (3-2-1)" if lb.get("auto") else
                     " - RunStart raises mission_ready, the boat stays in AUTO_HOLD") + (f"; tiers {lb['tiers']}" if lb.get("tiers") else ""))
    t = _ocs_copy(ocs_base, rates_ocs, lb)
    if t:
        files.append({"path": f"{gen_dir}/ocs.yaml", "text": t})
        env.append(f"OCS_CONFIG_OVERRIDE={gen_dir}/ocs.yaml")
    pre = pre or {"clocks": {}, "nic": None, "refuse": []}
    refuse = list(pre.get("refuse") or [])
    free = free_run_on(cfg)                                            # v0.1.68 (WP17 F7)
    if free and cfg["judge"] in ("course", "venue") + COURSES:
        refuse.append(f"FREE RUN with judge {cfg['judge']}: refused - free run is a bench mode (a ghost boat never goes to RoboNation's judge)")
    if cfg["judge"] == "venue":
        _venue_judge(ne, refuse, notes, links)
    if cfg["judge"] in COURSES:                                        # v0.1.82 (WP39): the course dropdown - its address from network.lan.env
        c = cfg["judge"]
        host = ne.get(f"RC_{c.upper()}_HOST")
        nic = pre.get("nic") or {}
        if not host:
            refuse.append(f"judge {c.upper()}: RC_{c.upper()}_HOST not set in network.lan.env (handbook 3.4.2: {COURSE_NET[c]}2)")
        here = nic_course(nic) if nic else None
        notes.insert(0, f"course {c.upper()}: RoboCommand {host}:{ne.get(f'RC_{c.upper()}_PORT') or 1883} · "
                     + (f"this Mac on {here.upper()} ({nic.get('addr')})" if here else "this Mac on no course network")
                     + (f" - CAUTION: selected {c.upper()}, the cable is on {here.upper()}" if here and here != c else "")
                     + ("" if here else " - CAUTION: plug the course cable (DHCP) before the run"))
    if cfg["judge"] == "course":                                       # v0.1.59 (A5): RC_HOST = the override, or the course NIC's gateway
        rc_host, why = resolve_rc_host(cfg, ne, pre.get("nic"))
        nic = pre.get("nic") or {}
        if why:
            refuse.append(why)
        else:
            env.append(f"RC_HOST={q(str(rc_host))}")
        notes.insert(0, (f"course NIC: {nic.get('addr')} ({nic.get('iface')}) · gateway {nic.get('gateway') or '-'} · "
                         f"{'DHCP' if nic.get('dhcp') else 'NO DHCP lease'}" if nic else "course NIC: none in 192.168.65-68.x")
                     + (f" · RoboCommand {rc_host}" + (" (the RC_HOST override)" if rc_host != nic.get("gateway") else " (auto)") if rc_host else ""))
    if cfg.get("theme") in THEMES:
        env.append(f"LAUNCH_THEME={cfg['theme']}")                         # v0.1.47: the launcher's theme (A6)
    ssh = "ssh -o ConnectTimeout=6 -o BatchMode=yes"
    for h, name in ((nx, "NX"), (agx, "AGX")):
        if name == "AGX" and cfg["cv"] == "off":
            continue
        steps.append({"host": "mac", "label": f"reach the {name} ({h}): who is there, what runs (look before starting)",
                      "cmd": f"{ssh} {q(h)} 'hostname; who | head -3; tmux ls 2>/dev/null || echo no tmux sessions'"})
        ck = (pre.get("clocks") or {}).get(name.lower()) or {}
        off = ck.get("offset_s")
        steps.append({"host": "mac", "label": f"clock check {name}: " + ("no answer yet" if off is None else f"{off:+.1f} s from this Mac")
                      + ("" if ck.get("ok") in (None, True) else f" - REFUSED (> {CLOCK_MAX_S:.0f} s); fix: {CLOCK_FIX.replace('<host>', h)}"),
                      "clock": ck or None,
                      "cmd": (f"o=$(( $({ssh} {q(h)} date +%s) - $(date +%s) )); echo \"{name} clock offset $o s\"; "
                              f"[ ${{o#-}} -le {int(CLOCK_MAX_S)} ] || {{ echo 'more than {int(CLOCK_MAX_S)} s: fix the clock first'; exit 1; }}")})
    if cfg["judge"] == "rctest":
        steps.append({"host": "mac", "label": "rc-test (RoboNation's test server on 1885, the judge console + the judge page wired to 1885)",
                      "cmd": RCTEST_STEP})
    uav_on = cfg.get("uav") == "on"                   # v0.1.65 (D15): the adapter (window uav) + the UAV window (tmux uavwin)
    uav_env = []
    if uav_on and net == "manual":
        for k in ("uav_host", "uav_listen_port", "uav_bridge_port", "uav_broker_host", "uav_broker_port", "uav_buoy_path"):
            if cfg.get(k):
                uav_env.append(f"{MANUAL_ENV[k]}={q(str(cfg[k]))}")
    steps.append({"host": "mac", "label": f"the OCS stack (hub, OCS app, judge {cfg['judge']}, op-tui, the OCS page)" + (" + the UAV adapter and window" if uav_on else ""),
                  "cmd": " ".join(["MAP=0"] + ([log.strip()] if log else []) + env + uav_env + ["scripts/run_ocs.sh"] + netflag + ["--judge", cfg["judge"]] + (["--ui", "v1"] if cfg.get("ui") == "v1" else []) + (["--uav"] if uav_on else []))})
    if uav_on:
        links["UAV window"] = "http://127.0.0.1:8099/"
        notes.append("UAV: our adapter to UAV_HOST (network.lan.env names; their listener on 14600, ours 14605) + the UAV window (8099); "
                     "their robocommand_bridge.py stays OFF; `declare uav on` declares it")
    uwc = ["--classic"] if (cfg.get("usv_window") or "").endswith("-classic") else []   # v0.1.74 (WP30): the preset forces the classic window
    if (cfg.get("usv_window") or "mac").startswith("mac"):  # v0.1.63: with the Mac's other steps (it reconnects to the boat broker)
        steps.append({"host": "mac", "label": f"the USV window on this Mac - team LAN (OCS_HOST), one commander, PIN {usv_window_pin(ne)}",
                      "cmd": " ".join(["scripts/run_boat_window.sh"] + netflag + uwc)})
    usv_base = {"pool": "profiles/pool/usv_pool.yaml", "lab": "profiles/pool/usv_lab.yaml", "course": "profiles/course/usv_course.yaml"}[cfg["usv_profile"]]
    nx_env = []
    t = _copy_with_rates(usv_base, rates_usv)
    if t:
        rpath = f"logs/launch/usv_{stamp}.yaml"
        remote_files.append({"host": nx, "path": rpath, "text": t})
        nx_env.append(f"MM_CONFIG_OVERRIDE={rpath}")
    if net == "manual" and cfg.get("ocs_host"):
        nx_env.append(f"OCS_HOST={q(str(cfg['ocs_host']))}")
    if log:
        nx_env.append("LOG=1")
    if cfg.get("preset"):
        nx_env.append(f"LAUNCH_PRESET={q(str(cfg['preset']))}")
    if uav_on:                                          # v0.1.69: broker = the NX bridges their mosquitto (direct); udp = the adapter relays
        nx_env.append("UAV_BUOY_PATH=" + ("relay" if cfg.get("uav_buoys") == "udp" else "direct"))
    world_file = f"profiles/bench/world_{cfg['world']}.yaml"
    fr_args = ["--free-run", "--world", world_file] if free else []
    usv_cmd = " ".join(nx_env + ["scripts/run_usv.sh", "--fcu", cfg["fcu"], f"--{cfg['usv_profile']}"] + fr_args + netflag + ([] if cfg.get("usv_ui", True) else ["--no-ui"]))
    if free:
        notes.insert(0, FREE_RUN_LINE + f" - the ghost starts at {world_file}'s start; the AGX runs the fake CV on the same world; "
                        "the FCU gets mode requests only; bench only, props off, ESC power off, NEVER at the pool")
    steps.append({"host": "nx", "label": f"the boat stack on the NX ({nx}): boat broker, usv_ui ({cfg['fcu']} FCU), the mission manager ({cfg['usv_profile']})"
                  + (" - " + FREE_RUN_LINE if free else ""),
                  "cmd": f"{ssh} {q(nx)} {q(f'cd {REMOTE_KIT} && {usv_cmd}')}"})
    uw = cfg.get("usv_window") or "mac"                 # v0.1.63 (Dmitrii 2026-11-07): the USV window on the team LAN for a delegate
    if uw.startswith("nx"):
        steps.append({"host": "nx", "label": f"the USV window on the NX ({nx}) - team LAN, one commander, PIN {usv_window_pin(ne)}",
                      "cmd": f"{ssh} {q(nx)} {q(f'cd {REMOTE_KIT} && scripts/run_boat_window.sh' + ''.join(' ' + x for x in netflag + uwc))}"})
        notes.append("USV window on the NX: open http://<the NX's team-LAN address>:8098/ on the delegate's laptop - take control there with the PIN " + usv_window_pin(ne))
    elif uw.startswith("mac"):
        notes.append("USV window on this Mac: open http://<this Mac's team-LAN address>:8098/ on the delegate's laptop - take control there with the PIN " + usv_window_pin(ne))
    cv = "fake" if (free and cfg["cv"] != "off") else cfg["cv"]        # v0.1.68: free run - the fake CV on the ghost's world
    if cv != "off":
        agx_env = [f"BOAT_HOST={q(str(cfg['boat_host']))}"] if net == "manual" and cfg.get("boat_host") else []
        agx_args = ["scripts/run_agx.sh"] + netflag + (["--sim", "--world", world_file] if cv == "fake" else [])
        steps.append({"host": "agx", "label": f"perception on the AGX ({agx}): " + ("the simulated CV on the world file" if cv == "fake" else "cv_publish + the camera stream")
                      + (" (free run: the same world as the ghost)" if free else ""),
                      "cmd": f"{ssh} {q(agx)} {q(f'cd {REMOTE_KIT} && ' + ' '.join(agx_env + agx_args))}"})
        if cv == "real":                                                # v0.1.83 (Dmitrii: "logging on by default when we run start"):
            steps.append({"host": "agx", "label": "the buoy observer on the AGX (buoys_multi + the guard) RECORDING the ZED (svo2 + frames.jsonl, "
                          "~/Desktop/dm3/recordings; stops by itself below 5 GB free)",
                          "cmd": f"{ssh} {q(agx)} {q(f'cd {REMOTE_KIT} && bash scripts/agx_observer.sh start --detector default' + {'auto': ' --pairs-s 10', 'off': ' --no-record'}.get(record_mode(cfg), ''))}"})
    if cfg["fcu"] != "fake":
        notes.append("a REAL Pixhawk: props off / ESC power off unless on the water; the companion never arms")
    if cfg["usv_profile"] == "lab":
        notes.append("the lab profile: bench guards (never on the water)")
    stop = f"ORIN_SSH={q(nx)} AGX_SSH={q(agx)} scripts/stop_all.sh"
    idx = lambda host: [i for i, st in enumerate(steps) if st["host"] == host and not st["label"].startswith(("reach the", "clock check"))]
    ops = {"mac": {"ssh": None, "expect": ["ocs"] + (["rctest"] if cfg["judge"] == "rctest" else []), "stop": "scripts/stop_ocs.sh",
                   "steps": idx("mac")},
           "nx": {"ssh": nx, "expect": ["usv"], "stop": f"{ssh} {q(nx)} {q(f'cd {REMOTE_KIT} && scripts/stop_usv.sh')}", "steps": idx("nx")}}
    if cfg["cv"] != "off":
        ops["agx"] = {"ssh": agx, "expect": ["agx"], "stop": f"{ssh} {q(agx)} {q(f'cd {REMOTE_KIT} && scripts/stop_agx.sh')}", "steps": idx("agx")}
    return {"steps": steps, "files": files, "remote_files": remote_files, "links": links, "notes": notes, "stamp": stamp,
            "hosts": {"nx": nx, "agx": agx}, "stop": stop, "ops": ops, "procs": plan_procs(cfg, nx, agx),
            "preflight": pre, "refuse": refuse}


# ---- what is up (v0.1.39.2) -------------------------------------------------------------------------------------------------
def tmux_sessions(ssh_target: Optional[str]) -> Optional[List[str]]:
    """The tmux session names on this Mac (None) or on an ssh host; None when the host does not answer."""
    cmd = "tmux ls -F '#S' 2>/dev/null; true"
    try:
        argv = ["bash", "-c", cmd] if not ssh_target else SSH.split() + [ssh_target, cmd]
        r = subprocess.run(argv, capture_output=True, text=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if ssh_target and r.returncode == 255:
        return None
    return [x.strip() for x in r.stdout.splitlines() if x.strip()]


def probe(record: Optional[dict], ls=tmux_sessions) -> dict:
    """{up, hosts: {host: {reach, found, expect, missing}}, foreign: [local sessions of ours without a record]}."""
    out = {"up": False, "hosts": {}, "foreign": []}
    ops = ((record or {}).get("plan") or {}).get("ops") or {}
    for host, op in ops.items():
        got = ls(op.get("ssh"))
        found = [x for x in (got or []) if x in op["expect"]]
        out["hosts"][host] = {"reach": got is not None, "found": found, "expect": op["expect"], "missing": [x for x in op["expect"] if x not in found],
                              "ssh": op.get("ssh")}
        out["up"] = out["up"] or bool(found)
    if not ops:
        out["foreign"] = [x for x in (ls(None) or []) if x in KNOWN_LOCAL]
        out["up"] = bool(out["foreign"])
    return out


# ---- v0.1.47 (A4, A5): the windows per host, the five pages, PROGRESS ---------------------------------------------------------
SHELLS = ("bash", "zsh", "sh", "fish", "dash", "-bash", "-zsh", "login")


def tmux_windows(ssh_target: Optional[str]) -> Optional[List[str]]:
    """'session:window:dead:command' for every tmux window here (None) or on an ssh host; None when the host does not answer."""
    cmd = "tmux list-windows -a -F '#{session_name}:#{window_name}:#{pane_dead}:#{pane_current_command}' 2>/dev/null; true"
    try:
        argv = ["bash", "-c", cmd] if not ssh_target else SSH.split() + [ssh_target, cmd]
        r = subprocess.run(argv, capture_output=True, text=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if ssh_target and r.returncode == 255:
        return None
    return [x.strip() for x in r.stdout.splitlines() if x.strip()]


_NAME_OK = re.compile(r"^[A-Za-z0-9_.-]{1,40}$")


def pane_tail(ssh_target: Optional[str], session: str, window: str, n: int = 40) -> Optional[List[str]]:
    """v0.1.52 (the launcher's CONSOLE): the last n lines of a tmux window the plan started - read only (capture-pane), here or
    over ssh; None when the host or the window does not answer. Only plain session / window names travel."""
    if not (_NAME_OK.match(session or "") and _NAME_OK.match(window or "")):
        return None
    cmd = f"tmux capture-pane -p -J -t {session}:{window} -S -{int(n)} 2>/dev/null"
    try:
        argv = ["bash", "-c", cmd] if not ssh_target else SSH.split() + [ssh_target, cmd]
        r = subprocess.run(argv, capture_output=True, text=True, timeout=8)
    except (OSError, subprocess.TimeoutExpired):
        return None
    if r.returncode != 0:
        return None
    lines = [x.rstrip() for x in r.stdout.splitlines()]
    while lines and not lines[-1]:
        lines.pop()
    return lines[-n:]


def _port(var: str, default: int) -> int:
    try:
        return int(os.environ.get(var, default))
    except ValueError:
        return default


def usv_window_pin(net_env: Optional[Dict[str, str]] = None) -> str:
    """v0.1.78.9: the USV window's PIN, shown in the launcher - USV_WINDOW_PIN from the network files, else boat_web's DEFAULT_PIN (0000)."""
    ne = net_env if net_env is not None else read_net("-")
    return (os.environ.get("USV_WINDOW_PIN") or ne.get("USV_WINDOW_PIN") or "0000").strip()


def usv_window_host(cfg: dict, net_env: Optional[Dict[str, str]] = None) -> str:
    """v0.1.78.6: where the USV window answers - a real launch binds it to the team LAN, not 127.0.0.1: on the NX = BOAT_HOST, on this Mac =
    OCS_HOST (the launch's profile, or the manual fields); the simulator / off = 127.0.0.1."""
    if (cfg or {}).get("mode") != "real":
        return "127.0.0.1"
    uw, net = str(cfg.get("usv_window") or "mac"), cfg.get("net")
    ne = net_env if net_env is not None else (read_net(net) if net in ("ts", "lan") else {})
    if uw.startswith("nx"):
        return (cfg.get("boat_host") if net == "manual" else None) or ne.get("BOAT_HOST") or "127.0.0.1"
    if uw.startswith("mac"):
        return (cfg.get("ocs_host") if net == "manual" else None) or ne.get("OCS_HOST") or "127.0.0.1"
    return "127.0.0.1"


def page_links(cfg: Optional[dict] = None, net_env: Optional[Dict[str, str]] = None) -> List[dict]:
    """The five links on PROGRESS (always shown): name, url, the probe path, what it is. Ports from the environment (the launchers' own)."""
    cfg = cfg or {}
    uwh = usv_window_host(cfg, net_env)
    ocs, jw, sim, bw, rct = (_port("OCS_WEB_PORT", 8096), _port("JUDGE_WEB_PORT", 8095), _port("SIM_PORT", 8097), _port("BOAT_WEB_PORT", 8098),
                             _port("RCTEST_PORT", 1885))
    return [
        {"name": "OCS page", "url": f"http://127.0.0.1:{ocs}/" + ("v1/" if cfg.get("ui") == "v1" else ""), "probe": f"http://127.0.0.1:{ocs}/prefs",
         "title": "the operator's page (ocs_web, tmux ocs / optiprimex window web)"},
        {"name": "judge page", "url": f"http://127.0.0.1:{jw}/", "probe": f"http://127.0.0.1:{jw}/status.json",
         "title": "the judge's view (judge_web): our fake judge's, or rc-test's when it runs (then wired to " + str(rct) + ")"},
        {"name": "SIM page", "url": f"http://127.0.0.1:{sim}/", "probe": f"http://127.0.0.1:{sim}/status", "title": "the simulator (simview; sim preset only)"},
        {"name": "USV window", "url": f"http://{uwh}:{bw}/", "probe": f"http://{uwh}:{bw}/state.json", "pin": usv_window_pin(net_env),
         "title": f"the boat's window (boat_web, {bw}) on " + ("this Mac" if uwh == "127.0.0.1" else uwh + " - the team LAN, where the launch bound it")},
        {"name": "rc-test console", "url": f"http://127.0.0.1:{_port('LAUNCH_PORT', 8094)}/?con=mac:judge:tui", "probe": f"http://127.0.0.1:{jw}/status.json",
         "rctest": rct, "title": f"rc-test's own console = rc-tui (tmux attach -t judge; --as-judge on {rct}): the link opens it in this page's CONSOLE "
                                f"(v0.1.53; the judge page is the other link) - the dot = the judge page answers wired to {rct}"},
    ] + ([{"name": "UAV window", "url": f"http://127.0.0.1:{_port('UAV_WEB_PORT', 8099)}/", "probe": f"http://127.0.0.1:{_port('UAV_WEB_PORT', 8099)}/state.json",
           "title": "v0.1.65: the UAV window (uav_web, tmux uavwin) - read-only, the whitelisted OCS lines"}] if cfg.get("uav") == "on" else [])


def _http(url: str, timeout: float = 0.8) -> Optional[bytes]:
    """The body (b'' for an HTTP error status: the server answers) or None (refused / timed out)."""
    import urllib.error
    import urllib.request
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.read(65536)
    except urllib.error.HTTPError:
        return b""
    except Exception:
        return None


def probe_pages(links: List[dict], http=_http) -> List[dict]:
    """Each link + up (bool) + why, the probes in parallel (a closed port answers at once; a hung one costs its timeout)."""
    res: Dict[str, Optional[bytes]] = {}
    urls = sorted({ln["probe"] for ln in links})
    th = [threading.Thread(target=lambda u=u: res.__setitem__(u, http(u)), daemon=True) for u in urls]
    for t in th:
        t.start()
    for t in th:
        t.join(3)
    out = []
    for ln in links:
        body = res.get(ln["probe"])
        up, why = body is not None, "answers" if body is not None else "does not answer"
        if up and ln.get("rctest"):
            try:
                port = json.loads(body or b"{}").get("broker_port")
            except ValueError:
                port = None
            up = port == ln["rctest"]
            why = f"the judge page is wired to {ln['rctest']}" if up else (f"the judge page is on {port} (not rc-test)" if port else "no rc-test judge page")
        out.append(dict(ln, up=up, why=why))
    return out


def progress(procs: List[dict], windows: Dict[str, Optional[List[str]]], pages: Dict[str, bool], seen: Dict[str, bool]) -> List[dict]:
    """PROGRESS lines: up (its probe answers: the window runs a program / the page answers), starting (not yet), died (a window back at
    its shell, or gone / silent after it was up), unknown (the host does not answer). `seen` remembers what was up (per launch)."""
    out = []
    for p in procs:
        if p["kind"] == "page":
            ok = bool(pages.get(p["page"]))
            st = "up" if ok else ("died" if seen.get(p["key"]) else "starting")
            why = "answers" if ok else ("stopped answering" if st == "died" else "not answering yet")
        else:
            w = windows.get(p["host"])
            if w is None:
                out.append(dict(p, state="unknown", why="the host does not answer"))
                continue
            row = next((x for x in w if x.split(":")[:2] == [p["session"], p["window"]]), None)
            if row is None:
                ok, st = False, ("died" if seen.get(p["key"]) else "starting")
                why = "window gone" if st == "died" else "no window yet"
            else:
                parts = row.split(":")
                dead = len(parts) > 2 and parts[2] == "1"
                cmd = parts[3] if len(parts) > 3 else "*"
                ok = not dead and cmd.lstrip("-") not in SHELLS
                st, why = ("up", f"runs {cmd}" if cmd != "*" else "session up") if ok else ("died", "back at the shell ([exited])")
        if ok:
            seen[p["key"]] = True
        out.append(dict(p, state=st, why=why))
    return out


def load_record(path: Optional[str] = None) -> Optional[dict]:
    path = path or RECORD
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def save_record(cfg: dict, plan: dict, path: Optional[str] = None) -> dict:
    path = path or RECORD
    rec = {"cfg": cfg, "plan": plan, "t": time.time(), "when": time.strftime("%H:%M"), "title": cfg.get("title") or cfg.get("preset") or cfg.get("mode")}
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w") as f:
        json.dump(rec, f, indent=1)
    return rec


def drop_record(path: Optional[str] = None) -> None:
    path = path or RECORD
    if os.path.exists(path):
        os.replace(path, path.replace("last.json", "last_stopped.json"))


def flavour(judge: Optional[str] = None, net: Optional[str] = None, fcu: Optional[str] = None, rec: Optional[dict] = "load") -> dict:
    """v0.1.42 (WP9d): the flavour line - preset · judge · net · FCU · profile · CV. From the launch record (logs/launch/last.json) when
    the stacks were launched from the page; what the console knows live (the judge, NET_PROFILE, usv_ui's FCU link) wins; `?` = unknown
    (a stack started by hand with flags)."""
    if rec == "load":
        rec = load_record()
    c = (rec or {}).get("cfg") or {}
    sim = c.get("mode") == "sim"
    f = {"preset": c.get("preset") or "?", "judge": judge or c.get("judge") or "?", "net": net or ("local" if sim else c.get("net")) or "?",
         "fcu": fcu or ("sim" if sim else c.get("fcu")) or "?", "profile": ("bench" if sim else c.get("usv_profile")) or "?",
         "cv": (f"sim {c.get('perception')}" if sim else c.get("cv")) or "?"}
    f["uav"] = c.get("uav") == "on"                  # v0.1.65 (D15): the chip gains "· uav" when the launch started the UAV side
    f["free_run"] = free_run_on(c)                   # v0.1.68 (WP17): the chip says FREE RUN
    f["text"] = (f"{f['preset']} · judge {f['judge']} · net {f['net']} · FCU {f['fcu']} · {f['profile']} · CV {f['cv']}" + (" · uav" if f["uav"] else "")
                 + (" · FREE RUN" if f["free_run"] else ""))
    am = c.get("auto_mission")                     # v0.1.59 (B4): the preset's --auto for `clear for run ▶` (the page's checkbox)
    if am is None:
        try:
            am = (load_presets().get(c.get("preset") or "") or {}).get("auto_mission")
        except Exception:
            am = None
    f["auto_mission"] = bool(am)
    return f


def summary(rec: Optional[dict], pr: dict) -> str:
    """The text Ctrl-C and --status print."""
    if not pr["up"]:
        return "nothing of ours is running" + (f" (the last launch, {rec['when']}, is down)" if rec else "")
    if not rec:
        return (f"running without a launch record: {', '.join(pr['foreign'])} on this Mac - stop: scripts/stop_all.sh "
                "(or stop_bench.sh / stop_ocs.sh)")
    lines = [f"the launch from {rec['when']} ({rec.get('title')}) is up:"]
    for h, st in pr["hosts"].items():
        where = "this Mac" if h == "mac" else f"{h.upper()} ({st['ssh']})"
        lines.append(f"  {where}: " + ("does not answer" if not st["reach"] else
                     (", ".join(st["found"]) or "nothing") + (f"  MISSING {', '.join(st['missing'])}" if st["missing"] else "")))
    lines.append(f"  reconnect: scripts/launch.sh -> RECONNECT    stop: {rec['plan']['stop']}")
    return "\n".join(lines)


class Runner:
    """Runs a plan's steps one after the other (a thread); the page polls the log."""

    def __init__(self, dry: bool = False):
        self.dry = dry
        self.lock = threading.Lock()
        self.log: List[dict] = []
        self.state = "idle"                  # idle | running | done | failed | adopted
        self.plan: Optional[dict] = None
        self.abort = False                   # v0.1.39.2: Ctrl-C during a launch - finish the step, skip the rest
        self.record_path: Optional[str] = None      # None = logs/launch/last.json (tests point it elsewhere)

    def _say(self, kind: str, text: str) -> None:
        with self.lock:
            self.log.append({"t": time.time(), "kind": kind, "text": text})
            self.log = self.log[-800:]

    def start(self, plan: dict, cfg: Optional[dict] = None) -> bool:
        with self.lock:
            if self.state == "running":
                return False
            self.state, self.plan, self.log, self.abort = "running", plan, [], False
        if cfg is not None and not self.dry:
            save_record(cfg, plan, self.record_path)
        threading.Thread(target=self._run, args=(plan,), daemon=True).start()
        return True

    def adopt(self, rec: dict) -> None:
        """RECONNECT: the running launch's plan becomes ours (links, STOP, restart per host); nothing is started."""
        with self.lock:
            self.plan, self.state = rec["plan"], "adopted"
            self.log = [{"t": time.time(), "kind": "ok", "text": f"reconnected to the launch from {rec['when']} ({rec.get('title')}) - nothing was started"}]

    def restart_host(self, host: str) -> bool:
        op = ((self.plan or {}).get("ops") or {}).get(host)
        if op is None:
            return False
        with self.lock:
            if self.state == "running":
                return False
            self.state, self.abort = "running", False
        sub = dict(self.plan, files=[], remote_files=[], steps=[{"host": host, "label": f"stop what is left on {host}", "cmd": op["stop"]}]
                   + [self.plan["steps"][i] for i in op["steps"]])
        self._say("step", f"restart {host}: stop what is left, then its launch step again")
        threading.Thread(target=self._run, args=(sub,), kwargs={"tolerate_first": True}, daemon=True).start()
        return True

    def _sh(self, cmd: str, stdin: Optional[str] = None, timeout: float = 120.0) -> int:
        p = subprocess.Popen(["bash", "-c", cmd], cwd=KIT, stdin=subprocess.PIPE if stdin is not None else subprocess.DEVNULL,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                             start_new_session=True)            # v0.1.39.2: the terminal's Ctrl-C never reaches a half-done step
        if stdin is not None:
            p.stdin.write(stdin)
            p.stdin.close()
        t0 = time.time()
        for line in p.stdout:
            self._say("out", line.rstrip())
            if time.time() - t0 > timeout:
                p.kill()
                self._say("bad", f"timed out after {timeout:.0f} s")
                break
        return p.wait()

    def _run(self, plan: dict, tolerate_first: bool = False) -> None:
        ok = True
        try:
            for f in plan["files"]:
                self._say("step", f"write {f['path']}")
                if not self.dry:
                    os.makedirs(os.path.join(KIT, os.path.dirname(f["path"])), exist_ok=True)
                    with open(os.path.join(KIT, f["path"]), "w") as fh:
                        fh.write(f["text"])
            for f in plan["remote_files"]:
                self._say("step", f"copy {f['path']} to {f['host']}")
                if not self.dry:
                    rc = self._sh(f"ssh -o ConnectTimeout=6 -o BatchMode=yes {shlex.quote(f['host'])} "
                                  f"{shlex.quote(f'mkdir -p {REMOTE_KIT}/logs/launch && cat > {REMOTE_KIT}/' + f['path'])}", stdin=f["text"])
                    if rc != 0:
                        raise RuntimeError(f"could not copy to {f['host']}")
            for i, s in enumerate(plan["steps"], 1):
                if self.abort:
                    ok = False
                    self._say("bad", f"Ctrl-C: steps {i}..{len(plan['steps'])} skipped - what is up stays up (scripts/launch.sh -> RECONNECT, or STOP)")
                    break
                self._say("step", f"{i}/{len(plan['steps'])} [{s['host']}] {s['label']}")
                self._say("cmd", s["cmd"])
                if self.dry:
                    continue
                rc = self._sh(s["cmd"])
                if rc != 0:
                    ok = s["label"].startswith("reach the") or (tolerate_first and i == 1)   # unreachable Jetson / nothing left to stop
                    self._say("bad" if not ok else "warn", f"exit {rc}")
                    if not ok:
                        break
        except Exception as exc:
            ok = False
            self._say("bad", f"{exc.__class__.__name__}: {exc}")
        with self.lock:
            self.state = "done" if ok else "failed"
        self._say("ok" if ok else "bad", "launched" + (" (dry run: nothing ran)" if self.dry else "") if ok else "stopped at the failed step")

    def stop_all(self) -> bool:
        with self.lock:
            if self.state == "running":
                return False
            self.state, self.log = "running", []
        cmd = (self.plan or {}).get("stop") or "scripts/stop_all.sh"      # after a sim launch: this Mac only; else every host, the OCS last

        def go():
            self._say("step", "STOP " + ("EVERYTHING (scripts/stop_all.sh)" if "stop_all" in cmd else "this Mac's stacks (the simulation)"))
            self._say("cmd", cmd)
            rc = 0 if self.dry else self._sh(cmd, timeout=240)
            if rc == 0 and not self.dry:
                drop_record(self.record_path)
            with self.lock:
                self.state = "stopped" if rc == 0 else "failed"
                if rc == 0:
                    self.plan = None
            self._say("ok" if rc == 0 else "bad", f"stop exit {rc}")
        threading.Thread(target=go, daemon=True).start()
        return True

    def status(self) -> dict:
        with self.lock:
            return {"state": self.state, "log": list(self.log), "links": (self.plan or {}).get("links", {}), "dry": self.dry,
                    "hosts": list(((self.plan or {}).get("ops") or {}).keys())}


class ThemePrefs:
    """v0.1.47 (A6): the launcher's theme - the source of truth. Starts from LAUNCH_THEME > logs/launch/prefs.json > the launch record >
    dark; sync() meets the OCS page's /prefs: first contact (the OCS just came up) or the OCS has none -> ours is pushed; the OCS changed
    since the last look (a page's own selector) -> adopted here. The pages all read the OCS's (or, the OCS down, ours)."""

    def __init__(self, path: Optional[str] = PREFS_FILE, record_path: Optional[str] = None, ocs=ocs_url):
        self.path, self.ocs = path, ocs                 # path None = not persisted; ocs None = no OCS (a dry run touches nothing)
        self.lock = threading.Lock()
        self.ocs_seen: Optional[str] = None             # the OCS's theme at the last look (None = not reachable)
        self.theme = norm_theme(os.environ.get("LAUNCH_THEME")) or self._file() or norm_theme(
            (((load_record(record_path) or load_record((record_path or RECORD).replace("last.json", "last_stopped.json")) or {}).get("cfg")) or {}).get("theme")) or "dark"

    def _file(self) -> str:
        if not self.path:
            return ""
        try:
            with open(self.path) as f:
                return norm_theme(json.load(f).get("theme"))
        except (OSError, ValueError, AttributeError):
            return ""

    def _save(self) -> None:
        if not self.path:
            return
        try:
            os.makedirs(os.path.dirname(self.path), exist_ok=True)
            with open(self.path, "w") as f:
                json.dump({"theme": self.theme, "t": time.time()}, f)
        except OSError:
            pass

    def set(self, theme: str) -> bool:
        t = norm_theme(theme)
        if not t:
            return False
        with self.lock:
            self.theme = t
            self._save()
        if self.ocs and post_theme(self.ocs(), t, "X-OCS-Web"):
            with self.lock:
                self.ocs_seen = t
        return True

    def sync(self) -> dict:
        got = get_theme(self.ocs()) if self.ocs else None
        push = None
        with self.lock:
            if got is None:
                self.ocs_seen = None
            elif got == self.theme:
                self.ocs_seen = got
            elif self.ocs_seen is None or not got or got == self.ocs_seen:
                push = self.theme                         # the OCS just came up / has none / we changed: ours goes there
            else:
                self.theme, self.ocs_seen = got, got        # a page changed it on the OCS: follow
                self._save()
        if push and post_theme(self.ocs(), push, "X-OCS-Web"):
            with self.lock:
                self.ocs_seen = push
        return self.get(ocs=got is not None)

    def get(self, ocs: Optional[bool] = None) -> dict:
        with self.lock:
            d = {"theme": self.theme}
        return {"ok": True, "prefs": d, "ocs": ocs if ocs is not None else self.ocs_seen is not None}


class Watch:
    """Probes what is up every `period` s in the background (ssh can take seconds - the page reads the cache)."""

    def __init__(self, runner: Runner, period: float = 5.0, ls=tmux_sessions, lw=None, http=_http, prefs: Optional[ThemePrefs] = None):
        self.runner, self.period, self.ls = runner, period, ls
        # v0.1.47: the windows (PROGRESS) - one tmux call per host gives both the sessions and the windows; an injected `ls` (tests)
        # without `lw` -> windows unknown, its sessions stand for them
        if lw is None:
            lw = tmux_windows if ls is tmux_sessions else (lambda ssh: None if ls(ssh) is None else [f"{x}:*" for x in ls(ssh)])
        self.lw, self.http, self.prefs = lw, http, prefs
        self.seen: Dict[str, bool] = {}
        self.seen_t = None
        self.lock = threading.Lock()
        self.last: dict = {"record": None, "probe": {"up": False, "hosts": {}, "foreign": []}, "t": 0.0, "pages": [], "progress": []}
        self.kick = threading.Event()

    def refresh(self) -> dict:
        rec = load_record(self.runner.record_path)
        cache: Dict[Optional[str], Optional[List[str]]] = {}

        def lw(ssh):
            if ssh not in cache:
                cache[ssh] = self.lw(ssh)
            return cache[ssh]
        ls = self.ls
        if self.ls is tmux_sessions:                    # derive the sessions from the windows: one call per host
            ls = lambda ssh: None if lw(ssh) is None else sorted({x.split(":")[0] for x in lw(ssh)})
        pr = probe(rec, ls)
        plan = (rec or {}).get("plan") or {}
        pages = probe_pages(page_links((rec or {}).get("cfg")), self.http)
        procs = plan.get("procs") or []
        if self.seen_t != (rec or {}).get("t"):
            self.seen, self.seen_t = {}, (rec or {}).get("t")
        hosts = {p["host"]: p.get("ssh") for p in procs if p["kind"] == "tmux"}
        prog = progress(procs, {h: lw(ssh) for h, ssh in hosts.items()}, {p["name"]: p["up"] for p in pages}, self.seen) if rec else []
        if self.prefs is not None:
            try:
                self.prefs.sync()
            except Exception:
                pass
        with self.lock:
            self.last = {"record": {k: rec[k] for k in ("cfg", "when", "title", "t")} if rec else None, "probe": pr, "t": time.time(),
                         "pages": pages, "progress": prog}
            return self.last

    def loop(self) -> None:
        while True:
            try:
                self.refresh()
            except Exception:
                pass
            self.kick.wait(self.period)
            self.kick.clear()

    def get(self) -> dict:
        with self.lock:
            return dict(self.last)


def make_handler(runner: Runner, watch: Optional[Watch] = None, prefs: Optional[ThemePrefs] = None, kit: Optional[str] = None):
    watch = watch or Watch(runner)
    # the Watch syncs only a ThemePrefs it was given (main); a dry run neither persists nor talks to the OCS
    prefs = prefs or watch.prefs or (ThemePrefs(None, runner.record_path, None) if runner.dry else ThemePrefs(record_path=runner.record_path))
    class H(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def _send(self, code: int, body: bytes, ctype: str) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code: int = 200) -> None:
            self._send(code, json.dumps(obj, default=str).encode(), "application/json")

        def do_GET(self):
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                with open(PAGE, "rb") as fh:
                    self._send(200, fh.read(), "text/html; charset=utf-8")
            elif path == "/presets.json":
                from optiprimex import __version__
                self._json({"version": __version__, "presets": load_presets(), "fields": {k: list(v) for k, v in list(FIELDS.items()) + list(OPTIONAL_FIELDS.items())}, "rates": RATE_KEYS,
                            "judge_label": JUDGE_LABEL,
                            "placeholders": net_placeholders(kit), "themes": list(THEMES)})     # v0.1.47 (A3): this page only, never logged
            elif path == "/themes.css":                                 # v0.1.47: the five palettes (+ --up)
                with open(os.path.join(os.path.dirname(PAGE), "themes.css"), "rb") as fh:
                    self._send(200, fh.read(), "text/css; charset=utf-8")
            elif path == "/prefs":                                      # v0.1.47 (A6): the theme, met with the OCS's on every read
                self._json(prefs.sync())
            elif path == "/status.json":
                self._json(runner.status())
            elif path == "/network.json":                               # v0.1.78.3: the NETWORK reference table (this page only, never logged)
                self._json(network_table(kit))
            elif path == "/reach.json":                                 # v0.1.86 (WP42): the judge dots + AGX · USV-NX · UAV-NX dots
                self._json(reach())
            elif path.startswith("/course_check.json"):                 # v0.1.82 (WP39): the RoboCommand network checker
                self._json(course_check(read_net("lan"), peek="nopeek" not in path))
            elif path == "/session.json":
                rec = load_record(runner.record_path) or {}            # v0.1.59 (A6): the launch's clock offsets for PROGRESS
                clocks = (((rec.get("plan") or {}).get("preflight") or {}).get("clocks")) or {}
                self._json(dict(watch.get(), adopted=runner.state == "adopted", clocks=clocks))
            elif path == "/console.json":                               # v0.1.52: a started window's last lines (the plan's windows only)
                from urllib.parse import parse_qs, urlparse
                key = (parse_qs(urlparse(self.path).query).get("key") or [""])[0]
                rec = load_record(runner.record_path) or {}
                procs = [p for p in ((rec.get("plan") or {}).get("procs") or []) if p.get("kind") == "tmux"]
                pr = next((p for p in procs if p["key"] == key), None)
                out = {"windows": [{"key": p["key"], "label": f"{p['host']} · {p['session']}:{p['window']} · {p['label']}"} for p in procs]}
                if pr is not None:
                    lines = pane_tail(pr.get("ssh"), pr["session"], pr["window"])
                    out.update(key=key, lines=lines, ok=lines is not None)
                self._json(out)
            else:
                self._send(404, b"not found", "text/plain")

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._json({"ok": False, "why": "bad json"}, 400)
            if self.headers.get("X-Launch") != "1":
                return self._json({"ok": False, "why": "missing X-Launch header"}, 403)
            if self.path == "/prefs":                                   # v0.1.47 (A6): this page's select (or a page, the OCS down)
                ok = prefs.set(str(body.get("theme", "")))
                return self._json(dict(prefs.get(), ok=ok, **({} if ok else {"why": f"theme: one of {', '.join(THEMES)}"})))
            if self.path == "/ssh":                                     # v0.1.78.4: a Terminal window with ssh to the NX / AGX (or the key copy)
                which, action = str(body.get("which", "")), str(body.get("action", "open"))
                if which not in ("nx", "agx", "uav") or action not in ("open", "copy-id"):
                    return self._json({"ok": False, "why": "which: nx | agx | uav; action: open | copy-id"}, 400)
                tgt = ssh_target(which, kit)
                if not tgt:
                    return self._json({"ok": False, "why": f"no {dict(nx='ORIN_SSH', agx='AGX_SSH', uav='UAV_SSH')[which]} in the network files"})
                try:
                    line = ssh_terminal_cmd(tgt, action)
                except ValueError as e:
                    return self._json({"ok": False, "why": str(e)})
                if runner.dry:
                    return self._json({"ok": True, "dry": True, "cmd": line})
                ok, why = open_terminal(line)
                return self._json({"ok": ok, "cmd": line, **({} if ok else {"why": why})})
            if self.path == "/observer":                                # v0.1.78.7: the AGX buoy observer (start | stop | status)
                action = str(body.get("action", "status"))
                if action not in OBSERVER_CMDS:
                    return self._json({"ok": False, "why": "action: start | stop | status"}, 400)
                tgt = ssh_target("agx", kit)
                det = str(body.get("detector") or "default")
                if det not in ("default", "model-a", "boundary-v14", "six-class"):
                    return self._json({"ok": False, "why": "detector: default | model-a | boundary-v14 | six-class"}, 400)
                if runner.dry:
                    return self._json({"ok": True, "dry": True, "state": "?", "text": "dry run - would run over ssh " + tgt + ": " + observer_cmd(action, detector=det)})
                return self._json(agx_observer(action, tgt, detector=det))
            cfg = body.get("cfg") or {}
            if self.path == "/plan":
                errs = validate(cfg)
                plan = None if errs else build_plan(cfg, pre=None if runner.dry else preflight_cached(cfg, plan_hosts(cfg)))
                refuse = (plan or {}).get("refuse") or []                # v0.1.59 (A5 / A6): shown in red, LAUNCH off
                self._json({"ok": not errs and not refuse, "errors": errs + refuse, "plan": plan})
            elif self.path == "/launch":
                errs = validate(cfg)
                if errs:
                    return self._json({"ok": False, "why": "; ".join(errs)})
                up = watch.refresh()
                if up["probe"]["up"] and not runner.dry:
                    rec = up["record"]
                    return self._json({"ok": False, "why": (f"the launch from {rec['when']} is still running: RECONNECT or STOP first" if rec
                                                            else f"{', '.join(up['probe']['foreign'])} already running here: STOP first")})
                plan = build_plan(cfg, pre=None if runner.dry else preflight(cfg, plan_hosts(cfg)))   # v0.1.59: measured again, not cached
                if plan.get("refuse") and not runner.dry:
                    return self._json({"ok": False, "why": "; ".join(plan["refuse"])})
                if norm_theme(cfg.get("theme")):                        # v0.1.47: what was launched with is the theme
                    prefs.set(cfg["theme"])
                ok = runner.start(plan, cfg)
                watch.kick.set()
                self._json({"ok": ok, "why": "" if ok else "a launch or a stop is still running"})
            elif self.path == "/adopt":
                rec = load_record(runner.record_path)
                if rec is None:
                    return self._json({"ok": False, "why": "no launch record (logs/launch/last.json)"})
                runner.adopt(rec)
                self._json({"ok": True, "cfg": rec["cfg"]})
            elif self.path == "/restart":
                ok = runner.restart_host(str(body.get("host", "")))
                watch.kick.set()
                self._json({"ok": ok, "why": "" if ok else "reconnect first (or a launch / stop is running)"})
            elif self.path == "/stop":
                if str(body.get("confirm", "")).strip().lower() != "stop":
                    return self._json({"ok": False, "why": "type stop to confirm"})
                if runner.plan is None:                      # not adopted: the record's stop (or stop_all when none)
                    rec = load_record(runner.record_path)
                    runner.plan = rec["plan"] if rec else None
                ok = runner.stop_all()
                watch.kick.set()
                self._json({"ok": ok, "why": "" if ok else "a launch or a stop is still running"})
            else:
                self._json({"ok": False, "why": "unknown"}, 404)
    return H


class Server(socketserver.ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def server_bind(self):
        socketserver.TCPServer.server_bind(self)
        self.server_name, self.server_port = str(self.server_address[0]), int(self.server_address[1])


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=int(os.environ.get("LAUNCH_PORT", "8094")))
    ap.add_argument("--open", action="store_true")
    ap.add_argument("--dry-run", action="store_true", help="LAUNCH / STOP ALL show what they would run, nothing runs")
    ap.add_argument("--status", action="store_true", help="print what runs where (from the last launch) and exit")
    a = ap.parse_args(argv)
    if a.status:
        rec = load_record()
        print(summary(rec, probe(rec)))
        return 0
    runner = Runner(dry=a.dry_run)
    watch = Watch(runner, prefs=ThemePrefs(None, None, None) if a.dry_run else ThemePrefs())
    threading.Thread(target=watch.loop, daemon=True).start()
    srv = Server(("127.0.0.1", a.port), make_handler(runner, watch))
    signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    signal.signal(signal.SIGINT, signal.default_int_handler)      # Ctrl-C even when started as a background job (SIGINT inherited as ignored)
    threading.Thread(target=srv.serve_forever, kwargs={"poll_interval": 0.5}, daemon=True).start()
    try:
        url = f"http://127.0.0.1:{a.port}/"
        print(f"[launcher] {url}{' (dry run)' if a.dry_run else ''} - pick a preset, change what you need, LAUNCH. Ctrl-C closes this page "
              "(the stacks it started keep running; STOP ALL or scripts/stop_all.sh stops them)", flush=True)
        if a.open and sys.platform == "darwin":
            subprocess.Popen(["open", url], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        if runner.state == "running":                     # mid-launch: finish this step, skip the rest
            runner.abort = True
            print("\n[launcher] Ctrl-C: finishing the current step, the rest is skipped (Ctrl-C again to leave now - the step keeps running)", flush=True)
            try:
                while runner.state == "running":
                    time.sleep(0.3)
            except KeyboardInterrupt:
                pass
        rec = load_record()
        print("\n[launcher] page closed. " + summary(rec, probe(rec)), flush=True)
    finally:
        srv.shutdown()
        srv.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
