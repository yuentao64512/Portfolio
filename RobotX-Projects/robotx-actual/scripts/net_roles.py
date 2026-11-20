#!/usr/bin/env python3
"""net_roles (v0.1.94, Dmitrii 2026-11-14: "can we make the course appearing regardless of port we plug in?") - the Mac's two wired
adapters get their roles from what they are plugged into, not from which adapter it is.

13 Nov 15:08: the course cable sat in the adapter configured with the fixed team address, so the Mac never took the course's DHCP address
(no RoboCommand); switching that adapter to DHCP fixed the course but took away the fixed team address (OCS_HOST), which the boat's bridge
dials - no boat heartbeat until an alias was added by hand with sudo, under the run timer.

The rule now: every wired adapter on DHCP (`dhcp`, once). Then, on every network change (the LaunchDaemon, `install`):
  - an adapter with an address in a course subnet (RC_<COURSE>_HOST's /24) is the COURSE side;
  - an adapter with an address in the team subnet (OCS_HOST's /24, the team switch's DHCP) is the TEAM side - it gets OCS_HOST as an
    alias (the address the boat / AGX dial); OCS_HOST is removed from every other adapter;
  - no team lease anywhere but one active adapter without any lease for >= fallback_s (a team switch without DHCP) -> that one is TEAM.
Wi-Fi is never touched (internet + the default route stay with the service order).

usage: net_roles.py status            the roles (no addresses printed)
       net_roles.py course            the course name the Mac is on (ALPHA ...), or nothing
       sudo net_roles.py apply        one pass (what the daemon runs)
       sudo net_roles.py dhcp         every wired USB / Thunderbolt LAN service -> DHCP (once)
       sudo net_roles.py install      copy this script + the needed addresses (root-owned) and load the LaunchDaemon
       sudo net_roles.py uninstall    unload + remove it (the alias stays until unplug / reboot)
The addresses come from network.env + network.lan.env (never printed); `install` snapshots them - re-run it after editing those files.
Python 3.9 stdlib only (runs as /usr/bin/python3 under launchd).
"""
from __future__ import annotations

import ipaddress
import json
import os
import re
import shutil
import subprocess
import sys
import time

LABEL = "ai.optiprime.netroles"
ROOT_SCRIPT = "/usr/local/libexec/optiprime_net_roles.py"
ROOT_CONF = "/usr/local/etc/optiprime_net_roles.json"
PLIST = f"/Library/LaunchDaemons/{LABEL}.plist"
STATE = "/var/run/optiprime_net_roles.json"
LOG = "/var/log/optiprime_net_roles.log"
FALLBACK_S = 20.0
TREE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


# ---- configuration ------------------------------------------------------------------------------------------------
def read_env(tree: str = TREE) -> dict:
    env = {}
    for name in ("network.env", "network.lan.env"):
        p = os.path.join(tree, name)
        if not os.path.exists(p):
            continue
        for line in open(p):
            m = re.match(r"^\s*([A-Z_][A-Z0-9_]*)=([^#\s]*)", line)
            if m:
                env[m.group(1)] = m.group(2).strip("\"'")
    return env


def conf_from_env(env: dict) -> dict:
    """{ocs_host, team_net, courses: {NAME: net}} - /24s (the venue's subnets)."""
    def net(ip):
        try:
            return str(ipaddress.ip_network(f"{ip}/24", strict=False))
        except ValueError:
            return None
    ocs = env.get("OCS_HOST")
    if not ocs or net(ocs) is None:
        raise SystemExit("net_roles: OCS_HOST (network.lan.env) is not an IPv4 address")
    courses = {}
    for k, v in env.items():
        m = re.match(r"^RC_([A-Z]+)_HOST$", k)
        if m and net(v):
            courses[m.group(1)] = net(v)
    if env.get("VENUE_RC_HOST") and net(env["VENUE_RC_HOST"]):
        courses.setdefault("VENUE", net(env["VENUE_RC_HOST"]))
    return {"ocs_host": ocs, "team_net": net(ocs), "courses": courses}


def load_conf() -> dict:
    if os.path.exists(ROOT_CONF) and (os.geteuid() == 0 or not os.path.exists(os.path.join(TREE, "network.lan.env"))):
        return json.load(open(ROOT_CONF))
    return conf_from_env(read_env())


# ---- the interfaces ---------------------------------------------------------------------------------------------
def _run(cmd) -> str:
    return subprocess.run(cmd, capture_output=True, text=True).stdout


def wifi_devices() -> set:
    out, dev, wifi = _run(["networksetup", "-listallhardwareports"]), None, set()
    port = None
    for line in out.splitlines():
        if line.startswith("Hardware Port:"):
            port = line.split(":", 1)[1].strip()
        elif line.startswith("Device:"):
            dev = line.split(":", 1)[1].strip()
            if port and ("Wi-Fi" in port or "AirPort" in port):
                wifi.add(dev)
    return wifi


def interfaces() -> dict:
    """{dev: {"active": bool, "addrs": [ip, ...]}} for the wired en* devices (Wi-Fi left out)."""
    out = _run(["ifconfig"])
    res, cur = {}, None
    wifi = wifi_devices()
    for line in out.splitlines():
        m = re.match(r"^([a-z]+\d+): flags=", line)
        if m:
            cur = m.group(1)
            if cur.startswith("en") and cur not in wifi:
                res[cur] = {"active": False, "addrs": []}
            else:
                cur = None
            continue
        if cur is None:
            continue
        s = line.strip()
        if s.startswith("inet "):
            res[cur]["addrs"].append(s.split()[1])
        elif s.startswith("status:"):
            res[cur]["active"] = s.endswith("active") and "inactive" not in s
    return res


# ---- the rule (pure) --------------------------------------------------------------------------------------------
def classify(addrs, conf) -> str:
    """course:<NAME> (a course lease wins - a stale alias there is removed) | team | none (no lease / link-local only)."""
    ips = [ipaddress.ip_address(a) for a in addrs]
    for name, net in conf["courses"].items():
        if any(ip in ipaddress.ip_network(net) for ip in ips):
            return f"course:{name}"
    if any(ip in ipaddress.ip_network(conf["team_net"]) for ip in ips):
        return "team"
    if any(not ip.is_link_local for ip in ips):           # v0.1.94.2: a lease from somewhere else (an iPhone's USB tether, a home router)
        return "other"                                    #   - never the team switch fallback
    return "none"


def plan(ifs: dict, conf: dict, since: dict, now: float, fallback_s: float = FALLBACK_S):
    """-> (roles {dev: role}, team_dev or None, [("add"|"del", dev)]). since = {dev: first time seen active without a lease}."""
    ocs = conf["ocs_host"]
    roles = {d: (classify(i["addrs"], conf) if i["active"] else "down") for d, i in ifs.items()}
    holders = [d for d, i in ifs.items() if ocs in i["addrs"]]
    team = [d for d, r in roles.items() if r == "team"]
    team_dev = next((d for d in team if d in holders), team[0] if team else None)
    if team_dev is None:                                   # a team switch without DHCP: the one adapter with no lease, after a while
        bare = [d for d, r in roles.items() if r == "none" and now - since.get(d, now) >= fallback_s]
        if len(bare) == 1:
            team_dev = bare[0]
            roles[team_dev] = "team?"
    acts = [("del", d) for d in holders if d != team_dev]
    if team_dev is not None and team_dev not in holders:
        acts.append(("add", team_dev))
    return roles, team_dev, acts


# ---- commands -----------------------------------------------------------------------------------------------------
def _log(msg: str) -> None:
    line = time.strftime("%Y-%m-%d %H:%M:%S ") + msg
    try:
        with open(LOG, "a") as f:
            f.write(line + "\n")
    except OSError:
        pass


def _since(ifs, conf) -> dict:
    try:
        st = json.load(open(STATE))
    except (OSError, ValueError):
        st = {}
    now = time.time()
    out = {}
    for d, i in ifs.items():
        if i["active"] and classify(i["addrs"], conf) == "none":
            out[d] = st.get(d, now)
    try:
        json.dump(out, open(STATE, "w"))
    except OSError:
        pass
    return out


def cmd_status() -> int:
    conf = load_conf()
    ifs = interfaces()
    roles, team_dev, acts = plan(ifs, conf, {}, time.time())
    for d in sorted(ifs):
        r = roles[d]
        if r == "down":
            continue
        alias = "yes" if conf["ocs_host"] in ifs[d]["addrs"] else "no"
        print(f"{d}: {r:16s} OCS_HOST here: {alias}")
    course = [r.split(":", 1)[1] for r in roles.values() if r.startswith("course:")]
    print(f"course network: {', '.join(course) or 'none'} · team side: {team_dev or 'none'} · "
          f"to do: {', '.join(f'{a} OCS_HOST on {d}' for a, d in acts) or 'nothing'}")
    daemon = os.path.exists(PLIST)
    print(f"auto daemon: {'installed' if daemon else 'NOT installed (sudo scripts/net_roles.sh install)'}")
    return 0


def judge_course(roles: dict):
    """v0.1.94.2 (Dmitrii: "judge auto should not connect to venue demo"): the course for --judge auto - ALPHA..DELTA only, never
    VENUE (the demo RC is an explicit --judge demo). None = no course lease."""
    names = sorted(r.split(":", 1)[1] for r in roles.values() if r.startswith("course:"))
    return next((n for n in names if n != "VENUE"), None)


def cmd_course() -> int:
    conf = load_conf()
    roles, _, _ = plan(interfaces(), conf, {}, time.time())
    name = judge_course(roles)
    if name:
        print(name)
    elif any(r == "course:VENUE" for r in roles.values()):
        print("judge auto: on the venue (demo) network only - auto never picks demo; --judge demo to use it", file=sys.stderr)
    return 0


def cmd_apply(quiet: bool = False) -> int:
    if os.geteuid() != 0:
        raise SystemExit("apply needs root: sudo scripts/net_roles.sh apply")
    conf = load_conf()
    ifs = interfaces()
    roles, team_dev, acts = plan(ifs, conf, _since(ifs, conf), time.time())
    for a, d in acts:
        if a == "del":
            subprocess.run(["ifconfig", d, "-alias", conf["ocs_host"]], capture_output=True)
        else:
            subprocess.run(["ifconfig", d, "alias", conf["ocs_host"], "255.255.255.0"], capture_output=True)
        _log(f"{a} OCS_HOST on {d} (roles: {', '.join(f'{k}={v}' for k, v in sorted(roles.items()) if v != 'down')})")
    if not quiet and not acts:
        print("nothing to do: " + ", ".join(f"{k}={v}" for k, v in sorted(roles.items()) if v != "down"))
    return 0


def wired_services() -> list:
    out = _run(["networksetup", "-listallnetworkservices"]).splitlines()[1:]
    return [s.lstrip("*").strip() for s in out if re.search(r"LAN|Ethernet|Thunderbolt|USB", s) and "Bridge" not in s
            and not re.search(r"Cube|FMU|PX4|Bluetooth|iPhone", s)]


def cmd_dhcp() -> int:
    if os.geteuid() != 0:
        raise SystemExit("dhcp needs root: sudo scripts/net_roles.sh dhcp")
    for s in wired_services():
        subprocess.run(["networksetup", "-setdhcp", s], capture_output=True)
        print(f"{s}: DHCP")
    return 0


PLIST_XML = """<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
  <key>Label</key><string>{label}</string>
  <key>ProgramArguments</key><array><string>/usr/bin/python3</string><string>{script}</string><string>apply</string><string>--quiet</string></array>
  <key>WatchPaths</key><array><string>/Library/Preferences/SystemConfiguration</string><string>/var/run/resolv.conf</string></array>
  <key>StartInterval</key><integer>5</integer>
  <key>RunAtLoad</key><true/>
  <key>StandardErrorPath</key><string>{log}</string>
</dict></plist>
"""


def cmd_install() -> int:
    if os.geteuid() != 0:
        raise SystemExit("install needs root: sudo scripts/net_roles.sh install")
    conf = conf_from_env(read_env())
    os.makedirs(os.path.dirname(ROOT_SCRIPT), exist_ok=True)
    os.makedirs(os.path.dirname(ROOT_CONF), exist_ok=True)
    shutil.copyfile(os.path.abspath(__file__), ROOT_SCRIPT)
    os.chown(ROOT_SCRIPT, 0, 0)
    os.chmod(ROOT_SCRIPT, 0o755)
    with open(ROOT_CONF, "w") as f:
        json.dump(conf, f)
    os.chown(ROOT_CONF, 0, 0)
    os.chmod(ROOT_CONF, 0o644)
    subprocess.run(["launchctl", "bootout", "system", PLIST], capture_output=True)
    with open(PLIST, "w") as f:
        f.write(PLIST_XML.format(label=LABEL, script=ROOT_SCRIPT, log=LOG))
    os.chown(PLIST, 0, 0)
    os.chmod(PLIST, 0o644)
    r = subprocess.run(["launchctl", "bootstrap", "system", PLIST], capture_output=True, text=True)
    print(f"installed: {PLIST} -> {ROOT_SCRIPT} (courses: {', '.join(sorted(conf['courses']))})" + (f" [{r.stderr.strip()}]" if r.returncode else ""))
    return 0


def cmd_uninstall() -> int:
    if os.geteuid() != 0:
        raise SystemExit("uninstall needs root: sudo scripts/net_roles.sh uninstall")
    subprocess.run(["launchctl", "bootout", "system", PLIST], capture_output=True)
    for p in (PLIST, ROOT_SCRIPT, ROOT_CONF, STATE):
        if os.path.exists(p):
            os.remove(p)
    print("uninstalled (an OCS_HOST alias stays until the adapter is unplugged or the Mac restarts)")
    return 0


def main(argv) -> int:
    cmd = argv[1] if len(argv) > 1 else "status"
    cmds = {"status": cmd_status, "course": cmd_course, "dhcp": cmd_dhcp, "install": cmd_install, "uninstall": cmd_uninstall,
            "apply": lambda: cmd_apply("--quiet" in argv)}
    if cmd not in cmds:
        print(__doc__)
        return 2
    return cmds[cmd]()


if __name__ == "__main__":
    sys.exit(main(sys.argv))
