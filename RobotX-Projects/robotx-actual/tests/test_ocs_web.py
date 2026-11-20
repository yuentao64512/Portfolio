"""The OCS in the browser (ROADMAP WP8a, v0.1.28): tools/ocs_web.py on op-tui's Model, the page, the command path, the playbook's
verdicts, the SYSTEM tab's restarts and reboots (dry run: the command lines and the guards), and tools/fcu_reboot.py against the
fake FCU (refused armed, accepted disarmed - the fake answers MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN as ArduPilot does)."""
from __future__ import annotations

import argparse
import json
import re
import socket
import subprocess
import threading
import urllib.request
from types import SimpleNamespace

import pytest

from optiprimex.tools import ocs_web, system_actions
from optiprimex.tools.system_actions import Runner

from .conftest import PY, ROOT, free_port, free_udp_port

NS = "team/optiprime/v1"


def args(**kw):
    a = ocs_web.parse_args([])
    for k, v in kw.items():
        setattr(a, k, v)
    return a


def feed(web, r="R2", state="MANUAL", armed=False, fcu_link="fake", task=None, sub=None):
    web.m.on_hub("ocs/state", json.dumps({"body": {"r": r, "run_id": 1001 if r == "R5" else None, "judge_connected": True,
                                                    "vehicles": {"USV1": {"state": state, "task_name": task, "task_state": sub}}}}).encode())
    web.m.on_hub(f"{NS}/status/usv", json.dumps({"body": {"state": state, "armed": armed, "fcu_link": fcu_link, "task_name": task,
                                                           "task_state": sub, "mode": "HOLD"}}).encode())
    web.m.hub_up = True


def test_meta_and_state_follow_the_playbook():
    web = ocs_web.Web(args(), env={"ORIN_SSH": "nx", "AGX_SSH": "agx"}, feeds=False)
    meta = web._meta
    m0t4 = next(m for m in meta["missions"] if m["code"] == "m0t4")
    flags = {o["flag"]: o for o in m0t4["options"]}
    assert flags["--from"]["choices"] == ["", "OBSERVE", "ALIGN", "APPROACH"] and "--step" in flags and "--scan-hdg" in flags
    assert next(m for m in meta["missions"] if m["code"] == "goto")["sim"] is False
    assert [a["id"] for a in meta["system"] if a["group"] == "system"] == ["ocs", "rctest", "nx", "agx", "fcu", "nx_reboot", "agx_reboot"]
    assert {"select_target", "declare"} <= {u["word"] for u in meta["usage"]}
    feed(web)
    s = web.state()
    assert s["verdicts"]["DECLARE"]["ok"] and not s["verdicts"]["START_TASK"]["ok"] and s["next"].startswith("declare")
    assert s["system"]["refused"]["nx"] == "" and s["system"]["refused"]["ocs"] == ""
    feed(web, r="R5", state="AUTO_TASK", armed=True, fcu_link="real", task="T3", sub="ENTER")
    s = web.state()
    assert s["verdicts"]["HOLD"]["ok"] and s["stage_meaning"]
    ref = s["system"]["refused"]
    assert "AUTO_TASK" in ref["nx"] and "AUTO_TASK" in ref["fcu"] and "R5" in ref["ocs"] and "task is running" in ref["agx"]
    json.dumps(s, default=str)                                             # what /state.json serves


def test_the_page_is_self_contained_and_has_every_tab():
    html = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    assert not re.search(r'(src|href)="https?://', html) and re.findall(r"<link[^>]*>", html) == ['<link rel="stylesheet" href="/themes.css">']   # no external resources (v0.1.43: the shared palettes, same server)
    for tab in ("OVERVIEW", "MISSION", "USV", "UAV", "COMMS", "JUDGE", "LOG", "HELP", "SETTINGS"):     # v0.1.44: the regrouped tabs
        assert f'id="t-{tab}"' in html
    assert "X-OCS-Web" in html


@pytest.fixture()
def server():
    web = ocs_web.Web(args(fcu_control=free_udp_port()), env={}, feeds=False)
    web.runner = Runner(env={"ORIN_SSH": "nx-host", "AGX_SSH": "agx-host"}, dry_run=True, log_path=None)
    srv = ocs_web.Server(("127.0.0.1", free_port()), ocs_web.make_handler(web))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield web, f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()


def post(url, body, header=True):
    req = urllib.request.Request(url, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json", **({"X-OCS-Web": "1"} if header else {})})
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_http_page_commands_and_system(server, tmp_path, monkeypatch):
    web, url = server
    feed(web)
    with urllib.request.urlopen(url + "/", timeout=5) as r:
        assert b"<title>OCS" in r.read()
    with urllib.request.urlopen(url + "/map.html", timeout=5) as r:
        assert r.status == 200
    st = json.loads(urllib.request.urlopen(url + "/state.json", timeout=5).read())
    assert st["ocs"]["r"] == "R2" and st["verdicts"]["DECLARE"]["ok"]
    assert post(url + "/cmd", {"line": "declare"}, header=False)[0] == 403                     # a cross-site form cannot do it
    # an intent goes through ocs_cli.execute with op-tui's grammar
    seen = []
    web.con = SimpleNamespace(state=None, status=None)
    monkeypatch.setattr(ocs_web.ocs_cli, "execute", lambda con, ns: (seen.append(ns) or (0, ["intent accepted", "  ack DONE state=AUTO_TASK"])))
    code, r = post(url + "/cmd", {"line": "start_task m0t4sim --scan-x 30 --strafe-side port --from align"})
    assert code == 200 and r["ok"] and seen[0].task == "m0t4sim" and seen[0].strafe_side == "port" and seen[0].from_stage == "align"
    # a plant word goes to the fake FCU's control port
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.bind(("127.0.0.1", web.a.fcu_control))
        s.settimeout(2)
        code, r = post(url + "/cmd", {"line": "kill on"})
        assert json.loads(s.recvfrom(1000)[0]) == {"set": {"kill": True}} and r["ok"]
    assert post(url + "/cmd", {"line": "arm"})[1]["lines"][0] == "intent accepted"                  # v0.1.50: the page's ARM button (FCU_ARM)
    assert "unknown command" in post(url + "/cmd", {"line": "burst"})[1]["lines"][0]                # bursts stay on the boat console
    # SYSTEM in dry run: the command lines, the confirmation word, the guards
    web.runner.log_path = str(tmp_path / "sa.jsonl")
    code, r = post(url + "/system", {"action": "nx", "options": {"fcu": "uart", "profile": "lab", "net": "lan", "log": "on"}})
    assert r["ok"]
    job = web.runner.jobs[-1].public()
    assert job["cmd"].startswith("ssh -o BatchMode=yes -o ConnectTimeout=6 nx-host") and \
        "LOG=1 scripts/run_usv.sh --fcu uart --lab --lan" in job["cmd"] and "scripts/stop_usv.sh" in job["cmd"]
    assert not post(url + "/system", {"action": "fcu", "confirm": "yes"})[1]["ok"]                  # type 'fcu'
    assert post(url + "/system", {"action": "fcu", "confirm": "FCU"})[1]["ok"]
    assert "fcu_reboot" in web.runner.jobs[-1].public()["cmd"]
    post(url + "/system", {"action": "agx", "options": {"mode": "sim", "world": "boxes", "cam": "evil; rm -rf ~"}})
    agx = web.runner.jobs[-1].public()["cmd"]
    assert "agx-host" in agx and "--sim --world profiles/bench/world_boxes.yaml --cam-source dir" in agx and "evil" not in agx   # unknown choice -> default
    feed(web, r="R5", state="AUTO_TASK", armed=True, fcu_link="real", task="T3", sub="ENTER")
    for a in ("nx", "fcu", "nx_reboot", "ocs", "agx"):
        code, r = post(url + "/system", {"action": a, "confirm": a.split("_")[0] if "reboot" in a else a})
        assert not r["ok"] and r["why"].startswith("refused:"), (a, r)
    lines = [json.loads(x) for x in open(tmp_path / "sa.jsonl")]
    assert [x["action"] for x in lines] == ["nx", "fcu", "agx"] and all(x["dry_run"] for x in lines)


def test_ocs_restart_is_local_and_the_jetson_reboot_says_what_it_needs():
    ocs = system_actions.BY_ID["ocs"].build({"do": "restart", "judge": "rctest", "net": "ts", "log": "off"}, {})
    assert ocs[:2] == ["bash", "-c"] and "scripts/stop_ocs.sh; sleep 1; MAP=0 scripts/run_ocs.sh --judge rctest --ts" in ocs[2]
    rb = system_actions.BY_ID["agx_reboot"].build({}, {"AGX_SSH": "a"})
    assert rb[5] == "a" and "sudo -n /sbin/reboot" in rb[6] and "passwordless sudo" in rb[6]


def test_run_ocs_starts_the_page_and_stop_ocs_checks_it():
    h = subprocess.run(["bash", str(ROOT / "scripts" / "run_ocs.sh"), "--help"], capture_output=True, text=True, timeout=20).stdout
    assert "ocs_web" in h and "8096" in h
    run = (ROOT / "scripts" / "run_ocs.sh").read_text()
    assert "-n web" in run and "optiprimex.tools.ocs_web" in run and 'open "http://127.0.0.1:$OCS_WEB_PORT/' in run
    assert "ocs_web" in (ROOT / "scripts" / "stop_ocs.sh").read_text()


def test_fcu_reboot_against_the_fake_fcu(fake_fcu):
    via = f"udpin:127.0.0.1:{fake_fcu.mav_port}"
    out = subprocess.run([PY, "-m", "optiprimex.tools.fcu_reboot", "--via", via, "--wait", "0"], cwd=ROOT, capture_output=True, text=True, timeout=30)
    assert out.returncode == 1 and "REFUSED - the FCU is armed" in out.stdout, out.stdout + out.stderr      # the fixture's plant is armed
    fake_fcu.set(armed=False)
    out = subprocess.run([PY, "-m", "optiprimex.tools.fcu_reboot", "--via", via, "--wait", "10"], cwd=ROOT, capture_output=True, text=True, timeout=40)
    assert out.returncode == 0 and "ACCEPTED" in out.stdout and "the FCU is back" in out.stdout, out.stdout + out.stderr
    fake_fcu.set(armed=True)
