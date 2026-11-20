"""v0.1.78.4: ssh to the NX / AGX from the launcher (Dmitrii 2026-11-09: "wire the ssh command to the NX through the launcher once we hear it;
from another laptop we might be prompted for the password"): the network card's ssh ↗ (a Terminal window, interactive - a password prompt
is fine), "key ok" / "password needed" per Jetson, copy my key ↗ (ssh-copy-id), the real launch refused with that fix when a Jetson asks this
laptop for a password; the target always from the network files (the page says nx | agx)."""
from __future__ import annotations

import json
import threading
import urllib.request

import pytest

from optiprimex.tools import launcher as L

from .conftest import ROOT, free_port


def test_the_auth_probe():
    assert L.ssh_auth("u@h", probe=lambda a: (0, "")) == "key"
    assert L.ssh_auth("u@h", probe=lambda a: (255, "u@h: Permission denied (publickey,password).")) == "password"
    assert L.ssh_auth("u@h", probe=lambda a: (255, "ssh: connect to host h port 22: Operation timed out")) == "down"
    seen = []
    L.ssh_auth("u@h", probe=lambda a: seen.append(a) or (0, ""))
    assert "BatchMode=yes" in seen[0] and seen[0][-2:] == ["u@h", "true"]
    assert L.ssh_auth("u@h; rm -rf ~", probe=lambda a: (0, "")) == "down"                       # never run: not a target


def test_the_terminal_lines():
    assert L.ssh_terminal_cmd("usv-pixhawk@10.1.0.218") == "ssh -o ConnectTimeout=6 usv-pixhawk@10.1.0.218"
    c = L.ssh_terminal_cmd("usv-pixhawk@10.1.0.218", "copy-id")
    assert "ssh-copy-id -i ~/.ssh/id_ed25519.pub usv-pixhawk@10.1.0.218" in c and "[ -f ~/.ssh/id_ed25519.pub ] || ssh-keygen" in c
    for bad in ("a b", "x@y;ls", "$(id)", "", "-oProxyCommand=x", "-oLocalCommand", "u@-oX"):
        with pytest.raises(ValueError):
            L.ssh_terminal_cmd(bad)
    calls = []

    class R:
        returncode, stderr = 0, ""
    ok, _ = L.open_terminal('echo "a\\b"', run=lambda a, **k: calls.append(a) or R())
    assert ok and calls[0][:2] == ["osascript", "-e"] and calls[0][2] == 'tell application "Terminal" to do script "echo \\"a\\\\b\\""'


def _kit(tmp_path):
    (tmp_path / "network.env").write_text("NET_PROFILE=lan\n")
    (tmp_path / "network.lan.env").write_text("BOAT_HOST=10.1.0.218\nAGX_HOST=10.1.0.99\nORIN_SSH=usv-pixhawk@10.1.0.218\nAGX_SSH=agx@10.1.0.99\n")
    return str(tmp_path)


def test_the_table_carries_the_targets(tmp_path):
    t = L.network_table(_kit(tmp_path), ping=lambda a: a != "10.1.0.99", auth=lambda tgt: "password")
    R = {r["key"]: r for r in t["rows"]}
    assert R["BOAT_HOST"]["ssh"] == {"which": "nx", "target": "usv-pixhawk@10.1.0.218", "auth": "password"}
    assert R["AGX_HOST"]["ssh"] == {"which": "agx", "target": "agx@10.1.0.99", "auth": None}       # not heard: not probed
    assert R["UAV_HOST"]["ssh"]["target"] == ""                          # v0.1.78.12: the UAV row, no UAV_SSH here -> no target
    assert L.ssh_target("nx", _kit(tmp_path)) == "usv-pixhawk@10.1.0.218" and L.ssh_target("agx", _kit(tmp_path)) == "agx@10.1.0.99"


def test_a_launch_from_another_laptop_is_refused_with_the_fix(monkeypatch):
    monkeypatch.delenv("OPTIPRIMEX_PREFLIGHT", raising=False)
    monkeypatch.setattr(L, "ssh_auth", lambda h, probe=None: "password")
    pre = L.preflight({"mode": "real", "judge": "fake"}, {"nx": "usv-pixhawk@10.1.0.218"}, run=lambda argv, timeout=6.0: (255, ""))
    assert pre["clocks"]["nx"]["auth"] == "password"
    assert any("asks this laptop for a password" in r and "copy my key" in r for r in pre["refuse"])
    monkeypatch.setattr(L, "ssh_auth", lambda h, probe=None: "down")
    pre = L.preflight({"mode": "real", "judge": "fake"}, {"nx": "usv-pixhawk@10.1.0.218"}, run=lambda argv, timeout=6.0: (255, ""))
    assert pre["refuse"] == []                                        # unreachable: the plan's `reach` step shows it, as before


def test_the_route_dry_and_the_card(monkeypatch, tmp_path):
    monkeypatch.setattr(L, "ssh_target", lambda which, kit=None: {"nx": "usv-pixhawk@10.1.0.218"}.get(which, ""))
    port = free_port()
    srv = L.Server(("127.0.0.1", port), L.make_handler(L.Runner(dry=True)))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    def post(body, hdr=True):
        rq = urllib.request.Request(f"http://127.0.0.1:{port}/ssh", data=json.dumps(body).encode(), method="POST",
                                    headers={"Content-Type": "application/json", **({"X-Launch": "1"} if hdr else {})})
        try:
            return json.loads(urllib.request.urlopen(rq, timeout=5).read())
        except urllib.error.HTTPError as e:
            return json.loads(e.read())
    try:
        assert post({"which": "nx"}) == {"ok": True, "dry": True, "cmd": "ssh -o ConnectTimeout=6 usv-pixhawk@10.1.0.218"}
        assert post({"which": "nx", "action": "copy-id"})["cmd"].startswith("[ -f ~/.ssh/id_ed25519.pub ]")
        assert post({"which": "agx"})["ok"] is False                                              # no target
        assert post({"which": "boat; ls"})["ok"] is False and post({"which": "nx"}, hdr=False)["ok"] is False
    finally:
        srv.shutdown()
    page = (ROOT / "optiprimex" / "tools" / "launcher.html").read_text()
    for needle in ("function sshCell(r)", 'data-act="open"', 'data-act="copy-id"', 'fetch("/ssh"', 'id="netmsg"', "password needed"):
        assert needle in page, needle
