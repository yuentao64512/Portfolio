"""v0.1.78.11 (Dmitrii 2026-11-09, Team Village: "the UAV is not connecting in OCS" - the launcher showed `uav:adapter ... back at the shell
([exited]) died` and the same for the UAV window, while both ran and the drone's telemetry flowed): the launcher's liveness check reads a tmux
window's current command, and `bash adapter.sh` shows as `bash` (the program is its child). The wrappers are now sourced (`. ./adapter.sh`):
the window's own shell runs the pipeline, as every other window does, and tmux reports the program."""
from __future__ import annotations

import shutil
import subprocess
import time

import pytest

from .conftest import ROOT


def test_the_wrappers_are_sourced():
    uav = (ROOT / "scripts" / "run_uav.sh").read_text()
    win = (ROOT / "scripts" / "run_uav_window.sh").read_text()
    assert '"$SESSION:uav" ". ./adapter.sh"' in uav and '"$SESSION:fakeuav" ". ./fake_uav.sh"' in uav and '"$SESSION:web" ". ./uav_web.sh"' in win
    for src in (uav, win, (ROOT / "optiprimex" / "tools" / "fault_timing.py").read_text()):
        assert '"bash adapter.sh"' not in src and '"bash uav_web.sh"' not in src and '"bash fake_uav.sh"' not in src


@pytest.mark.skipif(not shutil.which("tmux"), reason="no tmux")
def test_tmux_reports_the_program_when_sourced(tmp_path):
    (tmp_path / "w.sh").write_text(f"cd '{tmp_path}' && python3 -c 'import time; time.sleep(6)' 2>&1 | tee -i '{tmp_path}/x.log'; echo '[exited]'\n")
    s = f"v0178011_{int(time.time())}"
    T = ["tmux", "-L", s]                                               # its own tmux server: never the operator's live sessions
    subprocess.run(T + ["new-session", "-d", "-s", s, "-n", "w", "-c", str(tmp_path)], check=True)
    try:
        subprocess.run(T + ["send-keys", "-t", f"{s}:w", ". ./w.sh", "C-m"], check=True)
        cmd = ""
        for _ in range(30):
            time.sleep(0.2)
            cmd = subprocess.run(T + ["list-windows", "-t", s, "-F", "#{pane_current_command}"], capture_output=True, text=True).stdout.strip()
            if cmd.startswith("python"):
                break
        from optiprimex.tools import launcher as L
        assert cmd.startswith("python") and cmd.lstrip("-") not in L.SHELLS, cmd          # the launcher's own rule: not a shell = up
    finally:
        subprocess.run(T + ["kill-server"], capture_output=True)                      # only this private server
