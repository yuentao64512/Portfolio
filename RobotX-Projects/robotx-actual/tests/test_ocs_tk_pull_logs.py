"""WP44: scripts/pull_jetson_logs.sh - the Jetsons' logs to the Mac (dry run with made-up hosts; nothing is contacted)."""
from __future__ import annotations

import os
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "pull_jetson_logs.sh"


def run(*args, **env):
    e = dict(os.environ, ORIN_SSH="nx-user@nx.test", AGX_SSH="agx-user@agx.test", **env)
    return subprocess.run(["bash", str(SCRIPT), *args], capture_output=True, text=True, env=e, cwd=str(ROOT), timeout=20)


def test_dry_run_maps_every_source_and_contacts_nothing():
    r = run("--dry-run", "--date", "2026-11-12")
    assert r.returncode == 0, r.stderr
    out = r.stdout
    assert "nx-user@nx.test:~/Desktop/dm3/MASTER_SCRIPTS/logs/ -> logs/jetsons/2026-11-12/nx/logs/" in out
    assert "nx-user@nx.test:~/Desktop/dm3/MASTER_SCRIPTS/usv_ui/logs/ -> logs/jetsons/2026-11-12/nx/usv_ui_logs/" in out
    assert "agx-user@agx.test:~/Desktop/dm3/recordings/ -> logs/jetsons/2026-11-12/agx/recordings/" in out
    assert "agx-user@agx.test:~/Desktop/dm3/MASTER_SCRIPTS/logs/ -> logs/jetsons/2026-11-12/agx/tree_logs/" in out
    assert "nx" not in run("--agx", "--dry-run").stdout.split("AGX")[0].lower()           # --agx: the AGX only
    assert "AGX" not in run("--nx", "--dry-run").stdout


def test_it_never_deletes_and_refuses_unknown_options():
    text = SCRIPT.read_text()
    assert "--delete" not in text and "rm -" not in text and "_netenv.sh" in text
    assert run("--bogus").returncode == 2
