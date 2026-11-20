"""frame-compare (v0.1.31.1, for the lab day / WP9): usv_ui's swayto frames against m0t3's SWAY frames from the trace."""
from __future__ import annotations

import json
import subprocess

from optiprimex.tools import frame_compare as F

from .conftest import PY, ROOT

HEAD = ("t_mono,utc,mode,armed,lat,lon,hdg,gs,vn,ve,fix,sats,hdop,ekf,s1,s2,s3,s4,action,c1,c2,c3,c4,goto,wp_dist,x_n,y_e,"
        "path_state,path_s,path_xte,carrot_n,carrot_e,sway_state,sway_s,sway_c,sway_e\n")


def ui_csv(path, frame, n=30, state="sway"):
    import csv
    cols = HEAD.strip().split(",")
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=cols)
        w.writeheader()
        for i in range(n):
            w.writerow({"t_mono": f"{100 + 0.1 * i:.2f}", "mode": "CUSTOM_MANUAL", "action": "sway", "sway_state": state,
                        **{f"c{k + 1}": frame[k] for k in range(4)}})
        w.writerow({"t_mono": "104.0", "mode": "HOLD", "sway_state": "done", "c1": 0, "c2": 0, "c3": 0, "c4": 0})
    return str(path)


def trace(path, frame, n=30):
    ev = [{"k": "start", "t": 0.0}, {"k": "obs", "t": 0.1, "sub": "TO_START"},
          {"k": "frame", "t": 0.2, "m": [0.3, 0, 0.3, 0], "why": "cmd"}, {"k": "obs", "t": 1.0, "sub": "SWAY"},
          {"k": "frame", "t": 1.0, "m": [0, 0, 0, 0], "why": "entry"}]
    ev += [{"k": "frame", "t": round(1.0 + 0.1 * i, 2), "m": frame, "why": "cmd"} for i in range(1, n + 1)]
    path.write_text("\n".join(json.dumps(e) for e in ev) + "\n")
    return str(path)


def test_matching_and_differing_frames(tmp_path):
    sway = [0.05, 0.42, -0.05, 0.40]
    r = F.compare(F.ui_frames(ui_csv(tmp_path / "a.csv", sway)), F.trace_frames(trace(tmp_path / "t.jsonl", [0.06, 0.40, -0.05, 0.41])), 0.08)
    assert r["ok"] and r["ui"]["n"] == 30 and r["mm"]["n"] == 30 and 9.0 <= r["mm"]["rate_hz"] <= 10.5   # TO_START's frame left out
    r = F.compare(F.ui_frames(ui_csv(tmp_path / "b.csv", sway)), F.trace_frames(trace(tmp_path / "u.jsonl", [0.05, -0.42, -0.05, -0.40])), 0.08)
    assert r["ok"] is False and "signs" in r["why"]
    r = F.compare(F.ui_frames(ui_csv(tmp_path / "c.csv", sway)), F.trace_frames(trace(tmp_path / "v.jsonl", [0.05, 0.25, -0.05, 0.40])), 0.08)
    assert r["ok"] is False and "motor 2" in r["why"]
    assert F.compare([], F.trace_frames(trace(tmp_path / "w.jsonl", sway)), 0.08)["ok"] is None
    out = subprocess.run([PY, "-m", "optiprimex.tools.frame_compare", "--ui", str(tmp_path / "a.csv"), "--trace", str(tmp_path / "t.jsonl"), "--md"],
                         cwd=ROOT, capture_output=True, text=True, timeout=20)
    assert out.returncode == 0 and "| m0t3 SWAY |" in out.stdout and "**match**" in out.stdout
