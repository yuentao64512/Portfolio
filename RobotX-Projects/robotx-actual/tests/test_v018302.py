"""v0.1.83.2 (Dmitrii 2026-11-11): "the camera page ... a clear button to start recording video (if it is already recording - confirm)";
both = the ZED video on the AGX (agx_observer.sh with --record) + the 2 Hz frames on the Mac. GET /recording.json, POST /recording."""
from __future__ import annotations

import json
import subprocess
import time

from .conftest import ROOT, free_port


class FakeAgx:
    """The AGX side of agx_observer.sh: up / recording flags, the commands it was sent."""
    def __init__(self, up=False, rec=False):
        self.up, self.rec, self.cmds = up, rec, []

    def __call__(self, cmd: str) -> str:
        self.cmds.append(cmd)
        if cmd.endswith("status --json"):
            return json.dumps({"observer": "up" if self.up else "down", "recording": self.rec, "dir": "/r/20261111_120000_obs" if self.rec else "",
                               "mb": 12, "age_s": 1, "free_gb": 17.0})
        if cmd.endswith(" stop"):
            self.up = self.rec = False
            return "observer: down"
        if " start" in cmd:
            self.up, self.rec = True, "--no-record" not in cmd
            return "observer: up (buoys_multi)"
        return ""


class FakeRec:
    def __init__(self):
        self.st = {"state": "idle"}

    def status(self):
        return dict(self.st)

    def start(self, label, root, hz, max_min, depth):
        assert root == "mac_logs" and hz == 2.0 and max_min == 0.0
        self.st = {"state": "recording", "label": label, "frames": 0, "mb": 0}
        return self.st

    def stop(self, why="stopped"):
        self.st = {"state": "idle", "why": why}
        return self.st


def _web(tmp_path, agx):
    from optiprimex.tools import ocs_web as OW
    a = OW.parse_args(["--web-port", str(free_port()), "--judge-port", str(free_port())])
    web = OW.Web(a, env={"OCS_MARKERS": str(tmp_path / "m.json"), "OCS_WEB_DRYRUN": "1"}, feeds=False)
    web.camtab.ssh_run, web.camtab.rec = agx, FakeRec()
    return web


def test_start_both_then_already_recording_asks(tmp_path):
    agx = FakeAgx()
    ct = _web(tmp_path, agx).camtab
    assert ct.recording({"op": "status"})["recording"] is False
    r = ct.recording({"op": "start"})
    assert r["ok"] and r["agx"]["recording"] and r["mac"]["state"] == "recording" and "AGX video ●" in r["text"]
    assert any(c.endswith("agx_observer.sh start") for c in agx.cmds)
    r = ct.recording({"op": "start"})                                   # already recording -> confirm first, nothing sent
    n = len(agx.cmds)
    assert r["ok"] is False and r["already"] is True and "already recording" in r["why"]
    r = ct.recording({"op": "start", "confirm": "new"})                 # a new clip: stop, start
    assert r["ok"] and any(c.endswith(" stop") for c in agx.cmds[n:])


def test_observer_up_without_recording_needs_restart(tmp_path):
    agx = FakeAgx(up=True, rec=False)
    ct = _web(tmp_path, agx).camtab
    r = ct.recording({"op": "start"})
    assert r["ok"] is False and r["needs"] == "restart" and "ZED guard" in r["why"]
    assert ct.recording({"op": "start", "confirm": "restart"})["ok"]


def test_stop_asks_then_detection_goes_on(tmp_path):
    agx = FakeAgx(up=True, rec=True)
    ct = _web(tmp_path, agx).camtab
    assert ct.recording({"op": "stop"})["needs"] == "stop"
    r = ct.recording({"op": "stop", "confirm": "stop"})
    assert r["ok"] and r["recording"] is False and agx.up                 # the observer is back, detecting, not recording
    assert any("start --no-record" in c for c in agx.cmds)


def test_the_routes_and_the_script():
    py = (ROOT / "optiprimex" / "tools" / "ocs_web.py").read_text()
    assert 'path == "/recording.json"' in py and 'self.path == "/recording"' in py and 'self.path == "/camrec"' in py
    sh = ROOT / "scripts" / "agx_observer.sh"
    assert subprocess.run(["bash", "-n", str(sh)]).returncode == 0
    out = subprocess.run(["bash", str(sh), "status", "--json"], capture_output=True, text=True, env={"PATH": "/usr/bin:/bin", "HOME": "/tmp",
                                                                                                       "AGX_SESSION": "none"}).stdout
    j = json.loads(out.strip().splitlines()[-1])
    assert set(j) >= {"observer", "recording", "dir", "mb", "age_s", "free_gb"}
    assert subprocess.run(["bash", str(sh), "status"], capture_output=True, text=True).stdout.startswith("observer: ")   # the launcher's prefix


def test_an_old_agx_script_still_says_up_or_down(tmp_path):
    def old(cmd):
        if cmd.endswith("status --json"):
            return "unknown option --json"
        if cmd.endswith("status"):
            return "observer: up (buoys_multi)"
        return ""
    ct = _web(tmp_path, old).camtab
    r = ct.recording({"op": "status"})
    assert r["agx"]["observer"] == "up" and r["agx"]["recording"] is False and "predates 0.1.83.2" in r["agx"]["why"]


class PairsAgx(FakeAgx):
    """v0.1.84.4: an AGX whose observer saves RGB + depth pairs when started with --pairs-s."""
    def __init__(self, **kw):
        super().__init__(**kw)
        self.pairs = 0

    def __call__(self, cmd):
        if cmd.endswith("status --json"):
            d = json.loads(super().__call__(cmd))
            d.update(pairs=self.pairs, pairs_dir="/r/20261111_120000_obs/pairs" if self.pairs else "")
            return json.dumps(d)
        out = super().__call__(cmd)
        if " start" in cmd:
            self.pairs = 1 if "--pairs-s 10" in cmd else 0
        return out


def test_depth_is_honoured_and_said(tmp_path):
    """v0.1.84.4 (Dmitrii via session A: "record also at least 1 depth image along its RGB counterpart if I check the box ... see on the
    OCS where the images are being stored"): + depth starts the observer with --pairs-s 10 (the AGX pairs, the first at once) and the
    Mac recorder with depth; the answer's depth = {ok, text | why, where}; recording goes on either way; /recording.json says where."""
    agx = PairsAgx()
    ct = _web(tmp_path, agx).camtab
    ct.DEPTH_WAIT_S = 0.5
    seen = {}

    def start(label, root, hz, max_min, depth):
        seen["depth"] = depth
        ct.rec.st = {"state": "recording", "label": label, "frames": 0, "mb": 0, "depth_frames": 0}
        return ct.rec.st
    ct.rec.start = start
    ct.set_mode = lambda mode, vehicle="usv": {"ok": False, "why": "a detector holds the camera (source dir)"}
    st = ct.recording({"op": "status"})
    assert "/<stamp>_obs/" in st["targets"]["agx"] and "pairs/" in st["targets"]["agx"] and "logs/recordings" in st["targets"]["mac"]
    r = ct.recording({"op": "start", "depth": True})
    assert r["ok"] and seen["depth"] is True and any(c.endswith("agx_observer.sh start --pairs-s 10") for c in agx.cmds)
    assert r["depth"]["ok"] and r["depth"]["agx_pairs"] == 1 and r["depth"]["where"].endswith("/pairs")
    assert any(x.startswith("depth: 1 RGB + depth pair(s) on the AGX") for x in r["lines"])
    r = ct.recording({"op": "start", "confirm": "new"})                   # no depth asked: no pairs, no depth key
    assert "depth" not in r and any(c.endswith("agx_observer.sh start") for c in agx.cmds[-2:])

    old = FakeAgx()                                                       # an AGX before 0.1.84.4: no pairs -> why, recording goes on
    ct2 = _web(tmp_path, old).camtab
    ct2.DEPTH_WAIT_S = 0.5
    ct2.set_mode = lambda mode, vehicle="usv": {"ok": False, "why": "a detector holds the camera (source dir)"}
    r = ct2.recording({"op": "start", "depth": True})
    assert r["ok"] and r["depth"]["ok"] is False and "predates 0.1.84.4" in r["depth"]["why"]

    ct3 = _web(tmp_path, FakeAgx()).camtab                                # the Mac stream gives depth -> ok from the Mac side
    ct3.set_mode = lambda mode, vehicle="usv": {"ok": True, "mode": mode}

    def start3(label, root, hz, max_min, depth):
        ct3.rec.st = {"state": "recording", "label": label, "frames": 1, "mb": 0, "depth_frames": 1}
        return ct3.rec.st
    ct3.rec.start = start3
    r = ct3.recording({"op": "start", "depth": True})
    assert r["depth"]["ok"] and r["depth"]["mac_depth"] == 1 and "on the Mac" in r["depth"]["text"]


def test_the_observer_pairs(tmp_path):
    import importlib.util
    spec = importlib.util.spec_from_file_location("rec_core_t", ROOT / "cv" / "buoys_multi" / "rec_core.py")
    R = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(R)
    now = [100.0]
    p = R.Pairs(str(tmp_path), 10.0, clock=lambda: now[0])
    assert p.due() and p.names()["rgb"].endswith("pairs/rgb_000001.jpg") and p.names()["depth"].endswith("depth_000001.png")
    p.add({"frame": 0})
    assert not p.due()
    now[0] = 110.0
    assert p.due() and p.public() == {"on": True, "every_s": 10.0, "n": 1, "dir": str(tmp_path / "pairs")}
    p.close()
    line = json.loads((tmp_path / "pairs" / "pairs.jsonl").read_text().splitlines()[0])
    assert line == {"frame": 0, "i": 1}
    off = R.Pairs(str(tmp_path / "x"), 0)
    assert not off.due() and off.public()["on"] is False and not (tmp_path / "x" / "pairs").exists()
    src = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert "--pairs-s" in src and "save_pair(pairs, frame, xyz" in src and src.index("save_pair(pairs, frame, xyz") < src.index("model.predict(")
    sh = (ROOT / "scripts" / "agx_observer.sh").read_text()
    assert '--pairs-s) PAIRS="$2"' in sh and '"pairs": %s' in sh


def test_frame_rec_counts_depth(tmp_path, monkeypatch):
    from optiprimex.tools import frame_rec as FR
    pics = {"/snapshot.jpg": [b"a", b"b", b"c"], "/depth.jpg": [b"d1", None, b"d3"]}

    def fetch(url):
        for k, v in pics.items():
            if url.endswith(k):
                return v.pop(0) if v else None
        return None
    rec = FR.Recorder(lambda: "http://cam", env={"HOME": str(tmp_path)}, fetch=fetch)
    orig = FR.root_path
    monkeypatch.setattr(FR, "root_path", lambda root, env=None: str(tmp_path) if root == "mac_logs" else orig(root, env))
    rec.start("t", "mac_logs", hz=15.0, max_min=0, depth=True)
    t0 = time.time()
    while time.time() - t0 < 3 and rec.status().get("frames", 0) < 3:
        time.sleep(0.05)
    st = rec.stop()
    assert st["frames"] == 3 and st["depth_frames"] == 2 and st["depth_misses"] == 1


def test_the_live_feed_preview_every_frame():
    """v0.1.84.8 (Dmitrii: "our feed during run also slow ... maybe with lower resolution"): the observer writes latest_preview.jpg (what
    the camera stream serves while it holds the ZED) every frame at 960 px, atomically - it was every 15th frame (one picture per 4-15 s)."""
    src = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert '"--preview-every", type=int, default=1' in src and '"--preview-width", type=int, default=960' in src
    assert "write_preview(run_dir / \"latest_preview.jpg\", frame, a.preview_width)" in src and "os.replace(tmp, str(path))" in src
    assert 'cv2.imwrite(str(run_dir / "latest_preview.jpg"), frame)' not in src
    st = (ROOT / "cv" / "stream" / "cam_stream.py").read_text()
    assert 'os.path.join(CV, "buoys", "runs", "*", "latest_preview.jpg")' in st          # the .tmp never matches the stream's glob
