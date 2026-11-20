"""ROADMAP WP10j (v0.1.46): the Task 3 docking structure in the renderer (docs/DOCK_MOCKUP_SPEC.md: the white cube on its PVC frame,
windows 1 upper-left / 2 lower-right with black borders, the RED / GREEN indicator at the base, the window lights from fake_dock's
sim/dock/state) and the dock dataset for CV_TRAIN (`render --dataset --scene dock | mix`, five classes with `indicator`). Needs
requirements-sim.txt (numpy + Pillow)."""
from __future__ import annotations

import io
import json
import random
import time

import pytest
import yaml

from .conftest import ROOT, free_port, free_udp_port, wait_until

np = pytest.importorskip("numpy")
pytest.importorskip("PIL")

from optiprimex.sim import dataset as D  # noqa: E402
from optiprimex.sim import render as R  # noqa: E402
from optiprimex.sim import world as W  # noqa: E402
from optiprimex.sim.fake_dock import DockSim  # noqa: E402

POOL = str(ROOT / "profiles" / "bench" / "world_pool.yaml")
MARINA = str(ROOT / "profiles" / "bench" / "world_marina.yaml")
DOCKED = (43.5, 5.5, 0.0)                       # 1.5 m off bay 2's face (x_n 45), straight on - m3t1's stop


def codes(sc: R.Scene, tag: str):
    return [R.BOX_ID0 + j for j, b in enumerate(sc.boxes) if b.tag == tag]


def mask(sc, pid, tag):
    return np.isin(pid, codes(sc, tag))


def panes(sc, tag):
    return [b for b in sc.boxes if b.tag == tag and b.half[1] < 0.01 and b.half[0] < R.WINDOW_M / 2 + 0.001]


def test_the_dock_primitives_render_with_their_own_ids():
    w = W.load(POOL)
    sc = R.scene_from_world(w)
    tags = {b.tag for b in sc.boxes}
    for bay, oid in ((1, 21), (2, 22), (3, 23)):
        assert {f"dock:{oid}", f"cube:{oid}", f"deck:{oid}", f"indicator:{oid}"} <= tags
    assert {f"window:{i}" for i in range(31, 37)} <= tags                         # the world's window objects keep their ids
    for b in sc.boxes + sc.spheres:                                                # the sim observer's contract: kind:<int>
        k, o = b.tag.split(":")
        int(o)
    cam = R.Camera(w=640, h=360)
    _, depth, pid = R.render_ids(sc, *DOCKED, cam)
    face, w1, w2, ind = (mask(sc, pid, t) for t in ("dock:22", "window:33", "window:34", "indicator:22"))
    assert face.sum() > 20000 and w1.sum() > 2000 and w2.sum() > 2000
    assert not (face & w1).any() and len(set(np.unique(pid[w1])) & set(np.unique(pid[w2]))) == 0
    for m in (w1, w2):                                                             # both windows wholly in the frame at 1.5 m
        rr, cc = np.nonzero(m)
        assert rr.min() > 0 and cc.min() > 0 and rr.max() < 359 and cc.max() < 639
        assert (rr.max() - rr.min()) > 80                                          # ~0.32 m with the border at 1.5 m: > 80 px of 360
    (r1, c1), (r2, c2) = (np.argwhere(w1).mean(0)), (np.argwhere(w2).mean(0))
    assert c1 < c2 and r1 < r2                                                     # 1 upper-left, 2 lower-right as seen from the water
    assert abs(float(np.median(depth[face])) - 1.5) < 0.05
    mw = R.scene_from_world(W.load(MARINA))                                        # a world without window objects: synthetic ids
    assert f"window:{R.SYN_ID0 + 21 * 10 + 1}" in {b.tag for b in mw.boxes} and mw.parts[f"window:{R.SYN_ID0 + 212}"]["window"] == 2
    assert R.face_heading(w, W.bay_faces(w["objects"])[1]) == pytest.approx(180.0)                 # no face_hdg_deg: toward the start


def test_the_window_lights_follow_the_dock_state():
    w = W.load(POOL)
    ds = DockSim(w, seed=3)
    cam = R.Camera(w=640, h=360)
    idle = R.scene_from_world(w, dock_state=ds.state(0.0))
    assert ds.phase == "idle" and not any(b.emissive for b in idle.boxes if b.tag.startswith("window:"))
    assert all(p["light"] == "off" and not p["lit"] for k, p in idle.parts.items() if k.startswith("window:"))
    ds.command({"cmd": "docked"}, 1.0)
    st = ds.state(1.0)
    assert ds.phase == "lit" and st["window"]["colour"] == "red"
    wid = 33 if ds.window == 1 else 34
    lit = R.scene_from_world(w, dock_state=st)
    on = [b for b in lit.boxes if b.tag.startswith("window:") and b.emissive]
    assert len(on) == 1 and on[0].tag == f"window:{wid}" and lit.parts[f"window:{wid}"]["light"] == "red"
    rgb, _, pid = R.render_ids(lit, *DOCKED, cam)
    px = rgb[np.isin(pid, [R.BOX_ID0 + lit.boxes.index(on[0])])].astype(int)
    assert px[:, 0].mean() > 200 and px[:, 0].mean() > px[:, 1].mean() + 100        # RED in the camera
    ds.command({"cmd": "fire"}, 2.0)
    gr = R.scene_from_world(w, dock_state=ds.state(2.0))
    on = [b for b in gr.boxes if b.tag.startswith("window:") and b.emissive]
    assert ds.phase == "green" and len(on) == 1 and on[0].tag == f"window:{wid}" and gr.parts[f"window:{wid}"]["colour"] == "green"
    rgb, _, pid = R.render_ids(gr, *DOCKED, cam)
    px = rgb[np.isin(pid, [R.BOX_ID0 + gr.boxes.index(on[0])])].astype(int)
    assert px[:, 1].mean() > px[:, 0].mean() + 100
    ds.tick(10.0, None)                                                            # GREEN held 5 s -> off
    assert ds.phase == "done" and not any(b.emissive for b in R.scene_from_world(w, dock_state=ds.state(10.0)).boxes if b.tag.startswith("window:"))
    fl = R.scene_from_world(w, dock_state={"green_bay": 2, "window": {"bay": 2, "window": 1, "colour": "blue", "flash": True, "on": False}})
    assert fl.parts["window:33"]["light"] == "flash" and not fl.parts["window:33"]["lit"]   # the Advanced flash, in its off second
    # without a state: the world file's window objects (as fake_perception has them)
    objs = W.apply_dock_state(w["objects"], {"green_bay": 2, "window": {"bay": 2, "window": 2, "colour": "red"}})
    assert R.scene_from_world(dict(w, objects=objs)).parts["window:34"]["lit"] is True


def test_the_indicator_colour_per_bay():
    w = W.load(POOL)
    sc = R.scene_from_world(w, dock_state={"green_bay": 3, "window": {"bay": None, "window": None, "colour": "black"}})
    assert [sc.parts[f"indicator:{i}"]["colour"] for i in (21, 22, 23)] == ["red", "red", "green"]
    assert sc.parts["dock:23"]["indicator_colour"] == "green"
    plain = R.scene_from_world(w)                                                  # no state: the world file's colours (bay 2 GREEN)
    assert [plain.parts[f"indicator:{i}"]["colour"] for i in (21, 22, 23)] == ["red", "green", "red"]
    ind = [b for b in sc.boxes if b.tag == "indicator:23"]
    assert len(ind) == 1 and ind[0].shape == "cyl" and ind[0].emissive and ind[0].half[0] == pytest.approx(R.INDICATOR_R_M)
    cam = R.Camera(w=640, h=360)
    rgb, _, pid = R.render_ids(sc, 42.5, 9.0, 0.0, cam, pitch_deg=8.0)          # 2.5 m off bay 3, looking down a little
    px = rgb[pid == R.BOX_ID0 + sc.boxes.index(ind[0])].astype(int)
    assert len(px) > 30 and px[:, 1].mean() > px[:, 0].mean() + 80


def check_export(out, man, w, h, names):
    from PIL import Image
    data = yaml.safe_load((out / "data.yaml").read_text())
    assert data["names"] == dict(enumerate(names))
    seen = {c: 0 for c in names}
    for p in (out / "truth").glob("*.json"):
        t = json.loads(p.read_text())
        lines = (out / "labels" / f"{p.stem}.txt").read_text().splitlines()
        m = np.array(Image.open(out / "masks" / f"{p.stem}.png"))
        assert len(lines) == len(t["objects"]) and int(m.max()) == len(t["objects"])
        for ln, o in zip(lines, t["objects"]):
            f = ln.split()
            v = [float(x) for x in f[1:]]
            assert int(f[0]) == o["class_id"] and names[o["class_id"]] == o["class"] and len(v) >= 6 and len(v) % 2 == 0
            assert all(0.0 <= x <= 1.0 for x in v)
            x0, y0, x1, y1 = o["bbox"]
            assert min(v[0::2]) * w >= x0 - 1 and max(v[0::2]) * w <= x1 + 1 and min(v[1::2]) * h >= y0 - 1 and max(v[1::2]) * h <= y1 + 1
            assert int((m == o["k"]).sum()) == o["area_px"] >= D.MIN_PX and o["visible_frac"] >= D.MIN_VIS
            if o["kind"] == "window":
                assert o["window"] in (1, 2) and o["light"] in ("off", "red", "green", "flash") and isinstance(o["lit"], bool)
                assert o["lit"] == (o["light"] in ("red", "green") or (o["light"] == "flash" and t["dock"]["state"]["window"].get("on")))
                assert o["bay"] in (1, 2, 3) and 0.0 <= o["face_angle_deg"] <= 90.0
            if o["kind"] in ("indicator", "dock_face"):
                assert o["indicator_colour"] in ("red", "green")
                assert o["indicator_colour"] == ("green" if o["bay"] == t["dock"]["state"]["green_bay"] else "red")
            seen[o["class"]] += 1
    assert seen == man["instances"]["all"]
    return seen


def test_the_dock_dataset_five_classes(tmp_path):
    out = tmp_path / "dock"
    man = D.export(str(out), 18, ["pool", "marina"], seed=4, w=640, h=360, frames_per_pose=3, scene="dock", log=lambda *_: None)
    seen = check_export(out, man, 640, 360, D.CLASSES_KIND5)
    assert D.CLASSES_KIND5 == ["buoy", "dock_face", "window", "box", "indicator"]
    assert seen["dock_face"] > 0 and seen["window"] > 0 and seen["indicator"] > 0
    assert man["scene"] == "dock" and set(man["bands"]) <= {"approach", "docked", "passing", "negative"} and len(man["bands"]) >= 2
    assert len(list((out / "depth").glob("*.npz"))) == 18
    out2 = tmp_path / "mix"
    man2 = D.export(str(out2), 24, ["pool"], seed=8, w=320, h=180, frames_per_pose=2, scene="mix", negatives=0.2, log=lambda *_: None)
    check_export(out2, man2, 320, 180, D.CLASSES_KIND5)
    assert "random" in man2["bands"] and set(man2["bands"]) & {"approach", "docked", "passing"}
    out3 = tmp_path / "col"
    man3 = D.export(str(out3), 6, ["pool"], seed=4, w=320, h=180, scene="dock", scheme="colour", log=lambda *_: None)
    assert man3["names"][-2:] == ["indicator_red", "indicator_green"] and man3["names"][:-2] == D.CLASSES_COLOUR


def test_a_docked_frame_shows_both_windows_fully():
    """The spec's acceptance: the stop at 1.5 m (1280 x 720, the ZED's intrinsics) shows both windows whole, ~200 px each."""
    w = W.load(POOL)
    cam = R.Camera(w=1280, h=720)
    st = {"green_bay": 2, "window": {"bay": 2, "window": 1, "colour": "red"}}
    _, inst, _, meta, lines = D.frame(w, random.Random(1), cam, "kind", (43.5, 5.5, 0.0, 0.0, 0.8), {}, None, False,
                                      dock_state=st, names=D.CLASSES_KIND5)
    wins = [o for o in meta["objects"] if o["kind"] == "window"]
    assert sorted(o["window"] for o in wins) == [1, 2] and all(o["visible_frac"] > 0.97 for o in wins)
    for o in wins:
        x0, y0, x1, y1 = o["bbox"]
        assert x0 > 0 and y0 > 0 and x1 < 1280 and y1 < 720 and 190 < x1 - x0 < 320
    assert [o["light"] for o in sorted(wins, key=lambda o: o["window"])] == ["red", "off"]
    face = next(o for o in meta["objects"] if o["kind"] == "dock_face")
    assert face["truth"]["range_m"] == pytest.approx(1.5, abs=0.05) and face["face_angle_deg"] < 1.0
    assert face["visible_frac"] > 0.9                                              # its own windows do not count as hiding it


def test_the_buoy_dataset_is_unchanged_by_default(tmp_path):
    out = tmp_path / "b"
    man = D.export(str(out), 6, ["pool"], seed=3, w=160, h=90, log=lambda *_: None)
    assert yaml.safe_load((out / "data.yaml").read_text())["names"] == {0: "buoy", 1: "dock_face", 2: "window", 3: "box"}
    assert man["scene"] == "buoys" and set(man["instances"]["all"]) == set(D.CLASSES_KIND) and set(man["bands"]) <= {"random"}
    t = json.loads((out / "truth" / "world_pool_s3_p00000_f00.json").read_text())
    assert "dock" not in t and t["band"] == "random"
    # the rng stream of v0.1.33: the same seed gives the same poses whatever the dock looks like now
    rng = random.Random(3)
    w = W.load(POOL)
    cam = R.Camera(w=160, h=90)
    x, y, hdg = D.sample_pose(w, rng, cam.hfov_deg)
    assert (t["pose"]["x_n"], t["pose"]["y_e"]) == (round(x, 3), round(y, 3))
    # a four-class frame facing the dock: the indicator is seen but not labelled (listed under not_labelled)
    _, _, _, meta, _ = D.frame(w, random.Random(1), R.Camera(w=640, h=360), "kind", (40.0, 5.5, 0.0, 0.0, 0.8), {}, None, False)
    assert {o["class"] for o in meta["objects"]} <= set(D.CLASSES_KIND) and "window" in {o["class"] for o in meta["objects"]}
    assert any(s["kind"] == "indicator" for s in meta["not_labelled"])
    with pytest.raises(SystemExit):
        D.export(str(tmp_path / "x"), 1, ["pool"], scene="harbour", log=lambda *_: None)


def test_dock_poses_follow_the_bands():
    w = W.load(POOL)
    rng = random.Random(5)
    faces = W.bay_faces(w["objects"])
    got = {}
    for _ in range(300):
        x, y, hdg, band, bay = D.sample_dock_pose(w, rng, 51.0)
        f = faces[bay]
        d = 45.0 - x                                                               # the pool's faces look south (x_n 45)
        got[band] = got.get(band, 0) + 1
        assert D.inside([tuple(p) for p in w["bounds"]], x, y)
        if band == "docked":
            assert 1.1 < d < 2.6 and abs(y - f["y_e"]) < 0.45 and min(abs(hdg), abs(hdg - 360)) <= 15.01
        elif band == "approach":
            assert 2.5 < d < 15.1
    assert set(got) == {"approach", "docked", "passing"} and got["approach"] > got["docked"] > got["passing"] * 0.8


def test_the_render_cli_and_the_sim_page_camera(tmp_path, brokers):
    """`render --dataset --scene dock` from the command line; simview follows sim/dock/state (--boat-port) into /frame.npz."""
    import subprocess
    import sys
    out = tmp_path / "cli"
    p = subprocess.run([sys.executable, "-m", "optiprimex.sim.render", "--dataset", str(out), "--n", "3", "--scene", "mix", "--size", "160x90",
                        "--world", "pool", "--seed", "2"], capture_output=True, text=True, timeout=120, cwd=ROOT)
    assert p.returncode == 0, p.stderr[-800:]
    assert json.loads((out / "manifest.json").read_text())["scene"] == "mix" and "indicator" in (out / "data.yaml").read_text()
    from optiprimex.sim import simview as SV
    import paho.mqtt.publish as publish
    a = SV.parse_args(["--world", "pool", "--fcu-control", str(free_udp_port()), "--port", str(free_port()), "--hub-port", "0",
                       "--boat-port", str(brokers.boat_port), "--width", "160", "--height", "90"])
    assert SV.parse_args([]).boat_port == 0                                        # off unless the launcher asks (run_sim.sh: 1884)
    sim = SV.Sim(a)
    sim.watch_dock()
    try:
        st = {"green_bay": 1, "phase": "lit", "window": {"bay": 1, "window": 2, "colour": "red"}}
        publish.single("sim/dock/state", json.dumps({"schema_version": 1, "body": st}), hostname="127.0.0.1", port=brokers.boat_port, retain=True)
        assert wait_until(lambda: sim.dock_state is not None and sim.dock_state.get("green_bay") == 1, 10)
        sim.pose = (43.5, 2.0, 0.0)                                                # 1.5 m off bay 1
        z = np.load(io.BytesIO(sim.frame_npz(320, 180)))
        meta = json.loads(str(z["meta"]))
        kinds = dict(zip(z["prim_code"].tolist(), z["prim_kind"].tolist()))
        objs = dict(zip(z["prim_code"].tolist(), z["prim_obj"].tolist()))
        assert meta["dock_state"]["window"]["window"] == 2 and {"dock", "window", "indicator", "cube", "deck"} <= set(kinds.values())
        lit = [c for c, k in kinds.items() if k == "window" and objs[c] == 32]
        px = z["rgb"][np.isin(z["ids"], lit)].astype(int)
        assert len(px) and (px[:, 0] > 200).mean() > 0.4                          # window 2 of bay 1 glows RED in the camera
        assert sim.world_json()["dock_state"]["green_bay"] == 1
    finally:
        sim.dock_client.loop_stop()
        publish.single("sim/dock/state", b"", hostname="127.0.0.1", port=brokers.boat_port, retain=True)
        time.sleep(0.1)
