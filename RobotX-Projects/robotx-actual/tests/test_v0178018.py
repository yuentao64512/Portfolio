"""v0.1.78.18 (workstream A): the colleague's two-stage recognizer as a verbatim snapshot (cv/buoys/two_stage/), Model A in our
buoy observer (cv/buoys_multi --detector model-a: boxes -> an inner-ellipse mask -> the students' measure_mask -> our contract),
the colour precedence with our HSV vote as the backup (--colour auto|model|hsv), and score_sim (detect-capable scoring, class map)."""
from __future__ import annotations

import hashlib
import importlib.util
import math
import re
import subprocess
import sys

import numpy as np
import pytest

from .conftest import ROOT

CV = ROOT / "cv"
TWO = CV / "buoys" / "two_stage"


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    m = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(m)
    return m


TS = load("two_stage_core_v0178018", CV / "buoys_multi" / "two_stage_core.py")
SC = load("score_sim_v0178018", TWO / "score_sim.py")
NAMES = {0: "buoy_led_red", 1: "buoy_led_green", 2: "buoy_no_led", 3: "dock", 4: "home", 5: "obstacle"}


def test_snapshot_is_verbatim():
    """Every file in PROVENANCE.md's table still has the colleague's hash, and is read-only (integration, not improvement)."""
    rows = re.findall(r"^\| (\w+\.\w+)(?: \([^)]*\))? \| ([0-9a-f]{64}) \|", (TWO / "PROVENANCE.md").read_text(), re.M)
    assert len(rows) == 10
    for name, digest in rows:
        p = TWO / name
        assert hashlib.sha256(p.read_bytes()).hexdigest() == digest, name
        assert not p.stat().st_mode & 0o222, f"{name} should be read-only"


def test_buoy_classes_only():
    assert TS.buoy_class_ids(NAMES) == [0, 1, 2]
    assert TS.buoy_class_ids(list(NAMES.values())) == [0, 1, 2]


@pytest.mark.parametrize("mode,cls,conf,hsv,want", [
    ("auto", "buoy_led_red", 0.80, "green", ("red", "model")),       # Model A's LED wins when confident
    ("auto", "buoy_led_green", 0.30, "green", ("green", "hsv")),     # low score -> our vote
    ("auto", "buoy_led_red", 0.30, None, ("unknown", "hsv")),
    ("auto", "buoy_no_led", 0.90, "blue", ("blue", "hsv")),           # Model A has no blue: Task 1's entry / exit
    ("model", "buoy_no_led", 0.90, None, ("black", "model")),         # unlit -> cv_publish lit = False
    ("model", "buoy_led_green", 0.10, "red", ("green", "model")),
    ("hsv", "buoy_led_red", 0.99, "green", ("green", "hsv")),         # the backup: ours, whatever Model A says
])
def test_colour_precedence(mode, cls, conf, hsv, want):
    assert TS.pick_colour(mode, cls, conf, hsv, led_conf=0.5) == want


def test_colour_mode_checked():
    with pytest.raises(ValueError):
        TS.pick_colour("vote", "buoy_led_red", 0.9, "red")


def test_box_mask_inner_ellipse():
    m = TS.box_mask(200, 300, [100, 50, 200, 150], shrink=0.8)
    assert m.shape == (200, 300) and m.dtype == np.float32
    assert m[100, 150] == 1.0                                         # the centre
    assert m[52, 102] == 0.0 and m[148, 198] == 0.0                   # the box's corners (water / sky) stay out
    assert m[100, 109] == 0.0 and m[100, 111] == 1.0                  # semi-axis 40 px: the edge band stays out too
    assert m.sum() == pytest.approx(math.pi * 40 * 40, rel=0.03)
    with pytest.raises(ValueError):
        TS.inner_ellipse([0, 0, 10, 10], shrink=0.0)


def test_score_sim_matching():
    names = {0: "buoy", 1: "dock_face"}
    truths = SC.truth_boxes("0 0.1 0.1 0.2 0.1 0.2 0.2 0.1 0.2\n1 0.5 0.5 0.2 0.2\n", 100, 100, names)
    assert truths[0] == ("buoy", pytest.approx([10, 10, 20, 20])) and truths[1] == ("dock_face", pytest.approx([40, 40, 60, 60]))
    cmap = SC.parse_map(["buoy_led_red=buoy", "dock=dock_face"])
    dets = [("buoy_led_red", [10, 10, 20, 21]), ("home", [41, 40, 60, 60]), ("obstacle", [80, 80, 90, 90])]
    pairs, extra = SC.score_frame(truths, dets, cmap)
    assert pairs == [("buoy", "buoy"), ("dock_face", "home")] and extra == ["obstacle"]
    from collections import Counter
    s = SC.summarise(Counter(pairs + [("buoy", "missed")]), Counter(extra))
    assert s["per_class"]["buoy"] == {"truth": 2, "found": 1, "right_class": 1, "recall": 0.5, "class_recall": 0.5}
    assert s["per_class"]["dock_face"]["recall"] == 1.0 and s["per_class"]["dock_face"]["class_recall"] == 0.0
    with pytest.raises(ValueError):
        SC.parse_map(["dock"])


def test_observer_wiring():
    """The adapter is a branch of our loop: same measure_mask, same contract; the default (mask) path untouched."""
    src = (CV / "buoys_multi" / "multi_buoy_observer.py").read_text()
    assert 'choices=TS.DETECTORS, default="mask"' in src and 'choices=TS.COLOUR_MODES, default="auto"' in src
    assert "TS.box_mask(" in src and src.count("colour_for(frame, mask, box)") == 2
    assert 'task="detect" if boxes_only else "segment"' in src
    assert "meas = S.measure_mask(xyz, mask, max_depth_m=a.max_depth)" in src      # the students' function, unchanged call


def test_run_sh_model_a_path(tmp_path):
    """run.sh picks Model A for --detector model-a (CV_MODEL_A overrides) and says so when the file is missing."""
    r = subprocess.run(["bash", str(CV / "buoys_multi" / "run.sh"), "--headless", "--detector", "model-a"], capture_output=True,
                       text=True, env={"PATH": "/usr/bin:/bin", "HOME": str(tmp_path), "CV_MODEL_A": str(tmp_path / "nope.pt")})
    assert r.returncode == 1 and "nope.pt" in r.stderr and "CV_MODEL_A" in r.stderr


def test_recorder_reads_the_version_under_an_ascii_locale(tmp_path):
    """v0.1.78.18: the AGX recorder died at start under tmux (LANG unset -> ascii): kit_version read __init__.py (a non-ASCII byte)
    without an encoding. Every open() in rec_core / multi_core now says utf-8."""
    (tmp_path / "optiprimex").mkdir()
    (tmp_path / "optiprimex" / "__init__.py").write_text('"""v0.1 — the kit"""\n__version__ = "9.9.9"\n', encoding="utf-8")
    code = (f"import sys; sys.path.insert(0, {str(CV / 'buoys_multi')!r}); import rec_core as R; "
            f"print(R.kit_version({str(tmp_path)!r}))")
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True,
                       env={"PATH": "/usr/bin:/bin", "LANG": "C", "LC_ALL": "C", "PYTHONUTF8": "0", "PYTHONCOERCECLOCALE": "0"})
    assert r.returncode == 0 and r.stdout.strip() == "9.9.9", r.stderr
    for f in ("rec_core.py", "multi_core.py"):
        src = (CV / "buoys_multi" / f).read_text()
        assert all("encoding" in line for line in src.splitlines() if re.search(r"\bopen\(", line)), f
