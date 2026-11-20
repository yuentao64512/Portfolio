"""v0.1.94 (Dmitrii 2026-11-14: "bringing back number classifier"): cv/buoys_multi/number_core + the observer's --number wiring.
The one-to-one rule is the student's DP (agx_track_10_buoys.py unique_digit_assignment), pinned here against its source when present."""
import ast
import os
import re
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "cv" / "buoys_multi"))
import number_core as NC  # noqa: E402

OBS = (ROOT / "cv" / "buoys_multi" / "multi_buoy_observer.py").read_text()
STUDENT = Path.home() / "Downloads" / "agx_track_10_buoys.py"


def vec(**kw):
    v = [0.0] * 10
    for k, p in kw.items():
        v[int(k[1:])] = p
    return v


def test_assignment_one_digit_per_frame():
    # two boxes both want 8: the stronger keeps it, the other takes its own next best (or none)
    a = NC.unique_assignment([vec(d8=0.9, d3=0.6), vec(d8=0.7)], 0.5)
    assert a == {0: 3, 1: 8}                     # total 0.6 + 0.7 = 1.3 > 0.9 alone
    b = NC.unique_assignment([vec(d8=0.95), vec(d8=0.7)], 0.5)
    assert sorted(b.values()) == [8] and len(b) == 1


def test_assignment_low_score_is_unknown():
    assert NC.unique_assignment([vec(d4=0.3)], 0.5) == {}
    assert NC.unique_assignment([], 0.5) == {}
    assert NC.unique_assignment([vec(d4=0.6), vec(d1=0.2)], 0.5) == {0: 4}


def test_assignment_body_matches_student_source():
    if not STUDENT.exists():
        pytest.skip("the student's file is not on this machine")
    def body(src, name):
        fn = next(n for n in ast.walk(ast.parse(src)) if isinstance(n, ast.FunctionDef) and n.name == name)
        lines = src.splitlines()[fn.body[1].lineno - 1:fn.end_lineno]      # after the docstring
        return [l.strip() for l in lines if l.strip() and not l.strip().startswith("#")]
    ours, theirs = body((ROOT / "cv/buoys_multi/number_core.py").read_text(), "unique_assignment"), body(STUDENT.read_text(), "unique_digit_assignment")
    norm = lambda L: [re.sub(r"state\.number_scores\[digit\]|scores\[state\]\[digit\]", "S", l)
                      .replace("len(states)", "N").replace("len(scores)", "N").replace("states", "X").replace("scores", "X") for l in L]
    diff = [(a, b) for a, b in zip(norm(ours), norm(theirs)) if a != b]
    # the only allowed differences: the chosen list (indices vs objects) and the returned key (index vs track_id)
    assert all("chosen" in a or "state" in a for a, _ in diff), diff
    assert len(diff) <= 3, diff


def test_near_gate():
    assert NC.near([500, 500, 560, 540], 7.9)
    assert not NC.near([500, 500, 560, 540], 8.1)
    assert NC.near([500, 500, 600, 620], None)          # bearing-only, 120 px tall
    assert not NC.near([500, 500, 600, 580], None)      # 80 px: far
    assert not NC.near([0, 500, 100, 700], 5.0)          # cut by the left edge
    assert not NC.near([1850, 500, 1919, 700], 5.0)      # cut by the right edge
    assert NC.near([500, 900, 700, 1080], 4.0)           # the bottom edge is fine (near markers fill it)
    assert NC.near([500, 500, 560, 540], 10.0, {"max_m": 12})


def test_crop_pad_and_bounds():
    im = np.zeros((1080, 1920, 3), np.uint8)
    c = NC.crop(im, [100, 100, 200, 300], 0.15)
    assert c.shape[:2] == (260, 160)
    assert NC.crop(im, [1900, 1070, 1920, 1080], 0.15) is not None


class FakeReader(NC.NumberReader):
    def __init__(self, table):
        super().__init__("none")
        self.table, self.seen = table, 0

    def probs(self, crops):
        self.seen += len(crops)
        return [self.table[i] for i in range(len(crops))]


def test_read_output_shape_and_batch():
    im = np.zeros((1080, 1920, 3), np.uint8)
    rd = FakeReader([(vec(d5=0.97), "5"), (vec(d5=0.6, d2=0.55), "5")])
    out = rd.read(im, [[300, 300, 400, 450], [900, 300, 1000, 450], [1200, 600, 1220, 630]], [None, 6.0, 20.0])
    assert rd.seen == 2                                 # the far one never reaches the classifier
    assert out[0] == {"digit": 5, "p": 0.97, "cls": "5"}
    assert out[1] == {"digit": 2, "p": 0.55, "cls": "5"}   # 5 is taken this frame -> its next best
    assert out[2] is None


def test_read_c_marker_is_unknown():
    im = np.zeros((1080, 1920, 3), np.uint8)
    rd = FakeReader([([0.01] * 10, "C")])
    out = rd.read(im, [[300, 300, 400, 450]], [None])
    assert out[0]["digit"] is None and out[0]["cls"] == "C"


def test_observer_number_off_by_default_and_wired():
    assert re.search(r'"--number", choices=\("on", "off"\), default="off"', OBS)
    assert '"--number-max-m", type=float, default=8.0' in OBS
    assert 'b["number"] = n' in OBS
    assert 'if getattr(a, "number", "off") == "on"' in OBS
    assert "import number_core as NC" in OBS                     # lazy: off never imports it


def test_model_file_present():
    p = ROOT / "cv" / "buoys_multi" / "models" / "number_cls.pt"
    assert p.exists() and p.stat().st_size > 10_000_000
