"""v0.1.39.3: the three test tiers (quick / live / test) and the parallel run's one rule - a test file stays on one process
(`--dist loadfile`): test_integration_sil1's tests share a stack and build on each other (declaration_seq), split they fail."""
from __future__ import annotations

import re

from .conftest import ROOT


def test_the_tiers_and_the_parallel_rule():
    mk = (ROOT / "Makefile").read_text()
    body = lambda t: re.search(rf"^{t}:.*\n((?:\t.*\n)+)", mk, flags=re.M).group(1)
    assert "-n $(JOBS) --dist loadfile" in body("test") and "--dist loadfile" in body("live")
    assert "-n" not in body("test-serial") and 'not ($(LONG_LIVE))' in body("live")
    long_live = re.search(r"^LONG_LIVE = (.*)$", mk, flags=re.M).group(1).split(" or ")
    src = "".join(p.read_text() for p in (ROOT / "tests").glob("test_*.py"))
    for name in long_live:                                   # each long mission still exists (a rename would silently un-skip it)
        assert re.search(rf"def test_\w*{re.escape(name)}\w*\(", src), name
    assert "pytest-xdist" in (ROOT / "requirements.txt").read_text()
