"""v0.1.83.4: every word the OCS page sends passes ocs_web's /cmd filter (bench_tui.INTENT_WORDS) and parses in ocs_cli - station_keep, usv,
uav, cancel_auto and hold_on_start were refused as "unknown command" (found by session A on the pool sim, 12:53)."""
from __future__ import annotations

import re

from optiprimex.tools import bench_tui as T
from optiprimex.tools import ocs_cli

from .conftest import ROOT


def test_every_word_the_page_sends_is_known():
    p = (ROOT / "optiprimex" / "tools" / "ocs_web.html").read_text()
    words = set(re.findall(r'send\(\s*["\'`]([a-z_0-9]+)', p)) | set(re.findall(r'ask\(`<b>([a-z_0-9]+)', p))
    known = T.INTENT_WORDS | T.PLANT_WORDS
    page_local = {"clear", "drop", "stop"}            # modal titles / page-side helpers (ask(`<b>clear ...`), not /cmd words
    missing = sorted(w for w in words - page_local if T.ALIAS.get(w, w) not in known)
    assert not missing, missing


def test_the_five_words_parse():
    for line in ("station_keep hold", "station_keep loiter", "usv ready on", "usv ready off", "uav ready on", "uav scan done",
                 "cancel_auto", "hold_on_start"):
        assert line.split()[0] in T.INTENT_WORDS, line
        ocs_cli.parse(line.split())
