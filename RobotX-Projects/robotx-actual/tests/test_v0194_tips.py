"""v0.1.94.3: the Tk launcher's hover tooltips (Dmitrii 14 Nov)."""
import pytest

from optiprimex.tools import launcher
from optiprimex.tools.launcher_tips import CHOICE_TIPS, ROW_TIPS, choice_tip
from optiprimex.tools.launcher_tk import BOOL_KEYS, ROWS


def test_every_choice_has_its_own_tip():
    fields = dict(list(launcher.FIELDS.items()) + list(launcher.OPTIONAL_FIELDS.items()))
    for _, key, _, kind, _ in ROWS:
        if kind == "seg":
            for c in fields.get(key, ()):
                assert str(c) in CHOICE_TIPS.get(key, {}), (key, c)
        if kind == "bool":
            assert set(CHOICE_TIPS[key]) == {"true", "false"}, key
        if kind in ("seg", "bool", "num"):
            assert ROW_TIPS.get(key), key


def test_choice_tip_bool_and_fallback():
    assert choice_tip("lucky", True).startswith("on:")
    assert choice_tip("judge", "auto").endswith("never the venue DEMO")
    assert choice_tip("nope", "x") == "nope: x"


def test_tooltip_shows_after_hover_and_hides():
    tk = pytest.importorskip("tkinter")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("no display")
    try:
        from optiprimex.tools.launcher_tk import Tooltip
        b = tk.Button(root, text="x"); b.pack(); root.update()
        t = Tooltip(b, lambda: "hello")
        t._enter(); assert t.job is not None
        t._show(); assert t.top is not None
        t._hide(); assert t.top is None and t.job is None
    finally:
        root.destroy()
