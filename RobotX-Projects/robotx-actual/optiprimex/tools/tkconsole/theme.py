"""The page's palettes as Tk colours (K1): light (default) and dark, read from tools/themes.css so the console and the page share one
source; a built-in copy (themes.css as of v0.1.83.1) is used when the file is missing or unreadable."""
from __future__ import annotations

import os
import re
from typing import Dict

THEMES_CSS = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "themes.css")

_FALLBACK = {
    "light": {"bg": "#eef1f4", "panel": "#ffffff", "panel2": "#e4e9ee", "line": "#b9c3cd", "line2": "#d7dee5", "text": "#111820",
              "muted": "#4f5b67", "dim": "#74808c", "acc": "#007a97", "accText": "#075a70", "accBg": "#dff3f8", "warn": "#bb7725",
              "warnBg": "#fff0c7", "warnText": "#7a4800", "alarm": "#b9000f", "alarmBg": "#fde0e0", "alarmText": "#a00a12",
              "chipBg": "#f3f5f7", "chipText": "#1f2a35", "btnBg": "#ffffff", "btnLine": "#aab5c0", "grid": "#e1e6eb", "mapBg": "#f8fafb",
              "ok": "#1a7f37", "mag": "#8250df", "up": "#0a4f1d", "action": "#ffd60a", "actionLine": "#c20d16", "actionText": "#111111",
              "actionPale": "#fff4c4", "alarmPale": "#fde4e4"},
    "dark": {"bg": "#0e1318", "panel": "#151c24", "panel2": "#1b242e", "line": "#2f3b49", "line2": "#1f2a35", "text": "#e6ebf0",
             "muted": "#9aa6b3", "dim": "#6b7785", "acc": "#4bb9d2", "accText": "#a4e6f4", "accBg": "#12252c", "warn": "#ffc239",
             "warnBg": "#2a230f", "warnText": "#f6cf6b", "alarm": "#f94d4e", "alarmBg": "#331416", "alarmText": "#ff8a8a",
             "chipBg": "#1b242e", "chipText": "#cfd7df", "btnBg": "#1b242e", "btnLine": "#3a4756", "grid": "#151d27", "mapBg": "#0b1016",
             "ok": "#3fb950", "mag": "#d2a8ff", "up": "#b0f5c0", "action": "#7a5a00", "actionLine": "#ff5a5a", "actionText": "#ffffff",
             "actionPale": "#3a2f0e", "alarmPale": "#3a1618"},
}
_SEL = {"light": r':root\[data-theme="light"\]', "dark": r':root(?:,:root\[data-theme="dark"\])?'}


def parse_css(text: str, theme: str) -> Dict[str, str]:
    """Every `--name:#hex` of the rule blocks whose selector is the theme's (dark = `:root` / `:root[data-theme="dark"]`)."""
    out: Dict[str, str] = {}
    for sel, body in re.findall(r"([^{}]+)\{([^{}]*)\}", text):
        sels = [s.strip() for s in sel.split(",")]
        want = ('[data-theme="light"]' in s for s in sels) if theme == "light" else (s in (":root", ':root[data-theme="dark"]') for s in sels)
        if not any(want):
            continue
        for k, v in re.findall(r"--([A-Za-z0-9]+)\s*:\s*(#[0-9a-fA-F]{3,8})", body):
            out[k] = v
    return out


def palette(theme: str = "light", path: str = THEMES_CSS) -> Dict[str, str]:
    theme = theme if theme in _FALLBACK else "light"
    pal = dict(_FALLBACK[theme])
    try:
        with open(path, encoding="utf-8") as fh:
            pal.update(parse_css(fh.read(), theme))
    except OSError:
        pass
    return pal


# a View level -> (foreground, background) tokens; a word always travels with the colour (colour never alone)
LEVEL = {"ok": ("ok", "chipBg"), "warn": ("warnText", "warnBg"), "bad": ("alarmText", "alarmBg"), "idle": ("dim", "chipBg"),
         "info": ("accText", "accBg"), "": ("chipText", "chipBg")}


def level_colours(pal: Dict[str, str], level: str):
    fg, bg = LEVEL.get(level, LEVEL[""])
    return pal[fg], pal[bg]


# ---- design tokens (2026-11-12, the layout pass: one type scale, one spacing scale, used by every panel) ------------------------
# type: 5 sizes, ≥ 11 px everywhere (read on a sunlit deck), numerals in a monospace face so digits do not jump
FONT = {"small": ("Helvetica", 11), "body": ("Helvetica", 12), "label": ("Helvetica", 11, "bold"), "head": ("Helvetica", 13, "bold"),
        "title": ("Helvetica", 15, "bold"), "mono": ("Menlo", 12), "mono_b": ("Menlo", 13, "bold"), "big": ("Menlo", 44, "bold"),
        "big_label": ("Helvetica", 14, "bold"), "big_sub": ("Menlo", 16)}
# spacing: a 4-px grid - xs inside a row, s between rows, m inside a card, l between cards, xl around a panel
SPACE = {"xs": 4, "s": 8, "m": 12, "l": 16, "xl": 24}
