"""ocs_tk: the OCS operator console in tkinter (Phase 1, Dmitrii 2026-11-11; docs/TK_CONSOLE.md, docs/PROMPT_TK_CONSOLE_2026-11-11.md).

A client of ocs_web's HTTP contract - the backend is untouched and the web page at :8096 stays the fallback:

  python -m optiprimex.tools.ocs_tk --ocs http://127.0.0.1:8096 [--fast-hz 5] [--theme light|dark]
  normally scripts/run_ocs_tk.sh [--ts|--lan] (the port from OCS_WEB_PORT via scripts/_netenv.sh), beside scripts/run_ocs.sh.

Needs a Python with Tk 8.6 (Homebrew: brew install python-tk@3.10 for the venv's python3.10; Apple's system Tk 8.5 is not usable)."""
from __future__ import annotations

import argparse
import os
import sys

from .. import __version__


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ocs", default=f"http://127.0.0.1:{os.environ.get('OCS_WEB_PORT', '8096')}", help="the OCS page's base URL (ocs_web)")
    ap.add_argument("--fast-hz", type=float, default=None, help="the /fast.json rate (the pose, the HUD); default: ⚙ SETTINGS (5)")
    ap.add_argument("--theme", choices=("light", "dark"), default=None, help="default: ⚙ SETTINGS · CONSOLE (light)")
    a = ap.parse_args(argv)
    from .tkconsole.panels import load_prefs              # 2026-11-12: the console's saved preferences (logs/ocs/tk_prefs.json)
    pr = load_prefs()
    a.theme = a.theme or pr.get("theme", "light")
    a.fast_hz = a.fast_hz if a.fast_hz is not None else float(pr.get("fast_hz", "5"))
    return a


def tk_problem() -> str:
    """'' when a usable Tk is here, else what to do."""
    try:
        import tkinter
    except ImportError:
        return (f"no tkinter in {sys.executable} - Homebrew: brew install python-tk@{sys.version_info.major}.{sys.version_info.minor} "
                "(or the python.org installer); Apple's system Tk is not usable")
    if tkinter.TkVersion < 8.6:
        return f"Tk {tkinter.TkVersion} in {sys.executable} is too old (8.6 needed): brew install python-tk@3.10"
    return ""


def main(argv=None) -> int:
    a = parse_args(argv)
    why = tk_problem()
    if why:
        print(f"[ocs_tk] {why}", file=sys.stderr)
        return 2
    import tkinter as tk
    from .tkconsole.views import App
    try:
        root = tk.Tk(className="ocs_tk")
    except tk.TclError as exc:
        print(f"[ocs_tk] no display: {exc}", file=sys.stderr)
        return 2
    root.title(f"OCS · … · v{__version__}")
    app = App(root, a.ocs, fast_hz=a.fast_hz, theme=a.theme)
    print(f"[ocs_tk] v{__version__}: the console on {a.ocs} (fast {a.fast_hz:g} Hz, theme {a.theme}); the page stays at {a.ocs}/", flush=True)
    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        app.close()
    left = app.threads_left()
    if left:
        print(f"[ocs_tk] threads still alive at exit: {left}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
