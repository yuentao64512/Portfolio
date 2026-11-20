"""tkconsole: the OCS operator console in tkinter (Phase 1, Dmitrii 2026-11-11 - docs/PROMPT_TK_CONSOLE_2026-11-11.md).

A client of ocs_web's HTTP contract (/state.json, /fast.json, /alarms.json, /meta.json, /course.json, POST /cmd, /ack, /system);
the backend is untouched and the web page stays the fallback. Modules:
  model.py      the pure view-model: state + fast + alarms -> View (strings and flags, never widgets), tested without Tk
  net.py        the poller thread and the one command worker (HTTP only in worker threads, results handed to the main thread)
  theme.py      the page's light / dark palettes (read from tools/themes.css) as Tk colours
  views.py      the Tk widgets, updated by diff from the View
"""
