"""prefs: the one shared theme across the pages (v0.1.47, WP10k-a, docs/UI_QOL_2026-11-07.md item 6).

The flow (the source of truth is the launcher's choice; the OCS page's /prefs carries it at runtime):

  launcher select  --POST /prefs (8094)-->  launcher  (logs/launch/prefs.json, LAUNCH_THEME, the launch record's cfg.theme)
                                               |  sync(): pushes its theme to the OCS /prefs when the OCS comes up (first contact),
                                               |  adopts the OCS's theme when a page changed it there since the last look
                                               v
                                         OCS /prefs (8096: GET since v0.1.47, POST since v0.1.43)
                                               ^
  every other page (judge 8095, SIM 8097, boat window 8098, map) reads its own server's /prefs on load and every 2 s; its own
  selector POSTs there; the server proxies to the OCS (falls back to the launcher when the OCS is down). localStorage is a cache.

Helpers only (urllib, short timeouts); the servers call them. Ports from the environment: OCS_WEB_PORT (8096), LAUNCH_PORT (8094).
"""
from __future__ import annotations

import json
import os
import urllib.request
from typing import Optional

THEMES = ("dark", "light", "hc-dark", "hc-light", "fun")


def norm(theme) -> str:
    """A valid theme name, '' when not one (the old 'contrast' = hc-dark)."""
    t = str(theme or "").strip()
    t = "hc-dark" if t == "contrast" else t
    return t if t in THEMES else ""


def ocs_url() -> str:
    return os.environ.get("OCS_PREFS_URL") or f"http://127.0.0.1:{os.environ.get('OCS_WEB_PORT', '8096')}"


def launcher_url() -> str:
    return os.environ.get("LAUNCH_PREFS_URL") or f"http://127.0.0.1:{os.environ.get('LAUNCH_PORT', '8094')}"


def get_theme(base: str, timeout: float = 0.6) -> Optional[str]:
    """GET base/prefs -> the theme ('' = the server has none yet); None when the server does not answer."""
    try:
        with urllib.request.urlopen(base.rstrip("/") + "/prefs", timeout=timeout) as r:
            d = json.loads(r.read() or b"{}")
        return norm((d.get("prefs") or {}).get("theme"))
    except Exception:
        return None


def post_theme(base: str, theme: str, header: str, timeout: float = 0.6) -> bool:
    """POST base/prefs {theme} with the server's anti-CSRF header (X-OCS-Web / X-Launch / X-Judge-Web: 1)."""
    try:
        req = urllib.request.Request(base.rstrip("/") + "/prefs", data=json.dumps({"theme": theme}).encode(), method="POST",
                                     headers={"Content-Type": "application/json", header: "1"})
        with urllib.request.urlopen(req, timeout=timeout) as r:
            return bool(json.loads(r.read() or b"{}").get("ok"))
    except Exception:
        return False


def read_shared() -> dict:
    """For a page's server (judge_web, simview, boat_web): GET /prefs -> {theme, from: ocs | launcher | none}."""
    t = get_theme(ocs_url())
    if t is not None:
        return {"theme": t, "from": "ocs"}
    t = get_theme(launcher_url())
    return {"theme": t or "", "from": "launcher" if t is not None else "none"}


def write_shared(theme: str) -> dict:
    """For a page's server: POST /prefs -> the OCS (X-OCS-Web), or the launcher when the OCS does not answer."""
    t = norm(theme)
    if not t:
        return {"ok": False, "why": f"theme: one of {', '.join(THEMES)}"}
    if post_theme(ocs_url(), t, "X-OCS-Web"):
        return {"ok": True, "prefs": {"theme": t}, "to": "ocs"}
    if post_theme(launcher_url(), t, "X-Launch"):
        return {"ok": True, "prefs": {"theme": t}, "to": "launcher"}
    return {"ok": False, "prefs": {"theme": t}, "why": "neither the OCS page nor the launcher answers: this page only"}
