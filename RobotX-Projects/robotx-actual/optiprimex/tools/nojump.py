"""nojump: the no-layout-jump check of UI_QOL item 14 (v0.1.47) against a running OCS page (scripts/run_sim.sh / the launcher).

Headless Chrome loads the page with ?jumpcheck=N on each tab; the page records every visible card's height each second (after a 3 s
settle) and writes the changes into <pre id="jumpcheck">; this reads it. Exit 0 = no card changed height by more than 1 px.

  python -m optiprimex.tools.nojump [--page http://127.0.0.1:8096/] [--seconds 30] [--tabs OVERVIEW,MISSION,COMMS]
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def check(page: str, tab: str, seconds: int) -> dict:
    url = f"{page.rstrip('/')}/?jumpcheck={seconds}#{tab}"
    dom = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--window-size=1440,1000", f"--virtual-time-budget={(seconds + 6) * 1000}",
                          "--dump-dom", url], capture_output=True, text=True, timeout=seconds + 120).stdout
    return parse(dom)


def parse(dom: str) -> dict:
    """The last <pre id="jumpcheck"> whose text is a JSON object (the page's own comment mentions the tag)."""
    found = re.findall(r'<pre id="jumpcheck">(\{.*?)</pre>', dom, re.S)
    return json.loads(found[-1].replace("&quot;", '"').replace("&amp;", "&")) if found else {"error": "no result (the page did not finish)"}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--page", default="http://127.0.0.1:8096/")
    ap.add_argument("--seconds", type=int, default=30)
    ap.add_argument("--tabs", default="OVERVIEW,MISSION,COMMS")
    a = ap.parse_args(argv)
    bad = 0
    for tab in a.tabs.split(","):
        r = check(a.page, tab, a.seconds)
        n = len(r.get("changes", [])) if "changes" in r else None
        bad += 1 if n != 0 else 0
        print(f"{tab:10s} cards {r.get('cards')} changes {n if n is not None else r.get('error')}" +
              ("" if not n else "  " + "; ".join(f"{c['card']}: {c['from']}->{c['to']} at {c['s']} s" for c in r["changes"][:6])))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
