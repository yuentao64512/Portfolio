#!/usr/bin/env python3
"""md2pdf (v0.1.92): our Markdown docs -> the "manual" PDF style of docs/*_manual.pdf (the venue guides of 9 Nov): a small-caps running
header OPTIPRIME USV KIT, a light title + subtitle, ruled bold section heads, grey-header tables, tick boxes, a footer with the document
name and a bold page number. No dependencies: a small Markdown subset (# headings, paragraphs, - lists, | tables |, ``` code ```,
**bold**, *italic*, `code`, ☐) -> HTML -> headless Chrome --print-to-pdf.
usage: python3 scripts/md2pdf.py IN.md OUT.pdf ["the footer name"]   (or OUT.html to keep the HTML)"""
from __future__ import annotations

import html
import os
import re
import subprocess
import sys
import tempfile

CHROME = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
BASE = "."                                                       # image paths are relative to the .md (set in main)

CSS = """
@page { size: A4; margin: 14mm 14mm 14mm 14mm; }
:root { --ink:#111; --rule:#999; --soft:#555; --head:#eceef0; --code:#f3f4f6; }
* { box-sizing: border-box; }
html { -webkit-print-color-adjust: exact; print-color-adjust: exact; }
body { font: 400 8.6pt/1.42 'Helvetica Neue', Helvetica, Arial, sans-serif; color: var(--ink); margin: 0; }
.run { font-size: 7.5pt; letter-spacing: .14em; color: #555; border-bottom: .6pt solid var(--rule); padding-bottom: 4pt; margin-bottom: 18pt;
       display: flex; justify-content: space-between; } .run b { letter-spacing: 0; font-weight: 400; color: #333; }
h1 { font-weight: 300; font-size: 21pt; margin: 0 0 2pt; letter-spacing: -.01em; }
.sub { color: var(--soft); font-size: 10pt; margin: 0 0 12pt; padding-bottom: 10pt; border-bottom: .8pt solid var(--ink); }
h2 { font-size: 11.5pt; font-weight: 700; margin: 18pt 0 6pt; padding-bottom: 4pt; border-bottom: .8pt solid var(--ink); break-after: avoid; }
h3 { font-size: 9.6pt; font-weight: 700; margin: 12pt 0 4pt; break-after: avoid; }
h4 { font-size: 9pt; font-weight: 700; margin: 10pt 0 3pt; }
p { margin: 0 0 6pt; }
ul, ol { margin: 0 0 6pt 14pt; padding: 0; } li { margin: 0 0 2pt; }
table { width: 100%; border-collapse: collapse; margin: 4pt 0 9pt; font-size: 7.8pt; break-inside: auto; }
th { background: var(--head); text-align: left; font-weight: 700; padding: 4pt 5pt; border-top: .8pt solid var(--ink); border-bottom: .6pt solid var(--rule); }
td { padding: 3.5pt 5pt; border-bottom: .4pt solid #ccc; vertical-align: top; }
tr { break-inside: avoid; }
code { font: 7.6pt Menlo, Consolas, monospace; background: var(--code); padding: 0 2pt; border-radius: 2pt; }
pre { font: 7.4pt/1.35 Menlo, Consolas, monospace; background: var(--code); padding: 6pt 8pt; border-left: 2pt solid #bbb; white-space: pre-wrap;
      margin: 3pt 0 8pt; break-inside: avoid; }
pre code { background: none; padding: 0; }
hr { border: 0; border-top: .6pt solid var(--rule); margin: 12pt 0; }
.box { display: inline-block; width: 8pt; height: 8pt; border: .7pt solid #333; vertical-align: -1pt; margin-right: 3pt; }
blockquote { margin: 4pt 0 8pt; padding: 4pt 8pt; border-left: 2pt solid #111; background: #fafafa; }
.figs { display: flex; gap: 6pt; margin: 4pt 0 10pt; break-inside: avoid; align-items: flex-end; }
figure { margin: 0; flex: 1; text-align: center; } figure img { max-width: 100%; max-height: 118mm; object-fit: contain; }
figcaption { font-size: 7.2pt; color: #444; margin-top: 2pt; }
.end { margin-top: 16pt; font-size: 7.6pt; color: #333; border-top: .6pt solid var(--rule); padding-top: 3pt; }
"""


def cells(row: str):
    """A table row's cells: split on | outside `code` spans."""
    out, cur, code = [], "", False
    for ch in row.strip().strip("|"):
        if ch == "`":
            code = not code
        if ch == "|" and not code:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    out.append(cur.strip())
    return out


def inline(t: str) -> str:
    t = html.escape(t, quote=False)
    codes = []

    def keep(m):
        codes.append(m.group(1))
        return f"\x00{len(codes) - 1}\x00"
    t = re.sub(r"`([^`]+)`", keep, t)
    t = re.sub(r"\*\*([^*]+)\*\*", r"<b>\1</b>", t)
    t = re.sub(r"(?<![\w*])\*([^*\n]+)\*(?![\w*])", r"<i>\1</i>", t)
    t = t.replace("☐", '<span class="box"></span>')
    t = re.sub(r"\x00(\d+)\x00", lambda m: f"<code>{codes[int(m.group(1))]}</code>", t)
    return t


def convert(md: str) -> str:
    lines = md.splitlines()
    out, i, title_done = [], 0, False
    while i < len(lines):
        ln = lines[i]
        if ln.startswith("```"):
            j = i + 1
            buf = []
            while j < len(lines) and not lines[j].startswith("```"):
                buf.append(lines[j])
                j += 1
            out.append("<pre><code>" + html.escape("\n".join(buf)) + "</code></pre>")
            i = j + 1
            continue
        m = re.match(r"^(#{1,4})\s+(.*)$", ln)
        if m:
            lvl, txt = len(m.group(1)), m.group(2)
            if lvl == 1 and not title_done:
                head, _, sub = txt.partition(" (")
                out.append(f"<h1>{inline(head)}</h1>")
                out.append(f'<div class="sub">{inline("(" + sub) if sub else ""}</div>')
                title_done = True
            else:
                out.append(f"<h{lvl}>{inline(txt)}</h{lvl}>")
            i += 1
            continue
        if ln.strip().startswith("|") and i + 1 < len(lines) and re.match(r"^\s*\|[\s:|-]+\|\s*$", lines[i + 1]):
            hdr = cells(ln)
            rows = []
            j = i + 2
            while j < len(lines) and lines[j].strip().startswith("|"):
                rows.append(cells(lines[j]))
                j += 1
            t = "<table><thead><tr>" + "".join(f"<th>{inline(c)}</th>" for c in hdr) + "</tr></thead><tbody>"
            for r in rows:
                t += "<tr>" + "".join(f"<td>{inline(c)}</td>" for c in r) + "</tr>"
            out.append(t + "</tbody></table>")
            i = j
            continue
        if re.match(r"^\s*([-*]|\d+\.)\s+", ln):
            ordered = bool(re.match(r"^\s*\d+\.", ln))
            items, j = [], i
            while j < len(lines) and (re.match(r"^\s*([-*]|\d+\.)\s+", lines[j]) or (lines[j].startswith("   ") and lines[j].strip())):
                if re.match(r"^\s*([-*]|\d+\.)\s+", lines[j]):
                    items.append(re.sub(r"^\s*([-*]|\d+\.)\s+", "", lines[j]))
                else:
                    items[-1] += " " + lines[j].strip()
                j += 1
            tag = "ol" if ordered else "ul"
            out.append(f"<{tag}>" + "".join(f"<li>{inline(x)}</li>" for x in items) + f"</{tag}>")
            i = j
            continue
        if re.match(r"^!\[[^\]]*\]\([^)]+\)\s*$", ln):              # a row of figures (consecutive image lines)
            figs, j = [], i
            while j < len(lines) and re.match(r"^!\[[^\]]*\]\([^)]+\)\s*$", lines[j]):
                cap, src = re.match(r"^!\[([^\]]*)\]\(([^)]+)\)", lines[j]).groups()
                path = src if src.startswith(("/", "file:", "http")) else os.path.abspath(os.path.join(BASE, src))
                figs.append(f"<figure><img src='{'file://' + path if path.startswith('/') else path}'><figcaption>{inline(cap)}</figcaption></figure>")
                j += 1
            out.append(f"<div class='figs n{len(figs)}'>" + "".join(figs) + "</div>")
            i = j
            continue
        if ln.startswith(">"):
            out.append(f"<blockquote>{inline(ln.lstrip('> '))}</blockquote>")
            i += 1
            continue
        if ln.strip() in ("---", "***"):
            out.append("<hr>")
            i += 1
            continue
        if not ln.strip():
            i += 1
            continue
        buf, j = [ln], i + 1
        while j < len(lines) and lines[j].strip() and not re.match(r"^(#|\||```|\s*[-*]\s|\s*\d+\.\s|>)", lines[j]):
            buf.append(lines[j])
            j += 1
        out.append(f"<p>{inline(' '.join(x.strip() for x in buf))}</p>")
        i = j
    return "\n".join(out)


def page(md: str, foot: str) -> str:
    body = convert(md)
    return (f"<!doctype html><html><head><meta charset='utf-8'><title>{html.escape(foot)}</title><style>{CSS}</style></head><body>"
            f"<div class='run'><span>OPTIPRIME USV KIT</span><b>{html.escape(foot)}</b></div>{body}"
            f"<div class='end'>{html.escape(foot)} · end</div></body></html>")


def main() -> int:
    global BASE
    src, dst = sys.argv[1], sys.argv[2]
    BASE = os.path.dirname(os.path.abspath(src))
    md = open(src, encoding="utf-8").read()
    foot = sys.argv[3] if len(sys.argv) > 3 else os.path.splitext(os.path.basename(src))[0]
    doc = page(md, foot)
    if dst.endswith(".html"):
        open(dst, "w", encoding="utf-8").write(doc)
        return 0
    with tempfile.NamedTemporaryFile("w", suffix=".html", delete=False, encoding="utf-8") as fh:
        fh.write(doc)
        tmp = fh.name
    r = subprocess.run([CHROME, "--headless=new", "--disable-gpu", "--no-pdf-header-footer", "--allow-file-access-from-files",
                        f"--print-to-pdf={os.path.abspath(dst)}",
                        "file://" + tmp], capture_output=True, text=True, timeout=120)
    os.unlink(tmp)
    if not os.path.exists(dst):
        print(r.stderr[-800:], file=sys.stderr)
        return 1
    print(dst)
    return 0


if __name__ == "__main__":
    sys.exit(main())
