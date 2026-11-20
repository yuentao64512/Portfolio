"""
Label viewer / validator for YOLO detection labels and classifier crop folders.

Detection labels (hand or auto):
    python label_viewer.py det --images data/images --labels data/labels --names data.yaml
    python label_viewer.py det --images auto/review --labels auto/review --names data.yaml --out review.html

Classifier folders (train/0/*.jpg ... train/9/*.jpg):
    python label_viewer.py cls --root digits/train

Then open the .html in Chrome. Keys: arrows = prev/next, A = OK, F = needs fix,
Space = clear mark, Esc = close. "Export" downloads fix_list.txt (one image path per line)
and review.json. Marks are remembered in the browser.

Auto-flags shown on each image:
  EMPTY      no boxes (fine for negatives - check it really is empty water)
  BAD_CLASS  class id not in names
  OUT        box extends outside the image
  TINY       box smaller than 0.4% of image width/height
  DUP        two boxes overlap with IoU > 0.85 (double label)
  LOWCONF    confidence < 0.5 (only for auto-labels with a 6th column)
  MALFORMED  line that is not 5 or 6 numbers
"""
import argparse
import html
import json
import os
from pathlib import Path

IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}


def load_names(path):
    if not path:
        return {}
    p = Path(path)
    if p.suffix in (".yaml", ".yml"):
        try:
            import yaml
            names = yaml.safe_load(p.read_text())["names"]
        except ImportError:  # minimal fallback parser
            names, inside = {}, False
            for line in p.read_text().splitlines():
                if line.strip().startswith("names:"):
                    rest = line.split(":", 1)[1].strip()
                    if rest.startswith("["):
                        return {i: n.strip(" '\"") for i, n in enumerate(rest.strip("[]").split(","))}
                    inside = True
                    continue
                if inside:
                    s = line.strip()
                    if not s or not (line.startswith(" ") or s.startswith("-")):
                        break
                    if s.startswith("-"):
                        names[len(names)] = s[1:].strip(" '\"")
                    elif ":" in s:
                        k, v = s.split(":", 1)
                        names[int(k)] = v.strip(" '\"")
            return names
    else:
        names = [l.strip() for l in p.read_text().splitlines() if l.strip()]
    return dict(enumerate(names)) if isinstance(names, list) else {int(k): v for k, v in names.items()}


def iou(a, b):
    ax1, ay1, ax2, ay2 = a[0] - a[2] / 2, a[1] - a[3] / 2, a[0] + a[2] / 2, a[1] + a[3] / 2
    bx1, by1, bx2, by2 = b[0] - b[2] / 2, b[1] - b[3] / 2, b[0] + b[2] / 2, b[1] + b[3] / 2
    iw, ih = max(0, min(ax2, bx2) - max(ax1, bx1)), max(0, min(ay2, by2) - max(ay1, by1))
    inter = iw * ih
    union = a[2] * a[3] + b[2] * b[3] - inter
    return inter / union if union > 0 else 0


def parse_det(images, labels, names, out_dir):
    items = []
    for img in sorted(p for p in Path(images).rglob("*") if p.suffix.lower() in IMG_EXT):
        rel = img.relative_to(images)
        lbl = Path(labels) / rel.with_suffix(".txt")
        boxes, flags = [], set()
        if lbl.exists():
            for line in lbl.read_text().splitlines():
                parts = line.split()
                if not parts:
                    continue
                if len(parts) not in (5, 6):
                    flags.add("MALFORMED")
                    continue
                try:
                    c = int(float(parts[0]))
                    x, y, w, h = map(float, parts[1:5])
                    conf = float(parts[5]) if len(parts) == 6 else None
                except ValueError:
                    flags.add("MALFORMED")
                    continue
                if names and c not in names:
                    flags.add("BAD_CLASS")
                if x - w / 2 < -0.01 or y - h / 2 < -0.01 or x + w / 2 > 1.01 or y + h / 2 > 1.01:
                    flags.add("OUT")
                if w < 0.004 or h < 0.004:
                    flags.add("TINY")
                if conf is not None and conf < 0.5:
                    flags.add("LOWCONF")
                boxes.append({"c": c, "b": [x, y, w, h], "conf": conf})
        for i in range(len(boxes)):
            for j in range(i + 1, len(boxes)):
                if iou(boxes[i]["b"], boxes[j]["b"]) > 0.85:
                    flags.add("DUP")
        if not boxes:
            flags.add("EMPTY")
        confs = [b["conf"] for b in boxes if b["conf"] is not None]
        items.append({
            "id": str(rel),
            "src": os.path.relpath(img, out_dir).replace(os.sep, "/"),
            "boxes": boxes,
            "flags": sorted(flags),
            "minconf": min(confs) if confs else None,
            "classes": sorted({b["c"] for b in boxes}),
        })
    return items


def parse_cls(root, out_dir):
    items, names = [], {}
    for i, d in enumerate(sorted(p for p in Path(root).iterdir() if p.is_dir())):
        names[i] = d.name
        for img in sorted(p for p in d.rglob("*") if p.suffix.lower() in IMG_EXT):
            items.append({
                "id": str(img.relative_to(root)),
                "src": os.path.relpath(img, out_dir).replace(os.sep, "/"),
                "boxes": [], "flags": [], "minconf": None, "classes": [i],
            })
    return items, names


PAGE = r"""<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Label Viewer</title><style>
:root{--bg:#f6f6f4;--fg:#1d1d1b;--mut:#6b6b66;--card:#fff;--line:#deded8;--ok:#1f8a4c;--bad:#c4382f;--acc:#2a5bd7}
@media (prefers-color-scheme:dark){:root{--bg:#171716;--fg:#ecece8;--mut:#9a9a93;--card:#222220;--line:#363633}}
*{box-sizing:border-box}body{margin:0;background:var(--bg);color:var(--fg);font:14px system-ui,sans-serif}
header{position:sticky;top:0;z-index:5;background:var(--bg);border-bottom:1px solid var(--line);padding:10px 16px;display:flex;flex-wrap:wrap;gap:8px;align-items:center}
h1{font-size:16px;margin:0 12px 0 0}select,input,button{font:inherit;padding:5px 8px;border:1px solid var(--line);border-radius:6px;background:var(--card);color:var(--fg)}
button{cursor:pointer}#stats{color:var(--mut);font-size:13px;width:100%}
#grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(220px,1fr));gap:10px;padding:12px 16px}
.card{background:var(--card);border:2px solid var(--line);border-radius:8px;overflow:hidden;cursor:pointer}
.card.ok{border-color:var(--ok)}.card.fix{border-color:var(--bad)}
.wrap{position:relative;line-height:0}.wrap img{width:100%;display:block}
svg.ov{position:absolute;inset:0;width:100%;height:100%}.tag{position:absolute;transform:translateY(-100%);color:#fff;font-size:11px;line-height:1.3;padding:0 4px;border-radius:3px 3px 3px 0;white-space:nowrap;text-shadow:0 0 2px #000;pointer-events:none}.tag.big{font-size:14px}
.meta{padding:6px 8px;font-size:12px;line-height:1.4}.fn{color:var(--mut);word-break:break-all}
.flag{display:inline-block;background:var(--bad);color:#fff;border-radius:4px;padding:0 5px;margin:2px 3px 0 0;font-size:11px}
#modal{display:none;position:fixed;inset:0;background:rgba(0,0,0,.88);z-index:10;align-items:center;justify-content:center;flex-direction:column;padding:16px}
#modal .wrap{display:inline-block}#modal img{display:block}
#minfo{color:#fff;margin-top:10px;text-align:center;font-size:14px;line-height:1.6}
.legend span{display:inline-block;margin-right:10px;font-size:12px}.sw{display:inline-block;width:10px;height:10px;border-radius:2px;margin-right:4px;vertical-align:middle}
</style></head><body>
<header><h1>Label Viewer</h1>
<select id="fClass"><option value="">All classes</option></select>
<select id="fFlag"><option value="">All images</option><option value="__any">Any flag</option></select>
<select id="fMark"><option value="">Any mark</option><option value="none">Unreviewed</option><option value="ok">OK</option><option value="fix">Needs fix</option></select>
<select id="sort"><option value="name">Sort: name</option><option value="conf">Sort: lowest confidence</option><option value="flags">Sort: most flags</option></select>
<label><input type="checkbox" id="showLbl" checked> labels</label>
<button id="exp">Export</button>
<div id="stats"></div><div class="legend" id="legend"></div></header>
<div id="grid"></div>
<div id="modal"><div class="wrap" id="mwrap"></div><div id="minfo"></div></div>
<script>
const DATA=__DATA__, NAMES=__NAMES__, MODE="__MODE__", KEY="lv:"+location.pathname;
const PAL=["#e6194b","#3cb44b","#4363d8","#f58231","#911eb4","#42d4f4","#f032e6","#bfef45","#fabed4","#469990","#dcbeff","#9a6324","#fffac8","#800000","#aaffc3","#808000"];
const col=c=>PAL[c%PAL.length], nm=c=>NAMES[c]??("?"+c);
let marks={};try{marks=JSON.parse(localStorage.getItem(KEY)||"{}")}catch(e){}
const save=()=>{try{localStorage.setItem(KEY,JSON.stringify(marks))}catch(e){}};
const $=id=>document.getElementById(id);
Object.keys(NAMES).forEach(k=>$("fClass").add(new Option(nm(+k),k)));
[...new Set(DATA.flatMap(d=>d.flags))].sort().forEach(f=>$("fFlag").add(new Option(f,f)));
$("legend").innerHTML=Object.keys(NAMES).map(k=>`<span><i class="sw" style="background:${col(+k)}"></i>${nm(+k)}</span>`).join("");
function overlay(d,big){if(MODE==="cls"||!$("showLbl").checked)return"";
 let svg=`<svg class="ov" viewBox="0 0 1000 1000" preserveAspectRatio="none">`,tags="";
 d.boxes.forEach(b=>{const[x,y,w,h]=b.b,X=(x-w/2),Y=(y-h/2),c=col(b.c);
  svg+=`<rect x="${X*1000}" y="${Y*1000}" width="${w*1000}" height="${h*1000}" fill="none" stroke="${c}" stroke-width="${big?3:2.5}" vector-effect="non-scaling-stroke"/>`;
  const t=nm(b.c)+(b.conf!=null?" "+b.conf.toFixed(2):"");
  tags+=`<span class="tag${big?" big":""}" style="left:${Math.max(0,X*100)}%;top:${Math.max(0,Y*100)}%;background:${c}">${t}</span>`});
 return svg+"</svg>"+tags}
let view=[];
function render(){const fc=$("fClass").value,ff=$("fFlag").value,fm=$("fMark").value,s=$("sort").value;
 view=DATA.filter(d=>(fc===""||d.classes.includes(+fc))&&(ff===""||(ff==="__any"?d.flags.length:d.flags.includes(ff)))&&
  (fm===""||(fm==="none"?!marks[d.id]:marks[d.id]===fm)));
 if(s==="conf")view.sort((a,b)=>(a.minconf??2)-(b.minconf??2));
 else if(s==="flags")view.sort((a,b)=>b.flags.length-a.flags.length);
 $("grid").innerHTML=view.map((d,i)=>`<div class="card ${marks[d.id]||""}" data-i="${i}"><div class="wrap"><img loading="lazy" src="${d.src}">${overlay(d,false)}</div>
  <div class="meta"><div class="fn">${d.id}</div>${MODE==="cls"?"<b>"+nm(d.classes[0])+"</b> ":d.boxes.length+" boxes "}${d.minconf!=null?"· min conf "+d.minconf.toFixed(2):""}<br>${d.flags.map(f=>`<span class="flag">${f}</span>`).join("")}</div></div>`).join("");
 const nb=DATA.reduce((a,d)=>a+d.boxes.length,0),per={};DATA.forEach(d=>(MODE==="cls"?[{c:d.classes[0]}]:d.boxes).forEach(b=>per[b.c]=(per[b.c]||0)+1));
 const nOk=Object.values(marks).filter(m=>m==="ok").length,nFix=Object.values(marks).filter(m=>m==="fix").length;
 $("stats").textContent=`${view.length}/${DATA.length} shown · ${MODE==="cls"?"":nb+" boxes · "}flagged ${DATA.filter(d=>d.flags.length).length} · reviewed ${nOk} OK, ${nFix} fix · per class: `+
  Object.keys(per).sort((a,b)=>a-b).map(k=>nm(+k)+"="+per[k]).join(", ")}
let cur=-1;
function open(i){if(i<0||i>=view.length)return;cur=i;const d=view[i];
 $("mwrap").innerHTML=`<img id="mimg" src="${d.src}">${overlay(d,true)}`;
 $("mimg").onload=function(){const k=Math.min(innerWidth*.95/this.naturalWidth,innerHeight*.75/this.naturalHeight);this.style.width=this.naturalWidth*k+"px"};
 $("minfo").innerHTML=`${i+1}/${view.length} · ${d.id}<br>`+(MODE==="cls"?"class <b>"+nm(d.classes[0])+"</b>":d.boxes.map(b=>nm(b.c)+(b.conf!=null?" ("+b.conf.toFixed(2)+")":"")).join(", ")||"no boxes")+
  `<br>${d.flags.join(" ")}<br>mark: <b>${marks[d.id]||"—"}</b> &nbsp; [A] OK · [F] needs fix · [Space] clear · ←/→ · Esc`;
 $("modal").style.display="flex"}
$("grid").onclick=e=>{const c=e.target.closest(".card");if(c)open(+c.dataset.i)};
$("modal").onclick=e=>{if(e.target.id==="modal"){$("modal").style.display="none";render()}};
document.onkeydown=e=>{if($("modal").style.display!=="flex")return;const d=view[cur];
 if(e.key==="ArrowRight")open(cur+1);else if(e.key==="ArrowLeft")open(cur-1);
 else if(e.key==="Escape"){$("modal").style.display="none";render()}
 else if("aAfF ".includes(e.key)){e.preventDefault();if(e.key===" ")delete marks[d.id];else marks[d.id]=/a/i.test(e.key)?"ok":"fix";save();open(cur+1<view.length?cur+1:cur)}};
["fClass","fFlag","fMark","sort","showLbl"].forEach(id=>$(id).onchange=render);
function dl(name,text){const a=document.createElement("a");a.href=URL.createObjectURL(new Blob([text]));a.download=name;a.click()}
$("exp").onclick=()=>{dl("fix_list.txt",Object.keys(marks).filter(k=>marks[k]==="fix").join("\n"));
 setTimeout(()=>dl("review.json",JSON.stringify(marks,null,1)),300)};
render();
</script></body></html>"""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)
    d = sub.add_parser("det")
    d.add_argument("--images", required=True)
    d.add_argument("--labels", required=True)
    d.add_argument("--names", help="data.yaml or names.txt")
    d.add_argument("--out", default="label_viewer.html")
    c = sub.add_parser("cls")
    c.add_argument("--root", required=True)
    c.add_argument("--out", default="cls_viewer.html")
    a = ap.parse_args()

    out = Path(a.out).resolve()
    if a.mode == "det":
        names = load_names(a.names)
        items = parse_det(Path(a.images).resolve(), Path(a.labels).resolve(), names, out.parent)
        if not names:
            names = {c: str(c) for it in items for c in it["classes"]}
    else:
        items, names = parse_cls(Path(a.root).resolve(), out.parent)

    page = (PAGE.replace("__DATA__", json.dumps(items))
                .replace("__NAMES__", json.dumps({str(k): html.escape(str(v)) for k, v in names.items()}))
                .replace("__MODE__", a.mode))
    out.write_text(page, encoding="utf-8")
    flagged = sum(1 for i in items if i["flags"])
    print(f"{len(items)} images, {flagged} flagged -> {out}")


if __name__ == "__main__":
    main()