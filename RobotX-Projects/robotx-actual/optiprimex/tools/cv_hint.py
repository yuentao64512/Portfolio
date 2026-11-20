"""cv_hint: which CV model feeds which task (v0.1.49, WP10k-b item 23) - one line from perception/health, shared by the boat window's
camera callout and the OCS page's /state.json `cv_hint` (OVERVIEW's system card, MISSION CONTROL's mission line).

  CV: cvt-v2-l · 5 classes · buoys → T1 gates · dock / window / indicator → T3 · box → m0t4

cv_publish sends `model` and `classes` (v0.1.49: from --model's classes.yaml / PROVENANCE.md, the sim observer's payload, else
synthetic / students best.pt); fake_perception sends neither (a world-file stand-in, no model). Nothing is guessed: no health = none.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Tuple

STALE_S = 2.0
# class name -> (the words in the hint, the tasks that consume it - wire names as status/usv says them)
USE = (("buoy", "buoys", "T1 gates", ("T1", "TEST_PATH")),
       ("dock", "dock", "T3", ("T3", "TEST_SWAY")),
       ("window", "window", "T3", ("T3",)),
       ("indicator", "indicator", "T3", ("T3",)),
       ("light", "light", "T3", ("T3",)),
       ("box", "box", "m0t4", ("BOX_INSPECT",)))


def _use(cls: str) -> Optional[Tuple[str, str, Tuple[str, ...]]]:
    c = cls.lower()
    for key, word, dest, tasks in USE:
        if key in c:
            return word, dest, tasks
    return None


def hint(sources: List[Tuple[str, dict, float]]) -> Optional[dict]:
    """sources = [(name, health body, age s)]; the freshest that names a model wins, else the freshest. None = no health at all."""
    if not sources:
        return None
    named = [s for s in sources if (s[1] or {}).get("model")]
    name, body, age = min(named or sources, key=lambda s: s[2])
    body = body or {}
    model = body.get("model")
    classes = body.get("classes")
    if not model:
        model = "%s (no model: world-file stand-in)" % name if name == "fake_perception" else "%s (model not reported)" % name
    if not isinstance(classes, list):
        classes = None
    by_dest: Dict[str, List[str]] = {}
    by_task: Dict[str, List[str]] = {}
    for c in classes or []:
        u = _use(str(c))
        if u is None:
            continue
        word, dest, tasks = u
        if word not in by_dest.setdefault(dest, []):
            by_dest[dest].append(word)
        for t in tasks:
            if str(c) not in by_task.setdefault(t, []):
                by_task[t].append(str(c))
    parts = ["CV: %s" % model]
    if classes is not None:
        parts.append("%d class%s" % (len(classes), "" if len(classes) == 1 else "es"))
    parts += ["%s → %s" % (" / ".join(ws), dest) for dest, ws in by_dest.items()]
    if classes is None and name != "fake_perception":
        parts.append("classes not reported")
    stale = age > STALE_S
    return {"text": " · ".join(parts), "model": model, "classes": classes, "detector": body.get("detector") or name,
            "observer_detector": body.get("observer_detector"), "source": name, "age_s": round(age, 1), "stale": stale,
            "by_task": by_task, "sources": [{"name": n, "age_s": round(a, 1)} for n, _b, a in sorted(sources, key=lambda s: s[0])]}
