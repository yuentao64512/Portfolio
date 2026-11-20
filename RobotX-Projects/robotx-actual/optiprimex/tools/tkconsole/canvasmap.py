"""K6: the tile-less Canvas map. The frame: ENU metres centred on the course polygon's centroid (the scout sketch's projection; an
equirectangular projection - < 1 cm error over a 200 m course). Sources: /course.json (the polygon, lat/lon), GET /map.json (B v0.1.83.5:
the boat's map digest in local N/E metres around its origin `co` - entities, gates, path, carrot, points, scout's target and visited ids),
/fast.json (the boat at 5 Hz: lat, lon, heading).

The geometry and the layer (`build_layer`) are pure and tested without Tk; CanvasMap only draws: zoom by the wheel (around the cursor),
pan by drag, F = follow the boat, `fit` = the polygon (or everything) in view. The fence / course editing stays on the web map ↗."""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

M_PER_DEG = 111320.0
INSET_M = 2.0
OBSTACLE_RING_M = 1.5

LL = Tuple[float, float]
XY = Tuple[float, float]          # (east, north) metres


class Frame:
    def __init__(self, lat0: float, lon0: float):
        self.lat0, self.lon0 = lat0, lon0
        self.k = math.cos(math.radians(lat0))

    def en(self, lat: float, lon: float) -> XY:
        return ((lon - self.lon0) * M_PER_DEG * self.k, (lat - self.lat0) * M_PER_DEG)

    def ll(self, e: float, n: float) -> LL:
        return (self.lat0 + n / M_PER_DEG, self.lon0 + e / (M_PER_DEG * self.k))


def ring(pts: Sequence[Sequence[float]]) -> List[LL]:
    out = [(float(p[0]), float(p[1])) for p in pts if p is not None and len(p) >= 2]
    if len(out) > 1 and out[0] == out[-1]:
        out = out[:-1]                                           # handbook 3.4.9: first == last
    return out


def centroid(pts: Sequence[XY]) -> XY:
    """The area centroid (the vertex mean for a degenerate ring)."""
    a = cx = cy = 0.0
    n = len(pts)
    for i in range(n):
        x0, y0 = pts[i]
        x1, y1 = pts[(i + 1) % n]
        c = x0 * y1 - x1 * y0
        a += c
        cx += (x0 + x1) * c
        cy += (y0 + y1) * c
    if abs(a) < 1e-12:
        return (sum(p[0] for p in pts) / n, sum(p[1] for p in pts) / n) if n else (0.0, 0.0)
    return (cx / (3 * a), cy / (3 * a))


def frame_for(polygon_ll: Sequence[LL], fallback: Optional[LL] = None) -> Optional[Frame]:
    if polygon_ll:
        lat0 = sum(p[0] for p in polygon_ll) / len(polygon_ll)
        f0 = Frame(lat0, sum(p[1] for p in polygon_ll) / len(polygon_ll))
        c = centroid([f0.en(*p) for p in polygon_ll])
        return Frame(*f0.ll(*c))
    return Frame(*fallback) if fallback else None


def inset(poly: Sequence[XY], d: float = INSET_M) -> List[XY]:
    """Each edge moved d metres inward, consecutive edges intersected (the course quads are convex; a reflex corner is kept as is)."""
    n = len(poly)
    if n < 3:
        return []
    area = sum(poly[i][0] * poly[(i + 1) % n][1] - poly[(i + 1) % n][0] * poly[i][1] for i in range(n))
    s = 1.0 if area > 0 else -1.0                                # CCW: inward = left of the edge
    lines = []
    for i in range(n):
        (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % n]
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy) or 1.0
        nx, ny = -dy / L * s, dx / L * s
        lines.append(((x0 + nx * d, y0 + ny * d), (dx, dy)))
    out = []
    for i in range(n):
        (p, r), (q, u) = lines[i - 1], lines[i]
        den = r[0] * u[1] - r[1] * u[0]
        if abs(den) < 1e-9:
            out.append(q)
            continue
        t = ((q[0] - p[0]) * u[1] - (q[1] - p[1]) * u[0]) / den
        out.append((p[0] + r[0] * t, p[1] + r[1] * t))
    return out


def build_layer(course: Optional[dict], mapj: Optional[dict], pose_ll: Optional[LL], hdg: Optional[float],
                visited_log: Sequence[dict] = ()) -> Dict[str, object]:
    """Everything the Canvas draws, in the ENU frame (east, north metres). Pure."""
    c = course or {}
    rec = (c.get("received") or {}).get("boundary") or []
    d = c.get("default")
    dflt = (d.get("boundary") if isinstance(d, dict) else d) or []
    poly_ll = ring(rec or dflt)
    src = "received" if rec else ("default" if dflt else "")
    m = (mapj or {}).get("map") or {}
    co = m.get("co") or {}
    origin = (float(co["lat"]), float(co["lon"])) if co.get("lat") is not None else None
    fr = frame_for(poly_ll, pose_ll or origin)
    layer: Dict[str, object] = {"frame": fr, "polygon": [], "inset": [], "polygon_src": src, "boat": None, "items": [],
                                "obstacles": [], "points": [], "path": [], "carrot": None, "target": None, "visited": set(),
                                "geofence": [], "age": (mapj or {}).get("age"), "map_source": (mapj or {}).get("source")}
    if fr is None:
        return layer
    poly = [fr.en(*p) for p in poly_ll]
    layer["polygon"], layer["inset"] = poly, inset(poly)
    layer["geofence"] = [fr.en(*p) for p in ring(c.get("geofence") or [])]

    def local(xn: float, ye: float) -> XY:                     # the map digest's local N/E (origin co) -> this frame
        if origin is None:
            return (ye, xn)
        return fr.en(origin[0] + xn / M_PER_DEG, origin[1] + ye / (M_PER_DEG * math.cos(math.radians(origin[0]))))

    sc = m.get("scout") or {}
    visited = {int(i) for i in sc.get("visited_ids") or [] if isinstance(i, (int, float))}
    for e in m.get("entities") or []:
        if e.get("x_n") is None or e.get("y_e") is None:
            continue
        x, y = local(float(e["x_n"]), float(e["y_e"]))
        it = {"id": e.get("id"), "e": x, "n": y, "kind": e.get("kind", "?"), "colour": e.get("colour") or "unknown",
              "sigma": float(e.get("sigma_m") or 0.0), "source": e.get("source") or "usv", "state": e.get("state", ""),
              "visited": e.get("id") in visited, "outside": bool(e.get("outside_course"))}       # B WP46: never a task target
        if it["kind"] == "obstacle":
            it["ring"] = float(e.get("ring_m") or e.get("clear_m") or OBSTACLE_RING_M)
            layer["obstacles"].append(it)
        else:
            layer["items"].append(it)
    for o in m.get("obstacles") or []:                         # a separate list when the digest carries one
        if isinstance(o, dict) and o.get("x_n") is not None:
            x, y = local(float(o["x_n"]), float(o["y_e"]))
            layer["obstacles"].append({"id": o.get("id"), "e": x, "n": y, "kind": "obstacle",
                                       "ring": float(o.get("ring_m") or OBSTACLE_RING_M)})
    layer["visited"] = visited
    tgt = sc.get("target")
    layer["target"] = next(((it["e"], it["n"]) for it in layer["items"] if it["id"] == tgt), None) if tgt is not None else None
    layer["path"] = [local(float(p[0]), float(p[1])) for p in m.get("path") or [] if len(p) >= 2]
    if m.get("carrot"):
        layer["carrot"] = local(float(m["carrot"][0]), float(m["carrot"][1]))
    pll = m.get("points_ll") or {}
    for name, xy in sorted((m.get("points") or {}).items()):
        p = pll.get(name) or {}
        if p.get("lat") is not None:
            e, n = fr.en(float(p["lat"]), float(p["lon"]))
        elif isinstance(xy, (list, tuple)) and len(xy) >= 2:
            e, n = local(float(xy[0]), float(xy[1]))
        else:
            continue
        layer["points"].append({"name": name, "e": e, "n": n})
    if not visited:                                             # the scout log lines when the digest has no ids (older boats)
        layer["visited_ll"] = [fr.en(v["lat"], v["lon"]) for v in visited_log]
    if pose_ll is not None:
        layer["boat"] = (*fr.en(*pose_ll), hdg)
    elif m.get("boat"):
        b = m["boat"]
        layer["boat"] = (*local(float(b["x_n"]), float(b["y_e"])), b.get("hdg_deg"))
    return layer


def bounds(layer: dict, everything: bool = False) -> Optional[Tuple[float, float, float, float]]:
    pts: List[XY] = list(layer.get("polygon") or [])
    if everything or not pts:
        pts += [(i["e"], i["n"]) for i in layer.get("items") or []] + [(p["e"], p["n"]) for p in layer.get("points") or []]
        if layer.get("boat"):
            pts.append(layer["boat"][:2])
    if not pts:
        return None
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    return min(xs), min(ys), max(xs), max(ys)


COLOUR = {"red": "#d93a3a", "green": "#2a9d4b", "black": "#222222", "yellow": "#e0b400", "white": "#f2f2f2", "blue": "#2f6fd6",
          "orange": "#ff8c42", "unknown": "#9aa6b3", "": "#9aa6b3"}


class CanvasMap:
    """The MAP tab's Canvas (Tk, main thread only)."""

    def __init__(self, parent, pal: dict):
        import tkinter as tk
        self.tk, self.pal = tk, pal
        self.c = tk.Canvas(parent, background=pal["mapBg"], highlightthickness=0)
        self.scale = 6.0                     # px per metre
        self.cx, self.cy = 0.0, 0.0          # the view centre, metres
        self.follow = False
        self.layer: dict = {}
        self._fitted = False
        self._drag = None
        c = self.c
        c.bind("<MouseWheel>", self._wheel)                      # macOS / Windows
        c.bind("<Button-4>", lambda e: self._zoom(1.2, e.x, e.y))  # X11
        c.bind("<Button-5>", lambda e: self._zoom(1 / 1.2, e.x, e.y))
        c.bind("<ButtonPress-1>", self._press)
        c.bind("<B1-Motion>", self._move)
        c.bind("<Configure>", lambda e: self.redraw())
        c.bind("<Key-f>", lambda e: self.toggle_follow())
        c.bind("<Key-F>", lambda e: self.toggle_follow())
        c.bind("<Enter>", lambda e: c.focus_set())

    # ---- view ------------------------------------------------------------------------------------------------------------
    def px(self, e: float, n: float) -> Tuple[float, float]:
        w, h = max(1, self.c.winfo_width()), max(1, self.c.winfo_height())
        return (w / 2 + (e - self.cx) * self.scale, h / 2 - (n - self.cy) * self.scale)

    def m(self, x: float, y: float) -> XY:
        w, h = max(1, self.c.winfo_width()), max(1, self.c.winfo_height())
        return (self.cx + (x - w / 2) / self.scale, self.cy - (y - h / 2) / self.scale)

    def fit(self, everything: bool = False) -> None:
        b = bounds(self.layer, everything)
        if not b:
            return
        x0, y0, x1, y1 = b
        w, h = max(100, self.c.winfo_width()), max(100, self.c.winfo_height())
        self.cx, self.cy = (x0 + x1) / 2, (y0 + y1) / 2
        self.scale = max(0.2, min(80.0, 0.85 * min(w / max(1.0, x1 - x0), h / max(1.0, y1 - y0))))
        self.follow = False
        self.redraw()

    def toggle_follow(self) -> None:
        self.follow = not self.follow
        self.redraw()

    def _wheel(self, e) -> None:
        self._zoom(1.15 if e.delta > 0 else 1 / 1.15, e.x, e.y)

    def _zoom(self, f: float, x: float, y: float) -> None:
        mx, my = self.m(x, y)
        self.scale = max(0.2, min(200.0, self.scale * f))
        nx, ny = self.m(x, y)
        self.cx, self.cy = self.cx + (mx - nx), self.cy + (my - ny)          # the point under the cursor stays put
        self.redraw()

    def _press(self, e) -> None:
        self._drag = (e.x, e.y, self.cx, self.cy)

    def _move(self, e) -> None:
        if not self._drag:
            return
        x0, y0, cx, cy = self._drag
        self.cx, self.cy = cx - (e.x - x0) / self.scale, cy + (e.y - y0) / self.scale
        self.follow = False
        self.redraw()

    # ---- data ------------------------------------------------------------------------------------------------------------
    def set_layer(self, layer: dict) -> None:
        self.layer = layer
        if not self._fitted and layer.get("polygon") and self.c.winfo_width() > 50:
            self._fitted = True
            self.fit()
            return
        self.redraw()

    def redraw(self) -> None:
        c, p, L = self.c, self.pal, self.layer
        c.delete("all")
        if not L or L.get("frame") is None:
            c.create_text(12, 12, anchor="nw", text="no course / no position yet", fill=p["dim"], font=("Helvetica", 12))
            return
        if self.follow and L.get("boat"):
            self.cx, self.cy = L["boat"][0], L["boat"][1]
        self._grid()

        def poly(pts, **kw):
            if len(pts) >= 2:
                xy = [v for q in pts + [pts[0]] for v in self.px(*q)]
                c.create_line(*xy, **kw)
        poly(L.get("geofence") or [], fill=p["mag"], width=1, dash=(2, 4), tags=("geofence",))
        poly(L.get("polygon") or [], fill=p["acc"], width=2, dash=() if L.get("polygon_src") == "received" else (6, 4), tags=("polygon",))
        poly(L.get("inset") or [], fill=p["acc"], width=1, dash=(4, 4), tags=("inset",))
        if len(L.get("path") or []) >= 2:
            c.create_line(*[v for q in L["path"] for v in self.px(*q)], fill=p["warn"], width=2, tags=("path",))
        for o in L.get("obstacles") or []:
            x, y = self.px(o["e"], o["n"])
            r = o["ring"] * self.scale
            c.create_oval(x - r, y - r, x + r, y + r, outline=p["alarm"], dash=(3, 3), tags=("obstacle",))
            c.create_line(x - 5, y - 5, x + 5, y + 5, fill=p["alarm"], width=2, tags=("obstacle",))
            c.create_line(x - 5, y + 5, x + 5, y - 5, fill=p["alarm"], width=2, tags=("obstacle",))
        for it in L.get("items") or []:
            x, y = self.px(it["e"], it["n"])
            col = COLOUR.get(str(it["colour"]).lower(), COLOUR["unknown"])
            if it.get("outside"):                                  # WP46: a neighbouring course's buoy - drawn, greyed, never a target
                c.create_oval(x - 5, y - 5, x + 5, y + 5, outline=p["dim"], dash=(2, 2), tags=("item", "outside"))
                c.create_text(x + 8, y - 8, anchor="sw", text=f"B{it['id']} outside", fill=p["dim"], font=("Menlo", 9), tags=("label",))
                continue
            s = max(2.0, it["sigma"] * self.scale)
            if s > 5:
                c.create_oval(x - s, y - s, x + s, y + s, outline=col, dash=(2, 3), tags=("sigma",))
            uav = it["source"] in ("uav", "both")
            r = 6
            if it["source"] == "uav":
                c.create_rectangle(x - r, y - r, x + r, y + r, fill=col, outline=p["text"], tags=("item", "uav"))
            else:
                c.create_oval(x - r, y - r, x + r, y + r, fill=col, outline=p["text"], width=2 if uav else 1, tags=("item",))
            lab = f"B{it['id']}" + (" ✓" if it["visited"] else "") + (" uav" if it["source"] == "uav" else (" +uav" if it["source"] == "both" else ""))
            c.create_text(x + 9, y - 9, anchor="sw", text=lab, fill=p["ok"] if it["visited"] else p["text"], font=("Menlo", 10, "bold"),
                          tags=("label",))
            if it["visited"]:
                c.create_text(x, y, text="✓", fill="#ffffff", font=("Helvetica", 9, "bold"), tags=("visited",))
        for v in L.get("visited_ll") or []:
            x, y = self.px(*v)
            c.create_text(x, y - 12, text="✓", fill=p["ok"], font=("Helvetica", 11, "bold"), tags=("visited",))
        if L.get("target"):
            x, y = self.px(*L["target"])
            c.create_oval(x - 11, y - 11, x + 11, y + 11, outline=p["warn"], width=3, tags=("target",))
        if L.get("carrot"):
            x, y = self.px(*L["carrot"])
            c.create_oval(x - 3, y - 3, x + 3, y + 3, fill=p["warn"], outline="", tags=("carrot",))
        for pt in L.get("points") or []:
            x, y = self.px(pt["e"], pt["n"])
            c.create_polygon(x, y - 6, x + 5, y + 4, x - 5, y + 4, fill=p["mag"], outline="", tags=("point",))
            c.create_text(x + 7, y + 2, anchor="w", text=pt["name"], fill=p["mag"], font=("Menlo", 10), tags=("point",))
        if L.get("boat"):
            self._boat(*L["boat"])
        self._legend()

    def _boat(self, e: float, n: float, hdg: Optional[float]) -> None:
        x, y = self.px(e, n)
        a = math.radians(hdg or 0.0)
        pts = [(0, 14), (-7, -9), (0, -4), (7, -9)]                # a dart, nose = heading (0 = north, clockwise)
        xy = []
        for dx, dy in pts:
            xe = dx * math.cos(a) + dy * math.sin(a)
            yn = -dx * math.sin(a) + dy * math.cos(a)
            xy += [x + xe, y - yn]
        self.c.create_polygon(*xy, fill=self.pal["acc"], outline=self.pal["text"], width=1, tags=("boat",))

    def _grid(self) -> None:
        c, p = self.c, self.pal
        w, h = c.winfo_width(), c.winfo_height()
        step = next(s for s in (1, 2, 5, 10, 20, 50, 100, 200, 500, 1000) if s * self.scale >= 60)
        e0, n1 = self.m(0, 0)
        e1, n0 = self.m(w, h)
        e = math.floor(e0 / step) * step
        while e <= e1:
            x = self.px(e, 0)[0]
            c.create_line(x, 0, x, h, fill=p["grid"], tags=("grid",))
            e += step
        n = math.floor(n0 / step) * step
        while n <= n1:
            y = self.px(0, n)[1]
            c.create_line(0, y, w, y, fill=p["grid"], tags=("grid",))
            n += step
        c.create_text(8, h - 8, anchor="sw", text=f"grid {step} m", fill=p["dim"], font=("Menlo", 10), tags=("grid",))

    def _legend(self) -> None:
        L, p = self.layer, self.pal
        n_v = sum(1 for i in L.get("items") or [] if i["visited"])
        n_out = sum(1 for i in L.get("items") or [] if i.get("outside"))
        txt = (f"course {L.get('polygon_src') or '—'} (solid = received, dashed = profile default) · 2 m inset dashed · "
               f"{len(L.get('items') or []) - n_out} buoys · {n_v} visited ✓ · {len(L.get('obstacles') or [])} obstacles"
               + (f" · {n_out} outside course (ignored)" if n_out else "")
               + (f" · map {L['age']:.1f} s old" if isinstance(L.get("age"), (int, float)) else " · no map digest")
               + (" · FOLLOW (F)" if self.follow else " · F = follow"))
        self.c.create_text(8, 8, anchor="nw", text=txt, fill=p["muted"], font=("Helvetica", 11), width=max(200, self.c.winfo_width() - 16),
                           tags=("legend",))
