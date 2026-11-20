"""Task 3 dock geometry from the PerceptionMap (ROADMAP WP3): the bay faces numbered 1..n left to right when facing the
dock (handbook 3.3.4), the line's normal toward the water, and the standoff points. One function, `dock_geometry`, is the
only place the task reads the dock from - the students' box core (BOX_INSPECT, docs/merge/) can feed it later.

v0.1.58 (WP13, D14; Dmitrii 2026-11-07 "detect docking box key points (centre + boundaries) for at least 2 boxes, establish the line
and offset it, use it as the sway-search path"): `source="boxes"` builds the same DockGeom from the face key points of the box
tracks (`BoxTrack.stable()` left / midpoint / right) instead of the map's dock entities - `fit_line` through every face point
(total least squares), its normal toward the boat; the standoff line offset from it is the SWAY_SEARCH path (`DockGeom.sway_path`).
Fewer than 2 stable faces = no line (None; `box_faces` says how many there are). Default stays `map`.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Tuple

P = Tuple[float, float]


@dataclass
class Bay:
    number: int                       # 1..n left to right facing the dock
    face: P                           # the face point (building front) in local N/E
    colour: str                       # the indicator: green | red | ...
    entity_id: int


@dataclass
class DockGeom:
    centre: P
    normal: P                         # unit, from the face to the water (toward the boat)
    along: P                          # unit, left -> right when facing the dock
    bays: List[Bay]
    source: str = "map"               # v0.1.58: map | boxes
    rms_m: float = 0.0                # v0.1.58: the face points' rms distance from the fitted line (boxes)

    @property
    def bow_in_hdg(self) -> float:
        return math.degrees(math.atan2(-self.normal[1], -self.normal[0])) % 360.0

    def green(self) -> Optional[Bay]:
        g = [b for b in self.bays if b.colour == "green"]
        return g[0] if len(g) == 1 else None

    def abeam(self, bay: Bay, standoff: float) -> P:
        return (bay.face[0] + standoff * self.normal[0], bay.face[1] + standoff * self.normal[1])

    def on_standoff(self, p: P, standoff: float) -> P:
        """The point of the standoff line (parallel to the dock, `standoff` m out) nearest to p."""
        a = (self.centre[0] + standoff * self.normal[0], self.centre[1] + standoff * self.normal[1])
        s = (p[0] - a[0]) * self.along[0] + (p[1] - a[1]) * self.along[1]
        return (a[0] + s * self.along[0], a[1] + s * self.along[1])


    def sway_path(self, standoff: float, margin: float = 0.0) -> Tuple[P, P]:
        """v0.1.58: the sway-search path - the dock line offset `standoff` m toward the water, from `margin` m beyond the first bay
        to as far beyond the last (left -> right facing the dock)."""
        ss = [(b.face[0] - self.centre[0]) * self.along[0] + (b.face[1] - self.centre[1]) * self.along[1] for b in self.bays] or [0.0]
        a, b = min(ss) - margin, max(ss) + margin
        return tuple((self.centre[0] + s_ * self.along[0] + standoff * self.normal[0],
                      self.centre[1] + s_ * self.along[1] + standoff * self.normal[1]) for s_ in (a, b))


def fit_line(points: List[P], toward: Optional[P] = None) -> Optional[Tuple[P, P, P, float]]:
    """v0.1.58: the total-least-squares line through points -> (centre, normal (toward `toward`), along (left -> right facing the
    line from the normal's side), rms_m), None with fewer than 2 distinct points."""
    if len(points) < 2:
        return None
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    sxx = sum((p[0] - cx) ** 2 for p in points)
    syy = sum((p[1] - cy) ** 2 for p in points)
    sxy = sum((p[0] - cx) * (p[1] - cy) for p in points)
    if sxx + syy < 1e-6:
        return None
    theta = 0.5 * math.atan2(2 * sxy, sxx - syy)
    dn, de = math.cos(theta), math.sin(theta)
    nn, ne = -de, dn
    if toward is not None and (nn * (toward[0] - cx) + ne * (toward[1] - cy)) < 0:
        nn, ne = -nn, -ne
    rms = math.sqrt(sum(((p[0] - cx) * nn + (p[1] - cy) * ne) ** 2 for p in points) / len(points))
    return (cx, cy), (nn, ne), (ne, -nn), rms


BOX_STABLE = {"observation_s": 1.0, "min_samples": 3, "min_colour_share": 0.6, "require_blink": False, "min_span_frac": 0.6}


def box_faces(pmap, stable_kw: Optional[dict] = None) -> List[dict]:
    """v0.1.58: the stable faces with key points - the dock faces' tracks (camera / fake dock items with face_points) when there
    are 2 or more, else the box tracks (m0t4's boxes). Each {box_id, entity_id, face (FaceGeometry), colour}: the colour is the
    dock entity's indicator colour when it has votes, else the box face's colour."""
    if pmap is None:
        return []
    kw = dict(BOX_STABLE, **(stable_kw or {}))
    out = []
    for kind, tracks in (("dock", getattr(pmap, "dock_tracks", None) or {}), ("box", getattr(pmap, "box_tracks", None) or {})):
        out = []
        for tr in tracks.values():
            sb = tr.stable(colours=("red", "green", "blue", "white"), **kw)
            if sb is None:
                continue
            m = sb.face.midpoint
            ent = pmap.ents.get(sb.box_id) if kind == "dock" else None         # a dock track is keyed by its map entity
            if ent is None:
                ent = min((e for e in pmap.ents.values() if e.kind == kind and math.hypot(e.x_n - m.n, e.y_e - m.e) < 2.0),
                          key=lambda e: math.hypot(e.x_n - m.n, e.y_e - m.e), default=None)
            colour = sb.colour
            if ent is not None and getattr(ent, "ind_votes", None) and ent.colour in ("red", "green"):
                colour = ent.colour
            out.append({"box_id": sb.box_id, "entity_id": ent.id if ent is not None else sb.box_id, "face": sb.face, "colour": colour})
        if len(out) >= 2:
            return out
    return out


def boxes_geometry(faces: List[dict], pose=None, source: str = "boxes") -> Optional[DockGeom]:
    """v0.1.58: the DockGeom of >= 2 faces' key points (left, midpoint, right each): the fitted line, the bays numbered left -> right
    by their midpoints, the bay face = the face midpoint."""
    if len(faces) < 2:
        return None
    pts = [(p.n, p.e) for f in faces for p in (f["face"].left, f["face"].midpoint, f["face"].right)]
    fit = fit_line(pts, (pose.x_n, pose.y_e) if pose is not None else None)
    if fit is None:
        return None
    c, nrm, along, rms = fit
    fs = sorted(faces, key=lambda f: (f["face"].midpoint.n - c[0]) * along[0] + (f["face"].midpoint.e - c[1]) * along[1])
    bays = [Bay(i, (f["face"].midpoint.n, f["face"].midpoint.e), f["colour"], int(f["entity_id"])) for i, f in enumerate(fs, 1)]
    return DockGeom(c, nrm, along, bays, source=source, rms_m=round(rms, 3))


def dock_geometry(pmap, pose, now: Optional[float] = None, min_obs: int = 3, source: str = "map",
                  stable_kw: Optional[dict] = None) -> Optional[DockGeom]:
    if source == "boxes":                                      # v0.1.58 (WP13, D14)
        return boxes_geometry(box_faces(pmap, stable_kw), pose)
    if source != "map":
        raise ValueError(f"dock_source {source!r}: map | boxes")
    d = pmap.dock(pose, now) if pmap is not None else None
    if not d:
        return None
    psi = math.radians(d["psi_n"])
    nn, ne = math.cos(psi), math.sin(psi)
    sn, se = ne, -nn                                          # starboard when facing the dock = left -> right
    faces = [e for e in pmap.entities("dock") if e.id in d["faces"] and e.n_obs >= min_obs]
    faces.sort(key=lambda e: (e.x_n - d["x_n"]) * sn + (e.y_e - d["y_e"]) * se)
    bays = [Bay(i, (e.x_n, e.y_e), e.colour, e.id) for i, e in enumerate(faces, 1)]
    return DockGeom((d["x_n"], d["y_e"]), (nn, ne), (sn, se), bays)


def refit(geom: DockGeom, pmap) -> DockGeom:
    """v0.1.56 (D8): the locked dock line again from the map's current positions of the same face entities (any age - abeam one bay
    the camera no longer sees the others, so dock_geometry's fresh-faces line has too few): numbers / colours / ids kept, the faces,
    the centre and the normal (toward the water, as before) refitted. The line locked at FIND_DOCK from 20 m carries metres of error
    (the camera campaign: 17 deg off - the boat entered askew and backed out 0.24 m from a finger)."""
    ents = getattr(pmap, "ents", None) or {}
    pts = [(b, ents[b.entity_id]) for b in geom.bays if b.entity_id in ents]
    if len(pts) < 2:
        return geom
    cx = sum(e.x_n for _, e in pts) / len(pts)
    cy = sum(e.y_e for _, e in pts) / len(pts)
    sxx = sum((e.x_n - cx) ** 2 for _, e in pts)
    syy = sum((e.y_e - cy) ** 2 for _, e in pts)
    sxy = sum((e.x_n - cx) * (e.y_e - cy) for _, e in pts)
    th = 0.5 * math.atan2(2 * sxy, sxx - syy)
    nn, ne = -math.sin(th), math.cos(th)
    if nn * geom.normal[0] + ne * geom.normal[1] < 0:
        nn, ne = -nn, -ne
    bays = [Bay(b.number, (ents[b.entity_id].x_n, ents[b.entity_id].y_e) if b.entity_id in ents else b.face, b.colour, b.entity_id)
            for b in geom.bays]
    return DockGeom((cx, cy), (nn, ne), (ne, -nn), bays)


# ---- v0.1.56 (D6): the standoff lines from the dock geometry --------------------------------------------------------------------
FINGER_M = 3.5                        # the pontoon finger length from the bay line toward the water (world_pool's / world_extra's fingers)
HULL_LEN_M, CLEAR_M = 1.19, 0.60      # the BlueBoat's length; the 0.60 m rule of v0.1.54 (t1.clear_m)


def standoff_lines(t3: Optional[dict] = None, t1: Optional[dict] = None) -> Tuple[float, float]:
    """(standoff_m, standoff2_m): `finger_m + hull_len_m / 2 + clear_m + standoff_margin_m` (1.0) / `... + standoff2_margin_m` (0.3) -
    the sway lines keep the hull the 0.60 m rule plus a margin off the finger tips. A non-null `t3.standoff_m` / `standoff2_m` is an
    explicit override and wins (the TUNE knob sets it). hull_len_m / clear_m come from t3, else t1 (the clearance rule's section)."""
    t3, t1 = dict(t3 or {}), dict(t1 or {})

    def num(key, default):
        v = t3.get(key)
        if v is None:
            v = t1.get(key)
        return float(default if v is None else v)
    base = num("finger_m", FINGER_M) + num("hull_len_m", HULL_LEN_M) / 2.0 + num("clear_m", CLEAR_M)
    sd = t3.get("standoff_m")
    sd2 = t3.get("standoff2_m")
    return (float(sd) if sd is not None else round(base + num("standoff_margin_m", 1.0), 3),
            float(sd2) if sd2 is not None else round(base + num("standoff2_margin_m", 0.3), 3))


def dock_view(pmap, pose, now: Optional[float] = None, t3: Optional[dict] = None, t1: Optional[dict] = None) -> Optional[dict]:
    """v0.1.62 (Dmitrii 2026-11-07: "the CV boxes around the docking boxes and their surface points, then the sway path on the map"):
    what m3t1 would use right now, for the map digest (local/<v>/map `dock_view`) - the same dock_geometry / standoff_lines as the task
    (t3.dock_source map | boxes, the t3 / t1 sections): the faces' key points (left / midpoint / right, local N/E) from the tracks, the
    fitted line's centre / along / normal, the sway path at standoff_m (the inner one at standoff2_m) and the bays. None = no dock."""
    p = dict(t3 or {})
    src = str(p.get("dock_source") or "map")
    kw = {"observation_s": float(p.get("box_window_s", 1.0)), "min_samples": int(p.get("box_min_samples", 3))}
    faces = []
    for f in box_faces(pmap, kw):
        g = f["face"]
        faces.append({"colour": f.get("colour"), "left": [round(g.left.n, 2), round(g.left.e, 2)],
                      "mid": [round(g.midpoint.n, 2), round(g.midpoint.e, 2)], "right": [round(g.right.n, 2), round(g.right.e, 2)]})
    geom = dock_geometry(pmap, pose, now, source=src, stable_kw=kw)
    if geom is None and not faces:
        return None
    out = {"source": src, "faces": faces}
    if geom is not None:
        sd, sd2 = standoff_lines(p, t1)
        rnd = lambda q: [round(q[0], 2), round(q[1], 2)]                    # noqa: E731
        out.update(centre=rnd(geom.centre), along=[round(geom.along[0], 4), round(geom.along[1], 4)],
                   normal=[round(geom.normal[0], 4), round(geom.normal[1], 4)], rms_m=geom.rms_m, standoff_m=sd, standoff2_m=sd2,
                   sway=[rnd(q) for q in geom.sway_path(sd)], sway2=[rnd(q) for q in geom.sway_path(sd2)], geom_source=geom.source,
                   bays=[{"n": b.number, "colour": b.colour, "face": rnd(b.face)} for b in geom.bays])
    return out
