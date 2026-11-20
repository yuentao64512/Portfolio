"""textmap: a north-up text map of the local frame for terminal consoles (the BOAT tab of op-tui).

Same conventions as usv_ui's MAP screen: x_n up, y_e right, one column = half a row in metres (terminal cells are
about 2:1), auto-fit around the control origin, the boat, the saved points, the path and the perception-map entities.
Pure function: returns rows of (char, style) plus the metres spanned by ten columns; no curses here.
"""
from __future__ import annotations

from typing import Dict, List, Optional, Sequence, Tuple

ARROWS = ["^", "/", ">", "\\", "v", "/", "<", "\\"]
ENT_CHAR = {"red": "R", "green": "G", "white": "W", "yellow": "Y", "black": "K", "blue": "B", "orange": "O"}

Cell = Tuple[str, str]


def render_text_map(width: int, height: int, points: Dict[str, Tuple[float, float]], vehicle: Optional[Tuple[float, float, float]],
                    target: Optional[Tuple[float, float]], trail: Sequence[Tuple[float, float]], min_span_m: float = 10.0,
                    path: Optional[Sequence[Tuple[float, float]]] = None, entities: Optional[Sequence[Tuple[float, float, str, str, bool]]] = None,
                    gates: Optional[Sequence[Tuple[float, float]]] = None, dock: Optional[Tuple[float, float]] = None) -> Tuple[List[List[Cell]], float]:
    """points {name: (x, y)}; vehicle (x, y, hdg_deg); target (x, y); trail [(x, y)]; path [(x, y)];
    entities [(x, y, kind, colour, stale)]; gates [(mid_x, mid_y)]; dock (x, y) of the fitted line centre."""
    pts: List[Tuple[float, float]] = [(0.0, 0.0)] + list(points.values())
    if vehicle:
        pts.append((vehicle[0], vehicle[1]))
    if target:
        pts.append(target)
    if path:
        pts.extend(path)
    for e in entities or []:
        pts.append((e[0], e[1]))
    if dock:
        pts.append(dock)
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    xc, yc = (max(xs) + min(xs)) / 2.0, (max(ys) + min(ys)) / 2.0
    span_x = max(max(xs) - min(xs), min_span_m) * 1.25
    span_y = max(max(ys) - min(ys), min_span_m) * 1.25
    s = max(span_x / max(1, height - 1), span_y / max(1, width - 1) * 2.0, 1e-6)      # metres per row
    r_mid, c_mid = (height - 1) / 2.0, (width - 1) / 2.0

    def to_rc(x: float, y: float) -> Tuple[int, int]:
        return int(round(r_mid - (x - xc) / s)), int(round(c_mid + (y - yc) / (s / 2.0)))

    grid: List[List[Cell]] = [[(" ", "") for _ in range(width)] for _ in range(height)]

    def put(r: int, c: int, ch: str, style: str, soft: bool = False) -> None:
        if 0 <= r < height and 0 <= c < width and (not soft or grid[r][c][0] == " "):
            grid[r][c] = (ch, style)

    for r in range(0, height, 3):
        for c in range(0, width, 6):
            put(r, c, ".", "dim")
    for (x, y) in path or []:
        r, c = to_rc(x, y)
        put(r, c, ".", "path", soft=True)
    for (x, y) in trail:
        r, c = to_rc(x, y)
        put(r, c, ".", "trail", soft=True)
    for (x, y, kind, colour, stale) in entities or []:
        r, c = to_rc(x, y)
        ch = "D" if kind == "dock" else ENT_CHAR.get(colour, "o")
        style = "ent_red" if colour == "red" else "ent_green" if colour == "green" else "ent"
        put(r, c, ch.lower() if stale else ch, "ent_stale" if stale else style)
    for (x, y) in gates or []:
        r, c = to_rc(x, y)
        put(r, c, "|", "gate", soft=True)
    if dock:
        r, c = to_rc(dock[0], dock[1])
        put(r, c, "=", "gate", soft=True)
    r, c = to_rc(0.0, 0.0)
    put(r, c, "+", "co")
    for name, (x, y) in sorted(points.items()):
        r, c = to_rc(x, y)
        put(r, c, name[0].upper(), "point")
    if target:
        r, c = to_rc(target[0], target[1])
        if 0 <= r < height and 0 <= c < width and grid[r][c][1] != "point":
            put(r, c, "x", "target")
    if vehicle:
        r, c = to_rc(vehicle[0], vehicle[1])
        put(r, c, ARROWS[int(round((vehicle[2] % 360.0) / 45.0)) % 8], "vehicle")
    return grid, 10 * (s / 2.0)


def digest_layers(body: dict):
    """Pull the drawable layers out of a telemetry/<v>/map digest (usv_ui or the mission manager)."""
    boat = body.get("boat") or {}
    vehicle = (boat["x_n"], boat["y_e"], boat.get("hdg_deg") or 0.0) if boat.get("x_n") is not None else None
    trail = [tuple(p) for p in (boat.get("trail") or [])]
    path = [tuple(p) for p in (body.get("path") or [])]
    carrot = tuple(body["carrot"]) if body.get("carrot") else None
    points = {k: tuple(v) for k, v in (body.get("points") or {}).items()}
    entities = [(e["x_n"], e["y_e"], e.get("kind", "buoy"), e.get("colour", "unknown"), e.get("state") != "tracked") for e in body.get("entities") or []]
    gates = [(g["mid_n"], g["mid_e"]) for g in body.get("gates") or []]
    d = body.get("dock")
    dock = (d["x_n"], d["y_e"]) if d else None
    return vehicle, trail, path, carrot, points, entities, gates, dock
