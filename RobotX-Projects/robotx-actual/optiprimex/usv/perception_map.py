"""PerceptionMap: body-frame observations -> local-frame entities, gates and the dock line (MODULES 9.1, decision D1).

The map is the geolocation authority on the boat: the detector never knows where it is. ingest() takes an
Observation and the boat pose *at the capture time* (the caller interpolates its pose buffer), converts every item
to local NED metres through companion.frames.body_to_local, associates it with the nearest tracked entity of the same
kind inside assoc_gate_m (else creates one), and updates position by an EMA whose weight shrinks with the evidence.
Entities not seen for max_age_s become stale (kept for the map digest, excluded from gates and the dock line); stale
for drop_age_s they are dropped. gates() pairs red and green buoys 1.5-6 m apart with the through-heading that keeps
red to starboard; dock() fits a total-least-squares line through the dock faces seen recently and orients its normal
toward the boat. Pure in-process state; no I/O.

v0.1.34 (WP10g, CV_TRAIN's REQUEST_localisation.md item 1, the pathing spec's ray rules 5.3): an item without a range (range_m 0 /
null: beyond the camera's depth) is a RAY - the boat's camera position, the azimuth, the colour - kept `ray_ttl_s`, refreshed when
seen again from about the same place, never fused into an entity by itself, dropped when a ranged buoy turns up along it. Two rays of
the same colour from boat positions whose azimuths cross by `tri_min_angle_deg` or more TRIANGULATE: their intersection is folded in
as a sighting (so a far buoy becomes a point as the boat moves; the ghost filter min_obs still applies) - only with
`perception.triangulate: true`: decided 2026-11-05 with Dmitrii, the default is the pathing_sim way (spec 5.3 / 410: rays are leads,
never fused; a position only when depth arrives along the ray; the depth limit raised to 25 m instead).

v0.1.48 (WP10j-b, docs/M3T1_SIM_PLAN.md 2.1): the dock from the camera. A face (class dock, white) is folded as before; an
`indicator` item is not an entity: it votes its colour (red / green) on the nearest face within assoc_gate_m, and the face's colour is
the majority of its last IND_RECENT indicator sightings (`ind_votes` keeps the count per colour for `cv status` / the digest,
`ind_hist` the timed sightings T3 reads for its per-bay votes and the wrong-call check). A dock item that carries red / green itself
(the geometric fake, the students' dock mapper) votes the same way. A window with a number (quality.window: 1 upper / 2 lower) is
associated only with a window of that number (the two of a bay are ~0.4 m apart on the rendered face); its `light` (off / red / green)
is the majority of its last WINDOW_RECENT sightings.
"""
from __future__ import annotations

import math
import time
from typing import Dict, List, Optional, Tuple

from ..companion.config import Config
from ..companion.frames import body_to_local, wrap180, wrap360
from ..companion.state import Gate, LocalPose, MapEntity, Observation, ObsItem

KIND_OF_TOPIC = {"buoys": "buoy", "buoy": "buoy", "dock": "dock", "window": "window", "aruco": "aruco",
                 "boxes": "box", "box": "box"}


LIT_MIN_SHARE = 0.25       # a buoy with at least this share of lit (non-black) sightings has that colour, else it is black
FLASH_MIN_BLACK = 0.25     # a lit buoy black in at least this share of its sightings is flashing (1 s on / 1 s off), else steady
LIGHT_MIN_VOTES = 6        # fewer sightings: the light pattern is "?" (colour still decided)
LIT_DOMINANT = 0.45        # the lit colour must hold this share of the lit sightings: wrong-colour frames on an unlit buoy spread
                           # evenly over red / green / blue (~33 % each), a real light keeps >= 50 % even at the range limit (v0.1.21)
LIT_DECIDE_VOTES = 16      # v0.1.39 (WP10h): below the lit share with fewer sightings than this but a lit colour seen twice = PENDING (colour
                           # "unknown"): a flashing light sampled 9 times reads 2 lit / 7 dark 9 % of the time - not yet an unlit obstacle
IND_RECENT = 15           # v0.1.48: a dock face's colour = the majority of its last this many indicator sightings
WINDOW_RECENT = 5         # v0.1.48: a window's light = the majority of its last this many sightings
LED_WINDOW_S = 6.0         # v0.1.92: the observer's blink memory (cv/buoys_multi/led_core: steady / flashing per box) - the sightings kept
LED_MIN = 3                #   at least this many decided sightings (flashing / steady + red / green / blue) ...
LED_AGREE = 0.6            #   ... and this share agreeing -> the entity's colour + light come from the LED, not the colour votes
LIT_OVER_SECOND = 2.0      # v0.1.34: ... and, on a buoy dark at least half the time, twice the next lit colour (with the real 51 deg camera an unlit obstacle at 22 m
                           # voted blue 6 / green 4 / red 1 / black 16 - noise; a real light at that range is ~3x its runner-up)


def classify_light(votes: Dict[str, int]) -> Tuple[str, str]:
    """(colour, light) from the colour votes: black frames are the off phase of a flashing light (handbook 3.3.2), not a
    colour, unless the buoy is (almost) never lit. light = flash | steady | off | ? (too few sightings)."""
    total = sum(votes.values())
    if total <= 0:
        return "unknown", "?"
    black = votes.get("black", 0)
    lit = {k: v for k, v in votes.items() if k != "black"}
    n_lit = sum(lit.values())
    if n_lit == 0 or n_lit < LIT_MIN_SHARE * total:
        if total < LIT_DECIDE_VOTES and max(lit.values() or [0]) >= 2:
            return "unknown", "?"                  # pending: watch it longer (m1t1 LOOK) before calling it an obstacle
        return "black", ("off" if total >= LIGHT_MIN_VOTES else "?")
    colour, n_top = max(lit.items(), key=lambda kv: kv[1])
    second = max([v for k, v in lit.items() if k != colour] or [0])
    if total >= LIGHT_MIN_VOTES and (n_top < LIT_DOMINANT * n_lit or (black >= n_lit and n_top < LIT_OVER_SECOND * second)):
        return "black", "off"                      # no colour stands out: noise on an unlit buoy, not a light
    if total < LIGHT_MIN_VOTES:
        return colour, "?"
    return colour, ("flash" if black >= FLASH_MIN_BLACK * total else "steady")


class PerceptionMap:
    def __init__(self, cfg: Config, clock=None):
        self.cfg = cfg
        self.m = cfg.map
        self.clock = clock or time.monotonic
        # v0.1.85 (WP46, "our course only"): the RxCourse boundary in the local frame (the mission manager sets it); an entity outside it
        # (+ course_tol_m) is never a task target - kept in the map (outside_course: true) as an obstacle. [] = no filter (bench / sim)
        self.course_poly: List[Tuple[float, float]] = []
        self.course_tol_m = 1.5
        self.ocs_judge: Optional[str] = None
        self.ents: Dict[int, MapEntity] = {}
        self._next_id = 1
        self.n_ingested = 0
        self.last_ingest_t: float = -1e9
        self.last_alive_t: float = -1e9      # v0.1.27: the last perception message incl. the health beat (camera alive, sees nothing)
        self.last_pose: Optional[LocalPose] = None
        self.rays: List[dict] = []           # v0.1.34: bearing-only sightings
        self._ray_id = 1
        self.n_triangulated = 0
        from .box_inspection import BoxTrack
        self.box_tracks: Dict[int, BoxTrack] = {}
        self.dock_tracks: Dict[int, BoxTrack] = {}          # v0.1.58 (WP13): dock faces with face_points, keyed by the map entity
        # v0.1.66 (WP16, D22 of docs/OVERNIGHT_2026-11-07_UAV.md): the UAV's buoys. colour_source "camera" (as before) | "uav" (m1t2: the
        # boat is colour-blind for buoys coloured only from above - the UAV gives the colour, the camera + GPS the position); uav_frozen
        # (m1t2: the UAV layer never forgets during the attempt); the knobs from the profile's `uav:` block (assoc_m 1.0, forget_s 30,
        # min_obs 2, sigma_m 2.5)
        u = dict((cfg.extra or {}).get("uav") or {})
        self.uav_assoc_m = float(u.get("assoc_m", 1.0))
        self.uav_forget_s = float(u.get("forget_s", 30.0))
        self.uav_min_obs = int(u.get("min_obs", 2))
        self.uav_sigma_default = float(u.get("sigma_m", 2.5))
        self.colour_source = "camera"
        self.uav_ingest_off = False                 # v0.1.93.2 (Dmitrii 13 Nov: the UAV on our network, no conflicts in m1t1): set by the
                                                    # mission manager while T1 runs in camera mode (m1t1) - the UAV's map adds / recolours nothing
        self.uav_frozen = False
        self.uav_frames: List[str] = []                     # the last frame_ids ingested (a duplicate path dedups here)
        self.uav_last_t: float = -1e9                       # the last UAV message with items
        self.uav_change_t: float = -1e9                     # v0.1.89: the last UAV message that CHANGED the map (uav_gate's MAP QUIET)
        self.uav_sig = None
        self.uav_scan: dict = {}                            # {state, coverage_pct, n_items, t}
        self.uav_stats = {"published": 0, "linked": 0, "uav_only": 0, "merged": 0, "messages": 0, "dropped": 0}
        self.uav_ocs: dict = {}                             # ocs/state's uav block (the mission manager): active, flight, scan_done
        self.guard_forget_s = float(((cfg.extra or {}).get("zed_guard") or {}).get("forget_s", 900.0))
        zg = (cfg.extra or {}).get("zed_guard") or {}                # v0.1.81.1: the depth obstacles (usv/guard.map_far)
        self.depth_merge_m = float(zg.get("depth_merge_m", 1.0))
        self.depth_explain_m = float(zg.get("depth_explain_m", 1.5))
        self.depth_min_obs = int(zg.get("depth_min_obs", 3))       # sightings (map_far takes 2 per second) before a depth ring counts   # v0.1.80: the guard's obstacles
        from .lamp_tracks import LampTracker                # v0.1.94: the lamp blink decision by world bearing (usv/lamp_tracks)
        lp = dict(((cfg.extra or {}).get("perception") or {}).get("lamps") or {})
        self.lamps_on = bool(lp.pop("enabled", True))
        self.lamps = LampTracker(lp)
        mg = dict(((cfg.extra or {}).get("perception") or {}).get("merge") or {})   # v0.1.94: the buoy merge pass (merge_buoys)
        self.merge_p = {"enabled": False, "a": 0.35, "b": 0.08, "min_m": 0.8, "max_m": 2.5, "number_m": 5.0, "period_s": 1.0, **mg}
        self.depth_fuse_tol_deg = float(((cfg.extra or {}).get("perception") or {}).get("depth_fuse_tol_deg", 0.0) or 0.0)  # v0.1.94 (0 = off)
        self.depth_fuse_ratio = float(((cfg.extra or {}).get("perception") or {}).get("depth_fuse_ratio", 1.6) or 1.6)
        self.depth_fuse_k = float(((cfg.extra or {}).get("perception") or {}).get("depth_fuse_k", 1000.0) or 0.0)
        self.ranged_max_m = float(((cfg.extra or {}).get("perception") or {}).get("ranged_max_m", 0.0) or 0.0)   # v0.1.94 (0 = off)
        self.merged: Dict[int, int] = {}                   # merged-away id -> the id it went into (resolve())
        self._merge_t = -1e9
        self.uav_feeds: dict = {}                           # v0.1.78.19: per source feed ("contract" | "sgit"): messages, points, last_t, status

    # ---- ingest ----------------------------------------------------------------------------------------------------
    def ingest(self, obs: Observation, pose: LocalPose, now: Optional[float] = None, source: str = "usv") -> int:
        """Fold one observation in; returns how many items were used. pose = the boat pose at obs.t_cam."""
        now = self.clock() if now is None else now
        kind = KIND_OF_TOPIC.get(obs.kind, obs.kind)
        f = self.cfg.frames
        used = 0
        items = sorted(obs.items, key=lambda it: it.cls == "indicator") if kind == "dock" else obs.items   # v0.1.48: faces first
        if kind == "buoy" and self.depth_fuse_k > 0:  # v0.1.94 (truth, 13 Nov pm: marker 66, 148 px, read 21.9 m - true ~6.8 m; the depth
            for it in items:                           # inside a near box was the background): a range its box height denies -> depthless
                if it.range_m > 0.0 and not self._size_ok(it, it.range_m):
                    it.range_m = 0.0
                    it.quality = dict(it.quality or {}, reason="size_mismatch", bearing_only=True)
                    self.n_size_mismatch = getattr(self, "n_size_mismatch", 0) + 1
        lamp_of = self._lamp_frame(items, pose, obs.t_cam or now) if kind == "buoy" else {}
        if kind == "buoy":                             # v0.1.94: the frame's boxes without depth, for the guard's clusters (fuse_depth)
            self._no_depth = {"t": float(obs.t_cam or now), "pose": pose, "source": source,
                              "items": [it for it in items if it.range_m <= 0.0 and it.conf >= self.m.min_conf
                                        and (it.quality or {}).get("reason") != "beyond_depth"]}
        for it in items:
            if it.conf < self.m.min_conf:
                continue
            trk = lamp_of.get(id(it))
            if kind == "buoy" and self.ranged_max_m > 0 and it.range_m > self.ranged_max_m:
                it.range_m = 0.0                       # v0.1.94: far depth is metres off - beyond ranged_max_m only the bearing counts (a ray)
            if it.range_m <= 0.0:                      # v0.1.34: a bearing without a range -> a ray (buoys only)
                if kind == "buoy":
                    used += self._ingest_ray(it, pose, now, source, lamp=trk)
                continue
            x, y = body_to_local(it.range_m, it.bearing_deg, pose, f.cam_forward_m, f.cam_right_m, f.cam_yaw_deg)
            if kind == "dock" and it.cls == "indicator":           # v0.1.48: a vote on the face above it, not an entity
                used += self._ingest_indicator(it.colour, x, y, now)
                continue
            wn = (it.quality or {}).get("window") if kind == "window" else None
            g_r = None
            if kind == "buoy" and float(getattr(self.m, "assoc_gate_a", 0.0) or 0.0) > 0:      # v0.1.92: the range-dependent gate (off: 12 Nov's
                g_r = min(float(self.m.assoc_gate_max), max(float(self.m.assoc_gate_min),    # replay split 35 markers in two vs 18 at 2.0 m)
                                                            float(self.m.assoc_gate_a) + float(self.m.assoc_gate_b) * float(it.range_m)))
            if kind == "buoy":
                lc = (trk.result.get("colour") if trk is not None else None)
                self._colour_now, self._range_now = (lc if lc in ("red", "green", "blue") else it.colour), float(it.range_m)
            e = self._fold(kind, it.colour, x, y, it.range_m * 0.05 + 0.2, now, source, gate=g_r, window_no=wn)
            self._colour_now = None
            e.ranged = True
            if (it.quality or {}).get("depth_from") == "guard":
                e.n_fused = getattr(e, "n_fused", 0) + 1      # v0.1.94: sightings ranged by the guard's cloud (fuse_depth)
            e.last_range_m, e.last_bearing_deg = float(it.range_m), float(it.bearing_deg)
            e.min_range_m = min(float(getattr(e, "min_range_m", 1e9) or 1e9), float(it.range_m))     # v0.1.94: the merge radius     # v0.1.79: the raw sighting (m0t8: where it was last seen)
            if kind == "dock":                                      # v0.1.48: the face's colour is its indicator's
                if it.colour in ("red", "green"):
                    self._vote_indicator(e, it.colour, now)
                elif getattr(e, "ind_votes", None) is None:
                    e.colour = "white" if it.colour in ("white", "unknown", "") else it.colour
                if (it.quality or {}).get("face_points"):           # v0.1.58 (WP13): the face's key points -> its track
                    self._ingest_box_sample(it, pose, obs.t_cam or now, tracks=self.dock_tracks, track_id=e.id,
                                            colour=it.colour if it.colour in ("red", "green", "blue") else "white", lit=True)
                used += 1
                continue
            if it.colour and it.colour != "unknown":
                e.colour_votes[it.colour] = round(e.colour_votes.get(it.colour, 0) + self.vote_weight(it.range_m), 3)
                e.colour, e.light = classify_light(e.colour_votes)
                e.colour_now = it.colour
            if kind == "buoy" and getattr(e, "uav", None):          # v0.1.66: the UAV's colour where it rules (D22)
                self._apply_uav(e)
            if kind == "buoy":                                      # v0.1.92: the observer's LED reader beats the votes
                self._led_vote(e, (it.quality or {}).get("led"), now)
                if trk is not None:
                    self._lamp_link(e, trk, now)
                lamp_now = self._lamp_now(it)
                if lamp_now is not None:               # v0.1.94 (fork B: T1 rejected the true entry - its near votes read the body, white on
                    e.colour_now = lamp_now            # every real buoy): the sighting's colour is its lamp's - lit colour, else black (dark)
                self._number_vote(e, (it.quality or {}).get("number"))
            if kind == "window":
                self._window_light(e, it.colour, wn)
            if kind == "box":
                self._ingest_box_sample(it, pose, obs.t_cam or now)
            used += 1
        if kind == "buoy" and self.merge_p.get("enabled") and now - self._merge_t >= float(self.merge_p["period_s"]):
            self._merge_t = now
            self.merge_buoys(now)
        if used:
            self.n_ingested += used
            self.last_ingest_t = now
        self.last_pose = pose
        return used

    def vote_weight(self, range_m: float) -> float:
        """v0.1.34: a colour sighting's weight by range - 1 up to vote_full_m, falling to vote_far_w at vote_far_m (and beyond)."""
        m = self.m
        if range_m <= m.vote_full_m or m.vote_far_m <= m.vote_full_m:
            return 1.0
        k = min(1.0, (range_m - m.vote_full_m) / (m.vote_far_m - m.vote_full_m))
        return 1.0 - k * (1.0 - m.vote_far_w)

    # ---- v0.1.48 (WP10j-b): the dock's indicators and windows -----------------------------------------------------------
    def _ingest_indicator(self, colour: str, x: float, y: float, now: float) -> int:
        if colour not in ("red", "green"):
            return 0
        best, best_d = None, self.m.assoc_gate_m
        for e in self.ents.values():
            if e.kind == "dock" and e.state == "tracked":
                d = math.hypot(e.x_n - x, e.y_e - y)
                if d < best_d:
                    best, best_d = e, d
        if best is None:
            return 0
        self._vote_indicator(best, colour, now)
        return 1

    @staticmethod
    def _vote_indicator(e: MapEntity, colour: str, now: float) -> None:
        if getattr(e, "ind_votes", None) is None:
            e.ind_votes, e.ind_hist = {}, []
        e.ind_votes[colour] = e.ind_votes.get(colour, 0) + 1
        e.ind_hist.append((now, colour))
        del e.ind_hist[:-200]
        recent = [c for _, c in e.ind_hist[-IND_RECENT:]]
        r, g = recent.count("red"), recent.count("green")
        e.colour = "green" if g > r else ("red" if r > g else colour)
        e.colour_now = colour

    @staticmethod
    def _window_light(e: MapEntity, colour: str, wn) -> None:
        light = colour if colour in ("red", "green") else "off"
        hist = getattr(e, "light_hist", None)
        if hist is None:
            hist = e.light_hist = []
        hist.append(light)
        del hist[:-WINDOW_RECENT]
        n = {k: hist.count(k) for k in ("off", "red", "green")}
        top = max(n.values())
        e.light = light if n[light] == top else max(n, key=n.get)
        if wn in (1, 2):
            v = getattr(e, "wn_votes", None)
            if v is None:
                v = e.wn_votes = {}
            v[int(wn)] = v.get(int(wn), 0) + 1
            e.window_no = max(v, key=v.get)

    def _fold(self, kind: str, colour: str, x: float, y: float, sigma0: float, now: float, source: str,
              gate: Optional[float] = None, window_no=None) -> MapEntity:
        """One positioned sighting into the nearest entity of its kind within the gate (else a new one): the position EMA.
        v0.1.48: a numbered window only into a window of that number (or one not numbered yet)."""
        gate = gate if gate is not None else (self.m.assoc_gate_window_m if kind == "window" else self.m.assoc_gate_m)
        e = self._nearest(kind, x, y, gate, window_no)
        if kind == "buoy" and source != "uav" and e is not None and (e.source == "uav" or any(q.source == "uav" for q in self.ents.values())):
            # v0.1.66 (D22): a camera sighting goes to the nearest CAMERA track first, which absorbs a UAV-only twin within uav.assoc_m
            # (found live: folding it into a nearer UAV-only twin split one buoy into two camera tracks - m1t1 with the UAV reported 11
            # buoys, one OFF); a UAV-only buoy becomes the camera's track only when no camera track is near - then the boat owns its
            # position. Without UAV entities this is the old rule exactly (the m1t1 campaign is unchanged).
            cam = self._nearest_not_uav(kind, x, y, gate)
            uo = None
            for q in self.ents.values():
                if q.source == "uav" and q.kind == kind and math.hypot(q.x_n - x, q.y_e - y) <= self.uav_assoc_m:
                    if uo is None or math.hypot(q.x_n - x, q.y_e - y) < math.hypot(uo.x_n - x, uo.y_e - y):
                        uo = q
            if cam is not None:
                e = cam
                cid, oid = (getattr(cam, "uav", None) or {}).get("id"), (getattr(uo, "uav", None) or {}).get("id") if uo is not None else None
                if uo is not None and cid is not None and oid is not None and cid != oid:
                    uo = None                                        # two UAV ids are two buoys: never absorbed into each other
                if uo is not None:                                   # the twin's UAV features move to the camera's track
                    cv = dict(getattr(cam, "uav_cvotes", None) or {})
                    for k, v in (getattr(uo, "uav_cvotes", None) or {}).items():
                        cv[k] = cv.get(k, 0) + v
                    lv = dict(getattr(cam, "uav_lvotes", None) or {})
                    for k, v in (getattr(uo, "uav_lvotes", None) or {}).items():
                        lv[k] = lv.get(k, 0) + v
                    cam.uav_cvotes, cam.uav_lvotes = cv, lv
                    feat = dict(getattr(uo, "uav", None) or {})
                    if feat:
                        feat["colour"] = max(cv.items(), key=lambda kv: kv[1])[0] if cv else feat.get("colour")
                        feat["lit"] = max(lv.items(), key=lambda kv: kv[1])[0] if lv else feat.get("lit", "?")
                        feat["votes"] = dict(cv)
                        cam.uav = feat
                    del self.ents[uo.id]
                    self.uav_stats["merged"] += 1
            elif uo is not None:
                e = uo
                e.x_n, e.y_e, e.n_obs, e.sigma_m = x, y, 0, sigma0
                e.source = "usv"
                self.uav_stats["merged"] += 1
            else:
                e = None
        if e is None:
            e = MapEntity(id=self._next_id, kind=kind, colour=colour, x_n=x, y_e=y, sigma_m=sigma0, n_obs=0,
                          t_first=now, t_last=now, source=source)
            self._next_id += 1
            self.ents[e.id] = e
        n = e.n_obs + 1
        w = 1.0 / min(n, self.m.ema_n_max)
        res = math.hypot(x - e.x_n, y - e.y_e)
        e.x_n += w * (x - e.x_n)
        e.y_e += w * (y - e.y_e)
        e.sigma_m += w * (res - e.sigma_m)
        e.n_obs = n
        e.t_last = now
        e.state = "tracked"
        if source == "usv":
            e.source = "usv"             # a USV sighting confirms a UAV-sourced entity (D2)
        return e

    # ---- v0.1.34: rays (bearing-only sightings) and their triangulation --------------------------------------------------------
    def _ingest_ray(self, it, pose: LocalPose, now: float, source: str, lamp=None) -> int:
        f, m = self.cfg.frames, self.m
        ox, oy = body_to_local(0.0, 0.0, pose, f.cam_forward_m, f.cam_right_m, f.cam_yaw_deg)
        az = wrap360(pose.hdg_deg + f.cam_yaw_deg + it.bearing_deg)
        colour = it.colour
        if lamp is not None:                           # v0.1.94: the lamp's colour (decided on its world azimuth) is the ray's
            if lamp.result.get("colour") in ("red", "green", "blue"):
                colour = lamp.result["colour"]
            h = (it.quality or {}).get("bbox_h")
            e = self._entity_on(ox, oy, az, lamp.ent, r_hint=(self.depth_fuse_k / float(h)) if (h and self.depth_fuse_k > 0) else None)
            if e is not None:
                self._lamp_link(e, lamp, now)
        if self._explained(ox, oy, az, colour):
            return 0                                   # a ranged buoy already sits along it: depth has arrived
        for r in self.rays:
            if (r["colour"] == colour and math.hypot(r["x0"] - ox, r["y0"] - oy) <= 1.0          # within 1 m of where it began
                    and abs(wrap180(r["az_deg"] - az)) <= 2.0 * m.ray_ang_tol_deg):
                r.update(x_n=ox, y_e=oy, az_deg=az, t=now, n=r["n"] + 1)    # the newest sighting from about here (origin + azimuth together)
                return 1
        ray = {"id": self._ray_id, "colour": colour, "x_n": ox, "y_e": oy, "x0": ox, "y0": oy, "az_deg": az, "t": now, "n": 1, "source": source}
        self._ray_id += 1
        self.rays.append(ray)
        if len(self.rays) > 80:
            self.rays.pop(0)
        if m.triangulate:
            self._triangulate(ray, now)
        return 1

    def _explained(self, ox: float, oy: float, az: float, colour: str) -> bool:
        for e in self.ents.values():
            if e.kind != "buoy" or e.state != "tracked" or e.n_obs < self.m.min_obs or not getattr(e, "ranged", False):
                continue                               # only a buoy with a real range explains a ray (not one triangulated from rays)
            if colour not in ("unknown", "", None) and e.colour not in (colour, "unknown"):
                continue
            d = math.hypot(e.x_n - ox, e.y_e - oy)
            if d > 2.0 and abs(wrap180(math.degrees(math.atan2(e.y_e - oy, e.x_n - ox)) - az)) <= self.m.ray_ang_tol_deg:
                return True
        return False

    @staticmethod
    def intersect(a: dict, b: dict) -> Optional[Tuple[float, float, float, float]]:
        """Two rays (x_n, y_e, az_deg) -> (x, y, distance along a, along b), None when parallel or behind either."""
        ta, tb = math.radians(a["az_deg"]), math.radians(b["az_deg"])
        da, db = (math.cos(ta), math.sin(ta)), (math.cos(tb), math.sin(tb))
        den = da[0] * db[1] - da[1] * db[0]
        if abs(den) < 1e-6:
            return None
        rx, ry = b["x_n"] - a["x_n"], b["y_e"] - a["y_e"]
        sa = (rx * db[1] - ry * db[0]) / den
        sb = (rx * da[1] - ry * da[0]) / den
        if sa <= 0 or sb <= 0:
            return None
        return a["x_n"] + sa * da[0], a["y_e"] + sa * da[1], sa, sb

    def _triangulate(self, new: dict, now: float) -> int:
        """The new ray against every other ray of its colour from somewhere else: each good crossing is a sighting."""
        m, n = self.m, 0
        for r in self.rays:
            if r is new or r["colour"] != new["colour"] or math.hypot(r["x_n"] - new["x_n"], r["y_e"] - new["y_e"]) < 1.0:
                continue
            cross = abs(wrap180(r["az_deg"] - new["az_deg"]))
            if cross < m.tri_min_angle_deg or cross > 180.0 - m.tri_min_angle_deg:
                continue
            p = self.intersect(new, r)
            if p is None or max(p[2], p[3]) > m.tri_max_range_m or min(p[2], p[3]) < 2.0:
                continue
            sigma = max(p[2], p[3]) * math.radians(0.5) / max(math.sin(math.radians(cross)), 0.17) + 0.3
            e = self._fold("buoy", new["colour"], p[0], p[1], sigma, now, new["source"], gate=max(m.assoc_gate_m, 3.0 * sigma))
            if new["colour"] and new["colour"] != "unknown":
                e.colour_votes[new["colour"]] = e.colour_votes.get(new["colour"], 0) + 1
                e.colour, e.light = classify_light(e.colour_votes)
                self._led_apply(e)
            e.triangulated = getattr(e, "triangulated", 0) + 1
            n += 1
        self.n_triangulated += n
        return n

    def _ingest_box_sample(self, it, pose: LocalPose, t_sample: float, tracks: Optional[dict] = None, track_id: Optional[int] = None,
                           colour: Optional[str] = None, lit: Optional[bool] = None) -> None:
        from .box_inspection import BoxSample, BoxTrack, FaceGeometry, PointNE
        points = (it.quality or {}).get("face_points") or {}
        if not all(k in points for k in ("left", "midpoint", "right")):
            return

        def project(name: str) -> PointNE:
            p = points[name]
            n, e = body_to_local(float(p["range_m"]), float(p["bearing_deg"]), pose,
                                 self.cfg.frames.cam_forward_m, self.cfg.frames.cam_right_m,
                                 self.cfg.frames.cam_yaw_deg)
            return PointNE(n, e)

        tid = int(it.id) if track_id is None else int(track_id)
        tr = (self.box_tracks if tracks is None else tracks).setdefault(tid, BoxTrack(tid))
        tr.add(BoxSample(t=float(t_sample), box_id=tid,
                         face=FaceGeometry(project("left"), project("midpoint"), project("right")),
                         colour=str(it.colour if colour is None else colour).lower(), lit=bool(it.lit if lit is None else lit),
                         confidence=float(it.conf)))

    def stable_boxes(self, require_blink: bool = False):
        return [b for b in (t.stable(require_blink=require_blink) for t in self.box_tracks.values()) if b is not None]

    def _nearest_not_uav(self, kind: str, x: float, y: float, gate: float) -> Optional[MapEntity]:
        best, best_d = None, gate
        for e in self.ents.values():
            if e.kind == kind and e.source != "uav":
                d = math.hypot(e.x_n - x, e.y_e - y)
                if d < best_d:
                    best, best_d = e, d
        return best

    def _nearest(self, kind: str, x: float, y: float, gate: float, window_no=None) -> Optional[MapEntity]:
        best, best_d = None, gate
        cn = getattr(self, "_colour_now", None)
        sep = (kind == "buoy" and cn in ("red", "green", "blue")
               and getattr(self, "_range_now", 99.0) < float(getattr(self.m, "assoc_colour_m", 0.0) or 0.0))
        for e in self.ents.values():
            if e.kind != kind:
                continue
            if sep and e.colour in ("red", "green", "blue") and e.colour != cn:
                continue                                   # v0.1.92: near, lit one colour - never into a buoy decided another
            if window_no in (1, 2) and getattr(e, "window_no", None) not in (None, window_no):
                continue
            d = math.hypot(e.x_n - x, e.y_e - y)
            if d < best_d:
                best, best_d = e, d
        return best

    # ---- housekeeping ----------------------------------------------------------------------------------------------
    def age_out(self, now: Optional[float] = None) -> None:
        now = self.clock() if now is None else now
        for eid in list(self.ents):
            e = self.ents[eid]
            if e.source in ("guard", "depth"):                  # v0.1.80 (WP36): the guard's obstacles stay for guard_forget_s (the attempt)
                if now - e.t_last > self.guard_forget_s or (e.source == "depth" and (self.explained_obstacle(e.x_n, e.y_e) or
                                                                                     self.explained_obstacle(*(getattr(e, "face_xy", None) or (e.x_n, e.y_e))))):
                    del self.ents[eid]                           # v0.1.81.1: a buoy found there later explains a depth one
                continue
            if e.source == "uav":                                # v0.1.66: UAV-only - frozen in m1t2, else forgotten after uav.forget_s
                if not self.uav_frozen and now - e.t_last > self.uav_forget_s:
                    del self.ents[eid]
                continue
            if getattr(e, "uav", None) and not self.uav_frozen and now - e.uav["t"] > self.uav_forget_s:
                e.uav = None                                     # the linked features expire too (outside m1t2)
                if e.kind == "buoy" and e.colour_votes:
                    e.colour, e.light = classify_light(e.colour_votes)
                    self._led_apply(e)
            age = now - e.t_last
            if age > self.m.max_age_s + self.m.drop_age_s:
                del self.ents[eid]
            elif self.m.forget_s > 0 and e.n_obs < self.m.min_obs and age > self.m.forget_s:
                del self.ents[eid]                     # v0.1.31: a ghost (a few noisy sightings) is forgotten, not aged for minutes
            elif age > self.m.max_age_s:
                e.state = "stale"
        self.rays = [r for r in self.rays if now - r["t"] <= self.m.ray_ttl_s and not self._explained(r["x_n"], r["y_e"], r["az_deg"], r["colour"])]

    def add_obstacle(self, x: float, y: float, ring_m: float, now: Optional[float] = None, source: str = "guard",
                     face: Optional[Tuple[float, float]] = None) -> Optional[MapEntity]:
        """v0.1.80 (WP36): what the ZED guard stopped for - `kind obstacle`, source guard, its clearance ring as ring_m (the planners keep
        their keep rule round a disc of that radius); one within 0.5 m of an earlier one is the same (its ring grows to the larger).
        v0.1.81.1: source "depth" = the guard's `far` clusters (3-8 m, usv/guard.map_far): merged within depth_merge_m (1.0) into one
        obstacle (its centre the running mean), n_obs counts the sightings - on the map and in the planners only from depth_min_obs (3,
        = 1.5 s at map_far's 2 Hz) on (a wave's glitter is one frame); none where a mapped buoy / dock / box / window explains it (within depth_explain_m, 1.5)."""
        now = self.clock() if now is None else now
        if source == "depth":
            fx, fy = face if face is not None else (x, y)       # the seen face (the ring's centre is 1 m behind it)
            if self.explained_obstacle(fx, fy) or self.explained_obstacle(x, y):
                return None
            e = next((q for q in self.ents.values() if q.kind == "obstacle" and math.hypot(q.x_n - x, q.y_e - y) < self.depth_merge_m), None)
            if e is None:
                e = MapEntity(id=self._next_id, kind="obstacle", colour="grey", x_n=x, y_e=y, sigma_m=ring_m, n_obs=0, t_first=now, t_last=now,
                              source="depth")
                self._next_id += 1
                self.ents[e.id] = e
            if e.source == "depth":                              # a guard STOP's ring stays where the boat stopped
                n = e.n_obs + 1
                e.x_n, e.y_e = e.x_n + (x - e.x_n) / n, e.y_e + (y - e.y_e) / n
                f0 = getattr(e, "face_xy", None) or (fx, fy)
                e.face_xy = (f0[0] + (fx - f0[0]) / n, f0[1] + (fy - f0[1]) / n)
            e.n_obs += 1
            e.ring_m = max(float(getattr(e, "ring_m", 0.0) or 0.0), float(ring_m))
            e.t_last = now
            return e
        e = next((q for q in self.ents.values() if q.kind == "obstacle" and math.hypot(q.x_n - x, q.y_e - y) < 0.5), None)
        if e is None:
            e = MapEntity(id=self._next_id, kind="obstacle", colour="grey", x_n=x, y_e=y, sigma_m=ring_m, n_obs=max(self.m.min_obs, 1),
                          t_first=now, t_last=now, source="guard")
            self._next_id += 1
            self.ents[e.id] = e
        elif e.source == "depth":                                # the guard stopped for it: it is real (kept for the attempt)
            e.source, e.n_obs = "guard", max(e.n_obs, self.m.min_obs, 1)
        e.ring_m = max(float(getattr(e, "ring_m", 0.0) or 0.0), float(ring_m))
        e.t_last = now
        return e

    EXPLAINS = ("buoy", "dock", "box", "window")

    def explained_obstacle(self, x: float, y: float) -> bool:
        """v0.1.81.1: a depth cluster at (x, y) is a thing the map already holds (a buoy - the boat's or the UAV's -, a dock face, a box)."""
        r = self.depth_explain_m
        return any(q.kind in self.EXPLAINS and math.hypot(q.x_n - x, q.y_e - y) < r for q in self.ents.values())

    def obstacle_counts(self, e: MapEntity) -> bool:
        """v0.1.81.1: a depth obstacle counts (the planners, the digest) once seen zed_guard.depth_min_obs (3) times; a guard one always."""
        return e.source != "depth" or e.n_obs >= max(1, self.depth_min_obs)

    def clear(self) -> None:
        self.uav_frames, self.uav_scan, self.uav_last_t = [], {}, -1e9
        self.uav_change_t, self.uav_sig = -1e9, None
        self.ents.clear()
        self.rays.clear()
        self.box_tracks.clear()
        self.dock_tracks.clear()

    # ---- queries ---------------------------------------------------------------------------------------------------
    def set_course(self, poly, tol_m: Optional[float] = None) -> None:
        self.course_poly = [tuple(map(float, q)) for q in (poly or [])]
        if tol_m is not None:
            self.course_tol_m = float(tol_m)

    def in_course(self, x: float, y: float) -> bool:
        """WP46: inside the course boundary + course_tol_m (True without a boundary)."""
        if len(self.course_poly) < 3:
            return True
        from .guidance import in_course
        return in_course((x, y), self.course_poly, self.course_tol_m)

    def entities(self, kind: Optional[str] = None, include_stale: bool = False, include_young: bool = False,
                 include_uav: bool = False, include_outside: bool = False) -> List[MapEntity]:
        """The map's entities; v0.1.31: those seen fewer than perception.min_obs times stay out unless include_young.
        v0.1.66: UAV-only buoys (positioned by the drone alone) stay out unless include_uav - they are leads, never planned through."""
        mo = self.m.min_obs
        out = [e for e in self.ents.values() if (kind is None or e.kind == kind) and (include_stale or e.state == "tracked")
               and (include_young or e.n_obs >= mo) and (include_uav or e.source != "uav")]
        if not include_outside and len(self.course_poly) >= 3:         # v0.1.85 (WP46): a neighbour course's buoys are never targets
            out = [e for e in out if self.in_course(e.x_n, e.y_e)]
        return sorted(out, key=lambda e: e.id)

    def nearest(self, x: float, y: float, kind: Optional[str] = None) -> Optional[MapEntity]:
        ents = self.entities(kind)
        return min(ents, key=lambda e: math.hypot(e.x_n - x, e.y_e - y)) if ents else None

    def gates(self, from_pose: Optional[LocalPose] = None, min_obs: int = 1) -> List[Gate]:
        """Red/green pairs within the width bounds; each buoy in at most one gate (closest pairing first). min_obs drops
        buoys seen fewer times (ghosts from far, noisy sightings; usv_ui pathgen uses 5)."""
        reds = [e for e in self.entities("buoy") if e.colour == "red" and e.n_obs >= min_obs]
        greens = [e for e in self.entities("buoy") if e.colour == "green" and e.n_obs >= min_obs]
        pairs: List[Tuple[float, MapEntity, MapEntity]] = []
        for r in reds:
            for g in greens:
                w = math.hypot(r.x_n - g.x_n, r.y_e - g.y_e)
                if self.m.gate_width_min_m <= w <= self.m.gate_width_max_m:
                    pairs.append((w, r, g))
        pairs.sort(key=lambda p: p[0])
        used, gates = set(), []
        for w, r, g in pairs:
            if r.id in used or g.id in used:
                continue
            used.update((r.id, g.id))
            mid_n, mid_e = (r.x_n + g.x_n) / 2, (r.y_e + g.y_e) / 2
            brg_mid_red = math.degrees(math.atan2(r.y_e - mid_e, r.x_n - mid_n))
            through = wrap360(brg_mid_red - 90.0)          # red to starboard of the through heading
            if from_pose is not None:                     # of the two directions, the one that leads away from the boat
                away = math.degrees(math.atan2(mid_e - from_pose.y_e, mid_n - from_pose.x_n))
                if abs(wrap180(through - away)) > 90.0:
                    through = wrap360(through + 180.0)
            gates.append(Gate(red=r, green=g, mid_n=mid_n, mid_e=mid_e, through_hdg_deg=through, width_m=w))
        if from_pose is not None:
            gates.sort(key=lambda gt: math.hypot(gt.mid_n - from_pose.x_n, gt.mid_e - from_pose.y_e))
        return gates

    CERTAIN = ("red", "green", "blue")

    def lucky_gates(self, min_obs: int = 1, used_ids=(), mode: str = "one_known") -> List[Gate]:
        """v0.1.90 (Dmitrii 2026-11-12: "lucky" - our model is not perfect, take the risk): the closest pairs inside the course boundary
        (+ in_course's margin) within the gate width bounds where at least one buoy's colour is uncertain; v0.1.92 mode one_known (the default, Cowork consult): the other a
        known red / green - never two unknowns (markers 1.4-2.8 m apart look like a gate); mode any = v0.1.90 (Dmitrii's two unknowns)
        and neither is blue (the entry / exit markers) nor black (an obstacle) nor red + red / green + green; each buoy in one gate, the buoys of the real gates
        (used_ids) left out, the narrowest first. The roles: a known red stays red, a known green stays green; two unknowns - the lower
        id plays red (the planner may cross a lucky gate either way). Gate.lucky = True."""
        cand = [e for e in self.entities("buoy") if e.n_obs >= min_obs and e.id not in set(used_ids)
                and e.colour not in ("blue", "black") and self.in_course(e.x_n, e.y_e)]      # blue = entry / exit, black = an obstacle
        pairs: List[Tuple[float, MapEntity, MapEntity]] = []
        for i, a in enumerate(cand):
            for b in cand[i + 1:]:
                ca, cb = a.colour in ("red", "green"), b.colour in ("red", "green")
                if ca and cb:
                    continue                                    # both certain: a real gate (gates()) or not a gate at all
                if not ca and not cb and mode != "any":
                    continue                                    # v0.1.92 (Cowork consult): never two unknowns - markers 1.4-2.8 m apart look
                                                                # exactly like a 2-3 m gate; one known red / green is the least we take
                w = math.hypot(a.x_n - b.x_n, a.y_e - b.y_e)
                if self.m.gate_width_min_m <= w <= self.m.gate_width_max_m:
                    r, g = (a, b) if (a.colour == "red" or b.colour == "green" or (not ca and not cb and a.id < b.id)) else (b, a)
                    pairs.append((w, r, g))
        pairs.sort(key=lambda p: p[0])
        used, out = set(), []
        for w, r, g in pairs:
            if r.id in used or g.id in used:
                continue
            used.update((r.id, g.id))
            mid_n, mid_e = (r.x_n + g.x_n) / 2, (r.y_e + g.y_e) / 2
            through = wrap360(math.degrees(math.atan2(r.y_e - mid_e, r.x_n - mid_n)) - 90.0)
            out.append(Gate(red=r, green=g, mid_n=mid_n, mid_e=mid_e, through_hdg_deg=through, width_m=w, lucky=True))
        return out

    def markers(self) -> Dict[str, Optional[MapEntity]]:
        """Task 1 entry (FLASHING blue) and exit (SOLID blue) among the tracked buoys; the best-evidenced one of each."""
        out: Dict[str, Optional[MapEntity]] = {"entry": None, "exit": None}
        for e in self.entities("buoy"):
            if e.colour != "blue" or e.light not in ("flash", "steady"):
                continue
            k = "entry" if e.light == "flash" else "exit"
            if out[k] is None or e.n_obs > out[k].n_obs:
                out[k] = e
        return out

    def dock(self, from_pose: Optional[LocalPose] = None, now: Optional[float] = None) -> Optional[dict]:
        """Total-least-squares line through the dock faces seen within dock_max_age_s; normal toward the boat."""
        now = self.clock() if now is None else now
        faces = [e for e in self.entities("dock") if now - e.t_last <= self.m.dock_max_age_s]
        if len(faces) < 2:
            return None
        cx = sum(e.x_n for e in faces) / len(faces)
        cy = sum(e.y_e for e in faces) / len(faces)
        sxx = sum((e.x_n - cx) ** 2 for e in faces)
        syy = sum((e.y_e - cy) ** 2 for e in faces)
        sxy = sum((e.x_n - cx) * (e.y_e - cy) for e in faces)
        theta = 0.5 * math.atan2(2 * sxy, sxx - syy)         # direction of the principal axis (the dock line)
        dn, de = math.cos(theta), math.sin(theta)
        nn, ne = -de, dn                                       # a normal
        if from_pose is not None and (nn * (from_pose.x_n - cx) + ne * (from_pose.y_e - cy)) < 0:
            nn, ne = -nn, -ne                                  # orient toward the boat
        along = [((e.x_n - cx) * dn + (e.y_e - cy) * de) for e in faces]
        length = max(along) - min(along)
        resid = [abs((e.x_n - cx) * nn + (e.y_e - cy) * ne) for e in faces]
        return {"x_n": round(cx, 2), "y_e": round(cy, 2), "psi_n": round(wrap360(math.degrees(math.atan2(ne, nn))), 1),
                "line_hdg_deg": round(wrap360(math.degrees(theta)), 1), "length_m": round(length, 2), "faces": [e.id for e in faces],
                "rms_m": round(math.sqrt(sum(r * r for r in resid) / len(resid)), 2)}

    # ---- for the map digest ----------------------------------------------------------------------------------------
    def digest(self, from_pose: Optional[LocalPose] = None, now: Optional[float] = None) -> dict:
        now = self.clock() if now is None else now
        ents = []
        young = 0
        for e in sorted(self.ents.values(), key=lambda e: e.id):
            if e.n_obs < (self.uav_min_obs if e.source == "uav" else self.m.min_obs) or not self.obstacle_counts(e):   # v0.1.31: never in the digest / the map
                young += 1
                continue
            b = e.body()
            b["age_s"] = round(now - e.t_last, 1)
            b["outside_course"] = not self.in_course(e.x_n, e.y_e)          # v0.1.85 (WP46): kept as an obstacle, never a target
            if e.kind == "obstacle":                             # v0.1.81.1: the ring the planners keep clear of (the maps draw it)
                b["ring_m"] = round(float(getattr(e, "ring_m", 0.0) or e.sigma_m), 2)
            u = getattr(e, "uav", None)
            if u:                                                # v0.1.66 (D22): the UAV's features, the link, the disagreement
                b.update(uav_colour=u.get("colour"), uav_lit=u.get("lit"), uav_seen_t=round(now - u["t"], 1), linked=e.source != "uav",
                         uav_id=u.get("id"), disagree=bool(getattr(e, "uav_disagree", False)), cam_colour=getattr(e, "cam_colour", None))
            if e.kind == "dock" and getattr(e, "ind_votes", None):     # v0.1.48: the indicator votes per colour (cv status, the hover)
                b["votes"] = dict(e.ind_votes)
            elif e.kind == "window" and getattr(e, "window_no", None):
                b["window"] = e.window_no
            ents.append(b)
        mk = self.markers()
        return {"entities": ents, "gates": [g.body() for g in self.gates(from_pose)], "dock": self.dock(from_pose, now),
                "markers": {k: (e.id if e else None) for k, e in mk.items()},
                "rays": [{"id": r["id"], "colour": r["colour"], "x_n": round(r["x_n"], 2), "y_e": round(r["y_e"], 2), "az_deg": round(r["az_deg"], 1),
                          "n": r["n"], "age_s": round(now - r["t"], 1)} for r in self.rays],
                "n_triangulated": self.n_triangulated,
                "uav": self.uav_digest(now),                                                      # v0.1.66
                "provisional_gates": [{"red": g[0].id, "green": g[1].id, "mid": [round(g[2], 2), round(g[3], 2)]} for g in self.provisional_gates()],
                "course_boundary_local": [[round(x, 2), round(y, 2)] for x, y in self.course_poly], "course_tol_m": self.course_tol_m,
                "n_outside_course": sum(1 for b in ents if b.get("outside_course")),
                "n_ingested": self.n_ingested, "n_young": young, "min_obs": self.m.min_obs, "ingest_age_s": None if self.last_ingest_t < -1e8 else round(now - self.last_ingest_t, 1)}


    # ---- v0.1.66 (WP16, D22): the UAV's buoys --------------------------------------------------------------------------------
    UAV_LIT = {"flash": "flash", "steady": "steady", "off": "off", None: "?"}

    def ingest_geo(self, msg: dict, origin, now: Optional[float] = None) -> int:
        """One `uav/perception/buoys` message (docs/UAV_BUOY_CONTRACT.md): every item to local through the ControlOrigin, linked to the
        nearest boat-tracked buoy within uav.assoc_m (the boat keeps its position, takes the UAV's colour / light / id), else a UAV-only
        buoy at the UAV's position (sigma_m from the message) that merges into the boat's track when the camera sees it within assoc_m.
        A message with an already-seen frame_id (the direct and the relay path both up) is ignored. Returns the items used."""
        now = self.clock() if now is None else now
        if origin is None or not isinstance(msg, dict):
            return 0
        fid = str(msg.get("frame_id") or "")
        if fid and fid in self.uav_frames:
            return 0
        if fid:
            self.uav_frames = (self.uav_frames + [fid])[-50:]
        fd = self.uav_feeds.setdefault(str(msg.get("feed") or "contract"), {"messages": 0, "points": 0, "with_position": 0, "first_t": now,
                                                                          "last_t": None, "status": None, "status_t": None})
        fd["messages"] += 1
        fd["last_t"] = now
        fd["points"] = len(msg.get("items") or [])
        fd["with_position"] = sum(1 for it in msg.get("items") or [] if isinstance(it, dict) and it.get("lat") is not None and it.get("lon") is not None)
        sc = msg.get("scan") or {}
        if sc:
            self.uav_scan = {"state": sc.get("state"), "coverage_pct": sc.get("coverage_pct"), "n_items": sc.get("n_items"), "t": now}
        self.uav_stats["messages"] += 1
        sig = []                                                     # v0.1.89: the map's content (~0.5 m, colour, light) - a repeat is no change
        for it in msg.get("items") or []:
            try:
                sig.append((round(float(it["lat"]) * 2e5), round(float(it["lon"]) * 2e5), it.get("colour"), it.get("lit")))
            except (KeyError, TypeError, ValueError):
                pass
        sig = sorted(sig, key=repr)
        if sig and sig != self.uav_sig:
            self.uav_sig, self.uav_change_t = sig, now
        if self.uav_ingest_off:                                            # v0.1.93.2: m1t1 (T1 in camera mode) - the UAV's map is
            self.uav_stats["ignored"] = self.uav_stats.get("ignored", 0) + len(msg.get("items") or [])   # seen (feed, status, the change
            return 0                                                       # time for the touchdown) but never adds or recolours a buoy
        used = 0
        claimed = set()
        for it in msg.get("items") or []:
            try:
                lat, lon = float(it["lat"]), float(it["lon"])
            except (KeyError, TypeError, ValueError):
                self.uav_stats["dropped"] += 1                      # a bad item is dropped, the rest kept (D20)
                continue
            if str(it.get("class") or "buoy") != "buoy":
                continue
            colour = it.get("colour") if it.get("colour") in ("red", "green", "blue", "black", "yellow") else None
            x, y = origin.to_local(lat, lon)
            feat = {"colour": colour, "lit": self.UAV_LIT.get(it.get("lit"), "?"), "conf": it.get("conf"), "t": now, "id": it.get("id"),
                    "feed": str(msg.get("feed") or "contract"), "sgit_id": it.get("sgit_id"),
                    "sigma": float(it.get("sigma_m") or self.uav_sigma_default)}
            self.uav_stats["published"] += 1
            e = self._link_target(x, y, claimed)
            if e is not None:
                claimed.add(e.id)
                e.uav = self._uav_vote(e, feat)
                self._apply_uav(e)
                self.uav_stats["linked"] += 1
            else:
                e = self._uav_only(x, y, feat, it.get("id"), now, claimed)
                claimed.add(e.id)
                self.uav_stats["uav_only"] += 1
            used += 1
        if used:
            self.uav_last_t = now
        return used

    def uav_feed_status(self, feed: str, status: Optional[str], now: Optional[float] = None) -> None:
        """v0.1.78.19: a feed's own up / down word (the UAV team's retained sgit/uav/cv/status), shown beside its map counters."""
        now = self.clock() if now is None else now
        fd = self.uav_feeds.setdefault(feed, {"messages": 0, "points": 0, "with_position": 0, "first_t": None, "last_t": None,
                                              "status": None, "status_t": None})
        fd["status"], fd["status_t"] = status, now

    def _link_target(self, x: float, y: float, claimed: set) -> Optional[MapEntity]:
        """The nearest boat-tracked buoy within uav.assoc_m not yet taken by another item of this message (one entity per radius)."""
        best, bd = None, self.uav_assoc_m
        for e in self.ents.values():
            if e.kind != "buoy" or e.source == "uav" or e.id in claimed:
                continue
            d = math.hypot(e.x_n - x, e.y_e - y)
            if d <= bd:
                best, bd = e, d
        return best

    def _uav_only(self, x: float, y: float, feat: dict, uid, now: float, claimed: set = frozenset()) -> MapEntity:
        """The UAV-only buoy of this UAV id (its tracker's), else - for an item without an id - the nearest id-less UAV-only within
        assoc_m not taken by another item of the message, else a new one (two ids are two buoys, however close)."""
        e = None
        if uid is not None:
            e = next((q for q in self.ents.values() if q.source == "uav" and (getattr(q, "uav", None) or {}).get("id") == uid), None)
        elif e is None:
            best, bd = None, self.uav_assoc_m
            for q in self.ents.values():
                if q.source == "uav" and q.kind == "buoy" and q.id not in claimed and (getattr(q, "uav", None) or {}).get("id") is None:
                    d = math.hypot(q.x_n - x, q.y_e - y)
                    if d <= bd:
                        best, bd = q, d
            e = best
        if e is None:
            e = MapEntity(id=self._next_id, kind="buoy", colour=feat["colour"] or "unknown", x_n=x, y_e=y, sigma_m=feat["sigma"], n_obs=0,
                          t_first=now, t_last=now, source="uav")
            self._next_id += 1
            self.ents[e.id] = e
        n = e.n_obs + 1
        w = 1.0 / min(n, self.m.ema_n_max)
        e.x_n += w * (x - e.x_n)
        e.y_e += w * (y - e.y_e)
        e.sigma_m = feat["sigma"]
        e.n_obs, e.t_last, e.state = n, now, "tracked"
        e.uav = self._uav_vote(e, feat)
        e.colour = e.uav["colour"] or "unknown"
        e.light = e.uav["lit"]
        return e

    @staticmethod
    def _uav_vote(e: MapEntity, feat: dict) -> dict:
        """The UAV's colour / light on an entity = the majority of every UAV sighting of it (a single wrong-colour message must not
        recolour a buoy - found in the fast SIL: the last message's 5 % miscolour turned the entry red)."""
        cv = getattr(e, "uav_cvotes", None) or {}
        lv = getattr(e, "uav_lvotes", None) or {}
        if feat.get("colour"):
            cv[feat["colour"]] = cv.get(feat["colour"], 0) + 1
        if feat.get("lit") not in (None, "?"):
            lv[feat["lit"]] = lv.get(feat["lit"], 0) + 1
        e.uav_cvotes, e.uav_lvotes = cv, lv
        out = dict(feat)
        out["colour"] = max(cv.items(), key=lambda kv: kv[1])[0] if cv else None
        out["lit"] = max(lv.items(), key=lambda kv: kv[1])[0] if lv else "?"
        out["votes"] = dict(cv)
        return out

    def _apply_uav(self, e: MapEntity) -> None:
        """The effective colour (D22): the UAV's under colour_source uav (m1t2) or when the camera's vote is unknown; otherwise the
        camera's stays and a disagreement is flagged (the UAV card / the map: 'UAV red - camera green')."""
        u = getattr(e, "uav", None)
        if not u or not u.get("colour"):
            return
        cam, cam_light = classify_light(e.colour_votes) if e.colour_votes else ("unknown", "?")   # the camera's own, from its votes
        e.cam_colour = cam
        if self.colour_source == "uav" or cam in ("unknown", "", None):
            e.colour = u["colour"]
            e.light = u["lit"] if u.get("lit") in ("flash", "steady", "off") else e.light
            e.uav_disagree = False
        else:
            e.colour, e.light = cam, cam_light
            e.uav_disagree = cam != u["colour"]
            self._led_apply(e)

    def _led_vote(self, e: MapEntity, led, now: float) -> None:
        """v0.1.92 (Dmitrii 2026-11-13): one sighting's LED reading (cv_publish passes the observer's buoys[].led as quality.led:
        pattern steady | flashing | off | unknown, colour). The decided ones (flashing / steady with a colour) are kept LED_WINDOW_S; then
        _led_apply. The light reader sees the lamp under the hat directly; the colour votes only see a wavering body colour."""
        hist = getattr(e, "led_hist", None)
        if hist is None:
            hist = e.led_hist = []
        if isinstance(led, dict) and led.get("pattern") in ("flashing", "steady") and led.get("colour") in ("red", "green", "blue"):
            hist.append((now, led["pattern"], led["colour"]))
        while hist and now - hist[0][0] > LED_WINDOW_S:
            hist.pop(0)
        self._led_apply(e)

    def fuse_depth(self, msg: dict, pose: LocalPose, now: float) -> set:
        """v0.1.94 (Dmitrii: "if we use the depth picture ..."; 13 Nov pm: 321 near boxes had no valid depth inside the box, while the
        guard's levelled cloud saw a solid thing on the same bearing in the same frame - and mapped it again as a depth obstacle). Each
        guard `far` cluster whose bearing span (+- fuse_tol_deg) holds exactly one depthless buoy box of the same frame (t_cam within
        0.1 s) is that buoy: it is folded in at the cluster's range (+ the buoy radius) - its colour / number with it, its lamp sample was
        already taken - and the cluster is not mapped again. Returns the indices of the clusters used."""
        nd = getattr(self, "_no_depth", None)
        far = (msg or {}).get("far") or []
        used: set = set()
        if not nd or not nd["items"] or not far or abs(float(msg.get("t_cam") or 0.0) - nd["t"]) > 0.1:
            return used
        tol = float(self.depth_fuse_tol_deg)
        taken = set()
        for i, c in enumerate(far):
            try:
                r, b0, b1 = float(c["range_m"]), float(c.get("bearing_min_deg", c["bearing_deg"])), float(c.get("bearing_max_deg", c["bearing_deg"]))
            except (KeyError, TypeError, ValueError):
                continue
            hit = [it for it in nd["items"] if id(it) not in taken and b0 - tol <= it.bearing_deg <= b1 + tol and self._size_ok(it, r)]
            if len(hit) != 1:
                continue
            it = hit[0]
            taken.add(id(it))
            q = dict(it.quality or {}, depth_from="guard", bearing_only=False)
            led = q.pop("led", None)                   # the lamp sample went in with the box already; its reading stays for colour_now
            if isinstance(led, dict) and "on" in led:
                q["lamp_now"] = led.get("now") if led.get("on") else "black"
            fused = ObsItem(id=it.id, cls=it.cls, colour=it.colour, conf=it.conf, range_m=r + 0.2, bearing_deg=it.bearing_deg,
                            elev_deg=it.elev_deg, quality=q)
            self.ingest(Observation(t_cam=nd["t"], items=[fused]), nd["pose"], now=now, source=nd["source"])
            used.add(i)
        self.n_depth_fused = getattr(self, "n_depth_fused", 0) + len(used)
        return used

    def _size_ok(self, it, r: float) -> bool:
        """The box's height agrees with the range (range x box height ~ depth_fuse_k px*m: 13 Nov, 1090 ranged boxes < 15 m at hd1080,
        median 1000, 10-90 % 820-1210) within a factor of 1.6 - a box far behind a near thing on the same bearing is not that thing."""
        k = float(self.depth_fuse_k)
        h = (it.quality or {}).get("bbox_h")
        if k <= 0 or not h:
            return True
        ratio = r / (k / float(h))
        q = float(self.depth_fuse_ratio)
        return 1.0 / q <= ratio <= q

    # ---- v0.1.94: the merge pass (Dmitrii 2026-11-14: "merge radius growth by distance, but also by detected number") ----------
    def merge_radius(self, a: MapEntity, b: MapEntity) -> float:
        """How far apart two camera buoys may be and still be one: a + b * r, r = the CLOSEST range either was ever seen from (a buoy
        seen near is placed well, so it stays tight - the course's markers are 1.4-3 m apart; two far-only ghosts of one buoy are
        metres apart at 20 m), clamped [min_m, max_m]; the same number read on both -> number_m."""
        p = self.merge_p
        na, nb = getattr(a, "number", None), getattr(b, "number", None)
        if na is not None and na == nb:
            return float(p["number_m"])
        la, lb = getattr(a, "led", None), getattr(b, "led", None)   # one decided lamp seen twice (13 Nov pm: the entry as two flashing blues 2.3 m apart)
        if la and lb and (la["colour"], la["pattern"]) == (lb["colour"], lb["pattern"]) and float(p.get("lamp_m") or 0) > 0:
            return float(p["lamp_m"])
        r = min(float(getattr(a, "min_range_m", 1e9) or 1e9), float(getattr(b, "min_range_m", 1e9) or 1e9))
        if r >= 1e8:
            r = 25.0
        return min(float(p["max_m"]), max(float(p["min_m"]), float(p["a"]) + float(p["b"]) * r))

    @staticmethod
    def merge_veto(a: MapEntity, b: MapEntity) -> bool:
        """Never one buoy: two different numbers, or two different lamp colours (a pattern can differ between two copies of one lamp:
        13 Nov pm, the entry read steady on one copy, flashing on the other)."""
        na, nb = getattr(a, "number", None), getattr(b, "number", None)
        if na is not None and nb is not None and na != nb:
            return True
        la, lb = getattr(a, "led", None), getattr(b, "led", None)    # v0.1.94: not the pattern (truth: the entry split, steady + flashing)
        ca = (getattr(a, "lamp", None) or {}).get("colour") or (la or {}).get("colour")
        cb = (getattr(b, "lamp", None) or {}).get("colour") or (lb or {}).get("colour")
        return bool(ca and cb and ca != cb)

    def merge_buoys(self, now: float) -> int:
        """Camera buoys closer than merge_radius (no veto) -> one (the better seen keeps its id); returns how many merged."""
        n = 0
        while True:
            ents = [e for e in self.ents.values() if e.kind == "buoy" and e.source == "usv" and e.n_obs >= self.m.min_obs]
            best = None
            for i, a in enumerate(ents):
                for b in ents[i + 1:]:
                    d = math.hypot(a.x_n - b.x_n, a.y_e - b.y_e)
                    if d <= self.merge_radius(a, b) and not self.merge_veto(a, b) and (best is None or d < best[0]):
                        best = (d, a, b)
            if best is None:
                return n
            _, a, b = best
            keep, gone = (a, b) if (a.n_obs, -a.id) >= (b.n_obs, -b.id) else (b, a)
            self._absorb(keep, gone)
            n += 1

    def _absorb(self, keep: MapEntity, gone: MapEntity) -> None:
        w1, w2 = keep.n_obs, gone.n_obs
        keep.x_n = (keep.x_n * w1 + gone.x_n * w2) / (w1 + w2)
        keep.y_e = (keep.y_e * w1 + gone.y_e * w2) / (w1 + w2)
        keep.n_obs = w1 + w2
        keep.t_first, keep.t_last = min(keep.t_first, gone.t_first), max(keep.t_last, gone.t_last)
        for k, v in gone.colour_votes.items():
            keep.colour_votes[k] = keep.colour_votes.get(k, 0) + v
        if keep.colour_votes:
            keep.colour, keep.light = classify_light(keep.colour_votes)
        keep.led_hist = sorted((getattr(keep, "led_hist", None) or []) + (getattr(gone, "led_hist", None) or []))
        if not getattr(keep, "led", None) and getattr(gone, "led", None):
            keep.led = gone.led
        if not (getattr(keep, "lamp", None) or {}).get("colour") and getattr(gone, "lamp", None):
            keep.lamp = gone.lamp
        nv = dict(getattr(keep, "num_votes", None) or {})
        for k, v in (getattr(gone, "num_votes", None) or {}).items():
            nv[k] = nv.get(k, 0) + v
        if nv:
            keep.num_votes, keep.num_n = nv, getattr(keep, "num_n", 0) + getattr(gone, "num_n", 0)
            top = max(nv, key=nv.get)
            keep.number = top if (keep.num_n >= 2 and nv[top] >= 0.7 * sum(nv.values())) else None
        keep.min_range_m = min(float(getattr(keep, "min_range_m", 1e9) or 1e9), float(getattr(gone, "min_range_m", 1e9) or 1e9))
        keep.ranged = bool(getattr(keep, "ranged", False) or getattr(gone, "ranged", False))
        for tr in self.lamps.tracks.values():
            if tr.ent == gone.id:
                tr.ent = keep.id
        self._led_apply(keep)
        del self.ents[gone.id]
        self.merged[gone.id] = keep.id
        for k, v in list(self.merged.items()):
            if v == gone.id:
                self.merged[k] = keep.id

    def resolve(self, eid: Optional[int]) -> Optional[int]:
        """An id the merge pass took away -> the id it went into (None stays None)."""
        seen = 0
        while eid is not None and eid in self.merged and seen < 50:
            eid, seen = self.merged[eid], seen + 1
        return eid

    # ---- v0.1.94: lamps by world bearing (usv/lamp_tracks), numbers -------------------------------------------------------
    def _lamp_frame(self, items, pose: LocalPose, t: float) -> dict:
        """One frame's buoy items -> {id(item): LampTrack}: each sighting's lamp sample (quality.led on / now) at its world azimuth.
        Items without a lamp reading (an older observer) get none."""
        if not self.lamps_on:
            return {}
        f = self.cfg.frames
        ox, oy = body_to_local(0.0, 0.0, pose, f.cam_forward_m, f.cam_right_m, f.cam_yaw_deg)
        its, obs = [], []
        for it in items:
            led = (it.quality or {}).get("led")
            if it.conf < self.m.min_conf or not isinstance(led, dict) or "on" not in led:
                continue
            on = bool(led.get("on"))
            col = led.get("now") if led.get("now") in ("red", "green", "blue") else None
            its.append(it)
            h = (it.quality or {}).get("bbox_h")
            rh = (self.depth_fuse_k / float(h)) if (h and self.depth_fuse_k > 0) else None
            obs.append((ox, oy, wrap360(pose.hdg_deg + f.cam_yaw_deg + it.bearing_deg), on and col is not None, col, rh))
        return {id(it): tr for it, tr in zip(its, self.lamps.frame(float(t), obs))} if obs else {}

    def _entity_on(self, ox: float, oy: float, az: float, prefer: Optional[int] = None, r_hint: Optional[float] = None) -> Optional[MapEntity]:
        """The tracked buoy along this azimuth from (ox, oy) within the lamp tolerance: with the box's size range (r_hint) the one whose
        distance agrees best (within x/÷ 1.6 - truth 13 Nov pm: marker 8's red lamp went to a buoy 3 m off on nearly the same bearing),
        else the one the lamp already belongs to, else the smallest angle off. None when no buoy fits (the lamp waits for its own)."""
        tol = float(self.lamps.p["az_tol_deg"])
        cands = []
        for e in self.ents.values():
            r = math.hypot(e.x_n - ox, e.y_e - oy)
            if e.kind != "buoy" or e.state != "tracked" or e.source == "uav" or r < 1.0:
                continue
            d = abs(wrap180(math.degrees(math.atan2(e.y_e - oy, e.x_n - ox)) - az))
            if d <= tol:
                cands.append((e, d, r))
        if not cands:
            return None
        if r_hint:
            fit = [(abs(math.log(r / r_hint)), e) for e, d, r in cands if 1.0 / 1.6 <= r / r_hint <= 1.6]
            return min(fit, key=lambda q: q[0])[1] if fit else None
        for e, d, r in cands:
            if e.id == prefer:
                return e
        return min(cands, key=lambda q: q[1])[0]

    def _lamp_link(self, e: MapEntity, trk, now: float) -> None:
        """The lamp track's result onto its buoy: a decided pattern (flashing / steady with a colour) is one LED reading (led_hist,
        as the observer's); the colour alone (>= min_on lit samples agreeing) colours the buoy until a pattern is decided (_led_apply)."""
        trk.ent = e.id
        r = trk.result or {}
        e.lamp = {"colour": r.get("colour"), "pattern": r.get("pattern"), "n": r.get("n"), "on_frac": r.get("on_frac"), "track": trk.tid}
        if r.get("pattern") in ("flashing", "steady") and r.get("colour") in ("red", "green", "blue"):
            hist = getattr(e, "led_hist", None)
            if hist is None:
                hist = e.led_hist = []
            hist.append((now, r["pattern"], r["colour"]))
            while hist and now - hist[0][0] > max(LED_WINDOW_S, float(self.lamps.p["window_s"])):
                hist.pop(0)
        self._led_apply(e)

    @staticmethod
    def _lamp_now(it) -> Optional[str]:
        """The lamp reading of one sighting: red / green / blue lit, "black" off, None without a reading (an older observer)."""
        q = it.quality or {}
        if "lamp_now" in q:
            return q["lamp_now"]
        led = q.get("led")
        if not isinstance(led, dict) or "on" not in led:
            return None
        return led.get("now") if (led.get("on") and led.get("now") in ("red", "green", "blue")) else "black"

    @staticmethod
    def _number_vote(e: MapEntity, num) -> None:
        """v0.1.94: the observer's number reader (quality.number = {digit, p}): weighted votes; the buoy's number = the top digit once
        it holds >= 2 reads and >= 70 % of the weight (else None). A number is the strongest same-buoy evidence (merge)."""
        if not isinstance(num, dict) or num.get("digit") is None:
            return
        try:
            d, w = int(num["digit"]), float(num.get("p") or 0.5)
        except (TypeError, ValueError):
            return
        v = getattr(e, "num_votes", None)
        if v is None:
            v = e.num_votes = {}
        v[d] = round(v.get(d, 0.0) + w, 3)
        e.num_n = getattr(e, "num_n", 0) + 1
        tot = sum(v.values())
        top = max(v, key=v.get)
        e.number = top if (e.num_n >= 2 and v[top] >= 0.7 * tot) else None

    def _led_apply(self, e: MapEntity) -> None:
        """The LED's colour + light on the entity when LED_MIN sightings agree (LED_AGREE); else the votes' (classify_light) stay.
        Not under colour_source uav (m1t2: the UAV's colour rules). The memory outlives the view: once read, it stays until replaced."""
        if self.colour_source == "uav":
            return
        hist = getattr(e, "led_hist", None) or []
        if len(hist) >= LED_MIN:
            keys = [(p, c) for _, p, c in hist]
            top = max(set(keys), key=keys.count)
            if keys.count(top) >= LED_AGREE * len(keys):
                e.colour, e.light = top[1], ("flash" if top[0] == "flashing" else "steady")
                e.led = {"pattern": top[0], "colour": top[1], "n": len(keys)}
                return
        if getattr(e, "led", None):
            e.colour, e.light = e.led["colour"], ("flash" if e.led["pattern"] == "flashing" else "steady")
            return                                      # once read, it stays until replaced
        lamp = getattr(e, "lamp", None)                 # v0.1.94: the lamp's colour before its pattern is decided
        if lamp and lamp.get("colour") in ("red", "green", "blue"):
            e.colour = lamp["colour"]
            e.light = "?"

    def provisional_gates(self) -> List[tuple]:
        """Red / green pairs of UAV-only buoys within the gate width (D22): drawn dashed, used by T1 as leads, never planned through."""
        uo = [e for e in self.ents.values() if e.source == "uav" and e.kind == "buoy" and e.n_obs >= self.uav_min_obs and self.in_course(e.x_n, e.y_e)]
        reds = [e for e in uo if e.colour == "red"]
        greens = [e for e in uo if e.colour == "green"]
        pairs = sorted(((math.hypot(r.x_n - g.x_n, r.y_e - g.y_e), r, g) for r in reds for g in greens
                        if self.m.gate_width_min_m <= math.hypot(r.x_n - g.x_n, r.y_e - g.y_e) <= self.m.gate_width_max_m), key=lambda p: p[0])
        used, out = set(), []
        for _, r, g in pairs:
            if r.id in used or g.id in used:
                continue
            used.update((r.id, g.id))
            out.append((r, g, (r.x_n + g.x_n) / 2, (r.y_e + g.y_e) / 2))
        return out

    def forget_uav(self) -> int:
        """v0.1.93.2: m1t1 starts - drop the UAV-only buoys and the UAV's votes on the camera's (an earlier flight's map must not steer a
        USV-only run); a camera buoy coloured by the UAV goes back to its own votes. Returns the buoys dropped."""
        gone = [k for k, e in self.ents.items() if e.source == "uav"]
        for k in gone:
            del self.ents[k]
        for e in self.ents.values():
            if getattr(e, "uav", None):
                e.uav = None
                e.colour, e.light = classify_light(e.colour_votes) if e.colour_votes else ("unknown", "?")
        return len(gone)

    def uav_entities(self, include_outside: bool = False) -> List[MapEntity]:
        return [e for e in self.ents.values() if e.kind == "buoy" and e.source == "uav" and e.n_obs >= self.uav_min_obs
                and (include_outside or self.in_course(e.x_n, e.y_e))]

    def uav_entry_candidate(self) -> Optional[MapEntity]:
        """A flashing blue in the UAV layer (UAV-only or linked) - what WAIT_UAV_MAP needs before the boat transits (D22c)."""
        c = [e for e in self.ents.values() if e.kind == "buoy" and getattr(e, "uav", None) and e.uav.get("colour") == "blue"
             and e.uav.get("lit") == "flash" and self.in_course(e.x_n, e.y_e)]
        return max(c, key=lambda e: e.n_obs) if c else None

    def uav_digest(self, now: float) -> dict:
        st = dict(self.uav_stats)
        pub = st["published"]
        return {"colour_source": self.colour_source, "frozen": self.uav_frozen, "scan": dict(self.uav_scan),
                "age_s": None if self.uav_last_t < -1e8 else round(now - self.uav_last_t, 1),
                "change_age_s": None if self.uav_change_t < -1e8 else round(now - self.uav_change_t, 1),   # v0.1.89: MAP QUIET
                "assoc_m": self.uav_assoc_m, "stats": st, "assoc_rate": round(st["linked"] / pub, 3) if pub else None,
                "n_uav_only": len(self.uav_entities()),
                "feeds": {k: {**v, "age_s": None if v["last_t"] is None else round(now - v["last_t"], 1), "first_t": None, "last_t": None,
                              "status_age_s": None if v["status_t"] is None else round(now - v["status_t"], 1), "status_t": None}
                          for k, v in self.uav_feeds.items()},
                "n_linked": sum(1 for e in self.ents.values() if getattr(e, "uav", None) and e.source != "uav"),
                # v0.1.93.2 (Dmitrii 13 Nov: "which buoys we receive"): each drone buoy - linked to a camera track (the boat's position) or
                # UAV-only (the drone's position), the UAV's colour / light and what the map uses; the m1t1 gate's ignored count
                "items": [{"id": e.id, "src": "uav" if e.source == "uav" else "linked", "colour": e.colour, "lit": e.light,
                           "uav_colour": (e.uav or {}).get("colour"), "uav_lit": (e.uav or {}).get("lit"), "uav_id": (e.uav or {}).get("id"),
                           "n_cam": 0 if e.source == "uav" else e.n_obs, "x_n": round(e.x_n, 1), "y_e": round(e.y_e, 1)}
                          for e in sorted((q for q in self.ents.values() if getattr(q, "uav", None)), key=lambda q: q.id)][:20],
                "ignored": int(self.uav_stats.get("ignored", 0)), "ingest_off": bool(self.uav_ingest_off)}
