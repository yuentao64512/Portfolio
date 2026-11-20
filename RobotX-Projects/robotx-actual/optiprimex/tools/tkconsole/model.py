"""K2: the console's view-model - pure Python, no Tk. ConsoleModel.apply(state, fast, alarms) -> View: display strings and flags only.

Every rule here is the page's (tools/ocs_web.html drawLadder / fcuBtns / the tiles), read from the same /state.json; nothing is
fabricated - a missing field shows as "?" or "—". The widgets (views.py) write a string only when it changed (by diff)."""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

STALE_S = 3.0              # ocs_age above this -> STALE (amber)
DOWN_MISSES = 3            # this many failed /state.json polls in a row -> DOWN (red)
LOG_N = 300
REC_STALL_S = 10.0         # REC on and the file not growing this long -> REC STOPPED (red)

# a button's word -> the playbook action whose verdict (state.verdicts) says whether it is allowed now, and why not
WORD_ACTION = {"declare": "DECLARE", "enable_auto": "ENABLE_AUTO", "disable_auto": "DISABLE_AUTO", "start_task": "START_TASK",
               "hold": "HOLD", "resume": "RESUME", "abort": "ABORT_TASK", "end_run": "END_RUN", "arm": "FCU_ARM", "disarm": "FCU_ARM",
               "fcu_mode": "FCU_MODE", "station_keep": "STATION_KEEP", "usv": "USV_READY", "report": "REPORT", "uav": "UAV",
               "declare uav": "DECLARE_UAV", "mission": None, "stop_reset": None, "confirm_safe": "CONFIRM_SAFE"}
LOG_FILTERS = ("all", "ui", "judge", "usv", "uav", "cautions")


@dataclass
class Chip:
    text: str
    level: str = ""          # ok | warn | bad | idle | info | ''  (the colour; the text always says it in words)


@dataclass
class Btn:
    word: str                # the op-tui line send() posts - the one code path for every button
    label: str
    ok: bool = True          # the playbook's verdict; False = greyed, the why + hint shown under it (still pressable: the backend decides)
    why: str = ""
    hint: str = ""
    kind: str = ""           # act (the one live control) | go | danger | on | ''
    confirm: str = ""        # non-empty = asks first (ARM, STOP & RESET, declare uav off once declared)
    key: str = ""            # stable widget id (the views build each button once)


@dataclass
class Step:
    n: int
    title: str
    state: str               # now | done | next
    chip: str
    detail: str
    warn: str = ""
    done_at: str = ""
    buttons: List[Btn] = field(default_factory=list)
    sticky: bool = False     # v0.1.90.6: the buttons shown whatever the step's state (the crew's heartbeat toggles)


@dataclass
class Rec:
    text: str
    level: str               # bad (REC STOPPED) | ok (on) | warn (inferred) | idle (off)
    on: bool
    inferred: bool
    file: str = ""


@dataclass
class LogLine:
    t: float
    kind: str
    text: str
    sev: str
    group: str               # ui | judge | usv | uav (the filter buttons; cautions = sev warn / bad)


@dataclass
class MapLayer:
    pose: Optional[Tuple[float, float]] = None        # lat, lon
    hdg_deg: Optional[float] = None
    polygon: List[Tuple[float, float]] = field(default_factory=list)
    polygon_src: str = ""                              # received | default | ""
    geofence: List[Tuple[float, float]] = field(default_factory=list)
    points: List[dict] = field(default_factory=list)
    items: List[dict] = field(default_factory=list)    # perception items (not in /state.json yet: docs/TK_CONSOLE.md asks)
    uav_buoys: List[dict] = field(default_factory=list)
    obstacles: List[dict] = field(default_factory=list)
    visited: List[dict] = field(default_factory=list)  # the scout log's "B3 logged lat lon" lines
    target: Optional[dict] = None


@dataclass
class View:
    t: float
    title: str
    link: str                # UP | STALE | DOWN | WAITING
    link_level: str
    header: Dict[str, Chip]
    word: Chip               # the bar's state word (NOT READY / ARMED / RUNNING / HELD ...) + why
    hint: str
    steps: List[Step]
    usv: Dict[str, str]
    usv_buttons: List[Btn]
    uav: Dict[str, str]
    uav_buttons: List[Btn]
    missions: List[Tuple[str, str]]                    # (code to send, label)
    mission_armed: str
    alarms: List[dict]
    alarms_unacked: int
    alarms_level: str
    log: List[LogLine]
    map: MapLayer
    rec: Rec
    stale: bool
    down: bool
    events: List[Tuple[str, str]] = field(default_factory=list)   # (level, text): the state-transition toasts (K5) since the last apply
    profile: str = ""                # v0.1.89: the ladder's shape - "USV only" | "UAV only · boat holds" | "UAV + USV"
    banner: Optional[dict] = None    # v0.1.89: {title, stage, level} from RunStart to the end of the run; None = no banner
    #                                  v0.1.92: + fcu (FCU <MODE> · ARMED|disarmed · <V>), why, next, mods (the boat's micro-stage)
    mods: Optional[dict] = None      # v0.1.92: the "mission modifiers" subpanel {line, frozen, why, buttons}; None = not shown


def mss(s) -> str:
    s = max(0, int(round(float(s or 0))))
    return f"{s // 60}:{s % 60:02d}"


def log_group(kind: str, text: str) -> str:
    k, t = (kind or "").lower(), (text or "").lower()
    if k in ("ui", "cmd", "intent"):
        return "ui"
    if k == "judge":
        return "judge"
    if "uav" in t or k == "uav":
        return "uav"
    return "usv"


def _verdict(state: dict, word: str) -> Tuple[bool, str, str]:
    head = " ".join(word.split()[:2]) if word.startswith("declare uav") else word.split()[0]
    act = WORD_ACTION.get(head)
    v = (state.get("verdicts") or {}).get(act) if act else None
    if not v:
        return True, "", ""
    return bool(v.get("ok")), str(v.get("why") or ""), str(v.get("hint") or "")


def btn(state: dict, word: str, label: str, kind: str = "", confirm: str = "", key: str = "") -> Btn:
    ok, why, hint = _verdict(state, word)
    return Btn(word=word, label=label, ok=ok, why="" if ok else why, hint="" if ok else hint, kind=kind, confirm=confirm,
               key=key or word)


ARM_CONFIRM = "ARM the FCU? The props can turn as soon as a mode drives them; the FCU's pre-arm checks decide; the transmitter's kill still wins."
END_CONFIRM = ("END RUN? STOP_RUN to every declared vehicle - the judged attempt is over (R6); a new attempt needs a new DECLARE. "
               "To stop the boat only: ❚❚ HOLD.")
STOP_CONFIRM = ("STOP & RESET? Abort the task · AUTO off · FCU HOLD · DISARM · the run reset (R2). Reporting stays as it is "
                "(the heartbeat shows MANUAL). Then rerun the sequence.")


# ---- v0.1.92: the mission modifiers (COWORK_HANDOFF addendum 6 §6.3) - m1t1 / m1t2's START_TASK args, set before ENABLE AUTO ---------
MOD_KEYS = ("start_mode", "look_lock", "scout_entry_s", "circle_exit_tangent", "squeeze", "lucky_pairs")   # = ocs/cmd_authority.MOD_ARGS
MOD_DEFAULTS = {"start_mode": "advance360", "look_lock": True, "scout_entry_s": 420, "circle_exit_tangent": False,
                "squeeze": False, "lucky_pairs": "one_known"}                              # = its MOD_DEFAULTS
MOD_FIXED = "orbit=bow 2 m · follow=bow"                                                  # not switchable (v0.1.93.1: squeeze is)
# v0.1.93.1 (Dmitrii 13 Nov morning: "after I tick, I want to reconfirm the mission"): the risky switches ask, naming the whole mission
CONFIRM_LUCKY = ("LUCKY ON? The boat takes the closest uncertain-colour pair inside the course as a gate (one buoy a known red / green) and goes "
                 "straight to the gates when no flashing blue entry is found; a blue buoy whose light stays undecided after its checks is orbited as the entry (a risk).")
CONFIRM_ANY = ("LUCKY ANY? Lucky on AND two UNKNOWN buoys may make a gate - the white number markers (1.4-2.8 m apart) look exactly like one. "
               "Only when the colours are clearly failing.")
CONFIRM_SQUEEZE = ("SQUEEZE ON? An unknown object near a gate is passed on the wider gap beside it at 0.08 m/s (its guard stop 0.3 m instead of "
                   "the hold). The simulator: no gain, one contact on the box-near-a-gate course. Off = the boat holds there (AUTO_SAFE_HOLD).")
MOD_FROZEN = "frozen from ENABLE AUTO on (DISABLE AUTO to change them)"


def mods_values(mi: Optional[dict]) -> dict:
    """The modifiers in force for the armed mission: what the OCS holds (mission.mods), else the boat's default; + lucky."""
    mi = mi or {}
    v = dict(MOD_DEFAULTS)
    v.update({k: x for k, x in (mi.get("mods") or {}).items() if k in MOD_KEYS and x is not None})
    v["lucky"] = bool(mi.get("lucky"))
    return v


def mods_line(mi: Optional[dict]) -> str:
    """The one line (the subpanel, the DECLARE text, the run banner): `mods: start=advance+360 · orbit=bow 2 m · ... · 420 s`."""
    v = mods_values(mi)
    oo = lambda b: "on" if b else "off"
    try:
        budget = f"{float(v['scout_entry_s']):.0f} s"
    except (TypeError, ValueError):
        budget = f"{v['scout_entry_s']} s"
    return (f"mods: start={'advance+360' if v['start_mode'] == 'advance360' else str(v['start_mode'])} · {MOD_FIXED} · "
            f"squeeze {oo(v['squeeze'])} · look-lock {oo(v['look_lock'])} · lucky {lucky_word(v)} · {budget} · exit-tangent {oo(v['circle_exit_tangent'])}")


def lucky_word(v: dict) -> str:
    """off | on | ANY (v0.1.93.1: lucky_pairs any = two unknowns may pair)."""
    return ("ANY" if v.get("lucky_pairs") == "any" else "on") if v.get("lucky") else "off"


def _t4_flag_word(t4) -> str:
    w = str(t4 or "").upper().replace("TIER_", "")
    return {"NONE": "off", "CORE": "core", "ADVANCED": "advanced"}.get(w, "")


def mods_flags(mods: Optional[dict]) -> str:
    """The ocs_cli flags for a modifier dict (only the keys present)."""
    out = []
    oo = lambda b: "on" if b else "off"
    m = mods or {}
    if m.get("start_mode") is not None:
        out.append(f"--start-mode {m['start_mode']}")
    if m.get("look_lock") is not None:
        out.append(f"--look-lock {oo(m['look_lock'])}")
    if m.get("scout_entry_s") is not None:
        out.append(f"--scout-entry-s {int(float(m['scout_entry_s']))}")
    if m.get("circle_exit_tangent") is not None:
        out.append(f"--exit-tangent {oo(m['circle_exit_tangent'])}")
    if m.get("squeeze") is not None:                                                     # v0.1.93.1
        out.append(f"--squeeze {oo(m['squeeze'])}")
    if m.get("lucky_pairs") is not None:
        out.append(f"--lucky-pairs {m['lucky_pairs']}")
    return " ".join(out)


def mission_word(mi: Optional[dict], **change) -> str:
    """The `mission` line that re-arms the armed mission with one modifier changed - every other part kept (auto, lucky, t4) and all
    four modifiers written out (what the panel shows is what the boat gets). '' = no mission armed."""
    mi = mi or {}
    code = str(mi.get("code") or "")
    if not code:
        return ""
    v = mods_values(mi)
    v.update({k: x for k, x in change.items() if k in MOD_KEYS + ("lucky",)})
    t4 = _t4_flag_word(mi.get("t4")) if mi.get("t4") is not None else ""
    return " ".join(x for x in (f"mission {code}", "--auto" if mi.get("auto") else "", "--lucky" if v["lucky"] else "",
                                f"--t4 {t4}" if t4 else "", mods_flags({k: v[k] for k in MOD_KEYS})) if x)


def drone_map_line(um: Optional[dict]) -> str:
    """v0.1.93.2 (Dmitrii 13 Nov): the drone's map as the boat received it, one line: the feed, how many buoys, linked to a camera track
    or UAV-only, and each buoy's colour / light (the UAV's) - '' when no drone map has reached the boat."""
    um = um or {}
    feeds = um.get("feeds") or {}
    items = um.get("items") or []
    if not feeds and not items:
        return ""
    age = um.get("age_s")
    head = "drone map: " + (f"{age:.0f} s old" if isinstance(age, (int, float)) else "no message yet")
    head += f" · {len(items)} buoy(s): {um.get('n_linked') or 0} linked to the camera, {um.get('n_uav_only') or 0} UAV-only"
    if um.get("ingest_off"):
        head += f" · IGNORED in m1t1 ({um.get('ignored') or 0} items)"
    def one(it):
        col = it.get("uav_colour") or it.get("colour") or "?"
        lit = it.get("uav_lit") or it.get("lit") or "?"
        where = f"→ boat #{it.get('id')} ({it.get('n_cam')}×)" if it.get("src") == "linked" else "UAV-only"
        return f"{col} {lit} {where}"
    return head + ("\n" + " · ".join(one(it) for it in items[:8]) + (f" · +{len(items) - 8}" if len(items) > 8 else "") if items else "")


def micro_of(s: Optional[dict]) -> Optional[dict]:
    """The boat's micro-stage {stage, sub, why, next, mods} (v0.1.92, the map digest's extra.micro via state.micro); None = an older boat."""
    m = (s or {}).get("micro")
    return m if isinstance(m, dict) and (m.get("stage") or m.get("sub")) else None


def micro_stage(m: dict) -> str:
    return " · ".join(str(x) for x in (m.get("stage"), m.get("sub")) if x not in (None, ""))


class ConsoleModel:
    """Holds what must survive between polls (the done times of the ladder, the REC growth, the previous transition flags)."""

    def __init__(self, vehicle_id: str = "USV1", clock=time.time):
        self.vehicle_id = vehicle_id
        self.clock = clock
        self.meta: dict = {}
        self.course: dict = {}
        self.done_t: Dict[int, str] = {}
        self._rec_size: Optional[float] = None
        self._rec_grew_t: Optional[float] = None
        self._prev: Optional[dict] = None
        self.last_ok_state: Optional[dict] = None
        self.uav_latch: dict = {}                 # v0.1.89: the drone seen flying / RTL (the UAV-only run: no boat task)
        self._run_key = None
        self.profile = ""

    # ---- inputs that change rarely ---------------------------------------------------------------------------------------
    def set_meta(self, meta: Optional[dict]) -> None:
        self.meta = meta or {}

    def set_course(self, course: Optional[dict]) -> None:
        self.course = course or {}

    # ---- the one entry -----------------------------------------------------------------------------------------------------
    def apply(self, state: Optional[dict], fast: Optional[dict] = None, alarms: Optional[dict] = None, misses: int = 0,
              now: Optional[float] = None) -> View:
        now = self.clock() if now is None else now
        if state:
            self.last_ok_state = state
        s = state or self.last_ok_state or {}
        st, o, bar = s.get("status") or {}, s.get("ocs") or {}, s.get("bar") or {}
        fv = (fast or {}).get("v") or {}
        down = misses >= DOWN_MISSES
        oa = s.get("ocs_age")
        stale = (not down) and bool(s) and (oa is None or oa > STALE_S or misses > 0)
        if not s:
            link, link_level = ("DOWN", "bad") if down else ("WAITING", "idle")
        else:
            link, link_level = ("DOWN", "bad") if down else (("STALE", "warn") if stale else ("UP", "ok"))
        header = self._header(s, st, o, bar, fv, now, link, link_level, misses)
        rec = self._rec(s, now)
        header["rec"] = Chip(rec.text, rec.level)
        word = bar.get("word") or ("NO DATA" if not s else "?")
        wlevel = {"RUNNING": "ok", "ARMED": "info", "HELD": "warn", "LINK LOST": "bad"}.get(word, "warn" if s else "idle")
        if bar.get("kill") or bar.get("estop"):
            wlevel = "bad"
        why = bar.get("why") or ""
        view = View(t=now, title=self._title(s), link=link, link_level=link_level, header=header,
                    word=Chip(f"{word}{' · ' + why if why else ''}", wlevel), hint=bar.get("hint") or "",
                    steps=self._ladder(s, st, o), profile=self.profile, banner=self._banner(s, st, o, fv), mods=self._mods(s, st, o), usv=self._usv_words(s, st, o, fv), usv_buttons=self._usv_buttons(s, st, o),
                    uav=self._uav_words(o), uav_buttons=self._uav_buttons(s, o), missions=self._missions(s, st),
                    mission_armed=str((o.get("mission") or {}).get("code") or ""),
                    alarms=list((alarms or {}).get("active") or (s.get("alarms") or {}).get("active") or []),
                    alarms_unacked=0, alarms_level="idle", log=self._log(s), map=self._map(s, st, fv), rec=rec,
                    stale=stale, down=down)
        view.alarms_unacked = sum(1 for a in view.alarms if not a.get("acked") and a.get("level") != "CAUTION")
        lv = [a.get("level") for a in view.alarms]
        view.alarms_level = "bad" if "ALARM" in lv else ("warn" if "WARNING" in lv else ("info" if lv else "idle"))
        view.events = self._transitions(s, st, o, rec, link) if state else []
        return view

    # ---- the header (K4) ---------------------------------------------------------------------------------------------------
    def _title(self, s: dict) -> str:
        prof = (self.course or {}).get("profile") or ((s.get("flavour") or {}).get("profile")) or "?"
        prof = os.path.basename(str(prof)).replace(".yaml", "") if prof else "?"
        ver = (self.meta or {}).get("version")
        if not ver:
            from ... import __version__ as ver
        return f"OCS · {prof} · v{ver}"

    def _header(self, s, st, o, bar, fv, now, link, link_level, misses) -> Dict[str, Chip]:
        h: Dict[str, Chip] = {}
        ids = list((o.get("vehicles") or {}).keys()) or [self.vehicle_id]
        u = o.get("uav") or {}
        if u.get("configured") and u.get("vehicle_id"):
            ids.append(str(u["vehicle_id"]))
        h["ident"] = Chip(" · ".join(ids), "")
        if not s:
            h["links"] = Chip("hub ? · boat ? · judge ?", "idle")
        else:
            hub = "hub ok" if s.get("hub_up") else "hub DOWN"
            sa = s.get("status_age")
            boat = "boat —" if sa is None else f"boat {sa:.1f} s"
            j = s.get("judge") or {}
            judge = "judge ok" if j.get("connected") else "judge LOST"
            lvl = "bad" if (not s.get("hub_up") or sa is None or sa > 5) else ("warn" if (not j.get("connected") or sa > 2) else "ok")
            h["links"] = Chip(f"{hub} · {boat} · {judge}", lvl)
        holder = {"mission manager": "mm", "usv_ui": "usv_ui"}.get(bar.get("holder"), bar.get("holder") or "?")
        mode = fv.get("mode") or st.get("mode") or "?"
        armed = fv.get("armed", st.get("armed"))
        aw = "armed" if armed is True else ("disarmed" if armed is False else "armed ?")
        kill = bool(fv.get("kill") or st.get("kill") or fv.get("estop") or st.get("estop"))
        h["fcu"] = Chip(f"{holder} · {mode} · {aw}" + (" · KILL" if kill else ""), "bad" if kill else ("warn" if armed else ""))
        h["v"] = Chip(f"{o.get('r') or '?'} · {fv.get('state') or st.get('state') or '?'}", "")
        g = st.get("guard") or {}
        chip = str(g.get("chip") or g.get("mode") or "").lower()
        if not g:
            h["guard"] = Chip("GUARD —", "idle")
        else:
            words = {"clear": ("GUARD clear", "ok"), "slow": ("GUARD SLOW", "warn"), "stop": ("GUARD STOP", "bad"), "lost": ("GUARD lost", "warn")}
            txt, lvl = words.get(chip, (f"GUARD {chip or '?'}", "warn"))
            rng = g.get("range_m")
            if rng is not None and float(rng) < 10:
                txt += f" · {float(rng):.1f} m"
            h["guard"] = Chip(txt, lvl)
        h["clock"] = Chip(time.strftime("%H:%M:%S", time.localtime(now)), "")
        h["link"] = Chip(link if link != "DOWN" else f"DOWN · {misses} polls missed", link_level)
        return h

    # ---- REC (K4): state.logging when session B adds it, else inferred -----------------------------------------------------
    def _rec(self, s: dict, now: float) -> Rec:
        lg = s.get("logging")
        if isinstance(lg, dict) and isinstance(lg.get("usv"), dict):
            u = lg["usv"]
            if not u.get("on"):
                self._rec_size = self._rec_grew_t = None
                return Rec("REC ○ off", "idle", False, False)
            f = os.path.basename(str(u.get("file") or "")) or "?"
            b = u.get("bytes")
            if b is not None:
                if self._rec_size is None or b > self._rec_size:
                    self._rec_size, self._rec_grew_t = b, now
            size = f" · {b / 1e6:.0f} MB" if b is not None else ""
            if self._rec_grew_t is not None and now - self._rec_grew_t > REC_STALL_S:
                return Rec(f"REC STOPPED · {f}{size}", "bad", True, False, f)
            return Rec(f"REC ● {f}{size}", "ok", True, False, f)
        ui = next((n.get("body") or {} for n in ((s.get("nodes") or {}).get("boat") or []) if n.get("node") == "usv_ui" and n.get("up")), None)
        if ui and ui.get("log"):
            f = os.path.basename(str(ui["log"]).rstrip("/"))
            return Rec(f"REC ● {f} · inferred", "warn", True, True, f)
        if any((ln.get("kind") == "log") for ln in (s.get("log") or [])):
            return Rec("REC ● · inferred", "warn", True, True)
        return Rec("REC ○ off" if s else "REC ?", "idle", False, not bool(s))

    # ---- the RUN ladder (v0.1.89, Dmitrii: "rebuild mission ladder ... mission 1: USV only, UAV only (boat holds), UAV+USV") ---------
    # Every press of the run on this one column, in order - no tile, no tab. The shape follows what was launched: UAV declared and the
    # boat's mission auto -> UAV + USV; UAV declared, no auto mission -> UAV only (the boat holds); else USV only. The step's buttons sit
    # in generic slots lad<n>-<i> (the views build them once).
    def _profile(self, o) -> str:
        u, mi = o.get("uav") or {}, o.get("mission") or {}
        if not (u.get("configured") and (u.get("declare") or u.get("declared"))):
            return "USV only"
        return "UAV + USV" if (mi.get("code") and mi.get("auto")) else "UAV only · boat holds"

    def _drone(self, s, o) -> dict:
        """The touchdown sequence: the boat's own (uav_gate, from its m1t2 / scout task) when it publishes one, else the console's from
        the UAV status (the UAV-only run: the boat runs no task) - flying -> map quiet -> RTL/LAND -> disarmed."""
        g = s.get("uav_gate")
        if isinstance(g, dict) and g.get("step"):
            return g
        fl = ((o.get("uav") or {}).get("flight") or {})
        lt = self.uav_latch
        if fl.get("armed_seen") or fl.get("airborne_seen") or fl.get("armed") is True:
            lt["flying"] = True
        if str(fl.get("mode") or "").upper() in ("RTL", "SMART_RTL", "LAND", "AUTO_RTL") and lt.get("flying"):
            lt["rtl"] = True
        ca = (s.get("uav_map") or {}).get("change_age_s")
        quiet = ca is None or float(ca) > 5.0
        down = bool(lt.get("flying")) and fl.get("armed") is False
        if not lt.get("flying"):
            step, text = "wait_flight", "waiting for the drone to take off"
        elif not down:
            step, text = ("rtl", "drone returning (RTL / LAND)") if lt.get("rtl") else ("flying", "drone flying - mapping")
        elif not quiet:
            step, text = "map_quiet", f"drone down - map settling ({float(ca):.0f} / 5 s)"
        else:
            step, text = "touchdown", "TOUCHDOWN ✓ - drone disarmed, map quiet"
        return {"flying": bool(lt.get("flying")), "rtl": bool(lt.get("rtl")), "disarmed": down, "map_quiet": quiet,
                "go": down and quiet and bool(lt.get("flying")), "step": step, "text": text}

    @staticmethod
    def _seq_line(d: dict) -> str:
        mark = lambda ok: "✓" if ok else "·"
        return (f"{mark(d.get('flying'))} FLYING  →  {mark(d.get('map_quiet') and d.get('flying'))} MAP QUIET  →  "
                f"{mark(d.get('rtl'))} RTL/LAND  →  {mark(d.get('disarmed'))} DISARMED")

    def _ladder(self, s, st, o) -> List[Step]:
        self.profile = prof = self._profile(o) if s else ""
        if not s:
            return [Step(n, t, "next", "—", "") for n, t in enumerate(TITLES, 1)]
        r = o.get("r") or "R0"
        try:
            ri = int(r[1:])
        except ValueError:
            ri = 0
        rk = (o.get("run_id"), o.get("declaration_seq"))
        if ri < 3 or rk != self._run_key:                                # a new run: the drone's latches start again
            self._run_key = rk
            if ri < 5:
                self.uav_latch = {}
        ui = bool(s.get("ui_active"))
        v = st.get("state") or "?"
        auto = v.startswith("AUTO")
        arm = st.get("armed") is True
        mi = o.get("mission") or {}
        c = s.get("clearance") or {}
        f = o.get("usv_flow") or {}
        u = o.get("uav") or {}
        uav = prof != "USV only"
        can_decl = r in ("R2", "R6") or (r == "R4" and bool(o.get("stalled")))
        declared = ri >= 3 and not can_decl
        steps: List[Step] = []

        def add(title, state, chip, detail="", warn="", buttons=(), sticky=False):
            n = len(steps) + 1
            bs = [Btn(**{**b.__dict__, "key": f"lad{n}-{i}"}) for i, b in enumerate(buttons)]
            steps.append(Step(n, title, state, chip, detail, warn, buttons=bs, sticky=sticky))

        # 1 handover
        add("handover — the boat's FCU to the mission manager", "now" if ui else "done", "waiting" if ui else "done",
            "type handover in usv_ui on the boat (the console never takes it)" if ui else "the mission manager holds the FCU")
        # 2 crew: the heartbeats + pilot ready (both vehicles when the UAV flies)
        hb_u, rd_u = bool(f.get("heartbeat")), bool(f.get("ready"))
        hb_a, rd_a = bool(u.get("report_on") or u.get("heartbeats_on")), bool(u.get("pilot_ready"))
        crew_ok = hb_u and rd_u and (not uav or (hb_a and rd_a))
        boat_ok = hb_u and rd_u                 # v0.1.92: ENABLE AUTO is the boat's - a drone still getting ready never hides its button
        crew_wait = ", ".join(x for x in (("USV heartbeat off (▶ HEARTBEAT USV)" if not hb_u else ""), ("USV not ready" if hb_u and not rd_u else ""),
                                          ("UAV heartbeat off (▶ HEARTBEAT UAV)" if uav and not hb_a else ""),
                                          ("UAV pilot ready not back yet (the OCS sends it; the UAV adapter / their listener up?)" if uav and hb_a and not rd_a else "")) if x)
        # v0.1.90.6 (Dmitrii: "I still need heartbeat button preserved per vehicle"): a toggle per vehicle, always shown; no PILOT READY
        bs = [btn(s, "report off usv" if hb_u else "report on usv", "✓ HEARTBEAT USV" if hb_u else "▶ HEARTBEAT USV", "on" if hb_u else "go",
                  "Stop the USV's heartbeat? RoboCommand stops seeing the boat." if hb_u else "")]
        if uav:
            bs += [btn(s, "report off uav" if hb_a else "report on uav", "✓ HEARTBEAT UAV" if hb_a else "▶ HEARTBEAT UAV", "on" if hb_a else "go",
                       "Stop the UAV's heartbeat? RoboCommand stops seeing the drone." if hb_a else "")]
        add("crew — heartbeats on the RC page, pilots ready", "done" if crew_ok else ("now" if not ui else "next"),
            "✓" if crew_ok else "—",
            f"USV {'♥' if hb_u else '○'} {'ready' if rd_u else 'getting ready…'}" + (f" · UAV {'♥' if hb_a else '○'} {'ready' if rd_a else 'getting ready… (the OCS sends pilot ready by itself)'}" if uav else ""),
            buttons=bs, sticky=True)
        # v0.1.90.7 (Dmitrii at the venue): the course order is ENABLE AUTO (boat LOITER, the drone GUIDED by its pilot), then DECLARE
        # (ocs.auto_before_declare); else DECLARE first (the boat refuses ENABLE_AUTO before PREPARE: guards.require_prepare_for_auto)
        if o.get("auto_before_declare", True) is not False:          # v0.1.90.8: the rule everywhere (False = a profile's old order)
            # 4 autonomy (after DECLARE): ARM if needed, ENABLE AUTO (the boat holds in AUTO · LOITER)
            add("ENABLE AUTO — the boat in AUTO · LOITER", "done" if auto else ("now" if (declared or (o.get("auto_before_declare", True) is not False and boat_ok and not ui)) else "next"),
                "AUTO" if auto else ("armed" if arm else "disarmed"),
                (f"{v} · mode {st.get('mode') or '?'}" if auto else "ARM asks first; the transmitter's kill still wins")
                + (f"\nthe drone: {((u.get('flight') or {}).get('mode') or '?')} - its pilot puts it in GUIDED (RC)" if uav else ""),
                buttons=([] if arm else [btn(s, "arm", "ARM", "go", ARM_CONFIRM)]) + ([] if auto else [btn(s, "enable_auto", "▶ ENABLE AUTO\nHOLD → LOITER", "big")]))
            # 3 declare FIRST - the boat refuses ENABLE_AUTO before PREPARE (v0.1.90.1, the venue 16:52) (the tiers it sends, T4 incl. - no JUDGE tab needed)
            t = ((o.get("declaration_checks") or {}).get("tiers") or {})
            tl = " · ".join(f"{k} {str(t[k]).replace('TIER_', '')}" for k in ("T1", "T2", "T3", "T4") if k in t)
            who = "USV1 + " + str(u.get("vehicle_id") or "UAV") if uav else "USV1"
            add("DECLARE — the RunDeclaration", "done" if declared else ("now" if crew_ok and not ui and (c.get("clear") or can_decl) and (auto or o.get("auto_before_declare", True) is False) else "next"),
                f"seq {o.get('declaration_seq')}" if declared else "—",
                f"declares {who}" + (f" · {tl}" if tl else "") + (f" · mission {mi.get('code')}" if mi.get("code") else "")
                + (" · LUCKY gates (uncertain colours)" if mi.get("lucky") else "")
                + (("\n" + mods_line(mi)) if self._mods_apply(prof, mi) else ""),          # v0.1.92: the modifiers in the DECLARE text
                ("" if declared or c.get("clear") or not crew_ok else "not clear: " + " · ".join(c.get("blocked") or []))
                or ("" if declared or crew_ok or ui else "waiting for the crew: " + crew_wait),       # v0.1.92: never a silent grey step
                buttons=[] if declared else [btn(s, "declare", "DECLARE", "act")])
        else:
            # 3 declare FIRST - the boat refuses ENABLE_AUTO before PREPARE (v0.1.90.1, the venue 16:52) (the tiers it sends, T4 incl. - no JUDGE tab needed)
            t = ((o.get("declaration_checks") or {}).get("tiers") or {})
            tl = " · ".join(f"{k} {str(t[k]).replace('TIER_', '')}" for k in ("T1", "T2", "T3", "T4") if k in t)
            who = "USV1 + " + str(u.get("vehicle_id") or "UAV") if uav else "USV1"
            add("DECLARE — the RunDeclaration", "done" if declared else ("now" if crew_ok and not ui and (c.get("clear") or can_decl) and (auto or o.get("auto_before_declare", True) is False) else "next"),
                f"seq {o.get('declaration_seq')}" if declared else "—",
                f"declares {who}" + (f" · {tl}" if tl else "") + (f" · mission {mi.get('code')}" if mi.get("code") else "")
                + (" · LUCKY gates (uncertain colours)" if mi.get("lucky") else "")
                + (("\n" + mods_line(mi)) if self._mods_apply(prof, mi) else ""),          # v0.1.92: the modifiers in the DECLARE text
                ("" if declared or c.get("clear") or not crew_ok else "not clear: " + " · ".join(c.get("blocked") or []))
                or ("" if declared or crew_ok or ui else "waiting for the crew: " + crew_wait),       # v0.1.92: never a silent grey step
                buttons=[] if declared else [btn(s, "declare", "DECLARE", "act")])
            # 4 autonomy (after DECLARE): ARM if needed, ENABLE AUTO (the boat holds in AUTO · LOITER)
            add("ENABLE AUTO — the boat in AUTO · LOITER", "done" if auto else ("now" if (declared or (o.get("auto_before_declare", True) is not False and boat_ok and not ui)) else "next"),
                "AUTO" if auto else ("armed" if arm else "disarmed"),
                (f"{v} · mode {st.get('mode') or '?'}" if auto else "ARM asks first; the transmitter's kill still wins")
                + (f"\nthe drone: {((u.get('flight') or {}).get('mode') or '?')} - its pilot puts it in GUIDED (RC)" if uav else ""),
                buttons=([] if arm else [btn(s, "arm", "ARM", "go", ARM_CONFIRM)]) + ([] if auto else [btn(s, "enable_auto", "▶ ENABLE AUTO\nHOLD → LOITER", "big")]))
        # 5 RunStart
        add("RunStart — the course manager", "now" if r == "R4" else ("done" if ri >= 5 else "next"),
            mss(o.get("waiting_s")) if r == "R4" else (f"run {o.get('run_id')}" if ri >= 5 else "—"),
            ("READY · waiting for the course manager" + (" · the drone flies on RunStart" if uav else "")
             + (" · the boat starts by itself (3-2-1)" if prof == "USV only" and mi.get("auto") else "")) if r == "R4" else "",
            f"no RunStart after {mss(o.get('stall_s'))} — ask the course manager, or redeclare" if r == "R4" and o.get("stalled") else "")
        running = ri == 5
        # 6 the drone (UAV runs): the touchdown sequence
        task_state = str(st.get("task_state") or "")
        if uav:
            d = self._drone(s, o)
            boat_waits = prof == "UAV + USV" and (task_state in ("", "WAIT_START", "WAIT_UAV_MAP") or not v == "AUTO_TASK")
            add("the drone — fly, map, land (touchdown = disarmed)", ("done" if d.get("go") else "now") if running else ("done" if r == "R6" else "next"),
                {"wait_flight": "on the pad", "flying": "FLYING", "rtl": "RTL", "map_quiet": "settling", "touchdown": "TOUCHDOWN ✓"}.get(d.get("step"), "—"),
                self._seq_line(d) + "\n" + str(d.get("text") or "") +
                ("\nthe boat goes by itself at TOUCHDOWN (it needs a flashing blue entry in the drone's map)" if prof == "UAV + USV" else
                 "\nthe boat holds in AUTO · LOITER the whole run"),
                buttons=[btn(s, "uav scan done", "▶ BOAT GO NOW (override)", "danger",
                             "Release the boat now, before the touchdown is confirmed? (uav scan done)")] if (running and boat_waits and prof == "UAV + USV") else [])
        # 7 the boat's mission (USV only, UAV + USV)
        if prof != "UAV only · boat holds":
            run = v in ("AUTO_TASK", "AUTO_SAFE_HOLD")
            add(f"the boat — {mi.get('code') or 'no mission'}", "now" if (run or (running and mi.get("code"))) else ("done" if r == "R6" else "next"),
                task_state or ("armed" if mi.get("code") else "—"),
                (f"{st.get('task_name') or ''} · {task_state}" + self._micro_text(s) if run else
                 (f"mission {mi.get('code')} · starts by itself" + (" after the touchdown" if uav else " at RunStart (3-2-1)") if mi.get("auto")
                  else "no auto start: ▶ START")) if mi.get("code") else "no mission armed - the launcher's preset preselects it",
                "AUTO_SAFE_HOLD - see the LOG" if v == "AUTO_SAFE_HOLD" else "",
                buttons=([btn(s, "hold", "❚❚ HOLD", "act"), btn(s, "abort", "■ ABORT", "danger")] if run
                         else [btn(s, "start_task", "▶ START (override)")]) if running else [])
        # 8 end
        add("END RUN", "done" if r == "R6" else ("now" if running else "next"), "ended" if r == "R6" else "—",
            "the judges' clock stops when the last declared vehicle leaves AUTO" if running else "",
            buttons=[btn(s, "end_run", "END RUN", "act", END_CONFIRM)] if running else [])   # v0.1.92: asks first (ends the attempt)
        for stp in steps:
            if stp.state == "done":
                self.done_t.setdefault((prof, stp.n), time.strftime("%H:%M:%S", time.localtime(self.clock())))
                stp.done_at = self.done_t[(prof, stp.n)]
            else:
                self.done_t.pop((prof, stp.n), None)
        return steps

    # ---- v0.1.89: the run banner (the corner pop-up + the always-on-top window) ---------------------------------------------
    def _banner(self, s, st, o, fv) -> Optional[dict]:
        if not s:
            return None
        r = o.get("r") or "R0"
        if r not in ("R5", "R6"):
            return None
        prof = self._profile(o)
        v = fv.get("state") or st.get("state") or "?"
        ts = fv.get("task_state") or st.get("task_state") or ""
        tn = fv.get("task_name") or st.get("task_name") or ""
        if r == "R6":
            return {"title": "RUN ENDED", "stage": f"run {o.get('run_id') or ''}".strip(), "level": "idle"}
        if prof != "USV only" and (prof == "UAV only · boat holds" or ts in ("", "WAIT_START", "WAIT_UAV_MAP") or v != "AUTO_TASK"):
            d = self._drone(s, o)
            stage = "UAV · " + {"wait_flight": "TAKING OFF", "flying": "FLYING · MAPPING", "rtl": "RETURNING (RTL)",
                                "map_quiet": "LANDED · MAP SETTLING", "touchdown": "TOUCHDOWN ✓"}.get(d.get("step"), "?")
            if prof == "UAV + USV":
                stage += "  ·  boat waits" if not d.get("go") else "  ·  BOAT GOES"
            nxt = "the boat at TOUCHDOWN" if prof == "UAV + USV" else ""
        else:
            stage = f"USV · {tn} · {ts}" if tn else f"USV · {v}"
            nxt = ""
        level = "warn" if v == "AUTO_SAFE_HOLD" else "ok"
        b = {"title": f"RUN STARTED · run {o.get('run_id') or ''}".strip(), "stage": stage, "level": level}
        # v0.1.92 (Dmitrii: "not only pixhawk mode / status, but also the micro stage of the mission and which might be next")
        mode = fv.get("mode") or st.get("mode") or "?"
        armed = fv.get("armed", st.get("armed"))
        b["fcu"] = f"FCU {mode} · {'ARMED' if armed is True else ('disarmed' if armed is False else 'armed ?')} · {v}"
        mc = micro_of(s) if v in ("AUTO_TASK", "AUTO_SAFE_HOLD") else None     # a task's micro-stage only while a task runs (never a stale one)
        b["why"], b["next"] = "", (f"next: {nxt}" if nxt else "")
        if mc and stage.startswith("USV"):
            b["stage"] = "USV · " + micro_stage(mc)
            b["why"] = f"— {mc.get('why')}" if mc.get("why") else ""
        if mc and mc.get("next"):
            b["next"] = f"next: {mc.get('next')}"
        mods = (mc or {}).get("mods") or (mods_line(o.get("mission")) if self._mods_apply(prof, o.get("mission") or {}) else "")
        b["mods"] = str(mods) if not mods or str(mods).startswith("mods") else f"mods: {mods}"
        b["drone"] = drone_map_line(s.get("uav_map")) if prof != "USV only" or (s.get("uav_map") or {}).get("ingest_off") else ""   # v0.1.93.2
        return b

    # ---- v0.1.92: the micro-stage on the boat's step, the modifiers subpanel ------------------------------------------------------
    @staticmethod
    def _mods_apply(prof: str, mi: dict) -> bool:
        """The modifiers are T1's (m1t1 / m1t2) and only when the boat runs it (not "UAV only · boat holds")."""
        return str((mi or {}).get("code") or "").startswith("m1t") and prof != "UAV only · boat holds"

    @staticmethod
    def _micro_text(s: dict) -> str:
        mc = micro_of(s)
        if not mc:
            return ""
        return ("\n" + micro_stage(mc) + (f" — {mc.get('why')}" if mc.get("why") else "")
                + (f"\nnext: {mc.get('next')}" if mc.get("next") else ""))

    def _mods(self, s, st, o) -> Optional[dict]:
        if not s:
            return None
        mi = o.get("mission") or {}
        prof = self.profile or self._profile(o)
        if not self._mods_apply(prof, mi):
            return None
        v = mods_values(mi)
        r = o.get("r") or "R0"
        try:
            ri = int(r[1:])
        except ValueError:
            ri = 0
        can_decl = r in ("R2", "R6") or (r == "R4" and bool(o.get("stalled")))
        frozen = str(st.get("state") or "").startswith("AUTO") or (ri >= 3 and not can_decl)
        why = MOD_FROZEN if frozen else ""

        def mb(key, label, on, **change):
            word = mission_word(mi, **change)
            conf = (CONFIRM_ANY if change.get("lucky_pairs") == "any" else CONFIRM_LUCKY if change.get("lucky") is True
                    else CONFIRM_SQUEEZE if change.get("squeeze") is True else "")
            if conf:                                                    # v0.1.93.1: the mission as it will be re-armed
                after = dict(mi, lucky=change.get("lucky", v["lucky"]), mods={**{k: v[k] for k in MOD_KEYS}, **{k: x for k, x in change.items() if k in MOD_KEYS}})
                conf += f"\n\nRe-arm {mi.get('code') or 'the mission'} as:\n{mods_line(after)}"
            return Btn(word=word, label=label, ok=not frozen, why=why, kind="on" if on else "", confirm=conf, key=key)
        bs = [mb("mod-start", "start: advance 2 m + 360°" if v["start_mode"] == "advance360" else "start: STRAIGHT", v["start_mode"] != "advance360",
                 start_mode="straight" if v["start_mode"] == "advance360" else "advance360"),
              mb("mod-look", "look-lock ON" if v["look_lock"] else "look-lock OFF", not v["look_lock"], look_lock=not v["look_lock"]),
              (mb("mod-lucky", "LUCKY ON", True, lucky=False, lucky_pairs="one_known") if v["lucky"]      # v0.1.93.1: off also drops ANY
               else mb("mod-lucky", "lucky off", False, lucky=True)),
              (mb("mod-any", "lucky pairs: ANY", True, lucky_pairs="one_known") if v["lucky_pairs"] == "any"         # v0.1.93.1
               else mb("mod-any", "lucky pairs: one known", False, lucky=True, lucky_pairs="any")),
              mb("mod-squeeze", "SQUEEZE ON" if v["squeeze"] else "squeeze off", v["squeeze"], squeeze=not v["squeeze"]),
              mb("mod-budget", f"budget {float(v['scout_entry_s']):.0f} s", float(v["scout_entry_s"]) != 420,
                 scout_entry_s=300 if float(v["scout_entry_s"]) == 420 else 420),
              mb("mod-exit", "exit-tangent ON" if v["circle_exit_tangent"] else "exit-tangent off", v["circle_exit_tangent"],
                 circle_exit_tangent=not v["circle_exit_tangent"])]
        return {"line": mods_line(mi), "fixed": MOD_FIXED, "frozen": frozen, "why": why, "buttons": bs}

    # ---- the tiles (K4) ------------------------------------------------------------------------------------------------------
    def _usv_words(self, s, st, o, fv) -> Dict[str, str]:
        v = fv.get("state") or st.get("state") or "?"
        sk = st.get("station_keep")
        f = o.get("usv_flow") or {}
        task = fv.get("task_name") or st.get("task_name")
        stage = fv.get("task_state") or st.get("task_state")
        return {"state": f"{v}" + (f" · {'LOITER' if sk == 'loiter' else 'HOLD'}" if v == "AUTO_HOLD" else ""),
                "task": f"{task} · {stage}" if task else "no task",
                "meaning": str(s.get("stage_meaning") or ""),
                "station_keep": f"in force: {str(sk).upper()}" if sk else "not published",
                "flow": ("declared" if f.get("declared") else ("ready" if f.get("ready") else ("heartbeat on" if f.get("heartbeat") else "silent")))
                        + f" · heartbeat {'on' if f.get('heartbeat') else 'off'} · pilot {'ready' if f.get('ready') else 'not ready'}",
                "sog": "—" if fv.get("sog_mps") is None else f"{float(fv['sog_mps']):.2f} m/s",
                "hdg": "—" if fv.get("hdg_deg") is None else f"{float(fv['hdg_deg']):05.1f}°"}

    def _usv_buttons(self, s, st, o) -> List[Btn]:
        f = o.get("usv_flow") or {}
        sk = st.get("station_keep")
        v = st.get("state") or "?"
        auto = v.startswith("AUTO")
        return [btn(s, "report off usv" if f.get("heartbeat") else "report on usv", "✓ HEARTBEAT ON" if f.get("heartbeat") else "▶ HEARTBEAT USV",
                    "on" if f.get("heartbeat") else "", "Stop the USV's heartbeat? RoboCommand stops seeing the boat." if f.get("heartbeat") else "",
                    key="usv-hb"),
                # v0.1.90.5: no PILOT READY button (ready by itself)
                btn(s, "disable_auto" if auto else "enable_auto", "DISABLE AUTO" if auto else "ENABLE AUTO", "" if auto else "go", key="usv-auto"),
                btn(s, "station_keep hold", "HOLD", "on" if sk == "hold" else "", key="usv-sk-hold"),
                btn(s, "station_keep loiter", "LOITER", "on" if sk == "loiter" else "", key="usv-sk-loiter"),
                btn(s, "start_task", "▶ START TASK", "go", key="usv-start"),
                btn(s, "hold", "❚❚ HOLD", key="usv-hold"), btn(s, "resume", "▶ RESUME", key="usv-resume"),
                btn(s, "abort", "■ ABORT", "danger", key="usv-abort"),
                btn(s, "fcu_mode MANUAL", "MANUAL", "on" if st.get("mode") == "MANUAL" else "", key="fcu-manual"),
                btn(s, "fcu_mode HOLD", "HOLD", "on" if st.get("mode") == "HOLD" else "", key="fcu-hold"),
                btn(s, "arm", "ARM", "", ARM_CONFIRM, key="fcu-arm"), btn(s, "disarm", "DISARM", key="fcu-disarm"),
                Btn("stop_reset", "■ STOP & RESET", kind="danger", confirm=STOP_CONFIRM, key="stop-reset")]

    def _uav_words(self, o) -> Dict[str, str]:
        u = o.get("uav") or {}
        if not u.get("configured"):
            return {"state": "no UAV configured", "declared": "—", "pilot": "—", "ident": "—", "heartbeat": "—"}
        return {"heartbeat": ("on · " if (u.get("report_on") or u.get("heartbeats_on")) else "off · ") +
                             ("telemetry " + (f"{u['telemetry_age_s']:.1f} s" if isinstance(u.get("telemetry_age_s"), (int, float)) else "none (the UAV adapter / their listener)")),
                "state": "declared in this run" if u.get("declared") else ("in the next declaration" if u.get("declare") else "off"),
                "declared": "yes" if u.get("declared") else "no", "pilot": "✓ ready" if u.get("pilot_ready") else "waiting for the pilot",
                "ident": f"{u.get('vehicle_id') or '?'} · {u.get('registration') or '?'} · pilot {u.get('pilot_callsign') or '?'}"}

    def _uav_buttons(self, s, o) -> List[Btn]:
        u = o.get("uav") or {}
        on = bool(u.get("declare") or u.get("declared"))
        hb = bool(u.get("report_on") or u.get("heartbeats_on"))
        return [btn(s, "report off uav" if hb else "report on uav", "✓ HEARTBEAT ON" if hb else "▶ HEARTBEAT UAV", "on" if hb else "",
                    "Stop the UAV's heartbeat? RoboCommand stops seeing the drone." if hb else "", key="uav-hb"),   # 2026-11-11: both vehicles on the RC page
                btn(s, "declare uav off" if on else "declare uav on", "✓ UAV DECLARED" if on else "DECLARE UAV", "on" if on else "",
                    "declare uav off: the drone leaves the NEXT declaration (this run's declaration is already sent)." if u.get("declared") else "",
                    key="uav-decl"),
                # v0.1.90.5: no PILOT READY button (the OCS sends UAV_READY by itself)
                btn(s, "uav scan done", "uav scan done", key="uav-scan")]

    def _missions(self, s, st) -> List[Tuple[str, str]]:
        fake = (st.get("fcu_link") == "fake") or ((s.get("flavour") or {}).get("fcu") == "fake")
        out = []
        for m in (self.meta or {}).get("missions") or []:
            if m.get("hidden"):                                 # B v0.1.86 (WP42): the FSM's test harness, out of every picker
                continue
            code = m.get("code")
            send = code + "sim" if (fake and m.get("sim") and code != "goto") else code
            out.append((send, f"{send} · {m.get('title', '')}" + (" (planned)" if m.get("status") == "planned" else "")))
        return out

    # ---- the LOG tab ---------------------------------------------------------------------------------------------------------
    def _log(self, s) -> List[LogLine]:
        return [LogLine(float(x.get("t") or 0), str(x.get("kind") or ""), str(x.get("text") or ""), str(x.get("sev") or ""),
                        log_group(x.get("kind"), x.get("text"))) for x in (s.get("log") or [])[-LOG_N:]]

    # ---- the map layer (K6 draws it) ------------------------------------------------------------------------------------------
    def _map(self, s, st, fv) -> MapLayer:
        m = MapLayer()
        if fv.get("lat") is not None and fv.get("lon") is not None:          # v0.1.83.5 (B): the 5 Hz position
            m.pose = (float(fv["lat"]), float(fv["lon"]))
        elif st.get("lat") is not None and st.get("lon") is not None:
            m.pose = (float(st["lat"]), float(st["lon"]))
        hd = fv.get("hdg_deg", st.get("hdg_deg"))
        m.hdg_deg = None if hd is None else float(hd)
        c = self.course or {}
        rec = c.get("received") or {}
        if rec.get("boundary"):
            m.polygon, m.polygon_src = [tuple(p[:2]) for p in rec["boundary"]], "received"
        else:
            d = c.get("default")
            pts = (d.get("boundary") if isinstance(d, dict) else d) or []
            if pts:
                m.polygon, m.polygon_src = [tuple(p[:2]) for p in pts], "default"
        m.geofence = [tuple(p[:2]) for p in (c.get("geofence") or []) if isinstance(p, (list, tuple)) and len(p) >= 2]
        m.points = list(s.get("points") or [])
        for ln in s.get("log") or []:
            t = str(ln.get("text") or "")
            if t.startswith("scout: ") and " logged " in t:
                try:
                    bid, rest = t[7:].split(" logged ", 1)
                    lat, lon = rest.split()[:2]
                    m.visited.append({"id": bid, "lat": float(lat), "lon": float(lon)})
                except ValueError:
                    pass
        return m

    # ---- the state transitions the operator must see without a click (K5) ----------------------------------------------------
    def _transitions(self, s, st, o, rec: Rec, link: str) -> List[Tuple[str, str]]:
        cur = {"rec": rec.on, "r": o.get("r"), "v": st.get("state"), "judge": bool((s.get("judge") or {}).get("connected")),
               "guard": str(((st.get("guard") or {}).get("chip")) or "").lower(), "run_id": o.get("run_id")}
        p, self._prev = self._prev, cur
        if p is None:
            return []
        ev: List[Tuple[str, str]] = []
        if cur["rec"] != p["rec"]:
            ev.append(("ok" if cur["rec"] else "warn", ("REC on · " + (rec.file or "logging")) if cur["rec"] else "REC off - logging stopped"))
        if cur["r"] == "R5" and p["r"] != "R5":
            ev.append(("ok", f"RunStart received · run {cur['run_id']}"))
        if cur["v"] == "AUTO_SAFE_HOLD" and p["v"] != "AUTO_SAFE_HOLD":
            ev.append(("warn", "AUTO_SAFE_HOLD entered" + (f" · {st.get('hold_reason')}" if st.get("hold_reason") else "")))
        if cur["judge"] != p["judge"]:
            ev.append(("ok", "judge back") if cur["judge"] else ("bad", "judge lost"))
        if cur["guard"] == "stop" and p["guard"] != "stop":
            rng = (st.get("guard") or {}).get("range_m")
            ev.append(("bad", "GUARD STOP" + (f" · {float(rng):.1f} m" if rng is not None else "")))
        return ev


@dataclass
class VehicleCard:
    """The VEHICLES tab (Dmitrii 2026-11-11: "the important IMU, compass, heartbeat status per each vehicle")."""
    vid: str
    kind: str                                          # USV | UAV
    link: Chip
    hdg: Optional[float] = None
    roll: Optional[float] = None
    pitch: Optional[float] = None
    rows: List[Tuple[str, str, str]] = field(default_factory=list)      # (label, text, level)


def _f(x, fmt="{:.1f}", none="—"):
    return none if not isinstance(x, (int, float)) else fmt.format(x)


def _hb_rows(s: dict, vid: str) -> List[Tuple[str, str, str]]:
    o = s.get("ocs") or {}
    rep = o.get("reporting") or {}
    hb = (rep.get("hb") or {}).get(vid)
    j = (s.get("judge") or {})
    seen = (j.get("heartbeat_age") or {}).get(vid)
    if hb and not hb.get("stopped"):
        age = hb.get("age_s")
        lvl = "ok" if isinstance(age, (int, float)) and age < 2 else "warn"
        sent = (f"sending · {str(hb.get('state') or '?').replace('STATE_', '')} · #{hb.get('n')} · last {_f(age)} s", lvl)
    else:
        sent = ("not sending (▶ HEARTBEAT)" if vid else "—", "warn")
    jc = j.get("connected")
    judge = (f"the judge saw it {_f(seen)} s ago" if isinstance(seen, (int, float)) else
             ("judge link up, nothing from this vehicle seen" if jc else "judge link DOWN"),
             "ok" if isinstance(seen, (int, float)) and seen < 3 else ("warn" if jc else "bad"))
    return [("HEARTBEAT", *sent), ("JUDGE", *judge)]


def vehicle_cards(s: Optional[dict], fast: Optional[dict] = None) -> List[VehicleCard]:
    """One card per vehicle from /state.json (+ /fast.json for the USV's 5 Hz attitude). Never fabricated: a missing value is —."""
    s = s or {}
    o, st = s.get("ocs") or {}, s.get("status") or {}
    fv = (fast or {}).get("v") or {}
    fcu = (s.get("local") or {}).get("fcu") or {}
    h = st.get("health") or {}
    cards: List[VehicleCard] = []
    usv_ids = [v for v in (o.get("vehicles") or {}) if v != (o.get("uav") or {}).get("vehicle_id")] or ["USV1"]
    for vid in usv_ids[:1]:
        sa = s.get("status_age")
        link = Chip(f"status {_f(sa)} s" + ("" if s.get("bridge") else " · bridge DOWN"),
                    "bad" if (sa is None or sa > 5 or not s.get("bridge")) else ("warn" if sa > 2 else "ok"))
        g = lambda k: fv.get(k, st.get(k))
        roll, pitch, yr, hdg = g("roll_deg"), g("pitch_deg"), g("yaw_rate_dps"), g("hdg_deg")
        tilt = max(abs(roll or 0), abs(pitch or 0))
        fix = fcu.get("gps_fix")
        fixw = {0: "no GPS", 1: "no fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK float", 6: "RTK fixed"}.get(fix, "—" if fix is None else str(fix))
        rows = [("FCU", f"{g('mode') or '?'} · {'ARMED' if g('armed') else 'disarmed'} · FCU {_f(h.get('fcu_age_s'), '{:.2f}')} s"
                 + (f" · batt {_f(fcu.get('batt_v'))} V {fcu.get('batt_pct')}%" if fcu.get("batt_v") is not None else ""),
                 "bad" if (h.get("fcu_age_s") or 0) > 2 else ("warn" if g("armed") else "")),
                ("IMU", f"roll {_f(roll)}° · pitch {_f(pitch)}° · yaw rate {_f(yr)}°/s", "bad" if tilt > 25 else ("warn" if tilt > 12 else "ok")),
                ("COMPASS", f"heading {_f(hdg, '{:05.1f}')}° · COG {_f(fcu.get('cog_deg'), '{:05.1f}')}°", "" if hdg is not None else "warn"),
                ("SPEED", f"{_f(g('sog_mps'), '{:.2f}')} m/s", ""),
                ("GPS", f"{fixw} · {fcu.get('sats', '—')} sats · HDOP {_f(fcu.get('hdop'))} · EKF {'ok' if fcu.get('ekf_pos_ok') else 'NOT ok' if fcu else '—'}",
                 "ok" if (fix or 0) >= 3 and fcu.get("ekf_pos_ok") else "bad" if fcu else "warn"),
                ("RC", ("KILL" if st.get("kill") else "E-STOP" if st.get("estop") else ("ok" if h.get("rc_ok") else "LOST")) +
                 (f" · RC6 holds MANUAL" if (st.get("rc6") or {}).get("holds") else ""),
                 "bad" if (st.get("kill") or st.get("estop") or h.get("rc_ok") is False) else "ok")]
        tk_ = h.get("tick_ms") or {}                         # B v0.1.86.4: the mission manager's tick (60 s ring)
        if tk_:
            p95, mx, slow = tk_.get("p95"), tk_.get("max"), tk_.get("slow_per_min") or 0
            lvl = ("bad" if (isinstance(p95, (int, float)) and p95 > 150) or (isinstance(mx, (int, float)) and mx > 500)
                   else "warn" if (isinstance(p95, (int, float)) and p95 > 100) or slow else "ok")
            rows.append(("LOOP", f"tick p95 {_f(p95, '{:.0f}')} ms · max {_f(mx, '{:.0f}')} ms · {slow} slow / min", lvl))
        else:
            rows.append(("LOOP", "— (the boat's tick instrument arrives with v0.1.86.4 on the NX)", ""))
        gl = h.get("guard_ms") or {}                         # B v0.1.86.8: the ZED guard's camera → boat latency (target p95 < 150 ms)
        if gl and gl.get("pipe_ms_p95") is not None:
            p95 = gl.get("pipe_ms_p95")
            rows.append(("GUARD LAG", f"camera → publish p50 {_f(gl.get('pipe_ms_p50'), '{:.0f}')} · p95 {_f(p95, '{:.0f}')} ms (target < 150)"
                         f" · network {_f(gl.get('net_ms_p50'), '{:.0f}')} ms (incl. the clock offset) · {gl.get('n', 0)} msgs",
                         "bad" if p95 > 300 else ("warn" if p95 > 150 else "ok")))
        else:
            rows.append(("GUARD LAG", "— (needs v0.1.86.8 on the AGX + the NX)", ""))
        rows += _hb_rows(s, vid)
        nx = (s.get("hosts") or {}).get("nx") or {}
        if nx:
            rows.append(("HOST", f"NX {_f(nx.get('temp_c'), '{:.0f}')} °C · load {_f(nx.get('cpu_load1'))}" + (" · simulated" if nx.get("simulated") else ""),
                         "warn" if isinstance(nx.get("temp_c"), (int, float)) and nx["temp_c"] > 75 else ""))
        cards.append(VehicleCard(vid, "USV", link, hdg, roll, pitch, rows))
    u, uv = o.get("uav") or {}, s.get("uav") or {}
    if u.get("configured") or uv.get("state") not in (None, "none"):
        vid = u.get("vehicle_id") or "UAV1"
        if uv.get("state") in (None, "none"):
            link = Chip("no adapter (the launcher's UAV field)", "bad")
        else:
            link = Chip(f"adapter {_f(uv.get('age_s'))} s · {uv.get('link', '?')}", "ok" if uv.get("link") == "up" else "bad")
        uh = uv.get("health") or {}
        ta = uh.get("telemetry_age_s", u.get("telemetry_age_s"))
        roll, pitch, hdg = uv.get("roll_deg"), uv.get("pitch_deg"), uv.get("hdg_deg")
        rows = [("FCU", f"{uv.get('mode') or '?'} · {'ARMED' if uv.get('armed') else 'disarmed' if uv.get('armed') is False else '?'} · "
                        f"{str(uv.get('flight_phase') or '?').replace('FLIGHT_PHASE_', '').lower()}", "warn" if uv.get("armed") else ""),
                ("IMU", f"roll {_f(roll)}° · pitch {_f(pitch)}°" + ("" if roll is not None else " (not in /state.json yet)"), "ok" if roll is not None else ""),
                ("COMPASS", f"heading {_f(hdg, '{:05.1f}')}°", "" if hdg is not None else "warn"),
                ("ALTITUDE", f"{_f(uv.get('altitude_hae_m'))} m HAE · {_f(uv.get('sog_mps'), '{:.1f}')} m/s", ""),
                ("TELEMETRY", f"{_f(ta)} s old · {uh.get('telemetry_n', '—')} msgs" if ta is not None else "none (their listener / the adapter)",
                 "ok" if isinstance(ta, (int, float)) and ta < 3 else "bad"),
                ("FENCE", "ok" if uh.get("fence_ok") else ("NOT ok" if uv.get("fence") else "—"), "ok" if uh.get("fence_ok") else "warn"),
                ("PILOT", "ready ✓" if u.get("pilot_ready") else "not ready", "ok" if u.get("pilot_ready") else "")]
        pv = o.get("declaration_preview") or {}
        ref = pv.get("uav_refusal")
        geo = (pv.get("body") or {}).get("uav_geofence") or []
        rows.append(("DECLARE", ("✗ " + str(ref)) if ref else (f"✓ fence {max(0, len(geo) - 1)} pts from the drone's FCU" if geo else "—"),
                     "bad" if ref else ("ok" if geo else "")))            # 2026-11-11: why DECLARE UAV would refuse (the FCU's fence)
        rows += _hb_rows(s, vid)
        cards.append(VehicleCard(vid, "UAV", link, hdg, roll, pitch, rows))
    return cards


CLOCK_MAX_S = 0.2


@dataclass
class LogSession:
    """The one-press logging (Dmitrii 2026-11-11: everything together, the clocks checked first)."""
    running: bool
    label: str
    button: str
    clocks: str
    clocks_level: str
    clock_problem: str                                  # '' = fine; else the confirm text before a start
    auto: bool = False                                  # B v0.1.86: started by itself at DECLARE
    t0: str = ""


def logging_view(s: Optional[dict]) -> LogSession:
    """From state.logging.session + state.clocks (B's POST /logging contract); until they exist: not running, clocks not measured."""
    s = s or {}
    se = ((s.get("logging") or {}).get("session")) or {}
    running = bool(se.get("running"))
    parts = se.get("parts") or {}
    label = str(se.get("label") or "")
    if running and parts:
        label += "  (" + " · ".join(f"{k} {'✓' if (v or {}).get('ok') else '✗'}" for k, v in parts.items()) + ")"
    cl = s.get("clocks")
    words, bad = [], []
    if isinstance(cl, dict) and cl:
        for host, c in sorted(cl.items(), key=lambda kv: ({"nx": 0, "agx": 1}.get(kv[0], 2), kv[0])):
            c = c or {}
            off = c.get("offset_s")
            gate = host in ("nx", "agx")                   # B v0.1.84.5: the start checks the NX and the AGX; others are shown only
            if not isinstance(off, (int, float)):
                words.append(f"{host.upper()} ? (no recent beat)")
                if gate:
                    bad.append(f"{host.upper()} unknown")
            else:
                ok = bool(c.get("ok")) if "ok" in c else abs(off) <= CLOCK_MAX_S      # B's ok = |offset| <= 0.2 s and a beat < 10 s old
                words.append(f"{host.upper()} {off:+.2f} s {'✓' if ok else '✗'}")
                if not ok and gate:
                    bad.append(f"{host.upper()} {off:+.2f} s vs the Mac" + ("" if abs(off) > CLOCK_MAX_S else " (beat stale)"))
        text, lvl = "clocks: " + " · ".join(words), ("bad" if bad else "ok")
        problem = ("Clock offset above 0.2 s: " + ", ".join(bad) + ". The files will not line up by time. Start logging anyway?") if bad else ""
    else:
        text, lvl, problem = "clocks: not measured (the OCS has no state.clocks yet)", "warn", ""
    auto = bool(se.get("auto"))
    if running and auto:
        label += "  · automatic at DECLARE"
    return LogSession(running, label, "■ STOP LOGGING" if running else "● START LOGGING", text, lvl, problem, auto, str(se.get("t0") or ""))


@dataclass
class CmdInfo:
    """The CMD tab's catalogue row (2026-11-11: "all available commands with tooltips and modificators")."""
    group: str
    word: str
    usage: str = ""
    does: str = ""
    example: str = ""
    action: str = ""
    modifiers: List[Tuple[str, str]] = field(default_factory=list)     # (the flag / positional with its choices, its help)
    rules: str = ""                                                    # the playbook: where it is allowed, what it needs, what follows


def cli_spec() -> Dict[str, dict]:
    """ocs_cli's own parser, introspected: {word: {"help": ..., "modifiers": [(spec, help)]}} - the truth about the modifiers (the
    connection options every subcommand shares are left out). {} when ocs_cli cannot be imported."""
    import argparse
    try:
        from .. import ocs_cli
        ap = ocs_cli.build_parser()
    except Exception:
        return {}
    sub = next((a for a in ap._actions if isinstance(a, argparse._SubParsersAction)), None)
    if sub is None:
        return {}
    helps = {c.dest: (c.help or "") for c in sub._choices_actions}
    parsers = dict(sub.choices)
    common = None
    for p in parsers.values():
        d = {a.dest for a in p._actions}
        common = d if common is None else common & d
    out: Dict[str, dict] = {}
    for name, p in parsers.items():
        mods = []
        for a in p._actions:
            if a.dest in (common or set()) or isinstance(a, argparse._HelpAction):
                continue
            ch = "|".join(str(c) for c in a.choices) if a.choices else ""
            if a.option_strings:
                flag = a.option_strings[-1]
                spec = flag if a.nargs == 0 else f"{flag} {ch or (a.metavar or a.dest).upper()}"
            else:
                spec = ch or a.dest
                spec = f"[{spec}]" if a.nargs == "?" else spec
            mods.append((spec, (a.help or "").replace("%%", "%")))
        out[name] = {"help": helps.get(name, ""), "modifiers": mods}
    return out


def command_catalogue(meta: Optional[dict], spec: Optional[Dict[str, dict]] = None) -> List[CmdInfo]:
    """Every word the console can send: /meta.json's groups + usage + examples + the playbook entry, ocs_cli's modifiers; the words
    /cmd accepts without a usage line yet (report, station_keep, usv, uav, ...) come from the parser alone (group OTHER)."""
    meta = meta or {}
    spec = cli_spec() if spec is None else spec
    usage = {u.get("word"): u for u in meta.get("usage") or []}
    ex, w2a, ent = meta.get("examples") or {}, meta.get("word_to_action") or {}, meta.get("entries") or {}
    seen, out = set(), []

    def info(group: str, w: str) -> CmdInfo:
        u = usage.get(w) or {}
        s = spec.get(w) or {}
        act = w2a.get(w) or ""
        e = ent.get(act) or {}
        rr = ", ".join(e.get("ocs_r") or []) if e else ""
        vv = "/".join(e.get("vehicle_v") or []) if e else ""
        rules = " · ".join(x for x in (f"run {rr}" if rr and rr != "*" else "", f"vehicle {vv}" if vv and vv != "*" else "",
                                       f"needs: {e['needs']}" if e.get("needs") else "", f"then: {e['then']}" if e.get("then") else "",
                                       "bench only" if e.get("bench_only") else "") if x)
        mods = list(s.get("modifiers") or [])
        use = " ".join(str(u.get("usage") or "").split()) or " ".join([w] + [m[0] for m in mods])   # no usage line yet: from the parser
        return CmdInfo(group, w, use, str(u.get("does") or s.get("help") or e.get("does") or ""),
                       str(ex.get(w) or w), act, mods, rules)
    for group, words in meta.get("groups") or []:
        for w in words:
            if w not in seen:
                seen.add(w)
                out.append(info(group, w))
    for w in list(usage) + sorted(spec):
        if w and w not in seen and w not in ("help", "tui", "watch", "missions", "quit"):
            seen.add(w)
            out.append(info("OTHER", w))
    return out


UAV_RUN_WORDS = ["station_keep loiter", "enable_auto", "report on usv", "usv ready on", "report on uav", "uav ready on",
                 "declare uav on", "declare"]


def uav_run_plan(s: Optional[dict]) -> Tuple[List[str], str]:
    """2026-11-11 (Dmitrii: "boat holds in auto good for now, make it quick"): a UAV run with the boat declared and holding in AUTO.
    (the words still to send, a refusal) - never ARM (the operator's press), never handover (typed on the boat); steps already true
    are skipped (AUTO, heartbeats, ready flags, the UAV in the declaration)."""
    s = s or {}
    st, o = s.get("status") or {}, s.get("ocs") or {}
    if s.get("ui_active"):
        return [], "usv_ui holds the FCU: type handover on the boat first"
    if st.get("armed") is not True:
        return [], "the FCU is disarmed: ARM first (it asks), then UAV RUN"
    f, u = o.get("usv_flow") or {}, o.get("uav") or {}
    v = str(st.get("state") or "")
    done = {"station_keep loiter": st.get("station_keep") == "loiter", "enable_auto": v.startswith("AUTO"),
            "report on usv": bool(f.get("heartbeat")), "usv ready on": bool(f.get("ready")), "report on uav": bool(u.get("report_on")),
            "uav ready on": bool(u.get("pilot_ready")), "declare uav on": bool(u.get("declare") or u.get("declared")),
            "declare": (o.get("r") in ("R3", "R4", "R5")) and not o.get("stalled")}
    return [w for w in UAV_RUN_WORDS if not done.get(w)], ""


def mission_line(word: str, code: str, options: List[dict], values: Dict[str, object], auto: bool = False) -> str:
    """The MISSION tab's line: `mission <code> [--flag value ...] [--auto]` (arm) or `start_task <code> ...` (start now). An empty
    field is left out (the profile's default); a check option is its flag alone; values are shell-quoted (ocs_web splits with shlex)."""
    import shlex
    parts = [word, code]
    for o in options:
        f = o.get("flag")
        v = values.get(f)
        if not f or v in (None, "", False):
            continue
        if o.get("check"):
            parts.append(f)
        else:
            parts += [f, shlex.quote(str(v).strip())]
    if auto and word == "mission":
        parts.append("--auto")
    return " ".join(parts)


def mission_stages(meta: dict, code: str, task_name: Optional[str], task_state: Optional[str]) -> List[Tuple[str, str, str]]:
    """[(name, meaning, now|'')] of a mission's stages from /meta.json; `now` on the boat's current stage of that task."""
    base = code[:-3] if code.endswith("sim") else code
    m = next((x for x in (meta or {}).get("missions") or [] if x.get("code") == base), None)
    if not m:
        return []
    live = task_name == m.get("task")
    return [(s.get("name", "?"), s.get("meaning", ""), "now" if live and s.get("name") == task_state else "") for s in m.get("stages") or []]


TITLES = ["handover — the boat's FCU to the mission manager", "crew — heartbeats on the RC page, pilots ready",
          "ENABLE AUTO — the boat in AUTO · LOITER", "DECLARE — the RunDeclaration", "RunStart — the course manager",
          "the boat — the mission", "END RUN"]
