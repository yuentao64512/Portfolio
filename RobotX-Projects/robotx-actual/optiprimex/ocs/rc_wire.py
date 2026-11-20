"""rc_wire: the OCS's judge dicts <-> RoboNation's RobotX 2026 protobuf messages (ROADMAP WP4; handbook 3.4).

The OCS app and the fake judge speak small dicts (`kind` + fields, see ocs/judge_client.FakeJudgeClient); this module is the
only place that knows the protobuf schema (`ocs/rc_pb`, robonation/robocommand @ 8fc09f2). Both directions are here so the
fake judge (`--wire protobuf`), rc-tui and the tests use the same translation as the OCS:

  OCS -> RC   RxRequest (RunDeclaration) on robocommand/robotx/<team>/request
              RxReport  (Heartbeat, SafePassage, Docking, Firefighting, IncidentAck, Readiness, ResourceDelivery)
                        on robocommand/robotx/<team>/<vehicle>/report
  RC -> OCS   RxCourse  (retained) on robocommand/robotx/course
              RxCommand (RunStart, AssistanceRequest, KeepOutZone, AllClear, MovingObjectAlert, ReadinessConfirm)
                        on robocommand/robotx/<team>/command
Pure functions; no I/O.
"""
from __future__ import annotations

import time
from typing import Any, Dict, List, Optional, Tuple

from .rc_pb import common_pb2 as C
from .rc_pb import rx_commands_pb2 as CMD
from .rc_pb import rx_common_pb2 as RX
from .rc_pb import rx_course_pb2 as CRS
from .rc_pb import rx_reports_pb2 as REP
from .rc_pb import rx_requests_pb2 as REQ

ROOT = "robocommand/robotx"


def t_course() -> str:
    return f"{ROOT}/course"


def t_report(team: str, vehicle: str) -> str:
    return f"{ROOT}/{team}/{vehicle}/report"


def t_request(team: str) -> str:
    return f"{ROOT}/{team}/request"


def t_command(team: str) -> str:
    return f"{ROOT}/{team}/command"


# ---- enums -----------------------------------------------------------------------------------------------------------
TASK_TO_RX = {"TASK_NONE": "TASK_NONE", "TASK_SAFE_PASSAGE": "TASK_SAFE_PASSAGE", "TASK_INFRASTRUCTURE_SURVEY": "TASK_INFRA_SURVEY_REPAIR",
              "TASK_INFRA_SURVEY_REPAIR": "TASK_INFRA_SURVEY_REPAIR", "TASK_COORDINATED_LOGISTICS": "TASK_COORDINATED_LOGISTICS",
              "TASK_DYNAMIC_INCIDENT": "TASK_DYNAMIC_INCIDENT"}
TASK_FROM_RX = {"TASK_INFRA_SURVEY_REPAIR": "TASK_INFRASTRUCTURE_SURVEY"}
DOMAIN_OF_TYPE = {"TYPE_USV": "surface", "TYPE_UUV": "surface", "TYPE_UAV": "air"}
TYPE_OF_DOMAIN = {"surface": "TYPE_USV", "air": "TYPE_UAV", "subsurface": "TYPE_UUV"}
BEACON = {("red", "flashing"): "BEACON_STATE_FLASHING_RED", ("green", "flashing"): "BEACON_STATE_FLASHING_GREEN",
          ("blue", "flashing"): "BEACON_STATE_FLASHING_BLUE", ("blue", "solid"): "BEACON_STATE_STEADY_BLUE"}
COLOR = {"red": "COLOR_RED", "green": "COLOR_GREEN", "blue": "COLOR_BLUE", "any": "COLOR_ANY"}


def _enum(enum_type, name: Optional[str], default: str) -> int:
    try:
        return enum_type.Value(str(name)) if name else enum_type.Value(default)
    except ValueError:
        return enum_type.Value(default)


def _name(enum_type, value: int) -> str:
    return enum_type.Name(value)


def beacon_of(colour: Optional[str], state: Optional[str]) -> str:
    if state in ("off",) or colour == "black":
        return "BEACON_STATE_OFF"
    return BEACON.get((str(colour), str(state)), "BEACON_STATE_UNKNOWN")


def _ll(msg, p) -> None:
    """Fill a LatLng from {lat, lon} or [lat, lon]."""
    if p is None:
        return
    lat, lon = (p.get("lat"), p.get("lon")) if isinstance(p, dict) else (p[0], p[1])
    if lat is None or lon is None:
        return
    msg.latitude, msg.longitude = float(lat), float(lon)


def _pos(msg) -> Dict[str, float]:
    return {"lat": msg.latitude, "lon": msg.longitude}


def _stamp(ts, t: Optional[float] = None) -> None:
    t = time.time() if t is None else float(t)
    ts.seconds, ts.nanos = int(t), int((t - int(t)) * 1e9)


def _t(ts) -> float:
    return ts.seconds + ts.nanos / 1e9


# ---- OCS -> RC: reports ------------------------------------------------------------------------------------------------
def encode_report(team: str, vehicle: str, seq: int, body: dict, t: Optional[float] = None) -> bytes:
    m = REP.RxReport(team_id=team, vehicle_id=vehicle, seq=int(seq))
    _stamp(m.sent_at, t)
    k = body.get("kind")
    if k == "heartbeat":
        h = m.heartbeat
        h.state = _enum(C.RobotState, body.get("state"), "STATE_UNKNOWN")
        if body.get("lat") is not None and body.get("lon") is not None:
            _ll(h.position, {"lat": body["lat"], "lon": body["lon"]})
        for f in ("spd_mps", "heading_deg", "roll_deg", "pitch_deg", "altitude_hae_m", "depth_m"):
            if body.get(f) is not None:
                setattr(h, f, float(body[f]))
        h.current_task = _enum(RX.RxTask, TASK_TO_RX.get(body.get("current_task") or "TASK_NONE", "TASK_NONE"), "TASK_NONE")
        h.vehicle_type = _enum(RX.VehicleType, "TYPE_" + str(body.get("vehicle_type") or "USV").upper().replace("TYPE_", ""), "TYPE_UNKNOWN")
        if str(body.get("vehicle_type") or "USV").upper().replace("TYPE_", "") == "UAV":     # v0.1.59 (A4): explicit presence -
            h.flight_phase = _enum(RX.FlightPhase, body.get("flight_phase"), "FLIGHT_PHASE_UNKNOWN")   # set only for a UAV
        if body.get("uav_registration"):
            h.uav_registration = str(body["uav_registration"])
    elif k == "safe_passage":
        sp = m.safe_passage
        _ll(sp.entry_position, body.get("entry_position"))
        _ll(sp.exit_position, body.get("exit_position"))
        for b in body.get("buoys") or []:
            d = sp.buoys.add()
            _ll(d.position, b)
            d.state = _enum(RX.BeaconState, b.get("beacon") or beacon_of(b.get("colour"), b.get("state")), "BEACON_STATE_UNKNOWN")
    elif k == "docking":
        m.docking.bay_id = int(body.get("bay") or 0)
    elif k == "firefighting":
        m.firefighting.window_id = int(body.get("window_id") or 0)    # 1 = upper, 2 = lower (RoboNation A10, 2026-11-04); 0 = unset
    elif k == "incident_ack":
        m.incident_ack.command_seq = int(body.get("command_seq") or 0)
    elif k == "readiness":
        m.readiness.command_seq = int(body.get("command_seq") or 0)
    elif k == "pb":                                # v0.1.64 (D13): a UAV report as RoboNation's field (validated by the adapter)
        from google.protobuf import json_format
        json_format.ParseDict({str(body.get("field")): body.get("pb") or {}}, m)
    elif k == "resource_request":
        r = m.resource_delivery
        r.task = _enum(RX.RxTask, TASK_TO_RX.get(body.get("task") or "", body.get("task")), "TASK_UNKNOWN")
        r.resource_color = _enum(RX.Color, COLOR.get(str(body.get("resource_color")).lower()), "COLOR_UNKNOWN")
        r.delivery_circle_color = _enum(RX.Color, COLOR.get(str(body.get("delivery_color")).lower()), "COLOR_UNKNOWN")
    else:
        raise ValueError(f"no RxReport body for kind {k!r}")
    return m.SerializeToString()


def decode_report(payload: bytes) -> dict:
    m = REP.RxReport.FromString(payload)
    which = m.WhichOneof("body")
    out: Dict[str, Any] = {"team_id": m.team_id, "vehicle_id": m.vehicle_id, "report_seq": m.seq, "sent_at": _t(m.sent_at)}
    if which == "heartbeat":
        h = m.heartbeat
        out.update(kind="heartbeat", state=_name(C.RobotState, h.state), lat=h.position.latitude if h.HasField("position") else None,
                   lon=h.position.longitude if h.HasField("position") else None, spd_mps=h.spd_mps,
                   heading_deg=h.heading_deg, roll_deg=h.roll_deg, pitch_deg=h.pitch_deg, altitude_hae_m=h.altitude_hae_m, depth_m=h.depth_m,
                   uav_registration=h.uav_registration,
                   current_task=TASK_FROM_RX.get(_name(RX.RxTask, h.current_task), _name(RX.RxTask, h.current_task)),
                   vehicle_type=_name(RX.VehicleType, h.vehicle_type).replace("TYPE_", ""), flight_phase=_name(RX.FlightPhase, h.flight_phase))
    elif which == "safe_passage":
        sp = m.safe_passage
        out.update(kind="safe_passage", entry_position=_pos(sp.entry_position) if sp.HasField("entry_position") else None,
                   exit_position=_pos(sp.exit_position) if sp.HasField("exit_position") else None,
                   buoys=[dict(_pos(b.position), beacon=_name(RX.BeaconState, b.state)) for b in sp.buoys])
    elif which == "docking":
        out.update(kind="docking", bay=m.docking.bay_id)
    elif which == "firefighting":
        out.update(kind="firefighting", window_id=m.firefighting.window_id)
    elif which == "incident_ack":
        out.update(kind="incident_ack", command_seq=m.incident_ack.command_seq)
    elif which == "readiness":
        out.update(kind="readiness", command_seq=m.readiness.command_seq)
    elif which == "resource_delivery":
        r = m.resource_delivery
        out.update(kind="resource_request", task=_name(RX.RxTask, r.task), resource_color=_name(RX.Color, r.resource_color),
                   delivery_color=_name(RX.Color, r.delivery_circle_color))
    else:
        out.update(kind=str(which))
    return out


# ---- OCS -> RC: requests ---------------------------------------------------------------------------------------------
TIER_KEYS = (("T1", "task1_tier"), ("T2", "task2_tier"), ("T3", "task3_tier"), ("T4", "task4_tier"))


def encode_request(team: str, seq: int, body: dict, t: Optional[float] = None) -> bytes:
    m = REQ.RxRequest(team_id=team, seq=int(seq))
    _stamp(m.sent_at, t)
    if body.get("kind") != "run_declaration":
        raise ValueError(f"no RxRequest body for kind {body.get('kind')!r}")
    d = m.run_declaration
    d.vehicle_ids.extend([str(v) for v in body.get("vehicle_ids") or []])
    tiers = body.get("tiers") or {}
    for k, f in TIER_KEYS:
        setattr(d, f, _enum(C.TaskTier, tiers.get(k) or "TIER_NONE", "TIER_NONE"))
    for p in body.get("uav_geofence") or []:
        _ll(d.uav_geofence.add(), p)
    if body.get("uav_geofence_max_altitude_hae_m") is not None:
        d.uav_geofence_max_altitude_hae_m = float(body["uav_geofence_max_altitude_hae_m"])
    if body.get("uav_pilot_callsign"):
        d.uav_pilot_callsign = str(body["uav_pilot_callsign"])
    return m.SerializeToString()


def decode_request(payload: bytes) -> dict:
    m = REQ.RxRequest.FromString(payload)
    d = m.run_declaration
    return {"kind": "run_declaration" if m.WhichOneof("body") == "run_declaration" else str(m.WhichOneof("body")),
            "team_id": m.team_id, "request_seq": m.seq, "sent_at": _t(m.sent_at), "vehicle_ids": list(d.vehicle_ids),
            "tiers": {k: _name(C.TaskTier, getattr(d, f)) for k, f in TIER_KEYS},
            "uav_geofence": [[p.latitude, p.longitude] for p in d.uav_geofence],
            "uav_geofence_max_altitude_hae_m": d.uav_geofence_max_altitude_hae_m, "uav_pilot_callsign": d.uav_pilot_callsign}


# ---- RC -> OCS: commands -----------------------------------------------------------------------------------------------
def encode_command(team: str, seq: int, body: dict, t: Optional[float] = None) -> bytes:
    m = CMD.RxCommand(team_id=team, seq=int(seq))
    _stamp(m.sent_at, t)
    if body.get("vehicle_id"):
        m.vehicle_id = str(body["vehicle_id"])
    k = body.get("kind")
    vt = _enum(RX.VehicleType, TYPE_OF_DOMAIN.get(body.get("domain") or "surface", "TYPE_USV"), "TYPE_USV")
    if k == "run_start":
        m.run_start.declaration_seq, m.run_start.run_id = int(body.get("declaration_seq") or 0), int(body.get("run_id") or 0)
    elif k == "assistance_request":
        _ll(m.assistance_request.position, body.get("position"))
        m.assistance_request.vehicle_type = vt
    elif k == "keep_out_zone":
        _ll(m.keep_out_zone.center, body.get("center") or body.get("position"))
        m.keep_out_zone.radius_m = float(body.get("radius_m") or 0.0)
        m.keep_out_zone.vehicle_type = vt
    elif k == "all_clear":
        m.all_clear.vehicle_type = vt
    elif k == "moving_object":
        mo = m.moving_object_alert
        _ll(mo.position, body.get("position"))
        mo.heading_deg = float(body.get("heading_deg") or 0.0)
        mo.speed_mps = float(body.get("speed_mps") or 0.0)
        mo.affected_vehicle_types.append(vt)
        mo.keep_out_radius_m = float(body.get("keep_out_radius_m") or 5.0)
    elif k == "readiness_confirm":
        m.readiness_confirm.report_seq = int(body.get("report_seq") or 0)
        m.readiness_confirm.vehicle_id = str(body.get("vehicle_id") or "")
    else:
        raise ValueError(f"no RxCommand body for kind {k!r}")
    return m.SerializeToString()


def decode_command(payload: bytes) -> dict:
    m = CMD.RxCommand.FromString(payload)
    which = m.WhichOneof("body")
    out: Dict[str, Any] = {"team_id": m.team_id, "command_seq": m.seq, "sent_at": _t(m.sent_at)}
    if m.vehicle_id:
        out["vehicle_id"] = m.vehicle_id
    if which == "run_start":
        out.update(kind="run_start", declaration_seq=m.run_start.declaration_seq, run_id=m.run_start.run_id)
    elif which == "assistance_request":
        a = m.assistance_request
        out.update(kind="assistance_request", position=_pos(a.position), domain=DOMAIN_OF_TYPE.get(_name(RX.VehicleType, a.vehicle_type), "surface"),
                   vehicle_type=_name(RX.VehicleType, a.vehicle_type))
    elif which == "keep_out_zone":
        z = m.keep_out_zone
        out.update(kind="keep_out_zone", center=_pos(z.center), position=_pos(z.center), radius_m=z.radius_m,
                   domain=DOMAIN_OF_TYPE.get(_name(RX.VehicleType, z.vehicle_type), "surface"))
    elif which == "all_clear":
        out.update(kind="all_clear", domain=DOMAIN_OF_TYPE.get(_name(RX.VehicleType, m.all_clear.vehicle_type), "surface"))
    elif which == "moving_object_alert":
        mo = m.moving_object_alert
        types = [_name(RX.VehicleType, v) for v in mo.affected_vehicle_types]
        out.update(kind="moving_object", position=_pos(mo.position), heading_deg=mo.heading_deg, speed_mps=mo.speed_mps,
                   keep_out_radius_m=mo.keep_out_radius_m, affected=types,
                   domain="all" if len({DOMAIN_OF_TYPE.get(t) for t in types}) > 1 else DOMAIN_OF_TYPE.get(types[0], "surface") if types else "surface")
    elif which == "readiness_confirm":
        out.update(kind="readiness_confirm", report_seq=m.readiness_confirm.report_seq, vehicle_id=m.readiness_confirm.vehicle_id or out.get("vehicle_id"))
    else:
        out.update(kind=str(which))
    return out


# ---- RC -> OCS: course -------------------------------------------------------------------------------------------------
def encode_course(body: dict, t: Optional[float] = None) -> bytes:
    m = CRS.RxCourse(course_id=str(body.get("course_id") or ""), pinger_freq_hz=int(body.get("pinger_hz") or 0))
    _stamp(m.sent_at, t)
    for key, rep in (("boundary", m.corners), ("uav_transit_area", m.uav_transit_area)):
        pts = [list(p) if not isinstance(p, dict) else [p["lat"], p["lon"]] for p in body.get(key) or []]
        if pts and pts[0] != pts[-1]:
            pts.append(pts[0])                     # handbook 3.4.9: the first and last points are the same
        for p in pts:
            _ll(rep.add(), p)
    return m.SerializeToString()


def decode_course(payload: bytes) -> dict:
    m = CRS.RxCourse.FromString(payload)
    return {"course_id": m.course_id, "pinger_hz": m.pinger_freq_hz, "boundary": [[p.latitude, p.longitude] for p in m.corners],
            "uav_transit_area": [[p.latitude, p.longitude] for p in m.uav_transit_area], "published_at": _t(m.sent_at)}


def pb_dict(kind: str, payload: bytes) -> Dict[str, Any]:
    """v0.1.64 (D12): an RxCourse / RxCommand as the protobuf dict the UAV team's bridge forwards to their Orin
    (json_format.MessageToDict, preserving_proto_field_name=True - their robocommand_bridge.to_dict)."""
    from google.protobuf import json_format
    msg = {"course": CRS.RxCourse, "command": CMD.RxCommand}[kind].FromString(payload)
    return json_format.MessageToDict(msg, preserving_proto_field_name=True)


DECODERS = {"course": decode_course, "report": decode_report, "request": decode_request, "command": decode_command}


def classify(topic: str, team: Optional[str] = None) -> Optional[Tuple[str, Optional[str]]]:
    """(kind, vehicle) of a robocommand/robotx topic: course | request | command | report (+ vehicle); None if not ours."""
    parts = topic.split("/")
    if parts[:2] != ["robocommand", "robotx"]:
        return None
    if parts[2:] == ["course"]:
        return ("course", None)
    if team is not None and len(parts) > 2 and parts[2] != team:
        return None
    if len(parts) == 4 and parts[3] in ("request", "command"):
        return (parts[3], None)
    if len(parts) == 5 and parts[4] == "report":
        return ("report", parts[3])
    return None


def decode(topic: str, payload: bytes, team: Optional[str] = None) -> Optional[dict]:
    c = classify(topic, team)
    if c is None:
        return None
    d = DECODERS[c[0]](payload)
    d["_topic_kind"] = c[0]
    if c[1]:
        d.setdefault("vehicle_id", c[1])
    return d
