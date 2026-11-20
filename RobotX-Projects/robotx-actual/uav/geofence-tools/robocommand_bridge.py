#!/usr/bin/env python3
"""
GCS-laptop bridge: receives RoboCommand messages (RobotX 2026) over MQTT and
forwards them to the Orin's geofence_udp_listener.py, over the same plain
UDP link geofence_udp_publisher.py uses.

RoboCommand only talks to the team's Operator Control Station (this
laptop), never to a vehicle - so this is how its commands reach the drone.
It subscribes to:
    robocommand/robotx/<team_id>/command   RxCommand  (RunStart, Task 4 ...)
    robocommand/robotx/course              RxCourse   (boundary + UAV transit area)
decodes them with RoboNation's generated Python code, and sends each one to
the listener as JSON. Decoding happens here, so the Orin doesn't need the
protobuf library.

It also publishes the UAV's Heartbeat to RoboCommand, about once a second:
    robocommand/robotx/<team_id>/<vehicle_id>/report   RxReport (Heartbeat)
built from the FCU telemetry the listener sends back (it learns where from
the hello this sends every 2 s). state is STATE_AUTO in AUTO or GUIDED,
otherwise STATE_MANUAL - RoboCommand starts a run once every declared
vehicle reports STATE_AUTO. If the telemetry stops (Orin/FCU down), so does
the heartbeat, rather than reporting a stale state. Type 'task 1' ...
'task 4' / 'task none' in this terminal to set its current_task.

Run Requests (RunDeclaration): this bridge builds and sends every one.
run_declaration.py hands it the vehicles, task tiers, pilot call sign and
UAV vehicle id (local UDP, --declare-port); the bridge signs it with what the
drone itself has, from the listener's telemetry:
  - geofence: the polygon on the FCU (what Mission Planner shows);
  - ceiling: the FCU's FENCE_ALT_MAX (set in Mission Planner).
It checks it the way RoboCommand does (inside the course / UAV transit area
to 1 m, ceiling at most 50 m HAE). RoboCommand never answers a rejection, so
a request that fails the check counts as rejected: it is NOT sent, and the
listener clears the fence from the FCU and puts its startup FENCE_* params
back. Only while the drone is disarmed.

RoboCommand's Heartbeat has no geofence field, so a fence written in Mission
Planner, or a new FENCE_ALT_MAX, re-sends the last Run Request
(last_run_declaration.json) the same way - after landing if armed. The drone
GUI shows each result.

UDP can drop packets and these are one-off messages, so each one is
re-sent every 0.5 s until the listener acknowledges it (up to --retry-for
seconds). The listener ignores repeats of a message it already has.

Needs (laptop only):
    pip install protobuf==7.35 paho-mqtt==2.0
and RoboNation's repo (https://github.com/robonation/robocommand), for its
RobotX_2026/gen/python folder - see --robocommand-gen.

Usage (team SGIT and the Orin at 192.168.1.153 are the defaults):
    python3 robocommand_bridge.py --broker <robocommand-broker-ip>
or double-click start_bridge.bat.
"""

import argparse
import hashlib
import itertools
import json
import socket
import sys
import threading
import time
from collections import deque
from pathlib import Path

DEFAULT_TEAM_ID = "SGIT"
DEFAULT_LISTENER_HOST = "192.168.1.153"  # the Orin
DEFAULT_UAV_REGISTRATION = "AA53577"  # the drone's CAAS registration (tag) - the Drone ID MDF shows
DEFAULT_VEHICLE_ID = "SGIT-53577"  # the drone's RoboCommand vehicle name
DEFAULT_DECLARE_PORT = 14615  # local UDP port run_declaration.py hands its Run Request to
DEFAULT_CONTROL_PORT = 14616  # local UDP port refresh_Rxcourse.py sends its reapply request to
MAX_CEILING_HAE_M = 50.0  # the highest geofence ceiling RoboCommand accepts

# RoboNation's repo, cloned either inside this script's folder (the GCS
# laptop's Geofence folder) or next to it - first one found wins.
_GEN_SUBPATH = Path("robocommand") / "RobotX_2026" / "gen" / "python"
_HERE = Path(__file__).resolve().parent
DEFAULT_GEN = next((p for p in (_HERE / _GEN_SUBPATH, _HERE.parent / _GEN_SUBPATH) if p.is_dir()), _HERE / _GEN_SUBPATH)
RESEND_INTERVAL_S = 0.5


def load_robocommand(gen_dir: Path):
    """Import RoboNation's generated message code from gen_dir."""
    if not (gen_dir / "robotx" / "rx_commands_pb2.py").is_file():
        sys.exit(f"Error: RoboNation's generated code isn't in {gen_dir} - clone "
                 "https://github.com/robonation/robocommand and pass --robocommand-gen <repo>/RobotX_2026/gen/python")
    sys.path.insert(0, str(gen_dir))
    try:
        from google.protobuf import json_format
        from robotx import rx_commands_pb2, rx_course_pb2
    except ImportError as e:
        sys.exit(f"Error: {e} - install the laptop requirements: pip install protobuf==7.35 paho-mqtt==2.0")
    return json_format, rx_commands_pb2, rx_course_pb2


def load_report_types():
    """The messages for sending the UAV heartbeat (after load_robocommand)."""
    import common_pb2
    from robotx import rx_common_pb2, rx_reports_pb2
    return common_pb2, rx_common_pb2, rx_reports_pb2


# What you type in the bridge's terminal for current_task -> RxTask name.
TASKS = {
    "none": "TASK_NONE",
    "1": "TASK_SAFE_PASSAGE",
    "2": "TASK_INFRA_SURVEY_REPAIR",
    "3": "TASK_COORDINATED_LOGISTICS",
    "4": "TASK_DYNAMIC_INCIDENT",
}
HELLO_INTERVAL_S = 2.0  # how often the listener is told where to send UAV telemetry
TELEMETRY_STALE_S = 5.0

# Task 4 commands RoboCommand wants acknowledged with an IncidentAck.
ACKED_COMMANDS = {"assistance_request", "keep_out_zone", "all_clear"}
# UAV reports the Orin can send through the listener: RxReport field -> name.
UAV_REPORT_KINDS = {
    "incident_ack": "IncidentAck",
    "readiness": "ReadinessReport",
    "safe_passage": "SafePassageReport",
    "resource_delivery": "ResourceDeliveryRequest",
}


def course_fingerprint(course: dict) -> str:
    """Stable identity of an RxCourse for change detection.

    sent_at is deliberately excluded: RoboCommand republishes the retained
    course with a fresh timestamp, and that alone must not reapply the fence.
    """
    identity = {name: course.get(name) for name in ("course_id", "pinger_freq_hz", "corners", "uav_transit_area")}
    return hashlib.sha256(
        json.dumps(identity, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()


class ReliableSender:
    """Sends JSON datagrams to the listener and re-sends each one until the
    listener acknowledges its id."""

    def __init__(self, target, retry_for_s: float):
        self.target = target
        self.retry_for_s = retry_for_s
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("0.0.0.0", 0))  # bound up front, so waiting for acks works before the first send
        self.sock.settimeout(0.1)
        self._lock = threading.Lock()
        self._pending = {}  # id -> [payload bytes, first sent, last sent, label, on_ack]
        self._ids = itertools.count(1)
        self._run_tag = f"{int(time.time())}"  # keeps ids unique across bridge restarts

    def send(self, payload: dict, label: str, on_ack=None, on_fail=None) -> str:
        """on_ack/on_fail (optional) are called once the listener acknowledges
        it, or the retry window expires. Returns the message id (for cancel)."""
        msg_id = f"{self._run_tag}-{next(self._ids)}"
        data = json.dumps(dict(payload, id=msg_id)).encode("utf-8")
        now = time.time()
        with self._lock:
            self._pending[msg_id] = [data, now, now, label, on_ack, on_fail]
        self._send_raw(data)
        return msg_id

    def cancel(self, msg_id: str) -> bool:
        """Stop retrying a message. Returns whether it was still pending."""
        with self._lock:
            return self._pending.pop(msg_id, None) is not None

    @staticmethod
    def _notify(callback) -> None:
        if callback is None:
            return
        try:
            callback()
        except Exception as e:  # a completion callback must not stop the bridge
            print(f"  completion callback failed: {e}")

    def _send_raw(self, data: bytes) -> None:
        try:
            self.sock.sendto(data, self.target)
        except OSError as e:
            print(f"  send failed: {e}")

    def send_once(self, payload: dict) -> None:
        """Fire-and-forget, e.g. the periodic hello."""
        self._send_raw(json.dumps(payload).encode("utf-8"))

    def service(self, on_datagram=None) -> None:
        """Read acks and re-send anything unacknowledged. Call in a loop.
        Anything else the listener sends (UAV telemetry) goes to on_datagram."""
        try:
            data, _addr = self.sock.recvfrom(4096)
            ack = json.loads(data.decode("utf-8"))
            if ack.get("type") == "robocommand_ack":
                with self._lock:
                    entry = self._pending.pop(ack.get("id"), None)
                if entry:
                    print(f"  listener has it: {entry[3]}")
                    self._notify(entry[4])
            elif on_datagram is not None:
                on_datagram(ack)
        except (socket.timeout, ValueError, UnicodeDecodeError):
            pass
        except OSError:
            pass  # e.g. Windows reports an unreachable listener this way - keep retrying

        now = time.time()
        expired = []
        with self._lock:
            for msg_id, entry in list(self._pending.items()):
                data, first, last, label = entry[:4]
                if now - first > self.retry_for_s:
                    del self._pending[msg_id]
                    expired.append(entry)
                elif now - last >= RESEND_INTERVAL_S:
                    entry[2] = now
                    self._send_raw(data)
        for entry in expired:
            print(f"  WARNING: listener never acknowledged {entry[3]} - is it running at {self.target[0]}:{self.target[1]}?")
            self._notify(entry[5] if len(entry) > 5 else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--team-id", default=DEFAULT_TEAM_ID, help="your RobotX team id (default: %(default)s)")
    parser.add_argument("--broker", default="127.0.0.1", help="RoboCommand MQTT broker address (default: %(default)s)")
    parser.add_argument("--port", type=int, default=1883, help="MQTT broker port (default: %(default)s)")
    parser.add_argument("--listener-host", default=DEFAULT_LISTENER_HOST,
                        help="IP address of the Orin running geofence_udp_listener.py (default: %(default)s)")
    parser.add_argument("--listener-port", type=int, default=14600, help="UDP port the listener is bound to (default: %(default)s)")
    parser.add_argument("--robocommand-gen", type=Path, default=DEFAULT_GEN,
                        help="RoboNation's RobotX_2026/gen/python folder (default: %(default)s)")
    parser.add_argument("--retry-for", type=float, default=15.0,
                        help="seconds to keep re-sending a message the listener hasn't acknowledged (default: %(default)s)")
    parser.add_argument("--vehicle-id", default=DEFAULT_VEHICLE_ID, help="the UAV's RoboCommand vehicle id (default: %(default)s)")
    parser.add_argument("--uav-registration", default=DEFAULT_UAV_REGISTRATION,
                        help="the UAV's CAAS registration number for its heartbeat (default: %(default)s)")
    parser.add_argument("--task", choices=sorted(TASKS), default="none",
                        help="current_task to report at start (default: %(default)s); change it live by typing 'task 1' etc.")
    parser.add_argument("--declare-port", type=int, default=DEFAULT_DECLARE_PORT,
                        help="local UDP port run_declaration.py hands its Run Request to (default: %(default)s)")
    parser.add_argument("--control-port", type=int, default=DEFAULT_CONTROL_PORT,
                        help="local UDP port refresh_Rxcourse.py sends its reapply request to (default: %(default)s)")
    parser.add_argument("--no-incident-ack", action="store_true",
                        help="don't send IncidentAck for UAV-targeted Task 4 commands (sent by default once the Orin has them)")
    args = parser.parse_args()
    if args.control_port == args.declare_port:
        parser.error("--control-port and --declare-port must be different")

    json_format, rx_commands_pb2, rx_course_pb2 = load_robocommand(args.robocommand_gen)
    common_pb2, rx_common_pb2, rx_reports_pb2 = load_report_types()
    from robotx import rx_requests_pb2
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        sys.exit("Error: paho-mqtt isn't installed - pip install protobuf==7.35 paho-mqtt==2.0")

    command_topic = f"robocommand/robotx/{args.team_id}/command"
    course_topic = "robocommand/robotx/course"
    report_topic = f"robocommand/robotx/{args.team_id}/{args.vehicle_id}/report"
    sender = ReliableSender((args.listener_host, args.listener_port), args.retry_for)

    # --- UAV heartbeat: telemetry from the listener -> RxReport/Heartbeat ----
    hb = {"task": TASKS[args.task], "seq": 0, "last_rx": None, "last_print": 0.0, "last_state": None, "stale_warned": False}

    def publish_heartbeat(t: dict) -> None:
        if t.get("type") != "uav_telemetry":
            return
        fields = {
            "state": common_pb2.RobotState.Value(t["state"]),
            "vehicle_type": rx_common_pb2.TYPE_UAV,
            "flight_phase": rx_common_pb2.FlightPhase.Value(t["flight_phase"]),
            "current_task": rx_common_pb2.RxTask.Value(hb["task"]),
            "uav_registration": args.uav_registration,
        }
        for name in ("spd_mps", "heading_deg", "roll_deg", "pitch_deg", "altitude_hae_m"):
            if t.get(name) is not None:
                fields[name] = float(t[name])
        heartbeat = rx_reports_pb2.Heartbeat(**fields)
        if t.get("latitude") is not None:
            heartbeat.position.CopyFrom(common_pb2.LatLng(latitude=t["latitude"], longitude=t["longitude"]))
        hb["seq"] += 1
        report = rx_reports_pb2.RxReport(team_id=args.team_id, vehicle_id=args.vehicle_id, seq=hb["seq"], heartbeat=heartbeat)
        report.sent_at.FromNanoseconds(time.time_ns())
        client.publish(report_topic, report.SerializeToString(), qos=0)

        now = time.time()
        hb["last_rx"], hb["stale_warned"] = now, False
        changed = (t["state"], t.get("mode"), t["flight_phase"]) != hb["last_state"]
        if changed or now - hb["last_print"] > 10:
            hb["last_state"], hb["last_print"] = (t["state"], t.get("mode"), t["flight_phase"]), now
            alt = f"{t['altitude_hae_m']:.1f} m {t.get('altitude_source', '')}" if t.get("altitude_hae_m") is not None else "no position"
            print(f"UAV heartbeat #{hb['seq']} -> RoboCommand: {t['state']} (mode {t.get('mode')}), "
                  f"{t['flight_phase'].replace('FLIGHT_PHASE_', '').lower()}, alt {alt}, task {hb['task']}")

    # --- other UAV reports: IncidentAck / ReadinessReport / SafePassageReport /
    # ResourceDeliveryRequest. They share the heartbeat's per-vehicle seq. ----
    seen_reports = deque(maxlen=500)

    def publish_report(body_field: str, body: dict, label: str) -> None:
        """Raises (ParseError/ValueError) if body doesn't fit RoboNation's schema."""
        report = rx_reports_pb2.RxReport()
        json_format.ParseDict({"team_id": args.team_id, "vehicle_id": args.vehicle_id, "seq": hb["seq"] + 1,
                               body_field: body}, report)
        hb["seq"] += 1
        report.sent_at.FromNanoseconds(time.time_ns())
        client.publish(report_topic, report.SerializeToString(), qos=1)
        print(f"UAV report #{hb['seq']} -> RoboCommand: {label}: {json.dumps(body)}")

    # --- Run Requests: built, checked and sent here ---------------------------
    latest_course = {
        "corners": None, "transit": [],  # used by send_declaration()
        "fingerprint": None, "body": None, "course_id": None,  # cache / change detection
        "received_at": 0.0, "last_forwarded_at": 0.0, "last_result": "none", "msg_id": None,
    }
    latest = {"telemetry": None, "at": 0.0}  # the listener's last UAV telemetry
    fence_results = {}  # fence_change id -> the result sent to the listener
    fence_waiting = set()  # fence_change ids held back while the drone is armed
    declare_results = {}  # run_declaration.py request id -> the answer sent back
    # The drone's FENCE_ALT_MAX (from the listener's telemetry): the ceiling is
    # this value itself; a change re-sends the Run Request.
    alt_max = {"value": None, "resend": False, "waiting_noted": False}

    def ceiling_from_drone(t: dict):
        """(ceiling, how it was made) - the drone's FENCE_ALT_MAX itself, or (None, why not)."""
        if alt_max["value"] is None:
            return None, "no FENCE_ALT_MAX from the drone yet"
        return alt_max["value"], f"FENCE_ALT_MAX {alt_max['value']:g} m"

    def send_declaration(base: dict, fence_vertices, t, what: str) -> "tuple[bool, str]":
        """Sign base (vehicles, tiers, pilot) with the FCU's fence and the ceiling
        from FENCE_ALT_MAX, check it as RoboCommand does, and publish it - or,
        if RoboCommand would reject it, don't: the listener clears the fence."""
        from run_declaration import TIERS, _closed, geofence_problems, load_last_declaration, save_last_declaration
        has_uav = base.get("uav") is not None
        fence, ceiling, how = [], 0.0, ""
        if has_uav:
            if t is None or time.time() - latest["at"] > TELEMETRY_STALE_S:
                return False, f"{what}: no UAV telemetry from the drone (is the drone GUI running on the Orin?). NOT sent"
            if t.get("armed"):
                return False, f"{what}: the drone is ARMED - Run Requests are only sent disarmed, on the ground. NOT sent"
            if latest_course["corners"] is None:
                return False, f"{what}: no course from RoboCommand to check the geofence against. NOT sent"
            if len(fence_vertices or []) < 3:
                return False, f"{what}: the FCU has no single polygon fence to declare. NOT sent"
            fence = _closed([(float(lat), float(lon)) for lat, lon in fence_vertices])
            ceiling, how = ceiling_from_drone(t)
            if ceiling is None:
                return False, f"{what}: can't work out the ceiling ({how}). NOT sent"
            problems = geofence_problems(fence, latest_course["corners"], latest_course["transit"], ceiling,
                                         MAX_CEILING_HAE_M)
            if problems:
                reason = "; ".join(problems)
                sender.send({"type": "fence_rejected", "reason": reason}, "Run Request rejected - clear the fence")
                return False, (f"{what}: RoboCommand would REJECT it ({reason}, ceiling = {how}) - NOT sent. The "
                               "drone's fence is being cleared and its startup FENCE_* params restored")
        last = load_last_declaration() or {}
        seq = max(int(time.time()), int(last.get("seq", 0)) + 1)
        tiers = base["tiers"]
        declaration = rx_requests_pb2.RunDeclaration(
            vehicle_ids=base["vehicles"],
            **{f"task{n}_tier": common_pb2.TaskTier.Value(TIERS[tiers[str(n)]]) for n in range(1, 5)},
        )
        if has_uav:
            declaration.uav_geofence.extend(common_pb2.LatLng(latitude=lat, longitude=lon) for lat, lon in fence)
            declaration.uav_geofence_max_altitude_hae_m = ceiling
            declaration.uav_pilot_callsign = base["uav"]["pilot_callsign"]
        request = rx_requests_pb2.RxRequest(team_id=args.team_id, seq=seq, run_declaration=declaration)
        request.sent_at.FromNanoseconds(time.time_ns())
        info = client.publish(f"robocommand/robotx/{args.team_id}/request", request.SerializeToString(), qos=1)
        try:
            info.wait_for_publish(5)
        except (RuntimeError, ValueError):
            pass
        if not info.is_published():
            return False, f"{what}: the Run Request didn't reach RoboCommand's broker - NOT sent"
        uav = dict(base["uav"], geofence=[list(p) for p in fence], ceiling_hae_m=ceiling) if has_uav else None
        save_last_declaration({"team_id": args.team_id, "seq": seq, "vehicles": base["vehicles"], "tiers": tiers,
                               "uav_vehicle_id": base.get("uav_vehicle_id"), "uav": uav})
        detail = (f" ({len(fence)} points from the FCU, ceiling {ceiling:.1f} m HAE = {how}, pilot "
                  f"{base['uav']['pilot_callsign']})") if has_uav else ""
        return True, f"Run Request #{seq} sent for {what}{detail}"

    def on_fence_change(change: dict, t: dict) -> None:
        change_id = change.get("id")
        if change_id in fence_results:  # the listener didn't get the result - send it again
            sender.send_once(fence_results[change_id])
            return
        if t.get("armed"):
            if change_id not in fence_waiting:
                fence_waiting.add(change_id)
                print("Fence changed in Mission Planner while the drone is ARMED - the new Run Request waits until it's disarmed")
            return
        from run_declaration import DEFAULT_PILOT_CALLSIGN, DEFAULT_VEHICLE_ID, load_last_declaration
        last = load_last_declaration()
        what = "the Mission Planner fence"
        if not last or not last.get("uav"):
            # No Run Request sent yet: declare the UAV alone with the built-in vehicle id and pilot
            # (task tiers none) - run_declaration.py declares the full team and tiers.
            last = {"vehicles": [DEFAULT_VEHICLE_ID], "tiers": {str(n): "none" for n in range(1, 5)},
                    "uav_vehicle_id": DEFAULT_VEHICLE_ID, "uav": {"pilot_callsign": DEFAULT_PILOT_CALLSIGN}}
            what = "the Mission Planner fence (vehicle and pilot built in, task tiers none)"
        ok, text = send_declaration(last, change.get("vertices") or [], t, what)
        alt_max["resend"] = False  # this one already carries the current FENCE_ALT_MAX ceiling
        print(("" if ok else "WARNING: ") + text)
        fence_results[change_id] = {"type": "fence_change_result", "id": change_id, "ok": ok, "text": text}
        sender.send_once(fence_results[change_id])

    def on_fence_alt_max(t: dict) -> None:
        from run_declaration import load_last_declaration
        value = float(t["fence_alt_max"])
        changed = alt_max["value"] is not None and abs(value - alt_max["value"]) > 0.01
        alt_max["value"] = value
        last = load_last_declaration()
        if changed and last and last.get("uav"):
            print(f"FENCE_ALT_MAX on the drone is now {value:g} m - the Run Request goes again with the new ceiling")
            alt_max["resend"] = True
        if not alt_max["resend"] or t.get("fence_change"):
            return  # (a pending fence change re-sends with the new ceiling anyway)
        if t.get("armed"):
            if not alt_max["waiting_noted"]:
                alt_max["waiting_noted"] = True
                print("FENCE_ALT_MAX changed while the drone is ARMED - the new Run Request waits until it's disarmed")
            return
        if len(t.get("fcu_fence") or []) < 3:
            print("FENCE_ALT_MAX changed, but the FCU has no polygon fence - no Run Request to update")
            alt_max["resend"], alt_max["waiting_noted"] = False, False
            return
        ok, text = send_declaration(last, t["fcu_fence"], t, "the new FENCE_ALT_MAX")
        print(("" if ok else "WARNING: ") + text)
        alt_max["resend"], alt_max["waiting_noted"] = False, False
        sender.send_once({"type": "bridge_notice", "ok": ok, "text": text})

    def on_listener(d: dict) -> None:
        """Datagrams the listener sends here: UAV telemetry and UAV reports."""
        if d.get("type") == "uav_telemetry":
            latest["telemetry"], latest["at"] = d, time.time()
            publish_heartbeat(d)
            if d.get("fence_alt_max") is not None:
                on_fence_alt_max(d)
            if d.get("fence_change"):
                on_fence_change(d["fence_change"], d)
        elif d.get("type") == "uav_report":
            report_id, kind = d.get("id"), d.get("kind")
            if report_id in seen_reports:  # a re-send of one already published - just re-acknowledge
                sender.send_once({"type": "uav_report_ack", "id": report_id, "ok": True})
                return
            if kind not in UAV_REPORT_KINDS:
                sender.send_once({"type": "uav_report_ack", "id": report_id, "ok": False, "error": f"unknown report '{kind}'"})
                return
            try:
                publish_report(kind, d.get("body") or {}, UAV_REPORT_KINDS[kind])
            except Exception as e:  # a bad report must not stop the bridge
                print(f"UAV report REJECTED ({kind}): {e}")
                sender.send_once({"type": "uav_report_ack", "id": report_id, "ok": False, "error": str(e)[:200]})
                return
            seen_reports.append(report_id)
            sender.send_once({"type": "uav_report_ack", "id": report_id, "ok": True})

    def on_declare_request(data: bytes, addr) -> None:
        """A Run Request from run_declaration.py: sign it with the FCU's fence and
        ceiling, send it (or not), and answer with the result."""
        try:
            req = json.loads(data.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        if req.get("type") != "declare":
            return
        if req.get("id") not in declare_results:  # (a re-send of one already handled gets the same answer)
            t = latest["telemetry"]
            ok, text = send_declaration(req, (t or {}).get("fcu_fence") or [], t, "run_declaration.py")
            print(("" if ok else "WARNING: ") + text)
            declare_results[req.get("id")] = {"type": "declare_result", "id": req.get("id"), "ok": ok, "text": text}
            sender.send_once({"type": "bridge_notice", "ok": ok, "text": text})  # also in the drone GUI
        try:
            declare_sock.sendto(json.dumps(declare_results[req.get("id")]).encode("utf-8"), addr)
        except OSError:
            pass

    # --- cached-course reapply from refresh_Rxcourse.py ----------------------
    course_reapply = {"pending": [], "msg_id": None, "results": {}, "result_order": deque()}

    def _remember_reapply_result(result: dict) -> None:
        req_id = result.get("id")
        course_reapply["results"][req_id] = result
        course_reapply["result_order"].append(req_id)
        while len(course_reapply["result_order"]) > 100:
            old = course_reapply["result_order"].popleft()
            course_reapply["results"].pop(old, None)

    def _send_control_reply(addr, result: dict) -> None:
        try:
            control_sock.sendto(json.dumps(result).encode("utf-8"), addr)
        except OSError as e:
            print(f"  couldn't answer refresh_Rxcourse.py: {e}")

    def _finish_course_reapply(ok: bool, text: str, acked: bool, sent: bool = True) -> None:
        pending = course_reapply["pending"]
        course_reapply["pending"] = []
        course_reapply["msg_id"] = None
        for req_id, addr in pending:
            result = {"type": "course_reapply_result", "id": req_id, "ok": ok, "sent": sent, "acked": acked,
                      "fingerprint": latest_course["fingerprint"], "course_id": latest_course["course_id"],
                      "text": text if ok else None, "error": None if ok else text}
            _remember_reapply_result(result)
            _send_control_reply(addr, result)
        print(("" if ok else "WARNING: ") + text)

    def _cancel_course_reapply(reason: str) -> None:
        msg_id = course_reapply["msg_id"]
        if msg_id is not None:
            sender.cancel(msg_id)
        pending = course_reapply["pending"]
        course_reapply["pending"] = []
        course_reapply["msg_id"] = None
        for req_id, addr in pending:
            result = {"type": "course_reapply_result", "id": req_id, "ok": False, "sent": True, "acked": False,
                      "fingerprint": latest_course["fingerprint"], "course_id": latest_course["course_id"],
                      "error": reason}
            _remember_reapply_result(result)
            _send_control_reply(addr, result)

    def handle_control_request(data: bytes, addr) -> None:
        """A cached-course reapply request from refresh_Rxcourse.py."""
        try:
            req = json.loads(data.decode("utf-8"))
        except (ValueError, UnicodeDecodeError):
            return
        if req.get("type") != "course_reapply":
            return
        req_id = req.get("id")
        if req_id is None:
            return
        if req_id in course_reapply["results"]:  # a re-send of one already answered
            _send_control_reply(addr, course_reapply["results"][req_id])
            return
        if latest_course["body"] is None:
            result = {"type": "course_reapply_result", "id": req_id, "ok": False, "sent": False, "acked": False,
                      "error": "no RxCourse cached yet - wait for RoboCommand's course"}
            _remember_reapply_result(result)
            _send_control_reply(addr, result)
            return
        if all(req_id != pending_id for pending_id, _ in course_reapply["pending"]):
            course_reapply["pending"].append((req_id, addr))
        if course_reapply["msg_id"] is not None:
            print(f"Course reapply already in progress - {req_id} will get the same result")
            return
        payload = {"type": "robocommand", "kind": "course", "message": latest_course["body"]}

        def on_ack():
            latest_course["last_forwarded_at"] = time.time()
            latest_course["last_result"] = "reapplied"
            _finish_course_reapply(
                True,
                f"cached course {latest_course['course_id']} ({latest_course['fingerprint'][:12]}) re-sent and acknowledged",
                True,
            )

        def on_fail():
            latest_course["last_result"] = "failed"
            _finish_course_reapply(
                False,
                f"listener did not acknowledge the cached course within {args.retry_for:g} s",
                False,
            )

        course_reapply["msg_id"] = sender.send(payload, "course reapply", on_ack=on_ack, on_fail=on_fail)
        print(f"Course reapply requested - resending cached course {latest_course['course_id']} "
              f"({latest_course['fingerprint'][:12]})")

    def read_terminal() -> None:
        """'task 1' / 'task none' typed in this terminal sets current_task."""
        while True:
            try:
                line = input().strip().lower()
            except (EOFError, OSError):
                return  # no terminal (e.g. started in the background)
            parts = line.split()
            if len(parts) == 2 and parts[0] == "task" and parts[1] in TASKS:
                hb["task"] = TASKS[parts[1]]
                print(f"current_task -> {hb['task']} (in the next heartbeat)")
            elif line:
                print("type: task none | task 1 | task 2 | task 3 | task 4")

    def to_dict(message) -> dict:
        return json_format.MessageToDict(message, preserving_proto_field_name=True)

    def on_connect(client, _userdata, _flags, reason_code, _properties):
        if reason_code.is_failure:
            print(f"MQTT connect refused: {reason_code}")
            return
        client.subscribe([(command_topic, 1), (course_topic, 1)])
        print(f"Connected to RoboCommand at {args.broker}:{args.port}, listening on {command_topic} and {course_topic}")

    def on_disconnect(_client, _userdata, _flags, reason_code, _properties):
        print(f"MQTT disconnected ({reason_code}) - reconnecting...")

    def on_message(_client, _userdata, msg):
        try:
            if msg.topic == course_topic:
                course = rx_course_pb2.RxCourse()
                course.ParseFromString(msg.payload)
                body = to_dict(course)
                fingerprint = course_fingerprint(body)
                latest_course["corners"] = [(p.latitude, p.longitude) for p in course.corners]
                latest_course["transit"] = [(p.latitude, p.longitude) for p in course.uav_transit_area]
                latest_course["body"] = body
                latest_course["course_id"] = course.course_id or "?"
                latest_course["received_at"] = time.time()
                if fingerprint == latest_course["fingerprint"]:
                    latest_course["last_result"] = "unchanged"
                    print(f"RoboCommand -> course {latest_course['course_id']} unchanged "
                          f"({fingerprint[:12]}) - not forwarded")
                    return
                old_send = latest_course.get("msg_id")
                if old_send is not None:
                    sender.cancel(old_send)  # the newest course wins; don't let an old retry overwrite it
                _cancel_course_reapply("a newer RxCourse arrived before the reapply completed")
                previous = latest_course["fingerprint"]
                latest_course["fingerprint"] = fingerprint
                latest_course["last_result"] = "forwarded"
                label = f"course {latest_course['course_id']}"
                payload = {"type": "robocommand", "kind": "course", "message": body}

                def on_course_ack(expected=fingerprint):
                    if latest_course["fingerprint"] == expected:
                        latest_course["last_forwarded_at"] = time.time()
                        latest_course["last_result"] = "acked"
                        latest_course["msg_id"] = None

                latest_course["msg_id"] = sender.send(payload, label, on_ack=on_course_ack)
                change = (f"first {fingerprint[:12]}" if previous is None
                          else f"changed {previous[:12]} -> {fingerprint[:12]}")
                print(f"RoboCommand -> course {latest_course['course_id']} {change} - forwarding")
                return
            command = rx_commands_pb2.RxCommand()
            command.ParseFromString(msg.payload)
            body = to_dict(command)
            which = command.WhichOneof("body") or "empty"
            label = f"command #{command.seq} {which}"
            payload = {"type": "robocommand", "kind": "command", "message": body}
        except Exception as e:  # a malformed message must not stop the bridge
            print(f"Couldn't decode a message on {msg.topic}: {e}")
            return
        print(f"RoboCommand -> {label}: {json.dumps(body)}")
        on_ack = None
        if not args.no_incident_ack and which in ACKED_COMMANDS:
            # Task 4: acknowledge a UAV-targeted command - once the Orin has it,
            # so the ack means the vehicle side really received it.
            target = command.vehicle_id
            for_uav = target == args.vehicle_id if target else \
                getattr(command, which).vehicle_type == rx_common_pb2.TYPE_UAV
            if for_uav:
                seq = command.seq
                on_ack = lambda: publish_report("incident_ack", {"command_seq": seq}, f"IncidentAck for command #{seq}")
        sender.send(payload, label, on_ack=on_ack)

    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"{args.team_id}-ocs-bridge")
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=5)
    print(f"Forwarding RoboCommand to the listener at {args.listener_host}:{args.listener_port}, and the UAV's "
          f"heartbeat to {report_topic}. Type 'task 1' (etc.) to set current_task. Ctrl+C to stop.")
    threading.Thread(target=read_terminal, daemon=True).start()
    declare_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # Run Requests from run_declaration.py
    try:
        declare_sock.bind(("127.0.0.1", args.declare_port))
    except OSError as e:
        sys.exit(f"Error: can't take Run Requests on 127.0.0.1:{args.declare_port} ({e}) - is another bridge running?")
    declare_sock.setblocking(False)
    control_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # refresh_Rxcourse.py
    try:
        control_sock.bind(("127.0.0.1", args.control_port))
    except OSError as e:
        sys.exit(f"Error: can't take course reapply requests on 127.0.0.1:{args.control_port} ({e}) - "
                 "is another bridge running?")
    control_sock.setblocking(False)
    started, last_hello = time.time(), 0.0
    try:
        client.connect_async(args.broker, args.port, keepalive=30)
        client.loop_start()
        while True:
            sender.service(on_listener)
            try:
                data, addr = declare_sock.recvfrom(65536)
                on_declare_request(data, addr)
            except OSError:
                pass  # nothing waiting (or Windows reporting an old reply's port as closed)
            try:
                data, addr = control_sock.recvfrom(65536)
                handle_control_request(data, addr)
            except OSError:
                pass
            now = time.time()
            if now - last_hello >= HELLO_INTERVAL_S:
                sender.send_once({"type": "robocommand_hello"})
                last_hello = now
            silent_since = hb["last_rx"] or started
            if now - silent_since > TELEMETRY_STALE_S + (5 if hb["last_rx"] is None else 0) and not hb["stale_warned"]:
                hb["stale_warned"] = True
                print("WARNING: no UAV telemetry from the listener - the UAV heartbeat to RoboCommand is paused "
                      "(is the drone GUI running on the Orin, with the FCU connected?)")
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        client.loop_stop()
        client.disconnect()


if __name__ == "__main__":
    main()
