"""wire: the UAV team's §5 wire, our side (v0.1.64, D4 of docs/OVERNIGHT_2026-11-07_UAV.md).

PORTED VERBATIM from `uav/geofence-tools/robocommand_bridge.py` (the field-tested v2 reference, MERGE_GUIDE §4-§5): the constants,
`course_fingerprint()` and `ReliableSender` (id + ack, re-send every 0.5 s, give up after retry_for_s, cancel, on_ack / on_fail). Not one
character of these is changed (tests/test_v0164.py compares them with the snapshot's source text). What we need differently lives in
`BoundSender` below: the same class, its socket bound to UAV_BRIDGE_PORT (default 14605) instead of an ephemeral port - the listener
sends its telemetry to the address the hello came from, so the port must be the one the drone team opens."""
import hashlib
import itertools
import json
import socket
import threading
import time

DEFAULT_TEAM_ID = "SGIT"
DEFAULT_UAV_REGISTRATION = "SGIT-53577"  # OURS (v0.1.78.20): Garuda's drone id; his bridge: "AA53577" (the CAAS tag, venue: no match)
DEFAULT_VEHICLE_ID = "UAV1"  # OURS (v0.1.78.20): RoboCommand's vehicle id; his bridge: "SGIT-53577"
MAX_CEILING_HAE_M = 50.0  # the highest geofence ceiling RoboCommand accepts
RESEND_INTERVAL_S = 0.5
TASKS = {
    "none": "TASK_NONE",
    "1": "TASK_SAFE_PASSAGE",
    "2": "TASK_INFRA_SURVEY_REPAIR",
    "3": "TASK_COORDINATED_LOGISTICS",
    "4": "TASK_DYNAMIC_INCIDENT",
}
HELLO_INTERVAL_S = 2.0  # how often the listener is told where to send UAV telemetry
TELEMETRY_STALE_S = 5.0
ACKED_COMMANDS = {"assistance_request", "keep_out_zone", "all_clear"}
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


# ---- ours (not his) -------------------------------------------------------------------------------------------------------
DEFAULT_LISTEN_PORT = 14600          # the Orin's geofence_udp_listener.py
DEFAULT_BRIDGE_PORT = 14605          # ours: the adapter's socket (hello out, telemetry / acks / reports in)


class BoundSender(ReliableSender):
    """ReliableSender with its socket on a fixed port (the drone team opens UAV_BRIDGE_PORT on their side's firewall / relay)."""

    def __init__(self, target, retry_for_s: float, bind_host: str = "0.0.0.0", bind_port: int = DEFAULT_BRIDGE_PORT):
        super().__init__(target, retry_for_s)
        self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
        self.sock.bind((bind_host, int(bind_port)))
        self.sock.settimeout(0.1)
        self.bound = self.sock.getsockname()
