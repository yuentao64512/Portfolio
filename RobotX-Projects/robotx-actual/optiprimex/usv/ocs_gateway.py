"""OcsGateway: transitional TCP 9102 length-prefixed JSON link (ADR-3). heartbeat out at 1 Hz,
robocommand_command in -> Command on the command queue."""
from __future__ import annotations

import json
import logging
import socket
import struct
import threading
import time
import uuid
from typing import Callable, List, Optional

log = logging.getLogger("optiprimex.gateway")


class OcsGateway:
    def __init__(self, port: int, status_fn: Callable[[], dict], on_command: Callable[[dict], None],
                 host: str = "0.0.0.0", vehicle_id: str = "USV1"):
        self.port, self.host = port, host
        self.status_fn, self.on_command, self.vehicle_id = status_fn, on_command, vehicle_id
        self._clients: List[socket.socket] = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._srv: Optional[socket.socket] = None
        self._threads: List[threading.Thread] = []

    def start(self) -> None:
        self._srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._srv.bind((self.host, self.port))
        self._srv.listen(4)
        self._srv.settimeout(0.5)
        t = threading.Thread(target=self._accept_loop, name="gw-accept", daemon=True)
        t.start()
        h = threading.Thread(target=self._heartbeat_loop, name="gw-hb", daemon=True)
        h.start()
        self._threads = [t, h]

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except Exception:
                    pass
            self._clients = []
        if self._srv:
            self._srv.close()

    @staticmethod
    def frame(obj: dict) -> bytes:
        data = json.dumps(obj).encode()
        return struct.pack(">I", len(data)) + data

    def _accept_loop(self) -> None:
        while not self._stop.is_set():
            try:
                c, _ = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            c.settimeout(0.5)
            with self._lock:
                self._clients.append(c)
            threading.Thread(target=self._rx_loop, args=(c,), daemon=True).start()

    def _rx_loop(self, c: socket.socket) -> None:
        buf = b""
        while not self._stop.is_set():
            try:
                chunk = c.recv(4096)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buf += chunk
            while len(buf) >= 4:
                n = struct.unpack(">I", buf[:4])[0]
                if len(buf) < 4 + n:
                    break
                raw, buf = buf[4:4 + n], buf[4 + n:]
                try:
                    msg = json.loads(raw)
                except Exception:
                    continue
                if msg.get("type") == "robocommand_command":
                    env = {"schema_version": 1, "msg_id": msg.get("msg_id") or str(uuid.uuid4()), "seq": 0,
                           "sent_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()), "run_id": msg.get("run_id"),
                           "declaration_seq": msg.get("declaration_seq"), "vehicle_id": self.vehicle_id, "kind": "cmd",
                           "source": "ocs_gateway", "body": {"action": msg["action"], "task": msg.get("task"),
                                                              "args": msg.get("args") or {}}}
                    self.on_command(env)
        with self._lock:
            if c in self._clients:
                self._clients.remove(c)
        try:
            c.close()
        except Exception:
            pass

    def _heartbeat_loop(self) -> None:
        while not self._stop.is_set():
            time.sleep(1.0)
            try:
                hb = {"type": "heartbeat", "vehicle_id": self.vehicle_id, "status": self.status_fn()}
            except Exception:
                continue
            data = self.frame(hb)
            with self._lock:
                for c in list(self._clients):
                    try:
                        c.sendall(data)
                    except OSError:
                        self._clients.remove(c)
