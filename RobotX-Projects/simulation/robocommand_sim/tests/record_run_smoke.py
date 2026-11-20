#!/usr/bin/env python3
"""Exercise the recorder over an isolated Gazebo Transport partition."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile
import threading
import time
import uuid

os.environ["GZ_PARTITION"] = "recorder_smoke_" + uuid.uuid4().hex[:10]

from gz.msgs10.double_pb2 import Double
from gz.msgs10.odometry_pb2 import Odometry

from robocommand import topics
from robocommand.bus import JsonBus
from robocommand.record_run import RunRecorder


def main() -> None:
    package = Path(__file__).resolve().parents[1]
    with tempfile.TemporaryDirectory(prefix="robocommand-recorder-") as temp:
        recorder = RunRecorder(Path(temp), package, sample_hz=5)
        bus = JsonBus()
        thrust = bus.node.advertise(
            "/model/usv_four_thruster/joint/motor_port_joint/cmd_thrust", Double)
        odom = bus.node.advertise("/model/usv_four_thruster/odometry", Odometry)

        def publish() -> None:
            time.sleep(.8)
            for _ in range(12):
                bus.publish(topics.STATUS_USV, bus.envelope(
                    "smoke", state="AUTO", stage="ROUTE_FOLLOWING",
                    cv_frames=12, cv_detections=2))
                bus.publish(topics.heartbeat("USV1"), bus.envelope(
                    "smoke", state="AUTO", stage="ROUTE_FOLLOWING",
                    position={"x": 1, "y": 2, "z": .1}))
                thrust.publish(Double(data=4.5))
                message = Odometry()
                message.pose.position.x = 1.0
                message.pose.position.y = 2.0
                message.pose.orientation.w = 1.0
                odom.publish(message)
                time.sleep(.12)

        worker = threading.Thread(target=publish)
        worker.start()
        recorder.run(duration=3.5)
        worker.join()
        summary = json.loads((recorder.directory / "summary.json").read_text())
        assert summary["bus_events"] > 0, summary
        assert summary["boat_odometry_messages"] > 0, summary
        assert summary["thruster_command_messages"] > 0, summary
        assert summary["peak_abs_gazebo_thrust_command"] == 4.5, summary
        assert summary["telemetry_rows"] >= 10, summary
        print("RECORDER_TRANSPORT_SMOKE_OK", json.dumps({
            key: summary[key] for key in ("telemetry_rows", "bus_events",
                                     "boat_odometry_messages", "thruster_command_messages")
        }))


if __name__ == "__main__":
    main()
