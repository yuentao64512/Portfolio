#!/usr/bin/env python3
"""Read one rendered Gazebo frame from the boat and drone, without control."""

from __future__ import annotations

import threading
import time

import cv2

from gz.msgs10.image_pb2 import Image
from gz.transport13 import Node

from robocommand.perception import detect_beacons, detect_usv_markers, gazebo_image_to_bgr


def main() -> int:
    node = Node()
    boat_topic = "/usv/front_camera/image"
    drone_topics = [
        topic for topic in node.topic_list()
        if topic.endswith("/image") and "iris_drone" in topic
    ]
    if not drone_topics:
        raise RuntimeError("No Gazebo drone camera image topic")
    drone_topic = sorted(drone_topics)[0]
    results: dict[str, tuple] = {}
    ready = threading.Event()

    def capture(name: str):
        def callback(msg: Image, *_: object) -> None:
            if name in results:
                return
            frame = gazebo_image_to_bgr(msg)
            cv2.imwrite(f"/tmp/robotx_{name}_sim_cv.png", frame)
            detections = detect_usv_markers(frame) if name == "boat" else detect_beacons(frame)
            results[name] = (frame.shape, int(msg.pixel_format_type), [(d.colour, round(d.u), round(d.v)) for d in detections])
            if len(results) == 2:
                ready.set()
        return callback

    boat_callback = capture("boat")
    drone_callback = capture("drone")
    if not node.subscribe(Image, boat_topic, boat_callback):
        raise RuntimeError(f"Cannot subscribe {boat_topic}")
    if not node.subscribe(Image, drone_topic, drone_callback):
        raise RuntimeError(f"Cannot subscribe {drone_topic}")
    if not ready.wait(15.0):
        raise RuntimeError(f"Camera frame timeout; received={list(results)}")
    for name, topic in (("boat", boat_topic), ("drone", drone_topic)):
        shape, fmt, colours = results[name]
        print(f"{name} topic={topic} shape={shape} pixel_format={fmt} detections={colours}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
