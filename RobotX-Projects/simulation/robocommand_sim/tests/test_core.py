from __future__ import annotations

import math
import time

import cv2
import numpy as np

from robocommand.contract import heartbeat_ok, validate_declaration, validate_report, xy_to_latlon, latlon_to_xy
from robocommand.perception import TrackStore, detect_beacons, detect_usv_markers, gazebo_image_to_bgr, pixel_to_world
from robocommand.sim_boat_driver import body_error


def test_declaration_contract() -> None:
    assert validate_declaration({"tier": "core", "vehicles": ["USV1"]}) == (True, "ok")
    ok, reason = validate_declaration({"tier": "advanced", "vehicles": ["USV1"]})
    assert not ok and "UAV1" in reason


def test_heartbeat_fresh_auto() -> None:
    assert heartbeat_ok({"ts": time.time(), "mode": "AUTO"}, time.time())[0]
    assert not heartbeat_ok({"ts": time.time() - 4, "mode": "AUTO"}, time.time())[0]
    assert not heartbeat_ok({"ts": time.time(), "mode": "MANUAL"}, time.time())[0]


def test_coordinate_round_trip() -> None:
    lat, lon = xy_to_latlon(-22.0, 18.0, 1.3, 103.77)
    x, y = latlon_to_xy(lat, lon, 1.3, 103.77)
    assert abs(x + 22.0) < 1e-6
    assert abs(y - 18.0) < 1e-6


def test_report_contract() -> None:
    item = {"id": "x", "lat": 1.0, "lon": 2.0, "class": "gate", "colour": "green", "lit": True, "conf": 0.9, "sigma_m": 0.2}
    assert validate_report({"items": [item], "scan": {"state": "complete"}})[0]


def test_detector_finds_bright_colours() -> None:
    image = np.full((240, 320, 3), 90, np.uint8)
    cv2.circle(image, (70, 100), 12, (0, 0, 255), -1)
    cv2.circle(image, (150, 100), 12, (0, 255, 0), -1)
    # Harmonic's blue emissive beacon renders cyan-blue (about HSV H=99),
    # deliberately separated from the water texture around H=114.
    cv2.circle(image, (230, 100), 12, (255, 170, 0), -1)
    colours = {d.colour for d in detect_beacons(image)}
    assert {"red", "green", "blue"}.issubset(colours)


def test_simulated_boat_camera_cv() -> None:
    from gz.msgs10.image_pb2 import Image

    frame = np.full((240, 320, 3), 95, np.uint8)
    cv2.rectangle(frame, (65, 70), (75, 115), (0, 0, 255), -1)
    cv2.rectangle(frame, (145, 70), (155, 115), (0, 255, 0), -1)
    cv2.rectangle(frame, (225, 70), (235, 115), (255, 170, 0), -1)
    msg = Image(width=320, height=240, pixel_format_type=8, data=frame.tobytes())
    decoded = gazebo_image_to_bgr(msg)
    assert np.array_equal(decoded, frame)
    assert {"red", "green", "blue"}.issubset({item.colour for item in detect_usv_markers(decoded)})


def test_sim_boat_body_error() -> None:
    forward, left = body_error(0, 0, 0, 4, 2)
    assert (round(forward, 4), round(left, 4)) == (4, 2)
    forward, left = body_error(0, 0, -math.pi / 2, 0, -4)
    assert round(forward, 4) == 4 and abs(left) < 1e-9


def test_projection_centre_zero() -> None:
    dx, dy = pixel_to_world(320, 240, 640, 480, 15, 0, 2.0)
    assert abs(dx) < 1e-9 and abs(dy) < 1e-9


def test_downward_gimbal_pixel_axes() -> None:
    dx, dy = pixel_to_world(420, 240, 640, 480, 15, 0, 2.0)
    assert abs(dx) < 1e-9 and dy < 0


def test_track_merge_and_blink() -> None:
    store = TrackStore(1.5)
    for timestamp in (0.0, 0.1, 0.2, 1.1, 1.2, 3.0):
        store.add("blue", 2.0, 3.0, 40, timestamp)
    tracks = list(store.confirmed())
    assert len(tracks) == 1 and tracks[0].blinking


def test_confirmed_output_consolidation() -> None:
    store = TrackStore(1.5)
    for index in range(5):
        store.add("red", -5.8, -10.7, 40, index * 0.1, 100)
        store.add("red", -5.9, -7.7, 40, index * 0.1, 100)
    groups = store.consolidated(5, 3.5)
    assert len(groups) == 1

