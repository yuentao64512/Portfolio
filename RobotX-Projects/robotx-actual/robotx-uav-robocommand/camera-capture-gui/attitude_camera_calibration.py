#!/usr/bin/env python3
"""
Camera calibration with the FCU's attitude - for the down-facing camera.

From photos of a checkerboard plus the FCU's roll/pitch/yaw and altitude at
the moment each photo was taken, this works out:

  1. the lens - focal length, optical centre, distortion            -> lens
  2. the mounting offset / error - how far the camera's view is
     tilted from straight down, and how far the picture is rotated
     against the drone's nose - and how the FCU's altitude relates
     to the camera's real height above the ground                   -> attitude
  3. how the apparent size of things changes with altitude and
     roll/pitch/yaw, and how wrong a plain "metres per pixel"
     scale is when the drone tilts                                  -> attitude
  4. real ground distances from pixels, after correcting for the
     attitude, altitude, lens and mounting                          -> measure
  5. a document logging the ground footprint (the mapped size of
     the field of view) against altitude and attitude               -> fov-report

Photos come live from the camera (capture - on the Orin, FCU data through
the drone GUI's MAVProxy) or from a flight recorded with `record` in the
drone GUI (from-recording).

Boards (counted in squares, not inner corners):
  A  10 x 7 squares, 4.2 cm x 4.2 cm
  B   7 x 5 squares, 6.0 cm x 5.6 cm - `lens` checks which side is the 6.0 cm one

Steps:
  1. Lens - hold a board in front of the camera at many angles and distances:
       python3 attitude_camera_calibration.py capture --out calibration/sessions/lens1 --board A --no-fcu
       python3 attitude_camera_calibration.py lens calibration/sessions/lens1
  2. Mounting + altitude - lay a board FLAT on the ground, carry or hover the
     drone over it at 1-4 m, tilting it (roll and pitch, both ways, 0-20 deg):
       python3 attitude_camera_calibration.py capture --out calibration/sessions/ground1 --board B --ground --auto 1
       python3 attitude_camera_calibration.py attitude calibration/sessions/ground1
  3. Use it:
       python3 attitude_camera_calibration.py measure --capture calibration/sessions/ground1/img_0003.png --point 100,200 --point 400,220
       python3 attitude_camera_calibration.py fov-report

Frames: camera = OpenCV (x image-right, y image-down, z out of the lens);
body = FRD (x nose, y right wing, z down); world = NED. Attitude is the
FCU's ATTITUDE message (ZYX roll/pitch/yaw).
"""

import argparse
import csv
import itertools
import json
import math
import os
import queue
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

os.environ.setdefault("MAVLINK_DIALECT", "ardupilotmega")  # before pymavlink loads - see heartbeat_link.py

import cv2  # noqa: E402
import numpy as np  # noqa: E402

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CALIB_DIR = HERE / "calibration"
DEFAULT_CALIB = CALIB_DIR / "camera_calibration.json"
DEFAULT_RESULTS = CALIB_DIR / "attitude_results.csv"
DEFAULT_REPORT = CALIB_DIR / "FOV_footprint_log.md"

MIN_LENS_VIEWS = 8
MIN_GROUND_VIEWS = 4
MAX_PNP_ERROR_PX = 2.0  # a board pose worse than this (reprojection RMS) isn't trusted
MIN_RAY_DOWN = 0.02  # a ray must point at least this far down (about 1 deg below the horizon) to reach the ground
ATTITUDE_MAX_AGE_S = 0.5
ALT_MAX_AGE_S = 1.0
YAW_PRIOR_DEG = 5.0  # the picture's rotation about the lens axis is only taken from the data when the tilts spread wider than this
TILT_BINS = [(0, 5), (5, 10), (10, 15), (15, 20), (20, 90)]
HEIGHT_BINS = [(0, 1), (1, 2), (2, 3), (3, 5), (5, 1000)]

CAPTURE_COLUMNS = [
    "file", "time_unix", "source", "board", "ground", "board_height_m", "width", "height", "rotate_deg",
    "roll_deg", "pitch_deg", "yaw_deg", "roll_rate_dps", "pitch_rate_dps", "yaw_rate_dps", "attitude_age_ms",
    "alt_rel_m", "alt_age_ms", "rangefinder_m", "rangefinder_age_ms", "mode", "armed",
]
RESULT_COLUMNS = [
    "file", "board", "roll_deg", "pitch_deg", "yaw_deg", "tilt_fcu_deg", "tilt_camera_true_deg",
    "height_true_m", "alt_rel_m", "rangefinder_vertical_m", "alt_used_m", "apparent_square_px",
    "size_ratio_vs_level", "naive_error_pct", "attitude_model_error_pct", "corrected_error_pct",
    "corrected_span_error_cm", "mount_residual_deg",
]


# --------------------------------------------------------------------------- boards

@dataclass(frozen=True)
class Board:
    name: str
    squares: Tuple[int, int]  # squares along the board's x and y
    square_m: Tuple[float, float]  # one square's size along x and y

    @property
    def pattern(self) -> Tuple[int, int]:
        """Inner corners (cols, rows), as OpenCV wants them."""
        return self.squares[0] - 1, self.squares[1] - 1

    @property
    def is_square(self) -> bool:
        return abs(self.square_m[0] - self.square_m[1]) < 1e-6

    def object_points(self) -> np.ndarray:
        cols, rows = self.pattern
        pts = np.zeros((cols * rows, 3), np.float32)
        pts[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * np.array(self.square_m, np.float32)
        return pts

    def swapped(self) -> "Board":
        return Board(self.name, self.squares, (self.square_m[1], self.square_m[0]))

    def describe(self) -> str:
        return (f"board {self.name}: {self.squares[0]} x {self.squares[1]} squares, "
                f"{self.square_m[0] * 100:.1f} cm along the {self.squares[0]}-square side x "
                f"{self.square_m[1] * 100:.1f} cm along the {self.squares[1]}-square side")

    def to_json(self) -> dict:
        return {"squares": list(self.squares), "square_cm": [round(s * 100, 4) for s in self.square_m]}


DEFAULT_BOARDS = {
    "A": Board("A", (10, 7), (0.042, 0.042)),
    "B": Board("B", (7, 5), (0.060, 0.056)),
}


def parse_board_def(text: str) -> Board:
    """NAME=COLSxROWS:WxH - squares along x and y, one square's size in cm along x and y."""
    try:
        name, spec = text.split("=", 1)
        counts, size = spec.split(":", 1)
        cols, rows = (int(v) for v in counts.lower().split("x"))
        w, h = (float(v) / 100 for v in size.lower().split("x"))
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{text}' - expected NAME=COLSxROWS:WxH, e.g. A=10x7:4.2x4.2")
    return Board(name.strip().upper(), (cols, rows), (w, h))


def boards_for(args, calib: Optional[dict]) -> Dict[str, Board]:
    boards = dict(DEFAULT_BOARDS)
    if calib and calib.get("boards"):  # what `lens` settled on (e.g. which side of B is 6.0 cm)
        for name, spec in calib["boards"].items():
            boards[name] = Board(name, tuple(spec["squares"]), tuple(v / 100 for v in spec["square_cm"]))
    for board in getattr(args, "board_def", None) or []:
        boards[board.name] = board
    return boards


def find_corners(gray: np.ndarray, pattern: Tuple[int, int]) -> Optional[np.ndarray]:
    """Inner corners of a checkerboard as an (N, 2) float32 array, or None."""
    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_ACCURACY)
        if found:
            return corners.reshape(-1, 2).astype(np.float32)
    found, corners = cv2.findChessboardCorners(gray, pattern, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
    if not found:
        return None
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.01))
    return corners.reshape(-1, 2).astype(np.float32)


# --------------------------------------------------------------------------- geometry

def rot_x(a): c, s = math.cos(a), math.sin(a); return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
def rot_y(a): c, s = math.cos(a), math.sin(a); return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
def rot_z(a): c, s = math.cos(a), math.sin(a); return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def body_to_ned(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Rotation body (FRD) -> NED for the FCU's roll/pitch/yaw in radians."""
    return rot_z(yaw) @ rot_y(pitch) @ rot_x(roll)


def down_in_body(roll: float, pitch: float) -> np.ndarray:
    """The true-down direction, in the drone's body frame."""
    return body_to_ned(roll, pitch, 0.0).T @ np.array([0.0, 0.0, 1.0])


# Camera looking straight down with the top of the picture toward the nose:
# image-right = right wing, image-down = toward the tail, lens axis = down.
_CAM_TO_BODY_TOP_AT_NOSE = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def nominal_cam_to_body(image_top_deg: float) -> np.ndarray:
    """Camera -> body rotation for a perfectly mounted down-facing camera whose
    picture-top points image_top_deg clockwise from the nose (seen from above):
    0 nose, 90 right wing, 180 tail, 270 left wing."""
    return rot_z(math.radians(image_top_deg)) @ _CAM_TO_BODY_TOP_AT_NOSE


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def image_outline(size: Tuple[int, int], rotate_deg: float) -> np.ndarray:
    """The camera's real picture area as 4 corners (top-left, top-right,
    bottom-right, bottom-left of the original picture), in the saved frame's
    pixels. With a --rotate that isn't a multiple of 90 the saved frame is a
    bigger canvas with black corners (camera_link.py) - this is the rotated
    picture inside it."""
    canvas_w, canvas_h = size
    rotate_deg = float(rotate_deg or 0.0)
    if rotate_deg % 360 == 0:
        w, h = canvas_w, canvas_h
        matrix = np.array([[1.0, 0, 0], [0, 1.0, 0]])
    else:
        a = math.radians(rotate_deg)
        c, s = abs(math.cos(a)), abs(math.sin(a))
        det = c * c - s * s
        if abs(det) < 0.05:  # near 45 deg the original size can't be recovered - use the whole frame
            w, h = canvas_w, canvas_h
            matrix = np.array([[1.0, 0, 0], [0, 1.0, 0]])
        else:
            w, h = (c * canvas_w - s * canvas_h) / det, (c * canvas_h - s * canvas_w) / det
            matrix = cv2.getRotationMatrix2D((w / 2, h / 2), -rotate_deg, 1.0)
            matrix[0, 2] += canvas_w / 2 - w / 2
            matrix[1, 2] += canvas_h / 2 - h / 2
    corners = np.array([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]], float)
    corners = corners @ matrix[:, :2].T + matrix[:, 2]
    centre = corners.mean(axis=0)
    inward = centre - corners
    return corners + inward / np.linalg.norm(inward, axis=1, keepdims=True)  # 1 px inside the edge


def outline_samples(outline: np.ndarray, per_edge: int) -> np.ndarray:
    pts = []
    for i in range(4):
        a, b = outline[i], outline[(i + 1) % 4]
        for t in np.linspace(0, 1, per_edge, endpoint=False):
            pts.append(a + (b - a) * t)
    return np.array(pts)


def undistort(px: np.ndarray, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
    """Pixels -> normalised, distortion-free image coordinates (x/z, y/z)."""
    pts = np.asarray(px, np.float64).reshape(-1, 1, 2)
    criteria = (cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 50, 1e-10)  # the default 5 steps isn't enough near the edges
    if hasattr(cv2, "undistortPointsIter"):  # OpenCV 4
        out = cv2.undistortPointsIter(pts, K, dist, None, None, criteria)
    else:
        try:  # OpenCV 5 takes the criteria directly
            out = cv2.undistortPoints(pts, K, dist, None, None, None, criteria)
        except (TypeError, cv2.error):
            out = cv2.undistortPoints(pts, K, dist)
    return out.reshape(-1, 2)


class Projector:
    """Pixel -> point on the ground, from the lens, the camera mounting and
    the FCU's attitude and altitude. Also importable by other scripts:

        from attitude_camera_calibration import Projector, load_calib
        proj = Projector.from_calib(load_calib())
        ne = proj.ground([[u, v]], roll_rad, pitch_rad, yaw_rad, proj.height(fcu_alt_m, "rangefinder"))

    (rangefinder = the vertical height from it, i.e. reading x cos(roll) x cos(pitch).)
    """

    def __init__(self, K, dist, size, rotate_deg=0.0, cam_to_body=None, alt_scale=1.0, alt_offset_m=0.0, alt_source=None,
                 alt_fits=None):
        self.K = np.asarray(K, float)
        self.dist = np.asarray(dist, float).ravel()
        self.size = tuple(int(v) for v in size)
        self.rotate_deg = float(rotate_deg or 0.0)
        self.cam_to_body = nominal_cam_to_body(0.0) if cam_to_body is None else np.asarray(cam_to_body, float)
        self.alt_scale = float(alt_scale)
        self.alt_offset_m = float(alt_offset_m)
        self.alt_source = alt_source
        self.alt_fits = dict(alt_fits or {})  # every altitude source's own correction (attitude's altitude_fits)

    @classmethod
    def from_calib(cls, calib: dict, corrected: bool = True, image_top_deg: Optional[float] = None) -> "Projector":
        mount = calib.get("mount") if corrected else None
        if image_top_deg is not None:
            cam_to_body = nominal_cam_to_body(image_top_deg)
        elif mount:
            cam_to_body = mount["cam_to_body"]
        else:
            cam_to_body = nominal_cam_to_body(calib.get("image_top_deg", 0.0))
        alt = calib.get("altitude") if corrected else None
        return cls(calib["camera_matrix"], calib["dist_coeffs"], calib["image_size"], calib.get("rotate_deg", 0.0),
                   cam_to_body, alt["scale"] if alt else 1.0, alt["offset_m"] if alt else 0.0,
                   alt["source"] if alt else None, calib.get("altitude_fits") if corrected else None)

    @property
    def focal_px(self) -> float:
        return float((self.K[0, 0] + self.K[1, 1]) / 2)

    def has_correction(self, source: Optional[str]) -> bool:
        return source is None or source == self.alt_source or source in self.alt_fits

    def height(self, fcu_alt_m: float, source: Optional[str] = None) -> float:
        """Camera height above the ground from an FCU altitude, with that
        source's calibrated scale/offset applied (source None = the main one
        the calibration chose). A source with no calibration is used as-is."""
        if source is None or source == self.alt_source:
            return self.alt_scale * fcu_alt_m + self.alt_offset_m
        fit = self.alt_fits.get(source)
        return fit["scale"] * fcu_alt_m + fit["offset_m"] if fit else fcu_alt_m

    def rays(self, px, roll: float, pitch: float, yaw: float) -> np.ndarray:
        norm = undistort(px, self.K, self.dist)
        cam = np.column_stack([norm, np.ones(len(norm))])
        return cam @ (body_to_ned(roll, pitch, yaw) @ self.cam_to_body).T

    def ground(self, px, roll: float, pitch: float, yaw: float, height_m: float) -> np.ndarray:
        """(N, 2) north/east metres from the point straight below the camera;
        NaN for a pixel that looks at or above the horizon."""
        rays = self.rays(px, roll, pitch, yaw)
        out = np.full((len(rays), 2), np.nan)
        hits = rays[:, 2] > MIN_RAY_DOWN * np.linalg.norm(rays, axis=1)
        out[hits] = rays[hits, :2] * (height_m / rays[hits, 2])[:, None]
        return out

    def outline(self) -> np.ndarray:
        return image_outline(self.size, self.rotate_deg)


def to_body_heading(ne: np.ndarray, yaw: float) -> np.ndarray:
    """North/east -> forward/right of the drone's heading."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack([ne[:, 0] * c + ne[:, 1] * s, -ne[:, 0] * s + ne[:, 1] * c])


# --------------------------------------------------------------------------- files

def fnum(row: dict, key: str) -> Optional[float]:
    try:
        value = row.get(key)
        return None if value in (None, "") else float(value)
    except ValueError:
        return None


def load_calib(path: Path = DEFAULT_CALIB, required: bool = True) -> Optional[dict]:
    if not Path(path).is_file():
        if required:
            sys.exit(f"No calibration at {path} - run the `lens` step first (or pass --calib).")
        return None
    return json.loads(Path(path).read_text(encoding="utf-8"))


def save_calib(path: Path, calib: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(calib, indent=2), encoding="utf-8")
    os.replace(tmp, path)


def load_captures(paths: List[Path]) -> List[dict]:
    """Rows of every session's captures.csv, each with _path (the image)."""
    rows = []
    for path in paths:
        path = Path(path)
        csv_path = path / "captures.csv" if path.is_dir() else path
        if not csv_path.is_file():
            print(f"  (no captures.csv in {path} - skipped)")
            continue
        with open(csv_path, newline="", encoding="utf-8") as f:
            for row in csv.DictReader(f):
                image = csv_path.parent / row["file"]
                if image.is_file():
                    row["_path"] = image
                    rows.append(row)
    return rows


class SessionWriter:
    """A capture session: numbered PNGs (lossless) + captures.csv. Appends to
    an existing session."""

    def __init__(self, folder: Path):
        self.folder = Path(folder)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.folder / "captures.csv"
        self.count = len(load_captures([self.folder])) if self.csv_path.is_file() else 0
        existing = sorted(self.folder.glob("img_*.png"))
        self._next = max([int(p.stem[4:]) for p in existing if p.stem[4:].isdigit()] or [0]) + 1

    def save(self, frame: np.ndarray, row: dict) -> str:
        name = f"img_{self._next:04d}.png"
        self._next += 1
        cv2.imwrite(str(self.folder / name), frame)
        new = not self.csv_path.is_file()
        with open(self.csv_path, "a", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CAPTURE_COLUMNS, extrasaction="ignore")
            if new:
                writer.writeheader()
            writer.writerow(dict(row, file=name))
        self.count += 1
        return name


def different_enough(corners, state, last) -> bool:
    """For automatic captures: only keep a photo that adds something - the
    board moved in the picture, or the attitude/altitude changed."""
    if last is None:
        return True
    last_corners, last_state, min_move_px = last
    # centre and spread rather than corner-by-corner: the detector may number
    # the same board's corners the other way round between frames
    moved = np.linalg.norm(corners.mean(axis=0) - last_corners.mean(axis=0))
    resized = np.abs(corners.std(axis=0) - last_corners.std(axis=0)).max()
    if float(moved) > min_move_px or float(resized) > min_move_px / 2:
        return True
    for key, step in (("roll_deg", 2.0), ("pitch_deg", 2.0), ("yaw_deg", 5.0), ("alt_rel_m", 0.2), ("rangefinder_m", 0.1)):
        a, b = fnum(state, key), fnum(last_state, key)
        if a is not None and b is not None and abs(a - b) >= step:
            return True
    return False


# --------------------------------------------------------------------------- capture (live)

class FcuReader:
    """Latest attitude/altitude/rangefinder from the FCU, through the drone
    GUI's heartbeat code on a MAVProxy output."""

    TYPES = {"ATTITUDE", "GLOBAL_POSITION_INT", "DISTANCE_SENSOR", "RANGEFINDER"}

    def __init__(self, connection: str):
        sys.path.insert(0, str(ROOT / "drone-heartbeat-gui"))
        from heartbeat_link import HeartbeatMonitor
        from pymavlink import mavutil
        self.mavlink = mavutil.mavlink
        self.monitor = HeartbeatMonitor(connection)
        self._lock = threading.Lock()
        self._latest = {}
        self.monitor.add_message_listener(self._on_message)
        self.monitor.start()
        m = self.mavlink
        self.rates = {m.MAVLINK_MSG_ID_ATTITUDE: 20, m.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 10,
                      m.MAVLINK_MSG_ID_DISTANCE_SENSOR: 10, m.MAVLINK_MSG_ID_RANGEFINDER: 10}
        # put back afterwards: the drone GUI's / listener's rates; 0 = the FCU's default
        self.restore = {m.MAVLINK_MSG_ID_ATTITUDE: 10, m.MAVLINK_MSG_ID_GLOBAL_POSITION_INT: 5,
                        m.MAVLINK_MSG_ID_DISTANCE_SENSOR: 0, m.MAVLINK_MSG_ID_RANGEFINDER: 0}
        self._rates_sent = []

    def _on_message(self, msg) -> None:
        msg_type = msg.get_type()
        if msg_type not in self.TYPES:
            return
        _, system_id, _ = self.monitor.active_link()
        if system_id is not None and msg.get_srcSystem() != system_id:
            return
        if msg_type == "DISTANCE_SENSOR" and msg.orientation != self.mavlink.MAV_SENSOR_ROTATION_PITCH_270:
            return  # only a downward rangefinder
        with self._lock:
            self._latest[msg_type] = (msg, time.time())

    def _send_rates(self, rates: dict) -> bool:
        master, system_id, component_id = self.monitor.active_link()
        if master is None or system_id is None:
            return False
        for message_id, hz in rates.items():
            interval = 0 if hz == 0 else 1_000_000.0 / hz
            try:
                master.mav.command_long_send(system_id, component_id or 1, self.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                             0, message_id, interval, 0, 0, 0, 0, 0)
            except Exception:
                return False
        return True

    def tick(self) -> None:
        """Ask for the stream rates once connected, and once more 2 s later."""
        if not self.monitor.snapshot().connected:
            return
        now = time.time()
        if not self._rates_sent or (len(self._rates_sent) == 1 and now - self._rates_sent[0] > 2.0):
            if self._send_rates(self.rates):
                self._rates_sent.append(now)

    def state(self) -> dict:
        now = time.time()
        with self._lock:
            latest = dict(self._latest)
        row = {}
        if "ATTITUDE" in latest:
            m, t = latest["ATTITUDE"]
            if now - t <= ATTITUDE_MAX_AGE_S:
                row.update(roll_deg=f"{math.degrees(m.roll):.3f}", pitch_deg=f"{math.degrees(m.pitch):.3f}",
                           yaw_deg=f"{math.degrees(m.yaw):.3f}", roll_rate_dps=f"{math.degrees(m.rollspeed):.3f}",
                           pitch_rate_dps=f"{math.degrees(m.pitchspeed):.3f}",
                           yaw_rate_dps=f"{math.degrees(m.yawspeed):.3f}", attitude_age_ms=f"{(now - t) * 1000:.0f}")
        if "GLOBAL_POSITION_INT" in latest:
            m, t = latest["GLOBAL_POSITION_INT"]
            if now - t <= ALT_MAX_AGE_S:
                row.update(alt_rel_m=f"{m.relative_alt / 1000:.3f}", alt_age_ms=f"{(now - t) * 1000:.0f}")
        for msg_type in ("DISTANCE_SENSOR", "RANGEFINDER"):
            if msg_type in latest:
                m, t = latest[msg_type]
                if now - t <= ALT_MAX_AGE_S:
                    dist = m.current_distance / 100 if msg_type == "DISTANCE_SENSOR" else m.distance
                    row.update(rangefinder_m=f"{dist:.3f}", rangefinder_age_ms=f"{(now - t) * 1000:.0f}")
                    break
        snap = self.monitor.snapshot()
        if snap.connected:
            row.update(mode=snap.mode_name or "", armed=int(snap.armed))
        return row

    def stop(self) -> None:
        if self._rates_sent:
            self._send_rates(self.restore)
        self.monitor.stop()


def capture_problem(corners, state, args, need_fcu: bool) -> Optional[str]:
    if corners is None:
        return "board not found in the picture"
    if need_fcu:
        if "roll_deg" not in state:
            return "no attitude from the FCU"
        if args.ground and "alt_rel_m" not in state and "rangefinder_m" not in state:
            return "no altitude from the FCU"
        rates = [abs(fnum(state, k) or 0.0) for k in ("roll_rate_dps", "pitch_rate_dps", "yaw_rate_dps")]
        if max(rates) > args.max_rate_dps:
            return f"turning too fast ({max(rates):.1f} deg/s > {args.max_rate_dps}) - hold still"
    return None


def cmd_capture(args) -> None:
    board = boards_for(args, None).get(args.board)
    if board is None:
        sys.exit(f"Unknown board '{args.board}'")
    if args.ground and args.no_fcu:
        sys.exit("--ground needs the FCU's attitude and altitude - drop --no-fcu")
    sys.path.insert(0, str(HERE))
    from camera_link import CameraLink

    session = SessionWriter(args.out)
    cam = CameraLink(args.device, session.folder, rotation_deg=args.rotate)
    cam.start()
    fcu = None if args.no_fcu else FcuReader(args.connection)

    commands = queue.Queue()

    def read_stdin():
        for line in sys.stdin:
            commands.put(line.strip().lower())

    threading.Thread(target=read_stdin, daemon=True).start()
    print(f"Session {session.folder} ({session.count} photos already) - {board.describe()}")
    print("  board FLAT ON THE GROUND (mounting/altitude photos)" if args.ground
          else "  lens photos: hold the board at many angles and distances, fill different parts of the picture")
    print("  Enter = take a photo now" + (f", and automatically every {args.auto:g} s when the board is seen" if args.auto else "")
          + "; q + Enter (or Ctrl+C) = finish")

    last = None
    last_auto = last_check = last_print = 0.0
    corners = None
    problem = "starting"
    try:
        while True:
            try:
                command = commands.get(timeout=0.1)
            except queue.Empty:
                command = None
            if command in ("q", "quit", "exit"):
                break
            now = time.time()
            if fcu:
                fcu.tick()
            frame = cam.get_latest_frame()
            if frame is None:
                if now - last_print > 2:
                    status = cam.status()
                    print(f"  waiting for the camera... {status.last_error or ''}")
                    last_print = now
                continue
            state = fcu.state() if fcu else {}
            manual = command is not None
            auto_due = bool(args.auto) and now - last_auto >= args.auto
            if manual or auto_due or now - last_check >= 1.0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                corners = find_corners(gray, board.pattern)
                last_check = now
                problem = capture_problem(corners, state, args, fcu is not None)
            if manual or auto_due:
                if auto_due:
                    last_auto = now
                if problem is None and (manual or different_enough(corners, state, last)):
                    row = dict(state, time_unix=f"{now:.3f}", source="live", board=board.name, ground=int(args.ground),
                               board_height_m=f"{args.board_height_m:.3f}", width=frame.shape[1], height=frame.shape[0],
                               rotate_deg=f"{args.rotate:g}")
                    name = session.save(frame, row)
                    last = (corners, state, args.min_move * math.hypot(frame.shape[1], frame.shape[0]))
                    print(f"  saved {name} ({session.count} in session)  roll {state.get('roll_deg', '-')} "
                          f"pitch {state.get('pitch_deg', '-')} yaw {state.get('yaw_deg', '-')} "
                          f"alt {state.get('alt_rel_m', '-')} rng {state.get('rangefinder_m', '-')}")
                elif manual:
                    print(f"  not saved: {problem or 'same view as the last photo'}")
            if now - last_print >= 2.0:
                print(f"  [{'BOARD SEEN' if corners is not None else 'no board'}] "
                      + (f"roll {state.get('roll_deg', '-')} pitch {state.get('pitch_deg', '-')} "
                         f"yaw {state.get('yaw_deg', '-')} alt {state.get('alt_rel_m', '-')} "
                         f"rng {state.get('rangefinder_m', '-')} " if fcu else "")
                      + f"| {session.count} saved" + (f" | {problem}" if problem else ""))
                last_print = now
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        if fcu:
            fcu.stop()
    print(f"Done: {session.count} photos in {session.folder}")


# --------------------------------------------------------------------------- from a flight recording

def cmd_from_recording(args) -> None:
    board = boards_for(args, None).get(args.board)
    if board is None:
        sys.exit(f"Unknown board '{args.board}'")
    frames_csv = args.frames_csv or args.video.with_name(args.video.stem + "_frames.csv")
    if not frames_csv.is_file():
        sys.exit(f"No flight log {frames_csv} (pass --frames-csv)")
    with open(frames_csv, newline="", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    cap = cv2.VideoCapture(str(args.video))
    if not cap.isOpened():
        sys.exit(f"Can't open {args.video}")
    session = SessionWriter(args.out)
    last = None
    next_time = 0.0
    index = 0
    looked = 0
    while index < len(rows):
        ok, frame = cap.read()
        if not ok:
            break
        row = rows[index]
        index += 1
        t = fnum(row, "video_time_s") or 0.0
        if t < next_time or row.get("repeat") == "1" or fnum(row, "roll_deg") is None:
            continue
        rates = [abs(fnum(row, k) or 0.0) for k in ("roll_rate_dps", "pitch_rate_dps", "yaw_rate_dps")]
        if max(rates) > args.max_rate_dps:
            continue
        if args.ground and fnum(row, "alt_rel_home_m") is None:
            continue
        looked += 1
        corners = find_corners(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), board.pattern)
        if corners is None:
            continue
        state = {"roll_deg": row.get("roll_deg"), "pitch_deg": row.get("pitch_deg"), "yaw_deg": row.get("yaw_deg"),
                 "alt_rel_m": row.get("alt_rel_home_m")}
        if not different_enough(corners, state, last):
            continue
        next_time = t + args.every
        capture = dict(state, time_unix=row.get("orin_time_unix", ""), source=f"{args.video.name}#{row.get('frame')}",
                       board=board.name, ground=int(args.ground), board_height_m=f"{args.board_height_m:.3f}",
                       width=frame.shape[1], height=frame.shape[0], rotate_deg=f"{args.rotate:g}",
                       roll_rate_dps=row.get("roll_rate_dps"), pitch_rate_dps=row.get("pitch_rate_dps"),
                       yaw_rate_dps=row.get("yaw_rate_dps"), attitude_age_ms=row.get("attitude_age_ms"),
                       alt_age_ms=row.get("global_pos_age_ms"), mode=row.get("mode"), armed=row.get("armed"))
        session.save(frame, capture)
        last = (corners, state, args.min_move * math.hypot(frame.shape[1], frame.shape[0]))
    print(f"Checked {looked} steady frames, saved {session.count} photos with the board to {session.folder}")


# --------------------------------------------------------------------------- lens

def calibrate(views, boards: Dict[str, Board], size):
    obj = [boards[name].object_points() for name, _, _ in views]
    img = [corners.reshape(-1, 1, 2) for _, corners, _ in views]
    rms, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj, img, size, None, None)
    per_view = []
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        proj, _ = cv2.projectPoints(o, r, t, K, dist)
        per_view.append(float(np.sqrt(np.mean(np.sum((proj - i) ** 2, axis=2)))))
    return rms, K, dist, per_view


def fov_degrees(K, dist, size, rotate_deg) -> dict:
    o = image_outline(size, rotate_deg)
    norm = undistort(np.array([(o[0] + o[3]) / 2, (o[1] + o[2]) / 2, (o[0] + o[1]) / 2, (o[2] + o[3]) / 2,
                               o[0], o[2], o[1], o[3]]), K, dist)
    rays = np.column_stack([norm, np.ones(len(norm))])
    rays /= np.linalg.norm(rays, axis=1, keepdims=True)

    def angle(a, b):
        return math.degrees(math.acos(float(np.clip(rays[a] @ rays[b], -1, 1))))

    return {"horizontal": round(angle(0, 1), 2), "vertical": round(angle(2, 3), 2),
            "diagonal": round(max(angle(4, 5), angle(6, 7)), 2)}


def cmd_lens(args) -> None:
    old = load_calib(args.calib, required=False)
    boards = boards_for(args, None)
    captures = load_captures(args.sessions)
    views, size, rotations, sessions = [], None, set(), set()
    for c in captures:
        board = boards.get(c.get("board", ""))
        if board is None:
            print(f"  {c['_path'].name}: unknown board '{c.get('board')}' - skipped")
            continue
        gray = cv2.imread(str(c["_path"]), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        this_size = (gray.shape[1], gray.shape[0])
        if size is None:
            size = this_size
        elif this_size != size:
            print(f"  {c['_path'].name}: {this_size[0]}x{this_size[1]}, not {size[0]}x{size[1]} - skipped")
            continue
        corners = find_corners(gray, board.pattern)
        if corners is None:
            print(f"  {c['_path'].name}: board not found - skipped")
            continue
        views.append((board.name, corners, c["_path"]))
        rotations.add(float(c.get("rotate_deg") or 0.0))
        sessions.add(str(c["_path"].parent))
    if len(views) < MIN_LENS_VIEWS:
        sys.exit(f"Only {len(views)} usable photos - need at least {MIN_LENS_VIEWS} (15-30 is better).")
    if len(rotations) > 1:
        sys.exit(f"The photos were taken with different --rotate values {sorted(rotations)} - calibrate each separately.")
    rotate = rotations.pop()

    # Which side of a non-square board is which isn't always known - try both
    # and keep the one the lens model fits best.
    used = sorted({name for name, _, _ in views})
    choices = [[boards[n]] if boards[n].is_square or args.keep_board_sides else [boards[n], boards[n].swapped()] for n in used]
    best = None
    for combo in itertools.product(*choices):
        trial = dict(boards, **{b.name: b for b in combo})
        rms, K, dist, per_view = calibrate(views, trial, size)
        if len(choices) > 1 or any(len(c) > 1 for c in choices):
            print(f"  {'; '.join(b.describe() for b in combo if not b.is_square) or 'square boards'}: RMS {rms:.3f} px")
        if best is None or rms < best[0]:
            best = (rms, K, dist, per_view, trial)
    rms, K, dist, per_view, boards = best

    median = float(np.median(per_view))
    keep = [i for i, e in enumerate(per_view) if e <= max(3 * median, 1.0)]
    if len(keep) < len(views) and len(keep) >= MIN_LENS_VIEWS:
        for i in range(len(views)):
            if i not in keep:
                print(f"  dropped {views[i][2].name}: {per_view[i]:.2f} px off (blurred or a bad detection)")
        views = [views[i] for i in keep]
        rms, K, dist, per_view = calibrate(views, boards, size)

    calib = {
        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
        "image_size": list(size),
        "rotate_deg": rotate,
        "camera_matrix": K.tolist(),
        "dist_coeffs": dist.ravel().tolist(),
        "rms_px": round(float(rms), 4),
        "views": len(views),
        "views_per_board": {n: sum(1 for v in views if v[0] == n) for n in used},
        "boards": {n: boards[n].to_json() for n in used},
        "fov_deg": fov_degrees(K, dist, size, rotate),
        "lens_sessions": sorted(sessions),
    }
    if old and old.get("image_size") == calib["image_size"]:
        for key in ("mount", "altitude", "image_top_deg", "attitude_sessions"):
            if key in old:
                calib[key] = old[key]
        if "mount" in old:
            print("  kept the earlier mounting/altitude calibration - re-run `attitude` to update it for the new lens values")
    save_calib(args.calib, calib)

    quality = "good" if rms < 0.5 else "OK" if rms < 1.0 else "POOR - retake with sharper photos, more angles"
    print(f"\nLens calibrated from {len(views)} photos ({calib['views_per_board']}): RMS {rms:.3f} px ({quality})")
    print(f"  image {size[0]}x{size[1]}, focal {K[0, 0]:.1f} / {K[1, 1]:.1f} px, centre {K[0, 2]:.1f}, {K[1, 2]:.1f}")
    print(f"  distortion {np.round(dist.ravel(), 4).tolist()}")
    print(f"  field of view {calib['fov_deg']['horizontal']} x {calib['fov_deg']['vertical']} deg "
          f"(diagonal {calib['fov_deg']['diagonal']})")
    for n in used:
        if not boards[n].is_square:
            print(f"  {boards[n].describe()}  <- best fit")
    print(f"  saved {args.calib}")


# --------------------------------------------------------------------------- attitude: mounting, altitude, size

def fit_mount(down_cam: np.ndarray, down_body: np.ndarray, nominal_cb: np.ndarray) -> Tuple[np.ndarray, dict]:
    """Body -> camera rotation that best maps the FCU's down direction onto
    the down direction the camera actually saw (board normal). Rotation about
    the lens axis is only measurable when the attitudes spread; it is held
    near the nominal value otherwise (a weak prior)."""
    n = len(down_cam)
    lam = n * math.sin(math.radians(YAW_PRIOR_DEG)) ** 2
    R = nominal_cb.copy()
    for _ in range(30):
        pred = down_body @ R.T
        A = np.vstack([-skew(p) for p in pred])
        b = (down_cam - pred).ravel()
        total, _ = cv2.Rodrigues(R @ nominal_cb.T)
        H = A.T @ A
        g = A.T @ b
        H_reg = H + np.diag([0.0, 0.0, lam])
        g[2] -= lam * float(total[2, 0])
        step = np.linalg.solve(H_reg, g)
        R = cv2.Rodrigues(step.reshape(3, 1))[0] @ R
        if np.linalg.norm(step) < 1e-10:
            break
    pred = down_body @ R.T
    residual = np.degrees(np.arccos(np.clip(np.sum(pred * down_cam, axis=1), -1, 1)))
    A = np.vstack([-skew(p) for p in pred])
    H = A.T @ A
    sigma2 = float(np.sum((down_cam - pred) ** 2)) / max(1, 2 * n - 3)
    cov = sigma2 * np.linalg.inv(H + np.diag([0.0, 0.0, lam]))
    schur = H[2, 2] - H[2, :2] @ np.linalg.solve(H[:2, :2], H[:2, 2])
    stats = {"residual_deg": residual, "sigma_deg": np.degrees(np.sqrt(np.diag(cov))), "rotation_measured": bool(schur > 4 * lam)}
    return R, stats


def describe_mount(cam_to_body: np.ndarray, image_top_deg: float) -> dict:
    axis = cam_to_body[:, 2]  # lens axis in the body frame
    top = cam_to_body @ np.array([0.0, -1.0, 0.0])  # picture-top direction in the body frame
    top_deg = math.degrees(math.atan2(top[1], top[0])) % 360
    return {
        "tilt_total_deg": round(math.degrees(math.acos(float(np.clip(axis[2], -1, 1)))), 3),
        "tilt_forward_deg": round(math.degrees(math.atan2(axis[0], axis[2])), 3),
        "tilt_right_deg": round(math.degrees(math.atan2(axis[1], axis[2])), 3),
        "image_top_deg": round(top_deg, 3),
        "image_rotation_error_deg": round(wrap_deg(top_deg - image_top_deg), 3),
    }


def fit_altitude(fcu: np.ndarray, true: np.ndarray, source: str) -> Optional[dict]:
    if len(fcu) < 2:
        return None
    if len(fcu) >= 3 and float(np.ptp(fcu)) >= 0.5:
        scale, offset = np.polyfit(fcu, true, 1)
    else:  # heights too close together to tell a scale error - offset only
        scale, offset = 1.0, float(np.mean(true - fcu))
    rms = float(np.sqrt(np.mean((scale * fcu + offset - true) ** 2)))
    return {"source": source, "scale": round(float(scale), 5), "offset_m": round(float(offset), 4),
            "rms_m": round(rms, 4), "samples": int(len(fcu)), "range_m": [round(float(fcu.min()), 3), round(float(fcu.max()), 3)]}


def grid_error(ground: np.ndarray, board: Board) -> Tuple[float, float]:
    """Mean square-size error (%) of a board projected onto the ground, and
    the error (cm) of its corner-to-corner span."""
    cols, rows = board.pattern
    g = ground.reshape(rows, cols, 2)
    if np.isnan(g).any():
        return float("nan"), float("nan")
    along_x = float(np.mean(np.linalg.norm(np.diff(g, axis=1), axis=2)))
    along_y = float(np.mean(np.linalg.norm(np.diff(g, axis=0), axis=2)))
    err = (abs(along_x / board.square_m[0] - 1) + abs(along_y / board.square_m[1] - 1)) / 2 * 100
    span_true = math.hypot((cols - 1) * board.square_m[0], (rows - 1) * board.square_m[1])
    span = float(np.linalg.norm(g[-1, -1] - g[0, 0]))
    return err, (span - span_true) * 100


def cmd_attitude(args) -> None:
    calib = load_calib(args.calib)
    boards = boards_for(args, calib)
    K = np.array(calib["camera_matrix"])
    dist = np.array(calib["dist_coeffs"])
    size = tuple(calib["image_size"])
    f_px = float((K[0, 0] + K[1, 1]) / 2)
    image_top = args.image_top_deg if args.image_top_deg is not None else calib.get("image_top_deg", 0.0)

    samples, skipped = [], {}

    def skip(reason):
        skipped[reason] = skipped.get(reason, 0) + 1

    for c in load_captures(args.sessions):
        if c.get("ground") != "1":
            skip("not a board-on-the-ground photo")
            continue
        roll, pitch, yaw = (fnum(c, k) for k in ("roll_deg", "pitch_deg", "yaw_deg"))
        if roll is None or pitch is None:
            skip("no attitude")
            continue
        board = boards.get(c.get("board", ""))
        gray = cv2.imread(str(c["_path"]), cv2.IMREAD_GRAYSCALE)
        if board is None or gray is None or (gray.shape[1], gray.shape[0]) != size:
            skip("unknown board or wrong image size")
            continue
        corners = find_corners(gray, board.pattern)
        if corners is None:
            skip("board not found")
            continue
        obj = board.object_points()
        ok, rvec, tvec = cv2.solvePnP(obj, corners, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            skip("board pose failed")
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        if float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - corners) ** 2, axis=1)))) > MAX_PNP_ERROR_PX:
            skip("board pose too inaccurate")
            continue
        R = cv2.Rodrigues(rvec)[0]
        normal = R[:, 2]
        s = float(normal @ tvec.ravel())
        board_h = fnum(c, "board_height_m") or 0.0
        roll_r, pitch_r = math.radians(roll), math.radians(pitch)
        rng = fnum(c, "rangefinder_m")
        alt = fnum(c, "alt_rel_m")
        samples.append({
            "file": c["_path"].name, "board": board, "corners": corners,
            "roll": roll_r, "pitch": pitch_r, "yaw": math.radians(yaw or 0.0),
            "height_true": abs(s), "down_cam": normal * math.copysign(1.0, s),
            "alt_rel": None if alt is None else alt - board_h,
            "rng": None if rng is None else rng * math.cos(roll_r) * math.cos(pitch_r) - board_h,
        })
    if skipped:
        print("Skipped: " + ", ".join(f"{n} {why}" for why, n in skipped.items()))
    if len(samples) < MIN_GROUND_VIEWS:
        sys.exit(f"Only {len(samples)} usable board-on-the-ground photos - need at least {MIN_GROUND_VIEWS} (15+ is better).")

    # ---- altitude: the FCU's value vs the camera's real height above the board
    fits = {}
    for source, key in (("rangefinder", "rng"), ("alt_rel", "alt_rel")):
        have = [s for s in samples if s[key] is not None]
        fits[source] = fit_altitude(np.array([s[key] for s in have]), np.array([s["height_true"] for s in have]), source)
    source = args.alt_source
    if source == "auto":
        n_rng = sum(1 for s in samples if s["rng"] is not None)
        source = "rangefinder" if n_rng >= 0.6 * len(samples) and fits["rangefinder"] else "alt_rel"
    key = "rng" if source == "rangefinder" else "alt_rel"
    alt_fit = fits[source]
    if alt_fit is None:
        sys.exit(f"Not enough photos with a {source} altitude")
    samples = [s for s in samples if s[key] is not None]

    # ---- mounting
    nominal_cb = nominal_cam_to_body(image_top).T
    down_cam = np.array([s["down_cam"] for s in samples])
    down_body = np.array([down_in_body(s["roll"], s["pitch"]) for s in samples])
    R_cb, mstats = fit_mount(down_cam, down_body, nominal_cb)
    cam_to_body = R_cb.T
    mount = describe_mount(cam_to_body, image_top)
    tilts = np.degrees(np.arccos(np.clip(down_body[:, 2], -1, 1)))

    # ---- distances: a plain metres-per-pixel scale, the attitude model with
    # a perfect mount and the raw FCU altitude, and the fully corrected model
    # (each photo checked with corrections fitted on the OTHER photos).
    rows = []
    for i, s in enumerate(samples):
        board = s["board"]
        fcu_alt = s[key]
        px_grid = s["corners"].reshape(board.pattern[1], board.pattern[0], 2)
        apparent = float((np.mean(np.linalg.norm(np.diff(px_grid, axis=1), axis=2)) +
                          np.mean(np.linalg.norm(np.diff(px_grid, axis=0), axis=2))) / 2)
        square = sum(board.square_m) / 2
        naive_err, _ = grid_error(s["corners"].astype(float) * (fcu_alt / f_px), board)
        raw = Projector(K, dist, size, calib.get("rotate_deg", 0), nominal_cb.T)
        raw_err, _ = grid_error(raw.ground(s["corners"], s["roll"], s["pitch"], s["yaw"], fcu_alt), board)
        others = [j for j in range(len(samples)) if j != i]
        if len(others) >= 3:
            R_loo, _ = fit_mount(down_cam[others], down_body[others], nominal_cb)
            fit_loo = fit_altitude(np.array([samples[j][key] for j in others]),
                                   np.array([samples[j]["height_true"] for j in others]), source)
        else:
            R_loo, fit_loo = R_cb, alt_fit
        corrected = Projector(K, dist, size, calib.get("rotate_deg", 0), R_loo.T, fit_loo["scale"], fit_loo["offset_m"])
        cor_err, span_err = grid_error(corrected.ground(s["corners"], s["roll"], s["pitch"], s["yaw"],
                                                        corrected.height(fcu_alt)), board)
        rows.append({
            "file": s["file"], "board": board.name, "roll_deg": round(math.degrees(s["roll"]), 2),
            "pitch_deg": round(math.degrees(s["pitch"]), 2), "yaw_deg": round(math.degrees(s["yaw"]), 2),
            "tilt_fcu_deg": round(float(tilts[i]), 2),
            "tilt_camera_true_deg": round(math.degrees(math.acos(float(np.clip(s["down_cam"][2], -1, 1)))), 2),
            "height_true_m": round(s["height_true"], 3),
            "alt_rel_m": "" if s["alt_rel"] is None else round(s["alt_rel"], 3),
            "rangefinder_vertical_m": "" if s["rng"] is None else round(s["rng"], 3),
            "alt_used_m": round(fcu_alt, 3), "apparent_square_px": round(apparent, 2),
            "size_ratio_vs_level": round(apparent / (f_px * square / s["height_true"]), 4),
            "naive_error_pct": round(naive_err, 2), "attitude_model_error_pct": round(raw_err, 2),
            "corrected_error_pct": round(cor_err, 2), "corrected_span_error_cm": round(span_err, 2),
            "mount_residual_deg": round(float(mstats["residual_deg"][i]), 3),
        })

    Path(args.results).parent.mkdir(parents=True, exist_ok=True)
    with open(args.results, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=RESULT_COLUMNS)
        writer.writeheader()
        writer.writerows(rows)

    # ---- report
    sig = mstats["sigma_deg"]
    print(f"\nUsed {len(samples)} photos, tilt {tilts.min():.1f}-{tilts.max():.1f} deg, "
          f"true height {min(s['height_true'] for s in samples):.2f}-{max(s['height_true'] for s in samples):.2f} m")
    print("\nMOUNTING OFFSET / ERROR (camera vs the FCU's axes)")
    print(f"  lens axis tilted {mount['tilt_total_deg']:.2f} deg from straight down: "
          f"{mount['tilt_forward_deg']:+.2f} deg toward the nose, {mount['tilt_right_deg']:+.2f} deg toward the right wing "
          f"(+/- {2 * max(sig[0], sig[1]):.2f}, 95%)")
    if mstats["rotation_measured"]:
        print(f"  picture top points {mount['image_top_deg']:.1f} deg clockwise from the nose - "
              f"{mount['image_rotation_error_deg']:+.2f} deg from the expected {image_top:g} (+/- {2 * sig[2]:.2f}, 95%)")
    else:
        print(f"  picture rotation: not measurable from these photos (tilts too similar) - assumed {image_top:g} deg "
              f"(picture top toward the nose = 0); tilt the drone more, both ways, to measure it")
    print(f"  fit residual {np.sqrt(np.mean(mstats['residual_deg'] ** 2)):.2f} deg RMS "
          f"(worst {mstats['residual_deg'].max():.2f}) - includes any FCU level error (do the accel calibration first)")
    print("\nALTITUDE: FCU vs the camera's real height above the board")
    for name, fit in fits.items():
        if fit:
            mark = "  <- used" if name == source else ""
            print(f"  {name:12s} true = {fit['scale']:.4f} x FCU {fit['offset_m']:+.3f} m, "
                  f"{fit['rms_m'] * 100:.1f} cm RMS, {fit['samples']} photos{mark}")
    if not 0.8 <= alt_fit["scale"] <= 1.2:
        print("  WARNING: the altitude scale is far from 1 - check the altitude source")

    def summary(col):
        vals = np.array([r[col] for r in rows if not math.isnan(r[col])])
        return f"{np.mean(vals):5.2f}% mean, {np.max(vals):5.2f}% worst" if len(vals) else "n/a"

    print("\nGROUND DISTANCE ERROR (board squares measured from the picture vs their real size)")
    print(f"  plain metres-per-pixel (alt / focal), no attitude:  {summary('naive_error_pct')}")
    print(f"  attitude + altitude, perfect mount, raw FCU alt:     {summary('attitude_model_error_pct')}")
    print(f"  attitude + altitude + mount/alt corrections:         {summary('corrected_error_pct')}"
          f"   (each photo checked with corrections fitted on the others)")
    near_level = [(r, s) for r, s in zip(rows, samples) if r["tilt_camera_true_deg"] < 8] or list(zip(rows, samples))
    focal_eff = [r["apparent_square_px"] * r["height_true_m"] / (sum(s["board"].square_m) / 2) for r, s in near_level]
    print(f"\nSIZE vs ALTITUDE: apparent size x height / real size = {np.mean(focal_eff):.1f} +/- {np.std(focal_eff):.1f} px "
          f"over {len(focal_eff)} near-level photos (lens focal {f_px:.1f} px) - apparent size = focal x size / height")
    print("SIZE vs ATTITUDE (apparent size / size seen level at the same height; error of each method):")
    print("  tilt      n   size ratio   plain m/px   attitude   corrected")
    for lo, hi in TILT_BINS:
        sel = [r for r in rows if lo <= r["tilt_fcu_deg"] < hi]
        if sel:
            print(f"  {lo:2d}-{hi:<3d}  {len(sel):3d}   {np.mean([r['size_ratio_vs_level'] for r in sel]):9.3f}   "
                  f"{np.nanmean([r['naive_error_pct'] for r in sel]):8.2f}%   {np.nanmean([r['attitude_model_error_pct'] for r in sel]):7.2f}%   "
                  f"{np.nanmean([r['corrected_error_pct'] for r in sel]):7.2f}%")
    print(f"\nPer-photo results: {args.results}")

    if args.dry_run:
        print("(--dry-run: calibration file not changed)")
        return
    calib["image_top_deg"] = image_top
    calib["mount"] = dict(mount, cam_to_body=cam_to_body.tolist(), photos=len(samples),
                          tilt_range_deg=[round(float(tilts.min()), 2), round(float(tilts.max()), 2)],
                          sigma_deg=[round(float(v), 3) for v in sig], rotation_measured=mstats["rotation_measured"],
                          residual_rms_deg=round(float(np.sqrt(np.mean(mstats["residual_deg"] ** 2))), 3))
    calib["altitude"] = alt_fit
    calib["altitude_fits"] = {k: v for k, v in fits.items() if v}
    calib["attitude_sessions"] = sorted({str(Path(p)) for p in args.sessions})
    calib["attitude_updated"] = time.strftime("%Y-%m-%d %H:%M:%S")
    save_calib(args.calib, calib)
    print(f"Saved mounting + altitude corrections to {args.calib}")


# --------------------------------------------------------------------------- measure

def parse_point(text: str) -> Tuple[float, float]:
    try:
        u, v = (float(x) for x in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError(f"'{text}' - expected U,V pixels, e.g. 320,240")
    return u, v


def cmd_measure(args) -> None:
    calib = load_calib(args.calib)
    proj = Projector.from_calib(calib, corrected=not args.uncorrected, image_top_deg=args.image_top_deg)
    roll, pitch, yaw, alt = args.roll, args.pitch, args.yaw, args.alt
    source = args.alt_source or proj.alt_source or "alt_rel"
    if args.capture:
        rows = [r for r in load_captures([args.capture.parent]) if r["_path"].name == args.capture.name]
        if not rows:
            sys.exit(f"{args.capture} isn't in its folder's captures.csv")
        row = rows[0]
        roll, pitch, yaw = (fnum(row, k) for k in ("roll_deg", "pitch_deg", "yaw_deg"))
        if source == "rangefinder" and fnum(row, "rangefinder_m") is not None:
            alt = fnum(row, "rangefinder_m") * math.cos(math.radians(roll)) * math.cos(math.radians(pitch))
        else:
            source, alt = "alt_rel", fnum(row, "alt_rel_m")
        alt = None if alt is None else alt - (fnum(row, "board_height_m") or 0.0)
    if roll is None or pitch is None or (alt is None and args.height is None):
        sys.exit("Need the attitude and altitude: --capture IMAGE, or --roll --pitch [--yaw] and --alt (or --height)")
    yaw = yaw or 0.0
    height = args.height if args.height is not None else proj.height(alt, source) - args.ground_height_m
    r, p, y = math.radians(roll), math.radians(pitch), math.radians(yaw)
    corrected = not args.uncorrected and proj.alt_source and proj.has_correction(source)
    print(f"roll {roll:.2f}  pitch {pitch:.2f}  yaw {yaw:.2f} deg; camera {height:.3f} m above the ground"
          + ("" if args.height is not None else f" (FCU {source} {alt:.3f} m"
             + (", corrected)" if corrected else ", uncorrected)")))
    points = np.array(args.point, float)
    if len(points) == 0:
        sys.exit("Give at least one --point U,V")
    ne = proj.ground(points, r, p, y, height)
    fr = to_body_heading(ne, y)
    for (u, v), (n, e), (fwd, right) in zip(points, ne, fr):
        if math.isnan(n):
            print(f"  ({u:.0f},{v:.0f}): at or above the horizon - not on the ground")
        else:
            print(f"  ({u:.0f},{v:.0f}): {fwd:+.3f} m forward, {right:+.3f} m right of the drone "
                  f"(N {n:+.3f}, E {e:+.3f}), {math.hypot(n, e):.3f} m from the point below it")
    total = 0.0
    for i in range(1, len(ne)):
        d = float(np.linalg.norm(ne[i] - ne[i - 1]))
        total += d
        print(f"  point {i} -> {i + 1}: {d:.3f} m")
    if len(ne) > 2:
        print(f"  total path: {total:.3f} m")


# --------------------------------------------------------------------------- FOV footprint document

def footprint(proj: Projector, roll: float, pitch: float, height: float, per_edge: int = 40) -> dict:
    """Ground footprint of the whole picture (angles in radians, yaw 0 so
    north = the drone's nose)."""
    border = outline_samples(proj.outline(), per_edge)
    g = proj.ground(border, roll, pitch, 0.0, height)
    if np.isnan(g).any() or np.abs(g).max() > 200 * max(height, 0.1):
        return {"bounded": False}
    x, y = g[:, 0], g[:, 1]
    area = 0.5 * abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1))))
    centre = proj.outline().mean(axis=0)
    c = proj.ground(np.array([centre, centre + (1, 0), centre + (0, 1)]), roll, pitch, 0.0, height)
    gsd_c = (np.linalg.norm(c[1] - c[0]) + np.linalg.norm(c[2] - c[0])) / 2
    gx = proj.ground(border + (1, 0), roll, pitch, 0.0, height)
    gy = proj.ground(border + (0, 1), roll, pitch, 0.0, height)
    gsd_max = float(np.nanmax(np.maximum(np.linalg.norm(gx - g, axis=1), np.linalg.norm(gy - g, axis=1))))
    return {"bounded": True, "across_m": float(np.ptp(y)), "along_m": float(np.ptp(x)), "area_m2": area,
            "reach_m": float(np.max(np.hypot(x, y))), "gsd_centre_cm": float(gsd_c) * 100, "gsd_worst_cm": gsd_max * 100,
            "fwd_min": float(x.min()), "fwd_max": float(x.max()), "right_min": float(y.min()), "right_max": float(y.max())}


def lean_deg(roll_deg: float, pitch_deg: float) -> float:
    """Total tilt from level for a roll and pitch (deg)."""
    c = math.cos(math.radians(roll_deg)) * math.cos(math.radians(pitch_deg))
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def parse_list(text: str) -> List[float]:
    return [float(v) for v in text.split(",") if v.strip()]


def fmt(v, digits=1):
    return "—" if v is None else f"{v:.{digits}f}"


def cmd_fov_report(args) -> None:
    calib = load_calib(args.calib)
    proj = Projector.from_calib(calib, corrected=True, image_top_deg=args.image_top_deg)
    heights = parse_list(args.heights)
    limits = parse_list(args.tilts)
    step = args.grid_step
    max_limit = max(limits)
    grid = np.arange(-max_limit, max_limit + 1e-9, step)

    table = {}  # (height, roll, pitch) -> footprint
    for h in heights:
        for r in grid:
            for p in grid:
                table[(h, round(r, 3), round(p, 3))] = footprint(proj, math.radians(r), math.radians(p), h)

    def envelope(h, limit, only=None):
        """Largest footprint numbers for any attitude whose lean angle (roll and
        pitch combined, what ArduPilot's ANGLE_MAX limits) is within limit
        (only='roll'/'pitch': that axis alone)."""
        best = {"bounded": True}
        for (hh, r, p), fp in table.items():
            if hh != h or lean_deg(r, p) > limit + 1e-6:
                continue
            if (only == "roll" and p != 0) or (only == "pitch" and r != 0):
                continue
            if not fp["bounded"]:
                return {"bounded": False}
            for k in ("across_m", "along_m", "area_m2", "reach_m", "gsd_worst_cm"):
                if fp[k] > best.get(k, -1):
                    best[k] = fp[k]
                    if k == "reach_m":
                        best["at"] = (r, p)
        return best

    def cell(fp, fn):
        return "horizon in view" if not fp["bounded"] else fn(fp)

    mount = calib.get("mount")
    alt = calib.get("altitude")
    K = np.array(calib["camera_matrix"])
    L = []
    L.append("# Camera FOV footprint log")
    L.append("")
    L.append(f"Generated {time.strftime('%Y-%m-%d %H:%M')} by `attitude_camera_calibration.py fov-report` from "
             f"`{Path(args.calib).name}` (lens calibrated {calib.get('created', '?')}"
             + (f", mounting/altitude {calib.get('attitude_updated')}" if mount else "") + "). "
             "Re-run after any recalibration or change to the camera mount.")
    L.append("")
    L.append("The **footprint** is the patch of flat ground the whole picture covers. Sizes are across (left-right of "
             "the drone's heading) × along (nose-tail). **Reach** is the farthest ground point in the picture from the "
             "point straight below the drone. **GSD** (ground sample distance) is the ground size of one pixel — at the "
             "picture centre, and the worst (largest) anywhere in the picture.")
    L.append("")
    L.append("## Camera")
    L.append("")
    L.append("| | |")
    L.append("|---|---|")
    L.append(f"| Image | {calib['image_size'][0]} × {calib['image_size'][1]} px"
             + (f", rotated {calib.get('rotate_deg'):g}°" if calib.get("rotate_deg") else "") + " |")
    L.append(f"| Focal length | {K[0, 0]:.1f} / {K[1, 1]:.1f} px |")
    L.append(f"| Field of view | {calib['fov_deg']['horizontal']}° × {calib['fov_deg']['vertical']}° "
             f"(diagonal {calib['fov_deg']['diagonal']}°) |")
    L.append(f"| Lens fit | {calib['rms_px']} px RMS from {calib['views']} photos |")
    if mount:
        L.append(f"| Mounting offset | lens axis {mount['tilt_total_deg']:.2f}° off vertical: "
                 f"{mount['tilt_forward_deg']:+.2f}° toward the nose, {mount['tilt_right_deg']:+.2f}° toward the right wing "
                 f"(± {2 * max(mount['sigma_deg'][:2]):.2f}°, 95%) |")
        L.append(f"| Picture rotation | top points {mount['image_top_deg']:.1f}° clockwise from the nose"
                 + (f" ({mount['image_rotation_error_deg']:+.2f}° from expected, ± {2 * mount['sigma_deg'][2]:.2f}°, 95%)"
                    if mount.get("rotation_measured") else " (assumed — not measured)") + " |")
    else:
        L.append("| Mounting offset | **not calibrated** — perfect mount assumed (run `attitude`) |")
    if alt:
        L.append(f"| Altitude | camera height = {alt['scale']:.4f} × FCU {alt['source']} {alt['offset_m']:+.3f} m "
                 f"({alt['rms_m'] * 100:.1f} cm RMS) |")
    else:
        L.append("| Altitude | FCU altitude used as the camera height, uncorrected |")
    L.append("")
    L.append("## Level flight (roll = pitch = 0)")
    L.append("")
    L.append("| Height (m) | Across (m) | Along (m) | Area (m²) | Reach (m) | GSD centre (cm/px) | GSD worst (cm/px) |")
    L.append("|---|---|---|---|---|---|---|")
    for h in heights:
        fp = footprint(proj, 0.0, 0.0, h)
        L.append(f"| {h:g} | " + (" | ".join([fmt(fp['across_m'], 2), fmt(fp['along_m'], 2), fmt(fp['area_m2'], 1),
                                                  fmt(fp['reach_m'], 2), fmt(fp['gsd_centre_cm'], 2), fmt(fp['gsd_worst_cm'], 2)])
                                      if fp["bounded"] else "horizon in view | | | | |") + " |")
    L.append("")
    L.append("## Maximum mapped size vs attitude and altitude")
    L.append("")
    L.append(f"Largest value for **any** attitude whose lean angle — roll and pitch combined, cos(lean) = cos(roll)·cos(pitch), "
             f"the angle ArduPilot's `ANGLE_MAX` limits — is within the limit (roll/pitch searched every {step:g}°; yaw has "
             "no effect on size). Use these as the worst case for a flight that stays within that lean angle.")
    for title, key, digits in (("Footprint across × along (m)", None, 1), ("Footprint area (m²)", "area_m2", 1),
                               ("Reach from the point below the drone (m)", "reach_m", 1),
                               ("Worst GSD in the picture (cm/px)", "gsd_worst_cm", 2)):
        L.append("")
        L.append(f"### {title}")
        L.append("")
        L.append("| Height (m) | " + " | ".join(f"±{t:g}°" for t in limits) + " |")
        L.append("|---" * (len(limits) + 1) + "|")
        for h in heights:
            cells = []
            for t in limits:
                env = envelope(h, t)
                cells.append(cell(env, (lambda e: f"{e['across_m']:.{digits}f} × {e['along_m']:.{digits}f}") if key is None
                                  else (lambda e, k=key: f"{e[k]:.{digits}f}")))
            L.append(f"| {h:g} | " + " | ".join(cells) + " |")
    for axis in ("roll", "pitch"):
        L.append("")
        L.append(f"### {axis.capitalize()} only — across × along (m), larger of ± {axis}")
        L.append("")
        L.append("| Height (m) | " + " | ".join(f"{t:g}°" for t in limits) + " |")
        L.append("|---" * (len(limits) + 1) + "|")
        for h in heights:
            L.append(f"| {h:g} | " + " | ".join(cell(envelope(h, t, only=axis), lambda e: f"{e['across_m']:.1f} × {e['along_m']:.1f}")
                                              for t in limits) + " |")
    L.append("")
    L.append("**Yaw** turns the footprint on the ground but doesn't change its size or any distance in it; it only "
             "matters for turning forward/right offsets into north/east (which `measure` does).")
    L.append("")
    L.append("## How size relates to altitude and attitude")
    L.append("")
    L.append(f"- Level: an object of size S at height h looks f·S/h pixels big (f = {proj.focal_px:.1f} px), so its "
             "apparent size halves when the height doubles, and GSD = h / f.")
    L.append("- Tilted by θ (roll and pitch together: cos θ = cos roll · cos pitch): the picture centre is seen from "
             "h / cos θ away, so things there look about cos θ smaller across the tilt and cos² θ smaller along it; "
             "the far side of the footprint stretches fastest and reaches the horizon once θ + half the FOV ≥ 90°.")
    L.append("- A plain metres-per-pixel scale (h / f) is therefore only right for a level drone looking at the "
             "picture centre; `measure` and the tables above trace each pixel's ray through the lens, the mounting "
             "and the FCU attitude down to the ground instead.")

    results = Path(args.results)
    if results.is_file():
        with open(results, newline="", encoding="utf-8") as f:
            measured = list(csv.DictReader(f))
        if measured:
            L.append("")
            L.append("## Measured during calibration")
            L.append("")
            L.append(f"From `{results.name}`: board squares measured from each photo vs their real size, after "
                     "correcting for attitude and altitude (each photo checked with corrections fitted on the others).")
            L.append("")
            L.append("| Photo | Height (m) | Roll (°) | Pitch (°) | Yaw (°) | Footprint across × along (m) | "
                     "Size vs level | Plain m/px error | Corrected error | Span error (cm) |")
            L.append("|---|---|---|---|---|---|---|---|---|---|")
            for r in measured:
                h = fnum(r, "height_true_m")
                fp = footprint(proj, math.radians(fnum(r, "roll_deg")), math.radians(fnum(r, "pitch_deg")), h)
                L.append(f"| {r['file']} | {h:.2f} | {r['roll_deg']} | {r['pitch_deg']} | {r['yaw_deg']} | "
                         + cell(fp, lambda e: f"{e['across_m']:.2f} × {e['along_m']:.2f}")
                         + f" | {r['size_ratio_vs_level']} | {r['naive_error_pct']}% | {r['corrected_error_pct']}% | "
                         f"{r['corrected_span_error_cm']} |")

    for frames_csv in args.frames_csv or []:
        with open(frames_csv, newline="", encoding="utf-8") as f:
            frames = [r for i, r in enumerate(csv.DictReader(f)) if i % args.every_n == 0]
        bins = {}
        for r in frames:
            roll, pitch, a = fnum(r, "roll_deg"), fnum(r, "pitch_deg"), fnum(r, "alt_rel_home_m")
            if roll is None or pitch is None or a is None:
                continue
            h = proj.height(a, "alt_rel")
            if h <= 0.2:
                continue
            fp = footprint(proj, math.radians(roll), math.radians(pitch), h, per_edge=20)
            b = bins.setdefault(int(h), {"n": 0, "max": None, "unbounded": 0, "tilt": 0.0})
            b["n"] += 1
            b["tilt"] = max(b["tilt"], lean_deg(roll, pitch))
            if not fp["bounded"]:
                b["unbounded"] += 1
            elif b["max"] is None or fp["area_m2"] > b["max"]["area_m2"]:
                b["max"] = dict(fp, roll=roll, pitch=pitch, h=h)
        L.append("")
        L.append(f"## Flight log: {Path(frames_csv).name}")
        L.append("")
        L.append(f"Every {args.every_n}th video frame of a `record` flight, grouped by height. Largest footprint seen "
                 "in each 1 m band, with the attitude at that moment.")
        L.append("")
        L.append("| Height band (m) | Frames | Max tilt (°) | Largest footprint across × along (m) | Area (m²) | "
                 "Reach (m) | at roll / pitch (°) | Horizon in view |")
        L.append("|---|---|---|---|---|---|---|---|")
        for h in sorted(bins):
            b = bins[h]
            m = b["max"]
            L.append(f"| {h}–{h + 1} | {b['n']} | {b['tilt']:.1f} | "
                     + (f"{m['across_m']:.1f} × {m['along_m']:.1f} | {m['area_m2']:.1f} | {m['reach_m']:.1f} | "
                        f"{m['roll']:.1f} / {m['pitch']:.1f}" if m else "— | — | — | —")
                     + f" | {b['unbounded']} |")

    L.append("")
    L.append("## Field test log")
    L.append("")
    L.append("Fill in when checking the footprint in the field (e.g. two marks a known distance apart on the ground, "
             "measured with `measure`).")
    L.append("")
    L.append("| Date | Height (m) | Roll (°) | Pitch (°) | Yaw (°) | Predicted footprint (m) | Measured (m) | Notes |")
    L.append("|---|---|---|---|---|---|---|---|")
    for _ in range(6):
        L.append("| | | | | | | | |")
    L.append("")

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text("\n".join(L), encoding="utf-8")
    csv_path = out.with_suffix(".csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["height_m", "roll_deg", "pitch_deg", "bounded", "across_m", "along_m", "area_m2", "reach_m",
                         "forward_min_m", "forward_max_m", "right_min_m", "right_max_m", "gsd_centre_cm", "gsd_worst_cm"])
        for (h, r, p), fp in sorted(table.items()):
            if fp["bounded"]:
                writer.writerow([h, r, p, 1] + [round(fp[k], 4) for k in ("across_m", "along_m", "area_m2", "reach_m",
                                                                           "fwd_min", "fwd_max", "right_min", "right_max",
                                                                           "gsd_centre_cm", "gsd_worst_cm")])
            else:
                writer.writerow([h, r, p, 0] + [""] * 10)
    print(f"Wrote {out}\n  and every height/roll/pitch combination to {csv_path}")
    if not mount:
        print("  note: no mounting calibration yet - the tables assume a perfect mount (run `attitude`)")


# --------------------------------------------------------------------------- main

def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    def common(p, calib=True):
        p.add_argument("--board-def", action="append", type=parse_board_def,
                       help="add or change a board: NAME=COLSxROWS:WxH (squares, cm), e.g. B=7x5:6.0x5.6")
        if calib:
            p.add_argument("--calib", type=Path, default=DEFAULT_CALIB, help="calibration file (default: %(default)s)")

    p = sub.add_parser("capture", help="take calibration photos live, with the FCU's attitude/altitude")
    common(p, calib=False)
    p.add_argument("--out", type=Path, required=True, help="session folder (created; added to if it exists)")
    p.add_argument("--board", default="A", help="which board is in the picture: A (10x7, 4.2 cm) or B (7x5, 6.0x5.6 cm)")
    p.add_argument("--ground", action="store_true", help="the board lies flat on the ground (mounting/altitude photos)")
    p.add_argument("--board-height-m", type=float, default=0.0,
                   help="height of the board above the take-off ground, e.g. on a box (default 0)")
    p.add_argument("--auto", type=float, default=0.0, help="also take a photo every this many seconds when the board is seen")
    p.add_argument("--min-move", type=float, default=0.04,
                   help="auto: skip a photo unless the board moved this fraction of the picture diagonal or the attitude changed")
    p.add_argument("--max-rate-dps", type=float, default=5.0, help="don't save while turning faster than this (deg/s)")
    p.add_argument("--device", default="0", help="camera index or path (default 0)")
    p.add_argument("--rotate", type=float, default=0.0,
                   help="rotate frames like the drone GUI's --camera-rotate (use the SAME value as in flight)")
    p.add_argument("--connection", default="udp:127.0.0.1:14552",
                   help="MAVProxy output for the FCU (default %(default)s - the drone GUI's spare output; "
                        "don't run the lawnmower mission or param_set.py at the same time)")
    p.add_argument("--no-fcu", action="store_true", help="no FCU data (lens photos only)")
    p.set_defaults(func=cmd_capture)

    p = sub.add_parser("from-recording", help="make a session from a `record` flight (video + frames CSV)")
    common(p, calib=False)
    p.add_argument("video", type=Path)
    p.add_argument("--frames-csv", type=Path, help="default: <video>_frames.csv")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--board", default="A")
    p.add_argument("--ground", action="store_true", help="the board lies flat on the ground")
    p.add_argument("--board-height-m", type=float, default=0.0)
    p.add_argument("--every", type=float, default=0.5, help="at most one photo per this many seconds of video")
    p.add_argument("--min-move", type=float, default=0.04)
    p.add_argument("--max-rate-dps", type=float, default=5.0)
    p.add_argument("--rotate", type=float, default=0.0, help="the --rotate the recording was made with")
    p.set_defaults(func=cmd_from_recording)

    p = sub.add_parser("lens", help="calibrate the lens from capture sessions")
    common(p)
    p.add_argument("sessions", type=Path, nargs="+", help="session folders")
    p.add_argument("--keep-board-sides", action="store_true",
                   help="don't test which side of a non-square board is which - use the sizes exactly as given")
    p.set_defaults(func=cmd_lens)

    p = sub.add_parser("attitude", help="mounting offset, altitude and size/distance relations from board-on-ground sessions")
    common(p)
    p.add_argument("sessions", type=Path, nargs="+")
    p.add_argument("--image-top-deg", type=float, default=None,
                   help="where the top of the picture points, clockwise from the nose: 0 nose (default), 90 right, 180 tail, 270 left")
    p.add_argument("--alt-source", choices=["auto", "rangefinder", "alt_rel"], default="auto",
                   help="FCU altitude to calibrate (auto: rangefinder if most photos have one)")
    p.add_argument("--results", type=Path, default=DEFAULT_RESULTS, help="per-photo results CSV (default: %(default)s)")
    p.add_argument("--dry-run", action="store_true", help="report only, don't update the calibration file")
    p.set_defaults(func=cmd_attitude)

    p = sub.add_parser("measure", help="real ground distances between pixels, corrected for attitude and altitude")
    common(p)
    p.add_argument("--capture", type=Path, help="a session photo (its attitude/altitude are used)")
    p.add_argument("--roll", type=float, help="roll (deg)")
    p.add_argument("--pitch", type=float, help="pitch (deg)")
    p.add_argument("--yaw", type=float, help="yaw (deg, default 0)")
    p.add_argument("--alt", type=float, help="FCU altitude (m), from --alt-source")
    p.add_argument("--alt-source", choices=["rangefinder", "alt_rel"], default=None,
                   help="which FCU altitude --alt is (rangefinder = its vertical height); default: the one the calibration chose")
    p.add_argument("--height", type=float, help="camera height above the ground (m), used as-is")
    p.add_argument("--ground-height-m", type=float, default=0.0, help="height of the measured surface above take-off ground")
    p.add_argument("--point", type=parse_point, action="append", default=[], help="pixel U,V (repeat)")
    p.add_argument("--uncorrected", action="store_true", help="ignore the mounting/altitude corrections")
    p.add_argument("--image-top-deg", type=float, default=None, help="override the mount with a perfect one at this angle")
    p.set_defaults(func=cmd_measure)

    p = sub.add_parser("fov-report", help="write the FOV footprint document")
    common(p)
    p.add_argument("--out", type=Path, default=DEFAULT_REPORT, help="document to write (default: %(default)s)")
    p.add_argument("--heights", default="1,2,3,5,7.5,10,15,20", help="heights in metres (default %(default)s)")
    p.add_argument("--tilts", default="0,5,10,15,20,25,30,35", help="tilt limits in degrees (default %(default)s)")
    p.add_argument("--grid-step", type=float, default=5.0, help="roll/pitch search step in degrees (default %(default)s)")
    p.add_argument("--results", type=Path, default=DEFAULT_RESULTS, help="attitude results to include, if present")
    p.add_argument("--frames-csv", type=Path, action="append", help="a `record` flight log to summarise (repeat)")
    p.add_argument("--every-n", type=int, default=5, help="use every Nth frame of a flight log (default %(default)s)")
    p.add_argument("--image-top-deg", type=float, default=None, help="override the mount with a perfect one at this angle")
    p.set_defaults(func=cmd_fov_report)

    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
