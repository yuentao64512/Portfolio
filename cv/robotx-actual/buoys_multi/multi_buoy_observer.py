#!/usr/bin/env python3
"""multi_buoy_observer: OUR buoy observer - every buoy of a frame, each with a colour (v0.1.30, ROADMAP WP8c).

The students' actual_buoy_observer.py (cv/buoys/, a verbatim read-only snapshot) segments every buoy, measures each with the ZED
depth, then keeps only the nearest; its model has no colour. This program imports their functions unchanged (measure_mask,
buoy_geolocation, open_zed, PixhawkTelemetry, discover_mavlink) and runs the same loop, but keeps every measured buoy (nearest
first, at most --max-buoys) and classifies each one's colour from its own pixels (multi_core.classify_bgr). It writes the same
files into the same place (runs/<stamp>/latest_status.json, buoy_observations.csv one row per buoy, latest_preview.jpg with each
buoy's box and colour), so cv_publish follows it like the single observer; latest_status.json carries `buoys` (all) and
`measurement` (the nearest, the old key). Read-only like theirs: never arms, never commands.

v0.1.40 (ROADMAP WP9b), the recorder and the replay (the pure part in rec_core.py):
  --record DIR [--label L] [--max-min N]   also record: DIR/<stamp>_<label>/cam.svo2 (the ZED SDK's H.264 recording, at the loop's
                                           frame rate) + frames.jsonl (per frame: t, frame ids, the boat's lat / lon / heading, the
                                           buoys with colours and boxes) + session.json (label, start, versions, why it stopped).
                                           The recording stops at --max-min or below --min-free-gb (5) free; detecting goes on.
                                           scripts/agx_record.sh start LABEL | stop | status runs this in tmux (the OCS page's card).
  --svo FILE                               the same loop from a recorded file (no camera, no Pixhawk: the telemetry comes from the
                                           frames.jsonl beside it); the output goes to <the file's folder>/replay/<stamp>/ (never
                                           where cv_publish looks), with its own frames.jsonl -> rec_core.py compare.

  cv/buoys_multi/run.sh --headless [--max-buoys 6] [--min-share 0.45] [--sat-min 0.35] [observer options...]
  cv/buoys_multi/run.sh --svo ~/Desktop/dm3/recordings/<session>/cam.svo2 --headless
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import platform
import signal
import socket
import sys
import time
from dataclasses import asdict, replace
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(1, str(HERE.parent / "buoys"))           # the students' module, imported read-only

import cv2  # noqa: E402
import numpy as np  # noqa: E402
import pyzed.sl as sl  # noqa: E402
from ultralytics import YOLO  # noqa: E402

import actual_buoy_observer as S  # noqa: E402  (their measure_mask, buoy_geolocation, open_zed, PixhawkTelemetry, ...)
import multi_core as M  # noqa: E402
import rec_core as R  # noqa: E402  (v0.1.40: the recorder's pure part)
import two_stage_core as TS  # noqa: E402  (v0.1.78.18: the colleague's Model A, boxes -> our contract)
import guard_core as GC  # noqa: E402  (v0.1.80, WP36: the ZED proximity guard - this process holds the ZED)
import led_core as LC  # noqa: E402  (v0.1.91: the horizon cut, the LED colour under the hat, the blink memory)
from bg_writer import BgWriter  # noqa: E402  (v0.1.86.7, WP43 Tier 1.2: the slow writes off the loop)

BGR = {"red": (0, 0, 255), "green": (0, 200, 0), "blue": (255, 120, 0), "yellow": (0, 220, 255), "black": (60, 60, 60),
       "white": (255, 255, 255), "unknown": (200, 200, 200)}


def parse_args(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--weights", default=S.WEIGHTS)
    p.add_argument("--model", default=None, help="v0.1.34.2: a CV_TRAIN model folder (cv/models/<name>/, cv/sync_cvtrain.sh) - its weights "
                   "(the .engine on the AGX), input size, conf and iou; only its `buoy` class is used. Overrides --weights")
    p.add_argument("--output", default=None, help="where cv_publish looks (AGX_OBSERVER_LOGS; default cv/buoys/runs; with --svo: "
                   "<the file's folder>/replay/<stamp>)")
    p.add_argument("--mavlink", default=S.DEFAULT_MAVLINK, help="pymavlink endpoint for the pose (as the students' observer); 'auto' = USB")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--resolution", choices=["hd2k", "hd1080", "hd720", "vga"], default="hd2k")
    p.add_argument("--fps", type=int, default=15)
    p.add_argument("--depth-mode", choices=["neural", "neural_light", "ultra"], default="neural")
    p.add_argument("--conf", type=float, default=None, help="detection confidence (default: the model's input.yaml, else 0.60)")
    p.add_argument("--imgsz", type=int, default=None, help="network input size (default: the model's input.yaml, else 768)")
    p.add_argument("--frames", type=int, default=0)
    p.add_argument("--camera-forward-m", type=float, default=0.0)
    p.add_argument("--camera-right-m", type=float, default=0.0)
    p.add_argument("--camera-yaw-deg", type=float, default=0.0)
    p.add_argument("--telemetry-max-age-s", type=float, default=0.75)
    p.add_argument("--max-buoys", type=int, default=6, help="report at most this many per frame (nearest first)")
    p.add_argument("--buoy-radius", type=float, default=M.buoy_radius(), help="v0.1.34: added to the measured surface range along the "
                   "line of sight (the buoy's centre; 0 = the old surface range)")
    p.add_argument("--max-depth", type=float, default=25.0, help="v0.1.34: the depth limit in metres (the students' default 20; 25 = the "
                   "pathing spec's R for this 4 mm lens, decided 2026-11-05; beyond it a detection is a bearing only)")
    p.add_argument("--bearing-conf", type=float, default=None, help="v0.1.34: a detection measure_mask refused is still reported as a "
                   "bearing (range null) at this confidence or more; 1.1 = never (v0.1.34.2: default = --conf, so nothing found is dropped)")
    p.add_argument("--limit-sigma", type=float, default=2.0, help="v0.1.34.2: a range within this many sigma_z of --max-depth is reported "
                   "as a bearing (near_limit) - CV_TRAIN found those ~0.6 m short; 0 = off")
    p.add_argument("--min-share", type=float, default=0.45, help="colour: share of the saturated pixels one hue needs")
    p.add_argument("--sat-min", type=float, default=0.35, help="colour: minimum HSV saturation of a voting pixel")
    p.add_argument("--colour-top", type=float, default=0.5, help="v0.1.93.2: colour: vote over the mask's top share only (1 = the whole mask; "
                   "the markers' blue foam float sits at the bottom)")
    p.add_argument("--colour-min-sat", type=float, default=0.20, help="v0.1.93.2: colour: a colour only when this share of the voting pixels "
                   "is saturated (0 = the old rule; 13 Nov replay: blue masks 54 -> 2 with the top half)")
    p.add_argument("--detector", choices=TS.DETECTORS, default="mask", help="v0.1.78.18: mask = a segmenting model (the students' "
                   "best.pt, a CV_TRAIN folder); model-a = the colleague's Model A (boxes; --weights its .engine / .pt, run.sh finds it); "
                   "v0.1.87: boundary-v14 = the colleague's boundary-v14 (boxes: red / green / blue / unlit boundary buoys, dock, obstacle); "
                   "six-class = their 11 Nov six-class model (red / green / blue / buoy, dock, platform)")
    p.add_argument("--obstacle-classes", default="obstacle", help="v0.1.87, boundary-v14: these classes' boxes, measured with the depth, go "
                   "to the boat as obstacles (the guard's `far` clusters -> rings on its map); 'obstacle,dock' adds the dock (off by default: "
                   "the model calls the numbered white buoys dock); 'none' = off")
    p.add_argument("--obstacle-conf", type=float, default=0.35, help="v0.1.87: an obstacle box's minimum score")
    p.add_argument("--horizon", choices=("frac", "off"), default="frac", help="v0.1.91: drop a box whose bottom (the waterline) is above the "
                   "horizon line - --horizon-frac x the image height (12 Nov: the building / sky boxes); off = keep every box")
    p.add_argument("--horizon-frac", type=float, default=0.45, help="v0.1.91: the horizon line as a fraction of the image height (from the top)")
    p.add_argument("--led", choices=("on", "off"), default="on", help="v0.1.91: the LED under each buoy's hat - its colour (red / green / blue, "
                   "no model) and the blink memory (steady / flashing / off over 4 s) into every buoy record as `led`")
    p.add_argument("--led-burst", type=int, default=0, help="v0.1.92: after each full frame, up to N light-only grabs (no depth, no detector):"
                   " the lamps read at the last boxes - the blink memory at ~the camera's rate (12 Nov: 0.8-2.2 full fps; the rule needs >= 5). 0 = off")
    p.add_argument("--number", choices=("on", "off"), default="off", help="v0.1.94: the marker NUMBER of each near buoy (our fine-tuned "
                   "classifier, cv/buoys_multi/number_core): buoys[i].number = {digit 0-9 | null, p, cls}, one digit per buoy per frame; off by default")
    p.add_argument("--number-max-m", type=float, default=8.0, help="v0.1.94: read the number only within this range (12 Nov: 68/68 right <= 8 m)")
    p.add_argument("--number-min-h", type=float, default=110.0, help="v0.1.94: a bearing-only box (no range) is near when at least this many px "
                   "tall in the full frame (~8-9 m on 12 / 13 Nov)")
    p.add_argument("--number-weights", default=None, help="v0.1.94: the classifier (default cv/buoys_multi/models/number_cls.pt; an .engine on the AGX)")
    p.add_argument("--colour", choices=TS.COLOUR_MODES, default="auto", help="v0.1.78.18, model-a only: auto = Model A's red / green "
                   "LED class, else our HSV vote; model = Model A only; hsv = our HSV vote for every buoy (the backup)")
    p.add_argument("--led-conf", type=float, default=0.5, help="v0.1.78.18: auto takes Model A's LED colour at or above this score")
    p.add_argument("--box-shrink", type=float, default=0.8, help="v0.1.78.18: model-a's mask = the box's inscribed ellipse x this")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--allow-no-pixhawk", action="store_true", help="bench camera test only; no GPS")
    p.add_argument("--record", metavar="DIR", default=None, help="v0.1.40: also record DIR/<stamp>_<label>/ (cam.svo2, frames.jsonl, "
                   "session.json)")
    p.add_argument("--label", default="rec", help="the session's label (letters, digits, _ -; e.g. pool_t1_gate)")
    p.add_argument("--max-min", type=float, default=0.0, help="stop recording after N minutes (0 = until stopped); detecting goes on")
    p.add_argument("--min-free-gb", type=float, default=R.MIN_FREE_GB, help="stop recording (or refuse to start) below this free space")
    p.add_argument("--preview-every", type=int, default=1, help="v0.1.84.8: write latest_preview.jpg (the live feed) every N analysed "
                   "frames (1 = every frame; was 15); 0 = only the first")
    p.add_argument("--preview-width", type=int, default=960, help="v0.1.84.8: the preview's width in px (0 = full size; the stream "
                   "scales to its own --width 640 anyway)")
    p.add_argument("--svo-codec", choices=("h265", "h264"), default="h265", help="v0.1.86.3 (WP47): the SDK's svo2 compression - H.265 "
                   "(falls back to H.264 when the SDK / encoder refuses it)")
    p.add_argument("--svo-bitrate", type=int, default=8000, help="v0.1.86.3 (WP47): the svo2 bitrate in kbit/s (0 = the SDK default; "
                   "11 Nov: the default ran ~19 Mbit/s = 1.3-1.5 GB / 10 min whatever the frame rate; 8000 -> ~0.6 GB / 10 min)")
    p.add_argument("--bg-writer", choices=("on", "off"), default="on", help="v0.1.86.7 (WP43 Tier 1.2): the preview + the pairs written by a "
                   "writer thread (off = in the loop, as before)")
    p.add_argument("--pairs-width", type=int, default=960, help="v0.1.86.3 (WP47): the depth pairs' width in px (0 = full size; the depth "
                   "resized nearest-neighbour, the mm values kept)")
    p.add_argument("--pairs-s", type=float, default=0.0, help="v0.1.84.4: while recording, also save an RGB + depth pair (rgb jpg, depth "
                   "png uint16 mm, a coloured preview) into <session>/pairs/ - the first at once, then every N s (0 = off)")
    p.add_argument("--cannon", choices=("red", "green"), default=None, help="v0.1.61: also measure the cannon's target window of this "
                   "colour (../cannon/aim_cv.py, the students' colour segmentation + this frame's depth) into --cannon-out every frame")
    p.add_argument("--cannon-out", default=None, help="v0.1.61: the target file (default: cv/cannon/cannon_config.json target_file)")
    p.add_argument("--svo", metavar="FILE", default=None, help="v0.1.40: replay a recorded .svo2 through the same loop instead of the camera")
    p.add_argument("--guard", choices=("on", "off"), default="on", help="v0.1.80 (WP36): the ZED proximity guard in latest_status.json "
                   "`guard` (cv_publish -> perception/guard): the levelled cloud, 0.15-1.5 m above the water, < 3 m, +-35 deg of the bow")
    p.add_argument("--h-cam", type=float, default=0.55, help="the camera's height above the water, m (v0.1.92: 0.55 fitted from the water plane in 288 of 12 Nov's depth frames, every session 0.54-0.59; was 1.0, which read the water 2-3 m ahead as a 0.4 m object: SLOW in 286 / 301 frames)")
    p.add_argument("--guard-stop", type=float, default=1.5, help="the guard's STOP range, m")
    p.add_argument("--guard-slow", type=float, default=3.0, help="the guard's SLOW range, m")
    p.add_argument("--guard-map", type=float, default=8.0, help="v0.1.81.1: the guard's `far` clusters (slow .. this range, m) for the boat's "
                   "obstacle map; 0 = off")
    a = p.parse_args(argv)
    if a.detector in TS.BOX_DETECTORS and a.model:
        p.error(f"--detector {a.detector} takes --weights (the model's file), not a CV_TRAIN --model folder")
    try:
        a.obstacle_classes = TS.parse_classes(a.obstacle_classes, a.detector) if a.detector == "boundary-v14" else ()
    except ValueError as exc:
        p.error(str(exc))
    if a.svo and a.record:
        p.error("--svo replays a file, --record records the camera: one or the other")
    if a.svo and not os.path.isfile(a.svo):
        p.error(f"--svo {a.svo}: no such file")
    if (a.max_min or a.label != "rec" or a.pairs_s) and not a.record:
        p.error("--max-min / --label / --pairs-s need --record DIR")
    if a.pairs_s < 0:
        p.error("--pairs-s must be >= 0")
    if a.max_min < 0:
        p.error("--max-min must be >= 0")
    if a.record:
        try:
            R.check_label(a.label)
        except ValueError as exc:
            p.error(str(exc))
    if a.output is None:
        a.output = R.replay_output(a.svo) if a.svo else str(HERE.parent / "buoys" / "runs")
    return a


def open_camera(a):
    """The live ZED through the students' open_zed, or (--svo) the recorded file with the same depth settings."""
    if not a.svo:
        return S.open_zed(a.resolution, a.fps, a.depth_mode, depth_max_m=a.max_depth)
    init = sl.InitParameters()
    init.set_from_svo_file(str(a.svo))
    init.svo_real_time_mode = False                                  # every frame, as fast as the loop goes
    init.coordinate_units = sl.UNIT.METER
    init.coordinate_system = sl.COORDINATE_SYSTEM.IMAGE
    init.depth_mode = {"neural": sl.DEPTH_MODE.NEURAL, "neural_light": sl.DEPTH_MODE.NEURAL_LIGHT, "ultra": sl.DEPTH_MODE.ULTRA}[a.depth_mode]
    init.depth_minimum_distance = 0.30
    init.depth_maximum_distance = float(a.max_depth)
    camera = sl.Camera()
    err = camera.open(init)
    if err != sl.ERROR_CODE.SUCCESS:
        raise RuntimeError(f"Could not open the SVO {a.svo}: {err}")
    return camera


def versions(a, camera, weights, spec) -> dict:
    v = {"kit": R.kit_version(str(HERE.parent.parent)), "observer": "multi_buoy_observer", "python": platform.python_version(),
         "host": socket.gethostname(), "weights": str(weights), "model": spec["name"] if spec else None}
    try:
        v["zed_sdk"] = str(sl.Camera.get_sdk_version())
        info = camera.get_camera_information()
        v.update(camera_model=str(info.camera_model), camera_serial=int(info.serial_number),
                 camera_firmware=int(info.camera_configuration.firmware_version))
    except Exception as exc:                                         # an older SDK names them differently: keep what we have
        v["zed_info_error"] = str(exc)
    return v


def svo_params(svo, codec: str, bitrate_kbps: int):
    """v0.1.86.3 (WP47): the SDK's RecordingParameters for codec h265 | h264 at bitrate_kbps (0 = the SDK default)."""
    mode = sl.SVO_COMPRESSION_MODE.H265 if codec == "h265" else sl.SVO_COMPRESSION_MODE.H264
    if bitrate_kbps > 0:
        try:
            return sl.RecordingParameters(svo, mode, 0, int(bitrate_kbps))      # (video_filename, compression_mode, target_framerate, bitrate)
        except TypeError:                                                       # an older SDK without the bitrate argument
            pass
    return sl.RecordingParameters(svo, mode)


def svo_enable(camera, svo, codec: str, bitrate_kbps: int):
    """enable_recording with the asked codec; H.265 refused -> H.264 (same bitrate). Returns (err, 'h265 @ 8000 kbit/s')."""
    err = camera.enable_recording(svo_params(svo, codec, bitrate_kbps))
    used = codec
    if err != sl.ERROR_CODE.SUCCESS and codec == "h265":
        err = camera.enable_recording(svo_params(svo, "h264", bitrate_kbps))
        used = "h264 (h265 refused)"
    return err, f"{used} @ {bitrate_kbps or 'default'} kbit/s"


def start_recording(a, camera, weights, spec):
    """The session folder + the SDK's H.264 recording into it; None without --record."""
    if not a.record:
        return None
    opts = {k: getattr(a, k) for k in ("resolution", "fps", "depth_mode", "max_depth", "conf", "imgsz", "max_buoys", "buoy_radius",
                                       "mavlink", "max_min", "min_free_gb", "camera_forward_m", "camera_right_m", "camera_yaw_deg")}
    sess = R.Session(a.record, a.label, versions(a, camera, weights, spec), opts)
    err, used = svo_enable(camera, sess.svo, getattr(a, "svo_codec", "h264"), int(getattr(a, "svo_bitrate", 0) or 0))
    print(f"[multi_buoy_observer] svo2 compression {used}", flush=True)
    if err != sl.ERROR_CODE.SUCCESS:
        sess.stop(f"enable_recording failed: {err}")
        raise RuntimeError(f"ZED recording into {sess.svo} failed: {err}")
    print(f"[multi_buoy_observer] RECORDING {sess.dir} (label {sess.label}; max {a.max_min:g} min, stops below {a.min_free_gb:g} GB free)", flush=True)
    return sess


def _sigterm(*_):
    raise KeyboardInterrupt


def stop_recording(camera, sess, reason):
    if sess is None or not sess.active:
        return
    try:
        camera.disable_recording()
    finally:
        sess.stop(reason)
    print(f"[multi_buoy_observer] recording stopped ({reason}): {sess.frames} frames in {sess.dir}", flush=True)


def write_preview(path, frame_bgr, width):
    """v0.1.84.8 (Dmitrii: "our feed during run also slow ... maybe with lower resolution"): the camera stream (source dir) serves this
    file - a small JPEG (width px, 0 = full size), written atomically (tmp + replace: the stream never reads half a picture). ~3-5 ms."""
    img = frame_bgr
    if width and img.shape[1] > width:
        img = cv2.resize(img, (width, int(img.shape[0] * width / img.shape[1])), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 75])
    if ok:
        tmp = str(path) + ".tmp"
        with open(tmp, "wb") as fh:
            fh.write(buf.tobytes())
        os.replace(tmp, str(path))


def save_pair(pairs, frame_bgr, xyz, max_depth_m, extra, width: int = 0):
    """v0.1.84.4: one RGB + depth pair from the same grab (the depth = |z| of the point cloud the detections use, metres -> uint16 mm).
    v0.1.86.3 (WP47): `width` > 0 = both at that width (the RGB area-averaged, the depth nearest-neighbour: the mm values stay real)."""
    f = pairs.names()
    z = np.abs(np.asarray(xyz if xyz.ndim == 2 else xyz[:, :, 2], dtype=np.float32))   # v0.1.86.7: the z plane alone (a copy for the writer)
    if width and frame_bgr.shape[1] > width:
        h = int(round(frame_bgr.shape[0] * width / frame_bgr.shape[1]))
        frame_bgr = cv2.resize(frame_bgr, (width, h), interpolation=cv2.INTER_AREA)
        z = cv2.resize(z, (width, h), interpolation=cv2.INTER_NEAREST)
    ok = np.isfinite(z) & (z > 0)
    mm = np.where(ok, np.clip(z * 1000.0, 0, 65535), 0).astype(np.uint16)
    vis = np.zeros(frame_bgr.shape, np.uint8)
    if ok.any():
        n = np.clip(np.where(ok, z, max_depth_m) / max(0.1, max_depth_m), 0, 1)
        vis = cv2.applyColorMap(((1.0 - n) * 255).astype(np.uint8), cv2.COLORMAP_JET)   # near red .. far blue (as the OCS depth view)
        vis[~ok] = 0
    cv2.imwrite(f["rgb"], frame_bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
    cv2.imwrite(f["depth"], mm)
    cv2.imwrite(f["vis"], vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
    pairs.add(dict(extra, rgb=os.path.basename(f["rgb"]), depth=os.path.basename(f["depth"]), vis=os.path.basename(f["vis"]),
                   depth_valid_pct=round(100.0 * float(ok.mean()), 1), depth_unit="mm"))


def colour_of(frame_bgr, mask, a):
    ys, xs = np.nonzero(mask > 0.5)
    if len(ys) == 0:
        return "unknown", {"n": 0}
    top = float(getattr(a, "colour_top", 1.0) or 1.0)
    if top < 1.0:                                       # v0.1.93.2 (13 Nov run, Dmitrii): the vote over the mask's top part only - the
        y0, y1 = int(ys.min()), int(ys.max())           # markers' light-blue foam float at the waterline voted 22 of 108 masks "blue"
        keep = ys <= y0 + top * (y1 - y0)               # (88 % of their blue pixels in the bottom quarter; top 50 %: 4 left)
        if int(keep.sum()) >= 12:
            ys, xs = ys[keep], xs[keep]
    idx = M.sample_indices(len(ys))
    px = frame_bgr[ys[idx], xs[idx]].tolist()
    msf = float(getattr(a, "colour_min_sat", 0.0) or 0.0)
    colour, shares = M.classify_bgr(px, sat_min=a.sat_min, min_share=a.min_share, min_sat_frac=msf)
    if msf > 0 and colour == "unknown" and float(shares.get("saturated", 0.0)) < msf:
        colour = "white"                                # v0.1.93.2: no colour on it = the white marker (its black diamond kept "white" under
    return colour, shares                               # 60 %); T1 takes an "unknown" buoy as an entry suspect (13 Nov replay: 270 of 805)


def main(argv=None):
    a = parse_args(argv)
    guard = None
    if a.record:                                                     # refuse before touching the camera when the disk is full
        os.makedirs(a.record, exist_ok=True)
        guard = R.Guard(a.record, a.max_min, a.min_free_gb)
        why = guard.start_refusal()
        if why:
            print(f"[multi_buoy_observer] NOT RECORDING: {why}", file=sys.stderr, flush=True)
            raise SystemExit(3)
    replay_tel = R.ReplayTelemetry(os.path.join(os.path.dirname(os.path.abspath(a.svo)), R.FRAMES_NAME)) if a.svo else None
    endpoint = None if a.svo else (S.discover_mavlink() if a.mavlink == "auto" else a.mavlink)
    telemetry = None
    if a.svo:
        print(f"[multi_buoy_observer] REPLAY {a.svo} ({len(replay_tel)} recorded telemetry frames) -> {a.output}", flush=True)
    elif endpoint:
        telemetry = S.PixhawkTelemetry(endpoint, a.baud)
        telemetry.start()
    elif not a.allow_no_pixhawk:
        raise RuntimeError("No Pixhawk endpoint: --mavlink ENDPOINT, or --allow-no-pixhawk for a camera-only bench test")
    run_dir = Path(a.output) / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    spec = M.model_spec(a.model, prefer_engine=platform.machine() == "aarch64") if a.model else None
    weights = spec["weights"] if spec else a.weights
    boxes_only = a.detector in TS.BOX_DETECTORS                      # v0.1.78.18: Model A (1280 / 0.25 / 0.5); v0.1.87: boundary-v14 (1280 / 0.25 / 0.55)
    a.conf = a.conf if a.conf is not None else (spec["conf"] if spec else (0.25 if boxes_only else 0.60))
    a.imgsz = a.imgsz if a.imgsz is not None else (spec["imgsz"] if spec else (1280 if boxes_only else 768))
    a.bearing_conf = a.bearing_conf if a.bearing_conf is not None else a.conf
    iou = spec["iou"] if spec else (TS.V14_DEFAULTS["iou"] if a.detector in ("boundary-v14", "six-class") else (0.5 if boxes_only else 0.7))
    buoy_ids = set(spec["buoy_ids"]) if spec else None              # the students' model: one class, all buoys
    model = YOLO(weights, task="detect" if boxes_only else "segment")
    obst_ids = set()
    if boxes_only:
        TS.check_names(a.detector, model.names)                      # v0.1.87: boundary-v14's six classes, or stop here
        buoy_ids = set(TS.buoy_class_ids(model.names))              # the buoy classes; dock, home, obstacle skipped
        obst_ids = set(TS.class_ids(model.names, a.obstacle_classes))   # v0.1.87: boundary-v14's obstacle boxes -> the boat's map
        print(f"[multi_buoy_observer] {a.detector} (boxes): colour {a.colour}, led-conf {a.led_conf}, box-shrink {a.box_shrink}, "
              f"obstacles {list(a.obstacle_classes) or 'off'} >= {a.obstacle_conf}", flush=True)

    def colour_for(frame_bgr, mask, box):
        """(colour, shares, extra fields) - the HSV vote, or (model-a) Model A's LED class by --colour."""
        if not boxes_only:
            colour, shares = colour_of(frame_bgr, mask, a)
            return colour, shares, {}
        cls_name = str(model.names[int(box.cls.item())])
        hsv, shares = colour_of(frame_bgr, mask, a) if a.colour != "model" else (None, {})
        colour, source = TS.pick_colour(a.colour, cls_name, float(box.conf.item()), hsv, a.led_conf)
        return colour, shares, {"colour_source": source, "model_class": cls_name, "colour_hsv": hsv}
    print(f"[multi_buoy_observer] model {spec['name'] if spec else weights}: conf {a.conf}, imgsz {a.imgsz}, iou {iou}, bearing-conf "
          f"{a.bearing_conf}, buoy classes {sorted(buoy_ids) if buoy_ids is not None else 'all'}", flush=True)
    camera = open_camera(a)
    cal = camera.get_camera_information().camera_configuration.calibration_parameters.left_cam   # fx, fy, cx, cy (bearing-only)
    runtime = sl.RuntimeParameters()
    runtime.confidence_threshold = 50
    runtime.texture_confidence_threshold = 100
    runtime_fast = sl.RuntimeParameters()                                # v0.1.92: --led-burst grabs - the image only, no depth
    try:
        runtime_fast.enable_depth = False
    except Exception:                                                    # an SDK without the switch: the burst still works, slower
        pass
    image, cloud = sl.Mat(), sl.Mat()
    fields = ["utc", "frame", "index", "colour", "confidence", "range_m", "surface_range_m", "reason", "forward_m", "right_m", "down_m", "bearing_deg", "valid_fraction",
              "valid_pixels", "depth_spread_m", "vehicle_lat", "vehicle_lon", "buoy_lat", "buoy_lon", "north_offset_m", "east_offset_m", "status"]
    csv_file = (run_dir / "buoy_observations.csv").open("w", newline="")
    writer = csv.DictWriter(csv_file, fieldnames=fields, extrasaction="ignore")
    writer.writeheader()
    print(f"[multi_buoy_observer] run {run_dir} (max {a.max_buoys} buoys / frame, colour min-share {a.min_share})", flush=True)
    analysed = 0
    cannon = None
    if a.cannon:                                                     # v0.1.61: the cannon's target from this camera (one owner)
        sys.path.insert(1, str(HERE.parent / "cannon"))
        import aim_cv
        cannon = aim_cv.TargetWriter(a.cannon, a.cannon_out, source="buoys_multi", max_depth_m=a.max_depth)
        print(f"[multi_buoy_observer] cannon target: {a.cannon} -> {cannon.path}", flush=True)
    sess, n_rec, end = None, 0, "frames done"
    pairs = None
    bgw = BgWriter(maxsize=4) if getattr(a, "bg_writer", "on") == "on" else None     # v0.1.86.7 (WP43 Tier 1.2)
    replay_fh = (run_dir / R.FRAMES_NAME).open("w") if a.svo else None
    if a.record:
        signal.signal(signal.SIGTERM, _sigterm)                       # a plain kill closes the SVO like Ctrl-C
    try:
        sess = start_recording(a, camera, weights, spec)
        guard = R.Guard(a.record, a.max_min, a.min_free_gb) if sess else None      # the clock starts with the recording
        pairs = R.Pairs(sess.dir, a.pairs_s) if sess is not None and a.pairs_s > 0 else None   # v0.1.84.4: RGB + depth pairs
        if pairs is not None:
            print(f"[multi_buoy_observer] RGB + depth pairs every {a.pairs_s:g} s -> {pairs.dir}", flush=True)
        led_cache = LC.LedCache() if getattr(a, "led", "on") == "on" else None     # v0.1.91
        num_rd = None                                                    # v0.1.94: the marker number (off by default)
        if getattr(a, "number", "off") == "on":
            import number_core as NC
            num_rd = NC.NumberReader(a.number_weights or NC.DEFAULT_WEIGHTS, {"max_m": a.number_max_m, "min_h_px": a.number_min_h})
        gfilt = GC.GuardFilter({"h_cam_m": a.h_cam, "stop_m": a.guard_stop, "slow_m": a.guard_slow, "map_m": a.guard_map}) if a.guard == "on" else None
        sensors = sl.SensorsData() if gfilt is not None and hasattr(sl, "SensorsData") else None   # no IMU API: unlevelled
        _tim, _prev_t0 = {}, None                                        # v0.1.92: the stage timing
        while not a.frames or analysed < a.frames:
            _t0 = time.monotonic()                                       # v0.1.92: per-stage timing -> latest_status timing_ms (the speed trials)
            err = camera.grab(runtime)
            _tg = time.monotonic()
            if a.svo and err == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
                end = "end of the SVO file"
                break
            if err != sl.ERROR_CODE.SUCCESS:
                time.sleep(0.02)
                continue
            svo_frame = camera.get_svo_position() if a.svo else n_rec
            try:
                ts_ns = int(camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds())
            except Exception:
                ts_ns = None
            camera.retrieve_image(image, sl.VIEW.LEFT)
            camera.retrieve_measure(cloud, sl.MEASURE.XYZRGBA)
            _tr = time.monotonic()
            frame = image.get_data()[:, :, :3].copy()
            xyz = cloud.get_data()
            if cannon is not None:
                cannon.frame(frame, xyz)
            if pairs is not None and sess is not None and sess.active and pairs.due():    # before any drawing on frame
                try:
                    extra = {"utc": S.utc_now(), "frame": analysed, "svo_frame": svo_frame, "ts_ns": ts_ns}
                    pw = int(getattr(a, "pairs_width", 0) or 0)
                    if bgw is not None:                                    # v0.1.86.7: copies to the writer (frame + the z plane, once per pairs_s)
                        fr, zc = frame.copy(), np.array(xyz[:, :, 2], copy=True)
                        bgw.put("pair", lambda fr=fr, zc=zc, extra=extra: save_pair(pairs, fr, zc, a.max_depth, extra, width=pw), keep=True)
                    else:
                        save_pair(pairs, frame, xyz, a.max_depth, extra, width=pw)
                except Exception as exc:                                 # a pair never stops the detections
                    print(f"[multi_buoy_observer] pair not saved: {exc}", flush=True)
            result = model.predict(frame, conf=a.conf, imgsz=a.imgsz, iou=iou, device=0, verbose=False)[0]
            _tp = time.monotonic()
            tel = replay_tel.at(svo_frame) if replay_tel is not None else (telemetry.snapshot() if telemetry else {})
            fresh = (tel.get("position_age_s") is not None and tel["position_age_s"] <= a.telemetry_max_age_s
                     and tel.get("attitude_age_s") is not None and tel["attitude_age_s"] <= a.telemetry_max_age_s and (tel.get("gps_fix") or 0) >= 3)
            cands, bearings, obstacles = [], [], []
            hy = LC.horizon_y(frame.shape[0], frac=a.horizon_frac) if a.horizon == "frac" else None
            if result.boxes is not None and (result.masks is not None or boxes_only):
                masks = None if boxes_only else result.masks.data.cpu().numpy()
                for i, box in enumerate(result.boxes):
                    if hy is not None and LC.above_horizon(box.xyxy[0].cpu().numpy().tolist(), hy):
                        continue                                         # v0.1.91: above the horizon - a building, the sky, never a buoy
                    if int(box.cls.item()) in obst_ids:                  # v0.1.87: an obstacle box - its depth -> a `far` cluster
                        if float(box.conf.item()) >= a.obstacle_conf:
                            xyxy = box.xyxy[0].cpu().numpy().tolist()
                            om = S.measure_mask(xyz, TS.box_mask(frame.shape[0], frame.shape[1], xyxy, a.box_shrink), max_depth_m=a.max_depth)
                            if om:
                                obstacles.append((TS.obstacle_far(om.forward_m, om.right_m, xyxy, cal.fx, cal.cx, float(box.conf.item()),
                                                                  str(model.names[int(box.cls.item())])), xyxy))
                        continue
                    if buoy_ids is not None and int(box.cls.item()) not in buoy_ids:
                        continue                                         # a dock face / window / box of a CV_TRAIN model
                    if boxes_only:                                       # v0.1.78.18: Model A has no masks - the box's inner ellipse
                        mask = TS.box_mask(frame.shape[0], frame.shape[1], box.xyxy[0].cpu().numpy().tolist(), a.box_shrink)
                    else:
                        mask = cv2.resize(masks[i], (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_LINEAR)
                    meas = S.measure_mask(xyz, mask, max_depth_m=a.max_depth)
                    if meas and M.near_limit(meas.range_m, a.max_depth, cal.fx, a.limit_sigma):
                        if float(box.conf.item()) >= a.bearing_conf:
                            bearings.append((float(box.conf.item()), mask, box, "near_limit"))
                        continue
                    if meas:
                        meas = replace(meas, **{k: v for k, v in M.centre_correction(asdict(meas), a.buoy_radius).items()
                                                if k in ("range_m", "forward_m", "right_m", "down_m")})
                        cands.append((meas.range_m, float(box.conf.item()), meas, mask, box))
                    elif float(box.conf.item()) >= a.bearing_conf:
                        bearings.append((float(box.conf.item()), mask, box, None))
            cands.sort(key=lambda c: c[0])
            utc = S.utc_now()
            buoys, shown, boxes = [], [], []
            for k, (_, conf, meas, mask, box) in enumerate(cands[:max(1, a.max_buoys)]):
                colour, shares, more = colour_for(frame, mask, box)
                geo = S.buoy_geolocation(meas, tel, a.camera_forward_m, a.camera_right_m, a.camera_yaw_deg) if fresh else None
                rec = dict(asdict(meas), surface_range_m=round(meas.range_m - a.buoy_radius, 3), radius_m=a.buoy_radius)
                buoys.append(dict(M.buoy_record(rec, conf, colour, shares, geo, k), **more))
                shown.append((box, mask, meas, colour))
                boxes.append(box.xyxy[0].cpu().numpy().tolist())
            for conf, mask, box, why in sorted(bearings, key=lambda b: -b[0])[:max(0, a.max_buoys - len(buoys))]:
                ys, xs = np.nonzero(mask > 0.5)
                if len(xs) == 0:
                    continue
                u, v = M.mask_centre(xs.tolist(), ys.tolist())
                zs = xyz[ys, xs, 2]
                reason = why or ("depth_invalid" if int(np.isfinite(zs).sum()) >= 30 else "beyond_depth")   # no depth at all: past --max-depth
                colour, shares, more = colour_for(frame, mask, box)
                buoys.append(dict(M.bearing_record(u, v, cal.fx, cal.fy, cal.cx, cal.cy, conf, colour, shares, reason, len(buoys)), **more))
                boxes.append(box.xyxy[0].cpu().numpy().tolist())
            if led_cache is not None and buoys:                         # v0.1.91: the LED under each hat + the blink memory (no map)
                samples = [LC.led_sample(frame, bb) for bb in boxes]
                t_led = (ts_ns / 1e9) if ts_ns else time.time()
                for b, (colour, on, shares), (tid, pat) in zip(buoys, samples, led_cache.update(t_led, [(bb, s[1], s[0]) for bb, s in zip(boxes, samples)])):
                    b["led"] = dict(pat, on=on, now=colour)
                if getattr(a, "led_burst", 0) > 0 and not a.svo:         # v0.1.92: the lamps between full frames (image-only grabs)
                    led_img = sl.Mat()

                    def _stamp():
                        try:
                            return camera.get_timestamp(sl.TIME_REFERENCE.IMAGE).get_nanoseconds() / 1e9
                        except Exception:
                            return time.time()
                    n_led = LC.burst(a.led_burst, lambda: camera.grab(runtime_fast) == sl.ERROR_CODE.SUCCESS,
                                     lambda: (camera.retrieve_image(led_img, sl.VIEW.LEFT), led_img.get_data()[:, :, :3])[1], _stamp,
                                     boxes, led_cache)
                    if sess is not None and sess.active:
                        n_rec += n_led                                   # the SVO holds these frames too (svo_frame stays right)
                    for b, bb in zip(buoys, boxes):                      # the freshest pattern per box after the burst
                        tr = led_cache.find(bb)
                        if tr is not None:
                            b["led"] = dict(led_cache.pattern(tr), on=b["led"]["on"], now=b["led"]["now"], burst=n_led)
            if num_rd is not None and buoys:                            # v0.1.94: the near boxes' numbers, one batch, one digit per buoy
                for b, n in zip(buoys, num_rd.read(frame, boxes, [b.get("range_m") for b in buoys])):
                    b["number"] = n                                      # None = not read (far / cut by the frame's side)
                _tim["number"] = round(num_rd.last_ms if "number" not in _tim else 0.9 * _tim["number"] + 0.1 * num_rd.last_ms, 1)
            _tm = time.monotonic()
            payload = M.frame_payload(utc, analysed, buoys, tel)
            payload["detector"] = "default" if a.detector == "mask" else a.detector     # v0.1.87: which model runs (cv_publish -> the OCS)
            payload["model"] = os.path.basename(str(weights))
            if gfilt is not None:                                        # v0.1.80 (WP36): the guard on the same cloud, levelled by the IMU
                quat = None
                try:
                    if sensors is not None and camera.get_sensors_data(sensors, sl.TIME_REFERENCE.IMAGE) == sl.ERROR_CODE.SUCCESS:
                        quat = sensors.get_imu_data().get_pose().get_orientation().get()
                except Exception:
                    quat = None                                          # an SVO without IMU: unlevelled (the CAUTION is the roll 0)
                payload["guard"] = gfilt.frame(xyz, quat, time.time())
                payload["guard"]["imu"] = quat is not None
                if obstacles:                                            # v0.1.87: the model's obstacles join the depth clusters (the boat rings both)
                    payload["guard"]["far"] = sorted((payload["guard"].get("far") or []) + [o for o, _ in obstacles], key=lambda c: c["range_m"])
            if obstacles:
                payload["obstacles"] = [o for o, _ in obstacles]         # v0.1.87: also on their own (frames.jsonl, the log; with --guard off only here)
            if sess is not None or replay_fh is not None:                # v0.1.40: frames.jsonl (the recording's, or the replay's own)
                frec = R.frame_record(utc, analysed, svo_frame, ts_ns, tel, fresh, payload["status"], buoys, boxes)
                if obstacles:
                    frec["obstacles"] = [dict(o, bbox=[round(v, 1) for v in xyxy]) for o, xyxy in obstacles]    # v0.1.87
                if sess is not None and sess.active:
                    sess.add(frec)
                    n_rec += 1
                    why = guard.check()
                    if why:
                        stop_recording(camera, sess, why)
                if replay_fh is not None:
                    replay_fh.write(json.dumps(frec, separators=(",", ":")) + "\n")
                if sess is not None:
                    payload["recording"] = sess.public()
                    if pairs is not None:
                        payload["recording"]["pairs"] = pairs.public()
            for b in buoys:
                writer.writerow(dict(b, utc=utc, frame=analysed, status=payload["status"], vehicle_lat=tel.get("lat", ""), vehicle_lon=tel.get("lon", "")))
            if not buoys:
                writer.writerow({"utc": utc, "frame": analysed, "status": payload["status"]})
            csv_file.flush()
            if bgw is not None:
                payload["writer"] = bgw.public()                         # v0.1.86.7: done / dropped_previews / inline / max_job_ms / queued
            _te = time.monotonic()
            for k_, v_ in (("grab_depth", _tg - _t0), ("retrieve", _tr - _tg), ("detector", _tp - _tr), ("measure_led", _tm - _tp),
                           ("guard_write", _te - _tm), ("loop", (_t0 - _prev_t0) if _prev_t0 else None)):
                if v_ is not None:                                       # an EMA over ~10 frames, ms
                    _tim[k_] = round(v_ * 1000.0 if k_ not in _tim else 0.9 * _tim[k_] + 0.1 * v_ * 1000.0, 1)
            _prev_t0 = _t0
            payload["timing_ms"] = dict(_tim, fps=round(1000.0 / _tim["loop"], 2) if _tim.get("loop") else None)
            tmp = run_dir / "latest_status.json.tmp"
            tmp.write_text(json.dumps(payload, indent=1))
            os.replace(tmp, run_dir / "latest_status.json")
            preview = a.preview_every > 0 and analysed % a.preview_every == 0      # v0.1.84.8: the live feed (was every 15th frame)
            if preview or analysed == 0 or analysed % 15 == 0 or not a.headless:
                for j, (box, mask, meas, colour) in enumerate(shown):
                    c = BGR.get(colour, BGR["unknown"])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())
                    cv2.rectangle(frame, (x1, y1), (x2, y2), c, 2)
                    cv2.putText(frame, f"{colour} {meas.range_m:.1f}m {meas.bearing_deg:+.0f}deg", (max(0, x1), max(35, y1 - 10)), 0, 0.7, c, 2)
                    led = buoys[j].get("led") if j < len(buoys) else None
                    if isinstance(led, dict) and hasattr(cv2, "circle"):  # v0.1.92: the LED under the hat - a dot in its colour when lit, a
                        lc = BGR.get(led.get("now") or led.get("colour") or "", BGR["unknown"])     # grey ring when off; + the pattern
                        cx, cy = (x1 + x2) // 2, max(12, y1 - 32)
                        cv2.circle(frame, (cx, cy), 9, lc if led.get("on") else BGR["unknown"], -1 if led.get("on") else 2)
                        pat = {"flashing": "FLASH", "steady": "STEADY"}.get(led.get("pattern"), "")
                        if pat:
                            cv2.putText(frame, f"{pat} {led.get('colour') or ''}", (cx + 14, cy + 6), 0, 0.6, BGR.get(led.get("colour") or "", c), 2)
                for o, xyxy in obstacles:                                # v0.1.87: the model's obstacles, orange
                    x1, y1, x2, y2 = map(int, xyxy)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 128, 255), 2)
                    cv2.putText(frame, f"{o['cls']} {o['range_m']:.1f}m", (max(0, x1), max(35, y1 - 10)), 0, 0.7, (0, 128, 255), 2)
                if preview or analysed == 0:
                    if bgw is not None:                                    # v0.1.86.7: resized here (small), encoded + written by the writer
                        pw_ = a.preview_width
                        small = cv2.resize(frame, (pw_, int(frame.shape[0] * pw_ / frame.shape[1])), interpolation=cv2.INTER_AREA) \
                            if pw_ and frame.shape[1] > pw_ else frame.copy()
                        bgw.put("preview", lambda p=run_dir / "latest_preview.jpg", img=small: write_preview(p, img, 0))
                    else:
                        write_preview(run_dir / "latest_preview.jpg", frame, a.preview_width)
                if analysed == 0 or analysed % 15 == 0:
                    print(f"frame={analysed} status={payload['status']} buoys={len(buoys)} " +
                          " ".join(f"{b['colour']}@{b['range_m']:.1f}m/{b['bearing_deg']:+.0f}" if b.get("range_m") is not None
                                   else f"{b['colour']}@ray/{b['bearing_deg']:+.0f}({b['reason']})" for b in buoys), flush=True)
            if not a.headless:
                cv2.imshow("multi buoy observer - Q/Esc", frame)
                if cv2.waitKey(1) & 255 in (27, ord("q")):
                    break
            analysed += 1
    except KeyboardInterrupt:
        end = "stopped by the operator (Ctrl-C / agx_record.sh stop)"
    finally:
        if cannon is not None:
            cannon.stopped()
        if bgw is not None:
            bgw.close()                                                  # v0.1.86.7: the queued pairs land before the session closes
        stop_recording(camera, sess, end)
        if pairs is not None:
            pairs.close()
        if replay_fh is not None:
            replay_fh.close()
        csv_file.close()
        camera.close()
        if telemetry:
            telemetry.close()
        if not a.headless:
            cv2.destroyAllWindows()
    print(f"Logs: {run_dir}")


if __name__ == "__main__":
    main()
