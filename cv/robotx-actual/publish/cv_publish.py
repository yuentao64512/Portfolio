#!/usr/bin/env python3
"""cv_publish.py — AGX side of the CV leg. Two student programs write a latest_status.json each frame:
  * actual_buoy_observer.py  -> observer_logs/<run>/latest_status.json       (one buoy per frame: range, bearing, conf, pose)
    or OUR cv/buoys_multi/multi_buoy_observer.py (v0.1.30): the same file with `buoys` = every buoy of the frame, each with a colour
  * docking_cv/dock_mapper_sender.py -> reports/live_map/latest/latest_status.json (the red dock face: range, bearing, conf)
This script publishes every new frame as an ARCHITECTURE 5.4 Observation on the boat broker (mosquitto on the Orin NX):

    local/usv/perception/buoys     one envelope per observer frame (items: [] when NO_RELIABLE_BUOY)
    local/usv/perception/dock      one envelope per dock-mapper frame (items: [] when no target), kind dock, colour red
    local/usv/perception/guard     v0.1.80 (WP36): the ZED proximity guard of the observer's frame ({clear} | range / bearing extent / zone)
    local/usv/perception/health    1 Hz: ok, pose_age_s, rate_hz, n_published, detector, model, classes (v0.1.49: which CV model),
                                   boxes (v0.1.54: does the observer give pixel boxes)
    local/usv/perception/frame_dets  v0.1.54: one per observer frame that carries `frame_dets` (sim_observer): {t, t_cam, frame_w,
                                   frame_h, frame_id, source, dets: [{cls, colour, light, x0, y0, x1, y1, range_m, conf}]} - the
                                   camera overlays' boxes (boat window, OCS /dets.json); the students' observers give none
    presence/cv_publish            retained beat (the bench TUI / usv_ui NETWORK view)

Nothing in the student programs changes. usv_ui ('cv on') and the mission manager fold these into the PerceptionMap.

    .venv/bin/python cv_publish.py                                   # on the AGX: broker from ../network.env (BOAT_HOST:BOAT_PORT)
    .venv/bin/python cv_publish.py --mqtt 192.168.1.218:1884 --demo  # no camera: a synthetic buoy 8 m ahead (link test)
    .venv/bin/python cv_publish.py --dock-output /home/optiprime-agxorin/docking_cv/reports/live_map/latest   # + the dock mapper
    python3 cv_publish.py --mqtt 127.0.0.1:1884 --replay observer_logs/<run>/buoy_observations.csv   # a logged run, paced

Needs paho-mqtt (pip install paho-mqtt). Body frame: bearing + to starboard, range horizontal (forward/right), as the
observer measures it (its bearing_deg = atan2(right, forward)). The capture time (t_cam) is the observer's utc stamp:
keep the AGX and the NX clocks within a second (chrony / 'sudo date -s'), else the map geolocates with a stale pose.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import glob
import json
import math
import os
import signal
import socket
import sys
import time

import paho.mqtt.client as mqtt


def utc_iso():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def parse_utc(s):
    """ISO 8601 (the observer's datetime.now(timezone.utc).isoformat()) -> unix seconds; None when unparsable."""
    try:
        s = s.replace("Z", "+00:00")
        d = dt.datetime.fromisoformat(s)
        if d.tzinfo is None:
            d = d.replace(tzinfo=dt.timezone.utc)
        return d.timestamp()
    except Exception:
        return None


def fnum(x, default=None):
    try:
        return float(x)
    except (TypeError, ValueError):
        return default


def _extras(meas):
    """v0.1.92: the LED reader (pattern, colour, on, now); v0.1.94: on the bearing-only items too (13 Nov: 95 % of the sightings
    had no range, so the boat never saw their lamp readings), + bbox_h (the box height, px) and the number reader's result verbatim."""
    out = {}
    if isinstance(meas.get("led"), dict):
        out["led"] = {k: meas["led"].get(k) for k in ("pattern", "colour", "on", "now")}
    bb = meas.get("bbox")
    if isinstance(bb, (list, tuple)) and len(bb) == 4:
        out["bbox_h"] = round(float(bb[3]) - float(bb[1]), 1)
    if isinstance(meas.get("number"), dict):
        out["number"] = meas["number"]
    return out


class Publisher:
    def __init__(self, a):
        self.a = a
        self.ns = "local/%s" % a.vehicle
        self.seq = 0
        self.n_pub = 0
        self.n_dock = 0
        self.frames = []                     # monotonic times of published frames (rate)
        self.t_last_frame = None
        self.pose_age = None
        self.connected = False
        self.model, self.classes = model_info(a)       # v0.1.49 (WP10k-b item 23): which model; the observer's payload may say more
        self.obs_detector = None
        self.boxes = False                   # v0.1.54: the observer's payload carried frame_dets (pixel boxes)
        self.n_dets = 0
        self.c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="cv-publish-%d" % os.getpid(), protocol=mqtt.MQTTv311)
        self.c.on_connect = self._on_connect
        self.c.on_disconnect = self._on_disconnect
        self.c.reconnect_delay_set(1, 4)
        self.c.will_set("presence/cv_publish", json.dumps({"node": "cv_publish", "up": False, "t": utc_iso()}), qos=0, retain=True)

    def _on_connect(self, c, u, f, rc, p=None):
        self.connected = True
        self.log("connected to %s" % self.a.mqtt)

    def _on_disconnect(self, c, u, f, rc=None, p=None):
        self.connected = False

    def log(self, s):
        print("[cv_publish] %s" % s, flush=True)

    def envelope(self, body):
        self.seq += 1
        return json.dumps({"schema_version": 1, "msg_id": "cvp-%010d" % self.seq, "seq": self.seq, "sent_at": utc_iso(), "run_id": None,
                           "declaration_seq": None, "vehicle_id": self.a.vehicle_id, "kind": "local", "source": "cv_publish", "body": body})

    def start(self):
        host, _, port = self.a.mqtt.partition(":")
        self.c.connect_async(host or "127.0.0.1", int(port or 1884), keepalive=15)
        self.c.loop_start()

    def stop(self):
        try:
            self.c.publish("presence/cv_publish", json.dumps({"node": "cv_publish", "up": False, "t": utc_iso()}), qos=0, retain=True).wait_for_publish(1.0)
        except Exception:
            pass
        self.c.loop_stop()
        self.c.disconnect()

    # ---- the three outputs --------------------------------------------------------------------------------------
    def observation(self, t_cam, items, pose_hint, kind="buoys"):
        body = {"t": utc_iso(), "t_cam": round(t_cam, 3), "frame": "body_FRD", "camera": self.a.camera, "kind": kind,
                "pose_hint": pose_hint, "items": items, "source": "usv"}
        self.c.publish("%s/perception/%s" % (self.ns, kind), self.envelope(body), qos=0)
        self.n_pub += 1
        if kind == "dock":
            self.n_dock += 1
        now = time.monotonic()
        self.frames.append(now)
        self.frames = [t for t in self.frames if now - t <= 5.0]
        self.t_last_frame = now

    def health(self):
        now = time.monotonic()
        rate = (len(self.frames) - 1) / (self.frames[-1] - self.frames[0]) if len(self.frames) > 1 and self.frames[-1] > self.frames[0] else 0.0
        fresh = self.t_last_frame is not None and now - self.t_last_frame < 2.0
        body = {"ok": bool(fresh), "pose_age_s": self.pose_age, "rate_hz": round(rate, 2), "n_published": self.n_pub,
                "camera": self.a.camera, "detector": self.a.detector, "t": utc_iso(),
                "model": self.model, "classes": self.classes, "observer_detector": self.obs_detector, "boxes": self.boxes}
        self.c.publish("%s/perception/health" % self.ns, self.envelope(body), qos=0)

    def frame_dets(self, t_cam, fd):
        """v0.1.54 (item 11): the observer's per-frame pixel boxes as they are (nothing added) -> local/<v>/perception/frame_dets."""
        dets = [d for d in (fd.get("dets") or []) if isinstance(d, dict)]
        body = {"t": utc_iso(), "t_cam": round(t_cam, 3), "camera": self.a.camera, "source": self.obs_detector or self.a.detector,
                "frame_w": fd.get("frame_w"), "frame_h": fd.get("frame_h"), "frame_id": fd.get("frame_id"), "dets": dets}
        self.c.publish("%s/perception/frame_dets" % self.ns, self.envelope(body), qos=0)
        self.boxes = True
        self.n_dets += 1
        return body

    def presence(self):
        self.c.publish("presence/cv_publish", json.dumps({"node": "cv_publish", "broker": self.a.mqtt, "host": socket.gethostname(), "pid": os.getpid(), "up": True,
                                                          "t": utc_iso(), "t_unix": round(time.time(), 3)}), qos=0, retain=True)

    # ---- observer frame -> items -------------------------------------------------------------------------------
    def frame_to_items(self, meas, frame_no, colour=None, idx=0):
        """One measured buoy (the observer's measurement dict) -> one Observation item; [] when it has no range / bearing.
        colour: the multi observer's per-buoy colour (v0.1.30); unknown / none -> --colour."""
        rng = fnum(meas.get("range_m"))
        brg = fnum(meas.get("bearing_deg"))
        if brg is not None and rng is None and meas.get("bearing_only"):    # v0.1.34: a bearing without a range (beyond the depth)
            col = colour if colour not in (None, "", "unknown") else self.a.colour
            return [{"id": int(frame_no or 0) * 10 + int(idx), "class": "buoy", "colour": col, "confidence": fnum(meas.get("confidence"), 0.5),
                     "range_m": None, "bearing_deg": round(brg, 2), "elev_deg": round(fnum(meas.get("elev_deg"), 0.0), 2),
                     "quality": {"bearing_only": True, "reason": meas.get("reason"), "status": meas.get("status"), **_extras(meas)}}]
        if rng is None or brg is None:
            return []
        fwd, right, down = fnum(meas.get("forward_m")), fnum(meas.get("right_m")), fnum(meas.get("down_m"), 0.0)
        if fwd is not None and right is not None:
            rng_h = math.hypot(fwd, right)               # horizontal range for the map (the observer's range is 3-D)
            elev = -math.degrees(math.atan2(down, max(rng_h, 1e-3)))
        else:
            rng_h, elev = rng, 0.0
        col = colour if colour not in (None, "", "unknown") else self.a.colour
        return [{"id": int(frame_no or 0) * 10 + int(idx), "class": "buoy", "colour": col, "confidence": fnum(meas.get("confidence"), 0.5),
                 "range_m": round(rng_h, 3), "bearing_deg": round(brg, 2), "elev_deg": round(elev, 2),
                 "quality": {"valid_fraction": fnum(meas.get("valid_fraction")), "valid_pixels": fnum(meas.get("valid_pixels")),
                             "depth_spread_m": fnum(meas.get("depth_spread_m")), "surface_range_m": fnum(meas.get("surface_range_m")),
                             "status": meas.get("status"), **_extras(meas)}}]


    def pose_hint_from(self, tel, t_cam):
        if not tel or tel.get("lat") is None:
            return None
        yaw = tel.get("yaw_rad")
        hint = {"lat": tel.get("lat"), "lon": tel.get("lon"), "hdg_deg": None if yaw is None else round(math.degrees(yaw) % 360.0, 1), "t": round(t_cam, 3)}
        self.pose_age = tel.get("position_age_s")
        return hint


# ---- sources ----------------------------------------------------------------------------------------------------------
def _read_kv(path, out):
    for line in open(path):
        line = line.split("#", 1)[0].strip()
        if "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()


def read_network_env():
    """KEY=VALUE lines of network.env (next to this file, or one/two levels up: MASTER_SCRIPTS/network.env), then the
    address overlay network.<NET_PROFILE>.env (ts | lan; the NET_PROFILE environment variable wins over the file's line)."""
    here = os.path.dirname(os.path.abspath(__file__))
    out = {}
    for d in (here, os.path.dirname(here), os.path.dirname(os.path.dirname(here))):
        p = os.path.join(d, "network.env")
        if os.path.exists(p):
            _read_kv(p, out)
            out["_path"] = p
            prof = os.environ.get("NET_PROFILE") or out.get("NET_PROFILE") or "ts"
            overlay = os.path.join(d, "network.%s.env" % prof)
            if os.path.exists(overlay):
                _read_kv(overlay, out)
                out["_overlay"] = overlay
            out["NET_PROFILE"] = prof
            break
    return out


def model_info(a):
    """(model name, class names) for perception/health (v0.1.49): --model DIR = a CV_TRAIN folder (its PROVENANCE.md `# title` or the
    folder name; classes.yaml names), else synthetic (--demo, or the sim observer's synthetic detector), else the students' best.pt."""
    d = getattr(a, "model", None)
    if d:
        name, classes = os.path.basename(os.path.normpath(d)), None
        try:
            for line in open(os.path.join(d, "PROVENANCE.md"), encoding="utf-8"):
                if line.startswith("# "):
                    name = line[2:].strip().split(" ")[0] or name
                    break
        except OSError:
            pass
        try:
            import yaml
            names = (yaml.safe_load(open(os.path.join(d, "classes.yaml"))) or {}).get("names") or {}
            classes = [str(names[k]) for k in sorted(names, key=lambda x: int(x))] if isinstance(names, dict) else [str(x) for x in names]
        except Exception:
            pass
        return name, classes
    if getattr(a, "demo", False):
        return "synthetic", ["buoy"]
    return "students best.pt", ["buoy"]


def observer_model(pub, payload):
    """The observer's own word on its detector (sim_observer: synthetic | model + the model's name / classes) wins over the default."""
    det = payload.get("detector")
    if det is None or getattr(pub.a, "model", None):
        return
    pub.obs_detector = det
    if det == "synthetic":
        pub.model, pub.classes = "synthetic", ["buoy"]
    elif payload.get("model"):
        pub.model, pub.classes = str(payload["model"]), payload.get("classes") or pub.classes


def newest_run_dir(output):
    dirs = [d for d in glob.glob(os.path.join(output, "*")) if os.path.isdir(d)]
    return max(dirs, key=os.path.getmtime) if dirs else None


def watch_observer(pub, a):
    """Poll latest_status.json of the newest observer run dir (+ the dock mapper's latest dir); publish on every new frame."""
    run_dir, path, last_sig, last_check = None, None, None, 0.0
    dock_path = os.path.join(a.dock_output, "latest_status.json") if a.dock_output else None
    dock_sig = None
    last_health = last_presence = 0.0
    pub.log("watching %s for the newest run (latest_status.json)%s" % (a.output, ("; dock mapper: %s" % dock_path) if dock_path else ""))
    while True:
        now = time.monotonic()
        if now - last_check >= 2.0 or run_dir is None:
            last_check = now
            d = newest_run_dir(a.output)
            if d and d != run_dir:
                run_dir, path = d, os.path.join(d, "latest_status.json")
                pub.log("run dir: %s" % run_dir)
        if path and os.path.exists(path):
            try:
                st = os.stat(path)
                sig = (st.st_mtime_ns, st.st_size)
                if sig != last_sig:
                    last_sig = sig
                    payload = json.load(open(path))
                    publish_payload(pub, payload)
            except (OSError, ValueError) as e:            # mid-write (the observer uses os.replace, so rare)
                pub.log("read %s: %s" % (path, e))
        if dock_path and os.path.exists(dock_path):
            try:
                st = os.stat(dock_path)
                sig = (st.st_mtime_ns, st.st_size)
                if sig != dock_sig:
                    dock_sig = sig
                    publish_dock_payload(pub, json.load(open(dock_path)))
            except (OSError, ValueError) as e:
                pub.log("read %s: %s" % (dock_path, e))
        if now - last_health >= 1.0:
            pub.health()
            last_health = now
        if now - last_presence >= 2.0:
            pub.presence()
            last_presence = now
        time.sleep(1.0 / a.hz)


def publish_payload(pub, payload):
    observer_model(pub, payload)
    t_cam = parse_utc(payload.get("updated_utc") or "") or time.time()
    meas = payload.get("measurement") or {}
    status = payload.get("status")
    if isinstance(payload.get("buoys"), list):          # v0.1.30: our multi observer - every buoy, each with its colour
        items = [it for i, b in enumerate(payload["buoys"])
                 for it in pub.frame_to_items(dict(b, status=status), payload.get("frame"), b.get("colour"), b.get("index", i))]
    else:
        items = pub.frame_to_items(dict(meas, status=status), payload.get("frame")) if status in ("OK", "BUOY_DEPTH_OK_GPS_UNAVAILABLE") else []
    hint = pub.pose_hint_from(payload.get("telemetry") or {}, t_cam)
    pub.observation(t_cam, items, hint)
    if isinstance(payload.get("guard"), dict):          # v0.1.80 (WP36): the ZED proximity guard -> local/<v>/perception/guard, every frame
        t_pub = time.time()                             # v0.1.86.8 (WP43 §0): the pipeline latency on the AGX's own clock
        pub.c.publish("%s/perception/guard" % pub.ns, pub.envelope(dict(payload["guard"], t_cam=t_cam, t_pub=round(t_pub, 3),
                                                                      pipe_ms=round((t_pub - t_cam) * 1000.0, 1))), qos=0)
    if isinstance(payload.get("frame_dets"), dict):     # v0.1.54: sim_observer's pixel boxes (the camera overlays)
        pub.frame_dets(t_cam, payload["frame_dets"])
    if isinstance(payload.get("dock"), list):           # v0.1.48 (WP10j-b): sim_observer's dock parts -> perception/dock + /window
        dock_items = dock_to_items(payload["dock"], payload.get("frame"))
        pub.observation(t_cam, dock_items["dock"], hint, kind="dock")
        if dock_items["window"]:
            pub.observation(t_cam, dock_items["window"], hint, kind="window")
    if pub.n_pub == 1 or pub.n_pub % 50 == 0:
        pub.log("frame %s %s -> %d item(s), n_published %d" % (payload.get("frame"), status, len(items), pub.n_pub))


def dock_to_items(parts, frame_no):
    """v0.1.48: sim_observer's dock records -> Observation items: dock_face (class dock, colour white: the face is white, its colour
    comes from the indicator) and indicator (class indicator, red / green) on perception/dock, window (light off / red / green as the
    colour, the window number 1 upper / 2 lower in quality.window) on perception/window."""
    out = {"dock": [], "window": []}
    faces = [p for p in parts if p.get("cls") == "dock_face"]
    rest = [p for p in parts if p.get("cls") != "dock_face"]
    for i, p in enumerate(faces + rest):
        rng, brg = fnum(p.get("range_m")), fnum(p.get("bearing_deg"))
        if rng is None or brg is None:
            continue
        cls = p.get("cls")
        it = {"id": int(frame_no or 0) * 100 + i, "class": "dock" if cls == "dock_face" else cls, "confidence": fnum(p.get("confidence"), 0.5),
              "range_m": round(rng, 3), "bearing_deg": round(brg, 2), "elev_deg": 0.0,
              "quality": {"valid_pixels": fnum(p.get("valid_pixels")), "face": p.get("face")}}
        if p.get("face_points"):                                   # v0.1.58 (WP13): the face's left / midpoint / right (body-polar)
            it["quality"]["face_points"] = {k: {"range_m": v["range_m"], "bearing_deg": v["bearing_deg"]} for k, v in p["face_points"].items()}
        if cls == "window":
            it["colour"] = {"red": "red", "green": "green"}.get(p.get("light"), "black")
            it["lit"] = it["colour"] != "black"
            it["quality"]["window"] = p.get("window")
            out["window"].append(it)
        else:
            it["colour"] = p.get("colour") or ("white" if cls == "dock_face" else "unknown")
            out["dock"].append(it)
    return out


def publish_dock_payload(pub, body):
    """dock_mapper_sender.py's body: {status, target{confidence, range_m, forward_m, right_m, down_m, bearing_deg, ...,
    vehicle_latitude_deg, vehicle_longitude_deg, vehicle_heading_deg}, sent_utc, sequence}. Camera-frame measurement,
    like the buoy observer (usv_ui / the mission manager apply the camera offsets)."""
    t_cam = parse_utc(body.get("sent_utc") or "") or time.time()
    tgt = body.get("target") or {}
    items = []
    if body.get("status") == "OK" and tgt:
        meas = dict(tgt, status="OK")
        for it in pub.frame_to_items(meas, body.get("sequence")):
            it["class"] = "dock"
            it["colour"] = tgt.get("color") or tgt.get("colour") or "red"
            it["quality"]["red_fraction"] = fnum(tgt.get("red_fraction"))
            items.append(it)
    hint = None
    if tgt.get("vehicle_latitude_deg") is not None:
        hint = {"lat": tgt.get("vehicle_latitude_deg"), "lon": tgt.get("vehicle_longitude_deg"), "hdg_deg": tgt.get("vehicle_heading_deg"), "t": round(t_cam, 3)}
    pub.observation(t_cam, items, hint, kind="dock")
    if pub.n_dock == 1 or pub.n_dock % 50 == 0:
        pub.log("dock seq %s %s -> %d item(s), n_dock %d" % (body.get("sequence"), body.get("status"), len(items), pub.n_dock))


def replay_csv(pub, a):
    """buoy_observations.csv of a logged run, paced by its utc column (--speed x)."""
    rows = list(csv.DictReader(open(a.replay)))
    pub.log("replaying %d rows from %s at %gx" % (len(rows), a.replay, a.speed))
    t0_log = parse_utc(rows[0]["utc"]) if rows else None
    t0 = time.monotonic()
    last_health = last_presence = 0.0
    for r in rows:
        t_row = parse_utc(r["utc"]) or t0_log
        while time.monotonic() - t0 < (t_row - t0_log) / a.speed:
            now = time.monotonic()
            if now - last_health >= 1.0:
                pub.health(); last_health = now
            if now - last_presence >= 2.0:
                pub.presence(); last_presence = now
            time.sleep(0.01)
        tel = {"lat": fnum(r.get("vehicle_lat")), "lon": fnum(r.get("vehicle_lon")),
               "yaw_rad": None if not r.get("vehicle_yaw_deg") else math.radians(float(r["vehicle_yaw_deg"])), "position_age_s": 0.0}
        payload = {"updated_utc": utc_iso() if a.now else r["utc"], "frame": r.get("frame"), "status": r.get("status"), "measurement": r, "telemetry": tel}
        publish_payload(pub, payload)
    pub.log("replay done: %d frames" % pub.n_pub)


def demo(pub, a):
    """No camera: one buoy 8 m ahead, 5 deg to starboard, 5 Hz - proves AGX -> NX broker -> map without the ZED."""
    pub.log("demo: synthetic buoy range 8.0 m bearing +5 deg at 5 Hz (Ctrl-C to stop)")
    last_health = last_presence = 0.0
    while True:
        now = time.monotonic()
        pub.observation(time.time(), [{"id": 1, "class": "buoy", "colour": a.colour, "confidence": 0.9, "range_m": 8.0, "bearing_deg": 5.0,
                                       "elev_deg": -3.0, "quality": {"valid_fraction": 0.8, "depth_spread_m": 0.1, "status": "DEMO"}}], None)
        if now - last_health >= 1.0:
            pub.health(); last_health = now
        if now - last_presence >= 2.0:
            pub.presence(); last_presence = now
        time.sleep(0.2)


def _sigterm(*_):
    raise KeyboardInterrupt


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    net = read_network_env()
    mq_default = "%s:%s" % (net.get("BOAT_HOST", "192.168.1.218"), net.get("BOAT_PORT", "1884"))
    ap.add_argument("--mqtt", default=mq_default, help="boat broker host:port (default from network.env BOAT_HOST:BOAT_PORT = %s)" % mq_default)
    ap.add_argument("--output", default=net.get("AGX_OBSERVER_LOGS", "/home/optiprime-agxorin/actual_buoy/observer_logs"),
                    help="the buoy observer's --output dir, newest run followed (default from network.env AGX_OBSERVER_LOGS)")
    ap.add_argument("--dock-output", default=None, metavar="DIR",
                    help="also follow the dock mapper: its reports/live_map/latest dir (latest_status.json) -> perception/dock")
    ap.add_argument("--hz", type=float, default=20.0, help="poll rate of latest_status.json (v0.1.86.6, WP43 Tier 1.1: 20, was 10 - an os.stat each)")
    ap.add_argument("--vehicle", default="usv")
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--camera", default="zed2i_left")
    ap.add_argument("--detector", default="actual_buoy_observer")
    ap.add_argument("--model", default=None, metavar="DIR", help="v0.1.49: the CV_TRAIN model folder the observer runs (classes.yaml / PROVENANCE.md "
                                                                "name it in perception/health); none = the students' best.pt (synthetic with --demo)")
    ap.add_argument("--colour", default="unknown", help="the observer has no colour classifier yet; 'red'/'green' to force one for a test")
    ap.add_argument("--replay", default=None, metavar="CSV", help="replay a buoy_observations.csv instead of watching")
    ap.add_argument("--speed", type=float, default=1.0, help="replay pacing factor")
    ap.add_argument("--now", action="store_true", help="replay: stamp t_cam with the wall clock instead of the logged utc")
    ap.add_argument("--demo", action="store_true", help="publish a synthetic buoy (no camera, link test)")
    a = ap.parse_args(argv)
    pub = Publisher(a)
    if net.get("_path"):
        pub.log("network.env: %s, profile %s (BOAT_HOST %s)" % (net["_path"], net.get("NET_PROFILE"), net.get("BOAT_HOST")))
    signal.signal(signal.SIGTERM, _sigterm)          # a clean stop publishes presence up=false
    pub.start()
    try:
        if a.demo:
            demo(pub, a)
        elif a.replay:
            replay_csv(pub, a)
        else:
            watch_observer(pub, a)
    except KeyboardInterrupt:
        pass
    finally:
        pub.stop()
        pub.log("stopped after %d observations" % pub.n_pub)
    return 0


if __name__ == "__main__":
    sys.exit(main())
