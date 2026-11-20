#!/usr/bin/env python3
"""Receive authenticated green target coordinates, aim pan/tilt, and optionally
run the water pump once for 10 seconds after a stable lock.

BCM 18 = pan, BCM 19 = tilt, BCM 17 = pump. Pump operation requires all of:
--live, --enable-pump, the exact confirmation text, and a valid arm file.
"""
from __future__ import annotations

import argparse
import hashlib
import hmac
import json
import math
import signal
import socket
import time
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_CONFIG = ROOT / "color_cannon_config.json"
LOG_DIR = ROOT / "color_receiver_logs"
SERVO_CONFIRM = "I_ACCEPT_SERVO_MOVEMENT"
PUMP_CONFIRM = "I_ACCEPT_GREEN_10_SECOND_WATER_CANNON"


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def load_config(path):
    cfg = json.loads(Path(path).read_text())
    required = ("network", "target", "gpio_bcm", "pan_servo",
                "tilt_servo", "tracking", "pump")
    missing = [name for name in required if name not in cfg]
    if missing:
        raise ValueError(f"Missing config sections: {missing}")
    return cfg


def decode_verified(packet, key):
    envelope = json.loads(packet)
    body = envelope["body"]
    raw = json.dumps(body, separators=(",", ":"), sort_keys=True).encode()
    expected = hmac.new(key, raw, hashlib.sha256).hexdigest()
    if not hmac.compare_digest(expected, envelope["hmac_sha256"]):
        raise ValueError("BAD_HMAC")
    return body


def finite(value, name):
    if isinstance(value, bool):
        raise ValueError(f"{name}_NOT_NUMBER")
    number = float(value)
    if not math.isfinite(number):
        raise ValueError(f"{name}_NOT_FINITE")
    return number


def servo_pulse(angle_deg, servo_cfg, name):
    maximum = float(servo_cfg["maximum_abs_angle_deg"])
    if abs(angle_deg) > maximum:
        raise ValueError(f"{name}_ANGLE_OUT_OF_RANGE")
    pulse = (float(servo_cfg["center_pulse_us"])
             + float(servo_cfg["direction"]) * angle_deg
             * float(servo_cfg["microseconds_per_degree"]))
    return int(round(max(float(servo_cfg["minimum_pulse_us"]),
                         min(float(servo_cfg["maximum_pulse_us"]), pulse))))


def target_solution(target, cfg):
    target_cfg = cfg["target"]
    if target.get("class_name") != "dock":
        raise ValueError("WRONG_CLASS")
    color = str(target.get("color", "")).lower()
    if color not in target_cfg["allowed_colors"]:
        raise ValueError("WRONG_COLOR")
    confidence = finite(target.get("confidence"), "confidence")
    color_fraction = finite(target.get("color_fraction"), "color_fraction")
    x_right = finite(target.get("x_right_m"), "x_right_m")
    y_forward = finite(target.get("y_forward_m"), "y_forward_m")
    camera_z_up = finite(target.get("z_up_m"), "z_up_m")
    camera_height = finite(cfg.get("camera_height_above_cannon_m", 0.0),
                           "camera_height_above_cannon_m")
    z_up = camera_z_up + camera_height
    range_m = finite(target.get("range_m"), "range_m")
    if confidence < float(target_cfg["minimum_confidence"]):
        raise ValueError("LOW_CONFIDENCE")
    if color_fraction < float(target_cfg["minimum_color_fraction"]):
        raise ValueError("LOW_COLOR_FRACTION")
    if not (float(target_cfg["minimum_range_m"]) <= range_m
            <= float(target_cfg["maximum_range_m"])):
        raise ValueError("RANGE_OUT_OF_BOUNDS")
    if y_forward < float(target_cfg["minimum_forward_m"]):
        raise ValueError("TARGET_NOT_FORWARD")
    horizontal = math.hypot(x_right, y_forward)
    pan_deg = math.degrees(math.atan2(x_right, y_forward))
    tilt_deg = math.degrees(math.atan2(z_up, horizontal))
    return {
        "color": color, "confidence": confidence,
        "color_fraction": color_fraction,
        "x_right_m": x_right, "y_forward_m": y_forward,
        "camera_z_up_m": camera_z_up, "z_up_m": z_up,
        "camera_height_above_cannon_m": camera_height,
        "range_m": range_m,
        "pan_deg": pan_deg, "tilt_deg": tilt_deg,
        "pan_pulse_us": servo_pulse(pan_deg, cfg["pan_servo"], "PAN"),
        "tilt_pulse_us": servo_pulse(tilt_deg, cfg["tilt_servo"], "TILT"),
    }


def arm_file_is_valid(cfg):
    path = Path(cfg["pump"]["arm_file"])
    try:
        return (path.is_file()
                and path.read_text().strip()
                == str(cfg["pump"]["arm_file_required_text"]))
    except OSError:
        return False


class Hardware:
    def __init__(self, cfg, live, pump_authorized=False):
        self.cfg = cfg
        self.live = live
        self.pump_authorized = pump_authorized
        self.pi = None
        pins = cfg["gpio_bcm"]
        self.pan_pin = int(pins["pan_servo"])
        self.tilt_pin = int(pins["tilt_servo"])
        self.pump_pin = int(pins["pump_trigger"])
        self.pump_active = 1 if pins["pump_active_high"] else 0
        self.pump_inactive = 1 - self.pump_active
        if live:
            import pigpio
            self.pigpio = pigpio
            self.pi = pigpio.pi()
            if not self.pi.connected:
                raise RuntimeError("Could not connect to pigpiod")
            self.pi.set_mode(self.pump_pin, pigpio.OUTPUT)
            self.pi.write(self.pump_pin, self.pump_inactive)
            self.pi.set_mode(self.pan_pin, pigpio.OUTPUT)
            self.pi.set_mode(self.tilt_pin, pigpio.OUTPUT)
            self.center()

    def aim(self, pan_pulse, tilt_pulse):
        if self.live:
            self.pi.set_servo_pulsewidth(self.pan_pin, int(pan_pulse))
            self.pi.set_servo_pulsewidth(self.tilt_pin, int(tilt_pulse))

    def center(self):
        self.aim(int(self.cfg["pan_servo"]["center_pulse_us"]),
                 int(self.cfg["tilt_servo"]["center_pulse_us"]))

    def pump_off(self):
        if self.live and self.pi is not None:
            self.pi.write(self.pump_pin, self.pump_inactive)

    def pump_on(self):
        if not self.live or not self.pump_authorized:
            return False
        self.pi.write(self.pump_pin, self.pump_active)
        return True

    def safe_timeout(self):
        self.pump_off()
        self.center()

    def close(self):
        try:
            self.pump_off()
            if self.live and self.pi is not None:
                self.pi.set_servo_pulsewidth(self.pan_pin, 0)
                self.pi.set_servo_pulsewidth(self.tilt_pin, 0)
        finally:
            if self.live and self.pi is not None:
                self.pi.stop()


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--live", action="store_true")
    parser.add_argument("--enable-pump", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--max-runtime-s", type=float, default=0.0)
    return parser.parse_args()


def run_self_test(cfg):
    solution = target_solution({
        "class_name": "dock", "color": "green",
        "confidence": 0.9, "color_fraction": 0.4,
        "x_right_m": -0.2, "y_forward_m": 5.0, "z_up_m": 0.1,
        "range_m": math.sqrt(0.2 ** 2 + 5.0 ** 2 + 0.1 ** 2),
    }, cfg)
    hardware = Hardware(cfg, live=False, pump_authorized=False)
    hardware.aim(solution["pan_pulse_us"], solution["tilt_pulse_us"])
    assert hardware.pump_on() is False
    hardware.close()
    print(json.dumps({"self_test": "PASS", "gpio_touched": False,
                      "green_only": True, "pump_duration_s": 10.0,
                      "pump_fired": False, "example": solution}, indent=2))


def main():
    args = parse_args()
    cfg = load_config(args.config)
    if args.self_test:
        run_self_test(cfg)
        return
    if args.max_runtime_s < 0:
        raise ValueError("--max-runtime-s cannot be negative")

    pump_authorized = bool(args.live and args.enable_pump)
    if args.enable_pump and not args.live:
        raise RuntimeError("--enable-pump requires --live")
    if pump_authorized:
        if args.confirm != PUMP_CONFIRM:
            raise RuntimeError(f"Pump mode requires --confirm {PUMP_CONFIRM}")
        if not cfg["pump"].get("enabled"):
            raise RuntimeError("Pump is disabled in config")
        if not arm_file_is_valid(cfg):
            raise RuntimeError(f"Arm file missing or invalid: {cfg['pump']['arm_file']}")
    elif args.live and args.confirm != SERVO_CONFIRM:
        raise RuntimeError(f"Servo-only live mode requires --confirm {SERVO_CONFIRM}")

    key = Path(cfg["network"]["key_file"]).read_text().strip().encode()
    if len(key) < 32:
        raise RuntimeError("HMAC key must contain at least 32 characters")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((str(cfg["network"]["bind_ip"]), int(cfg["network"]["port"])))
    sock.settimeout(0.10)
    hardware = Hardware(cfg, live=args.live, pump_authorized=pump_authorized)
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    log_path = LOG_DIR / ("green_receiver_" +
                          datetime.now().strftime("%Y%m%d_%H%M%S") + ".jsonl")
    log_file = log_path.open("a", buffering=1)
    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True
        hardware.pump_off()
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    start = time.monotonic()
    last_valid_packet = 0.0
    last_session = None
    last_sequence = -1
    watchdog_safe = False
    last_pan = None
    last_tilt = None
    stable_packets = 0
    stable_since = 0.0
    fired_for_lock = False
    last_fire = -1e9
    firing = False
    fire_deadline = 0.0
    mode = ("LIVE_PUMP_10S" if pump_authorized else
            "LIVE_SERVOS_PUMP_OFF" if args.live else "DRY_RUN")
    print(f"receiver={mode} bind={cfg['network']['bind_ip']}:"
          f"{cfg['network']['port']} color=green "
          f"pan={cfg['gpio_bcm']['pan_servo']} tilt={cfg['gpio_bcm']['tilt_servo']} "
          f"pump={cfg['gpio_bcm']['pump_trigger']}", flush=True)

    def log(event, **extra):
        item = {"utc": utc_now(), "event": event, **extra}
        line = json.dumps(item, separators=(",", ":"))
        log_file.write(line + "\n")
        print(line, flush=True)

    def reset_lock():
        nonlocal last_pan, last_tilt, stable_packets, stable_since, fired_for_lock
        last_pan = None
        last_tilt = None
        stable_packets = 0
        stable_since = 0.0
        fired_for_lock = False

    try:
        while not stop:
            if args.max_runtime_s and time.monotonic() - start >= args.max_runtime_s:
                log("MAX_RUNTIME")
                break
            if firing and time.monotonic() >= fire_deadline:
                hardware.pump_off()
                firing = False
                log("PUMP_10S_COMPLETE_OFF", pump="OFF")
            try:
                packet, address = sock.recvfrom(65535)
            except socket.timeout:
                if (last_valid_packet and time.monotonic() - last_valid_packet
                        > float(cfg["network"]["watchdog_s"])
                        and not watchdog_safe):
                    was_firing = firing
                    hardware.safe_timeout()
                    firing = False
                    reset_lock()
                    watchdog_safe = True
                    log("GREEN_FEED_TIMEOUT_PUMP_OFF" if was_firing
                        else "WATCHDOG_CENTER_PUMP_OFF", pump="OFF")
                continue
            if address[0] != str(cfg["network"]["allowed_sender_ip"]):
                log("DROP_WRONG_SOURCE", source=address[0])
                continue
            try:
                body = decode_verified(packet, key)
                if body.get("protocol") != "dock_color_aim.v1":
                    raise ValueError("WRONG_PROTOCOL")
                age_s = (time.time_ns() - int(body["sent_unix_ns"])) / 1e9
                if age_s < -0.5 or age_s > float(cfg["network"]["maximum_packet_age_s"]):
                    raise ValueError("STALE_PACKET")
                session = str(body["session"])
                sequence = int(body["sequence"])
                if session == last_session and sequence <= last_sequence:
                    raise ValueError("REPLAY_OR_OUT_OF_ORDER")
                if session != last_session:
                    if firing:
                        hardware.pump_off()
                        firing = False
                        log("SENDER_SESSION_CHANGED_PUMP_OFF", pump="OFF")
                    last_session, last_sequence = session, -1
                    reset_lock()
                last_sequence = sequence
                last_valid_packet = time.monotonic()
                watchdog_safe = False

                if body.get("status") != "OK" or not body.get("target"):
                    if firing:
                        hardware.pump_off()
                        firing = False
                        log("GREEN_LOST_PUMP_OFF", sequence=sequence,
                            status=body.get("status"), pump="OFF")
                    reset_lock()
                    log("SAFE_NO_TARGET", sequence=sequence,
                        status=body.get("status"), pump="OFF")
                    continue

                solution = target_solution(body["target"], cfg)
                hardware.aim(solution["pan_pulse_us"], solution["tilt_pulse_us"])
                angle_delta = float(cfg["tracking"]["stable_angle_delta_deg"])
                if (last_pan is None or last_tilt is None
                        or abs(solution["pan_deg"] - last_pan) > angle_delta
                        or abs(solution["tilt_deg"] - last_tilt) > angle_delta):
                    stable_packets = 1
                    stable_since = time.monotonic()
                else:
                    stable_packets += 1
                last_pan = solution["pan_deg"]
                last_tilt = solution["tilt_deg"]

                stable = (stable_packets >= int(cfg["tracking"]["stable_packets_required"])
                          and time.monotonic() - stable_since
                          >= float(cfg["tracking"]["settle_time_s"]))
                cooldown_ok = (time.monotonic() - last_fire
                               >= float(cfg["pump"]["cooldown_s"]))
                can_fire = (pump_authorized and stable and cooldown_ok
                            and not firing and not fired_for_lock
                            and arm_file_is_valid(cfg))
                event = "AIM"
                if can_fire:
                    if hardware.pump_on():
                        firing = True
                        fire_deadline = time.monotonic() + min(
                            float(cfg["pump"]["requested_duration_s"]),
                            float(cfg["pump"]["absolute_maximum_duration_s"]),
                        )
                        last_fire = time.monotonic()
                        fired_for_lock = True
                        event = "PUMP_ON_GREEN"
                elif not args.live and stable:
                    event = "DRY_RUN_READY_10S"

                log(event, sequence=sequence, mode=mode, pump=("ON" if firing else "OFF"),
                    color=solution["color"], stable_packets=stable_packets,
                    x_right_m=round(solution["x_right_m"], 3),
                    y_forward_m=round(solution["y_forward_m"], 3),
                    z_up_m=round(solution["z_up_m"], 3),
                    pan_deg=round(solution["pan_deg"], 3),
                    tilt_deg=round(solution["tilt_deg"], 3),
                    pan_pulse_us=solution["pan_pulse_us"],
                    tilt_pulse_us=solution["tilt_pulse_us"])
            except Exception as exc:
                hardware.pump_off()
                firing = False
                reset_lock()
                log("DROP_INVALID_GREEN_PUMP_OFF", reason=str(exc), pump="OFF")
    finally:
        hardware.close()
        sock.close()
        log_file.close()
        print(f"receiver stopped safely; pump=OFF log={log_path}", flush=True)


if __name__ == "__main__":
    main()

