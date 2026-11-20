#!/usr/bin/env python3
"""Small Guided-mode controller for the separate ArduCopter SITL drone."""

from __future__ import annotations

import argparse
import sys
import time

from pymavlink import mavutil


def connect(address: str):
    connection = mavutil.mavlink_connection(address, source_system=250)
    heartbeat = connection.wait_heartbeat(timeout=15)
    print(
        f"Connected to drone system={connection.target_system} "
        f"mode={mavutil.mode_string_v10(heartbeat)}"
    )
    return connection


def armed(connection) -> bool:
    heartbeat = connection.messages.get("HEARTBEAT")
    return bool(
        heartbeat is not None
        and heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def set_mode(connection, mode: str) -> None:
    mapping = connection.mode_mapping() or {}
    mode_id = mapping.get(mode)
    if mode_id is None:
        raise RuntimeError(f"Drone did not advertise {mode} mode")
    connection.mav.set_mode_send(
        connection.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id,
    )
    end = time.monotonic() + 8.0
    while time.monotonic() < end:
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if message is not None and mavutil.mode_string_v10(message) == mode:
            print(f"mode={mode}")
            return
    raise RuntimeError(f"Drone did not enter {mode} mode")


def arm(connection) -> None:
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        1.0,
        0.0,
        0,
        0,
        0,
        0,
        0,
    )
    end = time.monotonic() + 10.0
    while time.monotonic() < end:
        message = connection.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
        if message is not None and armed(connection):
            print("armed=true")
            return
    raise RuntimeError("Drone did not arm")


def send_body_velocity(connection, forward: float, right: float, down: float) -> None:
    # Ignore position, acceleration, yaw and yaw-rate; use only BODY_NED vx/vy/vz.
    type_mask = 0x0FC7
    connection.mav.set_position_target_local_ned_send(
        int(time.monotonic() * 1000) & 0xFFFFFFFF,
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        type_mask,
        0.0,
        0.0,
        0.0,
        float(forward),
        float(right),
        float(down),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )


def takeoff(connection, altitude: float) -> None:
    if not armed(connection):
        raise RuntimeError("Drone is disarmed; use the launcher takeoff command, which arms it")
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        altitude,
    )
    print(f"takeoff_altitude={altitude:.1f}m")


def move(connection, forward: float, right: float, down: float, duration: float) -> None:
    set_mode(connection, "GUIDED")
    if not armed(connection):
        raise RuntimeError("Drone is disarmed; take off first")
    end = time.monotonic() + duration
    while time.monotonic() < end:
        send_body_velocity(connection, forward, right, down)
        time.sleep(0.1)
    for _ in range(10):
        send_body_velocity(connection, 0.0, 0.0, 0.0)
        time.sleep(0.1)
    print("velocity_command_complete=true")


def hold(connection, duration: float) -> None:
    move(connection, 0.0, 0.0, 0.0, duration)


def land(connection) -> None:
    set_mode(connection, "LAND")
    print("landing=true")


def disarm(connection) -> None:
    connection.mav.command_long_send(
        connection.target_system,
        connection.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0,
        0.0,
        0.0,
        0,
        0,
        0,
        0,
        0,
    )
    print("disarm_requested=true")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--connection", default="tcp:127.0.0.1:5760")
    subparsers = parser.add_subparsers(dest="action", required=True)

    takeoff_parser = subparsers.add_parser("takeoff")
    takeoff_parser.add_argument("altitude", type=float)
    takeoff_parser.add_argument("--arm", action="store_true")

    move_parser = subparsers.add_parser("move")
    move_parser.add_argument("forward", type=float)
    move_parser.add_argument("right", type=float)
    move_parser.add_argument("down", type=float)
    move_parser.add_argument("duration", type=float)

    hold_parser = subparsers.add_parser("hold")
    hold_parser.add_argument("duration", type=float)
    subparsers.add_parser("land")
    subparsers.add_parser("disarm")
    args = parser.parse_args()

    if args.action in {"move", "hold"} and args.duration <= 0:
        parser.error("duration must be positive")
    if args.action == "takeoff" and args.altitude <= 0:
        parser.error("altitude must be positive")

    try:
        connection = connect(args.connection)
        if args.action == "takeoff":
            set_mode(connection, "GUIDED")
            if args.arm and not armed(connection):
                arm(connection)
            takeoff(connection, args.altitude)
        elif args.action == "move":
            move(connection, args.forward, args.right, args.down, args.duration)
        elif args.action == "hold":
            hold(connection, args.duration)
        elif args.action == "land":
            land(connection)
        elif args.action == "disarm":
            disarm(connection)
        return 0
    except Exception as exc:
        print(f"Drone control failed: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
