#!/usr/bin/env python3
"""Render the observer's local North/East cone landmarks as a PNG."""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", default="", help="pipe_cone_map.json; empty selects the latest map log")
    p.add_argument("--output", default="", help="output PNG; defaults beside the input JSON")
    p.add_argument("--map-root", default="/home/jetson/pipe_cones/logs/map")
    p.add_argument("--size", type=int, default=1000)
    p.add_argument("--margin-m", type=float, default=2.0)
    p.add_argument("--watch", action="store_true", help="continuously refresh the PNG")
    p.add_argument("--period-s", type=float, default=0.05)
    return p.parse_args()


def latest_map(root: Path):
    candidates = sorted(root.glob("*/pipe_cone_map.json"))
    if not candidates:
        raise FileNotFoundError(f"No pipe_cone_map.json under {root}")
    return candidates[-1]


def font(size, bold=False):
    names = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
    ]
    for name in names:
        if Path(name).exists():
            return ImageFont.truetype(name, size)
    return ImageFont.load_default()


def gps_status_text(gps):
    age = gps.get("age_s")
    if age is None:
        return "GPS: waiting for telemetry   Satellites: unknown"
    if age > 3.0:
        return "GPS: stale telemetry   Satellites: unknown"
    fixes = {0: "No GPS", 1: "No fix", 2: "2D fix", 3: "3D fix", 4: "DGPS", 5: "RTK float", 6: "RTK fixed", 7: "Static", 8: "PPP"}
    fix = fixes.get(gps.get("fix_type"), "Unknown")
    sats = gps.get("satellites_visible")
    return f"GPS: {fix}   Satellites: {sats if sats is not None and sats != 255 else 'unknown'}"


def render(map_path: Path, output_path: Path, size: int, margin_m: float):
    data = map_path if isinstance(map_path, dict) else json.loads(map_path.read_text())
    vehicle = data.get("vehicle", {})
    landmarks = data.get("landmarks", [])
    vehicle_n = float(vehicle.get("north_m", 0.0))
    vehicle_e = float(vehicle.get("east_m", 0.0))
    yaw_deg = float(vehicle.get("yaw_deg", 0.0))

    points = [(vehicle_n, vehicle_e)] + [
        (float(item["north_m"]), float(item["east_m"])) for item in landmarks
    ]
    max_extent = max(
        5.0,
        max(max(abs(n - vehicle_n), abs(e - vehicle_e)) for n, e in points) + margin_m,
    )

    canvas = Image.new("RGB", (size, size), (247, 249, 252))
    draw = ImageDraw.Draw(canvas)
    title_font = font(max(24, size // 32), bold=True)
    label_font = font(max(12, size // 65), bold=True)
    small_font = font(max(15, size // 55))
    plot_left, plot_top, plot_right, plot_bottom = 90, 100, size - 55, size - 85
    plot_w, plot_h = plot_right - plot_left, plot_bottom - plot_top

    def pixel(north, east):
        x = plot_left + (east - vehicle_e + max_extent) / (2 * max_extent) * plot_w
        y = plot_bottom - (north - vehicle_n + max_extent) / (2 * max_extent) * plot_h
        return x, y

    def fading_trace(points, colour):
        for index, point in enumerate(points):
            x, y = pixel(*point)
            strength = 0.92 ** (len(points)-1-index)
            tint = tuple(int(247*(1-strength)+c*strength) for c in colour)
            if plot_left <= x <= plot_right and plot_top <= y <= plot_bottom:
                draw.ellipse((x-3,y-3,x+3,y+3), fill=tint)

    # Grid at a readable whole-metre interval.
    raw_step = (2 * max_extent) / 10
    grid_step = 1 if raw_step <= 1 else 2 if raw_step <= 2 else 5 if raw_step <= 5 else 10
    limit = int(math.ceil(max_extent / grid_step)) * grid_step
    for offset in range(-limit, limit + grid_step, grid_step):
        x, _ = pixel(vehicle_n, vehicle_e + offset)
        _, y = pixel(vehicle_n + offset, vehicle_e)
        if plot_left <= x <= plot_right:
            draw.line((x, plot_top, x, plot_bottom), fill=(218, 224, 231), width=1)
            draw.text((x + 3, plot_bottom + 6), f"{offset:+d}", fill=(75, 85, 99), font=small_font)
        if plot_top <= y <= plot_bottom:
            draw.line((plot_left, y, plot_right, y), fill=(218, 224, 231), width=1)
            draw.text((plot_left - 42, y - 8), f"{offset:+d}", fill=(75, 85, 99), font=small_font)

    draw.rectangle((plot_left, plot_top, plot_right, plot_bottom), outline=(80, 91, 105), width=2)
    draw.text((90, 24), "Vehicle and pipe-cone map", fill=(25, 35, 50), font=title_font)
    draw.text((size // 2 - 45, 67), "NORTH", fill=(35, 75, 120), font=small_font)
    draw.text((size - 105, size // 2), "EAST", fill=(35, 75, 120), font=small_font)
    rates = data.get("rates_hz", {})
    rate_text = f"CV: {float(rates.get('cv', 0.0)):.1f} Hz\nObserver: {float(rates.get('observer', 0.0)):.1f} Hz"
    rate_box = (size - 255, 18, size - 25, 79)
    draw.rounded_rectangle(rate_box, radius=10, fill=(232, 238, 246), outline=(155, 170, 190), width=2)
    draw.multiline_text((size - 238, 27), rate_text, fill=(35, 55, 80), font=small_font, spacing=3)

    # Vehicle marker with heading arrow.
    fading_trace(data.get("vehicle_trace", []), (25,95,190))
    vx, vy = pixel(vehicle_n, vehicle_e)
    draw.ellipse((vx - 10, vy - 10, vx + 10, vy + 10), fill=(25, 95, 190), outline="white", width=2)
    yaw = math.radians(yaw_deg)
    arrow_len = 55
    hx = vx + arrow_len * math.sin(yaw)
    hy = vy - arrow_len * math.cos(yaw)
    draw.line((vx, vy, hx, hy), fill=(25, 95, 190), width=5)
    draw.text((vx + 14, vy + 8), f"Vehicle (EKF yaw)\n{yaw_deg:.1f} deg", fill=(25, 65, 130), font=small_font)

    for item in landmarks:
        north = float(item["north_m"])
        east = float(item["east_m"])
        marker = "P"
        colour = (180, 45, 170)
        fading_trace(item.get("trace", []), colour)
        x, y = pixel(north, east)
        radius = max(10, size // 80)
        draw.ellipse((x - radius, y - radius, x + radius, y + radius), fill=colour, outline="white", width=3)
        box = draw.textbbox((0, 0), marker, font=label_font)
        tw, th = box[2] - box[0], box[3] - box[1]
        draw.text((x - tw / 2, y - th / 2 - 3), marker, fill="white", font=label_font)
        caption = f"#{item.get('landmark_id')}  N {north:.2f}  E {east:.2f} m"
        draw.text((x + radius + 6, y - 9), caption, fill=colour, font=small_font)
        spread = item.get("spread_rms_m")
        if spread is not None:
            draw.text((x + radius + 6, y + 14), f"Spread RMS {spread:.2f} m", fill=colour, font=small_font)
        latitude, longitude = item.get("latitude_deg"), item.get("longitude_deg")
        if latitude is not None and longitude is not None:
            draw.text(
                (x + radius + 6, y + 37),
                f"{float(latitude):.7f}, {float(longitude):.7f}",
                fill=colour,
                font=small_font,
            )

    source = vehicle.get("pose_source", "unknown")
    gps = data.get("gps", {})
    vehicle_lat, vehicle_lon = vehicle.get("latitude_deg"), vehicle.get("longitude_deg")
    gps_coordinates = "GPS coordinates: waiting"
    if vehicle_lat is not None and vehicle_lon is not None:
        gps_coordinates = f"Vehicle GPS: {float(vehicle_lat):.7f}, {float(vehicle_lon):.7f}"
    footer = f"{gps_status_text(gps)}   Pipe cones: {len(landmarks)}"
    draw.text((plot_left, size - 65), f"Pose: {source}", fill=(55, 65, 78), font=small_font)
    draw.text((plot_left + 210, size - 65), gps_coordinates, fill=(55, 65, 78), font=small_font)
    draw.text((plot_left, size - 40), footer, fill=(55, 65, 78), font=small_font)
    if output_path is None:
        return canvas
    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary = output_path.with_name(f"{output_path.stem}.tmp.{os.getpid()}.png")
    canvas.save(temporary, format="PNG")
    temporary.replace(output_path)
    return len(landmarks), max_extent


def main():
    args = parse_args()
    while True:
        started = time.monotonic()
        map_path = Path(args.input) if args.input else latest_map(Path(args.map_root))
        output_path = Path(args.output) if args.output else map_path.with_name("cone_map.png")
        count, extent = render(map_path, output_path, args.size, args.margin_m)
        print(f"Rendered {count} landmarks to {output_path} (half-width {extent:.1f} m)", flush=True)
        if not args.watch:
            break
        time.sleep(max(0.001, args.period_s - (time.monotonic()-started)))


if __name__ == "__main__":
    main()
