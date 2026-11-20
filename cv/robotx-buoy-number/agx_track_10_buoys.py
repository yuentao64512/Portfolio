#!/usr/bin/env python3
"""Track ten numbered buoys using two TensorRT engines plus OpenCV colour.

Expected scene:
  * exactly ten numbered buoys, numbered 0 through 9 with no repeats
  * one steady blue light, one blinking blue light
  * remaining lights are red or green

Pipeline:
  1. A buoy TensorRT engine detects and tracks whole buoys with ByteTrack.
  2. A number TensorRT engine reads digit markers inside each buoy crop.
  3. OpenCV detects red/green/blue caps with camera-tolerant HSV ranges.
  4. Temporal voting and a global one-to-one assignment prevent duplicate digits.
  5. One combined outer box is drawn around the buoy, digit and light cap.

The canonical reported colours are RGB, even though OpenCV draws in BGR:
red=(255,0,0), green=(0,255,0), blue=(0,0,255).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import time
from collections import Counter, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Deque, Dict, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np
from ultralytics import YOLO


Box = Tuple[int, int, int, int]

RGB = {
    "red": (255, 0, 0),
    "green": (0, 255, 0),
    "blue": (0, 0, 255),
    "blinking_blue": (0, 0, 255),
    "unknown": None,
}

BGR = {
    "red": (0, 0, 255),
    "green": (0, 255, 0),
    "blue": (255, 0, 0),
    "blinking_blue": (255, 80, 20),
    "unknown": (190, 190, 190),
}


def parse_source(value: str):
    """Interpret a plain integer as a camera index; otherwise keep a path/URI."""
    return int(value) if value.isdigit() else value


def clamp_box(box: Sequence[float], width: int, height: int) -> Box:
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    return (
        max(0, min(width - 1, x1)),
        max(0, min(height - 1, y1)),
        max(1, min(width, x2)),
        max(1, min(height, y2)),
    )


def union_boxes(boxes: Iterable[Optional[Box]], width: int, height: int) -> Box:
    valid = [box for box in boxes if box is not None]
    if not valid:
        return (0, 0, 1, 1)
    return clamp_box(
        (
            min(box[0] for box in valid),
            min(box[1] for box in valid),
            max(box[2] for box in valid),
            max(box[3] for box in valid),
        ),
        width,
        height,
    )


def expand_buoy_box(box: Box, width: int, height: int) -> Box:
    """Include the light cap that a whole-buoy detector may clip off."""
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    return clamp_box(
        (
            x1 - 0.10 * bw,
            y1 - 0.60 * bh,
            x2 + 0.10 * bw,
            y2 + 0.10 * bh,
        ),
        width,
        height,
    )


def largest_component(mask: np.ndarray) -> Tuple[int, Optional[Tuple[int, int, int, int]]]:
    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    if count <= 1:
        return 0, None
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    x, y, w, h, area = [int(v) for v in stats[index]]
    return area, (x, y, x + w, y + h)


def detect_cap_colour(frame: np.ndarray, buoy_box: Box):
    """Detect a coloured cap above the centre of a tracked buoy.

    Exact primary RGB equality is too brittle for compressed camera frames, so
    HSV tolerances are used and results are mapped back to the requested RGB
    tuples.
    """
    frame_h, frame_w = frame.shape[:2]
    x1, y1, x2, y2 = buoy_box
    bw, bh = x2 - x1, y2 - y1
    cx = (x1 + x2) / 2.0
    roi_box = clamp_box(
        (
            cx - 0.20 * bw,
            y1 - 0.60 * bh,
            cx + 0.20 * bw,
            y1 + 0.15 * bh,
        ),
        frame_w,
        frame_h,
    )
    rx1, ry1, rx2, ry2 = roi_box
    roi = frame[ry1:ry2, rx1:rx2]
    if roi.size == 0:
        return "unknown", 0.0, None

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    red = cv2.bitwise_or(
        cv2.inRange(hsv, (0, 90, 55), (12, 255, 255)),
        cv2.inRange(hsv, (168, 90, 55), (179, 255, 255)),
    )
    masks = {
        "red": red,
        "green": cv2.inRange(hsv, (38, 85, 45), (88, 255, 255)),
        "blue": cv2.inRange(hsv, (96, 90, 45), (140, 255, 255)),
    }

    candidates = []
    roi_area = max(1, roi.shape[0] * roi.shape[1])
    for colour, mask in masks.items():
        area, local_box = largest_component(mask)
        if local_box is None:
            continue
        score = area / roi_area
        candidates.append((area, score, colour, local_box))

    if not candidates:
        return "unknown", 0.0, None
    area, score, colour, local_box = max(candidates, key=lambda item: item[0])
    min_area = max(4, int(roi_area * 0.0015))
    if area < min_area:
        return "unknown", score, None
    lx1, ly1, lx2, ly2 = local_box
    global_box = (rx1 + lx1, ry1 + ly1, rx1 + lx2, ry1 + ly2)
    return colour, score, global_box


def smooth_binary(values: Sequence[bool]) -> List[bool]:
    values = list(values)
    if len(values) < 3:
        return values
    result = []
    for index in range(len(values)):
        left = max(0, index - 1)
        right = min(len(values), index + 2)
        window = values[left:right]
        result.append(sum(window) * 2 >= len(window))
    return result


def blink_metrics(history: Sequence[bool]):
    smooth = smooth_binary(history)
    if not smooth:
        return 0.0, 0, 0.0
    transitions = sum(a != b for a, b in zip(smooth, smooth[1:]))
    on_fraction = sum(smooth) / len(smooth)
    balance = 1.0 - abs(on_fraction - 0.5) * 2.0
    score = transitions * max(0.0, balance)
    return score, transitions, on_fraction


@dataclass
class TrackState:
    track_id: int
    number_scores: np.ndarray = field(default_factory=lambda: np.zeros(10, dtype=np.float32))
    colour_history: Deque[str] = field(default_factory=lambda: deque(maxlen=90))
    blue_history: Deque[bool] = field(default_factory=lambda: deque(maxlen=60))
    last_seen_frame: int = -1
    frames_seen: int = 0
    buoy_conf: float = 0.0
    buoy_box: Optional[Box] = None
    cap_box: Optional[Box] = None
    digit_box: Optional[Box] = None
    assigned_digit: Optional[int] = None
    assigned_colour: str = "unknown"

    def update_number_scores(self, scores: np.ndarray, decay: float = 0.88):
        self.number_scores *= decay
        self.number_scores += scores

    def recent_colour(self, name: str) -> int:
        return sum(value == name for value in self.colour_history)


def digit_scores_from_result(result) -> Tuple[np.ndarray, Optional[Tuple[int, Box, float]]]:
    """Return maximum confidence per digit and the strongest digit box."""
    scores = np.zeros(10, dtype=np.float32)
    strongest = None
    if result.boxes is None:
        return scores, strongest
    for box in result.boxes:
        class_id = int(box.cls.item())
        name = str(result.names[class_id])
        if name not in set("0123456789") or len(name) != 1:
            continue
        digit = int(name)
        confidence = float(box.conf.item())
        scores[digit] = max(scores[digit], confidence)
        local_box = tuple(int(round(value)) for value in box.xyxy[0].tolist())
        if strongest is None or confidence > strongest[2]:
            strongest = (digit, local_box, confidence)
    return scores, strongest


def unique_digit_assignment(states: Sequence[TrackState], minimum_score: float) -> Dict[int, int]:
    """Maximum-score one-to-one assignment using a small bitmask DP.

    At most ten visible tracks are passed in, so O(10 * 2^10 * 10) is tiny and
    avoids a SciPy dependency on the Jetson.
    """
    if not states:
        return {}
    count = min(10, len(states))
    chosen = list(states[:count])
    # Each DP entry maps used-digit mask to (score, assigned digit list).
    dp: Dict[int, Tuple[float, List[Optional[int]]]] = {0: (0.0, [])}
    for state in chosen:
        next_dp: Dict[int, Tuple[float, List[Optional[int]]]] = {}
        for mask, (total, assignment) in dp.items():
            # Unknown is always allowed and does not consume a digit.
            current = next_dp.get(mask)
            candidate = (total, assignment + [None])
            if current is None or candidate[0] > current[0]:
                next_dp[mask] = candidate
            for digit in range(10):
                if mask & (1 << digit):
                    continue
                score = float(state.number_scores[digit])
                if score < minimum_score:
                    continue
                new_mask = mask | (1 << digit)
                candidate = (total + score, assignment + [digit])
                current = next_dp.get(new_mask)
                if current is None or candidate[0] > current[0]:
                    next_dp[new_mask] = candidate
        dp = next_dp
    _, best_assignment = max(dp.values(), key=lambda item: item[0])
    return {
        state.track_id: digit
        for state, digit in zip(chosen, best_assignment)
        if digit is not None
    }


def assign_scene_colours(states: Sequence[TrackState], enforce_two_blue: bool):
    """Assign temporal colours, optionally enforcing one steady and one blinking blue."""
    for state in states:
        non_unknown = [value for value in state.colour_history if value != "unknown"]
        state.assigned_colour = Counter(non_unknown).most_common(1)[0][0] if non_unknown else "unknown"

    if not enforce_two_blue or len(states) < 2:
        for state in states:
            score, transitions, on_fraction = blink_metrics(state.blue_history)
            if (
                len(state.blue_history) >= 20
                and transitions >= 3
                and 0.12 <= on_fraction <= 0.88
                and state.recent_colour("blue") >= 3
            ):
                state.assigned_colour = "blinking_blue"
        return

    candidates = [
        state for state in states
        if len(state.blue_history) >= 20 and state.recent_colour("blue") >= 3
    ]
    if len(candidates) < 2:
        return

    def blink_key(state: TrackState):
        score, transitions, on_fraction = blink_metrics(state.blue_history)
        return (score, transitions, -abs(on_fraction - 0.5))

    blinking = max(candidates, key=blink_key)
    remaining = [state for state in candidates if state.track_id != blinking.track_id]
    steady = max(
        remaining,
        key=lambda state: (
            sum(state.blue_history) / max(1, len(state.blue_history)),
            state.recent_colour("blue"),
        ),
    )
    blinking.assigned_colour = "blinking_blue"
    steady.assigned_colour = "blue"
    for state in candidates:
        if state.track_id in {blinking.track_id, steady.track_id}:
            continue
        red_green = [value for value in state.colour_history if value in {"red", "green"}]
        state.assigned_colour = Counter(red_green).most_common(1)[0][0] if red_green else "unknown"


def draw_label(frame: np.ndarray, box: Box, lines: Sequence[str], colour: Tuple[int, int, int]):
    x1, y1, x2, y2 = box
    cv2.rectangle(frame, (x1, y1), (x2, y2), colour, 2)
    scale, thickness = 0.52, 2
    line_height = 20
    top = max(0, y1 - line_height * len(lines) - 4)
    max_width = max(
        cv2.getTextSize(line, cv2.FONT_HERSHEY_SIMPLEX, scale, thickness)[0][0]
        for line in lines
    )
    cv2.rectangle(frame, (x1, top), (min(frame.shape[1] - 1, x1 + max_width + 8), y1),
                  (15, 15, 15), -1)
    for index, line in enumerate(lines):
        y = top + 16 + index * line_height
        cv2.putText(frame, line, (x1 + 4, y), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, colour, thickness, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--buoy-engine", required=True, help="AGX-built buoy TensorRT .engine")
    parser.add_argument("--number-engine", required=True, help="AGX-built 0-9 TensorRT .engine")
    parser.add_argument("--source", default="0", help="Camera index, video path or stream URI")
    parser.add_argument("--output", default="agx_10_buoys_annotated.mp4")
    parser.add_argument("--log", default="agx_10_buoys_detections.csv")
    parser.add_argument("--summary", default="agx_10_buoys_summary.json")
    parser.add_argument("--buoy-imgsz", type=int, default=1280)
    parser.add_argument("--number-imgsz", type=int, default=320)
    parser.add_argument("--buoy-conf", type=float, default=0.12)
    parser.add_argument("--number-conf", type=float, default=0.08)
    parser.add_argument("--number-min-score", type=float, default=0.15)
    parser.add_argument("--classify-every", type=int, default=5,
                        help="Run number engine every N frames per track")
    parser.add_argument("--expected", type=int, default=10)
    parser.add_argument("--accepted-classes", default="number_buoy,boundary",
                        help="Comma-separated buoy-engine classes to keep; empty keeps all")
    parser.add_argument("--tracker", default="bytetrack.yaml")
    parser.add_argument("--track-buffer", type=int, default=30)
    parser.add_argument("--no-enforce-blue-layout", action="store_true",
                        help="Do not enforce one steady and one blinking blue track")
    parser.add_argument("--show", action="store_true", help="Display live window; Q/Esc stops")
    parser.add_argument("--no-save", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0,
                        help="0 means run until video ends or user stops")
    args = parser.parse_args()

    if args.classify_every < 1:
        raise SystemExit("--classify-every must be at least 1")

    buoy_model = YOLO(args.buoy_engine, task="detect")
    number_model = YOLO(args.number_engine, task="detect")
    accepted = {value.strip() for value in args.accepted_classes.split(",") if value.strip()}

    capture = cv2.VideoCapture(parse_source(args.source))
    if not capture.isOpened():
        raise SystemExit(f"Cannot open source: {args.source}")
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = float(capture.get(cv2.CAP_PROP_FPS))
    if not math.isfinite(fps) or fps <= 1.0:
        fps = 30.0

    writer = None
    if not args.no_save:
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(
            str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height)
        )
        if not writer.isOpened():
            raise SystemExit(f"Cannot create output video: {output}")

    log_path = Path(args.log)
    log_path.parent.mkdir(parents=True, exist_ok=True)
    log_fields = [
        "frame", "time_s", "track_id", "assigned_digit", "assigned_colour",
        "rgb", "buoy_conf", "x1", "y1", "x2", "y2",
    ]
    states: Dict[int, TrackState] = {}
    frame_index = 0
    started = time.time()

    with log_path.open("w", newline="", encoding="utf-8") as handle:
        log_writer = csv.DictWriter(handle, fieldnames=log_fields)
        log_writer.writeheader()

        while True:
            ok, raw_frame = capture.read()
            if not ok:
                break
            annotated = raw_frame.copy()
            result = buoy_model.track(
                raw_frame,
                persist=True,
                tracker=args.tracker,
                imgsz=args.buoy_imgsz,
                conf=args.buoy_conf,
                iou=0.45,
                agnostic_nms=True,
                verbose=False,
            )[0]

            visible_ids = []
            current = []
            if result.boxes is not None:
                ids = result.boxes.id
                id_values = ids.int().cpu().tolist() if ids is not None else list(range(len(result.boxes)))
                for detection_index, (box, track_id) in enumerate(zip(result.boxes, id_values)):
                    class_id = int(box.cls.item())
                    class_name = str(result.names[class_id])
                    if accepted and class_name not in accepted:
                        continue
                    confidence = float(box.conf.item())
                    buoy_box = clamp_box(box.xyxy[0].tolist(), width, height)
                    current.append((confidence, int(track_id), buoy_box))

            # When false positives exist, keep only the ten most confident objects.
            current.sort(reverse=True, key=lambda item: item[0])
            current = current[: args.expected]

            for confidence, track_id, buoy_box in current:
                visible_ids.append(track_id)
                state = states.setdefault(track_id, TrackState(track_id=track_id))
                state.last_seen_frame = frame_index
                state.frames_seen += 1
                state.buoy_conf = confidence
                state.buoy_box = buoy_box
                state.digit_box = None

                colour, colour_score, cap_box = detect_cap_colour(raw_frame, buoy_box)
                state.colour_history.append(colour)
                state.blue_history.append(colour == "blue")
                state.cap_box = cap_box

                should_classify = (
                    state.frames_seen == 1
                    or frame_index % args.classify_every == track_id % args.classify_every
                )
                if should_classify:
                    crop_box = expand_buoy_box(buoy_box, width, height)
                    cx1, cy1, cx2, cy2 = crop_box
                    crop = raw_frame[cy1:cy2, cx1:cx2]
                    if crop.size:
                        digit_result = number_model.predict(
                            crop,
                            imgsz=args.number_imgsz,
                            conf=args.number_conf,
                            iou=0.45,
                            verbose=False,
                        )[0]
                        scores, strongest = digit_scores_from_result(digit_result)
                        state.update_number_scores(scores)
                        if strongest is not None:
                            _, local_box, _ = strongest
                            dx1, dy1, dx2, dy2 = local_box
                            state.digit_box = clamp_box(
                                (cx1 + dx1, cy1 + dy1, cx1 + dx2, cy1 + dy2),
                                width,
                                height,
                            )

            # Retain recently missing tracks for ByteTrack recovery, but only draw visible ones.
            stale = [
                track_id for track_id, state in states.items()
                if frame_index - state.last_seen_frame > args.track_buffer
            ]
            for track_id in stale:
                del states[track_id]

            visible_states = [states[track_id] for track_id in visible_ids if track_id in states]
            visible_states.sort(
                key=lambda state: (float(state.number_scores.max()), state.buoy_conf),
                reverse=True,
            )
            assignments = unique_digit_assignment(visible_states, args.number_min_score)
            for state in visible_states:
                state.assigned_digit = assignments.get(state.track_id)

            assign_scene_colours(
                visible_states,
                enforce_two_blue=not args.no_enforce_blue_layout,
            )

            for state in visible_states:
                assert state.buoy_box is not None
                combined = union_boxes(
                    [state.buoy_box, state.cap_box, state.digit_box], width, height
                )
                digit_text = str(state.assigned_digit) if state.assigned_digit is not None else "?"
                colour = state.assigned_colour
                rgb_value = RGB.get(colour)
                rgb_text = str(rgb_value) if rgb_value is not None else "unknown"
                blink_score, transitions, on_fraction = blink_metrics(state.blue_history)
                lines = [
                    f"#{digit_text}  {colour}  RGB{rgb_text}",
                    f"track={state.track_id} conf={state.buoy_conf:.2f}",
                ]
                if colour == "blinking_blue":
                    lines.append(f"blink transitions={transitions} on={on_fraction:.2f}")
                draw_label(annotated, combined, lines, BGR.get(colour, BGR["unknown"]))
                x1, y1, x2, y2 = combined
                log_writer.writerow({
                    "frame": frame_index,
                    "time_s": f"{frame_index / fps:.3f}",
                    "track_id": state.track_id,
                    "assigned_digit": digit_text,
                    "assigned_colour": colour,
                    "rgb": rgb_text,
                    "buoy_conf": f"{state.buoy_conf:.6f}",
                    "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                })

            count_colour = (0, 255, 0) if len(visible_states) == args.expected else (0, 200, 255)
            cv2.putText(
                annotated,
                f"BUOYS: {len(visible_states)}/{args.expected}  UNIQUE NUMBERS: {len(assignments)}",
                (20, 36),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                count_colour,
                2,
                cv2.LINE_AA,
            )

            if writer is not None:
                writer.write(annotated)
            if args.show:
                cv2.imshow("AGX 10-buoy tracker", annotated)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break

            frame_index += 1
            if frame_index % 100 == 0:
                elapsed = max(0.001, time.time() - started)
                print(
                    f"frames={frame_index} speed={frame_index / elapsed:.1f}fps "
                    f"buoys={len(visible_states)}/{args.expected} unique={len(assignments)}",
                    flush=True,
                )
            if args.max_frames and frame_index >= args.max_frames:
                break

    capture.release()
    if writer is not None:
        writer.release()
    if args.show:
        cv2.destroyAllWindows()

    final_rows = []
    for state in sorted(states.values(), key=lambda item: item.track_id):
        blink_score, transitions, on_fraction = blink_metrics(state.blue_history)
        final_rows.append({
            "track_id": state.track_id,
            "assigned_digit": state.assigned_digit,
            "assigned_colour": state.assigned_colour,
            "rgb": RGB.get(state.assigned_colour),
            "frames_seen": state.frames_seen,
            "blue_transitions": transitions,
            "blue_on_fraction": round(on_fraction, 4),
            "number_scores": [round(float(value), 4) for value in state.number_scores],
        })
    summary_path = Path(args.summary)
    summary_path.parent.mkdir(parents=True, exist_ok=True)
    summary_path.write_text(json.dumps(final_rows, indent=2), encoding="utf-8")
    elapsed = time.time() - started
    print(f"COMPLETE frames={frame_index} elapsed={elapsed:.1f}s")
    if writer is not None:
        print(f"video={args.output}")
    print(f"csv={args.log}")
    print(f"summary={args.summary}")


if __name__ == "__main__":
    main()
