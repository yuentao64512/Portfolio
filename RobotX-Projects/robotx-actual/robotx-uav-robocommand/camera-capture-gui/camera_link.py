"""
USB camera link: opens a camera with OpenCV and grabs frames on a background
thread, tracks live status (resolution/FPS/frame count), and saves JPEG
snapshots to disk on command.

This module knows nothing about how it's displayed - it just exposes a
thread-safe snapshot that any GUI (terminal, web, desktop...) can poll, plus
a capture() call that any command handler can trigger.
"""

import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import cv2


@dataclass(frozen=True)
class CameraStatus:
    """A point-in-time, read-only view of the camera link."""

    opened: bool
    device: str
    width: Optional[int]
    height: Optional[int]
    fps: float
    frame_count: int
    last_frame_age: Optional[float]
    last_error: Optional[str]
    last_saved_path: Optional[str]
    last_saved_time: Optional[float]
    recording: bool
    recording_path: Optional[str]


class CameraLink:
    """Opens a USB camera and reads frames on a background thread."""

    def __init__(self, device: str, output_dir: Path, rotation_deg: float = 0.0):
        self.device = device
        self.output_dir = output_dir
        # Corrects for a physically mis-mounted camera: every captured frame
        # is rotated by this many degrees (clockwise) before it's shown,
        # saved, or recorded. 0 = no rotation. Not a multiple of 90, so the
        # canvas is expanded to fit the whole rotated image (see
        # _rotate_frame) rather than cropping it - the new corners fill
        # with black, which is expected.
        self.rotation_deg = rotation_deg

        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._cap: Optional[cv2.VideoCapture] = None
        self._opened = False
        self._width: Optional[int] = None
        self._height: Optional[int] = None
        self._latest_frame = None  # most recent frame (numpy array), for capture()
        self._frame_count = 0
        self._last_frame_time: Optional[float] = None
        self._fps_window: list = []
        self._fps = 0.0
        self._last_error: Optional[str] = None
        self._last_saved_path: Optional[str] = None
        self._last_saved_time: Optional[float] = None

        self._video_writer: Optional[cv2.VideoWriter] = None
        self._video_path: Optional[Path] = None

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()

    def status(self) -> CameraStatus:
        """Thread-safe, read-only view of the current camera status."""
        now = time.time()
        with self._lock:
            return CameraStatus(
                opened=self._opened,
                device=self.device,
                width=self._width,
                height=self._height,
                fps=self._fps,
                frame_count=self._frame_count,
                last_frame_age=(now - self._last_frame_time) if self._last_frame_time is not None else None,
                last_error=self._last_error,
                last_saved_path=self._last_saved_path,
                last_saved_time=self._last_saved_time,
                recording=self._video_writer is not None,
                recording_path=str(self._video_path) if self._video_path else None,
            )

    def get_latest_frame(self):
        """Thread-safe copy of the most recent frame (BGR numpy array), or None."""
        with self._lock:
            return None if self._latest_frame is None else self._latest_frame.copy()

    def capture(self, filename: Optional[str] = None) -> Tuple[bool, str]:
        """Save the most recent frame as a JPEG. Returns (ok, message)."""
        with self._lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            opened = self._opened

        if not opened:
            return False, "camera is not open - nothing to capture"
        if frame is None:
            return False, "no frame received yet - try again in a moment"

        self.output_dir.mkdir(parents=True, exist_ok=True)
        if filename is None:
            filename = time.strftime("photo_%Y%m%d_%H%M%S.jpg")
        elif not filename.lower().endswith((".jpg", ".jpeg")):
            filename += ".jpg"
        out_path = self.output_dir / filename

        ok = cv2.imwrite(str(out_path), frame)
        if not ok:
            return False, f"failed to write {out_path}"

        with self._lock:
            self._last_saved_path = str(out_path)
            self._last_saved_time = time.time()
        return True, f"saved {out_path}"

    def start_recording(self, filename: Optional[str] = None) -> Tuple[bool, str]:
        """Start writing incoming frames to an MP4 file. Returns (ok, message)."""
        with self._lock:
            if self._video_writer is not None:
                return False, "already recording"
            if not self._opened or self._latest_frame is None:
                return False, "camera is not open - nothing to record"
            height, width = self._latest_frame.shape[:2]
            fps = self._fps if self._fps >= 1.0 else 20.0

            self.output_dir.mkdir(parents=True, exist_ok=True)
            if filename is None:
                filename = time.strftime("video_%Y%m%d_%H%M%S.mp4")
            elif not filename.lower().endswith(".mp4"):
                filename += ".mp4"
            out_path = self.output_dir / filename

            writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
            if not writer.isOpened():
                return False, f"failed to open video writer for {out_path}"

            self._video_writer = writer
            self._video_path = out_path
        return True, f"recording to {out_path}"

    def stop_recording(self) -> Tuple[bool, str]:
        """Stop the current recording and finalize the file. Returns (ok, message)."""
        with self._lock:
            if self._video_writer is None:
                return False, "not recording"
            writer = self._video_writer
            path = self._video_path
            self._video_writer = None
            self._video_path = None
        writer.release()
        return True, f"saved {path}"

    def _rotate_frame(self, frame):
        """Rotate a frame by self.rotation_deg (clockwise), expanding the
        canvas so nothing gets cropped off the corners (they fill with
        black instead)."""
        height, width = frame.shape[:2]
        center = (width / 2, height / 2)
        matrix = cv2.getRotationMatrix2D(center, -self.rotation_deg, 1.0)

        cos = abs(matrix[0, 0])
        sin = abs(matrix[0, 1])
        new_width = int(height * sin + width * cos)
        new_height = int(height * cos + width * sin)

        matrix[0, 2] += (new_width / 2) - center[0]
        matrix[1, 2] += (new_height / 2) - center[1]

        return cv2.warpAffine(frame, matrix, (new_width, new_height))

    def _open_device(self):
        # A plain digit string ("0") means a device index; anything else
        # (e.g. "/dev/video2") is treated as a path.
        target = int(self.device) if self.device.isdigit() else self.device
        return cv2.VideoCapture(target)

    def _run(self) -> None:
        while not self._stop_event.is_set():
            try:
                cap = self._open_device()
                if not cap.isOpened():
                    raise RuntimeError(f"could not open camera device '{self.device}'")
            except Exception as e:
                with self._lock:
                    self._opened = False
                    self._last_error = str(e)
                time.sleep(2)
                continue

            with self._lock:
                self._cap = cap
                self._opened = True
                self._last_error = None
                self._width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or None
                self._height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or None

            while not self._stop_event.is_set():
                ok, frame = cap.read()
                now = time.time()

                if not ok or frame is None:
                    with self._lock:
                        self._opened = False
                        self._last_error = "lost connection to the camera"
                    break

                if self.rotation_deg:
                    frame = self._rotate_frame(frame)

                with self._lock:
                    self._latest_frame = frame
                    # Reflects the frame as actually stored/shown/saved - after
                    # rotation, not the camera's raw reported size.
                    self._height, self._width = frame.shape[:2]
                    self._frame_count += 1
                    self._last_frame_time = now
                    self._fps_window.append(now)
                    cutoff = now - 5.0
                    self._fps_window = [t for t in self._fps_window if t >= cutoff]
                    if len(self._fps_window) >= 2:
                        self._fps = (len(self._fps_window) - 1) / (self._fps_window[-1] - self._fps_window[0])
                    writer = self._video_writer

                if writer is not None:
                    writer.write(frame)

            cap.release()
            with self._lock:
                self._cap = None
                # Don't leave a half-written, unplayable file if the camera
                # drops (or the link stops) mid-recording.
                if self._video_writer is not None:
                    self._video_writer.release()
                    self._video_writer = None
                    self._video_path = None
            if not self._stop_event.is_set():
                time.sleep(2)  # brief pause before retrying the device
