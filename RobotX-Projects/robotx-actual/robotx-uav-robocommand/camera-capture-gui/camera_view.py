#!/usr/bin/env python3
"""
Graphical camera viewer for the Orin NX: a live preview window with buttons
to take a photo or record video.

This is a Tkinter window, not a terminal app - it needs an actual display to
show up on: a monitor plugged into the Orin, or a remote desktop/VNC session
(e.g. over Tailscale). It will not work over a plain SSH-only connection.

Reuses camera_link.py for the camera handling - the same module the
terminal tools (camera_monitor.py, and the drone GUI's 'camera'/'photo'
command) use. This file only builds the window and wires buttons to it.
"""

import argparse
import tkinter as tk
from pathlib import Path
from tkinter import font as tkfont

import cv2
from PIL import Image, ImageTk

from camera_link import CameraLink

UI_REFRESH_MS = 33  # ~30 fps preview redraw


class CameraViewApp:
    def __init__(self, root: tk.Tk, device: str, output_dir: Path, rotation_deg: float = 0.0):
        self.root = root
        self.root.title("Camera Viewer")
        self.root.geometry("900x650")
        self.root.minsize(480, 360)

        self.camera = CameraLink(device, output_dir, rotation_deg=rotation_deg)
        self.camera.start()

        self._photo_image = None  # keep a reference so Tk doesn't garbage-collect it
        self._message = f"Opening camera (device {device})..."
        self._message_color = "white"

        bold_font = tkfont.Font(weight="bold", size=11)

        self.video_label = tk.Label(root, bg="black")
        self.video_label.pack(fill="both", expand=True)

        self.status_var = tk.StringVar(value=self._message)
        status_bar = tk.Label(root, textvariable=self.status_var, anchor="w", font=bold_font, padx=8, pady=4)
        status_bar.pack(fill="x")
        self.status_bar = status_bar

        button_row = tk.Frame(root, pady=6)
        button_row.pack(fill="x")

        self.photo_button = tk.Button(button_row, text="Take Photo", command=self.take_photo, font=bold_font)
        self.photo_button.pack(side="left", padx=8)

        self.record_button = tk.Button(
            button_row, text="Start Recording", command=self.toggle_recording, font=bold_font
        )
        self.record_button.pack(side="left", padx=8)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._update_frame()

    def _update_frame(self) -> None:
        frame = self.camera.get_latest_frame()
        status = self.camera.status()

        if frame is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            label_w = self.video_label.winfo_width() or image.width
            label_h = self.video_label.winfo_height() or image.height
            image = image.copy()
            image.thumbnail((max(label_w, 1), max(label_h, 1)))
            self._photo_image = ImageTk.PhotoImage(image=image)
            self.video_label.configure(image=self._photo_image)

        if status.opened:
            line = f"{status.width}x{status.height}   {status.fps:.1f} fps   frames {status.frame_count}"
            if status.recording:
                line += "   ● RECORDING"
        else:
            line = "NOT OPEN"
            if status.last_error:
                line += f" - {status.last_error}"

        # The last button-press result (self._message) stays visible until
        # the next action, appended after the live stats.
        self.status_var.set(f"{line}    |    {self._message}")
        self.status_bar.configure(fg=self._message_color)

        self.root.after(UI_REFRESH_MS, self._update_frame)

    def take_photo(self) -> None:
        ok, message = self.camera.capture()
        self._set_message(message, ok)

    def toggle_recording(self) -> None:
        if self.camera.status().recording:
            ok, message = self.camera.stop_recording()
            if ok:
                self.record_button.configure(text="Start Recording")
        else:
            ok, message = self.camera.start_recording()
            if ok:
                self.record_button.configure(text="Stop Recording")
        self._set_message(message, ok)

    def _set_message(self, message: str, ok: bool) -> None:
        self._message = message
        self._message_color = "#4caf50" if ok else "#f44336"

    def on_close(self) -> None:
        if self.camera.status().recording:
            self.camera.stop_recording()
        self.camera.stop()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--device",
        default="0",
        help="Camera device: an index (e.g. 0) or a path (e.g. /dev/video2). Default: %(default)s",
    )
    parser.add_argument(
        "--output-dir",
        default=str(Path(__file__).resolve().parent / "captures"),
        help="Directory to save photos/videos in (default: %(default)s)",
    )
    parser.add_argument(
        "--rotate",
        type=float,
        default=0.0,
        help="Rotate every frame this many degrees clockwise, to correct a "
        "physically mis-mounted camera (default: %(default)s, no rotation)",
    )
    args = parser.parse_args()

    root = tk.Tk()
    CameraViewApp(root, args.device, Path(args.output_dir), rotation_deg=args.rotate)
    root.mainloop()


if __name__ == "__main__":
    main()
