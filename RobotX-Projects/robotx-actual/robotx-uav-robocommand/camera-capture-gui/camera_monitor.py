#!/usr/bin/env python3
"""
Terminal GUI for a USB camera on the Orin NX: shows live camera status and
lets you save photos to disk on command.

There's no live video preview here - that's not practical over a text
terminal/SSH session. This just confirms the feed is alive (resolution, FPS,
frame count) and saves a JPEG wherever you point --output-dir.

The camera handling (open/read frames/save JPEGs) lives in camera_link.py -
this file only renders status and turns typed commands into calls on that
module.
"""

import argparse
from pathlib import Path

from rich.console import Group
from rich.text import Text
from textual.app import App, ComposeResult
from textual.containers import Horizontal
from textual.widgets import Input, RichLog, Static

from camera_link import CameraLink, CameraStatus

REFRESH_INTERVAL = 0.25  # seconds

HELP_TEXT = """\
Commands:
  help               show this list
  photo              save a photo (auto-named by timestamp)
  photo <name>       save a photo under a specific filename
  reconnect          drop and reopen the camera device
  quit               exit
"""

COMMANDS_SUMMARY = [
    ("help", "full command list"),
    ("photo [name]", "save a photo"),
    ("reconnect", "reopen the camera device"),
    ("quit", "exit"),
]


def _section_header(title: str) -> Text:
    return Text(title, style="bold underline")


def render_status_section(status: CameraStatus) -> Text:
    out = Text()
    out.append_text(_section_header("CAMERA"))
    out.append("\n")

    out.append(f"  device {status.device}   ")
    if status.opened:
        out.append("OPEN", style="bold green")
    else:
        out.append("NOT OPEN", style="bold red")
    out.append("\n")

    if status.width and status.height:
        out.append(f"  resolution {status.width}x{status.height}   ")
    else:
        out.append("  resolution -   ")
    out.append(f"fps {status.fps:.1f}   frames seen {status.frame_count}\n")

    age = f"{status.last_frame_age:.1f}s ago" if status.last_frame_age is not None else "never"
    out.append(f"  last frame: {age}")

    if status.last_saved_path:
        out.append(f"\n  last photo: {status.last_saved_path}", style="green")

    if status.last_error:
        out.append(f"\n  error: {status.last_error}", style="red")

    return out


def render_commands_section() -> Text:
    out = Text()
    out.append_text(_section_header("COMMANDS"))
    out.append("\n")
    for name, desc in COMMANDS_SUMMARY:
        out.append(f"  {name:<16}", style="bold")
        out.append(f"{desc}\n")
    return out


def render_dashboard(status: CameraStatus) -> Group:
    return Group(render_status_section(status))


class CameraApp(App):
    CSS = """
    Screen {
        background: $surface;
    }
    #main-row {
        height: auto;
    }
    #dashboard {
        width: 1fr;
        height: auto;
        padding: 0 1;
    }
    #commands {
        width: 40;
        height: auto;
        padding: 0 1;
    }
    #log {
        height: 1fr;
        padding: 0 1;
        border-top: solid $panel;
    }
    #command {
        border: none;
        background: $surface;
    }
    """

    def __init__(self, device: str, output_dir: Path, rotation_deg: float = 0.0):
        super().__init__()
        self.rotation_deg = rotation_deg
        self.camera = CameraLink(device, output_dir, rotation_deg=rotation_deg)

    def compose(self) -> ComposeResult:
        with Horizontal(id="main-row"):
            yield Static(id="dashboard")
            yield Static(render_commands_section(), id="commands")
        yield RichLog(id="log", wrap=True, markup=True)
        yield Input(placeholder="command>", id="command")

    def on_mount(self) -> None:
        self.camera.start()
        self.query_one("#log", RichLog).write(
            f"Camera link started (device {self.camera.device}, saving to {self.camera.output_dir}). "
            "Type 'help' for commands."
        )
        self.set_interval(REFRESH_INTERVAL, self.refresh_status)
        self.query_one("#command", Input).focus()

    def refresh_status(self) -> None:
        status = self.camera.status()
        self.query_one("#dashboard", Static).update(render_dashboard(status))

    def on_input_submitted(self, event: Input.Submitted) -> None:
        if event.input.id != "command":
            return
        text = event.value.strip()
        event.input.value = ""
        if not text:
            return
        log = self.query_one("#log", RichLog)
        log.write(f"> {text}")
        self.dispatch_command(text, log)

    def dispatch_command(self, text: str, log: RichLog) -> None:
        parts = text.split()
        cmd = parts[0].lower()
        args = parts[1:]

        if cmd in ("quit", "q", "exit"):
            self.exit()
        elif cmd == "help":
            log.write(HELP_TEXT)
        elif cmd == "photo":
            ok, message = self.camera.capture(args[0] if args else None)
            log.write(Text(message, style="green" if ok else "red"))
        elif cmd == "reconnect":
            self.camera.stop()
            self.camera = CameraLink(self.camera.device, self.camera.output_dir, rotation_deg=self.rotation_deg)
            self.camera.start()
            log.write("Reconnecting...")
        else:
            log.write(f"Unknown command: {cmd} (type 'help')")


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
        help="Directory to save photos in (default: %(default)s)",
    )
    parser.add_argument(
        "--rotate",
        type=float,
        default=0.0,
        help="Rotate every frame this many degrees clockwise, to correct a "
        "physically mis-mounted camera (default: %(default)s, no rotation)",
    )
    args = parser.parse_args()

    app = CameraApp(args.device, Path(args.output_dir), rotation_deg=args.rotate)
    try:
        app.run()
    finally:
        app.camera.stop()


if __name__ == "__main__":
    main()
