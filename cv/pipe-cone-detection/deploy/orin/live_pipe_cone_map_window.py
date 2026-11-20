#!/usr/bin/env python3
"""Live local UDP map viewer. Run on the Orin graphical desktop."""
import base64
import json
import socket
import time
import tkinter as tk
from PIL import ImageTk
from render_pipe_cone_map import render


def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 15260))
    sock.setblocking(False)
    root = tk.Tk()
    root.title("Live pipe-cone map — waiting for observer")
    label = tk.Label(root, text="Waiting for observer on localhost:15260", width=80, height=30)
    label.pack(fill="both", expand=True)
    pending = {}
    last_received = [0.0]

    def update():
        newest = None
        for _ in range(300):
            try:
                raw = sock.recv(65535)
            except BlockingIOError:
                break
            try:
                packet = json.loads(raw)
                key = packet["id"]
                if not 0 <= packet["index"] < packet["count"] <= 1000:
                    continue
                entry = pending.setdefault(key, [time.monotonic(), {}, packet["count"]])
                entry[1][packet["index"]] = base64.b64decode(packet["data"])
                if len(entry[1]) == entry[2]:
                    newest = json.loads(b"".join(entry[1][i] for i in range(entry[2])))
                    del pending[key]
            except (ValueError, KeyError, TypeError):
                continue
        for key in list(pending):
            if time.monotonic()-pending[key][0] > 1.0:
                del pending[key]
        if newest is not None:
            label.configure(width=0, height=0)
            label.image = ImageTk.PhotoImage(render(newest, None, 1000, 2.0))
            label.configure(image=label.image)
            last_received[0] = time.monotonic()
            root.title("Live pipe-cone map — " + newest.get("observer_rate_source", "FCU"))
        elif last_received[0] and time.monotonic()-last_received[0] > 2.0:
            root.title("Live pipe-cone map — STALE: observer stream stopped")
        root.after(10, update)

    root.after(0, update)
    try:
        root.mainloop()
    finally:
        sock.close()


if __name__ == "__main__":
    main()
