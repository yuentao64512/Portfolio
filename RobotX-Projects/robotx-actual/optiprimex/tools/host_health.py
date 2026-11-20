"""host_health: the 1 Hz host beat of a Jetson (v0.1.43, OCS_V2_SPEC §11).

Publishes `local/usv/host/<host>` (QoS 0, not retained) on the boat broker in the team envelope (kind telemetry, source
host_health); the boat broker's bridge carries it out as `team/optiprime/v1/telemetry/usv/host/<host>` (profiles/*/mosquitto).
Started by scripts/run_usv.sh (host nx, window `host`) and scripts/run_agx.sh (host agx, window `host`).

Body: host · cpu_pct (from /proc/stat) · cpu_load1 (os.getloadavg, where /proc/stat is missing too) · ncpu · gpu_pct (the Tegra
GPU's load file) · temp_c {thermal zone type: deg C} (/sys/class/thermal/thermal_zone*/type, temp) · throttling (a zone at or above
its passive trip point, or a frequency cooling device engaged) · throttle_src · disk_free_gb · uptime_s · t · t_unix.
What cannot be read is null - never invented. Runs on the Mac too (no /sys, no /proc): nulls and what it can (load, disk, uptime).
Stdlib + paho only.

    python -m optiprimex.tools.host_health --host nx --mqtt 127.0.0.1:1884        (--once: print one beat as JSON and exit)

v0.1.55 (WP10f-rest, the overnight brief D3): `--simulated` + `--host nx,agx` (one process, one beat per name) - the bench and the
simulator (scripts/run_bench.sh, node `host`) beat for the NX and the AGX from the Mac, so the pages do not say "no beat"; the body
carries `simulated: true` and the Mac's real numbers, the pages label them "this Mac (simulated host)". On the Jetsons nothing changes.
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time
import uuid
from typing import Dict, Optional, Tuple

TOPIC = "local/usv/host/{host}"
TEGRA_RE = None          # compiled lazily (re)
GPU_LOAD_FILES = ("/sys/devices/gpu.0/load", "/sys/devices/platform/gpu.0/load", "/sys/devices/platform/17000000.ga10b/load",
                  "/sys/devices/17000000.ga10b/load", "/sys/devices/platform/17000000.gv11b/load", "/sys/devices/17000000.gv11b/load")


def utc_iso(t: Optional[float] = None) -> str:
    t = time.time() if t is None else t
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(t)) + f".{int((t % 1) * 1000):03d}Z"


def _read(path: str) -> Optional[str]:
    try:
        with open(path) as fh:
            return fh.read().strip()
    except (OSError, ValueError):
        return None


def _num(path: str) -> Optional[float]:
    s = _read(path)
    try:
        return float(s) if s is not None else None
    except ValueError:
        return None


class Sampler:
    """Reads the host; every method returns None where the host does not tell."""

    def __init__(self, sys_root: str = "/sys", proc_root: str = "/proc", disk_path: str = "/"):
        self.sys_root, self.proc_root, self.disk_path = sys_root.rstrip("/"), proc_root.rstrip("/"), disk_path
        self._cpu_prev: Optional[Tuple[float, float]] = None

    # ---- CPU ------------------------------------------------------------------------------------------------------
    def _cpu_times(self) -> Optional[Tuple[float, float]]:
        s = _read(f"{self.proc_root}/stat")
        if not s:
            return None
        for line in s.splitlines():
            if line.startswith("cpu "):
                v = [float(x) for x in line.split()[1:]]
                idle = v[3] + (v[4] if len(v) > 4 else 0.0)                 # idle + iowait
                return sum(v), idle
        return None

    def cpu_pct(self) -> Optional[float]:
        """Busy share since the previous call (the first call: since boot)."""
        cur = self._cpu_times()
        if cur is None:
            return None
        prev, self._cpu_prev = self._cpu_prev, cur
        tot, idle = (cur[0] - prev[0], cur[1] - prev[1]) if prev else cur
        return round(100.0 * (1.0 - idle / tot), 1) if tot > 0 else None

    @staticmethod
    def cpu_load1() -> Optional[float]:
        try:
            return round(os.getloadavg()[0], 2)
        except (OSError, AttributeError):
            return None

    # ---- GPU ------------------------------------------------------------------------------------------------------
    def gpu_pct(self) -> Optional[float]:
        for f in GPU_LOAD_FILES:
            v = _num(self.sys_root + f[len("/sys"):])
            if v is not None:
                return round(v / 10.0, 1)                                      # the Tegra load file counts 0..1000
        return None

    # ---- thermal ----------------------------------------------------------------------------------------------------
    def _zones(self):
        for d in sorted(glob.glob(f"{self.sys_root}/class/thermal/thermal_zone*")):
            name = _read(f"{d}/type") or os.path.basename(d)
            milli = _num(f"{d}/temp")
            yield d, name, milli

    def temp_c(self) -> Optional[Dict[str, float]]:
        out: Dict[str, float] = {}
        for _, name, milli in self._zones():
            if milli is None:
                continue
            key, n = name, 2
            while key in out:                                                  # two zones of one type: keep both
                key, n = f"{name}#{n}", n + 1
            out[key] = round(milli / 1000.0, 1)
        return out or None

    def throttling(self) -> Tuple[Optional[bool], Optional[str]]:
        """(True, why) when a zone is at or above one of its passive trip points (where the kernel starts throttling) or a
        frequency cooling device is engaged; (False, 'checked') when something was readable and none is; (None, None) otherwise."""
        seen = False
        for d, name, milli in self._zones():
            for tf in glob.glob(f"{d}/trip_point_*_type"):
                if _read(tf) != "passive":
                    continue
                trip = _num(tf[: -len("_type")] + "_temp")
                if trip is None or milli is None:
                    continue
                seen = True
                if milli >= trip:
                    return True, f"{name} {milli / 1000.0:.1f} C >= passive trip {trip / 1000.0:.1f} C"
        for cd in sorted(glob.glob(f"{self.sys_root}/class/thermal/cooling_device*")):
            typ = (_read(f"{cd}/type") or "").lower()
            if "freq" not in typ and "devfreq" not in typ:
                continue
            cur = _num(f"{cd}/cur_state")
            if cur is None:
                continue
            seen = True
            if cur > 0:
                return True, f"cooling device {typ} state {cur:g}"
        return (False, "checked") if seen else (None, None)

    # ---- disk, uptime -------------------------------------------------------------------------------------------------
    def disk_free_gb(self) -> Optional[float]:
        try:
            return round(shutil.disk_usage(self.disk_path).free / 1e9, 1)
        except OSError:
            return None

    def uptime_s(self) -> Optional[float]:
        s = _read(f"{self.proc_root}/uptime")
        if s:
            try:
                return round(float(s.split()[0]), 0)
            except (ValueError, IndexError):
                return None
        if sys.platform == "darwin":                                           # the Mac: kern.boottime { sec = N, usec = M } ...
            try:
                out = subprocess.run(["sysctl", "-n", "kern.boottime"], capture_output=True, text=True, timeout=2).stdout
                sec = float(out.split("sec =")[1].split(",")[0])
                return round(time.time() - sec, 0)
            except (OSError, IndexError, ValueError, subprocess.SubprocessError):
                return None
        return None

    def body(self, host: str, simulated: bool = False) -> dict:
        thr, why = self.throttling()
        now = time.time()
        extra = {"simulated": True} if simulated else {}
        b = {**extra, "host": host, "hostname": socket.gethostname(), "cpu_pct": self.cpu_pct(), "cpu_load1": self.cpu_load1(),
             "ncpu": os.cpu_count(), "gpu_pct": self.gpu_pct(), "temp_c": self.temp_c(), "throttling": thr, "throttle_src": why,
             "disk_free_gb": self.disk_free_gb(), "uptime_s": self.uptime_s(), "t": utc_iso(now), "t_unix": round(now, 2)}
        b.update(self.extra(now))                                      # v0.1.86: tegrastats / meminfo / psutil, fan, power mode, rec rate
        return b

    def extra(self, now: float) -> dict:
        """v0.1.86 (WP42 §2.5): ram_used_mb · ram_total_mb · swap_used_mb · cpu_core_pct · emc_pct · power_w {total, cpu_gpu_cv, soc} · fan_pct ·
        power_mode · temp_max_c · rec_gb_per_10min (null = unknown / not recording). tegrastats wins where it has a field."""
        out = {"ram_used_mb": None, "ram_total_mb": None, "swap_used_mb": None, "cpu_core_pct": None, "emc_pct": None, "power_w": None}
        out.update(meminfo(self.proc_root))
        if not out.get("ram_total_mb"):
            try:
                import psutil                                          # the Mac, if installed
                vm = psutil.virtual_memory()
                out.update(ram_used_mb=int((vm.total - vm.available) / 2 ** 20), ram_total_mb=int(vm.total / 2 ** 20))
            except Exception:
                pass
        ts = getattr(self, "tegra", None)
        tg = ts.sample() if ts is not None else {}
        temps = dict(self.temp_c() or {})
        temps.update(tg.pop("temp_c", {}) or {})
        tg.pop("cpu_pct", None)                                        # /proc/stat stays the cpu_pct source (unchanged); cores from tegrastats
        out.update(tg)
        out["fan_pct"] = fan_pct(self.sys_root)
        out["wifi"] = wifi(self.proc_root)                             # v0.1.86.2: the host's own Wi-Fi signal (None = wired / unknown)
        if now >= getattr(self, "_pm_next", 0.0):
            self._pm, self._pm_next = power_mode(), now + 60.0
        out["power_mode"] = getattr(self, "_pm", None)
        vals = [v for v in temps.values() if isinstance(v, (int, float))]
        out["temp_max_c"] = max(vals) if vals else None
        rr = getattr(self, "rec", None)
        out["rec_gb_per_10min"] = rr.rate(now) if rr is not None else None
        return out


# ---- v0.1.86 (WP42 §2.5): tegrastats, memory, fan, power mode, the recording rate --------------------------------------------------
def parse_tegrastats(line: str) -> dict:
    """One `tegrastats` line -> {ram_used_mb, ram_total_mb, swap_used_mb, cpu_core_pct, cpu_pct, gpu_pct, emc_pct, temp_c, power_w}; a field
    that is not in the line is left out (the caller merges and fills null). Handles the Orin NX (VDD_IN, VDD_CPU_GPU_CV, VDD_SOC) and the
    AGX Orin (VDD_GPU_SOC, VDD_CPU_CV, VIN_SYS_5V0) rails, `GR3D_FREQ 7%@[305,305]` and `GR3D_FREQ 7%@305` / `7%`."""
    import re
    out: dict = {}
    m = re.search(r"RAM (\d+)/(\d+)MB", line)
    if m:
        out["ram_used_mb"], out["ram_total_mb"] = int(m.group(1)), int(m.group(2))
    m = re.search(r"SWAP (\d+)/(\d+)MB", line)
    if m:
        out["swap_used_mb"] = int(m.group(1))
    m = re.search(r"CPU \[([^\]]*)\]", line)
    if m:
        cores = []
        for c in m.group(1).split(","):
            mm = re.match(r"\s*(\d+)%", c)
            cores.append(int(mm.group(1)) if mm else None)          # "off" = a parked core -> null
        out["cpu_core_pct"] = cores
        on = [c for c in cores if c is not None]
        if on:
            out["cpu_pct"] = round(sum(on) / len(on), 1)
    m = re.search(r"GR3D_FREQ (\d+)%", line)
    if m:
        out["gpu_pct"] = float(m.group(1))
    m = re.search(r"EMC_FREQ (\d+)%", line)
    if m:
        out["emc_pct"] = float(m.group(1))
    temps = {}
    for name, val in re.findall(r"([A-Za-z][A-Za-z0-9_]*)@(-?[\d.]+)C\b", line):
        v = float(val)
        if v > -200:                                                  # tegrastats prints -256C for an absent sensor
            temps[name.lower()] = v
    if temps:
        out["temp_c"] = temps
    rails = {name: int(cur) for name, cur in re.findall(r"\b((?:VDD|VIN)[A-Z0-9]*_[A-Z0-9_]+) (\d+)mW", line)}
    if rails:
        tot = rails.get("VDD_IN")
        if tot is None:
            tot = sum(rails.values())
        out["power_w"] = {"total": round(tot / 1000.0, 2),
                          "cpu_gpu_cv": round((rails.get("VDD_CPU_GPU_CV") or rails.get("VDD_CPU_CV") or 0) / 1000.0, 2) or None,
                          "soc": round((rails.get("VDD_SOC") or rails.get("VDD_GPU_SOC") or 0) / 1000.0, 2) or None}
    return out


class Tegrastats:
    """`tegrastats --interval 1000` spawned once; the last line kept (a daemon reader). absent() on a host without it (the Mac)."""

    def __init__(self, exe: Optional[str] = None):
        self.exe = exe or shutil.which("tegrastats")
        self.last: Optional[str] = None
        self.proc = None
        if self.exe:
            try:
                self.proc = subprocess.Popen([self.exe, "--interval", "1000"], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)
                threading.Thread(target=self._read, daemon=True).start()
            except OSError:
                self.proc = None

    def _read(self) -> None:
        for line in self.proc.stdout:
            self.last = line.strip()

    def sample(self) -> dict:
        return parse_tegrastats(self.last) if self.last else {}


def meminfo(proc_root: str = "/proc") -> dict:
    """/proc/meminfo -> ram_used_mb / ram_total_mb / swap_used_mb (Linux without tegrastats); {} elsewhere."""
    txt = _read(f"{proc_root}/meminfo")
    if not txt:
        return {}
    kv = {}
    for ln in txt.splitlines():
        k, _, v = ln.partition(":")
        try:
            kv[k.strip()] = int(v.split()[0])
        except (ValueError, IndexError):
            pass
    if "MemTotal" not in kv:
        return {}
    avail = kv.get("MemAvailable", kv.get("MemFree", 0))
    out = {"ram_total_mb": kv["MemTotal"] // 1024, "ram_used_mb": (kv["MemTotal"] - avail) // 1024}
    if "SwapTotal" in kv:
        out["swap_used_mb"] = (kv["SwapTotal"] - kv.get("SwapFree", 0)) // 1024
    return out


def fan_pct(sys_root: str = "/sys") -> Optional[float]:
    for f in sorted(glob.glob(f"{sys_root}/devices/platform/pwm-fan*/hwmon/*/pwm1")):
        v = _num(f)
        if v is not None:
            return round(v / 255.0 * 100.0, 0)
    for cd in sorted(glob.glob(f"{sys_root}/class/thermal/cooling_device*")):
        if "pwm-fan" in (_read(f"{cd}/type") or ""):
            cur, mx = _num(f"{cd}/cur_state"), _num(f"{cd}/max_state")
            if cur is not None and mx:
                return round(cur / mx * 100.0, 0)
    return None


def power_mode() -> Optional[str]:
    """`nvpmodel -q` (no sudo) -> 'MAXN' / '15W' ...; None when absent or refused."""
    exe = shutil.which("nvpmodel")
    if not exe:
        return None
    try:
        out = subprocess.run([exe, "-q"], capture_output=True, text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return None
    for ln in out.splitlines():
        if "Power Mode" in ln and ":" in ln:
            return ln.split(":", 1)[1].strip() or None
    return None


def wifi(proc_root: str = "/proc") -> Optional[dict]:
    """v0.1.86.2: /proc/net/wireless -> {iface, rssi_dbm, quality} of the first Wi-Fi interface; None on a wired-only host (or the Mac)."""
    txt = _read(f"{proc_root}/net/wireless")
    if not txt:
        return None
    for ln in txt.splitlines()[2:]:
        if ":" not in ln:
            continue
        iface, rest = ln.split(":", 1)
        f = rest.split()
        try:
            q, lvl = float(f[1].rstrip(".")), float(f[2].rstrip("."))
        except (IndexError, ValueError):
            continue
        if lvl > 0:                                                    # some drivers print the level unsigned (256 - dBm)
            lvl -= 256
        return {"iface": iface.strip(), "rssi_dbm": lvl, "quality": q}
    return None


def dir_bytes(path: str) -> int:
    n = 0
    for d, _s, fs in os.walk(path):
        for f in fs:
            try:
                n += os.path.getsize(os.path.join(d, f))
            except OSError:
                pass
    return n


class RecRate:
    """The recording rate measured on disk: the bytes under `path` sampled every `every_s`; GB per 10 min over the last >= 60 s while it
    grows; None while it does not (not recording) or before 60 s of samples. Feeds the HOSTS tab's "minutes of recording left"."""

    def __init__(self, path: Optional[str], every_s: float = 15.0, window_s: float = 300.0):
        self.path, self.every_s, self.window_s = path, every_s, window_s
        self.samples: list = []
        self.next_t = 0.0

    def rate(self, now: Optional[float] = None) -> Optional[float]:
        if not self.path or not os.path.isdir(self.path):
            return None
        now = time.time() if now is None else now
        if now >= self.next_t:
            self.next_t = now + self.every_s
            self.samples = [s for s in self.samples if now - s[0] <= self.window_s] + [(now, dir_bytes(self.path))]
        if len(self.samples) < 2 or self.samples[-1][0] - self.samples[0][0] < 60.0:
            return None
        (t0, b0), (t1, b1) = self.samples[0], self.samples[-1]
        if b1 <= b0:
            return None
        return round((b1 - b0) / (t1 - t0) * 600.0 / 1e9, 2)


def envelope(body: dict, seq: int, vehicle_id: str = "USV1") -> dict:
    """The team envelope (schemas/envelope.schema.json) like the other local/usv/* publishers (fake_perception, usv_ui)."""
    return {"schema_version": 1, "msg_id": str(uuid.uuid4()), "seq": seq, "sent_at": utc_iso(), "run_id": None, "declaration_seq": None,
            "vehicle_id": vehicle_id, "kind": "telemetry", "source": "host_health", "body": body}


def parse_args(argv=None) -> argparse.Namespace:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--host", default="nx", help="the name in the topic: nx | agx (default nx); v0.1.55: a list nx,agx = one beat each")
    ap.add_argument("--simulated", action="store_true", help="v0.1.55: the bench / simulator on the Mac - the body says simulated: true")
    ap.add_argument("--mqtt", default="127.0.0.1:1884", help="the boat broker host:port (default 127.0.0.1:1884)")
    ap.add_argument("--period", type=float, default=1.0, help="seconds between beats (default 1)")
    ap.add_argument("--vehicle-id", default="USV1")
    ap.add_argument("--disk", default=os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                    help="the filesystem whose free space is reported (default: this tree's)")
    ap.add_argument("--rec-dir", default=os.environ.get("REC_DIR") or "", help="v0.1.86: the recordings folder whose growth gives "
                    "rec_gb_per_10min (run_agx.sh: ~/Desktop/dm3/recordings)")
    ap.add_argument("--no-tegrastats", action="store_true", help="v0.1.86: do not spawn tegrastats (the tests)")
    ap.add_argument("--once", action="store_true", help="print one beat (the envelope) and exit; no broker")
    ap.add_argument("--count", type=int, default=0, help="stop after this many beats (0 = run until stopped)")
    return ap.parse_args(argv)


def main(argv=None) -> int:
    a = parse_args(argv)
    smp = Sampler(disk_path=a.disk)
    smp.tegra = None if a.no_tegrastats or a.once else Tegrastats()
    smp.rec = RecRate(os.path.expanduser(a.rec_dir)) if a.rec_dir else None
    hosts = [h.strip() for h in a.host.split(",") if h.strip()] or ["nx"]
    if a.once:
        for h in hosts:
            print(json.dumps(envelope(smp.body(h, a.simulated), 1, a.vehicle_id)), flush=True)
        return 0
    import paho.mqtt.client as mqtt
    host, _, port = a.mqtt.rpartition(":")
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"host-health-{a.host}-{os.getpid()}", protocol=mqtt.MQTTv311)
    except AttributeError:                                                     # paho 1.x
        c = mqtt.Client(client_id=f"host-health-{a.host}-{os.getpid()}", protocol=mqtt.MQTTv311)
    c.reconnect_delay_set(1, 5)
    c.connect_async(host or "127.0.0.1", int(port or 1884), keepalive=15)
    c.loop_start()
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    topics = {h: TOPIC.format(host=h) for h in hosts}
    print(f"[host_health] {a.host}{' (simulated)' if a.simulated else ''} -> {' '.join(topics.values())} on {a.mqtt} every {a.period:g} s", flush=True)
    seq, n = 0, 0
    try:
        while not stop.is_set():
            if c.is_connected():
                for i, h in enumerate(hosts):
                    body = smp.body(h, a.simulated) if i == 0 else dict(body, host=h)    # one sample per beat, shared by the names
                    seq += 1
                    c.publish(topics[h], json.dumps(envelope(body, seq, a.vehicle_id), separators=(",", ":")), qos=0)
                n += 1
                if a.count and n >= a.count:
                    break
            stop.wait(a.period)
    finally:
        c.loop_stop()
        try:
            c.disconnect()
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
