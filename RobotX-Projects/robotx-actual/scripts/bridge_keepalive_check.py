"""scripts/bridge_keepalive_check.py (v0.1.86.7, WP43 Tier 1.5): two local mosquittos + a bridge; the hub frozen (SIGSTOP) -> seconds until the boat broker reports the bridge down. python3 scripts/bridge_keepalive_check.py 5 0   (0 = mosquitto default 60)."""
import os, shutil, signal, socket, subprocess, sys, tempfile, time
import paho.mqtt.client as mqtt

MOSQ = shutil.which("mosquitto") or "/opt/homebrew/sbin/mosquitto"


def free_port():
    s = socket.socket(); s.bind(("127.0.0.1", 0)); p = s.getsockname()[1]; s.close(); return p


def run(keepalive):
    d = tempfile.mkdtemp(prefix="bka")
    hp, bp = free_port(), free_port()
    open(f"{d}/hub.conf", "w").write(f"listener {hp} 127.0.0.1\nallow_anonymous true\n")
    ka = f"keepalive_interval {keepalive}\n" if keepalive else ""
    open(f"{d}/boat.conf", "w").write(f"listener {bp} 127.0.0.1\nallow_anonymous true\nconnection ocs\naddress 127.0.0.1:{hp}\n"
                                      f"cleansession false\nnotifications true\nnotification_topic $SYS/broker/connection/ocs/state\n"
                                      f"restart_timeout 1 5\n{ka}topic x/# out 0\n")
    hub = subprocess.Popen([MOSQ, "-c", f"{d}/hub.conf"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(0.5)
    boat = subprocess.Popen([MOSQ, "-c", f"{d}/boat.conf"], stdout=subprocess.DEVNULL, stderr=open(f"{d}/boat.err", "w"))
    state = {"v": None, "t": None}
    c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)

    def on_msg(cl, u, m):
        state["v"], state["t"] = m.payload.decode(), time.monotonic()
    c.on_message = on_msg
    for _ in range(50):
        try:
            c.connect("127.0.0.1", bp); break
        except OSError:
            time.sleep(0.1)
    c.subscribe("$SYS/broker/connection/ocs/state")
    c.loop_start()
    t0 = time.monotonic()
    while state["v"] != "1" and time.monotonic() - t0 < 10:
        time.sleep(0.05)
    assert state["v"] == "1", "bridge never up"
    hub.send_signal(signal.SIGSTOP)
    t_dead = time.monotonic()
    while state["v"] != "0" and time.monotonic() - t_dead < 150:
        time.sleep(0.1)
    down = time.monotonic() - t_dead if state["v"] == "0" else None
    hub.send_signal(signal.SIGCONT)
    c.loop_stop()
    boat.terminate(); hub.terminate(); boat.wait(); hub.wait()
    shutil.rmtree(d, ignore_errors=True)
    return down


for ka in [int(x) for x in sys.argv[1:]]:
    r = run(ka)
    print(f"keepalive_interval {ka or 'default (60)'}: a frozen hub reported down after {r if r is None else round(r, 1)} s", flush=True)
