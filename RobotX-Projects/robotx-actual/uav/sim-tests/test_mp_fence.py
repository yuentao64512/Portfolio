"""Mission Planner fence change -> new Run Request: fake FCU (Mission Planner writes a fence at t=12 s),
listener, bridge, a fake RoboCommand (course + capturing requests), run_declaration.py first."""
import json, os, socket, subprocess, sys, time
_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEN = os.path.join(_ROOT, "robocommand", "RobotX_2026", "gen", "python")
G = os.path.join(_ROOT, "geofence-tools")
sys.path[:0] = [GEN]
import paho.mqtt.client as mqtt
import common_pb2
from robotx import rx_course_pb2, rx_requests_pb2
scenario, declare_first = sys.argv[1], sys.argv[2] == "declare"
PY = sys.executable
LAST = os.path.join(G, "last_run_declaration.json")
for _f in (LAST, os.path.join(G, "uav_fence_config.json")):
    if os.path.exists(_f): os.remove(_f)
COURSE = [(1.4120, 103.9130), (1.4130, 103.9130), (1.4130, 103.9140), (1.4120, 103.9140), (1.4120, 103.9130)]
procs = []
def start(cmd, **kw):
    p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, **kw); procs.append(p); return p
start([PY, "-u", "broker.py"]); time.sleep(3)
gui = socket.socket(socket.AF_INET, socket.SOCK_DGRAM); gui.bind(("127.0.0.1", 14671)); gui.settimeout(0.2)
got = []
c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="fake-rc")
c.on_message = lambda cl, u, msg: got.append(msg.payload)
c.connect("127.0.0.1", 18830); c.subscribe("robocommand/robotx/SGIT/request", qos=1); c.loop_start()
LL = common_pb2.LatLng
c.publish("robocommand/robotx/course", rx_course_pb2.RxCourse(course_id="TEST", corners=[LL(latitude=a, longitude=b) for a, b in COURSE]).SerializeToString(), qos=1, retain=True)
t0 = time.time()
fake = start(["python", "-u", "fake_copter.py", "14581", scenario])
lis = start(["python", "-u", os.path.join(G, "geofence_udp_listener.py"), "--connection", "udp:127.0.0.1:14581", "--listen-port", "14621",
       "--preset-fence-file", os.path.join(G, "preset_fence_example.csv"), "--notify", "127.0.0.1:14671", "--robocommand-state", "rc_state_mp.json"])
time.sleep(1)
bridge = start([PY, "-u", os.path.join(G, "robocommand_bridge.py"), "--port", "18830", "--listener-host", "127.0.0.1", "--listener-port", "14621"], stdin=subprocess.PIPE)
def drain(until):
    while time.time() - t0 < until:
        try: print(f"  t={time.time()-t0:4.1f}s GUI:", json.loads(gui.recv(4096))["text"])
        except socket.timeout: pass
drain(7)
pub_again = lambda: c.publish("robocommand/robotx/course", rx_course_pb2.RxCourse(course_id="TEST-AGAIN", corners=[LL(latitude=a, longitude=b) for a, b in COURSE]).SerializeToString(), qos=1, retain=True)
if declare_first:
    out = subprocess.run([PY, os.path.join(G, "run_declaration.py"), "--yes", "--wait", "10"], capture_output=True, text=True, timeout=60)
    print(f"  t={time.time()-t0:4.1f}s run_declaration.py:", (out.stdout.strip().splitlines() or [""])[-1:] + [out.stderr.strip()[-300:]])
drain(10); pub_again(); drain(36 if scenario.startswith("mpsilent") else 30 if scenario == "mpfence_armed" else 26)
c.loop_stop()
for p in procs: p.kill()
print("  --- Run Requests RoboCommand received:")
for payload in got:
    r = rx_requests_pb2.RxRequest(); r.ParseFromString(payload); d = r.run_declaration
    print(f"    #{r.seq} vehicles {list(d.vehicle_ids)} tiers {[d.task1_tier, d.task2_tier, d.task3_tier, d.task4_tier]} "
          f"fence {[(round(p.latitude, 4), round(p.longitude, 4)) for p in d.uav_geofence]} ceiling {d.uav_geofence_max_altitude_hae_m} pilot {d.uav_pilot_callsign}")
print("  --- bridge:", [l for l in bridge.stdout.read().splitlines() if "Run Request" in l or "ignored" in l or "ARMED" in l or "FENCE_ALT_MAX" in l or "REJECT" in l])
print("  --- listener:", [l for l in lis.stdout.read().splitlines() if "another program" in l or "RoboCommand: Run" in l or "NOT sent" in l or "Error" in l])
print("  --- fcu:", [l for l in fake.stdout.read().splitlines() if "fence stored" in l or "Mission Planner" in l or "disarm" in l or "FENCE_ALT_MAX" in l or "clear" in l or "param" in l])
for f in (LAST, "rc_state_mp.json", os.path.join(G, "uav_fence_config.json")):
    if os.path.exists(f): os.remove(f)
