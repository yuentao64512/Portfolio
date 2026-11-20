# FCU Heartbeat Monitor

A terminal GUI that shows live MAVLink `HEARTBEAT` status between the Holybro
6X Pro flight controller and the Orin NX companion computer. Runs directly on
the Orin NX, reading from a MAVProxy UDP output (it does not connect to the
FCU directly — MAVProxy keeps the primary link).

## How it fits together

```
Holybro 6X Pro (FCU) --Ethernet/MAVLink--> Orin NX: MAVProxy --UDP out--> this GUI
```

MAVProxy owns the actual connection to the FCU. This tool just taps one of
MAVProxy's output endpoints and decodes `HEARTBEAT` messages.

## Files

- `heartbeat_link.py` — the MAVLink side: connects to a MAVLink endpoint on a
  background thread, decodes `HEARTBEAT` messages, exposes a thread-safe
  `HeartbeatSnapshot` for any GUI to read, and handles reconnecting and
  sending commands (arm/disarm, mode change). No GUI code in here at all.
- `heartbeat_monitor.py` — the terminal GUI, built with
  [Textual](https://textual.textualize.io/): a live status panel, a
  scrolling log, and a command input line. Polls/calls `HeartbeatMonitor`
  from `heartbeat_link.py`. This is the file you run.

This GUI also has a `camera`/`photo` command (see below) that imports
`camera_link.py` from the sibling `camera-capture-gui/` project — that
project is the standalone camera tool, kept as separate code; this GUI just
calls into it. The two folders need to stay siblings (both directly under
the same parent directory) for that import to work.

## One-time setup on the Orin NX

```bash
pip3 install -r requirements.txt
```

### Only if you're running MAVProxy separately (Option B below)

In the MAVProxy console (the terminal where MAVProxy is already running),
add a UDP output for this tool if you don't already have a spare one:

```
output add 127.0.0.1:14550
```

If 14550 is already used by another GCS/tool, pick a free port (e.g. 14551)
and pass it to `--connection` below. Skip this if you're using Option A —
the GUI configures MAVProxy's output itself.

## Running

**Option A — let the GUI start MAVProxy for you** (recommended; no separate
terminal needed):

```bash
python3 heartbeat_monitor.py
```

This launches `mavproxy.py --master=udp:0.0.0.0:14555 --out=127.0.0.1:14550 --daemon`
plus `--out=127.0.0.1:14551` (geofence listener), `--out=127.0.0.1:14552`
(spare / lawnmower mission), `--out=192.168.1.100:14550` (Mission Planner on
the GCS laptop) and `--out=127.0.0.1:14553` (flight recorder, `record`)
in the background and connects to it automatically. Change those extra
outputs with `--mavproxy-out` (repeat it once per output). MAVProxy is stopped
automatically when you quit the GUI. Use whatever `--master` value matches
your FCU's actual link (see `NET_P1_TYPE`/`NET_P1_PORT`/`NET_IPADDR` on
ArduPilot boards with Ethernet — for a UDP-client-mode FCU this is
`udp:0.0.0.0:<NET_P1_PORT>`; for TCP it'd be `tcp:<fcu-ip>:<port>`).

**Option B — MAVProxy already running separately:**

```bash
python3 heartbeat_monitor.py --no-mavproxy --connection udp:127.0.0.1:14550
```

The screen has three parts: a status panel on top, a scrolling command log
in the middle, and a command input at the bottom — click/tab into it and
type. Type `help` and press Enter to see the command list. Type `quit` (or
`Ctrl+C`) to exit.

### Options

- `--master` — FCU connection for the auto-launched MAVProxy (default
  `udp:0.0.0.0:14555`, see Option A above).
- `--no-mavproxy` — don't launch MAVProxy; use one that's already running
  (Option B).
- `--mavproxy-out` — extra output for the auto-launched MAVProxy; repeat for
  more. Default: `127.0.0.1:14551`, `127.0.0.1:14552`, `192.168.1.100:14550`.
- `--connection` — MAVLink connection string this GUI listens on (default
  `udp:127.0.0.1:14550`). With `--master`, this also sets where the
  auto-launched MAVProxy sends its output.
- `--timeout` — seconds without a heartbeat before the link shows as LOST
  (default `3.0`).
- `--camera-device` — USB camera for the `camera` command: an index (e.g.
  `0`) or a path (e.g. `/dev/video2`). Default `0`. Not opened until you
  type `camera`.
- `--camera-output-dir` — where `photo` saves JPEGs (default: a `captures`
  folder next to this script).
- `--camera-rotate` — rotate the camera feed this many degrees clockwise, to
  correct a physically mis-mounted camera (e.g. `--camera-rotate 155`).
  Default `0` (no rotation). Any angle works, not just 90/180/270.
- `--notify-port` — local UDP port where other programs on the Orin (the
  geofence listener) send notifications to print in the message log.
  Default `14650`, `0` turns it off. These can't arrive over MAVLink:
  MAVProxy forwards what another program sends only to the FCU, never to
  this GUI's port.
- `--no-listener` — don't launch `geofence-tools/geofence_udp_listener.py`.
  By default the GUI starts it as a separate process once the FCU heartbeat
  is seen, writes its output to `geofence-tools/geofence_udp_listener.log`,
  and restarts it 10 s after it stops. It is stopped when you quit.
- `--listener-connection` — MAVProxy output the listener uses (default
  `udp:127.0.0.1:14551`).
- `--listener-preset` — preset fence CSV for the listener (default
  `geofence-tools/preset_fence_example.csv`).

## What it shows

A flat, plain-text dashboard (no boxes), updating live, with a scrolling
message log and command input below it:

- **Status**: current flight mode and armed state (first line), connected/
  lost state, time since last heartbeat, heartbeat rate, system/component ID,
  autopilot/vehicle type.
- **Attitude & health**: roll/pitch/yaw, altitude above home (plus the
  downward rangefinder's reading if one is fitted), GPS fix type + satellite count
  (green/yellow/red by quality), battery voltage/current/remaining %, CPU
  load, which sensors (if any) are unhealthy, EKF health.
- **RC inputs**: live per-channel PWM values and RSSI.
- **Motor/servo outputs**: a PWM bar graph per channel, same idea as
  `MOTOR PWM OUTPUTS` in MAVProxy's own console.
- **Message log**: scrolls both your typed commands and the vehicle's own
  `STATUSTEXT` messages (prearm checks, warnings, etc.), color-coded by
  severity — critical/error in red, warning in yellow.

The attitude/GPS/RC/servo fields need the FCU to actually be streaming those
message types — this GUI requests them explicitly (via
`MAV_CMD_SET_MESSAGE_INTERVAL`) once connected, so they should show up within
a second or two of `CONNECTED`. It also ignores any heartbeat that isn't from
the real flight controller (e.g. MAVProxy's own heartbeat on a shared link),
so system/component ID and mode always reflect the FCU, not the hub.

## Commands

Type these into the input bar at the bottom:

| Command | Effect |
|---|---|
| `help` | show the command list |
| `reconnect` | drop and reopen the current link |
| `connect <string>` | switch to a different MAVLink connection string |
| `timeout <seconds>` | change the heartbeat-lost timeout |
| `arm` | asks for confirmation; press Enter again (blank) to confirm and arm, or type anything else to cancel. `arm confirm` as one line still works too |
| `disarm` | sends the DISARM command immediately |
| `mode` | show the current flight mode |
| `mode <name>` | request a flight-mode change, e.g. `mode GUIDED` (names are the autopilot's own mode names — an unrecognized one lists the valid options) |
| `<name>` | shortcut: type a mode name directly with no `mode` prefix, e.g. just `GUIDED` (case-insensitive). Anything that isn't a real mode name falls through to "Unknown command" instead |
| `calibrate` | show a numbered menu (`1 accel`, `2 compass`) — then just type the number to start it |
| `calibrate accel` | start accelerometer calibration (6-position). Progress is shown as the live STATUSTEXT prompt ("Place vehicle on its LEFT side...") plus a step count (1-6) — ArduPilot doesn't report an exact percentage for this one |
| `calibrate compass` | start compass calibration. Shows a real progress bar per compass (0-100%) from MAG_CAL_PROGRESS |
| `reboot` | reboot the flight controller; refused while armed. Press Enter again (blank) to confirm |
| `camera` | open the USB camera (device/output dir set via `--camera-device`/`--camera-output-dir`, see below). Not opened automatically — only when you type this |
| `camera stop` | close the camera, freeing the device for another program (e.g. `camera_view.py` in `camera-capture-gui/`) without quitting this GUI |
| `photo [name]` | save a photo from the open camera (auto-named by timestamp, or a name you give) |
| `mission` | fly the lawnmower mission from `lawnmower-mission/` (press Enter again, blank, to confirm). Progress shows in the message log |
| `mission preview` | plan and upload the mission only, no flying - view it in Mission Planner (Plan > Read) |
| `mission stop` | stop the mission script. Doesn't change the flight mode - take over with the RC |
| `mission clear` | delete the mission stored on the FCU (press Enter again, blank, to confirm). Only the mission - the fence and rally points stay. Refused while the mission script runs or the drone is armed in AUTO. Then Plan > Read in Mission Planner to refresh its map |
| `fence clear` | delete all fence points stored on the FCU (press Enter again, blank, to confirm). Only the fence list - the mission, rally points and `FENCE_*` params stay. Refused while armed or while a mission script runs. A running laptop publisher or a new RoboCommand course puts a fence back. Then Plan > Fence > Read in Mission Planner to refresh its map |
| `fence preview` | read the fence stored on the FCU and show its coordinates in the log: each polygon's points (lat, lon, 7 decimals), circles with their radius, the return point. Read only - nothing on the FCU changes; works armed or disarmed. Needs MAVLink 2 |
| `mission2` / `mission3` | the same lawnmower mission (same checks, RC takeover, GUIDED + waypoints cleared after landing). `mission2`: take off to 5 m, hover 15 s, legs at 10 m AGL, 3 m/s, with the CV start signal. `mission3`: 30 s hover, 15 m AGL, 7 m/s - `lawnmower-mission/mission2.py` / `mission3.py`. Press Enter again (blank) to confirm. `mission2 preview` / `mission3 preview` = upload only; `mission stop` / `mission clear` work for them too |
| RoboCommand RunStart (automatic) | when the listener reports `RoboCommand: RUN START`, the GUI flies `lawnmower-mission/runstart_mission.py` by itself (once per run): arm, take off to 5 m, hover 15 s, then mission2's lawnmower (10 m) at 3 m/s, home, land. Not started if the drone is already armed, not connected, or a mission is running. RC takeover works as in every mission. `--no-runstart-mission` turns it off (bench) |
| `record` | record video + the FCU's flight data for every frame (`camera-capture-gui/flight_recorder.py`). A RECORDING section shows duration, file size, frames and the live attitude/position. Saved next to the photos (`--camera-output-dir`) |
| `record stop` | stop recording; the video file is finished so it plays, and the log shows a summary |
| `quit` | exit |
| `quit clear` | exit AND delete the mission and the fence stored on the FCU (press Enter again, blank, to confirm; refused while armed). Runs `fcu_clear.py` through MAVProxy just before it stops; the result prints in the terminal. Mission Planner shows them gone after its next Plan > Read / Fence > Read. Until a new fence is sent the drone has no polygon fence |

With the GUI already closed, the same delete straight from the FCU (nothing else may be using port 14555):
`python3 "/home/uav/Desktop/UAV software/drone-heartbeat-gui/fcu_clear.py"` (`--what mission` for the mission only).

`arm`/`disarm`/`mode` send real MAVLink commands to the vehicle over
whatever link MAVProxy has to the FCU — treat them as you would any other
GCS control, same safety precautions apply (props off, etc. while bench
testing).

## Troubleshooting

- **Stuck on "WAITING FOR HEARTBEAT"**: confirm MAVProxy actually has an
  `output` pointed at the port you passed to `--connection` (check with
  `output` in the MAVProxy console with no arguments to list current outputs).
- **"CONNECTION ERROR"**: the UDP port isn't bound yet — make sure MAVProxy
  is running and the output was added before (or shortly after) you start
  this tool; UDP outputs in MAVProxy only start sending once a packet has
  been seen from that address, so this tool sends nothing itself and just
  listens — that's expected and it will pick up heartbeats as soon as
  MAVProxy forwards them.
