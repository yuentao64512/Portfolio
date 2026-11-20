# Geofence / Parameter Tools

Ground-station and Orin-side command-line scripts for working with
ArduPilot parameters and the polygon geofence over MAVLink, built on two
shared modules.

Keep this folder as a sibling of `drone-heartbeat-gui/` (both directly
under the same parent directory) on the **Orin** - `mavlink_param_link.py`
reuses that project's `is_autopilot_heartbeat()` filter rather than
duplicating it. The GCS laptop doesn't need that folder at all - see
"What goes where" below.

## Files

- `fence_csv.py` — `FenceVertex` + CSV loading. Deliberately has **zero
  dependency on pymavlink or anything MAVLink-related** - this is what lets
  `geofence_udp_publisher.py` run on a GCS laptop with no MAVLink stack
  installed and no `drone-heartbeat-gui/` folder present.
- `mavlink_param_link.py` — shared MAVLink backend (Orin-side only):
  connect, get/set a parameter, watch for parameter changes, upload/
  download a polygon fence. Re-exports `FenceVertex`/`load_fence_csv` from
  `fence_csv.py` so existing imports keep working. No CLI parsing or output
  formatting in here, just the protocol work.
- `param_set.py` — read and/or change any ArduPilot parameter, with a
  confirmation prompt and readback verification.
- `param_listener.py` — passively watch for parameter changes and print
  them live, optionally filtered to a name pattern (e.g. `FENCE_*`).
- `geofence_publisher.py` — **one-shot, local**: upload a polygon fence
  from a CSV file to a vehicle you have a direct MAVLink path to (e.g.
  running on the Orin itself, talking to its own MAVProxy), verify it by
  downloading it back, and optionally enable it. Good for testing or a
  single manual upload.
- `geofence_udp_publisher.py` / `geofence_udp_listener.py` — **live,
  wireless, ground station <-> Orin**: see "Wireless live geofence" below.
  This is the one for "push a fence from my GCS laptop over the air and
  have it apply automatically."
- `your_fence.csv` — the Punggol Digital site fence from the hand-sketched
  5-point layout (point 5 repeats point 1 to close it).
- `mission_fence.csv` — the lawnmower mission area (4 corners, point 5
  closes it) with `max_alt,20`.
- `preset_fence_example.csv` — **placeholder** default/safe fence for the
  live pipeline below. Replace with your real preset before actual use.

A fence CSV may contain one `max_alt,<metres>` line. `geofence_publisher.py`
and `run_declaration.py` use it; the UDP publisher sends it along, but the
live listener doesn't apply it - it only turns on the polygon fence (see
below), leaving `FENCE_ALT_MAX` as it is.

## What goes where

| Machine | Files needed | Needs pymavlink? |
|---|---|---|
| **GCS laptop** | `fence_csv.py`, `geofence_udp_publisher.py`, your fence CSV; for RoboCommand also `robocommand_bridge.py`, `start_bridge.bat`, `run_declaration.py`, `course_fence.py` + RoboNation's `robocommand` repo | No |
| **Orin** | everything in this folder, plus `drone-heartbeat-gui/` as a sibling | Yes |

The GCS laptop only ever runs `geofence_udp_publisher.py` — it talks plain
UDP/JSON, not MAVLink, so it doesn't need `pymavlink` installed or the
`drone-heartbeat-gui/` folder copied over. Everything else
(`geofence_udp_listener.py`, `param_set.py`, `param_listener.py`,
`geofence_publisher.py`) runs on the Orin, where MAVProxy and the MAVLink
link to the FCU actually are.

## One-time setup

**On the Orin:**
```bash
cd geofence-tools
pip3 install -r requirements.txt
```

**On the GCS laptop:** nothing to install — `geofence_udp_publisher.py` and
`fence_csv.py` only use Python's standard library (`socket`, `json`,
`argparse`, `pathlib`). Just copy those two files (plus your fence CSV)
over; any Python 3 install will run it.

## Important technical note: MAVLink2 required

Fence upload/download uses the `mission_type` field on the MISSION_*
messages, which only exists in the MAVLink2 dialect - pymavlink's default
dialect is missing it. `mavlink_param_link.py` sets `MAVLINK20=1` before
importing pymavlink to force the right dialect; this happens automatically
and needs no action from you, but if you ever see fence messages go out
with no effect on the vehicle, check that env var is still being set at the
top of that file before anything else imports pymavlink.

## `param_set.py`

```bash
# read one param
python3 param_set.py --connection udp:127.0.0.1:14550 --get FENCE_ENABLE

# set one param (asks for confirmation first)
python3 param_set.py --set FENCE_ENABLE=1

# set several at once, skip confirmation
python3 param_set.py --set FENCE_ENABLE=1 --set FENCE_ACTION=1 -y

# quick look at the common FENCE_* params
python3 param_set.py --fence-summary
```

Every `--set` reads the current value first, shows you old -> new, asks to
confirm (unless `-y`), then sets it and waits for the vehicle's own
PARAM_VALUE echo to confirm it actually took — ArduPilot can silently clamp
or reject a bad value, so the echo (not just "message sent") is what's
trusted here. A mismatch between what you asked for and what came back is
reported as a failure, not silently accepted.

## `param_listener.py`

```bash
python3 param_listener.py --connection udp:127.0.0.1:14550
python3 param_listener.py --filter "FENCE_*"
```

Requests the full parameter list once at startup (so you see every current
value), then keeps watching for any further PARAM_VALUE messages —
including ones the vehicle sends unprompted whenever something actually
changes it (e.g. you, Mission Planner, or another GCS set a param
elsewhere). Changed values are highlighted. `--filter` limits what's
*printed*, not what's received — so the "initial dump complete" message
still fires at the right time even when your filter excludes the last
parameter in the vehicle's internal ordering.

## `geofence_publisher.py`

```bash
python3 geofence_publisher.py --connection udp:127.0.0.1:14550 --fence-file your_fence.csv

# also enable the fence once the upload is verified
python3 geofence_publisher.py --fence-file your_fence.csv --enable-fence

# upload as an exclusion zone (keep-out) instead of inclusion (allowed area)
python3 geofence_publisher.py --fence-file my_fence.csv --exclusion
```

The fence file is plain `lat,lon` per line — see `your_fence.csv`.
Blank lines and `#` comments are ignored. If the last point repeats the
first (closing the polygon explicitly, like the sketch this was built
from), it's detected and dropped before upload — ArduPilot auto-closes
polygons itself, connecting the last vertex back to the first, so sending
that duplicate point again would be redundant.

What happens, in order:
1. Loads and prints the vertices for review.
2. Asks for confirmation (unless `-y`) — this changes what the vehicle does
   near the fence boundary, so it isn't silent by default.
3. Uploads via the standard MAVLink mission protocol (MISSION_COUNT ->
   MISSION_REQUEST_INT -> MISSION_ITEM_INT -> MISSION_ACK), replacing
   whatever fence was there before.
4. **Downloads the fence back and compares it point-by-point** against what
   was sent. "Upload OK" only means the message round-trip worked — this
   step is what actually confirms the vehicle has the fence you intended.
5. If `--enable-fence` was given and verification passed: checks
   `FENCE_TYPE` includes the polygon bit (ArduPilot convention: bit 2,
   value 4 — adds it if missing) and sets `FENCE_ENABLE=1`.

## Wireless live geofence (GCS laptop <-> Orin)

For when your GCS laptop has no direct MAVLink path to the vehicle — just
WiFi/Tailscale to the Orin. `geofence_udp_publisher.py` runs on the GCS and
sends fence vertices over plain UDP (no MAVLink); `geofence_udp_listener.py`
runs on the Orin, receives them, and does the actual MAVLink upload+verify
(same logic as `geofence_publisher.py`'s, just triggered by network packets
instead of a one-shot CLI run).

**On the Orin** (needs its own MAVProxy output, separate from whatever port
the drone GUI uses — e.g. if the GUI is on `udp:127.0.0.1:14550`, add
another with `output add 127.0.0.1:14551` in the MAVProxy console):

```bash
python3 geofence_udp_listener.py \
    --connection udp:127.0.0.1:14551 \
    --listen-port 14600 \
    --preset-fence-file preset_fence_example.csv
```

**On the GCS laptop:**

```bash
python3 geofence_udp_publisher.py \
    --listener-host <orin-ip-address> \
    --fence-file your_fence.csv
```

The publisher's `--fence-type` and `--enable-fence` are still accepted but
no longer change anything: the live listener always sets the polygon fence
only, switched on (`FENCE_TYPE=4`, `FENCE_ENABLE=1`).

**What happens:**
- On startup, the listener reads the FCU's current `FENCE_TYPE`,
  `FENCE_ENABLE` and `FENCE_ALT_MAX` and remembers them as the **preset**
  (original) values. It refuses to
  start if it can't read them — a reset without them couldn't actually put
  things back.
- While the publisher runs, it re-sends the same fence every second (a
  liveness signal, not a wasteful re-upload — the listener only actually
  touches the FCU when the fence data, type, or enable flag *changes*).
- On each change the listener, in this order: uploads the polygon,
  downloads it back to verify, **then** turns on the polygon fence only:
  `FENCE_TYPE=4` (polygon - no max-altitude or circle fence) and
  `FENCE_ENABLE=1`. `FENCE_ALT_MAX` is not touched.
  Params are only changed after the polygon is verified, so a failed upload
  never leaves ArduPilot checking a fence it doesn't actually have. A
  failed update is retried on the next packet.
- On success it notifies the drone GUI: `[INFO] Geofence UPDATE: 4v T=4 EN=1 OK`
  appears in the GUI's message log (vertices, resulting FENCE_TYPE and
  FENCE_ENABLE). This goes as a small UDP message to the
  GUI's notification port (`--notify`, default `127.0.0.1:14650`, matching
  the GUI's `--notify-port`) - it **can't** go over MAVLink, because
  MAVProxy forwards what the listener sends only to the FCU, never to the
  GUI's output port, and ArduPilot doesn't echo it back down the same link.
  The same text is also sent as a MAVLink `STATUSTEXT`, which ArduPilot
  passes on to any *other* link the FCU has (e.g. a telemetry radio).
- **Stop the publisher (Ctrl+C)** and it sends one final "bye" packet, so
  the listener reverts immediately — preset polygon from
  `--preset-fence-file`, `FENCE_TYPE` and `FENCE_ENABLE` back to the values
  captured at startup: `Geofence RESET-CLOSED: 4v T=2 EN=0 OK`.
- If the publisher instead disappears ungracefully (crash, network drop,
  `kill -9`) with no "bye", the listener notices after
  `--publisher-timeout` seconds (default 5) of silence and does the same
  full revert: `Geofence RESET-TIMEOUT: 4v T=2 EN=0 OK`. This is the
  primary safety mechanism — "bye" is just a faster path for the clean-exit
  case, the timeout is what you can actually rely on.
- **Stop or kill the listener itself** (Ctrl+C, SIGTERM - the drone GUI
  quitting - or closing its terminal) and, if it changed any fence param,
  it puts `FENCE_TYPE`, `FENCE_ENABLE` and `FENCE_ALT_MAX` back to the
  original values from startup, armed or not:
  `Geofence RESTORED: T=2 EN=0 A=100 OK`. The polygon itself stays in the
  FCU's storage (unchecked if the original `FENCE_TYPE` has no 4 bit). Only
  `kill -9` can't be caught.
- The messages are kept short because the MAVLink `STATUSTEXT` copy is
  hard-limited to 50 bytes; full detail for debugging is in the listener's
  own terminal output on the Orin.

## RoboCommand (RobotX 2026) into the listener

RoboCommand - the competition server - only talks to the team's Operator
Control Station (the GCS laptop), over MQTT. `robocommand_bridge.py` runs
on the laptop, receives its messages, decodes them with RoboNation's
generated code, and forwards them to `geofence_udp_listener.py` over the
same UDP link the fence uses (port 14600). Each message is re-sent until the
listener acknowledges it, so a dropped Wi-Fi packet doesn't lose a command.

**On the laptop** (once):
```bash
pip install protobuf==7.35 paho-mqtt==2.0
git clone --depth 1 --filter=blob:none --sparse https://github.com/robonation/robocommand.git
git -C robocommand sparse-checkout set RobotX_2026
```
Then, with the listener running on the Orin:
```bash
python robocommand_bridge.py --broker <robocommand-broker-ip>
```
or double-click `start_bridge.bat` (set `BROKER` in it once). Team `SGIT` and
the Orin at `192.168.1.153` are the defaults (`--team-id`, `--listener-host`
to change them). RoboNation's repo is found by itself when cloned inside
this folder or next to it; otherwise point `--robocommand-gen` at its
`RobotX_2026/gen/python`. To try it without the competition, run
RoboNation's test server from the repo (`docker compose up --build` in
`robocommand/RobotX_2026`) and use `--broker 127.0.0.1`.

**On the Orin** nothing new to install (the bridge decodes; the listener gets
plain JSON) - just `robocommand_handler.py` next to the listener. The
listener then, for every RoboCommand message:
- prints it in the drone GUI's message log - run start, course, and for
  the UAV in yellow: assistance requests, keep-out zones, moving objects
  (messages for other vehicles are shown dimmed as "not this drone");
- saves the latest state to `robocommand_state.json` (course, run start,
  active keep-out zones...) for other scripts to read;
- checks the fence on the FCU against the course: every point must be inside
  the course boundary or the UAV transit area, or RoboCommand refuses the run
  declaration. Rechecked whenever the fence changes.

**UAV heartbeat to RoboCommand.** The listener also sends the FCU's
telemetry back to the bridge once a second (to wherever the bridge's hello,
sent every 2 s, came from), and the bridge publishes it as the UAV's
`Heartbeat` on `robocommand/robotx/<team_id>/UAV1/report`:

| Heartbeat field | From |
|---|---|
| `state` | `STATE_AUTO` in AUTO or GUIDED, else `STATE_MANUAL` - RoboCommand starts a run once every declared vehicle is `STATE_AUTO` |
| `position`, `spd_mps`, `heading_deg`, `roll_deg`, `pitch_deg` | FCU position (EKF) and attitude |
| `altitude_hae_m` | EKF altitude + the GPS's ellipsoid/MSL difference (falls back to MSL, and the bridge says so, if the GPS doesn't report it) |
| `flight_phase` | AIRBORNE while armed and off the ground |
| `current_task` | type `task 1` ... `task 4` / `task none` in the bridge's terminal (start value: `--task`) |
| `uav_registration` | `AA53577` (`DEFAULT_UAV_REGISTRATION` in the bridge; `--uav-registration` to change) |

No FCU heartbeat on the Orin (or no bridge) means no telemetry, so the
heartbeat to RoboCommand stops rather than reporting a stale state - the
bridge prints a warning. Bridge options: `--vehicle-id` (default `UAV1`),
`--uav-registration`, `--task`.

**The fence comes from RoboCommand.** When RoboCommand's course arrives,
the listener makes one polygon of the course boundary + the UAV transit
area (`course_fence.py` - one polygon, because ArduPilot requires the drone
to be inside every inclusion polygon at once) and puts it on the FCU with
only the polygon fence on (`FENCE_ALT_MAX` left as it is):
`Geofence COURSE: 6v T=4 EN=1 OK`. Only while the drone is disarmed -
a course that arrives in flight is applied after landing. It then becomes
the fence a reset returns to, so the laptop publisher can still override it
for a while and it comes back when the publisher stops. If the two areas
can't be merged into one polygon (they don't touch, or they overlap) the GUI
says so and the fence is left alone. `--no-course-fence` turns this off.

**RunDeclaration (laptop, `run_declaration.py` + the bridge).** Once before
every run - and only if your team's operator station doesn't send its own.
`run_declaration.py` only signs it with the vehicles, task tiers, pilot call
sign and the UAV's vehicle id, and hands it to the bridge on this laptop
(local UDP port 14615). The bridge signs it with the drone's own fence
config and sends it:
- geofence = the polygon on the FCU (what Mission Planner shows);
- ceiling = the FCU's `FENCE_ALT_MAX` (set in Mission Planner).

It checks it the way RoboCommand does (closed, 4+ points, within the course
or UAV transit area to 1 m, ceiling above 0 and at most 50 m HAE).
RoboCommand never replies to a refusal, so a request that fails this check
counts as **rejected**: it is NOT sent, and the listener clears the fence
from the FCU and puts its startup `FENCE_TYPE` / `FENCE_ENABLE` /
`FENCE_ALT_MAX` back (`Geofence REJECTED: cleared ...` in the GUI; Mission
Planner: Plan > Fence > Read to see it). Only while disarmed - an armed
drone's fence is never cleared. Keep `FENCE_ALT_MAX` at most 50 m. Needs the bridge running, with the drone GUI on the Orin.
```bash
python run_declaration.py --vehicles USV1 UAV1 --task1 advanced
```
The pilot call sign `Calwin-U2332301` is built in (`DEFAULT_PILOT_CALLSIGN`;
`--pilot-callsign` for another pilot).

**Fence changed in Mission Planner (automatic).** RoboCommand's Heartbeat
has no geofence field - only the Run Request carries one. So when the FCU
accepts a fence written by another program (Mission Planner's Fence > Write,
or the GUI's `fence clear`), the listener reads it back, and a single
inclusion polygon goes to the bridge with the UAV telemetry. The bridge
re-sends the last Run Request it sent (`last_run_declaration.json`) with
that geofence - same vehicles, tiers and pilot, ceiling = the current
`FENCE_ALT_MAX`. Changing `FENCE_ALT_MAX` (e.g. in Mission Planner)
also re-sends it with the new ceiling. It goes through the same check: if
RoboCommand would reject it (e.g. outside the course, ceiling above 50 m
HAE) it is NOT sent and the fence is cleared, startup params back, as
above. The drone GUI shows the result: `RoboCommand: Run Request #... sent
for the Mission Planner fence ...`, the rejection, or why it was NOT sent
(more than one polygon). With no earlier Run Request it declares the UAV
alone - built-in vehicle id and pilot, task tiers none (`run_declaration.py`
declares the full team and tiers). While armed it waits until the drone is
disarmed. So with the bridge left running: remove the RoboCommand course
fence in Mission Planner, plot the new one, Fence > Write - and it goes to
RoboCommand. Fences the listener uploads itself
(laptop publisher, RoboCommand course) don't trigger it.

**The course goes to the drone when it changes.** The bridge forwards RoboCommand's course
to the listener the first time, and again only when its content changes (course id, pinger
frequency, corners or UAV transit area - not the timestamp). A repeat - e.g. the retained
course after a reconnect - is not forwarded, so it doesn't overwrite a fence you edited in
Mission Planner; a *different* course does replace it (once the drone is disarmed). The
listener tries each upload once: if it doesn't verify, it is NOT retried.

To send the cached course again by hand - e.g. after the listener restarted, or after you
cleared the FCU fence - run this on the laptop while the bridge is running:
```bash
python refresh_Rxcourse.py
```
It says "re-sent and acknowledged" once the Orin has it. The listener only re-uploads the
same course if the FCU has no fence or the listener restarted - it doesn't overwrite a fence
you edited in Mission Planner. The bridge takes these requests on 127.0.0.1:14616
(`--control-port`).
While disarmed the listener also re-reads the FCU's fence every 10 s, so a Mission Planner
write is picked up (and sent to RoboCommand) even when Mission Planner is connected to the
FCU some other way (USB, telemetry radio, directly) and the FCU's reply never passes the Orin.

**Task 4 (automatic).** The bridge sends an `IncidentAck` for every
UAV-targeted AssistanceRequest, KeepOutZone and AllClear once the Orin has
it (`--no-incident-ack` turns this off). The listener sends the
`ReadinessReport` for an AssistanceRequest once the drone has held, airborne,
within 3 m of the requested position for 3 s - it never moves the drone;
the pilot or a mission gets it there.

**Task reports (Orin, `robocommand_reports.py`).** SafePassageReport (Task 1)
and ResourceDeliveryRequest (Tasks 2/3), from the command line or
`send_report()` in perception code; also manual `readiness` / `incident-ack`
fallbacks. The listener relays them to the bridge until it confirms; the
bridge checks them against RoboNation's schema; the GUI log says
"... sent" or "... REJECTED (reason)".
```bash
python3 robocommand_reports.py safe-passage --entry LAT,LON --exit LAT,LON --buoy LAT,LON,red --buoy LAT,LON,blue
python3 robocommand_reports.py resource-delivery --task 2 --resource any --circle green
```

It does **not** change anything on the FCU on its own: no fence edits, no
mode changes. Pushing a keep-out zone into the fence mid-flight could trigger
the fence action (e.g. land) over water, so acting on it stays an operator
decision. Listener options: `--vehicle-id` (default `UAV1`),
`--robocommand-state`.

## Safety notes

- Nothing here enables a fence unless you ask. Uploading geometry and
  switching the fence on are separate, deliberate steps (`--enable-fence`
  is opt-in on both publishers). The live listener is the exception: fence
  data it receives (test fence or RoboCommand course) turns on the polygon
  fence (`FENCE_TYPE=4`, `FENCE_ENABLE=1`); a reset restores the preset, and
  stopping the listener restores the original settings.
- `geofence_publisher.py` always verifies by reading the fence back after
  upload — treat a verification failure as "the vehicle does NOT have the
  fence you wanted" and don't proceed to `--enable-fence` territory until
  that's resolved.
- `param_set.py` never writes a value it hasn't gotten the vehicle to
  explicitly confirm via its own echo.
- The `geofence_udp_publisher.py`/`geofence_udp_listener.py` channel is
  **plain, unauthenticated UDP** — anyone on the same network who knows the
  listen port could send fence updates too. Fine for a closed field/test
  network; if this ever runs somewhere less trusted, it needs
  authentication added before relying on it (not currently implemented).
