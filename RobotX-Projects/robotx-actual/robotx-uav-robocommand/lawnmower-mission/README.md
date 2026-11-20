# Lawnmower Mission

Flies a lawnmower (back-and-forth) pattern inside the geofence that is on the
flight controller, then returns home and lands. Started from the drone GUI
(`mission`) or run on its own. Runs on the **Orin**.

Keep this folder next to `drone-heartbeat-gui/` and `geofence-tools/` - it
uses the GUI's heartbeat code (`heartbeat_link.py`) and the geofence tools'
parameter code (`mavlink_param_link.py`).

## Files

- `lawnmower_mission.py` - the script you run: checks, plans, uploads, flies, watches.
- `mission2.py` - the same mission at **10 m AGL, 5 m/s** (GUI: `mission2`).
- `mission3.py` - the same mission at **15 m AGL, 7 m/s** (GUI: `mission3`).
- `runstart_mission.py` - flown by the drone GUI when RoboCommand sends RunStart: take off to **5 m**, hover **60 s**,
  then mission2's legs at **10 m AGL, 3 m/s**.
  Both run `lawnmower_mission.py`'s own code, so every check, failsafe and
  RC takeover is identical; only the altitude/speed defaults differ. Any
  option below works with them too (e.g. `python3 mission3.py --upload-only`).
  The fence's `FENCE_ALT_MAX` must be at least 2 m above their altitude
  (12 m / 17 m) or they refuse to fly - the course fence's 20 m is fine.
- `lawnmower_planner.py` - the path geometry only (no MAVLink).
- `mission_link.py` - the MAVLink side, built on `HeartbeatMonitor` from
  `heartbeat_link.py`: one connection, the same heartbeat tracking as the GUI.

Install: nothing new - `pymavlink` (already installed for the GUI).

## What the flight does

1. Reads the fence **from the FCU** (whatever `geofence_udp_listener.py`
   uploaded), checks `FENCE_ENABLE=1`, the polygon bit in `FENCE_TYPE`, and
   that 5 m is safely under `FENCE_ALT_MAX`.
2. Plans legs inside the fence (any shape, see "Any fence"), `--margin` (3 m) in from every edge,
   `--spacing` (4 m max) apart, starting from the leg nearest home.
3. Uploads it as a normal ArduPilot mission and reads it back to verify:
   take off 5 m -> hover 30 s above home -> speed 3 m/s -> legs -> home -> land.
4. Checks: drone disarmed, FCU in GUIDED, **RC mode switch physically in its
   GUIDED position** (reads `FLTMODE_CH` and `FLTMODE1-6`), another switch
   position available for takeover, RC on, 3D GPS fix.
5. Arms and takes off in GUIDED, then switches to AUTO. ArduPilot flies the
   rest; the script only reports progress to the GUI log.
6. When the mission is COMPLETE (landed at home, disarmed) it switches AUTO ->
   GUIDED and clears the waypoints from the FCU (the fence stays). Not done
   after an RC/Mission Planner takeover - the mode is yours then.

## Who's in charge

ArduPilot, the RC and Mission Planner - always:

- Flip the RC mode switch (e.g. to LOITER) or change mode in Mission Planner
  at any moment: the script sees the mode change and stops. It never changes
  the mode back, never sends RC overrides, never forces arming.
- If the script, the Orin or the GUI dies after AUTO, the FCU finishes the
  mission by itself. The fence and failsafes are ArduPilot's own.
- `mission stop` in the GUI only stops the script - it does not change the
  flight mode. To stop the drone, use the RC switch.
- The RC switch is still in its GUIDED position after the script moves the
  FCU to AUTO, so to take over flip it to a *different* position.
- The script watches the RC mode-switch channel itself (`FLTMODE_CH`, ~10 Hz)
  as well as the flight mode (heartbeat, ~1 Hz). The moment the switch leaves
  its GUIDED position - before arming, during the climb, just before AUTO or
  during the legs - the script stops ("RC mode switch moved to position N ...
  you have control"), and it checks this right before every command it sends,
  so it never arms, takes off or switches to AUTO over an RC takeover.

## Running

From the drone GUI (recommended - progress shows in the message log):

| Command | Effect |
|---|---|
| `mission preview` | plan and upload only, no flying - then Mission Planner: Plan tab > **Read** to see the path |
| `mission` | the full flight; press Enter again (blank) to confirm |
| `mission stop` | stop the script (not the drone) |

Standalone, from any folder:

```bash
python3 "/home/uav/Desktop/UAV software/lawnmower-mission/lawnmower_mission.py" --upload-only
python3 "/home/uav/Desktop/UAV software/lawnmower-mission/lawnmower_mission.py"
```

Output also goes to `lawnmower_mission.log` in this folder when started from the GUI.

### Options

| Option | Default | |
|---|---|---|
| `--connection` | `udp:127.0.0.1:14552` | MAVProxy output (the GUI's auto-started MAVProxy has it) |
| `--altitude` | `5` | metres above home |
| `--speed` | `3` | m/s along the legs |
| `--hover` | `30` | seconds to hover after take-off |
| `--hover-altitude` | `--altitude` | take off and hover at this height, then fly the legs at `--altitude` |
| `--spacing` | `4` | max metres between legs |
| `--margin` | `3` | metres to stay inside the fence |
| `--sweep-bearing` | longest fence edge | compass bearing of the legs, e.g. `53` |
| `--upload-only` | | plan + upload, no flying |
| `--yes` | | don't ask before arming (the GUI passes this after you confirm) |

## Any fence

The path is planned from whatever fence is on the FCU at the time, at any
location, so a new fence just needs publishing - nothing to change here:

- inclusion polygons with any number of corners, convex or not (L, U, ...)
- keep-out (exclusion) polygons and circles inside the area
- inclusion circles, and ArduPilot's own circle fence (`FENCE_RADIUS`
  around home, when `FENCE_TYPE` has the 2 bit)

The drone stays `--margin` clear of every zone. Hops between legs (and to and
from home) go around corners and keep-out zones when a straight line wouldn't
be safe, and every segment is re-checked against the real fence every 10 cm.

It refuses, with the reason in the GUI log: a polygon whose edges cross, home
outside the fence or inside a keep-out zone, and a fence too small for the
margin. Legs that can only be reached through a gap narrower than 2 x margin
are skipped with a warning.

## Seeing the path in Mission Planner

After `mission preview` (or `mission`), in Mission Planner: **Plan** tab >
**Read**. The legs, hover point and landing show on the map; during the
flight the Flight Data map shows the drone following it.

## Not tested on the real drone

Tested against a simulated FCU only (protocol, checks, takeover, landing).
First real flight: props-on in an open area, `mission preview` first and
check the path in Mission Planner, a pilot on the RC ready to flip to LOITER.
