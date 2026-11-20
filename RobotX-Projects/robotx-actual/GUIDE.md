# GUIDE — MASTER_SCRIPTS, the operator's reference (v0.1.94.3, revised 2026-11-14)

**On a venue day, start with `docs/USER_GUIDE_VENUE.md`** (the day in order) and `docs/TEST_GUIDE_VENUE.md` (today's tests). This file
is the reference behind them: deploy, networks, start / stop, the missions, the safety latches, logs and commands. It describes only what is current.
- The full history (the per-version rows of §3, the architecture checks 4.1–4.22, the pool days, the old command list) is in `docs/old/GUIDE_v0.1.86_full.md`.
- What each version changed is in `CHANGELOG.md` / `DEVLOG.md`.

Addresses are never written here. They live in `network.env` + `network.ts.env` / `network.lan.env` (`NET_PROFILE`, or `--ts` / `--lan`).
In a shell on the Mac:

```bash
cd ~/Documents/BH1/RobotX/MASTER_SCRIPTS && . scripts/_netenv.sh && load_network_env "$PWD"    # $ORIN_SSH $AGX_SSH $OCS_HOST $BOAT_HOST $AGX_HOST
```

**Rules that never bend** (HANDOFF §0):
- The companion never arms or changes the FCU mode by itself. ARM asks first.
- The transmitter's kill and RC6 beat everything.
- Props off and ESC power off for bench work with a real Pixhawk.
- `apply_params.py` (orin_kit) is the only parameter writer.
- One program per serial port.
- Never copy `buoy_link.key`.

---

## 0. Before anything (Mac)

- `tailscale status` (ts) or the switch (lan): both Jetsons reachable. `ssh $ORIN_SSH true && ssh $AGX_SSH true && echo ssh ok`.
- The macOS firewall must accept incoming 1882 (the hub: the NX's bridge connects in) and 9001. Allow `mosquitto` when asked, or add `/opt/homebrew/sbin/mosquitto`.
  A bridge stuck at "Connecting bridge (step 2)" means this.
- The Mac's USB LAN adapter at the venue: a manual address (the one in `network.lan.env`). After it re-enumerates, set it again.
- `scripts/status.sh`: `clean` + the Jetsons' clock offsets.

## 1. Deploy (Mac → both Jetsons, `~/Desktop/dm3/MASTER_SCRIPTS`), only on Dmitrii's go

Before: `make test` green on the Mac for the version. The archive is built per CLAUDE.md step 5 into `_archive/`.

```bash
V=0.1.93
for h in $ORIN_SSH $AGX_SSH; do
  ssh $h 'who; tmux ls; pgrep -fl "observer|cv_publish|cam_stream|detect_buoys|yolo|train|export"; df -h ~ | tail -1'   # someone else working? stop and ask
  scp _archive/MASTER_SCRIPTS_v$V.tgz $h:~/Desktop/dm3/
  ssh $h "cd ~/Desktop/dm3 && tar xzf MASTER_SCRIPTS_v$V.tgz; grep __version__ MASTER_SCRIPTS/optiprimex/__init__.py"
  scp network.env network.ts.env network.lan.env $h:~/Desktop/dm3/MASTER_SCRIPTS/
done
```
- Plain `tar xzf`: the seven read-only student snapshots under `cv/` print errors. They are identical files; ignore them.
- Stop the host's stack first (`stop_usv.sh --wipe` / `stop_agx.sh`): a running program keeps its old code.
- Then on the AGX run `cv/buoys_multi/run.sh --self-test`, and add a line to `docs/JETSON_LOG.md` (who, what, before / after).
- The previous tgz stays beside the tree. It is the way back on a Jetson: the same loop with the older `V`.

**The date line** (the event switch has no NTP; the launcher refuses at > 5 s, and every command is REJECTED stale at > 10 s):
```bash
ssh $ORIN_SSH sudo date -u -s "'$(date -u '+%Y-%m-%d %H:%M:%S')'"      # the same for $AGX_SSH
```

**Once per fresh Jetson:** `bash scripts/setup_orin.sh` (NX) / `bash scripts/setup_agx.sh` + `cd cv && ./sync_models.sh` (AGX). They need the
internet (ts), and our launchers run their own mosquitto (`sudo systemctl disable --now mosquitto`).

### 1a. Clean reinstall

On the Jetson, with its stack stopped: `mv ~/Desktop/dm3/MASTER_SCRIPTS ~/Desktop/dm3/MASTER_SCRIPTS_old_$(date +%Y%m%d)`.
Then:
1. Run the §1 loop.
2. NX: copy back `usv_ui/{points.json,co.json}` from the old tree.
3. Run the setup script.
4. Delete the old tree once its `logs/` are on the Mac (`scripts/pull_jetson_logs.sh`).

### 1b. The way back on the Mac: `scripts/rollback_to.sh`

`scripts/rollback_to.sh --dry-run v0.1.85` shows what would move; without `--dry-run` it moves the tree into `_archive/rolled_<stamp>_…/`, extracts
the version fresh, keeps `.venv`, `logs/`, `network*.env`, `_archive/`, runs `make quick`. Nothing of ours may be running.

## 2. Networks

| profile | where | file |
|---|---|---|
| `lan` | the venue / team switch, no internet (`NET_PROFILE=lan`) | `network.lan.env`: OCS_HOST, BOAT_HOST, AGX_HOST, ORIN_SSH, AGX_SSH, UAV_SSH, RC_ALPHA…DELTA_HOST, VENUE_RC_* |
| `ts` | the Tailscale tailnet: lab, setup scripts | `network.ts.env` |

**Either cable, either adapter (v0.1.94).** Both USB LAN adapters stay on DHCP; a root service puts the Mac's team address
(OCS_HOST, the address the boat and AGX dial) on whichever adapter got a team-switch address, and takes it off the other. The adapter
that got a course address is the course side. Once per Mac:
`sudo scripts/net_roles.sh dhcp` then `sudo scripts/net_roles.sh install` (again after editing `network*.env`).
At the venue: plug the two cables in any way round, wait ~10 s, `scripts/net_roles.sh status` (roles, no addresses), then
`scripts/run_ocs.sh --lan --judge auto` (or **auto** in the launcher's judge list) picks the course you are plugged into (ALPHA-DELTA only; never the venue demo - that is always an explicit `--judge demo`, v0.1.94.2).
Manual fallback, one line in your own Terminal: `sudo ifconfig <team adapter> alias $(grep '^OCS_HOST=' network.lan.env | cut -d= -f2) 255.255.255.0`.

**The RoboCommand NIC** (the course network) is the Mac's second interface:
- It is a DHCP client, with bridging and Internet Sharing off. It gets an address 192.168.6x.15–254 (gateway .2, 65–68 per course).
- `RC_HOST` empty / `auto` means that gateway.
- It is never bridged to the team network.

**The team-LAN address of this Mac must be OCS_HOST** (manual, not DHCP). The boat bridges to it, and the launcher refuses a real launch
without it, printing the fix: `sudo networksetup -setmanual "<the USB LAN service>" <OCS_HOST> 255.255.255.0` (v0.1.90.10).

**Ports** (v0.1.90.12, Dmitrii: "no ambiguity with 1883"): one number per job, never reused on another host.

Ours:

| port | host | job |
|---|---|---|
| **1882** | Mac | MQTT **the hub** (`HUB_PORT`): OCS, launcher, UAV adapter; the NX's bridge connects in |
| **1883** | NX | MQTT **the drone's map listener** on the boat broker: their publisher's default `--broker <NX> --port 1883` (`sgit/uav/cv/#`) |
| **1884** | NX | MQTT **the boat broker** (`BOAT_PORT`): mission manager, usv_ui, the AGX's cv_publish, the boat window |
| 1885 | Mac | rc-test, when used |
| 9001 | Mac | the hub's websockets (`HUB_WS`) |
| 9002 | NX | the boat broker's websockets (`BOAT_WS`) |
| 8090 | AGX | the camera stream |
| 8093 | Mac | the tk launcher's backend |
| 8094 | Mac | the web launcher |
| 8095 | Mac | the fake judge's page |
| 8096 | Mac | the OCS page (the tk console reads it) |
| 8097 | Mac | the simulator page |
| 8098 | Mac or NX | the USV window (one place per launch) |
| 8099 | Mac | the UAV window |
| 5005 udp | NX | the cannon |
| 14550 udp | NX | the FCU link: the fake FCU, or the Pixhawk over Ethernet (`--fcu eth`) |
| 14551 udp | NX | usv_ui's FCU share (the mission manager reads it) |
| 14552 udp | Mac | the bench / simulator's FCU share |
| 14560 udp | NX / Mac | the fake FCU's control (bench, simulator) |
| 14605 udp | Mac | our UAV adapter (the drone's listener answers here) |

Not ours, fixed by others, never used for anything of ours:

| port | where | what |
|---|---|---|
| 1883 | the course RoboCommand (ALPHA–DELTA, 192.168.6x.2) | RoboNation's judge broker (handbook 3.4.2; `judge_port`, `RC_*_PORT`) |
| 1910 | the venue DEMO RoboCommand | `VENUE_RC_PORT` |
| 14600 udp | the UAV Orin | the drone team's geofence listener |
| 14550–14556 udp | the UAV Orin | the drone team's MAVProxy |

## 3. Start and stop

**The launcher** opens with `scripts/launch_tk.sh` (the tk window, backend :8093) or `scripts/launch.sh` (web :8094; `--dry-run` = the plan only).
1. Pick a preset, read the plan (every command per host), then press **LAUNCH**.
2. It refuses while anything is up and when a Jetson clock is off by > 5 s.
3. **STOP ALL** (type `stop`) runs `scripts/stop_all.sh`: NX → AGX → judge → rctest → OCS.

Closing the launcher leaves the stacks running; open it again to RECONNECT.

| preset | judge · net · FCU · boat profile | mission · auto at RunStart · record | use |
|---|---|---|---|
| **3 · m1t1-usv** | venue · lan · uart · course | scout · yes · auto | USV alone, scout as Task 1 |
| **3b · m1t1-gates** | venue · lan · uart · course | m1t1 · yes · auto | the gate mission (needs colours) |
| **1 · m1t2-uav** | venue · lan · uart · course, UAV on | m1t2 · no (the USV holds in LOITER) · auto | the drone maps |
| **2 · m1t2-both** | venue · lan · uart · course, UAV on | m1t2 · yes · auto | both vehicles |
| **0 · bench-uavmap** | fake · lan · fake · lab, UAV on, CV off | — | the UAV map → our map on the bench |
| **sim** | fake · local · the simulator (omni_usv hull, marina) · course | scout · no · off | rehearse on the Mac |
| **lab** | fake · ts · uart · lab | — | a real Pixhawk on the bench, props off |
| **custom** | as chosen (shipped: fake FCU, fake CV, lab) | as chosen | the NX smoke (FCU fake), the W0 tests (**profile pool**: TEST MODE) |

**The judge field:** venue (DEMO, the default) · alpha…delta (the assigned course) · course · fake · rctest. A green ● marks a RoboCommand that answers.
The row "reachable" shows AGX · USV-NX · UAV-NX ●/○. The launcher warns when a real launch reports to rctest / fake while the venue answers.

**What a launch does by itself (v0.1.86):**
- The boat starts **HANDED OVER**. `run_usv.sh --no-handover` / `USV_HANDOVER=0` = as before.
- The preset's mission is preselected, and the preset sets the declaration's tier row.
- The AGX observer records from launch (`--record --pairs-s 10`).
- **DECLARE** starts a logging session; R6 stops it.
- The boat's SQLite logger is on (`LOG=0` = off).

**The OCS console** (tk, `scripts/run_ocs_tk.sh`) opens by itself when the OCS comes up (the launcher's **OCS console** button).
The HTML page at :8096 (the launcher's **web ↗**) is the fallback, on the same wire with the same words.

The console's layout:
- **The header:** the state word, the R / V chips, LINK · FCU · GUARD · REC, the alarms, **START LOGGING**, ⛶ BIG VIEW, ⚙ SETTINGS.
- **The RUN column:** the 6-step ladder (heartbeat → pilot ready → enable auto → declare → RunStart → it starts by itself; ▶ START = the override), and the USV1 / UAV tiles with their buttons (HOLD · RESUME · ABORT · ARM (asks) · DISARM · STOP & RESET (asks) · ▶ UAV RUN).
- **The tabs:**
  - LOG (events · commands · alarms · judge)
  - MAP (zoom / pan / F follow; outside-course buoys grey)
  - VEHICLES (IMU · COMPASS · GPS · RC · LOOP · GUARD LAG · HEARTBEAT per vehicle)
  - JUDGE
  - HOSTS (CPU · GPU · RAM · temp · disk with minutes of recording left · signal)
  - CMD (every command with its options, hover = the tooltip)
  - MISSION (the picker, the stages)
  - CAMERA (the live feed, RECORD + depth, where each part is stored)
- **⚙ SETTINGS:** saved / restored configs, the tuning knobs in groups (SPEEDS · TURNING · APPROACH · OBSTACLES · ORBIT / SCAN · FIELD · DOCK), SYSTEM (restarts per host, typed confirmation), HELP.

The console's own contract is in `docs/TK_CONSOLE.md`.

**By hand, per host** (what the launcher runs; from the tree root):
```bash
# Mac
scripts/run_ocs.sh --lan [--judge venue|alpha|…|fake|rctest|course] [--uav]   # tmux ocs: hub rc ocs log cli web (+ uav)
scripts/run_ocs_tk.sh                                                         # the tk console (one instance)
scripts/run_sim.sh [world] [--judge fake|rctest|venue] [--uav]                # the simulator (HULL=ideal = the old plant)
# NX
scripts/run_usv.sh --lan --fcu uart --course       # --pool = TEST MODE (the W0 tests); --lab indoors with a real Pixhawk; --fcu fake = the smoke
tmux attach -t usv                                 # window ui = usv_ui: handover / takeover / mode / arm / disarm / log on
# AGX
scripts/run_agx.sh --lan                           # tmux agx: cv_publish, cam, cli
scripts/agx_observer.sh start [--detector default|model-a|boundary-v14] [--no-record] [--pairs-s N] | stop | status [--json]
```
**The detector** is picked with the page's OBSERVER select, the tk console's SYSTEM → Buoy observer, or `agx_observer.sh --detector`.

| detector | what it is | on our 11 Nov footage |
|---|---|---|
| `default` | our best.pt | the reference |
| `model-a` | the colleague's Model A | not reviewed |
| `six-class` (v0.1.87) | their 11 Nov model: red / green / blue / buoy | a buoy in 248 of 344 frames |
| `boundary-v14` (v0.1.87) | coloured buoys + obstacle rings on the boat's map (`run.sh --obstacle-classes obstacle,dock\|none`) | a buoy in 4 of 344 frames: **not for runs** |

The six-class model may have been trained on those same frames, so its number is optimistic. The reviews are in
`logs/cv_review/*/index.html`.

**Switching the model:** pick another detector and press START (the page: the OBSERVER button). That restarts the observer on it:
about 10 s without detections, and the map keeps what it has. It is refused while a task runs. Keep `default` for the scored runs
unless a review says otherwise.

**Stop:** `scripts/stop_usv.sh [--wipe]` · `stop_agx.sh` · `stop_ocs.sh [--wipe]` · `stop_all.sh`.
- Each prints `down: …` or `STILL UP:` with the one `pkill` to use.
- `--wipe` clears the broker persistence; use it at every version change.
- Never `pkill -f` with a bare pattern.

## 4. The missions

| code | what | profile |
|---|---|---|
| **scout** | T1 as scout: colour-blind, buoy to buoy inside our course, under way (forward ≥ 0.4 m/s while turning, ≤ 5 °/s, 1 m straight first), circle scan when the map runs dry, RETURN → DONE; budget 900 s, margin 4 m; `logs/scout_<run>.jsonl` | course |
| **m1t1** | T1 Core, the gates (entry circle → gates → exit circle), SafePassageReport | course |
| **m1t2** | T1 Advanced with the UAV's buoy map (WAIT_UAV_MAP, `uav scan done` releases it by hand) | course, `declare uav on` |
| **m3t1** | T3 docking + the water cannon (the AGX sender → the students' Pi) | course |
| **m4t0** | T4 alone: LISTEN for the AssistanceRequest → ACK → TRANSIT → READY | course |
| m0t8 / m0t9 | ground: the detection range / the ZED guard on a stand | any |
| m0t6 / m0t10 / goto / m0t2 / m0t3 | TEST_ORBIT (under way), `test_yaw --pair side\|aft\|both`, goto, path, sway: hidden from the pickers, typed still works | **pool** (TEST MODE) |

**Only our course** (v0.1.85): a detection outside the RxCourse boundary + 1.5 m is never a target. It is kept for avoidance (grey).
On a real judge, scout / m1t1 refuse to start without the RxCourse.

**The ZED guard:**
1. SLOW cap at 3 m.
2. STOP at 1.5 m → brake → 2 m astern → a keep-clear ring on the map → re-plan.
3. The 3rd stop in one place → AUTO_SAFE_HOLD.

Obstacles seen 3–8 m ahead become 2 m rings (dashed); `zed_guard.map false` = STOPs only.

**The run ladder:** R0 DISCONNECTED → R1 CONNECTED → R2 COURSE_VALID → R3 DECLARED → R4 READY → R5 RUNNING → R6 ENDED.
**The vehicle states:** V0 BOOT · V1 KILLED · V2 MANUAL · V3 AUTO_HOLD · V4 AUTO_TASK · V5 AUTO_SAFE_HOLD · V6 FAULT.
Station keep: `ocs_cli station_keep hold|loiter` (LOITER on the course; the safe hold is always HOLD).

## 5. After a run / at the end of the day

1. `scripts/pull_jetson_logs.sh [--lan] [--nx|--agx] [--dry-run]` copies everything to the Mac. It is read-only on the Jetsons and resumable, never deletes, and checks files and bytes per source.
2. `python3 scripts/svo2_mp4.py <folder|cam.svo2> [--eye left|right|both]` makes an .mp4 beside each cam.svo2 (no ZED SDK needed).
3. Free the AGX only after the pull (HOSTS: amber < 15 GB, red < 8 GB). The observer stops recording by itself below 5 GB.
4. `make day` (~2 min 15 s) when the code changed between runs; `make test` at night / before a deploy.

## 6. The real FCU

- On the bench: the `lab` preset / `run_usv.sh --lab`, props off, ESC power off, the transmitter on.
- Free run (`--lab --free-run --world …`) runs the whole stage sequence on a ghost plant, with mode requests only to the real FCU. The FREE RUN banner is on every page; never use it near the water.
- FCU parameters: `apply_params.py --dry-run` (read-back) first; `apply_params.py --file docs/fcu/<preset>.param` only on Dmitrii's go.
  The prepared presets:
  - `steering_rate.param` (W0.3), not applied;
  - `loiter_rc6.param` (`docs/fcu/PRESET_LOITER_RC6.md`), kept as is;
  - the lamp script `LUA/robotx_lamp_v5.lua` (MAVFtp → APM/scripts/).

### 6.1 The safety latches

| latch | what it does | how to clear |
|---|---|---|
| kill (SF / RC5) | → V1 KILLED | release the switch, then `confirm_safe` |
| RC6 / the FCU in MANUAL | → V2 MANUAL, **latched, last wins** (`rc6_policy hold` = no autonomy while high) | flip again, or a later `enable_auto` / mode command |
| ArduPilot RC failsafe | transmitter lost → failsafe action (HOLD) | transmitter on and bound |
| usv_ui ACTIVE | `enable_auto` refused "type handover there" | `handover` in usv_ui (`takeover` / `stop` / `x` / `disarm` take it back) |
| FCU_LOST · EKF_BAD (soft 3 s) · UNEXPECTED_DISARM | → V6 FAULT | the cause gone + `confirm_safe` |
| PERCEPTION_LOST · LOOP_SLOW · the guard's 3rd stop | → V5 AUTO_SAFE_HOLD | `resume` once cleared |
| `require_armed_for_auto` · `require_prepare_for_auto` · `require_run_for_task` | `enable_auto` needs armed + a declaration; tasks need RunStart (R5) | arm (transmitter / ARM), `declare`, the judge |
| OCS intent guards | `declare` in R2 / R4 / R6, `end_run` R3–R5; ARM / DISARM / SET_MODE never from software by itself | follow the ladder |
| stale commands | older than 10 s → REJECTED | the clocks (§1) |
| one program per serial port | `run_usv.sh` refuses while `.share` names a live holder | stop the holder |

### 6.2 The mode sequence: `mode-log`

Every run writes the mission manager's command trace `logs/usv/run_<stamp>/mm_trace.jsonl`: mode requests + the FCU's answers, mode-20
frames, GUIDED targets, the stage changes. On the NX:

    python3 -m optiprimex.tools.mode_log logs/usv/latest/mm_trace.jsonl [--pool|--lab] [--tlog usv_ui/logs/X.tlog] [--md]

It prints per stage the modes requested / reported, the frame cadence and a verdict (exit 0 = no FAIL).
- Since 0.1.86.4 a mode ack is waited for 0.2 s (2 s backoff). `mode_req ok false` lines mean the FCU answered later.
- `timeouts.mode_ack_s: 0.5` = the old wait.

## 7. The venue

The day's order is `docs/USER_GUIDE_VENUE.md`; the event checklist before the course network is its §3. Compliance facts we keep:
- One RoboCommand client per team: the drone team's own bridge stays off (the FOREIGN DECLARATION alarm says it is not).
- The report counters never reset (`logs/ocs/rc_seq.json`).
- A heartbeat is never fabricated: it stops when the status is stale (USV 10 s, UAV 5 s).
- The declaration is checked before it is sent: the JUDGE tab shows the bytes.

**The UAV** (identity: vehicle id UAV1, registration SGIT-53577, pilot Calwin-U2332301):
- Our adapter speaks their JSON / UDP to their listener (`--vehicle-id UAV1`, up before our OCS).
- It declares the UAV only with `declare uav on`, using the drone's own FCU fence.
- It forwards the course and RunStart; the first RunStart forward flies the drone.
- Their buoy map comes in over their broker (`UAV_BUOY_PATH direct`) or UDP (relay).

## 8. Logs — what records what

| what | where (host) | on the Mac after the pull |
|---|---|---|
| each program's output, the command trace, the recorded map (`maps/`: 1 Hz timeline, latest, a snapshot per task end) | NX `logs/usv/run_<stamp>/` | `logs/jetsons/<date>/nx/logs/` |
| the boat's MQTT in SQLite (on by default, `LOG=0` = off) | NX `logs/usv/run_<stamp>/boat_<stamp>.sqlite` | the same |
| usv_ui's FCU log (tlog + 5 Hz csv + events) | NX `usv_ui/logs/` | `nx/usv_ui_logs/` |
| scout's log | NX `logs/scout_<run>.jsonl` | `nx/logs/` |
| the ZED (cam.svo2 H.265 8000 kbit/s + IMU, `frames.jsonl`, `session.json`, `pairs/` RGB jpg + depth png mm) | AGX `~/Desktop/dm3/recordings/<session>/` | `agx/recordings/` |
| a logging session (label, t0, the clocks, what ran) | Mac `logs/ocs/sessions/<label>.json` | — |
| the hub SQLite, ocs / rc logs, RoboCommand traffic | Mac `logs/ocs/run_<stamp>/` | — |
| the Mac's frames (CAMERA tab, 640 px) | Mac `logs/recordings/` | — |
| the OCS's map | Mac `logs/ocs/maps/` | — |
| rc-test's own log (the PoR artifact) | Mac `logs/rctest/run_<stamp>/` | — |

## 9. Commands in one line each

| command | what |
|---|---|
| `scripts/launch_tk.sh` / `launch.sh [--dry-run]` | the launcher (tk / web) |
| `scripts/status.sh [--watch]` | what of ours runs on this host + the Jetsons' clocks (exit 0 = clean) |
| `scripts/pull_jetson_logs.sh [--lan] [--nx\|--agx] [--dry-run] [--date D]` | both Jetsons' logs → `logs/jetsons/<date>/` |
| `python3 scripts/svo2_mp4.py <dir\|svo2> [--eye …]` | ZED recording → mp4 |
| `scripts/agx_observer.sh start\|stop\|status` | our observer on the AGX (records unless `--no-record`) |
| `ocs_cli help` | every OCS word (also the console's CMD tab with tooltips) |
| `logs/ocs/cmd_timing.jsonl` (v0.1.86.9) | one line per press from the pages: queue · verdict · accepted · final · reply ms; `OCS_CMD_UNTIL=done scripts/run_ocs.sh` = the reply waits for DONE again |
| `ocs_cli mission CODE [--auto]` · `start_task [CODE]` · `hold` · `resume` · `abort` · `end_run` · `reset` · `stop_reset` | the run |
| `ocs_cli declare` · `enable_auto` / `disable_auto` · `station_keep hold\|loiter` · `report on\|off [uav]` | the ladder |
| `ocs_cli declare uav on\|off` · `uav ready on\|off` · `uav scan done` · `uav resend_course` | the UAV |
| `ocs_cli fcu_mode M` · `arm` · `disarm` | the operator's FCU words (ARM asks on the console) |
| `ocs_cli tune KEY VALUE` · `tune save` | runtime knobs (MANUAL / AUTO_HOLD only; the console's ⚙ SETTINGS) |
| `ocs_cli speed cruise\|circle\|gate\|all K` · `speed reset` | v0.1.94.3: the LIVE speed knobs x0.5-2.0 - any state, a running task included, at once. cruise = path legs (run-ins, transit, scout, exit), circle = orbits (speed + yaw rate: same radius), gate = T1 THROUGH; scans / turns on the spot are not scaled. The cap lifts with k (max_speed_mps x k), never above 1.5 m/s; the ZED guard's SLOW / STOP still win. Console: USV1 tile, SPEED row (− / + in 0.25, or type + Enter; x1 all) |
| usv_ui (NX): `handover` · `takeover` · `mode M` · `arm` · `disarm` · `log on NAME` · `cv on\|off` · `help` | the boat console |
| `python3 -m optiprimex.tools.mode_log TRACE` | §6.2 |
| `make day` · `make quick` · `make live` · `make test` | test tiers: between runs · every change · wire changes · nightly / deploy |
| `scripts/rollback_to.sh [--dry-run] VERSION` | the Mac's way back (§1b) |

## 10. Where the rest is

- `docs/USER_GUIDE_VENUE_v0.1.93.md`, `docs/TEST_GUIDE_VENUE_v0.1.93.md`, `docs/MORNING_2026-11-13.md`: the final day (13 Nov); the final pack `extra/FINAL_PACK_2026-11-13/` (PDFs). Older: `docs/USER_GUIDE_VENUE.md`, `docs/TEST_GUIDE_VENUE.md`.
- `docs/TK_CONSOLE.md`: the tk console.
- `docs/LAUNCHERS.md`: launchers and profiles.
- `HANDOFF.md`: the rules and the context.
- `ROADMAP.md`: the work packages (§0 = the competition week).
- `DEVLOG.md`: what each version did, ending with the go / no-go table.
- `docs/JETSON_LOG.md`: every Jetson change.
- `docs/fcu/`: the presets, the read-back.
- `docs/reports/SCOUT_LAYOUTS_2026-11-12.docx`: scout in 8 layouts.
- `docs/old/`: the full GUIDE of v0.1.86 (the history, the architecture checks 4.1–4.22, the pool days) and the venue guides of 9 Nov.
