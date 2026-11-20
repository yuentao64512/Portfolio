# ROADMAP — MASTER_SCRIPTS (global_v3), from v0.1.19 to the pool trials and RobotX 2026

Written 2026-11-02 by Dmitrii with Claude (Fable) after the v0.1.18 UX rehaul and the v0.1.19 reshuffle. **This is the
hand-off between sessions**: any session (another model, another day) reads this first, then the last block of `DEVLOG.md`,
picks the first work package that is not done, and leaves a DEVLOG block behind. `CLAUDE.md` has the session contract (one
request = one version, `make test`, DEVLOG before the archive); `HANDOFF.md` the rules, hosts and ladders; `GUIDE.md` what the
operator types. Nothing here overrides the hard rules in HANDOFF §0 (the companion never arms; props off on the bench; the
students' folders are never modified; `buoy_link.key` is never copied).

## 0. Status — competition week (updated 2026-11-12 00:30 SGT, Dmitrii with session A; read this first)

**Today (Thu 12 Nov):** test run **11:15**, scored runs hopefully after lunch, CV integration in the morning (Dmitrii's brief). Overnight
builds stop at **07:00**; then the morning window: AGX disk cleanup → deploy → bench / fake-FCU smoke on the NX → FCU read-back + steering
preset → water tests W0 → the 11:15 test run.

| host | version | note |
|---|---|---|
| Mac (OCS) | **v0.1.84.8** | tk console + tk launcher, START LOGGING (one press, clocks first), RECORD + depth pairs, ZED playback, live feed |
| NX (boat) | **0.1.83 or 0.1.81.1 — check** | JETSON_LOG's last line: 0.1.83 (11 Nov 02:39 UTC); the water-day note says the NX ran 0.1.81.1 later, no rollback line |
| AGX | 0.1.83 | **disk full** (4.7 GB free; recording stopped 17:36 on disk_low); cleaned in the morning (Dmitrii) |

**The water day (11 Nov, `docs/WATER_DAY_2026-11-11_NOTE.md`):** LOITER as the autonomous station keep works on this hull, RC6 override works,
the declaration / RunStart against the venue works, logging by default works. **Auto never moved the boat**: both planners turned in place,
the FCU's differential ramped 25 s to ±200 µs, the hull turns ~1.8 °/s in place. → WP41. The logs: `logs/keep_2026-11-11/` (run1 / run2 /
run3 + report; the non-water clips deleted on Dmitrii's word).

**Tonight's order (session B; prompts: `docs/PROMPT_COMPUTE_NIGHT_2026-11-11.md` P0 + addendum, `docs/PROMPT_PRESETS_NIGHT_2026-11-12.md` incl. §6):**

| WP | version | what | status |
|---|---|---|---|
| WP41 | v0.1.85 | turn under way: P0 T0.2–T0.5 + the five sketch rules T0.2a–e (target ring, bow-clear, boundary rule + margin 4, LOS 3 m, closest-approach arrival), `scan_v` 0.3, the `omni_usv` plant; acceptance + "no boundary exit in 20 scout runs" (`docs/SCOUT_BRIEF_2026-11-11.md` §8b: 95 % buoys, 0 exits). **move first: ≥ 1 m forward at ≥ 0.4 m/s within 1 s of the start, before any turn** (Dmitrii 00:40); T0.1 read-back + T0.6 preset: **morning** (Jetsons offline) | ✅ built v0.1.85 (Mac, 01:02; 20 scout runs 0 exits, 100 % buoys; make test 975) — T0.1 / T0.6 / W0 morning |
| WP46 | in v0.1.85 | **our course only**: a detection outside the RxCourse boundary (+ ~1.5 m) is never a task target (scout, m1t1 gates, m1t2 / the UAV map); kept in the map as "outside course" for avoidance; on a real judge no RxCourse = refuse to start (no square fallback) — 4 courses side by side | ✅ built v0.1.85 (`outside_course`, `course_boundary_local` live in /map.json) |
| WP42 | v0.1.86 | presets (0 bench-uavmap · 1 m1t2-uav · 2 m1t2-both · 3 m1t1-usv scout · 3b m1t1-gates) · judge order venue-first · auto start on RunStart · handover at stack start · logging + recording at DECLARE · test tiers (`make day`) · hosts feed `/hosts.json` · the cull **as hide-flag only** (deletion → WP42b) · the launcher's Jetson reachability (AGX · USV-NX · UAV-NX ●/○, as the course) | ✅ built v0.1.86 (Mac; make test 997) — cull = hide-flag, **WP42b** (deletion + pool→lab move) later; the NX fake-FCU smoke gates its deploy |
| WP47 | 0.1.86.x | smaller recordings, measured GB / 10 min before → after: svo2 H.265 / bitrate, depth pairs at 960 px, Mac frames 640 px (the ZED capture resolution unchanged: detection range) | ✅ built 0.1.86.3 (svo2 H.265 @ 8000 kbit/s, pairs 960, Mac frames 640) — AGX measurement in the morning |
| WP43 | 0.1.86.x | the compute tiers, Mac-side only tonight (cv_publish 20 Hz, the observer's writer thread, the /recording refresher, bounded mode acks, the bridge params); AGX MAXN + TensorRT: morning | partly: 0.1.86.4–.8 (tick_ms, mode acks, /recording cache, cv_publish 20 Hz, writer thread, bridge keepalive, pipe_ms); AGX MAXN / TensorRT / Tier 2–3 not done |
| WP44 | A | the tk console: HOSTS tab (AGX · NX · MAC: CPU · GPU · RAM · temp · power + **disk free with minutes of recording left**), the WP42 / WP46 follow-ups (auto-session REC, hidden missions, "starts by itself at RunStart", outside-course buoys greyed), `scripts/pull_jetson_logs.sh` | tonight (A) |
| — | morning | deploy rule: **v0.1.85 is the floor**; v0.1.86 only when the NX fake-FCU smoke is green; rollback tgz beside the tree | morning |

**Decided, not code:** ask RoboNation for both light types up (m1t2); the scout budget **900 s** (Dmitrii 00:35: the sketch ran out of time at 600 s in 7/20 runs).

## 1. Where we are (v0.1.19; v0.1.20 added the simulated pool — §1a, WP1)

What runs today, proven on the laptop (`make test`, 124 tests) and rehearsed live on the three hosts over the tailnet:

- **Comms stack**: two Mosquitto brokers (boat 1884 on the NX, hub 1883 on the Mac) joined by the real bridge; presence beats;
  the boat sees the OCS link (`ocs/state` bridged in, `$SYS` bridge state) and says when it is down; `link_loss_action`.
- **Mission manager** on the NX (vehicle FSM V0–V6, run context, command handler, task runner) through usv_ui's FCU share;
  tasks built: **T4** incident (full chain against the fake judge), **GOTO** (bench), **TEST_MANUAL / TEST_AUTO**.
- **OCS app** on the Mac (run lifecycle R0–R6, command authority, declare gate with PREPARE acks, mission preselect, judge view,
  reset on both ends) + **fake RoboCommand** (JSON on `rc/*`) + **op-tui** (OCS console) + `ocs_cli`.
- **usv_ui** on the NX (the boat console: holds the FCU link — fake plant or Pixhawk — shares it, handover / takeover, bursts,
  swaycal, path / pathto / swayto by hand, cv map, MISSION tab, WARNINGS / EVENTS / CONSOLE).
- **Perception link**: `cv/` on the AGX (students' buoy observer + our dock observer + cannon link + `cv_publish`) → MQTT
  Observations on the NX broker → `PerceptionMap` (entities, gates, dock line) → `telemetry/usv/map` → map.html / op-tui BOAT tab.
- **Simulation pieces that exist**: `sim/fake_fcu` (kinematic plant on MAVLink), `sim/fake_perception` (detections of a YAML world
  from the live pose in `local/<v>/mm/state`, publishes `sim/truth`), `sim/fake_robocommand`, `profiles/bench/world_bench.yaml`
  (a channel with three gates and a dock line), `scripts/run_bench.sh` (the whole laptop stack in tmux).
- **Mission codes** (v0.1.19): the operator says `m4t1`, `m0t1sim` …; the wire keeps `task: T4` + `args {tier, sim}`.

What is **not** there yet: T1 and T3 as task behaviours (Orbit, FollowPath, ScanYaw, SwayTransit, DockLos), a pool world for the
simulator (gates in the pool geometry, a dock with three bays and a RED→GREEN indicator), the simulator running on the AGX fed by
the boat's pose, the real-protobuf RoboCommand client (and an `rc-tui` to drive the fake one by hand), a camera stream to the
Mac, the UAV stack (the students will hand it over; details to follow), the course-network profile (real judge, ACLs,
`bench_mode: false`).

## 1a. Revision 2026-11-02 (second session: Dmitrii with Claude Opus 5.5, after reading the handbook and pathing_sim)

Decisions (polled): **simulator = Python sim + browser view** (fake_perception / fake_dock in Python, on the AGX or the laptop,
covered by `make test`; map.html / a browser page shows the world over the brokers' websockets); **Task 1 planner = both,
switchable** (the simple gate-midpoint chain and a Python port of pathing_sim S6/P4, chosen by a profile key — WP2); **pool =
50 × 25 m**; Jetsons deployed to v0.1.19 by Claude (trail in `docs/JETSON_LOG.md`); TUI tests may render larger than 150 × 50
(new ones at 200 × 60; the 150 × 50 ones stay as the small-screen regression). Inputs now in the tree: `handbook/` (RobotX 2026
Team Handbook, md + pdf) and `pathing_sim/` (the S6 browser sketch, its replication spec, the report) — read-only references.

What the handbook changed (sections cited): Task 1 has **ten** buoys whose lights **flash 1 s on / 1 s off** (RED, GREEN, BLUE
flashing = ENTRY, BLUE steady = EXIT, unlit = obstacle), "passed" = the closest buoy on that side when abeam (3.3.2) → the
sim models flashing and the PerceptionMap classifies `light: flash | steady | off` (v0.1.20). Task 3: bays numbered 1..3 left to
right facing the dock, two 25 cm target windows per bay, the window lights RED only **after the DockingReport**, GREEN 5 s then
off (3.3.4) → fake_dock (v0.1.20), the report path in WP3. Proof of Readiness: two red/green pairs 6–10 ft wide, start 3 m
behind (3.1.2) → `world_por.yaml`; the PoR video is a deliverable, so m0t2 on world_por is the first real target for WP2.
RoboCommand (3.4): port 1883, ≤ 5 msg/s per system, heartbeats 2 Hz per vehicle, `current_task` transitions mark attempts,
`RunStart.declaration_seq` must match; schemas from `robonation/robocommand` @ 8fc09f2 (`RobotX_2026/`, public, reachable with
`gh`) → WP4.

## 2. Target state (what "done" looks like before the pool trials)

1. **Any mission can be run three ways with one word changed**: `m1t1sim` on the laptop (fake FCU, fake CV from a world file),
   `m1t1sim` on the real hosts (fake FCU in usv_ui on the NX, **sim CV on the AGX** fed by the boat pose, the real brokers and
   bridge, map.html on the Mac), `m1t1` at the pool (real Pixhawk, real CV). Same mission manager code, same OCS, same consoles.
2. **Missions built and proven in simulation first**: m1t1 (Task 1 core), m3t1 (Task 3 core), the two test legs m0t2 (path) and
   m0t3 (sway); then at the pool: waypoints / path / sway through the final architecture, docking (`dockto`), sway along a
   CV-derived line, path generated from CV (`pathgen`).
3. **RoboCommand**: our OCS speaks the real protobuf wire to RoboNation's `rc-test` on the Mac; the fake judge speaks the same
   wire; an **rc-tui** shows what the judge receives (left) and what it sends (right) and lets us pick what to broadcast (course
   boundary, run start, incident messages …).
4. **Camera**: an independent MJPEG stream from the AGX to the Mac, also usable during a run.
5. **UAV**: the drone stack re-integrated (m1t2 = Task 1 advanced), after the students share it.

## 3. Mission codes (decided 2026-11-02)

| code | wire task · tier | what | status |
|---|---|---|---|
| `m0t0` | TEST_MANUAL | state machine by hand (`step`) | built |
| `m0t1` | TEST_AUTO | state machine on timers | built |
| `m0t2` | TEST_PATH | path test: points / `pathgen` → FollowPath | built (WP2) |
| `m0t3` | TEST_SWAY | sway test: dock line / two points → SwayTransit | built (WP3) |
| `m0t4` | BOX_INSPECT | the students' box survey of the dock faces (3 points per face) | built (WP8) |
| `m0t5` | TEST_CV_GATE | two unlit buoys as a pretend-coloured gate (left = GREEN) | planned (WP11) |
| `m1t1` | T1 · core | Safe Passage, boat camera only | built (WP2) |
| `m1t2` | T1 · advanced | Safe Passage + UAV buoy map | later (WP6) |
| `m3t1` | T3 · core | dock + fire | built (WP3) |
| `m4t1` | T4 · core | Dynamic Incident Response | built |
| `goto` | GOTO | bench waypoint | built (bench only) |

`<code>sim` = the same mission + a guard: the vehicle REJECTS it unless usv_ui's presence beat says `fcu_link: fake`. The code
never travels on the wire (`playbook.resolve()` on the OCS side, `ocs/ocs_app.mission_code()` for the state). Rationale: the
m-number is the RobotX task number so the handbook, the judge's `task` field and our words agree; the tier digit is the
handbook tier; m0 is ours. T2 (the UUV) is not attempted.

## 4. Work packages — one per version, in this order

Each WP = one version, one DEVLOG block, one archive. "Accept" is the list a session must tick before calling it done; "Files"
is where the work lands (create what is missing, keep names). Do not start the next WP in the same version.

### WP0 — v0.1.19 · reshuffle + codes + hand-off  ✅ done 2026-11-02
Tree flattened (`optiprimex/ tests/ profiles/ scripts/ logs/ docs/`), mission codes, `ROADMAP.md` / `DEVLOG.md` / `CHANGELOG.md` /
`README.md` map, CLAUDE.md session contract, sim guard in the mission manager, `TEST_PATH` / `TEST_SWAY` in the schema.

### WP1 — v0.1.20 · the simulated pool: world + sim CV on the AGX + sim dock  ✅ done 2026-11-02
Done as below, with these differences (DEVLOG v0.1.20): the pool is 50 × 25 m (decided); `world_por.yaml` (the PoR course) added;
"sees the 3 gates and the dock from the start pose" is impossible with a 25 m ZED range in a 50 m pool, so the test drives a
scripted pose sequence; fake_dock lights the window on "docked" = 3 s slow inside the GREEN bay (or `sim/dock/cmd docked`),
not on a DockingReport yet (WP3 sends it); usv_ui's `dockto` is refused unless the FCU is the fake plant.
Why first: every mission after this is proven here before any hull gets wet; it also gives the AGX a job in the architecture test.
- **World**: `profiles/bench/world_pool.yaml` (new; `world_bench.yaml` stays for the unit tests) in the **pool frame**: the
  Task 1 channel (flashing-blue entry buoy, 3 red/green gate pairs, solid-blue exit buoy) and the Task 3 dock (a dock face with
  three bays, one GREEN indicator, one RED window), sized to the real pool (ask Dmitrii for the pool length / lane; default 25 m ×
  12 m, origin = `frames.origin` of the profile). Objects carry `cls`, `colour`, `lit`, `bay`.
- **Sim CV on the AGX**: `scripts/run_agx.sh --sim [--world FILE]` runs `optiprimex.sim.fake_perception` on the AGX against the
  NX broker (`BOAT_HOST:BOAT_PORT` from network.env, pose from `local/usv/mm/state`, which is on the boat broker) instead of
  `cv_publish`; it publishes the same `local/usv/perception/{buoys,dock,health}` Observations the real CV does, plus `sim/truth` on
  the hub for map.html's truth layer. The AGX needs only paho (our `.venv`). `scripts/stop_agx.sh` stops it too.
- **Sim dock controller**: `optiprimex/sim/fake_dock.py` (new): keeps the dock's indicator state (which bay is GREEN, the window
  RED), listens to `local/usv/cannon/aim` or the cannon link's status file (`cv/cannon` writes `runs/latest/status.json`) and turns
  the window GREEN after N s of "on target"; publishes the state into the world so fake_perception reports `lit` / colour
  changes. Minimal: a time-based toggle driven by one MQTT topic `sim/dock/cmd` and the cannon status.
- **Consoles**: WHERE WE ARE says `CV sim (AGX, world_pool)` from `perception/health` (`source: sim`); map.html shows truth.
- **usv_ui by-hand helpers for the trials**: `pathgen` (a path from the perception map's gate midpoints: entry → gate 1..n → exit,
  shown on the map, saved as points; no motion) and `dockto` (sway to abeam of the chosen bay then bow-in with the guarded
  approach — mode 20, same guards as `swayto`; **bench and sim only until WP3 is proven**).
- Accept: `make test` green + `tests/test_sim_pool.py` (fake_perception on world_pool sees the 3 gates and the dock from the start
  pose; fake_dock flips RED→GREEN on the cannon status); live: `run_agx.sh --sim` on the AGX, `run_usv.sh --fcu fake` on the NX,
  `run_ocs.sh` on the Mac → map.html shows the pool world, gates form, `cv status` in usv_ui says `sim`; `pathgen` draws the
  channel path; GUIDE §4 gets a 4.14 "the simulated pool" row; DEVLOG block.
- Files: `optiprimex/sim/fake_perception.py` (add `--source sim` tag, `--world`, health `source`), `optiprimex/sim/fake_dock.py`,
  `profiles/bench/world_pool.yaml`, `scripts/run_agx.sh` / `stop_agx.sh`, `usv_ui/usv_ui.py` (`pathgen`, `dockto`),
  `optiprimex/tools/map.html` (bays / indicator glyphs), `optiprimex/tools/bench_tui.py` + `usv_ui` (WHERE WE ARE: CV source).

### WP2 — v0.1.21 · m1t1 (Task 1 core) + m0t2 as task behaviours  ✅ done 2026-11-02
Done (DEVLOG v0.1.21). Decided 2026-11-02 (polled): circles 3 m at 0.5 m/s; a missing gate = hold, then scan, then
AUTO_SAFE_HOLD; TRANSIT by GUIDED carrots; planner `simple` by default, `t1.planner` + START_TASK `args.planner`. Differences:
the S6/P4 port is the planner's leg (chain cost, pre / cross / post, Dubins with rmin 1.83 / d 3 from the spec's best tuning,
avoid) on our PerceptionMap — the S6 tracker, the search FSM and the benchmark harness are not ported (ROADMAP §4a), so of
the spec's §11 numbers only the Dubins check is reproduced; `optiprimex/sim/fast_sil.py` (new) runs a task in simulated time
for the logic tests; m0t2 from the map re-plans gate by gate (PoR). A wrong-side gate is refused only when no legal route
exists (a middle gate swapped is crossed the other way round, RED still to starboard).
- **Behaviours** in `optiprimex/usv/behaviours/`: `orbit.py` (one circle round a point at radius r, CW/CCW, from the GUIDED carrot
  or mode-20 frames), `follow_path.py` (waypoint list with arrival radius — what usv_ui's `pathto` does, moved into the task
  layer, carrying usv_ui's calibrated factors), `scan_yaw.py` (hold position, sweep heading until the map has the object).
- **Tasks**: `optiprimex/usv/tasks/t1_safe_passage.py` (stages as `playbook.MISSIONS["m1t1"]`: WAIT_START → SEARCH_ENTRY →
  CIRCLE_ENTRY_CW → TRANSIT → EXIT → CIRCLE_EXIT_CCW → DONE; gate order from `PerceptionMap.gates`, GREEN to port / RED to
  starboard check, `SafePassageReport` body built for the judge), `test_path.py` (m0t2: PLAN → FOLLOW → DONE from a points file
  or the `pathgen` output). Register both in `usv/tasks/__init__.py`; set their `status: built` in the playbook catalogue.
- **Judge**: `ocs/report_mapper.py` maps the `safe_passage` event to the RobotX `SafePassageReport` (fake judge logs it).
- Accept: `make test` + `tests/test_t1_sim.py` (SIL: m1t1sim end-to-end on `world_pool` with the fake FCU — entry circle, 3
  gates in order, exit circle, DONE, report seen by the fake judge; a wrong-side gate → the task refuses to pass it and holds);
  live on the three hosts with `run_agx.sh --sim`; GUIDE 4.15 row + §7 leg "m0t2 path" updated; DEVLOG block.
- **Two planners, one switch** (decided 2026-11-02): `planner: simple` = the gate-midpoint chain above (what usv_ui `pathgen`
  builds by hand since v0.1.20, moved into the task layer); `planner: s6` = a Python port of pathing_sim S6/P4
  (`optiprimex/usv/gatepath/`: world-independent tracker, chain ordering, Dubins local path, LOS, search FSM — spec §12 layout),
  proven against the spec's §11 reference numbers (Dubins endpoint < 1e-6 m; seed 7 default course ≈ 49 s, 4/4 gates). Key in
  the usv profile (`tasks.t1.planner`), overridable per run in START_TASK args; both run in `tests/test_t1_sim.py`.
- First target: **m0t2sim on world_por** (the PoR course: through both gates from 3 m behind) — the PoR video is a deliverable.
- Open design points to settle with Dmitrii before coding (poll him): circle radius and speed; behaviour on a missing gate
  (hold vs continue); whether TRANSIT uses GUIDED carrots or mode-20 frames at pool speed.

### WP3 — v0.1.22 · m3t1 (Task 3 core) + m0t3  ✅ done 2026-11-03
Done (DEVLOG v0.1.22). Decided 2026-11-03: standoff 5 m, stop 1 m, firing 30 s (until GREEN, at most), all in `t3:` of the
usv profile; undock astern (the default chosen, not polled). The dock is read only through `usv/dockpath.dock_geometry` so the
students' box core (BOX_INSPECT / m0t4, `docs/merge/MERGE_NOTES_box_mission.md`) can feed the line + offset stage after the merge.
- **Behaviours**: `sway_transit.py` (sway along a line at range r, bow to the line — usv_ui's `swayto` with the calibrated stern
  factors F+ 1.05 / F− 0.877 carried into `gains`), `dock_los.py` (guarded bow-in approach into a bay: range / rate limits, abort on
  bumper or timeout, astern out).
- **Tasks**: `t3_logistics.py` (FIND_DOCK → SWAY_TO_ABEAM → TURN_IN → ENTER → DOCKED → REPORT_DOCK → AIM_FIRE → REPORT_FIRE →
  UNDOCK → DONE), `test_sway.py` (m0t3: LINE → SWAY → DONE). Cannon: the mission manager publishes `local/usv/cannon/aim` {on, bay}
  and watches `local/usv/perception/dock` for the window colour; `cv/cannon/cannon_sender.py` keeps the HMAC link to the Pi.
- fake_dock (v0.1.20) already runs the lights; the task sends `sim/dock/cmd {"cmd": "docked"}` when it reports docking in
  sim (the real DockingReport goes to the judge), publishes `local/usv/cannon/aim {on, bay}` at ≥ 2 Hz while firing (fake_dock
  ignores an aim older than 1 s), and reads the window's `colour_now` from the map. usv_ui `dockto` (v0.1.20, bench only) is
  the behaviour to move into `dock_los.py`.
- Accept: `make test` + `tests/test_t3_sim.py` (m3t1sim on `world_pool` with `fake_dock`: dock, report, window turns GREEN, report,
  undock); live sim on the hosts; GUIDE rows; DEVLOG.

### WP4 — v0.1.23 · RoboCommand for real: protobuf client, protobuf fake, rc-tui, rc-test  ✅ done 2026-11-03
Done (DEVLOG v0.1.23). Differences: rc-test runs natively (`scripts/run_rctest.sh`: our mosquitto on 1885 + RoboNation's
unmodified test_server.py) instead of docker (the daemon was not running; docker compose remains possible); rc-tui sends through
the fake judge by default, `--as-judge` itself, `--monitor` never; the course address comes from `RC_HOST` (network*.env), not
the profile; no broker ACLs (the course broker is RoboNation's).
- `ocs/judge_client.py`: `ProtobufJudgeClient` (RoboNation `robocommand` repo @ 8fc09f2: topics `robocommand/robotx/...`,
  protobuf messages; keepalive 15 s; `wait_for_publish`), selected by `judge: protobuf` in the OCS profile; the fake judge gets a
  `--wire protobuf` mode speaking the same messages so the bench never diverges from the course.
- **rc-tui** (`optiprimex/tools/rc_tui.py`, console script `rc-tui`): left column = what the judge receives from us (heartbeats,
  declaration, reports), right column = what it sends (course, RunStart, AssistanceRequest, KeepOutZone, ReadinessConfirm …), with
  typed commands to broadcast each by hand (`course`, `start`, `assist --lat --lon`, `confirm`, `keepout`, `allclear`, `moving`), a
  log of both; works against our fake judge (control topic) and as a monitor against rc-test (read-only).
- **rc-test on the Mac**: `docs/ROBOCOMMAND.md` — `docker compose up --build` in `RobotX_2026`, Mosquitto remapped off 1883 (e.g.
  1885) so it never clashes with our hub, `test_server.py` publishes the retained course, the log `./logs/robocommand_<stamp>.log`
  is the PoR artifact; `profiles/course/` (new, third profile: real judge, ACLs, `bench_mode: false`, `allow_goto_task: false`).
- Accept: `make test` + `tests/test_protobuf_judge.py` (round trip against the protobuf fake); live: declare → RunStart → T4
  chain against rc-test on the Mac with the log showing our messages; GUIDE §4 "the real judge" rows; DEVLOG.

### WP5 — v0.1.24 · camera stream AGX → Mac  ✅ done 2026-11-03 (tailnet; the pool switch not tried)
Done (DEVLOG v0.1.24). Differences: the frames come from the observers' existing previews (`latest_preview.jpg` every 15th
frame, `latest_combined.jpg` every 5th) — they write no `frame.jpg` every 200 ms, and the students' observer code is not edited
for it; `--source test` added (a pattern, for the sim and for measuring); `zed` written but not run (no ZED attached on 2026-11-03).
- `cv/stream/cam_stream.py`: MJPEG over HTTP (`http://AGX_HOST:CAM_PORT/stream`, ~640×360 at 5 Hz, `--source zed|dir`; `dir` =
  `runs/latest/frame.jpg` that our observers write every 200 ms — independent of the detector); `CAM_PORT` in network.env;
  `cv/stream/run.sh`; `run_agx.sh` starts it in its own tmux window (`--no-cam` to skip); the Mac opens it in the browser (map.html
  gets a "camera" link). Decided 2026-11-02: MJPEG over HTTP (simplest, any browser, no GStreamer on the Mac).
- Accept: stream visible on the Mac over the tailnet and the pool switch; CPU on the AGX measured and noted; GUIDE §3.3 row; DEVLOG.

### WP6 — later · UAV stack + m1t2
After the students hand over the drone stack: revise, bring it under `optiprimex/uav/` (ARCHITECTURE §10.4 sub-states, the
`uav_bridge.conf`), `UavT1Report` into the boat's map, m1t2. Not scheduled; depends on their delivery.

## 4a. Later (ideas parked by a session, not scheduled)

- Port the rest of pathing_sim S6/P4: its tracker (Gaussian fusion, rays, merge), the search FSM (sweep / advance / frontier)
  and the benchmark harness, then check the §11 numbers (seed 7 ≈ 49 s, success rates) — WP2 ported the planner leg only.
- m1t1's SAFE_HOLD on a refused gate could instead go round to the exit when the exit is known (operator's call).
- Merge the students' BOX_INSPECT (m0t4) work (`docs/merge/`) and decide whether m3t1's FIND_DOCK / TO_LINE use its box core.
- Camera: an automatic source switch for `cv/stream` (the ZED itself while no observer holds it, the observer's previews as soon
  as one runs — asked 2026-11-04; today the source is fixed at launch, `--cam-source dir|zed|test`).
- Camera: a faster preview from the observers (their preview rate is every 5th / 15th frame) - a change to cv/ snapshots, with
  provenance; `zed` source to try when the camera is on the AGX; the pool-switch run of the stream.
- `--help` of the older launchers uses GNU-only sed (`{…!p}` without `;`): fails on the Mac's BSD sed (the new scripts are fixed).
- FirefightingReport.window_id (1 upper / 2 lower) needs the window's height from the AGX; today 0 (unset) is sent.
- After an OCS restart the RxRequest seq starts at 1 again: check with RoboNation whether a re-declaration must keep counting.
- m3t1 advanced / disruptive: the GREEN-then-flashing colour read-out and ResourceDeliveryRequest (needs the UAV, WP6).
- RoboCommand's display (seen 2026-11-05): **a run ends when the last declared vehicle leaves AUTO** — on a single-vehicle run a kill or
  a takeover ends it; the OCS must say so (WP10d's judges' view) and `declare` again; with two vehicles declared the drone keeps the
  run alive while the boat is in MANUAL (and the other way round) — a reason to declare both even for a boat-only attempt (ask RoboNation).
- RoboCommand holds a UAV run until **Garuda** clears the flight (*Run Held — waiting for Garuda*, a CFMS flight id) — the "about a
  minute" of A4; the UAV adapter (WP14) shows it as such, not as a stall.
- T4 disruptive: the moving obstacle is **virtual** — RoboCommand drives it and sends `MovingObjectAlert` with `keep_out_radius_m`
  (RoboNation, 2026-11-04, A9): the standoff behaviour tracks it from the message stream only; the ack question is still open.
- m1t2 lights: RoboNation may light Task 1 beacons only while the **UAV** heartbeat shows the task at advanced / disruptive
  (power; "the USV can't see the light" — A6): push back (the boat needs the blue entry / exit lights during its transit), or make
  sure the USV heartbeat carries TASK_SAFE_PASSAGE while it transits; sequential UAV-then-USV reporting is acceptable to them.
- Bearing-only detections beyond the ZED depth range (pathing_sim's 25–40 m band, rays not fused) in fake_perception.
- The mission manager's gate list with a `min_obs` filter (usv_ui pathgen uses 5 since v0.1.20; the mm keeps 1).
- A usv_ui MAIN render test (curses-free) like op-tui's Canvas tests.
- world_por gate spacing from the real PoR build (15 m assumed; the handbook drawing is unreadable in the extract).
- `sim.html`: a browser page with controls for the sim (pick world, dock cmds, cannon aim) — today map.html shows the sim and
  `mosquitto_pub` drives it (GUIDE 4.14).

- WP17 (FREE RUN, v0.1.68) left out by its brief: a time factor for the ghost; `skip` on the real plant; free run with the fake FCU;
  a ghost UAV; the ghost's own noise model.

- The tk OCS console (session A, v0.1.84 Phase 1; docs/TK_CONSOLE.md §3b), parked 2026-11-11: the web-only tabs in tk (COMMS/SYSTEM,
  JUDGE, USV, UAV, SETTINGS, HELP); the launcher's `console: tk|web` field; themes beyond light / dark; a Canvas fence editor (the fence
  stays in map.html); the USV window in tk; the recorded-map replay (usv_map_rec/1, v0.1.81.1); one shared toast module for the console
  and launcher_tk (v0.1.83.3: today each has its own).

- Parked 2026-11-12 (competition night): WP42b = the cull's deletion of goto / m0t2 / m0t3 / m0t6 + the pool → lab profile move (after the
  competition; tonight only the hide-flag); a UAV-only declaration (`declare usv off`, the boat left out of the declaration and the
  heartbeats); the UAV team's `course_fence.py` containment case (the course inside the transit area → the transit polygon; theirs, read-only
  for us — `docs/fences/uav_fence_ALPHA_transit.csv` is the hand fallback); jtop on the Jetsons; clock sync of the Jetsons to the Mac
  (chrony, needs sudo once); a per-preset automatic course pick; WP40 IMU camera geometry; §2's "pool trials" target is superseded by §0.

- Parked 2026-11-13 (final day): the student's `agx_track_10_buoys.py` one-to-one digit assignment per frame (+ the decaying per-track
  digit score) for the marker numbers, once the observer reads them live - measured on the hand-labelled 12 Nov crops: wrong reads 47 → 36
  (student classifier) / 2 → 0 (our fine-tune), turned into "?" (`logs/cv_review/student_tracker_2026-11-13/NOTES.md`).

## 5. Pool-trial legs this roadmap serves (GUIDE §7 will grow with them)

a) waypoints / `pathto` / `swayto` through the final architecture = m0t2 / m0t3 (WP2 / WP3), b) `dockto` (WP1 by hand, WP3 as
a task), c) sway along a CV line (m0t3 with the dock mapper's line), d) `pathgen` from CV (WP1 by hand in manual mode, then
inside m0t2 / m1t1).

## 6. Do-not list for any session

- Do not merge two WPs into one version; do not re-architect what the WP does not touch; keep names (ADR-level changes are Dmitrii's).
- Do not change the wire (envelope, body schemas, topics) without a schema change + tests + a GUIDE note; mission codes stay in the
  operator layer.
- Do not add behaviours that move the boat without the same guards usv_ui has (ACTIVE / armed / idle / `sim` on a real link).
- Do not touch the students' folders on the AGX (`actual_buoy/`, `docking_cv/`), the provenance of the `cv/` snapshots (`cv/PROVENANCE.md`), `control_v2/`, `orin_kit/apply_params.py`.
- Do not put addresses into YAML or code: `network*.env` only; never archive them.
- Do not leave a version without a DEVLOG block and a GUIDE that names it.

## 7. Phase 2 — revision and re-integration (decided 2026-11-03 with Claude Fable, after the review of v0.1.20–v0.1.24)

The first pass (WP0–WP5) is built, tested (156) and rehearsed on the three hosts with the fake FCU. What it lacks is a human's
eyes and the real Pixhawk. Decisions polled 2026-11-03: **human validation first**; the students' BOX_INSPECT joins as a
**separate m0t4 first**, and is folded into m3t1 after it has been verified and run; the **real cannon link waits for the dock
detector contract** (the students' `dock_color_aim.v1` sender / receiver received 2026-11-03 — see the project doc
`claude/review_v0.1.19_to_v0.1.24.md` §4 for the six points to settle); a **hardware-day checklist** closes the ZED stream and
the pool-switch run. Same rules as §4 and §6: one WP per version, DEVLOG block before the archive, deploys gated on the
concurrency check (JETSON_LOG 2026-11-03).

**v0.1.25 is taken** (2026-11-04, a texts-only version by a Claude Code session: `run_agx.sh --help` on the three camera sources,
`run_rctest.sh` printing `tmux attach -t rctest`, GUIDE §3.3, docs/ROBOCOMMAND.md — CHANGELOG; no DEVLOG block, no archive). The
versions below were renumbered on 2026-11-04: WP7 = v0.1.26 and so on. **Validation so far (Dmitrii, 2026-11-04, before the pool
day)**: rc-test worked, the ZED stream worked (`--cam-source zed`, first time), the clean reinstall + USV and OCS against rc-test
worked; one finding — the judge's interactive console was not found (it is rc-tui `--as-judge` in window `rc` of tmux `ocs`, which
nobody looks at while watching `rctest`). GUIDE revised the same day (§1a, §3.1, §3.1a, 4.2c, 4.17c, §7.0).

**Order after the pool day (decided 2026-11-04 evening; superseded by §7a on 2026-11-05 for everything after WP9)**: v0.1.29.3 fixes → WP8c → WP9 (lab day) → **WP10 our simulator** (decided 2026-11-04 night: after WP9; the later WPs renumbered +1) → WP12 cannon (when the students
answer; else skip ahead and come back) → WP9a perception → WP9b recorder → WP9c declaration checks → WP9d usability → WP11 CV gate →
WP13 → WP14–WP15 UAV. Versions are assigned in that order; a WP skipped for a dependency keeps its number and takes the next free version
when it is built.

FCU modes of the final docking motion, for the record (asked 2026-11-03): m3t1 = FIND_DOCK GUIDED velocity-yaw (ScanYaw) →
TO_LINE GUIDED carrots (FollowPath) → SWAY_TO_ABEAM **mode 20** frames (SwayTransit) → TURN_IN **mode 20** (DockLos align) →
ENTER **mode 20** (DockLos, tapered surge, centreline + heading guards, stop 1 m) → DOCKED / AIM_FIRE HOLD (StationKeep) →
UNDOCK **mode 20** astern (DockLos). A zero frame on every mode-20 entry / exit; HOLD re-asserted every 5 s by the adapter.
BOX_INSPECT (students): GUIDED to the align point, then its own mode-20 frames for STRAFE / APPROACH, HOLD while observing
(alternating tick by tick = the 5 Hz mode thrash in `docs/merge/BOX_MISSION_REVIEW.md` §4.3 — to fix in WP8).

### WP7 — v0.1.26 · the validation pass: the 2026-11-04 findings, stage stepping, the known fixes  ✅ done 2026-11-04 (Mac only; DEVLOG v0.1.26)
Brief for the session that builds it (Dmitrii asks a Claude Code agent on 2026-11-04, the evening before a pool day): build
**exactly this list**, `make test` + the new `make quick` green, DEVLOG block, archive `MASTER_SCRIPTS_v0.1.26.tgz` into `_archive/`
and extracted over the Mac tree — **do not deploy to the Jetsons**: Dmitrii deploys himself after the pool day (GUIDE §1 / §1a).
Poll him (AskUserQuestion) only where a choice is his; the two decisions already taken are marked *decided*.
- **The judge's console, found at once** (finding 2026-11-04; *decided*): one tmux session named **`judge`** holds rc-tui whatever
  the judge is — `scripts/run_ocs.sh` (fake judge) opens it with `rc-tui --port 1883` (through the fake judge), `scripts/run_rctest.sh`
  opens it with `rc-tui --port 1885 --as-judge`, `--judge course` with `--monitor`; `stop_ocs.sh` / `stop_rctest.sh` close it; every
  launcher's last lines say `judge console: tmux attach -t judge`; window `rc` of tmux `ocs` keeps only the fake judge's log
  (fake) or nothing (rctest / course). rc-tui **decodes both wires by topic root** (`rc/#` JSON and `robocommand/robotx/#` protobuf,
  subscribed together) so `--wire` only chooses what `--as-judge` publishes — today `rc-tui` without `--wire json` shows empty
  columns against the fake judge. A `scripts/run_judge.sh [--port P] [--as-judge|--monitor]` for a console started by hand.
- **Stage stepping** (asked 2026-11-04 for the docking approach at the pool; *decided*: generic in the runner, not per task):
  START_TASK `args.step: true` (operator: `ocs_cli start_task m3t1 --step`, `ocs_cli mission m3t1 --step`) makes the task runner
  park the vehicle in **AUTO_SAFE_HOLD at every stage boundary** with `hold_reason "step: next <STAGE>"`; `ocs_cli resume` enters
  the next stage; `abort` / `hold` unchanged; the MISSION box on both consoles shows `STEP` and the next stage, op-tui's `next:`
  line says `resume -> <STAGE>`. `args.from: "<STAGE>"` (operator `--from TURN_IN`) starts a task at a named stage **only** for
  stages a task lists as entry points with their preconditions re-checked on entry (m3t1: TO_LINE, SWAY_TO_ABEAM, TURN_IN, ENTER,
  AIM_FIRE, UNDOCK — all need `dock_geometry()`; m1t1: SEARCH_ENTRY, TRANSIT, EXIT — the gate list / exit buoy; m0t2 / m0t3 / T4:
  none); a stage not listed or whose precondition fails → `REJECTED from: <why>`. Schema + tests + GUIDE note (§6 do-not list:
  a wire change needs all three); the `sim` guard and the mode-20 zero frames on every stage entry / exit unchanged.
- Fixes in one version: anything else he reports from the pool day's rehearsal, plus the known test-hygiene items: the sil1 race
  (`test_reset_both_ends_mission_preselect_and_test_missions`: the stack's fake judge auto-incident starts T4 while the vehicle
  waits in AUTO_HOLD — drain it as `test_mission_codes_and_sim_guard` does, or lengthen `--auto-incident-after` for that stack),
  the `network.ts.env` dependency of `test_cv_kit` (the archive excludes the file), BSD-sed `--help` in the older launchers,
  a `make quick` (unit + fast SIL of every mission, < 1 min) beside `make test` (8 min).
- From the RoboNation answers of 2026-11-04 (project doc `claude/robonation_questions_2026-11-04.md`, "Answers — block A"):
  `ProtobufJudgeClient` with a **persistent session** (`clean_session=False`, stable client id) and QoS 1 on the command
  subscription (RunStart is published once, not retained — A1); `profiles/course/ocs_course.yaml` `stall_s` ≥ 180 (a RunStart
  waits for the course manager, "might take a minute" — A4) and the R4 / stall wording "declared, waiting for the course manager —
  a refusal is shown on RoboCommand's display, ask the staff" (A2); `window_id` 1 = upper / 2 = lower documented (A10);
  SafePassageReport keeps re-sending on change, unlit buoys reported as `off` (A11, A12); a paused task keeps `current_task`
  (the lights stay on — A5; verified: the runner keeps the task while paused).
- GUIDE: §3.1 / §3.1a / 4.17 point at `tmux attach -t judge`; 4.2c, 4.16 and 7.3 F-4c get `--step` / `--from` (the F-4c note
  "no stage stepping yet" goes); §7.0b "`zed` never run yet" → ran 2026-11-04; the stall wording of 4.17c follows `stall_s`.
- Accept: 160+ tests incl. fast SIL m3t1 with `step: true` reaching DONE through `resume` at every boundary and `from: TURN_IN`
  reaching DONE on world_pool with the plant abeam the bay; `from: ENTER` without a dock in the map → REJECTED; rc-tui's test
  renders both wires; `make quick` < 1 min; `tmux attach -t judge` works for fake and rctest (rehearsed on the Mac only);
  the findings list closed in the DEVLOG block; GUIDE rows above updated; the DEVLOG block ends with what the next session does first.

### WP8 — v0.1.27 · m0t4 in, as a separate test mission (decision A)  ✅ done 2026-11-04 (Mac only; DEVLOG v0.1.27)
- Apply `docs/merge/box_mission_onto_v0.1.24.patch` as one version (applies with 0 rejects on v0.1.24: their 9 tests, the
  registry / playbook / cli / schema additions, `world_boxes.yaml`, `SELECT_TARGET`). Events `box_survey` /
  `box_mission_complete` stay OCS-internal (never judge reports: the protobuf client would drop them).
- Then the blocking fixes from `BOX_MISSION_REVIEW.md` §4: ALIGN yaw-in-place + timeout → HOLD (4.1), PREFLIGHT on
  `perception/health` not on detections (4.2), no HOLD / frame alternation while observing (4.3), blink requirement as a
  parameter (4.5); STRAFE with a length and a side (4.4) if cheap — else parked.
- The five questions of BOX_MISSION_REVIEW §7, answered by Dmitrii 2026-11-04: (1) the target is Task 3's docking bays (so
  the indicator is steady: blink requirement off by default); (2) the detector gives 3 points per face (left / mid / right —
  the face plane is fitted from them); (3) the vehicle picks the bay (GREEN rule; `SELECT_TARGET` stays an optional override);
  (4) stop distance a config key, default 1 m (was 5 m in theirs); (5) `box_survey` / `box_mission_complete` OCS-internal.
  It stays m0t4, built so it can become a stage of m3t1 later (WP13).
- Accept: 165+ tests; fast SIL m0t4 from off-axis scan points (30, 8) and (25, −10) DONE; live m0t4sim on the three hosts;
  GUIDE 4.18; the answers above recorded in the DEVLOG block.
  As built: world_boxes got the ZED's 25 m range (60 m hid §4.2), from which (25, −10) cannot see the blue face at all, so the
  off-axis runs are (35, 8) → red and (35, −10) → blue with a search heading and a strafe side; live m0t4sim ran on the Mac only
  (Dmitrii: no Jetson deploy) — the three-host run is his, after the pool day.

### WP8a — v0.1.28 · the OCS in the browser: op-tui's view, its commands, and a SYSTEM tab (decided 2026-11-04)  ✅ done 2026-11-04 (Mac; SYSTEM not run against the Jetsons)
Dmitrii: the page `run_ocs.sh` opens should replicate op-tui "in the best way" — smarter grouping, tooltips, drop-downs — with
op-tui still the main console; the map moves to a secondary tab; a dedicated section to restart / reboot the pieces (Jetsons,
FCU, Mac side …) and to choose how they run (real or fake FCU, lab / pool profile …). Polled: all op-tui intents; tabs like op-tui;
built now (no hardware), WP9 → v0.1.29.
- `tools/ocs_web.py` (`ocs-web`, port OCS_WEB_PORT 8096): a Python server like judge_web on op-tui's own `Model` / `Feeds`,
  commands through `ocs_cli.execute` with op-tui's grammar (intents + the fake plant's words) — one source of truth, both consoles
  at once; the playbook's verdicts grey out what is not allowed now and say why; tooltips from the playbook entries.
- The page `tools/ocs_web.html` (plain HTML/JS, no build, no external resources): a top bar always visible (R / V ladders, the next
  step, warnings, links), tabs MAIN · MISSION (a mission picker with its options as drop-downs → the command line) · BOAT · NETWORK ·
  JUDGE · SYSTEM · MAP (map.html) · LOG · HELP.
- SYSTEM: restart the Mac's OCS stack (judge fake / rctest / course, ts / lan), the judge console, rc-test; the NX's boat stack
  (`--fcu fake|uart|usb|eth`, `--pool|--lab|--course`, LOG) and the AGX's (real / `--sim` + world, the camera source) over ssh
  (ORIN_SSH / AGX_SSH); reboot the FCU (`tools/fcu_reboot.py` on the NX through usv_ui's share, refused when armed) and the
  Jetsons (`sudo -n reboot`). Guards: nothing that touches the boat while the vehicle is in AUTO; the OCS stack not during a run
  (R3–R5); a typed confirmation for every reboot; every action logged. The ADR-9 NEVER list stays: no ARM / DISARM / SET_MODE /
  ACTUATOR from the page. (Amended v0.1.50, Dmitrii 2026-11-06: MANUAL / HOLD / ARM / DISARM buttons are allowed; ACTUATOR stays out.)
- Accept: tests for the snapshot, the command path, the verdicts, every SYSTEM action in dry-run with its guards, fcu_reboot
  against the fake FCU (refused armed, accepted disarmed); `run_ocs.sh` opens the page (MAP=0 = not); GUIDE rows.

### WP8b — v0.1.29 · the OCS at the pool: points from the OCS, judge buttons, MAIN for the run (decided 2026-11-04)  ✅ done 2026-11-04 (Mac; deploy at the pool)
Dmitrii at the pool, before m0t2: save points from the OCS; the judge page with rc-tui's buttons, better help and tooltips; on MAIN
the mission-specific commands and the mission picker; problems highlighted in the logs; a streamlined clearing for the run; a peek
at RoboCommand (did the declaration get an answer). Polled: WP8b = v0.1.29 (WP9 onward +1); points = the boat's fix **and** a click
on the map; clearing = **both** a checklist with a next-step button and a one-shot `clear for run`; the peek = a panel **and** a live
judge feed.
- `save_point NAME [--lat --lon]` / `del_point NAME` (ocs_cli, op-tui's grammar, the page): intents SAVE_POINT / DELETE_POINT (any R,
  run-free on the vehicle) -> the mission manager writes usv_ui's points.json (`usv/points_store.py`: re-read, change one name,
  atomic replace); usv_ui reloads the file on change and saves atomically (neither writer drops the other's points); the map digest
  carries `points` (+ `points_ll`). The map (in the page) has `drop point`: a click -> a name -> save_point with that lat / lon.
- The judge page: JUDGE ACTIONS (course, RunStart, assistance, readiness confirm, keep-out, all clear, moving object) with what each
  means for us, defaults from the course / the heartbeat; sent by rc-tui's own `JudgeSender` (fake judge / --as-judge / --monitor).
- MAIN: RUN CHECKLIST (each condition with its fix, bold = hard) + `next: …` + `clear for run` (`ready` once the hard ones are
  green); MISSION picker + the selected mission's controls (m0t2 / m0t3 points to tick into --names, save / drop; m0t4 colours; step;
  hold / resume / abort); ROBOCOMMAND (the declaration and the judge's answer, RunStart, reports, a live judge feed - read-only on the
  judge's broker); every log coloured by severity, LOG `problems only`.
- Accept: tests for the points path (store, usv_ui merge, live through the OCS to the file and the map), the judge buttons, the
  checklist / next / clear, the peek, the page parts; the bench walk on the page (save, drop, m0t2 EN -> G1, clear for run -> R5).

### v0.1.29.3 — the pool-log fixes (first, before WP8c; decided 2026-11-04 from `docs/POOL_REPORT_2026-11-04.md` §7)  ✅ done 2026-11-04 (Mac; deploy on the lab day)
- **F1 `rc_ok` / `RC_LOST`**: `health.rc_ok` was False and the fault `RC_LOST` raised the whole pool day on the real Pixhawk while the
  transmitter was live (RC5 kill and RC6 override worked). Find which signal the real link lacks through usv_ui's share (SYS_STATUS's
  RC_RECEIVER health bit, RC_CHANNELS / RC_CHANNELS_RAW freshness, the stream rates the share forwards — a MAVLink dump on the NX tells)
  and fix it at the source; the bench fake FCU must still show `rc ok`. Accept: a test per source; rehearsed on the lab day — transmitter
  on → `rc ok`, off → `rc NO` within `rc_lost_s`, back on → `rc ok`; op-tui / the page agree.
- **F5** `health.perception_age_s` = 1e9 before the first detection → `null`, shown as `never`. **F6** the refusal *RC override active
  (RC6 high or FCU in MANUAL)* → say what to do: *the FCU is in MANUAL (RC6 override or the transmitter's mode switch): put it in HOLD,
  then enable_auto*; the page's RUN CHECKLIST gets the row.
- Accept: `make quick` green, the three behaviours tested, GUIDE 4.4 / 6.1 texts, DEVLOG; a fourth-digit version (texts + health).

### WP8c — v0.1.30 · go-home override, the m0t6 orbit test, the pool's guards (decided 2026-11-04)  ✅ done 2026-11-04 (Mac; deploy on the lab day)
Polled at the pool: HOME = the saved point HOME, else the control origin; the override acts from any AUTO state with or without a
run (refuses from MANUAL), always visible on the page + `home` in op-tui, our own slow GUIDED task (not ArduPilot RTL); m0t6
TEST_ORBIT round a saved point or a map click (radius, cw / ccw, turns, speed; APPROACH → ORBIT; --step / --from); plus the guards
the pool showed (m0t2 without --names and no gate refused at once; MAIN's start greyed until points are ticked; save_point usage;
the checklist says m4t1 waits for the judge's assistance; **`--names` validated at the OCS** — split on commas and spaces, every
name checked against the map digest's `points` before the command is sent: at the pool `--names "1 2 3"` reached the boat as one
name and failed 8 s later, report F2). The proposals after it (declaration checks, a high-contrast theme, the
simulation day, venue data) are in docs/NEXT_STEPS_2026-11-04.md, to be discussed.

### WP9 — reduced 2026-11-05 evening (§7c): the lab day checks switches, connections and real CV; **the mode-20 walk DROPPED 2026-11-07 (§7f)** — the next free version when built (v0.1.31 went to WP9a) · stage B: the new behaviours on the real Pixhawk (--lab, props off, ESC power off) — the lab day
**Pool day 2026-11-04, result** (`docs/POOL_REPORT_2026-11-04.md`): the state machine walked on the real Pixhawk (m0t0 stepped, m0t1 ×4)
and **m0t2 drove the hull** (`--names 1,2,3`, 99 s, one GUIDED request, points passed at 0.35 / 0.43 / 0.08 m); `mode-log --pool`
PASS on both sessions and the rows are in GUIDE §6.2. **Still open — this WP**: no mode-20 stage ran; the lab day (Friday, under the
sky) does the trolley walk below with props off before any sway / docking leg on the water. Boats logs home in `logs/pool_20261104/`.
- `run_usv.sh --fcu uart --lab`, usv_ui `handover`, `ocs_cli start_task m0t3 --names A,B` (two saved points, a metre apart),
  m3t1 stages as far as the dock line allows, m1t1 SEARCH_ENTRY / CIRCLE on a fake entry point: what is checked is the **mode
  sequence and the frame cadence** in usv_ui's THRUSTERS/RC box and the mm log (HOLD → CUSTOM_MANUAL with a zero frame → frames
  → zero frame → HOLD), not motion. `mode20.py` re-implements usv_ui's `sway_lat` allocation: compare the frames of usv_ui
  `swayto` and of m0t3 on the same two points.
- Carried from v0.1.19: KILL / ESTOP as NAMED_VALUE_INT confirmed at L1; `guards.link_loss_action: safe_hold` live.
- Accept: a GUIDE §6.2 table "mode sequence per stage, as observed" with dates; no mode thrash; the §6.1 latches hold during a
  task; the frame comparison recorded.

### WP9a — v0.1.31 · perception hygiene: no ghosts, colours in the map (decided 2026-11-04 from the two-buoy test)  ✅ software half done 2026-11-04 as **v0.1.31** (WP9 waits for the lab day and takes the next free version then, the order rule above); **the lab-day half** (the ZED colour thresholds, the acceptance with two real buoys) stays open
- `PerceptionMap` digest: a `perception.min_obs` profile key (default 3) — entities below it stay internal, never in the digest, the
  map or a task's gate list (the raw count stays in `cv status`); ghost ageing (an entity not seen for `perception.forget_s` is dropped).
  At the pool entities with 1–3 observations appeared and vanished beside two tracks with 280 / 270 observations.
- `cv/buoys_multi` on the lab day with the ZED: `--min-share` / `--sat-min` tuned on the real buoys (red, green, the blue of the pool
  markers), the thresholds written into `cv/buoys_multi/run.sh` defaults with the date; colours arrive in the map (`cv status` lists
  them; map.html colours the glyphs). Accept: on the lab day two buoys → two coloured entities, no ghost in the digest; tests for
  `min_obs` / `forget_s`; GUIDE 4.14a / 7.2 T-B rows.

### WP9b — v0.1.40 ✅ (2026-11-06, built in a parallel stream; the 60 s AGX clip waits for the deploy) · the ZED recorder + the event's data pipeline rehearsed on the bench (decided 2026-11-04, NEXT_STEPS §6c / §7)
- Our observer holds the camera, so the recording lives in `cv/buoys_multi/multi_buoy_observer.py`: `--record DIR [--max-min N]`
  → `DIR/<session>/cam.svo2` through the ZED SDK's recording (H.264), beside it `frames.jsonl` (per frame: t, the boat's lat / lon /
  heading from PixhawkTelemetry, the detector's buoys with colours, the frame id) and `session.json` (label, start, versions). Stops
  at `--max-min`, or when the AGX has < 5 GB free (it has ~26 GB) — then warns on the page. `scripts/agx_record.sh start LABEL | stop |
  status` (like agx_cam.sh), a card under ⚙ SETTINGS / CAMERA. Replay: `multi_buoy_observer.py --svo FILE` runs the same loop from the
  file (the ZED SDK opens an SVO like a camera) → the detector's output for a recorded clip, offline — the "CV in the loop" of
  NEXT_STEPS §6 option (c) without a closed loop.
- Accept: a 60 s clip on the AGX opens with `--svo` and yields the same buoys; the free-space guard tested with a stub; `--help`; a
  `cv/buoys_multi/README.md` paragraph (what to label, where the clips go, how to bring them home).

### WP9c — v0.1.41 ✅ (2026-11-06, built in a parallel stream) · RunDeclaration checks in the OCS (decided 2026-11-04, NEXT_STEPS §4)
- Before `declare` and on `start_task` / `mission`: every task the profile declares has a tier (`declaration.tiers`), the preselected or
  started mission's task is in the declaration **with the same tier** (m1t1 under TIER_CORE, m1t2 needs TIER_ADVANCED …), test
  missions (m0t*) and `goto` inside a judged run, a declaration with TIER_NONE for a task the handbook makes mandatory. Policy per
  profile: `declaration.policy: refuse | warn` (course = refuse, pool / bench = warn). Shown in the page's RUN CHECKLIST (hard rows block
  `clear for run`), `ocs_cli next`, op-tui's `next:`.
- Accept: tests for each check and both policies; the handbook's mandatory list cited (`handbook/`); GUIDE 4.2a / MAIN row; no wire change.

- **Later (from WP9c)**: a check that a declared tier's vehicles exist (§3.3.2–3.3.4 Advanced tiers need the UAV; T2 needs a UUV +
  UAV) - not in the WP; and **ask RoboNation / the TD whether T4 is mandatory in every declaration** (our reading of §3.3.1 / §3.3.5).

### WP9d — v0.1.42 ✅ the LIGHT cut (2026-11-06, polled: no directory moves - presets sim / lab / event, the profile table, the flavour line, ten words, the refusal sweep; the full cut, up.sh / down.sh and the DAY PRESET card stay open - the launcher covers them) · THE PROFILE CUT (sim / lab / event, decided 2026-11-05 evening) + usability (the rest): day presets, the flavour line, ten words, the refusal sweep — STOP EVERYTHING and the page-first rule moved to WP10d (decided 2026-11-04 evening)
Decisions polled 2026-11-04 (all four): presets as the single switch; **the OCS page is the primary console** from now on (op-tui the
complete fallback; the judge page instead of rc-tui for the operator, rc-tui stays for a judge seat; usv_ui stays the safety console
on the boat — decision 25); STOP EVERYTHING = all stacks, refused while a task runs; the simulation day runs **lab-sim on the three hosts**.
- **STOP EVERYTHING**: one button in the page's top bar (always visible) + `stopall` in op-tui / ocs_cli, confirmed by typing `stop`.
  Order: the NX stack (usv_ui gets `quit` — mode-20 outputs zeroed; the FCU stays as the transmitter left it: the companion never arms
  or disarms), the AGX stack + camera + recorder, the judge console, rc-test, the OCS stack last (the page says goodbye and stops
  polling). Refused while the vehicle is in AUTO_TASK ("abort first"). Works over ssh on either network; every step logged to
  `system_actions.jsonl`; `status.sh` clean on each host afterwards.
- **Day presets**: `profiles/presets.yaml` — `bench` (laptop only: run_bench, fake judge, a world), `lab-sim` (NX `--fcu fake --pool`,
  AGX `--sim --world pool`, Mac fake judge or rc-test, tailnet), `lab-pixhawk` (NX `--fcu uart --lab`, AGX real or off, Mac fake judge,
  tailnet), `pool` (NX `--fcu uart --pool`, AGX real + camera, Mac fake judge or rc-test, `--lan`), `course` (NX `--course`, Mac
  `--judge course`, `--lan`; RC_HOST from network.lan.env). Each preset = judge, net profile, FCU, mm profile, CV mode / world, camera
  source, LOG, MAP. `scripts/up.sh <preset> [--judge rctest] [--lan|--ts]` and `scripts/down.sh` on every host compose **today's
  launcher flags** (thin wrappers; the launchers' logic unchanged; `up.sh --show <preset>` prints the exact commands); a DAY PRESET card
  in ⚙ SETTINGS runs them over ssh on all three hosts (one job each, the guards of SYSTEM). The old flags stay for the rare case.
- **The flavour line** on every console's top bar: `preset · judge · net · FCU · mm profile · CV · camera` (op-tui WHERE WE ARE already
  knows most of it; the page's top bar, usv_ui's bar, the judge page). The preset name comes from `up.sh` (an env / a file in `logs/`),
  `?` when a stack was started by hand with flags.
- **The page first**: new controls land in the page, op-tui follows in the same version only for the safety-relevant ones (stopall,
  home); GUIDE §3 / §4 rows name the page's tab before op-tui's word; a **ten-words card** in the HELP tab and GUIDE §9 (`ready`
  `start_task` `hold` `resume` `abort` `manual` `end_run` `reset` `save_point` `home`); a sweep over every refusal text so each says
  what to type next (F6 was the example).
- Accept: a test maps every preset to its flag set and refuses an unknown one; the dry-run of STOP EVERYTHING in order and its guard;
  `up.sh lab-sim` / `down.sh` rehearsed on the three hosts; the flavour line on all four consoles; `make test`; GUIDE §2 / §3 get a
  "the short way" paragraph (`up.sh`) above the per-host tables; the pool page and the scripts reference regenerated.

### WP9e — the simulation day on lab-sim (a scenario sheet + one day, no version unless something breaks)
`docs/SIM_DAY.md` (to write, from GUIDE §4 and the playbook): every mission start to end on the three hosts with the fake FCU and the
sim CV (m0t0, m0t1, m0t2 with saved points and from the map, m0t3, m0t4 on world_boxes, m1t1 both planners, m3t1, m4t1 with the judge's
assistance request); kill / RC6 / E-stop / FCU loss / link loss in each state; judge refusal and stall; the go-home override from every
AUTO state (after WP8c); `--step` through m3t1; STOP EVERYTHING at the end (after WP9d). Each row: who types what where, what the page
shows, what the logs must contain; findings → the next version. Date: after WP8c and WP9d exist (presets make the day start in one word).

### WP10 — our simulator: a pseudo-3D world, a rendered camera, the CV in the loop (decided 2026-11-04 night)  ✅ first version done 2026-11-04 as **v0.1.32** (world, renderer, camera stream, SIM page, arrow keys; built before the lab day so the day can be rehearsed - Dmitrii); **the second version** (the sim detector in the loop, usv_ui on the fake plant in run_sim.sh, the scenario sheet) stays open
Dmitrii: "our own simulator … a simulation-esque world, pseudo 3D for the visual, like the path-planner prototype (`pathing_sim/`);
mockups in it, linked to the fake FCU, plugged into our OCS and the rest; the camera feed of primitives (blue water surface, white
buoys, light grey background); with a CV module we check the main principles of the missions; main scripts in Python, the frontend
HTML". Decided with him: our own, not VRX (RobotX's Gazebo / ROS 2 simulator - too heavy for now); **the renderer in Python**;
**no disturbances and no real physics first** (the fake FCU's kinematic plant moves the boat); **arrow keys drive it in MANUAL**;
**its own window**, started by its own script or from the OCS - **not inside the OCS page**; after WP9.
- **World**: the world files (`profiles/bench/world_*.yaml`) + mockups: buoys (colour, size, shape), dock boxes, a wall / shoreline,
  placed or dragged from the SIM page; the truth stays the world file (the fake perception and the map's truth layer agree).
- **Movement**: the fake FCU's plant, unchanged in GUIDED and mode 20; **new: MANUAL sticks** - the fake FCU takes
  `{"set": {"sticks": {"surge", "yaw", "sway"}}}` on its control port with a short timeout (released keys = zero), so the SIM page's
  arrow keys (↑↓ surge, ←→ yaw, a/d sway) drive the boat while the vehicle is MANUAL - as the transmitter would.
- **Camera**: `optiprimex/sim/` renderer in Python (numpy + Pillow, sim-only dependencies): a pinhole camera with the ZED's field of
  view from the boat's pose; blue water with a little texture, a light-grey background and horizon, buoys as shaded spheres /
  cylinders, the dock boxes, the lit indicators; a depth image beside it. Published with the **same interface as `cv/stream`**
  (`/stream`, `/snapshot.jpg`, `/status`) - the OCS page's CAMERA tab and map.html's camera link show it unchanged.
- **CV in the loop**: a sim detector (colour segmentation + the rendered depth) writes the same `latest_status.json` as
  `cv/buoys_multi` → `cv_publish` → the mission manager's map, gates, tasks: pixels → detection → map → mission, end to end. (The
  students' model on primitives: optional, later, on the AGX.) `fake_perception` stays for the fast geometric runs.
- **SIM page** (own server, own port, own window): a pseudo-3D view (perspective, after the path-planner sketch), a top-down view,
  the camera feed, the world / mockup controls, fault injection through the fake FCU (kill, RC6, E-stop, GPS loss, FCU pause), the
  arrow-key driving. `scripts/run_sim.sh` = the laptop bench (with usv_ui on the fake plant, the OCS page and the judge page - the
  bench gaps found 2026-11-04) + the renderer + the sim detector + cv_publish; the OCS page's SETTINGS gets "open the simulator".
- Then the scenario sheet `docs/SIM_DAY.md` (WP9e) runs on it.
- Probably two versions: (1) world, renderer, camera stream, SIM page, arrow keys; (2) the sim detector in the loop, run_sim.sh,
  the scenario sheet and a full run. Accept: the CAMERA tab shows rendered buoys; arrow keys drive in MANUAL; a buoy pair placed on
  the SIM page reaches the map through the sim detector and m0t2 / m1t1 run on it; tests for the projection (a buoy at a known range
  / bearing lands at the right pixel and depth), the sticks timeout, the detector on rendered frames.

## 7a. The simulator-and-GUI programme (decided 2026-11-05, after the sim-day sheet and RoboCommand's own display)

Decided with Dmitrii on 2026-11-05 (polled): the **neural detector is tried on rendered frames** (a third agent trains it, see 7b);
the latch = **TEST MODE** — stay ARMED across missions and after a kill, no repeat loop; **STOP EVERYTHING and RESET EVERYTHING** as two
buttons; **all of it in one go**, in the order below (each a version, tested on the one before). Defaults move to **Marina Bay**:
RoboNation's own Course Alpha from `rc_test/test_server.py` is a true rectangle, 30.1 × 25.0 m, corners 90.0°, long side on bearing
032°/212°, centre **1.2810202 N 103.8550097 E** (55 m west of the Marina Bay City Gallery shore point) — the simulator, the profiles
and rc-test then agree to the metre; the RC course replaces it when received (that path exists). Two screenshots of RoboCommand's
display (SV2 build 2026.11.02, a UAV run on 2026-11-05) taught: the run timeline is **Declared → Held (waiting for Garuda to clear
the UAV flight) → Released → Started → Ended**; **the run ENDS when the last declared vehicle leaves AUTO** ("UAV1 left AUTO (MANUAL) —
the last vehicle out") — a kill or a takeover on the USV ends a single-vehicle run, redeclare afterwards; per task it shows the tier,
*challenge activated by* (= `current_task` in the heartbeat), *reported*, *actual*; vehicle cards show state, task, heading, speed,
HAE, flight phase, registration and the heartbeat age ("No heartbeat 789 s"); the map draws the course (solid) and the UAV geofence
(dashed) with the vehicles; it has a dark / light / high-contrast switch. We mirror that view so the operator sees what the judges see.

### WP10b — v0.1.33 · sim plumbing: rc-test, RESET SIM, teleports, Marina Bay defaults, the dataset export ✅ (2026-11-05)
- `scripts/run_sim.sh [world] [--judge fake|rctest]` (and `run_bench.sh`'s `OCS_CONFIG` / `--judge`): rctest = the OCS on
  `profiles/course/ocs_course.yaml` with `RC_HOST=127.0.0.1 RC_PORT=1885`, no fake judge window, the judge console (`run_judge.sh
  --port 1885 --as-judge`) and the pages' peek on 1885; `run_rctest.sh` may already be up (its judge page keeps 8095, the sim's `jweb`
  is skipped then). The course profile's rules apply as at the venue (no `goto`, no `inject`; T4 from the judge page).
- **RESET SIM** (SIM page + ⚙ SETTINGS, confirmed by typing `reset`): the boat to the world's `start` (pose + heading), mockups back to
  the file, every fault off (kill, RC5, RC6, GPS, FCU pause), the mode switch to HOLD, `clear_map`, `reset` on both ends, the fake
  judge's counters (a `{"kind": "sim_reset"}` control). One POST `/reset` on simview.
- **Teleports**: a `teleports:` list per world file (name, x_n, y_e, hdg) → buttons on the SIM page and on the OCS MAP tab (`pool`:
  start · before gate 1 · abeam bay 2 · N 32 E 3 facing the dock; `boxes`: the scan point; every world: HOME when saved); **teleport
  by click** (press *teleport*, then click the map — snaps to HOME / a saved point within 2 m — or type lat lon [hdg] beside the
  button); `teleport` keeps its heading argument everywhere.
- **Marina Bay defaults**: `profiles/*/usv_*.yaml` `frames.origin` and `profiles/*/ocs_*.yaml` `course.default` = Course Alpha's centre and
  rectangle above; `world_pool.yaml` / `world_por.yaml` / `world_boxes.yaml` / `world_empty.yaml` origin moved there (the pool frame stays
  x_n north / y_e east from the origin); the OCS shows the default boundary before any course arrives (dashed, labelled *default*) and
  the RC course once received (solid); the **geofence editable** on the MAP tab (drag corners, add / delete a corner, persisted in the
  profile's `geofence:` section, shown dashed as on RoboCommand's display) — for the UAV it becomes the declared polygon (WP14), for the
  boat a display today and a soft guard later.
- `python -m optiprimex.sim.render --dataset DIR --n N --world FILE [--seed]`: N rendered frames from random poses inside the course with
  **YOLO labels** (boxes, and masks as PNG) and depth — the synthetic data for 7b, the first thing the CV agent needs.
- Accept: `run_sim.sh pool --judge rctest` → R5 on the real wire, the T4 chain from the judge page, the rc-test log; RESET SIM from a
  killed, teleported, mocked-up state back to a clean R2 / MANUAL / start in one click (test); teleport by click snaps to HOME; the
  default course drawn with no judge, replaced by rc-test's (the same rectangle, now solid); the dataset export opens in Ultralytics
  (`yolo val` on 10 frames); `make test`; GUIDE rows; the sim-day sheet re-run on it.

### WP10g — v0.1.34 · buoy localisation: the real camera, range = surface + radius, far buoys as rays, ZED-like depth ✅ (2026-11-05)
CV_TRAIN's `REQUEST_localisation.md` (2026-11-05), polled: all five items in one package now, before WP10c (the later WPs +1).
- **One camera model** = the ZED 2i SN33545355's factory calibration (`optiprimex/sim/zed.py`, read from the AGX): hd2k fx 2317.2 →
  50.9° (was 72° in the world files, 100° in the renderer); the renderer, simview, the dataset export and fake perception (a world's
  `camera.intrinsics: zed2i_hd2k`) derive from it.
- **Range = surface + 0.85 r** (`cv/buoys_multi/multi_core.centre_correction`, r per type, 0.25 m: the handbook gives no RoboBuoy size);
  `surface_range_m` kept. Sim bias after it ≤ ±0.05 m.
- **Far buoys**: decided with Dmitrii 2026-11-05 after reading pathing_sim (spec 5.3, 410): **the pathing_sim way** - a confident
  detection without depth is a ray (a lead: drawn on the map, kept 30 s, dropped when a ranged buoy turns up along it), never a position;
  a position only from depth, the **depth limit raised 20 → 25 m** (`buoys_multi --max-depth`, the spec's R for the 4 mm lens).
  Triangulation of crossing rays is built and tested but off (`perception.triangulate: true` for tests). fake perception reports
  bearings 25–40 m.
- **ZED-like depth** (`zed.stereo_noise`: σ_z = z²σ_d/(fB), holes on flat surfaces, flying pixels; `render --dataset --depth-noise zed`).
- **Pool shot list** `docs/POOL_CALIBRATION.md` (5–30 m, ±30°, the overlap, the bearing band) → the real error, the 25 / 30 m decision.
- Found by the real field of view and fixed: an unlit buoy far off voting blue again (the 2026-11-02 bug) → a lit colour on a buoy dark half
  the time must double its runner-up, colour votes weighted by range; the dock seen from 5 m shows one bay → the TURN_IN pose / teleport
  9 m off; docked 1 m off the face the windows are out of view → bench `t3.stop_m` and usv_ui's dockto 1.5 m.
- Later (not built; **v0.1.39 WP10h: the stop is 1.5 m everywhere, the search drives toward rays, the seeds swept - `docs/CAMPAIGN.md`**): **the pool / course `t3.stop_m`** (1.0 m; the windows' real spacing decides); **m1t1's entry decision under
  miscolour** (6 of 8 seeds failed from a shifted noise stream alone at 72° - fixed by the rule above, but worth a dedicated robustness
  sweep over seeds and fields of view, ROADMAP §4a with the S6 tracker port); the search driving toward rays (pathing spec 335 "pending").

### WP10c — v0.1.35 · map interaction: add object, hover coordinates, tooltips, the mini-map on MAIN ✅ (2026-11-05)
- **Add object** on the OCS MAP tab and the SIM page: press *add object*, pick the kind — a buoy (colour red / green / blue / black /
  yellow / white, light flash / steady / off), a dock bay (colour, bay number), a box face, a **waypoint** (= `save_point`, asks the
  name), an **anchor point** / a marker (a named note on the map, OCS-side, never sent to the boat) — then click where it goes; drag
  moves, Delete removes. Sim mockups go to simview (`/objects`) only while a simulator is attached (the page knows from CAM_URL / the
  SIM page's `/status`); waypoints and markers always work (the real map too).
- **Hover** on both maps: lat / lon (7 decimals) and local N / E in a corner; click copies them. Range / bearing from the boat on
  hover over an object.
- **Tooltips everywhere**: every button and field of the OCS page, the SIM page and the judge page says *what · needs · then* (the
  OCS page has the playbook texts; the SIM page and the judge page get a small catalogue of their own); a `?` toggles a legend on each
  map (symbols, colours, the dashed / solid boundaries).
- **Mini-map on MAIN**: a square block on the right (≈ 320 px): the course boundary (default or RC), the geofence, the UAV transit
  area when known, the boat with heading and track, HOME, the points, the current path and carrot; click = the MAP tab. The same
  `map.html` in a `?mini=1` mode (no controls), so one code path.
- Accept: a buoy added on the SIM page appears in the camera picture at once and, after WP10e, in the map; a waypoint added on the
  real map lands in usv_ui's points.json; hover readout matches `teleport`'s coordinates; a headless-Chrome check of the tooltips (every
  `button` has a `title`); the mini-map renders with and without a course; `make test`; GUIDE.

### WP10d — v0.1.37 · the operator layer: TEST MODE, MISSION CONTROL, STOP / RESET EVERYTHING, judge tables, the judges' view ✅ (2026-11-05)
- **TEST MODE** (`test_mode: true` in usv_pool / usv_lab / usv_bench, `false` in usv_course; mirrored in the OCS profiles): with the boat
  handed over and in AUTO (`enable_auto`), the **run-free tasks** start without declaration or RunStart — `TEST_MANUAL`, `TEST_AUTO`,
  `TEST_PATH` (`pathto`: saved points), `TEST_SWAY` (`swayto`: two points), `TEST_ORBIT`, `GOTO` (a waypoint), `RETURN_HOME`, `BOX_INSPECT`;
  the judged missions (m1t1, m3t1, m4t1, later m1t2) keep the run lifecycle. The vehicle FSM (`can_start_task`), `run_context`
  (RUN_FREE_ACTIONS), the OCS authority and the playbook agree; the heartbeat carries TASK_NONE for them as today. Nothing changes in
  the guards: handover, armed, EKF, FCU not in MANUAL, kill clear, the transmitter. **RE-ARM** = `enable_auto` after a kill (KILLED →
  `confirm_safe` → MANUAL → RE-ARM → AUTO_HOLD); the run context, if any, survives as it did at the pool.
- **MISSION CONTROL tab** (the page; op-tui gets the words): one state word — **NOT READY** (and the one thing missing: handover /
  arm / EKF / FCU in MANUAL / kill / RunStart for a judged mission) · **ARMED** · **RUNNING** (task, stage, elapsed) · **HELD** (why) — the
  picker (primitives first: go to waypoint, path through points, sway A→B, orbit, home; then the test missions; then the judged ones),
  the options as today, the stage chain, and six buttons **START · HOLD · RESUME · ABORT · RESTART (abort + start the same) · ⌂ HOME**,
  plus RE-ARM when applicable. MAIN keeps the full command list.
- **STOP EVERYTHING** / **RESET EVERYTHING** (top bar + ⚙ SETTINGS): STOP = all stacks down in the agreed order (NX usv_ui `quit` → mode-20
  outputs zeroed, the FCU as the transmitter left it; AGX + camera + recorder; judge; rc-test; the OCS last), refused in AUTO_TASK;
  RESET = RESET SIM when a simulator is attached + `reset` on both ends + the fake judge's counters, stacks stay up. Both confirmed by
  typing the word.
- **Judge page as tables**: RECEIVED and SENT as scrollable tables (time · direction · kind · seq · vehicle · the key fields · ok / warn),
  a filter by kind, one live row per vehicle for the heartbeat (state, task, position, speed, heading, age), the declaration and the
  course as cards above; the typed commands stay.
- **The judges' view** (our page's ROBOCOMMAND panel and the judge page, after the screenshots of 2026-11-05): the run timeline
  Declared → Held → Released → Started → Ended with times and reasons (**Ended: the last vehicle left AUTO** shown in red when it is
  us), vehicle cards (state, task, heading, speed, HAE, phase, registration, **heartbeat age**), per task: tier · *challenge activated
  by* (`current_task`) · reported · actual; the map with course (solid) and geofence (dashed). The stall / hold wording gains
  *Run Held — waiting for Garuda to clear the UAV flight* for UAV runs (WP14). A dark / light / **high-contrast** switch under ⚙ SETTINGS
  for the OCS page, the judge page and the maps (NEXT_STEPS §5).
- Accept: in test mode `start_task m0t2 --names A,B` from AUTO_HOLD with no run → runs; on the course profile → refused with
  *declare first*; m1t1 still needs R5 in both; RE-ARM after a kill keeps run 1; MISSION CONTROL's state word follows the ladder in a
  headless-Chrome walk; STOP / RESET dry-run order and guards; the judge tables carry every message of a T4 chain; the judges' view
  shows *Ended — the last vehicle left AUTO* on a kill; `make test`; GUIDE (MISSION CONTROL row, test mode, the two buttons).

### WP10e — v0.1.36 · the CV loop closed: masks and depth → the sim observer → cv_publish → the map ✅ (2026-11-05)
- The renderer emits, beside the picture, **per-object masks and depth** (`/masks.png` with ids, `/depth.npy` or the 16-bit PNG) at the
  frame rate. A **sim observer** (`cv/sim_observer/` — ours, on the AGX's interpreter or the Mac's) runs the students' `measure_mask` /
  `buoy_geolocation` code (imported read-only, as `buoys_multi` does) and our colour vote on the rendered masks → the same
  `latest_status.json` → `cv_publish` → the mission manager's map. **fake_perception off** in `run_sim.sh` (`--perception geometric`
  brings it back for the fast runs). A mockup moved on the SIM page now moves on the map; m0t2 from gates, m0t5 (WP11), m1t1 and m0t4
  run through the camera path; the ghost filter and the colour thresholds are exercised on rendered scenes.
- `--detector synthetic | model`: `model` = the neural detector from 7b on the rendered frames (`models/<name>/` contract), its masks
  / boxes instead of the renderer's — the measured domain gap goes into the CV agent's eval.
- `run_sim.sh` also starts **usv_ui on the fake plant** (the `…sim` codes and `handover` work as on the three hosts).
- Accept: with fake_perception off, world_pool's ten buoys appear in the map with their colours from the sim observer within 10 s of
  `cv on`; a buoy dragged 5 m on the SIM page moves ~5 m on the map; m1t1 DONE through the camera path; `--detector model` runs with a
  stand-in weights file; `make test` (the sim tests skip without numpy as today); GUIDE 4.14 rows; the sim-day sheet re-run.

### WP10f — after the event · the BOAT window (GUI v2 of the boat console), designed with Dmitrii first
- A second page, its own window, served beside the boat broker (`boat-web`, port 8098 — on the NX by `run_usv.sh`, or on the Mac
  pointed at `BOAT_HOST`'s websocket): **the boat as the pilot sees it** — Dmitrii's list: heartbeat (FCU HB age, sysid / compid, mode,
  armed, kill / RC5 / RC6, EKF, GPS fix / sats / hdop, battery when available), a **radar** (heading-up PPI of the perception map:
  detections by colour with range / bearing rings, the gates, the dock line, the course edge), the **map** (north-up: track, points,
  path, carrot, HOME, boundary), a **compass** (heading, COG, the commanded yaw) and **speedometers** (SOG, the commanded surge / sway),
  the **mission indicators and buttons** (`handover` / `takeover` / `stop`, mm state V and task / stage, OCS link and bridge, AGX
  perception age, camera, `log on/off`, `note`, `save`, `cv on/off`), and tabs CONSOLE (usv_ui's console mirrored), LOG, MAVLINK (message
  rates, last HEARTBEAT / SYS_STATUS / RC_CHANNELS), NETWORK.
- The page talks to usv_ui over the boat broker (`local/usv/ui/cmd`, a **whitelist**: handover, takeover, stop, log, note, save, co, cv,
  points — no bursts, no `arm` / `disarm`, no mode changes: by-hand motion and arming stay on the boat console and the transmitter,
  decision 25). usv_ui publishes what the page draws (it already publishes the map and `tui/active`; add its FCU / link block at 2 Hz).
- **Design first**: a static wireframe (HTML mock with fake data) in its own version step, polled with Dmitrii (what is on the first
  screen, what goes to tabs), then the wiring. GUI v2 is allowed to differ from the TUI's skeleton; the TUI keeps decision 25's role.
- Accept: the wireframe approved; the live page on the bench (`run_sim.sh`, usv_ui on the fake plant) and on the NX with the fake FCU;
  `handover` from the page → the mission manager takes the FCU, `takeover` back within 2 s; the whitelist refuses anything else with a
  reason; the radar shows world_pool's buoys at the right range / bearing; `make test`; GUIDE §3.2 row.

Superseded by §7c (2026-11-05 evening) for the order. Then (renumbered +1 on 2026-11-05 for WP10g = v0.1.34) WP9c (declaration checks), WP9b (the recorder), WP9d-2 (presets, flavour line, ten words, refusal sweep — STOP EVERYTHING and the
page-first rule are done in WP10d), WP11–WP16 as below; versions assigned in that order.

## 7c. Re-plan 2026-11-05 evening — simulation first, no more water before the event (Dmitrii, polled)

**What changed**: no pool day before the event and no boat motion on the lab day (switches, connections and some real-world CV only
— `docs/LAB_DAY_2026-11-06.md` rewritten, v0.1.34.1); a few bench days remain. **The priority is to finish the simulation** so the main
mission profile is proven there, and to **collect real data at the event** with every pipeline known and rehearsed in advance. The
profile set is cut to three so it is easy to navigate.

**The order from v0.1.35** (one version each, as before):

| version | WP | what | why now |
|---|---|---|---|
| 0.1.35 | WP10c | map interaction: add object, hover coordinates, tooltips, the mini-map on MAIN | kept early (Dmitrii): it speeds up every sim session |
| 0.1.36 | WP10e | the CV loop closed: the sim observer on the rendered frames → cv_publish → the map; usv_ui on the simulated boat; `--depth-noise zed` in the loop | the mission profile must run through the camera path, as on the boat |
| 0.1.37 | WP10d | the operator layer: TEST MODE, MISSION CONTROL, STOP / RESET EVERYTHING, the judge tables, the judges' view | the event's operator flow, rehearsed in the sim |
| 0.1.38 | **the launcher** (new, Dmitrii 2026-11-05) | ✅ **one script, one settings page**: `scripts/launch.sh` → http://127.0.0.1:8094/ — presets simulation / lab / marina / custom (`profiles/presets.yaml`), every field changeable (sim or real, the judge fake / rc-test / course, the network ts / lan / typed, the FCU, the CV real / fake / off, the boat's profile, heartbeat / status / map rates, logging), the plan per host shown, LAUNCH over ssh in order, STOP | one entry point for every session |
| 0.1.39 | **WP10h** (new) | ✅ **the mission profile campaign** (240 / 240, `docs/CAMPAIGN.md`): m1t1 / m3t1 / m4t1 / m0t4 swept over seeds, both planners and the real camera (51°), with the noisy depth; the search driving toward rays (pathing spec 335 "pending", with the S6 tracker port of §4a); every failure fixed or written down as a known limit; the sim-day sheet as the regression | "ensure the main mission profile" |
| 0.1.40 | WP9b ✅ (Mac; the AGX clip after the deploy) | the ZED recorder (`--record`, `--svo` replay) + **the event's data pipeline rehearsed on the bench**: record → bring home → CV_TRAIN's import, a GUIDE checklist for the event days | collect at the event, pipelines known in advance |
| 0.1.41 | WP9c ✅ | RunDeclaration checks | the judged runs at the event |
| 0.1.42 | WP9d ✅ (the light cut) | **the profile cut** — `profiles/sim/` (the laptop simulator: usv_sim, ocs_sim + the worlds marina, pool, boxes, empty), `profiles/lab/` (usv_lab: the real Pixhawk on the bench), `profiles/event/` (usv_event + ocs_event, `--judge fake | rctest | course` for practice and the runs; usv_pool / ocs_pool / usv_course / ocs_course folded in); world_bench and world_por move to `tests/fixtures/`; the launchers, GUIDE, tests follow — + the presets (sim / lab / event: one word each) and the rest of WP9d | easier to navigate |
| after the event | WP10f | the BOAT window | not needed to win the event |

WP11 (CV gate), WP12 (cannon), WP13 (m0t4 → m3t1), WP14 (UAV adapter), WP15 (UAV launcher) take v0.1.47 → v0.1.51 after the packages of §7e (renumbered 2026-11-06 evening) —
reorder them when the event's schedule and the drone team's needs are known (WP14 owns the one declaration if the drone runs with us).
**WP9's mode-20 walk** on the real Pixhawk is reduced to an optional 15-minute dry check on the lab day (props off, modes only); the
first mode-20 stage on water will then be at the event's practice - a known risk, flagged in the lab-day page.

## 7d. Notes for the next planning session (Dmitrii, 2026-11-05 night) — to discuss with the main agent, not built yet

1. **Re-evaluate the mission start sequence.** Today: hand over → `enable_auto` (AUTO_HOLD) → `declare` (RunDeclaration: we are
   ready for these tasks) → wait for RoboCommand's RunStart → the run (R5) → start the mission. Questions: is this order right on the
   course; the moment RunStart arrives must be **a clear button / indication** on the page (today it is a state word and a timeline
   row); what the operator does at each step, one obvious control per step.
2. **The overall UI** — look at how the page (and usv_ui / the judge page) can be improved as a whole, after the sequence is settled.
   (WP10f, the BOAT window, already waits for after the event; this is the wider look.)

3. **T4 is a modifier, not a task we declare (Dmitrii, 2026-11-06).** T4 (Dynamic Incident Response) can interrupt any mission: it
   is not a separate declaration choice but a **flag on whatever mission runs** ("this run takes T4: yes / no"), with the proper
   acknowledgement and progress through its steps (AssistanceRequest → ack → transit → readiness → confirm → back to the mission).
   Consequences to work out: the declaration's T4 tier comes from that flag; WP9c's `mandatory_none` check (T4 at TIER_NONE refused on
   the course) is revised to match; m4t1 stays as the standalone test of the chain; the page shows the T4 flag beside the mission.
4. **A visible 3..2..1 countdown when RoboCommand answers (Dmitrii, 2026-11-06).** On RunStart (the judge's response to our
   declaration), the page shows a large 3 · 2 · 1 before the mission starts (the `--auto` start), instead of jumping straight in - the
   operator sees it coming (and can still hold / abort during it). Pairs with note 1's "RunStart received" indication.

## 7e. The UI programme after the pre-renders (Dmitrii with Claude Cowork, 2026-11-06) — §7d settled into work packages

Decided on the pre-renders (`docs/mock/`, the canvas "OCS v2 pre-renders") and two specs that replace the reviewer's handover
documents: **`docs/OCS_V2_SPEC.md`** (the operator page as the system-level console) and **`docs/BOAT_WINDOW_SPEC.md`** (WP10f, the
boat window with the five-system wireframe). The principles behind both: the project doc `claude/ui_principles_control_station.md`
(ISA-101 levels, ISA-18.2 / IMO BAM alert ladder, Endsley, the RPAS findings) — the short form is OCS_V2_SPEC §2. Non-goals are
written down (§12 there): no second map renderer, no environment switch, no framework port, no authority requests, no arm /
thruster / cannon controls. Themes: dark · light · hc-dark · hc-light · fun, default per preset (event → hc-light).

### WP10j — v0.1.46 ✅ (2026-11-07, overnight; renumbered: the next free version - see the note below) · the Task 3 docking structure in the renderer + the dock dataset (tonight, first — CV_TRAIN waits on it)
`docs/DOCK_MOCKUP_SPEC.md`: the white cube on its PVC frame with two 25 cm black-bordered windows (1 upper-left, 2 lower-right),
the RED / GREEN indicator at the base, the light states from `sim/dock/state`; `render --dataset --scene dock | mix`, five classes
(`indicator` added — decided 2026-11-06); `docs/REPLY_dock_dataset.md` as CV_TRAIN's signal. The camera-path dock loop (WP10j-b)
is after the event. CV_TRAIN's night: `../CV_TRAIN/docs/OVERNIGHT_2026-11-06.md` (buoy v2 at 51°, then the combined `cvt-v2-l`,
≤ 6 h GPU pre-approved).

### WP10j-b — v0.1.48 ✅ (2026-11-06) · m3t1 through the camera: the docking rehearsal (its own file, after cvt-v2-l exists)
`docs/M3T1_SIM_PLAN.md` (decided 2026-11-06 evening): the dock in the perception map from the camera (indicator → bay colour,
windows with their light state), T3 gains **SWAY_SEARCH** (sway the whole line, votes per bay, then commit) and **BACK_OUT on a wrong
call** (≤ 2 re-engagements), the RED window lights on our DockingReport (fake dock `auto: false`), the aim topic carries the
window id (no sender, no fake Pi), Ultralytics into the kit's venv for `--detector model` on the Mac, the campaign's m3t1 with
random starts and a strict reporting grader, the hand rehearsal with `--step`. Not in tonight's queue unless it is empty.

### WP10k — WP10k-a = v0.1.47 ✅ (2026-11-06), WP10k-b = v0.1.49 ✅, WP10k-c = v0.1.51 ✅ (2026-11-06; UI_QOL §F, the visual pass) · the quality-of-life round on the pages (Dmitrii's 23 items, 2026-11-07 morning)
`docs/UI_QOL_2026-11-07.md`. **WP10k-a** (same day, usable in the lab): the launcher (copy icons on commands, tooltips on every option,
network placeholders from the env files, PROGRESS shows what is up in bold `--up`, five page links with up / down dots incl. the rc-test
console, the theme chosen here and persisted on every page in one fixed place), the judge page (light themes, three columns, the judge
map for T4 positions and the geofence, liveness dots that flash per message), COMMS & DIAGNOSTICS (dense two-lane layout, aligned
node tables, SYSTEM ACTIONS first with per-host cards and the launched settings preselected), OVERVIEW (RUN SEQUENCE with history
buttons and the `--action` token, the picker + stage list back on OVERVIEW), no layout jumps, the theme screenshot set, the SIM page
camera bug. **WP10k-b**: the USV window phase 3 pulled forward (typography, the five tabs with content, radar / compass / speed dials,
the whitelist controls, the BlueBoat-proportioned wireframe with boxed callouts and every perception source, the TUNING card with
`TUNE` → `SET_TUNING` and a saved profile copy, the CV-model hint per task). Order after WP10j-b unless Dmitrii says otherwise.

### WP10i-lite — v0.1.43 ✅ (2026-11-07, overnight) · before the event (one version; the small, operator-facing half of §7d) — tonight, second
- **First, the v1 freeze**: `ocs_web.html` → `ocs_web_v1.html` served at `/v1/` by the same server, the SETTINGS "classic page" switch,
  the launcher's `ui: v1 | v2`, `run_ocs.sh --ui v1`; the v1 page is never edited again (one banner line).
- The **top bar on every page** with MISSION CONTROL's state word + why, the FCU holder · V · mode · armed · kill, the R chip, the
  alarm count, STOP ALL / RESET ALL, the theme switch (the OCS page, the judge page; usv_ui's bar unchanged).
- **RunStart as a slot** in the run ladder on MAIN: seconds waited, seconds to the stall alert, *arrives once*; on arrival a
  **3 · 2 · 1** on the page (one warning sound) before the `--auto` start, with HOLD-on-start and cancel-auto-start live (§7d.1, §7d.4).
- **The stages as a list** in MISSION CONTROL (name · what it does · exit condition · elapsed vs typical from `docs/CAMPAIGN.md` ·
  sub-progress; LOOK / APPROACH_LEAD as indented detours) from the playbook's stage tables, and a stage strip on MAIN's mission row.
- **The theme pack** as CSS variables (the five palettes of `docs/mock/README.md`), `ocs.theme`, the per-preset default; the judge
  page and the maps follow.
- **Two backend safety items** (OCS_V2_SPEC §6): intents without an ack in `ocs.ack_timeout_s` (5 s) → TIMED OUT + WARNING; the boat
  refuses a cmd older than `guards.cmd_max_age_s` (10 s) with ack REJECTED `stale` (the bridge replays queued QoS-1 commands after a
  link loss) — tested with a START_TASK published while the bridge is down.
- **The host-health beat** (OCS_V2_SPEC §11): NX and AGX publish CPU / GPU load, thermal zones, throttling, disk free at 1 Hz
  (`local/usv/host/<host>`, bridged as `telemetry/usv/host/<host>`); rows on MAIN and NETWORK; `host.warn_c` / `host.alarm_c`.
- Accept: the sim-day sheet 36 / 36; the **fault-injection timing test** (OCS_V2_SPEC §13: eight injected faults each named on MAIN
  within 3 s with the first action readable); no button without a title; every theme passes the contrast rule; `make test`; GUIDE rows.

### WP10i — v0.1.44 ✅ (2026-11-07, overnight) · OCS v2, the regrouping (tonight, unattended — decided 2026-11-06 evening)
The regrouping into OVERVIEW · MISSION · USV CONTROL ↗ · UAV · COMMS & DIAGNOSTICS · JUDGE · ⚙ SETTINGS (OCS_V2_SPEC §3), the
six-question cards (§4), the alarm model with the list, acknowledgement and history (§8 — the table and thresholds accepted; sounds
behind a SETTINGS slider, off by default), COMMS & DIAGNOSTICS, the UAV card (LOI-2), the mission picker on MISSION only. **The v1
page stays at `/v1/`** (same server; the SETTINGS switch; the launcher's `ui` field) until a version after the event removes it.
Phases, the pre-answered decisions and the acceptance (the sim-day sheet 36 / 36 on both pages) in the spec §12a–§13.

### WP10f — the BOAT window, revised · phases 1–2 as v0.1.45 ✅, phase 3 as WP10k-b v0.1.49 ✅ · **the rest v0.1.55 ✅ (2026-11-08)** (§7f: the 10 Hz frame topic, phase 5, the acceptance)
As `docs/BOAT_WINDOW_SPEC.md`: the five-system wireframe contract (ids `thr-mp thr-ms thr-sf thr-sa cam lamp ant box cannon`,
`setState(json)`), the data truth table (exists · to add · never: no thruster feedback by design, τ normalized, the lamp *expected*
from the state, the cannon one-way until WP12), the six tabs, usv_ui's FCU block at 2 Hz (+ an optional 10 Hz frame topic, never
bridged), the whitelist of decision 25. The wireframe skin ("StarCraft meets Teenage Engineering") is Dmitrii's separate session;
the contract keeps the two apart.

**As built overnight (2026-11-07)**: the night run started from Dmitrii's prompt (WP10i-lite v0.1.43 → WP10i v0.1.44 → WP10f 1–2
v0.1.45) before this section gained WP10j; those three were archived under those numbers, so WP10j took the next free version, v0.1.46,
and WP10j-b the one after (only if `cvt-v2-l` exists). WP11–WP15 keep the shift below by one more. **Numbering (2026-11-06 evening, superseded)**: WP10j v0.1.43 → WP10i-lite v0.1.44 → WP10i v0.1.45 → WP10f phases 1–2 v0.1.46, built overnight
in that order (stop on red, nothing deployed; a version not reached tonight keeps its number); WP11–WP15 shift to v0.1.47–v0.1.51
(reorder with the event's schedule as §7c says); WP10f's phases 3–5 and WP10j-b take the next free versions after the event. §7d's four notes are covered: 1 and 4 by WP10i-lite, 2 by WP10i, 3 (T4 as a
modifier) by WP10i-lite's picker flag + the WP9c row change described in OCS_V2_SPEC §5.

## 7f. Re-plan 2026-11-07 (Dmitrii) — what stays after v0.1.54

Dmitrii: "drop WP9 mode-20 walk, WP11. WP12 and WP13 we test in simulation. WP14-16 and WP6 we keep. WP10f - lets bring them up."

- **WP9's mode-20 walk: dropped** (the rest of WP9 - the lab day's switches, connections, real CV - stays a field checklist).
- **WP11 (the CV gate m0t5): dropped.**
- **WP12 (the cannon contract + link) and WP13 (m0t4 → m3t1): built and accepted in the simulator** (the fake cannon / sim dock, the
  campaign and sim-day checks), not on hardware; WP12a's hardware day no longer gates them.
- **WP14, WP15, WP16, WP6: kept** as written (§8), in that order after the WPs below.
- **WP10f's remaining phases: next.** Phases 1–2 = v0.1.45, phase 3 (the tabs, dials, controls, tuning) = WP10k-b v0.1.49; what is
  left of `docs/BOAT_WINDOW_SPEC.md` §5: the **10 Hz frame topic** (the mission manager's commanded τ / surge / sway / yaw - today
  "not published" on CONTROL, the compass's SET, the speed dial's commanded tick), **phase 5** (the host-health beat on the bench /
  in the sim so NX / AGX are not "no beat"; the lamp read-back question closed), and the spec's acceptance not yet run (the fault-
  injection test with the boat window open; the page's CPU on the NX - measured on the Mac until a deploy).

**Order from here**: WP10f-rest (next free version) → WP12 (sim) → WP13 (sim) → WP14 → WP15 → WP16 / WP6 (when the students deliver).

**Status after the night of 2026-11-07/08** (docs/REPORT_2026-11-08.md; nothing deployed - the Jetsons stay on v0.1.29.1):
- v0.1.54.1 ✅ the EKF bit (0x100 = PRED_POS_HORIZ_REL; ABS = 0x200) - `make test` 530.
- **WP10f-rest = v0.1.55 ✅** the commanded side (`mm/state cmd`: SET, the speed tick, CONTROL, ACTUATION), the simulated host beats, the
  lamp hook; fault test 8 / 8 with the boat window open, sim-day 46 / 46, no-jump 0 / 0 / 0, page CPU on the Mac ≈ 14 % of one core
  (the NX figure after a deploy) - `make test` 537.
- **m3t1 hardening = v0.1.56 ✅** (M3T1_SIM_PLAN continued): standoff lines from the dock geometry (5.70 / 5.00 m; the check-in's real
  finger length → `t3.finger_m`), every stage / failure in the fast SIL, the campaign 81 / 84, the votes in the map hover; live S9 4 / 4
  with cvt-v2-l + rc-test, the `--step` walk 11 / 11 - `make test` 555.
- **WP12 = v0.1.57 ✅ in the simulator** (the aim topic → the sender → the fake Pi, HMAC with a generated test key, `window_id`, the status
  on the boat window and the OCS SYSTEM card) - `make test` 564. **Open for the AGX day**: snapshot the students' color sender / receiver,
  the real receiver's acceptance of `window_id` / `target: null`.
- **WP13 = v0.1.58 ✅ in the simulator** + the box key points (Dmitrii's request of the night): `t3.dock_source: boxes`, the line from ≥ 2
  faces' key points = the sway path; cvt-v2-l dock faces 40 / 40 within 0.3 m / 5° on rendered worlds, no training needed - `make test` 576.
- Next: WP14 (the UAV adapter) when the colleague's Orin is on the network; the deploy + bench rehearsal (LAB_DAY block B) when Dmitrii says so.
- 2026-11-07 (day): v0.1.58 deployed to both Jetsons (JETSON_LOG). **v0.1.59 ✅ the run-start sequence + the RoboCommand compliance fixes**
  (`docs/RUNSTART_AND_RC_BRIEF_2026-11-08.md`): the counters never reset / persisted, EKF_BAD soft 3 s, the FAULT heartbeat grace, the
  monitors' namespace, RC_HOST auto + the clock check, heartbeats from the first status on the judge wire, RUN SEQUENCE in the handbook's
  order; live against rc-test with manual RunStart (report seq continuous across RunStart), sim-day 46 / 46, fault test 8 / 8, no-jump
  0 / 0 / 0. Not deployed. Next: merge the cannon agent's work; **WP10m - the OCS CAMERA tab** (below, poll first); WP14 when the Orin is up.
- **WP10m = v0.1.60 ✅** the OCS CAMERA tab (RGB / DEPTH / both, the box classes, the frame recorder to the allowed roots); live in the
  simulator (depth, the range under the cursor, a 2 Hz recording with the boxes per frame, a snapshot), no-jump 0 / 0. Not deployed; the
  real ZED's depth on the AGX day. Next: merge the cannon agent's work; WP14 when the Orin is up.
- **v0.1.63 ✅** the USV window on the team LAN for a delegate (nx | mac in the launcher, event nx; one commander, take over; PIN) - live on
  this Mac's LAN address: PIN refusal, take, viewer refusal, take over, the OCS row; the NX route (OCS lines through the hub) against the
  sim's hub. **UAV later**: the same window_control + launcher pattern for the UAV's own window on its laptop (WP15).
- **v0.1.62 ✅** (Dmitrii's round 2026-11-07): MISSION preselect with its answer + copy; the boat window's plate clickable (two clicks);
  m3t1's dock key points on the camera and the sway path on the map (live with cvt-v2-l: 4 dock faces with key points).
- **WP12-rest = v0.1.61 ✅ in the simulator** (2026-11-07, prepared in parallel by the cannon session, merged): the students' sender /
  receiver snapshotted (`cv/cannon/`, `cv/cannon/pi/`); RED, one camera owner, a status publisher beside their receiver (polled). Open for
  the AGX day: their two configs, the Pi's route to the boat broker, the real window's HSV band, the clocks.

## 7b. CV_TRAIN — the detector, a third agent in its own folder (decided 2026-11-05)

A separate folder beside the kit, `~/Documents/BH1/RobotX/CV_TRAIN/` on the Mac (its own `CLAUDE.md`, from `docs/CV_TRAIN_BRIEF.md`),
mirrored to the AGX at `~/Desktop/dm3/CV_TRAIN/` where **training and export run on the AGX's GPU** (its own venv with
`--system-site-packages` for JetPack's torch; the students' folders and venv stay read-only; ~26 GB free — clips and datasets sized
to it). Inputs: the students' dataset and `best.pt` (inventory first), the renderer's dataset export (WP10b), the pool / venue clips
once the recorder exists (WP9b). Outputs: `models/<name>/` = weights (.pt, .onnx, the .engine built on this AGX), `classes.yaml`,
input size, sha256, `PROVENANCE.md`, an eval sheet (mAP per class on real / synthetic / mixed, and the field metric: range and bearing
error against the sim's truth). The kit consumes it through `cv/buoys_multi --model` and WP10e's `--detector model`; the primary agent
integrates, the CV agent never edits MASTER_SCRIPTS. Open question for the CV agent to settle with Dmitrii first: detector classes
= {buoy, dock_face, window, box} with the colour / lit state from our vote (robust, little data) **or** colour classes in the model.

### WP11 — ~~v0.1.47 · CV gate: two unlit buoys as a pretend-coloured gate~~ — DROPPED 2026-11-07 (Dmitrii, §7f)
- Precondition, by hand (no mission): one real buoy detected, localised and mapped steadily — `cv on` / `cv status` in usv_ui
  and map.html. Then a second buoy is put in the water.
- A test mission (code m0t5, task TEST_CV_GATE): the two unlit buoys from the PerceptionMap, paired by the gate rule (1.5–6 m
  apart); when the pair is first formed the buoy on the boat's **left as it approaches = GREEN, right = RED** (the handbook's
  green-to-port), locked at first sight like S6; then the normal gate path pre → mid → post (simple / s6 planner), HOLD, DONE.
  `--step` / `--from` like the others.
- Sim: a world with two unlit buoys (`world_cvgate.yaml`), fake_perception unchanged.
- Accept: fast SIL m0t5 DONE from two approach sides (the colours swap with the side); the colours never flip after the lock;
  a single buoy → the mission holds and says so; live m0t5sim on the three hosts; GUIDE row; tests per behaviour.

### WP12 — **v0.1.57 ✅ (2026-11-08, in the simulator)** · the rest v0.1.61 ✅ (their receiver is the contract) · the cannon contract and link — **built and tested in the simulator** (§7f)
- Settle: the aim target is the RED window (GREEN = the hit, which m3t1 already waits for) — one sender with `--color`; the
  sender sends `target: null` unless `local/usv/cannon/aim {on, bay}` is fresh (the mission manager's AIM_FIRE), or the receiver
  requires it; `z_up` offset owned by one side; the dock observer stays the map's source, the colour sender is cannon-only;
  `window_id` upper / lower from the hit window's `z_up_m`.
- Then: snapshot `color_dock_sender.py` → `cv/cannon/` and `color_cannon_receiver.py` → `cv/cannon/pi/` (reference) with
  `PROVENANCE.md`; the aim gate; `cv/cannon/run.sh` for the new sender; `cannon_sender.py` (`dock_cannon.v1`) marked legacy.
- Accept: dry run end to end (m3t1sim on the AGX → the real sender in `--synthetic-send` / dry mode → the receiver's
  `DRY_RUN_READY_10S` on the Pi); `cv/cannon/README.md` = the contract; GUIDE 4.16b.

### WP12a — the hardware day (a GUIDE checklist, no version unless something breaks)
LAN (`--lan`) run of the three hosts incl. the stream and rc-tui; the ZED source of `cv/stream`; the cannon dry run of WP12;
the WP9 walk repeated on the boat's own NX with the boat's Pixhawk; every line ticked with a date before the pool.

### WP13 — **v0.1.58 ✅ (2026-11-08, in the simulator, + the box key points)** · m0t4 → m3t1 — **built and tested in the simulator** (§7f; was: after WP8 and WP9 have run)
Their `BoxTrack.stable()` and face endpoints as a second dock source behind `usv/dockpath.dock_geometry()`
(`t3.dock_source: map | boxes`); m3t1's stages unchanged above it; `SELECT_TARGET` as the optional override of the GREEN rule
(`--bay` today); their task wrapper's own mode-20 approach retired. Accept: fast SIL m3t1 on a world whose faces publish
`face_points`; live m3t1sim with the box source; the map source still passes; the students agree the AGX detector emits
`face_points`.

### The pool legs (GUIDE §7.3 F-4a → F-4c), with the WP9 checklist in hand
m0t2 from `pathgen`'s points → m1t1 (`--planner simple` first) → m0t3 between two saved points beside usv_ui's `swayto` on the
same points (compare the tracks) → m3t1 with the dock in view and the cannon in dry run. WP6 (UAV, m1t2) unchanged: after the
drone stack. Parked items stay in §4a (+ team id from RoboNation, ACLs on the hub, the re-declaration seq).

### WP10m — **v0.1.60 ✅ (2026-11-07)** · the OCS CAMERA tab (Dmitrii 2026-11-07), vehicle-agnostic so the UAV slots in

**Decided 2026-11-07 (the poll)**: depth only when no detector holds the ZED (greyed with the reason otherwise); JPEG frames + jsonl
(no SVO2 / MP4 from the tab - the ZED recorder stays in SETTINGS); the roots AGX recordings, Mac logs, CV_TRAIN staging (the Mac
staging folder `~/Documents/BH1/RobotX/recordings` CV_TRAIN imports from - never inside CV_TRAIN); its own tab. Built as below,
except: the AGX's depth preview from a running observer is not built (by the decision); keys space / R as planned. Open: the depth
on the real ZED (the reopen with DEPTH_MODE.PERFORMANCE) is untested until the AGX day; the UAV half waits for WP14 / WP15.


**Why**: today the camera is a card inside OVERVIEW / MISSION (the HUD, the boxes) and the recorder is a SETTINGS card; there is no
place to look only at what the camera sees, change what it shows, and record training clips where we want them.

**What exists to build on**: `cv/stream/cam_stream.py` on the AGX (sources dir / zed / test; `/stream`, `/snapshot.jpg`, `/status`;
ZED opened with `DEPTH_MODE.NONE`), `scripts/agx_cam.sh`, the observer boxes `perception/frame_dets` → the page's `/dets.json`
(v0.1.54), `cv_hint.by_task` (which classes a mission consumes, v0.1.49), the recorder `scripts/agx_record.sh` (SVO2 + frames.jsonl +
session.json, `AGX_REC_DIR`, v0.1.40) and `agx_pull_clips.sh`; in the sim the rendered camera (simview `snapshot.jpg`).

**Build (one version)**:
1. **A CAMERA tab** on the OCS page (after MISSION): the picture full width at the top, nothing else competing; a **vehicle selector
   USV · UAV** (UAV greyed "not connected" until WP14/15) - every control below is per vehicle, the cameras come from the profile
   (`cameras: {usv: {stream, status, dets, record}, uav: …}`), never hard-coded to the AGX.
2. **Modes**: RGB · DEPTH (colourised, the scale bar in metres, the range under the cursor) · RGB | DEPTH side by side · (later)
   confidence. Switched at run time on the stream (`POST /mode` on cam_stream; zed source with depth on when a depth mode is asked).
   **The one-owner rule stays**: when an observer holds the ZED, depth comes from the observer's own preview files (dir source:
   the observer writes `preview_depth.jpg` beside its RGB preview) - the tab says which.
3. **Boxes**: the overlay from `/dets.json` with a toggle per class (buoy · dock_face · window · indicator · box · cannon target),
   a **mission preset** (ticks the classes the picked mission consumes, from `cv_hint.by_task`), labels on / off (colour · range ·
   light), the frame / detection age always shown; nothing drawn when the boxes are stale (as v0.1.54).
4. **Record for training**: label, **directory** (a pick-list of allowed roots on the camera host: `AGX_REC_DIR`, the CV_TRAIN staging
   folder, a dated subfolder - typed paths refused outside them), format **SVO2** (full, with depth; through the recorder = the
   observer with `--record`) or **frames** (JPEG at N Hz + frames.jsonl: from the stream, works with any source incl. the sim),
   max minutes, start / stop, status (frames, MB, disk free, `disk_low`), the session list with *pull to the Mac* (agx_pull_clips.sh).
   Recording in the sim writes the rendered frames + the truth labels (a synthetic dataset for CV_TRAIN).
5. Keyboard: space = snapshot (saved with the label), R = start / stop recording (asks before stop).

**Acceptance**: tests for the profile's camera list, the mode switch on cam_stream (test source), the class filter and mission preset,
the directory allow-list (a path outside refused), frames recording in the sim (N frames + jsonl in the picked folder); live: the sim
camera in the tab, depth from the ZED on the AGX day, a 1-min SVO2 clip pulled to the Mac; no-jump on the tab; screenshots.
**Poll Dmitrii first** (AskUserQuestion): depth while an observer runs (observer previews vs pausing it), the record formats, the
allowed directory roots, the tab's place (own tab vs MISSION sub-tab).

**UAV (with WP14/WP15)**: the same tab, vehicle = UAV: the colleague's Orin publishes the same contract (an HTTP stream + `/status`
+ the detections in the `frame_dets` shape + a recorder with the same start / stop / status calls); the RoboCommand-side rules do not
change (the camera never touches the judge wire). A UAV-only mode later: the down camera with the course overlay.

## 8. UAV + RoboCommand integration (decided 2026-11-03 after inspecting `robotx-uav-robocommand/`)

The UAV team's field-tested RoboCommand stack arrived on 2026-11-03 (`robotx-uav-robocommand/`, snapshot in `_archive/`,
`MERGE_GUIDE.md` + `SGIT_UAV_RoboCommand_Integration.pdf`; the inspection and the diff against our stack are in the project doc
`claude/uav_robocommand_diff_2026-11-03.md`). Same concept as ours — only the OCS talks to RoboNation, the protobuf is packed
on the laptop, the vehicle prepares the content — and it carries the geofence loop we need: **the declared UAV geofence is the
polygon the FCU actually holds and the ceiling is `FENCE_ALT_MAX`; a Mission Planner edit is detected on the Orin and re-declared,
never overwritten by the course**. Decisions polled 2026-11-03: (1) **port his bridge into `ocs_app`** (one MQTT client, one
declaration owner; his guide's option B) — his Orin listener stays untouched and keeps its JSON/UDP wire (MERGE_GUIDE §5);
(2) **the UAV Orin is assumed to have an IP route to the OCS laptop** (Bullet antenna direct to ground, two simultaneous links,
to be explored; then UAV ↔ OCS cross-subscribe directly and USV ↔ UAV for buoy coordinates) — this revisits decision 26 (UAV via
the USV radio): amend the ADR register / frame 21 when the antenna test is done; (3) **nothing on the USV for the fence** for now
(it keeps the course it already receives in PREPARE); (4) **his UAV runtime as is** (GUI + listener + MAVProxy + the RunStart
flight) under a `run_uav.sh` wrapper, harmonised with `optiprimex/uav/` later. His compliance invariants (MERGE_GUIDE §4) become
ours in the port; the "odd" constants are field lessons (§7 there) and are kept verbatim: the 1e-6° fence compare, the 10 s
re-read while disarmed, the course taken once, MAVLink2 for the fence clear, SIGTERM for the parameter restore.

Priority: after WP7 (validation) and WP9 (stage B), before WP13 — the declaration path is on the critical path of every
run with the drone, and the orientation / PoR comms needs both vehicles declared.

### WP14 — ✅ v0.1.64 (2026-11-08, overnight) · the UAV adapter in the OCS (one declaration owner) - built to docs/OVERNIGHT_2026-11-07_UAV.md D0-D14
- `optiprimex/ocs/uav_adapter.py`: his §5 wire — `robocommand_hello` every 2 s, reliable `robocommand {kind: course | command}`
  forwards (id + ack, 0.5 s resend, 15 s give-up), `uav_telemetry` in (1 Hz: heartbeat fields, `fcu_fence`, `fence_alt_max`,
  optional `fence_change`), `fence_change_result` / `fence_rejected` out, `uav_report` in (validated against the schema, acked
  `uav_report_queued` → `uav_report_ack`), `bridge_notice` out.
- `optiprimex/ocs/geofence.py`: `geofence_problems()` ported verbatim from `run_declaration.py` (closed ring ≥ 4 points, every point
  and edge sampled at 0.5 m within course ∪ UAV transit area to 1 m, 0 < ceiling ≤ 50 m HAE) and `course_fence()` (corners +
  transit area merged into one inclusion polygon) for the display and the forward.
- `ocs_app` declaration: the UAV geofence and ceiling come **from the telemetry** (the profile's `uav_geofence` is only the bench
  stand-in when no UAV is declared); refuse to send (and say why in `ocs/state` + an alert) when the telemetry is stale (5 s), the
  drone is armed, no course was received, the FCU has no polygon, or the rule fails — then send `fence_rejected` so the listener
  clears the fence and restores the startup params; course forwarded **once** per bridge life; on `fence_change` or a
  `FENCE_ALT_MAX` change re-send the **last** declaration (same vehicles and tiers) with the new polygon, after landing if armed;
  `last_run_declaration.json` + the seq counters persisted (the restart question, RoboNation A3).
- Heartbeats: the UAV from the telemetry at the handbook's 2 Hz with his 5 s staleness cut (stop, never a stale state) — and
  **loud**: a UAV heartbeat gap of more than a few seconds is a Remote-ID event for Garuda / CAAS and the flight manager pulls the
  plug (RoboNation, 2026-11-04, A8), so op-tui alarms on any gap > 2 s and the pilot is told at once;
  `STATE_AUTO` in AUTO or GUIDED; `altitude_hae_m` as his `uav_telemetry` computes it; `uav_registration` in every UAV heartbeat.
- Commands: UAV-targeted `assistance_request` / `keep_out_zone` / `all_clear` auto-acked (`IncidentAck`) and forwarded; USV-targeted
  ones unchanged (m4t1); RunStart forwarded to the Orin (his GUI flies `runstart_mission.py`) and verified by us as today.
- Profiles: `uav:` block in `ocs_pool.yaml` / `ocs_course.yaml` / `ocs_bench.yaml` (vehicle id `SGIT-53577`, registration
  `AA53577`, pilot call sign, listener host / port, `declared: true | false`); addresses in `network*.env` (`UAV_HOST`,
  `UAV_LISTEN_PORT=14600`). op-tui NETWORK / JUDGE: the UAV link age, the FCU fence (points, ceiling), the last declaration and
  why the next one would be refused; `rc-tui` unchanged.
- Accept: his five `sim-tests` scenarios re-run against `ocs_app` in the bridge's place (fake copter + listener unchanged):
  `mpfence_alt20`, `mpalt_alt20`, `mpfence` (rejected > 50 m, fence cleared, nothing published), `mpsilent`, `mpsilentfix` — all
  pass; `MANIFEST.sha256` of the Orin-side files unchanged; the T4 chain with USV + UAV declared against `run_rctest.sh`
  (RoboNation's server sees both vehicles, RunStart after both STATE_AUTO); 160+ tests; GUIDE 4.19; DEVLOG.

### WP15 — the launcher field + the UAV window, ✅ v0.1.65 (2026-11-08); the Orin wrapper dropped - their stack runs as field-tested · (was: the UAV launcher, the snapshot, the network)
- `uav/` in the tree = a `cv/`-style snapshot of his `geofence-tools/`, `drone-heartbeat-gui/`, `lawnmower-mission/` with
  `PROVENANCE.md` and his `MANIFEST.sha256` (read-only, his code; `camera-capture-gui/` listed, not run by us).
- `scripts/run_uav.sh` / `stop_uav.sh` on the UAV's Orin: `heartbeat_monitor.py` once (it owns MAVProxy with the outs
  14550 / 14551 / 14552 / 14553 + Mission Planner on the laptop, restarts the listener, starts the RunStart flight), with
  `--vehicle-id SGIT-53577` on the listener (his known issue), a single-instance guard (14600 / 14650), SIGTERM stop, `--no-runstart-mission`
  for the bench; `setup_uav.sh` (pymavlink only; no protobuf, no MQTT on the drone).
- Accept: the sim loop end to end on three computers (fake copter on the UAV Orin, our OCS on the Mac, the USV stack on the
  NX): course → fence → declaration with both vehicles → an MP edit → re-declaration; the Bullet-antenna test recorded (two
  simultaneous links or not) and decision 26 amended on the board; GUIDE §3.4 "UAV" start / stop rows; DEVLOG.

### WP16 — ✅ v0.1.66 (2026-11-08) · UAV ↔ USV buoy map, m1t2 (the fusion of D22, WAIT_UAV_MAP) - left for after the event: `uav.assoc_unique_m`, UAV items for other kinds than buoys
The UAV's buoy positions into the USV's `PerceptionMap` over the direct link (`UavT1Report` per ARCHITECTURE §10.4, or their
detector's output), then m1t2 (Task 1 advanced). After WP15 and the students' drone CV. WP6 (`optiprimex/uav/`, the UAV mission
manager) replaces his GUI's RunStart path only when it exists.

Open with RoboNation (questions doc A13–A15): 1 Hz vs 2 Hz heartbeats (his was accepted at ~1 Hz on site; the handbook says
2 Hz), the vehicle-id convention (`SGIT-53577` carries the CAAS number), re-declaration expectations. Open with the colleague:
the `declare_result` texts his tests expect, persistence of `last_run_declaration.json` across an OCS restart, anything else on
the Orin that sends to 14600.


## 9. The venue queue (decided 2026-11-08, Dmitrii) — the Test Guide's prompts as work packages, in this order

`docs/TEST_GUIDE_VENUE_2026-11-08.md` §7 is the source of each WP's text; the order is the queue (one version each, the first not done first).

### WP17 — FREE RUN: the full stage sequence on the bench with the real Pixhawk (G0, queued first) — ✅ v0.1.68 (2026-11-08, Mac; the bench acceptance H2b open)
The brief: `docs/PROMPT_FREE_RUN_2026-11-08.md`, acceptance F1–F9 (`--free-run` on the lab profile only, the ghost plant, mode requests only to the
FCU, `skip`, `free_run.armed refuse | warn`, the FREE RUN banner, DECLARE / launcher refuse it on the course, mode-log, the fake FCU refuses GUIDED at
gps_fix < 3); the Mac rehearsal of its §2 + the m1t1 campaign 128 / 128 with the flag off; the bench acceptance = Test Guide H2b.

**Re-ordered 2026-11-09 (Dmitrii): the bench and the pages first - WP25-WP30 (`docs/OVERNIGHT_2026-11-09_GUI.md` N1-N6) come before WP18-WP24.**

### WP25 — the dry / wet SESSIONS (N1, v0.1.69) — ✅ v0.1.69 (2026-11-09, overnight, Mac)
Acceptance: `ocs/sessions.py` + `session start NAME [opts] | stop | result` (ocs_cli, op-tui, the page; bench / lab only, `collect` also on the course): dry buoys · dockface · windows · uavmap (broker | udp, fit to UAV items + the far-UAV caution) · cannon (--aim10 asks) · wet collect (= m0t7, DROP POINT) - each starts / stops on the simulator, its METRICS fill, its result file in `logs/ocs/sessions/`; the MISSION tab's camera fold + MISSION METRICS card; GUIDE §6 sessions rows.

### WP26 — the top bar, RUN (L1), the console drawer, the declaration constructor (N2, v0.1.70) — ✅ v0.1.70 (2026-11-09, overnight, Mac)
Acceptance: one-row bar (state word + reason, R / V chips, FCU chip, judge · hb, reporting, alarms, MANUAL · HOLD · ARM · DISARM, ■ STOP RUN, ?), the context line; RUN = OVERVIEW rebuilt (the stepper spine, MISSION card, ALARMS, mini-map, VEHICLES, LINKS); the drawer on every tab (commands · system · wire, `:`); the declaration constructor + the preview from the protobuf = what the fake judge receives (a test); the Chrome walk.

### WP27 — MISSION (L2): the picker in groups, T4 as a tier, m4t0, metrics, the map's tools (N3, v0.1.71) — ✅ v0.1.71 (2026-11-09, overnight, Mac)
Acceptance: EVENT / POOL / DRY groups, the T4 tick off · core · advanced, m4t1 → m4t0 (declared as m1t1 + T4 core; wire names unchanged), TOOLS ▾, the metrics per mission; sim_day S8 / S9 / S14 unchanged; m4t0's declaration on rc-test.

### WP28 — SYSTEM (L3/L4): one hosts table, settings ▾ / actions ▾ per host, the live action console (N4, v0.1.72) — ✅ v0.1.72 (2026-11-09, overnight, Mac)
Acceptance: every action of today reachable with the same word and guard (a test enumerates them), the outputs streamed to the drawer + `logs/ocs/system_actions/`, ⏻ / ↺ here, dry-run unchanged, the FCU reboot refused armed.

### WP29 — provenance tooltips, the ⚙ panel, the GUI switch (N5, v0.1.73) — ✅ v0.1.73 (2026-11-09, overnight, Mac; LOG · VEHICLES · JUDGE = WP31-33 not built)
Acceptance: every value / chip / header / button carries its provenance (`ocs/provenance.py`, a DOM-walk test), ⚙ as a panel (theme · sounds · numerals · labels · GUI new / classic).

### WP30 — the boat window redesign (N6, v0.1.74) — ✅ v0.1.74 (2026-11-09, overnight, Mac)
Acceptance: `/v1/` frozen first; the two-row header, OVERVIEW with the dials under the wireframe, the FCU words card (other ▾), CONTROL & ACTUATION, the MAVLINK console on DIAGNOSTICS, the classic toggle + the launcher's `classic`; the whitelist / FCU block / lock + PIN / menu words unchanged; a capture per tab.

WP31-WP33 (N5's "if time remains": LOG unified · VEHICLES · JUDGE) - built only because everything before them was green.

### WP31 — the unified LOG (N5's "if time remains", v0.1.75) — ✅ v0.1.75 (2026-11-09, overnight, Mac)
Acceptance: one list from five sources, newest first; kind chips with counts; search; a time range; ≥ 20 rows at 1440 × 900.


### WP32 — VEHICLES (N5's "if time remains", v0.1.76) — ✅ v0.1.76 (2026-11-09, overnight, Mac)
Acceptance: one card per vehicle, the same rows in the same order (the Chrome walk compares them); the tab never greyed; the UAV detail kept.

### WP33 — JUDGE (N5's "if time remains", v0.1.77) — ✅ v0.1.77 (2026-11-09, overnight, Mac)
Acceptance: reporting + RoboCommand on top, the timeline strip, the judges' table one row per vehicle, the feed full width, the rest in a fold; every older id kept.

### WP34 — the declutter pass by hand (v0.1.78) — ✅ v0.1.78 (2026-11-09, Dmitrii + Cowork, Mac)
Dmitrii 2026-11-09, after the diff of the night against the boards: "do the final rehauling, since you know how it is supposed to look". Built by the Cowork session, not the agent: the bar's width budget (one row under load), RUN SEQUENCE as the stepper of board 02 (the gate + the declaration inside the live step 2, the picker in the MISSION card, the buttons in their step), the map as the page (map.html's side column folded when embedded, the banner out of the map), SYSTEM's table first (NOW · CONNECTIONS · EVERYTHING folded), ALARMS quiet in one line, the boat window's row 1 one row + the click-box on demand + the dials on the first screen. Moves, folds and hides only - every v0.1.77 id and pinned string kept (`tests/test_v0178.py`); the patch as applied: `docs/screens/v0.1.78/patch_v0178.py`. Acceptance on the Mac: `make live`, `make test` (the Chrome walks), the U12 human look of the Test Guide.

### WP35–WP40 — the SCOUT night (inserted 2026-11-11 before WP18; docs/SCOUT_BRIEF_2026-11-11.md, docs/PROMPT_SCOUT_BUILD_2026-11-11.md)
- **WP35 mission `scout`** — ✅ v0.1.79 (2026-11-11, Mac): buoy to buoy colour-blind, declared T1 core / advanced (UAV), logs/scout_<run>.jsonl,
  the S15 world (`world_scout`: every light off, ZED 2i 4 mm at 1.0 m, 0.40 m buoys) + sim_day S15 / S16; the ground test **m0t8 TEST_DETECT** (m0t7 = the collect session);
  `tools/range_probe.py` (a 0.40 m object: ranged to 23 m, bearing-only beyond). Not deployed.
- **WP36 ZED proximity guard** — ✅ v0.1.80 (2026-11-11, Mac): cv/buoys_multi/guard_core.py in OUR observer (IMU-levelled by gravity), cv_publish
  → perception/guard, usv/guard.py between every task and the FCU (SLOW cap, STOP → brake → 2 m astern → rings → re-plan, block, safe hold, lost
  cap, the SLOW memory for low things under the frame's edge), the GUARD chip, sim/fake_guard + the sim observer's guard on rendered depth,
  S15's two obstacles; the ground test **m0t9 TEST_GUARD** (props in the air: a STOP pulses astern). Not deployed.
- **WP37 station keep HOLD / LOITER switch, the words, the FCU preset, Lua v5 table** — ✅ v0.1.81 (2026-11-11, Mac): Loiter() beside Hold(),
  `station_keep hold|loiter` (default hold until the water drift check), AUTO_HOLD + holding stages LOITER, the safe hold HOLD, the buttons, the words,
  HOLD yellow; docs/fcu/PRESET_LOITER_RC6.md + loiter_rc6.param (not applied); the Lua v5 table in the preset doc (v4 not in the tree yet). Not deployed.
- **WP38 per-vehicle declare** — partly ✅ v0.1.82 (the USV's ▶ HEARTBEAT → PILOT READY → DECLARE buttons; `declare uav on|off`); a UAV-only
  declaration (`declare usv off`) → §4a later.
- **WP39 course dropdown + network checker** — ✅ v0.1.82 (judge alpha…delta + DEMO, the RoboCommand networks card, course_checks).
- **WP40 IMU-corrected camera geometry** — not built → §4a later.

### The venue day (2026-11-11, Mac; v0.1.82 – v0.1.84.8; sessions A + B in one tree)
- v0.1.82 USV heartbeat buttons, observer from the OCS, WP39, scout yaw cap · v0.1.83 logging on by default · v0.1.83.1 the OBSERVER button fix.
- v0.1.83.2–.6 (B) RECORD for the console, the five page words /cmd refused (STATION KEEP, PILOT READY, 3-2-1 HOLD/CANCEL), `state.logging`,
  `/fast.json` lat/lon, `/map.json` · v0.1.83.3 the tk launcher (:8093).
- **WP-TK the OCS console in tkinter (session A)** — Phase 1 ✅ v0.1.84 (`docs/TK_CONSOLE.md`): ladder, tiles, toasts + the pressed button's
  colour, REC, alarms, LOG · Canvas MAP · CMD (catalogue + tooltips) · MISSION · CAMERA (RECORD + depth, where stored) · VEHICLES (IMU, compass,
  heartbeats per vehicle); ▶ UAV RUN (boat holds); START LOGGING; the launcher's OCS button opens it. K8 (HOSTS) = WP44; Phase 2 → §4a.
- v0.1.84.1–.8 (B) the launcher opens the console + the judge line, UAV roll/pitch, RECORD + depth pairs, START LOGGING + `state.clocks`,
  a sim run never touches the real AGX, ZED svo2 → mp4 on the Mac, the live feed at 960 px.

### WP41–WP47 — the night before the scored runs (2026-11-11 → 12): see §0 for the table and the prompts.

### WP48 — v0.1.92 (2026-11-12 → 13 night, session A): the entry-first ladder on the real hull — Cowork consult 2026-11-13
Plan: `docs/OVERNIGHT_2026-11-13.md` §2; the numbers and rules: `docs/CONSULT_ANSWER_2026-11-13.md` (summary, P1 a–d, the guard corridor, the 3
settings), `docs/COWORK_HANDOFF_2026-11-13.md` (decisions taken in Dmitrii's absence). Acceptance: the SEARCH_ENTRY ladder order (scan → leads ≤ 2
→ CHECK any blue within 7 m before the next scout step → SCOUT_ENTRY blue-first → budget 420 → doubtful re-check → lucky red+green only → hold);
a steady blue is never orbited (sim world C 0/20); the guard **corridor** (0 STOPs in a 2 m gate with ±0.3 m lateral noise); `circle_r_m 5` /
`circle_v 0.3` held on the `omni_usv` plant; the LED wiring observer → map with the 6 s / 12-sample run-length rule; association gate
0.8–1.6 m (12 Nov replay: entities ≈ truth); `make test` green; GUIDE §C. Later (not tonight): an IMU-levelled horizon; a learned LED-state
model once the venue gives recordings; SAHI never.
**02:45 SGT addendum (Dmitrii by message):** orbit **2 m crab** (heading held, mode-20 surge+sway, 0.25 m/s; bow-following 2 m @ 0.08 as the fallback) — not 5 m; **geofence** = the RxCourse polygon + 1 m bounds every target (alien buoys never); the ladder's **micro-stages** LOOK_LOCK → APPROACH_SUSPECT → CHECK_BLUE → ALIGN_ORBIT, each shown in the GUI with its reason; a mapped buoy stops the boat only on a collision course (≤ 1 m ahead, ≤ 0.3 m lateral). Rehearsal: `pathing_sim/m1t1_logic_check.html` (0 wrong entries in 120 runs; the open spots: stops in 2 m gates, an unknown object on the gate line).
**03:55 SGT:** + the **gate memory layer** (red+green pairs ordered along the axis from the entry, never forgotten; passed only inside the gate line, else MISSED → re-approach) and the **omni follower** (velocity on the path, sway ≤ 0.15, heading follows; guard corridor along the motion). Open for the real SIL: the planner's 1.68 m keep vs 2–2.5 m gates; the hold counter per object, not per 3 m; search legs round the gate field, not through it.
**04:45 SGT (Dmitrii):** bow-following in GUIDED is the **default** (orbit 2 m → `circle_v` ≈ 0.1); crab (mode 20: orbit + follower) is the secondary experiment behind ticks in a new **"Mission modifiers" subpanel** on the Run Sequence (set before ENABLE AUTO, frozen after; shown in the DECLARE text and the run banner); the run starts with a **small advance + a full rotation**, then the ladder. Reconciliation of the agent's v0.1.92 against these: `docs/COWORK_HANDOFF_2026-11-13.md` addendum 3 (orbit 0.2 → 0.1 + crab mode, the ordered gate list, LOOK_LOCK stages in the GUI, geofence check, modifiers subpanel; keep h-cam 0.55, the corridor, the 2.0 m merge gate).
**05:15 SGT:** keep-out and avoidance box, addendum 4 — the pair of the gate being passed has **no** planner keep (the gate is the corridor); other mapped buoys 1.68 → **1.2 m**; the guard STOPs for a *mapped* buoy only on a collision course (≤ 1 m ahead, ≤ 0.45 m lateral; SLOW ≤ 2 m / 1.0 m); the box for *unknown* objects stays 1.5 / 0.7 and 3 / 1.2; the hold rule counts 3 stops on the same object, not within 3 m.
**18:00 UTC 12 Nov (02:00 SGT; the SGT labels above are 0.5–3.5 h late — true 01:05–01:45 SGT), addendum 5:** "a proper passing through the gates" — the pathing_sim lessons (spec §6.5–§7): the through leg is **cross-track LOS on the gate normal** (look-ahead 1.5, **0.12 m/s**), legs shorter than 2·rmin are straight and entered by a **turn on the spot** (> 25° off the bow; onto the normal within **6°** at the pre-point), run-in = spacing − 1.5 m (≤ 3), `transit_v 0.6 → 0.15`, `lead_in_m 5 → ≤ 3`, orbit **2 m @ 0.08**, approach / scout 0.2 / 0.3. Rehearsal A/B/D × 20: pass offset median 0.00 · p90 0.00 · max 0.10 m (was 0.3 / 0.8 / 1.2), contacts 0 (was 1.1–1.8), exits 0, 60/60 done, 14.7 min median (+2 min; `circle_exit_tangent` ON recovers ~80 s). Acceptance for the tree: a boat 0.4 m off the normal at the pre-point passes within 0.1 m of the mid on the omni_usv plant; OCS shows the TRANSIT micro-stages. Risks: no current in the plant; GUIDED's speed floor / in-place yaw at 0.08–0.15 unmeasured (bench first).
**18:30 UTC 12 Nov (02:30 SGT), addendum 6 — FINAL from Cowork:** the **hybrid follower** as modifier 2 (bow-following + a slewed sway ≤ 0.1 toward the gate line while aligning / on the through leg / a sidestep when the pre-point is abeam; mode 20 only while swaying; 3 sway sign changes in 10 s → off for the gate + re-path), full crab demoted to modifier 3; the **squeeze** risk tick (an unknown object in a run-in passed through a gap ≥ beam + 0.5 m at 0.08 m/s, the guard's stop for it 0.3 m; E: 3/20 → 13/20 done, 0.65 light contacts a run; off by default). Final values in the hand-off §6.4 (transit 0.15, through 0.12, la_through 1.5 cross-track, lead_in ≤ 3, pre_turn 25°, align 6°, circle_v 0.08, exit-tangent ON). All worlds × 20: pass offsets 0.00 / 0.00 / 0.10 m, misses 0, exits 0, 14–15 min a run.

### WP49 — v0.1.93 (2026-11-13, after WP48 is archived): the gate pass — cross-track LOS, turn-before-go, squeeze on by default, the Mission modifiers subpanel (hybrid / crab as modifiers) — Dmitrii's decision 02:40 SGT 13 Nov
Prompt: `docs/PROMPT_GATE_PASS_2026-11-13.md` (§2 the values, §3 the behaviours with acceptance, §4 tests, §5 order). Why: `docs/COWORK_HANDOFF_2026-11-13.md`
addenda 5–7; the rehearsal `pathing_sim/m1t1_logic_check.html`; the lessons `pathing_sim/gate_pathing_simulator_spec.md` §6.5–§7. Acceptance: the through leg
is cross-track LOS on the gate normal (carrot on the pre → post line at the boat's projection + 1.5 m, `through_v 0.12`); legs > 25° off the bow start with a
turn on the spot, the bow is lined up within 6° at the pre-point (cap 90 s); run-in = min(spacing − 1.5, 3.0); `transit_v 0.15`, approach / scout / align
0.2 / 0.3 / 0.15, `circle_v 0.08` (0.10 if the FCU does not hold it), `circle_exit_tangent` ON after a SIL check; **squeeze on by default** (object 0.5 m wide,
need beam + 0.5 m, own side if + 0.3, dog-leg clamped inside the gate, plan before the gate line / pass at 0.3 / clear to 0.75, that object's STOP 0.3 m at
0.08 m/s); the Mission modifiers subpanel on RUN SEQUENCE (start · orbit · follower bow | hybrid | crab · squeeze · look-lock · lucky · budget; one `mods:` line,
in DECLARE and the banner; frozen after ENABLE AUTO); the hybrid follower (sway ≤ 0.1 slewed, 3 sign changes / 10 s → off + re-path, sidestep) and the crab
orbit / follower behind their ticks; micro-stage words on the stage line; keep-out / guard per addendum 4. Fast SIL A/B/D/E × 20 on `omni_usv`: pass offsets
p90 ≤ 0.2 m, contacts ≤ 0.1 a run (A/B/D), misses 0, no boundary exit, E ≥ 10/20 with the squeeze; a test per behaviour (`tests/test_v0193.py`); `make test`;
GUIDE §4 (modifiers, the stage words, the squeeze); MORNING note §4 updated. No Jetson deploy (morning, Dmitrii's go). Bench before the lake: GUIDED's speed
floor at 0.08–0.15 and the in-place yaw rate; the mode-20 flip inside a GUIDED leg (hybrid) with props off.

**WP48 + WP49 DONE → v0.1.93 (13 Nov night, session A; one archive — v0.1.92 was not archived separately):** the entry-first ladder, the LED path,
the guard corridor + mapped-buoy rule, the speed cap, the camera height (WP48) and the gate pass — start turn, look-lock, gate memory, turn first,
ALIGN_GATE, cross-track THROUGH, passed / MISSED, 2.4 m orbit, the modifiers panel, the micro-stage line (WP49 §3.1–3.4, 3.6, 3.9, 3.10). Sim: 400
runs, 0 contacts, 0 wrong entries, 70/70 done. **Not done:** §3.5 squeeze is built but OFF (our SIL: no gain, one contact); §3.7 hybrid, §3.8 crab
not built; `circle_exit_tangent` off (+1 min in the SIL); `circle_v` 0.10 not 0.08 (70/70 vs 65/70). Details: DEVLOG v0.1.92 + v0.1.93.

**Triage 2026-11-12 (competition):** relevant for the presets: WP18 (m1t2), WP19 / WP21 (UAV, preset 0). After the competition: WP20, WP22,
WP23, WP24 (cannon / dock CV — Task 3 is not in the presets).

### WP18 — sim_day S15: m1t2 on the colour-blind world (G1) — not built (half done by WP35: the colour-blind world_scout + S15; m1t2 on it open)
Acceptance: S15 (+ S15b `--no-complete` released by `uav scan done`) in `tools/sim_day.py` — two PREPARE acks, WAIT_UAV_MAP entered and left on `scan.complete`, DONE, the report = the world's truth; the dwell and merge rate in the row.

### WP19 — the UAV adapter's telemetry replay test (G2) — not built
Acceptance: `tools/uav_replay.py FILE --to … --speed 1|10` + `tests/test_v016N_replay.py` — status/uav at 2 Hz with the right fields, the 5 s staleness cut on a gap, the fence block, the heartbeat stop / resume; GUIDE §4.20e capture line.

### WP20 — the Pi's status publisher: run script + synthetic-send walk-through (G3) — not built
Acceptance: `cv/cannon/pi/run.sh start|stop|status` (their receiver untouched) and `docs/CANNON_PI_SETUP.md` (files, their config keys, clocks, the DRY_RUN check, the H4.3 aim loop).

### WP21 — the fake UAV survey on the real hub (G4) — not built
Acceptance: `run_ocs.sh --uav --fake-buoys [--world FILE]` + the launcher's `on + fake buoys`; real telemetry / fence, fake buoys, `scan.complete`, the relay to the NX; the UAV card says *buoys: simulated*; no new topic.

### WP22 — the real-frame CV sheet (G5) — not built
Acceptance: `tools/cv_sheet.py` — found rate per distance band, range error against typed distances, colour agreement, two models compared; markdown into `docs/screens/<version>/`; a CV_TRAIN request line for an under-performing class.

### WP23 — the real observers' key points (G6) — not built
Acceptance: `kp` + `face_points` from `cv/buoys_multi` and `cv/docking` as `sim_observer` computes them; `tools/dock_keypoints_eval.py` (centre ≤ 0.3 m, normal ≤ 5° at 3 / 5 / 8 m); `mission m3t1 --dock-source map|boxes` with the < 2 faces fallback; deploy only after U9b.

### WP24 — the dock measurements version (G7, a 0.1.N.M side fix when C1 has numbers) — not built
Acceptance: `t3.finger_m`, the dock side / window offsets in the three world files, `stop_m` if needed; the standoff tests updated; `sim_day --only S9` and the m3t1 campaign (4 seeds per world) green.
