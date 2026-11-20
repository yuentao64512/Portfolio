# MASTER_SCRIPTS — global_v3 build (v0.1.37)

Companion-side software for the OptiPrime RobotX 2026 USV (Pixhawk 6X Pro + Jetson Orin NX, custom ArduRover
`omni-usv-v2`, FRAME_TYPE 5, mode 20 CUSTOM_MANUAL). One tree, the same on the Mac (OCS), the Orin NX (boat) and the AGX
(perception). Architecture and module references: `USV/test_scripts/global_v3/docs/ARCHITECTURE_global_v3_v0.1.md`,
`MODULES_global_v3_v0.1.md`, Miro board "global_v3".

## The map

```
MASTER_SCRIPTS/
│
│  READ FIRST
├── GUIDE.md          the operator's reference (v0.1.86.8: deploy, networks, start / stop, missions, latches, logs); the venue day = docs/USER_GUIDE_VENUE.md + docs/TEST_GUIDE_VENUE.md
├── HANDOFF.md        the context for whoever picks this up (Claude Code included): rules, hosts, identities, layout, ladder, troubleshooting
├── ROADMAP.md        where this is going: target state, work packages WP0–WP6 with acceptance criteria, the mission codes
├── DEVLOG.md         where it is: one block per version, newest last — what changed · where · tests · open items · do this first
├── CHANGELOG.md      one line per version
├── CLAUDE.md         the short version Claude Code loads: rules + how a work session goes
│
│  THE CODE  (code once, profiles twice: everything in Python lives here, proven by `make test`)
├── optiprimex/       the package: companion/ (config, frames, health, state, MAVLink link, MQTT bus), usv/ (mission manager, FSM,
│                     tasks (T4, T1, T3, TEST_*), behaviours, gatepath planners, dockpath, perception map), ocs/ (ocs_app, authority, judge clients fake + protobuf, rc_wire, rc_pb = RoboNation schema), sim/ (fake FCU, fake perception,
│                     fake dock, world helpers, fast SIL, fake RoboCommand), tools/ (op-tui, ocs_cli, playbook = mission catalogue, map.html, textmap, scenario)
├── tests/            197 tests: unit + two-broker integration (B1, SIL-1, stall, perception, usv_ui, cv kit)
├── usv_ui/           the boat console usv_ui.py (holds the FCU link, shares it, handover/takeover, cv map, MISSION tab) + its README
├── cv/               the AGX perception programs, one folder per capability: buoys, docking, cannon, publish, stream (camera → Mac) (README.md, PROVENANCE.md)
├── pyproject.toml · requirements.txt · Makefile     make venv (→ .venv here) · make test · make bench · make stop
│
│  WHAT DIFFERS PER HOST  (no code)
├── profiles/course/  the competition: ocs_course.yaml (protobuf judge, course rules), usv_course.yaml (no GOTO), mosquitto/rctest.conf
├── profiles/pool/    the hosts: usv_pool.yaml (NX), usv_lab.yaml (--lab, bench guards), ocs_pool.yaml (Mac), mosquitto/*.conf.in
├── profiles/bench/   the laptop simulation: usv_bench.yaml, ocs_bench.yaml, mosquitto/{hub,boat,bridge}.conf; worlds for the sim CV:
│                     world_bench.yaml (unit tests), world_pool.yaml (the 50 × 25 m pool: Task 1 + Task 3 dock), world_por.yaml (PoR course)
├── scripts/          every launcher, run from this root: run_usv.sh / run_ocs.sh / run_agx.sh / run_bench.sh, stop_*.sh,
│                     setup_orin.sh / setup_agx.sh (once per Jetson), status.sh, _netenv.sh (loads the address files)
├── network.env       ports, FCU link, AGX paths, NET_PROFILE=ts|lan        (Mac only, scp'ed to the Jetsons; never in the archive)
├── network.ts.env    the addresses on the Tailscale tailnet (lab)          ─┘
├── network.lan.env   the addresses on the pool switch
├── logs/             run time: usv/ ocs/ agx/ bench/ → run_<stamp>/ (+ latest)
│
│  REFERENCE
├── docs/             BENCH.md (the package and the laptop bench), BENCH_GUIDE.md (the walkthrough), LAUNCHERS.md (what each launcher starts),
│                     JETSON_LOG.md (every change made on the Jetsons from the Mac), ROBOCOMMAND.md (the real judge wire, rc-test), merge/ (the students' box-inspection survey)
├── handbook/         the RobotX 2026 Team Handbook (md for reading, pdf; the pdf stays out of the archive)
├── pathing_sim/      Dmitrii's gate pathing simulator: the S6 browser sketch, its replication spec (WP2 planner port), the report (pdf not archived)
└── _archive/         Mac only: MASTER_SCRIPTS_v*.tgz, screenshots, the pre-v0.1.19 tree
```

## Which document when

| you want to … | read |
|---|---|
| start the stack on a host, test a link, run a pool leg, find a log | `GUIDE.md` (the section number is in every error hint) |
| understand the rules, the hosts, the ladder R/V, why something is refused | `HANDOFF.md` |
| know what to build next, and what "done" means for it | `ROADMAP.md`, then the last block of `DEVLOG.md` |
| run the laptop simulation, understand the package | `docs/BENCH.md`, `docs/BENCH_GUIDE.md` |
| the boat console's words, the AGX programs | `usv_ui/README.md`, `cv/README.md` |

## Three commands

```bash
scripts/launch.sh                           # Mac: the one script - presets 0 · 1 · 2 · 3 · 3b · sim · lab · custom (GUIDE §3; tk: scripts/launch_tk.sh)
make venv && make test                      # Mac, once per machine: the environment and the proof (289 tests, ~3 min on 6 processes; make live ~1 min, make quick < 1 min)
scripts/run_ocs.sh    ·  scripts/run_usv.sh --fcu fake  ·  scripts/run_agx.sh      # one per host, from this root (GUIDE §3)
scripts/stop_ocs.sh   ·  scripts/stop_usv.sh            ·  scripts/stop_agx.sh      # the matching stops
```

Rule: **code once, profiles twice.** All Python lives in `optiprimex/` (plus the two kits `usv_ui/`, `cv/`). `profiles/` and
`scripts/` only hold what differs per host. Anything proven on the bench is deployed unchanged.

v0.1.86: `profiles/presets.yaml` = the competition presets (GUIDE §3, docs/LAUNCHERS.md "v0.1.86"); `make day` = the between-runs test tier.
