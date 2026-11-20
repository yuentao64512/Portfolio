# HANDOFF — MASTER_SCRIPTS (global_v3 build) for the pool trials and the comms-stack test

Written 2026-10-27, updated 2026-11-02 (v0.1.19), for whoever runs this next with a terminal in hand — Dmitrii with
Claude Code, on the Mac, the Orin NX or the AGX. **The archive is `MASTER_SCRIPTS_v0.1.37.tgz`**, the same file for both
Jetsons (it extracts over whatever is already in `~/Desktop/dm3/MASTER_SCRIPTS`). **Renamed in v0.1.12: `pool_test4` → `usv_ui`**
(folder, program, tmux window `ui`, MQTT node `presence/usv_ui`, map source `usv_ui`; `release` / `active` → `handover` / `takeover`,
old words kept as aliases) — an old `pool_test4/` folder on a Jetson is stale, remove it. Since 2026-10-29 the three hosts are on the Tailscale tailnet
(`network.env`). **The operator's document is `GUIDE.md`** (v0.1.86.8 rewrite; the venue day: `docs/USER_GUIDE_VENUE.md`, `docs/TEST_GUIDE_VENUE.md`; the old stages A/B/C: `docs/old/GUIDE_v0.1.86_full.md`): deploy, then test brokers, bridge, presence, handshakes and the
handover with the fake FCU on the NX — no Pixhawk involved. Read this file first, then `ROADMAP.md` (where this goes) and the last block of `DEVLOG.md` (where it is), then `docs/BENCH.md`
(the reference), `docs/BENCH_GUIDE.md` (the walkthrough), `GUIDE.md` §7 (the trial).

## 0. Hard rules (these do not bend)

1. **The companion never arms by itself.** No program decides to ARM / DISARM or change the FCU's mode; those are the
   *operator's* commands - typed in `usv_ui.py` (as pool_test3), and since v0.1.50 (Dmitrii, 2026-11-06: "ignore ADR-9") the
   pages' buttons: the OCS page MANUAL / HOLD / ARM / DISARM (ocs_cli `fcu_mode` / `arm` / `disarm` -> the mission manager's
   FCU_MODE / FCU_ARM), the boat window every mode + ARM / DISARM (usv_ui's page words, whoever holds the FCU). ARM asks first.
   The transmitter's kill (SF = RC5) beats every program on the boat; RC6 MANUAL override is **latched, last wins** (a later
   page command clears it, the next flip takes the boat back) unless `rc6_policy hold` / `guards.rc6_policy: hold`.
2. **Bench = props off, ESC power off.** Any test with a real Pixhawk on a table runs with the propellers removed or the
   ESC supply disconnected. The handover test (section 6, L2) needs nothing but mode changes.
3. **`apply_params.py` (orin_kit) is the only parameter-writing tool** for presets. `usv_ui param NAME VAL` writes one
   parameter disarmed and reads it back — use it for a one-off, note it in the log (`note ...`), never for a preset.
4. **One program per serial port.** `usv_ui.py` (or `rc.sh`) holds `/dev/ttyTHS1`; everyone else (the mission
   manager, a second tool) uses the udp share `udpin:127.0.0.1:14551`. Never open the UART twice; `rc.sh` and
   `usv_ui.py` refuse when a `.share` file with a live pid exists.
5. **Secrets stay out of files, logs and chats:** `actual_buoy/buoy_link.key` (HMAC key of the AGX preview link) is never
   copied, printed or committed; same for any token. The archive does not contain it.
6. **Do not replace the student's telemetry-only Orin→AGX bridge with a bidirectional one** (their README says why).
   The CV programs in `cv/` are read-only on the FCU (telemetry only); `cv_publish.py` reads their status files and
   publishes MQTT; nothing on the AGX sends MAVLink commands. The students' own folders on the AGX are never modified:
   `cv/` is our snapshot of their programs (provenance in `cv/PROVENANCE.md`), it runs and writes only under `cv/`.
7. **Code once, profiles twice.** Python lives in `optiprimex/` (proven by `make test`) and in the pool kit
   `usv_ui/` (proven by `tests/test_usv_ui.py`). Profiles/launchers differ per host; code does not.
   `control_v2/` on the Mac is a frozen checkpoint — never edit it.
8. **Brokers are ours:** never `brew services start mosquitto` / `systemctl enable mosquitto`; the launchers run their
   own instances with their own configs. Port clashes are the first thing to check when something "does not connect".
9. Minimal, focused diffs; respect existing names; after a change say *what changed · where · risks · validation*.
   Tests by tier (2026-11-05): `make quick` for every change, `make live` when the wire / launchers / usv_ui / OCS change, `make test`
   (≈3 min on 6 processes, 289 tests, needs mosquitto on PATH) for every WP version and before a Jetson deploy - CLAUDE.md step 3.

## 1. What this is (one paragraph)

A companion-software stack for the Omni BlueBoat (Pixhawk 6X Pro + Jetson Orin NX hybrid board, custom ArduRover 4.7
`omni-usv-v2` bda2f167, FRAME_TYPE 5, mode 20 = CUSTOM_MANUAL direct thruster frames, mode 15 GUIDED). The **mission
manager** (`optiprimex.usv.mission_manager`) runs on the NX: vehicle states V0–V6, tasks (T4 incident, GOTO, TEST_MANUAL / TEST_AUTO = the
state-machine test missions, `usv/tasks/test_tasks.py`), a perception map; it talks MAVLink to the FCU and MQTT to the boat broker. The **OCS app** (`optiprimex.ocs.ocs_app`) runs
on the Mac: run lifecycle R0–R6, the single command authority (operator intents → vehicle commands), judge client (fake
at the pool, RoboCommand at the course). Two Mosquitto brokers — boat (NX, 1884) and hub (Mac, 1883) — are joined by
the real Mosquitto bridge. **usv_ui.py** is the operator's pool tool (mode-20 bursts, save/goto, paths, swayto,
logging, MAVLink inspector) that now *also* holds the FCU link for the mission manager, shows the CV map, and hands the
FCU over with `handover` / `takeover` (you stay in usv_ui either way). Architecture: `USV/test_scripts/global_v3/docs/ARCHITECTURE_global_v3_v0.1.md`,
`MODULES_global_v3_v0.1.md` (§10.11 is the latest amendment), Miro board "global_v3".

## 2. Hosts, links, identities

**All real addresses live in `MASTER_SCRIPTS/network.ts.env` (tailnet) and `network.lan.env` (pool switch), picked by `NET_PROFILE` in `network.env` or `--ts` / `--lan` on a launcher** (OCS_HOST = the Mac, BOAT_HOST = the Orin NX,
AGX_HOST, broker ports, websocket ports, FCU_LINK, ORIN_SSH/AGX_SSH). The launchers source it (a variable set on the
command line wins), `cv_publish.py` reads it for its default broker. Lab (tailnet): Mac `100.118.249.71` (OCS_HOST), NX `100.88.41.24`,
AGX `100.114.195.19` — edit the file on the Mac only and scp it to both Jetsons so all three copies agree. Pool
switch: the 192.168.1.x values in the comments (`ipconfig getifaddr en0` for the Mac). The YAML profiles keep `127.0.0.1` on purpose: every process
talks to the broker on its own host; only the bridge, op-tui's boat feed and map.html cross the network.

| host | role | address / user | what runs there |
|---|---|---|---|
| Mac (M3) | OCS laptop | `~/Documents/BH1/RobotX/MASTER_SCRIPTS` | hub broker 1883 (+ws 9001), `ocs_app`, fake RoboCommand, `ocs_cli`, `op-tui`, `map.html`, the bench |
| Orin NX (hybrid board) | USV companion | `usv-pixhawk@100.88.41.24` on the tailnet (pool switch: 192.168.1.218); kit at `~/Desktop/dm3/MASTER_SCRIPTS` | boat broker 1884 (+ws 9001), `usv_ui.py` (holds the FCU link; `--fake-fcu` = the bench plant instead), mission manager (`LOG=1`: boat logger) |
| Pixhawk 6X Pro | FCU | UART `/dev/ttyTHS1` @ 921600 (TELEM2, hybrid preset); USB jumper `/dev/ttyACM0`; Ethernet 10.41.10.2 → NX 10.41.10.1:14550 | ArduRover `omni-usv-v2`, sysid 1 |
| AGX Orin | perception | `optiprime-agxorin@100.114.195.19` on the tailnet (pool switch: 192.168.1.99); our kit at `~/Desktop/dm3/MASTER_SCRIPTS`; the students' originals at `/home/optiprime-agxorin/actual_buoy` and `.../docking_cv` (read-only for us: models, their venv, the key) | `cv/buoys` (buoy observer, ZED + YOLO), `cv/docking` (red-dock observer, perception only), `cv/cannon` (aiming link → water-cannon Pi 192.168.6.7:5005, HMAC UDP, T3), `cv/publish` (cv_publish → NX broker) — each with its own `run.sh` |

MAVLink identities (decided 2026-10-27): **both** companion programs are system id **200** (= `CMAN_SYSID`, the only
system mode 20 accepts control frames from). `usv_ui.py` is component **190**, the mission manager component
**191** (`usv_pool.yaml mavlink.compid`), so COMMAND_ACKs are addressed to the one that asked (the link code ignores
acks for the other component). `rc_monitor.py` is 255/191 (GCS-like) and is not used when usv_ui runs.

Who commands the FCU: exactly one at a time. `usv_ui` beats `local/usv/tui/active` every second while it is
**ACTIVE**; the mission manager treats a beat younger than `tui_active_s` (2 s) as "usv_ui active" → it sits in V2
(RC-override) and refuses ENABLE_AUTO. `handover` stops the beat (the mission manager may go AUTO on the OCS's
ENABLE_AUTO); `takeover` resumes it; **`stop`, `x`, `disarm` always take the FCU back at once** (the mission manager drops
to V2 within a tick). While handed over you stay in usv_ui (telemetry, map, `stop`) but every FCU command typed there is refused with that hint.
`release` / `active` still work as aliases (the words until v0.1.11).

MQTT topics that matter at the pool (team namespace `team/optiprime/v1`, boat-local `local/usv`):

| topic | who → who | content |
|---|---|---|
| `local/usv/perception/buoys`, `/dock`, `/health` | cv_publish (AGX) → usv_ui + mission manager | body-frame Observations (range, bearing, class, colour, confidence) from the buoy observer and, with `--dock-output`, the dock mapper; health 1 Hz |
| `local/usv/tui/active` | usv_ui → mission manager | `{active: true}` beat (1 Hz) while usv_ui has the FCU |
| `local/usv/mm/state` | mission manager → usv_ui, fake_perception, fake_dock | full vehicle state 5 Hz (usv_ui shows `mm: <V state>`, CO same/DIFFERS) |
| `team/.../telemetry/usv/map` | usv_ui and the mission manager → hub (bridged) → map.html, op-tui | boat, trail, entities, gates, dock, path, points; envelope `source` says which program |
| `team/.../status/usv`, `ack/usv`, `event/usv/#` | mission manager → hub | 2 Hz status, command acks, state events |
| `team/.../cmd/usv/#` | OCS → mission manager | vehicle commands (only through the OCS authority: `ocs_cli`) |
| `presence/<node>` | every node (retained, 2 s) | up/down of usv_ui, mission_manager, cv_publish, logger-*, ocs_app … (op-tui NETWORK tab) |

## 3. The archive layout (v0.1.19: one level, grouped by kind)

```
MASTER_SCRIPTS/
├── README.md                  the map of this tree (start here)        CLAUDE.md (the short version Claude Code loads)
├── GUIDE.md                   the one operator document: A deploy + architecture test (fake FCU), B real Pixhawk on the bench, C pool legs, logs
├── HANDOFF.md                 this file: rules, hosts, identities, layout, ladder, troubleshooting
├── ROADMAP.md                 target state and the work packages WP0–WP6 (acceptance criteria, file pointers, mission-code table)
├── DEVLOG.md                  append-only log, one block per version (what changed · where · tests · open items) — read the last block first
├── CHANGELOG.md               one line per version (what the user sees)
├── network.env                ports, FCU_LINK, AGX_* paths and NET_PROFILE=ts|lan; the addresses are in the two profile files below
├── network.ts.env             OCS_HOST, BOAT_HOST, AGX_HOST, ORIN_SSH, AGX_SSH on the tailnet (lab)      (Mac only; scp'ed to the Jetsons)
├── network.lan.env            the same on the pool switch (192.168.1.x; fill OCS_HOST = the Mac's switch address)
├── pyproject.toml, requirements.txt, Makefile   the package (`make venv` → .venv at this root, `make test`, `make bench`)
├── optiprimex/                THE code (package 0.1.19): companion/ (config, frames, health, state), usv/ (mission_manager, perception_map,
│   │                          behaviours), ocs/ (ocs_app, intents), sim/ (fake_fcu, fake_perception, fake_dock, world, fake_robocommand), tools/
│   └── tools/                 op-tui (bench_tui.py: MAIN = the shared skeleton, MISSION tab, BOAT, NETWORK, JUDGE, LOG, HELP), map.html
│                              (browser map over a broker websocket), textmap.py, ocs_cli, playbook (mission catalogue, codes), scenario
├── tests/                     124 tests (unit + two-broker integration; `make test`, mosquitto on PATH)
├── profiles/
│   ├── pool/                  the hosts: usv_pool.yaml (NX, endpoint = usv_ui's share 14551), usv_lab.yaml (--lab: bench guards, real Pixhawk
│   │                          indoors), ocs_pool.yaml (Mac: fake judge, bench_mode rules), mosquitto/*.conf.in (rendered by the launchers)
│   └── bench/                 the laptop simulation: usv_bench.yaml, ocs_bench.yaml, world_bench / world_pool / world_por.yaml (sim CV worlds), mosquitto/*.conf
├── scripts/                   every launcher, run from the tree root: run_usv.sh / stop_usv.sh (NX: broker + usv_ui + mission manager, tmux 'usv';
│                              --fcu fake|uart|usb|eth, --lab), run_ocs.sh / stop_ocs.sh (Mac: hub + fake judge + ocs_app + op-tui + cli, tmux 'ocs'),
│                              run_agx.sh / stop_agx.sh (AGX: cv_publish, tmux 'agx'), run_bench.sh / stop_bench.sh (laptop simulation, tmux 'bench'),
│                              setup_orin.sh, setup_agx.sh (once per Jetson), status.sh, _netenv.sh (loads network.env + the profile overlay)
├── logs/                      run time, git-ignored: usv/ ocs/ agx/ bench/ → run_<stamp>/ (+ latest symlink)
├── usv_ui/                    the boat console: usv_ui.py (pool_test3 + share + MQTT + cv map + handover/takeover; MAVLink 200/190),
│                              config/usv_bench_share.yaml (laptop rehearsal through the share), Makefile (make test | make rehearsal), README.md
├── cv/                        the perception programs for the AGX, one folder per capability (README.md, PROVENANCE.md):
│   ├── buoys/                 students' buoy observer (verbatim) + their UDP sender + setup check; run.sh; models/ (sync_models.sh); runs/
│   ├── docking/               dock_observer.py (derived from their dock mapper: perception + geolocation only); run.sh; models/; runs/
│   ├── cannon/                cannon_sender.py (ours): docking's status file -> HMAC dock_cannon.v1 packets to the Pi; run.sh
│   ├── publish/cv_publish.py  buoys' + docking's status files -> MQTT Observations on the NX broker (usv_ui 'cv on', the map)
│   └── _env.sh, sync_models.sh, requirements-agx.txt   interpreter resolver, model copy, their venv manifest = our venv recipe
├── docs/                      reference: BENCH.md (the package, the bench, the version history), BENCH_GUIDE.md (the laptop walkthrough),
│                              LAUNCHERS.md (what each launcher starts)
└── _archive/                  Mac only: MASTER_SCRIPTS_v*.tgz, screenshots, the pre-v0.1.19 tree (pre_v0.1.19_layout/)
```

Before v0.1.19 the same files sat in `bench/` (code, tests, laptop profile, venv), `deploy/` (host profiles, launchers, logs) and
`final/` (empty placeholder); `usv_ui/POOL_RUNBOOK.md` and `INFRA_TEST.md` were pointers into GUIDE.md. Old paths in a log or a
chat map 1:1: `bench/optiprimex` → `optiprimex`, `deploy/scripts` → `scripts`, `deploy/config` → `profiles/pool`, `bench/config` →
`profiles/bench`, `deploy/logs` → `logs`, `bench/.venv` → `.venv`.

Python: `pip install -e .` (console scripts `op-*`, `ocs_cli`) needs ≥ 3.10; **running** needs only ≥ 3.8 with
`PYTHONPATH=<root>` — the launchers set that, and `usv_ui.py` finds `../optiprimex` by itself. Dependencies:
pymavlink, paho-mqtt ≥ 2, PyYAML, jsonschema (+ pytest, pytest-timeout for the suite), mosquitto, tmux.

## 4. Setup per host

Three hosts, three separate setups, one rule for the two Jetsons: **our kit lives in `~/Desktop/dm3/MASTER_SCRIPTS`
and nothing of the students' folders is modified** (`actual_buoy/`, `docking_cv/` on the AGX; `orin_kit/` is ours but
stays as deployed on the NX). Fill `network.env` on the Mac first, so every copy carries the same addresses
(OCS_HOST = the Mac's pool IP from `ipconfig getifaddr en0`, BOAT_HOST = the NX, AGX_HOST = the AGX).

### 4.1 Mac (OCS laptop) — done for the bench

`make venv && make test` at the tree root (brew: python@3.10, mosquitto, tmux). `make test` must say `289 passed`. The pool
launchers for this side: `scripts/run_ocs.sh` / `stop_ocs.sh` (§5).

### 4.2 Orin NX (`usv-pixhawk@100.88.41.24`, the boat)

```bash
ssh usv-pixhawk@100.88.41.24 'mkdir -p ~/Desktop/dm3'
scp ~/Documents/BH1/RobotX/MASTER_SCRIPTS/_archive/MASTER_SCRIPTS_v0.1.37.tgz usv-pixhawk@100.88.41.24:~/Desktop/dm3/
ssh usv-pixhawk@100.88.41.24
cd ~/Desktop/dm3 && rm -rf MASTER_SCRIPTS/{bench,deploy,final} && tar xzf MASTER_SCRIPTS_v0.1.37.tgz && cd MASTER_SCRIPTS   # rm: the pre-v0.1.19 folders, once
bash scripts/setup_orin.sh          # venv in .venv at the root (pymavlink, paho-mqtt, pyyaml, jsonschema); says what is missing
sudo apt install -y mosquitto tmux  # if it said so;  then: sudo systemctl disable --now mosquitto
sudo date -u -s "$(date -u '+%Y-%m-%d %H:%M:%S')"   # from the Mac: the Orin clock drifts; t_cam needs clocks within 1 s
```
`scripts/run_usv.sh` is the boat launcher (§5). Python ≥ 3.8 is enough to run (PYTHONPATH); `pip install -e` needs 3.10.

### 4.3 AGX Orin (`optiprime-agxorin@100.114.195.19`, perception) — separately, our own folder

The AGX runs **our `cv/` tree**: snapshots of the students' programs, one folder per capability, each with its own
`run.sh`, writing only under `cv/*/runs` (`cv/README.md`, provenance and the exact changes in `cv/PROVENANCE.md`). Their
folders (`actual_buoy/`, `docking_cv/`) stay as they are and are used read-only for three things: the model files
(`cv/sync_models.sh` copies them — 63 + 23 MB, the TensorRT engine is built for this AGX), their venv (pyzed, torch,
ultralytics with system site-packages; `network.env AGX_BUOY_VENV`, until our own `cv/.venv` exists — the gated recipe
is `cv/requirements-agx.txt`), and the HMAC key file (path in `cv/cannon/cannon_config.json`, never copied).

```bash
# from the Mac: the same archive that went to the NX
ssh optiprime-agxorin@100.114.195.19 'mkdir -p ~/Desktop/dm3'
scp ~/Documents/BH1/RobotX/MASTER_SCRIPTS/_archive/MASTER_SCRIPTS_v0.1.37.tgz optiprime-agxorin@100.114.195.19:~/Desktop/dm3/
scp ~/Documents/BH1/RobotX/MASTER_SCRIPTS/network*.env optiprime-agxorin@100.114.195.19:~/Desktop/dm3/MASTER_SCRIPTS/   # after the extract below
ssh optiprime-agxorin@100.114.195.19
cd ~/Desktop/dm3 && rm -rf MASTER_SCRIPTS/{bench,deploy,final} && tar xzf MASTER_SCRIPTS_v0.1.37.tgz && cd MASTER_SCRIPTS
bash scripts/setup_agx.sh                 # our paho venv for cv_publish (.venv at the root); checks their venv, the runs dirs, the models
cd cv && ./sync_models.sh                 # models into cv/buoys/models and cv/docking/models (sha256 printed, compare with PROVENANCE.md)
buoys/run.sh --verify                     # CUDA / ZED / TensorRT / model import check with their interpreter
buoys/run.sh --self-test; docking/run.sh --self-test; cannon/run.sh --self-test     # no camera, no key, no network
sudo date -u -s "$(date -u '+%Y-%m-%d %H:%M:%S')"                        # clock within 1 s of the NX (the map uses t_cam)
cd .. && scripts/run_agx.sh --demo                # link test (tmux 'agx'): usv_ui on the NX -> 'cv status' shows the detector
```
Then the real thing, each from the `cli` window of tmux `agx` (`run_agx.sh` opens it in `cv/` with the venv and the hints;
`Ctrl-b c` for a second shell): `cv/buoys/run.sh --headless` (buoy observer →
`cv/buoys/runs/<stamp>/latest_status.json`), `cv/docking/run.sh --headless` (dock observer → `cv/docking/runs/latest/`),
`cv/cannon/run.sh --dry-run` then without the flag (the aiming link to the Pi; `--synthetic-send` is the receiver test),
and `scripts/run_agx.sh` (cv_publish, follows both runs dirs). Docking and cannon are independent: the dock
observer runs and maps without the Pi; the cannon link runs against a recorded or synthetic status file without the camera.

The AGX and the NX must agree on time within a second for the map to use the right pose at capture time (usv_ui
logs `cv: t_cam … off our clock` when they do not; it then falls back to the latest pose, fine at < 1 m/s).

## 5. Running

**Standalone on the boat (no Mac needed)** — Orin NX:
```bash
cd ~/Desktop/dm3/MASTER_SCRIPTS/deploy
scripts/run_usv.sh                  # tmux 'usv': boat broker, usv_ui (UART), mission manager  (LOG=1 ... adds the SQLite logger)
tmux attach -t usv                  # window ui = the tool; Ctrl-b n/p windows; Ctrl-b d detach
scripts/stop_usv.sh                 # usv_ui 'quit' first, then the rest; prints 'down: ...' or 'STILL UP: ...' (--wipe: fresh broker state)
```
The FCU switch: `--fcu fake | uart | usb | eth` (or `FCU_LINK` in `network.env`); `--lab` = `config/usv_lab.yaml`, the bench
guards for a real Pixhawk indoors (AUTO accepted disarmed and without EKF; `EKF_BAD` / `RC_LOST` reported, never a FAULT) —
GUIDE §6; never at the pool. `fcu fake | real` inside usv_ui switches the link live (GUIDE §6). `--ui-args "--log pool"` to start usv_ui logging.

**On the AGX** (`cd ~/Desktop/dm3/MASTER_SCRIPTS`): `scripts/run_agx.sh --demo` first (a synthetic buoy 8 m
ahead at 5 Hz, the link test without the camera), then `cv/buoys/run.sh --headless` and/or `cv/docking/run.sh --headless`
in their own shells plus `scripts/run_agx.sh`: it follows `cv/buoys/runs` and `cv/docking/runs/latest` and
publishes `perception/buoys` (+ `perception/dock` when the docking dir exists; `--no-dock` to skip). `cv/cannon/run.sh`
is the separate aiming link to the Pi. `tmux attach -t agx` opens on the `cli` shell (window `cv` = cv_publish's log,
nothing to type there); `scripts/stop_agx.sh` stops cv_publish and whatever runs in `cli`.
On the NX, usv_ui `cv status` must show `detector … ok=True`; `cv on` then logs `map: new buoy …` / `map: new dock …`.

**With the OCS on the Mac** (the comms-stack test): fill `network.env` (OCS_HOST = the Mac's IP, BOAT_HOST = the NX), then
on the Mac `scripts/run_ocs.sh` and on the NX `scripts/run_usv.sh` (both from the tree root). The boat broker bridges into the hub; the
Mac's op-tui opens on its MAIN tab, the **BOAT tab** is the boat seen from shore (vehicle, position/CO, the perception map
as text, detector health, who has the FCU — usv_ui ACTIVE / handed over or the mission manager; FLOW strip), the NETWORK tab lists the
boat nodes and `bridge up`; `ocs_cli` on the Mac drives the mission manager. The **browser map** (`optiprimex/tools/
map.html`) opens by itself when `run_ocs.sh` / `run_bench.sh` start on the Mac (`MAP=0` to skip), and `map` typed in
op-tui or in usv_ui opens it again (usv_ui on the Orin prints the URL to open on the Mac, with the boat's address).
The OCS side has no CV commands on purpose: `cv on|off` is the boat operator's switch in usv_ui; in the final stack the
mission manager ingests detections always and the OCS only watches the map.

**Laptop rehearsal of the whole pool layout (no hardware)** — Mac, three shells:
```bash
MAP=0 BOAT_HOST=127.0.0.1 scripts/run_ocs.sh                                                   # hub side  (all three at the tree root)
OCS_HOST=127.0.0.1 BOAT_WS=9002 scripts/run_usv.sh --fcu fake                                  # boat side: usv_ui starts the fake plant itself
#  (or a plant of your own first: .venv/bin/op-fake-fcu --conn udpout:127.0.0.1:14550 --control-port 14560 --armed, then LINK=udpin:127.0.0.1:14550)
```
then in usv_ui: `handover` → Mac: `ocs_cli ready`, `ocs_cli goto --x 10 --y 0 --speed 0.8` → usv_ui shows
`HANDED OVER … mm: AUTO_TASK GOTO/GO` → `stop` → `ACTIVE … mm: MANUAL`. (Also `cd usv_ui && make rehearsal`, which
starts the bench with the fake detector and usv_ui in one tmux session.) Proven 2026-10-27 on Linux.

## 6. Test ladder — what is proven, what is next

| level | where | status | what "good" looks like |
|---|---|---|---|
| L0 laptop | Mac | **done** (`make test` 111 pass; live rehearsal) | share carries the mission manager; handover/takeover; cv map 3 gates + dock within 1 m; map.html; op-tui BOAT tab |
| L0.5 real hosts, fake plant | Mac + NX + AGX over the tailnet | **done 2026-10-29** (`GUIDE.md` stage A) | `run_usv.sh --fake-fcu` on the NX: bridge up, presence of every node on both brokers, AGX demo → map, ENABLE_AUTO refused while ACTIVE, release → ready/goto/hold/T4, stop takes back, bridge outage and recovery |
| L1 bench, props off | NX + real FCU | next | `run_usv.sh --fcu uart --lab` (GUIDE §6: indoors EKF_BAD would otherwise park the vehicle in FAULT): usv_ui MAIN shows `HB … sys 1`, `share n/m`, `mm: MANUAL CO same`; `mode 20`, `arm`, `motor 1 0.2 1` as in pool_test3 (±80 µs on one channel) |
| L2 handover, props off | NX (+ Mac) | next | `handover` → `ocs_cli ready` on the Mac → status `AUTO_HOLD`, FCU mode HOLD held by the mission manager; `stop` → `ACTIVE`, status `MANUAL`. Nothing moves: mode changes only |
| L3 CV link | AGX + NX | next | `cv_publish.py --demo` → usv_ui `cv status`: `detector actual_buoy_observer ok=True`, `cv on` → `map: new buoy #1 unknown at N … E …` 8 m ahead of the bow. Then the real observer with a buoy in view; then `--dock-output` with the dock mapper and a red dock face |
| L4 pool legs | pool | tomorrow | `GUIDE.md` §7 T-A … T-F: transit (goto / pathto), CV (cv on, map, map.html), CV + transit, sway (swayto), docking approach (student's guarded_docking stays separate), sway + dock; final-stack legs through `ocs_cli` while released |

Known gaps to keep in mind: the student's detector reports **one buoy per frame, no colour** → the map shows buoys
(`unknown`), but gates (red/green pairing) will not form from real CV until colour exists (`cv_publish --colour red`
forces one for a test; a mask-HSV colour vote in the observer is the real fix, OPTIONAL, ~15 lines). No RoboCommand at
the pool: `run_ocs.sh` runs the fake judge. The course-network profile (real judge, ACLs, `bench_mode: false`,
`allow_goto_task: false`) does not exist yet — it will be a third folder under `profiles/` (ROADMAP WP4). The calibrated sway stern factors (F+ 1.05 / F− 0.877, pool 2026-10-21) live in
usv_ui only; the mission manager's `gains` still use the firmware constant — carry them over before any mode-20
behaviour (sway, docking) moves to the mission manager. Miro frame 23 module tables predate v0.1.2.

## 7. When something is off

- `port 1882/1884/9001/9002 already in use` (v0.1.90.12 numbers) → an old run (`stop_usv.sh` / `stop_ocs.sh` / `stop_bench.sh`, all in `scripts/`) or a
  system mosquitto. `BOAT_WS=9002` when hub and boat brokers share a host.
- usv_ui `NO HEARTBEAT` → wrong link (`LINK=usb|eth`), baud (57600 before the hybrid preset), `dialout`, or `rc.sh`
  holding the port (its `.share` file).
- `mm: not seen` → the mission manager is not on the boat broker (window `mm` in tmux shows why: config path, `PYTHONPATH`,
  a yaml typo); `mm: SILENT` → it died (its log in `logs/usv/latest/mm.log`).
- `CO DIFFERS` → usv_ui's co.json and the mission manager's origin differ: put co.json's lat/lon in
  `usv_pool.yaml frames.origin_lat/lon` (or `co` in usv_ui at the boat right after both started on the first fix).
- `cv on` refused → no optiprimex import (`PYTHONPATH`), no MQTT, or no control origin yet.
- `cv status` says `no perception/health yet` → cv_publish not running or publishing to another broker/host (check `network.env` BOAT_HOST on the AGX copy).
- the map window did not appear → `map` in op-tui / usv_ui prints the URL; open it in any browser on the Mac; `?host=` must be a broker with a websocket listener (hub 9001 on the Mac, boat 9001 on the NX).
- `DECLARE refused: USV1 not heard` → the OCS has no fresh status from the vehicle (3 s): the NX stack is down, the bridge is not up,
  or the profile points at the wrong address (`network: ts|lan` in the launcher's output). `prepare_timeout` alert → PREPARE went
  out but no ack within 3 s: the mission manager is not on the boat broker (tmux `usv` window `mm`).
- the OCS shows R5 but the vehicle MANUAL with `run -` → someone typed `reset` in usv_ui; `ocs_cli reset` brings the OCS to R2 too.
- ENABLE_AUTO REJECTED with "usv_ui active (type handover there)" → usv_ui commands the FCU: type `handover` in usv_ui (the op-tui
  `next:` line and the FLOW strip say the same; `takeover` / `stop` / `x` take it back).
- acks in usv_ui's log `(for comp 191 = mission manager)` are the mission manager's, not ours.
- usv_ui says `OCS LINK DOWN n s (bridge)` (ACTION / LINKS, WARNINGS, the bar): the boat broker's bridge to the hub dropped - the Mac
  stack is down, the Wi-Fi / tailnet link is gone, or `OCS_HOST` points at the wrong address (network profile, GUIDE §2). The mission
  manager continues (`guards.link_loss_action continue`; `safe_hold` would park a running task in V5 after `link_loss_s`); the OCS
  sees `status stale` on its side. It clears by itself when the bridge is back (`OCS LINK UP` in EVENTS).
- Logs: NX `logs/usv/run_<stamp>/` (boat.log, mm.log, fcu.log; `boat_<stamp>.sqlite` only with `LOG=1`; pool_<utc>.tlog/.csv/.events
  when `log on`); Mac `logs/ocs/run_<stamp>/` (hub.log, ocs.log, rc.log; `hub_<stamp>.sqlite` only with `LOG=1`); AGX `logs/agx/`.
  SQLite recording is off by default since v0.1.10 (disk space): `LOG=1 scripts/run_usv.sh` / `LOG=1 scripts/run_ocs.sh`.
- Stopping: `scripts/stop_usv.sh` / `stop_agx.sh` / `stop_ocs.sh` (GUIDE §3 per host, §5 all); each ends with `down: …`
  or `STILL UP: …` + the pkill to use; `tmux ls` must then be empty.
- A stale `usv_ui` in op-tui or map.html after a stop = its last retained presence beat / last digest, not a program
  (GUIDE §3, the note after 3.3): the top bars show the version that runs (`usv_ui v0.1.19`, `op-tui v0.1.19`);
  `stop_usv.sh --wipe` clears the retained beats.

### 7.1 Field lessons - why the odd code is there (v0.1.67, B13: the UAV team's MERGE_GUIDE §7 habit; keep it when you "clean up")

| symptom seen | the cause | what the code does (where) |
|---|---|---|
| a duplicate RxCommand acted on twice / a real one dropped | QoS 1 redelivers; rc-test's server and RoboNation's test_client both number commands from 1 | a duplicate = the same seq AND kind AND send time (`ocs_app.on_rc_command`, v0.1.23) |
| the AUTO guard passed on a relative-only position | 0x100 is PRED_POS_HORIZ_REL, not ABS (0x200) | the EKF bits from pymavlink's enum (`companion/link.py`, v0.1.54.1) |
| a far dark buoy circled as the entry | far frames miscolour; a few read "blue, flashing" | the locked entry is watched from near and rejected if never blue (`t1_safe_passage.watch_locked`, v0.1.54) |
| the fault test's GPS loss named 3.8 s late | EKF_BAD went hard at once, then the boat sat silent in a soft state | EKF_BAD soft for `guards.ekf_bad_hard_s` with its own warning row (`health.py`, `bench_tui`, v0.1.59) |
| a page drew no card at all | the nav is built before the first state; `S.x` read with `S` null | every early read guarded `((S&&S.x)||{})`; no-jump catches it (`ocs_web.html`, v0.1.65) |
| a tmux window's command cut off, the program never started | the tty's canonical input is 1 KB; long paths overflow it | the command goes into a small script file, the window runs `bash file.sh` (`run_uav.sh`, v0.1.64) |
| the UAV listener's fence restore never logged | C-c killed `tee` with the listener; the fake FCU died at the same moment | `tee -i`; the fake copter ignores SIGINT and goes with the session (`run_uav.sh`, v0.1.64) |
| the boat taken as "the UAV" in a test profile | `yaml.safe_dump` sorts keys: SGIT-53577 before USV1 | `ocs_app.primary_vid` = the first surface vehicle, never the first key (v0.1.64) |
| a preset value refused as "one of off, on" | YAML reads a bare `off` as False | quoted `"off"` in `presets.yaml` (v0.1.65); the fake UAV maps `light: off` (False) to off (v0.1.66) |
| a declaration silently ignored by RoboCommand | it never answers a rejected declaration | the UAV's fence and ceiling are checked by their `geofence_problems()` before sending; refused = not sent (`ocs_app.uav_declaration_content`, v0.1.64) |
| one wrong-colour UAV message recoloured the entry | the last UAV sighting overwrote the colour | the UAV's colour is the majority of its sightings (`PerceptionMap._uav_vote`, v0.1.66) |
| two UAV buoys 0.5 m apart became one | UAV-only items merged by nearness | an item with an id matches only that id (`PerceptionMap._uav_only`, v0.1.66) |
| a mosquitto bridge line mapped nothing | the topic pattern is relative to the prefixes | `topic perception/# in 0 local/usv/uav/ uav/` (not `uav/perception/#`) (v0.1.66) |
| the UAV's sim-test "re-declared" never happened, in their own stack too | the archive's fake copter reports FENCE_ALT_MAX 100 unless the scenario says alt20 | ours pinned to what their bridge actually does; the _alt20 variants carry the positive path (`test_v0164_scenarios.py`) |

## 8. Where the project context lives

Project docs (Claude project "SIT ASSIST"): `claude/ARCHITECTURE_global_v3_v0.1.md`, `claude/MODULES_global_v3_v0.1.md`,
`claude/optiprimex_bench_v0.1.6.md` (the bench guide), `claude/usv_kit_v0.1.19.md` (deploy / usv_ui / cv kits, this tree's
history per version; `pool_test4_kit_*` before), `claude/miro_board_global_v3.md`,
`claude/robocommand_handover_2026-10-25.md`, the RobotX 2026 handbook. Mac: `USV/test_scripts/global_v3/docs/`,
`global_v3/orin_kit/` (Orin bring-up, `apply_params.py`, firmware), `global_v3/actual_buoy/` (the student's CV, AGX).
