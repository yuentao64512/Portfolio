# usv_ui — the pool kit (pool_test3 + the global_v3 stack)

| file | what |
|---|---|
| `usv_ui.py` | the operator's pool tool: everything pool_test3 did (mode-20 bursts, swaycal, save/goto, path/pathto, swayto, on-demand logging, MAVLink inspector) **plus** the FCU link share for the mission manager, MQTT on the boat broker, `cv on\|off\|status\|clear` (AGX detections → perception map → MAP tab + `telemetry/usv/map`), `handover` / `takeover` (who commands the FCU; you stay in usv_ui either way), `fcu fake | real | status` (swap the link live: the bench plant on udp 14550 — started here if none answers — or the Pixhawk), `reset` / `step` (vehicle RESET = abort + DISABLE_AUTO + run context cleared; STEP advances TEST_MANUAL), `share`. MAVLink 200/190. Since v0.1.18 its MAIN is the same skeleton as op-tui's (ACTION/LINKS \| VEHICLE, MISSION \| WHERE WE ARE, FLOW, THRUSTERS/RC \| NETWORK \| WARNINGS, EVENTS \| CONSOLE \| COMMANDS) with a MISSION tab; it watches the OCS link (bridge state + `ocs/state` bridged in) and says when it is down; below 120x44 it draws the compact v0.1.17 MAIN. |
| `cv_publish.py` | runs on the AGX next to the student's programs: publishes the buoy observer's `latest_status.json` frames (and, with `--dock-output`, the dock mapper's red-dock target) as Observations; `--demo` (synthetic buoy), `--replay CSV` |
| `map` (command) | the browser map is `../optiprimex/tools/map.html` (no internet needed): boat, trail, entities, gates, dock line, path, points, truth (bench). `map` in usv_ui or op-tui opens it; `run_ocs.sh` / `run_bench.sh` open it at start on the Mac |
| `../network.env` + `../network.ts.env` / `../network.lan.env` | ports and FCU_LINK in the base file, the addresses (OCS_HOST, BOAT_HOST, AGX_HOST) in the profile picked by `NET_PROFILE` or `--ts` / `--lan` - the launchers source them, cv_publish reads them |
| `config/usv_bench_share.yaml` | the laptop rehearsal profile (mission manager through usv_ui's share) - `make rehearsal` |
| `../scripts/`, `../profiles/pool/` | the launchers (`run_usv.sh` / `run_ocs.sh` / `run_agx.sh`, `setup_*.sh`, `stop_*.sh`) and the host profiles (`usv_pool.yaml`, `usv_lab.yaml`, `ocs_pool.yaml`, broker templates) - this folder is only the boat console |
| the trial | the legs are `../GUIDE.md` §7; `../HANDOFF.md` has the context, rules, test ladder and troubleshooting |

```
make test        # ../tests/test_usv_ui.py: share, handover/takeover, cv map (needs ../.venv + mosquitto)
make rehearsal   # the laptop bench with the mission manager through usv_ui's share + usv_ui in tmux 'optiprimex'
../.venv/bin/python usv_ui.py --help
```
Typed commands and screens are documented in the file header and the HELP tab; `usage:` above the prompt shows the
syntax of what you are typing, `example:` a typical call; `last:` shows the result of the previous command.
