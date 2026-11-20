# uav/ - a read-only snapshot of the UAV team's Orin-side files (v0.1.64, D3 of docs/OVERNIGHT_2026-11-07_UAV.md)

- **Source**: `drone/robotx-uav-robocommand 2/` (the colleague's v2 reference archive, the code that cleared RoboCommand compliance on site,
  received 2026-11-07; its `MERGE_GUIDE.md` is the reference). Copied 2026-11-08 by the overnight session, **unmodified**.
- **Copied**: `geofence-tools/`, `sim-tests/`, `drone-heartbeat-gui/`, `lawnmower-mission/`, and his `MANIFEST.sha256`.
  **Not copied**: `buoy-detection/` (weights, the drone team's CV), `robocommand/` (RoboNation's repo; ours is vendored at
  `optiprimex/ocs/rc_pb/`), `camera-capture-gui/` (listed in his manifest, not used here), the PDF and the guide (read them in `drone/`).
- **Rules**: nothing of ours is written here except this file. `sha256sum -c --ignore-missing MANIFEST.sha256` must be clean (36 files;
  `sim-tests/` is not in his manifest - the table below covers every file). The simulator runs the listener and the fake copter **from
  here**, with the venv's pymavlink (`scripts/run_sim.sh --uav`); their state files go to `logs/uav/`, never into this folder.
- **Our side** of his §5 wire: `optiprimex/uav/adapter.py` (his `ReliableSender` ported verbatim) and `optiprimex/ocs/geofence.py`
  (`geofence_problems()` / `course_fence()` ported verbatim). Tests: `tests/test_v0164.py`, `tests/test_v0164_scenarios.py`.

| file | sha256 |
|---|---|
| `MANIFEST.sha256` | `a9128d37f1d093fa83d5a45bd2974b5469a70b75faec2a96e6839c29cf142f81` |
| `drone-heartbeat-gui/README.md` | `a1a745dd38ba91b732aecd806647fd886cbd5fddae269853df71f1ad16e62348` |
| `drone-heartbeat-gui/fcu_clear.py` | `752852cb758bbd109a579b35562541d6fa00d2ba2a89a5d2de72fd2759ac32f1` |
| `drone-heartbeat-gui/heartbeat_link.py` | `fe5400ddbe65536934d033ff929c38ac5e4b3be74eb8d180de336a4a0dea217c` |
| `drone-heartbeat-gui/heartbeat_monitor.py` | `e0ff7e8797d97a3dde076af6b9c7f5fa71cc7d6c10a9b4e6f46f525c71274085` |
| `drone-heartbeat-gui/requirements.txt` | `b7e6bdd9ef7647d957dfc32dfc868fe8df6aa0a3c8ee5f23a18894524e9cca35` |
| `geofence-tools/README.md` | `718ad99e95f5f49c3987a62b08c3b0abba2029ce44fb07bb2111d90e07172ee7` |
| `geofence-tools/course_fence.py` | `6040fe261336f08e753e0799aceef44ce8bb26ad0a60381f769b442e2868c433` |
| `geofence-tools/fence_csv.py` | `50cec27a6b96f5d0ba4f479f5ef6403a09d851598e41755023b82a4f6e364e05` |
| `geofence-tools/geofence_publisher.py` | `d979fe686c426d00e09b96773f99d4769bc7d6ddf7c164a580274a403cd58969` |
| `geofence-tools/geofence_udp_listener.py` | `60e19950cf497d6b61c4b20bf0d6e78bce158183b4f9b928a49d53b8d2b4a86a` |
| `geofence-tools/geofence_udp_publisher.py` | `04a611f1b33799fc2ea5cb8d047909f8510bbc7f25b90068c814e3c208b380ea` |
| `geofence-tools/mavlink_param_link.py` | `55f071d8283d1d9f9f2b534df139b280c12190c644ab7ba503b9f91cf6f8ded3` |
| `geofence-tools/mission_fence.csv` | `28ce17b6c5cfbcce3cc4c1b95577c3b528b03219810167dea3277dbaa405bc6d` |
| `geofence-tools/param_listener.py` | `ddabb1b43170edc79d6d2e7686eca6b8085ad7a4ba4cd3b4ccd2728486628042` |
| `geofence-tools/param_set.py` | `a28492c35445b494909dc2f8d2f1b4d57f9a648cc5e3fc7b9e47b62d6618a7a6` |
| `geofence-tools/pdd_fence_20m.csv` | `cbcb7efacf5964b18ff0e9c969fccc0224dbf52605d7031263bff4e7f1cfc8a4` |
| `geofence-tools/preset_fence_example.csv` | `62a6eaf89a9fd8901632015454b57476a581bc6231ff29f1d38589d2bb387e8a` |
| `geofence-tools/refresh_Rxcourse.py` | `a472cf48578d86cbeeef48bd5ed05036cf72156d79b9495e15b890cc4715ddf8` |
| `geofence-tools/requirements.txt` | `32b365d41a5c379b2aec05e61b96b863f2395c84b6f7a6ba880c14050f9e5e89` |
| `geofence-tools/robocommand_bridge.py` | `3f93265929bc7bef30a216735d3362a3f431a7e86f34dd7f9ca27715069ca124` |
| `geofence-tools/robocommand_handler.py` | `6d85dc297bf4b679173c2be332c102707a0c52e4e8c5df6bccd8dd1fe4a23e9e` |
| `geofence-tools/robocommand_reports.py` | `008677a821df2c2e223a86d4707e6beef9a19cb20d95e40f1b1c454da1c0d7a4` |
| `geofence-tools/run_declaration.py` | `2dd907a52856dc831545600e83e0c8e537495f66ed059fafc6bab6288f2aa0c2` |
| `geofence-tools/start_bridge.bat` | `5bcc03d023e76288d5960123964272654170073f37939015102b076ee41c7ba7` |
| `geofence-tools/uav_telemetry.py` | `be0d553a908e085fbbcdca4ffdd29973a9f06c7cae5735e5f9412a3979894c42` |
| `geofence-tools/your_fence.csv` | `1d5a1dbdce0559213eb678c349485b09b0f1da8e6e8e3101c73cab079b209e1b` |
| `lawnmower-mission/README.md` | `2919d518982b9de1b458e158c2aa2bb8b4e863de384ce13f1eb13a86c7c59c25` |
| `lawnmower-mission/lawnmower_mission.py` | `ab73a517c29a782793d6d05686d63f6921b1bb5a9dc7db5b728bf04b04ca115a` |
| `lawnmower-mission/lawnmower_planner.py` | `4013705ec0146e6efcbb485dbe8228c7306c9e51942bf028cea41078a5f2785d` |
| `lawnmower-mission/mission2.py` | `2b6e0ba982353fa682217831e15b62c80cb6b9860203e5f26db2b99930b03c8f` |
| `lawnmower-mission/mission3.py` | `e77228033ff22bb03e69dfe409879893b2ce7d02f1b578c18f2404238e72584b` |
| `lawnmower-mission/mission_link.py` | `a8caf5ab511ba47672412e105f91053deff276f268b659313eac483b9118049b` |
| `lawnmower-mission/requirements.txt` | `e47efcf4a1419513e4ae6d0a344b58c4f55f0e33a6a832bc202b60164fe629d2` |
| `lawnmower-mission/runstart_mission.py` | `6f0f11072bfe47adc1c68039d5fe58700eb80eced0c279dcb35363a07271b0f8` |
| `lawnmower-mission/task4_mission.py` | `e5a714022b00a661d7e4df9b02ba934f9366da38280c1c9499476e76c5e5f3a7` |
| `lawnmower-mission/task4_route.py` | `643acc93dd4e5328207dbbb8e289e019450d0f8fd52c21e15aca7a32c30ce163` |
| `sim-tests/broker.py` | `cac62d5f8f387c8cb90a14cc8f0dfa0edab90ff6d18d91416e81d98c9f87bcc0` |
| `sim-tests/fake_copter.py` | `752a7cf0681bb88fe81934ef4c160a27146c929a2bb2ff6bf3874eb9e7cac363` |
| `sim-tests/test_mp_fence.py` | `b3fbdb7be7fce49a97e28883d295ce3027e73b30006395990b64d586df7e0edd` |
