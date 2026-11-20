# cv/ — provenance of the snapshots (2026-10-29)

Sources are the students' folders as shared on the Mac (`USV/test_scripts/global_v3/actual_buoy`, `USV/test_scripts/CV/docking/docking_cv`),
which are the AGX deployments as of 2026-10-21 (buoys) / 2026-10-27 (docking). Their folders on the AGX stay untouched; a re-sync is
`sha256sum` of the source against this table, then a diff.

| file in cv/ | source | sha256 (source) | bytes | change in our copy |
|---|---|---|---|---|
| buoys/actual_buoy_observer.py | actual_buoy/actual_buoy_observer.py | e93f671a2c35d9c4… | 16754 | none (verbatim); paths given by run.sh (`--weights`, `--output`) |
| buoys/buoy_camera_sender.py | actual_buoy/buoy_camera_sender.py | 6f44fa4ef05d1284… | 5593 | 2 lines: preview JPEG written to our `runs/` instead of the students' folder |
| buoys/verify_agx_setup.py | actual_buoy/verify_agx_setup.py | 04dfb04bc1566756… | 1341 | model from `models/best.pt`; test image optional |
| buoys/README.student.md | actual_buoy/README.md | f64ea938828b8aec… | 3434 | none |
| buoys/models/best.pt | actual_buoy/best.pt | 351a554489e9afac… | 63427845 | not in the archive: `cv/sync_models.sh` on the AGX |
| docking/dock_observer.py | docking_cv/dock_mapper_sender.py | 1140a18aa937030a… | 22054 | DERIVED: cannon offsets, HMAC and UDP removed (→ cannon/); `--output`; status protocol `dock_observer.v1`; body_* frames; imports from `../buoys` |
| docking/run_docking.py | docking_cv/run_docking.py | fe18deac1daac9a5… | 1734 | none (verbatim); model path given by run.sh |
| docking/dock_config.json | docking_cv/dock_mapper_config.json | 9d141ffef0b087fb… | 701 | sections `water_cannon` and `network` moved to cannon/cannon_config.json |
| docking/CALIBRATION.student.md, README.student.md | docking_cv/CALIBRATION.md, README.md | 85ed739b233ec90a…, 22272e23301843fa… | 910, 1060 | none |
| docking/models/dock_partial_best.{pt,onnx,engine} | docking_cv/models/ | 7a230babd962d33d…, 205b0816d123c148…, 8e85a12a13062126… | 5.5 / 9.8 / 7.4 MB | not in the archive: `cv/sync_models.sh` (the .engine is built for this AGX) |
| cannon/cannon_sender.py | ours | — | — | NEW: the aiming link split out of dock_mapper_sender.py; `signed_packet` / `verify_packet` / synthetic body verbatim from it so the Pi receiver is unchanged |
| cannon/cannon_config.json | docking_cv/dock_mapper_config.json (`water_cannon`, `network`) | — | — | key_file path set to the AGX location; the key itself is never in this tree |
| publish/cv_publish.py | ours (was usv_ui/cv_publish.py) | — | — | moved |
| cannon/pi/color_cannon_receiver.py | the students' color_cannon_receiver.py (handed to Dmitrii; on the Mac in `MASTER_SCRIPTS/cannon/`, 2026-11-07) | cd9db52b2cdba9a43cd52124d90493a7225f92dd3a522db13a680d00dfc566a0 | 16255 | none (verbatim). It runs on the cannon's Pi; ours only runs it in the simulator (fake_cannon_pi, DRY_RUN, a copy in the run folder) and imports its functions in the tests. Its config `color_cannon_config.json` was not handed over: `pi/color_cannon_config.sim.json` is OURS (their keys, placeholder values, red) |
| cannon/color_dock_sender.py | the students' color_dock_sender.py (same hand-over) | 5e23b6994e69222587a8b60a9ebe09e5c167795c9c692c223133d788e2ceed5f | 14774 | none (verbatim); reference only, not run (it opens the ZED itself). Its per-frame steps are PORTED into `cannon/aim_cv.py` (green_mask → colour_mask with the colour a parameter, green_components, measure_mask, the nearest choice, the confidence rule) and its cannon_coordinates into `cannon/aim_core.py`. `color_dock_config.json` not handed over (camera_to_cannon → `cannon_config.json`, zeros) |

Not snapshotted on purpose: `actual_buoy_approach.py`, `guarded_docking.py` (motion controllers that command the FCU — in global_v3 the
mission manager is the only motion authority; they stay in the students' folder for their own tests), `buoy_link.key` (rule 5),
the students' `.venv` (used read-only through `network.env AGX_BUOY_VENV` until our own `cv/.venv` exists — recipe in `requirements-agx.txt`).

## Ours, beside the snapshots (v0.1.29.1)
`cv/buoys_multi/` (multi_buoy_observer.py, multi_core.py, self_test.py, run.sh) is OUR program, not a snapshot: it imports
`buoys/actual_buoy_observer.py` read-only (measure_mask, buoy_geolocation, open_zed, PixhawkTelemetry, discover_mavlink - none
changed) and runs the same loop, but keeps every measured buoy of a frame instead of the nearest (the snapshot's `min(candidates)`)
and adds a colour per buoy from its own pixels (the students' model has one class). Same output layout (runs/<stamp>/), plus a
`buoys` list in latest_status.json. Nothing in the snapshots or in the students' folders changed. v0.1.34 (WP10g): it passes
`max_depth_m` / `depth_max_m` = 25 (`--max-depth`; their default 20) to their measure_mask / open_zed through the functions' own
parameters, adds 0.85 × the buoy radius to the measured surface range (`surface_range_m` kept), and reports a confident detection that
measure_mask refused as a bearing (`range_m: null`, from the mask's centre and the ZED's own intrinsics). Their file is still unchanged.
v0.1.40 (WP9b): `rec_core.py` (ours, stdlib only) and `--record` / `--svo` in our observer: the ZED SDK's recording and an SVO
opened with the same depth settings as their open_zed (our own `open_camera` for the file - their open_zed still opens the live camera).

v0.1.61 (WP12-rest): `cannon/aim_core.py`, `cannon/aim_cv.py`, `cannon/pi/status_core.py`, `cannon/pi/cannon_status_pub.py` are OURS.
The observers (`buoys_multi/multi_buoy_observer.py`, `docking/dock_observer.py`, `sim_observer/sim_observer.py`) gained `--cannon COLOUR
[--cannon-out FILE]` (aim_cv on the frame and point cloud they already have); `docking/dock_observer.py` also lost a NameError (`destination`,
left from the 2026-10-29 split, raised at the first frame). The students' files above are unchanged.

`cv/models/<name>/` (v0.1.34.2, on the AGX only, not in the archive): copies of CV_TRAIN's published models (`cv/sync_cvtrain.sh`,
checked against the model's own sha256.txt); provenance, eval and the model card travel inside each folder (PROVENANCE.md, CARD.md).
