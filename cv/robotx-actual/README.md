# cv/ — the perception programs, one folder per capability (AGX Orin)

```
cv/
├── buoys/     buoy observer (students' actual_buoy_observer.py, verbatim) + their multi-buoy UDP sender + setup check; models/, runs/
├── docking/   red-dock observer: perception + geolocation only (dock_observer.py, derived from their dock mapper); models/, runs/
├── cannon/    water-cannon aiming link (ours): reads docking's status file -> HMAC dock_cannon.v1 packets to the Pi, unchanged format
├── publish/   cv_publish.py: buoys' and docking's status files -> MQTT Observations on the NX boat broker (usv_ui 'cv on', the map)
├── _env.sh    interpreter resolver (CV_VENV / cv/.venv / the students' venv), network.env reader
├── sync_models.sh   AGX: copy the model files from the students' folders into cv/*/models (not in the archive)
├── requirements-agx.txt   their venv manifest = the recipe for our own venv (gated step)
└── PROVENANCE.md    source, sha256 and the change for every snapshot
```

Why: the students' folders on the AGX keep changing and point at each other with absolute paths; this tree is ours, versioned
with the stack, runs from `~/Desktop/dm3/MASTER_SCRIPTS/cv`, writes only under `cv/*/runs`, and each capability starts and stops
on its own. Nothing in `actual_buoy/` or `docking_cv/` is modified; the HMAC key stays where it is (read by path).

## Run (AGX)

```bash
cd ~/Desktop/dm3/MASTER_SCRIPTS/cv
./sync_models.sh                                    # once: models into cv/*/models (sha256 printed)
buoys/run.sh --verify                               # CUDA / ZED / TensorRT / model import check
buoys/run.sh --self-test ; docking/run.sh --self-test ; cannon/run.sh --self-test     # no camera, no network
docking/run.sh --smoke --source test.jpg            # the TensorRT dock detector alone
buoys/run.sh --headless                             # the buoy observer -> buoys/runs/<stamp>/latest_status.json
docking/run.sh --headless                           # the dock observer -> docking/runs/latest/latest_status.json
cannon/run.sh --dry-run                             # what would go to the Pi (no key needed); --synthetic-send = receiver test; no flag = live
../scripts/run_agx.sh                               # cv_publish -> NX broker (reads buoys/runs and docking/runs/latest); tmux 'agx': window cv = its log, window cli = this shell
```
`tmux attach -t agx` opens on the `cli` window — a shell in this folder with the venv and these commands as hints; start the
observers there (`Ctrl-b c` for a second shell). `cv_publish up …` printed by `run_agx.sh` is its report, not a command.
Each `run.sh` takes the program's own arguments after its flags (`--mavlink`, `--frames N`, `--conf`, `--resolution` …). The FCU
telemetry (`--mavlink`, default `udpin:0.0.0.0:14550`) is the NX's telemetry-only forward — read-only, as before.

## Interpreter

`_env.sh` picks, in order: `CV_VENV` (network.env), `cv/.venv`, the students' `AGX_BUOY_VENV` (`/home/optiprime-agxorin/actual_buoy/.venv`,
also what their docking launcher uses), `python3`. Until our own venv exists the first two are absent and theirs is used read-only.

## Own venv (gated step - do after the integration runs with their venv)

`requirements-agx.txt` is the manifest of their venv and the recipe: `python3.10 -m venv --system-site-packages cv/.venv`, torch/torchvision
from the NVIDIA Jetson index (not PyPI), `pyzed` from the ZED SDK's `get_python_api.py`, the pinned rest, `paho-mqtt`. Then `buoys/run.sh
--verify` and the three `--self-test`s, and the first real run pays the one-time neural-depth engine optimisation (minutes). Set
`CV_VENV` in `network.env` or just leave `cv/.venv` in place - `_env.sh` prefers it.
