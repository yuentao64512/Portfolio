# RSE4207 normal-camera fall and action workspace

## Current trained model

The five-class model requested for this workspace is in `models/actions.pkl`.
It was trained from the local GMDCSA24 pose sequences plus the six recordings
`IMG_7562.MOV` through `IMG_7567.MOV`. See `TRAINING_REPORT.md` for the
dataset counts, evaluation, limitations, and reproducibility commands.

Run it on a video or camera with:

```bash
./run_trained.sh --source IMG_7565.MOV --headless
./run_trained.sh --source 0 --lying-seconds 10
```

The five action labels are `falling_down`, `lying_down`, `walking`, `standing`,
and `sitting`. `data/user_segments.csv` contains the editable temporal labels.

The older progressive stages below are inference configurations, not separate
training datasets. After continuous `lying_down`, the display changes to
`falling_down`; that is a configurable safety alert, not a medical diagnosis.

## Progressive stages

| Stage | Pose backbone | People | Display | Purpose |
|---|---|---:|---|---|
| 1 | Stock YOLO11n-pose PyTorch, 640 px | 1 | Full 17-point pose | Deliberately heavier/laggier baseline |
| 2 | Reduced custom backbone, TensorRT FP16, 384 px | 1 | 12 body points | Faster single-person version |
| 3 | Same custom TensorRT model | 4 | 12 body points | Multi-person tracking |
| 4 | Same custom TensorRT model | 8 | Full 17-point pose | Most complete version |

The action classifier always uses the same 12 body keypoints so backbone/person-count comparisons remain meaningful. “Full pose” changes visualization, not the trained classifier input.

## Run

```bash
cd /home/jetson/rse4207_ws
./run.sh 1 --source 0 --lying-seconds 10
./run.sh 2 --source 0 --lying-seconds 10
./run.sh 3 --source 0 --lying-seconds 10
./run.sh 4 --source 0 --lying-seconds 10
```

Stage 4 is the default:

```bash
./run.sh --source 0 --lying-seconds 10
```

Only run one live stage at a time because all stages use the same camera. Press `Q` to close the preview. Over SSH, add `--headless` and stop with Ctrl+C.

Recorded-video example:

```bash
./run.sh 4 --source /absolute/path/video.mp4 --headless \
  --output reports/result.mp4 --stats reports/result.json
```

Validate the installation without opening a camera:

```bash
./run.sh test
```

Logs created by this workspace go into `rse4207_ws/reports`. The legacy stage
scripts require Jetson-specific model assets; use `run_trained.sh` on this
machine for the newly trained model.
