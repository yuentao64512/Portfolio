# Fall Action Project

Remote project path:

```bash
/home/robotx/fall_action_project
```

## Dataset

Downloaded dataset:

```bash
data/GMDCSA24
```

This dataset contains 160 MP4 videos with CSV segment labels. The manifest builder maps:

- `Falling` / `Fall` -> `falling_down`
- `Walking` -> `walking`
- `Standing` -> `standing_still`
- `Sitting` -> `sitting_kneeling`
- `Sleeping` -> `lying_down`

It also derives `lying_down` from the final post-fall part of longer fall segments, because the drone logic needs:

```text
fall transition -> person remains down -> possible faint/injury alert
```

## Main Commands

Activate the environment:

```bash
cd /home/robotx/fall_action_project
source .venv/bin/activate
```

Rebuild the segment manifest:

```bash
python src/prepare_gmdcsa24.py \
  --dataset data/GMDCSA24 \
  --output outputs/gmdcsa24_segments.csv
```

Extract YOLO pose sequences:

```bash
python src/extract_pose_sequences.py \
  --manifest outputs/gmdcsa24_segments.csv \
  --output outputs/pose_sequences_gmdcsa24.npz \
  --device 0 \
  --sample-fps 5 \
  --seq-len 32 \
  --imgsz 640
```

Train the baseline classifier:

```bash
python src/train_pose_classifier.py \
  --pose-npz outputs/pose_sequences_gmdcsa24.npz \
  --manifest outputs/gmdcsa24_segments.csv \
  --model-out models/gmdcsa24_pose_rf.pkl \
  --report-out outputs/gmdcsa24_pose_rf_report.json
```

Run a demo on one clip:

```bash
python src/run_video_demo.py \
  --video "data/GMDCSA24/Subject 1/Fall/05.mp4" \
  --classifier models/gmdcsa24_pose_rf.pkl \
  --lying-alert-sec 1.0
```

Run live camera detection:

```bash
python src/run_live_camera.py \
  --camera 0 \
  --classifier models/current_pose_rf.pkl \
  --lying-alert-sec 10
```

If no preview window is needed:

```bash
python src/run_live_camera.py \
  --camera 0 \
  --classifier models/current_pose_rf.pkl \
  --lying-alert-sec 10 \
  --headless
```

If camera `0` is not the correct camera, try:

```bash
python src/run_live_camera.py --camera 1 --classifier models/current_pose_rf.pkl
```

For a drone alert, use a longer timer:

```bash
--lying-alert-sec 10
```

or:

```bash
--lying-alert-sec 30
```

## Current Baseline

The first trained model is:

```bash
models/gmdcsa24_pose_rf.pkl
```

The current combined model, including the user's Aug 20 DJI top-down clips, is:

```bash
models/current_pose_rf.pkl
```

The evaluation report is:

```bash
outputs/gmdcsa24_pose_rf_report.json
```

This is a baseline trained on indoor fall videos, not a final drone-ready model. It is useful for testing the pipeline and fall-state logic. For real UAV use, add drone-angle data from Okutama-Action, UAV-Human, or your own drone footage.
