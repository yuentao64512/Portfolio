# Hospital fall detection

Hospital CCTV fall-detection research prototype for multi-person scenes. It combines pose estimation, temporal action classification, per-person tracking, bed/floor context regions, and fall-to-lying alert logic.

## Included

- Source code for training, live/video inference, multi-person demos, scene stitching, and hospital context handling.
- Example configuration files and launch scripts.
- Lightweight trained models and portable YOLO model files.
- Evaluation reports, graphs, contact sheets, synthetic test clips, and prediction outputs.
- The detailed project report and presentation artifact.

Start with [`README_FALL_ACTION_PROJECT.md`](README_FALL_ACTION_PROJECT.md) and [`docs/Hospital_Fall_Detection_Project_Detailed_Report.md`](docs/Hospital_Fall_Detection_Project_Detailed_Report.md). The complete stitched demo can be run with `RUN_COMPLETE_HOSPITAL_DEMO.sh`.

## Scope and safety

This is a research prototype, not a medical device or a replacement for clinical observation. Synthetic and recorded-video demonstrations do not prove live hospital performance. Deployment requires consent, privacy controls, site-specific validation, camera calibration, occlusion testing, and an operational response process.

