# cv/buoys/two_stage — the colleague's two-stage buoy / dock recognizer (snapshot, 2026-11-10)

Source: `optiprime-agxorin@192.168.1.99:~/buoy_robotx/runtime/` (the colleague's; read-only for us), rsynced 2026-11-10 ≈09:35 SGT
to the Mac `../ws/pull_2026-11-10/buoy_robotx/` (42 files, sha256 identical to the AGX: `SHA256_AGX.txt` there), then copied here
**verbatim** (files `a-w`). Nothing in `~/buoy_robotx` changed. Their own hand-over: `~/Downloads/BUOY_PROJECT_HANDOVER.md` (Mac).

| file | sha256 | bytes | change in our copy |
|---|---|---|---|
| two_stage_buoy_recognizer.py | 18f310e9b446ff58acb119bd27a1fda05af552f152198905180b8f33a55bb7fe | 16684 | none |
| recognise_docking.py | 142ede14abe5508d7e252aab6883a352ef78f563cc471ed8456227e8984f7fa1 | 9236 | none |
| predict_buoys_zed.py (new on 10 Nov 09:34) | 38fb197e7699f503be279a4c464a7d1dbac47fe80e5920bd09a0ab70b8b66a28 | 4082 | none |
| run_buoy_two_stage.sh | 99de0b8cf7c2a6a3968107a96b2df5c3cf168c5857b2d99c1b72fb2d5d5d8c2f | 299 | none (calls the students' venv by absolute path) |
| run_dock_recognition.sh | 4528d139769dfd5e444f0e50a1fabca80be8ab94ac2bc1d42c9dab446a0188fe | 291 | none |
| run_predict_buoys_zed.sh | 7e477ba8307655f1b293f084829c179638153bfcc3f375ac37e5b0da13cbcabe | 292 | none |
| export_fp16_engines.sh | 3a494099e86077af7458cd1bf34e1a4033321ed2aa37b81da417738e409ce670 | 745 | none |
| status_agx.sh | 9832ff4a1591fa8769c388f865d68f6d02f972b4fb39f07bfd2b9ffd29b6e6ba | 1370 | none |
| README_AGX.md | 4463373ee9168d5a8e965b61b5b95d1253f3596b97c576c1303535e9065d3de5 | 1579 | none |
| tensorrt_engine_sha256.txt | 1b09b92b8afc1454a4ad3daa28e6fe6a37187bd6efe6cf738ae34fd6d19659c8 | 274 | none |

**Models — not copied into the tree** (used read-only where they are, `~/buoy_robotx/runtime/models/` on the AGX; a Mac copy for
offline work in `../ws/pull_2026-11-10/buoy_robotx/runtime/models/`):

| model | sha256 (.pt) | classes |
|---|---|---|
| model_a_best (YOLO11n detect, 1280) | e8b215194c92ad629c1197c168d8a389882f2262d3fc996c4e50f36c658daa8f | buoy_led_red, buoy_led_green, buoy_no_led, dock, home, obstacle |
| model_b_best (YOLO11m detect, on crops) | 8e1cb5acafe6c1e710c092a988751b7f2a1f848bc902761766e3c38b0765e2b9 | circle, diamond, digit_0..9 |

The engines (`model_{a,b}_best.engine`, built on this AGX 9 Nov) are listed in `tensorrt_engine_sha256.txt`.

**Ours in this folder (not the colleague's):** `score_sim.py` (v0.1.78.18) — score any YOLO model on a CV_TRAIN test split with a
class map (the detect-capable sibling of `../CV_TRAIN/eval/confuse.py`).

Review notes (2026-11-10): image boxes only — no range, no bearing, no `latest_status.json`; camera through `cv2.VideoCapture`
(two_stage, dock) or pyzed with `DEPTH_MODE.NONE` (predict_buoys_zed, preview only); a 30-frame majority vote per ByteTrack id
(LED, shape, digit) - it smooths flashing vs steady away; no blue class. The adapter that brings Model A into our contract is
`cv/buoys_multi` `--detector model-a` (v0.1.78.18).
