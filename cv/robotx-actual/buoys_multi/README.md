# cv/buoys_multi — OUR buoy observer (every buoy, a colour each) + the ZED recorder

`run.sh --headless` runs `multi_buoy_observer.py`: the students' measuring functions imported read-only from `cv/buoys/`, every
buoy of a frame with a colour, output into `cv/buoys/runs/<stamp>/` where `cv_publish` looks (see `cv/PROVENANCE.md`).
`run.sh --self-test` = no camera. `multi_core.py` / `rec_core.py` are the pure parts (tested on the Mac without pyzed).

## Recording clips for CV_TRAIN (v0.1.40, ROADMAP WP9b)

The observer holds the ZED, so it records: `scripts/agx_record.sh start LABEL [--max-min 10]` (on the AGX, or the page's
**⚙ SETTINGS → CAMERA → RECORDER** card over ssh) restarts it with `--record ~/Desktop/dm3/recordings --label LABEL`. It keeps
detecting (cv_publish and the map are fed as usual) and writes `~/Desktop/dm3/recordings/<YYYYmmdd_HHMMSS>_<LABEL>/`:
`cam.svo2` (the ZED SDK's H.264 recording: both images + IMU, at the loop's frame rate), `frames.jsonl` (per frame: UTC, the
observer's frame and the SVO's frame number, the ZED timestamp, the boat's lat / lon / heading from the Pixhawk telemetry, the buoys
with colours, ranges, bearings and boxes) and `session.json` (label, start / end, kit / ZED SDK / camera / model versions, the options,
why it stopped). The recording stops by itself at `--max-min` or below 5 GB free; `agx_record.sh status` (the card's *status*) says
which. **What to label**: `day<N>_<what>_<n>` in letters, digits, `_` and `-` — the task or scene (`t1_gate`, `t2_channel`,
`dock`, `cannon`, `course_run1`), and the light when it matters (`am`, `noon_glare`, `dusk`), e.g. `day1_t1_gate_am`,
`day2_dock_noon_glare`, `day3_course_run2`. **Where the clips go**: the AGX's `~/Desktop/dm3/recordings/` (outside the kit tree),
then each evening `scripts/agx_pull_clips.sh` on the Mac rsyncs them over AGX_SSH into
`~/Documents/BH1/RobotX/recordings/<YYYY-MM-DD>/` with a `SHA256SUMS` per session — a staging folder **outside** CV_TRAIN, which
imports from there itself. Replay a clip offline: `run.sh --headless --svo <session>/cam.svo2` (the same loop, the telemetry from the
`frames.jsonl` beside it; output into `<session>/replay/<stamp>/`, never where cv_publish looks), then
`python3 rec_core.py compare <session>/frames.jsonl <session>/replay/<stamp>/frames.jsonl` (same buoys? the share of frames
with the same count, colours, ranges within 0.5 m / 2°; the replay decodes H.264 images, so expect a few frames to differ).

## The event days — the recording checklist

| when | what |
|---|---|
| before leaving (the bench) | the 60 s rehearsal clip recorded, replayed with `--svo`, compared, pulled home; **write its MB per minute here**: ____ MB/min (until measured: an estimate of 30–120 MB/min at hd2k, so ~10 min ≈ 0.3–1.2 GB) |
| each morning | `agx_record.sh status` (page: RECORDER → status): recorder down, the free disk ≥ 15 GB; the AGX has ~26 GB, the recorder stops at 5 GB — the day's budget is the free disk − 5 GB |
| what to record | every practice slot on the water: the Task 1 gate and the channel from 25 m in and back, the dock faces from several angles and ranges, the cannon targets, each buoy colour near and far (5–30 m, ±30°), the light changing (morning, noon glare, dusk), negatives (open water, the shore, other boats) — 5–10 min each, `--max-min 10` |
| judged runs | record them too (label `course_runN`); start before `declare`, stop after `end_run` — not during a task (the card refuses stop / replace while a task runs: the recorder is the map's buoy observer) |
| after each clip | status: `stopped: …` with frames > 0 and a size; a `stopped: disk_low` means stop recording for the day |
| each evening | Mac: `scripts/agx_pull_clips.sh --list`, then `scripts/agx_pull_clips.sh` (refuses while RECORDING); check one `session.json` and the sizes in `~/Documents/BH1/RobotX/recordings/<date>/`; only then free the AGX (`rm -rf ~/Desktop/dm3/recordings/<session>` — printed by the script); tell CV_TRAIN the folder |
| never | copy clips into CV_TRAIN by hand, record into the kit tree, leave the AGX under 5 GB overnight |

## boundary-v14 (v0.1.87)

`run.sh --headless --detector boundary-v14` loads the colleague's boundary-v14 read-only. It reads
`~/buoy_robotx/runtime/boundary_v14_20261111/models/boundary_v14_best.engine` on the AGX (the .pt elsewhere, or `CV_MODEL_V14=<file>`).
- The class list is checked at start.
- **Buoys:** red, green and blue take the model's colour (`--colour auto` at `--led-conf`, else our HSV vote). An unlit boundary buoy
  is "black" in `--colour model`.
- **Obstacles:** the `--obstacle-classes` boxes (default `obstacle`; `obstacle,dock` or `none`) at `--obstacle-conf` (0.35) are
  measured over the ZED depth (the box's inner ellipse) and appended to the guard's `far` list. The boat rings them like the depth
  clusters (3 sightings; none where a mapped buoy explains it). They are also in `latest_status.json` `obstacles` and in frames.jsonl.
  With `--guard off` they do not reach the boat.

The review on our 11 Nov footage is in `logs/cv_review/boundary_v14_2026-11-12/`: a buoy in 4 of 344 frames, and the numbered
markers come out as "dock". Keep `default` for runs.

## six-class (v0.1.87) and switching

`--detector six-class` runs the colleague's 11 Nov model (`~/buoy_robotx/runtime/six_class_20261111/models/`, the .pt because there is
no engine yet, or `CV_MODEL_SIX`). Its classes are buoy_red, buoy_green, buoy_blue, buoy (coloured by our HSV vote), dock and platform.
- `agx_observer.sh start --detector X` while another detector runs = a switch: our tmux window is stopped and X is started.
- The observer writes `detector` / `model` into latest_status.json. cv_publish reports them (perception health `observer_detector`),
  and the page's OBSERVER tooltip shows them.

## The horizon cut and the LEDs (v0.1.91, `led_core.py`)

- **Horizon** (`--horizon frac|off`, `--horizon-frac 0.45`): a box whose bottom edge, the waterline, is above 0.45 × the image height
  is dropped, for every detector. On 12 Nov it dropped exactly the 6 building / sky boxes and nothing with depth.
- **LED colour without a model** (`--led on|off`): in the band under each buoy's hat (the top 5–40 % of its box, the middle 60 % of
  its width), the pixels with V ≥ 215 and S ≥ 120 are binned by hue into red / green / blue. On = one colour holds ≥ 6 px and ≥ 2 %
  of the band. On 12 Nov's 629 detections it fires once: the blue lantern on marker 1 at 7 m.
- **The blink memory, no map:** image tracks (the nearest box centre of the last frame within 60 px, a similar size) keep 4 s of
  on / off samples. The result is steady (on ≥ 85 %), flashing (on and off, ≥ 2 changes), off, or unknown (< 6 samples).
  Blue steady = Task 1's exit; blue flashing = its entry. The frame rate must be ≥ 2× the blink rate.
- Each buoy record carries `led`: `{track, pattern, colour, on_frac, changes, n, on, now}` (latest_status.json, frames.jsonl).
  cv_publish and the boat's map do not read it yet.
