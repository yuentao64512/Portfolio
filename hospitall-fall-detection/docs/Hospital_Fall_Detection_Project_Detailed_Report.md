# Hospital CCTV Patient-Fall Detection Project

## Detailed implementation, evaluation, strengths, limitations and improvement report

**Project:** RSE4207 Team 3 — WardWatch  
**Audit date:** 17 November 2026  
**Primary deployed workspace:** `/home/robotx/fall_action_project` on `optiprime-5060`  
**Local development workspace:** `C:\Users\User\Documents\Codex\2026-11-16\g\work\fall_hospital`

---

## 1. Executive summary

The original project was a pose-based action classifier designed around hikers and aerial/drone footage. The work completed so far preserves that baseline while adding the foundations needed for a fixed hospital CCTV use case. The central change is that the system no longer decides whether a person has fallen from body pose alone. It now combines:

1. YOLO human-pose estimation;
2. fixed-camera bed and floor regions;
3. body-keypoint and bounding-box overlap with those regions;
4. temporal action classification; and
5. a state machine that considers whether the patient was recently on a bed or was observed falling before remaining down on the floor.

This design addresses the main hospital-specific problem: a patient lying horizontally on a bed must not be treated the same as a patient lying on the floor.

The software foundation is working. Eight current context/state unit tests pass on the RTX 5060 Ti computer, the source files compile, a synthetic bed-to-floor sequence produces the intended alert, and two negative controls produce no fall alert. However, this is still a prototype. The context evidence is synthetic, the camera regions have not been calibrated on a real hospital camera, the hospital recorded-video demo currently handles only the largest visible person, and no clinical or real-ward accuracy evaluation has been completed.

The correct project status is therefore:

> **A functioning context-aware proof of concept, suitable for pipeline development and controlled testing, but not yet validated for hospital deployment or patient-safety decisions.**

---

## 2. What was implemented

### 2.1 Existing pose-action baseline was preserved

The existing project already contained a lightweight action-classification pipeline:

- YOLO11n-Pose extracts 17 human keypoints.
- Keypoints are normalized by image width and height.
- Fixed-length pose sequences are converted into summary features, including mean position, positional variation, average velocity, maximum velocity and average keypoint confidence.
- A Random Forest classifier predicts actions such as `falling_down`, `lying_down`, `sitting_kneeling`, `standing_still` and `walking`.
- The legacy live detector can track multiple people and maintain a separate pose buffer and alert timer for each track.

The hospital work was added without deleting the original hiker/drone models or scripts. This is useful because the old classifier remains available as a comparison baseline while the hospital-specific context is developed.

### 2.2 Portable pose-model export was added

The source YOLO pose model is `yolo11n-pose.pt`. An export workflow was added for a 640-pixel ONNX model:

- `src/export_pose_models.py`
- `models/pose_exports/yolo11n-pose_640.onnx`
- `models/pose_exports/yolo11n-pose_640_metadata.json`

A Jetson build helper was also added:

- `src/build_jetson_pose_engine.sh`

The helper intentionally builds the TensorRT engine on the target Jetson. TensorRT engines depend on the target GPU, JetPack, CUDA and TensorRT versions, so an engine built on the RTX 5060 computer should not be copied directly to the Orin Nano.

### 2.3 Per-camera bed and floor ROI editor was created

`src/draw_context_rois.py` allows the operator to draw normalized polygons over a reference frame. Supported region types include:

- bed;
- floor;
- couch;
- chair;
- stretcher; and
- ignore/privacy region.

The coordinates are stored from 0 to 1 instead of as fixed pixels. This allows the same region file to scale with image resolution, provided the physical camera position and view have not changed.

The synthetic example contains two bed polygons and two floor polygons. It is a demonstration config only; it is not a calibration for a real ward camera.

### 2.4 Context geometry was implemented

`src/hospital_context.py` provides the geometry layer. For each detected person it calculates:

- the fraction of the person bounding box overlapping each region type;
- the fraction of selected body keypoints inside each region; and
- optional overlap with automatically detected furniture.

The keypoint calculation uses shoulders, hips, knees and ankles. Face and wrist keypoints are deliberately excluded because they can extend outside a bed even when the torso and lower body are still supported by it.

Location classification combines bounding-box and keypoint evidence:

```text
35% bounding-box overlap + 65% body-keypoint occupancy
```

The result is one of:

- `ON_BED`;
- `ON_FLOOR`;
- `AMBIGUOUS`; or
- `UNKNOWN`.

The logic includes a minimum score and a decision margin so that nearly equal bed/floor scores do not force an unreliable classification.

### 2.5 Optional furniture detection was added

`FurnitureDetector` provides an optional COCO-model fallback for `bed`, `couch` and `chair`. This can help during setup or when fixed polygons are unavailable.

For a fixed CCTV camera, manually calibrated regions remain preferable because generic COCO models may miss hospital beds, rails, stretchers or unusual viewing angles.

### 2.6 Context preview and tracking were added

`src/run_hospital_context_preview.py` combines:

- YOLO pose estimation;
- ByteTrack person IDs;
- fixed bed/floor polygons; and
- optional furniture detection.

This preview is intended to validate whether the regions line up with the camera and whether each tracked person receives sensible bed/floor overlap scores.

### 2.7 A hospital-specific state machine was implemented

`src/hospital_state.py` separates physical location from alert state. Its main outputs are:

- `PATIENT_ON_BED`;
- `PATIENT_ON_FLOOR`;
- `PATIENT_DOWN_ON_FLOOR`; and
- `FALL_ALERT_ON_FLOOR`.

An alert is not generated merely because the patient is horizontal. The patient must be down on the floor for a configured duration and there must also be recent evidence of either:

- a bed exit; or
- a `falling_down` action.

This transition-aware design is why a clip that begins with a person already on the floor can be labelled as a person down without automatically claiming that the camera witnessed a fall.

### 2.8 Recorded-video demo and audit output were created

`src/run_hospital_video_demo.py` processes a video and produces:

- an annotated MP4;
- a frame-level CSV;
- action classification and confidence;
- bed and floor scores;
- hospital state;
- floor-down duration; and
- alert state.

The CSV is valuable for debugging because the exact evidence behind each decision can be inspected rather than relying only on a coloured overlay.

### 2.9 Synthetic test scenes were created

Two groups of synthetic test material were produced locally.

#### Bed-to-floor sequence

The sequence shows:

1. patient lying on the bed;
2. patient sitting up;
3. patient moving toward the floor; and
4. patient lying on the floor.

It was used to verify the bed/floor context logic and alert transition.

#### Walking and falling sequence

Additional fixed-CCTV clips were created:

- walking followed by a stumble, descent and sustained floor pose;
- walking-only control; and
- floor-only-from-start control.

These are stored locally under `hospital_cctv_test_videos`. They broaden the test scenarios beyond bed exit, but they are still AI-generated and have not yet been copied into the deployed 5060 project or evaluated as a real-world dataset.

### 2.10 Presentation was converted to the hospital scenario

The original drone/hiker presentation was converted into a hospital-CCTV presentation named `WardWatch`. The updated deck explains:

- one fixed CCTV camera;
- bed/floor scene context;
- patient state transitions;
- synthetic pipeline results;
- limitations; and
- proposed next steps.

The final presentation is available as `Team3_RSE4207_WardWatch_CCTV.pptx` in the local workspace.

---

## 3. Current architecture

```text
Fixed CCTV frame
      |
      v
YOLO11n-Pose: person box + 17 keypoints
      |
      +--------------------+
      |                    |
      v                    v
Pose sequence          Camera context
Random Forest          Bed/floor polygons
action prediction      bbox + keypoint overlap
      |                    |
      +----------+---------+
                 |
                 v
         Hospital state machine
     location + action + duration + transition
                 |
                 v
  ON_BED / ON_FLOOR / DOWN_ON_FLOOR / FALL_ALERT
                 |
                 v
      Annotated video + frame-level CSV
```

The key architectural principle is that pose explains **what the body is doing**, while scene context explains **where the body is**.

---

## 4. Verified results

### 4.1 Software verification

The following command was run on the live 5060 project:

```bash
PYTHONPATH=src python -m unittest -v \
  src/test_hospital_context.py \
  src/test_hospital_state.py
```

Result: **8 of 8 tests passed**.

The tests cover:

- normalized region save/load;
- overlap scaling across resolutions;
- person-relative rectangle overlap;
- exclusion of face and wrists from the default context keypoints;
- distinction between bed and floor;
- horizontal-pose calculation;
- bed-to-floor alert after the timer; and
- no witnessed-fall alert when a person begins on the floor.

The main hospital source files also passed Python byte-code compilation.

### 4.2 Existing action-classifier evaluation

The current combined Random Forest report contains 406 pose sequences:

| Metric | Result |
|---|---:|
| Train sequences | 268 |
| Test sequences | 138 |
| Accuracy | 80.4% |
| Weighted F1 | 0.809 |
| Macro F1 | 0.655 |
| Falling-down F1 | 0.833 |
| Lying-down F1 | 0.839 |
| Walking F1 | 0.840 |

The split is group-based. In the combined evaluation, DJI plus Subjects 2 and 3 were used for training, while Subjects 1 and 4 were used for testing. This is better than randomly mixing frames from the same person across train and test.

However, the report is not a hospital result. It contains only five group identities in total, uses one train/test split, and has no running samples in the test set. Therefore the 80.4% figure must not be presented as hospital CCTV accuracy.

### 4.3 Synthetic context sequence

The synthetic bed-to-floor output contains 259 frames covering approximately 17.2 seconds.

| State | Frames |
|---|---:|
| `PATIENT_ON_BED` | 119 |
| `PATIENT_ON_FLOOR` | 39 |
| `PATIENT_DOWN_ON_FLOOR` | 15 |
| `FALL_ALERT_ON_FLOOR` | 86 |

The first alert occurs at **11.53 seconds**, after one second of accumulated floor-down time.

A significant observation is that the action classifier labels 231 of the 259 frames as `sitting_kneeling`, including much of the sequence. The successful alert is therefore primarily evidence that the location and transition logic work; it is not evidence that the action classifier correctly understands the synthetic fall.

### 4.4 Negative controls

| Control | Frames | Result |
|---|---:|---|
| Bed only | 60 | `PATIENT_ON_BED` for all frames; zero alerts |
| Floor only from first frame | 90 | `PATIENT_DOWN_ON_FLOOR` for all frames; zero witnessed-fall alerts |

These controls demonstrate the intended logical distinction between normal bed occupancy, a person already on the floor and a witnessed transition. They do not measure performance under real occlusion, lighting variation, staff activity or patient diversity.

### 4.5 Live system status during this audit

At the time of the audit:

- the 5060 project was reachable;
- the GPU was an NVIDIA GeForce RTX 5060 Ti with 8 GB VRAM;
- no fall-detection process was running; and
- GPU utilization was 0%.

This confirms the project files and tests, not a currently active live camera deployment.

---

## 5. Advantages of the current approach

### 5.1 It directly addresses the bed-versus-floor problem

The strongest improvement is the explicit use of scene context. A horizontal body is no longer automatically treated as a fall. This is essential in hospitals, where lying down is normally expected on beds.

### 5.2 It uses transition history, not only a single frame

The alert logic considers recent bed occupancy or a recent falling action. This reduces the chance of claiming that a fall occurred when the camera only observes a person who was already on the floor.

### 5.3 It is explainable

The system exposes:

- the action label and confidence;
- bed score;
- floor score;
- duration on the floor;
- current state; and
- alert decision.

This makes failures easier to investigate than a single end-to-end black-box fall/no-fall model.

### 5.4 It is modular

Pose estimation, region calibration, context geometry, action classification and the state machine are separate components. Each can be improved without rewriting the entire pipeline.

### 5.5 It is computationally lightweight

YOLO11n-Pose and a Random Forest are relatively small. The approach should be practical for one camera on the RTX 5060 Ti and is a reasonable starting point for an Orin Nano deployment after target-native TensorRT optimization.

### 5.6 It is camera-configurable

Normalized regions allow one configuration per fixed camera. This is simpler and more dependable than repeatedly detecting beds with a generic object detector when the room layout is static.

### 5.7 It already has basic automated regression tests

The eight unit tests protect the most important logical behaviour, including the floor-only negative case. This gives a stable base for further development.

### 5.8 It preserves the previous baseline

The original action classifier and live multi-person code were retained. The team can compare pose-only behaviour against context-aware behaviour rather than losing the earlier work.

---

## 6. Limitations and disadvantages

### 6.1 There is no real hospital validation

This is the largest limitation. The context pipeline has been tested on synthetic video, not on recorded ward-like CCTV footage. Synthetic imagery does not reproduce all real problems, including:

- blankets hiding limbs;
- bed rails blocking keypoints;
- nurses leaning over patients;
- visitors sitting or lying near beds;
- wheelchairs, walking aids and IV stands;
- low light and glare;
- compression artefacts;
- different clothing and body shapes; and
- partial or complete occlusion.

No claim about clinical accuracy, safety or deployment readiness is currently justified.

### 6.2 The hospital video demo is single-person

The current recorded-video hospital demo selects the largest detected person in each frame. It has one pose sequence and one state machine for the whole video. If two people are visible, the selected identity can switch between the patient, nurse and visitor.

The older live pose-only script supports multiple tracks, but that tracking architecture has not yet been fully integrated with the hospital context/state machine.

### 6.3 The action model is not hospital-specific

The classifier was trained mainly from GMDCSA24 indoor actions plus DJI material. It was not trained on ordinary hospital behaviour. The synthetic demo being classified mostly as `sitting_kneeling` confirms that the current action labels do not transfer reliably to the generated hospital sequence.

### 6.4 Evaluation is too small for a reliable performance claim

The combined classifier has only five groups: four subjects plus one DJI group. One fixed group split can produce unstable results. The running class also has zero support in the held-out report.

There is no cross-camera evaluation, repeated cross-validation, confidence interval, event-level sensitivity, false-alerts-per-hour metric or time-to-alert distribution.

### 6.5 ROI calibration is manual and camera-dependent

The fixed-region approach is strong when the camera never moves, but every camera must be calibrated separately. A small physical shift, zoom change or maintenance adjustment can make the regions inaccurate.

The current system does not automatically detect camera movement or reject an outdated calibration.

### 6.6 Location decisions have limited temporal smoothing

The location classifier uses per-frame overlap scores. There is a margin for ambiguous cases, but there is no full temporal hysteresis or probability filter. Short keypoint failures can cause bed/floor classification to flicker.

When location becomes `AMBIGUOUS` or `UNKNOWN`, the current state machine clears its floor-down timer. This may prevent an alert during intermittent occlusion.

### 6.7 A recent bed exit can still create false alerts

After a recent bed exit, any sustained horizontal/down pose on the floor can satisfy the alert condition. This could include intentional floor exercise, prayer, retrieving an object, staff activity or a controlled sit/lie-down.

The transition rule reduces false positives, but it does not prove that an uncontrolled fall occurred.

### 6.8 Privacy regions are not yet privacy protection

The `ignore` region type can be drawn and displayed, but the current hospital decision path does not use it to mask video, suppress detections or redact stored output. There is also no retention policy, access control or encryption layer in the prototype.

### 6.9 There is no operational alert integration

The system changes the visual banner and writes CSV data, but it is not connected to a nurse-call system, dashboard, acknowledgement workflow, escalation timer or audit log.

### 6.10 Jetson deployment is incomplete

The ONNX export and TensorRT build helper exist, but the Orin-native engine was not built or benchmarked during the verified work. Camera-specific ROIs and sustained live performance on the target device remain unverified.

### 6.11 Project storage and reproducibility need improvement

The remote project is approximately 14 GB:

- `.venv`: 7.6 GB;
- `data`: 4.4 GB;
- training runs: 1.2 GB; and
- the remaining models, outputs and source files.

The directory has no Git repository. Source, datasets, virtual environment, generated outputs and training artefacts are stored together. This makes version tracking, backup, clean deployment and collaboration more difficult.

---

## 7. Recommended improvements

### Priority 0 — required before any real accuracy claim

#### 7.1 Calibrate one actual fixed camera

Record a reference frame from the final camera position and create a real ROI file for that exact view. Document:

- camera ID;
- mounting height and angle;
- resolution and frame rate;
- bed position;
- usable floor area;
- privacy/ignore zones; and
- calibration date.

Add a calibration fingerprint or reference-image comparison so the system can warn when the camera has moved.

#### 7.2 Record a ward-like validation dataset

Collect consented, staged videos covering both positive and negative cases:

- walking then falling;
- bed exit then falling;
- slipping beside the bed;
- slow collapse;
- sitting on the floor;
- kneeling to pick up an item;
- normal bed lying and turning;
- nurse leaning over a bed;
- blanket-covered patient;
- bed rails raised and lowered;
- visitor and patient together;
- walking aid, wheelchair and IV-stand use;
- partial occlusion; and
- different lighting conditions.

Keep the split independent by person and camera. Do not place clips from the same person/session into both training and testing.

#### 7.3 Define event-level evaluation metrics

The primary metrics should be:

- fall-event sensitivity/recall;
- false alerts per camera-hour;
- precision of alerts;
- median and worst-case time to alert;
- missed detections caused by occlusion;
- bed-versus-floor location accuracy; and
- percentage of time in `AMBIGUOUS` or `UNKNOWN`.

Frame accuracy alone is not enough for an alarm system.

#### 7.4 Integrate per-person hospital state

Combine the existing multi-person tracking code with the hospital context. Every track should maintain its own:

- pose sequence;
- location history;
- last-bed time;
- last-fall time;
- floor-down timer; and
- alert/acknowledgement state.

ByteTrack IDs should be used consistently in recorded and live processing. The system should never share one global fall timer across multiple people.

### Priority 1 — improve robustness

#### 7.5 Add temporal filtering and occlusion tolerance

Use a short rolling filter for bed/floor scores and introduce hysteresis:

- require several consecutive frames to enter a location state;
- use a slightly lower threshold to remain in the state;
- retain the last reliable state through a short occlusion grace period; and
- pause rather than reset the timer during brief uncertainty.

#### 7.6 Train or fine-tune on hospital behaviour

Retain the current Random Forest as a transparent baseline, but train with real ward-like sequences. Compare it against a small temporal model such as a TCN, GRU or temporal Transformer operating on tracked keypoints and context features.

The input should include:

- normalized keypoints;
- keypoint confidence;
- torso angle;
- vertical velocity;
- body-centre trajectory;
- bed/floor scores;
- track age; and
- occlusion indicators.

#### 7.7 Separate “person down” from “fall witnessed” and “assistance required”

Recommended operational states are:

```text
ON_BED
BED_EXIT
UPRIGHT_ON_FLOOR
POSSIBLE_FALL_TRANSITION
DOWN_ON_FLOOR
FALL_ALERT_PENDING
FALL_ALERT_CONFIRMED
ACKNOWLEDGED
```

This gives staff and evaluators a clearer account of what the system observed.

#### 7.8 Add hard-negative scenarios

False-positive reduction should specifically target:

- deliberate floor sitting;
- kneeling;
- stretching;
- staff crouching;
- bed-making;
- dropped blankets;
- empty beds; and
- partial people at the image edge.

#### 7.9 Use confidence and uncertainty explicitly

Do not treat a low-confidence action as equally reliable as a strong prediction. If pose quality or location evidence is insufficient, report `UNCERTAIN` and avoid escalating until enough evidence returns.

### Priority 2 — deployment and maintainability

#### 7.10 Complete target-native deployment

On the Orin Nano:

1. install compatible JetPack, CUDA, TensorRT and Python dependencies;
2. build the pose engine locally;
3. run a sustained benchmark using the actual camera resolution;
4. measure end-to-end latency, FPS, memory and temperature; and
5. test recovery after camera disconnect, dropped frames and service restart.

#### 7.11 Implement privacy and security controls

Before hospital use, add:

- privacy masking for configured regions;
- configurable video retention;
- encrypted storage and transport;
- authenticated access;
- audit logging;
- patient/staff consent and governance review; and
- the ability to run inference without storing raw video.

#### 7.12 Add a real alert workflow

The prototype should send a structured event containing:

- camera ID;
- track ID;
- event time;
- confidence/evidence summary;
- thumbnail or short buffered clip if policy permits; and
- acknowledgement status.

Use escalation and cooldown rules to avoid repeated alarms from the same event.

#### 7.13 Improve project structure

Recommended separation:

```text
src/           version-controlled source
configs/       camera and threshold configs
tests/         unit and integration tests
models/        versioned model references
data/          external or ignored dataset storage
outputs/       reproducible generated results
docs/          report and operating instructions
```

Initialize Git for source and configuration files. Keep `.venv`, raw datasets, large runs and generated videos out of Git. Record an environment lock file and model/data version identifiers.

---

## 8. Proposed next validation plan

### Phase A — one-camera calibration

- Fix one camera permanently.
- Record a clean reference frame.
- Draw bed, floor and privacy regions.
- Verify stable pose and tracking for one and two people.

### Phase B — controlled scenarios

- Record at least several repetitions of each positive and negative scenario.
- Run the current context-state baseline without retraining.
- Review every alert and miss using the output CSV.
- Adjust only clearly justified configuration thresholds.

### Phase C — hospital-specific learning

- Label tracked temporal sequences.
- Use person/session/camera-independent splits.
- Retrain the action model.
- Compare the new model against the rule-based baseline.

### Phase D — shadow-mode pilot

- Run without notifying staff or making care decisions.
- Measure false alerts per hour, misses and latency.
- Review events with authorized clinical stakeholders.
- Update the system only after a documented error analysis.

### Phase E — guarded operational trial

- Integrate acknowledgement and escalation.
- Keep a human in the loop.
- Monitor model drift, camera movement and hardware health.
- Maintain a clear manual fallback procedure.

---

## 9. Final assessment

The work completed so far is technically meaningful because it fixes the central conceptual weakness of the original system: **body orientation alone cannot distinguish a patient safely lying in bed from a patient lying on the floor**. The combination of pose, fixed-camera context and temporal transition logic is a sensible architecture for a hospital proof of concept.

The current strengths are explainability, modularity, low computational cost, transition-aware logic and passing automated tests. The main weaknesses are domain mismatch, lack of real ward data, single-person hospital demo logic, manual camera calibration and incomplete deployment/privacy/alert infrastructure.

The most valuable next step is not a larger model. It is to calibrate one real fixed camera and collect a carefully labelled set of realistic positive and hard-negative ward scenarios. That evidence will show which failures come from pose estimation, tracking, ROI context, temporal classification or alert thresholds. After that, model and deployment changes can be made based on measured needs rather than assumptions.

---

## 10. Key artefacts

### Remote 5060 project

- `/home/robotx/fall_action_project/src/hospital_context.py`
- `/home/robotx/fall_action_project/src/hospital_state.py`
- `/home/robotx/fall_action_project/src/draw_context_rois.py`
- `/home/robotx/fall_action_project/src/run_hospital_context_preview.py`
- `/home/robotx/fall_action_project/src/run_hospital_video_demo.py`
- `/home/robotx/fall_action_project/configs/synthetic_ward_context.json`
- `/home/robotx/fall_action_project/outputs/synthetic_hospital_context.csv`
- `/home/robotx/fall_action_project/outputs/context_controls/`

### Local deliverables

- `Team3_RSE4207_WardWatch_CCTV.pptx`
- `hospital_cctv_test_videos/hospital_cctv_walking_then_fall.mp4`
- `hospital_cctv_test_videos/hospital_cctv_walking_only_control.mp4`
- `hospital_cctv_test_videos/hospital_cctv_floor_only_control.mp4`
- `work/fall_hospital/hospital_context/outputs/synthetic_hospital_context_h264.mp4`

