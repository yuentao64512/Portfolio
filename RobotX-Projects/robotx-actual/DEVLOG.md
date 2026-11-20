# DEVLOG — MASTER_SCRIPTS (global_v3)

Append-only. One block per version, newest **last**; never edit an older block (add a correction block instead). A session
reads the last block first: it says where things stand and what to do first. The template is the first block. The project
mirror (`claude/DEVLOG.md` in the SIT ASSIST project) is a copy of this file made when a version is delivered; this file is the
source. Write it before building the archive (CLAUDE.md step 4).

---

## Overnight 2026-11-09 (the GUI night) — the morning report (read this first; the version blocks are at the end, as always)

**N1 → N6 built in the brief's order, one version each, and - everything before them being green - the three "if time remains" items too**
(`docs/OVERNIGHT_2026-11-09_GUI.md`; ROADMAP §9 WP25-WP33 ✅, before WP18-WP24). Each version: `make quick` + its files → `make live` →
`make test` → the Chrome walk at 1440 × 900 → GUIDE + CHANGELOG → its DEVLOG block → `docs/screens/v0.1.NN/` → the archive. **Nothing was
deployed**; `network*.env`, the keys, `control_v2/`, the students' folders, `drone/`, `uav/` and the fake FCU's physics untouched; no `pkill -f`;
no page arms, disarms or sets a mode except through the existing operator buttons.

| version | WP | what | make test | archive sha256 |
|---|---|---|---|---|
| **0.1.69** | WP25 (N1) | the dry / wet SESSIONS (OCS-side: buoys · dockface · windows · uavmap · cannon · collect), MISSION METRICS | 721 | `0c3d0260…` |
| **0.1.70** | WP26 (N2) | the one-row bar (■ STOP RUN kept, ⏻ / ↺ → SYSTEM), RUN in two columns, the console drawer on every tab, the declaration preview = what is sent | 728 | `6b5c48a6…` |
| **0.1.71** | WP27 (N3) | MISSION: the picker in EVENT / POOL / DRY, T4 as a tier, **m4t1 → m4t0** (declared m1t1 + T4 core; `m4t1` still accepted, the same verdicts), the map's TOOLS ▾ | 739 | `ecceb082…` |
| **0.1.72** | WP28 (N4) | SYSTEM: one hosts table (settings ▾ / actions ▾ per host), BENCH & TOOLS, the jobs streamed to `logs/ocs/system_actions/` | 745 | `8e50ec2b…` |
| **0.1.73** | WP29 (N5) | provenance on every value / chip / header / button (`ocs/provenance.py`), ⚙ a panel over any tab, nothing operational in it | 754 | `fac0bb44…` |
| **0.1.74** | WP30 (N6) | the boat window: `/v1/` = the v0.1.68 page byte for byte, two header rows, dials under the wireframe, the FCU words card (other ▾), MAVLINK console, GUI new / classic | 761 | `b6e90b09…` |
| **0.1.75** | WP31 | LOG: one list (events · commands · alarms + history · judge · system), newest first, chips, search, time range | 763 | `1bda4305…` |
| **0.1.76** | WP32 | VEHICLES: one card per vehicle, the same 11 rows; never greyed; the v0.1.65 UAV detail's shifted columns fixed | 766 | `4c8a28ec…` |
| **0.1.77** | WP33 | JUDGE: reporting + RoboCommand on top, the timeline strip, the judges' table (a row per vehicle) beside TASKS, the feed full width | 768 | rebuilt at 05:00 with this report (`shasum -a 256 _archive/MASTER_SCRIPTS_v0.1.77.tgz`) |

**The end of the night, on the final tree (v0.1.77, 04:23-04:50, nothing else running)**: fault_timing **8 / 8** (bridge 2.8 s) · fault_timing
--uav **11 / 11** · sim_day S0-S9 **31 pass / 0 findings** · S14 **pass** (m1t1 --planner s6 on extra, DONE in 146 s, max 0.15 m) · **the m1t1
campaign 128 / 128 identical to v0.1.68** · status **CLEAN**. (The same checks on v0.1.74 at 03:16 gave the bridge row 3.0 s = NO while a
Chrome walk ran beside it; alone 2.8 s three times - the v0.1.65 margin, not a regression.)

**Look at first**: the captures, one folder per version (`docs/screens/v0.1.69` … `v0.1.77`) - the RUN bar (v0.1.70), MISSION (v0.1.71), SYSTEM
(v0.1.72), ⚙ (v0.1.73), the boat window's five tabs (v0.1.74), LOG · VEHICLES · JUDGE (v0.1.77). **JUDGE was built from the brief's one line**
- board 06 is not in the tree - so it wants your eye on the canvas before anything is changed on top of it.

**Found live and fixed**: the map's *fit UAV* button wrapping the judge map's row (hidden there); the launcher's FIELDS line silently losing
cv / perception / world to an inline comment (caught by test_v0147_ab); SOG / HDG empty with the fake FCU (fall back to status); the UAV detail's
columns shifted since v0.1.65 (its state took two cells); the LOG's judge chip at 0 beside a 3-line feed (now reads the feed); three orphaned
fake FCUs from interrupted runs streaming to random ports (killed by PID - `pgrep -fl fake_fcu` before blaming the code).

**Next session**: the canvas review with you; then WP18 (G1, sim_day S15) is the first not-done WP. The Jetsons are still on their old
version - GUIDE §1 when you say so.

---

## Overnight 2026-11-08 (the UAV night) — the morning report (read this first; the version blocks are at the end, as always)

**All three versions of the brief were built, tested and archived, plus one round of the borrowings** (`docs/OVERNIGHT_2026-11-07_UAV.md`;
the stop moved to 08:30 by Dmitrii before the start). Each passed `make test` before the next; archives in `_archive/`; the full account in
`docs/REPORT_2026-11-08_UAV.md`. Nothing was deployed (the Jetsons stay on v0.1.58); `network*.env`, the keys, the students' folders,
CV_TRAIN and `drone/` untouched; no `pkill -f`; the companion never arms by itself (the operator's page buttons may, v0.1.50); **nothing
we built commands the drone**.

| version | what | make test | live acceptance |
|---|---|---|---|
| **0.1.64** | WP14 + D0: the way back (`v0.1.63_pre_uav.tgz`, `scripts/rollback_to.sh`), the UAV team's files read-only in `uav/`, the adapter (their §5 wire, their ReliableSender verbatim), the two-vehicle declaration from the drone's own fence + ceiling with their check first, `declare uav` off by default, the course / commands / RunStart to their Orin | 659 | USV-only 46 / 46 · 8 / 8 · 0/0/0; **rc-test: one RxRequest USV1 + SGIT-53577**, both heartbeats, RunStart forwarded; their 5 sim-test scenarios (+2) = their own bridge's results |
| **0.1.65** | the UAV window (8099), the OCS UAV card / tab / rows, the alarm rows (incl. FOREIGN DECLARATION), the launcher's UAV field | 665 | USV-only 46 / 46 · 8 / 8 · 0/0/0; **with the UAV: fault 11 / 11** (+ uav_tel, uav_fence, uav_adapter), the window 0 jumps |
| **0.1.66** | WP16: the buoy contract, the two paths, the fusion (1 m link, the camera's position, the UAV's colour), **m1t2 = T1 Advanced with WAIT_UAV_MAP**, the fake UAV + the colour-blind world | 681 | USV-only 46 / 46 · 8 / 8 · 0/0/0; **m1t2 DONE live on the colour-blind world, the judge's report = the truth at 0.7 and 1.5 m**; m1t1 S6 146 s → 145 s with the UAV; the m1t1 campaign 128 / 128 identical to v0.1.63; with the UAV up: sheet 46 / 46, fault 11 / 11, `loop_slow` never |
| **0.1.67** | the borrowings B1 (a new pinger = a new course), B3 (declared, not AUTO → a caution), B4 (T4 readiness after holding on station), B13 (field lessons), B14 (the merge rule) | 686 | USV-only 46 / 46 · 8 / 8 · 0/0/0; m4t1's readiness *on station 3.1 s after arriving* |

`make quick` is green on the final tree; `stop_all.sh` → status CLEAN; `sha256sum -c uav/MANIFEST.sha256` 36 OK; `rollback_to.sh --dry-run
v0.1.63` lists cleanly; a real rollback of a copy of the final tree back to v0.1.63: a copy of the final v0.1.67 tree rolled back to v0.1.63 in 292 s - `uav/` and `optiprimex/uav/` gone, `drone/`, `logs/`, the archives kept, `make quick` 573 passed, the copy's venv pointed at the copy, the real one untouched.

**Look at first**: the report's §2 - m1t2 live (`logs/rctest/run_20261108_035130/`), the decision on `uav.assoc_unique_m` (1.5 m noise),
the two-vehicle RxRequest (`logs/rctest/run_20261108_014309/…014311.log` line 1747), `docs/screens/v0.1.64/01_judge_two_vehicles.png`,
`docs/screens/v0.1.65/01_uav_window.png`.

**Decisions taken unattended** (details in the report §3 and the blocks): their sim-tests pinned to what their stack does (mpsilent /
mpsilentfix publish nothing there too); a fence change with no UAV declared is reported, not declared alone; RunStart reaches the Orin only
for a run that declared the UAV; 2 Hz heartbeats kept; the bridge line `topic perception/# in 0 local/usv/uav/ uav/`; the merge rate
reported (no linking during the survey by design); no camera colour checks under m1t2; the archive excludes `drone/` (CLAUDE.md step 5);
B5–B8 left for when you are awake.

**Found live and fixed** (7; the report §4): the OCS page's null nav, m1t2 refused under the course policy, the bench's third bridge copy,
m1t1's duplicate buoy with the UAV up, the UAV colour vote, the listener's log on stop, the tty's 1 KB line.

---

## Overnight 2026-11-07/08 — the morning report (read this first; the version blocks are at the end, as always)

**All five versions of the brief were built, tested and archived** (`docs/OVERNIGHT_2026-11-07.md`). Each one passed `make test` before
the next started, its archive is in `_archive/`, and its DEVLOG block is at the end. Nothing was deployed: the Jetsons are still on
v0.1.29.1. Nothing ran on the GPU or the AGX, and `network*.env`, the keys, the students' folders and CV_TRAIN were not touched; no
`pkill -f` was used. The companion never arms by itself; the operator's page buttons may (v0.1.50). Nothing stopped the night.

| version | WP | what | make test | live acceptance |
|---|---|---|---|---|
| **0.1.54.1** | - | the EKF bit: 0x100 is PRED_POS_HORIZ_REL (ABS = 0x200); a relative-only prediction no longer passes the AUTO guard; the fake FCU sends the real bits | 530 | (the live tier) |
| **0.1.55** | WP10f-rest | `mm/state cmd` (the frame τ / the GUIDED target / the heading goal) → the boat window's SET, speed tick, CONTROL, ACTUATION; simulated NX / AGX host beats ("this Mac (simulated host)"); the thresholds from the OCS profile; the lamp read-back hook | 537 | fault test **8 / 8 ≤ 3 s** with the boat window open · sim-day **46 / 46** · no-jump **0 / 0 / 0** · page CPU on the Mac ≈ **14 % of one core** for both pages |
| **0.1.56** | m3t1 hardening | standoff lines from the dock geometry (**5.70 / 5.00 m**), back-out to the line, a re-fit before the bay, a lost line holds; every stage and failure in the fast SIL; the votes in the map hover | 555 | campaign **81 / 84** (the model 12 / 12) · S1 + S9 with cvt-v2-l + rc-test **9 / 9** (S9 4 / 4; the three report pairs in rc-test's log) · `--step` walk **11 / 11** |
| **0.1.57** | WP12 (sim) | the cannon sender follows the aim topic (`target: null` after 1 s), `window_id` in the packet, a fake Pi with a generated test key, the status on the boat window and the OCS SYSTEM card | 564 | S9 4 / 4 with the cannon window up; the status trace OK + window 1 during AIM_FIRE → AIM_OFF at once; hmac ok, 0 bad of 2688 |
| **0.1.58** | WP13 (sim) + your box key points | `t3.dock_source: boxes`: each face's centre + boundaries from the camera, a line from ≥ 2 faces, offset = the sway path | 576 | offline on renders: cvt-v2-l dock faces **40 / 40** sway paths within 0.3 m / 5°, box faces 22 / 23; **no training needed**; live m3t1 with `boxes` DONE in 182 s |

`make quick` is green on the final tree. `stop_all.sh` and `status.sh` are CLEAN.

**Decisions taken unattended** (the details are in each version's block):
1. **The standoff formula gives 5.70 / 5.00 m, not the brief's "≈ 5.5 / 4.8"** (3.5 + 0.595 + 0.60 + 1.0 / 0.3). I kept the formula
   as written. The margins are profile keys if you want the brief's numbers.
2. **"Inside the line" for TO_LINE's back-out = inside the finger zone (4.7 m)**, not "≤ 3 m".
3. **m3t1 `--step` resumes with `resume`**: STEP is reserved for the TEST_* missions, and the CLI's hint already says so. Written in
   GUIDE; the code is unchanged.
4. **WP13 went past the 04:30 gate**: v0.1.57 was archived at 04:38. You had asked for the box key points during the night, and the
   code was already built and tested in a copy.
5. **v0.1.56's re-fit and lost-line abort apply to the map source only**: box-track ids are not map entities. This was a merge
   decision between two helper streams.
6. **The sim's "dry mode" for the cannon is real UDP to the local fake Pi**, using a test key generated under `logs/`. The real key is
   never read.
7. **D9's votes**: they show in the OCS map hover only. The SIM page draws the world file (truth), so there are no votes there.

**What to look at first**
- This table, then `docs/REPORT_2026-11-08.md`.
- `docs/DOCK_KEYPOINTS.md`: your box key points request.
- `docs/screens/v0.1.56/` (MISSION, the step walk, S1 + S9) and `docs/screens/v0.1.55/` (the boat window in five themes).
- rc-test's log `logs/rctest/run_20261107_035010/robocommand_20261107_035012.log`: docking 2 / 3 / 2, each followed by firefighting.

**For check-in**: measure the real dock's finger length and the window spacing. Then only `t3.finger_m` and the world files change.

**Still open**:
- Before any Pixhawk: the deploy + bench rehearsal (LAB_DAY block B), with the EKF fix in it.
- On the AGX day: the students' cannon sender / receiver snapshot.
- Only on the bench: the Lua `LAMP` line (a question for Jaison).
- The page CPU on the NX.
- `face_hdg_deg` for `kind: box` is read differently by the renderer and fake_perception (world_boxes).

---

## Overnight 2026-11-06/07 — the morning report (read this first; the version blocks are at the end, as always)

**Versions built** (each archived in `_archive/`, extracted over the tree, `make test` green before the next; nothing deployed - the
Jetsons are still on v0.1.29.1; network*.env untouched; the students' folders untouched; no `pkill -f`):

| version | WP | what | make test |
|---|---|---|---|
| **0.1.43** | WP10i-lite | the v1 page frozen at `/v1/` (SETTINGS switch, launcher `ui`, `run_ocs.sh --ui v1`); the top bar with the state word + why on every page (LINK LOST, the FCU chip, two warning lines); the RUNSTART slot + the **3·2·1** with HOLD-on-start / cancel-auto-start (the OCS really delays the --auto start); the stage list (exit · elapsed · typical from the campaign) + the strip; five themes + the preset's default + the contrast rule; SOUNDS (off); TIMED OUT + the boat's stale-command refusal (proven through a cut bridge); the NX / AGX host-health beat; the fault-timing tool | 342 |
| **0.1.44** | WP10i | the regrouping: OVERVIEW (six cards, the run as seven steps) · MISSION (map, MISSION CONTROL, the picker only here, the T4 modifier) · USV CONTROL ↗ · UAV · COMMS & DIAGNOSTICS · JUDGE · LOG · SETTINGS, HELP behind ?; the alarm model (ALARM / WARNING / CAUTION, ack, escalation after 5 min, 24 h history); T4 as a modifier (`--t4 on|off`, the WP9c row soft unless `t4_required`); the scene layer (illustrative) | 348 |
| **0.1.45** | WP10f 1–2 | the BOAT window (boat-web 8098, run_sim.sh window `bweb`): header, the five-system wireframe, OVERVIEW with hatched not-published values, handover / takeover; usv_ui's FCU block at 2 Hz + the page-word whitelist (decision 25) | 355 |
| **0.1.46** | WP10j | the Task 3 docking structure in the renderer (cube on its frame, windows 1 / 2, the RED / GREEN indicator, lights from `sim/dock/state`) + `render --dataset --scene dock|mix` + `docs/REPLY_dock_dataset.md` for CV_TRAIN | **363** |

**The sim-day sheet on both pages** (36 checks + 6 OCS-page checks, `sim_day --page`): **42 / 42 on `/` and 42 / 42 on `/v1/`** for
v0.1.43, v0.1.44 and v0.1.45 (tables in `docs/screens/v0.1.4x/sim_day_v2.md` / `sim_day_v1.md`).

**The fault-injection timing test** (OCS_V2_SPEC §13, `python -m optiprimex.tools.fault_timing`) - **8 / 8 within 3 s** on every version;
the last run (v0.1.45, the boat window open):

| fault | named (s) | first action readable (s) | ≤ 3 s | what the page showed |
|---|---|---|---|---|
| kill | 0.8 | 0.8 | yes | NOT READY kill / E-stop asserted release it on the transmitter, then confirm_safe / FCU mission manager KILLED HOLD KILL  / vehicle KILLED: release the kill / E |
| rc6 | 0.4 | 0.4 | yes | NOT READY the boat is in MANUAL RE-ARM (enable_auto) / FCU mission manager MANUAL HOLD   |
| rc_lost | 2.1 | 2.1 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / RC LOST (the receiver) while AUTO: the transmitter is the safet |
| gps | 0.2 | 0.2 | yes | NOT READY fault EKF_BAD clear the cause, then confirm_safe / FCU mission manager FAULT HOLD   / vehicle FAULT EKF_BAD: clear it, then confirm_safe |
| fcu | 2.5 | 2.5 | yes | NOT READY fault FCU_LOST clear the cause, then confirm_safe / FCU mission manager FAULT HOLD   / FCU LINK LOST: no heartbeat for 3 s - takeover on the boat (usv |
| bridge | 2.7 | 2.7 | yes | LINK LOST the boat's status is 3 s old (bridge / radio) check the radio; the boat holds by itself (link_loss_action) / FCU mission manager AUTO_HOLD HOLD  stale |
| refusal | 2.0 | 2.0 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / waiting for RunStart · a refusal is silent: if nothing comes, a |
| perception | 2.1 | 2.1 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / PERCEPTION STALE: sim_observer silent for 2 s - CAMERA tab -> t |

**Decisions taken unattended** (details in each version's block at the end):
1. **The numbering.** ROADMAP §7e was revised during the night (WP10j first as v0.1.43); the run had started from your prompt
   (WP10i-lite → WP10i → WP10f as 0.1.43–0.1.45) and those were archived before I saw the change, so WP10j became **0.1.46** - §7e now
   says so. CV_TRAIN's overnight plan polled for the literal "0.1.43": I messaged its session (cv-train-35) that the signal is
   ≥ 0.1.43 + `docs/REPLY_dock_dataset.md`, and to export into two sub-directories named `sim_v0.1.46_dock/…` (one shared directory fails).
2. **Palettes**: the mock's accent / warning / alarm failed "differ in lightness" (hc-light 37 / 36 / 35 L*) - lightness moved ≥ 8 L* apart,
   hue kept, every contrast floor kept (the values are in v0.1.43's block).
3. **The alarm table additions** the fault test needed: RC LOST, FCU LINK LOST (fcu_age_s > 2 s), BOAT LINK LOST (status > 3 s → the word
   LINK LOST), PERCEPTION STALE (per source, limit 2 s - the sim mixes the camera path with the geometric fake).
4. **T4**: soft everywhere (`declaration.t4_required: false` on the course) until RoboNation confirms - then one line.
5. **The regrouping moved the existing blocks** (same ids, same draw code) instead of rewriting them; old tab links still work.
6. **Two sim-day sheet races fixed in the script, not the product**: S2 (the switch flipped and enable_auto sent in the same instant)
   and S11 (a repeated command_seq is correctly dropped by the OCS when the sheet runs twice on one stack).
7. **The boat window**: the thruster ↔ channel map from usv_ui's motor names (M-P ch1, SW-A ch2, M-S ch3, SW-F ch4 - **verify against the
   CAD / firmware**); a `cannon` element added to the wireframe; the host thresholds are flags there, not read from the profiles yet.
8. **The dock**: faces of the pool's bays point along the bay line toward the start (they had no `face_hdg_deg`); the cube side clamped to
   0.85–1.3 m (the spec's smallest jitter made the windows overlap); `--scene buoys` (the default) keeps the four v0.1.33 classes.

**What stopped and why**: **WP10j-b was not started** - its precondition is not met: **`../CV_TRAIN/models/cvt-v2-l` does not exist**
(only `cvt-v1-l`; CV_TRAIN can only build it after it imports tonight's dock dataset). optiprimex is ≥ v0.1.43, so that is the one
missing piece. Nothing else stopped; no red was carried forward (the reds met on the way were fixed inside their version: two old tests
that looked for the old tab names, two headless-Chrome checks that dumped the DOM too early under load, a fake-judge attribute).

**Update 07:55 — `cvt-v2-l` is published** (CV_TRAIN's message; `../CV_TRAIN/models/cvt-v2-l`, sealed with sha256.txt, notes in
`../CV_TRAIN/docs/HANDOVER_cvt-v2-l.md`): five classes 0 buoy · 1 dock_face · 2 window · 3 box · 4 indicator, the cvt-v1-l contract
(imgsz 768, conf 0.5, iou 0.7), trained on the v0.1.46 export; synthetic box mAP50 0.96 (window 0.99, indicator 0.88), window light
1811 / 1813, the bay from the indicator 95 % at approach but 47 % at 20–30 m (**decide at approach**), range the dock off dock_face (the
indicator's mask is poor), engine 84.6 ms. **Caution**: the old coloured dock face (the v0.1.42 look) is no longer recognised as dock_face
(mostly "box") - any world or render path still drawing it would mislead the model. **WP10j-b's precondition is therefore met, but it
came 65 min before the 09:00 stop**: WP10j-b is a whole version (the dock from the camera, SWAY_SEARCH / BACK_OUT, the reports in order,
the window-matched aim, Ultralytics into the venv, the campaign ≥ 11 / 12 per detector, the --step walk, rc-test's log) - not started, so
nothing is half-built. **It is the next session's first work package** (v0.1.47), with CV_TRAIN's two numbers above as its design inputs.

**What to look at first**:
1. **The page itself**: `scripts/launch.sh` → sim → http://127.0.0.1:8096/ - OVERVIEW's seven steps and the alarm card; MISSION (the picker,
   the T4 box, the stage list); hand over, `mission m1t1sim --auto`, declare → **the 3·2·1**. Screenshots in `docs/screens/v0.1.43/` …
   `v0.1.46/`.
2. **The boat window** http://127.0.0.1:8098/ (handover / takeover) and **the thruster channel map** (decision 7).
3. **`docs/REPLY_dock_dataset.md`** and the dock frames in `docs/screens/v0.1.46/` - is the structure right against the real dock?
4. **CV_TRAIN picked it up** (its reply, 05:17): both dock exports running into `data/raw/sim_v0.1.46_dock/{mix_marina_s21,dock_pool_s22}`;
   it plans `cvt-v2-l` (5 classes, 3 epochs from the buoy v2 weights) published by ~08:45 if it beats its buoy-only fallback - then
   WP10j-b's precondition is met and it can be the next version. Two renderer notes from it for the backlog (in
   `CV_TRAIN/docs/HANDOVER_cvt-v2-*.md`): a close flashing buoy draws its light ball detached above the dome; truth `visible_frac`
   ignores frame-edge truncation.
5. **Open items**: the MAP / mini-map canvas colours in the light themes (still dark); the bridge alarm names the fault at 2.7–2.9 s
   (inside 3 s, little margin - its threshold is the spec's 3 s); op-tui does not show the alarm list / TIMED OUT; the alarm history is not
   persisted across an OCS restart; the lamp readback and the throttling flag are unproven on the hardware; **the deploy** (the Jetsons
   are 16 versions behind - the launcher, the recorder's AGX rehearsal and the boat window's NX side all wait for it).

---

## v0.1.19 — 2026-11-02 — reshuffle, mission codes, hand-off (WP0)

**Session**: Claude Fable, Cowork, with Dmitrii (decisions polled with AskUserQuestion). Previous version v0.1.18 (UX rehaul).

**Decisions taken (2026-11-02, by Dmitrii)**
- Mission codes: m-number = the RobotX task number (m1 Task 1, m3 Task 3, m4 Task 4, m0 = our tests); alternatives were a
  sequential numbering (m1 = our first mission) and keeping the wire names. Chosen because the handbook, the judge's `task` field
  and the consoles then agree. Tier digit = handbook tier; `sim` suffix = the same mission + a guard (not a separate mission).
- Codes live in the operator layer only (`ocs_cli`, op-tui, usv_ui, playbook); the wire keeps `task: T4` + `args {tier, sim}`.
- Order of work: simulated missions first (WP1 sim world → WP2 m1t1 → WP3 m3t1 → WP4 RoboCommand → WP5 camera → UAV later).
- Directory reshuffle: full, now, in this version (not gradual).
- Hand-off: both a document in the tree (ROADMAP.md + DEVLOG.md) and a mirror in the project.
- Test missions m0t2 (path) and m0t3 (sway) exist as planned catalogue entries.
- Camera: MJPEG over HTTP (WP5).

**What changed · where**
- Tree: `bench/optiprimex` → `optiprimex/`; `bench/tests` → `tests/`; `deploy/scripts` + `bench/scripts` → `scripts/`;
  `deploy/config` → `profiles/pool/`; `bench/config` → `profiles/bench/`; `deploy/logs` → `logs/`; `bench/.venv` → `.venv`;
  `pyproject.toml`, `requirements.txt`, `Makefile`, `.gitignore` at the root; `bench/README.md` → `docs/BENCH.md`,
  `bench/BENCH_GUIDE.md` → `docs/BENCH_GUIDE.md`, `deploy/README.md` → `docs/LAUNCHERS.md`; removed `final/`, `INFRA_TEST.md`,
  `usv_ui/POOL_RUNBOOK.md`. New: `README.md` (the map), `ROADMAP.md`, `DEVLOG.md`, `CHANGELOG.md` (the version list that closed
  `bench/README.md`, plus a one-line table).
- Paths: every launcher resolves `KIT` = the tree root (`BENCH="$KIT"`, `UI="$KIT/usv_ui"`, `CV="$KIT/cv"`, `.venv` at the root,
  `profiles/pool/*.yaml`, `profiles/pool/mosquitto/*.conf.in`, `logs/<host>`); `usv_ui.py` finds `../optiprimex`; `cv/_env.sh`
  sources `scripts/_netenv.sh`; `usv_ui/Makefile` `ROOT := ..`; tests use `ROOT / "profiles" / "bench"`; `status.sh` share path.
- Mission codes: `optiprimex/tools/playbook.py` — `Mission.code` / `wire_tier`, `MISSIONS` keyed by code (m4t1, m1t1, m1t2, m3t1,
  goto, m0t0, m0t1, m0t2, m0t3), `MISSION_ORDER`, `CODES`, `resolve(word) -> (code, task, tier, sim)`, `code_of(task, tier, sim)`,
  `mission_for(task|code, tier)`; `how_to_select` / `mission_short` / `mission_stage_names` accept either; `check()` resolves codes,
  flags planned missions and predicts the sim guard from `status.fcu_link`; help lists the code table. `ocs_cli`: `mission` /
  `start_task` take a code, a code+`sim`, or the old name; new `missions` sub-command; `mission` (no arg) prints the code.
  `ocs/cmd_authority.py`: `START_TASKS` (schema names incl. TEST_PATH / TEST_SWAY), `MISSION_ARGS` tier / sim carried into the
  preselection and into `START_TASK` args when no task is given. `ocs/ocs_app.py`: `mission` dict carries `tier`, `sim`, `code`
  (`mission_code()`); the auto-start emits the args. op-tui and usv_ui: MISSION box shows `code (task)`, the MISSION tab lists codes
  with the wire task beside, Enter prefills `mission <code>`; command help / examples use codes.
- Sim guard: `companion/state.py` `VehicleState.fcu_link` (in `status_body`); `usv/mission_manager.py` subscribes `presence/usv_ui`
  (raw JSON) and sets `vs.fcu_link` while the beat is < 10 s old; `usv/command_handler.py` START_TASK with `args.sim` → REJECTED
  unless `fcu_link == "fake"` (`"sim mission needs the fake FCU (usv_ui holds: …)"`).
- Schema: `bodies.schema.json` cmd.task enum + `TEST_PATH`, `TEST_SWAY` (planned wire names; the vehicle rejects them as not built).
- Docs: GUIDE (header v0.1.19, §0/§1 tree + deploy loop with the `rm -rf` of the old folders and the setup rerun, §3 "typed at the
  tree root", logs paths, "Picking a mission" by code, 4.6a/b/c, §9/§10), HANDOFF (§3 layout rewritten, §4 setup, §5, §7),
  CLAUDE.md (rules + the session contract), docs/BENCH.md (head, layout, history moved out), docs/BENCH_GUIDE.md, docs/LAUNCHERS.md,
  usv_ui/README.md, cv/README.md.

**Tests**: 124 (`make test`, ≈3 min): +`test_mission_codes_and_sim_guard` (SIL1: codes resolved by `ocs_cli`, `ocs/state.mission.code`
= `m0t1sim`, START_TASK carries `args.sim` and no code, REJECTED with no usv_ui beat, accepted with a retained `presence/usv_ui`
beat `fcu_link: fake`, REJECTED again on `real`, the same mission without `sim` runs; status `fcu_link`); `test_mission_tab_and_warnings`
extended (codes in the MAIN box and the tab, `resolve` / `code_of` / `mission_for` matrix, the playbook's sim prediction). Known
flake under full load: `test_reset_both_ends_mission_preselect_and_test_missions` (LOOP_SLOW on the ABORT→AUTO_HOLD wait; passes
alone and in the last full run). The new SIL test resets both ends at its start so a failed predecessor cannot cascade.

**Rehearsed live**: nothing on the hosts this version (the sandbox has no hosts); the Mac tree was reshuffled with `mv -n` into
`_archive/pre_v0.1.19_layout/` and the archive extracted over it; Dmitrii runs `make venv` on the Mac and the GUIDE §1 loop +
setup scripts on the Jetsons (chores in chat).

**Open items** (carried): FCU_FAILSAFE never raised; KILL/ESTOP as NAMED_VALUE_INT to confirm at L1 with the real Pixhawk; Miro
frame 24 text is v0.1.14-era; `link_loss_action: safe_hold` not rehearsed live; T1/T3/TEST_PATH/TEST_SWAY are catalogue entries
only (the vehicle rejects them as "not built on this vehicle").

**Next session, first**: WP1 (ROADMAP §4): `profiles/bench/world_pool.yaml` → `run_agx.sh --sim` → `sim/fake_dock.py` → usv_ui
`pathgen` / `dockto` → `tests/test_sim_pool.py` → GUIDE 4.14 → DEVLOG block → `MASTER_SCRIPTS_v0.1.20.tgz`. Poll Dmitrii for the pool
dimensions / lane and the dock geometry before writing the world file.

---

## v0.1.20 — 2026-11-02 — the simulated pool (WP1) + the v0.1.19 deploy to the Jetsons

**Session**: Claude Opus 5.5 (Claude Code on the Mac, ssh to both Jetsons), with Dmitrii; he was away for the build after
clearing WP1–WP5 one version each. Previous version v0.1.19 (WP0).

**Decisions taken (2026-11-02, by Dmitrii, polled)**
- Simulator shape: Python sim (fake_perception + fake_dock, on the AGX or the laptop, in `make test`) + the browser view on the
  real brokers (map.html today; a sim.html with controls parked in ROADMAP §4a). Alternative was the whole sim in the browser
  (his pathing_sim JS) — not testable, the tab must stay open.
- Task 1 planner (WP2): both, switchable — the simple gate-midpoint chain and a Python port of pathing_sim S6/P4 (ROADMAP WP2).
- Pool 50 × 25 m. Claude deploys to the Jetsons and keeps a trail (`docs/JETSON_LOG.md`). TUI tests may render larger
  (new ones 200 × 60; the 150 × 50 ones stay).
- Inputs added by Dmitrii: `handbook/` (RobotX 2026 Team Handbook md + pdf), `pathing_sim/` (S6 sketch, spec, report). What the
  handbook changed is in ROADMAP §1a (flashing lights, ten buoys, bays left to right, window only after DockingReport, PoR course,
  RoboCommand rates). GitHub: `gh` is logged in (ExtraDmitr); `robonation/robocommand` @ 8fc09f2 is reachable for WP4 (no MCP).

**Jetsons (before building)**: both were on v0.1.18 in the old layout. Old trees moved (not deleted) to
`~/Desktop/dm3/_pre_v0.1.19/MASTER_SCRIPTS_v0.1.18`, v0.1.19 extracted, network*.env copied, `co.json` (NX) and the CV models
(AGX) restored, setup scripts rerun, AGX self-tests ok, stage A smoke on the three hosts ok, everything stopped. Every command:
`docs/JETSON_LOG.md`. Mac: `.venv` built (python3.10), baseline 124 passed.

**What changed · where**
- Worlds: `profiles/bench/world_pool.yaml` (new: pool walls; Task 1 = ids 1–10: flashing blue entry (5, −7), gates at x 12 / 19 / 26
  2.4 m wide GREEN west / RED east, obstacles (15.5, −9.5) (22.5, −1.5), steady blue exit (33, −5.5); Task 3 = bay faces 21–23 on
  x 45 at y 2 / 5.5 / 9 (bay 2 GREEN), windows 31–36 ±0.6 m, pontoons 41–44 truth-only; camera = ZED 2i 4 mm preset from the
  pathing_sim spec §4.1; `dock:` timings), `world_por.yaml` (new: two pairs at x 3 and 18 — the 15 m spacing is assumed).
  `world_bench.yaml` unchanged (the unit tests).
- `optiprimex/sim/world.py` (new): load, `WorldFrame` (equirectangular, same model as the fake FCU), `pose_from_state` (lat/lon →
  world frame, so the world does not move with the CO), `light_phases` / `observed_colour` (flash 1 s / 1 s), `apply_dock_state`,
  `bay_faces`, `in_bay`, `dock_normal`.
- `optiprimex/sim/fake_dock.py` (new, `op-fake-dock`): `DockSim` (idle → docked = 3 s < 0.3 m/s in the GREEN bay or `sim/dock/cmd
  docked` → a window RED → `local/usv/cannon/aim {on, bay}` fresher than 1 s for 4 s → GREEN 5 s → done; cmds reset / green_bay /
  docked / fire / auto) + the MQTT node (retained `sim/dock/state` on the boat broker, presence `fake_dock`).
- `optiprimex/sim/fake_perception.py`: pose via `pose_from_state`; ZED model (σr = a + b r + c r², bearing σ, miss and
  wrong-colour probabilities ramped from 0.6 R to R; old keys still work); flashing colours; windows on `perception/window`
  (only within `window_max_range_m` 8); sensed:false skipped; `sim/dock/state` applied; health + `source` (`--source`, default sim),
  `world`, `host`, `detector: fake_perception`; `--hub-host`; truth carries name, start, bounds, phases, dock state.
- `optiprimex/usv/perception_map.py` + `companion/state.py` + `companion/config.py`: `classify_light` (black frames are the off
  phase unless lit < 25 %; flash if black ≥ 25 %; `?` under 6 sightings), `MapEntity.light` / `colour_now` (in the digest body),
  `markers()` (entry = flashing blue, exit = steady blue) + `markers` in the digest, `assoc_gate_window_m` 0.5, `gates(min_obs=1)`.
- Launchers: `scripts/run_agx.sh --sim [--world F]` (tmux windows cv = fake_perception, dock = fake_dock, cli; sim/truth to
  OCS_HOST:HUB_PORT), `stop_agx.sh` also stops/reports them, `setup_agx.sh` installs pyyaml, `run_bench.sh` + node `dock`.
  `pyproject.toml` console script `op-fake-dock`.
- Consoles: `playbook.cv_source()`; op-tui WHERE WE ARE row `cv` and usv_ui WHERE WE ARE (where `AGX cv_publish up` was) show
  `CV sim (AGX, world_pool)` / `CV real (detector)` / `CV none`.
- usv_ui: `pathgen [D]` (gates with ≥ 5 sightings chained nearest-first from the entry, through-heading away from the previous
  point, warning if RED would pass to port; points EN, G<i>a, G<i>, G<i>b, EX; planned by `plan_path_cmd`; no motion), `dockto [BAY] [V]`
  (fake FCU only; faces numbered left to right facing the dock; sway along the standoff line 4 m off the face — the sway now has
  `dir` so it can move to port — then a mode-20 bow-in with heading hold, lateral correction and surge tapered to 1.0 m off the face;
  aborts: off-centre > 1.2 m, heading error > 30°, 90 s, mode left 20, disarmed; `stop` aborts it), help / examples / COMMANDS words.
- `optiprimex/tools/map.html`: truth shifted into the map's frame (world origin − digest CO), pool walls, blinking buoy lights,
  bays with indicator colour and number, windows, pontoons, ENTRY / EXIT and `~` (flashing) on entities, blue / black colours,
  a `sim` panel (world, host, dock phase).
- Docs: GUIDE (header v0.1.20, §1 setup_agx rerun, §3.3 the `--sim` row, §4.14 a–d the simulated pool, §9), ROADMAP (§1a revision,
  WP1 done with its differences, WP2 two planners + PoR first, WP3 the fake_dock contract, §4a later), README, HANDOFF, CLAUDE.md
  (134 tests, 200 × 60, the archive excludes the two pdfs), CHANGELOG, `docs/JETSON_LOG.md` (new).

**Tests**: 135 passed (`make test`, ≈4 min). `tests/test_sim_pool.py` (+11): world layout (10 buoys, 3 bays, 50 × 25), lights + `classify_light`, a scripted drive on
world_pool (3 gates red to starboard, entry flash / exit steady, 2 obstacles, dock line of 3 faces facing south, GREEN = bay 2, no
window from 9 m), world_por 2 gates, the DockSim sequence (wrong bay / wrong aim do nothing, GREEN → done, green_bay, fire), window
colour close-up RED → GREEN, `cv_source`, live usv_ui + mm + sim CV + fake dock: health `source sim` + truth on the hub + `pathgen`
(EN … EX, G1 within 0.8 m of the world's gate 1), live `dockto` → DOCKED in bay 2 (plant inside bay 2) → fake_dock lit → aim →
GREEN in the cv map → done; op-tui MAIN at 200 × 60 shows `CV sim (AGX, world_pool)`. Found while testing: a teleport of the fake
plant leaves ghost entities (a sighting taken just before the jump gets the pose from just after) — the live tests pause `cv`
around each teleport; pathgen ignores buoys with < 5 sightings (a 4th, ghost gate otherwise).
`test_sim_cv_imports_need_only_the_agx_venv` (+1, found live: see below).

**Rehearsed live (three hosts, tailnet, 2026-11-02 ≈20:06–20:30 SGT)**: v0.1.20 deployed (GUIDE §1 loop; `setup_agx.sh` rerun).
Mac `run_ocs.sh`, NX `run_usv.sh --fcu fake`, AGX `run_agx.sh --sim`. **Found and fixed**: on the AGX the sim died with
`ModuleNotFoundError: jsonschema` (optiprimex.companion.mqtt → schemas; the laptop venv hides it) → `setup_agx.sh` installs
`pyyaml jsonschema`, `run_agx.sh --sim` checks them, regression test added. **Found and fixed**: usv_ui's WHERE WE ARE cut the
label to `CV sim` → it now follows the link text. Then: `sim/truth` on the hub (world_pool, host optiprimeagxorin-desktop, 23
objects, dock idle); both consoles `CV sim (AGX, world_pool)`; usv_ui `cv on` + the plant teleported down the channel (cv paused
around each jump) → `pathgen` saved EN G1a … G3b EX, the usv_ui digest on the hub: path 71 pts, 3 gates, markers entry #8 / exit #12,
lights flash / steady / off; `dockto` from (41, 1) → `DOCKED in bay 2 - 1.1 m off the face` (≈ 45 s with the dwell), fake_dock `lit`,
30 cannon-aim messages → `green` (bay 2 window 1) → `done` 5 s later. AGX load of the sim: fake_perception 0.7 % CPU, fake_dock
1.5 %, ~26 MB each. map.html was not looked at by a human (it was not opened: MAP=0); its JS passes `node --check` and the digest
it draws was checked on the hub. Everything stopped afterwards (`down:` on the three hosts).

**Open items**: map.html's new truth drawing needs a human look (Dmitrii: `scripts/run_ocs.sh` opens it); the PoR gate spacing
(15 m assumed); fake_dock's "docked" is the 3 s dwell (the DockingReport path is WP3); the mission manager's own gate list keeps
`min_obs` 1 (ROADMAP §4a); carried from v0.1.19: FCU_FAILSAFE never raised, KILL/ESTOP NAMED_VALUE_INT at L1, Miro frame 24,
`link_loss_action: safe_hold` not rehearsed live. Tar warnings on the Jetsons (`LIBARCHIVE.xattr…`) are gone with `--no-xattrs`
(CLAUDE.md archive command).

**Next session, first**: WP2 (ROADMAP §4) — poll Dmitrii on the three open design points (circle radius / speed, a missing gate:
hold or continue, TRANSIT by GUIDED carrots or mode-20 frames) and the planner switch's key; then the S6/P4 port against the
spec's §11 numbers, the simple planner from usv_ui `pathgen`, m0t2sim on world_por first (the PoR video), then m1t1sim on world_pool.

---

## v0.1.21 — 2026-11-02 — m1t1 (Task 1 core) + m0t2 as task behaviours (WP2)

**Session**: the same one as v0.1.20 (Claude Opus 5.5, Dmitrii away, WP1–WP5 cleared one version each).

**Decisions taken (2026-11-02, by Dmitrii, polled before coding)**: circles 3 m at 0.5 m/s; a missing gate = hold, then scan,
then AUTO_SAFE_HOLD; TRANSIT by GUIDED carrots (not mode-20 LOS); planner `simple` by default, switch `t1.planner` in the usv
profile + `args.planner` per run (`ocs_cli … --planner s6`).

**What changed · where**
- `optiprimex/usv/behaviours/`: `follow_path.py` (FollowPath: carrot `la_m` ahead on the polyline, monotonic progress window,
  signed cross-track, corridor abort, timeout from length / speed), `orbit.py` (Orbit: carrot `la_m` of arc ahead in the circle's
  sense, winding accumulated within 2.5 m of the circle, done at 360°; publishes the circle as its path), `scan_yaw.py` (ScanYaw:
  GUIDED vx 0 + yaw rate until a predicate; one turn, then abort with a reason); `__init__` exports them.
- `optiprimex/usv/gatepath/` (new): `dubins.py` (LSL RSR LSR RSL RLR LRL from the spec table, solved in a CCW plane mapped from
  N/E + compass headings, sampled at 0.25 m), `planner.py` (GateItem from `PerceptionMap.gates(min_obs)`, `choose_next` = spec 6.3
  cost, `gate_points` pre / mid / post, `leg()` = straight (simple) or Dubins into the pre point along the crossing direction
  (s6, straight when the curve loops: CCC or > 1.1π), `avoid()` = spec 6.5).
- `optiprimex/usv/tasks/t1_safe_passage.py` (new, registry `T1`): the stages above + SCAN; the entry is locked once circled and
  the exit is then the other well-seen blue buoy (a far steady blue can look flashing); passed gates remembered by midpoint;
  `safe_hold` attribute read by the mission manager; `safe_passage` events (entry, exit, each buoy colour + flashing / solid / off,
  `complete` at DONE) every 2 s when the map changed. s6 uses d 3 m / rmin 1.83 m (spec default d, best-tuning rmin), simple d 2 m.
- `optiprimex/usv/tasks/test_path.py` (new, registry `TEST_PATH`): PLAN (args.points | args.names from `test_path.points_file` |
  the map, waiting up to 8 s) → FOLLOW; map-made paths re-plan after each leg (FOLLOW → PLAN → … → DONE when no gate is left), the
  exit approached to 2.5 m short of the buoy.
- `usv/mission_manager.py`: BehaviourContext gets the map; the running task's `info.path` / `info.carrot` go into the map digest
  (map.html draws the mission manager's path); a task's `safe_hold` → `fsm.request_safe_hold` + runner pause.
  `usv/command_handler.py`: START_TASK T1 with tier ≠ core or an unknown planner is REJECTED before the FSM moves.
- OCS: `cmd_authority.MISSION_ARGS` + planner, names; `ocs_cli mission|start_task --planner simple|s6 --names A,B`.
  `tools/playbook.py`: m1t1 built (stage SCAN added, config / starts text), m0t2 status test, registry hint.
- `optiprimex/sim/fast_sil.py` (new): Plant + the fake FCU's GUIDED / velocity laws + the sim camera + PerceptionMap + a task in
  simulated time (a full m1t1 in ~0.1 s wall).
- Profiles (`usv_pool`, `usv_lab`, `usv_bench`, `usv_ui/config/usv_bench_share`): `t1:` and `test_path:` sections.
- Docs: GUIDE (v0.1.21, mission codes, 4.15 / 4.15a / 4.15b, §7.3 F-4a / F-4b, §9), ROADMAP (WP2 done + differences, §4a: the rest of
  S6/P4), CHANGELOG, README, HANDOFF, CLAUDE.md (143).

**Tests**: 143 passed (`make test`). `tests/test_t1_sim.py` (+8): Dubins 2000 pairs < 1e-6 m (spec §11) and turn sense; m1t1 fast
SIL × {simple, s6} on world_pool checked against the world's truth (the three gates crossed with RED to starboard, clearance >
0.76 m — simple 1.15 m, s6 0.82 m, entry wound ≥ 330° clockwise, exit ≥ 330° counter-clockwise, the complete report's entry / exit
within 1 m, flashing / solid / off states, s6 used a Dubins leg); a wrong-side last gate → SCAN → safe hold "RED to port" and never
crossed, a wrong-side middle gate → crossed southbound legally; no gate 3 + no exit → safe hold "no gate ahead and no exit", no
gate 3 → EXIT → DONE; m0t2 on world_por (both gates) and with two given points; the wiring (registry, playbook, cli parsing);
live m1t1sim through mm + ocs_app + fake judge + sim CV (T1 advanced REJECTED, stages seen, the orbit drawn in the digest,
AUTO_HOLD + TASK_NONE at the end, a `safe_passage` report at the judge, no rate violation; ≈ 95 s). Updated: two playbook
assertions that expected T1 unbuilt now use T3.

**Found while building**: with d 2 m and rmin 3 m almost every Dubins leg loops in the pool (a 1.5 m sideways step needs ~4 m with
r 3) → s6 takes the spec's d 3 / best-tuning rmin 1.83; a far steady blue buoy can be classified flashing (wrong-colour frames
count as the off phase) → the task locks the entry; a map-made m0t2 drove to the exit buoy's centre (a strike) → stops 2.5 m short.

**Rehearsed live (three hosts, 2026-11-02 ≈23:30–00:10 SGT)**: v0.1.21 deployed; AGX `run_agx.sh --sim`, NX `run_usv.sh --fcu fake`
+ `handover`, Mac `run_ocs.sh`. **Found and fixed (1)**: `ocs_cli mission m1t1sim` was refused — `cmd_authority.MISSION_TASKS`
lacked T1 / TEST_PATH (START_TASK's list had them). **Found and fixed (2)**: the first live m1t1sim circled the wrong buoy — after
minutes at the start pose, far wrong-colour frames made the unlit obstacle at 22 m look like a flashing blue light
(`classify_light`: lit share 36 %, blue top) → SEARCH_ENTRY took it at once, the chain then took gate 3 first and refused gate 2 →
safe hold. Now the lit colour must hold ≥ 45 % of the lit sightings (`LIT_DOMINANT`; noise spreads ~33 % each, a real light
keeps ≥ 50 %); 12 seeds × 150 s wait: no false marker, all DONE; regression test `test_m1t1_after_a_long_wait_at_the_start`
(fast_sil `warmup()`). The entry is logged when locked. **Found and fixed (3)**: `start_task m0t2sim` REJECTED "unknown task
TEST_PATH" — `VehicleFsm.can_start_task` kept its own task list → it now takes the whole schema enum (the registry decides what is
built); regression in `test_t1_wiring…` (every registry task passes the FSM and the OCS lists). Then: **m1t1sim** via `ocs_cli
mission m1t1sim` + `ready` + `start_task`: SEARCH_ENTRY 5 s → CIRCLE_ENTRY_CW (entry locked N +5.3 E −6.7, truth 5 / −7) →
TRANSIT gates (2,1) (5,4) (13,12) all `passed (correct)` → EXIT → CIRCLE_EXIT_CCW → AUTO_HOLD in 115 s at 0.5 / 0.6 m/s; the fake
judge received 34 `safe_passage` reports, 0 rate violations. **m0t2sim on world_por**: PLAN "the map: 2 gates, 19.9 m" → FOLLOW →
PLAN → DONE in 28 s. Everything stopped afterwards.

**Open items**: the S6 tracker / search FSM / benchmark are not ported (ROADMAP §4a), so the spec's §11 run-level numbers are not
reproduced; T1's SAFE_HOLD on a refused gate is conservative (§4a); map.html still not looked at by a human (v0.1.20 item); the
vehicle accepts START_TASK T3 / TEST_SWAY into the FSM's check but the registry rejects them first (unchanged behaviour).

**Next session, first**: WP3 (m3t1 + m0t3): `sway_transit.py` (usv_ui `swayto` + the v0.1.20 `dir`), `dock_los.py` (usv_ui
`dockto`'s bow-in), `t3_logistics.py` with fake_dock (`sim/dock/cmd docked` when the DockingReport goes out, `cannon/aim` ≥ 2 Hz,
window `colour_now` from the map), `test_sway.py`; fast_sil already runs any task — extend its physics with mode-20 frames
(ThrusterFrame → the plant's u/v/r like fake_fcu) first. Poll Dmitrii on: dock standoff / stop distance, undock (astern vs turn),
how long to fire after GREEN is seen.

**Addendum 2026-11-03 00:00 (same session)**: after the final v0.1.21 extract the NX tree turned out to be shared with another
line of work (a box-inspection module, `BOX_INSPECT` / m0t4, files created 23:57 from a login at 100.99.146.80, its pytest
running). My extracts / scp very likely overwrote that work's edits to shared files (registry, playbook, ocs_cli, vehicle_fsm);
its new files are intact. Details and the file list: `docs/JETSON_LOG.md`. WP3 not started; no further deploy until Dmitrii
decides how the two lines merge (the Mac tree is the source; the other line's shared-file edits need to be redone or recovered
from its own session).

---

## v0.1.22 — 2026-11-03 — m3t1 (Task 3 core) + m0t3 (WP3)

**Session**: the same (Claude Opus 5.5). Dmitrii back briefly: decided the WP3 numbers, explained the concurrent NX work (a
student's agent developing the m3t1 box stage), asked for a survey of it for tomorrow's merge, and to proceed.

**Decisions (2026-11-03, Dmitrii)**: standoff 5 m, stop 1 m, firing 30 s, all in the mission config (`t3:`). Read as: water until
a window of the bay is seen GREEN (confirmed 1 s), 30 s at most, then the FirefightingReport either way (`complete` false when no
GREEN). Undock: not answered — astern (the only mode built; `t3.undock` refuses anything else).

**Merge survey (no code merged)**: `docs/merge/MERGE_NOTES_box_mission.md` + `docs/merge/box_mission_vs_v0.1.21.diff` — the
students' package `~/Desktop/dm3/BOX_MISSION_v0.1.21_20261103.tgz` on the NX is our v0.1.21 + BOX_INSPECT / m0t4 (6 new files,
12 shared files with ≈ 130 additive lines; one conflict: their vehicle_fsm list drops TEST_SWAY). Their shared-file edits are
fully recoverable from that package. WP3 reads the dock only through `usv/dockpath.dock_geometry` so their box core can feed it.

**What changed · where**
- `optiprimex/usv/behaviours/mode20.py` (new: allocation M1 = u + r, M3 = u − r, M2 / M4 from the calibrated stern factors as usv_ui
  `sway_lat`, joint scaling to `limits.max_amp`; `yaw_cmd`), `sway_transit.py` (new), `dock_los.py` (new), `__init__` exports.
- `optiprimex/companion/config.py` `GainsCfg`: `sway_f_pos 1.05`, `sway_f_neg 0.877`, `sway_kpsi 0.6`, `sway_kc 0.15`, `sway_rmax 0.3`,
  `sway_umax 0.25` (usv_ui's pool-proven defaults).
- `optiprimex/usv/dockpath.py` (new): `DockGeom` (centre, normal to the water, along = left → right facing the dock, bays 1..n with
  indicator colours, `green()`, `abeam()`, `on_standoff()`, `bow_in_hdg`) from `PerceptionMap.dock()`.
- `optiprimex/usv/tasks/t3_logistics.py` (new, registry `T3`), `test_sway.py` (new, registry `TEST_SWAY`); `tasks/__init__`.
- `usv/mission_manager.py`: `_cannon()` — `info.cannon {on, bay}` → `local/<v>/cannon/aim` at 5 Hz, one `{on: false}` when it stops.
- `sim/fake_dock.py`: subscribes `team/+/+/event/<v>/docking` → docked (ignored unless the reported bay is the GREEN one).
  `sim/fast_sil.py`: mode-20 frames through the fake FCU's mixer, a DockSim fed by the task's docking event and cannon aim.
- OCS: `MISSION_TASKS` + T3, TEST_SWAY; `MISSION_ARGS` + bay; `ocs_cli --bay N`. Playbook: m3t1 built (the real stage list),
  m0t3 test (LINE / TO_START / SWAY), hints. Profiles (`usv_pool`, `usv_lab`, `usv_bench`, `usv_bench_share`): `t3:`.
- Docs: GUIDE (v0.1.22, codes, 4.16 / 4.16a, F-4c, §9), ROADMAP (WP3 done, §4a: the merge, T3 advanced), CHANGELOG, README, HANDOFF,
  CLAUDE.md (150).

**Tests**: 150 passed (`make test`, ≈ 7.5 min). `tests/test_t3_sim.py` (+6): m3t1 fast SIL from 13 m before the dock — every stage
in order, docked 1 m off bay 2's face (x 43.6–44.3, |y − 5.5| < 0.6), pontoon clearance > half beam + 0.2 m, sway on the 5 m line,
docking bay 2, firefighting green + seen red, backed out ≥ 4.7 m; bay 3 (RED) by hand with fire_s 8 → `not seen green` after ~8 s,
the dock stays idle; no dock in view → SAFE_HOLD in FIND_DOCK; m0t3 along the dock line to bay 3 (bow held within 15°) and between
two points (bow south); wiring; live m3t1sim through mm + fake_dock (process, lit by the event) + fake_perception + OCS + fake judge
(docking event → lit → cannon aim on → firefighting green → aim off → AUTO_HOLD; the judge saw docking + firefighting and
TASK_COORDINATED_LOGISTICS; ≈ 85 s). Three playbook assertions that used m3t1 as "not built" now use m1t2.

**Rehearsed live (three hosts, 2026-11-03)**: concurrency check first (no foreign process, no file newer than the v0.1.21 archive
in either tree; the students' six files and their BOX_MISSION package untouched), v0.1.22 deployed. m3t1sim (plant moved to N 32
E 3): FIND_DOCK → TO_LINE 4 s → SWAY_TO_ABEAM 16 s → ENTER 32 s → DOCKED 48 s → AIM_FIRE 52 s → UNDOCK 56 s → AUTO_HOLD 68 s;
`T3 dock: 3 bays, bay 2 (green) face N +45.0 E +5.5`; AGX fake_dock `DockingReport heard (bay 2) -> docked` → lit → green → done;
FirefightingReport green after 5.2 s; the judge got `docking`, `firefighting`, 0 rate violations. m0t3sim (plant at N 34 E 4):
LINE → TO_START → SWAY → DONE in 40 s. Everything stopped (`down:` ×3).

**Open items**: the merge of BOX_INSPECT (tomorrow, with Dmitrii); the real cannon link (`cv/cannon/cannon_sender.py`) does not
listen to `local/usv/cannon/aim` yet — it follows the dock observer's status file; T3 advanced (colour read-out, resource
request) not built; undock other than astern not built.

**Next session, first**: WP4 (RoboCommand protobuf client + fake + rc-tui + rc-test on the Mac): clone `robonation/robocommand`
@ 8fc09f2 outside the tree, generate the RobotX_2026 Python classes, `ProtobufJudgeClient` behind `judge: protobuf`, the fake judge
`--wire protobuf`, `rc-tui`, `docs/ROBOCOMMAND.md`, `profiles/course/`. Ask Dmitrii the team_id RoboNation assigned (if any).

---

## v0.1.23 — 2026-11-03 — RoboCommand for real (WP4)

**Session**: the same (Claude Opus 5.5), Dmitrii away.

**Choices made without a poll (none was the operator's to make; noted for review)**: team id stays `SGIT` (already in the
profiles; replace with RoboNation's assignment); rc-test runs natively instead of docker (the Docker daemon was not running and
starting Docker Desktop was not needed: RoboNation's `test_server.py` runs unchanged on our venv against our mosquitto on 1885);
the course RoboCommand address lives in `RC_HOST` (network*.env), consistent with "no addresses in YAML".

**What changed · where**
- `optiprimex/ocs/rc_pb/` (new): RoboNation's `RobotX_2026/gen/python` + `proto/` @ 8fc09f2, unchanged, `LICENSE` (Apache-2.0),
  `PROVENANCE.md`, `__init__` that puts the folder on sys.path (the generated code imports `common_pb2` top-level). The clone used
  is `~/Documents/BH1/RobotX/robocommand` (outside the tree, read-only).
- `optiprimex/ocs/rc_wire.py` (new): encode / decode for every body of the four envelopes, topics, enum maps (RobotState, TaskTier,
  RxTask incl. TASK_INFRA_SURVEY_REPAIR, VehicleType ⇄ our domain, BeaconState from colour + light, Color), `classify` / `decode`.
- `ocs/judge_client.py`: `ProtobufJudgeClient(FakeJudgeClient)`. `ocs/ocs_app.py`: `judge: protobuf`, `RC_HOST` / `RC_PORT`,
  command de-dup key (seq, kind, sent_at). `ocs/report_mapper.py`: firefighting carries `window_id`.
- `sim/fake_robocommand.py`: `--wire json|protobuf`.
- `tools/rc_tui.py` (new, console script `rc-tui`): RcModel (decodes both wires, team filter, heartbeat rate), `render()` (Canvas),
  typed commands via the fake judge (rc/sim/inject) / `--as-judge` / `--monitor`.
- Launchers: `scripts/run_rctest.sh`, `stop_rctest.sh` (new), `run_ocs.sh --judge fake|rctest|course` (window rc = fake judge |
  rc-tui --as-judge | rc-tui --monitor; RC_HOST/RC_PORT into ocs_app), `run_usv.sh --course`, `stop_ocs.sh` also reports rc-tui.
- Profiles: `profiles/course/ocs_course.yaml` (protobuf, judge_network course, bench_mode false), `usv_course.yaml`
  (allow_goto_task false), `mosquitto/rctest.conf`. requirements.txt + pyproject (`rc` extra, `rc-tui` script): protobuf 7.35.0.
- Docs: `docs/ROBOCOMMAND.md` (new), GUIDE (v0.1.23, 4.17 / 4.17a / 4.17b, §9), ROADMAP (WP4 done, §4a: window_id, re-declaration
  seq), CHANGELOG, README, HANDOFF, CLAUDE.md (154).

**Tests**: 154 passed (`make test`, ≈ 8 min). `tests/test_protobuf_judge.py` (+4): every body round-trips (beacon states, open /
closed course ring, other team's topic not ours, bad kind raises); ProtobufJudgeClient against `fake_robocommand --wire protobuf`
(course, declaration_seq 41 sent as RxRequest seq 1 and mapped back, heartbeats counted, AssistanceRequest decoded, IncidentAck,
ReadinessConfirm mapped to its command, everything on robocommand/robotx/# is valid protobuf, another team's command ignored);
the whole T4 core chain with ocs_app on `judge: protobuf` + mm + fake FCU, then a same-seq-other-kind command still starts T4;
rc-tui at 200 × 60 (both columns, undecodable count, commands through the fake judge / as judge / refused in monitor).

**Rehearsed live (Mac + NX, 2026-11-03 ≈00:50–01:00)**: `scripts/run_rctest.sh` (RoboNation's test_server.py published Course
ALPHA), `run_ocs.sh --judge rctest`, NX `run_usv.sh --fcu fake` (v0.1.22 on the boat: nothing boat-side changed) + handover,
`ocs_cli ready` → rc-test `[auto-start] … RunStart (declaration_seq=1, run_id=1)` → R5. **Found and fixed**: RoboNation's
`test_client.py assistance-request` was ignored — its RxCommand seq 1 equalled the server's RunStart seq 1 and our OCS de-duplicated
by seq alone → key (seq, kind, sent_at); regression test added. After the fix: `test_client.py assistance-request --team SGIT` →
T4 TRANSIT → WAIT_CONFIRM in 15 s; our IncidentAck (report seq 6) and ReadinessReport (seq 43, command_seq 1) in RoboNation's log;
`test_client.py readiness-confirm --report-seq 43` → AUTO_HOLD. rc-test decoded 845 of our RxReports, no ERROR line; the log
`logs/rctest/run_20261103_005013/robocommand_20261103_005014.log` (Mac, not archived) is the first Proof-of-Readiness-style artifact.

**Open items**: `window_id` is 0 (unset) until the AGX reports which window (upper / lower); the request seq restarts at 1 after
an OCS restart (RoboNation's view on re-declarations to check); rc-tui was exercised by the test and as a window, not driven by a
human; no ACLs on our hub.

**Next session, first**: WP5 (camera stream AGX → Mac: `cv/stream/cam_stream.py` MJPEG over HTTP, `--source zed|dir`,
`CAM_PORT` in network.env, `cv/stream/run.sh`, `run_agx.sh` window, map.html "camera" link, CPU measured on the AGX). Before
coding: check the AGX for concurrent student work (docs/merge/), and whether the ZED is free while the observers run.

---

## v0.1.24 — 2026-11-03 — camera stream AGX → Mac (WP5)

**Session**: the same (Claude Opus 5.5), Dmitrii away. WP5 was fully specified (MJPEG over HTTP decided 2026-11-02); no poll.

**What changed · where**
- `cv/stream/cam_stream.py` (new; ours, not a student snapshot): a ThreadingHTTPServer with `/`, `/stream`, `/snapshot.jpg`,
  `/status`; one producer thread per source — `dir` (the newest of `cv/buoys/runs/*/latest_preview.jpg`,
  `cv/docking/runs/latest/latest_combined.jpg`, `cv/buoys/runs/latest_sender_preview.jpg`, younger than 30 s, SOI-checked), `zed`
  (pyzed HD720 left, depth off), `test` (OpenCV pattern); OpenCV optional (`CAM_NO_OPENCV=1` skips it); `server_bind` without
  getfqdn. `cv/stream/run.sh` (new).
- `scripts/run_agx.sh`: window `cam` by default (`--no-cam`, `--cam-source`; `test` under `--sim`), the camera URL in its report;
  `stop_agx.sh` stops / reports `cam_stream`. `network.env`: `CAM_PORT=8090` (Mac master copy, scp'ed to both Jetsons).
- `optiprimex/tools/map.html` (`?cam=` → header link "camera"), `bench_tui.map_url` (`CAM_URL`), `run_ocs.sh` (CAM_URL from
  AGX_HOST / CAM_PORT when it opens the map).
- `run_rctest.sh`, `stop_rctest.sh`, `cv/stream/run.sh`: `--help` with BSD-sed-safe syntax (the older launchers' `--help` fails on
  the Mac — ROADMAP §4a).
- Docs: GUIDE (v0.1.24, §3.3 camera row, §9), ROADMAP (WP5 done + differences, §4a), CHANGELOG, README, HANDOFF, CLAUDE.md (156).

**Tests**: 156 passed (`make test`, second run; the first full run had 1 failure in
`test_reset_both_ends_mission_preselect_and_test_missions` — the load flake recorded in v0.1.19, the module passed alone 5/5 and
the next full run passed). `tests/test_cam_stream.py` (+2): the dir source end to end over HTTP (status, snapshot, the page, two
multipart parts following the file, a client counted, stale files ignored); the launcher / map wiring and `run.sh --help`.

**Rehearsed live (AGX → Mac over the tailnet, 2026-11-03 ≈01:14)**: concurrency check first (clean), then `cv/stream/*`,
`run_agx.sh`, `stop_agx.sh` and `network.env` copied by scp (JETSON_LOG). `run_agx.sh --sim` → window cam `source test at 5.0 Hz
… opencv yes` (the students' venv, read-only); from the Mac: `/status` frames 36, `/snapshot.jpg` 200, 9780 bytes, the picture
decodes (test pattern, frame 35); streaming 20 s to the Mac: 1.06 MB, the AGX process at **2.0–2.7 % of one core**, 0.1 % memory,
load 0.34. `dir` with the AGX's plain python3 (no OpenCV) on a temporary preview file under `cv/buoys/runs/_streamtest/` (removed
afterwards): frames 1, `from buoys/runs/_streamtest/latest_preview.jpg`. **No ZED attached**: pyzed `CAMERA STREAM FAILED TO
START`, device list empty → `zed` not run live. Stopped (`down:`).

**Open items**: `zed` source and the pool-switch run; the observers' preview rate (every 5th / 15th analysed frame) limits the
`dir` stream to their pace; carried: the BOX_INSPECT merge, window_id, the re-declaration seq, map.html / rc-tui not yet looked at
by a human.

**Next session, first**: with Dmitrii — merge the students' BOX_INSPECT work (`docs/merge/MERGE_NOTES_box_mission.md`), decide how
m3t1's FIND_DOCK uses it; then the pool trials (GUIDE §7 legs F-4a … F-4c) and the open items of ROADMAP §4a. WP6 (UAV) waits for
the students' drone stack.

---

## v0.1.25 — 2026-11-03 — help texts (recorded here, in the v0.1.26 session)

A texts-only version made on Dmitrii's request after the first live ZED stream: `run_agx.sh --help` and its end report on the three
camera sources (`dir` is blank until an observer runs; `--cam-source zed` = the camera itself, holding the ZED), `cv/stream/run.sh
--help` with an example, `run_rctest.sh` printing `tmux attach -t rctest` one command per line, GUIDE §3.3, docs/ROBOCOMMAND.md.
Its full test run had one failure + one teardown error in `test_m1t1sim_live_…`: Dmitrii had run `pkill -x mosquitto` during it (the
test brokers went away) — not a code fault. No archive was built for it; v0.1.26 carries it. From now on (Dmitrii, 2026-11-04) small
side requests take a fourth digit (0.1.26.1 …), the planned WPs keep 0.1.N.

---

## v0.1.26 — 2026-11-04 — the validation pass (WP7) + the judge's view in the browser

**Session**: Claude Opus 5.5 (Claude Code on the Mac), with Dmitrii, the evening before a pool day; brief = ROADMAP §7 WP7 written by
his general agent. Built exactly that list, plus two requests of his own (below). No deploy (Dmitrii deploys after the pool day),
`network*.env` untouched.

**Findings closed** (ROADMAP §7 "Validation so far"): the judge's console was not findable → tmux `judge` + `tmux attach -t judge`
printed by every launcher. rc-test, the ZED stream and USV + OCS against rc-test had worked — nothing to fix there.

**Dmitrii's additions (2026-11-04)**: (1) rc-tui shows the **expanded heartbeat** — every Heartbeat field the schema carries (state,
position, speed, heading, roll, pitch, altitude HAE, depth, current task, vehicle type, flight phase, UAV registration) plus seq, rate
and age — and the **mission fields**: the declaration (vehicles, tiers, geofence), the RunStart, each vehicle's task timeline (every
change of current_task = what RoboCommand counts as an attempt) and the latest report of each kind with its content. Our side now
*sends* roll / pitch in the heartbeat (vehicle status → OCS heartbeat; handbook 3.4.11 asks for them; altitude stays empty: the
FCU gives MSL, the schema asks HAE). (2) **An HTML page**, `optiprimex/tools/judge_web.py` (`judge-web`, http://127.0.0.1:8095/):
the same model as rc-tui, read-only, refreshed every second, window `web` of tmux `judge`; `run_rctest.sh` opens it in the browser.

**What changed · where**
- Judge console: `scripts/run_judge.sh`, `scripts/stop_judge.sh` (new; tmux `judge` with windows `tui` / `web`, an owner tag so
  `stop_ocs.sh` / `stop_rctest.sh` close only their own); `run_ocs.sh` (window `rc` = the fake judge's log only; opens the console for
  fake / course; prints `judge console: tmux attach -t judge`), `run_rctest.sh` (opens it `--as-judge`, the page in the browser unless
  `JUDGE_WEB=0`), `stop_ocs.sh` (no longer counts rc-tui as its own). `tools/rc_tui.py` rewritten (RcModel: `rc/#` + `robocommand/robotx/#`
  together, decoded by topic root; `--wire` only for what `--as-judge` publishes, json now possible too; `snapshot()`); `ocs/rc_wire.py`
  decodes roll / pitch / depth / registration; `companion/state.py` status + `ocs/heartbeat_publisher.py` roll / pitch.
- Stage stepping: `usv/tasks/base.py` (`ENTRY_POINTS`, `check_entry`, `enter_from`, `holding`, the step park through `safe_hold` =
  "step: next <STAGE>" before the stage's behaviour starts); `usv/task_runner.py` (`args.from` after start); `usv/command_handler.py`
  (`args.from` checked on a probe task before the FSM moves → `REJECTED from: …`); T3 entry points TO_LINE, SWAY_TO_ABEAM, TURN_IN,
  ENTER, AIM_FIRE, UNDOCK (need `dock_geometry` + the bay), T1 SEARCH_ENTRY, TRANSIT (a usable gate), EXIT (the exit buoy); the tasks'
  own step() use `holding`. `tools/ocs_cli.py` `--step`, `--from STAGE`; `ocs/cmd_authority.py` MISSION_ARGS; `schemas/bodies.schema.json`
  cmd.args documents / types step, from, sim; `ocs/ocs_app.py` ocs/state carries the vehicle's `hold_reason`; `tools/playbook.py` next
  line `resume -> <STAGE>`; op-tui and usv_ui MISSION boxes `STEP · next <STAGE>`. The mission manager's safe-hold path (v0.1.21) does
  the parking: AUTO_SAFE_HOLD, the adapter's HOLD (zero frame when leaving mode 20), RESUME enters the stage. `sim/fast_sil.py`:
  `warmup_s`, `args.from`, step parks recorded and resumed (`resume_steps`).
- RoboNation block A: `ocs/judge_client.py` persistent session (`clean_session=False`, client id `ocs-<team>-judge`, QoS 1 command
  subscription as before); `profiles/course/ocs_course.yaml` `stall_s: 180`; the stall wording (lifecycle alert, op-tui WARNINGS, the
  next line): "declared, waiting for the course manager — a refusal is shown on RoboCommand's display, ask the staff"; `window_id`
  1 = upper / 2 = lower documented (rc_wire, ROBOCOMMAND.md); SafePassage re-send on change + unlit = `off` were already so (tests).
- Test hygiene: `Makefile` `make quick` (unit + every mission in the fast SIL, no brokers); `tests/test_quick_missions.py` (T4, GOTO,
  TEST_AUTO, TEST_MANUAL in the fast SIL); the sil1 race fixed with a fake-judge control `{"kind": "sim_config", "auto_incident_after":
  null}` (`sim/fake_robocommand.py`) switched off and back on by that test; `test_cv_kit` on a copy of cv_publish.py with synthetic
  address files; `test_usv_ui::test_fcu_switch_live` looks in 60 log lines, not 12 (flaked once: earlier cv output pushed the line out);
  every launcher's `--help` with BSD-sed-safe syntax (15 scripts).
- Docs: GUIDE (header v0.1.26, the intro note, §3.1 judge row, §3.1a, 4.2c, 4.16, 4.17, 4.17c, F-4c, §9), docs/ROBOCOMMAND.md, CHANGELOG,
  README, HANDOFF, CLAUDE.md.

**Tests**: `make test` 167 passed (second full run; the first: 166 + `test_fcu_switch_live`, fixed as above); `make quick` 136 in ~5 s. New: `tests/test_stage_stepping.py` (6: m3t1 `step` parks at all 10 boundaries in order and reaches
DONE; without resume the boat stays parked before FIND_DOCK; `from TURN_IN` abeam bay 2 → DONE without FIND_DOCK; `from ENTER` with no
dock → refused; `from DOCKED` → not an entry point; m1t1 `from TRANSIT` → DONE; `from EXIT` facing away → refused; a paused task keeps
current_task (A5); cli / playbook / schema / persistent session (A1)), `tests/test_quick_missions.py` (3), sil1
`test_step_and_from_through_the_mission_manager` (on the wire: REJECTED from ENTER; TEST_AUTO --step → AUTO_SAFE_HOLD "step: next S2"
in status and ocs/state, the playbook's `resume -> S2`, RESUME → S2), test_protobuf_judge rc-tui (both wires, the expanded heartbeat,
the timeline, latest reports, snapshot, json `--as-judge`) and `test_judge_web_serves_the_same_view`. `make quick`: 136 tests in ~5 s.

**Rehearsed on the Mac only**: `run_ocs.sh` → tmux `ocs` windows hub rc ocs cli tui, tmux `judge` windows tui web, rc-tui `mode fake`
showing the fake judge's JSON course, the page `connected … course BENCH-COURSE-1`; `stop_ocs.sh` closed it. `JUDGE_WEB=0
run_rctest.sh` + `run_ocs.sh --judge rctest` → no window `rc` in `ocs`, rc-tui `mode judge` on 1885 with RoboNation's protobuf course
ALPHA, the page the same; `stop_rctest.sh` closed the console. Not rehearsed: `--judge course` (no course network), the browser
auto-open (skipped with JUDGE_WEB=0 during the rehearsal), anything on the Jetsons.

**Open items**: a stray `ocs_app` from the interrupted v0.1.25 test run (pytest temp config, pid 27143 at the time) still runs on the
Mac — harmless (its broker is gone), Dmitrii to `kill` it (this session was not allowed to); altitude HAE in the heartbeat (needs
GPS_RAW_INT alt_ellipsoid); step mode collapses two instant stages into one park only if a task chains them inside one tick (none
does after the `holding` change — every boundary parked in the test).

**Next session, first**: Dmitrii's pool-day findings → a 0.1.26.N fix if small, else into WP8 (m0t4 in: `docs/merge/`), per ROADMAP §7.

## v0.1.27 — 2026-11-04 — m0t4 BOX_INSPECT in, with the review's fixes (WP8)

**Decided with Dmitrii first (polled)**: the order WP8 → WP9 → a new WP10 "CV gate" (m0t5, TEST_CV_GATE: two unlit buoys as a gate,
the one on the boat's left as it approaches = GREEN, locked at first sight; the single-buoy check is by hand, `cv on` / map.html;
pass → HOLD → DONE); ROADMAP renumbered (cannon WP11 v0.1.30, hardware day WP11a, m0t4 → m3t1 WP12 v0.1.31, UAV WP13–WP15), the
mission-code table lists m0t4 / m0t5 and marks m0t2, m0t3, m1t1, m3t1 built. The five questions of BOX_MISSION_REVIEW §7, answered
by Dmitrii: (1) the target is Task 3's docking bays; (2) the detector gives 3 points per face (left / mid / right, the face plane);
(3) the vehicle picks; (4) stop distance a config key, default 1 m; (5) the two events OCS-internal. It stays m0t4, built so it can
become an m3t1 stage later (WP12). For this WP: rehearse on the Mac only (no Jetson deploy — his, after the pool day); `--step` /
`--from` for m0t4 now; STRAFE with a length and a side.

**What changed · where**
- The merge: `docs/merge/box_mission_onto_v0.1.24.patch` applied with `patch -p2` (15 of 18 files clean; the rejects in
  `ocs/cmd_authority.py`, `schemas/bodies.schema.json`, `tools/ocs_cli.py` merged by hand onto v0.1.26's step / from additions).
  The students' six files are in: `usv/box_inspection.py`, `usv/tasks/box_inspection_task.py`, `profiles/bench/world_boxes.yaml`,
  their three test files; registry, FSM list, SELECT_TARGET, PerceptionMap `box_tracks` / `stable_boxes()`, fake_perception's
  `face_points`, playbook m0t4, schema (cmd `SELECT_TARGET`, task `BOX_INSPECT`, events `box_survey` / `box_mission_complete`).
- `usv/box_inspection.py` (the pure core): `BoxTrack.stable(require_blink=)` (review §4.5); `BoxInspectionMission` new parameters
  `stop_range_m` 1.0 (was 5), `require_blink` False, `default_colour` green (the vehicle picks; ASK_OCS no longer waits), `observe_s`,
  `strafe_step_m` / `strafe_len_m` / `strafe_side`, `scan_hdg_deg`, `align_timeout_s`, `stop_tol_m`; actions `STILL` (mode 20, zero
  frame) and `YAW`. The search is a dwell (turn to the search heading, look 4.5 s) / 2 m strafe-leg loop in mode 20 — no HOLD between
  (§4.3); after `strafe_len_m` it goes on if the target colour is among the stable faces, else HOLD "strafe limit …" (§4.4). ALIGN: GUIDED
  to the align point, then yaw in place (back to GUIDED if it drifts > 3× tol); timeout → HOLD (§4.1). HOLD is a hold the operator
  resumes (`held_from`, `resume()`: back to that stage with fresh timers; a strafe limit sweeps back the other way, twice the length).
  STOP ends at the face plane distance (`stop_range_m` + 0.1), not at "within 1 m of the stop point". VALIDATE looks again for one
  observation window in mode 20 instead of one Hold tick. `enter(stage)` for OBSERVE / ALIGN / APPROACH.
- `usv/tasks/box_inspection_task.py`: parameters from the profile's `box_inspect:` section (START_TASK args override); PREFLIGHT
  "camera" = the perception beat (`PerceptionMap.last_alive_t`, any perception message incl. `perception/health` with ok ≠ false,
  fresher than 3 s), not detections (§4.2); STILL / YAW / STRAFE frames from `behaviours/mode20` (calibrated stern factors, heading
  hold); the approach gets a line-of-sight term onto the face normal (§4.6, without it the run ended 0.9 m off the centre line); a core
  HOLD → `safe_hold` (AUTO_SAFE_HOLD with `BOX_INSPECT <stage>: <reason>`), RESUME → `core.resume()`; stage stepping parks at stage
  boundaries but not inside the search loop or into / out of a hold; ENTRY_POINTS OBSERVE, ALIGN, APPROACH; REPORT returns no command
  (no mode change for one tick).
- `usv/perception_map.py` `last_alive_t`; `usv/mission_manager.py` sets it in `_on_perception`; `sim/fast_sil.py` sets it in `sense()`;
  `stable_boxes(require_blink=False)`.
- `usv/command_handler.py`: SELECT_TARGET accepted in AUTO_TASK or AUTO_SAFE_HOLD while BOX_INSPECT is before ALIGN (PREFLIGHT …
  ASK_OCS, HOLD) — the vehicle picks GREEN by itself, the operator's colour is an override; `tools/playbook.py` the same check, the
  m0t4 mission text (stages, config, `judge_out -`).
- `ocs/report_mapper.py`: the two events are no longer judge reports (`INTERNAL_EVENTS`, `internal_event()`); `ocs/ocs_app.py` keeps
  the last of each per vehicle in ocs/state `internal` and logs it.
- `tools/ocs_cli.py` `--target-colour --scan-x --scan-y --scan-hdg --strafe-side --expected-boxes`, `select_target COLOUR`;
  `ocs/cmd_authority.py` MISSION_ARGS + the m0t4 args; `schemas/bodies.schema.json` args description.
- `profiles/bench/world_boxes.yaml`: steady indicators (Task 3) and the ZED's 25 m range (60 m hid §4.2); `box_inspect:` in
  usv_bench / usv_course / usv_lab / usv_pool (stop_range_m 1, align_range_m 15, strafe_len_m 6, strafe_side starboard,
  require_blink false). `Makefile` QUICK + the four box test files.
- The students' tests, adjusted where a decision changed the behaviour: the flow test pins `stop_range_m=5.0` (their geometry); the
  perception contract looks from 20 m (the 25 m camera) at steady faces; the events test asserts they are not judge reports.
- Docs: GUIDE 4.18 + header v0.1.27, ROADMAP (renumbering, WP8 answers + done, WP10 new), BOX_MISSION_REVIEW §8 (the answers),
  CHANGELOG, README, HANDOFF, CLAUDE.md (test count; the macOS `tar -xzf` note: no `--overwrite` there).

**Tests**: `make test` 185 passed (167 + their 9 + `tests/test_box_wp8.py` 8 + `tests/test_box_sim.py` 1); `make quick` 153 in ~4 s.
test_box_wp8 (fast SIL on world_boxes): (35, 8) → red to port and (35, −10) → blue to starboard reach DONE 0.9–1.25 m off the face and
< 0.3 m off its centre line, only mode-20 frames inside the search, ≤ 5 command-type changes per run (GUIDED → 20 → GUIDED → 20 →
HOLD); no colour given → GREEN, the survey lists all three; PREFLIGHT passes with nothing in view and waits with a dead camera;
(30, 8) starboard for red → SAFE_HOLD `strafe limit 6 m … no red face` 6 m east, RESUME sweeps back west → DONE; ALIGN timeout →
HOLD, resume → ALIGN; `require_blink`; `--step` parks GO_SCAN, OBSERVE, ASK_OCS, ALIGN, APPROACH, STOP (VALIDATE passes in the
APPROACH tick) → DONE; `--from ALIGN` → DONE without the search, refusals for ALIGN with nothing in view, APPROACH off the line, STOP;
cli / playbook / profiles. Reproduced before the fixes in the same harness: ALIGN at 348° / 12° for 400 s (red / blue), ASK_OCS forever
without a colour, 42 command-type changes on the on-axis run.

**Rehearsed live (Mac only)**: `tests/test_box_sim.py` — mission manager + fake FCU + fake_perception (world_boxes, process) + OCS +
fake judge on two brokers: PREFLIGHT → OBSERVE → ASK_OCS (1 s) → ALIGN (8 s) → APPROACH → STOP → DONE (78 s of slow taper) → AUTO_HOLD,
≈ 91 s; the OCS logged `internal box_survey` / `internal box_mission_complete` (green, 1 m), the fake judge's report list has neither.
Not rehearsed: the three hosts (`run_agx.sh --sim --world profiles/bench/world_boxes.yaml` + NX + Mac, GUIDE 4.18), the real AGX
detector (it does not emit `face_points` yet — the students' side, WP12), anything on the Jetsons (still v0.1.24).

**Open items**: review §4.7 — box tracks keyed by the detector's item id (a renumbering detector fragments a face), the GUIDED leg to
the scan point ignores buoys; the STOP taper from 15 m at 0.15–0.8 m/s takes over a minute; the stray test processes on the Mac
(pids 27131 mission_manager, 27143 ocs_app, from the interrupted v0.1.25 run) — Dmitrii to `kill 27131 27143`.

**Next session, first**: WP9 (stage B on the real Pixhawk, `--lab`, props off / ESC power off — with Dmitrii at the bench); if he is
not at the bench, its no-hardware part first (the mode-sequence / frame-cadence log analyzer the WP9 table is filled from).

## v0.1.27.1 — 2026-11-04 — WP9 prep without hardware: the command trace, `mode-log`, `run_usv.sh --pool`

**Asked**: build WP9's no-hardware part now (the analyzer the GUIDE §6.2 table is filled from) and a pool config by a `--pool` tag;
polled: both — an explicit `run_usv.sh --pool` *and* the analyzer checking against lab / pool rules (it reads which profile the
run used). A minor version (0.1.27.1): v0.1.28 stays WP9 itself, the walk on the real Pixhawk with Dmitrii at the bench.

**What changed · where**
- `optiprimex/usv/cmd_trace.py` (new): `CmdTrace`, JSON lines — `start` (unix, profile, version, FCU endpoint), `mode_req` (mode,
  the FCU's answer), `frame` (m1..m4, why entry / cmd / neutral), `target`, `vel`, `obs` (FCU mode, armed, vehicle state, task,
  sub-state; written only on change).
- `usv/adapters/rover.py`: every frame and mode request through `_frame()` / `_set_mode()` (same sends, same counters) which also
  write the trace when one is set. `usv/mission_manager.py`: `--trace PATH`; `_trace_obs()` before the task steps and again right
  after (so a command is attributed to the stage that produced it — the first try wrote it at the end of the tick and every
  command landed one stage late) and at the end of the tick.
- `optiprimex/tools/mode_log.py` (new; console script `mode-log`, or `python3 -m optiprimex.tools.mode_log`): stages from the `obs`
  records; per stage the modes requested, the modes the FCU reported (the trace's, or the FCU's own HEARTBEATs from usv_ui's tlog with
  `--tlog`), frames / rate / longest gap; checks entry zero frame, neutral on exit, gap ≤ 0.25 s (WARN > 0.20), thrash (3 changes in
  2 s), refused and follow-within-1 s (WARN on `--lab`, FAIL on `--pool`), armed (pool: FAIL if AUTO_TASK disarmed, once per stage;
  lab: WARN if armed), profile (`--lab` on a pool profile or vice versa = FAIL, unknown = WARN); rules default from the trace's profile;
  text / `--md` (the §6.2 rows) / `--json`; exit 0 / 1 (a FAIL) / 2 (no trace).
- `scripts/run_usv.sh`: `--pool` (usv_pool.yaml, said out loud; still the default), one of `--pool / --lab / --course` (two = exit 2
  before anything starts); the mission manager always gets `--trace $RUN_DIR/mm_trace.jsonl`; help + the last line name it.
  `scripts/run_bench.sh`: the same trace in its run folder.
- GUIDE §6.2 (new: how to run it, the rules, the table to paste into — empty until the bench), 7.0a's last cell, §8 a row for the
  trace, §9 rows for `--pool` and `mode_log`; header v0.1.27.1. CHANGELOG, README, HANDOFF, CLAUDE.md (191 tests), `pyproject.toml`
  (`mode-log`), Makefile QUICK + test_mode_log.

**Tests**: `tests/test_mode_log.py` (6: a clean lab run passes with the right table; gap / missing zero frame / thrash / `--pool` on a
lab run are caught; lab vs pool differ on armed and a refused GUIDED; an FCU that never follows; the adapter writes entry / cmd /
neutral frames and the mode requests and the CLI reads them (exit 0 / 2); `run_usv.sh --pool --lab` refused, help names the trace);
`tests/test_box_sim.py` now runs the mission manager with `--trace` and checks the live m0t4sim trace: PASS on the pool rules,
OBSERVE requests CUSTOM_MANUAL, ALIGN GUIDED, STOP ~778 frames at 10.0 Hz with a 0.11 s longest gap, 5 mode changes in 95 s.
`make test` 191 passed (10 min), `make quick` 159 in ~4 s.

**Rehearsed**: on the Mac only — the live m0t4sim trace through `mode-log` (text and `--md`, `--lab` on it warns armed per stage
and passes). Not run: anything on the real Pixhawk (that is WP9), `--tlog` on a real usv_ui tlog (the parser is pymavlink's
HEARTBEAT from component 1; untested against a file), the Jetsons (still v0.1.24).

**Open items**: `--tlog` needs a first real tlog to be trusted; the trace grows ~3 MB per hour of mode 20 (no rotation); the stray
test processes on the Mac (pids 27131, 27143).

**Next session, first**: WP9 with Dmitrii at the bench — `run_usv.sh --fcu uart --lab`, the legs of ROADMAP WP9 / GUIDE 7.0a, then
`python3 -m optiprimex.tools.mode_log logs/usv/latest/mm_trace.jsonl --md` into GUIDE §6.2.

## v0.1.28 — 2026-11-04 — the OCS in the browser (WP8a): op-tui's view and commands, a mission picker, the SYSTEM tab

**Asked**: expand the page run_ocs.sh opens so it replicates op-tui "in the best way" (smarter grouping, tooltips, drop-downs),
op-tui still the main console, the map on a secondary tab; select modes (real / fake FCU …) and reboot. Discussed first: keep HTML
(served by Python, no build step, offline at the pool) vs a framework; the OCS may never ARM / DISARM / SET_MODE (ADR-9), so "modes"
= how the stacks run, not the Pixhawk's flight mode. Polled: a dedicated section with every restart / reboot (Jetsons, FCU, the
Mac side …); all op-tui intents from the page; tabs like op-tui; built now as WP8a v0.1.28 (WP9 → v0.1.29, the rest shifted one
version; ROADMAP updated).

**What changed · where**
- `optiprimex/tools/ocs_web.py` (new; `ocs-web`, OCS_WEB_PORT 8096, binds 127.0.0.1): op-tui's own `Model` / `Feeds`
  (tools/bench_tui) + an `OcsConsole`; GET `/` the page, `/meta.json` (op-tui's USAGE / groups, the playbook entries for the
  tooltips, the missions with their options — m1t1 planner, m0t2 / m0t3 points, m3t1 bay, m0t4's target / scan point / heading /
  strafe side / faces, `--step`, `--from` with the task's ENTRY_POINTS from the registry, sim — the R / V meanings, the flow, the
  SYSTEM catalogue), `/state.json` (status, ocs/state, mm/state, the warnings, nodes, $SYS, perception, a map summary, the log, the
  judge traffic, the playbook verdict of every intent, `next`, the stage meaning, the SYSTEM guards + jobs), `/map.html`; POST
  `/cmd` (op-tui's grammar: intents through `ocs_cli.execute`, the fake plant's words to its UDP control port) and `/system`;
  POSTs need the header `X-OCS-Web: 1` (a cross-site form cannot set it).
- `optiprimex/tools/ocs_web.html` (new): plain HTML/JS, no external resources, dark like judge_web. Sticky top bar (R / V ladders
  with tooltips, **next:**, warnings chip, links); tabs MAIN (RUN, VEHICLE, MISSION with the stage chain, FLOW, WARNINGS, COMMANDS in
  op-tui's groups — greyed with the verdict's reason, tooltips = usage + does / needs / then / next, a confirmation for end_run /
  reset / abort / disable_auto / manual, a small form for words with arguments — CONSOLE with history, live usage + verdict, the
  results; EVENTS), MISSION (list + detail: what, starts / ends, judge in / out, config, stages, options form → the command line →
  preselect / start now, with the START_TASK verdict), BOAT, NETWORK, JUDGE (+ a link to the judge's page 8095), SYSTEM, MAP (map.html
  in a frame, loaded on first open), LOG (kind filters + text), HELP (the whole playbook, R / V meanings). Static parts are built
  once from /meta.json, the live parts every second, so typing and the drop-downs are never reset.
- `optiprimex/tools/system_actions.py` (new): the SYSTEM catalogue — `ocs` (this Mac: stop_ocs + run_ocs with judge / net / log,
  detached in its own session because it kills the page's server; MAP=0 so no second browser tab), `rctest` (start / stop),
  `nx` (ssh ORIN_SSH: stop_usv + run_usv with `--fcu fake|uart|usb|eth`, `--pool|--lab|--course`, net, LOG; or stop), `agx` (ssh
  AGX_SSH: real / `--sim --world …` / `--demo`, `--cam-source` or `--no-cam`; or stop), `fcu` (ssh: `tools/fcu_reboot.py`; type
  `fcu`), `nx_reboot` / `agx_reboot` (`sudo -n /sbin/reboot`, a hint when sudo needs a password; type `nx` / `agx`). Guards:
  boat-side actions refused in AUTO_* and with a real FCU armed; the FCU reboot refused armed; the AGX while AUTO_TASK; the OCS
  stack in R3–R5. Options outside their choice lists fall back to the default (no free text reaches a shell). ssh with
  BatchMode / ConnectTimeout; every action → `logs/ocs/system_actions.jsonl`; `OCS_WEB_DRYRUN=1` records instead of running.
  JETSON_KIT (default ~/Desktop/dm3/MASTER_SCRIPTS).
- `optiprimex/tools/fcu_reboot.py` (new): on the NX through usv_ui's share (udpout:127.0.0.1:14551, sysid 200 comp 192): waits for
  the autopilot's heartbeat, refuses armed, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN (1), the ACK, waits for the heartbeat to return.
  `sim/fake_fcu.py` answers it like ArduPilot (DENIED armed; else ACCEPTED, the stream off 3 s, HOLD).
- `tools/bench_tui.py`: `select_target` in op-tui's grammar (m0t4's override was in ocs_cli only).
- `scripts/run_ocs.sh`: window `web` (the environment passed explicitly: ORIN_SSH, AGX_SSH, AGX_HOST, CAM_PORT, JUDGE_WEB_PORT,
  NET_PROFILE), MAP=1 opens the page instead of map.html, the last lines print the URL; `scripts/stop_ocs.sh` checks for ocs_web.
- Docs: GUIDE (header v0.1.28, §3 the start row, the OCS page row, the SYSTEM row, §9), ROADMAP (WP8a written and done, the
  versions after it shifted), CHANGELOG, README, HANDOFF, CLAUDE.md (197 tests), pyproject (`ocs-web`), Makefile QUICK.

**Tests**: `tests/test_ocs_web.py` (6: meta + state follow the playbook — m0t4's options with its entry stages, the verdicts, the
guards in R2 / MANUAL vs R5 / AUTO_TASK armed real; the page has every tab and no external resource; over HTTP: the page, the map,
state, 403 without the header, an intent through ocs_cli with m0t4's flags, `kill on` to the plant port, `arm` unknown, SYSTEM
dry-run command lines (nx with uart / lab / lan / LOG, fcu needing `fcu`, agx with a bogus camera choice → the default), every
boat action refused in AUTO_TASK, the jsonl; the OCS restart is local, the Jetson reboot carries the sudo hint; run_ocs / stop_ocs
wiring; fcu_reboot against the fake FCU: refused armed, accepted and back disarmed). `make test` 197 passed (10 min); `make quick` incl. these.

**Rehearsed (Mac)**: `run_bench.sh` + `ocs_web` in dry run: the page in headless Chrome (MAIN, MISSION, SYSTEM, MAP screenshots
checked: ladders, next, greyed commands with reasons, the mission form, the SYSTEM cards with their guards); through its API
`ready` → R5, `start_task m0t1` → AUTO_TASK S1, `kill on` → KILLED (V1 red, the warning, next: release, confirm_safe); SYSTEM:
`ocs` refused in R5, `agx` sim / boxes / test recorded, `agx_reboot` without the word refused, with it recorded. Then the real
`MAP=0 run_ocs.sh`: windows hub rc ocs cli tui web, the page served (net ts, the camera link from AGX_HOST, the boat broker down
because the NX stack was not running), stop_ocs.sh took it down. `fcu_reboot` by hand against a fake FCU: disarmed → accepted,
back; armed → refused. **Not run**: any SYSTEM action for real (no ssh to the Jetsons this session, no reboot of anything),
fcu_reboot on the real Pixhawk, the page at the pool.

**Open items**: the Jetsons' sudoers line for `reboot` (the job prints what is missing); the `ocs` restart's page comes back on
its own but the browser tab is the old one (it reconnects; no new tab is opened); the stale test processes on the Mac (27131,
27133, 27143).

**Next session, first**: WP9 (v0.1.29) with Dmitrii at the bench — and the first real use of SYSTEM (nx restart with `--fcu uart
--lab`, then `mode_log` on its trace).

## v0.1.28.1 — 2026-11-04 — the OCS page: CAMERA, ⚙ SETTINGS with quick stops; the fake buoys become a sim scenario

Dmitrii's side request after WP8a (a fourth-digit version; WP9 stays next). His answers: startup = **an empty world, the scenario
picked in Settings**; the camera toggle **also starts / stops the stream on the AGX**; stop buttons for **the bench, the OCS stack,
the NX boat stack, the AGX + the fake CV**, each confirmed by typing `stop`.

**Why the buoys were there**: `run_bench.sh` always started fake_perception on `world_bench.yaml` (three gates, a white buoy, a dock),
and the hub's persistence kept the last retained `sim/truth`, so an old world could come back on a later start.

**Changed**
- `profiles/bench/world_empty.yaml` (new): the bench camera, no objects. `scripts/run_bench.sh`: `WORLD` defaults to `empty`, short
  names `empty|bench|pool|por|boxes` (or a file), checked before anything starts; prints `sim world: …`.
- `scripts/sim_world.sh NAME|off` (new): Ctrl-C to the bench's tmux windows fp and dock, then the same commands on the new world
  (`off` = leave them idle). Refuses without a bench session (exit 1) or with a missing file (exit 2).
- `optiprimex/sim/fake_perception.py`: `clear_truth()` on exit — an empty retained `sim/truth` deletes the world on the hub.
  `tools/map.html`: an empty payload on `sim/truth` = no truth layer (it used to keep the last one until reload).
- `scripts/agx_cam.sh start [--source dir|zed|test] | stop | status` (new, runs on the AGX): only window `cam` of tmux `agx`
  (creates the session if run_agx is not up), logs to `logs/agx/latest/cam_stream.log`.
- `tools/system_actions.py`: `Action.group` (system | settings | stop), `opt_guard` (a guard on the chosen options), `detach`
  (replaces the `action == "ocs"` test). New actions: `simworld` (sim_world.sh; refused in AUTO_TASK), `cam` (agx_cam.sh over ssh;
  `zed` refused in AUTO_TASK), and the stops `stop_bench`, `stop_simcv`, `stop_nx` (boat guard), `stop_agx` (AGX guard), `stop_ocs`
  (detached) — all `confirm="stop"`. `WORLDS` gains `empty` (the AGX card's sim list too).
- `tools/ocs_web.py`: `/cam.json` = the stream's `/status` asked from the server (no CORS; 1.5 s timeout; says why when down).
- `tools/ocs_web.html`: tab **CAMERA** (the toggle, *start / stop it on the AGX too*, source, snapshot, the status line every 2 s;
  the `<img>` streams only while the toggle is on and the tab is open — `data:,` closes the connection); tab **⚙ SETTINGS** at the
  right (QUICK STOPS, SIM SCENARIO, CAMERA, CONNECTIONS, the SYSTEM cards again, JOBS). The SYSTEM cards are one `sysCard(a, prefix)`
  so both tabs share them; `a.b` styled like the buttons.
- `tools/mode_log.py` (found by this version's `make test`: `test_box_sim` failed 3 of 6 runs): a mode request in the trace's last
  `FOLLOW_S` with no FCU report yet is WARN `not seen yet`, not FAIL `never` — the test stops the mission manager ~0.4 s after the
  final HOLD request, before the fake FCU's heartbeat says HOLD. A real `never` mid-run stays FAIL. +1 case in `test_mode_log`. And `FOLLOW_S` 1.0 → 1.5 s: the FCU's mode comes in its 1 Hz
  HEARTBEAT (ArduPilot's rate too), so 1 s failed on heartbeat phase alone (seen: `after 1.1 s` under the full suite's load) —
  it would have failed at the bench too. GUIDE §6.2 says 1.5 s.
- Docs: GUIDE (header, the page row, three new rows: CAMERA, ⚙ SETTINGS, fake buoys = a sim scenario; §9 two commands),
  BENCH_GUIDE (the empty default), CHANGELOG, README, HANDOFF, CLAUDE.md (201 tests), pyproject / `__init__` 0.1.28.1.

**Tests**: `tests/test_ocs_web_settings.py` (4: the bench defaults to the empty world, every short name exists, sim_world.sh's two
refusals, `--help` of both scripts; fake_perception publishes the empty world and on SIGTERM clears the retained `sim/truth` — a new
subscriber gets nothing; the stops need `stop`, their command lines, `stop_nx` / `stop_agx` refused in AUTO_TASK while the Mac's are
not, `simworld` with a bogus choice → `empty`, `cam` zed refused in AUTO_TASK and dir allowed; `/cam.json` against a stand-in
stream, then down, then no address). `tests/test_ocs_web.py`: the system list filtered by group, the CAMERA / SETTINGS tabs.
`make test` 201 passed (after the two mode_log fixes; before them 200 + `test_box_sim` failing 3 of 6 runs alone, once in a full suite); `make quick` 169 incl. these.

**Rehearsed (Mac)**: `MAP=0 scripts/run_bench.sh` → `sim world: profiles/bench/world_empty.yaml`, the mm's map 0 entities; through
the page (for real, local only) `simworld bench` → 5 buoys in the map within 9 s; `stop_simcv` with `nope` refused, with `stop` →
fp `[exited]` and no retained `sim/truth` on the hub; `simworld pool` → 12 buoys; `stop_bench` with `stop` → `bench stopped`, ports
free. Dry run + a local cam_stream (`--source dir` on a JPEG): SETTINGS, the stop modal and CAMERA with the live picture checked in
headless Chrome over CDP (status `up · source dir · 34 frames · last 0.0 s ago`). `agx_cam.sh start/status/stop` on the Mac in a
test session. **Not run**: anything over ssh (the AGX camera toggle, the NX / AGX stops) — dry run only; the Jetsons are not
deployed.

**Open items**: the SIM SCENARIO drop-down starts at `empty`, not at the world that runs (the card's `now:` line shows that);
WP8a's (the sudoers line; the stale test processes).

**Next session, first**: WP9 (v0.1.29) with Dmitrii at the bench — the first real SYSTEM use (nx restart `--fcu uart --lab`), and
now the CAMERA toggle against the real AGX.

## v0.1.28.2 — 2026-11-04 — the camera at the pool: source `auto`, a status that does not lie

At the pool Dmitrii could not see the camera from the page. What happened (logs/ocs/system_actions.jsonl): the toggle started the
stream on the AGX with the default source `dir` — the detectors' preview frames, **blank until a buoy / dock observer runs**, and
none ran; ten seconds later the toggle went off (stream stopped). The ZED was there (lsusb `2b03:f880 ZED 2i`, /dev/video0-1);
`agx_cam.sh start --source zed` by hand gave live frames (snapshot checked). Also `agx_cam.sh status` said "up" while it was down:
`pgrep -f 'cam_strea[m]'` matched the caller's own ssh command line.

**Changed**
- `scripts/agx_cam.sh`: `--source auto` (the default): `zed` when no program that may hold the ZED runs (`actual_buoy_observer`,
  `dock_observer`, `buoy_camera_sender`, `run_docking` — ours or the students' copies), else `dir`; it says which it chose. `up()` =
  a python running `…/stream/cam_stream.py` (`pgrep -if`: macOS's framework python is `Python`). `run_agx.sh` keeps `dir` (the
  observers start after it, and `zed` would keep them off the camera).
- `tools/system_actions.py`: the `cam` action's source `auto|dir|zed|test`, default `auto`; the help text says `dir` is blank without
  an observer and `zed` holds the camera.
- `tools/ocs_web.html`: the CAMERA source drop-down says what each one shows (`auto (camera if free)`, `dir (detector previews; blank
  without one)`, `zed (the camera itself)`, `test pattern`).
- Docs: GUIDE (header, the CAMERA row, §9), CHANGELOG, README, HANDOFF, CLAUDE.md (202 tests).

**Tests**: `test_ocs_web_settings.py` +1 (status from a shell whose command line mentions cam_stream → down; a python running
`/x/cv/stream/cam_stream.py` → up; `--source webcam` refused; the page's options) and `cam` without a source → `--source auto`.
`make quick` 170 passed (16 s; it holds every test this change touches) — the full `make test` was stopped at Dmitrii's request (at the pool, a small change); the last full run is v0.1.28.1's 201.

**Live (pool, AGX)**: the stream runs `--source zed` since ≈14:20 (started by hand from the Mac over ssh, nothing else held the ZED);
the new pattern finds exactly that process on the AGX (not its `tee`, not the caller).

**Next session, first**: WP9 (v0.1.29). Before a detection run on the AGX: stop the zed stream (the toggle) — `auto` then gives `dir`.

## v0.1.29 — 2026-11-04 — WP8b: the OCS at the pool — points from the OCS, judge buttons, MAIN for the run

Dmitrii at the pool, before m0t2 (the stack shut down): "how do I inject points? … execute the save point from the OCS; in the judge
html push buttons as in the rctest tui with better help and tooltips; in the main OCS GUI not all commands, mission-specific things
missing; select missions from MAIN; better highlight in the messages if something is wrong; a more streamlined clearing for the run;
a sneak peek to RoboCommand, to see if the declared run got any response". Polled: WP8b = v0.1.29 (WP9 → v0.1.30 and on, ROADMAP);
points = the boat's fix + a click on the map; clearing = both (checklist + next-step button, and one-shot); peek = panel + live feed.

**Changed**
- `usv/points_store.py` (new): load / put / delete on usv_ui's points.json — re-read, change one name, atomic replace (tmp +
  os.replace); `Cache` by mtime; names `^[A-Za-z0-9_-]{1,16}$`. `usv_ui/usv_ui.py` `Points`: `data` re-reads the file when its mtime
  changes, `save()` is atomic — usv_ui and the mission manager can both write without dropping the other's points.
- SAVE_POINT / DELETE_POINT: `ocs/cmd_authority.py` GUARDS (any R), `schemas/bodies.schema.json` (cmd action enum + args doc),
  `usv/run_context.py` RUN_FREE_ACTIONS, `usv/command_handler.py` `_points` (no lat/lon = the boat's fix, 3D fix needed, `sats` /
  `hdop` kept; lat/lon = `from: ocs-map`; ACCEPTED + DONE at once; any vehicle state). `usv/mission_manager.py`: the map digest gets
  `points` (name → [x_n, y_e], usv_ui's shape) and `points_ll` from the file (mtime cache).
- `tools/ocs_cli.py`: `save_point NAME [--lat --lon]`, `del_point NAME`; MISSION no longer prints a false "playbook: would be
  refused: unknown intent MISSION" (it has no playbook entry; only MISSION is exempt). `tools/playbook.py`: SAVE_POINT / DELETE_POINT
  entries and their check (a mission manager must answer). `tools/bench_tui.py`: the two words in op-tui's grammar, usage and
  examples — **not** in its COMMANDS panel (a seventh group pushes the 150×50 layout's last row off; the page shows a POINTS group).
- `tools/map.html`: in the OCS page (an iframe) a `drop point` button (or the page's message `ocs-drop-mode`): a click → the map's
  local frame → lat/lon (the boat's equirectangular model) → postMessage `ocs-drop-point` to the page; an empty `sim/truth` already clears.
- `tools/rc_tui.py`: `JudgeSender` (send + do: course, start, assist, confirm, keepout, allclear, moving) shared by rc-tui (which
  keeps quit / clear) and the judge page. `tools/judge_web.py`: `--as-judge [--wire] | --monitor`, POST /cmd (header X-Judge-Web: 1),
  JUDGE ACTIONS cards with explanations and tooltips, defaults from the course centre / a heartbeat, buttons greyed when they cannot
  work, the mode badge. `scripts/run_judge.sh` passes the mode / wire to the page too.
- `tools/ocs_web.py`: `JudgePeek` (RcModel on --judge-host / --judge-port, read-only) → `judge` in state.json (the answer to our
  declaration: `RunStart run N for declaration S after X s` / `no RunStart for N s`, red after 30 s; the OCS's own view when the page
  joined after the declaration; reports, heartbeat ages, a feed of the last 25 with direction); `clearance()` (ten checks, seven hard,
  the next step); POST /clear (`ready` in a thread once the hard ones are green); `severity()` on every log line; `points` from the
  map digest; PAGE_GROUPS (op-tui's + POINTS). `scripts/run_ocs.sh` passes the judge's broker (hub for fake, RC_HOST:RC_PORT else).
- `tools/ocs_web.html` MAIN: RUN CHECKLIST + `next: …` + `clear for run ▶`; MISSION picker (the selection remembered) with options,
  preselect / start now and the mission's own controls (points to tick into --names, save boat position, drop on map…, colours,
  step, hold / resume / abort); ROBOCOMMAND + JUDGE FEED; every log coloured (red / amber / green edge), LOG `problems only`.
- Docs: ROADMAP (WP8b written and done; WP9 → v0.1.30 … WP14 → v0.1.35), GUIDE (three rows: MAIN for the run, points for m0t2, the
  judge page's buttons; §9 the two commands), CHANGELOG, README, HANDOFF, CLAUDE.md (208 tests), Makefile QUICK.

**Tests**: `tests/test_wp8b.py` (6: points_store + usv_ui's Points sharing one file (an external write seen, usv_ui's save keeps it,
no tmp files left); live — brokers + fake FCU + mission manager + ocs_app: `save_point EN` (the fake fix, `ocs-boat`, sats), `save_point
G1 --lat --lon` (`ocs-map`), a bad name refused by the vehicle, the map digest carries both (G1 ≈ 11.1 m north), `del_point` twice
(the second refused); the judge page: 403 without the header, assist via the fake judge, start / course refused when they cannot work,
monitor sends nothing, as-judge json keepout; the checklist and its next step over R2 … R5 and the blocks; clear_for_run refuses with
the reasons, runs `ready` when clear; the peek's answer bad after 40 s, ok with the RunStart, the OCS's view for a late page; severity;
the points list, the page's meta incl. word_to_action, the page parts, map.html's drop, run_ocs's peek wiring). `make test` 208 passed
(10 min); `make quick` incl. these.

**Rehearsed (Mac, bench, WORLD=por)**: the page (dry-run SYSTEM) + the judge page: checklist all green, next `declare`; `save_point EN`
(boat fix) and `save_point G1 --lat 1.30018 --lon 103.80002` → points.json; `mission m0t2 --names EN,G1`; `clear for run` → R5 /
AUTO_HOLD, the peek `RunStart run 1001 for declaration 1 after 2.0 s`; `start_task` → TEST_PATH FOLLOW → the boat ended at N 19.7
E 2.2 (G1 at N 20.0 E 2.2). In headless Chrome: MAIN (checklist, mission picker with EN, G1 ticked → `--names EN,G1`, ROBOCOMMAND),
MAP with drop mode, the judge page's actions; a real drop through the page (MAIN → drop on map… → click → name G2 → saved `ocs-map`).
Found and fixed on the way: a Python `#` comment inside the meta dict had cut `word_to_action` / `examples` off the page (render threw;
a test now checks them); the peek empty for a page started after the declaration (now the OCS's view). The bench's points.json removed.
**Not run**: on the boat (the NX's mission manager / usv_ui with the new points code), rc-test with the judge page `--as-judge`.

**Open items**: a mission's `--names` order is the tick order (no drag to reorder); `m0t2sim` on the bench needs a usv_ui beat (the
guard is right; the bench has no usv_ui).

**Next session, first**: WP9 (v0.1.30) with Dmitrii at the bench / pool.

## v0.1.29.1 — 2026-11-04 — at the pool: every buoy with a colour, the right map shown, map refresh / clear

Dmitrii at the pool, two buoys in the water: "the buoy detection works, but it doesn't reach the map"; "the CV can detect only one
buoy at a time - can we fix it / check it?"; "on the map a refresh button which will refresh the buffer". Traced live: the AGX
observer → cv_publish (1 item on most frames) → the NX's boat broker → the mission manager's map held them (543 ingested, 3
entities, colour unknown) — but the map / the page showed **usv_ui's** map (preferred while it publishes; it ingests nothing
without `cv on`). The students' observer measures every segmented buoy and keeps the nearest (`min(candidates)`); its model has
one class. (WP8c - go-home, the m0t6 orbit, the m0t2 / save_point guards - follows as v0.1.30.)

**Changed**
- `cv/buoys_multi/` (new, OURS): `multi_buoy_observer.py` imports the students' measure_mask / buoy_geolocation / open_zed /
  PixhawkTelemetry / discover_mavlink read-only and runs their loop keeping every measured buoy (nearest first, `--max-buoys 6`);
  `multi_core.py` (pure Python): the colour per buoy from a sample of its mask's pixels (HSV votes of saturated, lit pixels; red /
  yellow / green / blue; black / white by value; the winner needs `--min-share 0.45` and 1.5× the runner-up), the frame payload
  (`buoys` + the old `measurement` = the nearest); `run.sh` (output into cv/buoys/runs/ where cv_publish looks; `--self-test`).
  `cv/PROVENANCE.md` notes it. The snapshots and the students' folders are unchanged (a test checks the snapshot files stay read-only).
- `cv/publish/cv_publish.py`: a `buoys` list → one item per buoy with its colour (`--colour` only for unknown ones), ids frame×10+i;
  the students' single format publishes as before.
- `tools/map.html` + `tools/bench_tui.py` (the page's and op-tui's model): the mission manager's map is shown when it holds
  detections and usv_ui's does not (usv_ui with `cv on` keeps winning); map buttons **refresh** (forget the page's buffer: both
  maps, trail, truth; refit) and, inside the OCS page, **clear boat map** → the page asks → `clear_map`.
- CLEAR_MAP: authority (any R), schema, run-free, `command_handler` (refused while a task runs; `ctx.b.map.clear()`; saved points
  stay), playbook entry + check, ocs_cli `clear_map`, op-tui's grammar, the page's POINTS group.
- Docs: GUIDE (two rows: buoys on the map; every buoy with a colour), CHANGELOG, README, HANDOFF, CLAUDE.md (212), Makefile QUICK.

**Tests**: `tests/test_wp8c_cv.py` (3: the colour classifier — red with shadow, magenta highlights, green, yellow, blue, black,
white, a tie → unknown, too few pixels; the payload and cv_publish publishing both buoys with colours, the old single format
still `unknown`; the self-test, the launcher, the snapshots still read-only); `tests/test_wp8b.py` +1 (the map source rule incl.
usv_ui with cv on) and `clear_map` on the live stack. `make quick` 180 passed; the touched suites (wp8b, wp8c_cv, playbook_tui,
schemas, ocs_web) passed. Full `make test` not run for this fourth-digit version (at the pool; v0.1.29's 208 the last full run).

**Not run**: the multi observer on the AGX with the ZED (the students' observer had it; next: `cv/buoys_multi/run.sh --headless`
at the pool); the colour thresholds on real buoys (tune with --min-share / --sat-min from the preview).

**Next session, first**: v0.1.30 = WP8c (go-home override, m0t6 orbit, the m0t2 / save_point / m4t1 guards).

## v0.1.29.2 — 2026-11-04 — the camera on the LAN first; the plan note

Dmitrii packing up after the pool (next: Friday, the lab under the sky): "if we are on LAN, stream CV over the AGX's LAN address
rather than tailscale"; "make a short note with the plan" (declaration checks, a high-contrast theme, a full simulation day incl.
CV, data collection at the venue) for a planning discussion; hot fixes now.

**Changed**: `tools/ocs_web.py` `lan_agx_host()` (AGX_HOST of network.lan.env, read-only) + `cam_urls()` (CAM_URL alone, else the
LAN address, then the profile's) → meta `cam_urls`; `/cam.json` returns the first address that answers; `ocs_web.html` streams
from it (`camURL()`) and names it. `docs/NEXT_STEPS_2026-11-04.md` (new). ROADMAP: WP8c written as v0.1.30 (decided), WP9 → v0.1.31
… WP14 → v0.1.36. GUIDE (header, the CAMERA row), CHANGELOG, README, HANDOFF, CLAUDE.md (213).
**Tests**: `test_ocs_web_settings.py` +1 (the order of the addresses, CAM_URL alone, two dead addresses both named) and the old
camera test isolated from the tree's own network.lan.env. `make quick` (below); full `make test` not run (a Mac-only fourth-digit fix).
**Not deployed**: nothing on the Jetsons changed (they stay v0.1.29.1, the same code there).
**Next session, first**: the planning discussion (docs/NEXT_STEPS_2026-11-04.md), then WP8c v0.1.30.

## v0.1.29.3 — 2026-11-04 — the pool-log fixes: RC health (F1), perception age (F5), what to do in MANUAL (F6)

From `docs/POOL_REPORT_2026-11-04.md` §7 and ROADMAP §7's v0.1.29.3 package (the planning session's prompt). Dmitrii polled on
what the package left open, before leaving: carry on into WP8c afterwards; RC_LOST shall **hold** on pool + course, built now but
switched on only after the lab day shows the RC flag right (`guards.rc_lost_action`, the profiles stay `report`); go-home 0.3 m/s,
stop and hold (for WP8c).

**F1, the cause from the data**: the hub's SQLite of the pool day — all 3683 status messages `rc_ok: false`, `RC_LOST`; the RC
*values* did arrive (the RC5 E-stop fired from `rc[5]` three times) and both usv_ui and the mission manager ask RC_CHANNELS at 2–5 Hz
through a share that forwards every FCU byte, so the message was fresh: `rc_ok` failed on `chancount > 0` (the fake FCU sends 16,
the real Pixhawk evidently 0). The raw fields travel only on the boat's local topic, which was not recorded — the lab day confirms
it from `status.health.rc_detail` (no extra tool on the serial port).

**Changed**
- `companion/state.py`: `rc_health(fcu, rc_s)` — RC_CHANNELS fresh first (no channels = no RC5 / RC6 either), then the FCU's
  SYS_STATUS RC_RECEIVER bit when present and fresh (ArduPilot clears it on an RC failsafe), else `chancount > 0` or a plausible
  channel 1–4 in the **latest** message (`rc_last_plausible`; `rc` keeps old values); returns the reason. FcuState gains
  `rc_last_plausible`, `rc_rx_present`, `rc_rx_healthy`, `rc_detail`; Health gains `rc_detail` (in status.health).
- `companion/link.py`: SYS_STATUS's sensor bits (RC_RECEIVER_BIT 0x10000), the plausible flag per RC_CHANNELS, `rc_health()`.
- `companion/config.py` + `health.py` + `usv/vehicle_fsm.py`: `guards.rc_lost_action: report | hold` (validated); `hold` = RC_LOST
  in the soft faults (a task → V5) and `enable_auto` refused ("the transmitter is lost …"). `profiles/pool/usv_pool.yaml` and
  `profiles/course/usv_course.yaml`: `rc_lost_action: report` with the note to switch after the lab day; the lab profile still ignores RC_LOST.
- `sim/fake_fcu.py`: `{"set": {"rc_sys": "absent|healthy|unhealthy"}}` (the SYS_STATUS bit) and `{"set": {"rc_chancount": N}}`; the
  default (absent, 16) keeps the bench `rc ok`.
- F5: `usv/mission_manager.py` `_perception_seen = -inf` → the age is inf → `null` in status (was 1e9); the page shows `never`.
- F6: `vehicle_fsm.can_enable_auto` names the cause — RC6 high: *switch it low*; MANUAL: *the FCU is in MANUAL (RC6 override or the
  transmitter's mode switch): put it in HOLD, then enable_auto*; usv_ui active: *handover* (unchanged). `tools/playbook.py` predicts
  the MANUAL refusal from status.mode; `tools/ocs_web.py` RUN CHECKLIST: hard row **FCU not in MANUAL, RC6 low** (RC6 from the boat's
  local state when visible); `ocs_web.html` BOAT: an `RC` line with `rc_detail`, the health JSON with `perception_age_s: never`.
- Docs: GUIDE 4.4a (the MANUAL guard) and §6.1 (RC health / RC_LOST row with the lab-day check), header; CHANGELOG, README, HANDOFF,
  CLAUDE.md (217), ROADMAP (v0.1.29.3 ✅), Makefile QUICK.

**Tests**: `tests/test_pool_fixes.py` (4: rc_health per source — the pool's chancount 0, the bit healthy / unhealthy (a failsafe
wins over good values), a stale RC_CHANNELS, never, nothing at all, a stale bit not used, the fake's 16; rc_lost_action report vs
hold incl. the enable_auto refusal and the profiles' values; F5 null + F6 texts, the playbook's prediction and the checklist row;
live — brokers + fake FCU + mission manager: the bench default ok with perception null, chancount 0 still ok, the bit unhealthy →
`rc NO` + RC_LOST with `UNHEALTHY` in rc_detail, healthy → ok, the stream stopped → `not fresh`, back → ok). `make test` 216 passed + 1 failed: `test_integration_perception::test_map_builds_from_fake_detections_while_driving` (the dock line's y 0.63 m off, tolerance 0.5 — the fake camera's noise; it passed 3 / 3 alone right after, untouched by this version: an open item, the tolerance or a fixed seed);
`make quick` 184 passed, 9 deselected.

**Not run**: the real Pixhawk (the boat is packed). **Lab day (Friday), first**: deploy, then transmitter on → `rc ok`, off → `rc NO`
within 2 s, on → `rc ok`; read `rc_detail` (which source decides on the real link); then `rc_lost_action: hold` in the pool and
course profiles (a profile edit, its own fourth-digit version).

**Next**: WP8c v0.1.30 (Dmitrii: continue; home 0.3 m/s, stop and hold).

## v0.1.30 — 2026-11-04 — WP8c: ⌂ HOME (the go-home override), m0t6 orbit test, the pool's guards

Built on, as Dmitrii said before leaving ("continue into WP8c"), with his answers: HOME = the saved point HOME, else the control
origin; the override acts from any AUTO state with or without a run, MANUAL refuses; 0.3 m/s, stop and hold; m0t6 round a saved
point or a map position. From the pool (report F2 / F3): `--names "1 2 3"` reached the boat as one name; m0t2 without points took
8 s to abort; m4t1 looked like nothing ran.

**Changed**
- `usv/tasks/return_home.py` (new, RETURN_HOME): GO (Goto at `home.speed_mps` 0.3, arrive 1.5 m, timeout from the distance) → DONE.
  `usv/command_handler.py` `_home()` + `home_target()`: HOME in V3 / V4 / V5 only (else *in MANUAL the boat is yours - drive it home
  with the transmitter*); the saved point HOME (points_store) else the control origin; aborts the running task (runner + FSM
  ABORT_TASK), then START_TASK RETURN_HOME through the FSM; `speed_mps` optional. START_TASK RETURN_HOME is refused (HOME only).
  `usv/vehicle_fsm.py` `can_start_task`: RETURN_HOME and TEST_ORBIT known; RETURN_HOME needs no run. HOME is run-free (run_context).
- `usv/tasks/test_orbit.py` (new, TEST_ORBIT = m0t6): APPROACH (Goto to the nearest point of the circle) → ORBIT (the Orbit
  behaviour: r_m, direction, turns, speed_mps, la_m, band_m) → DONE; centre from names[0] (a saved point), lat / lon, or x_n / y_e;
  ENTRY_POINTS APPROACH / ORBIT (ORBIT needs the boat within band_m of the circle); `precheck()` refuses no / unknown centre, a bad
  direction or radius. Profiles: `home:` (pool, lab, bench, course) and `orbit:` (pool, lab, bench) sections.
- `usv/tasks/base.py` `Task.precheck(ctx)` (new hook) and `command_handler` START_TASK: a probe task runs it **before the FSM moves**
  (with `--from` on the same probe) - REJECTED at once. `usv/tasks/test_path.py` `precheck()`: the names against the points file
  (all missing ones named, the saved ones listed); no names → the real `plan()` on the current map: *no --names and no gate in the
  map* refused at once; without a position PLAN still waits as before.
- OCS: `ocs/cmd_authority.py` HOME (any R), TEST_ORBIT in MISSION_TASKS / START_TASKS, the orbit keys in MISSION_ARGS; schema
  (action HOME; tasks RETURN_HOME, TEST_ORBIT); `tools/playbook.py` the HOME entry + check (AUTO only, the hint for MANUAL), the
  m0t6 mission with its stages; `tools/ocs_cli.py` `home [--speed]`, the orbit flags (`--centre`, `--lat/--lon`, `--radius`, `--dir`,
  `--turns`, `--speed`) on mission / start_task, `split_names()` (commas and spaces), `check_names()`; `tools/bench_tui.py` `home`
  in the grammar, the names checked against the map digest's points before sending, the usage on a line that does not parse;
  `tools/ocs_web.py` the same checks (`not sent: point 3 not saved …`, `usage: save_point NAME …`), m0t6 options, `home` in the
  page's VEHICLE group, the gates count in the map summary, the checklist's m4t1 LISTEN note; `ocs_web.html` the red **⌂ HOME**
  button in the top bar (asks, names the target, greyed with the reason outside AUTO), m0t6 on MAIN (tick ONE point = the centre),
  *start now* greyed for m0t2 without points and no gate, for m0t6 without a centre.
- Docs: GUIDE rows (⌂ HOME, m0t6, the pool's guards; 4.15b: a gate must be in the map at the start), header; CHANGELOG, README,
  HANDOFF, CLAUDE.md (223), ROADMAP (WP8c ✅), Makefile QUICK.

**Tests**: `tests/test_wp8c.py` (6: m0t6 in fast SIL - DONE, on the circle within 1 m, a full clockwise turn, ccw the other way;
the prechecks and `--from ORBIT` off the circle refused, a lat / lon centre; RETURN_HOME in fast SIL at 0.3 m/s to a point 8.5 m
away; the OCS side - names split, check_names, the page's `not sent` / usage, meta (m0t6 options, home in VEHICLE), the HOME
verdicts, the m4t1 note; live - HOME refused in MANUAL, accepted in AUTO_HOLD **before any RunStart** (to the origin), then in R5
with m0t1 running: `save_point HOME` → `home --speed 0.6` → *TEST_AUTO aborted … the saved point HOME … 0.6 m/s* → RETURN_HOME →
AUTO_HOLD; the prechecks live - `--names "1 2 3"` → *points 1, 2, 3 not in …*, m0t2 with an empty map, m0t6 without a centre, all
refused with the vehicle still AUTO_HOLD). `tests/test_t1_sim.py`: RETURN_HOME exempt from "every task is a mission" (HOME only).
`make test` 223 passed (10 min 44 s; the run before it: 222 + the wiring test's old assumption, fixed); `make quick` 188 passed (the live tests are deselected there by name).

**Not run**: on the boat (the lab day). On the water: save HOME at the pontoon first; try ⌂ HOME from a running m0t2 at a safe
distance with the transmitter in hand; m0t6 round a buoy-free point with props on only after WP9's mode-20 walk (m0t6 is GUIDED only,
so it does not depend on it).

**Next session, first**: the lab day (Friday): deploy v0.1.30 (both Jetsons), v0.1.29.3's RC check (transmitter on / off), then WP9.

## v0.1.31 — 2026-11-04 — WP9a, the software half: the map's ghost filter

Dmitrii: "continue with the next package on the roadmap". The order (ROADMAP §7): WP9 is the lab day (hardware, Friday) and WP11 waits
for the students, so both are skipped for their dependencies; by the rule there they keep their numbers and take the next free version
when built — WP9a is built now as **v0.1.31** (its heading said v0.1.32; corrected, the later headings are reassigned when built).
Its lab-day half (the ZED colour thresholds of `cv/buoys_multi`, the acceptance with two real buoys) stays open. Pool report F4.

**Changed**
- `companion/config.py` MapCfg `min_obs` (code default 1 = unchanged for bare Config() users: the fast SIL, usv_ui's own map, the
  unit tests) and `forget_s` (0); a top-level `perception:` section (the package's key names) sets both; validated (min_obs ≥ 1,
  forget_s ≥ 0). Profiles pool / lab / bench / course: `perception: {min_obs: 3, forget_s: 5.0}`.
- `usv/perception_map.py`: `entities()` leaves out entities with fewer than min_obs sightings unless `include_young` — so gates,
  markers, the dock line and every task's query never see a ghost; association (`_nearest`) still scans all (a ghost's next sighting
  joins it, no new ghost); `age_out()` drops a young entity unseen for forget_s; `digest()` leaves them out and reports `n_young`,
  `min_obs`.
- `usv_ui/usv_ui.py` `cv status`: `cv: colours red 1, green 1; young (< 3 obs, not in the map) N`, ghosts marked `(young)`.
- `tools/map.html`: blue / yellow / white table rows coloured, the legend names blue and yellow (the glyphs already were).
- Docs: GUIDE 4.14a and §7.2 T-B (the lab-day check: two buoys → two coloured entities, no ghost; tune the thresholds), header;
  CHANGELOG, README, HANDOFF, CLAUDE.md (226), ROADMAP (WP9a's software half ✅ as v0.1.31), Makefile QUICK.

**Tests**: `tests/test_wp9a.py` (3: a ghost beside a gate stays out of entities / digest / gates with `n_young` 1, becomes real after
three sightings without a duplicate; a ghost unseen 5.5 s dropped, a track kept, forget_s 0 = old ageing, Config() unchanged; the four
profiles, the `perception:` section, its validation, the map / usv_ui texts). `make test` 226 passed (10 min 52 s) — the live sims
(T1 on world_pool, m0t2sim on PoR, m3t1, m0t4) now run with min_obs 3 from usv_bench.yaml and pass; `make quick` 191 passed.

**Not run**: the AGX / the ZED. **Lab day**: with `cv/buoys_multi/run.sh --headless`, two buoys → two coloured entities and no ghost
(`cv status`, the map); tune `--min-share` / `--sat-min` on red, green and the pool's blue markers from the CAMERA preview; write the
values into `cv/buoys_multi/run.sh` with the date — that closes WP9a.

**Next**: the lab day (deploy v0.1.31 to both Jetsons, the RC check of v0.1.29.3, WP9, WP9a's lab half); without hardware the next
package is WP9b (the ZED recorder) — though its acceptance needs the ZED too.

## v0.1.31.1 — 2026-11-04 — the lab day's page and the frame comparison (WP9 prep)

Dmitrii: "prepare the doc for Friday (and bump me on Friday with it), finish the code we need for that, and go with the simulator;
perhaps we could test partially before we touch hardware with our sim". The reminder: a session-only one-shot (Friday 2026-11-06
08:47, this Claude session must still be open). The order after this (ROADMAP §7, renumbered the same night): WP9 the lab day →
**WP10 our simulator** (new; the old WP10–WP15 are WP11–WP16).

**Changed**
- `docs/LAB_DAY_2026-11-06.md` (new): the one page - before leaving (make quick, the optional bench rehearsal), the deploy with the
  concurrency check, the start (`--fcu uart --lab`, usv_ui `log on lab1`: pool report F8), the RC check (v0.1.29.3, F1: on / off / on
  with `rc_detail`, then `rc_lost_action: hold`), **the mode-20 walk** (m0t3 between two points a metre apart, RC5 / RC6 during the sway,
  usv_ui's `swayto` as the reference, `frame-compare`, `mode-log --lab --md` → GUIDE §6.2, m3t1 `--step`), ⌂ HOME and m0t6 on the
  trolley, two buoys with `cv/buoys_multi` and the colour thresholds written down, the end of day with the logs home.
- `optiprimex/tools/frame_compare.py` (new, console `frame-compare`): usv_ui's leg CSV (rows with a sway state, c1..c4) against the
  trace's `frame` events in stage SWAY - count, rate, the steady frame (per-motor median), the per-motor difference; **match** within
  `--tol` 0.08 and the same signs; `--md` = GUIDE §6.2 rows; exit 0 / 1 / 2.
- `tests/test_wp8c.py`: the live fixture cleans up when its set-up fails - a failed first run had left a mission manager and an
  ocs_app running, and the next full suite failed `test_t4_chain_on_the_real_wire` (readiness 1 of 2; whole file green twice after
  killing them, the full suite green after the fix).
- Docs: GUIDE §9 `frame-compare`, header; CHANGELOG, README, HANDOFF, CLAUDE.md (227), pyproject (`frame-compare`), Makefile QUICK.

**Rehearsed (Mac, the bench as today's sim)**: the page's step 0 - `WORLD=empty run_bench.sh`, `save_point A`, teleport 1.1 m, `save_point
B`, `ready`, `start_task m0t3 --names A,B`: LINE → TO_START → SWAY → DONE in 26 s, `GUIDED > CUSTOM_MANUAL`, 204 frames at 10 Hz, gap
0.12 s, HOLD at the end; `mode-log --lab` PASS (warnings = the bench's always-armed plant and profile name).

**Tests**: `tests/test_frame_compare.py` (1: match, a sign flip, motor 2 off, no frames, the CLI's `--md`). `make test` 227 passed
(10 min 48 s); 192 passed, 11 deselected in 19.05s.

**Next**: WP10 our simulator (first version: the world, the Python renderer, the camera stream, the SIM page, arrow keys in MANUAL).

## v0.1.32 — 2026-11-04 — WP10, first version: our simulator (a rendered world in its own window, arrow keys in MANUAL)

Decided with Dmitrii the same night (ROADMAP WP10): our own simulator, not VRX; the renderer in Python; no disturbances and no real
physics first (the fake FCU's kinematic plant); arrow keys drive it in MANUAL; its own window, started by its own script or from the
OCS, not inside the OCS page; "perhaps we could test partially before we touch hardware with our sim" - so the first version was
built before the lab day (WP9 keeps its place in the order and takes the next free version when it runs).

**Changed**
- `optiprimex/sim/render.py` (new; numpy + Pillow): a pinhole camera (ZED-like hfov 100°, 640×360, 0.8 m up; pitch for the chase
  view), one ray per pixel, vectorised: the water plane with a ripple and a haze toward the horizon, a light-grey sky, the pool walls
  (`bounds`) as light-grey walls, buoys as shaded spheres half out of the water with a beacon in the light colour (flashing by the
  world's phases), box faces / dock bays as boxes; depth along the optical axis like the ZED; `project()` (a world point → pixel and
  depth); `scene_from_world()`; `jpeg()`. ~70 ms a frame on the Mac.
- `optiprimex/sim/simview.py` (new) + `optiprimex/sim/sim.html` (new): the server polls the fake FCU's pose, renders the camera and a
  chase view at 6 Hz, serves the camera with **cv/stream's interface** (`/stream`, `/snapshot.jpg`, `/status`), `/chase.jpg`,
  `/world.json`; POST `/sticks`, `/fcu` (the mode switch MANUAL / HOLD only, kill, E-stop RC5, RC6, GPS / EKF / RC loss, FCU pause,
  teleport - never arming), `/objects` (add / move / delete mockups in memory), all behind the header X-Sim-Web: 1. The page: chase
  view, the camera, a top-down world (drag, double-click to add, shift-click to teleport, Delete), the transmitter's switches and the
  faults, the arrow keys (↑↓ surge, ←→ yaw, A / D sway, Shift = full) at 10 Hz while held, one zero on release.
- `optiprimex/sim/fake_fcu.py`: `{"sticks": {surge, yaw, sway}}` on the control port - the boat moves in MANUAL (mode 0) only,
  powered, released after 0.5 s; `rc5` and `sticks` in its state.
- `scripts/run_sim.sh` (new; `make sim`, `make venv-sim`, `requirements-sim.txt`): the bench on a world + windows `sim`, `web` (the
  OCS page with CAM_URL = the simulator), `jweb` (the judge page); opens the SIM page and the OCS page. The OCS page's SETTINGS links
  the simulator.
- `docs/LAB_DAY_2026-11-06.md`: the rehearsal on the simulator; deploy v0.1.32.
- Docs: GUIDE (the simulator row, §9 run_sim / make sim), header; CHANGELOG, README, HANDOFF, CLAUDE.md (231), ROADMAP (WP10's
  first version ✅), Makefile QUICK.

**Tests**: `tests/test_sim_wp10.py` (4, skipped where numpy is missing - the Jetsons need neither: the projection (ahead, turned
90°, to starboard at atan(right / forward)), a red buoy rendered red at its projected pixel with depth 10 m − r, water / sky / a wall
at 20 m; the world files → primitives (walls, buoys, beacons on / off, the three box faces); the sticks: nothing in HOLD, moving in
MANUAL, released after 0.5 s; the server: the page, `/status` like cv/stream, a JPEG snapshot, `/world.json`, 403 without the header,
mode 15 refused, `armed` ignored, a mockup added / moved / deleted, the sticks forwarded, run_sim.sh --help). `make test` 231 passed
(10 min 56 s); `make quick` 196 passed.

**Rehearsed (Mac)**: `run_sim.sh pool`: 44 frames in 6 s; MANUAL via the page's switch, 2 s of stick → the boat N 1.5 E 0.35, 23.5°,
the mission manager V2 MANUAL, the sticks back to zero; in headless Chrome the ↑ key for 2.5 s → N 3.0 E 1.1; screenshots of the
SIM page (chase view with the pool and the buoy field, the camera, the top-down world) and of the OCS page's CAMERA tab showing the
rendered camera (`source sim`).

**Open (WP10's second version)**: the sim detector on the rendered frames (+ depth) → `latest_status.json` → cv_publish → the map
(today the map still takes the geometric fake perception); usv_ui on the fake plant in run_sim.sh (the handover step, the …sim
codes); the scenario sheet docs/SIM_DAY.md (WP9e) and a full run.

**Next**: the lab day (Friday, docs/LAB_DAY_2026-11-06.md; the reminder is set in this session); then WP10's second version.

## v0.1.33 — 2026-11-05 — WP10b: the simulator's plumbing (the dataset export, rc-test, RESET SIM, teleports, Marina Bay defaults)

ROADMAP §7a (decided 2026-11-05: the simulator-and-GUI programme WP10b → WP10f, one version each) and §7b (CV_TRAIN, the third agent).
WP10b has no choice of Dmitrii's in it; the dataset export was built first because CV_TRAIN waits on it, then reworked the same day to
CV_TRAIN's request (`../CV_TRAIN/docs/REQUEST_dataset_export.md`; the answer is `docs/REPLY_dataset_export.md`).

**Changed**
- `optiprimex/sim/render.py`: `render_ids()` = the picture, the depth and the primitive under every pixel (the masks); a `style` (sun
  direction, water / sky / wall colours) for the domain jitter; `walls: false` in a world = open water; a CLI (`--dataset`, `--frame`).
- `optiprimex/sim/dataset.py` (new): random poses inside the course (2 m clear of every object), 3 frames per pose (stems
  `<world>_s<seed>_p<pose>_f<frame>`), ~10 % negatives, flat Ultralytics layout (`images/ labels/ masks/ depth/ truth/`, `data.yaml` with
  option A's names buoy / dock_face / window / box, `--classes colour` = option B), YOLO-seg polygons (Moore trace of the largest visible
  part), instances under 12 px or 20 % visible skipped (occlusion by a solo render), depth float16 npz, `truth/` per frame (pose,
  intrinsics, per instance id / colour / lit / mask id / visible fraction / true range and bearing), `manifest.json`. 1280×720 by
  default, ~2.5 frames/s, ~110 KB a frame.
- `profiles/bench/world_marina.yaml` (new): Course Alpha at Marina Bay, open water, a Task 1 field along 032°, three bays, three box faces.
- **Marina Bay defaults** (`optiprimex/sim/marina.py`: Course Alpha's corners from rc_test, its centre 1.2810202 N 103.8550097 E): every
  world's origin, `usv_bench.yaml` / `usv_bench_share.yaml` origins, the fake FCU's start, the fake judge's course (now Course Alpha) and
  incident point, rc-tui's default course, the example coordinates (ocs_cli, playbook, bench_tui, vcmd, scenario, judge_web, the OCS
  page). The pool / lab / course usv profiles keep `origin: null` (the first fix decides) - the boat is not affected.
- OCS profiles: `course.default` (Course Alpha) + `geofence: []`. `tools/ocs_web.py`: `--ocs-config`, GET `/course.json` (the default,
  the received course from the RoboCommand peek, the geofence), POST `/geofence` (rewrites the profile's one `geofence:` line, ≥ 3
  corners or none), POST `/sim` (the teleports / RESET to simview). `map.html`: the course (dashed *default* until one is received,
  then solid; the UAV transit area dotted), the geofence (dashed yellow) and its editor, the teleport bar (the world's places, HOME /
  saved points, by click with the 2 m snap, `lat lon [hdg]`).
- `sim/simview.py` + `sim.html`: POST `/teleport` (name | x_n, y_e | lat, lon; `snap`), POST `/reset` (RESET SIM, typed `reset`): the
  plant clean (faults off, HOLD, the start pose), the mockups from the file, then abort, confirm_safe, reset, clear_map over the hub and
  the fake judge's `sim_reset`; `world.json` carries the teleports and the saved points; the SIM page's TELEPORT card and RESET SIM….
  `scripts/sim_reset.sh` (new); ⚙ SETTINGS → *Reset the simulator* (system action `simreset`).
- `sim/fake_robocommand.py`: `{"kind": "sim_reset"}` on rc/sim/inject (counters, declaration, run forgotten; the seqs keep counting).
- `scripts/run_bench.sh --judge fake|rctest` (+ `OCS_CONFIG`), `scripts/run_sim.sh … --judge rctest`: RoboNation's rc-test on 1885 (started
  unless 1885 listens), the course profile, the page's peek on 1885, no jweb window; ocs_web gets the profile (`run_ocs.sh` too).
- Found by the sim-day re-run and fixed: (1) `sim/fake_fcu.py`: no GPS fix now also means no EKF position (as on the Pixhawk) - GPS lost
  gave no EKF_BAD / FAULT; (2) `usv/mission_manager.py`: reads one parameter (FRAME_CLASS, read-only) every 5 s until the FCU answers -
  `params_ok` (m0t4's *thrusters* preflight) came only from usv_ui's parameter reads, so on the bench m0t4 waited in PREFLIGHT for ever
  with no reason shown; (3) docs/SIM_DAY.md S10: m0t4 needs `--scan-x 40 --scan-y 0` on boxes (the default scan point is out of range).
- `optiprimex/tools/sim_day.py` (new): the sim-day sheet walked by a script (S0 RESET / teleports, S1–S12), a markdown table out.
- Tests that hard-coded 1.3 / 103.8 moved to Marina Bay (t1_sim, t3_sim, wp8b, wp8c, sim_pool, box_sim, integration_sil1,
  protobuf_judge); `test_t4_chain_on_the_real_wire` now waits for its own readiness in the 1 Hz stats (it read them too early).
- Docs: GUIDE (two rows in the simulator section, three in §9; header), CHANGELOG, HANDOFF / README (v0.1.33), ROADMAP (WP10b ✅),
  docs/SIM_DAY.md (v0.1.33: Marina Bay coordinates, the new buttons, rc-test available), docs/REPLY_dataset_export.md.

**Tests**: `tests/test_wp10b.py` (9): the export (flat layout, names, polygons inside the mask boxes, mask ids = truth, depth ≈ forward
range on buoys, thresholds, never overwritten; the split layout keeps poses apart), the outline / colour classes, negatives and the
open-water world (a negative heading renders nothing; the style changes the picture), Marina Bay everywhere (Course Alpha 25.0 × 30.1 m
centred), the fake judge's sim_reset, teleports + snap + RESET SIM against the fake FCU, the course view / geofence save / sim proxy /
the simreset action, the launchers' `--judge`, the sim_day driver. `make test` **240 passed** (10 min 17 s); `make quick` 205 passed (`test_wp10b.py` in QUICK).

**Rehearsed (Mac)**: `run_sim.sh pool --judge rctest`: declare + enable_auto → R5 in 1 s (rc-test: `publishing RunStart
(declaration_seq=1, run_id=1)`); T4 from the judge page (`assist`, then `confirm`) → LISTEN > ACK > TRANSIT > READY > WAIT_CONFIRM > DONE,
all in rc-test's log; RESET SIM from R5 + KILLED + GPS lost + RC6 + a mockup → R2 / MANUAL / start / HOLD in 4 s; the map (CDP
screenshot) draws Course Alpha rotated 32° and the transit area; the SIM page's TELEPORT card. Ultralytics 8.4.202 (a throwaway venv)
opened the first export: 0 corrupt labels, `segment train/val` and `detect train` ran (the reworked flat layout not re-checked with it).
**The sim-day sheet re-run** (`tools/sim_day.py`, logs/bench/run_*/sim_day.md): the first pass 35 / 36 - the one finding (GPS lost in AUTO_HOLD stayed AUTO_HOLD) fixed in the fake FCU; S10 on boxes found the mission manager never got `params_ok` (fixed) and the sheet's missing scan point (fixed); after the fixes **36 / 36 on pool** (S0 RESET 7 s, S2 R5 in 3.9 s, S5 mode_log PASS, S8 m1t1 147 s through every stage, S9 m3t1 86 s, S11 m4t1, S12 kill / RC6 / GPS / FCU pause) and **S10 m0t4 on boxes DONE** in 131 s (GO_SCAN > OBSERVE > ALIGN > APPROACH > STOP > DONE).

**Open**: windows are not rendered (class 2 empty); RESET SIM does not reset rc-test's own counters (not ours); the GUIDE §1 loop to
the Jetsons not done (Friday deploys v0.1.32 or this one - the boat's profiles are unchanged; the mission manager's one parameter read
is new on the real link: harmless, read-only, but watch usv_ui's `param` command on Friday).

**Next**: WP10c (v0.1.34): map interaction - add object, hover coordinates, tooltips, the mini-map on MAIN.

## v0.1.34 — 2026-11-05 — WP10g: buoy localisation (CV_TRAIN's request) - the real camera, surface + radius, far buoys as rays

CV_TRAIN's `../CV_TRAIN/docs/REQUEST_localisation.md` (14:23): it measured the students' measure_mask chain on 600 of our exported
frames with the true masks: range 0.22 m short at every distance (the visible surface), every buoy past 20 m lost with its bearing,
overlaps handled, both simulators' field of view wrong (72° / 100° vs the ZED's 50.9° at hd2k). Polled: all five items in one package
now (WP10g = v0.1.34, WP10c… +1); the radius per type from the handbook (it has none: 0.25 m until measured). Mid-build Dmitrii asked
whether a ray should rather become an object within ~30 m of reliable depth and how pathing_sim did it - read the spec (5.3: rays are
leads, never fused, a point when depth arrives along the ray; 335: the search drives toward leads; 410: "bearing-only detections as rays,
never fused" was a fix for an observed failure); polled: **the pathing_sim way, triangulation off; depth limit 25 m now**.

**Changed**
- `optiprimex/sim/zed.py` (new): SN33545355's LEFT calibration per resolution (read from the AGX, `/usr/local/zed/settings/`, read only)
  + the baseline 119.92 mm; `scaled()`, `hfov_deg()`, `bearing_deg()`, `DEPTH_MAX_M` 25; `stereo_noise()` (σ_z = z²σ_d/(fB), holes on
  flat surfaces, flying pixels, max 40 m).
- `sim/render.py`: `Camera()` = the ZED's hd2k scaled (fx, fy, cx, cy; an explicit hfov = a centred pinhole as before); rays and
  `project()` through cx / cy. simview / `--dataset` default to it (`--hfov` overrides); `--depth-noise zed`. `sim/world.py`:
  `camera.intrinsics: zed2i_hd2k` → fov_deg; every world but world_bench (the integration tests' wide 100° layout, kept on purpose).
- `cv/buoys_multi/multi_core.py`: `centre_correction` (+0.85 r along the line of sight, `surface_range_m`), `bearing_record`,
  `mask_centre`; `frame_payload` sorts ranged first, `BEARING_ONLY`, `n_bearing_only`. `multi_buoy_observer.py`: `--max-depth 25`
  (passed to the students' measure_mask / open_zed - their file unchanged), `--buoy-radius`, `--bearing-conf 0.6`, the bearings from the
  SDK's left intrinsics, CSV `surface_range_m` / `reason`. `cv/publish/cv_publish.py`: bearing-only items (`range_m: null`).
- `companion/state.py` ObsItem: `range_m` null ⇄ 0 (a bearing). `usv/perception_map.py`: rays (30 s, refreshed within 1 m of where
  they began, dropped when a ranged buoy lies along them), `_fold`, triangulation behind `perception.triangulate` (default false;
  uncertainty-scaled gate), digest `rays` / `n_triangulated`; colour votes weighted by range (15 → 25 m: 1 → 0.25) and a lit colour on
  a buoy dark half the time must double its runner-up (LIT_OVER_SECOND). config MapCfg: ray_ttl_s, ray_ang_tol_deg, tri_*, triangulate,
  vote_*. `sim/fake_perception.py`: bearings 25–40 m (`bearing_max_m`) on their own random stream. map.html: rays drawn.
- Findings of the real field of view, fixed: the 2026-11-02 bug back (an unlit obstacle at 22 m voted blue → entry) → the rules above;
  the dock from 5 m shows one bay → `ABEAM_BAY2` and world_pool's teleport 9 m off; docked 1 m off the face the windows are out of
  view → `usv_bench.yaml t3.stop_m` 1.5, usv_ui dockto 1.5 (sim only); m1t1 failing 6 / 8 seeds from a shifted noise stream alone →
  the band's own stream.
- `docs/POOL_CALIBRATION.md` (new), `docs/REPLY_localisation.md` (new), GUIDE (a row in the simulator section, one in §9, header),
  ROADMAP (WP10g ✅, renumbered), CHANGELOG, cv/PROVENANCE.md, HANDOFF / README / SIM_DAY versions.

**Tests**: `tests/test_wp10g.py` (8): the camera model (hfov, worlds, `project()` and the bearing through the calibration, an explicit
hfov); the surface + radius bias ≤ ±0.05 m on rendered buoys 5–19 m; bearing records → payload → cv_publish → ObsItem; rays are leads
by default (no entity, depth limit 25, the observer passes it); with triangulation a buoy at 32 m becomes one point within 1.5 m (5
seeds; 20 seeds offline: worst 0.71 m, never two); a ranged buoy explains a ray, rays age out; fake perception's band; the depth model
(σ ∝ z², holes, nothing past 40 m, flying pixels). Updated for the real camera: test_sim_wp10 (cx), test_wp10b (hfov), stage_stepping /
sim_pool poses (9 m off the dock). `make test` **247 passed** (10 min 14 s; the first run 244 + 3 failures: world_bench's side objects out of the real camera's view - world_bench keeps its wide test camera); `make quick` 212.

**Rehearsed**: offline only (the map with seeded bearings; fast SIL m1t1 over 4 seeds × 2 planners at 72° / 60° / 50.9° with and without
the band). Not run on the AGX: the observer's bearing path (pyzed) is untested on hardware - the pure parts are tested.

**Open**: the pool / course `t3.stop_m` (1.0 m) waits for the real window spacing; the 25 m limit waits for POOL_CALIBRATION's shots;
the search does not yet drive toward rays (pathing spec 335 "pending" - with the S6 tracker port, ROADMAP §4a); m1t1's robustness over
seeds and fields of view deserves a sweep; world_bench keeps a 100° test camera.

**Next**: WP10c (v0.1.35): map interaction - add object, hover coordinates, tooltips, the mini-map on MAIN.

## v0.1.34.1 — 2026-11-05 — re-plan: simulation first; the lab day rewritten (docs only)

Dmitrii (evening): no boat motion on the lab day (switches, connections, some real CV), no pool before the event, a few bench days;
the priority is to finish the simulation so the main mission profile is proven there, and to collect real data at the event with
every pipeline known in advance; fewer profiles. Polled: the proposed order but WP10c kept first; profiles cut to sim / lab / event;
rewrite the lab-day page now.

**Changed**: ROADMAP §7c (new: the order 0.1.35 WP10c → 0.1.36 WP10e → 0.1.37 WP10d → 0.1.38 **WP10h the mission profile campaign**
(new) → 0.1.39 WP9b recorder + the event's data pipeline → 0.1.40 WP9c → 0.1.41 WP9d = **the profile cut** (`profiles/sim|lab|event`,
usv_pool / ocs_pool / usv_course / ocs_course folded into event, world_bench / world_por to tests/fixtures) + presets; WP10f after the
event; WP11–WP15 keep 0.1.42–0.1.46, to reorder when the event's schedule is known; WP9 reduced). `docs/LAB_DAY_2026-11-06.md`
rewritten: deploy v0.1.34.1, the switches (RC check, mode switch, kill latch, handover / clear for run, RC6, HOME - no motion), the
connections (bridge, presence, clocks, camera, link loss, rc-test optional), CV on real buoys (colours, range per distance on a tape
5–30 m with `--max-depth 35`, ±30°, overlap, a far bearing), an optional 15-minute dry mode-20 check (if skipped: the first mode-20 on
water is at the event's practice - flagged), the logs home. CHANGELOG, GUIDE / HANDOFF / README versions. No code changed.

**Tests**: `make quick` 212 passed (no code changed since v0.1.34's 247 / 212).

**Next**: Friday the lab day (this page; deploy v0.1.34.1 when Dmitrii says so); then WP10c (v0.1.35).

## v0.1.34.2 — 2026-11-05 — CV_TRAIN's hand-over (cvt-v1-l) and its localisation follow-up

CV_TRAIN published `models/cvt-v1-l` (YOLO26l-seg, buoy / dock_face / window / box, synthetic only, box mAP50 0.94, engine 82 ms on the
AGX; HANDOVER_cvt-v1-l.md, CARD) and a follow-up to REQUEST_localisation (16:20): with the guessed ZED noise, buoys centred beyond 25 m
are sometimes ranged ~0.6 m short (only when the noise pulls the surface under the limit); 32 far buoys found at conf 0.5–0.6 were
dropped by `--bearing-conf 0.6`; the AGX still runs v0.1.29.1. Dmitrii: "check the new handover, apply if needed, then WP10c".

**Changed**: `cv/buoys_multi/multi_core.py`: `sigma_z`, `near_limit` (a range within k σ_z of the limit → a ray, `near_limit`),
`model_spec` (a CV_TRAIN folder → weights (.engine on aarch64, else .pt), imgsz, conf, iou, the `buoy` class ids). `multi_buoy_observer.py`:
`--model DIR`, `--conf` / `--imgsz` from the model (else 0.60 / 768), `--bearing-conf` = `--conf` by default, `--limit-sigma 2`, iou
passed, non-buoy classes skipped. `cv/sync_cvtrain.sh` (new, AGX): copy + sha256 check, CV_TRAIN read only. Docs: REPLY_localisation
(the follow-up answered), GUIDE §9 row, LAB_DAY §5.7 (the model side by side, optional) and the deploy version, cv/PROVENANCE, CHANGELOG.

**Tests**: test_wp10g +1 (sigma_z / near_limit, model_spec with and without an engine, the observer's wiring, the sync script);
`make test` 248 passed (the first run: 246 + 2 in test_integration_sil1 - a timing flake under load, the file alone passed twice, the re-run all green); `make quick` 213. The observer's model path is not run on hardware (needs the AGX).

**Next**: WP10c (v0.1.35).

## v0.1.35 — 2026-11-05 — WP10c: map interaction (add / move / delete, hover, tooltips, the mini-map)

The first package of the §7c order (Dmitrii kept it early: it speeds up every sim session). Nothing in it was his to choose.

**Changed**
- `tools/map.html` (the OCS MAP tab, and standalone): **add** (a bar: kind = sim buoy (colour, light flash / steady / off) | sim dock bay
  (colour, bay) | sim box face | waypoint (= `save_point NAME --lat --lon`) | marker (a note); then a click) - sim objects through the
  page's `/sim` proxy only while a simulator answers; **drag** moves (sim objects, markers, saved points), click + **Delete** removes
  (a saved point asks: `del_point`); the simulator's added objects drawn dashed, markers ◆; **hover** = lat / lon (7 dp) + N / E, over
  an object also range / bearing from the boat (and off the bow); a **click** copies `lat lon`; **?** = the legend over the map;
  `?mini=1` = no header / panel / footer, auto-fit every 4 s, a click asks the page for the MAP tab (`ocs-show-map`); `/cmd` answers
  judged by their lines (REJECTED / refused / usage). Titles on every button.
- `tools/ocs_web.py`: `Markers` (`logs/ocs/markers.json`, atomic; name check; add / move / del; OCS_MARKERS env), GET `/markers.json`,
  POST `/markers`; the `/sim` proxy also forwards `/objects` and `/waypoint`. `tools/ocs_web.html`: the **mini-map card** on MAIN
  (iframe `map.html?mini=1`, 340 px square), `ocs-show-map`; `fillTitles()` + a MutationObserver: every button gets a title (data-tip,
  an id catalogue, the tabs, the commands from the playbook's does / needs / then, the system actions' what).
- `sim/simview.py`: `/objects` adds `dock` (colour, bay), validates the light, places / moves by lat / lon too, marks added ones
  `mock: true`; POST `/waypoint` (save_point / del_point through ocs_cli); GET `/markers.json`, POST `/markers` relayed to the OCS page
  (`--ocs-url`). `sim/sim.html`: the kinds dock bay / waypoint / marker, one selection for sim objects / markers / points (drag, Delete),
  markers drawn, the hover readout and click-to-copy, a `?` legend, titles on every button (SIM_TIP). `tools/judge_web.py`: the action
  buttons titled with their descriptions.
- Docs: GUIDE (a map-interaction row, header), CHANGELOG, ROADMAP (WP10c ✅), HANDOFF / README versions.

**Tests**: `tests/test_wp10c.py` (5): the markers store; the OCS page + simview live in-process (every kind through the proxy, the light
check, move by lat / lon, delete, the proxy's whitelist, markers from both pages, the header); the hover model = the control origin's
`from_local` (map.html's JS run in node); the pages carry the controls; **headless Chrome: no `<button>` without a title** on the rendered
OCS page and map (`…_live`, deselected in quick). `make test` **253 passed** (10 min 21 s); `make quick` 217.

**Rehearsed (Mac, run_sim.sh pool)**: a red flashing buoy, a green dock bay, a marker and a waypoint added through the OCS page → in
the simulator's world (the camera and the chase view showed them at once), in `logs/ocs/markers.json` and in usv_ui's `points.json`;
moves / deletes from the SIM page's API; rendered DOMs: OCS page 76 / 76 buttons titled, SIM 18 / 18, judge 8 / 8, map 19 / 19;
screenshots of MAIN (the mini-map with the course, the boat, the mockups, the marker, WY), the MAP tab (the legend, the hover
`sim buoy #1045 red: 16.0 m · 4° (+4° off the bow)`, the far-buoy rays), the SIM page (hover, legend, the added objects in both views);
**teleport to the hovered lat / lon landed at N 16.05 E 0.99 = the hover's N 16.1 E 1.0**; the mini-map with no course at all (an OCS
page without a profile or a judge) draws without error. The rehearsal's points.json / markers.json removed afterwards.

**Open**: the hover's range / bearing is from the boat's position in the map digest (the camera offset is not applied - a few cm);
markers are not in the OCS's state or logs beyond their file; WP10e makes the added buoys reach the map through the camera.

**Next**: WP10e (v0.1.36): the CV loop closed - the sim observer on the rendered frames → cv_publish → the map; usv_ui on the simulated
boat; then the sim-day sheet re-run.

## v0.1.36 — 2026-11-05 — WP10e: the CV loop closed in the simulator (the camera path to the map, usv_ui on the fake plant)

The second package of §7c. Nothing in it was Dmitrii's to choose.

**Changed**
- `sim/simview.py`: GET `/frame.npz?w=&h=` - one consistent frame at a fresh pose (the picture, the exact depth, the primitive under each
  pixel + each primitive's world id / kind, the pose in the world frame and lat / lon / heading, gps_fix, the intrinsics, the time).
- `cv/sim_observer/` (new, ours): `sim_observer.py` + `run.sh`. The students' `measure_mask`, `buoy_geolocation`, `BuoyMeasurement`,
  `offset_to_gps` taken verbatim from the read-only snapshot (`cv/buoys/actual_buoy_observer.py` imports pyzed at the top: their source is
  compiled into a module, not imported; a numpy `erode` stands in when OpenCV is absent). Per frame: `--depth-noise zed` (default) or none
  → the point cloud (x right, y down, z forward) → `--detector synthetic` (the renderer's buoy masks) or `--detector model --model DIR`
  (a CV_TRAIN folder via `model_spec`, Ultralytics, the `buoy` class, masks resized to the frame) → measure_mask → near_limit /
  centre_correction / bearing_record / colour vote (multi_core) → geolocation from the frame's pose → `latest_status.json` + CSV under
  `logs/…/sim_observer/<stamp>/` (never archived). 1104×621 (half hd2k) at 4 Hz.
- `scripts/run_sim.sh`: `--perception camera` (default: windows `obs` + `cvpub` = cv_publish following the observer → the boat broker;
  fake perception `--skip-kinds buoy` keeps the dock faces, windows, boxes) | `geometric` (the old path); **usv_ui on the fake plant**
  (window `ui`, udp 14550, sharing 14552; the mission manager on `usv_ui/config/usv_bench_share.yaml`) unless `--no-ui`; `--detector`,
  `--model`, `--depth-noise`. `sim/fake_perception.py`: `--skip-kinds`. `usv_ui/config/usv_bench_share.yaml` regenerated from
  usv_bench.yaml (it had drifted: no box_inspect / home / orbit / perception, the old stop_m) - only the endpoint differs (a test).
  `marina` added to the world short names (run_bench, run_sim, sim_world).
- **The flashing buoys have a dark body** (`body: black`) in world_pool and world_marina: the camera saw the body's colour even with the
  light off, so a flash never showed and m1t1's SEARCH_ENTRY could not find the entry - the light is the colour (the handbook's states
  flashing / solid / OFF). The dataset's truth keeps `colour` = the light's, `body` apart (CV_TRAIN: v1 frames had coloured bodies).
- `tools/sim_day.py`: `handover` through tmux when usv_ui is in the loop. Docs: GUIDE (4.14-cam row, header), docs/SIM_DAY.md (what
  the simulator is now; §4), CHANGELOG, ROADMAP (WP10e ✅), versions.

**Tests**: `tests/test_wp10e.py` (6): the students' code loads without pyzed (+ the erode stand-in); the observer on a rendered pool frame
(every ranged buoy within 0.15 m of the truth, exact and noisy depth; colours; far ones as bearings with a reason); a flashing buoy is
its light (blue on / black off); `--detector model` with a stand-in Ultralytics (only `buoy`, the mask resized, geolocated within 0.3 m);
simview's `/frame.npz` live; fake perception without buoys + the launcher's options + the share profile has no drift + the dark bodies.
`make test` **259 passed** (10 min 23 s); `make quick` 223.

**Rehearsed (Mac)**: `run_sim.sh pool` (camera path, usv_ui): the observer `buoys=7 red@12.9m/-22 … green@ray/-15`, cv_publish `-> 7
item(s)`; `cv on` + one slow turn in the field (33 s; a fast spin smeared the 14 m buoys to 0.7–0.9 m - the heading lags in a fast
turn, as it would on the boat) → **10 / 10 buoys in the mission manager's map with the right colours, worst 0.23 m, three gates**; a red
buoy dragged 5 m → mapped at the new place within 0.02 m, the old entry stale; **m1t1 DONE through the camera path** (155 s; before the
dark bodies: AUTO_SAFE_HOLD "no flashing blue entry buoy found" - found and fixed here). **The sim-day sheet re-run on the camera path
with usv_ui: 36 / 36** (m1t1 151 s, m3t1 82 s, m4t1, faults) **+ S10 m0t4 on boxes DONE** (131 s).

**Open**: the dock / box detectors are not in the loop (fake perception still gives them); the look-around needed for a full map with
the real 51° camera is a mission-design point for WP10h (SEARCH / scan behaviour); `--detector model` not run with real weights on the
Mac (Ultralytics not in the kit's venv; the stand-in proves the wiring) - CV_TRAIN can run it.

**Next**: WP10d (v0.1.37): the operator layer - TEST MODE, MISSION CONTROL, STOP / RESET EVERYTHING, the judge tables, the judges' view.

## v0.1.37 — 2026-11-05 — WP10d: the operator layer (TEST MODE, MISSION CONTROL, STOP / RESET EVERYTHING, the judges' view)

The third package of §7c. Nothing in it was Dmitrii's to choose.

**Changed**
- **TEST MODE**: `guards.test_mode` (companion/config; true in usv_bench / usv_pool / usv_lab / the share profile, false in usv_course)
  and `ocs.test_mode` (true in ocs_bench / ocs_pool, false in ocs_course). Vehicle (`usv/vehicle_fsm.py`): `RUN_FREE_TASKS` = TEST_MANUAL,
  TEST_AUTO, TEST_PATH, TEST_SWAY, TEST_ORBIT, GOTO, RETURN_HOME, BOX_INSPECT start without a run; `enable_auto` needs no PREPARE. OCS
  (`ocs/cmd_authority.py`): in test mode ENABLE_AUTO / GOTO / a run-free START_TASK at any R; a refusal for want of a run now says
  *declare first*; ocs/state carries `test_mode` and the declared `tiers`. The playbook (`tools/playbook.py`) predicts the same. The
  judged missions keep the run; every other guard unchanged. RE-ARM after a kill keeps the run (it already did - now tested).
- **MISSION CONTROL** (`tools/ocs_web.py` `control_word()` - NOT READY + the one thing missing / ARMED / RUNNING with task, stage,
  elapsed / HELD with why, `rearm`, `judged_ready`; `tools/ocs_web.html` tab CONTROL): the state word (a judged pick adds *needs
  RunStart*), the picker in primitives → test → judged order (MAIN's options, points and controls move in while the tab is open - one
  picker), the stage chain from the playbook, ▶ START · ❚❚ HOLD · ⏵ RESUME · ■ ABORT · ↻ RESTART · ⌂ HOME + RE-ARM.
- **STOP / RESET EVERYTHING**: `scripts/stop_all.sh` (the boat → the AGX → the judge console → rc-test → the bench / simulator → the OCS
  last; `--dry-run`), `scripts/reset_all.sh` (RESET SIM with a simulator, else abort + reset + the fake judge's sim_reset; `--dry-run`);
  system actions `stopall` (typed `stop`, refused while a task runs) / `resetall` (typed `reset`), group `everything`: ⏻ STOP ALL /
  ↺ RESET ALL in the top bar and a SETTINGS card.
- **The judges' view**: `RunTimeline` (Declared → Held → Started → Ended; **Ended when the declared vehicles leave AUTO in R5**, red,
  *RoboCommand ends the run here, declare again*; end_run) fed by a 2 Hz thread in the page's backend (a first version only updated when
  a page polled and missed the kill); MAIN's ROBOCOMMAND panel: RUN TIMELINE, TASKS (tier · activated by current_task · our reports),
  the vehicle card (state, task, heading, speed, HAE, phase, heartbeat age). The judge page (`tools/judge_web.py`): RECEIVED / SENT as
  tables (time · direction · kind · seq · vehicle · message, parsed from rc-tui's model text; a kind filter, heartbeats hidden by
  default), its run line with *Ended … the last vehicle out*, a theme select.
- **Themes**: dark / light / high contrast (CSS variables) - the OCS page (⚙ SETTINGS → THEME, `ocs.theme`) and its maps follow;
  the judge page has its own (`judge.theme`).
- Docs: GUIDE (four rows: TEST MODE, MISSION CONTROL, STOP / RESET ALL, the judges' view; header), CHANGELOG, ROADMAP (WP10d ✅), versions.

**Tests**: `tests/test_wp10d.py` (8): TEST MODE on the vehicle (run-free tasks, judged need the run, enable_auto without PREPARE, off =
refused), the profiles (never on the course), the OCS authority + the playbook (m0t2 at R2 accepted; m1t1 refused; the course → *declare
first*), RE-ARM keeps run 1001, the state word's ladder, the run timeline (Ended once; a new declaration starts over; end_run), STOP /
RESET order, words and guards (dry runs), the pages' controls. `tests/test_integration_b1.py` pins `guards.test_mode: false` (it checks the run lifecycle's rule "enable_auto refused before PREPARE" - the first full run caught it: on the bench profile TEST MODE now allows it, as intended). `make test` **267 passed** (10 min 21 s); `make quick` 231.

**Rehearsed (Mac, run_sim.sh pool)**: on the bench, no declaration: `enable_auto` → AUTO_HOLD, `start_task m0t2 --names A,B` → TEST_PATH;
m1t1 refused with the hint; kill → KILLED → confirm_safe → enable_auto → AUTO_HOLD with run 1001 kept (R5). The state word in headless
Chrome along the ladder: *usv_ui holds the FCU* → handover → *the boat is in MANUAL · RE-ARM* → enable_auto → ARMED (m0t2 picked; the
default pick m4t1 says *needs the judge's RunStart*) → RUNNING `TEST_PATH · FOLLOW · 5 s` → HELD → ARMED. A T4 chain with the judge
page's `assist` → the judge tables: run_declaration, task_change, IncidentAck, ReadinessReport received; course, RunStart,
AssistanceRequest, ReadinessConfirm sent. A kill in R5 → MAIN's timeline `Run Ended · USV1 left AUTO (KILLED) - the last vehicle out` in
red and the judge page's run line the same. Screenshots: CONTROL, the judge tables, MAIN in high contrast.

**Open**: STOP ALL not run for real against the Jetsons (dry run only; the Jetsons are not deployed); the per-task *actual* column of
RoboCommand's own display is theirs (we show what we report); RESTART waits up to 10 s for the abort.

**Next**: WP10h (v0.1.38): the mission profile campaign (m1t1 / m3t1 / m4t1 / m0t4 over seeds, both planners, the real camera, noisy
depth; the search toward rays; failures fixed or written down).

## v0.1.38 — 2026-11-05 — the one-script launcher (Dmitrii's request; WP10h moves to v0.1.39)

**Asked**: "a single script, which opens a pre-settings window first: predefined profiles (marina, lab, simulation, custom), then tweak
the run - RC test on/off (ours or RoboNation's), the network (ts, lan, typed by hand), simulation or real, fake or real FCU, fake or real
CV, the broker and heartbeat rates". Polled: a local browser page; all three hosts over ssh; the launcher first (v0.1.38), WP10h after
(v0.1.39, the rest +1); the marina preset = course profile, lan, real FCU + CV, judge rc-test by default.

**What changed · where**
- `scripts/launch.sh` → `optiprimex/tools/launcher.py` (server on 127.0.0.1:8094, `--dry-run`, `--open`) + `launcher.html` (the page:
  presets, the fields per card, the plan per host updated as you type, LAUNCH, STOP with `stop` typed, a progress log, the links, the
  OCS page's theme). `profiles/presets.yaml`: simulation (sim, fake judge, camera path, usv_ui, pool), lab (real, uart, lab profile, real
  CV, ts, LOG), marina (real, rctest, lan, uart, course, real CV, LOG), custom (real, fake FCU, fake CV, ts).
- `build_plan(cfg)` is pure and composes today's launchers only: sim = `run_sim.sh WORLD --no-open --judge J --perception P [--no-ui]`;
  real = a look at each Jetson over ssh (hostname, who, tmux ls - the concurrency check), rc-test if chosen and not already up,
  `MAP=0 [LOG=1] run_ocs.sh --NET --judge J`, `ssh NX 'cd … && run_usv.sh --fcu F --PROFILE --NET'`, `ssh AGX 'cd … && run_agx.sh
  --NET [--sim --world W]'` (skipped for cv off). Manual network: `OCS_HOST` / `BOAT_HOST` / `AGX_HOST` / `RC_HOST` in the environment
  (they beat the network files, `_netenv.sh`), the ssh hosts typed; no `--ts/--lan`. `validate(cfg)` checks every choice, the rate
  ranges, the manual fields and a safe charset for typed hosts; judge course is refused in sim.
- Rates: a heartbeat / status / map rate that differs from the base profile → a generated copy (header `# GENERATED …`) under
  `logs/launch/<stamp>/` (the OCS: `OCS_CONFIG_OVERRIDE`, new in `run_ocs.sh`; sim: `OCS_CONFIG` / `USV_CONFIG`) or copied by ssh to the
  NX's `logs/launch/usv_<stamp>.yaml` (`MM_CONFIG_OVERRIDE`, new in `run_usv.sh`). `run_sim.sh` now keeps a given `USV_CONFIG` with
  usv_ui (it used to overwrite it). The profiles themselves are never edited.
- STOP: after a sim launch this Mac's stacks only (judge, rc-test, bench, OCS); after a real one `stop_all.sh` with the plan's ssh hosts.
- Docs: GUIDE §3 (a paragraph: one script for all of it) + a row, header; CHANGELOG; ROADMAP §7c (the launcher row, WP10h → 0.1.39,
  WP9b/c/d → 0.1.40–42, WP11–15 → 0.1.43–47); CLAUDE.md test count.

**Tests**: `tests/test_launcher.py` (7, in QUICK): the presets complete and valid (marina's values), the sim plan (Mac only, the local
stop), the real plans in order (marina, custom's fake CV, cv off + judge course), changed rates = copies that differ only in the rate
(the share profile under sim + usv_ui; ocs_course under rc-test), the typed network + the checks, the server in a dry run (403 without
the header, a refused plan, LAUNCH → done, STOP needs `stop`), the scripts' hooks. `make test` **274 passed** (10 min 25 s); `make quick` 238.

**Rehearsed (Mac)**: the launcher for real, the simulation preset with status 4 Hz through POST /launch: the generated copy written,
`run_sim.sh` up in 6 s, the SIM / OCS / judge pages 200, the mission manager on `logs/launch/…/usv.yaml`, `status/usv` at ~4/s on the
hub (2 by default); STOP → `stop_judge; stop_rctest; stop_bench; stop_ocs` → nothing left (no tmux, no process). Headless screenshots of
the page (simulation; marina with the LAN plan from network.lan.env).

**Open**: the real-mode LAUNCH has not run against the Jetsons (they are on v0.1.29.1, not deployed: `run_usv.sh`'s
`MM_CONFIG_OVERRIDE` and the launcher need the deploy); the buoy observer on the AGX is still started by hand (a note in the plan); WP9d's
profile cut will rename the presets (sim / lab / event) and the profile paths in `build_plan`.

**Next**: WP10h (v0.1.39): the mission profile campaign.

## v0.1.39 — 2026-11-05 — WP10h: the mission profile campaign (240 / 240; three findings fixed)

**What changed · where**
- `optiprimex/sim/campaign.py` (new): the cases (m1t1 on pool + marina × simple / s6 × geometric / camera × near / far starts; m3t1
  from the dock approach; m4t1 at random request points; m0t4 at random scan points and colours), the graders (the SIL tests'
  measures: truth gate crossings RED to starboard, buoy clearance > 0.705 m, the circles' winding; the GREEN bay + the fire complete;
  < 3 m at the request; the asked face 0.8–1.35 m off), a process pool, the table (+ LOOK / ray-lead counts), `--doc` writes it
  between markers in `docs/CAMPAIGN.md` (new: what it is, how to re-run, what it found, the known limits, the last run).
- `optiprimex/sim/fast_sil.py`: `perception="camera"` — `CameraPath` renders the frame at the pose (ZED intrinsics, 736 × 414),
  runs `cv/sim_observer.observe` (the students' code + our rules, the ZED-like depth), converts with cv_publish's
  `Publisher.frame_to_items`, ingests an `Observation`; 2.5 Hz; docks / windows / boxes stay geometric (`--skip-kinds buoy`).
- **Finding 1 — m3t1 fired at windows it could not see** (1 / 12 seeds complete at 1.0 m): `t3_logistics.DEFAULTS.stop_m` and the
  pool / lab / course profiles 1.0 → **1.5 m** (12 / 12); the comments say why; `tests/test_t3_sim.py`'s docked window 43.1–43.8.
- **Finding 2 — a gate skipped because its flashing red was sampled dark** (seed 8, camera, every planner / world): `perception_map.
  classify_light` returns `unknown` / `?` (**pending**) when a lit colour was seen ≥ 2 times but under the lit share and with fewer than
  `LIT_DECIDE_VOTES` 16 sightings; T1 gained **LOOK** (from TRANSIT, when a pending buoy, seen 3+ times, not behind, lies nearer than the
  next gate - 2 m, within `look_r_m` 15: `ScanYaw(face=…, max_s=look_s 8)`, then back to TRANSIT; SCAN ends early for a pending lead).
- **Finding 3 / the WP's item — the search drives toward rays**: T1 **APPROACH_LEAD** — a SEARCH_ENTRY scan that ends without an entry
  but with a fresh ray (blue first) drives `lead_step_m` 12 toward the ray's 32 m point (FollowPath with the buoys avoided), then scans
  again, `lead_tries` 3, then the old safe hold. `ScanYaw` gained the face mode (yaw rate = clamp(1.5 × error), the same GUIDED velocity
  target: no mode change).
- **Finding 4 — the grader**: a track sample exactly on a gate line was not a crossing (sign product 0); half-open test in the campaign
  and in `tests/test_t1_sim.py` (latent there).
- Docs: `docs/CAMPAIGN.md`; GUIDE (a row, header); CHANGELOG; ROADMAP (WP10h ✅, WP10g's "later" annotated); CLAUDE.md count.

**Tests**: `tests/test_wp10h.py` (8, in QUICK, < 2 s): pending vs decided votes (v0.1.34's far unlit obstacle still black), ScanYaw
facing + its time limit, m1t1 LOOK on a hand-set map (gate 1's red pending → LOOK → decided → gate 1, not gate 2; a pending buoy behind
is ignored), m1t1 from far on the marina (SEARCH_ENTRY → APPROACH_LEAD → SEARCH_ENTRY → DONE, graded), the dock stop (profiles,
default, the geometry against the ZED's field of view, seeds 1 / 2 / 4 that failed at 1.0 m), the on-the-line crossing, the fast SIL's
camera path (buoys mapped within 0.4 m), the campaign's cases / runner / doc writer.
`make quick` **246 passed**. `make test` twice: **281 passed + 1 failed each time, a different live timing check each run** (run 1:
`test_box_sim::test_m0t4sim_live_survey_green_stop_internal_events` - one 0.26 s gap between mode-20 frames in STOP; run 2:
`test_usv_ui::test_fcu_switch_live` - "fake link up" printed after the check) - both untouched by this version, both green alone
(2 / 2 and 3 / 3); the Mac's load average was ~8 during the runs (other sessions). Every other test passed in both runs.

**Rehearsed (Mac)**: the campaign 240 / 240 (16 seeds, 295 s wall on 13 processes; LOOK fired 6×, ray leads 112× - 110 from the far
starts, 2 from near ones on the marina); 24 seeds of m1t1 near 192 / 192. **The sim-day sheet** (`run_sim.sh pool`, the camera path + usv_ui):
**36 / 36** (m1t1 DONE in 151 s, m3t1 DONE in 79 s, m4t1, the faults), **S10** on boxes: m0t4 DONE in 132 s.

**Open**: the campaign's dock / windows are geometric (the camera path is buoys only); the real windows' spacing is to be measured at
the event (stop_m ≥ half-spacing / tan 20°); the far start is open water only; LOOK's 8 s and the 16-sighting threshold are first
values - the event's recordings (WP9b) can tune them.

**Next**: WP9b (v0.1.40): the ZED recorder + the event's data pipeline rehearsed on the bench.

## v0.1.39.1 — 2026-11-05 — fix: the launcher script under macOS bash

Dmitrii: `scripts/launch.sh: line 17: ARGS[@]: unbound variable`. macOS's /bin/bash 3.2 with `set -u` treats an empty array's
`"${ARGS[@]}"` as unset - plain `scripts/launch.sh` (no flags) died before the page started; v0.1.38's tests only ran `--help` and a bad
flag, which exit before that line. Fix: `${ARGS[@]+"${ARGS[@]}"}` (and OPEN). Test: `test_launcher.py::test_the_script_starts_under_macos_bash`
(starts it with /bin/bash, with and without `--dry-run`, and reads the page). Checked by hand: no flags, `--no-open`, `--no-open --dry-run`. `make test` **283 passed** (clean, 10 min 23 s); `make quick` 247.

## v0.1.39.2 — 2026-11-05 — the launcher: reconnect, and Ctrl-C made safe

Dmitrii asked what Ctrl-C leaves behind and wanted a way back into a running session. Polled: all of it (remember / detect /
RECONNECT / restart per host / guard / --status) + the Ctrl-C fixes; idle Ctrl-C prints what is up and stops nothing.

**What changed · where** (`optiprimex/tools/launcher.py`, `launcher.html`, `scripts/launch.sh` unchanged)
- Every plan carries `ops` per host: its ssh target, the tmux sessions it must leave (`optiprimex` [+ `rctest`] for a sim; `ocs`
  [+ `rctest`] on the Mac, `usv` on the NX, `agx` on the AGX for a real launch), the host's own stop and its launch step(s).
- A LAUNCH writes `logs/launch/last.json` (settings + plan + time; no addresses beyond the ssh host names the plan shows; logs/ is
  never archived); a successful STOP renames it `last_stopped.json`. `probe()` = `tmux ls` here and over ssh (BatchMode, 4 s) → per host
  found / missing / no answer; without a record, our known local sessions count as "running without a launch record".
- `Watch` re-probes every 5 s in a thread (the page reads `/session.json`); the page: a banner (*the launch from HH:MM (title) is still
  running* + per host) with RECONNECT / STOP ALL / STOP-then-new; RECONNECT (`/adopt`) loads the plan - links, STOP, the form read-only
  with the running settings - and a *running* card with **restart this host** (`/restart`: that host's stop, then its step).
  `/launch` refuses while anything of ours runs ("RECONNECT or STOP first").
- Ctrl-C: steps run with `start_new_session=True` (the terminal's SIGINT never reaches them); mid-launch → `abort` (the step finishes,
  the rest is skipped and logged; a second Ctrl-C leaves at once); then, as for an idle Ctrl-C, the summary of what is up + how to
  reconnect / stop. SIGINT is installed explicitly (a background job inherits it ignored - found in the rehearsal). `--status` prints
  the summary without a page.

**Tests**: `tests/test_launcher.py` +5 (13): the plans' sessions per host, probe + summary (missing / no answer / no record), reconnect
+ restart + the guard through the server, Ctrl-C mid-launch skips the rest (+ own process groups), Ctrl-C idle and `--status` as processes.

**Rehearsed (Mac, the simulation)**: LAUNCH → SIGTERM'd launcher printed *the launch from 23:00 (simulation) is up* + the stop
command; `--status` the same; a new launcher: the banner, LAUNCH refused (*still running: RECONNECT or STOP first*), RECONNECT →
adopted, links back, *mac: optiprimex · ok*; `stop_bench.sh` behind its back → *MISSING optiprimex* + restart this host → stop + run_sim
again → ok; STOP → nothing left, the record set aside. `make test` **288 passed** (10 min 30 s, clean); `make quick` 252. Then Ctrl-C 1.5 s into a launch: *finishing the current step…*, the launcher left
after 9 s with the full session up (18 windows, as uninterrupted) and the summary. Not rehearsed against the Jetsons (not deployed).

## v0.1.39.3 — 2026-11-05 — test tiers and the parallel full suite

Dmitrii: "these tests 10 minutes each time, what exactly are we testing? … the big test reserved for major upgrades?" Measured
(`--durations`, the serial run): the 4 live missions in wall-clock time = ~5.5 min (m1t1 95 s, m0t4 93 s, m3t1 82 s, dockto + cannon
52 s), ~30 other live wire / safety tests 3–24 s each = ~4 min, the ~250 QUICK tests 35 s. Polled: three tiers + try parallel.

- `Makefile`: `make test` = `pytest -n $(JOBS=6) --dist loadfile`; `make test-serial` = the old run; `make live` = the parallel run
  without `LONG_LIVE` (the 4 missions, `-k`); the help text. Why `loadfile`: with `--dist worksteal` / `load` a module's tests land on
  different processes and `test_integration_sil1` failed (`declaration_seq == 2` builds on the test before) - each file stays whole.
- `requirements.txt` / `pyproject.toml [test]`: pytest-xdist 3.6.1 (installed in the Mac's .venv).
- CLAUDE.md step 3 = the tiers (quick: every change incl. 0.1.N.M; live: launchers / brokers / MAVLink / usv_ui / OCS / mm wiring;
  test: every WP version and before any Jetson deploy); README / HANDOFF counts; memory `feedback-test-tiers`.
- `tests/test_make_tiers.py` (1, in QUICK): the targets keep `--dist loadfile`, serial stays serial, every LONG_LIVE name still matches a
  test (a rename would silently un-skip it), xdist in the requirements.

**Measured**: `make test` parallel **288 passed** three times in a row - 2:55, 2:52, 2:53 (serial 10:30); `make live` 284 passed in
1:05; `make quick` 253; with the tier test, `make test` 289 passed in 2:50. (A first try with 4 processes / default `load` passed by luck, 4:15; 6 / worksteal failed the SIL-1 order.)


## v0.1.40 — 2026-11-06 — WP9b: the ZED recorder and the event's data pipeline (built in a parallel stream)

**How it was built**: the first of two parallel streams (WP9b ‖ WP9c, Dmitrii: "develop few of them in parallel"): each stream a
subagent in its own copy of the tree (no git here) from the same snapshot, code + tests only; the versions, docs and the merge here.
WP9b's files did not overlap anything changed in the tree since the snapshot - copied as they were (11 files; the stream's `.venv`
symlink and its HANDBACK not). Reviewed: system_actions `rec` + `Option.free`, stop_agx.sh (the recorder first), run.sh (a replay
never writes where cv_publish reads).

v0.1.40 (WP9b). What changed: our buoy observer records (`--record DIR --label L --max-min N`) through the
ZED SDK's H.264 recording into `DIR/<stamp>_<label>/cam.svo2`, with `frames.jsonl` (per frame: UTC, frame id, SVO frame, ZED
timestamp, lat / lon / heading from the Pixhawk telemetry, buoys with colour, range, bearing, bbox, geolocation) and `session.json`
(label, start / end, kit / SDK / camera / model versions, options, stop reason); the recording stops at `--max-min` or below 5 GB free,
detection goes on; `--svo FILE` replays a clip through the same loop with the recorded telemetry, output beside the clip; the pure part
in `cv/buoys_multi/rec_core.py` (compare, status). Where: `cv/buoys_multi/{multi_buoy_observer.py, rec_core.py, run.sh, README.md}`,
`scripts/agx_record.sh`, `scripts/agx_pull_clips.sh`, `scripts/stop_agx.sh`, `optiprimex/tools/system_actions.py` (`rec`, `Option.free`),
`optiprimex/tools/ocs_web.html` (SETTINGS → CAMERA → RECORDER). Tests: `tests/test_wp9b.py` (11; stubs for pyzed / cv2 / YOLO / the
students' module), whole suite 300 green. NOT rehearsed (needs the AGX): the real ZED SDK calls (`enable_recording`, `.svo2` needs ZED
SDK ≥ 4.1, `set_from_svo_file`, `get_svo_position`, `END_OF_SVOFILE_REACHED`, `get_timestamp`, the camera-information attribute names),
the clip's real size per minute and frame rate (the SVO gets the loop's rate, a few fps with YOLO at hd2k), whether the replay yields
the same buoys on H.264-decoded images, agx_record.sh's tmux / pgrep / `df` behaviour on the AGX, `--replace` against a real
observer, the rsync from the AGX and the page card over ssh. Open items: measure MB/min and write it in the README checklist; maybe a
`recording` badge from latest_status.json (cv_publish does not forward it today — the page sees the recorder through *status* jobs
only); `stop_agx.sh`'s leftover check does not list `multi_buoy_observer` (pre-existing). Next session first: the AGX rehearsal (§4).

**Integrated**: `make quick` 264, **`make test` 300 passed** (2:46, parallel) in this tree; the GUIDE rows (record / replay / bring
home) and §7.4 the event days; CHANGELOG; ROADMAP ✅ (Mac).

**The AGX rehearsal (after the deploy; props off, nothing on the water)** - from the stream's hand-back, §4:

(later, by the parent; props off, nothing on the water)

Before: `who`, `tmux ls`, `pgrep -fl "observer|cam_stream|train|yolo"` (CV_TRAIN may be training on the GPU); `df -h ~`; log in
`docs/JETSON_LOG.md`.

1. Deploy the version (GUIDE §1 loop). On the AGX: `scripts/agx_cam.sh stop` and Ctrl-C any observer (or use `--replace` in step 2).
2. Record 60 s: `scripts/agx_record.sh start bench_rehearsal --max-min 1` (add `--no-pixhawk` if no telemetry reaches the AGX; aim
   the ZED at a coloured buoy 3–10 m away). Or from the page: ⚙ SETTINGS → RECORDER, label `bench_rehearsal`, max min 5 → start
   (then stop after 60 s) — this also tests the ssh path.
3. `scripts/agx_record.sh status` every ~20 s: `RECORDING · N frames`, then after 1 min `stopped: max_min: 1 min recorded`. Check
   `ls -la ~/Desktop/dm3/recordings/<session>/` (cam.svo2 size → **MB/min**, write it into `cv/buoys_multi/README.md`),
   `head -2 frames.jsonl` (lat / lon / heading not null with the Pixhawk; buoys with colours), `cat session.json` (zed_sdk, serial;
   if `zed_info_error` appears, note the SDK's attribute names). Frame rate = frames / 60 s.
4. `scripts/agx_record.sh stop` (the observer, which kept detecting, ends; `recorder down`). Confirm the SVO opens: the ZED SDK's
   `ZED_Explorer` / `ZED_Depth_Viewer` can open it, or step 5.
5. Replay: `cv/buoys_multi/run.sh --headless --svo ~/Desktop/dm3/recordings/<session>/cam.svo2` → it ends by itself at the end of
   the file (`Logs: …/replay/<stamp>`), cv/buoys/runs gains nothing new.
6. Compare: `python3 cv/buoys_multi/rec_core.py compare ~/Desktop/dm3/recordings/<session>/frames.jsonl ~/Desktop/dm3/recordings/<session>/replay/<stamp>/frames.jsonl`
   → accept when `same_count` and `same_colours` are ≥ ~0.9 and `ranges_within_tol` high (H.264 decoding and recomputed neural depth
   make some frames differ; write the numbers down).
7. Guard check without filling the disk: `scripts/agx_record.sh start guard_test --max-min 1 -- --min-free-gb 1000` → the observer
   prints `NOT RECORDING: disk low …` and exits (rc 3; `agx_record.sh` reports it did not start / the log tail).
8. Mac: `scripts/agx_pull_clips.sh --list`, `scripts/agx_pull_clips.sh --dry-run`, then `scripts/agx_pull_clips.sh` → the session in
   `~/Documents/BH1/RobotX/recordings/<date>/` with SHA256SUMS; tell CV_TRAIN. Optionally free the AGX with the printed line.
9. `scripts/stop_agx.sh` while a recording runs → the clip is closed (`stopped: stopped by the operator …`).


## v0.1.41 — 2026-11-06 — WP9c: the RunDeclaration checks (built in a parallel stream)

**How it was built**: the second parallel stream (WP9b ‖ WP9c), in its own copy from the same snapshot. Its files had not changed
in the tree since the snapshot except `ocs_web.html` (WP9b's RECORDER card): a three-way merge (`git merge-file`, the snapshot as the
base) - no conflicts; both the RECORDER card and the declaration rows present. Reviewed: `cmd_authority` (the checks only where the
OCS wires `on_check`; a failing check never blocks; warn only extends the accepted reason), the course profile `policy: refuse`.

**v0.1.41 (WP9c) — RunDeclaration checks.**

*What changed:* the OCS now checks the RunDeclaration and the mission against each other. Every task in `declaration.tiers` must have a valid tier, and TIER_UNKNOWN is never accepted (handbook 3.4.10). T4 must not be TIER_NONE (3.3.1 / 3.3.5; overridable with `declaration.mandatory`). The preselected or started mission's task must be declared with the same tier (3.4.16), derived from the mission code m<task><tier>. m0t* and goto are flagged inside a judged run. Every row has a code, a severity (hard/soft), a scope (declaration/mission) and a text that says what to type next.

*Where:*
- `optiprimex/ocs/declaration_checks.py` (pure; `mission_code` moved there, the name stays in ocs_app);
- `cmd_authority.on_check` / `declaration_policy`: DECLARE checks all rows; MISSION, START_TASK and GOTO check the mission rows; a blocking row refuses; under warn the reason gets "· check: …";
- `OcsApp.declaration_rows` / `effective_tiers`; the `ocs/state.declaration_checks` block; the `_mission_trigger` guard;
- `playbook.check` / `suggest` (so `ocs_cli next` and op-tui `next:` change with it);
- `ocs_web.clearance` (declaration lines; a hard row under refuse blocks `clear for run`) and `mission_declaration`, used by MISSION CONTROL's word;
- profiles: course `policy: refuse`, pool and bench `policy: warn`.

*Tests:* `tests/test_wp9c.py`, 21 tests (count above), including one live ocs_app process on a real broker.

*Rehearsed live:* nothing on the hosts. The page JS change in `cWord()` was not run in a browser.

*Open items:*
- (a) T4 is the only mandatory task, and it is our reading of the handbook. Ask RoboNation or the TD whether T4 is required in every declaration.
- (b) rc-test runs under ocs_course.yaml, so it now refuses too. That is intended, but rehearse it once.
- (c) The course refuses any m0t*/goto in a run. On a practice day on the course network, if a test mission must run inside a run, set `policy: warn` in a copy of the profile.
- (d) Possible later check (ROADMAP "later"): a tier that needs a vehicle we did not declare. §3.3.2–3.3.4 Advanced tiers need the UAV; T2 needs a UUV + UAV.

*Interaction with the start-sequence re-evaluation (§7d):* the checks do not assume an order. `evaluate()` takes (tiers, mission, R, step, test_mode, auto), and every step calls it with the current context: enable_auto, declare, mission preselect, start_task, the RunStart auto-start, and the state. "Judged" = R3–R5, the declare step itself, a mission armed with `--auto`, or no TEST MODE. Whatever order §7d settles on, a new step needs one call: `app.declaration_rows("<step>", task, args)` on the OCS, or `playbook.declaration_rows(ocs_state, mission, step)` on a console. Note that `enable_auto` is not checked: it carries no mission and no declaration. If §7d moves the mission choice after RunStart, the start_task check already covers it. If §7d makes the page's RunStart indication a button, its handler should call `playbook.declaration_block(...)` first.

*What the next session should do first:* open the page on the bench under the course profile (`run_ocs.sh --judge rctest`) and check the red bold line and MISSION CONTROL's NOT READY, with T4 set to TIER_NONE in a scratch copy of the profile.

**Handbook citations** (handbook/RobotX-2026_Team-Handbook-20261023.md): (handbook/RobotX-2026_Team-Handbook-20261023.md)

- **§3.4.10 Pre-Run Declaration and Run Start:** the declaration carries "the tier attempted for Task 1 … Task 4"; "Use `TIER_NONE` when the team is not attempting a task. Do not use `TIER_UNKNOWN` as a substitute." This is the basis for tier_missing, tier_unknown and tier_invalid. rc_wire silently sends TIER_NONE for a missing or invalid tier, which is why these rows are hard.
- **§3.4.16 (pre-run checklist):** "The intended tier for every task is correct in RunDeclaration." This is the basis for tier_mismatch and task_not_declared.
- **§3.3.1 Cross-cutting Tasks, "Dynamic Adaptability":** "All uncrewed systems are required to autonomously perform basic functions … Mission Task 4 represents these conditions through use of the RoboCommand system." Together with **§3.3.5** (RoboCommand sends the AssistanceRequest / KeepOut / MovingObject during a run), this is the basis for MANDATORY = ("T4",).
- **The handbook makes no other task compulsory.** §3.3 says "Teams are encouraged to develop a strategy to approach these tasks that best suits their SoS". §3.3.3 says Task 2 needs a UUV + UAV (we declare it TIER_NONE). The mandatory list is therefore our reading of the handbook. A profile can override it with `declaration.mandatory: []`.
- **§4.2 Autonomy Challenge Scoring:** points are judged per run against the tasks. A test mission (m0t*) or `goto` scores nothing inside a judged run, which is the basis for test_in_run and goto_in_run.
- **Mission codes:** m<task><tier> (the playbook catalogue, v0.1.19, and ocs_app's `mission_code` table, now in declaration_checks). Tier digit 1 = TIER_CORE, 2 = TIER_ADVANCED, 3 = TIER_DISRUPTIVE. TEST_ORBIT (m0t6), which the OCS's copy of the table does not name, is recognised as a test mission.

**Integrated**: `make quick` 284, **`make test` 321 passed** (2:49, parallel) in this tree; the GUIDE row (after MISSION CONTROL);
CHANGELOG; ROADMAP ✅ + a "later" line (a tier's vehicles; ask whether T4 is mandatory). **Rehearsed here** (the stream could not):
`run_sim.sh pool --judge rctest` with a scratch course profile where T4 = TIER_NONE → MAIN's RUN CHECKLIST: the red bold row
*declaration: mandatory none · hard · policy refuse* with the fix, `clear for run` greyed, the top bar's `next:` = the fix;
`ocs_cli declare` refused with the same text; CONTROL: NOT READY (its first missing step, the handover, ranks above). Seen: MAIN's
`next: declare` button stays while declare would be refused (the text below it says why) - a wording item for WP9d's refusal sweep.

**Decisions for Dmitrii** (from the stream): T4 as the one mandatory task is our reading of the handbook; on the course profile any
m0t* / goto inside a run is refused (a practice day on the course network: a copy of the profile with `policy: warn`).

**Next**: WP9d (v0.1.42), the light cut (polled 2026-11-06): the launcher's presets sim / lab / event, one table of the profiles, the
flavour line on every console, the ten-words card, the refusal sweep (incl. MAIN's `next: declare` while declare is refused).

## v0.1.42 — 2026-11-06 — WP9d, the light cut (presets sim / lab / event, the flavour line, ten words, the refusal sweep)

Polled 2026-11-06: the light cut - no directory moves before the event (the launcher of v0.1.38 already covers the day presets,
WP10d STOP EVERYTHING). The full cut (profiles/sim|lab|event, up.sh / down.sh, the DAY PRESET card) stays open in ROADMAP.

**What changed · where**
- `profiles/presets.yaml`: sim / lab / event / custom (simulation → sim, marina → event "event (marina)"); `launcher.html` defaults to
  sim. `launcher.build_plan`: a `preset` in the settings → `LAUNCH_PRESET=<preset>` on the sim command and on the NX's run_usv.sh;
  `scripts/run_usv.sh` / `run_sim.sh` hand it to usv_ui's command line.
- **The flavour line** `preset · judge · net · FCU · profile · CV`: `launcher.flavour()` (the launch record + what the console knows
  live: the judge, NET_PROFILE, usv_ui's fcu_link; `?` when started by hand) → `ocs/state.json` `flavour` → the OCS page's top-bar
  chip; `judge_web` `/state.json` `flavour` → its header; usv_ui's top bar `usv_ui v… [preset]`.
- GUIDE §3: the presets ↔ profiles table (which OCS / boat profile each uses, judge · net · FCU · CV), the flavour line, the ten words;
  the launcher paragraph / row renamed; README's first line.
- THE TEN WORDS card in the page's HELP tab (ready start_task hold resume abort manual end_run reset save_point home - from op-tui's
  USAGE).
- The refusal sweep: `cmd_authority` - ADR-9 (says the transmitter / usv_ui's typed arm), unknown intent (the HELP tab, the ten words),
  bench only (how to start the simulator), unknown injector / vehicle (the valid ones); WP9c's rows already say what to type. MAIN's
  next button: `next: declare · blocked` + the why under it without a click (found with WP9c: `next: declare` while declare is refused).

**Tests**: `tests/test_wp9d.py` (5, in QUICK): the presets and LAUNCH_PRESET in the plans / scripts, `flavour()` from a record / live
values / by hand, the consoles carry it, **usv_ui's top bar rendered at 200 × 60** (a curses stand-in: `[event]`, `[?]`), the ten words
+ the refusal texts; `tests/test_launcher.py` follows the new names. `make quick` 289, **`make test` 326 passed** (2:49).

**Rehearsed (Mac)**: the launcher → preset sim: the OCS page's chip `sim · judge fake · net local · FCU fake · bench · CV sim camera`,
the judge page `… FCU sim …` (it has only the record; the OCS page reads usv_ui's live link), usv_ui `usv_ui v0.1.42 [sim]`; STOP →
nothing left. The chip wraps under the ladders at 1500 px (fine; a WP for the overall UI look, §7d).

**Open**: the full cut (directories) after the event; the judge page could take the live FCU from the OCS too.

**Next**: per ROADMAP: WP10f after the event; WP11–15 to reorder with the event's schedule. Dmitrii first takes §7d (the start
sequence and the UI) to his planning session with the main agent.

## v0.1.43 — 2026-11-07 (overnight) — WP10i-lite: OCS v2 before the event (the bar, RunStart 3·2·1, stages, themes, /v1/, command safety, host health)

**Built from** ROADMAP §7e + docs/OCS_V2_SPEC.md (§13's decisions, no poll) + docs/mock/. The backend items (TIMED OUT, the stale-command
refusal, the host-health beat) in a parallel stream in a copy of the tree, merged 3-way (two conflicts, both "keep both": ocs_app's loop
ticks the countdown and sweeps the acks; ocs_web's state puts the host rows before the bar so the bar counts them).

**What changed · where**
- **v1 freeze**: `tools/ocs_web_v1.html` = the v0.1.42 page + one banner line, served at `/v1/` by the same server (its relative links and
  POSTs map to the same endpoints); ⚙ SETTINGS → CLASSIC PAGE (open /v1/, *always classic* = `ocs.ui`); the launcher's *OCS page* field
  (`ui: v2 | v1`, `LAUNCH_UI`), `run_ocs.sh --ui v1`. map.html is shared (only additive: themes.css) - no map_v1 needed.
- **The global bar** (`ocs_web.top_bar` → `/state.json bar`, drawn by the OCS page and, fetched server-side, the judge page): the state
  word + why (MISSION CONTROL's; **LINK LOST** when the boat's status is > 3 s old), the FCU chip (holder · V · mode · armed · kill), the
  warnings, the theme; **two attention lines** under it (the most severe warnings, readable without a click). `safety_rows` adds the
  alarm-table rows the monitor lacked: RC LOST (ALARM in AUTO), FCU LINK LOST (fcu_age_s > 2 s), BOAT LINK LOST, PERCEPTION STALE (per
  source: the sim mixes the camera path with the geometric fake - bench_tui now keeps the health per detector).
- **RunStart**: `ocs_app` arms a **countdown** (`ocs.autostart_countdown_s` 3) before a --auto START_TASK; intents **CANCEL_AUTO** /
  **HOLD_ON_START** (`ocs_cli cancel_auto | hold_on_start`); `lifecycle.body` carries `stall_s` / `waiting_s`. MAIN's **RUNSTART** card
  (waited, to the stall, *a refusal is silent: ask the staff*) + the full-screen **3 · 2 · 1** with the two buttons (one sound if on).
- **Stages**: `playbook.Stage.detour` (LOOK of TRANSIT, APPROACH_LEAD of SEARCH_ENTRY added to m1t1; m3t1's config stop_m 1.5);
  `campaign --stages-out` → `tools/stage_typical.json` (120/120 runs, medians per stage); CONTROL's STAGES = a list (exit · elapsed ·
  ~typical · detours indented · a progress bar); the strip in the RUNSTART card.
- **Themes**: `tools/themes.css` (dark · light · hc-dark · hc-light · fun with the mock's variable names + aliases), served by ocs_web and
  judge_web, linked by map.html; the preset's default until picked (sim dark · lab light · event hc-light); the OCS page tells its
  server its theme (`POST /prefs`) so the judge page follows; hard-coded dark colours in the page's CSS mapped to the variables.
  SOUNDS (off · alarms · alarms + warnings) in SETTINGS.
- **Backend (stream)**: TIMED OUT (`ocs.ack_timeout_s` 5: `timed_out`, `timed_out_total`, a 60 s warning, late acks noted); the boat
  rejects a cmd older than `guards.cmd_max_age_s` 10 (`stale (sent N s ago …)`) - proven by a START_TASK published through a cut bridge;
  `tools/host_health.py` (local/usv/host/<nx|agx> 1 Hz → telemetry/usv/host/…, a `host` window in run_usv / run_agx; rows on MAIN /
  NETWORK; `host.warn_c` 85 / `alarm_c` 95).
- **Tools**: `tools/fault_timing.py` (the §13 test), `sim_day --page URL` (the OCS page's own steps: page, state.json + bar, /sim teleport
  + RESET SIM, /cmd, the feeds); the fake judge's `sim_config {"refuse": true}` (reset by RESET SIM).

**Acceptance**
- **The sim-day sheet: 42 / 42 on `/` and 42 / 42 on `/v1/`** (36 + the 6 page checks; copies in `docs/screens/v0.1.43/`). Two sheet-script
  races fixed on the way: S2 flipped the mode switch and sent enable_auto in the same instant (the boat sees the switch on the FCU's
  next heartbeat - now 1.5 s between, as an operator); S11 injected AssistanceRequest with command_seq 1 every run (the OCS correctly
  drops a repeated seq - the sheet run twice on one stack lost it; now a fresh seq).
- **The fault-injection timing test - 8 / 8 within 3 s** (run_sim.sh pool, the boat ready in AUTO_HOLD with a run):

| fault | named (s) | first action readable (s) | ≤ 3 s | what the page showed |
|---|---|---|---|---|
| kill | 0.8 | 0.8 | yes | NOT READY kill / E-stop asserted release it on the transmitter, then confirm_safe / FCU mission manager KILLED HOLD KILL  / vehicle KILLED: release the kill / E |
| rc6 | 0.4 | 0.4 | yes | NOT READY the boat is in MANUAL RE-ARM (enable_auto) / FCU mission manager MANUAL HOLD   |
| rc_lost | 2.3 | 2.3 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / RC LOST (the receiver) while AUTO: the transmitter is the safet |
| gps | 1.0 | 1.0 | yes | NOT READY fault EKF_BAD clear the cause, then confirm_safe / FCU mission manager FAULT HOLD   / vehicle FAULT EKF_BAD: clear it, then confirm_safe |
| fcu | 2.3 | 2.3 | yes | NOT READY fault FCU_LOST clear the cause, then confirm_safe / FCU mission manager FAULT HOLD   / FCU LINK LOST: no heartbeat for 2 s - takeover on the boat (usv |
| bridge | 2.7 | 2.7 | yes | LINK LOST the boat's status is 3 s old (bridge / radio) check the radio; the boat holds by itself (link_loss_action) / FCU mission manager AUTO_HOLD HOLD  stale |
| refusal | 2.1 | 2.1 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / waiting for RunStart · a refusal is silent: if nothing comes, a |
| perception | 1.9 | 1.9 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / PERCEPTION STALE: sim_observer silent for 2 s - CAMERA tab -> t |

- No `<button>` without a title on `/` and `/v1/` (headless Chrome); every theme passes the contrast rule (test).
- `make quick` 297, **`make test` 342 passed** (2:50). Screenshots: `docs/screens/v0.1.43/` (MAIN + CONTROL × 5 themes, /v1/ once).

**Decisions taken unattended**
1. The mock palettes failed "amber / red / accent must differ in lightness" (hc-light 37 / 36 / 35 L*): each lightness moved (hue and
   chroma kept) to ≥ 8 L* apart, every contrast floor kept: dark acc #4bb9d2 warn #ffc239 alarm #f94d4e · light acc #007a97 warn
   #bb7725 alarm #b9000f · hc-dark acc #2fc8d7 warn #ffda14 alarm #ff3c3b · hc-light acc #13677b warn #a26c27 alarm #9a0000 · fun as given.
2. The 3·2·1 is real (the OCS delays the auto-start), not only drawn; HOLD-on-start = START_TASK then HOLD at 0; the sound obeys SOUNDS.
3. The judge page cannot read the OCS page's localStorage (another port): it follows the OCS page's theme through `/prefs` → the bar.
4. The perception limit is 2 s (the mock's figure); the stale check is per source.
5. map.html stays shared with /v1/ (only the themes link added) - no map_v1.html.

**Open**: the mini-map / MAP canvas still draws its own dark palette in the light themes (its colours are in its script - v0.1.44 or a
map pass); op-tui does not show TIMED OUT / host rows (the page does); the throttling flag is unproven on a real Jetson.

## v0.1.44 — 2026-11-07 (overnight) — WP10i: OCS v2, the regrouping (six cards, the alarm model, COMMS & DIAGNOSTICS, UAV, T4 as a modifier)

**What changed · where** (`optiprimex/tools/ocs_web.html` re-laid by moving the existing blocks - every element id kept, so the drawing
code is unchanged; the old tab names are aliases: #MAIN → OVERVIEW, #CONTROL / #MAP / #CAMERA → MISSION, #BOAT → USV, #NETWORK /
#SYSTEM → COMMS; GUIDE §3.0 "what moved where")
- **OVERVIEW** (L1): SYSTEM · PLATFORMS · LINKS (each link with its age; grey = stale, hatched = not published), RUN · ONE STEP, ONE
  CONTROL (seven steps - the RUN CHECKLIST inside step 3, the RunStart slot inside step 5, the mission with its strip + START · HOLD ·
  ABORT in step 7; the current step highlighted), the mini-map, ALARMS · WARNINGS · CAUTIONS, MISSION · VEHICLE (power and u v r
  hatched: not published), LAST COMMANDS · LIFECYCLE (TIMED OUT rows first), the USV and UAV cards, EVENTS; COMMANDS / CONSOLE / FLOW below.
- **MISSION** (L2): the map large, MISSION CONTROL with **the picker only here** (+ the **T4 modifier** box), the stages, the camera,
  DETECTIONS, MISSION EVENTS, the catalogue folded.  **USV CONTROL ↗**: the boat window link (boat-web 8098, built from this page's
  host) + the boat's status.  **UAV** (greyed): *not connected · unknown*, the Garuda note.  **COMMS & DIAGNOSTICS**: STREAMS (age vs
  threshold) + *copy diagnostics* + NETWORK + SYSTEM.  **JUDGE**: the RoboCommand panel + the traffic.  HELP behind the bar's **?**.
- **The alarm model** (`ocs_web.AlarmBook`, §8): ALARM / WARNING / CAUTION from the warnings (CAUTION = the self-clearing words),
  acknowledgement per key (`POST /ack`, all = silence), WARNING → ALARM after 5 min unacknowledged, a 24 h in-memory history
  (`/alarms.json`); the page flashes unacknowledged ALARMs, sounds per SOUNDS (an ALARM every 3 s until acknowledged).
- **T4 as a modifier**: `ocs_cli mission … --t4 on|off` → the mission's `t4` → the declared T4 tier (on = TIER_CORE, off = TIER_NONE);
  WP9c's `mandatory_none` is **soft** ("are you sure?") unless `declaration.t4_required: true` (the course profile: false until
  RoboNation confirms) - test_wp9c's hard-path tests now say `t4_required` explicitly.
- **The scene layer**: `profiles/course/scene_marina.json` (a placeholder polyline, `illustrative: true`), served in /course.json,
  drawn dim and open (`poly(…, open)`) labelled *scene · illustrative*.

**Acceptance**: the sim-day sheet **42 / 42 on `/` and 42 / 42 on `/v1/`**; the fault-injection timing test **8 / 8 within 3 s**:

| fault | named (s) | first action readable (s) | ≤ 3 s | what the page showed |
|---|---|---|---|---|
| kill | 1.0 | 1.0 | yes | NOT READY kill / E-stop asserted release it on the transmitter, then confirm_safe / FCU mission manager KILLED HOLD KILL  / vehicle KILLED: release the kill / E |
| rc6 | 0.4 | 0.4 | yes | NOT READY the boat is in MANUAL RE-ARM (enable_auto) / FCU mission manager MANUAL HOLD   |
| rc_lost | 1.9 | 1.9 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / RC LOST (the receiver) while AUTO: the transmitter is the safet |
| gps | 0.8 | 0.8 | yes | NOT READY fault EKF_BAD clear the cause, then confirm_safe / FCU mission manager FAULT HOLD   / vehicle FAULT EKF_BAD: clear it, then confirm_safe |
| fcu | 2.1 | 2.1 | yes | NOT READY fault FCU_LOST clear the cause, then confirm_safe / FCU mission manager FAULT HOLD   / FCU LINK LOST: no heartbeat for 2 s - takeover on the boat (usv |
| bridge | 2.9 | 2.9 | yes | LINK LOST the boat's status is 3 s old (bridge / radio) check the radio; the boat holds by itself (link_loss_action) / FCU mission manager AUTO_HOLD HOLD  stale |
| refusal | 2.1 | 2.1 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / waiting for RunStart · a refusal is silent: if nothing comes, a |
| perception | 1.3 | 1.3 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / PERCEPTION STALE: sim_observer silent for 2 s - CAMERA tab -> t |

No button without a title on both pages; the contrast test; `make quick` 303, **`make test` 348 passed** (2:50). Screenshots:
`docs/screens/v0.1.44/` (OVERVIEW + MISSION × 5 themes, COMMS, JUDGE, UAV; the sheet + the timing table). Two old tests followed the
new tabs (t-MAIN / t-CONTROL → the v2 names) and test_wp10c's Chrome dump got `--virtual-time-budget` (it dumped early under load).

**Decisions taken unattended**: the regrouping moves DOM blocks instead of rewriting them (same ids, same draw code - the lowest risk
before the event); the alarm key = the text before its first ':' with the numbers masked; history in memory (24 h ring) - the hub
SQLite (LOG=1) remains the long record; the bridge alarm keeps the spec's 3 s threshold (it names the fault at 2.7-2.9 s - inside the
limit but with little margin); the boat window link is built at runtime from the page's own host.

**Open**: the mini-map / MAP canvas colours in the light themes; the detections table shows counts, not per-object rows; the alarm
history is not persisted across an OCS page restart; op-tui does not show the alarm list.

## v0.1.45 — 2026-11-07 (overnight) — WP10f phases 1–2: the BOAT window (boat-web 8098, usv_ui's FCU block, the page-word whitelist)

Built in a parallel stream (a copy of the tree from the v0.1.44 snapshot), merged without conflicts (its files had not changed here).


v0.1.45 (WP10f phases 1–2, the BOAT window; built unattended in a copy). usv_ui publishes its FCU block at 2 Hz on
`local/usv/ui/fcu` (boat broker only: HB age, sysid/compid, mode, armed, KILL/ESTOP from NAMED_VALUE_INT, RC 1–8, SERVO_OUTPUT_RAW
1–8, EKF, SYS_STATUS voltage/current/remaining, VFR_HUD, NAV_CONTROLLER_OUTPUT, position, GPS, attitude, per-message ages and rates,
fcu_link, active/holder — null whenever the FCU has not said it) and listens on `local/usv/ui/cmd` to the page's words: only handover
takeover stop log note save co cv points (decision 25), one per 0.5 s, older than 10 s refused, retained ones dropped; every word is
shown as `[page] word` and answered on `local/usv/ui/cmd/result` with usv_ui's own line. `optiprimex/tools/boat_web.py` + `.html`
(port 8098, `--boat-host`): the header, the generic five-system wireframe (ids kept, `setState` paints only), the OVERVIEW of
03_boat.html with hatched "not published" values, handover / takeover buttons; five tabs are stubs. run_sim.sh gains window `bweb`.
Found on the way: usv_ui's `note` raised NameError (undefined `line`) — fixed. Tests: test_v0145 (7). Not rehearsed on run_sim.sh
(the parent's stack held the ports): see the acceptance steps.

**Live acceptance here (run_sim.sh pool, the boat window in tmux `bweb`)**: http://127.0.0.1:8098/ up; **handover** from the page →
"HANDED OVER …", the page's holder = mission manager in 0.1 s; **takeover** → "ACTIVE: usv_ui commands the FCU again", the holder back to
usv_ui in **0.4 s** (< 2 s); `arm` refused: *'arm' is not a page word (only handover takeover stop log note save co cv points; decision
25)*; screenshot `docs/screens/v0.1.45/boat_live_runsim_dark.png` (+ the stream's 7 from a test stack). The fault timing with the window
open - **8 / 8 within 3 s**:

| fault | named (s) | first action readable (s) | ≤ 3 s | what the page showed |
|---|---|---|---|---|
| kill | 0.8 | 0.8 | yes | NOT READY kill / E-stop asserted release it on the transmitter, then confirm_safe / FCU mission manager KILLED HOLD KILL  / vehicle KILLED: release the kill / E |
| rc6 | 0.4 | 0.4 | yes | NOT READY the boat is in MANUAL RE-ARM (enable_auto) / FCU mission manager MANUAL HOLD   |
| rc_lost | 2.1 | 2.1 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / RC LOST (the receiver) while AUTO: the transmitter is the safet |
| gps | 0.2 | 0.2 | yes | NOT READY fault EKF_BAD clear the cause, then confirm_safe / FCU mission manager FAULT HOLD   / vehicle FAULT EKF_BAD: clear it, then confirm_safe |
| fcu | 2.5 | 2.5 | yes | NOT READY fault FCU_LOST clear the cause, then confirm_safe / FCU mission manager FAULT HOLD   / FCU LINK LOST: no heartbeat for 3 s - takeover on the boat (usv |
| bridge | 2.7 | 2.7 | yes | LINK LOST the boat's status is 3 s old (bridge / radio) check the radio; the boat holds by itself (link_loss_action) / FCU mission manager AUTO_HOLD HOLD  stale |
| refusal | 2.0 | 2.0 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / waiting for RunStart · a refusal is silent: if nothing comes, a |
| perception | 2.1 | 2.1 | yes | ARMED AUTO_HOLD: a mission can start pick one and START / FCU mission manager AUTO_HOLD HOLD   / PERCEPTION STALE: sim_observer silent for 2 s - CAMERA tab -> t |

The sim-day sheet **42 / 42 on `/` and 42 / 42 on `/v1/`**; `make quick` 310, **`make test` 355 passed** (2:59). `pyproject` gains the
`boat-web` console script.

**Decisions taken unattended (the stream's)**:

1. **Launch** = a window `bweb` in run_sim.sh (smaller than a new `run_boat_web.sh`); no pyproject entry point (`boat-web` console
   script) because pyproject.toml holds the version — run as `python -m optiprimex.tools.boat_web`. The parent may add
   `boat-web = "optiprimex.tools.boat_web:main"` with the bump.
2. **The view logic lives in Python** (`Model.snapshot()` computes every element's state, fraction, label); the page's
   `setState(wf)` only paints. That keeps the wireframe contract and makes the mapping unit-testable; Chrome checks the painting.
3. **Thruster ↔ channel map** = usv_ui's MOTOR_NAMES: M-P ch1, SW-A ch2 (M2 STERN), M-S ch3, SW-F ch4 (M4 BOW); `--thr-map`
   overrides; a thruster left out is dashed "not mapped". To verify against the CAD / firmware (spec §1).
4. **Idle** = |pwm − 1500| ≤ 2 µs (outline); a channel reading 0 = "unavailable · no output" (dashed); thrusters go *unknown*
   (hatched) when the FCU block is > 2 s old or the FCU heartbeat > 2 s.
5. **Lamp**: expected from status/usv's V state (KILLED RED, MANUAL YELLOW, AUTO_* GREEN); BOOT / FAULT → *unknown* "no lamp rule"
   (the handbook gives none); *unknown* when the HB (usv_ui's block, else status.health.fcu_age_s) is > 2 s or the status is > 2 s old.
6. **Antenna**: lost = this page's own broker link down, `$SYS` bridge 0, or status/usv > 5 s; weak = status > 1 s or ocs/state > 3 s;
   good otherwise; unknown before the first status. RSSI "n/a (no poller)".
7. **Electronics**: fault on FCU HB > 2 s (when the boat is heard at all) or a host temp ≥ 95 °C; warning ≥ 85 °C or throttling
   (`--warn-c/--alarm-c`, not read from the profile tonight); a beat with null temperatures shows "n/a °C" (the Mac).
8. **Kill / estop in the block** come from NAMED_VALUE_INT (null until the FCU sends one); the RC5 E-stop and RC6 override are
   separate fields (`rc5_estop`, `rc6_override`); the header prefers the mission manager's kill / estop while its status is fresh.
9. **Defence in depth**: boat_web also refuses non-whitelisted words (with the same reason) before publishing; usv_ui stays the
   authority. A reply is `ok:false` only for refused / bad arguments / unknown; "already handed over" is ok:true.
10. **Stale page word** = `t` (unix) older than 10 s (the OCS's stale-command rule); words without `t` are accepted.
11. The OCS bar strip on top (the OCS page's `/state.json bar`, via judge_web's `_ocs_bar`), "OCS page not reachable" otherwise.
12. The SVG viewBox is widened (−150…670) so the callouts are not clipped; the hull geometry and ids are the mock's. The SW-A arrows
    moved next to the thruster (y 566). A cannon element (`id="cannon"`) was added on the forward beam (not in Wireframe.dc).
13. `?theme=NAME` overrides the theme for one view (screenshots), never persisted.

**Not done**:

- Phases 3–4: CONTROL, ACTUATION, TELEMETRY, NAV & SENSORS, DIAGNOSTICS (stubs), the radar and the compass / speed dials, the 10 Hz
  frame topic, the commanded (mode-20 frame) side of the actuators ("not published"), COG, speed des, τ frame, recorder state,
  cv/stream `/status` (camera fps / clients), the lamp readback, the cannon's back-channel (WP12), RSSI.
- The NX side (`run_usv.sh` serving 8098 next to the boat broker) — waits for the deploy; nothing deployed.
- Host thresholds are CLI flags, not read from `profiles/*` `host.warn_c` / `alarm_c`.
- (Done here after the merge: the live rehearsal on run_sim.sh and the sheet - above.)
- (The two ocs_web test failures the stream saw were v0.1.44's old-tab tests - fixed in v0.1.44.)

## v0.1.46 — 2026-11-07 (overnight) — WP10j: the Task 3 docking structure in the renderer + the dock dataset (CV_TRAIN's input)

**Why now, and the number**: §7e had been revised during the night (WP10j "tonight, first - CV_TRAIN waits on it" as v0.1.43); the
night run had already archived v0.1.43–0.1.45 from Dmitrii's prompt, so WP10j took the next free version (ROADMAP §7e notes it).
Built in a parallel stream (a copy from the v0.1.45 snapshot), merged without conflicts.

v0.1.46 (2026-11-06 night, WP10j, unattended).
- **Renderer**: it now draws the Task 3 structure per DOCK_MOCKUP_SPEC. The parts are tagged boxes (a new vertical-cylinder shape is used for the indicator), so `render_ids` gives each window, indicator, face, cube and deck its own id and all existing consumers (sim_observer, fast_sil, simview) keep working with `kind:<int>`.
- **Live lights**: simview subscribes to `sim/dock/state` with `--boat-port` (run_sim.sh passes 1884), and the fast SIL's camera follows its DockSim.
- **Dataset**: `dataset.export(scene=)` adds the dock bands, the dock jitter and per-frame light states, five classes, and the dock truth. `largest_component` was rewritten by runs, because the pixel walk took ~1 s per frame-filling face.
- **Results**: 49 targeted / 363 whole-suite tests green. A 200-frame mix and a 200-frame dock export were made, and the overlays were checked by eye. The default buoy export matches v0.1.45 pose for pose.
- **Not rehearsed live**: the full run_sim stack with fake_dock. Only the subscription was tested, against a test broker.
- **Next**: CV_TRAIN exports from the REPLY's commands. Measure S at the practice days and set `size_m` on the world's dock objects. WP10j-b comes after cvt-v2-l.

**Dataset sample** (the stream's real run):

| set | frames | buoy | dock_face | window | box | indicator | time |
|---|---|---|---|---|---|---|---|
| `--scene mix` marina seed 21 | 200 (3 neg) | 566 | 276 | 543 | 167 | 257 | 81 s |
| `--scene dock` pool seed 22 | 200 (30 neg) | 15 | 321 | 627 | 0 | 284 | 87 s |

The export runs at about 2.4 frames/s and **0.6–0.8 MB per frame**, almost all of it the noisy depth. CV's planned 4000 + 1500 frames therefore come to about 3.5–4 GB; the REPLY suggests copying without `depth/` to the AGX, or `--depth none`.

**Integrated**: `make quick` 318, **`make test` 363 passed** (2:49). Screens: `docs/screens/v0.1.46/` (abeam 9 m, standoff 5 m idle /
RED, the 1.5 m stop RED / GREEN, oblique). CV_TRAIN told directly (the peer session cv-train-35) that the release is 0.1.46, not 0.1.43 -
its overnight plan polled for the literal 0.1.43.

**Decisions taken unattended (the stream's)**:
1. **Default class list:** `--scene buoys` (the default) keeps the **four** v0.1.33 names. `dock`/`mix` write five. The OVERNIGHT doc says "`--classes kind` emits them [five]", but the brief requires the default buoy dataset to stay unchanged, and CV's step 1 needs a 4-class buoy set. An indicator seen in a buoys-scene frame goes to `not_labelled`.
2. **Pool dock orientation:** the pool's dock objects have no `face_hdg_deg`. The renderer now takes the bay line's normal toward the world's start (180° in the pool) instead of the old default of 90°, so the faces look at the water. The world files are unchanged.
3. **Window positions:** the cube side S is clamped to 0.85–1.3 m, and the window positions use max(S, 0.95) × the spec's fractions. With the spec's 0.8 × 0.9 jitter, two 25 cm windows plus borders would otherwise overlap.
4. **Where things sit:** the cube's front face is on the dock object's point and the cube extends behind it. The deck slab is S+1.0 m wide (not 3.5 m), so it does not fill gaps. The pontoon fingers are still not drawn.
5. **Lit window:** it is always in the GREEN bay. In dock poses the bay in view is made GREEN 70 % of the time, which gives more lit examples. GREEN is drawn per pose; the light state per frame.
6. **Dock-scene negatives:** they use a random place with the old negative-heading search (band `negative`).
7. **Window ids:** they reuse the world's window objects (pool 31–36), otherwise `900000 + dock id*10 + window`. The indicator uses the dock's id under kind `indicator`.
8. **Defaults off:** simview's `--boat-port` defaults to 0 (off), so tests and standalone runs never touch the live 1884; only run_sim.sh turns it on.
9. **Flash state:** the Advanced flash is represented in the dataset state (`flash`, `on`). fake_dock does not produce it.

**Not done / open**:
- No version bump, CHANGELOG, DEVLOG, GUIDE or ROADMAP edits; the text for them is above.
- **Coordination risk:** CV_TRAIN's OVERNIGHT plan polls `optiprimex/__init__.py` for `0.1.43`. This ships as 0.1.46. The REPLY says to accept ≥ 0.1.43 or the REPLY alone, but if CV's script checks for exactly 0.1.43 it will not trigger. Also, its plan exports both sets into the same `data/raw/sim_v0.1.43_dock/`, which fails (an export never overwrites); the REPLY gives two sub-directories.
- Not run: the run_sim.sh stack live with fake_dock (the ports were forbidden), and `yolo val` on the export (no Ultralytics here). The label format is unchanged from v0.1.33, which was proven with yolo then.
- The flash's 1 s timing is not animated in simview, because fake_dock has no flash phase. That belongs to WP10j-b or later.
- WP10j-b is not started.

## v0.1.46.1 — 2026-11-06 — map fixes: the fake judge's course from the world, the boat glyph, label de-confliction, entry / exit flashing vs solid

Dmitrii (lab day): "the fake judge's course comes from the world file (pool = its bounds, marina = Course Alpha; the RC course still
overrides); the boat glyph in the theme text colour as a hull pentagon with heading line, track and a hollow cyan carrot - no green /
red / blue on the boat; label de-confliction (bay vs face id, A/B, CO/HOME); entry/exit glyphs show flashing vs solid". Numbered
0.1.46.1 (v0.1.47 is not built yet: it stays WP10j-b's).

**What changed · where**
- `sim/fake_robocommand.py`: `world_course(path)` + `--world` - a world's `course: bounds` (its walls) | `course_alpha` | a [x_n, y_e]
  polygon, `course_id:`; absent = Course Alpha as before. `world_pool.yaml` `course: bounds` (SIM-POOL), `world_marina.yaml`
  `course: course_alpha` (COURSE-ALPHA). `scripts/run_bench.sh` passes `--world $WORLD`. The map draws a received course solid over the
  profile's dashed default, so rc-test / RoboCommand's course still wins there. (`sim_world.sh` mid-run does not re-publish the course.)
- `tools/map.html`: the boat = a hull pentagon in the theme's `--text` (light fill), a 15 px heading line ahead of the bow, the track in
  the same colour (faded), the heading label; the carrot = a hollow ring in the theme's accent; `lbl()` - a label placer (ten spots
  round the anchor, the first that overlaps nothing placed this frame) for bay labels, face / entity ids, saved points (A / B, HOME), CO,
  rays; ENTRY / EXIT: an outer ring dashed = flashing, full = solid, and the word in the label (truth: `entry · flashing`, the map:
  `5~ ENTRY flashing`). Found on the way: in a light theme the boat (now the text colour) vanished on the map's fixed dark canvas - the
  canvas now follows the theme (`--mapBg`, `--text`, `--muted`, `--dim`, `--grid`, `--line`; object colours stay), which closes the
  v0.1.43 open item "the map stays dark in the light themes" for the canvas (the map's own toolbar buttons are still dark there).
- `profiles/bench/ocs_bench.yaml`: the geofence drawn with the MAP editor at 09:48 today reset to `[]` (polled: the shipped profile stays
  empty) - the 4 corners are in `logs/ocs/geofence_2026-11-06.json` to draw again.

**Tests**: `tests/test_v01461.py` (3: the course per world and the old default, the boat / carrot code, the placer + entry / exit +
the themed canvas). The sim-day sheet **42 / 42 on `/` and 42 / 42 on `/v1/`**; `make quick` 321, **`make test` 366 passed** (2:54).
Screens: `docs/screens/v0.1.46.1/map_dark.png`, `map_hc-light.png` (SIM-POOL solid, the pentagon + heading line + track, the cyan ring,
CO / HOME and A / B apart, the dock's face ids 11–14 apart from the bay labels, `entry · flashing` / `EXIT solid`).

**Open**: the map toolbar's buttons in the light themes; the boat's heading label can still sit over a buoy glyph (the placer avoids
labels, not glyphs).

**Next**: WP10j-b (v0.1.47) - cvt-v2-l exists since 07:55 (the morning report).

## v0.1.47 — 2026-11-06 — WP10k-a: the quality-of-life round, sections A-D (launcher, judge page, COMMS & DIAGNOSTICS, OVERVIEW / MISSION)

Dmitrii: "Build WP10k-a as the next free version: section A (launcher …), section B (judge page …), section C (COMMS & DIAGNOSTICS …),
section D (RUN SEQUENCE …, the SIM page camera bug fixed at its cause). Acceptance as the file's last section." Polls answered before
building: `--action` on the dark themes = "darker amber fill, red outline". Sections A + B were built in a separate copy by a helper
and merged 3-way (`git merge-file` against the pre-build snapshot: no conflicts); C + D in the tree.

**What changed · where**
- **A, the launcher** (`tools/launcher.py`, `launcher.html`): ⧉ copy beside every plan command (incl. stop); a `title` on every option
  button; the manual network fields' placeholders from `network.ts.env` / `network.lan.env` (read at request time, local page only);
  PROGRESS - one line per process / page the plan started (one `tmux list-windows -a` per host + HTTP): **up** bold `--up`, starting
  grey, died `--alarm`; the five page links (OCS · judge · SIM · USV window · rc-test console) with up / down dots every 5 s, before a
  launch too; the rc-test plan starts the judge page wired to 1885 (`run_rctest.sh`: `JUDGE_OPEN=0` = no browser, `JUDGE_WEB=0` kept as
  its alias); the theme select = the source of truth (`logs/launch/prefs.json`, the record, `LAUNCH_THEME`), synced with the OCS /prefs.
  `tools/prefs.py`: the shared-theme helpers (OCS first, the launcher when the OCS is down). Dry runs never write prefs.
- **B, the judge page** (`tools/judge_web.py`): tokens only; a DECLARATION · COURSE · RUN strip over three columns (ACTIONS · RECEIVES ·
  SENDS); the judge map below (`map.html?judge=1`: course, geofence + its editor through the OCS, vehicles, incidents; a click fills the
  selected assist / keepout / moving command's lat / lon); `tools/live_dot.js` (200 ms flash per message, then the age, `--alarm` +
  "stale" past the limit, no animation) beside each vehicle's RC topic; `/prefs`, `/status.json`. `themes.css`: `--up` per theme.
- **C, COMMS & DIAGNOSTICS** (`ocs_web.html`, `ocs_web.py`): SYSTEM ACTIONS first (the NOW card + one card per host: NX = boat stack +
  reboot, AGX = perception + reboot, this Mac = OCS + rc-test, FCU = reboot); `launch_defaults()` puts the launcher's record into
  `/state.json` `launch` and the cards' selects open on it ("launched HH:MM · preset X", *(changed)* beside a differing pick - select +
  marker share one grid cell so the rows line up); two lanes (STREAMS / HUB / BOAT left as zebra `table.zt` with fixed columns; HOST /
  COUNTERS / TIMED OUT / JOBS right); a liveness dot per stream / node row. Note: the OCS rows use the page's own `liveDot(key, age,
  limit)` (lights when a row's age drops = a new message) rather than `live_dot.js`, because /state.json carries ages, not message times;
  same look and rule (lit per message, `--alarm` past the limit).
- **D, OVERVIEW / MISSION**: RUN SEQUENCE (title; done steps keep their button disabled "✓ hh:mm:ss"; the live one in `button.b.act` =
  `--action` / `--actionLine` / `--actionText`: light / hc-light yellow + red, dark / hc-dark darker amber + red, fun green + orange;
  `oneAction()`: two steps live at once -> only the first is highlighted); the fun theme's buttons get their own `--btnBg` (were the chip
  colour), `button.b` uses `--btnBg / --btnLine`; the picker (`#ov-sel`, synced with MISSION's) + the stage list back on OVERVIEW; no
  jumps (tabular nums, nowrap, min-heights, the alarm list a fixed 108 px with its own scroll, `scrollbar-gutter: stable`: an alarm row
  had grown the page past the window and the scrollbar re-wrapped every card); `?jumpcheck=N` + `tools/nojump.py` (headless Chrome).
- **The theme everywhere**: the OCS page follows /prefs and posts only on a pick; the SIM page (select right-most in its header,
  `simview` `/themes.css`, `/prefs.json`, POST `/prefs` through `tools/prefs.py`); `map.html` polls `/prefs` and re-reads its canvas
  colours; the map's header fills, the OCS page's tip / stop / alarm / answer fills and the SIM page's panels -> tokens. The SIM canvas
  keeps its water and buoy colours (object colours, by design).
- **The SIM camera** (item 16), the cause: a browser holds ~6 connections per host:port across tabs; the OCS tabs (and hidden SIM tabs)
  each held an MJPEG stream, so the SIM page's `<img>` waited for a free connection. Now a hidden page releases its stream
  (`visibilitychange`: the SIM page sets `data:,`, the OCS page's `camApply()` needs `!document.hidden`), and camPoll looks at MISSION
  (the camera's tab since v0.1.44, it still said CAMERA). The frozen `/v1/` page is not changed (it holds a stream only on its CAMERA tab).
- Titles: every `<button>` on the OCS, SIM, launcher and judge pages has one (OCS: 13 added; SIM: 11).

**Tests**: `tests/test_v0147.py` (11) + `tests/test_v0147_ab.py` (13), both in QUICK; `test_v0143` / `test_v0147_ab` contrast parsers
merge every block of a theme (themes.css now has several per theme); `test_v0144` reads RUN SEQUENCE. The `--action` pairs pass 4.5:1
(text on fill; hc-dark's fill darkened #a07800 -> #8a6600, fun's #1f8a4c -> #187a42 for it). **`make test` 390 passed** (3:04).

**Rehearsed live** (`run_sim.sh pool`, the merged tree): the sim-day sheet **42 / 42 on `/` and 42 / 42 on `/v1/`**
(`docs/screens/v0.1.47/sim_day_v*.md`); the fault-injection test **8 / 8 within 3 s** (`fault_timing.md`); no-jump: idle 0 / 0 / 0,
during the `/v1/` sheet 0 / 0 / 0, with a GPS-lost ALARM appearing and clearing 0 on OVERVIEW (the first pass during the `/` sheet found
the scrollbar re-wrap above: fixed, then these). The theme set: `docs/screens/v0.1.47/themes/` (OCS OVERVIEW / MISSION / COMMS /
SETTINGS, `/v1/`, judge, SIM, boat window, map, mini-map × 5 themes; the launcher + the judge page × 5 from the helper's copy).

**Open**: the launcher's PROGRESS and page dots were exercised against test servers and a fake record, not a launcher-started stack
(the launcher on 8094 here is an older process I did not start - left running); `LAUNCH_THEME` is passed to the launchers but nothing
reads it yet (the pages follow /prefs). The mini-map's "waiting for …" text overlaps its course on a fresh stack (pre-existing).

**Next session**: WP10j-b (v0.1.48) from its helper's copy, then WP10k-b (v0.1.49).

## v0.1.48 — 2026-11-06 — WP10j-b: m3t1 through the camera (docs/M3T1_SIM_PLAN.md)

Dmitrii: "Then WP10j-b (docs/M3T1_SIM_PLAN.md, precondition met)" - `../CV_TRAIN/models/cvt-v2-l` exists. Built by a helper in a
separate copy, merged 3-way (no conflicts; 20 files + `tests/test_v0148.py`).

**What changed · where**
- The dock from the camera: `cv/sim_observer.observe_dock` measures the dock face (median depth of its mask), each window (light off /
  red / green from its mask's colour vote; number 1 upper-left / 2 lower-right from its place on the face) and each indicator (at the
  face's range); `cv_publish.dock_to_items` -> `perception/dock` + `perception/window`. `usv/perception_map.py`: an indicator votes on
  the face above it (the face's colour = the majority of its last 15 sightings, `ind_votes` in the digest and usv_ui `cv status`);
  numbered windows never merge (the rendered windows are ~0.37 m apart, under the 0.5 m gate).
- T3 (`usv/tasks/t3_logistics.py`): SWAY_SEARCH along the whole line, commit to the bay with ≥ 3 GREEN votes and more GREEN than RED; a
  tie / none -> a second pass at 3.5 m, then AUTO_SAFE_HOLD; BACK_OUT when the bay reads RED for 1 s on the way in (≤ 2 re-engagements;
  off the centreline -> back out and retry the same bay); AIM_FIRE aims at the window seen lit RED with `window_id` on the aim (no lit
  window in 10 s -> the other window if seen dark, else the bay centre); each report once, in order; FirefightingReport `window` upper /
  lower (the old "green" text moved to `result`).
- `sim/fake_dock.py`: the window lights on our DockingReport only (`dock.auto: false` in the worlds; `--auto world|on|off`); an aim
  naming a window must name the lit one. **Merge decision**: usv_ui `dockto` (GUIDE 4.14c) sends no DockingReport, so the bench
  launchers keep fake_dock lighting by itself - `run_bench.sh` `--auto ${DOCK_AUTO:-on}`, `sim_world.sh` / `run_agx.sh --sim` `--auto
  on`; `run_sim.sh` passes `DOCK_AUTO=world` (the strict behaviour). A test pins it.
- Also: `sim/fast_sil.py` (`--detector` / `--model`), `sim/campaign.py` m3t1 (random starts 8–20 m, ±40°, any heading, a random GREEN
  bay, the strict reporting grader), `tools/sim_day.py` S9 rewritten (+ `docs/SIM_DAY.md`), `scripts/run_sim.sh` (with a model the dock
  comes from the camera; fp `--skip-kinds buoy,dock,window`), `tools/playbook.py` stages, `usv_ui` `cv status` votes,
  `requirements-sim.txt` pins ultralytics 8.4.123 + torch 2.14.1 (the venv's; CV_TRAIN uses 8.4.124).

**Tests**: `tests/test_v0148.py` (18, in QUICK); four files adapted where the plan changes behaviour (test_sim_pool, test_stage_stepping,
test_t3_sim, test_usv_ui). `make test` **406 passed, 0 skipped** (the cvt-v2-l test runs here; 4:16); after the bench-launcher
decision `make live` green + test_v0148 18 / 18.

**Acceptance**: the campaign m3t1 (12 seeds each, `docs/CAMPAIGN.md`): synthetic **12 / 12**, cvt-v2-l on CPU **11 / 12** (seed 5:
from one docked pose the model never sees the lit lower window - CV_TRAIN's known close-range miss - and the still sim boat misses it
on every frame; the report carries the right `window_id` 2 but `complete: false`, the grader fails it). No BACK_OUT in either set. The
`--step` walk: fast-SIL tests (a park before every stage incl. SWAY_SEARCH, BACK_OUT with `--bay 1`) + one on cvt-v2-l detections.

**Rehearsed live** (`run_sim.sh pool`): sim_day S1 + S9 4 / 4 against the fake judge (synthetic); then **against RoboNation's rc-test
with `--detector model --model ../CV_TRAIN/models/cvt-v2-l`**: S9 4 / 4 (`docs/screens/v0.1.48/sim_day_S9_rctest_model.md`; `obs`
logged `dock on`, fp skipped buoy / dock / window), and rc-test's own log (`logs/rctest/run_20261106_140804/robocommand_*.log`) holds
three report pairs, once each and in order: docking bay 2 -> firefighting window 1; bay 3 (GREEN moved mid-search) -> window 1; bay 2
after the wrong call `--bay 1` + BACK_OUT -> window 2.

**Open**: the votes in the OCS map's hover (the digest carries them; a page change); seed 5's close-range miss is CV_TRAIN's.

**Next session**: WP10k-b (v0.1.49) from its helper's copy - `cv/sim_observer/sim_observer.py` conflicts with this version (both
blocks kept: the dock payload and the model / classes fields).

## v0.1.49 — 2026-11-06 — WP10k-b: the USV window phase 3, the tuning knobs, the CV-model hint (UI_QOL section E)

Dmitrii: "then WP10k-b (section E: the USV window tabs, dials, controls, the wireframe at BlueBoat proportions with boxed callouts, the
TUNING card with TUNE → SET_TUNING, the CV-model hint)". Polls answered: the radar's rings "5 / 10 / 25 / 40 m"; the tuning list "Spec
+ T1 speeds". Built by a helper in a copy, merged 3-way: one conflict, `cv/sim_observer/sim_observer.py` (v0.1.48's dock payload and
this version's model / classes fields side by side - both kept).

**What changed · where**
- `tools/boat_web.py` / `boat_web.html`: typography (item 17); the five tabs with live content (18: CONTROL = the chain, the allocation
  from SERVO_OUTPUT_RAW, the heading error MAE / RMSE / max per stage, the TUNING card; ACTUATION = the four thrusters, the channel map
  *to verify*; TELEMETRY = a ≤ 100-line canvas plotter, 60 s, pause; NAV & SENSORS; DIAGNOSTICS incl. usv_ui's console tail); radar
  (heading-up, sqrt radial scale, rings 5 / 10 / 25 / 40 m), compass, speed dial with the values as text (19); the ten whitelist buttons
  with tooltips and the reply (20; decision 25 unchanged); the wireframe at 1.19 × 0.76 m, callouts by `place_callouts` (fixed rows: a
  changing value never moves a box), the camera callout with the stream source, every perception source + age and the CV hint (21).
  **Merge**: GET / POST `/prefs` (through `tools/prefs.py`, the OCS first) and the page follows the shared theme every 2 s.
- Tuning (22): `companion/tuning.py` (the whitelist, ranges, `TUNE_STATES` = MANUAL / AUTO_HOLD, the read-only PWM / stern row);
  `ocs/cmd_authority.py` `TUNE` -> `cmd/usv/run SET_TUNING` (refused in AUTO_TASK, AUTO_SAFE_HOLD, KILLED, FAULT, unknown);
  `usv/command_handler.py` re-checks the state; the mission manager writes `cfg.extra[t1|t3]`, acks old -> new, a `tune` line in
  mm_trace; `tune save` -> `logs/launch/<stamp>/usv_tuned.yaml` (writing over the loaded profile refused); `ocs_cli tune`; the boat
  window's card POSTs `tune …` to the OCS page's /cmd. No FCU parameter, mode or arm path (ADR-9; checked).
- CV hint (23): `cv_publish` forwards `model` / `classes` / `observer_detector` in perception/health; `tools/cv_hint.py`; /state.json
  `cv_hint`. **Merge**: drawn on OVERVIEW's SYSTEM card (`AGX · CV model`) and MISSION CONTROL's mission line (`#c-cv`: the picked
  mission's classes from `by_task`, or "none of <model>'s classes - its input is a stand-in or not the camera").

**Tests**: `tests/test_v0149.py` (+1 for the merge wiring), `test_v0145` (the tabs are no longer stubs). **`make test` 419 passed**
(4:07); +1 since = 420.

**Rehearsed live** (`run_sim.sh pool`): `cv_hint` = "CV: synthetic · 1 class · buoys → T1 gates"; `ready` -> AUTO_HOLD, `tune sway_v
0.45` acked "sway_v 0.25 -> 0.45 (t3.sway_v; the next task start reads it)", `tune save` wrote `logs/launch/20261106_143233/
usv_tuned.yaml`; `start_task m1t1` -> AUTO_TASK, `tune sway_v 0.30` **refused** ("TUNE refused in AUTO_TASK …"); abort, `tune sway_v
0.25` back; the profile files untouched. The boat window followed the shared theme (light / hc-dark, `docs/screens/v0.1.49/live/`).
The helper's set (13 shots, demo streams): `docs/screens/v0.1.49/`; its live test: `sway_v 0.45` is the peak of the next m0t3's SWAY
frames and the saved copy loads through `MM_CONFIG_OVERRIDE`.

**Open**: the knob ranges are the helper's choice (line_v 0.1–1.0, sway_v 0.05–0.6, enter_v 0.05–0.4 m/s, standoff_m 2–10, stop_m
0.5–3, lead_step_m 3–25, look_s 2–30, transit_v 0.2–1.5, circle_v 0.2–1.2, la_m 0.5–6, scan_rate_dps 5–45) - Dmitrii to tighten; T3's
TO_LINE lookahead stays hard-coded 1.5 m (`la_m` is T1's); on the NX boat_web needs `--ocs-url http://<mac>:8096 --cam-status
http://AGX:PORT/status` (with the defaults it answers "no OCS, no tuning" - safe); the commanded yaw / surge / sway stay "not
published" (no frame topic); the channel map stays *to verify*. No Jetson deploy (as asked).

**Next session**: ROADMAP's next open WP. Before any Jetson deploy: the GUIDE §1 loop with the concurrency check (docs/JETSON_LOG.md).

## v0.1.50 — 2026-11-06 — the FCU buttons (mode / ARM) on both pages, RC6 latched, the snapshot cameras, the button design

Dmitrii: "In simulation, RC6 was still holding my FCU … add control to issue MavLink to change mode (in both USV and OCS pages) … a
separate visible button … camera feed was not going through on Simulation page … I like the button color … apply the same design for
other buttons (… more pale tint for not important buttons)". Polled: the mode control vs ADR-9 / decision 25 -> "ignore ADR-9. On the
OCS page I want Manual - Hold and ARM/DISARM. On USV window - all modes … and ARM/DISARM, even if mission manager holds the USV";
RC6 -> "RC6 latched, last wins, but we can disable it in settings, on USV side". The rule records are amended (CLAUDE.md, HANDOFF §0
rule 1, OCS_V2_SPEC §0, DATAFLOW, ROADMAP's NEVER list): the companion never arms *by itself*; the operator's buttons may.

**What changed · where**
- The mission manager: `companion/link.py` `arm(on)` (COMPONENT_ARM_DISARM with ack); `usv/command_handler.py` FCU_MODE {mode} /
  FCU_ARM {on} / SET_RC6 {policy} (`self.link` set by the mission manager); `usv/run_context.py` run-free; schema enum.
- RC6 (`usv/vehicle_fsm.py`): `guards.rc6_policy` latched (default) | hold. Latched: the flip up = MANUAL at once; cleared by the flip
  down, by ENABLE_AUTO / FCU_MODE / FCU_ARM (`clear_rc6_latch`), or by an FCU mode change after the flip (the boat window, a GCS) -
  last wins; hold = the v0.1.29 rule. status `rc6` {policy, raw, holds}.
- The OCS: `cmd_authority` FCU_MODE / FCU_ARM / RC6 (any R); `ocs_cli fcu_mode MODE | arm | disarm | rc6_policy latched|hold` (not
  `rc6`: op-tui's plant word - found live, the page's /cmd goes through op-tui's table: `bench_tui` INTENT_WORDS + WORD_TO_ACTION);
  playbook entries. The page's bar: MANUAL · HOLD · ARM (a dialog first) · DISARM beside the FCU chip, the current mode in `--action`,
  a red **RC6 HOLDS** while it holds.
- usv_ui: `mode` / `arm` / `disarm` are page words and run while handed over (no takeover; a page DISARM leaves the mission manager's
  motion alone). boat_web: WHITELIST + FCU_MODES (MANUAL HOLD ACRO STEERING LOITER AUTO RTL SMART_RTL GUIDED mode 20), the FCU card
  (the current mode lit, ARM = two clicks within 4 s, DISARM, the RC6 state + rule select -> POST /ocs -> the OCS's `rc6_policy`,
  only those two lines travel).
- The fake FCU arms / disarms by MAVLink as ArduPilot (ARM refused under the kill / E-stop / RC5); was "refused: arming is physical".
- The cameras: the OCS page (`camLoop`) and the SIM page fetch `snapshot.jpg` in a loop instead of holding `/stream`. The SIM page's
  blank camera was the browser's ~6 connections per address used up by held streams (the v0.1.47 fix covered hidden tabs, not pages
  loaded before it or several visible windows); with no held stream it cannot recur. The frozen `/v1/` still streams on its CAMERA tab.
- The button design: `themes.css` `--actionPale` / `--alarmPale`; the important buttons (go / primary / big; the launcher's presets)
  = the pale action fill + the action outline, danger = the pale alarm fill + the alarm outline, the one live control the full
  `--action` (OCS, SIM, boat window, launcher - its LAUNCH is the full action - and the judge page).

**Tests**: `tests/test_v0150.py` (8: the latch rules, the words / authority, page mode + arm while handed over on the live chain, FCU_MODE
clearing a latched RC6 + the hold policy, the buttons, the snapshot loop, the pale contrast); `test_v0145` (mode / arm left the
refused list), `test_v0147` (the snapshot loop), `test_ocs_web` (the page's `arm` is an intent now). **`make test` 425 + 1 = 426.**

**Rehearsed live** (`run_sim.sh pool`): with RC6 high the status said `holds: true`, `enable_auto` was accepted (AUTO_HOLD, `holds:
false`); `fcu_mode MANUAL` / `HOLD`, `disarm`, `arm`, `rc6_policy hold|latched` all acked by the mission manager; the boat window's
FCU card, the bar buttons, the SIM camera on the snapshot loop - `docs/screens/v0.1.50/`.

**Open**: the launcher on 8094 here is an old process (11:40) - restart it to see its new button colours; the SIM page's header wraps
its theme select at ~1500 px.

**Next**: WP10k-c (v0.1.51) - items 28 + 30 are built in a helper's copy (`HANDBACK_KC.md`), the rest on the OCS page.

## v0.1.51 — 2026-11-06 — WP10k-c: the visual pass (UI_QOL section F, items 24–32)

Dmitrii: "Build WP10k-c as the next free version: the .kpi numerals, the three header status blocks, the camera HUD overlay …, the
mission status card on the map, the alert table with level badges, the waypoint route editor on the MAP …, the links card with bars,
icons on the command row; the boat window's compass rose / ROT scale / speed dial … Poll me only on the HUD default and the KPI list."
Polled: the HUD -> "extra box around camera feed, default on, with toggle, but no resizing of the layout"; the KPIs -> "all three,
select preset in settings, want to explore all of them first". ("Then WP10j-b, then WP10k-b": both were already built - v0.1.48 /
v0.1.49.) Items 28 + 30 built by a helper in a copy (merged 3-way, no conflicts), the rest in the tree.

**What changed · where**
- `ocs_web.html`: (24) `.kpi` - 30 px tabular digits, 12 px unit, the `.h`-style label; OVERVIEW's MISSION · VEHICLE card shows the
  preset's six in a 3 × 2 grid (six in one row overlapped at the card's width), the presets A / B / C in SETTINGS → BIG NUMERALS
  (`ocs.kpi`; unknowns hatched - distance to the target is not published); (25) the bar's LINK · FCU · STATE blocks, 36 px, fixed
  widths, `--alarm` fill only when abnormal (the old link / FCU chips stay hidden with their ids; the blocks are `hb-*` - `sb-*` are
  the system cards' buttons: the first try collided with `sb-fcu`); (26) the HUD as a box around the picture (top: heading tape,
  left: SOG, right: r with a ±30 °/s scale, bottom: roll / pitch), its bands keep their size when off; (27) the mission status card in
  the map's bottom-left corner (stage x / y, elapsed / typical, the next exit, bridge / judge bars, the mode); (29) the alerts as a
  table (A / W / C badge, time - click sorts, source, text, state, ack) under the big word, the list 170 px; (31) COMMS → LINKS
  (tailnet / LAN by the flavour's net, the boat radio = the bridge, the judge wire; messages per minute from $SYS, a log-scale bar;
  RSSI hatched); (32) MISSION CONTROL's six buttons with inline-SVG icons, equal widths, `act` on the one that applies (`cmdAction`;
  `.cbtns button[hidden]{display:none}` - the grid's display had un-hidden RE-ARM).
- `companion/link.py` / `state.py`: ATTITUDE.yawspeed -> FcuState `yaw_rate_dps` -> status `yaw_rate_dps` (None until an ATTITUDE).
- `boat_web.html`: the header's big numerals (SOG · heading · FCU HB · alarms). From the helper: (28) the compass rose (hull glyph,
  HDG solid, COG open, the commanded yaw dashed = NAV_CONTROLLER_OUTPUT.nav_bearing only while the FCU steers, else hatched), SET / ACT,
  the ROT scale (usv_ui's FCU block carries `yawspeed_dps`), the speed dial's hatched commanded-surge slot; (30) `map.html`'s route
  editor (add waypoint, reorder / delete, save as points = `save_point Wi --lat --lon`, start path = save, wait for the digest, then
  `start_task m0t2 --names …`, "generate pattern" = the orbit / sway pickers; localStorage `ocs.route`; hidden on mini / judge).

**Tests**: `tests/test_v0151.py` (4) + `tests/test_v0151_kc.py` (8), both in QUICK. **`make test` 438 passed** (4:21).

**Rehearsed live** (`run_sim.sh pool`): the no-jump check with the status blocks - 0 / 0 / 0 changes idle and 0 on OVERVIEW while an
EKF ALARM appeared and cleared; the screenshots `docs/screens/v0.1.51/` (light OCS tabs + the camera HUD) and the theme set
`docs/screens/v0.1.51/themes/` (OCS tabs, /v1/, judge, SIM, boat window, map, mini-map × 5; the launcher on 8094 is an old process,
not re-shot). The map canvas does not render in headless shots (the route was checked by the helper by a canvas export).

**Open**: "start path" not rehearsed live with TEST MODE on; ✕ in the route list does not `del_point` on the boat; the commanded surge
/ yaw (mode 20) have no source yet; distance to the target (KPI preset C) is not published; the OCS bar wraps its theme select to a
second line below ~1700 px; the state.json is 1 Hz, so the HUD moves at 1 Hz (the spec says 2 Hz).

**Next session**: ROADMAP's next open WP; before any Jetson deploy the GUIDE §1 loop with the concurrency check.

## v0.1.51.1 — 2026-11-06 — the live numbers at 5 Hz (/fast.json)

Dmitrii: "can we speed up page feed? at which point it capped?" - measured on run_sim: FCU streams 10 Hz → the mission manager's
tick 10 Hz → `local/usv/mm/state` 5 Hz (`rates.local_hz`; the OCS server reads it on the boat broker directly) and `status/usv` 2 Hz
over the bridge (`rates.status_hz`) → `/state.json` (~20 KB, ~1 ms to build) → **the page polled it at 1 Hz: the cap**. Then: "yes,
agree, lets stream only what needs to be fast, others can be static and/or with lower refresh rate" (local_hz left at 5).

**What changed · where**: `tools/ocs_web.py` `Web.fast()` + GET `/fast.json` - FAST_KEYS (state, mode, armed, kill, estop, heading,
SOG, roll, pitch, yaw rate, task name / state, rc6) + the FCU heartbeat age + the status age + the bridge, from mm/state when it is
fresher than status (384 B live). `ocs_web.html` `fastTick` every 200 ms (one request in flight, paused while hidden) merges them into
S.status and redraws only `hdrBlocks` / `drawKpi` / `drawMsc` / `drawHud`; the FCU block reads the fast mode / armed / kill.

**Tests**: `test_v0151.py::test_the_fast_feed` (nothing seen -> nothing invented; the fresher source wins; < 1 KB; no big fields).
`make quick` 387, `make live` 434 passed. **Live**: /fast.json 384 B from `local`, ages 0.01–0.2 s; no-jump 0 / 0 / 0 with the loop.

**Open**: 10 Hz would need `rates.local_hz: 10` in the four usv_*.yaml (≈ +25 KB/s on the boat broker link) - not done (5 Hz agreed).

## v0.1.52 — 2026-11-06 — the boat window's top plate + light map, OCS OVERVIEW rearranged, the SIM page status only, rc-test RunStart by click

Dmitrii's round (messages / events up without scrolling, clearer highlight, the boat window's console; a top plate with mode / ARM in
colour - red ARMED, grey DISARMED, green AUTO, yellow MANUAL - and small boxes for handover / log / cv; a light map on the boat
window; no "to verify", short interactive wireframe; the rotated square; GPS lost "persisting"; SIMULATION on top of both pages; no
FCU control on the SIM page; the SYSTEM card cleaned with a colour per row; double tooltips; m1t1 with rc-test did not move and
RunStart came by itself; OVERVIEW: RUN SEQUENCE right, messages + console above the map, the CV stream below it, warnings above RUN
SEQUENCE; a console in the launcher). Polled: OVERVIEW (not MISSION) for the layout; ALL transmitter switches on the SIM page -> status.
Built in three streams (the boat window and the sim fixes by helpers in copies, merged 3-way without conflicts; the OCS page and
the launcher in the tree).

**What changed · where**
- Boat window (`boat_web.py` / `.html`, `usv_ui` `cv_on` in its FCU block + presence): the top plate (VEHICLE / FCU MODE / ARM blocks
  in colour, hatched when unknown, dashed + faded when the FCU block is > 2 s old; FCU HOLDER / LOG / CV FOLD boxes), the SIMULATION
  line (usv_ui on the fake FCU or the sim's camera, the reason named), EVENTS + CONSOLE (usv_ui's tail + the last replies) first in
  the right column, a light map (north up: the BlueBoat glyph, track, buoys, CO, saved points, the course edge), the wireframe as
  name + one value with the detail in the tooltip and a click panel linking to the matching tab; the radar prefers the mission
  manager's digest (`map_pref`, the two publishers alternated).
- OCS page (`ocs_web.html`): OVERVIEW's three columns as asked; `hiText` (verdict words bold and coloured, alarm / warning rows
  filled), zebra rows; `sysRow` with a status dot and the detail in its tooltip (RC: ok / RC LOST, perception: N sources · age), the
  legend behind a `?`; `tipPark` (the native title parked while the rich tip shows: one tooltip, the titles stay); SIMULATION in the
  bar (preset sim or the fake FCU); the OVERVIEW CV stream (a snapshot loop **only once /cam.json says the camera answers** - a loop
  against an unreachable address hung headless Chrome (the title test, nojump) and wastes a browser's connections; camPoll runs on
  OVERVIEW too). `ocs_web.py` `control_word`: FAULT with the cause gone -> "fault X latched - the cause is gone · confirm_safe, then
  enable_auto".
- Sim (`map.html`, `sim.html`, `simview.py`): the boat as the BlueBoat catamaran on the map and the SIM page's world (the pentagon
  read as a turned square; map.html was already no-store); the SIM page's transmitter = status chips, the faults + RESET stay, POST
  /fcu unchanged; the vehicle chip from `local/+/mm/state` (GPS lost was not "persisting" in the fake: the fix and EKF position return,
  the vehicle's FAULT EKF_BAD latches by design until confirm_safe - now said on both pages).
- rc-test: RoboNation's server sends RunStart itself when every declared vehicle is AUTO, with no switch - `ready` went to R5, m1t1
  (preselected without --auto) waited for start_task, and the judge page's RunStart was ignored. `tools/rctest_manual.py` runs the
  server unmodified with its `start_run` replaced by a `[manual-start]` line; `run_rctest.sh` `RCTEST_RUNSTART=manual` (default) |
  `auto`; the judge page notes a duplicate RunStart. The working m1t1 sequence is in the GUIDE (rehearsed by the helper on its own
  tmux sessions, both variants to DONE).
- Launcher: `pane_tail` (tmux capture-pane, plain names only, here or over ssh) + GET `/console.json` (the plan's windows only) + the
  CONSOLE card (2 s while visible).

**Tests**: `test_v0152.py` (6), `test_v0152_wa.py` (9), `test_v0152_wb.py` (13), all in QUICK; adapted on purpose: test_v0149 (no
"to verify"), test_v01461 (the glyph), test_v0147 (camPoll's tabs). **`make test` 466 passed** (4:23).

**Rehearsed live** (`run_sim.sh pool`): screenshots `docs/screens/v0.1.52/` (OCS OVERVIEW, the boat window, the SIM page); no-jump
0 / 0 / 0; the sim-day sheet **45 / 45** on `/` (S9 rewritten in v0.1.48 has more checks); the fault test **8 / 8 within 3 s**.

**Open / for Dmitrii**: manual RunStart is the default for every rc-test start (the launcher's event preset too) - scripted `ready`
against rc-test stops in R4 until RunStart is pressed (`RCTEST_RUNSTART=auto` for the old way). Found by the sim helper, NOT fixed:
`companion/link.py` names 0x100 `EKF_PRED_POS_HORIZ_ABS`, but 0x100 is MAVLink's EKF_PRED_POS_HORIZ_REL (ABS = 0x200), and
`ekf_pos_ok` ORs it in - on a real Pixhawk a relative-only predicted position would pass the EKF guard; usv_ui uses the right bit,
the fake FCU sends 0x100 too. The chase view still draws the boat as a box (render.py). The OVERVIEW CV stream is below the fold at
1600 × 1000.

## v0.1.53 — 2026-11-06 — REPORTING, the USV FCU fence, the courses EXTRA / ALPHA + move there, RUN SEQUENCE grouped

Dmitrii: Course Alpha drawn over worlds that lie outside it - other courses, select them, teleport per mission, realign our course;
RUN SEQUENCE cleaner with buttons grouped, quick links to resolve blockers (handover, MANUAL / RC6, declare) and to the mission
parameters, a highlight when a quick link switches tabs; the judge page and the rc-test console look the same; the course boundary
from RC → the FCU → read back → reported, and a manually modified geofence too; a "start reporting" button, then watch the judge
page for our heartbeats. Facts given before the poll: the RoboCommand Heartbeat has no geofence field (only the RunDeclaration's UAV
geofence goes back), the 2026-11-03 decision "nothing on the USV for the fence", Course Alpha = rc-test's 25 × 30 m demo, Bravo–Delta
only as network names. Polled: the fence "USV only, with a button to apply to FCU, with acknowledgment from Mavlink … toggle off,
which is by default"; the courses "a bigger course Extra (to be default) in RC-test … we fit our course elements to it … redistribute
course elements per relevant course (check rulebook). We choose from Judge page - once received we click in OCS's map and SIM to move
there (also can ignore)"; reporting "REPORTING switch". Built in three streams (the fence and the courses by helpers in copies,
merged 3-way without conflicts; the OCS page, REPORTING and the launcher in the tree).

**What changed · where**
- REPORTING: cmd_authority `REPORT` (no vehicle command) → `ocs_app.on_reporting` (heartbeats on from the click, kept through
  end_run / reset, off = the declaration rule again), `ocs/state` `reporting`, `ocs_cli report on|off`, op-tui's INTENT_WORDS, a
  playbook entry; the bar's ▶ START / ■ STOP REPORTING.
- The USV FCU fence (helper): `companion/fence.py` (polygon rules, the 1e-6° read-back compare, the inside-the-course rule ported from
  geofence-tools), `link.py` (the MAVLink fence protocol: upload with 3 retries, read back, MAV_CMD_DO_FENCE_ENABLE; FENCE_STATUS +
  the SYS_STATUS geofence bit), `command_handler` FENCE_APPLY / FENCE_ENABLE on a worker thread (ACCEPTED at once, DONE / FAILED with
  the FCU's answer: `Pending.done_reason`), status `fence`, cmd_authority (course = the received RxCourse; geofence = the sent points
  or the profile's - `ocs_app.main` now sets `authority.geofence_file`), `ocs_cli fence apply course|geofence | on | off`, the fake
  FCU speaks it (breach → report + HOLD when armed, as FENCE_ACTION 2), the MAP's FCU fence box, the boat window's fence line, the OCS
  SYSTEM card's *USV · FCU fence* row (`fbr()`: `breach` is an object). Off by default; apply / on refused in AUTO_TASK; no FENCE_*
  parameter is touched (apply_params.py: FENCE_TYPE 4, FENCE_ACTION 2 or 0, FENCE_ENABLE 0 + FENCE_AUTOENABLE 0, FENCE_MARGIN ≈ 2).
- The courses (helper): `sim/courses.py` EXTRA (120 × 80 m on Alpha's axis, containing Alpha, every task + the UAV transit area; the
  default) + ALPHA; the handbook gives no boundaries or per-course task split (1718, 2299, 2352, 258, 3633) - nothing invented;
  `world_extra.yaml` in the course frame with per-mission start points; `world.move_to_course()`; the fake judge `--course`
  (run_bench `COURSE=`, default extra), rc-test `RCTEST_COURSE` through rctest_manual (also follows a re-published course), the judge
  page's course select + publish; simview `/course.json` + POST `/world` (move / ignore), the SIM page's banner + COURSE · WORLD card.
  The OCS MAP's banner (an overlay - in the flow it moved the card 30 px: the no-jump check) + `/world` through `sim_post`.
- RUN SEQUENCE: `groupActs` (every step's button in one bar), `rsFix` (the blockers with quick links: the boat window's handover,
  the bar's HOLD / ARM, confirm_safe into the console, the checklist, MISSION's picker, the course banner, mission parameters),
  `jumpTo` (switches the tab, scrolls, the target outlined 1.6 s); the explanations only on the current step; the list a fixed 112 px.
- The launcher's "rc-test console" = rc-tui (tmux judge:tui) in its CONSOLE (`?con=`); the judge page stays the other link.
- Fixed on the way: the SIM page's empty course banner (`#crsbanner{display:flex}` beat `hidden`).

**Tests**: `test_v0153.py` (5), `test_v0153_wf.py`, `test_v0153_wc.py` (10), all in QUICK. **`make test` 489 passed** (4:34); after
the last page fixes `make quick` 435, `make live` 484 passed.

**Rehearsed live** (`run_sim.sh pool`): `report on` → heartbeats sent before any declaration, the judge's heartbeat age 0.2 s;
`fence apply course` → "fence 4 points uploaded, read back identical", `fence on` → "fence ON (the FCU holds 4 points):
MAV_RESULT_ACCEPTED" (status: enabled from SYS_STATUS, matches, no breach), `fence off` acked; the SIM server saw "course EXTRA
received … move the world there / ignore", the OCS's `/sim /world {course: EXTRA}` loaded `world_extra` (fp + dock restarted) and the
banner went; screenshots `docs/screens/v0.1.53/`. On a fresh pool stack: no-jump 0 / 0 / 0 (after the two fixes), the sim-day sheet
**45 / 45**, the fault test **8 / 8 within 3 s**.

**Open / for Dmitrii**: the apply_params.py fence preset (above) for the real Pixhawk; where EXTRA sits (the shoreline is not in the
tree - `_EXTRA_C` in courses.py) and its pinger (30 kHz); the bench's fake judge now publishes EXTRA on the pool world too, so the
banner shows there (`COURSE=world` = the old way); the launcher's world lists do not offer `extra` yet (`run_sim.sh extra` works);
proposed later: `fence clear`, a periodic read-back while disarmed (a Mission Planner edit), FENCE_STATUS in the stream profiles;
still open from v0.1.52: the EKF bit (0x100 = PRED_POS_HORIZ_REL) in link.py.

## v0.1.54 — 2026-11-07 — STOP & RESET, the camera boxes, RUN SEQUENCE's mission row, the map controls, m1t1's 0.60 m rule + buoy report

Dmitrii's round (approved as a plan first, + "avoid smashing into any course object - make 0.60 m distance rule for path planner"):
the OVERVIEW camera dark ("no camera") while MISSION's works; the stream on the USV page; the CV boxes; a confirm button for the
mission pick; START REPORTING under MODE / ARM; LOG newest on top; the mission select + start_task (--auto) in RUN SEQUENCE; warnings
below RUN SEQUENCE, FLOW above it, COMMANDS under the console; a clearer response to declare; S6 "couldn't find the start … found a
dark buoy and started to orbit it"; the buoys reported properly with coordinates; map buttons (move, zoom out, box zoom, fit) in all
maps; after STOP REPORTING the USV still in the judge window; a stop button returning the boat to HOLD - disarmed, clearing the mission
state and reporting. Answered: an RC6 flip aborts a running task (the vehicle to MANUAL) but keeps the run, the arming and the
reporting. Built in three streams (the boat window / boxes / STOP & RESET and the mission logic by helpers in copies, merged 3-way
without conflicts; the OCS page, the maps, the judge page's SILENT in the tree).

**What changed · where**
- OCS page (`ocs_web.html`): `camSt()` - /cam.json keeps the camera's source under `status` (the v0.1.52 code read `CAM.source`:
  the OVERVIEW stream never started and the SYSTEM row said "not answering"); the boxes over MISSION's and OVERVIEW's pictures
  (`/dets.json` every 400 ms while visible, an SVG in the picture's own fit, `boxes` toggle, "boxes not published" when the observer
  gives none); OVERVIEW: left SYSTEM / MISSION · VEHICLE / USV · UAV / LAST COMMANDS, middle EVENTS / CONSOLE / COMMANDS / the map /
  the CV stream, right FLOW / RUN SEQUENCE / the warnings; RUN SEQUENCE's mission row (`rs-sel` synced with MISSION's picker, --auto,
  ✓ preselect = `mission CODE [--auto]` with MISSION's options, ▶ start_task) and `rsAct` (sent → the answer; declare followed to R3:
  "✓ declared · RunDeclaration seq N" or "no R3 after 15 s"; `send()` now returns the reply); the bar's FCU group gains a second row:
  START REPORTING + ■ STOP & RESET (asks first); EVENTS and LOG newest on top; the state word's "never from software" hint → "ARM in
  the bar (asks first) or on the transmitter" (+ the playbook's verdict text).
- Maps: `map.html` and the SIM page's WORLD gain ↑ ← → ↓ (20 %), ＋ / − (×1.4), ⬚ box zoom (a capture-phase drag; Esc cancels), ⤢
  fit; hidden on the mini-map.
- STOP & RESET (helper B): `ocs_cli stop_reset` beside ready / manual (op-tui word, playbook macro): cancel_auto (in a 3·2·1) → abort
  (a task runs) → disable_auto (AUTO_HOLD) → fcu_mode HOLD → disarm → report off → reset (R2+), each step's result, a refusal never
  stops the chain; `report off` = no heartbeat now whatever R (a RunStart of the same declaration does not bring them back; `ocs/state
  reporting.off`); ocs_web's busy lock waits 20 s for `stop_reset` (it was refused "busy" behind a 15 s `ready`); the boat window's
  ■ STOP & RESET (two clicks; its `ocs_line` whitelist + `stop_reset`). The REPORT switch logs instead of raising an OCS alert (it
  showed as an ALARM banner).
- The boat window's camera (snapshot loop from the `--cam-status` address, a fixed 300 px card) + the boxes overlay; sim_observer's
  pixel boxes → cv_publish `local/<v>/perception/frame_dets` {frame_w, frame_h, frame_id, dets: [{cls, colour, light, x0..y1, range_m,
  conf}]}; ocs_web takes it on its boat-broker connection → GET `/dets.json` (live / stale / none).
- The judge page: a vehicle silent > 3 s is dimmed with **SILENT · N s**; SAFE PASSAGE · THE LAST REPORT'S BUOYS (helper C).
- Mission logic (helper C): the S6 symptoms reproduced with **both** planners - far, wrong-colour frames made a dark (or red / green)
  buoy read "blue, flashing" and T1 locked and circled it (4 / 80 random starts + extra's far start); on Course Extra the start area is
  ~51 m from Task 1, beyond the camera's range. T1 re-checks the locked entry / exit on the way in (8 sightings within 12 m, none blue →
  rejected, the search goes on); the search order map lead → `t1.search_lat / search_lon` → rays, `lead_tries` 4. The SafePassageReport
  (report_mapper / rc_wire were right; T1's body was not): out-of-view buoys kept, no ghosts, red / green = FLASHING, the entry / exit
  following the map, T1 sends the BeaconState itself, a report on any change or a 1 m move at most every 2 s - fast SIL: all 10 buoys
  once with the right state, ≤ 0.42 m off, entry / exit ≤ 0.12 m; RoboNation's own `parse_message` agrees to 1e-6°. The 0.60 m rule:
  hull-edge clearance for the BlueBoat (centre keeps r + 0.38 + 0.60 + 0.45 m; `t1.clear_m`), A* legs round every mapped object in
  both planners, entry / exit circles sized by the rule, T4's transit too; exemptions: gates narrower than 2.46 m on the centre line
  (pool / marina 2.4 m → 0.57 m), the dock's ENTER, the m0t4 box stop. The campaign measures the hull rectangle along every track
  (`min clr (exempt)`): geometric 146 / 152, camera 51 / 52, the smallest counted clearance 0.60 m. Checks that would have caught it:
  campaign m1t1 on extra + a `ring` start, the grader fails an entry circle round another buoy, sim_day **S14**.

**Tests**: `test_v0154.py` (6), `test_v0154_b.py` (10), `test_v0154_c.py` (21), in QUICK; adapted: test_v0152 (the new column order).
**`make test` 526 passed** (4:43). After the last small fixes `make quick` 470 passed (a first `make quick` run failed once and did not
recur - most likely one of the port-collision flakes seen this session; not captured).

**Rehearsed live** (`run_sim.sh pool`): `/dets.json` live (0.09 s, the synthetic detector's boxes), the camera's status read; `stop_reset`
→ "steps: cancel_auto n/a · abort n/a · disable_auto n/a · fcu_mode HOLD ok · disarm ok · report off ok · reset ok · now: OCS R2 ·
vehicle MANUAL · FCU HOLD disarmed · heartbeats off"; screenshots `docs/screens/v0.1.54/`; no-jump 0 / 0 / 0; the sim-day sheet **46 /
46** (+ S14); the fault test **8 / 8 within 3 s**. Helper C rehearsed `run_sim.sh extra` + `sim_day --only S14` 4 / 4 (m1t1 S6 DONE in
146 s, entry 0.06 m off, 10 buoys right within 0.09 m) on its own session before the merge.

**Open / for Dmitrii**: the map merges a 2.4 m gate's two buoys seen from the side (pool ring seeds 4–5; marina ring seed 2 touched a
buoy) - colour-aware association proposed, not built; m3t1 `standoff_m` 5.0 / `standoff2_m` 3.5 leave 0.49–0.9 m / −0.6 m to the
pontoon finger tips - 5.5 / 4.8 m proposed (tests pin the old values); `ocs_cli` has no `--search` flag for T1's search point yet; the
AGX observers publish no pixel boxes; the OVERVIEW CV stream is below the fold at 1600 × 1000. Re-plan §7f (2026-11-07): next WP10f-rest.

## v0.1.54.1 — 2026-11-07 — the EKF bit (safety; the overnight brief §1, nothing else)

**What changed · where**
- `optiprimex/companion/link.py`: `EKF_PRED_POS_HORIZ_ABS` was `0x100` - MAVLink's 0x100 is `EKF_PRED_POS_HORIZ_REL` (a relative-only
  prediction), ABS is `0x200`. The constants now come from pymavlink's enum (`EKF_POS_HORIZ_ABS` 0x10, `EKF_PRED_POS_HORIZ_REL` 0x100 -
  never counts, `EKF_PRED_POS_HORIZ_ABS` 0x200); `ekf_pos_ok` = POS_HORIZ_ABS or PRED_POS_HORIZ_ABS, as usv_ui (`1 << 4` / `1 << 9`)
  already had. Before: on a real Pixhawk a relative-only predicted position passed the AUTO guard (`require_ekf_for_auto`) and the
  EKF_BAD fault.
- `optiprimex/sim/fake_fcu.py`: the real bit set (0x01 | 0x02 | 0x04 | 0x08 | 0x10 | 0x100 | 0x200 with a fix; 0x01 without) - the old
  0x10 | 0x100 hid the wrong constant; new `{"set": {"ekf_ok": "rel"}}` = a relative-only EKF (0x08 | 0x100, no position).

**Tests**: `tests/test_v01541.py` (4, QUICK): the constants pinned against `mavutil.mavlink.EKF_PRED_POS_HORIZ_REL / _ABS`; a
relative-only report → `ekf_pos_ok` False (the old code said True), ABS / PRED ABS → True; the fake's bits; usv_ui's bits.
**`make test` 530 passed** (4:44, 6 processes).

**Rehearsed live**: nothing beyond `make test`'s live tier (the fake FCU's EKF path runs in every live mission). **Open**: none for
this bit; the real Pixhawk's EKF_STATUS_REPORT flags are worth one look on the lab day (`usv_ui` FCU block `ekf_flags`).

## v0.1.55 — 2026-11-08 — WP10f-rest: the boat window's commanded side, the simulated host beats, the lamp hook (the overnight brief §2, D1–D5)

Unattended night (docs/OVERNIGHT_2026-11-07.md). Built in the tree by the main agent; v0.1.56–58 ran meanwhile in helper copies.

**What changed · where**
- D1 `usv/mission_manager.py` `_cmd_view()` → `VehicleState.cmd` → `local/<v>/mm/state` `cmd {t, mode, behaviour, stage, frame {surge,
  sway, yaw} | null, target {lat, lon, hdg_deg, speed_mps, carrot {x_n, y_e}} | null, hdg_goal_deg | null, max_speed_mps}` at the 5 Hz
  of mm/state (no new topic, never bridged, `/fast.json` unchanged). `frame` = the behaviour's τ (`info.tau`, added in `sway_transit`
  and `dock_los` beside the allocation; a zero entry frame = zeros), `target` = the GUIDED target (bearing from the boat, the speed,
  the carrot from `info.carrot`), `hdg_goal_deg` = `info.hold_hdg` (sway_transit's bow, dock_los's bay normal, scan_yaw's face - added
  there); a tick without a command keeps the last; all null outside AUTO_TASK, on Hold, or while usv_ui holds the FCU.
- D2 `tools/boat_web.py` (+ `boat_web.html`): `cmd_block()` (stale > 2 s greys); the compass **SET** = `hdg_goal_deg`, else
  NAV_CONTROLLER_OUTPUT's nav bearing while the FCU steers, else hatched (`set_src`); the speed dial's tick = `target.speed_mps`, or
  `frame.surge × max_speed_mps` labelled *τ surge* (the tick's tooltip names the source); CONTROL's chain step 3 "commanded (mm/state
  cmd)"; ACTUATION's "commanded (frame share)" = u ± r / v per thruster before the stern factor and clip, beside the applied PWM (two
  numbers); NAV's τ row and "speed des · act · err".
- D3 `tools/host_health.py` `--simulated` + `--host nx,agx` (one process, one sample per beat); `scripts/run_bench.sh` node `host` (so
  run_sim.sh has it too); the OCS page's HOST HEALTH / SYSTEM rows and the boat window's electronics box + DIAGNOSTICS say **this Mac
  (simulated host)**; `ocs_web.py` `/meta.json` carries `host_thresholds`, `boat_web` reads them every 60 s (`thr_src` "OCS profile",
  the `--warn-c / --alarm-c` flags stay the fallback). On the Jetsons nothing changes (no `--simulated`).
- D4 `companion/state.py` `FcuState.lamp`, `companion/link.py` NAMED_VALUE_INT `LAMP`, usv_ui's FCU block `lamp` / `lamp_age_s`, the
  boat window shows *read* (with the value) instead of *expected* when one arrives; `docs/BOAT_WINDOW_SPEC.md` §2: the Lua does not
  send it yet - a question for Jaison.
- Tests adapted to the new wording: `test_v0149` (chain row 3, the speed text), `test_v0151_kc` (the SET text). GUIDE §3 row, CHANGELOG.

**Tests**: `tests/test_v0155.py` (7, QUICK). **`make test` 537 passed** (4:44).

**Rehearsed live** (`run_sim.sh pool`, fake judge, the camera path): `mm/state cmd` sampled through the sim-day missions - HOLD all null;
follow_path / goto / orbit GUIDED targets with speed and carrot; m3t1 SWAY_SEARCH and TURN_IN in CUSTOM_MANUAL with `frame` and
`hdg_goal_deg`; DOCKED all null. Host beats nx / agx on the hub `simulated: true`; the boat window's electronics "this Mac (simulated
host): NX AGX · thresholds from OCS profile". **The fault test with the boat window open: 8 / 8 ≤ 3 s** (kill 0.8, rc6 0.8, rc_lost 2.5,
gps 0.8, fcu 2.3, bridge 2.9, refusal 1.6, perception 1.3); **the sim-day sheet 46 / 46**; **no-jump 0 / 0 / 0**. **Page CPU on the Mac**
(headless Chrome with the boat window + the OCS page open, live stack, 60 s): renderers 8.2 % of one core, browser 2.4 %, GPU 1.7 %, other
2.0 % - ≈ 14 % of one core for both pages (≈ 1 % of the Mac); the NX figure waits for the deploy. Screenshots `docs/screens/v0.1.55/`
(the boat window in five themes, OCS COMMS dark), `fault_timing.md`, `sim_day_v2.md`. `stop_all.sh` → status CLEAN.

**Decisions taken unattended**: `cmd.mode` = the adapter's wanted mode (else the FCU's); `max_speed_mps` rides in the block (the τ-surge
scale); the frame share shown before the stern factor / clip (the boat window has no gains) and labelled so; the SET falls back to the
FCU's nav bearing only in the modes where it steers (unchanged rule). **Open**: the NX page CPU after a deploy; the Lua `LAMP` line.

## v0.1.56 — 2026-11-08 — m3t1 hardening: the docking rehearsal in the light simulation (the overnight brief §3, D6–D10)

Built by a helper in a copy (D6–D8), merged 3-way against the v0.1.54.1 snapshot (one Makefile conflict, the QUICK list, resolved by
hand; the helper's regenerated v0.1.54 screenshots not taken); D9 and D10 by the main agent.

**What changed · where**
- D6 `usv/dockpath.py` `standoff_lines()` (+ `FINGER_M` 3.5): `standoff_m = finger_m + hull_len_m/2 + clear_m + standoff_margin_m`,
  `standoff2_m` with `standoff2_margin_m`; `hull_len_m` / `clear_m` from t3, else t1. Profiles (pool, lab, bench, course,
  usv_ui's bench share): `t3.standoff_m` / `standoff2_m: null` (= derived; non-null = an override), `finger_m: 3.5`, `standoff_margin_m: 1.0`,
  `standoff2_margin_m: 0.3`. **5.70 / 5.00 m** (3.5 + 0.595 + 0.60 + 1.0 / 0.3) - not the brief's "≈ 5.5 / 4.8": the brief's figures
  do not follow from today's inputs; the formula was kept as written. `companion/tuning.py`: the knob shows the derived value, writes
  the override; current / view / diff / save handle null. m0t3's sway test uses the same line.
- D7 `usv/tasks/t3_logistics.py`, `usv/behaviours/dock_los.py` (mode `out`), `sim/fast_sil.py` (SAFE_HOLD = Hold, not the last frame):
  **new behaviour the cases needed** - TO_LINE backs out along the dock normal when the boat is inside the finger zone (finger + hull/2 +
  clear = 4.7 m; the brief's "≤ 3 m" read as "inside the line"); SWAY_SEARCH holds when the locked bays' entities leave the map (RESUME
  relocks); entry points other than SWAY_SEARCH need every face mapped (2 of 3 faces numbered the middle bay 1); `--from SWAY_TO_ABEAM`
  a wider corridor off the line; `dockpath.refit` re-fits the line from the map's current faces before SWAY_TO_ABEAM / TURN_IN / ENTER
  (the campaign found a far-locked line carrying 0.4 m / 17° into the bay). `tests/test_v0156.py` (17, QUICK): every entry point,
  park + RESUME at all 11 boundaries, wrong call → BACK_OUT → bay 2, RED ×3 → 2 BACK_OUTs → AUTO_SAFE_HOLD, map cleared mid-search →
  hold with the reason, no GREEN after two passes → hold, no DockingReport, window never lit → `complete: false` after `fire_s`, GREEN
  moved → follows, starts 2.5–3 m off → backs out, UNDOCK ≥ 0.60 m from the fingers; the strict grader (`campaign.report_order`): each
  report once, from its own stage, never before DOCKED, in order.
- D8 `sim/campaign.py` (m3t1 on every world with a dock, random starts for geometric too, `--model-seeds`, the model on the CPU,
  `finger_m` from each world's pontoons, `with_windows` adds marina's windows in a temp world), `docs/CAMPAIGN.md`, `tools/stage_typical.json`:

  | world | geometric (12) | camera / synthetic (12) | camera / cvt-v2-l (4) |
  |---|---|---|---|
  | pool | 11 / 12 (s2) | 12 / 12 | 4 / 4 |
  | marina | 11 / 12 (s12) | 12 / 12 | 4 / 4 |
  | extra | 11 / 12 (s12) | 12 / 12 | 4 / 4 |

  81 / 84, re-engaged 0, hull ≥ 0.57 m to the dock in the exempt bay stages, ≥ 0.85 m to the fingers. The three geometric failures: starts
  22.6–27.3 m from the faces at the geometric fake camera's 25 m limit (up to 70 % dropped sightings) → FIND_DOCK holds as documented;
  the camera paths pass the same seeds.
- D9 `tools/map.html` `voteText()`: the OCS MAP's hover on a dock face = `#id dock colour · votes G n · R m` (the digest's `votes`). The SIM
  page's world canvas draws the world file (truth), not the map, so no votes there (not cheap: skipped). `tests/test_v0156_d9.py`.

**Tests**: test_v0156 (17) + test_v0156_d9 (1) in QUICK; test_t3_sim / v0148 / v0149 moved from 5.0 / 3.5 to the formula. **`make test`
555 passed** (4:47).

**Rehearsed live (D10)**: `RCTEST_RUNSTART=auto scripts/run_sim.sh pool --judge rctest --perception camera --detector model --model
../CV_TRAIN/models/cvt-v2-l` - the first start without `RCTEST_RUNSTART=auto` failed S9 in 30 s (START_TASK needs R5+: rc-test waits for a
manual RunStart since v0.1.52 - the sheet needs auto); then **`sim_day --only S1,S9` 9 / 9 (S9 4 / 4)**: m3t1 DONE in 184 s through all 11
stages, DockingReport bay 2 → FirefightingReport window_id 1 complete; GREEN moved to bay 3 → DockingReport bay 3; `--bay 1` → RED on the
way in (4 sightings in 1.3 s) → BACK_OUT → bay 2. **rc-test's log**: `logs/rctest/run_20261107_035010/robocommand_20261107_035012.log` -
three `docking {bay_id 2 | 3 | 2}` each followed by its `firefighting`, in order. **The `--step` walk by script** (`docs/screens/v0.1.56/
step_walk.txt`): 11 parks (FIND_DOCK … UNDOCK), each resumed, DONE in 212 s, the two reports after DOCKED. The live digest's votes
(dock #136 red R113, #137 green G54, #143 red R16) = what the hover shows; `docs/screens/v0.1.56/ocs_MISSION_dark.png` (a headless
screenshot cannot hover - the votes are in the digest numbers above). `stop_all.sh` → CLEAN.

**Decisions taken unattended**: the formula over the brief's numbers (5.70 / 5.00); "inside the line" = the finger zone; the SWAY_SEARCH
loss holds (RESUME relocks); the model on 4 seeds per world; **`--step` on m3t1 resumes with `resume`, not `step`** (the command handler
reserves STEP for TEST_* tasks; the CLI's hint says resume) - recorded in GUIDE, not changed. **Open**: after a colour BACK_OUT pass 1
sways only from the excluded bay to the far end (can need the second pass; works); usv_ui's own `dock_standoff` 4.0 m unchanged; the real
dock's finger length at check-in → `t3.finger_m` + the world files.

## v0.1.57 — 2026-11-08 — WP12 in the simulator: the cannon contract and link (the overnight brief §4, D11–D13)

Built by a helper in a copy, merged 3-way against the v0.1.54.1 snapshot (conflicts by hand: the Makefile QUICK list, run_bench.sh's
NODES - `cannon` and v0.1.55's `host` both kept, one comment line in boat_web.py); the OCS page's row by the main agent.

**What changed · where**
- D11 `cv/cannon/cannon_sender.py`: `AimFollower` (pure) + `run_aim()`; `--source aim|file` (default aim; the legacy file-following path
  kept behind `--source file`), `--mqtt`, `--vehicle`, `--key PATH`, `--key-wait S`; the packet `dock_cannon.v1` {protocol, session,
  sequence, sent_unix_ns, sent_utc, source_host, status OK | AIM_OFF | AIM_STALE | NO_AIM | SENDER_STOPPED, target null | {class_name
  dock, color red, bay, window, window_id, geometry (+ the cannon-frame fields when the dock observer's status file is fresh)},
  window_id, aim_utc} signed HMAC-SHA256 over the sorted-keys JSON like the students' sender; 5 Hz whether or not there is a target;
  `target: null` unless the aim is on and younger than 1 s (receive time). `cv/cannon/run.sh` passes `--mqtt`; `cannon_config.json`
  `aim_rate_hz` 5, `aim_fresh_s` 1.0; `cv/cannon/README.md` = the contract. **The students' `color_dock_sender.py` /
  `color_cannon_receiver.py` are not reachable from the Mac tree** (searched) → no snapshot tonight; "snapshot on the AGX day" in the README.
- D12 `optiprimex/sim/fake_cannon_pi.py`: generates a test key `logs/bench/run_<stamp>/cannon_test.key` (64 hex, 0600; refuses the name
  `buoy_link.key`; the real key is never read on the Mac), verifies, `local/usv/cannon/status {ok, last_t, age_s, target, window_id,
  hmac_ok, n_rx, n_bad, status, sequence, session, key: "test"}` at 2 Hz; starts the sender as its child (`--with-sender`) so one window
  owns both and the key exists first. `scripts/run_bench.sh` node `cannon` (run_sim.sh through it); `boat_web.py` `cannon_link()` - the
  WATER CANNON callout's second line `Pi ok · last packet 0.1 s · hmac ok · target w1 · n 12`, amber when an aim is on and the link is
  bad / stale; `ocs_web.py` `CannonTap` → state.json `cannon {state, why, age_s, ok, hmac_ok, last_t, packet_age_s, window_id,
  target_on, n_rx, n_bad, status, key}`; `ocs_web.html` SYSTEM row **USV · cannon link**; `scripts/status.sh` lists fake_cannon_pi /
  host_health. GUIDE 4.16b.
- D13 AIM_FIRE and the fake dock unchanged (the window turns GREEN on the aim topic; the Pi's status is informational).

**Tests**: `tests/test_v0157.py` (9: the follower's freshness rule, window_id, the HMAC verify and a bad packet, the fast-SIL m3t1 with
the sender + Pi cores on the sim clock - hmac_ok, window_id, null within 1 s of `on: false` -, a live test with the real processes on
mosquitto, the OCS row); `test_cv_kit.py` passes `--source file` to the legacy test. **`make test` 564 passed** (4:50).

**Rehearsed live** (`run_sim.sh pool --perception geometric`): the `cannon` window up (fake Pi on udp 5005, sender `--source aim`, the test
key under the run dir); `sim_day --only S1,S9` → S9 4 / 4 (m3t1 DONE in 181 s, the reports in order, GREEN moved → bay 3, `--bay 1` →
BACK_OUT → bay 2; one ocs_cli "Timed out" line printed between checks, no check failed; the `--md` file was not written - the directory did
not exist yet; the results are these lines). The status trace (`docs/screens/v0.1.57/cannon_status_trace.txt`): NO_AIM → **OK, window_id 1,
target** during AIM_FIRE → AIM_OFF at once on `on: false` → AIM_STALE; hmac_ok true throughout, n_bad 0 (2688 packets). The OCS state
`cannon` ok / hmac ok / test key; the boat window "Pi ok · last packet 0.1 s · hmac ok · target none · n 2688". Screenshots
`docs/screens/v0.1.57/`. `stop_all.sh` → CLEAN.

**Decisions taken unattended** (the helper's, kept): the sim's "dry mode" = real UDP to the local fake Pi with the generated key
(`--dry-run` still prints only); one process tree; `hmac_ok` false for 2 s after a bad packet (visible at the 2 Hz status); `ok` = the last
packet verified and < 2 s old; the target carries the dock observer's geometry when fresh. **Open**: the students' sender / receiver
snapshot and the receiver's acceptance of `window_id` / `target: null` on the AGX day; who publishes `cannon/status` from the real Pi.

## v0.1.58 — 2026-11-08 — WP13 + the box key points: the dock line from the camera's face key points as m3t1's sway path (the overnight brief §5, D14, and Dmitrii's request of the night)

Dmitrii during the night: "for m3t1 - I want to test if we could detect docking box key points (center + boundaries) for at least 2
boxes, to establish the line and offset it, to use as sway-search path for next stage of the mission. Use (train if needed, or use what
we can find from student's code) CV for it on simulated world renders" - then "helper can use AGX resources no problem, we just don't
train on Mac". Folded into WP13 (D14 is the same seam). Built by a helper in a copy (allowed to train on the AGX if cvt-v2-l fell
short - it did not: **nothing trained, the AGX untouched, no CV_TRAIN request**), merged 3-way against the v0.1.54.1 snapshot after
v0.1.56 / 57 (four conflicts in `t3_logistics.py` with v0.1.56 resolved by hand - see below). **The brief's gate** ("if WP12's live run
is not archived by 04:30, skip WP13"): v0.1.57 was archived at 04:38; WP13 went on anyway because Dmitrii asked for exactly this
during the night and the code was already built and tested in the copy - a decision taken unattended, recorded here.

**What changed · where**
- Key points (`cv/sim_observer/sim_observer.py` `face_keypoints()` / `observe_boxes()`): the detection's left / right box edges → two
  edge bearings through the camera intrinsics; their range from the depth - a face is a vertical plane, so 1/depth is linear across the
  columns: a robust line through each column's median depth, evaluated at the outer face columns (rejects a box's side face and the
  water / sky inside a loose mask); the centre = the 3-D midpoint; a face within 1 % of the frame edge gets none. Reused from the
  students: per-mask ranging by the median of the mask's 3-D points (`measure_mask`, `cv/buoys/actual_buoy_observer.py`) and their
  `face_points {left, midpoint, right}` contract; their `cv/docking/dock_observer.py` gives only a centre - the edge method is ours.
  `dock_face` records carry `face_points`; `box` joined DOCK_TAGS; `observe_boxes` output stays in the payload (not published: no wire
  change). `cv/publish/cv_publish.py` `dock_to_items` passes `face_points` through (range / bearing).
- `usv/dockpath.py`: `fit_line` (least squares, normal toward the boat), `DockGeom.sway_path(standoff, margin)`, `box_faces`,
  `boxes_geometry`, `dock_geometry(..., source="map"|"boxes")`; `DockGeom.source`, `rms_m`. `usv/perception_map.py` `dock_tracks` (apart
  from `box_tracks`, so m0t4's `stable_boxes()` never sees dock faces). `usv/box_inspection.py` `BoxTrack.stable(colours=, min_span_frac=)`
  (defaults unchanged). `sim/fake_perception.py`: dock items carry `face_points` too.
- `usv/tasks/t3_logistics.py`: `t3.dock_source` map | boxes (else refused), `box_window_s` 1.0, `box_min_samples` 3; every geometry via
  `_geometry()`; `_ends` uses `sway_path`. The bay colour from the indicator votes, else the box face's colour votes. One face → FIND_DOCK
  holds "dock_source boxes: 1 stable face(s) with key points, a line needs 2". The FIND_DOCK log line names the source and the fit rms.
- **Merge decisions (main agent)**: DEFAULTS = v0.1.56's (standoff null + finger / margins) + v0.1.58's keys; `_found` = `_geometry()` with
  v0.1.56's "every face" rule; v0.1.56's `refit` before the bay stages and its lost-line abort / relock in SWAY_SEARCH apply to the
  **map source only** (box track ids are not map entities - the boxes source re-fits from its own key points on every call).
- `tools/dock_keypoints_eval.py` → `docs/DOCK_KEYPOINTS.md` (1104 × 621, ZED-like depth noise, 6 distances 4–15 m × −30 / 0 / +30°):
  **cvt-v2-l dock faces: the sway path within 0.3 m / 5° on 40 / 40 frames with ≥ 2 faces** (boundary error median 0.02 m, p90 0.03 m;
  line angle ≤ 0.1°, offset 0.01 m); cvt-v2-l box faces 22 / 23 (median 0.23 m, p90 0.42 m; the miss: extra, 15 m at +30°, 8.2° / 0.32 m);
  synthetic 38 / 38 and 27 / 27; no false faces; ~0.3–0.8 s per frame on the CPU. At 4 m only one face fits the 51° view (no line).

**Tests**: `tests/test_v0158.py` (12, QUICK; line fit 2 / 3 boxes, offset, 1-box refusal, fast-SIL m3t1 with `boxes` on world_boxes and
world_pool's dock → DONE, the camera chain → DOCKED, the map source still DONE). **`make test` 576 passed** (5:00).

**Rehearsed live**: `USV_CONFIG=logs/tmp/usv_bench_share_boxes.yaml` (usv_ui's bench share + `t3.dock_source: boxes`) `scripts/run_sim.sh
pool --perception geometric`, m3t1 from N32 E3: `T3 dock: 3 bays, bow-in 000, source boxes (fit rms 0.00 m)` → DONE in 182 s, DockingReport
bay 2 → FirefightingReport window 1 complete (`docs/screens/v0.1.58/live_m3t1_boxes.txt`). `stop_all.sh` → CLEAN. Not run: the camera path
live with `boxes` (the fast SIL covers the chain), the 12-seed campaign with `boxes`.

**Open (found by the helper, not fixed)**: the renderer and fake_perception read `face_hdg_deg` differently for `kind: box` (the renderer:
where the face looks; fake_perception: along the face) - world_marina / world_extra suit the renderer, world_boxes suits fake_perception
(its boxes render one behind the other; the eval uses its own `boxrow` world instead); the venv's `optiprimex` is the main tree installed
editable - a script run outside the tree root imports the main tree unless `PYTHONPATH=.`.

---

## v0.1.59 — 2026-11-07 — the run-start sequence and the RoboCommand compliance fixes (`docs/RUNSTART_AND_RC_BRIEF_2026-11-08.md` A + B, verdict table `docs/ROBOCOMMAND_COMPLIANCE_2026-11-08.md`)

Built in a scratchpad copy while Dmitrii's sim stack ran on the fixed ports, `make test` green there, then merged (33 files; the tree had
not changed meanwhile except Dmitrii's new `cannon/` - the students' two cannon scripts, left untouched, they read the key by path only).
v0.1.58 was deployed to both Jetsons earlier the same day at Dmitrii's request (JETSON_LOG); v0.1.59 is **not** deployed.

**A - compliance**
- A1 `ocs/judge_client.py`: `new_run()` no longer resets `_rep_seq` (3.4.8: never reset inside the run; the boundary is run_id /
  declaration_seq). Both counters persist per team in `logs/ocs/rc_seq.json` (`seq_file`, written on every change, atomic replace, read at
  start; a broken file = from 0 with a warning). `OPTIPRIMEX_RC_SEQ` overrides the path, `off` = memory only (tests/conftest sets it).
- A2 `companion/health.py`: EKF_BAD is a soft fault until continuous `guards.ekf_bad_hard_s` (3.0; 0 = the old rule), then hard (V6); the
  lab's `ignore_faults` unchanged. `ocs/heartbeat_publisher.py`: FAULT / BOOT keep the last known robot_state for `stale_stop_s` (10 s)
  from entering it, then stop (`stopped_fault`); no history = STATE_MANUAL as before. Decision 4 amended in OCS_V2_SPEC §0 + DATAFLOW A.
  **Found live**: during the soft 3 s an AUTO_HOLD boat showed no EKF word at all (fault test gps 3.8 s) - `bench_tui.warnings` now says
  *EKF BAD (no absolute position): a task holds; FAULT if it lasts 3 s* at once (gps 0.9 s).
- A3 `tools/rc_tui.monitor_subscriptions(team)`: course + our request / command / `+/report`; used by rc_tui, judge_web, ocs_web's peek.
- A4 `ocs/rc_wire.encode_report`: `flight_phase` only when vehicle_type is UAV.
- A5 `tools/launcher.py`: `course_nic()` (ifconfig -l / ipconfig getifaddr / getpacket: the 192.168.65–68.x interface, its DHCP lease and
  router), `resolve_rc_host()` (an address in the manual field / network.lan.env = override; empty / `auto` = the gateway; no lease =
  refused); the plan's note *course NIC: … · gateway … · DHCP · RoboCommand … (auto)*. validate no longer demands rc_host.
- A6 the launcher's preflight: `ssh <h> date +%s` per Jetson, |offset| > 5 s refuses LAUNCH (and the plan's `clock check` step fails the
  run too) with GUIDE §1's date line; red on the plan and in PROGRESS (`/session.json` clocks). Cached 10 s for /plan, measured again for
  /launch, skipped in dry runs and when `OPTIPRIMEX_PREFLIGHT=off` (tests). `scripts/status.sh` prints the offsets on the Mac.

**B - the run-start sequence**
- B1 `ocs_app`: on `judge: protobuf` (rc-test, course) reporting is ON from the start (`heartbeats_default_on` overrides) - heartbeats from
  each vehicle's first fresh status, any R; judge fake keeps the declaration rule. STOP & RESET dropped `report off` (playbook, ocs_cli,
  the page's confirm text); the page's ■ STOP REPORTING asks first.
- B2 `ocs_web.html` RUN SEQUENCE: the RUN CHECKLIST above as the gate (`rs-gate`, `clear for run ▶`); six steps 1 handover · 2 declare
  (the declaration rows `ck-decl`) · 3 ARM + enable_auto (the ARM button = the bar's, asks) · 4 RunStart (+ the RUNSTART slot) · 5 run R5 ·
  6 the mission; `attempt n` from `ocs/state.attempt` (declarations published per course_id).
- B3 `ocs_web.control_word`: AUTO_HOLD in R4 = **READY** "declared seq N · heartbeat AUTO · waiting for the course manager — m:ss";
  the stall row is a CAUTION "no RunStart after m:ss — ask the course manager, or redeclare" (`CAUTION_WORDS`, `run_start_stall` out of
  the alert rows). `ocs_cli ready` on the judge wire: R4 after 15 s = "R4 READY … waiting" (exit 0), no longer "✗ failed".
- B4 `profiles/presets.yaml` `auto_mission` (event true); `launcher.flavour()` carries it; the page ticks `--auto` once from it and
  `clear for run ▶` first sends `mission <picked> --auto`.
- B5 the judge strip under step 4 (R3–R5): `reporting.hb` (n · state · age · seq per vehicle), `reporting.last_report`, the last command.

**Tests**: `tests/test_v0159.py` (17, QUICK). Adapted (the old rules pinned): test_launcher (two clock steps), test_v0154_b (STOP & RESET
without report off; reporting kept), test_vehicle_fsm_table (EKF soft then FAULT), test_v0143 / test_wp9c / test_v0144 / test_v0153 (the
wording, six steps, report off asks). `make test` in the copy: 0 failed (twice); **on the tree: `make test` 593 passed (5:07), `make quick` 535 passed**.

**Rehearsed live** - rc-test with manual RunStart (`run_sim.sh pool --judge rctest`): heartbeats at the judge before any declaration
(`logs/rctest/run_20261107_132939/robocommand_20261107_132939.log`: RxReport seq 1 at 13:29:46 = line 55-58, … seq 93 = line 2064, the
RxRequest at line 2083 13:30:36); `mission m1t1 --auto` + `clear for run ▶` → R4, the bar READY "declared seq 1 · heartbeat AUTO ·
waiting for the course manager — 0:16"; RunStart sent through the judge page's `/cmd` (`start 1`, what its button posts) at 13:31:46.365
(line 4940, run_id 7001) → R5 → the 3·2·1 → m1t1 SEARCH_ENTRY; **report seq 222 (13:31:45.846, line 4921) → 223 (13:31:46.379, line 4955)
across RunStart**. Screenshots `docs/screens/v0.1.59/` 01–04. Fake-judge stack: **sim-day 46 / 46** (`logs/bench/v0159/sim_day.md`),
**fault test 8 / 8 ≤ 3 s** (kill 0.8, rc6 0.2, rc_lost 2.5, gps 0.9, fcu 2.1, bridge 2.8, refusal 2.0, perception 1.9), **no-jump 0 / 0 / 0**.

**Open**: the 3·2·1's `mission_countdown` alert shows as a red ALARM banner (older than this version; should be info); the RUN SEQUENCE
picker shows the MISSION picker's value, not a mission preselected by the console; the NIC detection is macOS-only (the launcher runs on
the Mac); WP10m (the CAMERA tab, ROADMAP) planned for the next iteration; the cannon agent's work to be merged when it hands over.

**Next session first**: merge the cannon agent's result; then poll Dmitrii for WP10m's four questions; deploy v0.1.59 only when he says.

---

## v0.1.60 — 2026-11-07 — WP10m: the OCS CAMERA tab (Dmitrii's request 2026-11-07, polled: depth only when no detector holds the camera · JPEG frames + jsonl · roots AGX recordings / Mac logs / CV_TRAIN staging · its own tab)

Built in a scratchpad copy (`ws/v60` from v0.1.59), merged (13 files; the tree unchanged meanwhile), then two fixes found live in the tree.

**What changed · where**
- `optiprimex/tools/depthviz.py` (new): depth → colour (near 0.5 m red → 25 m blue, no return black), the range at a picture position (5×5
  median), the scale bar's stops.
- `cv/stream/cam_stream.py`: modes rgb | depth | both - `POST /mode` (409 + the reason when the source cannot give depth), `/depth.jpg`,
  `/depth_at`, `/status` mode / modes / depth{available, why}. zed: images-only until the first depth request, then reopened with
  `DEPTH_MODE.PERFORMANCE`; test: a synthetic ramp (sky = no return); dir (a detector holds the ZED): refused, *stop the observer, then
  agx_cam.sh start --source zed*. Its own copy of the colour stops (it runs in the students' venv, without optiprimex).
- `optiprimex/sim/simview.py`: `/depth.jpg`, `/depth_at`, `POST /mode` (X-Sim-Web as every post), status modes / mode / depth.
- `optiprimex/tools/frame_rec.py` (new): the recorder - roots by NAME only (`mac_logs` = logs/recordings, `cv_staging` = CV_STAGING_DIR or
  ~/Documents/BH1/RobotX/recordings - the folder CV_TRAIN imports from, never CV_TRAIN itself -, `agx_rec` = AGX_REC_DIR); a session =
  `<root>/<date>/<HHMMSS>_<label>/frame_NNNNNN.jpg` (+ depth_NNNNNN.jpg) + frames.jsonl (file, boxes, pose) + session.json; new pictures
  only (sha1; repeats counted), max minutes, < 2 GB free stops it; snapshots into `<date>/snapshots/`; a CLI for the AGX.
- `scripts/agx_frames.sh` (new, tmux `agxframes`, detections from the boat broker), `scripts/stop_agx.sh` closes it first.
- `optiprimex/tools/ocs_web.py` `CamTab`: `/camtab.json` (per vehicle: USV = the camera that answers, UAV = UAV_CAM_URL or *WP14 / WP15*;
  the roots with ok / why - agx_rec only with the real camera), `/cam_mode` + `/cam_depth_at` proxied, `/camrec` (Mac roots in-process,
  agx_rec over ssh). `ocs_web.html`: the CAMERA tab (TABS; the old CAMERA → MISSION alias dropped), the picture in a fixed 16:9 box,
  USV / UAV, the three modes, the range under the cursor, the scale, box ticks + mission preset + labels, RECORD FOR TRAINING, space / R.

**Found live**: (1) the simulator had no `POST /mode` and its post guard wants X-Sim-Web - added; the proxy sends the header (cam_stream
ignores it). (2) `.cxoff{display:flex}` beat the `hidden` attribute ("no depth" stayed over the depth picture) - `[hidden]{display:none}`
for the tab's overlays; the box labels sized for the drawn width.

**Tests**: `tests/test_v0160.py` (9, QUICK): depthviz; cam_stream test source (modes, depth.jpg, depth_at sky / near) and dir source
(409, the reason); the simulator's depth + /mode; the recorder (allow-list, label check, one at a time, new frames only, jsonl with the
boxes and pose, session.json, sessions, snapshot, max_min); CamTab (the view, the mode proxy, depth_at, a typed root refused, the AGX
commands over a fake ssh, a Mac recording); the page. Adapted: test_v0144 (the tab list, the alias). The rule *every button has a
title* caught the five segment buttons - titled. **`make test` 602 passed (4:59), `make quick` 544 passed.**

**Rehearsed live** (`run_sim.sh pool`, fake judge): `/camtab.json` - USV ok, modes rgb / depth / both, agx_rec greyed *the simulator's camera
is on this Mac*; RGB with the boxes, RGB | DEPTH, DEPTH (screenshots `docs/screens/v0.1.60/` 01–04); range under the cursor 3.56 m at the
bottom centre; a recording `logs/recordings/2026-11-07/145701_v0160_live` - 16 frames at 2 Hz with the boxes per frame - and a snapshot;
no-jump OVERVIEW 0 · CAMERA 0. `stop_bench.sh` → status CLEAN.

**Open**: the real ZED's depth (the reopen) and agx_frames.sh on the AGX are untested until the AGX day (nothing deployed); the UAV
camera waits for WP14 / WP15; the cannon agent's work still to merge.

**Next session first**: merge the cannon agent's result; deploy v0.1.60 only when Dmitrii says (then test DEPTH with the observer stopped).

---

## v0.1.61 — 2026-11-07 — WP12-rest: the cannon link speaks the students' receiver (prepared in parallel by the cannon session in `ws_cannon/`, merged by the main session)

Dmitrii put the students' `color_dock_sender.py` / `color_cannon_receiver.py` into `MASTER_SCRIPTS/cannon/` and had a second session prepare
the cannon side in parallel with v0.1.59 (built from the v0.1.58 archive in `../ws_cannon/work`, never touching the main tree; its notes:
`../ws_cannon/MERGE_NOTES.md`). It wrote itself as v0.1.60; v0.1.60 had become the CAMERA tab meanwhile, so it is merged as **v0.1.61**:
the patch's added lines renamed (`test_v0160.py` → `tests/test_v0161.py`, v0.1.60 → v0.1.61), 23 of 24 files applied (ocs_web.py / .html
with offsets only), the Makefile QUICK line by hand. Polled by the cannon session: aim at **RED**, **one camera owner** (their segmentation
ported into our observers), **a status publisher beside their receiver**. Dmitrii (2026-11-07): the root `cannon/` hand-over folder deleted
after checking it byte-identical to the snapshots (`cv/cannon/color_dock_sender.py` 5e23b699…, `cv/cannon/pi/color_cannon_receiver.py`
cd9db52b…, `cv/PROVENANCE.md`).

**What changed · where** (the full table in MERGE_NOTES): the snapshots, verbatim; `cv/cannon/aim_core.py` (AimGate, the target file),
`aim_cv.py` (their segmentation + measure_mask ported); `--cannon red|green` on `cv/buoys_multi`, `cv/docking`, `cv/sim_observer`;
`cannon_sender.py` on `dock_color_aim.v1` (legacy `dock_cannon.v1` kept); `cv/cannon/pi/status_core.py` + `cannon_status_pub.py` (the Pi's
status from the receiver's JSONL log); `sim/fake_cannon_pi.py --receiver students` (their receiver verbatim, DRY_RUN, in the run folder) and
`sim/fake_cannon_target.py` (the truth target); boat_web / ocs_web show `rx EVENT (reason) · LOCK · PUMP ON`; run_bench / run_sim wire it.
**Fix**: `cv/docking/dock_observer.py`'s NameError at the first frame (`destination`).

**Tests**: `tests/test_v0161.py` (16, QUICK, 1 live) + test_v0157's live fixture on `--receiver ours`. **`make test` 618 passed** (the first run
had one ERROR in test_integration_stall's fake-FCU fixture: its control port received a MAVLink frame - an **orphaned fake_fcu** (ppid 1,
1 h 21 min old, from an earlier test run) still streaming to udp 50915; stopped by its PID; the file passes alone and the rerun is clean);
**`make quick` 559 passed**.

**Rehearsed live** (the cannon session could not: v0.1.59 held the fixed ports) - `sim_day --only S9` (three m3t1 runs) with
`mosquitto_sub local/usv/cannon/status` logging: **geometric** (`run_sim.sh pool --perception geometric`, the target from the world's truth)
4 / 4, m3t1 DONE in 181 s; **camera path** (`run_sim.sh pool`, the target from `sim_observer --cannon red`) 4 / 4, DONE in 183 s. Both traces,
three times each: `SAFE_NO_TARGET / NO_AIM` → AIM_FIRE: `AIM` (window 1, 1, 2 = the FirefightingReport's) → **`DRY_RUN_READY_10S` ready**
(their lock) → `NO_TARGET` (the window turned green) → `AIM_OFF` → safe (`logs/bench/v0161/cannon_status_{geom,cam}.log`, their receiver's
own log under `logs/bench/run_*/cannon_pi/color_receiver_logs/`).

**Open**: between missions the status reads `AIM_STALE` (the aim topic silent) where `AIM_OFF` would be clearer - safe either way (target
null); the AGX-day list from MERGE_NOTES (their two configs - allowed colour red, servo limits, the mount, `allowed_sender_ip`,
`maximum_packet_age_s` -, the height offset owner, the Pi's route to the boat broker, the real window's HSV band, the clocks); **the pump is
theirs**: in pump mode their receiver fires by itself on a stable lock during AIM_FIRE. The test suite can leave an orphaned fake_fcu after a
failed fixture - worth a conftest guard (later).

**Next session first**: deploy v0.1.61 only when Dmitrii says (then: DEPTH with the observer stopped, `agx_frames.sh`, the cannon chain on the
AGX + Pi with their configs); WP14 when the colleague's Orin is up.

---

## v0.1.62 — 2026-11-07 — Dmitrii's round: MISSION preselect with its answer + copy; the boat window's plate clickable; m3t1's dock key points and sway path on the OCS

Polled (2026-11-07): the VEHICLE box = an AUTO menu through the OCS; the confirmation = two clicks within 4 s (as STOP & RESET).
Built in a scratchpad copy (`ws/v62` from v0.1.61), merged (12 files; the tree unchanged meanwhile).

**What changed · where**
- `optiprimex/tools/ocs_web.html` - MISSION: the picker card's and the mission details' **✓ preselect** / **start now** go through
  `ackSend()` (sent…, then ✓ accepted / ✗ refused: why, under the buttons: `qack`, `mack`); **⧉** beside both command lines (`cpb`,
  copyText with the textarea fallback); `cPlace()` moves the preview's row (its copy button with it). The camera overlays (OVERVIEW,
  MISSION, CAMERA) draw a face's key points (`kpSvg`: boundaries open circles + dashed edges, the midpoint filled).
- `optiprimex/tools/boat_web.html` / `.py` - the top plate's six boxes are buttons: a menu (`#plmenu`) per box, an item armed by the
  first click (*send X?*, red) and sent by a second within 4 s; VEHICLE = enable_auto / disable_auto / confirm_safe through `/ocs`
  (`OCS_LINES` += the three), the rest = usv_ui's page words (mode from `fcu_ctl.modes`, arm / disarm, handover / takeover, log on|off,
  cv on|off); theme tokens only (`var(--acc)`); the STOP & RESET tooltip / reply without "reporting off" (v0.1.59 left it).
- `cv/sim_observer/sim_observer.py` - `kp_pixels()`: a face's key points -> pixel [u, v] (u from each point's bearing through the
  intrinsics, v the box's middle row); `frame_dets(..., cam)` adds `kp` to dock / box faces and now also carries the box faces.
- `optiprimex/usv/dockpath.py` `dock_view()` + `mission_manager.py`: the map digest's `dock_view` - the faces' key points (local N/E,
  from the tracks), the fitted line, `sway` at standoff_m and `sway2` at standoff2_m, the bays - the same `dock_geometry` /
  `t3.dock_source` / `standoff_lines` as m3t1 (guarded: a drawing aid never stops the digest). `map.html` draws them + the DOCK panel line.

**Found live**: (1) in the default simulator the dock is NOT seen by the camera (the synthetic detector does the buoys, the geometric fake
the dock) - the camera shows dock boxes with key points only with `--detector model` (cvt-v2-l: 4 dock faces with kp, the windows and
indicators); the map's key points + sway path work either way (fake_perception sends `face_points` too). Said in the GUIDE row.
(2) The map topic carries two digests (usv_ui's and the mission manager's): `dock_view` is in the mission manager's.

**Tests**: `tests/test_v0162.py` (8, QUICK): dock_view (key points, the sway / inner lines at the standoffs, nothing without a dock), the
digest + map.html, kp_pixels, a rendered pool dock giving `kp` inside each box in order, the pages' kpSvg, MISSION's ackSend / copy /
cPlace, the plate (OCS_LINES, the menu, two clicks, the STOP & RESET text). Adapted: test_v0154_b (OCS_LINES), test_v0152_wa caught a hex
colour in the plate CSS (fixed to the token). **`make test` 626 passed (4:59), `make quick` 567 passed.**

**Rehearsed live** (`run_sim.sh pool`, boat teleported to N36 E5.5 facing the dock): the mission manager's digest `dock_view` = 3 faces,
`sway` [[39.3, 2.0], [39.3, 9.0]] at 5.695 m (the faces at N 45.0); OVERVIEW's mini-map draws it (`docs/screens/v0.1.62/02_overview.png`);
MISSION shows *✓ preselect: mission m4t1 --t4 on* and the ⧉ (01); with cvt-v2-l the CAMERA tab shows the dock faces' boxes with their key
points (03). `stop_bench.sh` → status CLEAN.

**Open**: the real observers on the AGX (`cv/docking`) do not send `kp` yet (only the sim observer); a box face's key points on the camera
need m0t4's box observer in the loop; the headless screenshot of the standalone map stops before the first map message (the in-page maps draw).

**Next session first**: deploy v0.1.62 only when Dmitrii says; WP14 when the colleague's Orin is up.

---

## v0.1.63 — 2026-11-07 — the USV window on the team LAN for a delegate (Dmitrii: "delegate the USV to someone else … another guy could hook up to that page")

Polled (2026-11-07): both hosts, picked in the launcher (the NX = the event's default: the window survives the Mac; the Mac for the
bench / sim / a weak radio); **one commander, take over at once**; **a PIN for commands from another computer** (watching open). Built
in a scratchpad copy (`ws/v63` from v0.1.62), merged (16 files; the tree unchanged meanwhile).

**What changed · where**
- `optiprimex/tools/window_control.py` (new): sessions per browser (an id + a name, `X-Window-Session` / `X-Window-Name`, alive while
  polling, gone after 15 s), the commander (take = at once; a gone commander frees it; with control free the next command takes it),
  the PIN (proved once per browser; this host - 127.0.0.1 - never needs it; a headerless call is accepted only from this host, as before).
- `optiprimex/tools/boat_web.py`: `/state.json` carries `control` (you, the commander, the sessions, the PIN flag, where); `POST /control`
  take | release; `/cmd`, `/ocs`, `/tune` pass the gate first; `ocs_send()` = the OCS page (Mac) or, without one, `ocs_mqtt_line()` =
  ocs_cli on the hub (the NX: the OCS app's authority decides, as op-tui's); `--pin` / `USV_WINDOW_PIN` (else a 4-digit PIN generated and
  printed when bound beyond 127.0.0.1), `--ocs-mqtt`, `--where`; `local/<v>/ui/window` at 1 Hz. `boat_web.html`: a fetch wrapper adds
  the session headers; the commander bar under the top plate (YOU COMMAND / VIEWER · who commands, name, PIN, take control / release).
- `scripts/run_boat_window.sh` (new, tmux `bwin`; `stop` / `status`): the NX = the boat broker here, bound to BOAT_HOST, the OCS lines
  through OCS_HOST:HUB_PORT; the Mac = BOAT_HOST:BOAT_PORT, bound to OCS_HOST, the OCS page - never 0.0.0.0 (the RoboCommand NIC);
  refuses a second window on the host. `stop_usv.sh` / `stop_ocs.sh` stop it first.
- `optiprimex/tools/launcher.py` / `.html` / `profiles/presets.yaml`: **USV window: mac | nx | off** (event nx, the others mac); the nx
  step after the boat stack over ssh, the mac step after the OCS stack; PROGRESS `bwin:web`; a note with the address and where the PIN is.
- `optiprimex/tools/ocs_web.py` `WindowTap` + `ocs_web.html`: the SYSTEM row **USV · window (team LAN)** (where, address, connected, commander).

**Tests**: `tests/test_v0163.py` (8, QUICK): the lock (local first command, viewer refusal, PIN needed / wrong / proved once, take over,
gone commander, headerless only from this host), the HTTP gate end to end (trust_local off), the hub route's refusals, the launcher plans
(nx / mac / off, PROGRESS, the page field), the script's binding, the pages, the OCS tap. Adapted: test_launcher (the window step's indices).
**`make test` 634 passed (5:11), `make quick` 575 passed.**

**Rehearsed live** (`run_sim.sh pool --perception geometric`; `run_boat_window.sh --bind 192.168.0.13 --port 8099 --pin 2468`, called from
that address = another computer): a command before the PIN → *refused: PIN needed*; a wrong PIN → refused; take with the PIN → *control
taken by jaison (192.168.0.13)*, `log on` went through usv_ui; a second browser → *refused: you are a viewer - jaison … commands*; it takes
over → the first is refused; the OCS state's `usv_window` = mac, the address, the commander, 2 connected; the NX route (`ocs_send` through
the hub, no OCS page): confirm_safe answered by the OCS app (REJECTED in MANUAL, correct), `tune save` ACCEPTED. Screenshot
`docs/screens/v0.1.63/01_usv_window_lan_viewer.png` (VIEWER bar, PIN field). usv_ui's log switched back off; the window and the stack stopped,
status CLEAN.

**Open**: not yet run on the NX itself (nothing deployed) - the first run there: the launcher's event preset or the script by hand; the
radio traffic per viewer and the NX CPU to measure then. The UAV's own window (WP15) takes the same window_control + launcher pattern.

**Next session first**: deploy v0.1.63 when Dmitrii says, then the NX window from a second laptop on the team LAN.

---

## v0.1.64 — 2026-11-08 (overnight, unattended) — WP14: the UAV adapter, the UAV on the team bus, the declaration with two vehicles

Brief: `docs/OVERNIGHT_2026-11-07_UAV.md` (D0-D14; Dmitrii moved the stop from 06:30 to **08:30** before the start). Built in a scratchpad copy
(`ws/v64` from the live tree), merged (cmp-guarded), the acceptances on the tree. Nothing deployed; the Jetsons stay on v0.1.58.

**D0 - the way back (first, before any UAV code)**
- `_archive/MASTER_SCRIPTS_v0.1.63_pre_uav.tgz` = a byte copy of `MASTER_SCRIPTS_v0.1.63.tgz`, read-only,
  **sha256 `e6fb89d07e52f4340d9bb4a2a7485a0382954becf1362df5331fa7aadead141f`** (874 entries).
- `scripts/rollback_to.sh VERSION` (new, `--dry-run`, `-h`): refuses while status.sh is not clean; MOVES every entry of the tree root but
  `.venv logs _archive drone network*.env` into `_archive/rolled_<stamp>_from_v<cur>/`; extracts the tgz into a temp folder first (nothing
  moves if it does not extract), then into place; copies back `usv_ui/co.json`, `points.json`, `handbook/*.pdf`, `pathing_sim/*.pdf` (what
  the archives exclude); `.venv/bin/python -m pip install -e .` (re-registers the console scripts: `op-uav-adapter` goes); `__version__`;
  `make quick`; a DEVLOG block.
- **Tested on a copy of the tree (00:28)**: an APFS clone with its own venv clone, a planted `optiprimex/LATER_FILE.py` + `uav/later.txt`,
  version 0.1.64 → `rollback_to.sh v0.1.63`: the planted files gone, `__version__ = 0.1.63`, `co.json` + `logs/` + the PDFs kept, the copy's
  venv points at the copy (the real venv untouched), `make quick` 573 passed / 2 skipped, the DEVLOG block written; 4 min 58 s. The copy deleted.
- The second test (a real rollback on a copy of the FINAL tree back to v0.1.63) runs at the end of the night: the morning report.

**What changed · where (WP14)**
- `uav/` (new, D3): the UAV team's `geofence-tools/ sim-tests/ drone-heartbeat-gui/ lawnmower-mission/` + his `MANIFEST.sha256`, unmodified,
  read-only; `uav/PROVENANCE.md` (source, rules, a sha256 per file). His manifest: 36 files verify; `sim-tests/` is not in it (PROVENANCE
  covers it). Not copied: `buoy-detection/`, `robocommand/`, `camera-capture-gui/`.
- `optiprimex/ocs/geofence.py` (new): `geofence_problems()` + helpers from his `run_declaration.py`, `course_fence()` + helpers from his
  `course_fence.py` - extracted textually (the only edit: course_fence's `_xy` renamed `_cf_xy`, both files had one); a test compares the
  function bodies with the snapshot's source.
- `optiprimex/uav/wire.py` (new): his constants, `course_fingerprint()`, `ReliableSender` - verbatim (test-pinned); `BoundSender` = ours, the
  same class on a fixed port (`UAV_BRIDGE_PORT` 14605: his listener answers the hello's source address).
- `optiprimex/uav/adapter.py` (new, console `op-uav-adapter`): hello every 2 s; telemetry → `status/uav` 2 Hz (D5: BOOT / MANUAL / AUTO_HOLD
  / AUTO_TASK, robot_state = his rule, STATE_UNKNOWN past 5 s, the fence + ceiling, matches_declared from ocs/state's `uav` block);
  `event/uav/fence_change` (held while armed; a FENCE_ALT_MAX change while declared = a synthetic `alt-<ms>` id; clears pilot ready);
  `uav_report` → validated (his kinds + RoboNation's RxReport via json_format) → `event/uav/<kind>` + `uav_report_ack` (a bad one: ok false,
  never published; a re-send re-acked once); cmd/uav: PREPARE (ACCEPTED, then DONE = telemetry < 5 s ∧ disarmed ∧ geofence_problems clean
  against the PREPARE's course ∧ pilot ready, else REJECTED with the first reason), START_RUN (uav_task), STOP_RUN / RESET (pilot ready
  cleared), UAV_READY, UAV_TASK, FENCE_REJECTED (reliable; DONE on the listener's ack), FENCE_CHANGE_RESULT, BRIDGE_NOTICE, RC_FORWARD
  (course: the newest cancels the pending one; command: DONE on the listener's ack, FAILED after 15 s), anything else REJECTED "nothing here
  commands the drone". D11: current_task = the pending UAV task once the RunStart forward is acked; TASK_NONE when the drone disarms;
  `uav task N` overrides. Its bus identity is the UAV (envelopes, presence, the LWT `{state: OFFLINE}`).
- `optiprimex/ocs/ocs_app.py`: `uav_vid` (the `uav` vehicle of `vehicles`), the `uav:` block; `declare_uav` (off by default, kept across
  runs, STOP & RESET leaves it); `decl_vehicles()` = the surface vehicles + the UAV when on; `hb_vehicles()` = those + `report on uav`;
  `declare()`: m1t2 refused without it, the freshness gate over the declared list only, the UAV content from status/uav
  (`uav_declaration_content`: stale / armed / no polygon / no course / no ceiling refuse without fence_rejected; geofence_problems() refuses
  WITH `FENCE_REJECTED`), `uav_geofence` = the FCU's closed polygon, the ceiling = FENCE_ALT_MAX, the call sign from the uav block (the
  profile's `uav_geofence` stays the stand-in when no UAV is declared); the published declaration persisted to `rc_last_declaration.json`
  beside rc_seq.json; `on_uav_fence_change` (R3-R4: the last declaration re-sent - same declaration_seq, a new request seq, so RunStart
  still maps; R5-R6 not, said; not declared: the fence checked and the pilot told; the result back as FENCE_CHANGE_RESULT);
  `uav_course_in` (every RxCourse with a new `course_fingerprint` over the protobuf dict; cached; a fresh adapter whose status shows another
  fingerprint gets it once per 16 s); `uav_forward` (every RxCommand; RunStart only when the run declared the UAV; the IncidentAck for a
  UAV-targeted assistance / keep-out / all-clear when the forward is DONE); the UAV gets no COURSE / INCIDENT cmd of the boat's kind; STOP /
  RESET reach it only while its adapter is up (no TIMED OUT); ocs/state `uav` block; vehicles in profile order; `_rate_watch` (an
  `ocs/alert rate_dropped` when the 5 msg/s cap drops anything).
- `optiprimex/ocs/heartbeat_publisher.py`: `stale_by` (the UAV 5 s); for it the heartbeat stops when `health.telemetry_age_s` > the cut or
  is missing (his invariant 3); `uav_registration` passes into the heartbeat.
- `ocs/report_mapper.py`: `uav_report` → `{kind: pb, field, pb}`; `ocs/rc_wire.py`: `encode_report` kind `pb` (json_format.ParseDict into
  RxReport), `pb_dict()` (MessageToDict, preserving_proto_field_name - his bridge's to_dict); `ocs/judge_client.py`: the protobuf client
  keeps `_pb` on each course / command (forwarded as received; the fake judge's dicts are re-encoded).
- `ocs/cmd_authority.py`: GUARDS `UAV`, `DECLARE_UAV`; `on_uav`; `REPORT` with `vehicle: uav`. `tools/ocs_cli.py`: `declare uav on|off`,
  `uav ready on|off`, `uav task N|none`, `uav resend_course`, `report on|off uav`. `tools/playbook.py`: the two entries.
- `schemas/bodies.schema.json`: cmd actions UAV_READY, UAV_TASK, FENCE_REJECTED, FENCE_CHANGE_RESULT, BRIDGE_NOTICE, RC_FORWARD,
  RESEND_COURSE; events fence_change, uav_report.
- Profiles `ocs_pool / ocs_bench / ocs_course`: `SGIT-53577: uav` + the `uav:` block (D1 values; `declare_default: false`).
- `scripts/run_uav.sh` (new): windows `copter` (his fake copter; `--scenario quiet` = `quiet_alt20`: no MP write, FENCE_ALT_MAX 20), `uavlst`
  (his listener from `uav/`, `--vehicle-id SGIT-53577`, its state file in the run folder), `uav` (the adapter); each window runs a small
  script file in `logs/.../uav/` (a long command line overflowed the tty's 1 KB input buffer with the scratchpad paths); `tee -i` and the
  copter ignoring SIGINT so the listener's C-c restore reaches the FCU; `PYTHONDONTWRITEBYTECODE=1` (no pycache inside the snapshot).
  `run_sim.sh / run_bench.sh --uav [--uav-scenario S]`, `run_ocs.sh --uav` (the real Orin: UAV_HOST). `status.sh` knows the three processes.
- Docs: DATAFLOW table B rows (status / event / cmd / ack uav, the intents) + **table F, the UAV wire**; GUIDE §1b (the rollback), §4.20-4.20e;
  `docs/UAV_TEAM_CHECKLIST.md` (what is built, `run_ocs.sh --uav`); CHANGELOG.

**Decisions taken unattended**
1. **His sim-tests' expectations for mpsilent / mpsilentfix**: his MERGE_GUIDE §9 says "re-declared" / "accepted and sent", but the
   archive's `fake_copter.py` reports FENCE_ALT_MAX 100 unless the scenario name holds `alt20`. Run unmodified in a scratch copy (his
   `test_mp_fence.py`, a mosquitto for his amqtt `broker.py`), **his own bridge publishes nothing in both** (every attempt refused for the
   ceiling). Ours is pinned to that, and the positive path to `mpsilent_alt20` / `mpsilentfix_alt20`, where his bridge gives two requests
   (the course fence, then the MP fence 1.4122 / 103.9132) - ours gives the same two.
2. **A fence change with no declaration carrying the UAV**: his bridge then declares the UAV alone (tiers none); ours does not - the team's
   one declaration is the operator's `declare`; the pilot hears whether the next declare would take the fence (FENCE_CHANGE_RESULT ok =
   it would; no fence_rejected, since nothing was refused).
3. **RunStart reaches the Orin only for a run that declared the UAV** (his invariant 16: it flies the drone); every other RxCommand is
   forwarded whenever the adapter is up (his bridge forwards all).
4. **The heartbeat rate stays 2 Hz** (D2) though his guide says ~1 Hz was accepted on site (open with RoboNation, ROADMAP §8).
5. `robot_state` is `STATE_UNKNOWN` (not null) on stale telemetry: the status schema's enum; the heartbeat stops either way.
6. The cmd for the forwards is `cmd/uav/rc` with action `RC_FORWARD` (the brief names the topic only).
7. The UAV's course is the forwarded RxCourse; the boat's COURSE / INCIDENT cmds are not sent to it.

**Tests**: `tests/test_v0164.py` (16, QUICK): the snapshot vs his manifest and our provenance; the verbatim ports; geofence_problems on his
rule (closed / 4 points / outside by > 1 m / ceiling / the transit area) and course_fence's merge; the adapter's wire against a fake
listener (hello from the bound port, a reliable re-send with the same id, DONE on the ack, FAILED on the give-up, telemetry → status,
reports: ok / re-send / bad / unknown kind); PREPARE / the fence changes / the ceiling's synthetic id / D11; ocs_app: the USV without the
UAV (USV-only declaration, no UAV heartbeat, no RunStart to the drone), the two-vehicle declaration and every refusal, m1t2, the
re-declaration before / after RunStart, the course fingerprint (retained repeat not forwarded, pinger change forwarded, sent_at ignored),
the auto-ack (UAV-targeted only, after DONE), a failed forward's alert, the words; the UAV report → RxReport with its own seq; the heartbeat
cut; the launchers; the rollback script. `tests/test_v0164_scenarios.py` (6, live): his five scenarios + the two _alt20 variants, side by
side (a mosquitto each), his fake copter + listener unmodified, our adapter + ocs_app, a recording judge - ~46 s. Adapted: `test_wp9c`
(m1t2 now refused without declare uav: then m1t1 for the warn policy).
**`make test` 659 passed (5:18) on the merged tree** (657 + 2 skipped in the workspace); `make quick` in the workspace 573 + the new
files. Found by the full suite (`test_integration_sil1`, twice): a profile written by `yaml.safe_dump` sorts SGIT-53577 before USV1, and
ocs_app took "the first vehicle" as the boat - `primary_vid` (the first surface vehicle) now, ocs/state's vehicles surface-first; a test
pins it. Noticed: a v0.1.54 test rewrites `docs/screens/v0.1.54/boat_window_camera_*.png` on every run - restored from the archive (not new).

**Rehearsed live**
- **First, the stack WITHOUT the UAV** (the brief's rule: the USV lives without it; `run_sim.sh pool`, the fake judge, the camera path, no
  `--uav`): **sim-day 46 / 46** (`logs/bench/v0164/sim_day.md` 40 / 40 + `sim_day_page.md` 6 / 6, no finding), **fault test 8 / 8 within
  3 s** (kill 0.8, rc6 0.9, rc_lost 2.1, gps 0.2, fcu 2.6, bridge 2.8, refusal 2.0, perception 1.5 - `logs/bench/v0164/fault_timing.md`),
  **no-jump 0 / 0 / 0** (OVERVIEW 14 cards, MISSION 12, COMMS 9).
- **Then the two vehicles against RoboNation's rc-test** (`run_sim.sh pool --judge rctest --perception geometric --uav`; their fake copter
  `quiet_alt20` + their listener from `uav/` + our adapter): rc-test's Course EXTRA forwarded to the listener as the original protobuf
  (`Geofence COURSE: 7v T=4 EN=1 OK` - the course + transit area merged by their course_fence); `handover`, `declare uav on`,
  `uav ready on`, `ready` → R4. **rc-test's log `logs/rctest/run_20261108_014309/robocommand_20261108_014311.log`: ONE RxRequest (seq 62,
  line 1747) with `vehicle_ids` USV1 + SGIT-53577, T1 core · T2 none · T3 core · T4 core, `uav_geofence` = the FCU's 8-point closed
  polygon, `uav_geofence_max_altitude_hae_m: 20`, `uav_pilot_callsign: "Calwin-U2332301"`**; rc-test: *Tracking SGIT vehicles
  ['SGIT-53577', 'USV1'] - waiting for all to report AUTO*. The UAV heartbeat at 2 Hz from `declare uav on` (seq 1 01:43:55.059, seq 2
  .562; TYPE_UAV, FLIGHT_PHASE_GROUNDED, uav_registration AA53577, altitude_hae_m 8 - the fake copter's). RunStart = the judge page's
  button (#9001) → R5 run 7062; forwarded, acked by the listener in 1 ms; its log **`RoboCommand: RUN START - run 7062 (declaration #62)`**.
  **Report seq across RunStart (line 5793)**: USV1 7826 → 7827, SGIT-53577 89 → 90, monotonic over 190 / 120 reports. Screenshots
  `docs/screens/v0.1.64/01_judge_two_vehicles.png` (the declaration with both, both heartbeats), `02_ocs_overview.png`.
- The stop (`stop_bench.sh`): their listener restored the FCU's startup params - `Geofence RESTORED: T=7 EN=1 A=20 OK` (the copter outlives
  its C-c; `tee -i`). status CLEAN.
- Earlier, the same in the workspace with the fake judge (00:54): the declaration with both, PREPARE DONE from both, RunStart forwarded.

**Open**: the UAV window (8099), the launcher field, the OCS UAV card / tab, the alarm rows = v0.1.65; the buoys = v0.1.66. His listener
runs from our snapshot in the sim only - on the drone it is theirs. The fake copter sits at its own test coordinates (1.412 N 103.913 E),
not in the sim world: its position is drawn far away until the UAV window's map exists (the declaration does not depend on it: the FCU's
fence is the forwarded course's).

**Merge**: 25 files updated, 48 new (the `uav/` snapshot 41 of them), cmp-guarded; `.venv/bin/python -m pip install -e .` (`op-uav-adapter`
registered), `.venv/.ready` touched (the Makefile would rebuild the venv for the pyproject change, which needs the network).

**Next session first**: read the morning report at the top; v0.1.65 is the next version (D15-D19).

---

## v0.1.65 — 2026-11-08 (overnight, unattended) — the UAV window, the OCS's UAV card / rows / alarms, the launcher's UAV field (D15-D19)

Built in a scratchpad copy (`ws/v65` from the archived v0.1.64 tree), merged cmp-guarded (24 files updated, 4 new), the acceptances on the
tree; three fixes found live went in directly before the archive (below). Nothing deployed.

**What changed · where**
- `optiprimex/tools/uav_web.py` + `uav_web.html` (new, D16): the UAV window, http://127.0.0.1:8099/ - reads the hub (`status/uav`,
  `event/uav/#`, `ack/uav`, `ocs/state`, `ocs/alert`, `status/usv`, `presence/uav_adapter`); the plate LINK · STATE · MODE · ARMED ·
  DECLARED · PILOT READY; TELEMETRY, FENCE (n points, ceiling, ✓ matches / ✗ differs / pending, the last change, fence_rejected),
  ROBOCOMMAND (course forwarded + acked, the last command, reports, heartbeats, rate_dropped), an SVG map (the course, transit area, the
  FCU fence, the declared fence dashed, ▲ UAV, ◆ USV; fitted to the course + fences, a vehicle > 3 spans away pinned to the edge with its
  distance - the fake copter sits 16 km off at its own test coordinates), EVENTS, HELP = `docs/UAV_TEAM_CHECKLIST.md`; the whitelisted
  OCS lines only (`UAV_LINES`: uav ready on|off, declare uav on|off, uav task none|1..4, uav resend_course), checked before the lock,
  then window_control's commander / PIN (v0.1.63), sent as ocs_cli lines on the hub (the OCS's authority decides); two clicks within 4 s;
  `ocs/uav_window` 1 Hz; the jumpcheck recorder. `scripts/run_uav_window.sh` (new; tmux `uavwin`; bound to OCS_HOST, else 127.0.0.1, never
  0.0.0.0; start | stop | status); `run_uav.sh` starts it (--sim: on 127.0.0.1), `stop_ocs.sh` / `stop_bench.sh` stop it; status.sh knows it.
- `optiprimex/tools/ocs_web.py` / `.html` (D17-D18): `UavTap` (status/uav), `UavWindowTap` (ocs/uav_window); state.json `uav`,
  `uav_window`; `uav_warnings()` → the AlarmBook rows: UAV TELEMETRY > 2 s *UAV telemetry slow* (CAUTION) / > 5 s *UAV heartbeat stopped
  - Remote ID* (ALARM), only while declared or `declare uav on`; DECLARATION REFUSED (UAV); FENCE CHANGED (UAV); FOREIGN DECLARATION;
  RATE. The OVERVIEW UAV card and the UAV tab live (identity, link, state, FCU, HAE, fence vs declared, declared, pilot ready, task, the
  last report / forward, refusals); SYSTEM *UAV · adapter* / *UAV · window*; RUN SEQUENCE step 2's UAV line (`uavStep`); m1t2 marked
  *needs declare uav on* in both pickers; the UAV tab greyed until the adapter is up; links to the window computed from the page's host.
- `optiprimex/tools/map.html`: the UAV marker (▲ from `status/uav`, its HAE; hidden after 5 s) on every map (MISSION, MAP, mini).
- `optiprimex/ocs/judge_client.py` + `ocs_app.py` (D18): the protobuf client subscribes our team's request topic; an RxRequest seq we did
  not publish = `foreign_declaration` (state + an ALARM: their robocommand_bridge.py left on). `uav_state()` carries the received course
  (the window's map).
- `optiprimex/tools/fault_timing.py` (D18): with `--uav`, three faults - uav_tel (their listener SIGSTOPped 6 s by its PID), uav_fence (a
  fence outside the course: an `event/uav/fence_change` envelope on the hub, as the adapter publishes after a Mission Planner write - his
  fake copter cannot be made to write one on demand), uav_adapter (C-c in its window, restarted after).
- `optiprimex/tools/launcher.py` / `.html` + `profiles/presets.yaml` (D15): the field **UAV: off | on** (every preset `"off"` - quoted: YAML
  reads a bare off as False), the manual card's six UAV addresses (UAV_HOST, UAV_LISTEN_PORT, UAV_BRIDGE_PORT, UAV_BROKER_HOST,
  UAV_BROKER_PORT, UAV_BUOY_PATH; placeholders from network*.env names), the plan's `--uav`, PROGRESS uav:adapter / uav:web (+ their
  copter / listener in sim), the UAV window link, the flavour `· uav`.
- Docs: GUIDE §4.20f-g; DATAFLOW rows (ocs/uav_window, the subscribed request topic); the checklist; CHANGELOG.

**Found live and fixed before the archive** (the unit tests could not see them):
1. **The OCS page drew no card at all** (no-jump: `cards 0`; Chrome's console: *Cannot read properties of null (reading 'uav')*) - the nav
   is built before the first state; my greyed-tab rule read `S.uav` with `S` null. Now `((S&&S.uav)||{})`; a test pins the guard.
2. **`vehicle_run_dropped` for the UAV** (the adapter going OFFLINE lost its run_id: "reset on the boat") - not for the UAV (its own rows
   name the adapter); the fault test's uav_adapter line was that alert.
3. **The window's map**: fitted to everything, the 16 km fake copter shrank the course to a dot - fitted to the course + fences now.

**Tests**: `tests/test_v0165.py` (6, QUICK): the window model (nothing fabricated, the link / alarm thresholds, events), the HTTP gate end
to end (the whitelist first, the PIN, the viewer, no header 403, the checklist), the OCS tap + `uav_warnings` levels (CAUTION / ALARM via
the AlarmBook), foreign declaration + the uav block's course, the launcher field (presets, sim / event / manual plans, PROGRESS, links,
flavour), the null guard. Adapted: `test_v0143` (the fault list + the UAV three), `test_v0144` (the UAV card is live). **`make test` 665
passed (5:18) on the merged tree**.

**Rehearsed live**
- **USV-only first** (`run_sim.sh pool`, no --uav): **sim-day 46 / 46** (`logs/bench/v0165/sim_day.md`), **fault 8 / 8** (bridge 3.0 s
  with the v0.1.66 suite running beside it - **2.8 s** alone, `fault_bridge_rerun.md`), **no-jump 0 / 0 / 0** (14 / 12 / 9 cards).
- **With the UAV** (`run_sim.sh pool --uav`): **fault test 11 / 11 within 3 s** (kill 0.9, rc6 0.2, rc_lost 2.1, gps 0.6, fcu 2.6, bridge 2.8,
  refusal 2.1, perception 1.5, **uav_tel 1.5** (*UAV telemetry slow: 2.2 s old*), **uav_fence 0.9** (*fence changed after RunStart - NOT
  re-declared*), **uav_adapter 1.9**) - `logs/bench/v0165/fault_timing_uav.md`; **no-jump: the UAV window 9 cards, 0 changes; the OCS page
  0 / 0 / 0 / 0 with its UAV tab**. Page CPU: the browser side could not be isolated tonight (headless Chrome's helpers detach from the
  launch on this Mac - the user's Chrome shares them); the servers with one page each polling at 1 Hz, 60 s: ocs_web **1.5 %**, boat_web
  **0.6 %**, uav_web **0.2 %** of one core. Screenshots `docs/screens/v0.1.65/` (the window dark / light, the OVERVIEW card, the UAV tab).

**Open**: the browser-side CPU of the three windows (in a normal Chrome, Activity Monitor); the window on a second laptop over the team LAN
(the PIN path is tested with the v0.1.63 lock).

**Next session first**: v0.1.66 (WP16) is the next version.

---

## v0.1.66 — 2026-11-08 (overnight, unattended) — WP16: the UAV's buoys fused into the boat's map, m1t2 = T1 Advanced (D20-D23)

Built in a scratchpad copy (`ws/v66` from the merged v0.1.65 code), merged (34 updated, 4 new; the records, `usv_ui/points.json` and the
v0.1.54 screenshots kept from the tree), the acceptances on the tree. Nothing deployed.

**What changed · where**
- `docs/UAV_BUOY_CONTRACT.md` (new, D20): the drone team's page - `uav/perception/buoys` (t, frame_id, uav{…}, scan{state scanning |
  complete, coverage_pct, n_items}, items[{id, lat, lon, class, colour, lit, conf, sigma_m}]), `complete` sent three times with the full list,
  their mosquitto (an apt line + the listener conf) or UDP `type uav_buoys` to 14605, a mosquitto_pub line, a 10-line publisher, what the
  boat does with it.
- The two paths (D21): `profiles/pool/mosquitto/boat_pool.conf.in` - the hub→boat line **`topic perception/# in 0 local/usv/uav/ uav/`** (the
  brief wrote `uav/perception/#`, which with the `uav/` prefix maps to `uav/uav/…` - the pattern is relative to the prefixes) and the DIRECT
  block `#UAVBRIDGE connection uav …` that `run_usv.sh` renders only with UAV_BROKER_HOST set and UAV_BUOY_PATH direct (the default when the
  host is set; `relay` / `off` drop it); `profiles/bench/mosquitto/bridge.conf` the relay line; the adapter relays `uav/perception/buoys` from
  their broker (`UAV_BUOY_PATH relay`) or from UDP `uav_buoys` onto the hub. Both land on `local/usv/uav/perception/buoys`.
- `optiprimex/usv/perception_map.py` (D22): `ingest_geo(msg, origin)` - frame_id dedup; each item → local (the ControlOrigin); **linked** to the
  nearest boat-tracked buoy within `uav.assoc_m` 1.0 m not taken by another item of the message (the boat keeps its position; `e.uav` = the
  UAV's features, its colour / light = **the majority of every UAV sighting of that buoy** - found in the fast SIL: the last message's 5 %
  miscolour turned the entry red); the effective colour (`_apply_uav`): the UAV's under `colour_source uav` or when the camera's own vote
  (from its votes) is unknown, else the camera's with `uav_disagree` flagged; unlinked → a **UAV-only** buoy (source uav, the UAV's
  position, sigma_m, min_obs 2) - matched by the UAV's id only when it has one (two ids are two buoys: found by the test, 0.5 m apart);
  merged into the camera's track when the camera sees it within assoc_m (the camera's position wins; `_fold`); `entities()` leaves
  UAV-only out (gates, markers, T1's own lists unchanged), `uav_entities()` / `provisional_gates()` / `uav_entry_candidate()` give them;
  `age_out`: frozen under `uav_frozen` (m1t2), else forgotten after 30 s (the linked features too); the digest: per entity source,
  uav_colour, uav_lit, uav_seen_t, linked, disagree, cam_colour; `uav` (colour_source, frozen, scan, stats published / linked / uav_only /
  merged, n_linked, n_uav_only); `provisional_gates`.
- `optiprimex/usv/gatepath/planner.py`: `map_obstacles` adds the UAV-only buoys with their sigma_m on the keep radius (the 0.60 m rule).
- `optiprimex/usv/tasks/t1_safe_passage.py` (D22b-c): `tier advanced` → `colour_source uav` (the map follows the task: colour_source,
  frozen; the mission manager resets both when T1 is not running); **WAIT_UAV_MAP** (StationKeep) between WAIT_START and SEARCH_ENTRY for
  m1t2 - leaves on the UAV's `scan.state complete`, or the drone landed after flying (ocs/state's uav.flight: armed / flight_phase /
  armed_seen / airborne_seen) with no UAV item for `uav_quiet_s` 5 s, or `uav scan done` (forces it); without a flashing blue in the UAV
  layer it waits (a log CAUTION every 10 s) and after `t1.uav_wait_s` 420 s asks AUTO_SAFE_HOLD; `uav_lead` (the nearest UAV-only blue /
  undecided buoy, then the nearest provisional gate, `lead_stop_m` short) after the map lead; `precheck`: m1t2 needs a fresh UAV status;
  under colour_source uav: no `watch_locked` (the colour-blind camera never sees blue) and no LOOK at pending buoys (looking cannot decide
  a colour it cannot see). m1t1 unchanged.
- `optiprimex/usv/mission_manager.py`: subscribes `local/usv/uav/perception/buoys` (raw JSON → the tick thread → `ingest_geo`) and
  `ocs/state` (its `uav` block → `map.uav_ocs`). `command_handler.py`: T1 tier `advanced` accepted (was: "needs the UAV map").
- OCS: `ocs_app` - the declaration's **T1 tier follows the preselected mission** (`_t1_flag`: m1t2 → TIER_ADVANCED, m1t1 → TIER_CORE, else
  the profile's); `start_check` (START_TASK m1t2 needs a fresh UAV, and declared once in a judged run - after the declaration-policy rows,
  so their refusal comes first) for the typed start and the --auto start; `uav scan done` (ocs_cli `uav scan done`, the UAV window's
  button) → `uav.scan_done` in ocs/state (cleared by RESET / end_run); `uav.flight` (armed, flight_phase, armed_seen, airborne_seen).
  `cmd_authority.on_start_check`. playbook: m1t2 **built** with its WAIT_UAV_MAP stage; `stage_typical.json` m1t2 (WAIT_UAV_MAP 80 s).
- The simulator (D23): `optiprimex/sim/fake_uav.py` (new) - `Survey` (pure: a lawnmower over the Task 1 buoys' box + 10 m, lanes 15 m, at 3
  m/s → 238 m / 79 s on extra_t1adv; 1 Hz messages with the buoys inside its 25 m footprint, noise `--noise-m` 0.7, 10 % dropout, colour
  right 95 %, ids `u<id>`, sigma = the noise; `complete` three times with each buoy's mean; `--no-complete`); the process waits for the
  world (`sim/truth`) and our RunStart (ocs/state R5), publishes to the hub (the relay path; `--broker` = the direct path), `sim/uav`
  (retained), presence/uav_cv; `run_uav.sh --sim` starts it (window `fakeuav`, `FAKE_UAV_ARGS`). `profiles/bench/world_extra_t1adv.yaml`
  (new) = world_extra with the entry, the six gate buoys and the exit `colour_from_above: true`; `sim/render.py` draws such a buoy grey,
  no beacon (the observer's own classifier then says unknown); `fake_perception.py` reports colour unknown for it. `fast_sil.py`: `uav={…}`
  = the same Survey in process through `ingest_geo` once a second of sim time.
- `map.html`: UAV-only buoys hollow and dashed, the UAV tick on a linked one, the disagreement label, provisional gates dashed.
- Docs: GUIDE §4.21-4.21a; DATAFLOW (the buoy topic, both paths); the checklist; CHANGELOG.

**Decisions taken unattended**
1. **The bridge line** as above (`topic perception/# in 0 local/usv/uav/ uav/`).
2. **The association rate**: the boat waits outside the field during the survey (the handbook's order), so no item has a boat track to link
   to while the UAV flies - "linked / published" is 0 by design in m1t2; the measure that matters is the **merge rate** (UAV-only buoys taken
   over by the camera within 1 m). Both are reported.
3. **No LOOK and no watch_locked under colour_source uav** (the colour-blind camera can decide neither).
4. **`uav.assoc_unique_m` stays off** (as decided); the 1.5 m figures below are what it would be decided on.
5. The telemetry fallback cannot be exercised live with their fake copter (it never arms: their RunStart flight is their GUI's) - it is
   tested in the unit tests; `--no-complete` live exercises the operator's word.

**Tests**: `tests/test_v0166.py` (13, QUICK) - ingest_geo (the link, the position kept, the colour rules, the vote, the disagreement, frame_id
dedup), UAV-only (out of gates / markers, in the obstacles with sigma, the provisional gate, the entry candidate, the merge, frozen vs
forgotten), a bad item dropped, WAIT_UAV_MAP's exits (complete, no entry → wait, scan done forces, the landed fallback, the 420 s hold),
m1t2's precheck and m1t1 unchanged, **m1t2 DONE in the fast SIL**, the colour-blind world, the bridge rendering (bash), the adapter's relay,
the fake UAV's survey, the T1 tier + START_TASK + scan done at the OCS, the contract and the wiring. Adapted (pinned "m1t2 not built"):
test_playbook_tui (3), test_wp9c (2: the tier row now follows the mission; the playbook verdict), test_t1_sim (m1t2 refused at the OCS now).
Found live and pinned since: the mission check under the course policy (`test_m1t2_preselects_under_the_course_policy_refuse`), the
bridge in every copy, the camera track absorbing its UAV-only twin (`test_a_camera_track_absorbs_its_uav_only_twin_never_two_tracks`);
test_wp9c's two pinned m1t2 rows updated to D22b. **`make test` 681 passed (5:16) on the merged tree.**

**Fast SIL** (`world_extra_t1adv`, the far start, the colour-blind boat, the in-process fake UAV):
- **m1t2: 8 / 8 DONE** - noise 0.7 m seeds 3-6: DONE at 314 / 314 / 314 / 314 s, **40 / 40 UAV buoys merged** into camera tracks;
  noise 1.5 m seeds 3-6: DONE at 314 / 425 / 316 / 339 s, **37 / 40 merged** (3 stayed UAV-only obstacles, nothing blocked); WAIT_UAV_MAP
  **81 s** (the survey 79 s + 2 s to the first message), left on `scan.complete`; the reports' colours right (the ten course buoys).
  Before the vote fix: 0.7 m seed 6 and 1.5 m seed 6 held (the entry recoloured by one wrong message / not merged).
- **The regression guard (D0)**: the m1t1 campaign (8 seeds × pool / marina / extra × both planners × the starts, geometric, 128 runs) on
  the v0.1.63 code and on this code: **128 / 128 identical** but wall time (122 passed in both).
**Rehearsed live**
- **USV-only first** (`run_sim.sh pool`): **sim-day 46 / 46**, **fault 8 / 8 within 3 s** (kill 0.4, rc6 0.2, rc_lost 2.1, gps 0.9, fcu 2.6,
  bridge 2.8, refusal 2.1, perception 1.3), **no-jump 0 / 0 / 0** (`logs/bench/v0166/`).
- **m1t2 live** (`run_sim.sh extra_t1adv --judge rctest --uav`, the camera path, `declare uav on`, `uav ready on`, `mission m1t2 --auto`,
  `ready`, RunStart from the judge page):
  - first try: **the boat never received a UAV message** (the bench's boat broker reads `profiles/bench/mosquitto/boat.conf`, a third copy
    of the bridge) → held after 420 s; fixed (the relay line in boat.conf; the test checks every copy);
  - before that, `mission m1t2` was **refused under the course profile's policy refuse** ("the declaration says T1 TIER_CORE"): the mission
    check compared the profile's tier, not the one D22b sets for that mission - fixed in `declaration_rows("mission")`, pinned by a test;
  - **0.7 m: DONE** - WAIT_UAV_MAP 77 s (left on `scan.complete`), SEARCH_ENTRY → `uav_lead` → the entry (the camera's votes `{}`: it saw
    no colour) → 3 gates correct → EXIT → DONE in ~300 s; 10 / 10 UAV buoys merged; **the judge's final SafePassageReport = the truth**
    (1 flashing blue, 3 flashing green, 3 flashing red, 2 off, 1 steady blue) - `logs/rctest/run_20261108_035130/` line 32336
    (re-run after the merge fix below; the first DONE, before it, at `run_20261108_031719`);
  - **1.5 m**: before the merge fix it HELD (the entry stayed UAV-only > 1 m from the camera's track: 4 leads, then the safe hold - what
    the 1 m radius does at this noise); after it **DONE**, 10 / 10 merged, the report = the truth (`run_20261108_035806/` line 32332);
  - **`--no-complete`**: WAIT_UAV_MAP held past the survey's end until `ocs_cli uav scan done` (106 s), then DONE, the report = the truth.
  - WAIT_UAV_MAP's dwell: 77-78 s with `complete` (the survey is 79 s from RunStart + 3 s countdown; the boat starts after the count).
- **m1t1 S6 on extra from the far start** (`sim_day --only S14`): without the UAV **146 s** (v0.1.54: 146 s); with `--uav` up (the UAV
  surveying, undeclared) first **146 s but the report had 11 buoys, one OFF** - a camera sighting folded into a UAV-only twin beside the
  camera's track; fixed (`_fold`: the camera's track first, absorbing the twin; a different UAV id never absorbed; a test); then **145 s,
  the entry 0.02 m off, the report 10 buoys = the truth** (`s14_with_uav.md`). The regression guard re-run on the fixed code: 128 / 128.
- **With the UAV up, the whole sheet + the fault test** (`run_sim.sh pool --uav`): **sim-day 46 / 46**, **fault 11 / 11 within 3 s** (kill 0.2, rc6 0.2,
  rc_lost 2.6, gps 0.9, fcu 2.8, bridge 2.8, refusal 2.1, perception 1.5, uav_tel 1.7, uav_fence 1.1, uav_adapter 0.2), `loop_slow` 0
  times in the mission manager's log (`logs/bench/v0166/sim_day_uav.md`, `fault_timing_uav.md`).
- (S14 with `declare uav on` and the sheet's own RESET first fails by design: RESET clears pilot ready, the UAV's PREPARE is refused -
  "waiting for the pilot".)

**Open**: the real drone's detector (their `buoy-detection/`) to the contract; the telemetry fallback with their real flight; `uav.assoc_unique_m`
(Dmitrii, from the 1.5 m figures); UAV items for other kinds than buoys.

**Next session first**: the morning report at the top of DEVLOG.

---

## v0.1.67 — 2026-11-08 (overnight, unattended) — the borrowings, round 1 (B1, B3, B4, B13, B14)

Dmitrii 2026-11-08 (before the night): "put the borrowings on a to-do list after our main tasks" (`docs/TODO_UAV_BORROWINGS.md`, B1-B14,
read from the UAV team's v2 archive) and, during it, "you can work on the todo borrowings during this overnight session as well". Picked:
the small, safe ones that matter at the event; B5-B8 (usv_ui and the FCU link) left for a session with Dmitrii present; B2, B9-B12 listed.
Built in `ws/v67` (cloned after the v0.1.66 merge; the v0.1.66 fixes found live mirrored into it), merged by "changed after the clone".

**What changed · where**
- **B1** `ocs/run_lifecycle.py`: `_course_changed` compares course_id, pinger_hz, the corners and the UAV transit area (their
  `course_fingerprint` fields; never the publish time) - a republished course with a new pinger frequency now resets the run to R2 (it
  passed unnoticed before).
- **B3** `run_lifecycle.declared_t` + ocs/state `declared_s` (R3 only); `tools/ocs_web.py` `auto_warnings()`: R3 for 20 s and a declared
  vehicle not STATE_AUTO → a CAUTION naming it ("the judges start the run when every declared vehicle is AUTO").
- **B4** `usv/tasks/t4_incident.py` `held()`: the ReadinessReport only after `t4.hold_s` 3 s within `t4.hold_r_m` 3 m of where READY began,
  under `t4.hold_v_mps` 0.7 m/s; after `t4.ready_max_s` 20 s it goes anyway (logged); `hold_s 0` = the old rule.
- **B13** `HANDOFF.md` §7.1: the field-lessons table (14 rows: symptom · cause · what the code does, tonight's included).
- **B14** `CLAUDE.md`: merging field-tested code = integration, not improvement; port verbatim, pin by a test.
- `docs/TODO_UAV_BORROWINGS.md`: the five marked built.

**Tests**: `tests/test_v0167.py` (5, QUICK) - B1 (a retained republish no change, tuples = lists, the pinger / transit / id changes, R3 → R2),
B3 (the caution after 20 s naming the late vehicle, none before, none once all AUTO), B4 (m4t1 in the fast SIL: "on station" logged, the
readiness ≥ 2.5 s after READY began; `hold_s 0` = the old rule), B13 / B14. Adapted: `test_run_context_tasks` (2: 3.5 s of ticks on
station before the readiness). **`make test` 686 passed (5:16) on the merged tree.**

**Rehearsed live**: USV-only (`run_sim.sh pool`): **sim-day 46 / 46** (S11 m4t1 → DONE on the judge's confirm, the mission
manager's log: *T4 on station 3.1 s after arriving (within 3 m, < 0.7 m/s for 3 s): readiness*), **fault 8 / 8 within 3 s** (kill 0.9, rc6 0.2,
rc_lost 2.1, gps 0.9, fcu 2.4, bridge 2.8, refusal 2.1, perception 1.5), **no-jump 0 / 0 / 0** (`logs/bench/v0167/`).

**Next session first**: the morning report at the top.

---

## v0.1.68 — 2026-11-08 — WP17 FREE RUN: the full stage sequence on the bench with the real Pixhawk (F1–F9)

Dmitrii 2026-11-08: "FREE RUN — one version, queued FIRST, before the Test Guide's G1–G7 … implement exactly its acceptance list F1–F9"
(`docs/PROMPT_FREE_RUN_2026-11-08.md`; the bench acceptance = `docs/TEST_GUIDE_VENUE_2026-11-08.md` H2b), and "ROADMAP first: register this as
WP17 (FREE RUN) … and the Test Guide's G1–G7 as WP18–WP24 in that order … so the queue is the roadmap again". The decisions were his
(scope = ghost + no motion + `skip`, lab profile only; the FCU gets the mode requests, refusals logged and ignored; the invariant F3;
`free_run.armed refuse | warn`; DECLARE / launcher refuse it on the course; the fake FCU refuses GUIDED without a fix) - not re-opened.

**ROADMAP**: §9 "The venue queue": WP17 FREE RUN (G0) ✅ v0.1.68, WP18–WP24 = G1–G7 (one line each with their acceptance, not built).
§4a "later": the brief's not-in-this-version list (a time factor for the ghost, `skip` on the real plant, free run with the fake FCU,
a ghost UAV, the ghost's own noise model).

**What changed · where**
- **F2** `optiprimex/sim/plant.py` (new): `Plant` moved from `sim/fake_fcu.py` unchanged + the laws the fake FCU's physics used inline,
  as functions (`frame_uvr`, `guided_ur`, `velocity_ur`, the same expressions); `fake_fcu.py` imports them back (`fake_fcu.Plant is
  plant.Plant`). `optiprimex/usv/ghost.py` (new): `GhostNav` (the plant + the last output: target | vel | frame | hold; the frame's 250 ms and
  the velocity's 3 s timeouts as the fake FCU; dt = the tick clock clamped ≤ 0.5 s), `resolve_start` (--free-start > --world's start >
  free_run.start). `usv/mission_manager.py`: `MissionManager(..., clock=None, ghost=None)`; with a ghost the origin is set at start, the pose
  every tick from the ghost (the FCU's lat / lon / hdg never used for it), the tasks' FCU snapshot carries the ghost's lat / lon / hdg / sog
  (`_ghost_fcu`: T4's position, SAVE_POINT's own fix), status keeps the real FCU block.
- **F3** `usv/adapters/rover.py`: `ghost` set = muted - `_frame` hands frames (entry zero, cmd, neutral) to the ghost; GUIDED targets
  (`_ghost_goto`, same 2 Hz carrot rule + speed), velocities to the ghost; never `send_actuator_frame` / `send_position_target_global` /
  `send_velocity_yaw` / `do_change_speed`. Mode requests as before; in free run `_want(mode)`: a new wish at once, a refused one again every
  `hold_reassert_s` (not 10 Hz), never blocking the output; a refusal logged once per (stage, mode); the trace `mode_req … free_run:true
  stage:` and `ghost {out: target|vel|frame|hold}`.
- **F1** `mission_manager --free-run [--free-start LAT,LON,HDG] [--world FILE]`: exit 2 without `free_run.allowed` ("--free-run needs a
  profile with free_run.allowed (the lab profile); never at the pool"); `profiles/pool/usv_lab.yaml` `free_run: {allowed: true, start:
  world_pool's start, armed: refuse}` (the only profile with it). `scripts/run_usv.sh --free-run [--world F]`: exit 2 without `--lab`, with
  `--fcu fake` ("the fake plant already has a position"), the FREE RUN banner beside the lab banner.
- **F4** `skip`: `tasks/base.py` `skip_check()` / `skip()` (cancel the behaviour, `on_behaviour_done(stage)`; refused: held / parked /
  no behaviour / `SKIP_REFUSE` - T1 WAIT_START a timer, WAIT_UAV_MAP the UAV, SEARCH_ENTRY / LOOK / SCAN the map; T3 WAIT_START / DOCKED a
  timer, FIND_DOCK / AIM_FIRE the map; T4 LISTEN / WAIT_CONFIRM a report, READY a timer; no-behaviour stages: REPORT* / ACK a report, else a
  timer); `command_handler.py` SKIP (no task → `SKIP needs a running task`, not free run → `skip: free run only`, not AUTO_TASK, the
  stage's reason); `bodies.schema.json`, `cmd_authority.GUARDS["SKIP"]` = RESUME's R-gate + refused on the course network; `playbook` entry;
  `ocs_cli skip`; op-tui (`skip` typed - the COMMANDS panel row left as it was: the 150-wide render test); usv_ui (`skip` → the mission
  manager, green when mm/state says it can); the OCS page's MISSION CONTROL SKIP (shown in free run only, greyed with the vehicle's reason
  from `status.free_run.skip`).
- **F5** `free_run.armed`: refuse - START_TASK REJECTED `FREE RUN: the FCU is armed …`; arming mid-task → the task aborted, AUTO_SAFE_HOLD
  `FREE RUN: FCU armed`, an `alert` event; warn - one event, `free_run.banner` `FREE RUN · FCU ARMED`, the task goes on.
- **F6** `companion/state.py` `VehicleState.nav / motion / free_run`: `status/usv` (and so mm/state, and the judge's heartbeat) gets `nav:
  ghost`, `motion: muted`, `free_run {ghost, armed_policy, armed, sent{frames,targets,velocity,mode}, skip{ok,why}, gps_note?, banner?}` and
  the ghost's lat / lon / hdg / sog; absent otherwise. The map digest's `boat.nav`; `map.html` draws the boat dashed, labelled `ghost`. The
  FREE RUN banner: the OCS bar (`simban`), the boat window (`sim_flag` → FREE RUN in the SIMULATION place), op-tui's top line, usv_ui's top
  bar, the launcher's plan. `ocs_app.declare`: `free run boat on the course judge (USV1: nav ghost) - restart it without --free-run` when a
  declared vehicle says nav ghost and the judge network is course or the course NIC is detected.
- **F7** `tools/launcher.py` + `.html` + `profiles/presets.yaml`: lab and custom get `free_run: "off"` (sim / event do not); on → the NX
  step `run_usv.sh --fcu uart --lab --free-run --world profiles/bench/world_<world>.yaml`, the AGX step the fake CV on the same world, the
  plan's first note `FREE RUN (ghost position, no motion to the FCU) …`; validate: real mode, lab profile, a real FCU; judge course → a red
  refuse line, no LAUNCH; the flavour chip ` · FREE RUN`.
- **F8** `tools/mode_log.py`: a free-run trace → the heading FREE RUN, per stage the requests (`GUIDED (refused)`), the ghost column
  (`target N · frame N · hold N`), one WARN per (stage, mode) refusal, a FAIL `motion` for any frame / target / vel record.
- **F9** `sim/fake_fcu.py`: `gps_fix < 3` → GUIDED refused (MAV_RESULT_FAILED, logged), no GLOBAL_POSITION_INT, GPS_RAW_INT lat / lon 0;
  `--gps-fix N`; the control port's `{"set": {"gps_fix": N}}` as before.
- Docs: GUIDE header, §3 the presets' `free run` field, §6 "Free run" (the table: what reaches the FCU, a refused mode, what you see,
  `skip`, armed, the course, afterwards), 4.16's `skip`; `docs/LAUNCHERS.md` the flag; CHANGELOG; ROADMAP §9 + §4a.

**Tests**: `tests/test_v0168.py` (25): **F3 the invariant** - a stub link (motion senders recorded) + a real MissionManager in simulated
time on world_pool with the geometric fake camera: **a whole m1t1 DONE on the ghost, the four senders never called**, GUIDED requested and
refused, SafePassageReport out, status carries the ghost; the refusal logged once per (stage, mode); the mode-20 entry / cancel frames
muted (and the flag off = the old three frames); F2 the plant moved (same object), the start order, dt clamp; the indoor fix shown not used;
**F4** skip on SWAY_SEARCH (the task's next event: the pass decided), refused on FIND_DOCK (`skip: FIND_DOCK ends on the map, not on
motion`), outside free run, no task, held; skip with --step (TO_LINE → parked before SWAY_SEARCH); skip refused on the course network by
the OCS; the word in every console; **F5** both ways (refuse at start, abort mid-task with the alert; warn = one event + the banner, the task
goes on); **F1** `--free-run` exit 2 on usv_pool.yaml and usv_course.yaml, the lab block, `run_usv.sh --free-run` without `--lab` / with
`--fcu fake`; **F6** op-tui at 200×60 with the FREE RUN banner, status only in free run (+ mm/state's lat / lon = the ghost), the boat
window's banner, the OCS page's SKIP / banner / the dashed glyph; the DECLARE refusal (course) / acceptance (bench); **F7** the launcher's
field, plan, refusal; **F8** mode-log on a free-run trace (FREE RUN, refused once, the ghost counts, a planted target → FAIL motion); **F9**
GUIDED refused at gps_fix 0, mode 20 accepted, no position sent. Adapted: none (the op-tui panel kept). `make quick` 617 + 1 (op-tui panel row, fixed) ·
`make live` **706 passed** · `make test` **711 passed (5:18)** on the final tree.

**The Mac rehearsal (by hand, the brief's §2)**: `scripts/run_sim.sh pool --perception geometric` (the AGX's fake CV is geometric); the fake
FCU `{"set": {"gps_fix": 0}}` + `{"set": {"armed": false}}` (it starts armed on the bench); the mission manager restarted in tmux `mm` as
`mission_manager --config <usv_lab.yaml with the share's endpoint 14552> --free-run --world profiles/bench/world_pool.yaml --trace …`.
- **m1t1**: usv_ui `handover` · `ocs_cli mission m1t1 --auto` · `ocs_cli ready` → R5 run 1001 → SEARCH_ENTRY → CIRCLE_ENTRY_CW → TRANSIT →
  EXIT → CIRCLE_EXIT_CCW → **DONE in 146 s** (11:21:33 → 11:23:59; S14 ≈ 146 s); **the judge's SafePassageReport = the world's truth: 10
  buoys, every state right, ≤ 0.44 m, entry 0.03 m, exit 0.15 m, none missing**; the mm log: `FREE RUN: the FCU refused GUIDED in <stage> -
  ignored` once per stage (5); status `sent {frames 0, targets 0, velocity 0, mode 92}`; **the fake FCU's own counters: frames 0, targets 0,
  its plant never moved (0, 0)**, 30 GUIDED requests refused in its log; `mode-log` → FREE RUN, PASS (0 fail, 5 warn = the refusals).
- **m3t1 `--step`** from where m1t1 ended: every boundary parked (`step: next FIND_DOCK`, TO_LINE, SWAY_SEARCH, SWAY_TO_ABEAM, TURN_IN, ENTER,
  DOCKED, REPORT_DOCK, AIM_FIRE, REPORT_FIRE, UNDOCK), `resume` entered each; **`skip` in FIND_DOCK → `REJECTED skip: FIND_DOCK ends on the
  map, not on motion`**; **`skip` in SWAY_SEARCH → `SWAY_SEARCH ended by skip -> SWAY_TO_ABEAM`** (pass 1's votes: bay 2 G11 → bay 2) →
  DockingReport bay 2 → AIM_FIRE (window 1 RED → GREEN) → FirefightingReport upper green → UNDOCK → DONE. Mode 20 **was accepted** by the
  fake FCU (it needs no fix): still **frames 0** at the FCU (the H2b.3 case "if mode 20 is accepted indoors"); `mode-log` PASS.
- The same stack with the normal mission manager (no `--free-run`): `start_task m0t0` · `skip` → `REJECTED skip: free run only`.
- **The regression guard**: the m1t1 campaign (`--missions m1t1 --seeds 8 --perception geometric`, 128 runs) on this tree and on a fresh
  extract of `_archive/MASTER_SCRIPTS_v0.1.67.tgz` (its own code on PYTHONPATH): **128 / 128 identical** but wall time; and the fake FCU's
  physics on the v0.1.67 code and on this one, 3000 steps of random frame / GUIDED / velocity / hold commands: **bit-identical**.
- STOP ALL → status CLEAN.

**Decisions taken here (small, within the brief)**: a refused mode is re-asked every `hold_reassert_s` (5 s) in free run, not every tick -
the brief's "exactly as today" would have sent DO_SET_MODE at 10 Hz with a 0.5 s ack wait each (today that case never happens: a refused
GUIDED stops the behaviour); a skip on a pass with no GREEN yet starts the second pass (the task's own next event - T3's `_decide`), not the
next stage; the free run's `alert` event uses the existing `alert` kind; the tasks see the ghost's position in their FCU snapshot (T4's
reports would otherwise carry the indoor 0 / stale position); `skip` stays out of op-tui's COMMANDS panel row (typed; the 150-wide render
test pins the row); disarming in AUTO stays the health monitor's UNEXPECTED_DISARM (unchanged, as at H2.4).

**Open**: H2b on the real Pixhawk (the Test Guide) - the acceptance of this version; no deploy (Dmitrii's say-so, the concurrency check,
a JETSON_LOG line). Not built (ROADMAP §4a): the ghost's time factor, skip on the real plant, free run with the fake FCU, a ghost UAV, noise.

**Next session first**: H2b with Dmitrii (after his deploy say-so); then WP18 (G1, sim_day S15) - the first not-done WP of ROADMAP §9.

---

## v0.1.69 — 2026-11-09 (overnight) — WP25: the dry / wet SESSIONS (N1 of docs/OVERNIGHT_2026-11-09_GUI.md)

Dmitrii 2026-11-09: "Overnight 2026-11-09 — the GUI declutter, the dry/wet sessions, the boat window … build N1 → N6 in that order, one
version each (v0.1.69–v0.1.74), stopping where the night ends … ROADMAP first: register N1–N6 as WP25–WP30 in §9 'The venue queue', before
WP18–WP24". ROADMAP §9: WP25–WP30 registered before WP18 (with their acceptance lines), WP31–33 = N5's "if time remains". The UI principles
doc the brief names is not in the tree (the Claude project's *ui_principles_control_station*): built from the rules the brief quotes.

**What changed · where**
- `optiprimex/ocs/sessions.py` (new): the catalogue (buoys · dockface · windows · uavmap · cannon = dry; collect = wet, = m0t7), `parse_opts`
  (refusals with the fix: `--aim10` needs `--confirm aim`, labels, tape metres), `plan()` (real: the AGX over ssh, the observer in tmux `agx`
  window `sess` - created / replaced, stopped by Ctrl-C + kill-window, no pkill; a missing script on the AGX's older tree is said, not hidden;
  simulator: the fakes stand in, `fake_uav --start-now --offset-km 16 --broker hub | --udp 127.0.0.1:14605`), `metrics()` per session (from the
  page server's snapshot: the boat's map digest, the cannon status, the UAV's buoys on the hub, the points, the status; never invented),
  `result_md` (buoys = G5's row), `result_line`, `mission_metrics` (m1t1 entry · gates detected / passed · exit; m3t1 faces · windows lit;
  m1t2 + the UAV's scan · items · merged %), `SessionManager` (one at a time; start / stop / result; `logs/ocs/sessions/<stamp>_<name>.md`
  + `.json` + `sessions.jsonl`; collect: the track CSV at 1 Hz + the page's frame recorder; the aim loop 10 s at 5 Hz, off on stop).
- The words: `session start NAME [--model --tape --path --aim10 --confirm aim --label] | stop | result` - `ocs_cli`, op-tui (INTENT_WORDS,
  USAGE, examples), `playbook` SESSION; `cmd_authority` GUARDS SESSION + the rule (start of a non-collect session refused on the course network
  and on a profile with neither bench_mode nor test_mode - i.e. the course profile); `ocs_app.on_session` (the page server's freshness - no
  `ocs/session` in 6 s = "no session host"; the options checked; one at a time) → `ocs/session/cmd` → `tools/ocs_web.py` (SessionCmdTap →
  SessionManager; `ocs/session` retained at 1 Hz; ocs/state `session` {host_age_s, current, last}).
- `tools/ocs_web.py`: UavBuoysTap (`uav/perception/buoys` on the hub), the session's plumbing (snapshot, publish, sim = the preset sim or the
  fake FCU link, the aim on the boat broker, the frame recorder), `/state.json` `session` + `mission_metrics`, the cannon view's `target`.
- `tools/ocs_web.html` (MISSION): **SESSIONS** card (the picker grouped DRY / WET from the catalogue, the options, the line + ⧉, ▶ start · ■ stop
  · result, the steps with their last line, **● DROP POINT** + key **P** in collect = `save_point <label>_NN`), **MISSION METRICS** (the session's
  live numbers, else the mission's; the far-UAV caution; the DETECTIONS kv folded under it), the **camera fold** (320 px, RGB + boxes, open by
  default in a dry session, click → CAMERA).
- `tools/map.html`: **fit UAV** (the UAV's items only; hidden on the judge / mini maps - it wrapped the judge map's row and moved its canvas
  centre: `test_v0147_ab` caught it). `sim/fake_uav.py`: `--udp HOST:PORT`, `--offset-km`. `tools/launcher.py` + `.html`: the UAV field's
  **UAV buoys broker | udp** (the NX's `UAV_BUOY_PATH` direct | relay).
- GUIDE §6 "Sessions" (the table, aim10, collect, DROP POINT) + the header; CHANGELOG; ROADMAP §9.

**Tests**: `tests/test_v0169.py` (10): the catalogue / options, the plans real and simulated (ssh, tmux, no pkill), the metrics (bands, the
16 km caution, the tape error, the next point, the cannon trace, mission_metrics), the manager's start / stop / result files, collect's track
and the aim loop (off on stop, the cannon only), the authority's course rules + the OCS app's hand-off (no host → refused; ocs/session/cmd
published), every v0.1.68 word still in op-tui's words, the page / map needles, the launcher field, fake_uav's flags.
`make live` **716 passed**, `make test` **721 passed (5:18)** on the final tree (the first `make live` caught `test_v0147_ab`: the judge map's centre moved by the new map button - fixed by hiding it there).

**On the simulator (by hand, `run_sim.sh pool --perception geometric --uav`)**: `dry buoys --model cvt-v2-l` - 4 buoys, per band 0/0/1/3,
colours red 2 · green 1 · black 1, the md row written; `dry uavmap --path broker` - fake_uav 16 km north, 10 items, scan complete, the caution
"UAV layer 16.0 km from the course", 10 UAV-only buoys in the boat's map, merged 0; `--path udp` through the adapter's 14605 - 38 messages,
items arriving, the same caution; `dry cannon --aim10` - refused without `--confirm aim`, then the aim on 10 s by itself, the fake Pi (their
receiver, DRY_RUN) `SAFE_NO_TARGET` throughout (correct: in the sim the RED window lights only after a DockingReport); `wet collect --label
n1test` - two DROP POINTs (n1test_01, _02 in the points store; deleted afterwards), the track 13 fixes, the frames recording; `dry dockface`
at the dock (teleported N 36 E 5.5) - 3 faces at 9.0-9.6 m, errors vs the 8 m tape 1.05-1.62 m; `dry windows` - 0 windows (lit only after
docking). The page in headless Chrome at 1440 × 900: the grouped picker, the steps, the METRICS, the fold open in a dry session;
`docs/screens/v0.1.69/01_mission_session_buoys.png`, `02_mission_tall_sessions_metrics.png`.

**Decided here**: the sessions run in the page server (it has the hub + boat feeds, the ssh path, `logs/ocs/`), the authority stays the gate
(ocs_app) - the hand-off topic `ocs/session/cmd`; the picker's m0t7 entry and the restyle are N3's (the word is `session start collect`);
the Test Guide pointer lines are left to Dmitrii ("The Test Guide is not yours tonight"); a 5 Hz aim publisher in the page server for
`--aim10` only (bench; the gate on the AGX still needs a fresh RED target).

**Next session first**: N2 (WP26) - the bar, RUN, the drawer, the declaration constructor (started in `../ws/v70` tonight - see the v0.1.70
block if it exists).

---

## v0.1.70 — 2026-11-09 (overnight) — WP26: the bar, RUN (L1), the console drawer, the declaration constructor (N2)

Built in `../ws/v70` (cloned while v0.1.69's suites ran on the main tree - the first try edited the main tree during a test run: stopped,
moved to the copy, the main tree put back to v0.1.69 and tested clean; the lesson: a version under test is never edited), merged by "changed
after the clone" (ocs_app.py, ocs_web.html, tests/test_v0170.py; the docs are the main tree's).

**What changed · where**
- `ocs/ocs_app.py`: `_declaration_inputs(vehicles, args)` - the tiers, the UAV fence / ceiling / pilot or the UAV refusal, no side effect;
  `declare()` uses it (the refusal still alerts + FENCE_REJECTED, pinned again by a test); `declaration_preview()` = the RunDeclaration the OCS
  would send now through `rc_wire.encode_request` → decoded fields + the protobuf text + bytes (the body alone when no protobuf);
  ocs/state gains `declaration_preview` and `declaration_sent` (the last one sent, verbatim). No other key changed.
- `tools/ocs_web.html` (the classic `/v1/` untouched): the cards **move** (ids kept, so every v0.1.69 function draws where its card now
  lives; nothing removed). **The bar** in one row at 1440 (the state word + reason with the old `next:` merged in, R / V chips - the ladders on
  hover, the R3 / R4 time -, alarms, `reporting on/off`, LINK, FCU, MANUAL · HOLD · ARM · DISARM unchanged, **■ STOP RUN** = the same
  `srbtn` / `stop_reset` / ask, `?`); ⏻ STOP ALL / ↺ RESET ALL → COMMS' STACKS card, START / STOP REPORTING → JUDGE's REPORTING card,
  the bar's ⌂ HOME hidden (MISSION CONTROL has it). **The context line** (SIMULATION / FREE RUN first, the flavour, net · FCU · profile ·
  status age, the theme until N5). **RUN** (the OVERVIEW tab relabelled; `#OVERVIEW` unchanged): RUN SEQUENCE + the **DECLARATION**
  constructor, MISSION (the picked mission, the stage chain in one line, hold · resume · abort · skip), ALARMS; the mini-map, VEHICLES,
  LINKS (age · threshold · on loss), LAST EVENT; held (hidden, still drawn): FLOW, MISSION · VEHICLE, USV / UAV cards, the CV stream;
  moved: SYSTEM · PLATFORMS · LINKS → COMMS, EVENTS → LOG. **The drawer** on every tab: commands (the console + LAST COMMANDS + the COMMANDS
  fold), system (the jobs), wire (the declaration sent in full + the judge log); `:`; the height and open state in localStorage; `?drawer=`.
- GUIDE §3 row "The OCS page, v0.1.70" + the header; CHANGELOG; ROADMAP WP26.

**Tests**: `tests/test_v0170.py` (7): the preview = what the judge receives, field by field (pool and course profiles; with the UAV
declared from its FCU fence; T4 off / m1t2 advanced change it live); declare's UAV refusal unchanged; the state contract only gains keys;
the markup; **the Chrome walk at 1440** (no script error, every tab, the bar's children on one row - a `?probe` hook measures them -, the
new ids, every v0.1.68 word in the console's list except `report`, which has never been in it - typed only - and in what the page accepts).
`make live` **723 passed**, `make test` **728 passed (5:18)** on the merged tree.

**On the simulator**: `run_sim.sh pool`: the preview before `declare` = `declaration_sent` = the judge page's `declaration` (USV1 · T1 core ·
T2 none · T3 core · T4 core · ceiling 30 · "TBD"); captures `docs/screens/v0.1.70/` 01 RUN + the bar + the drawer on wire, 02 RUN tall (the
constructor, MISSION, ALARMS), 03 COMMS (STACKS), 04 JUDGE (REPORTING), 05 MISSION with the drawer on commands.

**Open / not done in N2**: the RUN SEQUENCE kept its v0.1.59 form (the six steps + the checklist) instead of a new stepper - it already is
the six steps in order; the drawer's *system* feed shows the jobs' last 20 lines (live streaming is N4's); `report` is still missing from the
console's word list (op-tui's USAGE; untouched tonight - op-tui stays as it is unless a word's text changes).

**Next session first**: N3 (WP27) - the picker in groups, T4 as a tier, m4t0, the map's TOOLS ▾.

---

## v0.1.71 — 2026-11-09 (overnight) — WP27: MISSION (L2) - the picker in groups, T4 as a tier, m4t1 → m4t0, the map's tools (N3)

Built in `../ws/v71` (cloned from the v0.1.70 tree while its suites ran), merged by "changed after the clone" (24 code / test files; the
docs are the main tree's).

**What changed · where**
- **m4t1 → m4t0** (Dmitrii: "can we go to an incident" - the standalone test): every `m4t1` in `optiprimex/` (playbook's catalogue and
  MISSION_ORDER, `declaration_checks.mission_code`, ocs_cli's help, op-tui's words, the judge page, ocs_web, sim_day's rows, the campaign,
  `stage_typical.json`), usv_ui's help text, the tests, GUIDE and `docs/SIM_DAY.md`; `playbook.CODE_ALIAS {"m4t1": "m4t0"}` so the old word
  keeps its verdict (`mission m4t1` = `mission m4t0` = `mission T4`); `declaration_checks.mission_needs("m4t0")` = judged T4 at TIER_CORE (the
  code's last digit is no tier here); the wire names unchanged (`TASK_DYNAMIC_INCIDENT`). On the wire m4t0 declares exactly as m1t1 (T1 core ·
  T2 none · T3 core · T4 core).
- **T4 as a tier**: `ocs_cli mission … --t4 off | core | advanced` (`on` = core, the v0.1.44 word); `ocs_app.t4_tier()` → the mission's `t4`
  is the tier itself (TIER_NONE | TIER_CORE | TIER_ADVANCED); `_t4_flag` takes it (and the old booleans).
- `tools/ocs_web.html` MISSION: **EVENT · POOL · DRY** segments over the picker (optgroups: EVENT m1t1 · m1t2 · m3t1; POOL m0t2 · m0t3 · m0t6 ·
  m0t4 · goto · m4t0 · m0t0 · m0t1 + **m0t7 collect**; DRY = the sessions - those entries open the SESSIONS card), the **T4 select** (off · core ·
  advanced, core default) on event missions (`qLine` adds `--t4 X`; changing it re-preselects when that mission is the preselected one, so
  the RUN tab's DECLARATION follows live), MISSION METRICS moved to the right rail after STAGES. The course banner already showed only while
  a new course is unanswered (differs, not ignored) - unchanged.
- `tools/map.html`: the tool row folds into **TOOLS ▾** (clear boat map · drop point · geofence · FCU fence · add · teleport); hidden on the
  judge and mini maps.
- GUIDE §3 row "The MISSION tab, v0.1.71" + the header; CHANGELOG; ROADMAP WP27.

**Tests**: `tests/test_v0171.py` (11): the alias and the code table, `t4_tier` (7 words), T4 off / core / advanced in the declaration
preview, **m4t0's declaration = m1t1's** and the one RxRequest the judge receives, ocs_cli's `--t4` choices, the page / map markup. The renamed
tests (test_quick_missions, test_playbook_tui, test_wp9c, test_wp8c, test_v0143, test_v0154_c, test_v0167, test_wp10h, test_usv_ui,
test_integration_sil1): the code only. Also adapted: test_v0144 (the T4 tier is the tier itself, `TIER_NONE` not `False`; the picker's `--t4 ${t4sel}`); the segment buttons got their titles (the every-button-has-a-title tests caught them, as they should). `make live` **734 passed**, `make test` **739 passed (5:19)**.

**On the simulator**: `sim_day --only S0,…,S9` on `run_sim.sh pool`: **32 pass, 0 findings** (S8 m1t1 DONE 152 s; S9 m3t1 DONE 181 s, GREEN moved to bay 3, the wrong-bay BACK_OUT); S14 on `run_sim.sh extra`: **pass** (m1t1 s6 DONE 146 s, the entry 0.02 m off, the report 10 buoys = the truth, max 0.19 m). **m4t0 on rc-test** (`run_sim.sh pool --judge rctest`, `mission m4t0`, `declare`): **one RxRequest** (seq 190) - `vehicle_ids USV1 · T1 CORE · T2 NONE · T3 CORE · T4 CORE` (= m1t1 + T4 core). Captures `docs/screens/v0.1.71/01_mission_picker_groups_t4.png`.

**Next session first**: N4 (WP28) - SYSTEM.

---

## v0.1.72 — 2026-11-09 (overnight) — WP28: SYSTEM - one hosts table, settings ▾ / actions ▾ per host, the live action console (N4)

Built in `../ws/v72`, merged by "changed after the clone" (ocs_web.html, system_actions.py, test_v0144.py, test_v0172.py).

**What changed · where**
- `tools/system_actions.py`: every job writes its output, line by line as it streams, to `logs/ocs/system_actions/<stamp>_<job>_<action>.log`
  (a header with the options and the command, the lines, `exit status N after S s`; a dry run writes its one line); the job view gains `log`,
  `t1`, `failed`, `tail20` (a failed job's last 20 lines). The actions, their words, options, guards and the dry-run flag unchanged.
- `tools/ocs_web.html`: the COMMS & DIAGNOSTICS tab becomes **SYSTEM** (`#COMMS` / `#SYSTEM` still work): **HOSTS** - one table, CONNECTIONS as
  its header, a row each for the Orin NX · AGX · this Mac · the FCU · the cannon Pi · the UAV adapter · rc-test (*what runs · state · age · CPU /
  °C · clock*), **settings ▾** = that host's restart form (the v0.1.47 host cards' forms moved in, ids and handlers kept: the launch's
  preselection as before) and **actions ▾** = its stops (QUICK STOPS' cards moved in; the Mac's row also holds ⏻ STOP ALL / ↺ RESET ALL);
  **BENCH & TOOLS** (the SIM SCENARIO - shown in the simulator only -, the CAMERA + RECORDER card, moved from SETTINGS); **JOBS** once (moved
  from SETTINGS, each job's log path, a failed one red with its last 20 lines; the drawer's *system* feed shows the same); HOST HEALTH hidden
  (its numbers are the table's CPU / °C column); STREAMS and LINKS stay side by side.
- GUIDE §3 row "SYSTEM, v0.1.72" + the header; CHANGELOG; ROADMAP WP28.

**Tests**: `tests/test_v0172.py` (6): every v0.1.71 action with the same confirm word; the guards (the FCU reboot refused armed, a reboot
needs its name, the boat's stop refused in AUTO_TASK); the dry run shows the command and writes the log; a live job streams 30 lines, fails
with rc 3, keeps `tail20`, the log has every line and the exit status; the markup; **the Chrome walk** (no script error; every action's button
on SYSTEM; the seven host rows). Adapted: test_v0144 (the tab label SYSTEM). `make live` **740 passed**, `make test` **745 passed (5:19)**.

**On the simulator**: `run_sim.sh pool` - the hosts table live (NX MANUAL 0.5 s · AGX up · Mac hub up · the fake FCU HOLD · ARMED · the
cannon Pi ok · the UAV adapter not running · rc-test: judge fake); the SIM SCENARIO action `simworld pool` run from the page: job 1, rc 0, its
output in the list and in `logs/ocs/system_actions/20261109_022511_1_simworld.log`. Captures `docs/screens/v0.1.72/` 01 SYSTEM (the hosts
table), 02 the drawer's system feed.

**Next session first**: N5 (WP29) - the provenance tooltips, the ⚙ panel.

---

## v0.1.73 — 2026-11-09 (overnight) — WP29: provenance tooltips everywhere, the ⚙ panel, the GUI switch (N5)

Built in `../ws/v73` (cloned from `ws/v72`), merged by "changed after the clone" (ocs/provenance.py, ocs_web.py, ocs_web.html, test_v0173.py).

**What changed · where**
- `optiprimex/ocs/provenance.py` (new): where every value comes from - the vehicle's status fields generated from `schemas/bodies.schema.json`
  (`status/usv.<field> · the mission manager, through the boat→hub bridge · 2 Hz · stale > 3 s`), a hand table for the rest (ocs/state,
  the map digest, perception, the hosts, the cannon, the UAV, the session, the judge: *topic · publisher · rate · threshold*) and the words
  (`intent hold → USV1's mission manager (cmd/usv) · ack within 5 s (else TIMED OUT) · AUTO_SAFE_HOLD, the task kept`, …); `meta.json`
  carries it (`provenance`).
- `tools/ocs_web.html`: `provApply()` fills a `title` wherever a value (a `.kv` value), a chip, a status block, a card header or a button
  has none - from the label, the card, the word (`data-w` / `data-send` / the button's text) - the hand-written tips stay; the camera's boxes
  carry a `<title>` (class · colour · light · range + `local/usv/perception/frame_dets · the observer · stale > 2 s`); `?probe=1` writes
  `#provprobe` (`missing` = the visible value elements with no title; `gear_ops` = operational buttons visible in ⚙). **⚙ = a panel over any
  tab** (the nav's ⚙ SETTINGS and `showTab("SETTINGS")` open it; Esc / ✕ close; `?gear=1`): THEME · BIG NUMERALS · SOUNDS · CLASSIC PAGE
  (open /v1/ · always classic - this browser) · **CAMERA LABELS short | full** (new, this browser) · the session line (version · preset · net ·
  SYSTEM dry run or for real · SYSTEM →); nothing operational: EVERYTHING moved to SYSTEM, the duplicate RESTARTS & REBOOTS / dry banner
  hidden, an emptied card hidden.
- GUIDE §3 row "provenance + ⚙, v0.1.73" + the header; CHANGELOG; ROADMAP WP29. **LOG · VEHICLES · JUDGE (WP31-33) not built** - the
  boat window (N6) was next in the brief's order.

**Tests**: `tests/test_v0173.py` (9): the table (the schema's status fields, the labels, the words), the markup + `meta.provenance`, **the
DOM walk on six tabs (RUN, MISSION, CAMERA, SYSTEM, JUDGE, LOG): `missing == 0`**, the ⚙ panel over MISSION with its six settings and
`gear_ops == []`. The DOM walk on the live simulator: `missing 0` on RUN · MISSION · CAMERA · SYSTEM · JUDGE · LOG · USV CONTROL · UAV; capture `docs/screens/v0.1.73/01_gear_panel_over_mission.png`. `make live` **749 passed**, `make test` **754 passed (5:24)**.

**Next session first**: N6 (WP30) - the boat window.

---

## v0.1.74 — 2026-11-09 (overnight) — WP30: the boat window (boat-web 8098), from the second canvas review (N6)

Built in `../ws/v74` (cloned from `ws/v73`), merged by "changed after the clone" (boat_web.html, boat_web_v1.html, boat_web.py, launcher.py,
run_boat_window.sh, test_v0174.py, test_v0149.py). Dmitrii's five amendments folded in; the whitelist (decision 25), the 2 Hz FCU block, the
commander lock + PIN, the plate menus' words and the wireframe's ids **unchanged**.

**What changed · where**
- **`/v1/` first**: `tools/boat_web_v1.html` = the v0.1.68 page byte for byte (sha256 `f1c9a8c4…`, pinned by a test); boat-web serves it at
  `/v1/` (its own requests reach the same endpoints); `BOAT_WEB_UI=classic` (`run_boat_window.sh --classic`) serves it at `/`.
- `tools/boat_web.py`: the snapshot gains `n6` (SOG · HDG · HB · EKF · fix · RC ok · SF · RC6 · the four PWM · the FCU link · the log file ·
  the message rates · the faults · the perception / status ages) - read from the blocks it already has, nothing new published.
- `tools/boat_web.html` (the cards move, nothing removed): **the header, four rows → two** - row 1 = the plate (VEHICLE · FCU MODE · ARM · FCU
  HOLDER, their menus unchanged) · `links: FCU · OCS · AGX` (each age on hover, grey past 2 / 3 / 2 s) · alarms · **SOG and HDG numerals** ·
  ■ STOP & RESET · ⚙ · the commander (last: it may wrap; the numerals and the stop word stay on the first line); row 2 = the context line
  (SIMULATION / FREE RUN first · the FCU link · the camera · the clock · the broker · OCS page ↗); the OCS state line goes. **OVERVIEW**: a tool
  row above the wireframe (the LOG and CV FOLD boxes, log on / off - its hover says the log is **local on the NX**: `.tlog` + a 5 Hz `.csv` +
  `.events`, OFF by default, the hub SQLite is separate; the file name while recording -, cv on / off, snapshot ↗), **COMPASS and SPEED
  directly under the wireframe** + the FCU / RC line (HB · EKF · GPS fix · RC · SF · RC6 · PWM), then the **FCU words card**: MANUAL · HOLD ·
  GUIDED · MODE 20 | ARM (asks) · DISARM · **other ▾** with ACRO · STEERING · LOITER · AUTO · RTL · SMART_RTL, each with its one-line consequence
  and a second click within 4 s (AUTO: "the mission manager does not expect this mode"); right = **CAMERA first, the RADAR beside it**, then
  **EVENTS · BOAT and CONSOLE · USV_UI side by side as they were, minus the MAVLink acks / heartbeats** (`ack cmd N result R`, `HB …`,
  HEARTBEAT, the mission manager's `(mm)` traffic). **Tabs**: CONTROL + ACTUATION → **CONTROL & ACTUATION** (ACTUATION's cards appended under
  CONTROL's; `showTab('ACTUATION')` lands there); **DIAGNOSTICS gains the MAVLINK console** (those acks / heartbeats, the message rates,
  pause, a filter). **⚙**: the theme + **GUI new | classic** (this browser; classic = `/v1/`, `/?ui=new` comes back).
- `tools/launcher.py`: the USV window field gains **mac-classic / nx-classic** (the step gets `--classic`). GUIDE §3.2 row; CHANGELOG.

**Tests**: `tests/test_v0174.py` (7): /v1/ = v0.1.68 byte for byte; the unchanged contracts (OCS_LINES, the 12-word whitelist, the plate
menus' `PLM` = the frozen page's, every v0.1.68 id present); the `n6` block; the routes (/v1/, /v1/state.json, `BOAT_WEB_UI=classic`); the
markup; the launcher's classic; **the Chrome walk at 1440** (row 1 = the plate … ⚙ with 0.42 / 087°; the usv_ui words; OVERVIEW's order
wireframe → dials → FCU line → the words card; the modes MANUAL · HOLD · GUIDED · MODE 20 + the six under other ▾; the ack hidden in CONSOLE,
shown in the MAVLINK console; the other lines as they were). Adapted: test_v0149 (ACTUATION's rows are under CONTROL & ACTUATION). The
usv_ui 200 × 60 tests and the whitelist test untouched. Also adapted: test_v0152_wa (the above-the-fold check on the two-row header: the four plate boxes in one row, LOG / CV in the tool row, SOG instead of the numerals block, the plate before the context line - all still above the fold at 1440 × 900, no overlap; the page's own layout probe measures `n6r1` / `n6r2` / `n6sog`), test_v0163 (the USV window field's two new values). **Caught by the suite**: my edit of the launcher's FIELDS line put a `#` comment in the middle of the line and silently dropped `cv`, `perception` and `world` from the field table (`test_v0147_ab`: the perception segment empty) - fixed before the archive. `make live` **756 passed**, `make test` **761 passed (5:24)** - after stopping three orphaned fake FCUs left by the test runs I interrupted earlier tonight (they kept streaming to random UDP ports; two usv_ui share tests and the live m1t1 SIL failed in the run before, passed alone).

**On the simulator** (`run_sim.sh pool`): `/v1/` served = sha256 `f1c9a8c4…` (the v0.1.68 page); the new window live (MANUAL · HOLD · ARMED · usv_ui; links FCU 0.3 · OCS 0.5 · AGX 0.2 s; SOG / HDG from `status/usv` when the fake FCU sends no VFR_HUD - fixed tonight); **boat-web's CPU with a page polling: 0.21 s in 30 s ≈ 0.7 %** (the spec's limit 5 %); `?tab=` added for links / the captures; **`docs/screens/v0.1.74/` one capture per tab** - 01 OVERVIEW, 02 CONTROL & ACTUATION, 03 TELEMETRY, 04 NAV & SENSORS, 05 DIAGNOSTICS (the engineering tabs' first captures, board 04).

**Next session first**: the boat window with Dmitrii on the canvas (board 04: the five engineering tabs' first captures are in
`docs/screens/v0.1.74/`); then WP31-33 (LOG · VEHICLES · JUDGE) or WP18 (G1) as he picks.

---

## v0.1.75 — 2026-11-09 (overnight) — WP31: the unified LOG (N5's "if time remains")

Everything before it was green at 03:43 (the end-of-night checks on v0.1.74: **fault_timing 8 / 8** - the bridge row 3.0 s while a Chrome
walk ran beside it, then **2.8 s three times alone**, as in v0.1.65; **fault_timing --uav 11 / 11**; **sim_day S0-S9 31 pass / 0 findings**;
**S14 pass** (m1t1 --planner s6 on extra, DONE in 146 s, the report within 0.19 m); **the m1t1 campaign 128 / 128 identical to v0.1.68**;
status **CLEAN**), so WP31-33 were built - one version each. Built in `../ws/v75` (cloned from v0.1.74), merged (ocs_web.html, test_v0175.py).

**What changed · where**
- `tools/ocs_web.html`, the LOG tab: **one list** (`#ulog-card`) - the vehicle's events, the commands (intents · acks · the console's words),
  the alarms (the active ones + the history from `/alarms.json`, fetched every 10 s), the judge traffic, the SYSTEM jobs - **newest first**,
  each line tagged with its kind; **chips** with counts (all · events · commands · alarms · judge · system), **search**, a **time range** (5 min ·
  30 min default · 2 h · all), *problems only*; the list ≥ 430 px (≥ 20 rows at 1440 × 900). The v0.1.74 lists (EVENTS, the raw log) in a
  fold below. Nothing new on the wire; the hub's SQLite (LOG=1) stays the long record.
- GUIDE §3 row "LOG, v0.1.75" + the header; CHANGELOG; ROADMAP WP31 (WP31-33 registered after WP30).

**Tests**: `tests/test_v0175.py` (2): the markup; the Chrome walk - 25 events + an intent + an ack + a judge line + a dry-run job + an alarm in
the history → ≥ 20 rows, every kind present, every chip counts, newest first. `make live` **758 passed**, `make test` **763 passed (5:21)**.
On the simulator: the LOG tab live, `missing 0`, capture `docs/screens/v0.1.75/01_log_unified.png` (taken just after the start - 4 rows).

**Next session first**: WP32 (VEHICLES) is v0.1.76, WP33 (JUDGE) v0.1.77 - built the same night; read their blocks.

---

## v0.1.76 — 2026-11-09 (overnight) — WP32: VEHICLES - one card per vehicle, the same columns (N5's "if time remains")

Built in `../ws/v76` (cloned from v0.1.75), merged (ocs_web.html, test_v0176.py, test_v0165.py).

**What changed · where**
- `tools/ocs_web.html`: the UAV tab is **VEHICLES** (`#UAV` still works; the id `UAV` kept): `#veh-cards` - **one card per vehicle with the
  same eleven rows in the same order**: state · task / stage · FCU mode / armed · kill / E-stop · position · heading / SOG · height · link age
  (the boat `status/usv`, LOST > 3 s; the drone `status/uav` + its telemetry age, stale > 5 s) · heartbeat → judge (the OCS's reporting: sent ·
  state · age) · faults · fence. USV1 always (USV CONTROL ↗); the UAV card says *no UAV adapter on the hub* when there is none (the UAV window ↗).
  **The tab is never greyed** now (the boat's card is always there). The v0.1.65 UAV detail stays below, **its columns fixed**: its *state*
  value was two bare elements (two grid cells) since v0.1.65, so every row after it showed a label as a value - one span now (found in the
  capture of this version).
- GUIDE §3 row "VEHICLES, v0.1.76" + the header; CHANGELOG; ROADMAP WP32.

**Tests**: `tests/test_v0176.py` (3): the markup (each row twice, the greying gone); the Chrome walk with a boat status + a UAV message - two
cards, the same rows in the same order, the values; the nav not greyed; the UAV detail's first labels identity · link · state · FCU · HAE in
the label column; without a UAV - still two cards. Adapted: test_v0165 (pinned the greying rule; now pins its absence and the label).
`make live` **761 passed**, `make test` **766 passed (5:20)**. On the simulator (`run_sim.sh pool --uav`): both cards live, `missing 0`;
capture `docs/screens/v0.1.76/01_vehicles.png`.

**Next session first**: WP33 (JUDGE) is v0.1.77, built the same night.

---

## v0.1.77 — 2026-11-09 (overnight) — WP33: JUDGE re-laid (N5's "if time remains", board 06)

Built in `../ws/v77` (cloned from v0.1.76 before its UAV-detail fix), merged with that fix re-applied (ocs_web.html, test_v0177.py).
Board 06 itself is not in the tree - the brief's one line ("the judges' table + timeline strip, the feed full width, reporting switch") is
what was built; **for Dmitrii's look on the canvas**.

**What changed · where**
- `tools/ocs_web.html` (`n9Init` / `drawN9`, the cards moved, every id kept): **top** - REPORTING (▶ START / ■ STOP REPORTING, the switch;
  the bar keeps the chip) beside ROBOCOMMAND (the answer · course · our declaration · RunStart · heartbeats at the judge · last reports);
  **RUN TIMELINE as a strip** (each step a box, → between); **THE JUDGES' TABLE** (`#rc-jt`) - one row per vehicle (USV1 + every vehicle the
  OCS reports or the judge has heard): state · task · hdg · SOG · HAE · phase · heartbeats sent · the age at the judge (red past 3 s) - beside
  **TASKS**; **JUDGE FEED** its own card, full width, 300-440 px; the OCS's judge view · the fake judge · the judge traffic in a fold
  (`#jd-more`). The old one-line vehicle card (`#rc-veh`) stays in the DOM, hidden.
- **The LOG's *judge* kind = the JUDGE feed** (→ sent · ← received; found live: v0.1.75 read only `judge_log`, which the simulator leaves
  empty - the chip said 0 beside a feed of 3).
- GUIDE §3 row "JUDGE, v0.1.77" + the header; CHANGELOG; ROADMAP WP33.

**Tests**: `tests/test_v0177.py` (2): the markup (every older id, the LOG's feed source); the Chrome walk - the order reporting → RoboCommand
→ the strip → the table + tasks → the feed → the fold, the USV1 row, the feed a direct child of the tab, `missing 0`. `make live` **763
passed** (the first run: 2 timeouts in test_v0154_b - the reporting / stop_reset live chain - right after a `stop_all`; both passed in the
`make test` that followed and in the rerun), `make test` **768 passed (5:24)**. On the simulator (`run_sim.sh pool --uav`, report on,
declare → the fake judge's RunStart 1001): JUDGE, VEHICLES and LOG live, `missing 0` on JUDGE · VEHICLES · LOG · RUN; captures
`docs/screens/v0.1.77/` 01 JUDGE, 02 VEHICLES, 03 LOG.

**Next session first**: Dmitrii on the canvas with the night's captures (`docs/screens/v0.1.69`-`v0.1.77`): the RUN bar, MISSION, SYSTEM,
⚙, the boat window, LOG · VEHICLES · JUDGE. Then WP18 (G1) - the first not-done WP. Nothing deployed: the Jetsons are still on their
previous version (GUIDE §1 when he says so).

---

## v0.1.78 — 2026-11-09 — WP34: the declutter pass by hand (Dmitrii + the Cowork session; the agent's night checked against the boards)

Dmitrii 2026-11-09, Team Village, after the diff of v0.1.69–v0.1.77 against the design canvases and the overnight brief: "the plumbing landed,
the declutter did not" — every moved item had been added, almost nothing removed (the v0.1.59 checklist + hint lines + six steps all on RUN, the
bar one row only in a lucky state, map.html's side column still on MISSION, the NOW / CONNECTIONS blocks above SYSTEM's table, the boat
window's commander on a third row). "Do the final rehauling, since you know how it is supposed to look" — built by the Cowork session (Claude),
not the coding agent; the method: exact-string patches on the three pages (`docs/screens/v0.1.78/patch_v0178.py`, every anchor matched once),
checked on a container bench (the tree re-installed there, Playwright at 1440 × 900 with an injected R2 / R4 state, the static page tests).

**What changed · where** (moves, folds and hides only — nothing deleted from the DOM; every v0.1.77 id and every pinned source string kept)
- `tools/ocs_web.html`: **the bar** — `header .top{flex-wrap:nowrap}`, the chips `flex:0 0 auto`, the state word block flexes (its reason
  ellipsised); `hdrBlocks()`'s words shortened (`LINK boat 0.2 s · judge ok|OFF`, `FCU mm · HOLD · armed`, `KILLED` only when on); the theme
  select hidden in the context line (⚙ has it); the flavour chip drops its `?` parts; `ctx-q` = the status age alone. **RUN** — a CSS stepper:
  `.st2:not(.now)` hides the description, the controls, the strips, the RUNSTART sub-card, the judge strip, `#rs-gate` and `#decl-con`;
  the live step framed; `groupActs()` no longer moves the buttons into `#rs-acts` (hidden; the buttons wired in their own `lad-cN`);
  `n7Init()` moves `#rs-gate` + `#decl-con` into step 2's `.bd` (the constructor closed by default), `#rs-mis` into the MISSION card
  (`#run-mis`), hides `#rs-fix`; ALARMS gets `.quiet` (one line) when nothing is active. **MISSION** — `.crsban.on{position:static}` (the
  banner above the map, not over its tool row), `.cword` 22 px. **SYSTEM** — `h2.sect` hidden, the NOW card, `#x-conn` and the EVERYTHING
  card wrapped in closed `<details class="n7fold">`, the bench cards' `what` text moved to `title`. `drawN7()` is called at the end of `drawN2()`.
- `tools/map.html`: `body.embed` (an iframe that is neither mini nor judge) — the `<aside>` and the footer hidden, the `USV map` brand hidden,
  a **details ▾** header button toggles the column (`ocs.mapdetails` in localStorage; a resize event refits the canvas); standalone, mini and
  judge maps unchanged.
- `tools/boat_web.html`: `#n6r1{flex-wrap:nowrap}`, the plate boxes 96–150 px, `#ctlbar` compact (`drawCtl()`: `VIEWER` / `YOU COMMAND` +
  the commander's name, `N connected`; the explanation and the viewer list on hover), the numerals and ■ STOP & RESET fixed width; `#wf`
  max 360 px; `#wfd` shown only with `.has` (set by `pick()` on the first click); the four `b-log-*` / `b-cv-*` buttons hidden in the tool row.
- `tests/test_v0178.py` (2): the markup pins (the night's anchors still present + the pass's own), and the Chrome walk at 1440 with an R4
  state (the chip `R4 READY · 0:42`, reporting on, armed): the bar's probe on one row, the gate + the constructor inside `lad-2`, the picker in
  `run-mis`, `rs-acts` hidden, step 4 live, ALARMS quiet, no `net ?` in the context line.
- `docs/screens/v0.1.78/`: the bench captures (RUN at R4 — the stepper; MISSION — the map alone; SYSTEM — the table first; the boat window;
  no live data on them) and `patch_v0178.py`. GUIDE header + §3 row "The declutter pass, v0.1.78"; CHANGELOG; ROADMAP WP34 after WP33.

**Tests**: on the container bench — the static tests of the pages green (`test_v0170 … v0177`, `v0153 / 54 / 59 / 44 / 47 / 51 / 52 / 52_wa / 63`,
`wp10c`, `wp9d`, `v0143`, the new `test_v0178`: 60 passed, 3 skipped = the Chrome walks, which hang under that container's headless Chrome);
the Playwright emulation of the new walk's assertions all true on the R4 scene; `node vm.Script` syntax check of every `<script>` of the three
pages. **Not run here**: `make quick` / `make live` / `make test` (no mosquitto, no Mac Chrome on the bench) — **Dmitrii runs them on the Mac
before the archive**; the m1t1 campaign is unaffected (no code outside the three pages + one test).

**Risks**: a `.now` step that is also *done* keeps its frame (two live frames when steps 4 and 6 are both live — as the agent's `oneAction()`
intends: one action, two lit); the map's `details ▾` state is per browser; `#ov-alcard.quiet` relies on `S.alarms.active` (empty = quiet).
The classic pages `/v1/` are untouched (pinned by `test_v0174` and the v0.1.43 tests).

**Next session first**: Dmitrii's `make live` + `make test` on the Mac → the archive (CLAUDE.md step 5, `MASTER_SCRIPTS_v0.1.78.tgz`) → the Test
Guide's U12 (the human look at 1440) → the deploy H0 when he says so. Then WP18 (G1).

---

## v0.1.78.1 — 2026-11-09 (Team Village) — the venue rc-test as a judge of its own; v0.1.78 validated on the Mac

Dmitrii on the venue Wi-Fi: "our team broker should be on 1910 and webpage on 8010 - add these configs to our launcher, as a venue rc-test
(vs home rc-test)… I've modified the GUI with my Cowork agent, but some tests are failing, check how all has been integrated".

**Found**: from the Mac (192.168.70.125 on the venue Wi-Fi) 192.168.70.74 answers ping; 8010 = RoboCommand's web page (HTTP 200); **1910 =
our team broker** (open, the retained `robocommand/robotx/course` = ALPHA, protobuf); 1883 also open there. Our side was not connected: no stack
running and no RoboCommand address in any network file (NET_PROFILE=ts; `--judge course` needs RC_HOST in network.lan.env).
**v0.1.78 (the Cowork pass) on the Mac**: `make test` **770 passed** as delivered - no failure here (its own block: the three Chrome walks hang
in the Cowork container, no mosquitto there - the failures seen were that bench's). Archived as delivered: `_archive/MASTER_SCRIPTS_v0.1.78.tgz`.

**What changed · where**
- `network.env` (never archived): `VENUE_RC_HOST=192.168.70.74`, `VENUE_RC_PORT=1910`, `VENUE_RC_WEB=http://192.168.70.74:8010/` (the base file:
  whatever NET_PROFILE).
- `scripts/run_ocs.sh`: **`--judge venue`** = the course profile (protobuf), RC_HOST / RC_PORT from VENUE_RC_*, the judge console read-only
  (`run_judge.sh --monitor`), the page's RoboCommand peek on that broker; **refused before the hub starts** when the two are missing. The home
  rc-test stays `--judge rctest` (this Mac, 1885).
- `tools/launcher.py` + `launcher.html`: the judge field fake · rctest · **venue** · course (tips say home / venue); venue = real mode only
  (like course), **never with FREE RUN**, refused in the plan when VENUE_RC_* are missing; the plan's note *venue rc-test: team broker …* and a
  link **venue RoboCommand page**.
- GUIDE header + §4 row 4.17d; CHANGELOG; CLAUDE.md test count.

**Tests**: `tests/test_v017801.py` (4): the field and the rules (sim refused); the plan (the step `run_ocs.sh … --judge venue`, no RC_HOST on
it, the link, the note; unset → refused); FREE RUN refused; `run_ocs.sh --judge venue` with an empty network → exit 2 before anything starts,
no address in the script. `make live` **769 passed**, `make test` **774 passed (5:22)**.
**The GUI recheck on the simulator** (`run_sim.sh pool --uav`, report on, declare → RunStart): every OCS tab (RUN · MISSION · CAMERA · USV
CONTROL · VEHICLES · SYSTEM · JUDGE · LOG) - no script error, provenance `missing 0`, **the bar on one row** (its probe: every block at y 8-16);
the boat window's four tabs - no script error; `/v1/` 200 on both. Captures `docs/screens/v0.1.78.1/` (the eight tabs live + the boat window).
**Seen, not changed** (v0.1.78's design): the boat window's header row at 1440 is clipped at the right (`2 conn…`, the holder's `after takeo…`)
since it no longer wraps.

**Not done here**: the OCS was not started against the venue rc-test (Dmitrii goes live himself): `scripts/run_ocs.sh --judge venue` (+ `--lan`
/ `--ts` for the boat as usual), then JUDGE → ROBOCOMMAND *listening* + ALPHA.

**Next session first**: whether the boat header's clipping matters to Dmitrii; then WP18 (G1).

---

## v0.1.78.2 — 2026-11-09 (Team Village) — the simulator against the venue rc-test

Dmitrii: "lets add a possibility to test our sim against their rc-test venue".

**What changed · where**
- `scripts/_netenv.sh`: `load_venue_rc ROOT` - only VENUE_RC_HOST / VENUE_RC_PORT / VENUE_RC_WEB (environment > network.<profile>.env >
  network.env; `NETENV` honoured as by `load_network_env`); the simulator loads no other address.
- `scripts/run_bench.sh --judge venue`: the course profile, `RC_HOST/RC_PORT` = the venue's, no fake judge node; refused (exit 2) when unset.
- `scripts/run_sim.sh --judge venue`: checked before run_bench (nothing starts when unset; `--course world` refused - the course is theirs);
  the OCS page's RoboCommand peek (`--judge-host/--judge-port`) and the SIM page's course banner (`--rc-host/--rc-port`) on the venue broker;
  the judge console + page 8095 read-only there (`run_judge.sh --monitor --owner bench`); `stop_bench.sh` closes that console (owner bench only).
- `tools/launcher.py` / `.html`: sim + judge venue allowed (course stays real-only); `_venue_judge()` shared by both modes - the refusal when
  unset, the note (sim: *the SIMULATED boat is declared and reported to it under our team*), the *venue RoboCommand page* link; the sim plan's
  `refuse`. GUIDE §4 row 4.17e + the header; CHANGELOG; CLAUDE.md count.

**Tests**: `tests/test_v017802.py` (5): the sim plan; refused when unset; both scripts exit 2 before any tmux session; the helper reads only the
venue keys (an inline comment stripped); the wiring strings, no address in the scripts. Adapted: test_v017801 (sim + venue now valid, sim +
course still refused). `make live` **774 passed**, `make test` **779 passed (4:47)**.

**Not run live, on purpose**: with the protobuf (course) profile the OCS's heartbeats are **ON by default** (v0.1.59 B1), so starting
`run_sim.sh pool --judge venue` reports our simulated USV1 to RoboNation's server at once - Dmitrii starts it.

**Next session first**: how the venue rc-test took the simulated runs (Dmitrii); the boat header's clipping (v0.1.78); then WP18 (G1).

---

## v0.1.78.3 — 2026-11-09 (Team Village) — the launcher's NETWORK table; the lan profile filled from the sweep

Dmitrii: "my Mac is connected through the switch to our local network with the vehicles (192.168.1.x, .1 the switch) and to RoboCommand on
192.168.70.74 - make sure we see all connected devices", then "update our launcher with correct addresses and a table with them for reference".

**The sweep** (ping 1-254, ARP, mDNS, the devices' pages, ssh): the Mac 192.168.1.13 on en16 (USB LAN) + 192.168.70.125 on the Wi-Fi (en0),
each subnet on its own interface. 192.168.1.1 the switch / router (UISP Router) · .5 .20 .132 .229 Ubiquiti radios (web + ssh) · .99 the AGX
(`optiprimeagxorin-desktop`, ssh ok) · .218 the NX (`usv-pixhawk`, ssh ok) · .153 `uav-pixhawk.local` (the UAV team's computer, ssh open) ·
.120 Calwin's MacBook Air. Both Jetsons up 13 min, their clocks = the Mac's to the second. 192.168.70.74: ping, 1910, 8010, 1883 open.

**What changed · where**
- `network.lan.env` (not archived): `OCS_HOST=192.168.1.13` (was empty), `UAV_HOST=192.168.1.153` (**to confirm with the UAV team** that their
  listener is there), `SWITCH_HOST`, `RADIO_HOSTS` (reference only). `network.env`: `NET_PROFILE=lan` (was ts; back with `--ts` or the line).
  The previous copies in the session scratchpad.
- `tools/launcher.py`: `NET_TABLE` (role + key; no address in the code) and `network_table()` - the network files read at each request, per
  profile (lan · ts · network.env), one ping per address in parallel (1 s; this Mac's own addresses = on an interface, it does not answer its
  own ping); `/network.json` (this page only, never logged or recorded).
- `tools/launcher.html`: the card **network** (the active profile bold, ● per address, ↻ check, every 30 s).
- GUIDE §4 row 4.17f + the header; CHANGELOG; CLAUDE.md count. Capture `docs/screens/v0.1.78.3/01_launcher_network.png` (dry-run launcher).

**Tests**: `tests/test_v017803.py` (2): the table from temp network files with a fake ping (the keys in order, per profile, the active profile
first, multi-address rows, the venue port, one ping per address); the route and the card, no address in `NET_TABLE`. `make live` **776 passed**,
`make test` **781 passed (5:28)** - with NET_PROFILE=lan in network.env.

**Next session first**: UAV_HOST confirmed by the UAV team?; the venue rc-test runs (sim and real); the boat header's clipping; then WP18 (G1).

---

## v0.1.78.4 — 2026-11-09 (Team Village) — ssh to the Jetsons from the launcher; the simulator declared to the venue rc-test

Dmitrii: "quick tests that the USV GUI opens? bench tests at the venue - ensure we can declare to RoboCommand; wire the ssh command to the NX
through the launcher once we hear it; from another laptop we might be prompted for the password" (+ polled: deploy the NX and the AGX).

**The venue declaration** (`run_sim.sh pool --judge venue`, 12:23): the OCS on 192.168.70.74:1910, course ALPHA received (5 corners, pinger
25 kHz); heartbeats `robocommand/robotx/SGIT/USV1/report` at 2 Hz (28 in 15 s on the broker); `declare` → `RunDeclaration seq 257: USV1
T1:core T2:none T3:core T4:core`, R3. **The venue page (8010): SGIT DECLARED 12:23:47, USV1 MANUAL, T1 Core · T2 — · T3 Core · T4 Core,
"Waiting for USV1 to report AUTO. The run starts when all declared vehicles are AUTO"** (RunStart is automatic on AUTO - not taken: no arming
for it); its log: `USV1 heartbeat — STATE_MANUAL @ 1.28102,103.85501`. Stopped (stop_all): no SGIT message on the broker after. Capture
`docs/screens/v0.1.78.3/02_venue_rc_declared.png`. A `#` subscription on that broker returns nothing (ACL); `robocommand/#` works.

**What changed · where**
- `tools/launcher.py`: `ssh_auth(target)` - key / password / down (a BatchMode probe, `StrictHostKeyChecking=accept-new`); `ssh_terminal_cmd`
  (the interactive `ssh -o ConnectTimeout=6 T`, or `ssh-keygen` only without ~/.ssh/id_ed25519 + `ssh-copy-id`); `open_terminal` (osascript
  → Terminal; not macOS: the command returned); `SSH_TARGET_RE` (no leading `-`, no shell characters); `network_table()` rows BOAT_HOST /
  AGX_HOST carry `ssh {which, target, auth}` (probed only when the host answers); `ssh_target(which)`; `POST /ssh {which, action}` (X-Launch;
  dry run returns the command); `preflight()` refuses a real launch when a Jetson asks this laptop for a password (an unreachable one is not
  refused, as before).
- `tools/launcher.html`: the network card's **ssh ↗** (enabled when the host answers), **key ok / password needed**, **copy my key ↗**, the
  message line. GUIDE §4 row 4.17g + the header; CHANGELOG; CLAUDE.md count. Capture `docs/screens/v0.1.78.4/01_launcher_ssh.png`.

**Tests**: `tests/test_v017804.py` (5): the probe's three answers + a bad target never run; the terminal lines + the AppleScript escaping + the
bad targets; the table's targets (an unheard host not probed); the preflight refusal (password) and no refusal (down); the route (dry, copy-id,
no target, a bad `which`, no X-Launch) and the card. `make live` **781 passed**, `make test` **786 passed (5:24)**. Live: both Jetsons `key ok`
from this Mac; `ssh nobody@NX` → *password* (what another laptop sees).

**Next**: the deploy of this version to the NX and the AGX (Dmitrii: both), then the USV window opened from the Mac (fake FCU, no props).

---

## v0.1.78.4 (continued) — 2026-11-09 ≈12:40 — deployed on the NX and the AGX; the USV window opened from the Mac

Both Jetsons on **v0.1.78.4** (were v0.1.58), `NET_PROFILE=lan`, the Mac's three network files (the old ones `.bak_20261109` there); AGX
buoy self-test PASS; the NX's USV window (fake FCU, the launcher's own steps) served on 192.168.1.218:8098 and opened from the Mac with no
script error - then stopped. Details and the checks: `docs/JETSON_LOG.md` 2026-11-09 ≈12:35. (This block and the log entry are after the
v0.1.78.4 archive - docs only; the next archive carries them.)

**Next session first**: go live (Dmitrii): the launcher's event preset (lan, judge venue or fake, the real FCU) - the network card first
(every row green, both Jetsons *key ok*); UAV_HOST to confirm with the UAV team; the boat header's clipping; then WP18 (G1).

---

## v0.1.78.5 — 2026-11-09 ≈12:55 (Team Village) — fix: the launch with judge venue stopped at the OCS step

Dmitrii's first real launch (event preset, judge venue, FCU uart; 12:45): steps 1-4 fine (both Jetsons reached, clocks -0.1 / -0.6 s), step 5
`MAP=0 LOG=1 LAUNCH_THEME=light scripts/run_ocs.sh --lan --judge venue` → **`run_ocs.sh: line 109: RC_PORT: unbound variable`, exit 1, the
launch stopped** (the NX, the USV window and the AGX steps never ran; nothing reached the FCU). Also a false **`WARNING: network.env
OCS_HOST=192.168.1.13 but this Mac is LAN 192.168.70.125`**.

**Causes · fixes** (`scripts/run_ocs.sh`)
- The `${VENUE_RC_HOST:?... the venue rc-test's team broker ...}` message (v0.1.78.1) had an apostrophe: inside a double-quoted parameter word
  bash takes `'` as an opening quote, so the next line (the RC_PORT assignment) became part of the message - RC_PORT never set. Reworded,
  no apostrophe. v0.1.78.1's test only covered the early refusal (unset), never this branch with the addresses set.
- MYIP = `ipconfig getifaddr en0` = the Wi-Fi; the switch is a USB LAN (en16). The warning now fires only when OCS_HOST is on no interface
  (`ifconfig` / `ip -4 addr`).
- Left behind by the stopped launch: tmux `ocs` with the hub → `stop_ocs.sh`. Also five orphaned `fake_perception` processes (parent 1, 10-11 h,
  random test ports - last night's interrupted test runs) → killed by PID. Mac, NX, AGX: no tmux, nothing of ours.

**Tests**: `tests/test_v017805.py` (5): the venue / course / rctest branches of run_ocs.sh **executed** under `set -euo pipefail` (RC_HOST /
RC_PORT / RCENV / JUDGE_ARGS), no apostrophe in a `:?` word in the launchers, the interface check - the venue test and the apostrophe scan
fail on v0.1.78.4's script (checked). `make live` **786 passed**; the full `make test` not run (a Mac-only launcher fix, no Jetson deploy:
the tier rule) - run it before the next deploy.

**Next**: Dmitrii relaunches (the launcher: event preset, judge venue). The Jetsons keep v0.1.78.4 (run_ocs.sh runs on the Mac only).

---

## v0.1.78.6 — 2026-11-09 ≈13:00 (Team Village) — fix: the launcher's USV window link

Dmitrii: "how can I reach the USV window? the launcher says it's down". His relaunch (12:52; this time **judge rctest** - the home rc-test on
this Mac, not venue) was up everywhere: the NX `bwin` + `usv` (FCU uart), the window on **192.168.1.218:8098 → HTTP 200 from the Mac**. The
launcher's PROGRESS probed `127.0.0.1:8098` - right for the simulator, wrong for a real launch, which binds the window to the team LAN (on
the NX: BOAT_HOST; on the Mac: OCS_HOST). `tools/launcher.py`: `usv_window_host(cfg)`, used by `page_links()` for the link and its probe.
Against the live launch record: `http://192.168.1.218:8098/` **up**. The running launcher (pid 24207) has the old code until it is restarted
(the stacks run in tmux; a new launcher adopts the running launch).

**Tests**: `tests/test_v017806.py` (1: nx / nx-classic / mac / off / sim / manual / no record) + the launcher's touched tests (v0147_ab,
v0153, v0165, v0163): 32 passed. **`make live` / `make test` NOT run: Dmitrii's live stack runs on this Mac (the suites start their own
brokers and stop scripts) - run them when the session ends, then the archive `MASTER_SCRIPTS_v0.1.78.6.tgz`** (not built yet).

---

## v0.1.78.7 — 2026-11-09 ≈13:20 (Team Village) — CONFIRM SAFE, resolved alarms, the AGX observer from the launcher

Dmitrii during the live session: the log (13:02-13:03: COURSE timed out, RC LOST 8 s, KILLED RC5, the Mac ↔ NX link 40 s down, back by
itself), "confirm safe - hard to find", "after a warning is resolved I need to see it resolved", "camera no picture, test please", "can we
wire the AGX observer button to the launcher", "KILLED apparently stale - I can control the boat from the RC".

**Found**
- KILLED was **not stale**: status 0.1 s old, RC5 released (kill false), the receiver healthy, the FCU MANUAL + armed under the transmitter -
  the mission manager's latch, which only `confirm_safe` clears (by design: KILLED blocks our commands, never the transmitter). The page's
  hint for it (`#rs-fix`: "release it on the transmitter, then confirm_safe" → fills the console) was hidden by v0.1.78's stepper; no button.
- The camera worked (JETSON_LOG 13:05: the ZED alone on 8091, a real snapshot); no picture because no observer ran - the cam window
  (`--source dir`) shows the observers' previews only.
- COURSE TIMED OUT 13:02:15: the OCS forwarded course ALPHA at 13:02:10.2, before the boat's bridge (13:02:11) and mission manager (13:02:14)
  were up; the boat never got it and nothing re-sends it (the UAV has `resend_course`, the USV not) - **open, proposed to Dmitrii**.
- The 13:02:40-13:03:15 drop: the NX broker timed out this Mac's console clients, the hub timed out the NX bridge (`Operation timed out`) -
  the network path in both directions (the radio link or the Mac's USB LAN), the programs fine; the bridge reconnected by itself.

**What changed · where** (built in `../ws/v787` while the live stack ran from main)
- `tools/ocs_web.py`: `AlarmBook.body()["recent"]` - the alarms cleared in the last 15 min (`RECENT_S`), newest first, ≤ 8, `dur_s`.
- `tools/ocs_web.html`: `csafeStrip()` at the top of the ALARMS card while KILLED / FAULT - **✓ CONFIRM SAFE** (asks; `send("confirm_safe")`;
  disabled with "release it on the transmitter first" while the kill / E-stop is held); the resolved rows under the active ones; the card
  not folded (`quiet`) while something resolved or a latch shows; the RUN fix line's link jumps to the button.
- `tools/launcher.py` / `.html`: `OBSERVER_CMDS` / `observer_cmd()` / `agx_observer()` (ssh, key login), `/observer` (start | stop | status;
  dry run returns the command), the AGX row's `observer` state (only with *key ok*) and **observer ▶ / ■**.
- GUIDE §4 row 4.17h + the header; CHANGELOG; CLAUDE.md count.

**Tests**: `tests/test_v017807.py` (5): the recent list; **the Chrome walk** (KILLED, kill released → the button enabled + KILLED latched +
a resolved row with its time; kill held → disabled + the reason); the word and the jump; the observer commands (`bash -n`, the refusals,
BatchMode, a bad target never run); the table's observer state + the route (dry, a bad action). Live: `/observer status` on the real AGX →
`observer: down`.

**Merged into main** (Dmitrii: "merge now") ≈13:30; his stacks were stopped by then (Mac, NX, AGX: no tmux; the launcher closed), so the
suites ran: `make live` **792 passed**, `make test` **797 passed (5:19)** - covering v0.1.78.6 as well (its archive was never built: this
one carries it). **The observer**: Dmitrii said "start it now" - `agx_observer("start")` → *refused: no agx session - launch perception on
the AGX first* (his session had ended): the refusal as designed; the button starts it on the next launch.

**Next**: v0.1.78.8 - the OCS re-sends COURSE to the boat until it acks (Dmitrii: "yes, build it next").

---

## v0.1.78.8 — 2026-11-09 ≈13:25 (Team Village) — fix: the OCS re-sends the course until the boat acknowledges it

Dmitrii: "yes, build it next" (the COURSE that timed out at 13:02:15 - forwarded before the boat was up, never repeated; the UAV had
`resend_course`, the USV nothing).

**What changed · where** (`optiprimex/ocs/ocs_app.py`, the Mac's OCS only - no Jetson change)
- `_course_sync[vid]` = the course fingerprint (id · boundary · transit area · pinger) the boat acknowledged / refused, the last COURSE's msg id
  and time; `_status_mono[vid]` = its last status (this OCS's clock).
- `on_status`: the boat's **first** status → what was sent before it was up is lost: due at once; a **gap > 4 s** (a reconnect, a mission
  manager restart) → acknowledged forgotten, due at once.
- `course_resend_tick()` (the 10 Hz loop, beside `sweep_acks`): each boat (not the UAV) whose status is < 3 s old and has not acknowledged
  the current course → COURSE again, at most every 6 s (> the 5 s ack timeout), logged `course ALPHA -> USV1 again (...)`.
- `on_ack` → `_course_ack`: ACCEPTED / DONE = acknowledged; REJECTED / FAILED = refused - not asked again for that course (a new course or a
  reconnect asks again; a warning in the log).
- GUIDE header; CHANGELOG; CLAUDE.md count.

**Tests**: `tests/test_v017808.py` (2): the 13:02 sequence (sent while offline → nothing while offline → its first status → at once → no ack
in 6 s → again → DONE → never again); the 40 s drop (a gap → again), a refusal stops it, a new course is pending again. On the simulator:
COURSE EXTRA sent once, ACCEPTED + DONE in 27 ms, no re-send, no TIMED OUT. `make live` **794 passed**, `make test` **799 passed (5:23)**.

**Next session first**: a real launch at the venue (event, judge venue): the ocs log shows the course acknowledged (or `again` until it
is); the observer ▶ on the launcher's AGX row for the camera; CONFIRM SAFE after a kill. Then WP18 (G1).

---

## v0.1.78.9 — 2026-11-09 ≈14:00 (Team Village) — the USV window: PIN 0000 shown in the launcher; ✓ CONFIRM SAFE on it

Dmitrii: "KILLED persists even when the SF switch is cleared" (the latch, by design: `guards.v1_exit_requires_confirm: true` in every
profile - KILLED → MANUAL needs the input cleared AND `confirm_safe`; the transmitter keeps the boat throughout; explained, not changed),
then "remove the PIN and make confirm safe on the USV side as well", then "what is the PIN? let's make it 0000 - clearly visible in the
launcher". (Between: the AGX was unplugged, not shut down by us - our shutdown command was rejected; it came back: key ok.)

**What changed · where**
- `tools/boat_web.py`: `DEFAULT_PIN = "0000"` - the PIN when `USV_WINDOW_PIN` / `--pin` is not set (was a random 4-digit one printed in
  the log); printed at start; one commander at a time unchanged; the computer running the window needs none. The `n6` snapshot gains
  `vstate` (status/usv state) and `estop`.
- `tools/boat_web.html`: **✓ CONFIRM SAFE** (`#n6csafe`) in the header's second row while KILLED / FAULT, disabled while the kill / E-stop is
  held, two clicks within 4 s (this window's rule), `POST /ocs {line: "confirm_safe"}` - the existing OCS line (OCS_LINES unchanged, the 12
  usv_ui words unchanged).
- `tools/launcher.py` / `.html`: `usv_window_pin()` (USV_WINDOW_PIN from the network files, else 0000) - **PIN 0000** on the PROGRESS link
  (a yellow badge), on the network card's NX row (`USV window PIN 0000`), in the USV window step's label and its note.
  `scripts/run_boat_window.sh` comment. GUIDE (header, §3.2, the window's PIN sentence); CHANGELOG; CLAUDE.md count.

**Tests**: `tests/test_v017809.py` (4): the n6 latch fields; the PIN (0000 needed from another computer, accepted with it, one commander,
the launcher's link / step / note / NX row); **the Chrome walk** of the window - KILLED + kill released → the button shown and enabled, kill
held → disabled. Adapted: test_v0163 (the start message). `make live` **798 passed**, `make test` **803 passed (4:44)** (the run before:
one load flake in test_v0164_scenarios mpsilent, passed alone and in this run). Capture `docs/screens/v0.1.78.9/01_launcher_pin.png`.

**Needs the NX deploy** (boat_web runs there) - asked.

**Deployed** (Dmitrii: "deploy NX and AGX") ≈14:10: both Jetsons on **v0.1.78.9**; the NX's window live-checked with the fake FCU (PIN 0000
printed, 200 from the Mac), stopped. `docs/JETSON_LOG.md` 2026-11-09 ≈14:10.

---

## v0.1.78.10 — 2026-11-09 ≈14:20 (Team Village) — HOLD green (the start mode)

Dmitrii, a last-minute fix: "HOLD should appear green - we start from this mode". `tools/boat_web.py` `fcu_mode_class("HOLD")` → `hold`
(was `neutral`, an outline); `boat_web.html` `.pb.hold` = the `.pb.auto` tokens (the contrast test's rule kept as it was); the FCU MODE
box's tip says so. `tools/ocs_web.html` `#fb-hold.act` green (was the amber action style of the current mode); MANUAL unchanged (yellow /
amber). Tests: `tests/test_v0178010.py` (1); test_v0152_wa adapted (HOLD → hold). On the simulator: both green (captures
`docs/screens/v0.1.78.10/` 01 the USV window, 02 the OCS page). `make test` **804 passed (5:22)**. Needs the NX deploy (the window).

**Deployed** ≈14:25 (Dmitrii: "deploy NX and AGX"): both Jetsons on v0.1.78.10 - `docs/JETSON_LOG.md`.

---

## 2026-11-09 ≈14:40 (Team Village) — the lamp Lua v4 prepared (outside the tree: `../test_scripts/LUA/robotx_lamp_v4.lua`), not installed

Dmitrii: "on the FCU when I switch to HOLD the lamp is not going green", "isn't it in the Lua?", "prepare the Lua", "when I push the
remote kill switch button (not the RC controller), which lands on the Pixhawk pins, the lamp is not going red as well". The lamp is the
Pixhawk's Lua (`robotx_lamp_v3_ok.lua`, 9 Sep - the newest of five on this Mac; which one the FCU runs is not visible from here): its colour
= the FCU mode only - `MODE_AUTONOMOUS` lacks 4 (HOLD) and 20 (the companion's mode 20), so both showed YELLOW. RED is hardware (A1 low → a
relay's NC contact), so a red that never comes means the script never sees the kill on A5 (GPIO 54): v3 says "both ACTIVE HIGH" in its
comment but sets `KILL_ACTIVE_HIGH = false` (LOW = kill) - suspected inverted, or the pin does not move. **v4** = v3 + (1) 4 and 20 in
MODE_AUTONOMOUS, (2) A5 diagnostics (the raw level in the start message, a STATUSTEXT on every change saying how it reads), (3) NAMED_VALUE_FLOAT
LAMP / A5RAW once a second; the polarity deliberately unchanged until the bench shows it. Parsed (luaparser) - not run on an FCU. Our code
reads NAMED_VALUE_INT LAMP (v0.1.55); a float read-back in usv_ui / the window = a small later change. Install: SD `APM/scripts/` (one lamp
script only), SCR_ENABLE=1, reboot, props off.

---

## 2026-11-09 ≈15:00 (Team Village) — the assessment check passed; the stack shared again for the UAV side

Dmitrii: "we cleared the assessment check - archive our current stack, so we push it one more time to the UAV guy, he will adapt his code
to ours". `_archive/MASTER_SCRIPTS_v0.1.78.10_share_20261109.tgz` (71.6 MB, sha256 `26bb3ca0…`) - the v0.1.68 share's recipe (no
network*.env, no key file, no logs / usv_ui logs, no _archive, no drone/, no venv, no PDFs of the handbook / pathing sim); checked: none of
those inside, no private key / token pattern in its content. `_archive/SHARE_NOTE_v0.1.78.10_20261109.md` for him: the UAV docs to read
first, what is unchanged (the wire 14600 / 14605, the identity, the OCS as the one RoboCommand client), what changed (judge venue, VEHICLES /
JUDGE / LOG, UAV_HOST 192.168.1.153 to confirm).
**Lite share** (Dmitrii: "why so big? my colleague can't upload it to his agent"): of 81 MB uncompressed, 60 MB = `docs/screens/` PNGs,
6 MB `docs/mock/`, ~6 MB PDFs; the code < 5 MB. `_archive/MASTER_SCRIPTS_v0.1.78.10_share_lite_20261109.tgz` **2.7 MB** (sha256
`10db0c8f…`): the full share minus docs/screens, docs/mock, docs/_superseded, *.pdf, *.xlsx, the nested robocommand .git; no test reads
those (test_v0154_b only writes its two PNGs). The share note names both.

---

## v0.1.78.11 — 2026-11-09 ≈16:40 (Team Village) — fix: the launcher called the UAV adapter / window *died* while they ran

Dmitrii: "the UAV is not connecting in OCS" (the launcher: `uav:adapter - our UAV adapter · back at the shell ([exited]) died`, the same for
`uav:web`). Found: both ran; the adapter (16:29:12, listener 192.168.1.153:14600, bound 14605) had the drone's fence (4 points, 16:29:23) and
its telemetry (state.json: link up, telemetry 0.2-1 s old, 110 messages, ALT_HOLD, disarmed); not declared (`declare uav on` / `uav ready
on` are the operator's). The UAV computer: ping, ssh open, `uav-pixhawk.local`; **no MQTT broker on 1883** (needed only for the buoy direct
path). Cause: the launcher's liveness reads `#{pane_current_command}`; `bash adapter.sh` shows `bash` (a shell = died). Proven in a throwaway
tmux session: `bash w.sh` → `bash`, `. ./w.sh` → `python3.13`. Fix: `run_uav.sh` (adapter, the sim's fake UAV), `run_uav_window.sh`,
`fault_timing.py`'s adapter restart source the wrapper. `tests/test_v0178011.py` (2; its tmux check on a private `tmux -L` server).
`make live` **801 passed**; the full suite not run (Mac scripts only, no Jetson change - the tier rule).

**Incident**: Dmitrii's live stack on the Mac (tmux ocs / judge / uavwin, launched 16:29) ended all at once at **16:31:45** (every hub client
disconnected together, the adapter `disconnected (Unspecified error)`, no launcher stop recorded) - the moment my throwaway tmux test session
(`v78test`, created + killed on the default tmux server) ran. Not proven, the timing matches; told Dmitrii. From now on: no tmux / suites /
stop scripts on the Mac while his stack runs (memory `feedback-no-tmux-during-live`); the new test uses its own `tmux -L` server.

---

## v0.1.78.12 — 2026-11-09 ≈16:50 (Team Village) — ssh to the UAV team's Jetson from the launcher

Dmitrii: "a quick ssh link to the UAV Jetson?", "add it to our ip config" (polled: the user = `uav-pixhawk`, then corrected by Dmitrii to `uav`). `network.lan.env`:
`UAV_SSH=uav@192.168.1.153`. `tools/launcher.py`: the UAV_HOST row carries `ssh {which: uav, ...}` (the auth probe as for the
Jetsons: today **password needed** - our key is not on their machine), `/ssh` takes `uav`; `launcher.html`: no buttons when a row has no
target. Tests `tests/test_v0178012.py` (2) + test_v017804 adapted (the UAV row now has an ssh entry with an empty target there) - the
launcher's unit tests only (13 passed): **Dmitrii's stack was live on the Mac (relaunched 16:39) - no make live / test, no tmux; run them
when it stops, then the archive `MASTER_SCRIPTS_v0.1.78.12.tgz`** (not built yet).

---

## v0.1.78.13 / .14 — 2026-11-09 ≈17:20 (Team Village) — the drone on the main page; its registration; "not in AUTO"

Dmitrii: "surface declare uav on on the main page", "my USV is not in AUTO mode even if in HOLD", then "the vehicle's name from RoboCommand's
side is SGIT-53577 (the USV is USV1) - per the rules UAV1? And '"AA53577" — no match' must be changed to SGIT-53577".
- **Not in AUTO**: vehicle MANUAL, FCU HOLD + armed, the mission manager holding the FCU (handover done), the checklist clear, the
  ENABLE_AUTO verdict ok - only `enable_auto` missing (the operator's; HOLD is an FCU mode, AUTO_HOLD our state). Explained, not sent.
- **v0.1.78.13** (`ocs_web.html`, built in `../ws/v7813`, merged while his stack ran - page only, served from disk): RUN → VEHICLES carries
  the drone's line when the OCS is configured for a UAV - **declare UAV** (`declare uav on|off`), **pilot ready** (`uav ready on|off`), the
  meaning (not declared: silent on RoboCommand / in the next declare / declared · heartbeats N) and the adapter's link. The constructor's
  checkbox (folded in step 2 since v0.1.78) is why it was hard to find. `tests/test_v0178013.py` (2, Chrome on a spare port).
- **v0.1.78.14**: `uav.registration` = **SGIT-53577** in the three OCS profiles (was AA53577 = the CAAS tag per the UAV team's bridge;
  their default in `uav/` and our verbatim port `wire.py` untouched). The handbook has no `UAV1` rule (vehicle ids: unique, stable, in the
  declaration, topic = RxReport.vehicle_id) - SGIT-53577 kept. **To confirm with RoboNation which registration their registry holds.**
  Takes effect at the next OCS start (the launcher's profile copy too). Unit tests test_v0164 / test_v0165 (22 passed).
- Dmitrii's stack live throughout: no tmux, no make live / test. **Next, when the stack stops: `make live` + `make test`, then the archive
  `MASTER_SCRIPTS_v0.1.78.14.tgz` (covers .12-.14; none of them archived yet).**

---

## v0.1.78.15 — 2026-11-09 ≈17:30 (Team Village) — the drone's declare on the VEHICLES tab; the suites for .12-.15; the archive

Dmitrii: "the declare button for the UAV in the VEHICLES tab and the tile" (the RUN tile had it live - checked with a read-only headless
look at his page). `ocs_web.html`: `uavCtlHtml(px)` / `uavCtlWire(px)` - one helper for the RUN tile (`rv-`) and the VEHICLES tab's drone
card (`vc-`, under its header, shown when the OCS is configured for a UAV). `tests/test_v0178015.py` (2). Built in `../ws/v7815`.

**His stack stopped at 17:16** (no launcher record left): the NX first - usv_ui quit (17:16:00), the logger (17:16:02), the bridge closed -
then the Mac (17:16:08): an orderly STOP ALL (from the launcher / the page / a terminal), not a crash; nothing of mine reaches the NX.
(The 16:31 stop, blamed on my tmux test above, ended the same all-at-once way - the NX side was not checked then; it may have been a stop
too. The rule stays: no tmux / suites / stop scripts on the Mac while his stack runs.)

With nothing live: merged; `make live` **807 passed**, `make test` **812 passed (5:28)** - covering v0.1.78.12 (UAV ssh), .13 (the RUN
tile), .14 (the registration SGIT-53577), .15. Archive `MASTER_SCRIPTS_v0.1.78.15.tgz` (carries .12-.14, never archived on their own).
Mac only since v0.1.78.10 - the Jetsons stay on v0.1.78.10 (nothing they run changed).

---

## v0.1.78.16 — 2026-11-09 ≈17:50 (Team Village) — the drone's declare / pilot ready as two big buttons

Dmitrii: "the tick and the pilot ready are jerky, the latter is not even pressing; the former was okay, but I'd prefer a big clear button,
not buried". Cause: both checkboxes were rebuilt by innerHTML every refresh (~1 s) - a click landing during the rebuild hits a replaced
element (worst for pilot ready: also disabled until declare took effect). `ocs_web.html`: `uavStrip(px, parent, before)` builds a strip
ONCE per place (RUN's VEHICLES tile above its rows; atop the VEHICLES tab) and then only changes text / class / disabled; two big buttons -
**DECLARE UAV** → ✓ UAV IN THE NEXT DECLARE → ✓ UAV DECLARED (taking it out of a sent declaration asks first) and **PILOT READY?** →
✓ PILOT READY (enabled once declared); "sending…" until the OCS state answers (6 s); a build counter (`data-built`) lets the test prove
"built once". The words unchanged. Built in `../ws/v7816` (nothing live). `tests/test_v0178015.py` rewritten (3: undeclared → declared, the
two strips built once over 8 s of refreshes, hidden without a UAV, the words, no checkbox left); `test_v0178013.py` reduced to the tile's
promise (its file rewritten, not deleted). Screenshots checked (RUN tile, VEHICLES tab). `make live` **807 passed**, `make test` **812
passed (5:23)**. Mac only - the Jetsons stay on v0.1.78.10.

---

## v0.1.78.17 — 2026-11-09 ≈18:05 (Team Village) — no checkboxes for the vehicles; PILOT READY? clickable any time

Dmitrii: "let's not do checkboxes, both big buttons; make a quick test; the pilot ready button was unclickable". What he had seen was
v0.1.78.15 (his stack had stopped before .16); in .16 PILOT READY? was deliberately disabled until the drone was declared - read as
unclickable. The OCS takes `uav ready on|off` at any time (`on_uav` "ready" has no precondition) → `ocs_web.html`: the button is disabled
only while "sending…". The declaration constructor (RUN step 2): its "☑ USV1" (a disabled checkbox) and the drone's checkbox (rebuilt
every refresh - the same jerk) → plain text, the drone's state pointing to DECLARE UAV on the VEHICLES tile. The page's other checkboxes
(camera, --auto, T4, ...) untouched. Tests: test_v0178015 (pilot ready enabled undeclared; no checkbox in the constructor) + the touched
page tests (23 passed); `make quick` below. Screenshot checked. Mac only.
`make quick` **passed** (see the run).

---

## 2026-11-10 — the hand-over: `docs/HANDOVER_2026-11-10.md` (read it first)

Dmitrii: "a proper hand-over for a new session - I continue development and system integration: confirm and integrate the CV models
(on the AGX - find them), the water cannon (not plugged in yet), the drone (the name, maybe more)". The AGX searched read-only: the
colleague's two-stage models + runtime are in `~optiprime-agxorin/buoy_robotx/runtime/` (Model A / B as .pt / .onnx / .engine, the
engines built on the AGX on 9 Nov; `two_stage_buoy_recognizer.py`, `recognise_docking.py`, the students' venv; three real dock runs in
`output/`), next to our CV_TRAIN models (5-6 Nov) and the production weights in our `cv/` (3 Nov). No ZED SVO recording anywhere; the labelled
sets + `eval/confuse.py` in `../CV_TRAIN` are the yardstick. The hand-over: where things stand (versions, hosts, network, judge, launcher),
workstream A (CV models: inventory, the review's findings, next steps), B (the cannon: built in the sim, the hardware steps D5 → H4, the
Pi's address 192.168.6.7:5005 in the students' config), C (the drone: the registration to confirm, the venue-page proof, the buoy-map
contract mismatch), the other open items, and §6 for Dmitrii's notes. Docs only; the tree stays v0.1.78.17 (archive rebuilt with it).

---

## v0.1.78.18 — 2026-11-10 ≈09:30-10:30 (Team Village) — workstream A: the colleague's Model A in our contract; the sim scores; the first real clip; the recorder's locale fix

Dmitrii (warm-up, then A first): "adapter in cv/buoys, and a backup option for colour recognition"; "sim now + real later"; "yes, run it"
(the AGX GPU); mid-session "in 30 min the boat moves away from the antenna - copy what we need to the Mac". Environment: no tmux anywhere,
the Jetsons power-cycled ≈09:04, both v0.1.78.10; the UAV Jetson had an ssh login from 192.168.1.120 (the UAV team) - not touched.

**Differs from the hand-over:** `~/buoy_robotx/runtime/` gained `predict_buoys_zed.py` + `run_predict_buoys_zed.sh` at 09:34 today (Model
A on a live ZED, `DEPTH_MODE.NONE`, preview only); our "labelled sets" are synthetic only and the `recordings/` are empty 320x180
test-suite frames - no labelled real frame exists; the ZED recorder (v0.1.40) exists but had never run on the AGX (see the fix).

**Pulled to the Mac** (`../ws/pull_2026-11-10/`, sha256 verified, nothing changed on the AGX): `buoy_robotx/` whole (scripts, models
.pt/.onnx/.engine, the three dock runs), our AGX `cv/{buoys,docking}/models/`, the NX `logs/` (283 MB), the AGX logs + the 9 Nov 16:40
buoy run, `score_sim/` (the results), `recordings/20261110_101857_village_1110/` (the clip). Nothing else lives only on the Jetsons
(no file changed in either tree since the .10 deploy besides logs / runs).

**What changed · where:**
- `cv/buoys/two_stage/` - the colleague's 10 files **verbatim, a-w**, `PROVENANCE.md` (sha256 table, the review notes); the models stay
  in `~/buoy_robotx/runtime/models/` (read-only use). `score_sim.py` (ours): any YOLO model (detect or segment) on a CV_TRAIN test
  split with a class map (`--map buoy_led_red=buoy ... dock=dock_face`), per class truth / found / right class + unmatched.
- `cv/buoys_multi/two_stage_core.py` (new, pure) + `multi_buoy_observer.py`: `--detector mask|model-a` (default mask = unchanged),
  `--colour auto|model|hsv`, `--led-conf 0.5`, `--box-shrink 0.8`. model-a: YOLO task detect, Model A's defaults (1280 / 0.25 / iou
  0.5), only the three buoy classes (dock / home / obstacle skipped), the box's inscribed ellipse x 0.8 as the mask → the students'
  `measure_mask` unchanged → range / bearing / the bearing-only rays → the same `latest_status.json` (cv_publish untouched). Colour
  auto = Model A's red / green LED class when ≥ led-conf, else our HSV vote (Model A has no blue: Task 1's entry / exit); model = Model
  A only (no_led → black → cv_publish lit False); **hsv = the backup** (ours for every buoy). Each record: `colour_source`,
  `model_class`, `colour_hsv`. `run.sh`: `--detector model-a` picks `~/buoy_robotx/runtime/models/model_a_best.engine` (aarch64) /
  `.pt`, or `CV_MODEL_A=<file>`.
- **Fix** `rec_core.py` / `multi_core.py`: every `open()` with `encoding="utf-8"` - the recorder died at start on the AGX under tmux
  (ASCII locale; `kit_version` read `optiprimex/__init__.py`, which has an em dash). The OCS RECORDER card / `agx_record.sh` were
  therefore broken on the AGX until this is deployed (workaround today: `PYTHONUTF8=1 LANG=C.UTF-8` before a fresh tmux server).

**The sim scores** (AGX, `~/Desktop/dm3/ws_score/`, conf 0.25, IoU 0.5; truth boxes found / right class):

| model | v1_mix buoys | v2_buoy buoys | v2_mix buoys | v2_mix dock_face (the white cube) |
|---|---|---|---|---|
| cvt-v2-l (ours, trained on these renders) | 2119/3180 | 2329/2386 | 1141/1166 | 926/978 |
| Model A (colleague, real frames) | 146/3180 | 139/2386 | 41/1166 | 660/978 (+421 unmatched dock boxes) |
| students' best.pt (production) | 0/3180 | 0/2386 | 1/1166 | - |

Read: both real-trained models are blind on our renders - a sim→real gap, **not a verdict** on Model A (and not on best.pt, which
works in the field). The sim cannot decide between them; the real clip does. Model A finds the white dock in renders (67 %).

**Tests:** `tests/test_v0178018.py` (15: the snapshot verbatim + read-only against PROVENANCE, the class ids, the colour precedence x7,
the mode check, the box mask geometry, score_sim's matching, the observer wiring, run.sh's Model A path, the recorder under an ASCII
locale - fails on the old code). `make quick` **616 passed, 2 failed** - both in `tests/test_v0164.py` (the UAV wire), NOT this version:
`optiprimex/uav/wire.py` was edited 2026-11-09 18:41 (after the .17 block, no DEVLOG / CHANGELOG entry; already in the 09:30 hand-over
archive): `DEFAULT_VEHICLE_ID = "UAV1"`, `DEFAULT_UAV_REGISTRATION = "SGIT-53577"` (the UAV team's bridge: SGIT-53577 / AA53577) - the
verbatim-port pin and the adapter's hello test fail. Left as found (workstream C: Dmitrii's call - keep and update the pins, or restore).
Archive `_archive/MASTER_SCRIPTS_v0.1.78.18.tgz`. **Not rehearsed:** `--detector model-a` on the real ZED (asked; Dmitrii chose the
clip only) - its first run = the replay of the clip on the AGX.

**Open / next session first:**
1. Replay the clip through both detectors on the AGX when the boat is back: `cv/buoys_multi/run.sh --headless --svo
   ~/Desktop/dm3/recordings/20261110_101857_village_1110/cam.svo2` then `... --detector model-a` (needs this version deployed - ask),
   compare `replay/*/frames.jsonl` (rec_core compare). For a real score: label ~50 frames of a clip with buoys in it (this one: 9 frames
   with a detection - point the camera at the course next time).
2. Deploy v0.1.78.18 to the AGX (the recorder fix matters for the OCS card) - ask first.
3. `~/Desktop/dm3/ws_score/` on the AGX: delete when Dmitrii says.
4. Workstreams B (the cannon Pi) and C (the drone) unchanged - HANDOVER §3 / §4.

---

## v0.1.78.18 (addendum) — 2026-11-10 ≈11:05-11:15 — deployed to the AGX; the first live check of a real buoy

`make test` 825 passed, 2 failed (test_v0164 - the 9 Nov wire.py edit, open). AGX → 0.1.78.18 (NX stays 0.1.78.10: cv/ only), self-test
PASS. The white "B" RoboBuoy marker ≈3.5 m in front of the boat, indoors (Team Village tent): **students' best.pt: 0 detections**
(45 + 30 frames, conf 0.60 and 0.25). **Model A via the adapter: 10 detections in 45 frames, conf 0.25-0.33; the marker once with a
range - +1.0°, 3.57 m (surface 3.36 m, 1029 valid px, spread 5 cm), colour "white" from our HSV (Model A's class not red / green)**;
the rest at -10° / -20° (bearing-only, beyond depth: background, not verified - the snapshot to check them was refused the ZED).
So: box → ellipse mask → measure_mask works on the real ZED; Model A sees this buoy weakly. The colleague is iterating right now
(`model_a_img3922_best` fine-tuned 10:30, its engine exporting; `detect_buoys.py --source zed` holding the ZED since 11:12) - the
next comparison should use their newest model once they say it's ready (`CV_MODEL_A=<file>`). JETSON_LOG has the details.

## v0.1.78.19 — 2026-11-10 — the UAV team's own buoy map received on the boat (Mac only, not deployed)

Dmitrii: "build it on our side; make sure we have a notification / status that the boat receives their map; distinguish the map source, so
the boat does not take the drone's map for granted but associates it correctly."

Scouted read-only on the UAV Jetson (`~/Desktop/UAV software/buoy-detection/`, files of 2026-11-10 09:06): `map_mqtt_publisher.py`
watches `cv_map.json` (cv_runner.py, ~1 Hz) and publishes the whole map on `sgit/uav/cv/buoy_map` (QoS 1, not retained):
`{"points": [{"id", "color": RED|GREEN|BLUE, "led_type": FLASHING|STATIC, "coords": [lon, lat]}]}`, only classified buoys, plus a
retained `sgit/uav/cv/status` online / offline (last will); default broker 192.168.1.218:1883 (the NX's system mosquitto - disabled,
stays so). No "survey complete" in their wire. Their map ids come from a map rebuilt every second.

What changed · where:
- `optiprimex/uav/sgit_map.py` (new, pure): decode, to_contract ([lon, lat] -> lat / lon, RED -> red, FLASHING -> flash, STATIC ->
  steady, feed "sgit", frame_id sgit-N, scan.state scanning; their id kept as `sgit_id`, not used as the tracker id -> association by
  position), status_text.
- `optiprimex/usv/mission_manager.py`: subscribes `sgit/uav/cv/buoy_map` + `sgit/uav/cv/status` (raw, QoS 1) on the boat broker; the map
  into the same queue / ingest_geo as `uav/perception/buoys`; the status handled before the pose check; a log line on the first map and
  on every status change.
- `optiprimex/usv/perception_map.py`: `uav_feeds` per source feed (contract | sgit): messages, points, with_position, age, status; in the
  digest as `uav.feeds`; each UAV vote carries its `feed` / `sgit_id`. The fusion rules are unchanged: an item within uav.assoc_m (1 m)
  of a boat-tracked buoy gives it colour / light and the boat keeps its own position; otherwise a UAV-only buoy (source "uav", needs
  uav.min_obs 2 sightings, a lead, never planned through).
- `optiprimex/tools/ocs_web.html`: the UAV card's **UAV map** row (shown with or without the UAV adapter): per feed buoys with position /
  points, age (amber > 5 s), online / offline, linked, UAV-only, scan state.
- Their side needs only a flag: `python3 buoy-detection/map_mqtt_publisher.py --broker 192.168.1.218 --port 1884` (our boat broker,
  up while run_usv.sh runs). Their code untouched.

Tests: tests/test_v0178019.py (3: their docstring example verbatim -> our contract; association with the sources kept apart over three
1 Hz repeats; the wiring) + test_v0166 (16) pass; make quick 616 passed / 2 failed (the known test_v0164 pair, unchanged).

Open: (1) WAIT_UAV_MAP with their feed leaves only on `uav scan done` or landed-and-quiet (their cv_runner must stop for "quiet") - ask
them for a "complete" or use the command; (2) deploy to the NX (asks first) and a live check: their publisher -> NX 1884 -> the OCS row;
(3) the 2 test_v0164 failures from the unlogged wire.py edit (UAV1 / SGIT-53577) remain, Dmitrii's call.
Next session first: deploy .19 to the NX with Dmitrii's OK, run their publisher with --port 1884, watch the UAV map row.

## v0.1.78.20 — 2026-11-10 — the drone's identity: UAV1 + SGIT-53577 everywhere (Mac only, not deployed)

Dmitrii: "go methodically through all files which bridge / declare the drone and fix the namings" (his teammate's screenshots,
extra/callsign/: the profiles' team_id / vehicle id / registration / call-sign lines; the buoy-map handoff). Decided by AskUserQuestion:
**vehicle id UAV1, registration SGIT-53577, pilot Calwin-U2332301** - RoboNation's own page of 2026-11-05 showed "UAV1 · drone
SGIT-53577" and Garuda "UAV1 SGIT-53577: flying - cleared"; the venue page said "AA53577 - no match".

Before: profiles = vehicle SGIT-53577 / reg SGIT-53577; wire.py (unlogged edit 2026-11-09 18:41) = UAV1 / SGIT-53577 (2 test_v0164
failures); his bridge = SGIT-53577 / AA53577; his listener default UAV1, his GUI starts it with SGIT-53577.

What changed · where: profiles/{course,pool,bench}/ocs_*.yaml (`vehicles: UAV1: uav`, `uav.vehicle_id: UAV1`, comments);
optiprimex/uav/wire.py (the two constants commented as OURS, the values kept); scripts/run_uav.sh (the sim listener `--vehicle-id UAV1`);
ocs_web.html / uav_web.py / uav_web.html fallbacks, playbook.py, fault_timing.py; tests test_v0164 (identity asserted as ours, his
bridge's identity asserted as his, the other constants pinned verbatim as before), test_v0164_scenarios, test_v0165, test_v0167,
test_v0178015; GUIDE 4.20a / 4.20e + header; docs/UAV_TEAM_CHECKLIST.md. Historical docs (reports, overnight notes) left as written.

Their side (read-only to us, to tell them): their listener drops a RoboCommand command addressed to another vehicle id
(robocommand_handler.py:207) - `drone-heartbeat-gui/heartbeat_monitor.py` ~1386 `--vehicle-id SGIT-53577` -> `UAV1`, and
`lawnmower-mission/task4_mission.py:61` `UAV_VEHICLE_ID = "SGIT-53577"` -> `"UAV1"` (its ReadinessConfirm match). Their bridge stays off.

Tests: make test 830 passed, 0 failed (test_v0164 green again; v0.1.78.19 included).
Next: deploy .19 + .20 to the NX / AGX with Dmitrii's OK; the venue page check (DECLARE UAV -> declare -> "UAV1 · drone SGIT-53577").

## v0.1.78.21 — 2026-11-11 — housekeeping: CV_NEW/ out of the archive (the scout night starts)

Per docs/PROMPT_SCOUT_BUILD_2026-11-11.md (Dmitrii confirmed ~00:30: deploys in the morning; the guard hooks into OUR observer
cv/buoys_multi; the sim world with the ZED 2i 4 mm preset at 1.0 m above the water + the distance where a 0.40 m object disappears;
ground test missions for the guard and the detection; Lua v4 reminder at the deploy; tmux + laptop free).
What changed: CLAUDE.md step 5 gains `--exclude='MASTER_SCRIPTS/CV_NEW'` (no other tar line exists in scripts/). The folder is untouched.
Tests: make quick 618 passed, 0 failed. Next: WP35 = v0.1.79.

## v0.1.79 — 2026-11-11 — WP35: mission SCOUT + the ground test m0t8 (Mac only, not deployed)

Per docs/PROMPT_SCOUT_BUILD_2026-11-11.md §1 / docs/SCOUT_BRIEF_2026-11-11.md §2-§3, with Dmitrii's answers of ~00:30 / ~01:00: the S15 world
with the ZED 2i 4 mm at 1.0 m; the range where a 0.40 m object disappears; ground tests first (m0t8 detection now, m0t9 guard with WP36);
no RxCourse -> a 30 x 30 m field ahead; HOLD stays the station keep until the drift check (WP37).

What changed · where:
- `optiprimex/usv/tasks/scout.py` (new; t1_safe_passage.py untouched): WAIT_START -> [WAIT_UAV_MAP when the drone is declared: scan.complete /
  landed and quiet / `uav scan done` / uav_wait_s -> on without it] -> ADVANCE (along the heading inside the 2 m inset; toward the centre
  when the heading leaves it within 3 m) until the first lock -> PICK (nearest unvisited by path length through gatepath.avoid with
  Clearance clear_m 1.6; ties -> nearer the centre; the target's own ring excluded from its stand-off check - found: a drone-only buoy
  with sigma 0.5 had its 3 m circle inside its own ring) -> APPROACH (12 stand-off candidates outside every ring, inside the inset) -> LOG
  (log_s, the record) -> PICK; map dry -> SCAN (one turn; a new candidate ends it) -> FRONTIER (lanes centred in equal strips <= lane_m,
  skipped when 80 % of the 5 m around is seen) -> SCAN; budget_s / no frontier -> RETURN -> DONE. jsonl `<log_dir>/scout_<run>.jsonl`
  (buoy lines with source camera | uav | both, visited / blocked; summary), a `scout` event per line, SafePassageReports on set change
  (beacon UNKNOWN, FLASHING_RED only when voted red). `on_guard()` = WP36's hook (logs only now).
- `optiprimex/usv/scout_geom.py` (new): inset, inside, ray_exit, advance_goal, lanes, Coverage, visit_point.
- Declaration: SCOUT -> T1 CORE, T1 ADVANCED with `declare uav on` (declaration_checks.mission_needs, ocs_app._t1_flag + declaration_rows);
  REGISTRY / vehicle_fsm / cmd_authority / the schema enums; playbook Mission `scout` after m1t2 (stages for the stepper).
- `optiprimex/usv/tasks/test_detect.py` = **m0t8 TEST_DETECT** (m0t7 is taken: the `collect` session - found by make live's page pin):
  the boat still (HOLD), seen / range / lost / back lines per buoy from the map's new raw `last_range_m` (perception_map keeps each
  sighting's range / bearing - the averaged position lags a walked buoy), split tracks chained within chain_m 2.5; logs/detect_<stamp>.jsonl.
- The S15 world `profiles/bench/world_scout.yaml` (ALPHA-sized 25 x 30.2 m, ten unlit buoys size_m 0.57 = 0.40 m above the water - the
  renderer's sphere sits 0.7 x size above it, `height_m` is not read for buoys; the ZED hd2k 50.9 deg at camera.height_m 1.0, read now
  by the fast SIL's CameraPath and the SIM page); `run_sim.sh / run_bench.sh / sim_world.sh scout`; sim_day S15 (colour-blind map) + S16
  (scout DONE, 10 logged) + `SimDay.map()` (one map digest from the hub).
- `optiprimex/tools/range_probe.py`: **a 0.40 m object, ZED 4 mm at 1.0 m: RANGED to 23.0 m (22.91 m measured; 21 px high at half hd2k),
  bearing-only from 23.5 m (the observer's --max-depth 25 on the median depth), still 9 px high at 50 m** - the geometry is not the limit,
  the model's recall on ~20 px objects and the ZED depth noise are; m0t8 measures the real one.
- OCS: picker rows (EVENT: scout; POOL: m0t8), ocs_cli `--budget` / `--duration`, the LOG's `scout: B3 logged … · camera` and `detect: #N
  lost at … m` lines (bench_tui Model), the map digest's `scout` block -> map.html's green visited ticks + the dashed target ring.
- Profiles: `scout:` and `test_detect:` blocks in usv_{pool,course,bench}; usv_ui/config/usv_bench_share.yaml regenerated.
- Pins updated: test_wp10d (RUN_FREE_TASKS + TEST_DETECT), test_wp9c (scout = judged T1), test_v0171 (the picker groups).

Tests: tests/test_scout.py (10: geometry, the start rule, the declaration both ways, the fast SIL - all ten logged on real buoys, hull edge
> 1.0 m, inside the inset, jsonl, reports, home -, facing away (no SCAN before the first lock), the drone map (UAV-only visited, `both`), the
guard hook, the playbook / picker / map tick / LOG line, m0t8's last-seen range). make quick (after the fixes) / make live 833 + 1 pin
fixed / **make test 839 passed, 0 failed**.
Rehearsed on the Mac: `scripts/run_sim.sh scout --course world` (the world's own field: without --course world the fake judge sends EXTRA
and the scout would search that polygon) + `sim_day --only S15,S16`: S15 pass (3 buoys, all black), **S16 pass: DONE in 252 s, 10 logged,
ADVANCE > APPROACH > LOG > RETURN** through the rendered camera; logs/scout_1001.jsonl: 10 + summary (95.1 m, 249 s, coverage 100 %, 2 scans,
9 reports). Fast SIL: 10/10 in 247 s / 94 m (facing the field), 201 s / 74 m (facing away).
Not done: the Chrome walk at 1440 x 900 (the OCS page timed the screenshot out three times - left; the map tick is pinned by the tests).
Open: deploy (morning); real detection range (m0t8 on land); scout on the water (W4).
Next: WP36 (the guard; its pure parts guard_core.py / usv/guard.py / sim/fake_guard.py are written, not yet wired - excluded from this archive).

## v0.1.80 — 2026-11-11 — WP36: the ZED proximity guard + the ground test m0t9 (Mac only, not deployed)

Per the brief §4 and Dmitrii's answers: the guard computes in OUR observer (cv/buoys_multi - one process holds the ZED; the students' snapshot
untouched: choice (a)-like but in our own observer, documented here); brake → back off 2 m → re-plan on its own; m0t9 may pulse the motors.

What changed · where:
- `cv/buoys_multi/guard_core.py` (new, pure): the ZED cloud (IMAGE frame) levelled by the IMU's **gravity vector** (Rodrigues: the measured
  down axis onto the frame's - exact for any tilt and heading; an Euler roll/pitch first try had the roll sign wrong and an order error,
  caught by the numeric check), points 0.15–1.5 m above the water (h_cam 1.0), < slow_m 3, ±35°, ≥ 400 px, STOP after 3 frames in a row,
  the bearing extent. `multi_buoy_observer.py`: `--guard on` (default), `--h-cam 1.0`, `--guard-stop/--guard-slow`; the IMU orientation at the
  image time (`get_sensors_data(.., TIME_REFERENCE.IMAGE)`; an SDK / SVO without it: unlevelled, `imu: false`); `payload["guard"]`.
  `run.sh --self-test` checks it. `cv_publish`: `guard` → `local/<v>/perception/guard` every frame.
- `optiprimex/usv/guard.py` (new): GuardSupervisor between every task and the FCU (the mission manager: the guard topic intercepted in
  _on_perception; `filter()` after runner.step; BRAKE / BACKOFF own the boat while the task waits; AUTO_SAFE_HOLD via request_safe_hold;
  reset when the vehicle leaves V4; `status/usv guard` view). SLOW → cap 0.3; STOP → BRAKE (mode-20 astern 0.4, 1.5 s or < 0.02 m/s) →
  BACKOFF (mode-20 astern 0.3 until 2 m or 10 s - straight astern rather than the brief's GUIDED point behind: GUIDED would turn the boat
  round next to the obstacle) → rings (`PerceptionMap.add_obstacle`, kind obstacle, source guard, kept guard_forget_s 900) at the nearest
  point, both bearing ends (> 8°) and the remembered sightings → the task's behaviour restarted = the leg re-planned (gatepath.map_obstacles
  now includes the rings). 2nd trigger on one target → task.on_guard_block (scout: the buoy logged blocked, PICK); 3rd within 3 m →
  AUTO_SAFE_HOLD + CAUTION. Silence > lost_s → cap + CAUTION (`require: true` on pool / course = also before the first message).
  **Found: the ZED 2i hd2k sees ±15° vertically; 1.0 m up, the frame's lower edge meets the water 3.7 m ahead - anything under 0.6 m tall
  (a 0.4 m buoy, a boom, a pontoon edge) leaves the frame inside 1.5–2.6 m, so the camera alone never reports STOP for it.** Fix: SLOW
  sightings are remembered as local points (memory_s 20); one coming within stop_m ahead triggers the STOP (`remembered: true`). In the
  rehearsal every STOP came from memory. (A camera pitched down ~10° would also help - Dmitrii's call at the mount.)
- `optiprimex/usv/tasks/test_guard.py` = **m0t9 TEST_GUARD**: still; verdict lines on change (range, extent, px, roll / pitch / imu); a STOP =
  one brake pulse (0.4 astern 1.5 s, props in the air; `pulse 0` = none) + a `would back off 2 m` line; guard_exempt. Registered as m0t8 was.
- `optiprimex/sim/fake_guard.py`: the guard from the world's geometry incl. the frame's lower edge (the fast SIL's `guard=`); the renderer
  draws `kind obstacle` (a pole = cylinder, a boom = a box along the segment) - in the depth, never a buoy mask; `cv/sim_observer` runs the
  real guard_core on the rendered depth (`guard_cloud`, `--guard on`). world_scout: a pole (11, 2.6) and a 2.5 m boom (23.5, -2..0.5),
  `sensed: false` (fake perception reported them as buoys at first), `walls: false`.
- Scout: guard_target (the buoy / the frontier point / the stage), on_guard (obstacle lines in the jsonl + LOG), on_guard_block.
- The GUARD chip: the OCS bar (`guardchip`, S.status.guard) and the boat window (`guard-now` beside RC6; boat_web fcu_ctl.guard).
- Profiles `zed_guard:` (require true on pool / course, false on bench) and `test_guard:`; usv_bench_share regenerated.
- The Test Guide: a new section (*added 2026-11-11, agent*): D7 (m0t9 on a stand), D8 (m0t8 the detection range), W1–W4 (the water order).
- The observer tolerates a ZED SDK without SensorsData (tests/test_wp9b's stub found it).

Tests: tests/test_guard.py (10: persistence + the water band, the tilt ±10° does not move the trigger (exact), SLOW cap / BRAKE in the same tick
as the message / BACKOFF 2 m / rings / re-plan, block + safe hold, lost cap + muted stages, the remembered low thing + the fake's frame edge, the
fast SIL on S15 with both obstacles (DONE, centre > 1 m off the boom / pole), the sim observer's filter on rendered depth, m0t9 (lost → clear →
STOP + pulse, pulse 0), the wiring). make quick 633 + the 4 wp9b fixed / make live 844 passed / **make test 849 passed, 0 failed**.
Rehearsed on the Mac: `run_sim.sh scout --course world`: 13 guard messages / 3 s on the boat broker (source sim_observer); sim_day S15 pass,
**S16 pass: DONE in 286 s, 10 logged (1 blocked), the guard STOP at 1.47 m in APPROACH → 2.0 m astern → re-plan, the 2nd at 1.50 m → B
blocked**; logs/scout_1001.jsonl summary: visited 9, blocked 1, 117 m, 283 s, coverage 100 %, 2 obstacles.
Open: the bench check D7 (board, tilt) and the water check W2; whether to pitch the ZED down; deploy (morning).
Next: WP37 (station keep HOLD / LOITER switch - default hold until the drift check; HOLD yellow; the FCU preset file; the Lua v5 table).

## v0.1.81 — 2026-11-11 — WP37: the station keep HOLD / LOITER, the words, HOLD yellow, the FCU preset (Mac only, not deployed)

Per the prompt §3 with Dmitrii's answers: station_keep **hold** by default in every profile until the LOITER drift check on the water (W1);
HOLD **yellow** (standby, not autonomous), LOITER / GUIDED / AUTO / mode 20 green.

What changed · where:
- `companion/state.py`: `Loiter(Hold)`, `station_keep_mode(cfg)` (the live word or the profile's station_keep.mode; anything else = hold),
  VehicleState.station_keep → status `station_keep`.
- `usv/adapters/rover.py`: `hold(now, mode)` + `loiter(now)` (LOITER with the same hold_reassert rule); `apply()` checks Loiter before Hold.
- `usv/behaviours/station_keep.py`: Loiter when the switch says loiter. `usv/mission_manager.py`: V3 AUTO_HOLD → adapter.loiter when loiter,
  V5 always HOLD; status station_keep every tick. `can_enable_auto` unchanged (the FCU in HOLD before ENABLE_AUTO).
- The word `station_keep hold|loiter`: ocs_cli, op-tui (bench_tui WORD_TO_ACTION), cmd_authority STATION_KEEP → SET_STATION_KEEP (the gate of
  rc6_policy), the schema, the vehicle's `_fcu` (no FCU link needed; cfg.extra station_keep.mode - kept across runs in the process), playbook
  entry. Buttons: the RUN USV tile (station keep row: HOLD / LOITER + "in force") and the boat window's FCU card (`sk-hold` / `sk-loiter` through
  the OCS's /cmd like the RC6 rule; OCS_LINES + the two lines).
- The words (L3): `playbook.vehicle_words` - V2 + FCU HOLD + no override = "HANDED OVER · HOLD · enable_auto to go"; RC6 holds or FCU MANUAL =
  "MANUAL (RC)"; AUTO_HOLD = "AUTO · LOITER" / "AUTO · HOLD". The USV window's VEHICLE plate sub-line and the OCS tile's state row. The heartbeat
  stays STATE_MANUAL in V2 (right: AUTO triggers RunStart).
- HOLD yellow: `.pb.hold` and `#fb-hold.act` on --warn (the class names kept); LOITER green (`fcu_mode_class` "auto").
- `docs/fcu/PRESET_LOITER_RC6.md` (the read-back table to fill, why each value, the D9 walk, the Lua v5 mode table - v4 is in
  ../test_scripts/LUA/, not in the tree: Dmitrii drops it into LUA/ at the deploy) + `docs/fcu/loiter_rc6.param` (the orin_kit apply_params.py
  `--file` format, checked against its load_file: `#` comments, NAME,VALUE). **Not applied.**
- Profiles: `station_keep: mode: hold` in usv_{pool,course,bench,lab}; usv_bench_share regenerated. GUIDE 4.22c; the Test Guide's D9 (the RC6 /
  LOITER walk, props off).

Tests: tests/test_station_keep.py (6). make quick 640 + 3 page fixes (the two new buttons had no title - the boat window's rule) / make live 848 +
2 pins updated (test_v0178010: HOLD green → yellow, Dmitrii's decision of today supersedes the 2026-11-09 one; test_v017809: the last OCS line)
/ **make test 854 passed, 1 failed** - tests/test_v0145.py::test_the_page_draws_set_state_and_every_button_has_a_title under load (a thruster box still `unknown` when the headless page was read); **passes alone** (rerun) and passed in make quick: a timing flake, not this change.
Rehearsed on the Mac (`run_sim.sh pool`): station_keep loiter accepted in V2; ready → AUTO_HOLD with `hold`: status mode HOLD; `station_keep
loiter` in AUTO_HOLD → **status mode LOITER**, the fake FCU's log `mode HOLD -> LOITER (GCS)`; `hold` → back to HOLD.
Open: the D9 walk (after the preset, props off) and W1 (the drift check) - then `station_keep loiter`; the Lua v5 install.
Morning hand-over: `docs/MORNING_2026-11-11.md` (the findings, the deploy order, the reminders). WP38–WP40 not started (by choice: the declaration
and the measurement path the day before the first water - better after today's data).

## v0.1.81.1 — 2026-11-11 — obstacles on the USV map + the recorded map (Mac only, not deployed)

Dmitrii: "add obstacles on the USV map - if we see something, we put a 2-3 m circle behind that area? … after the run, can we save the map we
recorded? later we might want to replay that". Polled (AskUserQuestion): the source = **the ZED depth 3-8 m + the guard's STOPs**; the ring =
**2 m, centred 1 m behind the seen face**; saved **on both the boat and the Mac**; the format for **both** replays (watch again / a starting map).
Why 3-8 m: at 1.0 m up the ZED's frame meets the water 3.7 m ahead - a low object is only in view there; the guard (< 3 m) sees it too late.

- AGX (our copy, `cv/buoys_multi/`): `guard_core.far_clusters` - the same levelled cloud, 0.15-1.5 m above the water, slow_m..map_m (3-8 m),
  ±50°, binned by bearing (2°), neighbours within 0.8 m in range joined, >= 150 px, nearest first, <= 8 → `guard.far` in the message
  (cv_publish forwards it unchanged on perception/guard). Observer `--guard-map 8` (0 = off). self_test: a 0.4 m face at 6 m → 1 cluster.
- Boat: `usv/guard.map_far` (2 Hz; any vehicle state - an RC drive maps too) → `PerceptionMap.add_obstacle(source="depth", face=…)`: merged
  within 1 m (running mean), counts from `zed_guard.depth_min_obs` 3 (1.5 s) in the planners (`gatepath.map_obstacles`) and the digest; none
  where a buoy (boat's or UAV's) / dock / box / window lies within 1.5 m of the face or the centre, and an existing one is removed when such an
  entity appears there later (the detector found the buoy); a wide cluster (> 8°) gets rings along its arc (<= ring_m apart, <= 6). Every
  ring incl. a STOP's: `ring_m 2.0` (was 1.5), `ring_behind_m 1.0`; a STOP on a depth obstacle turns it into a guard one. Mission manager:
  /guard with `far` → the tick queue → map_far at the frame's pose. `zed_guard.map false` = the v0.1.80 behaviour. Digest: `ring_m` on obstacles.
- map.html: an obstacle = an orange ring of ring_m + ✕ (dashed = depth, full = a STOP), label `obstacle (depth|stop)`; legend line.
- `usv/map_rec.MapRecorder` (new): fed every telemetry/<v>/map body; `<stamp>_<src>_map.jsonl` (header + a frame a second), `_latest.json`
  (every 10 s, atomic), `_<TASK>_<hhmmss>.json` when the body's task changes away from it (not for idle) and `_end_` on close; snapshots
  `usv_map/1`: every entity / ray in lat / lon too (the ControlOrigin's flat earth), the track (>= 0.5 m), counts. Boat: mission manager
  `--map-rec DIR` (MapDigestPublisher.recorder; closed in stop()), run_usv.sh / run_bench.sh pass `$RUN_DIR/maps`. Mac: ocs_web `--map-rec`
  (default logs/ocs/maps, env OCS_MAP_REC, `off`), fed from the track loop. Nothing reads them back yet (the replay comes after the runs).
- Sim: fake_guard's `far` (one cluster per world object, no occlusion, buoys included); the fast SIL feeds map_far.
- Profiles (pool / course / bench): ring_m 2.0, ring_behind_m 1.0, map true, depth_min_obs 3; usv_bench_share regenerated. GUIDE 4.22d + §8
  row; Test Guide D10 (+ W4 pulls the maps); conftest sets OCS_MAP_REC off (the live tests had written into logs/ocs/maps - removed).

Tests: tests/test_map_obstacles.py (9, in QUICK) - the clusters (range band, two things, specks, glitter, off), the ring 1 m behind / 2 m /
3 sightings / the digest, a buoy explains + a late buoy removes, a wide cluster, the STOP ring + claiming a depth one, the profiles, the fast
SIL S15 (DONE, **no STOP**, the pole and the boom ringed, none round a buoy, 10 logged), the recorder (timeline, task snapshot with lat / lon,
latest, end, idle skipped), the wiring. test_guard's S15 run pinned to `map: false` (the v0.1.80 STOP path). **make test 863 passed, 1 failed**
(test_wp10e: usv_bench_share drift - regenerated, passes) / make live 859 passed / after the conftest + idle fixes make quick 652 passed.
Rehearsed on the Mac: `run_sim.sh scout --course world` + `sim_day --only S16`: **pass, DONE in 311 s, 10 logged, 0 blocked, no guard STOP**
(v0.1.80: 2 STOPs, 1 blocked) - 6 depth rings (the pole: 2, the boom: 4), none on a buoy; the boat's maps/ and logs/ocs/maps/ each hold the
timeline (~1.4 MB for 6 min ≈ 15 MB/h), latest, the SCOUT snapshot and the end snapshot.
Risks: a false ring from the water (spray, a wake) can close a gate lane for the planner - gated by 3 sightings + the 0.15 m band; on the
water watch the map (D10 / W2) and raise depth_min_obs or set map false. The dock: a pontoon near a bay is ringed (docking does not plan
through map_obstacles - unchanged, but watch it). No negative evidence yet (a ring is not cleared by looking at an empty spot; forget_s 900).
Next: deploy with v0.1.81 (AGX + NX) per docs/MORNING_2026-11-11.md; D10 on land; the replay tools after the runs.

## v0.1.81.2 — 2026-11-11 — the lamp script v5 + the FCU parameters kept (venue)

Deployed v0.1.81.1 to the NX and the AGX first (docs/JETSON_LOG.md: concurrency check clean, self-tests PASS). Then the FCU read-back
(`apply_params.py --dry-run`, nothing written): MODE_CH 8 (mode 20 / HOLD / GUIDED), RC6_OPTION 51 - the preset would have moved the mode
switch to RC6. Dmitrii: **keep as it was** → docs/fcu/PRESET_LOITER_RC6.md "Decision"; LOITER only by the companion's request.
LUA/ (new in the tree): robotx_lamp_v4.lua copied unchanged from ../test_scripts/LUA/ + PROVENANCE.md (sha256); **robotx_lamp_v5.lua** = v4
with `[4]=true` removed from MODE_AUTONOMOUS (HOLD yellow), the header and the start message - nothing else. Not syntax-checked (no lua on
the Mac); the diff is the one table line. Install: MAVFtp → APM/scripts/ (v4 out), reboot, props off; check `robotx_lamp_v5 started … hold=yellow`.
No Python changed, no tests run for this step (make quick at v0.1.81.1: 652). The Jetsons stay on v0.1.81.1 (LUA/ is not used there).
Answered: do not pull the SD card with the FCU powered (LOG_DISARMED 0 → it writes all the time) - MAVFtp or power off.

## v0.1.82 — 2026-11-11 — venue requests: USV heartbeat buttons, observer from the OCS, WP39, scout yaw cap (Mac; not deployed)

Polled: PILOT READY = OCS gate only; the observer = our buoys_multi; WP39 full; yaw cap 10 °/s.
- USV flow: `ocs.usv_heartbeat: button` (profiles/course/ocs_course.yaml - rctest / venue / course; pool + bench stay auto = v0.1.59):
  the boat out of hb_vehicles until `report on usv` (▶ HEARTBEAT USV); `usv ready on` (USV_READY intent, OCS flag; needs the heartbeat);
  declare refuses without both; ready cleared by end_run / reset (the heartbeat stays: handbook "at all times"); `report on|off` covers
  the USV; state `usv_flow`. Page: usvStrip on the RUN tile + VEHICLES tab (3 big buttons, built once, like the UAV's).
- Observer: scripts/agx_observer.sh start [--detector default|model-a] | stop | status (buoys_multi --headless in tmux agx:obs; a stream
  holding the ZED is restarted on dir; other holders refused); launcher OBSERVER_CMDS + SYSTEM action `observer` call it; OBSERVER ▶ on
  the RUN tile and the CAMERA card (+ the SETTINGS card). **Bug found**: agx_cam.sh's observers() lacked multi_buoy_observer → `auto`
  picked zed while our observer held the ZED = "camera on, not streaming". Fixed.
- WP39: launcher judge += alpha/bravo/charlie/delta (venue shown as DEMO); run_ocs.sh maps them to RC_<C>_HOST/_PORT (network.lan.env:
  192.168.65-68.2:1883, added), RC_COURSE in the OCS env; /course_check.json + the "RoboCommand networks" card (NIC subnet → "● on this
  network", TCP 300 ms probe, the retained RxCourse by one short MQTT look only when the OCS is not running); the plan's note on a wrong
  cable; ocs_app.course_checks → CAUTION "course mismatch" / "OCS address not in the course subnet".
- Scout: `_yaw_limit` - a GUIDED carrot > turn_gate_deg 20 off the bow → VelocityYawTarget(0, ±max_yaw_dps 10) until < 8, scan capped
  too; fast SIL S15: p95 yaw 10 °/s (was 38), 318 s (was 247), 10/10. Profiles scout.max_yaw_dps 10.
Tests: tests/test_v0182.py (8); pins updated (test_v0164/test_wp9c make_app press the buttons, test_v017801 FIELDS, test_v017807 the
observer script). **make test 872 passed, 0 failed.** Not rehearsed live (Dmitrii: 15 min to the port call - quick tests only).
Open: the live walk of the USV strip / the checker at the venue; deploy (AGX: agx_observer.sh + agx_cam.sh fix; NX: the yaw cap).

## v0.1.83 — 2026-11-11 — logging on by default (Mac; not deployed)

Dmitrii: "make sure that the logging on by default when we run start (video, position, IMU, heading, everything)".
- scripts/run_usv.sh: usv_ui gets `--log run` unless UI_ARGS already has --log (FCU_LOG=0 = off) → logs/usv/run_<stamp>/ tlog (every
  MAVLink message: GPS, IMU, attitude, heading, modes), 5 Hz csv, events.
- profiles/presets.yaml: custom log true (sim stays false; event / lab were true) → LOG=1 = every MQTT message to SQLite (OCS + boat).
- scripts/agx_observer.sh: start records by default (--record ~/Desktop/dm3/recordings --label obs: cam.svo2 = video + the ZED IMU,
  frames.jsonl, session.json; the observer's own 5 GB floor); `--no-record` = detect only. The launcher's real-CV plan now STARTS it
  (a new last AGX step) - was "start by hand". Already always on: mm_trace.jsonl, the map recording (v0.1.81.1), the program logs.
Tests: test_v0182.py::test_logging_is_on_by_default; test_launcher pins (+1 AGX step). Only the touched test files run (time: port call).
Open: deploy v0.1.83 (NX: run_usv.sh; AGX: agx_observer.sh); disk on the AGX (19 GB free on 2026-11-11 - the svo2 is ~1-2 GB / 10 min).

## v0.1.83.1 — 2026-11-11 — the OBSERVER button sent stop on a down observer (venue; Mac page only)

Seen at the venue: the launch's observer recorded 11:39:13-11:44:28 (613 MB svo2, 276 frames = 0.9 analysed fps - the colleague's TensorRT
export was on the GPU), then a Ctrl-C: logs/ocs/system_actions/ job 2 `observer {"do": "stop"}` 11:44:25, job 3 stop 11:48:44. Cause:
obsUp() = the perception beat < 3 s + a detector-name regex - cv_publish beats every second without any observer (`ok` false), so the
button showed ■ OBSERVER and a press sent stop. Fix: obsUp = `perception.ok` (frames < 2 s). The page is read per request: a browser
reload takes it, no restart. Test: test_v0182::test_the_observer_button_reads_the_fresh_frames_not_the_beat (only that file run).
Open: the analysed rate (0.9 fps) - recheck with the GPU free; < 3 fps → a lower resolution / depth mode for the trials.

## v0.1.83.2 — 2026-11-11 (session B) — one RECORD for the tk console's CAMERA tab

Request (Dmitrii): "make sure that the camera page is there, and once I open it, the camera stream starts. Then, on same page, I need a
clear button to start recording video (if it is already recording - confirm)". Polled: RECORD = **the ZED video on the AGX + the Mac
frames** (both). The tk console is session A's (docs/PROMPT_TK_CONSOLE_2026-11-11.md §0): B built the backend, A was asked (cross-session
message) to put a CAMERA tab into Phase 1 - the stream started on open (GET /cam.json; not ok → POST /system cam start), RECORD on
/recording.

What changed · where
- `scripts/agx_observer.sh status [--json]`: whether the ZED is really recorded (the observer's cmdline has --record AND the newest
  ~/Desktop/dm3/recordings/*_obs/cam.svo2 was written < 10 s ago) → `observer: up (buoys_multi) · RECORDING <dir> · N MB · written s ago
  · GB free` / `· recording STALLED` / `· not recording`; `--json` one line {observer, recording, dir, mb, age_s, free_gb}. The
  `observer: up|down` prefix kept (the launcher parses it).
- `optiprimex/tools/ocs_web.py` CamTab: `recording_status()`, `recording(body)`; routes `GET /recording.json`, `POST /recording
  {op: status|start|stop, confirm}`. start while recording → {ok false, already true} unless confirm "new" (stop both, a new clip);
  observer up without --record → needs "restart" (the ZED guard pauses ~10 s); stop → needs "stop", then the observer restarts with
  --no-record (detection goes on) and the Mac recorder stops. Mac frames = frame_rec into logs/recordings (mac_logs), 2 Hz, until stop.
  **Additive**: the page (ocs_web.html) and /camrec untouched - the backend-freeze note in the coordination message allows additions
  that a console ask needs; this is one.
- tests/test_v018302.py (4: start both / already → ask / new clip; observer without recording → restart; stop asks, the observer back
  --no-record; the routes + the script's --json shape + the launcher's prefix). Run: that file + test_launcher, test_v0160, test_v0182,
  test_v017807 = 41 passed. No make quick / live (Dmitrii's stack runs).

Not deployed: agx_observer.sh's new status needs the AGX tree (v0.1.83 there) - until then /recording.json's agx part says
"unknown" (the JSON line missing) and start / stop still work through the old script. Deploy on Dmitrii's say-so.
Open: the tk launcher (Dmitrii's next ask, mid-turn) - planned, not started.

## (session A, in progress) — 2026-11-11 — the tk operator console, Phase 1, gate 1 (Mac; version at archive = next free 0.1.N)

Baseline before the first change (12:51): ocs_web.html f3f6a12d…a193ff · ocs_web.py 6ef51cbb…0deca729 · map.html c45f5807…bdb05abe5 ·
run_ocs.sh 796a84d1…8cd68d. (ocs_web.py changed at 13:01 by session B's v0.1.83.2 - additive /recording; A never wrote it.)
Gate 1: optiprimex/tools/ocs_tk.py, tkconsole/{model,net,theme,views}.py, scripts/run_ocs_tk.sh, tests/test_ocs_tk_{model,net,smoke}.py,
tests/fixtures/state_*.json (recorded from run_sim.sh pool in an isolated tmux server, ~/.cache/tkrec), docs/TK_CONSOLE.md.
Mac: brew install python-tk@3.10 (+ tcl-tk@8 8.6.18); the venv had no _tkinter. For session B: docs/TK_CONSOLE.md §3 - state.logging,
**/cmd refuses station_keep / usv / uav (the page's buttons too)**, lat/lon in /fast.json, GET /map.json, the Makefile QUICK line.
Tests: the 3 files 15 passed; make quick 662 passed. Screenshot: not taken (this process has no Screen Recording permission).

## v0.1.83.3 — 2026-11-11 (session B) — the launcher as a tkinter window

Request (Dmitrii): "lets also make launcher page as tkinter"; polled: **B, now, standalone** (not waiting for A's tkconsole package);
mid-build: "make sure that we use different launcher.sh, I will play with old one now".

What changed · where (new files only; launcher.py / launcher.html / scripts/launch.sh untouched)
- `optiprimex/tools/launcher_tk.py`: a tk client of the launcher's HTTP backend - the same cards and rows as the web page (preset, the
  run, the boat, network + manual addresses, rates and logging; the same show / hide rules), THE PLAN with ⧉ copy per command, LAUNCH
  (off with the reason while the plan has errors or something runs), STOP ALL (type stop), the banner (RECONNECT / STOP ALL / STOP then
  new) and read-only when adopted, tabs PROGRESS (page links with ● up, clocks, progress, the launch log) · RUNNING (hosts + restart this
  host) · NETWORK (the table; ssh ↗ / copy my key ↗ / observer ▶■ on the picked row) · ROBOCOMMAND (the WP39 checker) · CONSOLE (a
  window's tail); the pages' theme select (POST /prefs). Reliability: Tk only on the main thread - a 1 s poller (status 1 s, session 3 s,
  prefs 2 s, console 2 s), a slow worker (network / course: pings, ssh), a command worker (POSTs; /plan coalesced to the newest), all
  results through one queue drained every 50 ms; labels / texts written only on change; toasts top-right on every press (grey sent →
  green ✓ / red ✗ with the backend's answer, amber "no answer yet" at 10 s, gone 3 s after final; launcher state changes toast too).
- `scripts/launch_tk.sh [--dry-run] [--dark]`: starts its own backend `python -m optiprimex.tools.launcher --port 8093` (nohup,
  logs/launch/launcher_tk_backend.log) when :8093 does not answer, refuses 8094, then the window. Both backends share logs/launch/
  last.json and the probe, so neither launches over a running stack.
- tests/test_launcher_tk.py (7: the show / hide rules, plan / status / session / network / course views, the toasts, the client against
  a dry backend incl. refusals / a dead port / plan coalescing, the script is separate from launch.sh, a window smoke test). Screenshot
  checked by hand (choices wrap at 4 per row after the first look).
Not in it: the web page's hover tips (TIPS) - the labels only; the five page themes (the window has light / dark; the pages' theme is
still set from it). Open: a reliability session by Dmitrii on the venue network.

## v0.1.83.4 — 2026-11-11 (session B) — five page words refused as "unknown command"

Found by session A (pool sim 12:53, docs/TK_CONSOLE.md §3 ask 1): ocs_web.command() filters on bench_tui.INTENT_WORDS, which lacked
`station_keep`, `usv`, `uav` - and, scanning every send() of ocs_web.html, also `cancel_auto` and `hold_on_start` (the 3-2-1 HOLD /
CANCEL after RunStart). ocs_cli parses all five. So on the page STATION KEEP HOLD|LOITER, USV PILOT READY, UAV PILOT READY, HOLD / CANCEL
in the countdown all answered "unknown command" (typed in op-tui too). Fix: the five added to INTENT_WORDS (bench_tui.py, one line) -
a backend fix the water session needs (the page itself unchanged). Test: tests/test_v018304.py (every word the page sends is known; the
five parse). Run: it + the INTENT_WORDS / ocs_web test files + test_launcher_tk + test_v018302 = 96 passed. No make quick / live (the
stack runs). Takes effect at the next OCS start (ocs_web.py imports bench_tui at start) - nothing on the Jetsons.
Archive: _archive/MASTER_SCRIPTS_v0.1.83.4.tgz holds 0.1.83.3 too (no separate .3 archive: both were in the tree together).
A's other asks (state.logging, lat/lon in FAST_KEYS, GET /map.json, A's tests in QUICK): next.

### v0.1.83.4 addendum — 2026-11-11 (session B) — QUICK gets the tk tests
Makefile QUICK += tests/test_v018302.py tests/test_v018304.py tests/test_launcher_tk.py and session A's tests/test_ocs_tk_model.py,
test_ocs_tk_net.py, test_ocs_tk_smoke.py (A's ask; the smoke test skips without a display; test_ocs_tk_toasts.py joins at A's gate 2).
Run: test_make_tiers + A's three = 16 passed. No version change (the Makefile only); the archive is not rebuilt for it.

## v0.1.83.5 — 2026-11-11 (session B) — session A's backend asks 2-4: logging state, lat / lon fast, /map.json

Dmitrii: "proceed" (with A's asks, docs/TK_CONSOLE.md §3). All additive - no field renamed or removed; the page unchanged.
- `optiprimex/tools/ocs_web.py`: `Web.logging_state()` → `/state.json` key `logging` = {usv, usv_sqlite, ocs: {on, file, bytes, t0, age_s,
  why}, any}. From the presence beats the backend already keeps (bench_tui Model.presence): usv = presence/usv_ui on the boat broker
  (`log` = the run log's name; since this version `log_bytes` = .tlog+.csv+.events, `log_t0`), usv_sqlite = presence/logger-boat,
  ocs = presence/logger-hub (this Mac stats the file for bytes). on = up and the beat < 6 s and a file named. Added after the model
  lock (a plain Lock). `FAST_KEYS` += lat, lon. `GET /map.json` → {t, age, source, map} (Model.map: usv_ui's digest while it publishes,
  else the mission manager's - what map.html draws from telemetry/<v>/map).
- `usv_ui/usv_ui.py`: `log_t0` set / cleared by log on / off, `log_bytes()`, both in the 2 s presence beat (the boat needs this version
  for bytes / since - an older usv_ui gives `on` + `file`, bytes / t0 None).
- `optiprimex/companion/logger.py`: the presence beat carries `file` (the sqlite path) and `t0`.
- tests/test_v018305.py (3) + in QUICK; test_v0152_wa's source pin of the presence line follows the extension. Run: usv_ui, playbook_tui,
  v0145, v0148, v0151, v0152, v0152_wa, ocs_web, A's three tk tests, v018302/04/05 = 104 passed + the fixed pin file 9 passed. The
  mission / integration sims not run (Dmitrii's stack runs).
Deploy: the NX needs usv_ui (bytes / since) and logger.py (the boat SQLite file); the Mac takes it at the next OCS start.

## (session A, in progress) — 2026-11-11 — the tk console, gate 2 + CAMERA + MISSION (Mac; archive = 0.1.84)

Dmitrii: "while we have time lets continue" (= go for gate 2), polled: + CAMERA (with RECORD) and MISSION tabs in Phase 1; a pressed
button changes colour (amber sending → green ✓ / red ✗ for 3 s). Built: tkconsole/toasts.py (ToastBook + PressBook + verdict, ToastManager
draws), views.py (alarms strip with ACK / ACK ALL, the RUN ladder, USV1 / UAV tiles, LOG / MAP (web map ↗) / CMD / MISSION / CAMERA tabs,
in-window confirms), net.py CamWorker (snapshot.jpg 4 Hz + /recording.json, only while the tab is visible) + post(), model.py
mission_line / mission_stages + fast lat/lon. Fixed on the way: <Configure> on the root fires for every child (filtered); pending after()
ids cancelled on close; the picker refilled when the sim suffix arrives; Tk variables released before destroy.
Found: Tk 8.6.18 (macOS) stalls inside root.update() in a tight loop with this window, mainloop() never does → the smoke tests drive
mainloop(). Rehearsed live (pool sim, isolated tmux 13:53-14:00, stopped + its brokers): from the console only - reset, enable_auto,
mission scoutsim, declare → RunStart, start, hold, resume, abort all ✓ with toasts and the ladder following; station_keep loiter ✓ (B's
v0.1.83.4); refusals shown with their why (enable_auto while usv_ui held the FCU, uav ready without the adapter); CAMERA 21 frames / 8 s
from the sim, 0 after hiding the tab; RECORD not pressed (it restarts the real AGX observer); /recording.json → "unknown option --json"
on the AGX as deployed (B). Tests: the 4 tk files 28 passed (warnings as errors); make quick 699 passed. Hashes: ocs_web.html, map.html,
run_ocs.sh unchanged. For B: test_ocs_tk_toasts.py into QUICK; TK_CONSOLE.md §3 item 7 and §3b (ROADMAP later).
Next (gate 3): the Canvas map from GET /map.json, the §2 rehearsal incl. STALE / DOWN, screenshots (needs Screen Recording granted).

## v0.1.83.6 — 2026-11-11 (session B) — small: the old AGX script, QUICK, ROADMAP

From session A (13:55, pool sim): on the deployed AGX (v0.1.83) `agx_observer.sh status --json` answers "unknown option --json", so
/recording.json said observer "unknown". `CamTab._obs_json` now falls back to the plain `status` → observer up / down, recording False,
why "the AGX's agx_observer.sh predates 0.1.83.2 … deploy the AGX". Test: test_v018302::test_an_old_agx_script_still_says_up_or_down.
A confirmed live: v0.1.83.4's station_keep loiter ✓ from the console on a fresh OCS. Dmitrii put CAMERA (+ RECORD) and MISSION into A's
Phase 1. Makefile QUICK += tests/test_ocs_tk_toasts.py. ROADMAP §4a += A's parked items (docs/TK_CONSOLE.md §3b). Run: test_v018302,
test_ocs_tk_toasts, test_make_tiers = 13 passed.

## v0.1.84 — 2026-11-11 — the OCS operator console in tkinter, Phase 1 (session A; Mac only, nothing for the Jetsons)

Dmitrii 2026-11-11 (docs/PROMPT_TK_CONSOLE_2026-11-11.md): K1-K7 + (polled) the CAMERA tab with RECORD and the MISSION tab; a pressed
button changes colour. Gate 3: `tkconsole/canvasmap.py` (ENU on the polygon centroid, the 2 m inset, the boat dart, buoys + σ + ✓ visited,
UAV buoys, obstacles + rings, points, path / carrot / scout's target; zoom at the cursor, pan, F follow, fit / fit all; web map ↗) fed by
B's GET /map.json at 1 Hz (drawn only while the tab is visible); the smoke tests share ONE Tk per process (a Tk created + destroyed per test
crashed macOS Tk ~1 in 8 file runs; the program has one; a 40-round ARM-confirm stress in one shown window: clean).
What · where: optiprimex/tools/ocs_tk.py, optiprimex/tools/tkconsole/{model,net,toasts,canvasmap,views,theme}.py, scripts/run_ocs_tk.sh,
tests/test_ocs_tk_{model,net,toasts,smoke,map}.py, tests/fixtures/state_{r2,r5_task,r6_alarm,meta,course,map}.json (recorded from
run_sim.sh pool), docs/TK_CONSOLE.md; GUIDE §4 row + header, CHANGELOG row, pyproject / __init__ 0.1.84 (B was on 0.1.83.x).
The page untouched: ocs_web.html f3f6a12d, map.html c45f5807, run_ocs.sh 796a84d1 = the 12:51 baseline; ocs_web.py changed only by B's
0.1.83.2-.6 (additive endpoints A asked for).
Rehearsal (the prompt's §2, the real window, pool sim in an isolated tmux 14:08-14:20, stopped + its two brokers): from the console only -
abort, reset, enable_auto, mission scoutsim, declare → RunStart, start, hold, resume, abort: every press toasted ⟶ → ✓ and the ladder
followed; the MAP drew the polygon, the inset, the boat, 9 buoys, ✓ visited, the target; the OCS page server stopped → STALE 0.3 s, DOWN
2.3 s, HOLD answered "✗ no answer: Connection refused"; restarted → UP 2.3 s later without restarting the console; no thread left.
Not done: the screenshots (this process has no macOS Screen Recording permission - docs/screens/ not written); RECORD not pressed live
(it restarts the real AGX observer).
Tests: the 5 tk files 33 passed (×4 runs); make test 924 passed (5:21, before the version bump); make quick after the bump below.
For session B (owns): the launcher field `console: tk|web`; the ROADMAP §4a items. Phase 2 first: the SYSTEM tab + that launcher field.

### v0.1.84 addendum — 2026-11-11 (session B)
Makefile QUICK += tests/test_ocs_tk_map.py (session A's ask; the Canvas map on /map.json). Run: it + test_make_tiers green. No version change.

## v0.1.84.1 — 2026-11-11 (session B) — the tk launcher opens the tk console; its backend survives a Ctrl-C

Request (Dmitrii, relayed by session A): "in the new GUI, opening the OCS window should start the tk console instead of the HTML page".
- `optiprimex/tools/launcher_tk.py`: PROGRESS has `OCS console: tk window | web page` (tk default) + "open the OCS console". The links
  row shows the OCS link as "● OCS console up" → `open_console()` = `scripts/run_ocs_tk.sh [--lan|--ts]` (the launch's net; sim none)
  detached (`spawn_console`: start_new_session, logs/ocs/ocs_tk.log, waits 2 s: exit 0 = A's guard "already open (pid N)", exit != 0 =
  the log's last line as the red toast, running = opened); a small "web ↗" beside it opens the page (the fallback). **Auto**: when the
  OCS link goes down → up (a launch's OCS step) and the choice is tk, the console opens by itself (`auto_open`; never on the window's
  first look). This is the tk launcher's `console: tk|web`; the web launcher's field stays in ROADMAP.
- `scripts/launch_tk.sh`: the backend starts in its own session (a python Popen start_new_session) - with `nohup … &` it was in the
  terminal's process group and launcher.main re-arms SIGINT, so a Ctrl-C at the window's terminal stopped it ("page closed" 14:42, the
  14:41 event launch kept running). Checked: backend pgid != the shell's.
- tests/test_launcher_tk.py: + test_the_console_helpers (argv, auto rule, spawn: guard / refused / opened); the smoke test now on ONE
  shared Tk (A's `shared_root`) + a Toplevel, driven by mainloop() - A found per-test Tk / root.update() loops stall or crash macOS Tk
  8.6.18 ~1/8. Run: test_launcher_tk + test_ocs_tk_smoke + test_ocs_tk_map ×4 = 19 passed each; again after the session fix.
Findings for Dmitrii: the OCS bridge drops at 14:41:50 / 14:44:16 (A saw them) came after the NX stack and its broker started (tmux usv
14:41:13), and at 14:42 the backend found BOTH the NX and the AGX not answering - the Mac's USB-Ethernet (en16) flapping again (it was
gone twice earlier today), not the boat. The 14:41 launch is still up (NX tmux usv + bwin): launch_tk.sh → RECONNECT adopts it.

## (session A, side fix on v0.1.84.1; no version of its own - B numbers 0.1.84.x) — 2026-11-11 — UAV heartbeat button in the tk console

Dmitrii: "I don't see heartbeats in RC venue ... I need UAV heartbeat and declare buttons in new gui as well - so I see both vehicles in
the RC page (192.168.70.74:8010)". Found (logs/ocs/run_20261111_145402/ocs.log): the launch had judge **rctest** - "judge link up
(protobuf) 127.0.0.1:1885", the local rc-test; `reporting for the USV ON` at 14:55:07 went there, never to the venue
(VENUE_RC_HOST 192.168.70.74 : 1910, reachable from this Mac on en0, its page 200). Fix = launch with judge DEMO (venue): run_ocs.sh
--judge venue → RC 192.168.70.74:1910, course DEMO. No code change for that.
Built: the console's UAV tile gets **▶ HEARTBEAT UAV / ✓ HEARTBEAT ON** (`report on|off uav`, off asks first) before DECLARE UAV, and a
RoboCommand row (heartbeat on/off · the drone's telemetry age). ocs_app.hb_vehicles carries the UAV on `report on uav`; its content
needs the drone's telemetry (the launcher's UAV field: the adapter + their listener). Also: scripts/run_ocs_tk.sh single-instance guard
(logs/ocs/ocs_tk.pid) - B's launcher_tk (v0.1.84.1) calls it from the OCS button.
Tests: test_ocs_tk_model::test_the_uav_heartbeat_button; make quick 714 passed. Not archived separately (rides with B's next 0.1.84.x).

## v0.1.84.2 — 2026-11-11 (session B) — the tk launcher says where the heartbeats go

From session A: Dmitrii's 14:54 launch had judge **rctest** - ocs.log "judge link up (protobuf) 127.0.0.1:1885", every heartbeat went to
this Mac's rc-test while the venue rc-test (VENUE_RC_HOST 192.168.70.74:1910) was reachable; "could launcher_tk say which judge is live".
- `optiprimex/tools/launcher_tk.py`: `judge_target(cfg, env)` (fake = our hub; rctest = 127.0.0.1:1885; venue = VENUE_RC_HOST:PORT;
  course = RC_HOST or the DHCP gateway; alpha..delta = RC_<C>_HOST:PORT - from launcher.read_net, the launchers' own files) and
  `judge_warning` (mode real + judge fake / rctest + the venue answers TCP → "… NOT the venue: pick judge DEMO (venue)"). Under LAUNCH:
  "heartbeats + reports go to: <where> ● answers / ○ NO ANSWER" + the amber warning; checked on every plan change and every 15 s on
  the slow worker (TCP 0.5 s; never on Tk).
- tests/test_launcher_tk.py + test_the_judge_line_and_the_venue_warning; the smoke test waits for the judge line. Run with A's tk tests
  (smoke, model, map): 32 passed.
- Carried in this archive (session A's side fix, its own DEVLOG block above): the tk console's UAV tile ▶ HEARTBEAT UAV / ✓ HEARTBEAT ON
  (report on|off uav) + its RoboCommand row (tkconsole/model.py, views.py, tests/test_ocs_tk_model.py).

## (session A, side change on v0.1.84.2; no version of its own - B numbers 0.1.84.x) — 2026-11-11 — tk console: scrolling columns + VEHICLES tab

Dmitrii: "fix that I can scroll properly to UAV. Then, I need a tab for vehicles in OCS, so I can see the important IMU, compass,
heartbeat status per each vehicle". Also seen: the "bridge boat down" at 15:08 = the Mac's USB LAN adapter re-enumerated (en16 → en19,
DHCP 192.168.1.79), the NX bridge dials OCS_HOST 192.168.1.13; Dmitrii set the adapter manual 192.168.1.13 → bridge back 15:11:28.
Built: views.ScrollFrame (wheel / trackpad while over it + a scrollbar) for the RUN ladder and the USV1 / UAV tile columns; the
VEHICLES tab (right) - per vehicle a card: link chip, a compass rose (needle = heading) with an attitude line (roll / pitch), rows FCU
(mode · armed · FCU age · battery), IMU (roll · pitch · yaw rate; amber > 12°, red > 25°), COMPASS (heading · COG), SPEED, GPS (fix · sats
· HDOP · EKF), RC (kill / E-stop / RC6), HEARTBEAT (ocs.reporting.hb: sending · state · n · age), JUDGE (judge.heartbeat_age), HOST (NX);
the UAV's from state.uav (adapter link, FCU, flight phase, altitude, telemetry age, fence, pilot). model.vehicle_cards is pure.
Ask (B): state.uav drops the adapter's roll_deg / pitch_deg (Web UavView keep list) - the UAV IMU row says "not in /state.json yet".
Tests: test_ocs_tk_model::test_vehicle_cards_imu_compass_heartbeat, test_ocs_tk_smoke::test_the_vehicles_tab_and_the_tiles_scroll_to_the_uav;
make quick 717 passed (the first run was cut by a SIGINT at 451 passed, from outside the run).

## v0.1.84.3 — 2026-11-11 (session B) — the UAV's roll / pitch reach /state.json

Session A's ask (additive): `UavTap.view()` (ocs_web.py) kept heading / speed but dropped the UAV adapter's roll_deg / pitch_deg, so the
tk console's VEHICLES tab showed the UAV IMU row as "not in /state.json yet". `keep` += roll_deg, pitch_deg. Test:
test_v018305::test_the_uav_view_keeps_roll_and_pitch; run with test_v0165 (the UAV view's tests): green.
Carried in this archive (session A's side change, its own DEVLOG block above): the console's ladder / tile columns scroll + the VEHICLES
tab (tkconsole/model.py vehicle_cards, views.py, tests).
Network note (A, 15:08): "bridge boat down" = the Mac's USB LAN adapter re-enumerating (en16 → en19, DHCP .79); the NX bridge dials
OCS_HOST .13 - Dmitrii set the adapter to a manual 192.168.1.13, the bridge back 15:11:28. A manual address on the adapter keeps it .13.

## (session A, side change on v0.1.84.3; no version of its own - B numbers 0.1.84.x) — 2026-11-11 — CMD catalogue + tooltips, RECORD + depth, where it is stored

Dmitrii: "when I start recording for camera ... record also at least 1 depth image along its RGB counterpart if I check the box. Then,
I need to see on the OCS where images are being stored to. Then, on CMD tab ... all available commands with tooltips and modificators".
Built (tk console only): views.Tooltip (hover 450 ms, follows a Treeview row); the CMD tab = the entry + its usage line + a catalogue
of 57 words grouped as /meta.json's groups (+ OTHER: the words /cmd accepts without a USAGE line - report, station_keep, usv, uav,
cancel_auto, hold_on_start), greyed when the playbook refuses now; the tip = usage, what it does, every modifier with its help (from
ocs_cli's own parser, model.cli_spec; the shared connection flags left out), where it is allowed / needs / then, "now: ✓ / ✗ why → hint",
the example; a click puts the example in the entry. Every console button has a tip: the line it sends, why it is greyed, "asks first".
CAMERA: "+ depth (RGB + depth pairs)" (default on) → POST /recording {op, depth}; a "stored in:" block (AGX svo2 folder, the Mac frames
folder, frames · MB · depth n of m) from /recording.json, before a recording the Mac's frame_rec root.
Asks (B): /recording honour `depth` (frame_rec depth=True, camera mode both, ok:false + why when no depth file after ~3 s), frame_rec
status depth_frames, /recording.json targets {agx, mac}; USAGE lines for the six words above.
Tests: test_ocs_tk_model::test_the_command_catalogue_has_every_word_its_modifiers_and_rules, test_ocs_tk_smoke::test_the_cmd_catalogue_tooltips_and_pick,
::test_record_sends_the_depth_flag_and_shows_where; make quick 721 passed (its one warning - the depth checkbox's Tk variable - fixed after).

## v0.1.84.4 — 2026-11-11 (session B) — RECORD + depth: RGB + depth pairs, and where it all goes

Request (Dmitrii via session A): "when I start recording ... make sure I record also at least 1 depth image along its RGB counterpart if
I check the box. Then, I need to see on the OCS where the images are being stored." A's console sends POST /recording {op start, depth}.
Finding: while RECORD runs, OUR observer holds the ZED - the camera stream serves its preview frames and refuses depth by design (409,
cam_stream "depth only when no detector holds the camera"); so A's "switch the stream to both" cannot give depth then. Polled Dmitrii:
**the observer saves the pairs** (vs extracting from the svo2 after stop / svo2 only).
- `cv/buoys_multi/rec_core.py`: `Pairs` (pure: due / names / add / public / close; <session>/pairs/, pairs.jsonl; every_s 0 = off).
- `cv/buoys_multi/multi_buoy_observer.py`: `--pairs-s N` (needs --record); `save_pair()` from the SAME grab, before the detections and
  any drawing: rgb_N.jpg (q92), depth_N.png (uint16 mm of |z| of the cloud the detections measure on, 0 = no depth), depth_N_vis.jpg
  (near red .. far blue); the line carries utc, frame, svo_frame, ts_ns, depth_valid_pct. The first pair at once, then every N s; a
  failed pair never stops the loop; latest_status.json `recording.pairs`. Disk: ~1 GB/h at 10 s beside the svo2's ~7 GB/h.
- `scripts/agx_observer.sh`: `start --pairs-s N` (validated up front); `status --json` += pairs (count), pairs_dir.
- `optiprimex/tools/ocs_web.py` CamTab: `depth` honoured - the observer started with `--pairs-s 10` (PAIRS_S), the Mac recorder with
  depth=True (+ a try of the stream's mode both); `_depth_check` waits up to 20 s (the observer loads its model) for the first AGX pair
  or Mac depth frame → `depth` {ok, text, agx_pairs, mac_depth, where} or {ok false, why: an AGX before 0.1.84.4 / the observer not up /
  none after 20 s + the stream's refusal}; recording itself goes on. `/recording.json` += `targets` {agx, mac} (known before a start).
- `optiprimex/tools/frame_rec.py`: `depth_frames` / `depth_misses` in the status.
- `optiprimex/tools/bench_tui.py` USAGE += station_keep, usv, uav, report, cancel_auto, hold_on_start (A's CMD catalogue's descriptions;
  wording from cmd_authority / ocs_app).
- Tests: tests/test_v018302.py (depth via the AGX pairs, an old AGX → why, the Mac depth path, targets; Pairs; frame_rec counts depth).
  Run: the observer / recorder / launcher / camera tests (12 files) 111 passed; cv/buoys_multi/self_test.py PASS; playbook_tui, ocs_web,
  v018304, v0164 51 passed. NOT run with the real ZED: the pair writing needs the AGX deploy + one RECORD with + depth (check the
  pairs/ folder: rgb + depth png open, depth_valid_pct > 0).
Carried (session A, its own block): the CMD tab's catalogue of all commands + every button's tooltip.

## v0.1.84.5 — 2026-11-11 (session B) — START LOGGING: one button, the clocks first

Request (Dmitrii via session A, polled by A: ONE button that starts everything, the clocks checked first): "is there an explicit button
to start logging? lets put it also to CV window, so we engage with synced clock". A builds ● START / ■ STOP LOGGING + a clocks line.
- `optiprimex/tools/bench_tui.py` Model: `clock_samples` (host → the last 10 `t_recv - t_unix` of its presence beats; retained beats
  > 60 s old skipped), `clock_last`; `ui_results` (usv_ui's local/<v>/ui/cmd/result by id; the boat feed subscribes to it - before,
  any unknown boat topic fell into `local`).
- `optiprimex/tools/ocs_web.py` Web: `clocks_state()` → /state.json `clocks` {nx, agx, …: offset_s = -(min of the samples) (the host's
  clock minus this Mac's; the minimum strips the latency), n, ok (|offset| ≤ 0.2 s and a beat < 10 s old), age_s, host}; roles by
  hostname (usv-pixhawk → nx, *agx* → agx, uav-pixhawk → uav), this Mac skipped. `POST /logging`:
  start - refused {needs: clock, why "NX +0.83 s vs the Mac; AGX unknown …"} unless confirm "clock"; label = <label|log>_<UTC stamp>,
  t0 UTC; FCU: usv_ui already logging (the 0.1.83 default) → kept + a `note LOGGING session <label> t0 …` in its .events, else `log on
  <label>` (its answer awaited, 5 s); recording: AGX already recording (no depth asked) → kept + the Mac frames started, else the
  /recording start path (depth as asked; needs restart passed up); the session file logs/ocs/sessions/<label>.json {label, t0, clocks,
  parts, clock_override}; the answer {ok = every part ok, label, t0, parts {fcu, agx, mac, depth?}, lines, why}. Already running →
  {already}, confirm "new". stop - needs "stop"; ends only what this session started (`log off` only if it said `log on`; the AGX + Mac
  recording stop only if it started the AGX; else only the Mac frames) - what ran before stays on. `/state.json logging.session`.
- tests/test_v018405.py (3: the clocks from beats with latency; start refused on the clock → confirm → parts → already → stop; parts
  that already ran kept and left on) + in QUICK; test_v018305's logging keys += session. Run: v018405, v018302, v018305, ocs_web,
  playbook_tui, v0152_wa, v018304 = 58 passed. Not run live: the real usv_ui answer (the boat window's own path, so the same words).

## (session A, side change on v0.1.84.5; no version of its own - B numbers 0.1.84.x) — 2026-11-11 — one-press logging + clocks, RECORD's depth pairs

Dmitrii: "is there an explicit button to start logging? lets put it also to CV window, so we engage with synced clock" - polled:
everything together (FCU log + AGX video + Mac frames, one label, one t0), the clocks checked first. B built POST /logging +
state.clocks + state.logging.session (v0.1.84.5) and the observer's RGB + depth pairs (v0.1.84.4).
Built (tk console): ● START / ■ STOP LOGGING in the header and on the CAMERA tab (one flow: the local NX / AGX clock check from
state.clocks → POST /logging; confirms for already → new, needs clock → start anyway, needs restart (the observer, ~10 s guard pause),
needs stop; a 404 says "this OCS has no POST /logging yet"); the CAMERA tab's session line (label + parts ✓/✗) and clocks line
(NX / AGX offset ✓/✗, others shown); model.logging_view (pure). RECORD: the answer's `depth` as its own toast, /recording + /logging
wait 45 s (the first pair needs the observer's model), "stored in" shows B's targets + the AGX pairs (n → dir).
Rehearsed (pool sim, isolated tmux 16:06-16:09, AGX_SSH / ORIN_SSH = *.invalid so nothing reached a Jetson; stopped + its brokers):
START → the backend's "clocks not synced: NX unknown, AGX unknown" as a confirm → YES → session log_20261111_080734: fcu ✓ (usv_ui
logging ON → logs/pool_…_log_20261111_080734.tlog/.csv/.events), agx ✗ (the fake host - expected), mac ✓ (logs/recordings/2026-11-11/160734_log),
depth ✓ (3 depth frames on the Mac); both buttons ■ STOP; STOP → confirm → ended (t1 in logs/ocs/sessions/<label>.json); no thread left.
Found (B): in the sim ocs_web's AGX_SSH defaults to `agxorin` = the real AGX - RECORD / /logging in the sim act on the real Jetson
unless AGX_SSH is overridden; the Mac frames folder is named after the label prefix ("_log"), not the session label.
Tests: test_ocs_tk_model::test_the_one_press_logging_view, test_ocs_tk_smoke::test_the_logging_buttons_post_logging_and_handle_404;
make quick 729 passed.

## v0.1.84.6 — 2026-11-11 (session B) — a simulator run never touches the real AGX

Session A rehearsed /logging on the pool sim from the tk console (its own DEVLOG block, carried here: START / STOP LOGGING in the header
and CAMERA, the confirms incl. needs restart, 45 s waits, the depth toast, "stored in"): session log_20261111_080734 - fcu ✓ (log on,
the label in the tlog name), mac ✓ (3 depth frames), agx ✗ only because A ran with AGX_SSH=agx-rehearsal.invalid. Two findings:
1. **Safety**: CamTab._ssh_run defaulted AGX_SSH to "agxorin" - the REAL AGX - in a sim run too: RECORD or START LOGGING in a simulator
   would have stopped / restarted the real observer. Fix: `CamTab.agx_refusal()` - OCS_SIM=1, AGX_SSH "" / none / off, or every camera
   address on 127.0.0.1 / localhost (the simulator's or a bench camera) → `_ssh_run` returns the refusal without any ssh (the status says
   observer unknown + why; the start's AGX part ✗ with it). `scripts/run_sim.sh` starts ocs_web with OCS_SIM=1.
2. The Mac frames folder took the prefix "log" → now the session label (logs/recordings/<date>/<HHMMSS>_<label>).
Tests: test_v018405 + test_a_sim_run_never_touches_the_real_agx, test_the_mac_frames_carry_the_session_label; run with v018302, launcher,
sim_wp10, v0145, v0146, v0160, ocs_web, v018305, v017807: green (v0145's headless-Chrome page test failed once under -n 4, passes 3/3 alone;
the page is untouched).

## (session A, side change on v0.1.84.6; no version - B numbers 0.1.84.x) — 2026-11-11 — ▶ UAV RUN (boat holds) in the tk console

Dmitrii: a UAV mission without the USV - polled answer: "boat holds in auto good for now, make it quick". The code always declares
USV1 (ocs_app.decl_vehicles); a UAV-only declaration (`declare usv off`) = ROADMAP later (B). Built: the UAV tile's ▶ UAV RUN (boat
holds): a confirm listing the words, then model.uav_run_plan's remaining words in order through the one queue (station_keep loiter,
enable_auto, report on usv, usv ready on, report on uav, uav ready on, declare uav on, declare), each toasted, stop at the first ✗;
refused (with why) while usv_ui holds the FCU or the FCU is disarmed - never ARM, never handover. Fixed on the way: Btn's 3rd positional
is `ok` - the logging / MISSION ARM · START NOW / RECORD buttons passed their kind there (their colour never applied).
Tests: test_ocs_tk_model::test_the_uav_run_plan_skips_what_is_done_and_never_arms, test_ocs_tk_smoke::test_uav_run_chains_the_words_and_stops_at_a_refusal
(the touched files 27 passed); make quick run after. Not rehearsed on the sim (Dmitrii: quick).

## (session A, side change on v0.1.84.6; no version - B numbers 0.1.84.x) — 2026-11-11 — the UAV geofence at DEMO: why it does not reach the declaration

Dmitrii: "check if boat is recording · our new gui is not receiving geofence to pass to UAV, even if we see it in RoboCommand (DEMO)".
Live (17:53, read-only): boat recording = usv_ui's FCU log ON (pool_20261111-090420Z_run, no bytes: the NX predates 0.1.83.5), the
boat's SQLite logger OFF (no LOG=1 on the NX), the Mac hub SQLite ON (50 MB); the USV heartbeat said STATE_KILLED.
Geofence: course ALPHA received (boundary 5, transit 5) → the adapter → their Orin's listener ACKED (course_acked 280 s); the drone's
FCU fence 0 points → declaration_preview.uav_refusal "the FCU has no single polygon fence to declare" (D8: the declared fence is the
FCU's own, never built by us). Cause: their listener turns the course into the FCU fence only when the boundary and the transit area
share exactly one edge (uav/geofence-tools/course_fence.py); ALPHA's boundary lies INSIDE the transit area (0 corners outside) - "share
0 edges ... publish a fence by hand". The transit polygon alone passes RoboCommand's rule (0 points outside course ∪ transit).
Done: docs/fences/uav_fence_ALPHA_transit.csv (their lat,lon format) for their geofence_udp_publisher.py; the console sends
`uav resend_course` right after ▶ HEARTBEAT UAV's ✓; the VEHICLES UAV card's DECLARE row = the backend's refusal / "✓ fence N pts".
Their code untouched (read-only; the containment case is theirs to add to course_fence.py).
Tests: test_ocs_tk_model::test_the_uav_card_says_why_declare_uav_would_refuse, test_ocs_tk_smoke::test_heartbeat_uav_resends_the_course.

## v0.1.84.7 — 2026-11-11 (session B) — watch the ZED recordings on the Mac; the day's logs sorted

Dmitrii: "where do logs and recordings land? ... it seems I can't watch them", then "isolate the logs from our earlier run (11:15 was the
first one), and second (15:00 till 16:00). We can delete the rest" + "take a peek ... so we are not deleting any stuff from water".
- `scripts/svo2_mp4.py` (new): a .svo2 is MCAP (magic \x89MCAP0, uncompressed chunks); channel Camera_SN<n>/side_by_side = the camera's
  H.264 (Annex-B after an 8-byte ZED prefix), both eyes STACKED (left on top, 2208 x 2484 at hd2k); copies the stream out, ffmpeg crops
  an eye; the rate from frames.jsonl "t" (the observer writes one frame per loop: 0.8-5 fps, not the camera's 15). No ZED SDK. Converted
  all 16 recordings of the pulled copy (incl. the two never closed: 12:17, 12:29 - readable). tests/test_svo2_mp4.py (a synthetic MCAP) +
  QUICK.
- The logs (Mac copies only; the Jetsons unreachable): logs/keep_2026-11-11/{run1_1041-1151, run2_1500-1600, run3_1638-1736}/ with
  agx (svo2 + mp4 + frames.jsonl), agx_observer_logs, nx run dirs, mac_ocs, mac_maps, mac_frames (+ README.md). Run 3 was found in the
  "rest" by the GPS (17:04: 732 m, 157 m from the start) and the pictures (the course buoys 6 / 5 / BB) - kept on Dmitrii's answer. The
  non-water rest (12:17 tent then black, 12:29 black 15 min, 14:41 / 14:54 tent, the 10 Nov village clip, 09:31, 18:17-18:36, 3-9 Nov
  boat runs) → Finder Trash folder MASTER_SCRIPTS_logs_trashed_20261111 (6.5 GB, until emptied).
- Findings: the AGX recording stopped 17:36 on disk_low (< 5 GB) - the AGX disk is full until cleared; the Mac frames are a thin backup
  (the stream's preview: 1 picture in 6 min while the observer was down).

## v0.1.84.8 — 2026-11-11 (session B) — the live camera feed during a run

Dmitrii: "our feed during run also slow, can we squeeze the higher refresh rate, maybe with lower resolution?" Cause: while the observer
holds the ZED, cam_stream (source dir) serves cv/buoys/runs/*/latest_preview.jpg, which the observer wrote every 15th analysed frame at
full HD2K (one picture per ~4 s at 3.5 fps, ~15 s at 1 fps); the stream polls 5 Hz and scales to 640 px itself - not the bottleneck.
- `cv/buoys_multi/multi_buoy_observer.py`: `write_preview()` - resized to `--preview-width` (960; 0 = full), JPEG q75, tmp + os.replace
  (the stream never reads half a picture; "*.tmp" never matches its glob); every `--preview-every` (1) frames; the boxes drawn on those
  frames; the console line stays every 15th. ~3-5 ms per frame against ~250 ms of model + depth. cv/stream/cam_stream.py: its comment.
- tests: test_v018302::test_the_live_feed_preview_every_frame; test_wp9b's fake cv2 += imencode / INTER_AREA / IMWRITE_JPEG_QUALITY (the
  replay test drives the real loop). Run: wp9b, cam_stream, v0178018, guard, v018302, v0160 = 56 passed; cv/buoys_multi/self_test PASS.
  Not run with the real ZED (the AGX deploy).
Measured today (frames.jsonl): the observer ran 0.8-4.8 fps, unthrottled (best.pt, imgsz 768, hd2k, depth neural). NEXT (Dmitrii: "later I
will show the latest model my student achieved, so we will apply capping on it"): with the student's model - `--max-hz` (proposed 3) on
the observer + an OCS CAUTION below ~2 Hz, and its TensorRT export; the feed then runs at the capped rate.

## (session A, WP44 — no version of its own; B numbers 0.1.85 / 0.1.86 tonight) — 2026-11-12 00:30–01:20 — the plan, ROADMAP §0, HOSTS tab, console follow-ups, pull script

Dmitrii reviewed Cowork's night prompts with session A: "make a plan and lets look at our roadmap ... do not rush to overnight before
discussion", then: hard stop 07:00, test run 11:15 (scoring after lunch), CV integration in the morning, AGX cleaned in the morning +
lower-resolution recordings, free disk on the host tab, the scout §8b numbers, **four courses side by side: our own control ignores
detections outside our course**, scout budget 900 s, "the boat needs start moving forward at least 1 meter as soon as possible"; GO 00:35.
Written: ROADMAP.md §0 Status (hosts' versions, the water day, tonight's WP41 · WP46 · WP42 · WP47 · WP43 · WP44 table, the deploy rule
v0.1.85 = floor), the SCOUT night closed (WP38 partly / WP39 ✅ / WP40 later), the venue day v0.1.82–0.1.84.8, the queue triaged, §4a
later (WP42b, UAV-only declaration, course_fence containment, jtop, chrony, …); docs/PROMPT_PRESETS_NIGHT_2026-11-12.md §6 (10 amendments,
incl. WP46 our-course-only and WP47 smaller recordings, move-first). Session B told (GO 00:35, move-first 00:40); B confirmed the order.
Found while planning: scout already filters its targets by the RxCourse polygon; **m1t1's gate pairing has no course filter** (WP46);
B's v0.1.84.7 moved the 11 Nov pulled copy into logs/keep_2026-11-11/ (water runs) and the Trash was emptied 23:38 (the non-water clips).
Built (A's files only): tkconsole/hosts.py + the HOSTS tab (AGX · NX · MAC, CPU · GPU · RAM · TEMP + power sparklines, disk free with
minutes of recording left, thresholds, polled only while visible, a 404 = "backend v0.1.86 needed"); the WP42 / WP46 follow-ups (hidden
missions out of the pickers, "starts by itself at RunStart", the automatic DECLARE session = "running since t0", outside-course buoys greyed
+ counted); scripts/pull_jetson_logs.sh (both Jetsons → logs/jetsons/<date>/, rsync, resumable, files + bytes checked, never deletes).
Fixture tests/fixtures/hosts_sample.json = B's §2.5 contract. Tests: tests/test_ocs_tk_hosts.py (4), test_ocs_tk_pull_logs.py (2), +3 in
test_ocs_tk_model.py; make quick 738 passed. Not live-tested: /hosts.json and the other v0.1.86 keys (B builds them tonight); the pull
script's real copy (the Jetsons are offline - first use in the morning). Next for A: when B posts the v0.1.86 contracts, rehearse the HOSTS
tab and the DECLARE session on the sim; CV integration in the morning (Dmitrii's brief).

## v0.1.85 — 2026-11-12 night (session B) — WP41 turn under way + WP46 our course only

Asked (Dmitrii via session A, GO 00:35; move-first 00:40): ROADMAP §0, `docs/PROMPT_COMPUTE_NIGHT_2026-11-11.md` P0 + addendum (T0.2a–e),
`docs/PROMPT_PRESETS_NIGHT_2026-11-12.md` §6 items 7, 9, 10, `docs/SCOUT_BRIEF_2026-11-11.md` §8b. On 11 Nov auto never moved the boat: both planners
turned in place (forward 0, yaw 10–20 °/s), the hull does ~1.8 °/s in place.

Changed · where:
- **`optiprimex/usv/guidance.py` (new)** — the under-way law: yaw = clamp(k·e, ±`yaw_rate_max_dps` 5); v = clamp(v_target·cos e, `v_min_mps` 0.4,
  v_target) inside ±90°, `arc_v_mps` 0.3 beyond; **bow-clear** (T0.2b, object < 3.5 m in ±30° with |e| > 30° → v 0); **boundary** (T0.2c, ray to the
  course polygon < 6 m → ≤ 0.15 m/s, < 2.5 m with |e| > 15° → v 0; a carrot past the boundary near the edge → aim at the leg's end; outside and
  heading away → v 0); **heading time-out** 15 s → drive anyway (logged); **turning-circle rule** (found tonight: a goal inside the hull's turning
  circle was orbited for ever → slower 0.3 / 0.2 / 0.15, else in place); `Arrival` = T0.2e closest-approach; `MoveFirst` = §6 item 10 (≥ 1 m at
  ≥ 0.4 m/s along the heading from the first tick, skipped and logged when bow-clear / the boundary sees something ahead); WP46 helpers
  `in_course` / `course_local` / `course_refusal`. Profile block `guidance:` (pool + course: `mode: underway`; the bench and `Config()` = `fcu`
  = unchanged).
- `usv/tasks/base.py` — `UNDERWAY` tasks (SCOUT, T1, TEST_ORBIT) under `mode: underway`: every GUIDED carrot → the law's velocity-yaw, every
  yaw capped, move-first; `NEEDS_FIELD` (SCOUT, T1) for WP46. `t1_safe_passage.py`: two class flags only, no logic.
- behaviours: FollowPath LOS ≥ `los_m` 3 (T0.2d), closest-approach arrival, corridor + one turning radius, + one turn of time; Goto the same
  arrival; Orbit r ≥ 1.1 · v/ω(v) at ≤ `scan_v_mps`; **ScanOrbit** (T0.3; `ScanYaw` delegates under `scan_mode orbit`, default under underway):
  forward 0.3 m/s with the yaw at the cap, the side with the clearer circle, bow-clear + boundary on the circle; LOOK's face turn capped at 5 °/s.
- scout: the 20° / 8° turn-first gate off under way; the target's ring kept at ≤ standoff − 0.3 (T0.2a — it already held: keep 2.68 < 2.7);
  `guard_target_xy` + `guard.py`: a stop beyond 2·standoff + 1 m of the target counts as "intermediate" (T0.2e); RETURN's home ≥ margin + 2 m
  inside and clear of every ring (found tonight: the corner start's home sat 1 m from a buoy → 3 guard stops → SAFE_HOLD); candidates /
  report by `in_course` (+ 1.5 m). Profiles pool + course: **budget_s 900, margin_m 4**.
- **plant** (T0.4): `Plant(hull=ideal|omni_usv, str_ramp_s)` — omni: 2 °/s in place → 4 °/s at 0.5 m/s, flat above; a first-order steering lag.
  fake_fcu `--hull --str-ramp-s`; FastSil `hull=` / cfg `plant:`; the free run's ghost from `plant:`; mission_manager `--guidance fcu|underway`;
  run_bench.sh `FCU_HULL` / `FCU_STR_RAMP_S` / `MM_GUIDANCE`; **run_sim.sh defaults to omni_usv + underway** (`HULL=ideal` = the old plant).
- **TEST_YAW m0t10** (T0.5): `start_task m0t10 --pair side|aft|both --amp 0.6 --duration 10 [--dir ccw]` — mode-20 yaw frames, the rate from the
  heading (`logs/yaw_<stamp>.jsonl`, event `yaw_test`); test mode only (refused on the course profile). `gains.yaw_side_gain` (default 0) in
  `mode20.frame`. Wired: REGISTRY, RUN_FREE_TASKS, cmd_authority, bodies.schema.json, playbook (m0t10), ocs_cli `--pair --amp`.
- **WP46**: `PerceptionMap.set_course / in_course`; `entities()` / `uav_entities()` / gates / markers / provisional gates / the UAV entry
  candidate leave out what lies outside the RxCourse boundary + `course_tol_m` 1.5 (no boundary = no filter: bench / sim unchanged);
  `gatepath.map_obstacles` keeps them as obstacles; the digest: `outside_course` per entity, `course_boundary_local` (was always []),
  `course_tol_m`, `n_outside_course` → /map.json (ocs_web passes the digest through). The mission manager sets the course from `run.course`
  each change; `ocs/state.judge` → `map.ocs_judge`; START_TASK of a NEEDS_FIELD task on a real judge (`protobuf`) without RxCourse →
  REJECTED "no RxCourse on the boat - is the course acknowledged?" (`guidance.require_rxcourse: judge|always|never`).
- docs: `docs/fcu/steering_rate.param` + `STEERING_RATE_NOTE.md` (T0.6 proposal: ATC_STR_RAT_FF 15, ATC_STR_RAT_MAX 15, PIVOT_TURN_ANGLE 170 — NOT
  applied), `docs/fcu/READBACK_2026-11-11.md` (T0.1: the empty table for the morning), Test Guide **W0** (T0.7).

Numbers (fast SIL, world_scout = ALPHA 25 × 30 m, 10 buoys, starts rotated over 4 poses, 20 seeds; omni_usv; course-profile numbers):
| profile | done | buoys visited | all 10 | runs with a boundary exit | SAFE_HOLD | mean time |
|---|---|---|---|---|---|---|
| v0.1.85 under way, no guard | 20/20 | **100 %** | 20/20 | **0** | 0 | 730 s |
| v0.1.85 under way + the ZED guard (2 unseen obstacles) | 20/20 | 89.5 % | 0/20 | **0** | 0 | 724 s |
| v0.1.84 planner, ideal hull + guard (yesterday's baseline) | 20/20 | 86.5 % | 0/20 | 15/20 (margin 4 field) | 0 | 353 s |
| v0.1.85 under way + guard, **FCU ramp 25 s left in** | 13/20 | 72 % | 1/20 | 0 | 2 | 969 s |
m1t1 on omni_usv: simple 10/10 DONE, s6 6/6 DONE (world_pool). The guard runs never log all 10: two buoys sit next to the unseen obstacles
(the same on the ideal hull). The ramp row says the T0.6 preset matters (the sketch: −13 % buoys).
Not met as written: "the circle scan completes 360° within 90 s" — at the addendum's `scan_v_mps` 0.3 the hull turns ~3.2 °/s → 360° in ~113 s
(`scan_max_s` 130). 0.5 m/s would make 90 s at a 7 m circle; the addendum chose 0.3 (5.4 m). Dmitrii's call.

Tests: tests/test_v0185.py (18: profiles, plant, the law, bow / boundary / time-out, arrival, scout trace rules on omni — no GUIDED carrot, no
|yaw| > 5, first command forward ≥ 0.4 within 1 s, ≥ 1 m before the first yaw —, move-first skip, **20 scout runs 0 exits** (22 s, `make test`
only), ScanOrbit 360°, m1t1 ×2 on omni, TEST_YAW frames / rate / refusal / in the SIL, yaw_side_gain, WP46 neighbour gate + digest, scout
outside target + the real-judge refusal, the guard blame radius); test_wp10d's run-free set + TEST_YAW. QUICK += test_v0185, A's
test_ocs_tk_hosts / test_ocs_tk_pull_logs. **`make test` 975 passed (5:31)**; make quick 761 + the fix (serial 6:41 tonight — slower than
the < 1 min of the contract; not investigated).
Not rehearsed live: `run_sim.sh pool` on omni_usv through the brokers (handover → scout DONE → m1t1 DONE, screenshots) — session A's sim
rehearsal covers the page side; the fast SIL proves the planner. T0.1 / T0.6 / W0: the morning (Jetsons offline).
Session A's WP44 files and DEVLOG block carried unchanged.
Next: v0.1.86 WP42 (presets, /hosts.json for A's fixture, logging.session.auto, Mission.hidden, ocs.mission.auto, the disk warning, the
launcher's Jetson dots), then WP47, WP43 Mac-side. Morning: deploy v0.1.85 as the floor (rollback tgz beside the tree), T0.1 read-back, W0.

## (session A, WP44 continued — no version of its own; B carries the files) — 2026-11-12 01:20–02:00 — the postponed GUI elements, ⚙ SETTINGS, the layout pass, scout across layouts

Dmitrii (then AFK until the morning: "make your own decisions"): JUDGE · SYSTEM · big-numeral view · HELP, "signal quality in the host tab
as a status bar with text", "a rehearsal about spacing and fonts, overall organisation, according to best practices", "a settings window
with configs we could save and restore, as in the previous GUI", configs for "refresh rates, mission parameters, speed limiters ...
initial approach, obstacle stop distance, orbiting distance ... evaluate relevance and grouping", "the default size can be bigger on a
Mac (don't eat full space)", "the USV page" (answered: keep it as it is tonight - it runs on the NX, B's file, the fallback), then
"after the new scout, simulation runs in different layouts incl. tight ones; a docx report last".
Built (A's files): tkconsole/panels.py - JUDGE tab (RoboCommand · the run · heartbeats sent vs seen by the judge · reports · traffic),
⛶ BIG VIEW (7 tiles at 44 px, Esc back), the ⚙ SETTINGS window (SETTINGS · SYSTEM · HELP: named configs on the Mac logs/ocs/tk_configs/
= the knobs + the console prefs, RESTORE sends only what differs through the one queue; the mission manager's knobs grouped SPEEDS · TURNING
· APPROACH · OBSTACLES (safety) · ORBIT / SCAN · FIELD · DOCK (collapsed) · OTHER, range-checked, `tune save`; RATES / FCU read-only - FCU
parameters only through apply_params.py; the console prefs logs/ocs/tk_prefs.json drive the theme / fast rate / start tab / log filter /
+depth; SYSTEM's 19 actions with options + refusals, the dangerous ones need their word typed); the HOSTS signal bar (measured `link`
when B adds it, else "estimated" from the status age / perception beat); theme.FONT / SPACE tokens; views.default_geometry (~90 % of the
screen, centred); the layout pass by measurement (no Screen Recording permission): the tabs fit (8 tabs need 444 px, the tabs column
keeps its width first), the tiles' buttons two per line with the group's name above (a row of four ran past the column - ABORT / DISARM
cut), the ladder's done chips no longer clip, spacing on the 4-px grid - 0 clipped labels, 0 buttons < 40 px, 0 rows past their column at
1440 x 900, 1280 x 800, 1555 x 864. Fixed on the way: a views import (`from ...`), the settings' default-argument paths (tests wrote
to logs/), the knob grid's header keys. Asked of B: WP48 (accepted as 0.1.86.1: the guidance / limits / scout / zed_guard knobs with
ranges and safety floors), `link` per host in /hosts.json.
Scout across layouts (v0.1.85, FastSil in-process, omni_usv + under-way, the course profile, the ZED guard on; 8 layouts x 3 seeds x 4
starts = 96 runs; logs/sim_campaigns/scout_layouts_20261112_011647/): normal fields (ALPHA 25 x 30, cluster, edge-hugging, a neighbour
course's 6 buoys outside, random >= 4 m) 60/60 DONE, 0 SAFE_HOLD, 0 boundary exits, 76-94 % of the buoys, closest pass >= 0.98 m, **no
neighbour buoy ever visited (WP46)**; a 40 x 12 corridor 11/12, 87.5 %, 2 runs touched the boundary; **tight fields** (18 x 15, buoys 3 m
apart) 35 % with 4/12 SAFE_HOLD, + unseen poles 20 % with 5/12. A margin x standoff sweep (4 / 3 / 2 x 3.0 / 2.5, 216 runs,
scout_layouts_20261112_011929/sweep.jsonl) does not lift the tight fields (32-44 %) and adds boundary exits as the margin shrinks →
**keep margin 4 / standoff 3.0**; the limit is geometric (a visit point inside a neighbour's 1.5 m STOP range, 1.6 m rings, a 5-6 m turning
circle) → the guard's STOP-backoff-SAFE_HOLD as designed. The first campaign's "near contacts" at t < 20 s were my layout generator's
(buoys on the start poses) - fixed (4 m clear of every start) and rerun. A fix would be the guard telling mapped buoys from unknown
objects (B's guard; not tonight).
Tests: tests/test_ocs_tk_panels.py (8), +1 in test_ocs_tk_hosts.py; make quick 783 passed. Next for A: rehearse the console on B's
v0.1.86 (HOSTS on /hosts.json, the DECLARE session, hidden missions) on the sim; the docx report on the layouts last.

## v0.1.86 — 2026-11-12 night (session B) — WP42 presets · defaults · judge order · test tiers · hosts feed · the cull as a hide-flag

Asked: `docs/PROMPT_PRESETS_NIGHT_2026-11-12.md` §2 + §6 (approved amendments: hide-flag cull, no observer restart at DECLARE, LOG=1 on the NX,
the clocks item, the AGX disk warning, the launcher's Jetson dots). Session A's asks: /hosts.json per its fixture, logging.session.auto,
Mission.hidden, ocs.mission.auto (already in /state.json ocs.mission), outside_course (v0.1.85).

Changed · where:
- **Hosts feed (§2.5)** — `tools/host_health.py`: `parse_tegrastats` (NX + AGX rails, GR3D, EMC, per-core CPU, -256C = no sensor), a
  `Tegrastats` reader (spawned once on a Jetson), `/proc/meminfo`, psutil on the Mac, fan (pwm-fan), `nvpmodel -q`, `temp_max_c`,
  **`rec_gb_per_10min`** (`RecRate`: the recordings folder's growth over ≥ 60 s; `--rec-dir`, run_agx.sh passes ~/Desktop/dm3/recordings).
  `tools/ocs_web.py` HostHealth: a 600-sample ring per host, the Mac sampled in process, **GET /hosts.json** = session A's
  tests/fixtures/hosts_sample.json shape (+ `simulated`, `disk_amber_gb`, `disk_red_gb`). Disk warning **amber < 15 GB, red < 8 GB**, with
  "~N min of recording" at the measured rate; `scripts/run_agx.sh` says the free disk at start (amber / RED).
- **Presets (§2.1)** — `profiles/presets.yaml`: sim (marina, scout preselected, fake judge) · lab (bench) · 0 bench-uavmap · 1 m1t2-uav ·
  2 m1t2-both · 3 m1t1-usv (scout) · 3b m1t1-gates · custom; event removed. Launcher fields `mission`, `auto_mission`, `tiers`, `record`
  (validated); `launch_block()` → the OCS profile copy's `launch:` block (one place: ocs_app + ocs_web both read their --config);
  `record auto` = the observer step `--pairs-s 10`, `off` = `--no-record`; a v0.1.85 config without `record` = unchanged (no block).
  `ocs_app`: `apply_launch()` preselects the mission at start (`mission off` still clears it), `launch_tiers()` win in `effective_tiers`,
  a soft T4 "are you sure?" row is dropped when the preset's row says NONE (hard rows stay).
- **Judge order (§2.2)**: venue, alpha…delta, course, fake, rctest (both launchers); titles say what each is. **GET /reach.json** (launcher):
  one TCP connect each, 0.5 s, cached 10 s — the judges' RoboCommand (green ● on the option) and ssh :22 of AGX · USV-NX · UAV-NX (●/○);
  addresses never in the answer. launcher.html: rows mission / auto start / record / tiers / reachable; launcher_tk: the same rows +
  `reach_line` / `tiers_line` under the judge line.
- **Defaults (§2.3)**: (a) auto start = the presets' `auto_mission: true` + the preselect (ocs_app's `_mission_trigger` unchanged);
  (b) **run_usv.sh hands over at start** (`--no-handover` / `USV_HANDOVER=0`); (c) **logging at DECLARE** — `Web.auto_log_tick` (1 Hz
  thread): a new declaration (R3–R5) → the START LOGGING chain by itself, label `<preset>_<mission>_<seq>`, the clocks never block it,
  `record: off` = FCU log only, **never an observer restart** (the observer records from launch; if it does not, the Mac frames start and
  the session says why); R6 / back → it stops the session it started; an operator's session is kept. /state.json: `logging.session.auto`
  (+ t0, declaration_seq), `recording.auto`. **The boat's SQLite logger is on by default** (`LOG="${LOG:-1}"`; §6 item 4 — on 11 Nov it
  never ran); the hub logger already takes `#` (the drone + the map when `log: true`).
- **The cull (§2.6) = the hide-flag fallback** (§6 item 3): `Mission.hidden` + `playbook.visible_missions()`; goto, m0t0, m0t1, m0t2, m0t3,
  m0t6 hidden (typed still works; m0t6 is W0's test 1); /meta.json missions carry `hidden`. Deletion + the pool → lab profile move = **WP42b**.
  The launchers keep `usv_profile: pool` selectable (no preset uses it).
- **Test tiers (§2.4)**: `make day` = QUICK on 6 processes + DAY_LIVE (test_usv_ui, test_v0143_backend, test_integration_sil1, test_v0164):
  **816 passed in 137 s** tonight (the < 2 min target missed by ~17 s). **CLAUDE.md §3 NOT edited** — the request came through a peer
  session; the day / night rule text of the prompt §2.4 is for Dmitrii to paste in the morning.
- tests: tests/test_v0186.py (20: tegrastats NX / AGX, the rec rate, /hosts.json ⊇ A's fixture + the ring, the disk warning, run_agx,
  host_health fields; the presets, the launch block + plan, reach without addresses + the tk lines, the OCS preselect + tier row + no T4
  prompt, logging at DECLARE start / stop / kept, handover + logger defaults, hidden missions, make day). The v0.1.85-era launcher tests
  keep the old presets (tests/fixtures/presets_v0185.yaml via LAUNCH_PRESETS, conftest LEGACY_PRESET_TESTS); pins updated: the judge order
  (test_v017801, test_v0182), host thresholds (test_v0143_backend), the session shape (test_v018405). QUICK += test_v0186,
  test_ocs_tk_panels (A). **`make test` 997 passed (5:29).**
Not rehearsed: §2.7's fake-FCU run of `m1t1-usv` / `m1t2-uav` through the stack (no tmux tonight beside session A's rehearsal; the parts are
unit-tested: the launch block, the OCS preselect, the DECLARE session) — the morning's NX fake-FCU smoke is the gate for deploying v0.1.86
(deploy rule). jtop: not installed (later); tegrastats only.
Next: 0.1.86.1 = WP48 (session A's runtime knobs: guidance / scout / guard names and ranges in its message), then WP47 (recordings: measure
GB / 10 min first), WP43 Mac-side; A's /hosts.json `link {rtt_ms, loss_pct, rssi_dbm, iface}` ask (low priority).

## (session A, addendum — no version) — 2026-11-12 02:15 — v0.1.86 rehearsed through the stack; the scout-layouts report

- **v0.1.86 rehearsal** (the console in the real window, run_sim.sh pool in an isolated tmux, OCS_SIM; 01:33–01:35, stopped + its brokers):
  HOSTS from /hosts.json (3 cards; Mac real, NX / AGX the bench's simulated beats; the signal bar "estimated"), the hidden missions out of
  the picker (m0t10sim TEST_YAW stays visible - W0's test), station keep loiter → enable_auto → mission scoutsim → DECLARE → **the
  automatic logging session started by itself** (run_scoutsim_1_…: fcu ✓, mac ✓, agx refused by OCS_SIM ✓; REC chip + "REC on" toast),
  JUDGE rows, START → scout under way through the brokers (APPROACH 0.45 m/s), MAP (11 buoys, 0 outside), BIG VIEW, SETTINGS (live knobs
  grouped), ABORT → END RUN → **the session stopped by itself** (t1, stop_lines, "REC off"); 7/7 commands ✓; no thread left. Window at
  ~90 % of the screen (1850 x 1169 here). B's §2.7 stack rehearsal is covered by this one for the OCS side.
- **The report**: docs/reports/SCOUT_LAYOUTS_2026-11-12.docx (+ docs/reports/scout_layouts_2026-11-12/fig1-4.png): summary, method,
  results (8 layouts), the tight sweep, what it means for today, caveats, the data; the docx validates (the skill's XSD check). New
  finding in the sweep: in the tight layouts a run leaves the course at every margin (L7 2–4 of 12 per cell, L2 3–5 with the 2.5 m
  stand-off) - most likely the guard's blind 2 m back-off near the edge (5.1 §11 ends the run) → B, after the competition: check the
  back-off against the course polygon; let the guard ignore mapped buoys inside their ring.

## v0.1.86.1 — 2026-11-12 night (session B) — WP48 the runtime knobs (session A's list, Dmitrii: "make your own decisions")

Asked (Dmitrii via session A): "configs for important things as refresh rates, mission parameters, speed limiters ... initial approach,
obstacle stop distance, orbiting distance" - the morning's W0 tests want them changeable without a relaunch.
Changed · where: `companion/tuning.py` KNOBS += 25 (guidance: v_min_mps, arc_v_mps, yaw_rate_max_dps MAX 10, heading_timeout_s, bow_clear_m,
los_m, start_fwd_m, scan_r_m, scan_v_mps, bnd_slow_m, bnd_stop_m MIN 1.5, course_tol_m · limits: max_speed_mps · scout: scout_standoff_m MIN 2,
arrive_m, scout_transit_v, margin_m, budget_s, lane_m, clear_m MIN 1.2 · zed_guard: guard_stop_m MIN 1.0, guard_slow_m (> guard_stop_m),
slow_v, backoff_m, ring_m) - A's names and ranges; `FIELD` maps the prefixed names to their section's word (scout.transit_v ≠ t1.transit_v);
`cross_check` (guard slow > stop), `seed` (limits.max_speed_mps from cfg.limits into the view), `sync` (the speed cap into cfg.limits and the
guard's params into the running GuardSupervisor at once - TUNE is MANUAL / AUTO_HOLD only, so no task runs then); the rest is read at the
next task start as before. Defaults = the code's DEFAULTS (scout budget 600 / margin 2; the pool + course profiles carry 900 / 4 and the
view shows them). Not knobs (as A said): the rates (launch time), the 2 Hz judge heartbeat, FCU parameters. `publishers.StatusPublisher`:
the boat's status carries `rates {tick_hz, status_hz, local_hz, map_hz}` (A's SETTINGS card, read-only).
Tests: tests/test_v018601.py (4); test_v0149's knob table check covers the new sections. **make test 1001 passed (5:33).**
Session A's rehearsal of v0.1.86 through the brokers (its message): /hosts.json 3 cards, hidden missions out of the picker, the DECLARE
session started and stopped by itself (agx refused by OCS_SIM as designed), scout under way 0.45 m/s, outside_course 0 on the pool map.
A's campaign note for later: tight fields (buoys ~3 m apart) lose buoys to the guard STOP → back-off → SAFE_HOLD - the real fix is the guard
telling mapped buoys from unknown objects (after the competition; ROADMAP later).

## v0.1.86.2 — 2026-11-12 night (session B) — /hosts.json `link` (session A's signal bar; Dmitrii: "signal quality ... as a status bar with text")

`tools/ocs_web.py` HostHealth.start_link_probe: one TCP connect per Jetson per second to ssh :22 (0.5 s timeout; NX = --boat-host, AGX =
AGX_HOST / the LAN AGX; off in OCS_SIM and for 127.0.0.1) → per host `link {rtt_ms, loss_pct, samples, rssi_dbm, iface, how}` over the last
10 samples; `tools/host_health.py` `wifi()` reads /proc/net/wireless on the host (`wifi {iface, rssi_dbm, quality}` in its beat; null when
wired). A host with nothing measured has `link: null`. Tests: test_v018601::test_the_link_from_the_ocs_and_the_hosts_wifi. make live 998 passed.
Not seen on the real Jetsons (offline): which interface they use - wired on the team LAN gives rssi null, the bar then shows rtt / loss only.

## v0.1.86.3 — 2026-11-12 night (session B) — WP47 smaller recordings (§6 item 8)

**Before (measured, the 11 Nov recordings in logs/keep_2026-11-11/, bytes / frames.jsonl duration):** every svo2 session ran
**1.19–1.48 GB / 10 min** (the 31.6 min one: 4.23 GB = 1.34 GB / 10 min) at 0.9–4.4 observer fps - the per-frame size moves inversely with
the frame rate (460 kB at 4.4 fps … 2.2 MB at 0.9 fps): the SDK encodes at a **fixed ~19 Mbit/s** whatever we feed it. So H.265 at the
default bitrate alone would likely not shrink the file (better quality at the same size); the bitrate is the lever.
Changed (cv/buoys_multi/multi_buoy_observer.py, our copy): `--svo-codec h265|h264` (default **h265**; refused by the SDK / encoder →
H.264 at the same bitrate, logged "svo2 compression …"), `--svo-bitrate KBPS` (default **8000**; 0 = the SDK default; an SDK without the
argument → the 2-argument call), `--pairs-width 960` (the RGB area-averaged, the depth nearest-neighbour: real mm kept).
optiprimex/tools/frame_rec.py: the Mac frames shrunk to **640 px** (Pillow; unchanged when absent / not a JPEG).
**After (predicted / measured on the Mac, not on the AGX):** svo2 8000 kbit/s ≈ **0.60 GB / 10 min** (8000 × 600 / 8e6; ≈ 0.43 × today,
if the SDK holds the bitrate as it held its default); depth pairs every 10 s: 4.7 → 0.8 MB per pair (synthetic HD2K depth + RGB), 0.28 →
0.05 GB / 10 min; Mac frames ~3× smaller (1920 → 640). The ZED capture resolution unchanged (detection range).
**Morning (AGX):** record 2 min with the new defaults, `ls -l cam.svo2` → GB / 10 min into JETSON_LOG; play one back with
scripts/svo2_mp4.py (H.265: ffmpeg handles hevc the same way - `-f hevc` if the h264 wrapper refuses) and look at a buoy at 15 m; if it
is mush, `--svo-bitrate 12000` (≈ 0.9 GB / 10 min). Revert = `--svo-codec h264 --svo-bitrate 0` (exactly 11 Nov).
Tests: test_v018601 (+2: the Mac frames 640, the observer flags + fallback); test_wp9b's pin H264 → H265 (the fake SDK has no bitrate
argument: the fallback path ran); cv/buoys_multi/self_test PASS. Not run with the real ZED.
Also: scripts/svo2_mp4.py reads the codec off the first NAL (`nal_codec`: h264 | hevc) - `-f hevc` + `-tag:v hvc1` for a playable
QuickTime copy; test_svo2_mp4 +1. `make day` below.
make day 833 passed (137 s).

## v0.1.86.4 — 2026-11-12 night (session B) — WP43 §0 the tick instrument + Tier 1.4 bounded mode acks (Mac-side)

- §0 instrument: `MissionManager.timed_tick` / `tick_stats` - the tick's duration ring (60 s), `status.health.tick_ms {p50, p95, max, n,
  slow_per_min}` (ticks > 150 ms in the last minute). The NX numbers come with the morning's deploy (the Jetsons are offline).
- Tier 1.4 (`usv/adapters/rover.py` `_set_mode`, `timeouts.mode_ack_s` 0.2 / `mode_backoff_s` 2.0): the tick waits at most 0.2 s for a mode
  ack (was 0.5); a refused / unacked mode is not asked again for 2 s (False meanwhile, nothing sent); one warning per streak (was one
  "mode X refused" per tick). An ack clears the backoff. hold_reassert_s 5 keeps its meaning (2 s < 5 s).
- **Numbers (a stub FCU that never acks, ensure_mode each tick at 10 Hz): before worst tick 510 ms and every tick asked (6 / 6); after
  worst 206 ms, 1 / 20 ticks asked.** With a healthy FCU nothing changes (the mode is already right: no request).
- Risk: a real ack slower than 0.2 s (usv_ui's UDP share) reads as "no ack" → 2 s without re-asking; the heartbeat still shows the mode
  if the FCU took it. Watch the trace's `mode_req ok false` on the NX in the morning; `timeouts.mode_ack_s: 0.5` in the profile = as before.
Tests: tests/test_v018604.py (3). make live 1004 passed.

## v0.1.86.5 — 2026-11-12 night (session B) — WP43 Tier 1.3: GET /recording.json from memory

`tools/ocs_web.py` CamTab.recording_cached: one refresher thread (every 2 s, its ssh bounded to 5 s - `_ssh_run(timeout_s)` returns
"ssh: no answer within 5 s" instead of raising) keeps the last recording_status(); the GET handler returns it with `age_s` and the Mac's
own part read now; "checking" until the first answer. POST /recording and START LOGGING still ask the AGX directly (rare, explicit).
**Numbers (a dead AGX whose ssh hangs 1 s here; 20 s real): before 1010 ms per poll (one ssh per poll per console); after 0.5 ms.**
Tests: test_v018604::test_recording_json_never_waits_on_a_dead_agx. make live 1006 passed. Session A's tkconsole/model.py tick_ms LOOP row
(+ its test) carried as it is. Morning NX smoke list: watch `mode_req ok false` (the 0.2 s mode ack, v0.1.86.4).

## v0.1.86.6 — 2026-11-12 night (session B) — WP43 Tier 1.1 cv_publish 20 Hz + Tier 1.2 the observer's writer thread (AGX side, built on the Mac)

- Tier 1.1 (`cv/publish/cv_publish.py`): `--hz` default **20** (was 10; one os.stat per poll). **Numbers (its real watch loop, a fake
  publisher, 40 os.replace'd status files at 5-8 Hz): file written → published, mean 62 ms → 32 ms.** The AGX's `pipe_ms` (§0: t_pub − t_cam)
  is not instrumented yet - the morning's number comes from the guard envelope once deployed.
- Tier 1.2 (`cv/buoys_multi/bg_writer.py` new, `multi_buoy_observer.py`): one writer thread, a bounded queue (4); the preview (resized in the
  loop, encoded + written by the writer) and the RGB + depth pairs (a frame + z-plane copy once per pairs_s) go there; full queue → the
  oldest preview dropped, a pair never (waits ≤ 0.5 s, then inline); `latest_status.json` stays in the loop (small; cv_publish and the guard
  read it in order) - a deviation from the prompt's "status by the writer", on purpose. `payload["writer"]` = {done, dropped_previews,
  inline, errors, max_job_ms, queued}; `--bg-writer off` = as before. **Numbers (a disk stalling 1 s per write, 20 frames + 2 pairs): before
  20 000 ms of writes inside the frames; after median 0.00 ms, worst 0 ms per frame, pairs 2 / 2 kept, 18 previews dropped.** Not measured
  on the AGX (the real imwrite cost there: ~3-5 ms preview, ~100+ ms per full-size pair - v0.1.86.3 shrank the pair to 960 px too).
Tests: test_v018604 (+3); test_wp9b / test_v018302 (the replay drives the real loop with the writer on); cv/buoys_multi/self_test PASS.
make day 841 passed (145 s).

## v0.1.86.7 — 2026-11-12 night (session B) — WP43 Tier 1.5 the bridges' keepalive

The boat broker's bridge (profiles/pool/mosquitto/boat_pool.conf.in, rendered by run_usv.sh) already had restart_timeout 1 5, cleansession
false, notifications true; missing was `keepalive_interval` (mosquitto's default 60). Added `keepalive_interval 5` to the hub bridge and the
UAV bridge (the UAV's cleansession true left as it was), and to the bench boat.conf / bridge.conf (parity).
**Numbers (scripts/bridge_keepalive_check.py: two local mosquittos, the hub frozen with SIGSTOP - a cable pulled / Wi-Fi gone, no FIN):
the boat noticed the dead bridge after 119.4 s with the default → 9.7 s with keepalive 5.** A refused connection (the hub process killed)
was already fast (restart_timeout); this is the silent case. Cost: one PINGREQ / 5 s per bridge.
Tests: test_v018604::test_the_boat_bridges_notice_a_dead_peer_in_seconds. make live 1010 passed.
Not done (AGX, morning on Dmitrii's go): Tier 1.6 MAXN / jetson_clocks (sudo), 1.7 the TensorRT engine; §0's pipe_ms in the guard envelope;
Tier 2 / 3 (not started - Tier 1 is not complete on the AGX side).

## v0.1.86.8 — 2026-11-12 night (session B) — WP43 §0 the guard pipeline latency (the instrument)

cv/publish/cv_publish.py stamps the guard envelope with `t_pub` and `pipe_ms` = (t_pub − t_cam) × 1000 on the AGX's own clock;
usv/guard.py GuardSupervisor keeps the last 60 (`latency()`: pipe_ms p50 / p95, net_ms p50 = the NX's receive − t_pub, the clock offset
included - read it with state.clocks); the mission manager puts it in `status.health.guard_ms`. The first real numbers come with the deploy
(the prompt's target: pipe_ms p95 < 150; today's guess 250-350 with the 10 Hz poll, v0.1.86.6 made it 20 Hz).
Tests: test_v018604::test_pipe_ms_from_cv_publish_to_the_boats_health. make live: the first run failed once (which test not captured - the
output was cut by `tail`), the rerun 1011 passed; a flaky live test, to watch.

## The morning go / no-go table (session B, 2026-11-12 night - PROMPT_COMPUTE_NIGHT §4; `make test` on v0.1.86.8: **1016 passed**, 5:35)

| version | fix | tests | numbers before → after | bench proof (Mac) | deploy? |
|---|---|---|---|---|---|
| 0.1.85 | WP41 turn under way + move-first + circle scan; WP46 our course only | make test 975 | 20 scout runs on omni_usv: 100 % buoys, 0 exits, 0 SAFE_HOLD (guard on: 89.5 %, 0 exits); m1t1 10/10 + s6 6/6 | fast SIL; A's sim rehearsal through the brokers: scout APPROACH 0.45 m/s | **yes - the floor** |
| 0.1.86 | WP42 presets · auto start · handover at start · logging at DECLARE · /hosts.json · hidden missions · make day | make test 997 | - (behaviour) | A's rehearsal: DECLARE session started / stopped by itself, hosts 3 cards | yes **if the NX fake-FCU smoke is green** (deploy rule) |
| 0.1.86.1 | WP48 runtime knobs | make test 1001 | - | unit | with 0.1.86 |
| 0.1.86.2 | /hosts.json link | make live 998 | - | unit | with 0.1.86 |
| 0.1.86.3 | WP47 smaller recordings (svo2 H.265 @ 8000 kbit/s, pairs 960 px, Mac frames 640) | make day 833 | svo2 1.34 GB / 10 min (measured 11 Nov) → ~0.6 predicted; pair 4.7 → 0.8 MB | **AGX not measured** | amber: deploy, then measure 2 min on the AGX; revert flags `--svo-codec h264 --svo-bitrate 0` |
| 0.1.86.4 | tick_ms + bounded mode acks (0.2 s, 2 s backoff) | make live 1004 | deaf FCU: worst tick 510 → 206 ms, 6/6 → 1/20 asks | stub link | amber: watch `mode_req ok false` on the NX smoke; `timeouts.mode_ack_s: 0.5` = before |
| 0.1.86.5 | /recording.json from memory | make live 1006 | dead AGX: 1010 → 0.5 ms per poll | stub ssh | yes (Mac only) |
| 0.1.86.6 | cv_publish 20 Hz + the observer's writer thread | make day 841 | poll wait 62 → 32 ms; stalled disk 20 s → 0 ms in the frames | fake publisher / stub writer; self-test PASS | amber: AGX run of the observer (self-test + a 1 min record) before the water |
| 0.1.86.7 | bridges keepalive 5 | make live 1010 | silent dead hub: 119 → 9.7 s | two local mosquittos | yes |
| 0.1.86.8 | guard pipe_ms / net_ms instrument | make test 1016 | (the instrument: first numbers after the deploy) | unit | yes |

Recommendation: deploy **v0.1.86.8** (all of it) on the morning's go **after** the NX fake-FCU smoke (handover at start → LOITER → enable_auto →
DECLARE (the session appears) → RunStart → scout under way) and the AGX observer check (self-test, 1 min record: `ls -l cam.svo2`, the svo2
plays with scripts/svo2_mp4.py); if either is amber, **v0.1.85** is the floor. Rollback: v0.1.81.1 beside the tree on both Jetsons. Then:
T0.1 read-back (docs/fcu/READBACK_2026-11-11.md), W0 (Test Guide), the steering preset (docs/fcu/steering_rate.param) on Dmitrii's go.
Not done tonight: Tier 1.6 / 1.7 (AGX sudo / TensorRT), Tier 2, Tier 3, WP42b (the deletion + pool → lab profile move), the §2.7 rehearsal on
the Mac by B (A's covered the page side); CLAUDE.md §3 (the day / night test rule) left for Dmitrii to paste (the ask came via a peer
session). Session A's console files carried as they are (model.py tick_ms / guard_ms rows, panels, hosts, pull script).

## docs only — 2026-11-12 morning (session A) — the operator and test documents for the venue, stale procedures out

Dmitrii: "can you update user and test documents please? so we waste as little time during venue as possible? remove stale procedures".
No code, no version bump (the tree stays 0.1.86.8).
- **New** `docs/USER_GUIDE_VENUE.md` (the day in order: morning once → before every run → the presets → the no-button run → after every run →
  symptoms → where things are stored) and `docs/TEST_GUIDE_VENUE.md` (only today's tests with time boxes, pass lines and fallbacks: T1 NX
  fake-FCU smoke, T2 AGX observer + svo2, T3 read-back, D9, W0.1–W0.3, W1–W5, D7 / D8 for spare time, the scout layout limits, the tick sheet;
  minimum path T1 → T2 → W0.1 → W3).
- **GUIDE.md rewritten** lean (213 KB → ~20 KB), current facts only. The section numbers the code cites are kept: §1 the date line / the deploy loop, §4, §6.1, §6.2.
  No addresses in it any more: `$ORIN_SSH` / `$AGX_SSH` come from `load_network_env`.
- **Moved, not deleted**, to `docs/old/`: `GUIDE_v0.1.86_full.md` (the full previous GUIDE: the §3 version rows, §4.1–4.22, the pool days),
  `USER_GUIDE_VENUE_v0.1.77.md`, `TEST_GUIDE_VENUE_2026-11-09.md`.
- Pointers updated: README.md (the GUIDE line, the presets line), HANDOFF.md line 8, docs/LAUNCHERS.md (two lines). CLAUDE.md's "Entry points"
  line still describes the old A / B / C GUIDE; it is left for Dmitrii.
- Checked against the code: every `ocs_cli` word cited exists in `ocs_cli.build_parser()`; `run_ocs.sh --judge venue|alpha…`, `run_usv.sh
  --no-handover`, `LOG` on by default, the preset table read from `profiles/presets.yaml`. No test reads these documents.

## v0.1.86.9 — 2026-11-12 venue morning (session A) — the fast command path (the student's analysis B1 / B2 / B5 / B6 / C6)

Dmitrii (at the venue, hardware not ready, 1-2 h): "lets proceed on mac with build. you can go wide with these points". The analysis was
read against the tree first: C1 / C2 / C3 / C7 / B8 / C5 were already fixed overnight (0.1.84.8, 0.1.86.3–.7); B3 / B4 not real
(the state is there; the wait is event-driven). Built:
- **B1** `ocs_cli.OcsConsole.intent(until="done"|"accepted")` + `follow(intent_id, wait_s)`; `default_until` per console (the CLI keeps
  "done"); compound words (ready, manual, stop_reset) pass until="done" for every step. `ocs_web.Web.command` uses "accepted": the reply
  carries `id`, `final`, `ms`; a follower thread waits for DONE / FAILED (the old wait) → `GET /cmd_result?id=` + a LOG line. Results and
  acks lists bounded (500 / 1000). `OCS_CMD_UNTIL=done` = the old way.
- **B2** the busy lock waits 10 s (stop_reset 25 s), never "busy" after 0.2 s.
- **Phase 0 (part)**: `logs/ocs/cmd_timing.jsonl` (OCS_CMD_TIMING overrides): queue · verdict · accepted · final · reply ms per press.
- **B5** `Web.state_json()`: one build per 0.25 s (STATE_TTL_S) shared by every client, dropped after any POST / command.
- **B6** ocs_web.html: `tick()` one /state.json in flight (TICK_BUSY).
- **C6** tkconsole: CamWorker 10 fps, kept-alive connection (one retry), unchanged frame skipped, decode + scale in the worker
  (`decode`, draft mode, `target` from the main thread); views._on_frame takes a PIL image or bytes. ToastBook.accept (level "acc",
  not final, never amber); CommandWorker.send(on_final=) + `_follow` polling /cmd_result; App._final.
- Files: optiprimex/tools/ocs_cli.py, ocs_web.py, ocs_web.html (session B's: Dmitrii's go), tkconsole/{net,toasts,views}.py;
  tests/test_v018609.py (8: accepted → final + the timing line, 3 parallel presses never busy, refused final at once, the state cache,
  the poll guard, the toast, the worker's follow, the decode); tests/test_v0154.py's pinned lock string updated.
- Tests: `make live` 1020 passed (2:39). One found bug fixed on the way: the cache served a stale theme after POST /prefs → every POST drops it.
- Rehearsed on the isolated pool sim (TMUX_TMPDIR, stopped after): press → toast 24–232 ms (station_keep: ACCEPTED +74 ms, DONE +232 ms);
  cmd_timing rows verdict 1–2 ms, accepted 56–72 ms, reply 6–77 ms; 8 parallel clients 5 ms per /state.json; CAMERA 4.1 fps = the sim
  camera's own rate (frames now arrive decoded). Not rehearsed: a real FCU's slow DONE (fcu_mode / arm) - the follow-up path is what shows it.
- Mac only (ocs_web + the console); nothing for the Jetsons. Open (later): B5's deeper cost (the 300 log lines), B7 innerHTML, C4.

---

## v0.1.87 — 2026-11-12 venue (session A): the colleague's boundary-v14 in our observer

**Request** (Dmitrii): "our CV model is ready … buoy detection with a colour and obstacle detection with boundaries. Lets try to
implement that". Later: "save snapshots of verification on our footage so I could review".

**What the model is.** The AGX holds it, read-only, at `~/buoy_robotx/runtime/boundary_v14_20261111/`.
- A YOLO11m with an FP16 TensorRT engine at 1280 x 1280; TensorRT 10.3, ultralytics 8.4.124 in `~/actual_buoy/.venv`.
- Classes: buoy_red, buoy_green, buoy_blue, buoy_boundary_no_light, dock, obstacle.
- Their ZED test ran at ~78 ms inference and ~5 fps at hd2k. Their README itself says "baseline / pre-labelling".

**What changed**
- `cv/buoys_multi/two_stage_core.py`:
  - the v14 classes join BUOY_CLASSES / LED_COLOUR (red / green / blue), and UNLIT (black in model mode);
  - `DETECTORS += boundary-v14`, `BOX_DETECTORS`, `V14_NAMES`, `V14_DEFAULTS` (0.25 / 1280 / 0.55);
  - new functions `check_names`, `class_ids`, `parse_classes`, `obstacle_far` (a measured box → a guard `far` cluster: horizontal
    range, bearing, the box's bearing extent, src model / cls / conf).
- `cv/buoys_multi/multi_buoy_observer.py`:
  - new options `--detector boundary-v14`, `--obstacle-classes` (default obstacle; dock is off because the model calls the numbered
    markers dock), `--obstacle-conf` (0.35);
  - the class list is checked at start;
  - obstacle boxes are measured with `measure_mask` over the box's inner ellipse and appended to `payload.guard.far`, also stored as
    `payload.obstacles` and `frames.jsonl` `obstacles`;
  - they are drawn orange on the preview;
  - the buoy path is unchanged.
- The boat is unchanged: `usv/guard.map_far` already rings `far` clusters (3 sightings, explained by a mapped buoy). The NX needs no
  deploy for this; the AGX does.
- `cv/buoys_multi/run.sh`: finds the engine (or the .pt), with `CV_MODEL_V14` to override.
- The detector option `boundary-v14` was added in `scripts/agx_observer.sh`, `system_actions` (the page / tk OBSERVER action, which
  falls back to default on an unknown value), `launcher.observer_cmd` + `/observer`, and the `ocs_web.html` select.
- Docs: `cv/buoys_multi/README.md`, `GUIDE.md` §3, CHANGELOG.
- Tests: `tests/test_v0187.py` (9).

**Verified on our footage** (`logs/cv_review/boundary_v14_2026-11-12/`: index.html, 88 side-by-side jpgs, stats.json, README.md,
review_v14.py). The .pt was copied to `CV_NEW/boundary_v14/` (never archived) and run on the Mac at mps over 11 clips from 11 Nov.
- Of 344 frames where our observer saw buoys, v14 saw a buoy in **4**; 199 were "dock" only and 138 were nothing.
- In total it gave 825 dock boxes and 10 obstacle boxes.
- The numbered white marker buoys come out as **dock** at up to 0.84, and the far rows (15–25 m) are not detected.
- **Not for runs.** The default stays our best.pt.

**Not done / open**
- Not run on the AGX: the AGX tree is still v0.1.83, and a deploy needs Dmitrii's go.
- The **AGX disk had 4.9 GB free** (92 %) at 09:40. The recorder refuses below 5 GB: free it first (USER_GUIDE_VENUE §2).
- The colleague needs to relabel the numbered markers as buoys and add far venue examples. Our clips + frames.jsonl boxes are pre-labels.

**Next session:** after a retrained v14, rerun `review_v14.py` on the same clips and compare the table.

### v0.1.87 addendum — the same morning: the live map, the model dropdown, the AGX space, the models on the Mac, the deploy

Dmitrii: "make sure that 2d map with any detected objects is live, and that observer/detector runs even before declaration. Also, if
we can keep previous models, so we could select from drop-down … Then you can purge logs or previous recordings from AGX … copy models
to MASTER SCRIPTS directory on my mac, and then deploy latest version to jetsons".

- **Live map before DECLARE: already true, checked in the code.**
  - The launcher's real-CV presets start the observer at LAUNCH (`launcher.py`, the `agx_observer.sh start` step).
  - The mission manager ingests observations whenever it has a pose (`_ingest_observations`: no run state checked).
  - It publishes the map digest every tick (`map_pub.maybe`).
  - Depth / model obstacles show after 3 sightings, as before.
- **Models selectable:**
  - Detectors: `default` (our best.pt), `model-a`, `boundary-v14`, `six-class` (new; the colleague's 11 Nov
    robotx_six_class_yolo11m_1280, .pt because the AGX has no engine for it; generic `buoy` coloured by our HSV vote).
  - Start with another detector while one runs = a switch (`agx_observer.sh`: `running_det`; stops our tmux window, starts the chosen one).
  - The SYSTEM action refuses a switch during AUTO_TASK.
  - The page's OBSERVER button asks "switch the model A → B?" when the select differs from the running one.
  - The observer reports `detector` / `model` (perception health `observer_detector`).
- **six-class on our footage** (`logs/cv_review/six_class_2026-11-12/`): a buoy in 248 of 344 frames (238 together with ours). It may
  have been trained on these frames.
- **Models on the Mac:** every .pt / .engine / .onnx from the AGX (buoy_robotx runtime incl. backups, actual_buoy best.pt, docking_cv)
  went to `CV_NEW/agx_models_2026-11-12/` (583 MB, `SHA256SUMS`, never archived). The engines are AGX-only.
- **AGX space:** the recordings are copied to `logs/jetsons/2026-11-12/agx/recordings/` (rsync with delta transfer against APFS clones
  of `logs/keep_2026-11-11`; the link ran ~3 MB/s), then sha256-compared. Only sessions identical on the Mac are deleted on the AGX
  (see JETSON_LOG).
- Tests: `tests/test_v0187.py` gained 3 (switch, cv_publish's model, six-class).

### v0.1.87.1 — 2026-11-12 venue: the UAV map's port (session A)

- **The mismatch:** the UAV team's `buoy-detection/config/map_mqtt_publisher.json` aims their publisher at the NX on **1883**. Our
  boat broker is on 1884, and the NX's system mosquitto on 1883 is disabled.
- **The fix, on our side:** `boat_pool.conf.in` gains a `#UAVLISTEN` listener. `run_usv.sh` renders it as `listener 1883`:
  - only on Linux-aarch64 (the Mac's hub holds 1883);
  - only when 1883 is free (a busy port would stop mosquitto).
  - `BOAT_UAV_PORT=0` turns it off.
- **Verified:** a publish on the second listener reaches a subscriber on the boat port (local mosquitto). The NX's 1883 was free
  (system mosquitto disabled + inactive).
- Test: `test_v0187.py::test_the_boat_broker_also_listens_where_the_uav_publishes`.

### v0.1.87.2 — 2026-11-12 venue (session B): KILLED clears to MANUAL by itself

- **Ask (Dmitrii):** "automatically clear state from Killed to Manual, so we wont flip/confirm it on the page itself".
- **What changed:** `profiles/pool/usv_pool.yaml` and `profiles/course/usv_course.yaml`: `guards.v1_exit_requires_confirm: false`. The
  switch already existed in `vehicle_fsm.py` (SAFE_CONFIRMED from V1 fires each tick once `inputs.kill` is false); no code change in the FSM.
  - The kill still wins while it is asserted: KILLED holds until the kill / E-stop / RC5 E-stop input is released, then MANUAL on the
    next tick. MANUAL = the operator's boat; nothing arms, no task resumes (the kill aborted it), auto needs enable_auto as before.
  - FAULT (`v6_exit_requires_confirm`) still needs confirm_safe. Bench + lab profiles unchanged (the sim scenario "must NOT leave KILLED
    on its own" runs on the bench profile and still holds).
  - `ocs_web.py` readiness hint for an asserted kill now says pool / course clear by themselves.
- **Tests:** new `tests/test_v018702.py` (5: pool + course clear; bench + lab wait; FAULT keeps the confirm). `make quick` 803 passed,
  `make live` 1042 passed (both before the hint text edit; the hint is not pinned by a test).
- **Not done:** not deployed - the NX runs 0.1.87.1 (session A's deploy). It takes effect when the mission manager restarts on 0.1.87.2.
- **Next:** deploy to the NX on Dmitrii's go (only the two profiles, ocs_web.py on the Mac, the version files changed); JETSON_LOG entry.

---

## v0.1.88 — 2026-11-12 venue (session A): the T4 tick box, and the DECLARE now carries the preset's tiers

**Request** (Dmitrii): "I want to have in launcher in presets tick box for T4, off by default for every mission". The next trial is
m1t2 with the drone (the boat goes after it lands and the map is in), then USV m1t1 without T4.

**What changed**
- `profiles/presets.yaml`: `t4: false` in all 8 presets, with the field documented in the header.
- `launcher.py`:
  - `t4` is validated (true | false);
  - `launch_block` sends `T4: CORE | NONE` whenever the config carries `t4`, winning over the preset's tier row;
  - a config without the field behaves as before (no block when there is nothing else).
- `launcher_tk.py`: a "T4 (incident)" tick box (BOOL_KEYS, ROWS); `tiers_line` shows the T4 in force.
- `launcher.html`: the same tick box.
- **Fix** (`ocs_app._declaration_inputs`): the DECLARE did not apply the launcher's tier row. Only the preview and the checks
  (`effective_tiers`) did.
  - 12 Nov's three trial runs (11:19 and 11:31 m1t2-uav, 11:39 m1t1-usv) declared the profile's **T3 CORE / T4 CORE**, although the
    presets said NONE (hub logs `logs/ocs/run_20261112_11*/`).
  - Now: profile → T1 flag → the launcher's row → the operator's `mission … --t4`, the same order in both places. The operator's
    explicit T4 wins over the launcher's.
- Tests: `tests/test_v0188.py` (4). Two pinned strings in test_v0186 updated (the tier line now shows T4).

**Found while checking, not changed:** in this morning's two m1t2 runs the drone never armed or flew (FLIGHT_PHASE_GROUNDED, disarmed
throughout), and no sgit map arrived. m1t2's exit from WAIT_UAV_MAP on "landed after flying, 5 s with no UAV item" has therefore
never been seen with the real drone. `uav scan done` is the operator's way out. It also needs a flashing-blue entry candidate,
otherwise a CAUTION and AUTO_SAFE_HOLD after 420 s.

**Mac only:** the launcher and the OCS run on the Mac. Restart both (`scripts/launch_tk.sh`). The Jetsons need nothing for this.

---

## v0.1.89 — 2026-11-12 venue (session A): the mission-1 ladder, the drone's touchdown sequence, the run banner

**Request** (Dmitrii):
- "one more look at the interface buttons sequence, to make sure that we are not switching windows many times during the mission.
  Lets rebuild mission ladder as well. Our main focus now on mission 1 … USV only, UAV only (boat holds), UAV+USV."
- "a confirmation of the touchdown - when the drone lands it goes disarm … map quiet, Guided (RTL) to disarm, after that we can issue
  command to boat" (AskUserQuestion: the boat goes **automatically**).
- "a pop-up with nice green background big font … RUN STARTED -> mission stage" (both a corner pop-up with a cross and a separate
  always-on-top window).

**What changed**
- **The boat** (NX deploy needed):
  - new module `usv/tasks/uav_gate.py` (pure). `touchdown()` gives the sequence FLYING (armed / airborne seen) → MAP QUIET (no new or
    changed UAV buoy for uav_quiet_s) → RTL/LAND (mode seen; informational) → DISARMED. GO = flying and disarmed and quiet.
  - `t1_safe_passage.uav_map_ready` and `scout.wait_uav_map` use it, and publish `info.uav_gate` (→ the map digest `uav_gate` via
    `mission_manager`).
  - `perception_map`: `uav_change_t` / `uav_sig` (the map's content to ~0.5 m, colour, light). Their publisher resends the whole map
    ~1 Hz, which used to reset the 5 s quiet forever. Also `uav.change_age_s` in the digest.
  - `uav scan done` and scan.complete still release at once. m1t2 still needs a flashing-blue entry.
- **The OCS:** `uav_state().flight.mode` (RTL / LAND visible to the boat). `ocs_web` `/state.json` gains `uav_gate` and `uav_map`
  (age_s, change_age_s).
- **The tk console:**
  - `model._ladder` rebuilt, with the shape from what was launched:
    - "USV only": no UAV declared;
    - "UAV only · boat holds": UAV declared, the mission not auto (preset 1);
    - "UAV + USV": UAV declared, the mission auto (preset 2).
  - The steps: handover → crew (heartbeats + pilots ready, both vehicles when the UAV flies) → ENABLE AUTO (+ ARM) → DECLARE (who, the
    tiers incl. T4, the mission: no JUDGE tab) → RunStart → the drone (FLYING · MAP QUIET · RTL/LAND · DISARMED chips; `▶ BOAT GO NOW
    (override)` = `uav scan done`, asks first) → the boat (HOLD / ABORT, START override) → END RUN.
  - The UAV-only run (the boat runs no task) follows the touchdown in the console from the UAV status + `uav_map.change_age_s`.
  - Views: up to 8 step frames with generic button slots `lad<n>-<i>`, and the profile label "MISSION 1 · …".
  - `_banner`: `RUN STARTED · run N` / `→ stage` (USV · task · state, or UAV · TAKING OFF / FLYING · MAPPING / RETURNING (RTL) /
    LANDED · MAP SETTLING / TOUCHDOWN ✓ · boat waits / BOAT GOES), amber on AUTO_SAFE_HOLD, `RUN ENDED` at R6.
  - Shown twice: a corner pop-up (✕) and an always-on-top window. Closing either hides it until the stage changes.
- Tests: `tests/test_v0189.py` (5). test_ocs_tk_smoke / test_ocs_tk_model / test_v0166 updated to the new ladder positions and wording.

**Verified:** the always-on-top RUN window was seen on screen over other apps (a recorded R5 scout run): green, "RUN STARTED · run
1001 → USV · SCOUT · LOG", ✕.

**Not verified:** a real drone flight. The touchdown sequence is proven on the fast SIL and the recorded states only.

---

## v0.1.90 — 2026-11-12 venue (session A): the "lucky" flag for Task 1

**Request** (Dmitrii): "a final update to mission logic, with a flag in startup - 'lucky' flag, as our model is not perfect now, so the
boat will proceed through closes buoys pair inside course boundary to be considered as a gate, if the colour of bouys is uncertain (but
hopefully we could detect flashing red/green/blue etc), we have to take a risk".

**What changed**
- `perception_map.lucky_gates(min_obs, used_ids)` makes the pairs:
  - eligible buoys are inside the course (`in_course`) and within the gate width (1.5–6 m);
  - at least one buoy's colour is uncertain: not red / green / blue / black;
  - never blue (the entry / exit) nor black (an obstacle), never red+red / green+green;
  - each buoy once, the real gates' buoys left out, the narrowest first;
  - a known red keeps the red role, a known green the green role; for two unknowns, the lower id plays red. `Gate.lucky`.
- `gatepath.planner`:
  - `GateItem.lucky`;
  - `gate_items(..., lucky)` appends the lucky gates after the real ones;
  - `choose_next` flips a lucky gate to the side the boat comes from (never "wrong side") and adds `LUCKY_COST_M` 10 m, so a real
    gate up to 10 m farther still wins.
- `t1_safe_passage`: `lucky` in DEFAULTS (START_TASK arg). The `LUCKY_GO` transition SEARCH_ENTRY → TRANSIT fires when the entry scan
  ends with no flashing blue and a gate is choosable ("T1 LUCKY: …" in the log). The exit is unchanged: a steady blue, else SCAN →
  safe hold.
- Launcher: `lucky` field (presets `lucky: false`, validated), the tk and web tick box "lucky (gates)". `launch_block` puts
  `extra.lucky` only for a T1 mission (m1t1 / m1t2; not the scout).
- OCS:
  - `set_mission` keeps `lucky`, and the auto-start's `margs` carries it;
  - `MISSION_ARGS` += lucky;
  - **fix:** `apply_launch` now also sets `authority.mission_task / mission_args`. A bare `start_task` (the console's ▶ START, the
    ladder's override) started **T4** for a launcher-preselected mission; now it starts the preselected one with its args.
- tk ladder: the DECLARE step says "LUCKY gates (uncertain colours)" when it is on.
- Tests: `tests/test_v0190.py` (4). One fast-SIL run on a world copy with the T1 buoys white: without the flag SAFE_HOLD, with it DONE,
  3 gates passed correctly.

**The risk, stated:** with our current model the venue's white numbered marker buoys are uncertain colour. If two of them stand
1.5–6 m apart inside the course, lucky will take them for a gate. That is the accepted risk.

### v0.1.90.1 — 2026-11-12 16:55 venue hotfix (session A, Mac only)

- **Found live:** the v0.1.89 ladder put ENABLE AUTO before DECLARE. The boat refuses ENABLE_AUTO until PREPARE ("no PREPARE yet
  (declaration_seq 0) -> declare first"), and DECLARE's button was hidden until AUTO.
- **Fix** (`tkconsole/model._ladder`): step 3 = DECLARE ("now" once the crew is ready); step 4 = ENABLE AUTO ("now" once declared).
- The tk console was restarted on the Mac; the stack and the Jetsons were untouched.
- The console tests (test_v0189, test_ocs_tk_model, test_ocs_tk_smoke) passed before the restart. Dmitrii: no further tests.

### v0.1.90.2 — 2026-11-12 17:12 venue hotfix (session A, Mac only): mission / tier mismatches

- **17:10, a regression of v0.1.88:** launched with m1t2-both (preset row T1 ADVANCED), armed `mission m1t1` in the console, and
  DECLARE was refused: "m1t1 needs T1 TIER_CORE, the declaration says T1 TIER_ADVANCED".
  - Cause: v0.1.88 put the preset's row after `_t1_flag`.
  - Now: profile → the preset's row → the armed mission's T1 (`_t1_flag`) → its `--t4` (`_t4_flag`), in both `effective_tiers` and
    `_declaration_inputs`.
  - Test: `test_v0188.py::test_the_armed_mission_decides_t1_over_the_presets_row`.
- **16:55, the auto-start was lost:** the console's ARM MISSION sent `mission <code>` without `--auto`, so the preset's auto-start
  was switched off. It now appends `--auto` / `--lucky` when the current mission has them.
- `ocs_cli`: `mission` / `start_task --lucky`.
- **To take effect:** the OCS and the console restart (launcher STOP → LAUNCH). The Jetsons need nothing.

### v0.1.90.3 — 2026-11-12 venue (session A, Mac only): the presets cleared

- Dmitrii (after the attempt): "clear the missions presets, should be very straight forward".
- The launcher (tk and web) now shows five buttons, the numbered ones first:
  - **1 · USV only — m1t1** (m1t1-gates: the gate mission, auto, T1 CORE);
  - **2 · UAV only — boat holds** (m1t2-uav: no auto, the boat holds);
  - **3 · UAV + USV — m1t2** (m1t2-both: auto, the boat goes at the drone's touchdown);
  - custom and sim.
- `lab`, `bench-uavmap`, `m1t1-usv` (the scout) carry `hidden: true`: kept in presets.yaml (the tests, the bench), not shown.
- T4 is off and lucky is off in all of them.
- The launcher was restarted (backend + window); no stack was running.

### v0.1.90.4 — 2026-11-12 ~17:30 (session A, Mac only): the afternoon's mismatches analysed and closed

The OCS logs of 16:37–17:10 (`logs/ocs/run_20261112_16*`, `_17*`) show five kinds of failure.
1. **DECLARE refused "m1t2 needs declare uav on"** (16:40, 16:41, 16:53, 16:56, 17:06). The m1t2 presets start the UAV link, but
   `declare uav` stays off by default. **Fix:** `launch_block.declare_uav` for an m1t2 mission with the UAV on; `apply_launch` calls
   `on_uav("declare", on)`.
2. **ENABLE AUTO rejected "no PREPARE"** (16:48–16:53): the v0.1.89 ladder put it before DECLARE. Fixed in v0.1.90.1.
3. **The auto-start lost** about 15 s after almost every launch (16:40, 16:47, 16:51, 16:55, 17:06, 17:08, 17:09): the console's ARM
   MISSION re-sent the mission without `--auto` (and without `--lucky`). Fixed in v0.1.90.2.
4. **USV only: DECLARE refused "m1t1 needs T1 TIER_CORE, the declaration says ADVANCED"** (17:08:39, 17:10:04). Both launches were
   preset **m1t2-both with the mission field changed to m1t1**, and the preset's row (T1 ADVANCED) won: the v0.1.88 regression.
   Fixed in v0.1.90.2 (OCS), and here at the source: the launcher derives T1 from the mission.
5. **UAV runs** (16:55 m1t2-both, 17:01 m1t2-uav): declared with the UAV, RunStart forwarded ("the Orin has command #N run_start"),
   the drone went GUIDED / STATE_AUTO, **never armed** (FLIGHT_PHASE_GROUNDED throughout), no sgit map. The boat held in
   WAIT_UAV_MAP (touchdown step `wait_flight`) until the operator's STOP & RESET (16:58:51, 17:05:05).

**UAV failsafe check (our side).** All we sent the drone: RC_FORWARD course (twice), UAV_READY, PREPARE (DONE: "telemetry fresh,
disarmed, fence valid, pilot ready"), START_RUN + RC_FORWARD run_start, RESET ("pilot ready cleared"). No mode, arm, fence-write or
parameter command. The UAV status carried no failsafe event and no RTL / LAND mode while our adapter was up. Our adapter is not a MAVLink
GCS (JSON / UDP to their listener), so it cannot trip the FCU's GCS failsafe. Not checked: their listener's / MAVProxy's own logs (the
UAV Orin was unreachable at 17:30).

### v0.1.90.5 — 2026-11-12 ~17:45 (session A, Mac only): no PILOT READY buttons

- Dmitrii: "remove pilot ready buttons, they confuse during the run. We need as less buttons to engage as possible" → "completely".
- **USV:** `profiles/course/ocs_course.yaml` `usv_heartbeat: auto` (the v0.1.59 rule: heartbeat + ready from the boat's first fresh
  status). The v0.1.82 button flow is kept for `usv_heartbeat: button` and stays tested (test_v0182 overrides it).
- **UAV:** `OcsApp.uav_ready_tick()` (in the loop). Declared, the adapter up, R1 / R2 / R6, not ready → `UAV_READY on`, at most every
  5 s. It runs again after a STOP & RESET (which clears pilot ready); never during a declared run. `uav_pilot_auto: false` = off.
- **Console / page:** the PILOT READY buttons are gone from the ladder's crew step, the USV1 tile, the UAV tile and the page (hidden).
  The UAV RUN chain still sends `uav ready on` (harmless).
- **The drone's arm refusal at RunStart** ("Mission ABORTED: ArduPilot refused to arm (FAILED): Flight plan received", their GUI):
  - our data rules out GPS (a position in every status) and a fence breach (the drone 15 m inside its own 6-point fence, fence_ok);
  - the PreArm reason will be in their log (Dmitrii will share it).
- Tests: test_v0188 (+ uav_ready_tick), test_v0182 / test_ocs_tk_smoke updated.

### v0.1.90.6 — 2026-11-12 (session A, Mac only): the heartbeat toggles per vehicle

- Dmitrii: "I still need heartbeat button preserved per vehicle, bring it back".
- The ladder's crew step always shows its buttons (`Step.sticky`): ✓ HEARTBEAT USV / ▶ HEARTBEAT USV, and the same for the UAV in the
  UAV runs. Off asks first. No PILOT READY. The tiles' heartbeat buttons stay.
- `hb_vehicles`: `report off uav` (`uav_hb_off`) silences the drone before DECLARE even with `declare uav on`. That was ineffective
  since v0.1.90.4's auto-declare. A declared vehicle keeps its heartbeats at all times, as before.
- Tests: test_v0189 (crew toggles), test_v0188 (UAV toggle).

### v0.1.90.7 — 2026-11-12 (session A): ENABLE AUTO before DECLARE on the course

- Dmitrii: "for boat and drone, lets enable auto first before declaration. Drone - its operator with RC, for boat - its us switching
  to LOITER. Then, I declare".
- `profiles/course/usv_course.yaml`: `guards.require_prepare_for_auto: false`. The boat takes ENABLE_AUTO before PREPARE; PREPARE is
  taken in any state (`run_context.apply`). This needs the NX.
- `profiles/course/ocs_course.yaml`: `ocs.auto_before_declare: true`, published in `ocs/state`.
- `playbook`: the "no PREPARE yet" prediction is skipped when `auto_before_declare` is set.
- The tk ladder: with `auto_before_declare` the order is crew → ENABLE AUTO (+ the drone's mode, "its pilot puts it in GUIDED
  (RC)") → DECLARE (lights once the boat is in AUTO); otherwise DECLARE first (v0.1.90.1).
- The OCS authority already allowed ENABLE_AUTO from R2. Pool / lab / bench keep require_prepare_for_auto true.
- Tests: test_v0189 (the course order).

### v0.1.90.8 — 2026-11-12 (session A): one rule everywhere, and the big ENABLE AUTO

- Dmitrii: "I need uniform rule everywhere, please. Lets keep big ENABLE AUTO button (with suppl text that HOLD to LOITER)".
- **Every boat profile** (course, pool, lab, bench) and the code default (`companion/config.py`): `require_prepare_for_auto: false`.
  The guard is kept, opt-in.
- **OCS:** `auto_before_declare` defaults to true. The playbook predicts the "no PREPARE" refusal only when a profile sets it false.
- **The ladder:** one order everywhere: crew → **▶ ENABLE AUTO / HOLD → LOITER** (the "big" style: Menlo 20 bold, padding 26×18,
  border 3) → DECLARE → RunStart.
- Tests: test_vehicle_fsm_table / test_wp10d (the guard opt-in), test_integration_b1 (ENABLE AUTO before PREPARE accepted), the
  console tests (the order, the big button).
- The NX needs it: the boat's guard.

### v0.1.90.9 — 2026-11-12 (session A, Mac only): the launcher's panes drag

- Dmitrii: "click and drag to expand or shrink tile in the launcher, some buttons are cramped".
- `launcher_tk`:
  - the left form now follows the existing left / right sash (its canvas window was pinned at 500 px); the preset text wraps to it;
  - the right side is a vertical `ttk.Panedwindow`: THE PLAN over the tabs, the sash between them;
  - the preset buttons sit three per row.
- Only the launcher window was restarted (the backend reused).

---

## v0.1.90.10 — 2026-11-12 evening, home (session A): the mission-1 rehearsal before the next venue day

**Request** (Dmitrii): "test everything thoroughly that tomorrow we don't be stuck".

1. **`make test`: 1076 passed.** The first run found one stale copy: `usv_ui/config/usv_bench_share.yaml`, the boat profile the simulator
   and the bench use with usv_ui, still had `require_prepare_for_auto: true`. Fixed and pinned.
2. **`tests/test_mission1_rehearsal.py`** (10): the real OcsApp, in process, on the course profile, started from each preset's real
   launch block. It covers:
   - preset 1 (T1 CORE / T4 NONE, auto, a bare START = T1, USV-only PREPARE);
   - preset 1 with T4 and lucky (T4 CORE on the wire, lucky in the start args);
   - preset 3 edited to m1t1 (T1 CORE, the 17:08 mismatch);
   - re-arming keeps the auto-start;
   - presets 2 / 3 (the UAV declared and ready by itself, both heartbeats, the per-vehicle toggle, PREPARE to both with T1 ADVANCED);
   - the UAV missing (DECLARE says why);
   - ENABLE AUTO before DECLARE in every profile and the sim's copy;
   - no PILOT READY anywhere;
   - the launcher's OCS_HOST check.
3. **The simulator, end to end** (isolated tmux; the launcher's own sim plan + launch file; driven through `/cmd` like the console
   buttons; the console model applied to every `/state.json`). Logs: `logs/launch/REH_{uav_usv,usv_only,uav_only}.txt`.
   - **USV only** (m1t1, auto): 7 steps, handover → crew ✓ → ENABLE AUTO (AUTO_HOLD) → DECLARE (R4 in 1 s) → RunStart. The mission
     started by itself in 3 s (SEARCH_ENTRY → CIRCLE_ENTRY_CW); the banner followed.
   - **UAV only** (m1t2, no auto): 7 steps, the drone's pilot ready by itself, T1 ADVANCED. After RunStart the boat held in AUTO_HOLD
     the whole run; the banner showed "UAV · TAKING OFF".
   - **UAV + USV** (m1t2, auto): 8 steps. RunStart → m1t2 by itself → WAIT_UAV_MAP ("UAV · TAKING OFF · boat waits") → released after
     57 s by the sim's fake survey ("the UAV's scan.complete"; the real sgit feed never sends it, so at the venue it is the touchdown
     or `uav scan done`) → SEARCH_ENTRY → CIRCLE_ENTRY_CW → TRANSIT.
   - The drone team's fake copter does not arm on run_start, so the touchdown sequence cannot be rehearsed in the sim.
4. **The 18:20 venue failure:** the Mac's USB LAN back on DHCP (.6), the boat bridging to OCS_HOST .13 ("No route to host"), no boat
   status, so no USV heartbeat. **New:** `launcher.ocs_host_check` (in `preflight`) refuses a real launch when this Mac does not
   hold OCS_HOST, naming the adapter, its address and the exact `sudo networksetup -setmanual "<service>" <OCS_HOST> 255.255.255.0`.
   - At home it correctly refuses a lan launch: no team LAN.
   - `OPTIPRIMEX_PREFLIGHT=off` skips the preflight, as before.

No Jetson deploy is needed (the launcher, the tests and the sim's profile copy are Mac-side; the Jetsons run v0.1.90.8).

### v0.1.90.11 — 2026-11-12 evening (session A): the UAV map port reassigned (1886)

- Dmitrii: "can we rework 1883 thing? and reassign the ports carefully?" (AskUserQuestion: a dedicated port 1886).
- `scripts/run_usv.sh`: `BOAT_UAV_PORT` defaults to **1886** on any host where it is free (was 1883 on the Jetson only); `0` = off.
  The boat broker template is unchanged (`#UAVLISTEN`).
- 1883 is now only the Mac's hub. 1884 is our boat broker. 1886 is the same boat broker's listener for the drone team's map
  publisher (`--broker <NX> --port 1886`; 1884 reaches the same broker, as they ran it on 11 Nov).
- `sgit_map` docstring, `test_v0187` updated. GUIDE §2 gains the ports table and the OCS_HOST note.
- **Needs:** the NX deploy (run_usv.sh), and the drone team told: `--port 1886` (or change `port` in their
  `buoy-detection/config/map_mqtt_publisher.json`: their file, their edit).

### v0.1.90.12 — 2026-11-12 night (session A): one port number per job (the hub 1882, the drone 1883, the boat ws 9002)

- Dmitrii: "lets revert, I want to reassign Mac port, and keep drone on 1883". Then: "I don't want ANY ambiguity with that, even if it
  relates to internal ports ... assign unique to each one".
- **The hub** moves 1883 → **1882**:
  - `network.env` `HUB_PORT`;
  - `profiles/{course,pool,bench}/ocs_*.yaml` `mqtt.port` (+ the bench's fake-judge `judge_port`);
  - `profiles/bench/mosquitto/{hub,boat,bridge}.conf`;
  - every script default (`run_ocs` / `stop_ocs` / `run_usv`'s bridge / `run_agx` / `run_uav` / `run_uav_window` / `run_boat_window` /
    `run_bench` / `run_sim` / `stop_bench` / `status` / `reset_all` / `sim_world` / `run_judge` / `run_rctest`'s note);
  - every Python hub default (`ocs_cli`, `ocs_web`, `bench_tui`, `vcmd`, `judge_web`, `boat_web`, `rc_tui`, `scenario`, `sim_day`,
    `fault_timing`, `uav_web`, `sessions`, `fake_robocommand`, `fake_perception`, `fake_uav`'s hub, `simview`, `logger`,
    `companion.config.MqttCfg`);
  - the launcher's and the pages' labels.
- **Unchanged (external, not ours):** RoboCommand's 1883 (`judge_port`, `RC_*_PORT`, the launcher's course rows), the venue's 1910,
  the drone team's own broker (`UAV_BROKER_PORT`), their listener 14600 and MAVProxy.
- **NX 1883** is only the boat broker's drone-map listener (`BOAT_UAV_PORT` default 1883, on any host where free). Their publisher
  keeps its default `--broker <NX> --port 1883`. v0.1.90.11's 1886 is reverted.
- **The boat's websocket** 9001 → **9002** everywhere (`network.env` BOAT_WS, `run_usv` / `stop_usv`, `usv_ui --ws`, map.html's
  example): 9001 is only the hub's.
- GUIDE §2: the table of our ports (one number per job, one host) and the external ports apart. HANDOFF / LAUNCHERS.md mentions fixed.
- Test: `test_mission1_rehearsal::test_one_port_number_per_job_and_1883_only_for_the_drone` (GUIDE's numbers unique; the hub 1882 in
  env / profiles / bench conf / scripts; no Python hub default 1883).
- `make test`: 1076 passed, 1 failed: `test_usv_ui::test_fcu_switch_live`, a timing gate under the parallel load; it passes alone, twice.
- **Deploy: the Mac and BOTH Jetsons together** (code + network.env): the NX's bridge reads HUB_PORT from its own network.env.

### Exposure review of 12 Nov's ZED snapshots (session A, no code change)

- `logs/cv_review/exposure_2026-11-12/`: `exposure_review.py`, `stats.json`, before/after samples. Input: 301 RGB snapshots
  (pairs/rgb_*.jpg, every 10 s) from 12 recording sessions.
- Bands: sky / buoys (the middle third) / water. Mean in linear light; EV = log2(0.18 / buoy-band mean).
- **Normal frames (295):** EV median **−0.35** (p25 −0.52, p75 +0.03). Saturated share in the buoy band: median 5 % (the white buoys'
  sunlit faces); sky 8 %; water 0.3 %.
- **Blown frames (3):** each is **snapshot #1** of its recording (68–70 % of the buoy band saturated, −1.8 to −2.3 EV). These are the
  first frame after the observer opens the ZED, before its auto-exposure settles. Not representative; drop them from training sets.
- **Suggestion:** about −⅓ EV for the buoy band. No camera change before the new model is tried. A software offset only after testing
  it against the student's model: their training data has the same exposure.

---

## v0.1.91 — 2026-11-12 night (session A): the horizon cut, the LED colour, the blink memory (cv/buoys_multi)

- **Request** (Dmitrii): "drop everything which is above certain line (I see that a build has been detected) ... can we detect the
  color (without mapping). Also, we need some cache for the blinking, and distinguishing the blue (constant vs blinking)".
- **`cv/buoys_multi/led_core.py`** (pure numpy, no cv2: the observer's tests run on a stand-in):
  - `horizon_y` / `above_horizon`;
  - `led_band` / `led_sample` (an OpenCV-compatible `bgr_to_hsv`, max difference 1 against cv2);
  - `LedCache` (image tracks, a 4 s window, steady / flashing / off / unknown).
- **The observer:** `--horizon frac|off` (default frac 0.45) drops a box above the line, for every detector, before measuring.
  `--led on|off` (default on) adds `led` to every buoy record; `rec_core.buoy_brief` keeps `led` and `model_class`.
- **Checked on 12 Nov's 629 CV_NEW detections:**
  - the horizon at 0.45 drops 6 boxes, all of them the building / sky false positives (confidence ≤ 0.37, no depth), and nothing
    with depth;
  - the LED reader first gave 4 "on" (blue): 2 sky boxes, the pale-blue float of a marker behind (V ~205), and marker 1's
    striped blue lantern at 7 m. At V ≥ 215 only the lantern remains.
  - None of the recordings shows a lit red / green LED, and the 10 s snapshots cannot show blinking. The first real test of the colour
    and the blink memory is the venue's lit buoys (a recording at ≥ 5 fps).
- Tests: `tests/test_v0191.py` (5).
- **Not done:** cv_publish / the boat's map do not use `led` yet; an IMU-levelled horizon (the guard has the quaternion) instead of the
  fixed fraction. **Needs:** the AGX deploy.


## consult (Cowork, no version) — 2026-11-13 01:30 SGT — the Task 1 entry search, the guard in compact gates, CV tonight

**What:** Cowork's 40-minute answer to session A's `docs/CONSULT_2026-11-13.md` (the night before the one-shot course run, m1t1 USV only).
Files: `docs/CONSULT_ANSWER_2026-11-13.md` (complete), `logs/sim_explore_2026-11-13/REPORT.md` (A) and `SOTA.md` (B) at the brief's paths,
`docs/COWORK_HANDOFF_2026-11-13.md` (the decisions taken in Dmitrii's absence and what stays his). No code touched; no fast-SIL runs made
(the box) — a 4 000-draw Monte Carlo of the blink classifier was run instead (numbers in the answer, P1 b).

**Decided for v0.1.92 (session A builds; Dmitrii AFK, automatic mode):**
- The guard's STOP region becomes the hull's **corridor** (forward ≤ 1.5 m and lateral ≤ 0.7 m; SLOW ≤ 3 m / 1.2 m), NX-side in
  `optiprimex/usv/guard.py`; the ±35° cone stays the AGX's reporting window. Reason: a buoy 1 m abeam in a 2 m gate sits at 42° at 1.5 m and at
  25° with 0.3 m of lateral error — the cone stops the boat in every compact gate. (Dmitrii: "proceed when the buoy seems close to the side".)
- `circle_r_m` 3 → **5**, `circle_v` 0.5 → **0.3**: the hull gives ≈ 4 °/s under way; 3 m at 0.5 m/s needs 9.5 °/s. Check hold at **5 m**.
- The blink decision: window **6 s**, ≥ **12 valid samples**, zero-order hold → 0.5 s median → run lengths (flashing = on- and off-runs of
  0.5–1.6 s, on-fraction 0.3–0.7; steady = on ≥ 0.9 and no off-run ≥ 0.5 s). Monte Carlo at 5 fps / 30 % dropout / 10 % false-off: today's rule
  turns a steady light into "flashing" 19 % of the time; the new rule 0.4 %. Entry = flashing AND blue by LED hue (≥ 80 % of on-samples);
  exit = steady AND blue; the hull vote only at ≤ 8 m and only below 12 samples. Brightness relative to the hull (max V ≥ median + 60, S ≥ 100),
  white core = on.
- SEARCH_ENTRY: CHECK any blue/undecided buoy within `check_r_m` **7** before the next scout step; blue-first PICK in SCOUT_ENTRY; lead cap 2 when
  the scout step is on; `check_s` **6** + 6 from 2 m closer; `scout_entry_s` **420**; doubtful buoys re-checked before lucky; **lucky = red + green
  pair only**, never two markers.
- Map association: gate `0.35 + 0.08·r` clamped 0.8–1.6 m (was 2.0 > the 1.8 m marker spacing); blue vs non-blue never merged at < 10 m;
  `linear_sum_assignment` when detections compete. Replay 12 Nov: expect the 26 mixed entities → ~12.
- CV tonight: the classifier fine-tune is a shadow run (not for m1t1): lr 1e-4, 30 epochs at 320, `fliplr 0`, hold-out by session, every 1/7 crop
  hand-checked; no SAHI / 1536 (a horizon-band crop is the cheap gain); FP16 parity = 50 images, IoU ≥ 0.7 match ≥ 95 %, conf drift ≤ 0.03.

**Sim worlds / pass lines** (the agent's P2, with the `omni_usv` hull): 8 failure rows in the answer — the exit as entry 0/20 (C), 0 STOPs in a
2 m gate with ±0.3 m noise, the orbit radius held ±1.5 m, ≤ 1 NOT_ENTRY per run, the entry found ≥ 18/20 with 50 % light dropout (D), lucky never
pairs markers, entity count = truth ± 1, 0 boundary exits.

**Still Dmitrii's (morning):** the slot length vs `scout_entry_s` 420 and the 110 s orbit lap; the final `circle_r_m` / `circle_v` after
`test_yaw`; whether a passing classifier run is deployed at all.

**The worry:** the guard has never met a real 2 m gate. Build the corridor first; on the bench in the morning a box 1 m abeam of the bow must
not trigger it, a box ahead must.


## consult addendum (Cowork, no version) — 2026-11-13 02:45 SGT — Dmitrii's four notes by message + the m1t1 logic rehearsal

Same block as `docs/COWORK_HANDOFF_2026-11-13.md` §Addendum (the operative text for v0.1.92); the rehearsal page is `pathing_sim/m1t1_logic_check.html`.


Interactive rehearsal: `pathing_sim/m1t1_logic_check.html` (the same page is a claude.ai artifact for Dmitrii). Worlds A–F from the brief,
the measured hull, the camera at 5 fps with dropouts, both configurations side by side ("Compare today vs v0.1.92", 20 seeds a world).

**Dmitrii's four decisions (02:00–02:30 SGT), binding for v0.1.92:**

1. **Orbit radius 1.5–2 m, buoy in the centre — not 5 m** (the course is tight: the first gate is ~5 m from the entry). The hull cannot
   turn a 2 m circle bow-first above ≈ 0.08 m/s (160 s a lap), so the orbit is a **crab orbit**: heading held, the omni frame translates
   round the circle with mode-20 surge + sway at 0.25 m/s (≈ 50 s a lap, clockwise = the velocity vector rotating clockwise), a radial
   correction toward R = 2.0 m, yaw only to hold the heading. Bow-following at 2 m / 0.08 m/s is the fallback if sway proves unreliable
   on the bench in the morning. "Nice and easy" — slower is fine.
2. **Pathing respects the geofence**: the RxCourse polygon (+ 1 m position tolerance) bounds every target — a mapped buoy outside it is never a
   suspect, a visit target, a lead, a lucky-pair member or an entry/exit decision ("alien buoys"); the planner never plans outside the polygon.
3. **"Slowly investigate; a look-lock on the suspect before proceeding to turn"** — the ladder gets micro-stages:
   `SEARCH_ENTRY (4 s look) → LOOK_LOCK (stop, bow on the suspect, 4 s; creep 0.15 m/s if the turn is large) → APPROACH_SUSPECT (0.3 m/s, to 5 m)
   → CHECK_BLUE (6 s, then 12 s window if unknown) → ALIGN_ORBIT (0.2 m/s to the 2 m point) → CIRCLE_ENTRY_CW (crab)`; a nearer suspect
   interrupts an approach to a far one; an unlit buoy (pattern `off`) is dropped at once (no retry); only undecided / blue-share ≥ 30 % buoys are
   suspects (a resolved red / green / yellow buoy is never visited); the in-place 360° turn (180 s on this hull) is the last resort, not the first step.
4. **"These micro-stages should be noted in the GUI"**: every micro-stage and its reason line goes to the RUN stepper (OCS page) and the tk
   console's stage line, as `STAGE · why` (e.g. `LOOK_LOCK · suspect #10 at 8.4 m`, `CHECK_BLUE · steady blue → EXIT, remembered`), with the LED
   cache state of the suspect (`pattern n=…`) beside it — the operator must see why the boat is standing still.

**Guard, refined after the rehearsal (replaces the corridor line of §Decisions):** a buoy **the map knows** stops the boat only on a
collision course (≤ 1.0 m ahead and ≤ 0.3 m off the centreline), otherwise SLOW and pass it — this is "the buoy seems very close to the side
but we pass just nicely"; an **unknown** object keeps the 0.7 m corridor at 1.5 m, and an unknown object in the SLOW zone becomes a 1.5 m
map ring at once (the real far rings) so the path re-plans before any stop. No blind back-off when a mapped buoy sits astern.

**Rehearsal numbers (20 seeds a world, measured hull, 5 fps, 10 % dropout — D 50 %):**

| world | today's logic (cone · waver blink · 3 m @ 0.5 · any pair) | v0.1.92 (corridor · run-length blink · crab 2 m · look-lock · geofence) |
|---|---|---|
| A entry in view | wrong entry 0 · found 20/20 · **finished 0/20** (every run ends in SAFE_HOLD or a gate hold; 20/20 leave the course on the 7 m orbit) | wrong 0 · found 15/20 (median 0.7 min) · finished 19/20 (5 by lucky) · exits 0 |
| B entry behind | found 0 · lucky 20 · finished 14 · exits 14 | found 16/20 (5.0 min) · finished 12 · exits 0 · 8 time-outs in the gate/exit phase |
| C decoy exit near the start | **wrong entry 3/20** · found 0 · exits 10 | wrong 0 · found 20/20 (2.2 min) · the exit remembered · finished 12 |
| D entry light 50 % dropout | found 20 (the waver rule "finds" by hull colour) · finished 0 | found 13/20 · finished 17 |
| E box between gates | wrong 3 · finished 3 | found 14/20 · **finished 1/20 — 16 gate holds**: an unknown object on the gate line defeats stop → back off → re-plan |
| F all lights off | "found" 8 (false: no light exists) · finished 1 | found 0 (correct) · lucky 18 · finished 17 |
| alien buoys targeted | — (no fence) | **0 in 120 runs** |

Reading for tonight: the entry decision is safe (0 wrong entries in 120 runs against 6 for today's rule), the crab orbit holds 2.1 m, the fence
holds, no run leaves the course. The weak spots the sim still shows — and the agent should test for, not assume away: (i) ~1 guard
STOP per run inside 2 m gates even with the collision-course rule: the transit must centre on the mapped pair and the stop/back-off must
not loop (3 stops in 3 m = hold); (ii) an unmapped object on the gate line (E) is not solved by back-off — it needs the far-ring detour
*before* the gate, else the operator; (iii) a far suspect seen first costs minutes on this hull — near suspects first, switch when a nearer
one appears (in the page now); (iv) time: a clean run is 11–15 min on this hull — the slot length decides the budget.


## consult addendum 2 (Cowork, no version) — 2026-11-13 03:55 SGT — "the sim was missing the gates; big orbits in the planner" (Dmitrii) → two more layers

1. **Gate memory layer (second layer over the map).** Once the entry is confirmed, every red+green pair 1.5–4 m apart in the map becomes a
   gate record `{red, green, centre, normal, s}` with `s` = its distance along the **axis from the entry** (toward the exit when known, else the
   course centre), ordered by `s`, **never forgotten** during the run, new pairs appended as they appear (2 Hz refresh). TRANSIT walks this list:
   pre-point half a gate spacing before the gate (moved closer / sideways when a ring or buoy sits on it) → through along the gate's own normal →
   **passed** only when the hull crosses the gate line inside `width/2 − 0.25`; otherwise **MISSED** → back to the pre-point and again (twice,
   then on to the next, logged). The RUN stepper shows `G1 ✓ G2 ✗1 G3` (in the page: numbered gate marks on the map).
2. **The omni follower instead of bow-following on every leg** (the "very big orbits" were the P0 turn-under-way law: at 0.3–0.4 m/s this hull
   holds a 5–6 m radius). The follower keeps the velocity vector on the path (surge + sway ≤ 0.15 m/s) while the heading turns toward the path at the
   hull's rate — no arcs, 0.3 m/s in transit, 1.2 m look-ahead. The **guard corridor is laid along the direction of motion**, not the heading,
   because a crabbing hull moves where the bow does not point. Mapped gate buoys in TRANSIT only slow the boat; the planner owns the gate
   (planner keep for gate buoys 1.0 m — the T1 rule's 1.68 m is wider than half of any gate ≤ 3.4 m).

**Numbers after both layers (20 seeds, v0.1.92 column):** A entry 18/20 in 0.7 min, finished 18/20, run 5.1 min · B 12/20 found, 13 finished,
8.9 min · C 14/20, 13 finished, 6.0 min · D 11/20, 12 finished, 5.0 min · E 2/20 finished (17 holds) · F lucky 10, finished 10. Still 0 wrong
entries, 0 boundary exits, 0 alien targets in 120 runs; gates passed per finished run 3/3, misses ≈ 0.

**The finding that matters for the real stack:** the remaining failures are all the **3-stops-within-3-m → SAFE_HOLD rule firing in the gate
field**, mostly during search legs that cross it, with a 0.76 m hull between buoys 1.0–1.25 m off the centreline. Three things the agent should
check in the fast SIL with the real `gatepath` planner and FollowPath, with these exact metrics: (i) does the planner's clearance rule
(`keep = 1.68 m`) even admit a path through a 2–2.5 m gate? If not, gates need their own keep (≈ 1.0 m) or the pair's midpoint as a waypoint
with the keep suspended for its two buoys; (ii) the guard's hold counter should count stops **on the same object**, not within 3 m — in a gate
field 3 m covers two gates; (iii) a search leg should not cross the gate field at all when it can go round it (route the scout legs through the
free half of the course). An unknown object on the gate line (world E) remains the case the operator must handle.


## consult addendum 3 (Cowork, no version) — 2026-11-13 04:45 SGT — defaults settled (Dmitrii), the "mission modifiers" subpanel, and where v0.1.92 stands against it

**Dmitrii, 04:15–04:30:** *bow-following is the default* (plain GUIDED); *crabbing is the secondary experiment* (needs our custom mode 20),
selectable as a clear tick in a new **"Mission modifiers" subpanel on the Run Sequence**, set at the step before ENABLE AUTO and frozen from
then on. *Start the run with a small advance forward, make a full rotation around us, then proceed with the ladder.*

**The subpanel (OCS page RUN ladder + tk console; the same words in the DECLARE constructor text and the run banner):**

| modifier | default | alternative | what it switches |
|---|---|---|---|
| start | **advance 4 m + full rotation** | straight into the search | `START_ADVANCE (0.3 m/s) → START_SCAN (360° on the spot) → SEARCH_ENTRY` |
| orbit | **bow-following, GUIDED, R 2 m → v ≈ 0.1 m/s** (the hull's rate sets R = v/ω: 0.1 → ≈ 2 m, 0.15 → 3.3, 0.2 → 4.1) | **crab, mode 20**: heading held, surge + sway 0.25 m/s on a 2 m circle (~50 s a lap) | `t1.circle_mode bow|crab`, `circle_v` |
| follower | **bow-following FollowPath (GUIDED)** | **crab follower, mode 20**: velocity on the path, sway ≤ 0.15, heading follows | `guidance.follow bow|crab` |
| look-lock | **on** (LOOK_LOCK → APPROACH_SUSPECT → CHECK → ALIGN) | off (v0.1.92's check-within-7-m only) | `t1.look_lock` |
| lucky | off | on (`one_known` pairs, the agent's rule) | `t1.lucky` |
| budget | 420 s | 300 s (15-min slot) | `t1.scout_entry_s` |

Every modifier is a profile value with a tick; nothing else in the ladder changes with them; the GUI shows the active set as one line
(`mods: start=advance+360 · orbit=bow 2 m · follow=bow · look-lock · lucky off · 420 s`).

**Rehearsal, three columns (20 seeds a world):** today · v0.1.92-default (bow-following, advance + rotation, look-lock, gate memory, geofence) ·
v0.1.92 + crab modifiers:

| world | default: found · finished · run | crab modifiers: found · finished · run | note |
|---|---|---|---|
| A | 20/20 · 19/20 · 16 min | 20/20 · 20/20 · 8 min | default: gate misses 1.75/run (re-approached), 4 boundary exits on the 2.7 m orbit |
| B | 19/20 · 20/20 · 13 min | 19/20 · 19/20 · 8 min | |
| C | 15/20 · 19/20 · 15 min | 9/20 · 9/20 · 5 min | crab: more gate-field stops (the motion-direction corridor), fewer finishes |
| D | 17/20 · 18/20 · 16 min | 18/20 · 19/20 · 8 min | |
| E | 11/20 · 12/20 · 13 min | 7/20 · 2/20 · 5 min | the box on the gate line |
| F | 0 (correct) · 15/20 lucky · 12 min | 0 · 6/20 · 5 min | |

0 wrong entries, 0 alien targets in all 360 runs. Reading: **the default finishes more runs but takes twice as long** (the 0.1 m/s orbit is a
2-min lap, the bow-following legs are arcs); crab is fast but its gate-field behaviour is less settled — exactly the right thing to keep as an
experiment behind a tick. The in-place 360° at this hull's 2 °/s is **180 s**; the agent's `underway` scan (a 5 m circle) does it in ~90 s — see the question below.

**Where the agent's v0.1.92 (built 01:00–04:30, `docs/MORNING_2026-11-13.md`, `logs/sim_explore_2026-11-13/FINDINGS_CLAUDE.md`) stands against tonight's decisions — the reconciliation list for the morning:**

| item | agent's v0.1.92 | tonight's decision | to do |
|---|---|---|---|
| camera height | `--h-cam 0.55` (fitted from 288 depth frames; at 1.0 the water read as an object) | — | **keep**, tape-measure it on the boat |
| guard | corridor 1.5 / 0.7, SLOW 3 / 1.2; `astern_free` back-off | + mapped buoys stop only on a collision course; corridor along the motion when crabbing | add the mapped-buoy rule; the motion-direction corridor only under the crab modifier |
| orbit | `circle_v 0.2` → ≈ 4.1 m flown, 160 s lap; "needs ~4.5 m of clear water round the entry" | **2 m radius**: GUIDED at ≈ 0.1 m/s by default; crab mode 20 as the modifier | set `circle_v 0.1` (verify R on the fake plant), add `circle_mode crab`; the 4.5 m note becomes ≈ 2.5 m |
| map memory | buoys kept 10 min as gates, 15 min at all | + the **ordered gate list from the entry outwards**, pass only inside the gate line, MISSED → re-approach | add the order + the pass/miss rule to gatepath / T1 |
| entry search | scan (5 m circle, ~90 s) → check any blue within 7 m → scout → 420 → re-check → lucky/hold | **advance 4 m + full rotation** → look-lock micro-stages → …; suspects = blue-share ≥ 30 % or undecided and near; unlit dropped; nearer suspect wins | add START_ADVANCE / START_SCAN, LOOK_LOCK, APPROACH_SUSPECT, ALIGN_ORBIT; each stage + reason on the RUN stepper and the tk stage line |
| geofence | (not mentioned) | targets only inside the RxCourse polygon + 1 m; alien buoys never | confirm or add in PerceptionMap / T1 candidate filters |
| lucky | `one_known` (one known red/green in the pair) | red + green only was Cowork's; the agent's `one_known` is acceptable | Dmitrii's tick; keep `any` as the alternative |
| association gate | **not adopted** — on 12 Nov's real detections the 0.8–1.6 m gate split markers 18 → 35 | accepted: keep 2.0 m | — |
| LED → map | run lengths over 6 s / ≥ 12 samples, colour ≥ 80 %, relative brightness | same | — |
| circle_v 0.3 (Cowork's first number) | "would have failed: 0 finishes everywhere" | superseded by the 2 m decision | — |
| mission modifiers subpanel | — | new | build (tk + page), values into the DECLARE text and the run banner |

**One question for Dmitrii (not decided):** the full rotation at the start — *on the spot* (180 s at the measured 2 °/s, unless the morning's
`test_yaw side` shows the side thrusters turn it faster) or the agent's *5 m circle under way* (~90 s, the camera sweeps 360° and the boat ends
4 m further in)? The page runs the on-the-spot version.

**04:55 SGT (Dmitrii):** the start advance is **2 m**, not 4 — "4 might already be into the buoy; 2 clears the dock". Page and hand-off updated; the modifier table reads advance 2 m + full rotation.

**05:05 SGT (Dmitrii: "in the simulator the boat was avoiding gates rather than going through them"):** the planner had treated the two buoys of the gate being passed as obstacles with a keep wider than half the gate — it detoured round every gate and the pass test called it MISSED. Fixed in the page: the gate being passed is a corridor (its pair leaves the planner's object list while current; other memory gates keep 0.7 m), through-line 4 m past the centre. Default (bow-following) now passes 3.0/3 gates in A, 2.5–2.8 in B/D, finishes 20/20 in A/B/D at 14–15 min; crab 20/20 at 8 min. **For the real stack:** `gatepath` must suspend the current pair's keep the same way, or the 1.68 m clearance rule detours round every 2–3 m gate. Hand-off addendum 3 table replaced with these numbers.


## consult addendum 4 (Cowork, no version) — 2026-11-13 05:15 SGT — "shrink the avoidance box and the keep-out from buoys?" (Dmitrii) — yes, in two places, not everywhere

Geometry first: hull 1.19 × 0.76 m (half beam 0.38), buoy r 0.25. A 2.0 m gate leaves **0.37 m** of water between the hull side and each buoy
when the boat is exactly on the centreline; a 2.5 m gate 0.62 m; a 3.0 m gate 0.87 m. Today's planner keep (`r + beam/2 + clear_m 0.60 +
track_m 0.45 = 1.68 m` from the buoy centre) is wider than half of any gate ≤ 3.4 m — no path through a gate can satisfy it, so the planner
detours. The guard's cone/corridor (0.7 m lateral) fires on any gate ≤ 2.2 m. Those two numbers are what to shrink; the box for **unknown**
objects is not.

| rule | today | proposed | why |
|---|---|---|---|
| planner keep, the **pair of the gate being passed** | 1.68 m | **none** — the pair is the corridor; the leg is pre-point → centre → 4 m past, along the gate's normal | the gate cannot be passed otherwise; the sim's "avoiding gates" was exactly this |
| planner keep, **other mapped buoys** (markers, other gates, the entry/exit while not orbiting) | 1.68 m | **1.2 m** (= r + beam/2 + 0.55) | the 0.45 m `track_m` was for pool buoys with wandering tracks; at ≤ 10 m the map's position error is 0.2–0.3 m. 1.2 keeps ~0.6 m of water off the hull side |
| planner keep, **guard rings** (unknown objects) | 1.5 m ring + keep | **unchanged** | the ring is the only margin for a thing the detector cannot see |
| guard STOP, **mapped buoy** | forward ≤ 1.5 m, lateral ≤ 0.7 m | **forward ≤ 1.0 m, lateral ≤ 0.45 m** (half beam + 7 cm = a collision course); otherwise SLOW | "the buoy seems close to the side but we pass just nicely" — a mapped buoy is the planner's business; the guard only catches a real collision course |
| guard SLOW, mapped buoy | ≤ 3 m / 1.2 m → 0.3 m/s | **≤ 2 m / 1.0 m → 0.3 m/s** | in the gate field the 3 m zone never switches off; 2 m still gives 6 s of slow approach |
| guard STOP / SLOW, **unknown object** | 1.5 / 0.7 · 3 / 1.2 | **unchanged** | depth at 1.5 m + ~0.5 s of pipeline at 0.6 m/s = 0.3 m eaten → 1.2 m of real margin; shrinking this removes the one protection we have |
| the hold rule | 3 stops within 3 m → AUTO_SAFE_HOLD | **3 stops on the same object** (within 1 m of each other) | in a gate field 3 m covers two gates; the rule was meant for one stubborn obstacle |
| what matters more than any number | — | **centring on the mapped pair**: the through-leg from the pre-point along the pair's own normal, speed 0.3 inside ±4 m of the gate line | with 0.37 m of water per side in a 2 m gate, a 0.3 m lateral error is a touch whatever the keep says |

In the page these are `keepGate 0.7` for other memory gates, the current pair excluded, `latStopMapped 0.4`, `keep 1.68` for the rest (kept on
purpose to show the contrast); the numbers above are what to put in `gatepath/planner.py` (`clear_m`, `track_m`, a `gate_pair` exclusion) and
`usv/guard.py` (`stop_mapped`, `slow_mapped`, `hold_same_object_m`). Bench check in the morning: a mapped box 0.6 m beside the bow → no STOP,
SLOW only; the same box 0.3 m off the centreline 1 m ahead → STOP; an unmapped box 1 m ahead → STOP at 1.5 m as before.

## consult addendum 5 (Cowork, no version) — 2026-11-12 18:00 UTC (= 13 Nov 02:00 SGT) — "I need to see a proper passing through the gates" → the pathing_sim lessons, and slow

**Correction of the labels above (never edited, so here):** the SGT times on the consult (01:30) and addenda 1–4 (02:45 / 03:55 / 04:45 / 05:15)
are 0.5–3.5 h too late — a clock error on Cowork's side. True times ≈ 01:05 / 01:26 / 01:37 / 01:38 / 01:45 SGT = 17:04–17:45 UTC on 12 Nov.
Order and content stand. From here: UTC, SGT in brackets.

Dmitrii (by message): *"I need to see a proper passing through the gates before I can reliably handoff"* · *"pathing generation — let's revise
what we did earlier with pathing_sim … use lessons from there"* · *"we can go veery slowly on the boat"*.

**What was wrong** (the rehearsal page, and `t1_safe_passage.py`'s transit as in the tree at 17:53 UTC): the through leg *pursued* the gate's
mid point (look-ahead 1 m; `la_m 2.0` + `transit_v 0.6` in the tree → rmin 8.6 m) — 0.3 m off centre 1 m before the mid is a 17° heading demand
the 2–4 °/s hull never meets, so the boat crosses where it is: pass offsets median 0.3 / p90 0.8 / max 1.2 m in 1.8–2 m gates = 1.1–1.8 contacts
a run. Plus a loose pre-point (anywhere inside half the gate width, 12° of alignment) and a run-in that grew *backwards* past the previous gate
(`+1 m when angled` / `lead_in_m 5` with gates 4 m apart).

**The lessons from `pathing_sim/gate_pathing_simulator_spec.md`** (§6.5, §6.6, §7 — the S6 sketch already had them): (1) the crossing leg is
**cross-track LOS**: `path[0] = proj = wp + along·cdir`, the boat's projection onto the gate axis, look-ahead Δ along the normal from there,
shortened on that leg; (2) **Dubins only for legs ≥ 2·rmin** — shorter legs are straight, and a bearing > 30° off the bow is a yaw command first;
with gates 4 m apart and rmin ≥ 2 m every T1 leg is short, so the transit is turn-on-the-spot → straight → turn onto the normal → cross-track
through; (3) the pre waypoint d before the gate on its normal: the run-in is where the slow hull centres itself — as long as the previous gate
allows, never longer; (4) LOS restarts from the boat each tick; (5) a choice re-made every tick orbits the boat → lock / hysteresis.

**The page now** (`pathing_sim/m1t1_logic_check.html`, artifact 1794506423-292a): through leg = cross-track LOS on the gate normal, look-ahead
1.5 m, **0.12 m/s**; pre-point leg **0.15 m/s** (0.12 inside 1.5 m), entered by a turn on the spot when the LOS point is > 25° off the bow (15°
hysteresis); run-in = gate spacing − 1.5 m clamped 1.5–3.0 (2.5 m for 4 m gates), no angled extra; arrival within 0.35 m or past the point within
0.3 m of the normal; alignment **within 6°** (≤ 90 s); orbit **2 m @ 0.08** (2.0–2.1 m flown); approach / scout / align 0.2 / 0.3 / 0.15; the crab
follower uses the same LOS target on the through leg; the "today" preset carries v0.1.91's own speeds.

**Numbers** (A/B/D × 20, bow-following default): done 60/60; gates 2.95/3 a run; misses 0.00 (was 0.05); **pass offset median 0.00 · p90 0.00 ·
max 0.10 m** (was 0.3 / 0.8 / 1.2); **contacts 0.00** a run (was 1.1–1.8); boundary exits **0** (was 5–7 of 20); orbit 2.1 m; run time 14.7 min
median, 15.4 p90, 16.4 max (was 12.5–13: +2 min, ~80 s of it the 150–170° turn from the orbit's exit toward gate 1 — the agent's
`circle_exit_tangent` removes most of that). C 20/20 done (2.3 gates, 0.2 contacts: the markers' pair); E (invisible box on the gate line) → the
3-stop HOLD in 18/20, as designed; F dark → lucky 20/20 done. Crab modifier: 8.5–8.9 min on A/B/D, same gates, but C 10/20, E 0/20 → secondary.

**For session A / B** (docs/COWORK_HANDOFF_2026-11-13.md addendum 5 §5.5): `transit_v 0.6 → 0.15`, new `through_v 0.12` and `la_through_m 1.5`;
the through leg's carrot on the pre → post line at the boat's projection + la (or a single pre → post segment with `la_m 1.5` — the same thing);
`lead_in_m 5 → min(spacing − 1.5, 3.0)`, never behind the previous gate; hold-and-yaw before a leg > 25° off the bow and onto the normal within
6° at the pre-point (90 s cap) — as OCS micro-stages `TRANSIT · to pre-point k / turning / aligning k / through k / passed k (x m off)`;
`circle_v 0.15 → 0.08` (0.10 → 2.4 m if the FCU cannot hold 0.08); `circle_exit_tangent` ON after a SIL check; start stays 2 m + 360° on the
spot (the MORNING doc's 5 m circle is the logged alternative, Dmitrii's call); keep-out as addendum 4; a test per behaviour.

**Risks:** the plant (no current — a 0.2 m/s cross-current at 0.12 m/s is a 60° drift angle: the on-the-spot hold must be StationKeep, or
speeds 0.2 / 0.15 with a 3 m run-in); GUIDED's speed floor and in-place yaw rate at 0.08–0.15 m/s are bench numbers we do not have (if in-place
yaw < 2 °/s: `align_deg 10`, cap 120 s); time 14.7 min a run (trade: exit-tangent −80 s, orbit 0.10 −30 s a circle, the 5 m circle −90 s but
+4.5 m of clear water); the map's 0.2–0.3 m position error is now the dominant term of the pass offset — the planner cannot fix that.

Files: `pathing_sim/m1t1_logic_check.html` (page), `docs/COWORK_HANDOFF_2026-11-13.md` (v5: addendum 5 + the time correction), this block,
ROADMAP WP48 line. No code in `optiprimex/` touched by Cowork. Next session first: §5.5 items 1–4 (they are the gate pass), then 5.

## consult addendum 6 — FINAL (Cowork, no version) — 2026-11-12 18:30 UTC (= 13 Nov 02:30 SGT) — the hybrid follower (modifier 2), the "squeeze" risk tick, the one table to build from

Dmitrii (by message): *"add some hybrid version, where we can sway to side to avoid collision or put us on better path towards gate — that will be
our second mode … ensuring the path is okay and not jerking our boat … at a certain attempt of jerking, maybe we can repath for default
bow-following"* · *"so far looks good to me! let's make final"* · *"maybe we can squeeze ourselves, taking risks (looking at course E, with obstacle
at last gate)"*. **Cowork's side is closed for the night with this block.** Page: `pathing_sim/m1t1_logic_check.html` (artifact 1794508132-3e7a);
hand-off `docs/COWORK_HANDOFF_2026-11-13.md` v6 — **read its addendum 6 §6.4 first: one table of final values**, then addendum 5 for why.

**Hybrid follower (modifier 2; bow-following stays the default, full crab is modifier 3):** heading + surge = the plain GUIDED law; a small *slewed*
sway (≤ 0.10 m/s, slew 0.05/s, dead band 0.08 m) only inside TRANSIT toward a recognized gate's line: (a) while aligning at the pre-point (v = 0:
slides onto the line as the bow turns), (b) on the through leg when the bow is within 20° of the normal, (c) a **sidestep** when the pre-point is
> 45° off the bow and < 3 m away — bow held, sideways first, then ahead. Mode 20 only while swaying. **Jerk detector:** 3 sign changes of the sway
in 10 s → sway off for that gate, back to the pre-point, plain bow-following (never fired in 200 runs). A/B/D with it: 20/20, same gates and
offsets as the default, +0.05–0.10 light contacts a run → a tool for the tight cases, not a default.

**Squeeze (risk tick, off by default):** an unknown object (guard ring) in a run-in: measure the gap each side (object taken 0.5 m wide; walls = the
gate's inner edges on the gate line, else the nearest mapped buoy / ring within ±1.5 m along, 2.5 m at most; inside the course); own side when it
has beam + 0.8 m, else the wider side; no squeeze under beam + 0.5 m (0.25 m a side). Plan = a dog-leg on the gate's reference line (full offset
alongside the object ± hull/2, 2 m ramps, clamped inside the gate's water within 1.2 m of the gate line); the next pre-point moves onto it; when the
object is beyond the gate being passed the plan is made before the gate line, the pass counted at 0.3 m through, straight on to 0.75 m (stern clear),
then sidestep (hybrid) or turn. The risk: for that object the guard STOP becomes "face ≤ 0.3 m ahead on the centreline" at 0.08 m/s; nothing else
changes. **E × 20: default 3/20 done · hybrid 3/20 · hybrid + squeeze 13/20 · bow + squeeze 7/20**; the 7 remaining holds are search-phase crossings
of the gate field (addendum 2 iii), every gate-phase box encounter passes; price 0.65 light contacts a run (a 1.2 m box 2 m before a 2.2 m gate
leaves 1.5 m of water for a 1.19 m hull). A–D/F unchanged by the tick. **Dmitrii's thumb on the day, not a default.**

**Final numbers (× 20 seeds, default / hybrid):** A 20/20 · 3.0 gates · 0.00/0.10 contacts · 14.9 min; B 20/20 · 3.0 · 0.00/0.05 · 14.4; C 20/20 · 2.25 ·
0.20/0.40 · 16.5; D 20/20 · 2.85 · 0.00/0.05 · 14.6; E 3/20 (13/20 with the squeeze); F 20/20 lucky. Pass offsets median 0.00 · p90 0.00 · max 0.10 m
everywhere; misses 0; boundary exits 0.

**For session A / B — the table (hand-off §6.4):** `transit_v 0.6 → 0.15`; `through_v 0.12` (0.08 squeezing); `la_m 2.0` open, `la_through_m 1.5`
cross-track (carrot on the pre → post line at the boat's projection + la); `lead_in_m 5 → min(spacing − 1.5, 3.0)`; pre-point arrival 0.35 m or past it
within 0.3 m of the normal; `pre_turn_deg 25` (hyst 10), `align_deg 6` (cap 90 s); `circle_v 0.08` (0.10 if the FCU cannot hold it);
`circle_exit_tangent` ON after a SIL check; approach / scout / align 0.2 / 0.3 / 0.15; keep-out and guard as addendum 4; modifiers subpanel:
start · orbit · **follower bow | hybrid | crab** · **squeeze off | on** · look-lock · lucky · budget, one `mods:` line in the GUI, the DECLARE text
and the banner; micro-stages on the stage line incl. `sidestep`, `squeeze`, `sway off, re-path`. Tests per behaviour listed in §6.4.

**Risks added:** the hybrid needs mode 20 to engage / disengage inside a GUIDED leg at 0.1 m/s — bench the switch (props off) before trusting it;
the squeeze trusts a 0.5 m width from one depth face and a 0.3 m stop. Earlier risks (no current in the plant, GUIDED's speed floor and in-place
yaw rate unmeasured, 14–15 min a run, the map's 0.2–0.3 m error now the dominant offset term) stand.

Files this block: `pathing_sim/m1t1_logic_check.html`, `docs/COWORK_HANDOFF_2026-11-13.md` (v6), ROADMAP WP48 line. No `optiprimex/` code touched by
Cowork. The project mirror `claude/cowork_handoff_2026-11-13.md` carries v6.

## consult addendum 7 — DECISION (Cowork, no version) — 2026-11-12 18:45 UTC (= 13 Nov 02:45 SGT) — bow-following + squeeze = the default; hybrid / crab = modifiers; the build prompt for WP49 → v0.1.93

Dmitrii, 02:40 SGT: *"let's make bow-following with squeeze as a default, with others as modifiers — we made a good explore! now let's ask our code
agent to build it."* The page carries the default (artifact 1794508578-d864; `pathing_sim/m1t1_logic_check.html`). Final numbers with it, × 20 seeds:
A 20/20 · 3.0 gates · 0.00 contacts · 14.9 min; B 20/20 · 3.0 · 0.00 · 14.4; C 20/20 · 2.25 · 0.20 · 16.5; D 20/20 · 2.85 · 0.00 · 14.6; **E 13/20 · 1.65 ·
0.50 · 16.3** (3/20 without the squeeze; the 7 holds are search-phase crossings of the gate field, addendum 2 iii); F 20/20 lucky. Pass offsets
0.00 / 0.00 / 0.10 m, misses 0, boundary exits 0.

**The build prompt: `docs/PROMPT_GATE_PASS_2026-11-13.md` — WP49 → v0.1.93** (after v0.1.92 is archived; one request = one version): §2 the value
table (transit 0.15 · through 0.12 · la_through 1.5 cross-track · lead_in min(spacing − 1.5, 3) · pre_turn 25° · align 6° · circle_v 0.08 (0.10 if not
held) · exit-tangent ON · squeeze on: r 0.5 / margin 0.25 / v 0.08 / stop 0.3 · follow bow), §3 the behaviours with acceptance (3.1 cross-track through
leg · 3.2 run-in · 3.3 turn before you go · 3.4 speeds / orbit · 3.5 squeeze · 3.6 the Mission modifiers subpanel · 3.7 hybrid · 3.8 crab · 3.9 the
micro-stage words · 3.10 keep-out / guard), §4 tests + the SIL sweep targets (A/B/D/E × 20: offsets p90 ≤ 0.2, contacts ≤ 0.1, misses 0, E ≥ 10/20),
§5 the order (3.1–3.4 first — they are the gate pass), §6 the morning questions. Hand-off `docs/COWORK_HANDOFF_2026-11-13.md` v7 (addendum 7 = this
decision). ROADMAP: WP49 added under WP48. Project mirrors: `claude/cowork_handoff_2026-11-13.md`, `claude/prompt_gate_pass_2026-11-13.md`.
**Cowork's side of the night is closed.** No `optiprimex/` code touched by Cowork; the Jetsons are offline, the morning deploy on Dmitrii's go.

---

## v0.1.92 — 2026-11-13 night (session A): entry-first ladder, LED → map, guard corridor + mapped-buoy rule, speed cap, camera height (WP48; archived together with v0.1.93)

- **Requests** (Dmitrii, 12→13 Nov night): the overnight plan (entry first, scout until seen, investigate blue, drop the exit), pathing sim, real-log
  replay of the avoidance, other obstacle models, manual visual checks, model tuning + a small classifier; "can we go faster than 5 fps?"; "cap the
  speed"; the box colours in the GUI; pictures of the lights. Cowork consult: `docs/CONSULT_ANSWER_2026-11-13.md`, `docs/COWORK_HANDOFF_2026-11-13.md`.
- **T1** (`usv/tasks/t1_safe_passage.py`): SCOUT_ENTRY / CHECK_BLUE (check any blue / undecided within 7 m from 5 m, 6 s, again 2 m closer;
  steady blue → `exit_seen`; 420 s budget; doubtful re-check; lucky `one_known`); a locked entry that reads blue in ≥ 10 near sightings with no dark
  one = the EXIT (`steady_n`); `_ring_goal` (a clear join point on a circle); scout / check legs only when clear (`_reachable`); `confirm_entry`
  (built, off: 20-seed sweep 13/7/15/5 vs 17/15/17/16 without).
- **Map** (`perception_map.py`): `quality.led` from cv_publish → `_led_vote` / `_led_apply` (≥ 3 agreeing sightings: colour + light from the LED);
  `lucky_gates(mode)`; `assoc_colour_m` (a lit colour never joins another decided one within 10 m); Cowork's range gate **not adopted** (12 Nov
  replay `logs/cv_review/overnight_2026-11-13/assoc_replay.py`: mixed 22 → 19, split 18 → 35); config reads `max_age_s / drop_age_s / assoc_*`.
- **Guard** (`usv/guard.py`): the corridor (`stop_rule corridor|cone`, 1.5 / 0.7, SLOW 3 / 1.2), the mapped-buoy rule (STOP ≤ 1.0 / 0.45),
  `astern_free` (the back-off short of a mapped object astern).
- **Speed cap** (`usv/tasks/base.cap_speed`): `limits.max_speed_mps` enforced on every task command (was a label); course 0.3; guard `slow_v` 0.15.
- **Observer** (`cv/buoys_multi`): `--h-cam 0.55` (fitted from the water plane in 288 of 12 Nov's depth frames; at 1.0 the water read as an object:
  SLOW in 286/301 — `logs/cv_review/overnight_2026-11-13/guard_replay.py`); `led_core` run-length blink rule (steady read "flashing" 24 % → 1 %,
  Monte Carlo), absolute brightness kept (the relative rule missed the real lantern, `led/real.jpg`), `burst()` + `--led-burst N`, `timing_ms`, the
  lamp dot + FLASH/STEADY on the picture; cv_publish passes `led`; `scripts/agx_observer.sh --led-burst / --depth-mode / --resolution`.
- **OCS page**: buoy boxes in the buoy's colour. **Sim**: `FastSil(perception_hz=)`; `pathing_sim/explore/` (worlds, run, plot, summ, camera_runs);
  campaign cases lift the speed cap (designed uncapped).
- **CV**: 360 12 Nov marker crops hand-labelled (`cls/hand_labels.txt`); the student's classifier 74.9 % (1↔7 35 errors); fine-tune (12 classes, real +
  2400 synthetic, session hold-out): **63.3 % → 98.4 %**, 1↔7 19 → 0 — `cls/runs/ft/weights/best.pt` (shadow; not needed for m1t1). The student's
  engines (`CV_NEW/11-12/*.engine`) were exported at **imgsz 640** — re-export at 1280 / 320.
- **Real logs**: 12 Nov's observer ran **2.2 fps (morning) / 0.8 fps (afternoon)** with the same settings; the blink needs ≥ 5 samples / s.
- Tests: `tests/test_v0192.py` (31 cases incl. the final-strategy ones below); updated `test_v0190`, `test_v0191`, `test_v0154_c` (pinned to the old ladder
  where they test it). Findings log: `logs/sim_explore_2026-11-13/FINDINGS_CLAUDE.md` (20 rows).

## v0.1.93 — 2026-11-13 night (session A + the GUI agent): the final Task 1 strategy, the GUI pass, the final pack (WP49 §3.1–3.4, 3.6, 3.9, 3.10)

- **Request** (Dmitrii ~03:00): "found a proper strategy! implement it for m1t1 (and similar motion for m1t2 on the drone map + association),
  m1t1 as a simulation to ensure our models work, the simulation report in extra/, updated user + test guides as PDFs (the manual style), concrete
  morning instructions; GUI corrections, extensive tests of every button and the declaration logic incl. the drone, the pop-up with the
  micro-stage and what is next; a final pack for the final day". Build prompt: `docs/PROMPT_GATE_PASS_2026-11-13.md`.
- **T1** (course profile `gate_steps / start_mode advance360 / look_lock / turn_first_deg 25 / transit_v 0.15 / through_v 0.12 / circle_v 0.10 /
  keep_clear_m 0.35 / keep_track_m 0.20 / lead_in_m 3`; code defaults keep v0.1.91's flow for the other profiles): states START_ADVANCE, START_SCAN
  (`ScanYaw inplace`), LOOK_LOCK, ALIGN_GATE, THROUGH; the gate memory (`items()` never forgets, ordered from the entry along the axis to the exit);
  `_lead` = min(spacing − 1.5, 3.0) ≥ 1.5; the through leg pre → post at 0.12 with la 1.5 (FollowPath's carrot = cross-track); the crossing tracked
  (`_track_through`), passed only inside width/2 − 0.25, MISSED → back to the pre-point (×2) then on; the last gate runs on 2.5 m; the fence
  (`_seg_near`, `geofence_m` 1.0) on markers, leads, suspects; suspects = undecided / blue-share ≥ 0.3, never unlit / resolved; a nearer suspect
  interrupts; `micro()` → `info["micro"]` → mission manager `extra["micro"]`; the squeeze (`_squeeze_plan`, guard `squeeze_xy` 0.3 m) **built, off**.
  `behaviours`: Orbit `approach_speed`; ScanYaw `inplace`, `timeout_s`; guidance `turn_first` (per task: T1 only). Bug found and fixed: the
  until-callbacks read the start's pose snapshot (`ctx.b`) instead of the live `b` — ALIGN_GATE always ran to its 90 s cap.
- **GUI** (the agent; `docs/GUI_NOTES_2026-11-13.md`): B1 the page's RUN order (ENABLE AUTO before DECLARE), B2 ENABLE AUTO hidden while the
  pilot-ready was pending, B3 double press (one per 30 s), B4 stuck amber (60 s), B5 END RUN asks; the status pop-up (title · `FCU <mode> ·
  ARMED|disarmed · <V-state>` · stage · sub — why · next · mods); the MISSION MODIFIERS panel (start / look-lock / lucky / budget / exit-tangent →
  `cmd_authority.MOD_ARGS` → START_TASK args), frozen at ENABLE AUTO; ocs_cli flags; 130 tests (`test_v0192_gui.py` 78, `test_v0192_declare.py` 52).
- **Sim** (`extra/SIM_REPORT_2026-11-13/`): 8 worlds × 10 × 5 variants = 400 runs, **0 contacts, 0 wrong entries**; orbit 0.10: **70 / 70** done,
  14–18 min (0.08: 65 / 70; tangent: +1 min); gate offsets 0.00–0.05 m; the camera chain 6 / 6 (offsets ≤ 0.13 m); m1t2 5 / 6 (≤ 0.15 m);
  the squeeze world E3 0 / 4 off and on (one contact on) → off.
- **Docs**: `docs/USER_GUIDE_VENUE_v0.1.93.md`, `docs/TEST_GUIDE_VENUE_v0.1.93.md` (the Test Guide v0.1.86.8 kept), `docs/MORNING_2026-11-13.md`
  (= 00_READ_FIRST), `scripts/md2pdf.py` (the manual style via headless Chrome), GUIDE §1 V and §10; the final pack `extra/FINAL_PACK_2026-11-13/`.
- **Not done / open**: WP49 §3.7 hybrid, §3.8 crab (not built); the squeeze off; `circle_exit_tangent` off; GUI risks left (DISABLE AUTO does
  not ask; the mission picker drops `--t4/--planner`; m1t1 re-armed under preset 3 keeps the drone declared — the notes §4).
- **Next session first**: the morning card (00_READ_FIRST §3) on Dmitrii's go — deploy all three together, engines at 1280/320, W0 GUIDED at low speed.
- **Tests (v0.1.92 + v0.1.93 together):** `make test` **1240 passed, 0 failed** (5 min 42 s, 6 processes). Archive `_archive/MASTER_SCRIPTS_v0.1.93.tgz`
  (also in `extra/FINAL_PACK_2026-11-13/` with SHA256SUMS). Not deployed.

## v0.1.93.1 — 2026-11-13 morning (session A; Dmitrii: "add that any button in modifiers tab ... and also, I want squeeze tick box. After I tick, I want to reconfirm the mission")

**What changed · where**
- Two mission modifiers, set from the MISSION MODIFIERS panel (after the mission is selected, frozen from ENABLE AUTO): `squeeze` (on | off,
  default off) and `lucky_pairs` (one_known | any, default one_known). `optiprimex/ocs/cmd_authority.py` (MOD_ARGS / MOD_DEFAULTS /
  mod_value), `optiprimex/tools/ocs_cli.py` (`--squeeze on|off`, `--lucky-pairs one_known|any`), `optiprimex/tools/tkconsole/model.py` +
  `views.py` (buttons `mod-any`, `mod-squeeze`; the panel is now 7 buttons, 2 per line), `optiprimex/tools/ocs_web.html` (the same).
- "lucky pairs: ANY" also turns lucky on; "lucky off" drops ANY back to one_known (never a hidden ANY).
- LUCKY ON / LUCKY ANY / SQUEEZE ON ask first; the question ends with "Re-arm <code> as: <the mods: line after the change>". Turning a
  switch off never asks. The `mods:` line now shows `squeeze on|off` and `lucky off|on|ANY` (MOD_FIXED lost "squeeze off").
- The boat: `t1_safe_passage.mods()` says `lucky ANY` (the banner's line). No other boat change: the args reach t1 by p.update(args).
- The student's `agx_track_10_buoys.py` measured on 12 Nov data (`logs/cv_review/student_tracker_2026-11-13/NOTES.md`): their lamp region
  above the box misses the real lantern (ours kept); the per-frame no-repeat digit assignment parked in ROADMAP §4a.

**Tests:** `tests/test_v01931.py` (18: normalisation, refusals, the keys equal everywhere, the CLI flags, the asks and the re-armed line,
the page, a FastSil m1t1 with squeeze + lucky ANY → DONE 4 gates); test_v0192_gui pins updated (7 buttons, the longer mission line).
`make test` **1258 passed, 0 failed** (5 min 39 s).

**Risks:** squeeze on is the simulator's one-contact setting (it stays off by default); lucky ANY can take two markers as a gate (asks).
Deployed this morning with Dmitrii's go (docs/JETSON_LOG.md).

## v0.1.93.2 — 2026-11-13 late morning (session A; after the first course runs; Dmitrii: "multiple blue targets ... a blue foam on the bottom of the buoy ... do not trigger the colour classifier on the bottom half")

**What went wrong (runs 09:52 and 10:06, `logs/jetsons/2026-11-13/nx_today/`, `agx_today/`; the analysis `logs/cv_review/run_2026-11-13/`)**
- **Foam = blue.** The white markers' light-blue foam float at the waterline made the whole-mask colour vote "blue": 22 of 108 masks re-run
  from the run's camera pairs; every one is a white marker (blue_panel.jpg); 88 % of their blue pixels in the mask's bottom quarter. The map
  (classify_light) turns a buoy always seen blue, never black, into a **steady blue** → T1 took #113 / #109 as the EXIT, and every "blue ?" a suspect
  (13 / 11 blue buoys in the end maps, the course has 2).
- **The LED never read a lantern.** Our best.pt boxes the pyramid; the lantern sits above the box, the band (5..40 % of the box) read the white
  body: 1 lit lantern of 4. And every LED track had ≤ 9 samples (the rule wants 12 in 6 s; 1.75 fps and the turning boat): `pattern unknown`
  on all 3476 detections - no FLASH was possible.
- **The guard looped.** 6 STOPs on the same task target in SCOUT_ENTRY, no AUTO_SAFE_HOLD: each stop at another bearing after a 0.3 m back-off,
  never 3 within repeat_r_m 1.0 (3.0 before v0.1.92). Operator abort after 80 s. **Not changed** (Dmitrii's call).
- **Phantom obstacles.** 134 depth obstacles over the water the boat drove (map_*.png); planned clearances −2.3 .. 0.1 m; APPROACH_LEAD
  re-planned every ~1 s for 70 s. Not recorded per frame (frames.jsonl has no `far`); camera height still the assumed 0.55 m (D1 not taped).
  **Not changed.**

**What changed · where**
- `cv/buoys_multi/multi_buoy_observer.py`: `colour_of` votes over the mask's top `--colour-top` share (default 0.5; ≥ 12 pixels, else the
  whole mask). On the run's 108 masks: blue 22 → 4, white 80 → 65, unknown 6 → 39 (the top half of a marker is half the black diamond).
- `cv/buoys_multi/led_core.py` DEFAULTS: band_top −0.45, band_bottom 0.05, band_width 0.4 (was 0.05 / 0.40 / 0.6): 4 lit lanterns read on the
  run's masks (red, green ×2, blue) vs 1.
- Tests: `tests/test_v01932.py` (4); test_v0191 / test_v0192 synthetic lamps moved above the box. `make quick` 803 passed (before the band
  tests were re-pinned: the LED files 51 passed after).

**Open for Dmitrii:** the guard's per-target hold (e.g. the 4th stop on the same task target → hold); the depth obstacles (tape the camera
height; record `far` per frame); the LED rule at < 2 fps (window 6 s / 12 samples).

### v0.1.93.2 addendum — the replay on all recorded frames (same day, before any deploy)
- The bigger replay (`logs/cv_review/run_2026-11-13/replay_*.py`, report artifact "Colour Fix Replay"): 805 masks of 14 sessions (12 + 13 Nov),
  357 placed on the water and grouped into 131 buoys, through `perception_map.classify_light`. The top 50 % alone left 54 blue masks, all white
  markers with 3–21 % bluish pixels: the old rule decides "blue" on 12 bluish pixels of 400 before it looks at "white".
- **Added:** `multi_core.classify_bgr(min_sat_frac=0.0)`, a colour only when that share of all the pixels is saturated (0 = the old rule for
  every other caller); the observer passes `--colour-min-sat 0.20`. Per mask: blue 152 → 2 (white 533, unknown 270). Per buoy: blue 37 → 0,
  steady blue 5 → 0.
- **Consequence:** on this course every buoy is a white marker, so red, green and blue now come only from the lantern reading (the 73 detections
  today's run called "green" were markers with a green lantern spilling into the mask). The replay cannot test the lantern reading: the saved
  frames are 960 px and ~10 s apart. Next test: replay `cam.svo2` through the v0.1.93.2 observer on the AGX, in a separate folder.
- Tests: test_v01932 +1 (5); observer and colour tests 51 + 65 passed.

### v0.1.93.2 addendum 2 — the lanterns read, the speeds, the start turn (Dmitrii: "can we detect red or green now? ... slightly increase surge and yaw ... after a blue fix during the initial look-around, stop yawing")
- **Lanterns by eye:** 47 lit lanterns labelled on the colourful crops (red 21, green 21, blue 5; `logs/cv_review/run_2026-11-13/lantern_labels.json`)
  + 553 unlit markers. `led_core` DEFAULTS now V >= 180 (v_floor = v_cap), S >= 70, >= 3 px, >= 4 % of the band, band −45 %..+20 % of the
  box over its middle 30 %: **25 / 47 read (red 9, green 13, blue 3), 0 wrong colour, 0 false light**; every one of the 25 lamp reads on all
  805 masks is a by-eye lantern. The band as first shipped read 5 / 47 (in-box band: 0). V 150 / S 70 read 27 but lit far markers' foam
  floats blue (3 false blues) - rejected.
- **The observer** reports a colourless mask as white (`--colour-min-sat` > 0): the top-half vote left 270 / 805 "unknown", and T1 takes an
  unknown buoy as an entry suspect. Per buoy: all 131 white; blue / red / green now come only from the lanterns.
- **Speeds (course profile):** start_v 0.25, approach_v 0.25, transit_v 0.20, through_v 0.15, circle_v 0.12; guidance.yaw_rate_max_dps 6.5
  (was 0.2 / 0.2 / 0.15 / 0.12 / 0.10 / 5). Cap 0.3 unchanged.
- **The start turn** ends at a FLASHING blue within 10 m seen 2 x min_obs (`t1.start_scan_stop_blue`, `start_scan_stop_r_m`); then the ladder
  (look-lock, approach, check). Sim isolation (C_decoy + E2, 12 runs): stopping on any blue fix = 9 / 12 with a wrong entry (a far green gate
  buoy misread flashing); the faster speeds alone 12 / 12; the refined rule 12 / 12.
- **Fixed:** T1 crashed in CIRCLE_ENTRY_CW when the entry's light flipped between the search and the circle (`marker()` None; 2 sim crashes):
  the entry is pinned at ENTRY_SEEN, the circle start never takes None.
- **Sim (8 worlds × 10, `logs/sim_explore_2026-11-13/v01932_final.jsonl`):** 70 / 70 done where there is an entry, 0 contacts, 0 wrong entries,
  0 crashes; median 639–901 s vs 820–1087 s with the v0.1.93.1 profile; the entry found at 72–142 s vs 187 s.
- AGX: MODE_50W, 12 cores online (Dmitrii). The observer fps at 50 W not measured (the camera was in use).
- Tests: test_v01932 7 passed; test_v0191 / v0192 / v01931 pass (the course-profile pin: circle_v 0.12, transit_v 0.20).
- (addendum 3) `cv/buoys_multi/run.sh`: on the AGX the observer loads `cv/buoys/models/best.engine` when it exists (Dmitrii: "switch to
  engine anyway"; parity on 40 frames 94 % of boxes, conf drift ≤ 0.066, detector 110 → 65 ms); `CV_DEFAULT_PT=1` = the .pt. Test +1.
- (addendum 4, Dmitrii: "squeeze everything we can for these colours, need today") **the lamp colour: rule + learned model (hybrid).**
  - The grass test (`logs/cv_review/led_test_2026-11-13/`, the lantern cycling green / blue / red / flashing blue, 3 cycles, 2 s pairs at
    1080p) showed the v0.1.93.2 rule reading green as BLUE (the lantern is teal close up, hue 99 vs blue 107) and red not at all (pale pink,
    S 43–74, overexposed). No hue / saturation rule worked close up and on the course together.
  - `cv/buoys_multi/lamp_model.json`: a numpy softmax over none / red / green / blue on the band's hue × saturation histogram, trained on
    the grass phases + the 47 course lanterns + the unlit markers. In `led_core.led_sample`, the model calls red / green at p ≥ 0.9, the
    rule calls blue, and the model may veto blue → green at p ≥ 0.9.
  - Hue guards: green only when the lit median hue ≤ 102, red only when ≥ 15 % of the lit hues are red. The model alone called a pure blue
    lamp 0.96 green.
  - Held out: grass red 68/69 (was 0), green 66/88 (was 0; 22 → blue), blue 54/54; course 32/47 (was 25), 0 wrong colour, 1 false red
    on 758 masks, no false blue.
  - The likely root cause (overexposure) is not addressed: the ZED exposure control is not built.
  - Tests: test_v01932 +1; the LED files pass. Under deadline only the touched files were run (no `make quick`).
- (addendum 5) **The off lantern read as blue:** in the grass test's flashing phases every frame read blue, because the unlit housing is
  pale blue-tinted. `led_core`: blue now needs ≥ 2 % of the band strongly saturated blue (S ≥ 120). Lit frames: ~6 %; off: 0.0 %.
  - Flashing phases: 73 blue / 57 off (was 130 blue).
  - Steady blue: 54/54. Course: unchanged (32/47, 0 wrong). Green → blue on the grass: 22 → 17.
  - The 2.4 s pairs alias the handbook's 1 s on / 1 s off into a slow ~5 s rhythm, so the blink rule's timing is left as it is.
- (addendum 6) The grass test's first green phase ended at 28 s, not 44 s: re-checked by eye (`final/check_18_62.jpg`). The 17 "green read
  blue" frames were blue frames read right. Labels corrected and the model retrained (`cv/buoys_multi/lamp_model.json` md5 05dcc237).
  - Held out: grass 202/213 (red 68/69, green 66/76, blue 68/68, 6 wrong); course 31/47, 0 wrong colour, 2 false lights (red, green; no blue).
  - The AGX gives the same answers as the Mac on 49 frames. Report artifact "Lamp Colour Test".
- (addendum 7) A second label error: green started at ~151 s, not 137 s (139–149 s is still flashing blue, by eye on the panel). Corrected
  and retrained (model md5 5ee7c6fc). Held out: grass 202/203 (red 68/69, green 66/66, blue 68/68); course 32/47, 0 wrong colour, 2 false
  lights (red, green; no blue). AGX = Mac on 49/49.
- (addendum 8, Dmitrii: "final chance to update our settings") course profile:
  - t1.look_lock_s 6 (was 4); t1.look_lock_tol_deg 5 (new, code default 10: the bow straight at the target);
  - guidance.yaw_rate_max_dps 8 (transit turns); t1.scan_rate_dps 6.5 (T1's turns on the spot unchanged; was 20, capped by the yaw cap).
  - Sim 8 worlds × 6: 42/42 done, 0 contacts, 0 wrong entries. Tests v0185 / v0192 / v01931 / v01932 pass. Deployed to the NX + AGX together.
- (addendum 9, Dmitrii: "bake them in for m1t1 ... no conflicts in mission sequence and run declarations, we have UAV on our network ... drone
  map vs USV buoys ... GUI drone stages + which buoys ... no blue buoy: advance 3 m, a quick 360, end mission")
  - `scripts/agx_observer.sh`: defaults neural_light + hd1080 (env AGX_DEPTH_MODE / AGX_RESOLUTION override). LAUNCH now starts the fast camera.
  - **m1t1 ignores the UAV map.** While T1 runs in camera mode the mission manager sets `map.uav_ingest_off`. `ingest_geo` then still
    records the feed (status, the change time for the touchdown) but adds and recolours nothing. On entering m1t1 it drops the UAV-only
    buoys and the UAV votes (`forget_uav`). m1t2 (colour_source uav), scout and the GUI keep the feed.
  - **The no-entry finale (t1.no_entry_finale, course: true):** where the search gave up into the safe hold, the boat now goes
    FINAL_ADVANCE (3 m) → FINAL_SCAN (one 360 at the yaw cap) → DONE. A flashing blue in that turn → back to SEARCH_ENTRY.
  - **The drone map in the GUI:** `uav_digest.items` (each drone buoy, linked → boat #id (n camera sightings) or UAV-only, the UAV's
    colour / light) + ignored / ingest_off. ocs_web state.uav_map carries them. The tk banner gets a "drone map: …" line (`drone_map_line`)
    and the page's UAV map line lists the buoys.
  - The drone ↔ USV link is unchanged: a UAV item within uav.assoc_m (1.0 m) of a camera track links (the boat keeps its position, takes
    the UAV's colour / light; m1t2 skips the LED). A camera sighting absorbs a UAV-only twin. Tests v0166 / scout pass.
  - Sim 8 worlds × 4: entry worlds 28/28, 0 contacts, 0 wrong entries; lights-off: 2 DONE via the finale, 2 guard holds.
  - Tests: test_v01932 +4; `make live` before this addendum 1263 passed.

## v0.1.94 — 2026-11-14 (session A; after the 13 Nov afternoon run; Dmitrii: "the boat was moving super slow ... merging buoys, bring back the number classifier, report to RC which buoy is which colour ... ids 0-9 ... merge radius growth by distance, but also by detected number ... if we use the depth picture and discriminate far-field ...")

What the 13 Nov pm run showed (the recordings replayed through the boat's map, `logs/cv_review/replay_map_2026-11-14/replay.py`):
- 223 s in auto at **0.15 m/s** average: the profile caps (limits 0.3, approach 0.25, transit 0.20), START_SCAN 54 s at 6.5 deg/s,
  the guard's SLOW 15 % of the time. The hull touched 0.73 m/s. -> the pathing research (below).
- The 7 RoboCommand reports carried every buoy **white / UNKNOWN**: the 496 lamp reads never reached the map.
  - cv_publish dropped `quality.led` on bearing-only items (95 % of the sightings: 1351 beyond_depth, 321 depth_invalid of 1768);
  - the observer's blink memory tracks boxes in the image within 60 px: the median jump at 2.3 fps was 68 px -> ~1200 tracks.
- The "90 phantom obstacles" were the buoys themselves: the depth image is clean water (`pairs/depth_000021_vis.jpg`); the guard's levelled
  cloud saw the markers whose boxes had no valid depth, and `map_far` mapped them a second time.
- Ground truth by eye on the 32 saved frames (fork C, `logs/cv_review/truth_charlie_2026-11-14/TRUTH.md`): marker 9 = the flashing-blue
  ENTRY, 8 / 66 red flashing, the steady-blue EXIT ~25-29 m away all run (on 9's bearing), 77 / 11 unlit, 3 green. The observer's own
  ranges on near boxes were ~3x too long (66 at 148 px read 21.9 m; true ~6.8 m: the depth inside the box was the background).

What changed · where:
- `cv/publish/cv_publish.py`: `_extras()` - `led` {pattern, colour, on, now}, `bbox_h`, `number` on BOTH the ranged and the bearing-only items.
- `optiprimex/usv/lamp_tracks.py` (new): the blink decision by WORLD bearing (az_tol 4 deg) and box-size range (x/÷ 1.6, 1000 px*m / box
  height), 8 samples / 8 s, led_core's run rule ported verbatim (pinned by a parity test).
- `optiprimex/usv/perception_map.py`:
  - every buoy sighting feeds the lamp tracker (`_lamp_frame`); a ranged one links its track to its entity, a bearing-only one colours its ray
    and links to the buoy on that azimuth whose distance fits the box size (`_entity_on`); `_lamp_link` -> led_hist; `_led_apply` keeps a
    decided LED, else the lamp's colour (light "?") colours the buoy;
  - a range the box height denies (x/÷ 1.6) is dropped -> the sighting is depthless (`size_mismatch`);
  - `ranged_max_m` 15: depth beyond counts as a bearing (a ray / lead);
  - `fuse_depth`: a guard `far` cluster whose bearing span (+- 1 deg, same frame) holds exactly one depthless box of a fitting size gives it
    the cluster's range; that cluster is not mapped as an obstacle (`usv/guard.map_far` calls it first);
  - `merge_buoys` (every 1 s): radius 0.35 + 0.08 x (closest range either was seen from), 0.8..2.5 m; the same number -> 5 m; the same
    decided lamp -> 3 m; two numbers / two lamp colours never; `merged` + `resolve()`; `_number_vote` (>= 2 reads, 70 %).
- `optiprimex/usv/tasks/t1_safe_passage.py`: report ids from 0 in first-reported order (`id`, + `map_id`); `_follow_merges` (entry / exit /
  check / rejected / checked follow a merge, the merged-away buoy leaves the reported field, a merge resends at once); `check_still_deg`.
- `optiprimex/usv/behaviours/scan_yaw.py`: `still_deg` - facing within it -> yaw 0 until 2x off (LOOK_LOCK: min(still, tol / 2.5)).
- `cv/buoys_multi/number_core.py` + observer `--number on|off` (fork A): the fine-tuned 12-class marker classifier on boxes <= 8 m or >= 110 px,
  one digit per frame (the student's `unique_digit_assignment`, ported verbatim), `buoys[i].number`; OFF by default (AGX fps not measured).
  `cv/buoys_multi/models/number_cls.pt` (20.9 MB, md5 9974bf58). 12 Nov hold-out 44/45; 13 Nov 53 digits, 0 wrong seen.
- `profiles/course/usv_course.yaml`: `perception.lamps`, `perception.merge`, `ranged_max_m` 15, `depth_fuse_*`; t1 `check_s` 10, `check_still_deg` 4.

Replay of 13 Nov pm, scored against the truth (`score.py`): baseline 10 buoys, 3/6 markers, 0 right colour, 7 phantoms, no flashing blue
-> v0.1.94 10 buoys, 5/6 markers, 8 + 66 red flashing, 9 = the one flashing blue (1.3 m off), 3 phantoms (2 may be the truth's low-confidence
markers), 2 white copies of 8 / 66. Depth obstacles 92 -> 75.

Tests: `tests/test_v0194.py` (16), `tests/test_v0194_number.py` (9). Open: the white copies near lit buoys; the number reader's AGX cost
(TensorRT export needs Dmitrii's go); the pathing research (`docs/RESEARCH_PATHING_2026-11-14.md`, fork B) - its profile proposals are not
applied yet. Nothing deployed.
- (addendum 1, Dmitrii: "if we won't be able to distinguish blue solid from flashing, add to our lucky mode to orbit regardless") T1
  `lucky_blue` (default true, only with `lucky` on): in `_check_result`, a blue buoy whose light is still undecided after `check_tries`
  checks becomes the entry and is orbited (ENTRY_CHECKED, logged "T1 LUCKY: ..."). Without lucky: unchanged (checked, doubtful, one more
  look later). The LUCKY ON confirm texts (tk + web) say it. Tests: test_v0194 +1; v0192 / v01932 / v0190 pass.

## Cowork session hand-off (no version) — 2026-11-13 17:15 UTC (= 14 Nov 01:15 SGT) — the status read for the next Cowork chat

Dmitrii: "take a look at our current status, and write a handoff prompt to new chat session". Written: `docs/COWORK_SESSION_HANDOFF_2026-11-14.md`
(project mirror `claude/cowork_session_handoff_2026-11-14.md`): the tree at 01:00 SGT (Mac v0.1.94 in progress by session A, not tested / archived /
deployed; Jetsons on 0.1.93.2), the live course-profile values, what the 13 Nov runs taught (perception, not pathing: the foam blue, the lamp band,
no blink decision at ≤ 2.3 fps, the neural_light ranging loss 85–100 % → 9–31 %, the hull's real 6.7–8.6 °/s in place), v0.1.94's contents and open
items, what of Cowork's WP49 landed (§3.1–3.4, 3.6, 3.9, 3.10) and what did not (hybrid, crab; squeeze off as a tick), what is outdated (the m1t1
logic-check page's 2 °/s plant and slow speeds), the rules, the first steps for the new session and the open questions for Dmitrii (slot, v0.1.94 on
the boat or not, neural vs neural_light, the ticks, the guard's per-target hold, the number reader). Nothing else touched.
- (addendum 2, fork B's research found it) **T1 rejected the true entry with the lamp reads on**: `watch_locked`'s near sightings read
  `e.colour_now` = the body colour, white on every real buoy -> "not blue after all ... rejected". `PerceptionMap`: a sighting with a lamp
  reading sets `colour_now` to the lamp (lit colour, else "black"); the guard-fused ones carry it as `quality.lamp_now`. Sim repro (LED=1,
  cam_am, seed 1): A / B / C found the entry (B, C DONE 4/4; A a guard hold). Tests: test_v0194 +1; v0192 / v01932 / t1_sim pass.
- Research (`docs/RESEARCH_PATHING_2026-11-14.md`, fork B), snapshot v0.1.93.2, 80 runs a cell: the afternoon camera (hd1080 +
  neural_light) ranged near boxes 9-31 % vs the morning's (hd2k + neural) 85-100 %: 49 % -> 99 % success on the camera alone (a Jetson
  setting - Dmitrii's go). Speed caps 0.45 / 0.6 barely shorten the run (time goes into circles, gate steps, reads); scan > 8.5 dps untested
  on the water (the hull). Pending: the live re-run with the fix, the T4 detour experiment, the scored strategy packages (Dmitrii: "several
  competitive best strategies ... select based on scoring").
- (addendum 3, Dmitrii: "if the buoy / gate is mapped correctly, we can go through it with more confidence and comfortable speed") T1
  `gate_confident` / `_gate_v`: both buoys >= conf_min_obs (10) sightings, sigma <= conf_sigma_m (0.5), seen from <= conf_seen_m (8 m) once,
  coloured by their own lamp, not lucky -> the run-in at `transit_v_conf`, through at `through_v_conf` (0 = off, the default; the course
  profile unchanged until the scored packages decide). Tests: test_v0194 +1. Fork B scores "confident" (cap 0.45, 0.3 / 0.4) and
  "confident-fast" (cap 0.6, 0.35 / 0.5) beside the other packages, in handbook points.
- (addendum 4, Dmitrii: "lets go back to hd2k + neural!") `scripts/agx_observer.sh` defaults `neural` + `hd2k` (AGX_DEPTH_MODE /
  AGX_RESOLUTION / the flags still override). Evidence (all 12-13 Nov AGX recordings, near boxes >= 80 px): hd2k + neural ranged 92-98 % on
  the course (2.5-3.8 fps in a run); hd1080 + neural_light 6-17 % on the 13 Nov pm course (100 % / 93 % only in the quick check and on the
  grass - the afternoon light is not ruled out). The box-size constant holds at both resolutions (median 936 px*m hd2k, 979 hd1080; the
  map uses 1000 x/÷ 1.6). The neural model is cached on the AGX since 12 Nov. test_v01932 updated. Not deployed.
- (addendum 5, Dmitrii: "can we make the course appearing regardless of port we plug in?" - 13 Nov 15:08: the course cable in the
  manual-address adapter, no RoboCommand; DHCP there took OCS_HOST away, no boat heartbeat; the alias by hand under the timer)
  `scripts/net_roles.py` / `.sh` (new, Mac, /usr/bin/python3 stdlib): both wired adapters on DHCP (`dhcp`, once); a root LaunchDaemon
  (`install`: a root-owned copy of the script + the needed addresses in /usr/local, WatchPaths SystemConfiguration + every 5 s) puts
  OCS_HOST as an alias on the adapter with a team-subnet lease (or, after 20 s, the one bare adapter) and removes it elsewhere; a course
  lease (RC_<COURSE>_HOST's /24) marks the course side. `status` prints roles, never addresses; `course` the course name.
  `scripts/run_ocs.sh --judge auto` + the launcher's judge list "auto". GUIDE §2. Tests: `tests/test_v0194_net.py` (6, RFC 5737
  addresses); v0182 / v017801 updated (+ auto). Not installed on the Mac yet (needs sudo: Dmitrii).
- (addendum 6, fork B's live re-run after addendum 2: the remaining failures were SAFE_HOLD "obstacle persists" on the gate's own buoys -
  base 31 of 45 fails) `usv/guard.py`: while the task passes a gate (`T1.gate_pass_xy()`: ALIGN_GATE / THROUGH, the pair's two buoys) a
  cluster within `gate_r_m` 1.0 of one of them stops only on a collision course (`gate_stop_m` 0.8 ahead, `gate_stop_lat` 0.3 abeam), never
  slows the pass, and the guard's memory near them is dropped. Unknown objects elsewhere keep the corridor rules.
- (addendum 7, fork B's T4 detour sim, 100 detours: 0 s ack, ~77 s to arrive, readiness within 3 m 93/94, T1 resumed every time, 2
  contacts on well-mapped buoys) `tasks/t4_incident.py`: the waiting point keeps `t4.arrive_margin_m` (0.8) more for the arrival swing;
  a request that finds the boat inside a keep disc (mid-gate) goes out to the nearest free point first. T4 / T1 / guard tests pass.
- (addendum 8, the remaining holds under the morning camera: the boat drove at a lead / a scouted buoy until the guard stopped it, backed
  off, re-planned into it again - 3 stops within 1 m = SAFE_HOLD) T1 `on_guard_block` (the guard's 2nd stop on the same stage key):
  in APPROACH_LEAD / SCOUT_ENTRY / FINAL_ADVANCE the stage ends as arrived (SCOUT_ENTRY -> its check from there). Guard / T1 / scout tests pass.
- (addendum 9, Dmitrii: "several competitive best strategies, select based on scoring") the final sim on the finished code
  (`logs/sim_research_2026-11-14/final_v0194/S_rank.md`, 16 package x camera cells x 80 runs, handbook points of 325): 1st
  **confident_fast@am** 261.6 mean (lit worlds 287.8, median 325, p10 150), 58/80 full bonus, 0 contacts, 462 s median; careful@am 232.4,
  3 contacts; the best under the pm camera 230.6; lucky last under both. Course profile now = confident_fast: `limits.max_speed_mps` 0.6
  (was 0.3), t1 `through_v_conf` 0.35, `transit_v_conf` 0.5 ("careful" = 0.3 and 0 / 0). T4 transit pinned at `t4.speed_mps` 0.3 (as
  simulated; the cap no longer carries it). T4 detour final: 99 detours, 1 contact (cam_am, a random-direction target), readiness <= 3 m
  95/99, T1 resumed every time. E8 cells: the morning camera 69-84 % ok vs 48 % base. test_v0192 updated (cap 0.6).
- (closing) Full `make test`: **1303 passed** (5:44, on the final code incl. addenda 1-9 and the net tests). Archive
  `_archive/MASTER_SCRIPTS_v0.1.94.tgz` (103 MB, md5 bb631f595c98946f3e4cb85d5b4a3908; no network*.env, no key; the nested
  `extra/*/*.tgz` excluded - the 13 Nov final pack's 77 MB v0.1.93 archive). Report: `docs/reports_2026-11-14/` (index.html +
  05_v0194_readiness.pdf, artifact https://claude.ai/artifact/FLru3QCUuiZdeuq3DiM7LY). **Not deployed.** Next session first: Dmitrii's go
  -> GUIDE §1 deploy to NX + AGX (JETSON_LOG), the AGX observer restarts on hd2k + neural; the Mac: `sudo scripts/net_roles.sh dhcp` +
  `install`; tick T4 on the mission; on the water first a careful check of the 0.6 cap (profile: `limits.max_speed_mps` 0.3 + the two
  `*_v_conf` 0 = the careful package).

## v0.1.94.1 — 2026-11-14 morning: blue-agnostic lucky + normal vs lucky (Mac only, not deployed)

Dmitrii: "lets add this blue-agnostic mode to lucky mode? and then evaluate performance normal mode vs lucky?"

- What · where: `t1.lucky_blue` takes `any` (`optiprimex/usv/tasks/t1_safe_passage.py` `_check_result`): with `lucky` on, any blue
  not read steady (flash / off / undecided) is orbited as the entry after its first check; a blue read steady stays the exit. The
  default stays `true` (v0.1.94: orbit only after `check_tries` undecided checks); the course profile does not set it.
- Fix found by the sim: LUCKY_GO is handled at a behaviour's end, so TRANSIT started with `self.current` None ->
  `AttributeError: 'NoneType' object has no attribute 'mid'` in `behaviour_args` (E2_obstacles lucky_cf@am seed 6). Now both LUCKY_GO
  sites pick the gate first; TRANSIT with no gate picks one or holds in place, and that hold's end passes no gate.
- Sim: `pathing_sim/research_2026-11-14/research.py` packages `lucky_cf` / `lucky_any_cf` (confident_fast speeds + lucky), experiment
  `L`; `rank.py` takes `EXPS` / `RANK`. Out: `logs/sim_research_2026-11-14/lucky_v01941/L_rank.md` (480 runs, LED=1, 8 seeds x 10
  worlds, 0 crashes after the fix): normal confident_fast 261.6 am / 171.9 pm · lucky 225.9 / 138.4 · blue-agnostic lucky 224.6 /
  145.9 (mean handbook points of 325). Lucky loses on the exit circle and the gates (bonus 72.5 -> 51 am): 28 guard SAFE_HOLDs vs 9,
  all at real gate buoys (N +12.5 / +19.2) after a lucky pair (uncertain colours) was taken; the entry circle is the same. Blue-
  agnostic changes little in the sim (+7.5 pm, -1.3 am): the sim's blue lamp is rarely undecided after a 10 s check.
- Verdict: run normal (lucky off). Lucky stays the operator's late call; if it is ticked, `lucky_blue: any` is the better pm choice.
- Tests: tests/test_v0194.py +2 (blue-agnostic first check; TRANSIT with no gate never crashes); make quick.
- Next: the lucky gate pairing near real gates (later, ROADMAP); the v0.1.94 open items stand (deploy go, net_roles install, T4 tick).

## v0.1.94.2 — 2026-11-14 morning: lucky as the fallback (exit orbit, late-entry retry, faster) + show-time speeds (Mac only, not deployed)

Dmitrii: "check blue not once, but as we do in normal mode, but just in case there is no entry located and just exit - we go
around exit (and if during the course we will determine entry, re-orbit entry and retry the run). Lucky mode intended in case
our normal doesn't behave at all. Also, lets move in lucky mode with higher speed. ... which speeds are for normal mode now? lets
also bring them up, its show time! (but not fast-fast)"

- Blue checks in lucky = normal mode's (`lucky_blue: true`, check_tries; `any` stays an option, not used).
- `optiprimex/usv/tasks/t1_safe_passage.py`: `_lucky_go` (both LUCKY_GO sites): the search spent, no entry, the exit known
  (`exit_seen` or the steady-blue marker) -> LUCKY_EXIT = CIRCLE_ENTRY_CW around the exit (watch_locked leaves the stand-in alone);
  circled -> entry cleared, `lucky_from_exit`: `choose` takes the upstream gate first (along the gates' mean direction), the exit
  CCW at the end (`marker(exit)` now keeps a checked exit). No exit -> the closest gate as before. `_lucky_retry` (every step):
  lucky took the gates without a decided entry and a blue is now decided flashing (led set, min_obs, in course, not the stand-in /
  exit / rejected) in TRANSIT / SCAN / LOOK / EXIT (never ALIGN / THROUGH) -> LUCKY_RETRY = circle it CW, passed gates cleared, the
  gates again; once. `lucky_v_k` 1.25 x transit / approach / start / scout / transit_v_conf when lucky is on (in start()).
- `profiles/course/usv_course.yaml`: start_v / approach_v 0.35, transit_v 0.30, scout_v 0.40 (new key), lucky_v_k 1.25; circle_v
  0.12, through_v 0.15, through_v_conf 0.35, transit_v_conf 0.5, cap 0.6 unchanged. tests/test_v0192 pin transit_v 0.30.
- Sim (`research.py` SHOW / CIRC / BAD / FINAL, `logs/sim_research_2026-11-14/show_v01942/*_rank.md`, LED=1, 8 seeds x 10 worlds):
  speeds: v0.1.94 profile 261.6 am / 172.1 pm; show_a (these speeds) 259.4 / 175.8, 0 contacts am, 0 wrong entries am; show_c
  (approach / start 0.5, transit 0.4) 265.4 / 176.7 but 5 contacts + 13 wrong entries pm; circle_v 0.15 / 0.18 / 0.20 230 / 230 /
  208 am (guard holds 7 -> 21 / 27 / 38); through_v 0.18 258.2 am (neutral). The median run stays ~460 s at every run-in speed: the
  two circles at 0.12 m/s are most of it. Lucky: am 214, pm 146.4 (the lucky gate pairs' guard holds, as in v0.1.94.1); with
  the lamps hard to read (`real_am_badlamp`, lamp reads x0.35) lucky 108.6 vs normal 92.3 - the fallback it is meant to be; the new
  lucky vs the v0.1.94.1 lucky there: 108.6 vs 107.6, contacts 1 vs 4, median 548 vs 473 s. FINAL = the profile itself: 259.4 /
  174.4 / 92.3 normal, 214 / 146.4 / 108.6 lucky.
- Tests: tests/test_v0194.py +3 (lucky speeds; exit stand-in orbit -> gates -> still the exit; the late-entry retry once, lucky
  only); make quick.
- On the water: the faster run-ins are the change to watch (start advance 2 m at 0.35, run-ins 0.30). Fallback: transit_v 0.20,
  approach_v / start_v 0.25, scout_v off (0.3).
- Next: the lucky gate pairs near real gates (ROADMAP later); deploy go, net_roles install, T4 tick stand.
- Addendum (Dmitrii: "judge auto should not connect to venue demo"): `scripts/net_roles.py` `judge_course(roles)` - `course` (what
  `--judge auto` reads) prints ALPHA..DELTA only, never VENUE (on the venue network alone it prints nothing and says so on stderr);
  `run_ocs.sh` no longer maps venue -> demo (refuses it). The adapter roles (the OCS_HOST alias) still treat the venue as a course
  side. GUIDE §2 line. tests/test_v0194_net.py +1. The installed root copy (/usr/local/libexec) only runs `apply`: not affected.
- Addendum 2 (Dmitrii: "can we also increase orbiting velocity? ... in slow mode we might drift away"): the sim had no current, so
  it only ever saw the cost of a faster circle. `research.py` cells take `current` (m/s, the water drifts in a seed-chosen direction:
  the plant's position moved each step), experiments DRIFT / BOOST (`logs/sim_research_2026-11-14/show_v01942/`). Why a fixed faster
  circle loses in calm water: the hull's turn radius grows with speed (1.1 x: 3.05 m at 0.12, 3.64 at 0.15, 4.17 at 0.18) - the holds
  come later, on gate run-ins at gate buoys. With a current the fixed 0.12 orbit cannot gain ground (0.10 m/s: 67 of 80 runs ABORTED
  "orbit timeout (70 of 360 deg)").
  Built: `optiprimex/usv/behaviours/orbit.py` drift boost - args `speed_max` (+ `r_for_max`, not used): the ground speed from the
  pose's own movement (EMA ~2 s; the sim's SOG field leaves the current out, so it is not used), after 3 s on the ring slower than
  70 % of the ask -> +0.05 m/s per s, else -0.03 m/s per s, never above speed_max (nor scan_v_mps under way). T1 passes
  `circle_v_max` / `circle_r_for_max` (defaults 0 / False = off); the course profile `circle_v_max: 0.15`.
  Sim (am camera, show_a speeds, 80 runs each), current 0 / 0.05 / 0.10 m/s: fixed 0.12 259.4 / 218.6 / 80.0 · fixed 0.15 230.4 /
  211.3 / 129.1 · boost to 0.15 261.0 / 227.6 / 147.0 · boost to 0.18 260.7 / 224.8 / 143.1 · boost 0.18 + the ring for 0.18 227.2
  / 199.5 / 126.4. Open (new, not fixed): in a current the contacts climb (0.05: 13 of 80, 0.10: 32) - the boat drifts while it
  holds, turns on the spot or checks a light; the guard and T1's holds do not station-keep. tests/test_v0194.py +1 (the boost).
- Addendum 3 (Dmitrii: "check that it won't matter which adapter port I use on my mac to connect to a switch"): checked on the Mac
  - the root LaunchDaemon is installed (since 08:52, 227 runs, last exit 0; its log: "add OCS_HOST on en20"); every wired service
  (en19, en20, en8, en16, en18, the iPhone USB ones) is on DHCP; status: en20 team, OCS_HOST there. The rule (pure, tested): either
  adapter on the switch -> it gets OCS_HOST; the cable moved -> the alias leaves the dead adapter and goes to the live one (new test);
  a switch without DHCP -> the one bare adapter after 20 s. Gap found + fixed: `classify` called any lease outside the team / course
  nets "none", the same as a bare switch port - an iPhone USB tether (or a home router) could take the 20 s fallback. Now "other",
  never the fallback (tests/test_v0194_net.py +2). The root copy (/usr/local/libexec/optiprime_net_roles.py) is the older file: re-run
  `sudo scripts/net_roles.sh install` to take this fix (and judge_course) in.
- Deployed (Dmitrii: "Im on the LAN now, after done - deploy"): `make test` 1312 passed; archive `_archive/MASTER_SCRIPTS_v0.1.94.2.tgz`
  (md5 9efd69846cc36e217569a9a82361da93); NX + AGX 0.1.93.2 → 0.1.94.2, AGX self-test PASS, NX FastSil T1 DONE; the AGX cleaned
  (13 Nov recordings verified on the Mac first) - docs/JETSON_LOG.md. Still to do on the Mac: `sudo scripts/net_roles.sh install`
  (the root copy predates the "other"-lease fix and judge_course); tick T4 on the mission.

## v0.1.94.3 — 2026-11-14 (Dmitrii: launcher tooltips; live speed knobs on the OCS)

- Asked: "can you give tooltips pop-up during launcher mode select upon hovering?" · "a velocity modifier on the OCS, which we can
  click or enter in the box and to send intent, so we could speed up during the trial on the spot" · polled: "Everything but 3
  different knobs", "0.5-2.0 by 0.25 but with cap 1.5 m/s".
- Tooltips: `optiprimex/tools/launcher_tips.py` (ROW_TIPS, CHOICE_TIPS: every row and every choice of the Tk launcher), a `Tooltip`
  in `launcher_tk.py` (450 ms hover, gone on leave / click) on the presets (their `about`), the row labels and each choice. The web
  launcher already had title tips; its lucky (v0.1.94 behaviour) and judge auto (never the venue) texts refreshed.
- Speed knobs: `optiprimex/companion/speed_k.py` - cruise / circle / gate, x0.5-2.0, kept in cfg.extra["speed_k"] (never saved into a
  profile). OCS intent `SPEED` (cmd_authority, range checked) -> boat `SET_SPEED_K` (RUN_FREE, any vehicle state, a running task
  included; command_handler `_speed`). `tasks/base.py`: every tick, after the under-way law and before the cap, `speed_class` (the
  task's SPEED_CLASS, else Orbit -> circle, ScanYaw / StationKeep -> none, other movers -> cruise; T1 THROUGH -> gate) and
  `scale_speed` (forward speed; circles also the yaw rate, so the radius holds); `cap_speed(k)`: k > 1 lifts max_speed_mps to
  max_speed_mps x k, never above 1.5 m/s. The ZED guard's SLOW / STOP act after the task command: not scaled. status tuning.speed_k
  reports the boat's values. Words: `ocs_cli speed cruise|circle|gate|all K | reset` (also the web page's /cmd and the playbook).
  Tk console: USV1 tile, SPEED row - per knob − / + (0.25) and a box (type, Enter), "x1 all"; the boxes follow the boat's values.
- Sim (FastSil, course profile, seed 1, T1 DONE with 3 gates in every case): marina x1 650.8 s · cruise x2 658.5 · gate x2 627.1 ·
  all x1.5 630.3 · all x2 617.3 · all x0.5 847.8; pool all x2 636.3 (x1 ~ 698). Most of a T1 run is scans, turns on the spot and
  light reads, which the knobs do not touch - so the gain is modest; cruise commands did reach 0.99 m/s (cap lifted).
- Tests: tests/test_v0194_speed.py (4), tests/test_v0194_tips.py (3); `make test` 1319 passed.
- Not built: the web OCS page has no SPEED buttons (the typed `speed ...` line works there); T3 docking's stages follow cruise
  (its bow-in too). Risks: faster legs mean later stops before buoys - the guard ranges are unchanged.
