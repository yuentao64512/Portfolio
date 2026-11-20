# cv/cannon — the water-cannon link: the contract (v0.1.61, WP12-rest; v0.1.57 WP12, decisions D11–D13)

```
the observer that holds the ZED (AGX: cv/buoys_multi or cv/docking with --cannon red; sim: cv/sim_observer --cannon red,
                                 or the world's truth in the geometric sim: optiprimex/sim/fake_cannon_target.py)
    | aim_cv.find_target: the students' colour segmentation + this frame's depth -> the nearest RED blob in the cannon frame
    v
runs/latest/cannon_target.json  (cannon_target.v1, every frame)
    |                         task (m3t1 AIM_FIRE) -> mission manager -> local/<v>/cannon/aim (boat broker, 5 Hz while on)
    v                                                 |
cv/cannon/cannon_sender.py  <--------------------------+   aim_core.AimGate: the target only while the aim is on and < 1 s old
    |  UDP dock_color_aim.v1, HMAC-SHA256, 5 Hz            AND the target file is < 0.5 s old and RED; else target null
    v
the Pi: the students' color_cannon_receiver.py (snapshot: pi/)  -> pan / tilt servos; the pump only in their pump mode
    |  its JSONL log (color_receiver_logs/)
    v
pi/cannon_status_pub.py (ours, beside it on the Pi) -> local/<v>/cannon/status (2 Hz) -> boat window, OCS state.json "cannon"
    (sim: optiprimex/sim/fake_cannon_pi.py runs THEIR receiver in DRY_RUN + this publisher's loop)
```

**Decided with Dmitrii 2026-11-07**: aim at **RED** (handbook 3.3.4: the target window lights RED after the DockingReport and turns
GREEN when hit; `--color green` = the students' bench target); **one camera owner** (their sender opens the ZED itself, our observers
already hold it, so its per-frame steps are ported into `aim_cv.py` and run inside the observer); **a status publisher beside their
receiver** (it publishes nothing itself; theirs is not edited).

Nothing on this path arms or changes the FCU's mode: ARM / DISARM and the FCU mode are the operator's (typed in usv_ui, or the
pages' buttons since v0.1.50); the companion never arms by itself. There is no fire button on any page. **The pump**: their
receiver fires it only when started with `--live --enable-pump --confirm I_ACCEPT_GREEN_10_SECOND_WATER_CANNON`, `pump.enabled`
in its config and a valid arm file — and then **by itself**, once per stable lock (`stable_packets_required` packets within
`stable_angle_delta_deg` for `settle_time_s`), 10 s, with a cooldown. Our side's only say is the gate: no aim or no fresh RED
target → `target: null` → their receiver turns the pump off and centres the servos at that packet (and its watchdog after
`watchdog_s` without packets). Whoever starts the receiver in pump mode decides that the cannon may fire.

## 1. The aim topic (unchanged since v0.1.57)

`local/<v>/cannon/aim` on the boat broker, the envelope or a bare body, from `optiprimex/usv/mission_manager.py` `_cannon()`:
`on` (bool), `bay` (1..3), `window` (`upper` / `lower`), `window_id` (1 / 2: the FirefightingReport's), `t` (UTC). **Fresh** = received
less than **1 s** ago (`aim_fresh_s`); an `on: false` stops the target at the next packet.

## 2. The target file (observer → sender), new in v0.1.61

`cannon_config.json target_file` (default `cv/cannon/runs/latest/cannon_target.json`; `--cannon-out` / `--target-file` override; the
sim: `logs/bench/run_<stamp>/cannon_target.json`). Written atomically every frame:
`{protocol: "cannon_target.v1", sent_unix_ns, sent_utc, source, colour, status, target}` — `status` OK | NO_RED_TARGET |
RED_NO_RELIABLE_DEPTH | OBSERVER_STOPPED; `target` = their sender's target dict: `class_name: "dock"`, `color`, `confidence`
(0.5 + 0.5 × `color_fraction`), `color_fraction` (blob area / its box), `color_pixels`, `bbox_xyxy`, `x_right_m` (+ starboard),
`y_forward_m` (+ forward), `z_up_m` (+ up) from the cannon pivot (`camera_to_cannon` in `cannon_config.json`: their
`color_dock_config.json` section, **zeros until they send it**), `horizontal_m`, `range_m`, `bearing_deg`, `elevation_deg`, the depth
quality. Fresh = `sent_unix_ns` less than `target_fresh_s` (0.5 s; the sim's camera path 1.5 s) old.

## 3. The packet (sender → Pi), UDP

To `network.pi_ip:pi_port` (192.168.6.7:5005; `--destination`), `aim_rate_hz` (5 Hz), every packet whether or not there is a target.
`{"body": {...}, "hmac_sha256": "<hex>"}`, HMAC-SHA256 over `json.dumps(body, separators=(",", ":"), sort_keys=True)` with the key
file's stripped contents (≥ 32 characters) — their `signed_packet` / `decode_verified`.

| body field | meaning |
|---|---|
| `protocol` | **`"dock_color_aim.v1"`** — their receiver drops anything else (`WRONG_PROTOCOL`); `--protocol dock_cannon.v1` = our v0.1.57 packet (legacy) |
| `session`, `sequence` | the sender's run (uuid) and packet counter; their receiver refuses a replay / out-of-order sequence |
| `sent_unix_ns`, `sent_utc`, `source_host` | when / who; their receiver refuses a packet older than `maximum_packet_age_s` **by the Pi's clock** — the AGX and the Pi must agree on time |
| `status` | `OK` · `NO_AIM` · `AIM_OFF` · `AIM_STALE` · `NO_TARGET` (aim on, no fresh RED target) · `SENDER_STOPPED` |
| `target` | `null`, or the target file's target + the aim's `bay`, `window`, `window_id` |
| `color`, `window_id`, `aim_utc`, `why` | ours (their receiver ignores them): the colour aimed at, the window, the aim's `t`, why `target` is null |

Their receiver's checks (its `target_solution`, `color_cannon_config.json`): `class_name == "dock"`, `color` in `allowed_colors`,
`confidence` ≥ `minimum_confidence`, `color_fraction` ≥ `minimum_color_fraction`, `minimum_range_m` ≤ `range_m` ≤ `maximum_range_m`,
`y_forward_m` ≥ `minimum_forward_m`, every number finite, pan / tilt within the servos' `maximum_abs_angle_deg`. Pan =
atan2(x_right, y_forward), tilt = atan2(z_up + `camera_height_above_cannon_m`, horizontal) — **our z_up is already from the pivot,
so their `camera_height_above_cannon_m` must be 0** (or ours `camera_up_from_cannon_m` 0 and theirs the offset — one side, not both).

## 4. The key

Real: `network.key_file` in `cannon_config.json` (on the AGX the students' `buoy_link.key`), read at start, never printed, logged
or copied; never on the Mac. The simulator never touches it: `fake_cannon_pi` **generates** a random test key (`secrets.token_hex(32)`,
0600) into `--key-out` and writes it into the receiver's sim config.

## 5. The status back (Pi → boat broker)

`local/<v>/cannon/status`, 2 Hz, the envelope, from `pi/cannon_status_pub.py` (on the Pi; `pi/status_core.py`):

| field | meaning |
|---|---|
| `ok` | the receiver accepted a packet (AIM / SAFE_NO_TARGET / …) less than 2 s ago and HMAC ok |
| `last_t`, `age_s` | the last accepted packet's UTC and age |
| `target` | while the receiver aims: `{color, pan_deg, tilt_deg, x_right_m, y_forward_m, z_up_m, window_id}` (`window_id` from the aim topic); else `null` |
| `window_id`, `hmac_ok`, `n_rx`, `n_bad`, `status`, `sequence`, `key` | as v0.1.57 (`status` = OK while aiming, else the sender's word; `key` real / test) |
| `event`, `reason` | the receiver's last event (`AIM`, `DRY_RUN_READY_10S`, `PUMP_ON_GREEN`, `SAFE_NO_TARGET`, `DROP_INVALID_GREEN_PUMP_OFF`, `WATCHDOG_CENTER_PUMP_OFF` …) and a drop's reason (`BAD_HMAC`, `WRONG_COLOR`, `RANGE_OUT_OF_BOUNDS`, `STALE_PACKET` …) |
| `mode`, `pump`, `ready`, `stable_packets`, `pan_deg`, `tilt_deg` | `DRY_RUN` / `LIVE_SERVOS_PUMP_OFF` / `LIVE_PUMP_10S`; ON / OFF; a stable lock reached |

On the Pi: `python3 cannon_status_pub.py --log-dir <beside the receiver>/color_receiver_logs --mqtt <NX>:1884` (paho-mqtt ≥ 2; the Pi
needs a route to the boat broker — **to settle with the students on the AGX day**). Consumers: `optiprimex/tools/boat_web.py`
(`cannon()` / `cannon_link()`), `optiprimex/tools/ocs_web.py` (`CannonTap`).

## 6. Commands

```
cannon/run.sh --self-test                         HMAC, the v0.1.57 follower, the v0.1.61 gate (no network, no key)
cannon/run.sh --dry-run [--color red]             follow the aim and the target file, print, send nothing (no key)
cannon/run.sh                                     follow the aim on BOAT_HOST:BOAT_PORT + the target file, send (needs the key)
cannon/run.sh --synthetic-send                    one signed dock_color_aim.v1 packet with their synthetic target (receiver test)
buoys_multi/run.sh --headless --cannon red        the AGX observer also writes the cannon target (docking/run.sh: the same flag)
python3 pi/color_cannon_receiver.py --self-test   their receiver's own check (on the Pi; their config)
python -m optiprimex.sim.fake_cannon_pi --port 1884 --listen 127.0.0.1:5005 --key-out logs/x/cannon_test.key \
    --target-out logs/x/cannon_target.json --truth-world profiles/bench/world_pool.yaml --with-sender "--mqtt 127.0.0.1:1884"
```

## 7. Open — for the students / the AGX day

- Their `color_cannon_config.json` and `color_dock_config.json` (allowed colours → **red**, the servo limits, the mount offsets, the
  tracking and pump numbers, `allowed_sender_ip` = the AGX's address on the Pi's network, `maximum_packet_age_s` and the clocks).
- Who owns the height offset (§3), and that `allowed_colors` says `red` at the event.
- The Pi's route to the boat broker for `cannon_status_pub.py`; where it runs (a second terminal / a unit beside the receiver).
- The HSV band for the real lit window (`colour_hsv.red` in `cannon_config.json`) from a recorded clip (`--svo` replay, WP9b).
