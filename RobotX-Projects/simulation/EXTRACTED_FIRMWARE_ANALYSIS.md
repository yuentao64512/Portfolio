# Extracted Pixhawk6X firmware analysis

Extracted read-only over MAVLink FTP from `@SYS/flash.bin` on 2026-09-16.

## Image

- Full image: `extracted_pixhawk6x_flash_20260916_full.bin`
- Size: 2,097,152 bytes
- SHA-256: `7b01fbabfca81114ade702cf2d7df3fc804000f0d7f7dae4c02b63f3d3c951f1`
- Embedded version: `ArduRover V4.8.0-dev (e21b732a)`
- Board string: `Pixhawk6X`

The shorter `extracted_pixhawk6x_flash_20260916.bin` is an incomplete
100,000-byte first read caused by the virtual file reporting a placeholder
length. Use the `_full.bin` image for analysis.

## Custom modes

### Mode 20: OmniCustomDirect (`OCDR`)

- Mode-number routine returns `20` at flash address `0x08028e04`.
- Full and short names are stored at `0x08157a9c` and `0x08157ab0`.
- Entry clears the four-value motor vector.
- New direct motor vectors are accepted only while this mode is active.
- A freshness flag and millisecond timestamp are maintained.
- If a command is older than 250 ms, or the vehicle is not armed, the mode
  writes four zero motor values. This is a command-loss watchdog.

### Mode 17: OmniCustomGuided (`OCGD`)

- Mode-number routine returns `17` at flash address `0x08028f20`.
- Full and short names are stored at `0x08157b2c` and `0x08157b40`.
- The update routine obtains forward/throttle, lateral, and steering inputs,
  constrains them, and computes four motor values.
- The recovered mixer is approximately:

  ```text
  motor_1 = forward
  motor_2 = lateral - 0.25 * steering
  motor_3 = -forward
  motor_4 = lateral + steering
  ```

- If the largest absolute motor value exceeds 1.0, all four values are divided
  by that magnitude, preserving direction while limiting output to `[-1, 1]`.
- Entering the mode initializes the four motor values to zero.

### Motor-output prerequisite

The custom modes write their four-value vector into the Rover motor backend.
The backend only publishes that vector when `FRAME_TYPE` is nonzero, and maps
the four entries to motor functions 33, 34, 35, and 36. For this four-thruster
layout the validated configuration is `FRAME_CLASS=2`, `FRAME_TYPE=3`
(`OmniPlus`), and `SERVO1_FUNCTION` through `SERVO4_FUNCTION` set to 33–36.

The captured bench configuration instead had `FRAME_TYPE=0` and functions
73, 34, 74, 36. In that state mode 17 computes its vector internally but the
motor backend does not expose it on `SERVO_OUTPUT_RAW`; observed outputs were
1500, 0, 1500, 0 even while armed.

## Important limitation

This is stripped ARM Thumb machine code, not recovered source. Function names
and comments are unavailable. The mode IDs, embedded names, watchdog, mixer,
normalization, and zero-on-entry behavior are directly supported by the binary;
higher-level design intent remains an inference unless the matching source for
commit/build `e21b732a` is obtained.
