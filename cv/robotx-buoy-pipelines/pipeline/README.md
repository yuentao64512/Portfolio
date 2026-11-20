# Buoy two-model pipeline

This folder implements the supplied plan without changing the original media.

## Model A classes

0. `buoy_led_red`
1. `buoy_led_green`
2. `buoy_no_led`
3. `dock` — floating pontoon carrying white boxes
4. `home` — long walkway/pontoon
5. `obstacle` — small single block

Box only the object above the water; omit reflections. Letters on buoys are not
separate Model A classes.

## Model B classes

`circle`, `diamond`, and digits `0` through `9`. Each digit is a separate box.
Letters A/B are deliberately not trained, matching the supplied plan.

## Current automated stage

`setup_and_extract.sh` creates a private Python environment, installs a CUDA
12.8 PyTorch build and Ultralytics, verifies GPU FP16 execution, and extracts
two candidate frames per second while skipping near-static duplicates. Original
videos and photos remain untouched. The manifest preserves the source-video
name and a deterministic train/validation/test group.

Auto-generated object labels must be reviewed before training. A missed object
is harmful training data, so candidate crops and prelabels are kept separate
from the final dataset until checked.
