# RobotX dual-model perception package

This package implements the supplied ten-class master-label workflow.

Master class order:

| ID | Class |
|---:|---|
| 0 | `buoy_red` |
| 1 | `buoy_green` |
| 2 | `buoy_blue` |
| 3 | `buoy` |
| 4 | `dock` |
| 5 | `platform` |
| 6 | `obstacle` |
| 7 | `bay_red` |
| 8 | `bay_green` |
| 9 | `bay` |

`build_dual_datasets.py` creates:

- Nav: `buoy_red`, `buoy_green`, `buoy_blue`, `buoy`, `dock`, `platform`, `obstacle`.
- Dock: `bay_red`, `bay_green`, `bay`, `obstacle`.

The master dataset is never edited. Whole-dock boxes are dropped from the dock
dataset. Buoys, platform and master obstacles become dock-model `obstacle`.

```bash
python build_dual_datasets.py --master /path/to/master_dataset --output derived
python train_dual.py --derived derived
python evaluate_dual.py --derived derived
```

Training refuses to run when any master class has fewer than 150 boxes. TensorRT
engines must be generated on the target AGX Orin with `export_orin_fp16.py`.

