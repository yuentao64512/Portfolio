# CV training scripts from the RTX 5060

This folder centralizes the computer-vision training and dataset-preparation code developed in the main CV workspaces on the RTX 5060. The source workspaces remain unchanged; these are GitHub-safe copies of the scripts and text configuration needed to understand, reproduce, or adapt the training pipelines.

## Contents

| Folder | Original 5060 workspace | Purpose |
| --- | --- | --- |
| `robotx-buoy-number` | `/home/robotx/Robotx_train` | RobotX buoy, marker, boundary and number-detector dataset preparation and training |
| `robotx-buoy-pipelines` | `/home/robotx/buoy_actual` | Multi-version RobotX buoy/dock training, autolabelling, curation and Orin export pipelines |
| `buoy-yolo26` | `/home/robotx/buoy_yolo26` | YOLO26 buoy frame preparation, dataset building, training and evaluation |
| `box-size-detection` | `/home/robotx/box_size_training` | Box detector dataset creation, autolabelling, training, evaluation and ZED geometry experiments |
| `hospital-fall-detection` | `/home/robotx/fall_action_project` | Pose-sequence preparation, action-classifier training and hospital fall-context pipeline |
| `hiker-fall-detection` | `/home/robotx/rse4207_ws` | Hiker/action data preparation, pose extraction, action training and inference |
| `pipe-cone-detection` | `/home/robotx/pipe_cones` | Pipe/cone dataset preparation, review, validation and YOLO26 training |
| `air-purifier-segmentation` | `/home/robotx/air_purifier_yolo` | Image collection, annotation, negative mining and YOLO11 segmentation training |
| `misc-5060-training` | Several small 5060 locations | Root-level cone, generic YOLO and Isaac Sim image-training helpers |
| `robotx-actual` | AGX Orin `/home/optiprime-agxorin/Desktop/dm3/MASTER_SCRIPTS/cv` | Actual RobotX buoy, docking, cannon, publishing and camera-streaming deployment source |

## How to use the scripts

1. Open the README inside the relevant subfolder first.
2. Create a fresh Python virtual environment on the training machine.
3. Install the dependencies stated by that project. Most YOLO pipelines require Ultralytics, PyTorch, OpenCV and NumPy.
4. Copy or recreate the required dataset locally.
5. Update absolute paths in the dataset YAML and launcher scripts for the new machine.
6. Run the dataset validation/preparation script before starting training.
7. Launch the project-specific `train*.py` or `train*.sh` script.
8. Review validation metrics and predictions before selecting a checkpoint for deployment.

These scripts preserve their original 5060 paths so the development history remains inspectable. Paths are configuration values, not portable defaults.

## Files intentionally excluded

Raw and labelled datasets, private recordings, trained weights, TensorRT/ONNX exports, virtual environments, generated training runs, caches, logs and prediction media are not duplicated here. They remain in their original 5060 workspaces or in their dedicated portfolio project folders.

This code represents research and development pipelines. Saved metrics and synthetic or recorded-video tests must not be presented as proof of live deployment performance without a separate real-world validation.
