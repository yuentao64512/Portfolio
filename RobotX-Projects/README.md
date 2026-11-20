# RobotX Drone Vision Projects

A cleaned collection of RobotX aerial-drone computer-vision and radio-control experiments from the original YOLO workspace. The drone observes buoys, boats, coloured markers, people, and surface conditions from the air. Generated predictions, duplicate datasets, virtual environments, multi-gigabyte videos, and repeated training runs were deliberately left out so this folder can become a readable GitHub repository.

## Projects and detection cases

| Project | What it detects or does | Example application |
| --- | --- | --- |
| `01-buoy-boat-training` | Green buoys, orange/red buoys, and boats from aerial imagery | Drone-based maritime surveying, target recognition, and mission observation |
| `02-marker-training` | Blue, red, and green course markers from above | Training and testing an aerial marker detector from labelled drone images or video |
| `03-training-rejects` | Aerial buoy/marker detections while suppressing people and other false positives | Preventing clothing, people, shoreline objects, or reflections from being mistaken for targets |
| `04-clean-dirty-inspection` | Clean versus dirty surfaces viewed by a drone camera | Aerial wall, roof, panel, structure, or facility inspection |
| `05-drone-rc-control` | Converts drone-camera detections to a 100 x 100 grid and communicates over serial radio | Drone RC/mission experimentation and sending target positions to a controller |
| `simulation` | Gazebo boat and drone simulation with OCS/robocommand integration, ASV wave models, MAVLink control, worlds, and launch scripts | Testing the combined RobotX boat/drone control workflow before hardware deployment |
| `robotx-actual` | Actual USV Orin NX control source: UAV, USV, OCS, USV UI, shared scripts and robocommand | Reviewing and deploying the physical RobotX boat control stack |

Each project has its own README with its purpose, application cases, expected files, and commands.

## Quick start

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

Then enter one project folder and follow its README. Several original scripts still expose hardware paths, camera indexes, COM ports, and model paths near the top of the file; configure those values before running.

## Repository policy

- Datasets, videos, generated predictions, training caches, and large checkpoints are ignored by Git.
- The small clean/dirty checkpoint is included as a runnable example.
- Store larger YOLO checkpoints in GitHub Releases, cloud storage, or Git LFS and document the download location.
- Review dataset ownership and privacy before publishing any images or labels.

## Suggested GitHub setup

```powershell
git init
git add .
git commit -m "Organize RobotX projects"
git branch -M main
git remote add origin YOUR_GITHUB_REPOSITORY_URL
git push -u origin main
```

## Origin

This is an organized copy. The original `yolov11_train` and `/home/robotx/usv_sim_workspace` workspaces were not moved or deleted.
