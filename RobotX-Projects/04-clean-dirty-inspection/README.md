# Drone Clean and Dirty Surface Inspection

A YOLO-based prototype for classifying or detecting clean and dirty appearance from a drone camera. The original experiment used controlled paper/surface video, so deployment on real walls, roofs, panels, or structures requires new aerial images and independent validation.

## Detection cases

- Flag dirty regions while a drone inspects walls, roofs, panels, or structures.
- Check large or difficult-to-reach surfaces from the air.
- Display live inspection results from a drone camera feed or development webcam.
- Save annotated flight-inspection video for later review.

## Included approaches

- `live_vision.py`: live clean/dirty classification display.
- `prepare_dataset.py`, `train.py`, and `validate_model.py`: classification data preparation, training, and validation.
- `detect_dirt.py` and `display_dirt.py`: dirty-region detection with a minimum 0.90 confidence threshold.
- `best.pt`: small trained detection checkpoint retained for demonstration.

## Run the detector

```powershell
python detect_dirt.py PATH_TO_VIDEO.mp4 --confidence 0.90 --output annotated.mp4
python display_dirt.py --source 0
```

Press `Q` or `Esc` to close the live display.

## Limitation

The recorded metrics apply only to held-out frames from the original controlled source recordings. They do not prove performance on unseen walls, lighting, cameras, viewing angles, backgrounds, or dirt types. Collect and manually label representative wall footage before operational use.
