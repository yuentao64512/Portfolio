"""Train YOLO11s on the same revised dataset and settings as YOLO11n."""

from pathlib import Path
import json
from ultralytics import YOLO

root = Path('/home/robotx/box_size_training')
model = YOLO('yolo11s.pt')
results = model.train(
    data=str(root / 'dataset_v3_clean' / 'data.yaml'),
    project=str(root / 'runs'),
    name='yolo11s_dataset_v3_clean',
    exist_ok=False,
    imgsz=640,
    epochs=150,
    patience=30,
    batch=16,
    device=0,
    seed=42,
    deterministic=True,
    workers=4,
    plots=True,
    verbose=False,
)
run_dir = Path(results.save_dir)
print(json.dumps({'run_dir': str(run_dir),
                  'best_weights': str(run_dir / 'weights' / 'best.pt')}), flush=True)
