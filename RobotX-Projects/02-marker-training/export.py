import fiftyone as fo
import fiftyone.types as fot

# Load your edited dataset
dataset = fo.load_dataset("buoy_train")

# Export back to YOLO format
dataset.export(
    export_dir="exported_dataset",
    dataset_type=fot.YOLOv5Dataset,
    label_field="ground_truth",   # usually this is correct
    export_media=True             # copies the images too
)

print("Dataset exported successfully!")