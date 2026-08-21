import fiftyone as fo
import fiftyone.types as fot

dataset = fo.Dataset.from_dir(
    dataset_dir=r"C:\Users\OptiPrime-UAV\yolov11_train\water_test",
    dataset_type=fot.YOLOv5Dataset,
    yaml_path=r"C:\Users\OptiPrime-UAV\yolov11_train\water_test\data.yaml",
    split="train",
    name="buoy_train",
    overwrite=True,

)

print("Loaded samples:", len(dataset))

session = fo.launch_app(dataset, port=5152)
session.wait()