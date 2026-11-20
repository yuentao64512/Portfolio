#!/usr/bin/env python3
"""Capture one rectified ZED left frame for measurement setup diagnostics."""

from pathlib import Path

import cv2
import numpy as np
import pyzed.sl as sl


output = Path("/home/robotx/box_size_training/outputs/current_zed_frame.png")
init = sl.InitParameters()
init.camera_resolution = sl.RESOLUTION.HD720
init.camera_fps = 30
init.depth_mode = sl.DEPTH_MODE.NEURAL
init.coordinate_units = sl.UNIT.METER
init.depth_minimum_distance = 0.10
init.depth_maximum_distance = 8.0
camera = sl.Camera()
status = camera.open(init)
if status != sl.ERROR_CODE.SUCCESS:
    raise SystemExit(f"Cannot open ZED: {status}")
try:
    if camera.grab(sl.RuntimeParameters()) != sl.ERROR_CODE.SUCCESS:
        raise SystemExit("Could not grab a ZED frame")
    image = sl.Mat()
    depth = sl.Mat()
    point_cloud = sl.Mat()
    camera.retrieve_image(image, sl.VIEW.LEFT)
    camera.retrieve_measure(depth, sl.MEASURE.DEPTH)
    camera.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
    frame = cv2.cvtColor(np.asarray(image.get_data()), cv2.COLOR_BGRA2BGR)
    depth_values = np.asarray(depth.get_data())
    xyz = np.asarray(point_cloud.get_data())
    roi_depth = depth_values[157:387, 649:1092].reshape(-1)
    roi_z = xyz[157:387, 649:1092, 2].reshape(-1)
    for name, values in (("depth", roi_depth), ("xyz_z", roi_z)):
        finite = values[np.isfinite(values)]
        print(
            name,
            "shape=", values.shape,
            "finite=", len(finite),
            "min=", float(np.min(finite)) if len(finite) else None,
            "median=", float(np.median(finite)) if len(finite) else None,
            "max=", float(np.max(finite)) if len(finite) else None,
        )
    output.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output), frame):
        raise SystemExit(f"Could not save {output}")
    print(f"saved={output} shape={frame.shape}")
finally:
    camera.close()
