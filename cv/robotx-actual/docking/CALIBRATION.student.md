# Dock mapper calibration

Edit /home/jetson/docking_cv/dock_mapper_config.json.

Coordinate convention:
- x_right_m is positive to starboard/right and negative to port/left.
- y_forward_m is positive forward.
- A target 15 m left of the cannon is therefore approximately x_right_m = -15.

Offsets:
- camera.yaw_deg: camera yaw relative to the boat centreline; positive turns right.
- camera.forward_from_fcu_m: camera position forward of the FCU/GPS reference.
- camera.right_from_fcu_m: camera position right of the FCU/GPS reference.
- water_cannon.forward_from_camera_m: cannon pivot position forward of the camera.
- water_cannon.right_from_camera_m: cannon pivot position right of the camera.

Measure all offsets in metres. Negative values mean aft/left respectively.
The FCU offsets affect latitude/longitude projection. The camera-to-cannon offsets
affect the x/y sent to the Raspberry Pi for aiming.
