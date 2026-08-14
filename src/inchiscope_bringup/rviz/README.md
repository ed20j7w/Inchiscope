# RViz configs go here

`aurora.launch.py` opens RViz pointed at `aurora.rviz` in this directory by
default (`rviz_config` launch argument). Current displays: `Grid`, `TF`,
a `Pose` display on `/aurora/sensor_0/pose_relative_to_reference`, an
`Image` display (`NanEye Camera`) and a `Camera` display (`Camera`), both
on `/camera/image_raw`.

If you rearrange/add displays in the RViz GUI and want to keep it, save
directly into
`install/inchiscope_bringup/share/inchiscope_bringup/rviz/aurora.rviz`
(works immediately with `--symlink-install`, no rebuild needed) and copy
it back into this source directory afterwards to keep it under version
control.

The two camera displays serve different purposes:

- **`Image`** -- a simple 2D panel showing `/camera/image_raw` as-is.
  Ignores `/camera/camera_info` entirely and needs no TF, so it always
  works as long as `camera_node` is publishing.
- **`Camera`** -- overlays the image as a 3D perspective view within the
  scene and *does* use `CameraInfo`. Requires the image's frame
  (`naneye_camera`) to exist in the TF tree, which needs the placeholder
  static transform published by `camera_and_aurora.launch.py`/
  `inchiscope.launch.py` (`aurora_sensor_0 -> naneye_camera`, currently
  **zero offset -- not the real measured camera/EM-sensor rigid offset**,
  see those launch files' docstrings). Only resolves correctly when
  launched alongside Aurora (e.g. `camera_and_aurora.launch.py`, which is
  what actually publishes that transform) -- running `camera.launch.py`
  alone with this same RViz config will show the `Camera` display's status
  as an error (`Frame [naneye_camera] does not exist`) while `Image` keeps
  working fine.
