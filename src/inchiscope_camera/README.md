# inchiscope_camera

- `camera_node` -- captures the NanEye feed off the capture card (plain
  V4L2/UVC device, no vendor SDK needed) and publishes it on
  `/camera/image_raw`.
- `camera_viewer_node` -- subscribes to `/camera/image_raw` and shows it
  in a `cv2.imshow` window. Deliberately a separate node from `camera_node`
  (which never opens a GUI itself) so it can be skipped on a headless
  machine, or run against whatever is already publishing that topic.
  Press `q`/Esc or close the window to exit cleanly.

## Running it

```bash
ros2 launch inchiscope_bringup camera.launch.py                    # camera + viewer
ros2 launch inchiscope_bringup camera.launch.py show_viewer:=false # camera only, no GUI
ros2 launch inchiscope_bringup camera_and_aurora.launch.py         # camera + viewer + Aurora tracking + RViz
```

To record a synchronised bag of the camera feed and Aurora pose together
(e.g. once `camera_and_aurora.launch.py` is running), use
`inchiscope_bringup`'s `record.launch.py` -- see the top-level README's
Command reference. rosbag2 is the right tool for this: every recorded
topic keeps its own message timestamp in one bag file, so a `/camera/image_raw`
frame and an `/aurora/sensor_0/pose_relative_to_reference` sample can be
correlated by time during offline reconstruction without needing any
extra synchronisation machinery at record time.
