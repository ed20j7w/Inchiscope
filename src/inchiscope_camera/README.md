# inchiscope_camera

- `camera_node` -- captures the NanEye feed off the capture card (plain
  V4L2/UVC device, no vendor SDK needed) and publishes it on
  `/camera/image_raw`. Defaults to `pixel_format: MJPG`, `width: 1920`,
  `height: 1080` -- without an explicit `pixel_format`, V4L2 falls back to
  YUYV, which on this capture card both defaults to a 4:3 800x600 mode and
  caps out well below 30fps at any 16:9 size (confirmed via
  `v4l2-ctl -d /dev/video0 --list-formats-ext`; MJPG supports both 1280x720
  and 1920x1080 at up to 60fps on the same device, so there's no framerate
  cost to the bigger one). If you're on different capture card hardware,
  re-run that `v4l2-ctl` command and adjust `pixel_format`/`width`/`height`
  in `params.yaml` to match what it actually reports -- don't assume these
  same numbers apply. Setting `width`/`height` to `0` skips overriding them
  and uses the driver's default for whatever `pixel_format` is set, as an
  escape hatch.
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
