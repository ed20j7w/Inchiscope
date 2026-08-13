# inchiscope_camera

- `camera_node` -- captures the NanEye feed off the capture card (plain
  V4L2/UVC device, no vendor SDK needed) and publishes it on
  `/camera/image_raw`. Defaults to `pixel_format: MJPG`, `width: 640`,
  `height: 480`.

  **The sensor's real content is only ~320x320.** This capture card's ISP
  pads that with a black border (plus a logo/info overlay in part of that
  border) out to whichever fixed resolution is requested, then scales the
  whole padded frame -- border included -- up to that size. `640x480` is
  the *smallest* size this device offers at all (confirmed via
  `v4l2-ctl -d /dev/video0 --list-formats-ext`: both MJPG and YUYV only
  advertise a fixed generic "webcam" list from `640x480` up to `1920x1080`,
  with several 4:3/5:4 sizes in between -- none of it tailored to the
  actual sensor). Anything bigger than `640x480` is pure waste: more pixels
  for every downstream CV step, zero extra real detail. MJPG hits 60fps at
  every size this device offers, so there's no framerate reason to prefer
  a bigger one either. If you're on different capture card hardware, re-run
  that `v4l2-ctl` command and adjust `pixel_format`/`width`/`height` in
  `params.yaml` to match what it actually reports -- don't assume these
  same numbers apply. Setting `width`/`height` to `0` skips overriding them
  and uses the driver's default for whatever `pixel_format` is set, as an
  escape hatch.

  **`crop_x`/`crop_y`/`crop_width`/`crop_height`** cut the black
  border/logo out of the published image, since leaving it in wastes
  compute on non-image pixels through every downstream step the same way
  the oversized resolution did. Disabled (all `0`) by default -- part of
  the border isn't pure black (the logo/info overlay), which rules out a
  simple auto-detect-the-black-border approach, so these are set manually:
  1. Run `camera_node` at whatever resolution you intend to use (the
     `640x480` default), then save one frame -- e.g.
     `ros2 run inchiscope_camera camera_viewer_node`, screenshot it, or add
     a one-off `cv2.imwrite()` -- and open it in any image editor that
     shows pixel coordinates on hover.
  2. Find the top-left and bottom-right corners of the real image content,
     avoiding the border and the logo/info overlay.
  3. Set `crop_x`/`crop_y` to that top-left corner and
     `crop_width`/`crop_height` to the content's size, in `params.yaml`.
  4. Relaunch and check `/camera/image_raw` now shows only real content --
     `camera_node` logs the crop it applied (or an error if the configured
     crop doesn't fit inside the actual capture resolution) on startup.

  This crop is specific to whatever resolution you're capturing at --
  changing `width`/`height` almost certainly changes the border layout, so
  re-measure the crop if you change resolution.
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

## Camera calibration

`/camera/camera_info` isn't published yet (see the TODO in
`camera_node.py`) -- `scripts/` has the standalone tools (checkerboard
target generator + an OpenCV `calibrateCamera` capture/solve script) to
determine the intrinsics that TODO needs. See `scripts/README.md`.

Calibrate against whatever `camera_node` actually publishes in production,
**after** setting up the crop above -- not the raw `640x480` padded frame.
The sensor's real content is confirmed ~320x320 (see the crop notes
above); calibrating on the uncropped frame would fit intrinsics to an image
that includes the black border/logo, which don't move the way real scene
content does under the pinhole model calibration assumes.
