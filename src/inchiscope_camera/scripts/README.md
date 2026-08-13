# Camera calibration tools

Standalone scripts (not ROS nodes -- run directly with `python3`, not
`ros2 run`) for determining the NanEye/OV6948 capture-card camera's
intrinsics, needed before any of the offline reconstruction pipeline
(depth, SfM feature triangulation, meshing) can produce metrically correct
output. See `camera_node.py`'s `/camera/camera_info` TODO -- this is that
TODO's prerequisite.

Extra dependencies beyond what `inchiscope_camera` already needs:
`pip install matplotlib pyyaml` (matplotlib only needed to regenerate the
printable target; pyyaml only needed for `calibrate_camera.py calibrate`'s
output file).

## 1. Print a calibration target

```bash
python3 generate_calibration_target.py calibration_target.pdf
```

Produces a one-page PDF with the same checkerboard pattern (9x6 internal
corners) at five square sizes (1/2/3/5/8mm) plus a 50mm scale bar.

**Print at "Actual Size" / 100% scale -- not "fit to page".** Measure the
scale bar with a ruler afterwards; if it isn't exactly 50mm, your printer
or viewer rescaled the page and every measurement below will be wrong.

Cut out whichever square size actually fits the camera's field of view at
its in-focus working distance, with several full squares visible edge to
edge. As a starting estimate: `square_size_mm ≈ (2 * working_distance_mm *
tan(half_FOV_degrees)) / squares_across` -- but this camera's exact FOV/
working distance weren't available from a datasheet here, so treat the
five printed sizes as a testing range rather than a computed answer. Hold
each cutout at the distance where the live camera view is sharpest and see
which one fills a good fraction of the real content -- confirmed 480x480
once cropped (see `inchiscope_camera/README.md` and the current
`crop_x/y/width/height` in `inchiscope_bringup/config/params.yaml`) --
without corners falling outside it.

## 2. Capture calibration frames

```bash
python3 calibrate_camera.py capture --device /dev/video0 --width 1280 --height 720 \
    --crop-x 391 --crop-y 111 --crop-width 480 --crop-height 480 \
    --square-size-mm <the size you printed> --out-dir calib_frames/
```

The `--width`/`--height`/`--crop-*` values must match whatever
`camera_node` is actually configured with in `params.yaml` -- this script
opens the raw device directly, so without the same crop it would calibrate
against the padded pre-crop frame instead of what `/camera/image_raw`
actually publishes.

Live-views the camera (already cropped to the real 480x480 content); SPACE
grabs a frame when the board is detected (drawn in colour on the corners),
'q' finishes. Move and tilt the board between captures -- corners near
every edge of the frame, and a real range of tilt angles, not just flat-on
shots centred in frame. 15-25 good captures is typical; fewer than 10 will
print a warning.

## 3. Run calibration

```bash
python3 calibrate_camera.py calibrate --frames-dir calib_frames/ --square-size-mm <same size> --out camera_info.yaml
```

Reports how many frames were usable, the RMS reprojection error (>1px is
suspicious for this image size -- recheck square size and pose variety
before trusting the result), and the recovered camera matrix and
distortion coefficients. Writes `camera_info.yaml` in the same field
layout ROS's `sensor_msgs/CameraInfo` / `camera_info_manager` expects.

Validated against a synthetic pinhole-camera test (not real hardware, since
none was available here): given known ground-truth intrinsics, the script
recovers focal length and principal point to within ~0.5% at 0.11px
reprojection error, so the calibration math itself is sound -- accuracy on
your actual camera still depends on capture quality (sharp focus, real
tilt variety, correct square size).

## 4. What the intrinsics feed into next

See the top-level project discussion for the full reasoning, but briefly:
`camera_matrix` (K) turns a 2D pixel coordinate into a 3D ray, which is
what every step below needs.

- **SIFT features**: detect + describe keypoints per frame
  (`cv2.SIFT_create()`), match across frames (ratio-test + optionally
  epipolar/pose-consistency filtering using the *known* EM tracker pose at
  each frame rather than an estimated one).
- **Triangulation ("depth")**: because the EM tracker already gives a
  metric camera pose per frame (unlike classical SfM, which has to solve
  for pose from the images themselves), each matched SIFT correspondence
  triangulates directly via known projection matrices `P_i = K [R_i | t_i]`
  (`cv2.triangulatePoints` for a first estimate, refine by minimising
  reprojection error across all views the point was seen in).
- **Mesh generation**: once enough 3D points have accumulated into a point
  cloud, estimate normals and run Poisson or ball-pivoting surface
  reconstruction (e.g. Open3D's `create_from_point_cloud_poisson`) to get a
  mesh.

None of that is implemented yet (still Phase 5/6 per
`inchiscope_ros2_architecture.md`) -- this script set only gets you the one
prerequisite input (`K`, distortion) that all of it depends on.
