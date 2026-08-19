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

**`--frames-dir` accepts more than one directory** -- combine this
dedicated intrinsics session with hand-eye capture sessions too (as long
as they all used the same physical square size and the same crop/
resolution), since those already needed a real spread of poses/distances
for their own reasons and that diversity constrains intrinsics better
than a narrower dedicated session alone:

```bash
python3 calibrate_camera.py calibrate \
    --frames-dir calib_frames/ handeye_frames/ handeye_frames1/ handeye_frames2/ \
    --square-size-mm <same size> --out camera_info.yaml
```

Confirmed on real capture data to matter, not just in theory: combining a
27-frame dedicated session with 3 hand-eye sessions (97 frames total)
dropped RMS reprojection error from 0.55px to 0.50px, and -- more
importantly -- meaningfully tightened `calibrate_hand_eye.py`'s own
checkerboard-in-reference validation spread on one of those hand-eye
sessions (8.5mm -> 5.6mm, a bigger improvement than switching distortion
models gave). It did *not* improve a second hand-eye session at all,
which was itself a useful negative result -- see that script's section
below for what that split outcome means.

Validated against a synthetic pinhole-camera test (not real hardware, since
none was available here): given known ground-truth intrinsics, the script
recovers focal length and principal point to within ~0.5% at 0.11px
reprojection error, so the calibration math itself is sound -- accuracy on
your actual camera still depends on capture quality (sharp focus, real
tilt variety, correct square size).

### If your lens is wide-angle/fisheye (>=~90-100deg field of view)

**Use `calibrate_camera_fisheye.py` instead of `calibrate_camera.py` for
step 3.** The NanEye ships in variants from 90deg up to 160deg field of
view -- a 120deg unit is a real, named ams-OSRAM part
(`NEC_B&W_SGA_FOV120_F4.0`) -- and the plain pinhole model `calibrate_camera.py`
uses is only a good approximation up to roughly 90-100deg; past that its
residual error grows fastest right at the frame edges. Recalibrating with
`calibrate_camera.py` again at a different distance or with more frames
does **not** fix this, since it just refits the same wrong model shape
again -- it needs the different model in `calibrate_camera_fisheye.py`
(OpenCV's fisheye/equidistant model, `cv2.fisheye.calibrate`), not more/
better captures with the pinhole one.

Same two subcommands, same CLI shape (`capture` is the literal same code,
imported, since capturing frames doesn't depend on the distortion model
you'll fit them with):

```bash
python3 calibrate_camera_fisheye.py capture --device /dev/video0 --width 1280 --height 720 \
    --crop-x 391 --crop-y 111 --crop-width 480 --crop-height 480 \
    --square-size-mm <the size you printed> --out-dir calib_frames_fisheye/
python3 calibrate_camera_fisheye.py calibrate --frames-dir calib_frames_fisheye/ \
    --square-size-mm <same size> --out camera_info_fisheye.yaml
```

One capture-technique difference from step 2 above: deliberately include
views where the board reaches toward the frame's edges/corners, not just
centred ones -- the fisheye model's k3/k4 terms are only well constrained
by corners sampled out at the extreme radii where the distortion is
largest. (This is the opposite of hand-eye capture below, where a
comfortably-centred board is preferred once intrinsics are already known
-- two different steps, two different sampling needs.)

Output is the same `camera_info.yaml` layout, just with `distortion_model:
equidistant` and 4 coefficients (k1-k4) instead of `plumb_bob`'s 5 --
`camera_node.py` copies that field through as-is, and `calibrate_hand_eye.py`
and `inchiscope_reconstruction` both branch on it automatically, so nothing
downstream needs a separate flag to know which model is in play. Also
handles OpenCV's `CALIB_CHECK_COND` failure mode (a single ill-conditioned
view raises rather than just being down-weighted) by identifying and
dropping the specific offending view and retrying, rather than failing the
whole calibration outright.

Validated against a real OpenCV 4.9.0 build (this repo's dev-sandbox
`cv2` has a build-specific bug in `cv2.fisheye.calibrate` itself --
confirmed by hand, not a real-world blocker, same situation
`calibrate_hand_eye.py`'s own note above already flags for
`cv2.calibrateHandEye`): recovers known synthetic ground-truth K/D to
~1e-4, and the ill-conditioned-view auto-drop correctly identifies and
removes a deliberately-bad synthetic view before recovering the correct
calibration from the rest.

## 4. Hand-eye calibration (Aurora sensor <-> camera)

The camera and the Aurora 6D EM sensor are mounted together at the
endoscope's distal tip, but the rigid offset between them has never been
measured -- `camera_and_aurora.launch.py`/`inchiscope.launch.py` currently
publish a zero-offset **PLACEHOLDER** `aurora_sensor_0 -> naneye_camera`
transform (see those files' docstrings), which treats the two as
co-located even though they're not. `calibrate_hand_eye.py` solves for the
real transform via standard `AX=XB` hand-eye calibration
(`cv2.calibrateHandEye`, Tsai-Lenz by default).

Requires `camera_and_aurora.launch.py` running (needs both
`/camera/image_raw` and `/aurora/sensor_0/pose_relative_to_reference`
publishing at once) and the intrinsic calibration above already done (the
solve step needs `camera_info.yaml` for `solvePnP`). Works with either
`camera_info.yaml`/`camera_info_fisheye.yaml` unchanged -- `solve` reads
`distortion_model` and automatically undistorts the detected corners with
the matching model before `solvePnP` (a 4-coefficient fisheye D handed
straight to `cv2.solvePnP` would otherwise be silently misread as a
4-coefficient pinhole model instead, corrupting every pose with no error
-- confirmed by hand to produce a ~0.15 rad rotation error on synthetic
data with no warning).

```bash
# 1. Capture: hold a checkerboard FIXED and stationary somewhere in view,
#    then move the endoscope tip by hand through 15-20+ diverse poses --
#    a real range of rotation, not just translation/sliding -- capturing
#    an (image, Aurora pose) pair at each with SPACE:
python3 calibrate_hand_eye.py capture --square-size-mm <the size you printed> --out-dir handeye_frames/

# 2. Solve:
python3 calibrate_hand_eye.py solve --frames-dir handeye_frames/ \
    --square-size-mm <same size> \
    --camera-info ../../inchiscope_bringup/config/camera_info.yaml \
    --out hand_eye_transform.yaml
```

**`--frames-dir` accepts more than one capture session**
(`--frames-dir handeye_frames0/ handeye_frames1/ handeye_frames2/`), each
with its own `poses.yaml`, pooling every session's captures into one
solve. **Confirmed on real data this does not reliably help, and can hurt**:
combining 3 real sessions (69 captures total) gave a worse result (9.8mm
spread, PARK) than the best individual session alone (6.7mm) -- the
checkerboard-in-reference validation implicitly assumes the board sat in
the exact same physical spot for every pooled capture, which is a
reasonable assumption *within* one session (you set it down once) but not
necessarily *across* separate sessions unless you were careful never to
move it between them. This is a different situation from combining
frames for intrinsics calibration above, which has no such fixed-target
assumption and reliably helps. Only combine hand-eye sessions if you're
confident the board never moved between them; otherwise solve each
session separately and use whichever gives the tightest spread.

`solve` cross-checks all 5 methods OpenCV supports (TSAI/PARK/HORAUD/
ANDREFF/DANIILIDIS) for rough agreement, then validates the chosen one by
checking how consistent the *inferred* checkerboard-in-reference-frame
pose is across all captures -- since the board never actually moved during
the session, that inferred pose should come out (near-)identical every
time if the solve is correct. **TSAI (the default `--method`) is not
always the most numerically robust of the 5** -- on the combined-session
data above it came out badly wrong (32.9mm) while PARK/HORAUD/DANIILIDIS
agreed tightly with each other (~7.7-9.8mm); always check the 5-method
cross-check printout for agreement before trusting whichever `--method`
you asked for, and switch `--method` if the default disagrees with the
pack. A large spread (warns above 2mm on any axis
by default) usually means the board moved, too few/too rotation-poor
poses were captured, or a pose/frame got mismatched.

If the warning fires, it also runs a **leave-one-out diagnostic**:
re-solves once per capture with that capture excluded, to tell you whether
a single bad capture is responsible (dropping it recovers most of the
error -- remove it and re-run `solve`) or the error is spread across most
captures (recapture, or see the capture-card latency note below).

### Capture-card latency

If a USB capture card sits between the NanEye and `/camera/image_raw`, it
can buffer frames and make the topic lag the physical scene by a
noticeable amount (100ms+, sometimes much more). Since `capture`'s
timestamp-freshness check only compares the image and pose *message*
timestamps to each other, it cannot detect this -- both can look "fresh"
relative to each other while the image content itself is stale relative
to where the rig actually is *right now*. Symptom: `solve`'s spread stays
too high (mm-level) even when captured carefully, and the leave-one-out
diagnostic finds no single bad frame (a few mm improvement across the
board at best) -- i.e. it's systemic, not a bad capture or two.

`capture` defaults to **auto-capture** specifically to make this a
non-issue: rather than you judging how long to hold still (which depends
on a latency you can't see or measure), it requires BOTH the checkerboard
corners in the live video AND the raw Aurora pose to have stayed put,
continuously, for `--stability-window-sec` (default 1.0s) before firing.
Any one of the two drifting past its threshold resets the whole streak, so
"stable" always means both signals held still *at the same time*. Because
the video check runs on the *lagged* feed itself, it can't fire early --
the feed only starts looking stable once it has caught up to a scene that
has actually stopped moving, whatever the real delay turns out to be. The
Aurora check catches a second, independent problem: the tracker itself
being too noisy to trust at the current spot (nearby metal, distance from
the field generator) -- if it never settles even with a genuinely still
hand, that's diagnostic on its own (see below).

Practically: move to a pose, hold still, wait for the status line to say
`STABLE` (it captures the instant it does), then move on. Tunables:
`--stability-max-corner-px` (default 2.0px, video), `--stability-max-
aurora-pos-mm` (default 0.5mm) and `--stability-max-aurora-rot-deg`
(default 0.3deg) for Aurora. Loosen the corner one if hand tremor never
settles under it; do *not* casually loosen the Aurora ones just to make it
capture faster -- see the noise-floor note below first. SPACE still
force-captures manually on top of this; `--manual` disables auto-capture
entirely and reverts to SPACE-only.

If the status line's Aurora jitter never drops below its threshold even
though your hand is genuinely still, that's telling you something real:
the Aurora reading is noisy or biased at this physical location, most
likely from nearby metal or distance/orientation relative to the field
generator. No amount of capture technique fixes that -- move the whole
setup to a cleaner spot (away from the capture card, laptop, metal desk,
etc., and closer to the field generator) and see if the jitter number
drops before trying to calibrate again.

On success `solve` writes `hand_eye_transform.yaml` and prints the exact
`static_transform_publisher` arguments (as `--x/--y/--z` +
`--qx/--qy/--qz/--qw`, more precise than roll/pitch/yaw for an arbitrary
rotation) to paste into `camera_and_aurora.launch.py`'s and
`inchiscope.launch.py`'s placeholder transform, replacing the zero-offset
one.

Validated against a synthetic hand-eye test (known ground-truth transform,
15 synthetic poses spanning real rotation): all 5 methods recover the
ground truth to machine precision, and the checkerboard-pose-spread
validation metric correctly stays near-zero for the correct transform and
blows up for a deliberately wrong one -- so the math and validation logic
are sound; accuracy on the real rig still depends on capture quality
(board truly fixed, real rotation variety, correct square size).

## 5. What the intrinsics feed into next

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
