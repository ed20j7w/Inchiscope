# inchiscope_reconstruction

Offline 3D reconstruction from a recorded rosbag: fixed-pose dense stereo
(COLMAP) fused into a mesh (Open3D), using the Aurora EM tracker for
metric camera pose instead of estimating it from the images. See the
top-level project discussion for why this approach was chosen over
image-only SfM/SLAM (endoscopic mucosa is low-texture, specular, and
deformable -- all things vision-only pose estimation struggles with -- and
monocular SfM alone doesn't give real metric scale).

Scope: **reconstruction only**. This package consumes a rosbag and
produces a mesh + registered poses; it doesn't touch navigation, mode
control, or the rest of the control stack.

## Status

**Stage 1 (extract & associate) and Stage 2 (frame selection): implemented,
unit-tested against synthetic data, and verified against real hardware
data** (a real anchored-sweep bag, 1944 frames, 100% associated within
100ms of a pose).

**Stage 3 (sparse sanity check): implemented, unit-tested against
synthetic data.** Not yet run against real hardware data. The pure
projection/triangulation/epipolar-consistency math (`sanity_check.py`) is
covered by `test/test_sanity_check.py`, including a check that a
deliberately wrong pose gets caught by the epipolar-consistency filter --
this stage's actual job. SIFT matching itself can't be meaningfully
synthetic-tested (that's inherently about real image content), so the
first real-bag run is the real test of that part.

The ROS-dependent I/O (`bag_extraction.read_bag()`, needing rosbag2_py +
cv_bridge) still can't be exercised in the environment these were
developed in (no ROS2 install) -- everything else is pure Python/numpy/
OpenCV and unit-tested here.

**Not started**: Stages 4-5 (COLMAP fixed-pose dense stereo -> Open3D TSDF
fusion), Stages 6-7 (mesh cleanup, the `ReconstructFromBag` action
interface). COLMAP and Open3D aren't installed yet either -- not needed
until Stage 4.

## Fixes from the original workplan draft

Three things changed from the first workplan draft during review (see
project discussion):

1. **Pose topic.** Uses `/aurora/sensor_0/pose_relative_to_reference`, not
   the raw `/aurora/sensor_0/pose` the draft referenced -- the raw topic
   is in the field generator's own frame (not corrected to the fixed
   reference sensor) and isn't even recorded by
   `inchiscope_bringup/rosbag2/record_topics.yaml`.
2. **No duplicate intrinsics file.** `calibration.py` loads camera
   intrinsics from wherever `inchiscope_bringup`'s canonical
   `camera_info.yaml` lives (passed as a path), rather than keeping a
   second copy in this package's own config -- `inchiscope_camera` already
   hit a real bug from two `camera_info.yaml` files drifting out of sync.
3. **Hand-eye transform wiring.** `calibration.py` loads
   `hand_eye_transform.yaml` in the same format
   `inchiscope_camera/scripts/calibrate_hand_eye.py` writes -- the output
   of that calibration script is exactly the input this package needs.

## Running Stage 1-2 against a real bag

```bash
ros2 run inchiscope_reconstruction extract_and_select \
    --bag-path /path/to/your/bag \
    --camera-info src/inchiscope_bringup/config/camera_info.yaml \
    --hand-eye src/inchiscope_camera/scripts/hand_eye_transform.yaml \
    --out-dir stage2_frames/
```

Reports, at each stage: how many images/poses were read, how many images
got associated to a fresh-enough pose (and the pose-age distribution), the
**blur score distribution** across all extracted frames, and how many
survived frame selection (with counts for why the rest were dropped --
blurry vs. redundant). `--out-dir` writes the kept, undistorted frames as
PNGs so you can eyeball them.

**Tuning `--blur-threshold`**: variance-of-Laplacian (the metric used)
reads low-texture, smooth, evenly-lit content -- e.g. mucosa -- as
"blurry" even in perfect focus, so there's no universal good default
(100.0 was only ever a placeholder, not tuned against real footage). **On
a real capture this turned out to be a real effect, not a hypothetical
one**: all 1944 frames of a real bag scored 7.2-10.2, a real image from
that set visually showed clear, well-defined fold structure despite the
low score, and `--blur-threshold 0` (i.e. disabled) was what actually
worked -- the redundancy filter (`--min-baseline-m`/`--min-rotation-deg`,
pose-driven rather than image-content-driven) did the real work of
thinning 1944 frames down to a reasonable 189. Start with
`--blur-threshold 0` and only re-enable it if you have reason to believe
some frames really are unusably blurry (motion blur, defocus) -- check the
printed distribution and a few `--out-dir` frames near a candidate cutoff
before trusting it either way.

Other tunables: `--max-pose-age-sec` (default 0.1s), `--min-baseline-m`
(default 1mm), `--min-rotation-deg` (default 2.0). If fewer than 10
frames survive, the CLI warns -- check whether the thresholds are too
aggressive before trusting downstream stages with that few views.

## Running Stage 3 (sparse sanity check) against a real bag

```bash
ros2 run inchiscope_reconstruction sparse_sanity_check \
    --bag-path /path/to/your/bag \
    --camera-info src/inchiscope_bringup/config/camera_info.yaml \
    --hand-eye src/inchiscope_camera/scripts/hand_eye_transform.yaml \
    --expected-diameter-mm 25.0
```

Re-runs Stage 1-2 internally (cheap, no need to persist their output),
then: SIFT-matches frames `--pair-stride` apart (default 5 -- wider
baseline than adjacent Stage-2-kept frames, which are already near the
minimum useful separation), filters matches by whether they satisfy the
epipolar geometry implied by the **known** poses (not estimated -- a bad
hand-eye transform or corrupted pose stream will fail this even for
genuinely correct SIFT matches), triangulates the survivors, and checks
the resulting point cloud's scale against `--expected-diameter-mm` as a
coarse order-of-magnitude plausibility bound (`--tolerance-factor`,
default 5x either direction -- this is meant to catch a badly wrong scale,
not to validate the true lumen diameter precisely).

Reports the **actual pose baseline/rotation** between paired frames (not
just `--pair-stride` -- Stage 2's redundancy filter only guarantees a
*minimum* gap between consecutive kept frames, so real motion over
`--pair-stride` frames can still be small if the tip moved slowly for a
stretch) and a **per-stage funnel** summed across all attempted pairs --
SIFT keypoints found, matches surviving the ratio test, matches surviving
the epipolar check, points surviving cheirality -- specifically so a
failure can be pinned to a stage instead of just "nothing survived". If
`point_count` ends up below `--min-points-for-plausibility` (default 20 --
a handful of points can't give a meaningful scale estimate even if
individually well-conditioned), it prints a specific diagnosis based on
where the funnel actually died: keypoints too sparse (SIFT finding almost
nothing -- a texture problem, independent of poses), matches too sparse
after the ratio test (repetitive texture or `--pair-stride` too wide),
matches dying at the epipolar/cheirality checks with a **small actual
baseline** (near-degenerate triangulation geometry -- try a larger
`--pair-stride` before suspecting calibration) vs. a **reasonable
baseline** (the real signature of a bad hand-eye transform or corrupted
pose stream -- the *known* geometry the matches are checked against is
wrong), or dying at cheirality alone with good epipolar survival (a sign/
direction bug in the pose/projection convention, not a data problem).
Otherwise reports reprojection error (should be a few px at most against
known poses -- a warning fires above 5px median), then **drops points
whose reprojection error into either view exceeds `--max-point-reproj-
error-px`** (default 2.0) before computing scale plausibility. A point
that reprojects badly is either a mismatch that satisfied the epipolar
check by coincidence (that check only constrains a point to lie near a
*line*, which repetitive texture can satisfy without being the correct
correspondence) or a numerically unstable near-degenerate local
triangulation (a specific pair can have a small effective baseline even
when the overall pair-baseline distribution looks healthy) -- either way,
a handful of these can skew even a percentile-based extent estimate.
Reports how many points were kept/dropped, then the scale plausibility
verdict computed from the filtered set.

**`characteristic_size_mm` is deliberately crude** -- it's the median of
the 5th-95th percentile spread along each of the reference frame's own
X/Y/Z axes (not a true geometric diameter, not aligned to the scene's
actual shape), computed over ALL points pooled across every pair. Pooling
means it can't distinguish "each pair's own local reconstruction is
correctly scaled, but different pairs don't agree on where their patch
sits in the shared reference frame" (pair-to-pair placement inconsistency,
e.g. from a residual hand-eye rotation error) from "every pair is
individually mis-scaled" -- both look identical in the pooled number.

`--export-json PATH` writes the (outlier-filtered) triangulated points
grouped and labelled by which pair produced them, plus the full camera
trajectory, to a JSON file for visual inspection -- colouring points by
`pair_index` when plotting directly answers which of the two cases above
is happening: offset/separate clusters per pair points at placement
inconsistency; one uniformly-too-large blob points at a real scale
problem.

**On a real bag, SIFT found only ~13 keypoints/image** (vs. ~6000 on a
synthetic textured-test image) -- confirmed as a real local-contrast
problem, not a bug: `--clahe` applies contrast-limited adaptive histogram
equalization before SIFT detection, which recovers keypoints from real
edge structure that's genuinely present but compressed into a narrow
intensity band (validated on synthetic low-contrast-but-real-structure
content in `test/test_sanity_check.py`) -- it cannot invent structure
from truly flat/noise-floor content, so it's a real fix for faint-but-real
detail, not a workaround for footage with nothing in it. Try
`--clahe` first if the funnel shows few keypoints; `--clahe-clip-limit`
(default 4.0) and `--clahe-tile-size` (default 8) are tunable if the
defaults don't help on your footage. If keypoints stay sparse even with
`--clahe`, sparse feature matching may simply not be viable on this
tissue -- Stage 4-5's dense stereo doesn't have the same dependency on
distinctive sparse keypoints, so that isn't necessarily a blocker for the
overall pipeline, just for this particular cheap sanity check. A
FAIL here means don't proceed to the expensive COLMAP/Open3D stages yet --
suspect the hand-eye transform, a
corrupted pose stream, or (per project discussion) camera/Aurora
timestamp mismatch during fast motion corrupting the frame/pose pairing
for images captured while the tip was moving.

## Running the tests

```bash
cd src/inchiscope_reconstruction
python3 -m pytest test/
```

No ROS install needed for these -- they exercise the pure-logic functions
directly with synthetic data.
