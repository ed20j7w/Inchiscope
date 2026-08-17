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

**Implemented and unit-tested against synthetic data**: Stage 1 (extract
& associate) and Stage 2 (frame selection).

**Not yet run against real hardware data** -- there's no ROS2 install in
the environment these were developed in, so `bag_extraction.read_bag()`
(the only ROS-dependent piece -- rosbag2_py + cv_bridge) couldn't be
exercised directly. Everything else (`associate`, `apply_hand_eye` in
`bag_extraction.py`; `blur_score`, `pose_delta`, `select_frames` in
`frame_selection.py`) is pure Python/numpy/OpenCV and is covered by
`test/test_stage1_stage2.py`, including a check that the hand-eye
transform is composed the same way `calibrate_hand_eye.py` solves for it
(the one place a subtle convention mismatch would silently corrupt every
downstream stage without necessarily looking wrong at a glance).

**Not started**: Stage 3 (sparse sanity check), Stages 4-5 (COLMAP fixed-
pose dense stereo -> Open3D TSDF fusion), Stages 6-7 (mesh cleanup, the
`ReconstructFromBag` action interface). COLMAP and Open3D aren't installed
yet either -- not needed until Stage 4.

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
(100.0 was only ever a placeholder, not tuned against real footage). The
CLI always prints the real min/p10/p25/median/p75/p90/max distribution
for your actual frames before applying the cutoff -- if every frame
scores below the current threshold, it says so explicitly rather than
letting Stage 2 silently drop everything. Pick a threshold relative to
*that* distribution (e.g. drop only the bottom 10-25% via the p10/p25
values) rather than an absolute number, then check a few `--out-dir`
frames near the cutoff actually look unusably blurry before trusting it.

Other tunables: `--max-pose-age-sec` (default 0.1s), `--min-baseline-m`
(default 1mm), `--min-rotation-deg` (default 2.0). If fewer than 10
frames survive, the CLI warns -- check whether the thresholds are too
aggressive before trusting downstream stages with that few views.

## Running the tests

```bash
cd src/inchiscope_reconstruction
python3 -m pytest test/
```

No ROS install needed for these -- they exercise the pure-logic functions
directly with synthetic data.
