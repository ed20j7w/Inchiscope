"""Synthetic validation of the ROS-independent Stage 1/2 logic (no bag,
no rosbag2_py/rclpy needed -- see bag_extraction.py's module docstring for
why these are split out from read_bag()).

Critically, test_apply_hand_eye_matches_calibrate_hand_eye_convention
checks that this module composes the hand-eye transform the same way
inchiscope_camera/scripts/calibrate_hand_eye.py solves for it -- a mixed-
up convention here (e.g. applying the inverse, or composing in the wrong
order) would silently produce a badly wrong reconstruction despite every
other stage being correct, so this is the one test that most needs to
keep passing across any future refactor of either script.
"""

import numpy as np
import cv2
import pytest

from inchiscope_reconstruction.calibration import to_T
from inchiscope_reconstruction.bag_extraction import associate, apply_hand_eye
from inchiscope_reconstruction.frame_selection import blur_score, blur_scores, pose_delta, select_frames


def _random_rigid(rng):
    rvec = rng.uniform(-1, 1, 3)
    R, _ = cv2.Rodrigues(rvec)
    t = rng.uniform(-0.1, 0.1, 3)
    return R, t


def _rotmat_to_quat(R):
    tr = np.trace(R)
    S = np.sqrt(tr + 1.0) * 2
    w = 0.25 * S
    x = (R[2, 1] - R[1, 2]) / S
    y = (R[0, 2] - R[2, 0]) / S
    z = (R[1, 0] - R[0, 1]) / S
    return np.array([x, y, z, w])


def test_associate_picks_nearest_and_drops_stale():
    images = [(1.00, 'img0'), (1.05, 'img1'), (2.00, 'img2')]
    poses = [
        (1.00, np.array([0., 0., 0.]), np.array([0., 0., 0., 1.])),
        (1.04, np.array([1., 0., 0.]), np.array([0., 0., 0., 1.])),
        (1.10, np.array([2., 0., 0.]), np.array([0., 0., 0., 1.])),
    ]
    associated, dropped = associate(images, poses, max_pose_age_sec=0.1)
    assert dropped == 1  # img2 @ t=2.00 is >1.8s from any pose
    assert len(associated) == 2
    assert associated[0][2][0] == 0.0  # img0 @ t=1.00 -> nearest pose @ t=1.00
    assert associated[1][2][0] == 1.0  # img1 @ t=1.05 -> nearest pose @ t=1.04


def test_apply_hand_eye_matches_calibrate_hand_eye_convention():
    rng = np.random.default_rng(1)
    R_X, t_X = _random_rigid(rng)  # ground-truth cam2gripper
    R_bg, t_bg = _random_rigid(rng)  # a gripper pose in the reference frame
    q_bg = _rotmat_to_quat(R_bg)

    associated = [(0.0, 'img', t_bg, q_bg, 0.01)]
    (_, _, T_ref2cam, _), = apply_hand_eye(associated, R_X, t_X)

    # Same composition calibrate_hand_eye.py's synthetic test constructs
    # T_base2cam with: T_base2gripper @ T_cam2gripper_true.
    T_expected = to_T(R_bg, t_bg) @ to_T(R_X, t_X)
    assert np.allclose(T_ref2cam, T_expected, atol=1e-10)


def test_blur_score_ranks_sharp_above_blurred():
    sharp = np.zeros((100, 100), dtype=np.uint8)
    sharp[::4, :] = 255
    blurred = cv2.GaussianBlur(sharp, (15, 15), 5)
    assert blur_score(sharp) > blur_score(blurred)


def test_pose_delta_translation_and_rotation():
    T_a = np.eye(4)
    T_b = np.eye(4)
    T_b[:3, 3] = [0.01, 0, 0]
    dt, dtheta = pose_delta(T_a, T_b)
    assert dt == pytest.approx(0.01, abs=1e-9)
    assert dtheta == pytest.approx(0.0, abs=1e-9)

    R_rot, _ = cv2.Rodrigues(np.array([0, 0, np.radians(10)]))
    T_c = np.eye(4)
    T_c[:3, :3] = R_rot
    dt2, dtheta2 = pose_delta(T_a, T_c)
    assert dt2 == pytest.approx(0.0, abs=1e-9)
    assert dtheta2 == pytest.approx(10.0, abs=1e-6)


def _make_frame(t, tx, blurry=False):
    img = np.zeros((100, 100, 3), dtype=np.uint8)
    img[::4, :, :] = 255
    if blurry:
        img = cv2.GaussianBlur(img, (15, 15), 5)
    T = np.eye(4)
    T[0, 3] = tx
    return (t, img, T, 0.01)


def test_blur_scores_matches_blur_score_per_frame():
    sharp = np.zeros((100, 100, 3), dtype=np.uint8)
    sharp[::4, :, :] = 255
    blurred = cv2.GaussianBlur(sharp, (15, 15), 5)
    frames = [(0.0, sharp, np.eye(4), 0.0), (0.1, blurred, np.eye(4), 0.0)]
    scores = blur_scores(frames)
    assert scores[0] == pytest.approx(blur_score(cv2.cvtColor(sharp, cv2.COLOR_BGR2GRAY)))
    assert scores[1] == pytest.approx(blur_score(cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)))
    assert scores[0] > scores[1]


def test_select_frames_drops_blurry_and_redundant():
    frames = [
        _make_frame(0.0, 0.0),                # kept: first frame
        _make_frame(0.1, 0.0002),              # dropped: too close to frame 0
        _make_frame(0.2, 0.01, blurry=True),   # dropped: far enough but blurry
        _make_frame(0.3, 0.01),                # kept: far enough, sharp
    ]
    K = np.eye(3)
    D = np.zeros(5)
    kept, dropped_blur, dropped_redundant = select_frames(
        frames, K, D, blur_threshold=200.0, min_baseline_m=0.001, min_rotation_deg=2.0,
    )
    assert dropped_blur == 1
    assert dropped_redundant == 1
    assert len(kept) == 2


def test_select_frames_defaults_to_plumb_bob_undistort():
    """distortion_model defaults to 'plumb_bob' (cv2.calibrateCamera's
    model, from calibrate_camera.py) so every pre-existing caller that
    doesn't pass it keeps behaving exactly as before."""
    K = np.array([[80.0, 0, 50.0], [0, 80.0, 50.0], [0, 0, 1.0]])
    D = np.array([0.05, -0.01, 0.001, -0.0005, 0.0])
    frames = [_make_frame(0.0, 0.0)]
    kept, _, _ = select_frames(frames, K, D, blur_threshold=0.0)
    expected = cv2.undistort(frames[0][1], K, D)
    assert np.array_equal(kept[0][1], expected)


def test_select_frames_uses_fisheye_undistort_for_equidistant_model():
    """A camera_info.yaml written by calibrate_camera_fisheye.py
    (distortion_model: equidistant, 4 coefficients) must be undistorted
    with cv2.fisheye.undistortImage, not cv2.undistort -- the two models
    interpret D's coefficients completely differently, so using the wrong
    one doesn't just undistort slightly wrong, it actively distorts the
    image further, worst right at the frame edges a wide lens needs most."""
    K = np.array([[80.0, 0, 50.0], [0, 80.0, 50.0], [0, 0, 1.0]])
    D_fisheye = np.array([-0.05, 0.01, -0.002, 0.0005])  # k1,k2,k3,k4
    frames = [_make_frame(0.0, 0.0)]

    kept, _, _ = select_frames(
        frames, K, D_fisheye, blur_threshold=0.0, distortion_model='equidistant',
    )
    expected = cv2.fisheye.undistortImage(
        frames[0][1], K, D_fisheye, Knew=K, new_size=(100, 100),
    )
    assert np.array_equal(kept[0][1], expected)

    # And the plumb_bob path must NOT be what actually ran -- passing this
    # fisheye D into cv2.undistort would silently misinterpret it as a
    # 4-coefficient pinhole model (k1,k2,p1,p2) instead.
    wrong = cv2.undistort(frames[0][1], K, D_fisheye)
    assert not np.array_equal(kept[0][1], wrong)
