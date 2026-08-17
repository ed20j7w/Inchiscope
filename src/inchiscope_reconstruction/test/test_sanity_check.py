"""Synthetic validation of Stage 3's geometry (no bag, no real images
needed for the projection/triangulation/epipolar math -- SIFT matching
itself is inherently about real image content and can't be meaningfully
synthetic-tested, but everything downstream of "here are some matched
pixel coordinates" can be and is).

test_epipolar_filter_catches_bad_pose is the one that most directly
answers "would this stage actually catch a bad hand-eye calibration or
corrupted pose stream" -- its purpose per the reconstruction workplan.
"""

import numpy as np
import cv2
import pytest

from inchiscope_reconstruction.calibration import to_T
from inchiscope_reconstruction.sanity_check import (
    filter_by_epipolar_consistency, triangulate, reprojection_error, scale_plausibility,
    sparse_sanity_check,
)

K = np.array([[400., 0, 240], [0, 400., 240], [0, 0, 1]])


def _random_rigid(rng, max_rot=0.1, max_t=0.02):
    rvec = rng.uniform(-max_rot, max_rot, 3)
    R, _ = cv2.Rodrigues(rvec)
    t = rng.uniform(-max_t, max_t, 3)
    return R, t


def _project(pts3d, K, T):
    T_inv = np.linalg.inv(T)
    pts_h = np.hstack([pts3d, np.ones((len(pts3d), 1))])
    cam = (T_inv @ pts_h.T)[:3].T
    proj = (K @ cam.T).T
    return proj[:, :2] / proj[:, 2:3]


def _make_pair(rng, n=100):
    """A pair of camera poses with realistic small relative motion (like
    adjacent-ish frames from a slow sweep, not an arbitrary large jump),
    and n synthetic 3D points at a lumen-like ~25mm scale, in front of
    both."""
    T_a = to_T(np.eye(3), np.zeros(3))
    R_b, t_b = _random_rigid(rng, max_rot=0.1, max_t=0.005)
    T_b = to_T(R_b, t_b + np.array([0.005, 0.0, 0.0]))
    pts3d = rng.uniform(-0.0125, 0.0125, (n, 3)) + np.array([0, 0, 0.02])
    pts_a = _project(pts3d, K, T_a)
    pts_b = _project(pts3d, K, T_b)
    return T_a, T_b, pts3d, pts_a, pts_b


def test_triangulation_recovers_known_points():
    rng = np.random.default_rng(1)
    T_a, T_b, pts3d_true, pts_a, pts_b = _make_pair(rng)
    pts3d_est, valid = triangulate(pts_a, pts_b, K, T_a, T_b)
    assert valid.all()
    assert np.abs(pts3d_est - pts3d_true).max() < 1e-6


def test_reprojection_error_near_zero_for_correct_poses():
    rng = np.random.default_rng(2)
    T_a, T_b, pts3d_true, pts_a, pts_b = _make_pair(rng)
    pts3d_est, valid = triangulate(pts_a, pts_b, K, T_a, T_b)
    err_a = reprojection_error(pts3d_est, pts_a, K, T_a)
    err_b = reprojection_error(pts3d_est, pts_b, K, T_b)
    assert err_a.max() < 1e-6
    assert err_b.max() < 1e-6


def test_epipolar_filter_keeps_correct_correspondences():
    rng = np.random.default_rng(3)
    T_a, T_b, _, pts_a, pts_b = _make_pair(rng, n=200)
    kept_a, kept_b = filter_by_epipolar_consistency(pts_a, pts_b, K, T_a, T_b, max_epipolar_error_px=1.0)
    assert len(kept_a) == len(pts_a)


def test_epipolar_filter_rejects_shuffled_correspondences():
    rng = np.random.default_rng(4)
    T_a, T_b, _, pts_a, pts_b = _make_pair(rng, n=200)
    shuffled_b = pts_b[rng.permutation(len(pts_b))]
    kept_a, _ = filter_by_epipolar_consistency(pts_a, shuffled_b, K, T_a, T_b, max_epipolar_error_px=1.0)
    assert len(kept_a) < 0.3 * len(pts_a)


def test_epipolar_filter_catches_bad_pose():
    """The whole point of Stage 3: if the pose stream (hand-eye
    calibration or Aurora data) is wrong, even genuinely correct
    correspondences should fail the known-geometry check, since the
    geometry they're checked against is wrong."""
    rng = np.random.default_rng(5)
    T_a, T_b_true, _, pts_a, pts_b = _make_pair(rng, n=200)
    R_bad, _ = _random_rigid(rng, max_rot=0.3)
    T_b_wrong = to_T(T_b_true[:3, :3] @ R_bad, T_b_true[:3, 3] + np.array([0.05, 0.03, -0.02]))
    kept_a, _ = filter_by_epipolar_consistency(pts_a, pts_b, K, T_a, T_b_wrong, max_epipolar_error_px=3.0)
    assert len(kept_a) < 0.5 * len(pts_a)


def test_sparse_sanity_check_end_to_end_with_real_sift_on_textured_plane():
    """Unlike the other tests here, this feeds sparse_sanity_check actual
    images and runs real SIFT detection/matching -- not hand-fed pixel
    coordinates -- by synthesizing a textured planar scene and warping it
    between two known camera poses via the pose-induced homography. This
    is the one test that can meaningfully stand in for "does the whole
    pipeline work on real-looking image content", and is the baseline to
    compare a real-bag failure against: if this passes but a real capture
    doesn't, the problem is the scene's texture or the pose stream, not
    this module's logic."""
    rng = np.random.default_rng(7)
    size = (480, 480)
    noise = rng.integers(0, 256, size, dtype=np.uint8)
    img_a_gray = cv2.GaussianBlur(noise, (3, 3), 0.8)

    T_a = to_T(np.eye(3), np.zeros(3))
    R_b, _ = cv2.Rodrigues(rng.uniform(-0.05, 0.05, 3))
    T_b = to_T(R_b, np.array([0.003, 0.001, 0.0]))

    Z0 = 0.02  # metres -- planar scene depth
    plane_normal = np.array([0, 0, 1.0])
    T_camA_to_camB = np.linalg.inv(T_b) @ T_a
    R_rel, t_rel = T_camA_to_camB[:3, :3], T_camA_to_camB[:3, 3]
    H = K @ (R_rel + np.outer(t_rel, plane_normal) / Z0) @ np.linalg.inv(K)
    img_b_gray = cv2.warpPerspective(img_a_gray, H, size)

    kept_frames = [
        (0.0, cv2.cvtColor(img_a_gray, cv2.COLOR_GRAY2BGR), T_a, 0.01),
        (0.1, cv2.cvtColor(img_b_gray, cv2.COLOR_GRAY2BGR), T_b, 0.01),
    ]
    result = sparse_sanity_check(kept_frames, K, pair_stride=1, ratio=0.75, max_epipolar_error_px=3.0)
    assert result['pair_count'] == 1
    assert result['successful_pair_count'] == 1
    assert result['point_count'] > 100
    assert result['reproj_error_median_px'] < 1.0


def test_scale_plausibility_flags_order_of_magnitude_errors():
    rng = np.random.default_rng(6)
    points = rng.uniform(-0.0125, 0.0125, (100, 3))  # ~25mm characteristic scale
    good = scale_plausibility(points, expected_diameter_mm=25.0, tolerance_factor=5.0)
    assert good['plausible']

    bad = scale_plausibility(points * 100, expected_diameter_mm=25.0, tolerance_factor=5.0)
    assert not bad['plausible']
