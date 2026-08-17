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
from inchiscope_reconstruction.frame_selection import pose_delta
from inchiscope_reconstruction.sanity_check import (
    filter_by_epipolar_consistency, triangulate, reprojection_error, scale_plausibility,
    sparse_sanity_check, detect_sift, diagnose_pair, filter_outlier_points,
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


def test_diagnose_pair_reports_actual_pose_baseline():
    """baseline_m/rotation_deg must reflect the ACTUAL relative motion
    between the pair's known poses (matching frame_selection.pose_delta),
    independent of --pair-stride or match/point outcomes -- this is what
    lets the CLI tell a near-degenerate-baseline problem apart from a bad
    hand-eye/pose stream when matches fail downstream."""
    rng = np.random.default_rng(9)
    T_a, T_b, _, _, _ = _make_pair(rng)
    expected_baseline_m, expected_rotation_deg = pose_delta(T_a, T_b)

    blank = np.zeros((50, 50, 3), dtype=np.uint8)
    diag = diagnose_pair(blank, blank, K, T_a, T_b)
    assert diag['baseline_m'] == pytest.approx(expected_baseline_m)
    assert diag['rotation_deg'] == pytest.approx(expected_rotation_deg)


def test_clahe_recovers_keypoints_from_low_contrast_real_structure():
    """A real bag run found only ~13 SIFT keypoints/image on real footage
    (vs. ~6000 on the synthetic textured-plane test) -- consistent with
    real edge structure (visible fold boundaries) compressed into low
    local contrast starving SIFT of usable keypoints. CLAHE should
    recover keypoints from genuinely-present-but-faint structure; it
    should NOT invent keypoints from pure noise with no real structure."""
    rng = np.random.default_rng(8)
    size = (300, 300)
    yy, xx = np.mgrid[0:300, 0:300].astype(np.float64)
    curve = 150 + 60 * np.sin(yy / 60.0)
    structure = np.exp(-((xx - curve) ** 2) / (2 * 12 ** 2))
    structure /= structure.max()
    noise = rng.normal(0, 0.02, size)

    low_contrast_img = (110 + (structure + noise) * 15).clip(0, 255).astype(np.uint8)
    low_contrast_bgr = np.stack([low_contrast_img] * 3, axis=-1)
    kp_plain, _ = detect_sift(low_contrast_bgr, use_clahe=False)
    kp_clahe, _ = detect_sift(low_contrast_bgr, use_clahe=True, clahe_clip_limit=4.0, clahe_tile_size=16)
    assert len(kp_clahe) > len(kp_plain)

    flat_noise = (128 + rng.normal(0, 1.0, size)).clip(0, 255).astype(np.uint8)
    flat_noise_bgr = np.stack([flat_noise] * 3, axis=-1)
    kp_flat_clahe, _ = detect_sift(flat_noise_bgr, use_clahe=True, clahe_clip_limit=4.0, clahe_tile_size=16)
    assert len(kp_flat_clahe) == 0


def test_filter_outlier_points_fixes_a_skewed_plausibility_estimate():
    """A handful of high-reprojection-error points (e.g. from a mismatch
    that coincidentally satisfied the epipolar line, or a near-degenerate
    local triangulation) can pull even a percentile-based extent estimate
    far from the truth -- filtering by reprojection error first should
    recover a plausible estimate that filtering by percentile alone
    doesn't fully fix."""
    rng = np.random.default_rng(10)
    good_points = rng.uniform(-0.0125, 0.0125, (100, 3))  # ~25mm characteristic scale
    good_errors = rng.uniform(0.1, 1.0, 100)  # sub-pixel-to-1px, all trustworthy

    # A small number of wild outliers with large reprojection error, at a
    # scale an order of magnitude off -- enough to still shift even the
    # 5th-95th percentile extent noticeably with only 100 good points.
    bad_points = rng.uniform(-0.5, 0.5, (15, 3))
    bad_errors = rng.uniform(5.0, 50.0, 15)

    points = np.concatenate([good_points, bad_points])
    errors = np.concatenate([good_errors, bad_errors])

    unfiltered = scale_plausibility(points, expected_diameter_mm=25.0, tolerance_factor=5.0)
    assert not unfiltered['plausible'], 'expected the outliers to already skew the unfiltered estimate for this test to be meaningful'

    filtered_points, kept, dropped = filter_outlier_points(points, errors, max_reproj_error_px=2.0)
    assert kept == 100
    assert dropped == 15
    filtered = scale_plausibility(filtered_points, expected_diameter_mm=25.0, tolerance_factor=5.0)
    assert filtered['plausible']


def test_scale_plausibility_flags_order_of_magnitude_errors():
    rng = np.random.default_rng(6)
    points = rng.uniform(-0.0125, 0.0125, (100, 3))  # ~25mm characteristic scale
    good = scale_plausibility(points, expected_diameter_mm=25.0, tolerance_factor=5.0)
    assert good['plausible']

    bad = scale_plausibility(points * 100, expected_diameter_mm=25.0, tolerance_factor=5.0)
    assert not bad['plausible']
