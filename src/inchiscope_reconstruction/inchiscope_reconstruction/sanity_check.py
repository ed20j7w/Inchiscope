"""Stage 3 (sparse sanity check) of the reconstruction pipeline.

Runs cheap SIFT-based sparse triangulation using the KNOWN, fixed camera
poses from Stage 1/2 -- poses are never solved for here, only used --
purely to catch a bad hand-eye calibration or corrupted pose stream
before paying for COLMAP's dense stereo.

Pure OpenCV/numpy -- no ROS dependency, so the projection/triangulation
math is directly unit-testable with synthetic data (see the test suite);
the one thing that can't be meaningfully synthetic-tested is whether SIFT
finds good real-world matches, which is inherently about real image
content.
"""

import cv2
import numpy as np


def match_pair(img_a, img_b, ratio=0.75):
    """SIFT detect + knn-match with Lowe's ratio test. Returns (pts_a,
    pts_b): Nx2 pixel-coordinate arrays of matched keypoints, same order,
    same length."""
    gray_a = cv2.cvtColor(img_a, cv2.COLOR_BGR2GRAY) if img_a.ndim == 3 else img_a
    gray_b = cv2.cvtColor(img_b, cv2.COLOR_BGR2GRAY) if img_b.ndim == 3 else img_b
    sift = cv2.SIFT_create()
    kp_a, desc_a = sift.detectAndCompute(gray_a, None)
    kp_b, desc_b = sift.detectAndCompute(gray_b, None)
    if desc_a is None or desc_b is None or len(kp_a) < 2 or len(kp_b) < 2:
        return np.empty((0, 2)), np.empty((0, 2))
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    knn = matcher.knnMatch(desc_a, desc_b, k=2)
    good = [m for m, n in knn if m.distance < ratio * n.distance]
    pts_a = np.array([kp_a[m.queryIdx].pt for m in good]) if good else np.empty((0, 2))
    pts_b = np.array([kp_b[m.trainIdx].pt for m in good]) if good else np.empty((0, 2))
    return pts_a, pts_b


def _fundamental_from_known_poses(K, T_a, T_b):
    """T_a/T_b are camera-pose-in-reference-frame (T_ref2cam, same
    convention as bag_extraction.py). Returns the fundamental matrix
    relating the two views, computed directly from the KNOWN poses -- not
    estimated from the matches themselves, which is what makes this a
    genuine test of pose correctness rather than a self-fulfilling
    consistency check.

    The classic E = [t]_x R / x_b^T E x_a = 0 formulation needs (R, t)
    that maps a point's CAMERA-A-frame coordinates into CAMERA-B-frame
    coordinates (T_camA_to_camB), which is inv(T_b) @ T_a -- NOT
    inv(T_a) @ T_b (that gives the opposite direction, T_camB_to_camA).
    Verified against a synthetic pair with known-correct correspondences:
    only inv(T_b) @ T_a drives the epipolar residual to ~1e-18; the
    swapped order left a residual an order of magnitude too large to pass
    even a generous pixel threshold."""
    T_rel = np.linalg.inv(T_b) @ T_a
    R_rel = T_rel[:3, :3]
    t_rel = T_rel[:3, 3]
    t_cross = np.array([
        [0, -t_rel[2], t_rel[1]],
        [t_rel[2], 0, -t_rel[0]],
        [-t_rel[1], t_rel[0], 0],
    ])
    E = t_cross @ R_rel
    K_inv = np.linalg.inv(K)
    return K_inv.T @ E @ K_inv


def filter_by_epipolar_consistency(pts_a, pts_b, K, T_a, T_b, max_epipolar_error_px=3.0):
    """Discards matches whose epipolar error (against the fundamental
    matrix computed from the KNOWN poses) exceeds max_epipolar_error_px.
    Catches mismatched correspondences, and -- more importantly for this
    stage's purpose -- catches a bad hand-eye/pose stream: if the poses
    are wrong, even genuinely correct SIFT matches will fail this known-
    geometry check, since the geometry it's checked against is wrong."""
    if len(pts_a) == 0:
        return pts_a, pts_b
    F = _fundamental_from_known_poses(K, T_a, T_b)
    pts_a_h = np.hstack([pts_a, np.ones((len(pts_a), 1))])
    pts_b_h = np.hstack([pts_b, np.ones((len(pts_b), 1))])
    lines = pts_a_h @ F.T  # epipolar line in image B for each point in A
    norms = np.hypot(lines[:, 0], lines[:, 1])
    norms = np.where(norms < 1e-12, np.inf, norms)
    dist = np.abs(np.sum(pts_b_h * lines, axis=1)) / norms
    keep = dist <= max_epipolar_error_px
    return pts_a[keep], pts_b[keep]


def triangulate(pts_a, pts_b, K, T_a, T_b):
    """Linear DLT triangulation (cv2.triangulatePoints) given KNOWN
    camera poses. Returns (pts3d, valid): Nx3 points in the reference
    frame, and a boolean cheirality mask (True where the point is in
    front of BOTH cameras)."""
    P_a = K @ np.linalg.inv(T_a)[:3, :]
    P_b = K @ np.linalg.inv(T_b)[:3, :]
    if len(pts_a) == 0:
        return np.empty((0, 3)), np.empty((0,), dtype=bool)
    pts4d = cv2.triangulatePoints(P_a, P_b, pts_a.T, pts_b.T)
    pts3d = (pts4d[:3] / pts4d[3]).T

    pts3d_h = np.hstack([pts3d, np.ones((len(pts3d), 1))])
    depth_a = (np.linalg.inv(T_a) @ pts3d_h.T)[2]
    depth_b = (np.linalg.inv(T_b) @ pts3d_h.T)[2]
    valid = (depth_a > 0) & (depth_b > 0)
    return pts3d, valid


def reprojection_error(pts3d, pts2d, K, T):
    """Per-point reprojection error (px) of 3D points into camera T."""
    if len(pts3d) == 0:
        return np.empty((0,))
    pts3d_h = np.hstack([pts3d, np.ones((len(pts3d), 1))])
    pts_cam = (np.linalg.inv(T) @ pts3d_h.T)[:3].T
    proj = (K @ pts_cam.T).T
    proj_px = proj[:, :2] / proj[:, 2:3]
    return np.linalg.norm(proj_px - pts2d, axis=1)


def sparse_sanity_check(kept_frames, K, pair_stride=5, ratio=0.75, max_epipolar_error_px=3.0):
    """kept_frames: [(t_sec, image, T_ref2cam, pose_age_sec), ...] from
    Stage 2 (frame_selection.select_frames). Matches + triangulates over
    pairs pair_stride apart (wider baseline than adjacent frames, which
    Stage 2's redundancy filter already keeps close to the minimum useful
    separation), pools all triangulated points across pairs.

    Returns None if no pair produced enough surviving points to say
    anything, otherwise a dict with pair_count, point_count, per-pair
    match_counts, the pooled points (Nx3, reference frame, metres), and
    reprojection error stats (px).
    """
    all_points = []
    all_reproj_errors = []
    match_counts = []

    for i in range(0, len(kept_frames) - pair_stride, pair_stride):
        _, img_a, T_a, _ = kept_frames[i]
        _, img_b, T_b, _ = kept_frames[i + pair_stride]

        pts_a, pts_b = match_pair(img_a, img_b, ratio=ratio)
        if len(pts_a) < 8:
            continue
        pts_a, pts_b = filter_by_epipolar_consistency(pts_a, pts_b, K, T_a, T_b, max_epipolar_error_px)
        if len(pts_a) < 8:
            continue
        pts3d, valid = triangulate(pts_a, pts_b, K, T_a, T_b)
        pts3d, pts_a, pts_b = pts3d[valid], pts_a[valid], pts_b[valid]
        if len(pts3d) == 0:
            continue

        err_a = reprojection_error(pts3d, pts_a, K, T_a)
        err_b = reprojection_error(pts3d, pts_b, K, T_b)
        all_points.append(pts3d)
        all_reproj_errors.append(err_a)
        all_reproj_errors.append(err_b)
        match_counts.append(len(pts3d))

    if not all_points:
        return None

    points = np.concatenate(all_points, axis=0)
    reproj_errors = np.concatenate(all_reproj_errors, axis=0)
    return {
        'pair_count': len(match_counts),
        'point_count': len(points),
        'match_counts': match_counts,
        'points': points,
        'reproj_error_mean_px': float(reproj_errors.mean()),
        'reproj_error_median_px': float(np.median(reproj_errors)),
        'reproj_error_p90_px': float(np.percentile(reproj_errors, 90)),
    }


def scale_plausibility(points, expected_diameter_mm, tolerance_factor=5.0):
    """Coarse plausibility bound, NOT a precision measurement: uses the
    5th-95th percentile extent along each axis (robust to a handful of
    triangulation outliers) as a proxy for the point cloud's
    characteristic size, and checks the median of those three extents is
    within tolerance_factor of expected_diameter_mm in either direction.
    This is meant to catch order-of-magnitude errors (a badly wrong hand-
    eye scale or corrupted pose stream), not to validate the true lumen
    diameter precisely."""
    p5 = np.percentile(points, 5, axis=0)
    p95 = np.percentile(points, 95, axis=0)
    extent_mm = (p95 - p5) * 1000.0
    characteristic_size_mm = float(np.median(extent_mm))
    lo = expected_diameter_mm / tolerance_factor
    hi = expected_diameter_mm * tolerance_factor
    return {
        'extent_mm': extent_mm.tolist(),
        'characteristic_size_mm': characteristic_size_mm,
        'expected_diameter_mm': expected_diameter_mm,
        'bounds_mm': (lo, hi),
        'plausible': lo <= characteristic_size_mm <= hi,
    }
