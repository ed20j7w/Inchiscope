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

from inchiscope_reconstruction.frame_selection import pose_delta


def enhance_contrast(gray_image, clip_limit=4.0, tile_grid_size=8):
    """CLAHE (contrast-limited adaptive histogram equalization) --
    standard fix for exactly the case this stage hit on real footage:
    smooth, evenly-lit, low local-contrast content (e.g. mucosa) starves
    SIFT of usable keypoints even where real edge structure exists,
    because it's compressed into a narrow intensity band. CLAHE stretches
    local contrast per-tile rather than globally, so it can reveal real
    structure that's genuinely present but poorly quantized -- it cannot
    invent structure out of pure sensor noise/flat content, so this isn't
    a fix for truly featureless input, only for real-but-faint structure
    (validated against both cases with synthetic data before adding this)."""
    clahe = cv2.createCLAHE(clipLimit=clip_limit, tileGridSize=(tile_grid_size, tile_grid_size))
    return clahe.apply(gray_image)


def detect_sift(img, use_clahe=False, clahe_clip_limit=4.0, clahe_tile_size=8):
    """Returns (keypoints, descriptors) -- split out from match_pair so
    keypoint COUNTS (is SIFT finding anything at all in this content?) can
    be inspected independently of whether any of them go on to match."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
    if use_clahe:
        gray = enhance_contrast(gray, clahe_clip_limit, clahe_tile_size)
    sift = cv2.SIFT_create()
    return sift.detectAndCompute(gray, None)


def match_descriptors(kp_a, desc_a, kp_b, desc_b, ratio=0.75):
    """knn-match with Lowe's ratio test. Returns (pts_a, pts_b): Nx2
    pixel-coordinate arrays of matched keypoints, same order, same
    length."""
    if desc_a is None or desc_b is None or len(kp_a) < 2 or len(kp_b) < 2:
        return np.empty((0, 2)), np.empty((0, 2))
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    knn = matcher.knnMatch(desc_a, desc_b, k=2)
    good = [m for m, n in knn if m.distance < ratio * n.distance]
    pts_a = np.array([kp_a[m.queryIdx].pt for m in good]) if good else np.empty((0, 2))
    pts_b = np.array([kp_b[m.trainIdx].pt for m in good]) if good else np.empty((0, 2))
    return pts_a, pts_b


def match_pair(img_a, img_b, ratio=0.75, use_clahe=False, clahe_clip_limit=4.0, clahe_tile_size=8):
    """SIFT detect + match in one call -- convenience wrapper combining
    detect_sift() + match_descriptors()."""
    kp_a, desc_a = detect_sift(img_a, use_clahe, clahe_clip_limit, clahe_tile_size)
    kp_b, desc_b = detect_sift(img_b, use_clahe, clahe_clip_limit, clahe_tile_size)
    return match_descriptors(kp_a, desc_a, kp_b, desc_b, ratio)


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


def diagnose_pair(img_a, img_b, K, T_a, T_b, ratio=0.75, max_epipolar_error_px=3.0,
                   use_clahe=False, clahe_clip_limit=4.0, clahe_tile_size=8):
    """Runs one frame pair through every stage (SIFT detect -> ratio-test
    match -> epipolar-consistency filter -> triangulate -> cheirality),
    recording the surviving count at each step regardless of where (or
    whether) it ultimately succeeds. This is what lets a failure be
    pinned to a specific stage (no keypoints at all? matches found but
    failing the known-pose epipolar check? cheirality?) instead of just
    reporting "nothing survived"."""
    baseline_m, rotation_deg = pose_delta(T_a, T_b)
    kp_a, desc_a = detect_sift(img_a, use_clahe, clahe_clip_limit, clahe_tile_size)
    kp_b, desc_b = detect_sift(img_b, use_clahe, clahe_clip_limit, clahe_tile_size)
    diag = {
        'baseline_m': baseline_m,
        'rotation_deg': rotation_deg,
        'kp_a': len(kp_a) if kp_a is not None else 0,
        'kp_b': len(kp_b) if kp_b is not None else 0,
        'ratio_matches': 0,
        'epipolar_matches': 0,
        'triangulated_points': 0,
        'points': np.empty((0, 3)),
        'reproj_errors': np.empty((0,)),
    }
    pts_a, pts_b = match_descriptors(kp_a, desc_a, kp_b, desc_b, ratio)
    diag['ratio_matches'] = len(pts_a)
    if len(pts_a) == 0:
        return diag

    pts_a, pts_b = filter_by_epipolar_consistency(pts_a, pts_b, K, T_a, T_b, max_epipolar_error_px)
    diag['epipolar_matches'] = len(pts_a)
    if len(pts_a) == 0:
        return diag

    pts3d, valid = triangulate(pts_a, pts_b, K, T_a, T_b)
    pts3d, pts_a, pts_b = pts3d[valid], pts_a[valid], pts_b[valid]
    diag['triangulated_points'] = len(pts3d)
    if len(pts3d) == 0:
        return diag

    err_a = reprojection_error(pts3d, pts_a, K, T_a)
    err_b = reprojection_error(pts3d, pts_b, K, T_b)
    diag['points'] = pts3d
    # per-point, aligned 1:1 with 'points' -- worst of the two views, since
    # a point should reproject well into BOTH to be trustworthy
    diag['point_reproj_errors'] = np.maximum(err_a, err_b)
    diag['reproj_errors'] = np.concatenate([err_a, err_b])  # pooled, for aggregate mean/median/p90 only
    return diag


def sparse_sanity_check(kept_frames, K, pair_stride=5, ratio=0.75, max_epipolar_error_px=3.0,
                         use_clahe=False, clahe_clip_limit=4.0, clahe_tile_size=8):
    """kept_frames: [(t_sec, image, T_ref2cam, pose_age_sec), ...] from
    Stage 2 (frame_selection.select_frames). Matches + triangulates over
    pairs pair_stride apart (wider baseline than adjacent frames, which
    Stage 2's redundancy filter already keeps close to the minimum useful
    separation), pools all triangulated points across pairs.

    use_clahe applies contrast-limited adaptive histogram equalization
    before SIFT detection -- see enhance_contrast()'s docstring for why
    this matters for low-local-contrast content like mucosa.

    Always returns a dict (never None, even if nothing triangulated) with
    pair_diagnostics -- one diagnose_pair() result per attempted pair, so
    a failure can be pinned to a specific stage -- plus point_count,
    pooled points, and reprojection error stats aggregated over whatever
    did survive (empty/zero if nothing did).
    """
    pair_diagnostics = []
    for i in range(0, len(kept_frames) - pair_stride, pair_stride):
        _, img_a, T_a, _ = kept_frames[i]
        _, img_b, T_b, _ = kept_frames[i + pair_stride]
        pair_diagnostics.append(diagnose_pair(
            img_a, img_b, K, T_a, T_b, ratio, max_epipolar_error_px,
            use_clahe, clahe_clip_limit, clahe_tile_size,
        ))

    all_points = [d['points'] for d in pair_diagnostics if len(d['points']) > 0]
    all_point_reproj_errors = [d['point_reproj_errors'] for d in pair_diagnostics if len(d['points']) > 0]
    all_reproj_errors = [d['reproj_errors'] for d in pair_diagnostics if len(d['reproj_errors']) > 0]

    points = np.concatenate(all_points, axis=0) if all_points else np.empty((0, 3))
    point_reproj_errors = np.concatenate(all_point_reproj_errors, axis=0) if all_point_reproj_errors else np.empty((0,))
    reproj_errors = np.concatenate(all_reproj_errors, axis=0) if all_reproj_errors else np.empty((0,))
    return {
        'pair_count': len(pair_diagnostics),
        'successful_pair_count': len(all_points),
        'point_count': len(points),
        'pair_diagnostics': pair_diagnostics,
        'points': points,
        'point_reproj_errors': point_reproj_errors,  # aligned 1:1 with 'points'
        'reproj_error_mean_px': float(reproj_errors.mean()) if len(reproj_errors) else None,
        'reproj_error_median_px': float(np.median(reproj_errors)) if len(reproj_errors) else None,
        'reproj_error_p90_px': float(np.percentile(reproj_errors, 90)) if len(reproj_errors) else None,
    }


def filter_outlier_points(points, point_reproj_errors, max_reproj_error_px=2.0):
    """Drops points whose reprojection error into either view exceeds
    max_reproj_error_px. A point that doesn't reproject well is either a
    mismatch that survived the epipolar filter by coincidence (the
    epipolar check only constrains a point to lie near a LINE, which
    repetitive/self-similar texture can satisfy by chance -- full
    triangulation + reprojection is a much stricter, full check), or
    numerically unstable from a near-degenerate *local* triangulation
    (a specific pair can have a small effective baseline even if the
    overall pair-baseline distribution looks healthy). Either way, a
    handful of these can skew even a percentile-based extent estimate.

    Returns (filtered_points, kept_count, dropped_count).
    """
    if len(points) == 0:
        return points, 0, 0
    keep = point_reproj_errors <= max_reproj_error_px
    return points[keep], int(keep.sum()), int((~keep).sum())


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
