"""Stage 2 (frame selection) of the reconstruction pipeline: drops blurry
and near-duplicate frames from Stage 1's output, undistorts what's kept.

Pure OpenCV/numpy -- no ROS dependency, so it's directly unit-testable
with synthetic images (see the test suite).
"""

import cv2
import numpy as np


def blur_score(gray_image):
    """Variance of the Laplacian -- higher is sharper. Standard,
    inexpensive blur metric; the right threshold depends on scene
    content/resolution, so it's a tunable rather than a fixed constant."""
    return cv2.Laplacian(gray_image, cv2.CV_64F).var()


def blur_scores(frames):
    """blur_score() for every frame in a Stage 1 output list, with no
    threshold applied -- lets you inspect the real distribution before
    choosing --blur-threshold. A sensible cutoff depends heavily on scene
    content: low-texture, smooth, evenly-lit surfaces (e.g. mucosa) read
    as much "blurrier" by this metric than a textured scene even in
    perfect focus, so there's no universal good default -- judge relative
    to your own footage's distribution, not an absolute number."""
    scores = np.empty(len(frames))
    for i, (_, img, _, _) in enumerate(frames):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        scores[i] = blur_score(gray)
    return scores


def pose_delta(T_a, T_b):
    """Returns (translation_delta_m, rotation_delta_deg) between two
    camera poses in the same frame -- used to decide whether a new
    frame's viewpoint has moved enough from the last *kept* frame to be
    worth keeping."""
    delta_t = float(np.linalg.norm(T_b[:3, 3] - T_a[:3, 3]))
    R_rel = T_a[:3, :3].T @ T_b[:3, :3]
    rvec, _ = cv2.Rodrigues(R_rel)
    return delta_t, float(np.degrees(np.linalg.norm(rvec)))


def select_frames(frames, K, D, blur_threshold=100.0,
                   min_baseline_m=0.001, min_rotation_deg=2.0):
    """frames: [(t_sec, image, T_ref2cam, pose_age_sec), ...] from Stage 1
    (bag_extraction.apply_hand_eye). The first frame is always kept (there
    is no "last kept frame" yet to compare against) if it passes the blur
    check.

    Returns (kept, dropped_blur, dropped_redundant) where
    kept = [(t_sec, undistorted_image, T_ref2cam, pose_age_sec), ...].
    """
    kept = []
    last_T = None
    dropped_blur = 0
    dropped_redundant = 0
    for t_sec, img, T_ref2cam, pose_age in frames:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img
        if blur_score(gray) < blur_threshold:
            dropped_blur += 1
            continue
        if last_T is not None:
            dt, dtheta = pose_delta(last_T, T_ref2cam)
            if dt < min_baseline_m and dtheta < min_rotation_deg:
                dropped_redundant += 1
                continue
        undistorted = cv2.undistort(img, K, D)
        kept.append((t_sec, undistorted, T_ref2cam, pose_age))
        last_T = T_ref2cam
    return kept, dropped_blur, dropped_redundant
