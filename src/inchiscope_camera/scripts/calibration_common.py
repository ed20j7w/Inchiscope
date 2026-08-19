"""Corner-detection + multi-directory frame gathering shared by
calibrate_camera.py (standard/pinhole lenses) and
calibrate_camera_fisheye.py (>=~90-100deg lenses) -- kept in one place so
the two scripts can't drift apart on how corners are actually found, or on
how multiple frame directories get combined into one calibration.
"""

import glob
import os
import sys

import cv2
import numpy as np


def find_corners(gray, corners_x, corners_y):
    """Tries the robust SB detector first (handles the uneven ring-light
    illumination typical of an endoscope's onboard LED better than the
    classic detector), falls back to the classic one."""
    size = (corners_x, corners_y)
    found, corners = cv2.findChessboardCornersSB(gray, size)
    if found:
        return found, corners
    found, corners = cv2.findChessboardCorners(
        gray, size,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK,
    )
    if found:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
    return found, corners


def gather_calibration_views(frames_dirs, corners_x, corners_y, square_size_mm,
                              object_point_dtype):
    """Detects the checkerboard in every *.png across one or more
    directories and returns (objpoints, imgpoints, frame_paths, image_size,
    skipped), ready for cv2.calibrateCamera or cv2.fisheye.calibrate.

    Combining a dedicated intrinsics capture session with hand-eye capture
    sessions here is valid and often an improvement -- more views, and
    likely more of the distance/pose diversity a hand-eye session already
    needs for its own reasons -- as long as every directory used the same
    physical square size. Scaling every object point by a wrong-but-
    constant factor wouldn't even show up as a worse fit here: intrinsics
    (K, distortion) come out identical either way, since calibrateCamera/
    fisheye.calibrate are invariant to a uniform rescaling of the object
    points (it only rescales the recovered per-view translations, which
    this function doesn't return or use) -- square_size_mm only matters
    once you're past intrinsics, e.g. calibrate_hand_eye.py's metric
    translation output.

    object_point_dtype must be float32 for cv2.calibrateCamera (it rejects
    float64 outright) and float64 for cv2.fisheye.calibrate (validated by
    hand against a real OpenCV 4.9.0 build -- see
    calibrate_camera_fisheye.py's docstring for why this repo's
    dev-sandbox build can't run that function at all).
    """
    base_objp = np.zeros((corners_x * corners_y, 1, 3), object_point_dtype)
    base_objp[:, 0, :2] = np.mgrid[0:corners_x, 0:corners_y].T.reshape(-1, 2)
    base_objp *= square_size_mm

    objpoints, imgpoints, frame_paths = [], [], []
    image_size = None
    skipped = 0
    for frames_dir in frames_dirs:
        paths = sorted(glob.glob(os.path.join(frames_dir, '*.png')))
        if not paths:
            print(f'No .png frames found in {frames_dir}', file=sys.stderr)
            continue
        for path in paths:
            img = cv2.imread(path)
            if img is None:
                print(f'  could not read {path}, skipping')
                skipped += 1
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if image_size is None:
                image_size = (gray.shape[1], gray.shape[0])
            elif (gray.shape[1], gray.shape[0]) != image_size:
                print(f'  {path} is {gray.shape[1]}x{gray.shape[0]}, expected '
                      f'{image_size[0]}x{image_size[1]} -- skipping (every '
                      f'source must share the same resolution/crop)')
                skipped += 1
                continue
            found, corners = find_corners(gray, corners_x, corners_y)
            if not found:
                print(f'  no board found in {path}, skipping')
                skipped += 1
                continue
            objpoints.append(base_objp.copy())
            imgpoints.append(corners.astype(object_point_dtype))
            frame_paths.append(path)
    return objpoints, imgpoints, frame_paths, image_size, skipped
